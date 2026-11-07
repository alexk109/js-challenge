"""Self-tests for the pin-level JTAG reference models: `make vip-selftest VIP=jtag`.

Everything runs at 50 MHz (20 ns per tick) against the `ijet` timing profile, with a
`JtagMaster` driving a `JtagTarget` and a passive `JtagChecker` watching the pins.
`rig()` builds that bench; the tests then look at what came off TDO, at what the
target latched, and at what the checker says about the wire.

The suite has three jobs:

  * the models talk to each other -- reset, IR scans, IDCODE, BYPASS, a user register;
  * the checker passes legal traffic;
  * every one of the checker's 15 rules has a test that breaks exactly that rule, and
    the checker is shown not to be vacuous.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.jtag import (
    CHECKER_RULES,
    CAPTURE_IR,
    DEFAULT_IDCODE,
    EXIT1_DR,
    EXIT2_DR,
    EXIT2_IR,
    PAUSE_DR,
    PAUSE_IR,
    DEFAULT_DR_LENGTHS,
    IJET_TIMINGS,
    IR_CAPTURE_LSB_PATTERN,
    RESET_CLOCKS,
    RUN_TEST_IDLE,
    SELECT_DR_SCAN,
    SELECT_IR_SCAN,
    SHIFT_DR,
    SHIFT_IR,
    TEST_LOGIC_RESET,
    UPDATE_DR,
    UPDATE_IR,
    JtagChecker,
    JtagMaster,
    JtagTarget,
    attach_checker,
    attach_master,
    attach_target,
    idcode_fields,
    jtag_bus,
    jtag_timings,
    mask_opcode,
    next_state,
    scan_bits,
    scan_value,
    TAP_TRANSITIONS,
)

NS_PER_TICK = 20.0            # 50 MHz chip clock
LIMIT = 200_000               # generous tick limit for every run


def rig(ir_len=8, master=None, target=None, checker=None, check=True, **kw):
    """A bus with a JtagMaster, a JtagTarget and (by default) a JtagChecker.

    `master`, `target` and `checker` are kwargs dicts merged over the defaults, so a
    test can break one part of the bench without touching the others. `check=False`
    leaves the checker off, which the "models only" tests use.
    """
    bus = jtag_bus(ns_per_tick=NS_PER_TICK)
    m = attach_master(bus, **({"ir_len": ir_len} | (master or {})))
    tap = attach_target(bus, **({"ir_len": ir_len} | (target or {})))
    chk = (attach_checker(bus, **({"ir_len": ir_len} | (checker or {}))) if check else None)
    return bus, m, tap, chk


def run(master, bus=None, limit=LIMIT):
    """Run the bus until the master's queue is empty, and say so if it never is."""
    ticks = master.run(bus) if bus is None else master.run(bus, limit=limit)
    assert not master.busy, f"the master still has {master.pending()} scans queued"
    return ticks


def rules(checker) -> set:
    """The set of rules the checker flagged."""
    return {v.rule for v in checker.violations}


def only(checker, rule: str) -> None:
    """Assert `rule` was flagged, with a tick and a measurement that make sense."""
    hit = checker.violations_by_rule(rule)
    assert hit, f"{rule} was not flagged; got {rules(checker) or 'no violations'}"
    for v in hit:
        assert v.tick > 0, f"{rule} has no tick"
        assert v.sense in ("eq", "event") or v.measured != v.required, v
        assert v.detail, f"{rule} carries no explanation"


def test_master_and_target_agree_on_a_reset_and_an_idcode_read():
    """The first thing a master does: reset, then read the 32-bit IDCODE."""
    bus, m, tap, chk = rig()
    m.reset()
    idcode = m.scan_dr(0xFFFFFFFF)
    run(m, bus)

    assert idcode.value == DEFAULT_IDCODE == tap.idcode
    assert idcode.sent == 0xFFFFFFFF and len(idcode.out_bits) == 32
    assert tap.instruction == tap.opcode("IDCODE"), "a reset selects IDCODE"
    assert tap.state == RUN_TEST_IDLE and m.state == RUN_TEST_IDLE
    assert chk.clean, chk.report()

    # ... and the same IDCODE is what the checker decoded off the wire
    dr = [s for s in chk.scans if s.kind == "dr"]
    assert len(dr) == 1 and dr[0].value == DEFAULT_IDCODE and dr[0].length == 32
    assert dr[0].sent == 0xFFFFFFFF, "the checker saw the bits the master drove"
    assert tap.last_idcode == DEFAULT_IDCODE


def test_the_instruction_register_captures_the_fixed_pattern_and_the_sent_opcode():
    """Capture-IR loads 0b01 in its two LSBs; an IR scan returns that pattern."""
    bus, m, tap, chk = rig()
    m.reset()
    ir = m.scan_ir(tap.opcode("IDCODE"))
    ir2 = m.scan_ir(tap.opcode("SCRATCH"))
    run(m, bus)

    assert ir.value == IR_CAPTURE_LSB_PATTERN == 0b01, "bit 0 first off TDO"
    assert ir.out_bits[:2] == [1, 0], "the two LSBs read 0, 1"
    assert ir.sent == tap.opcode("IDCODE") == 0x02
    assert ir2.sent == tap.opcode("SCRATCH") == 0xE0
    assert [v for _t, v in tap.ir_reads] == [IR_CAPTURE_LSB_PATTERN] * 2
    assert chk.clean, chk.report()


def test_bypass_is_one_bit_and_recaptures_zero_on_every_scan():
    """BYPASS is a single-bit register, and Capture-DR loads a zero into it.

    So every scan of BYPASS reads 0 -- the bit a scan shifted in comes back only if the
    next clock happens while the TAP is *still* in Shift-DR, which is what the second
    half of this test does, at the cost of the DR length the checker then flags.
    """
    bus, m, tap, chk = rig()
    m.reset()
    m.scan_ir(tap.opcode("BYPASS"))
    scans = [m.scan_dr(0), m.scan_dr(1), m.scan_dr(0xA5)]
    over = m.scan_dr(0b01, length=2)          # two clocks in a 1-bit register
    run(m, bus)

    assert tap.dr_length(tap.opcode("BYPASS")) == 1
    assert [s.value for s in scans] == [0, 0, 0], "a zero is captured every time"
    assert all(len(s.out_bits) == 1 for s in scans)
    assert over.value == 0b10, "the second clock returned the first clock's bit"
    only(chk, "DR.length")                     # ... which is a real violation
    assert chk.violations_by_rule("DR.length")[0].measured == 2


def test_a_user_data_register_keeps_what_a_scan_wrote():
    """SCRATCH is a real register: a write latches at Update-DR and reads back."""
    bus, m, tap, chk = rig()
    m.reset()
    m.scan_ir(tap.opcode("SCRATCH"))
    write = m.scan_dr(0xA5, length=8)
    read = m.scan_dr(0x00, length=8)
    run(m, bus)

    assert tap.updates[-2][1:] == ("SCRATCH", 0xA5), "the write latched at Update-DR"
    assert tap.updates[-1][1:] == ("SCRATCH", 0x00), "and so did the zero of the read"
    assert read.value == 0xA5, "Capture-DR loads the register's own contents"
    assert write.value == 0x00, "SCRATCH starts at zero"
    assert tap.registers["SCRATCH"] == 0x00, "the read scan wrote a zero back"
    assert chk.clean, chk.report()


def test_a_long_mixed_run_is_legal_and_the_checker_stays_silent():
    """Every instruction shape in one run: resets, IR scans, all three DRs, idle."""
    bus, m, tap, chk = rig()
    m.reset()
    id1 = m.scan_dr(0)
    id2 = m.scan_dr(0)
    m.idle(4)
    m.scan_ir(tap.opcode("BYPASS"))
    b1 = m.scan_dr(1)
    b2 = m.scan_dr(0)
    m.scan_ir(tap.opcode("SCRATCH"))
    m.scan_dr(0x3C, length=8)
    m.scan_ir(0x00)                  # an instruction the target does not implement
    unknown = m.scan_dr(0, length=1)  # ... so the master has to be told the length
    m.reset()
    after = m.scan_dr(0)
    ticks = run(m, bus)

    assert id1.value == id2.value == DEFAULT_IDCODE, "IDCODE is stable"
    assert (b1.value, b2.value) == (0, 0), "BYPASS is one bit and captures a zero"
    assert tap.registers["SCRATCH"] == 0x3C
    assert unknown.length == 1, "an unknown opcode selects the 1-bit BYPASS register"
    assert after.value == DEFAULT_IDCODE, "a reset puts IDCODE back"
    assert tap.unknown_instructions and tap.unknown_instructions[-1][1] == 0x00
    assert chk.clean, chk.report()
    assert chk.resets == 1, "the second reset is the one that enters the state"
    assert len(chk.scans) == 10, "3 IR scans and 7 DR scans"
    assert (m.rises, tap.rises, chk.rises) == (chk.rises,) * 3
    assert bus.errors == [], "push-pull: no contention is possible, and none happened"
    assert ticks > 0


def test_scans_can_be_queued_while_the_bus_is_running():
    """`run()` may be called repeatedly, with more scans queued in between."""
    bus, m, tap, chk = rig()
    m.reset()
    m.scan_dr(0)
    run(m, bus)
    assert chk.scans and chk.clean, chk.report()

    m.scan_ir(tap.opcode("SCRATCH"))
    second = m.scan_dr(0x77, length=8)
    run(m, bus)
    assert second.value == 0x00
    assert tap.registers["SCRATCH"] == 0x77
    assert len(chk.scans) == 3, "one IR scan and two DR scans in total"
    assert chk.clean, chk.report()
    assert m.pending_ticks(bus.t) == 0, "nothing is left to drive"

# -- one test per rule ----------------------------------------------------------
# Every one of these drives the *same* legal master, or gives one device a fault, and
# then asks the checker for exactly one rule. The tick and the measurement are checked
# too: a violation with no tick is not evidence.

def test_rule_tck_low():
    """A checker told the target needs a longer TCK low time than the master gives."""
    bus, m, tap, chk = rig(checker={"timing": {"tck_low": 200}})   # 10 ticks, not 3
    m.reset()
    m.scan_dr(0)
    run(m, bus)

    only(chk, "TCK.low")
    v = chk.violations_by_rule("TCK.low")[0]
    assert v.measured == m.low == 3 and v.required == 10
    assert v.tick > m.low, "the miss is measured at the rising edge that ended it"
    assert chk.violations_by_rule("TCK.high") == [], "only the low rule is broken"
    assert tap.last_idcode == DEFAULT_IDCODE, "the data still arrived"


def test_rule_tck_high():
    """The same rule for the high half of the period."""
    bus, m, tap, chk = rig(checker={"timing": {"tck_high": 200}})
    m.reset()
    m.scan_dr(0)
    run(m, bus)

    only(chk, "TCK.high")
    v = chk.violations_by_rule("TCK.high")[0]
    assert v.measured == m.high == 3 and v.required == 10
    assert chk.violations_by_rule("TCK.low") == []
    assert tap.last_idcode == DEFAULT_IDCODE


def test_rule_tck_period():
    """A master clocked at `half=2` runs at 12.5 MHz, over the profile's 10 MHz cap.

    Both half periods still clear every floor -- two ticks is above Ttckl and Ttckh -- so
    only the frequency cap can catch this, which is why it is a rule of its own.
    """
    bus, m, tap, chk = rig(master={"half": 2})
    m.reset()
    idcode = m.scan_dr(0)
    run(m, bus)

    only(chk, "TCK.period")
    assert m.period == 4 and m.max_tck_hz == 12.5e6
    v = chk.violations_by_rule("TCK.period")[0]
    assert v.measured == 4 and v.required == jtag_timings(NS_PER_TICK)["min_period"] == 5
    assert chk.max_tck_hz() == 12.5e6 > chk.min["max_tck_hz"]
    assert "12.50 MHz" in v.detail and "10.00 MHz" in v.detail
    assert chk.violations_by_rule("TCK.low") == [], "the floor itself was met"
    assert chk.violations_by_rule("TCK.high") == []
    assert idcode.value == DEFAULT_IDCODE, "it works, it is just too fast"
    # ... and the same wire passes a checker whose profile allows 12.5 MHz
    bus2, m2, _t2, chk2 = rig(master={"half": 2},
                             checker={"timing": {"max_tck_hz": 12.5e6}})
    m2.reset()
    m2.scan_dr(0)
    run(m2, bus2)
    assert chk2.violations_by_rule("TCK.period") == [], chk2.report()


def test_rule_tdi_setup_and_tms_setup():
    """A master that launches data two ticks after the falling edge is too late.

    The profile allows Tsod = 2.0 ns, which is one tick at 20 ns, and a real adapter
    launches on the falling edge itself (launch_delay 0).
    """
    bus, m, tap, chk = rig(master={"launch_delay": 2})
    m.reset()
    m.scan_ir(tap.opcode("SCRATCH"))          # so TDI really moves on the wire
    dr = m.scan_dr(0xFFFF, length=8)
    run(m, bus)

    only(chk, "TDI.setup")
    only(chk, "TMS.setup")
    v = chk.violations_by_rule("TDI.setup")[0]
    assert v.measured == 2 and v.required == jtag_timings(NS_PER_TICK)["tdi_tms_valid"] == 1
    assert rules(chk) == {"TDI.setup", "TMS.setup"}, "no other rule is broken"
    assert dr.sent == 0xFF, "the bits were still driven, just too late"
    # ... and the same master with the delay inside the budget is clean
    bus2, m2, _t2, chk2 = rig(master={"launch_delay": 1})
    m2.reset()
    m2.scan_dr(0)
    run(m2, bus2)
    assert chk2.clean, chk2.report()


def test_rule_tdi_change_on_high():
    """Data that moves while TCK reads high is sampled at the wrong time."""
    bus, m, tap, chk = rig(master={"launch_delay": 5})   # low is 3 ticks, high starts at 4
    m.reset()
    m.scan_ir(tap.opcode("SCRATCH"))
    m.scan_dr(0xFFFF, length=8)
    run(m, bus)

    only(chk, "TDI.change-on-high")
    v = chk.violations_by_rule("TDI.change-on-high")[0]
    assert v.sense == "event" and v.tick > 0
    assert "rising edge" in v.detail
    assert "TDO.change-on-high" not in rules(chk), "the target still launched on the fall"


def test_rule_tms_change_on_high():
    """TMS decides the next state, so moving it during the high phase is worse still."""
    bus, m, tap, chk = rig(master={"launch_delay": 5})
    m.reset()
    m.scan_ir(tap.opcode("SCRATCH"))
    m.scan_dr(0xFFFF, length=8)
    run(m, bus)

    only(chk, "TMS.change-on-high")
    assert "TDO.change-on-high" not in rules(chk)
    assert chk.clean is False


def test_rule_tdo_change_on_high():
    """A target that updates TDO in the high phase: the master may sample the old bit."""
    bus, m, tap, chk = rig(target={"tdo_delay": 2})     # the low phase is only 3 ticks
    m.reset()
    idcode = m.scan_dr(0)
    run(m, bus)

    only(chk, "TDO.change-on-high")
    only(chk, "TDO.setup")
    assert "TDI.change-on-high" not in rules(chk), "only the target is at fault"
    assert idcode.value == DEFAULT_IDCODE, "the bit still made it, one tick late"


def test_rule_tdo_setup():
    """A target whose next TDO bit does not make it before the rising edge."""
    bus, m, tap, chk = rig(target={"tdo_delay": 2})
    m.reset()
    m.scan_dr(0)
    run(m, bus)

    only(chk, "TDO.setup")
    v = chk.violations_by_rule("TDO.setup")[0]
    assert v.measured < v.required == jtag_timings(NS_PER_TICK)["tdo_valid"]
    assert "TCK low time" in v.detail
    # one tick of delay is still inside the low phase and stays clean
    bus2, m2, _t2, chk2 = rig(target={"tdo_delay": 1})
    m2.reset()
    ok = m2.scan_dr(0)
    run(m2, bus2)
    assert chk2.clean, chk2.report()
    assert ok.value == DEFAULT_IDCODE


def test_rule_tdo_inactive():
    """TDO outside a Shift state must be released: an unselected TAP does not drive it."""
    bus, m, tap, chk = rig(target={"idle_tdo": 0})
    m.reset()
    m.scan_dr(0)
    run(m, bus)

    only(chk, "TDO.inactive")
    v = chk.violations_by_rule("TDO.inactive")[0]
    assert v.measured == 0 and v.required == 1 and v.sense == "eq"
    assert TEST_LOGIC_RESET in v.detail or RUN_TEST_IDLE in v.detail
    # a target that holds TDO high outside a shift is not flagged either
    bus2, m2, _t2, chk2 = rig(target={"idle_tdo": 1})
    m2.reset()
    m2.scan_dr(0)
    run(m2, bus2)
    assert chk2.clean, chk2.report()


def test_rule_ir_capture():
    """A target whose Capture-IR pattern is not 01 in its two LSBs."""
    bus, m, tap, chk = rig(target={"ir_capture": 0b00000011})
    m.reset()
    ir = m.scan_ir(tap.opcode("IDCODE"))
    run(m, bus)

    only(chk, "IR.capture")
    v = chk.violations_by_rule("IR.capture")[0]
    assert v.measured == 0b11 and v.required == IR_CAPTURE_LSB_PATTERN == 0b01
    assert v.sense == "eq" and "0, 1" in v.detail
    assert ir.out_bits[:2] == [1, 1], "the master really did read 11"
    # the rest of the capture pattern is implementer-defined, so this is legal
    bus2, m2, _t2, chk2 = rig(target={"ir_capture": 0b11111101})
    m2.reset()
    m2.scan_ir(0x02)
    run(m2, bus2)
    assert chk2.clean, chk2.report()


def test_rule_ir_length():
    """A master that shifts nine bits into an eight-bit instruction register."""
    bus, m, tap, chk = rig()
    m.reset()
    m.scan_ir(tap.opcode("IDCODE"), length=9)
    run(m, bus)

    only(chk, "IR.length")
    v = chk.violations_by_rule("IR.length")[0]
    assert v.measured == 9 and v.required == 8
    assert v.sense == "eq" and "Shift-IR" in v.detail
    # one bit short is the same mistake the other way round
    bus2, m2, _t2, chk2 = rig()
    m2.reset()
    m2.scan_ir(tap.opcode("IDCODE"), length=7)
    run(m2, bus2)
    only(chk2, "IR.length")
    assert chk2.violations_by_rule("IR.length")[0].measured == 7


def test_rule_idcode_lsb():
    """A target whose IDCODE has a 1 in bit 0, the bypass bit."""
    bus, m, tap, chk = rig(target={"idcode": DEFAULT_IDCODE | 1})
    m.reset()
    idcode = m.scan_dr(0)
    run(m, bus)

    only(chk, "IDCODE.lsb")
    v = chk.violations_by_rule("IDCODE.lsb")[0]
    assert v.measured == 1 and v.required == 0 and v.sense == "eq"
    assert idcode.value == DEFAULT_IDCODE | 1, "the master read the odd value out"
    # real parts do this, so the rule can be waived the way OpenOCD's -ignore-bypass is
    bus2, m2, _t2, chk2 = rig(target={"idcode": DEFAULT_IDCODE | 1},
                             checker={"require_idcode_lsb": False})
    m2.reset()
    m2.scan_dr(0)
    run(m2, bus2)
    assert chk2.clean, chk2.report()


def test_rule_dr_length():
    """A DR scan of the wrong length: too short, and too long."""
    bus, m, tap, chk = rig()
    m.reset()
    short = m.scan_dr(0xFFFF, length=31)
    long = m.scan_dr(0, length=33)
    run(m, bus)

    assert [v.measured for v in chk.violations_by_rule("DR.length")] == [31, 33]
    assert {v.required for v in chk.violations_by_rule("DR.length")} == {32}
    assert "IDCODE is 32 bits" in chk.violations_by_rule("DR.length")[0].detail
    assert short.length == 31 and long.length == 33
    assert tap.last_idcode == DEFAULT_IDCODE, "and the target still latched on Update-DR"


def test_rule_reset_tms_high():
    """Five rising edges with TMS high must land in Test-Logic-Reset.

    The checker decodes with the state table it was given, so this is the rule that
    protects a master which relies on the guarantee: handed a table without it, starting
    from Run-Test/Idle, the checker says so on the fifth edge. A *real* target that
    ignored TMS would be invisible from the pins -- it would simply return the wrong data
    -- which is the honest limit of a passive monitor.
    """
    broken = dict(TAP_TRANSITIONS)
    broken[RUN_TEST_IDLE] = (RUN_TEST_IDLE, PAUSE_IR)      # TMS high no longer resets
    bus, m, tap, chk = rig(checker={"transitions": broken,
                                   "initial_state": RUN_TEST_IDLE})
    m.reset()
    run(m, bus)

    only(chk, "RESET.tms-high")
    v = chk.violations_by_rule("RESET.tms-high")[0]
    assert v.sense == "event" and v.measured == v.required == RESET_CLOCKS == 5
    assert "Test-Logic-Reset" in v.detail
    # a rising edge reaches the bus one tick after the drive that made it
    assert v.tick == (RESET_CLOCKS - 1) * m.period + m.low + 1, "the fifth rising edge"
    assert tap.state == RUN_TEST_IDLE, "the real target did reset, as it should"

    # the real table has the guarantee, and the same traffic against it is clean
    state = RUN_TEST_IDLE
    for _ in range(RESET_CLOCKS):
        state = next_state(state, 1)
    assert state == TEST_LOGIC_RESET
    bus2, m2, _t2, chk2 = rig(checker={"initial_state": RUN_TEST_IDLE})
    m2.reset()
    m2.scan_dr(0)
    run(m2, bus2)
    assert chk2.clean, chk2.report()
    assert chk2.resets == 1, "and it saw the reset happen"


def test_a_target_without_idcode_resets_into_bypass():
    """has_idcode=False: a reset selects BYPASS, so data scans are one bit."""
    bus, m, tap, chk = rig(target={"has_idcode": False}, checker={"has_idcode": False})
    m.reset()
    scan = m.scan_dr(0, length=1)
    run(m, bus)

    assert tap.has_idcode is False
    assert tap.instruction == 0xFF, "BYPASS is all ones whatever the IR width"
    assert "IDCODE" not in tap.name_of(tap.instruction)
    assert scan.length == 1 and scan.value == 0
    assert chk.clean, chk.report()
    assert chk.dr_length(0xFF) == 1 and chk.name_of(0x02) == "BYPASS", "IDCODE is gone"


# -- the checker itself ----------------------------------------------------------

def test_checker_is_not_vacuous():
    """The same pin traffic, judged against a stricter profile, must fail.

    Two things at once: a checker whose limits are one tick above what the wire gives
    flags every one of those rules, and a checker watching the same run reports the very
    scans the master queued -- so a clean report means "looked at the wire and agreed",
    not "never looked".
    """
    strict = {"tck_low": 4 * NS_PER_TICK, "tck_high": 4 * NS_PER_TICK,
              "tdi_tms_valid": 0.0, "tdo_valid": 6 * NS_PER_TICK}
    bus, m, tap, chk = rig()
    m.reset()
    first = m.scan_dr(0)
    m.scan_ir(tap.opcode("SCRATCH"))
    second = m.scan_dr(0x3C, length=8)
    ticks = run(m, bus)
    assert chk.clean, chk.report()
    assert len(chk.scans) == 3 and chk.rises == m.rises
    assert chk.shortest_period == m.period and chk.periods == m.rises - 1

    bus2, m2, tap2, chk2 = rig(checker={"timing": strict})
    m2.reset()
    first2 = m2.scan_dr(0)
    m2.scan_ir(tap2.opcode("SCRATCH"))
    m2.scan_dr(0x3C, length=8)
    run(m2, bus2)

    flagged = rules(chk2)
    assert {"TCK.low", "TCK.high", "TDO.setup"} <= flagged, chk2.report()
    assert not ({"IR.capture", "IR.length", "DR.length", "IDCODE.lsb"} & flagged), \
        "the framing rules do not depend on the timing profile"
    assert chk2.scans == chk.scans, "the same wire decodes to the same scans"
    assert first2.value == first.value and chk2.periods == chk.periods
    assert chk2.violations_by_rule("TCK.low")[0].required == 4 > m.low
    assert ticks > 0


def test_checker_flags_the_whole_run_not_just_the_first_offence():
    """One bad wire feature is reported on every cycle it happens, not once."""
    bus, m, tap, chk = rig(master={"launch_delay": 2})
    m.reset()
    m.scan_ir(tap.opcode("SCRATCH"))
    m.scan_dr(0b01010101, length=8)      # every bit differs from the one before it
    run(m, bus)

    setups = chk.violations_by_rule("TDI.setup")
    assert len(setups) >= 6, "every bit that changes is a late bit"
    assert len({v.tick for v in setups}) == len(setups), "one per tick, all distinct"
    assert all(v.measured == 2 for v in setups)


def test_the_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting: the IAR I-jet table, in ns."""
    ijet = jtag_timings(NS_PER_TICK, "ijet")
    assert IJET_TIMINGS.tck_low == IJET_TIMINGS.tck_high == 15.6     # Ttckl, Ttckh
    assert IJET_TIMINGS.tdi_tms_valid == 2.0                         # Tsod, a maximum
    assert IJET_TIMINGS.tdo_valid == 15.6                            # Ttdo, a minimum
    assert IJET_TIMINGS.max_tck_hz == 10e6, "the low end of 'typically 10-100 MHz'"
    assert "ac-timing-iar-ijet.txt" in ijet["source"]
    # whole ticks at 20 ns, rounded up
    assert (ijet["tck_low"], ijet["tck_high"]) == (1, 1)
    assert ijet["tdi_tms_valid"] == 1 and ijet["tdo_valid"] == 1
    assert ijet["min_period"] == 5 and ijet["max_tck_hz"] == 10e6
    # a slower tick grid makes every floor longer, never shorter
    slow = jtag_timings(100.0, "ijet")
    assert slow["tck_low"] == 1 and slow["tdo_valid"] == 1 and slow["min_period"] == 1
    assert jtag_timings(2.0, "ijet")["tdi_tms_valid"] == 1          # 2 ns is one tick
    assert jtag_timings(2.0, "ijet", {"max_tck_hz": 50e6})["min_period"] == 10
    assert jtag_timings(NS_PER_TICK)["ns"]["tck_low"] == 15.6        # the raw ns survive
    with pytest.raises(ValueError):
        jtag_timings(NS_PER_TICK, "ijet-plus")
    with pytest.raises(ValueError):
        jtag_timings(NS_PER_TICK, "ijet", {"t_nonsense": 1})
    with pytest.raises(ValueError):
        jtag_timings(NS_PER_TICK, "ijet", {"max_tck_hz": 0})


def test_the_state_table_is_the_spec():
    """The 16 states, six stable ones, and five TMS-high clocks to the reset state."""
    assert len(TAP_TRANSITIONS) == 16
    # A state is stable when holding TMS at the level that selects it keeps the TAP
    # there: TMS low everywhere except Test-Logic-Reset, where it is high.
    stable = [st for st, nxt in TAP_TRANSITIONS.items() if st in nxt]
    assert len(stable) == 6, sorted(stable)
    assert set(stable) == {TEST_LOGIC_RESET, RUN_TEST_IDLE, SHIFT_DR, SHIFT_IR,
                           PAUSE_DR, PAUSE_IR}
    assert all(len(nxt) == 2 for nxt in TAP_TRANSITIONS.values())
    for state, (low, high) in TAP_TRANSITIONS.items():
        assert low in TAP_TRANSITIONS and high in TAP_TRANSITIONS
    assert TAP_TRANSITIONS[SELECT_IR_SCAN][1] == TEST_LOGIC_RESET
    assert TAP_TRANSITIONS[SHIFT_DR][1] == EXIT1_DR and TAP_TRANSITIONS[EXIT1_DR][1] == UPDATE_DR
    assert TAP_TRANSITIONS[EXIT2_DR] == (SHIFT_DR, UPDATE_DR)
    assert TAP_TRANSITIONS[EXIT2_IR] == (SHIFT_IR, UPDATE_IR)
    # the worked sequence in tap-state-machine-pbeirne.txt, TMS bit by bit
    tms = "0001100000111000000000001100"
    state = RUN_TEST_IDLE
    seen = []
    for bit in tms:
        seen.append(state)
        state = next_state(state, int(bit))
    assert seen[4:8] == [SELECT_DR_SCAN, SELECT_IR_SCAN, CAPTURE_IR, SHIFT_IR]
    assert seen[-1] == RUN_TEST_IDLE, "the worked sequence ends in Run-Test/Idle"


def test_the_idcode_fields_pack_as_documented():
    """version 31:28, manufacturer 27:12, part 11:1, and bit 0 always 0."""
    value = idcode_fields(manufacturer=0xBA0, part=0x023, version=0x1)
    assert value == DEFAULT_IDCODE == 0x10BA0046
    assert value & 1 == 0, "bit 0 is the bypass bit and is never part of the fields"
    assert (value >> 28) & 0xF == 0x1
    assert (value >> 12) & 0xFFFF == 0xBA0
    assert (value >> 1) & 0x7FF == 0x023


def test_scan_bits_are_lsb_first_in_both_directions():
    """The wire order is LSB first, so a scan of a plain value round-trips."""
    for value in (0x00, 0x01, 0b1010, 0x5A, 0xFFFF, 0x1BA00046):
        assert scan_bits(value, 8) == [(value >> i) & 1 for i in range(8)]
        assert scan_value(scan_bits(value, 32)) == value & 0xFFFFFFFF
    assert scan_bits(0x02, 8)[0] == 0 and scan_bits(0x02, 8)[1] == 1, "bit 0 goes first"
    assert mask_opcode(0x1FF, 8) == 0xFF


def test_guard_rails():
    """A 2-bit IR is the minimum, and the master refuses nonsense."""
    with pytest.raises(ValueError):
        JtagMaster(ir_len=1)
    with pytest.raises(ValueError):
        JtagTarget(ir_len=1)
    with pytest.raises(ValueError):
        JtagChecker(ir_len=1)
    with pytest.raises(ValueError):
        JtagMaster(half=1)                 # see MIN_HALF: no target could be read
    with pytest.raises(ValueError):
        JtagMaster(ir_len=8).reset(clocks=RESET_CLOCKS - 1)
    with pytest.raises(ValueError):
        JtagMaster(ir_len=8).scan_dr(0, length=0)
    with pytest.raises(ValueError):
        JtagMaster(ir_len=8, dr_lengths={}).scan_dr(0)   # no length known
    with pytest.raises(ValueError):
        JtagMaster(ir_len=8).run()         # no bus yet
    with pytest.raises(KeyError):
        JtagTarget().opcode("NOPE")
    assert DEFAULT_DR_LENGTHS[None] == 1, "an unknown instruction selects 1-bit BYPASS"

    # ... and the bench helpers add the nets themselves, with TCK pulled low
    bus = jtag_bus()
    assert [bus.nets[n].pullup for n in ("tck", "tms", "tdi", "tdo")] == [0, 1, 1, 1]
    assert all(bus.nets[n].push_pull for n in ("tck", "tms", "tdi", "tdo"))
    m2 = attach_master(bus)
    assert m2.period == 6 and m2.low == 3 and m2.max_tck_hz == pytest.approx(1e9 / 120)
    assert bus.errors == []


def test_every_rule_has_a_test():
    """No rule in `CHECKER_RULES` may be unreachable, untested or undocumented."""
    source = Path(__file__).read_text()
    doc = (Path(__file__).resolve().parents[1] / "vip" / "jtag.py").read_text()
    assert len(CHECKER_RULES) == 15 and len(set(CHECKER_RULES)) == 15
    for rule in CHECKER_RULES:
        assert f'"{rule}"' in source, f"{rule} has no test"
        assert rule in doc, f"{rule} is not in the module docstring"
    # every rule the checker can actually flag is on the list, not just the other way round
    flagged = set()
    for fault in ({"master": {"launch_delay": 2}},
                  {"master": {"launch_delay": 5}},
                  {"master": {"half": 2}},
                  {"target": {"tdo_delay": 2}},
                  {"target": {"idle_tdo": 0, "ir_capture": 0b11, "idcode": 0xFFFFFFFF}},
                  {"checker": {"timing": {"tck_low": 200, "tck_high": 200}}}):
        bus, m, tap, chk = rig(**fault)
        m.reset()
        m.scan_ir(tap.opcode("SCRATCH"))
        m.scan_dr(0b01010101, length=8)
        run(m, bus)
        flagged |= rules(chk)
    assert flagged <= set(CHECKER_RULES), flagged - set(CHECKER_RULES)

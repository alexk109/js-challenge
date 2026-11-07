"""Self-tests for the pin-level SWD reference models: `make vip-selftest VIP=swd`.

Everything runs at 50 MHz (20 ns per tick) with a 200 ns SWCLK period, so 5 MHz on the wire --
comfortably inside the spec floor [S19] and inside what a 50 MHz PIO can bit-bang. `bench()`
builds the three-device bench; the tests then look at what came back over the wire, at what the
target's registers hold, and at what the checker says about the pins.

The rule tests come in pairs: a legal session that must be silent, and the same session with one
rule broken. The broken versions use the fault knobs the models expose (`SwdTarget(ignore=1)`,
`bad_read_parity=1`, ...), the `request_byte()` and `data_phase()` hooks of the host, or the
hand-built drivers at the end of this file. Section numbers in brackets, e.g. [S3], are the
sources in `research/ref/protocols/swd/`.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.swd import (
    ABORT_ORUNERRCLR,
    ABORT_STKERRCLR,
    ABORT_WDERRCLR,
    ACK_FAULT,
    ACK_NONE,
    ACK_OK,
    ACK_WAIT,
    CTRL_CDBGRSTACK,
    CTRL_CDBGRSTREQ,
    CTRL_CSYSPWRUPACK,
    CTRL_CSYSPWRUPREQ,
    CTRL_ORUNDETECT,
    CTRL_READOK,
    CTRL_STICKYERR,
    CTRL_STICKYORUN,
    CTRL_WDATAERR,
    DATA_BITS,
    DP_ABORT,
    DP_CTRL_STAT,
    DP_IDCODE,
    DP_RDBUFF,
    DP_SELECT,
    IDCODE_DEFAULT,
    MIN_TICKS_PER_BIT,
    REQ_PARK,
    REQ_RNW,
    REQ_START,
    REQ_STOP,
    SwdChecker,
    SwdHost,
    SwdTarget,
    attach_checker,
    attach_host,
    attach_target,
    data_bits,
    data_parity,
    decode_header,
    header_byte,
    header_parity,
    ns_to_ticks,
    swd_timings,
)

NS_PER_TICK = 20.0             # 50 MHz chip clock
TICKS_PER_BIT = 10             # 200 ns SWCLK period: 5 MHz
IDCODE = IDCODE_DEFAULT


def bench(tpb=TICKS_PER_BIT, host=None, target=None, check=True, check_timing=None,
          host_class=SwdHost):
    """A bus with an SwdHost, an SwdTarget and a passive SwdChecker watching the pins.

    `host=` and `target=` take keyword overrides for either model, `host_class=` swaps in a
    SwdHost subclass (a faulty probe), and `check_timing=` overrides the limits the checker judges
    by -- that is how a test asks for a stricter checker without touching the traffic.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus, ticks_per_bit=tpb, host_class=host_class, **(host or {}))
    target = attach_target(bus, **(target or {}))
    checker = attach_checker(bus, timing=check_timing) if check else None
    return bus, host, target, checker


def settle(host, bus, limit=500_000, clean=True):
    """Run the bus until the host is idle, and assert it got there.

    `clean=False` is for the tests that expect the host to complain, e.g. about a target that
    answered nothing.
    """
    assert host.run(bus, limit=limit), "the host did not finish its queue"
    if clean:
        assert host.errors == [], host.errors
    return bus


def opened(bus_kwargs=None):
    """A bench that has already done the connection sequence: reset, then IDCODE.

    Every register test starts from there, because after a line reset the DP answers nothing but
    an IDCODE read [S12], so a session that starts with anything else is not a legal one.
    """
    bus, host, target, checker = bench(**(bus_kwargs or {}))
    host.line_reset()
    idcode = host.read_idcode()
    settle(host, bus)
    assert idcode.ok and idcode.data == IDCODE, idcode
    return bus, host, target, checker


def test_models_talk_to_each_other():
    """Reset, read IDCODE, write and read CTRL/STAT, write and read SELECT.

    Everything both sides believe has to agree, and nothing may end up on the wire twice: a
    push-pull SWDIO can only contend if the two sides drive opposite values on the same tick,
    which is the one electrical fault this bus can have.
    """
    bus, host, target, checker = bench()
    reset = host.line_reset()
    idcode = host.read_idcode()
    power = host.write_ctrl_stat(CTRL_CSYSPWRUPREQ)
    ctrl = host.read_ctrl_stat()
    host.write_select(0x0F0F0000)
    sel = host.read_select()
    settle(host, bus)

    assert reset.ok and reset.queued_tick == 0 and reset.start_tick > 0
    assert idcode.ok and idcode.data == IDCODE and not idcode.parity_error
    assert power.ok and ctrl.ok and sel.ok
    assert power.ack == ACK_OK and ctrl.ack == ACK_OK and sel.ack == ACK_OK
    assert ctrl.data == CTRL_CSYSPWRUPREQ | CTRL_CSYSPWRUPACK, hex(ctrl.data or 0)
    assert sel.data == 0x0F0F0000
    assert bus.errors == [], "the two sides never drove SWDIO against each other"
    assert target.reads and target.writes, "the target saw the traffic"
    assert [w[1] for w in target.writes] == [DP_CTRL_STAT, DP_SELECT]
    assert [r[1] for r in target.reads] == [DP_IDCODE, DP_CTRL_STAT, DP_SELECT]

    # the checker's own decode of the same five frames, from the pins alone
    assert checker is not None and len(checker.frames) == 5
    assert [f.header for f in checker.frames] == [0xA5, 0xA9, 0x8D, 0xB1, 0x95]
    assert [f.ack for f in checker.frames] == [ACK_OK] * 5
    assert [f.data for f in checker.frames][0] == IDCODE
    assert checker.frames[0].reset_after, "the IDCODE read is what the line reset demands"
    assert not checker.frames[1].reset_after


def test_a_read_frame_is_46_clocks_and_a_write_47():
    """The frame lengths the spec's layout gives, counted on the wire by the checker.

    REQ(8) Trn(1) ACK(3) DATA(33) Trn(1) is 46 clocks for a read, and a write has one more
    because the direction changes before WDATA [S3].
    """
    bus, host, target, checker = opened()
    host.read_ctrl_stat()
    host.write_ctrl_stat(0)
    settle(host, bus)
    read, write = checker.frames[-2:]
    assert read.periods == REQUEST_BITS_EXPECTED and read.read
    assert write.periods == REQUEST_BITS_EXPECTED + 1 and not write.read
    assert host.ticks_per_bit == TICKS_PER_BIT
    assert abs(host.swclk_hz - 5e6) < 1.0, "200 ns per clock is 5 MHz"
    assert checker.shortest_period == TICKS_PER_BIT
    assert abs(checker.max_swclk_hz - 5e6) < 1.0


REQUEST_BITS_EXPECTED = 8 + 1 + 3 + DATA_BITS + 1 + 1     # REQ Trn ACK DATA PAR Trn


def test_idcode_fields_are_the_ones_the_spec_lists():
    """The default IDCODE decodes to Arm's documented fields [S5]."""
    assert IDCODE & 1 == 1, "bit 0 is always 1"
    assert (IDCODE >> 28) & 0xF == 0x2, "version"
    assert (IDCODE >> 12) & 0xFFFF == 0xBA01, "PARTNO, one of the SW-DP part numbers"
    assert (IDCODE >> 1) & 0x7FF == 0x23B, "JEP-106 designer, Arm's registered value"
    bus, host, target, checker = opened()
    assert target.idcode == IDCODE
    assert checker.frames[-1].data == IDCODE


def test_only_an_idcode_read_is_answered_after_a_line_reset():
    """The connection sequence ends with an IDCODE read, and nothing else gets through [S12].

    A DP that has just been reset ignores everything else, so a host that skips the IDCODE read
    hears nothing at all -- which is exactly why the sequence exists.
    """
    bus, host, target, checker = bench()
    reset = host.line_reset()
    ctrl = host.write_ctrl_stat(CTRL_CSYSPWRUPREQ)
    sel = host.read_select()
    idcode = host.read_idcode()
    settle(host, bus, clean=False)

    assert reset.ok
    assert len(host.errors) == 2, "the host complained about the two silent requests"
    assert target.resets == 1, "the reset was seen"
    assert ctrl.no_ack and sel.no_ack, "a reset DP answers nothing but IDCODE"
    assert ctrl.ack == ACK_NONE and sel.ack == ACK_NONE
    assert [hex(b) for (_t, b) in target.ignored] == ["0xa9", "0x95"], target.ignored
    assert target.protocol_errors == [], "ignored is not an error: the DP just stays quiet"
    assert idcode.ok and idcode.data == IDCODE, "the IDCODE read works and clears the reset"
    assert not target.in_line_reset
    assert target.ctrl_stat() == 0, "and the CTRL/STAT write never happened"


def test_a_second_line_reset_puts_the_dp_back_in_reset():
    """A reset is a reset: the power-up request is dropped, the DP waits for IDCODE again."""
    bus, host, target, checker = opened()
    host.write_ctrl_stat(CTRL_CSYSPWRUPREQ)
    host.read_ctrl_stat()
    host.line_reset()
    late = host.write_ctrl_stat(CTRL_CSYSPWRUPREQ)
    idcode = host.read_idcode()
    settle(host, bus, clean=False)

    assert target.resets == 2
    assert late.no_ack, "after the second reset only IDCODE is answered"
    assert idcode.ok and idcode.data == IDCODE
    assert target.ctrl_stat() == 0, "CTRL/STAT went back to its power-on value"
    assert late.waits == 0 and target.resets == 2


def test_the_abort_register_clears_the_sticky_flags():
    """WDATAERR, STICKYORUN and friends are sticky until ABORT clears them [S8, S10]."""
    bus, host, target, checker = opened()
    host.write_ctrl_stat(CTRL_ORUNDETECT)
    host.queue_read(DP_CTRL_STAT, ap=True)              # an AP read sets READOK [S3]
    host.queue_read(DP_CTRL_STAT)
    host.write_abort(ABORT_WDERRCLR | ABORT_ORUNERRCLR | ABORT_STKERRCLR)
    after = host.read_ctrl_stat()
    settle(host, bus)

    assert target.ctrl_stat() & CTRL_ORUNDETECT, "ORUNDETECT is a plain read-write bit"
    assert after.data & CTRL_READOK, "READOK follows the previous AP read"
    assert not target.ctrl_stat() & (CTRL_WDATAERR | CTRL_STICKYORUN | CTRL_STICKYERR)
    assert target.writes[-1][1:] == (DP_ABORT, ABORT_WDERRCLR | ABORT_ORUNERRCLR
                                     | ABORT_STKERRCLR)
    # and a FAULT response does set the overrun flag, which ABORT then clears [S9]
    bus2, host2, target2, checker2 = opened()
    target2.fault = 1                              # after the connection sequence, so it hits the read
    host2.read_ctrl_stat()
    flags = host2.read_ctrl_stat()
    host2.write_abort(ABORT_ORUNERRCLR)
    cleared = host2.read_ctrl_stat()
    settle(host2, bus2)
    assert flags.data & CTRL_STICKYORUN, hex(flags.data or 0)
    assert not cleared.data & CTRL_STICKYORUN, "ABORT.ORUNERRCLR cleared it"


def test_an_ap_access_is_answered_but_does_nothing():
    """There is no Access Port in this model, and the model says so rather than pretending.

    An AP access is answered with OK and a zeroed data phase; a write is accepted and dropped.
    That is a documented stub, not a MEM-AP [S1].
    """
    bus, host, target, checker = opened()
    host.queue_read(DP_CTRL_STAT, ap=True)             # CSW at AP address 0x4
    host.queue_write(DP_CTRL_STAT, 0xDEADBEEF, ap=True)
    host.queue_read(DP_IDCODE)
    settle(host, bus)

    assert [f.ap for f in checker.frames[-3:]] == [True, True, False]
    assert all(f.ack == ACK_OK for f in checker.frames)
    assert checker.frames[-3].data == 0, "an AP read comes back zeroed"
    assert target.ctrl_stat() == CTRL_READOK, "an AP write changed nothing, but the AP read did"

    assert checker.frames[-1].data == IDCODE, "the DP is still there afterwards"


def test_rdbuff_reads_zero_and_select_round_trips():
    """RDBUFF is the posted-read buffer; with no AP there is nothing in it [S6]."""
    bus, host, target, checker = opened()
    host.write_select(0x12345678)
    sel = host.read_select()
    rdbuff = host.read_rdbuff()
    settle(host, bus)

    assert sel.data == 0x12345678
    assert rdbuff.ok and rdbuff.data == 0
    assert checker.frames[-1].header == 0xBD, "a RDBUFF read is header 0xBD on the wire"

# ---- the rules, one broken at a time ------------------------------------------------
#
# Every rule the checker reports has a test here that breaks it and a matching legal case that
# must stay silent. `assert_broken()` is the common shape: run the session, insist the named
# rule is among the violations, that every violation carries a real tick, and that the wire was
# otherwise quiet.


def assert_broken(checker, rule, op=None, tick_positive=True):
    """The named rule fired, and every violation it recorded is a real one."""
    hit = checker.violations_by_rule(rule)
    assert hit, f"no {rule} violation; the checker said {checker.report()}"
    assert all(v.tick >= 0 for v in hit) if tick_positive else True
    assert all(v.detail or v.required for v in hit), "every violation says what it saw"
    if op is not None:
        assert not op.ok, f"and the host noticed too: {op}"
    return hit


def broken_session(check_timing=None, host=None, target=None, ops="reset_idcode_read"):
    """Run one of the fixed traffic scripts and hand back everything the test needs."""
    bus, host_dev, target_dev, checker = bench(check_timing=check_timing, host=host,
                                                target=target)
    if ops == "reset_idcode_read":
        host_dev.line_reset()
        first = host_dev.read_idcode()
        host_dev.read_ctrl_stat()
        host_dev.write_ctrl_stat(CTRL_CSYSPWRUPREQ)
    elif ops == "reset_read_idcode":
        host_dev.line_reset(cycles=20)
        first = host_dev.read_idcode()
    elif ops == "reset_then_read":
        host_dev.line_reset()
        first = host_dev.read_ctrl_stat()
        host_dev.read_idcode()
    else:                                   # pragma: no cover - a typo in a test
        raise AssertionError(ops)
    settle(host_dev, bus, clean=False)
    return bus, host_dev, target_dev, checker, first


def test_checker_passes_a_full_legal_session():
    """The same traffic with the default limits: nothing at all, on any of the fourteen rules."""
    bus, host, target, checker, first = broken_session()
    assert first.ok and first.data == IDCODE
    assert checker.clean, checker.report()
    assert checker.rules() == set()
    assert checker.resets == 1
    assert len(checker.frames) == 3, [str(f) for f in checker.frames]
    assert [f.addr for f in checker.frames] == [DP_IDCODE, DP_CTRL_STAT, DP_CTRL_STAT]
    assert all(f.ack == ACK_OK and not f.parity_error for f in checker.frames)
    assert bus.errors == []


def test_rule_tck_flags_swclk_running_too_fast():
    """tCK: a period shorter than the checker demands. 10 ticks measured, 40 required."""
    bus, host, target, checker, first = broken_session(check_timing={"min_period": 40})
    hits = assert_broken(checker, "tCK")
    assert {v.measured for v in hits} == {TICKS_PER_BIT}
    assert {v.required for v in hits} == {40}
    assert first.ok, "the transfer still worked: it was only too fast"
    assert checker.max_swclk_hz == 5e6


def test_rule_thigh_and_tlow_flag_a_short_half_period():
    """tHIGH and tLOW: the host's 5-tick halves against a checker that wants 8."""
    bus, host, target, checker, first = broken_session(check_timing={"min_high": 160,
                                                                   "min_low": 160})
    high = assert_broken(checker, "tHIGH")
    low = assert_broken(checker, "tLOW")
    # the first high period of each frame is short: the host had just released the clock
    assert min(v.measured for v in high) == TICKS_PER_BIT // 2
    assert max(v.measured for v in high) < 8
    assert {v.required for v in high} == {8}
    assert {v.measured for v in low} == {TICKS_PER_BIT // 2}
    assert first.ok, "still a working transfer, just off the timing"


def test_rule_tsu_data_flags_data_changed_too_late():
    """tSU;DATA: the host moves SWDIO three ticks before the edge instead of five.

    A legal host against the same strict limit is silent, so the rule is not just "be slow".
    """
    strict = {"t_setup": 100}                      # 5 ticks at 20 ns per tick
    bus, host, target, checker, first = broken_session(check_timing=strict,
                                                      host={"data_offset": 3})
    hits = assert_broken(checker, "tSU;DATA")
    assert {v.measured for v in hits} == {2}, "two ticks of setup against five required"
    assert {v.required for v in hits} == {5}
    assert first.ok, "the DP still read it: two ticks is 40 ns, and Tis is 4 ns"

    bus2, host2, target2, checker2, first2 = broken_session(check_timing=strict)
    assert checker2.clean, checker2.report()
    assert first2.ok


def test_rule_thd_data_flags_data_moved_off_the_edge_too_soon():
    """tHD;DATA: the host changes SWDIO one tick after the edge, where two are wanted.

    With the spec's one tick of hold (Tih = 1 ns) the same host is silent, so the rule measures
    the hold rather than the change.
    """
    bus, host, target, checker, first = broken_session(check_timing={"t_hold": 40},
                                                      host={"data_offset": -4})
    hits = assert_broken(checker, "tHD;DATA")
    assert {v.measured for v in hits} == {1}
    assert {v.required for v in hits} == {2}

    bus2, host2, target2, checker2, first2 = broken_session(host={"data_offset": -4})
    assert checker2.clean, "one tick of hold is what the spec asks for: " + checker2.report()


def test_rule_stop_flags_a_stop_bit_that_is_not_zero():
    """STOP: the header's stop bit is high. The DP drops the request, so no ACK comes back."""
    from verif.vip.swd import SwdHost

    class StopBitHost(SwdHost):
        def request_byte(self, op):
            return super().request_byte(op) ^ 0x40        # bit 6 is the stop bit [S15]

    bus, host, target, checker = bench(host_class=StopBitHost)
    host.line_reset()
    idc = host.read_idcode()
    settle(host, bus, clean=False)

    assert_broken(checker, "STOP")
    assert target.protocol_errors, "the DP saw a framing error and stayed quiet [S10]"
    assert idc.no_ack and idc.ack == ACK_NONE
    assert not checker.clean


def test_rule_park_flags_a_park_bit_that_is_not_one():
    """PARK: the header's park bit is low, which also leaves the line low in the turnaround."""
    class ParkBitHost(SwdHost):
        def request_byte(self, op):
            return super().request_byte(op) ^ REQ_PARK

    bus, host, target, checker = bench(host_class=ParkBitHost)
    host.line_reset()
    idc = host.read_idcode()
    settle(host, bus, clean=False)

    assert_broken(checker, "PARK")
    assert_broken(checker, "Trn"), "and the line was never free for the turnaround"
    assert idc.no_ack


def test_rule_parity_req_flags_a_wrong_request_parity_bit():
    """PARITY;REQ: the header's parity bit does not cover APnDP, RnW and A[3:2] [S2]."""
    class ParityHost(SwdHost):
        def request_byte(self, op):
            return super().request_byte(op) ^ 0x20

    bus, host, target, checker = bench(host_class=ParityHost)
    host.line_reset()
    idc = host.read_idcode()
    settle(host, bus, clean=False)

    hit = assert_broken(checker, "PARITY;REQ")
    assert "APnDP" in hit[0].detail
    assert idc.no_ack, "a header with a bad parity bit is not answered [S10]"
    assert [why for (_t, _b, why) in target.protocol_errors] == ["parity"]


def test_a_start_bit_of_zero_leaves_the_wire_unframed():
    """What a cleared start bit does, since the checker has no START rule to report it.

    A frame is found by its start bit [S15], so a header that starts with a 0 cannot be located:
    the DP latches onto a later clock, decodes a byte shifted by one, and the checker reports
    that byte instead. Both the DP and the checker notice; neither invents a start bit.
    """
    class StartBitHost(SwdHost):
        def request_byte(self, op):
            return super().request_byte(op) ^ REQ_START

    bus, host, target, checker = bench(host_class=StartBitHost)
    host.line_reset()
    idc = host.read_idcode()
    settle(host, bus, clean=False)

    assert idc.header == 0xA4, "the header really did lose its start bit"
    assert idc.no_ack, "the DP could not frame it either"
    assert target.protocol_errors, "and it said so"
    assert checker.rules() & {"STOP", "PARK", "PARITY;REQ"}, checker.report()
    assert checker.frames == [], "no frame was ever completed"


def test_rule_trn_flags_a_line_that_is_never_released():
    """Trn: a host that keeps driving SWDIO through the turnaround, plus the contention it causes.

    Two things are true at once and both are reported: the checker sees the line held low where
    it must be free, and `core.Bus` records the ticks where the two sides drove against each
    other. The ACK then decodes as noise, which the checker says as well.
    """
    class GreedyHost(SwdHost):
        def turnaround_level(self):
            return 0                       # never let go of the line

    bus, host, target, checker = bench(host_class=GreedyHost)
    host.line_reset()
    idc = host.read_idcode()
    settle(host, bus, clean=False)

    assert_broken(checker, "Trn")
    assert bus.errors, "and the two sides drove against each other"
    assert all(e.net == "swdio" for e in bus.errors)
    assert not idc.ok, "the ACK was unreadable"


def test_rule_ack_flags_an_ack_code_that_is_not_a_real_one():
    """ACK: 0b000 is not OK, WAIT or FAULT, so the host cannot trust the transfer [S17]."""
    bus, host, target, checker, first = broken_session(target={"bad_ack": 1})
    hit = assert_broken(checker, "ACK", op=first)
    assert "0b000" in hit[0].detail
    assert first.ack == 0, "the host took the three bits at face value, which is the trap"
    assert not first.ok
    # 0b011 is just as impossible
    bus2, host2, target2, checker2, first2 = broken_session(target={"bad_ack": 1,
                                                                    "bad_ack_code": 0b011})
    assert_broken(checker2, "ACK", op=first2)
    assert first2.ack == 0b011


def test_rule_no_ack_flags_a_request_nobody_answered():
    """NO-ACK: the line reads 1,1,1 where the ACK should be, on a well-formed request [S10]."""
    bus, host, target, checker, first = broken_session(target={"ignore": 1})
    assert_broken(checker, "NO-ACK", op=first)
    assert first.ack == ACK_NONE and first.no_ack
    assert target.silent, "the target dropped it on purpose"
    assert target.protocol_errors == [], "and that is not a protocol error"
    # the host's own recovery: a line reset and a retry gets through [S11, S12]
    bus2, host2, target2, checker2 = bench(target={"ignore": 1})
    host2.retries = 1
    host2.line_reset()
    idc = host2.read_idcode()
    settle(host2, bus2, clean=False)
    assert idc.ok and idc.data == IDCODE and idc.retries == 1
    assert checker2.rules() == {"NO-ACK"}, "the failed attempt is still on the record"
    assert checker2.resets == 2, "the retry was preceded by a line reset"


def test_rule_parity_data_flags_a_wrong_data_parity_bit():
    """PARITY;DATA: the read comes back with a parity bit that does not match its 32 bits."""
    bus, host, target, checker, first = broken_session(target={"bad_read_parity": 1})
    hit = assert_broken(checker, "PARITY;DATA")
    assert first.parity_error, "and the host refuses to believe the data [S4]"
    assert not first.ok
    assert first.data == IDCODE, "the data itself arrived; only the parity bit was wrong"
    assert checker.frames[0].parity_error
    assert "parity bit" in hit[0].detail


def test_rule_parity_data_flags_a_write_with_a_wrong_parity_bit():
    """PARITY;DATA on a write, and the DP's own reaction: WDATAERR, and nothing written [S10]."""
    class WdataParityHost(SwdHost):
        def data_phase(self, value, write):
            bits = super().data_phase(value, write)
            if write:
                bits[-1] ^= 1
            return bits

    bus, host, target, checker = bench(host_class=WdataParityHost)
    host.line_reset()
    host.read_idcode()
    write = host.write_select(0x12345678)
    read = host.read_select()
    settle(host, bus, clean=False)

    assert_broken(checker, "PARITY;DATA")
    assert write.ok, "the ACK said OK: the host cannot tell until the next access [S4]"
    assert target.ctrl_stat() & CTRL_WDATAERR, "the DP flagged the write-data error"
    assert not target.writes, "and did not apply it"
    assert read.data == 0, "so the register is untouched"


def test_rule_line_reset_flags_a_reset_that_is_too_short():
    """LINE-RESET: 20 high clocks do not reset anything; 50 do [S12]."""
    bus, host, target, checker, first = broken_session(ops="reset_read_idcode")
    hit = assert_broken(checker, "LINE-RESET")
    assert {v.measured for v in hit} == {20}
    assert {v.required for v in hit} == {50}
    assert target.resets == 0, "the DP never saw a reset either"
    assert first.ok, "and the transfer worked, because the DP was never in its reset state"


def test_rule_line_reset_flags_a_first_request_that_is_not_an_idcode_read():
    """LINE-RESET: the connection sequence ends with an IDCODE read, and the checker insists."""
    bus, host, target, checker, first = broken_session(ops="reset_then_read",
                                                      target={"ignore_after_reset": False})
    hit = assert_broken(checker, "LINE-RESET")
    assert "IDCODE" in hit[0].detail
    assert target.resets == 1
    assert first.ok, "this DP answers anyway, which is why the rule is worth having"

    # and the legal sequence is silent, with the flag on the frame that followed the reset
    bus2, host2, target2, checker2, first2 = broken_session()
    assert checker2.clean, checker2.report()
    assert checker2.frames[0].reset_after and not checker2.frames[1].reset_after


def test_checker_is_not_vacuous():
    """Three ways of showing the checker is really looking.

    The same wire judged against impossible limits, the same wire with one fault, and the same
    wire with the limits it is meant to have: all three must say something different.
    """
    bus, host, target, checker, first = broken_session()
    assert checker.clean and first.ok

    # 1. the same traffic, limits no SWD link could meet
    strict, host_s, target_s, checker_s, first_s = broken_session(
        check_timing={"min_period": 1000, "t_setup": 400, "t_hold": 400})
    assert not checker_s.clean, "an impossible limit must be impossible to meet"
    assert {"tCK", "tSU;DATA"} <= checker_s.rules(), checker_s.report()
    assert all(v.measured < v.required for v in checker_s.violations_by_rule("tCK"))

    # 2. the same traffic with one fault, judged against the real limits
    faulty, host_f, target_f, checker_f, first_f = broken_session(target={"ignore": 1})
    assert checker_f.rules() == {"NO-ACK"}, checker_f.report()
    assert first_s.ok and not first_f.ok

    # 3. and the rules are read from the limits it was given, not hard-coded: a checker told
    #    the wire is allowed to be slow accepts what a strict one rejects
    slow, host_sl, target_sl, checker_sl, first_sl = broken_session(check_timing={"min_period": 8})
    assert checker_sl.clean, checker_sl.report()


def test_the_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting: the DSTREAM table in ns, and the cycles."""
    t = swd_timings(NS_PER_TICK)
    assert (t["t_setup"], t["t_hold"]) == (1, 1), "Tis 4 ns and Tih 1 ns, rounded up [S19]"
    assert (t["min_high"], t["min_low"]) == (1, 1), "Thigh and Tlow are 10 ns each [S19]"
    assert t["min_period"] == 2, "and the period is 1/f of their sum"
    assert t["turnaround"] == 1, "the default Trn is exactly one clock cycle [S3]"
    assert (t["reset_high"], t["reset_low"]) == (50, 2), "50 high clocks, then 2 low [S12, S14]"
    assert t["reset_detect"] == 10, "longer than the eight ones a header can hold [S12, S15]"
    # the overrides: ns for the four timings, whole ticks for min_period, cycles for the rest
    assert swd_timings(NS_PER_TICK, {"t_setup": 100})["t_setup"] == 5
    assert swd_timings(NS_PER_TICK, {"min_period": 40})["min_period"] == 40
    assert swd_timings(NS_PER_TICK, {"turnaround": 3})["turnaround"] == 3
    assert swd_timings(20.0, {"min_low": 40})["min_low"] == 2, "rounding is up, never down"
    for bad in ({"t_nonsense": 1}, {"turnaround": -1}, {"reset_high": 0}, {"min_period": 0},
                {"reset_detect": 60}):
        with pytest.raises(ValueError):
            swd_timings(NS_PER_TICK, bad)


def test_the_encoders_are_the_ones_on_the_wire():
    """The request byte, its parity, and the data phase's parity, against the sources."""
    assert header_byte(read=True, addr=DP_IDCODE) == 0xA5
    assert header_byte(read=False, addr=DP_ABORT) == 0x81
    assert header_byte(read=False, addr=DP_CTRL_STAT) == 0xA9
    assert header_byte(read=False, addr=DP_SELECT) == 0xB1
    assert header_byte(read=True, addr=DP_RDBUFF) == 0xBD
    assert header_byte(read=True, addr=DP_IDCODE, ap=True) == 0x87
    for addr in (0x0, 0x4, 0x8, 0xC):
        for read in (False, True):
            for ap in (False, True):
                byte = header_byte(ap, read, addr)
                info = decode_header(byte)
                assert info["framing_ok"] and info["parity_ok"]
                assert (info["ap"], info["read"], info["addr"]) == (ap, read, addr)
                assert info["byte"] & REQ_START and info["byte"] & REQ_PARK
                assert not info["byte"] & REQ_STOP
    with pytest.raises(ValueError):
        header_byte(addr=0x2)
    assert header_parity(0xA5 & ~0x20) == 1 and header_parity(0x81) == 0
    assert data_parity(0) == 0 and data_parity(0xFFFFFFFF) == 0
    assert data_parity(0x2BA01477) == 0
    assert data_bits(0x12345678)[:8] == [0, 0, 0, 1, 1, 1, 1, 0]
    assert len(data_bits(0x12345678)) == DATA_BITS + 1
    assert data_bits(0, parity=1)[-1] == 1
    assert ns_to_ticks(NS_PER_TICK, 4) == 1 and ns_to_ticks(NS_PER_TICK, 41) == 3
    assert ns_to_ticks(NS_PER_TICK, -1) == 0


def test_the_models_reject_impossible_settings():
    """Guard rails: the arguments that would silently produce nonsense raise instead."""
    with pytest.raises(ValueError):
        SwdHost(ticks_per_bit=MIN_TICKS_PER_BIT - 1)
    with pytest.raises(ValueError):
        SwdHost(ticks_per_bit=10, sample_delay=9)
    with pytest.raises(ValueError):
        SwdHost(ticks_per_bit=10, sample_delay=0)
    with pytest.raises(ValueError):
        SwdHost(idle_cycles=0)
    with pytest.raises(ValueError):
        SwdHost(data_offset=6)
    host = SwdHost()
    with pytest.raises(ValueError):
        host.queue_line_reset(cycles=0)
    with pytest.raises(ValueError):
        SwdTarget(idcode=IDCODE_DEFAULT & ~1), "IDCODE bit 0 is always 1 [S5]"
    checker = SwdChecker()
    with pytest.raises(ValueError):
        SwdChecker(latency=-1)


def test_a_wait_is_reported_and_can_be_retried():
    """WAIT is a legal answer: the host records it, and repeats the request [S4]."""
    bus, host, target, checker = bench()
    host.line_reset()
    host.read_idcode()
    settle(host, bus)                    # the connection sequence, answered OK
    target.wait = 1                      # only the next transfer gets a WAIT
    waited = host.read_ctrl_stat()
    settle(host, bus, clean=False)
    assert waited.ack == ACK_WAIT and waited.waits == 1 and not waited.ok
    assert target.ctrl_stat() & CTRL_STICKYORUN, "a WAIT is not OK, so STICKYORUN is set [S9]"
    assert "WAIT" in str(waited)

    bus2, host2, target2, checker2 = bench()
    host2.retries = 1
    host2.line_reset()
    host2.read_idcode()
    settle(host2, bus2)
    target2.wait = 1
    retried = host2.read_ctrl_stat()
    settle(host2, bus2, clean=False)
    assert retried.ok and retried.waits == 1 and retried.retries == 1
    assert retried.data == target2.ctrl_stat() | CTRL_STICKYORUN


def test_a_fault_response_is_answered_and_recorded():
    """FAULT: the host does not repeat it, because only a WAIT is retried [S4]."""
    bus, host, target, checker = bench()
    host.line_reset()
    host.read_idcode()
    settle(host, bus)
    target.fault = 1
    faulted = host.read_ctrl_stat()
    settle(host, bus, clean=False)
    assert faulted.ack == ACK_FAULT and not faulted.ok
    assert faulted.waits == 0 and faulted.retries == 0, "FAULT is not a WAIT"
    assert target.ctrl_stat() & CTRL_STICKYORUN


def test_overrun_detect_makes_every_response_carry_data():
    """ORUNDETECT set: a data phase follows every ACK, including a WAIT [S1]."""
    bus, host, target, checker = bench(host={"overrun_detect": True},
                                      target={"overrun_detect": True})
    host.line_reset()
    host.read_idcode()
    settle(host, bus)
    target.wait = 1
    waited = host.read_ctrl_stat()
    settle(host, bus, clean=False)
    assert waited.ack == ACK_WAIT
    assert waited.data is not None, "the host expected a data phase, because it set ORUNDETECT"
    assert target.ctrl_stat() & CTRL_ORUNDETECT
    # and the checker cannot know ORUNDETECT was set, so it reads that trailing data as noise
    assert not checker.clean, checker.report()


def test_a_frame_of_frames_keeps_the_line_in_step():
    """A long mixed session: nothing drifts, and every frame is where the host put it.

    The host keeps its own period counter and the checker counts rising edges of its own, so
    this is the test that would catch a slip in either.
    """
    bus, host, target, checker = bench()
    host.line_reset()
    ops = [host.read_idcode()]
    for value in (0x1, 0x2, 0x4, 0x8, CTRL_CSYSPWRUPREQ, 0x100):
        ops.append(host.write_select(value))
        ops.append(host.read_select())
        ops.append(host.write_ctrl_stat(value))
        ops.append(host.read_ctrl_stat())
    settle(host, bus)

    assert all(op.ok for op in ops), [str(o) for o in ops if not op.ok]
    assert len(ops) == 25
    assert [op.header for op in ops] == [0xA5] + [0xB1, 0x95, 0xA9, 0x8D] * 6
    assert [op.start_tick for op in ops] == sorted(op.start_tick for op in ops)
    assert [op.end_tick for op in ops] == sorted(op.end_tick for op in ops)
    assert len(checker.frames) == len(ops)
    # the host logs the tick it drove each edge on and the checker takes its own tick of
    # observation delay out, so the two name the same instant
    assert [(f.tick, f.header) for f in checker.frames] == \
        [(op.start_tick, op.header) for op in ops], "checker and host agree on every frame"
    assert all(op.queued_tick <= op.start_tick <= op.end_tick for op in ops)
    assert checker.clean, checker.report()
    assert bus.errors == []
    assert checker.shortest_period == TICKS_PER_BIT

"""End-to-end check of fw/jtag on the ISS: program, driver, chip and bus.

The judge here is not the driver: it is `verif.vip.jtag`, which is the only thing that
looks at the bus. `JtagTarget` is the TAP on the far end -- the chip is the controller,
so the VIP supplies the device the chip talks to -- and `JtagChecker` decodes the wire and
checks all 15 of its rules. Neither is edited and neither is softened: a violation fails
the test, and `test_a_tck_phase_that_is_too_short_is_caught` shows the checker really does
bite on this bench rather than passing everything.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`; the four JTAG lines are wired to whichever `uio` pins the
configuration names, push-pull, with TCK's pull-up low and the other three high -- the idle
levels `verif.vip/jtag.jtag_bus()` gives them. The VIP devices are constructed with those
net names rather than through `jtag.attach_*`, because those helpers add their own
`tck`/`tms`/`tdi`/`tdo` nets and the checker would then watch an empty bus.

`FW_TARGET=iss` runs it against `iss.pio_iss.Chip`; `FW_TARGET=rtl` runs the same bench
against `src/pio_chip.sv` under cocotb, one core cycle per tick (`fw/rtl_harness.py`).
"""
from __future__ import annotations

import pytest

from fw.jtag import assemble_program, JtagDriver, retime
from fw.jtag import driver as drv
from iss import pio_iss
from verif.vip import jtag as vip
from verif.vip.core import Bus

TICK_NS = 20.0                    # 50 MHz, the clock the timing minimums are quoted at
DEFAULT_PINS = (drv.DEFAULT_TCK_PIN, drv.DEFAULT_TMS_PIN,
                drv.DEFAULT_TDI_PIN, drv.DEFAULT_TDO_PIN)

# The fastest configuration the ISS can run, and the number fw/jtag/measure.py re-measures
# and the README quotes: CLKDIV 1.0 with the program's own LOW_HOLD 2 / HIGH_HOLD 3, which
# is 9 core cycles per TCK cycle, so 1e9 / (20 ns * 9) = 5.56 MHz. `docs/mvp-isa.md
# `## Timing` is what sets the floor: T_IN = 3 puts the `in pins, 1` exactly three cycles
# after the rising edge, so HIGH_HOLD cannot be shortened without moving the sample off the
# edge, and TDO appears two ticks after the falling edge with Ttdo = 1 tick wanted before
# the rising one, so LOW_HOLD cannot go below 2 either.
FAST = dict(clkdiv=1)
#: The slowest: the same wire at 1.39 MHz, four core cycles per SM cycle.
SLOW = dict(clkdiv=4)


class Bench:
    """A bus with the chip as controller, a TAP on the far end, and the checker watching.

    `pins` is `(TCK, TMS, TDI, TDO)`. `instructions` and `ir_len` configure the TAP *and*
    the checker, because the checker has to know the target's instruction table to judge a
    DR scan's length (`JtagChecker`, "Configuration").
    """

    def __init__(self, clkdiv=1, pins=DEFAULT_PINS, ir_len=8, instructions=None,
                 has_idcode=True, idcode=None, tdo_delay=0, program=None,
                 registers=None, **target):
        self.clkdiv = clkdiv
        self.pins = tuple(pins)
        tck_pin, tms_pin, tdi_pin, tdo_pin = self.pins
        jtag_pins = {tck_pin, tms_pin, tdi_pin, tdo_pin}
        self.bus = Bus(ns_per_tick=TICK_NS, trace=True)
        for k in range(pio_iss.NUM_PINS):
            # TCK idles low and a released TMS/TDI/TDO is pulled high, which is what
            # `jtag.jtag_bus()` builds; the other fourteen pins keep the default pull-up.
            self.bus.add_net(f"p{k}", push_pull=(k in jtag_pins),
                             pullup=(0 if k == tck_pin else 1))
        self.chip = pio_iss.Chip()
        self.device = pio_iss.ChipDevice(self.chip)
        self.bus.attach(self.device)
        nets = dict(tck_net=f"p{tck_pin}", tms_net=f"p{tms_pin}",
                    tdi_net=f"p{tdi_pin}", tdo_net=f"p{tdo_pin}")
        # `registers` is the target's own storage for the data registers in its instruction
        # table -- a name in the table is not a register with a value behind it, so a
        # write/read test has to say which ones have storage.
        self.target = vip.JtagTarget(ir_len=ir_len, instructions=instructions,
                                     has_idcode=has_idcode, idcode=idcode,
                                     tdo_delay=tdo_delay, registers=registers, **nets)
        self.bus.attach(self.target)
        self.checker = vip.JtagChecker(ir_len=ir_len, instructions=instructions,
                                       has_idcode=has_idcode, **nets)
        self.bus.attach(self.checker)
        self.program = program if program is not None else assemble_program()
        self.driver = JtagDriver(self.bus, self.device, self.program,
                                 clkdiv=clkdiv, pins=self.pins)

    def start(self) -> "Bench":
        self.driver.load()
        self.driver.clear_fdebug()
        return self

    # ---- what the wire carried -------------------------------------------------------
    def tck_ticks(self) -> int:
        """The TCK period the driver programs, in bus ticks."""
        return drv.tck_cycles(self.program) * self.clkdiv

    def tck_hz(self) -> float:
        return 1e9 / (TICK_NS * self.tck_ticks())

    def rises(self) -> list:
        """The ticks at which TCK rose, read off the bus trace.

        `JtagChecker` only keeps the *shortest* period it saw, which on this bench is the
        one that straddles the load and park transitions; the rate a scan actually runs at
        is the most common gap between rising edges, and that is what this histogram says.
        """
        out, prev = [], None
        for t, values in self.bus.trace:
            level = values[f"p{self.pins[0]}"]
            if prev is not None and level == 1 and prev == 0:
                out.append(t)
            prev = level
        return out

    def periods(self) -> dict:
        """How many times each rising-edge gap occurred, in ticks."""
        rises = self.rises()
        out = {}
        for a, b in zip(rises, rises[1:]):
            out[b - a] = out.get(b - a, 0) + 1
        return out

    def steady_period(self) -> int:
        """The TCK period of the clocks inside a scan: the most common gap, and there are
        far more of those than of the two transitions around a scan."""
        periods = self.periods()
        assert periods, "no rising TCK edge on the bus at all"
        return max(periods, key=lambda k: (periods[k], -k))

    def report(self) -> str:
        return self.checker.report()

    def no_violations(self) -> None:
        assert self.checker.violations == [], self.report()


# ---- the headline case: reset, select IDCODE, read it, at the fastest divider ----------
def test_idcode_at_the_fastest_divider():
    bench = Bench(**FAST).start()
    driver, target, checker = bench.driver, bench.target, bench.checker

    # The whole point of JTAG, and the only 32-bit register there is.
    got = driver.idcode()
    assert got == target.idcode == vip.DEFAULT_IDCODE == 0x10BA0046

    bench.no_violations()
    assert bench.bus.errors == [], "two devices drove a push-pull JTAG line differently"

    # The checker decoded the wire into the two scans the sequence is made of, and nothing
    # else: an 8-bit IR scan and a 32-bit IDCODE scan, both from Run-Test/Idle.
    assert [(s.kind, s.length) for s in checker.scans] == [("ir", 8), ("dr", 32)]
    ir, dr = checker.scans
    assert ir.value == vip.IR_CAPTURE_LSB_PATTERN, "Capture-IR shifted out 0,1"
    assert ir.sent == drv.OPCODE_IDCODE
    assert dr.value == target.idcode and dr.sent == 0

    # ... and the target agrees about what happened to it.
    assert target.instruction == vip.OPCODE_IDCODE
    assert target.last_idcode == vip.DEFAULT_IDCODE
    assert checker.resets == 1 and not target.unknown_instructions

    # The rate: every TCK cycle of a scan is the period the driver programs, so the chip is
    # running TCK as fast as this program can clock it.
    assert bench.steady_period() == bench.tck_ticks() == 9
    assert bench.tck_hz() == pytest.approx(5.555_555_6e6)
    assert checker.max_tck_hz() <= vip.IJET_TIMINGS.max_tck_hz, bench.report()

    # Every TCK cycle the driver thinks it clocked is one the checker saw, bar the single
    # rising edge of `init_pin` that turns the chip into a master.
    assert checker.rises == driver.clocks + 1

    # The host kept both FIFOs flowing and the engine never stalled on one.
    assert driver.fdebug() == 0, f"FDEBUG {driver.fdebug():#010x} after the sequence"
    assert driver.peak_fifo_depth <= drv.FIFO_DEPTH


def test_a_slower_divider_scales_the_wire_exactly():
    """The same sequence at CLKDIV 4: one SM cycle is four core cycles, so every TCK cycle
    is four times as long and the rate is a quarter of the fastest -- measured, not assumed."""
    fast, slow = Bench(**FAST).start(), Bench(**SLOW).start()
    for bench in (fast, slow):
        assert bench.driver.idcode() == vip.DEFAULT_IDCODE
        bench.no_violations()
    assert slow.tck_ticks() == 4 * fast.tck_ticks() == 36
    assert slow.steady_period() == 36
    assert slow.tck_hz() == pytest.approx(1.388_888_9e6)
    # One core cycle per SM cycle is the floor, so this is as fast as the chip can shift.
    assert bench_is_at_the_floor(fast) and bench_is_at_the_floor(slow)


def bench_is_at_the_floor(bench: Bench) -> bool:
    """True when the program is at the timing floor docs/mvp-isa.md `## Timing` sets.

    `HIGH_HOLD` cannot go below 3 (that is T_IN = 3, and the sample belongs on the rising
    edge), and `LOW_HOLD` cannot go below 2 (TDO lands two ticks after the falling edge and
    the profile wants Ttdo = 1 tick of validity before the rising one). Going faster means
    moving the sample off the edge, which is a different program.
    """
    low, high = drv.loop_delays(bench.program)
    return low == 2 and high == 3


# ---- the same program against a different TAP, on different pins ----------------------
#: A four-bit instruction register with a 16-bit user register and no IDCODE at all: the
#: program knows nothing about any of it, because the instruction table is the *target's*
#: and the driver is told the two lengths it needs (`scan_ir(..., ir_len=4)`,
#: `scan_dr(..., 16)`). The checker is configured with the same table, or it would judge the
#: DR scan against IDCODE's 32 bits.
NARROW = {0x3: ("USER", 16), 0xF: ("BYPASS", 1)}


def test_another_tap_on_another_pin_map():
    """A 4-bit IR, a 16-bit data register, no IDCODE, and the four lines on pins 2..5.

    Nothing in program.pio changes: the instruction register width, the data register
    length and the opcode are the target's business and arrive as data, and the pin map
    arrives in PINCTRL. What does not change is the wire: the checker sees the same kind of
    scans and finds nothing wrong with any of them.
    """
    bench = Bench(pins=(2, 3, 4, 5), ir_len=4, instructions=NARROW, registers={"USER": 0},
                  has_idcode=False, idcode=0, **FAST).start()
    driver, target, checker = bench.driver, bench.target, bench.checker

    driver.reset()
    assert checker.resets == 1
    # Capture-IR of a 4-bit register still reads 0,1 in bits 1:0 (IR_CAPTURE_LSB_PATTERN).
    assert driver.scan_ir(0x3, ir_len=4) == vip.IR_CAPTURE_LSB_PATTERN
    assert target.instruction == 0x3

    # A write and a read-back of the 16-bit user register.
    assert driver.scan_dr(0xBEEF, 16) == 0, "nothing was in it yet"
    assert target.registers["USER"] == 0xBEEF
    assert driver.scan_dr(None, 16) == 0xBEEF

    # And BYPASS, the one-bit register every chain has.
    driver.scan_ir(0xF, ir_len=4)
    assert target.name_of(target.instruction) == "BYPASS"
    assert driver.scan_dr(0, 1) == 0, "BYPASS captures a zero bit"

    bench.no_violations()
    assert bench.bus.errors == []
    assert [(s.kind, s.length) for s in checker.scans] == [
        ("ir", 4), ("dr", 16), ("dr", 16), ("ir", 4), ("dr", 1),
    ]
    assert driver.fdebug() == 0
    # The lines really are on pins 2..5 now: TDO is an input, the other three are driven.
    oe = bench.chip.pin_oe
    assert (oe >> 2) & 0b1111 == 0b0111, f"DBG_PADOE {oe:#010x}"
    assert not (oe >> 5) & 1, "TDO has to stay an input"


def test_bypass_and_a_user_register_on_the_built_in_tap():
    """The shortest and an eight-bit scan on the VIP's own target: BYPASS (1 bit) and
    SCRATCH (8 bits, read back). A one-bit scan is the case the loop arithmetic has to get
    right -- X = 0 is one clock, not none."""
    bench = Bench(**FAST).start()
    driver, target = bench.driver, bench.target

    driver.reset()
    assert driver.scan_ir(target.opcode("BYPASS")) == vip.IR_CAPTURE_LSB_PATTERN
    assert driver.scan_dr(0x5A, 1) == 0

    driver.scan_ir(target.opcode("SCRATCH"))
    assert driver.scan_dr(0xA5, 8) == 0
    assert target.registers["SCRATCH"] == 0xA5
    driver.scan_ir(target.opcode("SCRATCH"))
    assert driver.scan_dr(None, 8) == 0xA5

    bench.no_violations()
    assert driver.fdebug() == 0
    assert [s.length for s in bench.checker.scans if s.kind == "dr"] == [1, 8, 8]


def test_idle_clocks_and_a_second_reset():
    """Idle clocks keep the TAP where it is, and a reset from any state works -- which is
    the point of TMS being high for five clocks ("if TMS is high for at least five clock
    cycles", research/ref/protocols/jtag/segger-kb-jtag.txt)."""
    bench = Bench(**FAST).start()
    driver, checker = bench.driver, bench.checker

    driver.idcode()
    driver.idcode()
    driver.idcode()
    driver.idcode()
    driver.reset()
    driver.reset()
    assert checker.resets >= 5, "five TMS-high clocks per reset, every time"
    bench.no_violations()
    assert driver.fdebug() == 0


# ---- the driver against the spec's own tables, and against itself ------------------------
def test_the_tms_sequences_are_the_state_table_s_own():
    """Every TMS level the driver emits, walked through `verif.vip.jtag`'s transition table.

    The firmware must not borrow the model it is judged against, so `driver.py` spells its
    scans out as constants. This is the check that those constants are the spec's: each
    sequence is replayed through `jtag.next_state` from Run-Test/Idle and has to visit the
    states the scan is supposed to visit, shift exactly as many bits as the register is
    wide, and come back to Run-Test/Idle.
    """
    def walk(tms_levels, start=vip.RUN_TEST_IDLE):
        states = [start]
        for tms in tms_levels:
            states.append(vip.next_state(states[-1], tms))
        return states

    def levels(entry, bits, exit_):
        """The TMS levels of one scan, exactly as `scan()` builds them.

        A scan with nothing to shift still clocks once through the shift phase (a loop
        cannot be told "zero times", see program.pio), and that clock carries the level in
        front of it -- which is what makes the reset below five TMS-high clocks and not four.
        """
        out = list(entry)
        if not bits:
            out.append(entry[-1] if entry else 0)
        out += [1 if k == len(bits) - 1 else 0 for k in range(len(bits))]
        return out + list(exit_)

    # An IR scan: Select-IR-Scan, Capture-IR, Shift-IR for ir_len clocks, Exit1-IR,
    # Update-IR, Run-Test/Idle.
    ir = levels(drv.IR_ENTRY_TMS, [0] * 8, drv.EXIT_TMS)
    states = walk(ir)
    assert states[:5] == [vip.RUN_TEST_IDLE, vip.SELECT_DR_SCAN, vip.SELECT_IR_SCAN,
                          vip.CAPTURE_IR, vip.SHIFT_IR]
    assert states.count(vip.SHIFT_IR) == 8, "one Shift-IR clock per bit of the IR"
    assert states[-3:] == [vip.EXIT1_IR, vip.UPDATE_IR, vip.RUN_TEST_IDLE]

    # A DR scan: the same walk through the DR states.
    dr = levels(drv.DR_ENTRY_TMS, [0] * 32, drv.EXIT_TMS)
    states = walk(dr)
    assert states[:4] == [vip.RUN_TEST_IDLE, vip.SELECT_DR_SCAN, vip.CAPTURE_DR,
                          vip.SHIFT_DR]
    assert states.count(vip.SHIFT_DR) == 32, "one Shift-DR clock per bit of the register"
    assert states[-3:] == [vip.EXIT1_DR, vip.UPDATE_DR, vip.RUN_TEST_IDLE]

    # A reset: five TMS-high clocks reach Test-Logic-Reset from anywhere, then one low puts
    # the TAP in Run-Test/Idle where a scan starts.
    reset = levels((1,) * 4, [1], drv.EXIT_TMS)     # the shift clock carries the fifth
    assert reset == [1, 1, 1, 1, 1, 1, 0], "five high clocks, then one low"
    for start in vip.TAP_STATES:
        assert walk(reset, start)[-1] == vip.RUN_TEST_IDLE, start

    # Idle clocks: TMS low in Run-Test/Idle stays there.
    idle = levels(drv.IDLE_ENTRY_TMS, [], drv.IDLE_EXIT_TMS)
    assert len(idle) == 5 and set(idle) == {0}, "two framing clocks, one shift, two exit"
    assert walk(idle) == [vip.RUN_TEST_IDLE] * 6


def test_the_driver_packs_bits_the_way_the_wire_does():
    """`scan_bits` is the firmware's own LSB-first expansion, and the VIP has one too.

    They are independent implementations of the same sentence ("Least significant bit (LSB)
    comes first", research/ref/protocols/jtag/bit-order-svf-tutorial.txt), so they have to
    agree -- and `pair`/`pack_pairs` have to put a scan's first cycle in the OSR's first
    two bits, because that is the order `out pins, 2` walks them out in.
    """
    for value, width in ((0x00, 8), (0x02, 8), (0xA5, 8), (0x10BA0046, 32), (0xBEEF, 16)):
        assert drv.scan_bits(value, width) == vip.scan_bits(value, width)
        assert drv.scan_bits(value, width)[0] == value & 1, "the first bit out is bit 0"
    with pytest.raises(ValueError):
        drv.pair(2)
    # One 32-bit word is 16 TCK cycles, cycle 0 in bits 1:0, TMS in bit 0 and TDI in bit 1.
    cycles = [drv.pair(1, 0), drv.pair(0, 1)] + [drv.pair(0)] * 14
    word = drv.pack_pairs(cycles)[0]
    assert word & 0x3 == 0b01 and (word >> 2) & 0x3 == 0b10
    with pytest.raises(drv.JtagError):
        drv.pack_pairs([drv.pair(0)] * 17)


# ---- the checker is not vacuous on this bench, and the program is what it says it is -----
def test_a_tck_phase_that_is_too_short_is_caught():
    """The harness has to be able to fail: `retime` puts LOW_HOLD one SM cycle lower, which
    makes the TCK low phase two ticks, and TDO then appears on the very tick the rising edge
    is measured against. `JtagChecker` must notice -- a bench where nothing can fail proves
    nothing about the firmware."""
    program = retime(assemble_program(), low_hold=1)
    assert drv.tck_cycles(program) == 8, "one core cycle faster"
    bench = Bench(program=program, **FAST).start()
    assert bench.driver.idcode() == vip.DEFAULT_IDCODE, "the data still comes out right"
    rules = {v.rule for v in bench.checker.violations}
    assert "TDO.setup" in rules, bench.report()
    assert bench.checker.violations_by_rule("TDO.setup"), bench.report()

    # One SM cycle back the other way and every rule is satisfied again, which is what makes
    # the first result a property of the timing and not of the harness.
    slow = Bench(program=retime(assemble_program(), low_hold=3), **FAST).start()
    assert slow.driver.idcode() == vip.DEFAULT_IDCODE
    slow.no_violations()
    assert slow.steady_period() == 10 and slow.tck_hz() == pytest.approx(5.0e6)


def test_the_program_fits_the_instruction_memory():
    program = assemble_program()
    # 17 words of the 32 an engine has (docs/mvp-isa.md `### Engine state`). The budget is
    # in fw/jtag/README.md; what matters here is that the margin is real and that the wrap
    # window cannot drop a scan into the middle of one.
    assert len(program.instrs) == 17
    assert len(program.instrs) <= 32
    assert program.wrap_target == program.labels["park"]
    assert program.wrap == program.labels["last"] == len(program.instrs) - 1
    assert program.wrap_target <= program.labels["frame"] <= program.wrap
    # `.side_set 1` with no `opt`: every instruction drives TCK, which is what lets the two
    # halves of a TCK cycle live in the side-set value alone.
    assert program.sideset_bits == 1 and not program.sideset_opt
    assert not program.sideset_pindirs, "TCK is a level: JTAG is push-pull"
    # The loop arithmetic the driver's `loop_count` exists for.
    low, high = drv.loop_delays(program)
    assert (low, high) == (2, 3), "LOW_HOLD 2 is the floor, HIGH_HOLD 3 is T_IN"
    assert drv.tck_cycles(program) == 9
    assert drv.loop_count(1) == 0 and drv.loop_count(32) == 31
    with pytest.raises(drv.JtagError):
        drv.loop_count(0)


def test_the_driver_addresses_match_the_register_map_and_the_iss():
    """The driver is written against docs/mvp-regmap.md; this is the guard against the two
    drifting apart, in particular against the ISS renumbering something."""
    assert (drv.A_CTRL, drv.A_FSTAT, drv.A_FDEBUG, drv.A_FLEVEL) == (
        pio_iss.A_CTRL, pio_iss.A_FSTAT, pio_iss.A_FDEBUG, pio_iss.A_FLEVEL
    )
    assert (drv.A_TXF0, drv.A_RXF0) == (pio_iss.A_TXF[0], pio_iss.A_RXF[0])
    assert drv.A_IMEM == pio_iss.A_IMEM
    assert (drv.A_DBG_PADOUT, drv.A_DBG_PADOE, drv.A_DBG_CFGINFO) == (
        pio_iss.A_DBG_PADOUT, pio_iss.A_DBG_PADOE, pio_iss.A_DBG_CFGINFO
    )
    assert (drv.SM_CLKDIV, drv.SM_EXECCTRL, drv.SM_SHIFTCTRL, drv.SM_ADDR,
            drv.SM_INSTR, drv.SM_PINCTRL) == (
        pio_iss.SM_CLKDIV, pio_iss.SM_EXECCTRL, pio_iss.SM_SHIFTCTRL, pio_iss.SM_ADDR,
        pio_iss.SM_INSTR, pio_iss.SM_PINCTRL
    )


def test_the_pin_map_has_to_match_the_windows_the_program_writes():
    """TCK, TMS and TDI come out of three windows that are immediates in IMEM -- a one-bit
    side-set, a two-pin OUT window and a three-pin SET window -- so the map has to be three
    consecutive pins with TDO outside them. A map that is not, is refused rather than
    silently mis-wired."""
    program = assemble_program()
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    bus.attach(device)
    for good in ((0, 1, 2, 3), (2, 3, 4, 5), (5, 6, 7, 17), (0, 1, 2, 8)):
        JtagDriver(bus, device, program, pins=good)
    for bad in ((0, 2, 3, 4),      # TMS and TDI not adjacent: no two-pin window reaches both
                (0, 1, 2, 1),      # TDO inside the driven window: `set pindirs, 7` would
                (0, 1, 2, 2),      #   make TDO an output, and the chip could not read it
                (7, 8, 9, 10),     # not uio pins at the bottom of the map
                (6, 7, 8, 10)):    # TCK = 6 puts the driven window off the end
        with pytest.raises(ValueError):
            JtagDriver(bus, device, program, pins=bad)
    with pytest.raises(ValueError):
        JtagDriver(bus, device, program, clkdiv=0)
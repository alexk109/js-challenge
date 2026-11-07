"""End-to-end check of fw/swd on the ISS: program, driver, chip, DP and checker.

The judge here is not the driver: it is `verif.vip.swd`, whose `SwdTarget` is an SW-DP
that answers requests and whose `SwdChecker` decodes the wire and holds it to every rule
it knows.  Neither is edited and neither is softened: a violation fails the test.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`.  The two SWD nets are two of those pins: SWDIO is a
`uio` pin, because open drain on this chip is a direction (docs/mvp-isa.md
`### Open drain`), and SWCLK is an output-only pin, which is always driven and so needs
no direction written anywhere.  The two nets are push-pull, so a tick on which the chip
and the DP drove SWDIO opposite ways would show up as a `Contention` in `bus.errors` --
the one electrical fault a two-wire debug link has.

`FW_TARGET=iss` runs it against `iss.pio_iss.Chip`; `FW_TARGET=rtl` runs the same bench
against `src/pio_chip.sv` under cocotb, one core cycle per tick (`fw/rtl_harness.py`).
"""
from __future__ import annotations

import pytest

from fw.swd import driver as drv
from fw.swd.driver import SwdDriver, SwdError, load_program
from iss import pio_iss
from verif.vip import swd as vswd
from verif.vip.core import Bus

TICK_NS = 20.0                    # 50 MHz, the clock the timing minimums are quoted at
DEFAULT_PINS = (drv.DEFAULT_SWDIO_PIN, drv.DEFAULT_SWCLK_PIN, drv.DEFAULT_FLAG_PIN)

# Two configurations, both far enough from the sampling limit to have margin and both
# inside what the checker can measure: the fastest the program can clock (HIGH at its
# minimum of 6, LOW 0) and a slower one with a longer low phase, which is what a real
# probe's slowest rate looks like.  `retarget` puts the two numbers into the delay fields,
# so they are the same 31 words at both.
FAST = dict(high=6, low=0)         # 9 core cycles per bit period: 5.56 MHz
SLOW = dict(high=8, low=9)         # 20 core cycles: 2.50 MHz
SLOWEST = dict(high=13, low=14)    # 30 core cycles: 1.67 MHz, the longest the delay fields reach
# The fastest the program can go, and what stops it going further.  HIGH is the delay
# between the clock going high and `in pins, 1` sampling it, and the DP's answer is on the
# pad two ticks after the rising edge while this chip reads SWDIO T_IN = 3 ticks after
# the pad (docs/mvp-isa.md `## Timing`), so HIGH has to be at least 5 and the driver
# refuses less than 6.  LOW 0 is the shortest low phase the loops allow.
MAX_SPEED = dict(high=drv.MIN_HIGH, low=0)
MAX_SPEED_PERIOD = drv.LOOPS_PER_PERIOD + drv.MIN_HIGH     # 3 loop instructions + delays
MAX_SPEED_HZ = 1e9 / (TICK_NS * MAX_SPEED_PERIOD)          # 5.56 MHz at 50 MHz
MIN_SPEED_HZ = 1e9 / (TICK_NS * (drv.LOOPS_PER_PERIOD + 13 + 14))   # 1.67 MHz

# The record the README quotes, checked by `test_the_fastest_bit_period_still_passes`
# so a change in the word budget or in the input path cannot move it unnoticed.
RECORDED_WORDS = 31


class Bench:
    """A bus with the chip, an SW-DP and the checker on it."""

    def __init__(self, high=FAST["high"], low=FAST["low"], idcode=vswd.IDCODE_DEFAULT,
                 swdio=DEFAULT_PINS[0], swclk=DEFAULT_PINS[1], flag=DEFAULT_PINS[2],
                 target: dict | None = None, checker_timing: dict | None = None):
        self.swdio, self.swclk, self.flag = swdio, swclk, flag
        self.bus = Bus(ns_per_tick=TICK_NS)
        for k in range(pio_iss.NUM_PINS):            # p0..p17, all pulled up
            self.bus.add_net(f"p{k}", push_pull=k in (swdio, swclk))
        self.chip = pio_iss.Chip()
        self.device = pio_iss.ChipDevice(self.chip)
        self.bus.attach(self.device)
        # The models look at the bus through the chip's own pin nets, so there is no
        # coupling device in between and no extra tick of latency anywhere.
        nets = dict(clk_net=f"p{swclk}", io_net=f"p{swdio}")
        self.target = vswd.attach_target(self.bus, idcode=idcode, **(target or {}), **nets)
        self.checker = vswd.attach_checker(self.bus, timing=checker_timing, **nets)
        self.driver = SwdDriver(self.bus, self.device, high=high, low=low,
                                swdio=swdio, swclk=swclk, flag=flag)

    def start(self) -> "Bench":
        self.driver.load()
        self.driver.clear_fdebug()
        return self

    # ---- what the checker saw -----------------------------------------------------
    def clean(self) -> None:
        assert self.checker.violations == [], self.checker.report()
        assert self.bus.errors == [], f"contention on the SWD nets: {self.bus.errors[:3]}"
        assert self.driver.fdebug() == 0, (
            f"FDEBUG {self.driver.fdebug():#010x} after the transfer: the engine stalled on "
            f"a FIFO"
        )

    def headers(self) -> list:
        return [f.header for f in self.checker.frames]

    def report(self) -> str:
        return self.checker.report()

# ---- the headline: a whole ADIv5 connection sequence on the wire -------------
def test_line_reset_then_a_debug_session():
    """Reset the DP the way a probe does, then read and write its registers.

    Fifty SWDIO-high clocks are the line reset [S12]; from there the DP answers only an
    IDCODE read [S13], so that is the first transfer, and everything after it is an
    ordinary session.  The checker is what judges the whole thing.
    """
    bench = Bench(**FAST).start()
    driver, target, checker = bench.driver, bench.target, bench.checker

    driver.line_reset()
    assert target.resets == 1, "the DP did not see a line reset"
    assert target.in_line_reset, "the DP is not in its post-reset state"
    assert checker.resets == 1, checker.report()

    # After a reset only IDCODE is answered, and the checker wants exactly that.
    assert driver.read(vswd.DP_IDCODE) == vswd.IDCODE_DEFAULT
    assert not target.in_line_reset, "the IDCODE read should have taken the DP out of reset"
    first = checker.frames[0]
    assert first.header == 0xA5 and first.reset_after, checker.report()
    assert first.ack == vswd.ACK_OK and first.data == vswd.IDCODE_DEFAULT

    # CTRL/STAT: set CSYSPWRUPREQ, then read it back with the ACK bit the DP sets.
    driver.write(vswd.DP_CTRL_STAT, vswd.CTRL_CSYSPWRUPREQ)
    ctrl = driver.read(vswd.DP_CTRL_STAT)
    assert ctrl == vswd.CTRL_CSYSPWRUPREQ | vswd.CTRL_CSYSPWRUPACK, f"{ctrl:#010x}"
    assert ctrl == target.ctrl_stat(), "the chip and the DP disagree about CTRL/STAT"

    # SELECT takes any value, so it is the one register that proves the wire carries data
    # both ways rather than a pattern the two ends agree on by accident.
    driver.write(vswd.DP_SELECT, 0x1234_5678)
    assert driver.read(vswd.DP_SELECT) == 0x1234_5678
    assert driver.read(vswd.DP_RDBUFF) == 0, "nothing was posted, so RDBUFF reads zero"
    assert driver.read(vswd.DP_CTRL_STAT, ap=True) == 0, "this DP has no Access Port"

    bench.clean()
    assert [f.ack for f in checker.frames] == [vswd.ACK_OK] * len(checker.frames)
    assert bench.headers() == [
        0xA5,                                     # DP read IDCODE, after the reset
        0xA9,                                     # DP write CTRL/STAT
        0x8D,                                     # DP read CTRL/STAT
        0xB1,                                     # DP write SELECT
        0x95,                                     # DP read SELECT
        0xBD,                                     # DP read RDBUFF
        0xAF,                                     # AP read CTRL/STAT
    ], bench.report()
    assert [f.data for f in checker.frames if f.read] == [
        vswd.IDCODE_DEFAULT,
        vswd.CTRL_CSYSPWRUPREQ | vswd.CTRL_CSYSPWRUPACK,
        0x1234_5678,
        0,
        0,
    ], bench.report()


def test_two_speeds_and_a_second_pin_set():
    """The same program at two bit periods, and with the SWD nets on other pins."""
    for name, timing in (("fast", FAST), ("slow", SLOW)):
        bench = Bench(**timing).start()
        driver = bench.driver
        driver.line_reset()
        assert driver.read(vswd.DP_IDCODE) == vswd.IDCODE_DEFAULT
        driver.write(vswd.DP_SELECT, 0xDEAD_BEEF)
        assert driver.read(vswd.DP_SELECT) == 0xDEAD_BEEF
        bench.clean()
        assert bench.checker.shortest_period == driver.period, (
            f"{name}: the checker measured {bench.checker.shortest_period} ticks, the "
            f"driver asked for {driver.period}"
        )
        assert bench.checker.max_swclk_hz == pytest.approx(driver.config.swclk_hz), (
            f"{name}: {bench.checker.max_swclk_hz / 1e6:.3f} MHz on the wire against "
            f"{driver.config.swclk_hz / 1e6:.3f} MHz asked for"
        )

    # The pins are a PINCTRL field, so the same words run on another pair of them.
    bench = Bench(swdio=4, swclk=16, flag=17, **SLOW).start()
    assert bench.driver.read(vswd.DP_IDCODE) == vswd.IDCODE_DEFAULT
    bench.clean()


def test_a_sticky_flag_survives_and_an_abort_write_clears_it():
    """CTRL/STAT's sticky bits are the DP's memory of what went wrong.

    A write whose data parity bit is wrong is not applied and sets WDATAERR [S10], and an
    ABORT write clears it -- which is a register write that has to be exactly right in
    both directions to be worth anything.
    """
    bench = Bench(**FAST).start()
    driver, target, checker = bench.driver, bench.target, bench.checker
    driver.line_reset()
    driver.read(vswd.DP_IDCODE)

    driver.write(vswd.DP_SELECT, 0x1234_5678, bad_parity=True)
    assert driver.read(vswd.DP_SELECT) == 0, "a write with a bad parity bit was applied"
    ctrl = driver.read(vswd.DP_CTRL_STAT)
    assert ctrl & vswd.CTRL_WDATAERR, f"{ctrl:#010x}: the DP did not flag the parity error"
    assert [v.rule for v in checker.violations] == ["PARITY;DATA"], checker.report()

    driver.write(vswd.DP_ABORT, vswd.ABORT_WDERRCLR)
    ctrl = driver.read(vswd.DP_CTRL_STAT)
    assert not ctrl & vswd.CTRL_WDATAERR, f"{ctrl:#010x}: the ABORT write did not clear it"
    assert target.ctrl_stat() == ctrl
    # The checker saw the corrupted parity on the wire and still sees it: a wire rule is an
    # observation, not a register, so the ABORT write does not take it back.
    assert [v.rule for v in checker.violations] == ["PARITY;DATA"], checker.report()
    assert not bench.bus.errors, bench.bus.errors[:3]
    assert driver.fdebug() == 0, f"FDEBUG {driver.fdebug():#010x}"


def test_the_dp_answers_nothing_after_a_reset_but_an_idcode_read():
    """What ADIv5 says happens between a line reset and the IDCODE read [S13].

    The controller here cannot do much about it: `jmp pin` reads one pin and there is no
    IRQ to park a frame in.  So the test asserts the rule holds rather than the firmware
    enforcing it, which is the honest split.
    """
    bench = Bench(**FAST).start()
    driver, target, checker = bench.driver, bench.target, bench.checker
    driver.line_reset()
    driver.read(vswd.DP_CTRL_STAT)                  # not an IDCODE read: no answer
    assert target.ignored, "the DP should have dropped a non-IDCODE request"
    assert driver.read(vswd.DP_IDCODE) == vswd.IDCODE_DEFAULT
    assert driver.read(vswd.DP_CTRL_STAT) == target.ctrl_stat()

    # Three rules fire here, and all three are the checker being right about the wire:
    # the request was not the IDCODE read the reset asks for, nobody answered it, and the
    # line then sat high for 32 clocks without reaching 50 -- which is what an unanswered
    # request looks like to anything watching for a reset.
    assert [v.rule for v in checker.violations] == ["LINE-RESET", "NO-ACK", "LINE-RESET"], (
        checker.report()
    )
    assert [f.no_ack for f in checker.frames][:1] == [True], bench.report()
    # The IDCODE read that follows is the one the reset was waiting for; the checker's
    # `reset_after` flag rides on the frame that *starts* after the reset, which is the
    # dropped request, so the IDCODE read is plain.
    assert checker.frames[1].header == 0xA5 and not checker.frames[1].reset_after
    assert checker.frames[1].ack == vswd.ACK_OK, bench.report()
    assert not bench.bus.errors, bench.bus.errors[:3]
    assert driver.fdebug() == 0, f"FDEBUG {driver.fdebug():#010x}"


# ---- what the program and the driver are made of ----------------------------
def test_the_program_fits_the_instruction_memory():
    """31 of the 32 words, and the labels the driver addresses by name."""
    program = load_program()
    words = len(program.instrs)
    assert words <= 32, (
        f"{words} words, an engine has 32 (docs/mvp-isa.md `### Engine state`)"
    )
    assert words == RECORDED_WORDS, (
        f"the program is {words} words, recorded {RECORDED_WORDS}: re-measure with "
        f"`python -m fw.swd.measure` and update RECORDED_WORDS"
    )
    drv.check_program(retarget_shaped(program))
    # One `.side_set 1` and no `opt`: the side-set bit is SWCLK's level, so an instruction
    # with no `side` operand is the low half of a bit period.
    assert (program.sideset_bits, program.sideset_opt, program.sideset_pindirs) == (
        1, False, False
    )
    # The wrap window is what turns the receive loop's exit into the `push iffull` that
    # hands the received word to the host, so `execctrl_word` derives it from the labels.
    assert drv.wrap_bottom(program) == program.labels["e1"] + 1
    assert drv.wrap_top(program) == program.labels["rx"] + 2
    assert program.labels["wpre"] > drv.wrap_top(program), (
        "`wpre` has to sit outside the wrap window, or the receive loop's exit would run "
        "into it"
    )


def retarget_shaped(program):
    """The program as `retarget` hands it over, so `check_program` sees the same words."""
    return drv.retarget(program, drv.Config().high, drv.Config().low)


def test_the_driver_addresses_match_the_register_map_and_the_iss():
    """The driver is written against docs/mvp-regmap.md; this guards against the two
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


def test_the_pinctrl_windows_are_one_pin_wide_each():
    """Three pins, three windows, none of them wider than one pin.

    A window two pins wide would let `set pindirs` reach SWCLK or `out pindirs` reach
    FLAG, and the level register of an open-drain line is the wrong thing to write.
    """
    word = drv.pinctrl_word(swdio=0, swclk=13, flag=14)
    assert (word >> 29) & 7 == 1, "SIDESET_COUNT"
    assert (word >> 26) & 7 == 1, "SET_COUNT"
    assert (word >> 20) & 0x3F == 1, "OUT_COUNT"
    assert (word >> 15) & 0x1F == 0, "IN_BASE"
    assert (word >> 10) & 0x1F == 13, "SIDESET_BASE"
    assert (word >> 5) & 0x1F == 14, "SET_BASE"
    assert word & 0x1F == 0, "OUT_BASE"
    assert word == drv.pinctrl_word(0, 13, 14), "the two call styles must agree"
    for bad in ((0, 13, 13), (0, 0, 14), (0, 13, 0)):
        with pytest.raises(ValueError):
            drv.pinctrl_word(*bad)
    with pytest.raises(ValueError):
        drv.pinctrl_word(0, 13, 32)


def test_the_word_packing_round_trips():
    """The three TX words of a transfer, against what the DP expects to read."""
    for addr in (vswd.DP_IDCODE, vswd.DP_CTRL_STAT, vswd.DP_SELECT, vswd.DP_RDBUFF):
        for read in (True, False):
            header = vswd.header_byte(read=read, addr=addr)
            word = drv.request_word(header)
            # `out pindirs, 1` writes a direction, so the OSR bit is the wire bit inverted,
            # and bit 8 is the turnaround, which has to leave the line alone.
            assert word & 0xFF == ~header & 0xFF
            assert word >> 8 == 0, "the turnaround period must release SWDIO"
    for value in (0, 1, 0x8000_0000, 0xFFFF_FFFF, 0x1234_5678):
        assert drv.data_word(True, value) == ~value & 0xFFFF_FFFF
        parity = vswd.data_parity(value)
        closing = drv.closing_word(True, value)
        # The closing word's bit 0 goes out in the first of the three closing periods, and
        # a wire bit of 1 is an OSR bit of 0.
        assert closing & 1 == 1 - parity
        assert closing & 0b110 == 0b110, "a write pulls the line low to close the frame"
    assert drv.closing_word(False) == 0b100, (
        "a read releases the DP's parity bit and its turnaround, then pulls the line low"
    )
    assert drv.data_word(False) == 0, "a read pulls no data word at all"
    assert drv.transfer_words(0xA5, False) == [drv.request_word(0xA5), 0b100]
    assert len(drv.transfer_words(0xA5, True, 7)) == 3
    for bad in (-1, 0x100):
        with pytest.raises(ValueError):
            drv.set_pins_word(bad)
    with pytest.raises(ValueError):
        drv.jmp_abs_word(32)


def test_the_pins_have_to_be_where_open_drain_is():
    """SWDIO has to be a `uio` pin, and the three nets have to be different pins.

    Open drain on this chip is a direction (docs/mvp-isa.md `### Open drain`) and only
    the eight bidirectional pins have one, so an output-only SWDIO cannot be pulled low
    and an input-only one cannot be released.
    """
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    bus.attach(device)
    program = load_program()
    for swdio in (8, 13, 18, -1):
        with pytest.raises(ValueError):
            SwdDriver(bus, device, program, swdio=swdio)
    for swclk, flag in ((13, 13), (0, 0), (13, 0)):
        with pytest.raises(ValueError):
            SwdDriver(bus, device, program, swdio=0, swclk=swclk, flag=flag)


def test_retarget_only_moves_the_delay_field():
    """`retarget` is the one place the driver rewrites the program, so it is guarded.

    HIGH and LOW land in bits 11:8 of every word; nothing else may move, and an
    instruction that carries a delay the driver does not know about is an error rather
    than a silent change of behaviour.
    """
    program = load_program()
    other = drv.retarget(program, 9, 4)
    assert len(other.instrs) == len(program.instrs)
    for a, b in zip(program.instrs, other.instrs):
        assert a & ~(0xF << 8) == b & ~(0xF << 8), f"{a:#06x} -> {b:#06x}"
        assert (b >> 12) & 1 == (a >> 12) & 1, "the side-set bit moved"
    assert other.instrs != program.instrs, "retargeting to different numbers changed nothing"

    # HIGH is the input path: too small and `in pins, 1` samples before the DP's answer.
    for high, low in ((drv.MIN_HIGH - 1, 0), (6, drv.MAX_LOW + 1), (6, -1)):
        with pytest.raises(ValueError):
            drv.retarget(program, high, low)

    # A delay on an instruction no loop accounts for is refused.
    broken = list(program.instrs)
    broken[program.labels["e2"]] |= 0x2 << 8
    with pytest.raises(SwdError):
        drv.check_program(drv.retarget(
            type(program)(program.name, broken, program.origin, program.wrap_target,
                          program.wrap, program.sideset_bits, program.sideset_opt,
                          program.sideset_pindirs, program.labels), 6, 1))


def test_the_fastest_bit_period_still_passes():
    """How fast this program can go, which is the number the README quotes.

    Every bit period is 3 + HIGH + LOW core cycles, and HIGH cannot go below what the
    input path needs: the DP's answer is on the pad two ticks after the rising edge and
    this chip reads SWDIO T_IN = 3 ticks after the pad (docs/mvp-isa.md `## Timing`).
    So the fastest period is 3 + MIN_HIGH + 0 and the fastest SWCLK is 50 MHz over it.
    """
    bench = Bench(**MAX_SPEED).start()
    driver = bench.driver
    driver.line_reset()
    assert driver.read(vswd.DP_IDCODE) == vswd.IDCODE_DEFAULT
    driver.write(vswd.DP_SELECT, 0x0F0F_0F0F)
    assert driver.read(vswd.DP_SELECT) == 0x0F0F_0F0F
    bench.clean()
    assert driver.period == MAX_SPEED_PERIOD, (
        f"period {driver.period}, recorded {MAX_SPEED_PERIOD}"
    )
    assert bench.checker.shortest_period == MAX_SPEED_PERIOD, bench.report()
    assert bench.checker.max_swclk_hz == pytest.approx(MAX_SPEED_HZ), (
        f"{bench.checker.max_swclk_hz / 1e6:.3f} MHz on the wire, recorded "
        f"{MAX_SPEED_HZ / 1e6:.3f} MHz"
    )
    # One core cycle faster is one HIGH short of what the input path allows.
    with pytest.raises(ValueError):
        Bench(**dict(MAX_SPEED, high=MAX_SPEED["high"] - 1)).start()


def test_the_slowest_bit_period_still_passes():
    """The other end of the range: the longest period the 4-bit delay fields reach."""
    bench = Bench(**SLOWEST).start()
    driver = bench.driver
    driver.line_reset()
    assert driver.read(vswd.DP_IDCODE) == vswd.IDCODE_DEFAULT
    assert driver.read(vswd.DP_CTRL_STAT) == bench.target.ctrl_stat()
    bench.clean()
    assert bench.checker.max_swclk_hz == pytest.approx(MIN_SPEED_HZ), bench.report()
    with pytest.raises(ValueError):
        Bench(**dict(SLOWEST, low=SLOWEST["low"] + 1)).start()


def test_a_transfer_needs_its_three_words_in_the_fifo():
    """The host queues the whole transfer before starting it, and the FIFO is 4 deep.

    A write pulls three words, a read two, so both fit with room to spare; what must not
    happen is a `pull block` stalling mid-frame, which FDEBUG.TXSTALL would show.
    """
    assert drv.WORDS_PER_TRANSFER <= drv.FIFO_DEPTH
    bench = Bench(**FAST).start()
    bench.driver.line_reset()
    bench.driver.read(vswd.DP_IDCODE)
    assert bench.driver.tx_level() == 0, "the transfer did not consume the words it queued"
    assert bench.driver.fdebug() == 0, f"FDEBUG {bench.driver.fdebug():#010x}"
    bench.clean()

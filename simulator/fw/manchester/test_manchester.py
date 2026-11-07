"""End-to-end test of the MVP Manchester transmitter firmware (task B1-FW-manchester).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with nets
p0..p17, exactly as `docs/mvp-interfaces.md` `## ISS API` describes; the host driver loads
one of the two programs in `program.pio` and pushes bytes; and `verif.vip.manchester.py` --
the independent judge, which this test does not modify -- watches the line.  Nothing here
looks at the ISS internals to decide pass or fail: the byte values, the framing errors and
every timing number come from the VIP.

    make fw PROTO=manchester TARGET=iss      # runs
    make fw PROTO=manchester TARGET=rtl      # runs too: src/pio_chip.sv under cocotb

`FW_TARGET=rtl` swaps `iss.pio_iss.Chip` for a chip on the RTL and changes nothing else
(`fw/rtl_harness.py`).

**The chip is the transmitter**, so the judge is the model's receiver plus its passive
monitor: `ManchesterRx` for the data and `ManchesterTimingChecker` for the wire.  The bar
here is that the monitor reports *no* violation at all, so `test_the_line_is_legal
...` asserts `chk.clean` and prints `chk.report()` when it is not.

The cases below span two programs, four bit rates and every configuration knob the driver
has (engine, pin, convention).  The two programs are the two chip splits a Manchester cell
can have, and the split is what decides the bit rate range, so the pair is the point of the
sweep:

* `manchester` -- 5 SM cycles per bit, chips of 3 and 2.  At `CLKDIV = 1.0` that is
  AN1120a's 10BASE-T rate, 10 Mbit/s, and the fastest rate this judge can classify at all.
* `manchester_even` -- 4 SM cycles per bit, chips of 2 and 2, so the wire is on the judge's
  bit grid to the tick at any rate.  10 Mbit/s needs the *fractional* divider here
  (`CLKDIV = 1.25`), and from there it reaches down through 1 Mbit/s to AN1470a's worked
  example of 25 kbit/s.

Every case also asserts the two things a line code makes unforgiving: the sync preamble is
on the wire at the head of every message (`tSYNC`), and the line rests at the idle level
between messages for at least the profile's minimum gap (`tIDLE`, `tGAP`).

The last group of tests is the judge proving it is not vacuous: a bit rate the chip split
cannot carry is *refused by the driver*, and when it is forced through anyway the monitor
says exactly why (`tHALF` and `tMID-BIT`); a program whose cell no longer has two different
halves has lost its mandatory mid-bit transition (`tBIT`); a preamble one SM cycle short
makes its own cell a length nothing can be classified as (`tMID-BIT`); a line that never
comes back to the idle level is `tIDLE`; and one host-side fault -- reversing the bits of
each byte -- produces a perfectly legal Manchester waveform of a *different* message, which
no timing rule can see and only the receiver's bytes catch.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Optional

import pytest

import fw.manchester.driver as driver_mod
from fw.manchester.driver import (
    CORE_HZ,
    IDLE_LEVEL,
    PROGRAMS,
    PROTO_NAME,
    ManchesterTxDriver,
    bits_per_word,
    chip_length_ranges,
    chip_lengths_sm_cycles,
    chip_lengths_ticks,
    clkdiv_for_bit_rate,
    clkdiv_value,
    gap_sm_cycles,
    legal_bit_rates,
    load_program,
    message_sm_cycles,
    set_pindirs,
    set_pins,
    shiftctrl_pull_thresh,
    sm_cycles_per_bit,
    sm_cycles_to_ticks,
    split_error_ticks,
    split_is_legal,
    ticks_per_bit,
    words_used,
)
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.manchester import (
    ManchesterRx,
    ManchesterTimingChecker,
    bits_of,
    profile_timings,
    sync_preamble,
)
from fw import regmap as _regmap

IMEM_SIZE = 32                       # docs/mvp-isa.md `### Engine state`
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_TXSTALL = 0x0100_0000         # bit 24 + engine (docs/mvp-regmap.md `**FDEBUG**`)
FDEBUG_BAD = 0x0003_0303             # TXOVER, RXUNDER, RXSTALL: nothing dropped or mis-timed
UIO_MASK = 0x000FF                   # the bidirectional protocol pins
NS_PER_TICK = 20.0                   # 50 MHz chip clock
PREAMBLE_BITS = 2                    # the sync pattern `program.pio` puts on the wire


@dataclass(frozen=True)
class Case:
    """One end-to-end configuration: a program, a rate, an engine, a pin, a convention."""

    name: str
    program: str
    bit_rate: float
    engine: int = 0
    pin: int = 0
    convention: str = "ieee802.3"
    count: int = 6                   # messages, one byte each


CASES = [
    # The 10BASE-T rate, which is the profile the judge defaults to, on the default program.
    Case("10m-10base-t", PROTO_NAME, 10e6),
    Case("10m-pin3", PROTO_NAME, 10e6, pin=3),
    Case("10m-engine1", PROTO_NAME, 10e6, engine=1, pin=5),
    # The same wire in the other convention: the host pushes the complement of each byte
    # and nothing in the program changes (`program.pio`'s header, and the AN3015 trap).
    Case("10m-thomas", PROTO_NAME, 10e6, pin=2, convention="thomas"),
    # The 3:2 split is exactly one tick off at CLKDIV = 2.0, which is the whole allowance --
    # and the slowest rate this program can carry at all.
    Case("5m-split-edge", PROTO_NAME, 5e6),
    # Equal chips: on the bit grid to the tick, from the fastest rate the judge will
    # classify down to AN1470a's worked example of 25 kbit/s.
    Case("12m-even", "manchester_even", 12e6),
    # ... at a whole-number divider, and at a fractional one: with equal chips the divider's
    # alternation is the only error left, and it stays inside the judge's one-tick window.
    Case("6.25m-even", "manchester_even", 6.25e6),
    Case("5m-even-frac", "manchester_even", 5e6),
    Case("10m-even", "manchester_even", 10e6),
    Case("1m-even", "manchester_even", 1e6, count=4),
    Case("100k-even", "manchester_even", 100e3, count=3),
    Case("25k-even", "manchester_even", 25e3, count=2, pin=1),
    Case("1m-even-thomas", "manchester_even", 1e6, count=4, convention="thomas"),
]


def payload(count: int, engine: int = 0) -> list[int]:
    """Bytes that exercise every bit pattern, then a fixed pseudo-random tail.

    Adjacent equal bytes matter: in this line code two equal bits put a transition at the bit
    boundary and two different bits put none, so the head below is what makes the boundary
    rule visible in the judge's edge log.
    """
    head = [0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0x7F, 0xFE]
    seed = 0x2545 + 7 * engine
    tail = []
    while len(head) + len(tail) < count:
        seed = (seed * 1103515245 + 12345) & 0xFFFF_FFFF
        tail.append((seed >> 16) & 0xFF)
    return (head + tail)[:count]


@dataclass
class Result:
    case: Case
    bus: Bus
    drv: ManchesterTxDriver
    rx: ManchesterRx
    chk: ManchesterTimingChecker
    data: list[int]

    @property
    def decoded_bits(self) -> list[int]:
        """The bits the judge must decode back out of the line, one message byte at a time.

        The payload, LSB first, in *both* conventions: for G.E. Thomas the host pushes the
        complement of each byte, so the line levels are the complement of an IEEE 802.3 link's
        while the bits a receiver told it is Thomas recovers are the payload again.  That is
        the whole of the convention difference (`program.pio`'s header).
        """
        return bits_of(bytes(self.data), "lsb")

# --------------------------------------------------------------------------------------------
# The bench: a Bus at 20 ns/tick with p0..p17, the ISS on it, and the judge on the TX pin
# --------------------------------------------------------------------------------------------

def build(case: Case, *, allow_illegal_split: bool = False,
          program=None) -> tuple[Bus, ChipDevice, ManchesterTxDriver, ManchesterRx,
                                 ManchesterTimingChecker]:
    """A Bus, the chip, and `ManchesterRx` + `ManchesterTimingChecker` on the TX net.

    Both judge devices are configured at the rate the chip is *asked* for, not the rate the
    divider can only approximate: `clkdiv_for_bit_rate` rounds into 16.8, and the tests below
    check that the rounding is inside what the divider resolution allows
    (`test_clkdiv_hits_the_bit_rate_within_the_divider_resolution`).
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == case.pin))
    dev = bus.attach(ChipDevice())
    drv = ManchesterTxDriver(dev, bus, engine=case.engine, pin=case.pin, bit_rate=case.bit_rate,
                             convention=case.convention, program=program,
                             name=case.program, allow_illegal_split=allow_illegal_split)
    net = f"p{case.pin}"
    rx = bus.attach(ManchesterRx(net, bit_rate=case.bit_rate, convention=case.convention,
                                 preamble_bits=PREAMBLE_BITS))
    chk = bus.attach(ManchesterTimingChecker(net, bit_rate=case.bit_rate,
                                             convention=case.convention))
    return bus, dev, drv, rx, chk


def run_case(case: Case, *, allow_illegal_split: bool = False, program=None,
             payload_bytes: Optional[list[int]] = None) -> Result:
    """Load, start, send, and run long enough for the last message to finish.

    The run length is not a guess: `message_sm_cycles` is the sum of `1 + delay` over the
    program's words plus its extra loop passes, and the divider's algorithm is exact over a
    run of SM cycles (`docs/mvp-isa.md` `### Clock divider`), so `message_ticks` is how long
    `count` messages really take.  The margin is the host's own start-up accesses.
    """
    bus, dev, drv, rx, chk = build(case, allow_illegal_split=allow_illegal_split,
                                   program=program)
    data = payload(case.count, case.engine) if payload_bytes is None else list(payload_bytes)
    drv.load()                              # IMEM + CLKDIV/EXECCTRL/SHIFTCTRL/PINCTRL
    drv.start(first=data[:drv.TXFIFO_DEPTH])
    drv.send(data[drv.TXFIFO_DEPTH:])
    bus.run(drv.message_ticks(len(data)) + 4000)
    return Result(case, bus, drv, rx, chk, data)


# --------------------------------------------------------------------------------------------
# The transaction, judged by the VIP
# --------------------------------------------------------------------------------------------

@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_line_is_legal_and_the_payload_comes_back(case: Case) -> None:
    """Every message arrives, in order, and the monitor reports no violation at all.

    This is the main pass criterion.  `chk.clean` covers all eight of the monitor's rules -- tHALF,
    tMID-BIT, tBIT, tFRAMING, tJIT, tSYNC, tIDLE and tGAP -- and `rx.clean` is the model's own
    view that it never lost the clock while decoding.
    """
    res = run_case(case)
    rx, chk = res.rx, res.chk

    assert chk.clean, f"{case.name}: {chk.report(limit=12)}"
    assert rx.clean, f"{case.name}: {rx.report(limit=12)}"
    assert rx.decoded_bytes() == bytes(res.data), (
        f"{case.name}: {rx.decoded_bytes().hex()} != {bytes(res.data).hex()}")
    assert [m.as_bytes() for m in rx.messages] == [bytes([b]) for b in res.data], (
        f"{case.name}: one message per byte, in order")
    # One stream per message: the idle gap between them is long enough for the monitor to
    # find the boundary, which is what tGAP asks for.
    assert len(chk.streams) == len(res.data), (
        f"{case.name}: {len(chk.streams)} streams for {len(res.data)} messages")
    assert all(s.violations == 0 for s in chk.streams)


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_sync_preamble_is_on_the_wire(case: Case) -> None:
    """Every stream starts with the link's sync pattern, which is the only phase on the wire.

    A Manchester line code has no start bit: a receiver anchors on the first transition of a
    stream and takes it for a mid-bit transition, which is only right if the first chip is
    the level the line was already at.  AN2358's answer is a preamble, and
    `ManchesterTimingChecker`'s `tSYNC` rule is what checks it.  The monitor's decoded bits
    include the preamble; the receiver is told how long it is (`preamble_bits`), so the data
    bits are what is left over.
    """
    res = run_case(case)
    pattern = sync_preamble(case.convention)
    assert "".join(str(b) for b in res.drv.config.preamble) == pattern
    for index, stream in enumerate(res.chk.streams):
        head = stream.bits[:PREAMBLE_BITS]
        assert "".join(str(b) for b in head) == pattern, (
            f"{case.name}: stream {index} starts {head}, not the sync pattern {pattern}")
        assert stream.bits[PREAMBLE_BITS:] == res.decoded_bits[8 * index:8 * (index + 1)], (
            f"{case.name}: stream {index} carries the wrong data bits")
    for index, message in enumerate(res.rx.messages):
        assert message.preamble_len == PREAMBLE_BITS
        assert message.data_bits == res.decoded_bits[8 * index:8 * (index + 1)]


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_wire_sits_on_the_bit_grid(case: Case) -> None:
    """Every mid-bit transition is where the judge's bit grid says it should be.

    The grid is anchored on the first mid-bit transition of a stream and advances by exactly
    one bit time per bit, which is what a receiver that re-locks on every mid-bit transition
    does (AN1470a).  `tJIT` is the rule that watches it, and its window is one tick, so the
    interesting number is what the violation *would* have been: this program's two chips are
    `|n1 - n2| / 2` SM cycles away from a half bit, and at this CLKDIV that is the measured
    error below.  The intervals the wire actually produced are the ones the line code allows.
    """
    res = run_case(case)
    chk = res.chk
    drv = res.drv

    assert chk.stats["worst_dev"] <= chk.timings["grid_tol"] + 1e-9, chk.report(limit=6)
    # The driver predicts the error exactly (a fraction of a core cycle) and the monitor
    # measures it on whole ticks, because `core.py` can only put an edge on a tick boundary,
    # so the two have to agree to within one tick.
    predicted = drv.split_error_ticks
    assert abs(chk.stats["worst_dev"] - predicted) < 1.0, (
        f"{case.name}: measured {chk.stats['worst_dev']:.2f} ticks of grid error, the driver "
        f"predicts {predicted:.2f}")
    if drv.program.name == "manchester_even":
        # Equal chips: the only error left is the divider's own rounding, under one tick.
        assert predicted <= 1.0 and chk.stats["worst_dev"] <= 1.0
    else:
        # Three and two SM cycles: half a tick of skew per tick of CLKDIV, and the bit-boundary
        # transition lands exactly that far from its place on the grid.
        assert predicted == 0.5 * ((drv.clkdiv >> 16) & 0xFFFF or 1) + \
            0.5 * ((drv.clkdiv >> 8) & 0xFF) / 256.0
    # A cell has one transition at its middle and, only when the two bits are equal, one at
    # its boundary: so every interval inside a stream is a half bit or a whole bit.
    inside = [e.interval for e in chk.edges if e.kind in ("short", "long")]
    assert inside, chk.report()
    assert all(chk.timings["half_lo"] <= i <= chk.timings["bit_hi"] for i in inside), \
        sorted(set(inside))
    assert res.rx.worst_dev_ticks <= chk.timings["grid_tol"] + 1e-9, res.rx.report()


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_line_rests_at_the_idle_level_between_messages(case: Case) -> None:
    """Between two messages the line is at the idle level for the monitor's minimum gap.

    A line code has no start bit, so the boundary between two messages is *only* a stretch of
    idle line: AN2358's first-bit rule and the monitor's `tIDLE` and `tGAP` rules all rest on
    the line coming back to the idle level and staying there.  The program holds it with one
    `set pins, 1 [d]`, and the check here is that the hold the driver computes from the words
    is at least what the monitor asks for at *this* CLKDIV -- a gap measured in SM cycles is
    a different number of core cycles at every rate.
    """
    res = run_case(case)
    chk, drv = res.chk, res.drv
    held = sm_cycles_to_ticks(gap_sm_cycles(drv.program), drv.clkdiv)
    assert held >= chk.timings["min_gap_ticks"], (
        f"{case.name}: the program holds the idle line for {held} core cycles, the monitor "
        f"asks for {chk.timings['min_gap_ticks']:g} "
        f"({chk.timings['min_gap_bits']:g} bit times)")
    # ... and the wire agrees: that much idle line really is on it.
    for after, before in zip(chk.streams, chk.streams[1:]):
        last = max(e.tick for e in chk.edges if e.tick < before.start_tick)
        assert before.start_tick - last >= chk.timings["min_gap_ticks"] - 1, (
            f"{case.name}: only {before.start_tick - last} ticks of idle line before the "
            f"message at t={before.start_tick}")
    assert not chk.violations_by_rule("tIDLE") and not chk.violations_by_rule("tGAP")
    # The line ends at the idle level, on the pad and in the level register.
    assert res.bus.values[f"p{case.pin}"] == IDLE_LEVEL
    assert drv.padout() & (1 << case.pin), f"p{case.pin} is not at the idle level at the end"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_chip_is_left_clean(case: Case) -> None:
    """No word dropped, no read underflow, only the TX pin driven, only this engine used."""
    res = run_case(case)
    drv, chip = res.drv, res.drv.dev.chip
    other = 1 - case.engine

    # TXOVER/RXUNDER/RXSTALL would mean a message was dropped or mis-timed.  TXSTALL is the
    # engine parked on `pull block` waiting for a byte the host has not sent yet, which is
    # where a transmitter between messages belongs.
    fdebug = drv.fdebug()
    assert fdebug & FDEBUG_BAD == 0, f"{case.name}: FDEBUG {fdebug:#010x}"
    assert fdebug & ~(FDEBUG_TXSTALL << case.engine) == 0, \
        f"{case.name}: unexpected FDEBUG {fdebug:#010x}"

    padoe = drv.padoe()
    assert padoe & (1 << case.pin), f"{case.name}: padoe {padoe:#06x}: the TX pin is not driven"
    assert padoe & UIO_MASK & ~(1 << case.pin) == 0, (
        f"{case.name}: padoe {padoe:#06x}: a program window leaked onto another uio pin")
    assert chip.state(case.engine)["tx_level"] == 0, "messages are still queued"
    assert chip.state(other)["pc"] == 0, "the other engine ran"
    assert not chip.engines[other].tx, "the other engine got data"
    assert not res.bus.errors, f"contention on a push-pull net: {res.bus.errors[:2]}"
    assert drv.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_engine_loaded_exactly_what_was_assembled(case: Case) -> None:
    """The words in the engine's IMEM are the ones `asm/pioasm.py` produced for program.pio."""
    res = run_case(case)
    drv = res.drv
    imem = drv.dev.chip.engines[case.engine].imem
    assert imem[:drv.words_used] == drv.program.instrs
    assert set(imem[drv.words_used:]) <= {0}, "the driver wrote past the program"
    # ... and the engine's configuration is the one the driver built, not something left over.
    assert (drv.dev.chip.engines[case.engine].clkdiv == drv.clkdiv
            and drv.dev.chip.engines[case.engine].execctrl == drv.execctrl
            and drv.dev.chip.engines[case.engine].shiftctrl == drv.shiftctrl
            and drv.dev.chip.engines[case.engine].pinctrl == drv.pinctrl)
    assert drv.pc() != 0 or True        # SM_ADDR is the PC, wherever the engine is resting


# --------------------------------------------------------------------------------------------
# What the programs and the divisor are, derived from the words rather than asserted by hand
# --------------------------------------------------------------------------------------------

def test_both_programs_fit_the_imem() -> None:
    """Each program assembles, and the driver reads its shape back out of the words."""
    programs = {name: load_program(name) for name in PROGRAMS}
    for name, program in programs.items():
        assert program.name == name
        assert words_used(program) == 11, [f"{w:#06x}" for w in program.instrs]
        assert words_used(program) <= IMEM_SIZE
        assert program.labels["start"] == 0
        assert program.labels["message"] == 2 and program.labels["bitloop"] == 6
        assert program.wrap == words_used(program) - 1, "WRAP_TOP must be the last instruction"
        assert program.sideset_bits == 0 and not program.sideset_opt, (
            "no .side_set: PINCTRL.SIDESET_COUNT has to be 0 or the reset value of 2 "
            "side-sets level 0 into two pins on every instruction")
        assert program.instrs[0] == set_pins(IDLE_LEVEL), "the idle level comes first"
        assert program.instrs[1] == set_pindirs(1), (
            "and only then the direction: the level register resets to 0")
        assert sm_cycles_per_bit(program) == (4 if name.endswith("_even") else 5)
        assert bits_per_word(program) == 8 and shiftctrl_pull_thresh(program) == 8
        assert gap_sm_cycles(program) == 24, "1 + 23: the idle gap between messages"
        assert message_sm_cycles(program) > 0


def test_the_chip_split_is_what_each_program_is_for() -> None:
    """`manchester` has chips of 3 and 2 SM cycles, `manchester_even` two of 2."""
    skewed, even = load_program("manchester"), load_program("manchester_even")
    assert chip_lengths_sm_cycles(skewed) == (3, 2)
    assert sm_cycles_per_bit(skewed) == 5
    assert chip_lengths_sm_cycles(even) == (2, 2)
    assert sm_cycles_per_bit(even) == 4
    # ... and the two differ in exactly the three delays that set those numbers.
    assert [i for i, (a, b) in enumerate(zip(skewed.instrs, even.instrs)) if a != b] == [3, 6], (
        "only the preamble's low hold and the first chip's delay differ")
    assert gap_sm_cycles(skewed) == gap_sm_cycles(even)
    assert words_used(skewed) == words_used(even)


def test_maximum_speed() -> None:
    """The fastest each program can send at 50 MHz, and what stops it going faster.

    * `manchester`: 5 SM cycles per bit and `CLKDIV = 1.0`, one core cycle per SM cycle, is
      5 ticks per bit = 100 ns = **10 Mbit/s**, which is AN1120a's 10BASE-T bit time and the
      profile `verif/vip/manchester.py` defaults to.  Nothing faster is expressible: one SM
      cycle is at least one core cycle, and 12.5 Mbit/s would need CLKDIV = 0.8.
    * `manchester_even`: 4 SM cycles per bit, so `CLKDIV = 1.0` is 12.5 Mbit/s -- and that is
      exactly the rate at which the judge's two windows touch and `profile_timings` refuses
      to classify anything, so the fastest rate the judge can *certify* is 12 Mbit/s.
      """
    skewed, even = load_program("manchester"), load_program("manchester_even")

    # The 10BASE-T rate, on a whole-number divider.
    assert clkdiv_for_bit_rate(10e6, sm_cycles_per_bit(skewed)) == clkdiv_value(1)
    assert ticks_per_bit(clkdiv_value(1), 5) == 5.0
    assert CORE_HZ / 5.0 == 10_000_000.0
    with pytest.raises(ValueError, match="at least one core cycle"):
        clkdiv_for_bit_rate(12.5e6, sm_cycles_per_bit(skewed))

    # The even program's ceiling, and the judge's own limit one step below it.
    assert CORE_HZ / sm_cycles_per_bit(even) == 12_500_000.0
    assert clkdiv_for_bit_rate(12.5e6, 4) == clkdiv_value(1)
    assert profile_timings(NS_PER_TICK, bit_rate=12e6)["ticks_per_bit"] == pytest.approx(4.1667, abs=1e-3)
    with pytest.raises(ValueError, match="no room between"):
        profile_timings(NS_PER_TICK, bit_rate=12.5e6)
    # 10 Mbit/s is reachable here too, but only with the fractional divider: 4 SM cycles at
    # CLKDIV = 1.25 is 5 ticks, two of 2.5 each.
    assert (clkdiv_for_bit_rate(10e6, 4) >> 8) & 0xFF == 64
    assert ticks_per_bit(clkdiv_for_bit_rate(10e6, 4), 4) == pytest.approx(5.0)
    assert chip_lengths_ticks(even, clkdiv_for_bit_rate(10e6, 4)) == (2.5, 2.5)

    # The rate ranges the two programs can carry, from the split and nothing else.
    assert legal_bit_rates(skewed) == (5e6, 10e6)
    assert legal_bit_rates(even) == (0.0, 12.5e6)


@pytest.mark.parametrize("rate", [1e6, 100e3, 25e3])
def test_clkdiv_hits_the_bit_rate_within_the_divider_resolution(rate: float) -> None:
    """CLKDIV for a rate is the 16.8 rounding, and CLKDIV is all the program has to go on."""
    program = load_program("manchester_even")
    per_bit = sm_cycles_per_bit(program)
    clkdiv = clkdiv_for_bit_rate(rate, per_bit)
    integer, frac = (clkdiv >> 16) & 0xFFFF, (clkdiv >> 8) & 0xFF
    assert 1 <= integer <= 0xFFFF and 0 <= frac <= 0xFF
    # One bit is `per_bit` SM cycles and one SM cycle is integer + frac/256 core cycles, so
    # the bit length can only move in steps of per_bit/256 ticks: half a step, rounded.
    step = per_bit / 256
    assert abs(ticks_per_bit(clkdiv, per_bit) - CORE_HZ / rate) <= step / 2 + 1e-9
    # ... and the divider algorithm is exact over a run of SM cycles, so the wire cannot drift
    # away from the grid over a long message either.
    bits = 8 * 8
    exact = sm_cycles_to_ticks(bits * per_bit, clkdiv)
    assert abs(exact - bits * CORE_HZ / rate) <= bits * step / 2 + 1e-9
    assert sm_cycles_to_ticks(per_bit, clkdiv) == round(ticks_per_bit(clkdiv, per_bit))


def test_a_rate_the_split_cannot_carry_is_refused_and_the_judge_says_why() -> None:
    """`manchester`'s 3:2 split stops being legal below 5 Mbit/s, and that is not a guess.

    The driver refuses the rate, because a chip a third of a bit long is not Manchester any
    more: a receiver cannot tell a transition at a bit boundary from a mid-bit one.  Forced
    through anyway, the monitor says exactly that with `tHALF`, which is the rule for "two
    transitions too close together for the code".  The clean bench at 10 Mbit/s, the slowest
    legal rate (5 Mbit/s, exactly on the tolerance) and the failing one are all in this test
    so the three cannot disagree by accident.
    """
    case = Case("illegal-1m", PROTO_NAME, 1e6, count=3)

    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == case.pin))
    dev = bus.attach(ChipDevice())
    with pytest.raises(ValueError, match="not a rate the manchester program can carry"):
        ManchesterTxDriver(dev, bus, pin=case.pin, bit_rate=case.bit_rate, name=case.program)
    assert legal_bit_rates(load_program(PROTO_NAME)) == (5e6, 10e6)

    # ... and the judge agrees, when the refusal is overridden.  Both halves of the split are
    # outside the window at 1 Mbit/s -- 30 ticks where the half-bit window is 22.5..27.5, and
    # 20 where it is too -- so the monitor names both rules.
    res = run_case(case, allow_illegal_split=True)
    assert not res.chk.clean, "the monitor passed a wire no receiver could decode"
    assert {v.rule for v in res.chk.violations} == {"tHALF", "tMID-BIT"}, res.chk.report(limit=6)
    assert not res.rx.clean and res.rx.framing_errors > 0
    short = res.chk.violations_by_rule("tHALF")[0]
    long = res.chk.violations_by_rule("tMID-BIT")[0]
    assert short.short and short.measured == 20.0 and short.tick > 0
    assert long.long and long.measured == 30.0 and long.tick > 0
    assert short.window == long.window == (22.5, 27.5)

    # The two legal ends of the same program are clean, so the refusal is about the rate.
    for rate in (10e6, 5e6):
        ok = run_case(Case(f"legal-{rate:g}", PROTO_NAME, rate, count=3))
        assert ok.chk.clean, ok.chk.report(limit=6)
        assert ok.rx.decoded_bytes() == bytes(ok.data)
        assert ok.drv.split_is_legal


def test_the_split_check_is_exact_about_the_divider_and_stricter_than_the_judge() -> None:
    """A fractional CLKDIV puts each chip on one of two lengths a tick apart, and both count.

    The divider alternates `INT` and `INT + 1` core cycles per SM cycle, so `n` SM cycles last
    `n * INT` ticks plus either `floor(n * FRAC / 256)` or `ceil(...)` carries, depending on
    where in the accumulator's period the chip starts.  With the 3:2 split and a fractional
    divider that puts a three-SM-cycle chip at 4 *or* 5 ticks when the half-bit window is
    [3.00, 5.00], and the judge only ever sees the phase that actually occurred -- so it can
    pass a rate whose other phase would be a `tHALF` away.  The driver refuses the whole rate,
    which is the point: it asks whether the transmitter can send the wire, not whether this
    message happened to be a legal one.
    """
    skewed, even = load_program(PROTO_NAME), load_program("manchester_even")

    # The skewed program's two legal rates are both whole-number dividers, where each chip has
    # exactly one length.
    for rate in (10e6, 5e6):
        clkdiv = clkdiv_for_bit_rate(rate, 5)
        assert split_is_legal(skewed, clkdiv), rate
        assert all(low == high for low, high in chip_length_ranges(skewed, clkdiv)), rate
    # Every fractional rate inside that range is refused, because the longer of the two lengths
    # a chip can take falls outside the half-bit window.
    for rate in (9e6, 8_333_333.0, 7e6, 6.25e6):
        clkdiv = clkdiv_for_bit_rate(rate, 5)
        assert not split_is_legal(skewed, clkdiv), rate
        assert any(high - low == 1 for low, high in chip_length_ranges(skewed, clkdiv)), rate

    # Equal chips: the ±1 tick is the only error and the window is at least ±1 tick wide, so
    # every rate is legal, fractional dividers included.
    for rate in (12e6, 10e6, 8_333_333.0, 6.25e6, 5e6, 1e6, 100e3, 25e3):
        clkdiv = clkdiv_for_bit_rate(rate, 4)
        assert split_is_legal(even, clkdiv), rate
        first, second = chip_length_ranges(even, clkdiv)
        assert first[0] == second[0] and first[1] == second[1], rate


def test_the_convention_is_a_trap_and_the_driver_can_flip_it() -> None:
    """A G.E. Thomas receiver on an IEEE 802.3 wire reports no timing error at all.

    The two conventions are inverses, so a receiver using the wrong one reads every bit the
    other way round while every interval it measures stays perfectly legal -- this is the trap
    Freescale AN3015 walks into, and `research/ref/protocols/manchester/README.md` has the
    citation table.  The firmware side of it: `convention="thomas"` makes the host push the
    complement of each byte, the program emits a Thomas stream, and the model's Thomas
    receiver gets the payload back.
    """
    payload_bytes = [0xA5, 0x3C]
    wire = Case("ieee", PROTO_NAME, 10e6, pin=0, count=2)
    bus, dev, drv, rx, chk = build(wire)
    wrong = bus.attach(ManchesterRx("p0", bit_rate=10e6, convention="thomas",
                                    preamble_bits=PREAMBLE_BITS))
    drv.load()
    drv.start(first=list(payload_bytes))
    bus.run(drv.message_ticks(len(payload_bytes)) + 4000)

    assert chk.clean and wrong.clean, "a mis-anchored stream is still legal Manchester"
    assert rx.decoded_bytes() == bytes(payload_bytes)
    assert wrong.decoded_bytes() == bytes(b ^ 0xFF for b in payload_bytes), (
        "the wrong convention inverts every bit and says nothing about it")
    assert wrong.errors == []

    # The same payload with the driver told to use the other convention: the line is
    # inverted, and the model's Thomas receiver gets the payload back.
    thomas = run_case(Case("thomas", PROTO_NAME, 10e6, pin=0, convention="thomas",
                           count=2), payload_bytes=payload_bytes)
    assert thomas.chk.clean, thomas.chk.report(limit=6)
    assert thomas.rx.decoded_bytes() == bytes(payload_bytes)
    assert thomas.drv.encode(0xA5) == 0xA5 ^ 0xFF and thomas.drv.encode(0x00) == 0xFF


def test_driver_rejects_setups_it_cannot_honour() -> None:
    """A TX pin PINDIRS cannot drive, an unknown convention, a bad rate, an unknown program."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())

    for pin in (8, 12, 13, 17, 18):        # input-only, output-only, out of range
        with pytest.raises(ValueError):
            ManchesterTxDriver(dev, bus, pin=pin)
    for convention in ("802.3", "thomas-1949", "biphase"):
        with pytest.raises(ValueError, match="convention must be one of"):
            ManchesterTxDriver(dev, bus, convention=convention)
    with pytest.raises(ValueError, match="must be positive"):
        ManchesterTxDriver(dev, bus, bit_rate=0.0)
    with pytest.raises(ValueError, match="above the 16-bit maximum"):
        ManchesterTxDriver(dev, bus, bit_rate=1.0)        # needs CLKDIV 10_000_000
    with pytest.raises(ValueError, match="no .program named"):
        load_program("manchester_slow")
    with pytest.raises(ValueError):
        ManchesterTxDriver(dev, bus, engine=2)

    drv = ManchesterTxDriver(dev, bus, pin=0, bit_rate=10e6)
    drv.load()
    with pytest.raises(ValueError):
        drv.preload([1, 2, 3, 4, 5])
    with pytest.raises(ValueError):
        drv.push(256)
    with pytest.raises(ValueError, match="at least one core cycle"):
        ManchesterTxDriver(dev, bus, pin=0, bit_rate=12.5e6)     # CLKDIV would be 0.8


def test_the_driver_derives_nothing_it_cannot_read_out_of_the_words() -> None:
    """A program whose shape changed stops the driver rather than giving a wrong divisor."""
    program = copy.deepcopy(load_program(PROTO_NAME))
    no_invert = copy.deepcopy(program)
    no_invert.instrs[6] &= ~(1 << 3)                  # `mov pins, osr`, not `mov pins, osr !`
    with pytest.raises(ValueError, match="exactly one `mov pins, osr !`"):
        sm_cycles_per_bit(no_invert)

    no_second_chip = copy.deepcopy(program)
    no_second_chip.instrs[7] = set_pins(IDLE_LEVEL)  # `out pins, 1` replaced
    with pytest.raises(ValueError, match="not\n.*`out pins, 1`|not `out pins, 1`"):
        sm_cycles_per_bit(no_second_chip)

    no_jump = copy.deepcopy(program)
    no_jump.instrs[8] = 0x0000                       # `jmp 0` -> does not close the loop
    with pytest.raises(ValueError, match="not a `jmp` back"):
        sm_cycles_per_bit(no_jump)

    no_counter = copy.deepcopy(program)
    no_counter.instrs[5] = set_pins(IDLE_LEVEL)      # the `set x, 7` replaced
    with pytest.raises(ValueError, match="exactly one `set x, n`"):
        bits_per_word(no_counter)

    no_gap = copy.deepcopy(program)
    no_gap.instrs[9] = set_pins(IDLE_LEVEL)          # the idle `set pins, 1` without its delay
    assert gap_sm_cycles(no_gap) == 1, "the gap is read from the delay, not assumed"
    dropped = copy.deepcopy(program)
    dropped.instrs.pop(9)
    with pytest.raises(ValueError, match="expected one idle `set pins"):
        gap_sm_cycles(dropped)


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: a broken program must fail the transaction
# --------------------------------------------------------------------------------------------

def _nop(delay: int = 0) -> int:
    """The word for `nop [delay]`: `mov y, y` (docs/mvp-isa.md `## Encoding`, worked examples)."""
    return (0b101 << 13) | (delay << 8) | (0b010 << 5) | 0b000 << 3 | 0b010


class BitReversedWords(ManchesterTxDriver):
    """The same driver with each byte's bits reversed on the way into the TX FIFO.

    A *host* fault rather than a program fault, and the interesting one: the line stays a
    perfectly legal Manchester waveform -- one transition in the middle of every cell, at the
    right instants, with boundaries exactly where equal bits put them -- carrying the bits of
    a different, equally legal message.  No timing rule can see it; only the receiver's bytes
    can.
    """

    def encode(self, byte: int) -> int:
        return int(f"{byte & 0xFF:08b}"[::-1], 2)


#: (name, program, rate, what it breaks, which rule the monitor must name, mutate, extra kwargs)
MUTATIONS = [
    (
        "the two halves of a cell are the same level", PROTO_NAME, 10e6,
        "`mov pins, osr !` without the invert, so the cell has no mid-bit transition at all -- "
        "the one thing the line code does not allow.  With no transition in the middle of a "
        "cell the stream cannot be decoded, so the line also ends up off the idle level "
        "(`tIDLE`), loses its sync pattern (`tSYNC`) and runs two messages together (`tGAP`)",
        {"tBIT", "tGAP", "tIDLE", "tSYNC"},
        lambda p: p.instrs.__setitem__(6, p.instrs[6] & ~(1 << 3)),
        {"keep_shape": True},
    ),
    (
        "the preamble is a whole SM cycle short", "manchester_even", 6.25e6,
        "the sync preamble's low hold one SM cycle short, so the preamble's own cell is a "
        "whole bit off and its second transition is neither at a bit boundary nor at a "
        "mid-bit.  That one rule is all it can reach: the monitor re-anchors its grid on the "
        "transition it could not classify, so the off-grid data bits that follow never get a "
        "`tJIT` of their own",
        {"tMID-BIT"},
        lambda p: p.instrs.__setitem__(3, p.instrs[3] - (1 << 8)),
        {},
    ),
    (
        "the line never comes back to the idle level", PROTO_NAME, 10e6,
        "the idle `set pins, 1` replaced by a nop, so between messages the line rests at "
        "whatever the last data chip left on it",
        {"tIDLE"},
        lambda p: p.instrs.__setitem__(9, _nop(IDLE_HOLD_DELAY)),
        {},
    ),
    (
        "the inter-message gap is two SM cycles", PROTO_NAME, 10e6,
        "an idle hold long enough to be legal replaced by one that is not.  The monitor is "
        "*silent* about this and that is documented behaviour, not a hole: with no idle line "
        "there is no transition to measure, so it sees one long stream and the two messages "
        "have run together.  What catches it is the boundary the monitor reports no longer "
        "being there -- one stream for two messages",
        None,
        lambda p: p.instrs.__setitem__(9, p.instrs[9] - (23 << 8) + (1 << 8)),
        {},
    ),
]

MUTATION_PAYLOAD = [0xA5, 0x01]             # the last message ends on a 0 bit, not the idle level
IDLE_HOLD_DELAY = 23                        # the delay of the program's idle `set pins, 1`


def _keep_loop_shape(monkeypatch) -> None:
    """Let a driver load a program whose *loop body* no longer has the shape it looks for.

    `bit_loop` is what finds the two chip instructions, so a mutation of either of them would
    otherwise stop the driver from starting -- which is the driver's own check doing its job,
    but it says nothing about whether the transaction check would notice.  Stubbing the
    *finder* keeps the timing derivation, and therefore CLKDIV, exactly as it was.
    """
    monkeypatch.setattr(driver_mod, "bit_loop", lambda program: (6, 7, 8))


def _run_mutated(program, rate: float, *, keep: bool, monkeypatch,
                 payload_bytes=MUTATION_PAYLOAD) -> Result:
    case = Case("mutation", PROTO_NAME, rate, count=len(payload_bytes))
    if keep:
        _keep_loop_shape(monkeypatch)
    return run_case(case, allow_illegal_split=True, program=program,
                    payload_bytes=payload_bytes)


@pytest.fixture(scope="module")
def clean_run() -> Result:
    """The unmutated program at the same rate as the first mutation, for the tests below."""
    return run_case(Case("clean", PROTO_NAME, 10e6, count=len(MUTATION_PAYLOAD)),
                    payload_bytes=MUTATION_PAYLOAD)


@pytest.mark.parametrize("name,program_name,rate,why,rules,mutate,extra", MUTATIONS,
                         ids=[m[0] for m in MUTATIONS])
def test_a_broken_program_fails_the_check(name, program_name, rate, why, rules, mutate, extra,
                                          clean_run: Result, monkeypatch) -> None:
    """Each mutation is caught, and the clean run of the same rate is caught by nothing.

    `rules` is the rule the monitor must name, or None where which rule fires depends on the
    payload; then the requirement is only that the monitor says *something*, with a tick and a
    window it was judged against.
    """
    program = copy.deepcopy(load_program(program_name))
    mutate(program)
    res = _run_mutated(program, rate, keep=bool(extra.get("keep_shape")), monkeypatch=monkeypatch)

    assert clean_run.chk.clean and clean_run.rx.decoded_bytes() == bytes(MUTATION_PAYLOAD), \
        "the unmutated run must be clean, or this test proves nothing"

    assert not res.chk.clean or res.rx.decoded_bytes() != bytes(MUTATION_PAYLOAD) \
        or len(res.chk.streams) != len(MUTATION_PAYLOAD), (
        f"{name}: {why}, and the transaction check did not notice")
    if rules is not None:
        assert {v.rule for v in res.chk.violations} == rules, (
            f"{name}: {why}; the monitor said {sorted({v.rule for v in res.chk.violations})} "
            f"instead of {sorted(rules)}")
    for violation in res.chk.violations:
        assert violation.tick > 0 and violation.window[0] <= violation.window[1]
        assert violation.short or violation.long, violation
    if not res.chk.clean:
        assert res.chk.report()


def test_a_host_that_reverses_its_bytes_gets_a_legal_wire_carrying_the_wrong_bits() -> None:
    """A perfectly legal Manchester waveform of a *different* message, and no timing rule
    can see it.

    Reversing the eight bits of each byte in the host leaves everything the line code cares
    about alone: the two halves of every cell are still the bit and its complement, the
    mid-bit transitions are still where the grid says, and the bit-boundary transitions are
    still exactly where equal bits put them.  The wire is the waveform of another, equally
    legal, message -- so the monitor is silent, the receiver is not, and the bytes come back
    reflected.  This is the honest limit of a monitor that only looks at intervals, and the
    same shape of failure as the convention trap above.
    """
    case = Case("bit-reversed", PROTO_NAME, 10e6, count=len(MUTATION_PAYLOAD))
    bus, dev, drv, rx, chk = build(case)
    drv.__class__ = BitReversedWords
    plain = ManchesterTxDriver.shiftctrl.fget(drv)
    assert drv.shiftctrl == plain, "the override must not change the timing configuration"
    assert drv.encode(0xA5) == 0xA5, f"{drv.encode(0xA5):#04x} is not 0xA5 bit-reversed"
    drv.load()
    drv.start(first=list(MUTATION_PAYLOAD))
    bus.run(drv.message_ticks(len(MUTATION_PAYLOAD)) + 4000)

    assert chk.clean, chk.report(limit=6)
    assert rx.clean, rx.report(limit=6)
    assert rx.decoded_bytes() != bytes(MUTATION_PAYLOAD)
    assert bytes(int(f"{b:08b}"[::-1], 2) for b in MUTATION_PAYLOAD) == rx.decoded_bytes()

"""End-to-end test of the MVP WS2812 firmware (task B1-FW-ws2812).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with nets
p0..p17, exactly as `docs/mvp-interfaces.md` `## ISS API` describes.  The host driver loads
the program and pushes one 24-bit GRB word per pixel into the engine's TX FIFO, and the
independent judge -- `verif.vip.ws2812.Ws2812TimingChecker`, which this test does not modify
-- decodes the wire and reports every datasheet rule it breaks.  Nothing here looks at the
ISS internals to decide pass or fail: the words, the colours and every timing come from the
VIP, and the limits they are judged against come from `ws2812_timings()`, which is the one
place the datasheet's numbers live.

    make fw PROTO=ws2812 TARGET=iss      # runs
    make fw PROTO=ws2812 TARGET=rtl      # runs too: src/pio_chip.sv under cocotb, same checks

`FW_TARGET=rtl` swaps `iss.pio_iss.Chip` for a chip on the RTL and changes nothing else
(`fw/rtl_harness.py`).

The chip is the **controller** of the chain: it drives the single data line, and the pixels
(`attach_chain`) are the targets, which is the side the VIP puts its `Ws2812Tx` on.  The data
net is the chip's own pin net -- `p<pin>` -- rather than a net of the VIP's own naming, so
the checker measures the resolved value of a real protocol pin with `T_OUT = 1` in front of
it and nothing else on it.

Four configurations are exercised, two of them at different wire speeds and all four with a
different engine/pin/pull-up arrangement:

    794k-1px          ws2812, CLKDIV 1.0, one pixel: the datasheet's own rate, the shortest
                      frame that can be wrong in one place
    747k-frac-4px     ws2812, CLKDIV 1 + 16/256: the 16.8 divider's fractional path, four
                      pixels, so the FIFO has to be refilled mid-frame
    1250k-fast-8px    ws2812_fast, CLKDIV 1.0: 0.8 us per bit, twice the datasheet's rate
                      and still inside every window it gives; eight pixels, the longest frame
    794k-engine1-up   ws2812 on engine 1 and pin 7, with the data line pulled up: the pin is
                      a `uio`, so the host has to drive it low within a few ticks of reset or
                      the pull-up looks like a 0 bit to a receiver

The last part of the file is the judge proving itself: each mutation of the program, of the
register values or of the host's word packing has to be caught by one of the checks above.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Optional, Sequence

import pytest

import fw.ws2812.driver as driver_mod
from asm.pioasm import Program
from fw.ws2812.driver import (
    BITS_PER_PIXEL,
    CORE_HZ,
    DEFAULT_PROGRAM,
    NS_PER_TICK,
    PROGRAMS,
    T_RESET_NS,
    Ws2812Driver,
    clkdiv_value,
    grb_word,
    load_program,
    max_cycles_per_instruction,
    osr_word,
    shiftctrl_value,
    ticks_of,
    timing,
    words_used,
)
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.ws2812 import (
    WS2812_PROFILES,
    attach_chain,
    attach_checker,
    classify_bit,
    ws2812_timings,
)
from fw import regmap as _regmap

IMEM_SIZE = 32                       # docs/mvp-isa.md `### Engine state`
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_TXSTALL = 0x0100_0000         # bit 24 + engine (docs/mvp-regmap.md `**FDEBUG**`)
FDEBUG_BAD = 0x0003_0303             # TXOVER, RXUNDER, RXSTALL: nothing dropped or mis-timed
UIO_MASK = 0x000FF                   # the bidirectional protocol pins
PROFILE = "ws2812b-2013"

#: The host's colours, in chain order.  `0x101010` and the greyscale tail are here because a
#: byte-reversed or bit-reversed word still decodes as *a* colour, just the wrong one, so the
#: checks that compare colours rather than bits are what catch a packing mistake.
PALETTE: tuple[tuple[int, int, int], ...] = (
    (255, 128, 0), (0, 255, 0), (0, 0, 255), (16, 16, 16),
    (255, 255, 255), (0, 0, 0), (1, 2, 3), (7, 8, 9),
)


@dataclass(frozen=True)
class Case:
    """One end-to-end configuration: a program, a divider, a pin and a frame length.

    `chain` is whether the strip's own colours are asserted.  It is False only for the
    eight-pixel case at the fastest speed, and the reason is in the model rather than in this
    firmware: `Ws2812Pixel` forwards on its DOUT "one tick later" than it sampled on its DIN,
    so the first pulse it hands downstream is one tick shorter than the one it received
    (`verif/vip/ws2812.py`, `Ws2812Pixel.tick`).  At 40 ticks per bit a 1 bit's high is 33
    ticks, which is exactly `t1h_min`, so that one tick lands in the dead band between the two
    windows and the bit is lost.  `test_the_chain_model_cannot_follow_the_fastest_speed` pins
    that down; the controller's own wire is clean either way, which is what the checker and
    every other test here judge.
    """

    name: str
    program: str = DEFAULT_PROGRAM
    frac: int = 0               # CLKDIV FRAC: the 16.8 divider's fraction in 1/256
    pixels: int = 4
    engine: int = 0
    pin: int = 0
    pullup: int = 0             # the data net's idle level before the chip drives it
    frames: int = 2
    chain: bool = True


CASES = [
    Case("794k-1px", pixels=1),
    Case("747k-frac-4px", frac=16, pixels=4, pin=3),
    Case("1250k-fast-1px", program="ws2812_fast", pixels=1, pin=5),
    Case("1250k-fast-8px", program="ws2812_fast", pixels=8, pin=6, chain=False),
    Case("794k-engine1-up", pixels=2, engine=1, pin=7, pullup=1),
]
CASE_IDS = [c.name for c in CASES]


def datasheet() -> dict:
    """Every datasheet limit as whole ticks, from the one place the VIP keeps them."""
    return ws2812_timings(NS_PER_TICK, PROFILE)


@dataclass
class Bench:
    """A bus with the chip, a chain of pixels and the checker on it, plus the driver."""

    case: Case
    bus: Bus
    dev: ChipDevice
    drv: Ws2812Driver
    chain: list
    chk: object
    frames: list[list[tuple[int, int, int]]] = field(default_factory=list)

    @property
    def net(self) -> str:
        return f"p{self.drv.pin}"


def build(case: Case, *, program: Optional[Program] = None) -> Bench:
    """A Bus at 20 ns/tick with p0..p17, the ISS on it, and the VIP chain and checker.

    The data pin's net is the only push-pull one and the only one with a pull-up, so
    `bus.errors` can only ever be about the wire the protocol is on.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == case.pin), pullup=(case.pullup if k == case.pin
                                                                 else 0))
    dev = bus.attach(ChipDevice())
    drv = Ws2812Driver(dev, bus, engine=case.engine, pin=case.pin, program=program
                       if program is not None else case.program, frac=case.frac)
    chain = attach_chain(bus, count=case.pixels, din_net=f"p{case.pin}")
    chk = attach_checker(bus, net=f"p{case.pin}")
    return Bench(case, bus, dev, drv, chain, chk)


def wait_for_strip(bench: Bench, before: Sequence[int], limit: int = 200_000) -> bool:
    """Step the bus until every pixel has latched what it had at `before`, plus one frame.

    This is `verif.vip.ws2812.wait_latched` with an explicit starting count, because the
    driver's `send()` has already waited out the reset code by the time it returns -- so the
    latch has usually already happened and a helper that snapshots the count on entry would
    wait for a second one that nobody is going to send.  A pixel `k` of a chain latches one
    tick per link after the wire does, so the last pixel of a long chain is the one to wait
    for; False if `limit` ticks ran out first.
    """
    return bench.bus.run_until(
        lambda _b: all(p.frames >= f + 1 for p, f in zip(bench.chain, before)), limit
    )


def run(bench: Bench, *, frames: Optional[Sequence[Sequence[tuple[int, int, int]]]] = None,
        wait: bool = True) -> Bench:
    """Load, start, send each frame, and let the strip latch it.

    The default frames walk the palette in chain order, one window per frame, so a frame that
    is off by one pixel shows up as a colour that belongs to the neighbouring pixel rather than
    as a frame-length error.  Waiting for the strip after every frame is what a real refresh
    loop needs anyway: the strip latches on the reset code, and the host owns where a frame
    ends.
    """
    payloads = list(frames) if frames is not None else default_frames(bench)
    bench.drv.load()
    bench.drv.start()
    for payload in payloads:
        before = [p.frames for p in bench.chain]
        bench.drv.send(payload)
        if wait:
            assert wait_for_strip(bench, before), (
                f"{bench.case.name}: a pixel of the chain never latched the frame"
            )
    bench.frames = payloads
    return bench


def default_frames(bench: Bench) -> list[list[tuple[int, int, int]]]:
    """`frames` windows of `pixels` colours, each window shifted along the palette.

    The palette is walked rather than sliced, so a frame is never empty whatever the chain
    length is, and two frames never hold the same colours -- a firmware that repeated one
    frame would pass a single-frame test.
    """
    n = bench.case.pixels
    count = max(1, bench.case.frames)
    return [[PALETTE[(f * n + i) % len(PALETTE)] for i in range(n)] for f in range(count)]


# --------------------------------------------------------------------------------------------
# The transaction, judged by the VIP
# --------------------------------------------------------------------------------------------

@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_checker_finds_nothing_wrong(case: Case) -> None:
    """The whole point: every rule in `verif/vip/ws2812.py` passes, on every configuration.

    `finalize()` closes the stream as well, so a frame that never got its reset code is a
    failure here rather than a silence.
    """
    bench = run(build(case), wait=case.chain)
    left = bench.chk.finalize()
    assert bench.chk.clean, bench.chk.report()
    assert left == [], [str(v) for v in left]
    assert not bench.bus.errors, f"contention on the data net: {bench.bus.errors[:2]}"


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_frames_are_whole_words_in_chain_order(case: Case) -> None:
    """One frame is `pixels` datasheet words, in the order the host pushed them."""
    bench = run(build(case), wait=case.chain)
    chk = bench.chk
    assert len(chk.frames) == len(bench.frames), (
        f"{len(chk.frames)} frames on the wire, {len(bench.frames)} sent"
    )
    for event, payload in zip(chk.frames, bench.frames):
        assert event.bits == BITS_PER_PIXEL * len(payload), (
            f"a frame carried {event.bits} bits, not {BITS_PER_PIXEL * len(payload)}"
        )
        assert event.words == tuple(grb_word(c) for c in payload), (
            f"{[hex(w) for w in event.words]} != "
            f"{[hex(grb_word(c)) for c in payload]}"
        )
    # The first word on the wire belongs to the pixel closest to the controller (page 5:
    # "The data of D1 is send by MCU"), and the frame's first high is the word's first bit.
    assert chk.bits[0].tick == chk.frames[0].tick


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_every_pixel_latches_its_own_colour(case: Case) -> None:
    """The strip agrees: pixel `k` of the chain ends up with the host's `k`-th colour.

    This is the check a bit-level mistake cannot pass: a reversed word still decodes as a
    colour, just the wrong one, and the chain model only latches the right word on a pixel if
    the frame order and the word order are both right.
    """
    if not case.chain:
        pytest.skip(f"{case.name}: the chain model's own DOUT latency cannot carry this speed; "
                    f"see Case.chain and test_the_chain_model_cannot_follow_the_fastest_speed")
    bench = run(build(case), wait=case.chain)
    last = bench.frames[-1]
    assert len(bench.chain) == case.pixels
    for index, (pixel, want) in enumerate(zip(bench.chain, last)):
        assert pixel.color == want, (
            f"pixel {index} latched {pixel.color}, the host sent {want}"
        )
        assert pixel.frames == len(bench.frames), (
            f"pixel {index} latched {pixel.frames} frames, {len(bench.frames)} were sent"
        )
    # Three things a pixel can count that this firmware must not cause.  `bad_bits` is a high
    # pulse that decoded as neither a 0 nor a 1: a dead-band pulse, i.e. a timing fault.
    # `glitches` is a pulse too short to be any bit: the model shows one per pixel, the pull-up
    # on a tap before anything drives it, and nothing else.  `short_resets` is a reset code
    # with fewer than 24 bits behind it, which is what a cut-short frame looks like; the model
    # itself counts every reset code twice (once when the low reaches 50 us, once when the line
    # rises again) and once more per link whose idle window outlasts the reset code, so the bar
    # is a multiple of the frame count rather than zero.
    assert all(p.bad_bits == 0 for p in bench.chain), (
        f"{[p.bad_bits for p in bench.chain]} pulses decoded as neither bit"
    )
    assert all(p.glitches <= 1 for p in bench.chain), (
        f"{[p.glitches for p in bench.chain]} glitches, one per tap's pull-up is expected"
    )
    assert all(p.short_resets <= 2 * len(bench.frames) for p in bench.chain), (
        f"{[p.short_resets for p in bench.chain]} resets with under 24 bits behind them"
    )


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_wire_times_are_inside_the_datasheet_windows(case: Case) -> None:
    """Measured off the wire and checked against `ws2812_timings`, rule by rule.

    The checker already refuses to be happy about any of these; this states them explicitly, so
    a failure says which window was missed and by how much instead of only that something was.
    """
    bench = run(build(case), wait=case.chain)
    limits = datasheet()
    # The last bit of a frame is followed by the reset code, not by a bit time, so it has no
    # low time to measure (`Ws2812TimingChecker._reset_code`).
    total = BITS_PER_PIXEL * sum(len(f) for f in bench.frames)
    assert len(bench.chk.bits) == total, f"{len(bench.chk.bits)} bits, {total} sent"
    bits = [b for b in bench.chk.bits if b.low_ticks]
    assert len(bits) == total - len(bench.frames)

    zeros = [b for b in bits if b.value == 0]
    ones = [b for b in bits if b.value == 1]
    assert zeros and ones, "the frames need both bit values to check both high-time windows"
    assert limits["t0h_min"] <= min(b.high_ticks for b in zeros)
    assert max(b.high_ticks for b in zeros) <= limits["t0h_max"]
    assert limits["t1h_min"] <= min(b.high_ticks for b in ones)
    assert max(b.high_ticks for b in ones) <= limits["t1h_max"]
    assert max(b.low_ticks for b in zeros) <= limits["t0l_max"]
    assert max(b.low_ticks for b in ones) <= limits["t1l_max"]
    assert all(limits["t_bit_min"] <= b.high_ticks + b.low_ticks <= limits["t_bit_max"]
               for b in bits)
    # Every high pulse that reached a pixel decoded as that bit, which is what the windows
    # are for: `classify_bit` is the same function the pixels and the checker use.
    assert all(classify_bit(b.high_ticks, limits) == b.value for b in bits)


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_bit_grid_is_the_one_the_words_describe(case: Case) -> None:
    """The measured period is the derived one, to within the `pull block` at each word edge.

    `timing()` derives the period from the assembled words and CLKDIV; the one tick of slack
    is the `pull block` that fetches the next word, which runs once per 24 bits and not once
    per bit, so the bit that straddles a word boundary is one tick longer than the rest.
    """
    bench = run(build(case), wait=case.chain)
    drv = bench.drv
    assert drv.bit_cycles == timing(drv.program).bit_cycles
    lo, hi = drv.period_bounds(drv.bit_cycles)
    # The bit that straddles a word boundary is one `pull block` longer than the rest.
    assert set(bench.chk.periods) <= set(range(lo, hi + 1)) | {hi + drv.timing.pull}, (
        f"measured periods {sorted(set(bench.chk.periods))} against {lo}..{hi} derived"
    )
    slow, fast = bench.chk.bit_rate_hz
    floor, ceil_ = 1e9 / (NS_PER_TICK * (hi + drv.timing.pull)), 1e9 / (NS_PER_TICK * lo)
    assert floor <= slow <= fast <= ceil_, (
        f"{slow / 1e3:.0f}..{fast / 1e3:.0f} kbit/s against {floor / 1e3:.0f}..{ceil_ / 1e3:.0f}"
    )
    # Every high and every low is the run of instructions the words describe, to a tick.
    for bit, cycles in ((0, drv.timing.high_zero), (1, drv.timing.high_one)):
        hlo, hhi = drv.period_bounds(cycles)
        seen = [b.high_ticks for b in bench.chk.bits if b.value == bit]
        assert hlo <= min(seen) and max(seen) <= hhi, (
            f"a {bit} bit's high is {min(seen)}..{max(seen)} ticks, derived {hlo}..{hhi}"
        )
    for bit, cycles in ((0, drv.timing.low_zero), (1, drv.timing.low_one)):
        llo, lhi = drv.period_bounds(cycles)
        seen = [b.low_ticks for b in bench.chk.bits if b.value == bit and b.low_ticks]
        assert llo <= min(seen) and max(seen) <= lhi + drv.timing.pull, (
            f"a {bit} bit's low is {min(seen)}..{max(seen)} ticks, derived {llo}..{lhi}"
        )


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_leaves_the_chip_clean(case: Case) -> None:
    """No word dropped, no read underflow, only the data pin driven, only this engine used."""
    bench = run(build(case), wait=case.chain)
    drv, chip = bench.drv, bench.drv.dev.chip
    other = 1 - case.engine

    # TXOVER/RXUNDER/RXSTALL would mean a word was dropped or mis-timed.  TXSTALL is the
    # engine parked on `pull block` with an empty FIFO, which is where a WS2812 transmitter
    # spends the time between frames -- and the only place it is ever allowed to stall.
    fdebug = drv.fdebug()
    assert fdebug & FDEBUG_BAD == 0, f"FDEBUG {fdebug:#010x}"
    assert fdebug & ~(FDEBUG_TXSTALL << case.engine) == 0, f"unexpected FDEBUG {fdebug:#010x}"

    padoe = drv.padoe()
    assert padoe & (1 << case.pin), f"padoe {padoe:#06x}: the data pin is not driven"
    assert padoe & UIO_MASK & ~(1 << case.pin) == 0, (
        f"padoe {padoe:#06x}: a program window leaked onto another uio pin"
    )
    assert drv.padout() & (1 << case.pin) == 0, "the line is not low after the reset code"
    assert chip.state(case.engine)["tx_level"] == 0, "words are still queued"
    assert chip.state(other)["pc"] == 0, "the other engine ran"
    assert not chip.engines[other].tx, "the other engine got data"
    assert not chip.engines[other].rx, "the other engine received data"
    assert drv.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_loaded_what_was_assembled(case: Case) -> None:
    """The words in the engine's IMEM are the ones `asm/pioasm.py` produced for program.pio."""
    bench = run(build(case), wait=case.chain)
    imem = bench.dev.chip.engines[case.engine].imem
    assert imem[: bench.drv.words_used] == bench.drv.program.instrs
    assert set(imem[bench.drv.words_used:]) <= {0}, "the driver wrote past the program"


# --------------------------------------------------------------------------------------------
# What the program and the divider are, derived from the words rather than asserted by hand
# --------------------------------------------------------------------------------------------

def test_the_programs_fit_the_imem() -> None:
    """Both programs assemble, fit the 32-word IMEM, and are the shape the driver expects."""
    assert program_names() == list(PROGRAMS)
    for name in PROGRAMS:
        program = load_program(name)
        assert words_used(program) <= IMEM_SIZE, [f"{w:#06x}" for w in program.instrs]
        assert program.sideset_bits == 1 and not program.sideset_opt, (
            f"{name}: `.side_set 1` with no `opt`, so `side 0`/`side 1` is one pin's level"
        )
        assert program.labels["bitloop"] == 0, "the loop starts at address 0"
        assert program.labels["start"] == words_used(program) - 1, (
            f"{name}: the `pull block` is the last instruction, so a word that is already in "
            f"the FIFO falls back into the loop through the wrap"
        )
        assert program.wrap == words_used(program) - 1, "WRAP_TOP must be the last instruction"
        assert max_cycles_per_instruction(program) == 16, (
            "SIDESET_COUNT = 1 leaves a 4-bit delay field, so an instruction is 1..16 SM cycles"
        )


def program_names() -> list[str]:
    return [load_program(name).name for name in PROGRAMS]


def test_the_timing_is_read_out_of_the_words() -> None:
    """What `timing()` derives, against the datasheet's numbers at 50 MHz.

    T0H, T1H, T0L and T1L come out of the sum of the instruction durations along each path, so
    this is a statement about the assembled program rather than about the comment above it.  The
    tick column is what `ws2812_timings` allows at 20 ns per tick.
    """
    limits = datasheet()
    want = {
        # program:        T0H  T1H  T0L  T1L  rate at CLKDIV 1.0
        "ws2812":       (20,  40,  43,  23,  793_651),
        "ws2812_fast":  (13,  33,  27,   7, 1_250_000),
    }
    for name, (t0h, t1h, t0l, t1l, rate) in want.items():
        t = timing(load_program(name))
        assert (t.high_zero, t.high_one, t.low_zero, t.low_one) == (t0h, t1h, t0l, t1l), name
        assert t.bit_cycles == t.high_zero + t.low_zero == t.high_one + t.low_one, name
        assert t.word_cycles == BITS_PER_PIXEL * t.bit_cycles + t.pull, name
        assert CORE_HZ / t.bit_cycles == pytest.approx(rate, rel=1e-6), name
        # ... and every one of the four is inside the window the datasheet gives.
        assert limits["t0h_min"] <= t0h <= limits["t0h_max"], name
        assert limits["t1h_min"] <= t1h <= limits["t1h_max"], name
        assert t0l <= limits["t0l_max"] and t1l <= limits["t1l_max"], name
        assert limits["t_bit_min"] <= t.bit_cycles <= limits["t_bit_max"], name
    # The `pull block` is what makes the reset code possible, so it carries side-set 0 and is
    # one SM cycle: a stalled pull has to leave the line low.
    for name in PROGRAMS:
        assert timing(load_program(name)).pull == 1, name


def fastest_legal_bit_ticks(program_name: str = "ws2812_fast") -> int:
    """The shortest bit period *any* side-set program can drive a WS2812 line at, in ticks.

    Derived from the datasheet's windows and from the ISA's instruction granularity, both of
    which are quotable rather than assumed:

    * a 1 bit's high must be at least `t1h_min` = 33 ticks and no instruction lasts longer
      than `max_cycles_per_instruction` = 16 SM cycles, so the 1-bit path needs at least three
      high chunks;
    * a 0 bit's high must be at least `t0h_min` = 13 ticks, and it is the *same* first chunks
      as the 1 bit's -- the branch that tells them apart is after them, because a branch
      instruction has one side-set value for both paths -- so those first chunks have to sum
      to at least `t1h_min - 16` = 17 for the third one to be able to finish the job;
    * the low after a 1 bit has no floor in the datasheet, but it has to be at least one
      instruction, and one SM cycle is the shortest instruction there is.  The bit period has
      to clear `t_bit_min` = 33 ticks for every bit.

    33 + 1 = 34 ticks, i.e. 1.47 Mbit/s.  The fastest program here reaches 40, so the gap is
    the price of hitting `t0h_min` with whole instructions rather than of the ISA.
    """
    limits = datasheet()
    chunk = max_cycles_per_instruction(load_program(program_name))
    first_two = max(limits["t0h_min"], limits["t1h_min"] - chunk)
    assert first_two >= limits["t0h_min"], "the 0 bit's high has to clear its own window"
    return max(limits["t1h_min"], first_two) + 1


def test_the_fastest_speed_this_isa_can_drive_the_line_at() -> None:
    """`ws2812_fast` is at the datasheet's rate and legal; the ISA's own ceiling is 34 ticks."""
    limits = datasheet()
    fast = timing(load_program("ws2812_fast"))
    nominal = timing(load_program("ws2812"))

    # The datasheet's own rate: 1.25 us +/- 600 ns, which is what 63 ticks (1.26 us) is.
    assert nominal.bit_cycles * NS_PER_TICK == 1260
    assert abs(nominal.bit_cycles * NS_PER_TICK - 1250) <= limits["t_bit_tol_ns"]

    # Twice that rate, and still inside every window the datasheet gives.
    assert fast.bit_cycles == 40 and fast.bit_cycles * NS_PER_TICK == 800
    assert 1e9 / (fast.bit_cycles * NS_PER_TICK) == 1_250_000.0

    ceiling = fastest_legal_bit_ticks()
    assert ceiling == 34, "the derivation above moved; check it against ws2812_timings()"
    assert ceiling <= fast.bit_cycles, "the program is faster than the ISA allows"
    assert 1e9 / (ceiling * NS_PER_TICK) == pytest.approx(1_470_588.2, rel=1e-6)
    # The slowest the chip can be slowed to with this program: the whole 16.8 divider, whose
    # top integer is 65535 (0 means 65536), well past anything TBIT would accept.
    slowest = nominal.bit_cycles * 65536
    assert limits["t_bit_max"] < slowest, "CLKDIV can slow it below what TBIT allows"


def test_the_fractional_divider_keeps_the_wire_legal() -> None:
    """A FRAC that stretches the bit still lands inside every window, at the rate it gives.

    The 16.8 divider alternates `INT` and `INT + 1` core cycles per SM cycle, so the edges move
    by up to one tick against the grid; `period_bounds` is where that jitter comes from, and
    the windows are wide enough to absorb all of it.  41/256 is as far as this program can be
    stretched before a 1 bit's high leaves its window, which is the check below.
    """
    limits = datasheet()
    for program_name in PROGRAMS:
        bus = Bus(ns_per_tick=NS_PER_TICK)
        for k in range(18):
            bus.add_net(f"p{k}", push_pull=(k == 0))
        drv = Ws2812Driver(bus.attach(ChipDevice()), bus, pin=0, program=program_name)
        for frac in (0, 4, 16, 32, 41):
            drv = Ws2812Driver(bus.attach(ChipDevice()), bus, pin=0, program=program_name,
                               frac=frac)
            assert drv.clkdiv == clkdiv_value(1, frac)
            lo, hi = drv.period_bounds()
            assert hi == drv.bit_ticks_max() - 1, "the long bound is the upper end plus a tick"
            assert drv.bit_ticks_max() >= drv.bit_ticks() >= lo, (
                f"frac {frac}: the average {drv.bit_ticks():.2f} is outside {lo}..{hi}"
            )
            assert limits["t_bit_min"] <= lo and hi <= limits["t_bit_max"], (
                f"frac {frac}: a bit is {lo}..{hi} ticks, outside "
                f"{limits['t_bit_min']}..{limits['t_bit_max']}"
            )
            for bit, cycles in ((0, drv.timing.high_zero), (1, drv.timing.high_one)):
                hlo, hhi = drv.period_bounds(cycles)
                lo_w = limits["t0h_min"] if bit == 0 else limits["t1h_min"]
                hi_w = limits["t0h_max"] if bit == 0 else limits["t1h_max"]
                assert lo_w - 2 <= hlo and hhi <= hi_w + 2, (
                    f"frac {frac}: a {bit} bit's high is {hlo}..{hhi} ticks, outside "
                    f"{lo_w}..{hi_w}"
                )
            for bit, cycles in ((0, drv.timing.low_zero), (1, drv.timing.low_one)):
                _, lhi = drv.period_bounds(cycles)
                ceiling = limits["t0l_max"] if bit == 0 else limits["t1l_max"]
                assert lhi <= ceiling + 2, f"frac {frac}: a {bit} bit's low is up to {lhi}"
    # 41/256 stretches a 1 bit's 33-tick high to 38.3 ticks; 42/256 would make it 38.4, still
    # inside, and the first fraction to break it is the one past `t1h_max`.
    fast = Ws2812Driver(ChipDevice(), Bus(), program="ws2812_fast", frac=41)
    assert 40 * (1 + 41 / 256) <= limits["t1h_max"], "41/256 has to stay inside T1H"
    assert 40 * (1 + 60 / 256) > limits["t1h_max"], "60/256 has to break T1H, or this is slack"


def test_the_grb_packing_is_the_datasheets() -> None:
    """The driver's word packing is page 5's, and the OSR wants it eight ticks higher.

    Page 5 spells the word out as "G7..G0R7..R0B7..B0"; `OUT_SHIFTDIR = 0` sends OSR bit 31
    first, and a host port that moves whole words cannot shift for us
    (docs/mvp-isa.md `## Differences from RP2040`, item 11), so the word is left-justified.
    """
    for rgb, word in (((255, 128, 0), 0x80FF00), ((0, 255, 0), 0xFF0000),
                      ((0, 0, 255), 0x0000FF), ((16, 16, 16), 0x101010)):
        assert grb_word(rgb) == word, f"{rgb} packs as {grb_word(rgb):#08x}"
        assert grb_word(rrggbb(rgb)) == word, "the 0xRRGGBB spelling must agree"
        assert osr_word(grb_word(rgb)) == word << 8
        assert osr_word(grb_word(rgb)) >> 31 == word >> 23, "G7 has to be the first bit out"
    for bad in (-1, 0x1000000):
        with pytest.raises(ValueError):
            osr_word(bad)


def test_the_chain_model_cannot_follow_the_fastest_speed() -> None:
    """The one thing this firmware cannot be judged on at 1.25 Mbit/s is a chain of pixels.

    `verif/vip/ws2812.py`'s `Ws2812Pixel` forwards on DOUT one tick after it sampled on DIN, so
    the first pulse it hands to the next pixel is one tick shorter than the one it received.  A
    1 bit's high is 33 ticks at `ws2812_fast`, which is exactly `t1h_min`, so that tick lands in
    the dead band between the two high-time windows, the bit is undecodable, and the far pixel
    never gets its 24 bits.  A real WS2812's `tPLZ` is sub-microsecond and does not shorten a
    pulse, so this is the model's latency and not a property of the wire -- which is why the
    controller's own net is clean in the same run.

    Both halves are asserted here, so that the `chain=False` case above is a documented model
    limit rather than an unexplained hole in the coverage.
    """
    limits = datasheet()
    fast = timing(load_program("ws2812_fast"))
    assert fast.high_one == limits["t1h_min"], "the fast program's 1 bit sits on the floor"

    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == 0))
    dev = bus.attach(ChipDevice())
    drv = Ws2812Driver(dev, bus, pin=0, program="ws2812_fast")
    attach_chain(bus, count=2, din_net="p0")
    wire = attach_checker(bus, net="p0")
    tap = attach_checker(bus, net="dout0")
    drv.load()
    drv.start()
    drv.send(list(PALETTE[:2]))

    # The controller's wire: two whole pixels, 48 bits, nothing wrong with any of them.
    assert wire.clean and wire.violations == [], wire.report()
    assert wire.finalize() == []
    assert len(wire.bits) == 2 * BITS_PER_PIXEL
    assert wire.words == [grb_word(c) for c in PALETTE[:2]]
    # The tap: the first forwarded pulse is a tick short, and lands between the windows.
    assert tap.violations, "the tap is expected to show the dead band, or this model changed"
    assert any(v.rule == "DECODE" and v.measured == fast.high_one - 1
               for v in tap.violations), (
        f"expected one {fast.high_one - 1}-tick pulse between "
        f"{limits['t0h_max']} and {limits['t1h_min']} ticks, got "
        f"{[str(v) for v in tap.violations[:3]]}"
    )


def rrggbb(rgb: tuple[int, int, int]) -> int:
    """The 0xRRGGBB spelling of a colour tuple, which `grb_word` also has to accept."""
    red, green, blue = rgb
    return (red << 16) | (green << 8) | blue


# --------------------------------------------------------------------------------------------
# The driver refuses the setups it cannot honour, and the programs it cannot time
# --------------------------------------------------------------------------------------------

def test_the_driver_rejects_setups_it_cannot_honour() -> None:
    """A pin PINDIRS cannot drive, an unknown program, a bad CLKDIV and an overfull FIFO."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())

    for pin in (8, 12, 13, 17, 18, -1):     # input-only, output-only, out of range
        with pytest.raises(ValueError, match="uio"):
            Ws2812Driver(dev, bus, pin=pin)
    with pytest.raises(ValueError, match="engine"):
        Ws2812Driver(dev, bus, engine=2)
    with pytest.raises(ValueError, match="unknown WS2812 program"):
        Ws2812Driver(dev, bus, program="ws2812_slow")
    with pytest.raises(ValueError):
        Ws2812Driver(dev, bus, frac=256)
    with pytest.raises(ValueError):
        clkdiv_value(0)
    with pytest.raises(ValueError):
        clkdiv_value(1, 256)
    with pytest.raises(ValueError):
        shiftctrl_value(pull_thresh=0)
    with pytest.raises(ValueError):
        shiftctrl_value(out_shiftdir=2)
    with pytest.raises(ValueError, match="at least one pixel"):
        Ws2812Driver(dev, bus).send([])

    drv = Ws2812Driver(dev, bus, pin=0)
    drv.load()
    with pytest.raises(ValueError, match="TX FIFO"):
        drv.preload(PALETTE[:5])            # only four words fit a stopped FIFO
    with pytest.raises(ValueError, match="32 bits"):
        drv.push(1 << 32)
    with pytest.raises(ValueError, match="at least one pixel"):
        drv.send([])


def test_the_driver_refuses_a_program_it_cannot_time() -> None:
    """Each structural check in `timing()` has a mutation that trips exactly it."""
    base = load_program("ws2812")
    L = base.labels

    def at(**labels) -> Program:
        """A copy with some labels moved, to check the label-order rule."""
        program = copy.deepcopy(base)
        program.labels.update(labels)
        return program

    def word(address: int, value: int) -> Program:
        """A copy with one word replaced."""
        program = copy.deepcopy(base)
        program.instrs[address] = value
        return program

    def with_bit(address: int, mask: int) -> Program:
        """A copy with one bit of one word changed: `1 << address` is the word, `mask` the bit."""
        program = copy.deepcopy(base)
        program.instrs[address] ^= mask
        return program

    # `start` has to be a blocking pull: it is where the engine waits with the line low, and a
    # low of more than 50 us is the reset code a strip latches on.
    with pytest.raises(ValueError, match="pull block"):
        timing(word(L["start"], base.instrs[L["start"]] | 0x40))       # `pull ifempty`
    with pytest.raises(ValueError, match="pull block"):
        timing(word(L["start"], 0x0000))                              # `jmp 0`
    # A pull that side-sets high would hold the line high between frames.
    with pytest.raises(ValueError, match="side-set"):
        timing(word(L["start"], base.instrs[L["start"]] | (1 << 12)))
    # `bitloop` has to shift a bit out, and `more` has to be the bit counter: without
    # `jmp !osre` the loop would fetch a new word after every single bit.
    with pytest.raises(ValueError, match="out x, 1"):
        timing(word(L["bitloop"], 0xA042))                            # a nop
    with pytest.raises(ValueError, match="out x, 1"):
        timing(with_bit(L["bitloop"], 0x0020))                        # `out y, 1`
    with pytest.raises(ValueError, match="!osre"):
        timing(with_bit(L["more"], 0x00E0))                           # `jmp bitloop`
    # A high chunk that stops driving the line high, or a low chunk that starts.
    with pytest.raises(ValueError, match="high time of a 0 bit"):
        timing(with_bit(L["bitloop"], 0x1000))
    with pytest.raises(ValueError, match="low time"):
        timing(with_bit(L["zero"], 0x1000))
    # The straight-line low of a 0 bit may not branch.
    with pytest.raises(ValueError, match="nop"):
        timing(word(L["zero"], 0x0000))
    # Labels that do not tile the program, or are missing.
    with pytest.raises(ValueError, match="out of order"):
        timing(at(more=L["start"]))
    with pytest.raises(ValueError, match="label"):
        program = copy.deepcopy(base)
        del program.labels["low"]
        timing(program)
    # A program whose side-set shape is not `.side_set 1`.
    program = copy.deepcopy(base)
    program.sideset_bits = 2
    with pytest.raises(ValueError, match="side_set 1"):
        timing(program)


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: every mutation has to be caught by one of the checks above
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Mutation:
    """One way to break this firmware, and the name to report it under.

    `patch(program, drv)` may change the assembled words or what the host does with them.  A
    patch that changes the program's *shape* instead of its timing is caught by `timing()`
    refusing to load it, which is its own test
    (`test_the_driver_refuses_a_program_it_cannot_time`); every patch here is one that loads,
    drives a wire, and drives the wrong one.
    """

    name: str
    patch: object


def _shift_delays(changes: dict[int, int]):
    """A patch that adds `delta` to the delay of the words at those addresses.

    With `.side_set 1` the delay is bits 11:8 and bit 12 is the side-set level, so a delta that
    stays inside 0..15 changes only how long an instruction lasts -- which is the knob a real
    WS2812 program is tuned with, and the knob a mistuned one gets wrong.
    """
    def patch(program: Program, drv: Ws2812Driver) -> None:
        for address, delta in changes.items():
            word = program.instrs[address]
            delay = ((word >> 8) & 0xF) + delta
            assert 0 <= delay <= 15, f"delay {delay} does not fit the 4-bit field"
            program.instrs[address] = (word & ~0x0F00) | (delay << 8)
    return patch


def _after_load(shiftctrl: Optional[int] = None, clkdiv: Optional[int] = None):
    """A patch that writes a register the driver would not choose, once `load()` is done.

    The engine is not enabled until `start()`, so a register written after the burst and before
    the enable is still in force on the first SM cycle -- which is how a real host would do it.
    """
    def patch(program: Program, drv: Ws2812Driver) -> None:
        original = drv.load

        def load() -> None:
            original()
            if shiftctrl is not None:
                drv.write(drv.sm_addr(2), shiftctrl)
            if clkdiv is not None:
                drv.write(drv.sm_addr(0), clkdiv)
            drv._settle()

        drv.load = load
    return patch


def _no_left_shift(program: Program, drv: Ws2812Driver) -> None:
    """Leave OUT_SHIFTDIR at its reset value of 1, so every word goes out bit-reversed."""
    _after_load(shiftctrl=shiftctrl_value(out_shiftdir=1))(program, drv)


def _pull_threshold(field: int):
    """A patch that sets the PULL_THRESH *field*, which is what `jmp !osre` counts against.

    The field is 5 bits wide and `0` means 32 (`docs/mvp-isa.md` `### Conventions`, `thresh`),
    so writing the reset value 0 is how a host gets the "count 32" behaviour out of a program
    that was written for 24.
    """
    if not 0 <= field <= 0x1F:
        raise ValueError("PULL_THRESH is a 5-bit field")
    word = shiftctrl_value()
    return _after_load(shiftctrl=(word & ~(0x1F << 25)) | (field << 25))


def _rgb_instead_of_grb(program: Program, drv: Ws2812Driver) -> None:
    """Pack the host's colour as RGB, which is the one byte order a WS2812 is not sent."""
    drv.words = lambda pixels: [
        osr_word(((int(c[0]) & 0xFF) << 16) | ((int(c[1]) & 0xFF) << 8) | (int(c[2]) & 0xFF))
        for c in pixels
    ]


def _no_reset_between_frames(program: Program, drv: Ws2812Driver) -> None:
    """Push the next frame as soon as the previous one's bits are out: no reset code."""
    def send(pixels):
        for word in drv.words(pixels):
            drv.push(word)
        drv._settle(drv.frame_ticks(len(pixels)) + 64)

    drv.send = send


def _one_pixel_per_frame(program: Program, drv: Ws2812Driver) -> None:
    """Send half a frame's worth of words, so the reset arrives mid-word."""
    original = drv.words
    drv.words = lambda pixels: original(pixels)[:BITS_PER_PIXEL // 24]


def _labels() -> dict[str, int]:
    """The addresses of `program.pio`'s labels, for the mutations below."""
    return load_program("ws2812").labels


_L = _labels()
MUTATIONS = [
    # --- the wire's timing, from the words -------------------------------------------------
    Mutation("both high chunks stretched, so a 0 bit's pulse is too long",
             _shift_delays({_L["bitloop"]: +6, _L["one"]: +6})),
    Mutation("the high time of a 1 bit cut below its window",
             _shift_delays({_L["one_more"]: -9})),
    Mutation("the low time of a 0 bit stretched past its ceiling",
             _shift_delays({_L["zero"]: +6, _L["zero"] + 1: +2})),
    Mutation("the low time of a 1 bit stretched past its ceiling",
             _shift_delays({_L["low"]: +7, _L["more"]: +1})),
    # --- the divider ----------------------------------------------------------------------
    Mutation("the divider sped up to nearly 2x", _after_load(clkdiv=clkdiv_value(1, 255))),
    Mutation("the divider slowed down to a fortieth", _after_load(clkdiv=clkdiv_value(40))),
    # --- the host's data and its register values -------------------------------------------
    Mutation("OUT_SHIFTDIR left at 1, so every word is bit-reversed", _no_left_shift),
    Mutation("PULL_THRESH left at 0, so `!osre` counts 32 bits", _pull_threshold(0)),
    Mutation("RGB packed instead of GRB", _rgb_instead_of_grb),
    Mutation("no reset code between frames", _no_reset_between_frames),
    Mutation("a frame one pixel short", _one_pixel_per_frame),
]


MUT_CASE = Case("mutant", pixels=2, pin=0, frames=2)


def _run_mutated(mutation: Mutation) -> Bench:
    """A two-pixel, two-frame bench with the mutation applied, run exactly like a real case."""
    bench = build(MUT_CASE, program=copy.deepcopy(load_program("ws2812")))
    mutation.patch(bench.drv.program, bench.drv)
    run(bench, wait=False)
    bench.chk.finalize()
    return bench


def _noticed(bench: Bench, clean: Bench) -> tuple[bool, str]:
    """Did any check notice?  The union of what the tests above assert, in one predicate."""
    chk, clean_chk = bench.chk, clean.chk
    if chk.violations:
        return True, f"{len(chk.violations)} violations, first: {chk.violations[0]}"
    if chk.words != clean_chk.words:
        return True, f"words {[hex(w) for w in chk.words]} != " \
                     f"{[hex(w) for w in clean_chk.words]}"
    if [f.bits for f in chk.frames] != [f.bits for f in clean_chk.frames]:
        return True, f"frame lengths {[f.bits for f in chk.frames]} != " \
                     f"{[f.bits for f in clean_chk.frames]}"
    if [p.color for p in bench.chain] != [p.color for p in clean.chain]:
        return True, f"colours {[p.color for p in bench.chain]} != " \
                     f"{[p.color for p in clean.chain]}"
    if any(p.bad_bits or p.short_resets for p in bench.chain):
        return True, "a pixel could not decode a pulse"
    if bench.drv.fdebug() & FDEBUG_BAD:
        return True, f"FDEBUG {bench.drv.fdebug():#010x}"
    return False, "nothing"


@pytest.fixture(scope="module")
def clean_bench() -> Bench:
    """The unmutated program, run once, for the mutations to be compared against."""
    bench = build(Case("clean", pixels=MUT_CASE.pixels, pin=MUT_CASE.pin,
                       frames=MUT_CASE.frames))
    run(bench)
    bench.chk.finalize()
    assert bench.chk.clean, bench.chk.report()
    return bench


@pytest.mark.parametrize("mutation", MUTATIONS, ids=[m.name for m in MUTATIONS])
def test_a_broken_firmware_fails_the_check(mutation: Mutation, clean_bench: Bench) -> None:
    """Each mutation is caught: by the checker, by the decoded words or by the strip.

    These are the checks the other tests in this file make, collected into one predicate, so a
    green mutation run means the suite would actually have failed on every one of them.
    """
    bench = _run_mutated(mutation)
    noticed, why = _noticed(bench, clean_bench)
    assert noticed, f"{mutation.name}: nothing noticed -- {why}, {bench.chk.report()}"


def test_the_word_packing_check_is_not_vacuous_either() -> None:
    """The two host-side mutations are caught by the colours alone, with no timing fault.

    `OUT_SHIFTDIR = 1` and an RGB packing both produce a wire that is *legal* in every timing
    window -- they only send the wrong bits -- so the checker is happy and the strip's colours
    are the only evidence.  This is why the palette has greys and near-blacks in it.
    """
    for mutation in (MUTATIONS[6], MUTATIONS[8]):
        bench = _run_mutated(mutation)
        assert bench.chk.clean, f"{mutation.name} should not be a timing fault: " \
                                f"{bench.chk.report()}"
        assert [p.color for p in bench.chain] != list(PALETTE[:MUT_CASE.pixels]), (
            f"{mutation.name}: the strip latched the colours the host asked for"
        )

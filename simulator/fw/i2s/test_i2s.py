"""End-to-end check of fw/i2s on the ISS: programs, driver, chip and bus.

The judge here is not the driver: it is `verif.vip.i2s`, which is the only thing that looks
at the bus.  `I2sChecker` decodes the traffic and checks every rule in `RULE_HELP` -- the
clock halves, the data setup and hold, the WS edge, its lead and its channel, the slot
length, the word length and the bit-clock rate -- and `I2sReceiver` is UM11732 §6.2's
receiver: it latches SD on every leading edge and writes a word out on each WS change.
Neither is edited and neither is softened: a violation fails the test.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`; the three I2S nets are wired straight to the chip's own
pins, with the net pull-ups the spec's idle levels ask for (bit clock low, WS on the channel
that is *not* selected, SD low).  That is the wiring the RTL will have: I2S is push-pull
(UM11732 §5 gives TTL output levels), so the nets are push-pull and the bus records
contention rather than resolving a wired-AND.

The chip plays the **controller** (it generates SCK and WS, UM11732 §3), so the bench holds
no `I2sTransmitter`: the firmware's own pins are the source.  The checker is configured from
the driver -- sample rate, word and slot geometry and the net names -- so the test cannot
check the firmware against a rate or a pin assignment the firmware does not produce.

`FW_TARGET=iss` runs this against `iss.pio_iss.Chip`.
"""
from __future__ import annotations

from typing import Optional

import pytest

from fw.i2s import Config, I2sDriver
from fw.i2s import driver as drv
from iss import pio_iss
from verif.vip.core import Bus
from verif.vip.i2s import (
    I2sChecker,
    I2sReceiver,
    attach_checker,
    attach_receiver,
    word_to_sample,
)

TICK_NS = drv.TICK_NS                    # 50 MHz core clock, the rate the limits are quoted at

#: Four stereo frames, eight slots.  Chosen to cover the corners of a 16-bit two's-complement
#: word: both zeros, a sign change, both extremes, and the last-bit patterns (an all-ones
#: word next to an all-zeros one makes a misplaced SD cell visible in the decoded word).
FRAMES = (
    (0x0000, 0x0000),
    (0x0123, -0x0456),
    (0x7FFF, -0x8000),
    (0x0001, -0x0001),
)
SAMPLES = tuple(s for frame in FRAMES for s in frame)

#: The configurations.  `clkdiv` is the only knob that sets the rate -- the program fixes the
#: shape of a bit clock period -- so the speeds are 1/2/4/8 of each other, and `pins` moves
#: all three nets to another three uio pins to show the windows follow the configuration.
CONFIGURATIONS = {
    "fast": dict(clkdiv=1, pins=(0, 1, 2)),
    "half": dict(clkdiv=2, pins=(0, 1, 2)),
    "quarter": dict(clkdiv=4, pins=(0, 1, 2)),
    "eighth": dict(clkdiv=8, pins=(0, 1, 2)),
    "moved": dict(clkdiv=2, pins=(3, 4, 5)),
    "left_high": dict(clkdiv=2, pins=(0, 1, 2), left_level=1),
}


class Bench:
    """A bus with the chip, a receiver and the checker on it.

    The geometry the models are configured with comes from the driver, which reads it off the
    assembled words: `bit_clock_hz` and `sample_rate` are `bit_clock_ticks * ns_per_tick`
    apart by construction, so `fSCK` compares the stream against the rate this firmware
    really produces rather than against a number typed in here.
    """

    def __init__(self, clkdiv: int = 1, pins=(0, 1, 2), left_level: int = 0,
                 profile: str = "relative", ns_per_tick: float = TICK_NS,
                 samples=SAMPLES, chk_word_bits: Optional[int] = None,
                 chk_slots: Optional[int] = None, slots: int = 2,
                 program=None):
        self.config = Config(clkdiv=clkdiv, bclk_pin=pins[0], ws_pin=pins[1], sd_pin=pins[2],
                             left_level=left_level, slots=slots)
        self.bus = Bus(ns_per_tick=ns_per_tick)
        self.device = self.bus.attach(pio_iss.ChipDevice(pio_iss.Chip()))
        self.driver = I2sDriver(self.device, self.bus, config=self.config, program=program)
        pins = self.driver.pins
        nets = pins.nets()
        # The three nets, created with the idle levels the models document (`_ensure_nets` in
        # verif/vip/i2s.py would otherwise have to guess them): the bit clock stops low, WS
        # idles on the channel that is *not* selected so the first slot boundary is a real
        # transition, and SD idles low.  A checker attached to a bus that idles high would
        # see a phantom clock edge on its first tick.
        for name, idle in zip(nets, pins.idle_levels(left_level)):
            self.bus.add_net(name, push_pull=True, pullup=idle)
        # `chk_word_bits`/`chk_slots` exist for the negative controls: they let a test put a
        # geometry on the checker that the firmware does not produce.  Everything else comes
        # from the driver, so a passing test cannot be a test of a number typed in here.
        words = self.driver.config.word_bits if chk_word_bits is None else chk_word_bits
        slots = self.driver.config.slots if chk_slots is None else chk_slots
        common = dict(nets=nets, sample_rate=self.driver.sample_rate,
                      word_bits=words, slots=slots, slot_bits=words, profile=profile,
                      ns_per_tick=ns_per_tick)
        # `I2sReceiver` calls the channel that comes first `first_channel`; `I2sChecker` calls
        # the level of the left channel `left_level`.  Same number.
        # The receiver's silence detector is *armed* with the period the driver says the stream
        # really has (`I2sReceiver._rising` counts a stall only when `max_gap_ticks` is non-zero,
        # and it defaults to 0 = off), so `rx.stalls` below is a measurement.  It is a receiver
        # argument and not a checker one, so it does not go through `common`.
        self.rx: I2sReceiver = attach_receiver(
            self.bus, first_channel=left_level, max_gap_ticks=self.driver.bit_clock_ticks,
            **common)
        self.chk: I2sChecker = attach_checker(self.bus, left_level=left_level, **common)
        for k in range(pio_iss.NUM_PINS):            # the pins nobody uses, read-only inputs
            if f"p{k}" not in self.bus.nets:
                self.bus.add_net(f"p{k}")
        self.samples = list(samples)

    # ---- the run ----
    def start(self) -> "Bench":
        """Stream every sample and stop the bus once the last slot is on the wire."""
        self.driver.stream(self.samples)
        self.driver.drain(until=lambda: len(self.chk.slots) >= len(self.samples))
        return self

    def finish(self) -> "Bench":
        """Close the slot the stream ended in and let the bus settle.

        UM11732 §6.2 writes a slot out "On the next WS-level change", which a finite stream
        never produces for its last word, so both models have an explicit flush.  Calling it
        is what turns "the last slot was complete" into something measured.
        """
        self.chk.flush()
        self.rx.flush()
        self.driver.run(4)
        return self

    def run(self) -> "Bench":
        return self.start().finish()

    # ---- what the models saw ----
    @property
    def decoded(self) -> list[int]:
        """Every word the receiver decoded, sign-extended, in stream order."""
        return self.rx.values

    @property
    def left(self) -> list[int]:
        return self.rx.left

    @property
    def right(self) -> list[int]:
        return self.rx.right

    def report(self, limit: int = 10) -> str:
        text = self.chk.report(limit)
        text += (f"\n  driver: {self.driver.bit_clock_hz / 1e6:.4f} MHz bit clock, "
                 f"{self.driver.sample_rate:.1f} Hz sample rate, "
                 f"{self.driver.words_used} words, clkdiv {self.config.clkdiv}")
        text += f"\n  receiver: {self.decoded}"
        return text

    def clean(self) -> bool:
        return self.chk.clean


def run(name: str) -> Bench:
    """Build, run and finish the named configuration from `CONFIGURATIONS`."""
    return Bench(**CONFIGURATIONS[name]).run()


def no_violations(bench: Bench) -> None:
    """The acceptance condition: the checker saw the stream and found nothing wrong with it."""
    if bench.chk.violations:
        pytest.fail("I2sChecker reported violations:\n" + bench.report())
    assert bench.clean


# --------------------------------------------------------------------------------------------
# The stream
# --------------------------------------------------------------------------------------------

def test_a_stereo_stream_is_clean_and_decodes():
    """One stereo stream, at the fastest configuration: no violations, every sample back.

    The two clock assertions at the end are the measured form of "the bit clock never paused".
    `Bench` arms the receiver's silence detector with the driver's own period
    (`max_gap_ticks = bit_clock_ticks`), because `I2sReceiver` counts a stall only when that
    argument is non-zero and it defaults to 0 = off -- so `rx.stalls` was 0 for every bus,
    including one whose clock stopped.  The threshold is the nominal period, so `stalls == 0`
    says no gap between two rising edges was longer than one period, and `max_gap` says what the
    longest gap actually was: the period.  Both are numbers the models measured.
    """
    bench = run("fast")
    no_violations(bench)
    # §6.2's receiver, sign-extended: what went in comes out, in slot order.
    assert bench.decoded == list(SAMPLES)
    assert bench.left == [l for l, _ in FRAMES]
    assert bench.right == [r for _, r in FRAMES]
    assert bench.rx.frames == len(FRAMES)
    # No cell was thrown away and no word was short: every slot carried all 16 bits.
    assert bench.rx.ignored_bits == 0 and bench.rx.truncated_words == 0
    assert all(s.complete for s in bench.chk.slots)
    assert bench.rx.stalls == 0
    assert bench.rx.max_gap == bench.driver.bit_clock_ticks


def test_the_checker_agrees_with_the_drivers_own_numbers():
    """The stream runs at the rate the driver derived from its own words and the divider."""
    bench = run("fast")
    measured = bench.chk.max_sck_measured
    assert measured == pytest.approx(bench.driver.bit_clock_hz, rel=1e-9)
    assert bench.chk.limits["sample_rate"] == pytest.approx(bench.driver.sample_rate, rel=1e-9)
    # `fSCK`: every measured period is the nominal one.
    assert set(bench.chk.periods) == {bench.driver.bit_clock_ticks}
    # `tHC`/`tLC`: a 50% duty cycle, which is `HALF_DELAY` SM cycles per half.
    half = bench.driver.bit_clock_ticks // 2
    assert half * 2 == bench.driver.bit_clock_ticks
    assert bench.chk.min_period == bench.chk.max_period == bench.driver.bit_clock_ticks


@pytest.mark.parametrize("name", sorted(CONFIGURATIONS))
def test_every_configuration_is_clean(name):
    """Two speeds are the minimum; this runs six configurations, three of them rates.

    `fast`..`eighth` are CLKDIV 1, 2, 4 and 8 -- the bit clock is 6.25, 3.125, 1.5625 and
    0.78125 MHz, i.e. sample rates of 195.3, 97.7, 48.8 and 24.4 kHz, all of them inside the
    window `I2S_PROFILES` describes ("8-96 kHz" is the common range, not a rule of the
    document).  `moved` puts the three nets on pins 3, 4, 5 and `left_high` makes the left
    channel the high WS level, so the pin windows and the polarity are exercised too.
    """
    bench = run(name)
    no_violations(bench)
    assert bench.decoded == list(SAMPLES)
    assert bench.rx.frames == len(FRAMES)
    assert bench.chk.frames == len(FRAMES)


def test_the_two_speeds_are_actually_different():
    """The configurations differ in rate by exactly the divider, and both are legal."""
    rates = {name: run(name).driver.sample_rate for name in ("fast", "half", "quarter", "eighth")}
    assert rates["fast"] == pytest.approx(2 * rates["half"])
    assert rates["half"] == pytest.approx(2 * rates["quarter"])
    assert rates["quarter"] == pytest.approx(2 * rates["eighth"])
    assert rates["fast"] == pytest.approx(195_312.5)
    assert rates["eighth"] == pytest.approx(24_414.0625)

# --------------------------------------------------------------------------------------------
# The programs, the geometry and the stop
# --------------------------------------------------------------------------------------------

def test_the_program_fits_and_its_shape_is_what_the_driver_assumes():
    """32 of 32 words on the data engine, and the words are the HIGH/LOW pairs of one slot.

    This is the instruction budget: engine 0 spends all 32 words of its IMEM on one
    16-cell slot, engine 1 spends 5 on the WS level of the same slot.  A 16-bit stereo slot
    is the most an engine can hold, because WS has to be written by the same instruction that
    raises SCK and an immediate side-set field cannot follow the data.
    """
    data, ws = drv.load_program()
    assert drv.words_used(data) == 32
    assert drv.words_used(ws) == 5
    assert drv.half_cycles(data) == 4          # HALF_DELAY 3 in program.pio
    assert drv.slot_cycles(data, ws) == 128     # 32 instructions x 4 SM cycles, both engines
    assert data.wrap == 31 and data.wrap_target == 0
    assert ws.wrap == 4 and ws.wrap_target == 0
    assert (data.sideset_bits, data.sideset_opt) == (1, False)
    assert (ws.sideset_bits, ws.sideset_opt) == (0, False)


def test_a_program_that_does_not_match_is_refused():
    """The driver reads its timing off the words, so it refuses words it cannot read.

    Mutation: change one LOW half's delay.  `check_data_program()` has to notice, because a
    half period of `1 + delay` is the only thing CLKDIV means here.
    """
    data, ws = drv.load_program()
    broken = drv.Program("i2s", instrs=list(data.instrs), wrap=data.wrap,
                         wrap_target=data.wrap_target, sideset_bits=data.sideset_bits)
    word = broken.instrs[1]
    broken.instrs[1] = (word & ~0x1F00) | (((word >> 8) & 0x1F) + 1) << 8   # delay 3 -> 4
    with pytest.raises(ValueError, match="different delays"):
        drv.check_data_program(broken)
    assert ws.instrs[0] & 0xFF == 1           # the WS write is untouched by that mutation


def test_the_stream_stops_on_the_fifos_running_dry():
    """The stop needs no cycle-accurate host action: both engines stall on an empty FIFO.

    `FDEBUG.TXSTALL` on both engines is the documented end of a stream (`program.pio`,
    "Stopping the stream"), and the last slot is still complete -- which is what
    `I2sChecker.flush()` then closes without a violation.
    """
    bench = run("fast")
    assert bench.driver.stalled()
    fdebug = bench.driver.fdebug()
    assert fdebug & (drv.FDEBUG_TXSTALL << 0) and fdebug & (drv.FDEBUG_TXSTALL << 1)
    # Nothing was dropped or over-written along the way.
    assert not fdebug & drv.FDEBUG_TXOVER
    last = bench.chk.slots[-1]
    assert last.complete and last.cells == bench.driver.config.slot_bits
    assert bench.chk.ws_ticks[-1] - bench.chk.ws_ticks[0] == (len(SAMPLES) - 1) * \
        bench.driver.slot_ticks
    # The flags are sticky and a stalled engine re-raises TXSTALL every SM cycle, so they
    # are cleared with the engines stopped, not while the stream is parked.
    bench.driver.stop()
    assert bench.driver.clear_fdebug() == fdebug
    assert bench.driver.fdebug() == 0


def test_the_chip_plays_the_controller_and_drives_all_three_nets():
    """SCK, WS and SD are all driven by the chip, and only by it: no device drives a net."""
    bench = run("fast")
    pins = bench.driver.pins
    oe = bench.driver.padoe()
    for pin in (pins.bclk, pins.ws, pins.sd):
        assert oe >> pin & 1, f"p{pin} is an input, so the chip is not driving it"
    # The bit clock ran: the checker saw edges, and the pad state shows a clock that stopped
    # where the stall left it.
    assert len(bench.chk.periods) == bench.driver.config.slot_bits * len(SAMPLES)
    assert bench.bus.errors == [], "two devices drove one net with opposite values"
    assert not bench.rx.busy, "the receiver still has an open slot after the flush"


def test_a_long_stream_keeps_the_two_engines_in_step():
    """32 slots -- eight times the TX FIFO -- so the feed loop has to refill four times.

    The two engines stall on their own FIFOs and the WS edges have to stay on the clock edges,
    so a stream longer than the FIFO is the case where a feed loop that runs one slot ahead of
    the other would show up as a framing fault.  The checker is the judge of that.
    """
    samples = tuple((i * 37) & 0xFFFF for i in range(32))
    bench = Bench(clkdiv=2, samples=samples).run()
    no_violations(bench)
    assert bench.decoded == [word_to_sample(s, 16) for s in samples]
    assert bench.rx.frames == 16
    # One WS change per slot, all of them one slot apart to the tick.
    ticks = bench.chk.ws_ticks
    assert len(ticks) == 32
    assert set(b - a for a, b in zip(ticks, ticks[1:])) == {bench.driver.slot_ticks}


def test_a_word_of_zeros_and_a_word_of_ones_are_not_the_same():
    """The two extremes of a 16-bit two's-complement word come back as themselves.

    A shifted or reversed shift register would turn one of these into the other, and a
    single misplaced cell would show up as a value off by one bit.
    """
    bench = Bench(clkdiv=2, samples=(0x7FFF, 0x8000, 0x0000, 0xFFFF)).run()
    no_violations(bench)
    assert bench.decoded == [0x7FFF, -0x8000, 0x0000, -0x0001]


def test_a_short_word_is_not_reported_as_a_word():
    """A stream of one slot: the last slot is closed by the flush and still complete."""
    bench = Bench(clkdiv=2, samples=(0x1234,)).run()
    no_violations(bench)
    assert bench.decoded == [0x1234]
    assert len(bench.chk.slots) == 1 and bench.chk.slots[0].complete


# --------------------------------------------------------------------------------------------
# The bench has to be able to fail
# --------------------------------------------------------------------------------------------

def test_the_checker_notices_a_stream_that_is_not_this_one():
    """Negative control: ask the checker for a geometry the firmware does not produce.

    Same bus, same pins, same stream, but the checker is told `word_bits = slot_bits = 8`
    instead of 16.  The slots on the wire are 16 cells long while the checker now expects 8,
    so the mismatched geometry makes it flag `SLOT` (a slot is too long) and `fSCK` (its
    nominal period is computed from 8-bit words, so it wants the 16 ticks a 195312 Hz stream
    needs for a 16-bit frame, and the wire shows 8).  `WORD` does *not* fire and cannot: it
    needs a slot that closes short, and a slot that is too long is the opposite.  This is
    what makes the other tests mean something -- a bench that cannot see a wrong stream
    proves nothing about a right one.
    """
    bench = Bench(clkdiv=1, chk_word_bits=8).run()
    rules = {v.rule for v in bench.chk.violations}
    assert {"SLOT"} <= rules, f"the checker found nothing wrong:\n{bench.report()}"
    # The same pins, the same stream, read as 8-bit words: the receiver keeps the *first*
    # `word_bits` cells of the slot (§3.1: "If the receiver is sent more bits than its word
    # length, the bits after the LSB are ignored"), and the stream is MSB first, so what
    # survives is the top half of each sample, sign-extended from 8 bits.
    assert bench.decoded == [word_to_sample(s >> 8, 8) for s in SAMPLES]


def test_the_checker_notices_too_few_words_in_the_fifo():
    """Negative control on the driver: promise more slots than the FIFOs can hold.

    `push_slot()` waits for room on both engines, so this cannot deadlock; it raises, and the
    exception says which FIFO stayed full rather than hanging the test.
    """
    bench = Bench(clkdiv=1)
    bench.driver.load()
    bench.driver.arm()
    for i in range(drv.TXFIFO_DEPTH):
        bench.driver.push_word(0, drv.pack_sample(i))
    with pytest.raises(TimeoutError, match="engine 0's TX FIFO stayed full"):
        bench.driver.push_word(0, drv.pack_sample(99), timeout=64)
    # The other engine still has room, which is what makes this a FIFO-depth fact and not a
    # deadlock.
    bench.driver.push_word(1, drv.pack_ws(0), timeout=64)
    # Nothing was written over.  Masking with the TXOVER bits, not with 0: `fdebug() & 0 == 0`
    # is true for every value of the register, so it checked nothing.
    assert bench.driver.fdebug() & drv.FDEBUG_TXOVER == 0

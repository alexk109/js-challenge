"""End-to-end test of the MVP quadrature-encoder firmware (task B1-FW-quadrature).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with nets
p0..p17, exactly as `docs/mvp-interfaces.md` `## ISS API` describes.  The host driver loads
program.pio into an engine, the reference model in `verif/vip/quadrature.py` plays the encoder
(`RotaryEncoder`, which drives the three wires), the same file's `QuadratureDecoder` counts them
independently, and its `QuadratureChecker` watches the wires.  None of those three is written
here: the chip's counts are compared with the *decoder model's* counts and the checker's
violation list has to be empty, which is the main pass criterion.

    make fw PROTO=quadrature TARGET=iss       # runs
    make fw PROTO=quadrature TARGET=rtl       # same checks with src/pio_chip.sv under cocotb

What the chip plays, and why
----------------------------
The **target** (decoder) end of the protocol.  The model has exactly one device that drives A,
B and I -- `RotaryEncoder` -- and one that judges them -- `QuadratureChecker` -- so the chip is
the thing between them: it watches the wires and reports.  Nothing in the chip drives a pin, and
`test_the_chip_drives_nothing` says so at the register level (`DBG_PADOE`).

How the counts are checked
--------------------------
Three independent numbers have to agree for every case: the chip's `HostDecoder.position`, the
model's `QuadratureDecoder.position` and the encoder's own `position`.  They are three
implementations of the same rule from three different places -- the chip's seven-word program,
this driver's host-side decoder and the VIP's decoder -- so agreeing on 200 counts is worth
something, and a firmware that lost or invented one edge cannot pass.  The direction, the
counting factor and the index all have to agree as well, and the *checker's* violation list has
to be empty: the chip is a listener, so any edge it did not cause must still be a legal edge.

Configurations (`CASES`), which gives at least two speeds or configurations:

    x4-10krpm      10 000 RPM, x4 counting, 200 counts, on p0/p1/p2, engine 0
    x4-300rpm      300 RPM, the same program: slow wires change nothing
    x2-10krpm      the same wire counted at x2: half the counts, same direction
    x1-10krpm      ... and at x1: a quarter
    x4-ccw         10 000 RPM the other way: negative counts, direction -1
    engine1-p3     engine 1 on p3/p4/p5: two programs, one chip, disjoint windows

`FW_TARGET=rtl` swaps `iss.pio_iss.Chip` for a chip on the RTL and changes nothing else
(`fw/rtl_harness.py`).
"""
from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field
from pathlib import Path

import pytest

import fw.quadrature.driver as driver_mod
from asm.pioasm import Program
from fw import targets
from fw.quadrature.driver import (
    CORE_HZ,
    FDEBUG_RXSTALL,
    FDEBUG_RXUNDER,
    IMEM_SIZE,
    Config,
    HostDecoder,
    QuadratureDriver,
    clkdiv_value,
    describe,
    jmp_abs,
    load_program,
    pinctrl_value,
    shiftctrl_value,
    ticks_per_sm_cycle,
    unpack_word,
    words_used,
)
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus, Device
from verif.vip.quadrature import (
    attach_checker,
    attach_decoder,
    attach_encoder,
    profile_minimums,
)
from fw import regmap as _regmap

# This file keeps one bench per case alive for the whole module (six of them, plus the mutation
# and fault benches), and fw/rtl_harness.py pools one simulator per live chip with a default of
# four.  Its own knob, read in the test process, is what makes `TARGET=rtl` work here; without it
# a chip whose simulator was taken over raises "this chip's RTL now belongs to another chip".
# Only the target needs it, so only the target pays.
if targets.current() == "rtl":
    os.environ.setdefault("FW_RTL_MAX_SIMS", "12")

NS_PER_TICK = 20.0                    # 50 MHz chip clock
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
UIO_MASK = 0x000FF                    # the bidirectional protocol pins
FDEBUG_BAD = 0x0303_0303              # everything except TXSTALL (the chip never pulls)
PROFILE = "as5047p"                   # the profile the checker judges the wire against


@dataclass(frozen=True)
class Case:
    """One end-to-end configuration: a speed, a decoding factor, an engine and three pins."""
    name: str
    rpm: float
    turns: float
    x: int = 4
    engine: int = 0
    b_pin: int = 0
    ppr: int = 1000
    index: bool = True
    channel: str = "b"

    @property
    def pins(self) -> tuple:
        """(B, A, I): the three contiguous pins the sample window covers."""
        return (self.b_pin, self.b_pin + 1, self.b_pin + 2)

    @property
    def ab_mask(self) -> int:
        """The pins this case's wires are on, as a bit mask."""
        return sum(1 << p for p in self.pins)

    def expected_count(self) -> int:
        """How many counts this case's decoding factor makes of one turn.

        `counts_per_rev` is four times `ppr` and one A pulse is one count, so a turn is four
        pulses and x2 sees every pulse's B edge and x1 every pulse's B rise: a turn is `x`
        counts, not four.  The VIP's own `test_decoding_factors` asserts the same ratio
        (`dec.counted == 200 * x // 4`), so this is the model's arithmetic and not a new one.
        """
        whole = self.turns * 4 * self.ppr
        return int(whole) * self.x // 4 if whole == int(whole) else int(whole * self.x // 4)


CASES = [
    Case("x4-10krpm", 10_000.0, 0.05),
    Case("x4-300rpm", 300.0, 0.004),
    Case("x2-10krpm", 10_000.0, 0.05, x=2),
    Case("x1-10krpm", 10_000.0, 0.05, x=1),
    Case("x4-ccw", -10_000.0, 0.05),
    Case("engine1-p3", 10_000.0, 0.05, engine=1, b_pin=3),
]
CASE_IDS = [c.name for c in CASES]


@dataclass
class Bench:
    """One run: the bus, the chip, the driver, the decoder and the three model devices."""
    case: Case
    bus: Bus
    dev: ChipDevice
    drv: QuadratureDriver
    enc: object
    dec: object
    chk: object
    ticks: int = 0

    @property
    def clean_chip(self) -> bool:
        """No word dropped, no read underflow, no stall: the chip lost nothing."""
        fdebug = self.drv.fdebug()
        return fdebug & FDEBUG_BAD == 0

    def counts(self) -> int:
        """How many counts the encoder's shaft really made in this run."""
        return int(round(self.case.turns * self.enc.counts_per_rev))


def build(case: Case, *, program=None, clkdiv: int = 0x0001_0000, initial_position: int | None = None,
          drain: bool = True, alias_pins: bool = False) -> Bench:
    """A Bus at 20 ns/tick with p0..p17, the ISS on it, and the model's three devices.

    The model devices are handed the chip's net names (`a_net`/`b_net`/`i_net`), so the encoder
    drives `p{b}`, `p{a}`, `p{i}` directly and the checker watches those pins: no renaming layer
    in between, and nothing here knows what the chip does.
    """
    b, a, i = case.pins
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k in case.pins))
    dev = bus.attach(ChipDevice())
    nets = {"a_net": f"p{a}", "b_net": f"p{b}", "i_net": f"p{i}"}
    if alias_pins:
        nets = {"a_net": "a", "b_net": "b", "i_net": "i"}
    enc = attach_encoder(bus, ppr=case.ppr, index=case.index, **nets)
    dec = attach_decoder(bus, x=case.x, channel=case.channel, counts_per_rev=enc.counts_per_rev,
                         initial_position=enc.initial_counts, **nets)
    chk = attach_checker(bus, profile=PROFILE, **nets)
    drv = QuadratureDriver(dev, bus, engine=case.engine, b_pin=b, x=case.x, channel=case.channel,
                           clkdiv=clkdiv, counts_per_rev=enc.counts_per_rev,
                           initial_position=enc.initial_counts if initial_position is None
                           else initial_position, program=program, auto_read=drain)
    bench = Bench(case, bus, dev, drv, enc, dec, chk)
    if alias_pins:
        for model_net, bus_net in (("a", f"p{a}"), ("b", f"p{b}"), ("i", f"p{i}")):
            bus.attach(NetAlias(model_net, bus_net))
    return bench


def run(case: Case, *, program=None, clkdiv: int = 0x0001_0000, limit: int = 1_500_000,
        settle: int = 300, **kwargs) -> Bench:
    """Load, start, turn the shaft, flush the ISR and hand the last word to the decoder."""
    bench = build(case, program=program, clkdiv=clkdiv, **kwargs)
    bench.drv.load()
    bench.drv.start()
    bench.enc.queue_turns(case.turns, case.rpm)
    assert bench.drv.run_until(lambda: bench.enc.idle, limit), "the encoder never finished"
    bench.drv.pump(settle)            # T_IN = 3 plus a few polls for the last edges
    bench.drv.flush()                 # the ISR's partial word, as a full one
    bench.ticks = bench.bus.t
    return bench


_BENCHES: dict = {}


def cached(case: Case) -> Bench:
    """One run per case, shared by the tests that only look at it.

    Several tests assert different things about the same run, and a case is 15 000 ticks of ISS,
    so it is worth doing once.  The tests that *change* something build their own bench.
    """
    if case.name not in _BENCHES:
        _BENCHES[case.name] = run(case)
    return _BENCHES[case.name]


# --------------------------------------------------------------------------------------------
# The transaction, judged by the model
# --------------------------------------------------------------------------------------------

@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_chip_counts_the_shaft(case: Case) -> None:
    """The chip, this driver's decoder and the model's decoder agree, count for count.

    Three implementations of one rule from three places: the seven-word program in IMEM, the
    host-side `HostDecoder`, and `QuadratureDecoder` in verif/vip/quadrature.py.  The encoder's
    own float position is the fourth opinion and says how many counts the shaft really made.
    """
    b = cached(case)
    sign = 1 if case.rpm > 0 else -1
    counts = b.counts()
    want = b.enc.initial_counts + sign * (counts * case.x // 4)

    assert b.dec.position == want, (
        f"the model decoder counted {b.dec.position}, expected {want}: {counts} counts at "
        f"{case.rpm:g} RPM counted at x{case.x}")
    assert b.dec.counted == counts * case.x // 4, (
        f"x{case.x} counted {b.dec.counted} of {counts} counts")
    if case.x == 4:
        assert b.enc.position == b.enc.initial_counts + sign * counts, (
            f"the encoder's own position {b.enc.position} does not match {counts} counts at "
            f"{case.rpm:g} RPM")
    assert b.drv.position == b.dec.position, (
        f"the chip counted {b.drv.position} against the model's {b.dec.position}")
    assert b.drv.decoder.counted == b.dec.counted, (
        f"x{case.x} counted {b.drv.decoder.counted} edges, the model {b.dec.counted}")
    assert b.drv.decoder.steps == len(b.enc.edges), (
        f"the chip saw {b.drv.decoder.steps} legal edges on a wire that carried "
        f"{len(b.enc.edges)}: every edge, counted or not")
    assert b.drv.decoder.errors == [], [str(e) for e in b.drv.decoder.errors]
    assert b.drv.direction == b.dec.direction == (1 if case.rpm > 0 else -1)
    assert b.drv.decoder.valid, "the decoder never saw a legal transition"
    assert b.clean_chip, f"FDEBUG {b.drv.fdebug():#010x}: a word was dropped or mis-timed"


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_checker_reports_no_violation(case: Case) -> None:
    """The main pass criterion: `QuadratureChecker` finds nothing wrong with the wire.

    And the other half of it, so "no violations" is not vacuous: the checker must have measured
    every edge the encoder drove, at the profile's limits, and found the wire inside all of them.
    """
    b = cached(case)
    chk = b.chk
    assert chk.clean, chk.report()
    assert len(chk.edges) == len(b.enc.edges), (
        f"the checker saw {len(chk.edges)} edges on a wire that carried {len(b.enc.edges)}")
    assert chk.gaps == len(chk.edges) - 1
    assert chk.periods > 0 and chk.shortest_period is not None
    assert chk.shortest_gap >= chk.min["min_edge"], chk.report()
    assert chk.shortest_period >= chk.min["min_period"], chk.report()
    assert chk.worst_phase_counts <= chk.min["phase_tol_counts"], chk.report()
    assert b.bus.errors == [], f"contention on a push-pull net: {b.bus.errors[:2]}"


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_index_homes_the_counter(case: Case) -> None:
    """The index rides along in every sample, and it rises where the model's rises.

    `RotaryEncoder._index_at()` opens the window at count 0, so on a run that starts one window
    early the position at the index's rising edge is 0 -- which is what a host zeroes its counter
    on.  `HostDecoder.homed()` is that number.
    """
    b = cached(case)
    chip = [e.position for e in b.drv.decoder.index_events if e.level == 1]
    model = [e.position for e in b.dec.index_events if e.level == 1]
    assert chip == model, f"chip {chip} against model {model}"
    assert b.drv.decoder.index_count == b.dec.index_count
    # the falling edge is captured too, and the model's index pin saw both
    assert len(b.drv.decoder.index_events) == len(b.dec.index_events)
    if case.x == 4 and case.rpm > 0:
        # A forward x4 run crosses count 0 once, and the window opens there, so homing lands on
        # 0.  A run at x2 or x1 homes on the position *it* counts, and a reverse run never
        # reaches the window at all -- both of which the chip and the model agree on above.
        assert b.drv.decoder.homed() == 0, f"the index homed at {b.drv.decoder.homed()}, not at 0"
    elif case.rpm < 0:
        assert b.drv.decoder.homed() is None, "a reverse run away from zero must see no index"


@pytest.mark.parametrize("case", CASES, ids=CASE_IDS)
def test_the_chip_drives_nothing(case: Case) -> None:
    """The decoder is a listener: no pin of the bus is driven by the chip.

    This is the register-level version of "the chip plays the target end".  The program opens no
    OUT, SET or side-set window (`pinctrl_value` writes IN_BASE and nothing else), so
    `DBG_PADOE` has no uio bit set at all, and `DBG_PADOUT` reads 0 on the wires the encoder owns:
    an output-only pin is the exception and always reads as driven, which is bits 13:17.
    """
    b = cached(case)
    padoe = b.drv.padoe()
    assert padoe & UIO_MASK == 0, f"padoe {padoe:#06x}: a uio pin is driven"
    assert padoe & case.ab_mask == 0, f"padoe {padoe:#06x}: an ABI wire is driven"
    assert padoe & 0x3E000 == 0x3E000, (
        f"padoe {padoe:#06x}: bits 13:17 are the output-only pins, which are always driven")
    assert b.drv.padout() & case.ab_mask == 0, "the chip wrote a level onto an ABI wire"
    assert b.drv.pinctrl & (0x1F << 29 | 0x7 << 26 | 0x1F << 20 | 0x1F << 10 | 0x1F << 5 | 0x1F) == 0, (
        "PINCTRL opened a window the program never uses")
    other = 1 - case.engine
    assert b.dev.chip.state(other)["pc"] == 0, "the other engine ran"
    assert not b.dev.chip.engines[other].rx, "the other engine received data"


# --------------------------------------------------------------------------------------------
# Configurations: two engines, two encoders, one chip
# --------------------------------------------------------------------------------------------

def test_two_encoders_on_two_engines_at_once() -> None:
    """Engine 0 on p0/p1/p2 and engine 1 on p3/p4/p5, each with its own encoder and checker.

    The program is position independent -- no relative addressing, and its only wrap is the
    two-instruction poll -- so the same seven words run twice over disjoint IN_BASE windows, which
    is what the MVP's two engines are for (`docs/mvp-isa.md` `### Engine conflicts`: "firmware
    that wants two independent protocols gives the engines disjoint windows").  Two different
    speeds and two different decoding factors at the same time.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    abi = {0, 1, 2, 3, 4, 5}
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k in abi))
    dev = bus.attach(ChipDevice())
    plan = [(0, 0, 10_000.0, 4), (1, 3, -3_000.0, 2)]
    rigs = []
    for engine, base, rpm, x in plan:
        nets = {"a_net": f"p{base + 1}", "b_net": f"p{base}", "i_net": f"p{base + 2}"}
        enc = attach_encoder(bus, ppr=1000, **nets)
        dec = attach_decoder(bus, x=x, counts_per_rev=enc.counts_per_rev,
                             initial_position=enc.initial_counts, **nets)
        chk = attach_checker(bus, profile=PROFILE, **nets)
        drv = QuadratureDriver(dev, bus, engine=engine, b_pin=base, x=x,
                               counts_per_rev=enc.counts_per_rev,
                               initial_position=enc.initial_counts)
        drv.load()
        drv.start()
        enc.queue_turns(0.02, rpm)
        rigs.append((drv, enc, dec, chk, rpm, x))

    assert dev.chip.state(0)["pc"] != 0 and dev.chip.state(1)["pc"] != 0, "both engines run"
    for _ in range(60_000):
        for drv, *_ in rigs:
            drv.step()
        if all(enc.idle for _drv, enc, *_ in rigs):
            break
    for drv, enc, dec, chk, rpm, x in rigs:
        drv.pump(300)
        drv.flush()
        sign = 1 if rpm > 0 else -1
        counts = int(round(0.02 * enc.counts_per_rev))
        assert drv.position == dec.position == enc.initial_counts + sign * (counts * x // 4), (
            f"engine {drv.engine}: chip {drv.position}, model {dec.position}, {counts} counts "
            f"at x{x}")
        assert drv.decoder.errors == [], [str(e) for e in drv.decoder.errors]
        assert chk.clean, chk.report()
        assert drv.fdebug() & FDEBUG_BAD == 0
    assert bus.errors == [], f"contention: {bus.errors[:2]}"
    assert all(drv.words_read for drv, *_ in rigs), "an engine pushed nothing"
    for drv, *_ in rigs:      # each engine is inside its poll loop, at one of its five words
        pc = drv.pc()
        assert drv.program.labels["wrap"] <= pc < len(drv.program.instrs), (
            f"engine {drv.engine} has PC {pc}, outside the loop")


# --------------------------------------------------------------------------------------------
# What the firmware does with a wire it cannot make sense of
# --------------------------------------------------------------------------------------------

class NetAlias(Device):
    """Puts a model on the chip's pin names by giving the chip a renaming device.

    `verif.vip.quadrature`'s devices take `a_net`/`b_net`/`i_net`, and everything except the
    glitch injector honours them.  `RotaryEncoder._tick_faults` looks the net up in a hard-coded
    `("a", "b", "i")` tuple, so `glitch()` raises `ValueError` on any other name -- verif/vip is
    read-only for this task, so the test drives the fault through this alias instead: the model
    keeps the names it knows and the alias does the renaming to `p0..p17`.
    """
    name = "pin_alias"

    def __init__(self, model_net: str, bus_net: str) -> None:
        self.model_net, self.bus_net = model_net, bus_net

    def tick(self, t: int, pins):
        return {self.bus_net: pins.get(self.model_net, 1)}


class Squarer(Device):
    """Drives one net on and off, for a test that wants a pin nobody else touches to move.

    `mov x, pins` is unmasked, so this is the pin the program reacts to without logging a step:
    the point of the test below.  It drives a net nothing else drives, so there is no contention.
    """

    def __init__(self, net: str, period: int = 3, name: str = "squarer") -> None:
        self.net, self.period, self.name = net, period, name

    def tick(self, t: int, pins):
        return {self.net: (t // self.period) & 1}


def test_a_moving_pin_that_is_not_an_encoder_is_logged_but_not_counted() -> None:
    """Traffic on another pin costs FIFO words and must not cost counts.

    The change detector is `mov x, pins`, which reads all of `IN_ROT`; program.pio says so and
    says why it is still the right instruction.  This test is the other half of that argument: a
    pin moving at 6.7 MHz produces no-change samples, which the Gray rule already says are not
    steps, so the position still matches the model's to the count.
    """
    case = CASES[0]
    b = build(case)
    b.bus.attach(Squarer("p7", period=3))
    b.drv.load()
    b.drv.start()
    b.enc.queue_turns(case.turns, case.rpm)
    assert b.drv.run_until(lambda: b.enc.idle, 200_000)
    b.drv.pump(300)
    b.drv.flush()
    dec = b.drv.decoder

    assert b.drv.position == b.dec.position == b.enc.position, (
        f"chip {b.drv.position}, model {b.dec.position}, encoder {b.enc.position}")
    assert dec.errors == [], [str(e) for e in dec.errors]
    assert dec.steps == len(b.enc.edges), "every real edge was still seen"
    assert dec.repeats, "the moving pin produced no no-change samples, so nothing was tested"
    assert dec.samples > dec.steps, "samples logged exceed edges seen"
    assert b.bus.errors == [], "the squaring pin was contended"


def test_a_skipped_state_is_reported_not_counted() -> None:
    """A wire that breaks the Gray cycle must not turn into motion, in either model.

    `RotaryEncoder.skip_states(1)` makes the encoder fail to put a state out, so two lines move
    at once -- not a state the four-state cycle contains.  Both decoders must refuse it: the
    model's because `step_delta` has no entry for a two-bit change, this driver's because it is
    the same function.  The two positions therefore still agree, on a number that is exactly one
    count short of the shaft: a count lost, never invented.  And the checker, which sees the wire
    rather than the counts, calls it `AB_ORDER`.
    """
    case = CASES[0]
    b = _fault_run(case, skip=1)
    dec = b.drv.decoder

    assert b.drv.position == b.dec.position, (
        f"chip {b.drv.position} against the model's {b.dec.position}")
    assert [str(e) for e in dec.errors], "the chip did not see the impossible transition"
    assert b.dec.errors, "the model's decoder did not see it either"
    assert all("impossible" in str(e) for e in dec.errors)
    assert abs(b.dec.position - b.enc.position) == 1, (
        "one skipped state is exactly one count of loss")
    assert b.chk.rules_broken() == {"AB_ORDER"}, b.chk.report()
    assert b.chk.violations_by_rule("AB_ORDER")[0].ceiling


def test_a_line_glitch_cancels_and_only_the_checker_sees_it() -> None:
    """One line moving and moving back is two legal steps that cancel -- and nothing else.

    This is what the reference model says about a pure pin disturbance
    (`verif/vip/test_quadrature.py::test_rule_ab_same_a_glitch_on_one_line_only` and
    `test_a_short_glitch_is_invisible_to_both_models`): the transitions are legal, so neither
    decoder can tell the glitch from the shaft going out and back, and the position is right.
    The checker is the one that sees it, as `AB_SAME`.  The firmware's job here is to be invisible
    in the count and to leave the wire alone, and it is.
    """
    case = CASES[0]
    b = _fault_run(case, glitch=("b", 2000))
    dec = b.drv.decoder

    assert b.drv.position == b.dec.position == b.enc.position, (
        f"chip {b.drv.position}, model {b.dec.position}, encoder {b.enc.position}")
    assert dec.errors == [] and b.dec.errors == [], "a glitch is not an impossible transition"
    assert b.chk.rules_broken() & {"AB_SAME"}, b.chk.report()


def _fault_run(case: Case, *, skip: int = 0, glitch: tuple | None = None) -> Bench:
    """Half a run on a legal wire, then a fault, then the rest, with everything flushed."""
    b = build(case, alias_pins=glitch is not None)
    b.drv.load()
    b.drv.start()
    b.enc.queue_turns(case.turns, case.rpm)
    half = b.counts() // 2
    assert b.drv.run_until(lambda: len(b.enc.edges) >= half, 200_000), "the run ended early"
    if skip:
        b.enc.skip_states(skip)
    if glitch:
        b.enc.glitch(*glitch)
    assert b.drv.run_until(lambda: b.enc.idle, 200_000)
    b.drv.pump(300)
    b.drv.flush()
    return b


def test_a_host_that_stops_reading_is_reported_not_silently_wrong() -> None:
    """What happens when the host is slower than the shaft: the chip says so, and loses nothing
    it claims.

    With the drain switched off the RX FIFO fills after four words, the `in pins, 3` that reaches
    PUSH_THRESH stalls (FDEBUG.RXSTALL), and while the engine is stalled two edges can pass
    unobserved.  The chip then logs a two-bit jump, which `HostDecoder` reports as an impossible
    transition.  Both facts are visible from the host side, and the count is short by exactly the
    edges that went by while the chip was stuck -- never long, and never silently.
    """
    case = CASES[0]
    b = build(case, drain=False)
    b.drv.load()
    b.drv.start()
    b.enc.queue_turns(case.turns, case.rpm)
    assert b.drv.run_until(lambda: b.enc.idle, 200_000)
    b.drv.pump(300)             # no flush: flushing is the host reading, which is the thing
    fdebug = b.drv.fdebug()     # this test is about not doing
    dec = b.drv.decoder

    assert fdebug & FDEBUG_RXSTALL, f"FDEBUG {fdebug:#010x}: a full FIFO did not stall the chip"
    assert fdebug & FDEBUG_BAD & ~FDEBUG_RXSTALL == 0, (
        f"FDEBUG {fdebug:#010x}: RXSTALL is the only flag this run may set")
    assert dec.samples == 0, "auto_read was off, so no word reached the host"
    assert b.drv.rx_level() == b.drv.RXFIFO_DEPTH, f"the FIFO holds {b.drv.rx_level()} words"
    # now the host reads what is there, and the decoder has it
    got = b.drv.drain()
    assert got >= b.drv.RXFIFO_DEPTH, f"the FIFO held {got} words"
    assert b.drv.rx_level() == 0, "draining did not empty the FIFO: the chip did not recover"
    per_word = b.drv.shape["samples_per_word"]
    assert dec.samples == got * per_word, "every queued word arrived, ten samples each"
    assert b.drv.position < b.dec.position, (
        "a chip that stalled must be short of the wire, never ahead of it")


# --------------------------------------------------------------------------------------------
# What the program is, and how fast it can go
# --------------------------------------------------------------------------------------------

def test_the_program_is_seven_words_and_says_what_it_does() -> None:
    """program.pio assembles, fits the IMEM, and the driver reads its shape back out of it.

    The numbers the driver puts in SHIFTCTRL and the speed numbers it reports come from
    `describe()`, so they are read out of the words rather than typed in: if program.pio changes,
    this test is what notices.
    """
    program = load_program()
    shape = describe(program)
    assert program.name == "quadrature"
    assert words_used(program) == 7, [f"{w:#06x}" for w in program.instrs]
    assert words_used(program) <= IMEM_SIZE
    assert program.labels == {"start": 0, "wrap": 2, "moved": 3, "changed": 4}, program.labels
    assert program.wrap == 3 and program.wrap_target == 2
    assert program.sideset_bits == 0, "the decoder side-sets nothing"
    assert shape == {"sample_bits": 3, "unchanged": 2, "changed": 5, "sync_word": 0x4003,
                     "log_word": 0x4003, "baseline_word": 0xA040, "push_thresh": 30,
                     "samples_per_word": 10}, shape
    # the words themselves, so a change to program.pio has to be a deliberate one
    assert [f"{w:#06x}" for w in program.instrs] == ["0x4003", "0xa040", "0xa020", "0x00a4",
                                                     "0x4003", "0xa041", "0x0002"]
    assert unpack_word(0x085E_CCC8, program) == [1, 0, 2, 7, 5, 4, 6, 3, 1, 0], (
        "the packing of a word: oldest sample in bits 29:27, newest in 2:0, bits 31:30 zero")


def test_the_fastest_encoder_the_chip_can_follow() -> None:
    """The speed ceiling, from the words and the divider, against the profile's own limit.

    Two different numbers, and it matters which is which:

    * `min_step_ticks` -- the poll grid, two SM cycles.  Two edges closer together than this fall
      between two reads of the pins and the chip never sees the state in between.
    * `ticks_per_sample` -- the `changed` path, five SM cycles.  This is the rate at which the
      chip can *log* an edge, and it is the one that binds: an encoder faster than this outruns
      the loop, and the host then sees samples the Gray cycle cannot explain.

    Both are far above the AS5047P's 28 000 RPM, which is the point: CLKDIV = 1.0 is enough and
    the divider never has to be touched.
    """
    case = CASES[0]
    b = cached(case)
    drv = b.drv
    assert drv.clkdiv == 0x0001_0000 and ticks_per_sm_cycle(drv.clkdiv) == 1.0
    assert drv.min_step_ticks == 2.0, "two SM cycles per poll at CLKDIV 1.0"
    assert drv.ticks_per_sample == 5.0, "five SM cycles per logged sample"
    assert drv.max_sample_rate == CORE_HZ / 5 == 10_000_000.0

    limits = profile_minimums(NS_PER_TICK, PROFILE)
    steps_per_rev = 4 * case.ppr
    profile_rpm = limits["max_rpm"]
    profile_ticks = 750_000.0 / profile_rpm          # RotaryEncoder.ticks_per_count
    assert profile_ticks == pytest.approx(26.786, rel=1e-3)
    assert drv.min_step_ticks < limits["min_edge"], (
        f"the chip's {drv.min_step_ticks:.0f}-tick poll grid against the profile's "
        f"{limits['min_edge']}-tick minimum edge")
    # the fastest legal AS5047P edge is 26.8 ticks; the chip polls every 2 and logs every 5
    assert drv.max_rpm > profile_rpm, f"{drv.max_rpm:,.0f} RPM against the profile's {profile_rpm:,.0f}"
    log_rpm = CORE_HZ / drv.ticks_per_sample * 60.0 / steps_per_rev
    assert 100_000 < log_rpm < 1_000_000, f"{log_rpm:,.0f} RPM is outside what the words predict"


def test_max_speed_on_the_iss() -> None:
    """The empirical ceiling: the fastest encoder this program follows with no lost count.

    A sweep, and it writes what it measures to `metrics/quadrature_max_speed.csv` so the README
    can quote numbers instead of adjectives.  Each row is a fresh bench: an encoder turned a
    fixed small number of counts at that speed, with the chip, the model's decoder and the
    encoder's own position all compared.  The sweep starts at the profile's 28 000 RPM and doubles,
    which is cheap because a fast encoder needs few ticks: at 200 000 RPM one count is 3.75 ticks.

    What the words predict is 10 M logged samples/s, i.e. 150 000 RPM at 4000 counts per
    revolution; what the run finds has to be of that order and well past the profile's limit.
    """
    rows = []
    for rpm in (28_000, 56_000, 112_000, 224_000, 448_000, 896_000, 1_792_000, 3_584_000):
        case = Case(f"sweep-{rpm}", rpm, 0.002)          # 8 counts, whatever the speed
        b = run(case, limit=400_000)
        agree = (b.drv.position == b.dec.position == b.enc.position)
        # RotaryEncoder.ticks_per_count: one count is 750000 / rpm ticks at 20 ns per tick.
        rows.append({
            "rpm": rpm,
            "ticks_per_count": round(750_000.0 / rpm, 3),
            "poll_ticks": b.drv.min_step_ticks,
            "log_ticks": b.drv.ticks_per_sample,
            "counts": b.counts(),
            "chip_position": b.drv.position,
            "model_position": b.dec.position,
            "agree": agree,
            "errors": len(b.drv.decoder.errors),
            "violations": len(b.chk.violations),
            "words": b.drv.words_read,
        })
    limits = profile_minimums(NS_PER_TICK, PROFILE)
    profile_rpm = limits["max_rpm"]
    ok = [r["rpm"] for r in rows if r["agree"]]
    bad = [r["rpm"] for r in rows if not r["agree"]]
    assert profile_rpm in ok, f"the chip must follow the whole profile range: {rows}"
    assert ok, "the chip followed nothing"
    fastest = max(ok)
    assert fastest > profile_rpm, f"only {fastest:,.0f} RPM, the profile allows {profile_rpm:,.0f}"
    assert fastest >= 112_000, f"the words predict about 150 000 RPM, the run found {fastest:,.0f}"

    _write_sweep(rows, limits)
    lost = f", lost counts from {bad[0]:,.0f} RPM" if bad else ", still following at the top"
    print(f"\nquadrature max speed: {fastest:,.0f} RPM, "
          f"{fastest / profile_rpm:.1f}x the AS5047P's {profile_rpm:,.0f} RPM{lost}")


def _write_sweep(rows: list[dict], limits: dict) -> None:
    """The sweep as a CSV, so the numbers in the README have a file behind them."""
    path = Path(__file__).resolve().parents[2] / "metrics" / "quadrature_max_speed.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["rpm", "ticks_per_count", "poll_ticks", "log_ticks", "counts", "chip_position",
              "model_position", "agree", "errors", "violations", "words"]
    head = [f"# quadrature decoder speed sweep, ISS at 20 ns/tick (50 MHz), CLKDIV 1.0",
            f"# program.pio: {words_used(load_program())} words, poll grid "
            f"{describe(load_program())['unchanged']} SM cycles, "
            f"{describe(load_program())['changed']} SM cycles per logged sample",
            f"# as5047p profile: min_edge {limits['min_edge']} ticks, min_period "
            f"{limits['min_period']} ticks, max {limits['max_rpm']:.0f} RPM "
            f"(= {750_000.0 / limits['max_rpm']:.2f} ticks per count)",
            f"# poll_ticks is the chip's sampling grid and log_ticks the cost of logging one "
            f"edge; the chip can only follow an encoder while ticks_per_count > log_ticks",
            f"# checker violations are the wire against that profile, so they are expected to "
            f"appear above {limits['max_rpm']:.0f} RPM: that part is not an AS5047P"]
    lines = [",".join(fields)] + [",".join(str(r[f]) for f in fields) for r in rows]
    path.write_text("\n".join(head + lines) + "\n", encoding="utf-8")


def test_driver_rejects_setups_it_cannot_honour() -> None:
    """Guard rails, so a test that mis-configures a decoder fails loudly.

    The interesting one is the contiguity rule: `in pins, 3` reads three pins *in a row* from
    IN_BASE, so an A/B/I triple with a hole in it cannot be sampled by one instruction, and a
    window that runs off the end of the 18 real pins is not a sample at all.
    """
    case = CASES[0]
    b = build(case)
    for kwargs in ({"engine": 2}, {"x": 3}, {"channel": "c"},
                   {"b_pin": 1, "a_pin": 2, "i_pin": 4},        # not contiguous
                   {"b_pin": 16},                               # 16, 17, 18: 18 is phantom
                   {"b_pin": 17},                               # runs off the end
                   {"b_pin": -1}):
        with pytest.raises(ValueError):
            QuadratureDriver(b.dev, b.bus, **kwargs)
    with pytest.raises(ValueError):
        pinctrl_value(18)
    with pytest.raises(ValueError):
        shiftctrl_value(push_thresh=0)
    with pytest.raises(ValueError):
        shiftctrl_value(in_shiftdir=2)
    with pytest.raises(ValueError):
        jmp_abs(32)
    with pytest.raises(ValueError):
        clkdiv_value(0)
    drv = QuadratureDriver(b.dev, b.bus)
    drv.load()
    assert drv.words_used == 7
    drv.arm()
    assert drv.pc() == drv.start_addr, "arm() forces `jmp start`, and that is the only way in"


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: every broken program has to fail the transaction
# --------------------------------------------------------------------------------------------

MUT_CASE = CASES[0]                  # 10 000 RPM forward, x4, the plainest configuration


def _word(program, label: str) -> int:
    return program.instrs[program.labels[label]]


def _mutate_count(new: int):
    """A function that replaces the bit count of the `in pins, N` at `changed`."""
    def apply(program: Program) -> None:
        at = program.labels["changed"]
        program.instrs[at] = (program.instrs[at] & ~0x1F) | new
    return apply


def _replace(label: str, new_word: int):
    def apply(program: Program) -> None:
        program.instrs[program.labels[label]] = new_word
    return apply


def _break_sync(program: Program) -> None:
    """Replace the sync `in pins, 3` at `start` with a nop: the first edge is then a baseline."""
    program.instrs[program.labels["start"]] = 0xA042


def _break_wrap(program: Program) -> None:
    """Point WRAP_TOP at address 0 so the poll runs off the end of the program."""
    program.wrap = 0


def _bad_pinctrl(drv: QuadratureDriver) -> None:
    """IN_BASE on the wrong pin: the sample reads B, A and a pin nobody drives."""
    drv.write(drv.sm_addr(driver_mod.SM_PINCTRL), pinctrl_value(3))
    drv._settle()


def _bad_shiftctrl(drv: QuadratureDriver) -> None:
    """IN_SHIFTDIR = 1: the samples land in the top of the word and `unpack_word` misreads them."""
    drv.write(drv.sm_addr(driver_mod.SM_SHIFTCTRL), shiftctrl_value(in_shiftdir=1))
    drv._settle()


def _bad_push_thresh(drv: QuadratureDriver) -> None:
    """PUSH_THRESH for eight samples per word instead of ten: the host unpacks ten.

    29 would be a no-op -- isr_cnt is 3, 6, ... 27, 30, so a threshold of 29 still fires on the
    tenth sample -- but 24 fires on the ninth, and every word then holds eight samples where
    `unpack_word` reads ten.  The two samples of padding are zeros, which the host cannot tell
    from real samples of state 00, so this is the same trap `flush()` is written to avoid.
    """
    drv.write(drv.sm_addr(driver_mod.SM_SHIFTCTRL), shiftctrl_value(push_thresh=24))
    drv._settle()


MUTATIONS = [
    ("the log samples one pin instead of three", _mutate_count(1), None),
    ("the log samples two pins, so the index is lost", _mutate_count(2), None),
    ("the sync sample is gone", _break_sync, None),
    ("the detector reads its own baseline", _replace("wrap", 0xA021), None),   # mov x, y
    ("the baseline is never updated", _replace("changed", 0xA042), None),     # the log -> nop
    ("WRAP_TOP points into the middle", _break_wrap, None),
    ("IN_BASE is on the wrong pin", None, _bad_pinctrl),
    ("IN_SHIFTDIR is the reset value", None, _bad_shiftctrl),
    ("PUSH_THRESH is not a whole number of samples", None, _bad_push_thresh),
]


def run_broken(program_mutator=None, after_load=None) -> dict:
    """The same run as MUT_CASE with one thing broken, and what came of it.

    A program that no longer has the shape the driver derives its timing from is refused before it
    reaches the chip (`describe()`), which is one of the two ways a broken program is caught and
    the better one; the register mutations get past that and have to be caught by the transaction.
    """
    out = {"refused": None, "chip": None, "model": None, "errors": [], "fdebug": None}
    b = build(MUT_CASE)
    if program_mutator is not None:
        program = copy.deepcopy(load_program())
        program_mutator(program)
        b.drv.program = program
        b.drv.decoder.program = program
        b.drv._loaded = False
    try:
        b.drv.load()
        b.drv.shape = describe(b.drv.program)
        if after_load is not None:
            after_load(b.drv)
        b.drv.start()
        b.enc.queue_turns(MUT_CASE.turns, MUT_CASE.rpm)
        assert b.drv.run_until(lambda: b.enc.idle, 200_000)
        b.drv.pump(300)
        b.drv.flush()
    except ValueError as exc:
        out["refused"] = str(exc)
        return out
    out.update(chip=b.drv.position, model=b.dec.position, errors=list(b.drv.decoder.errors),
               fdebug=b.drv.fdebug())
    return out


def test_the_clean_run_is_clean() -> None:
    """The baseline every mutation is compared against: 200 counts, no errors, no flags."""
    got = run_broken()
    assert got["refused"] is None, got["refused"]
    assert (got["chip"], got["model"], got["errors"], got["fdebug"]) == (196, 196, [], 0), got


@pytest.mark.parametrize("name,mutate,after_load", MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_a_broken_program_does_not_count_the_shaft(name: str, mutate, after_load) -> None:
    """Each mutation is caught: refused by the driver's guard rails, or caught by the count.

    The count is the main check -- the chip's number against the model's -- plus the
    two ways a firmware can be wrong without being wrong about the count: a flag in FDEBUG, or an
    impossible transition in the log.  A mutation that produced the right answer would be a hole
    in this test, so each one has to move the number, the flag or the error list, or be refused
    outright with a message that says which word is wrong.
    """
    got = run_broken(mutate, after_load)
    if got["refused"] is not None:
        assert "quadrature" in got["refused"], got["refused"]
        return
    noticed = (got["chip"] != got["model"]) or bool(got["errors"]) or got["fdebug"] != 0
    assert noticed, (
        f"{name}: the check did not notice (chip {got['chip']}, model {got['model']}, "
        f"{len(got['errors'])} errors, FDEBUG {got['fdebug']:#010x})")


REGISTER_MUTATIONS = [m for m in MUTATIONS if m[2] is not None]


@pytest.mark.parametrize("name,mutate,after_load", REGISTER_MUTATIONS,
                         ids=[m[0] for m in REGISTER_MUTATIONS])
def test_a_broken_register_setting_is_caught_by_the_count(name: str, mutate, after_load) -> None:
    """The mutations that get past the guard rails are caught by the transaction, not by a raise.

    A register the driver does not check is the realistic mistake: the words are right and the
    configuration is wrong.  These three are exactly that, and each one moves the count, so the
    comparison against the model -- not a `ValueError` -- is what catches them.
    """
    got = run_broken(mutate, after_load)
    assert got["refused"] is None, f"{name} was refused instead of counted: {got['refused']}"
    noticed = (got["chip"] != got["model"]) or bool(got["errors"]) or got["fdebug"] != 0
    assert noticed, f"{name}: chip {got['chip']}, model {got['model']}, {got['errors']}"

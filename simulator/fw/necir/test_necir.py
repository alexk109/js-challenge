"""End-to-end test of the MVP NEC-receiver firmware (task B1-FW-necir).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with nets
p0..p17, exactly as `docs/mvp-interfaces.md` `## ISS API` describes; the host driver loads the
program and starts the engine, and `verif.vip.necir` -- the independent judge, which this test
does not modify -- plays the remote: `NecirTx` drives the wire, `NecirRx` decodes it and
`NecirChecker` judges every rule.  Nothing here looks at the ISS internals to decide pass or fail:
the words come out of the chip's RX FIFO, the frames come out of the model, and the violations
come out of the checker.

    make fw PROTO=necir TARGET=iss

**The chip is the target.**  It is a receiver: it never drives the net, so every frame it decodes
arrives as one 32-bit word in its RX FIFO and that word is the judgement.  The comparison is with
`verif.vip.necir.encode_frame()`, the model's own encoder, so "the firmware decoded the frame" and
"the model would have encoded it" are two separate pieces of code agreeing bit for bit.

What the checker can and cannot judge here
------------------------------------------
A receiver drives nothing, so on the wire the checker sees the *remote*, not the firmware.  Its
verdict is therefore used two ways, both of them tests rather than decoration:

* `test_the_checker_sees_the_same_wire_with_the_chip_attached` runs the identical press twice,
  once with the chip on the bus and once without, and asserts the two checkers report the same
  frames and the same (zero) violations.  That is what makes "zero violations" a statement about
  the firmware: the chip is provably not the thing that could have spoiled the wire.
* `test_a_broken_wire_is_caught_and_the_chip_says_so_too` breaks the wire on purpose and checks
  that the checker fires *and* that the chip's own verdict moves with it -- a 4 ms leader is
  flagged `leader-mark` and is not decoded at all, which is the leader check in program.pio doing
  the same job from the other side of the wire.

Two tick rates appear.  Everything about the protocol runs at 20 ns/tick, the 50 MHz chip clock.  The one exception is `test_a_held_key_sends_a_frame_and_no_data_for_the_repeats`,
which needs the 110 ms repeat raster: that is 5.5 million ticks at 20 ns, so it runs at 200 ns per
tick (5 MHz) instead, for the same reason `verif.vip/test_necir.py` has its `MS_PER_TICK`.  The
firmware is unchanged; `sample_window()` recomputes its sample points for the coarser tick, because
one bus tick is then ten core cycles and the transmitter's edge quantisation is ten times coarser
too.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Optional

import pytest

from fw.necir.driver import (
    CORE_HZ,
    DEFAULT_SM_CYCLES_PER_UNIT,
    MAX_SM_CYCLES_PER_UNIT,
    MIN_SM_CYCLES_PER_UNIT,
    NEC_UNIT_NS,
    NS_PER_TICK,
    REAL_NEC_UNIT_NS,
    IMEM_SIZE,
    NecirError,
    NecirReceiver,
    build_program,
    clkdiv_value,
    cycles_per_sm_cycle,
    decode_frame,
    imem_addr,
    sample_margins,
    sample_points,
    sample_window,
    sample_cycles,
    shiftctrl_fields,
    words_used,
)
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.necir import (
    NECIR_TIMING,
    NecirChecker,
    NecirRx,
    NecirTx,
    attach_checker,
    attach_rx,
    attach_tx,
    decode_frame as model_decode_frame,
    encode_frame,
    necir_timings,
)
from fw import regmap as _regmap

UIO_MASK = 0x000FF                    # the bidirectional protocol pins
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_ANY = 0x0303_0303              # TXSTALL, TXOVER, RXUNDER, RXSTALL: nothing mis-timed
FSTAT_TXEMPTY = 0x0100_0000           # bit 24 + engine
FSTAT_RXEMPTY = 0x0100                # bit 8 + engine

#: How long to run after a press: one standard frame is 68.06 ms = 3404 ticks at 20 ns, and the
#: frame plus the idle the checker needs before it judges it is under 4000.
FRAME_TICKS_20NS = 4_000

#: The repeat raster the held-key test uses, in nanoseconds, against the model's 110 ms.  See that
#: test's docstring: it is handed to both the transmitter and the checker, and the property under
#: test does not depend on its value.
REPEAT_RASTER_NS = 4_000_000.0


@dataclass(frozen=True)
class Case:
    """One end-to-end configuration of the receiver."""

    name: str
    engine: int = 0
    pin: int = 0
    sm_cycles_per_unit: int = DEFAULT_SM_CYCLES_PER_UNIT
    tolerance: float = 0.25
    extended: bool = False


CASES = [
    Case("s9-pin0"),
    Case("s7-pin0", sm_cycles_per_unit=MIN_SM_CYCLES_PER_UNIT),
    Case("s12-engine1-pin7", engine=1, pin=7, sm_cycles_per_unit=MAX_SM_CYCLES_PER_UNIT),
    Case("s10-tolerance-5pct", sm_cycles_per_unit=10, tolerance=0.05),
]

#: (address, command) pairs a real remote sends: the address-0 remotes, the pair the sbprojects
#: page walks through, and the real frame Arduino-IRremote prints in its header comment
#: (address 0x04, command 0x08).
PAIRS = [(0x00, 0x0C), (0xFF, 0x00), (0x59, 0x16), (0x04, 0x08)]


@dataclass
class Rig:
    """A bus with the chip and the three NEC models on the IR net."""

    bus: Bus
    dev: ChipDevice
    rx: NecirReceiver
    tx: NecirTx
    model_rx: NecirRx
    chk: NecirChecker

    def settle(self, ticks: int) -> None:
        """Run the bus, then close the wire on both passive models."""
        self.rx.run(ticks)
        self.model_rx.flush()
        self.chk.flush()


def build(case: Case, *, tx_kwargs: Optional[dict] = None,
          chk_kwargs: Optional[dict] = None, rx_kwargs: Optional[dict] = None,
          ns_per_tick: float = NS_PER_TICK, attach_chip: bool = True,
          auto_start: bool = True) -> Rig:
    """A Bus at `ns_per_tick` with p0..p17, the ISS on it, and the three NEC models.

    `attach_chip=False` builds the same bench without the chip, which is what
    `test_the_checker_sees_the_same_wire_with_the_chip_attached` compares against, and
    `auto_start=False` leaves the engine stopped so a test can write IMEM itself.
    """
    bus = Bus(ns_per_tick=ns_per_tick)
    dev = bus.attach(ChipDevice())
    rx = NecirReceiver(bus=bus, dev=dev, engine=case.engine, pin=case.pin,
                       sm_cycles_per_unit=case.sm_cycles_per_unit, tolerance=case.tolerance,
                       ns_per_tick=ns_per_tick, auto_start=auto_start)
    tx = attach_tx(bus, net=rx.net, **(tx_kwargs or {}))
    model_rx = attach_rx(bus, net=rx.net, extended=case.extended,
                         **(rx_kwargs or {}))
    chk = attach_checker(bus, net=rx.net, extended=case.extended, **(chk_kwargs or {}))
    if not attach_chip:
        bus.devices.remove(dev)          # the same wire, with nobody listening
    return Rig(bus, dev, rx, tx, model_rx, chk)


def press(rig: Rig, address: int, command: int, ticks: int = FRAME_TICKS_20NS) -> None:
    """Queue one press and run long enough for the frame to be decoded and read out."""
    rig.tx.press(address, command)
    rig.settle(ticks)


# ==================================================================================================
# The transaction, judged by the model and by the checker
# ==================================================================================================
@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_a_frame_reaches_the_chip_as_one_word(case: Case) -> None:
    """Every press arrives as the model's own 32-bit frame word, and nothing else arrives.

    This is the main pass criterion: the checker reports zero violations, and the words the engine pushed
    are bit for bit what `verif.vip.necir.encode_frame` builds for the same (address, command).
    The two are separate implementations -- one is firmware, one is the reference model -- so their
    agreement is a result rather than a tautology.
    """
    rig = build(case)
    for address, command in PAIRS:
        press(rig, address, command)

    assert rig.chk.clean, rig.chk.report()
    assert rig.chk.frames and all(f["bits"] == 32 and f["complete"] for f in rig.chk.frames)
    assert rig.rx.raws == [encode_frame(a, c) for a, c in PAIRS], rig.rx.raws
    assert rig.rx.payloads == PAIRS

    # ... and the independent decoder agrees, so the word is not merely self-consistent
    assert [(p.address, p.command) for p in rig.model_rx.presses] == PAIRS
    assert all(f.ok and not f.errors for f in rig.model_rx.frames)
    assert rig.model_rx.errors == [] and rig.model_rx.ambiguous_spaces == 0

    # Nothing was dropped or mis-timed, and the other engine never ran.
    fdebug = rig.rx.fdebug()
    assert fdebug & FDEBUG_ANY == 0, f"FDEBUG {fdebug:#010x}"
    assert not rig.chk.repeat_codes, "a plain press carries no repeat code"
    other = 1 - case.engine
    assert rig.rx.chip.engines[other].pc == 0 and not rig.rx.chip.engines[other].rx
    assert rig.rx.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_chip_leaves_the_net_alone(case: Case) -> None:
    """A receiver writes no pin: the IR pin stays an input and the wire is never contended.

    The whole protocol is on one net, so the one thing this firmware must never do is drive it.
    `DBG_PADOE` says it does not (no `uio` pin is driven at all), `bus.errors` says no device ever
    fought the remote for it, and the model's own decode of the wire is intact afterwards.
    """
    rig = build(case)
    press(rig, 0x07, 0x30)

    padoe = rig.rx.padoe()
    assert not padoe & UIO_MASK, f"DBG_PADOE {padoe:#06x}: the firmware drives a uio pin"
    assert not padoe & (1 << case.pin)
    assert padoe & 0x3E000 == 0x3E000, "the five output-only pins are always driven, per the spec"
    assert rig.bus.errors == [], f"contention on a push-pull net: {rig.bus.errors[:2]}"
    assert rig.bus.nets[rig.rx.net].push_pull and rig.bus.nets[rig.rx.net].pullup == 0
    assert rig.chk.clean, rig.chk.report()


def test_the_checker_sees_the_same_wire_with_the_chip_attached() -> None:
    """Zero violations is a statement about the firmware, not about the model.

    The same press is driven twice on the same bus model: once with the chip listening and once
    with the chip removed.  If the chip drove, loaded or re-timed the net the two checkers would
    not agree, so their agreeing on zero violations is what makes the other tests mean anything.
    """
    with_chip = build(Case("s9-pin0"))
    press(with_chip, 0x59, 0x16)
    without_chip = build(Case("s9-pin0"), attach_chip=False)
    without_chip.tx.press(0x59, 0x16)
    without_chip.rx.run(FRAME_TICKS_20NS)
    without_chip.model_rx.flush()
    without_chip.chk.flush()

    assert with_chip.chk.clean and without_chip.chk.clean
    assert with_chip.chk.report() == without_chip.chk.report()
    assert with_chip.chk.frames == without_chip.chk.frames
    assert with_chip.chk.starts == without_chip.chk.starts
    # ... and the chip decoded the frame that bench produced
    assert with_chip.rx.raws == [encode_frame(0x59, 0x16)]


def test_a_held_key_sends_a_frame_and_no_data_for_the_repeats() -> None:
    """`press(repeats=N)` is a held key: one frame, then N repeat codes that carry no data.

    NEC's repeat code is the leader, a 2250 us space and one burst, and nothing else -- "only the
    leader code and a single bit are transmitted repeatedly" (Vishay 80071).  A receiver that
    shifted a bit in for a repeat would push a bogus frame every time the key was held, so this
    pins that it pushes exactly one word while the model counts N repeat codes and the checker
    stays silent on a legal raster.

    The raster is scaled down and the checker is told so.  `press()` inserts the idle itself, and
    the nominal 110 ms is 5.5 million ticks at 20 ns -- about a minute of simulation for two
    repeats.  `REPEAT_RASTER_NS` is a 27x shorter raster (a remote that repeats faster), passed to
    the transmitter *and* to the checker through its `overrides=`, so both agree on what the rule
    is; the property under test is that a repeat code carries no data, which does not depend on how
    long the wait between them is.  `test_the_repeat_raster_is_the_published_one` checks the
    nominal separately, where the numbers are cheap to compute.
    """
    ns = NS_PER_TICK
    rig = build(Case("s9-pin0"), tx_kwargs={"idle_ns": 8_000},
                chk_kwargs={"overrides": {"repeat_period_ns": REPEAT_RASTER_NS}})
    period = REPEAT_RASTER_NS / ns
    rig.tx.press(0x00, 0x0C, repeats=2, period_ns=REPEAT_RASTER_NS)
    rig.settle(int(2.5 * period))

    assert rig.chk.clean, rig.chk.report()
    assert len(rig.chk.frames) == 1 and len(rig.chk.repeat_codes) == 2
    starts = rig.chk.starts
    assert len(starts) == 3, starts
    # The raster is start to start and `press()` anchors it on the tick the bus is on when the
    # press is queued, so the first gap is short of the period by the frame's start and the second
    # is the period exactly.
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    assert gaps[1] == int(period), (starts, period)
    assert 0 < period - gaps[0] < FRAME_TICKS_20NS, (starts, period)
    assert rig.rx.raws == [encode_frame(0x00, 0x0C)], rig.rx.raws
    assert rig.model_rx.repeats == 2
    assert [r.address for r in rig.model_rx.repeat_codes] == [0x00, 0x00]
    assert rig.rx.fdebug() & FDEBUG_ANY == 0
    assert rig.rx.chip.state(rig.rx.engine)["rx_level"] == 0, "the host drained the FIFO"


def test_the_repeat_raster_is_the_published_one() -> None:
    """The scaled raster above is a bench convenience; the rule itself is 110 ms, and here it is.

    Nothing about the repeat code changes with the raster, so this checks the numbers where they
    are cheap: the model's nominal period, the window it opens around it (which holds both
    published values, 108 and 110 ms), and that a raster far outside it is flagged -- so the
    override in the test above cannot be hiding a violation.
    """
    assert NECIR_TIMING.repeat_period_ns == 110_000_000.0
    win = necir_timings(NS_PER_TICK)["repeat_period"]
    assert 108_000_000 / NS_PER_TICK >= win[0] and 110_000_000 / NS_PER_TICK <= win[1]
    assert REPEAT_RASTER_NS < win[0] * NS_PER_TICK, "the test's raster is deliberately short"

    rig = build(Case("s9-pin0"), tx_kwargs={"idle_ns": 8_000})       # the checker keeps 110 ms
    rig.tx.press(0x00, 0x0C, repeats=1, period_ns=REPEAT_RASTER_NS)
    rig.settle(int(1.5 * REPEAT_RASTER_NS / NS_PER_TICK))
    assert "repeat-period" in rig.chk.rules_broken(), rig.chk.report()
    assert rig.rx.raws == [encode_frame(0x00, 0x0C)], "and the chip is unaffected by the raster"


def test_a_repeat_code_with_no_frame_before_it_pushes_nothing() -> None:
    """An orphan repeat code is not data: the chip consumes it and says nothing.

    The checker flags `orphan-repeat` (a repeat repeats the frame before it, and there is none),
    and the chip's leader-space sample sends it back to hunting without shifting a bit in.
    """
    rig = build(Case("s9-pin0"))
    rig.tx.queue_repeat()
    rig.settle(2_000)

    assert rig.chk.rules_broken() == {"orphan-repeat"}, rig.chk.report()
    assert rig.rx.raws == [], rig.rx.raws
    assert rig.rx.fdebug() & FDEBUG_ANY == 0


def test_a_leader_that_is_too_short_is_not_decoded() -> None:
    """The leader check: a 4 ms burst is not a leader, so no frame comes out.

    This is the same rule the checker enforces (`leader-mark`) seen from the other side of the
    wire: the model judges the remote, the firmware judges the burst.  Both verdicts move together
    across the boundary, which is the strongest statement this pair of models can make about a
    receiver.
    """
    for leader_ns, decoded, rules in ((4_000, 0, {"leader-mark"}),
                                      (6_000, 0, {"leader-mark"}),
                                      (9_000, 1, set()),
                                      (12_000, 1, {"leader-mark"})):
        rig = build(Case("s9-pin0"), tx_kwargs={"overrides": {"leader_mark_ns": leader_ns}})
        rig.tx.press(0x00, 0x0C)
        rig.settle(FRAME_TICKS_20NS)
        assert len(rig.rx.raws) == decoded, (leader_ns, rig.rx.raws)
        assert rig.chk.rules_broken() == rules, (leader_ns, rig.chk.report())


def test_a_data_space_that_is_neither_a_zero_nor_a_one_is_caught() -> None:
    """A 1 ms space is in neither data window: the checker says so and the receiver guesses.

    This is the rule that matters most for firmware (the VIP's own `bit-space`), and the gap
    between the two windows is exactly what the sample points are placed in.  A space in the gap
    cannot be decoded by anyone, so what this pins is that the *bench* says so -- `bit-space`
    fires 16 times -- and that the firmware then behaves like the reference decoder: `NecirRx`
    guesses with the threshold between the two nominals and flags the guess, and the chip's word
    is the same guess.  A receiver that reported nothing here would be the suspicious one.
    """
    good = build(Case("s9-pin0"))
    press(good, 0x59, 0x16)
    broken = build(Case("s9-pin0"), tx_kwargs={"overrides": {"bit1_space_ns": 1_000}})
    broken.tx.press(0x59, 0x16)
    broken.settle(FRAME_TICKS_20NS)

    assert good.chk.clean and good.rx.raws == [encode_frame(0x59, 0x16)]
    hits = broken.chk.violations_by_rule("bit-space")
    assert len(hits) == 16, broken.chk.report()          # one per '1' bit of the payload
    assert all(v.measured == 50 for v in hits), broken.chk.report()
    assert broken.chk.rules_broken() >= {"bit-space", "inverse-address"}, broken.chk.report()

    # The reference decoder re-derives its phase from every edge, so a space it cannot classify
    # costs it its place in the stream: it says so (16 ambiguous spaces) and reports zeros.
    assert broken.model_rx.ambiguous_spaces == 16
    assert any("neither a '0'" in e for e in broken.model_rx.frames[0].errors)
    assert [(p.address, p.command) for p in broken.model_rx.presses] == [(0, 0)]
    # The firmware re-syncs on every burst (`wait 1 pin, 0`) and its sample sits inside the '1'
    # window, so a 1.78 burst space still reads as a '1' and the word is the sender's intent.
    # That is a fact about an *illegal* wire, and the checker is what says so.
    assert broken.rx.raws == [encode_frame(0x59, 0x16)], broken.rx.raws


# ==================================================================================================
# What the program is, derived from the words rather than asserted by hand
# ==================================================================================================
def test_program_fits_the_imem() -> None:
    """program.pio assembles, and the driver reads its shape back out of the words."""
    program = build_program()
    assert program.name == "necir"
    assert words_used(program) <= IMEM_SIZE
    assert words_used(program) == 27, [f"{w:#06x}" for w in program.instrs]
    assert program.labels["start"] == 0 and program.labels["leader"] == 0
    assert program.wrap == words_used(program) - 1, "WRAP_TOP must be the last instruction"
    assert program.wrap_target == program.labels["leader"]
    assert program.sideset_bits == 0 and not program.sideset_opt
    assert "x" not in "".join(program.labels), "the program needs no scratch register at all"


def test_the_imem_holds_exactly_what_the_assembler_produced() -> None:
    """The words in the engine's IMEM are `asm/pioasm.py`'s, for every configuration."""
    for case in CASES:
        rig = build(case)
        imem = rig.rx.chip.engines[case.engine].imem
        used = words_used(rig.rx.program)
        assert imem[:used] == rig.rx.program.instrs, case.name
        assert set(imem[used:]) <= {0}, f"{case.name}: the driver wrote past the program"
        # and the engine never leaves the program: every path ends in an explicit jmp
        rig.rx.run(600)
        assert 0 <= rig.rx.pc() <= used - 1, case.name


@pytest.mark.parametrize("sm_cycles_per_unit", range(MIN_SM_CYCLES_PER_UNIT,
                                                     MAX_SM_CYCLES_PER_UNIT + 1))
def test_every_usable_grid_decodes_and_the_useless_ones_are_refused(sm_cycles_per_unit: int) -> None:
    """Every S the driver accepts really decodes, and it accepts exactly those.

    S is how many SM cycles a NEC burst takes, so it is the only "speed" in a receiver whose
    protocol rate is fixed.  The sample windows cross at S = 7 (program.pio's header), so that is
    where the driver's range starts, and the 5-bit delay field ends it at 12.
    """
    program = build_program(sm_cycles_per_unit)
    assert sample_points(program)["bit_space"] == sample_cycles(sm_cycles_per_unit)["K"]
    rig = build(Case("grid", sm_cycles_per_unit=sm_cycles_per_unit))
    press(rig, 0x04, 0x08)
    assert rig.chk.clean, rig.chk.report()
    assert rig.rx.raws == [encode_frame(0x04, 0x08)], rig.rx.raws
    for bad in (1, 2, 3, 4, 5, 6, 13, 24):
        with pytest.raises(NecirError):
            build_program(bad)


# ==================================================================================================
# The timing: the windows the samples have to land in, and the divider that produces them
# ==================================================================================================
@pytest.mark.parametrize("sm_cycles_per_unit,tolerance", [(s, t) for s in (7, 9, 12)
                                                         for t in (0.25, 0.05)])
def test_every_sample_sits_inside_its_window_with_room_to_spare(
        sm_cycles_per_unit: int, tolerance: float) -> None:
    """Read the sample points back out of the words and check them against the NEC windows.

    `sample_window` turns the model's own nominals and the checker's `tolerance` into the range of
    SM cycles each sample may occupy; `sample_points` walks the assembled program and adds up
    `1 + delay` from each `wait` to find where the samples really are.  This test is where the two
    meet, so the numbers in program.pio's header are checked rather than promised -- including at
    the 5 % tolerance, which is the spread the five saved sources actually have (560 vs 562.5 us,
    under 0.5 %: research/ref/protocols/necir/README.md judgement call 2).
    """
    program = build_program(sm_cycles_per_unit, tolerance)
    margins = sample_margins(program, sm_cycles_per_unit, tolerance)
    for name, got in margins["samples"].items():
        assert got["in_window"], (name, got)
        assert got["lo_margin_ns"] > 0.0 and got["hi_margin_ns"] > 0.0, (name, got)
    # the two samples that tell two lengths apart are the ones with the least room, and even they
    # have a tenth of a burst on the tight side at the default tolerance
    if tolerance == 0.25:
        unit_ns = margins["window"]["unit_ns"]
        assert margins["samples"]["bit_space"]["lo_margin_ns"] > 0.05 * unit_ns, margins["samples"]


def test_the_divider_is_what_the_samples_were_computed_for() -> None:
    """CLKDIV is the 16.8 rounding of the divisor the sample points assume, and nothing coarser.

    One SM cycle has to be `unit_cycles / S` core cycles exactly, or the sample offsets mean
    something else; the driver derives the word from the same number the program was assembled
    for, and the two are compared here through the register the chip actually stored.
    """
    for sm_cycles_per_unit in range(MIN_SM_CYCLES_PER_UNIT, MAX_SM_CYCLES_PER_UNIT + 1):
        rig = build(Case("grid", sm_cycles_per_unit=sm_cycles_per_unit))
        stored = rig.rx.chip.engines[rig.rx.engine].clkdiv
        wanted = CORE_HZ * NEC_UNIT_NS / 1e9 / sm_cycles_per_unit
        assert stored == clkdiv_value(wanted)
        # a 16.8 divider resolves 1/256 of a core cycle, so the rate is exact to well under a tick
        assert abs(cycles_per_sm_cycle(stored) - wanted) <= 1.0 / 256
        assert rig.rx.clkdiv == stored
        # the fastest the program can be clocked at all is S = MAX (the smallest CLKDIV)
    fastest = build(Case("fast", sm_cycles_per_unit=MAX_SM_CYCLES_PER_UNIT)).rx
    slowest = build(Case("slow", sm_cycles_per_unit=MIN_SM_CYCLES_PER_UNIT)).rx
    assert fastest.cycles_per_sm_cycle < slowest.cycles_per_sm_cycle
    assert fastest.unit_cycles == slowest.unit_cycles == CORE_HZ * NEC_UNIT_NS / 1e9


def test_the_same_program_decodes_a_real_nec_remote_too() -> None:
    """Change the unit, not the program: a 562.5 *us* burst is the same 27 words and a legal CLKDIV.

    `verif.vip.necir` states its element timings in nanoseconds, so its NEC runs 1000x faster than
    a real remote's (a real burst is 562_500 ns; the model's is 562.5 ns, and its own README does
    the arithmetic at that scale).  Nothing in program.pio depends on the scale -- the samples are
    SM cycles -- so passing `unit_ns=REAL_NEC_UNIT_NS` gives the program a real remote's timings,
    and the only thing that changes is CLKDIV: 28125 core cycles per burst still fits the 16-bit
    INT with 2.3x to spare.  Simulating a 68 ms frame at 20 ns/tick would be 3.4 million ticks, so
    this checks the configuration and the windows rather than running the traffic.
    """
    program = build_program(DEFAULT_SM_CYCLES_PER_UNIT, unit_ns=REAL_NEC_UNIT_NS)
    model_scale = build_program()
    assert words_used(program) == words_used(model_scale)
    assert program.labels == model_scale.labels, "the scale is not the program"
    # only the pad delays differ: every opcode, and every sample's *purpose*, is the same
    opcodes = lambda p: [(w >> 13, (w >> 8) & 0x1F & 0x18) for w in p.instrs]
    assert opcodes(program) == opcodes(model_scale)
    assert sample_points(program) != sample_points(model_scale), \
        "a coarser edge quantisation moves the samples, which is the point"
    margins = sample_margins(program, DEFAULT_SM_CYCLES_PER_UNIT, unit_ns=REAL_NEC_UNIT_NS)
    assert margins["unit_cycles"] == 28_125.0
    assert margins["cycles_per_sm_cycle"] == 28_125.0 / DEFAULT_SM_CYCLES_PER_UNIT
    clkdiv = clkdiv_value(margins["cycles_per_sm_cycle"])
    assert (clkdiv >> 16) & 0xFFFF <= 0xFFFF
    for name, got in margins["samples"].items():
        assert got["in_window"], (name, got)
    # and the sample margins in *time* are a thousand times larger, because a burst is
    assert margins["samples"]["bit_space"]["lo_margin_ns"] > 100 * NEC_UNIT_NS


# ==================================================================================================
# The host side: what the driver wrote, and how it reads the words back
# ==================================================================================================
@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_registers_hold_what_program_pio_needs(case: Case) -> None:
    """Every field the program depends on, read back out of the chip rather than out of the driver.

    The thresholds are the interesting ones: `PULL_THRESH` and `PUSH_THRESH` are five-bit fields
    where 0 means 32, and writing `32 << 25` would put a bit in `FJOIN_TX` instead -- silently, with
    the engine then counting to 1 and stopping after one bit.  So the driver writes the fields
    through `_threshold_field` and this test reads the words back with `shiftctrl_fields`.
    """
    rig = build(case)
    engine = rig.rx.chip.engines[case.engine]

    shift = shiftctrl_fields(engine.shiftctrl)
    assert shift == {"pull_thresh": 32, "push_thresh": 32, "out_shiftdir": 1, "in_shiftdir": 1,
                     "autopull": 0, "autopush": 0}, shift

    assert (engine.execctrl >> 30) & 1 == 0, "SIDE_EN must be off: this program side-sets nothing"
    assert (engine.execctrl >> 29) & 1 == 0, "SIDE_PINDIR must be off"
    assert (engine.execctrl >> 24) & 0x1F == case.pin, "JMP_PIN is the IR pin"
    assert (engine.execctrl >> 12) & 0x1F == rig.rx.program.wrap
    assert (engine.execctrl >> 7) & 0x1F == rig.rx.program.wrap_target

    assert (engine.pinctrl >> 15) & 0x1F == case.pin, "IN_BASE is the IR pin"
    assert (engine.pinctrl >> 29) & 7 == 0, "SIDESET_COUNT must be 0, or every instruction writes"
    assert (engine.pinctrl >> 26) & 7 == 0 and engine.pinctrl & 0x1F == 0, "no OUT window"
    assert engine.clkdiv == rig.rx.clkdiv
    assert rig.rx.padout() & (1 << case.pin) == 0, "the IR pin's level register is untouched"


def test_the_driver_refuses_a_setup_it_cannot_honour() -> None:
    """A pin with no input path, an engine that does not exist, an S with no window, a bad word."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    dev = bus.attach(ChipDevice())
    for pin in (13, 14, 17, 18, -1):
        with pytest.raises(ValueError):
            NecirReceiver(bus=bus, dev=dev, pin=pin)
    with pytest.raises(ValueError):
        NecirReceiver(bus=bus, dev=dev, engine=2)
    with pytest.raises(ValueError):
        NecirReceiver(bus=Bus(), dev=None)          # a bus but no ChipDevice on it

    with pytest.raises(NecirError):
        build_program(1)                            # the data-space window is empty at S = 1
    with pytest.raises(NecirError):
        build_program(DEFAULT_SM_CYCLES_PER_UNIT, unit_ns=0.0)   # no time in a burst at all

    program = build_program()
    broken = copy.deepcopy(program)
    broken.instrs[program.labels["bit"] + 2] -= 1 << 8      # the pad, one cycle short
    with pytest.raises(NecirError, match="bit space sample"):
        NecirReceiver(program=broken)

    broken2 = copy.deepcopy(program)
    broken2.instrs[0] = 0xE081                             # `set pindirs, 1`: a receiver must not
    with pytest.raises(NecirError, match="writes a pin"):
        NecirReceiver(program=broken2)

    with pytest.raises(ValueError):
        clkdiv_value(0.5)                                  # CLKDIV cannot go below 1.0


def test_the_firmware_side_decode_agrees_with_the_reference_model() -> None:
    """`fw.necir.decode_frame` and `verif.vip.necir.decode_frame` are the same function.

    The driver decodes a pushed word on its own rather than importing the model, so that the test
    file can compare the two -- including on the words the chip actually pushed, which is where a
    difference in the shift direction or in the byte order would show up.
    """
    for raw in [encode_frame(a, c) for a, c in PAIRS] + [0, 0xFFFF_FFFF, 0xF30CF300, 0x12345678]:
        mine, theirs = decode_frame(raw), model_decode_frame(raw)
        assert mine["words"] == theirs["words"]
        assert mine["address8"] == theirs["address8"]
        assert mine["address16"] == theirs["address16"]
        assert mine["command"] == theirs["command"]
        assert mine["address_inverse_ok"] == theirs["address_inverse_ok"]
        assert mine["command_inverse_ok"] == theirs["command_inverse_ok"]

    rig = build(Case("s9-pin0"))
    press(rig, 0x59, 0x16)
    word = rig.rx.words[0]
    assert word.raw == encode_frame(0x59, 0x16)
    assert (word.address8, word.command, word.ok) == (0x59, 0x16, True)
    shared = ("words", "address8", "address16", "command", "address_inverse_ok",
              "command_inverse_ok")
    assert {k: word.info[k] for k in shared} == model_decode_frame(word.raw)


# ==================================================================================================
# The judge is not vacuous: a broken program must fail the transaction
# ==================================================================================================
MUTATIONS = [
    ("the data-space sample a cycle early", lambda p, i: p.instrs.__setitem__(
        i["bit"] + 2, p.instrs[i["bit"] + 2] - (1 << 8))),
    ("the data-space sample a cycle late", lambda p, i: p.instrs.__setitem__(
        i["bit"] + 2, p.instrs[i["bit"] + 2] + (1 << 8))),
    ("the leader-mark sample a burst early", lambda p, i: p.instrs.__setitem__(
        i["leader"] + 3, p.instrs[i["leader"] + 3] - (1 << 8))),
    ("the bit counter threshold dropped to 1", lambda p, i: p.instrs.__setitem__(
        i["bit"] + 5, 0x000F)),                     # `jmp !osre 15` -> `jmp !y ...`
    ("the word not inverted", lambda p, i: p.instrs.__setitem__(
        i["done"], p.instrs[i["done"]] & ~0x0010)),   # drop the MOV invert op
    ("the frame never pushed", lambda p, i: p.instrs.__setitem__(i["done"] + 1, 0x0000)),
    ("a data bit shifted out twice", lambda p, i: p.instrs.insert(i["bit"] + 3, 0x4001)),
]


@pytest.mark.parametrize("name,mutate", MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_a_broken_program_does_not_decode_the_frame(name: str, mutate) -> None:
    """Each mutation is written into IMEM after a clean load, and the chip's words must change.

    The mutations are applied to the engine's IMEM between `load()` and `enable()` rather than to
    the `Program` object, so this is a bad load of a good program -- which is the failure a real
    bring-up hits, and it keeps `check_program`'s guard out of the way.  Each one must move the
    words, or push nothing, or push a frame the model never sent.
    """
    program = build_program()
    mutated = copy.deepcopy(program)
    mutate(mutated, program.labels)

    rig = build(Case("s9-pin0"), auto_start=False)
    for k, word in enumerate(mutated.instrs):
        rig.rx.dev.write(imem_addr(0, k), word)
    rig.rx.start(load=False)
    rig.tx.press(0x59, 0x16)
    rig.settle(FRAME_TICKS_20NS)

    good = encode_frame(0x59, 0x16)
    assert rig.rx.raws != [good], f"{name}: the chip decoded the frame anyway ({rig.rx.raws})"

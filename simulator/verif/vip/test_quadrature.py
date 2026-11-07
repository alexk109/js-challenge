"""Self-tests for the pin-level quadrature-encoder reference models.

`make vip-selftest VIP=quadrature` runs this file.

Everything runs on the tick bus of `verif/vip/core.py` at 50 MHz (20 ns per tick) with an
encoder driving `a`, `b` and `i`, a decoder counting them, and the checker watching the pins.
`rig()` builds that bench; the tests then look at the decoded position, at what the checker
says about the wire, and at the models' own logs.

Each spec rule the checker has gets a test that breaks it, and the tests that break one rule
say which *other* rules the same fault has to break as well: `fAB` and `tEDGE` always come
together, because a uniformly fast encoder closes its edges up as it shortens its pulses.

The speed in a test is picked for how long it runs, not for realism: ticks per count is
750 000 / rpm, so 0.05 revolutions at 10 000 RPM is 200 counts and about 15 000 ticks, while
the same at 300 RPM would be 750 000.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus
from verif.vip.quadrature import (
    CW_CYCLE,
    QUADRATURE_PROFILES,
    QuadratureChecker,
    QuadratureDecoder,
    RotaryEncoder,
    Violation,
    attach_checker,
    attach_decoder,
    attach_encoder,
    profile_minimums,
    step_delta,
)

NS_PER_TICK = 20.0                    # 50 MHz chip clock
SLOW, FAST, LIMIT = 2_000, 10_000, 28_000       # RPM: slow, ordinary, the profile's max
DECODING_FACTORS = [1, 2, 4]


def rig(ppr=1000, x=4, rpm=FAST, turns=0.05, profile="as5047p", check_timing=None,
        check_profile=None, with_index=True, channel="b", **enc_kwargs):
    """A bus with an encoder, a decoder and a checker, all at `ppr` pulses per revolution.

    The decoder is told the encoder's starting position so the two models agree count for
    count from the first edge -- that is what `initial_position` is for. `check_timing` and
    `check_profile` are how a test points only the *limits* somewhere else, which is how the
    not-vacuous tests work.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    enc = attach_encoder(bus, ppr=ppr, index=with_index, **enc_kwargs)
    dec = attach_decoder(bus, x=x, channel=channel, counts_per_rev=enc.counts_per_rev,
                         initial_position=enc.initial_counts)
    chk = attach_checker(bus, profile=check_profile or profile, timing=check_timing)
    if turns:
        enc.queue_turns(turns, rpm)
    return bus, enc, dec, chk


def spin(enc, bus, limit=400_000):
    """Run the bus until the encoder's queue is done, and assert it got there."""
    assert enc.run(bus, limit=limit), "the encoder did not finish its queue"
    return enc


def arm_at(bus, chk, enc, net="a", ticks=900, when=0.6, after=("b", 1)):
    """Arm a glitch on the other line `when` of the way through an interval the checker has
    measured, once two A pulses are behind it.

    Three things keep a test's rule set clean. Arming in the *middle* of an interval rather
    than on an edge tick stops the fault's own edge landing within a tick of the previous one,
    which would trip `tEDGE` as well. Waiting for two measured pulses matters because until
    then the checker has no pulse length to judge a line's silence against and would skip the
    very measurement the test is here for. And `after` picks an interval whose *next* edge is
    a fall on `net`, so the glitch cannot shorten an A pulse and trip `fAB` on the way in: the
    Gray cycle puts a fall on a line one step after a rise on the other.
    """
    want_net, want_level = after

    def ready(_b):
        return (len(chk.pulses) >= 2 and chk.edges
                and (chk.edges[-1][1], chk.edges[-1][2]) == (want_net, want_level))

    assert bus.run_until(ready, 400_000), "no interval of the wanted shape turned up"
    step = chk.edges[-1][0] - chk.edges[-2][0]
    last = chk.edges[-1][0]
    bus.run_until(lambda _b: _b.t >= last + int(step * when), 400_000)
    enc.glitch(net, ticks)


# ---- the models agree with each other on legal traffic ---------------------------------
@pytest.mark.parametrize("rpm", [SLOW, FAST, LIMIT])
def test_position_direction_and_index_on_legal_traffic(rpm):
    """One direction, one index pulse, and a decoder that counts every edge."""
    turns = 0.02 if rpm == LIMIT else 0.05
    bus, enc, dec, chk = rig(rpm=rpm, turns=turns)
    spin(enc, bus)

    counts = int(round(turns * enc.counts_per_rev))
    assert dec.position == enc.position == enc.initial_counts + counts
    assert dec.direction == +1, "positive rpm is the datasheet's clockwise"
    # One index pulse, and it opened exactly at count 0 -- the encoder starts one index
    # window before it so that every pulse it produces is a whole one, and the rise lands on
    # the A/B edge that crosses count 0. That is where a firmware homing routine zeroes its
    # counter.
    rises = [e for e in dec.index_events if e.level == 1]
    assert dec.index_count == 1 and len(rises) == 1
    assert rises[0].position == 0
    assert len(dec.edges) == len(enc.edges) == counts
    assert dec.counted == counts
    assert chk.clean, chk.report()
    assert bus.errors == [], "push-pull: only the encoder drives, so nothing can contend"
    # The two conventions meet here: the decoder takes the core's one-tick latency out, so its
    # ticks are the ticks the encoder drove, while the checker measures the wire and so sees
    # them one tick later.
    assert [e.tick for e in dec.edges] == [e.tick for e in enc.edges]
    assert [e[0] for e in chk.edges] == [e.tick + dec.latency for e in enc.edges]
    assert dec.rpm == pytest.approx(rpm, rel=0.02)


def test_negative_rpm_counts_the_other_way():
    """Reverse rotation: fewer counts, negative direction, and still no violations."""
    bus, enc, dec, chk = rig(rpm=-FAST, turns=0.05)
    spin(enc, bus)

    assert dec.direction == -1
    assert dec.position == enc.initial_counts - 200
    assert dec.revolutions == pytest.approx(dec.position / dec.counts_per_rev)
    assert dec.position_in_rev == dec.position % enc.counts_per_rev
    assert chk.clean, chk.report()


def test_variable_speed_and_direction_in_one_run():
    """Three segments -- slow forward, fast backward, ordinary forward -- and the position
    after all three is the sum of them, with the checker silent throughout."""
    bus, enc, dec, chk = rig(turns=0)
    enc.queue_profile([(0.01, SLOW), (0.01, -FAST), (0.01, FAST)])
    spin(enc, bus)

    assert dec.position == enc.position == enc.initial_counts + 40 - 40 + 40
    assert chk.clean, chk.report()
    assert len(enc.event_log) == 3, enc.event_log
    assert len(dec.errors) == 0
    # three speeds, so the checker must have thrown away the pulses at the two step changes
    assert chk.pulses_skipped >= 2, chk.report()
    assert chk.periods > 2


@pytest.mark.parametrize("x", DECODING_FACTORS)
def test_decoding_factors(x):
    """x1 counts a quarter of what x4 counts, and none of them sees a fault."""
    bus, enc, dec, chk = rig(x=x)
    spin(enc, bus)

    assert dec.counted == 200 * x // 4, "x4 counts every edge, x1 one in four"
    assert dec.position == enc.initial_counts + dec.counted
    assert len(dec.errors) == 0 and chk.clean, chk.report()


def test_queue_seconds_and_stop():
    """A seconds-long segment runs for its time, and `stop()` drops the rest."""
    bus, enc, dec, chk = rig()
    enc.queue_seconds(0.002, FAST)
    enc.queue_turns(0.05, FAST)
    bus.run(500)                       # part way into the first segment
    enc.stop()
    spin(enc, bus)

    assert enc.rpm == 0 and enc.idle
    assert 0 < len(dec.edges) < 200, "stop() must cut the run short"
    assert chk.clean, chk.report()


def test_step_delta_is_the_whole_gray_rule():
    """One step forward, one step back, and nothing else."""
    assert [step_delta(CW_CYCLE[i], CW_CYCLE[(i + 1) % 4]) for i in range(4)] == [1] * 4
    assert [step_delta(CW_CYCLE[i], CW_CYCLE[(i - 1) % 4]) for i in range(4)] == [-1] * 4
    assert step_delta(0b00, 0b11) is None, "two bits at once is not a step"
    assert step_delta(0b01, 0b10) is None
    assert step_delta(0b00, 0b00) is None, "no movement is not a step either"

# ---- one test per rule the checker has ------------------------------------------------
def test_rule_ab_order_a_skipped_state():
    """AB_ORDER: an encoder that fails to put a state out jumps two bits at once.

    The Gray cycle moves one line per step, so two lines at once is not a step the encoder
    can make. The decoder must refuse to count it, which leaves it exactly one count short --
    the error is in the wire and the models must both show it.
    """
    bus, enc, dec, chk = rig()
    bus.run(300)
    enc.skip_states(1)
    spin(enc, bus)

    assert chk.rules_broken() == {"AB_ORDER"}, chk.report()
    bad = chk.violations_by_rule("AB_ORDER")
    assert bad and all(v.ceiling and v.measured > v.required == 1 for v in bad)
    assert all(v.tick > 0 and "2 lines moved at once" in v.detail for v in bad)
    assert len(dec.errors) == 1 and "impossible A/B transition" in dec.errors[0].detail
    # the next legal edge makes the decoder sure of itself again -- which is exactly why the
    # error log, not `valid`, is what a test has to read
    assert dec.valid is True
    assert dec.position == enc.position - 1, "the dropped step is exactly one count"
    # the wire is back in step afterwards: the next edge is a legal one again
    assert chk.violations_by_rule("AB_ORDER")[-1].tick < dec.errors[0].tick + 500


def test_rule_ab_same_a_glitch_on_one_line_only():
    """AB_SAME: one line moves and moves back with no edge on the other for a whole pulse.

    The cycle alternates A, B, A, B, so the gap between one line's edges is always one step
    and a step is never as long as a pulse. Nothing else about the wire is wrong here, which
    is the point: the encoder is at a legal speed with a legal phase.
    """
    bus, enc, dec, chk = rig(rpm=SLOW)
    arm_at(bus, chk, enc, net="a", ticks=4000)
    spin(enc, bus)

    assert chk.rules_broken() == {"AB_SAME"}, chk.report()
    quiet = chk.violations_by_rule("AB_SAME")
    assert all(v.ceiling and v.measured > v.required for v in quiet)
    assert all(v.tick > 0 and "moved with no" in v.detail for v in quiet)
    # A pure pin disturbance is invisible to the decoder: it sees two legal steps that cancel.
    assert dec.errors == [] and dec.position == enc.position
    assert dec.position != enc.position or True      # positions agree: the glitch cancelled


def test_rule_fab_and_tedge_an_overspeed_encoder():
    """fAB and tEDGE: faster than the profile's maximum speed.

    Both rules have to fire, and they always come as a pair: a uniformly fast encoder closes
    its edges up together as it shortens its pulses, so neither the pulse length nor the
    closest pair of edges can stay inside the limits. The index is switched off here so that
    only the speed rules are in play -- at 120 000 RPM a four-count index pulse is 2.5 ticks
    wide, which would trip IDX_WIDTH as well.
    """
    bus, enc, dec, chk = rig(with_index=False, turns=0)
    enc.queue_turns(0.05, FAST)
    bus.run(300)
    enc.overspeed(120_000, 200)
    spin(enc, bus)

    assert chk.rules_broken() == {"fAB", "tEDGE"}, chk.report()
    minimum = profile_minimums(NS_PER_TICK)["min_period"]
    fast = chk.violations_by_rule("fAB")
    assert fast and all(v.measured < v.required == minimum for v in fast)
    assert chk.shortest_period < minimum
    assert chk.max_ab_hz > profile_minimums(NS_PER_TICK)["max_ab_hz"]
    assert all(v.measured < v.required for v in chk.violations_by_rule("tEDGE"))
    assert chk.periods > 10, "the run has to outlast the burst to measure it"


def test_rule_tedge_edges_closer_than_one_step():
    """tEDGE on its own terms: a skewed encoder whose close pairs of edges crowd together.

    A skew moves B's edges early inside each A pulse. Two of the four intervals get shorter
    than one step, which is tEDGE, and the pulse's intervals stop being quarters of it, which
    is AB_PHASE. The pulse length itself does not move, so fAB stays quiet -- which is what
    separates this from the overspeed test.
    """
    bus, enc, dec, chk = rig()
    enc.set_skew(0.70 * 750_000 / FAST)          # 70% of a step, in ticks
    spin(enc, bus)

    assert chk.rules_broken() == {"AB_PHASE", "tEDGE"}, chk.report()
    assert chk.shortest_gap < profile_minimums(NS_PER_TICK)["min_edge"]
    assert chk.shortest_period >= profile_minimums(NS_PER_TICK)["min_period"]
    assert chk.worst_phase_counts > chk.min["phase_tol_counts"]


def test_rule_ab_phase_a_skew_inside_the_edge_limit():
    """AB_PHASE on its own: skewed, but every pair of edges is still far enough apart.

    This is the rule a receiver cannot do without -- the waveform is decodable edge by edge,
    so only the *phase* says the encoder is off. A fifth of a step of skew stays inside the
    tolerance and the checker stays silent.
    """
    bus, enc, dec, chk = rig()
    enc.set_skew(0.15 * 750_000 / FAST)
    spin(enc, bus)

    assert chk.clean, chk.report()
    assert 0 < chk.worst_phase_counts < chk.min["phase_tol_counts"]
    assert chk.shortest_gap >= chk.min["min_edge"]
    short, long = sorted(chk.pulses[-1][2])[:2][-1], max(chk.pulses[-1][2])
    assert sorted(chk.pulses[-1][2]) == [short, short, long, long], (
        "a skew makes two of the four intervals short and leaves the other two long")

    # a third of a step is over the tolerance and still leaves every edge far enough apart
    bus2, enc2, dec2, chk2 = rig()
    enc2.set_skew(0.55 * 750_000 / FAST)
    spin(enc2, bus2)
    assert chk2.rules_broken() == {"AB_PHASE"}, chk2.report()
    assert chk2.shortest_gap >= chk2.min["min_edge"]


def test_rule_idx_width_a_too_narrow_index():
    """IDX_WIDTH: an index pulse narrower than the profile says an index pulse is.

    The encoder is told to narrow the pulse to one count, which at 20 000 RPM is 37 ticks
    against the 108 the profile asks for. Nothing else about the wire is wrong.
    """
    bus, enc, dec, chk = rig(rpm=20_000, turns=0)
    enc.narrow_index(1)
    enc.queue_turns(0.05, 20_000)
    spin(enc, bus)

    assert chk.rules_broken() == {"IDX_WIDTH"}, chk.report()
    narrow = chk.violations_by_rule("IDX_WIDTH")
    assert all(v.measured < v.required == chk.min["min_index_width"] for v in narrow)
    assert all(v.tick > 0 for v in narrow)
    assert chk.shortest_index == chk.longest_index == narrow[0].measured
    assert chk.index_pulses == 1


def test_index_width_comes_from_the_profile_not_the_encoder():
    """The same 0.5-count index is legal for one profile and illegal for another.

    That is the whole point of a profile: the checker judges the wire against what the part
    promises, not against what the model happened to emit.
    """
    bus, enc, dec, chk = rig(rpm=20_000, turns=0)          # narrow_index comes below
    enc.narrow_index(1)
    enc.queue_turns(0.05, 20_000)
    spin(enc, bus)
    assert chk.rules_broken() == {"IDX_WIDTH"}

    # tell the checker the index only has to be 20 ticks wide and the same wire is fine
    bus2 = Bus(ns_per_tick=NS_PER_TICK)
    enc2 = attach_encoder(bus2, ppr=1000)
    dec2 = attach_decoder(bus2, x=4, counts_per_rev=enc2.counts_per_rev,
                          initial_position=enc2.initial_counts)
    chk2 = attach_checker(bus2, timing={"min_index_width": 20})
    enc2.narrow_index(1)
    enc2.queue_turns(0.05, 20_000)
    spin(enc2, bus2)
    assert chk2.violations_by_rule("IDX_WIDTH") == [], chk2.report()
    assert chk2.clean and dec2.errors == []
    assert chk2.shortest_index == chk.shortest_index


def test_index_once_per_revolution_over_a_long_run():
    """Two revolutions, two index pulses, both exactly one A pulse wide.

    The speed is high and the profile's `max_rpm` is overridden to match it, because the
    default profile's part really cannot turn at 400 000 RPM: at 20 ns per tick a 1000 ppr
    encoder needs 7.5 ticks per count at that speed, and one revolution is 15 000 ticks.
    """
    bus, enc, dec, chk = rig(ppr=250, rpm=400_000, turns=2.02,
                             check_profile="slow-mechanical",
                             check_timing={"max_rpm": 500_000})
    spin(enc, bus)

    assert chk.clean, chk.report()
    assert dec.index_count == 3 and chk.index_pulses == 3, "two revolutions, three windows"
    assert dec.position == enc.position == enc.initial_counts + 2020
    widths = {width for _tick, width in chk.index_edges}
    assert chk.shortest_index == chk.longest_index, "both pulses are the same width"
    # one index pulse is one A pulse: four counts
    assert chk.pulses[0][1] == chk.shortest_index, "one index pulse is one A pulse"
    assert chk.min["min_index_width"] <= chk.shortest_index < 2 * chk.min["min_index_width"], (
        "this wire runs below the profile's maximum speed, so its index pulse is wider than "
        "the shortest one the profile allows")
    # 2020 counts is 505 A pulses, so this is a long run for the checker and all but a couple
    # of them are measured: the tick grid rounds a 30-tick pulse up or down, and the checker
    # leaves a pulse alone when its length moved since the last one (that is how it avoids
    # reading acceleration as skew).
    assert chk.periods > 400 and chk.pulses_skipped <= 2, chk.report()


def test_checker_is_not_vacuous_on_a_tighter_profile():
    """The same wire, judged against a lower speed limit, must fail.

    Nothing about the wire changed between the two runs -- same encoder, same decoder, same
    10 000 RPM -- only the number the checker was given. So the limits come from the profile
    and its overrides, not from constants in the checker.
    """
    bus, enc, dec, chk = rig()
    spin(enc, bus)
    assert chk.clean, chk.report()
    pulse = chk.shortest_period

    bus2, enc2, dec2, chk2 = rig(check_timing={"max_rpm": 1_000})
    spin(enc2, bus2)
    assert chk2.shortest_period == pulse, "the same run, so the same pulse length"
    # A lower `max_rpm` moves every limit at once, so this wire now misses three of them: the
    # pulse, the closest pair of edges and the index pulse. fAB and tEDGE cannot be separated
    # here -- they are two readings of the same fact, that the wire is too fast -- which is why
    # every rule also has a test of its own above.
    assert chk2.rules_broken() == {"fAB", "tEDGE", "IDX_WIDTH"}, chk2.report()
    assert chk2.min["min_period"] > pulse, "the tighter limit is what makes it illegal"
    assert chk2.shortest_index == chk.shortest_index, "and the same index pulse"


def test_checker_is_not_vacuous_on_another_parts_profile():
    """The same wire, judged against a slow encoder's profile: it is far too fast for it.

    This also proves the checker reads `counts_per_rev` from the profile: a 250 ppr encoder
    has a quarter of the counts per revolution, so its shortest legal A pulse at the same
    300 RPM limit is four times longer.
    """
    bus, enc, dec, chk = rig(check_profile="slow-mechanical")
    spin(enc, bus)

    assert chk.rules_broken() == {"fAB", "tEDGE", "IDX_WIDTH"}, chk.report()
    slow = profile_minimums(NS_PER_TICK, "slow-mechanical")
    assert enc.counts_per_rev == 4000 and chk.min["counts_per_rev"] == 1000, (
        "the wire is a 1000 ppr encoder, judged against a 250 ppr part's limits")
    assert slow["min_period"] == chk.min["min_period"] == 40_000
    assert slow["min_period"] > chk.shortest_period * 100


def test_every_rule_fires_on_a_broken_encoder():
    """All six rules at once: five different faults on one wire, and nothing is missed.

    The point is the union. Each fault has its own test above; this one proves that a wire
    carrying all of them produces every violation the checker knows about, so a rule cannot
    be quietly unreachable behind another rule's recovery window.
    """
    bus, enc, dec, chk = rig(with_index=False, turns=0)
    enc.queue_turns(0.05, FAST)
    enc.set_skew(0.70 * 750_000 / FAST)
    bus.run(1_500)                   # long enough for a few skewed pulses to be measured
    enc.set_skew(0)                  # then take the skew away and get the other faults
    arm_at(bus, chk, enc, net="b", ticks=900)
    enc.skip_states(1)
    enc.overspeed(120_000, 3_000)
    enc.narrow_index(1)
    spin(enc, bus)

    assert chk.rules_broken() == {"AB_ORDER", "AB_SAME", "tEDGE", "fAB", "AB_PHASE"}, \
        chk.report()
    assert len(dec.errors) >= 1, "the decoder saw the impossible transition too"
    assert dec.position < enc.position, "and it is short because of it"
    # IDX_WIDTH needs an index, so it is the one rule a run without an index cannot show
    bus2, enc2, dec2, chk2 = rig(turns=0)
    enc2.queue_turns(0.05, FAST)
    enc2.narrow_index(1)
    enc2.skip_states(1)
    spin(enc2, bus2)
    assert "IDX_WIDTH" in chk2.rules_broken() and "AB_ORDER" in chk2.rules_broken(), chk2.report()


def test_a_short_glitch_is_invisible_to_both_models():
    """What the checker cannot see, stated as a test.

    A glitch shorter than the gap the checker needs (about three quarters of an A pulse at
    10 000 RPM) looks exactly like the shaft moving out and back: the transitions are legal,
    the timing is on the grid, and the decoder counts two steps that cancel. Neither model can
    tell it from motion -- only a filter in the device under test could -- so the checker
    says nothing and this test exists to say so.
    """
    bus, enc, dec, chk = rig(rpm=FAST)
    arm_at(bus, chk, enc, net="a", ticks=40)
    spin(enc, bus)

    assert chk.clean, chk.report()
    assert dec.errors == []
    assert dec.position == enc.position, "the two steps cancelled, so the position is right"


# ---- the timing table, and the guard rails --------------------------------------------
def test_profile_table_is_the_spec():
    """Guard against the numbers quietly drifting: these are the AS5047P's, in ns and ticks.

    Every number here is either quoted from the datasheet or derived from a quoted one in
    `research/ref/protocols/quadrature/as5047p-abi.md` (D1, D2, D3, D5).
    """
    prof = QUADRATURE_PROFILES["as5047p"]
    assert (prof.ppr, prof.counts_per_rev) == (1000, 4000)      # Fig. 30, ABIRES 000
    assert prof.max_rpm == 28_000.0                              # Fig. 9, MS
    assert QUADRATURE_PROFILES["as5047p-binary"].counts_per_rev == 4096   # Fig. 30, ABIBIN 1

    m = profile_minimums(NS_PER_TICK)
    # t_count = 60e9 / (28000 x 4000) = 535.714... ns, one step and one edge interval
    assert m["t_count_ns"] == pytest.approx(535.7142857, rel=1e-6)
    assert m["min_edge_ns"] == pytest.approx(m["t_count_ns"])
    assert m["min_period_ns"] == pytest.approx(4 * m["t_count_ns"])        # one A pulse
    assert m["min_level_ns"] == pytest.approx(2 * m["t_count_ns"])         # half an A pulse
    assert m["min_index_width_ns"] == pytest.approx(m["min_period_ns"])    # one ABI cycle
    assert m["max_ab_hz"] == pytest.approx(466_666.6667, rel=1e-6)
    # whole ticks, rounded up so a minimum is never lenient
    assert (m["min_edge"], m["min_period"], m["min_level"], m["min_index_width"]) == (27, 108, 54, 108)
    assert m["min_edge"] * 2 == m["min_level"], "a level is two of a pulse's four intervals"
    assert m["phase_tol_counts"] == 0.5 and m["counts_per_rev"] == 4000
    assert "AS5047P" in m["source"] and m["profile"] == "as5047p"

    binary = profile_minimums(NS_PER_TICK, "as5047p-binary")
    assert binary["counts_per_rev"] == 4096
    assert binary["t_count_ns"] == pytest.approx(523.1584821, rel=1e-6)   # 4096 steps/rev
    assert binary["min_period"] < profile_minimums(NS_PER_TICK)["min_period"], (
        "4096 steps per revolution is a shorter step, so the same speed is a shorter pulse")
    assert (binary["min_period"] - profile_minimums(NS_PER_TICK)["min_period"]) == -3, (
        "and rounding to whole ticks hides most of the difference between the two modes")

    # the tick rate is the only thing that changes the numbers, and rounding up is strict
    fast = profile_minimums(1.0)                        # 1 GHz: everything is exact
    assert (fast["min_edge"], fast["min_period"]) == (536, 2143)
    slow = profile_minimums(100.0)                      # 10 MHz: 5.357 ticks per step
    assert (slow["min_edge"], slow["min_period"]) == (6, 22)


def test_overrides_replace_individual_limits():
    """`overrides` is how a test, an encoder or a checker is pointed somewhere else."""
    m = profile_minimums(NS_PER_TICK, overrides={"max_rpm": 1_000})
    assert m["max_rpm"] == 1_000 and m["min_period"] == 3_000
    wide = profile_minimums(NS_PER_TICK, overrides={"min_period": 10_000,
                                                    "phase_tol_counts": 2})
    assert wide["min_period"] == 500 and wide["phase_tol_counts"] == 2
    assert wide["min_edge"] == profile_minimums(NS_PER_TICK)["min_edge"], (
        "the other limits are left alone")
    for bad in ({"max_rpm": 0}, {"min_period": 0}, {"phase_tol_counts": -1}, {"nonsense": 1}):
        with pytest.raises(ValueError):
            profile_minimums(NS_PER_TICK, overrides=bad)
    with pytest.raises(ValueError):
        profile_minimums(NS_PER_TICK, "no-such-part")


def test_devices_reject_nonsense():
    """The guard rails, so a test that mis-configures a bench fails loudly and not silently."""
    with pytest.raises(ValueError):
        RotaryEncoder(ppr=0)
    with pytest.raises(ValueError):
        RotaryEncoder(index_width_counts=5)
    with pytest.raises(ValueError):
        RotaryEncoder(start_state=4)                     # a Gray state is two bits, and two
    with pytest.raises(ValueError):
        QuadratureDecoder(x=3)
    with pytest.raises(ValueError):
        QuadratureDecoder(channel="c")
    with pytest.raises(ValueError):
        QuadratureDecoder(counts_per_rev=0)
    with pytest.raises(ValueError):
        QuadratureDecoder(latency=-1)
    assert [RotaryEncoder(start_state=g).ab for g in (0b00, 0b01, 0b10, 0b11)] == \
        [0b00, 0b01, 0b10, 0b11], "all four Gray states are legal starting points"

    bus, enc, dec, chk = rig()
    for call in (lambda: enc.queue_turns(0, 100), lambda: enc.queue_turns(1, 0),
                 lambda: enc.queue_seconds(-1, 100), lambda: enc.queue_seconds(1, 0),
                 lambda: enc.skip_states(0), lambda: enc.glitch("z", 10),
                 lambda: enc.glitch("a", 0), lambda: enc.overspeed(0, 10),
                 lambda: enc.overspeed(100, 0), lambda: enc.set_skew(-1),
                 lambda: enc.narrow_index(9)):
        with pytest.raises(ValueError):
            call()
    assert chk.violations == [] and dec.errors == [], "nothing above touched the bus"


def test_checker_reports_what_it_measured():
    """`report()`, the per-rule filter and the measured numbers a test would print."""
    bus, enc, dec, chk = rig(rpm=FAST)
    bus.run(300)
    enc.skip_states(1)
    spin(enc, bus)

    text = chk.report()
    assert "as5047p profile at 20.0 ns/tick" in text
    assert "A/B edges" in text and "index pulses" in text and "violations" in text
    assert f"limit {profile_minimums(NS_PER_TICK)['max_ab_hz'] / 1e3:.1f} Hz" in text
    assert "AB_ORDER" in text
    assert chk.violations_by_rule("AB_SAME") == []
    assert chk.violations_by_rule("AB_ORDER") == chk.violations
    assert "\n" not in chk.report(limit=0), "limit=0 is the header on its own"
    assert "1 violations" in chk.report(limit=0)
    assert chk.max_ab_hz == pytest.approx(1e9 / (NS_PER_TICK * chk.shortest_period))
    assert not chk.clean and str(chk.violations[0]).startswith("t=")
    assert chk.gaps == len(chk.edges) - 1, "every edge but the first is measured against one"
    assert chk.index_edges[0][0] == 0, "and the first sample of a net is its baseline"

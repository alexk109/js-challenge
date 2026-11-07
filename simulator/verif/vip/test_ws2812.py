"""Self-tests for the pin-level WS2812 reference models: `make vip-selftest VIP=ws2812`.

Everything runs at 50 MHz (20 ns per tick), which is where the datasheet's nominal
times land on whole ticks: a 0 bit is 20 ticks high and 43 low, a 1 bit is 40 high and
23 low -- both 63 ticks, so 793 kbit/s against the 800 kbit/s the datasheet's feature
list quotes -- and the reset code is 2500 ticks.

`rig()` builds the bench: a transmitter, a chain of pixels and a passive checker. The
tests then look at what the pixels latched, at what the checker decoded, and at what it
says about the wire. Every rule in the checker has a test that breaks it, one test per
rule, and one test proves the checker is not vacuous by judging legal traffic against
tighter limits than the datasheet's.
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus
from verif.vip.ws2812 import (
    BITS_PER_PIXEL,
    WS2812_PROFILES,
    Ws2812Tx,
    attach_chain,
    attach_checker,
    attach_transmitter,
    bits_to_word,
    classify_bit,
    grb_word,
    rgb_from_word,
    wait_latched,
    word_bits,
    ws2812_timings,
)

NS_PER_TICK = 20.0                    # 50 MHz chip clock

# A frame of four pixels, in the host's RRGGBB order, and the words the datasheet's GRB
# packing makes of them. Nothing else in the tests knows the wire order.
COLORS = [(255, 128, 0), (0, 255, 0), (10, 20, 30), (255, 255, 255)]
WORDS = [grb_word(c) for c in COLORS]


def rig(pixels=4, tx_timing=None, check_timing=None, pixel_timing=None, idle_level=0,
        idle_ticks=0, delay_bits=BITS_PER_PIXEL, check=True, ns_per_tick=NS_PER_TICK):
    """A bus with a transmitter, a chain of `pixels` and a passive checker.

    The three timing hooks are how a test drives one part of the bench off-spec without
    touching the others: `tx_timing` for the wire the transmitter makes, `check_timing`
    for the limits the monitor judges by, and `pixel_timing` for what the receiver
    believes. `idle_level` is what the transmitter drives between frames, and
    `delay_bits` is how many bits each pixel takes off the front of the frame.
    """
    bus = Bus(ns_per_tick=ns_per_tick)
    tx = attach_transmitter(bus, timing=tx_timing, idle_level=idle_level,
                            idle_ticks=idle_ticks)
    chain = attach_chain(bus, count=pixels, timing=pixel_timing, delay_bits=delay_bits)
    chk = attach_checker(bus, timing=check_timing) if check else None
    return bus, tx, chain, chk


def frame(bus, tx, chain, colors, limit=4_000_000):
    """Queue one frame and run until every pixel of the chain has latched it.

    Waiting for the latch and not for the transmitter is the point: the last pixel of a
    chain latches one tick per link after the wire does, and by the time the transmitter
    has finished its reset the first pixels have already latched, so a test that ran
    `Ws2812Tx.run()` first and then asked for "one more frame" would wait forever.
    """
    before = [p.frames for p in chain]
    tx.send(*colors)
    assert wait_latched(bus, chain, 1, limit=limit), "the chain did not latch the frame"
    assert [p.frames for p in chain] == [n + 1 for n in before], "one latch per pixel"
    return tx


def sent(bus, tx, chain, colors=COLORS, limit=4_000_000):
    """`frame()` for the usual case, returning the transmitter."""
    return frame(bus, tx, chain, colors, limit)


def driven(bus, tx, chain, colors=COLORS, ticks=200):
    """Queue one frame and run the transmitter to the end of its reset, then `ticks`.

    The broken-traffic tests need this rather than `frame()`: traffic that is off-spec
    on purpose is often traffic no receiver can latch at all, and what is under test
    there is the checker, not the pixels.
    """
    tx.send(*colors)
    assert tx.run(bus, limit=4_000_000), "the transmitter did not finish"
    bus.run(ticks)
    return tx


# ---- the models talk to each other ------------------------------------------------
def test_a_frame_reaches_every_pixel_as_its_own_word():
    """The whole point of a chain: word k lands on pixel k, first word on the first."""
    bus, tx, chain, chk = rig(pixels=4)
    sent(bus, tx, chain)

    assert [p.color for p in chain] == COLORS, [p.color for p in chain]
    assert [p.word for p in chain] == WORDS, "the datasheet's GRB words, one per pixel"
    assert all(p.frames == 1 and p.bad_bits == 0 for p in chain)
    assert chk.clean, chk.report()
    assert bus.errors == [], "push-pull: nothing may contend, and nothing did"

    # The checker decoded the same frame off the wire, in the datasheet's order.
    assert len(chk.frames) == 1 and chk.frames[0].bits == 96
    assert list(chk.frames[0].words) == WORDS
    assert chk.colors == COLORS
    assert [b.value for b in chk.bits] == [b for w in WORDS for b in word_bits(w)]
    # ... and it agrees with the pixels, without having been told what they should be.
    assert chk.colors == [p.color for p in chain]


def test_the_bit_count_shrinks_by_one_word_per_link():
    """A pixel takes its 24 bits and forwards the rest, so the far end sees less."""
    bus, tx, chain, chk = rig(pixels=4)
    sent(bus, tx, chain)

    assert len(chk.bits) == 4 * BITS_PER_PIXEL                  # what the wire carries
    for index, pixel in enumerate(chain):
        assert len(pixel.bits) == 4 * BITS_PER_PIXEL - index * BITS_PER_PIXEL
    # Each pixel latches one tick after the one in front of it: DOUT is the input one
    # tick later and nothing else, so the latches march down the chain in step.
    latches = [p.latch_log[-1].tick for p in chain]
    assert latches == [latches[0] + i for i in range(len(chain))]
    assert chk.frames[0].reset_tick == latches[0], "the checker latches when pixel 0 does"


def test_a_second_frame_relatches_the_chain():
    """A reset latches every frame, so the chain follows the transmitter frame by frame."""
    bus, tx, chain, chk = rig(pixels=3)
    frame(bus, tx, chain, COLORS[:3])
    second = [(0, 0, 255), (255, 0, 0), (1, 2, 3)]
    frame(bus, tx, chain, second)
    assert tx.run(bus, limit=4_000_000), "the transmitter had not finished"

    assert [p.color for p in chain] == second
    assert all(p.frames == 2 for p in chain)
    assert [ev.rgb for ev in chain[0].latch_log] == [COLORS[0], second[0]]
    assert len(chk.frames) == 2 and chk.colors == list(second)
    assert chk.clean, chk.report()
    # The frame log knows when each one was latched, and the second one is later.
    assert chk.frames[0].reset_tick < chk.frames[1].reset_tick


def test_a_one_pixel_chain_is_a_receiver_on_its_own():
    """The smallest chain: one pixel, nothing downstream of it."""
    bus, tx, chain, chk = rig(pixels=1)
    sent(bus, tx, chain, colors=[(1, 2, 3)])
    assert [p.color for p in chain] == [(1, 2, 3)]
    assert len(chain[0].bits) == BITS_PER_PIXEL
    assert chk.clean and chk.colors == [(1, 2, 3)]


def test_a_frame_shorter_than_the_chain_reaches_one_pixel():
    """One word per pixel, so a four-pixel chain given one word leaves three behind."""
    bus, tx, chain, chk = rig(pixels=4)
    frame(bus, tx, chain, COLORS)
    tx.send((9, 9, 9))
    assert wait_latched(bus, chain[:1], 1, limit=4_000_000), "pixel 0 did not latch"
    bus.run(6_000)          # let the rest of the frame, and its reset, go by

    assert [p.color for p in chain] == [(9, 9, 9), COLORS[1], COLORS[2], COLORS[3]]
    assert [p.frames for p in chain] == [2, 1, 1, 1], "one word reaches one pixel"
    assert len(chain[1].bits) == 72, "the last three links saw nothing of this frame"
    assert chk.clean, chk.report()
    assert chk.colors == [(9, 9, 9)]


def test_a_plain_tap_latches_the_same_word_everywhere():
    """What the per-pixel shift is worth: with `delay_bits=0` every pixel is a tap."""
    bus, tx, chain, chk = rig(pixels=3, delay_bits=0)
    sent(bus, tx, chain)
    assert [p.color for p in chain] == [COLORS[0]] * 3
    assert all(len(p.bits) == 96 for p in chain), "a tap forwards every bit"
    assert chk.clean, chk.report()


def test_grb_packing_and_bit_order_round_trip():
    """The datasheet's word is G in 23:16, R in 15:8, B in 7:0, sent MSB first."""
    assert grb_word((0x34, 0x12, 0x56)) == 0x123456, "RGB in, GRB out"
    assert grb_word(0x123456) == 0x341256, "and 0xRRGGBB is read as R, G, B"
    assert rgb_from_word(0x123456) == (0x34, 0x12, 0x56), "the inverse of grb_word"
    assert word_bits(0x123456) == [0, 0, 0, 1, 0, 0, 1, 0,     # G7..G0 = 0x12
                                   0, 0, 1, 1, 0, 1, 0, 0,     # R7..R0 = 0x34
                                   0, 1, 0, 1, 0, 1, 1, 0]     # B7..B0 = 0x56
    assert bits_to_word(word_bits(0x123456)) == 0x123456
    assert word_bits(0xFFFFFF) == [1] * BITS_PER_PIXEL
    assert word_bits(0) == [0] * BITS_PER_PIXEL
    assert rgb_from_word(grb_word((0, 0, 0))) == (0, 0, 0)
    assert grb_word((0x1FF, 0, 0)) == 0x00FF00, "a colour is masked to 8 bits per channel"


# ---- the checker passes legal traffic ---------------------------------------------
@pytest.mark.parametrize("pixels", [1, 2, 4, 8])
def test_the_checker_passes_a_legal_frame(pixels):
    """Any chain length, and the checker still says nothing about the wire."""
    bus, tx, chain, chk = rig(pixels=pixels)
    colors = [(i * 37 % 256, 255 - i * 29 % 256, i * 11 % 256) for i in range(pixels)]
    sent(bus, tx, chain, colors=colors)

    assert chk.violations == [], chk.report()
    assert [p.color for p in chain] == colors
    assert chk.colors == list(colors)
    assert chk.bits and chk.frames, "a checker that decoded nothing would be vacuous"
    assert len(chk.bits) == pixels * BITS_PER_PIXEL
    # Every bit is 63 ticks, so the wire runs at 793 kbit/s against the quoted 800 -- the
    # 20 ns tick cannot say 1.25 us, and the transmitter idles one tick before the first
    # frame so no measurement is stretched by the pull-up.
    assert set(chk.periods) == {63}, sorted(set(chk.periods))
    slow, fast = chk.bit_rate_hz
    assert slow == fast == pytest.approx(1e9 / (20.0 * 63))
    assert 790e3 < fast < 800e3
    assert len(chk.periods) == len(chk.bits) - 1, "the last bit's low time is the reset"
    for pixel in chain:
        assert pixel.bad_bits == 0 and pixel.frames == 1


def test_the_checker_agrees_with_the_pixels_about_what_was_on_the_wire():
    """The checker decodes the wire; the pixels decode the wire. Neither is told."""
    bus, tx, chain, chk = rig(pixels=3)
    sent(bus, tx, chain, colors=COLORS[:3])
    assert [b.value for b in chk.bits] == list(chain[0].bits), "same stream, same bits"
    assert list(chk.frames[0].words) == [p.word for p in chain], "one word per pixel"


# ---- one test per rule, each on traffic that breaks exactly that rule -------------
def rules_fired(chk):
    """The set of rule names on the checker's list."""
    return {v.rule for v in chk.violations}


def only_chk(chk, rule):
    """A view of `chk` holding only the violations of one rule, for `only()`."""
    view = copy.copy(chk)
    view.violations = chk.violations_by_rule(rule)
    return view


def only(chk, rule):
    """Every violation a broken-traffic test is allowed to produce."""
    assert rules_fired(chk) == {rule}, chk.report()
    assert chk.violations, chk.report()
    for v in chk.violations:
        assert v.rule == rule and v.tick > 0
        assert {"min": v.measured < v.required,
                "max": v.measured > v.required,
                "exact": v.measured != v.required}[v.sense], v
        assert str(v).startswith(f"t={v.tick} {rule} "), str(v)


def test_the_checker_flags_a_zero_bit_that_is_too_short_high():
    """T0H's floor: a 0 bit whose high is 200 ns is not a 0 any more."""
    bus, tx, chain, chk = rig(tx_timing={"t0h": 200})
    driven(bus, tx, chain)

    assert tx.errors, "the transmitter knows it is off-spec and says so"
    assert any("T0H" in e for e in tx.errors)
    only(chk, "T0H")
    low = ws2812_timings(NS_PER_TICK)["t0h_min"]
    assert {v.required for v in chk.violations} == {low}
    # 200 ns is 10 ticks at 50 MHz, 11 for the first bit of the frame: the bus shows the
    # pull-up before anything drives the net, which is the tick the checker documents.
    assert {v.measured for v in chk.violations} <= {10, 11}
    # The period is still inside TH+TL, and the low time is fine: only T0H is wrong.
    assert 33 <= 10 + tx.low[0] <= 92 and tx.low[0] <= chk.min["t0l_max"]
    # A receiver throws a high under the floor away, so the 0 bits never arrive: the
    # pixels latch whatever is left, which is not what was sent.
    assert chain[0].glitches > 0 and chain[0].bad_bits == 0
    assert [p.color for p in chain] != COLORS


def test_the_checker_flags_a_one_bit_that_is_too_long_high():
    """T1H's ceiling: a 1 bit held high for 1 us is past what the datasheet allows."""
    bus, tx, chain, chk = rig(tx_timing={"t1h": 1_000})
    driven(bus, tx, chain)

    only(chk, "T1H")
    limits = ws2812_timings(NS_PER_TICK)
    assert {v.required for v in chk.violations} == {limits["t1h_max"]}
    assert {v.measured for v in chk.violations} <= {50, 51}, "1000 ns is 50 ticks"
    # Still inside the period window, and the receiver reads it as a 1 like everyone else.
    assert 33 <= 50 + tx.low[1] <= 92
    assert chk.colors == COLORS, "the data is right; the timing is not"


def test_the_checker_flags_a_zero_bit_that_is_too_long_low():
    """T0L's ceiling: the low time after a 0 bit is the one that gives the bit its rate."""
    bus, tx, chain, chk = rig(tx_timing={"t0l": 1_400})
    sent(bus, tx, chain)

    only(chk, "T0L")
    assert {v.required for v in chk.violations} == {ws2812_timings(NS_PER_TICK)["t0l_max"]}
    assert {v.measured for v in chk.violations} == {70}
    # The period is still legal (90 ticks is inside 33..92), so only the low time is wrong.
    assert chk.periods and max(chk.periods) <= 92


def test_the_checker_flags_a_one_bit_that_is_too_long_low():
    """T1L's ceiling, which is tighter than T0L's: a 1 bit's low is only 0.45 us."""
    bus, tx, chain, chk = rig(tx_timing={"t1l": 900})
    sent(bus, tx, chain)

    only(chk, "T1L")
    assert {v.required for v in chk.violations} == {ws2812_timings(NS_PER_TICK)["t1l_max"]}
    assert {v.measured for v in chk.violations} == {45}
    # The wire still decodes, because a long low does not stop being a low.
    assert chk.colors == COLORS, "the receiver is happy; the checker is not"
    assert [p.color for p in chain] == COLORS


def test_the_checker_flags_a_high_that_decodes_as_neither_bit():
    """DECODE: 620 ns is past a 0's ceiling and short of a 1's floor."""
    limits = ws2812_timings(NS_PER_TICK)
    assert limits["t0h_max"] == 27 and limits["t1h_min"] == 33, "the dead band is 28..32"

    bus, tx, chain, chk = rig(tx_timing={"t0h": 620})
    driven(bus, tx, chain)

    # DECODE on every one of them, and FRAME as well, because the bits that did arrive are
    # not a whole number of words -- that second report is true, not a cascade. RESET is
    # not among them: a bit nobody can read is this fault, not a missing reset code.
    assert rules_fired(chk) == {"DECODE", "FRAME"}, chk.report()
    only(only_chk(chk, "DECODE"), "DECODE")
    assert {v.required for v in chk.violations_by_rule("DECODE")} == {limits["t0h_max"]}
    assert {v.measured for v in chk.violations_by_rule("DECODE")} == {31}
    assert classify_bit(31, limits) is None, "and no receiver can call that a bit"
    assert [v.measured for v in chk.violations_by_rule("FRAME")] == [49]
    assert not chk.violations_by_rule("RESET"), "one fault, one report"

    # The pixels see the same dead band and count it instead of guessing. What is left
    # latches something, and it is nothing like what was sent: a strip cannot do better,
    # it has no bit to put in the gap.
    assert chain[0].bad_bits > 0, "no pixel can guess a bit in the dead band"
    assert [p.color for p in chain] != COLORS
    assert [p.frames for p in chain] == [1, 1, 0, 0], "and the frame ran out of words"
    assert any("DECODE" in e for e in tx.errors)


def test_the_checker_flags_a_bit_that_is_too_short():
    """TBIT's floor, and the only rule that catches a transmitter that is just fast:
    a 0 bit with a 0.4 us high and a 0.2 us low meets every floor and ceiling above."""
    bus, tx, chain, chk = rig(tx_timing={"t0l": 200})
    driven(bus, tx, chain)

    only(chk, "TBIT")
    limits = ws2812_timings(NS_PER_TICK)
    assert {v.required for v in chk.violations} == {limits["t_bit_min"]}
    assert {v.measured for v in chk.violations} <= {30, 31}, "20 + 10 ticks is 600 ns"
    assert limits["t_bit_min"] == 33, "650 ns at 20 ns a tick"
    assert 20 >= limits["t0h_min"] and 10 <= limits["t0l_max"], "nothing else is wrong"


def test_the_checker_flags_a_bit_that_is_too_long():
    """TBIT's ceiling, read off a legal wire by a checker told a tighter TH+TL."""
    bus, tx, chain, chk = rig(check_timing={"t_bit": 900, "t_bit_tol": 300})
    sent(bus, tx, chain)
    assert tx.run(bus, limit=1_000_000), "the transmitter had not finished"

    assert tx.errors == [] and tx.frames[0].ok, "the transmitter is on-spec"
    only(chk, "TBIT")
    assert {v.sense for v in chk.violations} == {"max"}
    assert {v.required for v in chk.violations} == {60}, "1200 ns is 60 ticks"
    assert chk.min["t_bit_max"] == 60 and chk.min["t_bit_min"] == 30, "900 +/- 300 ns"
    assert {v.measured for v in chk.violations} <= {63, 64}, "the wire runs at 1260 ns"
    # The same wire is clean against the datasheet's own caption, which allows 1.85 us.
    bus2, tx2, chain2, chk2 = rig()
    sent(bus2, tx2, chain2)
    assert chk2.clean, chk2.report()


def test_an_idle_high_transmitter_that_misses_the_reset_time_is_caught():
    """The same fault seen from two sides: the wire says the low was too long, and the
    frame that never latched says the reset never came.

    A transmitter that releases its pin between frames (`idle_level=1`, which is what a
    PIO program with a pull-up on the data pin does) and whose reset is too short puts the
    last bit's low and the gap together, so the wire shows one over-long low time and one
    over-long bit period. That is a real report of a real fault -- but on its own it does
    not say "no reset", because a low that is too long and a low that is a reset look the
    same from the wrong end. `finalize()` is what says the frame was never latched.
    """
    assert ws2812_timings(NS_PER_TICK)["t_reset_min"] == 2_500, "50 us at 20 ns a tick"
    bus, tx, chain, chk = rig(tx_timing={"t_reset": 1_000}, idle_level=1, idle_ticks=40)
    driven(bus, tx, chain)

    assert rules_fired(chk) == {"T1L", "TBIT"}, chk.report()
    assert chk.violations[0].measured == 73, "a 1 bit's low of 23 plus a 50-tick gap"
    assert all(p.frames == 0 for p in chain), "no strip latched a frame without a reset"
    left = chk.finalize(0)
    assert [v.rule for v in left] == ["RESET"], chk.report()
    assert left[0].required == 2_500 and left[0].measured == 0

    # The same transmitter with the datasheet's own reset is legal, idle high or not,
    # with or without a gap between frames.
    for idle_level, idle_ticks in ((0, 0), (1, 0), (0, 40), (1, 40)):
        bus, tx, chain, chk = rig(idle_level=idle_level, idle_ticks=idle_ticks)
        sent(bus, tx, chain)
        assert chk.clean, f"idle {idle_level}/{idle_ticks}: " + chk.report()
        assert [p.color for p in chain] == COLORS
        assert chk.finalize(0) == [], chk.report()


def test_finalize_reports_the_reset_the_wire_cannot_show():
    """RESET, on a stream that simply stops: the line stays low, so nothing on the wire
    says the frame was never latched, and `finalize()` has to say it."""
    bus, tx, chain, chk = rig(tx_timing={"t_reset": 1_000})
    tx.send(*COLORS)
    tx.run(bus, limit=4_000_000)
    bus.run(200)

    assert chk.violations == [], "a low-idle transmitter gives the wire nothing to see"
    left = chk.finalize()
    assert [v.rule for v in left] == ["RESET"], chk.report()
    assert left[0].measured == 0 and "no reset code" in left[0].detail
    assert "95 bits" in left[0].detail, "the last bit went with the reset, not before it"
    assert chk.violations == left, "finalize() reports into the same list"
    assert chk.finalize() == [], "and only once: the stream is closed now"

    # A stream that was properly reset has nothing left to report.
    bus, tx, chain, chk = rig()
    sent(bus, tx, chain)
    assert chk.finalize() == [], chk.report()
    assert chk.clean


def test_the_checker_flags_a_frame_that_is_not_a_whole_number_of_words():
    """FRAME: a reset after 5 bits. The word is incomplete and there is no rule for it."""
    bus, tx, chain, chk = rig()
    tx.send_bits([1, 0, 1, 1, 0])          # five bits, then the reset
    assert tx.run(bus, limit=4_000_000)
    bus.run(200)

    only(chk, "FRAME")
    assert chk.violations[0].measured == 5 and chk.violations[0].required == 24
    assert chk.violations[0].sense == "exact", "25 bits is as wrong as 23"
    assert not tx.frames[0].ok and any("FRAME" in e for e in tx.frames[0].errors)
    # Only pixel 0 was sent anything: it swallows 24 bits and only 5 arrived, so the
    # links behind it are idle and the pixels behind it see no frame at all.
    assert [p.short_resets for p in chain] == [1, 0, 0, 0]
    assert [p.frames for p in chain] == [0, 0, 0, 0]
    assert [p.color for p in chain] == [(0, 0, 0)] * 4, "a strip stays black"

    # 23 bits and 25 bits are both wrong, and both are caught.
    for count in (23, 25, 47):
        bus, tx, chain, chk = rig()
        tx.send_bits([1] * count)
        tx.run(bus, limit=4_000_000)
        bus.run(200)
        assert [v.rule for v in chk.violations] == ["FRAME"], (count, chk.report())
        assert chk.violations[0].measured == count


# ---- the checker is not vacuous ---------------------------------------------------
def test_the_checker_is_not_vacuous():
    """The same legal wire, judged against limits other than the datasheet's, must fail.

    That is the whole claim: the checker's limits are data it was given, not constants
    inside it, so a firmware author can hand it the limits their strip really has. The
    rule used here is T0L's ceiling -- the low time after a 0 bit, which is 43 ticks on
    this wire. Told T0L is 700 ns and to judge nominals only, it is 35 ticks and every
    0 bit is too long. The high-time rules are left alone on purpose: they are what
    decides which bit a high time is, so a checker that judged them differently would be
    a different decoder, not a stricter one.
    """
    bus, tx, chain, chk = rig()
    sent(bus, tx, chain)
    assert chk.clean and len(chk.bits) == 96 and len(chk.frames) == 1
    assert len(chk.violations_by_rule("T0L")) == 0

    bus2, tx2, chain2, chk2 = rig(check_timing={"t0l": 600})
    sent(bus2, tx2, chain2)
    only(chk2, "T0L")
    assert {v.required for v in chk2.violations} == {37}, "600 + 150 ns is 37 ticks"
    assert {v.measured for v in chk2.violations} == {43}, "and the wire sends 43"
    assert len(chk2.violations) == 47, "one per 0 bit in the frame"
    assert [p.color for p in chain2] == COLORS, "the pixels are not told about the limit"
    assert chk2.colors == COLORS, "and the data on the wire did not change"

    # The other direction: the same wire, both ends told to use the datasheet's nominals
    # and nothing else. The transmitter then drives 20/40 high and 42/22 low, which is
    # inside windows of zero width at 20 ns a tick -- except that it has to be told, since
    # 0.85 us and 0.45 us do not land on whole ticks.
    bus3, tx3, chain3, chk3 = rig(tx_timing={"tol": 0}, check_timing={"tol": 0})
    sent(bus3, tx3, chain3)
    assert (tx3.high, tx3.low) == ({0: 20, 1: 40}, {0: 42, 1: 22}), (tx3.high, tx3.low)
    assert chk3.clean, chk3.report()
    assert chk3.min["t0h_min"] == chk3.min["t0h_max"] == 20
    assert chk3.min["t1h_min"] == chk3.min["t1h_max"] == 40
    assert [p.color for p in chain3] == COLORS
    # A transmitter left at the datasheet's own profile would miss that by a tick on both
    # low times -- 0.85 and 0.45 us are not whole ticks at 20 ns -- which is the
    # transmitter's business and the checker's to report, and it reports both.
    bus4, tx4, chain4, chk4 = rig(check_timing={"tol": 0})
    driven(bus4, tx4, chain4)
    assert rules_fired(chk4) == {"T0L", "T1L"}, chk4.report()
    assert {v.required for v in chk4.violations} == {42, 22}


def test_a_checker_on_a_later_link_sees_a_shorter_frame():
    """The checker is a pin-level monitor, so it can watch any tap of the chain."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_transmitter(bus)
    chain = attach_chain(bus, count=4)
    first = attach_checker(bus, net="din")
    third = attach_checker(bus, net="dout2")
    sent(bus, tx, chain)

    assert first.clean and third.clean, (first.report(), third.report())
    assert first.colors == COLORS
    assert [f.bits for f in first.frames] == [96], "the wire carries all four words"
    # dout2 is what the third pixel forwards, so it carries what is left after the third
    # pixel took its own word: the fourth pixel's, and only that.
    assert [f.bits for f in third.frames] == [24], "one word on that link"
    assert third.colors == [COLORS[3]], "the fourth pixel's word"
    assert third.frames[0].reset_tick > first.frames[0].reset_tick


# ---- the timing table is the spec -------------------------------------------------
def test_the_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting. These are the Worldsemi WS2812B
    datasheet of 9-Apr-2013, page 3's "Data transfer time" table, as saved in
    research/ref/protocols/ws2812/ws2812b-2013-text.txt and read in TIMINGS.md."""
    m = ws2812_timings(NS_PER_TICK)
    assert (m["t0h_ns"], m["t1h_ns"]) == (400.0, 800.0), "T0H and T1H, +/-150 ns"
    assert (m["t0l_ns"], m["t1l_ns"]) == (850.0, 450.0), "T0L and T1L, +/-150 ns"
    assert m["tol_ns"] == 150.0 and m["t_bit_ns"] == 1_250.0, "TH+TL = 1.25 us"
    assert m["t_bit_tol_ns"] == 600.0 and m["t_reset_ns"] == 50_000.0, "+/-600 ns, 50 us"
    assert m["bit_rate"] == 800e3, "page 1: 'Send data at speeds of 800Kbps'"
    assert m["profile"] == "ws2812b-2013" and WS2812_PROFILES["ws2812b-2013"].t_bit == 1250

    # Floors round up and ceilings round down, so tick quantisation is never lenient.
    assert (m["t0h_min"], m["t0h_max"]) == (13, 27), "250 ns .. 550 ns at 20 ns a tick"
    assert (m["t1h_min"], m["t1h_max"]) == (33, 47), "650 ns .. 950 ns"
    assert (m["t0l_max"], m["t1l_max"]) == (50, 30), "ceilings: 1000 ns, 600 ns"
    assert (m["t_reset_min"], m["t_bit"]) == (2_500, 63), "50 us, and 1.25 us"
    assert (m["t_bit_min"], m["t_bit_max"]) == (33, 92), "650 ns .. 1850 ns"

    # The transmitter sits inside every window, and the dead band is where it should be.
    tx = Ws2812Tx()
    assert tx.high == {0: 20, 1: 40} and tx.low == {0: 43, 1: 23}
    assert tx.reset == 2_500 and tx.errors == []
    for bit in (0, 1):
        assert m[f"t{bit}h_min"] <= tx.high[bit] <= m[f"t{bit}h_max"]
        assert tx.low[bit] <= m[f"t{bit}l_max"]
        assert m["t_bit_min"] <= tx.high[bit] + tx.low[bit] <= m["t_bit_max"]
    assert classify_bit(12, m) == 0 and classify_bit(13, m) == 0
    assert classify_bit(27, m) == 0 and classify_bit(28, m) is None
    assert classify_bit(32, m) is None and classify_bit(33, m) == 1
    assert classify_bit(47, m) == 1 and classify_bit(48, m) == 1

    with pytest.raises(ValueError):
        ws2812_timings(NS_PER_TICK, "ws2812b-2027")
    with pytest.raises(ValueError):
        ws2812_timings(NS_PER_TICK, overrides={"t_nonsense": 1})



# ---- guard rails -----------------------------------------------------------------
def test_the_models_reject_nonsense():
    """Everything that can be got wrong in the constructors is refused, loudly."""
    with pytest.raises(ValueError):
        ws2812_timings(NS_PER_TICK, "ws2812-ultra")
    with pytest.raises(ValueError):
        ws2812_timings(NS_PER_TICK, overrides={"t0H": 400})   # the keys are lower case
    with pytest.raises(ValueError):
        Ws2812Tx(idle_level=2)
    with pytest.raises(ValueError):
        Ws2812Tx(idle_ticks=-1)
    with pytest.raises(ValueError):
        attach_chain(Bus(), count=0)
    with pytest.raises(ValueError):
        attach_chain(Bus(), count=1, delay_bits=-1)

    # A tick rate too coarse to express a legal T0H is refused rather than rounded into
    # something no receiver can read: the 300 ns window is narrower than a 600 ns tick.
    with pytest.raises(ValueError, match="cannot express a legal t0h"):
        Ws2812Tx(ns_per_tick=600.0)
    # 50 ns a tick is fine, and the transmitter still lands inside the windows.
    tx = Ws2812Tx(ns_per_tick=50.0)
    assert tx.high[0] == 8 and tx.low[0] == 17 and tx.reset == 1_000
    assert tx.errors == []

    # A device may not drive a net the bus does not have, and the checker never does.
    bus = Bus()
    tx = attach_transmitter(bus)
    chk = attach_checker(bus)
    bus.run(10)
    assert bus.errors == [], "a checker drives nothing, so nothing contends"
    assert tx.pending_bits == 0


def test_two_transmitters_on_one_wire_are_contention():
    """The WS2812 wire is push-pull, so two drivers disagreeing is a bus error."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    one = attach_transmitter(bus, name="ws2812_a")
    two = attach_transmitter(bus, name="ws2812_b")
    chk = attach_checker(bus)
    one.send((0xFF, 0, 0))
    two.send((0x00, 0xFF, 0))
    bus.run(2_000)

    assert bus.errors, "the two transmitters drove opposite levels"
    assert {e.net for e in bus.errors} == {"din"}
    assert {tuple(sorted(e.drivers)) for e in bus.errors} == {("ws2812_a", "ws2812_b")}
    assert chk is not None and chk.report()

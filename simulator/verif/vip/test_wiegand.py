"""Self-tests for the pin-level Wiegand reference models: `make vip-selftest VIP=wiegand`.

Every model in `verif/vip/wiegand.py` is exercised against the tick bus of `verif/vip/core.py`
with a `WiegandReader` on the wire, a `WiegandReceiver` decoding it and a passive
`WiegandChecker` measuring it. `bench()` builds that rig; the tests then look at the
receiver's decoded cards, at what the checker measured, and at which rules it flagged.

The bulk of the suite runs at 1 MHz (`NS_PER_TICK = 1000.0`, one tick = 1 us) because
Wiegand's own timebase is microseconds to milliseconds: at the chip's default 50 MHz one
26-bit frame is about 1.6 M ticks, which is a lot of Python for a rule that is stated in
nanoseconds. Nothing about the models depends on the tick rate -- every limit is derived
from it -- and `test_the_chip_clock_rate_quantises_the_same_limits` runs a frame at the
default 20 ns/tick to prove that, plus one test on each side of a quantisation boundary.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Device
from verif.vip.wiegand import (
    DEFAULT_FORMAT,
    WIEGAND_FORMATS,
    attach_checker,
    attach_reader,
    attach_receiver,
    decode_card,
    encode_card,
    ns_to_ticks,
    parity_bit,
    spec_minimums,
    wiegand_bus,
    wiegand_format,
)

NS_PER_TICK = 1000.0            # 1 MHz: one tick is 1 us, a Wiegand tick
CHIP_NS_PER_TICK = 20.0         # the chip's own clock, 50 MHz
CARD = (123, 4567)              # the credential most tests present
LEGAL_PW = 50_000               # 50 us: BALTECH's PulseWidth default
LEGAL_PI = 2_000_000            # 2 ms: BALTECH's PulseInterval default


class ShortedPair(Device):
    """Drags both lines to ground for a while, the way a shorted cable pair does.

    The reader model cannot do this: its pulses are strictly sequential, so the one thing
    the specification forbids outright -- D0 and D1 low at the same time -- has to come
    from the installation. Both lines low also merges two pulses into one long one, so a
    short shows up as D0D1 plus whatever the merged pulse does to the timing rules.
    """

    name = "short"

    def __init__(self, start: int, ticks: int):
        self.start, self.ticks = start, ticks

    def tick(self, t: int, pins) -> dict:
        return {"d0": 0, "d1": 0} if self.start <= t < self.start + self.ticks else {}


class SecondReader(Device):
    """A second reader wired to the same pair, pulsing the other line in the same window.

    Two readers in parallel is a real installation mistake and the other way to put D0 and
    D1 low together, this time with both pulses the right width.
    """

    name = "second_reader"

    def __init__(self, start: int, width: int, line: str = "d1"):
        self.start, self.width, self.line = start, width, line

    def tick(self, t: int, pins) -> dict:
        if self.start <= t < self.start + self.width:
            return {self.line: 0}
        return {}


def bench(ns_per_tick=NS_PER_TICK, facility=CARD[0], number=CARD[1], bit_format=None,
          reader=None, check=None, receiver=None, check_timing=None, extra=None):
    """A reader, a receiver and a checker on one pair of open-drain lines.

    The three keyword groups are how one part of the bench is driven off-spec without
    touching the others: `reader=` for the reader's construction, `check=`/`check_timing=`
    for the limits the monitor judges by, `receiver=` for the decoder's layout and
    timeout. `extra=` attaches a further device, for the faults a reader cannot make.
    """
    bus = wiegand_bus(ns_per_tick=ns_per_tick)
    reader_kw = {"facility": facility, "number": number}
    reader_kw.update(reader or {})
    if facility is None and number is None:
        reader_kw.pop("facility"), reader_kw.pop("number")
    rdr = attach_reader(bus, bit_format=bit_format or DEFAULT_FORMAT, **reader_kw)
    rcv = attach_receiver(bus, bit_format=bit_format or DEFAULT_FORMAT, **(receiver or {}))
    chk = attach_checker(bus, bit_format=bit_format or DEFAULT_FORMAT, timing=check_timing,
                         **(check or {}))
    if extra is not None:
        bus.attach(extra)
    return bus, rdr, rcv, chk


def swipe(bus, rdr, limit=2_000_000):
    """Run the queue out and the trailing frame gap with it."""
    assert rdr.run(bus, limit=limit), rdr.report()
    return rdr


def rules(chk) -> set:
    """The set of rule names the checker flagged."""
    return {v.rule for v in chk.violations}

# ---- the models talking to each other ---------------------------------------------
@pytest.mark.parametrize("bit_format", sorted(WIEGAND_FORMATS))
def test_the_reader_and_the_receiver_talk(bit_format):
    """One card in, the same facility code and ID number out, and a clean wire.

    Run for every layout, because the layouts differ in field order and in which half is
    even parity (research/ref/protocols/wiegand/README.md); the model must carry a card
    across each of them without either end caring which one is in use.
    """
    fmt = wiegand_format(bit_format)
    # ind26 splits the 24 bits 12 + 12, so the same card does not fit every layout; take
    # the biggest one that does rather than quietly testing a different credential.
    facility = min(CARD[0], (1 << fmt.facility[1]) - 1)
    number = min(CARD[1], (1 << fmt.number[1]) - 1)
    bus, rdr, rcv, chk = bench(bit_format=bit_format, facility=facility, number=number)
    swipe(bus, rdr)

    assert chk.clean, chk.report()
    assert rcv.clean, rcv.report()
    assert len(rcv.frames) == 1 and len(rcv.bits) == 26
    card = rcv.frames[0].card
    assert (card.facility, card.number) == (facility, number)
    assert card.parity_ok, str(card)
    # the checker decoded the same frame with the same answer
    assert (chk.frames[0].facility, chk.frames[0].number) == (facility, number)


@pytest.mark.parametrize("facility,number",
                         [(0, 0), (0, 65535), (255, 0), (255, 65535), (1, 1), (128, 32768)])
def test_every_corner_of_the_24_data_bits_survives_the_wire(facility, number):
    """The extremes of both fields, because a parity scheme that only works on the middle
    of its range is not a parity scheme."""
    bus, rdr, rcv, chk = bench(facility=facility, number=number)
    swipe(bus, rdr)

    assert chk.clean, chk.report()
    assert rcv.clean, rcv.report()
    assert (rcv.frames[0].card.facility, rcv.frames[0].card.number) == (facility, number)


def test_two_frames_arrive_separate_and_in_order():
    """Two swipes, a frame gap apart: two frames, in order, with the bits of each."""
    bus, rdr, rcv, chk = bench(facility=None, number=None)
    rdr.queue_card(1, 2)
    rdr.queue_card(200, 40000)
    swipe(bus, rdr)

    assert chk.clean, chk.report()
    assert len(chk.frames) == 2 and len(rcv.frames) == 2
    assert [f.card.number for f in rcv.frames] == [2, 40000]
    assert chk.frames[1].start - chk.frames[0].end >= chk.min["pi_max"], \
        "the reader must leave a full frame gap, or the frames run together"


def test_the_lines_idle_high_until_a_pulse_and_after_it():
    """The bus idles at 1 on both lines, is dragged low one line at a time, and comes back.

    This is the physical layer of the protocol: DATA0 low means a 0, DATA1 low means a 1,
    both high means idle (03_wikipedia_wiegand_interface.md). It is also why the nets are
    open-drain: a device that drove 1 would fight the pull-up.
    """
    bus, rdr, rcv, chk = bench(facility=None, number=None)
    assert bus.values == {"d0": 1, "d1": 1}, "both lines idle high before anything is sent"
    rdr.queue_bits("01")
    assert bus.run_until(lambda b: b.values["d0"] == 0, 100), "the 0 never came out"
    assert bus.values == {"d0": 0, "d1": 1}, "a 0 is a low pulse on DATA0 alone"
    assert bus.run_until(lambda b: b.values["d0"] == 1, 100), "the pulse never ended"
    assert bus.values == {"d0": 1, "d1": 1}, "the line comes back up between pulses"
    assert bus.run_until(lambda b: b.values["d1"] == 0, 10_000), "the 1 never came out"
    assert bus.values == {"d0": 1, "d1": 0}, "a 1 is a low pulse on DATA1 alone"
    swipe(bus, rdr)
    assert bus.values == {"d0": 1, "d1": 1}, "and both lines idle high again afterwards"
    assert rcv.bits == [0, 1] and rcv.collisions == 0


def test_the_checker_measures_the_timing_the_reader_sent():
    """What the checker measured is the reader's timing, in whole ticks, and inside the
    SIA window -- so the nominal traffic really is legal rather than merely unflagged."""
    bus, rdr, rcv, chk = bench()
    swipe(bus, rdr)

    assert (chk.min_pw, chk.max_pw) == (rdr.width_ticks, rdr.width_ticks)
    assert chk.min_pi == rdr.gap_ticks
    assert chk.min["pw_min"] <= chk.min_pw <= chk.max_pw <= chk.min["pw_max"]
    assert chk.min["pi_min"] <= chk.min_pi <= chk.min["pi_max"]
    assert len(chk.pulses) == 26 and chk.pulses[0].index == 0
    assert {p.bit for p in chk.pulses} == {0, 1}       # both lines really carried bits


# ---- the frame, against the saved specification -------------------------------------
def test_the_frame_bits_are_the_bit_map_in_the_reference():
    """`encode_card` against a bit string worked out by hand from Pyramid's Table 2
    (research/ref/protocols/wiegand/02_pyramid_26bit_format.md), not from the model:
    FC 123 = 0b01111011 into bits 2..9, ID 4567 = 0b0001000110100111 into bits 10..25,
    even parity of bits 2..13 (7 ones, odd) in bit 1, odd parity of bits 14..25 (7 ones,
    odd) in bit 26."""
    facility, number = CARD
    bits = encode_card("sia26", facility, number)

    assert "".join(str(b) for b in bits) == "10111101100010001110101110"
    # and the fields sit where Table 2 puts them
    assert bits[1:9] == [int(c) for c in f"{facility:08b}"]
    assert bits[9:25] == [int(c) for c in f"{number:016b}"]
    assert bits[0] == parity_bit(bits[1:13], "even")
    assert bits[25] == parity_bit(bits[13:25], "odd")


def test_the_layout_table_is_the_numbers_in_the_reference():
    """The layout is data, and the data is the sources: positions, widths and parity
    polarity, read straight out of 02_pyramid_26bit_format.md, 05_accessgrid and
    06_proxmark3_h10301.c.md. This is the test that stops the table drifting."""
    sia = wiegand_format("sia26")
    assert (sia.bits, sia.facility, sia.number) == (26, (2, 8), (10, 16))
    assert (sia.lead, sia.trail) == ((1, "even"), (26, "odd"))
    assert (sia.lead_covers, sia.trail_covers) == (tuple(range(2, 14)), tuple(range(14, 26)))
    assert (len(sia.lead_covers), len(sia.trail_covers)) == (12, 12)

    h10301 = wiegand_format("h10301")           # proxmark3: 16 ID bits first, odd parity first
    assert (h10301.facility, h10301.number) == ((18, 8), (2, 16))
    assert (h10301.lead, h10301.trail) == ((1, "odd"), (26, "even"))

    ind26 = wiegand_format("ind26")             # proxmark3 ind26: 12 + 12
    assert (ind26.facility, ind26.number) == ((2, 12), (14, 12))
    assert (ind26.lead, ind26.trail) == ((1, "even"), (26, "odd"))
    assert all(f.bits == 26 for f in WIEGAND_FORMATS.values())


def test_the_h10301_parity_polarity_really_is_the_other_way_round():
    """The one disagreement between the sources, handled rather than papered over.

    Pyramid, AccessGrid and BALTECH make the leading bit even parity over the first 12
    bits; proxmark3's H10301 makes it odd (06_proxmark3_h10301.c.md). So a frame that is
    correct in one is *not* correct in the other, in either polarity, and the two layouts
    do not even put the facility code in the same bits. A frame is only readable by a
    panel configured for its layout, and the checker has to say so.
    """
    sia = encode_card("sia26", *CARD)
    assert decode_card("sia26", sia).parity_ok
    h10301 = encode_card("h10301", *CARD)
    assert decode_card("h10301", h10301).parity_ok
    assert decode_card("h10301", h10301).parity_ok is not decode_card("sia26", h10301).parity_ok

    # neither half's parity survives the other layout, and the fields move with them
    for bits, other in ((sia, "h10301"), (h10301, "sia26")):
        card = decode_card(other, bits)
        assert not card.parity_ok, f"{other} accepted a {other}-foreign frame"
        assert (card.facility, card.number) != CARD, "the layouts do not share field bits"

    # on the wire: a correct h10301 frame is rejected by a sia26 checker, on both parities
    bus, rdr, rcv, chk = bench(bit_format="h10301")
    chk.bit_format, chk.format = "sia26", wiegand_format("sia26")
    swipe(bus, rdr)
    assert rules(chk) == {"PARITY;LEAD", "PARITY;TRAIL"}, chk.report()


# ---- one test per rule the checker enforces ---------------------------------------
def test_checker_flags_a_pulse_shorter_than_the_width_floor():
    """tPW;MIN: a reader pulsing 10 us instead of 20 us. The first pulse is the one that
    goes short, and the violation carries its tick."""
    bus, rdr, rcv, chk = bench(reader={"pw": 10_000})
    swipe(bus, rdr)

    flagged = chk.violations_by_rule("tPW;MIN")
    assert len(flagged) == 26, "every one of the 26 pulses is too narrow"
    first = flagged[0]
    assert first.tick == chk.pulses[0].rise, "the tick is when the pulse ended"
    assert first.measured == 10 and first.required == chk.min["pw_min"] == 20
    assert "bit 0" in first.detail
    assert rules(chk) == {"tPW;MIN"}, chk.report()
    # the bits still arrive: a panel that only looks at edges cannot see this fault
    assert rcv.bits == encode_card("sia26", *CARD) and len(rcv.frames) == 1


def test_checker_flags_a_pulse_wider_than_the_width_ceiling():
    """tPW;MAX: 200 us pulses against a 100 us ceiling. A ceiling is a different kind of
    rule from a floor, so this also proves the checker is not just testing 'long enough'."""
    bus, rdr, rcv, chk = bench(reader={"pw": 200_000})
    swipe(bus, rdr)

    flagged = chk.violations_by_rule("tPW;MAX")
    assert len(flagged) == 26, "one per pulse, however it was noticed"
    assert all(v.measured > v.required == chk.min["pw_max"] for v in flagged)
    # A pulse still low when it goes over is reported there and then, one tick past the
    # ceiling, rather than 100 ticks later when it finally rises; so the measured width
    # is the ceiling plus a tick, not the 200 the reader asked for.
    assert all(v.measured == chk.min["pw_max"] + 1 for v in flagged)
    assert flagged[0].tick == chk.pulses[0].fall + chk.min["pw_max"] + 1
    assert rules(chk) == {"tPW;MAX"}, chk.report()
    assert len(rcv.bits) == 26 and rcv.clean, rcv.report()


def test_checker_flags_a_pause_shorter_than_the_interval_floor():
    """tPI;MIN: a reader that rushes from one pulse to the next. Every pause in the frame
    is 100 us against a 200 us floor, so all 25 pauses are flagged."""
    bus, rdr, rcv, chk = bench(reader={"pi": 100_000})
    swipe(bus, rdr)

    flagged = chk.violations_by_rule("tPI;MIN")
    assert len(flagged) == 25, "there are 25 pauses in a 26-pulse frame"
    assert all(v.measured == 100 < v.required == chk.min["pi_min"] for v in flagged)
    assert rules(chk) == {"tPI;MIN"}, chk.report()
    assert chk.min_pi == 100


def test_checker_flags_a_pause_past_the_interval_ceiling():
    """tPI;MAX: a reader so slow that its pauses pass the 20 ms ceiling. The consequence
    is the interesting part: a receiver has to close the frame when the lines go quiet, so
    a 30 ms pause cuts the frame after one bit and the card never arrives."""
    bus, rdr, rcv, chk = bench(reader={"pi": 30_000_000})
    swipe(bus, rdr)

    flagged = chk.violations_by_rule("tPI;MAX")
    assert flagged, chk.report()
    assert all(v.measured >= v.required == chk.min["pi_max"] for v in flagged)
    assert "after only 1 of 26 bits" in flagged[0].detail
    assert rules(chk) == {"tPI;MAX", "BITS"}, chk.report()
    assert len(chk.frames) == 26, "every pulse became a one-bit frame"
    assert all(f.bits == [f.bits[0]] for f in chk.frames)


def test_checker_flags_both_lines_low_together():
    """D0D1: the one thing the specification forbids outright, "the pulses will not overlap
    or occur simultaneously" (01_..., 04_...). Two ways onto the wire: a shorted pair held
    low, and a second reader wired in parallel."""
    bus, rdr, rcv, chk = bench(facility=None, number=None, extra=ShortedPair(2_000, 300))
    rdr.queue_card(*CARD)
    swipe(bus, rdr)

    flagged = chk.violations_by_rule("D0D1")
    assert len(flagged) == 1, "one short is one violation, not one per tick"
    assert flagged[0].tick == 2_001, "the tick the pair went low"
    assert "must not overlap" in flagged[0].detail
    assert rcv.collisions == 300, "the receiver sees both lines low for the whole short"

    # The second reader has to use the line the card's first pulse does *not*: with both
    # readers on DATA1 the second one simply holds a line the first is already using and
    # no line is ever low twice. CARD's first bit is 1, so its first pulse is on DATA1.
    assert encode_card("sia26", *CARD)[0] == 1
    bus2, rdr2, rcv2, chk2 = bench(facility=None, number=None,
                                   extra=SecondReader(4, rdr.width_ticks - 10, line="d0"))
    rdr2.queue_card(*CARD)
    swipe(bus2, rdr2)
    flagged2 = chk2.violations_by_rule("D0D1")
    assert len(flagged2) == 1 and flagged2[0].tick == 5, chk2.report()
    assert rcv2.collisions == rdr.width_ticks - 10


def test_checker_flags_a_line_that_never_comes_back_up():
    """A stuck-low line has no end to time, so the checker has to catch the width going
    over the ceiling while the pulse is still open, not wait for a rise that never comes."""
    # A line dragged low for good. Its own pulses then have no falling edge to show, which
    # is why the width rule has to watch a pulse that is still open: nothing else sees it.
    class StuckLow(Device):
        name = "stuck"

        def tick(self, t, pins) -> dict:
            return {"d0": 0}

    bus, rdr, rcv, chk = bench(facility=None, number=None, extra=StuckLow())
    rdr.queue_bits([0, 0, 0])              # three bits on the same stuck line
    bus.run(chk.min["pw_max"] + 10)

    flagged = chk.violations_by_rule("tPW;MAX")
    assert flagged, chk.report()
    assert len(flagged) == 1, "the line that is stuck, once"
    assert flagged[0].measured == chk.min["pw_max"] + 1
    assert flagged[0].tick - 1 == flagged[0].measured, "measured from the tick it fell"
    assert not chk.violations_by_rule("D0D1"), "only one line is ever low"
    assert len(rcv.bits) == 0, "no rising edge ever arrived, so no bit did"


@pytest.mark.parametrize("extra_bit", [True, False])
def test_checker_flags_a_frame_of_the_wrong_length(extra_bit):
    """BITS: 25 bits is a short frame and 27 is a long one. The check is `exact`, so both
    directions have to be caught -- a frame-length rule that only counts up to the limit
    would pass a reader that never stops talking."""
    bits = encode_card("sia26", *CARD)
    sent = bits + [1] if extra_bit else bits[:-1]
    bus, rdr, rcv, chk = bench(facility=None, number=None)
    rdr.queue_bits(sent)
    swipe(bus, rdr)

    flagged = chk.violations_by_rule("BITS")
    assert len(flagged) == 1, f"one frame of {len(sent)} bits, one violation"
    assert (flagged[0].measured, flagged[0].required) == (len(sent), 26)
    # A short frame is reported twice over, and that is not double counting: the lines went
    # quiet to the frame timeout with bits still missing, which is both a pause that
    # reached the interval ceiling (tPI;MAX) and a frame of the wrong length (BITS). A
    # long frame has nothing missing, so it only trips BITS.
    assert rules(chk) == ({"BITS", "tPI;MAX"} if not extra_bit else {"BITS"}), chk.report()
    # a frame of the wrong length has no facility code or ID number, and must not pretend
    assert chk.frames[0].card is None and chk.frames[0].facility == -1
    assert rcv.clean is False and rcv.errors, "the receiver reports it too"


@pytest.mark.parametrize("bit_index,rule", [(0, "PARITY;LEAD"), (25, "PARITY;TRAIL")])
def test_checker_flags_each_parity_bit(bit_index, rule):
    """PARITY;LEAD and PARITY;TRAIL: one parity bit flipped. Both are caught on the tick
    the frame closed, because that is the first moment the frame's bits are all known."""
    bits = encode_card("sia26", *CARD)
    bits[bit_index] ^= 1
    bus, rdr, rcv, chk = bench(facility=None, number=None)
    rdr.queue_bits(bits)
    swipe(bus, rdr)

    assert rules(chk) == {rule}, chk.report()
    assert len(chk.violations) == 1, "exactly one rule, so the two do not double-report"
    assert chk.violations[0].tick >= chk.frames[0].end
    # the data is intact, which is what makes it a parity error rather than a format error
    assert (chk.frames[0].facility, chk.frames[0].number) == CARD
    assert not chk.frames[0].card.parity_ok
    assert rcv.clean is False and "parity wrong" in rcv.errors[0]


# ---- the checker is not vacuous ---------------------------------------------------
def test_checker_is_not_vacuous():
    """Legal traffic judged against stricter limits must fail.

    This is the test that would catch a checker that never looks: the same wire, the same
    reader, the same bits, only the limits moved in. tPW;MIN and tPI;MIN must fire because
    50 us and 2 ms no longer meet a 60 us floor and a 3 ms floor, which also shows the
    checker reads its limits from the profile and its overrides rather than from constants.
    """
    bus, rdr, rcv, chk = bench()
    swipe(bus, rdr)
    assert chk.clean, chk.report()
    assert len(chk.pulses) == 26 and len(chk.frames) == 1, "it really did watch the frame"

    strict_timing = {"pw_min": 60_000, "pi_min": 3_000_000}
    bus2, rdr2, rcv2, chk2 = bench(check_timing=strict_timing)
    swipe(bus2, rdr2)
    assert rules(chk2) == {"tPW;MIN", "tPI;MIN"}, chk2.report()
    assert len(chk2.violations_by_rule("tPW;MIN")) == 26
    assert len(chk2.violations_by_rule("tPI;MIN")) == 25
    assert all(v.tick > 0 for v in chk2.violations), "every violation carries a tick"
    assert all(v.measured < v.required for v in chk2.violations), "both are floors"
    # the reader and the receiver are untouched by the checker's limits
    assert rdr.errors == [] and rcv2.clean, rcv2.report()


def test_checker_says_nothing_about_an_idle_bus():
    """The other half of not being vacuous: no traffic is not a fault. A checker that
    flagged a quiet bus would be unusable, and one that flagged nothing here and something
    on every frame would be the same bug with the sign flipped."""
    bus, rdr, rcv, chk = bench(facility=None, number=None)
    bus.run(3 * chk.min["pi_max"])            # three frame timeouts of nothing at all

    assert chk.clean and chk.frames == [] and chk.pulses == []
    assert rcv.frames == [] and rcv.bits == []
    assert not chk.both_low
    # and the first pulse of a real frame after all that quiet is still legal
    rdr.queue_card(*CARD)
    swipe(bus, rdr)
    assert chk.clean, chk.report()
    assert len(chk.frames) == 1


def test_one_flipped_bit_anywhere_in_the_frame_is_caught():
    """Every single-bit error in the 26-bit frame must be caught: all 24 data bits by one
    of the two parity bits, and a flipped parity bit by itself. Done on the decoder, so all
    26 positions are covered without 26 bus runs, and then once on the wire to show the
    checker reaches the same verdict from the pins."""
    bits = encode_card("sia26", *CARD)
    for index in range(26):
        broken = list(bits)
        broken[index] ^= 1
        card = decode_card("sia26", broken)
        assert card is not None, "the frame is still 26 bits, so it still decodes"
        assert not card.parity_ok, f"flipping bit {index} went unnoticed"
        if index in (0, 25):
            assert not (card.lead_ok if index == 0 else card.trail_ok) and \
                (card.lead_ok if index == 25 else card.trail_ok), \
                "a flipped parity bit spoils only its own half"

    for index in (3, 17):                    # one data bit, one through the wire
        broken = list(bits)
        broken[index] ^= 1
        bus, rdr, rcv, chk = bench(facility=None, number=None)
        rdr.queue_bits(broken)
        swipe(bus, rdr)
        assert rules(chk) <= {"PARITY;LEAD", "PARITY;TRAIL"}, chk.report()
        assert chk.violations, f"bit {index} went unnoticed on the wire"


# ---- the chip's own clock, and the guard rails -------------------------------------
def test_the_chip_clock_rate_quantises_the_same_limits():
    """At the chip's 50 MHz (20 ns/tick) the limits are the same nanoseconds in ticks:
    the SIA window becomes 1000-5000 ticks of pulse width and 10000-1000000 of pause.

    Floors are rounded up and ceilings down, so the checker is never on the lenient side:
    20 us exactly is legal, 19.98 us (999 ticks) is not; 100 us exactly is legal, 100.02 us
    (5001 ticks) is not. Four bits at the floor rather than 26 at the nominal keeps this
    to a few hundred thousand ticks.
    """
    at_chip = spec_minimums(CHIP_NS_PER_TICK)
    assert (at_chip["pw_min"], at_chip["pw_max"]) == (1_000, 5_000)
    assert (at_chip["pi_min"], at_chip["pi_max"]) == (10_000, 1_000_000)
    assert at_chip["frame_gap"] == 1_000_001
    at_mhz = spec_minimums(NS_PER_TICK)
    assert (at_mhz["pw_min"], at_mhz["pw_max"], at_mhz["pi_min"], at_mhz["pi_max"]) == \
        (20, 100, 200, 20_000)

    def four_bits(pw_ns, pi_ns):
        bus, rdr, rcv, chk = bench(ns_per_tick=CHIP_NS_PER_TICK, facility=None, number=None)
        rdr.queue_bits([1, 0, 1, 0], widths=[pw_ns] * 4, gaps=[pi_ns] * 4)
        bus.run(4 * (pw_ns + pi_ns) // int(CHIP_NS_PER_TICK) + 20)
        return chk

    assert not four_bits(20_000, 200_000).violations, "exactly on both floors is legal"
    assert "tPW;MIN" in rules(four_bits(19_980, 200_000)), "999 ticks is under the floor"
    assert "tPI;MIN" in rules(four_bits(20_000, 199_980)), "9999 ticks is under the floor"
    assert "tPW;MAX" in rules(four_bits(100_020, 200_000)), "5001 ticks is over the ceiling"
    assert not four_bits(100_000, 200_000).violations, "exactly on the ceiling is legal"


def test_the_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting. These are the SIA AC-01 26-bit window
    quoted in research/ref/protocols/wiegand/01_farpointe_understanding_wiegand.md, in ns:
    a pulse width of 20 to 100 us and a pulse interval of 200 us to 20 ms."""
    ns = spec_minimums(20.0)["ns"]
    assert (ns["pw_min"], ns["pw_max"]) == (20_000, 100_000)
    assert (ns["pi_min"], ns["pi_max"]) == (200_000, 20_000_000)
    # the reader's defaults are a real reader's, and they sit inside that window
    from verif.vip.wiegand import LEGAL_DEFAULTS_NS
    assert LEGAL_DEFAULTS_NS == (50_000, 2_000_000)
    assert ns["pw_min"] <= LEGAL_DEFAULTS_NS[0] <= ns["pw_max"]
    assert ns["pi_min"] <= LEGAL_DEFAULTS_NS[1] <= ns["pi_max"]
    with pytest.raises(ValueError):
        spec_minimums(20.0, "wiegand-ii-plus")
    with pytest.raises(ValueError):
        spec_minimums(20.0, "sia26", {"pw_wobble": 1})
    with pytest.raises(ValueError):
        spec_minimums(20.0, "sia26", {"pw_min": 200_000})      # floor above the ceiling
    with pytest.raises(ValueError):
        spec_minimums(20.0, "sia26", {"pi_min": 30_000_000})
    with pytest.raises(ValueError):
        ns_to_ticks(20.0, -1)


def test_the_models_reject_nonsense():
    """Guard rails: a bit that is not a bit, a length mismatch, a credential too big for
    the layout, an unknown layout, and a reader with no bus to run on."""
    bus, rdr, rcv, chk = bench(facility=None, number=None)
    with pytest.raises(ValueError):
        rdr.queue_bits([0, 1, 2])
    with pytest.raises(ValueError):
        rdr.queue_bits("0101", widths=[50_000, 50_000])
    with pytest.raises(ValueError):
        rdr.queue_card(256, 0)                 # sia26 has 8 facility bits
    with pytest.raises(ValueError):
        rdr.queue_card(0, 65536)               # and 16 number bits
    with pytest.raises(ValueError):
        encode_card("hid35", 1, 2)
    with pytest.raises(ValueError):
        wiegand_format("wiegand-ii-plus")
    with pytest.raises(ValueError):
        parity_bit([1, 0], "maybe")
    with pytest.raises(ValueError):
        encode_card("sia26", -1, 0)
    assert decode_card("sia26", [1] * 25) is None, "no plausible card from a short frame"
    rdr.queue_bits([1] * 25)
    assert rdr.errors and "queued 25 bits" in rdr.errors[0], \
        "the reader notes a stimulus that cannot be its layout"

    from verif.vip.wiegand import WiegandReader
    loose = WiegandReader()                    # built without a bus
    with pytest.raises(ValueError):
        loose.run()                            # and asked to run with no bus at all
    assert loose.run(bus, limit=1_000_000), "or handed one at run time"


def test_run_gives_up_at_the_limit_instead_of_hanging():
    """`run()` is bounded: a queue that cannot finish inside `limit` ticks returns False
    rather than looping forever, which is what a test harness needs."""
    bus, rdr, rcv, chk = bench(facility=None, number=None)
    rdr.queue_card(*CARD)
    assert not rdr.run(bus, limit=1_000), "one frame of 73 k ticks does not fit in 1 k"
    assert not rdr.idle and rdr.queue == [], "the queue is being worked on, not lost"
    assert rdr.run(bus, limit=200_000)
    assert rdr.idle and len(rdr.sent) == 1


def test_the_receiver_timeout_can_be_pointed_at_another_decoder():
    """A panel's frame boundary is its own choice: the guide in 05_accessgrid uses a 25 ms
    timer where the SIA maximum interval is 20 ms, so the same wire decodes differently."""
    bus, rdr, rcv, chk = bench()
    swipe(bus, rdr)
    assert len(rcv.frames) == 1

    # A 25 ms timer still sees one frame, but only once the lines have been quiet that
    # long: the reader stops after its own frame gap, so the run has to go on for it.
    bus2, rdr2, rcv2, chk2 = bench(facility=None, number=None,
                                   receiver={"timeout_ns": 25_000_000})
    rdr2.queue_card(*CARD)
    swipe(bus2, rdr2)
    assert rcv2.frames == [], "the reader stopped before this decoder's timeout"
    bus2.run(25_000)
    assert len(rcv2.frames) == 1 and rcv2.clean, rcv2.report()

    # A decoder whose timeout is shorter than the reader's pause never sees a whole frame:
    # it times out between every pair of bits and reports one-bit frames, which is the
    # whole reason a panel has to be told the pulse interval and not just the frame length.
    bus3, rdr3, rcv3, chk3 = bench(facility=None, number=None,
                                   receiver={"timeout_ns": 1_000_000})
    rdr3.queue_card(*CARD)
    swipe(bus3, rdr3)
    bus3.run(1_000)
    assert rcv3.frames and all(len(f.bits) == 1 for f in rcv3.frames), rcv3.report()
    assert rcv3.clean is False, "and it says so rather than inventing a card"

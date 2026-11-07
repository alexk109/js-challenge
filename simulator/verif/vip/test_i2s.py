"""Self-tests for the pin-level I2S reference models: `make vip-selftest VIP=i2s`.

Everything runs on the tick bus of `verif/vip/core.py` at 50 MHz (20 ns per tick). `rig()` builds
a transmitter, a receiver and a passive checker on one bus from one geometry; `run_stream()` drives
a few frames through it and flushes the last slot, which is what both the receiver and the checker
need at the end of a finite stream (§6.2 writes a slot out "On the next WS-level change", and a
stream that stops never produces one).

The tests come in five groups:

* **helpers** -- `bit_clock_hz`, `ticks_per_bit_clock`, `i2s_limits`, the word helpers: the numbers
  and the rounding, including the worked example from the cited text (44.1 kHz x 16 x 2 = 1.4112 MHz).
* **models** -- the transmitter's wire format and idle levels, and the receiver decoding it, over
  a matrix of 8-96 kHz, 16/24/32-bit words, 16/24/32-bit slots and 1/2/4 channels.
* **legal traffic** -- the checker passes, for every geometry and for the `abs25` profile.
* **one test per spec rule** -- `RULE_FAULTS` maps every rule in `I2sChecker.rule_help()` to a
  deliberately broken transmitter or checker setting, and each is flagged with its tick.
* **not vacuous** -- the legal runs are shown to have been *examined* (slots, periods and WS
  transitions were seen), and the union of the rules the fault runs trigger is asserted to cover
  every rule the checker claims to enforce, so a rule cannot be added without a test.

Every rule and every number is cited to `research/ref/protocols/i2s/`, which holds the primary
specification (NXP UM11732, the reconstruction of the Philips I2S bus specification) with its URL
and fetch date.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.i2s import (
    I2S_PROFILES,
    LEFT,
    MIN_TICKS_PER_BIT_CLOCK,
    NETS,
    RIGHT,
    I2sChecker,
    I2sReceiver,
    I2sTransmitter,
    Violation,
    attach_bench,
    attach_checker,
    attach_receiver,
    attach_transmitter,
    bit_clock_hz,
    i2s_limits,
    ns_to_ticks,
    run_frames,
    sample_to_word,
    ticks_per_bit_clock,
    word_bits_msb_first,
    word_to_sample,
)

NS_PER_TICK = 20.0                 # 50 MHz chip clock, the default in verif/vip/core.py
FRAME = (0x1234, -0x5678, 0x0001, -0x8000)     # two stereo frames, MSB first, two's complement
FRAMES = 2

#: The supported geometries plus the awkward ones: 24-bit data in a 32-bit slot
#: ("16-, 24- or 32-bit data length ... and to 16- or 32-bit packet length", ST AN4309 §1.2) and
#: more channels than a stereo pair.
GEOMETRIES = [
    {"sample_rate": 8_000, "word_bits": 16},                  # the bottom of the supported range
    {"sample_rate": 44_100, "word_bits": 16},                 # CD audio
    {"sample_rate": 48_000, "word_bits": 16},
    {"sample_rate": 48_000, "word_bits": 24},
    {"sample_rate": 48_000, "word_bits": 32},
    {"sample_rate": 48_000, "word_bits": 24, "slot_bits": 32},  # 24 bits in a 32-bit slot
    {"sample_rate": 96_000, "word_bits": 32},                 # the top of the supported range
    {"sample_rate": 48_000, "word_bits": 16, "slots": 1},     # one channel: no WS toggling
    {"sample_rate": 48_000, "word_bits": 16, "slots": 4},     # four channels
]


def rig(**geometry):
    """`(bus, (tx, rx, chk))`: a bus with all three I2S devices on the same geometry.

    Any keyword may name a device instead of the geometry (`tx=`, `rx=`, `chk=`), which is how a
    test points one part of the bench off-spec -- or off-configuration -- without touching the
    others. The defaults are 48 kHz, 16-bit words, stereo, 16-bit slots.
    """
    geom = {"sample_rate": 48_000, "word_bits": 16, "slots": 2}
    geom.update({k: v for k, v in geometry.items() if k not in ("tx", "rx", "chk")})
    tx = {**geom, **geometry.get("tx", {})}
    rx = {**geom, **geometry.get("rx", {})}
    chk = {**geom, **geometry.get("chk", {})}
    bus = Bus(ns_per_tick=NS_PER_TICK)
    return bus, attach_bench(bus, tx, rx, chk)


def run_stream(bus, tx, rx=None, chk=None, words=FRAME, frames=None):
    """Queue `words`, run the stream out, flush the last slot on the receiver and the checker.

    `frames` defaults to whole frames of the transmitter's own `slots` samples per frame.
    """
    tx.send(*words)
    if frames is None:
        frames = len(words) // tx.slots
    ok = run_frames(bus, tx, rx, frames=frames)
    assert ok, f"the stream did not finish in {4_000_000} ticks"
    if rx is not None:
        rx.flush()
    if chk is not None:
        chk.flush()
    return ok


def rules(chk):
    """The set of rules a checker flagged."""
    return {v.rule for v in chk.violations}


class TrailingWsTransmitter(I2sTransmitter):
    """The same stream as `I2sTransmitter`, with every WS change on the *trailing* edge that
    precedes its MSB edge -- §3.2's "WS may change either on a trailing or leading edge" read the
    other way round.

    It is three lines because `ws_offset_ticks` cannot express it: a negative offset moves the
    first boundary too (it is pinned to the start of the stream) and so skews the first slot, while
    a positive one only moves it later. No *model* needs this layout -- the VIP rejects it by
    default -- so it lives here, in the test that puts it on the wire to show that it is flagged.
    """

    def _ws_level(self, t: int) -> int:
        first = self._first_rise + self.slot_span - self.low
        seen = 0 if t < first else 1 + (t - first) // self.slot_span
        return self._ws_idle ^ (seen & 1)


def rig_trailing_ws(**geometry):
    """`rig()` with a `TrailingWsTransmitter` in place of the normal transmitter."""
    geom = {"sample_rate": 48_000, "word_bits": 16, "slots": 2,
            **{k: v for k, v in geometry.items() if k not in ("rx", "chk")}}
    bus = Bus(ns_per_tick=NS_PER_TICK)
    _, rx, chk = attach_bench(bus, None,
                              {**geom, **geometry.get("rx", {})},
                              {**geom, **geometry.get("chk", {})})
    tx = TrailingWsTransmitter(bus=bus, **geom)
    bus.attach(tx)
    return bus, (tx, rx, chk)


# ---- helpers ---------------------------------------------------------------------
def test_bit_clock_hz_is_the_sample_rate_times_the_word_times_the_channels():
    # Wikipedia, "Details": "The bit clock frequency is the product of the sample rate, the
    # number of bits per channel and the number of channels", with its worked example.
    assert bit_clock_hz(44_100, 16, 2) == pytest.approx(1_411_200)
    assert bit_clock_hz(48_000, 16, 2) == pytest.approx(1_536_000)
    assert bit_clock_hz(96_000, 32, 2) == pytest.approx(6_144_000)
    assert bit_clock_hz(8_000, 16, 2) == pytest.approx(256_000)
    # three arguments: 24-bit words in 32-bit slots run at the slot rate
    assert bit_clock_hz(48_000, 24, 2, slot_bits=32) == pytest.approx(3_072_000)


def test_bit_clock_hz_rejects_nonsense():
    for args in ((0, 16, 2), (-1, 16, 2), (48_000, 33, 2), (48_000, 16, 0), (48_000, 16, 9)):
        with pytest.raises(ValueError):
            bit_clock_hz(*args)


def test_ticks_per_bit_clock_rounds_up_and_never_claims_too_much():
    # 48 kHz stereo 16-bit is 651.04 ns per bit clock, which is 32.55 ticks at 20 ns
    assert ticks_per_bit_clock(NS_PER_TICK, 48_000, 16, 2) == 33
    # rounding up means the modelled clock is at or below the nominal frequency
    assert 1e9 / (NS_PER_TICK * ticks_per_bit_clock(NS_PER_TICK, 48_000, 16, 2)) <= 1_536_000
    assert ticks_per_bit_clock(NS_PER_TICK, 96_000, 32, 2) == 9          # 162.8 ns -> 9 ticks
    assert ticks_per_bit_clock(NS_PER_TICK, 8_000, 16, 2) == 196         # 3906.25 ns -> 196
    assert ticks_per_bit_clock(NS_PER_TICK, 48_000, 24, 2, slot_bits=32) == 17


def test_ticks_per_bit_clock_refuses_what_it_cannot_time():
    with pytest.raises(ValueError, match="MIN_TICKS_PER_BIT_CLOCK"):
        # 768 kHz stereo 16-bit would be a 40 ns period: 2 ticks at 20 ns
        ticks_per_bit_clock(NS_PER_TICK, 768_000, 16, 2)
    with pytest.raises(ValueError):
        ticks_per_bit_clock(NS_PER_TICK)              # no rate and no slot length
    with pytest.raises(ValueError):
        ticks_per_bit_clock(NS_PER_TICK, 48_000, 16, 2, slot_bits=0)


def test_i2s_limits_rounds_every_fraction_to_whole_ticks():
    lim = i2s_limits(NS_PER_TICK, sample_rate=48_000, word_bits=16, slots=2)
    assert lim["period"] == 33
    # 0.35 * 33 = 11.55 -> 12 ticks, so the check is never lenient
    assert lim["t_hc"] == lim["t_lc"] == 12
    # 0.8 * 33 = 26.4 -> 26 ticks, a ceiling rounded down
    assert lim["t_dtr"] == 26
    assert lim["t_su"] == 7                  # 0.2 * 33 = 6.6 -> 7
    assert lim["t_htr"] == 1                 # thtr >= 0, but a whole tick is the resolution
    assert lim["bit_clock_hz"] == pytest.approx(1e9 / (NS_PER_TICK * 33))
    assert lim["sample_rate"] == 48_000


def test_i2s_limits_geometry_and_absolute_profiles():
    lim = i2s_limits(NS_PER_TICK, sample_rate=48_000, word_bits=24, slot_bits=32)
    assert (lim["word_bits"], lim["slot_bits"], lim["slots"]) == (24, 32, 2)
    assert lim["period"] == 17                # 48k x 32 x 2 = 3.072 MHz -> 326 ns -> 17 ticks
    # the "relative" profile states no absolute limit at all
    assert (lim["ns_hc"], lim["ns_lc"], lim["ns_t_min"], lim["ns_t_max"]) == (0, 0, 0, 0)
    # "abs25" is Tables 1 and 2 at 2.5 MHz, in ticks at 20 ns
    a = i2s_limits(NS_PER_TICK, ticks_per_period=20, profile="abs25")
    assert (a["ns_hc"], a["ns_lc"], a["ns_dtr"], a["ns_htr"], a["ns_su"]) == (8, 8, 15, 5, 3)
    assert a["ns_t_min"] == 18 and a["ns_t_max"] == 22     # 360 ns and 440 ns
    assert a["sample_rate"] == pytest.approx(1e9 / (400 * 2 * 16))


def test_i2s_limits_overrides_and_errors():
    lim = i2s_limits(NS_PER_TICK, sample_rate=48_000, timing={"t_hc": 0.5, "ns_hc": 200})
    assert lim["t_hc"] == 17                  # 0.5 * 33 = 16.5 -> 17
    assert lim["ns_hc"] == 10                 # 200 ns
    with pytest.raises(ValueError, match="unknown I2S profile"):
        i2s_limits(NS_PER_TICK, sample_rate=48_000, profile="i2c")
    with pytest.raises(ValueError, match="unknown timing override"):
        i2s_limits(NS_PER_TICK, sample_rate=48_000, timing={"t_hd_dat": 3})
    with pytest.raises(ValueError, match="shorter than word_bits"):
        i2s_limits(NS_PER_TICK, sample_rate=48_000, word_bits=32, slot_bits=16)
    with pytest.raises(ValueError):
        i2s_limits(NS_PER_TICK, sample_rate=48_000, word_bits=3)
    with pytest.raises(ValueError):
        i2s_limits(NS_PER_TICK, sample_rate=48_000, slots=9)


def test_word_helpers_are_two_complement_msb_first():
    assert word_to_sample(0x7FFF, 16) == 32767
    assert word_to_sample(0x8000, 16) == -32768
    assert word_to_sample(0xFFFF, 16) == -1
    assert word_to_sample(0xFFFFFF, 24) == -1
    assert word_to_sample(0x80000000, 32) == -2147483648
    assert sample_to_word(-1, 16) == 0xFFFF
    assert sample_to_word(-0x5678, 16) == 0xA988
    assert sample_to_word(0x12345, 16) == 0x2345          # "the word is truncated", §3.1
    assert word_bits_msb_first(0xA988, 16) == [
        1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0]
    assert word_bits_msb_first(0x80000000, 32)[0] == 1
    with pytest.raises(ValueError):
        word_to_sample(0, 33)


def test_ns_to_ticks_rounds_up_like_the_bus():
    assert ns_to_ticks(20.0, 100) == 5
    assert ns_to_ticks(20.0, 101) == 6
    assert ns_to_ticks(20.0, 0) == 0
    assert ns_to_ticks(20.0, -5) == 0


# ---- the models ------------------------------------------------------------------
@pytest.mark.parametrize("geom", GEOMETRIES, ids=lambda g: "-".join(
    f"{k}{v}" for k, v in sorted(g.items())))
def test_transmitter_and_receiver_round_trip(geom):
    """Every supported geometry: the receiver gets back exactly what the transmitter sent."""
    (bus, (tx, rx, chk)) = rig(**geom)
    word_bits, slots = geom.get("word_bits", 16), geom.get("slots", 2)
    words = [sample_to_word(v, word_bits) for v in FRAME][:len(FRAME) // slots * slots]
    frames = len(words) // slots
    run_stream(bus, tx, rx, chk, words=tuple(words))
    assert rx.values == [word_to_sample(w, word_bits) for w in words]
    assert rx.words == list(words)
    assert rx.frames == frames
    assert rx.truncated_words == 0
    # the cells of a slot past the LSB are the "bits after the LSB" §3.1 says are ignored
    assert rx.ignored_bits == frames * slots * (tx.slot_bits - word_bits)
    assert chk.clean, chk.report()
    assert chk.slots and all(s.complete for s in chk.slots)
    assert chk.frames == frames


def test_transmitter_frame_layout_is_the_one_the_spec_describes():
    """§3.1 "the MSB of the next word one clock period after the WS changes", §6.1 "at each WS-level
    change ... parallel-loading", §6.2 "the first serial data bit (the MSB) is latched into B1 on
    the rising edge of SCK"."""
    (bus, (tx, rx, chk)) = rig()
    run_stream(bus, tx, rx, chk)
    for slot in tx.tx_log:
        # WS changes on the leading edge that is a whole slot before the next slot's MSB ...
        assert slot.msb_tick - slot.ws_tick == tx.period
        # ... the MSB rides out on the first rising edge after it ...
        assert slot.msb_tick > slot.ws_tick
        # ... and the LSB rides out on the very edge where WS flips to the next slot (§6.2 writes
        # the latches out "On the next WS-level change"), one slot-span later.
        assert slot.lsb_tick == slot.end_tick - tx.period
        assert slot.channel == slot.index % tx.slots
        assert slot.frame == slot.index // tx.slots
        assert len(slot.bits) == tx.word_bits
    # the receiver saw the same ticks (its latency is already taken off)
    assert [s.msb_tick for s in rx.samples] == [s.msb_tick for s in tx.tx_log]
    assert [s.ws_tick for s in rx.samples] == [s.ws_tick for s in tx.tx_log]


def test_a_trailing_edge_ws_layout_is_flagged_not_accepted():
    """The other half of §3.2's "WS may change either on a trailing or leading edge" is *not* the
    layout §6.2's receiver decodes. A WS change on a trailing edge leaves only `low` ticks before
    the edge that latches the MSB, so the receiver and the checker would both count that slot from
    one cell too late -- which is why the default checker flags it instead of accepting it, and why
    `ws_lag_ticks` is the documented opt-in for a part that really lags half a period.
    """
    bus, (tx, rx, chk) = rig_trailing_ws()
    run_stream(bus, tx, rx, chk)
    period = tx.period
    assert tx.low == 17 and period - tx.high == 17        # 33-tick period, 50% duty
    assert rules(chk) == {"WS-1T"}, chk.report()             # the only rule it breaks
    assert [s.msb_tick - s.tick for s in chk.slots] == [17] * len(chk.slots) == [17] * 4
    assert "low half" in chk.violations[0].detail            # the message says why

    # the opt-in: told the lead is the low half, the same bus is clean -- and then mis-decoded,
    # exactly as `RULE_HELP["WS-1T"]` now says it would be
    bus, (tx, rx, chk) = rig_trailing_ws(chk={"ws_lag_ticks": 17})
    run_stream(bus, tx, rx, chk)
    assert chk.clean, chk.report()
    assert rx.values != list(FRAME)
    # a lead that is neither one period nor the low half is still flagged with `ws_lag_ticks` set
    bus, (tx, rx, chk) = rig_trailing_ws(chk={"ws_lag_ticks": period})
    run_stream(bus, tx, rx, chk)
    assert rules(chk) == {"WS-1T"}, chk.report()


def test_transmitter_is_msb_first_on_the_wire():
    """Decode the raw trace: the first cell after the WS change is the MSB (§3.1 "Serial data is
    transmitted in two's complement with the MSB first")."""
    bus = Bus(ns_per_tick=NS_PER_TICK, trace=True)
    tx = attach_transmitter(bus)
    tx.send(0xA55A, 0x1234)
    run_frames(bus, tx)
    trace = bus.trace                                            # every resolved tick
    rises = [i for i in range(1, len(trace))
             if trace[i][1]["bclk"] and not trace[i - 1][1]["bclk"]]
    changes = [i for i in range(1, len(trace)) if trace[i][1]["ws"] != trace[i - 1][1]["ws"]]
    first_slot = [trace[e][1]["sd"] for e in rises if changes[0] < e <= changes[1]]
    assert first_slot == word_bits_msb_first(0xA55A, 16)
    # the stream ends with the second slot, so its cells run to the last rising edge there is
    second_slot = [trace[e][1]["sd"] for e in rises if e > changes[1]][:tx.word_bits]
    assert second_slot == word_bits_msb_first(0x1234, 16)
    # the last edge of a slot carries the LSB and is the edge WS changes on (§6.2)
    assert changes[1] in rises


def test_transmitter_idle_levels_and_no_phantom_ws_change():
    """An idle controller holds the clock low and leaves WS and SD where they were, so a
    receiver waiting for "the next WS-level change" (§6.2) never sees one it did not get."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_transmitter(bus)
    assert (bus.values["bclk"], bus.values["ws"]) == (0, 1)     # from the net pull-ups
    assert not tx.busy and tx.idle and tx.pending == 0
    tx.send(0x1234, 0x5678)
    assert tx.busy and tx.pending == 2
    run_frames(bus, tx)
    bus.run(2000)
    assert tx.idle and bus.values["bclk"] == 0
    assert tx.pending == 0 and tx.sent == 2
    assert tx.tx_log[-1].end_tick is not None
    # no WS transition after the stream stopped: the level the last slot left it at is held
    held = bus.values["ws"]
    for _ in range(200):
        bus.step()
    assert bus.values["ws"] == held and bus.values["bclk"] == 0


def test_transmitter_stop_and_send_errors():
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_transmitter(bus)
    with pytest.raises(ValueError, match="whole number"):
        tx.send(1, 2, 3)                                  # three samples, stereo
    with pytest.raises(ValueError, match="whole number"):
        tx.send([1, 2, 3])                                 # a list is flattened, then checked
    tx.send(1, 2, 3, 4)
    tx.stop()
    assert tx.pending == 0


def test_receiver_channels_padding_and_truncation():
    # 24-bit data in a 32-bit slot: the bits after the LSB are ignored (§3.1)
    (bus, (tx, rx, chk)) = rig(word_bits=24, slot_bits=32)
    run_stream(bus, tx, rx, chk)
    assert rx.values == [word_to_sample(w, 24) for w in FRAME]
    # the 24-bit word starts one cell after the WS change ("the MSB has a fixed position, whereas
    # the position of the LSB depends on the word length", §3.1), so it is *left* justified in
    # the 32-bit slot and the eight cells after its LSB are the ignored ones
    assert rx.ignored_bits == FRAMES * 2 * (32 - 24)
    assert chk.clean, chk.report()
    # a slot that stops early: the missing bits are set to zero internally (§3.1)
    (bus, (tx, rx, chk)) = rig(word_bits=32, tx={"cells_per_slot": 24})
    run_stream(bus, tx, rx, chk)
    assert rx.truncated_words == FRAMES * 2
    assert all(s.bits == 24 for s in rx.samples)
    # A slot that stops early drops the *LSB* end of the word -- "the MSB has a fixed position,
    # whereas the position of the LSB depends on the word length" (§3.1) -- so what arrives is
    # the first 24 cells, and §3.1's "the missing bits are set to zero internally" puts zeros
    # where the LSBs would have been.
    assert rx.words == [sample_to_word(v, 32) >> 8 for v in FRAME]
    assert rx.values == [word_to_sample(w, 32) for w in rx.words]   # already sign-extended
    assert rx.samples[0].bits == 24 < 32
    assert "WORD" in rules(chk) and "SLOT" in rules(chk)


def test_receiver_latency_zero_shifts_the_ticks_it_reports():
    (bus, (tx, rx, chk)) = rig(rx={"latency": 0})
    run_stream(bus, tx, rx, chk)
    assert [s.ws_tick for s in rx.samples] == [s.ws_tick + 1 for s in tx.tx_log]
    with pytest.raises(ValueError):
        I2sReceiver(latency=-1)


def test_receiver_first_channel_and_flush():
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_transmitter(bus)
    rx = attach_receiver(bus, first_channel=RIGHT)
    tx.send(0x1111, 0x2222)
    run_frames(bus, tx, rx)
    # the first WS transition the receiver sees it calls slot 0, so `first_channel` names it
    assert rx.samples[0].channel == RIGHT
    assert rx.left == [0x2222] and rx.right == [0x1111]
    assert rx.pop().value == sample_to_word(0x1111, 16)
    assert rx.pending == 1 and rx.pop().value == sample_to_word(0x2222, 16)
    assert rx.pop() is None
    assert rx.flush() is None                     # nothing open any more


def test_receiver_notices_a_stalled_clock():
    """`max_gap_ticks` is the receiver's silence detector, and it has to be *reached*.

    A legal stream runs the bit clock at one period, so the gap it reports is the period and the
    counter never moves. Point the same receiver at a gap shorter than the clock period it sees and
    every edge counts as a stall -- which is the only way a stall can be counted at all: `stalls`
    rises on a *rising* edge, so a clock that stops dead (`tx.stop()`, or the stream simply
    running out) leaves `max_gap` and `stalls` exactly where they were. A dead clock is silence,
    not a slow one; see the checker class docstring.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_transmitter(bus)
    rx = attach_receiver(bus, max_gap_ticks=tx.period * 4)
    tx.send(*FRAME)
    bus.run_until(lambda b: rx.frames >= 1, 200_000)
    assert rx.stalls == 0
    assert rx.max_gap == tx.period

    # the same stream, the same receiver, a threshold the clock cannot meet
    (bus, (tx, rx, chk)) = rig(rx={"max_gap_ticks": 5})
    run_stream(bus, tx, rx, chk)
    assert chk.clean, chk.report()
    assert rx.max_gap == tx.period                      # the bus itself did not change
    assert rx.stalls == len(chk.periods), "every measured clock period is one 'stall'"
    assert rx.stalls > 0

    # and a clock that simply stops is not a stall: no edge, no gap, no count
    (bus, (tx, rx, chk)) = rig(rx={"max_gap_ticks": 5})
    tx.send(*FRAME)
    assert bus.run_until(lambda b: tx.sent >= 2, 200_000)
    before = rx.stalls
    tx.stop()                                            # the clock stops after this slot
    assert bus.run_until(lambda b: not tx.busy, 200_000)
    running = rx.stalls
    assert running > before > 0                          # it was counting while the clock ran
    bus.run(8 * tx.period)                               # eight periods of silence
    assert rx.stalls == running                         # ... and stops counting when it stops
    assert rx.max_gap == tx.period


# ---- legal traffic --------------------------------------------------------------
@pytest.mark.parametrize("geom", GEOMETRIES, ids=lambda g: "-".join(
    f"{k}{v}" for k, v in sorted(g.items())))
def test_checker_passes_legal_traffic(geom):
    """Zero violations for every supported geometry, and the checker saw all of it."""
    (bus, (tx, rx, chk)) = rig(**geom)
    run_stream(bus, tx, rx, chk)
    assert chk.clean, chk.report()
    words = len(FRAME) // tx.slots * tx.slots
    slots = words
    assert len(chk.slots) == slots and chk.frames == words // tx.slots
    assert len(chk.ws_ticks) == slots
    assert len(chk.periods) == slots * chk.slot_bits          # every bit clock period measured
    assert chk.edges == len(chk.periods) + 1
    assert all(s.word is not None and s.complete for s in chk.slots)
    assert not bus.errors                                     # no contention on a push-pull bus


def test_checker_passes_the_abs25_profile_at_2_5_mhz():
    """Tables 1 and 2 at their own 2.5 MHz: T = 400 ns = 20 ticks, both halves 10, the data one
    tick after the trailing edge. The absolute limits (160 ns, 300 ns, 100 ns, 60 ns) hold."""
    (bus, (tx, rx, chk)) = rig(ticks_per_period=20, profile="abs25")
    run_stream(bus, tx, rx, chk)
    assert chk.clean, chk.report()
    assert chk.min_period == chk.max_period == 20
    assert chk.limits["ns_period"] == 400.0
    assert chk.limits["sample_rate"] == pytest.approx(78_125.0)   # 2.5 MHz / 32


def test_checker_rejects_a_clock_the_abs25_profile_calls_too_fast():
    """The same relative traffic at 3.2 MHz meets every 0.35T / 0.8T rule but not Table 1's
    360 ns minimum clock period, so only the absolute part of the profile objects."""
    (bus, (tx, rx, chk)) = rig(ticks_per_period=16, profile="abs25")
    run_stream(bus, tx, rx, chk)
    assert rules(chk) == {"fSCK"}
    assert chk.limits["ns_period"] == 320.0
    # the relative profile has no opinion about the same bus
    (bus, (tx, rx, chk)) = rig(ticks_per_period=16)
    run_stream(bus, tx, rx, chk)
    assert chk.clean, chk.report()


def test_max_sck_hz_is_opt_in_because_ttr_is_a_device_figure():
    """Table 3 note [1]: T must be greater than Ttr "because both the transmitter and receiver
    have to be able to handle the data transfer rate" -- that is what the 1986 parts could do,
    not what the bus allows, so 96 kHz x 32-bit (6.144 MHz) is legal unless a test says otherwise.
    """
    (bus, (tx, rx, chk)) = rig(sample_rate=96_000, word_bits=32)
    run_stream(bus, tx, rx, chk)
    assert chk.clean, chk.report()
    assert chk.limits["bit_clock_hz"] > 5_000_000
    (bus, (tx, rx, chk)) = rig(sample_rate=96_000, word_bits=32,
                               chk={"max_sck_hz": 1 / 360e-9})           # Ttr = 360 ns = 2.78 MHz
    run_stream(bus, tx, rx, chk)
    assert rules(chk) == {"fSCK"}
    assert "faster than" in chk.violations[0].detail


#: Every rule of `I2sChecker.rule_help()`, and how to break exactly it.
#:
#: Each entry is the keyword arguments `rig()` takes: `tx=` for a transmitter that violates the
#: rule, `chk=` for a checker told to judge by a stricter limit than the transmitter obeys. The
#: optional "also" key lists the rules that come along *because* of this fault and cannot be
#: avoided -- a slot shorter than the geometry is both a SLOT and a WORD fault, and data that
#: goes out late is late for both the transmitter's 0.8T and the receiver's 0.2Tr. Every entry
#: says so; `test_each_spec_rule_is_flagged_with_its_tick` asserts that nothing else fires, so a
#: fault that quietly started breaking a fifth rule would fail.
RULE_FAULTS = {
    # Figure 2/3 and Table 3: "tHC >= 0.35T". The transmitter is given a 20% HIGH half (and the
    # relaxed limits that go with it, so the model does not clamp it), which is 7 of 33 ticks.
    "tHC": {"tx": {"high_ratio": 0.2, "timing": {"t_hc": 0.1, "t_lc": 0.1}}},
    # "tLC >= 0.35T": a LOW half of 11 of 33 ticks. The data change one tick after the trailing
    # edge still leaves 10 ticks of setup, so tD;TR and tSU;SD are untouched.
    "tLC": {"tx": {"high_ratio": 0.667, "timing": {"t_hc": 0.1, "t_lc": 0.1}}},
    # §4 "thtr >= 0": SD changes on the same tick as the leading edge that latches it, i.e.
    # zero ticks of hold, which a digital model cannot resolve. -(low + 1) does that.
    "tH;TR": {"tx": {"sd_offset_ticks": -17}},
    # Figure 2/Table 3 "tdtr <= 0.8T": SD changes 30 ticks after the leading edge, past the 26
    # (0.8 x 33) a 48 kHz 16-bit stream allows.
    "tD;TR": {"tx": {"sd_offset_ticks": 12}, "also": {"tSU;SD"}},
    # Figure 3/Table 3 "tsr >= 0.2Tr": the *receiver's* limit, so it is a checker that cannot
    # keep up: it is configured for a 60-tick period (t_su = 12) while the bus runs at 33 and the
    # data goes out 24 ticks after the leading edge, leaving 9. The rate rule fires too, because
    # a checker pointed at 60 ticks knows the bus is not running at it -- see the note below.
    # period_slack_ticks is opened up so the rate itself is not objected to: this entry is about
    # the setup time the *receiver* would get, not about the rate.
    "tSU;SD": {"tx": {"sd_offset_ticks": 6},
               "chk": {"ticks_per_period": 60, "period_slack_ticks": 30}},
    # §3.2 "WS may change either on a trailing or leading edge of the serial clock": WS moves
    # eight ticks after the edge it belongs on.
    # one tick off the edge is outside `ws_edge_slack_ticks` (0) but still within the WS-1T slack,
    # which is how WS-EDGE is broken on its own
    "WS-EDGE": {"tx": {"ws_offset_ticks": 1}},
    # §3.1 "the MSB of the next word one clock period after the WS changes". A WS change that is
    # off a clock edge is caught by WS-EDGE first, and a trailing-edge change is caught by the
    # one-period rule as such (see `test_a_trailing_edge_ws_layout_is_flagged_not_accepted`), so the
    # lead rule is broken here from the *receiver's* side: a perfectly legal bus, judged by a
    # target that was told the lead is 17 ticks. `ws_lag_ticks` states the lead in ticks -- 17 is
    # `low` at this geometry, i.e. the half-period lead a trailing-edge change really leaves.
    "WS-1T": {"chk": {"ws_lag_ticks": 17}},
    # §3.2 "WS = 0; channel 1 (left); WS = 1; channel 2 (right)": the transmitter's left channel
    # is level 1, so every slot comes out with the wrong polarity.
    "WS-CHAN": {"tx": {"left_level": RIGHT}},
    # §6.1 "At each WS-level change ... parallel-loading the shift register": the slot is 28
    # clock periods instead of the geometry's 32.
    "SLOT": {"word_bits": 32, "tx": {"cells_per_slot": 28}, "also": {"WORD"}},
    # §3.1 "If the receiver is sent fewer bits than its word length, the missing bits are set to
    # zero internally": a 24-cell slot carrying a 32-bit word. SLOT comes with it (the slot is
    # short), which is why the test below is the one place two rules are expected at once.
    "WORD": {"word_bits": 32, "tx": {"cells_per_slot": 24}, "also": {"SLOT"}},
    # Wikipedia "Details": "The bit clock frequency is the product of the sample rate, the number
    # of bits per channel and the number of channels". The bus runs at 20 ticks (2.5 MHz) while
    # the checker expects 40 (a 48 kHz 16-bit stereo stream).
    "fSCK": {"tx": {"ticks_per_period": 20}},
}


# ---- one test per spec rule ------------------------------------------------------
def _fault(name):
    """The deliberately broken transmitter and/or checker for the rule `name`."""
    entry = RULE_FAULTS[name]
    return {k: v for k, v in entry.items() if k != "also"}


def _also(name):
    """The rules that the fault for `name` unavoidably trips with it."""
    return set(RULE_FAULTS[name].get("also", ()))


def test_the_fault_table_covers_every_rule_the_checker_claims():
    """`RULE_FAULTS` must name every rule in `I2sChecker.rule_help()`: a rule with no way to
    break it is either dead code or untested, and this fails either way."""
    assert set(RULE_FAULTS) == set(I2sChecker.rule_help())


@pytest.mark.parametrize("rule", sorted(RULE_FAULTS))
def test_each_spec_rule_is_flagged_with_its_tick(rule):
    """Break one rule, check that rule fires, that nothing else does, and that every violation
    carries the tick it happened on."""
    (bus, (tx, rx, chk)) = rig(**_fault(rule))
    run_stream(bus, tx, rx, chk)
    assert rules(chk) == {rule} | _also(rule), chk.report()
    assert chk.violations, f"{rule} was not flagged at all"
    assert chk.violations_by_rule(rule), f"{rule} was not flagged at all"
    for v in chk.violations:
        assert isinstance(v, Violation)
        assert v.rule == rule or v.rule in _also(rule)
        assert v.tick > 0 and v.tick < bus.t
        assert str(v)                                     # the message names the rule and the tick


# ---- the checker is not vacuous ---------------------------------------------------
def test_a_clean_run_is_a_checked_run_not_an_idle_one():
    """The strongest statement about "no violations" is that there was something to violate: the
    checker must have decoded every slot, measured every bit-clock period and seen every WS
    transition of the stream it passed.
    """
    (bus, (tx, rx, chk)) = rig()
    run_stream(bus, tx, rx, chk)
    assert chk.clean
    assert len(chk.slots) == 4                       # four slots: FRAMES x 2 channels
    assert len(chk.ws_ticks) == 4
    assert len(chk.periods) == 4 * chk.slot_bits
    assert chk.min_period == chk.max_period == tx.period
    assert chk.rate_violations == 0
    assert all(v is None for v in [s.word for s in chk.slots]) is False   # the words were decoded
    # and the words it decoded are the ones that were sent
    assert [s.word for s in chk.slots] == [s.value for s in rx.samples]


def test_every_rule_in_the_checker_is_exercised_by_a_fault():
    """Run every fault once more and take the union: it must cover all of `rule_help()`.

    Together with `test_the_fault_table_covers_every_rule_the_checker_claims` (which keeps the
    table and the checker in step) this is what makes the suite non-vacuous: a rule cannot be
    added, renamed or quietly broken without one of these two failing.
    """
    seen = set()
    for rule in sorted(RULE_FAULTS):
        (bus, (tx, rx, chk)) = rig(**_fault(rule))
        run_stream(bus, tx, rx, chk)
        seen |= rules(chk)
        assert chk.violations, f"{rule}: the fault produced no violation at all"
    assert seen == set(I2sChecker.rule_help()), sorted(set(I2sChecker.rule_help()) - seen)


def test_a_wrong_bit_on_the_wire_is_legal_timing_and_the_receiver_takes_it():
    """What the checker cannot do, stated as a test.

    Wikipedia, "As an audio interconnect": "As the I2S bus doesn't have any error detection
    mechanism, using long cables can result in significant decoding errors." A bit that is simply
    *wrong* is a perfectly legal waveform -- it changes on a trailing edge, keeps its setup and
    its hold -- so the checker has nothing to say about it and the receiver decodes the wrong
    word. What the checker does catch is the *timing* of a change, which is what
    `test_each_spec_rule_is_flagged_with_its_tick` goes through.
    """
    class FlippedCell(I2sTransmitter):
        """The transmitter with one cell of the first slot driven with the other bit.

        This is the extension point `I2sTransmitter` documents: every pin level is a function of
        the tick, so a fault is a one-line override rather than a second state machine.
        """

        flip_cell = 2

        def _sd_level(self, t):
            cell = (t - self._first_rise + self.low - 1) // self.period - 1
            level = super()._sd_level(t)
            return 1 - level if cell == self.flip_cell else level

    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_transmitter(bus)
    FlippedCell.flip_cell = 2
    bus.devices[0] = FlippedCell(bus=bus)
    tx = bus.devices[0]
    rx = attach_receiver(bus)
    chk = attach_checker(bus)
    tx.send(*FRAME)
    run_frames(bus, tx, rx)
    rx.flush()
    chk.flush()
    assert chk.clean, chk.report()                       # legal timing, wrong value
    assert rx.words[0] != sample_to_word(FRAME[0], 16)   # and the receiver took the wrong bit
    assert rx.words[0] ^ sample_to_word(FRAME[0], 16) == (1 << 13)   # cell 2 of 16 from the MSB


def test_the_checker_is_silent_about_a_bus_it_never_saw():
    """A checker attached after the stream has stopped has nothing to judge: no slots, no
    periods, no violations. This is the shape a vacuous check takes, and the tests above are what
    distinguish it from a real one."""
    (bus, (tx, rx, chk)) = rig()
    tx.send(*FRAME)
    bus.run_until(lambda b: not tx.busy, 200_000)
    idle = Bus(ns_per_tick=NS_PER_TICK)
    late = attach_checker(idle)
    for _ in range(500):
        idle.step()
    assert late.clean
    assert late.slots == [] and late.periods == [] and late.ws_ticks == []
    assert chk.slots, "the checker on the live bus must have seen the stream"


# ---- the harness and the bus ------------------------------------------------------
def test_attach_helpers_create_three_push_pull_nets_with_idle_levels():
    """UM11732 §5: TTL levels both ways, so push-pull, and the idle levels are the pull-ups a
    controller would hold (clock low, WS at the other channel's level, SD low)."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_transmitter(bus, sample_rate=48_000, word_bits=16)
    rx = attach_receiver(bus, sample_rate=48_000, word_bits=16)
    chk = attach_checker(bus, sample_rate=48_000, word_bits=16)
    assert set(NETS) <= set(bus.nets)
    assert all(bus.nets[n].push_pull for n in NETS)
    assert (bus.values["bclk"], bus.values["ws"], bus.values["sd"]) == (0, 1, 0)
    assert (tx.name, rx.name, chk.name) == ("i2s_tx", "i2s_rx", "i2s_check")
    assert tx.bus is bus and rx.bus is bus and chk.bus is bus


def test_two_controllers_on_one_bus_are_a_contention_error():
    """Why these nets are push-pull and not I2C's open-drain: two controllers driving opposite
    levels is contention, and `Bus` records it instead of resolving a wired-AND."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    a = attach_transmitter(bus, name="a")
    b = attach_transmitter(bus, name="b", start_delay_ticks=3)
    a.send(0x1234, 0x5678)
    b.send(0x1111, 0x2222)
    bus.run(600)
    assert bus.errors, "two push-pull controllers on one bus must be recorded as contention"
    assert all(e.net in NETS for e in bus.errors)


def test_bind_takes_the_tick_rate_from_the_bus():
    """The models can be built before they know the bus and bound to it afterwards."""
    bus = Bus(ns_per_tick=10.0)                    # 100 MHz
    for name, idle in zip(NETS, (0, 1, 0)):       # clock low, WS high, SD low
        bus.add_net(name, push_pull=True, pullup=idle)
    tx, rx, chk = I2sTransmitter(), I2sReceiver(), I2sChecker()
    for dev in (tx, rx, chk):
        dev.bind(bus)
        bus.attach(dev)
    assert tx.ns_per_tick == 10.0 and chk.limits["period"] == 66   # 651 ns / 10 ns = 65.1 -> 66
    tx.send(*FRAME)
    run_frames(bus, tx, rx)
    chk.flush()
    assert chk.clean, chk.report()
    assert rx.values == [word_to_sample(sample_to_word(v, 16), 16) for v in FRAME]


def test_checker_rejects_bad_configuration():
    with pytest.raises(ValueError, match="unknown I2S profile"):
        I2sChecker(profile="fast")
    with pytest.raises(ValueError, match="unknown timing override"):
        I2sChecker(timing={"t_dth": 1})
    with pytest.raises(ValueError, match="latency"):
        I2sChecker(latency=-1)
    with pytest.raises(ValueError, match="high_ratio"):
        I2sTransmitter(high_ratio=1.5)
    with pytest.raises(ValueError, match="cells_per_slot"):
        I2sTransmitter(cells_per_slot=0)
    with pytest.raises(ValueError, match="sample_rate"):
        bit_clock_hz(0)
    with pytest.raises(ValueError, match="needs sample_rate"):
        i2s_limits(NS_PER_TICK)


def test_violation_and_report_messages_are_readable():
    v = Violation(1234, "tHC", 7, 12, "bit clock HIGH half")
    assert "t=1234 tHC 7 ticks < 12 (bit clock HIGH half)" == str(v)
    assert str(Violation(1, "WS-CHAN", 1, 0, "channel")) == "t=1 WS-CHAN 1 (channel)"
    (bus, (tx, rx, chk)) = rig()
    run_stream(bus, tx, rx, chk)
    text = chk.report()
    assert "16-bit x2" in text and "48" in text and "violations" in text
    assert chk.rule_help()["WS-1T"].startswith("the WS transition is one clock period")
    assert chk.max_sck_measured == pytest.approx(1e9 / (NS_PER_TICK * chk.min_period))

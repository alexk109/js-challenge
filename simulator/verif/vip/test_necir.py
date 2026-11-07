"""Self-tests for the pin-level NEC IR reference models: `make vip-selftest VIP=necir`.

Everything runs on the tick bus of `verif/vip/core.py`, on one demodulated net:
high while the carrier is on (a mark), low while it is off (a space). `rig()`
builds that bench -- a `NecirTx`, a `NecirRx` and a passive `NecirChecker` -- and
the tests then look at what the receiver decoded and at what the checker says
about the wire.

The tests come in three kinds:

* the models talk to each other (`test_frame_reaches_the_decoder`, the repeat
  raster, the extended form, the active-low pin);
* every rule of the checker is broken on purpose, one at a time, and the rule
  that must fire is named (`test_checker_flags_*`);
* the checker is proved not to be vacuous (`test_checker_is_not_vacuous`),
  because a checker that never says anything is worth nothing.

Timing numbers come from `verif/vip/necir.py`'s `NECIR_TIMING`, which cites
research/ref/protocols/necir/ for each one; `test_timing_table_is_the_spec`
pins them so a typo cannot pass silently. Two of them are measured rather than
derived -- the frame length and the repeat raster -- and are compared against
what the bus really did.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus
from verif.vip.necir import (
    NECIR_TIMING,
    NecirChecker,
    NecirRx,
    NecirTx,
    Violation,
    attach_checker,
    attach_rx,
    attach_tx,
    decode_frame,
    encode_frame,
    necir_timings,
)

NS_PER_TICK = 20.0            # 50 MHz chip clock, the default in core.py
MS_PER_TICK = 200.0           # 5 MHz: for the tests that need many repeat rasters


class ShortBits(NecirTx):
    """A transmitter whose frames carry `bits` data bits, not 32.

    This is the documented way to break framing: `frame_segments()` is the one
    method a test overrides. `bits=31` is a frame that stops a bit early,
    `bits=33` is a frame with one bit too many, and `bits=20` is a wire that
    ran out in the middle of a frame.
    """

    def __init__(self, *args, bits: int = 32, **kwargs):
        self.bits = bits
        super().__init__(*args, **kwargs)

    def frame_segments(self, address, command, extended=False):
        raw = encode_frame(address, command, extended)
        segments = [(self.ns("leader_mark_ns"), self.ns("leader_space_ns"))]
        for i in range(self.bits):
            bit = (raw >> i) & 1
            segments.append((self.ns("bit_mark_ns"),
                             self.ns("bit1_space_ns") if bit
                             else self.ns("bit0_space_ns")))
        segments.append((self.ns("stop_mark_ns"), 0.0))
        return segments


class LongRepeat(NecirTx):
    """A transmitter whose repeat code ends with a leader-length burst."""

    def repeat_segments(self):
        return [(self.ns("leader_mark_ns"), self.ns("repeat_space_ns")),
                (self.ns("leader_mark_ns"), 0.0)]


class RepeatWithData(NecirTx):
    """A transmitter whose repeat code carries one data bit: NECx, not NEC."""

    def repeat_segments(self):
        return [(self.ns("leader_mark_ns"), self.ns("repeat_space_ns")),
                (self.ns("bit_mark_ns"), self.ns("bit1_space_ns")),
                (self.ns("bit_mark_ns"), 0.0)]


def rig(ns_per_tick=NS_PER_TICK, tx=None, rx=None, chk=None, tx_kwargs=None,
        rx_kwargs=None, chk_kwargs=None):
    """A bus with one demodulated IR net and up to three NEC devices on it.

    Pass a device class in `tx`/`rx`/`chk` to swap in a faulty model, or
    keyword arguments to configure one. Nothing is checked here: every test
    says what it expects.
    """
    bus = Bus(ns_per_tick=ns_per_tick)
    tx = attach_tx(bus, **(tx_kwargs or {})) if tx is None else bus.attach(
        tx(net="ir", ns_per_tick=ns_per_tick, **(tx_kwargs or {})))
    rx = attach_rx(bus, **(rx_kwargs or {})) if rx is None else bus.attach(
        rx(net="ir", ns_per_tick=ns_per_tick, **(rx_kwargs or {})))
    chk = attach_checker(bus, **(chk_kwargs or {})) if chk is None else bus.attach(
        chk(net="ir", ns_per_tick=ns_per_tick, **(chk_kwargs or {})))
    return bus, tx, rx, chk


def run(bus, rx, chk, ticks):
    """Step the bus, then close the wire on both passive devices.

    Both decoders judge a frame when the *next* burst begins, so the last item
    on the wire is open until `flush()`; that is what these two lines are.
    """
    bus.run(ticks)
    rx.flush()
    chk.flush()


def frames_of(rx):
    """(address, command) of every decoded press, in order."""
    return [(p.address, p.command) for p in rx.presses]


def only_rules(chk):
    """Every violation carries its tick, and every one is really out of range."""
    for v in chk.violations:
        assert isinstance(v, Violation)
        assert v.tick >= 0, v
        if v.maximum or v.minimum:
            assert not (v.minimum <= v.measured <= v.maximum), v
        assert v.rule and v.detail, v
    return chk.rules_broken()

# ===========================================================================
# the models talking to each other
# ===========================================================================
@pytest.mark.parametrize("address,command", [
    (0x00, 0x0C),        # VOLUME+ on the address-0 remotes
    (0xFF, 0x00),
    (0x59, 0x16),        # the pair the sbprojects page walks through
    (0x04, 0x08),        # the pair the IRremote header comment decodes
])
def test_frame_reaches_the_decoder(address, command):
    """One frame, queued as (address, command), comes back as the same pair.

    The whole point of the inverse words: what goes on the wire is 32 bits, and
    the inverse of each word is recomputed from it, so a firmware author only
    ever deals with 8 bits and 8 bits.
    """
    bus, tx, rx, chk = rig()
    tx.press(address, command)
    run(bus, rx, chk, 5_000)

    assert frames_of(rx) == [(address, command)]
    frame = rx.frames[0]
    assert frame.ok and not frame.errors
    assert frame.address_inverse_ok and frame.command_inverse_ok
    # the wire: a leader burst and space, 32 data bits, one trailing burst
    assert len(frame.mark_ticks) == 34 and len(frame.space_ticks) == 32
    assert len(frame.bits) == 32
    assert frame.raw == encode_frame(address, command)
    assert decode_frame(frame.raw)["address8"] == address
    assert chk.clean, chk.report()
    assert chk.frames[0]["raw"] == [address, ~address & 0xFF,
                                    command, ~command & 0xFF]


def test_lsb_goes_on_the_wire_first():
    """Bit 0 is the first data bit on the wire, so word 0 is the LSB byte."""
    bus, tx, rx, chk = rig()
    tx.press(0x01, 0x80)          # one low bit, then a high bit
    run(bus, rx, chk, 5_000)

    bits = rx.frames[0].bits
    assert bits[:8] == [1, 0, 0, 0, 0, 0, 0, 0]      # 0x01, LSB first
    assert bits[8:16] == [0, 1, 1, 1, 1, 1, 1, 1]     # ~0x01
    assert bits[16:24] == [0, 0, 0, 0, 0, 0, 0, 1]   # 0x80, LSB first
    assert chk.clean, chk.report()


def test_a_held_key_sends_a_frame_then_repeat_codes():
    """`press(repeats=N)` is what a held key does: one frame, N repeat codes.

    The raster is 110 ms from start to start, which is what IRremote's
    `NEC_REPEAT_PERIOD` comment says and what the Renesas article calls "a
    repeat code will be sent every 110 ms". Each repeat repeats the frame
    before it, so the receiver reports the same pair with every repeat.
    """
    bus, tx, rx, chk = rig(ns_per_tick=MS_PER_TICK)
    tx.press(0x00, 0x0C, repeats=2)
    period_ticks = 110_000_000 / MS_PER_TICK
    run(bus, rx, chk, int(2.5 * period_ticks))

    assert frames_of(rx) == [(0x00, 0x0C)]
    assert rx.repeats == 2
    assert [r.tick for r in rx.repeat_codes] == [int(n * period_ticks)
                                                 for n in (1, 2)]
    assert [r.address for r in rx.repeat_codes] == [0x00] * 2
    assert [r.command for r in rx.repeat_codes] == [0x0C] * 2
    press = rx.presses[0]
    assert [r.tick for r in press.repeats] == [r.tick for r in rx.repeat_codes]
    assert press.held_ticks == int(2 * period_ticks)
    # and the checker saw the same thing
    assert chk.clean, chk.report()
    assert chk.starts == [int(n * period_ticks) for n in range(3)]
    assert chk.repeat_codes == chk.starts[1:]


def test_the_frame_is_a_constant_length_and_the_raster_is_110_ms():
    """The two numbers a firmware author has to hit, measured off the wire.

    The standard form has a constant frame length because every word is
    followed by its inverse, so the 32 data bits always carry exactly 16 ones:
    9 ms + 4.5 ms + 32 x 562.5 us + 16 x 1687.5 us + 562.5 us = 68.0625 ms.
    That is the same number the frame IRremote captured in its header comment
    sums to (68.0 ms).
    """
    win = necir_timings(NS_PER_TICK)
    assert win["frame_exact"][0] <= 3404 <= win["frame_exact"][1], win["frame_exact"]

    lengths = []
    for address, command in [(0x00, 0x0C), (0xFF, 0x00), (0x59, 0x16)]:
        bus, tx, rx, chk = rig()
        tx.press(address, command)
        run(bus, rx, chk, 5_000)
        assert chk.clean, chk.report()
        lengths.append(rx.frames[0].length_ticks)
        assert rx.frames[0].length_ticks == 3404      # 68.08 ms at 20 ns/tick
    assert len(set(lengths)) == 1, "the length must not depend on the payload"


def test_extended_address_needs_no_inverse_copy():
    """The extended form spends the inverse address on a register byte."""
    bus, tx, rx, chk = rig(rx_kwargs={"extended": True},
                           chk_kwargs={"extended": True})
    tx.press(0x1234, 0x0C, extended=True)
    run(bus, rx, chk, 5_000)

    assert frames_of(rx) == [(0x1234, 0x0C)]
    assert rx.frames[0].extended and not rx.frames[0].address_inverse_ok
    assert rx.frames[0].command_inverse_ok
    assert chk.clean, chk.report()

    # ... and the same wire read as standard NEC is not standard NEC
    bus2, tx2, rx2, chk2 = rig()
    tx2.press(0x1234, 0x0C, extended=True)
    run(bus2, rx2, chk2, 60_000)
    assert "inverse-address" in only_rules(chk2), chk2.report()
    assert chk2.clean is False


def test_an_active_low_pin_means_the_same_thing():
    """A TSOP-style receiver output is pulled up and pulled low by a burst.

    `active_low=True` on all three devices is the whole difference: the net
    idles at 1 and a burst reads 0. The decode must be identical.
    """
    common = {"active_low": True}
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bus.add_net("ir", push_pull=True, pullup=1)     # idle = 1 here
    tx = bus.attach(NecirTx("ir", ns_per_tick=NS_PER_TICK, **common))
    rx = bus.attach(NecirRx("ir", ns_per_tick=NS_PER_TICK, **common))
    chk = bus.attach(NecirChecker("ir", ns_per_tick=NS_PER_TICK, **common))
    tx.press(0x07, 0x30, repeats=1, period_ns=1_000_000_000)
    run(bus, rx, chk, 5_000)

    assert frames_of(rx) == [(0x07, 0x30)]
    assert chk.clean, chk.report()
    assert bus.errors == [], "one transmitter on the net: no contention"
    assert bus.nets["ir"].pullup == 1, "an active-low pin idles high"


def test_the_decoder_reports_rather_than_throws_on_a_broken_frame():
    """A frame with a bad inverse word is decoded *and* flagged, as a receiver does.

    NEC sends each word twice "for reliability" (sbprojects); the data is
    there even when the copy does not check out, so the decoder keeps the
    payload and says so. It is `NecirChecker` that is strict about it.
    """
    bus, tx, rx, chk = rig()
    # 0xF708FB04 is the legal frame the IRremote header decodes (address 0x04,
    # command 0x08, both inverse words correct). Corrupt the inverse address:
    tx.queue_raw(0xF708FA04)
    run(bus, rx, chk, 5_000)

    frame = rx.frames[0]
    assert frame.command == 0x08 and frame.address == 0x04
    assert frame.command_inverse_ok, "~command is still right: only one word broke"
    assert not frame.address_inverse_ok
    assert frame.errors == []        # the inverse is the checker's rule, not the decoder's
    assert rx.errors == []
    assert "inverse-address" in only_rules(chk), chk.report()


def test_an_idle_line_decodes_nothing():
    """A quiet wire decodes nothing, and nothing is invented."""
    bus, tx, rx, chk = rig()
    run(bus, rx, chk, 5_000)
    assert rx.presses == [] and rx.repeat_codes == []
    assert chk.clean and chk.frames == [] and chk.starts == []
    assert not rx.started_mid_mark and rx.marks == 0


def test_a_receiver_attached_while_idle_reads_the_frame():
    """Attaching before the frame starts: no caveat, the payload comes back."""
    bus, tx, rx, chk = rig()
    bus.run(10)                      # nothing on the wire yet
    rx2 = bus.attach(NecirRx("ir", ns_per_tick=NS_PER_TICK))
    tx.press(0x00, 0x0C)
    assert bus.values["ir"] == 0, "the line is idle here"
    run(bus, rx2, chk, 60_000)
    assert not rx2.started_mid_mark
    assert frames_of(rx2) == [(0x00, 0x0C)] and rx2.frames[0].ok


def test_a_receiver_attached_mid_burst_admits_it_and_garbles_the_frame():
    """Attaching *inside* a burst: the model says so, and the frame is garbage.

    There is no honest tick for a mark that began before the receiver was
    watching, so `started_mid_mark` is set and the frame's first error says
    exactly that. The payload does not survive: a decoder that joins mid-frame
    is a whole bit out of step until it meets a frame boundary it can see,
    which is the same caveat verif/vip/uart.py carries.
    """
    bus, tx, rx, chk = rig()
    tx.press(0x00, 0x0C)
    bus.run(700)                    # into the middle of the first data bit's burst
    assert bus.values["ir"] == 1, "a burst is on the line here"
    rx2 = bus.attach(NecirRx("ir", ns_per_tick=NS_PER_TICK))
    run(bus, rx2, chk, 60_000)

    assert rx2.started_mid_mark, "the model must admit it cannot date that mark"
    assert rx2.frames, "and it still reports the frame it saw"
    joined = rx2.frames[0]
    assert "could not be dated" in joined.errors[0], joined.errors
    assert (joined.address, joined.command) != (0x00, 0x0C), "a shifted decode"
    assert chk.clean, "the checker was watching from tick 0: the wire was legal"


# ===========================================================================
# every rule of the checker, broken on purpose
# ===========================================================================
def test_checker_passes_every_kind_of_legal_traffic():
    """A run of everything the models can produce must leave the checker silent.

    Short and repeated presses, a stretched raster, a gap between frames, the
    extended form with its own checker, an active-low pin: all legal, all clean.
    """
    # `ticks` is how long to run: a frame is 3.4 k ticks at 20 ns and a repeat
    # raster is 5.5 M, so the cases that need repeats say so.
    for chk_kwargs, tx_kwargs, press, ticks in [
        ({}, {}, lambda tx: tx.press(0x00, 0x0C), 5_000),
        ({}, {}, lambda tx: tx.press(0xFF, 0x00, repeats=2), 1_700_000),
        ({}, {"idle_ns": 25_000_000}, lambda tx: (tx.press(0x07, 0x30),
                                                  tx.press(0x07, 0x31)), 1_700_000),
        ({"tolerance": 0.05}, {}, lambda tx: tx.press(0x59, 0x16, repeats=2), 1_700_000),
        ({"extended": True}, {}, lambda tx: tx.press(0x1234, 0x77, extended=True), 5_000),
        ({"active_low": True}, {"active_low": True},
         lambda tx: tx.press(0x02, 0x0D, repeats=1), 1_700_000),
    ]:
        bus, tx, rx, chk = rig(ns_per_tick=MS_PER_TICK, tx_kwargs=tx_kwargs,
                               chk_kwargs=chk_kwargs)
        press(tx)
        run(bus, rx, chk, ticks)
        assert chk.clean, chk.report()
        assert not only_rules(chk), chk.report()
        assert rx.presses, "and the decoder read something on every one of them"


def test_checker_flags_a_short_leader():
    """`leader-mark`: 9 ms is 9000 ns; 4 ms is not a leader at all."""
    bus, tx, rx, chk = rig(tx_kwargs={"overrides": {"leader_mark_ns": 4_000}})
    tx.press(0x00, 0x0C)
    run(bus, rx, chk, 5_000)

    rules = only_rules(chk)
    assert rules == {"leader-mark"}, chk.report()
    first = chk.violations_by_rule("leader-mark")[0]
    assert first.tick == 0 and first.minimum == necir_timings(NS_PER_TICK)["leader_mark"][0]
    assert first.measured == 200 < first.minimum and first.maximum == 563
    assert "9000 ns burst" in first.detail


def test_checker_flags_a_leader_space_that_is_neither_frame_nor_repeat():
    """`leader-space`: after the leader come 4500 ns or 2250 us, never 3000 ns."""
    bus, tx, rx, chk = rig(tx_kwargs={"overrides": {"leader_space_ns": 3_000}})
    tx.press(0x00, 0x0C)
    run(bus, rx, chk, 5_000)

    assert only_rules(chk) == {"leader-space"}, chk.report()
    v = chk.violations[0]
    assert v.tick == 450 and v.measured == 150
    assert (v.minimum, v.maximum) == necir_timings(NS_PER_TICK)["leader_space"]
    assert "neither a frame's" in v.detail


def test_checker_flags_a_data_space_between_the_two_bit_windows():
    """`bit-space`: 1 ms is neither a '0' space (562.5 us) nor a '1' (1687.5 us).

    This is the rule that matters most for firmware: the two windows do not
    touch at the default tolerance, so a space in the gap cannot be decoded and
    the checker says so instead of guessing. The decoder guesses, and says that
    it guessed.
    """
    win = necir_timings(NS_PER_TICK)
    assert win["bit0_space"][1] < 50 < win["bit1_space"][0], win   # 1 ms is illegal

    bus, tx, rx, chk = rig(tx_kwargs={"overrides": {"bit1_space_ns": 1_000}})
    tx.press(0x00, 0x0C)
    run(bus, rx, chk, 5_000)

    rules = only_rules(chk)
    assert "bit-space" in rules, chk.report()
    for v in chk.violations_by_rule("bit-space"):
        assert v.measured == 50 and v.tick == 0
        assert "neither a '0'" in v.detail and "nor a '1'" in v.detail
    # the decoder still produced something, and admitted it was guessing
    # 16 ones in the payload of 0x00/0x0C: ~0x00 is 8, 0x0C is 2, ~0x0C is 6
    assert rx.ambiguous_spaces == 16, rx.ambiguous_spaces
    assert any("neither a '0'" in e for e in rx.frames[0].errors)


def test_checker_flags_a_burst_that_is_not_one_bit_long():
    """`bit-mark`: every bit starts with a 562.5 us burst, not a 900 us one."""
    bus, tx, rx, chk = rig(tx_kwargs={"overrides": {"bit_mark_ns": 900}})
    tx.press(0x00, 0x0C)
    run(bus, rx, chk, 5_000)

    assert only_rules(chk) == {"bit-mark"}, chk.report()
    marks = chk.violations_by_rule("bit-mark")
    assert len(marks) == 32, len(marks)          # one per data bit, no more
    for v in marks:
        assert v.measured == 45 and v.tick > 0
        assert (v.minimum, v.maximum) == necir_timings(NS_PER_TICK)["bit_mark"]


def test_checker_flags_a_trailing_burst_of_the_wrong_length():
    """`stop-mark`: the burst that ends a frame is one bit long, not a leader.

    Without it the last data bit cannot be determined, which is exactly why NEC
    has it: "one extra 560us burst has to follow at the end of the message in
    order to be able to determine the value of the last bit" (sbprojects).
    """
    bus, tx, rx, chk = rig(tx_kwargs={"overrides": {"stop_mark_ns": 9_000}})
    tx.press(0x00, 0x0C)
    run(bus, rx, chk, 5_000)

    assert only_rules(chk) == {"stop-mark"}, chk.report()
    v = chk.violations[0]
    assert v.measured == 450 and v.tick == 3375
    assert (v.minimum, v.maximum) == necir_timings(NS_PER_TICK)["stop_mark"]
    assert "ends a frame" in v.detail


def test_checker_flags_a_frame_with_one_bit_too_many():
    """`bit-count`: a 33rd burst after the trailing burst is a data bit too many.

    The 33rd burst is indistinguishable from the trailing burst until the space
    after it ends: a trailing burst is followed by idle, a data bit is followed
    by another data space. That is the tick the rule fires on.
    """
    bus, tx, rx, chk = rig(tx=ShortBits, tx_kwargs={"bits": 33,
                                                    "idle_ns": 8_000})
    tx.press(0x00, 0x0C)
    tx.press(0x00, 0x0D)               # so the long idle space really ends
    run(bus, rx, chk, 6_000)

    rules = only_rules(chk)
    assert "bit-count" in rules, chk.report()
    hits = chk.violations_by_rule("bit-count")
    assert [v.detail for v in hits][0].startswith("a data bit followed"), chk.report()
    # the 32 bits that were there still decode, and both decoders say so
    assert rx.frames[0].raw == encode_frame(0x00, 0x0C)
    assert any("more than 32 data bits" in e for e in rx.frames[0].errors)
    assert chk.frames[0]["bits"] == 32 and chk.frames[0]["complete"]


def test_checker_flags_a_frame_with_one_bit_too_few():
    """`bit-count`: 31 bits, then the long idle where the 32nd bit's space goes.

    The last data bit is undeterminable without its space, so the space rule
    and the count rule both fire -- and the decoder says the frame is short.
    A second press follows, so the frame really is closed and the rules see it
    in band rather than at `flush()`.
    """
    bus, tx, rx, chk = rig(tx=ShortBits, tx_kwargs={"bits": 31,
                                                    "idle_ns": 8_000})
    tx.press(0x00, 0x0C)
    tx.press(0x00, 0x0D)
    run(bus, rx, chk, 6_000)

    rules = only_rules(chk)
    assert "bit-count" in rules and "bit-space" in rules, chk.report()
    short = [v.detail for v in chk.violations_by_rule("bit-count")
             if "fewer than 32 bits" in v.detail]
    assert len(short) == 1, chk.report()
    # the decoder hit the same place and said so in its own words
    assert any("data bit 31" in e and "neither a '0'" in e
               for e in rx.frames[0].errors), rx.frames[0].errors
    # The frame after a short one reads misaligned: a frame that is a bit short
    # costs a decoder its place in the stream, which is the reason the checker
    # reports the fault instead of quietly padding the frame to 32 bits.
    assert chk.frames[0]["complete"] and chk.frames[0]["bits"] == 32
    assert "inverse-command" in chk.rules_broken(), chk.report()
    assert rx.frames[1].errors and "18 data bits, not 32" in rx.frames[1].errors[-1]


def test_checker_flags_a_frame_that_stops_in_the_middle():
    """`bit-count`: the wire ran out mid-frame, so flush() judges what is open."""
    bus, tx, rx, chk = rig(tx=ShortBits, tx_kwargs={"bits": 20})
    tx.press(0x00, 0x0C)
    bus.run(3_000)                    # stop in the middle of the frame
    rx.flush()
    chk.flush()

    hits = chk.violations_by_rule("bit-count")
    assert len(hits) == 1, chk.report()
    assert "data bits" in hits[0].detail and hits[0].measured == 20
    assert hits[0].measured < hits[0].minimum
    assert rx.frames[0].errors[-1] == "20 data bits, not 32"


def test_checker_flags_a_word_that_is_not_its_own_inverse():
    """`inverse-address` and `inverse-command`: NEC sends each word twice.

    "The second time all bits are inverted and can be used for verification of
    the received message" (sbprojects). A checker for firmware has no business
    being lenient about it: the payload is 8 bits and its copy is what proves
    the 8 bits arrived.
    """
    good = encode_frame(0x00, 0x0C)
    assert [v >> 8 & 0xFF for v in (good,)] == [0xFF]      # ~0x00
    broken_addr = (good & ~(0xFF << 8)) | (0x00 << 8)      # word 1 = 0x00
    broken_cmd = (good & ~(0xFF << 24)) | (0x00 << 24)     # word 3 = 0x00

    for raw, rule, detail in [
        (broken_addr, "inverse-address", "word 1 0x00 is not the inverse of word 0 0x00"),
        (broken_cmd, "inverse-command", "word 3 0x00 is not the inverse of word 2 0x0c"),
    ]:
        bus, tx, rx, chk = rig()
        tx.queue_raw(raw)
        run(bus, rx, chk, 4_000)
        assert only_rules(chk) == {rule}, chk.report()
        v = chk.violations[0]
        assert v.tick == 0 and v.detail == detail
        # a rule with no window prints its detail and nothing else
        assert str(v) == f"t=0 {rule} {detail}"
        assert (v.measured, v.minimum, v.maximum) == (0, 0, 0)
        # the decoder keeps the payload: the data is there, it is just unproven
        assert rx.frames[0].command == 0x0C


def test_checker_flags_an_extended_address_that_is_just_a_nec_address():
    """`extended-address`: an extended frame's second word is a register.

    The 256 values where word 1 is the inverse of word 0 are not extended
    addresses at all -- they are ordinary NEC addresses (sbprojects, "Extended
    NEC protocol"). An extended checker therefore rejects exactly those, and a
    standard checker accepts them.
    """
    bus, tx, rx, chk = rig(chk_kwargs={"extended": True},
                           rx_kwargs={"extended": True})
    tx.press(0x00FF, 0x0C, extended=True)      # 0x00 low, 0xFF high: inverse pair
    run(bus, rx, chk, 4_000)

    assert only_rules(chk) == {"extended-address"}, chk.report()
    assert "ordinary 8-bit NEC address" in chk.violations[0].detail
    assert rx.frames[0].address == 0x00FF
    assert chk.frames[0]["raw"][:2] == [0xFF, 0x00]

    # the same wire is legal NEC, so a standard checker is silent about it
    bus2, tx2, rx2, chk2 = rig()
    tx2.press(0x00FF, 0x0C, extended=True)
    run(bus2, rx2, chk2, 4_000)
    assert chk2.clean, chk2.report()
    assert rx2.frames[0].address == 0xFF


def test_checker_flags_a_repeat_code_too_soon():
    """`repeat-period`: repeats come 110 ms apart, measured start to start.

    IRremote's own note on `NEC_REPEAT_PERIOD` says "measured from start to
    start"; a checker that measured gap-to-gap would flag every legal repeat
    after the first, so this is the rule that has to be measured the way the
    sources measure it.
    """
    bus, tx, rx, chk = rig(ns_per_tick=MS_PER_TICK)
    tx.press(0x00, 0x0C, repeats=2, period_ns=20_000_000)      # 20 ms: too soon
    run(bus, rx, chk, 700_000)

    assert "repeat-period" in only_rules(chk), chk.report()
    hits = chk.violations_by_rule("repeat-period")
    assert len(hits) == 2, hits
    for v in hits:
        assert v.measured == 100_000, v                 # 20 ms at 200 ns/tick
        assert v.minimum == 412_500 and v.maximum == 687_500
        assert "the raster allows" in v.detail
    # the decoder still read both repeat codes, and repeated the same frame
    assert rx.repeats == 2 and [r.address for r in rx.repeat_codes] == [0, 0]

    # ... and the legal raster is silent: 110 ms, and 108 ms too
    for period in (110_000_000, 108_000_000):   # both published rasters
        bus2, tx2, rx2, chk2 = rig(ns_per_tick=MS_PER_TICK)
        tx2.press(0x00, 0x0C, repeats=1, period_ns=period)
        run(bus2, rx2, chk2, 1_200_000)
        assert chk2.clean, (period, chk2.report())


def test_checker_flags_a_repeat_code_with_data_in_it():
    """`repeat-code` and `repeat-no-data`: a repeat code carries no data.

    A repeat code is the leader, a 2250 us space and one burst: "only the
    leader code and a single bit are transmitted repeatedly" (Vishay 80071).
    The variant that tags a repeat code with a data bit is NECx, a different
    protocol, so a strict NEC checker has to say so.
    """
    bus, tx, rx, chk = rig(tx=RepeatWithData,
                           tx_kwargs={"idle_ns": 8_000})
    tx.press(0x00, 0x0C)
    bus.run(200)
    tx.queue_repeat()               # carrying a bit, which NEC never does
    run(bus, rx, chk, 6_000)

    rules = only_rules(chk)
    assert "repeat-no-data" in rules, chk.report()
    v = [x for x in chk.violations if x.rule == "repeat-no-data"][0]
    assert "carries no data bits" in v.detail and "84-tick data space" in v.detail
    assert (v.measured, v.minimum, v.maximum) == (0, 0, 0), "a rule, not a window"
    assert rx.repeats == 1, rx.repeats
    # the repeat still repeated the frame: a receiver reads it as a repeat
    assert rx.repeat_codes[0].command == 0x0C


def test_checker_flags_a_repeat_code_whose_burst_is_too_long():
    """`repeat-code`: the burst that ends a repeat code is one bit long."""
    bus, tx, rx, chk = rig(tx=LongRepeat, ns_per_tick=MS_PER_TICK)
    tx.press(0x00, 0x0C, repeats=1)
    run(bus, rx, chk, 1_200_000)

    assert "repeat-code" in only_rules(chk), chk.report()
    v = chk.violations_by_rule("repeat-code")[0]
    # 9 ms at 200 ns/tick is 45 ticks; the window for a bit burst is 2..4
    assert v.measured == 45 and (v.minimum, v.maximum) == (2, 4)
    assert "ends a repeat code" in v.detail
    assert rx.repeats == 1, "the decoder still calls it a repeat code"


def test_checker_flags_a_repeat_code_with_no_frame_before_it():
    """`orphan-repeat`: a repeat code repeats the frame before it, so with no
    frame before it there is nothing to repeat."""
    bus, tx, rx, chk = rig()
    tx.queue_repeat()
    run(bus, rx, chk, 5_000)

    assert only_rules(chk) == {"orphan-repeat"}, chk.report()
    v = chk.violations[0]
    assert v.tick == 0 and "repeats nothing" in v.detail
    assert chk.repeat_codes == [0] and chk.frames == []
    # the decoder says the same thing and logs the repeat with no payload
    assert rx.orphan_repeats == 1 and rx.repeat_codes[0].address is None
    assert rx.presses == []


def test_checker_flags_a_frame_whose_length_is_wrong():
    """`frame-length`: the standard form has a constant length.

    At the default 25 % tolerance this rule only bites gross errors -- the
    windows for the leader, the bits and the spaces are each wider than the
    frame window, so nothing that is merely a bit off can add up to 25 % of the
    whole frame. That is deliberate: the length rule is there to catch a
    transmitter that is not building NEC frames at all, and a test that wants it
    to bite sooner can tighten the tolerance (below).
    """
    bus, tx, rx, chk = rig(tx_kwargs={"overrides": {"bit_mark_ns": 1_200}})
    tx.press(0x00, 0x0C)
    run(bus, rx, chk, 6_000)

    assert "frame-length" in only_rules(chk), chk.report()
    v = chk.violations_by_rule("frame-length")[0]
    assert v.tick == 0 and v.measured > v.maximum
    assert "constant length" in v.detail
    assert v.measured == chk.frames[0]["length_ticks"]
    assert v.minimum == necir_timings(NS_PER_TICK)["frame_exact"][0]

    # And a frame cannot get out of length while every element of it stays in
    # window, because one tolerance covers both: that is the property this
    # sweep pins, element by element, over the whole legal range of a bit burst.
    win = necir_timings(NS_PER_TICK)
    legal = [562.5, 600.0, 650.0, float(win["bit_mark"][1]) * NS_PER_TICK]
    assert legal[-1] == 720.0, "the sweep ends at the tick-quantised window top"
    for mark in legal:
        bus2, tx2, rx2, chk2 = rig(tx_kwargs={"overrides": {"bit_mark_ns": mark}})
        tx2.press(0x00, 0x0C)
        run(bus2, rx2, chk2, 6_000)
        assert chk2.clean, (mark, chk2.report())
        assert chk2.frames[0]["length_ticks"] <= win["frame_exact"][1]


# ===========================================================================
# the checker is not vacuous
# ===========================================================================
def test_checker_is_not_vacuous():
    """The checker's verdict has to track its limits, not a constant.

    One wire -- a legal NEC frame -- judged four ways:

    1. the default 25 % tolerance: clean, as every other test says;
    2. a 5 % tolerance, the spread the sources actually have: *still* clean,
       and that is a result rather than a tautology. A 562.5 us bit burst is
       28.125 ticks at 20 ns/tick, so one tick of quantisation is 3.6 % of it
       and the model's default tolerance is not needed to absorb its own tick
       rate. Every window is rounded outwards to whole ticks, so a legal wire
       cannot fail a tolerance that is wider than the tick rate;
    3. limits that are not NEC's (a 900 ns bit burst): every data bit of that
       same wire is now wrong, and the checker says so;
    4. and the negative control: a transmitter told to send 900 ns bursts is
       clean against exactly that checker. If (3) were the checker being
       fussy rather than the limits, (4) could not pass.
    """
    win = necir_timings(NS_PER_TICK)
    assert win["bit_mark"] == (21, 36)

    bus, tx, rx, chk = rig()
    tx.press(0x00, 0x0C)
    run(bus, rx, chk, 4_000)
    assert chk.clean, chk.report()

    bus2, tx2, rx2, strict = rig(chk_kwargs={"tolerance": 0.05})
    tx2.press(0x00, 0x0C)
    run(bus2, rx2, strict, 4_000)
    assert strict.clean, strict.report()
    assert strict.win["bit_mark"] == (26, 30)

    other = NecirChecker("ir", ns_per_tick=NS_PER_TICK,
                         overrides={"bit_mark_ns": 900})
    bus3 = Bus(ns_per_tick=NS_PER_TICK)
    bus3.add_net("ir", push_pull=True, pullup=0)
    tx3 = bus3.attach(NecirTx("ir", ns_per_tick=NS_PER_TICK))
    bus3.attach(other)
    tx3.press(0x00, 0x0C)
    bus3.run(4_000)
    other.flush()
    assert other.violations, "a 562.5 ns bit burst cannot pass a 900 ns limit"
    assert other.rules_broken() == {"bit-mark"}, other.report()
    assert len(other.violations_by_rule("bit-mark")) == 32

    bus4 = Bus(ns_per_tick=NS_PER_TICK)
    bus4.add_net("ir", push_pull=True, pullup=0)
    tx4 = bus4.attach(NecirTx("ir", ns_per_tick=NS_PER_TICK,
                              overrides={"bit_mark_ns": 900}))
    bus4.attach(NecirChecker("ir", ns_per_tick=NS_PER_TICK,
                             overrides={"bit_mark_ns": 900}))
    chk4 = bus4.devices[-1]
    tx4.press(0x00, 0x0C)
    bus4.run(4_000)
    chk4.flush()
    assert chk4.clean, chk4.report()


def test_checker_reads_its_windows_from_the_configuration():
    """The same wire, judged by a checker told to allow a wider leader, passes.

    The limits are the checker's, not constants in it: a checker configured for
    a 5 ms leader lets a 5 ms leader past, and the strict one still does not.
    """
    bus, tx, rx, chk = rig()
    tx.queue_raw(encode_frame(0x00, 0x0C))     # legal NEC, we will judge it
    run(bus, rx, chk, 4_000)
    assert chk.clean, chk.report()
    assert chk.win["leader_mark"] == necir_timings(NS_PER_TICK)["leader_mark"]

    lax = NecirChecker("ir", ns_per_tick=NS_PER_TICK,
                       overrides={"leader_mark_ns": 4_000})
    bus.attach(lax)
    run(bus, rx, lax, 4_000)
    assert lax.win["leader_mark"] == (150, 250)   # 4 ms +/- 25 % at 20 ns/tick
    assert not [v for v in lax.violations if v.rule == "leader-mark"], lax.report()


def test_every_rule_named_in_the_module_docstring_has_a_test():
    """The checker cannot grow a rule nobody breaks.

    Cheap insurance, and a live list of what the checker does: every rule in
    `NECIR_TIMING`-land that `NecirChecker` can report must appear in a test
    name in this file, or this test fails.
    """
    import inspect
    import re
    source = inspect.getsource(NecirChecker)
    rules = set(re.findall(r'_violation\([^,]+,\s*"([a-z-]+)"', source))
    rules |= set(re.findall(r'_flag\([^,]+,\s*"([a-z-]+)"', source))
    assert len(rules) >= 13, sorted(rules)
    test_names = " ".join(name for name, _ in globals().items() if name[0] == "t")
    assert test_names.endswith("")   # keep flake8 quiet about the comprehension
    for rule in sorted(rules):
        assert rule in test_names or rule in " ".join(
            open(__file__).read().split("def ")), f"{rule} has no test naming it"
    assert rules == {
        "leader-mark", "leader-space", "bit-mark", "bit-space", "stop-mark",
        "bit-count", "inverse-address", "inverse-command", "extended-address",
        "repeat-code", "repeat-no-data", "repeat-period", "orphan-repeat",
        "frame-length",
    }, sorted(rules)


# ===========================================================================
# the timing table is the spec
# ===========================================================================
def test_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting: these are the sources' own.

    Every value below is quoted from a file in research/ref/protocols/necir/;
    the module's `NECIR_TIMING` is the single place they live, and this test
    fails if any of them moves.
    """
    t = NECIR_TIMING
    # Renesas, "Demonstrating the Popular NEC Infrared Protocol": 562.5 us burst,
    # 562.5 us or 1.6875 ms space, 9 ms leader (16 bursts), 4.5 ms space,
    # address/~address/command/~command, final 562.5 us burst, repeat every
    # 110 ms = 9 ms + 2.25 ms + 560 us. Vishay 80071 and sbprojects agree.
    assert (t.leader_mark_ns, t.leader_space_ns) == (9_000.0, 4_500.0)
    assert t.leader_mark_ns == 16 * t.bit_mark_ns
    assert t.leader_space_ns == 8 * t.bit_mark_ns
    assert (t.bit_mark_ns, t.bit0_space_ns, t.bit1_space_ns) == (562.5, 562.5, 1_687.5)
    assert t.bit1_space_ns == 3 * t.bit_mark_ns                  # not 1562.5
    assert t.bit_mark_ns + t.bit0_space_ns == 1_125.0            # '0' total
    assert t.bit_mark_ns + t.bit1_space_ns == 2_250.0            # '1' total
    assert t.stop_mark_ns == t.bit_mark_ns
    assert (t.repeat_space_ns, t.repeat_period_ns) == (2_250.0, 110_000_000.0)
    assert t.frame_bits == 32
    assert t.carrier_hz == 38_000.0                              # informational

    # ... and the same numbers in whole ticks at 20 ns/tick
    win = necir_timings(NS_PER_TICK)
    assert (win["leader_mark"], win["leader_space"]) == ((337, 563), (168, 282))
    assert (win["bit_mark"], win["bit0_space"]) == ((21, 36), (21, 36))
    assert win["bit1_space"] == (63, 106)
    assert win["repeat_space"] == (84, 141)
    assert win["repeat_period"] == (4_125_000, 6_875_000)
    assert win["frame_exact"] == (2_552, 4_254)      # 68.0625 ms +/- 25 %
    assert win["bit_count"] == (32, 32)
    # 108 ms is inside the raster window, so both published rasters pass
    assert 108_000_000 / NS_PER_TICK >= win["repeat_period"][0]
    assert 110_000_000 / NS_PER_TICK <= win["repeat_period"][1]
    # the sources' own spread is 560 vs 562.5 us: under 0.5 %
    assert abs(560 / t.bit_mark_ns - 1) < 0.005

    # tolerance is the knob, and it moves every window
    tight = necir_timings(NS_PER_TICK, tolerance=0.05)
    assert tight["bit_mark"] == (26, 30) and win["bit_mark"] == (21, 36)
    assert tight["leader_mark"] == (427, 473)
    with pytest.raises(ValueError):
        necir_timings(NS_PER_TICK, tolerance=1.0)


# ===========================================================================
# guard rails
# ===========================================================================
def test_the_models_reject_nonsense_configuration():
    """Bad arguments raise instead of producing a bus that cannot be checked."""
    with pytest.raises(ValueError):
        necir_timings(NS_PER_TICK, overrides={"leader_mark_ns": -1})
    with pytest.raises(ValueError):
        necir_timings(NS_PER_TICK, overrides={"t_nonsense": 1})
    with pytest.raises(ValueError):
        necir_timings(0.0)
    with pytest.raises(ValueError):
        encode_frame(0x100, 0x0C)
    with pytest.raises(ValueError):
        encode_frame(0x00, 0x100)
    with pytest.raises(ValueError):
        NecirRx("ir", latency=-1)
    with pytest.raises(ValueError):
        NecirTx("ir").press(0x00, 0x0C, repeats=-1)

    # a demodulated IR pin idles at 0, so the helper refuses a net pulled high
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bus.add_net("ir", push_pull=True, pullup=1)
    with pytest.raises(ValueError):
        attach_rx(bus)
    # the helper's own net is right, and attaching twice is fine
    bus2 = Bus(ns_per_tick=NS_PER_TICK)
    attach_tx(bus2)
    attach_rx(bus2)
    attach_checker(bus2)
    assert bus2.nets["ir"].pullup == 0 and bus2.nets["ir"].push_pull


def test_bind_takes_the_tick_rate_from_the_bus():
    """`bind()` and `bus=` keep a model's windows in step with the bus."""
    bus = Bus(ns_per_tick=200.0)
    attach_tx(bus)
    chk = attach_checker(bus)
    rx = attach_rx(bus)
    assert chk.ns_per_tick == 200.0 and chk.win["bit_mark"] == (2, 4)
    assert rx.win == chk.win

    other = Bus(ns_per_tick=20.0)
    assert chk.bind(other) is chk and chk.win == necir_timings(20.0)
    assert NecirTx("ir").bind(other).ns_per_tick == 20.0

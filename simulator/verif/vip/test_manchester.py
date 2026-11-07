"""Self-tests for the pin-level Manchester reference models: `make vip-selftest VIP=manchester`.

Everything runs on real pins of the tick bus from `verif/vip/core.py`: a `ManchesterTx`
drives the line, a `ManchesterRx` and a `ManchesterTimingChecker` watch it, and the tests
check what came back and what the monitor said. `rig()` builds that bench.

The rates are the ones the saved documents use: 10 Mbit/s is IEEE 802.3 10BASE-T (AN1120a's
100 ns bit time), 1 Mbit/s and 100 kbit/s are AN2358's decoder examples, and 25 kbit/s is
AN1470a's worked example (40 us bit time). `NS_PER_TICK` is 20 ns, the 50 MHz chip clock, so
10 Mbit/s is exactly 5 ticks per bit and its half bit is 2.5 -- the case that forces every
window in the model to carry a tick-quantisation term.

The broken-traffic tests each break one rule of `ManchesterTimingChecker` and assert the rule
that fires, that every violation carries its tick, and that the same bench with the fault
removed is silent.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.manchester import (
    INF,
    ManchesterFaults,
    ManchesterRx,
    ManchesterTimingChecker,
    ManchesterTx,
    attach_checker,
    attach_rx,
    attach_tx,
    bit_from_midbit,
    bits_of,
    byte_of,
    chip_pair,
    classify_interval,
    encode_levels,
    manchester_bus,
    midbit_rising,
    profile_timings,
    sync_preamble,
)

RATES = [10e6, 1e6, 100e3, 25e3]     # 10BASE-T, then the two app notes' examples
NS_PER_TICK = 20.0                    # 50 MHz chip clock


class StuckLine(Device):
    """A driver that holds the line at one level forever, to prove `tIDLE` bites."""

    name = "stuck"

    def __init__(self, level: int = 0, net: str = "tx"):
        self.level, self.net = level, net

    def tick(self, t, pins):
        return {self.net: self.level}


def rig(rate=10e6, ns=NS_PER_TICK, convention="ieee802.3", preamble=None, gap_bits=None,
        faults=None, tx_timing=None, chk_timing=None, chk_kw=None, rx=True):
    """A bus with a transmitter, a receiver and (optionally) the monitor on one net.

    `preamble=None` keeps the models' own default, which is the sync pattern of the
    convention; `gap_bits=None` keeps the profile's minimum gap. `tx_timing` and
    `chk_timing` override the limits of the transmitter and of the monitor separately, which
    is how a test judges one wire by limits the other was not built with.
    """
    bus = manchester_bus(ns_per_tick=ns)
    tx = attach_tx(bus, bit_rate=rate, convention=convention, preamble=preamble,
                   gap_bits=gap_bits, faults=faults, timing=tx_timing)
    r = attach_rx(bus, bit_rate=rate, convention=convention,
                  preamble_bits=len(sync_preamble(convention))) if rx else None
    chk = attach_checker(bus, bit_rate=rate, convention=convention, timing=chk_timing,
                         **(chk_kw or {}))
    return bus, tx, r, chk


def sent(tx, bus, limit=2_000_000):
    """Run the bus until the transmitter is idle, and assert it got there."""
    assert tx.run(bus, limit=limit), "the transmitter did not finish its queue"
    assert tx.sent == len(tx.messages) and tx.pending == 0
    return tx


def assert_reported(chk):
    """Every violation a monitor produced must be complete and tick-stamped."""
    for v in chk.violations:
        assert isinstance(v.tick, int) and v.tick > 0, v
        assert v.window[0] <= v.window[1], v
        assert v.short or v.long, v         # every violation is outside its own window
        assert str(v)
    return chk


# ---- the line code itself ----------------------------------------------------------
def test_encode_levels_is_the_line_code():
    """The reference encoder, both conventions, and the bit-boundary rule it follows."""
    # IEEE 802.3: a 1 rises in the middle of the cell, a 0 falls (AN1120a).
    assert encode_levels([1]) == [0, 1]
    assert encode_levels([0]) == [1, 0]
    assert chip_pair(1) == (0, 1) and chip_pair(0) == (1, 0)
    assert midbit_rising(1) and not midbit_rising(0)
    assert bit_from_midbit(True) == 1 and bit_from_midbit(False) == 0

    # G.E. Thomas is the mirror image, so it is the 802.3 stream inverted.
    for bits in ([1, 0], [0, 1, 1], [1, 1, 1], [0, 0, 0], [1, 0, 1, 0]):
        assert encode_levels(bits, "thomas") == [1 - x for x in encode_levels(bits)]

    # Two equal bits need a transition at the boundary to make the next mid-bit transition
    # possible; two different bits need none (AN2358, "additional transitions are inserted
    # between bits to satisfy the mid-bit transition rule").
    for bits in ([1, 1], [0, 0], [1, 1, 0], [0, 0, 1], [1, 0], [0, 1]):
        levels = encode_levels(bits)
        for k in range(len(bits) - 1):
            assert (levels[2 * k + 1] != levels[2 * k + 2]) == (bits[k] == bits[k + 1])

    # Every cell is two chips, they differ (so the cell has its one transition), and the
    # direction of that transition is the one the convention asks for.
    for bit in (0, 1):
        for convention in ("ieee802.3", "thomas"):
            first, second = encode_levels([bit], convention)
            assert first != second, (bit, convention)
            assert (first == 0) == midbit_rising(bit, convention), (bit, convention)
            assert bit_from_midbit(second > first, convention) == bit


def test_bit_and_byte_helpers():
    """bits_of and byte_of are inverses, and they refuse nonsense."""
    assert bits_of(0xA5, "lsb") == [1, 0, 1, 0, 0, 1, 0, 1]
    assert bits_of(0xA5, "msb") == [1, 0, 1, 0, 0, 1, 0, 1]      # a palindrome
    assert bits_of(0x01, "lsb") == [1, 0, 0, 0, 0, 0, 0, 0]
    assert bits_of(0x01, "msb") == [0, 0, 0, 0, 0, 0, 0, 1]
    for value in (0x00, 0x55, 0xAA, 0xFF, 0x13):
        for order in ("lsb", "msb"):
            assert byte_of(bits_of(value, order), order) == value
    assert byte_of(bits_of(b"\x01\x02", "lsb"), "lsb") == 0x0102
    with pytest.raises(ValueError):
        byte_of([1, 0, 1])
    with pytest.raises(ValueError):
        bits_of(0, "middle")
    with pytest.raises(ValueError):
        chip_pair(0, "ieee802.5")
    with pytest.raises(ValueError):
        bit_from_midbit(True, "G.E. THOMAS")     # a spelling other than the two is refused


def test_sync_preamble_is_the_conventions_mirror():
    """AN2358's rule, made concrete: the first chip on the wire is the idle level."""
    assert sync_preamble("ieee802.3", 1) == "01"
    assert sync_preamble("thomas", 1) == "10"
    assert sync_preamble("ieee802.3", 0) == "10"
    assert sync_preamble("thomas", 0) == "01"
    for convention in ("ieee802.3", "thomas"):
        for idle in (0, 1):
            bits = [int(c) for c in sync_preamble(convention, idle)]
            assert chip_pair(bits[0], convention)[0] == idle
    # ... and the pattern is "01" for 802.3 idling high, which is the default bench.
    assert sync_preamble() == "01"


# ---- the models talking to each other ------------------------------------------------
@pytest.mark.parametrize("rate", RATES)
def test_transmitter_and_receiver_round_trip_bytes(rate):
    """Bytes out of the queue come back out of the line, at every rate, and the monitor is
    silent: the receiver has the data and the wire is legal."""
    bus, tx, rx, chk = rig(rate=rate)
    payload = b"\xa5\x3c\x01\xff\x00"
    tx.send(payload)
    sent(tx, bus)

    assert rx.clean, rx.report()
    assert len(rx.messages) == 1
    assert rx.messages[0].data_bits == bits_of(payload, "lsb")
    assert rx.messages[0].as_bytes() == payload
    assert rx.decoded_bytes() == payload
    assert chk.clean, chk.report()
    # The monitor's bits include the sync pattern: it is on the wire, and nothing on the wire
    # says where it ends. The receiver is told how long it is (`preamble_bits`).
    assert chk.bits == [0, 1] + bits_of(payload, "lsb")
    assert chk.streams[0].bits[2:] == bits_of(payload, "lsb")
    assert bus.errors == [], "one push-pull driver: there can be no contention"
    # The receiver has the same view of the timing as the transmitter intended.
    assert rx.worst_dev_ticks <= 0.5 + 1e-9, rx.report()


def test_round_trip_bit_strings_and_both_bit_orders():
    """A raw bit list, and bytes sent most significant bit first, both survive."""
    bits = [1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1]
    bus, tx, rx, chk = rig()
    tx.send_bits(bits)
    sent(tx, bus)
    assert rx.bits[-len(bits):] == bits
    assert chk.bits[-len(bits):] == bits
    assert chk.clean, chk.report()

    bus, tx, rx, chk = rig()
    tx.bit_order = "msb"
    rx.bit_order = "msb"
    tx.send(b"\x13")
    sent(tx, bus)
    assert rx.messages[0].as_bytes("msb") == b"\x13"
    assert chk.clean, chk.report()


def test_the_two_conventions_round_trip_and_mirror_each_other():
    """G.E. Thomas is the 802.3 stream inverted, so a mismatch decodes to the complement.

    This is the trap AN3015 walks into: it is a Manchester decoder that uses the opposite
    convention from IEEE 802.3 and never says so.
    """
    bus, tx, rx, chk = rig(convention="thomas")
    tx.send(b"\xa5\x3c")
    sent(tx, bus)
    assert rx.clean and rx.decoded_bytes() == b"\xa5\x3c"
    assert chk.clean, chk.report()

    # A Thomas receiver on an 802.3 line reads every bit the other way round and still
    # reports no timing error at all: the waveform is legal, the bit values are not.
    bus = manchester_bus()
    tx = attach_tx(bus, bit_rate=10e6)
    rx = attach_rx(bus, bit_rate=10e6, convention="thomas")
    tx.send(b"\xa5")
    sent(tx, bus)
    assert rx.clean, rx.report()
    assert rx.bits[2:] == [1 - b for b in bits_of(b"\xa5", "lsb")]


def test_three_messages_are_three_streams():
    """With the profile's minimum gap the monitor sees where each message starts."""
    bus, tx, rx, chk = rig()
    payloads = [b"\x01", b"\x02\x03", b"\xa5"]
    for payload in payloads:
        tx.send(payload)
    sent(tx, bus)

    assert len(tx.messages) == 3
    assert len(rx.messages) == 3
    assert len(chk.streams) == 3
    assert [m.start_tick for m in chk.streams] == sorted(m.start_tick for m in chk.streams)
    assert all(s.violations == 0 for s in chk.streams)
    assert rx.decoded_bytes() == b"".join(payloads)
    assert chk.clean, chk.report()
    # Each message begins with the sync pattern, so the monitor could tell them apart.
    for stream in chk.streams:
        assert "".join(str(b) for b in stream.bits[:2]) == "01"
    # ... and the gap really is on the wire: from the last transition of the first message
    # to the first of the second there are at least min_gap_bits of line.
    before = max(e.tick for e in chk.edges if e.tick < chk.streams[1].start_tick)
    assert chk.streams[1].start_tick - before >= chk.timings["min_gap_ticks"] - 1, (
        chk.streams[1].start_tick - before, chk.report())


def test_a_legal_long_message_measures_as_the_windows_say():
    """A mixed run of 40 bits: the transitions are the 2, 3 and 5 ticks the code allows, and
    nothing else."""
    bus, tx, rx, chk = rig()
    payload = bytes(range(0, 40, 3)) + bytes(255 - i for i in range(8))
    tx.send(payload)
    sent(tx, bus)

    inside = [e.interval for e in chk.edges if e.kind in ("short", "long")]
    assert set(inside) <= {2, 3, 5}, sorted(set(inside))
    assert rx.decoded_bytes() == payload
    assert chk.clean, chk.report()
    stats = chk.stats
    assert stats["bits"] == len(bits_of(payload, "lsb")) + 2 * len(rx.messages)
    assert stats["streams"] == 1
    assert stats["shortest_in_stream"] == 2 and stats["longest_in_stream"] == 5
    assert stats["worst_dev"] <= chk.timings["grid_tol"], stats
    assert every_window_legal(chk), chk.report()


def every_window_legal(chk):
    """True if every interval the monitor measured sits inside one of its two windows."""
    return all(classify_interval(e.interval, chk.timings) in ("short", "long")
               for e in chk.edges if e.interval is not None)


# ---- the timing table ----------------------------------------------------------------
def test_the_timing_table_is_the_saved_documents():
    """Guard against the numbers quietly drifting: these are AN1120a, AN2358 and AN1470a."""
    ten = profile_timings(NS_PER_TICK)
    # AN1120a Table 2: "10Base-T 100 ns ... Bit Time". 100 ns / 20 ns = 5 ticks per bit.
    assert ten["bit_rate"] == 10e6 and ten["ticks_per_bit"] == 5.0
    assert ten["half_ticks"] == 2.5
    # AN2358: "Selecting an x16 oversampling rate gives more than 10 percent frequency
    # tolerance on each side". The quantisation terms keep the windows disjoint.
    assert ten["interval_tol"] == 0.10
    assert (ten["half_lo"], ten["half_hi"]) == (1.5, 3.5)
    assert (ten["bit_lo"], ten["bit_hi"]) == (4.0, 6.0)
    assert ten["grid_tol"] == 1.0            # one tick, the bus resolution
    assert ten["idle_level"] == 1
    assert ten["min_gap_bits"] == 4.0 and ten["min_gap_ticks"] == 20.0
    assert ten["stream_gap_ticks"] == 11.0   # two bit times plus the widest window
    assert "100 ns" in ten["cite"]

    # AN1470a's worked example: 25 kHz is a 40 us bit time.
    slow = profile_timings(NS_PER_TICK, bit_rate=25e3)
    assert slow["ticks_per_bit"] == 40_000 / NS_PER_TICK == 2_000.0
    assert slow["bit_rate"] == 25e3
    # AN2358's other example is a 100 kbit/s link oversampled x16.
    mid = profile_timings(NS_PER_TICK, bit_rate=100e3)
    assert mid["ticks_per_bit"] == 500.0     # 10 us
    assert (mid["half_lo"], mid["half_hi"]) == (225.0, 275.0)
    assert (mid["bit_lo"], mid["bit_hi"]) == (450.0, 550.0)


def test_the_windows_must_not_overlap_or_nothing_can_be_classified():
    """A bit rate the tick cannot resolve is refused, loudly, instead of guessed at."""
    # Below 12.5 Mbit/s at 20 ns/tick the two windows are still apart ...
    assert profile_timings(NS_PER_TICK, bit_rate=12e6)["ticks_per_bit"] > 4.0
    # ... at 12.5 Mbit/s a bit is 4 ticks and a half bit is 2, so the windows meet, ...
    with pytest.raises(ValueError, match="no room between"):
        profile_timings(NS_PER_TICK, bit_rate=12.5e6)
    # ... and anything faster is worse.
    for rate in (20e6, 50e6, 100e6):
        with pytest.raises(ValueError, match="no room between"):
            profile_timings(NS_PER_TICK, bit_rate=rate)
    # IEEE 802.3's own 10 Mbit/s is exactly on the fast side of that line.
    assert profile_timings(NS_PER_TICK)["ticks_per_bit"] == 5.0
    # A coarser tick buys back the headroom, which is what the error message says: at 40 ns
    # a 1 Mbit/s bit is 25 ticks and there is room again.
    assert profile_timings(40.0, bit_rate=1e6)["ticks_per_bit"] == 25.0
    with pytest.raises(ValueError, match="no room between"):
        profile_timings(40.0, bit_rate=10e6)


def test_profile_timings_rejects_nonsense():
    with pytest.raises(ValueError, match="unknown Manchester profile"):
        profile_timings(NS_PER_TICK, "ieee802.3-100base-t4")
    with pytest.raises(ValueError, match="unknown timing override"):
        profile_timings(NS_PER_TICK, "ieee802.3-10base-t", overrides={"t_rise": 1})
    with pytest.raises(ValueError, match="must be positive"):
        profile_timings(NS_PER_TICK, bit_rate=0)
    with pytest.raises(ValueError, match="must be positive"):
        profile_timings(NS_PER_TICK, bit_rate=-10e6)
    with pytest.raises(ValueError, match="interval_tol"):
        profile_timings(NS_PER_TICK, "ieee802.3-10base-t", overrides={"interval_tol": -0.1})
    with pytest.raises(ValueError, match="idle_level"):
        profile_timings(NS_PER_TICK, "ieee802.3-10base-t", overrides={"idle_level": 2})


def test_checker_passes_the_traffic_at_the_loosest_published_window():
    """AN3015 accepts a one-bit interval anywhere in 0.75..1.25 bit times. At 10 Mbit/s the
    monitor can still use that window, because the one-tick quantisation term keeps it a
    hair apart from the half-bit window -- and legal traffic stays legal under it."""
    bus, tx, rx, chk = rig(rate=10e6, chk_timing={"interval_tol": 0.25})
    tx.send(bytes(range(16)))
    sent(tx, bus)
    assert chk.timings["interval_tol"] == 0.25
    assert chk.timings["half_hi"] < chk.timings["bit_lo"], chk.report()
    assert chk.clean, chk.report()
    assert rx.decoded_bytes() == bytes(range(16))


# ---- one test per rule the monitor judges by ------------------------------------------
def test_checker_flags_a_transition_too_close_to_the_last_one():
    """tHALF: two transitions less than half a bit apart are not in the line code.

    The second half of bit 3 is held one tick under the 2.5-tick nominal, so it lands one
    tick long: below the 1.5..3.5 half-bit window. The same bench without the fault is
    silent.
    """
    fault = ManchesterFaults(stretch_halves=((3, 1, -1.0),))
    bus, tx, rx, chk = rig(faults=fault)
    tx.send(b"\xff")
    sent(tx, bus)
    assert {v.rule for v in chk.violations} == {"tHALF"}, chk.report()
    assert_reported(chk)
    hit = chk.violations_by_rule("tHALF")[0]
    assert hit.measured == 1 and hit.short and hit.window == (1.5, 3.5)
    assert not rx.clean, "the receiver saw the same runt"
    assert rx.framing_errors >= 1

    clean_bus, clean_tx, _, clean_chk = rig()
    clean_tx.send(b"\xff")
    sent(clean_tx, clean_bus)
    assert clean_chk.clean, clean_chk.report()


def test_checker_flags_a_transition_that_is_neither_boundary_nor_mid_bit():
    """tMID-BIT: late for the bit boundary and early for the next mid-bit transition.

    At 1 Mbit/s a bit is 20 ticks, so the band between the two windows is 12..17 ticks wide
    and a half bit stretched by four ticks (25 -> 29) lands in it. At 10 Mbit/s the two
    windows are 3.5 and 4.0 ticks, so that band is empty -- the rule cannot fire there, which
    is why this test runs at 1 Mbit/s.
    """
    assert (profile_timings(NS_PER_TICK)["half_hi"],
            profile_timings(NS_PER_TICK)["bit_lo"]) == (3.5, 4.0)
    one = profile_timings(NS_PER_TICK, bit_rate=1e6)
    assert (one["half_lo"], one["half_hi"]) == (22.5, 27.5)     # a bit is 50 ticks here
    assert (one["bit_lo"], one["bit_hi"]) == (45.0, 55.0)
    assert (one["half_hi"], one["bit_lo"]) == (27.5, 45.0)

    fault = ManchesterFaults(stretch_halves=((3, 0, 4.0),))
    bus, tx, rx, chk = rig(rate=1e6, faults=fault)
    tx.send(b"\xff")
    sent(tx, bus)
    assert {v.rule for v in chk.violations} == {"tMID-BIT"}, chk.report()
    assert_reported(chk)
    hit = chk.violations_by_rule("tMID-BIT")[0]
    assert hit.measured == 29 and hit.long and hit.window == (22.5, 27.5)
    assert "45" in hit.detail, hit.detail
    assert not rx.clean


def test_checker_flags_a_missing_mid_bit_transition():
    """tBIT: a bit cell with no transition in the middle of it.

    The mandatory mid-bit transition of bit 3 is dropped, so the line holds its level
    through the whole cell and the next transition is a bit and a half away -- legal-looking
    nowhere. The data still comes back (the receiver resynchronises on the next cell) but one
    bit is lost, which is why the monitor says so rather than the receiver.
    """
    fault = ManchesterFaults(drop_mid_bits=(3,))
    bus, tx, rx, chk = rig(faults=fault)
    tx.send(b"\x00")            # all zero bits, so the drop is unambiguous
    sent(tx, bus)
    assert {v.rule for v in chk.violations} == {"tBIT"}, chk.report()
    assert_reported(chk)
    hit = chk.violations_by_rule("tBIT")[0]
    assert hit.long and hit.window == (4.0, 6.0) and hit.measured > 6.0
    assert len(chk.bits) < len(tx.messages[0].bits), "a bit went missing"
    assert not rx.clean and rx.resyncs >= 1


def test_checker_flags_a_boundary_transition_with_no_mid_bit_after_it():
    """tFRAMING: the wire form of the same error as tBIT.

    The first half of bit 3 is stretched by half a bit, so what should be that cell's mid-bit
    transition arrives a whole bit after the previous one -- an interval the tBIT window
    accepts, because a whole bit time is a legal interval in general. Only the *sequence*
    says it cannot be a mid-bit transition: a transition at a bit boundary is always
    followed, half a bit later, by the cell's mid-bit transition (AN2358: "each bit is
    transmitted with a transition in the middle of bit time").
    """
    fault = ManchesterFaults(stretch_halves=((3, 0, 25.0),))
    bus, tx, rx, chk = rig(rate=1e6, faults=fault)
    tx.send(b"\xff")
    sent(tx, bus)
    assert_reported(chk)
    hit = chk.violations_by_rule("tFRAMING")
    assert hit, chk.report()
    assert hit[0].measured == 50.0 and hit[0].long
    assert {v.rule for v in chk.violations} == {"tFRAMING", "tJIT"}, chk.report()
    assert not rx.clean
    # The bit grid moved by the same stretch, so the tJIT rule sees it too. Widen the grid
    # window and tFRAMING is the only thing left -- proof that it is its own rule.
    bus2, tx2, _, chk2 = rig(rate=1e6, faults=fault, chk_timing={"grid_tol_ticks": 60.0})
    tx2.send(b"\xff")
    sent(tx2, bus2)
    assert {v.rule for v in chk2.violations} == {"tFRAMING"}, chk2.report()


def test_checker_flags_a_rate_error_the_intervals_do_not_show():
    """tJIT, the rule no per-interval window can do.

    A transmitter 5 % fast at 10 Mbit/s sends every interval 5 ticks long: dead inside the
    4..6 tick window, so tBIT and tHALF stay quiet. But its bit grid walks away from the
    anchor the monitor set on the first mid-bit transition, and after a few bits the
    mid-bit transitions are more than one tick from where the grid says they belong. Long
    messages are what makes this visible, so this one is 24 bytes.
    """
    fault = ManchesterFaults(rate_scale=1.05)
    bus, tx, rx, chk = rig(faults=fault)
    tx.send(bytes(24))
    sent(tx, bus)
    assert_reported(chk)
    assert {v.rule for v in chk.violations} == {"tJIT"}, chk.report()
    assert not chk.violations_by_rule("tBIT") and not chk.violations_by_rule("tHALF")
    assert max(abs(v.measured) for v in chk.violations) > chk.timings["grid_tol"]
    assert rx.clean, "a 5 % rate error is inside every receiver's window"

    # 5 % the other way: the grid runs ahead of the wire instead.
    bus, tx, rx, chk = rig(faults=ManchesterFaults(rate_scale=0.95))
    tx.send(bytes(24))
    sent(tx, bus)
    assert {v.rule for v in chk.violations} == {"tJIT"}, chk.report()

    # A long enough grid window makes the same wire legal again: the rule is a limit of the
    # monitor, not a judgement about the transmitter. 5 % of 5 ticks over ~200 bits is ~50
    # ticks of drift, so that is the window it takes to cover all of it.
    bus, tx, rx, chk = rig(faults=fault, chk_timing={"grid_tol_ticks": 60.0})
    tx.send(bytes(24))
    sent(tx, bus)
    assert chk.clean, chk.report()

    # And a grid window of zero cannot be met at all, because a 2.5-tick half bit can only
    # be placed on a tick boundary: one tick is the floor, not a choice.
    bus, tx, rx, chk = rig(chk_timing={"grid_tol_ticks": 0.0})
    tx.send(b"\xa5")
    sent(tx, bus)
    assert chk.violations_by_rule("tJIT"), chk.report()


def test_checker_flags_a_message_without_the_sync_pattern():
    """tSYNC: a line code has no start bit, so the sync pattern is the only thing on the wire
    that says where the bit cells are.

    Two cases, both with perfect timing. A message that starts with a bit agreeing with the
    idle level is anchored correctly, and the monitor sees at once that the second bit is not
    the `1` the pattern asks for. A message of nothing but ones, with the idle level high,
    cannot be anchored from the wire at all: the first transition is the bit-boundary one, so
    the receiver reads a stream of zeros -- a perfectly legal Manchester waveform of the
    wrong bits, which no timing rule can see.
    """
    bus, tx, rx, chk = rig(preamble="")
    tx.send(b"\x00")
    sent(tx, bus)
    assert {v.rule for v in chk.violations} == {"tSYNC"}, chk.report()
    assert_reported(chk)
    assert chk.violations_by_rule("tSYNC")[0].tick > 0

    bus, tx, rx, chk = rig(preamble="")
    tx.send(b"\xff")
    sent(tx, bus)
    assert {v.rule for v in chk.violations} == {"tSYNC"}, chk.report()
    assert chk.bits == [0] * 8, "the mis-anchored stream reads as all zeros"
    assert not rx.clean or rx.messages[0].data_bits != [0] * 8

    # The pattern back on the wire, and the same data reads correctly.
    bus, tx, rx, chk = rig()
    tx.send(b"\xff")
    sent(tx, bus)
    assert chk.clean, chk.report()
    assert rx.decoded_bytes() == b"\xff"

    # A monitor told nothing about the pattern cannot check it -- which is the honest
    # statement: without a preamble the phase of the stream is not on the wire.
    bus, tx, rx, chk = rig(preamble="", chk_kw={"preamble": ""})
    tx.send(b"\xff")
    sent(tx, bus)
    assert chk.clean, chk.report()
    assert chk.bits == [0] * 8, "and with nothing to check it against, that is all it can say"


def test_checker_flags_a_line_stuck_off_the_idle_level():
    """tIDLE: the link is broken, not Manchester. A line held at the idle level is fine."""
    bus = manchester_bus()
    bus.attach(StuckLine(1))
    chk = attach_checker(bus, bit_rate=10e6)
    bus.run(2_000)
    assert chk.clean, chk.report()
    assert chk.violations == [] and chk.edges == []

    bus = manchester_bus()
    bus.attach(StuckLine(0))
    chk = attach_checker(bus, bit_rate=10e6)
    bus.run(2_000)
    assert_reported(chk)
    assert {v.rule for v in chk.violations} == {"tIDLE"}, chk.report()
    hit = chk.violations_by_rule("tIDLE")[0]
    assert hit.tick == 1 and hit.long and hit.window == (0.0, 11.0)
    assert bus.errors == [], "the stuck driver is the only one on the net"


def test_checker_flags_messages_too_close_together():
    """tGAP: two messages with less than the profile's minimum of idle line between them.

    Legal on the wire and useless to a receiver, which has no boundary to lock onto. With
    the default gap it is silent; with half the gap it is flagged; with no gap at all there is
    not even a transition to measure and the monitor sees one long stream -- also silent,
    also documented.
    """
    def two(gap):
        bus, tx, rx, chk = rig(gap_bits=gap)
        tx.send(b"\xa5")
        tx.send(b"\x3c")
        sent(tx, bus)
        return chk

    clean = two(4)
    assert clean.clean and len(clean.streams) == 2, clean.report()

    close = two(2)
    assert {v.rule for v in close.violations} == {"tGAP"}, close.report()
    assert_reported(close)
    gap = close.violations_by_rule("tGAP")[0]
    assert gap.window == (20.0, INF)                    # 4 bit times
    assert 0 < gap.measured < gap.window[0], gap        # two bit times of idle line

    merged = two(0)
    assert merged.clean and len(merged.streams) == 1, merged.report()
    assert len(merged.bits) == 2 + 8 + 2 + 8, "one stream: the two messages run together"


# ---- the monitor is not vacuous, and the two devices agree -------------------------------
def watched(rate, mon_rate, payload, chk_timing=None, chk_kw=None, **tx_kw):
    """A bus whose transmitter runs at `rate` with a monitor judging it as `mon_rate`."""
    bus = manchester_bus()
    tx = attach_tx(bus, bit_rate=rate, **tx_kw)
    chk = attach_checker(bus, bit_rate=mon_rate, timing=chk_timing, **(chk_kw or {}))
    tx.send(payload)
    sent(tx, bus)
    return tx, chk


def test_checker_is_not_vacuous():
    """The same pin traffic, judged against limits the wire is not running at, must fail.

    A 10 Mbit/s stream on a 1 Mbit/s monitor has every interval five times too long, so the
    monitor cannot classify one of them; a 1 Mbit/s stream on a 10 Mbit/s monitor has every
    one of them shorter than a half bit. Neither is a licence to pass, and the clean bench on
    the right rate is here so the pair cannot both pass by being broken.
    """
    payload = bytes(range(16))
    _, clean = watched(10e6, 10e6, payload)
    assert clean.clean, clean.report()
    assert clean.stats["transitions"] > 100 and clean.stats["bits"] > 100, clean.stats
    assert clean.stats["shortest_in_stream"] == 2 and clean.stats["longest_in_stream"] == 5

    # Fast wire, slow monitor: every interval is far shorter than half a bit.
    _, short = watched(10e6, 1e6, payload)
    assert not short.clean and len(short.violations) > 10, short.report()
    assert {v.rule for v in short.violations} == {"tHALF"}, short.report()
    assert_reported(short)

    # Slow wire, fast monitor: the intervals that are not yet longer than the monitor's
    # stream gap are longer than a whole bit, so nothing can be a mid-bit transition.
    _, stretched = watched(1e6, 2e6, payload)
    assert not stretched.clean and len(stretched.violations) > 10, stretched.report()
    assert {v.rule for v in stretched.violations} == {"tBIT"}, stretched.report()
    assert_reported(stretched)

    # And with a monitor so far off that every transition looks like a message boundary, the
    # line never looks idle between them either.
    _, far_off = watched(1e6, 10e6, payload)
    assert not far_off.clean, far_off.report()
    assert "tIDLE" in {v.rule for v in far_off.violations}, far_off.report()

    # Same point from the other side: the limits come from the monitor's configuration, so
    # a grid window of zero cannot be met even by the traffic that is exactly on rate.
    _, tight = watched(10e6, 10e6, payload, chk_timing={"interval_tol": 0.0,
                                                          "grid_tol_ticks": 0.0})
    assert tight.violations, "no limit is not a licence to pass everything"
    assert_reported(tight)


def test_receiver_and_monitor_agree_on_what_was_broken():
    """Both passive devices read the same pins, so both must notice the same broken traffic
    and both must stay quiet on legal traffic."""
    for data, faults in ((b"\xa5", None),
                         (b"\x00", ManchesterFaults(drop_mid_bits=(3,))),
                         (b"\xff", ManchesterFaults(stretch_halves=((3, 1, -1.0),))),
                         (b"\xff", ManchesterFaults(stretch_halves=((3, 0, 4.0),)))):
        rate = 1e6 if faults and faults.stretch_halves == ((3, 0, 4.0),) else 10e6
        bus, tx, rx, chk = rig(rate=rate, faults=faults)
        tx.send(data)
        sent(tx, bus)
        assert bool(rx.errors) == bool(chk.violations), (rx.report(), chk.report())
        if faults is None:
            assert rx.clean and chk.clean
            assert rx.decoded_bytes() == data
        else:
            assert not rx.clean and not chk.clean
            assert len(rx.bits) <= len(chk.bits)


def test_transmitter_holds_the_requested_bit_rate():
    """Phase is accumulated, not rounded per chip, so the rate is right over a long message
    even when one chip cannot be drawn on the tick grid."""
    for rate, ticks in ((10e6, 5.0), (1e6, 50.0), (100e3, 500.0)):
        bus, tx, rx, chk = rig(rate=rate)
        tx.send(bytes(40))
        sent(tx, bus)
        message = tx.messages[0]
        n_bits = len(message.bits)
        # The message runs from the first chip of the sync pattern to the last chip of the
        # data, so its length in ticks divided by the bits is the bit period on the wire.
        measured = (message.end_tick - message.start_tick) / n_bits
        assert abs(measured - ticks) <= 1.0, (rate, measured)
        assert len([e for e in chk.edges if e.kind == "start"]) == 1
        assert chk.clean, chk.report()


# ---- guard rails ---------------------------------------------------------------------
def test_guards_reject_nonsense_configurations():
    with pytest.raises(ValueError, match="convention must be one of"):
        ManchesterTx(convention="802.3")
    with pytest.raises(ValueError, match="convention must be one of"):
        ManchesterRx(convention="biphase")
    with pytest.raises(ValueError, match="convention must be one of"):
        ManchesterTimingChecker(convention="thomas-1949")
    with pytest.raises(ValueError, match="bit_order"):
        ManchesterTx(bit_order="little")
    with pytest.raises(ValueError, match="preamble must be"):
        ManchesterTx(preamble="0x1")
    with pytest.raises(ValueError, match="preamble must be"):
        ManchesterTimingChecker(preamble="yes")
    with pytest.raises(ValueError, match="rate_scale"):
        ManchesterTx(faults=ManchesterFaults(rate_scale=0))
    with pytest.raises(ValueError, match="preamble_bits"):
        ManchesterRx(preamble_bits=-1)
    with pytest.raises(ValueError, match="not a string"):
        ManchesterTx().send("hello")
    with pytest.raises(ValueError, match="net must be a net name"):
        ManchesterTx(net="")
    with pytest.raises(ValueError, match="no room between"):
        ManchesterTx(bit_rate=100e6)
    # The helpers create the push-pull net themselves, on any bus.
    bare = Bus(ns_per_tick=NS_PER_TICK)
    attach_tx(bare, bit_rate=10e6)
    assert bare.nets["tx"].push_pull is True
    bare = Bus(ns_per_tick=NS_PER_TICK)
    with pytest.raises(KeyError, match="unknown net"):
        bare.attach(ManchesterTx(net="line", bit_rate=10e6))
        bare.step()
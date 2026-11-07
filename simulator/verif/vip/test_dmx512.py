"""Self-tests for the pin-level DMX512-A reference models: `make vip-selftest VIP=dmx512`.

Everything runs at 50 MHz (20 ns per tick), which makes one 250 kbit/s bit exactly 200 ticks
and every table value in the spec a whole number of ticks: a 92 us break is 4600, a 12 us MAB
is 600, and the 1204 us minimum packet time is 60200. `rig()` builds the bench -- a
transmitter, a receiver and the passive checker on one net -- and the tests look at the
receiver's levels, at what the checker measured, and at what it says about the wire.

The suite is in three parts: the models talking to each other, one test per rule the
checker enforces, and guards that the timing table is still the standard's. The numbers in
those guards come from the spec text saved in research/ref/protocols/dmx512/.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.dmx512 import (
    MAX_DATA_SLOTS,
    NULL_START_CODE,
    RX_PROFILE,
    RULE_TEXT,
    SLOT_BITS,
    TX_PROFILE,
    DmxChecker,
    DmxRx,
    DmxTx,
    Violation,
    attach_checker,
    attach_rx,
    attach_tx,
    dmx_minimums,
    ns_to_ticks,
)

NS_PER_TICK = 20.0                    # 50 MHz chip clock
BIT_TICKS = 200.0                     # 4 us at 20 ns/tick: 250 kbit/s exactly
BREAK_TX = 4600                       # 92 us, table 6 designator 1
MAB_TX = 600                          # 12 us, table 6 designator 2
B2B_TX = 60200                        # 1204 us, table 6 designator 11
ALT_START_CODE = 0xCF                  # a registered Alternate START Code (clause 8.5.3)


def rig(address=1, footprint=1, tx_timing=None, rx_timing=None, check_timing=None,
        strict_tx=False, pad=True, check=True, rx_ticks_per_bit=None,
        ns_per_tick=NS_PER_TICK):
    """A bus with a DmxTx, a DmxRx and a DmxChecker on it.

    The timing hooks are how one part of the bench is driven off-spec without touching the
    others: `tx_timing` for the controller, `rx_timing` for what the receiver recognises,
    `check_timing` for the limits the monitor judges by, and `strict_tx` to ask the monitor
    for table 6 instead of table 7.
    """
    bus = Bus(ns_per_tick=ns_per_tick)
    tx = attach_tx(bus, timing=tx_timing, pad_break_to_break=pad)
    rx = attach_rx(bus, address=address, footprint=footprint, timing=rx_timing,
                   ticks_per_bit=rx_ticks_per_bit)
    chk = attach_checker(bus, timing=check_timing, strict_tx=strict_tx) if check else None
    return bus, tx, rx, chk


def packet(values=(0x55, 0xAA), start_code=NULL_START_CODE, timing=None, faults=None):
    """The keyword arguments every `tx.queue_packet(...)` in these tests uses."""
    return {"values": list(values), "start_code": start_code, "timing": timing,
            "faults": faults}


def settle(bus, tx, chk, rx=None, limit=400_000):
    """Run the bus until the transmitter is idle, then close the packets in flight.

    `DmxChecker` and `DmxRx` can only know a packet is over when the next one starts, so the
    last packet of a run has to be closed by hand. `finish()` on both is idempotent.
    """
    assert tx.run(bus, limit=limit), f"the transmitter did not drain its queue in {limit} ticks"
    chk.finish()
    if rx is not None:
        rx.finish()


def run_packets(rig_kwargs=None, packets=(), **settle_kwargs):
    """A bench, a queue of packets, and the run: returns (bus, tx, rx, chk)."""
    bus, tx, rx, chk = rig(**(rig_kwargs or {}))
    for kwargs in packets:
        tx.queue_packet(**kwargs)
    settle(bus, tx, chk, rx, **settle_kwargs)
    return bus, tx, rx, chk


def rules(chk):
    """The set of rules the checker flagged, for an exact-match assertion."""
    return {v.rule for v in chk.violations}


def only(chk):
    """Assert every violation carries a tick inside the run and return the rule names."""
    for v in chk.violations:
        assert v.tick >= 0, v
    return rules(chk)


# ---- the models talking to each other ------------------------------------------------
def test_a_packet_reaches_the_receiver_and_the_checker_agrees():
    """One NULL START Code packet: the receiver holds the levels, the checker is silent."""
    values = [0xFF, 0x80, 0x00, 0x40]
    bus, tx, rx, chk = run_packets({"address": 1, "footprint": 4}, [packet(values)])

    assert bus.errors == [], "a push-pull link with one driver cannot have contention"
    assert chk.clean, chk.report()
    assert len(chk.packets) == 1
    seen = chk.packets[0]
    assert (seen.break_ticks, seen.mab_ticks) == (BREAK_TX, MAB_TX)
    assert seen.start_code == NULL_START_CODE and seen.slots == 4
    assert seen.bit_ticks == BIT_TICKS and seen.bit_rate == 250e3
    assert seen.break_tick == tx.tx_log[0].start_tick == 0
    assert seen.mab_ticks == tx.tx_log[0].mab_ticks

    got = rx.packets[-1]
    assert got.start_code == NULL_START_CODE and got.null_start
    assert got.slots == 4 and list(got.values) == values
    assert rx.window == bytes(values) and list(rx.levels[:4]) == values
    assert rx.breaks == 1 and not rx.framing_errors and not rx.mab_errors
    assert not rx.short_breaks and not rx.double_breaks and not rx.glitches
    # slot 1 follows the break, the MAB and the start code's eleven bit times; slot n+1
    # follows slot n's eleven
    assert [tick for tick, _slot, _value in rx.slot_log] == [7400 + n * 2200
                                                             for n in range(4)]
    assert [slot for _tick, slot, _value in rx.slot_log] == [1, 2, 3, 4]


def test_the_checker_decodes_what_the_transmitter_sent():
    """Slot values come back out of the edge list in order, LSB first, with the start code."""
    values = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF, 0x00]
    bench = {"address": 1, "footprint": len(values)}
    _bus, _tx, rx, chk = run_packets(bench, [packet(values)])

    assert chk.clean, chk.report()
    assert chk.packets[0].slots == len(values)
    assert list(rx.packets[0].values) == values        # every data slot, in order
    assert rx.window == bytes(values)                 # and all of them in this receiver
    assert rx.slot_count == len(values)


def test_repeated_packets_update_the_levels_and_count_separately():
    """Four packets of the same length: the last one wins, and each is measured."""
    bench = {"address": 1, "footprint": 3}
    packets = [packet([0x10, 0x20, 0x30]), packet([0x40, 0x50, 0x60]),
               packet([0x70, 0x80, 0x90]), packet([0xA0, 0xB0, 0xC0])]
    _bus, _tx, rx, chk = run_packets(bench, packets)

    assert chk.clean, chk.report()
    assert len(chk.packets) == 4 and all(p.slots == 3 for p in chk.packets)
    assert len(rx.packets) == 4 and rx.slot_count == 3
    assert rx.window == bytes([0xA0, 0xB0, 0xC0])
    # every packet is a full packet time apart, and each one is measured at its own tick
    starts = [p.break_tick for p in chk.packets]
    assert starts == [0, 60199, 120398, 180597]
    assert all(later - earlier == B2B_TX - 1
               for earlier, later in zip(starts, starts[1:]))


def test_a_receiver_takes_the_slots_in_its_own_address_window():
    """Three receivers on one link, each with its own address and footprint."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_tx(bus)
    a = attach_rx(bus, address=1, footprint=2)
    b = attach_rx(bus, address=3, footprint=1)
    c = attach_rx(bus, address=8, footprint=1)
    chk = attach_checker(bus)
    values = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
    tx.queue_packet(values)
    settle(bus, tx, chk)
    for receiver in (a, b, c):
        receiver.finish()

    assert chk.clean, chk.report()
    assert a.window == bytes([0x11, 0x22])
    assert b.window == bytes([0x33])
    assert c.window == bytes([0x88])            # slot 8 is the last value sent
    assert [len(rx.packets) for rx in (a, b, c)] == [1, 1, 1]
    assert all(rx.packets[0].slots == len(values) for rx in (a, b, c))


def test_a_full_universe_of_512_slots_is_legal_and_takes_22_7_ms():
    """513 slots: the whole universe, the update time the tables quote, and no complaint.

    Tables 6 and 7: "Minimum Update Time for 513 Slots 22.7 ms" and "Maximum Refresh Rate
    for 513 Slots 44 updates/s". A 513-slot packet at 250 kbit/s with the transmitter's
    92 us break and 12 us MAB is 4600 + 600 + 513 * 2200 = 1_133_800 ticks, which is 22.7 ms
    at 20 ns/tick.
    """
    values = [i % 256 for i in range(MAX_DATA_SLOTS)]
    bus, tx, rx, chk = run_packets({"address": 1, "footprint": 4},
                                   [packet(values)], limit=2_000_000)

    assert chk.clean, chk.report()
    seen = chk.packets[0]
    assert seen.slots == MAX_DATA_SLOTS and seen.bit_rate == 250e3
    assert seen.break_tick == 0
    assert tx.tx_log[0].end_tick == 1_133_800
    # the model always leaves one tick of mark between packets, which is all a full
    # universe needs to stay legal
    assert tx.tx_log[0].mbb_ticks == 1
    assert tx.tx_log[0].end_tick * NS_PER_TICK / 1e6 == 22.676             # ms, tables 6/7
    got = rx.packets[-1]
    assert got.slots == MAX_DATA_SLOTS and got.framing_errors == 0
    assert rx.window == bytes(values[:4])
    assert not rx.over_512


def test_more_than_512_data_slots_is_one_slot_too_many():
    """The tables stop at 512 data slots; a 513th is a violation, not a universe."""
    _bus, _tx, _rx, chk = run_packets({}, [packet([0x5A] * (MAX_DATA_SLOTS + 1))],
                                      limit=2_000_000)

    assert rules(chk) == {"MAX-SLOTS"}, chk.report()
    over = chk.violations_by_rule("MAX-SLOTS")
    assert len(over) == 1 and "data slot 513" in over[0].detail
    assert over[0].tick == 4600 + 600 + 513 * 2200


# ---- every rule the checker enforces, with the traffic that breaks it ----------------
def test_a_break_shorter_than_the_minimum_is_not_a_break():
    """BREAK: 10 us of space is not the 88 us table 7 asks a receiver to recognise."""
    _bus, _tx, _rx, chk = run_packets({"tx_timing": {"break": 10_000}},
                                       [packet([0x55])])

    assert "BREAK" in only(chk), chk.report()
    first = chk.violations_by_rule("BREAK")[0]
    assert (first.measured_ns, first.required_ns) == (10_000.0, 88_000.0)
    assert first.tick == 500, "the space ended at tick 500, and that is where it is flagged"
    # the same wire against the transmitter's own 92 us minimum is still too short
    _bus, _tx, _rx, strict = run_packets({"tx_timing": {"break": 10_000},
                                          "strict_tx": True}, [packet([0x55])])
    over = strict.violations_by_rule("BREAK")[0]
    assert over.required_ns == 92_000.0


def test_a_mark_after_break_shorter_than_the_minimum_is_flagged():
    """MAB: 2 us of mark where table 7 designator 2 asks for 8."""
    _bus, _tx, _rx, chk = run_packets({"tx_timing": {"mab": 2_000}}, [packet([0x55])])

    assert only(chk) == {"MAB"}, chk.report()
    only_one = chk.violations[0]
    assert (only_one.measured_ns, only_one.required_ns) == (2_000.0, 8_000.0)
    assert only_one.tick == BREAK_TX + 100, "the MAB ended where the START Code began"


def test_a_second_break_before_the_start_code_is_flagged():
    """MULTI-BREAK: 8.9 -- a packet is one reset sequence, not several breaks."""
    _bus, _tx, _rx, chk = run_packets({}, [packet([0x55], faults={"double_break": 1})])

    assert "MULTI-BREAK" in only(chk), chk.report()
    at = chk.violations_by_rule("MULTI-BREAK")[0]
    assert at.tick == BREAK_TX + MAB_TX, "the second break began right after the MAB"
    # the first "packet" holds no slots at all: it never got as far as a START Code
    assert chk.packets[0].slots == 0 and chk.packets[0].start_code is None
    # the MAB was measured, but never followed by a START Code, so mab_ticks stays None:
    # the checker only reports a MAB it could compare against a slot
    assert chk.packets[0].mab_ticks is None
    # and the traffic behind it is still a legal packet, so the checker carries on
    assert chk.packets[1].start_code == NULL_START_CODE and chk.packets[1].slots == 1
    # two breaks 104 us apart break 8.10 as well, which is the same fault seen twice
    assert "BREAK-TO-BREAK" in rules(chk)


def test_break_to_break_is_checked_on_both_sides():
    """BREAK-TO-BREAK: 8.10 -- at least 1196 us apart, and at most 1.25 s.

    Two one-slot packets with the transmitter's mark-before-break padding switched off are
    148 us apart, well under the minimum, and padding is what a real controller relies on to
    make a short packet legal at all (clause 8.7 sets no minimum number of data slots).
    """
    _bus, _tx, _rx, chk = run_packets({"pad": False},
                                       [packet([0x55]), packet([0xAA])])
    assert only(chk) == {"BREAK-TO-BREAK"}, chk.report()
    short = chk.violations[0]
    assert (short.measured_ns, short.required_ns) == (192_000.0, 1_196_000.0)
    assert short.tick == 9600, "the second break's falling edge"

    # the same wire, with a checker told the tables' maximum is 100 us
    _bus, _tx, _rx, capped = run_packets({"check_timing": {"b2b_max": 100_000}},
                                          [packet([0x55]), packet([0xAA])])
    assert only(capped) == {"BREAK-TO-BREAK"}, capped.report()
    long = capped.violations[0]
    assert long.required_ns == 100_000.0 and long.measured_ns == 1_203_980.0


def test_a_bit_rate_outside_245_to_255_kbit_per_second_is_flagged():
    """BIT-RATE: the tables' 3.92 .. 4.08 us bit time, on both sides.

    The checker measures the period off the wire rather than assuming 250 kbit/s, so these
    are measured numbers: 300 kbit/s is 3.333 us per bit and 150 kbit/s is 6.667 us, and the
    checker says which side of the window each one fell out of.
    """
    _bus, _tx, _rx, fast = run_packets({"tx_timing": {"bit_rate": 300e3}},
                                        [packet([0x55, 0xAA])])
    assert only(fast) == {"BIT-RATE"}, fast.report()
    quick = fast.violations[0]
    assert quick.measured_ns == pytest.approx(3_333.3, rel=0.01)   # 167 ticks, measured
    # 1 / 255 kbit/s is 3921.6 ns, which the table prints as 3.92 us
    assert quick.required_ns == pytest.approx(3_920.0, rel=0.001)
    assert 290e3 < _rate(fast) < 300e3, fast.report()

    _bus, _tx, _rx, slow = run_packets({"tx_timing": {"bit_rate": 150e3}},
                                        [packet([0x55, 0xAA])])
    assert only(slow) == {"BIT-RATE"}, slow.report()
    lazy = slow.violations[0]
    assert lazy.measured_ns == pytest.approx(6_666.7, rel=0.01)
    assert lazy.required_ns == pytest.approx(4_080.0, rel=0.001), "1 / 245 kbit/s"
    assert 150e3 < _rate(slow) < 152e3, slow.report()


def test_the_ends_of_the_legal_bit_rate_window_are_not_violations():
    """245 and 255 kbit/s are the tables' limits, so a checker must let them past.

    A 20 ns tick cannot express 3.92 us exactly: 255 kbit/s is 196.08 ticks and 245 kbit/s is
    204.08. The checker's comparison allows one tick of quantisation for that reason, so a
    transmitter running at either limit is legal and one 1 kbit/s outside it is not.
    """
    for rate in (245e3, 250e3, 255e3):
        _bus, _tx, _rx, chk = run_packets({"tx_timing": {"bit_rate": rate}},
                                          [packet([0x55, 0xAA, 0x00, 0xFF])])
        assert chk.clean, f"{rate / 1e3:.0f} kbit/s is in the window: {chk.report()}"
        assert chk.packets[0].bit_rate == pytest.approx(rate, rel=0.005)

    _bus, _tx, _rx, over = run_packets({"tx_timing": {"bit_rate": 260e3}},
                                        [packet([0x55, 0xAA])])
    assert only(over) == {"BIT-RATE"}, over.report()


def test_a_bit_that_is_not_a_whole_number_of_bit_times_is_jitter():
    """BIT-JITTER: one bit 60 ns too long is not a bit rate, it is jitter."""
    _bus, _tx, _rx, chk = run_packets({}, [packet([0x55, 0xAA],
                                                 faults={"bit_stretch": (1, 4, 3)})])

    assert only(chk) == {"BIT-JITTER"}, chk.report()
    wobble = chk.violations[0]
    assert (wobble.measured_ns, wobble.required_ns) == (4_060.0, 4_000.0)
    assert wobble.tick == BREAK_TX + MAB_TX + 2200 + 4 * BIT_TICKS


def _rate(chk):
    """The bit rate the checker measured, in bit/s."""
    return chk.packets[0].bit_rate


def test_a_missing_first_stop_bit_is_flagged_and_the_packet_is_dropped():
    """STOP1: 9.1 -- a receiver shall check the first stop bit, and shall then discard the
    improperly framed slot and every slot after it in that packet.

    Fault index 2 is the slot holding 0x22, counting the START Code as slot 0, so it is data
    slot 2. The checker reports the framing error and stops decoding the packet there, and
    the receiver does the same thing to its levels: 0x22 and the 0x33 behind it never reach
    `levels`.
    """
    values = [0x11, 0x22, 0x33, 0x44]
    _bus, _tx, rx, chk = run_packets({"address": 1, "footprint": 4},
                                     [packet(values, faults={"stop1_low": 2})])

    assert only(chk) == {"STOP1"}, chk.report()
    bad = chk.violations[0]
    assert bad.tick == 5200 + 2 * 2200, "the start bit of the slot whose stop bits are wrong"
    assert "first stop bit" in bad.detail and "slot 2" in bad.detail
    assert chk.packets[0].slots == 2, "clause 9.1: decoding stops at the badly framed slot"
    assert rx.framing_errors == 1
    assert rx.window == bytes([0x11, 0x00, 0x00, 0x00]), "0x22 never arrived"
    assert [slot for _t, slot, _v in rx.slot_log] == [1]


def test_a_missing_second_stop_bit_is_flagged_too():
    """STOP2: 9.1 says a receiver *should* check the second stop bit, and this one does."""
    _bus, _tx, rx, chk = run_packets({}, [packet([0x11, 0x22, 0x33],
                                                faults={"stop2_low": 1})])

    assert only(chk) == {"STOP2"}, chk.report()
    bad = chk.violations[0]
    assert bad.tick == 5200 + 2200 and "second stop bit" in bad.detail
    assert chk.packets[0].slots == 1, "the badly framed slot is counted, and then nothing"
    assert rx.framing_errors == 1 and rx.window == bytes([0x00]), "0x11 was thrown away"


def test_a_start_bit_too_short_to_sample_is_flagged():
    """SLOT-FORMAT: table 5 -- the start bit is a space, and it lasts a whole bit time."""
    _bus, _tx, _rx, chk = run_packets({}, [packet([0x11, 0x22],
                                                start_code=0xFF,
                                                faults={"short_start": (0, 10)})])

    assert "SLOT-FORMAT" in only(chk), chk.report()
    bad = chk.violations_by_rule("SLOT-FORMAT")[0]
    assert bad.tick == 5200 and "start bit" in bad.detail
    assert chk.packets[0].slots == 0, "the packet could not be decoded past slot 0"


def test_a_stray_space_between_slots_is_flagged():
    """IDLE-MARK: 8.8 -- the line shall remain in a marking state between slots."""
    _bus, _tx, _rx, chk = run_packets({}, [packet([0x55, 0xAA],
                                                faults={"space_between": (1, 4_000)})])

    assert only(chk) == {"IDLE-MARK"}, chk.report()
    bad = chk.violations[0]
    assert bad.tick == 5200 + 2 * 2200, "right where slot 1's eleven bit times ended"
    assert "mark state" in bad.detail
    assert chk.packets[0].slots == 2, "both slots are still decoded"


def test_an_alternate_start_code_is_not_a_null_start_code():
    """8.5.4: a receiver has to look at the START Code, not assume it is zero."""
    values = [0xDE, 0xAD]
    _bus, _tx, rx, chk = run_packets({}, [packet(values, start_code=ALT_START_CODE)])

    assert chk.clean, chk.report()
    assert chk.packets[0].start_code == ALT_START_CODE
    got = rx.packets[0]
    assert got.start_code == ALT_START_CODE and got.null_start is False
    assert rx.start_code == ALT_START_CODE and not rx.null_start
    assert list(rx.packets[0].values) == values

    # ... and the NULL START Code packet that follows is
    _bus, tx2, rx2, chk2 = run_packets({}, [packet(values, start_code=ALT_START_CODE),
                                            packet(values)])
    assert chk2.clean, chk2.report()
    assert [p.null_start for p in rx2.packets] == [False, True]
    assert rx2.packets[1].start_code == NULL_START_CODE


def test_a_receivers_limits_are_its_own():
    """The receiver's break window is table 7's, and `timing` moves it.

    A receiver told to require a 200 us break ignores a controller that sends the table's
    92 us, and counts the space as one that was too short to be a break; the same wire on a
    default receiver is two packets.
    """
    _bus, _tx, picky, _chk = run_packets({"rx_timing": {"break": 200_000}},
                                          [packet([0x55]), packet([0xAA])])
    assert picky.breaks == 0, "two 92 us breaks are not the 200 us this receiver wants"
    assert picky.slot_count == 0 and not picky.packets and picky.start_code is None
    assert picky.window == bytes(1), "so it never took a level from either packet"

    # A receiver told to want only 20 us is worse off, and this is why the tables say 88:
    # a NULL START Code slot holds the line low for nine bit times, 36 us, which such a
    # receiver mistakes for a break, so it sees four reset sequences instead of two and
    # decodes no data slots at all. The wire is still clean as far as the checker is
    # concerned -- it is the receiver that is wrong, not the controller.
    _bus, _tx, easy, chk = run_packets({"rx_timing": {"break": 20_000}},
                                       [packet([0x55]), packet([0xAA])])
    assert easy.breaks == 4, "two real breaks and the two NULL START Code slots"
    assert all(p.slots == 0 for p in easy.packets)
    assert easy.start_code is not NULL_START_CODE and easy.slot_count == 0
    assert chk.clean, chk.report()

    _bus, _tx, picky_mab, _chk = run_packets({"rx_timing": {"mab": 40_000}},
                                              [packet([0x55])])
    assert picky_mab.breaks == 1 and picky_mab.mab_errors == 1, "12 us is not the 40 us asked"
    assert picky_mab.packets[0].mab_ticks == MAB_TX
    # it decodes the slot anyway: a receiver has no better answer than to count the MAB and
    # take the slot that follows, which is why this shows up as a counter and not as silence
    assert picky_mab.start_code == NULL_START_CODE and picky_mab.slot_count == 1
    assert picky_mab.window == bytes([0x55])


# ---- the numbers are the standard's, and the limits are parameters --------------------
def test_the_timing_table_is_the_spec():
    """Guard against the numbers drifting. These are tables 6 and 7 of ANSI E1.11.

    The nanosecond values are quoted straight out of the spec text saved in
    research/ref/protocols/dmx512/, and the tick values are those numbers at 20 ns/tick.
    """
    tx = dmx_minimums(NS_PER_TICK, "transmit")
    rx = dmx_minimums(NS_PER_TICK, "receive")
    # table 6, output of the transmitting UART
    assert (tx["break"], tx["mab"], tx["b2b_min"]) == (4600, 600, 60200)   # 92, 12, 1204 us
    assert tx["b2b_max"] == 50_000_000 and TX_PROFILE.break_typ == 176_000.0
    # table 7, values for receivers
    assert (rx["break"], rx["mab"], rx["b2b_min"]) == (4400, 400, 59800)   # 88, 8, 1196 us
    assert rx["b2b_max"] == 62_500_000
    # both tables: 245 / 250 / 255 kbit/s, a 4 us bit time, 3.92 .. 4.08 us
    for table in (tx, rx):
        assert (table["bit_rate_min"], table["bit_rate"], table["bit_rate_max"]) == (
            245e3, 250e3, 255e3)
        assert table["bit_time"] == 4_000.0 and table["ticks_per_bit"] == BIT_TICKS
        assert table["bit_time_min_ns"] == pytest.approx(3_920.0, rel=0.001)
        assert table["bit_time_max_ns"] == pytest.approx(4_080.0, rel=0.001)
        assert table["min_bit_ticks"] == pytest.approx(196.08, rel=1e-3)
        assert table["max_bit_ticks"] == pytest.approx(204.08, rel=1e-3)
        assert table["max_slots"] == 512 and table["profile"] in ("transmit", "receive")
    assert TX_PROFILE.break_min == 92_000.0 and RX_PROFILE.break_min == 88_000.0
    assert SLOT_BITS == 11 and NULL_START_CODE == 0 and MAX_DATA_SLOTS == 512

    # rounding is always upwards, so a whole tick can only make a check stricter
    assert ns_to_ticks(20.0, 88_000) == 4400 and ns_to_ticks(20.0, 88_001) == 4401
    assert ns_to_ticks(8.0, 4_000) == 500 and ns_to_ticks(12.5, 4_000) == 320
    assert ns_to_ticks(20.0, -5) == 0

    # overrides and errors
    assert dmx_minimums(20.0, "receive", {"break": 10_000})["break"] == 500
    # 4000 ns of bit time is 200 ticks at 20 ns; at 300 kbit/s it is 3333.3 ns, 166.7 ticks
    assert dmx_minimums(20.0, "transmit", {"bit_rate": 300e3})["ticks_per_bit"] == pytest.approx(
        1000 / 6, rel=1e-9)
    with pytest.raises(ValueError):
        dmx_minimums(NS_PER_TICK, "fast-plus-plus")
    with pytest.raises(ValueError):
        dmx_minimums(NS_PER_TICK, "receive", {"t_nonsense": 1})
    with pytest.raises(ValueError):
        dmx_minimums(NS_PER_TICK, "receive", {"bit_rate": 0})
    with pytest.raises(ValueError):
        dmx_minimums(NS_PER_TICK, "receive", {"bit_rate_min": 300e3})


def test_every_rule_the_checker_can_flag_is_documented():
    """A firmware author looks a failure up in RULE_TEXT, so every rule has to be there."""
    flagged = set()
    benches = [
        {"tx_timing": {"break": 10_000}}, {"tx_timing": {"mab": 2_000}},
        {"faults": {"double_break": 1}}, {"pad": False, "packets": 2},
        {"tx_timing": {"bit_rate": 300e3}},
        {"faults": {"bit_stretch": (1, 4, 3)}}, {"faults": {"stop1_low": 2}},
        {"faults": {"stop2_low": 1}}, {"faults": {"space_between": (1, 4_000)}},
        {"check_timing": {"max_slots": 1}},
        {"faults": {"short_start": (0, 10)}, "start_code": 0xFF},
    ]
    for index, bench in enumerate(benches):
        bench = dict(bench)
        start_code = bench.pop("start_code", NULL_START_CODE)
        faults = bench.pop("faults", None)
        count = bench.pop("packets", 1)
        _bus, _tx, _rx, chk = run_packets(bench, [
            packet([0x55, 0xAA], start_code=start_code, faults=faults)
            for _ in range(count)])
        assert chk.violations, f"bench {index} ({bench}) flags nothing"
        flagged |= rules(chk)

    assert flagged == set(RULE_TEXT), f"undocumented: {sorted(flagged ^ set(RULE_TEXT))}"
    for rule, text in RULE_TEXT.items():
        assert "clause" in text or "table" in text, f"{rule} does not cite the standard"
        assert len(text) > 60, f"{rule} needs more than a word of explanation"


def test_the_transmitter_drives_what_the_transmitter_table_asks():
    """What DmxTx puts on the wire: the table's minimums, 11-bit slots, no gap between."""
    bus, tx, rx, chk = run_packets({}, [packet([0x55], start_code=NULL_START_CODE)])
    frame = tx.tx_log[0]
    # re-run the same bench with a probe on the net to look at the wire itself
    wire = Bus(ns_per_tick=NS_PER_TICK)
    tx2 = attach_tx(wire)
    probe = Probe()
    bus2 = wire
    bus2.attach(probe)
    tx2.queue_packet([0x55])
    tx2.run(bus2, limit=400_000)
    levels = probe.levels

    assert (frame.break_ticks, frame.mab_ticks) == (BREAK_TX, MAB_TX)
    assert frame.slots == 2 and frame.wire_ticks == frame.end_tick + 1
    assert frame.mbb_ticks == B2B_TX - BREAK_TX - MAB_TX - 2 * 11 * BIT_TICKS - 1
    # slots run back to back: the second slot's start bit is 11 bit times after the first
    assert [tick for tick, slot, _v in rx.slot_log] == [BREAK_TX + MAB_TX + 11 * BIT_TICKS]
    # the line idles high, then the break, then the mark after it. A probe records the value
    # the bus resolves at the end of a tick, so the tx's break is on the wire from tick 1.
    assert levels[0] == 1, "the line is a mark before the first break"
    assert levels[1:1 + BREAK_TX] == [0] * BREAK_TX, "a break"
    assert levels[1 + BREAK_TX:1 + BREAK_TX + MAB_TX] == [1] * MAB_TX, "then the MAB"
    assert len(levels) == frame.end_tick + 1 == frame.wire_ticks
    assert chk.clean and chk.min["profile"] == "receive"


class Probe(Device):
    """Records the level the bus shows every tick, so a test can look at the wire."""

    name = "probe"

    def __init__(self, net="d"):
        self.net = net
        self.levels: list[int] = []

    def tick(self, t, pins):
        self.levels.append(pins[self.net])
        return {}


def test_two_controllers_on_one_link_is_contention():
    """Clause 4.3: a data link shall consist of a single active differential line driver."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    one = attach_tx(bus, name="dmx_a")
    two = attach_tx(bus, name="dmx_b")
    chk = attach_checker(bus)
    one.queue_packet([0x55])
    two.queue_packet([0x00])
    one.run(bus, limit=400_000)
    chk.finish()

    assert bus.errors, "two drivers on a push-pull net have to show up as contention"
    assert {c.net for c in bus.errors} == {"d"}
    assert {tuple(sorted(c.drivers)) for c in bus.errors} == {("dmx_a", "dmx_b")}
    # Both controllers still finished their packets. The wire is their wired-AND, which for
    # these two packets decodes as one legal packet, so nothing here is a timing violation --
    # which is exactly why the standard allows only one driver per link and why the core
    # bus, not this checker, is the place that catches it.
    assert one.idle and two.idle and len(one.tx_log) == 1 and len(two.tx_log) == 1
    assert [f.slots for f in one.tx_log] == [2] == [f.slots for f in two.tx_log]
    # ... and the wire is their wired-AND: one packet with one data slot, because slot 1 is
    # 0x55 on one side and 0x00 on the other, which is 0x00 on the wire
    assert [p.slots for p in chk.packets] == [1]
    assert chk.packets[0].start_code == NULL_START_CODE


def test_the_padded_packet_time_is_the_refresh_rate_the_tables_quote():
    """The mark-before-break padding lands exactly on the tables' 830 Hz refresh rate.

    The smallest packet the tables allow, 1204 us of break-to-break, is 1 / 1.204 ms =
    830.6 packets per second, which is the "approximately 830 Hz" the note under tables 6
    and 7 gives as the fastest a transmitter may run; the receiver row says 836 Hz. A
    controller that padded to less would break 8.10, which is what `pad_break_to_break`
    exists to prevent.
    """
    _bus, _tx, _rx, chk = run_packets({}, [packet([0x55]), packet([0xAA])])
    starts = [p.break_tick for p in chk.packets]
    period = starts[1] - starts[0]
    assert period == B2B_TX - 1, "1204 us, less the one tick between two packets"
    assert 1 / (period * NS_PER_TICK / 1e9) == pytest.approx(830.6, abs=0.5)
    assert chk.clean, chk.report()


def test_loss_of_data_after_one_maximum_break_to_break():
    """Clause 9.2: no reset sequence within one maximum break-to-break time is loss of data.

    The tables' maximum is 1.25 s, which is 62.5 million ticks at 20 ns/tick, so the receiver
    is told a smaller maximum here and the rule is exercised against the same arithmetic: it
    is the same comparison, and `data_lost()` asks when the receiver considers itself lost.
    """
    bus, tx, rx, chk = run_packets({"rx_timing": {"b2b_max": 100_000}}, [])
    assert rx.data_lost(0), "a receiver that has seen nothing has lost its input"
    tx.queue_packet([0x55])
    tx.queue_packet([0xAA])
    settle(bus, tx, chk, rx)
    assert rx.lost_data == 1, "one 1204 us gap is over the 100 us this receiver allows"
    # the log is (tick of the late reset sequence, how late it was), and the receiver
    # measures from reset sequence to reset sequence: the MAB begins where the break ends
    first_reset, second_reset = BREAK_TX, chk.packets[1].break_tick + BREAK_TX
    assert rx.lost_data_log == [(second_reset, second_reset - first_reset - 5_000)]
    assert rx.data_lost(rx._last_reset + 4_999) is False, "still inside the window"
    assert rx.data_lost(rx._last_reset + 5_001) is True, "past it"
    assert not chk.violations, "the wire itself is legal; it is the receiver that gave up"


def test_a_receiver_whose_window_is_long_enough_never_loses_data():
    """The same run on a default receiver: 1.25 s is far longer than the gap between packets."""
    _bus, tx, rx, chk = run_packets({}, [packet([0x55]), packet([0xAA])])
    assert rx.lost_data == 0 and not rx.lost_data_log
    assert rx.data_lost(rx._last_reset + 62_499_999) is False
    assert rx.data_lost(rx._last_reset + 62_500_001) is True


def test_per_packet_timing_overrides_only_affect_their_own_packet():
    """A controller can be off-spec on one packet and legal on the next, and the checker
    says which: the violation lands inside the packet that carries the fault."""
    packets = [packet([0x55, 0xAA]), packet([0x55, 0xAA], timing={"mab": 2_000}),
               packet([0x55, 0xAA])]
    _bus, tx, _rx, chk = run_packets({}, packets)

    assert only(chk) == {"MAB"}, chk.report()
    at = chk.violations[0]
    first_break, second_break = (p.break_tick for p in chk.packets[:2])
    assert second_break < at.tick < chk.packets[2].break_tick, \
        "the fault is inside the second packet and nowhere else"
    assert tx.tx_log[1].mab_ticks == 100
    assert [f.mab_ticks for f in tx.tx_log] == [MAB_TX, 100, MAB_TX]
    assert first_break == 0


def test_the_checker_only_closes_a_packet_when_the_next_one_starts():
    """The contract a test has to know: the last packet needs `finish()`.

    A packet is only known to be over when the next BREAK arrives, so the checker holds the
    last one open. Calling `finish()` twice is harmless, which is what lets `settle()` be
    written the way it is.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_tx(bus)
    chk = attach_checker(bus)
    tx.queue_packet([0x55], faults={"stop1_low": 1})
    assert tx.run(bus, limit=400_000)
    assert chk.violations == [] and chk.packets == [], "the packet is still open"
    assert chk.finish() is not None and chk.finish() is None
    assert rules(chk) == {"STOP1"}
    assert len(chk.packets) == 1


def test_receiver_latency_moves_the_ticks_it_reports():
    """`latency` is the observation delay of where the line comes from.

    On this bus a driver changes the line one tick after it decides to (core.py's two-phase
    tick), so the default of 1 reports the tick the transmitter really drove. A receiver fed
    from RTL through verif/vip/cocotb_bridge.py sees a different delay and can be told.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    tx = attach_tx(bus)
    rx = attach_rx(bus, address=1, footprint=1, latency=0)
    chk = attach_checker(bus, latency=0)
    tx.queue_packet([0x55])
    settle(bus, tx, chk, rx)

    assert rx.packets[0].start_tick == BREAK_TX + 1, "one tick later than it really happened"
    assert chk.packets[0].break_tick == 1
    assert (chk.packets[0].break_ticks, chk.packets[0].mab_ticks) == (BREAK_TX, MAB_TX)
    # the durations do not move, only the tick they start on
    assert chk.clean, chk.report()
    with pytest.raises(ValueError):
        attach_rx(Bus(ns_per_tick=NS_PER_TICK), latency=-1)


def test_the_checker_is_not_vacuous():
    """Legal traffic judged against tighter limits must fail, on the rules it names.

    The checker has to be reading its limits from the profile it was given rather than from
    constants of its own, so a bench that is clean against the tables must fail against a
    checker asked for twice the break, five times the MAB and a much narrower bit window.
    """
    values = [0x55, 0xAA, 0x00, 0xFF]
    _bus, _tx, _rx, chk = run_packets({}, [packet(values)])
    assert chk.clean, chk.report()

    # a break of 92 us is not the 200 us this checker asks for, and without a break there is
    # no packet, so the other rules have nothing to say
    _bus, _tx, _rx, tight = run_packets({"check_timing": {"break": 200_000}},
                                         [packet(values)])
    assert rules(tight) == {"BREAK"}, tight.report()
    assert all(v.tick >= 0 for v in tight.violations)
    assert tight.violations_by_rule("BREAK")[0].required_ns == 200_000.0

    # with the break left alone, the MAB and the bit window bite
    _bus, _tx, _rx, tighter = run_packets(
        {"check_timing": {"mab": 40_000, "bit_rate_min": 251e3, "bit_rate_max": 253e3}},
        [packet(values)])
    assert rules(tighter) == {"MAB", "BIT-RATE"}, tighter.report()
    assert tighter.violations_by_rule("MAB")[0].required_ns == 40_000.0
    assert tighter.violations_by_rule("MAB")[0].measured_ns == 12_000.0
    # 251 .. 253 kbit/s is a window narrower than one tick of bit rate at 20 ns/tick, so
    # 250 kbit/s falls out of it
    assert tighter.violations_by_rule("BIT-RATE")[0].required_ns == pytest.approx(3_984.0,
                                                                                  rel=0.001)

    # and the same wire is clean against the transmitter's own table, which it does meet
    _bus, _tx, _rx, strict = run_packets({"strict_tx": True}, [packet(values)])
    assert strict.clean, strict.report()
    assert strict.min["profile"] == "transmit" and strict.min["break"] == 4600
    assert strict.min["mab"] == 600 and strict.min["b2b_min"] == B2B_TX
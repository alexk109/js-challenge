"""Self-tests for the pin-level UART reference models (verif/vip/uart.py).

Run with `make vip-selftest VIP=uart` (or VIP=all).  Everything here checks the Python
models, not RTL: the point is that firmware and RTL can later be compared against a
frame-by-frame golden model, so the golden model has to be right first.
"""
from __future__ import annotations

import math
import random

import pytest

from verif.vip.core import Bus, Device
from verif.vip.uart import (
    MIN_TICKS_PER_BIT,
    RxByte,
    UartRx,
    UartTx,
    frame_bits,
    frame_levels,
    parity_bit,
)

# 4 is the lowest ticks-per-bit the model supports; 434.03 is 115200 baud at 50 MHz (50e6/115200).
TICKS_PER_BIT = [4, 8, 16.5, 434.03]
PARITIES = ["none", "even", "odd"]
STOP_BITS = [1, 2]


# --------------------------------------------------------------------------- helpers
def loopback(tpb, data_bits=8, parity="none", stop_bits=1, idle_bits=0.0, seed=1,
             count=8, values=None):
    """A bus with a transmitter and a receiver on one net and `count` random bytes queued."""
    if values is None:
        rng = random.Random(seed)
        values = [rng.randrange(1 << data_bits) for _ in range(count)]
    bus = Bus(ns_per_tick=20.0, trace=False)
    bus.add_net("tx", push_pull=True)
    tx = UartTx("tx", tpb, data_bits, parity, stop_bits, idle_bits)
    rx = UartRx("tx", tpb, data_bits, parity, stop_bits)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(values)
    n = frame_bits(data_bits, parity, stop_bits)
    limit = int(len(values) * (n + idle_bits) * tpb) + int(8 * tpb) + 1000
    assert bus.run_until(lambda b: not tx.busy and rx.pending == len(values), limit)
    return bus, tx, rx, values


class Glitch(Device):
    """Fault injection: pulls the net low for a while, then releases it."""

    name = "glitch"

    def __init__(self, net: str, start_tick: int, ticks: int):
        self.net = net
        self.start_tick = start_tick
        self.ticks = ticks

    def tick(self, t, pins):
        if self.start_tick <= t < self.start_tick + self.ticks:
            return {self.net: 0}
        return {}


class BrokenTx(UartTx):
    """A transmitter with one deliberately wrong bit, to prove the receiver flags it.

    `wrong` is an index into frame_levels(); None means a healthy transmitter, so one
    instance can be healthy for the first byte and broken for the rest.
    """

    def __init__(self, *args, wrong=-1, **kwargs):
        super().__init__(*args, **kwargs)
        self.wrong = wrong

    def frame_levels(self, byte):
        levels = super().frame_levels(byte)
        if self.wrong is not None:
            levels[self.wrong] ^= 1
        return levels


def clean(byte: RxByte) -> bool:
    return not byte.framing_error and not byte.parity_error


# --------------------------------------------------------------------- the frame shape
def test_frame_bits():
    assert frame_bits() == 10                              # 8N1
    assert frame_bits(8, "even") == 11
    assert frame_bits(8, "odd", 2) == 12
    assert frame_bits(5) == 7
    assert frame_bits(9, "even", 2) == 13


def test_frame_levels_of_a_known_byte():
    # 0xA3 = 1010_0011, four ones, so even parity adds a 0.
    assert frame_levels(0xA3, 8, "even") == [0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1]
    assert frame_levels(0x01, 5) == [0, 1, 0, 0, 0, 0, 1]
    assert frame_levels(0xFF, 5) == frame_levels(0x1F, 5)      # masked to data_bits
    assert frame_levels(0x55, 8, "none", 2)[-2:] == [1, 1]     # two stop bits
    for value in (0x00, 0x5A, 0xFF, 0x1FF):                    # odd inverts even
        for bits in (5, 8, 9):
            assert parity_bit(value, bits, "odd") == 1 - parity_bit(value, bits, "even")
    with pytest.raises(ValueError):
        parity_bit(0x55, 8, "none")


@pytest.mark.parametrize("parity", ["even", "odd"])
@pytest.mark.parametrize("data_bits", [5, 8, 9])
def test_frame_levels_parity_matches_the_mode(parity, data_bits):
    for value in range(1 << data_bits):
        levels = frame_levels(value, data_bits, parity, 1)
        ones = sum(levels[1:1 + data_bits + 1])       # data bits plus the parity bit
        assert ones % 2 == (0 if parity == "even" else 1)


def test_bad_configuration_is_rejected():
    for device in (UartTx, UartRx):
        with pytest.raises(ValueError):                       # below the 4-tick floor
            device("tx", ticks_per_bit=MIN_TICKS_PER_BIT - 0.01)
        for bits in (4, 10):
            with pytest.raises(ValueError):
                device("tx", data_bits=bits)
        with pytest.raises(ValueError):
            device("tx", parity="maybe")
        for stops in (0, 3):
            with pytest.raises(ValueError):
                device("tx", stop_bits=stops)
    with pytest.raises(ValueError):
        UartRx("tx", latency=-1)


def test_the_line_idles_high():
    bus = Bus(trace=True)
    bus.add_net("tx", push_pull=True)
    tx = UartTx("tx", 8.0)
    rx = UartRx("tx", 8.0)
    bus.attach(tx)
    bus.attach(rx)
    bus.run(1000)
    assert all(values["tx"] == 1 for _, values in bus.trace)
    assert tx.idle and tx.pending == 0 and rx.pending == 0 and rx.received == []
    assert rx.edges == [] and rx.worst_dev_ticks == 0.0 and bus.errors == []


# ------------------------------------------------------------------------------ loopback
@pytest.mark.parametrize("tpb", TICKS_PER_BIT)
@pytest.mark.parametrize("parity", PARITIES)
@pytest.mark.parametrize("stop_bits", STOP_BITS)
def test_loopback_random_bytes(tpb, parity, stop_bits):
    bus, tx, rx, values = loopback(tpb, 8, parity, stop_bits, seed=int(tpb * 100) + stop_bits)
    assert bus.errors == [], "transmitter and receiver must never fight on the net"
    assert tx.sent == len(values) and tx.idle and tx.pending == 0
    assert rx.pending == len(values)
    assert rx.values == values
    assert all(clean(b) for b in rx.received)


@pytest.mark.parametrize("tpb", TICKS_PER_BIT)
@pytest.mark.parametrize("data_bits", [5, 6, 7, 8, 9])
def test_loopback_every_data_bit_count(tpb, data_bits):
    bus, tx, rx, values = loopback(tpb, data_bits, "even", 2, seed=data_bits)
    assert rx.values == values
    assert all(clean(b) for b in rx.received)
    assert bus.errors == []


@pytest.mark.parametrize("parity", PARITIES)
def test_every_byte_value_round_trips(parity):
    values = list(range(256))
    bus, tx, rx, _ = loopback(8.0, 8, parity, 1, values=values)
    assert rx.values == values
    assert rx.parity_errors == 0 and rx.framing_errors == 0
    assert [b.tick for b in rx.received] == [f.start_tick for f in tx.tx_log]


def test_loopback_with_an_idle_gap():
    bus, tx, rx, values = loopback(16.5, 8, "odd", 1, idle_bits=2.0, count=4)
    assert rx.values == values
    n = frame_bits(8, "odd", 1)
    for a, b in zip(tx.tx_log, tx.tx_log[1:]):
        assert abs((b.start_tick - a.start_tick) - (n + 2) * 16.5) < 1.0


def test_send_takes_ints_and_iterables():
    bus = Bus()
    bus.add_net("tx")
    tx = UartTx("tx", 4.0)
    rx = UartRx("tx", 4.0)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(0x31).send(b"\x32\x33", [0x34], 0x35)
    assert tx.pending == 5
    assert bus.run_until(lambda b: rx.pending == 5, 1000)
    assert rx.values == [0x31, 0x32, 0x33, 0x34, 0x35]


def test_pop_hands_out_each_byte_once():
    bus = Bus()
    bus.add_net("tx")
    tx = UartTx("tx", 4.0)
    rx = UartRx("tx", 4.0)
    bus.attach(tx)
    bus.attach(rx)
    assert rx.pop() is None
    tx.send([1, 2, 3])
    assert bus.run_until(lambda b: rx.pending == 3, 1000)
    assert [rx.pop().value for _ in range(3)] == [1, 2, 3]
    assert rx.pop() is None
    assert rx.pending == 0 and len(rx.received) == 3        # the log keeps everything


def test_start_ticks_are_the_ticks_the_transmitter_drove():
    """The receiver takes its observation latency out, so both agree on the grid."""
    bus, tx, rx, values = loopback(434.03, 8, "even", 1, count=6)
    assert [b.tick for b in rx.received] == [f.start_tick for f in tx.tx_log]
    n = frame_bits(8, "even", 1)
    for a, b in zip(tx.tx_log, tx.tx_log[1:]):
        assert abs((b.start_tick - a.start_tick) - n * 434.03) < 1.0


def test_a_receiver_attached_mid_frame_resynchronises_before_the_next_byte():
    """Documented consequence of hunting on a falling edge: put the receiver up first.

    What the partial frame decodes to is anyone's guess; what matters is that the
    receiver is clean again by the time the next frame starts."""
    bus = Bus()
    bus.add_net("tx")
    tx = UartTx("tx", 8.0)
    bus.attach(tx)
    tx.send(0x55)
    bus.run(20)                       # the first frame is half way out
    rx = UartRx("tx", 8.0)
    bus.attach(rx)
    bus.run(400)
    assert rx.received                 # the partial frame decoded into something
    before = len(rx.received)
    tx.send(0x66)                      # after an idle period the line is healthy again
    assert bus.run_until(lambda b: len(rx.received) == before + 1, 1000)
    assert rx.values[-1] == 0x66 and clean(rx.received[-1])


# ---------------------------------------------------------------------- framing errors
@pytest.mark.parametrize("tpb", TICKS_PER_BIT)
@pytest.mark.parametrize("parity", PARITIES)
@pytest.mark.parametrize("stop_bits", STOP_BITS)
def test_framing_error_when_a_stop_bit_is_low(tpb, parity, stop_bits):
    """One idle bit between frames, as the RTL stub sends it: every byte is flagged."""
    rng = random.Random(7)
    values = [rng.randrange(256) for _ in range(4)]
    bus = Bus()
    bus.add_net("tx")
    tx = BrokenTx("tx", tpb, 8, parity, stop_bits, 1.0, wrong=-1)  # last stop bit low
    rx = UartRx("tx", tpb, 8, parity, stop_bits)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(values)
    n = frame_bits(8, parity, stop_bits)
    assert bus.run_until(lambda b: rx.pending == len(values),
                         int(len(values) * (n + 2) * tpb) + 1000)
    assert rx.values == values
    assert rx.framing_errors == len(values)
    assert rx.parity_errors == 0
    assert all(b.framing_error for b in rx.received)


def test_a_stop_bit_stuck_low_without_an_idle_gap_loses_frames():
    """The line never goes high, so the next start bit has no edge to be found on."""
    values = [0xA5, 0x4D, 0xCA, 0x18]
    bus = Bus()
    bus.add_net("tx")
    tx = BrokenTx("tx", 8.0, 8, "none", 1, 0.0, wrong=-1)
    rx = UartRx("tx", 8.0, 8, "none", 1)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(values)
    assert bus.run_until(lambda b: not tx.busy and rx.pending >= 1, 1000)
    assert rx.framing_errors >= 1
    assert len(rx.received) < len(values)
    assert rx.values != values


def test_the_receiver_resynchronises_after_a_framing_error():
    """With a healthy idle gap between frames the line goes high and hunting resumes."""
    bus = Bus()
    bus.add_net("tx")
    tx = BrokenTx("tx", 8.0, 8, "none", 1, idle_bits=2.0, wrong=None)
    rx = UartRx("tx", 8.0, 8, "none", 1)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(0x00)
    bus.run(20)                        # the first byte is fine and already in flight
    tx.wrong = -1                      # from here on the stop bit is broken
    tx.send(0x11, 0x22)
    assert bus.run_until(lambda b: rx.pending == 3, 2000)
    assert rx.values == [0x00, 0x11, 0x22]
    assert not rx.received[0].framing_error
    assert rx.framing_errors == 2 and rx.parity_errors == 0


def test_a_line_stuck_low_does_not_decode_a_stream():
    """A break: one garbled byte when the receiver starts hunting, then nothing."""
    bus = Bus()
    bus.add_net("tx", pullup=0)
    rx = UartRx("tx", 8.0)
    bus.attach(rx)
    bus.run(5000)
    assert len(rx.received) <= 1        # it hunts once on the low line, then waits
    count = len(rx.received)
    bus.run(20_000)
    assert len(rx.received) == count


# ----------------------------------------------------------------------- parity errors
@pytest.mark.parametrize("tpb", [4, 16.5, 434.03])
@pytest.mark.parametrize("parity", ["even", "odd"])
def test_parity_error_when_the_parity_bit_is_flipped(tpb, parity):
    values = [0x00, 0xFF, 0x5A, 0xA5]
    bus = Bus()
    bus.add_net("tx")
    tx = BrokenTx("tx", tpb, 8, parity, 1, wrong=9)        # the parity bit
    rx = UartRx("tx", tpb, 8, parity, 1)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(values)
    assert bus.run_until(lambda b: rx.pending == len(values), int(len(values) * 13 * tpb) + 200)
    assert rx.values == values
    assert rx.parity_errors == len(values)
    assert rx.framing_errors == 0


def test_parity_error_when_the_receiver_expects_the_other_parity():
    values = [0x00, 0xFF, 0x5A, 0xA5]
    bus = Bus()
    bus.add_net("tx")
    tx = UartTx("tx", 16.0, 8, "even", 1)
    rx = UartRx("tx", 16.0, 8, "odd", 1)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(values)
    assert bus.run_until(lambda b: rx.pending == len(values), 2000)
    assert rx.values == values
    assert rx.parity_errors == len(values)
    assert rx.framing_errors == 0


# ------------------------------------------------------------------------------ glitches
@pytest.mark.parametrize("tpb", TICKS_PER_BIT)
def test_a_glitch_shorter_than_half_a_bit_makes_no_byte(tpb):
    width = max(1, int(tpb // 2) - 1)          # strictly shorter than half a bit
    assert width < tpb / 2
    bus = Bus()
    bus.add_net("tx")
    tx = UartTx("tx", tpb)
    rx = UartRx("tx", tpb)
    bus.attach(tx)
    bus.attach(rx)
    bus.attach(Glitch("tx", start_tick=5, ticks=width))
    bus.run(int(4 * tpb) + 100)
    assert rx.received == [] and rx.pending == 0 and rx.aborted == 1
    # the line is healthy again: the next byte still arrives
    tx.send(0x5A)
    assert bus.run_until(lambda b: rx.pending == 1, int(20 * tpb) + 100)
    assert rx.values == [0x5A] and clean(rx.received[0])


@pytest.mark.parametrize("tpb", [8, 16.5, 434.03])
def test_a_pulse_past_the_mid_bit_sample_is_taken_as_a_start_bit(tpb):
    """The other side of the threshold: the low pulse outlives the sampling instant."""
    width = math.ceil(tpb / 2) + 2           # plus the tick of sampling latency
    bus = Bus()
    bus.add_net("tx")
    rx = UartRx("tx", tpb)
    bus.attach(rx)
    bus.attach(Glitch("tx", start_tick=5, ticks=width))
    bus.run(int(12 * tpb) + 200)
    assert rx.aborted == 0 and rx.pending == 1
    assert rx.values == [0xFF]                # the line was high for every later bit


# ------------------------------------------------------------------- timing measurement
@pytest.mark.parametrize("tpb", TICKS_PER_BIT)
@pytest.mark.parametrize("parity", PARITIES)
@pytest.mark.parametrize("stop_bits", STOP_BITS)
def test_worst_case_edge_deviation_of_the_transmitter(tpb, parity, stop_bits):
    bus, tx, rx, values = loopback(tpb, 8, parity, stop_bits, count=12, seed=3)
    assert rx.edges, "no edge was measured, the check below would be vacuous"
    assert rx.worst_dev_ticks < 1.0, (
        f"UartTx drifted: worst edge deviation {rx.worst_dev_ticks} ticks at "
        f"ticks_per_bit={tpb}, deviations {sorted(round(e.dev, 4) for e in rx.edges)}"
    )
    assert all(abs(e.dev) < 1.0 for e in rx.edges)
    assert bus.errors == []


@pytest.mark.parametrize("tpb", [4, 4.0001, 5.25, 13.7, 100.01, 434.027, 1000.9])
def test_awkward_fractional_divisors_stay_inside_one_tick(tpb):
    """The phase accumulator, not rounding, is what keeps the drift under a tick."""
    bus, tx, rx, values = loopback(tpb, 8, "even", 2, count=6, seed=5)
    assert rx.values == values
    assert all(clean(b) for b in rx.received)
    assert rx.edges and rx.worst_dev_ticks < 1.0
    n = frame_bits(8, "even", 2)
    for a, b in zip(tx.tx_log, tx.tx_log[1:]):
        assert abs((b.start_tick - a.start_tick) - n * tpb) < 1.0


def test_edge_deviation_catches_a_transmitter_at_the_wrong_baud_rate():
    """3% fast: the error has to add up past a tick within two bits to be caught."""
    bus = Bus()
    bus.add_net("tx")
    tx = UartTx("tx", 16.0)
    rx = UartRx("tx", 16.5)
    bus.attach(tx)
    bus.attach(rx)
    tx.send([0x55, 0xAA, 0x55, 0xAA, 0x55])   # every data bit is a transition to measure
    bus.run(int(5 * 10 * 16.5) + 100)
    assert len(rx.edges) > 10
    assert rx.worst_dev_ticks >= 1.0
    assert rx.min_dev_ticks <= -1.0           # every edge is early: the transmitter is fast
    assert rx.max_dev_ticks == 0.0            # only each frame's own start bit is on time


def test_ticks_per_bit_is_settable_after_construction():
    bus = Bus()
    bus.add_net("tx")
    tx = UartTx("tx", 4.0)
    rx = UartRx("tx", 4.0)
    bus.attach(tx)
    bus.attach(rx)
    tx.ticks_per_bit = 434.03
    rx.ticks_per_bit = 434.03
    assert tx.ticks_per_bit == 434.03 and rx.ticks_per_bit == 434.03
    with pytest.raises(ValueError):
        tx.ticks_per_bit = 2.0
    with pytest.raises(ValueError):
        rx.ticks_per_bit = 2.0
    tx.send(b"AB")
    assert bus.run_until(lambda b: rx.pending == 2, 20_000)
    assert rx.values == list(b"AB")
    assert rx.worst_dev_ticks < 1.0
"""fw/uart_rx: the 8N1 receiver end to end on the golden ISS.

`iss.pio_iss.ChipDevice` sits on a `verif.vip.core.Bus` at 20 ns per tick with the protocol
pins as the nets `p0`..`p17`, the program of `fw/uart_rx/program.pio` runs on engine 0, and
`verif.vip.uart.UartTx` drives the RX net.  The UART models are the judge: they are never
imported from here for anything but the stimulus, and nothing below weakens a check -- a
glitch test that passes because the receiver never saw the frame would fail
`test_a_pulse_longer_than_half_a_bit_is_not_rejected`, and every test asserts the byte values,
not just the counts.

Set `FW_TARGET=iss` (the default) to run them; `make fw PROTO=uart_rx TARGET=iss` does it.
`FW_TARGET=rtl` runs the same tests against `src/pio_chip.sv` (`fw/rtl_harness.py`).
"""
from __future__ import annotations


import pytest

from verif.vip.core import Device
from verif.vip.uart import UartTx

from fw.uart_rx.driver import DBG_CFGINFO, UartRx, build_program
from fw import regmap as _regmap

RX_PIN = 0
BAUD = 115200


# ---------------------------------------------------------------------------------------------
# Line models the UART VIP does not provide
# ---------------------------------------------------------------------------------------------
class LowPulses(Device):
    """Pulls a net low inside fixed tick windows: line noise, and a break.

    Drives 0 for every tick of a window whatever else is on the net (the bus is wired-AND), so
    a pulse lasts exactly as long as its window; the tests keep the windows clear of the frames
    the transmitter sends and check that they are.
    """

    def __init__(self, net: str, windows) -> None:
        self.net = net
        self.name = f"low[{net}]"
        self.windows = [(int(s), int(e)) for s, e in windows]

    def tick(self, t, pins):
        for start, end in self.windows:
            if start <= t < end:
                return {self.net: 0}
        return {}

    def overlaps(self, start: int, end: int) -> bool:
        """True if any window overlaps the tick range [start, end)."""
        return any(not (end <= s or start >= e) for s, e in self.windows)


class BrokenStopTx(UartTx):
    """A UartTx whose stop bit is low for the bytes in `broken`.

    This is the fault the VIP documents (override `frame_levels`, the function it drives), and
    it is what makes the receiver's framing-error path run.
    """

    def __init__(self, *args, broken=(), **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.broken = set(broken)

    def frame_levels(self, byte: int):
        levels = super().frame_levels(byte)
        if byte in self.broken:
            levels[-1] = 0
        return levels


def transmitter(rx: UartRx, rate: float = 1.0, **kwargs) -> UartTx:
    """A `UartTx` on the receiver's net, running at `rate` times the rate `rx` was built for."""
    return rx.bus.attach(UartTx(rx.net, ticks_per_bit=rx.ticks_per_bit / rate, **kwargs))


def bit_times(rx: UartRx, n: float) -> int:
    return int(n * rx.ticks_per_bit)


def receive(rx: UartRx, tx: UartTx, n_bytes: int, slack_bits: float = 8.0) -> None:
    """Run the bus until `tx` has sent `n_bytes` frames and the host has read them all."""
    def done() -> bool:
        return not tx.busy and tx.sent >= n_bytes and len(rx.received) >= n_bytes

    limit = int((11.0 * n_bytes + slack_bits) * rx.ticks_per_bit)
    assert rx.run_until(done, limit), "the receiver did not deliver every byte in time"


# ---------------------------------------------------------------------------------------------
# The program and the register writes that start it
# ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize("sm_per_bit", [4, 6, 8, 16, 32])
def test_program_assembles_into_imem(sm_per_bit):
    prog = build_program(sm_per_bit)
    assert prog.name == "uart_rx"
    assert len(prog.instrs) <= 32, "IMEM is 32 words per engine"
    assert prog.labels["hunt"] < 32 and prog.wrap_target <= prog.wrap <= 31


def test_start_sequence_writes_the_register_map():
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    assert len(rx.program.instrs) == 24
    # A sanity read of the chip's own configuration register (docs/mvp-regmap.md `DBG_CFGINFO`).
    assert rx.chip.peek(DBG_CFGINFO) == _regmap.active().cfginfo
    eng = rx.chip.engines[0]
    assert rx.chip.sm_enable & 1, "engine 0 was not enabled by CTRL"
    assert eng.clkdiv == (27 << 16) | (32 << 8), "CLKDIV = ticks_per_bit / S, rounded"
    assert eng.execctrl & 0x1F == RX_PIN, "JMP_PIN is the RX pin, in the 32-pin space"
    assert (eng.execctrl >> 12) & 0x1F == rx.program.wrap
    assert (eng.execctrl >> 7) & 0x1F == rx.program.wrap_target
    assert (eng.pinctrl >> 15) & 0x1F == RX_PIN, "IN_BASE is the RX pin"
    assert (eng.pinctrl >> 5) & 0x1F == RX_PIN and (eng.pinctrl >> 26) & 7 == 1, "SET window"
    assert rx.chip.state(0)["pc"] == rx.program.labels["hunt"], "the forced jmp hunt"
    assert not rx.chip.pin_oe & (1 << RX_PIN), "the RX pin is an input (set pindirs, 0)"
    assert rx.chip.fdebug == 0


# ---------------------------------------------------------------------------------------------
# End-to-end transfers
# ---------------------------------------------------------------------------------------------
def test_back_to_back_bytes():
    """Eight frames with no idle bit between them at 115200 baud on pin 0."""
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    tx = transmitter(rx)
    tx.send(b"UART-RX!")
    receive(rx, tx, 8)
    assert rx.values == list(b"UART-RX!"), rx.values
    assert rx.framing_errors == 0 and rx.markers == []
    assert rx.chip.fdebug == 0, "no FIFO underrun, overrun or stall"
    # Eight frames of ten bit times cannot have arrived before this.
    assert rx.bus.t > 80 * rx.ticks_per_bit


@pytest.mark.parametrize("rate", [0.98, 1.00, 1.02])
def test_baud_tolerance_two_percent(rate):
    """The sender may be 2% fast or slow; CLKDIV is set for the nominal rate."""
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    tx = transmitter(rx, rate=rate)
    tx.send(b"\x00\x55\xff\x80")            # a 0 and a run of ones in every position
    receive(rx, tx, 4)
    assert rx.values == [0x00, 0x55, 0xFF, 0x80], f"at {rate}x: {[hex(v) for v in rx.values]}"
    assert rx.framing_errors == 0


@pytest.mark.parametrize("baud,pin", [(38400, 3), (9600, 9), (3125000, 7)])
def test_speeds_and_pins(baud, pin):
    """Three more configurations: a slower baud on another uio pin, on an input-only pin, and
    the fastest the default program can go (one bit time is exactly S = 16 core cycles)."""
    rx = UartRx(baud=baud, rx_pin=pin)
    tx = transmitter(rx)
    payload = bytes(range(0x30, 0x34))
    tx.send(payload)
    receive(rx, tx, 4)
    assert rx.values == list(payload), f"{baud} baud on pin {pin}: {[hex(v) for v in rx.values]}"
    assert rx.framing_errors == 0


def test_fastest_speed_a_smaller_s_reaches():
    """S = 8 SM cycles per bit, so a bit time is 8 core cycles: 6.25 Mbaud at 50 MHz."""
    rx = UartRx(baud=50e6 / 8, sm_cycles_per_bit=8)
    assert rx.max_baud == 6_250_000.0
    tx = transmitter(rx)
    tx.send(b"\xde\xad")
    receive(rx, tx, 2)
    assert rx.values == [0xDE, 0xAD]


def test_a_baud_the_program_cannot_reach_is_refused():
    rx = UartRx(baud=BAUD, sm_cycles_per_bit=16)
    assert rx.max_baud == 50e6 / 16
    with pytest.raises(ValueError, match="fastest this program goes"):
        UartRx(baud=rx.max_baud * 1.01)


# ---------------------------------------------------------------------------------------------
# Line noise: a low pulse shorter than half a bit is not a start bit
# ---------------------------------------------------------------------------------------------
def pulses(rx: UartRx, width: float, count: int = 6, first: float = 8.0,
           span: float = 4.0) -> LowPulses:
    """`count` low pulses of `width` bit times, spaced `span` bit times apart, starting
    `first` bit times from now.  Nothing is queued on the line while they happen."""
    start = rx.bus.t + bit_times(rx, first)
    step = bit_times(rx, span)
    return rx.bus.attach(
        LowPulses(rx.net, [(start + i * step, start + i * step + bit_times(rx, width))
                           for i in range(count)])
    )


@pytest.mark.parametrize("width", [0.1, 0.25, 0.45])
def test_short_pulses_in_idle_are_rejected(width):
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    tx = transmitter(rx)                        # its queue is empty: the line is idle
    noise = pulses(rx, width)
    rx.run(bit_times(rx, 36))                   # the pulses go by, still in idle
    tx.send(b"quiet")
    receive(rx, tx, 5)
    assert rx.values == list(b"quiet"), (
        f"a {width} bit pulse in idle produced {[hex(v) for v in rx.values]}"
    )
    assert rx.framing_errors == 0
    # The pulses really were in idle: none of them overlaps a frame the sender drove.
    for frame in tx.tx_log:
        assert not noise.overlaps(frame.start_tick, frame.end_tick)


@pytest.mark.parametrize("width", [0.6, 0.95])
def test_a_pulse_longer_than_half_a_bit_is_not_rejected(width):
    """The other side of the same edge, so the test above cannot pass by ignoring the line."""
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    tx = transmitter(rx)
    pulses(rx, width, count=1)
    rx.run(bit_times(rx, 12))
    tx.send(b"quiet")
    receive(rx, tx, 5)
    assert rx.values != list(b"quiet"), (
        "a pulse longer than half a bit must be taken for a start bit, as a real receiver "
        "does; if this fails the receiver is not looking at the line at all"
    )


def hunting(rx: UartRx) -> bool:
    """True while the engine sits in a `wait 0 pin, 0` looking for the next start bit."""
    state = rx.chip.state(0)
    return state["stalled"] == 1 and rx.program.instrs[state["pc"]] & 0xFF == 0x20


def test_a_break_is_decoded_and_the_receiver_then_resynchronises():
    """The line held low for two bit times is a frame like any other (here with a framing
    error, because the stop bit lands after the line came back), and because the idle wait
    resynchronises on the next 1 -> 0 transition the frames after it are read again."""
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    tx = transmitter(rx)
    noise = pulses(rx, 2.0, count=1)
    # The frames below must not overlap the frame the receiver takes the break for.
    assert rx.run_until(lambda: len(rx.received) >= 1, bit_times(rx, 40)), "no break frame"
    assert rx.run_until(lambda: hunting(rx), bit_times(rx, 5)), "the receiver is still busy"
    tx.send(b"ok")
    receive(rx, tx, 2)
    assert rx.values[-2:] == [ord("o"), ord("k")], [hex(v) for v in rx.values]
    assert rx.values[0] != ord("o"), "the break itself decoded to something"
    assert not noise.overlaps(tx.tx_log[0].start_tick, tx.tx_log[0].end_tick)


# ---------------------------------------------------------------------------------------------
# Framing errors and back pressure
# ---------------------------------------------------------------------------------------------
def test_framing_error_is_reported_in_the_fifo():
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    # A half-bit gap after the broken frame: the line has to be idle once for the receiver to
    # find the next start bit, which is what any receiver needs after a disturbed frame.
    tx = rx.bus.attach(
        BrokenStopTx(rx.net, ticks_per_bit=rx.ticks_per_bit, broken=(0x41,), idle_bits=0.5)
    )
    tx.send(b"ABC")
    receive(rx, tx, 3)
    assert [(b.value, b.framing_error) for b in rx.received] == [
        (0x41, True),
        (0x42, False),
        (0x43, False),
    ]
    assert rx.markers == [1], "one marker word, after the byte it flags"


def test_good_frames_carry_no_marker():
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    tx = transmitter(rx)
    tx.send(bytes(range(0x00, 0x100, 0x11)))
    receive(rx, tx, 16)
    assert rx.markers == [] and rx.framing_errors == 0


def test_a_full_rx_fifo_stalls_the_engine():
    """Six bytes with the host not reading: the 4-deep RX FIFO fills, `push block` stalls and
    sets FDEBUG.RXSTALL.  The four words it holds and the byte in the ISR survive; the sixth
    byte is lost, because a UART line keeps running while the receiver is stuck -- which is
    the flow-control rule the host has to honour, not a property of this program."""
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN)
    rx.auto_read = False
    tx = transmitter(rx)
    tx.send(b"123456")
    assert rx.run_until(
        lambda: not tx.busy and rx.chip.state(0)["rx_level"] == 4,
        int(11 * 6 * rx.ticks_per_bit),
    )
    assert rx.chip.fdebug & 0x01, "FDEBUG.RXSTALL of engine 0 was not set"
    assert rx.chip.state(0)["stalled"] == 1, "the engine is stalled in the blocking push"
    rx.auto_read = True
    receive(rx, tx, 5)
    assert rx.values == list(b"12345"), rx.values

"""Pin-level UART reference models for the tick bus in verif/vip/core.py.

The frame, LSB first, idle line high, is the only thing these two models speak:

    start(0) | data[data_bits-1..0] | [parity] | stop bits (1)

`data_bits` is 5..9, `parity` is "none", "even" or "odd", `stop_bits` is 1 or 2, and
`ticks_per_bit` is a float >= 4.  A fractional divisor is the normal case, not an
excursion: 115200 baud at 50 MHz is 434.03 ticks/bit, so the transmitter accumulates
phase and each bit lasts floor(ticks_per_bit) or ceil(ticks_per_bit) ticks.  Bit edges
therefore land within one tick of the nominal grid, which is what the receiver measures
and reports as `worst_dev_ticks`.

Two timing conventions, both taken from core.py's docstring:

* A driver that returns a value for net N at tick t has the net read that value from the
  end of tick t on; every device sees it at tick t+1.  The receiver models that as
  `latency` ticks and takes it out of every tick it reports, so `RxByte.tick` and the
  edge log are the ticks a transmitter really drove them.  If the line comes from
  somewhere with a different latency (RTL through verif/vip/cocotb_bridge.py), pass
  that number instead of 1.
* The receiver samples every bit at its centre, in transmitter time: sample k of a frame
  whose start bit began at tick `t0` is taken at `t0 + (k + 0.5) * ticks_per_bit`, plus
  `latency` for the observation delay.  Sampling that late is what rejects a start bit
  shorter than half a bit (a glitch): the sample lands after the glitch and reads high.

The receiver's start-bit detection is a falling edge, as a real receiver's is.  On its very
first tick it takes a low line as a start bit (a real receiver coming up on a busy line
does the same), and from then on it needs a 1->0 transition, so a frame already in progress
when the receiver is attached is picked up from its next low bit rather than from its
start: expect one garbled byte (usually with a framing error) before it resynchronises.

    bus = Bus(ns_per_tick=20)                 # 50 MHz
    bus.add_net("tx", push_pull=True)
    tx = UartTx("tx", ticks_per_bit=434.03)    # 115200 baud
    rx = UartRx("tx", ticks_per_bit=434.03)
    bus.attach(tx)
    bus.attach(rx)
    tx.send(b"hello")
    bus.run_until(lambda b: not tx.busy and rx.pending == 5, 50_000)
    assert [b.value for b in rx.received] == list(b"hello")
    assert rx.worst_dev_ticks < 1.0
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable, Mapping, Optional

from verif.vip.core import Device

PARITIES = ("none", "even", "odd")
MIN_TICKS_PER_BIT = 4.0
IDLE_LEVEL = 1

_EPS = 1e-9        # float slack when comparing accumulated phase against ticks_per_bit


def _check_ticks_per_bit(value: float) -> float:
    tpb = float(value)
    if tpb < MIN_TICKS_PER_BIT:
        raise ValueError(
            f"ticks_per_bit must be >= {MIN_TICKS_PER_BIT} (got {value!r}): at less than "
            "4 ticks per bit a mid-bit sample cannot be told from a glitch"
        )
    return tpb


def _check_data_bits(value: int) -> int:
    bits = int(value)
    if not 5 <= bits <= 9:
        raise ValueError(f"data_bits must be 5..9 (got {value!r})")
    return bits


def _check_parity(value: Optional[str]) -> str:
    name = "none" if value is None else str(value).lower()
    if name not in PARITIES:
        raise ValueError(f"parity must be one of {PARITIES} (got {value!r})")
    return name


def _check_stop_bits(value: int) -> int:
    stops = int(value)
    if stops not in (1, 2):
        raise ValueError(f"stop_bits must be 1 or 2 (got {value!r})")
    return stops


def frame_bits(data_bits: int = 8, parity: str = "none", stop_bits: int = 1) -> int:
    """Bits in one frame: start + data + parity + stops."""
    data_bits = _check_data_bits(data_bits)
    parity = _check_parity(parity)
    stop_bits = _check_stop_bits(stop_bits)
    return 1 + data_bits + (1 if parity != "none" else 0) + stop_bits


def parity_bit(value: int, data_bits: int = 8, parity: str = "none") -> int:
    """The parity bit for `value`: 0 if the frame's 1s should be even, 1 for odd."""
    parity = _check_parity(parity)
    if parity == "none":
        raise ValueError("parity is 'none': there is no parity bit")
    ones = (value & ((1 << _check_data_bits(data_bits)) - 1)).bit_count()
    return (ones % 2) ^ (1 if parity == "odd" else 0)


def frame_levels(
    value: int, data_bits: int = 8, parity: str = "none", stop_bits: int = 1
) -> list[int]:
    """Line levels of one frame, LSB first: start, data, parity, stop bits.

    `value` is masked to `data_bits`.  This is the function UartTx drives; subclass and
    override it to model a transmitter with a fault (see the self-tests).
    """
    data_bits = _check_data_bits(data_bits)
    parity = _check_parity(parity)
    stop_bits = _check_stop_bits(stop_bits)
    word = int(value) & ((1 << data_bits) - 1)
    levels = [0]
    levels += [(word >> i) & 1 for i in range(data_bits)]
    if parity != "none":
        levels.append(parity_bit(word, data_bits, parity))
    levels += [IDLE_LEVEL] * stop_bits
    return levels


@dataclass
class TxFrame:
    """One byte as the transmitter drove it."""

    value: int
    start_tick: int                      # tick the start bit began (this is the drive tick)
    end_tick: Optional[int] = None       # first idle tick after the frame
    levels: tuple[int, ...] = ()

    @property
    def bits(self) -> int:
        return len(self.levels)


class UartTx(Device):
    """Sends queued bytes on one net. Idle high; back-to-back frames by default.

    `ticks_per_bit` is a float and is accumulated, not rounded per bit, so a fractional
    divisor drifts by less than a tick over any number of bits (see the module docstring).
    `idle_bits` (default 0) holds the line high for that many bit times between frames.
    """

    def __init__(
        self,
        net: str = "tx",
        ticks_per_bit: float = 16.0,
        data_bits: int = 8,
        parity: str = "none",
        stop_bits: int = 1,
        idle_bits: float = 0.0,
        name: Optional[str] = None,
    ):
        self.net = net
        self.name = name or f"uarttx[{net}]"
        self._tpb = _check_ticks_per_bit(ticks_per_bit)
        self.data_bits = _check_data_bits(data_bits)
        self.parity = _check_parity(parity)
        self.stop_bits = _check_stop_bits(stop_bits)
        self.idle_bits = float(idle_bits)
        self.queue: deque[int] = deque()
        self.tx_log: list[TxFrame] = []
        self._levels: Optional[list[int]] = None
        self._index = 0
        self._phase = 0.0      # ticks elapsed in the current bit, at the start of this tick
        self._idle_until = 0    # tick before which no new frame may start

    # -- configuration -------------------------------------------------------
    @property
    def ticks_per_bit(self) -> float:
        return self._tpb

    @ticks_per_bit.setter
    def ticks_per_bit(self, value: float) -> None:
        self._tpb = _check_ticks_per_bit(value)

    # -- queue ---------------------------------------------------------------
    def send(self, *data: int | Iterable[int]) -> "UartTx":
        """Queue bytes: send(0x55), send(b"abc"), send([1, 2], 3), send(range(4))."""
        for item in data:
            if isinstance(item, int):
                self.queue.append(item)
            else:                       # bytes, bytearray, list, tuple, range, ...
                self.queue.extend(item)
        return self

    @property
    def pending(self) -> int:
        """Bytes queued but not started yet."""
        return len(self.queue)

    @property
    def busy(self) -> bool:
        return self._levels is not None or bool(self.queue)

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def sent(self) -> int:
        """Frames finished so far."""
        return sum(1 for f in self.tx_log if f.end_tick is not None)

    def frame_levels(self, byte: int) -> list[int]:
        """Levels for one frame; override to model a faulty transmitter."""
        return frame_levels(byte, self.data_bits, self.parity, self.stop_bits)

    # -- the device ----------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        levels = self._levels
        if levels is not None and self._phase >= self._tpb - _EPS:
            self._phase -= self._tpb
            self._index += 1
            if self._index >= len(levels):
                levels = self._end_frame(t)
        if levels is None and self.queue and t >= self._idle_until:
            levels = self._begin(t)          # back-to-back: start on this very tick
        if levels is not None:
            self._phase += 1.0
        if levels is None:
            return {self.net: IDLE_LEVEL}
        return {self.net: levels[self._index]}

    def _begin(self, t: int) -> list[int]:
        value = self.queue.popleft()
        levels = self.frame_levels(value)
        self._levels = levels
        self._index = 0
        self._phase = 0.0
        self.tx_log.append(TxFrame(value=value, start_tick=t, levels=tuple(levels)))
        return levels

    def _end_frame(self, t: int) -> None:
        self.tx_log[-1].end_tick = t
        self._levels = None
        self._index = 0
        self._phase = 0.0
        self._idle_until = t + self.idle_bits * self._tpb


@dataclass
class RxByte:
    """One byte as the receiver decoded it."""

    value: int
    tick: int                      # tick the start bit began (latency already removed)
    framing_error: bool = False    # a stop bit was low
    parity_error: bool = False     # the parity bit did not match the data bits
    levels: tuple[int, ...] = ()   # sampled levels, start bit first, for debugging


@dataclass
class Edge:
    """One measured transition on the line."""

    tick: int       # tick of the edge, latency already removed
    bit: int        # which bit boundary of the frame it landed on
    nominal: float  # where that boundary is on the nominal grid
    dev: float      # tick - nominal, in ticks


class UartRx(Device):
    """Receives on one net and decodes frames. Passive: it never drives.

    Start-bit detection is a falling edge (the first tick it sees a low line it hunts on
    that, like a real receiver); every bit is sampled at its centre.  Each decoded byte
    carries its framing and parity error flags and the tick its start bit began.  Every
    transition inside a frame is measured against the nominal grid
    (`start + k * ticks_per_bit`) and the worst absolute deviation is kept in
    `worst_dev_ticks`, so a test can assert the timing accuracy of a transmitter under
    test.  Transitions while no frame is being decoded are not measured: there is no
    grid to measure them against.
    """

    def __init__(
        self,
        net: str = "rx",
        ticks_per_bit: float = 16.0,
        data_bits: int = 8,
        parity: str = "none",
        stop_bits: int = 1,
        latency: int = 1,
        name: Optional[str] = None,
    ):
        self.net = net
        self.name = name or f"uartrx[{net}]"
        self._tpb = _check_ticks_per_bit(ticks_per_bit)
        self.data_bits = _check_data_bits(data_bits)
        self.parity = _check_parity(parity)
        self.stop_bits = _check_stop_bits(stop_bits)
        self.latency = int(latency)
        if self.latency < 0:
            raise ValueError(f"latency must be >= 0 (got {latency!r})")
        self.received: list[RxByte] = []
        self.edges: list[Edge] = []
        self.framing_errors = 0
        self.parity_errors = 0
        self.aborted = 0        # frames whose start bit was high when sampled (glitches)
        self.worst_dev_ticks = 0.0
        self.min_dev_ticks = 0.0
        self.max_dev_ticks = 0.0
        self._n_bits = frame_bits(self.data_bits, self.parity, self.stop_bits)
        self._parity_k = self.data_bits + 1 if self.parity != "none" else -1
        self._in_frame = False
        self._last = IDLE_LEVEL
        self._origin = 0          # tick the start bit began, latency removed
        self._next_k = 0          # next bit boundary on the nominal grid
        self._k = 0               # next sample to take
        self._sample_at = 0.0     # tick of that sample, in observation time
        self._word = 0
        self._levels_seen: list[int] = []
        self._framing = False
        self._parity_bad = False
        self._consumed = 0

    # -- configuration -------------------------------------------------------
    @property
    def ticks_per_bit(self) -> float:
        return self._tpb

    @ticks_per_bit.setter
    def ticks_per_bit(self, value: float) -> None:
        self._tpb = _check_ticks_per_bit(value)
        self._n_bits = frame_bits(self.data_bits, self.parity, self.stop_bits)
        self._parity_k = self.data_bits + 1 if self.parity != "none" else -1

    # -- the log -------------------------------------------------------------
    @property
    def pending(self) -> int:
        """Bytes decoded but not yet popped."""
        return len(self.received) - self._consumed

    @property
    def values(self) -> list[int]:
        """Values of every decoded byte, in order."""
        return [b.value for b in self.received]

    def pop(self) -> Optional[RxByte]:
        """Oldest byte not yet popped, or None."""
        if self._consumed >= len(self.received):
            return None
        byte = self.received[self._consumed]
        self._consumed += 1
        return byte

    # -- the device ----------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        level = pins.get(self.net, IDLE_LEVEL)
        previous = self._last
        self._last = level
        if not self._in_frame:
            if previous == IDLE_LEVEL and level == 0:
                self._start_frame(t)
        elif level != previous:
            self._measure_edge(t - self.latency)
        if self._in_frame and t >= self._sample_at:
            self._sample(level)
        return {}

    def _target(self, k: int) -> float:
        """Tick at which sample k is taken, in observation time."""
        return self._origin + (k + 0.5) * self._tpb + self.latency

    def _start_frame(self, t: int) -> None:
        self._in_frame = True
        self._origin = t - self.latency
        self._next_k = 0
        self._measure_edge(self._origin)          # the start-bit edge itself
        self._k = 0
        self._sample_at = self._target(0)
        self._word = 0
        self._levels_seen = []
        self._framing = False
        self._parity_bad = False

    def _measure_edge(self, tick: int) -> None:
        """Find the nominal bit boundary this transition belongs to and record it."""
        best_k = self._next_k
        best_dev = tick - (self._origin + best_k * self._tpb)
        for k in range(self._next_k, self._next_k + self._n_bits + 1):
            dev = tick - (self._origin + k * self._tpb)
            if abs(dev) < abs(best_dev) - _EPS:
                best_k, best_dev = k, dev
        self._next_k = best_k + 1
        nominal = self._origin + best_k * self._tpb
        self.edges.append(Edge(tick=tick, bit=best_k, nominal=nominal, dev=tick - nominal))
        self.worst_dev_ticks = max(self.worst_dev_ticks, abs(tick - nominal))
        self.min_dev_ticks = min(self.min_dev_ticks, tick - nominal)
        self.max_dev_ticks = max(self.max_dev_ticks, tick - nominal)

    def _sample(self, level: int) -> None:
        k = self._k
        self._levels_seen.append(level)
        if k == 0:
            if level != 0:
                self.aborted += 1        # the line was high at mid-bit: not a start bit
                self._clear_frame()
                return
        elif k <= self.data_bits:
            self._word |= level << (k - 1)
        elif k == self._parity_k:
            if level != parity_bit(self._word, self.data_bits, self.parity):
                self._parity_bad = True
        else:                             # stop bits
            if level == 0:
                self._framing = True
        if k >= self._n_bits - 1:
            self._finish_frame()
        else:
            self._k = k + 1
            self._sample_at = self._target(k + 1)

    def _finish_frame(self) -> None:
        byte = RxByte(
            value=self._word,
            tick=self._origin,
            framing_error=self._framing,
            parity_error=self._parity_bad,
            levels=tuple(self._levels_seen),
        )
        self.received.append(byte)
        self.framing_errors += bool(byte.framing_error)
        self.parity_errors += bool(byte.parity_error)
        self._clear_frame()

    def _clear_frame(self) -> None:
        self._in_frame = False
        self._k = 0
        self._sample_at = float("inf")
        self._word = 0
        self._levels_seen = []
        self._framing = False
        self._parity_bad = False
"""Pin-level PS/2 reference models on the tick bus of verif/vip/core.py.

PS/2 is the one protocol in this VIP set whose clock comes from the *device*, so one engine
that owns a single pin has to bit-bang the clock. Two nets, "clk" and "data", both open drain
with a pull-up: every device here drives 0 or releases (`None`), never 1.

    bus = Bus(ns_per_tick=20)                        # 50 MHz chip clock
    host = attach_host(bus)
    kbd = attach_keyboard(bus)
    chk = attach_checker(bus)                        # optional, passive

    host.send(0xFF)                                  # reset the keyboard
    assert host.run(bus) and kbd.run(bus)            # host.run ends when its queue is empty,
    assert host.values == [0xFA, 0xAA]              # kbd.run when the device is quiet too

    host.send(0xED, 0x07)                            # set all three LEDs
    assert host.run(bus) and kbd.run(bus)
    assert host.values[-2:] == [0xFA, 0xFA] and kbd.caps_lock

    kbd.press("a"); kbd.release("a")                 # 0x1C, then the break pair 0xF0 0x1C
    assert kbd.run(bus)
    assert host.values[-3:] == [0x1C, 0xF0, 0x1C]
    assert chk.clean, chk.report()                   # 7 frames, no violations

The frame is 11 bits, LSB first, odd parity, both directions (IBM PC/AT Technical Reference,
"Auxiliary Device"; OSDev "PS/2"):

    start(0) | d0 d1 d2 d3 d4 d5 d6 d7 | parity | stop(1)      11 clocks, device -> host
    start(0) | d0 ... d7 | parity | stop(1) | ack(0)           12 clocks, host -> device

The twelfth clock exists only in the host->device direction: the device pulls DATA low for it
("the auxiliary device pulls the 'data' line inactive, producing the line-control bit").

Who owns the clock, and who samples on which edge
------------------------------------------------
The two directions are deliberately different, and every model here obeys both halves:

* **device -> host**: the device generates the clock, the host samples DATA on the **falling**
  edge. The device changes DATA inside the high phase, at least 5 us after the rising edge
  (T2) and 5..25 us before the falling edge (T1). The clock's inactive and active durations are
  both 30..50 us (T3, T4). After the 11th clock the bus must stay free: the keyboard waits
  >= 50 us before it sends anything else, and the host may hold the clock low for up to 50 us
  after clock 11 to stop it (T5).
* **host -> device**: the host generates the clock, the device samples DATA on the **rising**
  edge. The host holds the clock low for >= 100 us before placing the start bit (the
  request-to-send), then clocks 12 times with the inactive and active durations of T7/T8
  (30..50 us). DATA must be stable within 1 us after each rising edge, and -- derived from T1's
  5 us floor, since the host->device table has no setup row of its own -- at least 5 us before
  the rising edge that samples it.

Every number above is in `research/ref/protocols/ps2/`, cited field by field in
`Ps2Timings`. Three things the sources leave open are named parameters, not limits:
`start_bit_hold` (how long the start bit sits on an idle-high DATA line before the first clock
edge), `data_after_rise` (where inside the high phase the device changes DATA: T2 bounds it
only), and the tick at which each side samples (both edges are legal sampling points).

The three devices
-----------------
* `Ps2Host` is the controller side: `send()` queues host->device bytes and it clocks them out,
  it decodes device->host bytes on the falling edge with parity and framing flags, and
  `inhibit()` pulls the clock low to stop the device transmitting.
* `Ps2Device` is the shared device half of the wire: it generates the clock for the bytes it
  sends, ACKs the host's 12th clock, answers `0xFE` by re-sending its last byte, and refuses to
  transmit while the host holds the clock low (the AT table: "if the line is inactive, output
  from the device is not allowed"). `Ps2Keyboard` and `Ps2Mouse` add the command sets, the scan
  code tables and the movement packets.
* `Ps2Checker` drives nothing. It watches the pins, works out which direction each frame is
  from the edge that started it (DATA falling on an idle-high clock = device -> host, CLK
  falling = the host's request-to-send), and checks every rule above plus the framing rules
  (`start-bit`, `parity`, `stop-bit`, `ack-bit`, `frame-length`). Every violation is appended
  to `violations` with the tick it happened on, the number of ticks measured, the number
  required and which limit was broken -- PS/2 is full of *maximums*, unlike I2C.

Firmware notes (the other half of this VIP is written against this file)
------------------------------------------------------------------------
* One `send()` is one 11-12 clock frame plus the device's ack. Do not start the next frame
  until the ack bit has been clocked, or the device will see a truncated frame.
* A byte is only complete when you have seen `0xFA`. `0xFE` means re-send the same byte;
  `Ps2Host` does that for you up to `retries` times and then records an error.
* Device->host bytes arrive whenever the device feels like it, so a host transmit has to wait
  for the bus; `Ps2Host` waits `bus_free` ticks of an idle-high bus before it pulls the clock
  low, which is what the 100 us request-to-send is for.
* Timing is not the only rule that bites: a short clock low period, a long clock high period,
  DATA moved too early or too late inside the high phase, even parity, a stop bit of 0 and a
  missing ack bit are all violations, and `Ps2Checker.report()` names the tick of each.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import ceil
from typing import Deque, Mapping, Optional

from .core import Bus, Device

CLK_NET = "clk"
DATA_NET = "data"

FRAME_BITS = 11              # start + 8 data + parity + stop
HOST_FRAME_BITS = 12         # the same, plus the device's acknowledgement bit
ACK = 0xFA                   # device: command acknowledged
RESEND = 0xFE                # device: send the last byte again
ECHO = 0xEE
SELFTEST_OK = 0xAA
ID_BYTE_MOUSE = 0x00
ID_BYTE_KBD = (0xAB, 0x83)   # Gdansk page: "two-byte device ID of 0xAB, 0x83"


def ps2_parity_bit(value: int) -> int:
    """The odd parity bit that follows the 8 data bits: 1 when they hold an even number of 1s.

    "The parity bit is either 1 or 0, and the 8 data bits, plus the parity bit, always have an
    odd number of 1's" (AT Technical Reference, Auxiliary Device).
    """
    return 1 - (int(value) & 0xFF).bit_count() % 2


def ps2_frame_bits(value: int) -> tuple[int, ...]:
    """The 11 bits of one frame on the wire, start bit first, data LSB first.

    Subclass-friendly through `Ps2Device.frame_bits`, which is how a test injects a wrong
    parity bit or a stop bit of 0.
    """
    word = int(value) & 0xFF
    bits = [0]                                     # start bit, always 0
    bits += [(word >> i) & 1 for i in range(8)]    # data, LSB first
    bits.append(ps2_parity_bit(word))
    bits.append(1)                                 # stop bit, always 1
    return tuple(bits)


@dataclass(frozen=True)
class Ps2Timings:
    """Every PS/2 timing in nanoseconds, each one named as the source names it.

    The two columns that are *limits* are the ones `Ps2Checker` enforces: PS/2 windows are
    two-sided (30..50 us for both clock halves), unlike I2C, where every timing is a floor.
    The rest are the positions the models put their edges at, and they are chosen strictly
    inside the window they sit in, so a model at the default timings is legal by construction.

    `rate` says which of the two legal rates the models clock at: "fast" is 30 us + 30 us,
    the 16.67 kbit/s the OSDev page gives as the top of "10 KHz to 16.7 KHz"; "slow" is
    50 + 50, the 10 kbit/s the same 30..50 us window gives at its other end.
    """

    name: str
    rate: str = "fast"
    # --- the clock, both directions (T3/T4 device->host, T7/T8 host->device) --------------
    clk_low: float = 30_000.0      # CLK inactive, the models' nominal half period
    clk_high: float = 30_000.0     # CLK active, the models' nominal half period
    clk_low_min: float = 30_000.0  # T3/T7 "Duration of CLK inactive 30/50 us"
    clk_low_max: float = 50_000.0
    clk_high_min: float = 30_000.0  # T4/T8 "Duration of CLK active 30/50 us"
    clk_high_max: float = 50_000.0
    # --- where DATA moves ------------------------------------------------------------------
    data_after_rise: float = 0.0       # device->host: DATA changes this long after the rising
                                       # edge. 0 means "the middle of the window T2 and T1
                                       # leave for this clock", which is how the models pick
                                       # it; no source gives a position at all.
    data_after_rise_min: float = 5_000.0    # T2 "Time from rising edge of CLK to DATA
                                            # transition 5/T4-5 us"
    data_after_rise_max: float = 1_000.0    # host->device: "Data must be stable within 1
                                            # microsecond after the rising edge of the clock"
    data_before_edge_min: float = 5_000.0   # T1 "Time from DATA transition to falling edge of
                                            # CLK 5/25 us", and the floor this model applies
                                            # to the rising edge in the host->device direction
    data_before_edge_max: float = 25_000.0  # T1, the 25 us of the same row
    # --- the edges that have no row of their own ------------------------------------------
    rts_hold: float = 100_000.0    # host->device only: "Bring the Clock line low for at least
                                   # 100 microseconds" before the start bit (Gdansk page,
                                   # host->device step 1). Checked as tRTS; it replaces
                                   # clk_low_min/max for that one low period.
    bus_free: float = 50_000.0     # the device's own wait after its stop bit ("the keyboard
                                   # waits for at least 50us, until it transmits next data").
                                   # The host waits the same before a request-to-send, and
                                   # `Ps2Checker` checks it as the floor of `tBUS-FREE`.
    start_bit_hold: float = 0.0    # 0 means "one clk_low", see _effective()
    inhibit_window: float = 50_000.0  # T5's maximum: how long after clock 11 the host may pull
                                   # the clock low to inhibit the device

    def _effective(self) -> "Ps2Timings":
        if self.start_bit_hold:
            return self
        return Ps2Timings(**{**self.__dict__, "start_bit_hold": self.clk_low})


#: The two legal rates, as (nominal low, nominal high) in ns. Both sit exactly on one of the
#: table's limits, which is deliberate: a model at "fast" is at the 30 us floor and a model at
#: "slow" is at the 50 us ceiling, so the checker's minimum and maximum rules are both exercised
#: by legal traffic.
PS2_RATES: dict[str, Ps2Timings] = {
    "fast": Ps2Timings(name="fast", rate="fast", clk_low=30_000.0, clk_high=30_000.0),
    "slow": Ps2Timings(name="slow", rate="slow", clk_low=50_000.0, clk_high=50_000.0),
}

#: Which of `Ps2Timings`' fields are limits, in the order `rate_minimums` documents them.
#: `bus_free_min` is not on the dataclass: T5's own floor is "greater than zero" rather than a
#: number of microseconds, and what the wire can be asked for is the keyboard's own 50 us wait
#: that the Gdansk page states, which `rate_minimums` adds.
LIMIT_KEYS = (
    "clk_low_min", "clk_low_max", "clk_high_min", "clk_high_max",
    "data_after_rise_min", "data_after_rise_max",
    "data_before_edge_min", "data_before_edge_max",
    "rts_hold", "inhibit_window",
)

_MODEL_KEYS = ("clk_low", "clk_high", "data_after_rise", "start_bit_hold")


def rate_minimums(
    ns_per_tick: float, rate: str = "fast", overrides: Optional[dict] = None
) -> dict:
    """Every limit of `rate` as whole ticks, rounded up, plus the model's own phase lengths.

    `overrides` replaces individual nanosecond values by their field names, which is how a test
    drives a model off-spec or points a checker at a different limit:
    ``{"data_after_rise_min": 20_000}`` means "DATA may not move until 20 us after the rising
    edge", and ``{"bus_free_min": 100_000}`` means "the bus must be free for 100 us between
    frames". A test can also override ``bus_free`` (the model's wait) to make the *model* break
    the rule the checker enforces.

    "min_period" is the shortest legal clock period, clk_low_min + clk_high_min, rounded up:
    the two half periods are both floors, so this is the fastest a device may clock.
    """
    try:
        t = PS2_RATES[rate]._effective()
    except KeyError:
        raise ValueError(
            f"unknown PS/2 rate {rate!r}; pick one of {sorted(PS2_RATES)}"
        ) from None
    ns = {key: getattr(t, key) for key in LIMIT_KEYS if hasattr(t, key)}
    # The bus-free minimum is the device's own wait: "After sending stop bit, the keyboard
    # waits for at least 50us, until it transmits next data" (Gdansk page). T5's own wording is
    # ">0/50 us" for the inhibit, and the AT table's ceiling is a duty on the host (it may pull
    # the clock low within 50 us of clock 11 to stop the device), not a limit on the device's
    # wait -- so the floor that can be measured from the wire is the Gdansk 50 us, and
    # `inhibit_window` keeps the 50 us the host has to act in.
    ns["bus_free_min"] = t.bus_free
    ns["bus_free"] = t.bus_free
    for key in _MODEL_KEYS:
        ns[key] = getattr(t, key)
    overrides = dict(overrides or {})
    for key, value in overrides.items():
        if key not in ns:
            raise ValueError(f"unknown timing override {key!r}; pick from {sorted(ns)}")
        if float(value) < 0:
            raise ValueError(f"timing {key!r} must not be negative, got {value!r}")
        ns[key] = float(value)
    out = {key: ns_to_ticks(ns_per_tick, value) for key, value in ns.items()}
    out["rate"] = rate
    out["mode"] = rate
    out["ns_per_tick"] = ns_per_tick
    out["min_period"] = out["clk_low_min"] + out["clk_high_min"]
    out["max_period"] = out["clk_low_max"] + out["clk_high_max"]
    return out


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, float(ns)) / ns_per_tick))


@dataclass
class Ps2Byte:
    """One byte as it went over the wire, with what the decoder made of it."""

    value: int
    tick: int                      # tick of the first bit's sampling edge
    direction: str                 # "to_host" (device -> host) or "to_device" (host -> device)
    parity_error: bool = False
    framing_error: bool = False
    ack: Optional[bool] = None     # host -> device frames: the 12th (acknowledgement) bit

    @property
    def ok(self) -> bool:
        return not self.parity_error and not self.framing_error

    def __str__(self) -> str:
        text = f"t={self.tick} 0x{self.value:02X} {self.direction}"
        if self.ack is not None:
            text += " ack" if self.ack else " NO ACK"
        if self.framing_error:
            text += " framing-error"
        if self.parity_error:
            text += " parity-error"
        return text


@dataclass
class Ps2Sent:
    """One byte as the host clocked it out."""

    value: int
    start_tick: int
    end_tick: Optional[int] = None
    ack: Optional[bool] = None
    attempt: int = 1

    def __str__(self) -> str:
        return (f"t={self.start_tick} sent 0x{self.value:02X}"
                + ("" if self.ack is None else " ack" if self.ack else " NO ACK"))


class Ps2Host(Device):
    """The controller side of a PS/2 port: it clocks bytes to the device and decodes the
    bytes the device clocks back.

    `send(0xFF)` queues one byte; `run(bus)` steps the bus until the queue is empty. Every
    queued byte is clocked out as the full 12-clock frame and the device's twelfth bit is read
    as the acknowledgement: an ACK (DATA low) counts, a missing ACK is recorded in `nacks` and
    the byte is re-sent up to `retries` times, which is what the OSDev driver rule asks for
    ("if you send a command and the keyboard responds with 0xFE (resend) then you have to send
    the command again ... possibly limited to 3 retries"). `Ps2Sent` records every attempt.

    The receive side is passive and needs no arming: a frame that starts with DATA falling while
    the clock reads high is a device -> host frame, and its 11 bits are sampled on the falling
    edges, as the Gdansk page requires ("Data sent by keyboard to PC are read during falling
    edge of the clock"). Parity and stop-bit errors are kept per byte in `received` rather than
    dropped, so a test can look at what the wire really carried. `inhibit()` pulls the clock low
    for a while, which is how the host stops the device transmitting ("The system can hold the
    'clock' signal inactive to inhibit the next transmission"); during a frame it lengthens that
    clock period, and `Ps2Checker` will say so if it goes past the 50 us maximum.

    Timing comes from `rate_minimums(bus.ns_per_tick, rate, timing)`, so the same class gives a
    legal 16.67 kbit/s or 10 kbit/s host. `timing` can drive it off-spec on purpose, e.g.
    `{"clk_low": 10_000}`.
    """

    name = "ps2_host"

    def __init__(
        self,
        rate: str = "fast",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        retries: int = 3,
        clk_net: str = CLK_NET,
        data_net: str = DATA_NET,
        name: Optional[str] = None,
    ):
        if retries < 0:
            raise ValueError(f"retries must be >= 0, got {retries!r}")
        self.rate = rate
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.retries = int(retries)
        self.clk_net, self.data_net = clk_net, data_net
        if name:
            self.name = name
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        m = rate_minimums(self.ns_per_tick, self.rate, self.timing_overrides)
        self.min_ticks = m
        self.clk_low = max(1, m["clk_low"])
        self.clk_high = max(1, m["clk_high"])
        self.rts = max(1, m["rts_hold"])
        # DATA moves this far into the low phase, which is both the hold after the falling edge
        # and the setup before the rising edge that samples it. Clamped inside the low phase so
        # an off-spec override cannot skip the move entirely.
        self.data_hold = max(1, min(m["data_before_edge_min"], self.clk_low - 1))
        self.bus_free = m["bus_free"]
        # A device frame that never finishes is dropped after this many ticks, so a bus that
        # stops mid-frame is an error and not a hang.
        self.frame_timeout = 3 * (FRAME_BITS * (self.clk_low + self.clk_high) + self.bus_free)

    def bind(self, bus: Bus) -> "Ps2Host":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        self._reset()
        return self

    def _reset(self) -> None:
        self.queue: Deque[int] = deque()
        self.sent: list[Ps2Sent] = []
        self.received: list[Ps2Byte] = []
        self.errors: list[str] = []
        self.acks = 0
        self.nacks = 0
        self.parity_errors = 0
        self.framing_errors = 0
        self.inhibited = 0
        self._popped = 0
        self._tx_reset()
        self._rx_reset()

    # ---- the transmit side ----------------------------------------------------------
    def _tx_reset(self) -> None:
        self.phase = "idle"           # idle | rts | start | low | high
        self.cnt = 0
        self.bit = 0
        self.bits: tuple[int, ...] = ()
        self.clk_out: Optional[int] = None
        self.data_out: Optional[int] = None
        self.next_data: Optional[int] = None
        self.change_in = -1          # ticks until DATA moves; -1 = nothing armed
        self.idle_left = self.bus_free
        self.inhibit_left = 0
        self.current: Optional[Ps2Sent] = None

    def _rx_reset(self) -> None:
        self.rx_bits: list[int] = []
        self.rx_start = -1
        self.rx_deadline = -1
        self.rx_seen = 0

    # ---- the queue -------------------------------------------------------------------
    def send(self, *data) -> "Ps2Host":
        """Queue bytes for the device: send(0xFF), send([0xF0, 0x02]), send(range(4))."""
        for item in data:
            if isinstance(item, int):
                self.queue.append(item & 0xFF)
            else:
                for byte in item:
                    self.queue.append(int(byte) & 0xFF)
        return self

    @property
    def pending(self) -> int:
        """Bytes queued but not started yet."""
        return len(self.queue)

    @property
    def busy(self) -> bool:
        return self.phase != "idle" or bool(self.queue)

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def values(self) -> list[int]:
        """The value of every byte decoded from the device, in order."""
        return [b.value for b in self.received]

    def pop(self) -> Optional[Ps2Byte]:
        """Oldest received byte that has not been popped, or None."""
        if self._popped >= len(self.received):
            return None
        byte = self.received[self._popped]
        self._popped += 1
        return byte

    def flush_rx(self) -> None:
        """Forget every byte received so far."""
        self.received = []
        self._popped = 0

    def request_resend(self) -> "Ps2Host":
        """Queue 0xFE, the device's "send that byte again" command."""
        return self.send(RESEND)

    def inhibit(self, ticks: Optional[int] = None) -> None:
        """Pull the clock low for `ticks` (default: T5's 50 us inhibit window).

        The device may not transmit while the clock reads low, so this is how a host keeps a
        talkative device off the bus while it does something else.
        """
        self.inhibit_left = self.min_ticks["inhibit_window"] if ticks is None else int(ticks)

    def run(self, bus: Bus, limit: int = 2_000_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out first."""
        return bus.run_until(lambda _b: self.idle, limit)

    def frame_bits(self, value: int) -> tuple[int, ...]:
        """The 11 bits this host clocks out; override to model a faulty controller."""
        return ps2_frame_bits(value)

    # ---- the device -------------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        """Advance first, then drive: a state change made now is on the wire from this tick."""
        self._advance(t, pins)
        return self._drive()

    def _drive(self) -> dict:
        clk = 0 if self.inhibit_left > 0 else self.clk_out
        return {self.clk_net: clk, self.data_net: self.data_out}

    def _advance(self, t: int, pins: Mapping[str, int]) -> None:
        if self.inhibit_left > 0:
            self.inhibit_left -= 1
            self.inhibited += 1
        self._receive(t, pins)
        # A phase change cascades: the tick that moves into the low period is that period's
        # first tick, so every counter below counts ticks of the drive it controls and the
        # phases on the wire are the phases the counter asked for, to the tick.
        for _ in range(4):
            phase = self.phase
            self._step_phase(t, pins, phase)
            if self.phase == phase:
                break

    def _step_phase(self, t: int, pins: Mapping[str, int], phase: str) -> None:
        if phase == "idle":
            # The bus has to be free, not merely "no frame of ours in progress": a device is
            # allowed to start sending at any time, and pulling the clock low in the middle of
            # its frame would truncate it ("The system checks for an auxiliary device
            # transmission in process").
            if pins[self.clk_net] == 1 and pins[self.data_net] == 1:
                if self.idle_left > 0:
                    self.idle_left -= 1
                elif self.queue:
                    self._start_frame(t)
            else:
                self.idle_left = self.bus_free
        elif phase == "rts":
            if self.cnt >= self.rts:
                self.cnt = 0
                # The start bit goes down while the clock is still low (Gdansk host->device
                # steps 1-3): clock low, then data low, then release the clock.
                self.data_out = self.bits[0]
                self.phase, self.cnt = "start", 0
            else:
                self.cnt += 1
        elif phase == "start":
            if self.cnt >= self.data_hold:
                self.cnt = 0
                # The start bit stays until the rising edge that samples it; DATA moves for
                # bit 1 in the low period that follows that edge.
                self.change_in = -1
                self.phase, self.cnt = "low", 0
            else:
                self.cnt += 1
        elif phase == "low":
            self._low(t)
        elif phase == "high":
            self._high(t, pins)

    def _start_frame(self, t: int) -> None:
        value = self.queue[0]
        self.bits = self.frame_bits(value)
        self.bit = 0
        self.phase, self.cnt = "rts", 0
        self.clk_out = 0                # the request-to-send: the clock goes low and stays low
        self.data_out = None
        self.next_data = None
        self.current = Ps2Sent(value=value, start_tick=t,
                                attempt=len([s for s in self.sent if s.value == value]) + 1)

    def _low(self, t: int) -> None:
        """One clock low period. DATA for the bit after this one moves at the hold point."""
        if self.change_in >= 0:
            self.change_in -= 1
            if self.change_in < 0:
                self.data_out = self.next_data
        if self.cnt >= self.clk_low:
            self.phase, self.cnt = "high", 0
            self.clk_out = None            # the rising edge the device samples on
        else:
            self.cnt += 1

    def _high(self, t: int, pins: Mapping[str, int]) -> None:
        """One clock high period: the device samples this bit at its start."""
        if self.cnt == 0:
            self._clock_done(t, pins)
        if self.cnt >= self.clk_high:
            if self.bit >= HOST_FRAME_BITS - 1:
                self._frame_done(t)
                return
            self.bit += 1
            # Bit n+1 goes on the wire during this low period; the ack bit is the device's,
            # so the host lets go of DATA before clocking it.
            self.next_data = (self.bits[self.bit] if self.bit < FRAME_BITS else None)
            self.change_in = self.data_hold
            self.phase, self.cnt = "low", 0
            self.clk_out = 0
        else:
            self.cnt += 1

    def _clock_done(self, t: int, pins: Mapping[str, int]) -> None:
        """The rising edge that made the device sample bit `self.bit` is now on the wire.

        Only the twelfth clock says anything new to the host: it is the device's
        acknowledgement bit. The other eleven are the bits this host just drove, so there is
        nothing to check -- a frame this host builds wrong is the *checker's* business, and a
        frame the device reads wrong is the *device's*.
        """
        if self.bit == HOST_FRAME_BITS - 1:
            acked = pins[self.data_net] == 0
            self.acks += int(acked)
            self.nacks += int(not acked)
            if self.current is not None:
                self.current.ack = acked
            if not acked:
                self._retry(t)

    def _frame_done(self, t: int) -> None:
        """The last clock period is over: release both lines and take the next byte."""
        if self.current is not None:
            self.current.end_tick = t
            self.sent.append(self.current)
        self.queue.popleft()
        self.current = None
        self.phase, self.cnt, self.bit = "idle", 0, 0
        self.clk_out, self.data_out = None, None
        self.idle_left = self.bus_free

    def _retry(self, t: int) -> None:
        """No acknowledgement: send the same byte again, up to `retries` times."""
        attempts = len([s for s in self.sent if s.value == self.queue[0]]) + 1
        if attempts > self.retries:
            self.errors.append(
                f"{self.name}: 0x{self.queue[0]:02X} was never acknowledged "
                f"({attempts} tries at t={t})"
            )
            self._frame_done(t)
            return
        if self.current is not None:
            self.current.end_tick = t
            self.sent.append(self.current)
        self.current = None
        self.phase, self.cnt, self.bit = "idle", 0, 0
        self.clk_out, self.data_out = None, None
        self.idle_left = self.bus_free

    # ---- the receive side ------------------------------------------------------------
    def _receive(self, t: int, pins: Mapping[str, int]) -> None:
        """Decode a device -> host frame: DATA falls on an idle-high clock, then 11 bits on
        the falling edges."""
        clk, data = pins[self.clk_net], pins[self.data_net]
        prev_clk, prev_data = self._prev_clk, self._prev_data
        self._prev_clk, self._prev_data = clk, data
        if not self.rx_active:
            if prev_data == 1 and data == 0 and clk == 1 and self.phase == "idle":
                self.rx_active = True
                self.rx_start = t
                self.rx_bits = []
                self.rx_deadline = t + self.frame_timeout
            return
        if prev_clk == 1 and clk == 0:
            self.rx_bits.append(data)
            if len(self.rx_bits) == FRAME_BITS:
                self._rx_done(t)
            return
        if t > self.rx_deadline:
            self.errors.append(
                f"{self.name}: device frame from t={self.rx_start} stalled after "
                f"{len(self.rx_bits)} of {FRAME_BITS} bits at t={t}"
            )
            self._rx_reset()

    def _rx_done(self, t: int) -> None:
        bits = self.rx_bits
        value = 0
        for i in range(8):
            value |= (bits[i + 1] & 1) << i
        framing = bits[0] != 0 or bits[10] != 1
        parity = bits[9] != ps2_parity_bit(value)
        byte = Ps2Byte(
            value=value, tick=self.rx_start, direction="to_host",
            parity_error=parity, framing_error=framing,
        )
        self.received.append(byte)
        self.parity_errors += int(parity)
        self.framing_errors += int(framing)
        self._rx_reset()

    def _rx_reset(self) -> None:
        self.rx_active = False
        self.rx_bits = []
        self.rx_start = -1
        self.rx_deadline = -1
        self._prev_clk = 1
        self._prev_data = 1


class Ps2Device(Device):
    """The device half of a PS/2 wire: it generates the clock for the bytes it sends, and it
    answers the host's twelfth clock.

    Subclasses (`Ps2Keyboard`, `Ps2Mouse`) add the command sets; everything about timing,
    framing and the two edges belongs here.

    Sending. A byte is sent when the bus has been idle -- both lines high -- for `bus_free`
    ticks, which is how the device obeys "if the line is inactive, output from the device is not
    allowed" (AT Technical Reference, system receiving data, step 1) and the keyboard's own
    "waits for at least 50us" after its stop bit. `inhibited` counts the ticks a queued byte
    spent waiting because the host was holding the clock low; the byte is not lost.

    Receiving. A host frame starts with DATA falling while the clock is already low -- that is
    the host's request-to-send followed by its start bit -- and its 12 bits are sampled on the
    rising edges, as the Gdansk page requires ("Data sent by PC are always read on rising edge
    of the clock"). The first 11 are the frame; the twelfth is the clock the device answers by
    pulling DATA low. Every such bit is counted in `acks`.

    Errors. A start bit of 1, a stop bit of 0 or a parity bit that does not make the nine bits
    odd is counted in `framing_errors` / `parity_errors`, recorded in `received`, and answered
    the way the AT Technical Reference describes: the device still clocks the acknowledgement
    bit and then queues `0xFE`, so the host re-sends the byte.

    `frame_bits()` is a method so a test can subclass or monkey-patch it and put a wrong parity
    or stop bit on the wire.

    `host_priority_ticks` (4 by default) is the gap the device leaves between "the bus has been
    free" and "I start my frame". It is how the host's priority is modelled: the host may pull
    the clock low at any moment, and the device must see that before it has put a start bit on
    the wire. Four ticks is 80 ns at 20 ns per tick, which is far less than any timing in the
    protocol and only decides who starts when both sides are ready on the same tick.
    """

    name = "ps2_device"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        rate: str = "fast",
        timing: Optional[dict] = None,
        clk_net: str = CLK_NET,
        data_net: str = DATA_NET,
        host_priority_ticks: int = 4,
        name: Optional[str] = None,
    ):
        if host_priority_ticks < 0:
            raise ValueError(f"host_priority_ticks must be >= 0, got {host_priority_ticks!r}")
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.rate = rate
        self.timing_overrides = dict(timing or {})
        self.clk_net, self.data_net = clk_net, data_net
        self.host_priority_ticks = int(host_priority_ticks)
        if name:
            self.name = name
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        m = rate_minimums(self.ns_per_tick, self.rate, self.timing_overrides)
        self.min_ticks = m
        self.clk_low = max(1, m["clk_low"])
        self.clk_high = max(1, m["clk_high"])
        self.data_after_rise = self._data_after_rise(m)
        self.start_hold = max(1, m["start_bit_hold"])
        self.bus_free = m["bus_free"]
        # The acknowledgement bit is pulled low this far into the clock low period that
        # follows the stop bit, clamped inside that period.
        self.ack_drive = max(1, min(m["data_before_edge_min"], max(1, self.clk_low - 1)))
        self.frame_timeout = 3 * (HOST_FRAME_BITS * (self.clk_low + self.clk_high) + self.bus_free)

    def _data_after_rise(self, m: dict) -> int:
        """Where inside the high period this device moves DATA, in ticks.

        No source puts a position in that window: T2 says "at least 5 us after the rising
        edge" and T1 says "5 to 25 us before the falling edge", so the legal band is
        [max(T2_min, clk_high - T1_max), clk_high - T1_min]. The models take the middle of it,
        which is inside the window at both rates, or whatever `timing` asked for.
        """
        if m["data_after_rise"] > 0:
            return min(m["data_after_rise"], max(0, self.clk_high - 1))
        low = max(m["data_after_rise_min"], self.clk_high - m["data_before_edge_max"])
        high = max(low, self.clk_high - m["data_before_edge_min"])
        return max(0, (low + high) // 2)

    def bind(self, bus: Bus) -> "Ps2Device":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        self._reset()
        return self

    def _reset(self) -> None:
        self.tx: Deque[int] = deque()
        self.sent: list[Ps2Byte] = []
        self.received: list[Ps2Byte] = []
        self.errors: list[str] = []
        self.last_byte: Optional[int] = None      # the last byte the host sent
        self.last_sent: Optional[int] = None     # the last byte we sent, for 0xFE
        self.acks = 0
        self.framing_errors = 0
        self.parity_errors = 0
        self.resends = 0
        self.inhibited = 0
        self.frames_sent = 0
        self.frames_received = 0
        self._tx_reset()
        self._rx_reset()

    def _tx_reset(self) -> None:
        self.phase = "idle"           # idle | start | low | high | release
        self.cnt = 0
        self.bit = 0
        self.bits: tuple[int, ...] = ()
        self.clk_out: Optional[int] = None
        self.data_out: Optional[int] = None
        self.next_data: Optional[int] = None
        self.need_data = False
        self.idle_left = self.bus_free
        self.priority_left = 0
        self.tx_start = -1

    def _rx_reset(self) -> None:
        self.rx_active = False
        self.rx_bits = []
        self.rx_start = -1
        self.rx_deadline = -1
        self.rx_ack: Optional[int] = None
        self._prev_clk = 1
        self._prev_data = 1

    # ---- the byte queue --------------------------------------------------------------

    def queue(self, *data) -> "Ps2Device":
        """Queue bytes for the host: queue(0xFA), queue(b"\\x1c\\xf0\\x1c")."""
        for item in data:
            if isinstance(item, int):
                self.tx.append(item & 0xFF)
            else:
                for byte in item:
                    self.tx.append(int(byte) & 0xFF)
        return self

    @property
    def pending(self) -> int:
        """Bytes queued but not started yet."""
        return len(self.tx)

    @property
    def busy(self) -> bool:
        return self.phase != "idle" or self.rx_active

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def values(self) -> list[int]:
        """The value of every byte this device clocked at the host, in order."""
        return [b.value for b in self.sent]

    def flush_rx(self) -> None:
        self.received = []

    def run(self, bus: Bus, limit: int = 2_000_000) -> bool:
        """Step `bus` until every queued byte has been clocked out. False on a limit."""
        return bus.run_until(lambda _b: self.idle and not self.tx, limit)

    def frame_bits(self, value: int) -> tuple[int, ...]:
        """The 11 bits this device clocks out; override to model a faulty device."""
        return ps2_frame_bits(value)

    def _command(self, value: int) -> list[int]:
        """What this device does with a byte the host sent: the bytes it answers with.

        The base class implements the three commands every PS/2 device shares.
        """
        if value == RESEND:
            self.resends += 1
            return [] if self.last_sent is None else [self.last_sent]
        if value == ECHO:
            return [ECHO]
        return [ACK]

    # ---- the device -------------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        """Advance first, then drive: a state change made now is on the wire from this tick."""
        self._advance(t, pins)
        return {self.clk_net: self.clk_out, self.data_net: self.data_out}

    def _advance(self, t: int, pins: Mapping[str, int]) -> None:
        self._receive(t, pins)
        # A phase change cascades, as in the host: the tick that moves into a period is that
        # period's first tick, so `data_after_rise` is exactly how many ticks after the rising
        # edge DATA goes on the wire and `clk_low`/`clk_high` are exactly what is measured.
        for _ in range(4):
            phase = self.phase
            self._step_phase(t, pins, phase)
            if self.phase == phase:
                break

    def _step_phase(self, t: int, pins: Mapping[str, int], phase: str) -> None:
        if phase == "idle":
            if pins[self.clk_net] == 1 and pins[self.data_net] == 1:
                if self.idle_left > 0:
                    self.idle_left -= 1
                    if self.idle_left == 0:
                        self.priority_left = self.host_priority_ticks
                elif self.priority_left > 0:
                    # The host goes first: it may pull the clock low at any time ("the PC has
                    # always a priority and can stop the transmission any time by setting
                    # CLOCK low"), so the device leaves a few ticks' gap after its bus-free
                    # wait. That gap is what makes the contest decidable: without it a host
                    # with a byte queued and a device with an answer ready would start on the
                    # same tick and neither frame would be readable.
                    self.priority_left -= 1
                elif self.tx:
                    self._start_frame(t)
            else:
                # The bus is not free: the host may be holding the clock low, and the AT
                # Technical Reference is explicit that the device may not transmit then. The
                # bus-free timer restarts whether or not there is anything to send, so a byte
                # queued during the host's frame still waits a full `bus_free` after it.
                self.idle_left = self.bus_free
                self.priority_left = 0
                if self.tx:
                    self.inhibited += 1
        elif phase == "start":
            if self.cnt >= self.start_hold:
                self.phase, self.cnt = "low", 0
                self.clk_out = 0
            else:
                self.cnt += 1
        elif phase == "low":
            self._low()
        elif phase == "high":
            self._high()
        elif phase == "release":
            self._frame_done(t)

    def _low(self) -> None:
        """One clock low period. Its end is where DATA for the next bit gets armed."""
        if self.cnt >= self.clk_low:
            self.phase, self.cnt = "high", 0
            self.clk_out = None
            self.need_data = self.bit + 1 < len(self.bits)
            self.next_data = self.bits[self.bit + 1] if self.need_data else None
        else:
            self.cnt += 1

    def _high(self) -> None:
        """One clock high period: DATA moves `data_after_rise` ticks into it, and its end is
        the falling edge that samples the bit, or the end of the frame."""
        if self.need_data and self.cnt == self.data_after_rise:
            self.data_out = self.next_data
            self.need_data = False
        if self.cnt >= self.clk_high:
            if self.bit >= FRAME_BITS - 1:
                self.phase, self.cnt = "release", 0
                self.clk_out, self.data_out = None, None
            else:
                self.bit += 1
                self.phase, self.cnt = "low", 0
                self.clk_out = 0
        else:
            self.cnt += 1

    def _start_frame(self, t: int) -> None:
        value = self.tx[0]
        self.bits = self.frame_bits(value)
        self.bit = 0
        self.phase, self.cnt = "start", 0
        self.clk_out = None
        self.data_out = self.bits[0]       # the start bit, on an idle-high clock
        self.tx_start = t
        self.last_sent = value

    def _frame_done(self, t: int) -> None:
        self.sent.append(Ps2Byte(value=self.tx[0], tick=self.tx_start, direction="to_host"))
        self.frames_sent += 1
        self.tx.popleft()
        self._tx_reset()

    # ---- the receive side ------------------------------------------------------------
    def _receive(self, t: int, pins: Mapping[str, int]) -> None:
        clk, data = pins[self.clk_net], pins[self.data_net]
        prev_clk, prev_data = self._prev_clk, self._prev_data
        self._prev_clk, self._prev_data = clk, data
        fell = prev_clk == 1 and clk == 0
        rose = prev_clk == 0 and clk == 1
        if not self.rx_active:
            # A host frame starts with DATA falling while the clock is already low: the host's
            # request-to-send has pulled the clock down, and the start bit follows it.
            if prev_data == 1 and data == 0 and clk == 0 and self.phase == "idle":
                self.rx_active = True
                self.rx_start = t
                self.rx_bits = []
                self.rx_deadline = t + self.frame_timeout
            return
        if rose:
            self.rx_bits.append(data)
            if len(self.rx_bits) == HOST_FRAME_BITS:
                # The twelfth clock is the acknowledgement, the last thing the host clocks:
                # the host has already read it, so let go of DATA now. One tick after the
                # rising edge is inside the "stable within 1 us" rule the host->device table
                # states, and the host releases the clock without a further edge, so there is
                # no later low period to wait for.
                self._rx_done(t)
                self.data_out = None
        if fell and len(self.rx_bits) == FRAME_BITS:
            self.rx_ack = self.ack_drive         # the acknowledgement bit, in the low period
        if self.rx_ack is not None:
            if self.rx_ack <= 0:
                self.data_out = 0
                self.acks += 1
                self.rx_ack = None
            else:
                self.rx_ack -= 1
        if self.rx_active and t > self.rx_deadline:
            self.errors.append(
                f"{self.name}: host frame from t={self.rx_start} stalled after "
                f"{len(self.rx_bits)} of {HOST_FRAME_BITS} bits at t={t}"
            )
            self._rx_reset()

    def _rx_done(self, t: int) -> None:
        bits = self.rx_bits[:FRAME_BITS]
        value = 0
        for i in range(8):
            value |= (bits[i + 1] & 1) << i
        framing = bits[0] != 0 or bits[10] != 1
        parity = bits[9] != ps2_parity_bit(value)
        byte = Ps2Byte(value=value, tick=self.rx_start, direction="to_device",
                       parity_error=parity, framing_error=framing, ack=True)
        self.received.append(byte)
        self.frames_received += 1
        self.framing_errors += int(framing)
        self.parity_errors += int(parity)
        if framing or parity:
            self.errors.append(
                f"{self.name}: bad host frame 0x{value:02X} at t={self.rx_start} "
                f"({'framing' if framing else 'parity'} error), answering {RESEND:#04x}"
            )
            self.queue(RESEND)
        else:
            self.last_byte = value
            for out in self._command(value):
                self.queue(out)
        self._rx_reset()


@dataclass(frozen=True)
class Ps2Violation:
    """One rule the wire did not keep, with the tick it was missed on.

    `limit` is "min" when the measurement has to be at least `required` (a floor, like T3's
    30 us) and "max" when it has to be at most (a ceiling, like T1's 25 us or T3's 50 us).
    PS/2 has both, unlike I2C, so a checker here cannot just compare one way.
    """

    tick: int
    rule: str
    measured: int
    required: int
    limit: str = "min"
    detail: str = ""
    ns_per_tick: float = 20.0

    @property
    def ok(self) -> bool:
        if self.limit == "min":
            return self.measured >= self.required
        if self.limit == "max":
            return self.measured <= self.required
        return True          # "flag": a rule with no number in it, recorded when broken

    @property
    def measured_ns(self) -> float:
        return self.measured * self.ns_per_tick

    @property
    def required_ns(self) -> float:
        return self.required * self.ns_per_tick

    def __str__(self) -> str:
        if self.limit == "flag":
            return f"t={self.tick} {self.rule} [{self.detail}]"
        op = ">=" if self.limit == "min" else "<="
        return (f"t={self.tick} {self.rule} {self.measured} ticks {op} {self.required}"
                f" ({self.measured_ns:.2f} us {op} {self.required_ns:.2f} us)"
                + (f" [{self.detail}]" if self.detail else ""))


@dataclass
class Ps2Frame:
    """One frame the checker decoded off the pins."""

    tick: int                      # the edge that began it: DATA falling (to the host) or
                                   # CLK falling (the host's request-to-send)
    end_tick: int                  # the edge that sampled its last bit
    direction: str                 # "to_host" or "to_device"
    bits: tuple[int, ...]
    value: int
    parity_error: bool = False
    framing_error: bool = False
    ack: Optional[bool] = None     # to_device frames only: the device's 12th bit

    def __str__(self) -> str:
        text = f"t={self.tick} 0x{self.value:02X} {self.direction}"
        if self.ack is not None:
            text += " ack" if self.ack else " NO ACK"
        if self.framing_error:
            text += " framing-error"
        if self.parity_error:
            text += " parity-error"
        return text


class Ps2Checker(Device):
    """Passive PS/2 bus monitor: it drives nothing, so it sees the wire exactly as an
    analyser would, including the core's one-tick sampling latency.

    It works out the direction of every frame from the edge that started it, because the two
    directions are not the same protocol: DATA falling on an idle-high clock is a device
    starting a frame (and the bits are then sampled on the falling edges), while CLK falling
    on an idle-high DATA line is the host's request-to-send (and the bits are sampled on the
    rising edges). Every rule below follows from that, and from which side is driving.

    Rules, each with its source in `research/ref/protocols/ps2/README.md`:

    ===================  ===========  ==========================================
    rule                 limit        what it means
    ===================  ===========  ==========================================
    `tRTS`               >= 100 us    the host's request-to-send, the Gdansk page's
                                    host->device step 1. It replaces `tCLK-low` for
                                    that one low period, which is why no ceiling
                                    applies to it and why the AT table's T7 30..50 us
                                    is not applied there. A clock that goes low and
                                    high again with DATA untouched is an inhibit, not a
                                    request-to-send, and is not measured at all.
    `tCLK-low`           30..50 us    T3 / T7, the clock's inactive duration.
    `tCLK-high`          30..50 us    T4 / T8, the clock's active duration.
    `tSU;DAT`            >= 5 us      device -> host, T2: DATA may not move sooner
                                    after the rising edge.
    `tSU;DAT`            <= 1 us      host -> device: "Data must be stable within 1
                                    microsecond after the rising edge of the clock".
    `tHD;DAT`            <= 25 us     device -> host, T1: DATA moved this long before
                                    the falling edge that samples it.
    `tHD;DAT`            >= 5 us      host -> device: DATA stable this long before the
                                    rising edge that samples it. T1's 5 us floor, applied
                                    to the edge the receiver samples on; the
                                    host->device table has no setup row of its own.
    `data-window`        --           device -> host: DATA must move inside the high
                                    period, the only place T2 and T1 put it.
    `tBUS-FREE`          >= 50 us     "After sending stop bit, the keyboard waits for at
                                    least 50us, until it transmits next data" (Gdansk
                                    page), measured from the *rising* edge that began
                                    the clock carrying the last bit -- the one edge that
                                    says when the frame stopped being clocked -- to the
                                    first edge of the next frame. T5's own ">0/50 us" is
                                    the host's inhibit window rather than a floor on
                                    the device, and lives in `inhibit_window`.
    `start-bit`          --           bit 0 of every frame is 0.
    `stop-bit`           --           bit 10 of every frame is 1.
    `parity`             --           the 8 data bits and the parity bit hold an odd
                                    number of 1s.
    `ack-bit`            --           a host->device frame has a twelfth bit and the
                                    device has to pull DATA low for it.
    `frame-length`       --           a frame that starts has to finish: one that stops
                                    after fewer bits than it has is reported, with the
                                    tick it ran out of patience on. A frame with *more*
                                    bits than it should has no such luck -- the extra edge
                                    closes the frame and opens a phantom request-to-send,
                                    which is what `tBUS-FREE` reports.
    ===================  ===========  ==========================================

    One rule is deliberately not here: "the device may not transmit while the clock reads
    low". On the wire that looks exactly like the host's own request-to-send, so a monitor
    cannot judge it -- `Ps2Device` obeys it instead and counts the ticks it spent waiting in
    `inhibited`.

    `shortest_period` and `longest_period` are the clock periods actually measured, in ticks
    between consecutive rising edges *inside one frame*, so a test can read the line rate off
    the wire; `fastest_khz` and `slowest_khz` are the same in kbit/s, and `periods` is how
    many there were. A rise is only paired with the rise before it when both are in the same
    frame: the gap between two frames is a bus-free wait or a request-to-send, not a clock
    period, and the first period of a frame has nothing to pair with.
    """

    name = "ps2_check"

    #: Every rule name, in the order `report()` lists them.
    RULES = ("tRTS", "tCLK-low", "tCLK-high", "tSU;DAT", "tHD;DAT", "data-window",
             "tBUS-FREE", "start-bit", "stop-bit", "parity", "ack-bit", "frame-length")

    def __init__(
        self,
        rate: str = "fast",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        clk_net: str = CLK_NET,
        data_net: str = DATA_NET,
        name: Optional[str] = None,
    ):
        self.rate = rate
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.clk_net, self.data_net = clk_net, data_net
        if name:
            self.name = name
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        m = rate_minimums(self.ns_per_tick, self.rate, self.timing_overrides)
        self.min_ticks = m
        self.frame_timeout = 4 * (HOST_FRAME_BITS * (m["clk_low_max"] + m["clk_high_max"])
                                  + 2 * m["bus_free"])

    def bind(self, bus: Bus) -> "Ps2Checker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        self._reset()
        return self

    def _reset(self) -> None:
        self.violations: list[Ps2Violation] = []
        self.frames: list[Ps2Frame] = []
        self.frames_to_host = 0
        self.frames_to_device = 0
        self.acks = 0
        self.no_acks = 0
        self.periods = 0
        self.shortest_period: Optional[int] = None
        self.longest_period: Optional[int] = None
        self._prev_clk = 1
        self._prev_data = 1
        self._rise_confirmed: Optional[int] = None
        self._rise_frame = -1                  # the frame the confirmed rise belongs to
        self.last_frame_end: Optional[int] = None
        self._reset_frame()

    def _reset_frame(self) -> None:
        self.direction: Optional[str] = None
        self.frame_start = -1
        self.bits: list[int] = []
        self.rise_tick = 0
        # None, not 0: the clock was already high when the checker was attached, so the first
        # high period has nothing to pair with and is not measured (as in i2c.py).
        self.high_since: Optional[int] = None  # tick the current high period began
        self.low_since: Optional[int] = None    # tick the current low period began
        self.data_in_high: Optional[int] = None  # last DATA move inside this high period
        self.data_in_low: Optional[int] = None   # last DATA move inside this low period
        self.frame_open = False
        self.saw_start = False       # a host frame's start bit went down while the clock was low

    # ---- what the tests ask for -------------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    def violations_by_rule(self, rule: str) -> list[Ps2Violation]:
        return [v for v in self.violations if v.rule == rule]

    @property
    def rules_broken(self) -> set:
        return {v.rule for v in self.violations}

    @property
    def fastest_khz(self) -> Optional[float]:
        """The fastest clock the wire actually ran at, in kilocycles per second."""
        if self.shortest_period is None:
            return None
        return 1e6 / (self.ns_per_tick * self.shortest_period)

    @property
    def slowest_khz(self) -> Optional[float]:
        """The slowest clock the wire actually ran at, in kilocycles per second."""
        if self.longest_period is None:
            return None
        return 1e6 / (self.ns_per_tick * self.longest_period)

    def report(self, limit: int = 8) -> str:
        head = (f"PS/2 {self.rate} at {self.ns_per_tick:g} ns/tick: "
                f"{self.frames_to_host} frame(s) to the host, {self.frames_to_device} to the "
                f"device, {self.acks} ack(s), {len(self.violations)} violation(s)")
        if self.shortest_period is not None:
            head += (f", clock period {self.shortest_period}-{self.longest_period} ticks "
                     f"({self.fastest_khz:.2f}-{self.slowest_khz:.2f} kbit/s)")
        lines = [head]
        for rule in self.RULES:
            hits = self.violations_by_rule(rule)
            if not hits:
                continue
            lines.append(f"  {rule}: {len(hits)} violation(s)")
            lines += [f"    {v}" for v in hits[:limit]]
        return "\n".join(lines)

    # ---- the monitor ------------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        clk, data = pins[self.clk_net], pins[self.data_net]
        prev_clk, prev_data = self._prev_clk, self._prev_data
        rose = prev_clk == 0 and clk == 1
        fell = prev_clk == 1 and clk == 0
        if rose:
            self._on_rise(t, data)
        if fell:
            self._on_fall(t, data)
        if prev_data != data:
            self._on_data(t, data, clk)
        if self.frame_open and t - self.frame_start > self.frame_timeout:
            self._flag(t, "frame-length",
                       f"the frame from t={self.frame_start} never finished: "
                       f"{len(self.bits)} of {self._expected()} bits and no clock edge")
            self._reset_frame()
        self._prev_clk, self._prev_data = clk, data
        return {}

    def _expected(self) -> int:
        """Clocks in a frame: 11 to the host, 12 to the device (the extra one is the ack)."""
        return HOST_FRAME_BITS if self.direction == "to_device" else FRAME_BITS

    def _min(self, key: str) -> int:
        return self.min_ticks[key]

    def _violation(self, t: int, rule: str, measured: int, required: int,
                   limit: str, detail: str = "") -> None:
        if limit == "min" and measured >= required:
            return
        if limit == "max" and measured <= required:
            return
        self.violations.append(
            Ps2Violation(t, rule, measured, required, limit, detail, self.ns_per_tick))

    def _flag(self, t: int, rule: str, detail: str) -> None:
        """A rule with no number in it (start bit, parity, ack bit, frame length)."""
        self.violations.append(Ps2Violation(t, rule, 0, 0, "flag", detail, self.ns_per_tick))

    def _period(self, t: int) -> None:
        """The clock period between consecutive rising edges, for fastest_khz/slowest_khz.

        Only rises inside one frame are paired: the gap between two frames is the bus-free
        wait or a request-to-send, neither of which is one of T4's active durations, so
        counting it would report a "clock period" far longer than the protocol allows.
        """
        if self._rise_confirmed is not None and self._rise_frame == self.frame_start:
            period = t - self._rise_confirmed
            if period > 0:
                self.periods += 1
                self.shortest_period = (period if self.shortest_period is None
                                        else min(self.shortest_period, period))
                self.longest_period = (period if self.longest_period is None
                                       else max(self.longest_period, period))
        self._rise_confirmed = t
        self._rise_frame = self.frame_start

    def _on_rise(self, t: int, data: int) -> None:
        if self.low_since is not None:
            low = t - self.low_since
            if self.frame_open and self.direction == "to_device" and not self.bits:
                # The first low period of a host frame is the request-to-send, which the AT
                # table does not bound at 30..50 us: the Gdansk page asks for 100 us of it.
                # Only if the start bit followed, though: a clock that goes low and high again
                # with DATA untouched is the host inhibiting the device ("The system can hold
                # the 'clock' signal inactive to inhibit the next transmission"), and that is
                # legal at any time and for as long as it likes.
                if self.saw_start:
                    self._violation(t, "tRTS", low, self._min("rts_hold"), "min",
                                    "the clock stayed low before the start bit")
            else:
                self._violation(t, "tCLK-low", low, self._min("clk_low_min"), "min",
                                "clock inactive (T3/T7)")
                self._violation(t, "tCLK-low", low, self._min("clk_low_max"), "max",
                                "clock inactive (T3/T7)")
        self._period(t)
        self.high_since, self.low_since = t, None
        self.rise_tick = t
        if self.frame_open and self.direction == "to_device":
            if not self.bits and not self.saw_start:
                # The clock went low and came back with DATA untouched and no start bit: the
                # host was inhibiting the device, which the AT Technical Reference allows it to
                # do at any time. Nothing was framed, so there is nothing to decode or to time.
                self._reset_frame()
                return
            if self.data_in_low is not None:
                self._violation(t, "tHD;DAT", t - self.data_in_low,
                                self._min("data_before_edge_min"), "min",
                                f"DATA moved at t={self.data_in_low}, too close to the rising "
                                f"edge that samples it")
            self._sample(t, data)
        self.data_in_low, self.data_in_high = None, None

    def _on_fall(self, t: int, data: int) -> None:
        # Only inside a frame: between frames the clock is just idle high, and that gap is not
        # one of T4's active durations.
        if self.high_since is not None and self.frame_open:
            high = t - self.high_since
            self._violation(t, "tCLK-high", high, self._min("clk_high_min"), "min",
                            "clock active (T4/T8)")
            self._violation(t, "tCLK-high", high, self._min("clk_high_max"), "max",
                            "clock active (T4/T8)")
        if self.frame_open and self.direction == "to_host" and self.data_in_high is not None:
            gap = t - self.data_in_high
            self._violation(t, "tHD;DAT", gap, self._min("data_before_edge_min"), "min",
                            f"DATA moved at t={self.data_in_high}, too close to the falling "
                            f"edge that samples it (T1)")
            self._violation(t, "tHD;DAT", gap, self._min("data_before_edge_max"), "max",
                            f"DATA moved at t={self.data_in_high}, too long before the "
                            f"falling edge that samples it (T1)")
        self.high_since, self.low_since = None, t
        self.data_in_high = None
        if not self.frame_open:
            if data == 1:
                self._begin(t, "to_device")          # the host's request-to-send
            else:
                self._flag(t, "start-bit",
                           "DATA was already low when the clock started: a frame begins with "
                           "the start bit on an idle-high clock, or with the host's "
                           "request-to-send on an idle-high DATA line")
                self._begin(t, "to_host")
            return
        if self.direction == "to_host":
            self._sample(t, data)

    def _on_data(self, t: int, data: int, clk: int) -> None:
        if clk == 1:
            if not self.frame_open:
                if data == 0:
                    self._begin(t, "to_host")         # the device put a start bit on the wire
                return
            if self.direction == "to_host":
                self._violation(t, "tSU;DAT", t - self.rise_tick,
                                self._min("data_after_rise_min"), "min",
                                "DATA moved too soon after the clock's rising edge (T2)")
            else:
                self._violation(t, "tSU;DAT", t - self.rise_tick,
                                self._min("data_after_rise_max"), "max",
                                "DATA moved after the clock's rising edge: the host must "
                                "have it stable within 1 us of the rise")
            self.data_in_high = t
            return
        if self.frame_open and self.direction == "to_device":
            self.saw_start = True
        if self.frame_open and self.direction == "to_host":
            self._flag(t, "data-window",
                       f"DATA moved {t - (self.low_since or t)} ticks into the clock low "
                       f"period: in a device->host frame it belongs inside the high period, "
                       f"between T2 and T1")
        self.data_in_low = t

    def _begin(self, t: int, direction: str) -> None:
        """A frame starts: check the bus was free, then start decoding."""
        if self.last_frame_end is not None and self._rise_confirmed is not None:
            # From the rising edge that opened the frame's last clock, not from the falling edge
            # that sampled its last bit: the clocking of that bit runs on past the sample, and
            # the Gdansk page's 50 us is what the device waits after it. No edge follows the last
            # clock, so that rising edge is the closest thing on the wire to "the frame ended".
            self._violation(t, "tBUS-FREE", t - self._rise_confirmed,
                            self._min("bus_free_min"), "min",
                            f"the next frame began {t - self._rise_confirmed} ticks after the "
                            f"previous frame's last clock began; a device waits at least "
                            f"{self._min('bus_free_min')} ticks after its stop bit. A device "
                            f"that clocks more bits than its frame has also lands here")
        self.frame_open = True
        self.direction = direction
        self.frame_start = t
        self.bits = []
        self.saw_start = False
        if direction == "to_device":
            # The frame began on a falling edge, so the clock has been low since this tick: that
            # low period is the request-to-send, and it is what `tRTS` measures.
            self.high_since, self.low_since = None, t
        else:
            # The device put a start bit on an idle-high clock, so the clock reads high from
            # here until its first falling edge, and that high period is what `tCLK-high` sees.
            self.high_since, self.low_since = t, None
        self.data_in_high = None
        self.data_in_low = None

    def _sample(self, t: int, bit: int) -> None:
        """One bit, sampled on the edge this frame's direction says samples it.

        A frame closes as soon as it has its eleven (or twelve) bits, so a device that clocks
        *more* than that leaves a falling edge with no frame open. That edge is
        unattributable -- it is the shape of a host's request-to-send -- and what the checker
        can say about it is that the bus came back far too soon, which is `tBUS-FREE`. The
        opposite fault, a frame that stops early, is this rule's `frame-length` timeout.
        """
        self.bits.append(bit & 1)
        if len(self.bits) == self._expected():
            self._close(t)

    def _close(self, t: int) -> None:
        bits = tuple(self.bits)
        value = 0
        for i in range(8):
            value |= (bits[i + 1] & 1) << i
        parity = bits[9] != ps2_parity_bit(value)
        framing = bits[0] != 0 or bits[10] != 1
        ack: Optional[bool] = None
        if self.direction == "to_device":
            ack = bits[11] == 0
            self.acks += int(ack)
            self.no_acks += int(not ack)
        frame = Ps2Frame(tick=self.frame_start, end_tick=t, direction=self.direction, bits=bits,
                         value=value, parity_error=parity, framing_error=framing, ack=ack)
        self.frames.append(frame)
        if self.direction == "to_host":
            self.frames_to_host += 1
        else:
            self.frames_to_device += 1
        if bits[0] != 0:
            self._flag(t, "start-bit", f"bit 0 was 1: {frame}")
        if bits[10] != 1:
            self._flag(t, "stop-bit", f"bit 10 was 0: {frame}")
        if parity:
            self._flag(t, "parity", f"the odd-parity bit does not fit 0x{value:02X}: {frame}")
        if ack is False:
            self._flag(t, "ack-bit", f"the device left DATA high for the ack bit: {frame}")
        self.last_frame_end = t
        self._reset_frame()


#: key name -> (make code in scan code set 1, make code in scan code set 2). From the OSDev
#: tables saved in research/ref/protocols/ps2/osdev-ps2-keyboard-commands.md. Set 3 uses the
#: set 2 make codes for every key here, which is what the page's set 3 table lists.
SCAN_CODES: dict[str, tuple[int, int]] = {
    **{letter: (s1, s2) for letter, s1, s2 in zip(
        "abcdefghijklmnopqrstuvwxyz",
        (0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23, 0x17, 0x24, 0x25, 0x26, 0x32,
         0x31, 0x18, 0x19, 0x10, 0x13, 0x1F, 0x14, 0x16, 0x2F, 0x11, 0x2D, 0x15, 0x2C),
        (0x1C, 0x32, 0x21, 0x23, 0x24, 0x2B, 0x34, 0x33, 0x43, 0x3B, 0x42, 0x4B, 0x3A,
         0x31, 0x44, 0x4D, 0x15, 0x2D, 0x1B, 0x2C, 0x3C, 0x2A, 0x1D, 0x22, 0x35, 0x1A))},
    **{str(d): (s1, s2) for d, s1, s2 in zip(
        "0123456789",
        (0x0B, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A),
        (0x45, 0x16, 0x1E, 0x26, 0x25, 0x2E, 0x36, 0x3D, 0x3E, 0x46))},
    "space": (0x39, 0x29), "enter": (0x1C, 0x5A), "backspace": (0x0E, 0x66),
    "tab": (0x0F, 0x0D), "escape": (0x01, 0x76), "capslock": (0x3A, 0x58),
    "shift_l": (0x2A, 0x12), "shift_r": (0x36, 0x59),
    "ctrl_l": (0x1D, 0x14), "ctrl_r": (0x1D, 0x14),
    "alt_l": (0x38, 0x11), "alt_r": (0x38, 0x11),
    **{f"f{n}": (s1, s2) for n, s1, s2 in zip(
        range(1, 13),
        (0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x57, 0x58),
        (0x05, 0x06, 0x04, 0x0C, 0x03, 0x0B, 0x83, 0x0A, 0x01, 0x09, 0x78, 0x07))},
    "up": (0x48, 0x48), "down": (0x50, 0x50), "left": (0x4B, 0x4B), "right": (0x4D, 0x4D),
    "home": (0x47, 0x6C), "end": (0x4F, 0x69), "page_up": (0x49, 0x7D),
    "page_down": (0x51, 0x7A), "insert": (0x52, 0x70), "delete": (0x53, 0x71),
    "kp_enter": (0x1C, 0x5A), "kp_plus": (0x4E, 0x79), "kp_minus": (0x4D, 0x7B),
    "kp_mul": (0x37, 0x7C), "kp_div": (0x35, 0x4A), "kp_dot": (0x53, 0x71),
    "kp_0": (0x52, 0x70), "kp_1": (0x4F, 0x69), "kp_2": (0x50, 0x72), "kp_3": (0x51, 0x7B),
    "kp_4": (0x4B, 0x6B), "kp_5": (0x4C, 0x6C), "kp_6": (0x4D, 0x6D), "kp_7": (0x4C, 0x6E),
    "kp_8": (0x4D, 0x6F), "kp_9": (0x4E, 0x71),
}

#: Keys whose scan code set 2 form carries the 0xE0 prefix. In set 1 the keyboard sends the
#: plain code and the *controller* emits 0xE0 when it translates, which is not this model's job.
EXTENDED_KEYS = frozenset({
    "ctrl_r", "alt_r", "up", "down", "left", "right", "home", "end", "page_up",
    "page_down", "insert", "delete", "kp_enter", "kp_div",
})

#: The typematic byte a keyboard loads from 0xF6 ("Set Default"): bits 5..6 = 500 ms delay,
#: bits 0..4 = 10.9 cps, which is 0b01011 01 = 0x2B.
DEFAULT_TYPEMATIC = 0x2B


class Ps2Keyboard(Ps2Device):
    """A PS/2 keyboard: the scan code sets, the LED byte and the command set from the OSDev
    tables, on top of `Ps2Device`'s wire.

    `press("a")` queues the make code of the key and `release("a")` its break code, but only
    while scanning is on (0xF4 turned it on; after a reset it is on). Nothing is queued while
    the keyboard is disabled, which is what 0xF5 does, so a test can watch the difference.
    Names come from `SCAN_CODES`; an unknown name raises KeyError with the list of names.

        kbd = attach_keyboard(bus)
        kbd.press("ctrl_l"); kbd.press("c"); kbd.release("c"); kbd.release("ctrl_l")
        # the host sees 0x14 0x21 0xF0 0x21 0xF0 0x14 in scan code set 2

    Commands: 0xFF reset (0xFA then the self-test byte), 0xF6 set defaults, 0xF5 disable,
    0xF4 enable, 0xF3 typematic byte, 0xF2 identify (0xFA, 0xAB, 0x83), 0xF0 get/set the scan
    code set, 0xED set the LEDs, 0xEE echo, 0xFE resend, and 0xFA..0xFD, the set 3 key-type
    commands, which are acknowledged and noted in `key_types`. An argument byte is consumed
    by the byte after its command, and every command answers 0xFA first -- except 0xEE, which
    answers 0xEE, and a command that takes an argument, which answers 0xFA twice, once for
    the command and once for the argument.
    """

    name = "ps2_kbd"

    def __init__(self, *args, scan_code_set: int = 2, leds: int = 0,
                 typematic: int = DEFAULT_TYPEMATIC, enabled: bool = True,
                 selftest: int = SELFTEST_OK, id_bytes=ID_BYTE_KBD, **kwargs):
        super().__init__(*args, **kwargs)
        if scan_code_set not in (1, 2, 3):
            raise ValueError(f"scan_code_set must be 1, 2 or 3, got {scan_code_set!r}")
        self.scan_code_set = scan_code_set
        self.leds = leds & 0x07
        self.typematic = typematic
        self.enabled = enabled
        self.selftest = selftest
        self.id_bytes = tuple(id_bytes)
        self.expect: Optional[str] = None
        self.key_types: dict[int, str] = {}
        self.down: list[str] = []
        self.pressed: set = set()

    @property
    def caps_lock(self) -> bool:
        return bool(self.leds & 0x04)

    @property
    def num_lock(self) -> bool:
        return bool(self.leds & 0x02)

    @property
    def scroll_lock(self) -> bool:
        return bool(self.leds & 0x01)

    def make_code(self, key: str) -> int:
        """The one-byte make code of `key` in the selected scan code set."""
        try:
            set1, set2 = SCAN_CODES[key]
        except KeyError:
            raise KeyError(f"no scan code for key {key!r}; try one of {sorted(SCAN_CODES)}") \
                from None
        return set1 if self.scan_code_set == 1 else set2

    def codes(self, key: str, make: bool = True) -> tuple[int, ...]:
        """Every byte `key` puts on the wire, in order.

        A make code is the code itself. A break code is the set 1 code with bit 7 set, or the
        set 2/3 pair 0xF0 followed by the make code. Set 2 and set 3 prefix an extended key
        with 0xE0 on both halves of the pair.
        """
        code = self.make_code(key)
        prefix = (0xE0,) if key in EXTENDED_KEYS and self.scan_code_set != 1 else ()
        if make:
            return prefix + (code,)
        if self.scan_code_set == 1:
            return prefix + (code | 0x80,)
        return prefix + (0xF0, code)

    def press(self, key: str) -> tuple[int, ...]:
        """Queue the make code of `key`. Returns the bytes queued (none while disabled)."""
        self.pressed.add(key)
        if key not in self.down:
            self.down.append(key)
        if not self.enabled:
            return ()
        codes = self.codes(key, make=True)
        self.queue(*codes)
        return codes

    def release(self, key: str) -> tuple[int, ...]:
        """Queue the break code of `key`. Returns the bytes queued."""
        self.pressed.discard(key)
        if key in self.down:
            self.down.remove(key)
        if not self.enabled:
            return ()
        codes = self.codes(key, make=False)
        self.queue(*codes)
        return codes

    def tap(self, key: str) -> tuple[int, ...]:
        """Make then break, both queued in one go."""
        return self.press(key) + self.release(key)

    def _defaults(self) -> None:
        self.scan_code_set = 2
        self.leds = 0
        self.typematic = DEFAULT_TYPEMATIC
        self.key_types = {}

    def _command(self, value: int) -> list[int]:
        if self.expect is not None:
            kind, self.expect = self.expect, None
            return self._argument(kind, value)
        if value == 0xFF:                                  # reset and self test
            self._defaults()
            self.enabled = True
            return [ACK, self.selftest]
        if value == 0xF6:                                  # set default parameters
            self._defaults()
            return [ACK]
        if value == 0xF5:                                  # disable scanning
            self.enabled = False
            return [ACK]
        if value == 0xF4:                                  # enable scanning
            self.enabled = True
            return [ACK]
        if value == 0xF2:                                  # identify
            return [ACK, *self.id_bytes]
        if value == 0xF3:                                  # typematic rate and delay
            self.expect = "typematic"
            return [ACK]
        if value == 0xF0:                                  # get or set the scan code set
            self.expect = "scan_code_set"
            return [ACK]
        if value == 0xED:                                  # set the LEDs
            self.expect = "leds"
            return [ACK]
        if value in (0xF7, 0xF8, 0xF9, 0xFA):
            self.key_types[value] = "all keys"
            return [ACK]
        if value in (0xFB, 0xFC, 0xFD):
            self.expect = "key_type"
            return [ACK]
        return super()._command(value)

    def _argument(self, kind: str, value: int) -> list[int]:
        if kind == "typematic":
            if value & 0x80:
                self.errors.append(f"{self.name}: typematic byte 0x{value:02X} has bit 7 set")
            self.typematic = value
            return [ACK]
        if kind == "scan_code_set":
            if value == 0:
                return [ACK, self.scan_code_set]
            if value in (1, 2, 3):
                self.scan_code_set = value
                return [ACK]
            self.errors.append(f"{self.name}: scan code set {value} is not 1, 2 or 3")
            return [ACK, RESEND]
        if kind == "leds":
            self.leds = value & 0x07
            return [ACK]
        if kind == "key_type":
            self.key_types[value] = f"key 0x{value:02X}"
            return [ACK]
        raise RuntimeError(f"{self.name}: bad argument kind {kind!r}")


@dataclass
class Ps2MousePacket:
    """One movement report, in the layout the OSDev packet table gives.

    Byte 0 carries the two overflow bits, the two sign bits of the 9-bit X and Y offsets, the
    always-one bit and the three buttons; bytes 1 and 2 hold the low eight bits of X and Y. The
    sign bits belong to the *low* byte: the page spells the sign extension out, and
    `wire()` does it the same way.
    """

    x: int = 0
    y: int = 0
    buttons: int = 0            # bit 0 left, bit 1 right, bit 2 middle
    x_overflow: bool = False
    y_overflow: bool = False
    z: Optional[int] = None     # the Intellimouse wheel, when the device has one

    @property
    def wire(self) -> bytes:
        """The bytes for this report: 3, or 4 with a wheel."""
        x, y = self.x, self.y
        head = 0x08                                   # "1 | Always One"
        head |= (self.buttons & 0x07)
        head |= 0x10 if x < 0 else 0
        head |= 0x20 if y < 0 else 0
        head |= 0x40 if self.x_overflow else 0
        head |= 0x80 if self.y_overflow else 0
        out = bytes([head, x & 0xFF, y & 0xFF])
        if self.z is not None:
            out += bytes([self.z & 0xFF])
        return out

    @property
    def signed(self) -> tuple[int, int]:
        """(x, y) the way a host decodes them: the head's sign bit extends the low byte.

        This is the arithmetic the OSDev page spells out, byte for byte:
        ``rel_x = d - ((state << 4) & 0x100)``, ``rel_y = d - ((state << 3) & 0x100)``.
        """
        head, xb, yb = self.wire[0], self.wire[1], self.wire[2]
        return xb - ((head << 4) & 0x100), yb - ((head << 3) & 0x100)


class Ps2Mouse(Ps2Device):
    """A PS/2 mouse: the command set and the 3-byte movement packet from the OSDev tables.

    `move(dx, dy)` builds a report and queues it; like a keyboard's `press`, it queues nothing
    while reporting is off, which 0xF5 turns off and 0xF4 turns on. The sample rate, the
    resolution (1, 2, 4 or 8 counts/mm) and the scaling (1:1 or 2:1) all change what goes on
    the wire, exactly as the 0xF3, 0xE8 and 0xE6/0xE7 rows of the command table say.

        mouse = attach_mouse(bus)
        mouse.move(10, -5, buttons=1)          # left button, 10 right and 5 up
        # the host sees 0x09 0x0A 0xFB, and reads x = +10, y = -5

    `wheel=True` adds the Intellimouse Z packet: the 200/100/80 sample-rate magic switches the
    device ID from 0x00 to 0x03 and every report grows a fourth byte. `packet_bytes` is what a
    test asserts on, so it works with or without the wheel.

    Remote mode (0xF0) is modelled as "reporting only happens when the host asks": in remote
    mode `move()` records the movement in `pending_report` and nothing goes on the wire until
    0xEB reads the data. The saved sources give the command but not its timing rules, so this
    is the model's choice and it is documented here rather than hidden.
    """

    name = "ps2_mouse"

    def __init__(self, *args, resolution: int = 1, sample_rate: int = 100,
                 scaling: int = 1, enabled: bool = True, stream: bool = True,
                 wrap: bool = False, wheel: bool = False, id_byte: int = ID_BYTE_MOUSE,
                 selftest: int = SELFTEST_OK, **kwargs):
        super().__init__(*args, **kwargs)
        if resolution not in (1, 2, 4, 8):
            raise ValueError(f"resolution must be 1, 2, 4 or 8 counts/mm, got {resolution!r}")
        if sample_rate not in (10, 20, 40, 60, 80, 100, 200):
            raise ValueError(
                f"sample_rate must be one of 10, 20, 40, 60, 80, 100, 200, got {sample_rate!r}")
        if scaling not in (1, 2):
            raise ValueError(f"scaling must be 1 (1:1) or 2 (2:1), got {scaling!r}")
        self.resolution = resolution
        self.sample_rate = sample_rate
        self.scaling = scaling
        self.enabled = enabled
        self.stream = stream
        self.wrap = wrap
        self.wheel = wheel
        self.id_byte = id_byte
        self.selftest = selftest
        self.expect: Optional[str] = None
        self.buttons = 0
        self._magic: list[int] = []          # the sample rates of the Intellimouse magic
        self.reports: list[Ps2MousePacket] = []
        self.pending_report: Optional[Ps2MousePacket] = None
        self.x = 0
        self.y = 0

    @property
    def packet_bytes(self) -> int:
        """Bytes in one report: 3, or 4 with the Intellimouse wheel."""
        return 4 if self.wheel else 3

    def _scale(self, value: int) -> int:
        """Counts on the wire for `value` counts of movement, resolution and scaling applied."""
        counts = int(value) * (self.resolution // 1) * self.scaling
        if self.wrap:
            # Wrap mode reports movement inside the 9-bit range only; anything past it is
            # dropped rather than wrapped around, which is the one thing the saved sources
            # are clear about ("a 9-Bit X-Axis Relative Offset").
            counts = max(-256, min(255, counts))
        return max(-256, min(255, counts))

    def move(self, dx: int = 0, dy: int = 0, buttons: Optional[int] = None,
             z: Optional[int] = None) -> Optional[Ps2MousePacket]:
        """Build a report and queue it, if the mouse is reporting. Returns the packet.

        `buttons` is the pressed set: bit 0 left, bit 1 right, bit 2 middle. Movement that
        does not fit the 8-bit field sets the matching overflow bit, as the packet table's
        `xo`/`yo` say.
        """
        if buttons is not None:
            self.buttons = buttons & 0x07
        x, y = self._scale(dx), self._scale(dy)
        packet = Ps2MousePacket(
            x=x, y=y, buttons=self.buttons,
            x_overflow=not (-256 <= x <= 255), y_overflow=not (-256 <= y <= 255),
            z=(z if self.wheel else None),
        )
        self.reports.append(packet)
        self.x, self.y = self.x + x, self.y + y
        if not self.enabled:
            return packet
        if not self.stream:
            self.pending_report = packet       # remote mode: 0xEB reads it
            return packet
        self.queue(*packet.wire)
        return packet

    def _command(self, value: int) -> list[int]:
        if self.expect is not None:
            kind, self.expect = self.expect, None
            return self._argument(kind, value)
        if value == 0xFF:                        # reset: ack, self test, then the device ID
            self._defaults()
            self.enabled = True
            return [ACK, self.selftest, self.id_byte]
        if value == 0xF6:                        # set defaults
            self._defaults()
            return [ACK]
        if value == 0xF5:                        # disable data reporting
            self.enabled = False
            return [ACK]
        if value == 0xF4:                        # enable data reporting
            self.enabled = True
            return [ACK]
        if value == 0xF2:                        # get device ID
            return [ACK, self.id_byte]
        if value == 0xF3:                        # set sample rate
            self.expect = "sample_rate"
            return [ACK]
        if value == 0xE8:                        # set resolution
            self.expect = "resolution"
            return [ACK]
        if value == 0xE6:                        # scaling 1:1
            self.scaling = 1
            return [ACK]
        if value == 0xE7:                        # scaling 2:1
            self.scaling = 2
            return [ACK]
        if value == 0xEA:                        # stream mode
            self.stream = True
            return [ACK]
        if value == 0xF0:                        # remote mode
            self.stream = False
            return [ACK]
        if value == 0xEE:                        # set wrap mode
            self.wrap = True
            return [ACK]
        if value == 0xEC:                        # reset wrap mode
            self.wrap = False
            return [ACK]
        if value == 0xEB:                        # read data
            return [ACK, *(self.pending_report.wire
                           if self.pending_report else self.reports[-1].wire)]
        if value == 0xE9:                        # status request
            return [ACK, self._status()]
        return super()._command(value)

    def _defaults(self) -> None:
        self.resolution, self.scaling = 1, 1
        self.sample_rate, self.stream = 100, True
        self.wrap, self.enabled = False, True
        self.buttons = 0
        self._magic = []
        self.pending_report = None

    def _status(self) -> int:
        """Byte 0 of the status request: bit 0 the always-one bit, bits 4..5 the buttons.

        The OSDev mouse page does not give the status byte's layout, so this is the model's
        own: it puts the packet's always-one bit and button state where a host would look for
        them, and the numbers are documented rather than claimed to be IBM's.
        """
        return 0x01 | ((self.buttons & 0x07) << 4)

    def _argument(self, kind: str, value: int) -> list[int]:
        if kind == "sample_rate":
            if value not in (10, 20, 40, 60, 80, 100, 200):
                self.errors.append(f"{self.name}: {value} is not a valid sample rate")
                return [ACK, RESEND]
            # The Intellimouse magic: 200, 100, 80 in a row turns the wheel on and the ID up.
            if value == 200:
                self._magic.append(200)
            elif value == 100 and self._magic[-1:] == [200]:
                self._magic.append(100)
            elif value == 80 and self._magic[-2:] == [200, 100]:
                self._magic.append(80)
                self.wheel = True
                self.id_byte = 0x03
            else:
                self._magic = []
            self.sample_rate = value
            return [ACK]
        if kind == "resolution":
            if value not in (0, 1, 2, 3):
                self.errors.append(f"{self.name}: resolution {value} is not 0..3")
                return [ACK, RESEND]
            self.resolution = (1, 2, 4, 8)[value]
            return [ACK]
        raise RuntimeError(f"{self.name}: bad argument kind {kind!r}")


# ---- harness helpers ---------------------------------------------------------------------
def _ensure_nets(bus: Bus) -> None:
    """Add the two open-drain PS/2 nets if they are not there yet."""
    for net in (CLK_NET, DATA_NET):
        if net not in bus.nets:
            bus.add_net(net)


def attach_host(bus: Bus, **kwargs) -> Ps2Host:
    """Build a Ps2Host that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    host = Ps2Host(bus=bus, **kwargs)
    bus.attach(host)
    return host


def attach_device(bus: Bus, **kwargs) -> Ps2Device:
    """Build a bare Ps2Device (shared commands only) that takes its timing from `bus`."""
    _ensure_nets(bus)
    dev = Ps2Device(bus=bus, **kwargs)
    bus.attach(dev)
    return dev


def attach_keyboard(bus: Bus, **kwargs) -> Ps2Keyboard:
    """Build a Ps2Keyboard that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    kbd = Ps2Keyboard(bus=bus, **kwargs)
    bus.attach(kbd)
    return kbd


def attach_mouse(bus: Bus, **kwargs) -> Ps2Mouse:
    """Build a Ps2Mouse that takes its timing from `bus` and attach it."""
    _ensure_nets(bus)
    mouse = Ps2Mouse(bus=bus, **kwargs)
    bus.attach(mouse)
    return mouse


def attach_checker(bus: Bus, **kwargs) -> Ps2Checker:
    """Build a passive Ps2Checker for `bus` and attach it."""
    _ensure_nets(bus)
    chk = Ps2Checker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk


def frame_ticks(ns_per_tick: float = 20.0, rate: str = "fast",
                direction: str = "to_host") -> int:
    """Roughly how many ticks one frame takes on the wire, for sizing a `run` limit.

    A device->host frame is 11 clocks plus the bus-free wait, a host->device frame is the same
    plus the request-to-send and the acknowledgement clock. Measured runs are a little longer
    than this; the exact figure is `Ps2Sent.start_tick`/`end_tick` in a run.
    """
    m = rate_minimums(ns_per_tick, rate)
    if direction == "to_device":
        return m["rts_hold"] + HOST_FRAME_BITS * (m["clk_low"] + m["clk_high"]) + m["bus_free"]
    return (m["start_bit_hold"] + FRAME_BITS * (m["clk_low"] + m["clk_high"]) + m["bus_free"])

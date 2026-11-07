"""Pin-level NEC IR reference models on the tick-based bus in verif/vip/core.py.

NEC is an infrared remote-control protocol, and this file models it the way a
receiver sees it: **after demodulation**, on one logic net. The net is high
while the 38 kHz carrier is on -- a *mark*, or burst -- and low while it is off
-- a *space*, or gap. There is no carrier on this net and the checker does not
look for one; the carrier numbers in the sources disagree (38 kHz vs 38.222 kHz)
and a demodulated pin cannot show it anyway.

    bus = Bus(ns_per_tick=20)                       # 50 MHz
    tx = attach_tx(bus)                             # the remote
    rx = attach_rx(bus)                             # the decoder
    chk = attach_checker(bus)                       # optional, passive
    tx.press(0x00, 0x0C, repeats=3)                 # 12 ms of VOLUME+ held
    bus.run_until(lambda b: not tx.busy and rx.pending == 4, 20_000_000)
    assert rx.address == 0x00 and rx.command == 0x0C
    assert rx.repeats == 3 and chk.clean, chk.report()

The wire, in full. Every time is the nominal from
research/ref/protocols/necir/README.md, which cites the saved source text for
each one; `necir_timings()` widens them into windows and this module only ever
compares against those windows.

    mark 9 ms      space 4.5 ms   32 data bits, LSB first,      trailing
    (leader, AGC)                 each = 562.5 us mark +         562.5 us
                                    562.5 us ('0') or             mark
                                    1687.5 us ('1') space
    word 0: address      word 1: ~address      word 2: command      word 3: ~command

    repeat code, every 110 ms start to start while a key is held:
    mark 9 ms      space 2250 us     mark 562.5 us      (then idle)

Three devices, all passive except the transmitter:

  NecirTx        builds frames and repeat codes from queued (address, command)
                 pairs and drives the net. `timing=` overrides any nominal, and
                 `segments()` is the one method to override to model a
                 transmitter with a *framing* fault.
  NecirRx        a decoder: it measures the leader, tells a frame from a repeat
                 code, reads the 32 bits, checks the inverse words and logs a
                 `NecirFrame` per press plus a `NecirRepeat` per repeat code.
  NecirChecker   a protocol checker. It drives nothing, measures the resolved
                 pin values, and appends a `Violation` with its tick for every
                 rule the wire breaks. It never guesses whether a frame is
                 standard or extended: it is told (`extended=`).

Timing conventions, both from core.py's docstring:

* A driver that returns a value for net N at tick t has the net read that value
  from the end of tick t on; every device sees it at tick t+1. The models that
  decode take that out of every tick they report (`latency=1` by default), so
  `NecirFrame.tick` and the violation ticks are the ticks the transmitter really
  drove them. Only the *absolute* ticks shift: every measurement here is a
  difference of two edges, so durations are unaffected by the latency.
* A device that is attached while the line is already high cannot know when the
  mark began. Both decoders treat that first mark as having begun at tick 0, set
  `started_mid_mark`, and say so in that frame's first error. The frame it
  joined comes back garbled, and so does the next one unless there is an idle
  gap long enough to find a frame boundary in -- the same caveat, in the same
  words, as the UART receiver in verif/vip/uart.py. Attach before the frame, or
  leave an idle gap, or accept the garbage.

The rules `NecirChecker` enforces, all as windows from `necir_timings()`:

  leader-mark       a frame starts with a 9 ms mark; a bit-length mark there is
                    a glitch or a truncated frame, not a frame
  leader-space      4.5 ms after the leader, or 2.25 ms for a repeat code;
                    anything else is neither
  bit-mark          every bit starts with a 562.5 us burst
  bit-space         every data space is a '0' space or a '1' space; a space
                    between the two windows cannot be decoded and is flagged
  stop-mark         the frame ends with one bit-length burst. Without it the
                    last data bit is undeterminable, which is why NEC has it
  bit-count         exactly 32 data bits. A 33rd burst before the frame ends is
                    flagged here rather than being read as the next frame
  inverse-address   word 1 == ~word 0 (standard form only)
  inverse-command   word 3 == ~word 2
  extended-address  in the extended form word 1 must *not* be ~word 0: those
                    256 addresses are ordinary NEC addresses (extended only)
  repeat-code       a repeat code is leader + 2250 us space + one bit mark,
                    and carries no data
  repeat-no-data    data bits after a repeat code: NECx, not NEC
  repeat-period     repeat codes and the frames that follow them start 110 ms
                    apart (the sources print 108 and 110; the window holds both)
  orphan-repeat     a repeat code with no frame before it repeats nothing
  frame-length      the standard form has a constant length, because each word
                    is followed by its inverse; the extended form must be at
                    least the all-zero length

Two things this model deliberately does not do, both from the sources:

* It does not detect NEC2 (whole frame repeated instead of a repeat code).
  NEC1 and NEC2 send the same first frame, so a stream that never repeats
  cannot be told apart from a single press -- hifi-remote's NEC page says the
  same thing ("NEC and NEC 2 only differ in the repeat frames, so the protocol
  can only be detected correctly after the first repeat").
* It does not judge the inverse words as optional. NEC transmits them "twice
  ... for reliability" (sbprojects) and "always followed, without a pause, by
  the inverted words" (Vishay 80071); a strict checker has to insist.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Mapping, Optional

from .core import Bus, Device

IDLE_LEVEL = 0                      # no carrier on the pin
MARK = 1                            # carrier present

@dataclass(frozen=True)
class NecirTiming:
    """The NEC nominals, in nanoseconds, exactly as the sources print them.

    Every field carries its citation; see research/ref/protocols/necir/README.md
    for the full rule-to-source table and for what each judgement call costs.
    Nothing here comes from memory, and nothing here is a window: the windows
    are `necir_timings()`'s job, because no source prints a tolerance.
    """

    leader_mark_ns: float = 9_000.0     # Renesas "9-ms leading pulse burst
                                        # (16 times the pulse burst length)";
                                        # Vishay "a burst with a length of 9 ms"
    leader_space_ns: float = 4_500.0    # Renesas/Vishay "4.5-ms space"
    bit_mark_ns: float = 562.5          # Renesas 562.5 us burst; IRremote and
                                        # sbprojects print the unit as 560 us
    bit0_space_ns: float = 562.5        # Renesas: '0' = 562.5 us burst + 562.5 us
    bit1_space_ns: float = 1_687.5      # Renesas: '1' = burst + 1.6875 ms
                                        # (= 3 x 562.5); not the 1562.5 us that
                                        # circulates -- see README judgement 6
    stop_mark_ns: float = 562.5         # Renesas "Final 562.5 us pulse burst"
    repeat_space_ns: float = 2_250.0    # Renesas "2.25-ms space"; IRremote
                                        # NEC_REPEAT_HEADER_SPACE = 4 * 560
    repeat_period_ns: float = 110_000_000.0   # Renesas/sbprojects/IRremote 110 ms
                                        # start to start; wiki and Vishay 108
    frame_bits: int = 32                # 4 words of 8, LSB first
    carrier_hz: float = 38_000.0        # informational only: demodulated signal


NECIR_TIMING = NecirTiming()

TIMING_KEYS = (
    "leader_mark_ns", "leader_space_ns", "bit_mark_ns", "bit0_space_ns",
    "bit1_space_ns", "stop_mark_ns", "repeat_space_ns", "repeat_period_ns",
)

#: (window name, the nominal it comes from). A window holds whole ticks.
_WINDOWS_FROM_NOMINAL = (
    ("leader_mark", "leader_mark_ns"), ("leader_space", "leader_space_ns"),
    ("bit_mark", "bit_mark_ns"), ("bit0_space", "bit0_space_ns"),
    ("bit1_space", "bit1_space_ns"), ("stop_mark", "stop_mark_ns"),
    ("repeat_space", "repeat_space_ns"), ("repeat_period", "repeat_period_ns"),
)


def _check_tolerance(value: float) -> float:
    tol = float(value)
    if not 0.0 <= tol < 1.0:
        raise ValueError(f"tolerance must be in [0, 1) (got {value!r})")
    return tol


def necir_timings(ns_per_tick: float = 20.0, tolerance: float = 0.25,
                  timing: Optional[NecirTiming] = None,
                  overrides: Optional[dict] = None) -> dict:
    """Every NEC limit as a window of whole ticks, plus the nominals behind them.

    A nominal `n` becomes the tick window `[floor(n*(1-tolerance)),
    ceil(n*(1+tolerance))]`, so at the default 25 % a 562.5 us bit burst is
    21..36 ticks at 20 ns/tick. No source prints a min/max column (README
    judgement call 2), so `tolerance` is this model's and it is the one knob
    that decides how strict the checker is; `tolerance=0.05` holds the sources'
    own spread (560 vs 562.5 us) and nothing more.

    `overrides` replaces individual nominals by their field names, which is how
    a test drives one device off-spec without touching the others, and how the
    frames below are shortened (e.g. `{"stop_mark_ns": 0}`: a frame with no
    trailing burst at all). It accepts the `NECIR_TIMING` field names, not the
    window names.

    Besides the eight windows it returns three more, all derived:

      frame_min    the all-zero frame, i.e. the shortest legal frame
      frame_max    the all-one frame, i.e. the longest one the extended form
                   can reach
      frame_exact  the standard form's length, which is constant: every word is
                   followed by its inverse, so the 32 bits always carry exactly
                   16 ones (sbprojects: "The total transmission time is
                   constant because every bit is repeated with its inverted
                   length"). The minimum frame, `frame_min`, happens to have the
                   same length, which is why `frame_min <= frame_exact <=
                   frame_max` always holds.
    """
    if ns_per_tick <= 0:
        raise ValueError(f"ns_per_tick must be positive (got {ns_per_tick!r})")
    tol = _check_tolerance(tolerance)
    base = timing or NECIR_TIMING
    ns = {key: getattr(base, key) for key in TIMING_KEYS}
    overrides = dict(overrides or {})
    unknown = set(overrides) - set(TIMING_KEYS)
    if unknown:
        raise ValueError(f"unknown NEC timing override(s) {sorted(unknown)}; "
                         f"pick from {list(TIMING_KEYS)}")
    ns.update({k: float(v) for k, v in overrides.items()})
    for key, value in ns.items():
        if value < 0:
            raise ValueError(f"{key} must not be negative (got {value!r})")

    def window(value: float) -> tuple[int, int]:
        lo = int(value * (1.0 - tol) // ns_per_tick)
        hi = -int(-value * (1.0 + tol) // ns_per_tick)      # ceil
        return max(1, lo), max(1, hi)

    out: dict = {"ns_per_tick": float(ns_per_tick), "tolerance": tol,
                 "nominal_ns": ns, "frame_bits": int(base.frame_bits)}
    for name, key in _WINDOWS_FROM_NOMINAL:
        out[name] = window(ns[key])
    lead = ns["leader_mark_ns"] + ns["leader_space_ns"] + ns["stop_mark_ns"]
    bit0 = ns["bit_mark_ns"] + ns["bit0_space_ns"]
    bit1 = ns["bit_mark_ns"] + ns["bit1_space_ns"]
    out["frame_min"] = window(lead + out["frame_bits"] * bit0)
    out["frame_max"] = window(lead + out["frame_bits"] * bit1)
    ones = out["frame_bits"] // 2       # 32 bits, each 1 paired with its 0
    out["frame_exact"] = window(lead + out["frame_bits"] * bit0
                                + ones * (ns["bit1_space_ns"] - ns["bit0_space_ns"]))
    out["bit_count"] = (int(base.frame_bits), int(base.frame_bits))
    return out


def _check_byte(value: int, what: str) -> int:
    byte = int(value)
    if not 0 <= byte <= 0xFF:
        raise ValueError(f"{what} must be one byte, got {value!r}")
    return byte


def encode_frame(address: int, command: int, extended: bool = False) -> int:
    """The 32 bits of one NEC frame, as an integer, LSB of it first on the wire.

    Standard form is `address, ~address, command, ~command` (Renesas; Vishay
    80071 "first as a normal byte followed by an inverted byte"; IRremote's
    `computeNECRawDataAndChecksum`). The extended form spends the inverse
    address on a register byte instead, so the first 16 bits are one address
    with no copy (Renesas; sbprojects "Extended NEC protocol").
    """
    command = _check_byte(command, "command")
    if extended:
        address &= 0xFFFF
        if not 0 <= address <= 0xFFFF:
            raise ValueError(f"extended address must be 16 bits, got {address!r}")
        return ((address & 0xFFFF) | ((command & 0xFF) << 16)
                | ((~command & 0xFF) << 24))
    return (_check_byte(address, "address") | ((~address & 0xFF) << 8)
            | (command << 16) | ((~command & 0xFF) << 24))


def decode_frame(raw: int) -> dict:
    """The four words of a frame, and whether the two inverse pairs check out.

    This is the decode `NecirRx` reports and `NecirChecker` compares. It does
    not decide whether a frame is standard or extended: with `extended=False`
    the caller is judging the standard form and `address_inverse_ok` is the
    rule; with `extended=True` it is the other way round and
    `address_is_not_inverse` is the rule (sbprojects: "Whenever the low byte
    is the exact inverse of the high byte it is not a valid extended address").
    """
    raw = int(raw) & 0xFFFFFFFF
    words = [(raw >> (8 * i)) & 0xFF for i in range(4)]
    return {
        "words": words,
        "address8": words[0],
        "address16": (words[1] << 8) | words[0],
        "command": words[2],
        "address_inverse_ok": words[1] == (~words[0] & 0xFF),
        "command_inverse_ok": words[3] == (~words[2] & 0xFF),
    }


class _Demod(Device):
    """Edge bookkeeping shared by `NecirRx` and `NecirChecker`.

    Both are passive and both work the same way: they watch one net, they turn
    it into "a mark began", "a mark ended", "a space ended" events with the
    tick already corrected for `latency`, and they keep the last mark and space
    measured. `active_low` models the other common pin convention, a TSOP-style
    receiver output that is pulled up and pulled *low* by a burst; internally
    the level always means the same thing (1 = mark), so the subclass state
    machines do not care.
    """

    def __init__(self, net: str = "ir", ns_per_tick: float = 20.0,
                 tolerance: float = 0.25, latency: int = 1,
                 active_low: bool = False, name: Optional[str] = None,
                 timing: Optional[NecirTiming] = None,
                 overrides: Optional[dict] = None):
        self.net = net
        self.name = name or f"{type(self).__name__.lower().replace('necir', 'necir_')}[{net}]"
        self.ns_per_tick = float(ns_per_tick)
        self.latency = int(latency)
        if self.latency < 0:
            raise ValueError(f"latency must be >= 0 (got {latency!r})")
        self.active_low = bool(active_low)
        self.timing = timing or NECIR_TIMING
        self.timing_overrides = dict(overrides or {})
        self.win = necir_timings(self.ns_per_tick, tolerance, self.timing,
                                 self.timing_overrides)
        self.tolerance = self.win["tolerance"]
        self.started_mid_mark = False
        self._polarity = 1 if self.active_low else 0
        self._last = IDLE_LEVEL ^ self._polarity
        self._watched = False
        self._mark_start: Optional[int] = None
        self.last_mark: Optional[tuple[int, int]] = None     # (start, ticks)
        self.last_space: Optional[tuple[int, int]] = None
        self.marks = 0
        self.spaces = 0

    # -- helpers ------------------------------------------------------------
    def level(self, pins: Mapping[str, int]) -> int:
        """The pin as a mark flag: 1 while the carrier is on."""
        return int(pins.get(self.net, IDLE_LEVEL)) ^ self._polarity

    def bind(self, bus: Bus) -> "_Demod":
        """Take the tick rate from `bus` and recompute the windows."""
        self.ns_per_tick = bus.ns_per_tick
        self.win = necir_timings(self.ns_per_tick, self.tolerance, self.timing,
                                 self.timing_overrides)
        return self

    def window(self, name: str) -> tuple[int, int]:
        return self.win[name]

    def in_window(self, name: str, ticks: float) -> bool:
        lo, hi = self.win[name]
        return lo <= ticks <= hi

    def classify_space(self, ticks: float) -> Optional[int]:
        """0, 1, or None: which data bit a space of `ticks` is, if either.

        The two data windows do not touch at the default tolerance (a '0' tops
        out at 36 ticks and a '1' starts at 64 at 20 ns/tick), so None is a
        real answer and not a rounding artefact: it means the space is
        undeterminable and both the decoder and the checker must say so.
        """
        lo0, hi0 = self.win["bit0_space"]
        lo1, hi1 = self.win["bit1_space"]
        if lo0 <= ticks <= hi0:
            return 0
        if lo1 <= ticks <= hi1:
            return 1
        return None

    def _observe(self, t: int, pins: Mapping[str, int]):
        """Feed one tick in; yield an event, or None if nothing moved.

        Events: ("mark", start_tick, ticks), ("space", start_tick, ticks) and
        ("mark_start", start_tick, 0). The first event of a mark is `mark_start`
        so a decoder can react to the leader immediately; `mark` carries the
        measured length, which only exists once the line falls again.
        """
        level = self.level(pins)
        previous, self._last = self._last, level
        start = t - self.latency
        if not self._watched:
            # First tick ever. If the line is already high the mark began
            # before we were watching, and there is no honest tick for it.
            self._watched = True
            if level == MARK:
                self._mark_start = 0
                self.started_mid_mark = True
                yield ("mark_start", 0, 0)
        if level == previous:
            return
        if level == MARK:
            if self._mark_start is not None:
                return                      # the line was already high
            if self.last_mark is not None:  # the space before it ends now
                space_start = self.last_mark[0] + self.last_mark[1]
                self.last_space = (space_start, start - space_start)
                self.spaces += 1
                yield ("space", *self.last_space)
            self._mark_start = start
            yield ("mark_start", start, 0)
        elif self._mark_start is not None:
            self.last_mark = (self._mark_start, start - self._mark_start)
            self.marks += 1
            self._mark_start = None
            yield ("mark", *self.last_mark)


@dataclass
class TxFrame:
    """One frame or repeat code as the transmitter drove it."""

    address: Optional[int]              # None for a repeat code
    command: Optional[int]
    extended: bool = False
    raw: int = 0
    repeat: bool = False
    start_tick: int = 0                 # tick the leader mark began
    end_tick: Optional[int] = None      # first idle tick after the last burst
    expected_start: Optional[int] = None   # raster: tick this item may start at

    def __str__(self) -> str:
        what = ("repeat code" if self.repeat
                else f"0x{(self.raw >> 16) & 0xFFFF:04x}/0x{self.command:02x}")
        return f"t={self.start_tick} {what}"


class NecirTx(Device):
    """A NEC remote: queued frames and repeat codes on one demodulated net.

    `queue(address, command)` puts one frame on the wire when the bus next
    steps, `queue_repeat()` one repeat code, and `press(address, command,
    repeats=N)` the pair a held key produces: the frame, then N repeat codes
    spaced `repeat_period_ns` apart, measured start to start, which is what
    IRremote's `NEC_REPEAT_PERIOD` comment describes. `bus=` or `ns_per_tick=`
    binds the tick rate; the segment lengths are fractional ticks and are
    accumulated, not rounded per element, so a 562.5 us burst at 20 ns/tick is
    28 or 29 ticks depending on where it starts and never drifts.

    Fault injection, for the tests and for firmware debug:

    * `timing=` replaces any nominal by its `NECirTiming` field name, so
      `timing={"leader_mark_ns": 4_000}` really does send a 4 ms leader.
      `timing={"stop_mark_ns": 0}` sends a frame with no trailing burst at all.
    * `segments()` is the one method to override to break the *framing* (drop
      the inverse command, send 33 bits, tag the repeat code with a data bit).
      It returns the same list `queue()` builds: `(mark_ns, space_ns)` pairs,
      the last one with a space of 0.
    """

    name = "necir_tx"

    def __init__(self, net: str = "ir", ns_per_tick: float = 20.0,
                 timing: Optional[NecirTiming] = None, overrides: Optional[dict] = None,
                 active_low: bool = False, polarity: int = None,
                 idle_ns: float = 0.0, name: Optional[str] = None):
        self.net = net
        if name:
            self.name = name
        self.ns_per_tick = float(ns_per_tick)
        self.timing = timing or NECIR_TIMING
        self.timing_overrides = dict(overrides or {})
        self.win = necir_timings(self.ns_per_tick, 0.0, self.timing,
                                 self.timing_overrides)
        self.active_low = bool(active_low)
        self.polarity = (1 if self.active_low else 0) if polarity is None else int(polarity)
        self.idle_ns = float(idle_ns)
        self.pending: deque[tuple] = deque()    # (segments, TxFrame)
        self.tx_log: list[TxFrame] = []
        self.errors: list[str] = []
        self._plan: list[tuple[float, int]] = []
        self._pi = 0
        self._phase = 0.0
        self._idle_until = 0.0                  # float tick, next item may start
        self._cur: Optional[TxFrame] = None

    # -- configuration ------------------------------------------------------
    def bind(self, bus: Bus) -> "NecirTx":
        self.ns_per_tick = bus.ns_per_tick
        self.win = necir_timings(self.ns_per_tick, 0.0, self.timing,
                                 self.timing_overrides)
        return self

    def ns(self, key: str) -> float:
        """One nominal, in nanoseconds, with the overrides applied."""
        return self.win["nominal_ns"][key]

    # -- the queue ----------------------------------------------------------
    def frame_segments(self, address: int, command: int,
                       extended: bool = False) -> list[tuple[float, float]]:
        """`[(mark_ns, space_ns), ...]` for one frame. Override to break framing."""
        raw = encode_frame(address, command, extended)
        bits = [(raw >> i) & 1 for i in range(32)]
        out = [(self.ns("leader_mark_ns"), self.ns("leader_space_ns"))]
        for bit in bits:
            space = self.ns("bit1_space_ns") if bit else self.ns("bit0_space_ns")
            out.append((self.ns("bit_mark_ns"), space))
        out.append((self.ns("stop_mark_ns"), 0.0))     # the burst that ends it
        return out

    def repeat_segments(self) -> list[tuple[float, float]]:
        """`[(mark_ns, space_ns)]` for one repeat code. Override to break framing."""
        return [(self.ns("leader_mark_ns"), self.ns("repeat_space_ns")),
                (self.ns("bit_mark_ns"), 0.0)]

    def queue(self, address: int, command: int, extended: bool = False) -> "NecirTx":
        """Queue one frame: leader, 32 data bits LSB first, trailing burst."""
        frame = TxFrame(address=address, command=command, extended=extended,
                        raw=encode_frame(address, command, extended))
        self.pending.append((self.frame_segments(address, command, extended), frame))
        return self

    def queue_raw(self, raw: int, extended: bool = False) -> "NecirTx":
        """Queue one frame from 32 bits of payload, bypassing encode_frame().

        This is the door for a broken frame: bits that are not the inverse of
        their partner go straight on the wire.
        """
        raw = int(raw) & 0xFFFFFFFF
        frame = TxFrame(address=(raw >> 16) & 0xFFFF, command=(raw >> 16) & 0xFF,
                        extended=extended, raw=raw)
        segments = [(self.ns("leader_mark_ns"), self.ns("leader_space_ns"))] + [
            (self.ns("bit_mark_ns"),
             self.ns("bit1_space_ns") if (raw >> i) & 1 else self.ns("bit0_space_ns"))
            for i in range(32)
        ] + [(self.ns("stop_mark_ns"), 0.0)]
        self.pending.append((segments, frame))
        return self

    def queue_repeat(self) -> "NecirTx":
        """Queue one repeat code: leader, 2250 us space, one bit burst."""
        frame = TxFrame(address=None, command=None, repeat=True)
        self.pending.append((self.repeat_segments(), frame))
        return self

    def press(self, address: int, command: int, repeats: int = 0,
              extended: bool = False, period_ns: Optional[float] = None) -> "NecirTx":
        """Queue a frame followed by `repeats` repeat codes, raster-timed.

        The repeat raster is this model's business, not the firmware's, so
        `press()` inserts the idle time itself: repeat `n` starts exactly
        `period_ns` after the frame started, which by default is the nominal
        110 ms. Pass another `period_ns` to model a remote that holds the key
        and repeats wrongly (too fast trips the checker's `repeat-period`).
        """
        if repeats < 0:
            raise ValueError(f"repeats must be >= 0 (got {repeats!r})")
        period = self.ns("repeat_period_ns") if period_ns is None else float(period_ns)
        self.queue(address, command, extended)
        for n in range(1, repeats + 1):
            self.queue_repeat()
            self.pending[-1][1].expected_start = n * period / self.ns_per_tick
        return self

    # -- the device ---------------------------------------------------------
    @property
    def busy(self) -> bool:
        return bool(self._plan) or bool(self.pending)

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def sent(self) -> int:
        """Frames and repeat codes finished so far."""
        return sum(1 for f in self.tx_log if f.end_tick is not None)

    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        while self._plan and self._phase >= self._plan[self._pi][0] - 1e-9:
            self._phase -= self._plan[self._pi][0]
            self._pi += 1
            if self._pi >= len(self._plan):
                self._end(t)
        if not self._plan and self.pending and t >= self._idle_until:
            self._begin(t)
        if not self._plan:
            return {self.net: IDLE_LEVEL ^ self.polarity}
        self._phase += 1.0
        return {self.net: self._plan[self._pi][1] ^ self.polarity}

    def _begin(self, t: int) -> None:
        segments, frame = self.pending.popleft()
        expected = frame.expected_start
        if expected is not None and t < expected:
            # honour the raster: idle out the rest of the period
            self._idle_until = expected
            self.pending.appendleft((segments, frame))
            return
        frame.start_tick = t
        frame.expected_start = None
        self.tx_log.append(frame)
        self._cur = frame
        self._plan = []
        for mark_ns, space_ns in segments:
            if mark_ns > 0:
                self._plan.append((mark_ns / self.ns_per_tick, MARK))
            if space_ns > 0:
                self._plan.append((space_ns / self.ns_per_tick, IDLE_LEVEL))
        self._pi = 0
        self._phase = 0.0

    def _end(self, t: int) -> None:
        if self._cur is not None:
            self._cur.end_tick = t
        self._plan = []
        self._pi = 0
        self._phase = 0.0
        self._cur = None
        self._idle_until = t + self.idle_ns / self.ns_per_tick


@dataclass
class NecirFrame:
    """One decoded frame, as the receiver saw it."""

    address: int                    # 8-bit, or the low byte of an extended address
    command: int
    extended: bool                  # True if it decoded as the extended form
    raw: int = 0
    tick: int = 0                   # tick the leader mark began (latency removed)
    stop_tick: Optional[int] = None # tick the trailing burst ended
    leader_mark_ticks: int = 0      # as measured
    leader_space_ticks: int = 0
    address_inverse_ok: bool = False
    command_inverse_ok: bool = False
    errors: list[str] = field(default_factory=list)
    mark_ticks: list[int] = field(default_factory=list)   # leader, 32 bits, stop
    space_ticks: list[int] = field(default_factory=list)  # the 32 data spaces
    bits: list[int] = field(default_factory=list)         # the 32 data bits, LSB first

    @property
    def ok(self) -> bool:
        """True if nothing about the frame was wrong."""
        return not self.errors

    @property
    def length_ticks(self) -> Optional[int]:
        """Leader start to end of the trailing burst, in ticks."""
        if self.stop_tick is None:
            return None
        return self.stop_tick - self.tick

    def __str__(self) -> str:
        return (f"t={self.tick} 0x{self.address:02x}/0x{self.command:02x}"
                + (" extended" if self.extended else "")
                + ("" if self.ok else " errors=" + ",".join(self.errors)))


@dataclass
class NecirRepeat:
    """One decoded repeat code, which repeats the frame before it."""

    tick: int                   # tick the leader mark began (latency removed)
    address: Optional[int]      # taken from the frame it repeats, None if orphan
    command: Optional[int] = None
    since_frame: Optional[int] = None      # ticks since that frame started
    mark_ticks: tuple[int, ...] = ()
    space_ticks: tuple[int, ...] = ()

    def __str__(self) -> str:
        what = "orphan" if self.address is None else f"0x{self.address:02x}/0x{self.command:02x}"
        return f"t={self.tick} repeat {what}"


@dataclass
class NecirPress:
    """One key press: the frame, and every repeat code that followed it."""

    frame: NecirFrame
    repeats: list[NecirRepeat] = field(default_factory=list)

    @property
    def address(self) -> int:
        return self.frame.address

    @property
    def command(self) -> int:
        return self.frame.command

    @property
    def held_ticks(self) -> int:
        """From the frame's leader to the last repeat's start."""
        if not self.repeats:
            return self.frame.length_ticks or 0
        return self.repeats[-1].tick - self.frame.tick


class NecirRx(_Demod):
    """Decodes NEC frames and repeat codes off one demodulated net. Passive.

    It measures, it never drives, and it reports rather than throws: a frame
    with a bad inverse word is decoded and flagged in `errors`, because that is
    what a real receiver does -- the data is there, it is just unverified. The
    rules themselves live in `NecirChecker`, which is the strict one; this is the
    model firmware is written against, so it is deliberately forgiving about
    *anything* that still decodes to the same 32 bits and explicit about what
    does not.

    A space that falls in neither data window (`ambiguous space`) is decoded
    with the threshold between the two nominals and flagged, because a decoder
    that guessed silently would hide the very fault `NecirChecker` reports.
    """

    name = "necir_rx"

    def __init__(self, net: str = "ir", ns_per_tick: float = 20.0,
                 tolerance: float = 0.25, latency: int = 1,
                 active_low: bool = False, extended: bool = False,
                 timing: Optional[NecirTiming] = None, overrides: Optional[dict] = None,
                 name: Optional[str] = None):
        if name:
            self.name = name
        super().__init__(net=net, ns_per_tick=ns_per_tick, tolerance=tolerance,
                         latency=latency, active_low=active_low, name=name,
                         timing=timing, overrides=overrides)
        self.extended = bool(extended)
        self.frames: list[NecirFrame] = []
        self.repeat_codes: list[NecirRepeat] = []
        self.presses: list[NecirPress] = []
        self.errors: list[str] = []              # decoder-level failures
        self.orphan_repeats = 0
        self.ambiguous_spaces = 0
        self.bad_marks = 0
        self._consumed = 0
        self._reset_frame()

    # -- configuration ------------------------------------------------------
    @property
    def extended(self) -> bool:
        """Whether an undecodable-as-standard frame is reported as extended."""
        return self._extended

    @extended.setter
    def extended(self, value: bool) -> None:
        self._extended = bool(value)

    # -- the log ------------------------------------------------------------
    @property
    def pending(self) -> int:
        """Key presses decoded but not yet popped."""
        return len(self.presses) - self._consumed

    @property
    def repeats(self) -> int:
        """Repeat codes decoded so far."""
        return len(self.repeat_codes)

    @property
    def address(self) -> Optional[int]:
        return self.presses[-1].address if self.presses else None

    @property
    def command(self) -> Optional[int]:
        return self.presses[-1].command if self.presses else None

    def pop(self) -> Optional[NecirPress]:
        """Oldest press not yet popped, or None."""
        if self._consumed >= len(self.presses):
            return None
        press = self.presses[self._consumed]
        self._consumed += 1
        return press

    # -- the device ---------------------------------------------------------
    #
    # Phases, one per burst or space that still means something:
    #   idle            nothing in progress
    #   mark            a burst is on the line; _is_leader says what it is
    #   leader          the leader burst is over, its space is running
    #   data            inside a frame, reading bits
    #   stop            32 bits are in, the trailing burst is running
    #   stopped         the trailing burst is over: the frame is complete
    #   repeat_mark     the leader space was a repeat code's, its burst is due
    #   repeat_stopped  the repeat code's burst is over, its space is running
    # `_expect` says what the next space means; a space arriving in `idle` is
    # idle time and carries nothing.
    def _reset_frame(self) -> None:
        """Everything about the frame being decoded. Called after every frame."""
        self._phase = "idle"
        self._expect = "none"
        self._is_leader = False
        self._is_repeat_burst = False
        self._is_stop_mark = False
        self._burst_start = 0
        self._marks: list[int] = []
        self._spaces: list[int] = []
        self._bits: list[int] = []
        self._errors: list[str] = []
        self._just_closed = False
        self._repeat_mark = 0
        self._leader_tick = 0
        self._leader_mark = 0
        self._leader_space = 0
        self._stop_end = None

    def _note(self, message: str) -> None:
        self._errors.append(message)

    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        for event in self._observe(t, pins):
            if event[0] == "mark_start":
                self._on_mark_start(event[1])
            elif event[0] == "mark":
                self._on_mark_end(event[1], event[2])
            elif event[0] == "space":
                self._on_space_end(event[1], event[2])
        return {}

    # -- a burst began ------------------------------------------------------
    def _on_mark_start(self, tick: int) -> None:
        if self._phase == "stopped":
            self._finish_frame()          # the previous frame is complete
        self._just_closed = False
        self._is_leader = self._phase == "idle"
        self._is_repeat_burst = self._phase == "repeat_mark"
        self._is_stop_mark = self._phase == "stop"
        self._burst_start = tick
        self._phase = "mark"

    # -- a burst ended ------------------------------------------------------
    def _on_mark_end(self, start: int, ticks: int) -> None:
        self._marks.append(ticks)
        if self._is_stop_mark:            # the trailing burst that ends a frame
            self._stop_end = start + ticks
            self._phase, self._expect = "stopped", "none"
            return
        if self._is_leader:
            self._leader_tick, self._leader_mark = start, ticks
            if self.started_mid_mark and len(self._marks) == 1:
                self._note("the first burst was already on the line when the "
                           "receiver attached: it could not be dated, so its "
                           "length is a guess")
            if not self.in_window("leader_mark", ticks):
                self.bad_marks += 1
                extra = (": that frame carried more than "
                         f"{self.win['frame_bits']} data bits" if self._just_closed
                         else "")
                self._note(f"first burst {ticks} ticks is not a "
                           f"{self.win['nominal_ns']['leader_mark_ns']:.0f} ns "
                           f"leader (window {self.window('leader_mark')}){extra}")
            self._phase, self._expect = "leader", "leader"
            return
        if self._is_repeat_burst:
            self._repeat_mark = ticks
            if not self.in_window("bit_mark", ticks):
                self.bad_marks += 1
                self._note(f"repeat code burst {ticks} ticks, window "
                           f"{self.window('bit_mark')}")
            self._phase, self._expect = "repeat_stopped", "repeat_mark"
            return
        if not self.in_window("bit_mark", ticks):
            self.bad_marks += 1
            self._note(f"burst {ticks} ticks before data bit {len(self._bits)} is "
                       f"not a {self.win['nominal_ns']['bit_mark_ns']:.0f} ns burst "
                       f"(window {self.window('bit_mark')})")
        self._phase, self._expect = "data", "data"

    # -- a space ended ------------------------------------------------------
    def _on_space_end(self, start: int, ticks: int) -> None:
        phase = self._phase
        if phase == "stopped":            # the frame is complete and closed
            if self.classify_space(ticks) is not None:
                self._note("a data bit followed the frame's trailing burst: "
                           "this frame carried more than "
                           f"{self.win['frame_bits']} data bits")
            self._finish_frame()
            return
        if phase == "leader":
            self._leader_space = ticks
            if self.in_window("repeat_space", ticks):
                self._phase = "repeat_mark"       # leader + 2250 us + a burst
                return
            if not self.in_window("leader_space", ticks):
                self._note(f"space after the leader {ticks} ticks is neither a "
                           f"frame's {self.window('leader_space')} nor a repeat "
                           f"code's {self.window('repeat_space')}")
            self._phase, self._expect = "data", "data"
            return
        if phase == "repeat_stopped":
            self._finish_repeat()
            return
        if phase == "data":
            self._space_end_data(ticks)
            return
        # "idle" or "mark": idle time, carries nothing

    def _space_end_data(self, ticks: int) -> None:
        self._spaces.append(ticks)
        bit = self.classify_space(ticks)
        if bit is None:
            self.ambiguous_spaces += 1
            self._note(f"space {ticks} ticks after data bit {len(self._bits)} is "
                       f"neither a '0' ({self.window('bit0_space')}) nor a '1' "
                       f"({self.window('bit1_space')})")
            threshold = self.win["nominal_ns"]["bit1_space_ns"] / self.ns_per_tick
            bit = 0 if ticks < threshold else 1
        self._bits.append(bit)
        if len(self._bits) >= self.win["frame_bits"]:
            # "stop" is only ever left through a space: a burst arriving here is
            # taken for the trailing burst, because a burst is a burst -- which
            # is exactly why a frame that is one bit short can only be caught
            # on the space where that bit's space should have been.
            self._phase, self._expect = "stop", "stop"      # the trailing burst
        else:
            self._phase, self._expect = "data", "data"

    # -- a repeat code is complete ------------------------------------------
    def _finish_repeat(self) -> None:
        press = self.presses[-1] if self.presses else None
        if press is None:
            self.orphan_repeats += 1
            self.errors.append(f"t={self._leader_tick} repeat code with no frame "
                               "before it")
        repeat = NecirRepeat(
            tick=self._leader_tick,
            address=None if press is None else press.address,
            command=None if press is None else press.command,
            since_frame=None if press is None
            else self._leader_tick - press.frame.tick,
            mark_ticks=(self._leader_mark, self._repeat_mark),
            space_ticks=(self._leader_space,))
        self.repeat_codes.append(repeat)
        if press is not None:
            press.repeats.append(repeat)
        self._reset_frame()

    # -- closing the wire ---------------------------------------------------
    def flush(self) -> None:
        """Log whatever is still half-decoded. Call this after `bus.run(...)`.

        Both decoders here are edge driven, and a frame is logged when the
        *next* burst begins, because that is the first tick at which its
        trailing burst is known to be over. So the last frame or repeat code
        on the wire is still open until either another burst arrives or you
        call this. A frame still open when you do is a truncated one: it is
        logged with whatever bits arrived and an `errors` entry saying so.
        """
        if self._phase in ("stopped", "data", "mark", "leader"):
            self._finish_frame()
        elif self._phase == "repeat_stopped":
            self._finish_repeat()

    # -- a frame is complete ------------------------------------------------
    def _finish_frame(self) -> None:
        raw = 0
        for i, bit in enumerate(self._bits[:self.win["frame_bits"]]):
            raw |= bit << i
        info = decode_frame(raw)
        nbits = len(self._bits)
        frame = NecirFrame(
            address=info["address16"] if self.extended else info["address8"],
            command=info["command"], extended=self.extended, raw=raw,
            tick=self._leader_tick, stop_tick=self._stop_end,
            leader_mark_ticks=self._leader_mark,
            leader_space_ticks=self._leader_space,
            address_inverse_ok=info["address_inverse_ok"],
            command_inverse_ok=info["command_inverse_ok"],
            errors=list(self._errors), mark_ticks=list(self._marks),
            space_ticks=list(self._spaces), bits=list(self._bits))
        if nbits != self.win["frame_bits"]:
            frame.errors.append(f"{nbits} data bits, not {self.win['frame_bits']}")
        self.frames.append(frame)
        self.presses.append(NecirPress(frame=frame))
        self._reset_frame()


@dataclass(frozen=True)
class Violation:
    """One spec rule the wire broke, with the tick it was broken on."""

    tick: int
    rule: str                # e.g. "bit-space", "inverse-command", "repeat-period"
    measured: int            # ticks the wire gave it
    minimum: int             # ticks the rule's low bound is
    maximum: int             # ticks the rule's high bound is
    detail: str = ""

    @classmethod
    def flag(cls, tick: int, rule: str, detail: str = "") -> "Violation":
        """A rule that has no window at all, e.g. a word that is not its inverse."""
        return cls(tick, rule, 0, 0, 0, detail)

    def __str__(self) -> str:
        if self.maximum == self.minimum == 0:
            return f"t={self.tick} {self.rule} {self.detail}"
        return (f"t={self.tick} {self.rule} {self.measured} ticks outside "
                f"{self.minimum}..{self.maximum}"
                + (f" ({self.detail})" if self.detail else ""))


class NecirChecker(_Demod):
    """A passive NEC protocol checker: decodes the wire and judges every rule.

    It drives nothing, so it sees the bus exactly as an analyser would,
    including the two-phase tick's one-tick sampling latency, and it does not
    add or subtract anything for a device reacting late: it measures you, not
    the devices. Every rule that fires is appended to `violations` with its
    tick, and `report()` prints them.

    The rules are the ones listed in the module docstring. Two of them need to
    know what form of NEC it is looking at and both are configuration, not
    detection, because a strict checker must not guess:

    * `extended=False` (default): the standard form, so `inverse-address` and
      `frame-length` are checked. This is what a receiver with 256 addresses
      sees, and it is the default because that is what "NEC" means.
    * `extended=True`: the first 16 bits are one address and the second word is
      a register, so `inverse-address` is not checked, `frame-length` becomes a
      minimum only, and `extended-address` is checked instead: word 1 must not
      be the inverse of word 0, because those 256 values are ordinary NEC
      addresses and never valid extended ones (sbprojects).

    Legal traffic produces `violations == []`. That is a claim the tests check
    both ways: the self-tests run every rule's failure as well as its pass, and
    `test_checker_is_not_vacuous` runs a legal frame through a checker set to
    windows it does not meet.
    """

    name = "necir_check"

    def __init__(self, net: str = "ir", ns_per_tick: float = 20.0,
                 tolerance: float = 0.25, extended: bool = False,
                 active_low: bool = False, timing: Optional[NecirTiming] = None,
                 overrides: Optional[dict] = None, name: Optional[str] = None):
        if name:
            self.name = name
        super().__init__(net=net, ns_per_tick=ns_per_tick, tolerance=tolerance,
                         latency=1, active_low=active_low, name=name,
                         timing=timing, overrides=overrides)
        self.extended = bool(extended)
        self.violations: list[Violation] = []
        self.frames: list[dict] = []      # every frame it saw, rules aside
        self.repeat_codes: list[int] = []  # the tick each repeat code began at
        self.starts: list[int] = []        # the tick every frame/repeat began at
        self._reset_check()

    def bind(self, bus: Bus) -> "NecirChecker":
        super().bind(bus)
        return self

    # -- what the tests ask for ---------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def rules_broken(self) -> set[str]:
        return {v.rule for v in self.violations}

    def report(self, limit: int = 12) -> str:
        head = (f"NEC at {self.ns_per_tick} ns/tick, tolerance "
                f"{self.tolerance:.0%}, {'extended' if self.extended else 'standard'}"
                f" form: {len(self.frames)} frame(s), {len(self.repeat_codes)} "
                f"repeat code(s), {len(self.violations)} violation(s)")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, tick: int, rule: str, measured: float, window: str,
                   detail: str = "") -> None:
        """Flag `measured` if it is outside the window named `window`."""
        self._range(tick, rule, measured, window, window, detail)

    def _range(self, tick: int, rule: str, measured: float, lo_window: str,
               hi_window: str, detail: str = "") -> None:
        """Flag `measured` unless it is in [lo_window.low, hi_window.high]."""
        lo, hi = self.win[lo_window][0], self.win[hi_window][1]
        measured = int(round(measured))
        if not lo <= measured <= hi:
            self.violations.append(Violation(tick, rule, measured, lo, hi, detail))

    def _flag(self, tick: int, rule: str, detail: str) -> None:
        self.violations.append(Violation.flag(tick, rule, detail))

    def _reset_check(self) -> None:
        self._phase = "idle"
        self._expect = "none"
        self._is_leader = False
        self._is_repeat_burst = False
        self._is_stop_mark = False
        self._burst_start = 0
        self._marks: list[int] = []
        self._spaces: list[int] = []
        self._bits: list[int] = []
        self._leader_tick = 0
        self._closed_tick = None
        self._leader_mark = 0
        self._leader_space = 0
        self._stop_end = None
        self._repeat_mark = 0

    # -- the device ---------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        for event in self._observe(t, pins):
            if event[0] == "mark_start":
                self._on_mark_start(event[1])
            elif event[0] == "mark":
                self._on_mark_end(event[1], event[2])
            elif event[0] == "space":
                self._on_space_end(event[1], event[2])
        return {}

    def flush(self) -> None:
        """Judge whatever is still half-open. Call after `bus.run(...)`.

        A frame is judged when the next burst begins, which is the first tick
        at which its trailing burst is known to be over, so the last one on the
        wire is open until you call this. If it is open *and* it has fewer bits
        than a frame must, that is a `bit-count` violation: the wire stopped
        in the middle of a frame.
        """
        if self._phase in ("data", "mark", "leader"):
            # report it where the open frame is: on its leader if it has one,
            # otherwise on the burst that was on the wire when the bus stopped
            where = self._burst_start if self._phase == "leader" else self._leader_tick
            self._violation(where, "bit-count",
                            len(self._bits), "bit_count",
                            "the wire stopped with only %d of %d data bits"
                            % (len(self._bits), self.win["frame_bits"]))
        if self._phase != "idle":       # a quiet wire has nothing to judge
            self._finish_check()

    # -- a burst began ------------------------------------------------------
    def _on_mark_start(self, tick: int) -> None:
        if self._phase in ("stopped", "repeat_stopped"):
            self._finish_check()        # the previous item is complete
        self._is_leader = self._phase == "idle"
        self._is_repeat_burst = self._phase == "repeat_mark"
        self._is_stop_mark = self._phase == "stop"
        self._burst_start = tick
        self._phase = "mark"

    # -- a burst ended ------------------------------------------------------
    def _on_mark_end(self, start: int, ticks: int) -> None:
        self._marks.append(ticks)
        if self._is_stop_mark:                     # the frame's trailing burst
            self._stop_end = start + ticks
            if not self.in_window("stop_mark", ticks):
                self._violation(start, "stop-mark", ticks, "stop_mark",
                                "the burst that ends a frame is one bit long")
            self._phase, self._expect = "stopped", "none"
            return
        if self._is_leader:
            self._leader_tick, self._leader_mark = start, ticks
            self.starts.append(start)
            gap = None if self._closed_tick is None else start - self._closed_tick
            if not self.in_window("leader_mark", ticks):
                if gap is not None and gap < self.win["leader_mark"][0]:
                    self._violation(start, "bit-count", ticks, "bit_count",
                                    "a burst that is not a leader follows the "
                                    "frame's trailing burst %d ticks later: this "
                                    "frame carried more than %d data bits"
                                    % (gap, self.win["frame_bits"]))
                else:
                    self._violation(start, "leader-mark", ticks, "leader_mark",
                                    "a frame has to start with a "
                                    f"{self.win['nominal_ns']['leader_mark_ns']:.0f} ns burst")
            self._phase, self._expect = "leader", "leader"
            return
        if self._is_repeat_burst:
            self._repeat_mark = ticks
            if not self.in_window("bit_mark", ticks):
                self._violation(start, "repeat-code", ticks, "bit_mark",
                                "the burst that ends a repeat code is one bit long")
            self._phase, self._expect = "repeat_stopped", "repeat_mark"
            return
        if not self.in_window("bit_mark", ticks):
            self._violation(start, "bit-mark", ticks, "bit_mark",
                            "burst before data bit %d" % len(self._bits))
        self._phase, self._expect = "data", "data"

    # -- a space ended ------------------------------------------------------
    def _on_space_end(self, start: int, ticks: int) -> None:
        phase = self._phase
        if phase == "stopped":
            # Whatever follows the frame's trailing burst must be idle time.
            # A data space here means the frame carried more than 32 data bits,
            # and a repeat or leader space means the raster is broken.
            bit = self.classify_space(ticks)
            if bit is not None:
                self._violation(self._leader_tick, "bit-count", ticks, "bit_count",
                                "a data bit followed the frame's trailing burst")
            elif self.in_window("repeat_space", ticks):
                self._violation(self._leader_tick, "repeat-period", ticks,
                                "repeat_space",
                                "a repeat code straight after the frame")
            elif self.in_window("leader_space", ticks):
                self._violation(self._leader_tick, "repeat-period", ticks,
                                "leader_space",
                                "a frame straight after the frame, the raster "
                                "allows %d..%d ticks" % self.win["repeat_period"])
            self._finish_check()
            return
        if phase == "repeat_stopped":
            if self.classify_space(ticks) is not None:
                self._flag(self._leader_tick, "repeat-no-data",
                           "a %d-tick data space follows the repeat code's burst: "
                           "a repeat code carries no data bits" % ticks)
            self._finish_check()
            return
        if phase == "leader":
            self._leader_space = ticks
            if self.in_window("repeat_space", ticks):
                # A repeat code only means anything at the raster's cadence, so
                # this is where the raster is checked. A *frame* is a new key
                # press and may start any time after the previous one: NEC2
                # repeats the whole frame every 110 ms, and NEC1 sends a frame
                # every time the user presses again.
                if self._closed_tick is not None:
                    gap = self._leader_tick - self._closed_tick
                    lo, hi = self.win["repeat_period"]
                    if not lo <= gap <= hi:
                        self._violation(self._leader_tick, "repeat-period", gap,
                                        "repeat_period",
                                        "a repeat code %d ticks after the previous "
                                        "item; the raster allows %d..%d" % (gap, lo, hi))
                self._phase = "repeat_mark"        # leader + 2250 us + a burst
                return
            if not self.in_window("leader_space", ticks):
                self._violation(start, "leader-space", ticks, "leader_space",
                                "neither a frame's %d..%d nor a repeat code's %d..%d"
                                % (self.win["leader_space"]
                                   + self.win["repeat_space"]))
            self._phase, self._expect = "data", "data"
            return
        if phase == "data":
            self._space_end_data(ticks)
            return
        # "idle" or "mark": idle time, carries nothing

    def _space_end_data(self, ticks: int) -> None:
        self._spaces.append(ticks)
        bit = self.classify_space(ticks)
        if bit is None:
            self._violation(self._leader_tick, "bit-space", ticks, "bit1_space",
                            "space after data bit %d is neither a '0' (%d..%d) "
                            "nor a '1' (%d..%d)"
                            % ((len(self._bits),) + self.win["bit0_space"]
                               + self.win["bit1_space"]))
            if len(self._bits) + 1 >= self.win["frame_bits"]:
                self._violation(self._leader_tick, "bit-count", ticks, "bit_count",
                                "the %dth data bit's slot holds no data space, "
                                "so this frame carries fewer than %d bits"
                                % (self.win["frame_bits"], self.win["frame_bits"]))
            threshold = self.win["nominal_ns"]["bit1_space_ns"] / self.ns_per_tick
            bit = 0 if ticks < threshold else 1
        self._bits.append(bit)
        if len(self._bits) >= self.win["frame_bits"]:
            # "stop" is only ever left through a space: a burst arriving here is
            # taken for the trailing burst, because a burst is a burst -- which
            # is exactly why a frame that is one bit short can only be caught
            # on the space where that bit's space should have been.
            self._phase, self._expect = "stop", "stop"       # the trailing burst
        else:
            self._phase, self._expect = "data", "data"

    # -- an item is complete ------------------------------------------------
    def _finish_check(self) -> None:
        if self._phase == "repeat_stopped":
            leader_tick = self._leader_tick
            if not self.frames:
                self._flag(leader_tick, "orphan-repeat",
                           "a repeat code with no frame before it repeats nothing")
            self.repeat_codes.append(leader_tick)
            self._reset_check()
            self._closed_tick = leader_tick        # _reset_check() cleared it
            return
        info = decode_frame(sum(bit << i for i, bit
                                in enumerate(self._bits[:self.win["frame_bits"]])))
        if self.extended:
            if info["address_inverse_ok"]:
                self._flag(self._leader_tick, "extended-address",
                           "word 1 is the inverse of word 0, which is an "
                           "ordinary 8-bit NEC address, never an extended one")
        else:
            if not info["address_inverse_ok"] and self._bits:
                self._flag(self._leader_tick, "inverse-address",
                           "word 1 %#04x is not the inverse of word 0 %#04x"
                           % (info["words"][1], info["words"][0]))
        if not info["command_inverse_ok"] and self._bits:
            self._flag(self._leader_tick, "inverse-command",
                       "word 3 %#04x is not the inverse of word 2 %#04x"
                       % (info["words"][3], info["words"][2]))
        if self._stop_end is not None:        # only a closed frame has a length
            length = self._stop_end - self._leader_tick
            if self.extended:
                self._range(self._leader_tick, "frame-length", length,
                            "frame_min", "frame_max",
                            "%d ticks, a frame is %d..%d"
                            % ((length,) + self.win["frame_max"]))
            else:
                self._violation(self._leader_tick, "frame-length", length,
                                "frame_exact",
                                "the standard form is a constant length, this "
                                "frame is %d ticks" % length)
        leader_tick = self._leader_tick
        complete = self._stop_end is not None
        self.frames.append({"tick": leader_tick, "raw": info["words"],
                            "length_ticks": (None if not complete
                                             else self._stop_end - leader_tick),
                            "bits": len(self._bits), "complete": complete})
        self._reset_check()
        self._closed_tick = leader_tick            # _reset_check() cleared it



# ---- harness helpers ------------------------------------------------------
def _ensure_net(bus: Bus, net: str = "ir") -> None:
    """Add the one demodulated IR net if it is not there yet.

    The pull-up is 0, not the core's default 1: on a demodulated IR pin the
    idle level is *no carrier*, so an unwired net has to read 0 or every model
    on it sees a permanent burst. The net is push-pull: a demodulated receiver
    output drives both levels, and there is one remote per room anyway.
    """
    if net not in bus.nets:
        bus.add_net(net, push_pull=True, pullup=IDLE_LEVEL)
    elif bus.nets[net].pullup != IDLE_LEVEL:
        raise ValueError(
            f"net {net!r} has pullup {bus.nets[net].pullup}, but a demodulated IR "
            f"pin idles at {IDLE_LEVEL}: add it with "
            f"bus.add_net({net!r}, push_pull=True, pullup={IDLE_LEVEL})")


def attach_tx(bus: Bus, net: str = "ir", **kwargs) -> NecirTx:
    """Build a `NecirTx` that takes its tick rate from `bus` and attach it."""
    _ensure_net(bus, net)
    tx = NecirTx(net=net, ns_per_tick=bus.ns_per_tick, **kwargs)
    bus.attach(tx)
    return tx


def attach_rx(bus: Bus, net: str = "ir", **kwargs) -> NecirRx:
    """Build a `NecirRx` that takes its tick rate from `bus` and attach it."""
    _ensure_net(bus, net)
    rx = NecirRx(net=net, ns_per_tick=bus.ns_per_tick, **kwargs)
    bus.attach(rx)
    return rx


def attach_checker(bus: Bus, net: str = "ir", **kwargs) -> NecirChecker:
    """Build a passive `NecirChecker` for `bus` and attach it."""
    _ensure_net(bus, net)
    chk = NecirChecker(net=net, ns_per_tick=bus.ns_per_tick, **kwargs)
    bus.attach(chk)
    return chk

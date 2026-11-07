"""Pin-level Manchester reference models on the tick bus in verif/vip/core.py.

Manchester is a biphase line code: every data bit becomes two level periods ("chips",
AN3435) of equal length, the *first* chip and the *second* chip, and the direction of the
transition between them carries the data. The transition at the middle of every bit is
mandatory -- that is what makes the signal self-clocking -- and a transition at a bit
boundary is inserted only when the two adjacent bits are equal, because the line has to be
in the right level to make the next mid-bit transition possible (AN2358: "additional
transitions are inserted between bits to satisfy the mid-bit transition rule"). Every
number here comes from the documents saved in `research/ref/protocols/manchester/`, whose
README.md maps each one to the file and the sentence it came from.

  ManchesterTx             transmitter: queued bits at a configurable bit rate, with a
                           sync preamble, an inter-message gap, and a ManchesterFaults
                           knob for driving the line wrong on purpose.
  ManchesterRx             receiver: recovers bits from the transitions alone, re-locking
                           whenever the timing stops making sense, and says when it did.
  ManchesterTimingChecker  passive monitor: decodes the traffic and flags every timing and
                           framing violation with the tick it happened on, without driving
                           anything.

Three things about this code that are not obvious:

* **The convention is a trap.** Two opposite conventions exist and they are inverses of
  each other: G.E. Thomas encodes 0 as low-to-high, IEEE 802.3 encodes 0 as high-to-low
  (AN1470a, Figure 1 versus Figure 2). Freescale AN3015 uses the Thomas one and never says
  so, so firmware written from it alone decodes 802.3 traffic bit-for-bit inverted. The
  default here is `ieee802.3`; `thomas` is the other one; nothing else is accepted.
* **Ticks quantise the half bit, and the windows have to allow for it.** One tick is 20 ns
  by default, so the profile's 10 Mbit/s (AN1120a Table 2: bit time 100 ns) is exactly 5
  ticks and a half bit is 2.5 ticks: a transmitter cannot make that, it makes 2, 3, 2, 3...
  Every window in `profile_timings` therefore has a tick-quantisation term as well as the
  spec term, and the constructor refuses a bit rate whose half-bit and bit windows would
  overlap, because then no transition could be classified at all.
* **A receiver cannot see framing.** There is no start bit in the line code, so both the
  receiver and the monitor anchor on the first transition of a stream and take it to be a
  mid-bit transition. AN2358 spells out when that is safe (the first bit must agree with the
  idle level, or a preamble must precede it) and the monitor flags the case where it is not
  (`tSYNC`).

Typical use, 10 Mbit/s on the default 50 MHz bus:

    bus = Bus(ns_per_tick=20)                            # 50 MHz
    tx = attach_tx(bus, bit_rate=10e6)                   # 5 ticks per bit
    rx = attach_rx(bus, bit_rate=10e6, preamble_bits=2)  # 2 = the tx's sync pattern
    chk = attach_checker(bus, bit_rate=10e6)             # passive, drives nothing
    tx.send(b"hello")
    assert tx.run(bus, limit=20_000)
    assert rx.messages[0].as_bytes() == b"hello"
    assert chk.clean, chk.report()

Notes for an MVP PIO implementation (`docs/mvp-isa.md`, 50 MHz chip clock):

* One tick is 20 ns, and IEEE 802.3's 10 Mbit/s is exactly 5 ticks per bit (AN1120a's
  100 ns), so a bit is five SM cycles and the half bit alternates 2 and 3. That is the
  natural rate for this engine: the mid-bit transition lands on a clock edge every bit, which
  is what the `tHALF` and `tMID-BIT` rules are about.
* Anything faster needs a bit period under 5 ticks, and `profile_timings` refuses rates
  whose half-bit and bit windows would overlap (below 12.5 Mbit/s at 20 ns/tick), because
  past that no transition can be told from another. Anything slower is fine up to the point
  the simulation gets slow: 25 kbit/s is 2000 ticks per bit.
* The line rests at the idle level (high) between messages and the monitor needs
  `min_gap_bits` (four bit times by default) of idle line to call a boundary, so an
  implementation should hold the line idle for at least that long between messages. That is
  what `ManchesterTx`'s `gap_bits` default does.
* To look at a waveform rather than a report, build the bus with
  `manchester_bus(trace=True)` and call `bus.write_vcd("manchester.vcd")`.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import ceil
from typing import Iterable, Mapping, Optional, Sequence

from .core import Bus, Device

#: The two conventions that exist, and the only two spellings accepted. See the docstring.
CONVENTIONS = ("ieee802.3", "thomas")

#: The level the line rests at between messages. IEEE 802.3 idles high (its "Link Active"
#: link pulses are high, AN1120a Fig. 12); links that idle low exist (AN2358 Figure 13), so
#: this is a parameter of every device rather than a constant.
IDLE_LEVEL = 1

DEFAULT_PROFILE = "ieee802.3-10base-t"

_EPS = 1e-9        # float slack when comparing accumulated phase against a tick count


def check_convention(name: str) -> str:
    """Return the canonical spelling of a convention name, or raise."""
    text = str(name).strip().lower()
    if text not in CONVENTIONS:
        raise ValueError(
            f"Manchester convention must be one of {CONVENTIONS} (got {name!r}); the two "
            "differ by line inversion, so picking the wrong one decodes every bit wrongly"
        )
    return text


def chip_pair(bit: int, convention: str = "ieee802.3") -> tuple[int, int]:
    """The two chips of one bit: (first half, second half).

    IEEE 802.3 rises in the middle of a 1 and falls in the middle of a 0, so a 1 is `0, 1`
    and a 0 is `1, 0` (AN1120a: "In Ethernet, a logical '0' is encoded as a high-to-low
    transition, while a logical '1' is encode as a low-to-high transition"). G.E. Thomas is
    the mirror image, so a 1 is `1, 0` and a 0 is `0, 1` (AN1470a).
    """
    bit = int(bit) & 1
    if check_convention(convention) == "ieee802.3":
        return 1 - bit, bit
    return bit, 1 - bit


def bit_from_midbit(rising: bool, convention: str = "ieee802.3") -> int:
    """The bit a mid-bit transition carries: rising is 1 in IEEE 802.3, 0 in Thomas."""
    return int(bool(rising)) if check_convention(convention) == "ieee802.3" else int(not rising)


def midbit_rising(bit: int, convention: str = "ieee802.3") -> bool:
    """The direction of the mandatory mid-bit transition of `bit`."""
    return bool(bit_from_midbit(bit, convention) == 1)


def sync_preamble(convention: str = "ieee802.3", idle_level: int = IDLE_LEVEL) -> str:
    """The two-bit sync pattern for a link that idles at `idle_level`.

    AN2358 gives the rule and both ways of satisfying it: "The first bit sent complies with
    the idle state of the line; for example, always start messages with a '0' bit when the
    idle state is low. Or, send a 0 to 1 (or 1 to 0) preamble to guarantee a correct
    synchronization in any situation." A line code has no start bit, so the first transition
    a receiver sees is a mid-bit transition only if the first chip on the wire *is* the idle
    level; this pattern is the pair of bits that guarantees it: the first bit agrees with the
    idle level, and the second brings the line back so the pattern also contains the
    low-to-high line transition AN2358 asks for.

    It comes out as the two conventions' mirrors of each other, which is what they must be:
    `"01"` for IEEE 802.3 idling high, `"10"` for G.E. Thomas idling high, `"01"` for Thomas
    idling low, `"10"` for IEEE 802.3 idling low.
    """
    check_convention(convention)
    first = int(idle_level) if convention == "thomas" else 1 - int(idle_level)
    return f"{first}{1 - first}"


def encode_levels(bits: Sequence[int], convention: str = "ieee802.3") -> list[int]:
    """The line levels of `bits`, two per bit: the reference encoder, as a level list.

    This is the whole line code: `encode_levels([1, 0])` is `[0, 1, 1, 0]` in IEEE 802.3, so
    the line falls 50 ns into bit 0 and rises 50 ns into bit 1 with no transition at the
    boundary, because the two bits differ. `ManchesterTx` drives this list (plus the
    `ManchesterFaults` a test asks for) and the monitor decodes it back.
    """
    check_convention(convention)
    out: list[int] = []
    for bit in bits:
        out.extend(chip_pair(bit, convention))
    return out


def bits_of(data: int | Iterable[int], bit_order: str = "lsb", width: int = 8) -> list[int]:
    """A byte (or an iterable of bytes) as a bit list, `bit_order` first.

    IEEE 802.3 sends the least significant bit of an octet first, so `bit_order="lsb"` is
    the default here; `"msb"` exists for the parts that send the other way (and for the
    tests). None of the saved documents state the bit order, so it is a parameter.
    """
    order = str(bit_order).strip().lower()
    if order not in ("lsb", "msb"):
        raise ValueError(f"bit_order must be 'lsb' or 'msb' (got {bit_order!r})")
    if isinstance(data, int):
        data = [data]
    out: list[int] = []
    for value in data:
        word = int(value) & ((1 << width) - 1)
        out += [(word >> i) & 1 for i in range(width)] if order == "lsb" else \
               [(word >> (width - 1 - i)) & 1 for i in range(width)]
    return out


def byte_of(bits: Sequence[int], bit_order: str = "lsb", width: int = 8) -> int:
    """The byte a bit list carries, `bit_order` first. Inverse of `bits_of`."""
    if len(bits) % width:
        raise ValueError(f"{len(bits)} bits is not a whole number of {width}-bit bytes")
    out = 0
    for index in range(0, len(bits), width):
        chunk = bits[index:index + width]
        word = sum(bit << i for i, bit in enumerate(chunk)) if bit_order == "lsb" else \
            sum(bit << (width - 1 - i) for i, bit in enumerate(chunk))
        out = (out << width) | word
    return out


@dataclass(frozen=True)
class ManchesterTimings:
    """One link's timing: its nominal bit rate and the tolerances the monitor judges by.

    `bit_rate` is the only thing that sets the wire's timing; the half bit is half of it and
    the mid-bit transition is expected at `1 / bit_rate / 2` after the start of a cell.

    The two tolerances are deliberately different things:

    * `interval_tol` is relative and applies to a *measured interval* between two
      transitions. 0.10 is AN2358's "more than 10 percent frequency tolerance on each side"
      for a receiver that oversamples x16. AN3015 accepts a looser 0.75..1.25 bit window for
      a one-bit interval, but +/-25 % leaves the half-bit window [0.25, 0.75] and the bit
      window [0.75, 1.25] touching at 0.75, so nothing could be classified; that is why the
      tighter of the two published numbers is the default and the looser one is only
      documented. Raise it through `profile_timings(..., {"interval_tol": 0.25})` to judge a
      link by the looser window.
    * `half_tol_ticks` and `grid_tol_ticks` are absolute and come from the bus, not from a
      spec: `core.py` places every transition on a tick boundary, so a 2.5-tick half bit can
      only be 2 or 3 ticks long, and any transmitter's mid-bit edges can only be measured to
      the tick. Both are overridable, and `grid_tol_ticks` is what catches a rate error that
      is legal interval by interval but lets the bit grid drift away.
    """

    name: str
    bit_rate: float
    interval_tol: float = 0.10
    half_tol_ticks: float = 1.0
    grid_tol_ticks: float = 1.0
    idle_level: int = IDLE_LEVEL
    min_gap_bits: float = 4.0
    cite: str = ""


#: The one profile shipped, the IEEE 802.3 rate AN1120a documents.
MANCHESTER_PROFILES: dict[str, ManchesterTimings] = {
    "ieee802.3-10base-t": ManchesterTimings(
        "ieee802.3-10base-t", 10e6, cite="Microchip AN1120a Table 2: 10Base-T bit time "
        "100 ns, i.e. 10 Mbit/s; minimum inter-packet gap 9.6 us = 96 bit times"
    ),
}


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, ns) / ns_per_tick))


def profile_timings(
    ns_per_tick: float,
    profile: str = DEFAULT_PROFILE,
    bit_rate: Optional[float] = None,
    overrides: Optional[dict] = None,
) -> dict:
    """Every window `ManchesterTx`, `ManchesterRx` and the monitor judge by, in whole ticks.

    The keys are what the three devices actually use:

    * `ticks_per_bit`, `half_ticks`: the nominal durations, floats, accumulated rather than
      rounded (a 2.5-tick half bit is sent as 2, 3, 2, 3... so the average is exact);
    * `half_lo`/`half_hi`, `bit_lo`/`bit_hi`: the two legal measured intervals, with both the
      spec tolerance and the tick quantisation folded in;
    * `grid_tol`: how far a mid-bit edge may sit from its place on the bit grid;
    * `min_gap_ticks`: the shortest gap between two messages the monitor will accept;
    * `stream_gap_ticks`: the interval above which the monitor stops looking for a bit and
      calls it the end of a message. It sits above 2 bit times because the longest interval
      a legal stream can produce is one bit time, and the longest a *broken* one can is two
      (a whole cell with no transition in it) -- which has to be reported as a violation, not
      mistaken for a new message.

    `overrides` replaces individual fields by name, which is how a test judges a link by
    different limits than the transmitter was built with: `{"interval_tol": 0.25}` or
    `{"grid_tol": 3}`. `bit_rate` (or the override of the same name) sets any rate, so the
    monitor is not tied to 10 Mbit/s.

    Raises ValueError if the bit rate is not positive or if the half-bit and bit windows
    would overlap at this tick rate: a monitor cannot classify a transition that fits both,
    and silently picking one would make it arbitrarily lenient.
    """
    try:
        base = MANCHESTER_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f"unknown Manchester profile {profile!r}; pick one of {sorted(MANCHESTER_PROFILES)}"
        ) from None
    fields = {
        "interval_tol": base.interval_tol,
        "half_tol_ticks": base.half_tol_ticks,
        "grid_tol_ticks": base.grid_tol_ticks,
        "idle_level": base.idle_level,
        "min_gap_bits": base.min_gap_bits,
    }
    overrides = dict(overrides or {})
    rate = overrides.pop("bit_rate", bit_rate if bit_rate is not None else base.bit_rate)
    for key, value in overrides.items():
        if key not in fields:
            raise ValueError(
                f"unknown timing override {key!r}; pick from {sorted(fields) + ['bit_rate']}"
            )
        fields[key] = value
    if not rate > 0:
        raise ValueError(f"the bit rate must be positive, got {rate!r}")
    if fields["interval_tol"] < 0:
        raise ValueError(f"interval_tol must be >= 0, got {fields['interval_tol']!r}")
    if fields["idle_level"] not in (0, 1):
        raise ValueError(f"idle_level must be 0 or 1, got {fields['idle_level']!r}")

    ticks_per_bit = 1e9 / (rate * ns_per_tick)
    half = ticks_per_bit / 2.0
    half_tol = max(float(fields["half_tol_ticks"]), fields["interval_tol"] * half)
    bit_tol = max(float(fields["half_tol_ticks"]), fields["interval_tol"] * ticks_per_bit)
    if half + half_tol >= ticks_per_bit - bit_tol:
        raise ValueError(
            f"at {ns_per_tick} ns/tick a {rate:g} bit/s bit period is {ticks_per_bit:.2f} "
            f"ticks, which leaves no room between the half-bit window "
            f"[{half - half_tol:.2f}, {half + half_tol:.2f}] and the bit window "
            f"[{ticks_per_bit - bit_tol:.2f}, {ticks_per_bit + bit_tol:.2f}]: a transition "
            "could not be classified. Use a slower bit rate, a coarser tick, or tighten "
            "half_tol_ticks / interval_tol."
        )
    out = dict(fields)
    out.update(
        profile=profile,
        name=profile,
        bit_rate=float(rate),
        ns_per_tick=float(ns_per_tick),
        ticks_per_bit=ticks_per_bit,
        half_ticks=half,
        half_lo=half - half_tol,
        half_hi=half + half_tol,
        bit_lo=ticks_per_bit - bit_tol,
        bit_hi=ticks_per_bit + bit_tol,
        grid_tol=float(fields["grid_tol_ticks"]),
        min_gap_ticks=fields["min_gap_bits"] * ticks_per_bit,
        stream_gap_ticks=2.0 * ticks_per_bit + max(bit_tol, half_tol),
        cite=base.cite,
    )
    return out


def classify_interval(interval: float, timings: Mapping[str, float]) -> str:
    """Which of the line code's two intervals `interval` is: "short", "long" or "bad".

    "short" is a transition at a bit boundary (half a bit after the previous one), "long" is
    the mandatory mid-bit transition (a whole bit after the previous one), and "bad" is
    neither: an interval longer than a bit time means a cell with no mid-bit transition in
    it, and an interval shorter than half a bit means two transitions too close together for
    the code. Both are violations; which rule name they get is the monitor's business.
    """
    if timings["half_lo"] - _EPS <= interval <= timings["half_hi"] + _EPS:
        return "short"
    if timings["bit_lo"] - _EPS <= interval <= timings["bit_hi"] + _EPS:
        return "long"
    return "bad"


@dataclass(frozen=True)
class ManchesterFaults:
    """Deliberate defects, for the self-tests and for a transmitter-under-test bench.

    All three keys are indexed the way the bits are counted *on the wire*, that is including
    the sync preamble (`ManchesterTx` logs the preamble in `TxMessage.bits`, so chip index
    `2 * bit` is the first half of that bit and `2 * bit + 1` the second).

    * `rate_scale` multiplies every bit period: 1.05 is a transmitter 5 % fast, which meets
      every interval window at the default 10 % tolerance but walks the bit grid away from
      its anchor, which is what `tJIT` exists to catch.
    * `stretch_halves` holds or shortens one chip: each entry is `(bit, half, ticks)`, so
      `((3, 1, 2.0),)` makes the second half of bit 3 two ticks longer than nominal.
    * `drop_mid_bits` are the bits whose *mandatory mid-bit transition is missing*, the
      classic single-event error. The line simply holds its level through the second half of
      the cell, which shifts the following bit-boundary transition as well.

    The default (all empty, rate_scale 1.0) is a legal transmitter.
    """

    rate_scale: float = 1.0
    stretch_halves: tuple = ()
    drop_mid_bits: tuple = ()

    def half_ticks(self, bit: int, half: int, nominal: float) -> float:
        """Length of one chip, faults applied, never shorter than a tick."""
        for f_bit, f_half, extra in self.stretch_halves:
            if f_bit == bit and f_half == half:
                return max(1.0, nominal + float(extra))
        return nominal


@dataclass
class TxMessage:
    """One message as the transmitter drove it, preamble included."""

    bits: list[int]                 # preamble first, then the data bits
    preamble_len: int = 0
    start_tick: int = -1            # tick the first chip began on
    end_tick: Optional[int] = None  # first idle tick after the last chip
    faults: Optional[ManchesterFaults] = None

    @property
    def data_bits(self) -> list[int]:
        return self.bits[self.preamble_len:]

    @property
    def done(self) -> bool:
        return self.end_tick is not None


class ManchesterTx(Device):
    """Transmitter: queued messages as Manchester chips on one push-pull net.

    `bit_rate` is in bits per second and is honoured exactly on average: half-bit phase is
    accumulated, never rounded per chip, so a 10 Mbit/s link on a 20 ns bus sends 2, 3, 2, 3
    ticks per half and its 100th bit is as early as its 1st (AN1120a's 100 ns bit time is
    5.0 ticks there, so nothing is lost at all; at other rates the phase carry is what keeps
    the average right).

    `send()` takes bytes or bit lists the way `UartTx.send` does, one message per call.
    `bit_order` says which end of a byte goes on the wire first (`"lsb"`, IEEE 802.3's
    choice). `preamble` is a string of "0"/"1" characters sent before every message; the
    default `"01"` is the sync preamble AN2358 asks for ("send a 0 to 1 (or 1 to 0)
    preamble to guarantee a correct synchronization in any situation") -- what actually
    matters is that the first chip on the wire equals the idle level, so that the first
    transition a receiver sees is a mid-bit transition rather than a boundary one. Pass
    `preamble=""` to leave it out and watch the monitor's `tSYNC` rule.

    `gap_bits` is the idle time between messages; it defaults to the profile's
    `min_gap_bits` so that messages are always separable by a receiver. With a gap of zero
    two messages run into one stream on the wire and a monitor cannot see where one ends.

    `faults` drives the line wrong on purpose; see `ManchesterFaults`.
    """

    name = "man_tx"

    def __init__(
        self,
        net: str = "tx",
        bit_rate: float = 10e6,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        convention: str = "ieee802.3",
        bit_order: str = "lsb",
        preamble: Optional[str] = None,
        gap_bits: Optional[float] = None,
        profile: str = DEFAULT_PROFILE,
        timing: Optional[dict] = None,
        faults: Optional[ManchesterFaults] = None,
        name: Optional[str] = None,
    ):
        if not str(net):
            raise ValueError("net must be a net name")
        self.net = net
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else float(ns_per_tick)
        self.bit_rate = float(bit_rate)
        self.convention = check_convention(convention)
        self.bit_order = str(bit_order).strip().lower()
        bits_of(0, self.bit_order)                      # validate now, not at the first send
        self._preamble_arg = preamble
        self.preamble = "" if preamble is None else str(preamble)
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        self.faults = faults or ManchesterFaults()
        if not self.faults.rate_scale > 0:
            raise ValueError(f"rate_scale must be positive, got {self.faults.rate_scale!r}")
        self._gap_bits = None if gap_bits is None else float(gap_bits)
        if name:
            self.name = name
        self.queue: deque[list[int]] = deque()
        self.messages: list[TxMessage] = []
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        self.timings = profile_timings(
            self.ns_per_tick, self.profile, bit_rate=self.bit_rate,
            overrides=self.timing_overrides,
        )
        self._tpb = self.timings["ticks_per_bit"] * self.faults.rate_scale
        self._half_ticks = self._tpb / 2.0
        if self._preamble_arg is None:
            self.preamble = sync_preamble(self.convention, self.timings["idle_level"])
        if self.preamble and set(self.preamble) - {"0", "1"}:
            raise ValueError(
                f"preamble must be '0' and '1' characters, got {self.preamble!r}"
            )

    def bind(self, bus: Bus) -> "ManchesterTx":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        return self

    @property
    def gap_bits(self) -> float:
        return self._gap_bits if self._gap_bits is not None else self.timings["min_gap_bits"]

    @property
    def idle_level(self) -> int:
        return self.timings["idle_level"]

    def _reset(self) -> None:
        self._levels: Optional[list[int]] = None
        self._msg: Optional[TxMessage] = None
        self._half = 0                  # index into the chip list
        self._phase = 0.0               # ticks elapsed in the current chip
        self._idle_until = 0            # tick before which no new message may start

    # ---- the queue ---------------------------------------------------------------
    def send(self, *data: int | Iterable[int]) -> "ManchesterTx":
        """Queue messages: send(0x55), send(b"ab"), send(b"ab", 0x0C)."""
        for item in data:
            if isinstance(item, str):
                raise ValueError(
                    f"send() takes bytes or bit lists, not a string: {item!r}. Use "
                    "ManchesterTx(preamble=...) for a bit pattern."
                )
            self.queue.append(bits_of(item, self.bit_order))
        return self

    def send_bits(self, bits: Sequence[int]) -> "ManchesterTx":
        """Queue one message given as a bit list, sent most significant bit first."""
        self.queue.append([int(b) & 1 for b in bits])
        return self

    @property
    def pending(self) -> int:
        """Messages queued but not started yet."""
        return len(self.queue)

    @property
    def busy(self) -> bool:
        return self._levels is not None or bool(self.queue)

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def sent(self) -> int:
        """Messages finished so far."""
        return sum(1 for m in self.messages if m.done)

    def run(self, bus: Bus, limit: int = 1_000_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out first."""
        return bus.run_until(lambda _b: self.idle, limit)

    # ---- the line ---------------------------------------------------------------
    def chip_levels(self, bits: Sequence[int]) -> list[int]:
        """The chip list this message drives, `ManchesterFaults` applied.

        The clean case is `encode_levels`; `drop_mid_bits` holds the first half's level
        through the second half of that cell, which removes the mandatory mid-bit transition.
        Subclass and override to model a different encoder.
        """
        levels = encode_levels(bits, self.convention)
        for bit in self.faults.drop_mid_bits:
            if 0 <= bit < len(bits):
                levels[2 * bit + 1] = levels[2 * bit]
        return levels

    def _chip_ticks(self, half: int) -> float:
        return self.faults.half_ticks(half // 2, half % 2, self._half_ticks)

    def _drive(self, half: Optional[int]) -> int:
        if half is None:
            return self.idle_level
        return self._levels[half]

    # ---- the device ---------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, int]:
        if self._levels is not None:
            while self._phase >= self._chip_ticks(self._half) - _EPS:
                self._phase -= self._chip_ticks(self._half)
                self._half += 1
                if self._half >= len(self._levels):
                    self._end_message(t)
        if self._levels is None and self.queue and t >= self._idle_until:
            self._begin(t)
        half = self._half if self._levels is not None else None
        if half is not None:
            self._phase += 1.0
        return {self.net: self._drive(half)}

    def _begin(self, t: int) -> None:
        bits = self.queue.popleft()
        preamble = [int(c) for c in self.preamble]
        self._msg = TxMessage(
            bits=preamble + bits, preamble_len=len(preamble), start_tick=t,
            faults=self.faults,
        )
        self.messages.append(self._msg)
        self._levels = self.chip_levels(self._msg.bits)
        self._half = 0
        self._phase = 0.0

    def _end_message(self, t: int) -> None:
        if self._msg is not None:
            self._msg.end_tick = t
        self._levels = None
        self._msg = None
        self._half = 0
        self._phase = 0.0
        self._idle_until = t + self.gap_bits * self._tpb


@dataclass
class RxBit:
    """One bit as the receiver decoded it, with where its mid-bit transition was."""

    index: int          # index in the message, preamble included
    value: int
    tick: int           # tick the mid-bit transition was seen on
    rising: bool
    dev: float = 0.0    # ticks away from the place it has on the bit grid


@dataclass
class RxMessage:
    """One decoded message: the bits, and the ticks the stream ran between."""

    start_tick: int = -1
    end_tick: Optional[int] = None
    bits: list[int] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    preamble_len: int = 0

    @property
    def data_bits(self) -> list[int]:
        return self.bits[self.preamble_len:]

    def value(self, bit_order: str = "lsb", width: int = 8) -> int:
        """The data bits as one integer (`bit_order` first)."""
        return byte_of(self.data_bits, bit_order, width) if len(self.data_bits) % width == 0 \
            else sum(bit << i for i, bit in enumerate(self.data_bits))

    def as_bytes(self, bit_order: str = "lsb", width: int = 8) -> bytes:
        """The data bits as bytes, dropping a trailing partial byte."""
        whole = len(self.data_bits) // width * width
        return bytes(
            byte_of(self.data_bits[i:i + width], bit_order, width)
            for i in range(0, whole, width)
        )

    @property
    def clean(self) -> bool:
        return not self.errors


class ManchesterRx(Device):
    """Receiver: bits back out of the transitions. Passive, it never drives.

    There is no start bit in the line code, so the receiver does what a real one has to: it
    anchors on the first transition of a stream, takes it for the mid-bit transition of bit
    0, and from then on classifies every interval (see `classify_interval`). A short interval
    is a transition at a bit boundary and is skipped; a long one is the mandatory mid-bit
    transition and yields the bit from its direction. Anything else is a lost clock: the
    receiver records it in `errors`, counts a `resyncs`, and re-anchors on the next
    transition, which is where the two sources agree a receiver recovers -- AN2358 notes that
    a `1-then-0` pair (and a `0-then-1`) has only mid-bit transitions and "forces receiver
    realignment on the right edge", and AN3435's receiver rejects edges until the next valid
    edge time.

    Every message is closed when the line goes quiet for longer than a stream gap, so
    `messages` lines up with the transmitter's. `bits` is every decoded bit of the run,
    preamble included, and `worst_dev_ticks` is the largest excursion of a mid-bit
    transition from the bit grid it is on, which is the receiver's own view of the link's
    timing accuracy.
    """

    name = "man_rx"

    def __init__(
        self,
        net: str = "tx",
        bit_rate: float = 10e6,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        convention: str = "ieee802.3",
        bit_order: str = "lsb",
        preamble_bits: int = 0,
        profile: str = DEFAULT_PROFILE,
        timing: Optional[dict] = None,
        name: Optional[str] = None,
    ):
        self.net = net
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else float(ns_per_tick)
        self.bit_rate = float(bit_rate)
        self.preamble_bits = int(preamble_bits)
        if self.preamble_bits < 0:
            raise ValueError(f"preamble_bits must be >= 0, got {preamble_bits!r}")
        self.convention = check_convention(convention)
        self.bit_order = str(bit_order).strip().lower()
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        if name:
            self.name = name
        self.timings = profile_timings(
            self.ns_per_tick, self.profile, bit_rate=self.bit_rate,
            overrides=self.timing_overrides,
        )
        self.bits: list[int] = []          # every decoded bit, all messages
        self.bit_log: list[RxBit] = []
        self.messages: list[RxMessage] = []
        self.errors: list[str] = []        # "t=<tick> <what was wrong>"
        self.framing_errors = 0
        self.resyncs = 0
        self.worst_dev_ticks = 0.0
        self._reset()

    def bind(self, bus: Bus) -> "ManchesterRx":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.timings = profile_timings(
            self.ns_per_tick, self.profile, bit_rate=self.bit_rate,
            overrides=self.timing_overrides,
        )
        return self

    def _reset(self) -> None:
        self._level = self.timings["idle_level"]
        self._last_edge: Optional[int] = None
        self._last_rising = False
        self._prev_kind: Optional[str] = None    # "short" | "long" | None
        self._grid: Optional[float] = None       # expected tick of the next mid-bit edge
        self._index = 0                          # next bit index within the message
        self._msg: Optional[RxMessage] = None

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.errors

    @property
    def values(self) -> list[int]:
        """Every decoded bit of the run, in order."""
        return list(self.bits)

    def decoded_bytes(self, bit_order: Optional[str] = None, width: int = 8) -> bytes:
        """Every message's data bits as bytes, messages concatenated."""
        out = bytearray()
        for msg in self.messages:
            out += msg.as_bytes(bit_order or self.bit_order, width)
        return bytes(out)

    def report(self, limit: int = 10) -> str:
        head = (f"Manchester {self.timings['bit_rate']:g} bit/s at {self.ns_per_tick} ns/tick: "
                f"{len(self.messages)} message(s), {len(self.bits)} bits, "
                f"{self.framing_errors} framing error(s), {self.resyncs} resync(s), "
                f"worst grid deviation {self.worst_dev_ticks:.2f} ticks")
        return head + "".join(f"\n  {e}" for e in self.errors[:limit])

    # ---- the device ---------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        level = pins.get(self.net, self.timings["idle_level"])
        if level != self._level:
            rising = level == 1
            self._edge(t, rising)
            self._level = level
        return {}

    def _edge(self, t: int, rising: bool) -> None:
        previous, self._last_edge = self._last_edge, t
        if previous is None:
            self._anchor(t, rising)
            return
        interval = t - previous
        if interval > self.timings["stream_gap_ticks"]:
            self._close_message(t)
            self._anchor(t, rising)
            return
        kind = classify_interval(interval, self.timings)
        if kind == "short":
            if self._prev_kind == "short":
                self._bit(t, rising)          # the cell's mid-bit transition
            else:
                self._prev_kind = "short"      # a transition at a bit boundary: skip it
            return
        if kind == "long":
            if self._prev_kind == "short":
                # A transition at a bit boundary must be followed by that cell's mid-bit
                # transition half a bit later. This one arrives a whole bit later, so it is
                # the *next* cell's boundary transition: skip it and keep the phase.
                self._error(t, "a transition at a bit boundary must be followed by that "
                               "cell's mid-bit transition half a bit later, not a whole "
                               "bit later", resync=False)
                self._prev_kind = "short"
            else:
                self._bit(t, rising)
            return
        self._error(t, f"interval {interval} ticks is neither a half bit "
                       f"({self.timings['half_lo']:.2f}..{self.timings['half_hi']:.2f}) nor a "
                       f"bit ({self.timings['bit_lo']:.2f}..{self.timings['bit_hi']:.2f}): "
                       "the clock is lost")
        self._anchor(t, rising)

    def _anchor(self, t: int, rising: bool) -> None:
        """Start a stream: this transition is bit 0's mid-bit transition."""
        self._msg = RxMessage(start_tick=t, preamble_len=self.preamble_bits)
        self.messages.append(self._msg)
        self._grid = float(t)
        self._index = 0
        self._prev_kind = None
        self._bit(t, rising)

    def _bit(self, t: int, rising: bool) -> None:
        value = bit_from_midbit(rising, self.convention)
        dev = t - (self._grid if self._grid is not None else t)
        self.bit_log.append(RxBit(self._index, value, t, rising, dev))
        self.bits.append(value)
        if self._msg is not None:
            self._msg.bits.append(value)
        self.worst_dev_ticks = max(self.worst_dev_ticks, abs(dev))
        self._index += 1
        self._grid = (self._grid if self._grid is not None else t) + self.timings["ticks_per_bit"]
        self._prev_kind = "long"

    def _error(self, t: int, what: str, resync: bool = True) -> None:
        self.errors.append(f"t={t} {what}")
        self.framing_errors += 1
        self.resyncs += int(resync)
        if self._msg is not None:
            self._msg.errors.append(f"t={t} {what}")

    def _close_message(self, t: int) -> None:
        if self._msg is not None:
            self._msg.end_tick = t
            self._msg = None
        self._grid = None
        self._index = 0


#: The open end of a one-sided window, e.g. "at least this long" for tIDLE and tGAP.
INF = float("inf")


@dataclass(frozen=True)
class Violation:
    """One rule the line broke, with the tick it broke on and the window it broke it against.

    `measured` is in ticks -- the interval the wire produced, the excursion of the bit grid,
    or the length of the idle run, depending on `rule`. `window` is the (low, high) pair the
    monitor judged it by -- `(0, limit)` for "the line must not stay off idle longer than
    this", `(minimum, INF)` for "there must be at least this much idle line" -- and
    `short`/`long` say which side of that window it fell off.
    """

    tick: int
    rule: str
    measured: float
    window: tuple[float, float]
    detail: str = ""

    @property
    def short(self) -> bool:
        return self.measured < self.window[0]

    @property
    def long(self) -> bool:
        return self.measured > self.window[1]

    def __str__(self) -> str:
        side = "below" if self.short else "above"
        return (f"t={self.tick} {self.rule} {self.measured:g} ticks {side} the "
                f"[{self.window[0]:g}, {self.window[1]:g}] window"
                + (f" ({self.detail})" if self.detail else ""))


@dataclass
class Edge:
    """One transition the monitor saw on the line."""

    tick: int
    rising: bool
    interval: Optional[int]      # ticks since the previous transition, None at a stream start
    kind: str                    # "start" | "short" | "long" | "bad"
    bit: Optional[int] = None    # the bit a mid-bit transition decoded, else None
    dev: float = 0.0             # ticks away from the bit grid

    def __str__(self) -> str:
        gap = "-" if self.interval is None else f"{self.interval}"
        return (f"t={self.tick} {'rise' if self.rising else 'fall'} after {gap} ticks "
                f"[{self.kind}]" + (f" bit={self.bit}" if self.bit is not None else ""))


@dataclass
class Stream:
    """One run of transitions the monitor read as one message."""

    start_tick: int
    end_tick: Optional[int] = None
    bits: list[int] = field(default_factory=list)
    violations: int = 0

    @property
    def value(self) -> int:
        """The decoded bits as one integer, first bit least significant."""
        return sum(bit << i for i, bit in enumerate(self.bits))


class ManchesterTimingChecker(Device):
    """Passive monitor: decodes the line and flags every timing and framing violation.

    It drives nothing, so it sees the pins exactly as an analyser would and does not adjust
    for the two-phase tick's one-tick sampling latency. It measures what the wire did.

    Eight rules, each with the tick it fired on in `violations`:

    * `tHALF`    a transition less than half a bit after the previous one: two transitions
                 too close together for the code (AN3435's receiver "can reject further
                 edges that occur until the next valid edge transition time").
    * `tMID-BIT` a transition that is neither at a bit boundary nor at the mid-bit of a
                 cell: it is too late to be the boundary transition and too early to be the
                 next mid-bit one. At 10 Mbit/s on a 20 ns bus the two windows nearly touch,
                 so this band is empty there and the rule only bites at slower rates; at
                 1 Mbit/s a half bit stretched by four ticks lands in it.
    * `tBIT`     a transition more than one bit time after the previous one: a bit cell with
                 no mid-bit transition in it, which the line code does not allow (AN2358:
                 "each bit is transmitted with a transition in the middle of bit time"). One
                 and a half bit times is the shape a dropped mid-bit transition takes when
                 the cell's boundary transition is still there, and two bit times when it
                 is not.
    * `tFRAMING` a transition at a bit boundary that is *not* followed by the mid-bit
                 transition half a bit later. The wire form of the same error.
    * `tJIT`     a transition further than `grid_tol` from its place on the bit grid, which
                 is anchored on the first mid-bit transition of the stream and advances by
                 exactly one bit per bit. This is the only rule a per-interval window cannot
                 do: a transmitter with a small rate error meets every interval window and
                 still walks off its clock over a message.
    * `tSYNC`    the stream does not begin with the sync pattern this link sends, or its
                 first bit disagrees with the level the line was at. Either way the first
                 transition a receiver sees would be a bit-boundary transition, so its
                 anchor would be half a bit out and every bit after it would be the wrong
                 one. AN2358 gives the two ways out: "always start messages with a '0' bit
                 when the idle state is low. Or, send a 0 to 1 (or 1 to 0) preamble to
                 guarantee a correct synchronization". This is the only rule that can see a
                 half-bit mis-anchor at all, because a mis-anchored line is a perfectly legal
                 Manchester waveform of the wrong bits.
    * `tIDLE`    the line stayed off the idle level for longer than a stream gap without a
                 transition: the link is stuck, not Manchester. A line held *at* the idle
                 level is fine and is never flagged, however long it stays there.
    * `tGAP`     two messages with less than `min_gap_bits` of idle line between them, which
                 is legal on the wire but leaves a receiver no boundary to lock onto. (With
                 no gap at all there is no transition to measure and the monitor sees one
                 long stream -- also documented, also legal.)

    `preamble` is the pattern the monitor expects at the head of every stream; it defaults to
    `sync_preamble()` for the convention and the idle level, which is the same default the
    transmitter uses. `check_sync=False` turns both halves of `tSYNC` off, for a bench that
    deliberately watches a stream with no way to tell its phase.

    What is *not* flagged, because the line code has no such thing: a transition at a bit
    boundary is optional and carries no data, so a missing one between two different bits and
    a present one between two equal bits are both correct, and the monitor only checks that
    a present one is in the right place.

    `stats` keeps what was measured -- transitions, bits, streams, the shortest and longest
    intervals -- and `streams` holds the decoded bits per message, so a test can assert both
    that the traffic was legal and that the monitor understood it.
    """

    name = "man_check"

    def __init__(
        self,
        net: str = "tx",
        bit_rate: float = 10e6,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        convention: str = "ieee802.3",
        preamble: Optional[str] = None,
        profile: str = DEFAULT_PROFILE,
        timing: Optional[dict] = None,
        check_sync: bool = True,
        name: Optional[str] = None,
    ):
        self.net = net
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else float(ns_per_tick)
        self.bit_rate = float(bit_rate)
        self.convention = check_convention(convention)
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        self.check_sync = bool(check_sync)
        if name:
            self.name = name
        self.timings = profile_timings(
            self.ns_per_tick, self.profile, bit_rate=self.bit_rate,
            overrides=self.timing_overrides,
        )
        self.preamble = (sync_preamble(self.convention, self.timings["idle_level"])
                         if preamble is None else str(preamble))
        if self.preamble and set(self.preamble) - {"0", "1"}:
            raise ValueError(f"preamble must be '0' and '1' characters, got {preamble!r}")
        self.violations: list[Violation] = []
        self.edges: list[Edge] = []
        self.streams: list[Stream] = []
        self._reset()

    def bind(self, bus: Bus) -> "ManchesterTimingChecker":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.timings = profile_timings(
            self.ns_per_tick, self.profile, bit_rate=self.bit_rate,
            overrides=self.timing_overrides,
        )
        return self

    def _reset(self) -> None:
        self._level = self.timings["idle_level"]
        self._last_edge: Optional[int] = None
        self._idle_since = 0            # tick the line last settled on the idle level
        self._stuck_since: Optional[int] = None
        self._stuck_flagged = False
        self._prev_kind: Optional[str] = None
        self._grid: Optional[float] = None
        self._stream: Optional[Stream] = None
        self._pattern_ok = True     # has this stream had a violation yet?
        self.framing_errors = 0     # intervals that were neither a half bit nor a bit

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def bits(self) -> list[int]:
        """Every decoded bit of the run, in order."""
        return [b for s in self.streams for b in s.bits]

    @property
    def stats(self) -> dict:
        """What was measured. `shortest`/`longest` are transition intervals, in ticks."""
        intervals = [e.interval for e in self.edges if e.interval is not None]
        inside = [e.interval for e in self.edges if e.kind in ("short", "long")]
        return {
            "transitions": len(self.edges),
            "bits": len(self.bits),
            "streams": len(self.streams),
            "violations": len(self.violations),
            "shortest_interval": min(intervals) if intervals else None,
            "longest_interval": max(intervals) if intervals else None,
            "shortest_in_stream": min(inside) if inside else None,
            "longest_in_stream": max(inside) if inside else None,
            "worst_dev": max((abs(e.dev) for e in self.edges), default=0.0),
        }

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def report(self, limit: int = 10) -> str:
        t = self.timings
        head = (f"Manchester {t['bit_rate']:g} bit/s ({t['ticks_per_bit']:.2f} ticks/bit, "
                f"half {t['half_ticks']:.2f}) at {self.ns_per_tick} ns/tick: "
                f"{len(self.streams)} stream(s), {len(self.bits)} bits, "
                f"{len(self.edges)} transitions, {len(self.violations)} violation(s); "
                f"windows half [{t['half_lo']:.2f}, {t['half_hi']:.2f}], "
                f"bit [{t['bit_lo']:.2f}, {t['bit_hi']:.2f}], "
                f"grid +/-{t['grid_tol']:g}")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: float, window: tuple[float, float],
                   detail: str = "") -> None:
        self.violations.append(Violation(t, rule, measured, window, detail))
        # A violation has already named what went wrong; after it the monitor re-anchors,
        # so the sync pattern it decodes next says nothing about the transmitter and
        # reporting it would only bury the real cause.
        self._pattern_ok = False
        if self._stream is not None:
            self._stream.violations += 1

    # ---- the monitor --------------------------------------------------------------
    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        level = pins.get(self.net, self.timings["idle_level"])
        if level != self._level:
            if level == self.timings["idle_level"]:
                self._idle_since = t
                self._stuck_since, self._stuck_flagged = None, False
            else:
                self._stuck_since = t
            self._edge(t, level == 1)
            self._level = level
        elif level != self.timings["idle_level"]:
            self._stuck(t)
        return {}

    def _stuck(self, t: int) -> None:
        """The line has sat off the idle level, without a transition, for too long."""
        if self._stuck_since is None or self._stuck_flagged:
            return
        limit = self.timings["stream_gap_ticks"]
        if t - self._stuck_since > limit:
            self._stuck_flagged = True
            self._violation(
                self._stuck_since, "tIDLE", t - self._stuck_since, (0.0, limit),
                f"the line sat at level {self._level} with no transition for "
                f"{t - self._stuck_since:g} ticks; between messages it must be at the idle "
                f"level {self.timings['idle_level']}",
            )

    def _edge(self, t: int, rising: bool) -> None:
        previous, self._last_edge = self._last_edge, t
        if previous is None:
            self._anchor(t, rising, None)
            return
        interval = t - previous
        if interval > self.timings["stream_gap_ticks"]:
            self._end_stream(t)
            self._anchor(t, rising, interval)
            return
        kind = classify_interval(interval, self.timings)
        if kind == "short":
            if self._prev_kind == "short":
                self._midbit(t, rising, interval, kind)   # this cell's mid-bit transition
            else:
                self._boundary(t, rising, interval, kind)  # a transition at a bit boundary
            return
        if kind == "long":
            if self._prev_kind == "short":
                self._violation(
                    t, "tFRAMING", interval,
                    (self.timings["half_lo"], self.timings["half_hi"]),
                    "a transition at a bit boundary must be followed by that cell's mid-bit "
                    "transition half a bit later, not a whole bit later",
                )
                self._boundary(t, rising, interval, kind)
            else:
                self._midbit(t, rising, interval, kind)
            return
        self._interval_bad(t, rising, interval)
        self._anchor(t, rising, interval, fresh=False)

    def _interval_bad(self, t: int, rising: bool, interval: int) -> None:
        """Report an interval that fits neither window, under the rule that names why."""
        w = self.timings
        if interval < w["half_lo"]:
            rule = "tHALF"
            window = (w["half_lo"], w["half_hi"])
            detail = ("a transition less than half a bit after the previous one: two "
                      "transitions that close together are not in the line code")
        elif interval > w["bit_hi"]:
            rule = "tBIT"
            window = (w["bit_lo"], w["bit_hi"])
            detail = ("a transition more than one bit time after the previous one: a bit "
                      "cell with no mid-bit transition in it. One and a half bit times is "
                      "what a dropped mid-bit transition leaves behind")
        else:
            rule = "tMID-BIT"
            # The window it missed is the half-bit one: the transition is past its end and
            # has not reached the bit window, which the detail spells out.
            window = (w["half_lo"], w["half_hi"])
            detail = (f"too late to be the transition at a bit boundary (past "
                      f"{w['half_hi']:g} ticks) and too early to be the mid-bit transition "
                      f"of the next bit cell (short of {w['bit_lo']:g} ticks)")
        self._violation(t, rule, interval, window, detail)
        self.edges.append(Edge(t, rising, interval, "bad"))
        self.framing_errors += 1

    def _midbit(self, t: int, rising: bool, interval: Optional[int], kind: str) -> None:
        """A mandatory mid-bit transition: check the grid, then take the bit off it."""
        grid = self.timings["grid_tol"]
        dev = 0.0 if self._grid is None else t - self._grid
        if abs(dev) > grid + _EPS:
            self._violation(
                t, "tJIT", dev, (-grid, grid),
                f"mid-bit transition {abs(dev):.2f} ticks from its place on the bit grid",
            )
        value = bit_from_midbit(rising, self.convention)
        index = len(self._stream.bits) if self._stream is not None else 0
        self.edges.append(Edge(t, rising, interval, kind, value, dev))
        if self._stream is not None:
            self._stream.bits.append(value)
        if self.check_sync and self._pattern_ok and index < len(self.preamble):
            # The sync pattern is the only thing on the wire that says where the bit cells
            # are: a mis-anchored stream is a legal waveform of the wrong bits (a stream of
            # ones read as a stream of zeros, say), so timing alone cannot catch it.
            if value != int(self.preamble[index]):
                self._violation(
                    t, "tSYNC", self.timings["half_ticks"], (0.0, 0.0),
                    f"bit {index} of this message decoded as {value}, but the sync pattern "
                    f"this link sends is {self.preamble!r}. Either the message does not start "
                    f"with the pattern, or the receiver is anchored half a bit out",
                )
        self._grid = (t if self._grid is None else self._grid + self.timings["ticks_per_bit"])
        self._prev_kind = "long"

    def _boundary(self, t: int, rising: bool, interval: int, kind: str) -> None:
        """An optional transition at a bit boundary: it carries no data, only a position."""
        grid = self.timings["grid_tol"]
        expected = None if self._grid is None else self._grid - self.timings["half_ticks"]
        dev = 0.0 if expected is None else t - expected
        if abs(dev) > grid + _EPS:
            self._violation(
                t, "tJIT", dev, (-grid, grid),
                f"bit-boundary transition {abs(dev):.2f} ticks from its place on the bit grid",
            )
        self.edges.append(Edge(t, rising, interval, kind, None, dev))
        self._prev_kind = "short"

    def _anchor(self, t: int, rising: bool, interval: Optional[int],
                fresh: bool = True) -> None:
        """Start a stream here: this transition is bit 0's mandatory mid-bit transition.

        `fresh` is False when this anchor is a re-synchronisation after a violation: the
        monitor no longer knows where the bit cells are, so it does not judge the sync
        pattern of the bits that follow (it already reported what went wrong).
        """
        self._stream = Stream(start_tick=t)
        self.streams.append(self._stream)
        self._grid = float(t)
        self._prev_kind = None
        self._pattern_ok = fresh
        if self.check_sync:
            first = self._level                 # the level the line sat at before this edge
            half = chip_pair(bit_from_midbit(rising, self.convention), self.convention)[0]
            if half != first:
                self._violation(
                    t, "tSYNC", self.timings["half_ticks"], (0.0, 0.0),
                    f"the line was at level {first} but the first bit of the stream has a "
                    f"first half of level {half}: a receiver's first transition would be a "
                    f"bit-boundary transition and its anchor half a bit out. Start the "
                    f"message with a bit that agrees with the idle level, or send a preamble",
                )
        self._midbit(t, rising, interval, "start")

    def _end_stream(self, t: int) -> None:
        """The gap was long enough to be between messages: close the stream and judge it."""
        if self._stream is not None:
            self._stream.end_tick = t
        min_gap = self.timings["min_gap_ticks"]
        idle_ticks = t - self._idle_since
        if idle_ticks < min_gap - _EPS:
            self._violation(
                t, "tGAP", idle_ticks, (min_gap, INF),
                f"only {idle_ticks:g} ticks of idle line before this message; the profile "
                f"asks for {min_gap:g} ({self.timings['min_gap_bits']:g} bit times) so a "
                f"receiver can find the boundary",
            )
        self._stream = None
        self._grid = None
        self._prev_kind = None


# ---- harness helpers -------------------------------------------------------------
def manchester_bus(ns_per_tick: float = 20.0, net: str = "tx", trace: bool = False) -> Bus:
    """A bus with the one push-pull net a Manchester link needs."""
    bus = Bus(ns_per_tick=ns_per_tick, trace=trace)
    bus.add_net(net, push_pull=True)
    return bus


def _ensure_net(bus: Bus, net: str) -> None:
    if net not in bus.nets:
        bus.add_net(net, push_pull=True)


def attach_tx(bus: Bus, **kwargs) -> ManchesterTx:
    """Build a ManchesterTx that takes its timing from `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "tx"))
    tx = ManchesterTx(bus=bus, **kwargs)
    bus.attach(tx)
    return tx


def attach_rx(bus: Bus, **kwargs) -> ManchesterRx:
    """Build a ManchesterRx for `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "tx"))
    rx = ManchesterRx(bus=bus, **kwargs)
    bus.attach(rx)
    return rx


def attach_checker(bus: Bus, **kwargs) -> ManchesterTimingChecker:
    """Build a passive ManchesterTimingChecker for `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "tx"))
    chk = ManchesterTimingChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk
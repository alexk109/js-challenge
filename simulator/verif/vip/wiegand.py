"""Pin-level Wiegand reference models on the tick-based bus in verif/vip/core.py.

Two open-drain nets, "d0" and "d1", both pulled high: the bus idles at 1 on both, and a
bit is a **low pulse** on one of them -- a 0 on D0, a 1 on D1. Both lines are high again
between pulses, so a device here drives 0 or releases (None) and never drives 1
([wiki, research/ref/protocols/wiegand/03_wikipedia_wiegand_interface.md],
[BALTECH, .../04_baltech_wiegand.md]). There is no clock, no framing symbol and no
handshake: every falling edge is one bit, and the value of the bit is the line that fell.

Three devices:

  WiegandReader      the card-reader model. Queues frames, drives each bit as a pulse of
                     `pw` ns with a pause of `pi` ns after it, both from the spec, and
                     leaves the lines idle between frames. Per-frame width and gap lists
                     let a test break one rule at a time.
  WiegandReceiver    a passive door-controller decoder, the other model on the wire: it
                     counts falling edges, times the pulses, closes a frame when the
                     lines have been quiet for longer than the pulse interval allows, and
                     decodes facility code, ID number and parity from what it saw.
  WiegandChecker     a passive protocol checker. It measures you, not the devices, and
                     appends every timing or framing violation it finds to `violations`
                     with the tick it happened on.

Typical use:

    bus = wiegand_bus(ns_per_tick=20.0)             # 50 MHz
    rdr = attach_reader(bus, facility=123, number=4567)
    rcv = attach_receiver(bus)
    chk = attach_checker(bus)                       # optional, passive
    rdr.run(bus, limit=200_000)
    assert chk.clean, chk.report()
    assert rcv.frames[-1].facility == 123 and rcv.frames[-1].number == 4567

Every number below is cited to text saved under research/ref/protocols/wiegand/; the
summary is that directory's README. Nothing here comes from memory.

Timing, in nanoseconds, and what each one is for:

  tPW = pulse width time, the duration of a low pulse       SIA AC-01, via [Farpointe,
       01_...]: "a Pulse Width time between 20 uS and 100 uS"
  tPI = pulse interval time, the pause *between* two low     SIA AC-01, via [Farpointe,
       pulses -- end of one pulse to the start of the next,  01_...]: "a Pulse Interval
       which is how BALTECH defines it [04_...]               time between 200 uS and
                                                                20 mS"
  the reader's defaults, 50 us and 2 ms, are BALTECH's shipped PulseWidth and
       PulseInterval defaults [04_...] and sit strictly inside the SIA window, so the
       default traffic this model produces is legal by two independent documents.

Both tPW and tPI are windows with a floor *and* a ceiling, so -- unlike, say, an I2C
minimum -- they cannot be met by being slower or faster than nominal; the checker tests
both ends. A frame ends when the lines have been quiet for longer than the tPI maximum
allows, because that is the only thing a receiver can actually observe [AccessGrid,
05_...]: a panel accumulates falling edges and resets a timeout on each one. There is no
minimum gap between frames to check: any gap longer than the tPI maximum *is* the frame
separator, so a reader that never leaves such a gap runs its frames together and is
caught by the frame-length rule instead.

The 26-bit format is data, not code. `WIEGAND_FORMATS` holds one table per layout, and
which one a device uses is a keyword:

  "sia26"    the default. Pyramid's Table 2, quoted verbatim in [02_...]: bit 1 is even
             parity over bits 2..13, bits 2..9 are the facility code (bit 2 = MSB), bits
             10..25 are the ID number (bit 10 = MSB), bit 26 is odd parity over bits
             14..25. AccessGrid prints the same map [05_...].
  "h10301"   proxmark3's 26-bit reader: the 16 ID bits first, then the 8 facility bits,
             and the parity polarity swapped -- **odd** parity over the first 12 bits,
             **even** parity over the last 12 [06_...]. Same coverage, opposite polarity.
  "ind26"    proxmark3's other 26-bit layout: 12-bit facility code, 12-bit card number,
             even first, odd last [06_...].

A panel that disagrees about polarity is one keyword away rather than a new model, and
the tests prove all three encode, decode and parity-check identically.

Against the RTL. The nets are called "d0" and "d1" so `verif/vip/cocotb_bridge.py` can map
them onto any two of the chip's pins -- `{"d0": ("uio", 0), "d1": ("uio", 1)}` -- and then a
`WiegandChecker` on the far side of the bridge judges exactly what the DUT put on the wire,
at whatever tick rate the DUT runs at. Nothing here knows about Verilog.

Speeds. Wiegand is a microsecond-to-millisecond protocol, so at the core's default
50 MHz (20 ns/tick) one 26-bit frame at the reader's nominal timing is about 1.6 M
ticks, which is a lot of Python. The models take the tick rate from the bus, so a
self-test that wants to run many frames passes `ns_per_tick=1000` (1 MHz) and keeps the
same nanosecond timing; the one test that runs at 20 ns/tick checks that quantisation is
right rather than fast.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import floor
from typing import Optional, Sequence

from .core import Bus, Device


def ns_to_ticks(ns_per_tick: float, ns: float, up: bool = True) -> int:
    """Nanoseconds to whole ticks, rounded up for a floor and down for a ceiling.

    A minimum has to be rounded up, or the checker would accept a pulse one tick short of
    the floor it was given. A maximum has to be rounded down, or the checker would flag a
    pulse the specification still allows. Both directions are therefore at worst one
    tick away from the exact limit, and never on the lenient side.
    """
    if ns < 0:
        raise ValueError(f"a Wiegand time cannot be negative, got {ns!r} ns")
    return -int(-ns // ns_per_tick) if up else int(floor(ns / ns_per_tick))


@dataclass(frozen=True)
class WiegandTimings:
    """One timing class: the SIA window every pulse and every pause has to sit inside.

    `pw_min`/`pw_max` bound the low pulse on D0 or D1; `pi_min`/`pi_max` bound the quiet
    pause after it, up to the next pulse. The window is from the SIA's Access Control
    Standard Protocol for the 26-Bit Wiegand Reader Interface as quoted by Farpointe
    (research/ref/protocols/wiegand/01_farpointe_understanding_wiegand.md) and repeated
    by Pyramid (02_...). The reader's own typical values, Tpw 100 us and Tpi 1 ms, are in
    Pyramid's Table 1 (02_...); they are a nominal, not a limit, and the model does not
    use them as one.
    """

    name: str
    pw_min: float        # pulse width, floor
    pw_max: float        # pulse width, ceiling
    pi_min: float        # pulse interval (the pause between pulses), floor
    pi_max: float        # pulse interval, ceiling, and the frame separator


WIEGAND_PROFILES: dict[str, WiegandTimings] = {
    "sia26": WiegandTimings("sia26", 20_000, 100_000, 200_000, 20_000_000),
}

DEFAULT_PROFILE = "sia26"

# The pulse width and pulse interval the reader model sends unless told otherwise: a real
# reader's shipped defaults, BALTECH's `PulseWidth` 50 us and `PulseInterval` 2 ms
# (research/ref/protocols/wiegand/04_baltech_wiegand.md). Both sit strictly inside the SIA
# window above, so the model's nominal traffic is legal by two independent documents.
LEGAL_DEFAULTS_NS = (50_000, 2_000_000)


def spec_minimums(
    ns_per_tick: float, profile: str = DEFAULT_PROFILE, overrides: Optional[dict] = None
) -> dict:
    """Every limit of `profile` as whole ticks, plus the frame separator it implies.

    `overrides` replaces individual nanosecond values by their long names, which is how a
    test drives a reader or a checker off-spec on purpose: {"pw_min": 200_000} means
    "pulse widths must be at least 200 us". The four names are `pw_min`, `pw_max`,
    `pi_min` and `pi_max`.

    "frame_gap" is the quiet time that ends a frame, in ticks: one more than the pulse
    interval maximum, because a receiver cannot tell the end of a frame from a pause that
    is still inside the window. It is the timeout `WiegandReceiver` and `WiegandChecker`
    use, and the idle gap `WiegandReader` leaves between queued frames.
    """
    try:
        prof = WIEGAND_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f"unknown Wiegand profile {profile!r}; pick one of {sorted(WIEGAND_PROFILES)}"
        ) from None
    ns = {"pw_min": prof.pw_min, "pw_max": prof.pw_max,
          "pi_min": prof.pi_min, "pi_max": prof.pi_max}
    overrides = dict(overrides or {})
    for key, value in overrides.items():
        if key not in ns:
            raise ValueError(f"unknown timing override {key!r}; pick from {sorted(ns)}")
        ns[key] = float(value)
    if ns["pw_min"] > ns["pw_max"]:
        raise ValueError(f"pulse width floor {ns['pw_min']} is above its ceiling "
                         f"{ns['pw_max']}")
    if ns["pi_min"] > ns["pi_max"]:
        raise ValueError(f"pulse interval floor {ns['pi_min']} is above its ceiling "
                         f"{ns['pi_max']}")
    out = {key: ns_to_ticks(ns_per_tick, value, up=(key.endswith("_min")))
           for key, value in ns.items()}
    out["ns"] = ns
    out["profile"] = profile
    out["frame_gap"] = out["pi_max"] + 1
    return out


@dataclass(frozen=True)
class WiegandFormat:
    """One bit layout: where the parity bits, the facility code and the number sit.

    Every position is 1-based and counted the way the sources count them: position 1 is
    the first bit put on the wire. A field is a `(start, width)` pair laid down MSB
    first, which is what Pyramid's Table 2 says for the 26-bit format ("Bit 2 is MSB",
    "Bit 10 is MSB", 02_farpointe...). `lead_covers` and `trail_covers` are the
    1-based positions each parity bit is computed over, so a format that checks parity
    over some other range is still expressible.
    """

    name: str
    bits: int
    facility: tuple                      # (start, width), MSB first
    number: tuple                        # (start, width), MSB first
    lead: tuple                          # (position, "even"|"odd")
    trail: tuple
    lead_covers: tuple
    trail_covers: tuple

    def field_of(self, facility: int, number: int) -> list[int]:
        """The frame's bits with both parity bits filled in: `(facility, number)` -> bits."""
        if not 0 <= facility < (1 << self.facility[1]):
            raise ValueError(f"{self.name}: facility code {facility} does not fit in "
                             f"{self.facility[1]} bits")
        if not 0 <= number < (1 << self.number[1]):
            raise ValueError(f"{self.name}: ID number {number} does not fit in "
                             f"{self.number[1]} bits")
        bits: list[Optional[int]] = [None] * self.bits
        for (start, width), value in ((self.facility, facility), (self.number, number)):
            for k in range(width):
                bits[start - 1 + k] = (value >> (width - 1 - k)) & 1
        for (pos, kind), covers in ((self.lead, self.lead_covers),
                                    (self.trail, self.trail_covers)):
            covered = [bits[c - 1] for c in covers]
            if any(b is None for b in covered):
                raise ValueError(f"{self.name}: parity at bit {pos} covers a bit the "
                                 f"format does not define")
            bits[pos - 1] = parity_bit(covered, kind)
        if any(b is None for b in bits):
            raise ValueError(f"{self.name}: the layout leaves a bit undefined")
        return [int(b) for b in bits]


def parity_bit(bits: Sequence[int], kind: str) -> int:
    """The parity bit `kind` ("even" or "odd") over `bits`: the value that makes the
    total number of ones even or odd."""
    ones = sum(1 for b in bits if b)
    if kind == "even":
        return ones & 1
    if kind == "odd":
        return 1 - (ones & 1)
    raise ValueError(f"parity must be 'even' or 'odd', got {kind!r}")


# The three 26-bit layouts, each one table from a source. `sia26` is Pyramid's Table 2
# verbatim (research/ref/protocols/wiegand/02_pyramid_26bit_format.md), which AccessGrid
# prints identically (05_...); `h10301` and `ind26` are proxmark3's Pack_H10301 and
# Pack_ind26 read back into 1-based positions (06_...). All three cover the 24 data bits
# with one parity bit over each of two 12-bit halves, which is why the parity *coverage*
# is the same in all of them and only the polarity and the field order differ.
WIEGAND_FORMATS: dict[str, WiegandFormat] = {
    "sia26": WiegandFormat(
        name="sia26", bits=26,
        facility=(2, 8), number=(10, 16),
        lead=(1, "even"), trail=(26, "odd"),
        lead_covers=tuple(range(2, 14)), trail_covers=tuple(range(14, 26)),
    ),
    "h10301": WiegandFormat(
        name="h10301", bits=26,
        facility=(18, 8), number=(2, 16),
        lead=(1, "odd"), trail=(26, "even"),
        lead_covers=tuple(range(2, 14)), trail_covers=tuple(range(14, 26)),
    ),
    "ind26": WiegandFormat(
        name="ind26", bits=26,
        facility=(2, 12), number=(14, 12),
        lead=(1, "even"), trail=(26, "odd"),
        lead_covers=tuple(range(2, 14)), trail_covers=tuple(range(14, 26)),
    ),
}

DEFAULT_FORMAT = "sia26"


def wiegand_format(name: str = DEFAULT_FORMAT) -> WiegandFormat:
    """Look up a layout by name, or raise with the list of the ones there are."""
    try:
        return WIEGAND_FORMATS[name]
    except KeyError:
        raise ValueError(f"unknown Wiegand format {name!r}; pick one of "
                         f"{sorted(WIEGAND_FORMATS)}") from None


@dataclass(frozen=True)
class Card:
    """A decoded credential: what the facility code and ID number were, and whether the
    frame's two parity bits check out."""

    facility: int
    number: int
    lead_ok: bool          # the leading parity bit (even in "sia26", odd in "h10301")
    trail_ok: bool         # the trailing parity bit

    @property
    def parity_ok(self) -> bool:
        return self.lead_ok and self.trail_ok

    def __str__(self) -> str:
        return (f"FC {self.facility} card {self.number}"
                + ("" if self.parity_ok else " (parity wrong)"))


def encode_card(fmt: str = DEFAULT_FORMAT, facility: int = 0, number: int = 0) -> list[int]:
    """The bits a reader puts on the wire for `(facility, number)` in `format`."""
    return wiegand_format(fmt).field_of(facility, number)


def decode_card(fmt: str, bits: Sequence[int]) -> Optional[Card]:
    """Read `(facility, number)` and both parity results out of a frame's bits.

    `None` if the frame is not `format.bits` bits long: there is nothing to decode, and
    the caller (a receiver or the checker) reports the length itself rather than getting
    a plausible-looking answer from a truncated frame.
    """
    spec = wiegand_format(fmt)
    if len(bits) != spec.bits:
        return None

    def field_at(start: int, width: int) -> int:
        return sum(int(bits[start - 1 + k]) << (width - 1 - k) for k in range(width))

    lead_pos, lead_kind = spec.lead
    trail_pos, trail_kind = spec.trail
    return Card(
        facility=field_at(*spec.facility),
        number=field_at(*spec.number),
        lead_ok=int(bits[lead_pos - 1]) == parity_bit(
            [bits[c - 1] for c in spec.lead_covers], lead_kind),
        trail_ok=int(bits[trail_pos - 1]) == parity_bit(
            [bits[c - 1] for c in spec.trail_covers], trail_kind),
    )


@dataclass
class Violation:
    """One spec rule the bus did not meet, with the tick it was missed on.

    `measured` and `required` are in ticks for the four timing rules and in counts or
    bits for the framing ones (`BITS`, `D0D1`, `PARITY;*`); `detail` always says in words
    what the numbers mean, so a violation reads on its own.
    """

    tick: int
    rule: str            # "tPW;MIN", "tPW;MAX", "tPI;MIN", "tPI;MAX", "BITS", "D0D1",
                          # "PARITY;LEAD", "PARITY;TRAIL"
    measured: int
    required: int
    detail: str = ""

    def __str__(self) -> str:
        return (f"t={self.tick} {self.rule} {self.measured} vs {self.required}"
                + (f" ({self.detail})" if self.detail else ""))


@dataclass
class Pulse:
    """One low pulse as the bus saw it: which line, when it fell and rose, and how wide."""

    line: str            # "d0" or "d1"
    bit: int             # 0 for a D0 pulse, 1 for a D1 pulse
    fall: int            # tick the line fell
    rise: int            # tick the line rose again
    index: int = 0       # position in the frame, 0-based

    @property
    def width(self) -> int:
        return self.rise - self.fall

    def __str__(self) -> str:
        return f"t={self.fall} D{self.bit} {self.width} ticks"


@dataclass
class DecodedFrame:
    """One frame as a receiver or the checker read it off the wire."""

    bits: list = field(default_factory=list)
    pulses: list = field(default_factory=list)
    start: int = 0             # tick the first pulse fell
    end: int = 0               # tick the last pulse rose
    card: Optional[Card] = None

    @property
    def facility(self) -> int:
        return self.card.facility if self.card else -1

    @property
    def number(self) -> int:
        return self.card.number if self.card else -1

    def __str__(self) -> str:
        return (f"t={self.start} {len(self.bits)}-bit frame "
                + (str(self.card) if self.card else "undecodable"))


class WiegandReader(Device):
    """The card-reader model: it puts a queued frame on D0/D1 as low pulses.

    It drives nothing but 0s, so both lines stay pulled up between pulses
    [03_wikipedia_wiegand_interface.md]. Each queued bit becomes one pulse `pw` ns wide
    followed by a quiet pause of `pi` ns -- the pause definition is BALTECH's
    [04_baltech_wiegand.md] -- and `pw`/`pi` default to that reader's shipped defaults of
    50 us and 2 ms, which are inside the SIA window this file's checker enforces.

    Fault injection is per frame and per bit: `queue_bits(bits, widths=..., gaps=...)`
    takes nanosecond lists, so a test can shorten one pulse or stretch one pause without a
    second model. Overriding `timing` moves the *profile* the reader takes its limits from,
    which is how a test drives the whole reader off-spec.

    The reader never drives both lines at once -- its pulses are strictly sequential, so
    it cannot produce the one thing the specification forbids outright. `D0D1` comes from
    the bus, not from a reader: a second reader wired to the same pair in parallel, or a
    shorted pair held low by the installation. Both are one line of `Device.tick()` in a
    test; `verif/vip/test_wiegand.py` has `ShortedPair` for exactly that.

    `facility=` and `number=` queue one frame of that credential straight away, which is
    the common case; `queue_card()` queues more, and `queue_bits()` sends bits that no
    parity rule has ever blessed.

    Frames are separated by a full frame gap (the profile's pulse interval maximum plus a
    tick) even when several are queued, because a reader that does not leave such a gap
    runs its frames together and a panel cannot tell where one ends.
    """

    name = "wiegand_reader"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = DEFAULT_PROFILE,
        timing: Optional[dict] = None,
        bit_format: str = DEFAULT_FORMAT,
        pw: float = LEGAL_DEFAULTS_NS[0],
        pi: float = LEGAL_DEFAULTS_NS[1],
        d0_net: str = "d0",
        d1_net: str = "d1",
        facility: Optional[int] = None,
        number: Optional[int] = None,
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        self.bit_format = bit_format
        wiegand_format(bit_format)                  # reject an unknown layout early
        self.pw_ns = float(pw)
        self.pi_ns = float(pi)
        self.d0_net, self.d1_net = d0_net, d1_net
        if name:
            self.name = name
        self._recalc()
        self.queue: list = []
        self.sent: list = []
        self.errors: list = []      # stimulus mistakes, not bus faults: see queue_bits()
        self.cur = None
        self.index = 0
        self.phase = "idle"
        self.left = 1
        self.driving: Optional[str] = None
        self.trailing_idle = 0
        if facility is not None or number is not None:
            if facility is None or number is None:
                raise ValueError("give both facility and number, or neither")
            self.queue_card(facility, number)

    # ---- what the tests ask for --------------------------------------------------
    def bind(self, bus: Bus) -> "WiegandReader":
        """Take the tick rate from `bus` and redo every tick count."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._recalc()
        return self

    @property
    def idle(self) -> bool:
        """True once the queue is empty and the trailing frame gap has been put out."""
        return self.cur is None and not self.queue and self.trailing_idle <= 0

    @property
    def width_ticks(self) -> int:
        """The default pulse width in ticks."""
        return ns_to_ticks(self.ns_per_tick, self.pw_ns)

    @property
    def gap_ticks(self) -> int:
        """The default pause between pulses in ticks."""
        return ns_to_ticks(self.ns_per_tick, self.pi_ns)

    def report(self) -> str:
        return (f"{self.name}: {len(self.sent)} frames sent, {len(self.queue)} queued, "
                f"pw {self.width_ticks} ticks, gap {self.gap_ticks} ticks, "
                f"{self.profile} at {self.ns_per_tick} ns/tick")

    def queue_bits(
        self,
        bits: Sequence,
        widths: Optional[Sequence[float]] = None,
        gaps: Optional[Sequence[float]] = None,
    ) -> list:
        """Queue one frame: the bits, and optionally a nanosecond width and gap per bit.

        `bits` is a sequence of 0/1 or a string of "0"/"1". `widths` and `gaps` override
        the reader's `pw` and `pi` for individual pulses, which is the hook every
        off-spec test uses. A gap shorter than the pulse before it cannot happen: the
        pulses are sequential, so the smallest gap is 0 (the next pulse starts on the tick
        the previous one ends).
        """
        if isinstance(bits, str):
            bits = [int(c) for c in bits.strip()]
        bits = [int(b) for b in bits]
        for b in bits:
            if b not in (0, 1):
                raise ValueError(f"a Wiegand bit is 0 or 1, got {b!r}")
        for name, seq in (("widths", widths), ("gaps", gaps)):
            if seq is not None and len(seq) != len(bits):
                raise ValueError(f"{name} has {len(seq)} entries for {len(bits)} bits")
        if len(bits) != wiegand_format(self.bit_format).bits:
            # Not a wire fault, a stimulus mistake: the reader does not judge the bus, but a
            # test that queues 25 bits into a 26-bit reader should hear about it somewhere
            # other than a puzzling BITS violation.
            self.errors.append(f"queued {len(bits)} bits, the {self.bit_format} layout "
                               f"has {wiegand_format(self.bit_format).bits}")
        frame = []
        for i, bit in enumerate(bits):
            width = self.pw_ns if widths is None else float(widths[i])
            gap = self.pi_ns if gaps is None else float(gaps[i])
            frame.append((bit, ns_to_ticks(self.ns_per_tick, width),
                          ns_to_ticks(self.ns_per_tick, gap)))
        self.queue.append(frame)
        return frame

    def queue_card(
        self,
        facility: int,
        number: int,
        bit_format: Optional[str] = None,
        **kwargs,
    ) -> list:
        """Queue one frame carrying `(facility, number)` in `bit_format`."""
        bits = encode_card(bit_format or self.bit_format, facility, number)
        return self.queue_bits(bits, **kwargs)

    def run(self, bus: Optional[Bus] = None, limit: int = 2_000_000) -> bool:
        """Step `bus` until the queue is done and the trailing frame gap is out.

        False if `limit` ticks pass first. `bus` defaults to the one the reader was
        attached to; a different bus re-derives the tick counts from its tick rate.
        """
        if bus is None:
            bus = self.bus
        if bus is None:
            raise ValueError("this reader has no bus; pass one to run()")
        if bus is not self.bus:
            self.bind(bus)
        for _ in range(limit):
            if self.idle:
                return True
            bus.step()
        return self.idle

    # ---- the device --------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        if self.trailing_idle > 0:
            self.trailing_idle -= 1
            return {}
        if self.left <= 0:
            self._next_phase(t)
        out = {}
        if self.driving is not None:
            out[self.driving] = 0
        self.left -= 1
        return out

    def _recalc(self) -> None:
        self.min = spec_minimums(self.ns_per_tick, self.profile, self.timing_overrides)

    def _next_phase(self, t: int) -> None:
        if self.phase == "low":
            self.phase, self.driving = "gap", None
            self.left = self.cur[self.index][2]
            return
        if self.phase == "gap":
            self.index += 1
            if self.index < len(self.cur):
                self._start_pulse()
            else:
                self._end_frame(t)
            return
        if self.queue:                      # idle with something to send
            self.cur = self.queue.pop(0)
            self.index = 0
            self._start_pulse()

    def _start_pulse(self) -> None:
        bit, width, _gap = self.cur[self.index]
        self.driving = self.d1_net if bit else self.d0_net
        self.phase, self.left = "low", width

    def _end_frame(self, t: int) -> None:
        self.sent.append({"tick": t, "bits": [b for b, _w, _g in self.cur],
                          "width_ticks": self.width_ticks,
                          "gap_ticks": self.gap_ticks})
        self.cur, self.phase, self.driving = None, "idle", None
        self.trailing_idle = self.min["frame_gap"]


class WiegandReceiver(Device):
    """The door-controller side of the wire: a passive decoder, the reader's counterpart.

    It drives nothing. Every falling edge of D0 or D1 is one bit, its value is the line
    that fell [05_accessgrid_26bit_decode.md], and it times each pulse and each pause as
    it goes. A frame ends when the lines have been quiet for longer than the pulse
    interval maximum allows -- the only frame boundary a receiver can observe -- and the
    frame is then decoded: facility code, ID number and the two parity results, whatever
    they are. A frame with the wrong number of bits is still reported, with `card` left
    as `None`, because "the reader sent me 25 bits" is the interesting fact.

    `timeout_ns` overrides that frame boundary if you want to model a decoder with its own
    idea of it: the guide in `05_...` uses a 25 ms timer, which is why `timeout_ns` takes
    nanoseconds rather than only a profile. `bit_format` picks the layout it decodes with.
    Note that a decoder whose timeout is *longer* than the pause the reader leaves at the
    end of a frame never sees that frame at all: the frame closes when the quiet time
    reaches the boundary, so a run that stops with the reader already idle has to go on
    for the decoder's own timer before anything appears in `frames`.
    """

    name = "wiegand_rx"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = DEFAULT_PROFILE,
        timing: Optional[dict] = None,
        bit_format: str = DEFAULT_FORMAT,
        timeout_ns: Optional[float] = None,
        d0_net: str = "d0",
        d1_net: str = "d1",
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        self.bit_format = bit_format
        wiegand_format(bit_format)
        self.timeout_ns = float(timeout_ns) if timeout_ns else None
        self.d0_net, self.d1_net = d0_net, d1_net
        if name:
            self.name = name
        self._recalc()
        self.frames: list = []
        self.bits: list = []
        self.pulses: list = []
        self.collisions = 0              # ticks on which both lines were low at once
        self.errors: list = []
        self._reset_frame()

    # ---- what the tests ask for --------------------------------------------------
    def bind(self, bus: Bus) -> "WiegandReceiver":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._recalc()
        return self

    @property
    def clean(self) -> bool:
        """True when every frame decoded: right length and both parity bits right."""
        return not self.errors

    @property
    def cards(self) -> list:
        return [f.card for f in self.frames if f.card is not None]

    def frames_of(self, length: Optional[int] = None) -> list:
        return [f for f in self.frames if length is None or len(f.bits) == length]

    def report(self, limit: int = 10) -> str:
        want = wiegand_format(self.bit_format).bits
        head = (f"{self.bit_format} at {self.ns_per_tick} ns/tick: {len(self.frames)} frames "
                f"({len(self.frames_of(want))} of {want} bits), {len(self.cards)} decoded, "
                f"{self.collisions} ticks with both lines low")
        return head + "".join(f"\n  {e}" for e in self.errors[:limit])

    # ---- the device --------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        d0, d1 = pins[self.d0_net], pins[self.d1_net]
        if d0 == 0 and d1 == 0:
            self.collisions += 1
        for net, level, bit in ((self.d0_net, d0, 0), (self.d1_net, d1, 1)):
            was_low = self.low[bit] == 0
            if level == 0 and not was_low:                 # a falling edge: one bit
                self._fall(t, bit, net)
            elif level == 1 and was_low:                   # the pulse ended
                self._rise(t, bit, net)
            self.low[bit] = level
        if self.last_rise is not None and t - self.last_rise >= self.timeout_ticks:
            self._close_frame(t)
        return {}

    def _reset_frame(self) -> None:
        self.low = {0: 1, 1: 1}
        self.frame_bits: list = []
        self.frame_pulses: list = []
        self.open_pulses: dict = {}                       # bit -> Pulse being timed
        self.last_rise = None

    def _recalc(self) -> None:
        self.min = spec_minimums(self.ns_per_tick, self.profile, self.timing_overrides)
        # No explicit timeout: a frame ends when the quiet time exceeds the profile's own
        # pulse interval ceiling, so overriding `timing` moves the boundary too.
        self.timeout_ticks = (ns_to_ticks(self.ns_per_tick, self.timeout_ns)
                              if self.timeout_ns else self.min["pi_max"])

    def _fall(self, t: int, bit: int, net: str) -> None:
        if self.last_rise is None:
            self._close_frame(t)                         # nothing open yet: tidy up
        pulse = Pulse(line=net, bit=bit, fall=t, rise=t, index=len(self.frame_bits))
        self.frame_pulses.append(pulse)
        self.open_pulses[bit] = pulse

    def _rise(self, t: int, bit: int, net: str) -> None:
        pulse = self.open_pulses.pop(bit, None)
        if pulse is None:
            return
        pulse.rise = t
        self.frame_bits.append(pulse.bit)
        self.bits.append(pulse.bit)
        self.pulses.append(pulse)
        self.last_rise = t

    def _close_frame(self, t: int) -> None:
        """The lines have been quiet long enough: whatever arrived is one frame."""
        bits, pulses = self.frame_bits, self.frame_pulses
        if not bits:
            self._reset_frame()
            return
        start = pulses[0].fall if pulses else t
        card = decode_card(self.bit_format, bits)
        self.frames.append(DecodedFrame(bits=list(bits), pulses=list(pulses),
                                        start=start, end=self.last_rise or t, card=card))
        want = wiegand_format(self.bit_format).bits
        if len(bits) != want:
            self.errors.append(f"t={start} {len(bits)}-bit frame, expected {want}")
        elif not card.parity_ok:
            self.errors.append(f"t={start} parity wrong: lead {card.lead_ok}, "
                               f"trail {card.trail_ok}")
        self._reset_frame()


class WiegandChecker(Device):
    """Passive protocol checker: every timing or framing rule, with the tick it broke on.

    It drives nothing, so it sees the bus exactly as an analyser would: it measures the
    resolved pin values and adds no latency of its own for a device reacting a tick late.
    It measures you, not the devices.

    The rules, all against its profile and layout (`profile`, `bit_format`), with the
    sources in research/ref/protocols/wiegand/:

      tPW;MIN    a low pulse shorter than the pulse width floor (SIA 20 us)
      tPW;MAX    a low pulse longer than the pulse width ceiling (SIA 100 us). Flagged
                 while the line is still low, at the tick it went over, so a line stuck
                 low is caught even if it never rises again.
      tPI;MIN    a pause shorter than the pulse interval floor (SIA 200 us)
      tPI;MAX    a pause longer than the pulse interval ceiling (SIA 20 ms) while the
                 frame was still incomplete: the reader let the lines go quiet before it
                 had sent every bit, so the frame is cut short
      D0D1       both lines low on the same tick. Never legal: the pulses must not
                 overlap or occur simultaneously [01_..., 04_...], and on a real reader a
                 D0-and-D1-at-once means the pair is shorted
      BITS       a frame that is not `bit_format.bits` bits long
      PARITY;LEAD  the leading parity bit does not match the bits it covers
      PARITY;TRAIL the trailing parity bit does not match the bits it covers

    A frame that ends before it is complete is reported twice, as tPI;MAX *and* as BITS,
    and that is not double counting: the lines went quiet to the frame timeout with bits
    still missing, and a receiver cannot tell a reader that is too slow from a reader that
    sent too few bits -- they look identical on the wire. A frame that is too *long* has
    nothing missing, so it trips BITS alone.

    Both lines low at once is tracked per line, not per pair: a line dragged low for good
    has no rising edge to time, and the pulses meant for it have no falling edge to show,
    so each line's open pulse is kept separately and tPW;MAX fires while the line is still
    down.

    Every miss is appended to `violations` with the tick it happened on; `frames` holds
    everything it decoded, with each pulse's width, so a failure can be traced back to a
    bit index. Overriding `timing` re-points the four timing limits and `bit_format`
    re-points the layout, which is how a test judges legal traffic against stricter limits
    and so proves the checker is not vacuous.

    Two caveats, both making the checker stricter rather than more lenient. A pulse that
    began before the first tick cannot be timed, so nothing before tick 0 is measured; and
    a pulse that ends after the last tick is never seen to end, so a run that stops inside
    a pulse reports no width for it (its `tPW;MAX` still fires if it was too wide).
    """

    name = "wiegand_chk"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = DEFAULT_PROFILE,
        timing: Optional[dict] = None,
        bit_format: str = DEFAULT_FORMAT,
        d0_net: str = "d0",
        d1_net: str = "d1",
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        self.bit_format = bit_format
        self.format = wiegand_format(bit_format)
        self.d0_net, self.d1_net = d0_net, d1_net
        if name:
            self.name = name
        self._recalc()
        self.violations: list = []
        self.frames: list = []
        self.pulses: list = []
        self.bits: list = []
        self._reset_frame()
        self.prev = {0: 1, 1: 1}
        self.low_since = {0: None, 1: None}     # tick each line last fell, per line
        self.both_low = False
        self._min_pw = self._max_pw = self._min_pi = None   # over the whole run

    # ---- what the tests ask for --------------------------------------------------
    def bind(self, bus: Bus) -> "WiegandChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._recalc()
        return self

    @property
    def clean(self) -> bool:
        return not self.violations

    def violations_by_rule(self, rule: str) -> list:
        return [v for v in self.violations if v.rule == rule]

    @property
    def min_pw(self) -> Optional[int]:
        """The narrowest pulse seen, in ticks."""
        return self._min_pw

    @property
    def max_pw(self) -> Optional[int]:
        """The widest pulse seen, in ticks."""
        return self._max_pw

    @property
    def min_pi(self) -> Optional[int]:
        """The shortest pause seen, in ticks."""
        return self._min_pi

    def report(self, limit: int = 10) -> str:
        head = (f"{self.profile} at {self.ns_per_tick} ns/tick: {len(self.frames)} frames, "
                f"{len(self.pulses)} pulses, {len(self.violations)} violations")
        head += (f"; pulse width {self.min['pw_min']}-{self.min['pw_max']} ticks, "
                 f"pulse interval {self.min['pi_min']}-{self.min['pi_max']} ticks")
        if self._min_pw is not None:
            head += (f"; measured pw {self._min_pw}-{self._max_pw}, "
                     f"pi min {self._min_pi}")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    # ---- the monitor -------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        d0, d1 = pins[self.d0_net], pins[self.d1_net]
        self._both_low(t, d0, d1)
        # D0 first, then D1: on the tick a D0 pulse ends and a D1 pulse begins, the pause
        # between them is still measured correctly, and an overlap is caught by D0D1
        # above rather than being mistaken for one very long pulse.
        for bit, level, net in ((0, d0, self.d0_net), (1, d1, self.d1_net)):
            was_low = self.prev[bit] == 0
            if level == 0 and not was_low:
                self._fall(t, bit, net)
            elif level == 1 and was_low:
                self._rise(t, bit, net)
            self.prev[bit] = level
        # tPW;MAX while the line is still low, so a pulse the run never sees end -- a
        # line stuck low -- is caught at the tick it went over rather than never.
        for bit, since in self.low_since.items():
            if since is not None and bit not in self._pw_max_flagged \
                    and t - since > self.min["pw_max"]:
                self._over_wide(t, bit, t - since)
        if self.last_rise is not None and t - self.last_rise >= self.min["pi_max"]:
            self._close_frame(t)
        return {}

    def _reset_frame(self) -> None:
        self.frame_bits: list = []
        self.frame_pulses: list = []
        # One open pulse per line, not one for the pair: a line stuck low while the other
        # one pulses is exactly the case a single slot cannot describe.
        self.open_pulses: dict = {}
        self.last_rise = None
        self._pw_max_flagged: set = set()

    def _recalc(self) -> None:
        self.min = spec_minimums(self.ns_per_tick, self.profile, self.timing_overrides)

    def _violation(self, t: int, rule: str, measured: int, required: int,
                   detail: str = "", sense: str = "floor") -> None:
        """Record a violation if the measurement breaks the limit it is judged against.

        Wiegand has both kinds of limit, which is why this takes a `sense`:

          "floor"   measured must be at least `required` -- the two timing minimums
          "ceiling" measured must be at most `required` -- the two timing maxima
          "exact"   measured must equal `required` -- the frame length
          "check"   record it whenever `measured` is not zero, `required` being what a
                    correct bus would have shown -- D0D1 and the parity rules
          "reached" as "ceiling", but a measurement that only just reaches the ceiling
                    already counts. A frame closes on the first tick the quiet time reaches
                    the pulse interval maximum, so tPI;MAX is judged this way: reaching it
                    with bits still missing is the violation.
        """
        broken = {"floor": measured < required, "ceiling": measured > required,
                  "exact": measured != required, "check": measured != 0,
                  "reached": measured >= required}[sense]
        if broken:
            self.violations.append(Violation(t, rule, measured, required, detail))

    def _both_low(self, t: int, d0: int, d1: int) -> None:
        """Both lines low on the same tick: a shorted pair, or two pulses overlapping."""
        now = (d0 == 0 and d1 == 0)
        if now and not self.both_low:
            self._violation(t, "D0D1", 1, 0,
                            "D0 and D1 low at once; the pulses must not overlap",
                            sense="check")
        self.both_low = now

    def _fall(self, t: int, bit: int, net: str) -> None:
        gap = None if self.last_rise is None else t - self.last_rise
        if gap is not None:
            self._min_pi = gap if self._min_pi is None else min(self._min_pi, gap)
            self._violation(t, "tPI;MIN", gap, self.min["pi_min"],
                            f"pause before bit {len(self.frame_bits)}")
        pulse = Pulse(line=net, bit=bit, fall=t, rise=t, index=len(self.frame_bits))
        self.frame_pulses.append(pulse)
        self.open_pulses[bit] = pulse
        self.low_since[bit] = t
        self._pw_max_flagged.discard(bit)

    def _rise(self, t: int, bit: int, net: str) -> None:
        pulse = self.open_pulses.pop(bit, None)
        if pulse is None:
            return
        self.low_since[bit] = None
        pulse.rise = t
        width = pulse.width
        self._min_pw = width if self._min_pw is None else min(self._min_pw, width)
        self._max_pw = width if self._max_pw is None else max(self._max_pw, width)
        self._violation(t, "tPW;MIN", width, self.min["pw_min"],
                        f"D{bit} pulse, bit {pulse.index} of the frame",
                        sense="floor")
        if width > self.min["pw_max"] and bit not in self._pw_max_flagged:
            self._over_wide(t, bit, width)
        self.frame_bits.append(pulse.bit)
        self.bits.append(pulse.bit)
        self.pulses.append(pulse)
        self.last_rise = t

    def _over_wide(self, t: int, bit: int, width: int) -> None:
        """One tPW;MAX per pulse, whichever way it is noticed: on the rise, or -- for a
        pulse that is still low -- the first tick the width goes over the ceiling."""
        self._violation(t, "tPW;MAX", width, self.min["pw_max"],
                        f"D{bit} pulse held low", sense="ceiling")
        self._pw_max_flagged.add(bit)

    def _close_frame(self, t: int) -> None:
        """The lines have been quiet for longer than a pause may be: a frame is over.

        `t - self.last_rise` is at its smallest legal-to-observe value here, because a
        frame closes on the *first* tick the quiet time reaches the pulse interval
        ceiling. So an incomplete frame is exactly what tPI;MAX is about: the reader let
        the lines go quiet past the ceiling while bits were still missing.
        """
        bits = self.frame_bits
        if not bits:
            self._reset_frame()
            return
        start = self.frame_pulses[0].fall
        gap = t - self.last_rise
        card = None
        if len(bits) < self.format.bits:
            self._violation(t, "tPI;MAX", gap, self.min["pi_max"],
                            f"lines quiet for {gap} ticks after only {len(bits)} of "
                            f"{self.format.bits} bits", sense="reached")
        if len(bits) != self.format.bits:
            self._violation(t, "BITS", len(bits), self.format.bits,
                            f"frame starting at t={start}", sense="exact")
        else:
            card = decode_card(self.bit_format, bits)
            if card is not None and not card.lead_ok:
                self._violation(t, "PARITY;LEAD", 1, 0,
                                f"leading parity bit over "
                                f"{len(self.format.lead_covers)} bits is wrong",
                                sense="check")
            if card is not None and not card.trail_ok:
                self._violation(t, "PARITY;TRAIL", 1, 0,
                                f"trailing parity bit over "
                                f"{len(self.format.trail_covers)} bits is wrong",
                                sense="check")
        self.frames.append(DecodedFrame(bits=list(bits), pulses=list(self.frame_pulses),
                                        start=start, end=self.last_rise, card=card))
        self._reset_frame()


# ---- harness helpers -------------------------------------------------------------
def wiegand_bus(ns_per_tick: float = 20.0, trace: bool = False) -> Bus:
    """A bus with the two open-drain Wiegand nets, D0 and D1, both pulled high."""
    bus = Bus(ns_per_tick=ns_per_tick, trace=trace)
    bus.add_net("d0")
    bus.add_net("d1")
    return bus


def _ensure_nets(bus: Bus) -> None:
    """Add the two open-drain Wiegand nets if they are not there yet."""
    for net in ("d0", "d1"):
        if net not in bus.nets:
            bus.add_net(net)


def attach_reader(bus: Bus, **kwargs) -> WiegandReader:
    """Build a WiegandReader that takes its timing from `bus`, and attach it."""
    _ensure_nets(bus)
    rdr = WiegandReader(bus=bus, **kwargs)
    bus.attach(rdr)
    return rdr


def attach_receiver(bus: Bus, **kwargs) -> WiegandReceiver:
    """Build a passive WiegandReceiver for `bus`, and attach it."""
    _ensure_nets(bus)
    rcv = WiegandReceiver(bus=bus, **kwargs)
    bus.attach(rcv)
    return rcv


def attach_checker(bus: Bus, **kwargs) -> WiegandChecker:
    """Build a passive WiegandChecker for `bus`, and attach it."""
    _ensure_nets(bus)
    chk = WiegandChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk

"""Pin-level WS2812 / WS2812B reference models on the tick bus in verif/vip/core.py.

Three devices, all on one single-ended wire, all derived from the Worldsemi WS2812B
datasheet of 9-Apr-2013.  Every number below is quoted from that datasheet; the pages
it was read from, with URL and fetch date, are in `research/ref/protocols/ws2812/`,
and `ws2812_timings()` is the single place any of them live.

    Ws2812Tx            bit-level transmitter. Queues 24-bit words, one per pixel, and
                        drives each bit for the datasheet's high time (T0H or T1H) and
                        its low time (T0L or T1L), then holds the line low for the
                        reset code before the next frame.
    Ws2812Pixel         one 5050 pixel of the chain. Decodes the stream on its DIN and
                        latches its first 24 bits of every frame into a colour when the
                        reset code arrives, and holds its DOUT at the idle level for
                        exactly those bits before forwarding the rest of the frame, one
                        tick later. `attach_chain()` builds N of them with the DOUT of one
                        wired to the DIN of the next, and pixel `k` of an N-pixel chain
                        ends up with word `k`.
    Ws2812TimingChecker passive monitor. Decodes the bit stream and flags every rule
                        below, each with the tick it happened on.

The wire
--------
One signal, no clock, no handshake, no reply. Page 5: "Composition of 24bit data:
G7G6G5G4G3G2G1G0R7R6R5R4R3R2R1R0B7B6B5B4B3B2B1B0 ... Follow the order of GRB to sent
data and the high bit sent at first." So one pixel is 24 bits, green byte first, and
each byte goes out most significant bit first. Page 4 puts the chain in the other
order: "first 24 bit / second 24 bit / third 24 bit", and page 5 says which pixel sees
which part of the stream: "The data of D1 is send by MCU, and D2, D3, D4 through pixel
internal reshaping amplification to transmit." The first word on the wire belongs to
the pixel closest to the controller. Page 1 gives the rate: "Send data at speeds of
800Kbps", which is the same as page 3's caption "Data transfer time ( TH+TL=1.25us".

Timings, all from page 3, and all given as a nominal with a +/- window rather than as
a hard limit:

    T0H  0.4 us +/-150 ns     "0 code ,high voltage time"    a 0 is a SHORT high
    T1H  0.8 us +/-150 ns     "1 code ,high voltage time"    a 1 is a LONG high
    T0L  0.85 us +/-150 ns    "0 code , low voltage time"
    T1L  0.45 us +/-150 ns    "1 code ,low voltage time"
    TH+TL  1.25 us +/-600 ns  the caption: the bit period
    RES  above 50 us          "RES low voltage time Above 50us", and on the page 4
                              figure as the label "reset code >=50us"

Because the four are windows, the axis of high times splits into five cases and the
checker names all five: shorter than 0.25 us is too short to be a 0 (`T0H`), up to 0.55 us
is a 0, 0.55 to 0.65 us is inside neither window and decodes as nothing (`DECODE`), from
0.65 us up to 0.95 us is a 1, and past 0.95 us is too long to be a 1 (`T1H`). So a "0
that is too long" and a "1 that is too short" are the same fault -- the dead band -- and
are reported once, as `DECODE`, rather than twice. The low times only have ceilings: a 0's
low may run to 1.0 us (`T0L`) and a 1's to 0.6 us (`T1L`), with no floor at all, because a
short low is only ever a faster bit.

At 50 MHz, 20 ns per tick, the datasheet's numbers land on:

    T0H   20 ticks, window 13..27        T1H   40 ticks, window 33..47
    T0L   43 ticks, ceiling 50           T1L   23 ticks, ceiling 30
    bit period 63 ticks (1.26 us, 793 kbit/s), legal window 33..92
    reset  2500 ticks

`Ws2812Tx` sends exactly that, so its 0 bits are 20+43 ticks and its 1 bits 40+23 --
both 63. `tol=0` in a timing override turns the windows into the nominal values only,
which is the strictest thing the datasheet's own numbers can be read as.

The rules `Ws2812TimingChecker` enforces, and what breaks each of them
--------------------------------------------------------------------
    T0H     a high time under T0H's floor, so short to be a 0
    T1H     a high time over T1H's ceiling, so long to be a 1
    T0L     a bit decoded 0 whose low time is longer than T0L's ceiling
    T1L     a bit decoded 1 whose low time is longer than T1L's ceiling
    DECODE  a high time between T0H's ceiling and T1H's floor, inside neither window,
            which is where "a 0 that is too long" and "a 1 that is too short" land
    TBIT    a bit period outside TH+TL's window. This is the only rule that catches a
            transmitter that is simply too fast: a 0 bit with a 0.4 us high and a
            0.2 us low meets every floor and ceiling above and is 600 ns long.
    RESET   the line went high again before the reset code, so the frame it ended was
            never latched by a strip
    FRAME   a reset ended a frame whose bit count is not a multiple of 24

Tick conventions, both from core.py's docstring
-----------------------------------------------
* A driver that returns a value for net N at tick t has the net read that value from
  the end of tick t on. `Ws2812Tx` advances its state and then drives, so the number of
  ticks it returns a level for is exactly the number of ticks the wire shows it: its log
  and the checker's measurements agree without any correction. A pixel's DOUT is one tick
  behind its DIN, which is the core's sampling latency, and it makes pixel `k` latch one
  tick after pixel `k - 1`.
* A net nobody has driven yet reads its pull-up, so every receiver sees a high at tick 0.
  The transmitter idles one tick before its first frame and the checker ignores a first
  high pulse that is too short to be a bit, which is what that pull-up looks like; without
  both, the first high of a run would measure one tick long.
* The checker drives nothing, so it sees the bus as an analyser would and measures
  resolved pin values: it measures you, not the devices.

Typical use
-----------
    bus = Bus(ns_per_tick=20)                          # 50 MHz
    tx = attach_transmitter(bus)                       # adds "din", push-pull
    pix = attach_chain(bus, count=8)                   # 8 pixels, DIN of one to DOUT of the next
    chk = attach_checker(bus)                          # optional, passive

    tx.send(0xFF8000, (0, 255, 0))                     # two pixels: orange, green
    tx.run(bus, limit=200_000)
    assert chk.clean, chk.report()
    assert pix[0].color == (255, 128, 0) and pix[1].color == (0, 255, 0)
    assert bus.errors == []                            # push-pull: no contention

    # Or let the chain drain itself, which is what a run of more than one pixel wants:
    # pixel k latches one tick per link after the wire does, so waiting for the
    # transmitter is not the same as waiting for the strip.
    tx.send((1, 2, 3))
    assert wait_latched(bus, pix, 1, limit=200_000)
    assert chk.finalize() == [], "nothing was left open on the wire"
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, floor
from typing import Iterable, Optional, Sequence, Union

from .core import Bus, Device

Color = Union[int, Sequence[int]]

BITS_PER_PIXEL = 24            # "Composition of 24bit data", page 5


@dataclass(frozen=True)
class Ws2812Profile:
    """One datasheet revision's numbers, all in nanoseconds.

    The four bit times are nominal values with a `+/- tol` window, which is how the
    datasheet gives them ("0.4us +/-150ns"), so they are kept as a nominal plus one
    shared tolerance rather than as four limits. `t_bit` is page 3's caption
    ("Data transfer time ( TH+TL=1.25us +/-600ns)") and `bit_rate` is page 1's
    "Send data at speeds of 800Kbps", which is 1/t_bit to the precision either is
    worth. `t_reset` is the datasheet's minimum low time for the reset code.
    """

    name: str
    t0h: float          # short high: a 0 bit
    t1h: float          # long high: a 1 bit
    t0l: float          # low time after a 0
    t1l: float          # low time after a 1
    tol: float          # +/- on each of the four above
    t_bit: float        # nominal bit period, T0H+T0L and T1H+T1L both
    t_bit_tol: float    # +/- on the bit period
    t_reset: float      # minimum low time of the reset code
    bit_rate: float     # the rate the feature list quotes, in bit/s


WS2812_PROFILES: dict[str, Ws2812Profile] = {
    # Worldsemi CO., LIMITED, "WS2812B Intelligent control LED integrated light
    # source", sheet of file dated 9-Apr-2013.  research/ref/protocols/ws2812/.
    "ws2812b-2013": Ws2812Profile(
        name="ws2812b-2013",
        t0h=400.0, t1h=800.0, t0l=850.0, t1l=450.0, tol=150.0,
        t_bit=1_250.0, t_bit_tol=600.0, t_reset=50_000.0, bit_rate=800e3,
    ),
}

_OVERRIDES = ("t0h", "t1h", "t0l", "t1l", "tol", "t_bit", "t_bit_tol", "t_reset")


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, ns) / ns_per_tick))


def ns_to_ticks_down(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded down.

    This is the rounding a *ceiling* wants: a duration that may not exceed `ns` may
    span at most floor(ns / ns_per_tick) ticks. Rounding a ceiling up would let the
    checker pass a duration the datasheet does not allow.
    """
    return max(0, int(floor(max(0.0, ns) / ns_per_tick)))


def ns_to_ticks_nearest(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to the nearest whole tick, halves up.

    What `Ws2812Tx` uses for a nominal time: the tick rate cannot express 0.85 us at
    20 ns per tick, and the transmitter wants the closest legal value, not the ceiling.
    """
    return int(floor(max(0.0, ns) / ns_per_tick + 0.5))


def ws2812_timings(
    ns_per_tick: float, profile: str = "ws2812b-2013", overrides: Optional[dict] = None
) -> dict:
    """Every rule limit as whole ticks, plus the nanosecond values they came from.

    Floors are rounded up (`ns_to_ticks`) and ceilings down (`ns_to_ticks_down`), so
    tick quantisation only ever makes the checker a hair stricter than the datasheet,
    never more lenient: at 20 ns per tick T0H's 250 ns floor is 13 ticks (260 ns) and
    T0L's 1000 ns ceiling is 50 ticks (1000 ns).

    `overrides` replaces individual nanosecond values by their long names, which is
    how a test drives one part of a bench off-spec on purpose: {"t0l": 1400} means "a
    0 bit's low time may last 1400 ns", {"tol": 0} means "judge against the nominal
    values alone", {"t_reset": 80_000} points the checker at a datasheet revision
    with a longer reset code. An override is taken as written and is never clamped
    into a legal window -- a transmitter built with one is broken on purpose, and
    `Ws2812Tx` says so in `tx.errors` rather than quietly fixing it.

    Keys: t0h_min, t0h_max, t1h_min, t1h_max (the two high-time windows), t0l_max,
    t1l_max (the low-time ceilings), t_reset_min, t_bit, t_bit_min, t_bit_max, plus
    the profile name, the tick rate and every nanosecond value behind them.
    """
    try:
        prof = WS2812_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f"unknown WS2812 profile {profile!r}; pick one of {sorted(WS2812_PROFILES)}"
        ) from None
    ns = {"t0h": prof.t0h, "t1h": prof.t1h, "t0l": prof.t0l, "t1l": prof.t1l,
          "tol": prof.tol, "t_bit": prof.t_bit, "t_bit_tol": prof.t_bit_tol,
          "t_reset": prof.t_reset}
    overrides = dict(overrides or {})
    for key, value in overrides.items():
        if key not in _OVERRIDES:
            raise ValueError(
                f"unknown timing override {key!r}; pick from {sorted(_OVERRIDES)}"
            )
        ns[key] = float(value)
    tol, bit_tol = ns["tol"], ns["t_bit_tol"]
    out = {
        "profile": profile,
        "ns_per_tick": ns_per_tick,
        # A high time may not be shorter than nominal - tol, and not longer than
        # nominal + tol; the two windows are what the receiver reads a bit with.
        "t0h_min": ns_to_ticks(ns_per_tick, ns["t0h"] - tol),
        "t0h_max": ns_to_ticks_down(ns_per_tick, ns["t0h"] + tol),
        "t1h_min": ns_to_ticks(ns_per_tick, ns["t1h"] - tol),
        "t1h_max": ns_to_ticks_down(ns_per_tick, ns["t1h"] + tol),
        # A low time only has a ceiling in the datasheet.
        "t0l_max": ns_to_ticks_down(ns_per_tick, ns["t0l"] + tol),
        "t1l_max": ns_to_ticks_down(ns_per_tick, ns["t1l"] + tol),
        "t_reset_min": ns_to_ticks(ns_per_tick, ns["t_reset"]),
        "t_bit": ns_to_ticks_nearest(ns_per_tick, ns["t_bit"]),
        "t_bit_min": ns_to_ticks(ns_per_tick, ns["t_bit"] - bit_tol),
        "t_bit_max": ns_to_ticks_down(ns_per_tick, ns["t_bit"] + bit_tol),
    }
    out.update({f"{key}_ns": value for key, value in ns.items()})
    out["bit_rate"] = prof.bit_rate
    return out


def classify_bit(high_ticks: int, min_t: dict) -> Optional[int]:
    """The bit a high time of `high_ticks` ticks decodes to: 0, 1, or None.

    None means the datasheet's two windows do not cover it, which is the `DECODE` rule
    and not a bit value: a receiver has no way to know which of the two was meant.
    Both the pixels and the checker call this, so a bit is decoded the same way
    everywhere in this module.
    """
    if high_ticks <= min_t["t0h_max"]:
        return 0
    if high_ticks >= min_t["t1h_min"]:
        return 1
    return None


# ---- colour and word packing -----------------------------------------------------
def grb_word(rgb: Color) -> int:
    """A pixel as the datasheet's 24-bit word: G in 23:16, R in 15:8, B in 7:0.

    Page 5 spells the word out as "G7G6G5G4G3G2G1G0R7R6R5R4R3R2R1R0B7B6B5B4B3B2B1B0",
    which is this packing, and page 3's VDD-level input is 5 V, so a word is masked to
    24 bits rather than range-checked.
    """
    if isinstance(rgb, int):
        red, green, blue = (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF
    else:
        red, green, blue = (int(c) & 0xFF for c in rgb)
    return ((green & 0xFF) << 16) | ((red & 0xFF) << 8) | (blue & 0xFF)


def rgb_from_word(word: int) -> tuple[int, int, int]:
    """A datasheet word back to (r, g, b), the inverse of `grb_word`."""
    return ((word >> 8) & 0xFF, (word >> 16) & 0xFF, word & 0xFF)


def word_bits(word: int) -> list[int]:
    """The 24 bits of one word on the wire, MSB first: G7 ... G0 R7 ... R0 B7 ... B0."""
    return [(word >> (BITS_PER_PIXEL - 1 - i)) & 1 for i in range(BITS_PER_PIXEL)]


def bits_to_word(bits: Sequence[int]) -> int:
    """The 24 bits a receiver shifts in, MSB first, back into a datasheet word."""
    word = 0
    for bit in bits[:BITS_PER_PIXEL]:
        word = (word << 1) | (1 if bit else 0)
    return word


@dataclass
class TxFrame:
    """One frame as the transmitter drove it: 24 * `pixels` bits and then a reset.

    The tick fields are *drive* ticks, the tick the transmitter returned the level for,
    which is how `verif/vip/uart.py` logs its frames too. A receiver sees each of them
    one tick later, and the n-th pixel of a chain one tick after that.
    """

    start_tick: int = -1          # first bit
    end_tick: int = -1            # end of the last bit's low time: the reset begins
    latch_tick: Optional[int] = None   # the tick the line leaves the reset, i.e. when a
                                      # receiver may latch: the idle gap's first tick, or
                                      # the next frame's first bit, whichever is first
    reset_ticks: int = 0          # ticks the line was held low for
    bits: int = 0                 # bits sent in this frame
    pixels: int = 0               # whole words sent in this frame
    errors: list[str] = field(default_factory=list)   # rules this frame knowingly breaks

    @property
    def ok(self) -> bool:
        return not self.errors


class Ws2812Tx(Device):
    """Bit-level transmitter for the WS2812 wire: queued words in, timed levels out.

    `send()` takes pixels, `send_word()` the datasheet's 24-bit word directly and
    `send_bits()` raw bits, which is the hook for deliberately broken traffic. Each bit
    is driven for the profile's high time and then for its low time, and every frame
    ends with the line held low for the reset code, so a receiver latches it.

    The nominal times are quantised to the tick rate and then clamped into the
    datasheet's own windows (`_fit`), so a legal profile produces a legal wire at any
    tick rate that can express one. A tick rate that cannot express a legal T0H raises
    instead of quietly sending something no receiver can read. A `timing` override
    bypasses that clamp on purpose -- see `ws2812_timings()` -- and anything the
    override makes illegal is reported in `errors` and in each frame's own `errors`,
    so a test never has to infer it from a violation list somewhere else.

    The line idles at `idle_level`, which defaults to 0: a low idle *is* the reset
    code, so a strip is already latched and ready when the first frame arrives, and a
    transmitter that releases the pin instead (`idle_level=1`, what a PIO program with a
    pull-up on its data pin does) is a legal thing to model too -- the datasheet fixes
    no idle level, and the checker's RESET rule is what tells the two apart.
    """

    name = "ws2812_tx"

    def __init__(
        self,
        net: str = "din",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = "ws2812b-2013",
        timing: Optional[dict] = None,
        idle_level: int = 0,
        idle_ticks: int = 0,
        name: Optional[str] = None,
    ):
        self.net = net
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        if idle_level not in (0, 1):
            raise ValueError(f"idle_level must be 0 or 1, got {idle_level!r}")
        if idle_ticks < 0:
            raise ValueError(f"idle_ticks must be >= 0, got {idle_ticks!r}")
        self.idle_level = idle_level
        self.idle_ticks = int(idle_ticks)
        if name:
            self.name = name
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        self.min = ws2812_timings(self.ns_per_tick, self.profile, self.timing_overrides)
        self.high = {
            0: self._fit("t0h", self.min["t0h_min"], self.min["t0h_max"]),
            1: self._fit("t1h", self.min["t1h_min"], self.min["t1h_max"]),
        }
        # A low time has no floor in the datasheet, so the nominal is all there is --
        # clamped to its ceiling, though, which a tight profile can put below the nominal
        # (a nominal-only T0L of 0.85 us is 42 ticks, not 43).
        self.low = {
            0: min(ns_to_ticks_nearest(self.ns_per_tick, self.min["t0l_ns"]),
                   self.min["t0l_max"]),
            1: min(ns_to_ticks_nearest(self.ns_per_tick, self.min["t1l_ns"]),
                   self.min["t1l_max"]),
        }
        self.reset = ns_to_ticks(self.ns_per_tick, self.min["t_reset_ns"])
        self.errors: list[str] = []
        if self.timing_overrides:
            # Deliberate: an override is a request for a specific, possibly illegal,
            # wire. Say what it will break now rather than leaving it to be inferred.
            # `errors` is judged against the *profile*, not against the overrides: an
            # override says what to drive, the profile says what the datasheet allows,
            # so `{"t0h": 200}` reports the T0H floor it is about to miss.
            self.spec = ws2812_timings(self.ns_per_tick, self.profile)
            for rule in self._broken_rules():
                self.errors.append(f"{self.name}: timing override breaks {rule}")

    def _fit(self, name: str, lo: int, hi: int) -> int:
        """Nearest legal tick count to the nominal for `name`, clamped into [lo, hi]."""
        want = ns_to_ticks_nearest(self.ns_per_tick, self.min[f"{name}_ns"])
        if lo > hi:
            raise ValueError(
                f"{self.name}: a {self.ns_per_tick} ns tick cannot express a legal "
                f"{name}: the datasheet's window is {lo}..{hi} ticks wide here"
            )
        return max(lo, min(hi, want))

    def _broken_rules(self) -> list[str]:
        """Which rules of the *profile* the current limits break, checked as the
        checker checks them. See `_compute_timings`: the judgements use `self.spec`,
        the un-overridden limits of the profile."""
        m = self.spec
        rules = []
        for bit, hname, lname in ((0, "t0h", "t0l"), (1, "t1h", "t1l")):
            high, low = self.high[bit], self.low[bit]      # what will be driven
            if high < m[f"{hname}_min"]:                   # a 0's floor: a 1's is covered
                rules.append(f"{hname.upper()}: {high} ticks < {m[f'{hname}_min']}")
            if classify_bit(high, m) is None:
                rules.append(f"DECODE: a {hname.upper()} of {high} ticks is neither a 0 nor a 1")
            if low > m[f"{lname}_max"]:
                rules.append(f"{lname.upper()}: {low} ticks > {m[f'{lname}_max']}")
            if not m["t_bit_min"] <= high + low <= m["t_bit_max"]:
                rules.append(f"TBIT: {high + low} ticks is outside "
                             f"{m['t_bit_min']}..{m['t_bit_max']}")
        if self.reset < m["t_reset_min"]:
            rules.append(f"RESET: {self.reset} ticks < {m['t_reset_min']}")
        return rules

    def bind(self, bus: Bus) -> "Ws2812Tx":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        self._reset()
        return self

    def _reset(self) -> None:
        self.queue: list[int] = []      # bits waiting, one entry per bit
        self.frames: list[TxFrame] = []
        self.errors: list[str] = [e for e in getattr(self, "errors", []) if "timing" in e]
        self.phase = "idle"              # idle | high | low | reset
        self.level = self.idle_level    # what to drive this tick
        self.left = 1                    # ticks left on this level, including this one
        self.bit = 0                     # bits sent in this frame
        self.bit_now = 0                 # the bit being driven
        self.frame: Optional[TxFrame] = None
        self._latched: Optional[TxFrame] = None    # frame waiting for the next bit
        self.started = False                       # has any frame begun yet

    # ---- the queue ---------------------------------------------------------------
    def send(self, *pixels: Color) -> "Ws2812Tx":
        """Queue pixels: send(0xFF8000), send((0, 255, 0)), send((r, g, b), ...).

        A pixel is 0xRRGGBB or an (r, g, b) triple, in either case the host's order;
        the datasheet's GRB packing is this module's business, not the caller's.
        """
        for pixel in pixels:
            self.queue += word_bits(grb_word(pixel))
        return self

    def send_word(self, *words: int) -> "Ws2812Tx":
        """Queue the datasheet's 24-bit words as they are, G in bits 23:16."""
        for word in words:
            self.queue += word_bits(word)
        return self

    def send_bits(self, bits: Iterable[int]) -> "Ws2812Tx":
        """Queue raw bits, which is how a test breaks the frame length or the order."""
        self.queue += [1 if b else 0 for b in bits]
        return self

    @property
    def pending_bits(self) -> int:
        """Bits queued but not started yet."""
        return len(self.queue)

    @property
    def busy(self) -> bool:
        return self.phase != "idle" or bool(self.queue)

    @property
    def idle(self) -> bool:
        return not self.busy

    def run(self, bus: Bus, limit: int = 500_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out first."""
        return bus.run_until(lambda _b: self.idle, limit)

    # ---- drives ------------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        # Advance first, then drive: the level returned for this tick is the one this
        # tick puts on the wire, so the number of ticks a level is returned for is
        # exactly the number of ticks the wire shows it. Driving first would spend a
        # tick on the idle level before every frame and put a one-tick glitch on the
        # wire at the start of each frame.
        self._advance(t)
        return {self.net: self.level}

    # ---- the state machine --------------------------------------------------------
    def _advance(self, t: int) -> None:
        if self.phase == "idle":
            self._start_frame(t)
        elif self.phase == "gap":                # the idle level, before a queued frame
            self.left -= 1
            if self.left <= 0:
                self._start_frame(t)
        elif self.phase == "high":
            self.left -= 1
            if self.left <= 0:                    # the high time is over: go low
                self.level, self.left = 0, max(1, self.low[self.bit_now])
                self.phase = "low"
        elif self.phase == "low":
            self.left -= 1
            if self.left <= 0:                    # this bit is on the wire
                self.queue.pop(0)
                self.bit += 1
                if self.queue:
                    self._start_bit()
                else:
                    self.level, self.left = 0, max(1, self.reset)
                    self.phase = "reset"
        elif self.phase == "reset":
            self.left -= 1
            if self.left <= 0:
                self._end_frame(t)
        else:
            raise RuntimeError(f"{self.name}: bad phase {self.phase!r}")

    def _start_frame(self, t: int) -> None:
        """Begin a frame at tick `t`, or sit at the idle level if nothing is queued."""
        if not self.started:
            # One tick of idle before the very first frame, whatever is queued. The bus
            # shows its pull-up on a net nothing has driven yet, so a frame that began on
            # tick 0 has its first high measured one tick long by every receiver on it.
            self.started, self.left = True, 2
            self.level, self.phase = self.idle_level, "idle"
            return
        if not self.queue:
            self.level, self.left = self.idle_level, 1
            self.phase = "idle"
            return
        if self._latched is not None:             # the previous reset ended at this tick
            self._latched.latch_tick = t
            self._latched = None
        self.frame = TxFrame(start_tick=t, errors=list(self.errors))
        self.bit = 0
        self._start_bit()

    def _start_bit(self) -> None:
        """Drive the bit at the head of the queue for its high time."""
        self.bit_now = self.queue[0]
        self.level, self.left = 1, max(1, self.high[self.bit_now])
        self.phase = "high"

    def _end_frame(self, t: int) -> None:
        """The reset code is over: the frame is latched. Wait for the next one, or for
        the idle gap in front of it if one is queued."""
        frame = self.frame or TxFrame(errors=list(self.errors))
        frame.end_tick = t
        frame.reset_ticks = max(1, self.reset)
        frame.bits = self.bit
        frame.pixels = self.bit // BITS_PER_PIXEL
        if self.bit % BITS_PER_PIXEL:
            frame.errors.append(
                f"FRAME: {self.bit} bits is not a multiple of {BITS_PER_PIXEL}"
            )
        self.frames.append(frame)
        self._latched = frame                  # its reset ends when the line leaves it
        self.frame = None
        self.bit, self.bit_now = 0, 0
        if self.queue and self.idle_ticks:
            frame.latch_tick = t                # the line leaves the reset on this tick
            self.level, self.left = self.idle_level, self.idle_ticks
            self.phase = "gap"
        else:
            self.level, self.left = self.idle_level, 1
            self.phase = "idle"


@dataclass
class LatchEvent:
    """One colour a pixel latched, with the tick the reset code ended on.

    The tick is in observation time: the pixel latches on the tick it *sees* the line
    come back high, which is one tick after the transmitter drove that edge and one
    more for every pixel ahead of this one in the chain.
    """

    tick: int
    rgb: tuple[int, int, int]
    word: int                    # the datasheet's 24-bit word, G in bits 23:16
    bits: int = 0                # bits the frame carried in total


class Ws2812Pixel(Device):
    """One 5050 pixel of the chain: a forwarding tap and a 24-bit receiver.

    Two jobs, and the datasheet gives both.

    It receives: it measures each high pulse, reads a 0 or a 1 out of it with
    `classify_bit`, and shifts those bits in -- the first `BITS_PER_PIXEL` of a frame are
    its own, and when the line stays low for at least the reset code the word is latched
    into `color` and `latch_log`.

    And it forwards, taking its own word off the front of the frame while it does. Page
    4 lays a refresh cycle out as "first 24 bit / second 24 bit / third 24 bit" and page
    5 says which node carries which: "The data of D1 is send by MCU, and D2, D3, D4
    through pixel internal reshaping amplification to transmit." So the second pixel sees
    the *second* word. A pure tap cannot do that -- every pixel would latch word 0 --
    which is why `delay_bits` exists: DOUT is held at the idle level until this pixel has
    taken `delay_bits` bits of the frame, and carries the rest of it from then on, one
    tick later. Pixel `k` of a chain therefore sees `24 * (n - k)` bits of an `n`-pixel
    frame and latches the word meant for it.

    The part's internal pipeline is not described anywhere in the datasheet, so this is
    a model of the datasheet's *behaviour*, not of its gates: what is modelled is which
    word lands on which pixel, and what that does to the wire -- the bit count shrinks by
    one word per link and each pixel latches one tick after the one in front of it. Two
    knobs let a test see what that is worth: `delay_bits=0` turns the pixel into a plain
    tap (every pixel latches word 0) and `delay_bits=k` makes it take a different number
    of bits. `swallow_ticks` is the width of the idle region DOUT held, measured on the
    wire, and `glitches` counts high pulses too short to be a bit.

    A high pulse shorter than the datasheet's T0H floor is a glitch and is not shifted
    in: that is what the floor is for, and it is also how the one-tick pull-up every net
    shows before anything drives it gets thrown away. A reset with fewer than 24 bits
    behind it latches nothing and the pixel keeps its last colour: a strip that is reset
    before any data arrives shows black and then stays whatever it was last told.
    """

    def __init__(
        self,
        din_net: str = "din",
        dout_net: str = "dout0",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = "ws2812b-2013",
        timing: Optional[dict] = None,
        delay_bits: int = BITS_PER_PIXEL,
        name: Optional[str] = None,
    ):
        self.din_net, self.dout_net = din_net, dout_net
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        if delay_bits < 0:
            raise ValueError(f"delay_bits must be >= 0, got {delay_bits!r}")
        self.delay_bits = int(delay_bits)
        self.name = name or f"ws2812_pix[{dout_net}]"
        self._compute_timings()
        self._reset()

    def _compute_timings(self) -> None:
        self.min = ws2812_timings(self.ns_per_tick, self.profile, self.timing_overrides)

    def bind(self, bus: Bus) -> "Ws2812Pixel":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        return self

    def _reset(self) -> None:
        self.level: Optional[int] = None    # last input level; None until the first tick
        self.level_since = 0
        self.pending: Optional[int] = None  # bit whose high pulse has ended
        self.shift = 0                      # this pixel's word, MSB first
        self.nbits = 0                      # bits seen since the last reset
        self.color: tuple[int, int, int] = (0, 0, 0)
        self.bits: list[int] = []           # every bit decoded, in order
        self.bad_bits = 0                   # high times that decode as neither 0 nor 1
        self.glitches = 0                   # high pulses too short to be a bit at all
        self.short_resets = 0               # resets with fewer than 24 bits behind them
        self.frames = 0                     # colours latched
        self.latch_log: list[LatchEvent] = []
        # DOUT is idle while this pixel takes its word; `delay_bits=0` is a plain tap.
        self.swallowing = self.delay_bits > 0
        self.frame_start = 0
        self.swallow_ticks: Optional[int] = None    # width of the region DOUT held idle

    @property
    def word(self) -> int:
        """This pixel's last latched colour as a datasheet word."""
        return grb_word(self.color)

    # ---- the device --------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        level = pins[self.din_net]
        # DOUT is the input one tick later, except while this pixel is taking its own
        # `delay_bits` bits off the front of the frame: those go to this pixel and the
        # rest are the next pixel's. Holding DOUT at the idle level for that width is
        # what turns one stream into a chain -- page 4's "first 24 bit / second 24 bit /
        # third 24 bit" and page 5's "D1 is send by MCU, and D2, D3, D4 through pixel
        # internal reshaping amplification to transmit".
        self.dout_level = 0 if self.swallowing else level
        self._advance(t, level)
        return {self.dout_net: self.dout_level}

    def _advance(self, t: int, level: int) -> None:
        if self.level is None:                          # first tick: no edge to read
            self.level, self.level_since = level, t
            return
        if level == self.level:
            # A low that has lasted long enough latches the frame on its own: a strip
            # does not wait for the line to come back up, and a transmitter that idles
            # low never brings it back up.
            if level == 0 and self.nbits and t - self.level_since >= self.min["t_reset_min"]:
                self._latch(self.level_since + self.min["t_reset_min"])
            return
        if level == 0:                                  # the high pulse just ended
            high = t - self.level_since
            if high < self.min["t0h_min"]:
                # Too short to be any bit: the datasheet's T0H is a floor, and a pulse
                # under it is a glitch. The first level a pixel sees is the pull-up on a
                # net nobody has driven yet, and that is the pulse this throws away.
                self.glitches += 1
                self.pending = None
            else:
                self.pending = classify_bit(high, self.min)
                if self.pending is None:
                    self.bad_bits += 1
        else:                                            # the low period just ended
            self._commit(t, t - self.level_since)
        self.level, self.level_since = level, t

    def _commit(self, t: int, low_ticks: int) -> None:
        """A rising edge: either the reset code ends here, or a bit is complete."""
        if low_ticks >= self.min["t_reset_min"]:
            self._latch(t)
            return
        if self.pending is None:
            return                                      # a rise with no high pulse: ignore
        self._shift_in(self.pending)
        if self.nbits == 0:                             # the first bit of a frame
            self.frame_start = t
        if self.nbits >= self.delay_bits and self.swallowing:
            # This pixel has its word: from the next bit on, DOUT carries the rest.
            self.swallowing = False
            self.swallow_ticks = t - self.frame_start

    def _shift_in(self, value: int) -> None:
        """Take one decoded bit: only the first 24 of a frame are this pixel's own."""
        if self.nbits < BITS_PER_PIXEL:
            self.shift = (self.shift << 1) | value
        self.nbits += 1
        self.bits.append(value)
        self.pending = None

    def _latch(self, t: int) -> None:
        """The reset code: take the word, or keep the colour if there is not one.

        A bit whose high pulse has already ended is the frame's last one -- its low
        time is the reset code, not a bit time -- so it is taken in here, before the
        frame is closed.
        """
        if self.pending is not None:
            self._shift_in(self.pending)
        word, bits = self.shift, self.nbits        # read the word before clearing it
        self.nbits, self.shift = 0, 0
        self.swallowing = True                     # the next frame starts with our word
        if bits < BITS_PER_PIXEL:
            self.short_resets += 1
            return
        self.color = rgb_from_word(word)
        self.frames += 1
        self.latch_log.append(LatchEvent(tick=t, rgb=self.color, word=word, bits=bits))


# ---- harness helpers -------------------------------------------------------------
def _ensure_net(bus: Bus, name: str) -> None:
    """Add one push-pull net if it is not there yet.

    Push-pull because the WS2812 wire is: page 1's "Cascading port transmission signal
    by single line" and page 3's VDD-referenced input thresholds describe a driven
    signal, not an open-drain one, so two devices driving opposite values on this net is
    contention and the core records it.
    """
    if name not in bus.nets:
        bus.add_net(name, push_pull=True)


def attach_transmitter(bus: Bus, **kwargs) -> Ws2812Tx:
    """Build a Ws2812Tx that takes its timing from `bus` and attach it."""
    tx = Ws2812Tx(bus=bus, **kwargs)
    _ensure_net(bus, tx.net)
    bus.attach(tx)
    return tx


def attach_chain(
    bus: Bus, count: int = 8, timing: Optional[dict] = None, **kwargs
) -> list[Ws2812Pixel]:
    """Build `count` pixels, the DOUT of one wired to the DIN of the next, and attach
    them. Returns the list in chain order: element 0 is the pixel the controller talks
    to, element `count - 1` the far end.

    The nets are `din` for the controller's wire and `dout0 .. dout{count-1}` for the
    taps, so a checker can be pointed at any of them and read what that pixel sees.
    """
    if count < 1:
        raise ValueError(f"a chain needs at least one pixel, got {count!r}")
    first = kwargs.pop("din_net", "din")
    pixels = []
    for index in range(count):
        din = first if index == 0 else f"dout{index - 1}"
        _ensure_net(bus, din)
        _ensure_net(bus, f"dout{index}")
        pixels.append(Ws2812Pixel(din_net=din, dout_net=f"dout{index}", bus=bus,
                                  timing=timing, **kwargs))
    for pixel in pixels:
        bus.attach(pixel)
    return pixels


def attach_checker(bus: Bus, **kwargs) -> Ws2812TimingChecker:
    """Build a passive Ws2812TimingChecker for `bus` and attach it."""
    chk = Ws2812TimingChecker(bus=bus, **kwargs)
    _ensure_net(bus, chk.net)
    bus.attach(chk)
    return chk


def wait_latched(bus: Bus, chain, frames: int = 1, limit: int = 4_000_000) -> bool:
    """Step `bus` until every pixel of `chain` has latched `frames` more colours.

    A chain needs this rather than `Ws2812Tx.run()`: pixel `k` latches one tick after
    pixel `k - 1` and the far end of a chain has seen one word fewer, so a run that ends
    when the transmitter ends stops before the last pixels have latched anything. False
    if `limit` ticks ran out first.
    """
    base = [p.frames for p in chain]
    return bus.run_until(
        lambda _b: all(p.frames >= b + frames for p, b in zip(chain, base)), limit
    )


@dataclass(frozen=True)
class Violation:
    """One rule the wire broke, with the tick it was broken on.

    `sense` is which way the limit goes: "min" means the rule is a floor and
    `measured < required` is the violation, "max" means a ceiling and `measured >
    required` is, and "exact" means the two have to be equal -- the frame rule, where a
    frame of 25 bits is as wrong as one of 23. The datasheet has all three kinds, so a
    violation says which it was.
    """

    tick: int
    rule: str            # "T0H", "T1H", "T0L", "T1L", "DECODE", "TBIT", "RESET", "FRAME"
    measured: int        # ticks the wire gave it
    required: int        # ticks the profile asks for
    sense: str = "min"
    detail: str = ""

    def __str__(self) -> str:
        sign = {"min": "<", "max": ">", "exact": "!="}[self.sense]
        unit = "ticks" if self.sense != "exact" else "bits"
        return (f"t={self.tick} {self.rule} {self.measured} {unit} {sign} {self.required}"
                + (f" ({self.detail})" if self.detail else ""))


@dataclass(frozen=True)
class BitEvent:
    """One bit as the checker decoded it.

    `tick` is the tick the bit's high pulse *began*; `high_ticks` is measured there and
    `low_ticks` when the bit's low period ended one bit later, which is why the event
    is only recorded at that later tick.
    """

    tick: int
    value: int                    # 0 or 1; a high that decoded as neither has no event
    high_ticks: int
    low_ticks: int

    def __str__(self) -> str:
        return f"t={self.tick} {self.value} {self.high_ticks}+{self.low_ticks} ticks"


@dataclass(frozen=True)
class FrameEvent:
    """One frame: the bits between two reset codes, and the words they carried."""

    tick: int                     # the tick the frame's first high began, -1 if unknown
    bits: int
    reset_tick: int               # the tick the reset code ended: the frame is latched
    words: tuple[int, ...]        # the datasheet's 24-bit words, in order

    @property
    def pixels(self) -> int:
        return len(self.words)

    @property
    def colors(self) -> list[tuple[int, int, int]]:
        return [rgb_from_word(w) for w in self.words]


class Ws2812TimingChecker(Device):
    """Passive monitor for the WS2812 wire: decodes the stream and checks the spec.

    It drives nothing, so it sees the bus exactly as an analyser would, including the
    core's two-phase tick: it measures resolved pin values and adds no latency of its
    own. It measures you, not the devices. Point it at any net -- the controller's
    wire or any tap in a chain -- and it reports what that node sees.

    Rules, each one a row of the datasheet's timing table or its frame description, and
    each one reported with the tick it happened on:

        T0H    a high time under T0H's floor, so short to be a 0
        T1H    a high time over T1H's ceiling, so long to be a 1
        DECODE a high time between T0H's ceiling and T1H's floor: inside neither window,
               so a receiver cannot know which bit was meant
        T0L    a bit decoded 0 whose low time is longer than T0L's ceiling
        T1L    a bit decoded 1 whose low time is longer than T1L's ceiling
        TBIT   a bit period outside TH+TL's window. The only rule that catches a
               transmitter that is simply too fast: a 0 bit with a 0.4 us high and a
               0.2 us low meets every floor and every ceiling above and is 600 ns long.
        RESET  the line came back high while a frame was still open, or `finalize()`
               found one still open: the frame is never latched by a strip
        FRAME  a reset ended a frame whose bit count is not a multiple of 24

    T0H and T1H are one-sided each, and that is not a gap in the checking but the whole
    story: a high time can be shorter than a 0, inside a 0, inside neither window, inside
    a 1, or longer than a 1, and those five cases are T0H, a 0, DECODE, a 1 and T1H. The
    "0 too long" and "1 too short" failures are the dead band between the two windows,
    which is what DECODE is for, and they are reported as such rather than twice.

    Every limit comes from `ws2812_timings()`, from the profile given or from a
    `timing` override, so a checker can be pointed at a different datasheet revision
    ({"t_reset": 80_000}) or at a tighter reading of this one ({"tol": 0}).

    One thing it measures one tick long: the bus shows the pull-up on a net nothing has
    driven yet, so the first tick of a run reads high whatever the transmitter is about
    to drive, and the first high pulse can come out one tick too wide. It is the
    two-phase tick's resolution rather than anything the transmitter did, it is at most
    one tick, and `periods` carries it.

    Two things a checker on this wire cannot see, both worth knowing before trusting a
    green result. It cannot know how many pixels are on the chain, so it cannot tell a
    correctly sized frame from a correctly sized *half* frame -- compare `frames`
    against the pixels you built. And without a reset code there is nothing that marks
    where one frame ends and the next begins, so a stream that never resets is one long
    frame to this checker: it reports the missing reset only when the line goes high
    again (RESET) or when `finalize()` is called.
    """

    name = "ws2812_chk"

    def __init__(
        self,
        net: str = "din",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = "ws2812b-2013",
        timing: Optional[dict] = None,
        name: Optional[str] = None,
    ):
        self.net = net
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        if name:
            self.name = name
        self._compute_timings()
        self.violations: list[Violation] = []
        self.bits: list[BitEvent] = []
        self.frames: list[FrameEvent] = []
        self.resets: list[int] = []
        self.periods: list[int] = []
        self._reset()

    def _compute_timings(self) -> None:
        self.min = ws2812_timings(self.ns_per_tick, self.profile, self.timing_overrides)

    def bind(self, bus: Bus) -> "Ws2812TimingChecker":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute_timings()
        return self

    def _reset(self) -> None:
        self.level: Optional[int] = None    # last level seen; None until the first tick
        self.level_since = 0
        self.high_start = 0                 # tick the current high pulse began
        self.high_ticks = 0
        self.pending: Optional[int] = None  # bit whose high pulse has ended
        self.undecodable = False            # the last high pulse decoded as neither
        # The first high pulse after attach cannot be measured: the bus shows the pull-up
        # on a net nothing has driven yet, so a checker comes up in the middle of a pulse
        # whose duration is not the transmitter's doing. `warm` is that one pulse.
        self.warm = True
        self.nbits = 0                      # bits since the last reset
        self.shift = 0                      # the current word, MSB first
        self.frame_words: list[int] = []      # the words of the frame being decoded
        self.frame_start = -1

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def colors(self) -> list[tuple[int, int, int]]:
        """Every colour of the last frame, in order."""
        return self.frames[-1].colors if self.frames else []

    @property
    def words(self) -> list[int]:
        """Every word of the last frame, in order."""
        return list(self.frames[-1].words) if self.frames else []

    @property
    def bit_rate_hz(self) -> tuple[float, float]:
        """The slowest and fastest bit period measured, in bit/s."""
        if not self.periods:
            return (0.0, 0.0)
        return (1e9 / (self.ns_per_tick * max(self.periods)),
                1e9 / (self.ns_per_tick * min(self.periods)))

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def report(self, limit: int = 10) -> str:
        rate = self.bit_rate_hz
        head = (f"{self.profile} on {self.net} at {self.ns_per_tick} ns/tick: "
                f"{len(self.bits)} bits, {len(self.frames)} frames, "
                f"{len(self.resets)} resets, {len(self.violations)} violations")
        if self.periods:
            head += (f", bit rate {rate[0] / 1e3:.0f}..{rate[1] / 1e3:.0f} kbit/s "
                     f"(the datasheet quotes {self.min['bit_rate'] / 1e3:.0f})")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: int, required: int,
                   sense: str = "min", detail: str = "") -> None:
        missed = {"min": measured < required,
                  "max": measured > required,
                  "exact": measured != required}[sense]
        if missed:
            self.violations.append(Violation(t, rule, measured, required, sense, detail))

    # ---- the monitor --------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        level = pins[self.net]
        if self.level is None:                      # first tick: a level, not an edge
            self.level, self.level_since = level, t
            return {}
        if level == self.level:
            # A low that has lasted long enough is the reset code by itself: a strip
            # latches on the elapsed time and never waits for the line to come back up,
            # so a transmitter that idles low still ends its frame. The frame is
            # stamped at the earliest tick it could legally be latched.
            if (level == 0 and self.nbits
                    and t - self.level_since >= self.min["t_reset_min"]):
                self._reset_code(self.level_since + self.min["t_reset_min"])
            return {}                                # no edge: the timestamps stand
        self._fall(t) if level == 0 else self._rise(t)
        self.level, self.level_since = level, t
        return {}

    def _fall(self, t: int) -> None:
        """The line fell: the high pulse just ended, so the bit can be read."""
        self.high_start = self.level_since
        self.high_ticks = t - self.level_since
        if self.warm and self.high_ticks < self.min["t0h_min"]:
            # Too short to be any bit, and it arrived before the frame did: this is the
            # pull-up on a net nothing has driven yet, not data. A pulse long enough to
            # be a bit is data, even the first one, so it is read normally.
            self.warm = False
            self.pending = None
            return
        self.warm = False
        self.pending = classify_bit(self.high_ticks, self.min)
        if self.pending is None:
            # Neither window covers it: a receiver cannot know which bit was meant, so
            # the bit is left out of the stream entirely rather than guessed at. The next
            # rise then has nothing to commit, and that is *this* fault, not a missing
            # reset -- `undecodable` is what keeps the two apart.
            self._violation(t, "DECODE", self.high_ticks, self.min["t0h_max"], "max",
                            f"a 0 ends at {self.min['t0h_max']} ticks and a 1 starts at "
                            f"{self.min['t1h_min']}")
            self.undecodable = True
            return
        high, rule = (self.min["t0h_min"], "T0H") if self.pending == 0 else (
            self.min["t1h_max"], "T1H")
        sense = "min" if self.pending == 0 else "max"
        self._violation(t, rule, self.high_ticks, high, sense,
                        f"the high time of a {self.pending} bit")

    def _rise(self, t: int) -> None:
        """The line rose: either the reset code ended here, or a bit is complete."""
        low = t - self.level_since
        if low >= self.min["t_reset_min"]:
            self._reset_code(t)
            return
        if self.pending is None:
            if self.nbits and not self.undecodable:
                # The line went high with no high pulse behind it, in the middle of a
                # frame: nothing is left to receive and no reset has been sent.
                self._violation(t, "RESET", low, self.min["t_reset_min"], "min",
                                "the line went high again before the reset code")
            return
        self._commit(t, low)

    def _commit(self, t: int, low: int) -> None:
        """One bit is complete: check its low time and its period, then shift it in."""
        rule, ceiling = ("T0L", self.min["t0l_max"]) if self.pending == 0 else (
            "T1L", self.min["t1l_max"])
        self._violation(t, rule, low, ceiling, "max",
                        f"the low time after a {self.pending} bit")
        period = self.high_ticks + low
        self.periods.append(period)
        self._violation(t, "TBIT", period, self.min["t_bit_min"], "min",
                        "the bit period is too short for TH+TL")
        self._violation(t, "TBIT", period, self.min["t_bit_max"], "max",
                        "the bit period is too long for TH+TL")
        self.bits.append(BitEvent(tick=self.high_start, value=self.pending,
                                  high_ticks=self.high_ticks, low_ticks=low))
        self._shift_in(self.pending)

    def _shift_in(self, value: int) -> None:
        """Take one decoded bit into the frame: count it, and pack it into a word."""
        self.undecodable = False
        if self.nbits == 0:
            self.frame_start = self.high_start
        self.nbits += 1
        self.shift = (self.shift << 1) | value
        if self.nbits % BITS_PER_PIXEL == 0:
            self.frame_words.append(self.shift)
            self.shift = 0

    def _reset_code(self, t: int) -> None:
        """A low at least as long as the reset code: the frame is over and latched."""
        if self.pending is not None:
            # The last bit of the frame. Its low time is the reset code itself, not a
            # bit time, so none of T0L, T1L or TBIT is checked against it -- a receiver
            # learns the bit from the high pulse and the reset from the low one.
            self.bits.append(BitEvent(tick=self.high_start, value=self.pending,
                                      high_ticks=self.high_ticks, low_ticks=0))
            self._shift_in(self.pending)
        self.pending = None
        self.undecodable = False
        if self.nbits:
            self.resets.append(t)
            if self.nbits % BITS_PER_PIXEL:
                self._violation(t, "FRAME", self.nbits, BITS_PER_PIXEL, "exact",
                                f"the reset ended a frame of {self.nbits} bits")
            if self.shift:                    # a trailing partial word, not counted above
                self.frame_words.append(self.shift)
            self.frames.append(FrameEvent(tick=self.frame_start, bits=self.nbits,
                                          reset_tick=t, words=tuple(self.frame_words)))
        self.nbits, self.shift, self.frame_words = 0, 0, []
        self.frame_start = -1

    # ---- closing the stream -------------------------------------------------------
    def finalize(self, t: Optional[int] = None) -> list[Violation]:
        """Close the stream and report whatever was still open. Returns what it added.

        A frame that never got its reset code was never latched by a strip, and while
        the line stays low there is nothing on the wire to say so: the checker only
        learns about it when the line goes high again or when the run ends. Call this at
        the end of a run to make that last case a violation instead of a silence.
        """
        t = self.level_since + 1 if t is None else t
        before = len(self.violations)
        if self.nbits:
            self._violation(t, "RESET", 0, self.min["t_reset_min"], "min",
                            f"the stream stopped with {self.nbits} bits and no reset code")
        self.nbits, self.shift, self.frame_words = 0, 0, []
        self.pending = None
        return self.violations[before:]


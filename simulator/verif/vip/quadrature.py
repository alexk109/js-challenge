"""Pin-level quadrature-encoder (A/B/I) reference models on the tick bus of verif/vip/core.py.

A quadrature encoder puts four Gray-coded states on two push-pull lines, 90 degrees apart, so
one mechanical step is four state changes and one A pulse is four steps. That is exactly the
numbers the ams/Infineon AS5047P datasheet gives for its ABI outputs, which is the source for
everything here (`research/ref/protocols/quadrature/as5047p-abi.md` holds the quotes and the
arithmetic):

    "4000 steps / 1000 pulses per revolution"          -> counts_per_rev = 4 * ppr
    "The phase shift between the signals A and B indicates the rotation direction:
     e.g. DIR-Bit = 0, clockwise (A leads, B follows)" -> which Gray cycle is "up"
    "During the start-up time ... all three ABI signals are high"  -> power-on state
    "ABI Output: The state of ABI is frozen to ABI = 111"          -> error safe state
    "MS Maximum speed ... 28000 RPM"                                -> shortest legal A period
    "DAEmax Dynamic angle error ... 0.18 degree"                    -> how much phase is legal

Three devices, all on the nets `a`, `b` and `i`:

  RotaryEncoder      drives A/B/I for a shaft that turns at a variable speed and in a
                     variable direction: a queue of (revolutions, rpm) segments, or a
                     runtime `rpm`, plus fault injectors for a state the encoder cannot
                     produce (a dropped output state, a one-line glitch, an overspeed burst,
                     a phase skew, a too-narrow index pulse).
  QuadratureDecoder  the device under test: counts A/B into a position at x1, x2 or x4,
                     works out the direction from the Gray successor table, flags an
                     impossible transition instead of counting it, and latches the index.
  QuadratureChecker  passive monitor: measures the wire against a profile's limits and
                     flags every violation with the tick it happened on.

Typical use, at the 50 MHz of `verif/vip/core.py` (20 ns per tick):

    bus = Bus(ns_per_tick=20)
    enc = attach_encoder(bus, ppr=1000)                 # 1000 pulses/rev, 4000 steps/rev
    dec = attach_decoder(bus, x=4, initial_position=enc.initial_counts)
    chk = attach_checker(bus, profile="as5047p")        # optional, passive
    enc.queue_turns(0.05, 10_000)                      # 200 counts, 167 rev/s
    enc.run(bus, limit=50_000)
    assert dec.position == enc.position == 196          # -4 counts of index window, then 200
    assert chk.violations == []

Ticks per count is 750 000 / rpm at 20 ns per tick, so pick the speed for the run you want: a
full second of a 1000 ppr encoder at its 28 000 RPM limit is about 1.9 million ticks and
1.9 million edges, while 0.05 revolutions at 10 000 RPM is 200 counts and takes 15 000 ticks.

Latency, as in `verif/vip/uart.py`: a driver that returns a value at tick `t` has the net read
that value from the end of `t`, so every device sees it at `t+1`. The decoder takes that one
tick out of every tick it reports (`latency=1`), so its positions and edge ticks are the ticks
the encoder really drove. The checker, like `I2cTimingChecker`, measures the wire and takes
nothing out: it measures the pins, not the devices.

What the datasheet does not say, and what this module therefore decides for itself (both are
configurable, and `research/ref/protocols/quadrature/as5047p-abi.md` says so where the numbers
are derived): the A/B waveform geometry is a picture in the datasheet (Figure 31) with no text
layer, so the minimum level width, the minimum edge separation and the index width are
*derived* from the maximum speed and the steps-per-pulses ratio, and the phase tolerance is a
half count. See `profile_minimums()` for each one's arithmetic.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import ceil, floor
from typing import Optional

from .core import Bus, Device

# Gray cycle with A as the MSB. 00 -> 10 -> 11 -> 01 -> 00 is the one where A rises before B,
# which is the datasheet's clockwise / DIR=0 / increasing direction ("A leads, B follows").
# Every step changes exactly one line, which is what makes a two-bit change illegal.
CW_CYCLE = (0b00, 0b10, 0b11, 0b01)
CCW_CYCLE = tuple(reversed(CW_CYCLE))
NEXT_CW = {CW_CYCLE[i]: CW_CYCLE[(i + 1) % 4] for i in range(4)}
NEXT_CCW = {CCW_CYCLE[i]: CCW_CYCLE[(i + 1) % 4] for i in range(4)}
STARTUP_LEVELS = (1, 1, 1)      # "all three ABI signals are high" after power-on
ERROR_SAFE_LEVELS = (1, 1, 1)   # "the state of ABI is frozen to ABI = 111"


def step_delta(old: int, new: int) -> Optional[int]:
    """+1, -1 or None for one A/B transition: a step forward, a step back, or impossible.

    This one table is the whole decoder rule. `None` means the pair changed by two bits at
    once, which is not a state the Gray cycle contains: the datasheet's sequence moves one
    output at a time (4000 steps / 1000 pulses means four steps per A pulse, four states per
    A pulse), so anything else is a fault on the wire, not motion.
    """
    if new == NEXT_CW.get(old):
        return 1
    if new == NEXT_CCW.get(old):
        return -1
    return None


@dataclass(frozen=True)
class QuadratureProfile:
    """One encoder's ABI parameters: what it emits and how fast it may emit it.

    `ppr` is the datasheet's "pulses per revolution" and `counts_per_rev` its "steps per
    revolution"; the ratio is 4 because one A pulse carries four steps, which is what x4
    decoding counts. `max_rpm` is the datasheet's `MS`, the only speed limit, and it is what
    `profile_minimums()` turns into the shortest legal A period.
    """

    name: str
    ppr: int
    counts_per_rev: int
    max_rpm: float
    source: str


QUADRATURE_PROFILES: dict[str, QuadratureProfile] = {
    # Figure 30, ABIRES = 000, ABIBIN = 0, and Figure 9's MS. The default.
    "as5047p": QuadratureProfile(
        "as5047p", 1000, 4000, 28_000.0,
        "AS5047P [v2-00] Fig. 30 (4000 steps / 1000 ppr) and Fig. 9 (MS = 28000 RPM)"),
    # Figure 30, ABIRES = 000, ABIBIN = 1: binary mode, 4096 steps / 1024 ppr, same MS.
    "as5047p-binary": QuadratureProfile(
        "as5047p-binary", 1024, 4096, 28_000.0,
        "AS5047P [v2-00] Fig. 30 (4096 steps / 1024 ppr) and Fig. 9 (MS = 28000 RPM)"),
    # Not a datasheet part: a slow mechanical encoder, for proving the checker reads its
    # limits from the profile instead of from constants. 250 ppr at 300 RPM gives an A
    # period of 800 us, 2 666 times longer than the AS5047P's fastest.
    "slow-mechanical": QuadratureProfile(
        "slow-mechanical", 250, 1000, 300.0,
        "model profile (no datasheet): 250 ppr, 300 RPM max, for checker-not-vacuous tests"),
}


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Rounding a minimum up only makes it stricter."""
    return int(ceil(max(0.0, ns) / ns_per_tick))


#: Overrides `profile_minimums()` understands, and in what unit. The times are nanoseconds,
#: like the ns timing tables in `verif/vip/i2c.py`, so a test can point the checker at another
#: encoder without knowing the tick rate.
TIMING_OVERRIDES = ("min_period", "min_edge", "min_level", "min_index_width")
OTHER_OVERRIDES = ("max_rpm", "phase_tol_counts")


def profile_minimums(
    ns_per_tick: float = 20.0, profile: str = "as5047p", overrides: Optional[dict] = None
) -> dict:
    """Every limit of one encoder profile, in nanoseconds and in whole ticks.

    The datasheet gives the maximum speed and the number of steps per revolution and nothing
    else about the waveform (its timing diagram is a picture), so the limits are derived --
    `research/ref/protocols/quadrature/as5047p-abi.md` D1, D2, D3 and D5 has the arithmetic:

        t_count   = 60e9 / (max_rpm * counts_per_rev)   one step = one edge interval
        min_edge  = t_count      two edges closer than one step cannot be told apart
        min_period= 4 * t_count  one A pulse; shorter means faster than the profile's MS
        min_level = 2 * t_count  each line holds its level for half an A pulse
        min_index_width = min_period   the index is one ABI cycle wide (a model rule: the
                              datasheet has no index width at all, only the I pin name)

    `min_level` is in the table for reference but is not a rule of its own: a line holds its
    level for two of the four intervals of an A pulse, so `min_edge` already implies it
    (2 x min_edge = min_level). It is there so a firmware author can see the number.

    For the default profile at 20 ns/tick that is t_count = 535.71 ns, min_edge = 536 ticks,
    min_period = 2143 ticks, min_level = 1072 ticks, min_index_width = 2143 ticks.

    `overrides` replaces individual values by name: the four times in nanoseconds, plus
    `max_rpm` in RPM and `phase_tol_counts` in steps. That is how a test drives an encoder, or
    a checker, off-profile on purpose -- `{"max_rpm": 1000}` says "this shaft must not exceed
    1000 RPM", `{"min_index_width": 5_000}` says "an index pulse shorter than 5 us is a fault".
    """
    try:
        prof = QUADRATURE_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f"unknown quadrature profile {profile!r}; pick one of {sorted(QUADRATURE_PROFILES)}"
        ) from None
    if not prof.max_rpm > 0 or not prof.ppr > 0:
        raise ValueError(f"profile {profile!r} needs a positive ppr and max_rpm")
    if prof.counts_per_rev != 4 * prof.ppr:
        raise ValueError(
            f"profile {profile!r}: {prof.counts_per_rev} steps per revolution is not four times "
            f"{prof.ppr} pulses per revolution, which is what one A pulse carries"
        )
    over = dict(overrides or {})
    for key in over:
        if key not in TIMING_OVERRIDES + OTHER_OVERRIDES:
            raise ValueError(
                f"unknown quadrature override {key!r}; pick from "
                f"{sorted(TIMING_OVERRIDES + OTHER_OVERRIDES)}"
            )
    max_rpm = float(over.get("max_rpm", prof.max_rpm))
    phase_tol = float(over.get("phase_tol_counts", 0.5))
    if not max_rpm > 0:
        raise ValueError(f"max_rpm must be positive, got {max_rpm!r}")
    if not phase_tol >= 0:
        raise ValueError(f"phase_tol_counts must be >= 0, got {phase_tol!r}")

    t_count = 60e9 / (max_rpm * prof.counts_per_rev)
    ns = {
        "t_count": t_count,
        "min_edge": 1.0 * t_count,
        "min_period": 4.0 * t_count,
        "min_level": 2.0 * t_count,
        "min_index_width": 4.0 * t_count,
    }
    for key in TIMING_OVERRIDES:
        if key in over:
            if not float(over[key]) > 0:
                raise ValueError(f"{key} must be positive, got {over[key]!r}")
            ns[key] = float(over[key])
    out = {f"{k}_ns": v for k, v in ns.items()}
    out.update({k: ns_to_ticks(ns_per_tick, v) for k, v in ns.items()})
    out["max_ab_hz"] = 1e9 / ns["min_period"]
    out["max_rpm"] = max_rpm
    out["phase_tol_counts"] = phase_tol
    out["ppr"] = prof.ppr
    out["counts_per_rev"] = prof.counts_per_rev
    out["profile"] = profile
    out["source"] = prof.source
    return out


@dataclass
class EdgeRecord:
    """One A/B edge the encoder drove."""

    tick: int            # the tick the drive was returned, as the bus saw it
    ab: int              # the A/B pair after the edge, A as the MSB
    net: str             # which line moved: "a" or "b"
    level: int           # the level that line took
    counts: float        # the true position, in counts, when the edge was driven
    faulted: bool = False    # a dropped state, a glitch or a skew was in effect


class RotaryEncoder(Device):
    """A quadrature encoder on the nets `a`, `b` and `i`, turning as fast or as slowly as it
    is told to.

    The shaft's position is a float count: `counts_per_rev` = 4 * `ppr`, and one A pulse is
    four counts, which is the datasheet's "4000 steps / 1000 pulses per revolution". Every
    count produces one edge of the Gray cycle, so the levels on the wire are whatever the
    cycle says and the *timing* of those edges is what the checker measures.

    Speed is a queue of segments, `queue_turns(turns, rpm)`, and the sign of `rpm` is the
    direction (positive is the datasheet's clockwise, where A leads B). `rpm` is a plain
    attribute, so a test may also set it mid-run: the running segment still ends after its
    own number of revolutions.

    `startup_ticks` (8 by default) models the datasheet's power-on state, in which all three
    ABI signals are high, before the shaft moves at all.

    The fault injectors exist so the checker can be tested against traffic a real encoder
    cannot produce, and each one names the rule it should trip in its docstring:

      skip_states(n)   a state the encoder failed to put out      -> AB_ORDER
      glitch(net, ticks)  one line moves and moves back alone    -> AB_SAME
      overspeed(...)    faster than the profile's MS             -> fAB, tEDGE
      set_skew(...)     B's edges early inside one A pulse       -> AB_PHASE, tEDGE
      narrow_index(n)   an index pulse too short                 -> IDX_WIDTH

    `index=False` models an encoder with no index output at all: I is held low, so an
    overspeed test sees only the speed rules.
    """

    name = "quad_enc"

    def __init__(
        self,
        ppr: int = 1000,
        ns_per_tick: float = 20.0,
        *,
        rpm: float = 0.0,
        profile: Optional[str] = None,
        index_width_counts: int = 4,
        index_offset_counts: int = 0,
        startup_ticks: int = 8,
        start_state: int = 0b11,
        index: bool = True,
        a_net: str = "a",
        b_net: str = "b",
        i_net: str = "i",
        name: Optional[str] = None,
    ):
        if profile is not None:
            ppr = QUADRATURE_PROFILES[profile].ppr
        if not int(ppr) >= 1:
            raise ValueError(f"ppr must be >= 1, got {ppr!r}")
        if int(start_state) not in CW_CYCLE:
            raise ValueError(
                f"start_state must be one of the four Gray states {CW_CYCLE}, got "
                f"{start_state!r}"
            )
        if not 1 <= int(index_width_counts) <= 4:
            raise ValueError(
                f"index_width_counts must be 1..4 counts (one A pulse is four), got "
                f"{index_width_counts!r}"
            )
        if name:
            self.name = name
        self.ppr = int(ppr)
        self.counts_per_rev = 4 * self.ppr
        self.ns_per_tick = float(ns_per_tick)
        self.a_net, self.b_net, self.i_net = a_net, b_net, i_net
        self.index_width_counts = int(index_width_counts)
        self.index_offset_counts = int(index_offset_counts)
        self.startup_ticks = int(startup_ticks)
        self.index = bool(index)
        self.rpm = float(rpm)

        # The shaft starts at count 0, whose Gray state is the one the datasheet calls the
        # power-on state (all three ABI signals high), so the first step off it is a legal
        # step and the decoder's first count matches this model's.
        # The shaft starts one index window *before* count 0, so the run begins with the
        # index low and every index pulse it produces is a whole one. Starting inside the
        # window would leave the first pulse high before anyone was watching it, and its
        # width would be `startup_ticks` short of the truth.
        self._initial_counts = -float(self.index_width_counts)
        self._counts = self._initial_counts
        self._index = CW_CYCLE.index(int(start_state))
        self._ab = CW_CYCLE[self._index]
        self._index_level = self._index_at(self._counts)
        self._wait = 0.0                         # ticks left before the next edge
        self._startup = self.startup_ticks
        self._queue: deque = deque()
        self._seg: Optional[dict] = None
        self._seg_end = 0.0
        self._levels = STARTUP_LEVELS

        # faults
        self._skip = 0                           # states still to drop
        self._glitch: Optional[dict] = None      # {"net","level","left","from"}
        self._overspeed: Optional[dict] = None   # {"rpm","left"}
        self._skew_ticks = 0.0                   # B's edges early inside one A pulse

        # logs
        self.edges: list[EdgeRecord] = []
        self.index_edges: list[tuple] = []
        self.event_log: list[tuple] = []   # (tick, what the model was told to do)

    # ---- configuration -------------------------------------------------------
    @property
    def ticks_per_count(self) -> float:
        """Ticks from one ABI edge to the next at the current speed (infinite when stopped)."""
        rpm = self.effective_rpm
        if rpm == 0.0:
            return float("inf")
        return 60e9 / (abs(rpm) * self.counts_per_rev) / self.ns_per_tick

    @property
    def effective_rpm(self) -> float:
        """`rpm`, or an overspeed fault's rpm while one is running."""
        if self._overspeed is not None and self._overspeed["left"] > 0:
            return float(self._overspeed["rpm"])
        return float(self.rpm)

    @property
    def direction(self) -> int:
        """+1 clockwise (A leads B), -1 counter-clockwise, 0 stopped."""
        rpm = self.effective_rpm
        return (rpm > 0) - (rpm < 0)

    @property
    def counts(self) -> float:
        """True shaft position in counts, signed and unbounded (not what the wire showed)."""
        return self._counts

    @property
    def position(self) -> int:
        """True shaft position in whole counts, signed and unbounded."""
        return int(floor(self._counts))

    @property
    def initial_counts(self) -> int:
        """Where `counts` starts, i.e. minus one index window. Tell a decoder this as its
        `initial_position` and the two models agree count for count from the first edge."""
        return int(self._initial_counts)

    @property
    def revolutions(self) -> float:
        return self._counts / self.counts_per_rev

    @property
    def ab(self) -> int:
        """The A/B pair the encoder last drove, A as the MSB."""
        return self._ab

    @property
    def index_level(self) -> int:
        return self._index_level

    @property
    def index_pulses(self) -> int:
        """Rising edges of I so far."""
        return sum(1 for _, level in self.index_edges if level == 1)

    @property
    def busy(self) -> bool:
        return not self.idle

    @property
    def idle(self) -> bool:
        return (
            not self._queue
            and self._seg is None
            and self.rpm == 0.0
            and (self._overspeed is None or self._overspeed["left"] <= 0)
            and (self._glitch is None or self._glitch["left"] <= 0)
        )

    def bind(self, bus: Bus) -> "RotaryEncoder":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.ns_per_tick = bus.ns_per_tick
        return self

    # ---- the motion queue -----------------------------------------------------
    def queue_turns(self, turns: float, rpm: float) -> dict:
        """Turn `turns` revolutions at `rpm` (signed: the sign is the direction), then stop.

        A segment ends on its revolution count whatever happens to `rpm` in between, so a test
        may retune the speed mid-turn and still get exactly the revolutions it asked for.
        """
        if float(turns) <= 0:
            raise ValueError(f"queue_turns wants a positive number of turns, got {turns!r}")
        if float(rpm) == 0:
            raise ValueError("queue_turns wants a non-zero rpm: 0 would never finish")
        seg = {"turns": float(turns), "rpm": float(rpm), "left": None}
        self._queue.append(seg)
        return seg

    def queue_seconds(self, seconds: float, rpm: float) -> dict:
        """Turn at `rpm` for `seconds`, then stop."""
        if float(seconds) <= 0:
            raise ValueError(f"queue_seconds wants a positive duration, got {seconds!r}")
        if float(rpm) == 0:
            raise ValueError("queue_seconds wants a non-zero rpm: 0 would never finish")
        seg = {"turns": None, "rpm": float(rpm),
               "left": int(round(float(seconds) * 1e9 / self.ns_per_tick))}
        self._queue.append(seg)
        return seg

    def queue_profile(self, segments) -> deque:
        """Several segments at once: `queue_profile([(0.25, 100), (0.25, -400), ...])`.

        This is how "variable speed and direction" is expressed: each item is a number of
        revolutions and the signed rpm to turn them at.
        """
        for turns, rpm in segments:
            self.queue_turns(turns, rpm)
        return self._queue

    def stop(self) -> None:
        """Drop every queued segment and stop where the shaft is."""
        self._queue.clear()
        self._seg = None
        self.rpm = 0.0

    def run(self, bus: Bus, limit: int = 2_000_000) -> bool:
        """Step `bus` until the queue drains and the shaft is still. False on a timeout."""
        return bus.run_until(lambda _b: self.idle, limit)

    # ---- fault injectors ------------------------------------------------------
    def _index_at(self, counts: float) -> int:
        """The index level at a position: high inside the index window, low outside it.

        The window is `index_width_counts` counts of every revolution, starting at
        `index_offset_counts`. It opens at count 0, which is why the power-on state is 111:
        at count 0 the index is high. With `index=False` there is no window and the pin is
        held low, which is what an encoder without an index output does.
        """
        if not self.index:
            return 0
        r = (counts - self.index_offset_counts) % self.counts_per_rev
        return 1 if r < self.index_width_counts else 0

    def skip_states(self, n: int = 1) -> None:
        """Drop the next `n` ABI states: the wire jumps a state instead of walking it.

        Two bits move at once, which is not in the Gray cycle, so the checker must flag
        `AB_ORDER` and the decoder must refuse to count the step.
        """
        if int(n) < 1:
            raise ValueError(f"skip_states wants at least one state, got {n!r}")
        self._skip += int(n)

    def glitch(self, net: str, ticks: int) -> None:
        """Drive one line to the opposite level for `ticks` ticks, with the shaft held still.

        One line moves and moves back with no edge on its partner, which is exactly what the
        `AB_SAME` rule is for. `net` is "a", "b" or "i"; only a and b can trip that rule.
        """
        if net not in (self.a_net, self.b_net, self.i_net):
            raise ValueError(f"glitch net must be one of the ABI nets, got {net!r}")
        if int(ticks) < 1:
            raise ValueError(f"glitch needs at least one tick, got {ticks!r}")
        self._glitch = {"net": net, "level": -1, "left": int(ticks), "from": None}

    def overspeed(self, rpm: float, ticks: int) -> None:
        """Turn at `rpm` for `ticks` ticks, whatever the profile's maximum speed is.

        Faster than `max_rpm` means an A pulse shorter than the profile allows (`fAB`) and
        edges closer together than one step (`tEDGE`).
        """
        if float(rpm) == 0:
            raise ValueError("overspeed wants a non-zero rpm")
        if int(ticks) < 1:
            raise ValueError(f"overspeed needs at least one tick, got {ticks!r}")
        self._overspeed = {"rpm": float(rpm), "left": int(ticks), "from": None}

    def set_skew(self, edge_ticks: float) -> None:
        """Move B's edges `edge_ticks` ticks early inside every A pulse, A pulse length kept.

        The four intervals of a pulse stop being a quarter of a cycle each, which is what
        `AB_PHASE` measures, and the two close pairs get closer than one step, which is what
        `tEDGE` measures. `set_skew(0)` is the ideal, symmetric waveform.
        """
        if float(edge_ticks) < 0:
            raise ValueError(f"skew cannot be negative, got {edge_ticks!r}")
        self._skew_ticks = float(edge_ticks)

    def narrow_index(self, counts: int) -> None:
        """Make the index pulse `counts` counts wide instead of the configured width."""
        if not 1 <= int(counts) <= 4:
            raise ValueError(f"narrow_index wants 1..4 counts, got {counts!r}")
        self.index_width_counts = int(counts)

    # ---- the device -----------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        self._advance(t)
        drv = {self.a_net: self._levels[0], self.b_net: self._levels[1],
               self.i_net: self._levels[2]}
        g = self._glitch
        if g is not None and g["left"] > 0:
            drv[g["net"]] = g["level"]
        return drv

    def _advance(self, t: int) -> None:
        """Everything that happens on this tick: faults, the queue, and at most one edge."""
        self._tick_faults(t)
        self._tick_segment(t)
        if self._startup > 0:
            self._startup -= 1
            return
        g = self._glitch
        if g is not None and g["left"] > 0:
            g["left"] -= 1
            if g["left"] == 0:                   # back on the grid, a whole step later
                self._wait = self.ticks_per_count
            return                               # the shaft is held while a line glitches
        tpc = self.ticks_per_count
        if tpc == float("inf"):
            return                               # stopped: no edges at all
        self._wait -= 1.0
        if self._wait <= 0.0:
            self._emit_edge(t, tpc)

    def _tick_faults(self, t: int) -> None:
        o = self._overspeed
        if o is not None and o["left"] > 0:
            if o["from"] is None:
                o["from"] = t
                self.event_log.append((t, f"overspeed {o['rpm']:g} RPM for {o['left']} ticks"))
            o["left"] -= 1
            if o["left"] <= 0:
                self._overspeed = None
        g = self._glitch
        if g is not None and g["left"] > 0 and g["from"] is None:
            g["from"] = t
            which = self.a_net if g["net"] == self.a_net else \
                (self.b_net if g["net"] == self.b_net else self.i_net)
            g["level"] = 1 - self._levels[("a", "b", "i").index(which)]
            self.event_log.append((t, f"glitch {g['net']}={g['level']} for {g['left']} ticks"))

    def _tick_segment(self, t: int) -> None:
        """Start the next queued segment, or stop the shaft when the queue runs dry."""
        if self._seg is None:
            if not self._queue:
                return
            seg = self._queue.popleft()
            self._seg = seg
            self.rpm = seg["rpm"]
            turns = seg["turns"]
            self._seg_end = (self._counts + turns * self.counts_per_rev
                             * (1 if seg["rpm"] > 0 else -1) if turns is not None else None)
            self.event_log.append((t, f"segment: {turns} turns at {seg['rpm']:g} RPM"))
            return
        seg = self._seg
        if seg["left"] is not None:               # a seconds-long segment
            seg["left"] -= 1
            if seg["left"] > 0:
                return
        else:
            done = ((self._counts >= self._seg_end) if seg["rpm"] > 0
                    else (self._counts <= self._seg_end))
            if not done:
                return
        self._seg = None
        self._seg_end = 0.0
        if self._queue:
            return                                # the next segment starts on the next tick
        self.rpm = 0.0

    def _emit_edge(self, t: int, tpc: float) -> None:
        """One ABI edge: move the Gray cycle on, move the shaft, re-derive the index."""
        src = self._ab
        step = 1
        faulted = self._skip > 0
        if faulted:
            self._skip -= 1
            step = 2                              # the state in between never reached the pin
        d = self.direction or 1
        self._index = (self._index + d * step) % 4
        self._ab = CW_CYCLE[self._index]
        self._counts += d                        # the shaft moved one step either way
        net = "a" if ((src ^ self._ab) & 0b10) else "b"
        level = (self._ab >> (1 if net == "a" else 0)) & 1
        self.edges.append(EdgeRecord(t, self._ab, net, level, self._counts, faulted))
        new_index = self._index_at(self._counts)
        if new_index != self._index_level:
            self._index_level = new_index
            self.index_edges.append((t, new_index))
        self._levels = ((self._ab >> 1) & 1, self._ab & 1, self._index_level)
        # Skew shortens the interval that leads into a B edge and lengthens the one after it,
        # so the A pulse (four intervals) keeps its length and only the phase moves.
        nxt = CW_CYCLE[(self._index + d) % 4]
        next_net = "a" if ((self._ab ^ nxt) & 0b10) else "b"
        self._wait = tpc - self._skew_ticks if next_net == "b" else tpc + self._skew_ticks
        self._wait = max(self._wait, 0.5)


# ---- harness helpers -------------------------------------------------------------
def _ensure_nets(bus: Bus, push_pull: bool = True) -> None:
    """Add the A/B/I nets if they are not there yet.

    They are push-pull, because an encoder drives 0 and 1 in turn on the same line: a second
    driver on one of these nets would be a real contention and `Bus` records it in
    `bus.errors`.
    """
    for net in ("a", "b", "i"):
        if net not in bus.nets:
            bus.add_net(net, push_pull=push_pull)


def attach_encoder(bus: Bus, **kwargs) -> RotaryEncoder:
    """Build a RotaryEncoder that takes its tick rate from `bus` and attach it."""
    _ensure_nets(bus)
    enc = RotaryEncoder(ns_per_tick=bus.ns_per_tick, **kwargs)
    bus.attach(enc)
    return enc


def attach_decoder(bus: Bus, **kwargs) -> "QuadratureDecoder":
    """Build a QuadratureDecoder for `bus` and attach it."""
    _ensure_nets(bus)
    dec = QuadratureDecoder(ns_per_tick=bus.ns_per_tick, **kwargs)
    bus.attach(dec)
    return dec


def attach_checker(bus: Bus, **kwargs) -> "QuadratureChecker":
    """Build a passive QuadratureChecker for `bus` and attach it."""
    _ensure_nets(bus)
    chk = QuadratureChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk


@dataclass
class DecodeError:
    """One impossible A/B transition, which the decoder refuses to count."""

    tick: int
    detail: str
    old_ab: int
    new_ab: int

    def __str__(self) -> str:
        return f"t={self.tick} {self.detail}"


@dataclass
class DecodeEdge:
    """One A/B edge as the decoder saw it, one tick's latency already taken out."""

    tick: int
    ab: int
    net: str
    level: int
    delta: Optional[int]     # +1, -1, or None for an impossible transition
    counted: bool            # whether this edge's step went into the position


@dataclass
class IndexEvent:
    """One edge of the index pin."""

    tick: int
    level: int
    position: int            # the decoded position when it happened
    revolutions: float


class QuadratureDecoder(Device):
    """The device under test: counts A/B into a position, with a direction and an index.

    `x` is the decoding factor: 4 counts every edge (the datasheet's 4000 steps per 1000
    pulses), 2 counts the edges of `channel` ("a" or "b", default "b"), and 1 counts that
    channel's rising edge. Direction comes from the Gray successor table on every edge,
    whichever edges `x` counts, because which line moved is what says which way the shaft
    turned ("A leads, B follows" is clockwise).

    An impossible transition -- two lines moving at once, which no state in the cycle can do
    -- is not counted. `position` does not move, `valid` goes False, and the transition is
    appended to `errors` with its tick. That is the behaviour a receiver IC with diagnostics
    has, and it is the point of the model: a fault on the wire must not turn into motion.

    `position` is signed and unbounded, so a test can see exactly how far off a dropped edge
    left it; `position_in_rev` is the same number folded into one revolution. `rpm` is the
    speed over the last `window_ticks` ticks (0 = since the first counted edge).
    `initial_position` homes the counter: give it the encoder's `initial_counts` and the two
    models agree count for count, or set it to whatever a firmware index-homing routine would
    have written after the last index pulse.
    """

    name = "quad_dec"

    def __init__(
        self,
        x: int = 4,
        *,
        channel: str = "b",
        ns_per_tick: float = 20.0,
        latency: int = 1,
        counts_per_rev: int = 4000,
        window_ticks: int = 0,
        initial_position: int = 0,
        initial_ab: int = 0b11,
        a_net: str = "a",
        b_net: str = "b",
        i_net: str = "i",
        name: Optional[str] = None,
    ):
        if int(x) not in (1, 2, 4):
            raise ValueError(f"decoding factor x must be 1, 2 or 4, got {x!r}")
        if channel not in ("a", "b"):
            raise ValueError(f"channel must be 'a' or 'b', got {channel!r}")
        if int(latency) < 0:
            raise ValueError(f"latency must be >= 0, got {latency!r}")
        if int(counts_per_rev) < 1:
            raise ValueError(f"counts_per_rev must be >= 1, got {counts_per_rev!r}")
        if name:
            self.name = name
        self.x = int(x)
        self.channel = channel
        self.ns_per_tick = float(ns_per_tick)
        self.latency = int(latency)
        self.counts_per_rev = int(counts_per_rev)
        self.window_ticks = int(window_ticks)
        self.a_net, self.b_net, self.i_net = a_net, b_net, i_net

        self.position = int(initial_position)
        self.initial_position = int(initial_position)
        self.direction = 0
        self.valid = False
        self.edges: list[DecodeEdge] = []
        self.errors: list[DecodeError] = []
        self.index_events: list[IndexEvent] = []
        self.index_rising = 0
        self.counted = 0
        self._ab = int(initial_ab)
        self._index_level = 0
        self._index_started = False
        self._ref_tick: Optional[int] = None
        self._ref_pos = 0
        self._rpm = 0.0

    def bind(self, bus: Bus) -> "QuadratureDecoder":
        self.ns_per_tick = bus.ns_per_tick
        return self

    # ---- what the tests ask for --------------------------------------------------
    @property
    def revolutions(self) -> float:
        return self.position / self.counts_per_rev

    @property
    def position_in_rev(self) -> int:
        return self.position % self.counts_per_rev

    @property
    def index_count(self) -> int:
        """Rising edges of the index seen so far."""
        return self.index_rising

    @property
    def rpm(self) -> float:
        """Speed over the last `window_ticks` ticks, signed; 0 before the first edge."""
        return self._rpm

    @property
    def clean(self) -> bool:
        """True while no impossible transition has been seen."""
        return not self.errors

    def pop_error(self) -> Optional[DecodeError]:
        """Oldest decode error not yet popped, or None."""
        return self.errors.pop(0) if self.errors else None

    # ---- the device -------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        a, b = pins[self.a_net], pins[self.b_net]
        ab = (a << 1) | b
        if ab != self._ab:
            net = "a" if (ab >> 1) != (self._ab >> 1) else "b"
            self._on_edge(t - self.latency, self._ab, ab, net, a if net == "a" else b)
            self._ab = ab
        i = pins[self.i_net]
        if not self._index_started:
            # The first sample of the net is its baseline, not an edge: `Bus` starts every net
            # at its pull-up, so an index pin that reads high at power-on would otherwise look
            # like an index pulse before the shaft has moved.
            self._index_started = True
            self._index_level = i
            return {}
        if i != self._index_level:
            self._index_level = i
            self.index_rising += int(i)
            self.index_events.append(
                IndexEvent(t - self.latency, i, self.position, self.revolutions))
        return {}

    def _on_edge(self, t: int, old: int, new: int, net: str, level: int) -> None:
        delta = step_delta(old, new)
        if delta is None:
            self.valid = False
            moved = bin(old ^ new).count("1")
            self.errors.append(
                DecodeError(t, f"impossible A/B transition {old:02b}->{new:02b}, "
                               f"{moved} lines moved at once", old, new))
            self.edges.append(DecodeEdge(t, new, net, level, None, False))
            return
        self.direction = delta
        counted = self._counts_this_edge(net, level)
        self.edges.append(DecodeEdge(t, new, net, level, delta, counted))
        if not counted:
            return
        self.position += delta
        self.counted += 1
        self.valid = True
        if self._ref_tick is None:
            self._ref_tick, self._ref_pos = t, self.position
            return
        span = t - self._ref_tick
        if span <= 0:
            return
        window = self.window_ticks or span
        if span >= window:
            seconds = span * self.ns_per_tick / 1e9
            self._rpm = ((self.position - self._ref_pos) / self.counts_per_rev) / seconds * 60.0
            self._ref_tick, self._ref_pos = t, self.position

    def _counts_this_edge(self, net: str, level: int) -> bool:
        """Which edges this decoder's `x` counts."""
        if self.x == 4:
            return True
        if self.x == 2:
            return net == self.channel
        return net == self.channel and level == 1


@dataclass(frozen=True)
class Violation:
    """One limit the wires did not meet, with the tick it was missed on."""

    tick: int
    rule: str            # "AB_ORDER", "AB_SAME", "tEDGE", "fAB", "AB_PHASE", "IDX_WIDTH"
    measured: float      # what the wire did
    required: float      # what the profile asks for
    detail: str = ""
    ceiling: bool = False    # True when the rule is "too much" rather than "too little"

    def __str__(self) -> str:
        return (f"t={self.tick} {self.rule} {self.measured:g} "
                f"{'>' if self.ceiling else '<'} {self.required:g}"
                + (f" ({self.detail})" if self.detail else ""))


class QuadratureChecker(Device):
    """Passive monitor: measures the A/B/I wires against one profile's limits.

    It drives nothing, so it sees the pins exactly as an analyser would, including the
    two-phase tick's sampling latency: every tick it reports is a tick of the wire, with
    nothing added or subtracted. It measures the pins, not the devices.

    Six rules, every one logged with the tick it was missed on. Five are floors -- the wire
    must stay above a limit -- and three of those six are checked as ceilings instead, because
    "too much" is the wrong way round for them; `Violation.ceiling` says which:

      AB_ORDER   a transition moved more than one line. The Gray cycle moves one line per
                 step (4000 steps per 1000 pulses means four steps, and four states, per A
                 pulse), so two lines at once is not a step the encoder can make. This is the
                 one rule measured as a ceiling rather than a floor.
      AB_SAME    one line moved with no edge on the other for a whole A pulse, which no legal
                 waveform does at any speed: the cycle alternates A, B, A, B, so the other
                 line always has an edge within one step. Only edges that were legal in the
                 first place are considered, so a transition that already failed AB_ORDER is
                 not counted here as well. A direction reversal is not a violation either --
                 see `_edge`.
      tEDGE      two consecutive edges closer together than one step. Two edges the decoder
                 cannot tell apart are a lost step whatever they are called.
      fAB        an A pulse (rising edge to rising edge) shorter than the profile allows.
                 This is the maximum speed: `max_rpm`, the datasheet's `MS`, is the only
                 speed limit there is.
      AB_PHASE   the four intervals inside one A pulse are not a quarter of a cycle each,
                 more than `phase_tol_counts` steps of skew. Only measured between two pulses
                 of the same length: while the shaft accelerates the four intervals of a pulse
                 are not a quarter of it, and reading that as skew would make the rule
                 useless on real motion. `pulses_skipped` counts the pulses left alone, and
                 a checker told a wider `phase_tol_counts` measures every one.
      IDX_WIDTH  an index pulse narrower than the profile's index width.

    `fAB` and `tEDGE` overlap on purpose: a uniformly fast encoder breaks both, because its
    edges close up together as its pulses shorten. A skewed encoder at a legal speed breaks
    only `tEDGE` and `AB_PHASE`, and a skewed encoder inside the tolerance breaks only
    `AB_PHASE`. `report()` and `violations_by_rule()` are what a test asserts on.

    The first A pulse has no predecessor to be measured against, and an index pulse the run
    ends inside has no width yet, so neither is measured; `periods`, `gaps` and `index_pulses`
    say how many were. Every sample of every net is a baseline rather than an edge, so a
    device that drives a net low from the first tick does not look like a one-tick pulse.
    """

    name = "quad_chk"

    def __init__(
        self,
        profile: str = "as5047p",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        a_net: str = "a",
        b_net: str = "b",
        i_net: str = "i",
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else float(ns_per_tick)
        self.timing_overrides = dict(timing or {})
        self.profile = profile
        self.min = profile_minimums(self.ns_per_tick, profile, self.timing_overrides)
        self.a_net, self.b_net, self.i_net = a_net, b_net, i_net
        if name:
            self.name = name
        self.violations: list[Violation] = []
        self.edges: list[tuple] = []        # (tick, net, level, ab)
        self.pulses: list[tuple] = []       # (start_tick, length, intervals)
        self.index_edges: list[tuple] = []  # (tick, level)
        self.shortest_period: Optional[int] = None
        self.shortest_gap: Optional[int] = None
        self.worst_phase_counts = 0.0
        self.shortest_index: Optional[int] = None
        self.longest_index: Optional[int] = None
        self._reset()

    def bind(self, bus: Bus) -> "QuadratureChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = profile_minimums(self.ns_per_tick, self.profile, self.timing_overrides)
        return self

    def _reset(self) -> None:
        self._prev_ab: Optional[int] = None
        self._last_edge: Optional[int] = None
        self._last_a_rise: Optional[int] = None
        self._last_pulse_len: Optional[int] = None
        self._last_change = {"a": None, "b": None}
        self._silence_skip = 0       # edges of silence measurement to skip after AB_ORDER
        self._gaps: deque = deque(maxlen=4)     # the four intervals of one A pulse
        self._index_rise: Optional[int] = None
        self.gaps = 0
        self.periods = 0
        self.pulses_skipped = 0
        self.index_pulses = 0

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def max_ab_hz(self) -> Optional[float]:
        """The fastest A pulse rate the wire actually ran at, from the shortest one seen."""
        if self.shortest_period is None:
            return None
        return 1e9 / (self.ns_per_tick * self.shortest_period)

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def rules_broken(self) -> set:
        """The set of rules that fired, which is what most tests assert on."""
        return {v.rule for v in self.violations}

    def report(self, limit: int = 10) -> str:
        head = (f"{self.profile} profile at {self.ns_per_tick} ns/tick: "
                f"{len(self.edges)} A/B edges, {self.periods} A pulses "
                f"({self.pulses_skipped} accelerating, not measured), "
                f"{self.index_pulses} index pulses, {len(self.violations)} violations")
        if self.shortest_period is not None:
            head += (f", fastest A pulse {self.shortest_period} ticks "
                     f"({self.max_ab_hz / 1e3:.1f} Hz, limit "
                     f"{self.min['max_ab_hz'] / 1e3:.1f} Hz)")
        if self.shortest_gap is not None:
            head += f", closest two edges {self.shortest_gap} ticks"
        head += f", worst phase skew {self.worst_phase_counts:.3f} steps"
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: float, required: float,
                   detail: str = "") -> None:
        """A limit the wire must stay above: fAB, tEDGE, IDX_WIDTH."""  # noqa: D401
        if measured < required:
            self.violations.append(Violation(t, rule, measured, required, detail))

    def _violation_max(self, t: int, rule: str, measured: float, allowed: float,
                       detail: str = "") -> None:
        """A ceiling rather than a floor: AB_ORDER (lines per transition) and AB_PHASE.

        Phase skew has no minimum -- the ideal waveform has none at all -- and a transition
        that moves two lines is worse than one that moves one, so neither can be checked by
        asking whether the measurement fell short. They get their own method rather than a
        sign trick inside `_violation`.
        """
        if measured > allowed:
            self.violations.append(Violation(t, rule, measured, allowed, detail, ceiling=True))

    # ---- the monitor -------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        ab = (pins[self.a_net] << 1) | pins[self.b_net]
        prev = self._prev_ab
        if prev is None:
            self._prev_ab = ab            # the first observation is the baseline, not an edge
        elif ab != prev:
            net = "a" if (ab >> 1) != (prev >> 1) else "b"
            self._edge(t, net, ab)
        i = pins[self.i_net]
        if not self.index_edges:
            # The first sample of a net is its baseline, not an edge: `Bus` starts every net
            # at its pull-up, so a device that drives 0 from tick 0 would otherwise look like
            # a one-tick pulse on every net it holds low.
            self.index_edges.append((t, i))
            return {}
        if i != self.index_edges[-1][1]:
            self.index_edges.append((t, i))
            self._index_edge(t, i)
        return {}

    def _edge(self, t: int, net: str, ab: int) -> None:
        """One observed A/B edge: gap, dead time, legality, and (on A) the pulse."""
        prev = self._prev_ab
        level = (ab >> 1) & 1 if net == "a" else ab & 1
        self.edges.append((t, net, level, ab))
        if self._last_edge is not None:
            gap = t - self._last_edge
            self.gaps += 1
            self._gaps.append(gap)
            self.shortest_gap = gap if self.shortest_gap is None else min(self.shortest_gap, gap)
            self._violation(t, "tEDGE", gap, self.min["min_edge"],
                            f"{net} edge only {gap} ticks after the previous edge")
        # AB_SAME: the cycle alternates A, B, A, B, so the gap between one line's edges is
        # one step, and a step is never as long as a pulse -- at any speed, with any legal
        # skew. The threshold is therefore the A pulse length *measured on the wire*, which
        # is the clock the silence has to fit inside, floored at the profile's shortest legal
        # pulse so that a burst that has just sped up cannot leave a stale, too-small one
        # behind it. The rule waits for the first pulse to be measured, exactly as fAB does.
        # A direction reversal is not a violation: a shaft that turns back retraces the state it
        # came from, so it does put two edges of one line next to each other, but the timing
        # stays on the grid.
        legal = prev is not None and step_delta(prev, ab) is not None
        other = "b" if net == "a" else "a"
        seen_other = self._last_change[other]
        self._last_change[net] = t
        if legal and seen_other is not None and not self._silence_skip \
                and self._last_pulse_len is not None:
            allowed = max(self._last_pulse_len, self.min["min_period"])
            self._violation_max(
                t, "AB_SAME", t - seen_other, allowed,
                f"{net} moved with no {other} edge for a whole A pulse ({allowed} ticks)")
        if prev is not None and not legal:
            moved = bin(prev ^ ab).count("1")
            self._violation_max(t, "AB_ORDER", moved, 1,
                                f"{prev:02b} -> {ab:02b}: {moved} lines moved at once")
            # The wire is out of sync now, so the next edge's silence says nothing: a skipped
            # state leaves the other line quiet for two steps by construction. AB_ORDER is
            # the finding; this is only there so one fault is not reported as two.
            self._silence_skip = 1
        if self._silence_skip:
            self._silence_skip -= 1
        self._prev_ab = ab
        self._last_edge = t
        if net == "a" and level == 1:
            self._a_rise(t)

    def _a_rise(self, t: int) -> None:
        """Close the A pulse that ended at this rising edge: its length, then its phase."""
        if self._last_a_rise is None:
            self._last_a_rise = t            # the first pulse has no predecessor to measure
            return
        length = t - self._last_a_rise
        self.periods += 1
        self.pulses.append((self._last_a_rise, length, tuple(self._gaps)))
        self.shortest_period = (length if self.shortest_period is None
                                else min(self.shortest_period, length))
        self._violation(t, "fAB", length, self.min["min_period"],
                        "A pulse is %.1f Hz, %s caps it at %.1f Hz"
                        % (1e9 / (self.ns_per_tick * length), self.profile,
                           self.min["max_ab_hz"]))
        self._last_a_rise = t
        # The phase is only worth measuring between two pulses of the same length: the four
        # intervals of a pulse a shaft is accelerating through are not a quarter of it, and
        # reading that as skew would make the rule useless on real motion. `pulses_skipped`
        # counts the pulses it left alone.
        steady = self._last_pulse_len is not None and abs(length - self._last_pulse_len) <= 1
        self._last_pulse_len = length
        if not steady:
            self.pulses_skipped += 1
            return
        self._phase(t)

    def _phase(self, t: int) -> None:
        """AB_PHASE: were the four intervals inside the last A pulse a quarter each?"""
        if len(self._gaps) < 4:
            return
        total = float(sum(self._gaps))
        if total <= 0:
            return
        dev = max(abs(4.0 * (g - total / 4.0) / total) for g in self._gaps)
        self.worst_phase_counts = max(self.worst_phase_counts, dev)
        self._violation_max(t, "AB_PHASE", dev, self.min["phase_tol_counts"],
                            "A pulse intervals %s ticks, %.3f steps of skew"
                            % (list(self._gaps), dev))

    def _index_edge(self, t: int, level: int) -> None:
        """Index pulse widths. A rise with no fall after it is never measured -- a pulse the
        run ended inside has no width yet -- and `index_pulses` counts what was."""
        if level == 1:
            self._index_rise = t
            return
        if self._index_rise is None:
            return
        width = t - self._index_rise
        self.index_pulses += 1
        self.shortest_index = width if self.shortest_index is None else min(self.shortest_index,
                                                                          width)
        self.longest_index = width if self.longest_index is None else max(self.longest_index,
                                                                          width)
        self._violation(t, "IDX_WIDTH", width, self.min["min_index_width"],
                        "index pulse high for %d ticks" % width)
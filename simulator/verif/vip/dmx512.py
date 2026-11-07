"""Pin-level DMX512-A reference models on the tick bus in verif/vip/core.py.

One net, named "d", carries the line a receiver sees: 1 is MARK (the idle state, which
is also a released driver) and 0 is SPACE. That single net is the differential pair of
clause 4 collapsed to logic, which is all a pin-level model can see; it is push-pull,
because EIA-485 drives both states, so two controllers on one link resolve to 0 and the
bus records it as contention -- which clause 4.3 forbids anyway ("a data link shall
consist of a single active differential line driver").

The frame, exactly as clause 8 and table 5 of ANSI E1.11 have it:

    BREAK (space, >= 88 us) | MAB (mark, >= 8 us) | slot 0 = START Code
    | slot 1 .. slot 512 (8N2, LSB first, mark when the slot is not used)
    | MBB (mark) .. the next BREAK

Three devices, all of them driven by the tick rate (bind it with `bus=`, 20 ns/tick by
default), so every timing below comes from the standard's tables rather than from
constants typed in here:

    DmxTx          controller: a queue of packets, each one BREAK, MAB, START Code and
                   up to 512 data slots, then a mark before the next break. Per-packet
                   timing overrides and a per-packet fault dict are how a test drives
                   itself off-spec on purpose.
    DmxRx          receiver: recognises the break, waits out the MAB, samples slot 0 and
                   the data slots at the centres of their bits, checks both stop bits
                   (clause 9.1) and answers with the levels of its own address window.
                   `DmxRx(..., timing={"break": ...})` shortens what it recognises.
    DmxChecker     passive monitor: measures the wire, fits the bit grid, and flags every
                   timing or framing rule the standard states, each with the tick it
                   happened on. It never drives, so two controllers can be checked at
                   once. Call `finish()` at the end of a run to close the last packet.

Typical use:

    bus = Bus(ns_per_tick=20)                                  # 50 MHz
    tx = attach_tx(bus)                                        # 250 kbit/s
    rx = attach_rx(bus, address=1, footprint=4)                # channels 1..4
    chk = attach_checker(bus)                                  # optional, passive
    tx.queue_packet([0xFF, 0x80, 0x00, 0x40])                  # NULL START Code
    tx.run(bus, limit=200_000)
    chk.finish()
    assert chk.violations == [], chk.report()
    assert list(rx.levels[0:4]) == [0xFF, 0x80, 0x00, 0x40]

Two timing tables, and which one a device is judged against matters. Table 6 is what the
*transmitting* UART must produce (break >= 92 us, MAB >= 12 us, break-to-break >= 1204 us
and <= 1.00 s) and table 7 is what a *receiver* must accept (break >= 88 us, MAB >= 8 us,
break-to-break >= 1196 us and <= 1.25 s). `dmx_minimums(ns_per_tick, profile)` returns one
of them as whole ticks. `DmxTx` uses "transmit" and `DmxRx` and `DmxChecker` use
"receive", which is the strict-but-fair choice: a controller that meets the transmitter
table always passes the receiver table, while the checker told "transmit" (strict_tx=True)
also demands the transmitter's own margins.

Every number below is cited against the spec text saved in
research/ref/protocols/dmx512/ (ANSI E1.11-2004 and the current ANSI E1.11-2024; the two
editions' timing tables are identical). See that directory's README.md for the citation
table: `dmx_minimums` and the profiles carry it line by line.

Four things to know before using this with firmware of your own:

* **Call `finish()`.** A packet is only known to be over when the next BREAK arrives, so
  `DmxChecker` and `DmxRx` hold the last packet of a run open; `finish()` closes it and is
  idempotent. `levels` and every counter are up to date at every tick, so `finish()` only
  matters for `DmxChecker.packets`, `DmxChecker.violations` and `DmxRx.packets`.
* **`DmxRx` is given a bit time, the checker measures one.** Like `UartRx` in
  verif/vip/uart.py, the receiver samples on a nominal grid (`ticks_per_bit`, 250 kbit/s by
  default), because a receiver needs a clock at the right rate. `DmxChecker` assumes
  nothing: it fits the bit grid from the edges of each packet. When a packet is outside the
  tables' 3.92 .. 4.08 us window the fit is a guess that pins the period to a whole number
  of the intervals observed, so the reported bit rate is exact whenever the data changes
  from bit to bit and can be a multiple of the truth otherwise -- it always names the right
  side of the window, and a packet whose bit rate is legal is always accepted exactly.
* **`latency` is the observation delay.** A driver on this bus changes the line one tick
  after it decides to, so both receivers default to `latency=1` and report the tick the
  transmitter really drove. Feed them from RTL through verif/vip/cocotb_bridge.py and pass
  that path's delay instead.
* **Only table 6, 7, 8.1 to 8.11 and 9 are modelled.** The electrical layer, the connectors,
  the annexes (Enhanced DMX512 / RDM, the SIP packet, "DMX512-A Protected") and the loss of
  data *handling* classes are not here: those are a receiver's product decisions, and a
  model cannot judge them from the wire.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil
from typing import Optional, Sequence

from .core import Bus, Device

__all__ = [
    "MARK",
    "SPACE",
    "SLOT_BITS",
    "NULL_START_CODE",
    "MAX_DATA_SLOTS",
    "DEFAULT_NET",
    "JITTER_TICKS",
    "DmxProfile",
    "DMX_PROFILES",
    "TX_PROFILE",
    "RX_PROFILE",
    "dmx_minimums",
    "ns_to_ticks",
    "DmxFrame",
    "DmxTx",
    "DmxPacket",
    "DmxRx",
    "Violation",
    "RULE_TEXT",
    "ObservedPacket",
    "DmxChecker",
    "attach_tx",
    "attach_rx",
    "attach_checker",
]

MARK = 1                  # a mark is a binary 1 and the idle state (definitions 3.24, 3.16)
SPACE = 0                 # a space is a binary 0 (definition 3.36)
SLOT_BITS = 11            # start + 8 data + 2 stop, no parity (table 5, clause 8.2)
NULL_START_CODE = 0x00    # clause 8.5.1, definition 3.28
MAX_DATA_SLOTS = 512      # slots 1..512; slot 0 is the START Code (clauses 8.1, 8.6)
DEFAULT_NET = "d"
_EPS = 1e-9               # float slack when comparing accumulated phase against a duration


@dataclass(frozen=True)
class DmxProfile:
    """One of the standard's two timing tables, in nanoseconds.

    `name` is "transmit" for table 6 (the values the output of a transmitting UART must
    meet) or "receive" for table 7 (the values a receiver must cope with). They differ in
    exactly three places, and a device that meets the transmitter table also meets the
    receiver table:

    |                 | transmit (table 6) | receive (table 7) |
    |-----------------|-------------------|--------------------|
    | `break_min`     | 92 us (typ. 176)  | 88 us              |
    | `mab_min`       | 12 us             | 8 us               |
    | `b2b_min`       | 1204 us           | 1196 us            |
    | `b2b_max`       | 1.00 s            | 1.25 s             |

    `bit_rate_min`, `bit_rate` and `bit_rate_max` are 245, 250 and 255 kbit/s in both
    tables; `break_min` is designator 1, `mab_min` designator 2, `b2b_min`/`b2b_max`
    designators 11 and 13, `idle_max` designator 9 (mark between slots) and `mbb_max`
    designator 10 (mark before the next break).
    """

    name: str
    break_min: float
    break_typ: Optional[float]
    mab_min: float
    b2b_min: float
    b2b_max: float
    idle_max: float = 1.0e9
    mbb_max: float = 1.0e9
    bit_rate_min: float = 245e3
    bit_rate: float = 250e3
    bit_rate_max: float = 255e3
    max_slots: int = MAX_DATA_SLOTS


#: table 6, "Timing Diagram Values - Output of Transmitting UART"
TX_PROFILE = DmxProfile("transmit", 92_000.0, 176_000.0, 12_000.0, 1_204_000.0, 1.0e9)
#: table 7, "Timing Diagram Values for Receivers"
RX_PROFILE = DmxProfile("receive", 88_000.0, 176_000.0, 8_000.0, 1_196_000.0, 1.25e9)
DMX_PROFILES = {"transmit": TX_PROFILE, "receive": RX_PROFILE}

_OVERRIDABLE = (
    "break", "mab", "b2b_min", "b2b_max", "idle_max", "mbb_max",
    "bit_rate", "bit_rate_min", "bit_rate_max", "max_slots",
)


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up, the way a minimum must be rounded."""
    return int(ceil(max(0.0, float(ns)) / ns_per_tick))


def dmx_minimums(
    ns_per_tick: float, profile: str = "receive", overrides: Optional[dict] = None
) -> dict:
    """Every limit of one timing table as whole ticks, plus the bit period in floats.

    The nanosecond minimums are rounded up, so a 20 ns tick can only ever make a check
    stricter than the exact value (88 us is 4400 ticks, never 4399). The bit period is
    *not* rounded: `ticks_per_bit` is a float, because 250 kbit/s is exactly 200 ticks at
    20 ns/tick but not at 25 ns/tick (4 us is 160.0, 8 ns is 500.0, 12.5 ns is 320.0), and
    a transmitter accumulates phase so each bit lasts floor() or ceil() ticks instead of
    accumulating error. `min_bit_ticks` and `max_bit_ticks` are the shortest and longest
    bit the tables allow (3.92 us and 4.08 us at 250 kbit/s), which is how the checker
    bounds a bit period it measured off the wire.

    `overrides` replaces individual values by their long names, which is how a test drives
    one device off-spec without touching the others: {"break": 10_000} means "a break of at
    least 10 us", {"bit_rate": 300e3} means "300 kbit/s". `bit_rate` may sit outside the
    `bit_rate_min`..`bit_rate_max` window on purpose -- that is how a test asks a
    transmitter to break the one rule a checker exists to catch -- so only positivity and
    the order of the two window bounds are enforced.
    """
    try:
        prof = DMX_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f"unknown DMX timing profile {profile!r}; pick one of {sorted(DMX_PROFILES)}"
        ) from None
    ns = {
        "break": prof.break_min,
        "mab": prof.mab_min,
        "b2b_min": prof.b2b_min,
        "b2b_max": prof.b2b_max,
        "idle_max": prof.idle_max,
        "mbb_max": prof.mbb_max,
    }
    slots = {"max_slots": prof.max_slots}
    hz = {
        "bit_rate": prof.bit_rate,
        "bit_rate_min": prof.bit_rate_min,
        "bit_rate_max": prof.bit_rate_max,
    }
    overrides = dict(overrides or {})
    for key, value in overrides.items():
        if key in ns:
            ns[key] = float(value)
        elif key in hz:
            hz[key] = float(value)
        elif key in slots:
            slots[key] = int(value)
        else:
            raise ValueError(
                f"unknown DMX timing override {key!r}; pick from {sorted(_OVERRIDABLE)}"
            )
    if not all(hz[key] > 0 for key in hz):
        raise ValueError(f"the bit rates must be positive, got {hz}")
    if hz["bit_rate_min"] > hz["bit_rate_max"]:
        raise ValueError(
            f"bit_rate_min {hz['bit_rate_min']} is above bit_rate_max {hz['bit_rate_max']}"
        )
    out = {key: ns_to_ticks(ns_per_tick, value) for key, value in ns.items()}
    out.update(hz)
    out.update(slots)
    out["bit_time"] = 1.0e9 / hz["bit_rate"]                     # ns
    out["ticks_per_bit"] = out["bit_time"] / ns_per_tick         # float, never rounded
    out["min_bit_ticks"] = 1.0e9 / hz["bit_rate_max"] / ns_per_tick   # 3.92 us: fastest
    out["max_bit_ticks"] = 1.0e9 / hz["bit_rate_min"] / ns_per_tick   # 4.08 us: slowest
    out["bit_time_min_ns"] = 1.0e9 / hz["bit_rate_max"]               # the tables' own
    out["bit_time_max_ns"] = 1.0e9 / hz["bit_rate_min"]               # 3.92 .. 4.08 us
    out["profile"] = profile
    return out


@dataclass
class DmxFrame:
    """One packet as the transmitter drove it, filled in while it is on the wire."""

    index: int
    start_tick: int = -1                       # tick the BREAK (the space) began
    end_tick: Optional[int] = None             # first idle tick after the last slot
    start_code: int = NULL_START_CODE
    values: tuple = ()
    break_ticks: int = 0
    mab_ticks: int = 0
    mbb_ticks: int = 0
    slot_ticks: float = 0.0            # 11 bit times, a float: the bit time need not be whole
    faults: tuple = ()
    timing: tuple = ()

    @property
    def slots(self) -> int:
        """Slots in this packet: the START Code plus every data slot."""
        return len(self.values) + 1

    @property
    def wire_ticks(self) -> int:
        """How many ticks the packet occupies, the mark before the next break included."""
        return (self.break_ticks + self.mab_ticks
                + int(round(self.slot_ticks * self.slots)) + int(self.mbb_ticks))

    def __str__(self) -> str:
        return (f"t={self.start_tick} packet {self.index}: break {self.break_ticks} ticks, "
                f"MAB {self.mab_ticks}, start code 0x{self.start_code:02x}, "
                f"{len(self.values)} data slot(s)")


class DmxTx(Device):
    """Controller: a queue of packets, each a BREAK, an MAB, a START Code and its slots.

    The durations it drives are the *transmit* profile's minimums (92 us break, 12 us MAB,
    250 kbit/s), because that is what a controller that wants the highest refresh rate does
    and because they are the shortest legal values: the checker's receiver-profile limits
    are below them, so a default DmxTx passes a default DmxChecker. `strict_tx=True` on
    the checker asks for the transmitter's own margins instead, which this class also meets
    by construction.

    Slots run back to back with no gap (table 6 designator 9 allows 0), each one 11 bit
    times long, and a slot whose value is not used is sent as all ones like any other
    value -- the standard has no separate "empty slot" encoding, so this model does not
    invent one.

    `pad_break_to_break` (default on) pads the mark before the next break so that the
    break-to-break time meets the transmit profile's 1204 us minimum. That is why a packet
    of one slot is legal at all: clause 8.7 sets no minimum number of data slots, but
    clause 8.10 sets a minimum packet time, and the two are satisfied with the pad.

    A test can make one packet off-spec without touching the class: `queue_packet(...,
    timing={"break": 10_000})` overrides that packet's limits, and `faults` injects a
    specific framing fault:

    * "stop1_low": n -- slot n's first stop bit is driven low (clause 9.1)
    * "stop2_low": n -- slot n's second stop bit is driven low (clause 9.1)
    * "bit_stretch": (n, bit, extra) -- slot n's bit `bit` (0 = start bit, 1..8 = data,
      9 and 10 = stop bits) lasts `extra` ticks longer than a bit, which is jitter no
      bit-rate tolerance covers
    * "short_start": (n, ticks) -- slot n's start bit is only `ticks` long, so it is not a
      whole bit time (table 5)
    * "double_break": 1 -- a second BREAK after the MAB, before the START Code (clause 8.9)
    * "space_between": (n, ticks) -- a stray SPACE of `ticks`, and as much mark again,
      between slot n and slot n+1, which clause 8.8 does not allow

    Slot n is counted from the START Code, so n = 0 is the START Code. For anything else,
    override `packet_segments()`.
    """

    name = "dmx_tx"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = "transmit",
        timing: Optional[dict] = None,
        net: str = DEFAULT_NET,
        pad_break_to_break: bool = True,
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = profile
        self.timing_overrides = dict(timing or {})
        self.net = net
        self.pad_break_to_break = pad_break_to_break
        if name:
            self.name = name
        self.queue: list[DmxFrame] = []
        self.tx_log: list[DmxFrame] = []
        self.errors: list[str] = []
        self._compute()
        self._reset()

    def bind(self, bus: Bus) -> "DmxTx":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute()
        return self

    def _compute(self) -> None:
        m = dmx_minimums(self.ns_per_tick, self.profile, self.timing_overrides)
        self.min = m
        self.ticks_per_bit = m["ticks_per_bit"]
        self.break_ticks = m["break"]
        self.mab_ticks = m["mab"]

    def _reset(self) -> None:
        self._segs: Optional[list] = None      # [(level, ticks), ...] of the packet in flight
        self._i = 0
        self._phase = 0.0                      # ticks into the current segment
        self._level = MARK
        self._cur: Optional[DmxFrame] = None

    # -- the queue ------------------------------------------------------------------
    def queue_packet(
        self,
        values: Sequence[int] = (),
        start_code: int = NULL_START_CODE,
        timing: Optional[dict] = None,
        faults: Optional[dict] = None,
    ) -> DmxFrame:
        """Queue one packet: BREAK, MAB, `start_code` in slot 0, then `values`.

        `timing` overrides this packet's timing limits (nanoseconds, or "bit_rate" in Hz)
        and `faults` injects a framing fault; both are for tests that need the wire to be
        wrong in one particular way. Returns the DmxFrame, which is filled in as the
        packet goes out and can be compared against what a receiver saw.
        """
        for value in list(values) + [start_code]:
            if not 0 <= int(value) <= 0xFF:
                raise ValueError(f"a DMX512 slot value is 0..255, got {value!r}")
        overrides = dict(self.timing_overrides, **(timing or {}))
        frame = DmxFrame(
            index=len(self.tx_log) + len(self.queue),
            start_code=int(start_code),
            values=tuple(int(v) for v in values),
            faults=tuple(sorted((faults or {}).items())),
            timing=tuple(sorted(overrides.items())),
        )
        self.queue.append(frame)
        return frame

    def queue(self, values: Sequence[int], start_code: int = NULL_START_CODE) -> DmxFrame:
        """Queue one NULL START Code packet (clause 8.5.1), the common case."""
        return self.queue_packet(values, start_code=start_code)

    @property
    def pending(self) -> int:
        """Packets queued but not started yet."""
        return len(self.queue)

    @property
    def busy(self) -> bool:
        return self._segs is not None or bool(self.queue)

    @property
    def idle(self) -> bool:
        return not self.busy

    def run(self, bus: Bus, limit: int = 2_000_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out first."""
        return bus.run_until(lambda _b: self.idle, limit)

    # -- the wire -------------------------------------------------------------------
    def packet_timings(self, frame: DmxFrame) -> dict:
        """The tick counts one packet is built from, its own overrides and all."""
        return dmx_minimums(
            self.ns_per_tick, self.profile, dict(self.timing_overrides, **dict(frame.timing))
        )

    def packet_segments(self, frame: DmxFrame, m: dict) -> list:
        """The packet as [(level, ticks), ...]. Override this to model another fault."""
        tpb = m["ticks_per_bit"]
        faults = dict(frame.faults)
        segs = [(SPACE, float(m["break"])), (MARK, float(m["mab"]))]
        if faults.get("double_break"):
            # clause 8.9 forbids this: a second break, with its own MAB so that a receiver
            # can tell the two reset sequences apart
            segs += [(SPACE, float(m["break"])), (MARK, float(m["mab"]))]
        for n in range(frame.slots):
            byte = frame.start_code if n == 0 else frame.values[n - 1]
            levels = [SPACE] + [(byte >> i) & 1 for i in range(8)] + [MARK, MARK]
            if faults.get("stop1_low") == n:
                levels[9] = SPACE
            if faults.get("stop2_low") == n:
                levels[10] = SPACE
            segs += [(level, tpb) for level in levels]
            stretch = faults.get("bit_stretch")
            if stretch is not None and stretch[0] == n:
                # one bit of slot n lasts `extra` ticks longer than it should
                segs[-len(levels) + stretch[1]] = (levels[stretch[1]], tpb + stretch[2])
            short = faults.get("short_start")
            if short is not None and short[0] == n:
                segs[-len(levels)] = (SPACE, float(short[1]))   # a start bit that is too short
            gap = faults.get("space_between")
            if gap is not None and gap[0] == n:
                # a stray space *and* as much mark again, or it would merge with the next
                # slot's start bit and never reach the wire as two pulses
                segs += [(SPACE, float(gap[1])), (MARK, float(gap[1]))]
        used = m["break"] + m["mab"] + SLOT_BITS * tpb * frame.slots
        mbb = 0
        if self.pad_break_to_break:
            # One tick of mark is unavoidable: a packet that ends on this tick lets the
            # next one start on the next, so b2b is `used + mbb + 1` ticks.
            mbb = max(1, int(m["b2b_min"]) - used - 1)
        if mbb > 0:
            segs.append((MARK, float(mbb)))
        frame.break_ticks, frame.mab_ticks, frame.mbb_ticks = m["break"], m["mab"], mbb
        frame.slot_ticks = SLOT_BITS * tpb
        return segs

    def _begin(self, t: int) -> None:
        frame = self.queue.pop(0)
        self._cur = frame
        frame.start_tick = t
        self._segs = self.packet_segments(frame, self.packet_timings(frame))
        self._i = 0
        self._phase = 0.0

    def _finish(self, t: int) -> None:
        if self._cur is not None:
            self._cur.end_tick = t
            self.tx_log.append(self._cur)
        self._segs, self._i, self._phase, self._level, self._cur = None, 0, 0.0, MARK, None

    def tick(self, t: int, pins) -> dict:
        if self._segs is None and self.queue:
            self._begin(t)
        level = MARK if self._segs is None else self._segs[self._i][0]
        if self._segs is not None:
            self._phase += 1.0
            while self._phase >= self._segs[self._i][1] - _EPS:
                self._phase -= self._segs[self._i][1]
                self._i += 1
                if self._i >= len(self._segs):
                    self._finish(t)
                    break
        return {self.net: level}

    # -- helpers --------------------------------------------------------------------
    def _ensure_net(self, bus: Bus) -> None:
        if self.net not in bus.nets:
            bus.add_net(self.net, push_pull=True)   # EIA-485 drives both states


@dataclass
class DmxPacket:
    """One packet as the receiver took it."""

    start_tick: int                # tick the BREAK ended, so the MAB begins
    break_ticks: int = 0
    mab_ticks: int = 0
    start_code: Optional[int] = None
    slots: int = 0                 # data slots accepted; framing-error slots are not counted
    values: tuple = ()             # the accepted data slot values, in slot order
    framing_errors: int = 0
    null_start: bool = False

    def __str__(self) -> str:
        code = "none" if self.start_code is None else f"0x{self.start_code:02x}"
        return (f"t={self.start_tick} packet: break {self.break_ticks} ticks, MAB "
                f"{self.mab_ticks}, start code {code}, {self.slots} data slot(s), "
                f"{self.framing_errors} framing error(s)")


class DmxRx(Device):
    """Receiver: break, MAB, START Code and slots, with the levels of its own window.

    It watches for a space at least as long as its `break` minimum rather than looking for
    one particular edge, because that is how the table 7 limit is written: a receiver has to
    recognise a break of 88 us whatever preceded it. After the break ends it waits out the
    MAB, then samples each slot at the centre of its eleven bits on a grid of
    `ticks_per_bit`, LSB first, and checks *both* stop bits (clause 9.1: a receiver shall
    check the first and should check the second). A slot with a missing stop bit is
    discarded together with every slot after it in that packet, as clause 9.1 requires.

    `ticks_per_bit` defaults to the nominal 250 kbit/s of the tables, exactly as `UartRx`
    in verif/vip/uart.py takes a nominal bit time: a receiver needs a clock at the right
    rate, and `DmxChecker` is the thing that verifies the transmitter ran at one. `latency`
    is subtracted from every tick reported, because a driver on this bus changes the line
    one tick after it decides to (core.py's two-phase tick), so the tick the receiver sees
    is one later than the tick the transmitter drove.

    `levels` is the whole 512-slot universe as the receiver currently believes it;
    `levels[address - 1:address - 1 + footprint]` is this device's window. A slot value of
    0xff is a real value, not "unused": the standard gives unused slots no special
    encoding, so a receiver cannot tell them apart from a full level.
    """

    name = "dmx_rx"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        ticks_per_bit: Optional[float] = None,
        address: int = 1,
        footprint: int = 1,
        profile: str = "receive",
        timing: Optional[dict] = None,
        latency: int = 1,
        net: str = DEFAULT_NET,
        name: Optional[str] = None,
    ):
        if not 1 <= address <= MAX_DATA_SLOTS:
            raise ValueError(
                f"a DMX512 receiver address is 1..{MAX_DATA_SLOTS} (slot 0 is the START "
                f"Code), got {address!r}"
            )
        if footprint < 1:
            raise ValueError(f"footprint must be at least one slot, got {footprint!r}")
        if address + footprint - 1 > MAX_DATA_SLOTS:
            raise ValueError(
                f"a {footprint}-slot receiver at address {address} runs past slot "
                f"{MAX_DATA_SLOTS}"
            )
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.address, self.footprint = address, footprint
        self.profile = profile
        self.latency = int(latency)
        if self.latency < 0:
            raise ValueError(f"latency must be >= 0, got {latency!r}")
        self.net = net
        if name:
            self.name = name
        self.min = dmx_minimums(self.ns_per_tick, profile, self.timing_overrides)
        self._tpb_default = ticks_per_bit is None
        self.ticks_per_bit = (float(ticks_per_bit) if ticks_per_bit is not None
                              else self.min["ticks_per_bit"])
        self.levels = bytearray(MAX_DATA_SLOTS)
        self.packets: list[DmxPacket] = []
        self.slot_log: list[tuple] = []          # (tick, slot number, value)
        self.start_code: Optional[int] = None
        self.slot_count = 0                      # data slots in the packet being taken
        self.breaks = 0
        self.framing_errors = 0
        self.mab_errors = 0
        self.short_breaks = 0
        self.double_breaks = 0
        self.glitches = 0
        self.over_512 = 0
        self.lost_data = 0
        self.lost_data_log: list[tuple] = []     # (tick of the reset that was late, how late)
        self._open: Optional[DmxPacket] = None
        self._reset()

    def bind(self, bus: Bus) -> "DmxRx":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = dmx_minimums(self.ns_per_tick, self.profile, self.timing_overrides)
        if self._tpb_default:
            self.ticks_per_bit = self.min["ticks_per_bit"]
        return self

    def _reset(self) -> None:
        self.state = "idle"      # idle | break | mab | slot | inter | drop
        self._last = MARK
        self._space_start: Optional[int] = None
        self._cand_slot: Optional[int] = None   # a fall not yet known to be a slot
        self._past: Optional[tuple] = None      # (fall, rise) of the slot being opened
        self._break_start = 0
        self._reset_tick = 0
        self._last_reset: Optional[int] = None
        self._origin = 0
        self._k = 0
        self._sample_at = float("inf")
        self._shift = 0
        self._framing = False

    # -- what the tests ask for ------------------------------------------------------
    @property
    def window(self) -> bytes:
        """The levels of this receiver's own slots."""
        return bytes(self.levels[self.address - 1:self.address - 1 + self.footprint])

    @property
    def null_start(self) -> bool:
        """True once a packet with a NULL START Code has been received."""
        return any(p.null_start for p in self.packets) or (
            self._open is not None and self._open.null_start
        )

    def data_lost(self, tick: int) -> bool:
        """True when no reset sequence has arrived within one maximum break-to-break time.

        Clause 9.2: a receiver that does not see a reset sequence within one maximum
        break-to-break time of the previous one has lost its data input, and it should stay
        in an operating condition for at least a minute waiting for the signal to come
        back. Pass the current tick (a test's `bus.t`); the receiver cannot know it by
        itself, because it sees no edges to count from.
        """
        if self._last_reset is None:
            return True
        return (tick - self._last_reset) > self.min["b2b_max"]

    def finish(self) -> Optional[DmxPacket]:
        """Close the packet being received, so the last packet of a run lands in `packets`.

        Every other packet is closed when the next BREAK is recognised, which is the only
        moment a packet is known to be over. `levels` and the counters are up to date at
        every tick, so this is only needed to read `packets`.
        """
        pkt, self._open = self._open, None
        if pkt is None:
            return None
        self.packets.append(pkt)
        return pkt

    # -- edges ------------------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        level = pins.get(self.net, MARK)
        prev = self._last
        self._last = level
        wire = t - self.latency
        if level == SPACE and prev == MARK:
            self._fall(wire)
        elif level == MARK and prev == SPACE:
            self._rise(wire)
        if level == SPACE:
            if self._space_start is None:
                self._space_start = wire
            elif self.state != "break" and wire - self._space_start >= self.min["break"]:
                self._break_seen()         # only a break is this long, wherever it is
        while self.state == "slot" and t >= self._sample_at:
            self._sample(self._level_at(self._sample_at - self.latency))
        return {}

    def _fall(self, wire: int) -> None:
        """Mark to space.

        Where a slot's start bit belongs -- after the MAB, or between two slots -- the space
        is only a candidate: a space that turns out to be a BREAK is a new packet, not a
        slot, and nothing may be decoded until the space has ended and said which it was.
        """
        self._space_start = wire
        if self.state in ("mab", "inter"):
            self._cand_slot = wire

    def _rise(self, wire: int) -> None:
        """Space to mark: the end of a space, and the only place the receiver decides."""
        start, self._space_start = self._space_start, wire
        cand, self._cand_slot = self._cand_slot, None
        length = wire - start
        if self.state == "break":
            if length < self.min["break"]:
                self.short_breaks += 1       # too short to be a break: no reset sequence
                self.state = "idle"
            else:
                self._end_break(wire, length)
            return
        if cand is not None and length < self.min["break"]:
            self._open_slot(cand, wire)     # a short space where a slot belongs was a slot

    def _level_at(self, wire: int) -> int:
        """The level at a tick in the past: the space we have just measured was a space."""
        past = self._past
        if past is not None and past[0] <= wire < past[1]:
            return SPACE
        return self._last

    def _end_break(self, wire: int, length: int) -> None:
        """The space ended and it was a real BREAK: a new packet starts here."""
        if self._open is not None:
            self.packets.append(self._open)
        self._open = None
        if self._last_reset is not None:
            late = wire - self._last_reset - self.min["b2b_max"]
            if late > 0:                            # clause 9.2 loss of data
                self.lost_data += 1
                self.lost_data_log.append((wire, late))
        self._last_reset = wire
        self._reset_tick = wire
        self.breaks += 1
        self.slot_count = 0
        self._open = DmxPacket(start_tick=wire, break_ticks=length)
        self.state = "mab"

    def _break_seen(self) -> None:
        """The space has lasted the break minimum, so it is a BREAK (table 7 designator 1).

        Anywhere: clause 9.1 asks a receiver to reject a badly framed slot and everything
        after it, so a BREAK in the middle of a packet ends that packet.
        """
        if self.state == "mab":
            self.double_breaks += 1        # a second BREAK before the START Code: clause 8.9
            if self._open is not None:
                self.packets.append(self._open)
                self._open = None
        self._break_start = (self._space_start if self._space_start is not None else 0)
        self.state = "break"

    def _open_slot(self, fall: int, rise: int) -> None:
        """The space was shorter than a break, so it was a slot: decode it from here.

        The bits it already covered are taken from the space just measured -- the line was a
        space over [fall, rise) -- and the rest are sampled as they come due.
        """
        if self._open is None:
            self.state = "idle"           # no reset sequence, so this is not a slot
            return
        if self.state == "mab":
            mab = fall - self._reset_tick
            self._open.mab_ticks = mab
            if mab < self.min["mab"]:
                # count it and carry on: a receiver that wants more mark than the
                # controller gave has nothing better to do than decode the slot anyway
                self.mab_errors += 1
        self.state = "slot"
        self._origin = fall
        self._past = (fall, rise)
        self._k = 0
        self._shift = 0
        self._framing = False
        self._sample_at = fall + 0.5 * self.ticks_per_bit + self.latency

    def _sample(self, level: int) -> None:
        """One bit, taken at the centre of its time, as a real receiver does."""
        k = self._k
        if k == 0:
            if level != SPACE:
                self.glitches += 1        # a start bit has to be a space (table 5)
                self.state = "inter"
                return
        elif k <= 8:
            self._shift |= level << (k - 1)   # LSB first (table 5)
        else:
            if level != MARK:
                self._framing = True      # clause 9.1: the stop bits have to be marks
        if k >= SLOT_BITS - 1:
            self._finish_slot()
        else:
            self._k = k + 1
            self._sample_at = self._origin + (k + 1.5) * self.ticks_per_bit + self.latency

    def _finish_slot(self) -> None:
        self._past = None
        if self._framing:
            # Clause 9.1: discard this slot and every slot after it in the packet.
            self.framing_errors += 1
            if self._open is not None:
                self._open.framing_errors += 1
            self.state = "drop"
            return
        pkt = self._open
        if pkt is None:
            self.state = "inter"
            return
        if pkt.start_code is None:
            pkt.start_code = self._shift           # slot 0 is the START Code (clause 8.5)
            pkt.null_start = self._shift == NULL_START_CODE
            self.start_code = self._shift
        else:
            pkt.slots += 1
            pkt.values += (self._shift,)
            self.slot_count = pkt.slots
            slot = pkt.slots                          # data slots are numbered 1..512
            if slot > self.min["max_slots"]:
                self.over_512 += 1
            elif self.address <= slot <= self.address + self.footprint - 1:
                self.levels[slot - 1] = self._shift
                self.slot_log.append((self._origin, slot, self._shift))
        self.state = "inter"


#: Every rule the checker enforces, and what the standard says about it. The keys are what
#: `Violation.rule` carries, so a firmware author can look a failure up here.
RULE_TEXT = {
    "BREAK": "a BREAK is a mark-to-space transition, a space of at least 88 us (92 us for a "
             "transmitter), then a space-to-mark transition; a shorter space is not a break "
             "(clause 8.3, definitions 3.3, tables 6 and 7 designator 1)",
    "MAB": "the mark after the break must last at least 8 us before the start bit of the "
           "START Code (12 us for a transmitter) (clause 8.4, tables 6 and 7 designator 2)",
    "MULTI-BREAK": "a transmitter shall not produce multiple BREAKs between data packets, "
                   "and every packet begins with a break, an MAB and a START Code "
                   "(clauses 8.1 and 8.9)",
    "BREAK-TO-BREAK": "the falling edge of one BREAK to the falling edge of the next must be "
                      "at least 1196 us apart (1204 us for a transmitter) and at most 1.25 s "
                      "(1.00 s for a transmitter); it is the minimum packet time, and it is "
                      "how a packet of fewer than 24 slots is still legal (clauses 8.7 and "
                      "8.10, tables 6 and 7 designators 11 and 13)",
    "SLOT-FORMAT": "a slot is a start bit, eight data bits LSB first and two stop bits; the "
                   "start bit is a space for a whole bit time (table 5, clause 8.2)",
    "BIT-RATE": "the bit rate must be between 245 and 255 kbit/s, a bit time between 3.92 "
                "and 4.08 us (tables 6 and 7)",
    "BIT-JITTER": "bit boundaries must sit on one grid: an interval that is not a whole "
                  "number of bit times to within the transmitter's own quantisation is "
                  "jitter the tables do not allow (tables 6 and 7, bit rate tolerance)",
    "STOP1": "a receiver shall check the first stop bit of every slot; a missing stop bit "
             "makes the slot improperly framed (clause 9.1, table 5)",
    "STOP2": "a receiver should check the second stop bit as well (clause 9.1, table 5)",
    "IDLE-MARK": "the line shall remain in a marking state between slots and between the "
                 "last slot and the next break (clauses 8.8 and 8.9, tables 6 and 7 "
                 "designators 9 and 10)",
    "MAX-SLOTS": "a link shall support up to 512 data slots, so slot 0 (the START Code) "
                 "plus 512 data slots is the whole universe (clauses 8.1 and 8.6)",
}

#: The BIT-JITTER budget in ticks, for reference: 2 ticks is 40 ns at 20 ns/tick, which is
#: what a transmitter with a fractional clock divider spends on rounding one bit up or down.
JITTER_TICKS = 2.0


@dataclass(frozen=True)
class Violation:
    """One rule the standard states and the wire did not meet, with its tick.

    `measured_ns` and `required_ns` are nanoseconds, because nanoseconds are the unit the
    standard's tables use; `required_ns` is the limit that was crossed, which for a
    two-sided rule like BIT-RATE is the bound on the far side of the legal window. Rules
    that are not about a duration (MAX-SLOTS, MULTI-BREAK) leave both at 0 and say what
    happened in `detail`.
    """

    tick: int
    rule: str
    measured_ns: float = 0.0
    required_ns: float = 0.0
    detail: str = ""

    def __str__(self) -> str:
        text = f"t={self.tick} {self.rule}"
        if self.measured_ns or self.required_ns:
            text += (f": {self.measured_ns / 1000:.3f} us against a limit of "
                     f"{self.required_ns / 1000:.3f} us")
        if self.detail:
            text += f" ({self.detail})"
        return text


@dataclass
class ObservedPacket:
    """One packet as the checker measured it, from the pins alone."""

    ns_per_tick: float
    break_tick: int                      # wire tick the BREAK's space began
    break_ticks: int = 0
    mab_ticks: Optional[int] = None      # None when no START Code followed the BREAK
    start_code: Optional[int] = None
    slots: int = 0                       # data slots decoded
    bit_ticks: Optional[float] = None    # the bit period it fitted, in ticks
    end_tick: Optional[int] = None       # wire tick the next BREAK's space began

    @property
    def bit_rate(self) -> Optional[float]:
        """The bit rate in bit/s that `bit_ticks` works out at, or None if unmeasurable."""
        if not self.bit_ticks:
            return None
        return 1.0e9 / (self.bit_ticks * self.ns_per_tick)

    def __str__(self) -> str:
        code = "none" if self.start_code is None else f"0x{self.start_code:02x}"
        rate = "unmeasurable" if self.bit_rate is None else f"{self.bit_rate / 1e3:.1f} kbit/s"
        return (f"t={self.break_tick} packet: break {self.break_ticks} ticks, MAB "
                f"{self.mab_ticks}, start code {code}, {self.slots} data slot(s), {rate}")


@dataclass
class _Open:
    """The packet the checker is in the middle of measuring."""

    break_tick: int
    break_end: int                      # the space-to-mark transition that ended the BREAK
    first_fall: Optional[int] = None    # the mark-to-space that ended the MAB, if it came
    edges: list = field(default_factory=list)    # [(wire tick, level)] from slot 0 on


class DmxChecker(Device):
    """Passive monitor: decodes the pins and flags every rule the standard states.

    It drives nothing, so it sees the line exactly as a protocol analyser would, and it
    judges a *transmitter*: `DmxTx` and firmware both meet the transmitter table by
    construction, while the limits it applies by default are the receiver's (table 7),
    which every compliant transmitter also meets. `strict_tx=True` applies table 6 instead,
    so a controller has to meet its own margins and not merely a receiver's tolerance.

    Checks, one rule each, every violation carrying the wire tick it happened on:

    ============= ====================================================================
    BREAK         a space shorter than the break minimum where a BREAK was expected
    MAB           a mark after the break shorter than the MAB minimum
    MULTI-BREAK   a second BREAK before the START Code (clause 8.9)
    BIT-RATE      a bit period outside 3.92 .. 4.08 us, i.e. outside 255 .. 245 kbit/s
    BIT-JITTER    a bit boundary off the packet's grid by more than `jitter_ticks`
    STOP1         a slot whose first stop bit is low (clause 9.1)
    STOP2         a slot whose second stop bit is low (clause 9.1)
    IDLE-MARK     the line left the mark state between slots or before a BREAK
    MAX-SLOTS     more than 512 data slots in one packet (clause 8.6)
    ============= ====================================================================

    It measures the bit period off the wire rather than assuming 250 kbit/s: within one
    packet every edge-to-edge interval inside a slot is a whole number of bit times, so
    the checker takes the shortest such interval, divides it by 1, 2, 3 ... up to 16, and
    keeps the candidates that put *every* observed edge on a grid of that period. Of those
    it takes the one inside the tables' 3.92 .. 4.08 us window; the tables make that choice
    unique, because twice or half a legal period is never legal. A further cross-check
    rejects any candidate that would make a slot shorter than its 11 bit times, which is
    what pins a packet whose data happens to have few edges. When no candidate falls in the
    window the wire is off spec and the candidate closest to the nominal period is reported,
    which names the right direction even when the data does not pin the value exactly.

    A packet is analysed when the next BREAK opens, and the last one by `finish()`. Call it
    at the end of a run, or `report()` and `violations` will be missing the last packet.
    A capture that starts with the line already low is measured from tick 0, and one that
    ends in a space is not analysed at all; both only ever make the checker stricter.
    """

    name = "dmx_check"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        profile: str = "receive",
        timing: Optional[dict] = None,
        net: str = DEFAULT_NET,
        strict_tx: bool = False,
        nominal_bit_rate: float = 250e3,
        jitter_ticks: float = JITTER_TICKS,
        latency: int = 1,
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.profile = "transmit" if strict_tx else profile
        self.timing_overrides = dict(timing or {})
        self.net = net
        self.strict_tx = bool(strict_tx)
        self.nominal_ticks = 1.0e9 / nominal_bit_rate / self.ns_per_tick
        self.jitter_ticks = float(jitter_ticks)
        self.latency = int(latency)
        if name:
            self.name = name
        self.min = dmx_minimums(self.ns_per_tick, self.profile, self.timing_overrides)
        self.violations: list[Violation] = []
        self.packets: list[ObservedPacket] = []
        self._open: Optional[_Open] = None
        self._reset()

    def bind(self, bus: Bus) -> "DmxChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = dmx_minimums(self.ns_per_tick, self.profile, self.timing_overrides)
        return self

    def _reset(self) -> None:
        self._last = MARK
        self._cand_fall: Optional[int] = None   # the fall whose space has not ended yet
        self._open: Optional[_Open] = None

    # -- what the tests ask for ------------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def report(self, limit: int = 12) -> str:
        head = (f"DMX512-A {self.min['profile']} table at {self.ns_per_tick} ns/tick: "
                f"{len(self.packets)} packet(s), "
                f"{sum(p.slots for p in self.packets)} data slot(s), "
                f"{len(self.violations)} violation(s); bit rate window "
                f"{self.min['bit_rate_max'] / 1e3:.0f} .. {self.min['bit_rate_min'] / 1e3:.0f} "
                f"kbit/s")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, tick: int, rule: str, measured_ns: float = 0.0,
                   required_ns: float = 0.0, detail: str = "") -> None:
        self.violations.append(Violation(tick, rule, measured_ns, required_ns, detail))

    def ns(self, ticks: float) -> float:
        """Ticks to the nanoseconds the standard's tables are written in."""
        return ticks * self.ns_per_tick

    # -- edges -----------------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        level = pins.get(self.net, MARK)
        prev = self._last
        self._last = level
        if level != prev:
            self._edge(t - self.latency, level)
        return {}

    def _edge(self, t: int, level: int) -> None:
        if level == SPACE:
            self._cand_fall = t
        else:
            self._rise(t)

    def _rise(self, t: int) -> None:
        """Space to mark: the end of a space, and the only place the checker decides.

        A space at least as long as the break minimum is a BREAK wherever it appears
        (table 7 designator 1), so that is the first question. Any shorter space is either a
        data transition inside a slot, the start bit of the next one, or a break that never
        got long enough -- and the packet the checker has open says which.
        """
        fall, self._cand_fall = self._cand_fall, None
        if fall is None:
            return                            # the line was already mark when the capture began
        if t - fall >= self.min["break"]:
            self._on_break(fall, t)
            return
        self._on_short_space(fall, t)

    def _on_break(self, fall: int, rise: int) -> None:
        """A space of at least the break minimum: a BREAK, wherever it fell."""
        previous = (self._open.break_tick if self._open is not None
                    else self.packets[-1].break_tick if self.packets else None)
        if previous is not None:
            self._check_break_to_break(fall, previous)
        if self._open is not None:
            if self._open.first_fall is None:
                self._violation(fall, "MULTI-BREAK",
                                detail="a second BREAK arrived before the START Code; a packet "
                                       "is one reset sequence (break, MAB, START Code) followed "
                                       "by its slots")
            self._analyse(self._open, end_tick=fall)
        self._open = _Open(break_tick=fall, break_end=rise)

    def _on_short_space(self, fall: int, rise: int) -> None:
        """A space shorter than the break minimum: a bit, or a break that was too short."""
        pkt = self._open
        if pkt is None:
            self._violation(rise, "BREAK", self.ns(rise - fall), self.ns(self.min["break"]),
                            "space too short to be a BREAK, where a packet had to begin")
            return
        if pkt.first_fall is None:
            mab = fall - pkt.break_end
            if mab < self.min["mab"]:
                self._violation(fall, "MAB", self.ns(mab), self.ns(self.min["mab"]),
                                "the mark after the break ended before the START Code's "
                                "start bit")
            pkt.first_fall = fall
            pkt.edges = [(fall, SPACE), (rise, MARK)]
            return
        pkt.edges.append((fall, SPACE))
        pkt.edges.append((rise, MARK))

    def _check_break_to_break(self, tick: int, previous: int) -> None:
        """Tables 6 and 7, designators 11 and 13: both bounds, on the falling edges."""
        gap = tick - previous
        if gap < self.min["b2b_min"]:
            self._violation(tick, "BREAK-TO-BREAK", self.ns(gap), self.ns(self.min["b2b_min"]),
                            "BREAK to BREAK is shorter than the tables allow")
        elif gap > self.min["b2b_max"]:
            self._violation(tick, "BREAK-TO-BREAK", self.ns(gap), self.ns(self.min["b2b_max"]),
                            "BREAK to BREAK is longer than the tables allow")

    # -- measuring one packet ---------------------------------------------------------
    def finish(self) -> Optional[ObservedPacket]:
        """Analyse the packet still open, so the last packet of a run is checked too.

        Every other packet is analysed the moment the next BREAK opens, which is the only
        moment a packet is known to be over. Call this once at the end of a run; without it
        the last packet's slots are never decoded.
        """
        pkt, self._open = self._open, None
        return None if pkt is None else self._analyse(pkt)

    def _analyse(self, pkt: _Open, end_tick: Optional[int] = None) -> ObservedPacket:
        obs = ObservedPacket(
            self.ns_per_tick,
            break_tick=pkt.break_tick,
            break_ticks=pkt.break_end - pkt.break_tick,
            mab_ticks=None if pkt.first_fall is None else pkt.first_fall - pkt.break_end,
            end_tick=end_tick,
        )
        falls = [i for i, (_t, lvl) in enumerate(pkt.edges) if lvl == SPACE]
        if falls:
            bit = self._fit_bit_ticks(pkt)
            obs.bit_ticks = bit
            self._decode(pkt, obs, self._slot_starts(pkt, falls, bit), bit)
        self.packets.append(obs)
        return obs

    def _fit_bit_ticks(self, pkt: _Open) -> float:
        """The bit period this packet ran at, in ticks, measured from its edges alone.

        Inside a slot the line only changes on bit boundaries, so every interval between two
        consecutive edges is a whole number of bit times (an interval that spans the gap
        between two slots is the one exception, and it can only ever be longer than one). So
        a period is found by guessing: take the shortest observed intervals, divide each of
        them by 1, 2, 3 ... up to twelve, and score every guess by how many of the packet's
        intervals it puts on its grid. The true period explains all of them; half of it
        misses every odd one, twice it misses every odd one, and the gap between slots is
        explained by nothing in particular. Among the guesses that explain the most, the one
        inside the tables' 3.92 .. 4.08 us window wins, and the tables make that unique,
        because twice or half a legal period is never legal. With no in-band guess left the
        wire is off spec, and the guess closest to the nominal period is reported, which
        names the right direction even where the data does not pin the value exactly.
        """
        edges = pkt.edges
        gaps = [edges[j + 1][0] - edges[j][0] for j in range(len(edges) - 1)]
        gaps = [g for g in gaps if g > 0]
        if not gaps:                        # a slot always has one edge, so this is rare
            return self.nominal_ticks
        tol = max(2.0, self.jitter_ticks)
        ordered = sorted(gaps)
        seeds = {ordered[len(ordered) // 2]}        # the median: one short glitch cannot move it
        seeds |= set(ordered[:4])                    # and the shortest ones, which usually are
        guesses = {g / k for g in seeds for k in range(1, 17)}
        # A candidate inside the tables' window beats any candidate outside it, however many
        # intervals it explains: a stray space between two slots costs the true period one
        # interval, and a period derived from that stray space is out of spec by definition.
        pool = [c for c in guesses if self._bit_in_window(c)] or list(guesses)
        hits = lambda cand: sum(1 for x in gaps
                                if abs(x - round(x / cand) * cand) <= tol)   # noqa: E731
        return max(pool, key=lambda c: (hits(c), c))

    def _bit_in_window(self, bit: float, slack: float = 0.5) -> bool:
        """Is a bit period inside the tables' 3.92 .. 4.08 us, to within half a tick?

        A measurement is a whole number of ticks and a transmitter rounds each bit up or
        down, so the comparison allows half a tick of quantisation -- the largest error a
        whole-tick measurement of a whole number of bit times can have. At 20 ns/tick the
        window is 196.08 .. 204.08 ticks, and a transmitter running at exactly 255 kbit/s
        (196.08 ticks) or exactly 245 kbit/s (204.08 ticks) is legal, not a violation.

        `slack` is that allowance, and one *measured bit* needs twice the fitted period's: a
        transmitter with a fractional clock divider rounds each bit up or down, so at 245
        kbit/s a single bit can be 205 ticks long where the period is 204.08, while the
        fitted period is 204. A consequence worth knowing: at 20 ns/tick one tick is 2.5
        kbit/s of bit rate, so no checker on this bus can judge the rate more finely than
        that.
        """
        return (self.min["min_bit_ticks"] - slack <= bit
                <= self.min["max_bit_ticks"] + slack)

    def _slot_starts(self, pkt: _Open, falls: list, bit: float) -> list:
        """Which of the falls in `pkt` are slot start bits, on the fitted grid.

        Every fall is a candidate -- a start bit, or a data bit that follows a one -- and two
        things tell them apart. Position: slot k + 1 cannot begin before slot k's eleventh
        bit is over, and the uncertainty in "eleven bit times" is eleven times the
        uncertainty in the period itself. And shape: a start bit is a space a mid-bit sample
        can see, and the slot goes on changing after it, while a stray space between two slots
        is followed by a long mark. A start bit's own space is *not* one bit time -- it is as
        long as the run of low data bits that follow it, up to nine of them -- so only the two
        tests above will do. A fall that is neither a start bit nor inside any slot is left
        over for the IDLE-MARK rule.
        """
        edges = pkt.edges
        tol = SLOT_BITS * max(2.0, self.jitter_ticks)
        starts = [falls[0]]         # the first fall after the MAB always opens slot 0
        while True:
            due = edges[starts[-1]][0] + SLOT_BITS * bit
            due_falls = [i for i in falls if i > starts[-1] and edges[i][0] >= due - tol]
            if not due_falls:
                return starts
            starts.append(next((i for i in due_falls
                                if self._is_start_bit(edges, i, bit)), due_falls[0]))

    def _is_start_bit(self, edges: list, i: int, bit: float) -> bool:
        """Does the space at edge `i` look like a slot's start bit?"""
        space = self._space_of(edges, i)
        if space < 0.5 * bit:                      # no mid-bit sample could see it
            return False
        return space <= SLOT_BITS * bit or space == float("inf")

    @staticmethod
    def _space_of(edges: list, i: int) -> float:
        """How long the space that starts at edge `i` lasted, from the edges alone."""
        if i + 1 >= len(edges):
            return float("inf")     # the space has not ended inside the packet
        return edges[i + 1][0] - edges[i][0]

    def _decode(self, pkt: _Open, obs: ObservedPacket, starts: list, bit: float) -> None:
        """Walk the slots on the fitted grid: framing first, then the timing rules.

        Clause 9.1 governs the walk: the first badly framed slot ends the decoding of that
        packet, so one missing stop bit does not turn into a page of violations. The timing
        rules are not affected by that and run for every slot, because they are measured from
        edge positions rather than from the decoded bytes. A packet whose bit period is
        outside the tables gets one BIT-RATE for the packet and no per-gap reports, since the
        grid it was fitted on is only a guess there.
        """
        edges = pkt.edges
        origins = [edges[i][0] for i in starts]
        low, high = self.min["min_bit_ticks"], self.min["max_bit_ticks"]
        tol = max(2.0, self.jitter_ticks)
        if not self._bit_in_window(bit):
            rate = 1.0e9 / self.ns(bit)
            self._violation(origins[0], "BIT-RATE", self.ns(bit),
                            self.min["bit_time_min_ns"] if bit < low
                            else self.min["bit_time_max_ns"],
                            "the packet's bit period is %.1f kbit/s, the tables allow %.0f .. "
                            "%.0f kbit/s" % (rate / 1e3, self.min["bit_rate_max"] / 1e3,
                                              self.min["bit_rate_min"] / 1e3))
        on_rate = self._bit_in_window(bit)
        decoding = True
        for n, i0 in enumerate(starts):
            i1 = starts[n + 1] if n + 1 < len(starts) else len(edges)
            if decoding:
                bits = [self._level_at(edges, origins[n] + (i + 0.5) * bit)
                        for i in range(SLOT_BITS)]
                value = sum(bits[1 + i] << i for i in range(8))
                if bits[0] != SPACE:
                    self._violation(origins[n], "SLOT-FORMAT",
                                    detail="slot %d's start bit was high at mid-bit" % n)
                    decoding = False
                for bit_index, rule in ((9, "STOP1"), (10, "STOP2")):
                    if bits[bit_index] != MARK:
                        self._violation(origins[n], rule,
                                        detail="slot %d's %s stop bit is low, so the slot is "
                                               "improperly framed"
                                               % (n, "first" if bit_index == 9 else "second"))
                        decoding = False
                if n == 0:
                    obs.start_code = value
                else:
                    obs.slots += 1
                    if obs.slots > self.min["max_slots"]:
                        self._violation(origins[n], "MAX-SLOTS",
                                        detail="data slot %d, but a link supports %d"
                                               % (obs.slots, self.min["max_slots"]))
                if not decoding:
                    continue                # clause 9.1: stop at the first bad slot
            # timing rules for this slot, from its own edges
            for j in range(i0, i1 - 1):
                gap = edges[j + 1][0] - edges[j][0]
                if gap <= 0:
                    continue
                span = max(1, int(round(gap / bit)))
                implied = gap / span
                if not on_rate:
                    continue           # the packet is off-rate already: see the BIT-RATE above
                if not self._bit_in_window(implied, slack=1.0):
                    self._violation(edges[j][0], "BIT-RATE", self.ns(implied),
                                    self.min["bit_time_min_ns"] if implied < low
                                    else self.min["bit_time_max_ns"],
                                    "%d bit times of this slot measure %.3f us"
                                    % (span, self.ns(implied) / 1000))
                elif abs(gap - span * bit) > tol:
                    self._violation(edges[j][0], "BIT-JITTER", self.ns(gap),
                                    self.ns(span * bit),
                                    "%d bit times of this slot are %.3f us, not %.3f us"
                                    % (span, self.ns(gap) / 1000, self.ns(span * bit) / 1000))
        for t, lvl in edges:                # the line shall remain a mark between slots (8.8)
            if lvl == SPACE and not any(o <= t < o + SLOT_BITS * bit for o in origins):
                self._violation(t, "IDLE-MARK",
                                detail="the line left the mark state between slots")

    @staticmethod
    def _level_at(edges: list, t: float) -> int:
        """The level on the line at tick `t`, from the edges seen up to there."""
        level = MARK
        for edge_tick, edge_level in edges:
            if edge_tick <= t:
                level = edge_level
            else:
                break
        return level


# ---- harness helpers ---------------------------------------------------------------
def _ensure_net(bus: Bus, net: str = DEFAULT_NET) -> None:
    """Add the DMX512 net if it is not there yet, push-pull as EIA-485 is."""
    if net not in bus.nets:
        bus.add_net(net, push_pull=True)


def attach_tx(bus: Bus, net: str = DEFAULT_NET, **kwargs) -> DmxTx:
    """Build a DmxTx that takes its timing from `bus` and attach it."""
    _ensure_net(bus, net)
    tx = DmxTx(bus=bus, net=net, **kwargs)
    bus.attach(tx)
    return tx


def attach_rx(bus: Bus, net: str = DEFAULT_NET, **kwargs) -> DmxRx:
    """Build a DmxRx that takes its timing from `bus` and attach it."""
    _ensure_net(bus, net)
    rx = DmxRx(bus=bus, net=net, **kwargs)
    bus.attach(rx)
    return rx


def attach_checker(bus: Bus, net: str = DEFAULT_NET, **kwargs) -> DmxChecker:
    """Build a passive DmxChecker for `bus` and attach it."""
    _ensure_net(bus, net)
    chk = DmxChecker(bus=bus, net=net, **kwargs)
    bus.attach(chk)
    return chk

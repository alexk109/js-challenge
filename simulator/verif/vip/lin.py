"""Pin-level LIN 2.x reference models on the tick-based bus in verif/vip/core.py.

One wire, one net named "lin", logic level only (no transceiver): a node drives 0 for
**dominant** and releases for **recessive**, and the bus reads the pull-up otherwise. That
is the LIN physical convention ("Bit values on the bus are recessive (logical high) or
dominant (logical low)", lin-cia.org/en/technology) and it fits the core's open-drain
rule, so two nodes may drive at once and `bus.errors` can never fire.

Three devices:

  LinCommander       the LIN commander (LIN 1.x called it the master). Queued frames; for
                     each one it emits the break field, the break delimiter, the 0x55 sync
                     byte and the protected identifier, then either publishes the response
                     itself or leaves the wire to a responder. Every space is a tick count
                     derived from the bit rate, so the wire is bit-exact at 2.4, 9.6, 10,
                     19.2 or 20 kbit/s.
  LinResponder       the LIN responder (LIN 1.x: slave). Detects the break with the spec's
                     11-dominant-bit threshold, measures its bit rate from the falling edges
                     of the sync field, decodes the protected identifier, checks the parity
                     bits, and publishes the bytes subscribed to that identifier with the
                     checksum its own LIN version calls for.
  LinChecker         passive monitor. Decodes the wire and checks 15 rules, appending every
                     miss to `violations` with the tick it was missed on.

Typical use:

    bus = Bus(ns_per_tick=20)                       # 50 MHz
    ctl = attach_commander(bus, rate=19_200)        # bit period = 2604 ticks
    rsp = attach_responder(bus, rate=19_200, publish={0x11: b"\\xa5\\x5a"})
    chk = attach_checker(bus, rate=19_200, lengths={0x11: 2})
    ctl.queue_frame(0x11)
    ctl.run(bus, limit=2_000_000)
    assert chk.clean, chk.report()
    assert rsp.published[0][1] == b"\\xa5\\x5a"

Frame shape, all of it from LIN 2.2A §2.3.1 (see research/ref/protocols/lin/):

    | break >= 13 bit times dominant | delimiter >= 1 bit time | space |
    | 0x55 | space | PID = ID | P0 | P1 | stop |     (the header: 34 bit times)   |
    | response space | 1..8 data bytes | checksum byte |   (the response)            |

Every byte field except the break is start bit, eight data bits **LSB first**, stop bit --
"one additional start bit, one additional stop-bit, but no parity bit" (lin-cia.org).

    P0 = ID0 ^ ID1 ^ ID2 ^ ID4
    P1 = not(ID1 ^ ID3 ^ ID4 ^ ID5)          (LIN 2.2A eq. (1) and (2))

so `make_pid(id) == id | P0 << 6 | P1 << 7`: ID 0x00 is PID 0x80, ID 0x02 is PID 0x40.

    classic  checksum = ~(sum(data))                    LIN 1.x, and 0x3C/0x3D always
    enhanced checksum = ~(sum(data) + PID)             every other frame from LIN 2.0

both modulo 256 with the carry wrapped back in, and both checked by the same property: the
sum of every byte of the response, checksum included, is 0xFF.

Every number in this file is cited in research/ref/protocols/lin/README.md, which also
lists the judgement calls. The four that matter most to a firmware author:

* The spaces (`response_space_bits`, `ibs_bits`) are floors of **zero** in the spec ("Both
  of them must be non-negative"), so the checker enforces them only if you raise them. The
  40 % reserve is a *ceiling*, checked as `tHEADER_MAX` / `tRESPONSE_MAX`, never slack in
  the minima.
* Frame length is not on the wire. Give the checker the LDF (`lengths=`), or let it read
  the frame's end from the idle gap a real commander leaves.
* Bit periods are not whole ticks at 19.2 or 9.6 kbit/s; `bit_ticks()` rounds to the
  nearest tick, which is at most 0.01 % of a bit, far inside the +/-1.5 % post-sync
  tolerance.
* The wire is modelled open-drain, so a collision between two transmitters is
  indistinguishable from normal transmission and is **not** flagged.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .core import Bus, Device

#: The bit rates the models are built for, in bit/s. 2400 is the low end of the
#: auto-baud-range a LIN 2.x cluster uses before a responder has measured the sync field
#: ("using baud rates between 2.4 kbit/s and 20 kbit/s", Infineon TLD4020-3ET data sheet);
#: 20000 is the protocol's own ceiling ("speed up to 20 kbit/s", LIN 2.2A clause 1.1).
LIN_RATES: tuple[float, ...] = (2400.0, 9600.0, 10000.0, 19200.0, 20000.0)

#: The published 2/4/8-byte response lengths per frame-identifier range (Microchip's LIN
#: data link layer page). Offered as `lengths="table"`; a real cluster's LDF wins over it.
LENGTH_TABLE: dict[int, int] = {**{i: 2 for i in range(0x00, 0x20)},
                                **{i: 4 for i in range(0x20, 0x30)},
                                **{i: 8 for i in range(0x30, 0x40)}}

#: Frame identifiers 0x3C and 0x3D carry diagnostic and configuration data and "shall
#: always use classic checksum" (LIN 2.2A clause 2.3.1.5).
DIAGNOSTIC_IDS = (0x3C, 0x3D)


@dataclass(frozen=True)
class LinLimits:
    """One LIN cluster version: every rule number, in bit times.

    Everything here is in **nominal bit times**, not ticks, so the same table serves any
    `ns_per_tick`. `bit_ticks()` turns the bit rate into ticks and `lin_limits()` turns
    this table into tick counts.

    The two floors that have a spec minimum are `break_bits` (13) and `delimiter_bits`
    (1). `ibs_bits` and `response_space_bits` are 0 because the spec's only statement about
    the spaces is that they "must be non-negative" -- they are here so a caller can raise
    them, and the models' defaults sit inside whatever a caller asks for.
    """

    version: str
    break_bits: float = 13.0             # dominant part of the break field, minimum
    delimiter_bits: float = 1.0          # recessive part of the break field, minimum
    break_detect_bits: float = 11.0      # responder break-detection threshold
    ibs_bits: float = 0.0                # inter-byte space, minimum
    response_space_bits: float = 0.0     # response space, minimum
    ftol_unsync: float = 0.14            # FTOL_UNSYNC, before the sync field
    ftol_sync: float = 0.015             # FTOL_SYNC, after the sync field
    header_slack: float = 1.4            # THeader_Maximum = 1.4 * THeader_Nominal
    response_slack: float = 1.4          # TResponse_Maximum = 1.4 * TResponse_Nominal
    min_data: int = 1                    # "four to eleven byte fields"
    max_data: int = 8                    # "one to eight bytes"
    enhanced_checksum: bool = True       # False: a LIN 1.3 cluster


#: LIN 2.2 is the version supported; 1.3 exists so a test can show that a
#: classic-checksum bench and an enhanced-checksum bench are told apart on the wire.
LIN_LIMITS: dict[str, LinLimits] = {
    "2.2": LinLimits("2.2"),
    "1.3": LinLimits("1.3", enhanced_checksum=False),
}

#: The bits of one response byte plus its start and stop bits, and of the header's sync and
#: PID bytes (LIN 2.2A eq. (3): 34 = 13 break + 1 delimiter + 10 sync + 10 PID).
BYTE_BITS = 10
BREAK_FIELD_BITS = 14                   # 13 dominant + 1 delimiter, "nominal break field"
HEADER_BITS = 34                        # THeader_Nominal / TBit
SYNC_BYTE = 0x55


def bit_ticks(ns_per_tick: float, rate: float) -> int:
    """One bit time in whole ticks, rounded to the nearest tick.

    LIN's bit rates do not divide a 20 ns tick: 2500 ticks at 20 kbit/s, 2604.17 at
    19.2, 5208.3 at 9.6, 20833.3 at 2.4. Rounding to nearest leaves at most 0.01 % of a bit
    at every rate in `LIN_RATES`, which is two orders of magnitude inside the +/-1.5 %
    post-sync tolerance, so the rounding can never be the cause of a `tBIT` violation.
    """
    if not rate > 0:
        raise ValueError(f"the LIN bit rate must be positive, got {rate!r}")
    return max(1, int(round(1e9 / rate / ns_per_tick)))


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up (floors only)."""
    return max(0, int(-(-max(0.0, ns) // ns_per_tick)))


def _bit_floor(bits: float, tick: int, minimum: int = 1) -> int:
    """A minimum in bit times as whole ticks, rounded up so it is never lenient.

    `minimum` is 0 for the two spaces, whose spec floor is zero bit times: rounding that up
    to a tick would put a one-tick gap on the wire that the spec never asked for, and a
    monitor reading an LDF-free frame would see an inter-byte space that is not there.
    """
    return max(minimum, int(-(-bits * tick // 1)))


def _bit_ceiling(bits: float, tick: int) -> int:
    """A maximum in bit times as whole ticks, rounded down so it is never lenient."""
    return max(1, int(bits * tick))


def lin_limits(
    ns_per_tick: float,
    rate: float,
    version: str = "2.2",
    overrides: Optional[dict] = None,
) -> dict:
    """Every rule for `rate` on a `version` cluster, as whole ticks.

    Keys that are times are tick counts; `bit_ticks`, `rate` and the counts are not. Floors
    are rounded up and ceilings down, so a quantised limit is at worst a hair stricter than
    the exact spec value and never more lenient than it.

    `overrides` replaces individual values, and is how a test drives a model or the
    checker off-spec on purpose: `{"break_bits": 9}` means "a break of 9 bit times is good
    enough here", `{"ftol_sync": 0.0}` means "the checker will not forgive any bit-period
    drift at all". The one override that is not a time is "rate", the bit rate in bit/s.

    Derived keys the checker uses: `bit_min` / `bit_max` (the post-sync period window),
    `bit_min_unsync` / `bit_max_unsync` (the pre-sync one), `header_min_ticks` /
    `header_max_ticks`, `byte_ticks`, and the helpers `frame_min_ticks()` and
    `response_max_ticks()` for the two frame-length bounds.
    """
    try:
        base = LIN_LIMITS[version]
    except KeyError:
        raise ValueError(
            f"unknown LIN version {version!r}; pick one of {sorted(LIN_LIMITS)}"
        ) from None
    vals = {
        "break_bits": base.break_bits,
        "delimiter_bits": base.delimiter_bits,
        "break_detect_bits": base.break_detect_bits,
        "ibs_bits": base.ibs_bits,
        "response_space_bits": base.response_space_bits,
        "ftol_unsync": base.ftol_unsync,
        "ftol_sync": base.ftol_sync,
        "header_slack": base.header_slack,
        "response_slack": base.response_slack,
        "min_data": base.min_data,
        "max_data": base.max_data,
    }
    overrides = dict(overrides or {})
    rate = float(overrides.pop("rate", rate))
    for key, value in overrides.items():
        if key not in vals:
            raise ValueError(
                f"unknown timing override {key!r}; pick from {sorted(vals) + ['rate']}"
            )
        vals[key] = value
    if vals["min_data"] < 0 or vals["max_data"] < vals["min_data"]:
        raise ValueError(f"nonsense data-byte bounds {vals['min_data']}..{vals['max_data']}")
    tick = bit_ticks(ns_per_tick, rate)
    out = {
        "rate": rate,
        "ns_per_tick": ns_per_tick,
        "version": version,
        "bit_ticks": tick,
        "bit_ns": ns_per_tick * tick,
        "break_ticks": _bit_floor(vals["break_bits"], tick),
        "delimiter_ticks": _bit_floor(vals["delimiter_bits"], tick),
        "break_detect_ticks": _bit_floor(vals["break_detect_bits"], tick),
        "ibs_ticks": _bit_floor(vals["ibs_bits"], tick, minimum=0),
        "response_space_ticks": _bit_floor(vals["response_space_bits"], tick, minimum=0),
        "bit_min_unsync": max(1, int((1.0 - vals["ftol_unsync"]) * tick + 0.999)),
        "bit_max_unsync": max(1, int((1.0 + vals["ftol_unsync"]) * tick)),
        "bit_min": max(1, int((1.0 - vals["ftol_sync"]) * tick + 0.999)),
        "bit_max": max(1, int((1.0 + vals["ftol_sync"]) * tick)),
        "header_min_ticks": HEADER_BITS * tick,
        "header_max_ticks": max(1, int(vals["header_slack"] * HEADER_BITS * tick)),
        "byte_ticks": BYTE_BITS * tick,
        # How long the dominant run that opens a byte field can be before it cannot be one:
        # its start bit, its eight data bits, and a stop bit that a broken frame left
        # dominant. A longer run is a break field (13 bit times) or neither (11), so this is
        # what tells the two apart on the wire.
        "byte_run_limit": BYTE_BITS * tick,
        # A byte field is ten bit times long, so the bit-period tolerance becomes a tolerance
        # on a byte field's whole duration. That is what makes tBIT measurable on the wire: the
        # grid does not move when a level is held for two or three bit times, so the length of
        # a byte field -- which always spans exactly ten of them -- is what carries the drift.
        "byte_min_ticks": max(1, int(BYTE_BITS * tick * (1.0 - vals["ftol_sync"]) + 0.999)),
        "byte_max_ticks": max(1, int(BYTE_BITS * tick * (1.0 + vals["ftol_sync"]))),
        "byte_min_ticks_unsync": max(
            1, int(BYTE_BITS * tick * (1.0 - vals["ftol_unsync"]) + 0.999)),
        "byte_max_ticks_unsync": max(
            1, int(BYTE_BITS * tick * (1.0 + vals["ftol_unsync"]))),
        "ftol_unsync": vals["ftol_unsync"],
        "ftol_sync": vals["ftol_sync"],
        "header_slack": vals["header_slack"],
        "response_slack": vals["response_slack"],
        "min_data": int(vals["min_data"]),
        "max_data": int(vals["max_data"]),
        "enhanced_checksum": base.enhanced_checksum,
    }
    return out


def frame_min_ticks(limits: dict, frame_n: int) -> int:
    """`T_Frame_Nominal` in ticks: 34 + 10 (N + 1) bit times (LIN 2.2A eq. (3)–(5)).

    A frame that is shorter than this could not have been sent at the nominal bit rate, so
    the checker reports it as `tFRAME_MIN` -- the wire-level statement of "all subscribing
    nodes shall be able to receive a frame that has a zero overhead".
    """
    return (HEADER_BITS + BYTE_BITS * (frame_n + 1)) * limits["bit_ticks"]


def response_max_ticks(limits: dict, frame_n: int) -> int:
    """`T_Response_Maximum` in ticks: 1.4 * 10 (N + 1) bit times (LIN 2.2A eq. (7))."""
    return max(1, int(limits["response_slack"] * BYTE_BITS * (frame_n + 1)
                      * limits["bit_ticks"]))


# ---- protected identifier and checksum --------------------------------------
def parity_bits(frame_id: int) -> tuple[int, int]:
    """The PID's two parity bits, from LIN 2.2A eq. (1) and (2).

        P0 = ID0 ^ ID1 ^ ID2 ^ ID4
        P1 = not(ID1 ^ ID3 ^ ID4 ^ ID5)

    P0 is the even-parity bit and P1 the odd-parity one (Vector, "Introduction to LIN").
    """
    if not 0 <= frame_id <= 0x3F:
        raise ValueError(f"a LIN frame identifier is 6 bits, got {frame_id:#x}")
    p0 = ((frame_id >> 0) ^ (frame_id >> 1) ^ (frame_id >> 2) ^ (frame_id >> 4)) & 1
    p1 = 1 ^ (((frame_id >> 1) ^ (frame_id >> 3) ^ (frame_id >> 4) ^ (frame_id >> 5)) & 1)
    return p0, p1


def make_pid(frame_id: int) -> int:
    """The protected identifier byte for `frame_id`: the ID with P0 and P1 in bits 7 and 6.

    "Bits 0 to 5 are the frame identifier and bits 6 and 7 are the parity" (LIN 2.2A
    clause 2.3.1.3), so ID 0x00 is PID 0x80 and ID 0x02 is PID 0x40.
    """
    p0, p1 = parity_bits(frame_id)
    return frame_id | (p0 << 6) | (p1 << 7)


def frame_id_of(pid: int) -> int:
    """The frame identifier inside a protected identifier byte (bits 0 to 5)."""
    return pid & 0x3F


def pid_parity_ok(pid: int) -> bool:
    """True if P0 and P1 in `pid` are the parity of the identifier it carries."""
    return (pid & 0xFF) == make_pid(frame_id_of(pid & 0xFF))


def checksum_model(frame_id: int, version: str = "2.2") -> str:
    """Which checksum model a frame identifier uses on a `version` cluster.

    Diagnostic and configuration frames (0x3C, 0x3D) "shall always use classic checksum"
    (LIN 2.2A clause 2.3.1.5). Every other frame uses the enhanced model from LIN 2.0 on,
    and on a LIN 1.3 cluster everything is classic.
    """
    if frame_id in DIAGNOSTIC_IDS or LIN_LIMITS[version].enhanced_checksum is False:
        return "classic"
    return "enhanced"


def lin_checksum(pid: int, data, enhanced: bool = True) -> int:
    """The checksum byte: the inverted modulo-256 sum of the data bytes (and the PID).

    "To form the checksum, the individual data bytes are added by modulo-256 arithmetic.
    This involves adding overrun bits to the specific intermediate result. Finally, the
    overall result is inverted" (Vector, "Introduction to LIN"). `enhanced` adds the
    protected identifier to the sum, which is the whole difference between the two models.
    """
    total = sum(data) & 0xFF
    if enhanced:
        total = (total + (pid & 0xFF)) & 0xFF
    return (~total) & 0xFF


def checksum_ok(pid: int, data, received: int, enhanced: bool = True) -> bool:
    """The receiver's test: the sum of every byte, checksum included, is 0xFF.

    "The receiver detects a transmission error if the sum of the data bytes and the
    arriving checksum is not 0xFF" (Vector). One property checks both models and the
    diagnostic frames' mandatory classic checksum.
    """
    total = sum(data) + (pid & 0xFF if enhanced else 0) + (received & 0xFF)
    return (total & 0xFF) == 0xFF


# ---- waveforms --------------------------------------------------------------
def byte_segments(value: int, bit_ticks: int, *, start: int = 0, stop: int = 1,
                   bit_ticks_override: Optional[list] = None) -> list[tuple[int, int]]:
    """One LIN byte field as `(ticks, level)` segments: start, 8 data bits, stop.

    Data bits go out LSB first -- "one additional start bit, one additional stop-bit, but
    no parity bit" (lin-cia.org/en/technology). `start` and `stop` are constructor-level
    knobs so a test can put a framing error on the wire, and `bit_ticks_override` gives
    per-bit durations (a list of 8) so a test can stretch one bit.
    """
    segs: list[tuple[int, int]] = [(bit_ticks, start)]
    for i in range(8):
        width = bit_ticks if bit_ticks_override is None else bit_ticks_override[i]
        segs.append((max(1, int(width)), (value >> i) & 1))
    segs.append((bit_ticks, stop))
    return segs


def frame_segments(
    *,
    bit_ticks: int,
    break_ticks: Optional[int] = None,
    delimiter_ticks: Optional[int] = None,
    ibs_ticks: int = 0,
    sync: int = SYNC_BYTE,
    pid: Optional[int] = None,
    data=(),
    checksum: Optional[int] = None,
    response_space_ticks: int = 0,
    response_ibs_ticks: Optional[int] = None,
    start_bit: int = 0,
    stop_bit: int = 1,
    bit_ticks_override: Optional[list] = None,
) -> list[tuple[int, int]]:
    """One whole LIN frame as `(ticks, level)` segments, break field to checksum.

    This is the single place a frame is built: `LinCommander` calls it for every frame it
    queues, and `LinRawTransmitter` plus the fault-injection tests call it with a knob
    turned so the wire comes out wrong. Every argument has a legal default, so
    `frame_segments(bit_ticks=2500, pid=make_pid(0x11))` is already a legal header-only
    frame.

    `break_ticks` defaults to 13 bit times and `delimiter_ticks` to 1, and both spaces to 0
    -- which makes the frame exactly `T_Frame_Nominal` long when a response is present, the
    length "all subscribing nodes shall be able to receive" (LIN 1.3 clause 2.2, quoted
    in research/ref/protocols/lin/06-ni-lin-bus-timing.md). Every time argument here is a
    tick count, not a bit time: pass `bit_ticks * n` for n bit times.
    """
    if break_ticks is None:
        break_ticks = 13 * bit_ticks
    if delimiter_ticks is None:
        delimiter_ticks = bit_ticks
    if ibs_ticks < 0 or delimiter_ticks < 0 or break_ticks < 0:
        raise ValueError("a LIN frame has no negative field lengths")
    r_ibs = ibs_ticks if response_ibs_ticks is None else response_ibs_ticks
    segs: list[tuple[int, int]] = [(int(break_ticks), 0), (int(delimiter_ticks), 1)]
    if ibs_ticks:
        segs.append((int(ibs_ticks), 1))
    segs += byte_segments(sync, bit_ticks, start=start_bit, stop=stop_bit)
    if pid is not None:
        if ibs_ticks:
            segs.append((int(ibs_ticks), 1))
        segs += byte_segments(pid, bit_ticks, start=start_bit, stop=stop_bit)
    if response_space_ticks:
        segs.append((int(response_space_ticks), 1))
    if data or checksum is not None:
        segs += response_segments(data, checksum, bit_ticks, r_ibs,
                                  start_bit=start_bit, stop_bit=stop_bit,
                                  bit_ticks_override=bit_ticks_override)
    return segs


class _SegmentPlayer(Device):
    """Shared bit-level driver: walks a flat list of `(ticks, level)` segments.

    Dominant is driven (0) and recessive is released (`None`), which is what the core's
    open-drain resolution turns into the LIN wire. It reacts a tick after the bus resolves,
    like every other device on this bus, so the waveform a player puts out is shifted one
    tick from its own intent -- uniformly, for every player, and therefore invisible to any
    limit that is a whole number of bit times.

    `label`s let a caller ask when a particular segment started, which is how the commander
    records a frame's break tick without any extra state machine.
    """

    name = "lin_player"

    def __init__(self, net: str = "lin"):
        self.net = net
        self.segs: list[tuple[int, int]] = []
        self.marks: list[tuple[int, str]] = []      # (segment index, label), in order
        self.si = 0
        self.left = 0

    def extend(self, segments, label: Optional[str] = None) -> None:
        """Append `segments` to the waveform still to come."""
        if label is not None:
            self.marks.append((len(self.segs) + self.si, label))
        self.segs.extend(segments)

    def clear(self) -> None:
        self.segs, self.marks, self.si, self.left = [], [], 0, 0

    @property
    def done(self) -> bool:
        return self.si >= len(self.segs)

    def _next_mark(self, t: int) -> None:
        while self.marks and self.marks[0][0] <= self.si:
            index, label = self.marks.pop(0)
            self.mark_ticks[label] = t

    def tick(self, t: int, pins) -> dict:
        if self.done:
            return {self.net: None}
        ticks, level = self.segs[self.si]
        if self.left == 0:
            self.left = max(1, int(ticks))
        self._next_mark(t)
        self.left -= 1
        if self.left == 0:
            self.si += 1
        return {self.net: None if level else 0}


class LinRawTransmitter(_SegmentPlayer):
    """Drives an explicit `(ticks, level)` waveform: the fault-injection fixture.

    It sends whatever it is given, bit for bit, and records every tick it drove in `sent`.
    `verif/vip/test_lin.py` builds deliberately broken frames with it -- a 9-bit break, a
    sub-bit delimiter, a recessive start bit, a dominant stop bit, a stretched bit, nine
    data bytes, a wrong checksum -- which is how the checker is shown to catch each rule.
    A firmware author can use the same class to replay a waveform captured from hardware.

        raw = LinRawTransmitter()
        raw.extend(frame_segments(bit_ticks=2500, pid=make_pid(0x11), break_ticks=9 * 2500))
    """

    name = "lin_raw"

    def __init__(self, net: str = "lin", name: Optional[str] = None):
        super().__init__(net)
        self.mark_ticks: dict[str, int] = {}
        self.sent: list[tuple[int, int]] = []        # (tick, level) actually driven
        if name:
            self.name = name

    def tick(self, t: int, pins) -> dict:
        drives = super().tick(t, pins)
        level = drives.get(self.net)
        self.sent.append((t, 0 if level == 0 else 1))
        return drives


@dataclass
class LinTx:
    """One frame the commander put on the wire, filled in as it goes out.

    `start_tick` is when the break field began, `header_end_tick` when the PID's stop bit
    ended (that instant is the start of the response space, per LIN 2.2A: "The response
    starts at the end of stop bit of the PID field") and `end_tick` when the frame's last
    byte ended. `answered` says the commander published the response itself or saw a
    responder start one.
    """

    frame_id: int
    pid: int
    data: bytes = b""              # what the commander published, b"" if it waited
    publish: bool = False
    checksum: Optional[int] = None
    checksum_model: str = "enhanced"
    start_tick: int = -1
    header_end_tick: int = -1
    end_tick: int = -1
    response_tick: int = -1        # when the bus went dominant after the header
    header_ticks: int = 0          # the model's own 34-bit-time header length
    span_ticks: int = 0            # the model's own whole-frame length
    answered: bool = False
    error: str = ""

    @property
    def ok(self) -> bool:
        return not self.error

    def __str__(self) -> str:
        what = f"ID={self.frame_id:#04x} PID={self.pid:#04x}"
        if self.publish:
            what += f" data={self.data.hex()} csum={self.checksum:#04x}({self.checksum_model})"
        elif self.answered:
            what += " answered"
        else:
            what += " no response"
        return f"t={self.start_tick} {what}"


class LinCommander(_SegmentPlayer):
    """Bit-level LIN commander (LIN 1.x: master) for queued frames.

    Every frame is break field, break delimiter, `0x55`, protected identifier, response
    space, then either the data bytes and the checksum (if this node publishes) or nothing
    at all until the next frame. All of it is a precomputed `(ticks, level)` waveform, so
    the wire is bit-exact at any of `LIN_RATES` and the commander's own reaction latency
    cannot smear a boundary: a byte boundary is where `frame_segments()` put it.

    Defaults are the legal minimums: a 13-bit break, a 1-bit delimiter, no inter-byte space
    (so a frame is exactly `T_Frame_Nominal` long), a 2-bit response space and a 2-bit gap
    between frames. The two space defaults are *not* spec minimums -- the spec's only
    statement is that the spaces are non-negative -- and they are here because a responder
    needs room to switch state and a pin-level monitor needs to see where a frame ended.

    `timing` overrides any of the limit table's bit-time values, and it is how a test drives
    this commander off-spec on purpose: `timing={"break_bits": 9}` sends a 9-bit break.
    The content knobs `sync_byte`, `pid_override`, `checksum_mode`, `checksum_override`,
    `start_bit` and `stop_bit` exist for the same reason -- there is no other way to get a
    framing error or a wrong checksum onto the wire from a correct model.
    """

    name = "lin_commander"

    def __init__(
        self,
        rate: float = 19200.0,
        version: str = "2.2",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        net: str = "lin",
        name: Optional[str] = None,
        sync_byte: int = SYNC_BYTE,
        pid_override: Optional[int] = None,
        checksum_mode: str = "auto",        # "auto" | "classic" | "enhanced"
        checksum_override: Optional[int] = None,
        start_bit: int = 0,
        stop_bit: int = 1,
        response_wait_bits: Optional[int] = None,
    ):
        super().__init__(net)
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.version = version
        self.rate = rate
        self.sync_byte = sync_byte
        self.pid_override = pid_override
        if checksum_mode not in ("auto", "classic", "enhanced"):
            raise ValueError(
                f"checksum_mode must be auto, classic or enhanced, got {checksum_mode!r}"
            )
        self.checksum_mode = checksum_mode
        self.checksum_override = checksum_override
        self.start_bit, self.stop_bit = start_bit, stop_bit
        self.response_wait_bits = response_wait_bits
        self.mark_ticks: dict[str, int] = {}
        self.frames: list[LinTx] = []          # queued and sent, in order
        self.errors: list[str] = []
        self._labels: dict[str, LinTx] = {}    # segment label -> the frame it starts
        self.bus = bus
        self._compute()
        if name:
            self.name = name

    def _compute(self) -> None:
        self.min = lin_limits(self.ns_per_tick, self.rate, self.version,
                              self.timing_overrides)
        self.bit = self.min["bit_ticks"]
        # The commander's own spaces default to something legal and useful: the response
        # space is where a responder switches state, the frame gap is where a monitor sees
        # that the frame is over. Neither is a spec minimum, so they are not in the table.
        self.response_space = self.min["response_space_ticks"] or 2 * self.bit
        self.frame_gap = 2 * self.bit
        self.response_wait = (self.response_wait_bits * self.bit
                              if self.response_wait_bits is not None
                              else BYTE_BITS * (self.min["max_data"] + 1) * self.bit)

    def bind(self, bus: Bus) -> "LinCommander":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self._compute()
        self.clear()
        return self

    # ---- the queue ---------------------------------------------------------------
    def queue_frame(self, frame_id: int, data=None, publish: bool = False) -> LinTx:
        """Queue one frame. `data` is the payload this node publishes when `publish`.

        With `publish=False` the commander sends the header, waits `response_wait` for a
        responder and goes on: the response is then whatever the responder puts on the wire,
        which `LinTx.answered` reports. With `publish=True` the data and the checksum go out
        here, and `LinTx.checksum_model` says which checksum was used.
        """
        pid = make_pid(frame_id) if self.pid_override is None else self.pid_override
        payload = b"" if data is None else bytes(data)
        if publish and not (self.min["min_data"] <= len(payload) <= self.min["max_data"]):
            raise ValueError(
                f"a LIN response carries {self.min['min_data']}..{self.min['max_data']} "
                f"data bytes, got {len(payload)}"
            )
        model = (checksum_model(frame_id, self.version) if self.checksum_mode == "auto"
                 else self.checksum_mode)
        tx = LinTx(frame_id=frame_id, pid=pid, data=payload, publish=publish,
                   checksum_model=model)
        csum = self.checksum_override
        if publish and csum is None:
            csum = lin_checksum(pid, payload, enhanced=(model == "enhanced"))
        tx.checksum = csum
        segs = frame_segments(
            bit_ticks=self.bit,
            break_ticks=self.min["break_ticks"],
            delimiter_ticks=self.min["delimiter_ticks"],
            ibs_ticks=self.min["ibs_ticks"],
            sync=self.sync_byte,
            pid=pid,
            data=payload,
            checksum=csum,
            response_space_ticks=self.response_space,
            response_ibs_ticks=self.min["ibs_ticks"],
            start_bit=self.start_bit,
            stop_bit=self.stop_bit,
        )
        if not publish:
            segs.append((self.response_wait, 1))
        segs.append((self.frame_gap, 1))
        tx.header_ticks = self._header_ticks()
        tx.span_ticks = sum(n for n, _ in segs)
        label = f"frame{len(self.frames)}"
        self._labels[label] = tx
        self.frames.append(tx)
        self.extend(segs, label=label)
        return tx

    def clear(self) -> None:
        """Drop the queued waveform and the frame log with it."""
        super().clear()
        self.frames, self._labels, self.mark_ticks = [], {}, {}

    # ---- driving -----------------------------------------------------------------
    @property
    def busy(self) -> bool:
        return not self.done

    @property
    def idle(self) -> bool:
        return self.done

    def run(self, bus: Bus, limit: int = 4_000_000) -> bool:
        """Step `bus` until the queue drains. False if `limit` ticks ran out first."""
        return bus.run_until(lambda _b: self.done, limit)

    def tick(self, t: int, pins) -> dict:
        drives = super().tick(t, pins)
        self._bookkeep(t, pins)
        return drives

    def _bookkeep(self, t: int, pins) -> None:
        """Fill in the queued frame's ticks, and notice a responder starting a response.

        The tick arithmetic is the commander's own schedule, not an observation: it was all
        fixed when the frame was queued, so `header_end_tick` is exactly the end of the PID's
        stop bit and `end_tick` exactly the end of the last byte. Only `response_tick`, which
        is the first tick the *bus* read dominant after the header, is measured.
        """
        for label, tx in list(self._labels.items()):
            if label not in self.mark_ticks:
                continue
            start = self.mark_ticks[label]
            if tx.start_tick < 0:
                tx.start_tick = start
                tx.header_end_tick = start + tx.header_ticks
                tx.end_tick = start + tx.span_ticks
                tx.answered = tx.publish
            if (tx.response_tick < 0 and t > tx.header_end_tick
                    and pins.get(self.net) == 0):
                tx.response_tick, tx.answered = t, True
            if t >= tx.end_tick:
                del self._labels[label]

    def _header_ticks(self) -> int:
        m = self.min
        return (m["break_ticks"] + m["delimiter_ticks"] + 2 * m["ibs_ticks"]
                + 2 * BYTE_BITS * m["bit_ticks"])


class LinResponder(_SegmentPlayer):
    """Bit-level LIN responder (LIN 1.x: slave) that publishes what it subscribes to.

    It watches the wire for a break, using the spec's own detection threshold -- "a break
    detection threshold of 11 dominant local slave bit times", or 9.5 for a node with an
    oscillator better than `FTOL_RES_SLAVE` (`break_detect_bits=9.5`). Then it measures its
    bit rate from the falling edges of the `0x55` sync field, which is exactly what the
    sync field is for, and from then on it transmits at *that* rate, not at its own: the
    measured value is in `bit_ticks`, and `bit_error_ticks` is how far the measurement was
    from this node's configured rate.

    A frame is answered only if all three hold: the PID's parity bits match, the identifier
    is in `publish`, and this node's `version` matches the checksum the header implies.
    Anything else is logged in `ignored` / `parity_errors` and stays off the wire, which is
    what "all responder nodes use it to decide if they should respond to the message, ignore
    it, or just listen" (LIN 2.2A clause 2.3.1.3) means at pin level.

    It starts its response `response_delay_bits` bit times after the PID's stop bit ends, so
    the response always begins inside the commander's response space; `response_delay_bits`
    defaults to 1, the smallest value that leaves it a whole bit time to switch state.
    """

    name = "lin_responder"

    def __init__(
        self,
        rate: float = 19200.0,
        version: str = "2.2",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        net: str = "lin",
        name: Optional[str] = None,
        publish: Optional[dict] = None,
        subscribe=None,
        break_detect_bits: Optional[float] = None,
        response_delay_bits: float = 1.0,
    ):
        super().__init__(net)
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.version = version
        self.rate = rate
        self.publish_map: dict[int, bytes] = {k: bytes(v) for k, v in (publish or {}).items()}
        self.subscribe = set(subscribe) if subscribe is not None else set(self.publish_map)
        self.break_detect_bits = break_detect_bits
        self.response_delay_bits = response_delay_bits
        self.min = lin_limits(self.ns_per_tick, rate, version, self.timing_overrides)
        self.bit_ticks = self.min["bit_ticks"]
        self.bit_now = self.bit_ticks           # replaced by the sync-field measurement
        self.bit_error_ticks: Optional[int] = None
        self.mark_ticks = {}
        # what the responder saw
        self.frames_seen: list[LinTx] = []      # every header decoded, answered or not
        self.published: list[tuple[int, int, bytes, str, int]] = []
        self.parity_errors: list[tuple[int, int]] = []
        self.ignored: list[tuple[int, int, str]] = []
        self.breaks = 0
        self.bus = bus
        self._reset()

    def publish(self, frame_id: int, data) -> "LinResponder":
        """Subscribe to `frame_id` and publish `data` for it. Usable while the bus runs."""
        payload = bytes(data)
        if not self.min["min_data"] <= len(payload) <= self.min["max_data"]:
            raise ValueError(
                f"a LIN response carries {self.min['min_data']}..{self.min['max_data']} "
                f"data bytes, got {len(payload)}"
            )
        self.publish_map[frame_id] = payload
        self.subscribe.add(frame_id)
        return self

    def bind(self, bus: Bus) -> "LinResponder":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = lin_limits(self.ns_per_tick, self.rate, self.version,
                              self.timing_overrides)
        self.bit_ticks = self.min["bit_ticks"]
        return self

    def _reset(self) -> None:
        self.watch = LinRx(self.bit_ticks, mode="edges")
        self.state = "idle"            # idle | header | sending
        self.break_start: Optional[int] = None   # first tick of the dominant run being timed
        self.frame_start: Optional[int] = None   # first tick of the current frame's break
        self.arm_tick: Optional[int] = None       # where the sync field's start bit began
        self.expect = SYNC_BYTE        # the header is 0x55 then the protected identifier
        self.pid_end = 0
        self.idle_since: Optional[int] = None
        self.armed = False
        self.sync_falls = 0
        self.current: Optional[LinTx] = None
        # Break-detection threshold in ticks. The table's 11 bit times is the default;
        # `break_detect_bits=9.5` takes the narrower threshold the spec allows a node with a
        # good oscillator ("Slave nodes with a bit rate tolerance better than FTOL_RES_SLAVE
        # ... may use a 9.5 dominant nominal bit times break detection threshold").
        self.detect = (self.min["break_detect_ticks"] if self.break_detect_bits is None
                       else max(1, int(-(-self.break_detect_bits * self.bit_ticks // 1))))

    # ---- driving -----------------------------------------------------------------
    @property
    def busy(self) -> bool:
        return not self.done

    def tick(self, t: int, pins) -> dict:
        drives = super().tick(t, pins)
        self._advance(t, pins.get(self.net, 1))
        return drives

    def _advance(self, t: int, level: int) -> None:
        if self.state == "sending" and not self.done:
            self.break_start = None      # our own dominant bits are not somebody's break
            self.watch.observe(t, level)
            return
        if self.state != "header":
            self._break_watch(t, level)
        edge = self.watch.observe(t, level)
        if edge is not None:
            new = edge[1]
            if self.state == "header" and not self.armed:
                if new != 0:
                    return              # still inside the break's delimiter
                # This fall is the sync field's start bit, so the bit grid starts here. The
                # spec does not let a responder check the delimiter ("A slave node shall not
                # check that the break delimiter is at least one nominal bit time long"), so
                # a short one is not this node's business -- it shows up as a sync field
                # that does not decode.
                self.armed, self.arm_tick = True, edge[0]
                self.watch.arm(edge[0])
            elif new == 0 and self.state == "header":
                self.sync_falls += 1
        if self.state == "header":
            got = self.watch.sample(t, self.bit_now)
            if got is not None:
                self._byte_done(t, got[0], got[1], got[2])

    def _break_watch(self, t: int, level: int) -> None:
        """Look for a frame start. This runs every tick, not only on edges.

        A dominant run has no edges inside it, so a monitor that only looked at edges would
        never notice that the run had grown long enough to be a break. No byte field can
        reach 11 bit times of dominant -- the longest dominant run inside one is its start
        bit plus eight dominant data bits, and the stop bit always breaks the run -- which is
        exactly why the spec's threshold is 11 and not 9.
        """
        if level == 0:
            if self.break_start is None:
                self.break_start = t
            elif t - self.break_start >= self.detect:
                self._start_frame(t)
            self.idle_since = None
        elif self.state == "sending" and self.done:
            if self.idle_since is None:
                self.idle_since = t
            elif t - self.idle_since >= 2 * self.bit_ticks:
                self.state, self.idle_since = "idle", None
        else:
            # The dominant run is over, so its length is spent: timing has to start again from
            # the next dominant tick. Without this the timer would keep counting across the
            # stop bits between data bits and call a run of ordinary data bits a break.
            self.break_start = None
            self.idle_since = None

    def _start_frame(self, t: int) -> None:
        """The break field is long enough: this node has seen a frame start."""
        self.breaks += 1
        self.state, self.armed, self.sync_falls = "header", False, 0
        self.expect = SYNC_BYTE
        self.frame_start = self.break_start
        self.break_start, self.arm_tick = None, None
        self.watch.reset()
        self.current = None

    def _byte_done(self, t: int, value: int, start_level: int, stop: int) -> None:
        """One byte of the header has arrived, with both of its framing bits."""
        if self.expect == SYNC_BYTE:
            self._sync_byte(t, value, start_level, stop)
        else:
            self._pid_byte(t, value, start_level, stop)

    def _sync_byte(self, t: int, value: int, start_level: int, stop: int) -> None:
        """The sync field has arrived: check it, then measure the commander's bit rate."""
        if value != SYNC_BYTE or start_level != 0 or stop != 1:
            self.state = "idle"        # not a sync field: this is not a frame we can read
            self.break_start = None
            return
        # 0x55 has a level change on every bit boundary, so the ten bit times the sync field
        # spans are measurable exactly -- which is the whole point of the field: "The
        # Responder nodes measure the time between these signal transitions to determine the
        # master's precise bit timing" (TI SDAA220A). The sample tick is one before the end
        # of the field, hence the +1.
        span = max(1, t - (self.arm_tick or t))
        self.bit_now = max(1, int(round(span / 10.0)))
        self.bit_error_ticks = self.bit_now - self.bit_ticks
        self.expect = None
        self.current = None

    def _pid_byte(self, t: int, pid: int, start_level: int, stop: int) -> None:
        """The protected identifier has arrived: check the parity, then decide."""
        self.state = "idle"
        self.break_start = None
        frame_id = frame_id_of(pid)
        tx = LinTx(frame_id=frame_id, pid=pid)
        self.frames_seen.append(tx)
        if start_level != 0 or stop != 1:
            self.ignored.append((t, frame_id, "framing error in the PID byte"))
            return
        if not pid_parity_ok(pid):
            self.parity_errors.append((t, pid))
            self.ignored.append((t, frame_id, "PID parity"))
            return
        if frame_id not in self.subscribe:
            self.ignored.append((t, frame_id, "not subscribed"))
            return
        if frame_id not in self.publish_map:
            self.ignored.append((t, frame_id, "subscribed but no data"))
            return
        self._respond(t, pid, frame_id, tx)

    def _respond(self, t: int, pid: int, frame_id: int, tx: LinTx) -> None:
        """Put this node's response on the wire, starting inside the response space."""
        payload = self.publish_map[frame_id]
        model = checksum_model(frame_id, self.version)
        csum = lin_checksum(pid, payload, enhanced=(model == "enhanced"))
        bit = getattr(self, "bit_now", self.bit_ticks)
        delay = max(1, int(round(self.response_delay_bits * bit)))
        body = response_segments(payload, csum, bit, self.min["ibs_ticks"])
        self.extend([(delay, 1)] + body)
        self.state = "sending"
        self.idle_since = None
        tx.publish, tx.data, tx.checksum, tx.checksum_model = True, payload, csum, model
        tx.pid_end = self.pid_end = t               # the PID's stop bit ends on this tick
        tx.start_tick = self.frame_start if self.frame_start is not None else t
        tx.header_end_tick = tx.pid_end
        tx.response_tick = tx.header_end_tick + delay
        tx.end_tick = tx.response_tick + sum(n for n, _ in body)
        self.published.append((t, frame_id, payload, model, csum))
        self.current = tx


class LinRx:
    """Receive front end for one LIN wire: levels, runs, bit timing and byte framing.

    A LIN node has no clock of its own. It re-synchronises on a byte's start bit and then
    *counts* bit times, which is why this class samples on a timer rather than on edges: the
    sample point of a bit is its end, every `bit_ticks` ticks after the start bit began, and
    the level found there is the bit's value. Edges are used for what only edges can tell a
    monitor -- where the break field began and ended, how long the delimiter was, and how
    long a level ran -- but they never move the bit grid, so a level that happens to hold for
    two or three bit times (three dominant bits in a row, or three recessive ones at the end
    of a frame) cannot shift the framing by a bit. That property is what lets the checker
    report a missing start bit or a dominant stop bit instead of losing sync on it.

        rx = LinRx(2604)            # bit period in ticks
        rx.observe(t, level)        # every tick: level tracking and edges
        rx.arm(t)                   # a byte's start bit began at t: the bit grid starts here
        got = rx.sample(t)          # every tick: the byte, if one completed on this tick

    `sample()` returns `(value, start_level, stop_level, nsampled)`, with `value` None until
    a byte completes. `start_level` and `stop_level` are the byte's two framing bits. Bits go
    in LSB first and the ten samples of a byte are start, eight data, stop -- the start and
    stop bits are counted but not folded into the value.
    """

    def __init__(self, bit_ticks: int, mode: str = "timer"):
        if mode not in ("timer", "edges"):
            raise ValueError(f"LinRx mode must be timer or edges, got {mode!r}")
        self.bit = max(1, int(bit_ticks))
        #: "timer" samples on a bit grid anchored at the byte's start bit and never moves it:
        #: that is what a *checker* wants, because the drift the grid carries is the thing it
        #: measures. "edges" samples every level change and counts how many bit times the level
        #: it replaced covered: that is what a *responder* wants while it is reading the header,
        #: because its own oscillator is not the commander's and a grid anchored at the wrong
        #: period would read the wrong bit on its very first sample and never see the 0x55 that
        #: is there to correct it. 0x55 has an edge on every one of its ten bit times, so edge
        #: mode reads it perfectly at any rate.
        self.mode = mode
        self.level = 1
        self.prev_level = 1
        self.transition: Optional[int] = None   # tick the level last changed at
        self.span = 0                           # ticks the level that just ended lasted
        self.since = 0                    # tick the current level began
        self.low_since: Optional[int] = None   # start of the dominant run, None if recessive
        self.rises: list[int] = []
        self.falls: list[int] = []
        self.bit_start: Optional[int] = None    # tick the current bit began
        self.next_sample: Optional[int] = None  # tick the next sample point falls on
        self._reset_byte()

    def _reset_byte(self) -> None:
        self.shift = 0
        self.nbits = 0
        self.pos = 0                       # 0 = start bit next, 1..8 = data bits, 9 = stop
        self.ready: Optional[tuple] = None  # (value, start_level) waiting for its stop bit
        self.start_level = 1

    def reset(self) -> None:
        """Forget the byte in progress, and the bit grid with it."""
        self.bit_start = self.next_sample = None
        self.transition = None
        self._reset_byte()

    def arm_at(self, t_start: int, t_now: int, level: int = 0,
               bit_ticks: Optional[int] = None):
        """Arm the grid at `t_start` and catch up to `t_now`, where `level` held throughout.

        This is how a byte field is found: a byte field begins with a dominant level, and the
        level stays dominant for its start bit and however many of its data bits are zero, so
        the *rise* that ends that run is also the moment the checker can say how long the byte
        field's leading dominant stretch was. Anchoring the grid at `t_start` and feeding the
        samples the run covered is exact, and it cannot mistake a break field for a byte
        field: a 13-bit run is 13 bit times, which is more than the ten a byte field spans, so
        it comes back as a byte with the run's length still unspent.

        Returns the byte if the run was long enough to complete one, else None.
        """
        bit = max(1, int(bit_ticks or self.bit))
        self.bit_start = t_start
        self.next_sample = t_start + bit
        self.transition = None
        self._reset_byte()
        n = max(1, int(round((t_now - t_start) / bit)))
        out = None
        for _ in range(n):
            got = self._sample(level)
            if got is not None:
                out = got
                break
        if out is None:
            # The run that opened the byte field is one bit time of start bit plus however
            # many of its data bits were zero, so it measures the wire's bit period: count
            # the run in bit times and the rest of the byte is timed at that period. A real
            # node re-synchronises on exactly this measurement, and without it a wire running
            # at a different rate than nominal would be read one bit out of step by the time
            # it reached the stop bit -- which is also how `tBIT` gets something to measure.
            period = max(1, int(round((t_now - t_start) / n)))
            self.bit = period
            self.next_sample = t_now + period
        return out

    def arm(self, t: int, bit_ticks: Optional[int] = None) -> None:
        """A byte field's start bit began at `t`: the bit grid starts from here.

        The first sample point is one whole bit time later, and that is the same for every
        byte. A device on this bus sees the wire one tick after it changed (the core resolves
        all the drives at the end of a tick), so a bit that begins at tick `t` is still that
        bit on the snapshot taken at `t + bit - 1` and is the *next* bit at `t + bit`: the
        sample point is therefore `t + bit`, and the value to read there is the level that
        ended, not the one that began. Reading the level that is on the wire at the sample
        tick would report the next bit on every bit.
        """
        self.bit_start = t
        self.next_sample = t + max(1, int(bit_ticks or self.bit))
        self.transition = None      # the edge that armed the grid is the start bit itself
        self._reset_byte()

    def observe(self, t: int, level: int):
        """Feed one tick's resolved level. Returns `(tick, new, old, span)` on an edge."""
        if level == self.level:
            return None
        span = t - self.since
        old = self.level
        if level == 0:
            self.low_since = t
            self.falls.append(t)
        else:
            self.low_since = None
            self.rises.append(t)
        self.level, self.since = level, t
        self.prev_level, self.transition, self.span = old, t, span
        return (t, level, old, span)

    def sample(self, t: int, bit_ticks: Optional[int] = None):
        """Take the sample point if it has come. Returns the byte, or None.

        In "timer" mode a caller that arrives late takes one sample per bit time it missed, so
        the grid can never drift: every bit time is sampled exactly once, whichever call took it.
        In "edges" mode the sample points are the level changes themselves, and the level that
        was replaced is the bit that just ended.
        """
        if self.next_sample is None:
            return None
        bit = max(1, int(bit_ticks or self.bit))
        if self.mode == "edges":
            # One sample per level change, and as many as the level it replaced covered in bit
            # times. A tick that is not an edge samples nothing -- except once the eight data
            # bits are in, because then the level that followed them *is* the stop bit even
            # if nothing else ever comes: the protected identifier's stop bit is followed by the
            # response space, which is recessive and silent.
            if self.transition == t:
                self.transition = None
                # How many bit times have gone since the last sample? Counting from the last
                # sample rather than from the level's own start matters: a level that was
                # already partly sampled on a timer must not be counted twice.
                level = self.prev_level
                n = int(round((t - self.bit_start) / bit))
                if n < 1:
                    # The clock got to this bit before the wire did, so the timer has already
                    # sampled it. Counting it again would shift the whole byte by one bit.
                    return None
            elif t - self.bit_start >= bit + max(4, bit // 16):
                # Nothing is coming: the level has held for a bit time and an eighth, which no
                # edge can still overtake. A wire that ends a byte on recessive bits -- a
                # protected identifier followed by the response space, say -- has edges for
                # only part of it, and the rest has to come from the clock.
                level, n = self.level, 1
            else:
                return None
            self.bit_start = t
        else:
            if t < self.next_sample:
                return None
            # The sample point is where the level is expected to change, so the bit that just
            # ended is the level that was just replaced -- not the one now on the wire. An
            # edge within an eighth of a bit of the sample point pulls the grid onto it, so a
            # wire running a fraction of a per cent off still decodes; a wire more than that
            # out loses sync, which is what a real receiver does too. The grid only ever moves
            # by that much, so the drift `tBIT` measures -- a byte field's whole length -- is
            # untouched.
            window = max(1, bit // 8)
            edge = self.transition
            if edge is not None and edge <= t and t - edge <= window:
                self.next_sample, edge_here = edge, True
            else:
                edge_here = edge == t
            self.transition = None
            level = self.prev_level if edge_here else self.level
            n = (t - self.next_sample) // bit + 1
        out = None
        for _ in range(n):
            got = self._sample(level)
            if got is not None:
                out = got
                break
        self.next_sample += n * bit
        return out

    def _sample(self, level: int):
        """Fold one timed sample into the byte in progress."""
        if self.pos == 0:                         # the start bit: counted, not folded
            self.start_level, self.pos = level, 1
            return None
        if self.pos == 9:                         # the stop bit, reported with the byte
            self.pos = 0
            if self.ready is not None:
                value, start_level = self.ready
                self.ready = None
                return (value, start_level, level, 1)
            return None
        self.shift |= (level & 1) << self.nbits
        self.nbits += 1
        self.pos += 1
        if self.nbits == 8:
            self.ready, self.shift, self.nbits = (self.shift, self.start_level), 0, 0
            self.pos = 9
        return None


def response_segments(data, checksum: Optional[int], bit_ticks: int,
                      ibs_ticks: int = 0, *, start_bit: int = 0, stop_bit: int = 1,
                      bit_ticks_override: Optional[list] = None
                      ) -> list[tuple[int, int]]:
    """A response (data bytes then the checksum) as `(ticks, level)` segments.

    No break field, no sync, no PID: a responder sends exactly this, and so does a commander
    that publishes. The inter-byte space goes *before* each byte, so the first byte's start
    bit is the first thing on the wire after the response space. `bit_ticks_override` gives
    per-bit durations for the first data byte, which is how a test stretches one bit.
    """
    segs: list[tuple[int, int]] = []
    for i, byte in enumerate(bytes(data)):
        if ibs_ticks:
            segs.append((int(ibs_ticks), 1))
        segs += byte_segments(byte, bit_ticks, start=start_bit, stop=stop_bit,
                              bit_ticks_override=bit_ticks_override if i == 0 else None)
    if checksum is not None:
        if ibs_ticks:
            segs.append((int(ibs_ticks), 1))
        segs += byte_segments(checksum, bit_ticks, start=start_bit, stop=stop_bit)
    return segs


# ---- harness helpers ---------------------------------------------------------
def _ensure_net(bus: Bus, net: str = "lin") -> None:
    """Add the single LIN wire if it is not there yet: open drain, recessive = pull-up."""
    if net not in bus.nets:
        bus.add_net(net, push_pull=False, pullup=1)


def attach_commander(bus: Bus, **kwargs) -> LinCommander:
    """Build a LinCommander that takes its timing from `bus` and attach it."""
    net = kwargs.get("net", "lin")
    _ensure_net(bus, net)
    ctl = LinCommander(bus=bus, **kwargs)
    bus.attach(ctl)
    return ctl


def attach_responder(bus: Bus, **kwargs) -> LinResponder:
    """Build a LinResponder that takes its timing from `bus` and attach it."""
    net = kwargs.get("net", "lin")
    _ensure_net(bus, net)
    rsp = LinResponder(bus=bus, **kwargs)
    bus.attach(rsp)
    return rsp


def attach_raw(bus: Bus, **kwargs) -> LinRawTransmitter:
    """Build a LinRawTransmitter on `bus` and attach it (see its docstring)."""
    net = kwargs.get("net", "lin")
    _ensure_net(bus, net)
    raw = LinRawTransmitter(**kwargs)
    bus.attach(raw)
    return raw


def attach_checker(bus: Bus, **kwargs) -> "LinChecker":
    """Build a passive LinChecker for `bus` and attach it."""
    net = kwargs.get("net", "lin")
    _ensure_net(bus, net)
    chk = LinChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk


@dataclass(frozen=True)
class Violation:
    """One spec rule the wire did not keep, with the tick it was missed on.

    `measured` and `required` are ticks. A rule that is a maximum (`tHEADER_MAX`,
    `tRESPONSE_MAX`) puts `measured` above `required` instead of below it, so
    `Violation.measured < Violation.required` does not hold for those two; `over` says which
    way the rule went.
    """

    tick: int
    rule: str            # e.g. "tBREAK", "SYNC", "PID", "tCS"
    measured: int        # ticks the wire gave it
    required: int        # ticks the rule asks for
    detail: str = ""
    over: bool = False   # True: this is a maximum, and measured went over it

    def __str__(self) -> str:
        if self.over:
            return (f"t={self.tick} {self.rule} {self.measured} ticks > {self.required}"
                    + (f" ({self.detail})" if self.detail else ""))
        return (f"t={self.tick} {self.rule} {self.measured} ticks < {self.required}"
                + (f" ({self.detail})" if self.detail else ""))


@dataclass
class LinFrame:
    """One frame the checker decoded, with the fields it found and what it made of them.

    `data` holds the response's data bytes -- the checksum is *not* in it -- and
    `checksum` is the byte that came last. `n_data` is how many data bytes the wire carried,
    which is not the same thing as how many the LDF says it should, which is why both are
    here. `ok` means every rule this frame touched was kept.
    """

    tick: int
    frame_id: Optional[int] = None
    pid: Optional[int] = None
    sync: Optional[int] = None
    data: bytes = b""
    checksum: Optional[int] = None
    n_data: int = 0
    expect_n: Optional[int] = None
    checksum_model: str = "enhanced"
    checksum_ok: Optional[bool] = None
    parity_ok: Optional[bool] = None
    bit_ticks: Optional[int] = None
    header_ticks: int = 0
    response_ticks: int = 0
    start_tick: int = -1
    header_end_tick: int = -1
    end_tick: int = -1
    response_space: int = 0
    violations: list[Violation] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.violations

    @property
    def label(self) -> str:
        """The frame's name for a violation message, even if its PID never decoded."""
        return "frame ?" if self.frame_id is None else f"frame {self.frame_id:#04x}"

    def __str__(self) -> str:
        what = f"ID={self.frame_id if self.frame_id is None else hex(self.frame_id)}"
        if self.pid is not None:
            what += f" PID={self.pid:#04x}"
        what += f" data={self.data.hex() or '-'}"
        if self.checksum is not None:
            what += (f" csum={self.checksum:#04x}({self.checksum_model})"
                     f"{'' if self.checksum_ok else ' BAD'}")
        if self.violations:
            what += f" [{len(self.violations)} violation(s)]"
        return f"t={self.tick} {what}"


class LinChecker(Device):
    """Passive LIN bus monitor: decodes the wire and checks 12 rules against the spec.

    It drives nothing, so it sees the bus exactly as an analyser would, and it measures the
    resolved pin values without adding or subtracting anything for a device reacting a tick
    late. It measures you, not the devices.

    What it checks, and where each number comes from (research/ref/protocols/lin/README.md
    has the full table):

      tBREAK         the dominant run that starts a frame is at least 13 bit times
      tDELIM         the recessive run after it is at least 1 bit time
      tBIT           every bit period is inside the tolerance: +/-14 % before the sync field
                     (FTOL_UNSYNC), +/-1.5 % after it (FTOL_SYNC)
      SYNC           the sync field carries 0x55
      tSTOP          every byte field's stop bit is recessive (a recessive *start* bit is not
                     a rule of its own: it re-frames the monitor instead, see `_decode`)
      PID            P0 and P1 are the parity of the identifier they travel with
      tDATALEN       the response carries as many data bytes as the LDF says (1..8 of
                     them). A response one byte *too long* is not reported here: at pin
                     level its last byte can only be the checksum, so it comes out as tCS
                     plus a stray byte field (tRESPONSE), which is what a receiving node
                     would see
      tCS            the checksum is right, in the model the identifier requires (classic for
                     0x3C/0x3D, enhanced for the rest of a 2.x cluster)
      tRS            the response space is at least `response_space_bits` (default 0)
      tIB            so is every inter-byte space, at `ibs_bits` (default 0)
      tRESPONSE      the frame ends exactly after the checksum: no response, a short one, or
                     more bytes than the LDF allows
      tFRAME_MIN     the frame is at least T_Frame_Nominal = 34 + 10(N+1) bit times long
      tHEADER_MAX    the header is at most 1.4 x 34 bit times, which also bounds the break
      tRESPONSE_MAX  the response is at most 1.4 x 10(N+1) bit times

    Every miss lands in `violations` with the tick it happened on, and
    `violations_by_rule(rule)` picks one rule out. Decoded frames are in `frames`.

    **Frame length is not on the wire.** A node only knows how long a response is from the
    LDF ("The publisher and all subscribers shall agree on the number of data bytes contained
    in a frame with a specific frame identifier"), so pass `lengths=`:
    `lengths={0x11: 2}` is an LDF, `lengths="table"` is the published 2/4/8 convention per
    identifier range, and `lengths=None` (the default) falls back to "the frame ends at the
    first recessive gap of at least one bit time", which only works if something leaves one.
    With `lengths` given, a frame that does not carry the agreed number of bytes is a
    `tRESPONSE` violation rather than a silent re-sync.

    One caveat, the same one a real analyser has: the tick a frame's fields are timed from is
    the tick the *break* began, and a frame still in progress when the simulation stops is
    never closed out, so nothing is reported for it.
    """

    name = "lin_check"

    def __init__(
        self,
        rate: float = 19200.0,
        version: str = "2.2",
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        timing: Optional[dict] = None,
        lengths=None,
        idle_bits: float = 8.0,
        net: str = "lin",
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.timing_overrides = dict(timing or {})
        self.min = lin_limits(self.ns_per_tick, rate, version, self.timing_overrides)
        self.lengths = self._lengths(lengths)
        #: How long the bus must be recessive, in bit times, before the checker calls a frame
        #: over. It has to exceed the longest run of recessive time *inside* a frame -- the
        #: response space plus the longest inter-byte space the bus uses -- because the wire
        #: looks exactly the same in both places. Eight bit times is above anything the spec
        #: asks a node to leave; a cluster that stretches its spaces should raise it. A frame
        #: with a known length is closed by its bytes arriving, and by the next frame's break
        #: field, so this only decides when a response that came back short is noticed.
        self.idle_bits = idle_bits
        self.idle_ticks = max(1, int(round(idle_bits * self.min["bit_ticks"])))
        self.net = net
        if name:
            self.name = name
        self.violations: list[Violation] = []
        self.frames: list[LinFrame] = []
        self._reset()

    def _lengths(self, lengths) -> Optional[dict]:
        """Normalise the LDF argument: a mapping, "table", or None."""
        if lengths is None or lengths is False:
            return None
        if lengths == "table":
            return dict(LENGTH_TABLE)
        if not isinstance(lengths, dict):
            raise ValueError(
                'lengths must be a mapping of frame identifier to data-byte count, '
                '"table", or None'
            )
        out = {}
        for key, value in lengths.items():
            if not 0 <= int(key) <= 0x3F:
                raise ValueError(f"a LIN frame identifier is 6 bits, got {key!r}")
            if not self.min["min_data"] <= int(value) <= self.min["max_data"]:
                raise ValueError(
                    f"frame {key:#04x}: a response carries "
                    f"{self.min['min_data']}..{self.min['max_data']} data bytes, "
                    f"got {value!r}"
                )
            out[int(key)] = int(value)
        return out

    def bind(self, bus: Bus) -> "LinChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.min = lin_limits(self.ns_per_tick, self.min["rate"], self.min["version"],
                              self.timing_overrides)
        return self

    def _reset(self) -> None:
        self.watch = LinRx(self.min["bit_ticks"])
        self.state = "idle"            # idle | delim | wait | byte
        self.prev_field: Optional[str] = None   # field of the byte just decoded
        self.field_end: Optional[int] = None    # tick its stop bit ended
        self.break_start: Optional[int] = None
        self.delim_start: Optional[int] = None   # the tick the break's dominant run ended
        self.frame: Optional[LinFrame] = None
        self.field = ""                # which byte is being decoded: sync, pid, data, csum
        self.byte_start: Optional[int] = None     # tick the byte field's start bit began
        self.byte_done: Optional[int] = None      # tick the last byte field completed
        self.data = bytearray()
        self.shortest_byte: Optional[int] = None
        self.longest_byte: Optional[int] = None
        self.byte_times: list[int] = []
        self.bytes = 0
        self.breaks = 0
        self.in_break = False     # the current dominant run is a break field
        #: (start, length) of a dominant run too short to be a break field, waiting to
        #: see whether a 0x55 follows it. That is the only way to tell a frame whose
        #: break was too short from ordinary bus activity inside somebody else's frame.

    # ---- what the tests ask for --------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def max_bit_hz(self) -> Optional[float]:
        """The fastest bit the wire actually ran at, from the shortest byte field."""
        if self.shortest_byte is None:
            return None
        return 1e9 / (self.ns_per_tick * self.shortest_byte / BYTE_BITS)

    @property
    def min_bit_hz(self) -> Optional[float]:
        """The slowest bit the wire actually ran at, from the longest byte field."""
        if self.longest_byte is None:
            return None
        return 1e9 / (self.ns_per_tick * self.longest_byte / BYTE_BITS)

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def report(self, limit: int = 12) -> str:
        m = self.min
        head = (f"LIN {m['version']} at {self.ns_per_tick} ns/tick, "
                f"{m['rate'] / 1e3:.1f} kbit/s ({m['bit_ticks']} ticks/bit): "
                f"{self.breaks} break(s), {self.bytes} byte field(s), "
                f"{len(self.frames)} frame(s), {len(self.violations)} violation(s)")
        if self.shortest_byte is not None:
            head += (f", byte fields {self.shortest_byte}..{self.longest_byte} ticks "
                     f"= {self.min_bit_hz / 1e3:.2f}..{self.max_bit_hz / 1e3:.2f} kbit/s, "
                     f"tolerance {m['ftol_unsync'] * 100:.1f}% then "
                     f"{m['ftol_sync'] * 100:.1f}%")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _violation(self, t: int, rule: str, measured: int, required: int,
                   detail: str = "", over: bool = False) -> None:
        v = Violation(t, rule, measured, required, detail, over)
        self.violations.append(v)
        if self.frame is not None:
            self.frame.violations.append(v)

    def _floor(self, t: int, rule: str, measured: int, required: int, detail: str = "") -> None:
        if measured < required:
            self._violation(t, rule, measured, required, detail)

    def _ceiling(self, t: int, rule: str, measured: int, required: int,
                 detail: str = "") -> None:
        if measured > required:
            self._violation(t, rule, measured, required, detail, over=True)

    # ---- the monitor --------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        self._advance(t, pins.get(self.net, 1))
        return {}

    def _advance(self, t: int, level: int) -> None:
        """One tick of the wire.

        A byte field is found at the *rise* that ends its leading dominant run, not at the
        fall that starts it: everything between two byte fields is recessive (an inter-byte
        space, a response space, or the idle bus), so the dominant level that opens a byte
        field is also how long the checker has to wait before it can say whether that was a
        byte field at all. A run of 13 bit times is a break field, not a byte field, and the
        two are told apart by exactly that length.
        """
        self.watch.observe(t, level)
        self._runs(t, level)
        if self.state == "delim":
            if level == 0:
                self._delimiter(t)
        elif self.state == "byte":
            got = self.watch.sample(t)
            if got is not None:
                self._decode(t, got[0], got[1], got[2])
        # A frame with a known length ends when the bytes arrive, not when the bus goes
        # quiet: a response is allowed to leave a long inter-byte space in it, and a node
        # that knew how many bytes to expect would sit there and wait. Only a checker with no
        # LDF entry for this identifier has to fall back on the silence.
        if self.state == "wait" and self.frame is not None and self.frame.expect_n is None:
            if self.byte_done is not None and level == 1 \
                    and t - self.byte_done >= self.idle_ticks:
                self._end_frame(t, "the bus went idle")

    def _runs(self, t: int, level: int) -> None:
        """Time the dominant run on the wire, whatever else is going on.

        Runs have to be timed every tick, not on edges: a dominant run has no edges inside it.
        A run of `break_detect_ticks` dominant is a break field, and no byte field can reach
        that length -- the longest dominant run inside one is its start bit plus eight
        dominant data bits -- so this is unambiguous, and it is also how a frame that is still
        open when the next break arrives gets closed out.
        """
        if level != 0:
            start, self.break_start = self.break_start, None
            if start is None:
                return
            self.rec_since = t          # the recessive run begins now
            run = t - start
            if not self.in_break:
                # Too short to be a break field. It is still a byte field if it is short
                # enough to be one -- the longest dominant run a byte field can produce is
                # start bit plus eight data bits plus a stop bit -- and the run in front of a
                # data bit inside a byte field belongs to the grid already armed for it.
                if run <= self.min["byte_run_limit"]:
                    if self.state in ("wait", "idle"):
                        self._byte_start(t, start, run)
                    return
                # Longer than any byte field and shorter than a break field: neither, and
                # illegal as a break field on both counts. This is also where a break field of
                # about ten bit times lands -- see judgement call 1 in the README.
                self._floor(t, "tBREAK", run, self.min["break_ticks"],
                            "dominant run too long to be a byte field and too short to be a "
                            "break field")
                self.state, self.delim_start = "delim", t
                return
            self.in_break = False
            self._floor(t, "tBREAK", run, self.min["break_ticks"], "break field")
            self.state, self.delim_start = "delim", t
            return
        if self.break_start is None:
            self.break_start = t
        elif (t - self.break_start + 1 >= self.min["break_detect_ticks"]
              and not self.in_break):
            self.in_break = True
            if self.frame is not None:
                self._end_frame(t, "the next break field began")
            self.breaks += 1
            self.frame = LinFrame(tick=self.break_start, start_tick=self.break_start)
            self.frames.append(self.frame)
            self.field, self.prev_field = "sync", None
            self.data = bytearray()
            self.byte_done = self.field_end = None

    def _delimiter(self, t: int) -> None:
        """The break delimiter is over: the sync field's start bit begins at `t`."""
        delim = t - (self.delim_start or t)
        self._floor(t, "tDELIM", delim, self.min["delimiter_ticks"], "break delimiter")
        self._floor(t, "tIB", delim, self.min["ibs_ticks"],
                    "space between the break field and the sync field")
        # The sync field's own framing is timed like every other byte field's, at the rise
        # that ends its leading dominant run, which `_runs` does next.
        self.prev_field, self.field_end = None, None
        self.field, self.state = "sync", "wait"

    def _byte_start(self, t: int, start: int, run: int) -> None:
        """A dominant run of `run` ticks began at `start`: a byte field starts there.

        The space in front of the byte field is timed first -- that one rule covers the
        response space after the PID and every inter-byte space -- and then the grid is
        anchored at `start` and caught up to now. If the run turned out to be long enough to
        finish a byte inside itself, that is a byte field with a dominant stop bit, and it is
        reported here and now.
        """
        prev, self.prev_field = self.prev_field, None
        if prev is not None:
            self._space(start, prev)
        self.state, self.byte_start = "byte", start
        got = self.watch.arm_at(start, t)
        if got is not None:
            self._decode(t, got[0], got[1], got[2])

    def _byte(self, t: int, value: int, start_level: int, stop: int) -> None:
        """One byte field, decoded, with its two framing bits and its duration checked."""
    def _space(self, start: int, prev: str) -> None:
        """Time the space in front of the byte field that begins at `start`.

        A gap that comes out slightly negative is a wire running faster than nominal, which
        `tBIT` already reports against FTOL, not a spacing error: the previous byte field's
        end is known only to the bit grid, which is up to ten bit times of drift out on a
        fast wire. A gap negative by more than half a bit time is two byte fields genuinely
        overlapping, and that is reported.
        """
        gap = start - (self.field_end or start)
        if prev == "sync":
            # The gap between the sync field and the PID is not an inter-byte space: the two
            # are one 20-bit-time header, which is what makes the header 34 bit times exactly.
            # A gap here makes the header long, and tHEADER_MAX is measured over the whole
            # header once the PID is in, so there is nothing to add here.
            return
        rule = "tRS" if prev == "pid" else "tIB"
        floor = (self.min["response_space_ticks"] if prev == "pid"
                 else self.min["ibs_ticks"])
        what = ("response space between the PID and the first data byte" if prev == "pid"
                else f"inter-byte space after the {prev} byte")
        if prev == "pid" and self.frame is not None:
            self.frame.response_space = max(gap, 0)
        if gap < -self.min["bit_ticks"] // 2:
            self._violation(start, rule, gap, floor,
                            f"{what}: the byte fields overlap by {-gap} ticks")
        else:
            self._floor(start, rule, max(gap, 0), floor, what)

    def _decode(self, t: int, value: int, start_level: int, stop: int) -> None:
        """Report one byte field: framing bits, duration, and the field's own rule."""
        field = self.field          # the handler below may change it, so keep our own copy
        if self.frame is None:
            self._violation(t, "tRESPONSE", 0, 1,
                            f"a 0x{value:02x} byte field arrived with no frame open")
            # Back to hunting for the next frame. The grid that decoded this field runs on a
            # timer, so leaving it armed would sample the rest of an idle bus every bit time
            # and report a byte field of recessive bits for each of them.
            self.state = "idle"
            return
        self.bytes += 1
        if self.byte_start is not None:
            self._byte_time(t)
        # A recessive start bit is deliberately *not* a rule of its own here, and the reason is
        # worth writing down: everything between two byte fields is recessive, so a start bit
        # that is recessive too simply extends that gap by a bit and the monitor re-frames on
        # the next dominant level. The byte then decodes shifted, which shows up as a checksum
        # or a length error -- both of which are reported. A pin-level monitor cannot tell the
        # two apart, and a rule that can never fire is worse than none.
        if stop != 1:
            self._violation(t, "tSTOP", stop, 1,
                            f"the {field} byte's stop bit is dominant")
        # The byte field ends here, before the handler runs: the handler may close the frame
        # out, and the frame's length is measured from its last byte field, not from the tick
        # the checker happened to notice.
        self.byte_done, self.field_end = t, t
        handler = {"sync": self._sync_byte, "pid": self._pid_byte}.get(
            field, self._data_byte)
        handler(t, value)
        if self.frame is None:
            # The handler closed the frame out -- and may have said where to go next (a bad
            # sync field leaves the monitor hunting for the next frame rather than waiting for
            # a byte field), so only fall back to "wait" if it did not.
            if self.state == "byte":
                self.state = "wait"
            return
        self.prev_field, self.state = field, "wait"

    def _byte_time(self, t: int) -> None:
        """A byte field spans exactly ten bit times; its length carries the bit drift.

        The bit grid is re-armed at each byte field's start bit, so this measures the ten bit
        times this byte field actually took -- which is where a commander's rate error shows
        up, and what `tBIT` compares against FTOL_SYNC (or FTOL_UNSYNC, before the sync field
        has been decoded and a node is still allowed to be 14 % out).
        """
        dur = t - self.byte_start
        self.byte_times.append(dur)
        self.shortest_byte = (dur if self.shortest_byte is None
                              else min(self.shortest_byte, dur))
        self.longest_byte = (dur if self.longest_byte is None
                             else max(self.longest_byte, dur))
        unsynced = self.field == "sync"
        lo = self.min["byte_min_ticks_unsync" if unsynced else "byte_min_ticks"]
        hi = self.min["byte_max_ticks_unsync" if unsynced else "byte_max_ticks"]
        tol = self.min["ftol_unsync" if unsynced else "ftol_sync"]
        what = "before the sync field (FTOL_UNSYNC)" if unsynced else "after it (FTOL_SYNC)"
        detail = (f"{self.field} byte field is {dur / BYTE_BITS:.1f} ticks per bit, "
                  f"{what} allows {tol * 100:.1f}%")
        self._floor(t, "tBIT", dur, lo, detail)
        self._ceiling(t, "tBIT", dur, hi, detail)

    def _sync_byte(self, t: int, value: int) -> None:
        if value != SYNC_BYTE:
            if self.frame is not None:
                self.frame.sync = value
                self._violation(t, "SYNC", value, SYNC_BYTE,
                                f"the sync field carried {value:#04x}")
                self._end_frame(t, "the sync field was not 0x55", check_length=False)
            # A frame this monitor cannot read is over: stop decoding it, because reporting
            # every byte that follows as a stray byte field would bury the sync error that
            # caused it. The exception is a short-break candidate, where the 0x55 that settles
            # it may still be coming -- keep hunting for it.
            self.state = "hunt"
            self.byte_done = self.byte_start = self.field_end = None
            return
        self.frame.sync = value
        self.field = "pid"

    def _pid_byte(self, t: int, pid: int) -> None:
        """The protected identifier has arrived: check the parity and learn the length."""
        f = self.frame
        f.pid, f.frame_id = pid, frame_id_of(pid)
        f.parity_ok = pid_parity_ok(pid)
        if not f.parity_ok:
            self._violation(t, "PID", pid, make_pid(f.frame_id),
                            f"parity bits of PID {pid:#04x} do not belong to ID "
                            f"{f.frame_id:#04x} (P0={make_pid(f.frame_id) >> 6 & 1}, "
                            f"P1={make_pid(f.frame_id) >> 7 & 1})")
        f.expect_n = self.lengths.get(f.frame_id) if self.lengths else None
        f.checksum_model = checksum_model(f.frame_id, self.min["version"])
        f.header_end_tick = t
        f.header_ticks = f.header_end_tick - f.start_tick
        self._ceiling(t, "tHEADER_MAX", f.header_ticks,
                      self.min["header_max_ticks"],
                      "the header, break field included")
        self.data = bytearray()
        self.field = "data"

    def _data_byte(self, t: int, value: int) -> None:
        """A response byte: a data byte, or the checksum once the agreed length is in.

        The checksum is not a data byte, so the payload count is one less than the number of
        bytes seen -- unless the LDF says how long the payload is, in which case that is what
        the count is checked against, and a byte past it ends the frame instead.
        """
        f = self.frame
        if self.field == "csum":
            self.data.append(value)
            f.n_data = len(self.data) - 1
            self._checksum(t)
            self._end_frame(t, "the checksum byte arrived")
            return
        self.data.append(value)
        f.n_data = len(self.data)
        limit = f.expect_n if f.expect_n is not None else self.min["max_data"]
        if len(self.data) > limit:
            if f.expect_n is None:
                self._violation(t, "tDATALEN", len(self.data), limit,
                                "a response carries at most 8 data bytes and then a "
                                "checksum", over=True)
            else:
                self._violation(t, "tRESPONSE", len(self.data), limit,
                                f"{f.label} carries more data bytes than the LDF says",
                                over=True)
            self.field = "csum"      # this byte can only be the checksum
        else:
            self.field = "csum" if len(self.data) == limit else "data"

    def _checksum(self, t: int) -> None:
        f = self.frame
        if f is None:
            return
        if len(self.data) < 2:
            f.data, f.n_data = bytes(self.data), max(0, len(self.data) - 1)
            f.checksum = self.data[-1] if self.data else None
            f.checksum_ok = False
            if self.data:
                self._violation(t, "tCS", 0, 1,
                                "the frame ended before a checksum byte could be checked")
            return
        payload, csum = bytes(self.data[:-1]), self.data[-1]
        enhanced = f.checksum_model == "enhanced"
        f.data, f.checksum, f.n_data = payload, csum, len(payload)
        f.checksum_ok = checksum_ok(f.pid, payload, csum, enhanced)
        if not f.checksum_ok:
            want = lin_checksum(f.pid, payload, enhanced)
            self._violation(t, "tCS", csum, want,
                            f"{f.checksum_model} checksum of PID {f.pid:#04x} and "
                            f"{len(payload)} data byte(s); the receiver's test is that "
                            "they all sum to 0xFF")

    def _end_frame(self, t: int, why: str, check_length: bool = True) -> None:
        """Close the frame out and check the bounds that only the whole frame can show."""
        f = self.frame
        if f is None:
            self.state = "idle"
            return
        if f.checksum is None:
            # A checker with no LDF never knew where the payload ended, so the last byte on
            # the wire is the checksum; a checker that did know and ran out of bytes has none.
            self._checksum(t)
        # The frame ends with its last byte field, not with the tick the checker noticed: a
        # frame closed out by the next break field has eleven bit times of that break inside
        # it, and counting them would make every frame in a schedule look too long.
        f.end_tick = self.field_end or t
        f.response_ticks = max(0, f.end_tick - (f.header_end_tick or t))
        if not check_length:
            self.frame = None
            self.state = "idle"
            self.field = ""
            self.data = bytearray()
            self.byte_done = self.byte_start = self.field_end = None
            return
        if f.n_data == 0:
            # Nothing answered at all. That is a different fault from a response that came
            # back short, and it is the one that tells a firmware author their node is not
            # subscribed rather than that its data is out of date.
            self._violation(t, "tRESPONSE", 0, self.min["min_data"] + 1,
                            f"{f.label} sent no response at all ({why})")
        elif f.expect_n is not None and f.n_data < f.expect_n:
            self._violation(t, "tDATALEN", f.n_data, f.expect_n,
                            f"{f.label} should carry {f.expect_n} data byte(s) and a "
                            f"checksum; {why} after {f.n_data}")
        if f.frame_id is not None and not self.min["min_data"] <= f.n_data <= self.min["max_data"]:
            self._violation(t, "tDATALEN", f.n_data, self.min["min_data"],
                            f"{f.label} carried {f.n_data} data bytes")
        self._ceiling(t, "tRESPONSE_MAX", f.response_ticks,
                      response_max_ticks(self.min, max(f.n_data, 1)),
                      "the response, response space included")
        self._floor(t, "tFRAME_MIN", f.end_tick - f.start_tick,
                    frame_min_ticks(self.min, max(f.n_data, 1)),
                    "T_Frame_Nominal = 34 + 10(N+1) bit times")
        self.frame = None
        self.state = "idle"
        self.field = ""
        self.data = bytearray()
        # The next byte field after the frame is over is not an inter-byte space behind
        # anything, and timing it against a field that is gone is how a report ends up
        # measuring a gap between two frames.
        self.prev_field = self.field_end = None
        self.byte_done = None

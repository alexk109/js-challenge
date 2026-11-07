"""Pin-level classic CAN 2.0A reference models on the tick bus in verif/vip/core.py.

One open-drain net named "can": a node drives 0 for a DOMINANT bit and releases for a
RECESSIVE one, so the wired-AND of core.py gives the bus the spec's rule that simultaneous
dominant and recessive bits read dominant (Bosch CAN Specification 2.0 Part A sec. 2,
"Bus values", in `research/ref/protocols/can/01-bosch-frame-fields-coding.txt`). There is no
transceiver in this model, so the pin *is* the bus level: no propagation delay, no loop delay,
no oscillator tolerance and therefore no resynchronisation ever fires. Every timing number
here is quoted from that same saved spec; see `research/ref/protocols/can/README.md` for the
table that maps each one to the text it came from.

Two devices on the one net, plus the frame codec they share:

  CanNode        a full CAN node: queued frames, multimaster arbitration, reception, CRC
                 checking, it ACKs a frame whose CRC it matched, and on any error it signals
                 an ERROR FLAG (6 dominant bits, 6 recessive when error-passive) plus the
                 8-bit ERROR DELIMITER and retransmits, with the sec. 7 transmit/receive
                 error counters driving error-active -> error-passive -> bus-off and back to
                 error-active again (sec. 7 rules 9, 10, 11 and 12).
  CanChecker     a passive monitor. It drives nothing, locks its own bit grid onto every SOF
                 edge, samples each bit at the sample point, measures every edge against the
                 nominal bit time, and flags every timing, stuffing, CRC, form, ACK and
                 intermission violation with the tick it happened on.
  frame codec     `frame_bits`, `wire_bits`, `destuff`, `crc15` and `parse_frame`: the pure
                 functions the two devices share, so they only ever agree about the wire
                 format. `wire_bits` is also how a test plants one specific fault: it takes
                 the logical bits and a mutation and returns the bits to put on the wire.

What the checker enforces (every rule cites Bosch Part A, files `01`-`04` of the research dir):

  bit time        every edge must fall on the nominal bit grid, +-`edge_tol` ticks
  stuff           6 consecutive equal bits inside SOF..CRC SEQUENCE (the stuff rule is sec. 5)
  crc             the CRC recomputed over SOF..end of DATA FIELD must equal the 15 received
  form            SOF dominant; r1/r0 dominant; DLC <= 8; the CRC delimiter, the ACK
                  delimiter and all 7 EOF bits recessive; and an ERROR FLAG no longer than
                  the 12 bits sec. 3.1.3 allows for superposed flags
  ack             the ACK SLOT must read dominant: somebody acknowledged the frame
  intermission    3 recessive bits after a frame; a dominant bit inside them is a SOF that
                  the spec does not allow there
  delimiter       the 8 recessive bits of an ERROR or OVERLOAD delimiter after a 6-bit flag
  priority        ID-10..ID-4 must not be all recessive. Its own rule, because a real
                  controller accepts such a frame and gives it the lowest priority rather
                  than treating it as a FORM ERROR -- so `CanNode` does not flag it either.

A standard (11-bit) DATA FRAME, as built by `frame_bits`, is

    SOF | ID10..ID0 | RTR | r1 r0 | DLC3..DLC0 | data 0..dlc-1 (MSB first) | CRC14..CRC0 |
    CRC delim | ACK slot | ACK delim | EOF (7 recessive)

and SOF..CRC14..CRC0 is the stuffed region: after five equal bits the transmitter inserts the
complement (sec. 5), everything from the CRC delimiter on is fixed form and not stuffed.

Typical use, both speeds, at 50 MHz:

    bus = Bus(ns_per_tick=20)
    a = attach_node(bus, name="a", speed="125k")
    b = attach_node(bus, name="b", speed="125k")
    chk = attach_checker(bus, speed="125k")       # optional, passive
    a.send(0x123, b"\\xde\\xad")
    assert a.run(bus, limit=2_000_000)
    assert b.rx[0].frame.data == b"\\xde\\xad" and chk.clean, chk.report()
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil
from typing import Optional, Sequence

from .core import Bus, Device

DOMINANT = 0            # Bosch sec. 2: wired-AND bus, dominant is a logical '0'
RECESSIVE = 1           # ... and recessive a logical '1'
STUFF_RUN = 5           # sec. 5: after five consecutive equal bits insert the complement
CRC_POLY = 0x4599       # sec. 3.1.1: X^15 + X^14 + X^10 + X^8 + X^7 + X^4 + X^3 + 1
CRC_BITS = 15
ID_BITS = 11            # sec. 3.1.1: standard format, ID-10 (MSB) .. ID-0 (LSB)
MAX_DLC = 8             # sec. 3.1.1: "admissible numbers of data bytes: {0,1,....,7,8}"
#: How long `CanNode.run` waits for the bus to free itself, in BIT TIMES rather than ticks, so
#: the guard does not have to be re-derived for each speed. A DATA FRAME with eight data bytes is
#: at most 127 bit times on the wire (98 logical bits inside the stuffed region, plus one stuff
#: bit per five of them at the worst, plus the ten fixed-form recessive bits) and 130 with the
#: three INTERMISSION bits, so this is room for dozens of frames: the point is that a bus on
#: which nothing ever goes right fails a test instead of hanging it for ever.
ACK_TIMEOUT_TICKS = 20_000

#: core.py's two-phase tick means a device that drives a level at tick t has the net *read*
#: that level from tick t+1 on (see its docstring). Everything here that locks a bit grid onto
#: an observed edge takes this many ticks off the edge, so the grid is the transmitter's own
#: grid and not the monitor's view of it one tick later. uart.py calls the same number
#: `latency` and does the same with it.
OBSERVATION_LATENCY = 1


def stuff(bits: Sequence[int]) -> list[int]:
    """Bit-stuff `bits` the way a transmitter does (sec. 5).

    After STUFF_RUN equal bits in a row the complement is inserted. This is what a CAN
    transmitter puts on the wire, and it is why the longest run of equal bits inside a
    frame is five plus the stuffed one.
    """
    out: list[int] = []
    run = 0
    prev = RECESSIVE
    for bit in bits:
        run = run + 1 if bit == prev else 1
        prev = bit
        out.append(bit)
        if run == STUFF_RUN:
            # The inserted bit is a bit of the wire like any other, so it also starts the
            # next run: that is what keeps the longest run on the wire at five, which is what
            # makes "six equal bits in a row" (sec. 6.1) the stuff error a receiver looks for.
            prev = RECESSIVE if bit == DOMINANT else DOMINANT
            out.append(prev)
            run = 1
    return out


def destuff(bits: Sequence[int], stuffed: Optional[int] = None
            ) -> tuple[list[int], list[tuple[int, int]]]:
    """Undo `stuff`, returning (logical bits, [(index, level)] of any stuff violations).

    `stuffed` is how many of the leading *logical* bits are inside the stuffed region --
    `stuff_region(frame)` for a real frame. It has to be said, because the rule only applies up
    to the CRC sequence: without it the seven recessive bits of END OF FRAME look like a
    missing stuff bit. None means "the whole stream is stuffed", which is what a test wants
    when it is looking at a bare run of bits.

    A stuff violation is reported at the index of the offending bit: six equal bits in a row
    inside the stuffed region, which sec. 6.1 calls a STUFF ERROR. The bit is dropped from
    the logical stream, because a receiver that has lost the stuffing rule can no longer know
    where the frame fields are; the caller decides what to do about it.
    """
    out: list[int] = []
    bad: list[tuple[int, int]] = []
    run = 0
    prev = RECESSIVE
    for i, bit in enumerate(bits):
        if (stuffed is not None and len(out) >= stuffed and run != STUFF_RUN):
            # past the CRC sequence and no stuff bit owed: the rest is fixed form. A run of
            # five that ends the CRC SEQUENCE still gets its stuffed complement first, so the
            # region is only over once that bit has been consumed (sec. 5).
            return out + list(bits[i:]), bad
        if run == STUFF_RUN:                      # this position must be a stuff bit
            if bit == prev:
                bad.append((i, bit))
            run = 1                               # the stuff bit starts the next run
            prev = bit
            continue
        out.append(bit)
        run = run + 1 if bit == prev else 1
        prev = bit
    return out, bad


def crc15(bits: Sequence[int]) -> int:
    """The 15-bit CRC of the destuffed stream SOF..end of DATA FIELD (sec. 3.1.1).

    This is Bosch's shift register verbatim: CRCNXT = NXTBIT XOR CRC(14); shift left one;
    XOR the generator 0x4599 when CRCNXT is set. `test_can.py` checks it against an
    independent polynomial division of the same bit string.
    """
    crc = 0
    for bit in bits:
        crcnxt = bit ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if crcnxt:
            crc ^= CRC_POLY
    return crc


def stuff_region(frame: Frame) -> int:
    """How many logical bits of `frame` are inside the stuffed region.

    sec. 5: "The frame segments START OF FRAME, ARBITRATION FIELD, CONTROL FIELD, DATA FIELD
    and CRC SEQUENCE are coded by the method of bit stuffing. ... The remaining bit fields
    of the DATA FRAME or REMOTE FRAME (CRC DELIMITER, ACK FIELD, and END OF FRAME) are of
    fixed form and not stuffed."

    SOF (1) + IDENTIFIER (11) + RTR (1) + two reserved bits + DATA LENGTH CODE (4) is 19 bits,
    then 8 per data byte -- none at all in a REMOTE FRAME, whatever its DLC says (sec. 3.1.2)
    -- then the 15 CRC bits. Everything from the CRC delimiter on is fixed form, so the seven
    recessive bits of END OF FRAME are seven recessive bits on the wire.
    """
    data_bits = 0 if frame.rtr else 8 * frame.dlc
    return ID_BITS + 8 + data_bits + CRC_BITS


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as Bus.ticks()."""
    return int(ceil(max(0.0, ns) / ns_per_tick))


@dataclass
class Frame:
    """One standard-format CAN frame as the application sees it."""

    identifier: int
    data: bytes = b""
    rtr: bool = False                    # True: REMOTE FRAME, which carries no data field
    dlc: Optional[int] = None            # None: use len(data)

    def __post_init__(self) -> None:
        self.identifier = int(self.identifier)
        self.data = bytes(self.data)
        if len(self.data) > MAX_DLC:
            raise ValueError(
                f"a classic CAN frame carries 0..{MAX_DLC} data bytes, got {len(self.data)}"
            )
        if not 0 <= self.identifier < (1 << ID_BITS):
            raise ValueError(
                f"a standard CAN identifier is {ID_BITS} bits, got {self.identifier:#x}"
            )
        if self.dlc is None:
            self.dlc = len(self.data)
        if not 0 <= self.dlc <= MAX_DLC:
            raise ValueError(f"DLC must be 0..{MAX_DLC}, got {self.dlc}")
        if self.rtr and self.data:
            raise ValueError("a REMOTE FRAME has no DATA FIELD (sec. 3.1.2)")

    def __str__(self) -> str:
        what = "remote" if self.rtr else "data"
        body = "" if self.rtr else " " + " ".join(f"{b:02x}" for b in self.data)
        return f"id={self.identifier:#05x} {what} dlc={self.dlc}{body}"


def _logical_bits(frame: Frame) -> tuple[list[int], list[str]]:
    """The destuffed bit stream of `frame` plus the role of every bit.

    The roles, in the order sec. 3.1.1 puts the fields on the wire, are "sof", "id", "rtr",
    "reserved", "dlc", "data", "crc", "crc_delim", "ack", "ack_delim" and "eof". The node
    plans its own drive from them, and the wire/stuff cut is taken from where "crc_delim"
    starts, because everything from there on is fixed form.

    CRC covers SOF through the end of the DATA FIELD (sec. 3.1.1), so it is computed over
    everything up to that point and then appended. For a REMOTE FRAME the DATA FIELD is
    absent and the DLC is still sent (sec. 3.1.2).
    """
    bits: list[int] = [DOMINANT]                                   # SOF
    roles: list[str] = ["sof"]
    for k in range(ID_BITS - 1, -1, -1):                           # ID-10 .. ID-0
        bits.append((frame.identifier >> k) & 1)
        roles.append("id")
    bits.append(RECESSIVE if frame.rtr else DOMINANT)              # RTR
    roles.append("rtr")
    bits += [DOMINANT, DOMINANT]                                   # r1, r0: reserved
    roles += ["reserved", "reserved"]
    for k in range(3, -1, -1):                                     # DLC3 .. DLC0
        bits.append((frame.dlc >> k) & 1)
        roles.append("dlc")
    payload = b"" if frame.rtr else frame.data
    for byte in payload:                                           # data, MSB first
        for k in range(7, -1, -1):
            bits.append((byte >> k) & 1)
            roles.append("data")
    body = list(bits)
    crc = crc15(body)
    for k in range(CRC_BITS - 1, -1, -1):
        bits.append((crc >> k) & 1)
        roles.append("crc")
    bits.append(RECESSIVE)                                         # CRC delimiter
    roles.append("crc_delim")
    bits.append(RECESSIVE)                                         # ACK slot: the transmitter
    roles.append("ack")                                            #   releases, receivers win
    bits.append(RECESSIVE)                                         # ACK delimiter
    roles.append("ack_delim")
    bits += [RECESSIVE] * 7                                        # END OF FRAME
    roles += ["eof"] * 7
    return bits, roles


def frame_bits(frame: Frame, stuff_bits: bool = True) -> list[int]:
    """The bits a transmitter drives for `frame`, stuffed by default (sec. 5).

    `stuff_bits=False` gives the logical stream, which is what a test uses to plant a frame
    that violates the stuffing rule or the CRC on purpose.
    """
    return wire_bits(frame, stuffed=stuff_bits)


def wire_bits(frame: Frame, mutate=None, stuffed: bool = True) -> list[int]:
    """The wire bits of `frame`, with `mutate` applied to the *logical* stream first.

    This is how a test plants one specific violation: `mutate` is called with the logical bit
    list (SOF first) and returns the list to transmit, so flipping one CRC bit or setting a
    reserved bit recessive needs no knowledge of where those bits are:

        wire_bits(frame, lambda b: b[:20] + [1] + b[21:])     # a broken reserved bit

    The cut between the stuffed and the fixed-form part is taken from the field roles, so it
    stays right even when `mutate` changes the DLC.
    """
    bits, roles = _logical_bits(frame)
    if mutate is not None:
        bits = list(mutate(bits))
    if not stuffed:
        return bits
    cut = roles.index("crc_delim")
    head = stuff(bits[:cut])
    return head + bits[cut:]


def plan(frame: Frame) -> tuple[list[int], list[str]]:
    """(stuffed bits, role of every bit) as the node drives them.

    Stuffing never changes which logical bit a wire bit came from, so the roles are matched
    back to the stuffed stream by re-running the same run-length walk `destuff` uses
    (`_roles_for_stuffed`), which keeps them aligned with the bit list.
    """
    bits, roles = _logical_bits(frame)
    cut = stuff_region(frame)
    head = stuff(bits[:cut])
    return head + bits[cut:], _roles_for_stuffed(head, roles[:cut]) + roles[cut:]


def _roles_for_stuffed(wire: Sequence[int], roles: Sequence[str]) -> list[str]:
    """One role per *wire* bit, matched to `roles` by re-running the same run-length walk.

    The inserted bits get the role "stuff": they carry no field, but a transmitter still has
    to know they are there, so the list is the same length as `wire`.
    """
    out: list[str] = []
    i = 0
    run = 0
    prev = RECESSIVE
    for bit in wire:
        if run == STUFF_RUN:
            out.append("stuff")
            run, prev = 1, bit
            continue
        out.append(roles[i])
        i += 1
        run = run + 1 if bit == prev else 1
        prev = bit
    return out


def parse_frame(bits: Sequence[int]) -> tuple[Optional[Frame], list[str]]:
    """Decode a destuffed bit stream into a `Frame`, with a list of framing problems.

    Only the structure is decoded here; the *rules* (reserved bits, DLC range, CRC match,
    delimiters, EOF) are what `CanChecker` checks as the bits arrive, so that it can report
    the tick of each one. This is the node's own after-the-fact decoder.
    """
    problems: list[str] = []
    if len(bits) < 1 + ID_BITS + 1 + 2 + 4 + CRC_BITS + 1 + 1 + 1 + 7:
        return None, [f"frame too short: {len(bits)} bits"]
    if bits[0] != DOMINANT:
        problems.append("SOF is not dominant")
    ident = 0
    for k, bit in enumerate(bits[1:1 + ID_BITS]):
        ident = (ident << 1) | bit
    i = 1 + ID_BITS
    rtr = bits[i] == RECESSIVE
    i += 1
    if bits[i] == RECESSIVE:
        problems.append("reserved bit r1 is recessive")
    if bits[i + 1] == RECESSIVE:
        problems.append("reserved bit r0 is recessive")
    i += 2
    dlc = 0
    for bit in bits[i:i + 4]:
        dlc = (dlc << 1) | bit
    i += 4
    if dlc > MAX_DLC:
        return None, problems + [f"DLC {dlc} is not an admissible data length"]
    if rtr:
        payload = b""
    else:
        payload = bytes(
            sum(bits[i + 8 * n + k] << (7 - k) for k in range(8))
            for n in range(dlc)
        )
        i += 8 * dlc
    computed = crc15(bits[:i])
    got = 0
    for bit in bits[i:i + CRC_BITS]:
        got = (got << 1) | bit
    if computed != got:
        problems.append(f"CRC {got:04x} does not match the computed {computed:04x}")
    i += CRC_BITS
    if bits[i] != RECESSIVE:
        problems.append("CRC delimiter is dominant")
    i += 1
    if bits[i] != DOMINANT:
        problems.append("ACK slot is recessive: nobody acknowledged")
    i += 2                                       # ACK slot + ACK delimiter
    if any(bit != RECESSIVE for bit in bits[i:i + 7]):
        problems.append("END OF FRAME has a dominant bit")
    frame = Frame(identifier=ident, data=payload, rtr=rtr, dlc=dlc)
    return frame, problems


@dataclass(frozen=True)
class CanTiming:
    """One CAN bit-timing configuration, in the spec's own segments.

    A bit time is SYNC_SEG + PROP_SEG + PHASE_SEG1 + PHASE_SEG2, measured in TIME QUANTA
    (sec. 8). SYNC_SEG is fixed at 1 TQ, PROP_SEG and PHASE_SEG1 are 1..8 TQ, PHASE_SEG2 is
    the maximum of PHASE_SEG1 and the INFORMATION PROCESSING TIME (<= 2 TQ), and a bit has
    8..25 TQ in total. The SAMPLE POINT sits at the end of PHASE_SEG1, so it is at
    `1 + prop_tq + phase1_tq` TQ from the start of the bit, and the RESYNCHRONIZATION JUMP
    WIDTH is 1..min(4, PHASE_SEG1) TQ.

    `tq_ns` is the quantum this configuration was designed with, and `ticks()` turns the
    whole thing into whole ticks of the bus clock. Because the tick grid rounds, the quantum
    and therefore the TQ count per bit can differ from the design; what never changes is
    the bit time and the sample point in ticks, which is all the models use.

    The segment defaults are those of the 125 kbit/s configuration in `CAN_BIT_TIMINGS` (16 TQ,
    sample point at 75 %), so a `CanTiming` built without them is a legal one rather than a
    plausible-looking illegal one. `CAN_BIT_TIMINGS` names every value anyway.
    """

    name: str
    bitrate: float          # NOMINAL BIT RATE, bit/s; bit time = 1 / bitrate
    tq_ns: float            # TIME QUANTUM of the design
    sync_tq: int = 1
    prop_tq: int = 7
    phase1_tq: int = 4
    phase2_tq: int = 4
    sjw_tq: int = 4

    @property
    def tq_per_bit(self) -> int:
        """TIME QUANTA in one bit time."""
        return self.sync_tq + self.prop_tq + self.phase1_tq + self.phase2_tq

    @property
    def sample_point_tq(self) -> int:
        """TQ from the start of the bit to the SAMPLE POINT (end of PHASE_SEG1)."""
        return self.sync_tq + self.prop_tq + self.phase1_tq

    @property
    def bit_ns(self) -> float:
        return 1e9 / self.bitrate

    def sample_point_fraction(self) -> float:
        return self.sample_point_tq / self.tq_per_bit

    def ticks(self, ns_per_tick: float = 20.0) -> dict:
        """Every length of this configuration in whole ticks of `ns_per_tick`.

        Returns `bit_ticks` (one bit time), `sample_point_ticks` (where inside the bit the bus
        is read), `sjw_ticks`, `tq_ticks` and `tq_per_bit`. The time quantum is rounded to
        whole ticks, which can only shift the sample point by less than half a tick -- 10 ns
        at 50 MHz -- so the checker is never lenient about the bit time, which is exact.
        """
        bit_ticks = int(round(self.bit_ns / ns_per_tick))
        tq_ticks = max(1, int(round(self.tq_ns / ns_per_tick)))
        if bit_ticks % tq_ticks:
            raise ValueError(
                f"{self.name}: {bit_ticks} ticks per bit is not a whole number of "
                f"{tq_ticks}-tick time quanta at {ns_per_tick} ns/tick; pick a speed whose "
                "time quantum divides the bit time on this tick grid"
            )
        return {
            "bit_ticks": bit_ticks,
            "sample_point_ticks": max(1, int(round(self.sample_point_fraction() * bit_ticks))),
            "sjw_ticks": max(1, int(round(self.sjw_tq / self.tq_per_bit * bit_ticks))),
            "tq_ticks": tq_ticks,
            "tq_per_bit": bit_ticks // tq_ticks,
            "bitrate": self.bitrate,
            "ns_per_tick": ns_per_tick,
            "speed": self.name,
        }


# The two speeds supported, on the 20 ns tick of core.py at 50 MHz.
#
# sec. 8 (`research/ref/protocols/can/04-bosch-bit-timing.txt:27`) makes PHASE_SEG2 "the maximum
# of PHASE_SEG1 and the INFORMATION PROCESSING TIME", which AN1798 Table 1 spells out as
# tPHASE_SEG2 = MAX(IPT, tPHASE_SEG1). The worked example of `05` sec. 2.2 (16 TQ, PROP_SEG 6,
# PHASE_SEG1 7, PHASE_SEG2 2) is therefore *not* what this model ships: it needs PHASE_SEG2 >=
# PHASE_SEG1, and it is still available through `timing_for(..., overrides=...)` if a test wants
# it -- `test_bit_timing_rejects_what_the_spec_forbids` moves the sample point that way.
#
#   125 kbit/s   8000 ns/bit = 16 TQ of 500 ns, SYNC_SEG 1, PROP_SEG 7, PHASE_SEG1 4,
#                PHASE_SEG2 4, SJW 4 -> sample point at 12 TQ = 75 % of the bit time
#   500 kbit/s   2000 ns/bit = 20 TQ of 100 ns, SYNC_SEG 1, PROP_SEG 8, PHASE_SEG1 5,
#                PHASE_SEG2 6, SJW 4 -> sample point at 14 TQ = 70 % of the bit time
#
# Both land on whole ticks of a 20 ns clock (25 and 5 ticks per TQ), so `CanTiming.ticks()` is
# exact for both and `check_bit_timing` accepts them. 70 % is as far up as 100 ns quanta can go
# with PROP_SEG <= 8 TQ and PHASE_SEG2 >= PHASE_SEG1; `research/ref/protocols/can/README.md`
# says so where a firmware author will read it.
CAN_BIT_TIMINGS: dict[str, CanTiming] = {
    "125k": CanTiming("125k", 125e3, tq_ns=500.0, prop_tq=7, phase1_tq=4, phase2_tq=4, sjw_tq=4),
    "500k": CanTiming("500k", 500e3, tq_ns=100.0, prop_tq=8, phase1_tq=5, phase2_tq=6, sjw_tq=4),
}


def check_bit_timing(timing: CanTiming) -> CanTiming:
    """Return `timing` if every limit of Bosch sec. 8 holds, else raise ValueError.

    The limits checked, all of them sec. 8 of `04-bosch-bit-timing.txt`:
    SYNC_SEG exactly 1 TQ; 8..25 TQ per bit; PROP_SEG and PHASE_SEG1 each 1..8 TQ; PHASE_SEG2
    at least the INFORMATION PROCESSING TIME of 2 TQ *and* at least PHASE_SEG1 -- "PHASE_SEG2 is
    the maximum of PHASE_SEG1 and the INFORMATION PROCESSING TIME", which
    `06-nxp-an1798-can-bit-timing-examples.txt` Table 1 writes tPHASE_SEG2 = MAX(IPT,
    tPHASE_SEG1); the sample point strictly inside the bit with at least one TQ of PHASE_SEG2
    after it; and the RESYNCHRONIZATION JUMP WIDTH 1..min(4, PHASE_SEG1).

    sec. 8 states no band for the sample point, only its position at the end of PHASE_SEG1; the
    75 %-87.5 % band of `05` sec. 1.3 and the 75 %-90 % band of `07` sec. 3.3 are application
    recommendations, not limits, so they are not enforced here. Both configurations in
    `CAN_BIT_TIMINGS` are inside `05`'s band at 125 kbit/s and below it at 500 kbit/s, which the
    research README explains.
    """
    problems = []
    if timing.sync_tq != 1:
        problems.append(f"SYNC_SEG is {timing.sync_tq} TQ, sec. 8 fixes it at 1 TQ")
    if not 8 <= timing.tq_per_bit <= 25:
        problems.append(f"{timing.tq_per_bit} TQ per bit, sec. 8 allows 8..25")
    for what, value in (("PROP_SEG", timing.prop_tq), ("PHASE_SEG1", timing.phase1_tq)):
        if not 1 <= value <= 8:
            problems.append(f"{what} is {value} TQ, sec. 8 allows 1..8")
    if timing.phase2_tq < 2:
        problems.append(
            f"PHASE_SEG2 is {timing.phase2_tq} TQ, less than the 2 TQ "
            "INFORMATION PROCESSING TIME of sec. 8"
        )
    if timing.phase2_tq < timing.phase1_tq:
        problems.append(
            f"PHASE_SEG2 is {timing.phase2_tq} TQ, below PHASE_SEG1 {timing.phase1_tq} TQ: "
            "sec. 8 makes PHASE_SEG2 the maximum of PHASE_SEG1 and the INFORMATION PROCESSING "
            "TIME, which 06-nxp-an1798-*.txt Table 1 writes tPHASE_SEG2 = MAX(IPT, tPHASE_SEG1)"
        )
    if not 1 <= timing.sjw_tq <= min(4, timing.phase1_tq):
        problems.append(
            f"SJW is {timing.sjw_tq} TQ, sec. 8 allows 1..min(4, PHASE_SEG1={timing.phase1_tq})"
        )
    if not timing.sync_tq + 1 < timing.sample_point_tq < timing.tq_per_bit:
        problems.append("the SAMPLE POINT is not strictly inside the bit time")
    if problems:
        raise ValueError(f"{timing.name}: " + "; ".join(problems))
    return timing


def timing_for(ns_per_tick: float = 20.0, speed: str = "125k",
                overrides: Optional[dict] = None) -> dict:
    """Every timing number of `speed` in whole ticks, plus the `CanTiming` behind it.

    `overrides` replaces individual values by their long names, which is how a test drives a
    node or a checker off-spec on purpose: {"bitrate": 1e6} makes it transmit too fast, and
    {"tq_per_bit": ...} / {"sample_point_tq": ...} move the quantum and the sample point.
    The sample point is the one to break when testing a receiver: sampling at the wrong point
    inside a bit is a real hardware bug the spec pins down, and this model can plant it.
    """
    try:
        base = CAN_BIT_TIMINGS[speed]
    except KeyError:
        raise ValueError(
            f"unknown CAN speed {speed!r}; pick one of {sorted(CAN_BIT_TIMINGS)}"
        ) from None
    overrides = dict(overrides or {})
    unknown = set(overrides) - {
        "bitrate", "tq_ns", "tq_per_bit", "sample_point_tq", "sjw_tq", "prop_tq",
        "phase1_tq", "phase2_tq", "sync_tq",
    }
    if unknown:
        raise ValueError(f"unknown timing override(s) {sorted(unknown)}")
    values = {
        "name": base.name, "bitrate": base.bitrate, "tq_ns": base.tq_ns,
        "sync_tq": base.sync_tq, "prop_tq": base.prop_tq, "phase1_tq": base.phase1_tq,
        "phase2_tq": base.phase2_tq, "sjw_tq": base.sjw_tq,
    }
    if "tq_per_bit" in overrides or "sample_point_tq" in overrides:
        tq = int(overrides.pop("tq_per_bit", base.tq_per_bit))
        sp = int(overrides.pop("sample_point_tq", base.sample_point_tq))
        if not 1 < sp < tq:
            raise ValueError(f"the sample point must sit inside the bit: {sp} of {tq} TQ")
        values.update(tq_ns=1e9 / values["bitrate"] / tq, prop_tq=0, phase1_tq=sp - 1,
                      phase2_tq=tq - sp)
    values.update(overrides)
    timing = CanTiming(**values)
    return dict(timing.ticks(ns_per_tick), timing=timing)


@dataclass(frozen=True)
class Violation:
    """One spec rule the bus did not meet, with the tick it was missed on."""

    tick: int
    rule: str            # "bit time", "stuff", "crc", "form", "ack", "intermission", ...
    measured: int        # what the bus did, in the rule's own unit
    required: int        # what the rule asks for
    detail: str = ""

    def __str__(self) -> str:
        return (f"t={self.tick} {self.rule} {self.measured} != {self.required}"
                + (f" ({self.detail})" if self.detail else ""))


@dataclass
class TxRecord:
    """One frame as the node put it on the wire (one record per transmission attempt)."""

    frame: Frame
    start_tick: int
    end_tick: Optional[int] = None
    acked: bool = False
    won_arb: bool = True           # False: lost arbitration and became a receiver
    error: str = ""

    @property
    def ok(self) -> bool:
        return self.end_tick is not None and self.acked and not self.error

    def __str__(self) -> str:
        what = "ok" if self.ok else (self.error or "incomplete")
        return f"t={self.start_tick} tx {self.frame} {what}"


@dataclass
class RxRecord:
    """One frame as the node received it."""

    tick: int                      # tick the SOF bit was sampled
    frame: Frame
    acked: bool = False            # this node drove the ACK slot dominant
    valid: bool = True             # no error up to the last-but-one EOF bit (sec. 4)
    error: str = ""

    def __str__(self) -> str:
        return (f"t={self.tick} rx {self.frame}"
                + (f" error={self.error}" if self.error else ""))


class CanNode(Device):
    """A CAN node: sends queued frames, arbitrates, receives, ACKs and counts its errors.

    Timing comes from the bus tick rate (bind it with `bus=`, see attach_node) and the bit
    timing of `speed`, so the same class is a 125 kbit/s or a 500 kbit/s node. Every node runs
    its own bit grid of `bit_ticks` ticks and reads the bus at `sample_point_ticks` into each
    bit, which is the SAMPLE POINT at the end of PHASE_SEG1 (sec. 8); what to drive for the
    next bit is decided at the bit boundary, which is a whole PHASE_SEG2 later and therefore
    always inside the INFORMATION PROCESSING TIME the spec allows.

    There is no propagation delay and no oscillator error in this model, so
    resynchronisation never fires and a node's bit grid stays where it was locked: onto its
    own transmission, or -- if it was attached while another node was already talking -- by
    HARD SYNCHRONIZATION onto the next recessive-to-dominant edge during BUS IDLE (sec. 8
    rule 3). A node that comes up mid-frame therefore joins the frame in progress, which is
    what a real receiver does.

    Fault confinement follows sec. 7: sending an ERROR FLAG as the transmitter adds 8 to the
    transmit error count, receiving an error adds 1 to the receive count, a successful
    transmission or reception takes 1 off, 128 makes the node error-passive (which sends a
    recessive PASSIVE ERROR FLAG and waits 8 extra recessive bits before its next frame) and
    256 makes it bus-off until it has seen 128 occurrences of 11 consecutive recessive bits.
    Rule 11 is the way back: with both counts at or below 127 the node is error-active again,
    so a node whose traffic improves owes no SUSPEND TRANSMISSION from that frame on.
    The two exceptions to rule 3 (error-passive transmitter with an ACKNOWLEDGMENT ERROR, and
    a STUFF ERROR in the arbitration field before the RTR bit) are not implemented; they only
    stop the counter from moving, never the wire.
    """

    name = "can_node"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        speed: str = "125k",
        timing: Optional[dict] = None,
        net: str = "can",
        max_attempts: int = 0,
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.speed = speed
        self.timing_overrides = dict(timing or {})
        self.net = net
        self.max_attempts = int(max_attempts)     # 0: retry for ever, as the spec does
        if name:
            self.name = name
        self.tk = timing_for(self.ns_per_tick, speed, self.timing_overrides)
        self.records: list[TxRecord] = []
        self.rx: list[RxRecord] = []
        self.errors: list[str] = []
        self.error_log: list[tuple] = []          # (tick, kind, detail)
        self._reset()

    def bind(self, bus: Bus) -> "CanNode":
        """Take the tick rate from `bus`. Done by the constructor when bus= is given."""
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.tk = timing_for(self.ns_per_tick, self.speed, self.timing_overrides)
        self._reset()
        return self

    def _reset(self) -> None:
        self.queue: list[Frame] = []
        self.state = "wait"          # wait | tx | rx | flag
        self.drive: Optional[int] = None
        self.origin = 0              # tick of bit 0's boundary
        self.phase = 0               # ticks into the current bit
        self._t = 0                  # last tick this node advanced on
        self.level = RECESSIVE       # what the node saw on the net last tick
        self.idle_bits = 0           # consecutive recessive bits seen (INTERMISSION)
        self.suspend = 0             # SUSPEND TRANSMISSION bits still owed (sec. 3.1.5)
        self.off_recovery = 0        # occurrences of 11 recessive bits seen (sec. 7 rule 12)
        self.off_run = 0
        self.tec = 0                 # TRANSMIT ERROR COUNT
        self.rec = 0                 # RECEIVE ERROR COUNT
        self.bus_off = False
        self.error_active = True
        self.rec_ = None             # the RxRecord being filled in
        self.rx_wire: list[int] = []
        self.rx_logical: list[int] = []
        self.rx_run = 0
        self.rx_prev = RECESSIVE
        self.rx_index = 0
        self.rx_dlc = 0
        self.rx_rtr = False
        self.rx_crc_ok = False
        self.rx_fault = ""           # first error seen while receiving this frame
        self.rx_armed = ""           # error whose ERROR FLAG waits for a later bit (CRC)
        self.rx_acked = False
        self.rx_monitored: list[int] = []
        self.wire: list[int] = []
        self.roles: list[str] = []
        self.wire_i = 0
        self.tx: Optional[TxRecord] = None
        self.flag_index = 0
        self.attempts = 0

    # ---- the application interface ------------------------------------------------
    def send(self, identifier, data: bytes = b"", rtr: bool = False) -> Frame:
        """Queue one DATA FRAME, or REMOTE FRAME with rtr=True. Frames go out in order.

        `identifier` may also be a `Frame`, which is how a REMOTE FRAME with a DLC but no
        data is queued: `node.send(Frame(0x7ff, rtr=True, dlc=2))`.
        """
        frame = identifier if isinstance(identifier, Frame) else Frame(identifier, data, rtr)
        self.queue.append(frame)
        return frame

    def send_frame(self, frame: Frame) -> Frame:
        self.queue.append(frame)
        return frame

    @property
    def pending(self) -> int:
        """Frames queued but not yet transmitted."""
        return len(self.queue)

    @property
    def busy(self) -> bool:
        return self.state != "wait" or bool(self.queue) or self.suspend > 0

    @property
    def idle(self) -> bool:
        return not self.busy

    @property
    def sent(self) -> list[TxRecord]:
        """Transmission attempts that reached the end of END OF FRAME."""
        return [r for r in self.records if r.end_tick is not None]

    @property
    def ok_frames(self) -> list[TxRecord]:
        return [r for r in self.records if r.ok]

    @property
    def error_passive(self) -> bool:
        return not self.error_active

    @property
    def state_name(self) -> str:
        return "bus-off" if self.bus_off else (
            "error-passive" if not self.error_active else self.state)

    def run(self, bus: Bus, limit: Optional[int] = None) -> bool:
        """Step `bus` until this node is idle. False if `limit` ticks ran out first.

        `limit` defaults to `ACK_TIMEOUT_TICKS` bit times at this node's own bit timing, which
        is what stops a bus that never frees itself from hanging a test for ever.
        """
        if limit is None:
            limit = ACK_TIMEOUT_TICKS * self.tk["bit_ticks"]
        return bus.run_until(lambda _b: self.idle, limit)

    # ---- error counters (sec. 7) --------------------------------------------------
    # `tec` and `rec` are the TRANSMIT and RECEIVE ERROR COUNTs and are public: a test that
    # provokes errors watches them move, and firmware does the same over a status register.

    def _bump_tec(self, amount: int) -> None:
        self.tec += amount
        self._reassess_state()

    def _bump_rec(self, amount: int) -> None:
        self.rec += amount
        self._reassess_state()

    def _reassess_state(self) -> None:
        """error-active -> error-passive at 128, error-passive -> bus-off at 256, and
        error-passive -> error-active again below 128 (sec. 7 rules 9, 10 and 11 of `03`)."""
        if self.tec >= 256 or self.rec >= 256:
            self.bus_off, self.error_active = True, False
            self.state = "wait"
            self.drive = None
            self._requeue()
            self.error_log.append((self._t, "bus-off", f"TEC={self.tec} REC={self.rec}"))
            return
        passive = self.tec >= 128 or self.rec >= 128
        if passive and self.error_active:
            self.error_active = False
            self.error_log.append((self._t, "error-passive", f"TEC={self.tec} REC={self.rec}"))
            return
        # sec. 7 rule 11 (`03`): "An 'error passive' node becomes 'error active' again when both
        # the TRANSMIT ERROR COUNT and the RECEIVE ERROR COUNT are less than or equal to 127".
        # The counters only come down one at a time (rules 7 and 8), so a node whose traffic
        # improves is error-active again after up to 128 acknowledged frames -- and owes no
        # SUSPEND TRANSMISSION on any frame from that one on, since `suspend` is only armed while
        # the node is error-passive.
        if not passive and not self.error_active and not self.bus_off:
            self.error_active = True
            self.error_log.append((self._t, "error-active", f"TEC={self.tec} REC={self.rec}"))

    def _requeue(self) -> None:
        """Put the frame that failed back at the head of the queue: CAN retransmits."""
        if self.tx is not None and self.tx.frame not in self.queue:
            self.queue.insert(0, self.tx.frame)

    # ---- the bit clock -------------------------------------------------------------
    def _drive(self) -> dict:
        """What this node puts on the net this tick: 0 dominant, None released.

        A bus-off node has its drivers off (sec. 7), so it releases everything; it still
        watches the bus, which is how it ever recovers.
        """
        return {self.net: None if self.bus_off else self.drive}

    def tick(self, t: int, pins) -> dict:
        # Advance first and return the level afterwards, so the level a bit boundary chose is
        # on the net *during* that bit: core.py resolves whatever a device returns for the
        # current tick at the end of it, so a node that decided one tick early would drive one
        # tick late and its last bit would fall outside the window a monitor gets.
        self._advance(t, pins)
        return self._drive()

    def _advance(self, t: int, pins) -> None:
        self._t = t
        level = pins[self.net]
        if self.phase == 0:
            self._bit_boundary(t, level)
        if self.phase == self.tk["sample_point_ticks"]:
            self._sample(t, level)
        self.level = level
        self.phase += 1
        if self.phase >= self.tk["bit_ticks"]:
            self.phase = 0

    def _bit_boundary(self, t: int, level: int) -> None:
        """Start of a bit: choose what to drive for the bit that begins now.

        Everything the node knows has been sampled by now -- the previous bit's SAMPLE POINT
        is a whole PHASE_SEG2 earlier -- so this is where the ACK decision for the coming ACK
        SLOT is taken, and where a transmitter starts a frame or gives up the bus.
        """
        state = self.state
        if state == "wait":
            if level == DOMINANT and self.level == RECESSIVE and not self.bus_off:
                # HARD SYNCHRONIZATION (sec. 8 rule 3): a recessive-to-dominant edge while
                # the bus is idle is somebody's SOF, and the bit time restarts on it.
                self.origin, self.phase = t - OBSERVATION_LATENCY, 0
                self.idle_bits = 0
                self.drive = None
                return
            self.drive = None
            if self.suspend > 0:
                self.suspend -= 1             # SUSPEND TRANSMISSION (sec. 3.1.5)
                return
            if (self.idle_bits >= 3 and self.queue and not self.bus_off
                    and not (self.max_attempts and self.attempts >= self.max_attempts)):
                self._start_tx(t)
            elif self.max_attempts and self.attempts >= self.max_attempts and self.queue:
                # a test hook: stop the spec's endless retransmission and record why
                self.errors.append(
                    f"{self.name}: gave up on {self.queue[0]} after {self.attempts} attempts "
                    f"(max_attempts={self.max_attempts})"
                )
                self.queue.clear()
            return
        if state == "flag":
            if self.flag_index < 6:
                # ACTIVE ERROR FLAG: 6 dominant bits. An error-passive node sends the
                # PASSIVE ERROR FLAG instead: 6 recessive bits, i.e. it just releases.
                self.drive = None if not self.error_active else DOMINANT
            else:
                self.drive = None               # ERROR DELIMITER: 8 recessive bits
            return
        if state == "tx":
            self.drive = DOMINANT if self.wire[self.wire_i] == DOMINANT else None
            return
        self.drive = self._rx_drive()           # state == "rx"

    def _sample(self, t: int, level: int) -> None:
        """The SAMPLE POINT of the current bit: read the bus and act on it."""
        state = self.state
        if state == "wait":
            if level == RECESSIVE:
                self.idle_bits += 1
                self._recovery(t, level)
            else:
                self.idle_bits = 0
                if not self.bus_off:
                    # somebody else's SOF: become a receiver and take that bit as the SOF
                    self._begin_rx(t)
                    self._feed_rx(level, t)
            return
        if state == "flag":
            self.flag_index += 1
            if self.flag_index >= 14:           # 6 flag bits + 8 delimiter bits
                self.state, self.drive = "wait", None
                self.idle_bits = 0
            return
        if state == "tx":
            self._sample_tx(t, level)
            return
        self._sample_rx(t, level)

    def _recovery(self, t: int, level: int) -> None:
        """sec. 7 rule 12: bus-off ends after 128 occurrences of 11 recessive bits."""
        if not self.bus_off:
            return
        self.off_run += 1
        if self.off_run >= 11:
            self.off_run = 0
            self.off_recovery += 1
            if self.off_recovery >= 128:
                self.tec = self.rec = 0
                self.bus_off, self.error_active = False, True
                self.error_log.append((t, "recovered", "128 x 11 recessive bits"))

    # ---- transmitting --------------------------------------------------------------
    def _start_tx(self, t: int) -> None:
        """Bus is free and a frame is queued: send it, starting with the SOF now."""
        frame = self.queue.pop(0)
        self.attempts += 1
        self.wire, self.roles = plan(frame)
        self.wire_i = 0
        self.tx = TxRecord(frame=frame, start_tick=t)
        self.records.append(self.tx)
        self.rx_monitored = []
        self.state, self.drive, self.idle_bits = "tx", DOMINANT, 0

    def _sample_tx(self, t: int, level: int) -> None:
        role, sent = self.roles[self.wire_i], self.wire[self.wire_i]
        self.rx_monitored.append(level)
        if sent == RECESSIVE and level == DOMINANT and role in ("sof", "id", "rtr"):
            self._lose_arb(t)                  # arbitration, not an error (sec. 2)
            return
        if role in ("crc_delim", "ack_delim", "eof"):
            if level == DOMINANT:
                self._fault(t, "form", f"{role} is dominant")
                return
        elif role == "ack":
            if level == RECESSIVE:
                self._fault(t, "ack", "no receiver acknowledged the frame")
                return
            self.tx.acked = True
        elif level != sent:
            self._fault(t, "bit", f"sent {'dominant' if sent == DOMINANT else 'recessive'}"
                                  f" but monitored {'dominant' if level == DOMINANT else 'recessive'}"
                                  f" in the {role} field")
            return
        self.wire_i += 1
        if self.wire_i >= len(self.wire):
            self._finish_tx(t)

    def _finish_tx(self, t: int) -> None:
        """END OF FRAME is over and nothing went wrong: the frame is valid for us."""
        self.tx.end_tick = t
        self.state, self.drive, self.idle_bits = "wait", None, 0
        self.tec = max(0, self.tec - 1)        # sec. 7 rule 7
        if not self.error_active:
            self.suspend = 8                   # SUSPEND TRANSMISSION (sec. 3.1.5)
        self._reassess_state()                 # sec. 7 rule 11: this may make us active again
        self.tx = None

    def _lose_arb(self, t: int) -> None:
        """A recessive bit of ours went out while the bus read dominant: we are a receiver.

        The unit "must withdraw without sending one more bit" (sec. 2), so it drives nothing
        from here on, and it must still receive the frame -- so the bits it already monitored
        while transmitting are replayed into its receive assembly.
        """
        self.tx.won_arb = False
        self.state, self.drive = "rx", None
        self._begin_rx(t)
        for level in self.rx_monitored:
            self._feed_rx(level, t)
        self.queue.insert(0, self.tx.frame)     # retransmit once the bus is free again
        self.tx = None

    def _fault(self, t: int, kind: str, detail: str) -> None:
        """Record an error and start signalling it: ERROR FLAG from the next bit (sec. 6.2).

        The flag is 6 dominant bits for an error-active node and 6 recessive bits for an
        error-passive one, so a fault detected by an error-passive node is not visible on the
        wire at all unless somebody else is signalling too. The frame goes back on the queue:
        CAN retransmits automatically once the bus is idle (sec. 4).
        """
        self.errors.append(f"{self.name}: {kind} error at t={t}: {detail}")
        self.error_log.append((t, kind, detail))
        if self.tx is not None:
            self.tx.error = kind
            self._requeue()
            self._bump_tec(8)                  # sec. 7 rule 3
        else:
            self._bump_rec(1)                  # sec. 7 rule 1
        self.drive = None
        if not self.bus_off:
            self.state, self.flag_index = "flag", 0

    # ---- receiving -----------------------------------------------------------------
    def _sample_rx(self, t: int, level: int) -> None:
        """The SAMPLE POINT of a bit while receiving: the bit goes into the frame."""
        self._feed_rx(level, t)

    def _begin_rx(self, t: int) -> None:
        """A frame is on the bus that we did not start: start receiving it."""
        self.state = "rx"
        self.rec_ = RxRecord(tick=t, frame=Frame(0, b"", dlc=0))
        self.rx_logical = []
        self.rx_run, self.rx_prev = 0, RECESSIVE
        self.rx_index, self.rx_dlc = 0, 0
        self.rx_crc_ok = False
        self.rx_fault = ""
        self.rx_armed = ""
        self.rx_acked = False
        self.idle_bits = 0

    def _rx_crc_end(self) -> int:
        """Logical index one past the last DATA FIELD bit: where the CRC sequence starts."""
        return ID_BITS + 8 + 8 * self.rx_dlc

    def _rx_ack_index(self) -> int:
        """Logical index of the ACK SLOT."""
        return self._rx_crc_end() + CRC_BITS + 1

    def _rx_eof_end(self) -> int:
        """Logical index one past the last END OF FRAME bit."""
        return self._rx_crc_end() + CRC_BITS + 3 + 7

    def _rx_role(self) -> str:
        """Which field the logical bit at `rx_index` belongs to."""
        i = self.rx_index
        crc_end = self._rx_crc_end()
        if i == 0:
            return "sof"
        if i <= ID_BITS:
            return "id"
        if i == ID_BITS + 1:
            return "rtr"
        if i <= ID_BITS + 3:
            return "reserved"
        if i <= ID_BITS + 7:
            return "dlc"
        if i < crc_end:
            return "data"
        if i < crc_end + CRC_BITS:
            return "crc"
        if i == crc_end + CRC_BITS:
            return "crc_delim"
        if i == crc_end + CRC_BITS + 1:
            return "ack"
        if i == crc_end + CRC_BITS + 2:
            return "ack_delim"
        if i < self._rx_eof_end():
            return "eof"
        return "beyond"

    def _rx_drive(self) -> Optional[int]:
        """What to drive for the bit starting now, as a receiver.

        The only bit a receiver drives is the ACK SLOT, and only if the CRC matched and
        nothing has gone wrong: "All stations having received the matching CRC SEQUENCE
        report this within the ACK SLOT" (sec. 3.1.1). It has to decide at the bit boundary
        before that slot, because the slot is on the wire while the ACK bit is being driven.
        """
        if (self.rx_index == self._rx_ack_index() and self.rx_crc_ok
                and not self.rx_fault and not self.bus_off):
            self.rx_acked = True
            return DOMINANT
        return None

    def _feed_rx(self, level: int, t: int) -> None:
        """One sampled bit of the frame we are receiving.

        The bits are destuffed on the way in (sec. 5): until the stuffing is undone the
        position of a field on the wire is unknown, so the receiver cannot know where the ACK
        SLOT is until it has seen the DATA LENGTH CODE. Six equal bits inside SOF..CRC
        SEQUENCE is a STUFF ERROR, and then the field positions are lost for good.
        """
        role = self._rx_role()
        if self.rx_index <= self._rx_crc_end() + CRC_BITS and self.rx_run == STUFF_RUN:
            if level == self.rx_prev:
                self._fault(t, "stuff", "6 equal bits inside SOF..CRC SEQUENCE")
                return
            self.rx_run, self.rx_prev = 1, level       # the stuffed bit starts a new run
            return
        self.rx_logical.append(level)
        self.rx_run = self.rx_run + 1 if level == self.rx_prev else 1
        self.rx_prev = level
        self.rx_index += 1
        self._check_rx(role, level, t)
        if role == "eof" and self.rx_index >= self._rx_eof_end():
            self._finish_rx(t)

    def _check_rx(self, role: str, level: int, t: int) -> None:
        """The rules a receiver checks on the bits it has just taken (sec. 6.1)."""
        if role == "rtr":
            self.rx_rtr = level == RECESSIVE
            return
        if role == "reserved" and level != DOMINANT:
            # r1 is the first reserved bit of the CONTROL FIELD and r0 the second (sec.
            # 3.1.1); rx_index has already counted the bit just taken, so it is one past it
            which = "r1" if self.rx_index == ID_BITS + 3 else "r0"
            self._fault(t, "form", f"reserved bit {which} is recessive")
            return
        if role == "dlc" and self.rx_index == ID_BITS + 8:
            self.rx_dlc = 0
            for bit in self.rx_logical[ID_BITS + 4:ID_BITS + 8]:
                self.rx_dlc = (self.rx_dlc << 1) | bit
            if self.rx_dlc > MAX_DLC:
                self._fault(t, "form", f"DLC {self.rx_dlc} is not an admissible data length")
                return
            if self.rx_rtr:
                self.rx_dlc = 0          # no DATA FIELD at all in a REMOTE FRAME (sec. 3.1.2)
        if role == "crc" and self.rx_index == self._rx_crc_end() + CRC_BITS:
            computed = crc15(self.rx_logical[:self._rx_crc_end()])
            got = 0
            for bit in self.rx_logical[self._rx_crc_end():self.rx_index]:
                got = (got << 1) | bit
            self.rx_crc_ok = computed == got
            if not self.rx_crc_ok:
                # sec. 6.2: a CRC ERROR FLAG starts at the bit after the ACK DELIMITER, and
                # this node does not acknowledge a frame whose CRC did not match.
                self.rx_fault = "crc"
                self.rx_armed = "crc"
                self.errors.append(f"{self.name}: crc error at t={t}: received CRC "
                                   f"{got:04x}, computed {computed:04x}")
                self.error_log.append((t, "crc", f"received {got:04x} computed {computed:04x}"))
            return
        if role == "crc_delim" and level != RECESSIVE:
            self._fault(t, "form", "CRC delimiter is dominant")
            return
        if role == "ack_delim":
            if level != RECESSIVE:
                self._fault(t, "form", "ACK delimiter is dominant")
                return
            if self.rx_armed:
                self._fault(t, self.rx_armed, "flagged after the ACK delimiter (sec. 6.2)")
                return
        if role == "eof" and level == DOMINANT:
            last = self.rx_index >= self._rx_eof_end()
            # sec. 4: a receiver's message is valid up to the last but one EOF bit, so a
            # dominant bit in the very last one invalidates it without being an error.
            self.rx_fault = self.rx_fault or ("form: dominant bit in END OF FRAME" if not last
                                             else "dominant bit in the last EOF bit")
            if not last:
                self._fault(t, "form", "dominant bit inside END OF FRAME")

    def _finish_rx(self, t: int) -> None:
        """END OF FRAME is over: file the frame and hand the bus back to the wait state."""
        frame, problems = parse_frame(self.rx_logical)
        rec = self.rec_
        if frame is not None:
            rec.frame = frame
        rec.acked = self.rx_acked
        rec.valid = not problems and not self.rx_fault
        rec.error = "; ".join(p for p in ([self.rx_fault] if self.rx_fault else []) + problems)
        self.rx.append(rec)
        if rec.valid and rec.acked:
            self.rec = max(0, self.rec - 1)         # sec. 7 rule 8
        self._reassess_state()                       # sec. 7 rule 11: this may make us active
        self.rec_, self.state, self.drive, self.idle_bits = None, "wait", None, 0


def _ensure_net(bus: Bus, net: str = "can") -> None:
    """Add the open-drain CAN net if it is not there yet."""
    if net not in bus.nets:
        bus.add_net(net)                            # wired-AND with a pull-up: sec. 2


def attach_node(bus: Bus, **kwargs) -> CanNode:
    """Build a CanNode that takes its bit timing from `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "can"))
    node = CanNode(bus=bus, **kwargs)
    bus.attach(node)
    return node


@dataclass
class CanFrameReport:
    """One frame the checker saw, with everything it decoded and everything wrong with it."""

    tick: int                  # tick the SAMPLE POINT of the SOF bit
    identifier: Optional[int] = None
    dlc: Optional[int] = None
    data: bytes = b""
    rtr: bool = False
    acked: Optional[bool] = None
    bits: int = 0              # sampled bits in the frame, SOF..last EOF
    problems: list[str] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return self.identifier is not None and not self.problems

    def __str__(self) -> str:
        if self.identifier is None:
            return f"t={self.tick} undecodable frame ({'; '.join(self.problems)})"
        body = "" if self.rtr else " " + " ".join(f"{b:02x}" for b in self.data)
        return (f"t={self.tick} id={self.identifier:#05x} "
                f"{'remote' if self.rtr else 'data'} dlc={self.dlc}{body}"
                + (" ACK" if self.acked else "")
                + (f" [{'; '.join(self.problems)}]" if self.problems else ""))


class CanChecker(Device):
    """Passive CAN bus monitor: decodes the wire and flags every rule it breaks.

    It drives nothing and keeps no state shared with any node: it finds the SOF itself,
    locks its own bit grid on that edge, samples every bit at the sample point, measures
    every edge against the nominal bit time and checks every rule of the frame as the bits
    arrive, so each `Violation` carries the tick the rule was missed on.

    Rules, with the section of Bosch CAN Specification 2.0 Part A each comes from:

      bit time      sec. 8, NOMINAL BIT TIME = 1 / bit rate: every edge must land on the
                   nominal bit grid, within `edge_tol` ticks. The one tick of core.py's
                   observation delay (OBSERVATION_LATENCY) is taken out before the
                   comparison, so a legal transmitter measures 0 and the default slack of
                   one tick is pure quantisation. Anything more means the bit time, or the
                   phase of the drive, is wrong.
      stuff         sec. 5 and 6.1: six equal bits inside SOF..CRC SEQUENCE.
      crc           sec. 3.1.1 and 6.1: the CRC recomputed over SOF..end of DATA FIELD
                   against the 15 bits on the wire.
      form          sec. 6.1: a fixed-form field holding an illegal bit -- r1/r0 must be
                   dominant, DLC must be 0..8, and the CRC delimiter, the ACK delimiter and
                   all seven END OF FRAME bits must be recessive -- plus sec. 3.1.3: an ERROR
                   or OVERLOAD FLAG longer than the twelve bits superposed flags may add up
                   to, which is reported as `form` because it is a fixed-form violation too.
      ack           sec. 3.1.1 and 6.1: the ACK SLOT must read dominant, so somebody
                   acknowledged the frame.
      intermission  sec. 3.1.5: three recessive bits after a frame, and no SOF inside them.
      priority      sec. 3.1.1: ID-10..ID-4 must not be all recessive. Reported as its own
                   rule because a real controller accepts such a frame and simply gives it
                   the lowest priority, rather than treating it as a FORM ERROR.
      delimiter     sec. 3.1.3 and 3.1.4: the eight recessive bits of an ERROR or OVERLOAD
                   delimiter after a six-bit flag.

    The monitor is strict about what it can measure and silent about what it cannot: an edge
    is only measured while a frame is being decoded, and a frame with no edges in it (all its
    bits equal) gives nothing to measure, so `edges` and `worst_off_grid` say how much was
    actually looked at.

    One case it cannot decide from the wire alone, and handles by waiting: a dominant bit in
    END OF FRAME is a FORM ERROR, but six dominant bits in a row there are an ERROR FLAG,
    and sec. 6.2 says a form error in that field signals the flag at the *next* bit. Such a
    report is therefore held in `pending_form` until the run turns out to be six bits or
    less -- so the checker names the flag when a node signals one and the FORM ERROR when
    nothing does, which is what a real analyser shows.
    """

    name = "can_check"

    def __init__(
        self,
        bus: Optional[Bus] = None,
        ns_per_tick: float = 20.0,
        speed: str = "125k",
        timing: Optional[dict] = None,
        net: str = "can",
        edge_tol: int = 1,
        check_ack: bool = True,
        name: Optional[str] = None,
    ):
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick if bus is not None else ns_per_tick
        self.speed = speed
        self.timing_overrides = dict(timing or {})
        self.net = net
        self.edge_tol = int(edge_tol)
        self.check_ack = bool(check_ack)
        if name:
            self.name = name
        self.tk = timing_for(self.ns_per_tick, speed, self.timing_overrides)
        self.violations: list[Violation] = []
        self.frames: list[CanFrameReport] = []
        #: every ERROR or OVERLOAD flag seen, as (tick, dominant bits, delimiter bits)
        self.error_flags: list[tuple[int, int, int]] = []
        self._reset()

    def bind(self, bus: Bus) -> "CanChecker":
        self.bus = bus
        self.ns_per_tick = bus.ns_per_tick
        self.tk = timing_for(self.ns_per_tick, self.speed, self.timing_overrides)
        return self

    def _reset(self) -> None:
        self.level = RECESSIVE
        self.origin = None         # tick of bit 0's boundary of the frame in progress
        self.next_sample = self.tk["sample_point_ticks"]   # tick of the next SAMPLE POINT
        self.idle_bits = 0         # recessive bits seen with no frame in progress
        self.edges = 0             # edges the bit-time rule has looked at
        self.worst_off_grid = 0     # worst distance an edge sat from the nominal grid
        self.bits = 0
        self.saw_frame = False     # a frame has been seen: INTERMISSION applies from now on
        self.state = "idle"        # idle | frame | flag | delim
        self.logical: list[int] = []
        self.run = 0
        self.prev = RECESSIVE
        self.flag_bits = 0
        self.delim_bits = 0
        self.rep: Optional[CanFrameReport] = None
        self._crc_end = ID_BITS + 8
        self._dlc = self._drc = self._ident = self._id_recessive = 0
        self._data: list[int] = []
        self._ack_seen: Optional[bool] = None
        self.pending_form: Optional[tuple[int, str]] = None

    # ---- what the tests ask for -------------------------------------------------
    @property
    def clean(self) -> bool:
        return not self.violations

    @property
    def ids(self) -> list[int]:
        """Identifiers of every frame decoded, in order."""
        return [f.identifier for f in self.frames if f.identifier is not None]

    def violations_by_rule(self, rule: str) -> list[Violation]:
        return [v for v in self.violations if v.rule == rule]

    def report(self, limit: int = 10) -> str:
        head = (f"{self.speed} at {self.ns_per_tick} ns/tick "
                f"({self.tk['bit_ticks']} ticks per bit, sample point at tick "
                f"{self.tk['sample_point_ticks']}): {len(self.frames)} frame(s), "
                f"{self.edges} edge(s) measured, {len(self.error_flags)} error/overload "
                f"flag(s), {len(self.violations)} violation(s)")
        return head + "".join(f"\n  {v}" for v in self.violations[:limit])

    def _flag(self, t: int, rule: str, measured: int, required: int, detail: str) -> None:
        """Record one broken rule. The caller has already compared measured against required."""
        self.violations.append(Violation(t, rule, measured, required, detail))

    def _defer_form(self, t: int, detail: str) -> None:
        """A dominant bit in a fixed-form field is a FORM ERROR, unless six of them in a row
        turn out to be an ERROR FLAG (sec. 3.1.3), so the report waits until we know which."""
        if self.pending_form is None:
            self.pending_form = (t, detail)

    def _flush_form(self) -> None:
        """Report the held FORM ERROR, now that no ERROR FLAG turned up after it."""
        if self.pending_form is not None:
            tick, detail = self.pending_form
            self._flag(tick, "form", DOMINANT, RECESSIVE, detail)
            self._problem(detail)
            self.pending_form = None

    def _problem(self, text: str) -> None:
        """Note a problem with the frame in progress; it is reported with the frame."""
        if self.rep is not None and text not in self.rep.problems:
            self.rep.problems.append(text)

    # ---- the monitor -------------------------------------------------------------
    def tick(self, t: int, pins) -> dict:
        """One tick of observation: notice edges, and take the SAMPLE POINT when it is due.

        The sample ticks come straight off the grid rather than off a counter, so the checker
        samples bit k at `origin + k * bit_ticks + sample_point_ticks` exactly -- the same
        tick the transmitter sampled it at, which is what lets the last bit of a frame be
        checked before the transmitter declares the frame finished.
        """
        level = pins[self.net]
        if level != self.level:
            self._edge(t, level)
        self.level = level
        if t >= self.next_sample:
            if self.origin is not None:
                self._sample(t, level)
            elif level == RECESSIVE:
                self.idle_bits += 1      # counting the INTERMISSION of sec. 3.1.5, in bits
            self.next_sample += self.tk["bit_ticks"]
        return {}

    def _edge(self, t: int, level: int) -> None:
        """A transition on the net: measure it, and look for a SOF or a flag."""
        if self.origin is None:
            if level == DOMINANT:
                # sec. 3.1.5: INTERMISSION is three recessive bits and "during INTERMISSION no
                # station is allowed to start transmission", so a SOF inside those three is a
                # violation rather than the start of a frame
                if self.saw_frame and self.idle_bits < 3:
                    self._flag(t, "intermission", self.idle_bits, 3,
                               f"SOF after {self.idle_bits} recessive bit(s): INTERMISSION "
                               "is three recessive bits")
                self._start_frame(t)             # a SOF, or the flag of a broken frame
            return
        if self.state in ("flag", "delim"):
            return                              # the flag and its delimiter are counted

        self._measure_edge(t)                  # at their SAMPLE POINTs, not at the edges

    def _measure_edge(self, t: int) -> None:
        """bit time: an edge must fall on the nominal grid of the frame in progress."""
        self.edges += 1
        bit_ticks = self.tk["bit_ticks"]
        off = (t - OBSERVATION_LATENCY - self.origin) % bit_ticks
        self.worst_off_grid = max(self.worst_off_grid, off)
        if off > self.edge_tol:
            self._flag(t, "bit time", off, self.edge_tol,
                       f"edge {off} tick(s) past a bit boundary of {bit_ticks} ticks, grid "
                       f"locked at t={self.origin}")

    def _start_frame(self, t: int) -> None:
        """Lock the bit grid on this dominant edge and start a fresh frame report.

        Hard synchronisation (sec. 8 rule 3): the SOF edge restarts the bit time, so this is
        where the checker's grid comes from and not from anything a node told it.
        """
        self._flush_form()               # a held FORM ERROR from the frame before this one
        self.origin = t - OBSERVATION_LATENCY
        self.next_sample = max(t + 1, self.origin + self.tk["sample_point_ticks"])
        self.bits, self.logical, self.run, self.prev = 0, [], 0, RECESSIVE
        self.rep = CanFrameReport(tick=t)
        self.saw_frame = True
        self.state = "frame"
        self.flag_bits = self.delim_bits = 0
        self._crc_end = ID_BITS + 8
        self._dlc = self._drc = self._ident = self._id_recessive = 0
        self._data = []
        self._ack_seen = None
        self.pending_form = None

    def _role(self, i: int) -> str:
        """Which field the logical bit at index `i` belongs to (sec. 3.1.1, field order)."""
        crc_end = self._crc_end
        if i == 0:
            return "sof"
        if i <= ID_BITS:
            return "id"
        if i == ID_BITS + 1:
            return "rtr"
        if i <= ID_BITS + 3:
            return "reserved"
        if i <= ID_BITS + 7:
            return "dlc"
        if i < crc_end:
            return "data"
        if i < crc_end + CRC_BITS:
            return "crc"
        if i == crc_end + CRC_BITS:
            return "crc_delim"
        if i == crc_end + CRC_BITS + 1:
            return "ack"
        if i == crc_end + CRC_BITS + 2:
            return "ack_delim"
        if i < crc_end + CRC_BITS + 3 + 7:
            return "eof"
        return "beyond"

    def _sample(self, t: int, level: int) -> None:
        """The SAMPLE POINT of a bit: read the level and check the rules that apply to it."""
        self.bits += 1
        if self.state == "flag":
            self._sample_flag(t, level)
            return
        if self.state == "delim":
            self._sample_delim(t, level)
            return
        role = self._role(len(self.logical))
        if len(self.logical) <= self._crc_end + CRC_BITS and self.run == STUFF_RUN:
            # this position must carry the stuffed complement of the run before it
            if level == self.prev:
                self._flag(t, "stuff", STUFF_RUN + 1, STUFF_RUN,
                           f"{STUFF_RUN + 1} equal bits inside SOF..CRC SEQUENCE")
                self._problem("stuff error inside the stuffed region")
            self.run, self.prev = 1, level
            return
        self.logical.append(level)
        self.run = self.run + 1 if level == self.prev else 1
        self.prev = level
        self._check(t, role, level)
        if role == "eof":
            if level == DOMINANT:
                self._defer_form(t, "a dominant bit inside END OF FRAME")
                if self.run >= STUFF_RUN + 1:
                    # six dominant bits where the frame was supposed to end: an ERROR or
                    # OVERLOAD FLAG, six dominant bits plus an eight-bit delimiter
                    # (sec. 3.1.3, 3.1.4). Only a dominant run can be a flag: the ten
                    # recessive bits of a healthy tail are not.
                    self.pending_form = None
                    self._problem("frame replaced by an error/overload flag after the CRC")
                    self.state, self.flag_bits, self.delim_bits = "flag", self.run, 0
                    return
            if len(self.logical) >= self._crc_end + CRC_BITS + 3 + 7:
                self._end_frame(t)

    def _check(self, t: int, role: str, level: int) -> None:
        """Every rule that can be decided from this bit (all of sec. 6.1)."""
        i = len(self.logical) - 1
        if role == "sof" and level != DOMINANT:
            self._flag(t, "form", level, DOMINANT, "SOF is not dominant")
            self._problem("SOF is not dominant")
            return
        if role == "id":
            self._ident = (self._ident << 1) | level
            if i <= 7 and level == RECESSIVE:      # the 7 bits ID-10 .. ID-4
                self._id_recessive += 1
            if i == ID_BITS:
                self.rep.identifier = self._ident
                if self._id_recessive == 7:
                    self._flag(t, "priority", self._id_recessive, 0,
                               "ID-10..ID-4 are all recessive")
                    self._problem("the 7 most significant identifier bits are all recessive")
            return
        if role == "rtr":
            self.rep.rtr = level == RECESSIVE
            return
        if role == "reserved" and level != DOMINANT:
            self._flag(t, "form", level, DOMINANT,
                       f"reserved bit r{ID_BITS + 3 - i} is recessive")
            self._problem("a reserved control bit is recessive")
            return
        if role == "dlc":
            if level:
                self._dlc = (self._dlc << 1) | 1
            else:
                self._dlc <<= 1
            if i == ID_BITS + 7:
                self.rep.dlc = self._dlc
                if self._dlc > MAX_DLC:
                    self._flag(t, "form", self._dlc, MAX_DLC,
                               f"DLC {self._dlc} is not an admissible data length")
                    self._problem(f"DLC {self._dlc} is not an admissible data length")
                    return
                # no DATA FIELD at all in a REMOTE FRAME, whatever the DLC says (sec. 3.1.2)
                self._crc_end = ID_BITS + 8 + (0 if self.rep.rtr else 8 * self._dlc)
            return
        if role == "data":
            if self._drc < 8 * MAX_DLC:
                self._data.append(level)
            self._drc += 1
            return
        if role == "crc" and i == self._crc_end + CRC_BITS - 1:
            computed = crc15(self.logical[:self._crc_end])
            got = 0
            for bit in self.logical[self._crc_end:]:
                got = (got << 1) | bit
            if computed != got:
                self._flag(t, "crc", got, computed,
                           f"CRC over SOF..DATA FIELD is {computed:04x}")
                self._problem(f"CRC {got:04x} does not match the computed {computed:04x}")
            # a DLC the spec does not allow may ask for more bytes than are there, so only
            # whole bytes that actually arrived are reported
            whole = min(self.rep.dlc or 0, len(self._data) // 8)
            self.rep.data = b"" if self.rep.rtr else bytes(
                sum(self._data[8 * n + k] << (7 - k) for k in range(8))
                for n in range(whole)
            )
            return
        if role == "crc_delim" and level != RECESSIVE:
            # sec. 6.2: a FORM ERROR here flags at the *next* bit, so this bit itself is
            # always the violation and never the first bit of somebody else's flag
            self._flag(t, "form", DOMINANT, RECESSIVE, "the CRC delimiter is dominant")
            self._problem("the CRC delimiter is dominant")
            return
        if role == "ack":
            self._ack_seen = level == DOMINANT
            return
        if role == "ack_delim":
            if level != RECESSIVE:
                self._flag(t, "form", DOMINANT, RECESSIVE,
                           "the ACK delimiter is dominant")
                self._problem("the ACK delimiter is dominant")
                return
            if self.check_ack and self._ack_seen is False:
                self._flag(t, "ack", RECESSIVE, DOMINANT,
                           "the ACK SLOT was recessive: no receiver acknowledged")
                self._problem("nobody acknowledged the frame")
            return

    def _end_frame(self, t: int) -> None:
        """Seven recessive bits in a row where END OF FRAME belongs: the frame is over."""
        self._flush_form()
        if self.rep is not None:
            self.rep.bits = len(self.logical)
            self.rep.acked = self._ack_seen
            self.frames.append(self.rep)
        self.rep, self.origin = None, None
        self.state, self.idle_bits = "idle", 0
        # The grid needs no fixing up here: `tick` has already put the next sample one bit
        # time ahead, so the SAMPLE POINT of the first bit of the INTERMISSION is where the
        # next frame bit would have been and `idle_bits` counts intermission *bits*.

    def _sample_flag(self, t: int, level: int) -> None:
        """ERROR or OVERLOAD FLAG: six dominant bits (sec. 3.1.3 and 3.1.4).

        Six error flags superposed may make up to twelve dominant bits, so a run longer than
        that is a violation; a shorter one is not, because the checker only ever enters this
        state on a run of six or more (see `_sample`, which is where a flag is recognised).
        """
        if level != DOMINANT:
            self.state, self.delim_bits = "delim", 1
            return
        self.flag_bits += 1
        if self.flag_bits > 12:
            self._flag(t, "form", self.flag_bits, 12,
                       "more than twelve dominant bits in the error flag")
            self._problem("more than twelve dominant bits in the error flag")

    def _sample_delim(self, t: int, level: int) -> None:
        """ERROR or OVERLOAD DELIMITER: eight recessive bits after the flag."""
        if level != RECESSIVE:
            self.delim_bits = 0
            self._flag(t, "delimiter", 0, 8, "a dominant bit inside the 8-bit delimiter")
            self._problem("a dominant bit inside the error/overload delimiter")
            return
        self.delim_bits += 1
        if self.delim_bits >= 8:
            self.error_flags.append((t, self.flag_bits, self.delim_bits))
            self.rep, self.origin = None, None
            self.state, self.idle_bits = "idle", 0


def attach_checker(bus: Bus, **kwargs) -> CanChecker:
    """Build a passive CanChecker for `bus` and attach it."""
    _ensure_net(bus, kwargs.get("net", "can"))
    chk = CanChecker(bus=bus, **kwargs)
    bus.attach(chk)
    return chk
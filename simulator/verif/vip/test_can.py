"""Self-tests for the pin-level classic CAN 2.0A models: `make vip-selftest VIP=can`.

Everything runs at 50 MHz (20 ns per tick) at 125 kbit/s and 500 kbit/s, with `CanNode`s
talking to each other on the one open-drain net and a passive `CanChecker` watching the pins.

Three kinds of test, and the third is the one that matters most:

  * the codec and the bit-timing table are checked against the spec text saved in
    `research/ref/protocols/can/` -- the CRC against an independent polynomial division, the
    frame layout field by field, the timing table against the sec. 8 limits;
  * two nodes talk to each other: arbitration, reception, acknowledgement, retransmission
    after an error, fault confinement, and the checker stays silent on all of it;
  * every rule the checker enforces is broken on purpose, one at a time, by a `WirePlayer`
    that replays a crafted bit sequence, and each broken frame must be flagged with that rule
    and its tick. `test_the_checker_is_not_vacuous` proves the same planted faults are seen by
    a `CanNode` too, so a checker that quietly stopped looking could not pass this file.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.can import (
    ACK_TIMEOUT_TICKS,
    CAN_BIT_TIMINGS,
    CRC_POLY,
    DOMINANT,
    MAX_DLC,
    RECESSIVE,
    STUFF_RUN,
    CanChecker,
    CanNode,
    Frame,
    ID_BITS,
    attach_checker,
    attach_node,
    check_bit_timing,
    crc15,
    destuff,
    frame_bits,
    ns_to_ticks,
    parse_frame,
    plan,
    stuff,
    stuff_region,
    timing_for,
    wire_bits,
)

SPEEDS = ["125k", "500k"]      # the two supported rates
NS_PER_TICK = 20.0             # 50 MHz chip clock


# ---- the bench -----------------------------------------------------------------------
def rig(speed="125k", count=2, check=True, check_kwargs=None, node_kwargs=None):
    """A bus with `count` CanNodes at `speed` and a passive checker looking at the net.

    A CAN bus needs at least two nodes for anything to work: a frame nobody receives is an
    ACKNOWLEDGMENT ERROR, so a one-node bench retransmits forever. That is what the fault
    confinement test wants, and nothing else does.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    node_kwargs = dict(node_kwargs or {})
    nodes = [attach_node(bus, name=f"n{i}", speed=speed, **node_kwargs)
             for i in range(count)]
    chk = attach_checker(bus, speed=speed, **(check_kwargs or {})) if check else None
    return bus, nodes, chk


def settled(*nodes, bus=None, limit=4_000_000):
    """Run the bus until every node given is idle, and assert they all got there."""
    assert bus.run_until(lambda _b: all(n.idle for n in nodes), limit), \
        f"{[n.state_name for n in nodes]} still busy after {limit} ticks"
    for node in nodes:
        assert not node.errors, node.errors[:3]


class WirePlayer(Device):
    """Replays a crafted bit sequence on the net, to plant one specific violation.

    It is not a CAN node: it has no bit timing of its own beyond the `bit_ticks` it is given,
    no acknowledgement and no error handling, so whatever it puts on the wire is exactly the
    list of bits it was handed. `planted()` builds those bits from a legal frame with one
    thing changed, and `run_wire()` hangs it off a bus with a checker and nothing else.
    """

    name = "wire"

    def __init__(self, bits, bit_ticks, start_tick=0, net="can"):
        self.bits = list(bits)
        self.bit_ticks = int(bit_ticks)
        self.start_tick = int(start_tick)
        self.net = net

    def tick(self, t, pins):
        if t < self.start_tick:
            return {}
        index = (t - self.start_tick) // self.bit_ticks
        if index >= len(self.bits):
            return {}
        return {self.net: DOMINANT if self.bits[index] == DOMINANT else None}


class ShortSofPlayer(Device):
    """Replays a frame whose SOF is held for `sof_ticks` ticks instead of a whole bit time.

    `WirePlayer` can only plant a fault that is a bit on the wire, so it cannot make the checker
    see a recessive START OF FRAME: the net goes dominant on the edge that starts the frame and
    stays dominant for the bit. A node that releases before the SAMPLE POINT is the only way to
    reach the checker's "SOF is not dominant" rule, so it needs a player of its own. `bits` is
    the frame *without* its SOF, which is what follows.
    """

    name = "short_sof"

    def __init__(self, bits, bit_ticks, sof_ticks, net="can"):
        self.bits = list(bits)
        self.bit_ticks = int(bit_ticks)
        self.sof_ticks = int(sof_ticks)
        self.net = net

    def tick(self, t, pins):
        if t < self.sof_ticks:
            return {self.net: DOMINANT}
        index = (t - self.sof_ticks) // self.bit_ticks
        if index >= len(self.bits):
            return {}
        return {self.net: DOMINANT if self.bits[index] == DOMINANT else None}


def planted(frame=None, mutate=None, stuff_bits=True):
    """The wire bits of a legal frame with one thing changed by `mutate`.

    `mutate` gets the *logical* bit list (SOF first, unstuffed) and returns the list to send,
    so a test can flip the CRC or make a reserved bit recessive without knowing where those
    bits are. `wire_bits` re-stuffs the result, so a mutation that breaks the stuffing rule
    itself still comes out as a legal-looking wire stream.
    """
    return wire_bits(frame or Frame(0x123, b"\xde\xad\xbe\xef"), mutate, stuffed=stuff_bits)


def run_wire(bits, speed="500k", bit_ticks=None, check_ack=False, check_kwargs=None,
             extra_bits=40):
    """Play `bits` on an otherwise empty bus and return the checker that watched them."""
    if bit_ticks is None:
        bit_ticks = timing_for(NS_PER_TICK, speed)["bit_ticks"]
    bus = Bus(ns_per_tick=NS_PER_TICK)
    chk = attach_checker(bus, speed=speed, check_ack=check_ack, **(check_kwargs or {}))
    bus.attach(WirePlayer(bits, bit_ticks))
    bus.run(bit_ticks * (len(bits) + extra_bits))
    return bus, chk


def flip(index):
    """A mutation that inverts one logical bit."""
    return lambda bits: bits[:index] + [1 - bits[index]] + bits[index + 1:]


def setbit(index, level):
    """A mutation that forces one logical bit to `level`."""
    return lambda bits: bits[:index] + [level] + bits[index + 1:]


# ---- the codec, against the spec -----------------------------------------------------
def crc15_by_division(bits) -> int:
    """The same CRC by polynomial division, so it shares no code with `crc15`.

    sec. 3.1.1 divides the destuffed bit stream, taken as a polynomial, by the generator
    x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1 over GF(2) and transmits the remainder.
    """
    generator = CRC_POLY | (1 << 15)            # the leading x^15 term of the generator
    dividend = 0
    for bit in bits:
        dividend = (dividend << 1) | bit
    for shift in range(len(bits) - 1, -1, -1):
        if dividend & (1 << (shift + 15)):
            dividend ^= generator << shift
    return dividend & 0x7FFF


def test_crc_is_the_bosch_polynomial_division():
    """`crc15` (the shift register of sec. 3.1.1) must equal the polynomial division it says
    it is, for every data length, and the generator really is 0x4599."""
    assert CRC_POLY == 0x4599
    for dlc in range(0, MAX_DLC + 1):
        for identifier in (0x001, 0x123, 0x7F8):
            frame = Frame(identifier, bytes(range(dlc)))
            bits = wire_bits(frame, stuffed=False)
            covered = bits[:stuff_region(frame)]
            assert crc15(covered) == crc15_by_division(covered), (identifier, dlc)
    assert crc15([0]) == 0 and crc15([]) == 0


def test_stuffing_never_leaves_six_equal_bits_on_the_wire():
    """sec. 5 and 6.1: after five equal bits the complement goes in, so the longest run of
    equal bits inside SOF..CRC SEQUENCE is five, and six is a stuff error."""
    for data in (b"", b"\x00", b"\xff", b"\xaa\x55", bytes(range(8)), b"\x00" * 8):
        frame = Frame(0x123, data)
        wire, roles = plan(frame)
        head = wire[:roles.index("crc_delim")]
        run = longest = 1
        for i in range(1, len(head)):
            run = run + 1 if head[i] == head[i - 1] else 1
            longest = max(longest, run)
        assert longest <= STUFF_RUN, (data, longest)
        # and the tail is not stuffed: ten recessive bits, CRC delimiter to last EOF bit
        assert wire[-10:] == [RECESSIVE] * 10, data
        assert len(roles) == len(wire) and roles.count("stuff") == len(wire) - len(
            destuff(wire, stuffed=stuff_region(frame))[0])


def test_stuff_and_destuff_are_inverse_over_random_frames():
    """Every frame of every length round-trips through the stuffing rule."""
    import random

    rng = random.Random(20261004)
    for _ in range(400):
        dlc = rng.randrange(MAX_DLC + 1)
        frame = Frame(rng.randrange(1, 0x800), bytes(rng.randrange(256) for _ in range(dlc)))
        wire = frame_bits(frame)
        logical, bad = destuff(wire, stuffed=stuff_region(frame))
        assert not bad, (frame, bad)
        assert logical == wire_bits(frame, stuffed=False), frame
        assert stuff(logical[:stuff_region(frame)]) + logical[stuff_region(frame):] == wire


def test_frame_layout_is_the_seven_fields_of_sec_3_1_1():
    """Field by field, on the logical stream: SOF, ID-10..ID-0, RTR, r1 r0, DLC3..DLC0,
    data MSB first, CRC-14..CRC-0, CRC delimiter, ACK slot, ACK delimiter, EOF."""
    def msb(value, width):
        return [(value >> k) & 1 for k in range(width - 1, -1, -1)]

    frame = Frame(0x2A5, b"\xde\xad")
    bits = wire_bits(frame, stuffed=False)
    assert len(bits) == 1 + 11 + 1 + 2 + 4 + 16 + 15 + 1 + 1 + 1 + 7
    assert bits[0] == DOMINANT                                  # SOF
    assert bits[1:12] == msb(0x2A5, 11)                     # ID-10 .. ID-0
    assert bits[12] == DOMINANT                                 # RTR: a DATA FRAME
    assert bits[13:15] == [DOMINANT, DOMINANT]                  # r1, r0 reserved
    assert bits[15:19] == msb(2, 4)                          # DLC3..DLC0 = 2
    assert bits[19:27] == msb(0xDE, 8)                     # 0xde, MSB first
    assert bits[27:35] == msb(0xAD, 8)                     # 0xad, MSB first
    crc = 0
    for bit in bits[:35]:
        crc = (crc << 1) | bit
    assert bits[35:50] == [(crc15(bits[:35]) >> k) & 1 for k in range(14, -1, -1)]
    assert bits[50:53] == [RECESSIVE, RECESSIVE, RECESSIVE]    # CRC delim, ACK, ACK delim
    assert bits[53:] == [RECESSIVE] * 7                         # END OF FRAME
    # a REMOTE FRAME sends RTR recessive and has no DATA FIELD whatever the DLC says
    remote = Frame(0x123, rtr=True, dlc=2)
    rbits = wire_bits(remote, stuffed=False)
    assert rbits[12] == RECESSIVE and len(rbits) == 1 + 11 + 1 + 2 + 4 + 15 + 3 + 7


# ---- the bit-timing table, against sec. 8 ---------------------------------------------
@pytest.mark.parametrize("speed", SPEEDS)
def test_bit_timing_is_legal_and_on_the_grid(speed):
    """Both modelled configurations must satisfy every sec. 8 limit and land on whole ticks."""
    timing = CAN_BIT_TIMINGS[speed]
    assert check_bit_timing(timing) is timing
    tk = timing_for(NS_PER_TICK, speed)
    assert tk["tq_per_bit"] == timing.tq_per_bit
    assert 8 <= tk["tq_per_bit"] <= 25                 # sec. 8: 8..25 TIME QUANTA per bit
    assert tk["bit_ticks"] == round(1e9 / timing.bitrate / NS_PER_TICK)
    # sec. 8: PHASE_SEG2 is the maximum of PHASE_SEG1 and the 2 TQ INFORMATION PROCESSING TIME
    assert timing.phase2_tq >= max(timing.phase1_tq, 2), timing
    # the SAMPLE POINT sits at the end of PHASE_SEG1, inside the bit, with PHASE_SEG2 after it
    assert 0 < tk["sample_point_ticks"] < tk["bit_ticks"]
    assert tk["sample_point_ticks"] == round(
        timing.sample_point_fraction() * tk["bit_ticks"])
    # 20 ns per tick, so the quantisation can only move the sample point by half a tick
    assert abs(tk["sample_point_ticks"] / tk["bit_ticks"]
               - timing.sample_point_fraction()) <= 0.5 / tk["bit_ticks"]
    # the two supported rates, their bit times, and the sample point each one lands on.
    # `research/ref/protocols/can/README.md` quotes these three lines, so they are asserted
    # here rather than only in prose: 75 % at 125 kbit/s (16 TQ of 500 ns) and 70 % at
    # 500 kbit/s (20 TQ of 100 ns, the most 100 ns quanta can reach with PROP_SEG <= 8 TQ
    # and PHASE_SEG2 >= PHASE_SEG1).
    assert (CAN_BIT_TIMINGS["125k"].bitrate, CAN_BIT_TIMINGS["125k"].bit_ns) == (125e3, 8000.0)
    assert (CAN_BIT_TIMINGS["500k"].bitrate, CAN_BIT_TIMINGS["500k"].bit_ns) == (500e3, 2000.0)
    assert (CAN_BIT_TIMINGS["125k"].tq_per_bit, CAN_BIT_TIMINGS["500k"].tq_per_bit) == (16, 20)
    assert [round(CAN_BIT_TIMINGS[s].sample_point_fraction(), 4) for s in SPEEDS] == [0.75, 0.7]
    assert [tk_of(s)["sample_point_ticks"] for s in SPEEDS] == [300, 70]


def tk_of(speed):
    """The shipped timing of `speed` in whole 20 ns ticks, for the assertions above."""
    return timing_for(NS_PER_TICK, speed)


def test_bit_timing_rejects_what_the_spec_forbids():
    """`check_bit_timing` is the sec. 8 limit list, and it says which limit was broken."""
    from verif.vip.can import CanTiming

    base = CAN_BIT_TIMINGS["125k"]
    for kwargs, needle in [
        ({"sync_tq": 2}, "SYNC_SEG"),
        ({"prop_tq": 9}, "PROP_SEG"),
        ({"phase1_tq": 0}, "PHASE_SEG1"),
        ({"phase2_tq": 1}, "INFORMATION PROCESSING TIME"),
        # sec. 8 and AN1798 Table 1: PHASE_SEG2 is the maximum of PHASE_SEG1 and the 2 TQ IPT,
        # so a PHASE_SEG2 below PHASE_SEG1 is illegal even though it clears the IPT floor
        ({"phase2_tq": 3}, "below PHASE_SEG1"),
        ({"sjw_tq": 5}, "SJW"),
        ({"phase1_tq": 8, "phase2_tq": 13}, "8..25"),   # 28 TIME QUANTA per bit
    ]:
        bad = CanTiming(**{**base.__dict__, **kwargs})
        with pytest.raises(ValueError, match=needle):
            check_bit_timing(bad)
    # and the override plumbing: an unknown speed, an unknown key, a sample point outside the
    # bit, and a time quantum that does not divide the bit time on this tick grid
    with pytest.raises(ValueError, match="unknown CAN speed"):
        timing_for(NS_PER_TICK, "250k-plus")
    with pytest.raises(ValueError, match="unknown timing override"):
        timing_for(NS_PER_TICK, "125k", {"t_nonsense": 1})
    with pytest.raises(ValueError, match="inside the bit"):
        timing_for(NS_PER_TICK, "125k", {"sample_point_tq": 40})
    with pytest.raises(ValueError, match="whole number of"):
        timing_for(NS_PER_TICK, "500k", {"tq_ns": 125.0})       # 2000 ns is not 16 x 125 ns
    # an override that moves the sample point is honoured, and the node reports the new one
    tk = timing_for(NS_PER_TICK, "125k", {"sample_point_tq": 8})
    assert tk["sample_point_ticks"] == 8 * tk["tq_ticks"] and tk["tq_per_bit"] == 16
    assert CanNode(ns_per_tick=NS_PER_TICK, speed="125k",
                   timing={"sample_point_tq": 8}).tk["sample_point_ticks"] == \
        tk["sample_point_ticks"]
    assert ns_to_ticks(NS_PER_TICK, 19.9) == 1 and ns_to_ticks(NS_PER_TICK, 20.0) == 1


# ---- the models talking to each other -------------------------------------------------
@pytest.mark.parametrize("speed", SPEEDS)
def test_one_node_talks_to_another(speed):
    """Every data length, both frame types, in order: sent, acknowledged, received, clean."""
    bus, nodes, chk = rig(speed)
    a, b = nodes
    frames = [Frame(0x001, b""), Frame(0x400, bytes(range(8))), Frame(0x123, b"\xde\xad"),
              Frame(0x2A5, b"\x00"), Frame(0x555, rtr=True, dlc=3), Frame(0x7E8, b"\xff")]
    for frame in frames:
        a.send(frame)
    settled(a, b, bus=bus)

    assert [r.ok for r in a.records] == [True] * len(frames)
    assert all(r.acked and r.won_arb and not r.error for r in a.records), a.records
    assert not a.errors and not b.errors
    assert [r.frame for r in b.rx] == frames, b.rx
    assert all(r.valid and r.acked for r in b.rx)
    assert chk.ids == [f.identifier for f in frames], chk.report()
    assert [f.data for f in chk.frames] == [b"", bytes(range(8)), b"\xde\xad", b"\x00", b"",
                                             b"\xff"]
    assert frames[-1].identifier == 0x7E8, "the highest identifier sec. 3.1.1 allows"
    assert chk.frames[4].rtr and chk.frames[4].dlc == 3 and chk.frames[4].data == b""
    assert all(f.valid for f in chk.frames), chk.frames
    assert chk.clean, chk.report()
    # nothing but the 125 kbit/s and 500 kbit/s rates is on offer
    assert chk.tk["bit_ticks"] == (400 if speed == "125k" else 100)
    assert chk.worst_off_grid == 0, "every edge of a legal frame sits on the bit grid"


@pytest.mark.parametrize("speed", SPEEDS)
def test_the_bit_time_on_the_wire_is_the_nominal_one(speed):
    """The checker measures what the nodes drive, so this is the bit timing end to end: the
    distance between two edges must be a whole number of bit times, and 8000/2000 ns it is."""
    bus, nodes, chk = rig(speed)
    a, b = nodes
    a.send(0x123, b"\xde\xad\xbe\xef")
    settled(a, b, bus=bus)
    bit_ticks = chk.tk["bit_ticks"]
    sp = chk.tk["sample_point_ticks"]
    origin = chk.frames[0].tick - 1                      # the grid the checker locked on
    span = chk.frames[0].bits                           # logical bits in the frame
    # every edge the checker measured was on the grid, and it measured a lot of them
    assert chk.edges > 10 and chk.worst_off_grid == 0
    # the transmitter's last SAMPLE POINT sits exactly on the nominal grid, and the frame
    # spanned a whole number of bit times
    record = a.records[0]
    assert (record.end_tick - origin) % bit_ticks == sp
    assert record.start_tick == origin, "the SOF began on the bit boundary the checker locked"
    assert (record.end_tick - record.start_tick) // bit_ticks >= span
    assert b.tk["bit_ticks"] == bit_ticks == a.tk["bit_ticks"] == chk.tk["bit_ticks"]
    assert span == 1 + 11 + 1 + 2 + 4 + 8 * 4 + 15 + 3 + 7


@pytest.mark.parametrize("speed", SPEEDS)
def test_arbitration_gives_the_bus_to_the_lower_identifier(speed):
    """Both nodes start at the same bit, so the identifier decides bit by bit: the loser
    becomes a receiver, keeps the frame and sends it again once the bus is free."""
    bus, nodes, chk = rig(speed)
    a, b = nodes
    a.send(0x321, b"loser")
    b.send(0x123, b"winner")
    b.send(0x322, b"later")
    settled(a, b, bus=bus)

    assert [r.frame.identifier for r in a.ok_frames] == [0x321]
    assert [r.frame.identifier for r in b.ok_frames] == [0x123, 0x322]
    # a lost arbitration is not an error and is recorded as such: a (0x321) loses to b
    # (0x123) and then wins against b's 0x322, which has the higher identifier
    assert [r.won_arb for r in a.records] == [False, True], a.records
    assert [r.won_arb for r in b.records] == [True, False, True], b.records
    assert not a.errors and not b.errors
    assert [r.frame.data for r in a.rx] == [b"winner", b"later"]
    assert [r.frame.data for r in b.rx] == [b"loser"]
    assert chk.ids == [0x123, 0x321, 0x322] and chk.clean, chk.report()
    assert a.tec == 0 and b.tec == 0, "no error, so no error count"


def test_the_winner_of_an_arbitration_is_the_one_on_the_wire():
    """The checker sees one frame per transmission attempt that completed, so a losing node's
    abandoned attempt must not appear as a frame of its own."""
    bus, nodes, chk = rig("500k")
    a, b = nodes
    for i in range(4):
        a.send(0x400 + i, b"A" * i)
        b.send(0x100 + i, b"B" * i)
    settled(a, b, bus=bus)
    # every completed transmission appears on the wire exactly once, and in the order the
    # nodes started them; an abandoned arbitration attempt is not a frame of its own
    wire_order = [r.frame.identifier
                  for r in sorted((r for n in nodes for r in n.records),
                                  key=lambda r: r.start_tick)
                  if r.end_tick is not None]
    assert len(chk.ids) == 8 == len(wire_order), chk.report()
    assert chk.ids == wire_order, (chk.ids, wire_order)
    # each node still got all four of its own frames out, some of them after losing
    assert [r.frame.identifier for r in nodes[0].ok_frames] == [0x400, 0x401, 0x402, 0x403]
    assert [r.frame.identifier for r in nodes[1].ok_frames] == [0x100, 0x101, 0x102, 0x103]
    assert all(len(n.records) >= 4 for n in nodes)


# ---- the checker, on legal traffic and on broken traffic ------------------------------
def test_the_checker_passes_legal_traffic_and_ignores_an_idle_bus():
    """The other half of "not vacuous": a legal frame and a quiet bus produce nothing at all,
    so every violation below really is the fault that was planted and not background noise."""
    bits = planted()
    _, chk = run_wire(bits)
    assert chk.clean and chk.violations == [], chk.report()
    assert chk.ids == [0x123] and chk.frames[0].data == b"\xde\xad\xbe\xef"
    assert chk.frames[0].valid and chk.frames[0].bits == 1 + 11 + 1 + 2 + 4 + 32 + 15 + 3 + 7
    assert chk.edges > 10, "the checker really did look at the edges of this frame"

    bus = Bus(ns_per_tick=NS_PER_TICK)
    quiet = attach_checker(bus, speed="500k")
    bus.run(50_000)
    assert quiet.clean and quiet.frames == [] and quiet.edges == 0 and quiet.idle_bits > 100


#: Every rule the checker enforces and how to break exactly that one. Each case says what the
#: checker's Violation must say and what the receiving CanNode must log for the same frame,
#: which is what makes this table the test that neither of them is vacuous.
def break_cases():
    """One planted fault per spec rule. `checker_needle` is in Violation.detail,
    `node_needle` in the node's error text, and `node_kind` is None where the fault is not an
    error a receiver can see at all."""
    frame = Frame(0x123, b"\xde\xad\xbe\xef")
    region = stuff_region(frame)
    wire, roles = plan(frame)
    six_in_a_row = list(wire)
    stuffed_bit = roles.index("stuff")
    six_in_a_row[stuffed_bit] = six_in_a_row[stuffed_bit - 1]        # the stuff bit never flips
    dlc_nine = lambda b: b[:15] + [1, 0, 0, 1] + b[19:]             # DLC3..DLC0 = 9
    return [
        dict(id="stuff", rule="stuff", bits=six_in_a_row, node_kind="stuff",
             checker_needle="6 equal bits", node_needle="6 equal bits"),
        dict(id="crc", rule="crc", bits=planted(mutate=flip(region - 3)), node_kind="crc",
             checker_needle="CRC over SOF", node_needle="computed"),
        dict(id="reserved-r1", rule="form",
             bits=planted(mutate=setbit(ID_BITS + 2, RECESSIVE)), node_kind="form",
             checker_needle="reserved bit r1", node_needle="reserved bit r1"),
        dict(id="reserved-r0", rule="form",
             bits=planted(mutate=setbit(ID_BITS + 3, RECESSIVE)), node_kind="form",
             checker_needle="reserved bit r0", node_needle="reserved bit r0"),
        dict(id="dlc-9", rule="form", bits=planted(mutate=dlc_nine), node_kind="form",
             checker_needle="DLC 9", node_needle="DLC 9"),
        dict(id="crc-delim", rule="form", bits=planted(mutate=setbit(region, DOMINANT)),
             node_kind="form", checker_needle="CRC delimiter is dominant",
             node_needle="CRC delimiter is dominant"),
        dict(id="ack-delim", rule="form", bits=planted(mutate=setbit(region + 2, DOMINANT)),
             node_kind="form", checker_needle="ACK delimiter is dominant",
             node_needle="ACK delimiter is dominant"),
        dict(id="eof", rule="form", bits=planted(mutate=setbit(region + 3, DOMINANT)),
             node_kind="form", checker_needle="END OF FRAME", node_needle="END OF FRAME"),
        # sec. 3.1.1 only requires ID-10..ID-4 not all recessive; a real controller accepts the
        # frame and gives it the lowest priority, so this one is the checker's alone
        dict(id="priority", rule="priority", bits=planted(Frame(0x7FF, b"\x00")),
             node_kind=None, checker_needle="ID-10..ID-4", node_needle=None),
    ]


@pytest.mark.parametrize("case", break_cases(), ids=[c["id"] for c in break_cases()])
def test_the_checker_flags_each_broken_rule(case):
    """One spec rule broken per case, and the checker must flag that rule with its tick."""
    _, chk = run_wire(case["bits"])
    assert chk.violations, f"{case['id']}: nothing was flagged"
    assert case["rule"] in {v.rule for v in chk.violations}, f"{case['id']}: " + chk.report()
    first = chk.violations_by_rule(case["rule"])[0]
    assert first.tick > 0, f"{case['id']}: {first}"
    assert case["checker_needle"] in first.detail, f"{case['id']}: {first}"
    assert str(first).startswith(f"t={first.tick} {case['rule']} "), str(first)


def test_the_checker_flags_a_missing_acknowledgement():
    """sec. 6.1: an ACKNOWLEDGMENT ERROR is a transmitter seeing no dominant bit in the ACK
    SLOT, which on the wire is a recessive ACK SLOT -- so that is what the checker looks for."""
    bits = planted()
    _, quiet = run_wire(bits, check_ack=False)
    assert "ack" not in {v.rule for v in quiet.violations}, quiet.report()
    _, checked = run_wire(bits, check_ack=True)
    assert [v.rule for v in checked.violations] == ["ack"], checked.report()
    assert checked.violations[0].measured == RECESSIVE
    assert checked.violations[0].required == DOMINANT
    assert checked.frames[0].acked is False and not checked.frames[0].valid


def test_the_checker_flags_a_dominant_bit_inside_the_error_delimiter():
    """sec. 3.1.3 and 3.1.4: an ERROR or OVERLOAD FRAME is six dominant bits (the FLAG) and
    eight recessive ones (the DELIMITER), so a dominant bit anywhere in those eight is a
    violation of the `delimiter` rule -- and up to twelve dominant flag bits are legal, because
    six superposed ERROR FLAGs may add up (sec. 3.1.3)."""
    legal = planted()
    # the seven END OF FRAME bits are the tail, so this replaces them
    eof = legal[:-7]
    bit_ticks = timing_for(NS_PER_TICK, "500k")["bit_ticks"]

    # a legal six-bit flag with a clean eight-bit delimiter: the delimiter rule stays silent,
    # which is the control that says the next case is the delimiter and not the flag
    _, clean = run_wire(eof + [DOMINANT] * 6 + [RECESSIVE] * 8 + [RECESSIVE] * 40)
    assert not clean.violations_by_rule("delimiter"), clean.report()
    assert clean.error_flags and clean.error_flags[0][1:] == (6, 8), clean.error_flags

    # one dominant bit inside the eight: the delimiter is broken, not the flag
    _, broken = run_wire(eof + [DOMINANT] * 6 + [RECESSIVE, DOMINANT] + [RECESSIVE] * 40)
    found = broken.violations_by_rule("delimiter")
    assert found, broken.report()
    assert (found[0].measured, found[0].required) == (0, 8), str(found[0])
    assert "dominant bit inside the 8-bit delimiter" in found[0].detail

    # thirteen dominant bits is more than the twelve sec. 3.1.3 allows for superposed flags
    _, long_flag = run_wire(eof + [DOMINANT] * 13 + [RECESSIVE] * 40)
    long_form = [v for v in long_flag.violations_by_rule("form")
                 if "twelve" in v.detail]
    assert long_form, long_flag.report()
    assert (long_form[0].measured, long_form[0].required) == (13, 12), str(long_form[0])

    # and a flag that is both too long and followed by a broken delimiter is flagged twice, once
    # per rule: the flag is thirteen dominant bits, then a dominant bit lands in the delimiter
    _, both = run_wire(eof + [DOMINANT] * 13 + [RECESSIVE, DOMINANT] + [RECESSIVE] * 40)
    assert both.violations_by_rule("delimiter"), both.report()
    assert [v for v in both.violations_by_rule("form") if "twelve" in v.detail], both.report()
    assert both.violations[0].tick > 0 and bit_ticks == 100


def test_the_checker_flags_a_start_of_frame_that_is_not_dominant():
    """sec. 3.1.1 and 6.1: START OF FRAME is one dominant bit, so a node that releases the net
    before the SAMPLE POINT has not sent a SOF and the frame must be flagged -- while a SOF held
    for a whole bit time, which is what every legal frame sends, must not be. This is the one
    rule `WirePlayer` cannot plant, because a bit-level fault cannot make the SOF recessive."""
    tk = timing_for(NS_PER_TICK, "500k")
    bit_ticks, sample = tk["bit_ticks"], tk["sample_point_ticks"]
    # ID-10 of 0x7E8 is recessive, so the first bit after the SOF releases the net and the SOF is
    # the only thing holding it dominant
    rest = planted(Frame(0x7E8, b"\xde\xad"))[1:]

    def run(sof_ticks):
        bus = Bus(ns_per_tick=NS_PER_TICK)
        chk = attach_checker(bus, speed="500k", check_ack=False)
        bus.attach(ShortSofPlayer(rest, bit_ticks, sof_ticks))
        bus.run(bit_ticks * (len(rest) + 60))
        return chk

    legal = run(bit_ticks)                          # one whole bit time: the legal case
    assert not [v for v in legal.violations if "SOF" in v.detail], legal.report()
    assert legal.ids == [0x7E8], legal.report()

    for sof_ticks in (1, sample // 2, sample - 1):   # released before the sample point: illegal
        chk = run(sof_ticks)
        found = [v for v in chk.violations_by_rule("form") if "SOF is not dominant" in v.detail]
        assert found, (sof_ticks, chk.report())
        assert (found[0].measured, found[0].required) == (RECESSIVE, DOMINANT), str(found[0])
        # the violation lands on the SAMPLE POINT of the SOF bit, the grid locked on its edge
        assert found[0].tick == sample, (sof_ticks, str(found[0]), sample)


def test_the_checker_flags_a_sof_inside_the_intermission():
    """sec. 3.1.5: INTERMISSION is three recessive bits and nothing may start inside it."""
    bits = planted()
    for gap, legal in [(0, False), (1, False), (2, False), (3, True), (4, True)]:
        _, chk = run_wire(bits + [RECESSIVE] * gap + bits)
        flagged = chk.violations_by_rule("intermission")
        assert bool(flagged) is not legal, (gap, chk.report())
        if not legal:
            assert flagged[0].required == 3 and flagged[0].measured == gap


def test_the_checker_flags_a_wrong_bit_time():
    """sec. 8: NOMINAL BIT TIME = 1 / bit rate, so every edge must land on the bit grid. Ten
    ticks fast or slow per bit is a tenth of a bit out of place, which is 10x `edge_tol`."""
    bits = planted()
    nominal = timing_for(NS_PER_TICK, "500k")["bit_ticks"]
    _, chk = run_wire(bits)
    assert chk.clean and chk.worst_off_grid == 0, chk.report()
    for bit_ticks in (nominal - 10, nominal + 10):
        _, slow = run_wire(bits, bit_ticks=bit_ticks)
        found = slow.violations_by_rule("bit time")
        assert found, f"{bit_ticks} ticks per bit must not look like {nominal}"
        assert found[0].measured > slow.edge_tol, slow.report()
    # and the tolerance is a knob, not a rule change: at 10 ticks of slack it passes
    _, loose = run_wire(bits, bit_ticks=nominal + 10,
                        check_kwargs={"edge_tol": nominal})
    assert not loose.violations_by_rule("bit time"), loose.report()


def test_the_checker_is_not_vacuous():
    """The same planted faults a CanNode must notice.

    Both devices watch the same pins, so a rule either of them had quietly stopped checking
    would show up here: the node has to log the error of sec. 6.1 and the checker has to flag
    it, from the wire, with nothing else on the bus but a listener.
    """
    bit_ticks = timing_for(NS_PER_TICK, "500k")["bit_ticks"]
    for case in break_cases():
        bus = Bus(ns_per_tick=NS_PER_TICK)
        node = attach_node(bus, name="listener", speed="500k")
        chk = attach_checker(bus, speed="500k", check_ack=False)
        bus.attach(WirePlayer(case["bits"], bit_ticks))
        bus.run(bit_ticks * (len(case["bits"]) + 60))

        # The checker sees the same thing from the wire. Which rule it names is checked
        # above, on a bus with no node on it, because a node that sees the same fault
        # signals it itself (sec. 6.2) and the six dominant bits of its ERROR FLAG then
        # stand where the checker was about to complain -- so what the checker reports there
        # is the flag, not the field, which is also what a real analyser shows.
        assert chk.violations or chk.error_flags, f"{case['id']}: " + chk.report()
        if case["node_kind"] is None:
            assert not node.errors, f"{case['id']}: " + str(node.errors[:2])
            continue
        assert node.errors, f"{case['id']}: the node saw nothing"
        assert any(case["node_kind"] in text for text in node.errors), \
            f"{case['id']}: " + str(node.errors[:2])
        assert any(case["node_needle"] in text for text in node.errors), \
            f"{case['id']}: " + str(node.errors[:2])
        # sec. 6.1 and 6.2: the error is counted, and an ERROR FLAG goes out at the next bit
        assert node.error_log and node.rec >= 1, (case["id"], node.rec, node.tec)


# ---- error frames, retransmission and fault confinement (sec. 3.1.3, 6.2 and 7) ---------
def level_runs(bus):
    """The bus trace as a list of (level, first tick, ticks), so a test can measure a flag."""
    runs = []
    for tick, values in bus.trace:
        level = values["can"]
        if runs and runs[-1][0] == level and runs[-1][1] + runs[-1][2] == tick:
            runs[-1][2] += 1
        else:
            runs.append([level, tick, 1])
    return [(level, start, length) for level, start, length in runs]


def test_a_receiver_signals_an_error_flag_and_its_delimiter():
    """sec. 6.2: a CRC ERROR is signalled at the bit after the ACK DELIMITER, as six dominant
    bits (an ACTIVE ERROR FLAG) followed by eight recessive ones (the ERROR DELIMITER)."""
    bus = Bus(ns_per_tick=NS_PER_TICK, trace=True)
    node = attach_node(bus, name="listener", speed="500k")
    chk = attach_checker(bus, speed="500k")
    bit_ticks = node.tk["bit_ticks"]
    bad = planted(mutate=flip(stuff_region(Frame(0x123, b"\xde\xad\xbe\xef")) - 3))
    bus.attach(WirePlayer(bad, bit_ticks))
    bus.run(bit_ticks * (len(bad) + 60))

    # the node saw a bad CRC, did not acknowledge the frame, and counted the error
    assert any("crc error" in text for text in node.errors), node.errors[:2]
    assert node.rx == [], "a node that signals an error abandons the frame (sec. 4)"
    assert node.rec >= 1
    # the wire carries a six-bit dominant flag and an eight-bit recessive delimiter
    flags = [run for run in level_runs(bus)
             if run[0] == DOMINANT and run[2] == 6 * bit_ticks]
    assert flags, level_runs(bus)[-8:]
    after = [run for run in level_runs(bus)
             if run[1] == flags[0][1] + 6 * bit_ticks and run[0] == RECESSIVE]
    assert after and after[0][2] >= 8 * bit_ticks, after
    # the node did not acknowledge a frame whose CRC did not match (sec. 3.1.1), and the
    # checker decoded the flag and its eight-bit delimiter without complaining about them
    assert chk.violations_by_rule("ack"), chk.report()
    assert not chk.violations_by_rule("delimiter"), chk.report()
    assert chk.error_flags and chk.error_flags[0][1:] == (6, 8), chk.error_flags


def test_a_passive_error_flag_is_not_visible_on_the_wire():
    """sec. 3.1.3: an error-passive node sends six *recessive* bits as its PASSIVE ERROR FLAG,
    so a broken bus it is sitting on does not get six more dominant bits from it."""
    bus = Bus(ns_per_tick=NS_PER_TICK, trace=True)
    solo = attach_node(bus, name="solo", speed="500k")
    bit_ticks = solo.tk["bit_ticks"]
    solo.send(0x123, b"\xde\xad")
    # enough attempts to reach the error-passive threshold of 128 (sec. 7 rule 9) at +8 each
    for _ in range(400):
        bus.run(bit_ticks * 90)
        if solo.error_passive:
            break
    assert solo.attempts >= 16 and solo.tec >= 128, (solo.attempts, solo.tec)
    assert solo.error_passive and solo.state_name == "error-passive", solo.state_name
    assert any(kind == "error-passive" for _t, kind, _d in solo.error_log)
    dominant_flags = [run for run in level_runs(bus)
                      if run[0] == DOMINANT and run[2] == 6 * bit_ticks]
    assert dominant_flags, "an error-active node must send active error flags"
    # sec. 3.1.3: from the tick it goes error-passive its PASSIVE ERROR FLAG is six *recessive*
    # bits, so no run of exactly six dominant bits may start at or after that tick. Keep the
    # bus running for a full flag plus its 8-bit delimiter first, so this is measured over
    # retransmissions rather than over the single tick the state changed on. (A frame's own
    # bits never run six dominant in a row: the stuffing rule of sec. 5 keeps them at five, so
    # a run of exactly six is an ERROR FLAG and nothing else.)
    passive_tick = next(t for t, kind, _d in solo.error_log if kind == "error-passive")
    bus.run(bit_ticks * 30)
    assert not [r for r in level_runs(bus) if r[0] == DOMINANT and r[2] == 6 * bit_ticks
                and r[1] >= passive_tick], level_runs(bus)[-6:]
    # once error-passive it also waits before its next frame: SUSPEND TRANSMISSION owes 8
    # recessive bits (sec. 3.1.5, and `test_suspend_transmission_holds_an_error_passive_node_back`)
    assert solo.tec >= 128


def two_frame_gap(error_passive: bool):
    """The inter-frame space on the wire between two frames a node sends one after the other.

    Both frames are queued before the first one goes out and a partner is on the bus to
    acknowledge them, so nothing the test does decides when the second SOF happens: the only
    thing that can move it is the node's own state machine. Returns the gap in ticks -- the real
    inter-frame space, from the end of the first frame's END OF FRAME to the second frame's
    SOF -- and the node, with `error_passive` putting it into error-passive first through the
    sec. 7 counter it exposes (rule 9: TEC >= 128). The trace stays on so the number can be
    re-derived from the wire by hand, with `level_runs`, if a reader does not believe it.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK, trace=True)
    solo = attach_node(bus, name="solo", speed="500k")
    partner = attach_node(bus, name="partner", speed="500k")
    if error_passive:
        # 130 and not 128: sec. 7 rule 7 takes one off after each successful transmission, so
        # 128 would leave the node at 126 after the pair -- both counts <= 127, which is rule
        # 11's return to error-active, and the node would owe no SUSPEND TRANSMISSION at all
        # on the second frame. 130 leaves it at 128, still error-passive across both frames.
        solo._bump_tec(130)
    solo.send(0x123, b"\xde\xad")
    solo.send(0x456, b"\x11\x22")
    assert bus.run_until(lambda _b: solo.idle and partner.idle, 2_000_000), solo.state_name
    assert len(solo.records) == 2 and all(r.ok for r in solo.records), solo.records
    first_end, second_start = solo.records[0].end_tick, solo.records[1].start_tick
    gap = second_start - first_end
    return gap, solo


def test_suspend_transmission_holds_an_error_passive_node_back():
    """sec. 3.1.5: "After an 'error passive' station has transmitted a message, it sends eight
    'recessive' bits following INTERMISSION", so the inter-frame space between two frames of an
    error-passive transmitter is *five* bit times longer than the same gap on an error-active
    one -- which is what the model measures on the bus, in bit times.

    Not eight, because sec. 3.1.5's 8 recessive bits are counted after INTERMISSION's 3 while
    `CanNode` starts draining `suspend` at the first bit boundary after the frame, so the two
    overlap: 3 + 8 - 3 = 8 bit times of space where an error-active node has 3."""
    bit_ticks = timing_for(NS_PER_TICK, "500k")["bit_ticks"]
    active_gap, active = two_frame_gap(error_passive=False)
    passive_gap, passive = two_frame_gap(error_passive=True)

    assert active.error_active and passive.error_passive, (active.state_name,
                                                           passive.state_name)
    # sec. 7 rule 7 takes one off the count after each successful transmission: 130 -> 128, so
    # the node is still error-passive across the pair (rule 11 needs both counts <= 127) and owes
    # its 8 recessive bits both times
    assert (active.tec, passive.tec) == (0, 128), (active.tec, passive.tec)
    assert passive.suspend == 0, "the 8 recessive bits have drained by the time both are out"
    assert passive_gap >= active_gap + 5 * bit_ticks, (
        f"error-passive gap {passive_gap / bit_ticks:.1f} bit times, "
        f"error-active gap {active_gap / bit_ticks:.1f}: SUSPEND TRANSMISSION is missing")
    # ... and the space is at least 8 bit times in absolute terms, so the 8 recessive bits have to
    # be there at all: without them the passive node's gap is the error-active one, 3 bit times.
    # Deliberately "at least", not "exactly": the one-line change that makes the model literal
    # (`if self.suspend > 0 and self.idle_bits >= 3` -- 11 bit times of space) is still possible,
    # and both readings hold here.
    assert passive_gap >= 8 * bit_ticks, passive_gap / bit_ticks


@pytest.mark.parametrize("counter", ["tec", "rec"])
def test_sec7_rule_11_an_error_passive_node_becomes_error_active_again(counter):
    """sec. 7 rule 11: "An 'error passive' node becomes 'error active' again when both the
    TRANSMIT ERROR COUNT and the RECEIVE ERROR COUNT are less than or equal to 127."

    One acknowledged frame is enough, whichever counter made the node error-passive: rule 7
    takes one off the TRANSMIT ERROR COUNT after a successful transmission and rule 8 one off
    the RECEIVE ERROR COUNT after a successful reception, so both land on 127 -- and from that
    frame on the node owes no SUSPEND TRANSMISSION, because `suspend` is only armed while it is
    error-passive."""
    bus, (passive, partner), chk = rig("500k")
    getattr(passive, f"_bump_{counter}")(128)            # error-passive by rule 9
    assert passive.error_passive and passive.state_name == "error-passive", passive.state_name
    (passive if counter == "tec" else partner).send(0x123, b"\xde\xad")
    settled(passive, partner, bus=bus)

    assert getattr(passive, counter) == 127, getattr(passive, counter)   # rule 7 / rule 8
    assert passive.error_active and not passive.error_passive, passive.state_name
    assert passive.state_name == "wait", passive.state_name
    assert any(kind == "error-active" for _t, kind, _d in passive.error_log), \
        passive.error_log[-3:]
    assert passive.suspend == 0 and chk.clean, chk.report()


def test_a_lone_node_goes_bus_off_and_comes_back():
    """sec. 7: +8 per ERROR FLAG sent as transmitter, bus-off at 256, and back to
    error-active after 128 occurrences of 11 consecutive recessive bits."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    solo = attach_node(bus, name="solo", speed="500k")
    bit_ticks = solo.tk["bit_ticks"]
    solo.send(0x123, b"\xde\xad")
    for _ in range(400):
        bus.run(bit_ticks * 90)
        if solo.bus_off:
            break
    assert solo.bus_off and solo.state_name == "bus-off", (solo.state_name, solo.tec)
    assert solo.tec >= 256, solo.tec
    assert any("ack error" in text for text in solo.errors[:40])
    # a bus-off node has its drivers off and its frame back on the queue
    assert solo.drive is None and solo.pending == 1
    # nothing but recessive bits for 128 x 11 bit times brings it back, error active, zeroed
    bus.run(bit_ticks * 128 * 11 + 10 * bit_ticks)
    assert not solo.bus_off and solo.error_active and solo.tec == 0 and solo.rec == 0
    assert solo.state_name in ("wait", "tx"), solo.state_name
    assert any(kind == "recovered" for _t, kind, _d in solo.error_log)


def test_a_lone_node_retransmits_and_stops_when_told_to(monkeypatch):
    """sec. 4: a corrupted frame is retransmitted automatically as soon as the bus is idle;
    `max_attempts` is the test hook that stops the spec's endless retrying."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    solo = attach_node(bus, name="solo", speed="500k", max_attempts=3)
    solo.send(0x123, b"\xde\xad")
    assert solo.run(bus, limit=2_000_000), solo.state_name
    assert solo.attempts == 3 and len(solo.records) == 3
    assert all(r.error == "ack" and not r.acked for r in solo.records), solo.records
    assert solo.tec == 24, solo.tec                     # 3 error flags, +8 each (sec. 7 rule 3)
    assert any("gave up" in text for text in solo.errors), solo.errors[-1]
    assert solo.idle and solo.pending == 0
    # `CanNode.run`'s own guard: ACK_TIMEOUT_TICKS bit times at the node's own bit timing, which
    # is 2,000,000 ticks at 500 kbit/s, and False rather than a claim of idle when the limit runs
    # out. A lone node with one frame queued retransmits for ever (sec. 4), so it never gets there
    assert ACK_TIMEOUT_TICKS * solo.tk["bit_ticks"] == 2_000_000
    alone = Bus(ns_per_tick=NS_PER_TICK)
    stuck = attach_node(alone, name="stuck", speed="500k")
    stuck.send(0x124, b"\x01")
    assert not stuck.run(alone, limit=stuck.tk["bit_ticks"]), stuck.state_name
    assert stuck.busy and not stuck.idle
    # The default limit itself, not the arithmetic identity above: `run` reads the module global
    # at call time, so shrinking it to 3 bit times and calling `run` with no `limit=` at all is
    # what proves the guard is the constant and not a hard-coded number (monkeypatch restores it).
    monkeypatch.setattr("verif.vip.can.ACK_TIMEOUT_TICKS", 3)
    before = alone.t
    assert not stuck.run(alone), stuck.state_name
    # ... and it stopped after exactly that many ticks: a hard-coded 2,000,000 would return False
    # just as quietly, so the tick count is what says which limit was used
    assert alone.t - before == 3 * stuck.tk["bit_ticks"], alone.t - before
    assert stuck.busy and not stuck.idle


def test_a_frame_with_a_dominant_last_eof_bit_is_invalid_but_not_an_error():
    """sec. 4: "Receivers: The message is valid for the receivers, if there is no error until
    the last but one bit of END OF FRAME." A dominant bit in the very last one therefore
    invalidates the frame without being signalled -- while the checker still flags the form."""
    frame = Frame(0x123, b"\xde\xad\xbe\xef")
    last_eof = len(wire_bits(frame, stuffed=False)) - 1
    bus = Bus(ns_per_tick=NS_PER_TICK)
    node = attach_node(bus, name="listener", speed="500k")
    chk = attach_checker(bus, speed="500k", check_ack=False)
    bus.attach(WirePlayer(planted(mutate=setbit(last_eof, DOMINANT)), node.tk["bit_ticks"]))
    bus.run(node.tk["bit_ticks"] * 200)
    assert node.errors == [], node.errors
    assert node.rx and not node.rx[0].valid and "last EOF bit" in node.rx[0].error
    assert chk.violations_by_rule("form"), chk.report()


# ---- guard rails ----------------------------------------------------------------------
def test_frames_and_timings_that_cannot_exist_are_refused():
    """A standard identifier is 11 bits, a classic frame 0..8 data bytes, and a REMOTE FRAME
    has no DATA FIELD (sec. 3.1.2), whatever its DLC says."""
    for kwargs in ({"identifier": 0x800}, {"identifier": -1}):
        with pytest.raises(ValueError, match="11 bits"):
            Frame(**kwargs)
    with pytest.raises(ValueError, match="0..8 data bytes"):
        Frame(0x123, b"x" * 9)
    with pytest.raises(ValueError, match="DLC must be"):
        Frame(0x123, b"", dlc=9)
    with pytest.raises(ValueError, match="no DATA FIELD"):
        Frame(0x123, b"x", rtr=True)
    assert Frame(0x7FF).identifier == 0x7FF and Frame(0x123, b"x" * 8).dlc == 8
    assert Frame(0x123, b"", dlc=3).dlc == 3, "a DLC with no data is legal: it is a REMOTE FRAME"
    with pytest.raises(ValueError, match="unknown CAN speed"):
        CanNode(speed="250k")
    with pytest.raises(ValueError, match="unknown CAN speed"):
        CanChecker(speed="2M")


def test_parse_frame_refuses_a_stream_that_is_not_a_frame():
    """`parse_frame` is the after-the-fact decoder: it says what is wrong instead of raising,
    because a receiver has to cope with whatever arrives."""
    frame, problems = parse_frame([RECESSIVE] * 20)
    assert frame is None and "too short" in problems[0]
    stream = wire_bits(Frame(0x123, b"\xde\xad"), stuffed=False)
    frame, problems = parse_frame([RECESSIVE] + stream[1:])
    assert frame is not None and "SOF is not dominant" in problems
    bits = wire_bits(Frame(0x123, b"\xde\xad"), stuffed=False)
    frame, problems = parse_frame(bits[:len(bits) // 2])
    assert frame is None and problems


def test_the_bench_helpers_choose_the_net_and_the_tick_rate():
    """`attach_node`/`attach_checker` add the open-drain net if it is not there yet, take the
    tick rate from the bus, and work on a net of any name."""
    bus = Bus(ns_per_tick=10.0)                      # 100 MHz this time
    assert "can" not in bus.nets
    node = attach_node(bus, name="n0", speed="500k")
    chk = attach_checker(bus, speed="500k")
    assert "can" in bus.nets and bus.nets["can"].pullup == 1
    assert node.tk["bit_ticks"] == 200 and chk.tk["bit_ticks"] == 200
    assert chk.tk["sample_point_ticks"] == 140, "the sample point stays at 70% of the bit"
    assert bus.nets["can"].push_pull is False, "CAN is wired-AND with a pull-up (sec. 2)"
    # bind() takes the rate from the bus afterwards, which is how a node outlives its Bus
    other = Bus(ns_per_tick=20.0)
    late = CanNode(speed="125k").bind(other)
    assert late.ns_per_tick == 20.0 and late.tk["bit_ticks"] == 400
    assert other.nets == {} and late.bus is other, "bind() does not attach the node"

"""End-to-end check of fw/can on the ISS: program, driver, chip and bus.

The judge here is not the driver: it is `verif/vip/can.py`. `CanNode` is the other
node on the one open-drain net -- the *receiver*, which acknowledges the frames this
firmware transmits -- and `CanChecker` is the passive monitor, which locks its bit
grid on every SOF edge and flags every timing, stuffing, CRC, form, ACK and
intermission violation with the tick it happened on. Neither is edited and neither is
softened: a violation fails the test.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`; the CAN line is one of those nets, so there is no
coupling device in between and no extra tick of latency anywhere. Every device on the
net drives 0 or releases it, which is what CAN's wired-AND bus is
(`01-bosch-frame-fields-coding.txt` sec. 2).

What is tested and what is not is written down in `fw/can/CANNOT-EXPRESS.md`: the PIO
carries the bit layer (open-drain drive at the nominal bit time, read back in the same
bit time), and the host carries the frame layer (CRC-15, stuffing, field order). The
receive side, the hardware CRC and the error handling of sec. 7 are not expressible in
this ISA; the tests that would need them skip with that reason.

`FW_TARGET=iss` runs it against `iss.pio_iss.Chip`, which is what `make fw PROTO=can
TARGET=iss` does.
"""
from __future__ import annotations

import csv
from pathlib import Path

import pytest

from fw.can import CanDriver, CanError, assemble_program
from fw.can import driver as drv
from iss import pio_iss
from verif.vip.can import (  # the model: never edited, always the judge
    DOMINANT as VIP_DOMINANT,
    RECESSIVE as VIP_RECESSIVE,
    Frame,
    attach_checker,
    attach_node,
    timing_for,
    wire_bits as vip_wire_bits,
)
from verif.vip.core import Bus

TICK_NS = 20.0                    # 50 MHz, the clock every CAN timing is quoted at
DEFAULT_PIN = drv.DEFAULT_CAN_PIN

#: The two speeds the model knows, as core cycles per bit time on this tick grid
#: (`docs/mvp-isa.md` `## Timing` and `timing_for`): 8 us and 2 us.
SPEED_125K = "125k"
SPEED_500K = "500k"

#: The frames the headline tests put on the wire. Every identifier here has at least one
#: *dominant* bit in ID-10..ID-4, because a frame whose seven most significant
#: identifier bits are all recessive is the lowest-priority frame on the bus and the
#: checker has a rule for exactly that (sec. 3.1.1, reported as "priority"), which is a
#: property of the identifier and not of this firmware.
FRAME = (0x123, b"\xde\xad")


class Bench:
    """A bus with the chip, one receiving CAN node and the checker on it."""

    def __init__(self, speed: str = SPEED_125K, can_pin: int = DEFAULT_PIN,
                 engine: int = 0, peer: bool = True, timing: dict = None,
                 bit_ticks: int = None):
        """`timing` overrides the model's bit timing, which is how `fw/can/measure.py`
        asks about a speed `CAN_BIT_TIMINGS` does not name."""
        self.speed = speed
        self.timing = dict(timing or {})
        self.can_pin, self.engine = can_pin, engine
        self.tk = timing_for(TICK_NS, speed, self.timing)
        self.bit_ticks = bit_ticks if bit_ticks is not None else self.tk["bit_ticks"]
        self.bus = Bus(ns_per_tick=TICK_NS)
        for k in range(pio_iss.NUM_PINS):                 # p0..p17, all pulled up
            self.bus.add_net(f"p{k}")
        self.chip = pio_iss.Chip()
        self.device = pio_iss.ChipDevice(self.chip)
        self.bus.attach(self.device)
        net = f"p{can_pin}"
        self.node = (attach_node(self.bus, name="peer", speed=speed, net=net,
                                 timing=self.timing) if peer else None)
        self.checker = attach_checker(self.bus, speed=speed, net=net, timing=self.timing)
        self.program = assemble_program()
        self.driver = CanDriver(self.bus, self.device, self.program,
                                bit_ticks=self.bit_ticks, engine=engine, can_pin=can_pin)

    def start(self) -> "Bench":
        self.driver.load()
        self.driver.clear_fdebug()
        return self

    # ---- what the checker saw ------------------------------------------------------
    def report(self) -> str:
        return self.checker.report()

    def decoded(self) -> list:
        return list(self.checker.frames)

    def node_frames(self) -> list:
        return [r for r in self.node.rx if r.valid]


# ---- the headline: a frame on the wire, judged by the model ------------------------
def test_a_data_frame_goes_out_and_is_acknowledged_at_125k():
    """125 kbit/s: one DATA FRAME, zero violations, and the model node ACKs it.

    125 kbit/s is 8 us per bit, i.e. 400 core cycles on this clock, and
    `program.pio`'s loop is 10 SM cycles per bit, so CLKDIV is 40.
    """
    bench = Bench(SPEED_125K).start()
    assert bench.driver.clkdiv == 40, bench.driver.clkdiv

    result = bench.driver.send(*FRAME)

    # The judge. Zero violations, one frame, and the frame it decoded is ours.
    assert bench.checker.violations == [], bench.report()
    assert len(bench.checker.frames) == 1, bench.report()
    frame = bench.checker.frames[0]
    assert (frame.identifier, frame.data, frame.rtr, frame.dlc) == (0x123, b"\xde\xad", False, 2)
    assert frame.acked is True, "the ACK SLOT read dominant"
    assert frame.problems == [], frame
    # The checker counts the *destuffed* bits of the frame it decoded; the host pushed
    # the stuffed stream plus INTERMISSION, so the two differ by the stuffed bits.
    assert result.wire_bits - drv.TAIL_BITS - frame.bits == result.stuff_bits, (
        f"{frame.bits} frame bits decoded, {result.wire_bits} pushed, "
        f"{result.stuff_bits} of them stuffed"
    )

    # The other node agrees, and its error counters never moved (sec. 7 rules 3, 7, 8):
    # a clean frame is the only thing that can happen here.
    assert [(r.frame.identifier, r.frame.data) for r in bench.node_frames()] == [(0x123, b"\xde\xad")]
    assert all(r.acked for r in bench.node.rx), bench.node.rx
    assert (bench.node.tec, bench.node.rec) == (0, 0), bench.node.error_log
    assert bench.node.errors == []

    # And the chip itself saw the bus: the readback is the complement of what was
    # pushed, with the ACK SLOT the one bit a receiver drove.
    assert result.acked, "the PIO's readback shows the ACK SLOT as dominant"
    assert result.bit_errors == [], f"bit errors in {result.bit_errors}"
    assert bench.driver.fdebug() == 0, f"FDEBUG {bench.driver.fdebug():#010x} after the frame"


def test_a_data_frame_goes_out_and_is_acknowledged_at_500k():
    """500 kbit/s: the same words, the same program, CLKDIV 5 instead of 20.

    500 kbit/s is 2 us per bit, i.e. 100 core cycles, which is exactly one quarter of
    the 125 kbit/s bit time, so this configuration is a quarter of the divider.
    """
    bench = Bench(SPEED_500K).start()
    assert bench.driver.clkdiv == 10, bench.driver.clkdiv

    result = bench.driver.send(*FRAME)

    assert bench.checker.violations == [], bench.report()
    assert len(bench.checker.frames) == 1, bench.report()
    assert bench.checker.frames[0].acked is True
    assert [(r.frame.identifier, r.frame.data) for r in bench.node_frames()] == [(0x123, b"\xde\xad")]
    assert (bench.node.tec, bench.node.rec) == (0, 0), bench.node.error_log
    assert result.acked and result.bit_errors == [], result
    assert bench.driver.fdebug() == 0


# ---- configurations: pin, engine, frame shapes --------------------------------------
def test_another_pin_and_the_other_engine():
    """The windows are PINCTRL fields, so the same five words run on pin 4 / engine 1.

    Engine 1 has the higher priority in the pin resolution (docs/mvp-isa.md
    `### Pin writes`), which is why nothing else may be writing this pin.
    """
    bench = Bench(SPEED_500K, can_pin=4, engine=1).start()
    result = bench.driver.send(0x7A1, b"\x01\x02\x03\x04\x05\x06\x07\x08")

    assert bench.checker.violations == [], bench.report()
    assert bench.checker.ids == [0x7A1], bench.report()
    assert bench.checker.frames[0].data == bytes(range(1, 9))
    assert result.acked
    # A full-length frame: 8 data bytes is the longest a classic CAN frame gets, and
    # the TX FIFO is four words, so the host has to keep feeding during the frame.
    assert bench.driver.peak_fifo == drv.FIFO_DEPTH, "the driver filled the TX FIFO"
    assert bench.driver.fdebug() == 0


def test_every_data_length_code_from_zero_to_eight():
    """DLC 0..8, which is every admissible length of sec. 3.1.1, one after another.

    DLC matters twice over: it sets the length of the DATA FIELD and it moves the ACK
    SLOT along the wire, so a frame of each length is a different bit pattern for the
    checker's CRC, form, stuff and ACK rules.
    """
    bench = Bench(SPEED_500K).start()
    for dlc in range(9):
        payload = bytes(range(0x80, 0x80 + dlc))
        result = bench.driver.send(0x321, payload)
        assert result.acked, f"DLC {dlc} was not acknowledged"
        assert bench.checker.violations == [], f"DLC {dlc}: " + bench.report()

    assert bench.checker.ids == [0x321] * 9, bench.report()
    assert [f.dlc for f in bench.checker.frames] == list(range(9)), bench.report()
    assert [f.data for f in bench.checker.frames] == [
        bytes(range(0x80, 0x80 + n)) for n in range(9)]
    assert all(f.acked for f in bench.checker.frames)
    assert [(r.frame.dlc, r.frame.data) for r in bench.node_frames()] == [
        (n, bytes(range(0x80, 0x80 + n))) for n in range(9)]
    assert (bench.node.tec, bench.node.rec) == (0, 0), bench.node.error_log


def test_a_remote_frame_carries_no_data_field():
    """REMOTE FRAME (sec. 3.1.2): RTR recessive, no DATA FIELD whatever the DLC says."""
    bench = Bench(SPEED_500K).start()
    result = bench.driver.send(0x5A5, rtr=True, dlc=6)

    assert bench.checker.violations == [], bench.report()
    frame = bench.checker.frames[0]
    assert frame.rtr is True and frame.dlc == 6 and frame.data == b"", frame
    assert frame.acked is True
    assert result.acked
    assert bench.node_frames()[0].frame.rtr is True
    assert (bench.node.tec, bench.node.rec) == (0, 0)


def test_back_to_back_frames_respect_intermission():
    """Four frames on one bus, and sec. 3.1.5 INTERMISSION between them.

    The model checker has a rule for exactly this: "a SOF after fewer than three
    recessive bits" is an intermission violation. The driver leaves the engine parked
    with the line released and waits four bit times before the next SOF, so the three
    recessive bits are there with room to spare.
    """
    bench = Bench(SPEED_500K).start()
    ids = [0x101, 0x102, 0x103, 0x104]
    for i in ids:
        assert bench.driver.send(i, bytes([i & 0xFF, (i >> 8) & 0xFF])).acked

    assert bench.checker.violations == [], bench.report()
    assert bench.checker.ids == ids, bench.report()
    assert all(f.acked for f in bench.checker.frames)
    assert [r.frame.identifier for r in bench.node_frames()] == ids
    assert (bench.node.tec, bench.node.rec) == (0, 0), bench.node.error_log
    # Every frame was acknowledged and no error flag went out on the wire.
    assert bench.checker.error_flags == [], bench.report()


# ---- the invariants the driver and the program depend on ----------------------------
def test_the_program_fits_the_instruction_memory():
    """Five words of thirty-two, and the loop the driver times is the program itself."""
    program = assemble_program()
    assert len(program.instrs) == 5, (
        f"{len(program.instrs)} words; the docs quote this number "
        f"(docs/mvp-isa.md `### Engine state` gives 32)"
    )
    assert program.sideset_bits == 0, "no side-set: the OUT channel drives the direction"
    assert program.labels["idle"] == 0 and program.labels["engine"] == 2
    assert program.wrap_target == program.labels["idle"]
    assert program.wrap == program.labels["idle_end"]
    assert 0 <= program.wrap_target <= program.wrap, "the wrap window holds the idle loop"
    assert drv.loop_cycles(program) == drv.SM_CYCLES_PER_BIT == 10
    # Every word is one of the five this header documents, so a changed delay in
    # program.pio moves the number the driver derives CLKDIV from.
    assert program.instrs[program.labels["engine"]] == (7 << 8) | (3 << 13) | (4 << 5) | 1
    assert program.instrs[program.labels["engine"] + 1] == (0 << 8) | (2 << 13) | 0 | 1
    assert program.instrs[program.labels["engine"] + 2] == program.labels["engine"]


def test_the_sof_edge_lands_on_the_bus_bit_grid():
    """The alignment the driver computes is the alignment the models see.

    The reference model has no propagation delay and no oscillator error, so a node
    can only hard synchronise on a SOF edge that falls on one of its own bit
    boundaries (`CanNode._bit_boundary`), and those are the multiples of the bit time
    because the model's grid starts at tick 0. `verif/vip/core.py` resolves a drive at
    the end of the tick it was returned for, and the chip's T_OUT is one cycle
    (docs/mvp-isa.md `## Timing`), so the edge a model sees at tick T came out of the
    engine in core cycle T-1.
    """
    for speed in (SPEED_125K, SPEED_500K):
        bench = Bench(speed).start()
        result = bench.driver.send(*FRAME)
        bit_ticks = bench.driver.bit_ticks
        assert result.sof_cycle % bit_ticks == bit_ticks - 1, (
            f"{speed}: SOF `out` in core cycle {result.sof_cycle}, which is "
            f"{result.sof_cycle % bit_ticks} past a bit boundary of {bit_ticks}"
        )
        # The checker locked its grid on that very edge, one tick after the write.
        assert bench.checker.frames[0].tick % bit_ticks == 0, bench.report()
        assert bench.checker.worst_off_grid <= bench.checker.edge_tol, (
            f"{speed}: an edge sat {bench.checker.worst_off_grid} ticks off the grid"
        )


def test_the_driver_addresses_match_the_register_map_and_the_iss():
    """The driver is written against docs/mvp-regmap.md; this is the guard against the
        two drifting apart, in particular against the ISS renumbering something."""
    assert (drv.A_CTRL, drv.A_FSTAT, drv.A_FDEBUG, drv.A_FLEVEL) == (
        pio_iss.A_CTRL, pio_iss.A_FSTAT, pio_iss.A_FDEBUG, pio_iss.A_FLEVEL)
    assert (drv.A_TXF, drv.A_RXF, drv.A_IMEM) == (pio_iss.A_TXF, pio_iss.A_RXF, pio_iss.A_IMEM)
    assert (drv.A_DBG_PADOUT, drv.A_DBG_PADOE, drv.A_DBG_CFGINFO) == (
        pio_iss.A_DBG_PADOUT, pio_iss.A_DBG_PADOE, pio_iss.A_DBG_CFGINFO)
    assert (drv.SM_CLKDIV, drv.SM_EXECCTRL, drv.SM_SHIFTCTRL, drv.SM_ADDR,
            drv.SM_INSTR, drv.SM_PINCTRL) == (
        pio_iss.SM_CLKDIV, pio_iss.SM_EXECCTRL, pio_iss.SM_SHIFTCTRL, pio_iss.SM_ADDR,
        pio_iss.SM_INSTR, pio_iss.SM_PINCTRL)
    assert drv.A_SM == pio_iss.A_SM_BASE
    assert (drv.DOMINANT, drv.RECESSIVE) == (VIP_DOMINANT, VIP_RECESSIVE), (
        "the driver and the model must agree on which level is dominant (sec. 2)"
    )


def test_the_driver_and_the_model_build_the_same_frame():
    """Two independent transcriptions of sec. 3.1.1, 5 and 3.1.1's CRC.

    `fw/can/driver.py` was written from the Bosch CAN 2.0 text
    and `verif/vip/can.py` was written independently from the same sources, so
    agreeing bit for bit is evidence that both read the spec the same way. The CRC is
    additionally checked against a plain polynomial division of the same bit string,
    which is a third way of computing it.
    """
    cases = [
        (0x123, b"\xde\xad", False, None),
        (0x000, b"", False, None),                 # DLC 0, and ID-10..ID-4 recessive
        (0x7FF, b"\xff" * 8, False, None),          # all recessive: the longest run
        (0x123, b"\x00\x00\x00\x00", False, None),  # all dominant: the other longest run
        (0x555, b"", True, 8),                      # REMOTE FRAME with a DLC but no data
        (0x2AA, b"\x55", False, 1),                 # a DLC that does not match the data
    ]
    for identifier, data, rtr, dlc in cases:
        mine, roles = drv.wire_bits(identifier, data, rtr, dlc, tail=0)
        theirs = vip_wire_bits(Frame(identifier, data, rtr, dlc))
        assert mine == theirs, f"{identifier:#05x} {data!r}: wire bits differ"
        assert len(mine) == len(roles)
        assert roles.count("id") == 11 and roles.count("crc") == 15
        assert roles[-1] != "stuff", "the fixed-form tail is never stuffed (sec. 5)"
        # A third CRC: divide the destuffed SOF..DATA FIELD by the generator polynomial.
        region = drv.stuffed_region(len(data) if dlc is None else dlc, rtr)
        destuffed, stuffed_count = _destuff(mine)
        body = destuffed[:region]
        assert len(body) == region, f"{identifier:#05x}: destuffing lost bits"
        assert drv.crc15(body) == _crc_by_division(body), f"{identifier:#05x}"
        # sec. 5: one complementary bit after every five equal ones, and never outside the
        # stuffed region, so the wire is never longer than the frame by more than that.
        assert stuffed_count == len(mine) - len(destuffed)


def _destuff(wire):
    """Undo `stuff` for the length check above; returns (bits, how many were stuffed)."""
    out, run, prev, stuffed_bits = [], 0, drv.RECESSIVE, 0
    for bit in wire:
        if run == drv.STUFF_RUN:
            stuffed_bits += 1
            run, prev = 1, bit
            continue
        out.append(bit)
        run = run + 1 if bit == prev else 1
        prev = bit
    return out, stuffed_bits


def _crc_by_division(bits):
    """The CRC as a polynomial long division, i.e. a third implementation.

    The register is 15 bits wide (CRC14..CRC0) and the generator has an x^15 term, so
    shifting a bit in can put a 1 above the register: that is the bit that means "the
    running polynomial is now of degree >= 15", and it is cleared by XOR-ing the
    generator in. This computes the same 15 bits as the shift register `crc15` does, by
    the other method.
    """
    remainder = 0
    for bit in bits:
        remainder = (remainder << 1) | bit
        if remainder & (1 << 15):
            remainder ^= drv.CRC_POLY
    return remainder & ((1 << 15) - 1)


def test_the_word_packing_round_trips():
    """The OSR words and the readback words are each other's complement.

    `program.pio` shifts MSB first in both directions, so word k's bit 31 is wire
    bit 32k and the readback of a word nobody else drove is its complement: a released
    line reads high. The one bit that breaks the pattern is the ACK SLOT.
    """
    wire, roles = drv.wire_bits(0x123, b"\xde\ad")
    words = drv.tx_words(wire)
    frame_bits = [b for b, r in zip(wire, roles) if r != "intermission"]
    assert len(wire) == len(frame_bits) + drv.TAIL_BITS, len(wire)
    assert len(frame_bits) == len(vip_wire_bits(Frame(0x123, b"\xde\ad"))), (
        "the frame itself, without the tail, is what the model builds"
    )
    assert roles[-1] == "intermission" and roles[-1] == roles[-drv.TAIL_BITS]
    assert len(words) == 3 and words[-1] == 0, (
        "the last word is all recessive padding, which releases the line"
    )
    dirs = drv.directions(wire)
    for i, word in enumerate(words):
        for k in range(32):
            if 32 * i + k < len(wire):
                assert (word >> (31 - k)) & 1 == dirs[32 * i + k], f"word {i} bit {k}"
        assert drv.readback_word(word) == (~word & 0xFFFF_FFFF)
    assert drv.readback_word(words[0], acked=True) == (~words[0] & 0xFFFF_FFFF) & ~1
    # The OSR word and its readback disagree only where somebody else drove the bus.
    assert drv.ack_bit_mask(roles.index("ack") // 32, roles.index("ack") % 32) != 0


@pytest.mark.parametrize("bad", [
    dict(identifier=0x800),                       # 12 bits: not a standard identifier
    dict(identifier=0x123, data=b"\x00" * 9),     # 9 bytes: sec. 3.1.1 admits 0..8
    dict(identifier=0x123, rtr=True, data=b"\x00"),   # REMOTE FRAME carries no data
    dict(identifier=0x123, dlc=9),                 # DLC 9 is not admissible
])
def test_the_frame_builder_refuses_what_the_spec_forbids(bad):
    """Sec. 3.1.1's three limits and sec. 3.1.2's REMOTE FRAME rule, as errors."""
    with pytest.raises(ValueError):
        drv.logical_bits(**bad)


def test_the_pins_have_to_be_a_uio_pin():
    """Open drain is a direction, and only the bidirectional uio pins have one."""
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    bus.attach(device)
    program = assemble_program()
    for pin in (-1, 8, 13, 18, 31):
        with pytest.raises(ValueError):
            CanDriver(bus, device, program, bit_ticks=400, can_pin=pin)
    for bit_ticks, why in ((7, "whole number"), (13, "whole number"), (401, "whole number"),
                           (0, "out of range")):
        with pytest.raises(CanError, match=why):
            CanDriver(bus, device, program, bit_ticks=bit_ticks)


def test_the_engine_never_stalls_on_a_fifo():
    """A stall inside a frame is a stretched bit time, which is a violation.

    Both FIFOs are four deep and this program keeps one word in flight in each
    direction, so the driver has to keep both flowing: it pushes the next TX word
    whenever there is room and takes the readback out as it arrives. If it ever failed
    to, FDEBUG would carry TXSTALL or RXSTALL and `send` would raise.
    """
    bench = Bench(SPEED_500K).start()
    bench.driver.clear_fdebug()
    result = bench.driver.send(0x6FE, b"\xff" * 8)
    assert bench.driver.fdebug() == 0, f"FDEBUG {bench.driver.fdebug():#010x}"
    assert result.acked
    assert len(result.readback) == len(result.tx_words)
    assert bench.checker.violations == [], bench.report()


# ---- what the MVP cannot express, and why -------------------------------------------
CANNOT_EXPRESS = Path(__file__).with_name("CANNOT-EXPRESS.md")


@pytest.mark.skip(reason=f"the MVP PIO cannot do this; see {CANNOT_EXPRESS.name}: "
                         "no XOR and no data memory for a CRC-15 (sec. 3.1.1), no "
                         "destuffing, and a receiver that cannot destuff cannot find "
                         "the ACK SLOT to acknowledge in")
def test_the_chip_cannot_receive_and_acknowledge():
    """A CAN *target*: drive the ACK SLOT dominant, after checking the CRC.

    This is the half of the protocol that needs a receiver, and it is missing. The
    ACK SLOT's position on the wire depends on the DATA LENGTH CODE *and* on how many
    stuff bits were inserted before it, so a receiver has to destuff before it can
    count; and it may only acknowledge a frame whose CRC it matched, which needs the
    CRC-15 of sec. 3.1.1 -- an XOR-feedback shift register, which the MVP instruction
    set has no instruction for and no memory to tabulate. `fw/can/CANNOT-EXPRESS.md`
    has the full map.
    """
    bench = Bench(SPEED_500K, peer=False).start()
    bench.node.send(0x123, b"\xde\xad")             # the model transmits instead
    bench.node.run(bench.bus, limit=200_000)
    assert bench.checker.violations == [], "would pass: the chip ACKed"


@pytest.mark.skip(reason=f"the MVP PIO cannot do this; see {CANNOT_EXPRESS.name}: "
                         "sec. 7 needs two saturating 8-bit error counters, a "
                         "128 x 11 recessive-bit recovery counter and an ERROR FLAG "
                         "that starts within one bit of the error, with no IRQ to "
                         "report any of it")
def test_fault_confinement_and_error_frames():
    """sec. 3.1.3 ERROR FLAG, sec. 7 fault confinement, sec. 3.1.5 SUSPEND TRANSMISSION.

    All of it is missing: the counters are 8-bit saturating registers with thresholds
    at 128 and 256, the recovery from bus-off needs 128 occurrences of 11 recessive
    bits, and an ERROR FLAG has to be superposed on the bus within one bit time of the
    error that caused it. A PIO with 32 words of IMEM, four-deep FIFOs and no IRQ has
    nowhere to keep the state and no way to be told. `fw/can/CANNOT-EXPRESS.md` has
    the word budget.
    """
    bench = Bench(SPEED_500K).start()
    bench.driver.send(0x123, b"\xde\xad")
    assert bench.node.error_log == [], "would pass: the chip counted errors"


# ---- the speed limit, measured -------------------------------------------------------
MAX_SPEED_CSV = Path(__file__).resolve().parents[2] / "metrics" / "fw-can-speed.csv"


def test_the_fastest_bit_time_the_checker_still_accepts():
    """How fast this program can go, measured rather than argued.

    The program costs 10 SM cycles per bit and the divider cannot be finer than
    one core cycle per SM cycle, so the shortest bit time the loop can express is
    `10 * CLKDIV` core cycles -- 200 ns at CLKDIV 1, 5 Mbit/s, far above anything CAN
    allows (`04-bosch-bit-timing.txt` sec. 1: "bitrates up to 1 Mbit/s"). The useful
    number is the fastest bit time the *checker* still accepts on this 20 ns tick grid,
    with the time quantum a whole number of ticks, so this sweeps the speeds and records
    the boundary. `metrics/fw-can-speed.csv` is the sweep.
    """
    from fw.can import measure

    rows = measure.sweep()  # noqa: F841 -- returned for a human reading the -v output
    assert rows, "no bit time was measured"
    usable = measure.usable_rows(rows)
    fastest = usable[0]

    # The boundary: the shortest bit time this program can put a frame out on, and the
    # recorded numbers, so a change in program.pio or in the model fails here.
    assert (fastest["bit_ticks"], int(fastest["bitrate"]), fastest["clkdiv"]) == (
        measure.FASTEST_BIT_TICKS, measure.FASTEST_BITRATE, measure.FASTEST_CLK_DIV), (
        f"the fastest bit time moved: {fastest}; re-run python -m fw.can.measure"
    )
    assert fastest["violations"] == 0 and fastest["acked"] and fastest["node_saw"], fastest
    assert fastest["worst_off_grid"] <= 1, f"an edge sat off the grid: {fastest}"

    # Every bit time this program can time is clean, and every one it cannot time says
    # so: the sweep is a boundary, not a single lucky point.
    for row in rows:
        if row["expressible"] and row["tq_ticks"]:
            assert row["usable"], f"{row['bit_ticks']} core cycles per bit: {row}"
        elif not row["expressible"]:
            assert row["bit_ticks"] % drv.SM_CYCLES_PER_BIT, row
            assert "cannot time" in row["note"], row

    # 1 Mbit/s is the top of the classic CAN range and is exactly reachable: 50 core
    # cycles is ten SM cycles times CLKDIV 5 (`04-bosch-bit-timing.txt` sec. 1).
    ceiling = [r for r in rows if r["bit_ticks"] == measure.CAN_CEILING_BIT_TICKS][0]
    assert ceiling["usable"] and ceiling["clkdiv"] == measure.CAN_CEILING_CLK_DIV, ceiling
    assert ceiling["bitrate"] == "1000000", ceiling

    # The committed CSV has to be the sweep this test just took, or the numbers in
    # fw/can/README.md and CANNOT-EXPRESS.md are quoting an old run.
    assert MAX_SPEED_CSV.is_file(), f"{MAX_SPEED_CSV} is missing: re-run python -m fw.can.measure"
    with MAX_SPEED_CSV.open() as handle:
        recorded = list(csv.DictReader(handle))
    assert len(recorded) == len(rows), f"{len(recorded)} rows recorded, {len(rows)} measured"
    for saved, row in zip(recorded, rows):
        assert saved["bit_ticks"] == str(row["bit_ticks"])
        assert saved["violations"] == str(row["violations"]), f"{saved} vs {row}"
        assert saved["usable"] == str(row["usable"]), f"{saved} vs {row}"
"""End-to-end test of the MVP MDIO/MDC firmware (task B3-FW-mdio).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with nets
p0..p17, exactly as `docs/mvp-interfaces.md` `## ISS API` describes; the host driver loads
program.pio and puts clause 22 frames on the two nets; and `verif.vip/mdio.py` -- the
independent judge, which this test does not modify -- answers them.  `MdioPhy` is a clause 22
PHY with the datasheet's register file, and `MdioChecker` decodes every frame off the pins and
reports each timing or framing rule the bus breaks.  **The bar is that the checker
reports zero violations.**

    make fw PROTO=mdio TARGET=iss      # runs
    FW_TARGET=iss_b python -m pytest -q fw/mdio    # the second ISS: a third independent check
    FW_TARGET=rtl    python -m pytest -q fw/mdio    # src/pio_chip.sv under cocotb

Nothing here looks at the ISS internals to decide pass or fail.  The values come from
`MdioPhy` (what the bus actually delivered to a PHY), from `MdioChecker.frames` (what a
monitor decoded off the pins) and from the RX FIFO word the driver read back over the host
port; the chip's own `pin_out`, `osr` and `isr` are only ever used to show that the program
words loaded are the ones `asm/pioasm.py` assembled.

Three things the tests pin down that matter here:

* **The chip is the station**, not a PHY: it owns MDC and drives the data, and `MdioPhy` only
  answers a read's turnaround and data bits.  So the *station* model in `verif/vip/mdio.py` is
  the thing this firmware replaces, and the two models that judge it are the ones that stay.
* **Speed.** `MdioChecker.max_mdc_hz` is the fastest MDC the bus carried, measured from the
  pins, and the tests ask for three rates including the 2.5 MHz ceiling clause 22 puts on it.
* **Words.** `check_program` reads the shape of program.pio back out of the assembled words,
  so the IMEM budget is measured rather than asserted by hand.

The wiring matters and is not incidental: MDIO is a wired-AND line with a pull-up, so its net
is `push_pull=True` and every device drives 0 or releases.  A PIO that drove a hard 1 would
show up as contention, and a PIO that kept driving through a PHY's turnaround would show up as
contention too -- which is exactly what `test_the_judge_is_not_vacuous` plants.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field

import pytest

from fw.mdio import driver as drv
from fw.mdio.driver import (
    CORE_HZ,
    FRAME_BITS,
    IMEM_SIZE,
    MAX_MDC_HZ,
    MdioBusMonitor,
    MdioDriver,
    bit_cycles,
    check_program,
    clkdiv_for_mdc,
    clkdiv_value,
    control_word,
    decode_word,
    frame_words,
    load_program,
    read_data,
    ticks_per_bit,
    words_used,
)
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.mdio import (
    BMSR,
    BMSR_LINK,
    BMCR,
    DATA_BITS,
    OP_READ,
    OP_WRITE,
    POWER_ON_DEFAULTS,
    PREAMBLE_BITS,
    ST_END,
    attach_checker,
    attach_phy,
    bit_offset,
    bit_role,
)
from fw import regmap as _regmap

NS_PER_TICK = 20.0                    # 50 MHz chip clock, one Bus tick
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_TXSTALL = 0x0100_0000          # bit 24 + engine (docs/mvp-regmap.md `**FDEBUG**`)
FDEBUG_RXSTALL = 0x0000_0001          # bit 0 + engine
FDEBUG_TXOVER = 0x0001_0000           # bit 16 + engine
FDEBUG_RXUNDER = 0x0000_0100          # bit 8 + engine
FDEBUG_BAD = FDEBUG_TXOVER | FDEBUG_RXUNDER | FDEBUG_RXSTALL

#: Clause 22 register indexes used below (verif/vip/mdio.py, lan8720a-clause22-registers.txt).
ANAR = 4
PHYIDR1 = 2


@dataclass(frozen=True)
class Case:
    """One configuration: an MDC rate, the PHY addresses on the bus, pins and engine."""

    name: str
    mdc_hz: float
    phyads: tuple = (0,)
    mdc_pin: int = 0
    mdio_pin: int = 1
    engine: int = 0
    #: Frames to put on the bus: a write, two reads and a second write.
    burst: tuple = (("w", BMCR, 0x2100), ("r", BMCR, None), ("r", BMSR, None),
                    ("w", ANAR, 0x01FD), ("r", ANAR, None))


#: Three rates and three pin/engine placements.  2.5 MHz is the clause 22 ceiling
#: (`MDC period >= 400 ns`), so the fastest case is exactly as fast as the protocol allows.
CASES = [
    Case("2.5MHz-max", MAX_MDC_HZ),
    Case("1.25MHz", 1_250_000.0),
    Case("500kHz", 500_000.0),
    Case("2.5MHz-two-phys", MAX_MDC_HZ, phyads=(0, 7)),
    Case("2.5MHz-engine1-p5", MAX_MDC_HZ, mdc_pin=5, mdio_pin=6, engine=1),
]


class Bench:
    """A bus with the chip as the station, the PHYs, and the passive checker.

    The PHYs and the checker are wired to the same two nets the chip drives, by name, so what
    they see is the pad, with the pull-up and the contention detector of `verif/vip/core.py`
    in between -- the same wiring a real MDIO bus has, and the same wiring the RTL will have.
    """

    def __init__(self, case: Case, *, driver_cls=MdioDriver, program=None, driver_kw=None,
                 keep_bits: int = 0, phy_kw=None):
        self.case = case
        self.bus = Bus(ns_per_tick=NS_PER_TICK)
        for k in range(18):
            # MDIO is the bidirectional wired-AND line: push-pull, so two devices driving
            # opposite values is recorded as contention (docs/mvp-isa.md `### Open drain`).
            self.bus.add_net(f"p{k}", push_pull=(k == case.mdio_pin))
        self.chip = ChipDevice()
        self.device = self.bus.attach(self.chip)
        self.monitor = MdioBusMonitor(case_net(case, "mdc"), case_net(case, "mdio"),
                                     keep_bits=keep_bits)
        self.bus.attach(self.monitor)
        nets = dict(mdc_net=case_net(case, "mdc"), mdio_net=case_net(case, "mdio"))
        self.phys = {a: attach_phy(self.bus, phyad=a, **(phy_kw or {}), **nets)
                     for a in case.phyads}
        self.checker = attach_checker(self.bus, expect_phyad=set(case.phyads), **nets)
        self.program = program if program is not None else load_program()
        self.driver = driver_cls(self.device, self.bus, engine=case.engine,
                                 mdc_pin=case.mdc_pin, mdio_pin=case.mdio_pin,
                                 mdc_hz=case.mdc_hz, program=self.program,
                                 monitor=self.monitor, **(driver_kw or {}))
        self.driver.start()

    # ---- what to ask the chip for ----
    def burst(self, case: Case = None) -> list:
        """Run the case's burst of frames and return the ops, reads filled in."""
        case = case or self.case
        ops = []
        for kind, reg, value in case.burst:
            if kind == "w":
                ops.append(self.driver.write_reg(reg, value))
            else:
                ops.append(self.driver.read_reg(reg))
        return ops

    def settle(self, ticks: int = 400) -> "Bench":
        """Let the bus go quiet after the traffic.

        The checker calls a frame finished, and the bus idle, only once MDC has stayed put for
        its own idea of a quiet window, which is three spec bit times (verif/vip/mdio.py
        `MdioChecker.quiet_ticks`).  Too few ticks and a test would miss a `frame`, `release`
        or `idle` violation that is there.
        """
        self.bus.run(ticks)
        return self

    def rules(self) -> set:
        return {v.rule for v in self.checker.violations}

    def no_violations(self) -> None:
        """The bar: the checker must find nothing wrong with the bus."""
        assert self.checker.clean, self.checker.report()

    def report(self) -> str:
        return self.checker.report()


def case_net(case: Case, which: str) -> str:
    """The Bus net name of one of the two MDIO lines, which are the chip's p0..p17 pins."""
    return f"p{case.mdc_pin if which == 'mdc' else case.mdio_pin}"


def phy_value(bench: Bench, reg: int, phyad: int = 0) -> int:
    """What the PHY at `phyad` holds in `reg`, with BMSR's link latch applied."""
    return bench.phys[phyad].read_reg(reg)


# --------------------------------------------------------------------------------------------
# The transaction, judged by the VIP
# --------------------------------------------------------------------------------------------

@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_clause_22_traffic_is_clean(case: Case) -> None:
    """A burst of frames at this rate breaks no rule the checker knows.

    The traffic covers both directions and both field widths: a write and a read of a 5-bit
    register index, a read of the link status, and a write/read pair of a 16-bit value.  The
    burst is run in one `run()` so the frames go out back to back, which is what a station
    does when it walks a PHY's register file.
    """
    bench = Bench(case)
    ops = bench.burst()
    bench.settle()
    bench.no_violations()

    # One frame per op, in order, and every one of them decoded by the monitor.
    assert len(bench.checker.frames) == len(case.burst), bench.report()
    for frame, (kind, reg, value) in zip(bench.checker.frames, case.burst):
        assert frame.is_write == (kind == "w"), f"{frame} is not a {kind}"
        assert frame.phyad in case.phyads, f"{frame} addressed {frame.phyad}"
        assert frame.reg == reg, f"{frame} decoded REGAD {frame.reg}, asked for {reg}"
        assert frame.st_bits == 0b01, f"{frame} start code is not 01"
        assert frame.op_bits == (OP_WRITE if kind == "w" else OP_READ), f"{frame} opcode"
        assert frame.preamble_bits == PREAMBLE_BITS, f"{frame} carried a short preamble"
        assert frame.ta == 0b10, f"{frame} turnaround is {frame.ta:02b}, expected 10"
        assert not frame.unanswered, f"{frame} came back as ones"
        if kind == "w":
            assert frame.data == value, f"{frame} carried 0x{frame.data:04x}, sent 0x{value:04x}"
    # The rate the *bus* carried, measured off the pins, is the one that was asked for.
    assert bench.checker.shortest_period == pytest.approx(
        bench.driver.ticks_per_bit, abs=0.02), (
        f"the bus ran at {bench.checker.max_mdc_hz / 1e6:.4f} MHz, the driver asked for "
        f"{CORE_HZ / bench.driver.ticks_per_bit / 1e6:.4f} MHz")


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_data_reaches_the_phy_and_comes_back(case: Case) -> None:
    """The register file on the far end of the bus is what the frames said.

    `MdioPhy` is the only thing that applies a write, and reading it back is a different path
    from the one the driver uses: the driver reads the chip's RX FIFO, the test reads the PHY's
    register file.  If the two disagree, one of them is wrong and the test says which.
    """
    bench = Bench(case)
    ops = bench.burst()
    bench.settle()
    bench.no_violations()

    phyad = case.phyads[0]
    assert phy_value(bench, BMCR, phyad) == 0x2100, "the BMCR write did not land"
    assert phy_value(bench, ANAR, phyad) == 0x01FD, "the ANAR write did not land"
    # The reads the host made, against what the PHY holds.
    assert ops[1].value == 0x2100, f"BMCR read back 0x{ops[1].value:04x}"
    assert ops[2].value == phy_value(bench, BMSR, phyad), "BMSR read disagrees with the PHY"
    assert ops[2].value & BMSR_LINK, "the link is up, so BMSR bit 2 must be set"
    assert ops[4].value == 0x01FD, f"ANAR read back 0x{ops[4].value:04x}"
    # And the RX word really is {TA1, TA2, DATA}: the turnaround bits sit above the data.
    for op in (ops[1], ops[2], ops[4]):
        assert op.value is not None
    assert bench.phys[phyad].accesses, "the PHY logged no access at all"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_chip_leaves_the_block_clean(case: Case) -> None:
    """No word dropped, no read underflow, only the two MDIO pins driven, only this engine used.

    FDEBUG is the chip's own account of the run: a TXOVER or RXUNDER here would mean a word was
    lost, and a TXSTALL would mean the engine sat on `pull block` waiting for a word the host
    had not sent yet.  The driver pushes both words before each frame, so neither may happen.
    """
    bench = Bench(case)
    bench.burst()
    bench.settle()
    bench.no_violations()
    drv_, chip = bench.driver, bench.device.chip

    fdebug = drv_.fdebug()
    assert fdebug == 0, f"FDEBUG {fdebug:#010x}: a stall, an overrun or an underflow"
    assert drv_.halted(), f"engine 0 is at PC {drv_.pc()}, not parked in halt"

    # MDC stays driven (it is parked high); MDIO is driven after a write and released after a
    # read, because a read hands the line to the PHY and the bus must come back to the pull-up.
    padoe = drv_.padoe()
    want = (1 << case.mdc_pin) | (1 << case.mdio_pin)
    ends_in_write = case.burst[-1][0] == "w"
    driven = (1 << case.mdc_pin) | ((1 << case.mdio_pin) if ends_in_write else 0)
    assert padoe & 0x000FF & ~want == 0, (
        f"padoe {padoe:#06x}: a program window leaked onto another uio pin")
    assert padoe & want == driven, (
        f"padoe {padoe:#06x}: expected {driven:#06x} for a burst ending in "
        f"{'a write' if ends_in_write else 'a read'}")
    # Bits 12:8 are input-only pins and read 0, 17:13 are output-only and read 1.
    assert padoe & 0x1F00 == 0, f"padoe {padoe:#06x}: an input-only pin reads as driven"
    assert padoe & 0x3E000 == 0x3E000, f"padoe {padoe:#06x}: an output-only pin is not driven"

    other = 1 - case.engine
    assert chip.state(other)["pc"] == 0, "the other engine ran"
    assert not chip.engines[other].tx, "the other engine got data"
    assert not bench.bus.errors, f"contention on the bus: {bench.bus.errors[:2]}"
    assert drv_.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_the_words_in_imem_are_the_ones_assembler_made(case: Case) -> None:
    """IMEM holds program.pio's words, and nothing was written past them."""
    bench = Bench(case)
    bench.burst()
    bench.settle()
    imem = bench.device.chip.engines[case.engine].imem
    assert imem[: bench.driver.words_used] == bench.program.instrs
    assert set(imem[bench.driver.words_used:]) <= {0}, "the driver wrote past the program"


# --------------------------------------------------------------------------------------------
# What clause 22 says about the far end
# --------------------------------------------------------------------------------------------

def test_two_phys_answer_only_their_own_address() -> None:
    """Two PHYs on one bus: a write and a read reach the PHY that was addressed."""
    case = Case("two", MAX_MDC_HZ, phyads=(0, 7))
    bench = Bench(case)
    bench.driver.write_reg(BMCR, 0x2100, phyad=0)
    bench.driver.write_reg(BMCR, 0x0040, phyad=7)
    seven = bench.driver.read_reg(BMCR, phyad=7)
    zero = bench.driver.read_reg(BMCR, phyad=0)
    bench.settle()
    bench.no_violations()

    assert phy_value(bench, BMCR, 0) == 0x2100
    assert phy_value(bench, BMCR, 7) == 0x0040
    assert zero.value == 0x2100 and seven.value == 0x0040
    assert [f.phyad for f in bench.checker.frames] == [0, 7, 7, 0]


def test_a_read_to_an_empty_address_comes_back_as_ones() -> None:
    """0xffff is clause 22's "no PHY there": the pull-up holds MDIO and nobody drives.

    This is how a station scans the 32 addresses for PHYs, so the firmware has to be able to
    tell it from a register that happens to read 0xffff -- and the checker must not call it a
    violation, because nothing broke: the wire was correct for a bus with no PHY at 5.
    """
    case = Case("empty", MAX_MDC_HZ, phyads=(0,))
    bench = Bench(case)
    op = bench.driver.read_reg(BMCR, phyad=5)
    bench.settle()
    bench.no_violations()
    assert op.value == 0xFFFF and op.unanswered
    assert bench.checker.frames[-1].unanswered, "the monitor did not see the ones"
    assert bench.phys[0].regs[BMCR] == POWER_ON_DEFAULTS[BMCR], "the wrong PHY was written"


def test_link_status_latches_low_until_read() -> None:
    """BMSR bit 2 is RO/LL: a falling link latches it low, and reading BMSR releases it."""
    case = Case("link", MAX_MDC_HZ)
    bench = Bench(case)
    up = bench.driver.read_reg(BMSR).value
    bench.phys[0].link = False
    bench.phys[0].link = True                 # back up without polling in between
    latched = bench.driver.read_reg(BMSR).value
    released = bench.driver.read_reg(BMSR).value
    bench.settle()
    bench.no_violations()

    assert up & BMSR_LINK, "the link is up, so BMSR bit 2 must be set"
    assert not latched & BMSR_LINK, "the outage did not latch bit 2 low"
    assert released & BMSR_LINK, "reading BMSR did not release the latch"
    assert (up & ~BMSR_LINK) == (released & ~BMSR_LINK), "the rest of BMSR moved"


def test_the_post_reset_silent_window_is_visible_on_the_wire() -> None:
    """A soft reset (BMCR bit 15) silences the PHY for 3 us, and a read inside it reads ones.

    The DP83848 asks for 3 us of silence after a reset
    (`research/ref/protocols/mdio/ti-dp83848-smc-timing.txt`), and `MdioPhy(init_ticks=150)`
    models that at 50 MHz.  The interesting part is what the *checker* makes of it: a passive
    monitor cannot know the PHY is busy, so the missing turnaround is reported as `TA2`, and
    the station has to wait the window out and ask again (verif/vip/mdio.py, judgement call
    10).  That is why this test expects a violation here and none anywhere else.
    """
    case = Case("reset", MAX_MDC_HZ)
    bench = Bench(case, phy_kw={"init_ticks": 150})
    bench.driver.write_reg(BMCR, 0x8000)           # self-clearing soft reset
    early = bench.driver.read_reg(BMSR)
    bench.settle()
    assert early.value == 0xFFFF and early.unanswered, "a read in the window should read ones"
    assert bench.rules() == {"TA2"}, bench.report()
    assert any("post-reset window" in e for e in bench.phys[0].errors)

    bench.bus.run(200)                            # 4 us: past the 3 us window
    later = bench.driver.read_reg(BMSR)
    bench.settle()
    assert later.value == phy_value(bench, BMSR), "the PHY did not answer after the window"
    assert bench.rules() == {"TA2"}, f"the recovery frame broke something new: {bench.report()}"


# --------------------------------------------------------------------------------------------
# An independent decode: what the driver sent, against what the monitor sampled
# --------------------------------------------------------------------------------------------

def test_the_bits_on_the_wire_are_the_frame_the_host_built() -> None:
    """Decode the monitor's log with `verif/vip/mdio.py`'s own field map and compare.

    This does not use `MdioChecker` at all: it takes the bit the monitor sampled on each MDC
    rising edge, walks the clause 22 field map (`bit_role`, `bit_offset`), and rebuilds both
    frames.  So it is a second opinion on the same wire, from the same reference the checker
    decodes with, and it fails if the checker and the host ever agree on something wrong.
    """
    case = Case("decode", MAX_MDC_HZ, phyads=(0, 5))
    bench = Bench(case, keep_bits=8)
    write = bench.driver.write_reg(PHYIDR1, 0xBEEF, phyad=5)
    read = bench.driver.read_reg(BMSR)
    bench.settle()
    assert not bench.checker.violations, bench.report()

    def decode(index: int) -> dict:
        """Rebuild one frame out of the bits the monitor sampled."""
        bits = bench.monitor.frame(index)
        assert len(bits) == FRAME_BITS, f"frame {index} has {len(bits)} bit times, not {FRAME_BITS}"
        fields: dict = {}
        for i, bit in enumerate(bits):
            fields.setdefault(bit_role(i), []).append((bit_offset(i), bit))
        assert all(bit == 1 for _, bit in fields["preamble"]), "the preamble was not 32 ones"
        assert len(fields["data"]) == DATA_BITS, f"{len(fields['data'])} data bits"
        return {
            "st": sum(bit << (1 - off) for off, bit in fields["st"]),
            "op": sum(bit << (1 - off) for off, bit in fields["op"]),
            "phyad": sum(bit << (4 - off) for off, bit in fields["phyad"]),
            "regad": sum(bit << (4 - off) for off, bit in fields["regad"]),
            "ta": [bit for _, bit in fields["ta1"]] + [bit for _, bit in fields["ta2"]],
            "data": sum(bit << (15 - off) for off, bit in fields["data"]),
        }

    sent, got = decode(0), decode(1)
    assert (sent["st"], sent["op"], sent["phyad"], sent["regad"]) == (0b01, OP_WRITE, 5, PHYIDR1)
    assert sent["ta"] == [1, 0] and sent["data"] == 0xBEEF, sent
    assert (got["st"], got["op"], got["phyad"], got["regad"]) == (0b01, OP_READ, 0, BMSR)
    assert got["data"] == read.value == phy_value(bench, BMSR), (
        f"the wire carried 0x{got['data']:04x}, the host read 0x{read.value:04x}")
    assert write.value == 0xBEEF and phy_value(bench, PHYIDR1, 5) == 0xBEEF

    # The word the driver built is the one the frame carries, bit for bit.
    word = control_word(True, 5, PHYIDR1, 0xBEEF)
    assert [word >> (31 - i) & 1 for i in range(32)] == bench.monitor.frame(0)[32:64]
    assert frame_words(True, 5, PHYIDR1, 0xBEEF) == [drv.PREAMBLE_WORD, word]


# --------------------------------------------------------------------------------------------
# What the program and the divisor are, derived from the words rather than asserted by hand
# --------------------------------------------------------------------------------------------

def test_program_fits_the_imem() -> None:
    """program.pio assembles, and the driver reads its shape back out of the words."""
    program = load_program()
    shape = check_program(program)
    assert program.name == "mdio"
    assert words_used(program) == 23, [f"{w:#06x}" for w in program.instrs]
    assert shape["words"] <= IMEM_SIZE, "the program does not fit the 32-word IMEM"
    assert shape["bit_cycles"] == 20, "20 SM cycles per bit, i.e. one delay of 9 and one of 8"
    assert program.sideset_bits == 1 and not program.sideset_opt, ".side_set 1, no opt"
    assert program.wrap == words_used(program) - 1, "WRAP_TOP must be the last instruction"
    assert program.labels["halt"] == 0, "SM_RESTART leaves PC = 0, which must be halt"
    # Both frames are 64 bit times: two words of 32, which is what `jmp !osre` counts.
    for label in ("write_frame", "read_frame"):
        assert label in program.labels, f"the host needs a PUBLIC {label} label"
    pull = [w for w in program.instrs if (w >> 13) == 0b100 and (w & 0x80)]
    assert len(pull) == 2, f"one `pull block` per frame, found {[hex(w) for w in pull]}"


def test_the_frame_is_64_bit_times_of_20_sm_cycles() -> None:
    """The bit time is 20 SM cycles, so CLKDIV = 1.0 is 20 core cycles = 400 ns.

    Every instruction of a bit time is in the words: the low-phase instruction with a delay of
    9, the high phase with a delay of 8, and the `jmp` that costs its own cycle.  That is the
    whole reason the MDC rate is a CLKDIV and not something the host re-times per frame.
    """
    program = load_program()
    words = [decode_word(program, w) for w in program.instrs]
    # The low phase is `out pins, 1` and `in pins, 1` with MDC low; the high phase is `nop`
    # (`mov y, y`) with MDC high.  The third instruction of a bit time is the `jmp` back.
    lows = sorted({f["delay"] for f in words if f["op"] in (0b010, 0b011) and f["sideset"] == 0})
    highs = sorted({f["delay"] for f in words if f["op"] == 0b101 and f["sideset"] == 1})
    jumps = sum(1 for f in words if f["op"] == 0b000)
    assert lows == [9], f"low-phase delays {lows}"
    assert highs == [8], f"high-phase delays {highs}"
    # Nine jumps: two `halt` loops, and three loop-back jumps in each frame.
    assert jumps == 9, f"expected nine jumps, found {jumps}"
    assert bit_cycles(program) == (1 + 9) + (1 + 8) + 1 == 20
    # The fastest CLKDIV is 1.0, and one bit is then exactly the 400 ns the spec allows.
    fastest = ticks_per_bit(clkdiv_value(1), 20)
    assert fastest == 20.0, f"the fastest bit is {fastest} ticks"
    assert CORE_HZ / fastest == MAX_MDC_HZ == 2_500_000.0


@pytest.mark.parametrize("mdc_hz", [2_500_000.0, 2_000_000.0, 1_250_000.0, 1_000_000.0,
                                   625_000.0, 500_000.0, 100_000.0])
def test_clkdiv_hits_the_rate_within_the_divider_resolution(mdc_hz: float) -> None:
    """CLKDIV for a rate is the 16.8 rounding, and CLKDIV is all the program has to go on."""
    clkdiv = clkdiv_for_mdc(mdc_hz, 20)
    integer, frac = (clkdiv >> 16) & 0xFFFF, (clkdiv >> 8) & 0xFF
    assert 1 <= integer <= 0xFFFF and 0 <= frac <= 0xFF
    # One bit is 20 SM cycles and one SM cycle is integer + frac/256 core cycles, so the bit
    # length can only move in steps of 20/256 ticks: half a step, rounded.
    step = 20 / 256
    assert abs(ticks_per_bit(clkdiv, 20) - CORE_HZ / mdc_hz) <= step / 2 + 1e-9


def test_the_driver_refuses_a_rate_above_the_spec_ceiling() -> None:
    """2.5 MHz is a hard limit, so the driver says so instead of building an illegal bus."""
    with pytest.raises(ValueError, match="ceiling"):
        clkdiv_for_mdc(MAX_MDC_HZ * 1.01, 20)
    with pytest.raises(ValueError, match="ceiling"):
        clkdiv_for_mdc(10_000_000.0, 20)
    with pytest.raises(ValueError, match="positive"):
        clkdiv_for_mdc(0.0, 20)
    with pytest.raises(ValueError, match="SM cycle"):
        clkdiv_for_mdc(2_500_000.0, 0)


def test_driver_rejects_setups_it_cannot_honour() -> None:
    """Pins PINDIRS cannot drive, a pin pair the SET window cannot hold, and a full FIFO."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())

    for mdc, mdio in ((8, 9), (12, 13), (0, 17), (0, 18)):
        # MDC has to be a uio pin too: it is the OUT/SET window's neighbour and the side-set
        # pin, and only a uio pin has a direction register.
        with pytest.raises(ValueError, match="uio pins"):
            MdioDriver(dev, bus, mdc_pin=mdc, mdio_pin=mdio)
    with pytest.raises(ValueError, match="mdio_pin must be mdc_pin \\+ 1"):
        MdioDriver(dev, bus, mdc_pin=0, mdio_pin=3)
    with pytest.raises(ValueError, match="engine 2"):
        MdioDriver(dev, bus, engine=2)
    with pytest.raises(ValueError, match="ceiling"):
        MdioDriver(dev, bus, mdc_hz=3_000_000.0)
    with pytest.raises(ValueError, match="REGAD|reg "):
        MdioDriver(dev, bus).write_reg(32, 0)
    with pytest.raises(ValueError, match="phyad"):
        MdioDriver(dev, bus).write_reg(0, 0, phyad=32)
    with pytest.raises(ValueError, match="data"):
        MdioDriver(dev, bus).write_reg(0, 0x1_0000)
    drv_ = MdioDriver(dev, bus)
    with pytest.raises(RuntimeError, match="before load"):
        drv_.run()


def test_bit_cycles_refuses_a_program_it_cannot_time() -> None:
    """The driver will not divide for a program whose bit times are not all alike, or has none."""
    program = load_program()
    uneven = copy.deepcopy(program)
    uneven.instrs[2] -= 1 << 8               # one low-phase delay shortened by one
    with pytest.raises(ValueError, match="not uniform"):
        bit_cycles(uneven)
    loopless = copy.deepcopy(program)
    loopless.instrs = [0xA042] * len(loopless.instrs)      # all `nop`
    with pytest.raises(ValueError, match="no bit-time loop"):
        bit_cycles(loopless)


def test_check_program_refuses_a_program_it_cannot_drive() -> None:
    """`halt` has to be at address 0, and the program has to fit the IMEM."""
    program = load_program()
    moved = copy.deepcopy(program)
    moved.labels["halt"] = 7
    with pytest.raises(ValueError, match="halt"):
        check_program(moved)
    opts = copy.deepcopy(program)
    opts.sideset_opt = True
    with pytest.raises(ValueError, match="side_set"):
        check_program(opts)
    nameless = copy.deepcopy(program)
    nameless.labels.pop("read_frame")
    with pytest.raises(ValueError, match="read_frame"):
        check_program(nameless)
    assert len(program.instrs) <= IMEM_SIZE


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: a broken program must break the traffic
# --------------------------------------------------------------------------------------------

class FastDriver(MdioDriver):
    """A driver whose CLKDIV is too small, so MDC comes out above the 2.5 MHz ceiling."""

    @property
    def clkdiv(self) -> int:
        return clkdiv_value(1)


class BadTurnaroundDriver(MdioDriver):
    """A driver whose write frames send `<11>` instead of `<10>` in the turnaround."""

    def words_for(self, op):
        words = super().words_for(op)
        if op.is_write:
            words[-1] |= 1 << 16                # ta2 from 0 to 1: the turnaround is <11>
        return words


class ShortBitDriver(MdioDriver):
    """A driver whose CLKDIV makes the bit time one tick under the 400 ns floor."""

    @property
    def clkdiv(self) -> int:
        return clkdiv_value(1, 252)            # 20 * (1 + 252/256) = 19.6875 ticks


def _program_without(word_index: int, keep: int = 0xA042) -> object:
    """program.pio with one word replaced by `keep` (a NOP), as the program objects allow."""
    program = copy.deepcopy(load_program())
    program.instrs[word_index] = keep
    return program


def _retimed(bench_case: Case, low: int, high: int, *, stub_bit_cycles=None):
    """A Bench whose program has its bit-time delays changed to `low`/`high`.

    Every low-phase instruction and every high-phase `nop` is retimed, so the program stays
    uniform and `bit_cycles` follows it -- which is the point: the driver is honest about the
    bit time it is given, and the *rate* it can legally reach with that bit time is what
    decides whether the bus breaks the 400 ns floor.  `stub_bit_cycles` freezes what the driver
    thinks the bit time is, which is how "the divider was set for the wrong bit time" is
    modelled.
    """
    program = copy.deepcopy(load_program())
    if stub_bit_cycles is not None:
        original, drv.bit_cycles = drv.bit_cycles, lambda prog: stub_bit_cycles
    try:
        for index, word in enumerate(program.instrs):
            fields = decode_word(program, word)
            if fields["op"] in (0b010, 0b011) and fields["sideset"] == 0:
                field = (fields["sideset"] << 4) | low
                program.instrs[index] = (word & ~(0x1F << 8)) | (field << 8)
            elif fields["op"] == 0b101 and fields["sideset"] == 1:
                field = (fields["sideset"] << 4) | high
                program.instrs[index] = (word & ~(0x1F << 8)) | (field << 8)
        return Bench(bench_case, program=program)
    finally:
        if stub_bit_cycles is not None:
            drv.bit_cycles = original


#: (name, low delay, high delay, what the driver is told the bit time is, rules expected)
RETIMINGS = [
    # A 5 SM cycle bit time: 5 core cycles per bit at CLKDIV 1.0 is 100 ns, five times faster
    # than clause 22 allows, and the divider cannot be told otherwise because the driver is
    # still timing for 20 SM cycles.
    ("half-the-bit-time", 1, 1, 20, {"MDC"}),
    # An 18 SM cycle bit time cannot reach 2.5 MHz on a 16.8 divider: the closest it gets is
    # 18 * (1 + 28/256) = 19.97 core cycles, and the divider rounds some periods down to 18.
    ("eighteen-cycle-bit", 8, 7, None, {"MDC"}),
]


@pytest.mark.parametrize("name,low,high,stub,rules", RETIMINGS, ids=[r[0] for r in RETIMINGS])
def test_a_bit_time_that_cannot_be_mdnslow_breaks_the_period_rule(
        name: str, low: int, high: int, stub, rules: set) -> None:
    """A program whose bit time does not divide into a legal MDC period is caught.

    `MdioChecker`'s `MDC` rule is a floor on the period: 400 ns, so 2.5 MHz.  Nothing else in
    the test suite can produce it -- the clean cases all sit at or above the floor -- so it is
    worth showing that the rule fires when the bit time is retimed.
    """
    bench = _retimed(Case(name, MAX_MDC_HZ), low, high, stub_bit_cycles=stub)
    bench.driver.write_reg(BMCR, 0x2100)
    bench.settle()
    assert bench.checker.shortest_period < bench.checker.min["period_ticks"], (
        f"{name}: the shortest period is {bench.checker.shortest_period} ticks, which is not "
        f"under the floor, so this case cannot test the rule")
    assert rules <= bench.rules(), f"{name}: {bench.report()}"


class BadTurnaroundDriver(MdioDriver):
    """A driver whose write frames send `<11>` instead of `<10>` in the turnaround."""

    def words_for(self, op):
        words = super().words_for(op)
        if op.is_write:
            words[-1] |= 1 << 16                # ta2 from 0 to 1: the turnaround is <11>
        return words


def _release_word_index() -> int:
    """The word of program.pio that releases MDIO for a read's turnaround and data."""
    labels = load_program().labels
    # `tdata` is the first `in pins, 1`; the release is the `set pindirs, 1` three words
    # before it (`set x, 17`, `set pindirs, 1 [7] side 0`, then the label itself).
    return labels["tdata"] - 2


def test_the_judge_is_not_vacuous() -> None:
    """Each way of breaking the firmware is caught, and the clean run is clean first.

    A checker that never fired would pass every test above, so this one plants faults and asks
    `MdioChecker` to name the rule each of them breaks: a write turnaround of `<11>` instead of
    `<10>`, a read that keeps driving MDIO through the PHY's turnaround and data, and (in
    `test_a_bit_time_that_cannot_be_mdnslow_breaks_the_period_rule` above) a bit time that
    cannot make a legal MDC period.  The retimed cases need the driver's own timing model
    frozen, so they live in their own test.
    """
    clean = Bench(Case("clean", MAX_MDC_HZ))
    clean.driver.write_reg(BMCR, 0x2100)
    clean.driver.read_reg(BMCR)
    clean.settle()
    clean.no_violations()

    # A write turnaround of <11> instead of <10>: the second bit must be a driven 0.
    bench = Bench(Case("ta", MAX_MDC_HZ), driver_cls=BadTurnaroundDriver)
    bench.driver.write_reg(BMCR, 0x2100)
    bench.settle()
    assert "TA2" in bench.rules(), bench.report()

    # A read that never releases MDIO: the chip drives the turnaround and the data while the
    # PHY does, which on a push-pull net is contention and on the wire is a TA1 that reads 0.
    # Replacing the release with a `nop` keeps MDC low for that instruction, which is where it
    # already was, so the pin direction is the only thing that changes.
    program = copy.deepcopy(load_program())
    program.instrs[_release_word_index()] = 0xA042         # `nop`
    bench = Bench(Case("release", MAX_MDC_HZ), program=program)
    bench.driver.read_reg(BMSR)
    bench.settle()
    assert bench.rules() & {"TA1", "contention"}, bench.report()
    assert bench.bus.errors, "the core saw no contention either"


def test_a_frame_the_driver_misreads_is_not_hidden_by_the_checker() -> None:
    """The RX word is `{TA1, TA2, DATA[15:0]}`, and only the data is the register value.

    This pins the ISR layout the read path relies on: the driver masks bits 15:0, and a test
    that forgot to would read a turnaround bit as data.  The two turnaround bits are read back
    here from the chip's own RX word, which is the only place they are visible.
    """
    case = Case("isr", MAX_MDC_HZ)
    bench = Bench(case)
    bench.driver.write_reg(BMCR, 0x1234)
    op = bench.driver.read_reg(BMCR)
    bench.settle()
    bench.no_violations()
    assert op.value == 0x1234
    assert read_data(0x2780D) == 0x780D, "read_data masks bits 15:0"
    assert read_data(0x2780D) != 0x2780D, "the turnaround bits sit above the data"

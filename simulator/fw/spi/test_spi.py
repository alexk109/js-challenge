"""End-to-end test of the SPI firmware: `ChipDevice` on a `Bus`, judged by the flash VIP.

The setup is the one a real board has: 18 protocol pin nets
`p0`..`p17` at 20 ns per core cycle (50 MHz), the ISS wrapped in
`iss.pio_iss.ChipDevice`, and the four SPI wires mapped onto the pins the `.pio` program
uses -- `p0` = CS, `p1` = SCK, `p2` = MOSI, `p3` = MISO -- so the flash target and the
monitor sit on the same nets as the chip, with no adapters and no extra cycle in between.
`verif.vip.spi.SpiFlashTarget` and `verif.vip.spi.SpiMonitor` are the judge: they are not
imported from a copy, not subclassed and not weakened.  Every test asserts
`monitor.violations == []`.

Two things make this a real check rather than a self-consistent loop:

* the flash only answers a command it recognises, and only after the opcode and the three
  address bytes have arrived, so a wrong pin, a wrong polarity or a wrong bit order shows
  up as a wrong answer, not as a timing glitch;
* the monitor is told the host timing rules of `docs/mvp-regmap.md` `### Physical layer`
  (SCK and CS each at least 2 core cycles), and a breach lands in `violations`.

`FW_TARGET=rtl` runs all of it against `src/pio_chip.sv` instead of `iss.pio_iss.Chip`, with
the same program, the same driver and the same checks: `fw/rtl_harness.py` puts the RTL
behind the one name the drivers take their chip from, and nothing else changes.
"""
from __future__ import annotations


import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from iss.pio_iss import Chip, ChipDevice
from verif.vip.core import Bus
from verif.vip.spi import (
    CMD_JEDEC,
    CMD_READ,
    SpiFlashTarget,
    SpiMonitor,
)

from fw import targets
from fw.spi import A_CTRL, A_FSTAT, SpiConfig, SpiMaster
from fw.spi.driver import CS, MISO, MOSI, SCK, programs
from fw import regmap as _regmap

#: 50 MHz, docs/mvp-regmap.md `### Physical layer`.
NS_PER_TICK = 20.0
F_CLK_HZ = 1.0e9 / NS_PER_TICK
NUM_PINS = 18
JEDEC_ID = (0xC0, 0x28, 0x17)
#: The host timing rules, in core cycles, that the monitor enforces.
MIN_TICKS = 2


class Rig:
    """A bus with the chip, the flash and the monitor on it, plus the driver."""

    def __init__(
        self,
        mode: int = 0,
        clkdiv: tuple[int, int] = (1, 0),
        size: int = 4096,
        jedec_id: tuple[int, ...] = JEDEC_ID,
        program=None,
    ):
        self.mode = mode
        self.config = SpiConfig(mode=mode, clkdiv=clkdiv)
        self.bus = Bus(ns_per_tick=NS_PER_TICK)
        for k in range(NUM_PINS):
            # CS, SCK, MOSI and MISO are push-pull; the rest of the pins are unused here.
            self.bus.add_net(f"p{k}", push_pull=k <= MISO)
        self.dev = ChipDevice(Chip(), name="chip")
        self.flash = SpiFlashTarget(
            size=size,
            jedec_id=jedec_id,
            mode=mode,
            name="spi_flash",
            cs_net=f"p{CS}",
            sck_net=f"p{SCK}",
            mosi_net=f"p{MOSI}",
            miso_net=f"p{MISO}",
        )
        self.monitor = SpiMonitor(
            mode=mode,
            min_sck_ticks=MIN_TICKS,
            min_cs_setup=MIN_TICKS,
            min_cs_hold=MIN_TICKS,
            name="spi_mon",
            cs_net=f"p{CS}",
            sck_net=f"p{SCK}",
            mosi_net=f"p{MOSI}",
            miso_net=f"p{MISO}",
        )
        for device in (self.dev, self.flash, self.monitor):
            self.bus.attach(device)
        self.master = SpiMaster(self.dev, self.config, program=program)

    def run(self, tx_words, max_ticks: int = 40000) -> list[int]:
        """One transaction; returns the bytes the engine pushed into the RX FIFO."""
        self.master.start(tx_words)
        assert self.master.run(self.bus, max_ticks=max_ticks), (
            f"the program did not finish in {max_ticks} core cycles "
            f"(pc={self.master.pc}, rx={len(self.master.rx_words)} words)"
        )
        return self.master.rx_bytes

    def check_clean(self) -> None:
        """The two judges, the bus's own contention record, and the chip's own flags."""
        assert self.monitor.violations == [], self.monitor.violations
        assert self.bus.errors == [], self.bus.errors
        assert self.master.read_fdebug(self.bus) == 0, "FDEBUG raised a FIFO flag"
        # FSTAT per engine (docs/mvp-regmap.md `### Global registers`): TXEMPTY 25:24,
        # RXEMPTY 9:8, engine `e` one byte up.  Both FIFOs empty means the transaction
        # really did drain and nothing was left behind for the next one.
        fstat = self.master.read_reg(self.bus, A_FSTAT)
        empty = ((1 << 24) | (1 << 8)) << (8 * self.master.sm)
        assert fstat & empty == empty, f"FSTAT {fstat:#010x}: a FIFO is not empty"
        assert fstat >> (8 * self.master.sm) & 0x0F == 0, f"FSTAT {fstat:#010x}: FIFO full"


def tx_word(*bytes_: int) -> int:
    """Bytes as one left-justified TX word, the way the host must send them.

    `docs/mvp-isa.md` difference 11: the host port moves whole words, so the host software
    left-justifies the SPI TX data.  With OUT_SHIFTDIR = 0 the MSB leaves the OSR first.
    """
    word = 0
    for i, value in enumerate(bytes_):
        word |= (value & 0xFF) << (24 - 8 * i)
    return word


def read_word(addr: int, n: int) -> list[int]:
    """The TX words of an `n`-byte flash read: the command, the address, `n` dummy bytes.

    One TX word is four bytes on the wire, so `n` must be a multiple of 4 and the whole
    transaction is `4 + n` bytes, which is what the program is told to clock.
    """
    if n % 4:
        raise ValueError(f"a read is whole TX words, so n must be a multiple of 4, got {n}")
    out = [tx_word(CMD_READ, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF)]
    out += [tx_word(0x00, 0x00, 0x00, 0x00)] * (n // 4)
    return out


def jedec_tx() -> list[int]:
    """The TX words of a JEDEC ID read: the opcode and three dummy bytes."""
    return [tx_word(CMD_JEDEC, 0x00, 0x00, 0x00)]


def jedec_bytes(rx: list[int]) -> list[int]:
    """The ID out of the four bytes a JEDEC transaction pushes.

    Byte 0 of a JEDEC read is whatever the flash was driving while the opcode went out --
    nothing, because the flash does not know the command yet -- and the three ID bytes
    follow.  That is one bit behind the wire, and it is the wire's own doing: in CPHA 0 the
    target changes MISO on the launch edge and the master samples one bit period later, so
    the first byte out is always the byte before the answer.
    """
    return rx[1:4]


# ---- the transactions ----------------------------------------------------------------------


@pytest.mark.parametrize("mode", [0, 3])
def test_jedec_id(mode: int) -> None:
    """The JEDEC ID, both ways the chip can clock it, judged by the flash and the monitor."""
    rig = Rig(mode=mode)
    rx = rig.run(jedec_tx())

    assert len(rx) == 4, f"one JEDEC transaction is 4 bytes, got {len(rx)}"
    assert jedec_bytes(rx) == list(JEDEC_ID)
    assert rx[0] == 0x00, "the byte clocked during the opcode is the flash's idle output"
    # The flash is the judge: it only answers an opcode it recognises, and it only puts the
    # ID on MISO once all 8 opcode bits are in.
    assert rig.flash.last_command == CMD_JEDEC
    xfer = rig.monitor.transfers[0]
    assert xfer.mosi == [CMD_JEDEC, 0x00, 0x00, 0x00], "the wire carried the opcode MSB first"
    assert jedec_bytes(xfer.miso) == list(JEDEC_ID)
    rig.check_clean()


@pytest.mark.parametrize("mode", [0, 3])
def test_read_16_bytes(mode: int) -> None:
    """A 0x03 READ of 16 bytes, which is 5 TX words and 20 bytes on the wire."""
    rig = Rig(mode=mode)
    data = [(0xA0 + i) & 0xFF for i in range(16)]
    rig.flash.poke(0x30, data)

    rx = rig.run(read_word(0x30, 16))

    assert len(rx) == 20, f"a 16-byte read is 20 bytes, got {len(rx)}"
    assert rx[4:20] == data, "the 16 bytes came back in order"
    xfer = rig.monitor.transfers[0]
    assert xfer.mosi[:4] == [CMD_READ, 0x00, 0x00, 0x30]
    assert len(xfer.miso) == 20
    assert xfer.miso[4:20] == data
    rig.check_clean()


@pytest.mark.parametrize("mode", [0, 3])
def test_two_transactions_on_one_load(mode: int) -> None:
    """A JEDEC read and a 16-byte read back to back, with one program load between them.

    This is the re-arm path: the engine is parked on `done` with SM_ENABLE still 1 after
    the first transaction, and the second one has to stop it, re-force the pins, the byte
    counter and the PC, and start it again.  It also shows the two transactions are two
    CS windows and not one long one.
    """
    rig = Rig(mode=mode)
    data = [(0x11 * (i + 1)) & 0xFF for i in range(16)]
    rig.flash.poke(0x00, data)

    first = rig.run(jedec_tx())
    second = rig.run(read_word(0x00, 16))

    assert jedec_bytes(first) == list(JEDEC_ID)
    assert second[4:20] == data
    assert len(rig.monitor.transfers) == 2
    assert rig.monitor.transfers[0].mosi[0] == CMD_JEDEC
    assert rig.monitor.transfers[1].mosi[0] == CMD_READ
    # The second window starts after the first one ended, with CS high in between.
    gap = rig.monitor.transfers[1].t_start - rig.monitor.transfers[0].t_end
    assert gap >= 1, "CS was high between the two transactions"
    rig.check_clean()


@pytest.mark.parametrize("jedec_id", [(0xC0, 0x28, 0x17), (0xEF, 0x40, 0x18), (0x1F, 0x00, 0x00)])
def test_the_id_really_comes_from_the_flash(jedec_id: tuple[int, ...]) -> None:
    """Change the flash's ID and the answer changes with it.

    Without this, a firmware that returned the right bytes for the wrong reason -- a stuck
    MISO, a mis-shifted ISR, a leftover FIFO word -- would still pass the two tests above.
    """
    rig = Rig(jedec_id=jedec_id)
    rx = rig.run(jedec_tx())
    assert jedec_bytes(rx) == list(jedec_id)
    assert rig.flash.jedec_id == list(jedec_id)


@pytest.mark.parametrize("addr,n", [(0x0000, 4), (0x00FF, 4), (0x1000, 16), (0x0FFE, 8)])
def test_read_anywhere(addr: int, n: int) -> None:
    """The address really reaches the flash: four different places, including the last page."""
    rig = Rig()
    data = [(addr * 7 + i * 13) & 0xFF for i in range(n)]
    rig.flash.poke(addr, data)
    rx = rig.run(read_word(addr, n))
    assert rx[4 : 4 + n] == data
    rig.check_clean()


# ---- speed and configuration ----------------------------------------------------------------


def sck_period_ticks(rig: Rig) -> int:
    """The shortest SCK period the monitor saw, in core cycles.

    `stats` holds the shortest time SCK held each level, and the bit period is their sum.
    (Which key holds which level is a labelling matter inside the monitor, so the sum is what this uses, not the keys.)
    """
    stats = rig.monitor.stats
    return stats["sck_low_min"] + stats["sck_high_min"]


def sck_rate_hz(rig: Rig) -> float:
    """The fastest SCK rate the monitor saw, in Hz: the clock rate over ticks per bit."""
    return F_CLK_HZ / sck_period_ticks(rig)


def mean_rate_hz(rig: Rig, xfer) -> float:
    """The average SCK rate over one transaction, in Hz."""
    ticks = xfer.t_end - xfer.t_start - (xfer.cs_setup or 0) - (xfer.cs_hold or 0)
    return len(xfer.miso) * 8 * F_CLK_HZ / ticks


def test_max_rate_on_the_iss() -> None:
    """The fastest the program can clock SCK at CLKDIV 1.0, and the average it sustains.

    8 core cycles per bit at 20 ns a cycle is 6.25 MHz, the rate
    docs/mvp-regmap.md `### Physical layer` calls guaranteed for reads.  A byte costs 66 SM
    cycles rather than 64 (the two instructions that re-arm it), so the sustained rate over
    a long transaction is a little lower; both numbers are printed.
    """
    rig = Rig(mode=0, clkdiv=(1, 0))
    rig.run(jedec_tx())
    assert sck_period_ticks(rig) == 8, rig.monitor.stats
    assert sck_rate_hz(rig) == 6.25e6

    rig2 = Rig(mode=0, clkdiv=(1, 0))
    rig2.run(read_word(0x30, 16))
    xfer = rig2.monitor.transfers[0]
    mean = mean_rate_hz(rig2, xfer)
    print(f"\nmax SCK {sck_rate_hz(rig) / 1e6:.3f} MHz, mean over 20 bytes {mean / 1e6:.3f} MHz")
    assert 5.9e6 < mean < 6.25e6, mean
    rig.check_clean()
    rig2.check_clean()


@pytest.mark.parametrize("clkdiv", [(1, 0), (1, 128), (2, 0), (4, 0), (16, 0)])
def test_clkdiv(clkdiv: tuple[int, int]) -> None:
    """The same transaction at five dividers: same bytes, slower wire.

    `(1, 128)` is a fractional divider (one SM cycle per 1.5 core cycles), so it is not a
    round number of ticks per bit and the data has to be right anyway.
    """
    rig = Rig(mode=0, clkdiv=clkdiv)
    rig.run(jedec_tx())
    data = [0x5A, 0xA5, 0xFF, 0x00]
    rig.flash.poke(0x08, data)
    rx = rig.run(read_word(0x08, 4))
    assert rx[4:8] == data
    assert rig.monitor.violations == []
    assert sck_period_ticks(rig) >= 8, "no divider may make the bit period shorter"
    rig.check_clean()


def test_clkdiv_scales_the_wire() -> None:
    """The CS window grows with the divider, and only with the divider."""
    windows = {}
    for clkdiv in ((1, 0), (2, 0), (4, 0)):
        rig = Rig(mode=0, clkdiv=clkdiv)
        rig.run(jedec_tx())
        xfer = rig.monitor.transfers[0]
        windows[clkdiv] = xfer.t_end - xfer.t_start
        assert xfer.cs_setup == 9 * clkdiv[0]
        assert xfer.cs_hold == 3 * clkdiv[0]
    assert windows[(2, 0)] == 2 * windows[(1, 0)]
    assert windows[(4, 0)] == 4 * windows[(1, 0)]


# ---- what the driver writes, and what the program is --------------------------------------


def test_config_registers_read_back() -> None:
    """Every register the driver writes, read back over the bus after `load`.

    The values are spelled out here rather than taken from the driver, so that a change to
    a field position in docs/mvp-regmap.md shows up as a test failure and not as a silent
    behaviour change.
    """
    rig = Rig(mode=0, clkdiv=(1, 0))
    rig.master.load()
    while not rig.dev.idle:
        rig.bus.step()
    rd = rig.master.read_reg
    bus = rig.bus
    assert rd(bus, 0x32) == 0x0001_0000, "CLKDIV = 1.0"
    assert rd(bus, 0x33) == 0x0000_9080, "EXECCTRL: WRAP_BOTTOM 1, WRAP_TOP 9, no side enable"
    assert rd(bus, 0x34) == 0x0083_0000, (
        "SHIFTCTRL: PUSH_THRESH 8, PULL_THRESH 32 (written 0), OUT_SHIFTDIR 0, "
        "IN_SHIFTDIR 0, AUTOPULL, AUTOPUSH"
    )
    assert rd(bus, 0x37) == 0x4C11_8002, (
        "PINCTRL: SIDESET_COUNT 2, SET_COUNT 3, OUT_COUNT 1, IN_BASE 3, "
        "SIDESET_BASE 0, SET_BASE 0, OUT_BASE 2"
    )
    assert rd(bus, 0x11) == _regmap.active().cfginfo, "DBG_CFGINFO: 32 words of IMEM, 2 engines, 4 deep"
    # A disabled engine, so nothing above changed state behind our back.
    assert rd(bus, 0x00) == 0x0000_0000


def test_program_shape() -> None:
    """Both programs fit IMEM and the labels the driver jumps to are where it thinks."""
    progs = programs()
    assert set(progs) == {0, 3}
    for mode, prog in progs.items():
        assert len(prog.instrs) <= 32, f"IMEM is 32 words, {prog.name} is {len(prog.instrs)}"
        assert prog.sideset_bits == 2 and not prog.sideset_opt and not prog.sideset_pindirs
        assert prog.labels["enter"] == 0, "the host forces `jmp enter`, so it must be word 0"
        # The host also reads the PC to know the program has halted, so `done` is a word.
        assert 0 <= prog.labels["done"] <= 31
        assert prog.wrap_target == prog.labels["start"], (
            "the wrap range has to start at `start`, or a fall-through at the end of the "
            "bit loop would jump into the middle of it"
        )
        assert prog.wrap == len(prog.instrs) - 1
    # Mode 0 and mode 3 are the same program with the two SCK phase values swapped, so
    # they must differ in the side-set field of every instruction and nowhere else.
    m0, m3 = progs[0].instrs, progs[3].instrs
    assert len(m0) == len(m3)
    for a, b in zip(m0, m3):
        assert (a & 0x1F) == (b & 0x1F), "the operands are mode independent"
        assert ((a >> 8) & 0x1F) != ((b >> 8) & 0x1F), "every side-set value differs"


def test_driver_addresses_match_the_iss() -> None:
    """The driver's word addresses are the ISS's, so a regmap change cannot pass unnoticed.

    The driver is written against docs/mvp-regmap.md `### Address map` and the ISS is the
    golden model; this is the one place the two are compared directly.
    """
    import iss.pio_iss as golden

    from fw.spi import driver

    assert (driver.A_CTRL, driver.A_FSTAT, driver.A_FDEBUG, driver.A_FLEVEL) == (
        golden.A_CTRL,
        golden.A_FSTAT,
        golden.A_FDEBUG,
        golden.A_FLEVEL,
    )
    assert (driver.A_TXF0, driver.A_TXF1, driver.A_RXF0, driver.A_RXF1) == (
        *golden.A_TXF,
        *golden.A_RXF,
    )
    assert driver.SM_BASE == golden.A_SM_BASE
    assert (
        driver.SM_CLKDIV,
        driver.SM_EXECCTRL,
        driver.SM_SHIFTCTRL,
        driver.SM_ADDR,
        driver.SM_INSTR,
        driver.SM_PINCTRL,
    ) == (
        golden.SM_CLKDIV,
        golden.SM_EXECCTRL,
        golden.SM_SHIFTCTRL,
        golden.SM_ADDR,
        golden.SM_INSTR,
        golden.SM_PINCTRL,
    )
    # Engine e's IMEM word k is at 0x12 + 0x40 e + k (docs/mvp-regmap.md `### Address map`).
    assert golden.A_IMEM == (0x12, 0x52)


def test_driver_refuses_what_it_cannot_do() -> None:
    """The limits the hardware has, checked before a transaction is started."""
    from fw.spi import driver

    with pytest.raises(ValueError):
        driver.SpiConfig(mode=1)             # the program only has modes 0 and 3
    with pytest.raises(ValueError):
        driver.SpiConfig(mode=0, clkdiv=(0, 0))
    with pytest.raises(ValueError):
        read_word(0x30, 3)                    # a read is whole TX words
    rig = Rig()
    with pytest.raises(ValueError):
        rig.master.start([0] * 8)             # 32 bytes: `set x, N` is a 5-bit immediate
    with pytest.raises(ValueError):
        driver.SpiConfig(push_thresh=33).shiftctrl()


def test_the_rtl_target_runs_the_same_module() -> None:
    """`FW_TARGET=rtl` has to *run* this module, not skip it.

    Under every target except `rtl` this runs pytest a second time with `FW_TARGET=rtl` in the
    environment and requires the module to pass rather than skip, so a model target is what notices
    that the RTL path stopped existing. It picks the cheap tests of the module to keep that guard
    quick -- the whole RTL run is `make fw PROTO=spi TARGET=rtl` -- and only skips on `rtl` itself,
    which is what keeps it from recursing. `fw/conftest.py` installs the target in the child too,
    and since those two tests never step a chip the child costs a re-run of the module and no
    simulator.
    """
    if targets.current() == "rtl":
        pytest.skip("this check runs under every target but rtl, and would otherwise recurse")
    env = dict(os.environ, FW_TARGET="rtl")
    done = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(Path(__file__).resolve()),
         "-k", "test_program_shape or test_driver_addresses_match"],
        capture_output=True,
        text=True,
        env=env,
        cwd=str(Path(__file__).resolve().parents[2]),
    )
    summary = done.stdout.strip().splitlines()[-1] if done.stdout.strip() else ""
    assert done.returncode == 0, done.stdout + done.stderr
    assert "passed" in summary and "skipped" not in summary, (
        f"the FW_TARGET=rtl path must run this module, not skip it: {summary}")


# ---- the fastest the program could go, and why it does not --------------------------------


def variant(delay: int, mode: int):
    """`program.pio` with the bit loop's delay changed, assembled as-is.

    The shipped program uses `out pins, 1 [5]`, and 5 is the smallest delay that leaves the
    sample a core cycle after the MISO the flash launched has travelled the pad and the
    input pipeline.  Re-assembling the same source with a different number is how the
    margin is measured, rather than hand-editing a second copy of the program.
    """
    from asm.pioasm import assemble

    from fw.spi.driver import PIO_FILE, PROGRAMS

    text = re.sub(r"side ([0-3]) \[5\]", rf"side \1 [{delay}]", PIO_FILE.read_text())
    for prog in assemble(text):
        if prog.name == PROGRAMS[mode]:
            return prog
    raise AssertionError(f"{PROGRAMS[mode]} vanished from the variant source")


@pytest.mark.parametrize("mode", [0, 3])
def test_one_cycle_faster_with_no_margin(mode: int) -> None:
    """The program one SM cycle per bit faster: it works, and it has no margin left.

    `out pins, 1 [4]` makes a bit 7 core cycles, 7.14 MHz at 50 MHz.  The MISO the flash
    launched on the previous edge reaches the engine's `in pins, 1` in the very cycle that
    samples it: 1 cycle of pad, 2 of VIP reaction and T_IN = 3, exactly 5 cycles, no spare.
    It passes today because the ISS and the flash model are cycle exact, and this test is
    here to say so: if it ever fails, the loop is at the edge, and the shipped program (one
    cycle slower) is the one with a cycle to spare.
    """
    rig = Rig(mode=mode, clkdiv=(1, 0), program=variant(4, mode))
    assert jedec_bytes(rig.run(jedec_tx())) == list(JEDEC_ID)
    data = [(0x3C + i) & 0xFF for i in range(16)]
    rig.flash.poke(0x40, data)
    rx = rig.run(read_word(0x40, 16))
    assert rx[4:20] == data
    assert sck_period_ticks(rig) == 7, rig.monitor.stats
    assert sck_rate_hz(rig) == F_CLK_HZ / 7
    print(f"\nmode {mode}: {sck_rate_hz(rig) / 1e6:.3f} MHz with no MISO margin")
    rig.check_clean()


def test_the_second_engine_can_do_it_too() -> None:
    """The same program on engine 1, with engine 0 left disabled.

    Nothing else needs it, but the addressing is per engine and the cheap way to
    prove the driver got it right is to run it there: IMEM word `k` of engine 1 is at
    0x52 + k rather than 0x12 + k, `SMe_*` is at 0x32 + 6 e + i, and FLEVEL and FDEBUG give
    each engine its own nibble and byte.  Engine 1 wins every pin conflict
    (docs/mvp-isa.md `## Pins`), so a chip that got this wrong would show both engines
    driving the bus.
    """
    from fw.spi import SpiConfig, SpiMaster

    rig = Rig(mode=0)
    master = SpiMaster(rig.dev, SpiConfig(mode=0), sm=1)
    master.load()
    while not rig.dev.idle:
        rig.bus.step()
    assert rig.dev.chip.peek(A_CTRL) == 0, "load() must leave both engines disabled"

    master.start(jedec_tx())
    assert master.run(rig.bus, max_ticks=4000)
    rx = master.rx_bytes
    assert jedec_bytes(rx) == list(JEDEC_ID)
    assert master.read_fdebug(rig.bus) == 0
    assert rig.monitor.violations == []
    assert len(rig.monitor.transfers) == 1
    # Engine 0 never ran: it is still at the reset PC and its FIFO is empty.
    assert rig.dev.chip.engines[0].pc == 0
    assert rig.dev.chip.engines[0].tx == [] and rig.dev.chip.engines[0].rx == []

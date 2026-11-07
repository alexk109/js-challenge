# SPDX-License-Identifier: Apache-2.0
"""Chip-level test of `tt_um_protocol_emulator`: the MVP chip on its Tiny Tapeout pins.

Every signal this file touches is a top-level port of the design -- `ui_in`, `uo_out`,
`uio_in/out/oe`, `ena`, `clk`, `rst_n` -- because TT's `gl_test` action runs this same
module against the hardened gate-level netlist, where the internal hierarchy is gone.
So the chip is driven exactly the way a real board drives it: an SPI host model on
`ui_in[2:0]` / `uo_out[0]` speaking the framing of `docs/mvp-regmap.md`, protocol pins on
the `uio`/`ui_in[7:3]`/`uo_out[7:3]` pins, and the two PIO interrupt lines on
`uo_out[2]`/`uo_out[1]`.

What runs where:

    test_spi_reset_state      the register map's reset values and the burst rules over
                              SPI, plus `rx_valid`/`tx_ready` on `uo_out[2:1]`
    test_protocol_pin_map     protocol pins 8..12 come from `ui_in[7:3]` and 13..17 go
                              out on `uo_out[7:3]`, checked through `DBG_PADOUT` and
                              through a forced `set pins`
    test_uart_tx_rx_loopback  fw/uart_tx/program.pio on engine 0, fw/uart_rx/program.pio
                              on engine 1, a wire from the TX pin to the RX pin, bytes
                              pushed over SPI and popped back over SPI, with the TX
                              waveform judged by `verif.vip.uart.UartRx`
    test_i2c_eeprom_read      fw/i2c/program.pio on engine 0 against
                              `verif.vip.i2c.I2cEepromTarget` on the `uio` pins, with
                              `I2cTimingChecker` asked for zero violations

The programs are assembled here with `asm/pioasm.py` from the `fw/*/program.pio` files,
and every register word comes from the `fw/*/driver.py` modules, which are written
against `docs/mvp-regmap.md`; this file only says *when* to write them and waits for
the answers.  The register accesses go over SPI, so one access costs tens of core
cycles instead of one, and everything that paced itself per cycle on the internal bus
(an ISS-side driver pumping a `Bus`) is done here with `await ClockCycles`.

`verif/vip/` and the `fw/` programs are used as they are: not one line of them is
edited or softened.  Waves: `tb.vcd`, dumped by `tb.v`.
"""

import inspect
import sys
from pathlib import Path

# cocotb imports this module with test/ as the working directory, so put the repo root
# on sys.path before reaching for verif.vip and fw.
_ROOT = str(Path(__file__).resolve().parents[1])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from fw.i2c import driver as i2cd
from fw.i2c import assemble_program
from fw.uart_rx import driver as rxd
from fw.uart_tx import driver as txd
from verif.vip.cocotb_bridge import Bridge
from verif.vip.core import Device
from verif.vip.i2c import I2C_MODES, I2cEepromTarget, I2cTimingChecker, ns_to_ticks
from verif.vip.spi import SpiController, SpiMonitor
from verif.vip.uart import UartRx

CLK_HZ = 50_000_000                 # info.yaml `project.clock_hz`
NS_PER_TICK = 1e9 / CLK_HZ           # 20 ns: one core cycle is one tick of verif/vip/core.py

# The host port's timing (docs/mvp-regmap.md `### Physical layer`): every one of SCK high,
# SCK low, CSn setup and CSn hold has to last at least 2 core cycles.  A 16-tick SCK
# period is 3.125 MHz, well inside the f/8 that reads are guaranteed at, so the host model
# is legal by the spec's own rules and SpiMonitor checks that it stayed that way.
SPI_HALF = 8                        # ticks per SCK half period -> 16-tick bit period
SPI_CS_SETUP = SPI_CS_HOLD = SPI_CS_GAP = 2

# Host port pins (docs/mvp-regmap.md `### Physical layer`): CSn/SCK/MOSI are ui_in[2:0]
# and MISO is uo_out[0].  The pin names are the ones verif/vip/spi.py uses.
SPI_PINS = {"cs_n": ("ui_in", 0), "sck": ("ui_in", 1),
            "mosi": ("ui_in", 2), "miso": ("uo_out", 0)}

# The two PIO interrupt lines (docs/mvp-regmap.md `### tx_ready and rx_valid`):
# rx_valid = OR of IRQ0_INTS on uo_out[2], tx_ready = OR of IRQ1_INTS on uo_out[1].
BIT_RX_VALID, BIT_TX_READY = 2, 1

TX_LIMIT = 40_000                   # ticks one host transaction may take before we give up
READY_LIMIT = 60_000                # ticks a wait for a line or a FIFO may take
PHASE_LIMIT = 60_000                # ticks one I2C phase may take without making progress
WAIT_STEP = 8                       # core cycles between two looks at an interrupt line

# Register word addresses, from the firmware drivers (which follow docs/mvp-regmap.md)
# rather than from a second copy here.  `check_addresses` below proves the two driver
# modules and this list agree, so a driver that moved a register cannot pass silently.
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = txd.A_CTRL, txd.A_FSTAT, txd.A_FDEBUG, txd.A_FLEVEL
A_TXF, A_RXF = txd.A_TXF, txd.A_RXF
A_PADOUT, A_PADOE, A_CFGINFO = txd.A_DBG_PADOUT, txd.A_DBG_PADOE, txd.A_DBG_CFGINFO
A_IMEM, A_SM = txd.A_INSTR_MEM, txd.A_SM_BASE
(SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL) = txd.SM_CLKDIV, \
    txd.SM_EXECCTRL, txd.SM_SHIFTCTRL, txd.SM_ADDR, txd.SM_INSTR, txd.SM_PINCTRL

# FSTAT field masks, engine e's bit is the mask shifted left by e.
FSTAT_TXEMPTY, FSTAT_TXFULL = txd.FSTAT_TXEMPTY, txd.FSTAT_TXFULL
# FDEBUG (docs/mvp-regmap.md `**FDEBUG**`): TXSTALL at 25:24 and RXSTALL at 1:0 only say
# the engine waited for the host, which over SPI it does between phases, because one
# register access costs tens of core cycles instead of one.  TXOVER at 17:16 and RXUNDER
# at 9:8 say a word was dropped or popped out of an empty FIFO, and this host never does
# either: it gates every FIFO access on FLEVEL.
FDEBUG_LOST = 0x0003_0300


def sm_addr(engine: int, index: int) -> int:
    """Word address of `SMe_<register>` (docs/mvp-regmap.md `### Per-engine registers`)."""
    return A_SM + 6 * engine + index


def imem_addr(engine: int, word: int) -> int:
    """Word address of `INSTR_MEM<word>` of `engine`."""
    return A_IMEM[engine] + word


def check_addresses() -> None:
    """The three firmware drivers must agree on the register map (docs/mvp-regmap.md)."""
    assert (A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL) == (i2cd.A_CTRL, i2cd.A_FSTAT,
                                                      i2cd.A_FDEBUG, i2cd.A_FLEVEL)
    assert (A_TXF[0], A_RXF[0], A_IMEM[0], A_SM) == (i2cd.A_TXF0, i2cd.A_RXF0,
                                                     i2cd.A_IMEM[0], i2cd.A_SM)
    assert (imem_addr(1, 0), sm_addr(1, 0)) == (rxd.imem_addr(1, 0), rxd.sm_addr(1, "CLKDIV"))


check_addresses()


# ---------------------------------------------------------------------------------------
# Devices the test itself adds to the bus
# ---------------------------------------------------------------------------------------


class Wire(Device):
    """A wire from one net to another, which is how the testbench shorts two pins.

    `verif.vip.core.Bus` gives every driver one tick of latency -- a value returned for a
    net at tick `t` is on that net from the end of `t`, and every other device reads it at
    `t + 1` -- so this is a wire one core cycle long: 20 ns against a 8.68 us bit time at
    115200 Bd, which is all the loopback's two pins have in common.
    """

    name = "wire"

    def __init__(self, src: str, dst: str, name: str = "wire"):
        self.src, self.dst, self.name = src, dst, name

    def tick(self, t: int, pins) -> dict:
        return {self.dst: pins.get(self.src, 1)}


# ---------------------------------------------------------------------------------------
# The host model: docs/mvp-regmap.md `## SPI transaction format`, on the TT top-level pins
# ---------------------------------------------------------------------------------------


def cmd_byte(addr: int, read: bool = False) -> int:
    """`CMD = { RW, ADDR[6:0] }`, MSB first (docs/mvp-regmap.md `### Framing`)."""
    return (0x80 if read else 0x00) | (int(addr) & 0x7F)


def word_bytes(word: int) -> list:
    """One 32-bit data word as four big-endian bytes, the order the framing uses."""
    word = int(word) & 0xFFFF_FFFF
    return [(word >> 24) & 0xFF, (word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF]


class SpiHost:
    """The chip's host port, over the Bridge: register words in, register words out.

    One instance per test.  It owns

      * the `Bridge`, one tick per rising edge of `clk`, so every `verif.vip` device on
        the chip's pins sees the chip's own pins with a one-tick delay, exactly as
        `verif/vip/cocotb_bridge.py` documents;
      * an `SpiController` (mode 0, 16-tick SCK period) that drives CSn, SCK and MOSI and
        samples MISO, queueing one transaction per host access;
      * an `SpiMonitor` on the same four nets, which checks that this host model obeyed
        the spec's timing rules and decodes what actually went over the wire.  It drives
        nothing, so it sees the chip's MISO the way a real analyser would.

    `pins` and `devices` add the protocol-pin nets of a test (and the models on them) to
    the same Bridge, so one Bus runs the host port and the protocol under test together.
    """

    def __init__(self, dut, pins=None, devices=()):
        self.dut = dut
        mapping = dict(SPI_PINS)
        mapping.update(pins or {})
        self.bridge = Bridge(dut, mapping, ns_per_tick=NS_PER_TICK)
        for net in ("cs_n", "sck", "mosi", "miso"):
            self.bridge.set_net(net, push_pull=True)
        self.spi = self.bridge.attach(SpiController(
            mode=0, half_period=SPI_HALF, cs_setup=SPI_CS_SETUP, cs_hold=SPI_CS_HOLD,
            cs_gap=SPI_CS_GAP, fill=0x00))
        self.monitor = self.bridge.attach(SpiMonitor(
            mode=0, min_sck_ticks=SPI_CS_HOLD, min_cs_setup=SPI_CS_SETUP,
            min_cs_hold=SPI_CS_HOLD))
        self.devices = [self.bridge.attach(d) for d in devices]
        self._tasks = []
        #: Every register access this host made, as ("w"/"r", addr, data), for the log.
        self.accesses = []

    # -- lifecycle ---------------------------------------------------------------

    async def start(self) -> None:
        """Reset the chip the way TT does: rst_n low, then high, with the clock running."""
        dut = self.dut
        dut.ena.value = 1
        dut.ui_in.value = 0x00          # CSn high: the host port is idle out of reset
        dut.uio_in.value = 0x00
        self._tasks.append(cocotb.start_soon(Clock(dut.clk, NS_PER_TICK, unit="ns").start()))
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 8)
        self._tasks.append(cocotb.start_soon(self.bridge.start()))
        dut.rst_n.value = 1
        # Two of those cycles are the top level's reset synchronizer releasing, so the
        # chip proper sees rst_n go high before anything is asked of it.
        await ClockCycles(dut.clk, 8)

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        self._tasks = []

    async def __aenter__(self) -> "SpiHost":
        await self.start()
        return self

    async def __aexit__(self, *exc) -> None:
        await self.stop()

    # -- the chip's own output pins ------------------------------------------------

    def uo_out(self) -> int:
        return int(self.dut.uo_out.value)

    def rx_valid(self) -> int:
        """`uo_out[2]`, the OR of IRQ0_INTS (docs/mvp-regmap.md `### tx_ready and rx_valid`)."""
        return (self.uo_out() >> BIT_RX_VALID) & 1

    def tx_ready(self) -> int:
        """`uo_out[1]`, the OR of IRQ1_INTS."""
        return (self.uo_out() >> BIT_TX_READY) & 1

    # -- waiting ------------------------------------------------------------------

    async def wait_ticks(self, ticks: int) -> None:
        """Let the core clock run: the firmware keeps working while the host is idle."""
        await ClockCycles(self.dut.clk, int(ticks))

    async def _idle(self) -> None:
        """Wait until the controller has driven everything that was queued."""
        waited = 0
        while not self.spi.idle():
            await ClockCycles(self.dut.clk, 1)
            waited += 1
            if waited > TX_LIMIT:
                raise AssertionError(
                    f"the SPI controller never went idle in {TX_LIMIT} ticks "
                    f"({len(self.spi.queue)} transactions still queued)"
                )

    async def wait_for(self, what: str, cond, limit: int = READY_LIMIT, step: int = 1) -> None:
        """Step the clock until `cond()` holds, or fail saying what was being waited for.

        `cond` may be a coroutine function -- it can then read registers, which costs a
        transaction -- or a plain one over signals already on the pins.  `step` is how
        many core cycles pass between two looks: every wait in this test is for something
        that changes on the order of a hundred cycles, so looking every 8 costs nothing in
        latency and saves the simulator four of every five Python round trips.  The limit
        counts bus ticks, not looks, so a wait that spends its time in transactions still
        times out when it should.
        """
        start = self.bridge.bus.t
        while True:
            value = cond()
            if inspect.isawaitable(value):
                value = await value
            if value:
                return
            await ClockCycles(self.dut.clk, step)
            waited = self.bridge.bus.t - start
            if waited > limit:
                raise AssertionError(f"{what}: still waiting after {waited} core cycles")

    # -- register access ----------------------------------------------------------

    async def write(self, addr: int, *words: int) -> None:
        """One burst write: `CMD | W0 | W1 | ...` in one CSn low window.

        The address steps by one per word and wraps mod 128, except in the FIFO data
        registers 0x04..0x0B where it stays put (docs/mvp-regmap.md `### Bursts`), so a
        burst from 0x04 streams into TXFn and a burst from 0x12 loads IMEM.
        """
        mosi = [cmd_byte(addr)] + [b for word in words for b in word_bytes(word)]
        self.spi.add(mosi, 0)
        await self._idle()
        for word in words:
            self.accesses.append(("w", addr, word & 0xFFFF_FFFF))

    async def read(self, addr: int, count: int = 1) -> list:
        """One burst read: `CMD | TA | R0 | ... `, `count` words.

        The controller clocks `fill` on MOSI after the command, which is what the spec's
        `xx` bytes are, and samples 5*count MISO bytes: one turnaround byte per word, then
        the word.  Only the data words are returned.
        """
        before = len(self.spi.rx)
        self.spi.add([cmd_byte(addr, read=True)], 5 * count)
        await self._idle()
        raw = self.spi.rx[before:]
        assert len(raw) == 5 * count, f"read of {count} word(s) from {addr:#04x} sampled {raw}"
        words = []
        for i in range(count):
            word = int.from_bytes(bytes(raw[5 * i + 1: 5 * i + 5]), "big")
            words.append(word)
            self.accesses.append(("r", addr, word))
        return words

    async def read1(self, addr: int) -> int:
        """One register read."""
        return (await self.read(addr, 1))[0]

    # -- what the host model itself did --------------------------------------------

    def check(self) -> None:
        """The host model was legal, and every access went over the wire as one window.

        SpiMonitor is a passive observer on the same four nets, so an empty `violations`
        is the spec's own timing rules checked against this test's stimulus rather than
        against itself, and one monitored CSn window per queued transaction is the framing
        of docs/mvp-regmap.md `### Framing` counted from the wire.
        """
        assert not self.monitor.violations, self.monitor.violations
        assert self.monitor.stats["transactions"] == len(self.spi.transfers), (
            f"{self.monitor.stats['transactions']} CSn windows for "
            f"{len(self.spi.transfers)} transactions"
        )
        assert not self.bridge.contentions, self.bridge.contentions

    def log_stats(self) -> None:
        """One line of numbers for the acceptance log."""
        cocotb.log.info(
            "top: %d core cycles, %d host transactions (%d SPI windows), "
            "%d bits on the wire, sck low/high min %s/%s ticks, cs setup/hold min %s/%s",
            self.bridge.bus.t, len(self.spi.transfers), self.monitor.stats["transactions"],
            self.monitor.stats["bits"], self.monitor.stats["sck_low_min"],
            self.monitor.stats["sck_high_min"], self.monitor.stats["cs_setup_min"],
            self.monitor.stats["cs_hold_min"],
        )


# ---------------------------------------------------------------------------------------
# Loading an engine, in the order of docs/mvp-regmap.md `### Typical host sequences`
# ---------------------------------------------------------------------------------------


def ctrl_bits(engine: int, *fields: int) -> int:
    """The CTRL bits named, each moved up to this engine's bit (fw/uart_tx/driver.py)."""
    mask = 0
    for field in fields:
        mask |= field << engine
    return mask


async def load_engine(host: SpiHost, engine: int, program, clkdiv: int, execctrl: int,
                      shiftctrl: int, pinctrl: int, forced=(), start: bool = True) -> None:
    """Steps 1-6 of the host sequence for one engine, over SPI.

    Every word is the one the matching `fw/*/driver.py` builds for the same program, so
    this only orders and times them: the program into IMEM as one burst, the four
    configuration registers, then the forced `SMx_INSTR` instructions the driver pushes
    (the initial pin state and `jmp <entry>`), then `CTRL` with `SM_ENABLE`, `SM_RESTART`
    and `CLKDIV_RESTART` in the same write so the engine's first SM cycle is two core
    cycles later.  CTRL is read-modify-written, so the other engine keeps running.
    `start=False` stops after step 4, for a host that wants to fill the TX FIFO first.
    """
    e = ctrl_bits(engine, txd.CTRL_SM_ENABLE, txd.CTRL_SM_RESTART, txd.CTRL_CLKDIV_RESTART)
    ctrl = await host.read1(A_CTRL)
    await host.write(A_CTRL, ctrl & ~e)                          # 1: the engine is stopped
    origin = program.origin or 0
    await host.write(imem_addr(engine, origin), *program.instrs)   # 2: the program
    await host.write(sm_addr(engine, SM_CLKDIV), clkdiv, execctrl,
                     shiftctrl)                                # 3: CLKDIV/EXECCTRL/SHIFTCTRL
    await host.write(sm_addr(engine, SM_PINCTRL), pinctrl)      # ... and PINCTRL, which
                                                                 #     is not next to them
    for word in forced:                                         # 4: forced pin state and jmp
        await host.write(sm_addr(engine, SM_INSTR), word)
    if not start:
        return
    ctrl = await host.read1(A_CTRL)
    await host.write(A_CTRL, ctrl | e)                          # 5: start it


# ---------------------------------------------------------------------------------------
# 1. The register map over SPI, out of reset
# ---------------------------------------------------------------------------------------


@cocotb.test()
async def test_spi_reset_state(dut):
    """Reset values, the burst rules and the two interrupt lines, all over the SPI port."""
    host = SpiHost(dut)
    await host.start()
    try:
        # docs/mvp-regmap.md `### Address map`: DBG_CFGINFO is a constant the host can
        # read as a sanity check, FSTAT resets to all-empty and DBG_PADOE says which of
        # the 18 pins this chip can drive.
        assert await host.read1(A_CFGINFO) == 0x0020_0204
        assert await host.read1(A_FSTAT) == 0x0300_0300
        assert await host.read1(A_PADOE) == 0x0003_E000     # pins 17..13 driven, no uio pin
        assert await host.read1(A_PADOUT) == 0x0000_0000
        assert await host.read1(A_FLEVEL) == 0x0000_0000
        # IRQ0_INTE resets to 0x003 and IRQ1_INTE to 0x010, so out of reset rx_valid is
        # low (no RX FIFO holds anything) and tx_ready is high (engine 0's TX FIFO has
        # room).  These are the two lines docs/mvp-regmap.md `### tx_ready and rx_valid`
        # puts on uo_out[2] and uo_out[1].
        assert (host.rx_valid(), host.tx_ready()) == (0, 1), (host.rx_valid(), host.tx_ready())

        # `### Bursts`: the address stays put in the FIFO data registers, so one write
        # burst from TXF0 streams four words into engine 0's TX FIFO (four deep).
        await host.write(A_TXF[0], 0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444)
        fstat = await host.read1(A_FSTAT)
        assert fstat & 0x0100_0000 == 0, f"engine 0 TX FIFO should not be empty: {fstat:#x}"
        assert fstat & FSTAT_TXFULL, f"engine 0 TX FIFO should be full: {fstat:#x}"
        assert fstat & (FSTAT_TXEMPTY << 1), f"engine 1 TX FIFO should be empty: {fstat:#x}"
        assert await host.read1(A_FLEVEL) == 0x0000_0004, "TX0 occupancy is not 4"
        # SM0_TXNFULL has fallen out of INTR, so IRQ1_INTS is 0 and tx_ready follows,
        # two core cycles later than the FIFO change.
        await host.wait_for("tx_ready to follow the full TX FIFO", lambda: host.tx_ready() == 0)

        # Reads of unmapped and reserved addresses return 0 and ignore writes
        # (`### Address map`), and a write burst out of the FIFO range steps by one word.
        await host.write(0x3E, 0xFFFF_FFFF)
        assert await host.read1(0x3E) == 0
        assert await host.read1(0x00) == 0        # CTRL: no SM_RESTART/CLKDIV_RESTART latched
        host.check()
    finally:
        await host.stop()
    host.log_stats()


# ---------------------------------------------------------------------------------------
# 2. The 18 protocol pins on the Tiny Tapeout pins
# ---------------------------------------------------------------------------------------

# Protocol pins 13..17 are the five output-only ones, on uo_out[7:3].  `set pins` writes
# the SET window's level register, which for an output-only pin is also what the pin
# drives, so this is the cheapest way to put a known pattern on uo_out[7:3].
OUT_ONLY_BASE, OUT_ONLY_COUNT = 13, 5

# Protocol pins 8..12 are the five input-only ones, on ui_in[7:3].
IN_ONLY_BASE, IN_ONLY_COUNT = 8, 5

# `set pins, <5 bits>`: opcode 111, destination PINS, data in bits 4:0.
SET_PINS_WORD = 0xE000
# `set pindirs, <level>`: the same opcode with destination PINDIRS (bits 7:5 = 100).
SET_PINDIRS_WORD = 0xE080
# `wait 0 pin, 0`: opcode 001 (WAIT), Pol = 0, Source = 01 (PIN, so the index is an offset
# from IN_BASE), index 0.  Checked against the assembler: `python -m asm.pioasm` on that
# one line prints 0x2020.  A forced one stalls in the engine's EXEC latch for as long as
# the pin is high, which is what EXECCTRL.EXEC_STALLED (bit 31) reports -- the only way a
# host can see that an input-only pin changed, since its level register reads 0.
WAIT_LOW_PIN_WORD = 0x2020
EXEC_STALLED = 0x8000_0000


@cocotb.test()
async def test_protocol_pin_map(dut):
    """`ui_in[7:3]` reaches protocol pins 8..12, and 13..17 come out on `uo_out[7:3]`.

    Both directions are checked the way a host sees them: the output side through the pad
    itself (`uo_out[7:3]` is wired to pins 17..13) and through `DBG_PADOUT`, and the input
    side through a forced `wait 0 pin` whose stall is visible in `EXEC_STALLED`, one pin
    at a time.  Only forced instructions run: the engines stay disabled, so nothing here
    is about the firmware.
    """
    host = SpiHost(dut)
    await host.start()
    try:
        # A forced `set pins` runs in the next core cycle whatever SM_ENABLE says, and it
        # executes with the divider stopped, so one host access is enough.
        pinctrl = ((OUT_ONLY_COUNT << 26) | (OUT_ONLY_BASE << 5))   # SET window only
        await host.write(sm_addr(0, SM_PINCTRL), pinctrl)
        for pattern in (0b10101, 0b00000, 0b11111, 0b01010):
            await host.write(sm_addr(0, SM_INSTR), SET_PINS_WORD | pattern)
            # The pin is registered, so the pin value reaches the top-level pad in the
            # cycle after the forced instruction wrote the level register.
            await host.wait_ticks(4)
            assert (host.uo_out() >> 3) & 0x1F == pattern, (
                f"uo_out[7:3] = {(host.uo_out() >> 3) & 0x1F:#07b}, wanted {pattern:#07b}")
            assert await host.read1(A_PADOUT) >> OUT_ONLY_BASE & 0x1F == pattern, (
                f"DBG_PADOUT[17:13] does not show {pattern:#07b}")

        # The input-only pins.  Their level registers read 0 whatever the pin says
        # (docs/mvp-regmap.md `**DBG_PADOUT**`), so the input path is proved by what the
        # engine does with it: a forced `wait 0 pin, 0` on IN_BASE = k stalls exactly
        # while ui_in[k - 5] is high, and EXEC_STALLED says so.
        #
        # The engine runs for this.  docs/mvp-isa.md `### EXEC` says a stalled host-forced
        # instruction stays in the latch and is "retried on the engine's normal SM cycles",
        # so a disabled engine would keep EXEC_STALLED asserted for ever; running the
        # engine (CLKDIV is its reset value of 1, and IMEM word 0 is `jmp 0`) is what lets the
        # stall clear when the pin falls.  Nothing else here is about the firmware.  Word 0 is
        # written, not assumed: the flop IMEM resets to zero, but the SRAM IMEM
        # (SRAM_IMEM=1) is undefined until the host writes it, which gate level shows as X.
        await host.write(imem_addr(0, 0), 0x0000)
        ctrl = await host.read1(A_CTRL)
        await host.write(A_CTRL, ctrl | ctrl_bits(0, txd.CTRL_SM_ENABLE))
        for pin in range(IN_ONLY_BASE, IN_ONLY_BASE + IN_ONLY_COUNT):
            bit = pin - IN_ONLY_BASE + 3          # pin 8 is ui_in[3]
            # The wait is forced while the pin is already high, so it stalls at once.
            dut.ui_in.value = (int(dut.ui_in.value) & 0b0000_0111) | (1 << bit)
            await host.write(sm_addr(0, SM_PINCTRL), pin << 15)   # IN_BASE = pin
            await host.write(sm_addr(0, SM_INSTR), WAIT_LOW_PIN_WORD)
            await host.wait_ticks(12)
            execctrl = await host.read1(sm_addr(0, SM_EXECCTRL))
            assert execctrl & EXEC_STALLED, (
                f"a forced `wait 0 pin` on pin {pin} did not see ui_in[{bit}] high")
            dut.ui_in.value = int(dut.ui_in.value) & ~(1 << bit) & 0xFF
            await host.wait_ticks(12)
            execctrl = await host.read1(sm_addr(0, SM_EXECCTRL))
            assert not execctrl & EXEC_STALLED, (
                f"a forced `wait 0 pin` on pin {pin} stayed stalled after ui_in[{bit}] fell")
        ctrl = await host.read1(A_CTRL)
        await host.write(A_CTRL, ctrl & ~ctrl_bits(0, txd.CTRL_SM_ENABLE))

        # Put the output-only pins back to 0, so DBG_PADOUT is all zero: the level
        # registers of the five input-only pins and of every uio pin this test did not
        # drive read 0, because an input has no level register of its own.
        pinctrl = (OUT_ONLY_COUNT << 26) | (OUT_ONLY_BASE << 5)
        await host.write(sm_addr(0, SM_PINCTRL), pinctrl)
        await host.write(sm_addr(0, SM_INSTR), SET_PINS_WORD)
        await host.wait_ticks(4)
        assert (host.uo_out() >> 3) & 0x1F == 0
        padout = await host.read1(A_PADOUT)
        assert padout == 0, f"DBG_PADOUT = {padout:#010x}"

        # `set pindirs` on a uio pin is what reaches `uio_oe` (pin 0 here), and the
        # output-only pins cannot be turned into inputs: `DBG_PADOE` bits 17..13 read 1.
        await host.write(sm_addr(0, SM_PINCTRL), (1 << 26) | (0 << 5))  # SET window: pin 0
        for level in (1, 0):
            await host.write(sm_addr(0, SM_INSTR), SET_PINDIRS_WORD | level)  # set pindirs
            await host.wait_ticks(4)
            assert int(dut.uio_oe.value) & 1 == level, f"uio_oe = {int(dut.uio_oe.value):#04b}"
            assert await host.read1(A_PADOE) & 0xFF == level
        assert await host.read1(A_PADOE) & 0x0003_E000 == 0x0003_E000
        host.check()
    finally:
        await host.stop()
    host.log_stats()


# ---------------------------------------------------------------------------------------
# 3. UART: fw/uart_tx on engine 0, fw/uart_rx on engine 1, one wire between two uio pins
# ---------------------------------------------------------------------------------------

# Protocol pins: engine 0 drives uio[0] and engine 1 watches uio[1].  They have to be two
# pins and not one, because the pin a PINDIRS write names is driven or released by
# direction: a receiver has to leave its pin alone, and the Bridge only samples a uio pin
# as an output while uio_oe says so.
UART_TX_PIN, UART_RX_PIN = 0, 1
UART_BAUD = 115200
UART_TICKS_PER_BIT = CLK_HZ / UART_BAUD          # 434.03, the divider's job is to hit it
# The receiver program spends S SM cycles per bit time and the host divides the clock so
# one bit time is exactly S SM cycles (fw/uart_rx/program.pio's header), so S and the baud
# fix the receiver's CLKDIV.
UART_RX_SM_CYCLES_PER_BIT = rxd.DEFAULT_SM_CYCLES_PER_BIT
UART_PAYLOAD = bytes([0x48, 0x69, 0x21, 0x0D, 0x0A])      # "Hi!\r\n"

# A data word carries the byte at bits 31:24; a word with any of bits 23:0 set is the
# marker that flags a framing error on the byte before it (fw/uart_rx/driver.py).
RX_WORD_MARKER = 0x00FF_FFFF


async def push_tx(host: SpiHost, byte: int) -> None:
    """Push one byte into engine 0's TX FIFO, waiting for `tx_ready` first.

    `tx_ready` is IRQ1, which out of reset is `SM0_TXNFULL` and nothing else
    (docs/mvp-regmap.md `### tx_ready and rx_valid`), so it is this engine's "the FIFO has
    room" line.  Waiting for it is what keeps FDEBUG.TXOVER clear.
    """
    await host.wait_for(f"room in the TX FIFO for {byte:#04x}",
                        lambda: host.tx_ready() == 1, step=WAIT_STEP)
    await host.write(A_TXF[0], byte)


async def pop_rx(host: SpiHost, engine: int, count: int) -> list:
    """Pop `count` words out of `engine`'s RX FIFO, waiting on `rx_valid` first.

    `rx_valid` is IRQ0, which out of reset is the OR of both engines' `RXNEMPTY`, so it
    says that some FIFO has something in it; `FLEVEL` then says which and how much, and
    only a non-zero level is followed by a read of `RXFn` (reading an empty FIFO returns 0
    and sets FDEBUG.RXUNDER, docs/mvp-regmap.md `**RXF0, RXF1**`).
    """
    shift = 8 * engine + 4                          # FLEVEL: RX1 at 15:12, RX0 at 7:4
    words = []
    while len(words) < count:
        await host.wait_for(f"{count} words out of RXF{engine}",
                            lambda: host.rx_valid() == 1, step=WAIT_STEP)
        while len(words) < count:
            level = (await host.read1(A_FLEVEL)) >> shift & 0xF
            if not level:
                break
            words.append(await host.read1(A_RXF[engine]))
    return words


@cocotb.test()
async def test_uart_tx_rx_loopback(dut):
    """Bytes into engine 0's TX FIFO over SPI, out of engine 1's RX FIFO over SPI.

    `fw/uart_tx/program.pio` runs on engine 0 with the register values
    `fw/uart_tx/driver.py` builds for it, `fw/uart_rx/program.pio` runs on engine 1 with the
    ones `fw/uart_rx/driver.py` builds, and the `Wire` in the testbench is the one-wire
    loopback from the TX pin to the RX pin.  Three independent judges, none of them the
    chip: `UartRx` watches the TX waveform, the bytes engine 1 pushed come back over SPI
    and are decoded, and `FDEBUG` says whether a word was ever dropped.
    """
    monitor = UartRx("tx", ticks_per_bit=UART_TICKS_PER_BIT)
    host = SpiHost(dut, pins={"tx": ("uio", UART_TX_PIN), "rx": ("uio", UART_RX_PIN)},
                   devices=[monitor, Wire("tx", "rx")])
    await host.start()
    try:
        # ---- engine 1: the receiver, started first so it is hunting before the first
        # ---- start bit goes out
        rx_prog = rxd.build_program(UART_RX_SM_CYCLES_PER_BIT)
        await load_engine(
            host, 1, rx_prog,
            clkdiv=rxd.clkdiv(UART_TICKS_PER_BIT / UART_RX_SM_CYCLES_PER_BIT),
            execctrl=rxd.execctrl(wrap_bottom=rx_prog.wrap_target, wrap_top=rx_prog.wrap,
                                  jmp_pin=UART_RX_PIN),
            shiftctrl=rxd.shiftctrl(in_right=1, out_right=1),
            pinctrl=rxd.pinctrl(in_base=UART_RX_PIN, set_base=UART_RX_PIN, set_count=1),
            # The receiver's two forced instructions, as fw/uart_rx/driver.py writes them:
            # release the RX pin, then jump to `hunt`.
            forced=[SET_PINDIRS_WORD, rx_prog.labels["hunt"]])

        # ---- engine 0: the transmitter, armed and preloaded before it is enabled
        tx_prog = txd.load_program()
        await load_engine(
            host, 0, tx_prog,
            clkdiv=txd.clkdiv_for_baud(UART_BAUD, txd.bit_cycles(tx_prog)),
            execctrl=txd.execctrl_value(tx_prog),
            shiftctrl=txd.shiftctrl_value(),
            pinctrl=txd.pinctrl_value(UART_TX_PIN,
                                      sideset_count=tx_prog.sideset_bits),
            forced=[txd.jmp_abs(tx_prog.labels["start"])],
            start=False)
        # The first `pull block` of the program would stall on an empty FIFO (and set
        # FDEBUG.TXSTALL), and the forced JMP above has already run the background
        # autopull, so the first bytes go in before the engine starts -- four of them,
        # which is all a stopped engine's FIFO holds.
        for byte in UART_PAYLOAD[:4]:
            await push_tx(host, byte)
        ctrl = await host.read1(A_CTRL)
        await host.write(A_CTRL, ctrl | ctrl_bits(0, txd.CTRL_SM_ENABLE,
                                                  txd.CTRL_SM_RESTART,
                                                  txd.CTRL_CLKDIV_RESTART))
        # The rest goes in as the engine drains, gated on tx_ready so nothing is dropped.
        for byte in UART_PAYLOAD[4:]:
            await push_tx(host, byte)

        words = await pop_rx(host, 1, len(UART_PAYLOAD))

        # ---- judge 1: the bytes engine 1 pushed, read back out over SPI
        assert [w >> 24 & 0xFF for w in words] == list(UART_PAYLOAD), (
            f"RXF1 gave {[hex(w >> 24 & 0xFF) for w in words]}, "
            f"wanted {[hex(b) for b in UART_PAYLOAD]}")
        assert [w & RX_WORD_MARKER != 0 for w in words] == [False] * len(UART_PAYLOAD), (
            "the receiver reported a framing error")
        # ---- judge 2: the TX waveform, decoded by verif/vip/uart
        assert monitor.values == list(UART_PAYLOAD), (
            f"UartRx decoded {monitor.values}, wanted {list(UART_PAYLOAD)}")
        assert monitor.framing_errors == 0, monitor.framing_errors
        assert monitor.aborted == 0, monitor.aborted
        assert monitor.worst_dev_ticks < 2.0, (
            f"the TX bit edges are {monitor.worst_dev_ticks} ticks off the "
            f"{UART_TICKS_PER_BIT:.2f}-tick grid")
        # ---- judge 3: nothing was lost on the way
        fdebug = await host.read1(A_FDEBUG)
        assert fdebug & FDEBUG_LOST == 0, f"FDEBUG = {fdebug:#010x}"
        # Every frame starts 10 bit times plus the loop's own two SM cycles after the one
        # before it: `jmp start` and `pull block` carry no delay (fw/uart_tx/program.pio),
        # so the line idles high for two SM cycles between a stop bit and the next start
        # bit.  Within a frame the bit times are the nominal ones, which is what
        # `worst_dev_ticks` above measured against the grid.
        sm_cycle = UART_TICKS_PER_BIT / txd.bit_cycles(tx_prog)
        spacing = 10 * UART_TICKS_PER_BIT + 2 * sm_cycle
        starts = [b.tick for b in monitor.received]
        for a, b in zip(starts, starts[1:]):
            assert abs(b - a - spacing) < 2, (
                f"frame spacing {b - a} ticks, expected {spacing:.1f}")
        host.check()
    finally:
        await host.stop()
    host.log_stats()


# ---------------------------------------------------------------------------------------
# 4. I2C: fw/i2c on engine 0 against an EEPROM on the uio pins
# ---------------------------------------------------------------------------------------

I2C_SDA_PIN, I2C_SCL_PIN = 0, 1          # both have to be uio pins: open drain is PINDIRS
I2C_MODE = "fast"
I2C_CLK_DIV = 14                        # 9 SM cycles per bit at 14 core cycles each
I2C_EEPROM_ADDRESS = 0x50
I2C_STRETCH_TICKS = 12                  # the EEPROM holds SCL low this long after an ACK


class NoDeviceQueue:
    """Stands in for the `iss.pio_iss.ChipDevice` an `I2cDriver` would queue accesses on.

    `I2cFirmware` below never uses the driver to *perform* an access -- the SPI host does
    that -- only the register words it builds from the program and the configuration, so
    the queue is never touched.  Refusing loudly if it ever is keeps that honest.
    """

    def write(self, addr: int, data: int):
        raise AssertionError(f"I2cFirmware must go through the SPI host, not {addr:#04x}")

    def read(self, addr: int):
        raise AssertionError(f"I2cFirmware must go through the SPI host, not {addr:#04x}")


class I2cFirmware:
    """The host half of `fw/i2c`, over the SPI port instead of the internal bus.

    `fw/i2c/driver.py`'s `I2cDriver` writes its accesses to an `iss.pio_iss.ChipDevice`
    queue and pumps a `verif.vip.core.Bus` itself, one access per core cycle; that cannot
    run inside a cocotb test, where the clock is the simulator and every wait is an
    `await`.  So this is the same host protocol with awaits: every register word, every
    forced instruction and every FIFO gate comes from `fw/i2c/driver.py` (an `I2cDriver`
    is built for its word properties and labels, and the module functions build the words
    that go into the FIFOs), and only the stepping is different.
    """

    def __init__(self, host: SpiHost, clkdiv: int = I2C_CLK_DIV, sda_pin: int = I2C_SDA_PIN,
                 scl_pin: int = I2C_SCL_PIN, bus_free_ticks: int = 70):
        self.host = host
        self.bus_free_ticks = bus_free_ticks
        self.driver = i2cd.I2cDriver(
            bus=host.bridge.bus, device=NoDeviceQueue(), program=assemble_program(),
            clkdiv=clkdiv, sda_pin=sda_pin, scl_pin=scl_pin, bus_free_ticks=bus_free_ticks)
        self.prog = self.driver.prog          # retargeted onto `scl_pin` by the driver
        self.peak_tx = self.peak_rx = 0
        self._tick_down = PHASE_LIMIT
        init = self.prog.labels["init"]
        # The three places the engine can sit with the bus idle: the two-word preamble of
        # the wrap region and the one-word spin at the end of a phase.
        self.parked = frozenset((init, init + 1, self.prog.labels["idle"]))

    # -- the forced instructions --------------------------------------------------

    def forced(self, word: int, side: int = i2cd.SIDE_SCL_LOW) -> int:
        """One instruction with `side` in its delay/side-set field, as `I2cDriver.force`."""
        return (word & ~(0x1F << 8)) | (i2cd.side_field(self.prog, side) << 8)

    async def push_forced(self, word: int, side: int = i2cd.SIDE_SCL_LOW) -> None:
        """Force one instruction into `SM0_INSTR` and let it execute.

        A host-forced instruction runs in the next core cycle whatever SM_ENABLE and the
        divider say (docs/mvp-isa.md `### EXEC`), so the engine has left the parked spin
        by the time this transaction over SPI has ended.
        """
        await self.host.write(sm_addr(0, SM_INSTR), self.forced(word, side))

    async def pc(self) -> int:
        return await self.host.read1(sm_addr(0, SM_ADDR)) & 0x1F

    async def wait_parked(self) -> None:
        await self.host.wait_for("the engine to park", self.parked_pc, step=WAIT_STEP)

    async def parked_pc(self) -> bool:
        return await self.pc() in self.parked

    async def wait_left_parked(self) -> None:
        """Wait until a forced jump has landed: `SM0_ADDR` still shows the old PC right
        after the write that forced it (docs/mvp-isa.md `### EXEC`)."""
        await self.host.wait_for("the forced jump to land", self.left_parked, step=WAIT_STEP)

    async def left_parked(self) -> bool:
        return await self.pc() not in self.parked

    # -- the FIFOs ---------------------------------------------------------------

    async def tx_level(self) -> int:
        level = await self.host.read1(A_FLEVEL) & 0xF
        self.peak_tx = max(self.peak_tx, level)
        return level

    async def rx_level(self) -> int:
        level = await self.host.read1(A_FLEVEL) >> 4 & 0xF
        self.peak_rx = max(self.peak_rx, level)
        return level

    async def load(self) -> None:
        """`I2cDriver.load`: the program, the four registers, one forced instruction, CTRL.

        The forced instruction is the program's own `init`, which releases both lines, so
        the engine then sits in the wrap region with an idle bus.  The first forced
        instruction of the first phase pulls SCL low, which ends the high period the bus
        has had since reset, and `I2cTimingChecker` measures that first period from tick 0
        on purpose -- so the bus is left free for tBUF before anything is sent.
        """
        await load_engine(
            self.host, 0, self.prog,
            clkdiv=self.driver.clkdiv_word, execctrl=self.driver.execctrl_word,
            shiftctrl=self.driver.shiftctrl_word, pinctrl=self.driver.pinctrl_word,
            forced=[self.forced(self.prog.instrs[self.prog.labels["init"]],
                                 side=i2cd.SIDE_SCL_RELEASED)])
        await self.host.wait_ticks(self.bus_free_ticks)
        await self.wait_parked()

    # -- one phase of a transfer ---------------------------------------------------

    async def stream(self, words) -> list:
        """Push `words`, taking back one RX word per byte pushed.

        This is `I2cDriver._stream` with two things done in bigger bites, both of them
        documented: the address stays put in the FIFO data registers
        (docs/mvp-regmap.md `### Bursts`), so one write burst fills the TX FIFO and one
        read burst drains the RX FIFO, and the two interrupt lines say when there is
        something to do.  The engine eats one word every nine bit times and both FIFOs
        are four deep, so the host always has room and the engine never overruns it: what
        is left is only to keep the transactions off the critical path, which matters
        more over SPI, where one access is tens of core cycles instead of one.
        """
        pending = list(words)
        received = []
        self._tick_down = PHASE_LIMIT
        while len(received) < len(words):
            if pending and self.host.tx_ready():
                level = await self.tx_level()
                room = min(len(pending), i2cd.FIFO_DEPTH - level)
                if room:
                    batch, pending = pending[:room], pending[room:]
                    await self.host.write(A_TXF[0], *batch)
                    self._tick_down = PHASE_LIMIT
            if self.host.rx_valid():
                level = await self.rx_level()
                if level:
                    received += await self.host.read(A_RXF[0], level)
                    self._tick_down = PHASE_LIMIT
            if len(received) < len(words):
                await self.spend(WAIT_STEP, f"{len(words)} RX words, {len(received)} in")
        return received

    async def spend(self, ticks: int, what: str) -> None:
        """Run the clock, bounded: one phase of a transfer has to make progress."""
        await self.host.wait_ticks(ticks)
        self._tick_down -= ticks
        if self._tick_down <= 0:
            raise AssertionError(
                f"{what}: no progress in {PHASE_LIMIT} core cycles "
                f"(FDEBUG {await self.host.read1(A_FDEBUG):#010x}, "
                f"SM0_ADDR {await self.pc()})"
            )

    async def phase(self, words) -> list:
        """START, then `words`, then park: `I2cDriver.phase`.

        The two forced instructions are `set y, <n>` (how many words this phase will
        consume) and `jmp main` (which makes the START), both of which hold SCL low, since
        that is where the previous phase left the bus.

        The one exception is the first phase after `load()`, where the engine is parked in
        the wrap region with both lines released and the bus is idle -- that idle time is
        the tBUF `I2cTimingChecker` wants to see -- so there the first forced instruction
        releases SCL as well.  It has to: over SPI the two forced instructions are
        hundreds of core cycles apart, and holding SCL low for the first one would put a
        two-cycle-wide low pulse on the bus between them, which is a tLOW violation the
        ISS-side driver never sees because there it writes both in consecutive cycles.
        """
        if not 1 <= len(words) <= 31:
            raise AssertionError(f"a phase of {len(words)} words: Y counts them in 5 bits")
        await self.wait_parked()
        parked = await self.pc()
        side = (i2cd.SIDE_SCL_LOW if parked == self.prog.labels["idle"]
                else i2cd.SIDE_SCL_RELEASED)
        await self.push_forced(i2cd.set_y_word(len(words)), side)
        await self.push_forced(i2cd.jmp_word(self.prog.labels["main"]), i2cd.SIDE_SCL_LOW)
        return await self.stream(words)

    async def stop(self) -> None:
        """STOP on the bus, then wait out the bus-free time: `I2cDriver.stop`."""
        await self.wait_parked()
        await self.push_forced(i2cd.jmp_word(self.prog.labels["stop_seq"]))
        await self.wait_left_parked()             # the forced jump has landed
        await self.wait_parked()                  # and the STOP sequence is done
        await self.host.wait_ticks(self.bus_free_ticks)

    # -- transactions --------------------------------------------------------------

    async def write(self, address: int, data) -> list:
        """START, address+W, every data byte, then park with the bus still held.

        No STOP: this is the write half of a combined transfer, so the bus is left with
        SCL low and SDA released, which is where a repeated START grows from.
        """
        words = [i2cd.tx_word(self.control_byte(address, 0))] + [i2cd.tx_word(b) for b in data]
        received = await self.phase(words)
        self.check_ack(address, received[0], "write")
        return [i2cd.rx_ack(word) for word in received[1:]]

    async def write_read(self, address: int, data, count: int):
        """One bus transaction: write `data`, repeated START, read `count` bytes, STOP."""
        if count < 1:
            raise ValueError(f"read {count} bytes: an I2C read has at least the address")
        acked = await self.write(address, data) if data else []
        words = [i2cd.tx_word(self.control_byte(address, 1))]
        words += [i2cd.read_word(ack=(i < count - 1)) for i in range(count)]
        received = await self.phase(words)
        self.check_ack(address, received[0], "read")
        read = bytes(i2cd.rx_byte(word) for word in received[1:])
        # On a read byte the ninth sampled bit is this master's own ACK bit coming back
        # off SDA, so comparing it with what was pushed checks that the ACKs reached the
        # wire: an ACK reads low, and the last byte is NACKed on purpose to end the read.
        wanted = [(i < count - 1) for i in range(count)]
        echoed = [i2cd.rx_ack(word) for word in received[1:]]
        await self.stop()
        if echoed != wanted:
            raise AssertionError(
                f"the read bytes came back with ACK bits {echoed}, the slave was sent "
                f"{wanted}; the RX words were {[hex(w) for w in received]}")
        return acked, read

    @staticmethod
    def control_byte(address: int, read: int) -> int:
        if not 0 <= address <= 0x7F:
            raise ValueError(f"I2C address must be 7 bits, got {address:#x}")
        return (address << 1) | read

    @staticmethod
    def check_ack(address: int, word: int, what: str) -> None:
        """The ninth sampled bit of an address byte is the slave's answer to it."""
        if not i2cd.rx_ack(word):
            raise AssertionError(f"nothing ACKed the {what} address {address:#04x}")


@cocotb.test()
async def test_i2c_eeprom_read(dut):
    """`fw/i2c` on engine 0 reads an `I2cEepromTarget` on uio[1:0], with clean timing.

    One transaction does the lot: the word address, one data byte written (which leaves the
    EEPROM's counter one past it), a repeated START, an address+R, three bytes read, STOP.
    The judge is not this test: `I2cEepromTarget` keeps the memory image and NACKs every
    address but its own, and `I2cTimingChecker` decodes the wire and checks every minimum
    of fast mode.  Neither is edited or softened -- a violation fails the test.
    """
    eeprom = I2cEepromTarget(address=I2C_EEPROM_ADDRESS, mode=I2C_MODE,
                             stretch_ticks=I2C_STRETCH_TICKS, scl_net="scl", sda_net="sda")
    checker = I2cTimingChecker(mode=I2C_MODE, scl_net="scl", sda_net="sda")
    host = SpiHost(dut, pins={"sda": ("uio", I2C_SDA_PIN), "scl": ("uio", I2C_SCL_PIN)},
                   devices=[eeprom, checker])
    # Both models take their tick rate from the bus they are attached to.
    eeprom.bind(host.bridge.bus)
    checker.bind(host.bridge.bus)
    # The driver leaves the bus idle this long after every STOP and after load: the larger
    # of tBUF and tHIGH, because the checker measures the first SCL high period from tick 0.
    idle = ns_to_ticks(NS_PER_TICK, max(I2C_MODES[I2C_MODE].t_buf,
                                         I2C_MODES[I2C_MODE].t_high))
    fw = I2cFirmware(host, bus_free_ticks=idle)
    await host.start()
    try:
        await fw.load()
        eeprom.memory[0x0D:0x10] = bytes([0x15, 0x16, 0x17])

        acked, data = await fw.write_read(I2C_EEPROM_ADDRESS, [0x0C, 0xC0], 3)

        assert acked == [True, True], f"the EEPROM did not ACK both written bytes: {acked}"
        # The write left the EEPROM's counter at 0x0D, so the repeated START reads from
        # there: 0xC0 is at 0x0C and the three bytes come back from 0x0D.
        assert data == bytes([0x15, 0x16, 0x17]), f"read {data.hex()}, wanted 151617"
        assert bytes(eeprom.memory[0x0C:0x10]) == bytes([0xC0, 0x15, 0x16, 0x17]), (
            f"the EEPROM holds {bytes(eeprom.memory[0x0C:0x10]).hex()}")
        # The wire itself: one transaction, so two STARTs (the second a repeated one, no
        # STOP between them), one STOP, and seven bytes: address+W, two written, address+R,
        # three read.
        assert checker.starts == 2 and checker.repeated_starts == 1, (
            f"{checker.starts} START, {checker.repeated_starts} repeated")
        assert checker.stops == 1, checker.stops
        assert checker.bytes == 7, checker.bytes
        # The one NACK is the master's own, on the last read byte: that is how an I2C read
        # ends (`I2cDriver.write_read` pushes `read_word(ack=False)` for the last byte).
        nacked = [t for t in checker.transfers if not t.ack]
        assert len(nacked) == 1 and nacked[0] is checker.transfers[-1], (
            f"{len(nacked)} NACKs: {[str(t) for t in nacked]}")
        assert checker.clean, checker.report()
        # No word was dropped on the way in or out.  TXSTALL and RXSTALL are expected: over
        # SPI the host is tens of core cycles per access, so the engine waits for it.
        fdebug = await host.read1(A_FDEBUG)
        assert fdebug & FDEBUG_LOST == 0, f"FDEBUG = {fdebug:#010x}"
        cocotb.log.info("top: i2c deepest TX/RX FIFO level seen by the host %d/%d, "
                        "%d bytes on the bus, %d core cycles",
                        fw.peak_tx, fw.peak_rx, checker.bytes, host.bridge.bus.t)
        host.check()
    finally:
        await host.stop()
    host.log_stats()
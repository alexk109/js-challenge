"""End-to-end check of fw/spi_target on the ISS: program, driver, chip and bus.

The judge here is not the driver: it is `verif.vip.spi`, which is the only thing that looks at
the bus.  `SpiController` is the master that drives CSn/SCK/MOSI and samples MISO, and
`SpiMonitor` watches the same four wires, decodes every CSn window into (mosi, miso) and
checks the host timing rules of `docs/mvp-regmap.md` `### Physical layer`.  Neither is
edited, neither is subclassed and neither is weakened: a violation fails the test.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick (50 MHz) through
`iss.pio_iss.ChipDevice`.  Every test asserts `monitor.violations == []` **and** that the
bytes the monitor decoded off MISO are the ones the device was told to send, because
`violations` only holds timing breaches -- MISO that arrives late is a *data* error, and a
data error is exactly what this test file is about.

## The one adapter in this file, and why it exists

`SpiController` names its four nets by module constants -- `verif/vip/spi.py` gives it no
net-name arguments -- while `ChipDevice` names the chip's 18 pins `p0`..`p17`.  The two
therefore cannot share a net as they stand.  Bridging them with a device that copies values
from one net to the other would cost **one core cycle in each direction**, and one core cycle
is 20% of the five-cycle budget `fw/spi_target/program.pio` has to hit at the maximum rate:
the measurement would be of the bridge, not of the chip.  So `ChipPins` below names the
chip's pin nets from a list instead, which puts the four SPI wires on the VIP's own nets
with nothing in between, and leaves the chip's behaviour entirely to `iss.pio_iss.Chip`.
As a bonus the pin map becomes a parameter, so `test_the_four_lines_can_be_on_any_four_pins`
moves them.

`FW_TARGET=iss` runs all of it against `iss.pio_iss.Chip`; `FW_TARGET=iss_b` against
`verif/iss_b/pio_iss_b.py` behind the same contract; `FW_TARGET=rtl` against
`src/pio_chip.sv` under cocotb, one core cycle per tick (`fw/rtl_harness.py`).
"""
from __future__ import annotations


import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from asm.pioasm import Program, assemble
from iss import pio_iss
from iss.pio_iss import Chip, ChipDevice
from verif.vip.core import Bus
from verif.vip.spi import (
    CMD_JEDEC,
    CMD_READ,
    CS_N,
    MISO as MISO_NET,
    MOSI as MOSI_NET,
    SCK as SCK_NET,
    SpiController,
    SpiMonitor,
)

from fw import targets
from fw.spi_target import (
    CS,
    PROGRAM_NAME,
    MISO,
    MOSI,
    SCK,
    SpiTargetConfig,
    SpiTargetDriver,
    assemble_program,
    jedec_stream,
    program_with_pins,
    read_stream,
)
from fw import regmap as _regmap

#: 50 MHz, docs/mvp-regmap.md `### Physical layer`.
NS_PER_TICK = 20.0
F_CLK_HZ = 1.0e9 / NS_PER_TICK
JEDEC_ID = (0xC0, 0x28, 0x17)
#: The host timing rules, in core cycles, that the monitor enforces: SCn and SCK each at
#: least 2 core cycles, on every edge and around the window.
MIN_TICKS = 2

# ---- the measured rate ------------------------------------------
#
# The ceiling is `half_period >= T_IN + 1 (the `out`) + T_OUT = 5`, i.e. 10 core cycles per
# bit, 5.00 MHz at 50 MHz.  `test_the_fastest_half_period_that_serves_every_transaction`
# re-measures the boundary and checks it against these three numbers, so a change in the
# program, the VIP or the ISS that moves the ceiling fails here instead of quietly
# invalidating the README.
MAX_HALF_CLEAN = 5
MAX_BIT_TICKS = 2 * MAX_HALF_CLEAN
MAX_SCK_HZ = F_CLK_HZ / MAX_BIT_TICKS
#: Half-periods that are *below* the ceiling, with the reason they fail.  Each one is an
#: expected failure in `test_the_rate_sweep`, and the reason is the measured one: the MISO
#: bit reaches the net (MAX_HALF_CLEAN - half) core cycles after the cycle the master samples
#: it, so the device answers `MAX_HALF_CLEAN - half` bits late and the answer is shifted.
HALVES_TOO_FAST = (2, 3, 4)
#: The configuration the other tests run at: the fastest rate with a core cycle of margin,
#: so nothing here depends on the answer landing in the very cycle it is sampled.
SHIPPED_HALF = 6


#: SM cycles the loop spends per bit: `wait 0`, `out pins, 1`, `wait 1`, `in pins, 1` and the
#: `jmp !osre` that loops it back.  The byte boundary spends three more (`jmp !osre`,
#: `jmp pin, idle`, `pull block`) and the wrap replaces the fourth jump.
LOOP_CYCLES = 5


def why_too_fast(half: int) -> str:
    """The measured reason `half` cannot be served, for an xfail marker.

    Two failure modes, and which one you get is arithmetic rather than luck: below
    `LOOP_CYCLES / 2` ticks per bit the loop cannot retire in a bit period at all, so the
    engine loses edges and the *receive* side breaks too; from there up to the ceiling the
    loop keeps up and the only thing wrong is that MISO reaches the net after the master has
    sampled, so the answer comes back shifted.
    """
    if LOOP_CYCLES > 2 * half:
        return (
            f"half_period {half} gives {2 * half} core cycles per bit and the loop retires "
            f"{LOOP_CYCLES} instructions per bit, so it cannot keep up: the engine misses "
            f"SCK edges and the received bytes break as well as the transmitted ones.  "
            f"half_period {LOOP_CYCLES // 2 + 1} is the first that fits."
        )
    late = MAX_HALF_CLEAN - half
    return (
        f"half_period {half} is {late} core cycle(s) below the ceiling: MISO reaches the net "
        f"at T+3 (T_IN) +1 (the `out` instruction) +1 (T_OUT) = T+{MAX_HALF_CLEAN}, and the "
        f"master samples it at T+{half}, so the device answers {late} bit(s) late and the "
        f"monitor decodes shifted bytes.  Raise half_period to {MAX_HALF_CLEAN}."
    )


class ChipPins(ChipDevice):
    """`ChipDevice` whose 18 pin nets are named by a list instead of by a prefix.

    Everything else -- reading `pin_in`, performing at most one queued host access per tick,
    stepping the chip, driving `pin_out` where `pin_oe` is 1 -- is `iss.pio_iss.ChipDevice`
    unchanged; only the net names differ, and the class exists so that the four SPI wires can
    be the VIP's own `cs_n`/`sck`/`mosi`/`miso` nets with no cycle of adapter in between (see
    this module's docstring).  `nets[k]` is protocol pin `k`, `docs/mvp-isa.md`
    `### The 18 protocol pins`.
    """

    def __init__(self, chip=None, nets=None, name: str = "chip") -> None:
        super().__init__(chip if chip is not None else Chip(), name=name)
        self.nets = list(nets) if nets is not None else [f"p{k}" for k in range(pio_iss.NUM_PINS)]
        if len(self.nets) != pio_iss.NUM_PINS:
            raise ValueError(
                f"{len(self.nets)} net names for {pio_iss.NUM_PINS} protocol pins"
            )

    def tick(self, t: int, pins) -> dict:
        pin_in = 0
        for k, net in enumerate(self.nets):
            if pins.get(net, 0):
                pin_in |= 1 << k
        bus = fut = None
        if self.queue:
            kind, addr, data, fut = self.queue.popleft()
            bus = ("w", addr, data) if kind == "w" else ("r", addr)
        r = self.chip.step(pin_in, bus)
        if fut is not None:
            fut.set_result(r)
        out, oe = self.chip.pin_out, self.chip.pin_oe
        return {
            net: ((out >> k) & 1 if (oe >> k) & 1 else None)
            for k, net in enumerate(self.nets)
            if net in pins
        }


class Rig:
    """A bus with the chip as the SPI *target*, a controller as the master, and the monitor.

    The four SPI nets are the VIP's own, push-pull; the other fourteen protocol pins get the
    default pull-up and nothing drives them.  The controller is created once and reused, so
    a rig can queue several transactions in a row and `monitor.transfers` holds one entry
    per CSn window.
    """

    def __init__(
        self,
        half_period: int = SHIPPED_HALF,
        clkdiv: tuple[int, int] = (1, 0),
        mode: int = 0,
        pins: tuple[int, int, int, int] = (CS, MOSI, SCK, MISO),
        jedec_id: tuple[int, ...] = JEDEC_ID,
        program=None,
        cs_setup: int = MIN_TICKS,
        cs_hold: int = MIN_TICKS,
        cs_gap: int = MIN_TICKS,
        fill: int = 0x00,
    ):
        if mode not in (0, 3):
            raise ValueError(f"this firmware is mode 0 only, got mode {mode}")
        self.mode = mode
        self.half = half_period
        self.pins = tuple(pins)
        if len(set(self.pins)) != 4 or not all(0 <= p < 8 for p in self.pins):
            raise ValueError(
                f"the four SPI lines must be four distinct uio pins 0..7, got {self.pins}"
            )
        self.bus = Bus(ns_per_tick=NS_PER_TICK)
        nets = [None] * pio_iss.NUM_PINS
        for net, pin in zip((CS_N, MOSI_NET, SCK_NET, MISO_NET), self.pins):
            nets[pin] = net
        nets = [n if n is not None else f"unused{k}" for k, n in enumerate(nets)]
        for net in nets:
            self.bus.add_net(net, push_pull=net in (CS_N, MOSI_NET, SCK_NET, MISO_NET))
        self.chip = Chip()
        self.dev = ChipPins(self.chip, nets=nets, name="chip")
        self.bus.attach(self.dev)
        self.ctl = self.bus.attach(
            SpiController(
                mode=mode, half_period=half_period, cs_setup=cs_setup, cs_hold=cs_hold,
                cs_gap=cs_gap, fill=fill, name="spi_ctl",
            )
        )
        self.monitor = self.bus.attach(
            SpiMonitor(
                mode=mode, min_sck_ticks=MIN_TICKS, min_cs_setup=MIN_TICKS,
                min_cs_hold=MIN_TICKS, name="spi_mon",
            )
        )
        # The two clock lines are absolute `gpio` waits, so their pin numbers are in the
        # instruction words: the program is re-assembled for this rig's map (for the shipped
        # map that is the shipped program, word for word).
        if program is None:
            program = program_with_pins(self.pins)
        self.driver = SpiTargetDriver(self.dev, SpiTargetConfig(pins=self.pins, clkdiv=clkdiv),
                                      program=program)
        self.driver.load()

    # ---- one transaction -------------------------------------------------------------------

    def transfer(self, mosi, n_miso: int, miso_stream) -> None:
        """Arm the device, then let the master send `mosi` and read `n_miso` more bytes.

        The order matters and is the whole reason this is two steps: the device must be
        armed and parked in `idle` *before* the master starts, because it sees CSn through
        T_IN = 3 core cycles and misses nothing if it is already waiting.  `SpiController`
        plans from the tick it is queued on, so the master is queued only after the setup
        queue has drained, and `pending_ticks` is asked with the bus's own tick so the plan
        is not built from the controller's last-seen tick (which is one behind).

        `SpiController.queue_transfer(mosi, n_miso)` sends `len(mosi)` bytes and then clocks
        `n_miso` *more* (`verif/vip/spi.py`, `_plan_queue`), so a 0x03 READ is four bytes of
        MOSI (opcode plus 24-bit address) and `n_miso` dummy bytes clocked in while the data
        comes out, exactly as `SpiFlashTarget` expects.
        """
        self.driver.start(miso_stream, n_rx=len(mosi) + n_miso)
        self.driver.run(self.bus, MIN_TICKS)          # drain the setup, engine armed
        self.ctl.add(mosi, n_miso)
        self.driver.run(self.bus, self.ctl.pending_ticks(self.bus.t) + 64)
        self.driver.stop(self.bus)                    # release MISO, stop the engine

    def jedec(self, jedec_id=JEDEC_ID):
        """One 0x9F transaction: the opcode out, three ID bytes back."""
        self.transfer([CMD_JEDEC], len(jedec_id), jedec_stream(jedec_id))
        return self.monitor.last

    def read(self, addr: int, data):
        """One 0x03 transaction: opcode, 24-bit address, then `len(data)` bytes back."""
        data = list(data)
        mosi = [CMD_READ, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]
        self.transfer(mosi, len(data), read_stream(data))
        return self.monitor.last

    # ---- what the judges saw ---------------------------------------------------------------

    def check_clean(self) -> None:
        """The monitor, the bus's own contention record, and the chip's own FIFO flags."""
        assert self.monitor.violations == [], self.monitor.violations
        assert self.bus.errors == [], self.bus.errors
        assert self.driver.read_fdebug(self.bus) == 0, (
            "FDEBUG raised a FIFO flag: the loop keeps up only if no stall happened"
        )

    def sck_period_ticks(self) -> int:
        """The shortest SCK period the monitor saw, in core cycles.

        `stats` holds the shortest time SCK held each level and the bit period is their sum.
        (Which key holds which level is a labelling matter inside the monitor, so this uses the sum, not the keys.)
        """
        stats = self.monitor.stats
        return stats["sck_low_min"] + stats["sck_high_min"]

    def sck_hz(self) -> float:
        """The fastest SCK rate the monitor saw: the clock rate over ticks per bit."""
        return F_CLK_HZ / self.sck_period_ticks()

    def report(self) -> str:
        xfer = self.monitor.last
        return (
            f"half_period={self.half} stats={self.monitor.stats} "
            f"mosi={[hex(b) for b in xfer.mosi]} miso={[hex(b) for b in xfer.miso]} "
            f"violations={self.monitor.violations[:3]}"
        )

# ---- the transactions ----------------------------------------------------


def test_jedec_id_off_the_wire() -> None:
    """0x9F: the device puts the three ID bytes on MISO, and the monitor decodes them there.

    This is the whole claim in one test: a PIO program that never drives SCK
    answers a master inside the bit period.  The bytes are compared *off the wire*, from the
    monitor's decode, not from the chip's RX FIFO -- the FIFO is the device's own account of
    what it received, and only the monitor saw what the master actually sampled.
    """
    rig = Rig()
    xfer = rig.jedec()
    assert xfer.mosi[0] == CMD_JEDEC, "the master sent the opcode MSB first"
    assert xfer.miso == [0x00, *JEDEC_ID], rig.report()
    # The device also received the opcode: it is in its RX FIFO, one byte per word.
    assert rig.driver.received()[:4] == [CMD_JEDEC, 0x00, 0x00, 0x00]
    rig.check_clean()


def test_read_off_the_wire() -> None:
    """0x03 with a 24-bit address: four silent bytes, then the data, in order."""
    rig = Rig()
    data = [(0xA0 + i) & 0xFF for i in range(16)]
    xfer = rig.read(0x30, data)
    # The monitor decodes a byte per capture edge in both directions, so the window holds the
    # four bytes the master sent and the sixteen fill bytes it clocked while reading.
    assert xfer.mosi[:4] == [CMD_READ, 0x00, 0x00, 0x30], rig.report()
    assert xfer.mosi[4:] == [0x00] * 16, rig.report()
    assert xfer.miso == [0x00, 0x00, 0x00, 0x00, *data], rig.report()
    # The 24-bit address really arrived: four bytes into the transaction the device holds
    # [0x03, 0x00, 0x00, 0x30], and the rest are the master's fill.
    assert rig.driver.received()[:4] == [CMD_READ, 0x00, 0x00, 0x30]
    assert rig.driver.received()[4:] == [0x00] * 16
    rig.check_clean()


@pytest.mark.parametrize("addr", [0x000000, 0x000030, 0x00FFFF, 0x123456, 0xFFFFFF])
def test_the_whole_24_bit_address_reaches_the_device(addr: int) -> None:
    """The address is clocked in whole, all three bytes, at every value that has bits set.

    The MVP PIO cannot *use* an address as an index (see `README.md`, "What the role cannot
    express"), but it can receive one, and the host can read it back over the register bus --
    which is what makes a host-positioned flash window possible at all.  So this test pins
    the receiving half of the contract: all 24 bits, MSB first, no byte dropped or swapped.
    """
    rig = Rig()
    xfer = rig.read(addr, [0x5A])
    assert xfer.miso == [0x00] * 4 + [0x5A], rig.report()
    assert rig.driver.received()[:4] == [CMD_READ, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF,
                                         addr & 0xFF], rig.report()
    rig.check_clean()


@pytest.mark.parametrize("n", [1, 2, 7, 8, 15, 16, 31])
def test_any_length_comes_back_whole(n: int) -> None:
    """Every length from one byte to the 31 the FIFO can stream, across byte boundaries.

    A length that is not a multiple of eight is the interesting half: it puts the device's
    own byte boundary (`jmp !osre`, `pull block`) in the middle of the master's read and it
    ends the transaction inside one, which is where a phase slip would show.
    """
    rig = Rig()
    data = [(0x11 * (i + 1)) & 0xFF for i in range(n)]
    xfer = rig.read(0x10, data)
    assert xfer.miso[4:] == data, f"{n} bytes: {xfer.miso[4:]}"
    rig.check_clean()


@pytest.mark.parametrize("jedec_id", [(0xC0, 0x28, 0x17), (0xEF, 0x40, 0x18), (0x1F,), (0x00,)])
def test_the_answer_really_comes_from_the_host(jedec_id) -> None:
    """Change what the host queues and the answer changes with it.

    Without this, a firmware that emitted a fixed pattern would pass the two tests above.
    It is the check that makes "the response content is the host's" a tested property rather
    than an assumption -- and it is also the honest statement of what the role became:
    `README.md` has the rest of the limits.
    """
    rig = Rig()
    xfer = rig.jedec(jedec_id=jedec_id)
    assert xfer.miso == [0x00, *jedec_id], rig.report()
    rig.check_clean()


def test_two_transactions_on_one_load() -> None:
    """A JEDEC ID and a READ back to back, one program load, two CSn windows.

    The re-arm path: the engine is parked in `idle` when the master released CSn, and the
    second transaction has to stop it, re-force the pin direction, the OSR and the PC, and
    start it again.  It is also the proof that the loop really does park (`jmp pin, idle`)
    rather than free-running on a low SCK between transactions.
    """
    rig = Rig()
    first = rig.jedec()
    data = [(0x5A + i) & 0xFF for i in range(8)]
    second = rig.read(0x00, data)
    assert first.miso == [0x00, *JEDEC_ID]
    assert second.miso == [0x00] * 4 + data
    assert len(rig.monitor.transfers) == 2
    assert rig.monitor.transfers[0].mosi[0] == CMD_JEDEC
    assert rig.monitor.transfers[1].mosi[0] == CMD_READ
    gap = rig.monitor.transfers[1].t_start - rig.monitor.transfers[0].t_end
    assert gap >= 1, "CSn was high between the two transactions"
    rig.check_clean()


def test_miso_is_released_between_transactions() -> None:
    """MISO is high-Z while no transaction is running, the way a real flash sits.

    `SpiFlashTarget` drives MISO only while CSn is low and releases it otherwise, so a device
    that held MISO low between transactions would still pass every decode above and would
    fight a real bus.  `stop()` is the host's half of this: a forced `set pindirs, 0`.
    """
    rig = Rig()
    rig.jedec()
    assert (rig.chip.pin_oe >> MISO) & 1 == 0, "MISO is still driven after stop()"
    # ... and the device still works after being released and re-armed.
    data = [0x11, 0x22, 0x33, 0x44]
    assert rig.read(0x08, data).miso == [0x00] * 4 + data
    rig.check_clean()


def test_the_device_parks_when_the_master_stops_clocking() -> None:
    """With CSn high and SCK at its idle level the engine is *stalled*, and stays that way.

    The loop's two waits are level waits, so when the master stops clocking the engine
    settles in whichever of the two it reached last (with SCK idles low in mode 0, that is
    the capture wait `wait 1 gpio SCK`).  Where it parks is a property of the master's last
    edge; what matters is that it is parked at all: an engine that free-ran would keep
    shifting MISO and pushing RX FIFO words after the master had walked away, which is what
    `jmp pin, idle` at the byte boundary is there to prevent on a CPOL 1 master.

    So this test asserts the property, not the address: the PC sits on a WAIT (which is where
    `IR` points, and `IR` is refreshed from IMEM every core cycle whatever else happens), both
    FIFOs stay empty, and MISO is frozen for 40 core cycles afterwards.  It also shows why
    the host re-pre-loads the OSR between transactions: the spurious `out pins, 1` on the CSn
    hold edge leaves osr_cnt at 1.

    `engine.stalled` is *not* asserted: `stop()` disables the engine with a CTRL write and
    then forces `set pindirs, 0`, and a host-forced instruction that completes clears
    `stalled` for its own cycle (docs/mvp-isa.md `### Stalls, delay and side-set,
    summarised`).  Whether the engine was stalled a moment earlier is not observable after the
    host has stopped it, and does not need to be.
    """
    rig = Rig()
    rig.jedec()
    engine = rig.chip.engines[0]
    assert engine.ir >> 13 == 0b001, f"parked outside a WAIT: IR {engine.ir:#06x}"
    assert engine.tx == [] and engine.rx == [], "a FIFO word was left behind"
    before = (rig.chip.pin_out, rig.chip.pin_oe)
    for _ in range(40):
        rig.bus.step()
    assert (rig.chip.pin_out, rig.chip.pin_oe) == before, "MISO moved after the transaction"
    assert engine.ir >> 13 == 0b001, "the engine came out of its wait with no clock edges"
    assert engine.tx == [] and engine.rx == []


# ---- the rate sweep: where the ceiling is, and what sets it -------------------------------

#: Half-periods the sweep walks.  2..12 covers "faster than anything" to "far slower than
#: anything", and every entry is either a pass or an expected failure with a measured reason.
HALF_PERIODS = (2, 3, 4, 5, 6, 8, 12)


def sweep_half_period(half: int, clkdiv: tuple[int, int] = (1, 0), program=None) -> dict:
    """Run both transactions at `half` and report what the wire carried.

    Returns a record rather than asserting, so the sweep itself can be printed, measured and
    written to a log: `fw/spi_target/measure.py` is this function over a grid, and
    `test_the_fastest_half_period_that_serves_every_transaction` is the same grid with an
    assertion on the boundary.
    """
    rig = Rig(half_period=half, clkdiv=clkdiv, program=program)
    data = [(0xA0 + i) & 0xFF for i in range(16)]
    jedec = rig.jedec()
    read = rig.read(0x30, data)
    fdebug = rig.driver.read_fdebug(rig.bus)
    return {
        "half": half,
        "clkdiv": clkdiv,
        "rate_hz": rig.sck_hz(),
        "period_ticks": rig.sck_period_ticks(),
        "jedec_ok": jedec.miso == [0x00, *JEDEC_ID],
        "read_ok": read.miso == [0x00] * 4 + data,
        "miso": list(jedec.miso),
        "read_miso": list(read.miso[4:20]),
        "violations": list(rig.monitor.violations),
        "bus_errors": len(rig.bus.errors),
        "fdebug": fdebug,
    }


def served(record: dict) -> bool:
    """True when the record is a clean serve: right bytes, no violations, no flags.

    `violations` alone is not enough and `miso` alone is not enough either: the monitor only
    holds timing breaches, and a device whose MISO arrives a cycle late breaks nothing in
    `violations` -- it just returns the wrong bytes.  Both, plus `FDEBUG` (a stall or an
    overrun the transaction survived by luck), is the bar.
    """
    return (
        record["jedec_ok"]
        and record["read_ok"]
        and not record["violations"]
        and not record["bus_errors"]
        and record["fdebug"] == 0
    )


def _half_period_params() -> list:
    """One parametrisation entry per half period, with the measured xfail marks attached.

    The reason string is built by `why_too_fast()` from the measured ceiling, so a test report
    says *why* a rate is expected to fail rather than only that it is.  `strict=True` means a
    pass at one of those rates fails this test, so the ceiling cannot move silently upwards.
    """
    out = []
    for half in HALF_PERIODS:
        if half in HALVES_TOO_FAST:
            out.append(pytest.param(half, marks=pytest.mark.xfail(
                strict=True, reason=why_too_fast(half))))
        else:
            out.append(pytest.param(half))
    return out


@pytest.mark.parametrize("half", _half_period_params())
def test_the_rate_sweep(half: int) -> None:
    """Every half period in the sweep, at both transactions, with the reason attached.

    This is the parametrised sweep: the rates that are served pass, and the
    ones below the ceiling are expected failures carrying the measured reason.
    """
    record = sweep_half_period(half)
    assert served(record), record


def test_the_fastest_half_period_that_serves_every_transaction() -> None:
    """The headline number, re-measured: half_period 5, 10 core cycles, 5.00 MHz.

    Walks the sweep, finds the fastest half period that serves *both* transactions cleanly,
    and checks it against the recorded `MAX_HALF_CLEAN` / `MAX_BIT_TICKS` / `MAX_SCK_HZ`.  If
    the program, the VIP or the ISS moves the ceiling, this fails and says what moved -- which
    is what keeps the README's number honest.
    """
    records = [(h, sweep_half_period(h)) for h in HALF_PERIODS]
    clean = [h for h, r in records if served(r)]
    assert clean, f"not one half period served both transactions: {records}"
    fastest = min(clean)
    assert fastest == MAX_HALF_CLEAN, (
        f"the fastest clean half period moved to {fastest} ({(1e9 / NS_PER_TICK / (2 * fastest)) / 1e6:.2f} MHz), "
        f"recorded {MAX_HALF_CLEAN} ({MAX_SCK_HZ / 1e6:.2f} MHz); re-run "
        f"`python -m fw.spi_target.measure` and update the constants"
    )
    # Everything faster fails, and the way it fails is the one the reason string names.
    for half in HALVES_TOO_FAST:
        record = dict(records)[half]
        assert not served(record), f"half_period {half} served after all"
        assert not record["violations"], (
            f"half_period {half} failed on monitor violations, not on late MISO: {record}"
        )
        if half >= 3:
            # MISO one or two bits late: the answer is the same bits shifted, which is what
            # "the device cannot answer inside the bit time" looks like on the wire.
            assert record["miso"] != [0x00, *JEDEC_ID] and record["read_miso"], record
        else:
            # Too fast for the loop itself: MOSI reception fails as well, because the engine
            # misses edges rather than answering them late.
            assert rig_rx_broken(half), f"half_period {half} failed for an unexpected reason"
    # The ceiling itself: T_IN = 3 + the `out` instruction + T_OUT = 1 = 5 core cycles from
    # the launch edge to MISO on the net, and the master samples half a bit period later.
    assert MAX_HALF_CLEAN == 3 + 1 + 1
    assert MAX_BIT_TICKS == 2 * MAX_HALF_CLEAN == 10
    assert MAX_SCK_HZ == 5.0e6, MAX_SCK_HZ


def rig_rx_broken(half: int) -> bool:
    """True when the engine also mis-received at `half`, i.e. it lost edges rather than
    answering late.  Only half_period 2 does: five SM cycles per bit against four core
    cycles in a bit period."""
    rig = Rig(half_period=half)
    rig.read(0x30, [0x5A])
    return rig.driver.received()[:4] != [CMD_READ, 0x00, 0x00, 0x30]


#: The measured ceiling per integer divider, `half_period`, from
#: the sweep.  Recorded rather than derived: at
#: CLKDIV 1.0 the answer is exactly `T_IN + 1 instruction + T_OUT = 5`, but from INT 2 on the
#: loop's own length starts to bind as well (a byte boundary costs eight SM cycles, five in
#: the loop and three at the boundary, and a bit period is `2 * half / INT` of them), so the
#: two effects are not a sum and the table is what it is.  `measure.py` re-measures it.
DIVIDER_CEILINGS = {1: 5, 2: 9, 3: 12, 4: 15}


def pio_text() -> str:
    from fw.spi_target import PROGRAM_PIO
    return PROGRAM_PIO.read_text(encoding="utf-8")


def late_out_program() -> Program:
    """`program.pio` with one `nop` between the launch wait and the `out pins, 1`.

    This is the experiment that measures the instruction term of the ceiling.  The loop is
    otherwise untouched -- same registers, same byte boundary, same length in the steady state
    (the nop runs on every bit) -- so the only thing that moves is how many SM cycles lie
    between "the engine has seen the falling edge" and "the engine writes MISO": one more.
    """
    text = re.sub(r"(\n    wait 0 gpio SCK_PIN[^\n]*\n)", r"\1    nop\n", pio_text())
    return next(p for p in assemble(text) if p.name == PROGRAM_NAME)


@pytest.mark.parametrize("integer,ceiling", sorted(DIVIDER_CEILINGS.items()))
def test_the_measured_ceiling_at_each_divider(integer: int, ceiling: int) -> None:
    """The divider's contribution, as a measured table rather than a formula.

    One SM cycle is `integer` core cycles, so the `out pins, 1` that answers a launch edge
    retires `integer` core cycles after the wait that saw it and the deadline becomes
    `half_period >= T_IN + integer + T_OUT = integer + 4`.  At integer 1 that is the whole
    story and the measured ceiling is 5.  From integer 2 on, the loop's own length starts to
    bind as well -- a byte boundary costs eight SM cycles and a bit period is
    `2 * half_period / integer` of them -- so this test records what is measured
    (`DIVIDER_CEILINGS`) and checks the ceiling never moves without the record being updated.
    """
    halves = range(2, 3 * ceiling + 2)
    clean = [h for h in halves if served(sweep_half_period(h, clkdiv=(integer, 0)))]
    assert clean, f"nothing served at CLKDIV {integer}.0"
    assert min(clean) == ceiling, (
        f"CLKDIV {integer}.0: the fastest clean half period is {min(clean)}, recorded "
        f"{ceiling}; re-run `python -m fw.spi_target.measure` and update DIVIDER_CEILINGS"
    )
    assert ceiling >= integer + 4, (
        f"CLKDIV {integer}.0 measured {ceiling}, below the deadline law integer + 4"
    )


def test_one_more_instruction_on_the_launch_path_costs_one_half_period() -> None:
    """The instruction term, measured: a `nop` before the `out` moves the ceiling from 5 to 6.

    `late_out_program` is the shipped program with one instruction inserted between the launch
    wait and the `out pins, 1`, and nothing else changed.  One SM cycle of latency on the
    launch path is one half period of rate -- which is what makes the decomposition below
    arithmetic rather than a story: the ceiling is the sum of the pad-to-engine latency, the
    instructions on the launch path, and the engine-to-pad latency, and the program can only
    ever pay the middle one.
    """
    shipped = [h for h in (4, 5, 6) if served(sweep_half_period(h))]
    late = [h for h in (4, 5, 6, 7) if served(sweep_half_period(h, program=late_out_program()))]
    assert min(shipped) == MAX_HALF_CLEAN
    assert min(late) == MAX_HALF_CLEAN + 1, (
        f"one extra instruction moved the ceiling to {min(late)}, expected "
        f"{MAX_HALF_CLEAN + 1}"
    )


def test_the_latency_decomposition_adds_up_to_the_ceiling() -> None:
    """Measured edge-to-wire latency of the answer: T_IN + 1 instruction + T_OUT = 5.

    Every term is read off the chip rather than off the spec, on the second bit of a
    transaction so that both of its edges are real:

      * the net's falling SCK edge, and the cycle the `out pins, 1` executes in -- that is
        T_IN plus the instructions on the launch path;
      * the cycle before that, which is the launch wait's own retirement -- the difference is
        the instruction term;
      * the tick MISO changes on the net after it -- T_OUT;
      * the net's rising SCK edge before, and the cycle the `in pins, 1` executes in -- T_IN on
        its own.

    They add up to 5, which is the measured ceiling, and 3 of the 5 are T_IN: the input
    synchroniser no program can remove, which is the reflex thesis C1 is about
    (research/ideation/round2/research/C1.md `### F3`).
    """
    half = SHIPPED_HALF
    rig = Rig(half_period=half)
    rig.driver.start(jedec_stream(), n_rx=4)
    rig.driver.run(rig.bus, MIN_TICKS)
    rig.ctl.add([CMD_JEDEC], 3)
    ticks = rig.ctl.pending_ticks(rig.bus.t) + 64
    engine = rig.chip.engines[0]
    # `IR` is the instruction the engine executes in this cycle, so the cycle an instruction
    # *completed* in is the cycle before the next one starts.
    trace = []
    for _ in range(ticks):
        trace.append((rig.bus.t, rig.bus.values[SCK_NET], rig.bus.values[MISO_NET], engine.ir))
        rig.bus.step()
    sck = [row[1] for row in trace]
    miso = [row[2] for row in trace]
    irs = [row[3] for row in trace]
    # Transitions of SCK on the net, and the bit-1 edges among them.
    edges = [i for i in range(1, len(sck)) if sck[i] != sck[i - 1]]
    rise0, fall1, rise1 = edges[0], edges[1], edges[2]   # bit 0 rises, bit 1 falls and rises
    WAIT1, OUT, IN = 0x2082, 0x6001, 0x4001
    # The `in pins, 1` of bit 0 is the first time the engine executes word 5.
    in0 = irs.index(IN)
    # The `out pins, 1` of bit 1 is the first one after bit 1's falling edge.
    out1 = next(i for i in range(fall1, len(irs)) if irs[i] == OUT)
    wait1_retired = out1 - 1 - (irs[out1 - 1] == OUT)   # the launch wait retired before it
    t_in = in0 - 1 - rise0                              # the capture wait retired here
    instructions = out1 - wait1_retired
    # T_OUT, on the first bit of the answer that is not the same as the one before it: the
    # JEDEC stream is 0x00 then 0xC0, so the MSB of the first ID byte is the first value the
    # net changes to, and it has to be the `out` of the cycle before that.
    transitions = [i for i in range(1, len(miso)) if miso[i] != miso[i - 1]]
    miso_after = transitions[0]
    assert irs[miso_after - 1] == OUT, (
        f"MISO changed at {miso_after} but the cycle before ran IR {irs[miso_after - 1]:#06x}"
    )
    t_out = miso_after - (miso_after - 1)
    assert (t_in, instructions, t_out) == (3, 1, 1), (
        f"measured {t_in}+{instructions}+{t_out}, expected 3+1+1 "
        f"(rise {rise0}, `in` at {in0}, fall {fall1}, `out` at {out1}, MISO at {miso_after})"
    )
    assert t_in + instructions + t_out == MAX_HALF_CLEAN, (
        f"the measured decomposition does not add up to the measured ceiling {MAX_HALF_CLEAN}"
    )
    # And the master's sample of a bit is half a bit period after its launch edge, which is
    # the other half of the inequality: the answer has to beat it.  At half_period 5 it wins
    # by nothing, which is why the shipped tests run at half_period 6 and this is the one core
    # cycle of margin that buys.
    assert rise1 - fall1 == half, (rise1, fall1, half)
    answer_on_the_net = out1 + 1                      # T_OUT = 1 after the `out` cycle
    assert answer_on_the_net <= rise1, "the answer lands after the master sampled it"
    assert rise1 - answer_on_the_net == 1, (
        f"half_period {half} leaves {rise1 - answer_on_the_net} core cycles of margin"
    )


# ---- what the program and the driver are made of ------------------------------------------


def test_the_program_fits_the_instruction_memory() -> None:
    """Nine words of thirty-two, and the labels the driver and the tests jump to are real.

    The word count is a headline result, so it is asserted rather than described:
    `README.md` quotes it, and a program that grows past 32 words cannot be loaded into one
    engine at all (docs/mvp-isa.md `### Engine state`).
    """
    program = assemble_program()
    assert len(program.instrs) == 9, f"{len(program.instrs)} words, expected 9"
    assert len(program.instrs) <= 32
    assert program.labels["idle"] == 0, "the host forces `jmp idle`, so it must be word 0"
    assert program.labels["bit_loop"] == 2
    # The wrap window is the bit loop: a fall-through at the end of it returns to the top, and
    # `wait 0 gpio CS` in `idle` is outside it on purpose.
    assert program.wrap_target == program.labels["bit_loop"]
    assert program.wrap == len(program.instrs) - 1 == program.labels["last"]
    assert program.sideset_bits == 0, "a target drives one pin and needs no side-set"
    # EXECCTRL.JMP_PIN is CSn, and the loop's `jmp pin, idle` is the only user of it.
    from fw.spi_target.driver import JMP_PIN
    assert JMP_PIN == CS, "the loop's `jmp pin, idle` tests CSn, so JMP_PIN is CSn"


def test_the_two_waits_are_on_sck_and_cs_is_watched_with_gpio() -> None:
    """The program's inputs, read out of its words.

    `wait <pol> gpio <n>` takes an *absolute* pin (docs/mvp-isa.md `### WAIT Source`), and the
    two waits in the loop have to be SCn and SCK for the pin map the driver programs.  Getting
    one of them onto a relative `pin` index instead would silently watch a different pin, so
    the encoding is spelled out here rather than left to the assembler's good behaviour.
    """
    program = assemble_program()
    words = program.instrs
    wait0, out1, wait1, in1, jmp = words[2], words[3], words[4], words[5], words[6]
    from fw.spi_target.driver import CS, MISO, MOSI, SCK
    assert wait0 == 0x2000 | (0b00 << 5) | SCK, f"{wait0:#06x} is not `wait 0 gpio SCK`"
    assert wait1 == 0x2080 | (0b00 << 5) | SCK, f"{wait1:#06x} is not `wait 1 gpio SCK`"
    assert words[0] == 0x2000 | (0b00 << 5) | CS, f"{words[0]:#06x} is not `wait 0 gpio CSn`"
    assert (out1 >> 13, out1 & 0x1F) == (0b011, 1), f"{out1:#06x} is not `out pins, 1`"
    assert (in1 >> 13, in1 & 0x1F) == (0b010, 1), f"{in1:#06x} is not `in pins, 1`"
    assert (jmp >> 13, (jmp >> 5) & 7, jmp & 0x1F) == (0b000, 0b111, program.labels["bit_loop"]), (
        f"{jmp:#06x} is not `jmp !osre, bit_loop`"
    )
    # The IN window makes MOSI data bit 0, which is what `in pins, 1` samples.
    assert MOSI + 1 == SCK, "the pin map puts SCK one pin above MOSI on purpose"
    assert MISO == 3


def test_the_config_registers_read_back() -> None:
    """Every register the driver writes, read back over the bus after `load`.

    The values are spelled out rather than taken from the driver, so a change to a field
    position in docs/mvp-regmap.md shows up as a test failure instead of a silent behaviour
    change -- and the four windows are the whole pin contract of a device-mode program.
    """
    from fw.spi_target import driver as drv

    rig = Rig()
    while not rig.dev.idle:
        rig.bus.step()
    rd = rig.driver.read_reg
    bus = rig.bus
    assert rd(bus, drv.SM_BASE + drv.SM_CLKDIV) == 0x0001_0000, "CLKDIV = 1.0"
    assert rd(bus, drv.SM_BASE + drv.SM_EXECCTRL) == 0x0000_8100, (
        "EXECCTRL: JMP_PIN 0 (CSn), WRAP_BOTTOM 2 (bit_loop), WRAP_TOP 8 (last), no side enable"
    )
    assert rd(bus, drv.SM_BASE + drv.SM_SHIFTCTRL) == 0x1081_0000, (
        "SHIFTCTRL: PULL_THRESH 8 (the byte counter), PUSH_THRESH 8, both shift directions "
        "left, AUTOPUSH, no AUTOPULL"
    )
    assert rd(bus, drv.SM_BASE + drv.SM_PINCTRL) == 0x0410_8063, (
        "PINCTRL: no side-set, SET_COUNT 1, OUT_COUNT 1, IN_BASE 1 (MOSI), SIDESET_BASE 0, "
        "SET_BASE 3 (MISO), OUT_BASE 3 (MISO)"
    )
    assert rd(bus, pio_iss.A_DBG_CFGINFO) == _regmap.active().cfginfo
    assert rd(bus, drv.A_CTRL) == 0, "load() must leave both engines disabled"


def test_the_driver_addresses_match_the_register_map_and_the_iss() -> None:
    """The driver's word addresses are the ISS's, so a regmap change cannot pass unnoticed.

    The driver is written against docs/mvp-regmap.md and `iss.pio_iss` is the golden model;
    this is the one place the two are compared directly, and it is the guard against the ISS
    renumbering something under the firmware.
    """
    from fw.spi_target import driver as drv

    assert (drv.A_CTRL, drv.A_FSTAT, drv.A_FDEBUG, drv.A_FLEVEL) == (
        pio_iss.A_CTRL, pio_iss.A_FSTAT, pio_iss.A_FDEBUG, pio_iss.A_FLEVEL
    )
    assert (drv.A_TXF0, drv.A_TXF1, drv.A_RXF0, drv.A_RXF1) == (*pio_iss.A_TXF, *pio_iss.A_RXF)
    assert pio_iss.A_IMEM == (drv.IMEM_BASE, drv.IMEM_BASE + drv.IMEM_STRIDE), (
        "engine e's IMEM is at 0x12 + 0x40 e (docs/mvp-regmap.md `### Address map`)"
    )
    assert drv.SM_BASE == pio_iss.A_SM_BASE
    assert (drv.SM_CLKDIV, drv.SM_EXECCTRL, drv.SM_SHIFTCTRL, drv.SM_ADDR,
            drv.SM_INSTR, drv.SM_PINCTRL) == (
        pio_iss.SM_CLKDIV, pio_iss.SM_EXECCTRL, pio_iss.SM_SHIFTCTRL, pio_iss.SM_ADDR,
        pio_iss.SM_INSTR, pio_iss.SM_PINCTRL
    )


def test_the_driver_refuses_what_it_cannot_do() -> None:
    """The limits checked before anything is queued."""
    from fw.spi_target import driver as drv

    with pytest.raises(ValueError):
        SpiTargetConfig(clkdiv=(0, 0))             # INT 0 is 65536 on this chip, not 1
    with pytest.raises(ValueError):
        SpiTargetConfig(push_thresh=33)
    with pytest.raises(ValueError):
        SpiTargetConfig(autopull=1)                # the program refills explicitly
    with pytest.raises(ValueError):
        drv.tx_word(0x100)
    rig = Rig()
    with pytest.raises(ValueError):
        rig.driver.start([0x00] * 300)
    with pytest.raises(ValueError):
        Rig(pins=(0, 1, 1, 3))                     # MOSI and SCK on one pin
    with pytest.raises(ValueError):
        Rig(pins=(0, 1, 2, 8))                     # a line on an input-only pin
    with pytest.raises(ValueError):
        Rig(mode=1)                                # mode 0 only, see the module docstring
    with pytest.raises(ValueError):
        drv.jedec_stream((1, 2, 3, 4))             # a JEDEC ID is three bytes


@pytest.mark.parametrize("pins", [(0, 1, 2, 3), (0, 4, 5, 6), (7, 5, 4, 6), (2, 6, 7, 3),
                                  (5, 7, 0, 6)])
def test_the_four_lines_can_be_on_any_four_pins(pins) -> None:
    """The pin map is a parameter, not a constant: four distinct `uio` pins, anywhere.

    MISO has to be a `uio` pin because the host drives its *direction* with a forced
    `set pindirs` (docs/mvp-isa.md `### Open drain`), and the device must not drive CSn, SCK or
    MOSI.  The two clock lines are free because the program watches them with the absolute
    `gpio` wait source (`program.pio`, "Why IN_BASE = 1"), which is why `pins` is a
    configuration field and not four constants.  The maps below include MOSI and SCK the
    "wrong" way round and the whole 0..7 range.
    """
    data = [(0x5A + i) & 0xFF for i in range(8)]
    rig = Rig(pins=pins)
    assert rig.jedec().miso == [0x00, *JEDEC_ID], rig.report()
    assert rig.read(0x20, data).miso == [0x00] * 4 + data, rig.report()
    rig.check_clean()


def test_re_assembling_the_default_pin_map_changes_nothing() -> None:
    """`program_with_pins` on the shipped map is the shipped program, word for word.

    The pin map is re-assembled per rig so a test can move the four lines, and this is what
    keeps that from being a second copy of the program: the default path must be identical to
    `program.pio` as committed, or the tests would be running a program nobody ships.
    """
    program = assemble_program()
    same = program_with_pins((CS, MOSI, SCK, MISO))
    assert same.instrs == program.instrs, (
        f"default re-assembly differs: {[hex(w) for w in same.instrs]} vs "
        f"{[hex(w) for w in program.instrs]}"
    )
    assert same.labels == program.labels


def test_the_second_engine_can_do_it_too() -> None:
    """The same program on engine 1, with engine 0 left disabled.

    Nothing else needs it, but the addressing is per engine and the cheap way to prove
    the driver got it right is to run it there: IMEM word `k` of engine 1 is at 0x52 + k rather
    than 0x12 + k, `SMe_*` is at 0x32 + 6 e + i, and FLEVEL and FDEBUG give each engine its own
    nibble and byte.  Engine 1 wins every pin conflict (docs/mvp-isa.md `## Pins`), so a chip
    that got this wrong would show both engines driving MISO.
    """
    from fw.spi_target import SpiTargetConfig, SpiTargetDriver

    bus = Bus(ns_per_tick=NS_PER_TICK)
    for net in (CS_N, MOSI_NET, SCK_NET, MISO_NET):
        bus.add_net(net, push_pull=True)
    chip = Chip()
    dev = ChipPins(chip, nets=[CS_N, MOSI_NET, SCK_NET, MISO_NET] +
                   [f"unused{k}" for k in range(4, pio_iss.NUM_PINS)])
    bus.attach(dev)
    ctl = bus.attach(SpiController(mode=0, half_period=SHIPPED_HALF, cs_setup=MIN_TICKS,
                                   cs_hold=MIN_TICKS, cs_gap=MIN_TICKS, name="spi_ctl"))
    mon = bus.attach(SpiMonitor(mode=0, min_sck_ticks=MIN_TICKS, min_cs_setup=MIN_TICKS,
                                min_cs_hold=MIN_TICKS, name="spi_mon"))
    driver = SpiTargetDriver(dev, SpiTargetConfig(), sm=1)
    driver.load()
    while not dev.idle:
        bus.step()
    assert chip.peek(0x00) == 0, "load() must leave both engines disabled"
    driver.start(jedec_stream(), n_rx=4)
    driver.run(bus, MIN_TICKS)
    ctl.add([CMD_JEDEC], 3)
    driver.run(bus, ctl.pending_ticks(bus.t) + 64)
    driver.stop(bus)
    assert mon.last.miso == [0x00, *JEDEC_ID], mon.last.miso
    assert mon.violations == [] and not bus.errors
    assert driver.read_fdebug(bus) == 0
    # Engine 0 never ran: it is still at the reset PC with empty FIFOs.
    assert chip.engines[0].pc == 0
    assert chip.engines[0].tx == [] and chip.engines[0].rx == []


def test_the_rtl_target_runs_the_same_module() -> None:
    """`FW_TARGET=rtl` has to *run* this module, not skip it.

    Under every target except `rtl` this runs pytest a second time with `FW_TARGET=rtl` in the
    environment and requires the module to pass rather than skip, so a model target is what
    notices that the RTL path stopped existing.  It picks the cheap tests of the module to keep
    the guard quick -- the whole RTL run is `make fw PROTO=spi_target TARGET=rtl` -- and only
    skips on `rtl` itself, which is what keeps it from recursing.  `fw/conftest.py` installs the
    target in the child too, and since the tests it picks never step a chip, the child costs a
    re-run of the module and no simulator.
    """
    if targets.current() == "rtl":
        pytest.skip("this check runs under every target but rtl, and would otherwise recurse")
    env = dict(os.environ, FW_TARGET="rtl")
    done = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(Path(__file__).resolve()),
         "-k", "test_the_program_fits_the_instruction_memory or "
               "test_the_driver_addresses_match_the_register_map_and_the_iss"],
        capture_output=True, text=True, env=env,
        cwd=str(Path(__file__).resolve().parents[2]),
    )
    summary = done.stdout.strip().splitlines()[-1] if done.stdout.strip() else ""
    assert done.returncode == 0, done.stdout + done.stderr
    assert "passed" in summary and "skipped" not in summary, (
        f"the FW_TARGET=rtl path must run this module, not skip it: {summary}")

# SPDX-License-Identifier: Apache-2.0
#
# Cocotb bench for `host_spi` (`make host-test`).
#
# What is under test: `host_spi` and nothing else. The chip side of the register bus
# is `regfile_model.RegFile`, a dict plus one FIFO-pop address, and the host side is
# `verif.vip.spi.SpiController` through `verif.vip.cocotb_bridge.Bridge`. `host_spi` is
# supposed to be a pure transport between the two, so almost every check here is of the
# form "these bits on the wire in, that one bus access, those bits back out".
#
# Spec: `docs/mvp-regmap.md` `## SPI transaction format` and
# `docs/mvp-interfaces.md` `## RTL ports` / `## Register bus`.
#
# -------------------------------------------------------------------------------------
# The one number in this file that is not read off a wire: RISE_TO_ACCESS = 3.
#
# `Bridge` steps one `verif.vip.core.Bus` tick per rising edge of clk. During tick t the
# devices are handed the snapshot that was resolved at the end of tick t, and that same
# snapshot is what the bridge writes onto the DUT's input pins half a cycle later, for
# the DUT to sample at the edge into cycle t + 1. So a device watching the nets and the
# DUT's own cycle counter are looking at the same value in the same cycle: what a device
# reads at tick t is the DUT's input pin in cycle t. From there, the spec does the rest:
#
#   * the port has 2-flop synchronizers on CSn/SCK/MOSI plus one more flop on SCK, so
#     the pin value of cycle t is in `sck_s2` in cycle t + 2, and the `rise` of an SCK
#     0->1 pin edge is detected in cycle n = t + 2 ("the port is oversampled in the core
#     clock"),
#   * "if rise for bit 0 of a data word is detected in cycle n, the write is performed
#     in cycle h = n + 1" (and the same for the first TA bit of a read),
#   * `bus_we`/`bus_re` are outputs, so the bus carries the access in cycle h, which is
#     the cycle the register-file model samples.
#
#   t + 2 + 1 = t + 3.
#
# A device that sees the rise at tick t must see the bus access at tick t + 3. Every
# cycle check in this file goes through that one number, and a wrong number fails loudly
# instead of quietly passing. It is not a fudge factor: changing it fails 15 of the 16
# tests.
#
# `IDLE_MISO_DELAY` is the same argument for the other rule, "MISO ... is 0 while the
# port is idle". sel = !cs_s2, so a CSn rise read at tick q makes the port idle in cycle
# q + 2, the reset of the MISO shift register lands at the end of that cycle, MISO reads
# 0 in cycle q + 3, and a device sees it one tick later because it sees the DUT's
# registered outputs: tick q + 4.
#
# Not asserted here: reads above 6.25 MHz. The spec guarantees reads at f_clk/8 and only
# allows faster ones "if the host's setup margin covers 3T + t_pad", so a read at
# f_clk/4 is measured and reported, not required (see `test_fastest_sck` and
# `test_reads_faster_than_the_spec_guarantees_are_measured`).
# -------------------------------------------------------------------------------------

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Sequence

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parents[1]
for _p in (str(_HERE), str(_ROOT)):          # regfile_model, then verif.vip
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

from verif.vip.cocotb_bridge import Bridge
from verif.vip.core import Device
from verif.vip.spi import (
    CS_ACTIVE,
    CS_IDLE,
    CS_N,
    MISO,
    MOSI,
    SCK,
    SpiController,
    SpiMonitor,
)
from regfile_model import FIFO_ADDR, RegFile, RegFileBus, next_addr

CLK_HZ = 50_000_000
NS_PER_TICK = 1e9 / CLK_HZ                 # T = 20 ns, as docs/mvp-regmap.md says
NETS = (CS_N, SCK, MOSI, MISO)
PIN_MAP = {"cs_n": ("ui_in", 0), "sck": ("ui_in", 1),
           "mosi": ("ui_in", 2), "miso": ("uo_out", 0)}

#: Ticks from the tick a device sees an SCK rise to the tick the bus access happens. See
#: the derivation at the top of this file.
RISE_TO_ACCESS = 3
#: Ticks from the SCK rise that starts a turnaround byte to the tick another device can
#: see the read word in MISO: the access is in cycle rise + RISE_TO_ACCESS, the value is
#: "loaded into the MISO shift register at the end of h", and a device sees a DUT output
#: one tick after the edge that registered it.
LOAD_VISIBLE = RISE_TO_ACCESS + 2
#: Ticks after CSn reads high that MISO must read 0.
IDLE_MISO_DELAY = 4

#: "Each of the following must last at least 2 core cycles" -- SCK high, SCK low,
#: CSn fall to the first SCK rise, the last SCK rise to CSn rise, CSn high between
#: transactions. The monitor measures all of them; nothing may go below this.
MIN_LEVEL_TICKS = 2
#: The rates the spec names, as a half period in core cycles (a bit period is 2*half).
SCK_FASTEST_WRITE_HALF = 2     # f_clk/4 = 12.5 MHz, "writes work up to SCK = f_clk/4"
SCK_FASTEST_READ_HALF = 4      # f_clk/8 = 6.25 MHz, "reads are guaranteed for ... 8"
#: A wait that cannot hang: 20000 ticks is 400 us, a transaction is a few hundred.
TICK_LIMIT = 20_000


# --------------------------------------------------------------------------- the wire


@dataclass
class Window:
    """One CS_N low window, as the wire watcher saw it."""

    index: int = 0
    first_tick: int = 0        #: first tick CS_N read low
    last_tick: int = 0         #: last tick CS_N read low
    end_tick: Optional[int] = None   #: first tick CS_N read high again
    #: (tick, mosi) for every SCK rising edge, in order. Bit k of the transaction is
    #: rises[k].
    rises: list[tuple[int, int]] = field(default_factory=list)


class WireWatcher(Device):
    """Watches the four SPI nets and records where the SCK rises were, and when.

    `SpiMonitor` already decodes the bytes and checks the host timing rules; what it
    does not hand out is the tick of each individual rising edge, which is what the
    cycle checks need. It also records MISO while CS_N is high, to check the idle rule.

    It drives nothing, so it cannot perturb what it watches.
    """

    name = "watch"

    def __init__(self) -> None:
        self.windows: list[Window] = []
        #: (tick, ticks_cs_has_been_high) where MISO was not 0 while it should be 0.
        self.idle_miso_bad: list[tuple[int, int]] = []
        self._window: Optional[Window] = None
        self._sck: Optional[int] = None
        self._cs: Optional[int] = None
        self._cs_high = 0
        self._hist: list[int] = []           # CS_N per tick, so streaks can be measured
        self.last_tick = -1

    def tick(self, t: int, pins) -> dict:
        cs, sck, mosi, miso = (pins[CS_N], pins[SCK], pins[MOSI], pins[MISO])
        self.last_tick = t
        self._hist.append(cs)
        if cs == CS_IDLE:
            self._cs_high += 1
            if self._cs_high >= IDLE_MISO_DELAY and miso:
                self.idle_miso_bad.append((t, self._cs_high))
        else:
            self._cs_high = 0
        if self._cs is None:
            self._cs, self._sck = cs, sck
            return {}
        if cs != self._cs:
            # CS_N moves: a new window starts or the current one closes, and whatever
            # level SCK sits at is not an edge.
            if cs == CS_ACTIVE:
                self._window = Window(index=len(self.windows), first_tick=t,
                                      last_tick=t)
            elif self._window is not None:
                self._window.end_tick = t
                self.windows.append(self._window)
                self._window = None
            self._cs, self._sck = cs, sck
            return {}
        if cs == CS_ACTIVE and sck != self._sck:
            if sck and not self._sck:                # a rising edge
                self._window.rises.append((t, mosi & 1))
                self._window.last_tick = t
            self._sck = sck
        return {}

    def cs_high_streak(self, upto: int = None) -> int:
        """How many ticks in a row CS_N has read high, ending at `upto` (default: last)."""
        if upto is None:
            upto = self.last_tick
        streak = 0
        for value in reversed(self._hist[: upto + 1]):
            if value != CS_IDLE:
                break
            streak += 1
        return streak

    def last_window(self) -> Window:
        assert self.windows, "no CS_N low window was seen yet"
        return self.windows[-1]


# ------------------------------------------------------------------- the golden model


@dataclass
class WordAccess:
    """One bus access the wire says has to happen, and when."""

    kind: str            #: "w" or "r"
    addr: int
    tick: int            #: the tick bus_we/bus_re must be asserted in: the cycle h
    data: int = 0        #: writes: the word; reads: filled in from the model


@dataclass
class Trajectory:
    """What one transaction must do, derived from the spec and the wire it ran on."""

    cmd: int
    mosi: list[int]
    n_miso: int
    window: Window
    writes: list[WordAccess] = field(default_factory=list)
    reads: list[WordAccess] = field(default_factory=list)
    #: expected MISO bit per bit index of the transaction (only sampled bits matter)
    miso_bits: dict[int, int] = field(default_factory=dict)
    #: bit index -> the word a read returned, for the MISO expectation
    read_data: dict[int, int] = field(default_factory=dict)

    @property
    def all_accesses(self) -> list[WordAccess]:
        return sorted(self.writes + self.reads, key=lambda a: a.tick)

    def expected_miso(self) -> list[int]:
        """The MISO bytes the controller must have sampled for this transaction."""
        first = 8 * len(self.mosi)
        bits = [self.miso_bits.get(k, 0) for k in range(first, first + 8 * self.n_miso)]
        return [sum(bits[8 * j + i] << (7 - i) for i in range(8)) for j in range(self.n_miso)]


def int_of_bits(bits: Sequence[int]) -> int:
    """An MSB-first bit list as the integer it spells."""
    return sum((b & 1) << (len(bits) - 1 - i) for i, b in enumerate(bits))


def stream_bytes(mosi: Sequence[int], n_miso: int, fill: int) -> list[int]:
    """The bytes the controller clocks, fill bytes included: what a wire sees."""
    return [int(b) & 0xFF for b in mosi] + [fill & 0xFF] * n_miso


def stream_bits(mosi: Sequence[int], n_miso: int, fill: int) -> list[int]:
    """The bits the controller clocks: `mosi` MSB first, then `n_miso` fill bytes."""
    bits: list[int] = []
    for byte in stream_bytes(mosi, n_miso, fill):
        bits += [(byte >> (7 - i)) & 1 for i in range(8)]
    return bits


def predict(mosi: Sequence[int], n_miso: int, window: Window,
            read_data: Optional[dict] = None) -> Trajectory:
    """Decode one transaction from the spec and the SCK rises the wire actually had.

    Returns the bus accesses the DUT has to make (with the tick each has to happen on)
    and the MISO bit the host has to sample at every bit index.

    `read_data` maps a predicted read tick to the word the register file returned for it.
    The MISO expectation is only complete with it; the accesses do not need it.

    The turnaround byte needs the sample times, and those are the rise ticks:
    `SpiController` in mode 0 samples bit `k` at the tick it sees the rise for bit `k`
    (`sample_offset = half + 1`, and the rise of a bit period is visible at the same
    offset). The word is not in MISO until LOAD_VISIBLE ticks after that first rise, so
    the turnaround bits sampled before then are the *old* MISO -- 0, because the previous
    word shifted all 32 bits out -- and the rest are bit 31. That is the spec's "MOSI
    ignored, MISO undefined-but-deterministic" together with "During a TA byte, it shows
    bit 31 of the word being read once that word is loaded".
    """
    read_data = read_data or {}
    traj = Trajectory(cmd=mosi[0], mosi=list(mosi), n_miso=n_miso, window=window)
    rises = window.rises
    total = 8 * (len(mosi) + n_miso)
    bits = stream_bits(mosi, n_miso, 0)
    is_read = bool(mosi[0] & 0x80)
    addr = mosi[0] & 0x7F
    pos = 8                       # bits of the command byte are behind us

    while pos < total:
        if pos >= len(rises):
            break                 # not enough edges to have clocked this bit yet
        if not is_read:
            if pos + 32 > total:
                break             # a partial word: "that partial word is discarded"
            word = int_of_bits(bits[pos: pos + 32])
            traj.writes.append(WordAccess("w", addr, rises[pos + 31][0] + RISE_TO_ACCESS,
                                          word))
            for k in range(pos, pos + 32):
                traj.miso_bits[k] = 0          # "MISO ... is 0 ... during writes"
            addr = next_addr(addr)
            pos += 32
        else:
            if pos + 8 > total:
                break             # not even a whole turnaround byte
            tick = rises[pos][0] + RISE_TO_ACCESS
            data = read_data.get(tick)
            loaded = rises[pos][0] + LOAD_VISIBLE
            top = ((data >> 31) & 1) if data is not None else 0
            for i in range(8):
                k = pos + i
                if k >= total or k >= len(rises):
                    break
                # Bit 31 of the word, from the moment the word is in the register; the
                # bits the host samples before that are the old register, which is 0.
                traj.miso_bits[k] = top if rises[k][0] >= loaded else 0
            traj.reads.append(WordAccess("r", addr, tick))
            if data is not None:
                traj.read_data[tick] = data
            addr = next_addr(addr)
            start = pos + 8
            stop = min(start + 32, total)
            for k in range(start, stop):
                if data is None:
                    traj.miso_bits[k] = 0
                else:
                    traj.miso_bits[k] = (data >> (31 - (k - start))) & 1
            pos = stop
    for k in range(8):            # the command byte
        traj.miso_bits[k] = 0     # "MISO ... is 0 ... during CMD"
    return traj


# ------------------------------------------------------------------------------ bench


@dataclass
class Result:
    """Everything one transaction produced, from the wire and from the model."""

    traj: Trajectory
    accesses: list = field(default_factory=list)     # Access, as the model saw them
    rx: list = field(default_factory=list)           # the MISO bytes sampled
    monitored: object = None                         # SpiMonitor's decode


def cmd_byte(addr: int, read: bool = False) -> int:
    """`CMD = { RW, ADDR[6:0] }`."""
    return (0x80 if read else 0x00) | (addr & 0x7F)


def word_bytes(word: int) -> list[int]:
    """A 32-bit word as 4 bytes on the wire: MSB first (big-endian)."""
    return [(word >> (24 - 8 * i)) & 0xFF for i in range(4)]


_logged = False


def log_setup_once() -> None:
    """Put the bench's timing constants and the compiled module in the log, once."""
    global _logged
    if _logged:
        return
    _logged = True
    cocotb.log.info(
        "host-test: compiled %s, T=%.0f ns, RISE_TO_ACCESS=%d, LOAD_VISIBLE=%d, "
        "IDLE_MISO_DELAY=%d, MIN_LEVEL_TICKS=%d",
        os.environ.get("HOST_DUT", "?"), NS_PER_TICK, RISE_TO_ACCESS, LOAD_VISIBLE,
        IDLE_MISO_DELAY, MIN_LEVEL_TICKS)


class Bench:
    """Bridge + SpiController + SpiMonitor + WireWatcher + the register-file model.

    One instance per test: it resets the DUT, then every transaction it runs is checked
    against the spec by :meth:`check_transfer`, so a test only has to assert the things
    that are about *its* transaction.
    """

    def __init__(self, dut, half: int = 4, mode: int = 0, fill: int = 0x00,
                 cs_setup: int = MIN_LEVEL_TICKS, cs_hold: int = MIN_LEVEL_TICKS,
                 cs_gap: int = MIN_LEVEL_TICKS, fifo_addr: int = FIFO_ADDR):
        self.dut = dut
        self.half = half
        self.fill = fill
        self.rf = RegFile(fifo_addr=fifo_addr)
        self.bridge = Bridge(dut, PIN_MAP, ns_per_tick=NS_PER_TICK)
        for net in NETS:
            self.bridge.set_net(net, push_pull=True)
        self.ctl = SpiController(mode=mode, half_period=half, cs_setup=cs_setup,
                                 cs_hold=cs_hold, cs_gap=cs_gap, fill=fill)
        self.mon = SpiMonitor(mode=mode, min_sck_ticks=MIN_LEVEL_TICKS,
                              min_cs_setup=MIN_LEVEL_TICKS, min_cs_hold=MIN_LEVEL_TICKS)
        self.watch = WireWatcher()
        for device in (self.ctl, self.mon, self.watch):
            self.bridge.attach(device)
        self.rfbus = RegFileBus(dut, self.rf, cycle_of=lambda: self.bridge.bus.t)
        self._tasks = []

    # -- lifecycle ---------------------------------------------------------------

    async def start(self) -> None:
        log_setup_once()
        dut = self.dut
        dut.ena.value = 1
        dut.uio_in.value = 0
        dut.ui_in.value = 0x00        # CS_N high: the port is idle
        dut.bus_rdata.value = 0
        self._tasks.append(cocotb.start_soon(Clock(dut.clk, NS_PER_TICK, unit="ns").start()))
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 4)
        self._tasks.append(cocotb.start_soon(self.bridge.start()))
        self._tasks.append(cocotb.start_soon(self.rfbus.run()))
        dut.rst_n.value = 1
        await ClockCycles(dut.clk, 8)
        # The reset itself is not a transaction; start the log at the first one.
        self.rf.accesses.clear()
        self.rf.cycle = self.bridge.bus.t

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        self._tasks = []

    async def idle_wait(self, cycles: int = 20) -> None:
        """Ticks of nothing happening, so the idle rules get checked there too."""
        await ClockCycles(self.dut.clk, cycles)

    async def wait_idle(self, limit: int = TICK_LIMIT) -> int:
        """Ticks until the controller has driven everything it was queued."""
        waited = 0
        while not self.ctl.idle():
            await ClockCycles(self.dut.clk, 1)
            waited += 1
            if waited > limit:
                raise AssertionError(
                    f"the controller never went idle in {limit} ticks; "
                    f"{len(self.ctl.queue)} transactions left")
        return waited

    # -- transactions -----------------------------------------------------------

    async def transfer(self, mosi: Sequence[int], n_miso: int = 0,
                       check: bool = True) -> Result:
        """Run one transaction (one CS_N low window) and check it against the spec."""
        mosi = [int(b) & 0xFF for b in mosi]
        n_window = len(self.watch.windows)
        n_access = len(self.rf.accesses)
        n_rx = len(self.ctl.rx)
        self.ctl.add(mosi, n_miso)
        await self.wait_idle()
        await ClockCycles(self.dut.clk, 4)
        assert len(self.watch.windows) == n_window + 1, (
            f"expected one CS_N low window, got {len(self.watch.windows) - n_window}")

        window = self.watch.windows[n_window]
        reads = {a.cycle: a.rdata for a in self.rf.accesses[n_access:] if a.kind == "r"}
        traj = predict(mosi, n_miso, window, read_data=reads)
        res = Result(traj=traj,
                     accesses=self.rf.accesses[n_access:],
                     rx=self.ctl.rx[n_rx: n_rx + n_miso],
                     monitored=self.mon.transfers[-1])
        if check:
            self.check_transfer(res)
        return res

    async def write(self, addr: int, *words: int, check: bool = True) -> Result:
        """`write: CMD | W0 | W1 | ...` in one transaction."""
        mosi = [cmd_byte(addr)] + [b for w in words for b in word_bytes(w)]
        return await self.transfer(mosi, 0, check=check)

    async def read(self, addr: int, n: int = 1, check: bool = True) -> Result:
        """`read: CMD | TA | R0 | ...` in one transaction, `n` words.

        The controller clocks `fill` (0x00) on MOSI after the command, which is what the
        spec's `xx` bytes are, and samples 5*n MISO bytes: each turnaround byte followed
        by its word.
        """
        return await self.transfer([cmd_byte(addr, read=True)], 5 * n, check=check)

    # -- the checks every transaction gets --------------------------------------

    def check_transfer(self, res: Result) -> None:
        traj, window = res.traj, res.traj.window
        where = (f"transaction {window.index} cmd={traj.cmd:#04x} "
                 f"mosi={[hex(b) for b in traj.mosi]} n_miso={traj.n_miso}")

        # 1. Every clocked bit is one rising edge, and MOSI is what we meant to send at
        #    it. (If this fails, the stimulus itself is wrong and nothing below means
        #    anything, so it comes first.)
        bits = stream_bits(traj.mosi, traj.n_miso, self.fill)
        assert len(window.rises) == len(bits), (
            f"{where}: {len(window.rises)} SCK rises for {len(bits)} clocked bits; "
            f"rises at ticks {[t for t, _ in window.rises]}")
        for k, (tick, mosi) in enumerate(window.rises):
            assert mosi == bits[k], (
                f"{where}: MOSI at the rise of bit {k} (tick {tick}) is {mosi}, "
                f"wanted {bits[k]}; the host is not holding MOSI across the rise")

        # 2. The bus accesses, on exactly the cycles the spec puts them.
        got = [(a.cycle, a.kind, a.addr, a.wdata) for a in res.accesses]
        want = [(a.tick, a.kind, a.addr, a.data if a.kind == "w" else 0)
                for a in traj.all_accesses]
        assert got == want, (
            f"{where}: bus accesses differ.\n"
            f"  got  (cycle, kind, addr, wdata): {got}\n"
            f"  want (cycle, kind, addr, wdata): {want}\n"
            f"  SCK rise ticks: {[t for t, _ in window.rises]}\n"
            f"  a write must land in cycle h = n + 1 with n the cycle of the rise for bit 0 "
            f"of the word, a read in cycle h = n + 1 with n the cycle of the first TA bit "
            f"rise, and a core.py device sees the pin {RISE_TO_ACCESS} ticks before that")

        # 3. The MISO bytes the host sampled, turnaround bytes included.
        assert res.rx == traj.expected_miso(), (
            f"{where}: MISO bytes sampled {res.rx} are not the spec's "
            f"{traj.expected_miso()} "
            f"(reads returned {[(a.cycle, hex(a.rdata)) for a in res.accesses if a.kind == 'r']})")

        # 4. The independent decode by SpiMonitor. It samples on every capture edge, so
        #    it decodes the whole frame: MOSI is the command byte plus the fill bytes,
        #    and MISO is one byte for the command byte (which the port holds at 0) plus
        #    the bytes the controller sampled. Two different sampling instants landing on
        #    the same bytes is what makes this a check and not a copy.
        if res.monitored is not None:
            wire_mosi = stream_bytes(traj.mosi, traj.n_miso, self.fill)
            assert res.monitored.mosi == wire_mosi, (
                f"{where}: the monitor decoded MOSI {res.monitored.mosi} on the wire, "
                f"which carried {wire_mosi}")
            wire_miso = [0x00] * len(traj.mosi) + res.rx
            assert res.monitored.miso == wire_miso, (
                f"{where}: the monitor decoded MISO {res.monitored.miso}, which is "
                f"{wire_miso} with a zero for every byte the port holds MISO low on "
                f"(the command byte and any MOSI bytes)")

    def check_bench(self) -> None:
        """Invariants that are about the DUT as a whole, not one transaction."""
        assert not self.mon.violations, (
            f"the host broke the timing rules of the spec: {self.mon.violations}")
        assert not self.watch.idle_miso_bad, (
            f"MISO was not 0 while the port was idle at ticks "
            f"{self.watch.idle_miso_bad[:8]}")
        assert not self.rf.errors, self.rf.errors
        assert not self.bridge.contentions, self.bridge.contentions
        # "While sel = 0, the port is idle and is reset at the end of the cycle": with
        # CS_N read high for 4 ticks, sel could not have been set in that cycle, so a
        # bus access there would be the port talking while idle.
        stray = [str(a) for a in self.rf.accesses
                 if self.watch.cs_high_streak(a.cycle) >= 4]
        assert not stray, f"bus access while CS_N was high: {stray}"


# ------------------------------------------------------------------------------ tests

#: The registers the spec's own examples use, from `### Address map`.
CTRL, FSTAT, FDEBUG, TXF0 = 0x00, 0x01, 0x02, 0x04
RXF0, RXF1 = 0x08, 0x09
IMEM0 = 0x12            # engine 0's INSTR_MEM0
RESERVED = 0x51         # "reserved": legal for a transport to move, never decoded


async def bench(dut, **kwargs) -> Bench:
    """A started bench: `b = await bench(dut, half=2)`."""
    b = Bench(dut, **kwargs)
    await b.start()
    return b


async def with_bench(dut, coro: Callable[[Bench], object], **kwargs):
    """Run `coro(bench)` and always tear the bench down."""
    b = await bench(dut, **kwargs)
    try:
        return await coro(b)
    finally:
        await b.stop()


def split_read(res: Result, n: int) -> list[dict]:
    """The bytes of an `n`-word read: turnaround byte, then the word."""
    out = []
    for j in range(n):
        ta = res.rx[5 * j]
        word = int.from_bytes(bytes(res.rx[5 * j + 1: 5 * j + 5]), "big")
        out.append({"ta": ta, "word": word})
    return out


@cocotb.test()
async def test_reset_idle_and_no_stray_accesses(dut):
    """After reset the port is idle: MISO 0, no bus traffic, and a host can start."""
    b = await bench(dut)
    try:
        await b.idle_wait(200)
        assert not b.rf.accesses, b.rf.free
        assert int(dut.uo_out.value) & 1 == 0, "MISO is not 0 while idle"
        # A read of a word with bit 31 clear: the turnaround byte must be 0x00 too.
        await b.read(FSTAT)
        await b.idle_wait(50)
        b.check_bench()
    finally:
        await b.stop()


@cocotb.test()
async def test_spec_example_enable_and_restart(dut):
    """Spec example: "Enable engine 0 and restart its divider: 00 | 00 00 01 01"."""
    async def body(b):
        res = await b.transfer([0x00, 0x00, 0x00, 0x01, 0x01])
        assert res.traj.writes[0].addr == CTRL, res.traj.writes
        assert res.traj.writes[0].data == 0x0000_0101, hex(res.traj.writes[0].data)
        # SM_ENABLE[0] = 1, SM_RESTART[0] = 1 (bits 5:4), CLKDIV_RESTART[0] = 1 (9:8)
        assert b.rf.regs[CTRL] == 0x0000_0101, hex(b.rf.regs[CTRL])
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_spec_example_imem_burst(dut):
    """Spec example: "Load IMEM 0..2 of engine 0: 12 | 00 00 E0 81 | ... | 00 00 E0 00".

    Three words in one transaction, so this is also the auto-increment case.
    """
    async def body(b):
        res = await b.transfer([0x12, 0x00, 0x00, 0xE0, 0x81,
                                0x00, 0x00, 0xE0, 0x01,
                                0x00, 0x00, 0xE0, 0x00])
        assert [(w.addr, w.data) for w in res.traj.writes] == [
            (IMEM0 + 0, 0x0000_E081), (IMEM0 + 1, 0x0000_E001),
            (IMEM0 + 2, 0x0000_E000)], res.traj.writes
        for k, word in enumerate((0x0000_E081, 0x0000_E001, 0x0000_E000)):
            assert b.rf.regs[IMEM0 + k] == word, hex(b.rf.regs[IMEM0 + k])
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_spec_example_read_fstat(dut):
    """Spec example: "Read FSTAT (word address 0x01): 81 | xx | xx xx xx xx".

    FSTAT resets to 0x0300_0300, so this has 1 bits to shift out, and MISO has to be 0
    for the command byte and for the turnaround byte (bit 31 of FSTAT is 0).
    """
    async def body(b):
        res = await b.read(FSTAT, 1)
        assert res.rx[0] == 0x00, f"turnaround byte {res.rx[0]:#04x}, wanted 0"
        assert int.from_bytes(bytes(res.rx[1:5]), "big") == 0x0300_0300, res.rx
        # The burst address moved on: a second word of this transaction is FDEBUG.
        assert res.traj.reads[0].addr == FSTAT
        assert next_addr(res.traj.reads[0].addr) == FDEBUG, hex(res.traj.reads[0].addr)
        assert b.rf.regs[FSTAT] == 0x0300_0300
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_spec_example_pop_three_rxf1(dut):
    """Spec example: "Pop three words from RXF1 (word address 0x09)".

    89 | xx | 4 bytes | xx | 4 bytes | xx | 4 bytes. The model is pointed at 0x09 so
    that 0x09 is the address whose reads pop; the FIFO data registers hold the burst
    address still, so all three reads are of 0x09.
    """
    words = [0x1111_1111, 0x8000_0001, 0x0BAD_F00D]

    async def body(b):
        b.rf.push(*words)
        res = await b.read(RXF1, 3)
        assert [r.addr for r in res.traj.reads] == [RXF1] * 3, res.traj.reads
        got = split_read(res, 3)
        assert [g["word"] for g in got] == words, got
        # "During a TA byte, it shows bit 31 of the word being read once that word is
        # loaded." The host samples a turnaround bit at the tick it sees that bit's SCK
        # rise, and the word is not in MISO until LOAD_VISIBLE = 5 ticks later, so the
        # first sampled bit of a turnaround byte is still the old register (0) and the
        # other seven are bit 31: 0b0111_1111, not 0xff. For bit 31 clear the whole byte
        # is 0, which is the other two words here.
        assert [g["ta"] for g in got] == [0x00, 0x7F, 0x00], got
        assert b.rf.fifo == [], "the model popped fewer than three words"
        assert b.rf.n_pops == 3, b.rf.n_pops
        b.check_bench()
    await with_bench(dut, body, fifo_addr=RXF1)


@cocotb.test()
async def test_read_burst_auto_increment(dut):
    """A read burst steps through the map, and stops stepping at the FIFO registers."""
    async def body(b):
        for addr in (0x40, 0x41, 0x42, 0x7F):
            b.rf.poke(addr, 0xA000_0000 | addr)
        res = await b.read(0x40, 3)
        assert [r.addr for r in res.traj.reads] == [0x40, 0x41, 0x42], res.traj.reads
        words = [g["word"] for g in split_read(res, 3)]
        assert words == [0xA000_0040, 0xA000_0041, 0xA000_0042], [hex(w) for w in words]
        # Starting at 0x7F the address wraps mod 128 to 0x00.
        res = await b.read(0x7F, 2)
        assert [r.addr for r in res.traj.reads] == [0x7F, 0x00], res.traj.reads
        words = [g["word"] for g in split_read(res, 2)]
        assert words == [0xA000_007F, 0x0000_0000], [hex(w) for w in words]
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_write_burst_address_holds_on_fifo_registers(dut):
    """"The exception is the FIFO data registers (0x04-0x0B), where the address stays put."

    So a burst write of four words at TXF0 leaves all four in 0x04 and touches nothing
    else, which is what streams a word into TXF0.
    """
    words = [0x0000_0001, 0x0000_0002, 0x0000_0003, 0x0000_0004]

    async def body(b):
        res = await b.write(TXF0, *words)
        assert [w.addr for w in res.traj.writes] == [TXF0] * 4, res.traj.writes
        assert b.rf.regs[TXF0] == words[-1], hex(b.rf.regs[TXF0])
        for addr in (0x05, 0x06, 0x07, 0x0A, 0x0B):
            assert addr not in b.rf.regs, f"a burst write touched 0x{addr:02x}"
        # One word short of the FIFO range: it does step.
        res = await b.write(0x03, 0xAAAA_AAAA, 0xBBBB_BBBB)
        assert [w.addr for w in res.traj.writes] == [0x03, 0x04], res.traj.writes
        assert b.rf.regs[0x03] == 0xAAAA_AAAA and b.rf.regs[0x04] == 0xBBBB_BBBB
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_read_returns_the_state_of_its_own_cycle(dut):
    """A read returns the register's state in cycle h, not later.

    The model replaces the word at the *end* of the read cycle, the way an engine
    pushing into the RX FIFO would. The DUT has already taken its answer for cycle h, so
    the burst has to return the old word anyway -- if it kept sampling bus_rdata while
    the data bits went out, the second word would come back wrong.
    """
    first, second = 0x0BAD_C0DE, 0xFEED_FACE
    pushed = 0x5A5A_5A5A

    async def body(b):
        b.rf.push(first, second)
        fired = []

        def hook(cycle):
            # Runs at the end of every cycle; fire once, in the cycle the first pop
            # lands in, which is the end of the first read's cycle h.
            if b.rf.n_pops == 1 and not fired:
                fired.append(cycle)
                b.rf.fifo.append(pushed)

        b.rf.on_cycle_end = hook
        res = await b.read(RXF0, 2)
        b.rf.on_cycle_end = None
        words = [g["word"] for g in split_read(res, 2)]
        assert words == [first, second], (
            f"got {[hex(w) for w in words]}, wanted {[hex(w) for w in (first, second)]} "
            f"-- a read must return the state of its own cycle h")
        assert len(fired) == 1, fired
        assert b.rf.fifo == [pushed], [hex(w) for w in b.rf.fifo]
        b.check_bench()
    await with_bench(dut, body, fifo_addr=RXF0)


@cocotb.test()
async def test_long_burst_crossing_the_fifo_range(dut):
    """A burst with no length limit, and the increment rule where it changes.

    Ten words from 0x01: the address steps 0x01, 0x02, 0x03 and then sits on 0x04, which
    is inside the FIFO data register range.
    """
    words = [0x1000_0000 + i for i in range(10)]

    async def body(b):
        res = await b.write(FSTAT, *words)
        assert [w.addr for w in res.traj.writes] == (
            [0x01, 0x02, 0x03] + [0x04] * 7), [hex(w.addr) for w in res.traj.writes]
        assert [w.data for w in res.traj.writes] == words
        assert b.rf.regs[0x02] == words[1] and b.rf.regs[0x03] == words[2]
        assert b.rf.regs[0x04] == words[-1], hex(b.rf.regs[0x04])
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_cs_n_abort_mid_word(dut):
    """"If CSn rises during a word, that partial word is discarded."

    Also "A transaction with no data words has no effect": a command byte on its own
    does nothing, and the port is ready for the next transaction because "while sel = 0
    the port is idle and is reset at the end of the cycle".
    """
    async def body(b):
        b.rf.poke(CTRL, 0x1234_5678)
        # A command byte and two of the four bytes of its word.
        await b.transfer([0x00, 0xDE, 0xAD])
        assert b.rf.regs[CTRL] == 0x1234_5678, hex(b.rf.regs[CTRL])
        # A command byte and nothing else.
        await b.transfer([0x00])
        assert b.rf.regs[CTRL] == 0x1234_5678, hex(b.rf.regs[CTRL])
        # A whole word now: the port did restart.
        await b.write(CTRL, 0x0000_00FF)
        assert b.rf.regs[CTRL] == 0x0000_00FF, hex(b.rf.regs[CTRL])
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_cs_n_abort_during_a_turnaround_byte(dut):
    """A read that already happened is not undone: the popped word is lost.

    "If the host ends the transaction during a TA byte or during a word, a read that has
    already happened is not undone: an RXFn word popped that way is lost." The command
    byte and the turnaround byte are clocked and nothing else, so the read fires on the
    first turnaround bit and the transaction ends before any data bit goes out.
    """
    word = 0x8000_0001

    async def body(b):
        b.rf.push(word)
        res = await b.transfer([cmd_byte(RXF0, read=True)], 1)
        assert len(res.traj.reads) == 1 and res.traj.reads[0].addr == RXF0, res.traj.reads
        # Bit 31 of the word for the seven turnaround bits the host samples after the
        # word lands in MISO; the first one is still the old register, which is 0.
        assert res.rx == [0x7F], res.rx
        assert b.rf.fifo == [], f"the word should be lost, {b.rf.fifo} is left"
        assert b.rf.n_pops == 1
        b.check_bench()
    await with_bench(dut, body, fifo_addr=RXF0)


@cocotb.test()
async def test_cs_n_abort_mid_read_word(dut):
    """A read cut in half: the word that was read came out as far as it was clocked."""
    word = 0x1122_3344

    async def body(b):
        b.rf.push(word)
        # Command, turnaround, then two of the four data bytes, and CS_N rises: 3 bytes
        # of fill, which is exactly one turnaround byte and half a word.
        res = await b.transfer([cmd_byte(RXF0, read=True)], 3)
        assert len(res.traj.reads) == 1, res.traj.reads
        assert res.rx == [0x00, 0x11, 0x22], res.rx
        assert b.rf.fifo == []
        b.check_bench()
    await with_bench(dut, body, fifo_addr=RXF0)


@cocotb.test()
async def test_miso_is_zero_while_idle_and_on_the_bus(dut):
    """MISO is 0 while the port is idle, during CMD and during writes.

    Idle is checked over 200 idle ticks after a transaction (the watcher samples MISO
    every tick while CS_N is high); the command byte and the write words are checked by
    the byte comparison in `check_transfer`, which wants 0x00 for every sampled byte of
    a write transaction.
    """
    async def body(b):
        await b.write(CTRL, 0x0F0F_0F0F, 0xF0F0_F0F0)
        await b.idle_wait(200)
        assert not b.watch.idle_miso_bad, b.watch.idle_miso_bad[:8]
        # A read brings 1 bits out; MISO must be 0 again everywhere else.
        b.rf.poke(RESERVED, 0xFFFF_FFFF)
        res = await b.read(RESERVED, 1)
        assert res.rx == [0x7F, 0xFF, 0xFF, 0xFF, 0xFF], res.rx
        await b.idle_wait(100)
        b.check_bench()
    await with_bench(dut, body)


@cocotb.test()
async def test_spi_mode_is_mode_0(dut):
    """Mode 0 works, and mode 3 does not.

    The spec fixes "SPI mode 0 (CPOL 0, CPHA 0)" with no mode pins, so a transfer in
    mode 3 (CPOL 1, CPHA 0: same phase, opposite idle level) must not read back a word
    the host can recognise.
    """
    planted = 0x8000_0001

    async def body(b):
        b.rf.poke(RESERVED, planted)
        res = await b.read(RESERVED, 1)
        assert int.from_bytes(bytes(res.rx[1:5]), "big") == planted, res.rx
        b.check_bench()
    await with_bench(dut, body)

    async def body3(b):
        b.rf.poke(RESERVED, planted)
        res = await b.read(RESERVED, 1, check=False)
        word = int.from_bytes(bytes(res.rx[1:5]), "big")
        assert word != planted, (
            f"mode 3 read back the planted word {word:#010x}; the port is specified as "
            f"mode 0 only, so it must not follow mode 3")
        cocotb.log.info("mode 3 read of 0x%02x returned %#010x (planted %#010x)",
                        RESERVED, word, planted)
    await with_bench(dut, body3, mode=3)


@cocotb.test()
async def test_fastest_sck(dut):
    """The two rates the spec names: reads at f_clk/8, writes at f_clk/4.

    "MISO changes at most 3 core cycles (plus the output pad delay) after an SCK rise,
    so reads are guaranteed for SCK periods of at least 8 core cycles (6.25 MHz). Faster
    reads are allowed if the host's setup margin covers 3T + t_pad." and "writes work up
    to SCK = f_clk/4 (12.5 MHz)".
    """
    async def body(b):
        b.rf.push(0x8000_0001)
        res = await b.read(RXF0, 2)
        assert [g["word"] for g in split_read(res, 2)] == [0x8000_0001, 0], res.rx
        b.check_bench()
        cocotb.log.info("read burst at SCK %d ticks = f_clk/%d, the rate the spec "
                        "guarantees", 2 * b.half, 2 * b.half)
    await with_bench(dut, body, half=SCK_FASTEST_READ_HALF)

    async def body_w(b):
        res = await b.write(CTRL, 0x0000_0301, 0x0000_0402, 0x0000_0503)
        assert [w.addr for w in res.traj.writes] == [CTRL, FSTAT, FDEBUG], res.traj.writes
        assert b.rf.regs[FDEBUG] == 0x0000_0503, hex(b.rf.regs[FDEBUG])
        b.check_bench()
        cocotb.log.info("write burst at SCK %d ticks = f_clk/%d, the rate the spec "
                        "guarantees", 2 * b.half, 2 * b.half)
    await with_bench(dut, body_w, half=SCK_FASTEST_WRITE_HALF)


@cocotb.test()
async def test_reads_faster_than_the_spec_guarantees_are_measured(dut):
    """Not a requirement: measure a read at f_clk/4 and report it.

    The spec guarantees reads only up to f_clk/8, so a conforming port is free to fail
    this. The result goes in the log either way, which is what makes the number worth quoting.
    """
    planted = 0x8000_0001

    async def body(b):
        b.rf.poke(RESERVED, planted)
        res = await b.read(RESERVED, 1, check=False)
        word = int.from_bytes(bytes(res.rx[1:5]), "big")
        cocotb.log.info(
            "read at SCK %d ticks (f_clk/4): turnaround byte %#04x, word %#010x, "
            "planted %#010x -> %s", 2 * b.half, res.rx[0], word, planted,
            "matches" if word == planted else "does NOT match (allowed: the spec only "
            "guarantees reads down to f_clk/8)")
    await with_bench(dut, body, half=SCK_FASTEST_WRITE_HALF)



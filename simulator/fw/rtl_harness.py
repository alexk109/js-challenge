"""The firmware tests on the RTL: `pio_chip` behind the ISS's `ChipDevice` interface.

With `FW_TARGET=rtl` every firmware test in `fw/` runs the
*same* program, the *same* driver and the *same* protocol checks against `src/pio_chip.sv`
instead of `iss/pio_iss.Chip`: `install()` swaps `iss.pio_iss.Chip` for `RtlChip`, which
implements the `Chip` API of `docs/mvp-interfaces.md` `## ISS API` by talking to a real
simulator over a pair of pipes, and the test files then keep using the real
`iss.pio_iss.ChipDevice` (unmodified) on the real `verif.vip.core.Bus` with the real VIPs.
Nothing in `src/`, `iss/`, `verif/vip/` or `fw/*/driver.py` is touched, and no check is
relaxed: the only thing that changes is which chip answers the bus.

    BENCH_CONFIG='{"n_engines":3,"fifo_depth":8}' FW_TARGET=rtl python -m pytest -q fw/spi   # a parametric chip
    FW_TARGET=rtl python -m pytest -q fw/spi          # the RTL
    FW_TARGET=iss python -m pytest -q fw/spi          # the ISS, unchanged
    python -m fw.rtl_harness --selftest               # cycles/s of this plumbing

The pieces, and why each one is where it is:

* **The clock, the pins and the bus are driven by the simulator**, one `verif.vip.core.Bus`
  tick per rising edge of `dut.clk`, exactly as `verif/fuzz/lockstep.py` drives the same
  design: inputs are written at the falling edge, `bus_rdata` is read in `ReadOnly` (it is
  combinational in the access cycle, `docs/mvp-interfaces.md` `## Register bus`), and the
  registered outputs are sampled after the rising edge. `lockstep.py` is also where the
  visibility flag (`--public-flat-rd`) comes from, and `fw/Makefile` mirrors its build.
* **Pin resolution stays in the test process.** The Bus, the VIPs and the two-phase
  resolution of `verif/vip/core.py` are the ISS's own code path, so a net that the chip and a
  device drive with opposite values is recorded as a `Contention` here exactly as it is on the
  ISS. The simulator only ever sees the 18-bit `pin_in` the `ChipDevice` computed, and the chip
  feeds back its own level on pins 13..17 itself (`docs/mvp-isa.md` `### The 18 protocol pins`).
* **Every value the tests read is the RTL's own.** `pin_out`, `pin_oe`, `irq0`, `irq1` come
  back with every cycle. `peek()`, `state()`, `engines[e].imem` and the FIFO contents are read
  out of `g_eng[e].u_eng` and the chip's registers (`docs/mvp-interfaces.md`
  `### Lockstep visibility`) when something asks for them, not modelled: `test_..._loaded_what_was_assembled`
  has to see the words the RTL really holds in its IMEM.
* **`Chip.peek()` is computed in the test process** from that RTL snapshot, by the read mux of
  `docs/mvp-regmap.md`. Poking `bus_addr` mid-simulation would either move a real core cycle or
  leave a stray read on the bus, and `peek()` must be free: it is a register read, not a cycle.
  To keep that honest the harness checks the two against each other: every host read of a cycle
  must return the RTL's own `bus_rdata`, and the child's first act, while the chip is still in
  reset, is to compare the mux with the RTL's combinational read path for all 128 addresses.
  A disagreement is a harness error and stops the test.

The cost is one pipe round trip and four simulator triggers per cycle, which is why `make fw
PROTO=... TARGET=rtl` is slower than the ISS path and why the ISS path is untouched.

Environment (read inside the simulator process):

| Variable | Meaning |
|---|---|
| `FW_RTL_CMD_FD` / `FW_RTL_RESP_FD` | the two pipe fds the parent passes down; both must be set |
| `FW_RTL_VIS` | the visibility flag this build got (from `fw/Makefile`), reported by `--selftest` |
| `BENCH_CONFIG` | JSON of a bench config (the variable the parametric ISS reads too): `n_engines`, `imem_words`, `fifo_depth`, `engine_route`, `t_in` select the RTL build (`tools/bench/rtlparams.py`, `fw/Makefile`; one `sim_build` directory per parameter set) and the shapes this harness reads (engine count, IMEM words, FIFO depth, register map `tools/bench/sim/REGMAP.md`). Unset: the MVP |

The parent's knobs are `FW_RTL_SIM` (default `verilator`), `FW_RTL_LOG` (where the simulator's
own output goes, default `fw/sim_build/<sim>-fw.log`) and `FW_RTL_CYCLES` (a hard cap on the
cycles one chip may run, so a runaway test fails instead of hanging).
"""
from __future__ import annotations

import atexit
import fcntl
import json
import os
import struct
import subprocess
import sys
import time
import weakref
from pathlib import Path
from typing import Mapping, Optional

import cocotb

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from iss import pio_iss as _pio_iss                       # noqa: E402

#: `iss.pio_iss.Chip` as it is before `install()` swaps the name: the harness compares itself
#: with the golden model, and must compare with the real thing.
_GOLDEN_CHIP = _pio_iss.Chip

#: Gate level (fw/Makefile GL_NETLIST, fw/gl_wrap.py, tools/bench/backends/gl_rig.py): the DUT is the
#: post-layout netlist behind `gl_top`. Same cycle protocol, three differences: a host read's data is
#: sampled after its edge (from host_spi's capture register, see fw/gl_wrap.py), the netlist has no
#: internal signals to read (so no read-mux check, no `state()`, and `peek()` answers FDEBUG only), and
#: the reset sequence is longer by the top level's two-flop reset synchronizer and host_spi's CS
#: synchronizer.
GL = bool(os.environ.get("FW_RTL_GL"))

#: One tick is one rising edge of `dut.clk`; `verif/vip/core.Bus` is at 20 ns per tick (50 MHz).
CLOCK_NS = 20.0
from tools.bench.sim import pio_iss_p as _pio_iss_p      # noqa: E402

#: The build's structural parameters, from env BENCH_CONFIG (MVP when unset): `src/` has
#: NUM_ENGINES engines, IMEM_SIZE words each and FIFO_DEPTH words per FIFO. The MVP is 2 / 32 / 4.
PARAMS = _pio_iss_p.Params.from_env()
REGMAP = PARAMS.regmap
NUM_ENGINES = PARAMS.n_engines
IMEM_SIZE = PARAMS.imem_words
FIFO_DEPTH = PARAMS.fifo_depth
#: addresses on the register bus: 128, or 1024 when the map is wide (REGMAP.md section 2)
NUM_ADDRS = REGMAP.addr_space
NUM_PINS = 18

# ---------------------------------------------------------------------------------------
# The wire: fixed-size binary frames on two pipes, so a cycle costs one read and one write.
# ---------------------------------------------------------------------------------------

#: step:  pin_in, bus_we, bus_re, bus_addr (10 bits wide on a wide map), bus_wdata
STEP_CMD = struct.Struct("<IBBHI")
#: answer: pin_out, pin_oe, irq0, irq1, bus_rdata
STEP_RSP = struct.Struct("<IIBBI")

CMD_RESET = b"R"
CMD_STEP = b"S"
CMD_QUERY = b"Q"
CMD_QUIT = b"X"
RSP_ACK = b"A"
RSP_ERROR = b"E"

#: Every answer but a step's is `RSP_ACK` or `RSP_ERROR`, a 4-byte length and then that many
#: bytes; a step's answer is always STEP_RSP.size bytes and carries its own tag.
_LEN = struct.Struct("<I")

#: `Chip.state(e)`'s keys (docs/mvp-interfaces.md `## ISS API`): the RTL signal names of
#: `g_eng[e].u_eng` carry exactly these, except `forced` which is the spec's `force`.
STATE_KEYS = ("pc", "ir", "x", "y", "osr", "isr", "osr_cnt", "isr_cnt", "delay_cnt", "stalled",
              "latch", "latch_pend", "latch_host", "forced", "tx_level", "rx_level")


# ---------------------------------------------------------------------------------------
# The simulator side: one cocotb test that serves the pipes, cycle by cycle
# ---------------------------------------------------------------------------------------

def _read_exact(fd: int, size: int) -> bytes:
    """`size` bytes from `fd`, however many reads that takes."""
    chunks = []
    got = 0
    while got < size:
        chunk = os.read(fd, size - got)
        if not chunk:
            raise EOFError(f"the test process closed the command pipe after {got} of {size} "
                           "bytes: the simulator died")
        chunks.append(chunk)
        got += len(chunk)
    return b"".join(chunks)


def _write_all(fd: int, data: bytes) -> None:
    sent = 0
    while sent < len(data):
        sent += os.write(fd, data[sent:])


def _fifo_words(fifo, level: int) -> list[int]:
    """The words in a `pio_fifo`, oldest first: `mem` read from `rp` for `level` words.

    `src/pio_fifo.sv` wraps `wp` and `rp` at FIFO_DEPTH, so `(rp + i) % FIFO_DEPTH` is the
    address of the i-th word from the head.
    """
    rp = int(fifo.rp.value)
    mem = fifo.mem
    return [int(mem[(rp + i) % FIFO_DEPTH].value) for i in range(level)]


def _fifo_handle(eng, name: str):
    """`u_txf` / `u_rxf` of an engine."""
    return getattr(eng, name, None)


def _imem_words(eng):
    """The engine's 32 instruction words, read out of the RTL wherever this build keeps them.

    `docs/mvp-interfaces.md` `### Lockstep visibility` asks for the IMEM contents to be
    readable, and `test_..._loaded_what_was_assembled` has to see the words the chip really
    holds. In the flop build that is `u_eng.imem`, the array in `src/pio_engine.sv`. Built
    with `-DIMEM_SRAM` the words live in the SRAM
    macro instead, in the array the PDK behavioural model holds the macro's storage in -- which
    is the very array port A fetches from, so the words read here are the words the engine
    executes. Same 32 words, same indices, same checks; only the place they are read from
    follows the RTL. A build with neither is a harness error, reported like the others.
    """
    # The flop array lives in the `g_imem_flop` generate branch of
    # src/pio_engine.sv, not directly in `u_eng`; older flop builds had `u_eng.imem`.
    for flop in ("g_imem_flop.imem", "imem"):
        obj = eng
        try:
            for step in flop.split("."):
                obj = getattr(obj, step)
            return obj
        except AttributeError:
            continue
    # `g_imem_sram` is the generate branch src/pio_engine.sv instantiates the macro in; the
    # second `u_imem` is the macro instance inside pio_imem_sram.sv.
    for path in ("g_imem_sram.u_imem.u_imem", "u_imem.u_imem"):
        obj = eng
        try:
            for step in path.split("."):
                obj = getattr(obj, step)
            return obj.i_SRAM_2P_behavioral_bm_bist.mem_arr
        except AttributeError:
            continue
    raise RuntimeError(
        "no IMEM array in g_eng[e].u_eng: this build needs src/pio_engine.sv's `imem` "
        "or, with -DIMEM_SRAM, the SRAM macro's array (see "
        "docs/mvp-interfaces.md `### Lockstep visibility`)"
    )


class _Chip:
    """The handles this harness reads, bound once so a cycle costs no `getattr`."""

    def __init__(self, dut) -> None:
        self.dut = dut
        self.engines = [dut.g_eng[e].u_eng for e in range(NUM_ENGINES)]
        # STATE_KEYS, in the order of the tuple above.
        self.state = [[getattr(eng, k) for k in STATE_KEYS] for eng in self.engines]
        # The per-engine registers the read mux serves (ports of `pio_engine`).
        self.sm_regs = [[getattr(eng, f"r_{n}") for n in
                         ("clkdiv", "execctrl", "shiftctrl", "addr", "instr", "pinctrl")]
                        for eng in self.engines]
        self.imem = [_imem_words(eng) for eng in self.engines]
        self.txf = [_fifo_handle(eng, "u_txf") for eng in self.engines]
        self.rxf = [eng.u_rxf for eng in self.engines]
        # The chip's own registers `peek()` needs that no engine holds.
        self.sm_enable = dut.sm_enable
        self.route_en = dut.route_en
        self.fdebug = dut.fdebug
        self.inte = (dut.inte0, dut.inte1)
        self.intf = (dut.intf0, dut.intf1)
        self.outs = (dut.pin_out, dut.pin_oe, dut.irq0, dut.irq1)
        self.bus = (dut.bus_we, dut.bus_re, dut.bus_addr, dut.bus_wdata, dut.bus_rdata)
        self.pin_in = dut.pin_in
        self.rst_n = dut.rst_n
        self.clk = dut.clk

    def check_handles(self) -> None:
        """Fail loudly, naming the signal, if the build does not expose what we read."""
        for group, label in ((self.state, "g_eng[e].u_eng"), (self.sm_regs, "g_eng[e].u_eng"),
                             (self.imem, "g_eng[e].u_eng IMEM words"), (self.txf, "g_eng[e].u_eng.u_txf"),
                             (self.rxf, "g_eng[e].u_eng.u_rxf")):
            for e, handles in enumerate(group):
                for h in handles:
                    if h is None:
                        raise RuntimeError(f"{label}[{e}] is not visible: this build needs the "
                                           "visibility flag of docs/mvp-interfaces.md "
                                           "`### Lockstep visibility`")
        for name in ("sm_enable", "fdebug", "inte0", "inte1", "intf0", "intf1", "pin_out",
                     "pin_oe", "irq0", "irq1", "bus_we", "bus_re", "bus_addr", "bus_wdata",
                     "bus_rdata", "pin_in", "rst_n", "clk"):
            if getattr(self.dut, name, None) is None:
                raise RuntimeError(f"the DUT has no {name!r} port")

    def snapshot(self) -> dict:
        """Everything `Chip.peek()`, `Chip.state()` and `chip.engines[e]` can be asked for,
        read out of the RTL at the point in time the parent asks."""
        out = {
            "sm_enable": int(self.sm_enable.value),
            "route_en": int(self.route_en.value),
            "fdebug": int(self.fdebug.value),
            "inte": [int(h.value) for h in self.inte],
            "intf": [int(h.value) for h in self.intf],
            "outs": [int(h.value) for h in self.outs],
            "engines": [],
        }
        for e in range(NUM_ENGINES):
            state = {k: int(h.value) for k, h in zip(STATE_KEYS, self.state[e])}
            state["sm"] = [int(h.value) for h in self.sm_regs[e]]
            state["imem"] = [int(self.imem[e][k].value) for k in range(IMEM_SIZE)]
            state["tx"] = _fifo_words(self.txf[e], state["tx_level"])
            state["rx"] = _fifo_words(self.rxf[e], state["rx_level"])
            out["engines"].append(state)
        return out


class _GlChip:
    """The handles the gate-level `gl_top` exposes: ports only, and the FDEBUG flops."""

    def __init__(self, dut) -> None:
        self.dut = dut
        self.outs = (dut.pin_out, dut.pin_oe, dut.irq0, dut.irq1)
        self.bus = (dut.bus_we, dut.bus_re, dut.bus_addr, dut.bus_wdata, dut.bus_rdata)
        self.pin_in, self.rst_n, self.clk = dut.pin_in, dut.rst_n, dut.clk
        self.gl_fdebug = dut.gl_fdebug
        self.gl_levels = dut.gl_levels

    def check_handles(self) -> None:
        pass

    out_of_reset = False        # before the first reset the netlist's flops are X (the RTL's are initialised)

    def snapshot(self) -> dict:
        if not self.out_of_reset:
            return {"fdebug": 0, "engines": [{"tx_level": 0, "rx_level": 0} for _ in range(NUM_ENGINES)]}
        try:
            lv = int(self.gl_levels.value)                  # an X here is a finding: it raises
        except ValueError:
            raise RtlError(f"gate level: FIFO levels are not 0/1 out of reset: gl_levels={self.gl_levels.value} gl_fdebug={self.gl_fdebug.value}") from None
        return {"fdebug": int(self.gl_fdebug.value),
                "engines": [{"tx_level": (lv >> (8 * e)) & 15, "rx_level": (lv >> (8 * e + 4)) & 15}
                            for e in range(NUM_ENGINES)]}


class _Server:
    """Drives the DUT from the parent's commands, one core cycle per `step` command.

    The order inside a cycle is `verif/fuzz/lockstep.py`'s, which is also the order
    `docs/mvp-interfaces.md` `## Register bus` describes:

      1. at the falling edge, write `pin_in` and the bus signals of this cycle;
      2. in `ReadOnly`, read `bus_rdata`: it is combinational, the register's state in this
         very cycle, and a host read's side effect happens at the edge that follows;
      3. at the rising edge, the cycle is done; in `ReadOnly` again, sample `pin_out`,
         `pin_oe`, `irq0`, `irq1`, which are the state the next cycle starts from.
    """

    def __init__(self, dut, cmd_fd: int, resp_fd: int) -> None:
        self.chip = (_GlChip if GL else _Chip)(dut)
        self.chip.check_handles()
        self.cmd_fd, self.resp_fd = cmd_fd, resp_fd
        self.clk = self.chip.clk
        self.bus_we, self.bus_re, self.bus_addr, self.bus_wdata, self.bus_rdata = self.chip.bus
        self.pin_in = self.chip.pin_in
        self.rst_n = self.chip.rst_n
        self.pin_out, self.pin_oe, self.irq0, self.irq1 = self.chip.outs
        #: True while the coroutine sits in a write phase (just after a reset), where a cycle
        #: may be driven without waiting for the next falling edge.
        self.at_falling = False
        #: cycles and seconds since this simulator started; a reset starts neither again
        self.total_cycles = 0
        self.busy_s = 0.0
        self.wait_s = 0.0
        self.cycles = 0
        self.chips = 0
        #: what the inputs currently hold, so a cycle writes only what changed: a cocotb
        #: write costs several microseconds, and in a run with no host access most cycles
        #: would write the same zeros again.
        self.driven: dict = {}

    def _drive(self, handle, value: int) -> None:
        """Write `handle` only if it does not already hold `value`."""
        if self.driven.get(id(handle)) != value:
            handle.value = value
            self.driven[id(handle)] = value

    # -- the wire ------------------------------------------------------------------------

    def _answer(self, tag: bytes, payload: bytes) -> None:
        _write_all(self.resp_fd, tag + _LEN.pack(len(payload)) + payload)

    def _fail(self, exc: BaseException) -> None:
        """Report an exception to the parent, so it fails the test it was running."""
        import traceback
        self._answer(RSP_ERROR, traceback.format_exc().encode("utf-8", "replace"))

    # -- the simulator ------------------------------------------------------------------

    async def reset(self) -> None:
        """From reset: `rst_n` low across at least one rising edge, released at a falling edge.

        That is `verif/fuzz/lockstep.py`'s sequence, and it leaves the next rising edge to
        close core cycle 0, which is the cycle `iss.pio_iss.Chip` steps first after `reset()`.
        The read-mux check runs first, while `rst_n` is still low, where nothing it drives can
        change any state.
        """
        from cocotb.triggers import FallingEdge, RisingEdge, Timer
        self.at_falling = False
        await FallingEdge(self.clk)
        self._drive(self.rst_n, 0)
        self._drive(self.pin_in, 0)
        self._drive(self.bus_we, 0)
        self._drive(self.bus_re, 0)
        self._drive(self.bus_addr, 0)
        self._drive(self.bus_wdata, 0)
        await RisingEdge(self.clk)          # one reset edge: the registers hold reset values
        if GL:
            await RisingEdge(self.clk)      # (a second one: nothing in the netlist is initialised but by reset)
            report = {"intr_delta": [], "mux_checked": 0}
        else:
            report = await self.check_read_mux(Timer)
        await FallingEdge(self.clk)              # the rising edges in between are all reset
        self._drive(self.rst_n, 1)
        if GL:
            # The chip leaves reset two edges after rst_n (synchronizer) and host_spi's CS synchronizer is
            # transparent two edges after that, which a read's capture needs (`sel`): run these idle, so that
            # the next step is core cycle 0 of a chip that is out of reset and has its read path.
            for _ in range(4):
                await RisingEdge(self.clk)
            await FallingEdge(self.clk)
            self.chip.out_of_reset = True
        self.at_falling = True
        self.cycles = 0
        self.chips += 1
        return report

    async def check_read_mux(self, Timer) -> dict:
        """Compare the harness's read mux with the RTL's own read path, every bus address.

        In reset, so the scan moves time (a nanosecond per address, no clock edge in between:
        the clock period is 20 ns) without a core cycle passing and without any register
        seeing a write. Three answers have to agree: the RTL, `fw/rtl_harness.read_mux()` and
        `iss.pio_iss.Chip.peek()`. The first is the check; the third is a report, because a
        difference there is a finding about one of the two, not about this harness.
        """
        golden = _reference_chip()
        bad, delta, rtl_values = [], [], []
        self._drive(self.bus_re, 1)
        for addr in range(NUM_ADDRS):
            self._drive(self.bus_addr, addr)
            await Timer(1, unit="ns")
            rtl_values.append(int(self.bus_rdata.value))
        # The state the reads above saw: reset is asynchronous, so it is read after them, not
        # before, and after them rather than while they are being taken.
        snap = self.chip.snapshot()
        for addr, rtl in enumerate(rtl_values):
            mine = read_mux(addr, snap)
            if rtl != mine:
                bad.append(f"{addr:#04x} {addr_name(addr)}: rtl {rtl:#010x} != mux {mine:#010x}")
            if rtl != golden.peek(addr):
                delta.append(addr)
        self._drive(self.bus_re, 0)
        self._drive(self.bus_addr, 0)
        if bad:
            raise RuntimeError("fw/rtl_harness.read_mux does not match src/pio_chip.sv:\n  "
                               + "\n  ".join(bad))
        return {"intr_delta": delta, "mux_checked": NUM_ADDRS}

    async def step(self, cmd: bytes) -> bytes:
        """One core cycle: drive at a falling edge, answer after the rising edge."""
        from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge
        busy = time.perf_counter()
        pin_in, we, re_, addr, wdata = STEP_CMD.unpack(cmd)
        if not self.at_falling:
            await FallingEdge(self.clk)
        self.at_falling = False
        self._drive(self.pin_in, pin_in)
        self._drive(self.bus_we, we)
        self._drive(self.bus_re, re_)
        self._drive(self.bus_addr, addr)
        self._drive(self.bus_wdata, wdata)
        await ReadOnly()
        rdata = int(self.bus_rdata.value) if (re_ and not GL) else 0
        await RisingEdge(self.clk)
        await ReadOnly()
        if GL and re_:
            rdata = int(self.bus_rdata.value)       # host_spi's capture of this very cycle's read data
        self.cycles += 1
        self.total_cycles += 1
        self.busy_s += time.perf_counter() - busy
        return STEP_RSP.pack(int(self.pin_out.value), int(self.pin_oe.value),
                             int(self.irq0.value), int(self.irq1.value), rdata)

    async def serve(self) -> None:
        """Answer commands until the parent quits. Raises whatever it cannot answer."""
        sizes = {CMD_RESET: 0, CMD_STEP: STEP_CMD.size, CMD_QUERY: 0, CMD_QUIT: 0}
        while True:
            idle = time.perf_counter()
            head = _read_exact(self.cmd_fd, 1)
            self.wait_s += time.perf_counter() - idle
            if head not in sizes:
                raise ValueError(f"unknown command {head!r} from the test process")
            size = sizes[head]
            cmd = _read_exact(self.cmd_fd, size) if size else b""
            try:
                if head == CMD_QUIT:
                    self._answer(RSP_ACK, b"")
                    return
                if head == CMD_RESET:
                    report = await self.reset()
                    self._answer(RSP_ACK, json.dumps(report).encode("utf-8"))
                elif head == CMD_STEP:
                    self._answer(RSP_ACK, await self.step(cmd))
                elif head == CMD_QUERY:
                    payload = json.dumps(self.chip.snapshot()).encode("utf-8")
                    self._answer(RSP_ACK, payload)
            except Exception as exc:            # tell the parent, then fail this cocotb test
                self._fail(exc)
                raise


@cocotb.test()
async def fw_rtl(dut) -> None:
    """The cocotb test of `fw/Makefile`: serve `src/pio_chip.sv` to the firmware tests.

    It is a server, not a test of its own: it ends when the parent says so, and every check
    is the parent pytest's. What it does own is the report at the end, which says how many
    cycles it ran and with which visibility flag, and the two sanity checks that the harness
    itself is sound: that every handle it reads exists, and that its `peek()` read mux agrees
    with the RTL's own combinational read path for all 128 addresses.
    """
    import cocotb
    from cocotb.clock import Clock

    log = dut._log
    cmd_fd = int(os.environ.get("FW_RTL_CMD_FD", "-1"))
    resp_fd = int(os.environ.get("FW_RTL_RESP_FD", "-1"))
    if cmd_fd < 0 or resp_fd < 0:
        raise RuntimeError("FW_RTL_CMD_FD/FW_RTL_RESP_FD are not set: this cocotb test is "
                           "served by fw/rtl_harness.py and is not meant to run on its own")
    cocotb.start_soon(Clock(dut.clk, CLOCK_NS, unit="ns").start())
    server = _Server(dut, cmd_fd, resp_fd)
    try:
        await server.serve()
    finally:
        log.info(f"fw/rtl_harness: {server.total_cycles} core cycles for "
                 f"{server.chips} chip(s), visibility {os.environ.get('FW_RTL_VIS', '?')}, "
                 f"git {git_stamp()}")
        n = max(server.total_cycles, 1)
        log.info(f"fw/rtl_harness: simulator side {server.busy_s * 1e6 / n:.0f} us/cycle "
                 f"inside the cycle, {server.wait_s * 1e6 / n:.0f} us/cycle waiting for the "
                 "test process")


# ---------------------------------------------------------------------------------------
# The read mux: what a host read of `addr` returns (docs/mvp-regmap.md `### Address map`)
# ---------------------------------------------------------------------------------------

A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = 0x0F, 0x10, 0x11
A_SM_BASE = 0x32
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)
A_INTR = 0x4A
A_IRQ0_INTE, A_IRQ0_INTF, A_IRQ0_INTS = 0x4B, 0x4C, 0x4D
A_IRQ1_INTE, A_IRQ1_INTF, A_IRQ1_INTS = 0x4E, 0x4F, 0x50
#: this build's addresses (the MVP's are 0x04/0x05, 0x08/0x09, 0x12/0x52)
A_TXF = tuple(REGMAP.txf(e) for e in range(NUM_ENGINES))
A_RXF = tuple(REGMAP.rxf(e) for e in range(NUM_ENGINES))
A_IMEM = tuple(REGMAP.imem_base(e) for e in range(NUM_ENGINES))
DBG_CFGINFO_VALUE = REGMAP.cfginfo          # 0x0020_0204 at the MVP


def _reference_chip():
    """The ISS the read mux is reported against: the golden ISS at the MVP, else the parametric one."""
    return _GOLDEN_CHIP() if PARAMS == _pio_iss_p.MVP_PARAMS else _pio_iss_p.Chip(PARAMS)


def _irq_word(bits: int) -> int:
    """A packed IRQ word {TXNFULL[N-1:0], RXNEMPTY[N-1:0]} in the register layout of
    `docs/mvp-regmap.md` `**INTR**`: RXNEMPTY of engine e at bit e, TXNFULL at bit 4+e (bits 9:8,
    the `SMn` IRQ flags, are always 0). `src/pio_chip.sv` `irq_word` does the same."""
    n = NUM_ENGINES
    return (bits & ((1 << n) - 1)) | (((bits >> n) & ((1 << n) - 1)) << 4)


def read_mux(addr: int, snap: Mapping) -> int:
    """What a host read of `addr` returns in the cycle this snapshot was taken in.

    `snap` is one snapshot of the RTL: `sm_enable`, `route_en`, the packed `fdebug` word (4 fields
    of NUM_ENGINES bits), the `inte` and `intf` pairs, `outs` (pin_out, pin_oe, irq0, irq1) and one
    dict per engine with the `STATE_KEYS`, the six `sm` registers, the IMEM and the two FIFOs.
    Addresses with no register (TXF, IMEM, reserved) return 0, as `docs/mvp-interfaces.md`
    `## Register bus` says of a read. Layout of every register: tools/bench/sim/REGMAP.md.
    """
    n = NUM_ENGINES
    if not 0 <= addr < NUM_ADDRS:
        raise ValueError(f"bus address {addr:#x} out of range")
    eng = snap["engines"]
    tx = [e["tx_level"] for e in eng]
    rx = [e["rx_level"] for e in eng]
    if addr == A_CTRL:
        return snap["sm_enable"] | (snap.get("route_en", 0) << 16)
    if addr == A_FSTAT:
        v = 0
        for e in range(n):
            v |= (tx[e] == 0) << (24 + e) | (tx[e] == FIFO_DEPTH) << (16 + e)
            v |= (rx[e] == 0) << (8 + e) | (rx[e] == FIFO_DEPTH) << e
        return v
    if addr == A_FDEBUG:
        f = snap["fdebug"]            # {TXSTALL, TXOVER, RXUNDER, RXSTALL}, N bits each
        v = 0
        for field in range(4):
            v |= ((f >> (n * field)) & ((1 << n) - 1)) << (8 * field)
        return v
    if addr == A_FLEVEL:
        v = 0
        for e in range(n):
            v |= min(tx[e], 15) << (8 * e) | min(rx[e], 15) << (8 * e + 4)   # a nibble saturates at 15
        return v
    for e in range(n):
        if addr == REGMAP.rxf(e):
            if (snap.get("route_en", 0) >> e) & 1:
                return 0              # engine_route: this RX FIFO belongs to the link
            return eng[e]["rx"][0] if eng[e]["rx"] else 0
    if addr == A_DBG_PADOUT:
        return snap["outs"][0]
    if addr == A_DBG_PADOE:
        return snap["outs"][1]
    if addr == A_DBG_CFGINFO:
        return DBG_CFGINFO_VALUE
    if A_SM_BASE <= addr < A_SM_BASE + 6 * n:
        e, i = divmod(addr - A_SM_BASE, 6)
        value = eng[e]["sm"][i]
        if i == SM_EXECCTRL:          # bit 31 is EXEC_STALL, which has no register of its own
            value |= (eng[e]["latch_pend"] & eng[e]["latch_host"]) << 31
        return value
    if addr == A_INTR:
        return _irq_word(_intr4(tx, rx))
    if addr in (A_IRQ0_INTE, A_IRQ1_INTE):
        return _irq_word(snap["inte"][0 if addr == A_IRQ0_INTE else 1])
    if addr in (A_IRQ0_INTF, A_IRQ1_INTF):
        return _irq_word(snap["intf"][0 if addr == A_IRQ0_INTF else 1])
    if addr in (A_IRQ0_INTS, A_IRQ1_INTS):
        k = 0 if addr == A_IRQ0_INTS else 1
        return _irq_word((_intr4(tx, rx) & snap["inte"][k]) | snap["intf"][k])
    return 0


def _intr4(tx: list, rx: list) -> int:
    """The IRQ source bits as the RTL keeps them: {TXNFULL[N-1:0], RXNEMPTY[N-1:0]}."""
    n = NUM_ENGINES
    packed = 0
    for e in range(n):
        packed |= (tx[e] != FIFO_DEPTH) << (n + e)
        packed |= (rx[e] != 0) << e
    return packed


# ---------------------------------------------------------------------------------------
# The test-process side: the simulator, the `Chip` API and the install hook
# ---------------------------------------------------------------------------------------

class RtlError(RuntimeError):
    """The RTL, the harness or the simulator did something a firmware test must see."""


def git_stamp() -> str:
    try:
        out = subprocess.run(["git", "describe", "--always", "--dirty"], cwd=ROOT,
                             capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception:                                     # pragma: no cover
        return "?"
    return f"{out} {time.strftime('%Y-%m-%d')}"


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    return default if value in (None, "") else int(value)


class _Simulator:
    """One `make -C fw sim` process and the pipes to it: one RTL.

    A test process may hold several of these at once, because a test may hold several rigs
    (`fw/spi/test_spi.py` keeps two). A simulator is handed to a chip when nothing else holds
    it -- the owner is a weak reference, so a chip that has been collected leaves its simulator
    to the next one, which resets it -- and one more is started when they all have an owner.
    `FW_RTL_MAX_SIMS` bounds how many (default 4).
    """

    #: bytes of a step's answer: tag, length, payload
    STEP_FRAME = 1 + _LEN.size + STEP_RSP.size

    def __init__(self, sim: str = "verilator") -> None:
        self.sim = sim
        self.cycles = 0
        self.queries = 0
        self.resets = 0
        self.owner_ref = None                     # weakref to the chip holding this RTL
        self.intr_delta: list = []
        self.t_last = time.perf_counter()  # when this RTL last stepped, for the pool
        #: seconds spent blocked on the simulator's answer, for the rate report
        self.rtt_s = 0.0
        cmd_r, cmd_w = os.pipe()
        resp_r, resp_w = os.pipe()
        self.cmd_w, self.resp_r = cmd_w, resp_r        # what this process keeps
        self.log_path = Path(os.environ.get("FW_RTL_LOG") or
                             ROOT / "fw" / "sim_build" /
                             f"{sim}-fw{'-gl-' + os.environ['GL_TAG'] if GL and os.environ.get('GL_TAG') else ''}"
                             f"-{len(_SIMS) + 1}.log")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ)
        env.setdefault("PYTHON", sys.executable)         # fw/Makefile runs tools.bench.rtlparams with it
        env["PYTHONPATH"] = os.pathsep.join(p for p in (str(ROOT), env.get("PYTHONPATH")) if p)
        env["FW_RTL_CMD_FD"] = str(cmd_r)
        env["FW_RTL_RESP_FD"] = str(resp_w)
        env.setdefault("FW_RTL_CYCLES", "0")
        started = time.time()
        self.started = started
        with open(self.log_path, "w") as log:
            # Build first, one process at a time (fw/Makefile `build`); a failure here is left for `make sim` to
            # repeat into the same log, so the existing "the simulator died" path reports it.
            with open(ROOT / "fw" / "sim_build" / ".build.lock", "w") as lk:
                fcntl.flock(lk, fcntl.LOCK_EX)
                subprocess.run(["make", "-j2", "-C", str(ROOT / "fw"), f"SIM={sim}", "build"],
                               env=env, cwd=str(ROOT / "fw"), stdout=log, stderr=subprocess.STDOUT)
            self.proc = subprocess.Popen(
                ["make", "-j2", "-C", str(ROOT / "fw"), f"SIM={sim}", "sim"],
                env=env, cwd=str(ROOT / "fw"), stdout=log, stderr=subprocess.STDOUT,
                pass_fds=(cmd_r, resp_w))
        os.close(cmd_r), os.close(resp_w)   # only the child needs these two now
        self.boot_s = time.time() - started
        atexit.register(self.close)

    # -- the wire -------------------------------------------------------------------

    def _command(self, tag: bytes, payload: bytes = b"") -> None:
        _write_all(self.cmd_w, tag + payload)

    def _answer(self, size: int) -> bytes:
        head = _read_exact(self.resp_r, 1 + _LEN.size)
        if head[:1] == RSP_ERROR:
            length = _LEN.unpack(head[1:])[0]
            raise RtlError("the simulator could not answer:\n"
                           + _read_exact(self.resp_r, length).decode("utf-8", "replace"))
        if head[:1] != RSP_ACK:
            raise RtlError(f"unexpected answer tag {head[:1]!r} from the simulator")
        (length,) = _LEN.unpack(head[1:])
        return _read_exact(self.resp_r, length)

    def step(self, pin_in: int, bus: Optional[tuple]) -> int:
        """One core cycle. `bus` is `("w", addr, data)`, `("r", addr)` or None."""
        kind = "-" if bus is None else bus[0]
        addr = 0 if bus is None else bus[1]
        data = 0 if bus is None or kind == "r" else bus[2]
        we = 1 if kind == "w" else 0
        re_ = 1 if kind == "r" else 0
        waited = time.perf_counter()
        self._command(CMD_STEP, STEP_CMD.pack(pin_in, we, re_, addr, data))
        pin_out, pin_oe, irq0, irq1, rdata = STEP_RSP.unpack(self._answer(self.STEP_FRAME))
        self.rtt_s += time.perf_counter() - waited
        self.t_last = time.perf_counter()
        self.cycles += 1
        limit = _env_int("FW_RTL_CYCLES", 0)
        if limit and self.cycles > limit:
            raise RtlError(f"the chip ran {self.cycles} cycles, past FW_RTL_CYCLES={limit}")
        return pin_out, pin_oe, irq0, irq1, rdata

    def reset(self) -> dict:
        """From reset, and read back what the harness found out about this build."""
        self._command(CMD_RESET)
        report = json.loads(self._answer(0).decode("utf-8") or "{}")
        self.resets += 1
        if report.get("intr_delta"):
            self.intr_delta = report["intr_delta"]
        return report

    def query(self) -> dict:
        """Everything the tests can ask about the RTL's inside, read out of it now."""
        self._command(CMD_QUERY)
        self.queries += 1
        return json.loads(self._answer(0).decode("utf-8"))

    def close(self) -> None:
        """Quit the simulator and make sure its own verdict was a pass.

        Called from `atexit`, where an exception would only be printed, so a failure here
        prints the tail of the simulator's log and exits non-zero: a run that cannot have
        been a real run must not report itself as one.
        """
        if _SIMS and self in _SIMS:
            _SIMS.remove(self)
        if self.proc.poll() is not None:
            return
        try:
            self._command(CMD_QUIT)
            self._answer(0)
        except Exception as exc:                       # the child may already be gone
            print(f"fw/rtl_harness: quitting the simulator: {exc}", file=sys.stderr)
        try:
            self.proc.wait(timeout=60)
        except subprocess.TimeoutExpired:              # pragma: no cover
            self.proc.kill()
            self.proc.wait(timeout=10)
        for fd in (self.cmd_w, self.resp_r):
            try:
                os.close(fd)
            except OSError:
                pass
        if self.proc.returncode == 0:
            return
        print(f"fw/rtl_harness: the simulator exited {self.proc.returncode}; the last lines of "
              f"{self.log_path} follow", file=sys.stderr)
        tail = self.log_path.read_text(errors="replace").splitlines()[-40:]
        print("\n".join(tail), file=sys.stderr)
        sys.stderr.flush()
        os._exit(1)

    # -- the report ------------------------------------------------------------------

    def report(self) -> str:
        rate = self.cycles / max(time.time() - self.started, 1e-9)
        return (f"fw/rtl_harness: {self.sim}, visibility {os.environ.get('FW_RTL_VIS', 'in the log')}, "
                f"{self.cycles} core cycles, {self.queries} state queries, {self.resets} resets, "
                f"boot {self.boot_s:.1f}s, {rate:.0f} cycles/s "
                f"({self.rtt_s * 1e6 / max(self.cycles, 1):.0f} us/cycle on the wire), "
                f"git {git_stamp()}")

    def intr_delta_text(self) -> str:
        """The addresses where this build's read path and `iss.pio_iss.Chip.peek` differ."""
        if not self.intr_delta:
            return f"fw/rtl_harness: the read path agrees with the ISS's peek on all {NUM_ADDRS}"
        rows = ", ".join(f"{a:#04x}" for a in self.intr_delta)
        return (f"fw/rtl_harness: FINDING the RTL's read path and iss.pio_iss.Chip.peek differ "
                f"on {len(self.intr_delta)} address(es): {rows} (IRQ0/IRQ1 INTE/INTF/INTS/INTR)")


#: Every simulator this process has started and not shut down. A chip holds one of these; the
#: list holds no chip, so a collected chip's simulator is available for the next chip.
_SIMS: list = []


def acquire(chip: "RtlChip") -> "_Simulator":
    """A simulator `chip` may run on: its own, else one nobody owns, else a new one.

    A chip that has been collected leaves its simulator to the next chip, which resets it.
    pytest keeps a finished test's locals alive for a while after it returns, so the pool is
    `FW_RTL_MAX_SIMS` deep (default 4) and, when that is not enough, the least recently stepped
    simulator is taken over. A chip whose RTL was taken over says so when it next steps, so
    the state can never go quietly missing.
    """
    sim = chip.simulator
    if sim is not None:
        return sim
    for candidate in list(_SIMS):
        if candidate.proc.poll() is not None:
            continue                       # its process is gone; nothing to reuse
        owner = candidate.owner_ref() if candidate.owner_ref is not None else None
        if owner is None or owner is chip:
            candidate.owner_ref = weakref.ref(chip)
            chip.simulator = candidate
            return candidate
    limit = _env_int("FW_RTL_MAX_SIMS", 4)
    if len(_SIMS) >= limit:
        victim = min(_SIMS, key=lambda s: s.t_last)
        print(f"fw/rtl_harness: all {limit} simulators are in use; the least recently stepped "
              f"one ({victim.log_path.name}) moves to this chip", file=sys.stderr)
        chip.simulator = victim
        victim.owner_ref = weakref.ref(chip)
        return victim
    sim = _Simulator(os.environ.get("FW_RTL_SIM", "verilator"))
    sim.owner_ref = weakref.ref(chip)
    chip.simulator = sim
    _SIMS.append(sim)
    return sim


class _RtlEngine:
    """One engine's state, read out of `g_eng[e].u_eng` when something looks at it.

    The names are `iss.pio_iss.Engine`'s: the `STATE_KEYS` plus the six configuration words
    (`clkdiv`, `execctrl`, `shiftctrl`, `pinctrl`) and the two FIFOs as lists, oldest word
    first, the way the ISS keeps them.
    """
    _SM_NAMES = ("clkdiv", "execctrl", "shiftctrl", "addr", "instr", "pinctrl")

    def __init__(self, chip: "RtlChip", idx: int) -> None:
        self.chip, self.idx = chip, idx

    def __getattr__(self, name: str):
        if name.startswith("_"):
            raise AttributeError(name)
        state = self.chip._engine_state(self.idx)
        if GL and name not in state:                   # hasattr() probes (verif/perf/probe.py) get a plain no
            raise AttributeError(f"gate level: the flat netlist exposes only the FIFO levels ({name!r})")
        if name in state:
            return state[name]
        if name in self._SM_NAMES:                    # EXECCTRL bit 31 is EXEC_STALL, as above
            i = self._SM_NAMES.index(name)
            value = state["sm"][i]
            if i == 1:
                value |= (state["latch_pend"] & state["latch_host"]) << 31
            return value
        if name in ("imem", "tx", "rx"):
            return list(state[name])
        raise AttributeError(f"iss.pio_iss.Engine has no attribute {name!r} either; this "
                             "harness reads only the state, the SM registers, the IMEM and the "
                             "FIFOs, and a test that needs more of the ISS's internals has to "
                             "say so")

    def __repr__(self) -> str:
        return f"<rtl engine {self.idx}>"


class RtlChip:
    """`iss.pio_iss.Chip` on the RTL: the API of `docs/mvp-interfaces.md` `## ISS API`.

    `step()` advances one core cycle and returns the read data of a host read in that cycle;
    `pin_out`, `pin_oe`, `irq0` and `irq1` are the state after it, as on the ISS. `peek()`,
    `state()` and everything under `engines` are answered from the RTL when they are asked
    for, and cached until the next `step()`.
    """

    #: what `fw.regmap.active()` and the drivers read off the installed chip class
    PARAMS = PARAMS
    REGMAP = REGMAP

    def __init__(self) -> None:
        self.engines = [_RtlEngine(self, e) for e in range(NUM_ENGINES)]
        self.pin_out = 0
        self.pin_oe = 0
        self.irq0 = 0
        self.irq1 = 0
        #: the simulator this chip runs on, once one has been acquired for it
        self.simulator: Optional["_Simulator"] = None
        self._snap: Optional[dict] = None
        self._started = False

    # -- the Chip API ---------------------------------------------------------------

    def reset(self) -> None:
        """Back to the power-on state; the RTL is reset on this chip's first `step()`."""
        self.pin_out = self.pin_oe = self.irq0 = self.irq1 = 0
        self._snap = None
        self._started = False

    def step(self, pin_in: int, bus: Optional[tuple] = None) -> Optional[int]:
        """One core cycle, driven exactly as `ChipDevice.tick` drives the ISS's `step()`."""
        sim = acquire(self)
        if not self._started:
            sim.reset()
            self._started = True
        elif sim.owner_ref() is not self:
            raise RtlError(f"this chip's RTL ({sim.log_path.name}) now belongs to another chip; "
                           "a test kept more chips alive at once than FW_RTL_MAX_SIMS allows")
        expect = None
        if bus is not None and bus[0] == "r" and self._xcheck:
            expect = read_mux(bus[1], self._snapshot())    # the state this read cycle starts in
        pin_out, pin_oe, irq0, irq1, rdata = sim.step(pin_in & 0x3FFFF, bus)
        self.pin_out, self.pin_oe, self.irq0, self.irq1 = pin_out, pin_oe, irq0, irq1
        self._snap = None
        if expect is not None and rdata != expect:
            raise RtlError(f"the harness read {addr_name(bus[1])} as {expect:#010x} but the RTL "
                           f"returned {rdata:#010x}: the read mux in fw/rtl_harness.py and "
                           "src/pio_chip.sv disagree")
        return rdata if (bus is not None and bus[0] == "r") else None

    def peek(self, addr: int) -> int:
        """What a read of `addr` would return now, from this chip's read path."""
        if GL:
            if addr != A_FDEBUG:
                raise RtlError(f"the gate-level harness can peek FDEBUG only, not {addr_name(addr)}: "
                               "the flat netlist has no other internal signals (a host read through "
                               "the bus works)")
            f = self._snapshot()["fdebug"]
            n = NUM_ENGINES
            return sum(((f >> (n * k)) & ((1 << n) - 1)) << (8 * k) for k in range(4))
        return read_mux(addr, self._snapshot())

    def state(self, e: int) -> dict:
        """`docs/mvp-interfaces.md` `## ISS API`: the canonical engine state, exactly those keys."""
        if GL:
            raise RtlError("gate level: the flat netlist has no engine state to read (Chip.state)")
        return {k: self._engine_state(e)[k] for k in STATE_KEYS}

    # -- what the tests reach for that the API does not name --------------------------

    @property
    def fdebug(self) -> int:
        """FDEBUG as the ISS keeps it: the 32-bit word a read of `0x02` returns."""
        return self.peek(A_FDEBUG)

    @property
    def sm_enable(self) -> int:
        return self.peek(A_CTRL)

    # -- internals ------------------------------------------------------------------

    _xcheck = not GL             # every host read is checked against the harness's read mux (not at gate level)

    def _snapshot(self) -> dict:
        if self._snap is None:
            self._snap = acquire(self).query()
        return self._snap

    def _engine_state(self, e: int) -> dict:
        return self._snapshot()["engines"][e]


def addr_name(addr: int) -> str:
    """The register name of `addr`, for the messages the harness prints."""
    names = {A_CTRL: "CTRL", A_FSTAT: "FSTAT", A_FDEBUG: "FDEBUG", A_FLEVEL: "FLEVEL",
             A_DBG_PADOUT: "DBG_PADOUT", A_DBG_PADOE: "DBG_PADOE",
             A_DBG_CFGINFO: "DBG_CFGINFO", A_INTR: "INTR", A_IRQ0_INTE: "IRQ0_INTE",
             A_IRQ0_INTF: "IRQ0_INTF", A_IRQ0_INTS: "IRQ0_INTS", A_IRQ1_INTE: "IRQ1_INTE",
             A_IRQ1_INTF: "IRQ1_INTF", A_IRQ1_INTS: "IRQ1_INTS"}
    for i, word in enumerate(A_TXF):
        names[word] = f"TXF{i}"
    for i, word in enumerate(A_RXF):
        names[word] = f"RXF{i}"
    if A_SM_BASE <= addr < A_SM_BASE + 6 * NUM_ENGINES:
        e, i = divmod(addr - A_SM_BASE, 6)
        return f"SM{e}_{('CLKDIV', 'EXECCTRL', 'SHIFTCTRL', 'ADDR', 'INSTR', 'PINCTRL')[i]}"
    for e, base in enumerate(A_IMEM):
        if base <= addr < base + IMEM_SIZE:
            return f"IMEM{e}[{addr - base}]"
    return names.get(addr, "reserved")


def install(module: str = "iss.pio_iss", attr: str = "Chip") -> None:
    """Make `iss.pio_iss.Chip` build a chip on the RTL. Call before a driver imports it.

    `fw/*/driver.py` takes its chip from `iss.pio_iss.Chip` and everything else from the real
    `iss.pio_iss.ChipDevice`, so swapping that one name is what puts the RTL under a firmware
    test while the host queue, the `Bus` and the VIPs stay exactly what they are on the ISS.
    Only `FW_TARGET=rtl` may do this, and doing it twice is harmless.
    """
    mod = sys.modules.get(module) or __import__(module)
    if getattr(mod, attr) is RtlChip:
        return
    if os.environ.get("FW_TARGET", "iss").strip().lower() != "rtl":
        raise RtlError(f"install() replaces {module}.{attr} with the RTL chip, which is only what "
                       "FW_TARGET=rtl means; refusing to do it here")
    setattr(mod, attr, RtlChip)


# ---------------------------------------------------------------------------------------
# The command line: `python -m fw.rtl_harness --selftest`
# ---------------------------------------------------------------------------------------

def selftest(cycles: int = 2000, quiet: bool = False) -> int:
    """Prove the whole path once, without pytest: the pipes, the cycle, the read mux.

    Runs the same wiring a firmware test uses -- an `iss.pio_iss.ChipDevice` on a real
    `verif.vip.core.Bus` with nets p0..p17, with `iss.pio_iss.Chip` swapped for the RTL chip --
    writes a word into IMEM through the bus and reads it back out of the RTL's IMEM, and then
    runs the bus. Prints the cycle rate, so the cost of `TARGET=rtl` is a number and not a
    surprise.
    """
    from iss import pio_iss
    from verif.vip.core import Bus

    install()
    bus = Bus(ns_per_tick=CLOCK_NS)
    for k in range(NUM_PINS):
        bus.add_net(f"p{k}")
    dev = bus.attach(pio_iss.ChipDevice(pio_iss.Chip(), name="chip"))
    word = 0x1A57                              # out pins, 1 / mov x, 1: not a NOP either way
    dev.write(A_IMEM[0], word)                # IMEM0[0]
    dev.write(A_IMEM[0] + 1, word ^ 0xFFFF)
    dev.write(0x0C, 0xDEAD_BEEF)                # a reserved word: ignored, but still driven
    for _ in range(4):
        bus.step()
    chip = dev.chip
    sim = chip.simulator or acquire(chip)
    checks = [
        ("IMEM0[0] is the word the host wrote", chip.engines[0].imem[0] == word),
        ("IMEM0[1] is the second one", chip.engines[0].imem[1] == (word ^ 0xFFFF) & 0xFFFF),
        ("IMEM0[2] is still reset", chip.engines[0].imem[2] == 0),
        ("a read of DBG_CFGINFO returns the MVP value", chip.peek(A_DBG_CFGINFO)
         == DBG_CFGINFO_VALUE),
        ("FSTAT says both TX FIFOs are empty", chip.peek(A_FSTAT) == sum((1 << (24 + e)) | (1 << (8 + e)) for e in range(NUM_ENGINES))),
        ("FLEVEL is zero", chip.peek(A_FLEVEL) == 0),
        ("FDEBUG is zero", chip.peek(A_FDEBUG) == 0),
        ("no engine runs", all(chip.state(e)["pc"] == 0 for e in range(NUM_ENGINES))),
        ("the engine's PC is not yet a running one", chip.pin_oe == 0x3E000),
        ("state(0) has exactly the ISS's keys",
         tuple(chip.state(0)) == tuple(iss_keys())),
    ]
    started = time.time()
    bus.run(cycles)
    rate = cycles / max(time.time() - started, 1e-9)
    ok = all(passed for _, passed in checks)
    if not quiet:
        for what, passed in checks:
            print(f"  {'ok  ' if passed else 'FAIL'}  {what}")
        print("  " + sim.report())
        print("  " + sim.intr_delta_text())
    print(f"fw/rtl_harness selftest: {'PASS' if ok else 'FAIL'}, {cycles} bus ticks in "
          f"{time.time() - started:.1f}s ({rate:.0f} cycles/s), log {sim.log_path}")
    return 0 if ok else 1


def iss_keys() -> tuple:
    """`iss.pio_iss.Chip.state`'s keys, read from the ISS itself so the harness cannot drift."""
    return tuple(_GOLDEN_CHIP().state(0))


def _main(argv: list[str]) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--selftest", action="store_true",
                    help="run the plumbing once and print the cycle rate")
    ap.add_argument("--cycles", type=int, default=2000, help="bus ticks for --selftest")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    if not args.selftest:
        ap.error("nothing to do: pass --selftest")
    return selftest(args.cycles, quiet=args.quiet)


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
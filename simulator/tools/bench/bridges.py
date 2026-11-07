"""Bridge and mix workloads for the `iss_p` backend.

A *bridge* is two engines wired through the on-chip engine route (tools/bench/sim/REGMAP.md section 3):
engine A receives a protocol, its RX FIFO feeds engine B's TX FIFO word for word, engine B transmits another
protocol. The host configures the chip once (programs, dividers, CTRL with ROUTE_EN) and then does nothing:
the bridge runs with the host out of the loop. A source VIP feeds A, a sink VIP checks B.

    bridge.uart_rx->uart_tx      UART in at `in_rate` baud, UART out at `out_rate` baud (rate conversion)
    bridge.spi_target->uart_tx   an SPI master writes bytes (mode 0, `in_rate` = SCK Hz), UART out
    bridge.uart_rx->i2c_target   UART in; an I2C controller VIP reads the bytes from a read-only I2C target

A *mix* is k independent protocols on k engines at once, each with its own VIP checker, serviced by a polling
host (one FLEVEL read every `poll` cycles, plus the FIFO pops/pushes it implies):

    mix.uart_tx+uart_rx          full-duplex UART on 2 engines
    mix.uart_tx+spi_rx+i2c_rd    UART out, SPI in, I2C read on 3 engines

Word formats on the route (what a RX word of the source is, what the sink's TX word must be):
    uart_rx   byte at bits 31:24                      uart_tx_hi / i2c_rd_target take this
    spi_rx    byte at bits 7:0                        the stock fw/uart_tx program takes this

Metrics (tools/bench/metrics.py, "task J" section). Definitions:
  bytes_in / bytes_out   bytes the source sent / the sink received.
  bytes_lost             input bytes that do not appear, in order, in the output (longest common subsequence).
  bytes_spurious         output bytes that match no input byte (garbage from a receiver that lost its phase).
                         The accounting is LCS-based, so bytes_in = bytes_out + bytes_lost - bytes_spurious always
                         holds. Spurious bytes are real receiver behaviour, not a harness artefact: a UART receiver
                         whose push stalled resynchronises in the middle of a frame (seen at 115200 -> 57600, burst
                         20, depth 4: 3 lost + 1 spurious), so lost and spurious can both be non-zero.
  peak_buffer_words      max over cycles of len(A.rx) + len(B.tx): words held in the route's two FIFOs.
                         Bytes inside an engine (its ISR/OSR, a push stalled on a full FIFO) are not counted.
  latency_*_cycles       sink finish tick minus source finish tick, per matched byte (sink finish = end of
                         its stop bit / last ACK clock). Includes the output frame time, which is protocol,
                         not bridge; for 9600 baud out that is ~52k cycles.
  sustained_rate_no_loss (assumes loss is monotone in the gap, resolution = the GAP_LADDER steps; see metrics.py)
                         highest source byte rate (bytes/s) of a `burst`-byte stream with no loss, found by
                         stepping the inter-byte gap down a fixed ladder (galloping + bisection); only with search=1.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Optional

PIO_DIR = Path(__file__).with_name("bridges")
CORE_HZ = 50_000_000
NS_PER_TICK = 20.0
I2C_NOTE = ("I2C target rig ACKs any address and has no underrun protection (the master reads only bytes "
            "already buffered; latency is set by the master's read schedule)")
GAP_LADDER = (0, 1, 2, 4, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256)   # idle bit times between source bytes

BRIDGES = ("uart_rx->uart_tx", "spi_target->uart_tx", "uart_rx->i2c_target")
MIXES = ("uart_tx+uart_rx", "uart_tx+spi_rx+i2c_rd")

#: default params per workload id (without the bridge./mix. prefix)
DEFAULTS: dict[str, dict] = {
    "uart_rx->uart_tx": {"in_rate": 115200, "out_rate": 9600, "burst": 12, "gap": 0, "search": 0},
    "spi_target->uart_tx": {"in_rate": 1_000_000, "out_rate": 115200, "burst": 12, "gap": 0, "search": 0},
    "uart_rx->i2c_target": {"in_rate": 115200, "out_rate": 100_000, "burst": 8, "gap": 0, "search": 0},
    "uart_tx+uart_rx": {"baud": 115200, "n": 8, "poll": 2000},
    "uart_tx+spi_rx+i2c_rd": {"baud": 115200, "spi_hz": 200_000, "i2c_hz": 100_000, "n": 8, "poll": 2000},
}


def split_id(wid: str) -> tuple[str, str]:
    kind, _, name = wid.partition(".")
    return kind, name


def data_pattern(n: int, seed: int = 0) -> list[int]:
    """Deterministic test bytes: all distinct for n <= 256 (so a lost or duplicated byte is visible)."""
    return [(0x5A + 73 * (i + seed)) & 0xFF for i in range(n)]


# ---- pio programs ------------------------------------------------------------------------------------

def _assemble(name: str, **defines: int):
    from asm.pioasm import assemble
    text = (PIO_DIR / f"{name}.pio").read_text()
    for k, v in defines.items():
        text, n = re.subn(rf"(?m)^(\.define PUBLIC {k}) \d+", rf"\g<1> {v}", text)
        assert n == 1, (name, k)
    progs = assemble(text)
    assert len(progs) == 1
    return progs[0]


def lcs_pairs(a: list[int], b: list[int]) -> list[tuple[int, int]]:
    """Index pairs (i, j) of a longest common subsequence of a and b."""
    n, m = len(a), len(b)
    t = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n - 1, -1, -1):
        for j in range(m - 1, -1, -1):
            t[i][j] = t[i + 1][j + 1] + 1 if a[i] == b[j] else max(t[i + 1][j], t[i][j + 1])
    out, i, j = [], 0, 0
    while i < n and j < m:
        if a[i] == b[j]:
            out.append((i, j))
            i += 1
            j += 1
        elif t[i + 1][j] >= t[i][j + 1]:
            i += 1
        else:
            j += 1
    return out


def percentile(xs: list[float], p: float) -> float:
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(round(p / 100.0 * len(s) + 0.4999) - 1)))]


# ---- the rig -----------------------------------------------------------------------------------------

class Rig:
    """One iss_p chip on a Bus, the agents' VIPs, a host access counter and the route FIFO probe."""

    def __init__(self, agents: list["Agent"]) -> None:
        from iss import pio_iss
        from verif.vip.core import Bus
        self.agents = agents
        self.chip = pio_iss.Chip()
        self.P = getattr(self.chip, "P")
        nets = [f"p{k}" for k in range(18)]
        push_pull = set()
        for a in agents:
            for pin, net in a.pin_nets().items():
                if nets[pin] != f"p{pin}":
                    raise ValueError(f"pin {pin} claimed twice ({nets[pin]} / {net})")
                nets[pin] = net
            push_pull |= set(a.push_pull_nets())
        self.nets = nets
        self.bus = Bus(ns_per_tick=NS_PER_TICK)
        for n in nets:
            self.bus.add_net(n, push_pull=n in push_pull)
        self.dev = _CountingChip(self.chip, nets)
        self.bus.attach(self.dev)
        self.ctrl = 0
        self.peak = 0
        self.route: Optional[tuple[int, int]] = None
        for a in agents:
            a.attach(self)

    # config-time helpers (host writes through the queue, one per cycle)
    def w(self, addr: int, data: int) -> None:
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def settle(self, extra: int = 2) -> None:
        while not self.dev.idle:
            self.bus.step()
        for _ in range(extra):
            self.bus.step()

    def start(self, links: tuple[int, ...] = ()) -> None:
        """Program every agent's engine, then one CTRL write: enable+restart all, and ROUTE_EN for `links`."""
        from fw import regmap
        for a in self.agents:
            a.configure(self)
        self.settle()
        for a in self.agents:
            a.arm(self)
        self.settle()
        for a in self.agents:
            a.preload(self)
        self.settle()
        ctrl = 0
        for a in self.agents:
            m = 1 << a.engine
            ctrl |= m | (m << 4) | (m << 8)
        for e in links:
            ctrl |= 1 << (regmap.CTRL_ROUTE_SHIFT + e)
        self.ctrl = ctrl
        self.w(regmap.A_CTRL, ctrl)
        self.settle()
        self.dev.reset_counts()

    def step(self) -> None:
        self.bus.step()
        if self.route is not None:
            a, b = self.route
            n = len(self.chip.engines[a].rx) + len(self.chip.engines[b].tx)
            if n > self.peak:
                self.peak = n

    def run_until(self, cond, limit: int) -> bool:
        t_end = self.bus.t + limit
        while self.bus.t < t_end:
            if cond():
                return True
            self.step()
        return cond()


def _chip_pins_class():
    from iss.pio_iss import ChipDevice

    class CountingChip(ChipDevice):
        """ChipDevice with arbitrary net names per pin, counting host register accesses."""

        def __init__(self, chip, nets) -> None:
            super().__init__(chip, name="chip")
            self.nets = list(nets)
            self.reads = self.writes = 0

        def reset_counts(self) -> None:
            self.reads = self.writes = 0

        def tick(self, t, pins):
            pin_in = 0
            for k, net in enumerate(self.nets):
                if pins.get(net, 0):
                    pin_in |= 1 << k
            bus = fut = None
            if self.queue:
                kind, addr, data, fut = self.queue.popleft()
                bus = ("w", addr, data) if kind == "w" else ("r", addr)
                if kind == "w":
                    self.writes += 1
                else:
                    self.reads += 1
            r = self.chip.step(pin_in, bus)
            if fut is not None:
                fut.set_result(r)
            out, oe = self.chip.pin_out, self.chip.pin_oe
            return {net: ((out >> k) & 1 if (oe >> k) & 1 else None) for k, net in enumerate(self.nets)}

    return CountingChip


_CHIP_CLS = None


def _CountingChip(chip, nets):
    global _CHIP_CLS
    if _CHIP_CLS is None:
        _CHIP_CLS = _chip_pins_class()
    return _CHIP_CLS(chip, nets)


# ---- agents: one engine + one VIP each --------------------------------------------------------------

class Agent:
    """An engine running one protocol program and the VIP that talks to it."""
    role = ""          # "source" (VIP sends, chip receives) or "sink" (chip sends, VIP receives)
    name = ""
    fmt = ""           # route word format: "hi" (byte at 31:24) or "lo" (byte at 7:0)

    def __init__(self, engine: int) -> None:
        self.engine = engine

    def pin_nets(self) -> dict[int, str]:
        return {}

    def push_pull_nets(self) -> list[str]:
        return []

    def attach(self, rig: Rig) -> None: ...
    def configure(self, rig: Rig) -> None: ...
    def arm(self, rig: Rig) -> None: ...
    def preload(self, rig: Rig) -> None: ...

    # --- helpers shared by the agents
    def reg(self, rig: Rig, i: int, value: int) -> None:
        from fw import regmap
        rig.w(regmap.sm_addr(self.engine, i), value)

    def load_program(self, rig: Rig, prog) -> None:
        from fw import regmap
        for k, word in enumerate(prog.instrs):
            rig.w(regmap.imem_addr(self.engine, (prog.origin or 0) + k), word)

    def force(self, rig: Rig, word: int) -> None:
        from fw import regmap
        rig.w(regmap.sm_addr(self.engine, regmap.SM_INSTR), word)


class UartTxAgent(Agent):
    """Engine transmits 8N1 at `baud` on `pin` (a uio pin); the VIP UartRx checks the wire.

    fmt "lo": fw/uart_tx's own program (byte at bits 7:0). fmt "hi": tools/bench/bridges/uart_tx_hi.pio.
    """
    role, name = "sink", "uart_tx"

    def __init__(self, engine: int, pin: int, baud: float, fmt: str = "lo") -> None:
        super().__init__(engine)
        self.pin, self.baud, self.fmt = pin, float(baud), fmt
        self.tpb = CORE_HZ / self.baud
        self.pending: list[int] = []       # host-fed bytes (mix mode)

    def pin_nets(self):
        return {self.pin: f"p{self.pin}"}

    def push_pull_nets(self):
        return [f"p{self.pin}"]

    def attach(self, rig):
        from verif.vip.uart import UartRx
        self.vip = rig.bus.attach(UartRx(f"p{self.pin}", ticks_per_bit=self.tpb, data_bits=8, parity="none",
                                         stop_bits=1, latency=1))

    def configure(self, rig):
        from fw.uart_tx import driver as d
        from fw import regmap
        self.prog = d.load_program() if self.fmt == "lo" else _assemble("uart_tx_hi")
        rig.w(regmap.A_CTRL, 0)
        self.load_program(rig, self.prog)
        self.reg(rig, regmap.SM_CLKDIV, d.clkdiv_for_baud(self.baud, 8))
        self.reg(rig, regmap.SM_EXECCTRL, d.execctrl_value(self.prog))
        self.reg(rig, regmap.SM_SHIFTCTRL,
                 d.shiftctrl_value() if self.fmt == "lo" else (1 << 19))      # hi: shift right, no autopull
        self.reg(rig, regmap.SM_PINCTRL, d.pinctrl_value(self.pin, sideset_count=self.prog.sideset_bits))

    def arm(self, rig):
        from fw.uart_tx import driver as d
        self.force(rig, d.jmp_abs(self.prog.labels["start"]))

    @property
    def received(self) -> list[int]:
        return self.vip.values

    def finish_ticks(self) -> list[int]:
        return [int(b.tick + 10 * self.tpb) for b in self.vip.received]

    def busy(self) -> bool:
        return bool(self.vip._in_frame)


class UartRxAgent(Agent):
    """Engine receives 8N1 at `baud` on `pin`; the VIP UartTx sends `data` (`gap` idle bits between bytes)."""
    role, name, fmt = "source", "uart_rx", "hi"

    def __init__(self, engine: int, pin: int, baud: float, data: list[int], gap: float = 0) -> None:
        super().__init__(engine)
        self.pin, self.baud, self.data, self.gap = pin, float(baud), list(data), gap
        self.tpb = CORE_HZ / self.baud
        self.words: list[int] = []         # host-popped RX words (mix mode)

    def pin_nets(self):
        return {self.pin: f"p{self.pin}"}

    def attach(self, rig):
        from verif.vip.uart import UartTx
        self.vip = rig.bus.attach(UartTx(f"p{self.pin}", ticks_per_bit=self.tpb, idle_bits=self.gap))

    def configure(self, rig):
        from fw.uart_rx import driver as d
        from fw import regmap
        self.prog = d.build_program(d.DEFAULT_SM_CYCLES_PER_BIT)
        rig.w(regmap.A_CTRL, 0)
        self.load_program(rig, self.prog)
        self.reg(rig, regmap.SM_CLKDIV, d.clkdiv(self.tpb / d.DEFAULT_SM_CYCLES_PER_BIT))
        self.reg(rig, regmap.SM_EXECCTRL,
                 d.execctrl(wrap_bottom=self.prog.wrap_target, wrap_top=self.prog.wrap, jmp_pin=self.pin))
        self.reg(rig, regmap.SM_SHIFTCTRL, d.shiftctrl(in_right=1, out_right=1))
        self.reg(rig, regmap.SM_PINCTRL, d.pinctrl(in_base=self.pin, set_base=self.pin, set_count=1))

    def arm(self, rig):
        self.force(rig, 0xE080)                              # set pindirs, 0
        self.force(rig, self.prog.labels["hunt"])            # jmp hunt

    def start_source(self) -> None:
        self.vip.send(self.data)

    def done(self) -> bool:
        return self.vip.idle

    def finish_ticks(self) -> list[int]:
        return [f.end_tick for f in self.vip.tx_log if f.end_tick is not None]


class SpiRxAgent(Agent):
    """Engine is an SPI mode-0 receive-only target (spi_rx.pio); the VIP SpiController writes `data`."""
    role, name, fmt = "source", "spi_rx", "lo"
    MOSI, SCK, CS, MISO = 9, 10, 12, 13

    def __init__(self, engine: int, sck_hz: float, data: list[int], gap: float = 0) -> None:
        super().__init__(engine)
        self.half = max(3, int(round(CORE_HZ / (2.0 * sck_hz))))
        self.data, self.gap = list(data), gap
        self.words: list[int] = []

    def pin_nets(self):
        return {self.MOSI: "mosi", self.SCK: "sck", self.CS: "cs_n", self.MISO: "miso"}

    def push_pull_nets(self):
        return ["mosi", "sck", "cs_n", "miso"]

    def attach(self, rig):
        from verif.vip.spi import SpiController
        self.vip = rig.bus.attach(SpiController(mode=0, half_period=self.half, cs_setup=4, cs_hold=4,
                                                cs_gap=max(4, int(self.gap * 2 * self.half))))

    def configure(self, rig):
        from fw.uart_rx import driver as d
        from fw import regmap
        self.prog = _assemble("spi_rx", SCK_PIN=self.SCK)
        rig.w(regmap.A_CTRL, 0)
        self.load_program(rig, self.prog)
        self.reg(rig, regmap.SM_CLKDIV, d.clkdiv(1.0))
        self.reg(rig, regmap.SM_EXECCTRL, d.execctrl(wrap_bottom=self.prog.wrap_target, wrap_top=self.prog.wrap))
        self.reg(rig, regmap.SM_SHIFTCTRL, d.shiftctrl(in_right=0, autopush=1, push_thresh=8))
        self.reg(rig, regmap.SM_PINCTRL, d.pinctrl(in_base=self.MOSI, set_base=0, set_count=0))

    def arm(self, rig):
        self.force(rig, 0x0000 | self.prog.labels["bit"])    # jmp bit

    def start_source(self) -> None:
        chunk = 1 if self.gap > 0 else len(self.data)
        for i in range(0, len(self.data), chunk):
            self.vip.queue_transfer(self.data[i:i + chunk])

    def done(self) -> bool:
        return self.vip.idle()

    def finish_ticks(self) -> list[int]:
        out = []
        for x in self.vip.transfers:
            for i in range(len(x.mosi)):
                out.append(x.t_start + 4 + (i + 1) * 2 * self.half)
        return out


class I2cRdAgent(Agent):
    """Engine is a read-only I2C target (i2c_rd_target.pio); the VIP I2cController reads from it."""
    role, name, fmt = "sink", "i2c_rd", "hi"

    def __init__(self, engine: int, sda: int, scl: int, scl_hz: float = 100_000) -> None:
        super().__init__(engine)
        self.sda, self.scl, self.scl_hz = sda, scl, scl_hz
        self.pending: list[int] = []

    def pin_nets(self):
        return {self.sda: f"p{self.sda}", self.scl: f"p{self.scl}"}

    def attach(self, rig):
        from verif.vip.i2c import I2cController

        class Ctl(I2cController):
            def _rx_byte(s, t):
                s.rx_ticks.append(t)
                super()._rx_byte(t)

        mode = "standard" if self.scl_hz <= 100_000 else "fast"
        self.vip = Ctl(address=0x50, mode=mode, bus=rig.bus, scl_net=f"p{self.scl}", sda_net=f"p{self.sda}",
                       name="i2c_ctrl")
        self.vip.rx_ticks = []
        rig.bus.attach(self.vip)

    def configure(self, rig):
        from fw.uart_rx import driver as d
        from fw import regmap
        self.prog = _assemble("i2c_rd_target", SDA_PIN=self.sda, SCL_PIN=self.scl)
        rig.w(regmap.A_CTRL, 0)
        self.load_program(rig, self.prog)
        self.reg(rig, regmap.SM_CLKDIV, d.clkdiv(1.0))
        self.reg(rig, regmap.SM_EXECCTRL, d.execctrl(wrap_bottom=0, wrap_top=31, jmp_pin=self.sda))
        self.reg(rig, regmap.SM_SHIFTCTRL, d.shiftctrl(in_right=0, out_right=0))
        self.reg(rig, regmap.SM_PINCTRL, d.pinctrl(in_base=self.sda, set_base=self.sda, set_count=1,
                                                   out_base=self.sda, out_count=1))

    def arm(self, rig):
        self.force(rig, 0x0000 | self.prog.labels["idle"])   # jmp idle

    def finish_ticks(self) -> list[int]:
        return list(self.vip.rx_ticks)


# ---- the generic run loop ---------------------------------------------------------------------------

def _stats(data: list[int], out: list[int], in_done: list[int], out_done: list[int]) -> dict:
    pairs = lcs_pairs(data, out)
    matched = len(pairs)
    res = {"bytes_in": len(data), "bytes_out": len(out), "bytes_lost": len(data) - matched,
           "bytes_spurious": len(out) - matched}
    lat = []
    for i, j in pairs:
        if i < len(in_done) and j < len(out_done):
            lat.append(out_done[j] - in_done[i])
    res["latency_first_byte_cycles"] = lat[0] if lat else None
    res["latency_p99_cycles"] = percentile(lat, 99) if lat else None
    res["latencies"] = lat
    return res


def _run_bridge_once(name: str, cfg: dict, p: dict, gap: float) -> dict:
    """One trial: build the rig, run the burst, return the stats dict."""
    n = int(p["burst"])
    data = data_pattern(n)
    if name == "uart_rx->uart_tx":
        src = UartRxAgent(0, 8, p["in_rate"], data, gap)
        snk = UartTxAgent(1, 0, p["out_rate"], fmt="hi")
    elif name == "spi_target->uart_tx":
        src = SpiRxAgent(0, p["in_rate"], data, gap)
        snk = UartTxAgent(1, 0, p["out_rate"], fmt="lo")
    elif name == "uart_rx->i2c_target":
        src = UartRxAgent(0, 8, p["in_rate"], data, gap)
        snk = I2cRdAgent(1, 2, 11, p["out_rate"])
    else:
        raise ValueError(name)
    rig = Rig([src, snk])
    rig.route = (0, 1)
    rig.start(links=(0,))
    src.start_source()
    in_frame = 10 * CORE_HZ / p["in_rate"] if src.name == "uart_rx" else 8 * 2 * src.half
    out_frame = 10 * CORE_HZ / p["out_rate"] if snk.name == "uart_tx" else 9 * CORE_HZ / p["out_rate"]
    total = int((n + 4) * (in_frame + out_frame + (gap * in_frame / 10)) * 1.5) + 20000
    eA, eB = rig.chip.engines[0], rig.chip.engines[1]
    if snk.name == "uart_tx":
        quiet = int(4 * out_frame)
        state = {"t": 0}

        def done():
            if len(snk.vip.received) >= n:
                return True
            if src.done() and not eA.rx and not eB.tx and not snk.busy():
                if state["t"] == 0:
                    state["t"] = rig.bus.t
                return rig.bus.t - state["t"] > quiet
            state["t"] = 0
            return False
        rig.run_until(done, total)
        out, out_done = snk.received, snk.finish_ticks()
    else:
        rig.run_until(lambda: src.done(), total)
        out, out_done = [], []
        idle_rounds = 0
        while idle_rounds < 2:
            # let the engine and the route settle: two UART frames of nothing new in the FIFOs
            before = len(eA.rx) + len(eB.tx)
            rig.run_until(lambda: False, int(2 * (10 * CORE_HZ / p["in_rate"])))
            avail = len(eA.rx) + len(eB.tx)
            if avail == 0:
                idle_rounds += 1
                continue
            idle_rounds = 0
            op = snk.vip.queue_read(avail)
            rig.run_until(lambda: snk.vip.idle, int(avail * out_frame * 3) + 40000)
            out += list(op.data)
        out_done = snk.finish_ticks()
    stats = _stats(data, out, src.finish_ticks(), out_done)
    stats["peak_buffer_words"] = rig.peak
    stats["cycles"] = rig.bus.t
    stats["host_accesses"] = rig.dev.reads + rig.dev.writes
    stats["src_gap_bits"] = gap
    stats["in_frame_cycles"] = in_frame
    return stats


def run_bridge(name: str, cfg: dict, params: dict) -> dict:
    """Run bridge `name` (id without the `bridge.` prefix) at `cfg`; return {metric: (value, note)}."""
    p = {**DEFAULTS[name], **params}
    if not cfg.get("engine_route") or cfg.get("n_engines", 2) < 2:
        raise ValueError("bridges need engine_route=true and n_engines >= 2")
    st = _run_bridge_once(name, cfg, p, p["gap"])
    out = {
        "bytes_in": (st["bytes_in"], ""), "bytes_out": (st["bytes_out"], ""),
        "bytes_lost": (st["bytes_lost"], "input bytes missing from the output (LCS)"),
        "bytes_spurious": (st["bytes_spurious"], "output bytes matching no input byte"),
        "peak_buffer_words": (st["peak_buffer_words"], f"len(A.rx)+len(B.tx), capacity {2 * cfg['fifo_depth']}"),
        "latency_first_byte_cycles": (st["latency_first_byte_cycles"], "sink finish - source finish, includes the output frame"),
        "latency_p99_cycles": (st["latency_p99_cycles"], "over matched bytes"),
        "host_port_utilization": (0.0 if st["host_accesses"] == 0 else None,
                                  f"{st['host_accesses']} host register accesses after configuration"),
    }
    if p.get("search"):
        out["sustained_rate_no_loss"] = _sustained(name, cfg, p, st)
    else:
        out["sustained_rate_no_loss"] = (None, "not searched (search=0)")
    if name == "uart_rx->i2c_target":
        for k in ("bytes_out", "latency_first_byte_cycles", "latency_p99_cycles"):
            out[k] = (out[k][0], (out[k][1] + "; " if out[k][1] else "") + I2C_NOTE)
    out["_pass"] = (st["bytes_lost"] == 0 and st["bytes_spurious"] == 0, "")
    return out


def _byte_rate(name: str, p: dict, gap: float) -> float:
    """Source byte rate (bytes/s) with `gap` idle bit times between bytes."""
    if name == "spi_target->uart_tx":
        half = max(3, int(round(CORE_HZ / (2.0 * p["in_rate"]))))
        return CORE_HZ / (8 * 2 * half + max(4, int(gap * 2 * half)) + 8) if gap > 0 else CORE_HZ / (16 * half)
    return p["in_rate"] / (10 + gap)


def _sustained(name: str, cfg: dict, p: dict, first: dict):
    """Binary search over GAP_LADDER for the smallest gap with no loss (assumes loss shrinks with gap)."""
    def ok(g):
        st = first if g == p["gap"] else _run_bridge_once(name, cfg, p, g)
        return st["bytes_lost"] == 0 and st["bytes_spurious"] == 0
    if ok(GAP_LADDER[0]):
        return (_byte_rate(name, p, 0), "no loss even back to back: the burst fits the buffers; limited by the source")
    lo, hi = 0, 1                       # gallop up the ladder (cheap trials first), then bisect
    while not ok(GAP_LADDER[hi]):
        lo = hi
        if hi == len(GAP_LADDER) - 1:
            return (None, f"loses bytes even with {GAP_LADDER[hi]} idle bits between bytes")
        hi = min(2 * hi + 1, len(GAP_LADDER) - 1)
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ok(GAP_LADDER[mid]):
            hi = mid
        else:
            lo = mid
    g = GAP_LADDER[hi]
    return (_byte_rate(name, p, g), f"bytes/s at gap={g} idle bits (ladder step; gap {GAP_LADDER[lo]} loses); assumes monotone loss, "
                  f"resolution = ladder step")


# ---- mixes ------------------------------------------------------------------------------------------

class HostPoller:
    """The host of a mix, behind a host port that is busy while it serves an access.

    Every `period` cycles it wants a FLEVEL read, then the RX pops / TX pushes the levels imply. Each access is
    held back until the port is free (`busy_until`) and then occupies it for the measured cost of that access
    (verif/perf/hostport.py: the same numbers as the spi_host_port workload), so the host cannot do more than the
    SPI port carries. `late` is the worst delay of a scheduled poll; `infeasible` = a poll slipped by more than a
    period (the port cannot keep up with this poll rate) or work was still queued when the run ended.
    """

    def __init__(self, rig: Rig, period: int) -> None:
        from fw import regmap
        from verif.perf import hostport
        self.rig, self.period, self.next = rig, period, 0
        cost = {c.name: c.cycles for c in hostport.measure()}
        self.cost = {"r": cost["read"], "w": cost["write"]}
        self.busy_until = 0
        self.ops: list = []                  # ("r", addr, tag) | ("w", addr, data)
        self.futs: list = []                 # (tag, future)
        self.fl = None
        self.late = 0
        self.depth = rig.P.fifo_depth
        self.TXF, self.RXF = regmap.txf_addrs(), regmap.rxf_addrs()
        self.A_FLEVEL = regmap.A_FLEVEL
        self.rx_words: dict[int, list[int]] = {}

    @property
    def infeasible(self) -> bool:
        return self.late > self.period or bool(self.ops)

    def tick(self) -> None:
        r = self.rig
        t = r.bus.t
        for item in list(self.futs):
            tag, f = item
            if f.done():
                self.futs.remove(item)
                if tag == "fl":
                    self._plan(f.result())
                else:
                    self.rx_words.setdefault(tag, []).append(f.result())
        if not self.ops and self.fl is None and not self.futs and t >= self.next:
            self.late = max(self.late, t - self.next)
            self.ops.append(("r", self.A_FLEVEL, "fl"))
            self.next = max(self.next + self.period, t)
        if self.ops and t >= self.busy_until and r.dev.idle:
            kind, addr, arg = self.ops.pop(0)
            self.busy_until = t + self.cost[kind]
            if kind == "r":
                self.futs.append((arg, r.dev.read(addr)))
            else:
                r.dev.write(addr, arg)

    def _plan(self, lv: int) -> None:
        for a in self.rig.agents:
            e = a.engine
            if a.role == "source":
                self.ops += [("r", self.RXF[e], e)] * ((lv >> (8 * e + 4)) & 0xF)
            elif a.pending:
                room = self.depth - ((lv >> (8 * e)) & 0xF)
                for _ in range(min(room, len(a.pending))):
                    self.ops.append(("w", self.TXF[e], a.word(a.pending.pop(0))))


def _word_lo(b): return b


def _word_hi(b): return b << 24


UartTxAgent.word = lambda self, b: _word_hi(b) if self.fmt == "hi" else _word_lo(b)
I2cRdAgent.word = lambda self, b: _word_hi(b)


def run_mix(name: str, cfg: dict, params: dict) -> dict:
    p = {**DEFAULTS[name], **params}
    n = int(p["n"])
    need = 2 if name == "uart_tx+uart_rx" else 3
    if cfg.get("n_engines", 2) < need:
        raise ValueError(f"mix {name} needs n_engines >= {need}")
    res: dict[str, tuple] = {}
    if name == "uart_tx+uart_rx":
        d_tx, d_rx = data_pattern(n, 0), data_pattern(n, 5)
        tx = UartTxAgent(0, 0, p["baud"], "lo")
        rx = UartRxAgent(1, 8, p["baud"], d_rx, 0)
        agents = [tx, rx]
        tx.pending = list(d_tx)
        rig = Rig(agents)
        rig.start()
        poller = HostPoller(rig, p["poll"])
        rx.start_source()
        limit = int((n + 3) * 12 * CORE_HZ / p["baud"]) + 20000
        rig.run_until(lambda: (poller.tick() or False) or (len(tx.vip.received) >= n and
                                                            len(poller.rx_words.get(1, [])) >= n), limit)
        # the loop above ticks the host before each step
        got_rx = [w >> 24 for w in poller.rx_words.get(1, []) if not w & 0x00FF_FFFF]
        passes = {"uart_tx": tx.received == d_tx, "uart_rx": got_rx == d_rx}
    else:
        d_tx, d_spi, d_i2c = data_pattern(n, 0), data_pattern(n, 5), data_pattern(n, 9)
        tx = UartTxAgent(0, 0, p["baud"], "lo")
        spi = SpiRxAgent(1, p["spi_hz"], d_spi, 0)
        i2c = I2cRdAgent(2, 2, 11, p["i2c_hz"])
        tx.pending, i2c.pending = list(d_tx), list(d_i2c)
        rig = Rig([tx, spi, i2c])
        rig.start()
        poller = HostPoller(rig, p["poll"])
        spi.start_source()
        op = i2c.vip.queue_read(n)
        limit = int(max((n + 3) * 12 * CORE_HZ / p["baud"], (n + 2) * 10 * CORE_HZ / p["i2c_hz"])) + 40000
        rig.run_until(lambda: (poller.tick() or False) or (len(tx.vip.received) >= n and i2c.vip.idle and
                                                            len(poller.rx_words.get(1, [])) >= n), limit)
        got_spi = [w & 0xFF for w in poller.rx_words.get(1, [])]
        passes = {"uart_tx": tx.received == d_tx, "spi_rx": got_spi == d_spi, "i2c_rd": list(op.data) == d_i2c}
    cycles = rig.bus.t
    for k, v in passes.items():
        res[f"pass_{k}"] = (bool(v), I2C_NOTE if k == "i2c_rd" else "")
    res["all_pass"] = (all(passes.values()) and not poller.infeasible,
                       "host port infeasible" if poller.infeasible else "")
    busy = rig.dev.reads * poller.cost["r"] + rig.dev.writes * poller.cost["w"]
    infeasible = poller.infeasible
    res["host_port_utilization"] = (round(busy / cycles, 4),
                                    f"{rig.dev.reads} reads x {poller.cost['r']} + {rig.dev.writes} writes x "
                                    f"{poller.cost['w']} cycles over {cycles} (measured single-access costs, MVP framing; "
                                    f"a wide map adds a command byte)"
                                    + ("; host port infeasible: polls slipped by up to "
                                       f"{poller.late} cycles (period {p['poll']}) or work was left queued"
                                       if infeasible else ""))
    res["run_cycles"] = (cycles, "")
    res["_pass"] = (all(passes.values()) and not infeasible, "host port infeasible" if infeasible else "")
    return res

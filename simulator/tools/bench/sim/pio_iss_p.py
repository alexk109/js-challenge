"""Parametric copy of the golden ISS (`iss/pio_iss.py`, read-only) for the evidence bench.

Same model, same cycle conventions; the structural sizes are parameters (`Params`):
n_engines 1-4, imem_words 16/32/64/128, fifo_depth 2-16, engine_route, t_in 2-4. Read from the
env var BENCH_CONFIG (JSON of a normalized bench config; knobs not modelled here are ignored) by
`chip_class_from_env()`, which `fw/targets.py` calls when the `iss_p` target is installed. At the
MVP parameters this is cycle-identical to the golden ISS (tools/bench/tests/test_iss_p.py and
tools/bench/sim/lockstep_p.py check it). Register map and engine route: tools/bench/sim/REGMAP.md.
The original docstring follows.

Golden cycle-accurate ISS of the MVP chip (two RP2040-PIO-compatible engines).

Normative sources: docs/mvp-isa.md (semantics, timing, pins) and docs/mvp-regmap.md
(register map). API: docs/mvp-interfaces.md `## ISS API`. Section names quoted in the
comments below (`### Reference step` etc.) are sections of docs/mvp-isa.md.

One call to `Chip.step()` is one core cycle `n`:

  1. Every decision uses the start-of-cycle state (the state "in cycle n").
  2. A host read returns the register's value in cycle n (combinational).
  3. Each engine with an SM cycle runs the reference step on its own start-of-cycle state.
  4. Host writes and read side effects are applied after the engines, so the host wins where
     both touch the same state (the latch), except FDEBUG, where engine sets survive a W1C.
  5. IR, the dividers, the registered pin state, the input synchronizers and the IRQ line
     flops are all clocked at the end of the cycle.

After `step()`, `pin_out`, `pin_oe`, `irq0`, `irq1` and `state(e)` show the state in cycle n+1.
"""
from __future__ import annotations

from collections import deque
from concurrent.futures import Future
from typing import Any, Mapping, Optional

import json
import os
from dataclasses import dataclass

from fw.regmap import MVP as _MVP_MAP, RegMap, CTRL_ROUTE_SHIFT
from verif.vip.core import Device

try:  # the shared constants module named by the spec; absent so far (see SPEC-QUESTIONS.md)
    from iss.isa import T_IN, T_OUT, T_IMEM  # type: ignore
except ImportError:  # pragma: no cover - the normal case today
    T_OUT = 1   # an engine pin write in cycle n is on the pad in cycle n+1
    T_IN = 3    # an engine in cycle n sees the pad value of cycle n-3
    T_IMEM = 2  # a host IMEM write in cycle h is first executed in cycle h+2

# T_OUT = 1 and T_IMEM = 2 are structural here (one register each), T_IN is a pipeline depth.
assert T_OUT == 1 and T_IMEM == 2, "this ISS models T_OUT = 1 and T_IMEM = 2 structurally"

M32 = 0xFFFF_FFFF
NUM_PINS = 18


@dataclass(frozen=True)
class Params:
    """The structural parameters. The defaults are the MVP."""
    n_engines: int = 2
    imem_words: int = 32
    fifo_depth: int = 4
    engine_route: bool = False
    t_in: int = T_IN
    isa_xor: bool = False       # MOV Op 11 = XOR (tools/bench/sim/ISA_EXT.md)
    isa_cond: bool = False      # opcode 110 = JMPC, more JMP conditions
    isa_irq: bool = False       # opcode 110 = IRQ, WAIT source 10 = IRQ flag

    def __post_init__(self) -> None:
        if not 1 <= self.n_engines <= 4:
            raise ValueError(f"n_engines {self.n_engines} not in 1..4")
        if self.imem_words not in (16, 32, 64, 128):
            raise ValueError(f"imem_words {self.imem_words} not in 16/32/64/128")
        if not 2 <= self.fifo_depth <= 16:
            raise ValueError(f"fifo_depth {self.fifo_depth} not in 2..16")
        if not 2 <= self.t_in <= 4:
            raise ValueError(f"t_in {self.t_in} not in 2..4")
        if self.isa_cond and self.isa_irq:
            raise ValueError("isa_cond and isa_irq both need opcode 110 (16-bit encoding): choose one")
        if self.engine_route and self.n_engines < 2:
            raise ValueError("engine_route needs at least 2 engines")

    @classmethod
    def from_config(cls, cfg: Optional[Mapping[str, Any]]) -> "Params":
        cfg = cfg or {}
        d = cls()
        return cls(n_engines=int(cfg.get("n_engines", d.n_engines)),
                   imem_words=int(cfg.get("imem_words", d.imem_words)),
                   fifo_depth=int(cfg.get("fifo_depth", d.fifo_depth)),
                   engine_route=bool(cfg.get("engine_route", d.engine_route)),
                   t_in=int(cfg.get("t_in", d.t_in)),
                   isa_xor=bool(cfg.get("isa_xor", d.isa_xor)),
                   isa_cond=bool(cfg.get("isa_cond", d.isa_cond)),
                   isa_irq=bool(cfg.get("isa_irq", d.isa_irq)))

    @classmethod
    def from_env(cls) -> "Params":
        raw = os.environ.get("BENCH_CONFIG", "").strip()
        return cls.from_config(json.loads(raw) if raw else None)

    @property
    def regmap(self) -> RegMap:
        return RegMap(self.n_engines, self.imem_words, self.fifo_depth, self.engine_route)

    @property
    def page_bits(self) -> int:
        """Width of EXECCTRL.PAGE: log2(imem_words / 32), 0 up to 32 words."""
        return max(0, (self.imem_words // 32).bit_length() - 1)


MVP_PARAMS = Params()
PAGE_SHIFT = 20                 # EXECCTRL[21:20] = PAGE (non-MVP: imem_words > 32)

# Pin kinds (docs/mvp-isa.md `### The 18 protocol pins`).
UIO_MASK = 0x000FF          # 0..7   bidirectional
IN_ONLY_MASK = 0x01F00      # 8..12  input only
OUT_ONLY_MASK = 0x3E000     # 13..17 output only
PIN_MASK = 0x3FFFF
LEVEL_WRITABLE = UIO_MASK | OUT_ONLY_MASK   # pins with a level register
DIR_WRITABLE = UIO_MASK                     # pins with a direction register

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) ----
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = 0x0F, 0x10, 0x11
A_SM_BASE = 0x32               # SMe_* at 0x32 + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)
A_INTR = 0x4A
A_IRQ0_INTE, A_IRQ0_INTF, A_IRQ0_INTS = 0x4B, 0x4C, 0x4D
A_IRQ1_INTE, A_IRQ1_INTF, A_IRQ1_INTS = 0x4E, 0x4F, 0x50

# DBG_CFGINFO = FIFO_DEPTH 5:0 | SM_COUNT 11:8 | IMEM_SIZE 23:16 (MVP: 0x0020_0204)

# Reset values and stored-bit masks of the per-engine registers.
CLKDIV_RESET, CLKDIV_MASK = 0x0001_0000, 0xFFFF_FF00
EXECCTRL_RESET = 0x0001_F000
EXECCTRL_MASK = 0x7F01_FF9F    # 30 SIDE_EN, 29 SIDE_PINDIR, 28:24 JMP_PIN, 16:12, 11:7, 4, 3:0
SHIFTCTRL_RESET, SHIFTCTRL_MASK = 0x000C_0000, 0x3FFF_0000   # 29:16
PINCTRL_RESET, PINCTRL_MASK = 0x1400_0000, 0xFFFF_FFFF
# INTE/INTF stored bits: TXNFULL at 4+e, RXNEMPTY at e, for e < n_engines (MVP: 0x33)
INTE0_RESET, INTE1_RESET = 0x003, 0x010

# Instruction sources in the reference step.
SRC_IMEM, SRC_LATCH, SRC_HOST = 0, 1, 2


def thresh(f: int) -> int:
    return f if f else 32


def mask(k: int) -> int:
    return M32 if k >= 32 else (1 << k) - 1


def rotr32(v: int, s: int) -> int:
    s &= 31
    return ((v >> s) | (v << (32 - s))) & M32 if s else v


def bitrev32(v: int) -> int:
    return int(f"{v & M32:032b}"[::-1], 2)


def window(base: int, count: int, data: int) -> tuple[int, int]:
    """(mask, value) over the 32-pin space: data bit k -> pin (base + k) mod 32, k < count."""
    m = v = 0
    for k in range(min(count, 32)):
        p = (base + k) & 31
        m |= 1 << p
        if (data >> k) & 1:
            v |= 1 << p
        else:
            v &= ~(1 << p)
    return m, v


def _req18(req: Optional[tuple[int, int, int]]) -> Optional[tuple[int, int, int]]:
    """A pin write request over the 18 pins that exist, for `Chip.deep_state`.

    `window()` computes over the 32-pin virtual space and the pin resolution drops everything
    above pin 17 ("## Pins"), so the mask and the value are cut here rather than in the caller.
    """
    if req is None:
        return None
    is_dir, mask, val = req
    return is_dir, mask & PIN_MASK, val & PIN_MASK


class Engine:
    """One engine: its state, its configuration registers, its IMEM and its FIFOs."""

    def __init__(self, idx: int, P: Params = MVP_PARAMS) -> None:
        self.idx = idx
        self.P = P
        self.depth = P.fifo_depth
        self.pmask = P.imem_words - 1           # PC width
        self.execctrl_mask = EXECCTRL_MASK | (((1 << P.page_bits) - 1) << PAGE_SHIFT)
        self.reset()

    def reset(self) -> None:
        self.imem = [0] * self.P.imem_words
        self.pc = 0
        self.ir = 0
        self.x = self.y = self.osr = self.isr = 0
        self.osr_cnt = 32
        self.isr_cnt = 0
        self.delay = 0
        self.stalled = 0
        self.latch = 0
        self.latch_pend = 0
        self.latch_host = 0
        self.force = 0
        self.jp_prev = None         # isa_cond: JMP_PIN at the previous SM cycle (None until the first)
        self.jp_rise = self.jp_fall = 0
        self.irqw = 0               # isa_irq: this engine has raised the flag of a pending `irq wait`
        self.irq_in = 0             # isa_irq: chip flags at the start of the cycle (set by Chip.step)
        self._irq_set = self._irq_clr = 0
        self.tx: list[int] = []
        self.rx: list[int] = []
        self.div_cnt, self.div_acc, self.en = 1, 0, 0
        self.clkdiv = CLKDIV_RESET
        self.execctrl = EXECCTRL_RESET
        self.shiftctrl = SHIFTCTRL_RESET
        self.pinctrl = PINCTRL_RESET

    # ---- configuration fields ----
    @property
    def side_en(self) -> int: return (self.execctrl >> 30) & 1
    @property
    def side_pindir(self) -> int: return (self.execctrl >> 29) & 1
    @property
    def jmp_pin(self) -> int: return (self.execctrl >> 24) & 0x1F
    @property
    def page_base(self) -> int: return (((self.execctrl >> PAGE_SHIFT) & 3) << 5) & self.pmask
    @property
    def wrap_top(self) -> int: return self.page_base | ((self.execctrl >> 12) & 0x1F)
    @property
    def wrap_bottom(self) -> int: return self.page_base | ((self.execctrl >> 7) & 0x1F)
    @property
    def status_sel(self) -> int: return (self.execctrl >> 4) & 1
    @property
    def status_n(self) -> int: return self.execctrl & 0xF
    @property
    def pull_thresh(self) -> int: return thresh((self.shiftctrl >> 25) & 0x1F)
    @property
    def push_thresh(self) -> int: return thresh((self.shiftctrl >> 20) & 0x1F)
    @property
    def out_shiftdir(self) -> int: return (self.shiftctrl >> 19) & 1
    @property
    def in_shiftdir(self) -> int: return (self.shiftctrl >> 18) & 1
    @property
    def autopull(self) -> int: return (self.shiftctrl >> 17) & 1
    @property
    def autopush(self) -> int: return (self.shiftctrl >> 16) & 1
    @property
    def sideset_count(self) -> int: return min((self.pinctrl >> 29) & 7, 5)
    @property
    def set_count(self) -> int: return (self.pinctrl >> 26) & 7
    @property
    def out_count(self) -> int: return min((self.pinctrl >> 20) & 0x3F, 32)
    @property
    def in_base(self) -> int: return (self.pinctrl >> 15) & 0x1F
    @property
    def sideset_base(self) -> int: return (self.pinctrl >> 10) & 0x1F
    @property
    def set_base(self) -> int: return (self.pinctrl >> 5) & 0x1F
    @property
    def out_base(self) -> int: return self.pinctrl & 0x1F

    @property
    def exec_stalled(self) -> int:
        return self.latch_pend & self.latch_host

    # ---- decode helpers ----
    def delay_field(self, instr: int) -> int:
        return ((instr >> 8) & 0x1F) & mask(5 - self.sideset_count)

    def sideset(self, instr: int) -> Optional[tuple[int, int]]:
        """(data, pin count) if this instruction side-sets, else None."""
        sc = self.sideset_count
        if sc == 0:
            return None
        bits = ((instr >> 8) & 0x1F) >> (5 - sc)
        if self.side_en:
            if not (bits >> (sc - 1)) & 1:
                return None
            return bits & mask(sc - 1), sc - 1
        return bits, sc

    @staticmethod
    def is_exec(instr: int) -> bool:
        """OUT EXEC, or a MOV EXEC that is not made a NOP by a reserved Op or Source."""
        op, dst = instr >> 13, (instr >> 5) & 7
        if op == 3:
            return dst == 7
        if op == 5:
            return dst == 4 and ((instr >> 3) & 3) != 3 and (instr & 7) != 4
        return False

    # ---- the reference step (docs/mvp-isa.md `### Reference step`) ----
    def sm_step(self, in_vec: int, fd: dict) -> dict:
        """Run one SM cycle. Reads only start-of-cycle values; returns pin write requests
        {'out': (is_dir, mask, val) | None, 'side': ...}. Sets bits in fd['txstall'/'rxstall']."""
        self._tx_n = len(self.tx)      # start-of-cycle FIFO levels
        self._rx_n = len(self.rx)
        self._fd = fd
        self._req = {"out": None, "side": None}
        self._new_pc = None
        self._irq_set = self._irq_clr = 0
        stalled0 = self.stalled
        latch_host0 = self.latch_host

        if self.force:
            instr, src = self.latch, SRC_HOST
        elif self.latch_pend:
            instr, src = self.latch, SRC_LATCH
        elif self.delay > 0:
            self.delay -= 1
            self._background_autopull()
            self.force = 0
            return self._req
        else:
            instr, src = self.ir, SRC_IMEM

        # 1. side-set, first cycle of an instruction only
        ss = self.sideset(instr)
        if ss is not None and (src == SRC_HOST or not stalled0):
            data, n = ss
            m, v = window(self.sideset_base, n, data)
            self._req["side"] = (self.side_pindir, m, v)

        # Config used by bookkeeping is start-of-cycle; capture before execute (it cannot
        # change config, but keep the order explicit).
        pc0 = self.pc
        delay_val = self.delay_field(instr)

        # 2. main operation
        stall = self._execute(instr, in_vec)

        # 3. bookkeeping
        exec_ = self.is_exec(instr)
        if stall:
            self.stalled = 1
            if src != SRC_IMEM:
                self.latch = instr
                self.latch_pend = 1
                self.latch_host = 1 if (src == SRC_HOST or latch_host0) else 0
        else:
            self.stalled = 0
            if src != SRC_IMEM and not exec_:
                self.latch_pend = 0
            # pc_update
            if self._new_pc is not None:
                self.pc = self._jump_pc(self._new_pc)
            elif src != SRC_IMEM:
                self.pc = pc0
            elif pc0 == self.wrap_top:
                self.pc = self.wrap_bottom & self.pmask   # PC is log2(imem_words) bits: the wrap target is truncated like a jump
            else:
                self.pc = (pc0 + 1) & self.pmask
            if not exec_ and not (src != SRC_IMEM and latch_host0):
                self.delay = delay_val
        if instr >> 13 != 3:
            self._background_autopull()
        self.force = 0
        return self._req

    def _irq_flag(self, idx: int) -> int:
        """Flag number of an IRQ/WAIT index field: bit 4 = rel adds this engine's number to bits 1:0 (mod 4)."""
        f = idx & 7
        if idx & 0x10:
            f = (f & 4) | (((f & 3) + self.idx) & 3)
        return f

    def _jump_pc(self, addr5: int) -> int:
        """THE place a taken JMP / JMPC / OUT PC / MOV PC address becomes a PC (page base, PC width)."""
        return (self.page_base | (addr5 & 0x1F)) & self.pmask

    def _edge_track(self, in_vec: int) -> None:
        """isa_cond: latch JMP_PIN edges every CORE cycle (not only SM cycles, so a pulse shorter than the poll is kept);
        an edge seen this cycle wins over a same-cycle clear (Chip.step calls this after the engines)."""
        if not self.P.isa_cond:
            return
        jp = (in_vec >> self.jmp_pin) & 1
        if self.jp_prev is not None:
            self.jp_rise |= int(jp and not self.jp_prev)
            self.jp_fall |= int(self.jp_prev and not jp)
        self.jp_prev = jp

    def _pop_tx(self) -> int:
        assert self._tx_n > 0 and self.tx
        self._tx_n = 0  # at most one pop per cycle; makes a second pop impossible
        return self.tx.pop(0)

    def _background_autopull(self) -> None:
        if self.autopull and self.osr_cnt >= self.pull_thresh and self._tx_n > 0:
            self.osr = self._pop_tx()
            self.osr_cnt = 0

    def _out_req(self, is_dir: int, base: int, count: int, val: int) -> None:
        m, v = window(base, count, val)
        self._req["out"] = (is_dir, m, v)

    def _execute(self, instr: int, in_vec: int) -> bool:
        """Main operation of one instruction. Returns True if it stalls."""
        op = instr >> 13
        a = (instr >> 5) & 7
        idx = instr & 0x1F
        n = idx or 32
        x0, y0, isr0, osr0 = self.x, self.y, self.isr, self.osr
        osr_cnt0, isr_cnt0 = self.osr_cnt, self.isr_cnt
        in_rot = rotr32(in_vec, self.in_base)

        if op == 0:  # JMP
            if a == 0:
                take = True
            elif a == 1:
                take = x0 == 0
            elif a == 2:
                take = x0 != 0
                self.x = (x0 - 1) & M32
            elif a == 3:
                take = y0 == 0
            elif a == 4:
                take = y0 != 0
                self.y = (y0 - 1) & M32
            elif a == 5:
                take = x0 != y0
            elif a == 6:
                take = bool((in_vec >> self.jmp_pin) & 1)
            else:
                take = osr_cnt0 < self.pull_thresh
            if take:
                self._new_pc = idx
            return False

        if op == 1:  # WAIT
            pol = (instr >> 7) & 1
            srcw = (instr >> 5) & 3
            if srcw == 0:
                bit = (in_vec >> idx) & 1
            elif srcw == 1:
                bit = (in_vec >> ((self.in_base + idx) & 31)) & 1
            elif srcw == 2:
                if self.P.isa_irq:
                    f = self._irq_flag(idx)
                    bit = (self.irq_in >> f) & 1
                    if bit == pol and pol:
                        self._irq_clr |= 1 << f     # a WAIT 1 that passes lowers the flag
                    return bit != pol
                bit = 0                     # IRQ flags hardwired to 0
            else:
                return False                # reserved: NOP
            return bit != pol

        if op == 2:  # IN
            if a in (4, 5):
                return False                # reserved: NOP
            srcv = {0: in_rot, 1: x0, 2: y0, 3: 0, 6: isr0, 7: osr0}[a]
            d = srcv & mask(n)
            if n == 32:
                new_isr = d
            elif self.in_shiftdir:
                new_isr = ((isr0 >> n) | (d << (32 - n))) & M32
            else:
                new_isr = ((isr0 << n) | d) & M32
            cnt = min(32, isr_cnt0 + n)
            if self.autopush and cnt >= self.push_thresh:
                if self._rx_n >= self.depth:
                    self._fd["rxstall"] = True
                    return True             # atomic: nothing shifts
                self.rx.append(new_isr)
                self.isr, self.isr_cnt = 0, 0
            else:
                self.isr, self.isr_cnt = new_isr, cnt
            return False

        if op == 3:  # OUT
            if self.autopull and osr_cnt0 >= self.pull_thresh:
                if self._tx_n > 0:
                    self.osr = self._pop_tx()
                    self.osr_cnt = 0
                else:
                    self._fd["txstall"] = True
                return True
            if self.out_shiftdir:
                v = osr0 & mask(n)
                self.osr = 0 if n == 32 else osr0 >> n
            else:
                v = (osr0 >> (32 - n)) & mask(n)
                self.osr = 0 if n == 32 else (osr0 << n) & M32
            self.osr_cnt = min(32, osr_cnt0 + n)
            if a == 0:
                self._out_req(0, self.out_base, self.out_count, v)
            elif a == 1:
                self.x = v
            elif a == 2:
                self.y = v
            elif a == 3:
                pass
            elif a == 4:
                self._out_req(1, self.out_base, self.out_count, v)
            elif a == 5:
                self._new_pc = v & 0x1F
            elif a == 6:
                self.isr, self.isr_cnt = v, n
            else:
                self.latch, self.latch_pend, self.latch_host = v & 0xFFFF, 1, 0
            if self.autopull and self.osr_cnt >= self.pull_thresh and self._tx_n > 0:
                self.osr = self._pop_tx()
                self.osr_cnt = 0
            return False

        if op == 4:  # PUSH / PULL (bits 4:0 ignored)
            if_flag = (instr >> 6) & 1
            block = (instr >> 5) & 1
            if not (instr >> 7) & 1:        # PUSH
                if if_flag and isr_cnt0 < self.push_thresh:
                    return False
                if self._rx_n >= self.depth:
                    self._fd["rxstall"] = True
                    if block:
                        return True
                    self.isr, self.isr_cnt = 0, 0
                    return False
                self.rx.append(isr0)
                self.isr, self.isr_cnt = 0, 0
                return False
            # PULL
            if self.autopull and osr_cnt0 == 0:
                return False
            if if_flag and osr_cnt0 < self.pull_thresh:
                return False
            if self._tx_n == 0:
                if block:
                    self._fd["txstall"] = True
                    return True
                self.osr, self.osr_cnt = x0, 0
                return False
            self.osr = self._pop_tx()
            self.osr_cnt = 0
            return False

        if op == 5:  # MOV
            mop = (instr >> 3) & 3
            msrc = instr & 7
            if mop == 3 and self.P.isa_xor:     # XOR: dst ^= src, X/Y/ISR/OSR only (ISA_EXT.md)
                if a not in (1, 2, 6, 7) or msrc == 4:
                    return False
                if msrc == 5:
                    level = self._rx_n if self.status_sel else self._tx_n
                    sv = M32 if level < self.status_n else 0
                else:
                    sv = {0: in_rot, 1: x0, 2: y0, 3: 0, 6: isr0, 7: osr0}[msrc]
                if a == 1:
                    self.x = x0 ^ sv
                elif a == 2:
                    self.y = y0 ^ sv
                elif a == 6:
                    self.isr, self.isr_cnt = isr0 ^ sv, 0
                else:
                    self.osr, self.osr_cnt = osr0 ^ sv, 0
                return False
            if a == 3 or mop == 3 or msrc == 4:
                return False                # reserved: NOP
            if msrc == 5:
                level = self._rx_n if self.status_sel else self._tx_n
                v = M32 if level < self.status_n else 0
            else:
                v = {0: in_rot, 1: x0, 2: y0, 3: 0, 6: isr0, 7: osr0}[msrc]
            if mop == 1:
                v = ~v & M32
            elif mop == 2:
                v = bitrev32(v)
            if a == 0:
                self._out_req(0, self.out_base, self.out_count, v)
            elif a == 1:
                self.x = v
            elif a == 2:
                self.y = v
            elif a == 4:
                self.latch, self.latch_pend, self.latch_host = v & 0xFFFF, 1, 0
            elif a == 5:
                self._new_pc = v & 0x1F
            elif a == 6:
                self.isr, self.isr_cnt = v, 0
            else:
                self.osr, self.osr_cnt = v, 0
            return False

        if op == 6:
            if self.P.isa_cond:             # JMPC (ISA_EXT.md); Condition in bits 7:5
                if a == 0:
                    take = x0 == y0
                elif a == 1:
                    take = x0 < y0
                elif a == 2:
                    take = y0 < x0
                elif a == 3:
                    take = not (in_vec >> self.jmp_pin) & 1
                elif a == 4:
                    take = bool(self.jp_rise)
                    self.jp_rise = 0
                elif a == 5:
                    take = bool(self.jp_fall)
                    self.jp_fall = 0
                elif a == 6:
                    take = osr_cnt0 >= self.pull_thresh
                else:
                    take = isr_cnt0 >= self.push_thresh
                if take:
                    self._new_pc = idx
                return False
            if self.P.isa_irq and not (instr >> 7) & 1:     # IRQ: bit 6 Clr, bit 5 Wait, bits 4:0 index
                f = self._irq_flag(idx)
                if (instr >> 6) & 1:
                    self._irq_clr |= 1 << f
                    return False
                if not (instr >> 5) & 1:
                    self._irq_set |= 1 << f
                    return False
                if not self.irqw:           # irq wait: raise once, then stall until the flag reads 0
                    self.irqw = 1
                    self._irq_set |= 1 << f
                    return True
                if (self.irq_in >> f) & 1:
                    return True
                self.irqw = 0
                return False
            return False                    # IRQ: NOP in the MVP

        # op == 7: SET
        if a == 0:
            self._out_req(0, self.set_base, self.set_count, idx)
        elif a == 1:
            self.x = idx
        elif a == 2:
            self.y = idx
        elif a == 4:
            self._out_req(1, self.set_base, self.set_count, idx)
        return False                        # 011, 101, 110, 111 reserved: NOP

    # ---- end-of-cycle register updates that do not depend on executing ----
    def divider_next(self, restart: bool) -> None:
        if restart:
            self.div_cnt, self.div_acc, self.en = 1, 0, 0
        elif self.div_cnt == 1:
            integer = (self.clkdiv >> 16) & 0xFFFF
            frac = (self.clkdiv >> 8) & 0xFF
            total = self.div_acc + frac
            carry, self.div_acc = total >> 8, total & 0xFF
            self.en = 1
            self.div_cnt = (integer or 65536) + carry
        else:
            self.en = 0
            self.div_cnt -= 1

    def sm_restart(self) -> None:
        self.isr = 0
        self.osr_cnt = 32
        self.isr_cnt = 0
        self.delay = 0
        self.stalled = 0
        self.latch_pend = 0
        self.latch_host = 0
        self.force = 0
        self.irqw = 0
        self.jp_prev = None
        self.jp_rise = self.jp_fall = 0


class Chip:
    """The whole chip: `P.n_engines` engines, register file, pin resolution, IRQ lines."""

    PARAMS: Params = MVP_PARAMS
    REGMAP: RegMap = _MVP_MAP       # `fw.regmap.active()` reads this off the installed class

    def __init__(self, params: Optional[Params] = None) -> None:
        self.P = params if params is not None else type(self).PARAMS
        self.REGMAP = self.P.regmap
        self.inte_mask = ((1 << self.P.n_engines) - 1) * 0x11
        self.reset()

    def reset(self) -> None:
        P = self.P
        self.engines = [Engine(e, P) for e in range(P.n_engines)]
        self.sm_enable = 0
        self.route = 0          # engine_route: bit e = link engine e -> engine e+1 enabled
        self.fdebug = 0
        self.inte = [INTE0_RESET & self.inte_mask, INTE1_RESET & self.inte_mask]
        self.intf = [0, 0]
        self.level = 0          # registered resolved level state, 32-pin space (writable bits)
        self.dir = 0            # registered resolved direction state
        self.sync = [0] * P.t_in  # input pipeline; sync[-1] is IN_VEC of the current cycle
        self.irq0 = 0
        self.irq1 = 0
        self.irq_flags = 0      # isa_irq: the eight IRQ flags (no host register yet: ISA_EXT.md)
        self.cycle = 0          # number of the next cycle to be stepped
        self.last_in_vec = 0    # IN_VEC used in the last stepped cycle (debug aid)
        self.last_req = [{"out": None, "side": None} for _ in range(P.n_engines)]

    # ---- outputs ----
    @property
    def pin_out(self) -> int:
        return self.level & LEVEL_WRITABLE & PIN_MASK

    @property
    def pin_oe(self) -> int:
        return (self.dir & DIR_WRITABLE) | OUT_ONLY_MASK

    # ---- combinational views of the current state ----
    def fstat(self) -> int:
        v = 0
        for e, eng in enumerate(self.engines):
            v |= (len(eng.tx) == 0) << (24 + e)
            v |= (len(eng.tx) == self.P.fifo_depth) << (16 + e)
            v |= (len(eng.rx) == 0) << (8 + e)
            v |= (len(eng.rx) == self.P.fifo_depth) << e
        return v

    def flevel(self) -> int:
        v = 0
        for e, eng in enumerate(self.engines):
            v |= min(len(eng.tx), 15) << (8 * e) | min(len(eng.rx), 15) << (8 * e + 4)   # saturates at 15
        return v

    def intr(self) -> int:
        v = 0
        for e, eng in enumerate(self.engines):
            v |= (len(eng.rx) != 0) << e
            v |= (len(eng.tx) != self.P.fifo_depth) << (4 + e)
        return v

    def ints(self, k: int) -> int:
        return (self.intr() & self.inte[k]) | self.intf[k]

    def _sm_reg_read(self, eng: Engine, i: int) -> int:
        if i == SM_CLKDIV:
            return eng.clkdiv
        if i == SM_EXECCTRL:
            return eng.execctrl | (eng.exec_stalled << 31)
        if i == SM_SHIFTCTRL:
            return eng.shiftctrl
        if i == SM_ADDR:
            return eng.pc
        if i == SM_INSTR:
            return eng.ir
        return eng.pinctrl

    def cfginfo(self) -> int:
        return self.P.fifo_depth | self.P.n_engines << 8 | self.P.imem_words << 16

    def _engine_reg(self, addr: int):
        """(engine, index) of an SMe_* register address, or None."""
        n = self.P.n_engines
        if A_SM_BASE <= addr < A_SM_BASE + 6 * n:
            return divmod(addr - A_SM_BASE, 6)
        return None

    def _imem_slot(self, addr: int):
        """(engine, word) of an IMEM address that exists in this configuration, or None."""
        rm = self.REGMAP
        for e in range(self.P.n_engines):
            k = addr - rm.imem_base(e)
            if 0 <= k < self.P.imem_words:
                return e, k
        return None

    def peek(self, addr: int) -> int:
        """Value a host read of `addr` returns in the current cycle (no side effects)."""
        n = self.P.n_engines
        if addr == A_CTRL:
            return self.sm_enable | self.route << CTRL_ROUTE_SHIFT
        if addr == A_FSTAT:
            return self.fstat()
        if addr == A_FDEBUG:
            return self.fdebug
        if addr == A_FLEVEL:
            return self.flevel()
        if 0x08 <= addr < 0x08 + n:
            e = addr - 0x08
            if (self.route >> e) & 1:
                return 0                    # engine_route: this RX FIFO belongs to the link
            rx = self.engines[e].rx
            return rx[0] if rx else 0
        if addr == A_DBG_PADOUT:
            return self.pin_out
        if addr == A_DBG_PADOE:
            return self.pin_oe
        if addr == A_DBG_CFGINFO:
            return self.cfginfo()
        er = self._engine_reg(addr)
        if er is not None:
            return self._sm_reg_read(self.engines[er[0]], er[1])
        if addr == A_INTR:
            return self.intr()
        if addr in (A_IRQ0_INTE, A_IRQ1_INTE):
            return self.inte[0 if addr == A_IRQ0_INTE else 1]
        if addr in (A_IRQ0_INTF, A_IRQ1_INTF):
            return self.intf[0 if addr == A_IRQ0_INTF else 1]
        if addr in (A_IRQ0_INTS, A_IRQ1_INTS):
            return self.ints(0 if addr == A_IRQ0_INTS else 1)
        return 0  # TXF, IMEM (WO), reserved

    # ---- one core cycle ----
    def step(self, pin_in: int, bus: tuple | None = None) -> int | None:
        engs = self.engines
        in_vec = self.sync[-1]
        self.last_in_vec = in_vec
        pad = (pin_in & (UIO_MASK | IN_ONLY_MASK)) | (self.level & OUT_ONLY_MASK)
        tx_n0 = [len(e.tx) for e in engs]
        rx_n0 = [len(e.rx) for e in engs]
        enable0 = self.sm_enable
        route0 = self.route
        n_eng = self.P.n_engines

        # IRQ line flops: next value from this cycle's state.
        irq0_next = int(self.ints(0) != 0)
        irq1_next = int(self.ints(1) != 0)

        # Host read value (start-of-cycle).
        rdata = None
        kind = None
        if bus is not None:
            kind = bus[0]
            addr = bus[1]
            if not 0 <= addr < self.REGMAP.addr_space:
                raise ValueError(f"bus address {addr:#x} out of range")
            if kind == "r":
                rdata = self.peek(addr)
            elif kind != "w":
                raise ValueError(f"bad bus op {bus!r}")

        # Engines.
        fdebug_set = 0
        reqs = []
        irq_set = irq_clr = 0
        for e, eng in enumerate(engs):
            if eng.force or ((enable0 >> e) & 1 and eng.en):
                fd = {}
                eng.irq_in = self.irq_flags
                reqs.append(eng.sm_step(in_vec, fd))
                irq_set |= eng._irq_set
                irq_clr |= eng._irq_clr
                if fd.get("txstall"):
                    fdebug_set |= 1 << (24 + e)
                if fd.get("rxstall"):
                    fdebug_set |= 1 << e
            else:
                reqs.append({"out": None, "side": None})

        if self.P.isa_irq:      # clears first, then sets: a same-cycle set wins
            self.irq_flags = (self.irq_flags & ~irq_clr | irq_set) & 0xFF

        if self.P.isa_cond:
            for eng in engs:
                eng._edge_track(in_vec)

        # IR <= IMEM[PC_next], IMEM as it stands in this cycle (before a host write).
        for eng in engs:
            eng.ir = eng.imem[eng.pc]

        # Dividers (restart is applied by the CTRL write below).
        for eng in engs:
            eng.divider_next(False)

        # Pin resolution: within an engine side-set beats OUT/SET; engine 1 beats engine 0.
        level, dirs = self.level, self.dir
        for r in reqs:
            for ch in ("out", "side"):
                if r[ch] is None:
                    continue
                is_dir, m, v = r[ch]
                if is_dir:
                    dirs = (dirs & ~m) | (v & m)
                else:
                    level = (level & ~m) | (v & m)
        self.level = level & LEVEL_WRITABLE
        self.dir = dirs & DIR_WRITABLE
        # what the engines asked for in this cycle, for `deep_state`; nothing else reads it
        self.last_req = tuple(reqs)

        # Host side effects (host wins, except engine FDEBUG sets).
        fdebug = self.fdebug
        host_push = [0] * n_eng
        if kind == "r":
            addr = bus[1]
            if 0x08 <= addr < 0x08 + n_eng:
                e = addr - 0x08
                if (route0 >> e) & 1:
                    pass                            # engine_route: link-owned, no pop, no RXUNDER
                elif rx_n0[e] > 0:
                    engs[e].rx.pop(0)
                else:
                    fdebug |= 1 << (8 + e)          # RXUNDER
        elif kind == "w":
            addr, data = bus[1], bus[2] & M32
            if addr == A_CTRL:
                self.sm_enable = data & ((1 << n_eng) - 1)
                if self.P.engine_route:
                    self.route = (data >> CTRL_ROUTE_SHIFT) & ((1 << (n_eng - 1)) - 1)
                for e, eng in enumerate(engs):
                    if (data >> (4 + e)) & 1:
                        eng.sm_restart()
                    if (data >> (8 + e)) & 1:
                        eng.divider_next(True)
            elif addr == A_FDEBUG:
                fdebug &= ~data
            elif 0x04 <= addr < 0x04 + n_eng:
                e = addr - 0x04
                if tx_n0[e] < self.P.fifo_depth:
                    engs[e].tx.append(data)
                    host_push[e] = 1
                else:
                    fdebug |= 1 << (16 + e)         # TXOVER
            elif self._imem_slot(addr) is not None:
                e, k = self._imem_slot(addr)
                engs[e].imem[k] = data & 0xFFFF
            elif self._engine_reg(addr) is not None:
                e, i = self._engine_reg(addr)
                eng = engs[e]
                if i == SM_CLKDIV:
                    eng.clkdiv = data & CLKDIV_MASK
                elif i == SM_EXECCTRL:
                    eng.execctrl = data & eng.execctrl_mask
                elif i == SM_SHIFTCTRL:
                    eng.shiftctrl = data & SHIFTCTRL_MASK
                elif i == SM_INSTR:
                    eng.latch = data & 0xFFFF
                    eng.latch_host = 1
                    eng.force = 1
                elif i == SM_PINCTRL:
                    eng.pinctrl = data & PINCTRL_MASK
                # SM_ADDR is read-only
            elif addr in (A_IRQ0_INTE, A_IRQ1_INTE):
                self.inte[0 if addr == A_IRQ0_INTE else 1] = data & self.inte_mask
            elif addr in (A_IRQ0_INTF, A_IRQ1_INTF):
                self.intf[0 if addr == A_IRQ0_INTF else 1] = data & self.inte_mask
            # everything else: RO, RF or reserved -> ignored
        self.fdebug = (fdebug | fdebug_set) & (((1 << n_eng) - 1) * 0x0101_0101)   # H: was 0x0303_0303 (2 engines only)

        # Engine route (tools/bench/sim/REGMAP.md): link e moves the head of engine e's RX FIFO
        # to the tail of engine e+1's TX FIFO. Start-of-cycle levels decide; a host push to the
        # same TX FIFO this cycle takes the slot first.
        for e in range(n_eng - 1):
            if (route0 >> e) & 1 and rx_n0[e] > 0 and tx_n0[e + 1] + host_push[e + 1] < self.P.fifo_depth:
                engs[e + 1].tx.append(engs[e].rx.pop(0))

        # Input synchronizers.
        self.sync = [pad] + self.sync[:-1]
        self.irq0, self.irq1 = irq0_next, irq1_next
        self.cycle += 1
        return rdata

    def state(self, e: int) -> dict:
        eng = self.engines[e]
        return {
            "pc": eng.pc, "ir": eng.ir, "x": eng.x, "y": eng.y, "osr": eng.osr, "isr": eng.isr,
            "osr_cnt": eng.osr_cnt, "isr_cnt": eng.isr_cnt, "delay_cnt": eng.delay,
            "stalled": eng.stalled, "latch": eng.latch, "latch_pend": eng.latch_pend,
            "latch_host": eng.latch_host, "forced": eng.force,
            "tx_level": len(eng.tx), "rx_level": len(eng.rx),
        }

    def deep_state(self) -> dict:
        """The state neither `state()` nor `peek()` can show, for `verif/fuzz/lockstep.py --deep`.

        `state()` is the architectural state and `peek()` is the register file; between them they
        leave five things unobserved, and the RTL has all five as ordinary signals
        (`docs/mvp-interfaces.md` `### Lockstep visibility` names the state ones):

        | key | what |
        |---|---|
        | `"in_vec"` | the `IN_VEC` the engines use in the current cycle, which is `sync[-1]` (`last_in_vec` is the same number one `step()` earlier, so it is the wrong one to read between steps) |
        | `"req"` | this cycle's pin write requests, per engine: `{"out": (is_dir, mask, val) or None, "side": ...}`, mask and val 18-bit, `None` where the channel did not write |
        | `"res_level"`, `"res_dir"` | what this cycle's pin write requests resolve to, which the output register holds from the next cycle on (T_OUT = 1). The RTL holds the same number combinationally in `lvl_n`/`dir_n` *during* this cycle, so a harness compares the two in the same cycle; 18-bit, masked to the pins that have a register |
        | `str(e)` | engine `e`: `"tx"`/`"rx"` the FIFO **contents**, oldest word first (the levels are in `state()`), `"imem"` the 32 IMEM words, `"div"` the 16.8 divider `(div_cnt, div_acc, en)` |

        Additive by construction: nothing else in this file reads any of it, `state()` keeps
        exactly the keys `docs/mvp-interfaces.md` `## ISS API` fixes, and no `step()` decision
        depends on it, so every verdict the ISS test suite produces is unchanged by its presence.
        """
        out: dict[str, Any] = {
            "in_vec": self.sync[-1],
            "res_level": self.level & PIN_MASK,
            "res_dir": self.dir & PIN_MASK,
            "req": [{"out": _req18(r["out"]), "side": _req18(r["side"])} for r in self.last_req],
        }
        for e, eng in enumerate(self.engines):
            out[str(e)] = {
                "tx": list(eng.tx), "rx": list(eng.rx), "imem": list(eng.imem),
                "div": (eng.div_cnt, eng.div_acc, eng.en),
            }
        return out


# ---- pin-level wrapper for verif.vip.core.Bus ----
class ChipDevice(Device):
    """A Chip on a verif.vip.core.Bus with nets "p0".."p17".

    Each tick: read the snapshot into pin_in, perform at most one queued host access, step the
    chip one core cycle, then drive pin k with pin_out[k] where pin_oe[k] = 1, else release."""

    def __init__(self, chip: Chip | None = None, name: str = "chip", prefix: str = "p") -> None:
        self.chip = chip if chip is not None else Chip()
        self.name = name
        self.prefix = prefix
        self.queue: deque = deque()

    def write(self, addr: int, data: int) -> None:
        self.queue.append(("w", addr, data, None))

    def read(self, addr: int) -> Future:
        fut: Future = Future()
        self.queue.append(("r", addr, None, fut))
        return fut

    @property
    def idle(self) -> bool:
        return not self.queue

    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        pin_in = 0
        for k in range(NUM_PINS):
            if pins.get(f"{self.prefix}{k}", 0):
                pin_in |= 1 << k
        bus = fut = None
        if self.queue:
            kind, addr, data, fut = self.queue.popleft()
            bus = ("w", addr, data) if kind == "w" else ("r", addr)
        r = self.chip.step(pin_in, bus)
        if fut is not None:
            fut.set_result(r)
        out, oe = self.chip.pin_out, self.chip.pin_oe
        drives: dict[str, Optional[int]] = {}
        for k in range(NUM_PINS):
            name = f"{self.prefix}{k}"
            if name in pins:
                drives[name] = (out >> k) & 1 if (oe >> k) & 1 else None
        return drives


# ---- target plumbing (fw/targets.py `iss_p`) ----
_CLASSES: dict[Params, type] = {}


def chip_class(params: Params) -> type:
    """A `Chip` subclass with `params` and the matching `REGMAP` bound (one class per Params)."""
    if params == MVP_PARAMS:
        return Chip
    if params not in _CLASSES:
        name = (f"ChipP_n{params.n_engines}_i{params.imem_words}_f{params.fifo_depth}"
                f"_r{int(params.engine_route)}_t{params.t_in}"
                f"_x{int(params.isa_xor)}c{int(params.isa_cond)}q{int(params.isa_irq)}")
        _CLASSES[params] = type(name, (Chip,), {"PARAMS": params, "REGMAP": params.regmap})
    return _CLASSES[params]


def chip_class_from_env() -> type:
    """The class for the bench config in env BENCH_CONFIG (default: the MVP)."""
    return chip_class(Params.from_env())

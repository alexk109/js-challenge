"""A small direct rig for the ISA-extension firmware: one parametric-ISS chip, one engine, a pin stimulus.

Everything goes through the host bus (IMEM, SMx_* registers, FIFOs, CTRL), as a driver would; there is no peeking
except `chip.irq_flags`, which has no host register yet (ISA_EXT.md).
"""
from __future__ import annotations

from typing import Callable, Optional, Sequence

from asm.pioasm import Program, assemble
from tools.bench.sim.pio_iss_p import (A_CTRL, A_SM_BASE, SM_CLKDIV, SM_EXECCTRL, SM_INSTR, SM_PINCTRL,
                                       SM_SHIFTCTRL, Chip, Params, chip_class)

A_IMEM0, A_TXF0, A_RXF0 = 0x12, 0x04, 0x08


def params_for(isa: Sequence[str]) -> Params:
    return Params(isa_xor="xor" in isa, isa_cond="cond" in isa, isa_irq="irq" in isa)


class Rig:
    def __init__(self, program: Program, isa: Sequence[str] = (), *, clkdiv: int = 1, jmp_pin: int = 0,
                 in_base: int = 0, shiftctrl: Optional[int] = None, pinctrl: int = 0, start: int = 0,
                 params: Optional[Params] = None):
        self.params = params or params_for(isa)
        self.chip: Chip = chip_class(self.params)()
        self.program = program
        self.stim: Callable[[int], int] = lambda t: 0
        self.t = 0
        for k, w in enumerate(program.instrs):
            self.w(A_IMEM0 + k, w)
        self.w(A_SM_BASE + SM_CLKDIV, clkdiv << 16)
        top, bot = program.wrap, program.wrap_target
        self.w(A_SM_BASE + SM_EXECCTRL, (jmp_pin << 24) | (top << 12) | (bot << 7))
        if shiftctrl is not None:
            self.w(A_SM_BASE + SM_SHIFTCTRL, shiftctrl)
        self.w(A_SM_BASE + SM_PINCTRL, pinctrl | (in_base << 15))
        self.start = start

    # ---- host bus
    def step(self, bus=None):
        r = self.chip.step(self.stim(self.t), bus)
        self.t += 1
        return r

    def w(self, addr: int, data: int) -> None:
        self.step(("w", addr, data))

    def r(self, addr: int) -> int:
        return self.step(("r", addr))

    def run(self, n: int) -> None:
        for _ in range(n):
            self.step()

    def push(self, word: int) -> None:
        self.w(A_TXF0, word)

    def rx_level(self) -> int:
        return (self.r(0x03) >> 4) & 0xF

    def pop(self) -> int:
        return self.r(A_RXF0)

    def drain(self) -> list[int]:
        out = []
        while self.rx_level():
            out.append(self.pop())
        return out

    def go(self) -> None:
        """Force `jmp start` and enable engine 0."""
        self.w(A_SM_BASE + SM_INSTR, self.start & 0x1F)
        self.w(A_CTRL, 1)

    @property
    def eng(self):
        return self.chip.engines[0]


def asm(text: str, isa: Sequence[str] = ()) -> Program:
    return assemble(text, isa)[0]

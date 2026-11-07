"""Register addresses for the firmware drivers, taken from the active chip.

Every `fw/*/driver.py` used to carry the MVP word addresses as literals. They now ask this module,
so one driver runs on the MVP map and on the generalized map of a parametric chip
(`tools/bench/sim/REGMAP.md`). The map comes from the chip class installed behind
`iss.pio_iss.Chip` (`fw/targets.py`): a class that has a `REGMAP` attribute (a `RegMap`) is
asked; any other class (golden ISS, ISS-B, RTL, TT chip) is the MVP and gets `MVP`.

Drivers call these at import time, and `fw/conftest.py` installs the target before any driver is
imported, so the map is fixed for the process. A tool that builds a non-MVP chip must call
`fw.targets.install(...)` before importing a driver.

At the MVP every value here is bit-for-bit the literal it replaced.
"""
from __future__ import annotations

from dataclasses import dataclass

# ---- fixed across all configs (MVP positions; RP2040 offsets / 4) ----
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = 0x0F, 0x10, 0x11
A_INTR = 0x4A
A_IRQ0_INTE, A_IRQ0_INTF, A_IRQ0_INTS = 0x4B, 0x4C, 0x4D
A_IRQ1_INTE, A_IRQ1_INTF, A_IRQ1_INTS = 0x4E, 0x4F, 0x50
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)
SM_NAMES = ("CLKDIV", "EXECCTRL", "SHIFTCTRL", "ADDR", "INSTR", "PINCTRL")

#: CTRL: SM_ENABLE bit e, SM_RESTART bit 4+e, CLKDIV_RESTART bit 8+e, ROUTE_EN bit 16+e (link e -> e+1)
CTRL_ROUTE_SHIFT = 16

#: Wide-map IMEM window: engine e word k at WIDE_IMEM_BASE + WIDE_IMEM_STRIDE * e + k.
WIDE_IMEM_BASE, WIDE_IMEM_STRIDE = 0x80, 0x80
NARROW_ADDRS, WIDE_ADDRS = 128, 1024


@dataclass(frozen=True)
class RegMap:
    n_engines: int = 2
    imem_words: int = 32
    fifo_depth: int = 4
    engine_route: bool = False

    @property
    def wide(self) -> bool:
        """True when the MVP's 7-bit address space cannot hold this configuration's IMEM."""
        return self.n_engines > 2 or self.imem_words > 32

    @property
    def addr_space(self) -> int:
        return WIDE_ADDRS if self.wide else NARROW_ADDRS

    @property
    def addr_bits(self) -> int:
        return self.addr_space.bit_length() - 1

    def txf(self, e: int) -> int:
        return 0x04 + e

    def rxf(self, e: int) -> int:
        return 0x08 + e

    def sm_base(self, e: int = 0) -> int:
        return 0x32 + 6 * e

    def sm(self, e: int, i: int) -> int:
        return 0x32 + 6 * e + i

    def imem_base(self, e: int) -> int:
        if self.wide:
            return WIDE_IMEM_BASE + WIDE_IMEM_STRIDE * e
        return 0x12 + 0x40 * e

    def imem(self, e: int, k: int) -> int:
        return self.imem_base(e) + k

    @property
    def cfginfo(self) -> int:
        """DBG_CFGINFO: FIFO_DEPTH 5:0 | SM_COUNT 11:8 | IMEM_SIZE 23:16 (MVP 0x0020_0204)."""
        return self.fifo_depth | self.n_engines << 8 | self.imem_words << 16

    @property
    def fifo_addrs(self) -> frozenset:
        """Addresses where a burst does not advance (TXF/RXF windows)."""
        return frozenset(range(0x04, 0x0C))


MVP = RegMap()


def active() -> RegMap:
    """The map of the chip class installed behind `iss.pio_iss.Chip` (MVP if it declares none)."""
    from iss import pio_iss

    return getattr(pio_iss.Chip, "REGMAP", MVP)


def txf_addrs() -> tuple:
    m = active()
    return tuple(m.txf(e) for e in range(m.n_engines))


def rxf_addrs() -> tuple:
    m = active()
    return tuple(m.rxf(e) for e in range(m.n_engines))


def imem_bases() -> tuple:
    """Per engine: the word address of IMEM word 0 (`bases[e] + k`)."""
    m = active()
    return tuple(m.imem_base(e) for e in range(m.n_engines))


def sm_base() -> int:
    return active().sm_base(0)


def imem_addr(e: int, k: int) -> int:
    return active().imem(e, k)


def sm_addr(e: int, i: int) -> int:
    return active().sm(e, i)


def n_engines() -> int:
    return active().n_engines


def imem_words() -> int:
    return active().imem_words


def fifo_depth() -> int:
    return active().fifo_depth


def fifo_full_level() -> int:
    """The largest value an `FLEVEL` nibble can show for a full FIFO: `min(fifo_depth, 15)`.

    A nibble saturates at 15 (REGMAP.md section 1), so with `fifo_depth = 16` a full FIFO reads 15
    and a `level >= fifo_depth()` wait would never end. Compare *levels read from FLEVEL* against
    this; compare *word counts* (how many words fit) against `fifo_depth()`. At depth <= 15 they
    are equal. Read at use time: the active map is only fixed once the target is installed.
    """
    return min(active().fifo_depth, 15)

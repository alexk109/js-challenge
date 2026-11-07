"""Test helpers: hand encoders from docs/mvp-isa.md `## Encoding` and a small harness.

The encoders are written from the spec's bit table, not from the assembler (which is written in
parallel and must not be imported). test_encoding.py checks them against the spec's worked
encodings.
"""
from __future__ import annotations

from iss.pio_iss import Chip

# ---- field values (docs/mvp-isa.md `### Field values`) ----
COND = dict(always=0, not_x=1, x_dec=2, not_y=3, y_dec=4, x_ne_y=5, pin=6, not_osre=7)
WAIT_SRC = dict(gpio=0, pin=1, irq=2, reserved=3)
IN_SRC = dict(pins=0, x=1, y=2, null=3, rsv4=4, rsv5=5, isr=6, osr=7)
OUT_DST = dict(pins=0, x=1, y=2, null=3, pindirs=4, pc=5, isr=6, exec=7)
MOV_DST = dict(pins=0, x=1, y=2, rsv3=3, exec=4, pc=5, isr=6, osr=7)
MOV_OP = dict(none=0, invert=1, reverse=2, rsv3=3)
MOV_SRC = dict(pins=0, x=1, y=2, null=3, rsv4=4, status=5, isr=6, osr=7)
SET_DST = dict(pins=0, x=1, y=2, rsv3=3, pindirs=4, rsv5=5, rsv6=6, rsv7=7)


def _w(opc: int, ds: int, low8: int) -> int:
    assert 0 <= ds < 32 and 0 <= low8 < 256
    return (opc << 13) | (ds << 8) | low8


def _cnt(n: int) -> int:
    assert 1 <= n <= 32
    return n & 0x1F  # 32 encodes as 0


def jmp(addr: int, cond: str = "always", ds: int = 0) -> int:
    return _w(0, ds, (COND[cond] << 5) | addr)


def wait(pol: int, src: str, index: int, ds: int = 0) -> int:
    return _w(1, ds, (pol << 7) | (WAIT_SRC[src] << 5) | index)


def in_(src: str, n: int, ds: int = 0) -> int:
    return _w(2, ds, (IN_SRC[src] << 5) | _cnt(n))


def out(dst: str, n: int, ds: int = 0) -> int:
    return _w(3, ds, (OUT_DST[dst] << 5) | _cnt(n))


def push(iffull: int = 0, block: int = 1, ds: int = 0) -> int:
    return _w(4, ds, (0 << 7) | (iffull << 6) | (block << 5))


def pull(ifempty: int = 0, block: int = 1, ds: int = 0) -> int:
    return _w(4, ds, (1 << 7) | (ifempty << 6) | (block << 5))


def mov(dst: str, src: str, op: str = "none", ds: int = 0) -> int:
    return _w(5, ds, (MOV_DST[dst] << 5) | (MOV_OP[op] << 3) | MOV_SRC[src])


def irq(low8: int = 0, ds: int = 0) -> int:
    return _w(6, ds, low8)


def set_(dst: str, data: int, ds: int = 0) -> int:
    return _w(7, ds, (SET_DST[dst] << 5) | data)


NOP = mov("y", "y")  # 0xA042

# ---- register addresses (docs/mvp-regmap.md, written out again here on purpose) ----
CTRL, FSTAT, FDEBUG, FLEVEL = 0x00, 0x01, 0x02, 0x03
TXF = (0x04, 0x05)
RXF = (0x08, 0x09)
DBG_PADOUT, DBG_PADOE, DBG_CFGINFO = 0x0F, 0x10, 0x11
INTR = 0x4A
IRQ0_INTE, IRQ0_INTF, IRQ0_INTS = 0x4B, 0x4C, 0x4D
IRQ1_INTE, IRQ1_INTF, IRQ1_INTS = 0x4E, 0x4F, 0x50


def imem_addr(e: int, k: int) -> int:
    return 0x12 + 0x40 * e + k


def sm(e: int, name: str) -> int:
    i = ["CLKDIV", "EXECCTRL", "SHIFTCTRL", "ADDR", "INSTR", "PINCTRL"].index(name)
    return 0x32 + 6 * e + i


def execctrl(wrap_bottom=0, wrap_top=31, side_en=0, side_pindir=0, jmp_pin=0,
             status_sel=0, status_n=0) -> int:
    return (side_en << 30) | (side_pindir << 29) | (jmp_pin << 24) | (wrap_top << 12) \
        | (wrap_bottom << 7) | (status_sel << 4) | status_n


def shiftctrl(pull_thresh=0, push_thresh=0, out_right=1, in_right=1, autopull=0,
              autopush=0) -> int:
    return (pull_thresh << 25) | (push_thresh << 20) | (out_right << 19) | (in_right << 18) \
        | (autopull << 17) | (autopush << 16)


def pinctrl(sideset_count=0, set_count=5, out_count=0, in_base=0, sideset_base=0,
            set_base=0, out_base=0) -> int:
    return (sideset_count << 29) | (set_count << 26) | (out_count << 20) | (in_base << 15) \
        | (sideset_base << 10) | (set_base << 5) | out_base


def clkdiv(integer: int, frac: int = 0) -> int:
    return (integer << 16) | (frac << 8)


class H:
    """Harness around a Chip. `cycle` is the number of the next cycle to run.

    pin_in defaults to loopback (each pad follows the chip's own level register, i.e. an
    unloaded pad); set `ext` to an int to drive inputs explicitly instead."""

    def __init__(self) -> None:
        self.c = Chip()
        self.ext: int | None = None

    @property
    def cycle(self) -> int:
        return self.c.cycle

    def tick(self, bus=None, pin_in: int | None = None):
        if pin_in is None:
            pin_in = self.c.pin_out if self.ext is None else self.ext
        return self.c.step(pin_in, bus)

    def idle(self, n: int = 1) -> None:
        for _ in range(n):
            self.tick()

    def w(self, addr: int, data: int) -> None:
        self.tick(("w", addr, data))

    def r(self, addr: int) -> int:
        return self.tick(("r", addr))

    def load(self, e: int, words, base: int = 0) -> None:
        for k, word in enumerate(words):
            self.w(imem_addr(e, base + k), word)

    def force(self, e: int, instr: int) -> None:
        """Host-force one instruction: write in cycle h, executed in h+1 (both stepped)."""
        self.w(sm(e, "INSTR"), instr)
        self.tick()

    def s(self, e: int = 0) -> dict:
        return self.c.state(e)

    def set_reg(self, e: int, reg: str, value: int) -> None:
        """Load a 32-bit value into x, y, isr or osr through the TX FIFO and forced instrs."""
        self.w(TXF[e], value)
        self.force(e, pull(block=1))
        if reg != "osr":
            self.force(e, mov(reg, "osr"))

    def start(self, mask: int, restart: bool = True) -> int:
        """Write CTRL = enable|restart|clkdiv_restart for engines in mask; return h."""
        h = self.cycle
        v = mask | (mask << 8) | ((mask << 4) if restart else 0)
        self.w(CTRL, v)
        return h

    def run_program(self, e: int, words, *, wrap=None, shift=None, pins=None, div=None,
                    others: int = 0, restart: bool = True) -> int:
        """Load and configure engine e, then start it (and engines in `others`). Returns h;
        the first SM cycle is h+2 at divisor 1."""
        self.load(e, words)
        if div is not None:
            self.w(sm(e, "CLKDIV"), div)
        if wrap is not None:
            self.w(sm(e, "EXECCTRL"), wrap)
        if shift is not None:
            self.w(sm(e, "SHIFTCTRL"), shift)
        if pins is not None:
            self.w(sm(e, "PINCTRL"), pins)
        self.idle(2)
        return self.start((1 << e) | others, restart)

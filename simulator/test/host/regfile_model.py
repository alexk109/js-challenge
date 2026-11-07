"""Register-file model on the host port's register bus, for ``test/host``.

``host_spi`` is a pure transport: it turns SPI frames into register-bus accesses and
carries ``bus_rdata`` back out on MISO. It knows nothing about the registers, so the
bench needs something on the far side of the bus. That something is this file: a Python
model of the chip's register file, driven one core cycle at a time by
:class:`RegFileBus`.

The model is deliberately small, because everything the bench checks about the
registers is in the spec text, not in a model:

* ``regs`` is a plain ``dict`` of word address to value. An address that is not in it
  reads 0, which is what ``docs/mvp-regmap.md`` says unmapped and reserved addresses
  do ("### Unmapped or reserved addresses").
* One address has a FIFO-pop side effect: ``fifo_addr`` (RXF0, ``0x08`` by default).
  A read returns the oldest word and pops it at the *end* of the access cycle. This is
  the "read has a FIFO side effect" rule of ``### RXF0, RXF1``. Point ``fifo_addr`` at
  ``0x09`` (RXF1) to model the other engine's FIFO.
* ``bus_rdata`` is combinational: ``read()`` returns the state of the register *in*
  the access cycle, and the side effect lands at the end of that cycle, per
  ``docs/mvp-interfaces.md`` `## Register bus` and ``docs/mvp-regmap.md``
  `### Cycle-exact mapping to bus accesses`.

The model does **not** implement the access types (RO/WO/W1C/SC/WF/RF beyond the one
FIFO) or the address map: a write to any address is stored and reads back, which is
what a plain dict does. That is fine, because the unit under test never sees the map;
it only moves 32-bit words to and from whatever answers on the bus. Tests poke
whatever value they want into ``regs`` and check that exactly those bits come back over
SPI.

Importable without cocotb for the model half; :class:`RegFileBus` needs cocotb.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

__all__ = [
    "Access",
    "RegFile",
    "RegFileBus",
    "FIFO_ADDR",
    "FIFO_DEPTH",
    "next_addr",
    "FIFO_FIRST",
    "FIFO_LAST",
]

#: Word addresses 0x04-0x0B are the FIFO data registers, where the burst address stays
#: put instead of incrementing (``docs/mvp-regmap.md` `### Bursts`).
FIFO_FIRST = 0x04
FIFO_LAST = 0x0B
#: RXF0: the default address whose reads pop.
FIFO_ADDR = 0x08
FIFO_DEPTH = 4


def next_addr(addr: int) -> int:
    """The address the next word of a burst uses, after an access to `addr`.

    "After each word access, the address increments by 1, wrapping mod 128. The
    exception is the FIFO data registers (word addresses 0x04-0x0B), where the address
    stays put."
    """
    addr &= 0x7F
    if FIFO_FIRST <= addr <= FIFO_LAST:
        return addr
    return (addr + 1) & 0x7F


@dataclass
class Access:
    """One bus access the DUT asked for, as the model saw it."""

    cycle: int          #: the bus access cycle h
    kind: str           #: "w" or "r"
    addr: int           #: word address
    wdata: int = 0
    rdata: int = 0      #: for "r": what the model returned, i.e. the state in cycle h

    def __str__(self) -> str:
        if self.kind == "w":
            return f"h={self.cycle} W [{self.addr:#04x}] = {self.wdata:#010x}"
        return f"h={self.cycle} R [{self.addr:#04x}] -> {self.rdata:#010x}"


def _reset_values() -> dict:
    """The registers ``docs/mvp-regmap.md` `## Registers` gives a nonzero reset value.

    Only the interesting ones; every other address starts absent from the dict, which
    reads 0 anyway.
    """
    regs = {
        0x01: 0x0300_0300,   # FSTAT: TXEMPTY and RXEMPTY for both engines
        0x11: 0x0020_0204,   # DBG_CFGINFO: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
        0x4B: 0x0000_0003,   # IRQ0_INTE (MVP reset)
        0x4E: 0x0000_0010,   # IRQ1_INTE (MVP reset)
    }
    for engine in (0, 1):
        base = 0x32 + 6 * engine
        regs[base + 0] = 0x0001_0000   # SMe_CLKDIV
        regs[base + 1] = 0x0001_F000   # SMe_EXECCTRL
        regs[base + 2] = 0x000C_0000   # SMe_SHIFTCTRL
        regs[base + 5] = 0x1400_0000   # SMe_PINCTRL
    return regs


class RegFile:
    """The chip side of the register bus: a dict, one FIFO-pop address, and a log."""

    def __init__(self, fifo_addr: int = FIFO_ADDR):
        self.fifo_addr = fifo_addr & 0x7F
        #: word address -> value. The register file itself.
        self.regs: dict[int, int] = {}
        #: RXF words, oldest first.
        self.fifo: list[int] = []
        #: Every access, in order, with the cycle it happened in.
        self.accesses: list[Access] = []
        #: The access cycle currently being modelled (0 before the first cycle).
        self.cycle = 0
        #: Called as ``on_cycle_end(cycle)`` at the end of every cycle, after that
        #: cycle's write and FIFO pop have landed. A test uses it to change a register
        #: in the middle of a burst, the way an engine would.
        self.on_cycle_end: Optional[Callable[[int], None]] = None
        #: Accesses of the cycle being closed, so a hook can see them.
        self.closing: Optional[Access] = None
        #: Bus-protocol complaints: (cycle, what).
        self.errors: list[str] = []
        self.n_pops = 0
        self._pending_write: Optional[tuple[int, int]] = None
        self._pending_pop: Optional[int] = None
        self.reset()

    # -- state ---------------------------------------------------------------

    def reset(self) -> None:
        self.regs = _reset_values()
        self.fifo = []
        self.accesses = []
        self.cycle = 0
        self.on_cycle_end = None
        self.closing = None
        self.errors = []
        self.n_pops = 0
        self._pending_write = None
        self._pending_pop = None

    def poke(self, addr: int, data: int) -> None:
        """Set a register without going through the bus (test setup)."""
        self.regs[addr & 0x7F] = data & 0xFFFF_FFFF

    def push(self, *words: int) -> None:
        """Put words in the RX FIFO, oldest first (the engine's job, for a test)."""
        for word in words:
            if len(self.fifo) >= FIFO_DEPTH:
                raise ValueError(f"RX FIFO holds {FIFO_DEPTH} words")
            self.fifo.append(word & 0xFFFF_FFFF)

    @property
    def free(self) -> list[str]:
        """A printable summary, for an assertion message."""
        return [str(a) for a in self.accesses]

    # -- the bus -------------------------------------------------------------

    def read(self, addr: int) -> int:
        """The value ``bus_rdata`` must carry in *this* cycle for a read of `addr`.

        Combinational in the access cycle, and no side effect here: the pop happens in
        :meth:`end_cycle`, at the end of the cycle, so a read in cycle ``h`` returns the
        oldest word and the *next* read sees the one behind it.
        """
        addr &= 0x7F
        if addr == self.fifo_addr:
            return self.fifo[0] if self.fifo else 0
        return self.regs.get(addr, 0)

    def write(self, addr: int, data: int) -> None:
        """Apply a write that happened this cycle; it lands at the end of the cycle."""
        self._pending_write = (addr & 0x7F, data & 0xFFFF_FFFF)

    def request_read(self, addr: int) -> int:
        """Queue the pop a read of `addr` causes; returns the value for this cycle."""
        addr &= 0x7F
        if addr == self.fifo_addr and self.fifo:
            self._pending_pop = addr
        return self.read(addr)

    def end_cycle(self, cycle: Optional[int] = None) -> None:
        """Close cycle `cycle`: its write and its pop land at its end.

        Called once per cycle by :class:`RegFileBus`, with the number of the cycle
        that just finished, so a hook sees the cycle whose effects have just landed.
        """
        if cycle is not None:
            self.cycle = cycle
        if self._pending_write is not None:
            addr, data = self._pending_write
            self.regs[addr] = data
            self._pending_write = None
        if self._pending_pop is not None:
            if self.fifo:
                self.fifo.pop(0)
                self.n_pops += 1
            self._pending_pop = None
        if self.on_cycle_end is not None:
            self.on_cycle_end(self.cycle)


# ----------------------------------------------------------------------------- cocotb


def _uint(handle) -> int:
    """A signal's value as an int; unknown (x/z) bits read as 0."""
    if handle is None:
        return 0
    value = getattr(handle, "value", None)
    if value is None:
        return 0
    try:
        return int(value)
    except Exception:
        bits = str(getattr(value, "binstr", value))
        return int(bits.translate(str.maketrans("xXzZuUwW", "00000000")), 2) or 0


class RegFileBus:
    """Drives ``bus_rdata`` from a :class:`RegFile`, one core cycle per clock.

    The rules it implements, from ``docs/mvp-interfaces.md` `## Register bus`:

    * ``bus_rdata`` is combinational: it carries the register's state in the access
      cycle, so it is written during that cycle and the DUT latches it at the edge that
      ends it. One access per cycle at most, ``bus_we`` and ``bus_re`` never together.
    * a write, and a read's side effect, take effect at the *end* of the access cycle.
      So each pass of the loop (a) closes the previous cycle, then (b) samples the
      access of this cycle, then (c) puts the read data on ``bus_rdata`` for this cycle.

    ``cycle_of()`` supplies the cycle number, which the bench ties to the bridge's bus
    tick so the same number indexes the wire-level golden model.
    """

    def __init__(self, dut, regfile: RegFile, cycle_of: Callable[[], int]):
        self.dut = dut
        self.rf = regfile
        self.cycle_of = cycle_of
        self._started = False

    @property
    def clk(self):
        return self.dut.clk

    async def run(self) -> None:
        """Coroutine: one pass per clock, until the task is killed."""
        from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer

        dut = self.dut
        while True:
            await RisingEdge(self.clk)
            await ReadOnly()
            # 1 ns is well inside the cycle (20 ns at 50 MHz) and past the point where
            # the Bridge has stepped this edge, so bridge.bus.t is this edge's number.
            await Timer(1, unit="ns")

            cycle = self.cycle_of()
            self.rf.end_cycle(cycle - 1)   # the previous cycle's effects land now
            self.rf.cycle = cycle

            we, re = _uint(dut.bus_we), _uint(dut.bus_re)
            addr = _uint(dut.bus_addr) & 0x7F
            if we and re:
                self.rf.errors.append(f"h={cycle}: bus_we and bus_re together")
            self.rf.closing = None
            rdata = 0
            if re:
                rdata = self.rf.request_read(addr)
                access = Access(cycle, "r", addr, rdata=rdata)
                self.rf.accesses.append(access)
                self.rf.closing = access
            elif we:
                data = _uint(dut.bus_wdata)
                self.rf.write(addr, data)
                access = Access(cycle, "w", addr, wdata=data)
                self.rf.accesses.append(access)
                self.rf.closing = access

            await FallingEdge(self.clk)
            dut.bus_rdata.value = rdata     # valid for the rest of cycle `cycle`
        # (unreachable)

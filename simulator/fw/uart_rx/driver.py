"""fw/uart_rx/driver.py -- the host side of the 8N1 UART receiver.

Everything here is written against `docs/mvp-regmap.md` (word addresses and field positions)
and goes through `iss.pio_iss.ChipDevice`'s write/read queue, one access per core cycle, on a
`verif.vip.core.Bus` at 20 ns per tick (50 MHz).

The host's job, in the order of `### Typical host sequences`:

1. `CTRL` with the engine disabled, then the program into its IMEM (`INSTR_MEM0..31`).
2. `CLKDIV`, `EXECCTRL` (wrap and `JMP_PIN`), `SHIFTCTRL`, `PINCTRL`.
3. A forced `set pindirs, 0` so the RX pin is an input, then a forced `jmp hunt`.
4. `CTRL` with `SM_ENABLE`, `SM_RESTART` and `CLKDIV_RESTART` together.
5. Read `RXF0` whenever the chip's `rx_valid` line (IRQ0, `uo_out[2]`) says the RX FIFO has
   something: a data word carries the byte at bits 31:24, and a word with bits 23:0 nonzero is
   the marker that flags the byte before it as a framing error.

The push side of the FIFO is the program's (`push block`): the engine pushes into the RX FIFO
and the host pops it.  A receiver has nothing to send, so the driver never writes `TXF0`; that
is the `uart_tx` firmware's half of the port.

The program spends a fixed number of SM cycles per bit (`S` in program.pio), so the host sets
`CLKDIV = ticks_per_bit / S`: the divider then makes one bit time exactly `S` SM cycles and the
program's fixed cycle counts land in the middle of every bit.  `S` and the program's loop are a
pair, so this module rewrites the `.define S` line of program.pio before assembling it.
"""
from __future__ import annotations

from fw import regmap

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from asm.pioasm import Program, assemble
from iss.pio_iss import Chip, ChipDevice
from verif.vip.core import Bus

from fw.uart_rx.framing import PARITY_ERROR_MARKER, DEFAULT as DEFAULT_FRAMING
from fw.uart_rx.framing import Framing
from fw.uart_rx.framing import layout as word_layout
from fw.uart_rx.framing import program_for

# ---------------------------------------------------------------------------------------------
# Register word addresses (docs/mvp-regmap.md `### Address map`)
# ---------------------------------------------------------------------------------------------
CTRL = 0x00
FSTAT = 0x01
FDEBUG = 0x02
FLEVEL = 0x03
TXF = regmap.txf_addrs()
RXF = regmap.rxf_addrs()
DBG_PADOUT = 0x0F
DBG_PADOE = 0x10
DBG_CFGINFO = 0x11
INTR = regmap.A_INTR

SM_NAMES = ("CLKDIV", "EXECCTRL", "SHIFTCTRL", "ADDR", "INSTR", "PINCTRL")


def imem_addr(engine: int, word: int) -> int:
    """`INSTR_MEM0..31` of `engine` (0: 0x12..0x31, 1: 0x52..0x71)."""
    return regmap.imem_addr(engine, word)


def sm_addr(engine: int, name: str) -> int:
    """`SMe_CLKDIV`, `SMe_EXECCTRL`, ... : word `0x32 + 6 * engine + i`."""
    return regmap.sm_addr(engine, SM_NAMES.index(name))


# Register fields, packed from the per-register tables of docs/mvp-regmap.md.
def clkdiv(value: float) -> int:
    """`SMe_CLKDIV` from a divisor in core cycles per SM cycle (INT in 31:16, FRAC in 15:8)."""
    if value < 1.0:
        raise ValueError(f"CLKDIV must be at least 1.0 (INT = 0 means 65536), got {value!r}")
    integer = int(value)
    frac = int(round((value - integer) * 256))
    if frac > 255:                      # rounding can carry into INT
        integer, frac = integer + 1, 0
    return (integer << 16) | (frac << 8)


def execctrl(wrap_bottom: int = 0, wrap_top: int = 31, jmp_pin: int = 0, side_en: int = 0,
             side_pindir: int = 0, status_sel: int = 0, status_n: int = 0) -> int:
    return (
        (side_en << 30)
        | (side_pindir << 29)
        | (jmp_pin << 24)
        | (wrap_top << 12)
        | (wrap_bottom << 7)
        | (status_sel << 4)
        | status_n
    )


def shiftctrl(in_right: int = 1, out_right: int = 1, autopush: int = 0, autopull: int = 0,
              push_thresh: int = 0, pull_thresh: int = 0) -> int:
    return (
        (pull_thresh << 25)
        | (push_thresh << 20)
        | (out_right << 19)
        | (in_right << 18)
        | (autopull << 17)
        | autopush << 16
    )


def pinctrl(in_base: int = 0, set_base: int = 0, set_count: int = 5, out_base: int = 0,
            out_count: int = 0, sideset_base: int = 0, sideset_count: int = 0) -> int:
    return (
        (sideset_count << 29)
        | (set_count << 26)
        | (out_count << 20)
        | (in_base << 15)
        | (sideset_base << 10)
        | (set_base << 5)
        | out_base
    )


# ---------------------------------------------------------------------------------------------
# The program
# ---------------------------------------------------------------------------------------------
PROGRAM_PATH = Path(__file__).with_name("program.pio")
DEFAULT_SM_CYCLES_PER_BIT = 16      # program.pio's `.define S`
RX_WORD_DATA_MASK = 0x00FF_FFFF     # bits a data word leaves clear; a marker sets one of them


class ProgramError(RuntimeError):
    pass


def _set_define(text: str, name: str, value: int) -> str:
    """Return `text` with the single `.define NAME ...` line's value replaced by `value`."""
    pattern = re.compile(rf"(?m)^(\s*\.define\s+{re.escape(name)}\s+)\S+")
    new, count = pattern.subn(rf"\g<1>{value}", text)
    if count != 1:
        raise ProgramError(
            f"{PROGRAM_PATH.name}: expected exactly one '.define {name}' line, found {count}"
        )
    return new


def build_program(sm_cycles_per_bit: int = DEFAULT_SM_CYCLES_PER_BIT,
                  framing: Optional[Framing] = None) -> Program:
    """Assemble the receiver for `sm_cycles_per_bit` SM cycles per bit time and a framing.

    With no framing -- or 8N1 -- this is `program.pio` itself, with its `.define S` line rewritten,
    because that is the file that ships and the existing tests check it.  Any other framing is the
    same program generated for it (`fw/uart_rx/framing.py`), which for 8N1 assembles to the shipped
    words exactly: `test_generated_8n1_is_the_shipped_program` is what holds the two together.

    The assembler's `.define` may not be redefined, so the source line is rewritten first.
    """
    if sm_cycles_per_bit < 4 or sm_cycles_per_bit > 32 or sm_cycles_per_bit % 2:
        raise ProgramError(
            f"sm_cycles_per_bit must be an even number of 4..32: the program samples the "
            f"middle of the start bit and of every data bit (so it must be even) and its "
            f"delays have to fit the 5-bit delay field (S-1 <= 31). Got {sm_cycles_per_bit}."
        )
    if framing is not None and framing != DEFAULT_FRAMING:
        return program_for(framing, sm_cycles_per_bit)
    text = _set_define(PROGRAM_PATH.read_text(), "S", sm_cycles_per_bit)
    programs = assemble(text)
    if len(programs) != 1:
        raise ProgramError(f"{PROGRAM_PATH.name} must hold exactly one .program")
    prog = programs[0]
    if len(prog.instrs) > 32:
        raise ProgramError(f"{prog.name} is {len(prog.instrs)} words, IMEM is 32")
    return prog


# ---------------------------------------------------------------------------------------------
# The receiver
# ---------------------------------------------------------------------------------------------
@dataclass
class RxByte:
    """One byte as the chip received it, decoded from one RX FIFO word.

    `value` is `data_bits` wide (bits 7:0 for the 8-bit framings, which is what the shipped
    program pushes), `framing_error` is set by the *program*: it saw a low stop bit and pushed the
    marker word after this one.  `parity_error` is set by the *host*: the parity bit the program
    sampled does not match the parity of the data bits.  The PIO subset
    cannot compare them itself -- see `fw/uart_rx/framing.py`'s module docstring -- so the bit
    travels up in the word and is checked here.
    """
    value: int
    framing_error: bool = False
    parity_error: bool = False


class UartRx:
    """The 8N1 receiver on engine `engine` of an ISS chip, on a `verif.vip.core.Bus`.

    `baud` sets `CLKDIV` for the program; the sender may then be up to about 5% off that rate
    (see program.pio's header) and back-to-back frames resynchronise at up to about 5% too.
    Nothing above `clk_hz / sm_cycles_per_bit` baud can work: one bit time is at least
    `sm_cycles_per_bit` core cycles, because `CLKDIV` cannot go below 1.

    `sm_cycles_per_bit` (S) trades speed for margin.  S must be at least 6 for back-to-back
    frames: after the stop bit the program needs 0.25S + 3 SM cycles to be back at `hunt`, and
    the next start bit arrives 10S - 9 * 0.02 * S SM cycles later.  The default S = 16 leaves
    0.36 of a bit time at -2% baud; S = 4 is one SM cycle short and loses the second frame.

    The host has to keep up with the line: the RX FIFO is four words deep and a UART has no
    flow control, so a host that stops reading loses the bytes that arrive while `push block`
    is stalled.
    """

    def __init__(self, baud: float = 115200, rx_pin: int = 0, clk_hz: float = 50e6,
                 ns_per_tick: float = 20.0, sm_cycles_per_bit: int = DEFAULT_SM_CYCLES_PER_BIT,
                 engine: int = 0, chip: Optional[Chip] = None,
                 framing: Optional[Framing] = None,
                 program: Optional[Program] = None) -> None:
        if not 0 <= rx_pin <= 12:
            raise ValueError(f"rx_pin must be 0..12 (13..17 are output-only), got {rx_pin}")
        #: The frame this receiver is built for (7 and 9 data bits, even and
        #: odd parity, two stop bits).  8N1, the shipped `program.pio`, is the default.  It decides
        #: the program (`fw/uart_rx/framing.py`), how many SM cycles a bit time is, and how one
        #: pushed word is split into a value, a parity bit and the marker space.
        self.framing = framing if framing is not None else DEFAULT_FRAMING
        self.layout = word_layout(self.framing)
        self.engine = engine
        self.rx_pin = rx_pin
        self.clk_hz = float(clk_hz)
        self.ns_per_tick = float(ns_per_tick)
        self.baud = float(baud)
        self.ticks_per_bit = self.clk_hz / self.baud
        self.sm_cycles_per_bit = int(sm_cycles_per_bit)
        if self.ticks_per_bit < self.sm_cycles_per_bit:
            raise ValueError(
                f"{self.baud:g} baud is {self.ticks_per_bit:.2f} ticks/bit at {self.clk_hz:g} Hz, "
                f"less than the {self.sm_cycles_per_bit} SM cycles the program spends per bit; "
                f"the fastest this program goes is {self.max_baud:g} baud"
            )
        #: `program` is the shipped one unless a caller hands one in, which is how a test runs a
        #: variant (a sample deleted, say) through the whole driver without touching the file --
        #: the same argument `fw/uart_tx/driver.py`'s `UartTxDriver` takes.
        self.program = (program if program is not None
                        else build_program(self.sm_cycles_per_bit, self.framing))
        self.chip = chip if chip is not None else Chip()
        self.bus = Bus(ns_per_tick=self.ns_per_tick)
        for k in range(18):
            self.bus.add_net(f"p{k}")
        self.dev = self.bus.attach(ChipDevice(self.chip, name="chip", prefix="p"))
        self.received: List[RxByte] = []
        self.markers: List[int] = []
        self.auto_read = True       # poll the RX FIFO every core cycle
        self._reads: List[Tuple[object, bool]] = []
        self._configure()

    # -- configuration ------------------------------------------------------------------------
    @property
    def max_baud(self) -> float:
        """Fastest baud this program can run at: one bit time is `sm_cycles_per_bit` cycles."""
        return self.clk_hz / self.sm_cycles_per_bit

    @property
    def net(self) -> str:
        return f"p{self.rx_pin}"

    @property
    def values(self) -> List[int]:
        """The byte values received so far, framing errors and all."""
        return [b.value for b in self.received]

    @property
    def framing_errors(self) -> int:
        return sum(1 for b in self.received if b.framing_error)

    @property
    def parity_errors(self) -> int:
        """Bytes whose parity bit did not match the data bits (host-side check, see `RxByte`)."""
        return sum(1 for b in self.received if b.parity_error)

    @property
    def good_values(self) -> List[int]:
        """The values of the bytes with no framing and no parity error, in order."""
        return [b.value for b in self.received
                if not b.framing_error and not b.parity_error]

    def _configure(self) -> None:
        """Queue the whole start sequence, then run the bus until it has been performed."""
        e = self.engine
        prog = self.program
        origin = prog.origin or 0
        self.dev.write(CTRL, 0)                                  # the engine is stopped
        for k, word in enumerate(prog.instrs):                   # the program
            self.dev.write(imem_addr(e, origin + k), word)
        self.dev.write(sm_addr(e, "CLKDIV"), clkdiv(self.ticks_per_bit / self.sm_cycles_per_bit))
        self.dev.write(
            sm_addr(e, "EXECCTRL"),
            execctrl(wrap_bottom=prog.wrap_target, wrap_top=prog.wrap, jmp_pin=self.rx_pin),
        )
        self.dev.write(sm_addr(e, "SHIFTCTRL"), shiftctrl(in_right=1, out_right=1))
        self.dev.write(
            sm_addr(e, "PINCTRL"),
            pinctrl(in_base=self.rx_pin, set_base=self.rx_pin, set_count=1),
        )
        self.dev.write(sm_addr(e, "INSTR"), 0xE080)              # forced set pindirs, 0
        self.dev.write(sm_addr(e, "INSTR"), prog.labels["hunt"])  # forced jmp hunt
        mask = 1 << e
        self.dev.write(CTRL, mask | (mask << 4) | (mask << 8))   # enable, restart, clkdiv restart
        while not self.dev.idle:
            self.bus.step()

    # -- running ------------------------------------------------------------------------------
    def step(self) -> None:
        """One core cycle: start a FIFO read if one is due, then tick the bus, then collect.

        `rx_valid` (IRQ0) is a registered line, so it stays high for a couple of core cycles
        after the FIFO has emptied: reading `RXF` on it alone would pop nothing and would set
        `FDEBUG.RXUNDER`.  The host therefore reads `FLEVEL` first and pops `RXF` only when the
        level says there is a word.
        """
        if self.auto_read and not self._reads and self.chip.irq0:
            self._reads.append((self.dev.read(FLEVEL), False))
        self.bus.step()
        while self._reads and self._reads[0][0].done():
            future, is_word = self._reads.pop(0)
            value = future.result()
            if is_word:
                self._decode(value)
            elif self.rx_level(value):
                self._reads.append((self.dev.read(RXF[self.engine]), True))

    def rx_level(self, flevel: int) -> int:
        """The RX FIFO occupancy that a read of `FLEVEL` returned (bits 7:4, 15:12)."""
        return (flevel >> (8 * self.engine + 4)) & 0xF

    def run(self, ticks: int) -> None:
        for _ in range(ticks):
            self.step()

    def run_until(self, cond: Callable[[], bool], limit: int) -> bool:
        """Step until `cond()` or `limit` ticks have passed; True if `cond()` ended it."""
        for _ in range(limit):
            if cond():
                return True
            self.step()
        return cond()

    def _decode(self, word: int) -> None:
        """One RX FIFO word: a data word, or a marker that flags the byte before it.

        The marker space is every bit below the frame (`fw/uart_rx/framing.py`'s `WordLayout`), so
        the byte at bits 31:24 is the shipped 8N1 layout and a 9-bit frame with parity puts its
        ninth bit and its parity bit at bits 23 and 22 with the marker space below them.  A marker
        of `PARITY_ERROR_MARKER` is one this module synthesised: the program cannot see a parity
        bit it has just shifted into the ISR.
        """
        if word & self.layout.marker_mask:
            self.markers.append(word)
            if self.received:
                if word == PARITY_ERROR_MARKER:
                    self.received[-1].parity_error = True
                else:
                    self.received[-1].framing_error = True
            return
        value = (word >> self.layout.data_shift) & self.layout.data_mask
        byte = RxByte(value=value)
        if self.framing.has_parity:
            received = (word >> self.layout.parity_shift) & 1
            byte.parity_error = received != self.framing.parity_bit(value)
            if byte.parity_error:
                # Reported the way the program reports a framing error: the byte first, then a
                # marker word, so `markers` reads the same for both kinds of error.
                self.markers.append(PARITY_ERROR_MARKER)
        self.received.append(byte)
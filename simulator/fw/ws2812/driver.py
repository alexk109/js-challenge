"""Host side of the MVP WS2812 firmware: load program.pio, set up one engine, send frames.

Everything here is written against `docs/mvp-regmap.md` (register word
addresses, the `### Typical host sequences` startup order, the per-engine fields) and
`docs/mvp-isa.md` (divider, side-set, shift direction, the blocking PULL).  Host accesses go
through `iss.pio_iss.ChipDevice`'s write/read queue, one per tick, so every wait in here is
the simulation running -- the same thing a real host does over the SPI port of
`docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                     # 50 MHz, one tick per core cycle
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())
    drv = Ws2812Driver(dev, bus, pin=0)           # engine 0, the ws2812 program
    drv.load()                 # PINCTRL first, then the line low, then IMEM and the rest
    drv.start()                # forced `jmp start`, then CTRL restart + enable
    drv.send([(255, 128, 0), (0, 255, 0)])        # one frame: two pixels, chain order
    drv.send([(1, 2, 3)])                        # the next frame, after the reset code

The chip core clock is 50 MHz: one `verif.vip.core.Bus` tick at `ns_per_tick = 20`, the
configuration the WS2812 VIP documents in its own docstring.

What this driver has to get right that a UART driver does not:

* **The line must be low before anything else happens.**  A low of more than 50 us is the
  reset code (page 3, "RES low voltage time Above 50us"), so the pin is driven low by the
  host before the engine is enabled.  `init_pins()` forces `set pindirs, 1` and
  `set pins, 0` through `SMe_INSTR`, which is the only way a host writes a pin level
  (`docs/mvp-regmap.md` `### Typical host sequences`, step 4), and it is called *before* the
  IMEM burst so that a data line with a pull-up reads low within a few ticks of reset.
* **The pin window has to be configured before the pin is forced.**  `PINCTRL` resets to
  `SET_BASE = 0, SET_COUNT = 5`, so a forced `set pindirs, 1` before `PINCTRL` is written
  would turn *five* pins into outputs.  `load()` therefore writes the four per-engine
  registers first and only then forces the level and the direction.
* **`OUT_SHIFTDIR = 0`.**  The datasheet sends a word most significant bit first
  (page 5), so the OSR has to shift left and the host has to push each pixel's 24 bits
  left-justified, in bits 31:8.  A driver that leaves `OUT_SHIFTDIR` at its reset value of 1
  would send every word backwards, and a strip would latch the colours in the wrong order.
* **Autopull stays off.**  The program's `pull block` is where the engine waits for the next
  pixel word, and it has to be able to stall: a transmitter that has run out of data must
  leave the line low, which is the reset code.
* **Frames are separated by the host, not by the engine.**  Nothing in a 24-bit word says
  where a frame ends, so the driver waits out the reset code after each frame instead of
  pushing the next one into the middle of the previous frame's reset.

Nothing in this module reads the ISS to decide what to send: the bit lengths, the word
packing and the divider all come from `program.pio` and the datasheet, and
`fw/ws2812/test_ws2812.py` measures the result back off the wire with the checker in
`verif/vip/ws2812.py`.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from math import ceil
from pathlib import Path
from typing import Iterable, Optional, Sequence, Union

from asm.pioasm import Program, assemble

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
NS_PER_TICK = 20.0
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
BITS_PER_PIXEL = 24           # page 5, "Composition of 24bit data"
PULL_THRESH = BITS_PER_PIXEL  # SHIFTCTRL.PULL_THRESH, so that `jmp !osre` counts one word
T_RESET_NS = 50_000.0         # page 3, "RES low voltage time Above 50us"
PROGRAM_PATH = Path(__file__).with_name("program.pio")
DEFAULT_PROGRAM = "ws2812"
PROGRAMS = ("ws2812", "ws2812_fast")
Color = Union[int, Sequence[int]]

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) ----
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF = regmap.txf_addrs()
A_RXF = regmap.rxf_addrs()
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_INSTR_MEM = regmap.imem_bases()    # engine e, word k: A_INSTR_MEM[e] + k
A_SM_BASE = regmap.sm_base()              # SMe_CLKDIV = A_SM_BASE + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

# CTRL fields (docs/mvp-regmap.md `**CTRL**`)
CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART = 0x001, 0x010, 0x100
# FSTAT fields, engine e (docs/mvp-regmap.md `**FSTAT**`)
FSTAT_TXEMPTY, FSTAT_TXFULL = 0x0100_0000, 0x0001_0000
# INTS bits (docs/mvp-regmap.md `**IRQ0_INTS / IRQ1_INTS**`)
INTS_RXNEMPTY, INTS_TXNFULL = 0x001, 0x010


def ticks_of(ns: float) -> int:
    """Nanoseconds to whole ticks at 50 MHz, rounded up (a minimum time, as `Bus.ticks`)."""
    return int(ceil(ns / NS_PER_TICK))


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def sm_cycles_bounds(n: int, clkdiv: int) -> tuple[int, int]:
    """The shortest and longest `n` SM cycles can take, in core cycles (ticks).

    One SM cycle is `INT` core cycles, plus one more whenever the 8-bit fractional accumulator
    wraps (`docs/mvp-isa.md` `### Clock divider`), so over `n` SM cycles the total is
    `n * INT + carries` with `floor(n * FRAC / 256) <= carries <= ceil(n * FRAC / 256)`.  With
    `FRAC = 0` both bounds are the same number, which is why the integer divider's grid is
    exact and the fractional one's is not.  `sm_cycles_ticks` is this bound's upper end plus a
    core cycle of slack, for waits that have to be safe rather than tight.
    """
    integer, frac = (clkdiv >> 16) & 0xFFFF or 0x10000, (clkdiv >> 8) & 0xFF
    return n * integer + (n * frac) // 256, n * integer + -(-n * frac // 256)


def sm_cycles_ticks(n: int, clkdiv: int) -> int:
    """The longest `n` SM cycles can take, in core cycles (ticks): `sm_cycles_bounds` + 1.

    The host uses this long bound for every wait it has to be sure of, so a frame's own timing
    jitter can only make the real thing shorter than the wait, never longer.
    """
    return sm_cycles_bounds(n, clkdiv)[1] + 1


# --------------------------------------------------------------------------------------------
# Host-forced instructions: the words a host writes to `SMe_INSTR` (docs/mvp-isa.md `### EXEC`)
# --------------------------------------------------------------------------------------------

def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address (`## Encoding`)."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (0b000 << 13) | addr


def set_pins_word(value: int) -> int:
    """`set pins, value`: a 5-bit immediate into the SET window, with no delay or side-set."""
    if not 0 <= value <= 0x1F:
        raise ValueError(f"SET data {value} does not fit in 5 bits")
    return (0b111 << 13) | (0b000 << 5) | value


def set_pindirs_word(value: int) -> int:
    """`set pindirs, value`: the same, through the direction registers instead of the levels."""
    if not 0 <= value <= 0x1F:
        raise ValueError(f"SET data {value} does not fit in 5 bits")
    return (0b111 << 13) | (0b100 << 5) | value


# --------------------------------------------------------------------------------------------
# The program: assemble it, and read back out of the words what it says about the timing
# --------------------------------------------------------------------------------------------

def load_program(name: str = DEFAULT_PROGRAM,
                 path: Path | str = PROGRAM_PATH) -> Program:
    """Assemble program.pio (`asm/pioasm.py`) and return the one program called `name`."""
    programs = assemble(Path(path).read_text(encoding="utf-8"))
    found = [p for p in programs if p.name == name]
    if not found:
        raise ValueError(
            f"unknown WS2812 program {name!r}: {path} has "
            f"{[p.name for p in programs]}"
        )
    return found[0]


def program_names(path: Path | str = PROGRAM_PATH) -> list[str]:
    """The names of the programs in a .pio file, in the order the assembler returns them."""
    return [p.name for p in assemble(Path(path).read_text(encoding="utf-8"))]


def decode_word(program: Program, word: int) -> dict:
    """Split one word into the fields `docs/mvp-isa.md` `## Encoding` defines.

    Only what the driver needs: the opcode, the side-set bits and the delay.  The split
    between those two depends on `PINCTRL.SIDESET_COUNT` rather than on the instruction, so
    it comes from the assembler's `.side_set` (through `Program.sideset_bits`, which counts
    the `opt` enable bit), not from the word.
    """
    sc = min(program.sideset_bits, 5)
    field = (word >> 8) & 0x1F
    return {
        "op": word >> 13,
        "sideset": (field >> (5 - sc)) if sc else 0,
        "delay": field & ((1 << (5 - sc)) - 1),
        "cycles": 1 + (field & ((1 << (5 - sc)) - 1)),
        "dest": (word >> 5) & 0x7,
        "count": (word & 0x1F) or 32,
        "low": word & 0xFF,
    }


def is_out_x_1(program: Program, word: int) -> bool:
    """True for `out x, 1`, the instruction that shifts one bit out and starts a pulse."""
    f = decode_word(program, word)
    return f["op"] == 0b011 and f["dest"] == 0b001 and f["count"] == 1


def is_pull_block(program: Program, word: int) -> bool:
    """True for a bare `pull block`, the instruction the engine waits on between frames."""
    return word >> 13 == 0b100 and (word & 0xFF) == 0xA0


def is_nop(program: Program, word: int) -> bool:
    """True for `nop`, which is `mov y, y` (docs/mvp-isa.md, the worked encoding table).

    Bits 12:8 are masked out because they are the delay and side-set field, which is what
    tells a `nop` how long the low time it is part of lasts.
    """
    return word & 0xE0FF == 0xA042


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


def max_cycles_per_instruction(program: Program) -> int:
    """The longest one instruction of this program can be, in SM cycles.

    `.side_set 1` leaves the delay field `5 - 1 = 4` bits wide, so an instruction lasts
    1..16 SM cycles (`docs/mvp-isa.md` `### Delay/side-set field`).  This is the granularity
    every timing decision in the program is made of, which is why it is worth naming.
    """
    sc = min(program.sideset_bits, 5)
    return 1 << (5 - sc)


@dataclass(frozen=True)
class Timing:
    """What one bit costs, in SM cycles, as the words of the program say.

    The high time is the run of `side 1` instructions from `bitloop` to the end of the pulse,
    which is `high_zero` for a 0 bit (the branch instruction is the last of it) and
    `high_one` for a 1 bit.  The low time is everything after that until the next `bitloop`,
    which is `low_one` for a 1 bit and `low_zero` for a 0 bit: the `zero` path adds the extra
    low a short pulse needs.  `pull` is *not* part of either -- the `pull block` runs once per
    word, not once per bit, because `jmp !osre` loops over the word's other 23 bits -- so it is
    reported on its own and `word_cycles` adds it back where a frame is measured.
    """

    pull: int
    high_zero: int
    high_one: int
    low_zero: int
    low_one: int

    @property
    def bit_cycles(self) -> int:
        """SM cycles one bit lasts.  `timing()` checks that both bit values agree."""
        return self.high_one + self.low_one

    @property
    def word_cycles(self) -> int:
        """SM cycles one pixel word lasts: its bits plus the one `pull block` that fetched it."""
        return BITS_PER_PIXEL * self.bit_cycles + self.pull


def timing(program: Program) -> Timing:
    """Derive the per-bit timing from the assembled words, and check it is what a WS2812 wants.

    Every path through the program is between two labels, and every instruction is a fixed
    number of SM cycles long, so the high and low times are sums over those ranges rather
    than guesses.  Anything the driver could not time -- a side-set the program does not have,
    a `bitloop` that is not an `out x, 1`, a program whose two bit values do not come out the
    same length -- raises here instead of producing a divisor that means something else.
    """
    if program.sideset_bits != 1 or program.sideset_opt or program.sideset_pindirs:
        raise ValueError(
            f"{program.name}: the WS2812 line is driven through `.side_set 1` with no `opt` "
            f"and no `pindirs`, so `side 0`/`side 1` is one pin's level; the assembler read "
            f"sideset_bits={program.sideset_bits}, opt={program.sideset_opt}, "
            f"pindirs={program.sideset_pindirs}"
        )
    missing = {"start", "bitloop", "one", "one_more", "zero", "low", "more"} - set(program.labels)
    if missing:
        raise ValueError(f"{program.name}: the program has no label(s) {sorted(missing)}")
    words = program.instrs
    if not is_pull_block(program, words[program.labels["start"]]):
        raise ValueError(
            f"{program.name}: `start` is not a bare `pull block`; the engine has to be able to "
            f"stall there with the line low, because that is the reset code"
        )
    if not is_out_x_1(program, words[program.labels["bitloop"]]):
        raise ValueError(
            f"{program.name}: `bitloop` is not `out x, 1`, so it would not shift one bit out "
            f"per high pulse"
        )

    start = program.labels["start"]
    loop, one = program.labels["bitloop"], program.labels["one"]
    one_more, zero = program.labels["one_more"], program.labels["zero"]
    low, more = program.labels["low"], program.labels["more"]
    if not (loop <= one <= one_more + 1 == zero <= low <= more < start == len(words) - 1):
        raise ValueError(
            f"{program.name}: the labels are out of order ({start=}, {loop=}, {one=}, "
            f"{one_more=}, {zero=}, {low=}, {more=}); the timing below walks them as the two "
            f"paths of one bit and the end of the loop, which have to nest that way, tile the "
            f"program with no dead words, and leave the `pull block` as the last instruction"
        )
    back = words[more]
    if back >> 13 != 0b000 or (back >> 5) & 0x7 != 0b111 or back & 0x1F != loop:
        raise ValueError(
            f"{program.name}: the instruction at `more` (0x{back:04X}) is not `jmp !osre "
            f"bitloop`, which is the only bit counter the ISA has: without it the loop would "
            f"fetch a new word after every bit instead of after {BITS_PER_PIXEL}"
        )
    fields = [decode_word(program, w) for w in words]

    def side_ok(lo: int, hi: int, side: int, what: str) -> None:
        bad = [i for i in range(lo, hi) if fields[i]["sideset"] != side]
        if bad:
            raise ValueError(
                f"{program.name}: {what} is instruction(s) {bad} with side-set "
                f"{[fields[i]['sideset'] for i in bad]}, expected {side}"
            )

    # `loop` is where every high pulse starts, `one` is where the branch lets a 1 bit
    # continue, so the 0 bit's pulse is [loop, one) and the 1 bit's is [loop, one_more + 1).
    # Everything from `zero` on is low, and the two paths meet again at `low`.  The `jmp
    # !osre` at `more` is part of every low time -- it runs once per bit -- but the `pull
    # block` after it is not: that one runs once per word.
    side_ok(loop, one, 1, "the high time of a 0 bit")
    side_ok(one, one_more + 1, 1, "the rest of the high time of a 1 bit")
    side_ok(zero, len(words), 0, "the low time")
    if fields[start]["sideset"] != 0:
        raise ValueError(
            f"{program.name}: the `pull block` at `start` side-sets "
            f"{fields[start]['sideset']}; a transmitter waiting for data must hold the line low"
        )
    nops = [i for i in range(zero, low) if not is_nop(program, words[i])]
    if nops:
        raise ValueError(
            f"{program.name}: the low time of a 0 bit is instruction(s) {nops}, which are not "
            f"all `nop`; anything that branches there would not be a straight-line low"
        )

    cycles = [f["cycles"] for f in fields]
    out = Timing(
        pull=cycles[start],
        high_zero=sum(cycles[loop:one]),
        high_one=sum(cycles[loop:one_more + 1]),
        low_zero=sum(cycles[zero:more + 1]),
        low_one=sum(cycles[low:more + 1]),
    )
    if out.high_zero + out.low_zero != out.bit_cycles:
        raise ValueError(
            f"{program.name}: a 0 bit is {out.high_zero}+{out.low_zero} SM cycles and a 1 bit "
            f"{out.high_one}+{out.low_one}; the bit grid has to be the same length for both or "
            f"the bit period drifts"
        )
    if out.word_cycles <= out.bit_cycles:
        raise ValueError(
            f"{program.name}: {out.word_cycles} SM cycles for a word is not longer than the "
            f"{out.bit_cycles} of a bit, so the `pull block` cannot be where it is"
        )
    return out


# --------------------------------------------------------------------------------------------
# Register values the program needs (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program, *, wrap_bottom: int = 0) -> int:
    """EXECCTRL: the wrap window the assembler worked out, and SIDE_EN left at 0.

    SIDE_EN = 0 is what makes `.side_set 1` mean "this one pin's level": with
    `PINCTRL.SIDESET_COUNT = 1` the top bit of the side-set field is then data, not an enable
    (`docs/mvp-isa.md` `### Delay/side-set field`), and every instruction drives the line --
    which is the whole mechanism the WS2812 wire is made of.  With SIDE_EN = 1 the same one
    bit would be the enable and no instruction would write anything at all.
    """
    if not 0 <= wrap_bottom <= 31:
        raise ValueError(f"WRAP_BOTTOM {wrap_bottom} outside 0..31")
    return (program.wrap << 12) | (wrap_bottom << 7)


def shiftctrl_value(*, out_shiftdir: int = 0, autopull: int = 0,
                    pull_thresh: int = PULL_THRESH) -> int:
    """SHIFTCTRL: shift *left*, no autopull, and a pull threshold of 24.

    `OUT_SHIFTDIR = 0` is the important one.  With it, `out x, 1` puts OSR bit 31 on the pin
    and shifts left (`docs/mvp-isa.md` `### Instruction semantics`, OUT), which is the order
    the datasheet sends a word in -- most significant bit first, green byte first (page 5) --
    so the host pushes each pixel's 24 bits left-justified, in bits 31:8.  The register resets
    to `OUT_SHIFTDIR = 1`, which would send every word backwards.

    `AUTOPULL = 0` because the program refills the OSR with its own blocking `pull`, and that
    pull is what holds the line low between frames: an autopulled `out x, 1` would stall in
    the middle of a high pulse instead, and a stalled instruction keeps the level it last
    wrote (`docs/mvp-isa.md` `### Stalls, delay and side-set`).

    `PULL_THRESH = 24` is the bit counter.  The JMP condition `!osre` is `osr_cnt <
    thresh(PULL_THRESH)` (`docs/mvp-isa.md` `### Field values`), so with one bit shifted per
    `out x, 1` and this threshold the program's `jmp !osre bitloop` loops 24 times -- one
    word -- and then falls through to the `pull block`.  At the reset value of 0 the
    threshold is 32 and the program would count 32 bits, which is not a word.
    """
    if out_shiftdir not in (0, 1) or autopull not in (0, 1):
        raise ValueError("OUT_SHIFTDIR and AUTOPULL are single bits")
    if not 1 <= pull_thresh <= 31:
        raise ValueError(f"PULL_THRESH {pull_thresh} out of range 1..31")
    return (pull_thresh << 25) | (out_shiftdir << 19) | (autopull << 17)


def pinctrl_value(pin: int, *, sideset_count: int = 1, set_count: int = 1,
                  out_count: int = 1) -> int:
    """PINCTRL: every window of the program points at the data pin, one pin wide.

    The side-set window is what drives the line during the program; the SET window is what
    the host's forced `set pins, 0` / `set pindirs, 1` use before the engine starts, and OUT
    is never used by either program (it writes bits through the side-set field, not through
    `out pins`).  All three are pointed at the same pin so that no window can reach a second
    one: a data pin that is not `uio` has no direction register at all
    (`docs/mvp-isa.md` `### The 18 protocol pins`).
    """
    if not 0 <= pin < NUM_PINS:
        raise ValueError(f"data pin {pin} is outside 0..{NUM_PINS - 1}")
    if not all(0 <= b <= 32 for b in (sideset_count, set_count, out_count)):
        raise ValueError("window counts must be 0..32")
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (pin << 15)
            | (pin << 10) | (pin << 5) | pin)


# --------------------------------------------------------------------------------------------
# The wire format: one pixel as the 24 bits of a datasheet word, and as the OSR wants them
# --------------------------------------------------------------------------------------------

def grb_word(rgb: Color) -> int:
    """One pixel as the datasheet's 24-bit word: G in 23:16, R in 15:8, B in 7:0.

    Page 5 spells the word out as "G7G6G5G4G3G2G1G0R7R6R5R4R3R2R1R0B7B6B5B4B3B2B1B0",
    which is this packing.  It is the *wire* order and it is not the host's order: the host
    asks for a colour as 0xRRGGBB or as an (r, g, b) triple.
    """
    if isinstance(rgb, int):
        red, green, blue = (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF
    else:
        red, green, blue = (int(c) & 0xFF for c in rgb)
    return ((green & 0xFF) << 16) | ((red & 0xFF) << 8) | (blue & 0xFF)


def osr_word(word: int) -> int:
    """A datasheet word left-justified into the OSR, in bits 31:8.

    `OUT_SHIFTDIR = 0` sends bit 31 first, and the datasheet wants the word's bit 23 first,
    so the word goes in eight ticks higher than it looks.  A host port that moves whole
    32-bit words (docs/mvp-isa.md `## Differences from RP2040`, item 11) has to left-justify
    the pixel data itself; that is this function, and it is the only host-side packing step
    there is.
    """
    if not 0 <= word <= 0xFF_FFFF:
        raise ValueError(f"a pixel word is 24 bits (got {word:#x})")
    return (word & 0xFF_FFFF) << 8


@dataclass(frozen=True)
class Config:
    """One WS2812 setup: which engine, which pin, which program and which CLKDIV fraction."""

    engine: int = 0
    pin: int = 0
    program: str = DEFAULT_PROGRAM
    frac: int = 0               # CLKDIV FRAC: 0 is 1.0, i.e. one core cycle per SM cycle

    def __post_init__(self) -> None:
        if not 0 <= self.frac <= 0xFF:
            raise ValueError(
                f"CLKDIV FRAC {self.frac} out of range 0..255: it is 8 bits, in 1/256"
            )

    @property
    def clkdiv(self) -> int:
        return clkdiv_value(1, self.frac)


class Ws2812Driver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as a WS2812 controller.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are
    queued on `dev` and land one per tick as the simulation runs, so `read()` and `send()`
    pump the bus while they wait, exactly as a host does over `docs/mvp-regmap.md`
    `## SPI transaction format`.
    """

    MAX_TICKS = 8_000_000

    @property
    def TXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)
    RESET_TICKS = ticks_of(T_RESET_NS)

    def __init__(self, dev, bus, *, engine: int = 0, pin: int = 0,
                 program: str | Program = DEFAULT_PROGRAM, frac: int = 0,
                 path: Path | str = PROGRAM_PATH) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        if pin not in UIO_PINS:
            raise ValueError(
                f"data pin {pin} is not a uio pin {list(UIO_PINS)}: a WS2812 line has to be "
                f"driven, and PINDIRS cannot drive input-only or output-only protocol pins "
                f"(docs/mvp-isa.md `### The 18 protocol pins`)"
            )
        self.dev, self.bus = dev, bus
        self.engine, self.pin = engine, pin
        self.program = program if isinstance(program, Program) else load_program(program, path)
        self.config = Config(engine=engine, pin=pin, frac=frac,
                             program=self.program.name)
        self.timing = timing(self.program)
        self.peak_tx_level = 0        # deepest the TX FIFO got, read from FLEVEL by `push`

    # ---- addresses ----
    @property
    def imem_base(self) -> int:
        return A_INSTR_MEM[self.engine]

    def sm_addr(self, index: int) -> int:
        return A_SM_BASE + 6 * self.engine + index

    @property
    def start_addr(self) -> int:
        return self.program.labels["start"]

    @property
    def words_used(self) -> int:
        return words_used(self.program)

    # ---- the register values this configuration asks for ----
    @property
    def clkdiv(self) -> int:
        return self.config.clkdiv

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value()

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(self.pin, sideset_count=self.program.sideset_bits)

    # ---- what the wire will do, from the words and the divider ----
    @property
    def bit_cycles(self) -> int:
        """SM cycles one bit lasts (`timing()`, read out of the assembled words)."""
        return self.timing.bit_cycles

    def bit_ticks(self, clkdiv: Optional[int] = None) -> float:
        """The average length of one bit on the wire, in core cycles (ticks)."""
        word = self.clkdiv if clkdiv is None else clkdiv
        integer, frac = (word >> 16) & 0xFFFF, (word >> 8) & 0xFF
        return self.bit_cycles * (integer + frac / 256.0)

    def bit_ticks_max(self, clkdiv: Optional[int] = None) -> int:
        """The longest one bit can take on the wire, in ticks (`sm_cycles_ticks`)."""
        return sm_cycles_ticks(self.bit_cycles, self.clkdiv if clkdiv is None else clkdiv)

    def period_bounds(self, sm_cycles: Optional[int] = None,
                      clkdiv: Optional[int] = None) -> tuple[int, int]:
        """The shortest and longest a run of `sm_cycles` SM cycles can take on the wire.

        Defaults to one bit.  With an integer divider the two ends are the same number, so the
        grid is exact; with a fraction they differ by the number of times the accumulator
        wrapped, which is the jitter a fractional `CLKDIV` puts on the wire.
        """
        return sm_cycles_bounds(self.bit_cycles if sm_cycles is None else sm_cycles,
                                self.clkdiv if clkdiv is None else clkdiv)

    @property
    def bit_rate_hz(self) -> float:
        """The bit rate this configuration drives the wire at, from the chip's own clock."""
        return CORE_HZ / self.bit_ticks()

    def frame_ticks(self, pixels: int) -> int:
        """The longest `pixels` pixels can take on the wire, in ticks.

        One word is `BITS_PER_PIXEL` bit periods plus the `pull block` that fetched it
        (`Timing.word_cycles`), because the pull runs once per word rather than once per bit.
        `send()` uses this to know how long to let the engine run before a frame is over, so
        that the reset code which follows it is a real reset code.
        """
        if pixels < 1:
            raise ValueError(f"a frame is at least one pixel (got {pixels})")
        return sm_cycles_ticks(pixels * self.timing.word_cycles, self.clkdiv)

    def words(self, pixels: Iterable[Color]) -> list[int]:
        """The TX FIFO words for one frame: one per pixel, chain order, GRB and left-justified.

        The first pixel of the chain gets the first word (page 5, "The data of D1 is send by
        MCU, and D2, D3, D4 through pixel internal reshaping amplification to transmit"), so
        the host's order is the wire's order.  `push()` is the other direction: it takes a raw
        32-bit FIFO word, so a host that has packed its own can skip this.
        """
        return [osr_word(grb_word(pixel)) for pixel in pixels]

    # ---- host accesses: queued on the ChipDevice, one per tick ----
    def write(self, addr: int, data: int) -> None:
        """Queue one host write (docs/mvp-regmap.md: it takes effect at the end of its bus
        access cycle `h`, and the engines use the new configuration from `h+1`)."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def _run(self, ticks: int) -> None:
        """Step the bus for `ticks` core cycles, letting every queued access land."""
        for _ in range(ticks):
            self.bus.step()

    def _settle(self, extra: int = 2) -> None:
        """Run the bus until the host queue is empty, then `extra` more ticks."""
        ticks = 0
        while not self.dev.idle:
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host writes still queued after {ticks} ticks")
        self._run(extra)

    def read(self, addr: int) -> int:
        """Queue one host read and run the bus until its data comes back.

        The value is the register's state in the access cycle, so a FIFO pop or a W1C
        happens as this returns (docs/mvp-regmap.md `### Cycle-exact mapping to bus
        accesses`).
        """
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host read of {addr:#04x} unanswered after {ticks} ticks")
        return int(future.result())

    # ---- register reads ----
    def ctrl(self) -> int:
        return self.read(A_CTRL)

    def fstat(self) -> int:
        return self.read(A_FSTAT)

    def flevel(self) -> int:
        return self.read(A_FLEVEL)

    def fdebug(self) -> int:
        return self.read(A_FDEBUG)

    def clear_fdebug(self) -> int:
        """Write FDEBUG back to clear the sticky stall flags (W1C), and return what was set."""
        value = self.fdebug()
        if value:
            self.write(A_FDEBUG, value)
            self._settle()
        return value

    def padout(self) -> int:
        return self.read(A_DBG_PADOUT)

    def padoe(self) -> int:
        return self.read(A_DBG_PADOE)

    def cfginfo(self) -> int:
        return self.read(A_DBG_CFGINFO)

    def pc(self) -> int:
        return self.read(self.sm_addr(SM_ADDR))

    def irq_ints(self, line: int = 0) -> int:
        """IRQx_INTS: the PIO's own interrupt lines, the ones `rx_valid`/`tx_ready` mirror."""
        return self.read(0x4D if line == 0 else 0x50)

    @property
    def tx_level(self) -> int:
        return (self.flevel() >> (8 * self.engine)) & 0xF

    @property
    def tx_full(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXFULL << self.engine))

    @property
    def tx_empty(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXEMPTY << self.engine))

    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask

    # ---- the startup sequence (docs/mvp-regmap.md `### Typical host sequences`) ----
    def load(self) -> None:
        """Steps 1-4: stop the engine, configure it, drive the line low, load the program.

        The order differs from the document's list in one place, and only one.  The document
        has the program burst before the registers because on an RP2040 the pin windows are
        written before the code runs either way; here the host has to *force* a `set pindirs,
        1` to make the data pin an output, and `PINCTRL` resets to `SET_BASE = 0, SET_COUNT =
        5`, so a forced `set pindirs, 1` written before `PINCTRL` would turn five pins into
        outputs.  The four registers therefore go in before the first forced pin write, and
        the IMEM burst after it: nothing is enabled yet, so the engine cannot execute any of
        it in between.
        """
        self.stop()
        self.write(self.sm_addr(SM_CLKDIV), self.clkdiv)
        self.write(self.sm_addr(SM_EXECCTRL), self.execctrl)
        self.write(self.sm_addr(SM_SHIFTCTRL), self.shiftctrl)
        self.write(self.sm_addr(SM_PINCTRL), self.pinctrl)
        self._settle()
        self.init_pins()
        for k, word in enumerate(self.program.instrs):
            self.write(self.imem_base + k, word)
        self._settle()

    def init_pins(self) -> None:
        """Step 4: the data pin becomes a driven low before anything else happens.

        Both writes are forced through `SMe_INSTR`, which is the only way a host writes a pin
        level (`docs/mvp-isa.md` `### EXEC`): a forced instruction runs in the very next core
        cycle whatever the divider and `SM_ENABLE` say.  Low is the level to leave the pin at
        because a low of more than 50 us is the reset code (page 3, "Above 50us"), so the
        strip is latched and ready when the first frame starts, and so that a data line with a
        pull-up reads low within a few ticks of reset instead of showing a high pulse that a
        receiver could mistake for a 0 bit.

        The two forced words carry no side-set field, so with `SIDE_EN = 0` each of them also
        side-sets the data pin low -- which is what is wanted here, and the reason this driver
        could not pre-load the line *high* the same way: the side-set channel beats the
        SET channel on the same pin (`docs/mvp-isa.md` `### Pin writes`).
        """
        self.write(self.sm_addr(SM_INSTR), set_pindirs_word(1))
        self.write(self.sm_addr(SM_INSTR), set_pins_word(0))
        self._settle(2)

    def arm(self) -> None:
        """Step 5: force `jmp start` through `SMe_INSTR`, the only way to set the PC.

        `SMe_ADDR` is read-only, and a forced instruction does not advance the PC
        (`docs/mvp-isa.md` `### EXEC`), so this lands the engine on the `pull block` and
        leaves it parked there with the line low until the host pushes a word.
        """
        self.write(self.sm_addr(SM_INSTR), jmp_abs(self.start_addr))
        self._settle(2)

    def enable(self) -> None:
        """Step 6: SM_ENABLE with SM_RESTART and CLKDIV_RESTART in one CTRL write.

        The engine's first SM cycle is then two core cycles after this write
        (`docs/mvp-isa.md` `### Clock divider`), so the engine and the divider start on a
        known phase whatever the divider was doing before.
        """
        ctrl = self.read(A_CTRL) & ~self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, ctrl | self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def start(self) -> None:
        """Arm and enable, with no data in the FIFO: the engine waits on `pull block`."""
        self.arm()
        self.enable()

    def stop(self) -> None:
        """Disable the engine, keeping every register (`docs/mvp-isa.md` `### Enable, restart`)."""
        ctrl = self.read(A_CTRL)
        self.write(A_CTRL, ctrl & ~self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def restart(self) -> None:
        """SM_RESTART + CLKDIV_RESTART, keeping the enable (`docs/mvp-regmap.md` `**CTRL**`)."""
        ctrl = self.read(A_CTRL)
        keep = self.ctrl_bits(CTRL_SM_ENABLE)
        self.write(A_CTRL, ctrl | (self.ctrl_bits(*self.CTRL_BITS) & ~keep))
        self._settle()

    # ---- the TX FIFO ----
    def push(self, word: int) -> None:
        """Push one pixel word, waiting for room first so nothing is dropped (TXOVER).

        The word is a datasheet 24-bit GRB word left-justified into bits 31:8
        (`osr_word`), because `OUT_SHIFTDIR = 0` sends the most significant bit first.  A bare
        integer is accepted too, so a host can push a raw 32-bit FIFO word; it is checked to be
        32 bits and nothing else, because the FIFO does not know what a pixel is.
        """
        if not 0 <= word <= 0xFFFF_FFFF:
            raise ValueError(f"a FIFO word is 32 bits (got {word:#x})")
        while self.tx_full:
            self._settle(1)
        self.peak_tx_level = max(self.peak_tx_level, self.tx_level + 1)
        self.write(A_TXF[self.engine], word)
        self._settle(1)

    def preload(self, pixels: Sequence[Color]) -> None:
        """Fill the TX FIFO with a whole frame while the engine is still stopped.

        Only `TXFIFO_DEPTH` words fit and a disabled engine never pops, so this is the way to
        have the first frame ready before the engine is enabled: the program's `pull block`
        would otherwise stall on the empty FIFO and set `FDEBUG.TXSTALL` on its first SM cycle.
        """
        words = self.words(pixels)
        if len(words) > self.TXFIFO_DEPTH:
            raise ValueError(
                f"{len(words)} pixel(s) do not fit the stopped {self.TXFIFO_DEPTH}-deep TX "
                f"FIFO; call send() after start()"
            )
        for word in words:
            self.push(word)

    # ---- one frame ----
    def send(self, pixels: Sequence[Color]) -> None:
        """Send one frame -- the pixels in chain order -- and wait out the reset code.

        The WS2812 wire carries no frame marker, so where a frame ends is decided entirely by
        the host: nothing but a low of more than 50 us tells a strip that the frame it has been
        receiving is complete (`verif/vip/ws2812.py`, `Ws2812Pixel._latch`).  This therefore
        waits `frame_ticks(n)` for the bits to go out and then `RESET_TICKS` for the line to
        stay low, both bounded from above by the words and the divider, so the next frame
        cannot start inside this one's reset.  A caller that sends one frame and stops leaves
        the line low, which is where a WS2812 line idles.
        """
        words = self.words(pixels)
        if not words:
            raise ValueError("a frame needs at least one pixel")
        for word in words:
            self.push(word)
        self._settle(self.frame_ticks(len(words)) + self.RESET_TICKS + 64)
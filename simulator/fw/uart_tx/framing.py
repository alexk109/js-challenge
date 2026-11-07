"""The UART frame the TX firmware can put on the wire: data bits, parity, stop bits.

`fw/uart_tx/program.pio` is one shape -- 8 data bits, no parity, one stop bit -- and this module
is what turns that shape into any other (7 or 9 data bits, even or odd parity,
two stop bits) without a second copy of the program per framing.  Two properties are load-bearing:

*   the generated program for **8N1 is the shipped `program.pio`, word for word**.  The generator
    emits the same instructions in the same order with the same delays, and
    `fw/uart_tx/test_uart_tx_framing.py::test_generated_8n1_is_the_shipped_program` assembles both
    and compares the words.  So the shipped 8N1 firmware cannot drift away from the generator, and
    the existing 8N1 tests keep testing the program that is actually in the tree.
*   the framing arithmetic is written here, not imported from the judge.  `pack()` and
    `parity_bit()` below are one implementation of the frame convention (start 0, data LSB first,
    parity over the data bits, stops 1) and `verif/vip/uart.py` is another; a test compares the
    two over all 256 byte values, so the convention is pinned by two independent implementations
    instead of being read identically by both sides of the same test.

Why the parity bit is a host computation.  The program shifts out `data_bits` data bits and then,
if the framing has parity, one more bit, and the host puts the parity bit in the TX word above the
data.  The PIO subset cannot compute parity: `JMP`'s conditions are X, Y, ISR-non-zero, OSR,
GPIO and IRQ only (`docs/mvp-isa.md` `## Encoding`), and `MOV`'s ops are invert and reverse
(`### Instruction semantics`, MOV), so there is no way to fold a register's bits together and
branch on the result.  This is not a gap in the check: the VIP receiver on the other end of the
wire is told the parity mode and raises `parity_error` if the bit on the wire is wrong, so a
program that shifts the wrong bit out is caught by the judge, not by this module.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from asm.pioasm import Program, assemble

#: `data_bits` 5..9 and `stop_bits` 1..2, the ranges `verif/vip/uart.py` accepts and the ones a
#: UART frame has.  `9` is a ninth data bit in the OSR and in the ISR, both 32 bits wide.
DATA_BITS_MIN, DATA_BITS_MAX = 5, 9
STOP_BITS = (1, 2)
PARITIES = ("none", "even", "odd")

#: `.define BIT_DELAY 7` of `program.pio`: a bit is `1 + BIT_DELAY` = 8 SM cycles.  The
#: generator emits the same number, so the same divider derivation works for every framing.
BIT_DELAY = 7
SM_CYCLES_PER_BIT = 1 + BIT_DELAY


@dataclass(frozen=True)
class Framing:
    """One frame shape: `data_bits` data bits, `parity` (none/even/odd), `stop_bits` stops."""

    data_bits: int = 8
    parity: str = "none"
    stop_bits: int = 1

    def __post_init__(self) -> None:
        if not DATA_BITS_MIN <= int(self.data_bits) <= DATA_BITS_MAX:
            raise ValueError(
                f"data_bits must be {DATA_BITS_MIN}..{DATA_BITS_MAX} (got {self.data_bits!r}): a "
                "UART frame is 5 to 9 data bits wide and the OSR/ISR are 32 bits"
            )
        parity = str(self.parity).lower()
        if parity not in PARITIES:
            raise ValueError(f"parity must be one of {PARITIES} (got {self.parity!r})")
        if int(self.stop_bits) not in STOP_BITS:
            raise ValueError(f"stop_bits must be one of {STOP_BITS} (got {self.stop_bits!r})")
        object.__setattr__(self, "parity", parity)
        object.__setattr__(self, "data_bits", int(self.data_bits))
        object.__setattr__(self, "stop_bits", int(self.stop_bits))

    @property
    def has_parity(self) -> bool:
        return self.parity != "none"

    @property
    def shift_bits(self) -> int:
        """Bits the program shifts out per byte: the data bits, plus the parity bit if any.

        This is `SHIFTCTRL.PULL_THRESH` too: the autopull refills the OSR once this many bits have
        gone out, so one TX word is one frame's worth of data and parity.
        """
        return self.data_bits + (1 if self.has_parity else 0)

    @property
    def frame_bits(self) -> int:
        """Bits on the wire per byte: start, data, parity, stops."""
        return 1 + self.shift_bits + self.stop_bits

    @property
    def mask(self) -> int:
        """The bits of a byte value that are data, `data_bits` wide."""
        return (1 << self.data_bits) - 1

    @property
    def word_mask(self) -> int:
        """The bits of a *TX word* this framing uses: data bits plus the parity bit, if any."""
        return (1 << self.shift_bits) - 1

    def parity_bit(self, value: int) -> int:
        """The parity bit of `value`: 0 if the frame's 1s should be even, 1 for odd.

        Written out rather than imported, so that the judge (`verif/vip/uart.py`) and the
        stimulus agree by two implementations of the same convention instead of by one shared
        function.  `test_parity_bit_agrees_with_the_judge` is what turns that into evidence.
        """
        ones = (int(value) & self.mask).bit_count()
        return (ones % 2) ^ (1 if self.parity == "odd" else 0)

    def pack(self, value: int) -> int:
        """`value` as the word the host pushes into the TX FIFO for this framing.

        Right-justified, because `SHIFTCTRL.OUT_SHIFTDIR = 1` (its reset value) shifts the OSR
        right and `out pins, 1` sends bit 0 first, which is the order a UART frame is in.  The
        parity bit, if the framing has one, sits directly above the data bits at bit
        `data_bits`, which is where the program shifts it out from.
        """
        if not 0 <= int(value) <= self.mask:
            raise ValueError(
                f"{self} carries {self.data_bits} data bits, so a byte is 0..{self.mask} "
                f"(got {value!r})"
            )
        word = int(value) & self.mask
        if self.has_parity:
            word |= self.parity_bit(word) << self.data_bits
        return word

    @property
    def name(self) -> str:
        """`8E2`-style tag, the way a datasheet writes it."""
        return f"{self.data_bits}{self.parity[0].upper()}{self.stop_bits}"

    def __str__(self) -> str:
        return self.name


DEFAULT = Framing()


def all_framings() -> Iterator[Framing]:
    """Every framing this firmware can send, in a fixed order (the order the tests use)."""
    for data_bits in (7, 8, 9):
        for parity in PARITIES:
            for stop_bits in STOP_BITS:
                yield Framing(data_bits, parity, stop_bits)


# ---------------------------------------------------------------------------------------------
# The program
# ---------------------------------------------------------------------------------------------

def program_source(framing: Framing = DEFAULT) -> str:
    """The PIO source of the transmitter for `framing`.

    The shape is `program.pio`'s, unchanged: one instruction per bit, so a bit lasts
    `1 + BIT_DELAY` SM cycles and `CLKDIV` sets the baud exactly as `fw/uart_tx/driver.py`
    derives it.  What changes with the framing is only how many `out pins, 1` (data bits, plus
    the parity bit if the framing has one) and how many `set pins, 1` (stop bits) sit between the
    start bit and the jump back to `start`.

    For 8N1 this emits `program.pio` word for word, which is the check that keeps the two from
    drifting apart.
    """
    data = [f"        out pins, 1        [BIT_DELAY]   ; data bit {i}"
            for i in range(framing.data_bits)]
    if framing.has_parity:
        data.append("        out pins, 1        [BIT_DELAY]   ; the parity bit, computed by the "
                    "host and packed above the data")
    stops = [f"        set pins, 1        [BIT_DELAY]   ; stop bit"
             for _ in range(framing.stop_bits)]
    lines = [
        ";",
        f"; UART TX, {framing.name}, generated by fw/uart_tx/framing.py from the shape of",
        "; fw/uart_tx/program.pio.",
        ";",
        "; One instruction per bit, so a bit lasts (1 + BIT_DELAY) SM cycles.  CLKDIV then sets",
        "; the baud: one bit = (1 + BIT_DELAY) * CLKDIV core cycles.  See fw/uart_tx/driver.py,",
        "; which builds every register this program needs and derives the divisor from",
        "; these words.",
        ";",
        "; The host pushes each byte right-justified, in bits {0}:{1},".format(
            framing.shift_bits - 1, 0),
        "; with the parity bit, if the framing has one, at bit {0}.".format(framing.data_bits),
        "; The program frames it: start bit, {0} data bits{1}, {2} stop bit{3}.".format(
            framing.data_bits,
            ", one parity bit" if framing.has_parity else "",
            framing.stop_bits,
            "" if framing.stop_bits == 1 else "s",
        ),
        ";",
        ".program uart_tx",
        ".side_set 1 opt",
        "",
        f".define BIT_DELAY {BIT_DELAY}                ; a bit is 1 + {BIT_DELAY} = "
        f"{SM_CYCLES_PER_BIT} SM cycles",
        "",
        "PUBLIC start:",
        "        pull block               ; OSR <- the next word from the TX FIFO, which",
        "                                   ; holds this framing's data bits and its parity bit",
        f"        set pindirs, 1 side 2 [BIT_DELAY]    ; the TX pin becomes an output and the "
        "side-set",
        "                                   ; field drives it low: the start bit",
    ]
    lines += data + stops + ["        jmp start                     ; the line stays high (idle) "
                             "until the next byte",
                             ""]
    return "\n".join(lines)


def program_for(framing: Framing = DEFAULT, source: str | None = None) -> Program:
    """Assemble the transmitter for `framing` and check the shape the driver relies on.

    The checks are the ones `fw/uart_tx/driver.py` cannot derive for itself: `start` has to be
    word 0 (the host forces `jmp start` through `SMe_INSTR`, which is the only way to write the
    PC), the wrap window has to end on the last instruction, and the whole program has to fit the
    32-word IMEM.
    """
    prog = assemble(source if source is not None else program_source(framing))[0]
    if len(prog.instrs) > 32:
        raise ValueError(f"{prog.name} for {framing} is {len(prog.instrs)} words, IMEM holds 32")
    if prog.labels["start"] != 0:
        raise ValueError(f"{prog.name}: `start` is word {prog.labels['start']}, the host forces "
                         "`jmp start`")
    if prog.wrap != len(prog.instrs) - 1:
        raise ValueError(f"{prog.name}: WRAP_TOP is {prog.wrap}, the last word is "
                         f"{len(prog.instrs) - 1}")
    if prog.sideset_bits != 2 or not prog.sideset_opt:
        raise ValueError(f"{prog.name}: `.side_set 1 opt` is 2 side-set bits, one enable and one "
                         "data bit, or the start bit never reaches the wire")
    return prog
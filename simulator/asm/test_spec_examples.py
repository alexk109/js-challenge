"""The worked examples the spec itself gives, assembled and checked word for word.

Two of them, both in ``docs/mvp-isa.md``:

* ``### Delay/side-set field, bits 12:8`` has a table of eleven "Worked encodings (pioasm
  output, used as test vectors)".  All eleven are what this assembler emits.  The
  ``pull side 1 [7]`` row is the one that needs the pioasm default (Block set, cleared with
  ``noblock``): the row names no flag word and prints 0x9FA0, whose bit 5 is Block, so the
  line is a blocking pull.  ``test_pull_side_set_worked_encoding`` pins the row and the keyword
  that gives the non-blocking reading the line looks like.
* ``### Anchor trace (normative example)`` gives three instructions and the register values
  around them, which is the one complete program in the spec.
"""

import pytest

from asm import assemble


def asm_program(text: str):
    programs = assemble(text)
    assert len(programs) == 1
    return programs[0]


def one(line: str, directives=()):
    src = ".program p\n" + "".join(f"{d}\n" for d in directives) + f"{line}\n"
    return assemble(src)[0].instrs[0]


# --- `### Delay/side-set field, bits 12:8`, "Worked encodings (pioasm output, used as test
# vectors)".  Every word is re-derived from `## Encoding` in the comment.
WORKED = [
    # | `nop` (= `mov y, y`) | | `0xA042` |  -> 101 00000 000 0010
    ("nop", 0xA042),
    # | `jmp 0` | | `0x0000` |  -> 000 00000 000 00000
    ("jmp 0", 0x0000),
    # | `set pindirs, 1` | | `0xE081` |  -> 111 00000 100 00001
    ("set pindirs, 1", 0xE081),
    # | `set pins, 1 [1]` | | `0xE101` |  -> 111 00001 000 00001
    ("set pins, 1 [1]", 0xE101),
    # | `pull block` | | `0x80A0` |  -> 100 00000 101 00000
    ("pull block", 0x80A0),
    # | `out pins, 1` | | `0x6001` |  -> 011 00000 000 00001
    ("out pins, 1", 0x6001),
    # | `in pins, 1` | | `0x4001` |  -> 010 00000 000 00001
    ("in pins, 1", 0x4001),
    # | `wait 0 pin 0` | | `0x2020` |  -> 001 00000 0 01 00000
    ("wait 0 pin 0", 0x2020),
    # | `jmp x-- 3 [6]` | | `0x0643` |  -> 000 00110 010 00011
    ("jmp x-- 3 [6]", 0x0643),
    # | `mov pins, x` | | `0xA001` |  -> 101 00000 00 001
    ("mov pins, x", 0xA001),
]


@pytest.mark.parametrize("line,expected", WORKED, ids=[w[0] for w in WORKED])
def test_worked_encodings(line, expected):
    assert one(line) == expected


def test_pull_side_set_worked_encoding():
    # `.side_set 1 opt` is SC = 2: the top two bits of 12:8 are side-set and the low three
    # are the delay.  `side 1` sets the enable bit and the one data bit, so the field is
    # 11 111 = 31, and a bare `pull` blocks (pioasm's default) with IfEmpty clear: bit 7 and
    # bit 5 set, 6:4 clear, so 101 00000 = 0x80A0 over the side-set and delay.
    assert one("pull side 1 [7]", [".side_set 1 opt"]) == 0x9FA0
    assert one("pull side 1 [7]", [".side_set 1 opt"]) == 0x80A0 | (0b11111 << 8)
    # The spec's table prints 0x9FA0 for this line and names no flag word for it, so the two
    # now agree: 0x9FA0 has IfEmpty clear and Block set.
    assert 0x9FA0 & 0x60 == 0x20  # IfEmpty clear, Block set
    assert one("pull block side 1 [7]", [".side_set 1 opt"]) == 0x9FA0
    # `noblock` is the spelling for the non-blocking pull the line looks like.
    assert one("pull noblock side 1 [7]", [".side_set 1 opt"]) == 0x9F80


def test_anchor_trace_program():
    # `### Anchor trace (normative example)`: IMEM 0: E081 set pindirs,1 / 1: E001 set pins,1
    # / 2: E000 set pins,0, with WRAP_BOTTOM 1 and WRAP_TOP 2.
    program = asm_program(
        """
        .program anchor
        set pindirs, 1
        set pins, 1
        set pins, 0
        """
    )
    assert [f"0x{word:04X}" for word in program.instrs] == ["0xE081", "0xE001", "0xE000"]
    assert program.labels == {}


def test_anchor_trace_with_its_wrap_window():
    # The same three instructions with the wrap window the trace runs in: .wrap_target 1 and
    # .wrap 2, so the engine counts 1 -> 2 -> 1 as the trace's table says.
    program = asm_program(
        """
        .program anchor
        .wrap_target 1
        set pindirs, 1
        .wrap 2
        set pins, 1
        set pins, 0
        """
    )
    assert [f"0x{word:04X}" for word in program.instrs] == ["0xE081", "0xE001", "0xE000"]
    assert (program.wrap_target, program.wrap) == (1, 2)


# --- the RP2040 example programs the MVP ports.  These are not in the spec as source, only
# as a list in `## Differences from RP2040` item 19, so here they are only assembled to
# check that the instruction forms those programs use all work.
UART_TX = """
.program uart_tx
.side_set 1 opt
    out pins, 1   side 0 [3]   ; the start bit
    out pins, 1   side 1 [1]   ; one data bit, in the frame
    jmp send       side 1 [1]
send:
    in pins, 1                ; the stop bit back from the pin
.wrap send
"""


def test_uart_tx_shaped_program_assembles():
    program = asm_program(UART_TX)
    assert len(program.instrs) == 4
    assert program.wrap == 3  # `send:` is the fourth instruction
    assert program.sideset_bits == 2 and program.sideset_opt
    # `out pins, 1 side 0 [3]`: SC = 2, `side 0` clears the enable bit, so the field is the
    # delay alone, 00 011 = 3: 0x6000 | (3 << 8) | 0x0001.
    assert program.instrs[0] == 0x6301


SPI = """
.program spi
.side_set 1
loop:
    out pindirs, 1 side 1
    set pins, 0    side 0
    out pins, 8    side 0
    in pins, 8     side 0
    push iffull
.wrap_target loop
    jmp loop !x    side 0 [3]
"""


def test_spi_shaped_program_assembles():
    program = asm_program(SPI)
    assert len(program.instrs) == 6
    assert program.labels == {"loop": 0}
    assert (program.wrap_target, program.wrap) == (0, 5)  # .wrap defaults to the last word
    assert program.sideset_bits == 1 and not program.sideset_opt



def test_a_program_that_the_mvp_cannot_assemble_is_the_documented_one():
    # i2c.pio and uart_rx.pio need the edits of `## Differences from RP2040` item 19; the
    # assembler refuses the two instruction forms they use (`irq wait 0 rel`, `irq 4 rel`).
    with pytest.raises(Exception, match="IRQ"):
        assemble(".program i2c\nirq wait 0 rel\n")

"""Every instruction and every operand, checked bit for bit against ``docs/mvp-isa.md``.

The expected words here are computed from the spec's own tables, transcribed below and
nowhere else, so a test failure means the assembler disagrees with the spec (or the spec
disagrees with itself), not with another copy of the same encoder.

``## Encoding``::

    | Bits | 15:13 | 12:8 | 7 | 6 | 5 | 4:0 |
    | JMP  | 000   | d/s  | Condition[2:0] (7:5) | | Address[4:0] |
    | WAIT | 001   | d/s  | Pol | Source[1:0] (6:5) | | Index[4:0] |
    | IN   | 010   | d/s  | Source[2:0] (7:5) | | Bit count[4:0] |
    | OUT  | 011   | d/s  | Destination[2:0] (7:5) | | Bit count[4:0] |
    | PUSH | 100   | d/s  | 0 | IfFull | Block | 00000 |
    | PULL | 100   | d/s  | 1 | IfEmpty | Block | 00000 |
    | MOV  | 101   | d/s  | Destination[2:0] (7:5) | | Op[1:0] (4:3), Source[2:0] (2:0) |
    | SET  | 111   | d/s  | Destination[2:0] (7:5) | | Data[4:0] |
"""

import pytest

from asm import AsmError, assemble

# `## Encoding`, column 15:13
OP_JMP, OP_WAIT, OP_IN, OP_OUT = 0b000, 0b001, 0b010, 0b011
OP_SHIFT, OP_MOV, OP_SET = 0b100, 0b101, 0b111  # 0b110 is IRQ, removed in the MVP


def word(opcode: int, delay_side: int = 0, bits7_5: int = 0, bits4_0: int = 0) -> int:
    """The 16-bit word of `## Encoding`, one argument per column."""
    return (opcode << 13) | (delay_side << 8) | (bits7_5 << 5) | bits4_0


def one(line: str, directives=()) -> int:
    """Assemble a single instruction line inside a one-instruction program."""
    src = ".program p\n" + "".join(f"{d}\n" for d in directives) + f"{line}\n"
    words = assemble(src)[0].instrs
    assert len(words) == 1, f"{line!r} did not assemble to one word"
    return words[0]


# --- JMP: Condition (7:5), `### Field values`: 000 always, 001 !X, 010 X--, 011 !Y,
# 100 Y--, 101 X!=Y, 110 PIN, 111 !OSRE; Address (4:0) absolute.
JMP_CONDS = [
    ("", 0b000), ("!x", 0b001), ("x--", 0b010), ("!y", 0b011),
    ("y--", 0b100), ("x!=y", 0b101), ("pin", 0b110), ("!osre", 0b111),
]


@pytest.mark.parametrize("cond,code", JMP_CONDS)
def test_jmp_conditions(cond, code):
    line = f"jmp 17{(' ' + cond) if cond else ''}"
    assert one(line) == word(OP_JMP, 0, code, 17)


@pytest.mark.parametrize("addr", [0, 1, 5, 16, 30, 31])
def test_jmp_addresses(addr):
    assert one(f"jmp {addr}") == word(OP_JMP, 0, 0b000, addr)


def test_jmp_address_is_five_bits():
    assert one("jmp 31") == 0x001F
    with pytest.raises(AsmError, match="jmp target 32 does not fit in 5 bits"):
        one("jmp 32")


# --- WAIT: Pol (7), Source (6:5) `00` GPIO, `01` PIN; Index (4:0).
@pytest.mark.parametrize("pol", [0, 1])
@pytest.mark.parametrize("src,code", [("gpio", 0b00), ("pin", 0b01)])
def test_wait(pol, src, code):
    assert one(f"wait {pol} {src} 9") == word(OP_WAIT, 0, (pol << 2) | code, 9)
    assert one(f"wait {pol} {src} 9") & 0xE000 == OP_WAIT << 13  # 15:13 is 001
    assert one(f"wait {pol} {src} 31") & 0x1F == 31


def test_wait_polarity_must_be_a_bit():
    with pytest.raises(AsmError, match="polarity must be 0 or 1"):
        one("wait 2 pin 0")


# --- IN: Source (7:5) 000 PINS, 001 X, 010 Y, 011 NULL, 110 ISR, 111 OSR (100/101 reserved).
IN_SOURCES = [("pins", 0b000), ("x", 0b001), ("y", 0b010), ("null", 0b011),
              ("isr", 0b110), ("osr", 0b111)]


@pytest.mark.parametrize("src,code", IN_SOURCES)
def test_in_sources(src, code):
    assert one(f"in {src} 8") == word(OP_IN, 0, code, 8)


# --- OUT: Destination (7:5) 000 PINS, 001 X, 010 Y, 011 NULL, 100 PINDIRS, 101 PC, 110 ISR,
# 111 EXEC.
OUT_DESTS = [("pins", 0b000), ("x", 0b001), ("y", 0b010), ("null", 0b011), ("pindirs", 0b100),
             ("pc", 0b101), ("isr", 0b110), ("exec", 0b111)]


@pytest.mark.parametrize("dest,code", OUT_DESTS)
def test_out_destinations(dest, code):
    assert one(f"out {dest} 8") == word(OP_OUT, 0, code, 8)


# --- bit counts: 1..31 encode themselves, 32 is 00000 (`## Encoding`).
@pytest.mark.parametrize("count", [1, 2, 7, 16, 31])
def test_bit_counts(count):
    assert one(f"in pins {count}") & 0x1F == count
    assert one(f"out pins {count}") & 0x1F == count


def test_bit_count_32_is_the_zero_field():
    assert one("in pins 32") & 0x1F == 0
    assert one("out pins 32") & 0x1F == 0
    assert one("in pins 32") == word(OP_IN, 0, 0b000, 0)


@pytest.mark.parametrize("line", ["in pins 0", "in pins 33", "out pins 0", "out pins 33"])
def test_bit_count_out_of_range(line):
    with pytest.raises(AsmError, match="bit count .* out of range: a bit count is 1..32"):
        one(line)


# --- PUSH/PULL: bit 7 = 0/1, bit 6 = IfFull/IfEmpty, bit 5 = Block, bits 4:0 = 0.  Block is
# the default (pioasm sec. 3.4.6/3.4.7), so `noblock` is what clears it and the flag order
# does not matter.
@pytest.mark.parametrize(
    "line,flags",
    [
        ("push", 0b001), ("push block", 0b001), ("push noblock", 0b000),
        ("push iffull", 0b011), ("push noblock iffull", 0b010), ("push iffull noblock", 0b010),
        ("push iffull block", 0b011), ("push block iffull", 0b011),
        ("pull", 0b101), ("pull block", 0b101), ("pull noblock", 0b100),
        ("pull ifempty", 0b111), ("pull noblock ifempty", 0b110), ("pull ifempty noblock", 0b110),
        ("pull ifempty block", 0b111), ("pull block ifempty", 0b111),
    ],
)
def test_push_pull_flags(line, flags):
    assert one(line) == word(OP_SHIFT, 0, flags, 0)


def test_pull_block_matches_the_specs_worked_encoding():
    assert one("pull block") == 0x80A0


def test_bare_pull_is_a_pull_block_and_noblock_is_the_only_way_to_clear_block():
    # The spec's worked-encoding table prints 0x9FA0 for `pull side 1 [7]` with no flag word,
    # and 0x9FA0 has Block set, so the default has to be Block set (review finding F1, see
    # asm/test_spec_examples.py).
    assert one("pull") == one("pull block") == 0x80A0
    assert one("pull noblock") == 0x8080
    assert one("push") == one("push block") == 0x8020
    assert one("push noblock") == 0x8000


# --- MOV: Destination (7:5) 000 PINS, 001 X, 010 Y, 100 EXEC, 101 PC, 110 ISR, 111 OSR;
# Op (4:3) 00 none, 01 invert, 10 reverse; Source (2:0) 000 PINS, 001 X, 010 Y, 011 NULL,
# 101 STATUS, 110 ISR, 111 OSR.
MOV_DESTS = [("pins", 0b000), ("x", 0b001), ("y", 0b010), ("exec", 0b100), ("pc", 0b101),
             ("isr", 0b110), ("osr", 0b111)]
MOV_SRCS = [("pins", 0b000), ("x", 0b001), ("y", 0b010), ("null", 0b011), ("status", 0b101),
            ("isr", 0b110), ("osr", 0b111)]
MOV_OPS = [("", 0b00), ("!", 0b01), ("r", 0b10)]


@pytest.mark.parametrize("dest,dcode", MOV_DESTS)
@pytest.mark.parametrize("src,scode", MOV_SRCS)
def test_mov_destination_source(dest, dcode, src, scode):
    assert one(f"mov {dest}, {src}") == word(OP_MOV, 0, dcode, scode)


@pytest.mark.parametrize("op,ocode", MOV_OPS)
def test_mov_ops(op, ocode):
    line = f"mov x, y{', ' + op if op else ''}"
    assert one(line) == word(OP_MOV, 0, 0b001, (ocode << 3) | 0b010)


def test_nop_is_mov_y_y():
    assert one("nop") == 0xA042 == word(OP_MOV, 0, 0b010, 0b010)


# --- SET: Destination (7:5) 000 PINS, 001 X, 010 Y, 100 PINDIRS; Data (4:0).
SET_DESTS = [("pins", 0b000), ("x", 0b001), ("y", 0b010), ("pindirs", 0b100)]


@pytest.mark.parametrize("dest,code", SET_DESTS)
def test_set_destinations(dest, code):
    assert one(f"set {dest} 1") == word(OP_SET, 0, code, 1)


@pytest.mark.parametrize("data", [0, 1, 15, 16, 30, 31])
def test_set_data(data):
    assert one(f"set pins {data}") == word(OP_SET, 0, 0b000, data)


def test_set_data_is_five_bits():
    with pytest.raises(AsmError, match="set data 32 does not fit in 5 bits"):
        one("set pins 32")
    with pytest.raises(AsmError, match="set data -1 does not fit in 5 bits"):
        one("set pins -1")


def test_every_word_is_16_bits():
    for line in ["nop", "jmp 31 !osre", "wait 1 pin 31", "in osr 32", "out exec 1", "push",
                 "pull ifempty block", "mov osr, status r", "set pindirs 31"]:
        assert 0 <= one(line) <= 0xFFFF

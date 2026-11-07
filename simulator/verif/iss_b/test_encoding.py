"""The spec's own encoding table: ISA `### Delay/side-set field` "Worked encodings".

Every hand-encoder in `enc.py` is checked here against the word the spec gives, so
a wrong bit position in a test program fails loudly instead of silently testing
the wrong instruction.
"""

from verif.iss_b import enc
from verif.iss_b.enc import jmp, mov, out, pull, push, set_, wait_, in_

# (assembly, config, word) -- verbatim from docs/mvp-isa.md
WORKED = [
    ("nop", {}, 0xA042),
    ("jmp 0", {}, 0x0000),
    ("set pindirs, 1", {}, 0xE081),
    ("set pins, 1 [1]", {}, 0xE101),
    ("pull block", {}, 0x80A0),
    # .side_set 1 opt: SC = 2, so the field is the 2 side-set bits 11 then delay 111.
    ("pull side 1 [7]", {"sideset_count": 2, "side_en": 1}, 0x9FA0),
    ("out pins, 1", {}, 0x6001),
    ("in pins, 1", {}, 0x4001),
    ("wait 0 pin 0", {}, 0x2020),
    ("jmp x-- 3 [6]", {}, 0x0643),
    ("mov pins, x", {}, 0xA001),
]


def test_worked_encodings():
    for asm, _cfg, expect in WORKED:
        got = {
            "nop": enc.NOP,
            "jmp 0": jmp(enc.JMP_ALWAYS, 0),
            "set pindirs, 1": set_(enc.SET_PINDIRS, 1),
            "set pins, 1 [1]": set_(enc.SET_PINS, 1, delay=1),
            "pull block": pull(block=1),
            "pull side 1 [7]": pull(block=1, delay=7, side=0b11, sc=2),
            "out pins, 1": out(enc.OUT_PINS, 1),
            "in pins, 1": in_(enc.IN_PINS, 1),
            "wait 0 pin 0": wait_(enc.WAIT_PIN, 0, pol=0),
            "jmp x-- 3 [6]": jmp(enc.JMP_DECX, 3, delay=6),
            "mov pins, x": mov(enc.MOV_PINS, enc.MOVSRC_X),
        }[asm]
        assert got == expect, f"{asm}: encoded {got:#06x}, spec says {expect:#06x}"


def test_nop_is_mov_y_y():
    assert enc.NOP == mov(enc.MOV_Y, enc.MOVSRC_Y)


def test_opcode_field_positions():
    # 15:13 selects the instruction; PUSH and PULL share 100 and differ in bit 7.
    for name, op in enc.OPCODE.items():
        if name == "pull":
            continue
        assert (enc.word(op) >> 13) == op
    assert (enc.word(enc.PULL) >> 13) == enc.PUSH
    assert enc.pull() & 0x80 and not enc.push() & 0x80


def test_bit_count_zero_means_32():
    """ISA `## Encoding`: a bit count of `00000` encodes 32, so it is the same word."""
    assert enc.in_(enc.IN_X, 0) == enc.in_(enc.IN_X, 32 & 0x1F)
    assert enc.out(enc.OUT_X, 0) == enc.out(enc.OUT_X, 32 & 0x1F)


def test_push_if_full_block_bits():
    """PUSH: bit 6 IfFull, bit 5 Block; bits 4:0 are documented 0 (ignored)."""
    assert enc.push() == 0x8000
    assert enc.push(iffull=1) == 0x8040
    assert enc.push(block=1) == 0x8020
    assert enc.push(iffull=1, block=1) == 0x8060
    assert enc.pull(ifempty=1, block=1) == 0x80E0
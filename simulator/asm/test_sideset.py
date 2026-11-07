"""Side-set and delay: how the 5-bit field at 12:8 splits, per the spec.

``### Delay/side-set field, bits 12:8``: ``SC`` is the side-set count; the **top** ``SC``
bits of the field are side-set and the low ``5 - SC`` bits are the delay, so the delay is
``0 .. 2**(5-SC) - 1``.  With ``SIDE_EN`` (``opt``) the MSB of the side-set bits is the
enable and the rest is data, and side-set happens only when the enable bit is 1.

The expected words below are built from that sentence with a helper written here, not from
the assembler.
"""

import pytest

from asm import AsmError, assemble

NOP = 0xA042  # mov y, y: opcode 101, dest 010, op 00, src 010


def field(side: int, delay: int, sc: int) -> int:
    """Bits 12:8: side-set on top, delay below."""
    assert 0 <= delay <= (1 << (5 - sc)) - 1
    return (side << (5 - sc)) | delay


def side_bits(value: int, sc: int, opt: bool) -> int:
    """The ``SC`` side-set bits: the value, with the opt (enable) bit on top if set."""
    assert 0 <= value < (1 << sc)
    return value | ((1 << (sc - 1)) if (opt and value) else 0)


def one(line: str, sideset: str = "") -> int:
    src = ".program p\n" + (f"{sideset}\n" if sideset else "") + f"{line}\n"
    return assemble(src)[0].instrs[0]


# --- the field really is [side-set | delay], side-set on top.
@pytest.mark.parametrize(
    "sideset,sc,side,delay",
    [
        (".side_set 1", 1, 1, 0),          # 1 0000
        (".side_set 1", 1, 1, 15),         # 1 1111
        (".side_set 2", 2, 2, 7),          # 10 111
        (".side_set 3", 3, 4, 3),          # 100 11
        (".side_set 4", 4, 8, 1),          # 1000 1
        (".side_set 5", 5, 16, 0),         # 10000, no delay room left
        (".side_set 1 opt", 2, 0b11, 7),   # opt bit + one data bit
        (".side_set 2 opt", 3, 0b111, 3),
        (".side_set 0 opt", 1, 0b1, 0),    # only the enable bit
    ],
)
def test_side_set_occupies_the_top_of_the_field(sideset, sc, side, delay):
    assert one(f"nop side {side} [{delay}]", sideset) == NOP | (field(side, delay, sc) << 8)


@pytest.mark.parametrize("sideset,sc,max_delay", [("", 0, 31), (".side_set 1", 1, 15),
                                                   (".side_set 2", 2, 7), (".side_set 3", 3, 3),
                                                   (".side_set 4", 4, 1), (".side_set 5", 5, 0)])
def test_maximum_delay_is_two_to_the_five_minus_sc(sideset, sc, max_delay):
    assert one(f"nop [{max_delay}]", sideset) == NOP | (field(0, max_delay, sc) << 8)
    with pytest.raises(AsmError, match=f"delay {max_delay + 1} does not fit"):
        one(f"nop [{max_delay + 1}]", sideset)


# --- opt: the topmost side-set bit is the enable, so a non-zero value sets it.
@pytest.mark.parametrize("value,bits", [(0, 0b00), (1, 0b11)])
def test_side_set_opt_enable(value, bits):
    assert one(f"nop side {value}", ".side_set 1 opt") == NOP | (field(bits, 0, 2) << 8)


def test_no_side_operand_leaves_the_field_clear():
    assert one("nop", ".side_set 2 opt") == NOP
    assert one("nop", ".side_set 3 pindirs") == NOP


def test_side_zero_with_opt_is_a_nop_side_set():
    # The enable bit is 0, so the side-set does not happen: same word as no side operand.
    assert one("nop side 0", ".side_set 2 opt") == one("nop", ".side_set 2 opt")


# --- the settings the Program carries.
def test_program_reports_the_side_set_setting():
    prog = assemble(".program p\n.side_set 2 opt pindirs\nnop\n")[0]
    assert (prog.sideset_bits, prog.sideset_opt, prog.sideset_pindirs) == (3, True, True)


@pytest.mark.parametrize(
    "directive,bits,opt,pindirs",
    [
        (".side_set 0", 0, False, False),
        (".side_set 1", 1, False, False),
        (".side_set 4", 4, False, False),
        (".side_set 1 pindirs", 1, False, True),
        (".side_set 4 opt", 5, True, False),
        (".side_set 1 opt pindirs", 2, True, True),
        (".side_set 3 pindirs opt", 4, True, True),  # flag order does not matter
    ],
)
def test_side_set_directive(directive, bits, opt, pindirs):
    prog = assemble(f".program p\n{directive}\nnop\n")[0]
    assert (prog.sideset_bits, prog.sideset_opt, prog.sideset_pindirs) == (bits, opt, pindirs)


def test_pindirs_does_not_change_the_word():
    assert one("nop side 1", ".side_set 1") == one("nop side 1", ".side_set 1 pindirs")


# --- limits
def test_side_value_must_fit_the_side_set_bits():
    with pytest.raises(AsmError, match="side-set value 2 does not fit in 1 side-set field bit"):
        one("nop side 2", ".side_set 1")
    assert one("nop side 1", ".side_set 1") == NOP | (field(1, 0, 1) << 8)


def test_side_value_fits_the_whole_field_with_opt():
    # With opt the value is checked against SC = data + 1, so `side 3` with `.side_set 1 opt`
    # puts 3 in the enable bit and 2 in the data bit.
    assert one("nop side 3", ".side_set 1 opt") == NOP | (field(0b111, 0, 2) << 8)


def test_side_needs_a_side_set_directive():
    with pytest.raises(AsmError, match="'side' operand but the program has no .side_set"):
        one("nop side 1")
    with pytest.raises(AsmError, match="'side' operand but the program has no .side_set"):
        one("nop side 0", ".side_set 0")


@pytest.mark.parametrize("directive", [".side_set 6", ".side_set 5 opt", ".side_set 7 opt"])
def test_side_set_wider_than_the_field(directive):
    with pytest.raises(AsmError, match="5-bit delay/side-set field"):
        assemble(f".program p\n{directive}\nnop\n")


# --- every instruction takes the same side and delay tail
@pytest.mark.parametrize(
    "line", ["nop", "jmp 3 !x", "wait 1 gpio 4", "in pins 1", "out pindirs 1", "push block",
             "pull ifempty", "mov x, status r", "set pins 3"],
)
def test_tail_on_every_instruction(line):
    plain = one(line)
    assert one(f"{line} side 1", ".side_set 1") == plain | (0b10000 << 8)
    assert one(f"{line} [2]") == plain | (2 << 8)
    assert one(f"{line} side 1 [2]", ".side_set 1") == plain | (0b10010 << 8)


def test_specs_side_set_example():
    # The worked encoding `pull side 1 [7]` with `.side_set 1 opt`: SC = 2, the top two
    # bits of the field are side-set (enable 1, data 1) and the low three are delay 7.  A bare
    # `pull` blocks, so the flag word is PULL with Block set and IfEmpty clear: 101 00000.
    assert one("pull side 1 [7]", ".side_set 1 opt") == 0x9FA0
    assert 0x9FA0 == 0x80A0 | (field(side_bits(1, 2, True), 7, 2) << 8)

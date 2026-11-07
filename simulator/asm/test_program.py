"""Programs, directives, labels and defines: everything `## Assembler API` promises."""

import pytest

from asm import AsmError, Program, assemble

NOP = 0xA042
SET_ONE = 0xE081  # set pindirs, 1


def prog(text: str) -> Program:
    programs = assemble(text)
    assert len(programs) == 1
    return programs[0]


# --- one Program per .program, in file order
def test_one_program_per_directive():
    programs = assemble(
        """
        .program first
        nop
        .program second
        set pindirs, 1
        nop
        """
    )
    assert [p.name for p in programs] == ["first", "second"]
    assert [p.instrs for p in programs] == [[NOP], [SET_ONE, NOP]]


def test_empty_program():
    assert prog(".program empty\n").instrs == []
    assert prog(".program empty\n").wrap == 0  # no last instruction to point at
    assert prog(".program empty\n").wrap_target == 0
    assert prog(".program empty\n").origin is None


def test_no_programs_at_all():
    assert assemble("; nothing but a comment\n") == []


def test_defaults_without_directives():
    program = prog(".program p\nnop\nnop\nnop\n")
    assert program.origin is None
    assert program.wrap_target == 0
    assert program.wrap == len(program.instrs) - 1 == 2
    assert program.labels == {}
    assert (program.sideset_bits, program.sideset_opt) == (0, False)
    assert program.sideset_pindirs is False


# --- labels
def test_labels_are_instruction_indices():
    program = prog(".program p\nstart:\nnop\nmiddle:\nset pindirs, 1\nafter:\nnop\n")
    assert program.labels == {"start": 0, "middle": 1, "after": 2}


def test_label_on_its_own_line_and_on_the_instruction_line():
    program = prog(".program p\nstart:\nnop\nloop: jmp loop\n")
    assert program.labels == {"start": 0, "loop": 1}
    assert program.instrs == [NOP, 0x0001]


def test_several_labels_on_one_line():
    program = prog(".program p\na: b: c: nop\n")
    assert program.labels == {"a": 0, "b": 0, "c": 0}


def test_public_label():
    program = prog(".program p\nPUBLIC entry:\nnop\n")
    assert program.labels == {"entry": 0}


def test_labels_can_be_used_before_they_are_defined():
    program = prog(".program p\njmp end\nnop\nend:\nnop\n")
    assert program.labels == {"end": 2}
    assert program.instrs[0] == 0x0002  # jmp always, address 2


def test_labels_are_usable_in_expressions():
    # An expression with a space in it needs parentheses; an operand is otherwise one token.
    program = prog(".program p\nloop:\nnop\nnop\njmp (loop + 1)\n")
    assert program.instrs[2] == 0x0001


# --- .origin: labels, wrap and wrap_target are absolute addresses
def test_origin_offsets_the_labels():
    program = prog(".program p\n.origin 4\nstart:\nnop\njmp start\n")
    assert program.origin == 4
    assert program.labels == {"start": 4}
    assert program.instrs == [NOP, 0x0004]  # jmp 4


def test_origin_and_wrap_together():
    program = prog(
        ".program p\n.origin 8\n.wrap_target start\n.wrap bottom\nstart:\nnop\nbottom:\nnop\n"
    )
    assert program.origin == 8
    assert program.wrap_target == 8
    assert program.wrap == 9


def test_origin_may_be_an_expression():
    assert prog(".program p\n.define BASE 4\n.origin BASE + 1\nnop\n").origin == 5


# --- .wrap and .wrap_target
def test_wrap_with_a_label():
    program = prog(".program p\n.wrap_target top\n.wrap bottom\nnop\ntop:\nnop\nbottom:\nnop\n")
    assert program.wrap_target == 1
    assert program.wrap == 2


def test_wrap_with_a_number():
    program = prog(".program p\n.wrap_target 3\n.wrap 0x1f\nnop\nnop\nnop\n")
    assert (program.wrap_target, program.wrap) == (3, 31)


def test_bare_wrap_points_at_the_next_instruction():
    program = prog(".program p\nnop\n.wrap\nnop\n.wrap_target\nnop\n")
    assert program.wrap == 1
    assert program.wrap_target == 2


def test_wrap_outside_imem():
    with pytest.raises(AsmError, match="wrap address 32 is outside the instruction memory"):
        prog(".program p\n.wrap 32\nnop\n")


# --- .define
def test_define_is_usable_as_an_operand():
    program = prog(".program p\n.define N 3\n.define DATA (N * 2 + 1)\nset pins, DATA\n")
    assert program.instrs == [0xE007]  # SET PINS, 7


def test_public_define_is_visible_in_another_program():
    with pytest.raises(AsmError, match="undefined symbol 'LOCAL'"):
        assemble(
            """
            .program one
            .define PUBLIC WIDTH 4
            .define LOCAL 1
            in pins, WIDTH
            .program two
            in pins, LOCAL
            """
        )
    programs = assemble(
        """
        .program one
        .define PUBLIC WIDTH 4
        .define LOCAL 1
        in pins, WIDTH
        .program two
        in pins, WIDTH
        """
    )
    assert [p.instrs for p in programs] == [[0x4004], [0x4004]]


def test_public_define_before_the_first_program():
    programs = assemble(".define PUBLIC SHARED 2\n.program p\nset pins, SHARED\n")
    assert programs[0].instrs == [0xE002]


def test_define_before_any_program_must_be_public():
    with pytest.raises(AsmError, match=".define before the first .program must be PUBLIC"):
        assemble(".define N 1\n.program p\nnop\n")


def test_define_defined_twice():
    with pytest.raises(AsmError, match="symbol 'N' is defined twice"):
        prog(".program p\n.define N 1\n.define N 2\nnop\n")


def test_define_referring_to_a_later_define():
    assert prog(".program p\n.define A B + 1\n.define B 4\nset pins, A\n").instrs == [0xE005]


# --- comments and numbers
def test_repeated_directives_take_the_last_one():
    # Documented: .side_set is one setting per program, and so is .wrap.
    program = prog(".program p\n.side_set 2\nnop\n.side_set 1 opt\nnop\n.wrap 1\n.wrap 0\n")
    assert (program.sideset_bits, program.sideset_opt) == (2, True)
    assert program.instrs == [NOP, NOP]  # the second .side_set applies to both
    assert program.wrap == 0


def test_program_name_errors():
    with pytest.raises(AsmError, match=r"\.program takes one name"):
        prog(".program\n")
    with pytest.raises(AsmError, match=r"\.program takes one name"):
        prog(".program p extra\n")

    program = prog(
        """
        .program p          ; a semicolon comment
        // a C++ comment
        nop                 ; trailing
        set pins, 1 [1]     // another
        ; whole line
        """
    )
    assert program.instrs == [NOP, 0xE101]


def test_semicolon_inside_a_comment_does_not_end_the_line_early():
    assert prog(".program p\nnop ; a ; b ; c\n").instrs == [NOP]


@pytest.mark.parametrize(
    "text,expected", [("16", 0xE010), ("0x10", 0xE010), ("0b10000", 0xE010),
                      ("0b1::0000", 0xE010), ("0b1::0::0::0", 0xE008), ("1<<4", 0xE010),
                      ("((1 + 1) * 8)", 0xE010), ("32/2", 0xE010), ("33-17", 0xE010),
                      ("(0xff & 0x1f)", 0xE01F), ("(~0 & 0x1f)", 0xE01F), ("(0x10 | 0)", 0xE010),
                      ("0x10%16", 0xE000), ("(2*8)", 0xE010)]
)
def test_number_and_expression_spellings(text, expected):
    assert prog(f".program p\nset pins, {text}\n").instrs == [expected]


def test_uppercase_hex_and_binary_prefixes():
    assert prog(".program p\nset pins, 0X1F\n").instrs == [0xE01F]
    assert prog(".program p\nset pins, 0B101\n").instrs == [0xE005]


def test_commas_are_optional():
    # mov x, y is opcode 101, dest 001, op 00, src 010.
    assert prog(".program p\nset pins 1\nmov x y\nin pins 1\n").instrs == [0xE001, 0xA022, 0x4001]


def test_tabs_and_extra_spaces():
    assert prog(".program p\n\tset   pins,\t1\t\n").instrs == [0xE001]

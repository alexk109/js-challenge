"""Bad input: every rejection carries the line it happened on.

The MVP removals of ``## Removed and reserved encodings`` come first, because they are the
ones firmware ported from the RP2040 examples will hit.
"""

import pytest

from asm import AsmError, assemble


def err(src: str):
    """Assemble a whole source and return the ``AsmError`` it must raise."""
    with pytest.raises(AsmError) as excinfo:
        assemble(src)
    return excinfo.value


def err1(line: str, directives=()):
    """Assemble one instruction line, after ``.program`` and ``directives``, and return its error.
    """
    src = ".program p\n" + "".join(f"{d}\n" for d in directives) + f"  {line}\n"
    return err(src)


# --- what the MVP removed
@pytest.mark.parametrize("line", ["irq 0", "irq 1 rel", "irq set 4", "irq nowait 4",
                                  "irq wait 0", "irq clear 1"])
def test_irq_is_rejected(line):
    error = err1(line)
    assert error.line == 2
    assert "IRQ" in error.message and "removed" in error.message
    assert str(error) == f"line 2: {error.message}"


@pytest.mark.parametrize("line", ["wait 1 irq 0", "wait 0 irq 7", "wait 1 IRQ 3"])
def test_wait_irq_is_rejected(line):
    error = err1(line)
    assert error.line == 2
    assert "irq" in error.message.lower()


@pytest.mark.parametrize("line", ["jmp 4 rel", "jmp start rel [2]", "set pins, 1 rel"])
def test_rel_is_rejected(line):
    assert "rel" in err1(line).message


def test_wait_source_11_reserved_has_no_name():
    # WAIT Source 11 is reserved: there is no spelling for it, so nothing assembles to it.
    assert "unknown wait source 'gpios'" in err1("wait 0 gpios 0").message


# --- the reserved field values have no spelling either
@pytest.mark.parametrize(
    "line,fragment",
    [("in 100 1", "unknown in source '100'"), ("in 101 1", "unknown in source '101'"),
     ("out 101 1", "unknown out destination '101'"),
     ("mov 011, x", "unknown mov destination '011'"),
     ("mov x, 100", "unknown mov source '100'"),
     ("set 011 1", "unknown set destination '011'"),
     ("set 110 1", "unknown set destination '110'")],
)
def test_reserved_field_values_are_rejected(line, fragment):
    assert fragment in err1(line).message


@pytest.mark.parametrize("line", ["mov x, y !r", "mov x, y r!", "mov x, y ! r"])
def test_mov_cannot_do_both_ops(line):
    # Op 11 is reserved, so invert and bit-reverse cannot be combined.
    assert "not both" in err1(line).message


# --- missing and extra operands
@pytest.mark.parametrize(
    "line,fragment",
    [
        ("wait 0 pin", "wait needs POLARITY, SOURCE and INDEX"),
        ("in pins", "in needs a SOURCE and a bit COUNT"),
        ("out pins", "out needs a DESTINATION and a bit COUNT"),
        ("mov x", "mov needs a DESTINATION and a SOURCE"),
        ("set pins", "set needs a DESTINATION and DATA"),
        ("jmp", "jmp needs a target"),
        ("jmp 3 !x !y", "only one condition per jmp"),
        ("push iffull iffull", "duplicate IfFull flag"),
        ("pull block block", "duplicate Block flag"),
        ("pull noblock noblock", "duplicate Block flag"),
        ("pull block noblock", "conflicting Block flags"),
        ("nop side", "'side' needs a value"),
        ("nop side 1 side 2", "only one 'side' operand"),
        ("nop [3", "a delay is written [n]"),
        ("nop ]3[", "']' closes a delay"),
        ("frobnicate", "unknown instruction 'frobnicate'"),
        ("mov x, y 3", "unexpected '3'"),
        ("set pins, 1 [1] extra", "unexpected 'extra'"),
    ],
)
def test_operand_errors(line, fragment):
    error = err1(line)
    assert error.line == 2
    assert fragment in error.message


# --- ranges
@pytest.mark.parametrize(
    "line,fragment,directives",
    [
        ("nop [32]", "delay 32 does not fit", ()),
        ("nop [16]", "delay 16 does not fit", (".side_set 1",)),
        ("nop [8]", "delay 8 does not fit", (".side_set 2",)),
        ("nop [4]", "delay 4 does not fit", (".side_set 3",)),
        ("nop [2]", "delay 2 does not fit", (".side_set 4",)),
        ("nop [1]", "delay 1 does not fit", (".side_set 5",)),
        ("nop side 2", "side-set value 2 does not fit", (".side_set 1",)),
        ("nop side 4", "side-set value 4 does not fit", (".side_set 2",)),
        ("nop side 16", "side-set value 16 does not fit", (".side_set 4",)),
        ("nop side 1", "no .side_set directive", ()),
        ("nop side -1", "side-set value -1 does not fit", (".side_set 1",)),
        ("nop [-1]", "delay -1 does not fit", ()),
        ("set pins 32", "set data 32 does not fit in 5 bits", ()),
        ("set pins -1", "set data -1 does not fit in 5 bits", ()),
        ("in pins 0", "a bit count is 1..32", ()),
        ("out pins 33", "a bit count is 1..32", ()),
        ("jmp 32", "jmp target 32 does not fit in 5 bits", ()),
        ("wait 0 pin 32", "wait index 32 does not fit in 5 bits", ()),
        ("wait 3 pin 0", "polarity must be 0 or 1", ()),
    ],
)
def test_range_errors(line, fragment, directives):
    error = err1(line, directives)
    assert error.line == 2 + len(directives)  # .program line, the directives, then the line
    assert fragment in error.message


# --- directives
@pytest.mark.parametrize(
    "src,line,fragment",
    [
        (".program p\n  .origin 32\n", 2, ".origin 32 is outside"),
        (".program p\n  .origin -1\n", 2, ".origin -1 is outside"),
        (".program p\n  .wrap 99\n", 2, "wrap address 99 is outside"),
        (".program p\n  .side_set\n", 2, ".side_set needs a bit count"),
        (".program p\n  .side_set x\n", 2, ".side_set needs a literal bit count"),
        (".program p\n  .side_set 1 opt opt\n", 2, "duplicate 'opt'"),
        (".program p\n  .side_set 1 pindirs pindirs\n", 2, "duplicate 'pindirs'"),
        (".program p\n  .side_set 1 side\n", 2, "unknown .side_set flag"),
        (".program p\n  .side_set 6\n", 2, "5-bit delay/side-set field"),
        (".program p\n  .origin\n", 2, ".origin needs an address"),
        (".program p\n  .word 3\n", 2, "unknown directive '.word'"),
        (".program p\n  .define\n", 2, ".define needs a NAME and a value"),
        (".program p\n  .define 3 1\n", 2, "not a valid symbol name"),
        (".program\n", 1, ".program takes one name"),
        (".program p extra\n", 1, ".program takes one name"),
        (".program 3\n", 1, "not a valid program name"),
        (".program p\n  .side_set 1\n  1bad:\n", 3, "not a valid label name"),
        (".program p\n  PUBLIC\n", 2, "PUBLIC must be followed by a label"),
    ],
)
def test_directive_errors(src, line, fragment):
    error = err(src)
    assert error.line == line
    assert fragment in error.message


# --- symbols
def test_undefined_symbol():
    error = err1("jmp nowhere")
    assert error.line == 2
    assert "undefined symbol 'nowhere'" in error.message


def test_undefined_define_symbol():
    # Resolved even though nothing uses it: a broken .define is an error either way.
    assert "undefined symbol 'MISSING'" in err(".program p\n.define A MISSING\n").message


def test_define_cycle():
    error = err(".program p\n  .define A B + 1\n  .define B A\n  set pins, A\n")
    assert error.line in (2, 3)  # one of the two .define lines the cycle runs through
    assert "cycle" in error.message


def test_label_defined_twice():
    error = err(".program p\n  a:\n  nop\n  a:\n  nop\n")
    assert error.line == 4
    assert "defined twice" in error.message and "line 2" in error.message


def test_program_defined_twice():
    assert "defined twice" in err(".program p\nnop\n.program p\nnop\n").message


def test_label_outside_a_32_word_imem():
    src = ".program p\n.origin 30\n" + "".join("nop\n" for _ in range(3)) + "late:\n"
    assert "does not fit the 5-bit address field" in err(src).message


# --- structure
def test_instruction_outside_a_program():
    error = err("  nop\n")
    assert error.line == 1
    assert "outside any .program" in error.message


def test_directive_outside_a_program():
    assert "outside any .program" in err("  .side_set 1\n").message


@pytest.mark.parametrize("line", ["set pins, (1 + 2", "set pins, 1)", "set pins, @"])
def test_bad_expression(line):
    assert "expression" in err1(line).message


def test_division_by_zero():
    assert "division by zero" in err1("set pins, (1/0)").message
    assert "division by zero" in err1("set pins, (1%0)").message


def test_shift_count_range():
    assert "shift count" in err1("set pins, (1 << -1)").message


def test_asmin_error_without_a_line():
    error = AsmError("something file-wide")
    assert error.line is None
    assert str(error) == "something file-wide"

"""The hand encoders in testkit.py against docs/mvp-isa.md `Worked encodings`."""
import pytest

from iss.testkit import NOP, in_, jmp, mov, out, pull, set_, wait

SIDE_OPT = lambda side, delay: (1 << 4) | (side << 3) | delay  # SC = 2, SIDE_EN = 1


@pytest.mark.parametrize("word,expected", [
    (NOP, 0xA042),                              # nop = mov y, y
    (jmp(0), 0x0000),
    (set_("pindirs", 1), 0xE081),
    (set_("pins", 1, ds=1), 0xE101),
    (pull(block=1), 0x80A0),
    (pull(block=1, ds=SIDE_OPT(1, 7)), 0x9FA0),  # pull side 1 [7] (block), .side_set 1 opt
    (out("pins", 1), 0x6001),
    (in_("pins", 1), 0x4001),
    (wait(0, "pin", 0), 0x2020),
    (jmp(3, "x_dec", ds=6), 0x0643),
    (mov("pins", "x"), 0xA001),
])
def test_worked_encodings(word, expected):
    assert word == expected


def test_bit_count_32_encodes_as_zero():
    assert in_("x", 32) & 0x1F == 0
    assert out("x", 32) & 0x1F == 0

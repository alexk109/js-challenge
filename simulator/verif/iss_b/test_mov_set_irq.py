"""MOV, SET and the removed IRQ opcode (ISA `### Instruction semantics`,
`### Removed and reserved encodings`).
"""

import pytest

from verif.iss_b import enc
from verif.iss_b.enc import execs, in_, irq, jmp, mov, out, poke, set_, setup
from verif.iss_b.enc import IN_X, JMP_ALWAYS, MOV_INVERT, MOV_ISR, MOV_NONE
from verif.iss_b.enc import MOVSRC_STATUS as MOV_STATUS
from verif.iss_b.enc import MOV_OSR, MOV_PC, MOV_PINS, MOV_REVERSE
from verif.iss_b.enc import MOV_X, MOV_Y, MOV_EXEC, MOV_RSVD3, MOVSRC_ISR, MOVSRC_NULL
from verif.iss_b.enc import MOVSRC_OSR, MOVSRC_PINS, MOVSRC_X, MOVSRC_Y
from verif.iss_b.enc import OUT_NULL, SET_PINDIRS, SET_PINS, SET_X, SET_Y

REVERSED = 0x80000001  # bit 0 -> bit 31, bit 31 -> bit 0


# --------------------------------------------------------------------- MOV


def test_mov_copies_the_source_unchanged():
    for src, val, expect in (
        (MOVSRC_X, 0x1234, 0x1234),
        (MOVSRC_Y, 0x5678, 0x5678),
        (MOVSRC_NULL, 0xFFFF, 0),
        (MOVSRC_ISR, 0xABCD, 0xABCD),
        (MOVSRC_OSR, 0xDCBA, 0xDCBA),
    ):
        b = setup(imem=[mov(MOV_X, src)])
        poke(b, 0, x=0x1234, y=0x5678, isr=0xABCD, osr=0xDCBA)
        assert execs(b, 1)[0]["x"] == expect


def test_mov_pins_reads_the_in_rot_vector_unmasked():
    """MOV source PINS is `IN_ROT` (all 32 bits, unmasked), not masked to OUT_COUNT."""
    b = setup(
        imem=[mov(MOV_X, MOVSRC_PINS)],
        pinctrl_v=enc.pinctrl(in_base=0, out_count=1),
        pin=0b1011,
    )
    assert execs(b, 1)[0]["x"] == 0b1011


def test_mov_invert_and_bit_reverse():
    b = setup(imem=[mov(MOV_X, MOVSRC_X, MOV_INVERT)])
    poke(b, 0, x=0x0F0F)
    assert execs(b, 1)[0]["x"] == 0xFFFFF0F0  # a 32-bit NOT

    b = setup(imem=[mov(MOV_X, MOVSRC_X, MOV_REVERSE)])
    poke(b, 0, x=REVERSED)
    assert execs(b, 1)[0]["x"] == REVERSED  # a palindrome, for sanity

    b = setup(imem=[mov(MOV_X, MOVSRC_X, MOV_REVERSE)])
    poke(b, 0, x=0x00000001)
    assert execs(b, 1)[0]["x"] == 0x80000000  # bit 0 -> bit 31


def test_mov_operations_apply_after_the_source_and_before_the_destination():
    b = setup(imem=[mov(MOV_X, MOVSRC_X, MOV_INVERT)])
    poke(b, 0, x=0, y=0xFFFFFFFF)
    assert execs(b, 1)[0]["x"] == 0xFFFFFFFF  # ~Y, not ~X


def test_mov_destinations():
    # PINS: the OUT window, OUT_COUNT pins
    b = setup(
        imem=[mov(MOV_PINS, MOVSRC_X)],
        pinctrl_v=enc.pinctrl(out_base=2, out_count=4),
    )
    poke(b, 0, x=0xF)
    execs(b, 1)
    assert b.chip.pin_out & 0b1111 << 2 == 0b1111 << 2

    # X, Y
    b = setup(imem=[mov(MOV_Y, MOVSRC_X)])
    poke(b, 0, x=0x99)
    assert execs(b, 1)[0]["y"] == 0x99

    # PC: the next PC is v[4:0]
    b = setup(imem=[mov(MOV_PC, MOVSRC_X)])
    poke(b, 0, x=0x15)
    assert execs(b, 1)[0]["pc"] == 0x15

    # ISR: ISR = v and isr_cnt = 0
    b = setup(imem=[mov(MOV_ISR, MOVSRC_X)])
    poke(b, 0, x=0x77, isr=0xFFFF, isr_cnt=12)
    st = execs(b, 1)[0]
    assert st["isr"] == 0x77 and st["isr_cnt"] == 0

    # OSR: OSR = v and osr_cnt = 0
    b = setup(imem=[mov(MOV_OSR, MOVSRC_X)])
    poke(b, 0, x=0x88, osr=0xFFFF, osr_cnt=12)
    st = execs(b, 1)[0]
    assert st["osr"] == 0x88 and st["osr_cnt"] == 0

    # EXEC
    b = setup(imem=[mov(MOV_EXEC, MOVSRC_X)])
    poke(b, 0, x=0xA042)
    st = execs(b, 1)[0]
    assert st["latch"] == 0xA042 and st["latch_pend"] == 1


def test_mov_exec_of_a_nop_runs_on_the_next_sm_cycle():
    b = setup(imem=[mov(MOV_EXEC, MOVSRC_X), jmp(JMP_ALWAYS, 0)])
    poke(b, 0, x=enc.NOP)
    sts = execs(b, 2)
    assert sts[0]["latch_pend"] == 1 and sts[0]["pc"] == 1
    assert sts[1]["latch_pend"] == 0 and sts[1]["pc"] == 1  # the NOP ran


def test_mov_status_compares_the_selected_fifo_level_with_status_n():
    """STATUS is all-ones if the level selected by STATUS_SEL is < STATUS_N."""
    for sel, fifo in ((0, "tx"), (1, "rx")):
        for level in range(5):
            for status_n in (1, 2, 4):
                b = setup(
                    imem=[mov(MOV_X, MOV_STATUS)],
                    execctrl_v=enc.execctrl(status_sel=sel, status_n=status_n),
                )
                poke(b, 0, tx=[0] * level if sel == 0 else [], rx=[0] * level if sel else [])
                expect = 0xFFFFFFFF if level < status_n else 0
                assert execs(b, 1)[0]["x"] == expect, (sel, fifo, level, status_n)


def test_mov_status_n_of_zero_is_all_ones():
    b = setup(
        imem=[mov(MOV_X, MOV_STATUS)],
        execctrl_v=enc.execctrl(status_sel=0, status_n=0),
        tx=[1, 2, 3, 4],
    )
    # Every level 0..4 is not < 0, so STATUS is 0.
    assert execs(b, 1)[0]["x"] == 0


@pytest.mark.parametrize(
    "dst,src,op",
    [
        (MOV_RSVD3, MOVSRC_X, MOV_NONE),  # reserved destination
        (MOV_X, MOVSRC_X, 3),  # reserved Op
        (MOV_X, 4, MOV_NONE),  # reserved source
    ],
)
def test_mov_reserved_field_values_are_nops(dst, src, op):
    b = setup(imem=[mov(dst, src, op), set_(SET_X, 3)])
    poke(b, 0, x=0x11, osr=0x22, isr=0x33)
    sts = execs(b, 2)
    assert sts[0]["x"] == 0x11 and sts[0]["osr"] == 0x22 and sts[0]["isr"] == 0x33
    assert sts[1]["x"] == 3  # the PC advanced: a NOP retires like any other


@pytest.mark.parametrize("op,src", [(3, MOVSRC_OSR), (MOV_NONE, 4)])
def test_a_reserved_mov_exec_is_a_nop_and_not_an_exec_er(op, src):
    """Regression (iss-diff, W3): `MOV EXEC` with a reserved Op or Source is a NOP.

    A NOP loads no latch ("no main effect") and its delay field applies as normal, so it
    is not an EXEC-er even though its Destination is EXEC.
    """
    b = setup(
        imem=[mov(MOV_EXEC, src, op, delay=3, side=0b10, sc=2), set_(SET_X, 3)],
        pinctrl_v=enc.pinctrl(sideset_count=2),
    )
    sts = execs(b, 2 + 4)
    assert sts[0]["latch"] == 0 and sts[0]["latch_pend"] == 0   # no main effect
    assert sts[0]["delay_cnt"] == 3                            # the delay applies
    assert sts[0]["pc"] == 1                                   # and it retires like a NOP
    assert [s["delay_cnt"] for s in sts[:4]] == [3, 2, 1, 0]
    assert sts[4]["x"] == 3                                    # 1 + delay SM cycles later


def test_mov_never_stalls():
    b = setup(imem=[mov(MOV_X, MOVSRC_OSR)])
    poke(b, 0, osr=0, osr_cnt=32)  # an empty OSR would stall an OUT with autopull
    assert execs(b, 1)[0]["stalled"] == 0


# --------------------------------------------------------------------- SET


def test_set_x_and_y_take_a_five_bit_immediate():
    b = setup(imem=[set_(SET_X, 31), set_(SET_Y, 0)])
    sts = execs(b, 2)
    assert sts[0]["x"] == 31
    assert sts[1]["y"] == 0


def test_set_pins_and_pindirs_use_the_set_window():
    b = setup(
        imem=[set_(SET_PINS, 0b101), set_(SET_PINDIRS, 0b011)],
        pinctrl_v=enc.pinctrl(set_base=4, set_count=3),
    )
    # set_(SET_PINS, 0b101): data bits 0..2 go to pins 4..6, so pin 4 and pin 6 high.
    execs(b, 1)
    # SET_COUNT = 3, so the window is pins 4, 5, 6 and data 0b101 goes to them.
    assert b.chip.pin_out & 0b111 << 4 == 0b101 << 4
    execs(b, 1)
    assert b.chip.pin_oe & 0b111 << 4 == 0b011 << 4


def test_set_count_of_six_or_seven_writes_zeros_to_the_extra_pins():
    """MVP: the extra pins are written, with zero data."""
    b = setup(
        imem=[set_(SET_PINS, 0b1111)],
        pinctrl_v=enc.pinctrl(set_base=0, set_count=7),
    )
    execs(b, 1)
    # SET_COUNT = 7 covers pins 0..6; only the four data bits are non-zero.
    assert b.chip.pin_out & 0b1111111 == 0b001111


def test_set_count_zero_writes_nothing():
    b = setup(
        imem=[set_(SET_PINS, 1)],
        pinctrl_v=enc.pinctrl(set_base=0, set_count=0),
    )
    execs(b, 1)
    assert b.chip.pin_out == 0


def test_set_reserved_destinations_are_nops():
    for dst in (3, 5, 6, 7):
        b = setup(imem=[set_(dst, 7), set_(SET_X, 3)])
        sts = execs(b, 2)
        assert sts[0]["x"] == 0 and sts[1]["x"] == 3


def test_set_never_stalls_or_writes_the_shift_registers():
    b = setup(imem=[set_(SET_X, 5)])
    poke(b, 0, osr=0x1234, osr_cnt=32, isr=0x5678, isr_cnt=32)
    st = execs(b, 1)[0]
    assert st["stalled"] == 0
    assert st["osr"] == 0x1234 and st["isr"] == 0x5678
    assert st["x"] == 5


# --------------------------------------------------------------------- IRQ


@pytest.mark.parametrize("bits", [0x00, 0x40, 0x80, 0xC0])
def test_irq_is_a_nop_in_every_form(bits):
    """MVP: no main effect, never stalls; side-set and delay apply as normal."""
    b = setup(imem=[irq(clr=bits >> 7, wait=(bits >> 6) & 1, delay=2), set_(SET_X, 3)])
    sts = execs(b, 4)
    assert [s["x"] for s in sts] == [0, 0, 0, 3]
    assert [s["stalled"] for s in sts] == [0, 0, 0, 0]
    assert sts[0]["delay_cnt"] == 2  # the delay field still applies
    assert b.read(enc.A_FDEBUG) == 0  # no flag is raised


def test_irq_wait_is_a_nop_like_every_other_irq_form():
    """MVP: "**IRQ (opcode 110), all forms** (set, clear, `wait`): no main effect and
    never stalls". `wait 1 irq` (the WAIT instruction) is the form that stalls."""
    b = setup(imem=[irq(wait=1), set_(SET_X, 3)])
    sts = execs(b, 4)
    assert [s["stalled"] for s in sts] == [0] * 4
    # It retires like a NOP, so IMEM[1] runs in the very next SM cycle. IMEM[2] is
    # `jmp 0`, so the two instructions alternate.
    assert [s["x"] for s in sts] == [0, 3, 3, 3]
    assert [s["pc"] for s in sts] == [1, 2, 0, 1]


def test_irq_carries_side_set_like_any_other_instruction():
    b = setup(
        imem=[irq(side=1, sc=1)],
        pinctrl_v=enc.pinctrl(sideset_count=1, sideset_base=5),
    )
    execs(b, 1)
    assert b.chip.pin_out & (1 << 5) == 1 << 5


# ------------------------------------------------------- removed rel/irq form


def test_irq_bits_7_and_6_do_not_change_the_encoding_of_other_opcodes():
    """The bit layout is shared: 110 in 15:13 is IRQ whatever 7:6 hold."""
    assert (irq() >> 13) & 7 == 6
    assert (out(OUT_NULL, 0) >> 13) & 7 == 3
    assert (in_(IN_X, 0) >> 13) & 7 == 2
    assert (jmp(JMP_ALWAYS, 0) >> 13) & 7 == 0
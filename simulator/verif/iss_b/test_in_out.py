"""IN and OUT (ISA `### Instruction semantics`).

Shift direction, bit counts, `osr_cnt`/`isr_cnt` bookkeeping, autopush/autopull,
the OUT refill and its stall, and the register writes of every destination.
"""

import pytest

from verif.iss_b import enc
from verif.iss_b.enc import execs, in_, jmp, out, poke, push, set_, setup
from verif.iss_b.enc import IN_ISR, IN_NULL, IN_OSR, IN_PINS, IN_X, IN_Y
from verif.iss_b.enc import OUT_EXEC, OUT_ISR, OUT_NULL, OUT_PC, OUT_PINDIRS
from verif.iss_b.enc import OUT_PINS, OUT_X, OUT_Y, JMP_ALWAYS

PIN1 = 1 << 1


# --------------------------------------------------------------------- IN


@pytest.mark.parametrize(
    "source,pin,expect",
    [
        (IN_X, 0, 0xA5),
        (IN_Y, 0, 0x5A),
        (IN_NULL, 0, 0),
        (IN_ISR, 0, 0x1234),
        (IN_OSR, 0, 0x5678),
        (IN_PINS, 0b101, 0b101),  # IN_BASE = 0, so IN_ROT is IN_VEC
    ],
)
def test_in_sources_read_the_start_of_cycle_value(source, pin, expect):
    """`d` is the low `n` bits of the source, with n = 32 for the whole word."""
    b = setup(imem=[in_(source, 32)], pin=pin)
    poke(b, 0, x=0xA5, y=0x5A, isr=0x1234, osr=0x5678)
    # With n = 32 the ISR becomes d in either shift direction, so it shows the read.
    assert execs(b, 1)[0]["isr"] == expect
    assert execs(b, 1)[0]["isr_cnt"] == 32


def test_in_shift_right_and_left():
    b = setup(imem=[in_(IN_X, 8)], shiftctrl_v=enc.shiftctrl(in_shiftdir=1))
    poke(b, 0, x=0xFF, isr=0)
    assert execs(b, 1)[0]["isr"] == 0xFF000000  # (0 >> 8) | (0xFF << 24)

    b = setup(imem=[in_(IN_X, 8)], shiftctrl_v=enc.shiftctrl(in_shiftdir=0))
    poke(b, 0, x=0xFF, isr=1)
    assert execs(b, 1)[0]["isr"] == 0x1FF  # (1 << 8) | 0xFF


def test_in_bit_count_of_32_replaces_the_isr_in_both_directions():
    for shiftdir in (0, 1):
        b = setup(imem=[in_(IN_X, 0)], shiftctrl_v=enc.shiftctrl(in_shiftdir=shiftdir))
        poke(b, 0, x=0xDEADBEEF, isr=0xFFFFFFFF)
        assert execs(b, 1)[0]["isr"] == 0xDEADBEEF


def test_in_masks_the_source_to_the_bit_count():
    b = setup(imem=[in_(IN_X, 4)], shiftctrl_v=enc.shiftctrl(in_shiftdir=0))
    poke(b, 0, x=0xFFFFFFFF, isr=0)
    assert execs(b, 1)[0]["isr"] == 0xF  # only the low 4 bits


def test_in_isr_cnt_saturates_at_32():
    b = setup(imem=[in_(IN_X, 8), in_(IN_X, 8), in_(IN_X, 8), in_(IN_X, 8), in_(IN_X, 8)])
    poke(b, 0, isr_cnt=16)
    sts = execs(b, 3)
    assert [s["isr_cnt"] for s in sts] == [24, 32, 32]


def test_in_pins_uses_in_rot_with_in_base():
    """IN_VEC is rotated right by IN_BASE, so bit 0 is pin IN_BASE."""
    b = setup(
        imem=[in_(IN_PINS, 4)],
        pinctrl_v=enc.pinctrl(in_base=2),
        pin=0b1100,  # pins 2 and 3
    )
    # IN_ROT bit 0 = pin 2 = 1, bit 1 = pin 3 = 1. Shifting right by 4 puts them
    # at the MSBs.
    assert execs(b, 1)[0]["isr"] == 0x30000000


def test_in_reserved_sources_are_nops():
    b = setup(imem=[in_(enc.IN_RSVD45, 8), set_(enc.SET_X, 1)])
    st = execs(b, 2)[0]
    assert st["isr"] == 0 and st["isr_cnt"] == 0  # no shift at all


def test_in_autopush_pushes_and_resets_isr():
    b = setup(
        imem=[in_(IN_X, 8)],
        shiftctrl_v=enc.shiftctrl(autopush=1, push_thresh=8),
    )
    poke(b, 0, x=0xAB, isr=0)
    st = execs(b, 1)[0]
    assert st["isr"] == 0 and st["isr_cnt"] == 0
    assert st["rx_level"] == 1
    # IN_SHIFTDIR = 1 by default, so 0xAB entered at the MSB: ISR = 0xAB000000.
    assert b.pop_rx(0) == 0xAB000000


def test_in_autopush_stalls_on_a_full_rx_fifo_and_is_atomic():
    """MVP: a stalled IN shifts nothing and re-samples its source when it completes."""
    b = setup(
        imem=[in_(IN_X, 8)],
        shiftctrl_v=enc.shiftctrl(autopush=1, push_thresh=8),
    )
    poke(b, 0, x=0x11, rx=[0x01010101] * 4)  # a full RX FIFO to start with
    sts = execs(b, 4)
    # The IN finds the FIFO full from its first SM cycle and stalls there.
    assert [s["stalled"] for s in sts] == [1, 1, 1, 1]
    assert [s["rx_level"] for s in sts] == [4] * 4  # nothing pushed
    # Atomic: the ISR was not shifted and isr_cnt was left where it was.
    assert sts[0]["isr"] == 0 and sts[0]["isr_cnt"] == 0
    assert [s["pc"] for s in sts] == [0] * 4  # the PC does not advance while stalled
    assert b.read(enc.A_FDEBUG) & 0x1  # FDEBUG.RXSTALL[0]

    # Draining one word lets it complete; it samples X again in that cycle.
    assert b.pop_rx(0) == 0x01010101
    st = execs(b, 1)[0]
    assert st["stalled"] == 0 and st["rx_level"] == 4
    assert st["pc"] == 1  # the instruction retired this time


def test_in_autopush_pushes_until_the_fifo_is_full():
    b = setup(
        imem=[in_(IN_X, 8)],
        shiftctrl_v=enc.shiftctrl(autopush=1, push_thresh=8),
    )
    poke(b, 0, x=0x11)
    sts = execs(b, 9)
    # IMEM[1] resets to `jmp 0`, so the engine loops over IMEM[0]: it pushes on every
    # other SM cycle, and once the FIFO is full the IN stalls and stays there.
    assert [s["rx_level"] for s in sts] == [1, 1, 2, 2, 3, 3, 4, 4, 4]
    assert [s["stalled"] for s in sts] == [0, 0, 0, 0, 0, 0, 0, 0, 1]
    assert sts[-1]["pc"] == 0  # stalled: the PC does not advance
    assert sts[-1]["isr"] == 0 and sts[-1]["isr_cnt"] == 0  # pushed, so ISR is clear
    assert b.read(enc.A_FDEBUG) & 0x1  # FDEBUG.RXSTALL[0]


def test_in_without_autopush_just_fills_the_isr_past_the_threshold():
    b = setup(imem=[in_(IN_X, 8)], shiftctrl_v=enc.shiftctrl(autopush=0, push_thresh=8))
    poke(b, 0, x=0xFF, isr_cnt=0)
    st = execs(b, 1)[0]
    assert st["rx_level"] == 0 and st["isr_cnt"] == 8 and st["isr"] == 0xFF000000


# -------------------------------------------------------------------- OUT


def test_out_destinations():
    b = setup(imem=[out(OUT_X, 8)])
    poke(b, 0, osr=0xAABBCCDD)
    st = execs(b, 1)[0]
    assert st["x"] == 0xDD  # right shift, the low 8 bits
    assert st["osr"] == 0x00AABBCC and st["osr_cnt"] == 32

    b = setup(imem=[out(OUT_Y, 8)], shiftctrl_v=enc.shiftctrl(out_shiftdir=0))
    poke(b, 0, osr=0xAABBCCDD)
    assert execs(b, 1)[0]["y"] == 0xAA  # left shift, top 8 bits

    # NULL
    b = setup(imem=[out(OUT_NULL, 32)])
    poke(b, 0, osr=0xFFFFFFFF)
    st = execs(b, 1)[0]
    assert st["osr"] == 0 and st["osr_cnt"] == 32  # the shift still happens

    # ISR: ISR = v and isr_cnt = n
    b = setup(imem=[out(OUT_ISR, 4)])
    poke(b, 0, osr=0xF0)
    st = execs(b, 1)[0]
    assert st["isr"] == 0x0 and st["isr_cnt"] == 4  # the low 4 bits of 0xF0


    # PC: the next PC is v[4:0]
    b = setup(imem=[out(OUT_PC, 5)])
    poke(b, 0, osr=0x1D)
    assert execs(b, 1)[0]["pc"] == 0x1D & 0x1F


def test_out_shift_right_and_left_and_the_32_bit_case():
    b = setup(imem=[out(OUT_X, 8)])
    poke(b, 0, osr=0x12345678)
    st = execs(b, 1)[0]
    assert st["osr"] == 0x00123456 and st["osr_cnt"] == 40 - 8

    b = setup(imem=[out(OUT_X, 8)], shiftctrl_v=enc.shiftctrl(out_shiftdir=0))
    poke(b, 0, osr=0x12345678)
    st = execs(b, 1)[0]
    assert st["osr"] == 0x34567800

    b = setup(imem=[out(OUT_X, 0)])  # bit count 00000 encodes 32
    poke(b, 0, osr=0xFFFFFFFF)
    st = execs(b, 1)[0]
    assert st["x"] == 0xFFFFFFFF and st["osr"] == 0


def test_out_pins_writes_the_out_window_and_pindirs_the_direction():
    b = setup(
        imem=[out(OUT_PINS, 4), out(OUT_PINDIRS, 4)],
        pinctrl_v=enc.pinctrl(out_base=3, out_count=4),
    )
    poke(b, 0, osr=0xFF)  # a low nibble for the levels and one for the directions
    execs(b, 1)  # OUT PINS 4: the low nibble 0xF to pins 3..6
    assert b.chip.pin_out & 0b1111 << 3 == 0b1111 << 3
    assert b.chip.eng[0].osr == 0xF  # shifted right by 4
    execs(b, 1)  # OUT PINDIRS 4: the same nibble drives pins 3..6
    assert b.chip.pin_oe & 0b1111 << 3 == 0b1111 << 3
    assert b.chip.eng[0].osr == 0
    assert execs(b, 2)[1]["pc"] == 1  # IMEM[2] is `jmp 0`, so IMEM[0] runs again
    # That OUT PINS has an empty OSR: the pins go low, and the levels hold until
    # something writes them.
    assert b.chip.pin_out & 0b1111 << 3 == 0
    assert b.chip.pin_oe & 0b1111 << 3 == 0b1111 << 3  # directions are sticky


def test_out_count_zero_writes_no_pin():
    b = setup(
        imem=[out(OUT_PINS, 1)],
        pinctrl_v=enc.pinctrl(out_base=0, out_count=0),
    )
    poke(b, 0, osr=1)
    execs(b, 1)
    assert b.chip.pin_out == 0


def test_out_pins_clears_bits_at_and_above_the_bit_count():
    """The window is OUT_COUNT pins wide, but the data above `n` is zero."""
    b = setup(
        imem=[out(OUT_PINS, 2)],
        pinctrl_v=enc.pinctrl(out_base=0, out_count=8),
    )
    poke(b, 0, osr=0xFFFFFFFF)
    execs(b, 1)
    assert b.chip.pin_out & 0xFF == 0b11  # 8 pins written, only 2 with data


def test_out_autopull_refills_once_the_threshold_is_reached():
    b = setup(
        imem=[out(OUT_NULL, 8)],
        shiftctrl_v=enc.shiftctrl(autopull=1, pull_thresh=8),
    )
    b.push_tx(0, 0xAAAA5555)
    poke(b, 0, osr=0x12345678, osr_cnt=0)
    st = execs(b, 1)[0]
    assert st["osr_cnt"] == 0  # the refill happened in the same cycle
    assert st["osr"] == 0xAAAA5555
    assert st["tx_level"] == 0
    assert st["x"] == 0  # NULL: the old OSR's low 8 bits went nowhere


def test_out_autopull_stalls_at_least_one_cycle_when_the_osr_is_at_the_threshold():
    """`osr_cnt >= thresh(PULL_THRESH)` at the start of the cycle: stall, and refill
    from the TX FIFO in the same cycle if it is not empty."""
    b = setup(
        imem=[out(OUT_NULL, 8)],
        shiftctrl_v=enc.shiftctrl(autopull=1, pull_thresh=8),
    )
    poke(b, 0, osr=0xFFFFFFFF, osr_cnt=8)
    st = execs(b, 1)[0]
    assert st["stalled"] == 1 and st["osr_cnt"] == 8  # unchanged, and no shift
    assert st["osr"] == 0xFFFFFFFF
    assert b.read(enc.A_FDEBUG) & (1 << 24)  # FDEBUG.TXSTALL[0]: the FIFO was empty


def test_out_autopull_the_stall_cycle_refills_the_fifo_and_the_next_one_shifts():
    b = setup(
        imem=[out(OUT_NULL, 8)],
        shiftctrl_v=enc.shiftctrl(autopull=1, pull_thresh=8),
        tx=[0xDEAD],
    )
    poke(b, 0, osr=0xFFFFFFFF, osr_cnt=8)
    st = execs(b, 1)[0]
    # This stall cycle pops the FIFO, so the OSR is refilled and osr_cnt is 0.
    assert st["stalled"] == 1 and st["osr"] == 0xDEAD and st["osr_cnt"] == 0
    assert st["tx_level"] == 0
    st = execs(b, 1)[0]
    # The stall is over: 0xDE is shifted out and osr_cnt is back at the threshold.
    # The refill that would follow finds the FIFO empty, so nothing changes.
    assert st["stalled"] == 0 and st["osr"] == 0xDE and st["osr_cnt"] == 8


def test_out_autopull_an_empty_osr_cannot_be_refilled_and_shifted_in_one_cycle():
    """An empty OSR stalls at least one SM cycle even when the FIFO holds data."""
    b = setup(
        imem=[out(OUT_NULL, 8)],
        shiftctrl_v=enc.shiftctrl(autopull=1, pull_thresh=8),
        tx=[0xBEEF],
    )
    poke(b, 0, osr=0, osr_cnt=8)
    st = execs(b, 1)[0]
    assert st["stalled"] == 1  # stalled, but refilled from the FIFO
    assert st["osr"] == 0xBEEF and st["osr_cnt"] == 0
    st = execs(b, 1)[0]
    assert st["stalled"] == 0  # the refill cycle is over
    # 0xBEEF shifted right by 8 leaves 0xBE, and the OSR is at the threshold again.
    assert st["osr"] == 0xBE and st["osr_cnt"] == 8
    st = execs(b, 1)[0]
    # Back at the threshold with an empty FIFO, so the next cycle stalls again.
    assert st["stalled"] == 0 and st["osr"] == 0xBE and st["osr_cnt"] == 8
    assert execs(b, 1)[0]["stalled"] == 1  # now it stalls on the empty FIFO


def test_out_exec_latches_and_ignores_its_delay():
    b = setup(imem=[out(OUT_EXEC, 16, delay=5), jmp(JMP_ALWAYS, 0)])
    poke(b, 0, osr=0xA042)
    sts = execs(b, 3)
    # The OUT EXEC runs in one SM cycle, advances the PC and loads no delay.
    assert sts[0]["latch"] == 0xA042 and sts[0]["latch_pend"] == 1
    assert sts[0]["delay_cnt"] == 0 and sts[0]["pc"] == 1
    # The next SM cycle runs the executee from the latch: `nop` does not advance the
    # PC, and it clears latch_pend because it is not itself an EXEC.
    assert sts[1]["latch_pend"] == 0 and sts[1]["pc"] == 1
    # Then IMEM[1] runs, and IMEM[2] is `jmp 0`.
    assert sts[2]["pc"] == 0
    # latch_host is 0 throughout, so EXECCTRL.EXEC_STALLED reads 0.
    assert b.read(enc.a_sm(0, 1)) & (1 << 31) == 0


def test_out_is_an_out_even_while_stalled_and_does_not_shift():
    b = setup(
        imem=[out(OUT_X, 8)],
        shiftctrl_v=enc.shiftctrl(autopull=1, pull_thresh=8),
    )
    poke(b, 0, osr=0x1234, osr_cnt=8)
    st = execs(b, 1)[0]
    assert st["stalled"] == 1 and st["osr"] == 0x1234 and st["x"] == 0
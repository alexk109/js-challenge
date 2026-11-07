"""Side-set, delay, and pin resolution (ISA `### Stalls, delay and side-set,
summarised`, `### Pin writes`, `## Pins`).
"""

from verif.iss_b import enc
from verif.iss_b.enc import execs, jmp, mov, out, poke, set_, setup, wait_
from verif.iss_b.enc import JMP_ALWAYS, MOV_X, OUT_PINS
from verif.iss_b.enc import SET_PINDIRS, SET_PINS, SET_X, WAIT_GPIO


# ------------------------------------------------------------------- side-set


def test_side_set_writes_every_instruction_when_side_en_is_zero():
    """`With SIDE_EN = 0: all SC bits are data, and side-set happens on every
    instruction when SC >= 1.`"""
    prog = [set_(SET_X, 0, side=0b11, sc=2), set_(SET_X, 1, side=0b10, sc=2)]
    b = setup(imem=prog, pinctrl_v=enc.pinctrl(sideset_count=2, sideset_base=0))
    execs(b, 1)
    assert b.chip.pin_out & 0b11 == 0b11  # data bits 0 and 1
    execs(b, 1)
    assert b.chip.pin_out & 0b11 == 0b10  # bit 0 cleared, bit 1 held


def test_side_en_makes_the_msb_of_the_side_set_bits_an_enable():
    b = setup(
        imem=[set_(SET_X, 0, side=0b11, sc=2), set_(SET_X, 1, side=0b10, sc=2)],
        execctrl_v=enc.execctrl(side_en=1),
        pinctrl_v=enc.pinctrl(sideset_count=2, sideset_base=0),
    )
    execs(b, 1)
    assert b.chip.pin_out & 0b11 == 0b01  # enable 1, data 1
    execs(b, 1)
    # side = 0b10 with SIDE_EN = 1: the MSB is the enable (1) and the data bit is 0,
    # so both pins are written with 0.
    assert b.chip.pin_out & 0b11 == 0


def test_side_en_leaves_only_sc_minus_one_pins_in_the_side_set_window():
    """Regression (iss-diff, W3): with `SIDE_EN = 1` the enable bit is *not* a data bit.

    ISA `### Delay/side-set field`: "the MSB of the side-set bits is an enable, and
    the remaining `SC - 1` bits are side-set data ... Side-set data bit 0 goes to pin
    `SIDESET_BASE`". So the window is `SC - 1` pins wide: the pin at
    `SIDESET_BASE + SC - 1` is not written at all.
    """
    # `set pins, 0b100` drives pin 2 high (SET_BASE = 0, SET_COUNT = 3).
    b = setup(
        imem=[set_(SET_PINS, 0b100), set_(SET_X, 0, side=0b111, sc=3)],
        execctrl_v=enc.execctrl(side_en=1),
        pinctrl_v=enc.pinctrl(sideset_count=3, sideset_base=0, set_base=0, set_count=3),
    )
    execs(b, 1)
    assert b.chip.pin_out & 0b111 == 0b100
    execs(b, 1)
    # side = 0b111 with SC = 3: enable 1, data 0b11 -> pins SIDESET_BASE + 0 and + 1.
    assert b.chip.pin_out & 0b111 == 0b111          # pin 2 still 1 from the SET


def test_side_set_happens_only_on_the_first_cycle_of_an_instruction():
    """`Side-set is written once, in the first SM cycle of the instruction, even if it
    stalls`: a stalled instruction must not put its level back."""
    prog = [wait_(WAIT_GPIO, 0, pol=1, side=0b1, sc=1), jmp(JMP_ALWAYS, 0)]
    b = setup(imem=prog, pinctrl_v=enc.pinctrl(sideset_count=1, sideset_base=2))
    assert execs(b, 1)[0]["stalled"] == 1
    assert b.chip.pin_out & (1 << 2) == 1 << 2  # written in its first cycle

    # Something else clears pin 2 (the SET window covers pins 0..4 by default).
    b.force(0, set_(SET_PINS, 0))
    assert execs(b, 1)[0]["x"] == 0
    assert b.chip.pin_out & (1 << 2) == 0  # the stalled WAIT did not write it again

    # When the WAIT finally completes it is on a later cycle, so it writes again only
    # if `stalled` was cleared in between; a forced instruction that completes does
    # exactly that (see test_exec.py).


def test_side_set_with_sideset_count_zero_never_happens():
    b = setup(imem=[set_(SET_X, 0, delay=5)], pinctrl_v=enc.pinctrl(sideset_count=0))
    st = execs(b, 1)[0]
    assert st["delay_cnt"] == 5  # the whole field is delay
    assert b.chip.pin_out == 0


def test_side_set_count_of_six_or_seven_behaves_as_five():
    for sc in (6, 7):
        b = setup(
            imem=[set_(SET_X, 0, side=0b11111, sc=5)],
            pinctrl_v=enc.pinctrl(sideset_count=sc, sideset_base=1),
        )
        execs(b, 1)
        assert b.chip.pin_out & 0b11111 << 1 == 0b11111 << 1


def test_side_pindir_sends_the_side_set_to_the_direction_registers():
    b = setup(
        imem=[set_(SET_X, 0, side=0b1111, sc=4)],
        execctrl_v=enc.execctrl(side_pindir=1),
        pinctrl_v=enc.pinctrl(sideset_count=4, sideset_base=4),
    )
    execs(b, 1)
    assert b.chip.pin_oe & 0b1111 << 4 == 0b1111 << 4
    assert b.chip.pin_out & 0b1111 << 4 == 0  # levels untouched


def test_side_set_data_bits_map_to_sideset_base_plus_k_mod_32():
    b = setup(
        imem=[set_(SET_X, 0, side=0b1000, sc=4)],
        pinctrl_v=enc.pinctrl(sideset_count=4, sideset_base=31),
    )
    execs(b, 1)
    # Data bit 3 -> pin (31 + 3) mod 32 = 2.
    assert b.chip.pin_out & (1 << 2) == 1 << 2


# ---------------------------------------------------------------------- delay


def test_a_non_jump_retires_in_one_plus_delay_sm_cycles():
    for delay in (0, 1, 2, 5):
        b = setup(imem=[set_(SET_X, 1, delay=delay), set_(SET_X, 2)])
        sts = execs(b, delay + 2)
        assert sts[0]["x"] == 1  # the first SM cycle retires the SET
        # The counter counts down over the next `delay` SM cycles, then IMEM[1] runs:
        # one instruction per 1 + delay SM cycles.
        assert [s["delay_cnt"] for s in sts[: delay + 1]] == list(range(delay, -1, -1))
        assert [s["x"] for s in sts[: delay + 1]] == [1] * (delay + 1), delay
        assert sts[delay + 1]["x"] == 2, delay


def test_the_delay_counter_is_cleared_by_a_restart():
    b = setup(imem=[set_(SET_X, 1, delay=7)])
    assert execs(b, 1)[0]["delay_cnt"] == 7
    b.write(enc.A_CTRL, enc.ctrl(sm_enable=1, sm_restart=1))
    assert b.state(0)["delay_cnt"] == 0


def test_a_stalled_instruction_loads_no_delay():
    b = setup(imem=[wait_(WAIT_GPIO, 0, pol=1, delay=7)])
    st = execs(b, 1)[0]
    assert st["stalled"] == 1 and st["delay_cnt"] == 0


def test_a_blocking_instruction_takes_stall_plus_one_plus_delay():
    """`A blocking instruction therefore takes stall + 1 + delay SM cycles.`"""
    b = setup(imem=[wait_(WAIT_GPIO, 0, pol=1, delay=2), set_(SET_X, 9)])
    # The pin is low, so it stalls; the level driven at drive cycle 0 reaches IN_VEC
    # at drive cycle 3, so the WAIT completes there.
    sts = enc.drive(b, [1] * 7)
    assert [s["pc"] for s in sts] == [0, 0, 0, 1, 1, 1, 2]
    assert [s["stalled"] for s in sts] == [0, 1, 1, 0, 0, 0, 0]
    assert sts[6]["x"] == 9  # 2 stall cycles + 1 + delay 2


# ------------------------------------------------------------ pin resolution


def test_side_set_wins_over_the_out_channel_within_one_engine():
    """`Within an engine, the side-set channel wins over the OUT/SET channel.`"""
    b = setup(
        imem=[set_(SET_PINS, 0b1, side=0b1, sc=1)],
        pinctrl_v=enc.pinctrl(sideset_count=1, sideset_base=0, set_base=0, set_count=1),
    )
    execs(b, 1)
    assert b.chip.pin_out & 1 == 1  # side-set wrote 1; SET wanted 0 but lost


def test_engine_one_wins_over_engine_zero():
    b = setup()
    b.cfg(1, pinctrl_v=enc.pinctrl(set_base=0, set_count=1))
    b.load_imem(0, [set_(SET_PINS, 0b1), jmp(JMP_ALWAYS, 0)])
    b.load_imem(1, [set_(SET_PINS, 0b0), jmp(JMP_ALWAYS, 0)])
    b.write(enc.A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    b.idle(1)
    while not (sm(b, 0) and sm(b, 1)):
        b.step()
    b.step()
    assert b.chip.pin_out & 1 == 0  # engine 1's zero won


def test_each_register_is_resolved_separately():
    """Engine 0 may set a level while engine 1 sets the direction of the same pin."""
    b = setup()
    b.cfg(1, pinctrl_v=enc.pinctrl(set_base=0, set_count=1))
    b.load_imem(0, [set_(SET_PINS, 0b1), jmp(JMP_ALWAYS, 0)])
    b.load_imem(1, [set_(SET_PINDIRS, 0b1), jmp(JMP_ALWAYS, 0)])
    b.write(enc.A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    b.idle(1)
    while not (sm(b, 0) and sm(b, 1)):
        b.step()
    b.step()
    assert b.chip.pin_out & 1 == 1  # engine 0's level
    assert b.chip.pin_oe & 1 == 1  # and engine 1's direction


def sm(b, e):
    from verif.iss_b.enc import sm_cycle_now

    return sm_cycle_now(b, e)


def test_a_pin_nobody_writes_keeps_its_value():
    b = setup(imem=[set_(SET_PINS, 0b1, sc=0)], pinctrl_v=enc.pinctrl(set_base=0, set_count=1))
    execs(b, 1)
    assert b.chip.pin_out & 1 == 1
    poke(b, 0, x=0)
    b.force(0, jmp(JMP_ALWAYS, 0))
    b.idle(4)
    assert b.chip.pin_out & 1 == 1  # nothing wrote pin 0 again


def test_writes_to_input_only_pins_are_dropped():
    """Pins 8..12 have no level and no direction register: writes are dropped.

    The window is placed over exactly those pins, so the write is aimed at them.
    """
    for base, count in ((8, 5), (9, 3), (0, 32)):
        b = setup(
            imem=[set_(SET_PINS, 0b11111, sc=0), set_(SET_PINDIRS, 0b11111, sc=0)],
            pinctrl_v=enc.pinctrl(set_base=base, set_count=count),
        )
        execs(b, 1)
        assert b.chip.lvl & 0x1F00 == 0, base  # pins 8..12 have no level register
        assert b.chip.dir & 0x1F00 == 0, base  # nor a direction register
        assert b.chip.pin_out & 0x1F00 == 0, base
        assert b.chip.pin_oe & 0x1F00 == 0, base
        assert b.read(enc.A_PADOUT) & 0x1F00 == 0
        assert b.read(enc.A_PADOE) & 0x1F00 == 0
        if base != 0:
            # With OUT_COUNT = 32 the same write reaches pins 8..12 and stops there.
            assert b.chip.pin_out & 0xFF == 0, base


def test_writes_beside_the_input_only_pins_still_take_effect():
    """Pins 7 and 13 are writable; only 8..12 are dropped."""
    b = setup(
        imem=[set_(SET_PINS, 0b1111_1111, sc=0)],
        # SET_COUNT is three bits, so 7 is the widest window: pins 6..12.
        pinctrl_v=enc.pinctrl(set_base=6, set_count=7),
    )
    execs(b, 1)
    assert b.chip.pin_out & 0b11 << 6 == 0b11 << 6  # pins 6 and 7: uio
    assert b.chip.pin_out & 0x1F00 == 0  # pins 8..12 dropped

    # An OUT window reaches pin 13, which is output-only and writable.
    c = setup(imem=[out(OUT_PINS, 32)], pinctrl_v=enc.pinctrl(out_base=0, out_count=32))
    poke(c, 0, osr=0xFFFFFFFF)
    execs(c, 1)
    assert c.chip.lvl & (1 << 13) == 1 << 13  # pin 13 takes a level write


def test_writes_to_phantom_pins_18_to_31_are_dropped():
    b = setup(imem=[out(OUT_PINS, 32)], pinctrl_v=enc.pinctrl(out_base=0, out_count=32))
    poke(b, 0, osr=0xFFFFFFFF)
    execs(b, 1)
    # A window that starts in the phantom range drops everything it covers.
    c = setup(
        imem=[out(OUT_PINS, 32), set_(SET_PINS, 0b11111, sc=0)],
        pinctrl_v=enc.pinctrl(out_base=20, out_count=12, set_base=30, set_count=2),
    )
    poke(c, 0, osr=0xFFFFFFFF)
    execs(c, 2)
    # Nothing lands in any level register, phantom or otherwise.
    assert c.chip.pin_out == 0 and c.chip.lvl == 0
    assert c.chip.dir == 0
    # Only pins 0..7 (uio) and 13..17 (uo_out) have level registers.
    assert b.chip.pin_out == 0xFF | 0x3E000
    assert b.read(enc.A_PADOUT) & 0xFFFC0000 == 0  # bits 31:18 read 0
    assert b.read(enc.A_PADOE) & 0xFFFC0000 == 0


def test_output_only_pins_take_levels_ignore_pindirs_and_read_back():
    """Pins 13..17: levels are written, directions are dropped and `pin_oe` is 1."""
    b = setup(imem=[set_(SET_PINS, 0b1, sc=0), set_(SET_PINDIRS, 0b1, sc=0)])
    b.cfg(0, pinctrl_v=enc.pinctrl(set_base=13, set_count=1))
    execs(b, 1)
    assert b.chip.pin_out & (1 << 13) == 1 << 13
    assert b.chip.pin_oe & (1 << 13) == 1 << 13  # always driven
    execs(b, 1)
    assert b.chip.pin_oe & (1 << 13) == 1 << 13  # the PINDIRS write was dropped


def test_an_output_only_pin_feeds_back_its_driven_level_after_t_in():
    b = setup(
        imem=[set_(SET_PINS, 1, sc=0), mov(MOV_X, 0)],
        pinctrl_v=enc.pinctrl(set_base=13, set_count=1),
    )
    sts = execs(b, 12)
    xs = [s["x"] for s in sts]
    assert xs[0] == 0
    # `set pins` writes pin 13 at the end of SM cycle 0, so it is on the pad in the
    # next cycle, and T_IN = 3 puts it in IN_VEC three cycles after that.
    hi = [i for i, x in enumerate(xs) if x & (1 << 13)]
    assert hi == list(range(4, 12)), xs


def test_open_drain_by_toggling_the_direction_keeps_the_level_at_zero():
    """`For open drain on a uio pin, the level stays 0 and the program toggles the
    direction.`"""
    # `set pindirs, 1` then `set pindirs, 0`, then `jmp 0` to loop.
    b = setup(
        imem=[set_(SET_PINDIRS, 1), set_(SET_PINDIRS, 0), jmp(JMP_ALWAYS, 0)],
        pinctrl_v=enc.pinctrl(set_base=0, set_count=1),
    )
    seen = []
    for _ in range(6):
        execs(b, 1)
        # pad_out/pad_oe are the pad values of the cycle just stepped (T_OUT = 1).
        seen.append((b.pad_out(0), b.pad_oe(0)))
    # The level never leaves 0; the direction is written on the two SETs and holds on
    # the `jmp` cycle, so the pattern is 1, 0, 0, 1, 0, 0.
    assert [lvl for lvl, _oe in seen] == [0] * 6
    assert [oe for _lvl, oe in seen] == [1, 0, 0, 1, 0, 0]


def test_dbg_padout_and_dbg_padoe_reset_values():
    b = setup()
    assert b.chip.pin_out == 0  # all levels 0
    assert b.chip.pin_oe == 0x3E000  # pins 13..17 always driven, 8..12 read 0
    assert b.read(enc.A_PADOUT) == 0
    assert b.read(enc.A_PADOE) == 0x3E000


def test_pin_out_is_an_eighteen_bit_vector():
    b = setup(imem=[out(OUT_PINS, 32)], pinctrl_v=enc.pinctrl(out_base=0, out_count=32))
    poke(b, 0, osr=0xFFFFFFFF)
    execs(b, 1)
    assert b.chip.pin_out < (1 << 18)
    assert b.chip.pin_oe < (1 << 18)
"""Clock divider, reset and SM enable (ISA `### Clock divider`, `### Enable,
restart`, `## Timing` `### Constants`).
"""

from verif.iss_b import enc
from verif.iss_b.enc import Bus, poke, set_, setup, sm_cycle_now, execs
from verif.iss_b.enc import A_CTRL, JMP_ALWAYS, SET_X

A_CLK = enc.a_sm(0, 0)  # SM0_CLKDIV is word 0 of the engine's register block


def divider(chip: Bus, e: int = 0):
    """The engine's divider state, read from the ISS (no register exposes it)."""
    eng = chip.chip.eng[e]
    return eng.d_cnt, eng.d_acc, eng.d_en


def test_divider_reset_state():
    b = Bus()
    assert divider(b) == (1, 0, 0)  # cnt 1, acc 0, en 0


def test_a_restart_gives_en_zero_then_one():
    """`A restart in cycle h gives en = 0 in h+1 and en = 1 in h+2.`"""
    b = Bus()
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    assert divider(b) == (1, 0, 0)  # end of cycle h
    b.idle(1)
    assert divider(b)[2] == 1  # end of h+1: the reload sets en
    assert divider(b)[0] == 1  # and cnt reloads to INTe + carry = 1 for CLKDIV 1.0


def test_an_integer_divisor_gives_one_sm_cycle_every_d_core_cycles():
    for d in (1, 2, 3, 5):
        b = Bus()
        b.write(A_CLK, enc.clkdiv(d, 0))
        b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
        # cnt reloads to INT every INT cycles, and en is 1 on the reload cycle.
        ens = []
        for _ in range(3 * d + 1):
            ens.append(divider(b)[2])
            b.step()
        # en is high on the reload cycle and low in between: one SM cycle every d
        # core cycles, starting two cycles after the restart write.
        assert ens.count(1) == 3, (d, ens)
        assert [i for i, e in enumerate(ens) if e] == [1, 1 + d, 1 + 2 * d], (d, ens)


def test_clkdiv_one_gives_en_in_every_cycle_from_cycle_one():
    b = Bus()
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.step()
    ens = [divider(b)[2]]
    for _ in range(5):
        b.step()
        ens.append(divider(b)[2])
    assert ens == [1] * 6


def test_a_fractional_divisor_alternates_the_spacing():
    """`the spacing alternates between INT and INT + 1 as the algorithm dictates`."""
    b = Bus()
    b.write(A_CLK, enc.clkdiv(2, 0x80))  # FRAC 128/256: one long period in two
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    cnts = []
    for _ in range(14):
        cnts.append(divider(b)[0])
        b.step()
    # cnt reloads to INTe + carry; FRAC = 128/256 carries every other reload, so both
    # spacings appear.
    assert 2 in cnts  # INT
    assert 3 in cnts  # INT + carry


def test_the_accumulator_wraps_and_carries():
    b = Bus()
    b.write(A_CLK, enc.clkdiv(4, 0xFF))  # nearly always carries
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    seen = [divider(b) for _ in range(20)]
    for _ in range(20):
        b.step()
        seen.append(divider(b))
    # With FRAC = 255 every reload carries, so cnt is always INT + 1 = 5 and acc
    # wraps every time.
    assert {cnt for cnt, _a, _e in seen} == {1, 5, 4, 3, 2}
    # Every reload adds FRAC (255) to acc and keeps only the low 8 bits, so acc
    # decreases by one per reload.
    reloads = [acc for cnt, acc, _e in seen if cnt == 5]
    assert reloads[:4] == [0xFE, 0xFD, 0xFC, 0xFB]
    # (0 + 255) & 0xFF = 255 with a carry, then (255 + 255) & 0xFF = 254, and so on:
    # every reload carries, so the period is always INT + 1 = 5 core cycles.


def test_int_zero_means_65536():
    b = Bus()
    b.write(A_CLK, enc.clkdiv(0, 0))
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.idle(1)
    assert divider(b)[0] == 65536  # INTe with INT = 0
    assert divider(b)[2] == 1
    b.idle(65535)
    assert divider(b)[0] == 1  # back at the reload


def test_int_zero_with_a_fraction_is_defined():
    """MVP: `INT = 0 with FRAC != 0` is defined by the formula above."""
    b = Bus()
    b.write(A_CLK, enc.clkdiv(0, 0x80))
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.idle(1)
    cnt, acc, _en = divider(b)
    assert cnt in (65536, 65537)


def test_the_divider_runs_even_when_the_engine_is_disabled():
    """`The divider runs all the time, enabled or not`."""
    b = Bus()
    b.write(A_CLK, enc.clkdiv(3, 0))
    b.write(A_CTRL, enc.ctrl(sm_enable=0, clkdiv_restart=1))
    counts = []
    for _ in range(9):
        counts.append(divider(b)[0])
        b.step()
    assert counts != [1] * 9  # it is counting down
    assert not sm_cycle_now(b, 0)  # but the engine never runs


def test_a_clkdiv_write_is_picked_up_at_the_next_reload():
    b = Bus()
    b.write(A_CLK, enc.clkdiv(8, 0))
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.idle(1)
    assert divider(b)[0] == 8
    b.write(A_CLK, enc.clkdiv(2, 0))  # no restart
    b.idle(1)
    # The counter is still counting down the old period (7, then 6, ...), and the
    # reload at cnt == 1 picks the new value up.
    assert divider(b)[0] == 6
    for _ in range(6):
        b.step()
    assert divider(b)[0] == 2  # the reload used the new INT = 2


def test_two_engines_restarted_together_stay_in_lockstep():
    b = Bus()
    for e in (0, 1):
        b.write(enc.a_sm(e, 0), enc.clkdiv(4, 0x40))
    b.write(A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    for _ in range(20):
        assert divider(b, 0) == divider(b, 1)
        b.step()


def test_two_engines_with_different_divisors_run_at_different_rates():
    b = Bus()
    b.write(enc.a_sm(0, 0), enc.clkdiv(1, 0))
    b.write(enc.a_sm(1, 0), enc.clkdiv(4, 0))
    b.write(A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    b.idle(1)
    sm0 = sm1 = 0
    for _ in range(12):
        sm0 += sm_cycle_now(b, 0)
        sm1 += sm_cycle_now(b, 1)
        b.step()
    assert sm0 == 12 and sm1 < sm0


def test_sm_enable_is_per_engine():
    b = Bus()
    b.load_imem(0, [set_(SET_X, 1), enc.jmp(JMP_ALWAYS, 0)])
    b.load_imem(1, [set_(enc.SET_Y, 1), enc.jmp(JMP_ALWAYS, 0)])
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.cfg(1, clkdiv_v=enc.clkdiv(1, 0))
    b.write(A_CTRL, enc.ctrl(sm_enable=0b10, clkdiv_restart=0b10))
    b.idle(3)
    for _ in range(3):
        b.step()
    assert b.state(0)["x"] == 0  # engine 0 never ran
    assert b.state(1)["y"] == 1  # engine 1 did


def test_disabling_keeps_the_state_and_re_enabling_continues():
    """`Disabling keeps all state (PC, delay, stall, latch). Re-enabling continues
    from that state.`"""
    b = setup(imem=[set_(SET_X, 1, delay=3), set_(SET_X, 2)])
    st = execs(b, 1)[0]
    assert st["pc"] == 1 and st["delay_cnt"] == 3
    b.write(A_CTRL, 0)  # disable: takes effect from the next cycle, so the disable
    # cycle itself is still an SM cycle and decrements the counter to 2
    assert b.state(0)["delay_cnt"] == 2
    before = dict(b.state(0))
    b.idle(5)
    assert b.state(0) == before  # nothing moved while disabled
    b.write(A_CTRL, enc.ctrl(sm_enable=1))
    # The delay resumes where it left off, then IMEM[1] runs.
    sts = execs(b, 3)
    assert [s["delay_cnt"] for s in sts] == [1, 0, 0]
    assert sts[-1]["x"] == 2


def test_imem_writes_of_a_running_engine_are_allowed():
    b = Bus()
    b.cfg(0, clkdiv_v=enc.clkdiv(2, 0))
    b.load_imem(0, [enc.jmp(JMP_ALWAYS, 0)])
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    b.idle(3)
    assert b.state(0)["pc"] == 0  # it is running `jmp 0`
    # Write a word into IMEM while the engine runs, then send it there with a forced JMP.
    h = b.cycle
    b.write(enc.a_imem(0, 4), set_(SET_X, 0x0C))
    assert b.chip.eng[0].imem[4] == set_(SET_X, 0x0C)
    b.force(0, enc.jmp(JMP_ALWAYS, 4))
    # A forced instruction runs in the cycle after its write: the PC reaches 4 and IR
    # holds IMEM[4] from the end of that cycle. With CLKDIV = 2.0 the next SM cycle
    # executes it, which is T_IMEM = 2 core cycles after the IMEM write plus the JMP.
    for _ in range(4):
        b.step()
        assert b.state(0)["pc"] == 4 or b.state(0)["x"] == 0x0C
    assert b.state(0)["x"] == 0x0C  # executed
    assert b.cycle - h >= 4


def test_reset_returns_the_chip_to_its_power_on_state():
    b = Bus()
    b.load_imem(0, [set_(SET_X, 1)])
    b.write(A_CLK, enc.clkdiv(9, 3))
    b.write(A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    poke(b, 0, rx=[1, 2])
    b.idle(4)
    assert b.state(0)["x"] == 1
    b.reset()
    assert b.state(0) == {
        "pc": 0,
        "ir": 0,
        "x": 0,
        "y": 0,
        "osr": 0,
        "isr": 0,
        "osr_cnt": 32,
        "isr_cnt": 0,
        "delay_cnt": 0,
        "stalled": 0,
        "latch": 0,
        "latch_pend": 0,
        "latch_host": 0,
        "forced": 0,
        "tx_level": 0,
        "rx_level": 0,
    }
    assert b.chip.pin_out == 0 and b.chip.pin_oe == 0x3E000
    assert (b.chip.irq0, b.chip.irq1) == (0, 0)
    assert divider(b) == (1, 0, 0)
    assert b.read(A_CTRL) == 0
    assert b.read(A_CLK) == 0x00010000  # the config registers reset too
    assert b.chip.eng[0].imem == [0] * 32


def test_reset_resets_the_input_pipeline_and_the_pin_registers():
    b = Bus()
    # SET_COUNT is three bits, so 7 pins is the widest window a SET can write.
    b.cfg(0, pinctrl_v=enc.pinctrl(set_base=0, set_count=7))
    b.load_imem(0, [set_(enc.SET_PINS, 0x1F)])
    b.start(0)
    execs(b, 1)
    assert b.chip.pin_out == 0x1F
    b.reset()
    assert b.chip.pin_out == 0
    b.cfg(0, pinctrl_v=enc.pinctrl(set_base=0, set_count=7))
    b.load_imem(0, [set_(enc.SET_PINS, 0x1F)])
    b.start(0)
    b.idle(4)
    assert b.chip.pin_out == 0x1F


def test_state_keys_are_exactly_the_ones_the_api_lists():
    b = Bus()
    assert list(b.state(0)) == [
        "pc",
        "ir",
        "x",
        "y",
        "osr",
        "isr",
        "osr_cnt",
        "isr_cnt",
        "delay_cnt",
        "stalled",
"latch",
            "latch_pend",
            "latch_host",
            "forced",
            "tx_level",
        "rx_level",
    ]
    assert b.state(0) == b.state(1)  # two engines, two states


def test_two_engines_have_independent_imem_and_state():
    b = Bus()
    for e in (0, 1):
        b.cfg(e, clkdiv_v=enc.clkdiv(1, 0))
    b.load_imem(0, [set_(SET_X, 1), set_(SET_X, 2), enc.jmp(JMP_ALWAYS, 0)])
    b.load_imem(1, [set_(enc.SET_Y, 3), enc.jmp(JMP_ALWAYS, 0)])
    b.write(A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    b.idle(1)
    while not (sm_cycle_now(b, 0) and sm_cycle_now(b, 1)):
        b.step()
    for _ in range(2):
        b.step()
    assert (b.state(0)["x"], b.state(1)["y"]) == (2, 3)
    assert b.state(0)["y"] == 0 and b.state(1)["x"] == 0
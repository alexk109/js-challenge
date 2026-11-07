"""EXEC and host-forced instructions (ISA `### EXEC`, `### Reference step`).

Covers OUT EXEC / MOV EXEC, the `SMx_INSTR` host latch, `EXEC_STALLED`, the delay
rules for EXEC'd and forced instructions, the host-wins rule for the latch, and the
side-set rule for a forced instruction that completes.
"""

import pytest

from verif.iss_b import enc
from verif.iss_b.enc import execs, jmp, mov, out, poke, set_, setup, sm_cycle_now, wait_
from verif.iss_b.enc import JMP_ALWAYS, MOVSRC_X, OUT_EXEC, OUT_NULL, OUT_PC
from verif.iss_b.enc import MOV_EXEC, SET_PINS, SET_X, SET_Y, WAIT_GPIO

EXEC_STALLED = 1 << 31


def execctrl(e: int = 0) -> int:
    return enc.a_sm(e, 1)


# ------------------------------------------------------------------ OUT/MOV EXEC


def test_out_exec_runs_its_executee_on_the_next_sm_cycle():
    b = setup(imem=[out(OUT_EXEC, 16), set_(SET_X, 1)])
    poke(b, 0, osr=enc.set_(SET_X, 0x15))
    sts = execs(b, 3)
    assert sts[0]["latch"] == enc.set_(SET_X, 0x15) and sts[0]["latch_pend"] == 1
    assert sts[0]["pc"] == 1  # the OUT EXEC advances the PC as normal
    # The executee runs from the latch and does not advance the PC.
    assert sts[1]["x"] == 0x15 and sts[1]["pc"] == 1
    assert sts[2]["pc"] == 2  # then IMEM[1]


def test_mov_exec_is_the_same_except_for_the_value_source():
    b = setup(imem=[mov(MOV_EXEC, MOVSRC_X), set_(SET_X, 1)])
    poke(b, 0, x=enc.set_(SET_X, 9))
    sts = execs(b, 3)
    assert sts[0]["latch"] == enc.set_(SET_X, 9) and sts[0]["latch_pend"] == 1
    assert sts[1]["x"] == 9 and sts[1]["pc"] == 1


def test_exec_ers_load_no_delay_but_advance_the_pc():
    """`Its delay field is ignored`: with SC = 0 the field is the whole 5-bit delay."""
    words = {"out exec": out(OUT_EXEC, 16, delay=7), "mov exec": mov(MOV_EXEC, MOVSRC_X, delay=7)}
    for name, word in words.items():
        b = setup(imem=[word, set_(SET_X, 1)])
        poke(b, 0, osr=enc.NOP, x=enc.NOP)
        sts = execs(b, 2)
        assert sts[0]["delay_cnt"] == 0, name
        assert sts[0]["pc"] == 1, name


def test_an_exec_er_may_execute_another_exec():
    """`Any instruction may be executed this way, including another EXEC.`"""
    inner = mov(MOV_EXEC, MOVSRC_X)
    b = setup(imem=[out(OUT_EXEC, 16), set_(SET_X, 1)])
    poke(b, 0, osr=inner, x=out(OUT_NULL, 32))
    sts = execs(b, 3)
    assert sts[0]["latch"] == inner
    assert sts[1]["latch"] == out(OUT_NULL, 32)  # the inner EXEC loaded the latch
    assert sts[1]["pc"] == 1  # and the PC still did not advance
    assert sts[2]["pc"] == 1


def test_an_executee_that_writes_the_pc_moves_it():
    b = setup(imem=[out(OUT_EXEC, 16), set_(SET_X, 1)])
    poke(b, 0, osr=enc.jmp(JMP_ALWAYS, 20))  # a taken JMP as the executee
    sts = execs(b, 2)
    assert sts[1]["pc"] == 20


def test_an_executee_that_stalls_is_retried_from_the_latch():
    b = setup(imem=[out(OUT_EXEC, 16), set_(SET_X, 1)])
    poke(b, 0, osr=wait_(WAIT_GPIO, 0, pol=1))  # the pin is low: it stalls
    sts = execs(b, 4)
    assert sts[1]["latch"] == wait_(WAIT_GPIO, 0, pol=1)
    assert sts[1]["latch_pend"] == 1 and sts[1]["stalled"] == 1
    assert sts[1]["pc"] == 1  # the OUT EXEC had already advanced the PC
    assert sts[2]["stalled"] == 1 and sts[2]["pc"] == 1  # retried, still stalled
    assert sts[3]["stalled"] == 1
    assert b.read(execctrl()) & EXEC_STALLED == 0  # latch_host is 0 for an EXEC


def test_a_host_instruction_that_stalls_shows_exec_stalled_and_keeps_the_latch():
    b = setup(imem=[set_(SET_X, 0)])
    poke(b, 0, x=0xAB)
    b.force(0, wait_(WAIT_GPIO, 0, pol=1))  # the pin is low, so it stalls
    sts = execs(b, 2)
    assert sts[0]["stalled"] == 1
    assert sts[0]["latch"] == wait_(WAIT_GPIO, 0, pol=1) and sts[0]["latch_pend"] == 1
    assert sts[0]["pc"] == 0  # a forced instruction does not advance the PC
    assert b.read(execctrl()) & EXEC_STALLED  # latch_pend & latch_host
    # The host clears it with a NOP, as the regmap's drain recipe says.
    b.force(0, enc.NOP)
    sts = execs(b, 2)
    assert sts[0]["pc"] == 0  # the forced NOP retired without advancing the PC
    assert sts[1]["pc"] == 1  # and IMEM[0] runs normally again


# ------------------------------------------------------------- host SMx_INSTR


def test_host_instr_runs_in_the_cycle_after_the_write_whatever_the_divider_says():
    """`SMx_INSTR` executes in cycle `h+1` regardless of `SM_ENABLE` and `en`."""
    b = setup(imem=[set_(SET_X, 0)], restart=1)
    b.write(enc.A_CTRL, 0)  # the engine is disabled
    poke(b, 0, x=0x33)
    b.force(0, set_(SET_X, 0x0A))
    assert b.state(0)["x"] == 0x33  # the write only set the latch
    b.step()
    assert b.state(0)["x"] == 0x0A  # it ran, with the engine disabled


def test_a_forced_instruction_does_not_advance_the_pc_or_touch_the_delay():
    b = setup(imem=[set_(SET_X, 0xFF, delay=3)], restart=1)
    poke(b, 0, x=0x33, pc=7, delay=2)
    b.force(0, set_(SET_Y, 0x05))
    st = execs(b, 1)[0]
    assert st["y"] == 0x05
    assert st["pc"] == 7  # no PC change
    assert st["delay_cnt"] == 2  # the delay in progress resumes, untouched


def test_a_forced_instruction_ignores_its_own_delay_field():
    b = setup(imem=[set_(SET_X, 0)], restart=1)
    poke(b, 0, delay=0)
    b.force(0, set_(SET_Y, 0x05, delay=7))
    sts = execs(b, 2)
    assert sts[0]["y"] == 0x05
    assert sts[0]["delay_cnt"] == 0  # the forced delay field is not loaded
    assert sts[1]["pc"] == 1  # and the engine's PC moved on: the force is over


def test_a_forced_instruction_does_not_consume_the_delay_counter():
    """`a forced cycle never counts as a delay cycle`."""
    b = setup(imem=[set_(SET_X, 0xEE, delay=4)], restart=1)
    poke(b, 0, delay=2)
    b.force(0, set_(SET_Y, 0x05))
    sts = execs(b, 1)
    assert sts[0]["delay_cnt"] == 2  # neither decremented nor reloaded
    # The engine resumes its delay where it left off.
    assert execs(b, 1)[0]["delay_cnt"] == 1


def test_a_forced_instruction_may_write_the_pc():
    b = setup(imem=[set_(SET_X, 0)], restart=1)
    b.force(0, jmp(JMP_ALWAYS, 11))
    assert execs(b, 1)[0]["pc"] == 11
    b = setup(imem=[set_(SET_X, 0)], restart=1)
    poke(b, 0, osr=13)
    b.force(0, out(OUT_PC, 5))
    assert execs(b, 1)[0]["pc"] == 13


def test_a_host_write_wins_the_latch_against_an_out_exec_in_the_same_cycle():
    """`If the host writes SMx_INSTR in the same cycle as the engine executes an
    OUT/MOV EXEC, the host's value wins the latch.`"""
    b = setup(imem=[out(OUT_EXEC, 16), set_(SET_X, 0)])
    poke(b, 0, osr=set_(SET_Y, 0x01))
    # Step to the SM cycle whose instruction is the OUT EXEC, and do the host write
    # in that same cycle.
    while not (sm_cycle_now(b) and b.state(0)["ir"] == out(OUT_EXEC, 16)):
        b.step()
    b.step(bus=("w", enc.a_sm(0, 4), set_(SET_X, 0x02)))
    assert b.state(0)["latch"] == set_(SET_X, 0x02)  # the host's value won the latch
    b.step()
    st = b.state(0)
    assert st["x"] == 0x02  # the host's instruction ran, not the engine's
    assert st["y"] == 0


def test_a_forced_instruction_that_completes_lets_a_stalled_imem_one_retry():
    """`a stalled IMEM instruction it interrupted writes its side-set again`."""
    prog = [
        wait_(WAIT_GPIO, 0, pol=1, side=0b1, sc=1),
        set_(SET_PINS, 0b1, side=0b1, sc=1),
    ]
    b = setup(imem=prog, pinctrl_v=enc.pinctrl(sideset_count=1, sideset_base=3))
    # IMEM[0] stalls on the low pin, and writes side-set to pin 3 in its first cycle.
    assert execs(b, 1)[0]["stalled"] == 1
    assert b.chip.pin_out & (1 << 3) == 1 << 3
    # A stalled instruction does not write side-set again while it stays stalled.
    assert execs(b, 2)[0]["stalled"] == 1
    assert b.chip.pin_out & (1 << 3) == 1 << 3

    # Force an instruction in the cycle before the pin reaches IN_VEC, so it
    # interrupts the stalled WAIT.
    enc.settle(b, 1, n=2)
    b.force(0, set_(SET_X, 0x07))
    sts = execs(b, 2)
    assert sts[0]["x"] == 0x07 and sts[0]["stalled"] == 0  # the forced SET completed
    # Now the pin is high in IN_VEC, so the retried WAIT completes and, because
    # `stalled` was cleared by the forced cycle, writes its side-set again.
    assert sts[1]["pc"] == 1 and sts[1]["stalled"] == 0
    assert b.read(execctrl()) & EXEC_STALLED == 0


def test_the_host_replaces_a_stuck_latched_instruction():
    b = setup(imem=[set_(SET_X, 0)])
    poke(b, 0, x=0x33)
    b.force(0, wait_(WAIT_GPIO, 0, pol=1))  # stuck: the pin is low
    assert execs(b, 1)[0]["stalled"] == 1
    b.force(0, set_(SET_Y, 0x04))
    sts = execs(b, 1)
    assert sts[0]["y"] == 0x04 and sts[0]["stalled"] == 0
    assert b.read(execctrl()) & EXEC_STALLED == 0


def test_sm_restart_clears_a_stuck_forced_instruction():
    b = setup(imem=[set_(SET_X, 0)])
    poke(b, 0, x=0x33)
    b.force(0, wait_(WAIT_GPIO, 0, pol=1))
    assert execs(b, 1)[0]["latch_pend"] == 1
    b.write(enc.A_CTRL, enc.ctrl(sm_enable=1, sm_restart=1))
    assert b.state(0)["latch_pend"] == 0
    assert b.read(execctrl()) & EXEC_STALLED == 0


@pytest.mark.parametrize("word", [set_(SET_X, 3), out(OUT_NULL, 32), mov(MOV_EXEC, MOVSRC_X)])
def test_any_instruction_can_be_forced(word):
    b = setup(imem=[set_(SET_X, 0)], restart=1)
    poke(b, 0, x=0x77, osr=0x88)
    b.force(0, word)
    st = execs(b, 1)[0]
    assert st["pc"] == 0  # never advances the PC
    assert st["stalled"] == 0
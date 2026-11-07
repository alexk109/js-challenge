"""Instruction-level tests of the ISA extensions in the parametric ISS and the assembler (tools/bench/sim/ISA_EXT.md)."""
import pytest

from asm.pioasm import AsmError, assemble
from tools.bench.isa_fw.rig import Rig, asm
from tools.bench.sim.pio_iss_p import Params, chip_class


# ---- assembler ----------------------------------------------------------------------------------
@pytest.mark.parametrize("t", ["xor x, y", "jmpc pin_rise 0", "irq 1", "wait 1 irq 2"])
def test_default_assembler_rejects_the_extensions(t):
    with pytest.raises(AsmError):
        assemble(".program p\n" + t)


def test_cond_and_irq_cannot_be_combined():
    with pytest.raises(AsmError):
        assemble(".program p\nirq 1", ("cond", "irq"))
    with pytest.raises(ValueError):
        Params(isa_cond=True, isa_irq=True)


def test_encodings():
    w = lambda t, isa: assemble(".program p\n" + t, isa)[0].instrs[0]
    assert w("xor x, isr", ("xor",)) == 0xA000 | (1 << 5) | (3 << 3) | 6
    assert w("xor osr, y", ("xor",)) == 0xA000 | (7 << 5) | (3 << 3) | 2
    assert w("jmpc pin_rise 5", ("cond",)) == 0xC000 | (4 << 5) | 5
    assert w("jmpc x_eq_y 0 [3]", ("cond",)) == 0xC000 | (3 << 8)
    assert w("irq 3", ("irq",)) == 0xC000 | 3
    assert w("irq wait 2 rel", ("irq",)) == 0xC000 | (1 << 5) | 0x12
    assert w("irq clear 7", ("irq",)) == 0xC000 | (1 << 6) | 7
    assert w("wait 1 irq 4", ("irq",)) == 0x2000 | (1 << 7) | (2 << 5) | 4
    with pytest.raises(AsmError):
        assemble(".program p\nxor pc, x", ("xor",))


# ---- helpers ------------------------------------------------------------------------------------
def run_prog(text, isa, n=60, **kw):
    rig = Rig(asm(".program p\n" + text, isa), isa, **kw)
    rig.go()
    rig.run(n)
    return rig


def test_mvp_chip_treats_op11_and_110_as_nop():
    prog = assemble(".program p\nset x, 5\nxor x, x\nirq 1\nmov y, x\n", ("xor", "irq"))[0]
    r = Rig(prog, ())                      # assembled with the extensions, run on the MVP chip
    r.go()
    r.run(40)
    assert r.eng.x == 5 and r.eng.y == 5


# ---- xor ----------------------------------------------------------------------------------------
def test_xor_x_y_isr_osr():
    r = run_prog("set x, 5\nset y, 3\nxor x, y\nmov isr, x\nset y, 1\nxor y, isr\nmov osr, y\nset x, 7\nxor osr, x\n"
                 "xor isr, osr\nl: jmp l\n", ("xor",), n=80)
    # x=5^3=6, isr=6, y=1^6=7, osr=7, x=7, osr=7^7=0, isr=6^0=6
    assert (r.eng.x, r.eng.y, r.eng.osr, r.eng.isr) == (7, 7, 0, 6)


def test_xor_resets_the_osr_count_like_mov():
    r = run_prog("set x, 1\nxor osr, x\nl: jmp l\n", ("xor",), n=12)
    assert r.eng.osr_cnt == 0


def test_xor_with_pc_destination_is_a_nop():
    prog = assemble(".program p\nset x, 3\nnop\nl: jmp l\n", ("xor",))[0]
    prog.instrs[1] = 0xA000 | (5 << 5) | (3 << 3) | 1            # xor pc, x: undefined -> NOP
    r = Rig(prog, ("xor",))
    r.go()
    r.run(30)
    assert r.eng.x == 3 and r.eng.pc == 2


# ---- cond ---------------------------------------------------------------------------------------
@pytest.mark.parametrize("cond,xv,yv,jumps", [("x_eq_y", 4, 4, True), ("x_eq_y", 4, 5, False),
                                              ("x_lt_y", 3, 4, True), ("x_lt_y", 4, 4, False),
                                              ("y_lt_x", 5, 4, True), ("y_lt_x", 4, 5, False)])
def test_jmpc_register_conditions(cond, xv, yv, jumps):
    r = run_prog(f"set x, {xv}\nset y, {yv}\njmpc {cond} hit\nset x, 30\nhit: l: jmp l\n", ("cond",), n=30)
    assert (r.eng.x == xv) == jumps


def test_jmpc_pin_low():
    text = ".program p\nnop [6]\njmpc pin_low hit\nset x, 30\nhit: set y, 9\nl: jmp l\n"
    r = Rig(asm(text, ("cond",)), ("cond",), jmp_pin=3)
    r.go()
    r.run(20)
    assert r.eng.x == 0 and r.eng.y == 9                          # pin 3 is 0: taken
    r = Rig(asm(text, ("cond",)), ("cond",), jmp_pin=3)
    r.stim = lambda t: 8
    r.go()
    r.run(20)
    assert r.eng.x == 30                                          # high: not taken


def test_jmpc_osre_and_isr_full():
    text = "set x, 1\nmov osr, x\njmpc osre a\nset y, 1\na: l: jmp l\n"
    assert run_prog(text, ("cond",), n=30).eng.y == 1             # mov osr resets the count: not empty
    r = run_prog("jmpc osre a\nset y, 1\na: set x, 2\nl: jmp l\n", ("cond",), n=30)   # reset count 32: empty
    assert r.eng.y == 0 and r.eng.x == 2
    r = run_prog("in null, 8\njmpc isr_full a\nset y, 1\na: l: jmp l\n", ("cond",), n=30)
    assert r.eng.y == 1                                           # 8 < PUSH_THRESH 32: not full


def test_pin_edge_latch_is_set_by_an_edge_and_cleared_by_the_test():
    text = ".program p\nl: jmpc pin_rise got\njmp l\ngot: set x, 7\nl2: jmpc pin_rise bad\njmp l2\nbad: set y, 1\n"
    r = Rig(asm(text, ("cond",)), ("cond",), jmp_pin=0)
    r.stim = lambda t: 1 if t >= 60 else 0                        # one rising edge
    r.go()
    r.run(120)
    assert r.eng.x == 7 and r.eng.y == 0                          # seen once, then cleared by the test


def test_pin_fall_latch_catches_a_pulse_shorter_than_the_poll():
    text = ".program p\nl: jmpc pin_fall got\njmp l\ngot: set x, 7\nend: jmp end\n"
    r = Rig(asm(text, ("cond",)), ("cond",), jmp_pin=0, clkdiv=4)
    r.stim = lambda t: 0 if t == 60 else 1                        # a one-core-cycle low pulse
    r.go()
    r.run(200)
    assert r.eng.x == 7
    # ... where polling `jmp pin` at the same clkdiv misses it
    poll = ".program p\nl: jmp pin l\nset x, 7\nend: jmp end\n"
    r = Rig(asm(poll), (), jmp_pin=0, clkdiv=4)
    r.stim = lambda t: 0 if t == 60 else 1
    r.go()
    r.run(200)
    assert r.eng.x == 0


# ---- irq ----------------------------------------------------------------------------------------
def test_irq_set_clear_and_wait_irq():
    r = run_prog("irq 3\nirq 5\nirq clear 3\nl: jmp l\n", ("irq",), n=20)
    assert r.chip.irq_flags == 1 << 5
    r = run_prog("irq 2\nwait 1 irq 2\nset x, 4\nwait 1 irq 2\nset y, 4\nl: jmp l\n", ("irq",), n=40)
    # flag 2 is raised, the first wait passes and clears it, the second waits forever
    assert r.eng.x == 4 and r.eng.y == 0 and r.chip.irq_flags == 0


def _two_engine_chip(prog0, prog1):
    c = chip_class(Params(isa_irq=True))()
    for base, text in ((0x12, prog0), (0x52, prog1)):
        for k, w in enumerate(assemble(".program p\n" + text, ("irq",))[0].instrs):
            c.step(0, ("w", base + k, w))
    for e in (0, 1):
        c.step(0, ("w", 0x32 + 6 * e + 4, 0))                     # force jmp 0
    c.step(0, ("w", 0, 3))
    return c


def test_irq_wait_stalls_until_another_engine_clears():
    c = _two_engine_chip("irq wait 1\nset x, 9\nl: jmp l\n", "nop [20]\nwait 1 irq 1\nset y, 6\nl: jmp l\n")
    for _ in range(10):
        c.step(0)
    assert c.engines[0].x == 0 and c.irq_flags & 2               # engine 0 waits on its own raised flag
    for _ in range(60):
        c.step(0)
    assert c.engines[0].x == 9 and c.engines[1].y == 6 and c.irq_flags == 0


def test_irq_rel_adds_the_engine_number():
    c = _two_engine_chip("irq 1 rel\nl: jmp l\n", "irq 1 rel\nl: jmp l\n")
    for _ in range(10):
        c.step(0)
    assert c.irq_flags == (1 << 1) | (1 << 2)


def test_wait_irq_is_hardwired_on_the_mvp():
    prog = assemble(".program p\nwait 0 irq 1\nset x, 1\nl: jmp l\n", ("irq",))[0]
    r = Rig(prog, ())
    r.go()
    r.run(20)
    assert r.eng.x == 1                                           # wait 0 irq passes at once on the MVP


# ---- added after review ---------------------------------------------------------------------------
def test_jmpc_isr_full_taken_at_push_thresh():
    # in null, 32 fills the ISR (count 32 >= PUSH_THRESH 32); autopush is off so the count stays
    r = run_prog("in null, 32\njmpc isr_full a\nset y, 1\na: l: jmp l\n", ("cond",), n=30)
    assert r.eng.y == 0


def test_pin_rise_latch_is_cleared_by_sm_restart():
    from tools.bench.sim.pio_iss_p import A_CTRL
    text = ".program p\nnop [10]\nl: jmpc pin_rise got\njmp l\ngot: set x, 7\nend: jmp end\n"
    r = Rig(asm(text, ("cond",)), ("cond",), jmp_pin=0)
    r.stim = lambda t: 1 if t >= 20 else 0               # the rise happens while the engine is in its nop delay
    r.go()
    r.run(15)
    assert r.eng.jp_rise == 1                            # latched, not yet tested
    r.w(A_CTRL, 1 | (1 << 4))                            # SM_RESTART
    assert r.eng.jp_rise == 0 and r.eng.jp_prev is None


def test_xor_with_pins_and_status_sources():
    # PINS: the rotated, unmasked IN window; STATUS: all ones when TX level < N (empty TX FIFO, N reset = 0 -> 0)
    r = Rig(asm(".program p\nset x, 1\nxor x, pins\nl: jmp l\n", ("xor",)), ("xor",), in_base=0)
    r.stim = lambda t: 0b110
    r.go()
    r.run(30)
    assert r.eng.x == 1 ^ 0b110
    r = Rig(asm(".program p\nset x, 5\nxor x, status\nl: jmp l\n", ("xor",)), ("xor",))
    r.w(0x32 + 1, (r.r(0x32 + 1) & ~0xF) | 2)           # EXECCTRL.STATUS_N = 2: TX level 0 < 2 -> STATUS = all ones
    r.go()
    r.run(30)
    assert r.eng.x == 5 ^ 0xFFFFFFFF

"""Timing: divider, IMEM write latency, enable/restart, stall + delay + side-set, EXEC/force
(docs/mvp-isa.md `## Timing`, `### Stalls, delay and side-set, summarised`, `### EXEC`,
`### Enable, restart`)."""
import pytest

from iss.pio_iss import Engine
from iss.testkit import (CTRL, FDEBUG, H, NOP, TXF, clkdiv, execctrl, imem_addr, in_, jmp,
                         mov, out, pinctrl, pull, push, set_, shiftctrl, sm, wait)

M = 0xFFFF_FFFF


@pytest.fixture
def h():
    return H()


def sm_cycles(h, e, n_cycles):
    """Cycles (absolute numbers) in which engine e executes a NOP program (PC changes)."""
    out_ = []
    for _ in range(n_cycles):
        c = h.cycle
        pc = h.s(e)["pc"]
        h.tick()
        if h.s(e)["pc"] != pc:
            out_.append(c)
    return out_


# ------------------------------------------------------------------ divider
@pytest.mark.parametrize("div,offsets", [
    (clkdiv(1), [2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    (clkdiv(2), [2, 4, 6, 8, 10]),
    (clkdiv(3), [2, 5, 8, 11]),
    (clkdiv(1, 128), [2, 3, 5, 6, 8, 9, 11]),            # 1.5
    (clkdiv(2, 64), [2, 4, 6, 8, 11, 13, 15, 17, 20]),  # 2.25
])
def test_divider_sequences(h, div, offsets):
    hc = h.run_program(0, [NOP] * 32, div=div)
    got = sm_cycles(h, 0, offsets[-1] + 1)
    assert [c - hc for c in got if c - hc <= offsets[-1]] == offsets


def test_divider_restart_phase_en_flop():
    """Restart in h: en = 0 in h+1, en = 1 in h+2 (divisor 1)."""
    h = H()
    eng = h.c.engines[0]
    assert eng.en == 0          # cycle 0
    h.tick()
    assert eng.en == 1          # CLKDIV = 1.0 gives en = 1 from cycle 1
    h.idle(3)
    h.w(CTRL, 0x100)            # cycle h: restart only
    assert eng.en == 0          # h+1
    h.tick()
    assert eng.en == 1          # h+2


def _divider_en_seq(clk, n):
    eng = Engine(0)
    eng.clkdiv = clk
    eng.divider_next(True)
    seq = []
    for _ in range(n):
        seq.append(eng.en)
        eng.divider_next(False)
    return seq  # seq[k] = en in cycle h+1+k


@pytest.mark.parametrize("integer,frac", [(1, 1), (3, 255), (7, 77), (100, 3), (2, 0)])
def test_divider_long_run_rate(integer, frac):
    n_periods = 512
    seq = _divider_en_seq(clkdiv(integer, frac), n_periods * (integer + 1) + 4)
    ones = [k for k, v in enumerate(seq) if v]
    gaps = {b - a for a, b in zip(ones, ones[1:])}
    assert gaps <= {integer, integer + 1}
    span = ones[256] - ones[0]                 # 256 periods
    assert span == 256 * integer + frac        # exact: the accumulator wraps `frac` times


def test_divider_int_zero_means_65536():
    seq = _divider_en_seq(clkdiv(0, 0), 2 * 65536 + 3)
    ones = [k for k, v in enumerate(seq) if v]
    assert ones[:3] == [1, 1 + 65536, 1 + 2 * 65536]
    seq = _divider_en_seq(clkdiv(0, 128), 3 * 65537 + 3)
    ones = [k for k, v in enumerate(seq) if v]
    # first reload (h+1): acc = 128, no carry; second: acc wraps, period 65537
    assert [b - a for a, b in zip(ones, ones[1:])][:2] == [65536, 65537]


def test_clkdiv_write_picked_up_at_next_reload(h):
    hc = h.run_program(0, [NOP] * 32, div=clkdiv(4))
    h.idle(3)                                  # h+1..h+3; SM cycle at h+2
    assert h.cycle == hc + 4
    h.w(sm(0, "CLKDIV"), clkdiv(2))           # h+4: mid-period (next SM cycle h+6)
    got = sm_cycles(h, 0, 10)
    assert [c - hc for c in got] == [6, 8, 10, 12, 14]


def test_divider_runs_while_disabled(h):
    h.load(0, [NOP] * 32)
    h.w(sm(0, "CLKDIV"), clkdiv(3))
    hc = h.cycle
    h.w(CTRL, 0x100)                           # restart the divider only
    h.idle(4)                                  # en at h+2, h+5 (disabled: nothing happens)
    assert h.s()["pc"] == 0
    h.w(CTRL, 0x001)                           # cycle h+5: enable from h+6
    got = sm_cycles(h, 0, 9)
    assert [c - hc for c in got] == [8, 11, 14]


def test_engines_restarted_together_run_in_lockstep(h):
    h.load(0, [NOP] * 32)
    h.load(1, [NOP] * 32)
    h.w(sm(0, "CLKDIV"), clkdiv(2, 100))
    h.w(sm(1, "CLKDIV"), clkdiv(2, 100))
    h.idle(5)
    h.start(0b11)
    for _ in range(60):
        h.tick()
        assert h.s(0)["pc"] == h.s(1)["pc"]
    assert h.s(0)["pc"] != 0


# ------------------------------------------------------------------ IMEM write latency, IR
def test_imem_write_reaches_execution_at_h_plus_2(h):
    hc = h.run_program(0, [set_("x", 1)], wrap=execctrl(wrap_bottom=0, wrap_top=0))
    h.idle(5)
    assert h.s()["x"] == 1
    c = h.cycle
    h.w(imem_addr(0, 0), set_("x", 2))        # cycle c
    assert h.s()["x"] == 1                    # c executed the old word
    h.tick()                                  # c+1: still the old word
    assert h.s()["x"] == 1
    h.tick()                                  # c+2: new word
    assert h.s()["x"] == 2


def test_smx_instr_read_returns_ir_lagging_one_cycle(h):
    h.load(0, [set_("x", 3)])
    h.idle(1)
    assert h.r(sm(0, "INSTR")) == set_("x", 3)
    c = h.cycle
    h.w(imem_addr(0, 0), set_("y", 4))        # c
    assert h.r(sm(0, "INSTR")) == set_("x", 3)  # c+1: IR = IMEM as in c
    assert h.r(sm(0, "INSTR")) == set_("y", 4)  # c+2


def test_ir_follows_pc_next(h):
    prog = [set_("x", k) for k in range(32)]
    h.load(0, prog)
    h.idle(1)
    h.force(0, jmp(17))
    assert h.s()["ir"] == set_("x", 17)
    assert h.r(sm(0, "ADDR")) == 17


def test_imem_write_to_other_engine_is_private(h):
    h.w(imem_addr(1, 0), set_("x", 7))
    h.idle(1)
    assert h.s(0)["ir"] == 0 and h.s(1)["ir"] == set_("x", 7)


# ------------------------------------------------------------------ wrap and PC
def test_wrap_and_counting_outside_wrap(h):
    prog = [NOP] * 32
    hc = h.run_program(0, prog, wrap=execctrl(wrap_bottom=3, wrap_top=5))
    h.tick()
    pcs = []
    for _ in range(9):
        pcs.append(h.s()["pc"])
        h.tick()
    assert pcs == [0, 1, 2, 3, 4, 5, 3, 4, 5]
    h2 = H()
    h2.run_program(0, prog, wrap=execctrl(wrap_bottom=3, wrap_top=5))
    h2.force(0, jmp(30))
    pcs = []
    for _ in range(6):
        pcs.append(h2.s()["pc"])
        h2.tick()
    assert pcs[:5] == [30, 31, 0, 1, 2]


# ------------------------------------------------------------------ enable / restart
def test_sm_enable_takes_effect_next_cycle_and_disable_keeps_state(h):
    prog = [set_("x", 1, ds=5), set_("x", 2)] + [jmp(2)] * 30
    hc = h.run_program(0, prog)
    h.idle(3)                                 # h+1 .. h+3: set x,1 at h+2, delay 5 loaded
    assert h.s()["delay_cnt"] == 4
    h.w(CTRL, 0)                              # h+4: still runs (delay 4->3), stops from h+5
    assert h.s()["delay_cnt"] == 3
    snap = h.s()
    h.idle(20)
    assert h.s() == snap
    h.w(CTRL, 1)                              # re-enable without restart: resumes
    h.idle(3)
    assert h.s()["x"] == 1 and h.s()["delay_cnt"] == 0
    h.tick()
    assert h.s()["x"] == 2


def test_sm_restart_clears_listed_state_and_keeps_the_rest(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(autopull=0))
    h.set_reg(0, "x", 0x1111)
    h.set_reg(0, "y", 0x2222)
    h.set_reg(0, "osr", 0x3333)
    h.force(0, out("null", 4))
    h.force(0, in_("x", 12))
    h.force(0, jmp(9))
    h.w(TXF[0], 0xAA)
    h.force(0, push())
    h.force(0, in_("y", 5))
    h.w(sm(0, "INSTR"), wait(1, "irq", 0))    # host instruction stuck in the latch
    h.tick()
    before = h.s()
    assert before["latch_pend"] == 1 and before["stalled"] == 1
    assert h.r(sm(0, "EXECCTRL")) >> 31 == 1
    h.w(CTRL, 0x10)
    s = h.s()
    assert s["isr"] == 0 and s["isr_cnt"] == 0 and s["osr_cnt"] == 32
    assert s["delay_cnt"] == 0 and s["stalled"] == 0 and s["latch_pend"] == 0
    for k in ("pc", "x", "y", "osr", "latch", "tx_level", "rx_level"):
        assert s[k] == before[k], k
    assert s["pc"] == 9 and s["tx_level"] == 1 and s["rx_level"] == 1
    assert h.r(sm(0, "EXECCTRL")) >> 31 == 0
    assert h.r(CTRL) == 0                     # SC bits read 0


def test_sm_restart_wins_over_same_cycle_execution(h):
    prog = [in_("x", 4)] + [jmp(0)] * 31
    h.force(0, set_("x", 0xF))
    h.run_program(0, prog, wrap=execctrl(wrap_bottom=0, wrap_top=0), restart=False)
    h.idle(4)
    assert h.s()["isr_cnt"] > 0
    h.w(CTRL, 0x11)                           # engine executes `in` in this cycle too
    assert h.s()["isr"] == 0 and h.s()["isr_cnt"] == 0
    h.tick()
    assert h.s()["isr_cnt"] == 4


def test_ctrl_restart_start_sequence(h):
    """docs/mvp-regmap.md `Start a program`: first instruction in h+2 at the forced JMP."""
    prog = [NOP] * 32
    prog[12] = set_("x", 21)
    h.load(0, prog)
    h.force(0, jmp(12))
    hc = h.start(1)
    assert h.s()["x"] == 0                    # h+1
    h.tick()
    assert h.cycle == hc + 2
    h.tick()
    assert h.s()["x"] == 21


# ------------------------------------------------------------------ stall + delay + side-set
SIDE1 = pinctrl(sideset_count=1, sideset_base=0, set_base=0, set_count=1)


def test_blocking_pull_side_set_once_then_stall_then_delay(h):
    """`pull block side 1 [2]`: side-set in the first SM cycle only; stall + 1 + delay."""
    h.w(sm(1, "PINCTRL"), pinctrl(set_base=0, set_count=1))
    prog = [pull(ds=(1 << 4) | 2), set_("x", 1)] + [jmp(2)] * 30
    hc = h.run_program(0, prog, pins=SIDE1)
    h.tick()                                  # h+1
    h.tick()                                  # h+2: pull stalls, side-set requested
    assert h.c.pin_out & 1 == 1 and h.s()["stalled"] == 1
    h.force(1, set_("pins", 0))               # engine 1 overwrites pin 0
    assert h.c.pin_out & 1 == 0
    h.idle(5)
    assert h.c.pin_out & 1 == 0               # stalled engine 0 does not re-write it
    assert h.r(FDEBUG) == 1 << 24
    c = h.cycle
    h.w(TXF[0], 0x55)                         # c: push; engine pops in c+1
    assert h.s()["pc"] == 0
    h.tick()                                  # c+1: completes, delay 2 loaded
    s = h.s()
    assert s["pc"] == 1 and s["osr"] == 0x55 and s["delay_cnt"] == 2 and s["stalled"] == 0
    h.idle(2)                                 # c+2, c+3: delay
    assert h.s()["x"] == 0
    h.tick()                                  # c+4: set x
    assert h.s()["x"] == 1
    assert h.c.pin_out & 1 == 0


def test_forced_instruction_completing_makes_stalled_instr_resideset(h):
    h.w(sm(1, "PINCTRL"), pinctrl(set_base=0, set_count=1))
    prog = [pull(ds=1 << 4)] + [jmp(1)] * 31
    h.run_program(0, prog, pins=SIDE1)
    h.idle(3)
    assert h.c.pin_out & 1 == 1
    h.force(1, set_("pins", 0))
    h.idle(2)
    assert h.c.pin_out & 1 == 0
    h.w(sm(0, "INSTR"), NOP)                  # forced NOP completes: stalled := 0
    h.tick()                                  # forced NOP executes (side 0 bit: field 0)
    assert h.s()["stalled"] == 0 and h.c.pin_out & 1 == 0
    h.tick()                                  # stalled pull retried: side-set written again
    assert h.c.pin_out & 1 == 1 and h.s()["stalled"] == 1


def test_forced_instruction_does_not_consume_or_load_delay(h):
    prog = [set_("x", 1, ds=5), set_("y", 2)] + [jmp(2)] * 30
    h1 = H()
    hc1 = h1.run_program(0, prog)
    ref = None
    for _ in range(15):
        c = h1.cycle
        h1.tick()
        if h1.s()["y"] == 2 and ref is None:
            ref = c - hc1
    assert ref == 8                           # h+2 + 1 + 5
    hc = h.run_program(0, prog)
    h.idle(3)                                 # h+1..h+3
    h.w(sm(0, "INSTR"), set_("pins", 0, ds=7))  # h+4; executes at h+5 (delay field ignored)
    got = None
    for _ in range(15):
        c = h.cycle
        h.tick()
        if h.s()["y"] == 2 and got is None:
            got = c - hc
    assert got == 9                           # one cycle later: the forced cycle is not a delay cycle


def test_forced_instruction_ignores_divider_and_enable(h):
    h.w(sm(0, "CLKDIV"), clkdiv(1000))
    c = h.cycle
    h.w(sm(0, "INSTR"), set_("x", 9))
    assert h.s()["x"] == 0
    h.tick()                                  # c+1 executes though disabled
    assert h.s()["x"] == 9


def test_side_set_opt_enable_bit(h):
    pins = pinctrl(sideset_count=2, sideset_base=5)
    wrap = execctrl(side_en=1)
    prog = [NOP | ((0b11 << 3) << 8),        # side 1
            NOP,                              # no side-set
            NOP | ((0b10 << 3) << 8),        # side 0
            NOP | ((0b01 << 3) << 8)] + [jmp(4)] * 28   # enable bit 0: no side-set
    h.run_program(0, prog, wrap=wrap, pins=pins)
    h.tick()
    seen = []
    for _ in range(5):
        h.tick()
        seen.append((h.c.pin_out >> 5) & 1)
    assert seen == [1, 1, 0, 0, 0]


def test_side_set_opt_delay_bits(h):
    """SC = 2 with opt: 3 delay bits."""
    pins = pinctrl(sideset_count=2)
    prog = [set_("x", 1, ds=(1 << 4) | 7), set_("x", 2)] + [jmp(2)] * 30
    hc = h.run_program(0, prog, wrap=execctrl(side_en=1), pins=pins)
    h.idle(2)
    assert h.s()["delay_cnt"] == 7
    h.idle(7)
    assert h.s()["x"] == 1
    h.tick()
    assert h.s()["x"] == 2


def test_side_pindir(h):
    pins = pinctrl(sideset_count=1, sideset_base=2)
    prog = [NOP | (1 << 12), NOP] + [jmp(2)] * 30
    h.run_program(0, prog, wrap=execctrl(side_pindir=1), pins=pins)
    h.idle(2)
    assert h.c.pin_oe & 0xFF == 1 << 2 and h.c.pin_out & 0xFF == 0


def test_side_set_beats_set_within_engine(h):
    pins = pinctrl(sideset_count=1, sideset_base=0, set_base=0, set_count=2)
    h.w(sm(0, "PINCTRL"), pins)
    h.force(0, set_("pins", 0b10, ds=1 << 4))   # SET wants pin0 = 0, side-set wants 1
    assert h.c.pin_out & 0b11 == 0b11
    h.force(0, set_("pins", 0b01, ds=0))        # side 0 vs SET 1 on pin 0
    assert h.c.pin_out & 0b11 == 0b00


@pytest.mark.parametrize("sc", [5, 6, 7])
def test_sideset_count_6_7_behave_as_5(h, sc):
    h.w(sm(0, "PINCTRL"), (sc << 29) | (0 << 10) | (0 << 26))
    h.force(0, set_("x", 3, ds=0b10101))
    assert h.c.pin_out & 0x1F == 0b10101 and h.s()["x"] == 3
    prog = [set_("x", 4, ds=0b11111), set_("x", 5)] + [jmp(2)] * 30
    h.run_program(0, prog, pins=(sc << 29))
    h.idle(2)
    assert h.s()["x"] == 4 and h.s()["delay_cnt"] == 0
    h.tick()
    assert h.s()["x"] == 5


def test_side_set_not_written_during_delay(h):
    h.w(sm(1, "PINCTRL"), pinctrl(set_base=0, set_count=1))
    prog = [NOP | (1 << 12) | (5 << 8), NOP] + [jmp(2)] * 30   # side 1 [5]
    h.run_program(0, prog, pins=SIDE1)
    h.idle(2)
    assert h.c.pin_out & 1 == 1
    h.force(1, set_("pins", 0))
    h.idle(2)
    assert h.c.pin_out & 1 == 0 and h.s()["delay_cnt"] > 0


def test_stall_with_host_force_resideset_rules(h):
    """A stalled host-forced instruction side-sets on its forced cycle only, not on retries."""
    h.w(sm(1, "PINCTRL"), pinctrl(set_base=0, set_count=1))
    h.w(sm(0, "PINCTRL"), SIDE1)
    h.w(CTRL, 1)
    h.w(sm(0, "INSTR"), pull(ds=1 << 4))
    h.tick()
    assert h.c.pin_out & 1 == 1 and h.s()["latch_pend"] == 1
    h.force(1, set_("pins", 0))
    h.idle(4)
    assert h.c.pin_out & 1 == 0 and h.s()["latch_pend"] == 1
    assert h.r(sm(0, "EXECCTRL")) >> 31 == 1


# ------------------------------------------------------------------ EXEC
def test_out_exec_runs_executee_next_cycle_with_its_delay(h):
    prog = [out("exec", 16, ds=7), set_("y", 1)] + [jmp(2)] * 30
    h.set_reg(0, "osr", set_("x", 5, ds=3))
    hc = h.run_program(0, prog, restart=False)
    h.tick()                                  # h+1
    h.tick()                                  # h+2: out exec (its delay ignored)
    s = h.s()
    assert s["pc"] == 1 and s["latch_pend"] == 1 and s["delay_cnt"] == 0
    assert s["latch"] == set_("x", 5, ds=3)
    h.tick()                                  # h+3: executee
    s = h.s()
    assert s["x"] == 5 and s["pc"] == 1 and s["delay_cnt"] == 3 and s["latch_pend"] == 0
    h.idle(3)
    assert h.s()["y"] == 0
    h.tick()                                  # h+7
    assert h.s()["y"] == 1


def test_mov_exec_executee_writes_pc(h):
    prog = [mov("exec", "x")] + [NOP] * 31
    h.set_reg(0, "x", jmp(20))
    h.run_program(0, prog, restart=False)
    h.idle(3)
    assert h.s()["pc"] == 20


def test_exec_of_exec(h):
    prog = [mov("exec", "x")] + [NOP] * 31
    h.set_reg(0, "x", mov("exec", "y"))
    h.set_reg(0, "y", set_("x", 17))
    h.run_program(0, prog, restart=False)
    h.idle(2)                                 # h+1, h+2: mov exec, x
    assert h.s()["pc"] == 1
    h.tick()                                  # h+3: mov exec, y (no PC change)
    assert h.s()["pc"] == 1 and h.s()["latch"] == set_("x", 17) and h.s()["latch_pend"]
    h.tick()                                  # h+4: set x, 17
    assert h.s()["x"] == 17 and h.s()["pc"] == 1 and not h.s()["latch_pend"]


def test_exec_executee_stall_retried_from_latch(h):
    prog = [mov("exec", "x"), set_("y", 3)] + [jmp(2)] * 30
    h.set_reg(0, "x", pull())
    h.run_program(0, prog, restart=False)
    h.idle(6)
    s = h.s()
    assert s["pc"] == 1 and s["latch_pend"] == 1 and s["stalled"] == 1
    assert h.r(sm(0, "EXECCTRL")) >> 31 == 0  # not a host instruction
    c = h.cycle
    h.w(TXF[0], 0xBEEF)                       # c; pop in c+1
    h.tick()
    s = h.s()
    assert s["osr"] == 0xBEEF and s["latch_pend"] == 0 and s["pc"] == 1
    h.tick()
    assert h.s()["y"] == 3


def test_exec_executee_with_divider(h):
    """The executee runs on the engine's next SM cycle, not the next core cycle."""
    prog = [mov("exec", "x"), NOP] + [jmp(2)] * 30
    h.set_reg(0, "x", set_("y", 6))
    hc = h.run_program(0, prog, div=clkdiv(3), restart=False)
    h.idle(3)                                 # h+1..h+3 (mov exec at h+2)
    assert h.s()["latch_pend"] == 1
    h.idle(1)                                 # h+4: not an SM cycle
    assert h.s()["y"] == 0
    h.tick()                                  # h+5
    assert h.s()["y"] == 6


def test_host_stalled_instruction_cleared_by_nop_and_retried_on_sm_cycles(h):
    h.w(sm(0, "INSTR"), wait(1, "gpio", 9))
    h.tick()
    assert h.r(sm(0, "EXECCTRL")) >> 31 == 1
    h.w(sm(0, "INSTR"), NOP)
    h.tick()
    assert h.r(sm(0, "EXECCTRL")) >> 31 == 0 and h.s()["latch_pend"] == 0
    # retried on normal SM cycles once enabled; pc stays put; completes when the pin rises
    h.w(sm(0, "INSTR"), wait(1, "gpio", 9))
    h.tick()
    h.w(CTRL, 1)
    h.idle(5)
    assert h.s()["latch_pend"] == 1 and h.s()["pc"] == 0
    h.ext = 1 << 9
    h.idle(4)
    assert h.s()["latch_pend"] == 0 and h.s()["pc"] == 0  # IMEM is all `jmp 0`


def test_host_write_wins_latch_over_same_cycle_out_exec(h):
    prog = [out("exec", 16)] + [jmp(1)] * 31
    h.set_reg(0, "osr", set_("x", 1))
    hc = h.run_program(0, prog, wrap=execctrl(), restart=False)
    h.tick()                                  # h+1
    h.w(sm(0, "INSTR"), set_("y", 2))         # h+2: out exec executes in the same cycle
    assert h.s()["latch"] == set_("y", 2)
    h.tick()                                  # h+3: host instruction
    assert h.s()["y"] == 2
    h.idle(4)
    assert h.s()["x"] == 0                    # the OUT EXEC executee was replaced


def test_forced_out_exec_delay_rules(h):
    """A host-forced MOV EXEC: the executee runs on the next SM cycle and its delay applies."""
    prog = [NOP] * 32
    h.set_reg(0, "x", set_("y", 1, ds=2))
    h.w(CTRL, 1)
    h.force(0, mov("exec", "x"))
    assert h.s()["latch_pend"] == 1
    h.tick()
    assert h.s()["y"] == 1 and h.s()["delay_cnt"] == 2

"""Register map, FIFOs, FSTAT/FDEBUG/FLEVEL, IRQ lines, pins and engine conflicts
(docs/mvp-regmap.md; docs/mvp-isa.md `### Host-to-engine timing`, `## Pins`)."""
import pytest

from iss.testkit import (CTRL, DBG_CFGINFO, DBG_PADOE, DBG_PADOUT, FDEBUG, FLEVEL, FSTAT, H,
                         INTR, IRQ0_INTE, IRQ0_INTF, IRQ0_INTS, IRQ1_INTE, IRQ1_INTF,
                         IRQ1_INTS, NOP, RXF, TXF, execctrl, imem_addr, in_, jmp, mov, out,
                         pinctrl, pull, push, set_, shiftctrl, sm)

M = 0xFFFF_FFFF


@pytest.fixture
def h():
    return H()


# ------------------------------------------------------------------ reset values, access types
RESET_VALUES = {
    CTRL: 0, FSTAT: 0x0300_0300, FDEBUG: 0, FLEVEL: 0, DBG_PADOUT: 0, DBG_PADOE: 0x3E000,
    DBG_CFGINFO: 0x0020_0204, INTR: 0x30, IRQ0_INTE: 0x003, IRQ0_INTF: 0, IRQ0_INTS: 0,
    IRQ1_INTE: 0x010, IRQ1_INTF: 0, IRQ1_INTS: 0x010,
    **{sm(e, "CLKDIV"): 0x0001_0000 for e in (0, 1)},
    **{sm(e, "EXECCTRL"): 0x0001_F000 for e in (0, 1)},
    **{sm(e, "SHIFTCTRL"): 0x000C_0000 for e in (0, 1)},
    **{sm(e, "ADDR"): 0 for e in (0, 1)},
    **{sm(e, "INSTR"): 0 for e in (0, 1)},
    **{sm(e, "PINCTRL"): 0x1400_0000 for e in (0, 1)},
}


def test_reset_values(h):
    for addr, val in RESET_VALUES.items():
        assert h.r(addr) == val, hex(addr)


def test_reset_state():
    from iss.pio_iss import Chip
    c = Chip()
    for e in (0, 1):
        assert c.state(e) == dict(pc=0, ir=0, x=0, y=0, osr=0, isr=0, osr_cnt=32, isr_cnt=0,
                                  delay_cnt=0, stalled=0, latch=0, latch_pend=0,
                                  latch_host=0, forced=0, tx_level=0, rx_level=0)
    assert (c.pin_out, c.pin_oe, c.irq0, c.irq1) == (0, 0x3E000, 0, 0)


def test_reset_method_restores_power_on_state(h):
    h.w(TXF[0], 5)
    h.w(sm(0, "PINCTRL"), 0)
    h.force(0, set_("x", 3))
    h.c.reset()
    assert h.r(FLEVEL) == 0 and h.r(sm(0, "PINCTRL")) == 0x1400_0000 and h.s()["x"] == 0


@pytest.mark.parametrize("addr,stored", [
    (sm(0, "CLKDIV"), 0xFFFF_FF00), (sm(1, "CLKDIV"), 0xFFFF_FF00),
    (sm(0, "EXECCTRL"), 0x7F01_FF9F), (sm(1, "EXECCTRL"), 0x7F01_FF9F),
    (sm(0, "SHIFTCTRL"), 0x3FFF_0000), (sm(1, "SHIFTCTRL"), 0x3FFF_0000),
    (sm(0, "PINCTRL"), M), (sm(1, "PINCTRL"), M),
    (IRQ0_INTE, 0x33), (IRQ1_INTE, 0x33), (IRQ0_INTF, 0x33), (IRQ1_INTF, 0x33),
])
def test_rw_register_masks(h, addr, stored):
    h.w(addr, M)
    assert h.r(addr) == stored
    h.w(addr, 0)
    assert h.r(addr) == 0


def test_ctrl_reads_only_enable(h):
    h.w(CTRL, M)
    assert h.r(CTRL) == 0x3
    h.w(CTRL, 0)
    assert h.r(CTRL) == 0


RESERVED = [0x06, 0x07, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, *range(0x3E, 0x4A), 0x51,
            *range(0x72, 0x80)]


def _snapshot(h):
    regs = {a: h.c.peek(a) for a in RESET_VALUES}
    return regs, [h.s(e) for e in (0, 1)], h.c.pin_out, h.c.pin_oe


def test_reserved_and_write_only_addresses(h):
    before = _snapshot(h)
    for a in RESERVED:
        h.w(a, M)
        assert h.r(a) == 0, hex(a)
    for a in [TXF[0], TXF[1], imem_addr(0, 3), imem_addr(1, 31)]:
        assert h.r(a) == 0
    after = _snapshot(h)
    assert after == before


def test_ro_registers_ignore_writes(h):
    h.w(TXF[0], 1)
    for a in (FSTAT, FLEVEL, DBG_PADOUT, DBG_PADOE, DBG_CFGINFO, INTR, IRQ0_INTS, IRQ1_INTS,
              sm(0, "ADDR"), RXF[0]):
        before = h.c.peek(a)
        h.w(a, M)
        assert h.c.peek(a) == before, hex(a)
    assert h.s()["pc"] == 0 and h.s()["tx_level"] == 1


def test_imem_ignores_upper_bits(h):
    h.w(imem_addr(0, 0), 0xFFFF_0000 | set_("x", 9))
    h.idle(1)
    assert h.s()["ir"] == set_("x", 9)


# ------------------------------------------------------------------ FIFOs and flags
def test_tx_fifo_fill_overflow_levels(h):
    for k in range(4):
        h.w(TXF[1], k)
        assert (h.r(FLEVEL) >> 8) & 0xF == k + 1
    assert h.r(FSTAT) == 0x0100_0300 | (1 << 17)    # TX1 full, TX0 empty, RX both empty
    assert h.r(FDEBUG) == 0
    h.w(TXF[1], 99)                                  # dropped
    assert h.r(FDEBUG) == 1 << 17
    assert (h.r(FLEVEL) >> 8) & 0xF == 4
    for k in range(4):
        h.force(1, pull())
        assert h.s(1)["osr"] == k


def test_rx_read_and_underflow(h):
    assert h.r(RXF[1]) == 0
    assert h.r(FDEBUG) == 1 << 9
    for v in (0xA, 0xB):
        h.set_reg(1, "isr", v)
        h.force(1, push())
    assert h.r(FLEVEL) == 2 << 12
    assert h.r(FSTAT) == 0x0300_0100
    assert h.r(RXF[1]) == 0xA
    assert h.r(RXF[1]) == 0xB
    assert h.r(RXF[1]) == 0
    assert h.r(FLEVEL) == 0


def test_rx_full_flag(h):
    for k in range(4):
        h.force(0, push())
    assert h.r(FSTAT) & 0x3 == 0x1 and (h.r(FSTAT) >> 8) & 3 == 0x2
    assert h.r(FLEVEL) & 0xF0 == 0x40


def test_fdebug_w1c(h):
    h.r(RXF[0])
    h.r(RXF[1])
    for k in range(5):
        h.w(TXF[0], k)
    assert h.r(FDEBUG) == 0x0001_0300
    h.w(FDEBUG, 0x0000_0100)
    assert h.r(FDEBUG) == 0x0001_0200
    h.w(FDEBUG, M)
    assert h.r(FDEBUG) == 0


def test_fdebug_set_and_clear_same_cycle_stays_set(h):
    prog = [pull()] + [jmp(0)] * 31             # stalls every SM cycle: TXSTALL set every cycle
    h.run_program(0, prog)
    h.idle(4)
    assert h.r(FDEBUG) == 1 << 24
    h.w(FDEBUG, 1 << 24)                         # engine sets it in the same cycle
    assert h.r(FDEBUG) == 1 << 24
    h.w(CTRL, 0)                                 # stop the engine (runs this cycle still)
    h.w(FDEBUG, 1 << 24)
    assert h.r(FDEBUG) == 0


def test_host_push_to_full_fifo_dropped_even_if_engine_pops(h):
    for k in range(4):
        h.w(TXF[0], k)
    prog = [pull()] * 32
    hc = h.run_program(0, prog)
    h.tick()                                     # h+1
    h.w(TXF[0], 77)                              # h+2: engine pops; FIFO full at start
    assert h.r(FDEBUG) == 1 << 16
    h.w(CTRL, 0)
    words = [h.s()["osr"]]
    while h.s()["tx_level"]:
        h.force(0, pull())
        words.append(h.s()["osr"])
    assert 77 not in words


def test_engine_cannot_pop_word_pushed_same_cycle(h):
    prog = [pull()] + [jmp(1)] * 31
    h.run_program(0, prog)
    h.idle(4)
    assert h.s()["stalled"] == 1
    c = h.cycle
    h.w(TXF[0], 0x1234)                          # c: engine sees an empty FIFO
    s = h.s()
    assert s["stalled"] == 1 and s["tx_level"] == 1 and s["pc"] == 0
    h.tick()                                     # c+1: pops
    s = h.s()
    assert s["osr"] == 0x1234 and s["tx_level"] == 0 and s["pc"] == 1


def test_push_and_pop_same_cycle_both_happen(h):
    h.w(TXF[0], 1)
    h.w(TXF[0], 2)
    prog = [pull()] + [jmp(1)] * 31
    h.run_program(0, prog)
    h.tick()                                     # h+1
    h.w(TXF[0], 3)                               # h+2: engine pops 1, host pushes 3
    assert h.s()["osr"] == 1 and h.s()["tx_level"] == 2


def test_host_read_empty_rx_while_engine_pushes(h):
    prog = [push()] + [jmp(1)] * 31
    h.set_reg(0, "isr", 0xCAFE)
    h.run_program(0, prog, restart=False)
    h.tick()                                     # h+1
    assert h.r(RXF[0]) == 0                      # h+2: engine pushes in this cycle
    assert h.r(FDEBUG) == 1 << 8
    assert h.r(RXF[0]) == 0xCAFE


def test_rx_push_visibility_exact():
    h = H()
    h.set_reg(0, "isr", 0x77)
    h.w(sm(0, "INSTR"), push())                  # c: write; push executes in c+1
    assert h.r(FLEVEL) == 0                      # c+1: start-of-cycle, push not yet visible
    assert h.r(FLEVEL) == 0x10                   # c+2 >= n+1


def test_tx_push_in_h_poppable_from_h_plus_1(h):
    c = h.cycle
    h.w(TXF[0], 0x42)                            # c
    h.w(sm(0, "INSTR"), pull(block=0))           # c+1 write, executes c+2
    h.tick()
    assert h.s()["osr"] == 0x42


@pytest.mark.parametrize("e", [0, 1])
def test_autopull_basic_and_stall_at_least_one_cycle(h, e):
    """OUT with autopull and an empty OSR stalls one SM cycle even when the FIFO has data."""
    h.w(sm(e, "SHIFTCTRL"), shiftctrl(pull_thresh=8, autopull=1))
    h.w(TXF[e], 0x0000_A5C3)
    prog = [out("x", 8)] + [jmp(1)] * 31
    hc = h.run_program(e, prog, restart=True)    # restart: osr_cnt = 32 (empty)
    h.tick()                                     # h+1
    h.tick()                                     # h+2: stall + refill
    s = h.s(e)
    assert s["pc"] == 0 and s["stalled"] == 1 and s["osr"] == 0xA5C3 and s["osr_cnt"] == 0
    assert h.r(FDEBUG) == 0                      # h+3: out completes
    s = h.s(e)
    assert s["pc"] == 1 and s["x"] == 0xC3 and s["osr_cnt"] == 8


def test_autopull_empty_fifo_sets_txstall(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(autopull=1))
    h.force(0, out("x", 4))
    assert h.s()["latch_pend"] == 1
    assert h.r(FDEBUG) == 1 << 24


def test_autopull_refill_after_out_reaches_threshold(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(pull_thresh=8, autopull=1))
    h.w(TXF[0], 0x11)
    h.w(TXF[0], 0x22)
    h.force(0, pull())                           # osr = 0x11, cnt 0
    h.force(0, out("x", 4))
    assert h.s()["osr_cnt"] == 4 and h.s()["tx_level"] == 1
    h.force(0, out("y", 4))                      # reaches 8: refill in the same cycle
    s = h.s()
    assert s["y"] == 1 and s["osr"] == 0x22 and s["osr_cnt"] == 0 and s["tx_level"] == 0


def test_background_autopull_during_delay_and_other_stalls(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(autopull=1))
    prog = [NOP | (4 << 8)] + [jmp(1)] * 31      # nop [4], osr_cnt stays 32
    hc = h.run_program(0, prog)
    h.idle(3)                                    # h+1..h+3: nop at h+2 (FIFO empty)
    assert h.s()["osr_cnt"] == 32
    h.w(TXF[0], 0x99)                            # h+4 (a delay cycle): FIFO empty at start
    assert h.s()["osr_cnt"] == 32
    h.tick()                                     # h+5: delay cycle, background autopull
    assert h.s()["osr"] == 0x99 and h.s()["osr_cnt"] == 0 and h.s()["delay_cnt"] > 0


def test_background_autopull_not_in_non_sm_cycles(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(autopull=1))
    h.w(TXF[0], 5)
    h.idle(5)
    assert h.s()["osr_cnt"] == 32 and h.s()["tx_level"] == 1
    h.force(0, set_("x", 1))                     # a forced cycle runs it
    assert h.s()["osr"] == 5 and h.s()["tx_level"] == 0


def test_autopush_threshold_32_default(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(autopush=1))
    h.force(0, set_("x", 1))
    for k in range(31):
        h.force(0, in_("x", 1))
    assert h.s()["rx_level"] == 0 and h.s()["isr_cnt"] == 31
    h.force(0, in_("x", 1))
    assert h.s()["rx_level"] == 1 and h.r(RXF[0]) == M


# ------------------------------------------------------------------ IRQ lines
def test_intr_ints_values(h):
    assert h.r(INTR) == 0x30
    h.force(1, push())
    assert h.r(INTR) == 0x32
    assert h.r(IRQ0_INTS) == 0x2 and h.r(IRQ1_INTS) == 0x10
    for k in range(4):
        h.w(TXF[0], k)
    assert h.r(INTR) == 0x22 and h.r(IRQ1_INTS) == 0
    h.w(IRQ1_INTE, 0x20)
    assert h.r(IRQ1_INTS) == 0x20
    h.w(IRQ0_INTF, 0x10)
    assert h.r(IRQ0_INTS) == 0x12


def test_irq_pins_one_cycle_late(h):
    """A FIFO change made at the end of cycle n shows on the pin from cycle n+2."""
    h.tick()
    assert h.c.irq1 == 1 and h.c.irq0 == 0
    h.w(sm(0, "INSTR"), push())                  # executes in n = c+1
    assert h.c.irq0 == 0                         # value in c+1
    h.tick()                                     # n: push at the end of n
    assert h.c.irq0 == 0                         # value in n+1
    h.tick()
    assert h.c.irq0 == 1                         # value in n+2
    h.r(RXF[0])                                  # pop at end of m
    assert h.c.irq0 == 1
    h.tick()
    assert h.c.irq0 == 0


def test_irq1_tracks_tx_not_full(h):
    for k in range(3):
        h.w(TXF[0], k)
    h.w(TXF[0], 3)                               # full at end of m
    assert h.c.irq1 == 1
    h.tick()
    assert h.c.irq1 == 0


def test_irq_intf_forces_line(h):
    h.w(IRQ0_INTF, 0x1)                          # in m
    assert h.c.irq0 == 0
    h.tick()
    assert h.c.irq0 == 1
    h.w(IRQ1_INTE, 0)
    h.tick()
    assert h.c.irq1 == 0


# ------------------------------------------------------------------ pins
def _lockstep(h, p0, p1, pins0=None, pins1=None):
    h.load(0, p0)
    h.load(1, p1)
    for e, p in ((0, pins0), (1, pins1)):
        if p is not None:
            h.w(sm(e, "PINCTRL"), p)
        h.w(sm(e, "EXECCTRL"), execctrl(wrap_bottom=0, wrap_top=0))
    h.idle(2)
    return h.start(0b11)


def test_engine_1_wins_level_conflict(h):
    _lockstep(h, [set_("pins", 1)], [set_("pins", 0)],
              pinctrl(set_base=4, set_count=1), pinctrl(set_base=4, set_count=1))
    h.idle(4)
    assert (h.c.pin_out >> 4) & 1 == 0
    h2 = H()
    _lockstep(h2, [set_("pins", 0)], [set_("pins", 1)],
              pinctrl(set_base=4, set_count=1), pinctrl(set_base=4, set_count=1))
    h2.idle(4)
    assert (h2.c.pin_out >> 4) & 1 == 1


def test_level_and_direction_resolved_separately(h):
    _lockstep(h, [set_("pins", 1)], [set_("pindirs", 1)],
              pinctrl(set_base=3, set_count=1), pinctrl(set_base=3, set_count=1))
    h.idle(4)
    assert (h.c.pin_out >> 3) & 1 == 1 and (h.c.pin_oe >> 3) & 1 == 1


def test_engine_1_set_beats_engine_0_sideset(h):
    _lockstep(h, [NOP | (1 << 12)], [set_("pins", 0)],
              pinctrl(sideset_count=1, sideset_base=2), pinctrl(set_base=2, set_count=1))
    h.idle(4)
    assert (h.c.pin_out >> 2) & 1 == 0


def test_engine_conflict_only_where_both_write(h):
    _lockstep(h, [set_("pins", 0b11)], [set_("pins", 0)],
              pinctrl(set_base=0, set_count=2), pinctrl(set_base=1, set_count=1))
    h.idle(4)
    assert h.c.pin_out & 0b11 == 0b01


def test_unwritten_pins_keep_value(h):
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=0, set_count=5))
    h.force(0, set_("pins", 0b10101))
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=1, set_count=1))
    h.force(0, set_("pins", 1))
    assert h.c.pin_out & 0x1F == 0b10111


def test_input_only_and_phantom_writes_dropped(h):
    h.w(sm(0, "PINCTRL"), pinctrl(out_base=0, out_count=32))
    h.set_reg(0, "osr", M)
    h.force(0, out("pins", 32))
    h.set_reg(0, "osr", M)
    h.force(0, out("pindirs", 32))
    assert h.c.pin_out == 0x3E0FF
    assert h.c.pin_oe == 0x3E0FF
    assert h.r(DBG_PADOUT) == 0x3E0FF
    assert h.r(DBG_PADOE) == 0x3E0FF
    h.set_reg(0, "osr", 0)
    h.force(0, out("pindirs", 32))
    assert h.c.pin_oe == 0x3E000                 # output-only pins stay driven
    assert h.r(DBG_PADOE) == 0x3E000


def test_output_only_pin_reads_back_driven_level_after_t_in(h):
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=15, set_count=1, in_base=15))
    h.ext = 0x3FFFF                             # raw input for pins 13..17 is ignored
    h.w(sm(0, "INSTR"), set_("pins", 1))        # c write, executes c+1, on pad from c+2
    seen = []
    for _ in range(6):
        h.tick()
        seen.append((h.c.last_in_vec >> 15) & 1)
    # cycles c+1..c+6: pad is 1 from c+2, so IN_VEC is 1 from c+5
    assert seen == [0, 0, 0, 0, 1, 1]
    h.force(0, in_("pins", 1))
    assert h.s()["isr"] >> 31 == 1


def test_input_only_pin_read(h):
    h.ext = 1 << 10
    h.idle(3)
    h.force(0, mov("x", "pins"))
    assert h.s()["x"] == 1 << 10


def test_phantom_pins_read_zero(h):
    h.ext = 0x3FFFF
    h.idle(4)
    h.force(0, mov("x", "pins"))
    assert h.s()["x"] == 0x01FFF                # uio + input-only; out-only read 0 (driven 0)


def test_dbg_padout_shows_level_registers(h):
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=5, set_count=5))
    h.force(0, set_("pins", 0b11111))           # pins 5..9: 8, 9 input-only
    assert h.r(DBG_PADOUT) == 0b111 << 5
    assert h.r(DBG_PADOE) == 0x3E000

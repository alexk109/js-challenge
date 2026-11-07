"""Every instruction, source, destination and condition (docs/mvp-isa.md `### Instruction
semantics`). Most cases host-force single instructions on a disabled engine, which executes them
one cycle after the SMx_INSTR write; PC-related cases run from IMEM."""
import pytest

from iss.testkit import (FDEBUG, H, NOP, RXF, TXF, execctrl, in_, irq, jmp, mov, out,
                         pinctrl, pull, push, set_, shiftctrl, sm, wait)

M = 0xFFFF_FFFF


def nolatch(s):
    return {k: v for k, v in s.items() if k not in ("latch", "latch_host")}


def bitrev(v):
    return int(f"{v:032b}"[::-1], 2)


@pytest.fixture
def h():
    return H()


# ------------------------------------------------------------------ JMP
@pytest.mark.parametrize("cond,x,y,taken,x_after,y_after", [
    ("always", 0, 0, True, 0, 0),
    ("not_x", 0, 7, True, 0, 7),
    ("not_x", 5, 7, False, 5, 7),
    ("x_dec", 0, 0, False, M, 0),
    ("x_dec", 1, 0, True, 0, 0),
    ("x_dec", 0x80000000, 0, True, 0x7FFFFFFF, 0),
    ("not_y", 3, 0, True, 3, 0),
    ("not_y", 3, 1, False, 3, 1),
    ("y_dec", 0, 0, False, 0, M),
    ("y_dec", 0, 9, True, 0, 8),
    ("x_ne_y", 3, 3, False, 3, 3),
    ("x_ne_y", 3, 4, True, 3, 4),
    ("x_ne_y", M, 0x7FFFFFFF, True, M, 0x7FFFFFFF),
])
def test_jmp_conditions(h, cond, x, y, taken, x_after, y_after):
    h.set_reg(0, "x", x)
    h.set_reg(0, "y", y)
    assert h.s()["pc"] == 0
    h.force(0, jmp(21, cond))
    s = h.s()
    assert s["pc"] == (21 if taken else 0)
    assert (s["x"], s["y"]) == (x_after, y_after)


@pytest.mark.parametrize("pin", [0, 9, 15, 17])
def test_jmp_pin_uses_absolute_synchronized_input(h, pin):
    h.w(sm(0, "EXECCTRL"), execctrl(jmp_pin=pin))
    h.w(sm(0, "PINCTRL"), pinctrl(in_base=5))  # IN_BASE must not matter
    if pin >= 13:  # output-only pins read back their driven level
        h.w(sm(0, "PINCTRL"), pinctrl(in_base=5, set_base=pin, set_count=1))
        h.force(0, set_("pins", 1))
    else:
        h.ext = 1 << pin
    h.idle(4)
    h.force(0, jmp(9, "pin"))
    assert h.s()["pc"] == 9
    h.force(0, jmp(0))
    h.ext = 0
    if pin >= 13:
        h.force(0, set_("pins", 0))
    h.idle(4)
    h.force(0, jmp(9, "pin"))
    assert h.s()["pc"] == 0


def test_jmp_pin_latency_is_t_in(h):
    """A pad change in cycle c is seen by an instruction executing in c+3, not c+2."""
    h.w(sm(0, "EXECCTRL"), execctrl(jmp_pin=3))
    h.ext = 0
    h.idle(4)
    c = h.cycle
    h.ext = 1 << 3
    h.w(sm(0, "INSTR"), jmp(9, "pin"))     # cycle c, executes in c+1
    h.tick()
    assert h.s()["pc"] == 0
    h.w(sm(0, "INSTR"), jmp(9, "pin"))     # c+2, executes c+3: sees pad from c
    assert h.cycle == c + 3
    h.tick()
    assert h.s()["pc"] == 9


def test_jmp_not_osre(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(pull_thresh=8))
    h.set_reg(0, "osr", 0x1234)            # pull: osr_cnt = 0
    h.force(0, jmp(4, "not_osre"))
    assert h.s()["pc"] == 4
    h.force(0, jmp(0))
    h.force(0, out("null", 7))             # osr_cnt 7 < 8
    h.force(0, jmp(4, "not_osre"))
    assert h.s()["pc"] == 4
    h.force(0, jmp(0))
    h.force(0, out("null", 1))             # osr_cnt 8, not < 8
    assert h.s()["osr_cnt"] == 8
    h.force(0, jmp(4, "not_osre"))
    assert h.s()["pc"] == 0


def test_jmp_not_osre_threshold_zero_means_32(h):
    h.set_reg(0, "osr", 1)
    h.force(0, out("null", 31))
    h.force(0, jmp(4, "not_osre"))
    assert h.s()["pc"] == 4
    h.force(0, out("null", 1))
    h.force(0, jmp(6, "not_osre"))
    assert h.s()["pc"] == 4


def test_jmp_from_imem_taken_and_not_taken_cost_one_cycle(h):
    # 0: jmp 5 ; 5: jmp !x 9 (x = 0, taken) ; 9: jmp x-- 20 (x = 0, not taken) ; 10: set y 3
    prog = [NOP] * 32
    prog[0] = jmp(5)
    prog[5] = jmp(9, "not_x")
    prog[9] = jmp(20, "x_dec")
    prog[10] = set_("y", 3)
    hc = h.run_program(0, prog)
    h.tick()                                # h+1
    trace = []
    for _ in range(5):
        trace.append(h.s()["pc"])
        h.tick()
    assert trace == [0, 5, 9, 10, 11]
    assert h.s()["x"] == M and h.s()["y"] == 3


# ------------------------------------------------------------------ WAIT
def _wait_prog(h, instr, pins=None):
    prog = [instr, set_("x", 1)] + [jmp(2)] * 30
    return h.run_program(0, prog, pins=pins)


@pytest.mark.parametrize("pol", [0, 1])
def test_wait_gpio(h, pol):
    h.ext = (0 if pol else 1) << 4
    h.idle(4)
    hc = _wait_prog(h, wait(pol, "gpio", 4))
    h.idle(10)
    s = h.s()
    assert s["pc"] == 0 and s["stalled"] == 1 and s["x"] == 0
    c = h.cycle
    h.ext = (1 if pol else 0) << 4         # pad changes in cycle c, seen in c+3
    h.idle(3)                               # c, c+1, c+2: still stalled
    assert h.s()["pc"] == 0
    h.tick()                                # c+3: wait completes
    assert h.s()["pc"] == 1 and h.s()["stalled"] == 0
    h.tick()
    assert h.s()["x"] == 1


def test_wait_pin_uses_in_base_mod_32(h):
    h.ext = 0
    _wait_prog(h, wait(1, "pin", 4), pins=pinctrl(in_base=30))   # pin (30+4) mod 32 = 2
    h.idle(6)
    assert h.s()["pc"] == 0
    h.ext = 1 << 2
    h.idle(4)
    assert h.s()["pc"] >= 1


def test_wait_pin_relative(h):
    h.ext = 1 << 11
    h.idle(4)
    _wait_prog(h, wait(1, "pin", 3), pins=pinctrl(in_base=8))
    h.idle(3)
    assert h.s()["pc"] == 2


def test_wait_0_irq_passes_and_wait_1_irq_stalls_forever(h):
    h.force(0, wait(0, "irq", 3))
    assert h.s()["latch_pend"] == 0 and h.s()["stalled"] == 0
    h2 = H()
    _wait_prog(h2, wait(1, "irq", 0))
    h2.idle(200)
    assert h2.s()["pc"] == 0 and h2.s()["stalled"] == 1


@pytest.mark.parametrize("pol", [0, 1])
def test_wait_reserved_source_is_nop(h, pol):
    _wait_prog(h, wait(pol, "reserved", 0))
    h.idle(3)
    assert h.s()["x"] == 1 and h.s()["pc"] == 2


def test_wait_delay_starts_after_condition(h):
    h.ext = 0
    h.idle(4)
    prog = [wait(1, "gpio", 0, ds=3), set_("x", 1)] + [jmp(2)] * 30
    h.run_program(0, prog)
    h.idle(8)
    c = h.cycle
    h.ext = 1
    h.idle(4)                               # wait completes in c+3
    assert h.s()["pc"] == 1 and h.s()["delay_cnt"] == 3
    h.idle(3)                               # 3 delay cycles
    assert h.s()["x"] == 0
    h.tick()
    assert h.s()["x"] == 1


# ------------------------------------------------------------------ IN
@pytest.mark.parametrize("right", [1, 0])
@pytest.mark.parametrize("n", [1, 5, 8, 31, 32])
@pytest.mark.parametrize("src", ["x", "y", "null", "isr", "osr"])
def test_in_sources_and_shift(h, right, n, src):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(in_right=right))
    vals = dict(x=0xDEADBEEF, y=0x0F0F1234, isr=0x12345678, osr=0xCAFEF00D)
    h.set_reg(0, "osr", vals["osr"])
    h.set_reg(0, "x", vals["x"])
    h.set_reg(0, "y", vals["y"])
    h.set_reg(0, "isr", vals["isr"])      # mov isr, osr ... then fix isr
    h.w(TXF[0], vals["isr"])
    h.force(0, pull())
    h.force(0, mov("isr", "osr"))
    h.w(TXF[0], vals["osr"])
    h.force(0, pull())
    h.force(0, in_("null", 3))             # isr_cnt = 3 so saturation shows
    isr0 = h.s()["isr"]
    srcv = {"x": vals["x"], "y": vals["y"], "null": 0, "isr": isr0, "osr": vals["osr"]}[src]
    d = srcv & ((1 << n) - 1)
    if n == 32:
        exp = d
    elif right:
        exp = (isr0 >> n) | (d << (32 - n))
    else:
        exp = ((isr0 << n) | d) & M
    h.force(0, in_(src, n))
    assert h.s()["isr"] == exp
    assert h.s()["isr_cnt"] == min(32, 3 + n)


def test_in_literal_vectors(h):
    h.set_reg(0, "x", 0xAB)
    h.set_reg(0, "isr", 0x12345678)
    h.force(0, in_("x", 8))                # right (reset)
    assert h.s()["isr"] == 0xAB123456
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(in_right=0))
    h.set_reg(0, "isr", 0x12345678)
    h.force(0, in_("x", 8))
    assert h.s()["isr"] == 0x345678AB


def test_in_pins_rotated_by_in_base(h):
    h.w(sm(0, "PINCTRL"), pinctrl(in_base=8))
    h.ext = 0b10110 << 8 | 0b1                # pins 9,10,12 and 0
    h.idle(4)
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(in_right=0))
    h.force(0, in_("pins", 5))
    assert h.s()["isr"] == 0b10110
    h.force(0, in_("pins", 32))             # full 32-bit rotated vector; pin 0 lands at bit 24
    assert h.s()["isr"] == (0b10110 | (1 << 24))


@pytest.mark.parametrize("src", ["rsv4", "rsv5"])
def test_in_reserved_source_is_nop(h, src):
    h.set_reg(0, "isr", 0x5555AAAA)
    h.force(0, in_("null", 4))
    before = h.s()
    h.force(0, in_(src, 8))
    after = h.s()
    assert (after["isr"], after["isr_cnt"]) == (before["isr"], before["isr_cnt"])


def test_in_autopush(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(push_thresh=8, autopush=1, in_right=0))
    h.set_reg(0, "x", 0xF)
    h.force(0, in_("x", 4))
    assert h.s()["rx_level"] == 0 and h.s()["isr_cnt"] == 4
    h.force(0, in_("x", 4))
    s = h.s()
    assert s["rx_level"] == 1 and s["isr"] == 0 and s["isr_cnt"] == 0
    assert h.r(RXF[0]) == 0xFF


def test_in_autopush_stall_is_atomic_and_resamples(h):
    """An IN that stalls on autopush shifts nothing and samples its source when it completes."""
    prog = [in_("x", 8), jmp(1)] + [jmp(1)] * 30
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(push_thresh=8, autopush=1))
    for k in range(4):
        h.set_reg(0, "isr", 0)
        h.force(0, push())
    h.set_reg(0, "isr", 0x11)
    h.force(0, set_("x", 5))
    assert h.s()["rx_level"] == 4
    h.run_program(0, prog, restart=False)   # SM_RESTART would clear the ISR
    h.idle(6)
    s = h.s()
    assert s["pc"] == 0 and s["stalled"] == 1 and s["isr"] == 0x11 and s["isr_cnt"] == 0
    assert h.r(FDEBUG) & 1
    h.force(0, set_("x", 9))              # change the source while the IN is stalled
    h.r(RXF[0])                            # make room
    h.idle(2)
    s = h.s()
    assert s["pc"] == 1 and s["rx_level"] == 4
    vals = [h.r(RXF[0]) for _ in range(4)]
    assert vals[-1] == (0x11 >> 8) | (9 << 24)


# ------------------------------------------------------------------ OUT
@pytest.mark.parametrize("right", [1, 0])
@pytest.mark.parametrize("n", [1, 4, 31, 32])
@pytest.mark.parametrize("dst", ["x", "y", "null", "isr"])
def test_out_shift_and_register_destinations(h, right, n, dst):
    osr = 0xC0FFEE81
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(out_right=right))
    h.set_reg(0, "osr", osr)
    h.force(0, out(dst, n))
    if right:
        v = osr & ((1 << n) - 1)
        exp_osr = 0 if n == 32 else osr >> n
    else:
        v = osr >> (32 - n)
        exp_osr = 0 if n == 32 else (osr << n) & M
    s = h.s()
    assert s["osr"] == exp_osr and s["osr_cnt"] == n
    if dst in ("x", "y"):
        assert s[dst] == v
    if dst == "isr":
        assert s["isr"] == v and s["isr_cnt"] == n


def test_out_literal_and_count_saturates(h):
    h.set_reg(0, "osr", 0x12345678)
    h.force(0, out("x", 8))
    assert h.s()["x"] == 0x78 and h.s()["osr"] == 0x00123456
    h.force(0, out("y", 30))
    assert h.s()["osr_cnt"] == 32
    h.force(0, out("y", 30))
    assert h.s()["osr_cnt"] == 32 and h.s()["y"] == 0
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(out_right=0))
    h.set_reg(0, "osr", 0x12345678)
    h.force(0, out("x", 8))
    assert h.s()["x"] == 0x12 and h.s()["osr"] == 0x34567800


def test_out_pins_window_zero_above_n(h):
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=2, set_count=4, out_base=2, out_count=4))
    h.force(0, set_("pins", 0b1111))
    h.force(0, set_("pindirs", 0b1111))
    assert h.c.pin_out == 0b111100
    h.set_reg(0, "osr", 0b0101)
    h.force(0, out("pins", 2))             # v = 0b01, written to 4 pins: 2<-1, 3<-0, 4<-0, 5<-0
    assert h.c.pin_out == 0b000100
    assert h.c.pin_oe & 0xFF == 0b111100


def test_out_pindirs(h):
    h.w(sm(0, "PINCTRL"), pinctrl(out_base=6, out_count=3))
    h.set_reg(0, "osr", 0b101)
    h.force(0, out("pindirs", 3))          # pins 6, 7, 8 (8 is input-only: dropped)
    assert h.c.pin_oe & 0x1FF == 1 << 6


def test_out_pc(h):
    h.set_reg(0, "osr", 0xFFFFFFF7)
    h.force(0, out("pc", 6))               # v = 0x37 -> PC = 0x17
    assert h.s()["pc"] == 0x17


def test_out_window_wraps_mod_32(h):
    h.w(sm(0, "PINCTRL"), pinctrl(out_base=30, out_count=4))
    h.set_reg(0, "osr", 0b1111)
    h.force(0, out("pins", 4))             # pins 30, 31 (phantom), 0, 1
    assert h.c.pin_out & 0x3 == 0b11


def test_out_count_above_32_behaves_as_32(h):
    h.w(sm(0, "PINCTRL"), pinctrl(out_base=0, out_count=63))
    h.set_reg(0, "osr", 0x3FFFF)
    h.force(0, out("pins", 32))
    assert h.c.pin_out == 0x3E0FF          # every pin with a level register


# ------------------------------------------------------------------ PUSH / PULL
def test_push_block_and_clear(h):
    h.set_reg(0, "isr", 0xABCD)
    h.force(0, push())
    s = h.s()
    assert s["rx_level"] == 1 and s["isr"] == 0 and s["isr_cnt"] == 0
    assert h.r(RXF[0]) == 0xABCD


def test_push_iffull(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(push_thresh=8))
    h.set_reg(0, "x", 0x3)
    h.force(0, mov("isr", "null"))
    h.force(0, in_("x", 7))
    h.force(0, push(iffull=1))
    assert h.s()["rx_level"] == 0 and h.s()["isr"] == 0x3 << 25 and h.s()["isr_cnt"] == 7
    h.force(0, in_("x", 1))
    h.force(0, push(iffull=1))
    assert h.s()["rx_level"] == 1 and h.s()["isr_cnt"] == 0


def _fill_rx(h, e=0):
    for k in range(4):
        h.set_reg(e, "isr", 0x100 + k)
        h.force(e, push())


def test_push_full_block_stalls_noblock_drops(h):
    _fill_rx(h)
    h.set_reg(0, "isr", 0x77)
    h.w(sm(0, "INSTR"), push(block=1))
    h.tick()
    assert h.s()["latch_pend"] == 1 and h.s()["isr"] == 0x77
    assert h.r(FDEBUG) == 0x1
    h.w(FDEBUG, 0x1)
    h.force(0, push(block=0))              # replaces the stuck push; drops data
    s = h.s()
    assert s["latch_pend"] == 0 and s["isr"] == 0 and s["isr_cnt"] == 0 and s["rx_level"] == 4
    assert h.r(FDEBUG) == 0x1
    assert [h.r(RXF[0]) for _ in range(4)] == [0x100, 0x101, 0x102, 0x103]


def test_pull_variants(h):
    h.w(TXF[0], 0x11)
    h.force(0, pull())
    assert h.s()["osr"] == 0x11 and h.s()["osr_cnt"] == 0
    # empty, noblock: OSR = X, no TXSTALL
    h.force(0, set_("x", 13))
    h.force(0, pull(block=0))
    assert h.s()["osr"] == 13 and h.r(FDEBUG) == 0
    # empty, block: stall + TXSTALL
    h.force(0, pull(block=1))
    assert h.s()["latch_pend"] == 1 and h.r(FDEBUG) == 1 << 24


def test_pull_ifempty(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(pull_thresh=8))
    h.w(TXF[0], 0xAA)
    h.w(TXF[0], 0xBB)
    h.force(0, pull())
    h.force(0, out("null", 4))
    h.force(0, pull(ifempty=1))            # osr_cnt 4 < 8: no-op
    assert h.s()["osr"] == 0xA and h.s()["tx_level"] == 1
    h.force(0, out("null", 4))
    h.force(0, pull(ifempty=1))
    assert h.s()["osr"] == 0xBB and h.s()["tx_level"] == 0


def test_pull_with_autopull_and_full_osr_is_noop(h):
    h.w(sm(0, "SHIFTCTRL"), shiftctrl(autopull=1))
    h.w(TXF[0], 1)
    h.w(TXF[0], 2)
    h.idle(1)
    h.force(0, pull())                     # osr_cnt 32 at start: pops 1
    assert h.s()["osr"] == 1 and h.s()["osr_cnt"] == 0 and h.s()["tx_level"] == 1
    h.force(0, pull())                     # osr_cnt == 0: no-op
    assert h.s()["osr"] == 1 and h.s()["tx_level"] == 1


def test_push_pull_ignore_low_bits(h):
    h.w(TXF[0], 0x99)
    h.force(0, pull() | 0x1F)
    assert h.s()["osr"] == 0x99
    h.set_reg(0, "isr", 0x42)
    h.force(0, push() | 0x15)
    assert h.r(RXF[0]) == 0x42


# ------------------------------------------------------------------ MOV
@pytest.mark.parametrize("op", ["none", "invert", "reverse"])
@pytest.mark.parametrize("src", ["x", "y", "null", "isr", "osr"])
def test_mov_sources_ops(h, op, src):
    vals = dict(x=0x80000001, y=0x12345678, null=0, isr=0x0000F00F, osr=0xA5A5A5A5)
    h.set_reg(0, "x", vals["x"])
    h.set_reg(0, "y", vals["y"])
    h.set_reg(0, "isr", vals["isr"])
    h.set_reg(0, "osr", vals["osr"])
    h.force(0, mov("x", src, op))
    v = vals[src]
    v = {"none": v, "invert": ~v & M, "reverse": bitrev(v)}[op]
    assert h.s()["x"] == v


def test_mov_pins_source_is_unmasked_rotated_vector(h):
    h.w(sm(0, "PINCTRL"), pinctrl(in_base=16))
    h.ext = (1 << 17) | (1 << 12) | (1 << 3)   # pin 17 is output-only: its input is ignored
    h.idle(4)
    h.force(0, mov("y", "pins"))
    assert h.s()["y"] == (1 << (12 - 16 + 32)) | (1 << (3 + 16))


@pytest.mark.parametrize("sel,n,tx,rx,exp", [
    (0, 2, 1, 0, M), (0, 2, 2, 0, 0), (0, 0, 0, 0, 0),
    (1, 3, 4, 2, M), (1, 2, 0, 2, 0), (1, 15, 0, 4, M),
])
def test_mov_status(h, sel, n, tx, rx, exp):
    h.w(sm(0, "EXECCTRL"), execctrl(status_sel=sel, status_n=n))
    for k in range(rx):
        h.force(0, push())
    for k in range(tx):
        h.w(TXF[0], k)
    h.force(0, mov("x", "status"))
    assert h.s()["x"] == exp


def test_mov_destinations(h):
    h.set_reg(0, "x", 0xDEADBEEF)
    h.force(0, in_("null", 5))
    h.force(0, mov("isr", "x"))
    assert h.s()["isr"] == 0xDEADBEEF and h.s()["isr_cnt"] == 0
    h.force(0, out("null", 3))
    h.force(0, mov("osr", "x", "invert"))
    assert h.s()["osr"] == 0x21524110 and h.s()["osr_cnt"] == 0
    h.force(0, mov("y", "x", "reverse"))
    assert h.s()["y"] == bitrev(0xDEADBEEF)
    h.force(0, mov("pc", "x"))
    assert h.s()["pc"] == 0xDEADBEEF & 31
    h.w(sm(0, "PINCTRL"), pinctrl(out_base=0, out_count=8))
    h.force(0, mov("pins", "x"))
    assert h.c.pin_out & 0xFF == 0xEF


@pytest.mark.parametrize("word", [
    mov("rsv3", "x"), mov("x", "y", "rsv3"), mov("x", "rsv4"),
    mov("exec", "x", "rsv3"), mov("exec", "rsv4"), mov("pc", "rsv4"),
])
def test_mov_reserved_is_nop(h, word):
    h.set_reg(0, "x", 7)
    h.set_reg(0, "y", 9)
    before = h.s()
    h.force(0, word)
    after = h.s()
    assert nolatch(after) == nolatch(before)


# ------------------------------------------------------------------ SET
def test_set_registers_and_pins(h):
    h.force(0, set_("x", 31))
    h.force(0, set_("y", 17))
    assert (h.s()["x"], h.s()["y"]) == (31, 17)
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=3, set_count=3))
    h.force(0, set_("pins", 0b11111))      # only 3 pins written
    assert h.c.pin_out & 0xFF == 0b111 << 3
    h.force(0, set_("pindirs", 0b101))
    assert h.c.pin_oe & 0xFF == 0b101 << 3


@pytest.mark.parametrize("count", [6, 7])
def test_set_count_6_7_writes_zeros_to_extra_pins(h, count):
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=0, set_count=7, out_base=0, out_count=8))
    h.set_reg(0, "x", 0xFF)
    h.force(0, mov("pins", "x"))
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=0, set_count=count))
    h.force(0, set_("pins", 0b11111))
    assert h.c.pin_out & 0xFF == 0b11111 | (0xFF & ~((1 << count) - 1))


@pytest.mark.parametrize("dst", ["rsv3", "rsv5", "rsv6", "rsv7"])
def test_set_reserved_is_nop(h, dst):
    before = h.s()
    h.force(0, set_(dst, 31))
    assert nolatch(h.s()) == nolatch(before) and h.c.pin_out == 0


# ------------------------------------------------------------------ IRQ and reserved: NOP with delay
@pytest.mark.parametrize("word", [
    irq(0x00), irq(0x20), irq(0x40), irq(0x1F), irq(0xFF),
    wait(0, "reserved", 5), wait(1, "reserved", 5), in_("rsv4", 3), in_("rsv5", 3),
    mov("rsv3", "x"), mov("x", "x", "rsv3"), mov("y", "rsv4"), set_("rsv3", 1),
    set_("rsv5", 1), set_("rsv6", 1), set_("rsv7", 1), wait(0, "irq", 2),
])
def test_nop_forms_take_delay_and_advance_pc(h, word):
    """Run `word [3]` from IMEM: next instruction executes 4 SM cycles later at PC+1."""
    prog = [word | (3 << 8), set_("x", 1), jmp(2)] + [jmp(3)] * 29
    h.run_program(0, prog)
    h.tick()                               # h+1
    h.tick()                               # h+2: executes word
    assert h.s()["pc"] == 1 and h.s()["delay_cnt"] == 3 and h.s()["stalled"] == 0
    h.idle(3)
    assert h.s()["x"] == 0
    h.tick()
    assert h.s()["x"] == 1


def test_irq_carries_sideset(h):
    pins = pinctrl(sideset_count=2, sideset_base=4)
    prog = [irq(0x40, ds=0b10 << 3), jmp(1)] + [jmp(1)] * 30
    h.run_program(0, prog, pins=pins)
    h.idle(2)
    assert h.c.pin_out & 0x30 == 0b10 << 4

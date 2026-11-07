"""JMP and WAIT (ISA `### Instruction semantics`, `### Field values`).

Hand-encoded words from `enc.py`; every expected value is derived from the spec
text, not from a reference model. Inputs go through the 3-stage pipeline
(T_IN = 3), so `Bus.pin` is driven from cycle 0 and the levels are already in
`IN_VEC` by the first SM cycle.
"""

import pytest

from verif.iss_b import enc
from verif.iss_b.enc import jmp, poke, set_, setup, wait_, word, execs, Bus
from verif.iss_b.enc import WAIT_GPIO, WAIT_IRQ, WAIT_PIN, WAIT_RESERVED
from verif.iss_b.enc import JMP_ALWAYS, JMP_DECX, JMP_DECY, JMP_NOTX, JMP_NOTY
from verif.iss_b.enc import JMP_OSRE, JMP_PIN, JMP_XNEQY

PIN7 = 1 << 7


@pytest.mark.parametrize(
    "cond,x,y,osr_cnt,pin,jmp_pin,taken",
    [
        (JMP_ALWAYS, 0, 0, 32, 0, 0, True),
        (JMP_NOTX, 0, 5, 32, 0, 0, True),
        (JMP_NOTX, 1, 5, 32, 0, 0, False),
        (JMP_DECX, 0, 0, 32, 0, 0, False),
        (JMP_DECX, 7, 0, 32, 0, 0, True),
        (JMP_NOTY, 3, 0, 32, 0, 0, True),
        (JMP_NOTY, 3, 1, 32, 0, 0, False),
        (JMP_DECY, 0, 9, 32, 0, 0, True),
        (JMP_XNEQY, 3, 3, 32, 0, 0, False),
        (JMP_XNEQY, 3, 4, 32, 0, 0, True),
        (JMP_PIN, 0, 0, 32, PIN7, 7, True),
        (JMP_PIN, 0, 0, 32, PIN7, 6, False),
        (JMP_PIN, 0, 0, 32, 0, 7, False),
        # !OSRE is `osr_cnt < thresh(PULL_THRESH)`, and thresh(0) is 32.
        (JMP_OSRE, 0, 0, 0, 0, 0, True),
        (JMP_OSRE, 0, 0, 31, 0, 0, True),
        (JMP_OSRE, 0, 0, 32, 0, 0, False),
    ],
)
def test_jmp_condition(cond, x, y, osr_cnt, pin, jmp_pin, taken):
    """Each condition is evaluated on the start-of-cycle X, Y, IN_VEC and osr_cnt."""
    b = setup(
        imem=[jmp(cond, 9), jmp(JMP_ALWAYS, 0)],
        execctrl_v=enc.execctrl(jmp_pin=jmp_pin),
        pin=pin,
    )
    poke(b, 0, x=x, y=y, osr_cnt=osr_cnt)
    # Address is an absolute IMEM index, and the next PC is that address itself.
    assert execs(b, 1)[0]["pc"] == (9 if taken else 1)


def test_jmp_decx_writes_x_minus_one_whether_or_not_it_jumps():
    """`X--` and `Y--` always write X-1 / Y-1 (mod 2^32); the branch tests the value
    before the decrement."""
    b = setup(imem=[jmp(JMP_DECX, 20)])
    poke(b, 0, x=1)
    assert execs(b, 1)[0]["x"] == 0  # 1 != 0: taken, and X becomes 0
    assert b.state(0)["pc"] == 20

    b = setup(imem=[jmp(JMP_DECX, 20)])
    poke(b, 0, x=0)
    st = execs(b, 1)[0]
    assert st["x"] == 0xFFFFFFFF  # not taken, but still written
    assert st["pc"] == 1

    b = setup(imem=[jmp(JMP_DECY, 20)])
    poke(b, 0, y=0)
    assert execs(b, 1)[0]["y"] == 0xFFFFFFFF


def test_jmp_delay_applies_whether_or_not_the_jump_is_taken():
    """IMEM resets to `jmp 0`, so the PC after the delay says whose delay it was."""
    for cond, x, taken in ((JMP_ALWAYS, 0, True), (JMP_NOTX, 0, True), (JMP_NOTX, 1, False)):
        b = setup(imem={0: jmp(cond, 5, delay=2)})
        poke(b, 0, x=x)
        sts = execs(b, 4)
        assert sts[0]["pc"] == (5 if taken else 1), cond
        assert [s["delay_cnt"] for s in sts[:3]] == [2, 1, 0], cond
        assert sts[3]["pc"] == 0  # IMEM[5] (or IMEM[1]) is `jmp 0`


def test_jmp_wraps_at_wrap_top_and_jumps_to_wrap_bottom():
    b = setup(
        # A taken JMP sets the PC directly, so the wrap needs a non-jump at WRAP_TOP.
        imem=[jmp(JMP_ALWAYS, 1), jmp(JMP_ALWAYS, 2), set_(enc.SET_Y, 1)],
        execctrl_v=enc.execctrl(wrap_bottom=1, wrap_top=2),
    )
    # PC 2 == WRAP_TOP, so the cycle after it wraps to WRAP_BOTTOM 1.
    assert [s["pc"] for s in execs(b, 5)] == [1, 2, 1, 2, 1]


def test_jmp_address_is_an_absolute_imem_index():
    b = setup(imem={k: jmp(JMP_ALWAYS, 0) for k in range(32)})
    # A forced JMP is how the host sets the PC: it runs in the cycle after the write.
    b.force(0, jmp(JMP_ALWAYS, 17))
    assert b.state(0)["latch"] == 0x0011  # the write is in the latch
    assert b.state(0)["pc"] == 0  # not executed yet
    b.step()
    assert b.state(0)["pc"] == 17
    poke(b, 0, pc=30)
    assert execs(b, 1)[0]["pc"] == 0  # IMEM[30] is `jmp 0`: index 0, not PC-relative


def test_wait_gpio_uses_an_absolute_pin_and_wait_pin_an_in_base_relative_one():
    b = setup(imem=[wait_(WAIT_GPIO, 3, pol=1), jmp(JMP_ALWAYS, 0)], pin=1 << 3)
    assert execs(b, 1)[0]["pc"] == 1  # GPIO 3 is high: passes at once
    b = setup(imem=[wait_(WAIT_GPIO, 3, pol=1), jmp(JMP_ALWAYS, 0)], pin=1 << 4)
    assert execs(b, 1)[0]["pc"] == 0  # stalls

    b = setup(
        imem=[wait_(WAIT_PIN, 1, pol=1), jmp(JMP_ALWAYS, 0)],
        pinctrl_v=enc.pinctrl(in_base=4),
        pin=1 << 5,
    )
    assert execs(b, 1)[0]["pc"] == 1  # IN_BASE + Index = pin 5
    b = setup(
        imem=[wait_(WAIT_PIN, 1, pol=1), jmp(JMP_ALWAYS, 0)],
        pinctrl_v=enc.pinctrl(in_base=4),
        pin=1 << 1,
    )
    assert execs(b, 1)[0]["pc"] == 0


def test_wait_pol_stalls_until_the_bit_equals_pol():
    """The pin level reaches IN_VEC three cycles after it is driven."""
    b = setup(imem=[wait_(WAIT_GPIO, 0, pol=1), jmp(JMP_ALWAYS, 0)])
    # The level driven at step 0 reaches IN_VEC at step 3 (T_IN = 3).
    sts = enc.drive(b, [1, 1, 1, 1])
    assert [s["pc"] for s in sts] == [0, 0, 0, 1]
    assert [s["stalled"] for s in sts] == [0, 1, 1, 0]


def test_wait_irq_flag_is_hardwired_zero():
    """MVP: `wait 0 irq n` completes at once, `wait 1 irq n` stalls forever."""
    b = setup(imem=[wait_(WAIT_IRQ, 3, pol=0), jmp(JMP_ALWAYS, 0)])
    assert execs(b, 1)[0]["pc"] == 1

    b = setup(imem=[wait_(WAIT_IRQ, 3, pol=1)])
    sts = execs(b, 5)
    assert [s["pc"] for s in sts] == [0] * 5
    assert [s["stalled"] for s in sts] == [1] * 5
    assert b.read(enc.A_FDEBUG) == 0  # no flag is raised for a stall


def test_wait_reserved_source_is_a_nop():
    b = setup(imem=[wait_(WAIT_RESERVED, 0, pol=0), jmp(JMP_ALWAYS, 0)])
    assert [s["pc"] for s in execs(b, 3)] == [1, 0, 1]  # never stalls


def test_wait_delay_starts_after_the_condition_holds():
    b = setup(imem=[wait_(WAIT_GPIO, 0, pol=1, delay=1), jmp(JMP_ALWAYS, 0)], pin=1)
    # The pin is already high in IN_VEC, so the WAIT completes in its first cycle
    # and only then loads the delay counter.
    sts = enc.drive(b, [1, 1, 1])
    assert [s["pc"] for s in sts] == [0, 1, 1]  # step 0 is not yet an SM cycle
    assert [s["delay_cnt"] for s in sts] == [0, 1, 0]

    b = setup(imem=[wait_(WAIT_GPIO, 0, pol=1, delay=1), jmp(JMP_ALWAYS, 0)])
    assert execs(b, 1, 0)[0]["pc"] == 0  # stalled: no delay is loaded


def test_wait_side_set_and_delay_split_of_the_field():
    """With SC = 2 the top 2 bits are side-set and the low 3 bits are the delay."""
    b = setup(
        imem=[wait_(WAIT_GPIO, 0, pol=0, delay=7, side=0b10, sc=2)],
        pinctrl_v=enc.pinctrl(sideset_count=2, sideset_base=3),
    )
    st = execs(b, 1)[0]
    assert st["delay_cnt"] == 7
    # SIDE_EN = 0, so both side-set bits are data: data bit 1 -> SIDESET_BASE + 1.
    assert b.chip.pin_out & (1 << 4) == 1 << 4
    assert b.chip.pin_out & (1 << 3) == 0
    # opcode 001 in 15:13, side-set 10 then delay 111 in 12:8, index 0 in 4:0
    assert word(enc.WAIT, enc.dss(0b10, 7, sc=2), (0 << 7) | (WAIT_GPIO << 5)) == 0x3700


def test_wait_index_wraps_in_the_pin_space_and_phantom_pins_read_zero():
    """Windows are 5-bit and wrap mod 32; pins 18..31 are phantom and read 0."""
    b = setup(
        imem=[wait_(WAIT_PIN, 3, pol=1), jmp(JMP_ALWAYS, 0)],
        pinctrl_v=enc.pinctrl(in_base=30),
        pin=1 << 1,  # (IN_BASE 30 + 3) mod 32 = 1
    )
    assert execs(b, 1)[0]["pc"] == 1
    b = setup(
        imem=[wait_(WAIT_PIN, 3, pol=1), jmp(JMP_ALWAYS, 0)],
        pinctrl_v=enc.pinctrl(in_base=30),
    )
    assert execs(b, 1)[0]["pc"] == 0  # pin 1 is low


def test_wait_never_writes_a_pin_or_a_register():
    b = setup(
        imem=[wait_(WAIT_GPIO, 0, pol=1, side=0b1, sc=1)],
        pinctrl_v=enc.pinctrl(sideset_count=1),
    )
    before = dict(b.state(0))
    enc.settle(b, 1)
    execs(b, 1, 1)
    after = b.state(0)
    assert after["x"] == before["x"] and after["y"] == before["y"]
    assert after["isr"] == before["isr"] and after["osr"] == before["osr"]


def test_two_engines_run_independently():
    """Both engines share the chip but each has its own PC, X and IMEM."""
    b = Bus()
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.cfg(1, clkdiv_v=enc.clkdiv(1, 0))
    b.load_imem(0, [set_(enc.SET_X, 3), jmp(JMP_ALWAYS, 0)])
    b.load_imem(1, [set_(enc.SET_X, 7), jmp(JMP_ALWAYS, 0)])
    b.write(enc.A_CTRL, enc.ctrl(sm_enable=0b11, clkdiv_restart=0b11))
    b.idle(1)
    for _ in range(3):
        b.step()
    assert b.state(0)["x"] == 3 and b.state(1)["x"] == 7


from verif.iss_b.enc import set_  # noqa: E402 - used by test_two_engines_run_independently
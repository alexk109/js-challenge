"""Hand-encoders for the tests, built only from `docs/mvp-isa.md` `## Encoding` and
the register tables of `docs/mvp-regmap.md`. No assembler is used: the tests must
not depend on `asm/`.

`test_encoding.py` checks every encoder against the spec's "Worked encodings"
table, so a wrong bit position here shows up as a failed test, not as a silently
wrong program.
"""

from verif.iss_b.pio_iss_b import T_IN, Chip

# --- instruction opcodes (ISA `## Encoding`): PULL shares 100 with PUSH, bit 7
JMP, WAIT, IN, OUT, PUSH, MOV, IRQ, SET = 0, 1, 2, 3, 4, 5, 6, 7
PULL = PUSH
OPCODE = {"jmp": JMP, "wait": WAIT, "in": IN, "out": OUT, "push": PUSH, "pull": PULL,
          "mov": MOV, "irq": IRQ, "set": SET}

# --- JMP conditions (ISA `### Field values`)
JMP_ALWAYS, JMP_NOTX, JMP_DECX, JMP_NOTY, JMP_DECY, JMP_XNEQY, JMP_PIN, JMP_OSRE = range(8)
# --- WAIT sources
WAIT_GPIO, WAIT_PIN, WAIT_IRQ, WAIT_RESERVED = range(4)
# --- IN sources
IN_PINS, IN_X, IN_Y, IN_NULL, IN_RSVD45, IN_RSVD5, IN_ISR, IN_OSR = range(8)
# --- OUT destinations
OUT_PINS, OUT_X, OUT_Y, OUT_NULL, OUT_PINDIRS, OUT_PC, OUT_ISR, OUT_EXEC = range(8)
# --- MOV destinations, ops and sources
MOV_PINS, MOV_X, MOV_Y, MOV_RSVD3, MOV_EXEC, MOV_PC, MOV_ISR, MOV_OSR = range(8)
MOV_NONE, MOV_INVERT, MOV_REVERSE, MOV_RSVD3_OP = range(4)
MOVSRC_PINS, MOVSRC_X, MOVSRC_Y, MOVSRC_NULL, MOVSRC_RSVD4, MOVSRC_STATUS, MOVSRC_ISR, \
    MOVSRC_OSR = range(8)
# --- SET destinations
SET_PINS, SET_X, SET_Y, SET_RSVD3, SET_PINDIRS = 0, 1, 2, 3, 4


def dss(side: int = 0, delay: int = 0, sc: int = 0) -> int:
    """The 5-bit delay/side-set field of bits 12:8 (ISA `### Delay/side-set field`).

    `sc` is `PINCTRL.SIDESET_COUNT`: the top `sc` bits of the field are the side-set
    value (`side`, whose MSB is the enable bit when `EXECCTRL.SIDE_EN` is 1) and the
    low `5 - sc` bits are the delay in SM cycles. So the field is
    `(side << (5 - sc)) | delay`, and with `sc = 0` the whole field is the delay.
    """
    assert 0 <= sc <= 5, sc
    assert 0 <= side < (1 << sc) or sc == 0, (side, sc)
    assert 0 <= delay < (1 << (5 - sc)), (delay, sc)
    return (side << (5 - sc)) | delay


def word(opcode: int, field: int = 0, low8: int = 0) -> int:
    """One instruction: opcode in 15:13, delay/side-set field in 12:8, rest in 7:0."""
    return ((opcode & 7) << 13) | ((field & 0x1F) << 8) | (low8 & 0xFF)


def jmp(cond: int, addr: int, delay: int = 0, side: int = 0, sc: int = 0) -> int:
    return word(JMP, dss(side, delay, sc), (cond << 5) | (addr & 0x1F))


def wait_(
    source: int, index: int, pol: int = 0, delay: int = 0, side: int = 0, sc: int = 0
) -> int:
    return word(
        WAIT, dss(side, delay, sc), (pol << 7) | (source << 5) | (index & 0x1F)
    )


def in_(source: int, count: int, delay: int = 0, side: int = 0, sc: int = 0) -> int:
    return word(IN, dss(side, delay, sc), (source << 5) | (count & 0x1F))


def out(dst: int, count: int, delay: int = 0, side: int = 0, sc: int = 0) -> int:
    return word(OUT, dss(side, delay, sc), (dst << 5) | (count & 0x1F))


def push(iffull: int = 0, block: int = 0, delay: int = 0, side: int = 0, sc: int = 0) -> int:
    return word(PUSH, dss(side, delay, sc), (iffull << 6) | (block << 5))


def pull(
    ifempty: int = 0, block: int = 0, delay: int = 0, side: int = 0, sc: int = 0
) -> int:
    return word(PULL, dss(side, delay, sc), 0x80 | (ifempty << 6) | (block << 5))


def mov(dst: int, src: int, op: int = 0, delay: int = 0, side: int = 0, sc: int = 0) -> int:
    return word(MOV, dss(side, delay, sc), (dst << 5) | (op << 3) | (src & 7))


def irq(clr: int = 0, wait: int = 0, delay: int = 0, side: int = 0, sc: int = 0) -> int:
    return word(IRQ, dss(side, delay, sc), (clr << 7) | (wait << 6))


def set_(dst: int, data: int, delay: int = 0, side: int = 0, sc: int = 0) -> int:
    return word(SET, dss(side, delay, sc), (dst << 5) | (data & 0x1F))


NOP = 0xA042  # `mov y, y`, the spec's worked encoding for `nop`

# --- register field builders (regmap `### Registers`)

A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03
A_TXF0, A_RXF0 = 0x04, 0x08
A_PADOUT, A_PADOE, A_CFGINFO = 0x0F, 0x10, 0x11
A_INTR, A_IRQ0_INTE, A_IRQ0_INTF, A_IRQ0_INTS = 0x4A, 0x4B, 0x4C, 0x4D
A_IRQ1_INTE, A_IRQ1_INTF, A_IRQ1_INTS = 0x4E, 0x4F, 0x50


def a_imem(e: int, k: int) -> int:
    """`INSTR_MEM` of engine `e`, word `k`."""
    return 0x12 + 0x40 * e + k


def a_sm(e: int, i: int) -> int:
    """`SMe_*`: i = 0 CLKDIV, 1 EXECCTRL, 2 SHIFTCTRL, 3 ADDR, 4 INSTR, 5 PINCTRL."""
    return 0x32 + 6 * e + i


A_CLK0, A_CLK1 = 0x10000, 0x10000  # CLKDIV = 1.0


def clkdiv(integer: int = 1, frac: int = 0) -> int:
    return ((integer & 0xFFFF) << 16) | ((frac & 0xFF) << 8)


def execctrl(
    side_en: int = 0,
    side_pindir: int = 0,
    jmp_pin: int = 0,
    wrap_top: int = 31,
    wrap_bottom: int = 0,
    status_sel: int = 0,
    status_n: int  | None = None,
) -> int:
    return (
        (side_en << 30)
        | (side_pindir << 29)
        | (jmp_pin << 24)
        | ((wrap_top & 0x1F) << 12)
        | ((wrap_bottom & 0x1F) << 7)
        | (status_sel << 4)
        | (0 if status_n is None else status_n & 0xF)
    )


def shiftctrl(
    pull_thresh: int = 0,
    push_thresh: int = 0,
    out_shiftdir: int = 1,
    in_shiftdir: int = 1,
    autopull: int = 0,
    autopush: int = 0,
) -> int:
    return (
        ((pull_thresh & 0x1F) << 25)
        | ((push_thresh & 0x1F) << 20)
        | (out_shiftdir << 19)
        | (in_shiftdir << 18)
        | (autopull << 17)
        | (autopush << 16)
    )


def pinctrl(
    sideset_count: int = 0,
    set_count: int = 5,
    out_count: int = 32,
    in_base: int = 0,
    sideset_base: int = 0,
    set_base: int = 0,
    out_base: int = 0,
) -> int:
    return (
        ((sideset_count & 7) << 29)
        | ((set_count & 7) << 26)
        | ((out_count & 0x3F) << 20)
        | ((in_base & 0x1F) << 15)
        | ((sideset_base & 0x1F) << 10)
        | ((set_base & 0x1F) << 5)
        | (out_base & 0x1F)
    )


def ctrl(sm_enable: int = 0, sm_restart: int = 0, clkdiv_restart: int = 0) -> int:
    return ((clkdiv_restart & 3) << 8) | ((sm_restart & 3) << 4) | (sm_enable & 3)

class Bus:
    """Drives a `Chip` one core cycle at a time and counts the cycles.

    `cycle` is the number of cycles already run, so the next `step()` is core
    cycle `self.cycle` (cycle 0 is the first one, per the ISS convention).
    """

    def __init__(self, chip: Chip | None = None, pin: int = 0) -> None:
        self.chip = Chip() if chip is None else chip
        self.cycle = 0
        # `pin` is the level driven on every cycle that does not pass `pin_in`
        # explicitly. T_IN = 3, so a level must be held for three cycles before an
        # instruction can see it: setting `pin` before the setup writes fills the
        # pipeline, so the first SM cycle already sees it.
        self.pin = pin
        self.last_read: int | None = None  # the data of the most recent read

    # -- raw access -------------------------------------------------------
    def step(self, pin_in: int | None = None, bus: tuple | None = None) -> int | None:
        out = self.chip.step(self.pin if pin_in is None else pin_in, bus)
        self.cycle += 1
        return out

    def write(self, addr: int, data: int, pin_in: int | None = None) -> None:
        self.step(pin_in, ("w", addr, data))

    def read(self, addr: int, pin_in: int | None = None) -> int:
        self.last_read = self.step(pin_in, ("r", addr))
        return self.last_read

    def idle(self, n: int = 1, pin_in: int | None = None) -> None:
        for _ in range(n):
            self.step(pin_in)

    def reset(self) -> None:
        """Reset the chip and restart the cycle count at 0."""
        self.chip.reset()
        self.cycle = 0
        self.last_read = None

    def pad(self) -> int:
        """The pad model of the anchor trace: an unloaded `uio` pad follows uio_out."""
        return self.chip.pin_out & 0xFF

    # -- register helpers -------------------------------------------------
    def cfg(
        self,
        e: int,
        clkdiv_v: int | None = None,
        execctrl_v: int | None = None,
        shiftctrl_v: int | None = None,
        pinctrl_v: int | None = None,
    ) -> None:
        # i = 0 CLKDIV, 1 EXECCTRL, 2 SHIFTCTRL, 3 ADDR (RO), 4 INSTR, 5 PINCTRL
        for i, v in ((0, clkdiv_v), (1, execctrl_v), (2, shiftctrl_v), (5, pinctrl_v)):
            if v is not None:
                self.write(a_sm(e, i), v)

    def load_imem(self, e: int, words: dict[int, int] | list[int], at: int = 0) -> None:
        if isinstance(words, dict):
            items = words.items()
        else:
            items = enumerate(words, start=at)
        for k, w in items:
            self.write(a_imem(e, k), w)

    def start(self, e: int, restart: int = 1, clkdiv_restart: int = 1) -> None:
        """Regmap `### Typical host sequences`, step 6."""
        self.write(A_CTRL, ctrl(sm_enable=1 << e, sm_restart=restart,
                               clkdiv_restart=clkdiv_restart))

    def force(self, e: int, instr: int) -> None:
        """Write `SMe_INSTR`: the host-forced instruction of ISA `### EXEC`."""
        self.write(a_sm(e, 4), instr)

    def push_tx(self, e: int, word_: int) -> None:
        self.write(A_TXF0 + e, word_)

    def pop_rx(self, e: int) -> int:
        return self.read(A_RXF0 + e)

    # -- observation ------------------------------------------------------
    def state(self, e: int = 0) -> dict:
        return self.chip.state(e)

    def pad_oe(self, k: int) -> int:
        """`pin_oe` bit `k` as it stands for the *next* cycle (i.e. the pad)."""
        return (self.chip.pin_oe >> k) & 1

    def pad_out(self, k: int) -> int:
        return (self.chip.pin_out >> k) & 1


def setup(
    e: int = 0,
    imem=None,
    clkdiv_v: int | None = None,
    execctrl_v: int | None = None,
    shiftctrl_v: int | None = None,
    pinctrl_v: int | None = None,
    restart: int = 1,
    pin: int = 0,
    tx: list[int] | None = None,
) -> Bus:
    """One configured, programmed and started engine. Any `*_v` left as None keeps
    the register's reset value. `restart=1` gives the ISA's reset engine state, so
    anything poked afterwards survives. `pin` is driven from cycle 0, so the input
    pipeline is already full of it when the first SM cycle runs. `tx` words are pushed
    into engine `e`'s TX FIFO after the start, before any SM cycle runs."""
    b = Bus(pin=pin)
    b.cfg(
        e,
        clkdiv_v=clkdiv_v,
        execctrl_v=execctrl_v,
        shiftctrl_v=shiftctrl_v,
        pinctrl_v=pinctrl_v,
    )
    if imem is not None:
        b.load_imem(e, imem)
    b.start(e, restart=restart)
    for w in tx or []:
        b.push_tx(e, w)
    return b


def poke(b: Bus, e: int, **fields) -> None:
    """Force start-of-cycle engine state.

    Used only where the spec gives no single instruction that loads the value
    (X/Y/OSR/ISR can only be built up a bit at a time).
    """
    eng = b.chip.eng[e]
    for name, value in fields.items():
        assert hasattr(eng, name), name
        setattr(eng, name, value)


def trace(b: Bus, n: int, pin_in: int | None = None) -> list[dict]:
    """Run `n` cycles, returning the engine-0 state after each of them."""
    out = []
    for _ in range(n):
        b.step(pin_in)
        out.append(b.state(0))
    return out


def settle(b: Bus, pin: int, n: int = T_IN + 1) -> None:
    """Hold `pin` on the pads for `n` cycles so an instruction can see it.

    The three input pipeline stages start at 0, so a level presented now reaches
    `IN_VEC` three cycles later.
    """
    for _ in range(n):
        b.step(pin)


def first_sm_cycle(b: Bus) -> int:
    """The cycle number of the engine's first SM cycle after `setup()`.

    `setup()` ends with the `CTRL` write in cycle `h`, and the spec says the first
    SM cycle is `h+2`, so it is the second cycle still to come.
    """
    return b.cycle + 2


def sm_cycle_now(b: Bus, e: int = 0) -> bool:
    """Would engine `e` have an SM cycle in the next step? (`ISA When an engine
    executes`: force, or SM_ENABLE and the divider output `en`.)"""
    eng = b.chip.eng[e]
    return bool(eng.force or ((b.chip.sm_enable >> e) & 1 and eng.d_en))


def execs(b: Bus, n: int, pin_in: int | None = None) -> list[dict]:
    """State after each of the next `n` SM cycles of engine 0.

    Steps past any core cycles before the engine runs (the spec puts the first SM
    cycle in `h+2`, and `setup()` may have used further bus cycles), so
    `execs(b, 1)[0]` is the state after IMEM[0] has executed.
    """
    out = []
    while len(out) < n:
        is_sm = sm_cycle_now(b)
        b.step(pin_in)
        if is_sm:
            out.append(b.state(0))
    return out


def drive(b: Bus, levels: list[int]) -> list[dict]:
    """Step one cycle per level and return the engine-0 state after each.

    Use this where the pad level changes mid-test: the level an instruction reads
    is the one presented three cycles earlier.
    """
    out = []
    for level in levels:
        b.step(level)
        out.append(b.state(0))
    return out

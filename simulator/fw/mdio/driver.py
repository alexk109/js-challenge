"""Host side of the MVP MDIO/MDC firmware: load program.pio, configure engine 0, send frames.

Everything here is written against `docs/mvp-regmap.md` (register word
addresses, the `### Typical host sequences` startup order and the per-engine fields) and
against `docs/mvp-isa.md` (divider, side-set, shift control, T_OUT/T_IN).  Host accesses go
through `iss.pio_iss.ChipDevice`'s write/read queue, one per tick, so every wait here is the
simulation running -- the same thing a real host does over the SPI port of
`docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                     # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == 1))   # MDIO is the wired-AND bus
    dev = bus.attach(ChipDevice())
    drv = MdioDriver(dev, bus, mdc_pin=0, mdio_pin=1, mdc_hz=2_500_000)
    drv.start()                # IMEM, CLKDIV/EXECCTRL/SHIFTCTRL/PINCTRL, idle pins, enable
    drv.write_reg(BMCR, 0x2100)   # one clause 22 frame; the PHY's register file changes
    drv.read_reg(BMSR).value     # the same, with the data arriving in the RX FIFO

THE CHIP IS THE STATION, not a PHY: clause 22 has one station per bus and it owns MDC
(`research/ref/protocols/mdio/ti-dp83tc811-smi.txt`), so this driver clocks MDC and drives
the data, and the `MdioPhy` models on the other end only answer a read's turnaround and
data bits.  That is the side `verif/vip/mdio.py` can be judged against: its
`MdioStation` model is a station, so it is the thing this firmware replaces, while
`MdioPhy` and `MdioChecker` stay on the bus.

Five things about this chip that a clause 22 host driver does not have to think about, and
that this module exists to get right (each cites the section of `docs/mvp-isa.md` that fixes
it):

* **The pins.** Both lines must be uio pins (0..7): MDIO is bidirectional and only a uio pin
  has a direction register (`## The 18 protocol pins`).  `mdc_pin` must be `mdio_pin - 1`, so
  that one 2-pin SET window holds MDC at data bit 0 and MDIO at bit 1 -- the program releases
  MDIO with `set pindirs, 1`, and with the pins the other way round that would release MDC
  in the middle of a bit time.
* **`SIDESET_COUNT` resets to 2 with `SIDE_EN = 0`**, where every instruction side-sets
  `SIDESET_COUNT` pins and the side-set channel beats OUT/SET on the same pin
  (`### Delay/side-set field`, `### Pin writes`).  The program uses one `side` operand with no
  `opt`, so the driver writes `SIDESET_COUNT = 1`: one bit, and `side 1` means MDC high.
* **The host cannot set the PC** (`SMe_ADDR` is read-only, `### Enable, restart`), so a frame
  starts with a forced `jmp write_frame` or `jmp read_frame` through `SMe_INSTR`, and the
  first OSR word comes from a forced `pull block` in the same way: `### EXEC` says a
  host-forced instruction runs in the very next core cycle whatever the divider and
  `SM_ENABLE` say.
* **`T_IN = 3`** (`## Timing`): an engine in cycle `n` sees the pad value of cycle `n-3`, so
  the program samples a read's data during the low phase of the bit time it belongs to rather
  than on the edge.  program.pio works the arithmetic out; this module only has to leave MDC
  and the pin directions alone.
* **There is no IRQ**, so the program cannot park in `irq wait`: it idles in `halt` at address
  0 and the host sees the PC in `SMe_ADDR`.

The bit time and the two window layouts are read back out of the assembled words
(`bit_cycles`, `check_program`) rather than assumed, so a change to program.pio that breaks
the shape this driver assumes fails a test instead of producing a wrong CLKDIV.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Optional, Sequence

from asm.pioasm import Program, assemble
from verif.vip.core import Bus, Device

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
IMEM_SIZE = 32                # docs/mvp-isa.md `### Engine state`
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "mdio"           # the .program name inside program.pio

# ---- clause 22 frame layout, from research/ref/protocols/mdio/ ------------------
#: 32 ones of preamble (wikipedia-mdio-clause22.txt PRE_32, LAN8720A figures 3.5/3.6).
PREAMBLE_WORD = 0xFFFF_FFFF
#: ST = 01 (DP83TC811 Table 7-15), OP = 10 read / 01 write, TA = <10> on a write.
ST_BITS = 0b01
OP_READ, OP_WRITE = 0b10, 0b01
TA_WRITE = 0b10
#: Bit indexes inside a 64-bit frame, so a word can be packed without counting by hand.
#: Frame bit 32 is the first bit of the second TX word (the first is the preamble).
ST_BIT, OP_BIT, PHYAD_BIT, REGAD_BIT, TA_BIT, DATA_BIT = 32, 34, 36, 41, 46, 48
FRAME_BITS = 64               # verif/vip/mdio.py FRAME_BITS, the same 64
ADDRESS_BITS, REGISTER_BITS, DATA_BITS = 5, 5, 16

#: The shortest legal MDC period: 400 ns, so MDC may not exceed 2.5 MHz
#: (LAN8720A Table 5.12 `tclkp` 400 ns min; verif/vip/mdio.py `MdioTimings.t_period`).
MIN_PERIOD_NS = 400.0
MAX_MDC_HZ = 1e9 / MIN_PERIOD_NS          # 2.5 MHz

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) ----
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF = regmap.txf_addrs()
A_RXF = regmap.rxf_addrs()
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_INSTR_MEM = regmap.imem_bases()    # engine e, word k: A_INSTR_MEM[e] + k
A_SM_BASE = regmap.sm_base()              # SMe_CLKDIV = A_SM_BASE + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

# CTRL fields (docs/mvp-regmap.md `**CTRL**`)
CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART = 0x001, 0x010, 0x100
# FSTAT fields, engine e
FSTAT_TXEMPTY, FSTAT_TXFULL = 0x0100_0000, 0x0001_0000

# Instruction encodings the host forces (docs/mvp-isa.md `## Encoding`).
OP_JMP, OP_SHIFT, OP_SET = 0b000, 0b100, 0b111
JMP_ALWAYS = 0b000
JMP_COND_INVERT_Y = 0b011     # `jmp !y`, only used to check the assembled program's shape
SET_DESTS_PINS, SET_DESTS_X, SET_DESTS_Y, SET_DESTS_PINDIRS = 0b000, 0b001, 0b010, 0b100
SHIFT_PULL, SHIFT_BLOCK = 0b1000_0000, 0b0010_0000


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (OP_JMP << 13) | (JMP_ALWAYS << 5) | addr


def set_word(dest: int, data: int, *, side: int = 0, delay: int = 0, sideset_bits: int = 1) -> int:
    """A `set` word with the side-set/delay field split the way the engine will read it.

    With `SIDESET_COUNT = sideset_bits` and `SIDE_EN = 0`, the top `sideset_bits` bits of
    bits 12:8 are side-set data and the rest is the delay (docs/mvp-isa.md
    `### Delay/side-set field, bits 12:8`), so `side 1` is the top bit set and the delay is
    the low `5 - sideset_bits` bits.
    """
    if not 0 <= side < (1 << sideset_bits):
        raise ValueError(f"side {side} does not fit in {sideset_bits} side-set bit(s)")
    limit = 1 << (5 - sideset_bits)
    if not 0 <= delay < limit:
        raise ValueError(f"delay {delay} does not fit: with {sideset_bits} side-set bit(s) "
                         f"it is 0..{limit - 1}")
    field = (side << (5 - sideset_bits)) | delay
    return (OP_SET << 13) | (field << 8) | ((dest & 0b111) << 5) | (data & 0x1F)


def pull_word(*, side: int = 0, sideset_bits: int = 1) -> int:
    """`pull block`, optionally side-setting.  Block is pioasm's default spelling."""
    field = side << (5 - sideset_bits)
    return (OP_SHIFT << 13) | (field << 8) | SHIFT_PULL | SHIFT_BLOCK


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def clkdiv_for_mdc(mdc_hz: float, sm_cycles_per_bit: int) -> int:
    """The CLKDIV word that puts MDC at `mdc_hz` for a bit of `sm_cycles_per_bit` SM cycles.

    One bit must last CORE_HZ / mdc_hz core cycles and one SM cycle lasts one CLKDIV period,
    so the divisor wanted is CORE_HZ / (mdc_hz * sm_cycles_per_bit), rounded into INT and
    FRAC (docs/mvp-isa.md `### Clock divider`).  The rate is exact on average, so MDC keeps
    the nominal grid to within one core cycle per SM cycle.

    Anything above 2.5 MHz is refused rather than rounded into an illegal bus: 400 ns is the
    shortest legal MDC period, so a station that asks for less is not making an MDIO frame,
    it is making a violation (verif/vip/mdio.py `MdioChecker` rule `MDC`).
    """
    if mdc_hz <= 0:
        raise ValueError(f"MDC rate must be positive (got {mdc_hz})")
    if sm_cycles_per_bit < 1:
        raise ValueError(f"a bit needs at least one SM cycle (got {sm_cycles_per_bit})")
    if mdc_hz > MAX_MDC_HZ * (1 + 1e-9):
        raise ValueError(
            f"{mdc_hz:.6g} Hz is above the {MAX_MDC_HZ:.6g} Hz ceiling: clause 22 puts a "
            f"{MIN_PERIOD_NS:.0f} ns floor on the MDC period (LAN8720A Table 5.12 `tclkp`)"
        )
    want = CORE_HZ / (mdc_hz * sm_cycles_per_bit)
    if want > 0xFFFF:
        raise ValueError(
            f"{mdc_hz:.6g} Hz needs a CLKDIV INT of {want:.1f}, above the 16-bit maximum"
        )
    integer = max(1, int(want))
    frac = round((want - integer) * 256)
    if frac > 0xFF:                       # rounding pushed it past the next integer
        integer, frac = integer + 1, 0
    return clkdiv_value(integer, frac)


def ticks_per_bit(clkdiv: int, sm_cycles_per_bit: int) -> float:
    """The average bit length in core cycles (ticks) a CLKDIV word produces."""
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    return sm_cycles_per_bit * (integer + ((clkdiv >> 8) & 0xFF) / 256.0)


# --------------------------------------------------------------------------------------------
# The program: assemble it, and read back what the words say about its timing
# --------------------------------------------------------------------------------------------

def load_program(path: Path | str = PROGRAM_PATH) -> Program:
    """Assemble program.pio (asm/pioasm.py) and return its one program."""
    programs = assemble(Path(path).read_text(encoding="utf-8"))
    names = [p.name for p in programs]
    if names != [PROTO_NAME]:
        raise ValueError(f"{path}: expected one .program named {PROTO_NAME}, got {names}")
    return programs[0]


def decode_word(program: Program, word: int) -> dict:
    """Split one word into the fields `docs/mvp-isa.md` `## Encoding` defines.

    Only what this driver needs: the opcode, the side-set bits and the delay, whose split
    depends on PINCTRL.SIDESET_COUNT rather than on the instruction.  `sc` counts the `opt`
    enable bit, so `sideset` is the data bits and `delay` the low `5 - sc` bits.
    """
    sc = min(program.sideset_bits, 5)
    field = (word >> 8) & 0x1F
    return {
        "op": word >> 13,
        "sideset": field >> (5 - sc) if sc else 0,
        "delay": field & ((1 << (5 - sc)) - 1),
        "low": word & 0xFF,
    }


def bit_cycles(program: Program) -> int:
    """SM cycles one bit time lasts, taken from the assembled words rather than assumed.

    The program spends one bit time in three instructions: the low-phase instruction, the
    high-phase instruction and the `jmp` back to the loop.  So a bit is
    `(1 + low delay) + (1 + high delay) + 1`, and CLKDIV turns that into core cycles.  The
    three delays are read off the loop and checked to be the same everywhere, because a bit
    time that is not uniform would make CLKDIV mean two different things.
    """
    lows, highs = set(), set()
    for word in program.instrs:
        fields = decode_word(program, word)
        # `out pins, 1` and `in pins, 1` are the low phase of a bit time (OP 011 or 010 with
        # PINS as the operand); `nop`, which is `mov y, y`, is the high phase.
        is_low = fields["op"] in (0b011, 0b010) and (fields["low"] >> 5) == 0b000
        is_high = fields["op"] == 0b101 and fields["low"] == 0x42
        if is_low and fields["sideset"] == 0:
            lows.add(fields["delay"])
        elif is_high and fields["sideset"] == 1:
            highs.add(fields["delay"])
    if not lows or not highs:
        raise ValueError(
            f"{PROTO_NAME}: no bit-time loop found in the assembled words; the driver cannot "
            f"derive a divisor from this program"
        )
    if len(lows) != 1 or len(highs) != 1:
        raise ValueError(
            f"{PROTO_NAME}: the bit times are not uniform: low delays {sorted(lows)}, high "
            f"delays {sorted(highs)}.  One CLKDIV cannot time two different bit lengths"
        )
    return (1 + lows.pop()) + (1 + highs.pop()) + 1


def check_program(program: Program) -> dict:
    """Check the shape of program.pio against what this driver assumes, and report it.

    The driver hard-codes one thing about the program: a frame is 64 bit times of three
    instructions, and `jmp !osre` counts one 32-bit word per pass.  Everything else it reads
    back out of the words.  This function returns the numbers a test asserts on and raises if
    the program no longer has the shape the driver's timing model needs.
    """
    if program.sideset_bits != 1 or program.sideset_opt:
        raise ValueError(
            f"{PROTO_NAME}: expected `.side_set 1` with no `opt` (one MDC level bit, every "
            f"instruction side-setting), got {program.sideset_bits} bit(s), "
            f"opt={program.sideset_opt}"
        )
    for label in ("halt", "write_frame", "read_frame"):
        if label not in program.labels:
            raise ValueError(f"{PROTO_NAME}: the driver needs a PUBLIC `{label}` label")
    if program.labels["halt"] != 0:
        raise ValueError(
            f"{PROTO_NAME}: `halt` must be at address 0, because SM_RESTART keeps PC = 0 and "
            f"the host relies on a freshly restarted engine idling with MDC parked high"
        )
    if program.wrap != len(program.instrs) - 1:
        raise ValueError(
            f"{PROTO_NAME}: WRAP_TOP must be the last instruction ({len(program.instrs) - 1}), "
            f"the assembler worked out {program.wrap}"
        )
    if words_used(program) > IMEM_SIZE:
        raise ValueError(
            f"{PROTO_NAME}: {words_used(program)} words do not fit the {IMEM_SIZE}-word IMEM"
        )
    return {
        "words": words_used(program),
        "bit_cycles": bit_cycles(program),
        "labels": dict(program.labels),
        "write_frame": program.labels["write_frame"],
        "read_frame": program.labels["read_frame"],
        "halt": program.labels["halt"],
    }


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


# --------------------------------------------------------------------------------------------
# Register values the program needs (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program) -> int:
    """EXECCTRL: the wrap window the assembler worked out, and SIDE_EN = 0.

    SIDE_EN has to be 0 because the program uses `.side_set 1` with no `opt`: with SIDE_EN = 1
    the single side-set bit would be the enable and `side 1` would side-set nothing at all
    (docs/mvp-isa.md `### Delay/side-set field`).  The wrap window covers the whole program,
    and the program never relies on it: both frames end in `jmp halt`.
    """
    return (program.wrap << 12) | 0        # WRAP_BOTTOM = 0


def shiftctrl_value(*, push_thresh: int = 18, out_left: int = 1, in_left: int = 1,
                    autopush: int = 1) -> int:
    """SHIFTCTRL: shift both ways left, autopush at 18 bits, no autopull.

    `OUT_SHIFTDIR = 0` (shift left) is what makes `out pins, 1` send OSR bit 31 first, so the
    host packs each word with the frame's first bit in bit 31: MDIO is MSB first, as clause
    22 is.  `IN_SHIFTDIR = 0` makes the ISR hold the sampled bits in the same order.

    `AUTOPULL = 0` is what lets `jmp !osre` count a word: with autopull on, the refill would
    reset `osr_cnt` to 0 and the loop would never end.  The program fetches each word with an
    explicit `pull block` instead.

    `PUSH_THRESH = 18` is the read's sample count: 18 samples are TA1, TA2 and DATA[15:0],
    so the eighteenth one autopushes one RX word whose bits 15:0 are the register value
    (docs/mvp-isa.md `### Instruction semantics`, IN).  PUSH_THRESH is 5 bits, so 18 is the
    largest count that is not the reset value 32.
    """
    if not 1 <= push_thresh <= 31:
        raise ValueError(f"PUSH_THRESH {push_thresh} out of range 1..31")
    pull_thresh = 0                        # field 0 means thresh(PULL_THRESH) = 32
    return ((pull_thresh << 25) | (push_thresh << 20)
            | ((1 - out_left) << 19) | ((1 - in_left) << 18) | autopush << 16)


def pinctrl_value(mdc_pin: int, mdio_pin: int, *, sideset_count: int = 1) -> int:
    """PINCTRL: side-set on MDC alone, OUT/IN on MDIO alone, SET on both.

    `sideset_count` must be the assembler's `sideset_bits`, which is 1 here.  The reset value
    is 2 with SIDE_EN = 0, where every instruction would side-set two pins and the side-set
    channel would beat OUT/SET on the shared pin, which would hold MDC low
    (docs/mvp-isa.md `### Delay/side-set field`, `### Pin writes`).

    SET_COUNT = 2 with SET_BASE = mdc_pin is what lets one `set pindirs, n` write both
    directions, with MDC at data bit 0 so that `set pindirs, 1` means "MDC stays driven, MDIO
    is released"; the program needs exactly that to hand the bus to the PHY for a read.
    """
    if mdc_pin not in UIO_PINS or mdio_pin not in UIO_PINS:
        raise ValueError(
            f"MDC={mdc_pin} and MDIO={mdio_pin} must both be uio pins {list(UIO_PINS)}: MDIO is "
            f"bidirectional and only a uio pin has a direction register"
        )
    if mdio_pin != mdc_pin + 1:
        raise ValueError(
            f"mdio_pin must be mdc_pin + 1 ({mdc_pin + 1}), got {mdio_pin}: the program's "
            f"`set pindirs, 1` releases MDIO and holds MDC only because MDC is the lower pin "
            f"of the two-pin SET window"
        )
    return ((sideset_count << 29) | (2 << 26) | (1 << 20) | (mdio_pin << 15)
            | (mdc_pin << 10) | (mdc_pin << 5) | mdio_pin)


# --------------------------------------------------------------------------------------------
# The words a frame is made of
# --------------------------------------------------------------------------------------------

def control_word(is_write: bool, phyad: int, reg: int, data: int = 0) -> int:
    """The second TX word of a frame: ST, OP, PHYAD, REGAD, TA and DATA, MSB first.

    `OUT_SHIFTDIR = 0` shifts the OSR left, so `out pins, 1` sends bit 31 first and the frame's
    bit 32 lands on the wire first.  That fixes the packing (frame bit 32 = word bit 31):

        31:30 ST = 01        29:28 OP = 01 write / 10 read
        27:23 PHYAD[4:0]     22:18 REGAD[4:0]
        17:16 TA  = 10 write / released on a read
        15:0  DATA[15:0]

    The turnaround of a read is `<Z Z>`: the station must not drive the first turnaround bit
    and the PHY drives the second, so both bits of `ta` are left at zero and the program has
    already released the pin by then (`research/ref/protocols/mdio/ti-dp83tc811-smi.txt`,
    Table 7-15).  Zero is the right filler because the pull-up reads as 1.
    """
    for name, value, limit in (("phyad", phyad, (1 << ADDRESS_BITS) - 1),
                               ("reg", reg, (1 << REGISTER_BITS) - 1),
                               ("data", data, (1 << DATA_BITS) - 1)):
        if not 0 <= value <= limit:
            raise ValueError(f"{name} {value!r} does not fit in {limit + 1} bits")
    ta = TA_WRITE if is_write else 0
    op = OP_WRITE if is_write else OP_READ
    return ((ST_BITS << 30) | (op << 28) | (phyad << 23) | (reg << 18) | (ta << 16) | data)


def frame_words(is_write: bool, phyad: int, reg: int, data: int = 0) -> list[int]:
    """The two TX words one frame needs: the 32 preamble ones, then `control_word`.

    Two words is not an optimisation, it is what the program counts: `jmp !osre` runs while
    `osr_cnt < thresh(PULL_THRESH)` and PULL_THRESH is 32, so one word is exactly one pass of
    the bit loop and a clause 22 frame is exactly two of them (32 preamble + 32 control/data).
    """
    return [PREAMBLE_WORD, control_word(is_write, phyad, reg, data if is_write else 0)]


def read_data(rx_word: int) -> int:
    """The register value inside the RX word one read pushed.

    The ISR collects the 18 samples of the turnaround and data phase in wire order, so the
    word is `{TA1, TA2, DATA[15:0]}`: DATA is in bits 15:0 and the two turnaround bits above
    it are the pull-up's 1 and the PHY's 0 (program.pio, `tdata`).
    """
    return rx_word & 0xFFFF


@dataclass
class MdioOp:
    """One queued or executed clause 22 access."""

    is_write: bool
    reg: int
    phyad: int = 0
    value: Optional[int] = None       # what was written, or what came back
    start: Optional[int] = None       # bus tick of the frame's first MDC rising edge
    end: Optional[int] = None         # ... and of its 64th
    errors: list = field(default_factory=list)

    @property
    def unanswered(self) -> bool:
        """A read that came back as 0xffff: with the pull-up high and nobody driving, no PHY
        on the bus claimed the address (clause 22's way of saying "no PHY there")."""
        return not self.is_write and self.value == 0xFFFF

    def __str__(self) -> str:
        verb = "write" if self.is_write else "read"
        tail = "" if self.value is None else f" 0x{self.value:04x}"
        if self.unanswered:
            tail += " (no PHY answered)"
        return f"{verb} PHYAD={self.phyad} REGAD={self.reg}{tail}"


# --------------------------------------------------------------------------------------------
# Watching the bus: how the host knows a frame is over
# --------------------------------------------------------------------------------------------

class MdioBusMonitor(Device):
    """A passive counter on the two nets: MDC rising edges, and the bit sampled on each.

    This is how the host knows a frame is finished without looking inside the chip: clause 22
    has no end-of-frame signal, so a station counts MDC periods, exactly as `MdioStation` and
    `MdioChecker` do.  It drives nothing, so attaching it cannot change what the chip sees
    (`verif/vip/core.py` `Device`), and a test can use its `bits` to decode frames without
    asking the checker.

    `frames` is the number of complete 64-bit-time frames seen so far, and `frame(n)` is the
    `n`-th frame's bits, most significant bit first, as the wire carried them.
    """

    name = "mdio_mon"

    def __init__(self, mdc_net: str = "mdc", mdio_net: str = "mdio",
                 frame_bits: int = FRAME_BITS, keep_bits: int = 0):
        self.mdc_net, self.mdio_net = mdc_net, mdio_net
        self.frame_bits = frame_bits
        #: How many frames' worth of bits to keep (0 = none, which is all the driver needs).
        self.keep_bits = keep_bits
        self.prev_mdc = 1
        self.rises = 0
        self.bits: list = []

    @property
    def frames(self) -> int:
        """Complete frames seen: one per `frame_bits` MDC rising edges."""
        return self.rises // self.frame_bits

    def frame(self, index: int) -> list:
        """The bits of frame `index`, MSB first, as sampled on each MDC rising edge."""
        start = index * self.frame_bits
        if self.keep_bits and start + self.frame_bits > len(self.bits):
            raise ValueError(
                f"frame {index} is not in the log: {self.keep_bits} frame(s) kept, "
                f"{len(self.bits) // self.frame_bits} seen"
            )
        return self.bits[start:start + self.frame_bits]

    def tick(self, t: int, pins: Mapping[str, int]) -> dict:
        mdc, mdio = pins[self.mdc_net], pins[self.mdio_net]
        if self.prev_mdc == 0 and mdc == 1:
            self.rises += 1
            if self.keep_bits and self.rises <= self.keep_bits * self.frame_bits:
                self.bits.append(mdio)
        self.prev_mdc = mdc
        return {}


# --------------------------------------------------------------------------------------------
# One configuration: which engine, which pins, how fast
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Config:
    """One MDIO setup: which engine, which pins, and the MDC rate."""

    mdc_hz: float = MAX_MDC_HZ        # 2.5 MHz, the clause 22 ceiling
    engine: int = 0
    mdc_pin: int = 0
    mdio_pin: int = 1

    @property
    def mdc_net(self) -> str:
        return f"p{self.mdc_pin}"

    @property
    def mdio_net(self) -> str:
        return f"p{self.mdio_pin}"


class MdioDriver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as a clause 22 station.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are queued
    on `dev` and land one per tick as the simulation runs, so `read()`, `write()` and `run()`
    pump the bus while they wait.

    One frame is four forced instructions and two pushes, in this order, because the host cannot
    write the OSR, the PC or the pin directions any other way (docs/mvp-regmap.md
    `### Typical host sequences`, docs/mvp-isa.md `### EXEC`):

    1. push the two TX words (`frame_words`), so the frame never waits on an empty FIFO;
    2. force `set pindirs, 3` -- the regmap's step 4, repeated every frame because a read leaves
       MDIO released for its turnaround and the next frame has to drive it again;
    3. force `set y, 0`, the write frame's word counter, which the last frame left at
       0xffffffff;
    4. force `pull block`, which loads the first word into the OSR even though the divider and
       `SM_ENABLE` say nothing should run;
    5. force `jmp write_frame` or `jmp read_frame`, the only way to set the PC, and run the bus
       until `MdioBusMonitor` has counted 64 MDC rising edges.

    Steps 2 to 5 all carry `side 1`, so MDC never moves while the host is setting the frame up:
    a side-set on a forced instruction is real (docs/mvp-isa.md `### EXEC`), and an edge in the
    middle of this sequence would be a 65th MDC rising edge in the middle of a 64-bit-time frame.

    The steps need MDC parked high, or the bus would show an MDC low phase with nothing clocked
    (the checker's `idle` rule), so `GAP_TICKS` of idle pass between frames and before the
    first one.  They are not dead time in the protocol: clause 22 puts no maximum time between
    edges, and every frame here carries its own 32-bit preamble, which is what the spec asks
    for after an idle gap anyway.
    """

    MAX_TICKS = 2_000_000

    @property
    def TXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)
    #: Idle ticks the driver leaves between frames.  The floor the checker enforces is
    #: `t_high` = 160 ns = 8 ticks, and the frame's own first low phase is 10, so 16 leaves
    #: room for the two forced instructions and the host accesses around them.
    GAP_TICKS = 16
    #: A frame is `frame_words` long; the program pulls the second word itself.
    WORDS_PER_FRAME = 2

    def __init__(self, dev, bus, *, engine: int = 0, mdc_pin: int = 0, mdio_pin: int = 1,
                 mdc_hz: float = MAX_MDC_HZ, program: Optional[Program] = None,
                 path: Path | str = PROGRAM_PATH, monitor: Optional[MdioBusMonitor] = None
                 ) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        self.dev, self.bus = dev, bus
        self.engine, self.pin_pair = engine, (mdc_pin, mdio_pin)
        self.program = program if program is not None else load_program(path)
        self.shape = check_program(self.program)
        self.config = Config(mdc_hz=mdc_hz, engine=engine, mdc_pin=mdc_pin, mdio_pin=mdio_pin)
        # Check the pin pair and the rate now rather than at the first frame: PINCTRL is a
        # property, and a driver that only complained when it was loaded would have written
        # half a configuration by then.
        self.pinctrl, self.clkdiv
        self.monitor = monitor if monitor is not None else MdioBusMonitor(
            self.config.mdc_net, self.config.mdio_net)
        #: Frames queued but not yet run, and how many have run.
        self.queue: list[MdioOp] = []
        self.done: list[MdioOp] = []
        self.frames_run = 0
        self.loaded = False

    # ---- addresses ----
    @property
    def imem_base(self) -> int:
        return A_INSTR_MEM[self.engine]

    def sm_addr(self, index: int) -> int:
        return A_SM_BASE + 6 * self.engine + index

    # ---- the register values this configuration asks for ----
    @property
    def mdc_hz(self) -> float:
        return self.config.mdc_hz

    @property
    def sm_cycles_per_bit(self) -> int:
        return self.shape["bit_cycles"]

    @property
    def clkdiv(self) -> int:
        return clkdiv_for_mdc(self.mdc_hz, self.sm_cycles_per_bit)

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value()

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(*self.pin_pair, sideset_count=self.program.sideset_bits)

    @property
    def words_used(self) -> int:
        return self.shape["words"]

    @property
    def ticks_per_bit(self) -> float:
        """The bit time this configuration produces, in core cycles."""
        return ticks_per_bit(self.clkdiv, self.sm_cycles_per_bit)

    # ---- host accesses: queued on the ChipDevice, one per tick ----
    def write(self, addr: int, data: int) -> None:
        """Queue one host write: it takes effect at the end of its bus access cycle."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def _settle(self, extra: int = 2) -> None:
        """Run the bus until the queue is empty, then `extra` more ticks."""
        ticks = 0
        while not self.dev.idle:
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host writes still queued after {ticks} ticks")
        for _ in range(extra):
            self.bus.step()

    def read(self, addr: int) -> int:
        """Queue one host read and run the bus until its data comes back.

        The value is the register's state in the access cycle, so a FIFO pop or a W1C happens
        as this returns (docs/mvp-regmap.md `### Cycle-exact mapping to bus accesses`).
        """
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host read of {addr:#04x} unanswered after {ticks} ticks")
        return int(future.result())

    # ---- register reads ----
    def ctrl(self) -> int:
        return self.read(A_CTRL)

    def fstat(self) -> int:
        return self.read(A_FSTAT)

    def flevel(self) -> int:
        return self.read(A_FLEVEL)

    def fdebug(self) -> int:
        return self.read(A_FDEBUG)

    def clear_fdebug(self) -> int:
        """Write FDEBUG back to clear the sticky stall flags (W1C)."""
        value = self.fdebug()
        if value:
            self.write(A_FDEBUG, value)
            self._settle()
        return value

    def padout(self) -> int:
        return self.read(A_DBG_PADOUT)

    def padoe(self) -> int:
        return self.read(A_DBG_PADOE)

    def cfginfo(self) -> int:
        return self.read(A_DBG_CFGINFO)

    def pc(self) -> int:
        return self.read(self.sm_addr(SM_ADDR))

    # ---- the startup sequence (docs/mvp-regmap.md `### Typical host sequences`) ----
    def init_pins(self) -> None:
        """Force `set pins, 3` then `set pindirs, 3`: park both lines high and drive them.

        Step 4 of the startup sequence, done first of all.  Until something drives them, MDC
        and MDIO are inputs whose level registers read 0, and a bus with MDC low and nothing
        clocking it is the checker's `idle` rule.  Parked high, the bus is idle in the state
        clause 22 asks for: "MDIO pin requires a pullup resistor, which pulls MDIO high during
        IDLE and turnaround" (`research/ref/protocols/mdio/ti-dp83tc811-smi.txt`).

        **Levels before directions**, and that order is not cosmetic.  A pin's level register
        is 0 until a `set pins` writes it, so `set pindirs, 3` first would make the chip start
        *driving* 0 on both lines: the pull-up loses, MDC falls for the two ticks before the
        next forced instruction, and the bus shows an MDC low phase with nothing clocked in it
        plus an MDIO change next to it.  That is three checker violations (`MDC-half`, `tSU`,
        `tH`) and one spurious frame, out of two instructions.

        The reset PINCTRL has `SET_BASE = 0` and `SET_COUNT = 5`, which covers the two pins the
        same way the driver's own PINCTRL does, so these two words mean the same thing before
        and after the register write.
        """
        sc = self.program.sideset_bits
        self.force(set_word(SET_DESTS_PINS, 0b11, side=1, sideset_bits=sc))
        self.force(set_word(SET_DESTS_PINDIRS, 0b11, side=1, sideset_bits=sc))

    def load(self) -> None:
        """Steps 1-4: stop the engine, burst-write the program, set the four registers, park.

        The engine is left disabled, so nothing runs while the host is still writing IMEM: a
        host IMEM write reaches execution two cycles later, and an engine that ran the program
        while it was half written would clock MDC on whatever it found in IMEM.
        """
        self.stop()
        for k, word in enumerate(self.program.instrs):
            self.write(self.imem_base + k, word)
        self.write(self.sm_addr(SM_CLKDIV), self.clkdiv)
        self.write(self.sm_addr(SM_EXECCTRL), self.execctrl)
        self.write(self.sm_addr(SM_SHIFTCTRL), self.shiftctrl)
        self.write(self.sm_addr(SM_PINCTRL), self.pinctrl)
        self._settle()
        # PINCTRL before the pins, not after: the forced `set pins`/`set pindirs` go through
        # the SET window, and the reset window is `SET_BASE = 0, SET_COUNT = 5`.  With MDC and
        # MDIO on pins 0 and 1 that window happens to cover them, but on any other pair -- say
        # MDC p5 and MDIO p6 -- it does not, and the forced `set pindirs, 3` would then start
        # driving a pin whose level register is still 0.  MDIO driven low with the bus idle is
        # the checker's `release` rule, and it happens long before the first frame would fix it.
        self.init_pins()
        self.loaded = True

    def enable(self) -> None:
        """Step 6: SM_ENABLE with SM_RESTART and CLKDIV_RESTART in the same CTRL write.

        The engine's first SM cycle is then two core cycles after this write
        (docs/mvp-regmap.md `### Clock divider`).  PC is 0 after the restart, which is where
        `halt` is, so the engine parks with MDC high and the host can queue frames at leisure.
        """
        ctrl = self.read(A_CTRL) & ~self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, ctrl | self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def start(self) -> None:
        """`load()` then `enable()`, in that order: the usual way to bring the link up."""
        self.load()
        self.enable()

    def stop(self) -> None:
        """Disable the engine, keeping every register (docs/mvp-isa.md `### Enable, restart`)."""
        ctrl = self.read(A_CTRL)
        self.write(A_CTRL, ctrl & ~self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def restart(self) -> None:
        """SM_RESTART + CLKDIV_RESTART, keeping the enable."""
        ctrl = self.read(A_CTRL)
        keep = self.ctrl_bits(CTRL_SM_ENABLE)
        self.write(A_CTRL, ctrl | (self.ctrl_bits(*self.CTRL_BITS) & ~keep))
        self._settle()

    # ---- CTRL helpers ----
    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask

    # ---- the TX FIFO ----
    def push_word(self, word: int) -> None:
        """Push one TX word, waiting for room first so no word is dropped (TXOVER)."""
        while self.fstat() & (FSTAT_TXFULL << self.engine):
            self._settle(1)
        self.write(A_TXF[self.engine], word)
        self._settle(1)

    def push_words(self, words: Sequence[int]) -> None:
        """Push several words, in order, waiting for FIFO room between them."""
        for word in words:
            self.push_word(word)

    def force(self, word: int, settle: int = 2) -> None:
        """Force one instruction through `SMe_INSTR` (docs/mvp-isa.md `### EXEC`).

        A host write performed in cycle `h` executes in `h+1` whatever the divider and
        `SM_ENABLE` say, it does not advance the PC, and its delay is ignored -- which is why
        the delay field of the words this method sends is 0.  Its side-set does apply, so the
        words carry `side 1` where MDC must stay parked.
        """
        self.write(self.sm_addr(SM_INSTR), word)
        self._settle(settle)

    # ---- one frame ----
    def words_for(self, op: MdioOp) -> list:
        """The TX words one op needs.  A seam: a test subclasses this to put a wrong frame on
        the wire and check that `MdioChecker` names the rule it broke."""
        return frame_words(op.is_write, op.phyad, op.reg, op.value if op.is_write else 0)

    def queue_write(self, reg: int, value: int, phyad: int = 0) -> MdioOp:
        """Queue a write of `value` to `reg` on the PHY at `phyad`; nothing runs yet."""
        frame_words(True, phyad, reg, value)          # range-check now, not mid-frame
        op = MdioOp(is_write=True, reg=reg, value=value & 0xFFFF, phyad=phyad)
        self.queue.append(op)
        return op

    def queue_read(self, reg: int, phyad: int = 0) -> MdioOp:
        """Queue a read of `reg` on the PHY at `phyad`; nothing runs yet."""
        frame_words(False, phyad, reg)                 # range-check now, not mid-frame
        op = MdioOp(is_write=False, reg=reg, phyad=phyad)
        self.queue.append(op)
        return op

    def run(self, limit: Optional[int] = None) -> list:
        """Run every queued frame in order and return the ops, reads filled in.

        `limit` runs only the first `limit` of them, which is what a test needs when it wants
        to look at the bus between two frames (the soft-reset window, say).  Frames do not
        depend on each other: each one pushes its own two words and forces its own `pull block`,
        so the OSR is empty or full at the start of a frame and never in between
        (program.pio, `jmp !osre`).
        """
        if not self.loaded:
            raise RuntimeError("run() before load(): the engine has no program and no pins")
        todo = self.queue if limit is None else self.queue[:limit]
        if not todo:
            return []
        for op in todo:
            self.run_one(op)
            self.done.append(op)
            self.frames_run += 1
        del self.queue[:len(todo)]
        return list(todo)

    def run_one(self, op: MdioOp) -> MdioOp:
        """Put one frame on the wire: two words, a forced `pull block`, a forced `jmp`."""
        entry = "write_frame" if op.is_write else "read_frame"
        if self.monitor.rises % FRAME_BITS:
            # A frame is 64 MDC rising edges and nothing else may put an edge on this bus.
            # If one did, every later frame count would be short by one and the driver would
            # read each RX word a frame late, so say so instead of decoding nonsense.
            raise RuntimeError(
                f"{self.monitor.rises} MDC rising edges so far, not a whole number of "
                f"{FRAME_BITS}-bit frames: something else is clocking this bus"
            )
        self.bus.run(self.GAP_TICKS)                   # MDC parked high between frames
        self.push_words(self.words_for(op))
        # The host owns the pin directions (docs/mvp-regmap.md `### Typical host sequences`
        # step 4): a read releases MDIO for its turnaround, so the next frame has to drive it
        # again.  MDC is parked high while this happens, so the forced `set pindirs` makes no
        # edge of its own -- see program.pio on why the frame must not start with it.
        self.force(set_word(SET_DESTS_PINDIRS, 0b11, side=1,
                            sideset_bits=self.program.sideset_bits))
        # Y is the write frame's word counter: `jmp y--` at the end of the first word tests
        # it and wraps it to 0xffffffff, which is what makes the second word the last one.
        # The host primes it per frame, for the same reason it primes the OSR: a frame has to
        # start from a known state, and SM_RESTART keeps X and Y (docs/mvp-isa.md
        # `### Engine state`).
        self.force(set_word(SET_DESTS_Y, 0, side=1, sideset_bits=self.program.sideset_bits))
        self.force(pull_word(side=1, sideset_bits=self.program.sideset_bits))
        op.start = self.monitor.rises + 1
        self.force(jmp_abs(self.program.labels[entry]), settle=1)
        self._wait_frame(op)
        if not op.is_write:
            op.value = read_data(self.read(A_RXF[self.engine]))
        return op

    def _wait_frame(self, op: MdioOp) -> None:
        """Run the bus until the monitor has counted this frame's 64 MDC rising edges."""
        target = self.monitor.frames + 1
        ticks = 0
        while self.monitor.frames < target:
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(
                    f"the frame {op} never finished: {self.monitor.rises} MDC rising edges "
                    f"after {ticks} ticks, engine PC {self.pc()}"
                )
        op.end = self.monitor.rises
        op.start = self.monitor.rises - FRAME_BITS + 1
        self._settle(2)

    # ---- one-call access: queue a frame and run it ----
    # Named `*_reg` rather than `write`/`read`, which stay the raw host register accesses
    # every method above uses.
    def write_reg(self, reg: int, value: int, phyad: int = 0) -> MdioOp:
        """Queue one write frame and run it; the op carries the value that was sent."""
        op = self.queue_write(reg, value, phyad)
        self.run()
        return op

    def read_reg(self, reg: int, phyad: int = 0) -> MdioOp:
        """Queue one read frame and run it; the op's `value` is what came back."""
        op = self.queue_read(reg, phyad)
        self.run()
        return op

    # ---- what the chip is doing ----
    def halted(self) -> bool:
        """True when the engine is parked in `halt`, which is where a frame leaves it."""
        return self.pc() == self.program.labels["halt"]

    def report(self) -> str:
        """A one-paragraph summary a test or a log can print."""
        shape = self.shape
        return (f"mdio: {shape['words']} of {IMEM_SIZE} IMEM words, "
                f"{shape['bit_cycles']} SM cycles per bit, CLKDIV {self.clkdiv:#010x} "
                f"= {self.ticks_per_bit:.3f} ticks = {CORE_HZ / self.ticks_per_bit:.6g} Hz MDC, "
                f"engine {self.engine}, MDC p{self.pin_pair[0]} / MDIO p{self.pin_pair[1]}, "
                f"{self.frames_run} frame(s) run")

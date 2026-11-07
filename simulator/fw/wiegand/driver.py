"""Host side of the MVP Wiegand firmware: load program.pio, set up an engine, send cards.

Everything here is written against `docs/mvp-regmap.md` (register word
addresses, the `### Typical host sequences` startup order) and `docs/mvp-isa.md` (divider,
open drain, autopull, side-set).  Host accesses go through the write/read queue of
`iss.pio_iss.ChipDevice`, one per core cycle, so every wait here is the simulation running --
the same thing a real host does over the SPI port of `docs/mvp-regmap.md`.

    bus = Bus(ns_per_tick=20)                       # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())
    drv = WiegandDriver(dev, bus, d0_pin=0, pw_ns=50_000)
    drv.load()                 # IMEM, CLKDIV, EXECCTRL, SHIFTCTRL, PINCTRL
    drv.start()                # forces `jmp send`; the engine parks in `idle`
    drv.send_card(123, 4567)   # two TX words, one frame gap later the other end can read it
    bus.run(drv.frame_ticks + drv.frame_gap_ticks)

Which side of the wire the chip plays
-------------------------------------
The **reader**: the box at the door, which is the side that initiates.  There is no clock, no
handshake and no reply -- a reader emits 26 low pulses on two open-drain lines and then goes
quiet, and the only thing that tells a panel where the frame ended is the quiet
(research/ref/protocols/wiegand/05_accessgrid_26bit_decode.md).  So the chip drives the bus
and `verif/vip/wiegand.py`'s `WiegandReceiver` (the panel, which decodes facility code, ID
number and both parity bits) and `WiegandChecker` (an analyser, which judges every timing rule)
watch it.  The model that *sends* cards, `WiegandReader`, is the counterpart of this firmware
rather than its judge, so the firmware tests use the decoder and the checker.

The credential itself is the host's, not the engine's
------------------------------------------------------
A real reader reads the card and formats the frame itself; this chip has no card input at all,
so the host supplies the facility code and ID number over the SPI port and the engine is the
wire-timing machine: it shifts out direction pairs at the pulse widths and pulse intervals the
SIA window allows.  The frame layout and both parity bits are computed here, from
research/ref/protocols/wiegand/ (Pyramid's PR-25 Table 2), and *not* from
`verif/vip/wiegand.py`, so that the model decoding what reaches the wire is an independent
second implementation of the same table.  `test_wiegand.py` cross-checks the two tables against
each other in one direction only, so a transcription error in either shows up as a parity or
frame-length violation rather than cancelling out.

Open drain, on a chip whose pins have no output-enable inversion
-----------------------------------------------------------------
`docs/mvp-isa.md` `### Open drain` fixes the sense: the level register of an open-drain pin
stays 0 and the *direction* is what moves, `set pindirs, 1` pulling the line low and
`set pindirs, 0` releasing it.  So the program writes both directions (2 uio pins, D1 = D0 + 1)
and never a level, and this module has to get two register values right for that to work:
`PINCTRL.SIDESET_COUNT = 0`, because the reset value 2 with `SIDE_EN = 0` makes *every*
instruction side-set level 0 into `SIDESET_BASE` and its neighbour on every SM cycle and the
side-set channel wins over the OUT channel (`docs/mvp-isa.md` `### Pin writes`); and
`OUT_COUNT = SET_COUNT = 2`, because `set pindirs, 0` is what releases the pause between two
pulses and it has to reach both lines.

The timing, and what it can and cannot reach
--------------------------------------------
`pw_ns` is the pulse width the host asks for; the divider follows from it and from the two
phase lengths the program itself declares, which this module reads back out of the assembled
words (`phase_cycles()`).  The window both phases have to sit in is transcribed from the SIA
AC-01 numbers quoted by three independent documents
(research/ref/protocols/wiegand/01_farpointe_understanding_wiegand.md), and
`clkdiv_for_pulse()` refuses a speed whose pulse *or* pause would fall outside it, so a
configuration that cannot be expressed is an error rather than a silent violation.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

from asm.pioasm import Program, assemble

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
TICK_NS = 20.0                # the chip's own clock, 50 MHz
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "wiegand"        # the .program name inside program.pio
MAX_TICKS = 8_000_000         # core cycles any one wait here may take

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

# Instruction field values (docs/mvp-isa.md `## Encoding`, `### Field values`)
OP_JMP, OP_WAIT, OP_IN, OP_OUT = 0b000, 0b001, 0b010, 0b011
OP_SHIFT, OP_MOV, OP_SET = 0b100, 0b101, 0b111
JMP_ALWAYS = 0b000
OUT_PINDIRS, OUT_PINS = 0b100, 0b000
SET_PINDIRS, SET_PINS = 0b100, 0b000
COND_Y_DEC = 0b100            # `jmp y--`
PULL = 0b100                  # bit 7 of the shift instructions: 1 = PULL


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address (docs/mvp-isa.md `## Encoding`)."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (OP_JMP << 13) | (JMP_ALWAYS << 5) | addr


def bit_timing(n_pw: int, n_pi: int, sm_cycle_ticks: float) -> tuple:
    """The ticks a pulse and the pause after it last on the wire.

    The pulse is the `out pindirs, 2` itself: `n_pw` SM cycles.  The pause is longer than the
    `set pindirs, 0` by one SM cycle, because the `jmp y-- loop` that runs between two pulses
    takes a cycle of its own (docs/mvp-isa.md `### Cycle accounting`: the next instruction
    executes `1 + delay` SM cycles later, and the loop has three instructions).  Getting this
    wrong is not a rounding error: the pause would be one SM cycle short of what the engine
    really does, which at CLKDIV 1250 is 50 us.
    """
    return n_pw * sm_cycle_ticks, (n_pi + 1) * sm_cycle_ticks


def divider_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def ticks_per_sm_cycle(clkdiv: int) -> float:
    """The core cycles (ticks) one SM cycle lasts, the average the 16.8 divider gives."""
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    return integer + ((clkdiv >> 8) & 0xFF) / 256.0


def ceil_ticks(ns: float, tick_ns: float = TICK_NS) -> int:
    """Nanoseconds to whole ticks, rounded up: what a *minimum* has to be measured against.

    The direction of the rounding is the point.  A floor is rounded up, or the host would
    believe it may emit a pulse one tick shorter than the specification allows.
    """
    return -int(-int(ns * 1000) // int(tick_ns * 1000))


# ---- the timing window (research/ref/protocols/wiegand/) ----

# The SIA AC-01 26-bit window, as quoted by Farpointe's *Understanding Wiegand* (pulse width
# 20 us .. 100 us, pulse interval 200 us .. 20 ms) and repeated by Pyramid's PR-25 note and by
# BALTECH: research/ref/protocols/wiegand/01_farpointe_understanding_wiegand.md,
# 02_pyramid_26bit_format.md, 04_baltech_wiegand.md.  This is a transcription of the cited
# numbers, deliberately *not* an import of verif/vip/wiegand.py: the firmware has to be able to
# refuse a speed the specification forbids without asking the judge that measures it.
# `test_wiegand.py` asserts the two tables agree, so neither can drift away from the sources
# unnoticed.
WIEGAND_WINDOW_NS = {
    "pw_min": 20_000, "pw_max": 100_000,        # tPW, the low pulse
    "pi_min": 200_000, "pi_max": 20_000_000,     # tPI, the pause between two pulses
}
# BALTECH's shipped reader defaults, PulseWidth 50 us and PulseInterval 2 ms
# (research/ref/protocols/wiegand/04_baltech_wiegand.md), and Pyramid's typical Tpw 100 us /
# Tpi 1 ms (02_pyramid_26bit_format.md Table 1).  Both are quoted so the tests can aim at a real
# reader's numbers; neither is a limit.
READER_DEFAULTS_NS = {"baltech_pw": 50_000, "baltech_pi": 2_000_000,
                      "pyramid_pw": 100_000, "pyramid_pi": 1_000_000}

# --------------------------------------------------------------------------------------------
# The program: assemble it, and read its shape back out of the words
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

    Only what this module needs: the opcode, the side-set bits and the delay, which are the
    fields whose split depends on PINCTRL.SIDESET_COUNT rather than on the instruction.  `sc`
    counts the `opt` enable bit, so the delay is the low `5 - sc` bits, as the spec says.
    """
    sc = min(program.sideset_bits, 5)
    field = (word >> 8) & 0x1F
    return {
        "op": word >> 13,
        "sideset": field >> (5 - sc) if sc else 0,
        "delay": field & ((1 << (5 - sc)) - 1),
        "dest": (word >> 5) & 0x7,
        "low": word & 0xFF,
    }


def is_out_pindirs(program: Program, word: int, count: int) -> bool:
    """True for `out pindirs, count`: the instruction that pulls one line low."""
    f = decode_word(program, word)
    return f["op"] == OP_OUT and f["dest"] == OUT_PINDIRS and (f["low"] & 0x1F) == count


def is_set_pindirs(program: Program, word: int, data: int) -> bool:
    """True for `set pindirs, data`: the instruction that releases (or pulls) a line."""
    f = decode_word(program, word)
    return f["op"] == OP_SET and f["dest"] == SET_PINDIRS and (f["low"] & 0x1F) == data


def phase_cycles(program: Program) -> tuple:
    """The two phase lengths of one frame bit, in SM cycles, taken from the assembled words.

    One bit is one `out pindirs, 2` (the pulse) followed by one `set pindirs, 0` (the pause),
    so a phase lasts `1 + delay` SM cycles (docs/mvp-isa.md `### Cycle accounting`).  Both are
    read out of the loop body rather than assumed, so CLKDIV cannot be derived from a program
    that says something else; anything unexpected raises instead of guessing.
    """
    for name in ("send", "loop", "idle"):
        if name not in program.labels:
            raise ValueError(f"{PROTO_NAME}: program.pio has no `{name}` label for the host "
                             f"or the loop to be found")
    start = program.labels["loop"]
    end = start                        # the loop body is what runs before its own `jmp`
    while end < len(program.instrs) and decode_word(program, program.instrs[end])["op"] != OP_JMP:
        end += 1
    body = program.instrs[start:end + 1]
    pulses = [w for w in body if is_out_pindirs(program, w, 2)]
    pauses = [w for w in body if is_set_pindirs(program, w, 0)]
    if len(pulses) != 1 or len(pauses) != 1:
        raise ValueError(
            f"{PROTO_NAME}: the bit loop must be one `out pindirs, 2` and one "
            f"`set pindirs, 0`, found {len(pulses)} and {len(pauses)}; the divider cannot be "
            f"derived from this program"
        )
    last = decode_word(program, body[-1])
    if last["op"] != OP_JMP or ((last["low"] >> 5) & 0x7) != COND_Y_DEC:
        raise ValueError(f"{PROTO_NAME}: the bit loop must end in `jmp y-- loop`")
    if program.sideset_bits:
        raise ValueError(
            f"{PROTO_NAME}: the program declares {program.sideset_bits} side-set bit(s), so "
            f"the delay field is shorter than 5 bits and the pulse and pause lengths this "
            f"module derives from the words would be wrong"
        )
    return decode_word(program, pulses[0])["delay"] + 1, decode_word(program, pauses[0])["delay"] + 1


def frame_pairs(program: Program) -> int:
    """How many direction pairs one frame is: `set y, n` at `send` means `n + 1` iterations."""
    word = program.instrs[program.labels["send"]]
    f = decode_word(program, word)
    if f["op"] != OP_SET or (f["low"] & 0x1F) > 31:
        raise ValueError(f"{PROTO_NAME}: `send` does not begin with a `set y, n`")
    return (f["low"] & 0x1F) + 1


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


def clkdiv_for_pulse(pw_ns: float, n_pw: int, n_pi: int, tick_ns: float = TICK_NS) -> int:
    """The CLKDIV word that makes the pulse `pw_ns` long, if that is expressible at all.

    One phase lasts `n` SM cycles and one SM cycle lasts `CLKDIV` core cycles, so the pulse is
    `n_pw * CLKDIV * tick_ns` and the pause `n_pi * CLKDIV * tick_ns` (docs/mvp-isa.md
    `### Clock divider`).  Asking for a pulse width therefore also fixes the pause, which is the
    one coupling this firmware cannot avoid: the divisor is shared.  What is left free is the
    ratio, which is fixed by the program's two delays -- at most 32, because the delay field is
    five bits, so a pause can be at most 32 SM cycles and a pulse at least one.

    The answer is only returned when both phases sit inside the SIA window; otherwise this
    raises, because a divider that puts a 5 us pulse or a 30 ms pause on the wire is not a
    working configuration but a protocol violation the checker would rightly flag.
    """
    if pw_ns <= 0:
        raise ValueError(f"a pulse width must be positive (got {pw_ns} ns)")
    if not 1 <= n_pw <= 32 or not 1 <= n_pi <= 32:
        raise ValueError(f"phase lengths {n_pw}/{n_pi} SM cycles are not 1..32")
    want = pw_ns / tick_ns / n_pw           # core cycles wanted per SM cycle
    if want > 0xFFFF:
        raise ValueError(
            f"a {pw_ns:.0f} ns pulse needs a CLKDIV of {want:.1f} core cycles per SM cycle, "
            f"above the 16-bit maximum of 65535"
        )
    integer = max(1, min(0xFFFF, int(want)))
    frac = round((want - integer) * 256)
    if frac > 0xFF:                        # rounding pushed it past the next integer
        integer, frac = integer + 1, 0
    clkdiv = divider_value(integer, frac)
    step = ticks_per_sm_cycle(clkdiv)
    pw, pi = (t * step * tick_ns for t in bit_timing(n_pw, n_pi, 1.0))
    window = WIEGAND_WINDOW_NS
    bad = []
    if not window["pw_min"] <= pw <= window["pw_max"]:
        bad.append(f"pulse {pw:.0f} ns")
    if not window["pi_min"] <= pi <= window["pi_max"]:
        bad.append(f"pause {pi:.0f} ns")
    if bad:
        raise ValueError(
            f"a {pw_ns:.0f} ns pulse with pauses of {n_pi} SM cycles puts the "
            f"{' and the '.join(bad)} outside the SIA window (pulse "
            f"{WIEGAND_WINDOW_NS['pw_min'] / 1000:.0f}..{WIEGAND_WINDOW_NS['pw_max'] / 1000:.0f} us, "
            f"pause {WIEGAND_WINDOW_NS['pi_min'] / 1000:.0f} us.."
            f"{WIEGAND_WINDOW_NS['pi_max'] / 1e6:.0f} ms, "
            f"research/ref/protocols/wiegand/01_farpointe_understanding_wiegand.md)"
        )
    return clkdiv


# --------------------------------------------------------------------------------------------
# The frame: the 26-bit layout, its parity, and the direction pairs the engine shifts out
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Layout:
    """One bit layout, as a table: where the parity bits, the facility code and the number sit.

    Positions are 1-based and counted the way the sources count them -- position 1 is the first
    bit on the wire -- and each field is a `(start, width)` pair laid down MSB first, which is
    what Pyramid's PR-25 Table 2 says.  This is a *second* transcription of that table: the
    reference model in verif/vip/wiegand.py has its own, written separately from the same
    sources, and the test checks the two agree rather than either being taken on trust.
    """

    name: str
    bits: int
    facility: tuple                       # (start, width), MSB first
    number: tuple                         # (start, width), MSB first
    lead: tuple                           # (position, "even" | "odd")
    trail: tuple
    lead_covers: tuple
    trail_covers: tuple


LAYOUTS = {
    # Pyramid's Table 2 verbatim (02_pyramid_26bit_format.md), which AccessGrid prints
    # identically (05_accessgrid_26bit_decode.md): bit 1 is even parity over bits 2..13, bits
    # 2..9 are the facility code, bits 10..25 the ID number, bit 26 is odd parity over bits
    # 14..25.  proxmark3's h10301 layout has the opposite polarity and a different field order
    # (06_proxmark3_h10301.c.md); a panel is only configurable for one of them, so this
    # firmware offers the Pyramid one and says so.
    "sia26": Layout(
        name="sia26", bits=26,
        facility=(2, 8), number=(10, 16),
        lead=(1, "even"), trail=(26, "odd"),
        lead_covers=tuple(range(2, 14)), trail_covers=tuple(range(14, 26)),
    ),
}
DEFAULT_LAYOUT = "sia26"


def layout(name: str = DEFAULT_LAYOUT) -> Layout:
    """Look up a layout by name, or raise with the list of the ones there are."""
    try:
        return LAYOUTS[name]
    except KeyError:
        raise ValueError(f"unknown Wiegand layout {name!r}; pick one of "
                         f"{sorted(LAYOUTS)}") from None


def parity_bit(bits: Sequence[int], kind: str) -> int:
    """The parity bit `kind` over `bits`: the value that makes the total number of ones even or
    odd.  The same definition the sources give and the model implements
    (verif/vip/wiegand.py `parity_bit`)."""
    ones = sum(1 for b in bits if b)
    if kind == "even":
        return ones & 1
    if kind == "odd":
        return 1 - (ones & 1)
    raise ValueError(f"parity must be 'even' or 'odd', got {kind!r}")


def frame_bits(facility: int, number: int, name: str = DEFAULT_LAYOUT) -> list:
    """The bits one frame puts on the wire for `(facility, number)`, first bit first.

    Both parity bits are computed here, on the host: the engine shifts out whatever pairs it is
    given, and the MVP has no instruction that can reduce twelve bits to a parity bit (see
    fw/wiegand/program.pio `## Why the data is two bits per frame bit`).
    """
    spec = layout(name)
    for value, (what, (_start, width)) in ((facility, ("facility code", spec.facility)),
                                           (number, ("ID number", spec.number))):
        if not 0 <= value < (1 << width):
            raise ValueError(f"{name}: {what} {value} does not fit in {width} bits")
    bits = [None] * spec.bits
    for (start, width), value in ((spec.facility, facility), (spec.number, number)):
        for k in range(width):
            bits[start - 1 + k] = (value >> (width - 1 - k)) & 1
    for (pos, kind), covers in ((spec.lead, spec.lead_covers),
                                (spec.trail, spec.trail_covers)):
        covered = [bits[c - 1] for c in covers]
        if any(b is None for b in covered):
            raise ValueError(f"{name}: parity at bit {pos} covers a bit the layout leaves open")
        bits[pos - 1] = parity_bit(covered, kind)
    if any(b is None for b in bits):
        raise ValueError(f"{name}: the layout leaves a bit of the frame undefined")
    return [int(b) for b in bits]


def direction_pairs(bits: Sequence[int]) -> list:
    """The frame bits as the pairs the engine shifts into the two pin directions.

    `out pindirs, 2` writes OSR[0] to D0 and OSR[1] to D1, and a direction of 1 pulls the line
    low (docs/mvp-isa.md `### Open drain`), so a data 0 is 0b01 and a data 1 is 0b10.  Both
    lines idle high because the other direction stays 0 in every pair: the program never drives
    a hard 1 against the pull-up, and the specification forbids both lines low at once
    (research/ref/protocols/wiegand/01_farpointe_understanding_wiegand.md).
    """
    pairs = []
    for bit in bits:
        if bit not in (0, 1):
            raise ValueError(f"a Wiegand bit is 0 or 1, got {bit!r}")
        pairs.append(1 if bit == 0 else 2)
    return pairs


def bits_from_pairs(pairs: Sequence[int]) -> list:
    """The frame bits a list of direction pairs would put on the wire: the inverse of
    `direction_pairs`, and what the test uses to check the pairs against the VIP's own encoder."""
    bits = []
    for pair in pairs:
        if pair not in (1, 2):
            raise ValueError(f"a direction pair is 1 (D0) or 2 (D1), got {pair!r}")
        bits.append(0 if pair == 1 else 1)
    return bits


def frame_words(bits: Sequence[int]) -> list:
    """The TX words the engine shifts a frame out of, in order.

    `OUT_SHIFTDIR = 1` (its reset value) takes the pair from the bottom of the OSR, so pair `i`
    of the frame sits in bits `2i+1:2i` of the word that carries it and the words come out of
    the TX FIFO in order.  A 26-bit frame is 52 OSR bits, so two words: 16 pairs and 10.
    """
    words, current, index = [], 0, 0
    for pair in direction_pairs(bits):
        current |= pair << (2 * index)
        index += 1
        if index == 16:                      # a word is 32 bits: 16 pairs
            words.append(current)
            current, index = 0, 0
    if index:
        words.append(current)
    return words


# --------------------------------------------------------------------------------------------
# Register values this configuration asks for (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program, *, side_en: int = 0, wrap_bottom: int = 0) -> int:
    """EXECCTRL: the wrap window the assembler worked out (`.wrap`), and no SIDE_EN.

    `SIDESET_COUNT` is 0 in this program, so there is never a side-set whatever SIDE_EN says
    (docs/mvp-isa.md `### Delay/side-set field`), and with it the whole five bits of the
    delay/side-set field are delay -- which is what lets a phase be 32 SM cycles long.
    """
    if side_en:
        raise ValueError(f"{PROTO_NAME}: the program has no side-set, so SIDE_EN must be 0")
    return (side_en << 30) | (program.wrap << 12) | (wrap_bottom << 7)


def shiftctrl_value(*, pull_thresh: int = 0, out_right: int = 1, autopull: int = 1) -> int:
    """SHIFTCTRL: shift right (pair 0 of the word first), autopull every 32 bits.

    `PULL_THRESH = 0` is not "no threshold": the field means 32 (`thresh(f) = f if f != 0 else
    32`, docs/mvp-isa.md `## Conventions`), which is exactly right for direction pairs -- a
    32-bit word is 16 of them, and AUTOPULL refills the OSR from the TX FIFO as soon as a whole
    word has been shifted out, so the program only pulls by hand for the first one
    (docs/mvp-isa.md `### Reference step`, `background_autopull`).

    `OUT_SHIFTDIR` keeps its reset value of 1, so `out pindirs, 2` takes OSR[1:0] and shifts
    the OSR right: pair `i` of the frame is in bits `2i+1:2i` of the word that carries it.
    """
    if not 0 <= pull_thresh <= 31:
        raise ValueError(f"PULL_THRESH {pull_thresh} out of range 0..31")
    return (pull_thresh << 25) | (out_right << 19) | (autopull << 17)


def pinctrl_value(d0_pin: int, *, sideset_count: int = 0, set_count: int = 2,
                  out_count: int = 2) -> int:
    """PINCTRL: both windows on the D0/D1 pair, two pins wide, and no side-set at all.

    `SIDESET_COUNT = 0` is not a default, it is the whole trick: the reset value is 2 with
    `SIDE_EN = 0`, which makes *every* instruction side-set level 0 into `SIDESET_BASE` and its
    neighbour on every SM cycle (docs/mvp-isa.md `### Delay/side-set field`), and the side-set
    channel wins over the OUT channel on the same pin (`### Pin writes`), so the lines would be
    held low for ever.

    `OUT_COUNT = SET_COUNT = 2` because both channels have to reach both lines: `out pindirs, 2`
    pulls one of them and `set pindirs, 0` releases both for the pause.  A one-pin SET window
    would leave the line of the previous pulse driven, and the two pulses would overlap, which
    the specification forbids outright.  `IN_BASE` is unused by the program and is set to D0 for
    consistency.
    """
    if not 0 <= d0_pin < NUM_PINS:
        raise ValueError(f"D0 pin {d0_pin} is outside 0..{NUM_PINS - 1}")
    if d0_pin not in UIO_PINS or d0_pin + 1 not in UIO_PINS:
        raise ValueError(
            f"open drain needs two uio pins, 0..7 (docs/mvp-isa.md `## Pins`); "
            f"D0={d0_pin} and D1={d0_pin + 1} are not"
        )
    if sideset_count:
        raise ValueError(f"SIDESET_COUNT {sideset_count} would put a level on the lines")
    if set_count != 2 or out_count != 2:
        raise ValueError(f"OUT_COUNT {out_count} and SET_COUNT {set_count} both have to be 2")
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (d0_pin << 15)
            | (d0_pin << 10) | (d0_pin << 5) | d0_pin)


@dataclass(frozen=True)
class Config:
    """One Wiegand setup: which engine, which pin pair and how wide a pulse."""

    pw_ns: float = 50_000.0          # the pulse width asked for; CLKDIV follows from it
    engine: int = 0
    d0_pin: int = 0
    name: str = DEFAULT_LAYOUT

    @property
    def d1_pin(self) -> int:
        return self.d0_pin + 1

    @property
    def pins(self) -> tuple:
        return self.d0_pin, self.d1_pin

    @property
    def bits(self) -> int:
        return layout(self.name).bits


# --------------------------------------------------------------------------------------------
# The host side of one engine
# --------------------------------------------------------------------------------------------

class WiegandDriver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as a Wiegand reader.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are queued
    on `dev` and land one per core cycle as the simulation runs, so `read()` and `send_card()`
    pump the bus while they wait.

    The frame timing is not something the caller supplies as a bit count but as a pulse width in
    nanoseconds; CLKDIV, and with it the pause, follow from the program's own two phase lengths
    and from the SIA window, and `clkdiv_for_pulse()` refuses a width that cannot be expressed
    inside that window.  What lands on the wire is therefore never off-spec by construction, and
    the checker's job is to confirm that, not to catch a mistake in this module.
    """

    @property
    def TXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)

    def __init__(self, dev, bus, *, engine: int = 0, d0_pin: int = 0, pw_ns: float = 50_000.0,
                 layout_name: str = DEFAULT_LAYOUT, program: Optional[Program] = None,
                 path: Path | str = PROGRAM_PATH) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        self.dev, self.bus = dev, bus
        self.engine, self.d0_pin = engine, d0_pin
        self.d1_pin = d0_pin + 1
        self.config = Config(pw_ns=pw_ns, engine=engine, d0_pin=d0_pin, name=layout_name)
        self.program = program if program is not None else load_program(path)
        self.n_pw, self.n_pi = phase_cycles(self.program)
        self.pairs = frame_pairs(self.program)
        if self.pairs != self.config.bits:
            raise ValueError(
                f"program.pio sends {self.pairs} direction pairs but the {layout_name} layout has "
                f"{self.config.bits} bits: one pair per bit, so the program and the layout have "
                f"to agree"
            )
        self.clkdiv = clkdiv_for_pulse(pw_ns, self.n_pw, self.n_pi)
        self.pinctrl = pinctrl_value(d0_pin)      # a pin pair PINDIRS cannot drive is refused
        self.last_frame_tick: Optional[int] = None

    # ---- addresses ----
    @property
    def imem_base(self) -> int:
        return A_INSTR_MEM[self.engine]

    def sm_addr(self, index: int) -> int:
        return A_SM_BASE + 6 * self.engine + index

    @property
    def send_addr(self) -> int:
        return self.program.labels["send"]

    @property
    def idle_addr(self) -> int:
        return self.program.labels["idle"]

    @property
    def words_used(self) -> int:
        return words_used(self.program)

    # ---- the timing this configuration puts on the wire ----
    @property
    def sm_cycle_ticks(self) -> float:
        """Core cycles one SM cycle lasts: the divider this pulse width needed."""
        return ticks_per_sm_cycle(self.clkdiv)

    @property
    def pulse_ticks(self) -> float:
        """Ticks the low pulse lasts: `n_pw` SM cycles (docs/mvp-isa.md `### Cycle accounting`)."""
        return bit_timing(self.n_pw, self.n_pi, self.sm_cycle_ticks)[0]

    @property
    def pause_ticks(self) -> float:
        """Ticks between two pulses: the pause phase plus the `jmp` back into the loop."""
        return bit_timing(self.n_pw, self.n_pi, self.sm_cycle_ticks)[1]

    @property
    def bit_ticks(self) -> float:
        return self.pulse_ticks + self.pause_ticks

    @property
    def frame_span_ticks(self) -> int:
        """An upper bound on the ticks from the forced `jmp send` to the frame's last rise.

        Three SM cycles reach the first `out pindirs, 2` after the forced jump (the `set y` and
        the `pull block`, plus at most one cycle of divider phase), and one whole bit period per
        pair after that.  Eight ticks are added for the pad (docs/mvp-isa.md `## Timing`,
        T_OUT) and for the phase.  It is an upper bound on purpose: a host has to be sure the
        next frame starts late enough, never that it starts early, and the cost of the slack is
        a few thousand core cycles against a frame gap of a million.
        """
        return int(self.sm_cycle_ticks * (3 + self.pairs * (self.n_pw + self.n_pi + 1)) + 8)

    @property
    def frame_gap_ticks(self) -> int:
        """The quiet time that has to separate two frames, in ticks.

        One tick more than the pulse interval *maximum*: a receiver cannot tell the end of a
        frame from a pause that is still inside the window, so a reader that returns to the
        wires sooner runs its frames together and the other end sees a 52-bit frame
        (research/ref/protocols/wiegand/05_accessgrid_26bit_decode.md, and the
        `tPI;MAX`/`BITS` rules of verif/vip/wiegand.py).
        """
        return ceil_ticks(WIEGAND_WINDOW_NS["pi_max"]) + 1

    @property
    def run_ticks(self) -> int:
        """How long the bus has to run after `send_card()` for the frame to be fully observable.

        The frame itself, then a quiet period longer than any pause may be: that is the tick at
        which the other end closes the frame and checks its length and its two parity bits, so a
        run that stops earlier leaves the frame unjudged.
        """
        return self.frame_span_ticks + self.frame_gap_ticks

    # ---- register values ----
    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value()

    # ---- host accesses: queued on the ChipDevice, one per tick ----
    def write(self, addr: int, data: int) -> None:
        """Queue one host write (it takes effect at the end of its bus access cycle `h`)."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def _settle(self, extra: int = 2) -> None:
        """Run the bus until the queue is empty, then `extra` more ticks."""
        ticks = 0
        while not self.dev.idle:
            self.bus.step()
            ticks += 1
            if ticks > MAX_TICKS:
                raise TimeoutError(f"host writes still queued after {ticks} ticks")
        for _ in range(extra):
            self.bus.step()

    def read(self, addr: int) -> int:
        """Queue one host read and run the bus until its data comes back."""
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.bus.step()
            ticks += 1
            if ticks > MAX_TICKS:
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

    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask

    # ---- the startup sequence (docs/mvp-regmap.md `### Typical host sequences`) ----
    def load(self) -> None:
        """Steps 1-3 and 4: stop the engine, burst-write the program, set the registers, and
        put the lines where an idle bus leaves them.

        Step 4 is a forced `set pindirs, 0` through `SMe_INSTR`: at reset every pin is an input,
        which is already an idle bus, but the program owns these two directions from then on and
        a host that starts a second transaction wants them released before it queues anything.
        The engine is left disabled at PC 0, which is `idle` -- the program's first instruction --
        so enabling it starts an idle reader with both lines released and nothing to send.
        """
        self.stop()
        for k, word in enumerate(self.program.instrs):
            self.write(self.imem_base + k, word)
        self.write(self.sm_addr(SM_CLKDIV), self.clkdiv)
        self.write(self.sm_addr(SM_EXECCTRL), self.execctrl)
        self.write(self.sm_addr(SM_SHIFTCTRL), self.shiftctrl)
        self.write(self.sm_addr(SM_PINCTRL), self.pinctrl)
        self.force(self.program.instrs[self.idle_addr])       # step 4: the initial pin state
        self._settle(2)

    def force(self, word: int) -> None:
        """Push one instruction into the engine's EXEC latch.

        It runs in the next core cycle whatever the divider and `SM_ENABLE` say, does not advance
        the PC and ignores its delay (docs/mvp-isa.md `### EXEC`).
        """
        self.write(self.sm_addr(SM_INSTR), word & 0xFFFF)

    def enable(self) -> None:
        """Step 6: SM_ENABLE with SM_RESTART and CLKDIV_RESTART in one CTRL write, so the
        engine's first SM cycle is two core cycles later (docs/mvp-isa.md `### Clock divider`)."""
        ctrl = self.read(A_CTRL) & ~self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, ctrl | self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def start(self, first: Sequence[int] = ()) -> None:
        """Preload the TX FIFO while the engine is stopped, then enable it.

        There is no jump to arm here: `idle` is the program's first instruction, so the engine
        comes up idle and the host sends a frame whenever it wants one (`send_card`).  `first` is
        for a host that wants the very first frame's words in place before the engine is ever
        enabled; the four-deep TX FIFO is not joined to the RX FIFO, so nothing else drains it
        while the engine is stopped (docs/mvp-isa.md `## Differences from RP2040`, item 5).
        """
        if first:
            self.preload(first)
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

    # ---- the TX FIFO ----
    @property
    def tx_full(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXFULL << self.engine))

    @property
    def tx_empty(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXEMPTY << self.engine))

    def push(self, word: int) -> None:
        """Push one TX word, waiting for room first so nothing is dropped (TXOVER).

        A word is the direction pairs of up to sixteen frame bits, pair `i` in bits `2i+1:2i`
        (`frame_words`), and the engine shifts them out lowest pair first.
        """
        if not 0 <= word <= 0xFFFF_FFFF:
            raise ValueError(f"a TX word is 0..2**32-1 (got {word!r})")
        while self.tx_full:
            self._settle(1)
        self.write(A_TXF[self.engine], word)
        self._settle(1)

    def preload(self, words: Sequence[int]) -> None:
        """Fill the TX FIFO while the engine is stopped.

        Only `TXFIFO_DEPTH` words fit and a disabled engine never pops.  A 26-bit frame is two
        words, so a host that wants no stall at all pushes them here before enabling.
        """
        words = list(words)
        if len(words) > self.TXFIFO_DEPTH:
            raise ValueError(
                f"{len(words)} words do not fit the stopped {self.TXFIFO_DEPTH}-deep TX FIFO; "
                f"call send_card() after start()"
            )
        for word in words:
            self.push(word)

    # ---- the frames ----
    @property
    def idle(self) -> bool:
        """True when the engine is parked in `idle`, with both lines released.

        There is no IRQ in the MVP, so `SMx_ADDR` and the FIFO levels are all the host can see of
        the program's progress (docs/mvp-regmap.md `### Per-engine registers`, `**SMe_ADDR**`).
        """
        return self.pc() in (self.idle_addr, self.idle_addr + 1)

    def wait_idle(self, limit: int = MAX_TICKS) -> None:
        ticks = 0
        while not self.idle:
            self.bus.step()
            ticks += 1
            if ticks > limit:
                raise TimeoutError(f"the engine is still at PC {self.pc()} after {ticks} ticks")

    def wait_frame_gap(self, limit: int = MAX_TICKS) -> None:
        """Run the bus until the quiet time since the last frame can separate two of them.

        A frame ends when the other end sees the lines quiet for longer than any pause may be,
        so a second frame pushed out earlier runs into the first: the panel would decode one
        52-bit frame instead of two 26-bit ones.  This waits the frame's own length plus that
        quiet time, both counted from the forced `jmp send` (see `frame_span_ticks`).
        """
        if self.last_frame_tick is None:
            return
        want = self.last_frame_tick + self.run_ticks
        ticks = 0
        while self.bus.t < want:
            self.bus.step()
            ticks += 1
            if ticks > limit:
                raise TimeoutError(f"the frame gap did not arrive within {ticks} ticks")

    def frame(self, facility: int, number: int) -> list:
        """The TX words for one card, in the order the engine shifts them out."""
        return frame_words(frame_bits(facility, number, self.config.name))

    def send_card(self, facility: int, number: int) -> int:
        """Queue one frame and start the engine on it; returns the tick it was started at.

        The words go in first, then a forced `jmp send`, which is the only way to set the PC.
        Everything this needs has already been checked: the facility code and ID number fit the
        layout, the frame is the number of pairs the program shifts, and the divider puts both
        phases inside the SIA window.
        """
        self.wait_idle()
        self.wait_frame_gap()
        words = self.frame(facility, number)
        self.wait_idle()
        for word in words:
            self.push(word)
        self.force(jmp_abs(self.send_addr))
        self._settle(2)
        self.last_frame_tick = self.bus.t
        return self.last_frame_tick

    def run(self, ticks: Optional[int] = None) -> None:
        """Run the bus for `ticks` core cycles (default: long enough to judge the last frame)."""
        for _ in range(int(self.run_ticks if ticks is None else ticks)):
            self.bus.step()

    def report(self) -> str:
        """What this configuration puts on the wire, as one block of text."""
        window = WIEGAND_WINDOW_NS
        return "\n".join([
            f"{PROTO_NAME} on engine {self.engine}, D0={self.d0_pin} D1={self.d1_pin}, "
            f"{self.config.name}",
            f"  words used      {self.words_used} of 32 (IMEM addresses 0..{self.words_used - 1})",
            f"  pairs per frame {self.pairs} (one direction pair per frame bit)",
            f"  phase lengths   {self.n_pw} SM cycle pulse, {self.n_pi} SM cycle pause",
            f"  CLKDIV          {self.clkdiv:#010x} -> {self.sm_cycle_ticks:g} core cycles "
            f"per SM cycle",
            f"  pulse           {self.pulse_ticks:g} ticks = {self.pulse_ticks * TICK_NS / 1000:g} us"
            f"  (window {window['pw_min'] / 1000:g}..{window['pw_max'] / 1000:g} us)",
            f"  pause           {self.pause_ticks:g} ticks = {self.pause_ticks * TICK_NS / 1000:g} us"
            f"  (window {window['pi_min'] / 1000:g}..{window['pi_max'] / 1e6:g} ms)",
            f"  bit time        {self.bit_ticks:g} ticks = {self.bit_ticks * TICK_NS / 1000:g} us",
            f"  frame           {self.pairs} bits, at most {self.frame_span_ticks} ticks",
            f"  frame gap       {self.frame_gap_ticks} ticks "
            f"(pi_max + 1 = {self.frame_gap_ticks * TICK_NS / 1e6:g} ms)",
        ])

"""Host side of the MVP DMX512-A controller firmware: load program.pio, set up an engine,
feed it slot values, and keep it fed.

Written against `docs/mvp-regmap.md` (register word addresses, the
`### Typical host sequences` startup order, the per-engine fields) and `docs/mvp-isa.md`
(divider, delay, autopull, pin windows).  Host accesses go through `iss.pio_iss.ChipDevice`'s
write/read queue, one per tick, so every wait here is the simulation running -- the same
thing a real host does over the SPI port of `docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                       # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == 0))
    dev = bus.attach(ChipDevice())
    drv = Dmx512Driver(dev, bus, pin=0, bit_rate=250e3, slots=25)
    drv.load()                 # IMEM, CLKDIV, EXECCTRL, SHIFTCTRL, PINCTRL
    drv.start()                # forced pins, then forced `jmp start` + CTRL restart/enable
    drv.send([0x00, 0xFF, 0x55, ...], packets=2)   # keeps the TX FIFO fed while it runs
    chk.finish()               # verif.vip.dmx512.DmxChecker: close the last packet

The chip core clock is 50 MHz: one `verif.vip.core.Bus` tick at `ns_per_tick = 20`, the
configuration the DMX512 VIPs document in their own docstrings.

**The chip is the controller.** Every rule the standard puts on the wire is a transmitter's
rule (tables 6 and 7, clauses 8.1 to 8.10), and `verif.vip.dmx512.DmxChecker` judges a
transmitter; a receiver role would need the chip to sample an asynchronous line through a
3-cycle input pipeline (`T_IN = 3`) at 250 kbit/s with no IRQ to tell the host when a slot
has landed, and nothing in `verif/vip/dmx512.py` measures a receiver's timing margins.

**Every number this module uses is derived, not typed.** The bit delay comes out of the
requested bit rate and the 5-bit delay field, the break and the mark-before-break pad out of
the standard's own minimums (`verif.vip.dmx512.dmx_minimums`, i.e. tables 6 and 7 as
transcribed from ANSI E1.11), and the frame's SM-cycle count out of the assembled words.  The
one thing the standard cannot shorten is the *number* of slots: `SET`'s data field is 5 bits,
so a frame is 1..31 slots.  See `slots_limit`.

Two things about this chip that a DMX512 host does not have to think about, and that this
module exists to get right:

* `PINCTRL.SIDESET_COUNT` resets to 2 and `EXECCTRL.SIDE_EN` to 0, where *every* instruction
  side-sets 2 pins from the top of the delay/side-set field, and `SET_COUNT` resets to 5, so
  a `set pins, 1` would drive five pins. The program side-sets nothing and writes one pin, so
  the driver writes `SIDESET_COUNT = 0`, `SET_COUNT = 1`, `OUT_COUNT = 1` and points all
  three windows at the DMX pin (`docs/mvp-isa.md` `### The 18 protocol pins`).
* `SHIFTCTRL.PULL_THRESH = 8` with autopull makes the eighth `out pins, 1` of a slot refetch
  the OSR from the TX FIFO in the same SM cycle.  A refill that finds the FIFO empty stalls
  one SM cycle, which is one whole bit time of extra jitter on that edge, and the tables'
  bit-rate tolerance is two ticks.  So the host has to keep the FIFO fed: `feed` tops it up
  while the bus runs, and `send` is the whole transaction.
"""
from __future__ import annotations

from fw import regmap

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Optional, Sequence

from asm.pioasm import Program, assemble

from verif.vip.dmx512 import dmx_minimums

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
NS_PER_TICK = 20.0            # docs/mvp-isa.md `## Timing`: the MVP's nominal core clock
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "dmx512"         # the .program name inside program.pio

# ---- frame shape, ANSI E1.11 clauses 8.2, 8.6 and table 5 (also in verif/vip/dmx512.py) ----
SLOT_BITS = 11                # start + 8 data + 2 stop, no parity
DATA_BITS = 8

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) ----
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF = regmap.txf_addrs()
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_INSTR_MEM = regmap.imem_bases()    # engine e, word k: A_INSTR_MEM[e] + k
A_SM_BASE = regmap.sm_base()              # SMe_CLKDIV = A_SM_BASE + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

# CTRL fields (docs/mvp-regmap.md `**CTRL**`)
CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART = 0x001, 0x010, 0x100
# FSTAT fields, engine e
FSTAT_TXEMPTY, FSTAT_TXFULL = 0x0100_0000, 0x0001_0000
# FLEVEL: TX0 in 3:0, TX1 in 11:8
FLEVEL_TX_SHIFT = 8

# Instruction encodings used by the host (docs/mvp-isa.md `## Encoding`).
OP_JMP = 0b000
JMP_ALWAYS = 0b000
#: `set pins, n` as the host writes it through SMe_INSTR (opcode 111, dest PINS, 5-bit data)
SET_PINS_WORD = 0xE000
#: `set pindirs, n` through SMe_INSTR (opcode 111, dest PINDIRS)
SET_PINDIRS_WORD = 0xE080

#: The five `.define`s fw/dmx512/program.pio carries, in the order `assemble_for` takes them.
DEFINES = ("BIT_DELAY", "BREAK_ITERS", "SLOTS", "PAD_OUTER", "PAD_INNER")
_DEFINE_RE = re.compile(r"^(\s*\.define\s+(?:PUBLIC\s+)?)(\w+)(\s+)(\S.*?)(\s*)$")

#: `set <dest>, <data>` data field: 5 bits (docs/mvp-isa.md `### Field values`, SET).
SET_IMMEDIATE_MAX = 0x1F
#: The delay field is all 5 bits when the program side-sets nothing, so a delay is 0..31 and
#: one instruction is 1..32 SM cycles (docs/mvp-isa.md `### Delay/side-set field`).
MAX_DELAY = 0x1F


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address (docs/mvp-isa.md `## Encoding`)."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (OP_JMP << 13) | (JMP_ALWAYS << 5) | addr


# --------------------------------------------------------------------------------------------
# The program: substitute its .defines, assemble it, and read the timing back out of the words
# --------------------------------------------------------------------------------------------

def defines_of(path: Path | str = PROGRAM_PATH) -> dict:
    """The `.define` names and values program.pio carries, as integers.

    Every `.define` in program.pio is a plain literal (the rest of the line is a comment
    explaining it, which is why the value stops at the first token), so the defaults a caller
    sees here are the ones `asm/pioasm.py` would assemble.
    """
    out: dict = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        match = _DEFINE_RE.match(line.strip())
        if match:
            try:
                out[match.group(2)] = int(match.group(4).split()[0])
            except ValueError:
                raise ValueError(
                    f"{path}: `.define {match.group(2)}` is {match.group(4).split()[0]!r}, and "
                    "this driver reads the defaults as plain integers"
                ) from None
    missing = set(DEFINES) - set(out)
    if missing:
        raise ValueError(f"{path}: no .define for {sorted(missing)}")
    return out


def set_define(text: str, name: str, value: int) -> str:
    """One `.define NAME value` line of `text`, with `value` substituted.

    The comment after the value is kept, so the header of program.pio still explains what the
    number is.  A name that is not defined is an error rather than an insertion: a silently
    added `.define` would leave the program assembling with a value nobody chose.
    """
    pattern = re.compile(
        r"^(\s*\.define\s+(?:PUBLIC\s+)?" + re.escape(name) + r"\s+)(\S+)(.*)$", re.MULTILINE
    )
    if not pattern.search(text):
        raise ValueError(f"program.pio has no `.define {name}` to substitute")
    return pattern.sub(lambda m: f"{m.group(1)}{int(value)}{m.group(3)}", text, count=1)


def assemble_for(
    *,
    bit_delay: int,
    break_iters: int,
    slots: int,
    pad_outer: int,
    pad_inner: int,
    path: Path | str = PROGRAM_PATH,
) -> Program:
    """Assemble program.pio with the five numbers this configuration needs.

    `asm/pioasm.py` is the assembler and it is the only one: the words come out of it, and
    everything the driver does next -- the clock divider, the break length, the frame time --
    is read back out of those words rather than out of the arguments.
    """
    text = Path(path).read_text(encoding="utf-8")
    for name, value in (("BIT_DELAY", bit_delay), ("BREAK_ITERS", break_iters),
                        ("SLOTS", slots), ("PAD_OUTER", pad_outer), ("PAD_INNER", pad_inner)):
        text = set_define(text, name, value)
    programs = assemble(text)
    names = [p.name for p in programs]
    if names != [PROTO_NAME]:
        raise ValueError(f"{path}: expected one .program named {PROTO_NAME}, got {names}")
    return programs[0]


def load_program(path: Path | str = PROGRAM_PATH) -> Program:
    """Assemble program.pio with its own `.define` values, unchanged."""
    return assemble_for(path=path, **{name.lower(): value
                                       for name, value in defines_of(path).items()})


def decode_word(word: int) -> dict:
    """Split one word into the fields `docs/mvp-isa.md` `## Encoding` defines.

    `delay` is only the delay if the program side-sets nothing, which this one does not; the
    program is checked for that by `bit_cycles`, so a future program that grows a `.side_set`
    cannot be timed by mistake.
    """
    return {
        "op": word >> 13,
        "delay": (word >> 8) & 0x1F,
        "field": (word >> 5) & 0x7,
        "src": word & 0x7,
        "idx": word & 0x1F,
    }


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


def instruction_cycles(word: int) -> int:
    """SM cycles one instruction occupies: 1 + delay (docs/mvp-isa.md `### Cycle accounting`)."""
    return 1 + decode_word(word)["delay"]


def word_at(program: Program, label: str) -> int:
    """The word at `label`, or raise: the timing below is read from real instructions."""
    if label not in program.labels:
        raise ValueError(f"{PROTO_NAME}: no label {label!r} in program.pio")
    return program.instrs[program.labels[label]]


def bit_cycles(program: Program) -> int:
    """SM cycles one bit lasts, taken from the assembled words rather than assumed.

    The slot loop sends one bit per instruction, all of them with the same delay, so a bit is
    `1 + delay` SM cycles and CLKDIV turns that into core cycles.  Anything else in the words
    would make CLKDIV mean something else, so this raises instead of guessing.
    """
    if program.sideset_bits:
        raise ValueError(
            f"{PROTO_NAME}: the program side-sets {program.sideset_bits} bit(s), so bits 12:8 "
            "are not all delay and the divider cannot be derived from this function"
        )
    data = [decode_word(w)["delay"] for w in program.instrs
            if decode_word(w)["op"] == 0b011 and decode_word(w)["field"] == 0b000
            and decode_word(w)["idx"] == 1]
    if len(data) != DATA_BITS:
        raise ValueError(
            f"{PROTO_NAME}: expected {DATA_BITS} `out pins, 1` data bits, found {len(data)}; "
            "the divider cannot be derived from this program"
        )
    if len(set(data)) != 1:
        raise ValueError(f"{PROTO_NAME}: the data bits have different delays {data}")
    # The whole slot body has to be one bit time per instruction: the start bit at `slot`, the
    # eight data bits, the two stop bits and the inter-slot `jmp`.  Anything else would put an
    # edge off the grid the tables' bit-rate tolerance is measured against.
    body = program.instrs[program.labels["slot"]: program.labels["slot"] + SLOT_BITS + 1]
    if len(body) != SLOT_BITS + 1:
        raise ValueError(f"{PROTO_NAME}: the slot body is not {SLOT_BITS + 1} words")
    if [decode_word(w)["delay"] for w in body] != [data[0]] * len(body):
        raise ValueError(
            f"{PROTO_NAME}: the slot body's delays are "
            f"{[decode_word(w)['delay'] for w in body]}, not all {data[0]}, so its edges are "
            "not all one bit time apart"
        )
    return 1 + data[0]


def loop_passes(counter: int) -> int:
    """Passes a `jmp x--` loop whose test is at the bottom of its body runs: `counter + 1`.

    The body runs once more after the last taken branch, and `docs/mvp-isa.md` JMP writes
    X - 1 whether or not the branch was taken, so the counter also ends at -1.  Every loop in
    program.pio is shaped that way, so this is the one place the off-by-one lives.
    """
    return counter + 1


def _counter_before(program: Program, label: str, dest: int) -> int:
    """The 5-bit immediate of the `set <dest>, ...` word in front of `label`."""
    word = word_at(program, label)
    where = program.labels[label] - 1
    prev = program.instrs[where]
    if decode_word(prev)["op"] != 0b111 or decode_word(prev)["field"] != dest:
        raise ValueError(
            f"{PROTO_NAME}: the word before `{label}` is not a `set` into destination {dest}"
        )
    return decode_word(prev)["idx"]


def slot_count(program: Program) -> int:
    """Slots in a frame, read out of the `set x, ...` word in front of the slot loop."""
    return loop_passes(_counter_before(program, "slot", 0b001))


def slot_cycles(program: Program) -> int:
    """SM cycles one slot occupies on the wire: `SLOT_BITS + 1` bit times.

    One more than the standard's eleven: the loop's back-jump is an instruction too, and an
    instruction is a bit time, so the line sits at mark for one bit time between slots.  That
    is what clause 8.8 wants there and table 6 designator 9 allows, and it is the price of a
    32-word IMEM: there is no room for an unconditional jump with a zero delay, which would
    be the only way to spend a bit time without an instruction.
    """
    return (SLOT_BITS + 1) * bit_cycles(program)


def break_iteration_cycles(program: Program) -> int:
    """SM cycles one pass of the BREAK loop: the `set pins, 0` plus its `jmp` back."""
    brk = program.labels["brk"]
    space, back = program.instrs[brk], program.instrs[brk + 1]
    if decode_word(space)["delay"] != MAX_DELAY:
        raise ValueError(f"{PROTO_NAME}: the break loop's space instruction is not [31]")
    if decode_word(back)["op"] != OP_JMP:
        raise ValueError(f"{PROTO_NAME}: the break loop does not end in a jump")
    return instruction_cycles(space) + instruction_cycles(back)


def break_cycles(program: Program) -> int:
    """SM cycles of space the BREAK is, read out of the loop counter.

    One more pass than the counter holds, because the loop's test is at the bottom of its body
    (`loop_passes`), which is also why the counter ends at -1: the next frame reloads it, so
    the wrap to 0xFFFFFFFF never escapes.
    """
    return loop_passes(break_counter(program)) * break_iteration_cycles(program)


def mab_cycles(program: Program) -> int:
    """SM cycles between the end of the BREAK and the START Code's start bit.

    That is every instruction between the break loop and the slot loop: the three that drive
    mark, and the `set x, SLOTS` that loads the slot counter.  The last of them is why the MAB
    is 90 SM cycles rather than 89 -- the checker measures mark to the START Code's start
    bit, so a word between the two counts, however little it does to the wire.
    """
    brk, slot = program.labels["brk"], program.labels["slot"]
    gap = program.instrs[brk + 2: slot]
    if [decode_word(w)["field"] for w in gap[:-1]] != [0b000] * (len(gap) - 1):
        raise ValueError(f"{PROTO_NAME}: the MAB is not a run of `set pins` instructions")
    return sum(instruction_cycles(w) for w in gap)


def pad_cycles(program: Program, pad_inner: int, pad_outer: int) -> int:
    """SM cycles the mark-before-break padding takes, from the two nested loops' words.

    An outer pass is the `set x, PAD_INNER` reload, the inner loop, and the `jmp y--` back, so
    the outer counter multiplies the inner one.  As with the break loop neither loop is skipped
    when its counter is 0 -- the inner pair runs once and the outer jump once -- hence the + 1
    on each, which is also where an unpadded frame's 35 SM cycles come from.
    """
    reload = program.instrs[program.labels["pad_r"]]
    inner = program.labels["pad_i"]
    mark, back = program.instrs[inner], program.instrs[inner + 1]
    if decode_word(mark)["delay"] != MAX_DELAY or decode_word(back)["op"] != OP_JMP:
        raise ValueError(f"{PROTO_NAME}: the pad loop is not a `set pins, 1 [31]` and a jump")
    one_pass = instruction_cycles(mark) + instruction_cycles(back)
    outer_jump = instruction_cycles(program.instrs[program.labels["pad_o"]])
    return ((instruction_cycles(reload) + one_pass * (pad_inner + 1) + outer_jump)
            * (pad_outer + 1))


def frame_cycles(program: Program, pad_outer: int, pad_inner: int) -> int:
    """SM cycles from one BREAK's space to the next one's, i.e. the tables' break-to-break.

    The path is the break loop, the MAB, every slot body, the two counter reloads, the pad and
    the jump back to `packet` -- every instruction a frame executes, and the number of slots is
    read out of the word that loads the counter, so the frame time is what the words say rather
    than what the standard would like it to be.
    """
    brk = program.labels["brk"]
    back_to_packet = instruction_cycles(program.instrs[-1])               # `jmp packet`
    counter_reloads = (instruction_cycles(program.instrs[brk - 1])       # `set y, BREAK_ITERS-1`
                       + instruction_cycles(program.instrs[program.labels["pad_r"] - 1]))
    return (break_cycles(program)
            + mab_cycles(program)
            + slot_count(program) * slot_cycles(program)
            + counter_reloads
            + pad_cycles(program, pad_inner, pad_outer)
            + back_to_packet)


def break_counter(program: Program) -> int:
    """The BREAK loop's counter, read back out of the word that carries it.

    The break count is the one number the frame time depends on that is not an argument here,
    and it is read out of the `set y, BREAK_ITERS` word rather than remembered.  It is a
    counter, not a number of passes: `break_cycles` adds the pass `jmp y--` does not take.
    """
    word = word_at(program, "packet")
    if decode_word(word)["field"] != 0b010 or decode_word(word)["op"] != 0b111:
        raise ValueError(f"{PROTO_NAME}: `packet` does not start with `set y, ...`")
    return decode_word(word)["idx"]


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def ticks_per_bit(clkdiv: int, per_bit: int) -> float:
    """The average bit length in core cycles (ticks) a CLKDIV word produces."""
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    return per_bit * (integer + ((clkdiv >> 8) & 0xFF) / 256.0)


def select_bit_delay(ticks_per_bit_wanted: float, max_cycles: int = MAX_DELAY + 1) -> tuple:
    """The (bit delay, CLKDIV INT) pair that makes a bit exactly `ticks_per_bit_wanted` ticks.

    One instruction is `1 + delay` SM cycles and one SM cycle is CLKDIV INT core cycles, so
    a bit is a product of the two and it has to be *exactly* the wanted number of ticks: the
    tables measure a bit period, and the checker compares every edge-to-edge interval with the
    fitted one to within two ticks (`JITTER_TICKS`), which an off-rate transmitter cannot do.

    The longest SM cycle the delay field allows is chosen first, because that is the one with
    the coarsest time base: an SM cycle is CLKDIV INT core cycles, so the smallest INT that
    divides the bit length is the one where a stall costs the least.  A delay of 0 (one SM cycle
    per instruction) is a legal delay and is what a bit length with no smaller factor needs --
    197 ticks is prime, so it is a delay of 0 and CLKDIV 197 or nothing.  Returns
    (delay, integer).
    """
    if ticks_per_bit_wanted < 1:
        raise ValueError(f"a bit must last at least one tick (got {ticks_per_bit_wanted})")
    whole = int(round(ticks_per_bit_wanted))
    if whole != ticks_per_bit_wanted:
        raise ValueError(
            f"a bit of {ticks_per_bit_wanted} ticks is not a whole number of core cycles: with "
            "an integer divider and a delay field the bit period is a product of two integers, "
            "so no CLKDIV expresses it"
        )
    for per_bit in range(min(max_cycles, whole), 0, -1):
        if whole % per_bit == 0:
            return per_bit - 1, whole // per_bit
    raise ValueError(
        f"a bit of {whole} ticks cannot be split into 1..{max_cycles} SM cycles of a whole "
        "number of core cycles each"
    )


def bit_rate_limits(profile: str = "transmit") -> dict:
    """The bit rates and bit lengths this chip's clock can put on the wire, in ticks.

    Two windows, and the difference matters.  The tables' window is
    `dmx_minimums(...)["min_bit_ticks"] .. ["max_bit_ticks"]`, 3.92 .. 4.08 us, which is what a
    receiver must accept.  The checker's own window is half a tick wider either way, because a
    transmitter with a fractional divider rounds each bit up or down and a whole-tick
    measurement of a whole number of bit times can be that far out
    (`verif.vip.dmx512.DmxChecker._bit_in_window`).  What the program can actually *express* is
    narrower still: a bit is `(1 + delay) * CLKDIV INT` core cycles with the delay 0..31, so the
    bit lengths available are the multiples of 2 .. 32 at this clock.
    """
    limits = dmx_minimums(NS_PER_TICK, profile)
    expressible = sorted({(1 + delay) * integer
                          for delay in range(MAX_DELAY + 1)
                          for integer in range(1, 0x10000)
                          if limits["min_bit_ticks"] - 0.5 <= (1 + delay) * integer
                          <= limits["max_bit_ticks"] + 0.5})
    return {
        "ns_per_tick": NS_PER_TICK,
        "bit_rate_min": limits["bit_rate_min"],
        "bit_rate_max": limits["bit_rate_max"],
        "bit_rate": limits["bit_rate"],
        "ticks_per_bit": limits["ticks_per_bit"],
        "min_bit_ticks": limits["min_bit_ticks"],
        "max_bit_ticks": limits["max_bit_ticks"],
        "checker_min_bit_ticks": limits["min_bit_ticks"] - 0.5,
        "checker_max_bit_ticks": limits["max_bit_ticks"] + 0.5,
        "expressible_bit_ticks": expressible,
        "fastest_bit_ticks": min(expressible),
        "slowest_bit_ticks": max(expressible),
    }


def expressible_rates(profile: str = "transmit") -> dict:
    """The bit rates (bit/s) of each bit length the program can express inside the tables."""
    out = {}
    for ticks in bit_rate_limits(profile)["expressible_bit_ticks"]:
        out[ticks] = CORE_HZ / ticks
    return out


def bit_delay_for_rate(bit_rate: float, profile: str = "transmit") -> tuple:
    """(bit delay, CLKDIV INT) for a bit rate inside the tables' window.

    The bit length a rate asks for is 1e9 / rate / 20 ns, and one 20 ns tick is 2.5 kbit/s, so
    a rate inside the tables' 245 .. 255 kbit/s window lands within half a tick of a whole
    number of ticks.  The whole number is what goes on the wire, and it has to stay inside the
    window the checker applies (half a tick either way, which is exactly the quantisation error
    a whole-tick measurement can have), so a rate whose rounding left the window is refused
    here rather than put on the wire to be flagged.
    """
    limits = dmx_minimums(NS_PER_TICK, profile)
    if not limits["bit_rate_min"] <= bit_rate <= limits["bit_rate_max"]:
        raise ValueError(
            f"{bit_rate / 1e3:.2f} kbit/s is outside the {limits['bit_rate_min'] / 1e3:.0f} .. "
            f"{limits['bit_rate_max'] / 1e3:.0f} kbit/s both tables allow"
        )
    wanted = 1.0e9 / bit_rate / NS_PER_TICK
    rounded = float(round(wanted))
    if not (limits["min_bit_ticks"] - 0.5 <= rounded <= limits["max_bit_ticks"] + 0.5):
        raise ValueError(
            f"{bit_rate / 1e3:.2f} kbit/s is {wanted:.3f} ticks per bit, and the nearest whole "
            f"tick ({rounded:.0f}) is outside the {limits['min_bit_ticks']:.2f} .. "
            f"{limits['max_bit_ticks']:.2f} ticks the tables allow at 20 ns/tick"
        )
    return select_bit_delay(rounded)


# --------------------------------------------------------------------------------------------
# Register values the program needs (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program) -> int:
    """EXECCTRL: SIDE_EN = 0 and the wrap window the assembler worked out (`.wrap`).

    SIDE_EN stays 0 because the program side-sets nothing and SIDESET_COUNT is 0, which is
    what leaves the whole 5-bit delay field for delays (`docs/mvp-isa.md`
    `### Delay/side-set field`).  The wrap window is the program's own span: the last word
    is a jump back to `packet`, and a jump does not count up through WRAP_TOP, so the window
    is only here so that a stray count past the last word lands somewhere legal.
    """
    return (program.wrap << 12)


def shiftctrl_value(*, pull_thresh: int = DATA_BITS, out_right: int = 1, autopull: int = 1) -> int:
    """SHIFTCTRL: shift right (LSB out first), autopull every 8 bits.

    A DMX slot puts its least significant data bit on the wire first (table 5), which is what
    `OUT_SHIFTDIR = 1` gives (the reset value): `out pins, 1` sends OSR bit 0 and shifts the
    OSR right.  So the host pushes each slot value right-justified, in bits 7:0.

    PULL_THRESH = 8 is one slot: the eighth `out pins, 1` brings the OSR count to the threshold
    and refills it from the TX FIFO in the same SM cycle (`docs/mvp-isa.md` `### Instruction
    semantics`, OUT).  One FIFO word is therefore one slot.
    """
    if not 1 <= pull_thresh <= 31:
        raise ValueError(f"PULL_THRESH {pull_thresh} out of range 1..31")
    return (pull_thresh << 25) | (out_right << 19) | (autopull << 17)


def pinctrl_value(pin: int, *, sideset_count: int = 0, set_count: int = 1,
                  out_count: int = 1) -> int:
    """PINCTRL: no side-set, and every window of the program on the DMX pin, one pin wide.

    OUT writes the data bits, SET writes the start bit, the stop bits and the mark of the
    MAB and the pad, and they are the same pin throughout.  The two counts and the side-set
    count are all written because all three reset values are wrong for this program:
    SIDESET_COUNT resets to 2 with SIDE_EN = 0, where every instruction side-sets two pins and
    would fight the SET writes here, and SET_COUNT resets to 5, where one `set pins, 1`
    drives the DMX pin and its four neighbours (`docs/mvp-isa.md` `### The 18 protocol pins`).
    """
    if not 0 <= pin < NUM_PINS:
        raise ValueError(f"DMX pin {pin} is outside 0..{NUM_PINS - 1}")
    if not all(0 <= b <= 32 for b in (sideset_count, set_count, out_count)):
        raise ValueError("window counts must be 0..32")
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (pin << 15)
            | (pin << 10) | (pin << 5) | pin)


def slots_limit() -> int:
    """The longest frame this program can send, and why it is 31.

    The slot count is loaded with `set x, SLOTS`, and SET's data field is 5 bits
    (`docs/mvp-isa.md` `### Field values`), so a frame is 1..31 slots: a START Code and up to
    30 data slots.  A 512-slot frame needs a 9-bit counter, which the ISA cannot load without
    the host: the only 32-bit load is a FIFO word, so the host would have to force `pull` and
    `mov x, osr` for every frame and the program would lose the word that reloads X.  That is
    the one requirement of DMX512-A (clause 8.1: "up to 512 data slots") the MVP meets only
    partially, and `words_used` is what stops the frame getting longer instead.
    """
    return SET_IMMEDIATE_MAX


@dataclass(frozen=True)
class Config:
    """One controller setup: which engine and pin, at which bit rate, and how long a frame is.

    `slots` counts the START Code, so a frame of 25 slots is a START Code and 24 data slots.
    `pad` is the mark-before-break padding in outer,inner iterations of the program's two
    nested pad loops; `pad=None` asks for the smallest padding that meets the tables'
    break-to-break minimum for this frame, which is what a controller has to do.
    """

    bit_rate: float = 250_000.0
    slots: int = 25
    engine: int = 0
    pin: int = 0
    pad: Optional[tuple] = None
    profile: str = "transmit"
    out_right: int = 1              # SHIFTCTRL.OUT_SHIFTDIR; 1 is the only value that works

    def __post_init__(self) -> None:
        if self.out_right not in (0, 1):
            raise ValueError(f"OUT_SHIFTDIR is 0 or 1 (got {self.out_right!r})")
        if not 1 <= self.slots <= slots_limit():
            raise ValueError(
                f"a frame is 1..{slots_limit()} slots (a START Code and up to "
                f"{slots_limit() - 1} data slots): SET's data field is 5 bits, see "
                "slots_limit()"
            )
        if self.pad is not None:
            outer, inner = self.pad
            if not (0 <= outer <= SET_IMMEDIATE_MAX and 0 <= inner <= SET_IMMEDIATE_MAX):
                raise ValueError(
                    f"pad {self.pad} is outside 0..{SET_IMMEDIATE_MAX} in each half: the pad "
                    "counters are SET immediates too"
                )

    @property
    def data_slots(self) -> int:
        """Data slots in a frame: the START Code is slot 0 and is not one of these."""
        return self.slots - 1

    @property
    def tick_delay(self) -> float:
        """The bus tick this configuration's core clock is worth, in ns."""
        return NS_PER_TICK

    def limits(self) -> Mapping:
        """The timing table this configuration is judged against, in whole ticks."""
        return dmx_minimums(self.tick_delay, self.profile)


# --------------------------------------------------------------------------------------------
# One configuration, derived: the words, and every duration that follows from them
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Plan:
    """A configuration's assembled words and the timing they produce, in bus ticks.

    Every field here is either read out of `program.instrs` or computed from it with
    `instructions/frame_cycles`; nothing is a constant of this module.  The test compares the
    durations against what `verif.vip.dmx512.DmxChecker` measured off the wire, so a word that
    drifts from these numbers fails rather than being papered over.
    """

    config: Config
    program: Program
    bit_delay: int             # the .define, i.e. the delay field every bit instruction has
    clkdiv: int                # SMe_CLKDIV, integer: FRAC is 0 so a bit is a whole tick count
    break_iters: int
    pad: tuple                 # (PAD_OUTER, PAD_INNER)

    @property
    def words(self) -> int:
        return words_used(self.program)

    @property
    def sm_cycles_per_bit(self) -> int:
        return 1 + self.bit_delay

    @property
    def divider_int(self) -> int:
        return (self.clkdiv >> 16) & 0xFFFF

    @property
    def tick_ticks(self) -> float:
        """Core cycles (bus ticks) one bit lasts: `sm_cycles_per_bit` * CLKDIV INT, exact."""
        return ticks_per_bit(self.clkdiv, self.sm_cycles_per_bit)

    @property
    def bit_rate(self) -> float:
        """The bit rate this plan puts on the wire, in bit/s, from the words."""
        return CORE_HZ / self.tick_ticks

    @property
    def break_ticks(self) -> float:
        return break_cycles(self.program) * self.divider_int

    @property
    def mab_ticks(self) -> float:
        return mab_cycles(self.program) * self.divider_int

    @property
    def slot_ticks(self) -> float:
        """One slot on the wire: eleven bit times plus the loop's own bit time of mark."""
        return slot_cycles(self.program) * self.divider_int

    @property
    def frame_ticks(self) -> float:
        """One whole frame: break, MAB, every slot, and the mark before the next break."""
        return frame_cycles(self.program, *self.pad) * self.divider_int

    @property
    def refresh_hz(self) -> float:
        """Frames per second: the tables' "approximately 830 Hz" is 1 / 1204 us."""
        return CORE_HZ / self.frame_ticks

    @property
    def pad_ticks(self) -> float:
        return pad_cycles(self.program, *self.pad) * self.divider_int

    def limits(self) -> Mapping:
        return self.config.limits()


def _pad_for(needed_ticks: float, base: Program, divider_int: int) -> tuple:
    """The smallest (PAD_OUTER, PAD_INNER) whose padding covers `needed_ticks`.

    The inner counter is left at its maximum and the outer one carries the remainder, because
    a 5-bit immediate cannot be multiplied: one outer pass is `pad_i` plus one instruction, so
    the padding a frame can ask for is up to 31 * (32 * 31 + 1) SM cycles, 5.1 ms of mark at
    250 kbit/s -- far more than the shortest frame needs.
    """
    if needed_ticks <= 0:
        return (0, 0)
    inner = SET_IMMEDIATE_MAX
    # `pad_cycles` multiplies the whole loop by (outer + 1), so one more outer pass costs the
    # difference between one and two: the loops run once even at zero, and that first pass is
    # what the difference leaves out.
    per_pass = (pad_cycles(base, inner, 1) - pad_cycles(base, inner, 0)) * divider_int
    outer = max(0, -(-needed_ticks // per_pass) - 1)     # ceil, then the pass already there
    if outer > SET_IMMEDIATE_MAX:
        raise ValueError(
            f"this frame needs {needed_ticks:.0f} ticks of mark before the next break, which "
            f"is more than the pad loops can hold ({SET_IMMEDIATE_MAX} * {per_pass:.0f} ticks)"
        )
    return (int(outer), inner)


def plan_for(config: Config, path: Path | str = PROGRAM_PATH) -> Plan:
    """Assemble program.pio for `config` and derive every duration from the words.

    The order matters.  The bit delay and the divider come from the requested bit rate and the
    5-bit delay field; the program is then assembled *once* with placeholders so that its fixed
    structure -- how long the break loop's iteration is, how many SM cycles a slot is -- can
    be read out of real words at this bit rate rather than at the file's default one; the break
    count and the pad follow from the tables' minimums and that structure; and the file is
    assembled for real.  Every duration in the `Plan` is then computed from *those* words, so a
    `.define` that did not land shows up as a frame that does not meet the tables rather than as
    a driver that believes it does.
    """
    limits = config.limits()
    bit_delay, divider_int = bit_delay_for_rate(config.bit_rate, config.profile)
    clkdiv = clkdiv_value(divider_int)
    bit_ticks = (bit_delay + 1) * divider_int
    if not limits["min_bit_ticks"] - 0.5 <= bit_ticks <= limits["max_bit_ticks"] + 0.5:
        raise ValueError(
            f"{bit_ticks} ticks per bit is outside the {limits['min_bit_ticks']:.2f} .. "
            f"{limits['max_bit_ticks']:.2f} ticks the tables allow at {NS_PER_TICK} ns/tick"
        )
    # Assembled twice: once to measure the break loop's own length, then again with the break
    # count that length implies, because the frame time below has to include the break the frame
    # will really send -- a probe with the placeholder count of 1 would ask for less padding than
    # the frame needs.
    probe = assemble_for(bit_delay=bit_delay, break_iters=1, slots=config.slots,
                         pad_outer=0, pad_inner=0, path=path)
    iteration = break_iteration_cycles(probe)
    break_iters = max(1, -(-limits["break"] // (iteration * divider_int)))
    probe = assemble_for(bit_delay=bit_delay, break_iters=break_iters, slots=config.slots,
                         pad_outer=0, pad_inner=0, path=path)
    unpadded = frame_cycles(probe, 0, 0) * divider_int
    pad = config.pad if config.pad is not None else _pad_for(
        limits["b2b_min"] - unpadded, probe, divider_int
    )
    program = assemble_for(bit_delay=bit_delay, break_iters=break_iters, slots=config.slots,
                           pad_outer=pad[0], pad_inner=pad[1], path=path)
    plan = Plan(config=config, program=program, bit_delay=bit_delay, clkdiv=clkdiv,
                break_iters=break_iters, pad=pad)
    problems = []
    if bit_cycles(program) != bit_delay + 1:
        problems.append(f"the assembled slot loop is {bit_cycles(program)} SM cycles per bit, "
                        f"not {bit_delay + 1}")
    if slot_count(program) != config.slots:
        problems.append(f"the assembled frame has {slot_count(program)} slots, not "
                        f"{config.slots}")
    if loop_passes(break_counter(program)) != break_iters:
        problems.append(f"the assembled break loop runs "
                        f"{loop_passes(break_counter(program))} passes, not {break_iters}")
    if plan.break_ticks < limits["break"]:
        problems.append(f"break {plan.break_ticks:.0f} ticks < {limits['break']}")
    if plan.mab_ticks < limits["mab"]:
        problems.append(f"MAB {plan.mab_ticks:.0f} ticks < {limits['mab']}")
    if plan.frame_ticks < limits["b2b_min"]:
        problems.append(f"break-to-break {plan.frame_ticks:.0f} ticks < {limits['b2b_min']}")
    if plan.frame_ticks > limits["b2b_max"]:
        problems.append(f"break-to-break {plan.frame_ticks:.0f} ticks > {limits['b2b_max']}")
    if plan.words > 32:
        problems.append(f"{plan.words} words > the 32-word IMEM")
    if problems:
        raise ValueError("; ".join(problems))
    return plan


# --------------------------------------------------------------------------------------------
# The driver: program the chip through iss.pio_iss.ChipDevice and feed it slots
# --------------------------------------------------------------------------------------------

class Dmx512Driver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as a DMX512-A controller.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are queued
    on `dev` and land one per tick as the simulation runs, so `read()` and `send()` pump the
    bus while they wait.  The constructor assembles program.pio for the configuration and
    raises if the words it produced cannot meet the timing table, so a driver that exists is a
    driver whose frame is legal on paper.
    """

    MAX_TICKS = 2_000_000

    @property
    def TXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)
    #: How many ticks `feed` runs between two looks at the TX FIFO.  A slot is 2400 ticks at
    #: 250 kbit/s, so a chunk this size cannot see the FIFO drop by more than one word, and
    #: `feed` keeps it topped up long before a refill finds it empty.
    CHUNK = 200

    def __init__(self, dev, bus, *, engine: int = 0, pin: int = 0, bit_rate: float = 250_000.0,
                 slots: int = 25, pad=None, profile: str = "transmit", out_right: int = 1,
                 config: Optional[Config] = None, path: Path | str = PROGRAM_PATH) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        if pin not in UIO_PINS:
            raise ValueError(
                f"DMX pin {pin} is not a uio pin {list(UIO_PINS)}: PINDIRS cannot drive "
                f"input-only or output-only protocol pins (docs/mvp-isa.md `## Pins`)"
            )
        self.dev, self.bus = dev, bus
        self.path = Path(path)
        self.config = config if config is not None else Config(
            bit_rate=bit_rate, slots=slots, engine=engine, pin=pin, pad=pad, profile=profile,
            out_right=out_right,
        )
        self.plan = plan_for(self.config, self.path)

    # ---- what the configuration is, read back out of the plan ----
    @property
    def program(self):
        return self.plan.program

    @property
    def engine(self) -> int:
        return self.config.engine

    @property
    def pin(self) -> int:
        return self.config.pin

    @property
    def slots(self) -> int:
        """Slots in a frame: the START Code plus the data slots."""
        return self.config.slots

    @property
    def data_slots(self) -> int:
        return self.config.data_slots

    @property
    def bit_rate(self) -> float:
        """The bit rate the words and the divider put on the wire, not the one that was asked
        for: at 20 ns/tick the nearest whole tick to a rate is what goes out."""
        return self.plan.bit_rate

    @property
    def words_used(self) -> int:
        return self.plan.words

    @property
    def imem_base(self) -> int:
        return A_INSTR_MEM[self.engine]

    def sm_addr(self, index: int) -> int:
        return A_SM_BASE + 6 * self.engine + index

    @property
    def start_addr(self) -> int:
        return self.program.labels["start"]

    @property
    def clkdiv(self) -> int:
        return self.plan.clkdiv

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value(out_right=self.config.out_right)

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(self.pin)

    # ---- host accesses: queued on the ChipDevice, one per tick ----
    def write(self, addr: int, data: int) -> None:
        """Queue one host write (docs/mvp-regmap.md: it takes effect at the end of its
        bus access cycle `h`)."""
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

        The value is the register's state in the access cycle, so a FIFO pop or a W1C
        happens as this returns (docs/mvp-regmap.md `### Cycle-exact mapping`).
        """
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.bus.step()
            ticks += 1
            if ticks > self.MAX_TICKS:
                raise TimeoutError(f"host read of {addr:#04x} unanswered after {ticks} ticks")
        return int(future.result())

    def run_ticks(self, ticks: int) -> None:
        """Run the bus, letting the engine do whatever it does, with no host traffic."""
        self._settle(0)
        for _ in range(ticks):
            self.bus.step()

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
        """The engine's PC, i.e. `SMe_ADDR`: which instruction it will fetch next."""
        return self.read(self.sm_addr(SM_ADDR))

    @property
    def tx_level(self) -> int:
        """Words in this engine's TX FIFO (FLEVEL)."""
        return (self.flevel() >> FLEVEL_TX_SHIFT * self.engine) & 0xF

    @property
    def tx_full(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXFULL << self.engine))

    @property
    def tx_empty(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXEMPTY << self.engine))

    # ---- the startup sequence (docs/mvp-regmap.md `### Typical host sequences`) ----
    def load(self) -> None:
        """Steps 1-3: stop the engine, burst-write the program, set the four registers.

        The engine is left disabled at whatever PC it had, so the caller can arm it and
        preload the TX FIFO before enabling.
        """
        self.stop()
        for k, word in enumerate(self.program.instrs):
            self.write(self.imem_base + k, word)
        self.write(self.sm_addr(SM_CLKDIV), self.clkdiv)
        self.write(self.sm_addr(SM_EXECCTRL), self.execctrl)
        self.write(self.sm_addr(SM_SHIFTCTRL), self.shiftctrl)
        self.write(self.sm_addr(SM_PINCTRL), self.pinctrl)
        self._settle()

    def arm_pins(self) -> None:
        """Step 4: put the pin at mark *before* driving it, through `SMe_INSTR`.

        The level register and the direction register are separate, and a pin follows its level
        only once it is driven (`docs/mvp-isa.md` `## Pins`).  PINDIRS resets to 0 and the level
        register to 0, so a program that drove first would put a SPACE on an idle bus for one
        SM cycle -- a break of the wrong length, which is what `DmxChecker` flags as BREAK
        before the first packet.  Mark first, then drive: the line never leaves mark.
        """
        self.write(self.sm_addr(SM_INSTR), SET_PINS_WORD | 1)
        self._settle(2)
        self.write(self.sm_addr(SM_INSTR), SET_PINDIRS_WORD | 1)
        self._settle(2)

    def arm(self) -> None:
        """Step 5: force `jmp start` through `SMe_INSTR`, the only way to set the PC.

        The host-forced instruction runs in the very next core cycle whatever the divider and
        SM_ENABLE say (`docs/mvp-isa.md` `### EXEC`).  That forced cycle also runs the
        background autopull, so it must happen *before* the host fills the TX FIFO: otherwise
        the forced cycle pops a word into the OSR and SM_RESTART (which does not touch the OSR,
        only `osr_cnt`) leaves that stale word to be shifted out.
        """
        self.write(self.sm_addr(SM_INSTR), jmp_abs(self.start_addr))
        self._settle(2)                 # the forced JMP executes one cycle after its write

    def enable(self) -> None:
        """Step 6: SM_ENABLE with SM_RESTART and CLKDIV_RESTART in the same CTRL write.

        The engine's first SM cycle is then two core cycles after this write
        (`docs/mvp-isa.md` `### Clock divider`).
        """
        ctrl = self.read(A_CTRL) & ~self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, ctrl | self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def start(self, first: Sequence[int] = (), justify=None) -> None:
        """Set the pins, arm, optionally fill the TX FIFO while stopped, then enable.

        `justify` is applied to every value in `first` before it goes into the FIFO, so a host
        that has chosen `SHIFTCTRL.OUT_SHIFTDIR = 0` can left-justify its bytes the same way here
        as in `feed`.
        """
        self.arm_pins()
        self.arm()
        if first:
            self.preload([justify(v) for v in first] if justify else first)
        self.enable()

    def stop(self) -> None:
        """Disable the engine, keeping every register (`docs/mvp-isa.md` `### Enable, restart`)."""
        ctrl = self.read(A_CTRL)
        self.write(A_CTRL, ctrl & ~self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def restart(self) -> None:
        """SM_RESTART + CLKDIV_RESTART, keeping the enable (`docs/mvp-regmap.md` `**CTRL**`)."""
        ctrl = self.read(A_CTRL)
        keep = self.ctrl_bits(CTRL_SM_ENABLE)
        self.write(A_CTRL, ctrl | (self.ctrl_bits(*self.CTRL_BITS) & ~keep))
        self._settle()

    # ---- the TX FIFO: one word is one slot ----
    def push(self, value: int) -> None:
        """Queue one slot value, waiting for room first so no word is dropped (TXOVER).

        The word is right-justified, in bits 7:0: `OUT_SHIFTDIR = 1` makes the PIO shift the
        OSR right, so bit 0 of the word is the first bit of the slot on the wire, which is the
        order a DMX slot is in (LSB first, table 5).
        """
        if not 0 <= value <= 0xFF:
            raise ValueError(f"a DMX512 slot value is 0..255 (got {value!r})")
        while self.tx_full:
            self._settle(1)
        self.write(A_TXF[self.engine], value)
        self._settle(1)

    def push_word(self, word: int) -> None:
        """Queue one raw 32-bit TX FIFO word, waiting for room first (TXOVER).

        `push` is for slot values, which are bytes in bits 7:0.  This one is for a host that has
        chosen `SHIFTCTRL.OUT_SHIFTDIR = 0`, where each slot value has to be *left*-justified in
        bits 31:24 instead, because the OSR shifts the other way and the last eight of a slot's
        32 shifts are the ones the wire sees.  See `fw/dmx512/test_dmx512.py`'s
        `test_the_slot_values_have_to_be_right_justified`.
        """
        if not 0 <= word <= 0xFFFF_FFFF:
            raise ValueError(f"a TX FIFO word is 32 bits (got {word!r})")
        while self.tx_full:
            self._settle(1)
        self.write(A_TXF[self.engine], word)
        self._settle(1)

    def preload(self, values: Sequence[int]) -> None:
        """Fill the TX FIFO while the engine is stopped.

        Only `TXFIFO_DEPTH` words fit and a disabled engine never pops, so this is how a host
        gets the first slots in before starting.  The first frame does not reach its first
        refill for thousands of ticks, so `send` does not need this; it is here for a host that
        wants the FIFO full before the engine's first SM cycle.
        """
        values = list(values)
        if len(values) > self.TXFIFO_DEPTH:
            raise ValueError(
                f"{len(values)} slot values do not fit the stopped {self.TXFIFO_DEPTH}-deep TX "
                "FIFO; call send() instead"
            )
        for value in values:
            self.push_word(value)

    def feed(self, values: Sequence[int], packets: int = 1, preloaded: int = 0,
             justify=None) -> int:
        """Transmit `packets` frames whose slot values are `values`, cyclically.

        One FIFO word is one slot, the engine takes one word per 11 bit times, and a refill
        that finds the FIFO empty costs a whole SM cycle -- a whole bit time of jitter, where
        the tables' tolerance is two ticks.  So the host keeps the FIFO topped up while the bus
        runs, which is the one piece of timing a DMX512 controller cannot delegate to hardware.

        `preloaded` is how many words `start(first=...)` pushed before the engine ran.  They are
        the head of the same list, so they are not sent twice, and it has to be passed in rather
        than read back from FLEVEL: the engine's first instruction is a `pull block`, which has
        already taken one of those words by the time the first host access of this call returns,
        so the live level is one short of what is really on its way.

        Stops as soon as the last word has been taken, with a few ticks left over so the last
        slot's two stop bits are on the wire: what follows is the pad, which only drives mark,
        and then the next frame's BREAK, so a run that stops here leaves exactly `packets`
        frames for `DmxChecker` to measure.  Returns the number of ticks it ran.
        """
        if packets < 1:
            raise ValueError(f"packets={packets}: there is no frame to send")
        values = [int(v) for v in values]
        if not values:
            raise ValueError("a frame needs at least a START Code, so `values` cannot be empty")
        if justify is not None:
            values = [int(justify(v)) for v in values]
        # One extra word: the program's first instruction is a `pull block`, so the first slot's
        # byte comes out of the FIFO before the first frame has sent anything.  One word per
        # slot after that, so `packets` frames are `packets * slots + 1` words -- and whatever
        # the caller already preloaded (see `send`) is the head of the same list, so it is
        # skipped here rather than sent twice.
        stream = [values[i % len(values)] for i in range(1 + packets * self.slots)]
        if not 0 <= preloaded <= len(stream):
            raise ValueError(
                f"preloaded={preloaded}: the run needs {len(stream)} words, so a preload has "
                f"to be between 0 and {len(stream)}"
            )
        stream = stream[preloaded:]
        for word in stream:
            if not 0 <= word <= 0xFFFF_FFFF:
                raise ValueError(f"a TX FIFO word is 32 bits (got {word!r})")
            if justify is None and not 0 <= word <= 0xFF:
                raise ValueError(
                    f"a DMX512 slot value is 0..255 (got {word!r}); a 32-bit word only gets "
                    "here through `justify`, which is how a host left-justifies its bytes when "
                    "SHIFTCTRL.OUT_SHIFTDIR is 0"
                )
        first = self.bus.t
        pushed = 0
        while pushed < len(stream):
            # Top the FIFO up first: the engine's first refill is `break + MAB + 9 bits` into
            # the frame, thousands of ticks away, so the four words of a stopped FIFO are all
            # in place long before it needs one.  After that the engine takes one word per slot
            # and one word per CHUNK is enough to stay ahead of it.
            while pushed < len(stream) and not self.tx_full:
                self.push_word(stream[pushed])
                pushed += 1
            for _ in range(self.CHUNK):
                self.bus.step()
        # The last word is gone, so the engine is on its eighth `out pins, 1` of the final slot
        # and three instructions of that slot are still to come: the two stop bits and the
        # loop's jump.  Three bit times is more than the two stop bits need and more than a
        # receiver's last mid-bit sample (the eleventh, at 10.5 bit times after the slot's start
        # bit, which is 2.5 after the refill) and it is less than the mark before the next
        # frame's BREAK, which is at least the pad block's 35 SM cycles on top of those three.
        # So the run stops with the last frame complete and no fourth frame begun.
        while self.tx_level:
            for _ in range(8):
                self.bus.step()
        for _ in range(int(3 * self.plan.tick_ticks) + 8):
            self.bus.step()
        return self.bus.t - first

    def send(self, values: Sequence[int], packets: int = 1, preloaded: int = 0,
             justify=None) -> int:
        """`load()`, `start()` with the FIFO filled, then `feed()`: one whole transaction.

        The FIFO is filled *before* the engine is enabled because the program's first
        instruction is a `pull block`, which stalls on an empty FIFO: the engine's first SM
        cycle is two core cycles after the enabling write, which is far too soon for a host
        access to get there first.  Filling it while the engine is stopped is what
        `docs/mvp-regmap.md` `### Typical host sequences` means by arming before enabling, and
        it is why the run that follows has no FDEBUG.TXSTALL on it.
        """
        values = [int(v) for v in values]
        if not values:
            raise ValueError("a frame needs at least a START Code, so `values` cannot be empty")
        self.load()
        if not preloaded:
            # as much of the run's word list as the stopped FIFO holds; the shortest frame
            # (a START Code and nothing else) needs fewer words than the FIFO is deep
            preloaded = min(self.TXFIFO_DEPTH, 1 + packets * self.slots)
        # `justify` goes to both halves rather than to the list, so the bytes are placed in a
        # word exactly once on the way in and once on the way to each FIFO write
        self.start(first=[values[i % len(values)] for i in range(preloaded)], justify=justify)
        return self.feed(values, packets, preloaded=preloaded, justify=justify)

    # ---- CTRL helpers ----
    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask

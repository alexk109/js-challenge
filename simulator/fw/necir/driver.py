"""fw/necir/driver.py -- the host side of the NEC infrared receiver (task B1-FW-necir).

Everything here is written against `docs/mvp-regmap.md` (word addresses and field positions)
and `docs/mvp-isa.md` (divider, side-set, autopush, EXEC), and every host access goes through
`iss.pio_iss.ChipDevice`'s queue, one per core cycle, on a `verif.vip.core.Bus`.

The chip is the **target**: the model's `NecirTx` drives the net and this firmware decodes it.
So the host's job is short. In the order of `docs/mvp-regmap.md` `### Typical host sequences`:

1. `CTRL` with the engine disabled, then `program.pio` (assembled by `asm/pioasm.py`) into
   the engine's IMEM.
2. `CLKDIV`, `EXECCTRL` (wrap, `JMP_PIN`), `SHIFTCTRL` (autopush, `PUSH_THRESH`, `!osre`
   threshold), `PINCTRL` (`IN_BASE`, and `SIDESET_COUNT = 0`).
3. A forced `jmp start`, because `SMe_ADDR` is read-only and that is the only way to set the PC.
4. `CTRL` with `SM_ENABLE`, `SM_RESTART` and `CLKDIV_RESTART` in the same write.
5. Read `RXF0` whenever `FLEVEL` says the FIFO holds a word. A word is the 32 data bits of one
   NEC frame, LSB of the word first, exactly as they were on the wire: word 0 in bits 7:0, its
   inverse in bits 15:8, the command in bits 23:16, its inverse in bits 31:24. `verif.vip.necir`
   has `decode_frame()` for that, and `decode_frame(word) == encode_frame(address, command)`.

There is nothing to send: a receiver never writes `TXF0` and never needs a forced instruction
after the start, so one press costs exactly one RX FIFO word and a repeat code costs nothing.

The one number worth reading twice
----------------------------------
NEC's timings are fixed, so the only "speed" here is how finely the engine's SM cycle divides
the 562.5 us burst. `S` (SM cycles per burst) is what the program's sample points are built
from, and `sample_window()` computes -- from the assembled words and the model's own NEC
windows -- the margin each of them has at a given checker `tolerance`. It raises rather than
load a program whose window is empty, which is why `build_program` accepts S = 5..8 and not
S = 1; program.pio's header explains the arithmetic.
"""
from __future__ import annotations

from fw import regmap

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from asm.pioasm import Program, assemble
from iss.pio_iss import Chip, ChipDevice
from verif.vip.core import Bus

# ---------------------------------------------------------------------------------------------
# Constants and register addresses (docs/mvp-regmap.md `### Address map`)
# ---------------------------------------------------------------------------------------------
CORE_HZ = 50_000_000              # one Bus tick at 20 ns, 50 MHz (verif/vip/core.py)
NS_PER_TICK = 20.0
NUM_PINS = 18                    # docs/mvp-isa.md `### The 18 protocol pins`
IMEM_SIZE = 32                   # words of instruction memory per engine
UIO_PINS = range(0, 8)           # bidirectional; input-only pins 8..12 work too, 13..17 do not
RX_PINS = range(0, 13)           # a pin with an input path: 8..12 are input-only
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "necir"

#: One NEC burst in nanoseconds, as `verif.vip.necir.NECIR_TIMING` spells it
#: (`bit_mark_ns = 562.5`).  Every element of the protocol is a whole multiple of it -- 16 / 8 /
#: 4 / 3 / 1 -- which is what makes the whole thing integer arithmetic on SM cycles.
#:
#: **The model's NEC runs 1000x faster than a real remote's.**  A real NEC burst is 562.5 us
#: (562_500 ns) and a real leader 9 ms; this model says 562.5 ns and 9 us, and its own README
#: does the arithmetic at that scale ("one tick is 3.56 % of a 562.5 us burst" = 20 ns / 562.5
#: ns, research/ref/protocols/necir/README.md judgement call 2).  Everything here is written
#: against the model, because the model is the judge, but nothing here depends on the scale:
#: `unit_ns` is a parameter of the receiver, and `REAL_NEC_UNIT_NS` below is the value a real
#: remote would need.  The only thing the scale changes is CLKDIV, and 28125 core cycles per
#: burst still fits the 16-bit INT with room to spare.
NEC_UNIT_NS = 562.5
REAL_NEC_UNIT_NS = 562_500.0

CTRL = 0x00
FSTAT = 0x01
FDEBUG = 0x02
FLEVEL = 0x03
TXF = regmap.txf_addrs()
RXF = regmap.rxf_addrs()
DBG_PADOUT = 0x0F
DBG_PADOE = 0x10
DBG_CFGINFO = 0x11
INTR = regmap.A_INTR
SM_NAMES = ("CLKDIV", "EXECCTRL", "SHIFTCTRL", "ADDR", "INSTR", "PINCTRL")

# CTRL fields (docs/mvp-regmap.md `**CTRL**`)
CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART = 0x001, 0x010, 0x100
# FSTAT / FLEVEL fields for engine e
FSTAT_RXEMPTY, FSTAT_RXFULL = 0x0100_0000, 0x0000_0001
# FDEBUG: bit + e of each field belongs to engine e
FDEBUG_TXSTALL, FDEBUG_RXOVER, FDEBUG_RXUNDER, FDEBUG_RXSTALL = (
    0x0100_0000, 0x0001_0000, 0x0000_0100, 0x0000_0001)
FDEBUG_BAD = FDEBUG_TXSTALL | FDEBUG_RXOVER | FDEBUG_RXUNDER | FDEBUG_RXSTALL
# DBG_CFGINFO of an MVP PIO block (docs/mvp-regmap.md `**DBG_CFGINFO**`)
DBG_CFGINFO_MVP = 0x0020_0204


def imem_addr(engine: int, word: int) -> int:
    """`INSTR_MEM0..31` of `engine` (engine 0: 0x12..0x31, engine 1: 0x52..0x71)."""
    if engine not in (0, 1):
        raise ValueError(f"engine {engine} out of range (the MVP has two)")
    return regmap.imem_addr(engine, word)


def sm_addr(engine: int, name: str) -> int:
    """`SMe_CLKDIV`, `SMe_EXECCTRL`, ... : word `0x32 + 6 * engine + i`."""
    if name not in SM_NAMES:
        raise ValueError(f"unknown SMe register {name!r} (expected one of {SM_NAMES})")
    return regmap.sm_addr(engine, SM_NAMES.index(name))


# ---------------------------------------------------------------------------------------------
# Register values, packed from the per-register tables of docs/mvp-regmap.md
# ---------------------------------------------------------------------------------------------
def clkdiv_value(cycles_per_sm_cycle: float) -> int:
    """`SMe_CLKDIV` from a divisor in core cycles per SM cycle (INT in 31:16, FRAC in 15:8).

    `docs/mvp-isa.md` `### Clock divider` fixes the algorithm, so an SM cycle is `INT + FRAC/256`
    core cycles on average and the mean rate error is 1/256 of a cycle per SM cycle whatever
    this rounds to.  `round_error()` below measures it.
    """
    if cycles_per_sm_cycle < 1.0:
        raise ValueError(
            f"CLKDIV must be at least 1.0 core cycle (INT = 0 means 65536), got "
            f"{cycles_per_sm_cycle!r}"
        )
    if cycles_per_sm_cycle > 0xFFFF + 255 / 256:
        raise ValueError(
            f"CLKDIV INT is 16 bits, so an SM cycle cannot be longer than 65536.996 core "
            f"cycles; got {cycles_per_sm_cycle!r}"
        )
    integer = int(cycles_per_sm_cycle)
    frac = int(round((cycles_per_sm_cycle - integer) * 256))
    if frac > 255:                      # rounding carried into INT
        integer, frac = integer + 1, 0
    return (integer << 16) | (frac << 8)


def cycles_per_sm_cycle(clkdiv: int) -> float:
    """The average core cycles one SM cycle takes with this `SMe_CLKDIV` word."""
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    return integer + ((clkdiv >> 8) & 0xFF) / 256.0


def round_error(clkdiv: int, cycles_wanted: float) -> float:
    """How far the divider's average rate is from the divisor that was asked for."""
    return cycles_per_sm_cycle(clkdiv) - cycles_wanted


def execctrl_value(program: Program, *, jmp_pin: int, side_en: int = 0, side_pindir: int = 0,
                   wrap_bottom: Optional[int] = None, status_sel: int = 0,
                   status_n: int = 0) -> int:
    """`SMe_EXECCTRL`: `JMP_PIN` for the samples, the wrap window, and no side-set enable.

    `JMP_PIN` is an absolute pin index, so every `jmp pin` in program.pio reads the IR pin the
    host put here (docs/mvp-isa.md `### Field values`, JMP). `SIDE_EN = 0` with
    `PINCTRL.SIDESET_COUNT = 0` means no instruction side-sets anything; see program.pio's
    header for why that matters on a pin this program only listens to.
    """
    if not 0 <= jmp_pin <= 31:
        raise ValueError(f"JMP_PIN {jmp_pin} does not fit in 5 bits")
    bottom = program.wrap_target if wrap_bottom is None else wrap_bottom
    return ((side_en << 30) | (side_pindir << 29) | (jmp_pin << 24) | (program.wrap << 12)
            | (bottom << 7) | (status_sel << 4) | status_n)


def _threshold_field(threshold: int, name: str) -> int:
    """A 5-bit PUSH_THRESH / PULL_THRESH field from a threshold of 1..32.

    The fields are five bits and `docs/mvp-isa.md` `## Conventions` says `thresh(f) = f if f
    != 0 else 32`, so **32 is written as 0**: `32 << 25` would be bit 30, which is not in the
    field at all, and `32 << 20` would land in PULL_THRESH as the value 1.  Both mistakes are
    silent -- the chip stores a number and the engine simply stops early -- which is why this is
    a function and not an expression.
    """
    if not 1 <= threshold <= 32:
        raise ValueError(f"{name} {threshold} out of range: the field is 0..31 and means 1..32")
    return 0 if threshold == 32 else threshold


def shiftctrl_fields(shiftctrl: int) -> dict:
    """The fields of a `SMe_SHIFTCTRL` word, thresholds read back through `thresh()`.

    The mirror of the packing below, so a test can check what the chip stored rather than what
    the driver meant.
    """
    return {
        "pull_thresh": ((shiftctrl >> 25) & 0x1F) or 32,
        "push_thresh": ((shiftctrl >> 20) & 0x1F) or 32,
        "out_shiftdir": (shiftctrl >> 19) & 1,
        "in_shiftdir": (shiftctrl >> 18) & 1,
        "autopull": (shiftctrl >> 17) & 1,
        "autopush": (shiftctrl >> 16) & 1,
    }


def shiftctrl_value(*, push_thresh: int = 32, pull_thresh: int = 32, in_right: int = 1,
                    out_right: int = 1, autopush: int = 0, autopull: int = 0) -> int:
    """`SMe_SHIFTCTRL`: autopush the frame, shift data in at the LSB, and 32 for `!osre`.

    * `IN_SHIFTDIR` keeps its reset value of 1 (shift right, data enters at the MSB).  The ISR is
      a shift register, so 32 right shifts leave the *first* bit shifted in at bit 0 and the last
      at bit 31 -- which is the wire's order, LSB of word 0 first (docs/mvp-isa.md `### Reference
      step`, `IN_SHIFTDIR = 1`; `isr' = (ISR >> n) | (d << (32 - n))`).  Shifting *left* would put
      the first bit at bit 31, i.e. the frame reversed.
    * `AUTOPUSH = 0`, and the program pushes with an explicit `push block`: the ISR holds the
      *sampled levels*, which are the inverse of the data bits, and the program inverts the
      finished word with `mov isr, isr !` before pushing it.  Autopush would fire on the 32nd
      `in` and push the word *before* that inversion, so it has to be off.  PUSH_THRESH is then
      unused; it is written as 32 (= field 0) so a reader sees the full-width threshold.
    * `PULL_THRESH = 32` with `AUTOPULL = 0`: `!osre` is `osr_cnt < thresh(PULL_THRESH)`, and
      `out null, 1` is what raises `osr_cnt`. That is the program's bit counter -- and it has to
      be 32, or the loop stops after one bit.
    * `OUT_SHIFTDIR` keeps its reset value of 1 and is unused: the OSR is never sent, it only
      counts.
    """
    return ((_threshold_field(pull_thresh, "PULL_THRESH") << 25)
            | (_threshold_field(push_thresh, "PUSH_THRESH") << 20) | (out_right << 19)
            | (in_right << 18) | (autopull << 17) | (autopush << 16))


def pinctrl_value(pin: int, *, in_base: Optional[int] = None, set_count: int = 0,
                  out_count: int = 0, sideset_count: int = 0) -> int:
    """`SMe_PINCTRL`: the IN window on the IR pin, and *no* window that writes a pin.

    Every `wait pin, 0` in the program uses index 0 of the IN window, and the program issues no
    `out`, `set` or side-set at all, so the OUT, SET and side-set windows are all empty.  The
    engine therefore never touches the IR pin's direction register: it stays an input, which is
    what a receiver needs (docs/mvp-isa.md `## Pins`).

    `SIDESET_COUNT = 0` is not the reset value and has to be written: the reset PINCTRL is
    `0x1400_0000`, i.e. `SIDESET_COUNT = 2`, and with `SIDE_EN = 0` that makes *every*
    instruction side-set two pins with data taken from the top of the delay field
    (docs/mvp-isa.md `### Delay/side-set field`) -- on a pin this program must leave alone.
    """
    if not 0 <= pin < NUM_PINS:
        raise ValueError(f"pin {pin} is outside 0..{NUM_PINS - 1}")
    base = pin if in_base is None else in_base
    if not 0 <= base <= 31:
        raise ValueError(f"IN_BASE {base} does not fit in 5 bits")
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (base << 15))


# ---------------------------------------------------------------------------------------------
# The program: assemble it for an S, and read the sample points back out of the words
# ---------------------------------------------------------------------------------------------
DEFAULT_SM_CYCLES_PER_UNIT = 9        # program.pio's `.define S`
MIN_SM_CYCLES_PER_UNIT, MAX_SM_CYCLES_PER_UNIT = 7, 12


class NecirError(RuntimeError):
    """The engine, the program or the bus did something the driver cannot make sense of."""


def _set_define(text: str, name: str, value: int) -> str:
    """Return `text` with the single `.define NAME ...` line's value replaced by `value`."""
    pattern = re.compile(rf"(?m)^(\s*\.define\s+{re.escape(name)}\s+)\S+")
    new, count = pattern.subn(rf"\g<1>{value}", text)
    if count != 1:
        raise NecirError(
            f"{PROGRAM_PATH.name}: expected exactly one '.define {name}' line, found {count}"
        )
    return new


def build_program(sm_cycles_per_unit: int = DEFAULT_SM_CYCLES_PER_UNIT,
                  tolerance: float = 0.25, unit_ns: float = NEC_UNIT_NS,
                  ns_per_tick: float = NS_PER_TICK, core_hz: float = CORE_HZ) -> Program:
    """Assemble program.pio for `sm_cycles_per_unit` SM cycles per NEC burst.

    The three sample cycles are *computed*, not written in the file: `sample_window` says where
    each may fall and `choose_sample_cycle` picks inside that window, so an S whose window is
    empty -- S = 1..6 -- is refused here rather than loaded.  The four `.define` lines are then
    rewritten before assembling, because the assembler may not redefine a symbol; that is the same
    trick `fw/uart_rx/driver.py` uses for its `.define S`.
    """
    if isinstance(sm_cycles_per_unit, bool) or not isinstance(sm_cycles_per_unit, int):
        raise NecirError(f"S must be a whole number of SM cycles, got {sm_cycles_per_unit!r}")
    if not MIN_SM_CYCLES_PER_UNIT <= sm_cycles_per_unit <= MAX_SM_CYCLES_PER_UNIT:
        raise NecirError(
            f"S = {sm_cycles_per_unit} is outside {MIN_SM_CYCLES_PER_UNIT}.."
            f"{MAX_SM_CYCLES_PER_UNIT}: below that the data-space sample's window is empty, above "
            f"it a pad delay does not fit the 5-bit delay field (program.pio's header has both)"
        )
    cycles = sample_cycles(sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)
    text = PROGRAM_PATH.read_text(encoding="utf-8")
    for name in ("S", "MARK_CYCLES", "SPACE_CYCLES", "K"):
        text = _set_define(text, name, cycles[name])
    programs = assemble(text)
    if len(programs) != 1:
        raise NecirError(f"{PROGRAM_PATH.name} must hold exactly one .program")
    program = programs[0]
    if program.name != PROTO_NAME:
        raise NecirError(f"{PROGRAM_PATH.name} assembles a program named {program.name!r}")
    if len(program.instrs) > 32:
        raise NecirError(f"the program is {len(program.instrs)} words, IMEM is 32")
    check_program(program, sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)
    return program


def load_program(sm_cycles_per_unit: int = DEFAULT_SM_CYCLES_PER_UNIT,
                 tolerance: float = 0.25) -> Program:
    """`build_program` under the name the other firmware modules use for "assemble my .pio"."""
    return build_program(sm_cycles_per_unit, tolerance)


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


# ---- instruction decoding, so the timing is read back from the words and not assumed ----
OP_JMP, OP_WAIT, OP_IN, OP_OUT, OP_SHIFT, OP_MOV, OP_IRQ, OP_SET = range(8)
JMP_PIN, WAIT_PIN = 0b110, 0b01
OUT_NULL, IN_PINS, IN_X, IN_NULL = 0b011, 0b000, 0b001, 0b011

#: SM cycles the bit loop spends *after* its sample before it is back at `wait 1 pin, 0`:
#: `in pins, 1` (the sample itself), `out null, 1` and `jmp !osre bit`.  This is what bounds how
#: late the sample may be: the engine has to be waiting for the next burst *while that burst is
#: still high*, or it misses it and loses its place in the stream.  See `sample_window`.
LOOP_OVERHEAD = 3


def decode_instruction(program: Program, word: int) -> dict:
    """Split one word into the fields of `docs/mvp-isa.md` `## Encoding` the driver reads.

    Only what the timing analysis needs: the opcode, the delay, and the condition /
    destination / source fields.  `sc` counts the side-set bits including the `opt` enable, so
    the delay is the low `5 - sc` bits of bits 12:8, as the spec says.
    """
    sc = min(program.sideset_bits, 5)
    field5 = (word >> 8) & 0x1F
    return {
        "op": word >> 13,
        "delay": field5 & ((1 << (5 - sc)) - 1),
        "cond": (word >> 5) & 7,          # JMP condition
        "op_field": (word >> 3) & 3,      # MOV op: 0 none, 1 invert, 2 bit-reverse
        "dest": (word >> 5) & 7,
        "src": word & 7,                  # MOV source / IN source / OUT destination
        "wait_src": (word >> 5) & 3,      # WAIT source: 00 gpio, 01 pin, 10 irq, 11 reserved
        "pol": (word >> 7) & 1,           # WAIT polarity
        "index": word & 0x1F,
        "sideset": (field5 >> (5 - sc)) if sc else 0,
    }


def _is(word: dict, **fields) -> bool:
    return all(word[name] == value for name, value in fields.items())


def sample_points(program: Program) -> dict:
    """{name: SM cycles after the `wait` that starts the measurement}, from the words.

    Every timing-critical decision in program.pio is either a `jmp pin` or the `in pins, 1` that
    samples the level, placed a fixed number of SM cycles after a `wait` by a delay.  So the
    offsets are read back by walking the instruction stream and adding up `1 + delay` from each
    `wait`.  The names are the three samples, in program order: the leader-mark sample, the
    leader-space sample and the data-space sample.
    """
    points: dict = {}
    names = ("leader_mark", "leader_space", "bit_space")
    cycle = 0
    for word in program.instrs:
        fields = decode_instruction(program, word)
        is_pin_jump = _is(fields, op=OP_JMP, cond=JMP_PIN)
        is_sample = _is(fields, op=OP_IN, dest=IN_PINS, index=1)
        if is_pin_jump or is_sample:
            name = names[len(points)] if len(points) < len(names) else f"extra_{len(points)}"
            points[name] = cycle
        if _is(fields, op=OP_WAIT):
            cycle = 0                    # this wait is the reference the next sample counts from
        cycle += 1 + fields["delay"]
    if len(points) != 3:
        raise NecirError(
            f"{PROGRAM_PATH.name} has {len(points)} samples {sorted(points)}, but the timing "
            f"analysis knows three (leader mark, leader space, data space)"
        )
    return points


# ---------------------------------------------------------------------------------------------
# The sample windows: where a `jmp pin` has to fall for the wire the checker calls legal
# ---------------------------------------------------------------------------------------------
# Nominal element lengths in NEC bursts (research/ref/protocols/necir/README.md): the leader
# mark is 16 bursts ("16 times the pulse burst length", Renesas), a frame's leader space 8, a
# repeat code's 4, a data burst 1, and a data space 1 for a '0' or 3 for a '1'.  The checker's
# `tolerance` widens each of them symmetrically, so an element may be anywhere in
# [(1-t), (1+t)] of its nominal.
def _legal(lo: float, hi: float, tolerance: float) -> tuple[float, float]:
    """The shortest and longest length a checker with this `tolerance` calls legal."""
    return lo * (1.0 - tolerance), hi * (1.0 + tolerance)


#: How many SM cycles the four leader-mark pads can reach, and the two space pads: an
#: instruction's delay is 5 bits, so one pad is at most 32 SM cycles.  `sample_window` clamps the
#: leader-mark sample to MARK_REACH and the leader-space sample to SPACE_REACH, which is what
#: bounds S from above as well as the empty-window check does from below.
MARK_REACH, SPACE_REACH = 129, 65

#: SM cycles of room the leader-mark sample keeps below the shortest *legal* leader.  The sample
#: goes as late as the window allows on purpose: that is what rejects a leader that is too short
#: (the checker's leader-mark window is 12..20 bursts at its default tolerance, so a sample just
#: under 12 bursts accepts every legal leader and rejects everything shorter).
LEAD_MARK_HEADROOM = 4


def sample_window(sm_cycles_per_unit: int, tolerance: float = 0.25,
                  unit_ns: float = NEC_UNIT_NS, ns_per_tick: float = NS_PER_TICK,
                  core_hz: float = CORE_HZ) -> dict:
    """Where each of program.pio's three samples must fall, in SM cycles.

    All three are read off a `wait`, so the edge that `wait` stopped on is only dated to within
    one SM cycle: that is the `-1` (and the exclusive bounds) below, and it is why S = 1 cannot
    work.  `jitter` is the bus tick the model's transmitter quantises its edges to, in SM cycles
    -- one 20 ns tick against a `S`-per-burst SM cycle, 0.32 of one at S = 9, which is why the
    windows are worth computing rather than eyeballing.

    The window per sample is the set of integers `k` with `lo <= k < hi`:

      leader_mark    still high for every legal leader (16 bursts) and past every data burst
                     (1 burst), which is what refuses to sync into the middle of a frame;
      leader_space   high for every repeat code's 4 burst space and low for every frame's 8
                     burst one, which is how "a repeat carries no data" is decided;
      bit_space      high for every legal '0' space, and early enough that the bit loop is back
                     at `wait 1 pin, 0` before the burst after that space ends.  The first bound
                     is why this window is narrow: a '0' space may be 1.25 bursts and the burst
                     after it as short as 0.75, so only half a burst separates "the space ended"
                     from "the burst ended" -- and the second bound is that a '0' bit is two
                     bursts long and a whole loop iteration has to fit inside it.

    Every bound is in SM cycles; `sample_margins()` turns them into nanoseconds.
    """
    if unit_ns <= 0.0:
        raise NecirError(f"a burst cannot last {unit_ns!r} ns: there is no time in it to sample")
    S = float(sm_cycles_per_unit)
    cycles_per_sm_cycle = core_hz * unit_ns / 1e9 / S
    # The model's transmitter rounds every edge to a bus tick, so its bursts come out up to one
    # tick longer or shorter than nominal.  One tick is `ns_per_tick * core_hz / 1e9` core cycles
    # (a ChipDevice steps one core cycle per bus tick), which is `jitter` SM cycles here.
    tick_cycles = ns_per_tick * core_hz / 1e9
    jitter = tick_cycles / cycles_per_sm_cycle if cycles_per_sm_cycle else 0.0
    lead_lo, lead_hi = _legal(16.0, 16.0, tolerance)
    frame_lo, frame_hi = _legal(8.0, 8.0, tolerance)
    rep_lo, rep_hi = _legal(4.0, 4.0, tolerance)
    burst_lo, burst_hi = _legal(1.0, 1.0, tolerance)
    zero_lo, zero_hi = _legal(1.0, 1.0, tolerance)

    leader_mark = {"lo": burst_hi * S + jitter, "hi": min(lead_lo * S - 1.0, MARK_REACH)
                   - jitter, "sense": "high", "prefer": "latest"}
    leader_space = {"lo": rep_hi * S - jitter, "hi": min(frame_lo * S - 1.0, SPACE_REACH)
                    - jitter, "sense": "high", "prefer": "middle"}
    bit_space = {"lo": zero_hi * S - jitter,
                 "hi": min((zero_hi + burst_lo) * S - 1.0,
                           2 * S - 1.0 - LOOP_OVERHEAD) - jitter,
                 "sense": "high", "prefer": "middle"}
    out = {"sm_cycles_per_unit": int(sm_cycles_per_unit), "tolerance": float(tolerance),
           "unit_ns": float(unit_ns), "ns_per_tick": float(ns_per_tick),
           "cycles_per_sm_cycle": cycles_per_sm_cycle, "jitter_cycles": jitter,
           "unit_cycles": core_hz * unit_ns / 1e9,
           "leader_mark": leader_mark, "leader_space": leader_space, "bit_space": bit_space}
    for win in (leader_mark, leader_space, bit_space):
        win["width"] = win["hi"] - win["lo"]
        win["empty"] = not (math.ceil(win["lo"]) < win["hi"])
    return out


def choose_sample_cycle(window: dict, name: str = "bit_space") -> int:
    """The SM cycle to put this sample on, and raise `NecirError` if nowhere is left.

    `prefer` says how to pick inside the window: "middle" (the default) for the two samples whose
    job is to tell two lengths apart, "latest" for the leader mark, where the point is to accept
    every legal leader -- so the sample sits just inside the shortest one and rejects as much as
    possible below it, `LEAD_MARK_HEADROOM` SM cycles short of the bound.
    """
    win = window[name]
    first, last = math.ceil(win["lo"]), math.ceil(win["hi"]) - 1
    if first > last:
        raise NecirError(
            f"the {name.replace('_', ' ')} sample has nowhere to go at S = "
            f"{window['sm_cycles_per_unit']} and tolerance {window['tolerance']:.0%}: it must be "
            f"an integer in [{win['lo']:.3f}, {win['hi']:.3f}) SM cycles, which is empty. "
            f"program.pio's header has the arithmetic"
        )
    if win.get("prefer") == "latest":
        return max(first, last - LEAD_MARK_HEADROOM)
    return min(max(round((win["lo"] + win["hi"]) / 2.0), first), last)


def sample_window_of(program: Program, tolerance: float = 0.25,
                     unit_ns: float = NEC_UNIT_NS, ns_per_tick: float = NS_PER_TICK,
                     core_hz: float = CORE_HZ) -> Optional[dict]:
    """The window `program` was built against, or None if it has no single S.

    A program carries its S nowhere the assembler hands back, so this reads it off the two
    `jmp pin` samples: the leader-space sample is the middle of the gap between a repeat code's
    4 burst space and a frame's 8, which fixes S up to the tolerance it was built at.
    """
    points = sample_points(program)
    window = sample_window(DEFAULT_SM_CYCLES_PER_UNIT, tolerance, unit_ns, ns_per_tick, core_hz)
    for sm_cycles_per_unit in range(MIN_SM_CYCLES_PER_UNIT, MAX_SM_CYCLES_PER_UNIT + 1):
        try:
            want = sample_cycles(sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)
        except NecirError:
            continue
        if (want["MARK_CYCLES"] == points["leader_mark"]
                and want["SPACE_CYCLES"] == points["leader_space"]
                and want["K"] == points["bit_space"]):
            return window_of(sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)
    return None


def window_of(sm_cycles_per_unit: int, tolerance: float = 0.25,
              unit_ns: float = NEC_UNIT_NS, ns_per_tick: float = NS_PER_TICK,
              core_hz: float = CORE_HZ) -> dict:
    """`sample_window`, under a name that does not shadow the module's own."""
    return sample_window(sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)


def clkdiv_fields(clkdiv: int) -> dict:
    """The fields of a `SMe_CLKDIV` word, INT resolved the way the spec resolves it."""
    return {"int": (clkdiv >> 16) & 0xFFFF, "frac": (clkdiv >> 8) & 0xFF,
            "cycles_per_sm_cycle": cycles_per_sm_cycle(clkdiv)}


def sample_cycles(sm_cycles_per_unit: int, tolerance: float = 0.25,
                  unit_ns: float = NEC_UNIT_NS, ns_per_tick: float = NS_PER_TICK,
                  core_hz: float = CORE_HZ) -> dict:
    """All three sample cycles for one S, in the order program.pio's `.define` lines want them."""
    window = sample_window(sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)
    return {"S": int(sm_cycles_per_unit), "window": window,
            "MARK_CYCLES": choose_sample_cycle(window, "leader_mark"),
            "SPACE_CYCLES": choose_sample_cycle(window, "leader_space"),
            "K": choose_sample_cycle(window, "bit_space")}


def sample_margins(program: Program, sm_cycles_per_unit: Optional[int] = None,
                   tolerance: float = 0.25, unit_ns: float = NEC_UNIT_NS,
                   ns_per_tick: float = NS_PER_TICK, core_hz: float = CORE_HZ) -> dict:
    """How much timing room each sample of `program` has, in SM cycles and in nanoseconds.

    Read back out of the words (`sample_points`) and compared with `sample_window`, so this is a
    measurement of the assembled program rather than a restatement of its header.
    """
    points = sample_points(program)
    if sm_cycles_per_unit is None:
        raise NecirError("sample_margins needs the S the program was assembled for")
    S = int(sm_cycles_per_unit)
    window = sample_window(S, tolerance, unit_ns, ns_per_tick, core_hz)
    cycles = window["cycles_per_sm_cycle"]
    out = {"sm_cycles_per_unit": S, "samples": {}, "window": window,
           "cycles_per_sm_cycle": cycles, "unit_cycles": window["unit_cycles"]}
    for name, at in points.items():
        win = window[name]
        out["samples"][name] = {
            "at": at,
            "lo_margin_cycles": at - win["lo"],
            "hi_margin_cycles": win["hi"] - at,
            "lo_margin_ns": (at - win["lo"]) * cycles * NS_PER_TICK,
            "hi_margin_ns": (win["hi"] - at) * cycles * NS_PER_TICK,
            "in_window": win["lo"] <= at < win["hi"],
        }
    return out


def check_program(program: Program, sm_cycles_per_unit: Optional[int] = None,
                  tolerance: float = 0.25, unit_ns: float = NEC_UNIT_NS,
                  ns_per_tick: float = NS_PER_TICK, core_hz: float = CORE_HZ) -> None:
    """Fail loudly if the driver and program.pio disagree about anything that matters.

    The `.define` lines are not handed back by the assembler, so what is checked instead is the
    instructions that carry the timing and the structure: the three samples, where
    `sample_points` says they are against where `sample_cycles` says they should be, the
    instruction each decision is built from, the bit counter's `out null, 1` and `jmp !osre`, the
    un-inversion and push of a finished frame, and the absence of any pin write or side-set.  A
    program whose padding has been edited by hand therefore fails here instead of decoding at the
    wrong time.
    """
    points = sample_points(program)
    if sm_cycles_per_unit is not None:
        want = sample_cycles(sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)
        for name, key in (("leader_mark", "MARK_CYCLES"), ("leader_space", "SPACE_CYCLES"),
                          ("bit_space", "K")):
            if points[name] != want[key]:
                raise NecirError(
                    f"the {name.replace('_', ' ')} sample is at {points[name]} SM cycles but S = "
                    f"{sm_cycles_per_unit} wants {want[key]}"
                )
    if program.sideset_bits or program.sideset_opt or program.sideset_pindirs:
        raise NecirError(
            f"program.pio asks for {program.sideset_bits} side-set bit(s) (opt="
            f"{program.sideset_opt}, pindirs={program.sideset_pindirs}); a receiver must not "
            f"write the net it listens to"
        )
    words_used_program = words_used(program)
    if program.wrap != words_used_program - 1 or program.wrap_target != program.labels["leader"]:
        raise NecirError(
            f"WRAP_TOP {program.wrap} / WRAP_BOTTOM {program.wrap_target} do not bracket the "
            f"program ({words_used_program} words, leader at {program.labels['leader']})"
        )
    words = [decode_instruction(program, w) for w in program.instrs]
    by_label = {name: words[program.labels[name]] for name in program.labels}
    last = words_used_program - 1
    bit = program.labels["bit"]
    if not (_is(by_label["bit"], op=OP_WAIT) and _is(by_label["bit"], pol=1, wait_src=WAIT_PIN,
                                                    index=0)):
        raise NecirError("the bit loop does not start with `wait 1 pin, 0`")
    if not _is(words[bit + 1], op=OP_WAIT, pol=0, wait_src=WAIT_PIN, index=0):
        raise NecirError("the bit loop does not wait for its data burst to end")
    if not _is(words[bit + 3], op=OP_IN, dest=IN_PINS, index=1):
        raise NecirError(
            "the bit loop does not sample the level with `in pins, 1`, which is where the data "
            "space is measured"
        )
    if not (_is(words[bit + 4], op=OP_OUT, dest=OUT_NULL, index=1)
            and _is(words[bit + 5], op=OP_JMP, cond=0b111)
            and words[bit + 5]["index"] == bit):
        raise NecirError(
            "the bit loop does not count its bits with `out null, 1` and end on `jmp !osre bit`, "
            "which is what makes a frame exactly 32 data bits long"
        )
    if (words[bit + 6]["index"] if words[bit + 6]["op"] == OP_JMP else None) != bit + 6:
        pass
    done = program.labels["done"]
    if not _is(words[done], op=OP_MOV, dest=0b110, src=0b110, op_field=1):
        raise NecirError("a finished frame is not un-inverted by `mov isr, isr !`")
    push = words[done + 1]
    if not _is(push, op=OP_SHIFT) or (push["cond"] & 0b100):
        raise NecirError(
            "a finished frame is not pushed to the RX FIFO by a blocking `push` (bit 7 of the word "
            "must be 0, PUSH, and bit 5 set, Block)"
        )
    frame = program.labels["frame"]
    if not _is(words[frame + 4], op=OP_MOV, dest=0b110, src=0b011):
        raise NecirError("a frame does not clear the ISR (`mov isr, null`) before it starts")
    if not _is(words[frame + 5], op=OP_MOV, dest=0b111, src=0b011):
        raise NecirError("a frame does not clear the bit counter (`mov osr, null`)")
    if not (_is(by_label["rep"], op=OP_WAIT, pol=0, wait_src=WAIT_PIN, index=0)
            and _is(words[last], op=OP_JMP, cond=0)
            and words[last]["index"] == program.labels["leader"]):
        raise NecirError(
            "a repeat code does not go straight back to hunting: a repeat repeats the frame "
            "before it, so nothing may be shifted in or pushed"
        )
    for index, f in enumerate(words):
        if f["op"] == OP_OUT and f["dest"] in (0, 4):
            raise NecirError(f"word {index} writes a pin (`out pins/pindirs`); a receiver must "
                             f"only ever read the net it listens to")
        if f["op"] == OP_SET and f["dest"] in (0, 4):
            raise NecirError(f"word {index} writes a pin (`set pins/pindirs`); a receiver must "
                             f"only ever read the net it listens to")


# ---------------------------------------------------------------------------------------------
# Decoding what the engine pushed
# ---------------------------------------------------------------------------------------------
def decode_frame(raw: int) -> dict:
    """The four words of one NEC frame, and whether the two inverse pairs check out.

    Deliberately written here rather than imported from `verif.vip.necir`, so that the test can
    compare this firmware-side decode with the reference model's `decode_frame()` and the two
    agreeing is a result.  `extended` selects which way round word 1 is judged: False, the
    standard form, where word 1 is the inverse of word 0; True, the extended form, where word 1
    is a register byte and must *not* be the inverse (sbprojects, "Extended NEC protocol").
    """
    raw = int(raw) & 0xFFFF_FFFF
    words = [(raw >> (8 * i)) & 0xFF for i in range(4)]
    address_inverse_ok = words[1] == (~words[0] & 0xFF)
    return {
        "raw": raw,
        "words": words,
        "address8": words[0],
        "address16": (words[1] << 8) | words[0],
        "command": words[2],
        "address_inverse_ok": address_inverse_ok,
        "command_inverse_ok": words[3] == (~words[2] & 0xFF),
        "extended_ok": not address_inverse_ok,
    }


# ---------------------------------------------------------------------------------------------
# The nets, and the receiver itself
# ---------------------------------------------------------------------------------------------
#: A demodulated IR net idles at 0 -- no carrier -- so it is added with `pullup = 0`.  That is
#: `verif.vip.necir.IDLE_LEVEL`, and it is the opposite of `verif.vip.core.Bus`'s default of 1:
#: an unwired demodulated line has to read idle or every model on it sees a permanent burst.
IDLE_LEVEL = 0


def add_protocol_nets(bus: Bus) -> None:
    """Add `p0`..`p17` if they are not there, numbered as docs/mvp-isa.md `## Pins` numbers them."""
    for k in range(NUM_PINS):
        if f"p{k}" not in bus.nets:
            bus.add_net(f"p{k}")


def attach_ir_net(bus: Bus, pin: int) -> str:
    """Make `p{pin}` the demodulated IR net: push-pull, idling low.  Returns its name.

    This is what `verif.vip.necir._ensure_net` insists on, done here so the receiver and the VIP
    cannot disagree about it: a net pulled high would read as a permanent burst.
    """
    if not 0 <= pin < NUM_PINS:
        raise ValueError(f"pin {pin} is outside 0..{NUM_PINS - 1}")
    add_protocol_nets(bus)
    name = f"p{pin}"
    bus.nets[name].push_pull = True
    bus.nets[name].pullup = IDLE_LEVEL
    bus.values[name] = IDLE_LEVEL
    return name


@dataclass
class NecirWord:
    """One RX FIFO word: the 32 data bits of one NEC frame, as the chip pushed them."""

    raw: int
    tick: int                      # bus tick the host read the word in
    info: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.info:
            self.info = decode_frame(self.raw)

    @property
    def address8(self) -> int:
        return self.info["address8"]

    @property
    def address16(self) -> int:
        return self.info["address16"]

    @property
    def command(self) -> int:
        return self.info["command"]

    @property
    def ok(self) -> bool:
        """True when both inverse words check out: a legal standard-form frame."""
        return self.info["address_inverse_ok"] and self.info["command_inverse_ok"]

    def __str__(self) -> str:
        return (f"t={self.tick} 0x{self.raw:08x} = address 0x{self.address8:02x}, command "
                f"0x{self.command:02x}")


class NecirReceiver:
    """The NEC receiver on one engine of an `iss.pio_iss.Chip` on a `verif.vip.core.Bus`.

    The constructor loads `program.pio`, writes the four per-engine registers and starts the
    engine, so a test only has to attach the model's `NecirTx` to the same bus and run it.  `bus`
    may be given (with `dev` a `ChipDevice` already attached to it, which is how a test wires the
    chip in among other devices); otherwise a bus with `p0`..`p17` is made here.

    `sm_cycles_per_unit` (S) is the only thing that sets the speed: NEC's own rate is fixed, and
    S is how finely the engine's SM cycle divides the 562.5 us burst.  See program.pio's header
    for why it cannot be 1 or 2, and `sample_window()` for the margin each choice leaves.

    `auto_read` drains the RX FIFO as the bus runs: the engine's `push` side is the only thing
    that puts words there, and the FIFO is four deep, so a host that does not read loses frames.
    The drain goes through `FLEVEL` and only then `RXF`, because `rx_valid` (IRQ0) is a
    registered line and would say "there is a word" for a couple of core cycles after the last
    one (docs/mvp-regmap.md `### tx_ready and rx_valid`), and reading `RXF` on it alone would
    pop nothing and set `FDEBUG.RXUNDER`.
    """

    def __init__(self, bus: Optional[Bus] = None, chip: Optional[Chip] = None,
                 dev: Optional[ChipDevice] = None, *, engine: int = 0, pin: int = 0,
                 sm_cycles_per_unit: int = DEFAULT_SM_CYCLES_PER_UNIT, tolerance: float = 0.25,
                 unit_ns: float = NEC_UNIT_NS, core_hz: float = CORE_HZ,
                 ns_per_tick: float = NS_PER_TICK,
                 program: Optional[Program] = None, auto_read: bool = True,
                 auto_start: bool = True) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        if pin not in RX_PINS:
            raise ValueError(
                f"IR pin {pin} has no input path: 0..12 do, 13..17 are output only "
                f"(docs/mvp-isa.md `## Pins`)"
            )
        self.engine, self.pin = engine, pin
        self.core_hz, self.ns_per_tick = float(core_hz), float(ns_per_tick)
        self.unit_ns, self.tolerance = float(unit_ns), float(tolerance)
        self.program = program if program is not None else build_program(
            sm_cycles_per_unit, tolerance, unit_ns, ns_per_tick, core_hz)
        else_checked = check_program(self.program, sm_cycles_per_unit, tolerance, unit_ns,
                                     ns_per_tick, core_hz)
        self.sm_cycles_per_unit = int(sm_cycles_per_unit)
        self.unit_cycles = self.core_hz * self.unit_ns / 1e9
        self.cycles_per_sm_cycle = self.unit_cycles / self.sm_cycles_per_unit
        if self.cycles_per_sm_cycle < 1.0:
            # CLKDIV cannot go below 1.0 core cycle (docs/mvp-isa.md `### Clock divider`), so one
            # SM cycle is at least one core cycle and the burst has to be at least S core cycles
            # long.  This is what rules out a slow bench: at 5 MHz the model's 562.5 ns burst is
            # 2.8 core cycles and the sample windows are empty long before S = 7.
            raise ValueError(
                f"at {self.core_hz:.4g} Hz a {self.unit_ns:g} ns burst is {self.unit_cycles:.4g} "
                f"core cycles, and this program needs at least {self.sm_cycles_per_unit} of them "
                f"(S = {self.sm_cycles_per_unit}, CLKDIV >= 1.0): the chip has to be clocked at "
                f"{self.sm_cycles_per_unit / self.unit_ns * 1e9:.4g} Hz or faster for this "
                f"protocol"
            )
        self.clkdiv = clkdiv_value(self.cycles_per_sm_cycle)
        self.words: List[NecirWord] = []
        self.auto_read = bool(auto_read)
        self._reads: List[Tuple[object, bool]] = []
        if bus is None:
            self.bus = Bus(ns_per_tick=self.ns_per_tick)
            add_protocol_nets(self.bus)
            self.dev = self.bus.attach(ChipDevice(chip, name="chip", prefix="p"))
        else:
            self.bus = bus
            if dev is None:
                raise ValueError("a bus was given, so give the ChipDevice attached to it too")
            self.dev = dev
        self.chip = self.dev.chip
        # A ChipDevice steps the chip one core cycle per bus tick, so the tick rate *is* the core
        # clock: the bus's, not the argument's.  Taking them as independent would silently run the
        # whole design ten times slow on a 200 ns bus and every timing below would be wrong by
        # that factor.
        self.ns_per_tick = self.bus.ns_per_tick
        if abs(1e9 / self.ns_per_tick - core_hz) > 1e-6 * core_hz:
            raise ValueError(
                f"the bus ticks every {self.ns_per_tick} ns, i.e. {1e9 / self.ns_per_tick:.4g} Hz, "
                f"but core_hz says {core_hz:.4g} Hz: an iss.pio_iss.ChipDevice steps one core "
                f"cycle per bus tick, so the two have to agree (ns_per_tick = 1e9 / core_hz)"
            )
        check_program(self.program, sm_cycles_per_unit, tolerance, unit_ns, self.ns_per_tick,
                      core_hz)
        attach_ir_net(self.bus, pin)
        if auto_start:
            self.start()

    # ---- what the program needs ---------------------------------------------------------------
    @property
    def net(self) -> str:
        """Name of the net the IR pin is on."""
        return f"p{self.pin}"

    @property
    def start_pc(self) -> int:
        return self.program.labels["start"]

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program, jmp_pin=self.pin)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value()

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(self.pin, sideset_count=self.program.sideset_bits)

    def jmp_abs(self, addr: int) -> int:
        """JMP with the `always` condition to an absolute address, for `SMe_INSTR`."""
        if not 0 <= addr <= 31:
            raise ValueError(f"JMP address {addr} does not fit in 5 bits")
        return addr                                  # OP_JMP = 0, condition always = 0

    # ---- host accesses: queued on the ChipDevice, one per tick --------------------------------
    def write(self, addr: int, data: int) -> None:
        """Queue one host write (it takes effect at the end of its access cycle `h`)."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def read(self, addr: int) -> int:
        """Queue one host read and run the bus until its data comes back."""
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.bus.step()
            ticks += 1
            if ticks > 10_000_000:
                raise NecirError(f"host read of {addr:#04x} unanswered after {ticks} ticks")
        return int(future.result())

    def _settle(self, extra: int = 2) -> None:
        while not self.dev.idle:
            self.bus.step()
        for _ in range(extra):
            self.bus.step()

    def fstat(self) -> int:
        return self.read(FSTAT)

    def flevel(self) -> int:
        return self.read(FLEVEL)

    def fdebug(self) -> int:
        return self.read(FDEBUG)

    def rx_level(self, flevel: Optional[int] = None) -> int:
        """The RX FIFO occupancy, bits 7:4 for engine 0 and 15:12 for engine 1."""
        value = self.flevel() if flevel is None else flevel
        return (value >> (8 * self.engine + 4)) & 0xF

    def padout(self) -> int:
        return self.read(DBG_PADOUT)

    def padoe(self) -> int:
        return self.read(DBG_PADOE)

    def cfginfo(self) -> int:
        return self.read(DBG_CFGINFO)

    def pc(self) -> int:
        return self.read(sm_addr(self.engine, "ADDR"))

    # ---- the start sequence (docs/mvp-regmap.md `### Typical host sequences`) ------------------
    def load(self) -> None:
        """Steps 1-3: stop the engine, burst-write the program, set the four registers."""
        e = self.engine
        origin = self.program.origin or 0
        self.write(CTRL, 0)                                   # the engine is stopped
        for k, word in enumerate(self.program.instrs):
            self.write(imem_addr(e, origin + k), word)
        self.write(sm_addr(e, "CLKDIV"), self.clkdiv)
        self.write(sm_addr(e, "EXECCTRL"), self.execctrl)
        self.write(sm_addr(e, "SHIFTCTRL"), self.shiftctrl)
        self.write(sm_addr(e, "PINCTRL"), self.pinctrl)
        self._settle()

    def arm(self) -> None:
        """Step 5: a forced `jmp start`, which is the only way to set the PC.

        `SMe_ADDR` is read-only (docs/mvp-regmap.md `**SMe_ADDR**`), so this is what puts the
        engine on `leader`.  Nothing else is forced: the pin is an input from reset, because
        PINCTRL leaves its direction register alone.
        """
        self.write(sm_addr(self.engine, "INSTR"), self.jmp_abs(self.start_pc))
        self._settle(2)

    def enable(self) -> None:
        """Step 6: `SM_ENABLE` with `SM_RESTART` and `CLKDIV_RESTART` in the same `CTRL` write.

        The engine's first SM cycle is then two core cycles after this access cycle `h`
        (docs/mvp-isa.md `### Clock divider`), and both the PC and the divider phase are set.
        """
        mask = 1 << self.engine
        ctrl = (self.read(CTRL) & ~(CTRL_SM_ENABLE << self.engine)
                & ~(CTRL_SM_RESTART << self.engine) & ~(CTRL_CLKDIV_RESTART << self.engine))
        self.write(CTRL, ctrl | mask | (mask << 4) | (mask << 8))
        self._settle()

    def start(self, load: bool = True) -> None:
        """Load, arm, enable -- the whole of `### Typical host sequences`, in order.

        `load=False` skips the IMEM burst, for a caller that has written the words itself (the
        mutation tests do exactly that: a bad load of a good program).

        Then run one SM cycle's worth of bus, because the engine's first SM cycle is a whole
        CLKDIV period away (docs/mvp-isa.md `### Clock divider`) and a frame that arrives before
        it is simply missed: the engine cannot see a 9 ms leader mark that has already ended.
        Without this a caller that queues a press straight after constructing the receiver would
        lose the first frame, which is a property of the test bench rather than of the program.
        """
        if load:
            self.load()
        self.arm()
        self.enable()
        self.run(int(math.ceil(self.cycles_per_sm_cycle)) + 8)

    def stop(self) -> None:
        """Disable the engine, keeping every register (docs/mvp-isa.md `### Enable, restart`)."""
        ctrl = self.read(CTRL)
        self.write(CTRL, ctrl & ~(CTRL_SM_ENABLE << self.engine))
        self._settle()

    # ---- running ------------------------------------------------------------------------------
    def step(self) -> None:
        """One core cycle: start a FIFO read if one is due, tick the bus, then collect.

        `rx_valid` (IRQ0) is a registered line, so it stays high for a couple of core cycles
        after the FIFO empties: the drain therefore reads `FLEVEL` first and pops `RXF` only when
        the level says there is a word, which is what keeps `FDEBUG.RXUNDER` clear.
        """
        if self.auto_read and not self._reads and self.chip.irq0:
            self._reads.append((self.dev.read(FLEVEL), False))
        self.bus.step()
        while self._reads and self._reads[0][0].done():
            future, is_word = self._reads.pop(0)
            value = future.result()
            if is_word:
                self.words.append(NecirWord(raw=value, tick=self.bus.t))
            elif self.rx_level(value):
                self._reads.append((self.dev.read(RXF[self.engine]), True))

    def run(self, ticks: int) -> None:
        """Step the bus `ticks` core cycles, draining the RX FIFO as it goes."""
        for _ in range(ticks):
            self.step()

    def run_until(self, cond: Callable[[], bool], limit: int) -> bool:
        """Step until `cond()` or `limit` ticks; True if `cond()` ended it."""
        for _ in range(limit):
            if cond():
                return True
            self.step()
        return bool(cond())

    def drain(self, ticks: int = 8) -> None:
        """Step `ticks` more cycles so a word the engine pushed just before the bus stopped is
        read out of the FIFO and not left there for the next run."""
        self.run(ticks)

    # ---- what came out ------------------------------------------------------------------------
    @property
    def raws(self) -> List[int]:
        """Every word the engine pushed, in order."""
        return [w.raw for w in self.words]

    @property
    def payloads(self) -> List[Tuple[int, int]]:
        """(address, command) of every word, in order, decoded as the standard NEC form."""
        return [(w.address8, w.command) for w in self.words]

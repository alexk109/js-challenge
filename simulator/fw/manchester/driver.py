"""Host side of the MVP Manchester firmware: load program.pio, run one engine, push bytes.

Everything here is written against `docs/mvp-regmap.md` (register word
addresses, the `### Typical host sequences` startup order, the per-engine fields) and
`docs/mvp-isa.md` (divider, delay/side-set split, autopull).  Host accesses go through
`iss.pio_iss.ChipDevice`'s write/read queue, one per tick, so every wait here is the
simulation running -- the same thing a real host does over the SPI port of
`docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                     # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())
    drv = ManchesterTxDriver(dev, bus, pin=0, bit_rate=10e6)
    drv.load()                 # IMEM, CLKDIV, EXECCTRL, SHIFTCTRL, PINCTRL
    drv.start()                # forced `set pins/pindirs`, `jmp start`, CTRL restart+enable
    drv.send(b"hello")         # one byte per TX word, blocks while the FIFO is full
    bus.run(20_000)
    drv.stop()

**The chip is the transmitter** (the chip plays the side the model's devices
do not).  `verif/vip/manchester.py` has a transmitter, a
receiver and a passive monitor; this driver plus `program.pio` is the chip's transmitter, and
the model's `ManchesterRx` and `ManchesterTimingChecker` are the judge in
`fw/manchester/test_manchester.py`.  The chip as *receiver* is a different program and a
different program: a receiver has to tell a half-bit interval from a whole-bit one, which with
no IRQ means counting in a loop against a pin that arrives three cycles late
(`docs/mvp-isa.md` `### Timing`), and 32 words is not enough to do it and still frame
anything.  See `fw/manchester/README.md`.

The chip core clock is 50 MHz: one `verif.vip.core.Bus` tick at `ns_per_tick = 20`, the
configuration the Manchester VIP documents in its own docstring.

Four things about this chip that the driver has to get right and that are worth stating once:

* **The bit rate is a divider, not a loop.** `program.pio` spends five SM cycles per bit (see
  its header comment), so a bit rate is `5 * (INT + FRAC/256)` core cycles and
  `clkdiv_for_bit_rate` divides `CORE_HZ` by it.  `10 Mbit/s` is `CLKDIV = 1.0` and is also
  the fastest this program can express: a bit cannot be shorter than three SM cycles, and
  four SM cycles per bit puts a bit-boundary transition and the next mid-bit transition one
  tick apart, which is shorter than the half bit the line code allows.
* **`PINCTRL.SIDESET_COUNT` must be 0.** The program has no `.side_set`, so all five bits of
  bits 12:8 are delay.  The *reset* value of `SIDESET_COUNT` is 2 and `EXECCTRL.SIDE_EN`
  resets to 0, and that combination makes every instruction side-set level 0 into
  `SIDESET_BASE` and `SIDESET_BASE + 1`; the side-set channel beats OUT/SET on the same pin
  (`docs/mvp-isa.md` `### Pin writes`), so the line would sit low forever.  `pinctrl_value`
  takes the count from the assembler (`program.sideset_bits`) for the same reason
  `fw/uart_tx/driver.py` does.
* **The level register comes out of reset at 0 and the idle level is 1.**  So the pin must be
  told the idle level *before* it is made an output, in that order: driving it first would
  put a one-tick low pulse on the line, which the monitor reads as a runt edge.  Both
  happen in `arm()`, by forcing `set pins, 1` and `set pindirs, 1` through `SMe_INSTR` while
  the engine is still stopped (step 4 of the documented startup order), and the program
  repeats them so it cannot be started wrong.
* **The convention is a knob, and it is a bit-order knob.**  `ieee802.3` (the default, and
  the one AN1120a states) encodes 0 as high-to-low; G.E. Thomas encodes it the other way
  round and the two are inverses.  `program.pio` emits IEEE 802.3 chips.  Its *sync preamble*
  is convention-independent (both conventions' preambles drive the same two chip levels in
  the same two slots), so switching to Thomas is exactly "push the complement of each byte":
  `ManchesterTxDriver(..., convention="thomas")` does that in `push`, and nothing else
  changes.  Freescale AN3015 uses the Thomas convention and never says so, which is the trap
  the VIP's docstring warns about; `research/ref/protocols/manchester/README.md` has the
  citation table.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

from asm.pioasm import Program, assemble

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "manchester"     # the default .program name inside program.pio

#: The `.program` names in program.pio, in the order the file declares them.  The first is
#: the 10BASE-T one (five SM cycles per bit, three and two chips), the second has equal chips
#: (four SM cycles per bit) and is exact at every bit rate; program.pio's header says why.
PROGRAMS = ("manchester", "manchester_even")

#: The two conventions that exist and the only two spellings accepted, as in
#: `verif.vip/manchester.py`: they differ by line inversion, so picking the wrong one puts
#: every bit on the wire the wrong way round.
CONVENTIONS = ("ieee802.3", "thomas")

#: The line rests at this level between messages. The profile's default is high (AN1120a's
#: 10BASE-T link pulses are high), and `program.pio` hard-codes that: the sync preamble is
#: built from `set pins, 0` then `set pins, 1`, so a link that idles low would need the two
#: words swapped.
IDLE_LEVEL = 1

#: One `verif.vip.core.Bus` tick in nanoseconds: 20 ns is the 50 MHz chip clock these tests
#: run at, and the tick is the unit every timing number here is in.
TICK_NS = 20.0

#: A receiver's own frequency tolerance, AN2358: "Selecting an x16 oversampling rate gives
#: more than 10 percent frequency tolerance on each side and is the retained value for the
#: design" (`research/ref/protocols/manchester/README.md`, citation table).  Used only to say
#: whether a chip split is inside what a receiver can still classify.
INTERVAL_TOLERANCE = 0.10

#: The two bits of the sync pattern, as the wire carries them (first bit first). `ieee802.3`
#: with the idle line high, i.e. `sync_preamble("ieee802.3", 1)` in the VIP. The program
#: generates them itself; the driver only knows them so a test can check what the judge saw.
SYNC_PREAMBLE = {"ieee802.3": (0, 1), "thomas": (1, 0)}

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

# Instruction encodings the host forces through SMe_INSTR (docs/mvp-isa.md `## Encoding`).
OP_JMP, OP_SET = 0b000, 0b111
JMP_ALWAYS = 0b000
SET_DEST_PINS, SET_DEST_PINDIRS = 0b000, 0b100

#: SM cycles one message loop iteration costs at `CLKDIV = 1.0`, before the divider. It is
#: derived from the assembled words by `sm_cycles_per_bit` and friends, never assumed; this
#: is only the value the program is expected to come out at, so a change to `program.pio` that
#: is not visible in the tests is a bug.
EXPECTED_SM_CYCLES_PER_BIT = 5


def check_convention(name: str) -> str:
    """Return the canonical spelling of a convention name, or raise."""
    text = str(name).strip().lower()
    if text not in CONVENTIONS:
        raise ValueError(
            f"Manchester convention must be one of {CONVENTIONS} (got {name!r}); the two "
            "differ by line inversion, so picking the wrong one puts every bit the wrong way "
            "round on the wire"
        )
    return text


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def clkdiv_for_bit_rate(bit_rate: float, sm_cycles_per_bit: int) -> int:
    """The CLKDIV word that makes one bit last `sm_cycles_per_bit` SM cycles at `bit_rate`.

    One bit must last `CORE_HZ / bit_rate` core cycles and one SM cycle lasts one CLKDIV
    period, so the divisor wanted is `CORE_HZ / (bit_rate * sm_cycles_per_bit)`, rounded into
    INT and FRAC (`docs/mvp-isa.md` `### Clock divider`).  The divisor is exact on average,
    so the transmitter keeps the nominal bit grid to within one core cycle per SM cycle.

    A rate the program cannot reach -- one that needs a divisor below 1.0 -- raises rather
    than being silently clamped, because a clamp would put the wire at a rate nobody asked
    for.
    """
    if bit_rate <= 0:
        raise ValueError(f"bit_rate must be positive (got {bit_rate})")
    if sm_cycles_per_bit < 1:
        raise ValueError(f"a bit needs at least one SM cycle (got {sm_cycles_per_bit})")
    want = CORE_HZ / (bit_rate * sm_cycles_per_bit)
    if want < 1.0:
        raise ValueError(
            f"{bit_rate:g} bit/s needs {sm_cycles_per_bit} SM cycles per bit and so a CLKDIV "
            f"of {want:.3f}: one SM cycle is at least one core cycle, so the fastest this "
            f"program can send is {CORE_HZ / sm_cycles_per_bit:g} bit/s"
        )
    if want > 0xFFFF:
        raise ValueError(
            f"{bit_rate:g} bit/s needs a CLKDIV INT of {want:.1f}, above the 16-bit maximum"
        )
    integer = max(1, int(want))
    frac = round((want - integer) * 256)
    if frac > 0xFF:                       # rounding pushed it past the next integer
        integer, frac = integer + 1, 0
    return clkdiv_value(integer, frac)


def sm_cycles_to_ticks(sm_cycles: int, clkdiv: int) -> int:
    """How many core cycles `sm_cycles` SM cycles really take, exactly.

    The divider alternates `INT` and `INT + 1` core cycles when `FRAC != 0`
    (`docs/mvp-isa.md` `### Clock divider`), so over `n` SM cycles the total is
    `n * INT + floor(n * FRAC / 256)`.  This is what a test needs in order to say how long a
    message takes without guessing.
    """
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    frac = (clkdiv >> 8) & 0xFF
    return sm_cycles * integer + (sm_cycles * frac) // 256


def ticks_per_bit(clkdiv: int, sm_cycles_per_bit: int) -> float:
    """The average bit length in core cycles (ticks) a CLKDIV word produces."""
    return sm_cycles_to_ticks(sm_cycles_per_bit, clkdiv)


def bit_rate_for_clkdiv(clkdiv: int, sm_cycles_per_bit: int) -> float:
    """The bit rate a CLKDIV word produces, in bits per second."""
    return CORE_HZ / ticks_per_bit(clkdiv, sm_cycles_per_bit)


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address (`docs/mvp-isa.md` `## Encoding`)."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (OP_JMP << 13) | (JMP_ALWAYS << 5) | addr


def set_pins(level: int) -> int:
    """`set pins, level`, forced by the host to put the idle level on the TX pin."""
    if level not in (0, 1):
        raise ValueError(f"a pin level is 0 or 1 (got {level!r})")
    return (OP_SET << 13) | (SET_DEST_PINS << 5) | level


def set_pindirs(value: int) -> int:
    """`set pindirs, value`, forced by the host to make the TX pin an output."""
    if value not in (0, 1):
        raise ValueError(f"a pin direction is 0 or 1 (got {value!r})")
    return (OP_SET << 13) | (SET_DEST_PINDIRS << 5) | value


# --------------------------------------------------------------------------------------------
# The program: assemble it, and read back what the words say about the timing
# --------------------------------------------------------------------------------------------

def load_program(name: str = PROTO_NAME, path: Path | str = PROGRAM_PATH) -> Program:
    """Assemble program.pio (asm/pioasm.py) and return the `.program` called `name`.

    The file holds more than one program, one per chip split, so the name is a parameter and
    a name that is not in it is an error naming the ones that are.
    """
    programs = assemble(Path(path).read_text(encoding="utf-8"))
    for program in programs:
        if program.name == name:
            return program
    raise ValueError(
        f"{path}: no .program named {name!r}; it has {[p.name for p in programs]}"
    )


def decode_word(program: Program, word: int) -> dict:
    """Split one word into the fields `docs/mvp-isa.md` `## Encoding` defines.

    Only what the driver needs: the opcode, the side-set bits and the delay, which are the
    fields whose split depends on PINCTRL.SIDESET_COUNT rather than on the instruction. `sc`
    is the count including the `opt` enable bit, so the delay field is the low `5 - sc` bits,
    as the spec says.
    """
    sc = min(program.sideset_bits, 5)
    field = (word >> 8) & 0x1F
    return {
        "op": word >> 13,
        "sideset": field >> (5 - sc) if sc else 0,
        "delay": field & ((1 << (5 - sc)) - 1),
        "low": word & 0xFF,
    }


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (`docs/mvp-isa.md` `### Engine state`)."""
    return len(program.instrs)


def is_mov_pins_inverted_osr(program: Program, word: int) -> bool:
    """True for `mov pins, osr !`: the first half of a Manchester cell."""
    low = decode_word(program, word)["low"]
    return decode_word(program, word)["op"] == 0b101 and (low >> 5) == 0b000 \
        and ((low >> 3) & 3) == 0b01 and (low & 7) == 0b111


def is_out_pins_1(program: Program, word: int) -> bool:
    """True for `out pins, 1`: the second half of a Manchester cell."""
    fields = decode_word(program, word)
    return fields["op"] == 0b011 and (fields["low"] >> 5) == 0b000 and fields["low"] & 0x1F == 1


def bit_loop(program: Program) -> tuple[int, int, int]:
    """The address of the data-bit loop and the two chip instructions in it.

    The driver has to know how long a bit lasts, and it must read that out of the assembled
    words rather than trust a comment: `mov pins, osr !`, `out pins, 1` and the `jmp` that
    closes the loop.  Anything else -- a second such pair, a loop the `jmp` does not close, a
    loop that is not three instructions long -- raises instead of producing a divisor that
    would mean something else.
    """
    starts = [i for i, w in enumerate(program.instrs) if is_mov_pins_inverted_osr(program, w)]
    if len(starts) != 1:
        raise ValueError(
            f"{PROTO_NAME}: expected exactly one `mov pins, osr !` (the first half of a cell),"
            f" found {len(starts)}; the bit rate cannot be derived from this program"
        )
    first = starts[0]
    if first + 2 >= len(program.instrs):
        raise ValueError(f"{PROTO_NAME}: the bit loop at {first} is not three instructions long")
    second, jmp = program.instrs[first + 1], program.instrs[first + 2]
    if not is_out_pins_1(program, second):
        raise ValueError(
            f"{PROTO_NAME}: the instruction after `mov pins, osr !` is {second:#06x}, not "
            "`out pins, 1`; the bit rate cannot be derived from this program"
        )
    if decode_word(program, jmp)["op"] != OP_JMP or (jmp & 0x1F) != first:
        raise ValueError(
            f"{PROTO_NAME}: the instruction after `out pins, 1` is {jmp:#06x}, not a `jmp` back "
            f"to the loop at {first}; the bit rate cannot be derived from this program"
        )
    return first, first + 1, first + 2


def sm_cycles_per_bit(program: Program) -> int:
    """SM cycles one data bit lasts, taken from the assembled words.

    Three instructions: `mov pins, osr ! [d]` takes `1 + d` SM cycles and holds the first
    chip, `out pins, 1 [d]` takes `1 + d` and writes the second chip, and the `jmp` takes one
    more SM cycle during which the second chip's level holds (`docs/mvp-isa.md`
    `### Cycle accounting`).  The bit is their sum.
    """
    first, second, third = bit_loop(program)
    delays = [decode_word(program, program.instrs[i])["delay"] for i in (first, second, third)]
    return sum(1 + d for d in delays)


def bits_per_word(program: Program) -> int:
    """How many data bits one TX word carries: the `set x, n` count plus one.

    The loop body sends one bit per pass and `jmp x--` runs one more pass than the counter
    says, so `set x, 0` is one bit and `set x, 7` is eight.
    """
    _, _, jmp_index = bit_loop(program)
    counts = []
    for word in program.instrs[:jmp_index]:
        fields = decode_word(program, word)
        if fields["op"] == OP_SET and (fields["low"] >> 5) == 0b001:      # SET X
            counts.append(fields["low"] & 0x1F)
    if len(counts) != 1:
        raise ValueError(
            f"{PROTO_NAME}: expected exactly one `set x, n` before the bit loop, found "
            f"{len(counts)}; the word size cannot be derived from this program"
        )
    if counts[0] > 31:
        raise ValueError(f"{PROTO_NAME}: `set x, {counts[0]}` would need 32 passes, and the "
                         "loop's `jmp x--` wraps X to all ones on the 32nd")
    return counts[0] + 1


def message_sm_cycles(program: Program) -> int:
    """SM cycles one message costs: the loop body once, plus its other passes.

    `program.pio` is one message per pass of `message`: every instruction outside the bit
    loop runs once and every instruction inside it runs `bits_per_word` times.  An
    instruction retires in `1 + delay` SM cycles (`docs/mvp-isa.md` `### Cycle accounting`),
    so this is the sum over the words plus the extra loop passes.  Used by the tests to know
    how long to run the bus, and to say what one byte costs.
    """
    first, _, _ = bit_loop(program)
    once = sum(1 + decode_word(program, w)["delay"] for w in program.instrs)
    return once + (bits_per_word(program) - 1) * sm_cycles_per_bit(program)


def chip_lengths_sm_cycles(program: Program) -> tuple[int, int]:
    """SM cycles the first and the second chip of a cell last.

    The first chip is written by `mov pins, osr !` and lasts `1 + delay` cycles; the second is
    written by `out pins, 1` and lasts `1 + delay` cycles *plus* the loop's `jmp`, which
    holds the line at the second chip's level while it runs.  A cell's two chips should be
    equal -- that is what "half a bit each" means -- and the difference between them is the
    whole of a program's rate range; see `split_error_ticks`.
    """
    first, second, _ = bit_loop(program)
    return (1 + decode_word(program, program.instrs[first])["delay"],
            1 + decode_word(program, program.instrs[second])["delay"] + 1)


def chip_lengths_ticks(program: Program, clkdiv: int) -> tuple[float, float]:
    """The two chip lengths in core cycles (ticks), averaged over the divider's alternation."""
    first, second = chip_lengths_sm_cycles(program)
    return _sm_cycles_to_ticks_float(first, clkdiv), _sm_cycles_to_ticks_float(second, clkdiv)


def _sm_cycles_to_ticks_float(sm_cycles: int, clkdiv: int) -> float:
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    frac = (clkdiv >> 8) & 0xFF
    return sm_cycles * (integer + frac / 256.0)


def chip_length_ranges(program: Program, clkdiv: int) -> tuple[tuple[int, int], tuple[int, int]]:
    """The whole-number tick lengths each chip can actually take at this CLKDIV.

    This is the exact answer, and it is not the nominal one.  The divider alternates `INT`
    and `INT + 1` core cycles per SM cycle (`docs/mvp-isa.md` `### Clock divider`), so `n` SM
    cycles last `n * INT` ticks plus the number of fractional carries in them, and that
    number is either `floor(n * FRAC / 256)` or `ceil(n * FRAC / 256)` depending on where in
    the accumulator's period the chip starts.  So a chip is one of two lengths one tick apart,
    and which one it is depends on the data: with `CLKDIV = 1 + 154/256` the three-SM-cycle
    chip of the 10BASE-T program is 4 or 5 ticks.  A transmitter has to keep *both* inside a
    receiver's window, and `split_is_legal` is what checks that.
    """
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    frac = (clkdiv >> 8) & 0xFF
    out = []
    for n in chip_lengths_sm_cycles(program):
        carries = (n * frac) // 256
        out.append((n * integer + carries, n * integer + carries + (1 if n * frac % 256 else 0)))
    return out[0], out[1]


def half_bit_ticks(clkdiv: int, sm_cycles_per_bit: int) -> float:
    """Half a bit time in core cycles."""
    return _sm_cycles_to_ticks_float(sm_cycles_per_bit, clkdiv) / 2.0


def half_bit_tolerance_ticks(clkdiv: int, sm_cycles_per_bit: int,
                             interval_tol: float = INTERVAL_TOLERANCE,
                             ns_per_tick: float = TICK_NS) -> float:
    """How far a measured interval may be from a half bit and still be a bit-boundary
    transition.

    Two things set it, and both are real:

    * `interval_tol` is AN2358's own figure for a receiver that oversamples x16: "Selecting
      an x16 oversampling rate gives more than 10 percent frequency tolerance on each side
      and is the retained value for the design"
      (`research/ref/protocols/manchester/README.md`, citation table).
    * one tick, because `verif/vip/core.py` can only place an edge on a tick boundary, so a
      half bit that is not a whole number of ticks cannot be put anywhere more precisely.

    The larger of the two wins, which is what a receiver has to assume.
    """
    return max(1.0, interval_tol * half_bit_ticks(clkdiv, sm_cycles_per_bit))


def split_error_ticks(program: Program, clkdiv: int,
                      interval_tol: float = INTERVAL_TOLERANCE) -> float:
    """How far the program's longer chip is from a half bit, in core cycles.

    Zero when the two chips are equal.  This is the number that decides a program's whole
    rate range: while it is inside `half_bit_tolerance_ticks` the receiver can still classify
    the transitions, and once it is outside, the wire is not Manchester any more.
    """
    first, second = chip_lengths_ticks(program, clkdiv)
    half = half_bit_ticks(clkdiv, sm_cycles_per_bit(program))
    return max(abs(first - half), abs(second - half))


def split_is_legal(program: Program, clkdiv: int, *,
                   interval_tol: float = INTERVAL_TOLERANCE) -> bool:
    """True when every chip length the divider can produce is inside the half-bit window.

    `chip_length_ranges` gives the two lengths each chip can actually be, so this is the
    whole question and nothing is left to the tolerance of a floating-point comparison; the
    comparison allows the window edge exactly, because a receiver's window is inclusive and
    `verif/vip/manchester.py` judges with `<=` too.
    """
    half = half_bit_ticks(clkdiv, sm_cycles_per_bit(program))
    tol = half_bit_tolerance_ticks(clkdiv, sm_cycles_per_bit(program), interval_tol)
    return all(half - tol - 1e-9 <= low and high <= half + tol + 1e-9
               for low, high in chip_length_ranges(program, clkdiv))


def legal_bit_rates(program: Program, *, ns_per_tick: float = TICK_NS,
                    interval_tol: float = INTERVAL_TOLERANCE) -> tuple[float, float]:
    """The bit rates this program can carry at this tick rate, slowest and fastest.

    Derived from the split and nothing else, for a *whole-number* divider, so it is a bound
    rather than the answer: the chips are `n1` and `n2` SM cycles and a bit is `n1 + n2`, so
    the error from a half bit is `|n1 - n2| / 2` SM cycles whatever CLKDIV is, and the tick
    grid turns that into a tolerance of at least one tick.  For a program with unequal chips
    the slow end is where that skew reaches the one-tick floor; for equal chips no rate is out
    of reach and only the divider's own limit (CLKDIV = 1.0) bounds the fast end.

    `split_is_legal` is the exact per-rate answer, and it can refuse a rate inside this range:
    a *fractional* divider puts each chip on one of two lengths a tick apart, and the longer
    of the two can be outside the window even when the average is inside it.
    """
    n1, n2 = chip_lengths_sm_cycles(program)
    per_bit = n1 + n2
    skew = abs(n1 - n2) / 2.0                # SM cycles of error from a half bit
    core_hz = 1e9 / ns_per_tick
    fastest = core_hz / per_bit              # CLKDIV = 1.0, one core cycle per SM cycle
    if skew <= interval_tol * per_bit / 2.0:
        return (0.0, fastest)                # equal chips: no rate is out of reach
    # Otherwise the one-tick floor binds and sets the slow end: skew * CLKDIV <= 1 tick.
    return (core_hz * skew / per_bit, fastest)


def gap_sm_cycles(program: Program) -> int:
    """SM cycles the program holds the line idle between messages, and where it starts.

    The idle phase is the `set pins, 1 [d]` that follows the bit loop: it is the only
    instruction after the loop that is not a `jmp`, and it is what the monitor's minimum gap
    between messages is measured against (`verif/vip/manchester.py`, `tGAP`).
    """
    first, _, jmp_index = bit_loop(program)
    tail = [
        (i, w) for i, w in enumerate(program.instrs)
        if i > jmp_index and decode_word(program, w)["op"] == OP_SET
        and (w & 0x1F) == IDLE_LEVEL and (decode_word(program, w)["low"] >> 5) == SET_DEST_PINS
    ]
    if len(tail) != 1:
        raise ValueError(
            f"{PROTO_NAME}: expected one idle `set pins, {IDLE_LEVEL}` after the bit loop, "
            f"found {len(tail)}; the inter-message gap cannot be derived from this program"
        )
    index, word = tail[0]
    return 1 + decode_word(program, word)["delay"]

# --------------------------------------------------------------------------------------------
# Register values the program needs (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program, *, side_en: int = 0, wrap_bottom: int = 0) -> int:
    """EXECCTRL: SIDE_EN (0 for this program) plus the wrap window the assembler worked out.

    `SIDE_EN` has to be 0 because `program.pio` has no `.side_set`: with `SIDE_EN = 1` and
    `SIDESET_COUNT = 0` there is no side-set anyway, but writing 1 here would make
    `SIDESET_COUNT` mean something else the moment anyone gave the program a side-set bit.
    """
    if side_en and program.sideset_bits:
        raise ValueError(
            f"{PROTO_NAME}: the program has {program.sideset_bits} side-set bit(s) but "
            "`execctrl_value` was asked for SIDE_EN = 0"
        )
    return (side_en << 30) | (program.wrap << 12) | (wrap_bottom << 7)


def shiftctrl_value(*, pull_thresh: int = 8, out_right: int = 1, autopull: int = 1) -> int:
    """SHIFTCTRL: shift right (LSB out first), autopull every 8 bits.

    The host pushes each byte right-justified, in bits 7:0, and `OUT_SHIFTDIR = 1` (the
    reset value) makes `out pins, 1` send the least significant bit first, which is the order
    `verif.vip.manchester.bits_of(byte, "lsb")` decodes -- IEEE 802.3's choice.

    `PULL_THRESH = 8` is one message: the eighth `out pins, 1` brings the OSR count to the
    threshold and refills it from the TX FIFO in the same SM cycle
    (`docs/mvp-isa.md` `### Instruction semantics`, OUT), so the `pull block` at the top of
    the loop finds `osr_cnt == 0` and is a no-op barrier instead of stalling
    (`docs/mvp-isa.md` `### Instruction semantics`, PULL).
    """
    if not 1 <= pull_thresh <= 31:
        raise ValueError(f"PULL_THRESH {pull_thresh} out of range 1..31")
    return (pull_thresh << 25) | (out_right << 19) | (autopull << 17)


def pinctrl_value(pin: int, program: Program, *, set_count: int = 1, out_count: int = 1) -> int:
    """PINCTRL: every window of the program points at the TX pin, one pin wide.

    OUT carries the data chips (`mov pins, osr !` and `out pins, 1`), SET carries the sync
    preamble and the idle level, and the side-set is not used at all.  `SIDESET_COUNT` comes
    from the assembler (`program.sideset_bits`, which is 0 here) rather than being written
    out: the reset value 2 with the reset `SIDE_EN = 0` would make every instruction side-set
    level 0 into `SIDESET_BASE` and `SIDESET_BASE + 1`, and the side-set channel beats
    OUT/SET on the same pin (`docs/mvp-isa.md` `### Pin writes`), so the line would sit low
    for ever.  IN_BASE is unused by the program and is set to the TX pin for consistency.
    """
    if not 0 <= pin < NUM_PINS:
        raise ValueError(f"TX pin {pin} is outside 0..{NUM_PINS - 1}")
    if not all(0 <= b <= 32 for b in (set_count, out_count)):
        raise ValueError("window counts must be 0..32")
    sideset_count = min(program.sideset_bits, 5)
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (pin << 15)
            | (pin << 10) | (pin << 5) | pin)


@dataclass(frozen=True)
class Config:
    """One Manchester transmitter setup: which engine and pin, at which rate and convention."""

    bit_rate: float = 10e6
    engine: int = 0
    pin: int = 0
    convention: str = "ieee802.3"

    @property
    def nominal_ticks_per_bit(self) -> float:
        return CORE_HZ / self.bit_rate

    @property
    def preamble(self) -> tuple[int, ...]:
        """The two sync bits this link puts at the head of every message, first bit first."""
        return SYNC_PREAMBLE[self.convention]


class ManchesterTxDriver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as a Manchester transmitter.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are queued
    on `dev` and land one per tick as the simulation runs, so `read()` and `send()` pump the
    bus while they wait.

    `convention` picks which of the two Manchester conventions the *wire* uses.  The program
    emits IEEE 802.3 chips either way; for `thomas` the host pushes the complement of each
    byte, which is the whole of the difference (`program.pio`'s header comment, and
    `research/ref/protocols/manchester/README.md` `## The convention, and the trap in it`).
    """

    MAX_TICKS = 2_000_000

    @property
    def TXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)

    def __init__(self, dev, bus, *, engine: int = 0, pin: int = 0, bit_rate: float = 10e6,
                 convention: str = "ieee802.3", program: Optional[Program] = None,
                 path: Path | str = PROGRAM_PATH, name: str = PROTO_NAME,
                 allow_illegal_split: bool = False) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        if pin not in UIO_PINS:
            raise ValueError(
                f"TX pin {pin} is not a uio pin {list(UIO_PINS)}: PINDIRS cannot drive "
                f"input-only or output-only protocol pins (docs/mvp-isa.md `## Pins`)"
            )
        self.dev, self.bus = dev, bus
        self.engine, self.pin = engine, pin
        self.program = program if program is not None else load_program(name, path)
        self.config = Config(bit_rate=bit_rate, engine=engine, pin=pin,
                             convention=check_convention(convention))
        self.sm_cycles_per_bit = sm_cycles_per_bit(self.program)
        self.bits_per_word = bits_per_word(self.program)
        if self.bits_per_word != shiftctrl_pull_thresh(self.program):
            raise ValueError(
                f"{PROTO_NAME}: the program sends {self.bits_per_word} bits per word but "
                f"SHIFTCTRL.PULL_THRESH would be {shiftctrl_pull_thresh(self.program)}"
            )
        self.allow_illegal_split = bool(allow_illegal_split)
        if not self.split_is_legal and not self.allow_illegal_split:
            slowest, fastest = legal_bit_rates(self.program)
            raise ValueError(
                f"{bit_rate:g} bit/s is not a rate the {self.program.name} program can carry: "
                f"its two chips are {chip_lengths_sm_cycles(self.program)[0]} and "
                f"{chip_lengths_sm_cycles(self.program)[1]} SM cycles, which at this CLKDIV is "
                f"{split_error_ticks(self.program, self.clkdiv):.2f} core cycles away from a "
                f"half bit -- outside the "
                f"{half_bit_tolerance_ticks(self.clkdiv, self.sm_cycles_per_bit):.2f}-tick "
                f"window a receiver can classify. Use the other program in program.pio "
                f"({[n for n in PROGRAMS if n != self.program.name]}), a rate in "
                f"[{slowest:g}, {fastest:g}] bit/s, or allow_illegal_split=True to send it "
                "anyway and watch the judge say so"
            )

    # ---- the chip split, which is what decides the rate range ----
    @property
    def chip_lengths_ticks(self) -> tuple[float, float]:
        """The two chips of a cell in core cycles, at this CLKDIV."""
        return chip_lengths_ticks(self.program, self.clkdiv)

    @property
    def split_error_ticks(self) -> float:
        """How far the worse chip is from a half bit, in core cycles."""
        return split_error_ticks(self.program, self.clkdiv)

    @property
    def split_is_legal(self) -> bool:
        """True when both chips are inside the receiver's half-bit window at this CLKDIV."""
        return split_is_legal(self.program, self.clkdiv)

    # ---- addresses ----
    @property
    def imem_base(self) -> int:
        return A_INSTR_MEM[self.engine]

    def sm_addr(self, index: int) -> int:
        return A_SM_BASE + 6 * self.engine + index

    @property
    def start_addr(self) -> int:
        return self.program.labels["start"]

    # ---- the register values this configuration asks for ----
    @property
    def bit_rate(self) -> float:
        return self.config.bit_rate

    @bit_rate.setter
    def bit_rate(self, value: float) -> None:
        self.config = Config(bit_rate=value, engine=self.engine, pin=self.pin,
                             convention=self.config.convention)

    @property
    def convention(self) -> str:
        return self.config.convention

    @convention.setter
    def convention(self, value: str) -> None:
        self.config = Config(bit_rate=self.bit_rate, engine=self.engine, pin=self.pin,
                             convention=check_convention(value))

    @property
    def clkdiv(self) -> int:
        return clkdiv_for_bit_rate(self.bit_rate, self.sm_cycles_per_bit)

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value(pull_thresh=shiftctrl_pull_thresh(self.program))

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(self.pin, self.program)

    @property
    def words_used(self) -> int:
        return words_used(self.program)

    @property
    def nominal_ticks_per_bit(self) -> float:
        return self.config.nominal_ticks_per_bit

    def message_ticks(self, count: int = 1) -> int:
        """Core cycles `count` messages take, exactly, from the divider algorithm."""
        return sm_cycles_to_ticks(message_sm_cycles(self.program) * count, self.clkdiv)

    # ---- host accesses: queued on the ChipDevice, one per tick ----
    def write(self, addr: int, data: int) -> None:
        """Queue one host write (`docs/mvp-regmap.md`: it takes effect at the end of its bus
        access cycle `h`)."""
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
        as this returns (`docs/mvp-regmap.md` `### Cycle-exact mapping`).
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
    def load(self) -> None:
        """Steps 1-3: stop the engine, burst-write the program, set the four registers.

        The engine is left disabled at whatever PC it had, so the caller can arm it and
        preload the TX FIFO before enabling (a disabled engine never drains it).
        """
        self.stop()
        for k, word in enumerate(self.program.instrs):
            self.write(self.imem_base + k, word)
        self.write(self.sm_addr(SM_CLKDIV), self.clkdiv)
        self.write(self.sm_addr(SM_EXECCTRL), self.execctrl)
        self.write(self.sm_addr(SM_SHIFTCTRL), self.shiftctrl)
        self.write(self.sm_addr(SM_PINCTRL), self.pinctrl)
        self._settle()

    def arm(self) -> None:
        """Steps 4-5: the initial pins, then a forced `jmp start`.

        A host-forced instruction runs in the very next core cycle whatever the divider and
        `SM_ENABLE` say (`docs/mvp-isa.md` `### EXEC`), and it is the only way to set the PC.

        The order of the two pin instructions is not cosmetic.  The TX pin's level register
        comes out of reset at 0 while the idle level is 1, so `set pindirs, 1` on its own
        would drive the line low for one core cycle and put a runt edge on it -- one tick,
        which is shorter than the half bit the monitor allows.  Forcing `set pins, 1` first
        means the pin is already at the idle level by the time it becomes an output, so the
        line never moves.  (`program.pio` repeats the same two instructions in that order, so
        a program started without this step still comes up idle.)

        That forced `jmp start` cycle also runs the background autopull
        (`docs/mvp-isa.md` `### Reference step`), so it has to happen *before* the host fills
        the TX FIFO: otherwise the forced cycle pops a word into the OSR and `SM_RESTART`
        (which does not touch the OSR, only `osr_cnt`) leaves that stale word to be sent.
        """
        self.write(self.sm_addr(SM_INSTR), set_pins(IDLE_LEVEL))
        self.write(self.sm_addr(SM_INSTR), set_pindirs(1))
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

    def start(self, first: Sequence[int] = ()) -> None:
        """Arm, optionally fill the TX FIFO while stopped, then enable.

        `first` is what a host that must not see a stall in FDEBUG pushes between the two:
        the program's first message instruction is a blocking `pull`, so an engine started
        with an empty TX FIFO sits on FDEBUG.TXSTALL until the first byte arrives, which is
        where a transmitter between messages belongs anyway.
        """
        self.arm()
        if first:
            self.preload(first)
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

    # ---- the TX FIFO ----
    @property
    def tx_full(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXFULL << self.engine))

    @property
    def tx_empty(self) -> bool:
        return bool(self.fstat() & (FSTAT_TXEMPTY << self.engine))

    def encode(self, byte: int) -> int:
        """The TX word for one message byte, right-justified in bits 7:0.

        `OUT_SHIFTDIR = 1` makes the OSR shift right, so bit 0 of the word is the first bit on
        the wire.  For G.E. Thomas the line levels are the complement of IEEE 802.3's, so the
        word is the complement of the byte: the complement of the complement is the line
        inversion the convention asks for, and nothing else in the program changes.
        """
        if not 0 <= byte <= 0xFF:
            raise ValueError(f"a byte is 0..255 (got {byte!r})")
        word = byte & 0xFF
        return word ^ 0xFF if self.convention == "thomas" else word

    def push(self, byte: int) -> None:
        """Queue one message, waiting for FIFO room first so no word is dropped (TXOVER)."""
        while self.tx_full:
            self._settle(1)
        self.write(A_TXF[self.engine], self.encode(byte))
        self._settle(1)

    def send(self, values: Iterable[int]) -> None:
        """Queue several messages, in order, waiting for FIFO room between them."""
        for byte in values:
            self.push(byte)

    def preload(self, values: Sequence[int]) -> None:
        """Fill the TX FIFO while the engine is stopped.

        Only `TXFIFO_DEPTH` words fit and a disabled engine never pops, so this is how a host
        gets the first messages in before starting: the program's `pull block` would otherwise
        stall on the empty FIFO (and set FDEBUG.TXSTALL) on its first SM cycle.
        """
        values = list(values)
        if len(values) > self.TXFIFO_DEPTH:
            raise ValueError(
                f"{len(values)} bytes do not fit the stopped {self.TXFIFO_DEPTH}-deep TX FIFO; "
                "call send() after start()"
            )
        for byte in values:
            self.push(byte)

    # ---- CTRL helpers ----
    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask


def shiftctrl_pull_thresh(program: Program) -> int:
    """PULL_THRESH the program needs: one message, i.e. the bits its `set x, n` counts."""
    return bits_per_word(program)

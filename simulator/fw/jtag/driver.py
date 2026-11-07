"""Host side of the MVP JTAG firmware: the register writes that run fw/jtag/program.pio.

Written against docs/mvp-regmap.md, not against the ISS: every address and field below is
the word address and bit position of the register map, so the same driver drives the RTL
once it exists. Host accesses go through the queue of `iss.pio_iss.ChipDevice` (one access
per core cycle, in order) and the driver is what steps the `verif.vip.core.Bus` while it
waits for a read to come back.

**The division of labour, and why it is this way.** `program.pio` is a bit pump: it clocks
TCK and shifts one (TMS, TDI) pair per cycle out of the TX FIFO, sampling TDO into the ISR
in the cycles where the host said the TAP is in a shift state. Everything above that -- the
16-state machine, which TMS bit goes with which scan phase, when a scan starts -- lives
here, because it is the part a PIO state machine cannot hold in 32 words and because it is
the part a real JTAG driver owns: `verif/vip/jtag.py`'s own `JtagMaster` is a bit pump too
(`scan_ir()`/`scan_dr()` queue cycles, `JtagTarget` decodes them). The sequences below are
the ones IEEE 1149.1 needs and that `verif/vip/jtag.py`'s transition table spells out, and
fw/jtag/test_jtag.py cross-checks every one of them against that table.

One entry point, forced through `SMx_INSTR` (`SMx_ADDR` is read-only and the MVP has no IRQ
to wake the engine with) -- three instructions, and `program.pio` calls the address both
`scan` and `frame`:

    set y, e - 1  framing clocks before the shift phase
    set x, n - 1  clocks in the shift phase
    jmp frame     then 2 exit clocks, and the engine parks

The "- 1" is because a PIO loop tests its counter at the bottom and `jmp x--` tests before
it decrements, so a loop runs one more time than the value it holds; `loop_count` is the
only place that knows. It has two consequences this module has to respect: the shift phase
counts X (so that X = 31 is the 32 clocks of an IDCODE, which Y cannot hold) and no phase
can be told "zero times", so every scan clocks at least once in each of them.

The framing and exit clocks' TMS bits come from the FIFO as well, so `reset`, `idle`,
`scan_ir` and `scan_dr` are all the same forced triple with different words behind it.

**One TX word is sixteen TCK clocks, always.** The program's OSR holds one 32-bit word and
is refilled by autopull exactly when it empties (PULL_THRESH = 32), so a scan that does not
end on a word boundary would leave the OSR holding bits that belong to the *next* scan, and
the host cannot know how many they are without reading SM0's OSR back (it cannot). Every
scan below is therefore padded with idle clocks -- real TCK cycles with TMS = 0 in
Run-Test/Idle, which is what a probe does between scans anyway -- up to a whole number of
words, and each scan starts with an empty OSR. The padding goes in front of the entry
sequence, so the extra clocks are idle clocks in Run-Test/Idle and the scan still starts from
the state the host meant.
"""
from __future__ import annotations

from fw import regmap

from typing import List, Optional, Sequence, Tuple

from asm.pioasm import Program

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) -----------------
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF0, A_RXF0 = regmap.txf_addrs()[0], regmap.rxf_addrs()[0]
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_IMEM = regmap.imem_bases()              # engine e, IMEM word k at A_IMEM[e] + k
A_SM = regmap.sm_base()                        # SMe_* at A_SM + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

FLEVEL_TX0, FLEVEL_RX0 = 0, 4      # shift of the engine-0 field of FLEVEL 3:0 / 7:4

# Pin assignment of the program. fw/jtag/program.pio carries the same four numbers in its
# `.define` lines; `check_program` makes sure the two files still agree.
DEFAULT_TCK_PIN, DEFAULT_TMS_PIN, DEFAULT_TDI_PIN, DEFAULT_TDO_PIN = 0, 1, 2, 3


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


PAIRS_PER_WORD = 16                # one 32-bit TX word is 16 (TMS, TDI) pairs
SCAN_LIMIT = 200_000               # core cycles one scan may take before we give up

# The TAP sequences, as TMS levels.  Each is the shortest walk from Run-Test/Idle that the
# 16-state table in `research/ref/protocols/jtag/tap-state-machine-pbeirne.txt` allows:
#   IR scan  1,1,0,0   Select-IR-Scan, Capture-IR, Shift-IR      (jtag.py `### The states`)
#   DR scan  1,0,0     Select-DR-Scan, Capture-DR, Shift-DR
#   exit     1,0       Exit1-*, Update-*, Run-Test/Idle
# The last shift clock carries TMS = 1 as well, which is why the bit pairs below end in 1:
# "The clock that leaves Shift-IR is a shifting clock too" (jtag.py `scan_ir`).
IR_ENTRY_TMS = (1, 1, 0, 0)
DR_ENTRY_TMS = (1, 0, 0)
EXIT_TMS = (1, 0)
RESET_ENTRY_TMS = (1, 1, 1, 1)    # plus EXIT_TMS: five TMS-high clocks, then one low,
RESET_EXIT_TMS = (1, 0)           # which is the "at least five clock cycles" of
                                  # segger-kb-jtag.txt, and lands in Run-Test/Idle
IDLE_ENTRY_TMS = (0, 0)            # plus EXIT_TMS: TMS-low clocks in Run-Test/Idle
IDLE_EXIT_TMS = (0, 0)
#: The exit length the *program* walks, whatever the host's pair stream says
#: (`set x, 2` in `exit_go`): the host has to supply exactly this many exit pairs.
PROGRAM_EXIT_CLOCKS = 2
#: The most bits one pass can shift.  The shift phase counts X and `jmp x--` runs X+1
#: times, so X = 31 is 32 clocks -- the whole of a 32-bit IDCODE, which is the longest
#: scan IEEE 1149.1 has, in one contiguous Shift-DR phase.  That contiguity is not a
#: nicety: `JtagChecker` ends a DR scan every time the TAP leaves Shift-DR and checks each
#: one against the register's full length, so splitting a 32-bit scan over two Shift-DR
#: phases (a Pause-DR, which real SVF drivers do use for long scans) is reported as a short
#: DR scan twice over.
MAX_SHIFT_BITS = 32
#: The opcode the bench's TAP answers to for IDCODE.  Not the firmware's business in
#: general -- `scan_dr(length=...)` is -- but every JTAG tool starts by reading IDCODE, so
#: `idcode()` and the benchmarks below want the conventional value, which is the one
#: `verif/vip/jtag.py` uses for its built-in target (an implementer-defined choice,
#: openocd-tap-declaration.txt).
OPCODE_IDCODE = 0x02


class JtagError(RuntimeError):
    """The engine or the bus did something the driver cannot make sense of."""


# ---- one (TMS, TDI) pair and the words that carry them --------------------------------
def pair(tms: int, tdi: int = 0) -> int:
    """One TCK cycle as the program consumes it.

    `out pins, 2` walks the OSR from the bottom (OUT_SHIFTDIR = 1) and the OUT window puts
    data bit 0 on TMS_PIN and bit 1 on TDI_PIN, so a cycle is two bits: TMS first.
    """
    for name, value in (("TMS", tms), ("TDI", tdi)):
        if value not in (0, 1):
            raise ValueError(f"{name} must be 0 or 1, got {value!r}")
    return (tdi << 1) | tms


def scan_bits(value: int, width: int) -> List[int]:
    """`width` bits of `value`, LSB first: the order a JTAG scan puts them on TDI.

    Independent of `verif.vip.jtag.scan_bits`, on purpose -- the firmware must not borrow
    the model it is judged against. fw/jtag/test_jtag.py checks the two agree.
    """
    if width < 0:
        raise ValueError(f"a scan of {width} bits")
    value = int(value) & ((1 << int(width)) - 1)
    return [(value >> i) & 1 for i in range(int(width))]


def pack_pairs(pairs: Sequence[int]) -> List[int]:
    """`pairs` into TX words, 16 cycles to a word, pair 0 in bits 1:0.

    `len(pairs)` must be a whole number of words: see the module docstring for why every
    scan is padded to one.
    """
    if len(pairs) % PAIRS_PER_WORD:
        raise JtagError(
            f"{len(pairs)} (TMS, TDI) pairs is not a whole number of {PAIRS_PER_WORD}-cycle "
            f"words; the caller has to pad the scan with idle clocks"
        )
    words = []
    for i in range(0, len(pairs), PAIRS_PER_WORD):
        word = 0
        for k, p in enumerate(pairs[i:i + PAIRS_PER_WORD]):
            word |= (p & 3) << (2 * k)
        words.append(word)
    return words


# ---- what the program's instruction words say -----------------------------------------
def side_word(word: int, sideset_bits: int, value: int) -> int:
    """The instruction `word` with its side-set field forced to `value`.

    The program's `.side_set 1` has no `opt`, so the side-set bits are the top bit of the
    delay/side-set field and *every* instruction carries one -- a forced one included
    ("Its side-set applies", docs/mvp-isa.md `### EXEC`). That makes the side value part of
    a forced instruction's meaning: `fw/i2c/driver.py` and `fw/spi/driver.py` patch it for
    the same reason. Here it has to be 1, TCK high, because `park` leaves TCK high and a
    forced instruction that pulled it low would put a falling edge on the wire with no data
    change under it, `LOW_HOLD` core cycles before the scan's first pair -- which
    `JtagChecker` reads as a TDI.setup and a TMS.setup violation.
    """
    if not 1 <= sideset_bits <= 5:
        raise ValueError(f"the program has {sideset_bits} side-set bits")
    if not 0 <= value < (1 << sideset_bits):
        raise ValueError(f"side {value} does not fit in {sideset_bits} side-set bit(s)")
    shift = 8 + 5 - sideset_bits
    return (word & ~(((1 << sideset_bits) - 1) << shift)) | (value << shift)


def jmp_word(address: int, sideset_bits: int = 1, side: int = 1) -> int:
    """`jmp <address>` with no condition, no delay and the given side value.

    JMP is opcode 000, so with no condition and no delay the word is the address in bits
    4:0 and the side value in the top bit of the delay/side-set field; `side_word` puts it
    there.
    """
    return side_word(int(address) & 0x1F, sideset_bits, side)


def set_x_word(value: int, sideset_bits: int = 1, side: int = 1) -> int:
    """`set x, value`: opcode 111, destination X (001), data in bits 4:0."""
    if not 0 <= value <= 31:
        raise ValueError(f"set x, {value}: the data field is 5 bits, and X counts TCK cycles")
    return side_word((7 << 13) | (1 << 5) | value, sideset_bits, side)


def set_y_word(value: int, sideset_bits: int = 1, side: int = 1) -> int:
    """`set y, value`: opcode 111, destination Y (010), data in bits 4:0."""
    if not 0 <= value <= 31:
        raise ValueError(f"set y, {value}: the data field is 5 bits, and Y counts shift bits")
    return side_word((7 << 13) | (2 << 5) | value, sideset_bits, side)


def loop_count(clocks: int) -> int:
    """What the host loads into X or Y for a loop that has to run `clocks` times.

    Both of the program's loops test their counter with `jmp x--` / `jmp y--` at the
    *bottom*, and that condition "tests X before it decrements, whether or not the jump is
    taken" (docs/mvp-isa.md `### Field values`), so the loop runs one more time than the
    loaded value: `X = clocks - 1` runs `clocks` clocks. `exit_go`'s `set x, 1` in
    program.pio is the same arithmetic written out for the fixed two-clock exit.
    """
    if clocks < 1:
        raise JtagError(f"{clocks} clocks: a loop that runs is skipped by `jmp !x` / `jmp !y`")
    if clocks > 32:
        raise JtagError(f"{clocks} clocks: X and Y are 5-bit `set` fields, so one pass is 32")
    return clocks - 1


def _delay_of(word: int, sideset_bits: int) -> int:
    """The delay of an instruction word, given the program's side-set width.

    The low `5 - SC` bits of the delay/side-set field are the delay (docs/mvp-isa.md
    `### Delay/side-set field`), so with SC = 1 the delay is bits 11:8.
    """
    return (word >> 8) & ((1 << (5 - sideset_bits)) - 1)


def loop_delays(program: Program) -> Tuple[int, int]:
    """The `(LOW_HOLD, HIGH_HOLD)` the program was assembled with.

    Read back out of the instruction words rather than repeated here, so that a number in
    this file and the number on the wire cannot drift apart: the driver quotes the TCK
    period it expects the checker to measure, and the test asserts the two agree.

    `HIGH_HOLD` is the *smallest* delay on a `nop`, which is the shift loop's: the two
    loops that sample no TDO have no `in pins, 1` to spend that SM cycle on and put it in
    the high phase instead (`FRAME_HIGH`), so that every TCK cycle on the wire is the same
    length. The shift loop is what sets the rate, so it is the one whose delay counts.
    """
    lows = {_delay_of(w, program.sideset_bits) for w in program.instrs
            if (w >> 13) == 3 and ((w >> 5) & 7) == 0 and (w & 0x1F) == 2}
    highs = {_delay_of(w, program.sideset_bits) for w in program.instrs
             if (w >> 13) == 5 and ((w >> 5) & 7) == 2 and (w & 7) == 2
             and _delay_of(w, program.sideset_bits)}
    if len(lows) != 1 or not highs:
        raise JtagError(
            f"program.pio has {len(lows)} different `out pins, 2` delays {sorted(lows)} and "
            f"{len(highs)} different `nop` delays {sorted(highs)}; the bit loop needs one "
            f"low-phase delay and a high-phase delay, so that every TCK cycle is the same "
            f"length"
        )
    return lows.pop(), min(highs)


def tck_cycles(program: Program) -> int:
    """SM cycles one TCK cycle costs: the two delays plus the four instructions.

    `out pins, 2 [LOW_HOLD]` (1 + LOW_HOLD), `nop [HIGH_HOLD]` (1 + HIGH_HOLD),
    `in pins, 1`, `jmp x--, shift`: LOW_HOLD + HIGH_HOLD + 4.

    T_OUT does not add to the period -- it shifts the falling edge and the rising edge by
    the same tick -- but it does set the low time, which is LOW_HOLD + 1 ticks on the wire
    (the `side 0` of `out pins, 2` reaches the pad one tick after that instruction runs).
    Three ticks is the floor: TDO appears two ticks after the falling edge and the
    profile's Ttdo wants one more before the rising edge.
    """
    low, high = loop_delays(program)
    return low + high + 4


def tck_hz(program: Program, clkdiv: int, ns_per_tick: float) -> float:
    """The TCK rate this program puts on the bus at `clkdiv`, in Hz.

    One SM cycle is `clkdiv` core cycles and one core cycle is `ns_per_tick`, so the rate
    is 1e9 / (ns_per_tick * clkdiv * tck_cycles). Nothing here is a model number: the
    checker measures the period off the wire and the test compares the two.
    """
    if clkdiv < 1:
        raise ValueError(f"CLKDIV {clkdiv} is below 1")
    return 1e9 / (float(ns_per_tick) * int(clkdiv) * tck_cycles(program))


def retime(program: Program, low_hold: Optional[int] = None,
           high_hold: Optional[int] = None) -> Program:
    """The same program with different TCK phase delays, in SM cycles.

    What program.pio's `.define LOW_HOLD` / `.define HIGH_HOLD` lines do at assembly time,
    done here on the words -- which is how fw/jtag/test_jtag.py asks for a TCK cycle that
    is *too* short and checks that `JtagChecker` catches it. `HIGH_HOLD` is the shift
    loop's: it is what puts the `in pins, 1` exactly three cycles after the rising edge,
    which is the T_IN = 3 pipeline of docs/mvp-isa.md `## Timing`, so moving it moves the
    sample off the rising edge rather than just changing a delay. The two loops that sample
    no TDO keep their one-cycle-longer high phase, so every TCK cycle stays the same length.
    """
    old_low, old_high = loop_delays(program)
    low = old_low if low_hold is None else int(low_hold)
    high = old_high if high_hold is None else int(high_hold)
    if not 0 <= low <= 15 or not 0 <= high <= 15:
        raise JtagError(f"LOW_HOLD {low} / HIGH_HOLD {high}: the delay field is 4 bits wide")
    sc = program.sideset_bits
    field = (1 << (5 - sc)) - 1          # the delay bits are the low `5 - SC` of the field
    words = []
    for word in program.instrs:
        delay = _delay_of(word, sc)
        if (word >> 13) == 3 and ((word >> 5) & 7) == 0 and (word & 0x1F) == 2:
            delta = low - old_low                                   # `out pins, 2`
        elif (word >> 13) == 5 and ((word >> 5) & 7) == 2 and (word & 7) == 2 and delay:
            delta = (high if delay == old_high                      # `shift`'s `nop`
                     else high + 1) - delay                         # `frame`'s and `exit`'s
        else:
            words.append(word)
            continue
        words.append((word & ~(field << 8)) | ((delay + delta) << 8))
    return Program(
        name=program.name, instrs=words, origin=program.origin,
        wrap_target=program.wrap_target, wrap=program.wrap,
        sideset_bits=sc, sideset_opt=program.sideset_opt,
        sideset_pindirs=program.sideset_pindirs, labels=program.labels,
    )


def check_program(program: Program, tck: int, tms: int, tdi: int, tdo: int) -> None:
    """Fail loudly if the driver and the .pio file disagree about the pins or the shape.

    The program names its pins in `.define` lines, which the assembler does not hand back,
    so what is checked instead is that the instructions the driver relies on are there:
    a two-pin `out pins` for one (TMS, TDI) pair per TCK cycle, a one-pin `in pins` for
    TDO, a one-bit non-optional side-set for TCK, and the labels the host forces jumps to.
    """
    if len(program.instrs) > 32:
        raise JtagError(
            f"the program needs {len(program.instrs)} words, the IMEM holds 32 "
            f"(docs/mvp-isa.md `### Engine state`)"
        )
    if program.sideset_bits != 1 or program.sideset_opt or program.sideset_pindirs:
        raise JtagError(
            f"the driver programs PINCTRL for `.side_set 1` on a level, program.pio says "
            f"{program.sideset_bits} bit(s), opt={program.sideset_opt}, "
            f"pindirs={program.sideset_pindirs}: TCK is the side-set bit and it is driven "
            f"high or low, never a direction"
        )
    if not any((w >> 13) == 3 and ((w >> 5) & 7) == 0 and (w & 0x1F) == 2 for w in program.instrs):
        raise JtagError("program.pio never puts a (TMS, TDI) pair on the wire with `out pins, 2`")
    if not any((w >> 13) == 2 and ((w >> 5) & 7) == 0 and (w & 0x1F) == 1 for w in program.instrs):
        raise JtagError("program.pio never samples TDO with `in pins, 1`")
    if not any((w >> 13) == 4 and (w & 0x1F) == 0 for w in program.instrs):
        raise JtagError("program.pio has no `push`, so a scan's TDO bits never reach the host")
    # The three startup instructions that make TCK, TMS and TDI outputs and park TCK high:
    # without them the chip is an input on a pulled-up bus, which no JTAG checker accepts.
    for what, dest, data in (("set pins, 6", 0, 6), ("set pindirs, 7", 4, 7)):
        if not any((w >> 13) == 7 and ((w >> 5) & 7) == dest and (w & 0x1F) == data
                   for w in program.instrs):
            raise JtagError(
                f"program.pio has no `{what}`, so it never makes TCK, TMS and TDI outputs "
                f"at levels that leave the wire alone"
            )
    # `park` has to hold TCK high: that is the falling edge the next scan's first pair
    # rides on.  The side-set bits are the top `SC` bits of the delay/side-set field
    # (docs/mvp-isa.md `### Delay/side-set field`), so with SC = 1 they are bit 12.
    sc = program.sideset_bits
    parked = program.instrs[program.labels["park"]]
    if sc and ((parked >> (8 + 5 - sc)) & ((1 << sc) - 1)) != 1:
        raise JtagError(
            f"program.pio parks with TCK low (word {parked:#06x} carries side "
            f"{(parked >> (8 + 5 - sc)) & ((1 << sc) - 1)}): the next scan's first "
            f"(TMS, TDI) pair would then change with no falling edge under it, which is "
            f"a TDI.setup and a TMS.setup violation"
        )
    if program.labels["scan"] != program.labels["frame"]:
        raise JtagError(
            f"program.pio's `scan` ({program.labels['scan']}) and `frame` "
            f"({program.labels['frame']}) are different words: the host forces one jump and "
            f"the scan then runs the framing clocks and the shift phase in sequence"
        )
    for name in ("init_pin", "park", "scan", "frame", "shift", "exit_go", "exit", "last"):
        if name not in program.labels:
            raise JtagError(f"program.pio has no `{name}` label for the host to reach")
    if not program.wrap_target <= program.labels["park"] <= program.wrap:
        raise JtagError(
            f"the wrap window [{program.wrap_target}, {program.wrap}] does not contain "
            f"`park` at {program.labels['park']}, so a finished scan could wrap into the "
            f"middle of a scan"
        )
    # The pins themselves: the OUT window is two pins wide and the SET window three, and
    # the program writes both from immediates, so the map has to line up with them.
    if not 0 <= tck <= 5:
        raise ValueError(f"TCK pin {tck}: the three driven pins are TCK, TCK+1 and TCK+2")
    if (tms, tdi) != (tck + 1, tck + 2):
        raise ValueError(
            f"the program drives TMS and TDI from one two-pin OUT window at TCK+1, and "
            f"`set pindirs, 7` from a three-pin SET window at TCK, so TMS = TCK+1 and "
            f"TDI = TCK+2; got TCK={tck}, TMS={tms}, TDI={tdi}"
        )
    if not 0 <= tdo <= 17:
        raise ValueError(f"TDO pin {tdo} is not one of the 18 protocol pins")
    if tck <= tdo <= tdi:
        raise ValueError(
            f"TDO pin {tdo} is inside the driven window TCK..TCK+2: `set pindirs, 7` would "
            f"make TDO an output and the chip could not read it"
        )


class JtagDriver:
    """Load fw/jtag/program.pio into one engine and drive JTAG scans with it.

    The pin map is `(TCK, TMS, TDI, TDO)`; the three driven pins have to be three
    consecutive pins starting at TCK, because the program drives TCK from a one-bit
    side-set, TMS and TDI from one two-pin OUT window and all three directions from one
    three-pin SET window, and every one of those is an immediate in IMEM. `retarget` is
    deliberately absent: there is nothing in the program that names a pin except the
    `.define` lines, which the assembler does not hand back, so a map that does not line
    up with those windows is refused rather than patched.
    """

    def __init__(
        self,
        bus,
        device,
        program: Program,
        clkdiv: int = 1,
        engine: int = 0,
        pins: Tuple[int, int, int, int] = (
            DEFAULT_TCK_PIN, DEFAULT_TMS_PIN, DEFAULT_TDI_PIN, DEFAULT_TDO_PIN
        ),
    ) -> None:
        if not 1 <= clkdiv <= 0xFFFF:
            raise ValueError(f"CLKDIV integer part {clkdiv} is out of range")
        if not 0 <= engine <= 1:
            raise ValueError(f"engine {engine}: the MVP has two (docs/mvp-isa.md `### Differences`)")
        tck, tms, tdi, tdo = (int(p) for p in pins)
        check_program(program, tck, tms, tdi, tdo)
        self.bus = bus
        self.dev = device
        self.prog = program
        self.engine = engine
        self.clkdiv = clkdiv
        self.pins = (tck, tms, tdi, tdo)
        self.peak_tx = self.peak_rx = 0
        self._init_pin = program.labels["init_pin"]
        self._park = program.labels["park"]
        self._frame = program.labels["frame"]      # `scan` is the same word
        self.clocks = 0             # TCK cycles this driver has put on the bus

    # ---- the register words the program needs --------------------------------------
    @property
    def clkdiv_word(self) -> int:
        return (self.clkdiv & 0xFFFF) << 16

    @property
    def execctrl_word(self) -> int:
        """The wrap window, so that a finished scan parks; no SIDE_EN, because the program
        side-sets every instruction (so TCK follows the program out of every stall), and no
        SIDE_PINDIR, because TCK is a level and JTAG is push-pull."""
        return ((self.prog.wrap & 0x1F) << 12) | ((self.prog.wrap_target & 0x1F) << 7)

    @property
    def shiftctrl_word(self) -> int:
        """PULL_THRESH reads 0, which means 32 (`docs/mvp-regmap.md`: "0 means 32"), so
        AUTOPULL refills the OSR exactly when a whole TX word has been shifted out --
        one refill per 16 TCK cycles, which is why every scan is padded to a whole number
        of words.  Both shift directions are right: OUT_SHIFTDIR 1 walks the (TMS, TDI)
        pairs out from the bottom, so pair 0 is bits 1:0, and IN_SHIFTDIR 1 lands the first
        TDO bit in bit 0 (a right shift puts the first bit sampled in the bottom, and a
        left shift would leave it in the top -- the two are not the same, and only one of
        them is JTAG's LSB-first order).  AUTOPUSH is off: `push` at the end of the shift
        loop gives one RX word per scan whatever the scan length is."""
        return (1 << 19) | (1 << 18) | (1 << 17)

    @property
    def pinctrl_word(self) -> int:
        """Every window on the four JTAG pins: a one-bit side-set on TCK, a two-pin OUT
        window on TMS/TDI, a three-pin SET window on the driven three (`set pindirs, 7`
        has to reach all of them), and the IN window on TDO."""
        tck, tms, _tdi, tdo = self.pins
        return (
            (1 << 29)                     # SIDESET_COUNT 1: TCK
            | (3 << 26)                   # SET_COUNT 3: TCK, TMS, TDI
            | (2 << 20)                   # OUT_COUNT 2: TMS, TDI
            | (tdo << 15)                 # IN_BASE: `in pins, 1` reads TDO
            | (tck << 10)                 # SIDESET_BASE
            | (tck << 5)                  # SET_BASE
            | tms                         # OUT_BASE
        )

    @property
    def sm(self) -> int:
        """Word address of this engine's `SMe_CLKDIV`."""
        return A_SM + 6 * self.engine

    @property
    def tck_hz(self) -> float:
        """The TCK rate this driver programs, from the program's own delays."""
        return tck_hz(self.prog, self.clkdiv, getattr(self.bus, "ns_per_tick", 20.0))

    # ---- register access -----------------------------------------------------------
    def step(self) -> None:
        """One core cycle: the ChipDevice performs at most one queued host access."""
        self.bus.step()

    def run(self, ticks: int) -> None:
        for _ in range(ticks):
            self.step()

    def reg_write(self, addr: int, data: int) -> None:
        """Queue a register write; it is performed on the next cycle the driver steps."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def reg_read(self, addr: int) -> int:
        """Queue a register read and step the bus until the answer is back."""
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.step()
            ticks += 1
            if ticks > SCAN_LIMIT:
                raise JtagError(f"read of word address {addr:#04x} never came back")
        return future.result()

    def wait_until(self, what, limit: int = SCAN_LIMIT) -> None:
        ticks = 0
        while not what():
            self.step()
            ticks += 1
            if ticks > limit:
                raise JtagError(f"timed out after {ticks} core cycles")

    # ---- engine setup --------------------------------------------------------------
    def load(self) -> None:
        """The host sequence of `docs/mvp-regmap.md ### Typical host sequences`.

        The engine is left parked in `park` with TCK high and nothing shifting: a scan
        forces `jmp frame` itself, once its words are in the FIFO, so no TCK cycle can go
        out before the data to put on it. The three pins are outputs by then, because
        `init_pin` runs first out of IMEM -- which is why step 4 of the sequence ("set the
        initial pins with a forced instruction") is not needed here: the program needs an
        edge on TCK to make itself a master, and a forced `set pindirs` would have had to
        change TMS and TDI on a tick with no falling edge anywhere near it.
        """
        self.reg_write(A_CTRL, 0)                                     # stop the engine
        for k, word in enumerate(self.prog.instrs):                   # IMEM burst
            self.reg_write(A_IMEM[self.engine] + k, word)
        self.reg_write(self.sm + SM_CLKDIV, self.clkdiv_word)
        self.reg_write(self.sm + SM_EXECCTRL, self.execctrl_word)
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word)
        self.reg_write(self.sm + SM_PINCTRL, self.pinctrl_word)
        self.reg_write(A_CTRL, self._ctrl_start)
        self.wait_parked()

    @property
    def _ctrl_start(self) -> int:
        """CTRL with this engine enabled and its divider restarted, so the first SM cycle
        is two core cycles after this access (docs/mvp-isa.md `### Clock divider`)."""
        e = self.engine
        return (1 << (8 + e)) | (1 << (4 + e)) | (1 << e)

    def force(self, word: int) -> None:
        """Push one instruction into the engine's EXEC latch.

        It executes in the next core cycle whatever SM_ENABLE and the divider say, it does
        not advance the PC and its delay is ignored (docs/mvp-isa.md `### EXEC`). Every
        forced instruction here carries no side value, i.e. TCK low, which is where a
        parked engine holds it: a forced instruction that drove TCK high for a cycle would
        put a TCK cycle nobody asked for on the wire.
        """
        self.reg_write(self.sm + SM_INSTR, word & 0xFFFF)

    @property
    def pc(self) -> int:
        """`SMx_ADDR`, the engine's PC. There is no IRQ in the MVP, so this and the FIFO
        levels are all the host can see of the program's progress."""
        return self.reg_read(self.sm + SM_ADDR) & 0x1F

    @property
    def parked(self) -> bool:
        return self.pc == self._park

    def wait_parked(self) -> None:
        self.wait_until(lambda: self.parked)

    def wait_left_parked(self) -> None:
        """Step until the engine has left `park`, i.e. a forced jump landed.

        A forced instruction runs in the *next* core cycle, so reading SM0_ADDR right after
        writing SM0_INSTR still shows the old PC: without this the caller would go on
        believing the scan it just started has already finished.
        """
        self.wait_until(lambda: self.pc != self._park)

    def fdebug(self) -> int:
        return self.reg_read(A_FDEBUG)

    def clear_fdebug(self) -> None:
        self.reg_write(A_FDEBUG, 0xFFFF_FFFF)

    def _tx_level(self) -> int:
        level = (self.reg_read(A_FLEVEL) >> FLEVEL_TX0) & 0xF
        self.peak_tx = max(self.peak_tx, level)
        return level

    def _rx_level(self) -> int:
        level = (self.reg_read(A_FLEVEL) >> FLEVEL_RX0) & 0xF
        self.peak_rx = max(self.peak_rx, level)
        return level

    @property
    def peak_fifo_depth(self) -> int:
        """The deepest either FIFO got while the driver was running.

        Both are four deep (docs/mvp-isa.md `### Engine state`), so anything below four means
        the host queued each scan's words before starting it instead of relying on the
        engine's blocking stalls to throttle it.
        """
        return max(self.peak_tx, self.peak_rx)

    # ---- one scan on the wire ------------------------------------------------------
    def scan(self, entry_tms: Sequence[int], bits: Sequence[int],
             exit_tms: Sequence[int] = EXIT_TMS) -> Optional[int]:
        """One scan: `entry_tms` framing clocks, `bits` shifted LSB first, `exit_tms`.

        Framing clocks shift a (TMS, TDI) pair like any other cycle but sample nothing, so
        the host alone decides which cycles of a scan carry data. That is what lets the
        entry and exit sequences be data (`scan_dr`) or not (`reset`, `idle`), and it is
        why `exit_tms` is a parameter rather than a constant in the program.

        Two of the program's rules show up here: the scan is padded with idle clocks until
        both phases have at least one clock (a loop cannot be told "zero times"), and a scan
        with nothing to shift still clocks once through the shift phase and still pushes its
        RX word, which is read and thrown away. Returns the `len(bits)` TDO bits of the
        shift phase as an integer, bit 0 = the first bit off TDO, or None when there was
        nothing to shift.
        """
        entry = tuple(int(t) & 1 for t in entry_tms)
        bits = [int(b) & 1 for b in bits]
        exit_ = tuple(int(t) & 1 for t in exit_tms)
        if len(exit_) != PROGRAM_EXIT_CLOCKS:
            raise JtagError(
                f"program.pio walks {PROGRAM_EXIT_CLOCKS} exit clocks (`set x, 1` in "
                f"`exit_go`, one clock per loaded count), so a scan needs exactly that many "
                f"exit TMS levels, got "
                f"{len(exit_)}: {exit_tms}"
            )
        if len(entry) > 32 or len(bits) > MAX_SHIFT_BITS:
            raise JtagError(
                f"{len(entry)} framing clocks and {len(bits)} shift bits: X and Y are 5-bit "
                f"`set` fields and a loop runs one more time than it holds, so one pass "
                f"takes 1..32 of either"
            )
        # The clock that leaves the shift state is a shifting clock too, so the last bit
        # goes in on it (jtag.py `scan_dr`, "The clock that leaves Shift-IR is a shifting
        # clock too").
        bit_pairs = [pair(1 if k == len(bits) - 1 else 0, b) for k, b in enumerate(bits)]
        framing = [pair(t) for t in entry]
        shift = len(bit_pairs)
        if not shift:
            # A loop cannot be told "zero times" (see program.pio, "THE PROGRAM IS THREE
            # LOOPS AND A PARK"), so the shift phase runs one clock whatever it has to
            # shift.  Its TMS is the level in front of it, which keeps a reset a reset:
            # one more TMS-high clock still leaves the TAP in Test-Logic-Reset.
            shift = 1
            bit_pairs = [pair(entry[-1] if entry else 0)]
        if not framing:
            framing = [pair(0)]
        pairs = framing + bit_pairs + [pair(t) for t in exit_]
        # Pad the front with idle clocks so the scan is a whole number of TX words: see the
        # module docstring. The pad clocks are TMS = 0 in Run-Test/Idle, so the TAP sees a
        # scan that starts when the host meant it to.
        pad = (-len(pairs)) % PAIRS_PER_WORD
        if pad:
            pairs = [pair(0)] * pad + pairs
            framing = [pair(0)] * pad + framing
        words = pack_pairs(pairs)
        if len(words) > regmap.fifo_depth():
            raise JtagError(
                f"this scan is {len(words)} TX words and the FIFO holds {regmap.fifo_depth()}; a scan "
                f"of {len(bits)} bits cannot be queued in one go"
            )
        self.wait_parked()
        for word in words:
            while self._tx_level() >= regmap.fifo_full_level():
                self.step()
            self.reg_write(A_TXF0, word)
        sc = self.prog.sideset_bits
        self.force(set_y_word(loop_count(len(framing)), sc))
        self.force(set_x_word(loop_count(shift), sc))
        self.force(jmp_word(self._frame, sc))
        self.clocks += len(pairs)
        self.wait_left_parked()
        self.wait_parked()
        # Every scan pushes exactly one RX word, including a framing-only one (whose shift
        # clock had nothing to sample), so the word is always taken and then either used
        # or discarded.
        got = self._take_rx(shift)
        if not bits:
            return None
        if got >> len(bits):
            raise JtagError(
                f"the RX word {got:#010x} has bits above the {len(bits)} this scan shifted; "
                f"the ISR should hold nothing else"
            )
        return got

    def _take_rx(self, bits: int) -> int:
        """This scan's RX word, as the n bits the target shifted out, bit 0 first.

        IN_SHIFTDIR = 1 is a *right* shift, so after n samples the n bits sit in the top n
        bits of the ISR (docs/mvp-isa.md `### Instruction semantics`: `isr' = (ISR >> n) |
        (d << (32-n))`), the first one sampled highest. That is right for a full 32-bit scan
        and has to be brought down for a shorter one, which is the shift below. A *left*
        shift would put the first bit in bit 0 instead, but then a 32-bit scan comes back
        reversed -- the two settings are not interchangeable and only one of them is
        JTAG's LSB-first order for every scan length.
        """
        self.wait_until(lambda: self._rx_level() > 0)
        word = self.reg_read(A_RXF0)
        return (word >> (32 - bits)) & ((1 << bits) - 1)

    # ---- the scans a JTAG driver runs -----------------------------------------------
    def reset(self, clocks: int = 5) -> None:
        """`clocks` TCK cycles with TMS high, then one with TMS low.

        Five clocks with TMS high is what forces Test-Logic-Reset from any state
        ("if TMS is high for at least five clock cycles", segger-kb-jtag.txt); the extra
        cycle with TMS low lands in Run-Test/Idle, where every scan starts. This is also the
        one scan whose entry TMS levels are the same as its exit's, because both halves are
        just TMS = 1: four in, two out.
        """
        if clocks < 5:
            raise JtagError(
                f"a synchronous reset needs at least 5 TCK cycles with TMS high to reach "
                f"Test-Logic-Reset, got {clocks}"
            )
        high = clocks - 1                      # the program's exit contributes the fifth
        self.scan((1,) * high, (), (1, 0))

    def idle(self, clocks: int = 4) -> None:
        """`clocks` TCK cycles in Run-Test/Idle (TMS low), as a probe leaves a chain."""
        if clocks < PROGRAM_EXIT_CLOCKS:
            raise JtagError(f"idle({clocks}): a scan is at least {PROGRAM_EXIT_CLOCKS} clocks")
        self.scan((0,) * (clocks - PROGRAM_EXIT_CLOCKS), (), (0, 0))

    def scan_ir(self, instruction: int, ir_len: int = 8) -> Optional[int]:
        """Scan `ir_len` bits into the target's instruction register.

        Bits go out LSB first, so pass the plain opcode: `scan_ir(0x02)` puts 0x02 in an
        `ir_len`-bit register whatever its width. Returns the bits shifted back out of
        Capture-IR, which for a legal TAP are 0,1 in bits 1:0 ("JTAG requires the two LSBs
        of this value to be 01", openocd-tap-declaration.txt).
        """
        if not 2 <= ir_len <= MAX_SHIFT_BITS:
            raise ValueError(
                f"ir_len {ir_len}: 2 bits is the minimum, {MAX_SHIFT_BITS} the most one pass holds"
            )
        return self.scan(IR_ENTRY_TMS, scan_bits(instruction, ir_len), EXIT_TMS)

    def scan_dr(self, value: Optional[int] = None, length: int = 32) -> Optional[int]:
        """Scan `length` bits through the data register the current instruction selects.

        `value` goes out on TDI LSB first (None shifts zeros, which is what reading a
        register wants) and the bits shifted back out of TDO are returned as an integer,
        bit 0 first. `length` has to be the register's length: the checker knows the
        instruction table and flags a DR scan of any other length (`DR.length`).
        """
        if not 1 <= length <= MAX_SHIFT_BITS:
            raise ValueError(
                f"a DR scan of {length} bits: 1..{MAX_SHIFT_BITS}, and 32 is the IDCODE"
            )
        bits = scan_bits(0 if value is None else value, length)
        return self.scan(DR_ENTRY_TMS, bits, EXIT_TMS)

    def idcode(self, instruction: int = OPCODE_IDCODE, ir_len: int = 8) -> int:
        """reset, select the IDCODE instruction and read its 32 bits.

        The one sequence every JTAG tool starts with, so it is the one the benchmark in
        fw/jtag/measure.py and the headline test in fw/jtag/test_jtag.py run.
        """
        self.reset()
        self.scan_ir(instruction, ir_len)
        return self.scan_dr(None, 32) or 0
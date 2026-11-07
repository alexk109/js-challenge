"""Host side of the MVP SWD firmware: the register writes that run fw/swd/program.pio.

Written against docs/mvp-regmap.md (word addresses and field positions) and
docs/mvp-isa.md (divider, side-set, shift control), not against the ISS: every
number below is the word address or bit position of the register map, so the
same driver drives the RTL once it exists.  Host accesses go through the queue
of `iss.pio_iss.ChipDevice`, one access per core cycle, and this module is what
steps the `verif.vip.core.Bus` while it waits.

The chip is the **controller** -- the debug probe -- and `verif.vip.swd.SwdTarget`
is the SW-DP on the other end.  One PIO program, three host entry points, all
forced through `SMx_INSTR` because `SMx_ADDR` is read-only and there is no IRQ:

    set pins, n      n = 1 write, 0 read: the flag `jmp pin` reads at `e2`
    jmp xfer         run one transfer; the TX FIFO holds its three words

A transfer is one 47-bit-period SWD frame.  The three words the program pulls,
in order, are:

    word 0  the request: the 8-bit header inverted for the open-drain output
            (1 = release) in bits 7:0, bit 8 = 0 for the turnaround period
    word 1  read: unused, it only resets osr_cnt.  write: WDATA, inverted
    word 2  the closing word, bits 2:0 -- see `closing`

The program hands back what it received through the RX FIFO: for a read, the
32 RDATA bits in one word (`data_word`).  The ACK is clocked but not sampled, so
`verif.vip.swd.SwdChecker` is what judges it; see fw/swd/LIMITS.md for the two
things this division of labour costs.

Parity is computed here, on the host, and pushed into the FIFO as a data bit:

* the request's own parity bit is bit 5 of the header byte, which
  `verif.vip.swd.header_byte` already puts there [S15];
* the WDATA parity bit is `verif.vip.swd.data_parity` of the word being written,
  which the driver packs into bit 1 of the closing word.  A 32-bit parity in PIO
  costs about six instructions per bit, so the host does it -- exactly as a real
  probe's CPU does, and the wire is what the checker looks at either way.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from asm.pioasm import Program
from verif.vip import swd as vswd

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) ----------
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF0, A_RXF0 = regmap.txf_addrs()[0], regmap.rxf_addrs()[0]
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_IMEM = regmap.imem_bases()              # engine e, IMEM word k at A_IMEM[e] + k
A_SM = regmap.sm_base()                        # SMe_* at A_SM + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

# CTRL fields (docs/mvp-regmap.md `**CTRL**`)
CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART = 0x001, 0x010, 0x100
# FSTAT fields, engine e
FSTAT_TXEMPTY, FSTAT_TXFULL = 0x0100_0000, 0x0001_0000
# FLEVEL nibbles, engine 0
FLEVEL_TX0, FLEVEL_RX0 = 0, 4


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "swd"
WORDS_PER_TRANSFER = 3             # the most TX words one transfer pulls, see the docstring
READ_PERIODS = 46                  # bit periods one transfer clocks, read or write

# Pin assignment.  SWDIO must be a uio pin: open drain is a direction
# (docs/mvp-isa.md `### Open drain`).  SWCLK is an output-only pin, so it is
# always driven and the side-set only has to write its level, with no direction
# to set anywhere.  FLAG is a third pin the host writes with `set pins` and the
# program reads with `jmp pin`.
DEFAULT_SWDIO_PIN, DEFAULT_SWCLK_PIN, DEFAULT_FLAG_PIN = 0, 13, 14

# The program: PULL_THRESH 32 (`jmp !osre`), PUSH_THRESH 1 (`push iffull`),
# shift right on the way out and left on the way in, no autopull or autopush.
PULL_THRESH = 31                   # `jmp !osre` at `e1`: 9 after a request, 32 after WDATA
PUSH_THRESH = 1                    # `push iffull` hands the ISR over when it is not empty
REQUEST_PERIODS = 9                # the request byte plus its turnaround
ACK_PERIODS = 2                    # the periods the program clocks for the ACK: two,
                                    # measured on the bench: the DP puts data bit 0 in the
                                    # window after the second (fw/swd/program.pio `wack`)
WDATA_TX_PERIODS = 32              # WDATA bits `tx` sends; `p2` sends the parity bit
OUT_SHIFTDIR = 1                   # 1 = the OSR shifts right, so bit 0 goes out first
IN_SHIFTDIR = 1                    # 1 = the ISR shifts right, which puts bit k of the
                                    # received word in bit k (both are their reset values)
RDATA_PERIODS = 32                 # the length the host pushes for the receive run

# One bit period is 3 + HIGH + LOW core cycles (fw/swd/program.pio, `## The bit
# period`): a loop body is three instructions -- the clock-high `nop side 1`, the
# clock-low `out`/`in`/`nop`, and the `jmp` back -- and with CLKDIV = 1 one SM
# cycle is one core cycle, so the delays are the whole period.  HIGH has to be at
# least 6 for the input path: the DP's answer is on the pad two ticks after the
# rising edge and this chip reads SWDIO T_IN = 3 ticks after the pad
# (docs/mvp-isa.md `## Timing`), so six leaves two ticks of margin.
MIN_HIGH = 6
MIN_LOW = 0                        # `wack` pads the phase boundary with max(0, LOW - 1)
MAX_LOW = 14                       # `.side_set 1` leaves a 4-bit delay field
LOOPS_PER_PERIOD = 3               # instructions in tx, ck and rx
IDLE_DELAY = 15                   # the idle spin at `done`: one word, 16 cycles
PHASE_LIMIT = 400_000              # core cycles one transfer may take


class SwdError(RuntimeError):
    """The engine or the bus did something the driver cannot make sense of."""


def period_ticks(high: int, low: int) -> int:
    """One bit period in core cycles: the three instructions of a loop body plus
    their delays."""
    if high < MIN_HIGH:
        raise ValueError(
            f"HIGH {high} is below {MIN_HIGH}: `in pins, 1` runs HIGH + 1 cycles after "
            f"the edge it samples, and the answer is T_IN = 3 ticks after the pad "
            f"(docs/mvp-isa.md `## Timing`)"
        )
    if not MIN_LOW <= low <= MAX_LOW:
        raise ValueError(
            f"LOW {low} is outside {MIN_LOW}..{MAX_LOW}: the delay field is 4 bits, and "
            f"`wack` spends max(0, LOW - 1) of them making the phase boundary a whole "
            f"bit period"
        )
    return LOOPS_PER_PERIOD + high + low


def clkdiv_word(period: int) -> int:
    """`SMe_CLKDIV` for a bit period of `period` core cycles.

    A loop body is `period` instructions long and its delays are already counted
    in `period`, so the divider has to run at one SM cycle per core cycle:
    CLKDIV = 1.0.  Anything else would multiply the whole period again -- at
    CLKDIV = 2 a "9 tick" bit period would take 18 core cycles and the bit rate
    would be half what `period_ticks` says.
    """
    if period < LOOPS_PER_PERIOD + MIN_HIGH:
        raise ValueError(f"a bit period of {period} cycles is below one loop body")
    return 1 << 16


def swclk_hz(period: int, ns_per_tick: float = 20.0) -> float:
    """SWCLK in Hz for a bit period of `period` core cycles."""
    return 1e9 / (ns_per_tick * period)


# ---- the program -------------------------------------------------------------
def load_program(path: Path | str = PROGRAM_PATH) -> Program:
    """Assemble program.pio (asm/pioasm.py) and return its one program."""
    programs = assemble_program_text(Path(path).read_text(encoding="utf-8"))
    if len(programs) != 1 or programs[0].name != PROTO_NAME:
        raise SwdError(f"{path}: expected one .program named {PROTO_NAME}")
    return programs[0]


def assemble_program_text(text: str) -> List[Program]:
    from asm.pioasm import assemble

    return assemble(text)


def xfer_of(program: Program) -> int:
    """The word of `pull block` the transfer starts with, i.e. the label `xfer`."""
    return program.labels["xfer"]


def unrolled_period_words(program: Program) -> set:
    """The words that make up a bit period without a loop around it.

    `p2`'s three closing periods and `wpre`'s turnaround are two instructions
    each, where a loop body is three, so `retarget` gives their clock-low half one
    extra delay cycle.  They are found by label: `p2` is `pull block` followed by
    three `out pindirs, 1` / `nop side 1` pairs, and `wpre` is one `nop side 1` /
    `nop` pair.
    """
    p2 = program.labels["p2"]
    words = set()
    for period in range(3):
        words.update({p2 + 1 + 2 * period, p2 + 2 + 2 * period})
    wpre = program.labels["wpre"]
    words.update({wpre, wpre + 1})
    return words


def retarget(program: Program, high: int, low: int) -> Program:
    """The same program with the bit period `high`/`low` instead of the file's.

    The delays live in bits 12:9 of every word (`.side_set 1` takes bit 12 as
    the side-set bit and leaves 4 for the delay, docs/mvp-isa.md
    `### Delay/side-set field`), so retargeting is a rewrite of that field and
    nothing else.  Which value each word gets is decided from the instruction
    itself, not from its address, so the program's shape can move:

* a MOV Y,Y (`nop`) with side 1 is the clock-high instruction -> HIGH
     * a MOV Y,Y with side 0 is a clock-low instruction -> LOW
     * an IN or an OUT is the clock-low instruction of its loop -> LOW
     * the `set x, 1` at `wack` is the phase-boundary padding and gets max(0, LOW - 1),
       which is what makes the gap between the last rising edge of `tx` and the first of
       `ck` as long as every other bit period
    * a JMP whose target is its own address is the idle spin at `done`, and
       keeps the long delay that holds the engine there cheaply
     * everything else must have delay 0, and this raises if it does not

    That last rule is a guard as much as a rewrite: if a future edit puts a
    delay on a `jmp` or a `set`, the driver refuses to run instead of silently
    running at the wrong rate.
    """
    period_ticks(high, low)                       # validates the range
    unrolled = unrolled_period_words(program)
    words: List[int] = []
    for address, word in enumerate(program.instrs):
        op, a = word >> 13, (word >> 5) & 7
        field = (word >> 8) & 0x1F
        side, delay = field >> 4, field & 0xF
        # An unrolled bit period is two instructions where a loop body is three,
        # so its clock-low half carries one extra delay cycle and the two come
        # out the same length.  The DP only counts edges, but a period that was
        # one tick out would make the checker see a different bit rate on four
        # periods out of 46.
        tail = low + 1 if address in unrolled else low
        if address == program.labels["wack"]:      # the phase-boundary padding
            new = max(0, low - 1)
        elif op == 0b101 and a == 0b010:            # `nop` = mov y, y
            new = high if side else tail
        elif op in (0b010, 0b011):                # `in` / `out`
            new = tail
        elif op == 0 and a == 0 and (word & 0x1F) == address:   # `jmp` to itself
            new = IDLE_DELAY
        elif delay:
            raise SwdError(
                f"word {address} ({word:#06x}) carries delay {delay} on an instruction "
                f"that is not part of a bit period; retarget() only rewrites the two "
                f"loop bodies and would change what it does"
            )
        else:
            new = 0
        words.append((word & ~(0xF << 8)) | (side << 12) | (new << 8))
    return Program(
        name=program.name, instrs=words, origin=program.origin,
        wrap_target=program.wrap_target, wrap=program.wrap,
        sideset_bits=program.sideset_bits, sideset_opt=program.sideset_opt,
        sideset_pindirs=program.sideset_pindirs, labels=program.labels,
    )

# ---- the words one transfer needs --------------------------------------------
def request_word(header: int) -> int:
    """The TX word the request phase shifts out: the header, inverted.

    `out pindirs, 1` writes SWDIO's direction from an OSR bit, and the level
    register of an open-drain line stays 0, so an OSR bit of 1 pulls the line
    low and a 0 releases it (docs/mvp-isa.md `### Open drain`).  The wire bit
    of a request is 1 for start, park and every 1 of the header, so the OSR
    wants the header inverted -- the same inversion fw/i2c's driver applies.

    Bit 8 is 0 as well: it is the ninth period, which is the turnaround, and a
    turnaround has to leave SWDIO to the pull-up [S3].
    """
    return (~int(header)) & 0xFF


def data_word(write: bool, data: int = 0) -> int:
    """The TX word the data phase works from, inverted for the open-drain line.

    A write shifts it out through `tx`.  A read pulls no second word at all: its
    run length is `set x, 30` in the program, and the middle word of a transfer is
    the WDATA one only when there is data to send.
    """
    if not write:
        return 0
    return (~int(data)) & 0xFFFF_FFFF


def closing_word(write: bool, data: int = 0) -> int:
    """The TX word of the three closing periods, bits 2:0, as OSR bits.

    Period 44, 45 and 46 of a transfer (fw/swd/program.pio `p2`).  For a write
    the DP is reading WDATA throughout, so the first of the three is the parity
    bit and the other two pull the line low.  For a read the DP owns the first
    two: it drives its RDATA parity bit in 44 and releases for its trailing
    turnaround in 45, so both are 0 here and only the last one pulls the line
    low -- which is the clock with a low line the DP needs before it will look at
    another header (ADIv5 [S11]).
    """
    if not write:
        # 44: the DP drives its RDATA parity bit, so release; 45: its trailing
        # turnaround, so release; 46: pull the line low, which is the clock with a low
        # line the DP needs before it will look at another header.
        return 0b100
    # A write: the DP is reading WDATA right through, so the parity bit goes out in
    # the first of the three and the line is pulled low for the other two -- the first
    # of those is the clock with a low line it resynchronises on.
    return 0b110 | (1 - vswd.data_parity(data))


def transfer_words(header: int, write: bool, data: int = 0) -> List[int]:
    """The TX words of one transfer, in the order the program pulls them.

    A read pulls two (the request and the closing word); a write pulls three, the
    middle one being the WDATA.  Two fit in the four-deep FIFO with room to spare,
    which is what lets the host queue a whole transfer before starting it.
    """
    words = [request_word(header)]
    if write:
        words.append(data_word(write, data))
    words.append(closing_word(write, data))
    return words


# ---- the register words the program needs ------------------------------------
def execctrl_word(program: Program, flag_pin: int) -> int:
    """EXECCTRL: JMP_PIN, and the wrap window the program relies on.

    JMP_PIN is how `jmp pin wpre` reads FLAG.  There is no SIDE_EN and no
    SIDE_PINDIR: the program's `.side_set 1` carries no `opt`, so all one bit is
    data and every instruction side-sets it (docs/mvp-isa.md
    `### Delay/side-set field`).  The wrap window is what gives the `rx` loop its
    exit -- WRAP_BOTTOM is the `push iffull` behind `e1`, so falling off the end
    of the loop lands there -- and WRAP_TOP is the last word, so `done` is
    inside the window and the idle spin does not leave it.
    """
    if not 0 <= flag_pin <= 31:
        raise ValueError(f"FLAG pin {flag_pin} does not fit EXECCTRL.JMP_PIN (5 bits)")
    return ((flag_pin & 0x1F) << 24) | ((wrap_top(program) & 0x1F) << 12) | (
        (wrap_bottom(program) & 0x1F) << 7
    )


def wrap_bottom(program: Program) -> int:
    """WRAP_BOTTOM: where the `rx` loop's exit lands, the `push iffull` behind `e1`."""
    return program.labels["e1"] + 1


def wrap_top(program: Program) -> int:
    """WRAP_TOP: the `jmp x-- rx` that ends the receive loop, so falling off the end
    of it wraps instead of running into `wpre`."""
    return program.labels["rx"] + 2


def shiftctrl_word() -> int:
    """SHIFTCTRL: PULL_THRESH 31, PUSH_THRESH 1, shift out right, in left.

    AUTOPULL and AUTOPUSH stay off, so the program pulls and pushes explicitly
    and nothing happens behind its back.  Both shift directions are 1, which is
    what the register resets to (docs/mvp-isa.md `### Reset values` in
    docs/mvp-regmap.md) and what the wire order needs: OUT_SHIFTDIR 1 sends OSR
    bit 0 first, and IN_SHIFTDIR 1 shifts the samples down so that the k-th bit
    sampled lands in bit k.  Getting IN_SHIFTDIR wrong is invisible on the wire
    -- the samples are all still the right bits -- and hands the host the
    received word reversed.
    """
    return (PULL_THRESH << 25) | (PUSH_THRESH << 20) | (OUT_SHIFTDIR << 19) | (IN_SHIFTDIR << 18)


def pinctrl_word(swdio: int, swclk: int, flag: int) -> int:
    """PINCTRL: one window per pin, each one pin wide.

    The side-set writes SWCLK, OUT writes SWDIO's direction, IN samples SWDIO
    and SET writes FLAG's level.  None of the three windows may be wider than
    one pin: two of them would reach the other's pin, and `set pindirs` on
    SWDIO or `out pindirs` on SWCLK would drive a line the protocol does not own.
    """
    for name, pin in (("SWDIO", swdio), ("SWCLK", swclk), ("FLAG", flag)):
        if not 0 <= pin <= 31:
            raise ValueError(f"{name} pin {pin} does not fit a 5-bit window field")
    if len({swdio, swclk, flag}) != 3:
        raise ValueError(f"SWDIO, SWCLK and FLAG must be three different pins, got "
                         f"{swdio}, {swclk}, {flag}")
    return (
        (1 << 29)          # SIDESET_COUNT 1: SWCLK's level
        | (1 << 26)        # SET_COUNT 1: FLAG's level
        | (1 << 20)        # OUT_COUNT 1: SWDIO's direction
        | (swdio << 15)    # IN_BASE
        | (swclk << 10)    # SIDESET_BASE
        | (flag << 5)      # SET_BASE
        | swdio            # OUT_BASE
    )


def check_program(program: Program) -> None:
    """Fail loudly if program.pio is not the shape the driver was written for.

    The driver hard-codes three things the program could change without the
    driver noticing: that the side-set is one level bit on SWCLK, that the wrap
    window turns the `rx` loop's exit into the `push iffull` behind `e1`, and
    that `e2` is where the host's read/write flag is read.  Each of those is a
    word the driver addresses by label or by counting, so each is checked here
    rather than left to fail as a mysterious timeout.
    """
    if len(program.instrs) > 32:
        raise SwdError(f"the program needs {len(program.instrs)} words, the IMEM holds 32")
    if (program.sideset_bits, program.sideset_opt, program.sideset_pindirs) != (1, False, False):
        raise SwdError(
            f"the driver programs PINCTRL for `.side_set 1` with no `opt` and no `pindirs`, "
            f"program.pio says {program.sideset_bits} bit(s), opt={program.sideset_opt}, "
            f"pindirs={program.sideset_pindirs}: the side-set bit is SWCLK's level"
        )
    for name in ("xfer", "tx", "e1", "p2", "done", "wack", "ck", "e2", "wpre", "rx"):
        if name not in program.labels:
            raise SwdError(f"program.pio has no `{name}` label for the driver to address")
    if (program.instrs[xfer_of(program) + 1] >> 13, (program.instrs[xfer_of(program) + 1] >> 5) & 7,
            program.instrs[xfer_of(program) + 1] & 0x1F
            ) != (0b111, 0b001, REQUEST_PERIODS - 1):
        raise SwdError(
            f"word {xfer_of(program) + 1} is not `set x, {REQUEST_PERIODS - 1}`, the counter "
            f"of the {REQUEST_PERIODS} request periods"
        )
    e1, p2 = program.labels["e1"], program.labels["p2"]
    rx = program.labels["rx"]
    if program.instrs[e1] >> 13 != 0 or ((program.instrs[e1] >> 5) & 7) != 0b111:
        raise SwdError(f"word {e1} (`e1`) is not a `jmp !osre`, which is how the request run "
                       f"is told from the WDATA run")
    e2 = program.labels["e2"]
    if (program.instrs[e2] >> 13, (program.instrs[e2] >> 5) & 7) != (0, 0b110):
        raise SwdError(f"word {e2} (`e2`) is not a `jmp pin`, which is where the host's "
                       f"read/write flag is read")
    if (program.instrs[rx] >> 13) != 0b101 or ((program.instrs[rx] >> 5) & 7) != 0b010:
        raise SwdError(f"word {rx} (`rx`) is not the `nop side 1` that starts the clock")
    if ((program.instrs[rx + 1] >> 13), (program.instrs[rx + 1] >> 5) & 7, program.instrs[rx + 1]
            & 0x1F) != (0b010, 0b000, 1):
        raise SwdError(f"word {rx + 1} is not the `in pins, 1` that samples SWDIO")
    last = program.instrs[rx + 2]
    if (last >> 13, (last >> 5) & 7, last & 0x1F) != (0, 0b010, rx):
        raise SwdError(
            f"word {rx + 2} is not the `jmp x-- rx` that ends the receive loop; the "
            f"driver's wrap window turns the loop's exit into the `push iffull` at {e1 + 1}"
        )
    if program.labels["wpre"] <= rx + 2:
        raise SwdError(
            f"`wpre` is at {program.labels['wpre']}, at or before the end of the receive "
            f"loop at {rx + 2}: the loop's exit is the wrap, so nothing may follow it "
            f"inside the wrap window"
        )
    if (program.instrs[e1 + 1] >> 13) != 0b100 or ((program.instrs[e1 + 1] >> 5) & 7) != 0b011:
        raise SwdError(f"word {e1 + 1} is not `push iffull`, the wrap target and the only "
                       f"place the RDATA word reaches the host")
    if p2 != e1 + 2:
        raise SwdError(f"`p2` is at {p2}, expected {e1 + 2}: `e1`'s fall-through is the "
                       f"closing phase")
    wack, ck = program.labels["wack"], program.labels["ck"]
    if (program.instrs[wack] >> 13, (program.instrs[wack] >> 5) & 7,
            program.instrs[wack] & 0x1F) != (0b111, 0b001, ACK_PERIODS - 1):
        raise SwdError(
            f"word {wack} (`wack`) is not `set x, {ACK_PERIODS - 1}`, the counter of the "
            f"{ACK_PERIODS} ACK periods the program clocks"
        )
    if (program.instrs[wack + 1] >> 13, program.instrs[wack + 1] & 0x1F) != (0, ck):
        raise SwdError(f"word {wack + 1} is not the `jmp ck` that starts the ACK")
    if (program.instrs[e2 + 1] >> 13, (program.instrs[e2 + 1] >> 5) & 7,
            program.instrs[e2 + 1] & 0x1F) != (0b111, 0b001, RDATA_PERIODS - 1):
        raise SwdError(
            f"word {e2 + 1} is not `set x, {RDATA_PERIODS - 1}`, the counter of the "
            f"{RDATA_PERIODS} periods the receive loop samples"
        )
    wpre = program.labels["wpre"]
    if (program.instrs[wpre + 2] >> 13, (program.instrs[wpre + 2] >> 5) & 7) != (0b100, 0b101):
        raise SwdError(f"word {wpre + 2} is not the `pull block` of the WDATA run")
    if (program.instrs[wpre + 3] >> 13, (program.instrs[wpre + 3] >> 5) & 7,
            program.instrs[wpre + 3] & 0x1F) != (0b111, 0b001, WDATA_TX_PERIODS - 1):
        raise SwdError(
            f"word {wpre + 3} is not `set x, {WDATA_TX_PERIODS - 1}`, the counter of the "
            f"{WDATA_TX_PERIODS} WDATA periods `tx` sends before `p2` finishes the frame"
        )
    if (program.instrs[wpre] >> 13, (program.instrs[wpre] >> 5) & 7) != (0b101, 0b010):
        raise SwdError(f"word {wpre} (`wpre`) is not the clock-high `nop` of the "
                       f"ACK/WDATA turnaround")


@dataclass(frozen=True)
class Config:
    """One SWD setup: which engine, which pins and how long a bit period is."""

    high: int = 6
    low: int = 0
    engine: int = 0
    swdio: int = DEFAULT_SWDIO_PIN
    swclk: int = DEFAULT_SWCLK_PIN
    flag: int = DEFAULT_FLAG_PIN

    @property
    def period(self) -> int:
        """The bit period in core cycles, which is also CLKDIV."""
        return period_ticks(self.high, self.low)

    @property
    def clkdiv(self) -> int:
        return clkdiv_word(self.period)

    @property
    def swclk_hz(self) -> float:
        return swclk_hz(self.period)

    @property
    def frame_ticks(self) -> int:
        """Core cycles one transfer takes on the wire, ignoring host overhead."""
        return READ_PERIODS * self.period


class SwdDriver:
    """Load fw/swd/program.pio into one engine and run SWD transfers with it."""

    def __init__(
        self,
        bus,
        device,
        program: Optional[Program] = None,
        high: int = 6,
        low: int = 0,
        engine: int = 0,
        swdio: int = DEFAULT_SWDIO_PIN,
        swclk: int = DEFAULT_SWCLK_PIN,
        flag: int = DEFAULT_FLAG_PIN,
        path: Path | str = PROGRAM_PATH,
    ) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        if not 0 <= swdio <= 7:
            raise ValueError(
                f"SWDIO pin {swdio} is not a uio pin 0..7: open drain is a direction, and "
                f"only the bidirectional pins have one (docs/mvp-isa.md `## Pins`)"
            )
        if len({swdio, swclk, flag}) != 3:
            raise ValueError(
                f"SWDIO, SWCLK and FLAG must be three different pins, got "
                f"{swdio}, {swclk}, {flag}: each window is one pin wide, so a shared pin "
                f"would be written by two of them"
            )
        for name, pin in (("SWCLK", swclk), ("FLAG", flag)):
            if not 0 <= pin <= 31:
                raise ValueError(f"{name} pin {pin} does not fit a 5-bit window field")
        self.bus, self.dev = bus, device
        self.engine = engine
        base = program if program is not None else load_program(path)
        self.config = Config(high=high, low=low, engine=engine, swdio=swdio,
                             swclk=swclk, flag=flag)
        self.prog = retarget(base, high, low)
        check_program(self.prog)
        self._xfer = self.prog.labels["xfer"]
        self._done = self.prog.labels["done"]
        self._flag = flag
        # Forced instructions the line reset needs, taken from the program so the
        # two cannot drift: the clock halves and the one `out pindirs, 1`, each
        # with its delay cleared, because a host-forced instruction ignores its
        # delay (docs/mvp-isa.md `### EXEC`).
        tx, ck = self.prog.labels["tx"], self.prog.labels["ck"]
        self._nop_high = self.prog.instrs[tx] & ~(0xF << 8)
        self._nop_low = self.prog.instrs[ck + 1] & ~(0xF << 8)
        self._out_dir = self.prog.instrs[tx + 1] & ~(0xF << 8)
        self.transfers = 0

    # ---- the register words this configuration asks for ------------------------
    @property
    def period(self) -> int:
        return self.config.period

    @property
    def clkdiv_word(self) -> int:
        return self.config.clkdiv

    @property
    def execctrl(self) -> int:
        return execctrl_word(self.prog, self._flag)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_word()

    @property
    def pinctrl(self) -> int:
        return pinctrl_word(self.config.swdio, self.config.swclk, self._flag)

    @property
    def words_used(self) -> int:
        return len(self.prog.instrs)

    @property
    def sm(self) -> int:
        """Word address of this engine's `SMe_CLKDIV`."""
        return A_SM + 6 * self.engine

    # ---- host accesses: queued on the ChipDevice, one per core cycle ----------
    def reg_write(self, addr: int, data: int) -> None:
        """Queue a register write; it is performed on the next cycle we step."""
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def reg_read(self, addr: int) -> int:
        """Queue a register read and step the bus until the answer is back."""
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.step()
            ticks += 1
            if ticks > PHASE_LIMIT:
                raise SwdError(f"read of word address {addr:#04x} never came back")
        return int(future.result())

    def step(self) -> None:
        """One core cycle: the ChipDevice performs at most one queued access."""
        self.bus.step()

    def run(self, ticks: int) -> None:
        for _ in range(ticks):
            self.step()

    def wait_until(self, what, limit: int = PHASE_LIMIT) -> None:
        ticks = 0
        while not what():
            self.step()
            ticks += 1
            if ticks > limit:
                raise SwdError(f"timed out after {ticks} core cycles")

    # ---- register reads --------------------------------------------------------
    def pc(self) -> int:
        """`SMx_ADDR`, the engine's PC.

        There is no IRQ in the MVP, so this and the FIFO levels are all the host
        can see of the program's progress.  `done` is the idle spin, so PC ==
        `done` means "parked".
        """
        return self.reg_read(self.sm + SM_ADDR) & 0x1F

    @property
    def parked(self) -> bool:
        return self.pc() == self._done

    def wait_parked(self) -> None:
        self.wait_until(lambda: self.parked)

    def fdebug(self) -> int:
        return self.reg_read(A_FDEBUG)

    def clear_fdebug(self) -> None:
        self.reg_write(A_FDEBUG, 0xFFFF_FFFF)      # FDEBUG is W1C
        self.run(2)

    def tx_level(self) -> int:
        return (self.reg_read(A_FLEVEL) >> FLEVEL_TX0) & 0xF

    def rx_level(self) -> int:
        return (self.reg_read(A_FLEVEL) >> FLEVEL_RX0) & 0xF

    def padout(self) -> int:
        return self.reg_read(A_DBG_PADOUT)

    def padoe(self) -> int:
        return self.reg_read(A_DBG_PADOE)

    # ---- the startup sequence (docs/mvp-regmap.md `### Typical host sequences`)
    def load(self) -> None:
        """Steps 1-6: stop the engine, burst-write the program, set the registers,
        then enable it and let it reach its idle spin."""
        self.reg_write(A_CTRL, 0)                                    # 1. stop
        for k, word in enumerate(self.prog.instrs):                  # 2. IMEM burst
            self.reg_write(A_IMEM[self.engine] + k, word)
        self.reg_write(self.sm + SM_CLKDIV, self.clkdiv_word)        # 3. the four registers
        self.reg_write(self.sm + SM_EXECCTRL, self.execctrl)
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl)
        self.reg_write(self.sm + SM_PINCTRL, self.pinctrl)
        self.run(2)
        # 4. the entry point.  PC is 0 after a reset, and 0 is `xfer`, whose first
        # instruction is a blocking `pull` -- an engine started there would sit on
        # FDEBUG.TXSTALL until the first transfer.  `SMx_ADDR` is read-only, so the
        # only way in is a forced JMP (docs/mvp-isa.md `### Enable, restart`), and
        # it has to happen before the enable: a host-forced instruction runs in the
        # next core cycle whatever the divider says.
        self.force(jmp_abs_word(self._done))
        # 5-6. enable with a restart, so the ISR and osr_cnt are at their reset
        # values and the divider's phase is known.
        e = self.engine
        self.reg_write(A_CTRL, (1 << (8 + e)) | (1 << (4 + e)) | (1 << e))
        self.wait_parked()

    def stop(self) -> None:
        """Disable the engine, keeping every register."""
        ctrl = self.reg_read(A_CTRL)
        self.reg_write(A_CTRL, ctrl & ~(0x011 << self.engine))
        self.run(2)

    # ---- forcing instructions through the EXEC latch ---------------------------
    def force(self, word: int) -> None:
        """Queue one instruction into `SMx_INSTR`.

        It runs in the next core cycle whatever SM_ENABLE and the divider say,
        it does not advance the PC and its delay is ignored (docs/mvp-isa.md
        `### EXEC`), which is why the line reset below has to queue one
        instruction per core cycle.
        """
        self.reg_write(self.sm + SM_INSTR, word & 0xFFFF)

    @property
    def _pull_block(self) -> int:
        """The program's own `pull block`, reused by the line reset."""
        return self.prog.instrs[self._xfer] & 0xFFFF

    # ---- the TX FIFO ----------------------------------------------------------
    def push(self, word: int) -> None:
        """Queue one TX word, waiting for room first so no word is dropped."""
        while self.tx_level() >= regmap.fifo_full_level():
            self.step()
        self.reg_write(A_TXF0 + self.engine, word & 0xFFFF_FFFF)
        self.run(1)

    def drain_rx(self) -> int:
        """Throw away whatever the RX FIFO holds and return how many words went.

        A transfer that received data leaves one word behind, and a line reset leaves
        one too: both run the receive loop, which fills the ISR and pushes it.  The
        driver reads the word it wants after each transfer rather than leaving it, so
        this is only for the paths that do not want it.
        """
        dropped = 0
        while self.rx_level():
            self.reg_read(A_RXF0 + self.engine)
            dropped += 1
        return dropped

    def pop_rx(self) -> int:
        """Read one word out of the RX FIFO."""
        while not self.rx_level():
            self.step()
        return self.reg_read(A_RXF0 + self.engine)

    # ---- one transfer ---------------------------------------------------------
    def transfer(self, header: int, write: bool, data: int = 0,
                 bad_parity: bool = False) -> int:
        """Run one SWD frame and return the 32 bits the chip received.

        `header` is the request byte as `verif.vip.swd.header_byte` builds it,
        parity bit included.  For a read the return value is the RDATA word the
        program sampled into the ISR and pushed; for a write it is 0, because the ISR
        stays empty and nothing is pushed.  The ACK is clocked but not sampled
        (fw/swd/LIMITS.md), so `verif.vip.swd.SwdChecker` is what says whether the DP
        answered.

        `bad_parity` flips the WDATA parity bit.  It is the one fault the driver can
        inject from the host side, and it is what a test needs to show the DP noticing
        a corrupted write [S10]: the write is not applied and the WDATAERR sticky bit
        comes up in the next CTRL/STAT read.
        """
        self.wait_parked()
        self.drain_rx()                # a line reset or an earlier read may have left one
        words = transfer_words(header, write, data)
        if bad_parity:
            if not write:
                raise ValueError("bad_parity is a WDATA fault; it needs write=True")
            words[-1] ^= 1
        for word in words:
            self.push(word)
        # Two forced instructions: the flag `jmp pin` reads at `e2`, then the jump
        # into the program.  Queued back to back so they land on consecutive
        # cycles; a host-forced instruction does not advance the PC, so the first
        # one leaves the engine parked in `done` and the second leaves it at `xfer`.
        self.force(set_pins_word(write))
        self.force(self._xfer)
        self.run(4)                       # let both forced cycles execute
        self.wait_parked()
        self.transfers += 1
        if write:
            return 0
        return self.pop_rx()

    def read(self, addr: int, ap: bool = False) -> int:
        """One DP or AP register read."""
        header = vswd.header_byte(ap=ap, read=True, addr=addr)
        return self.transfer(header, write=False)

    def write(self, addr: int, value: int, ap: bool = False,
              bad_parity: bool = False) -> None:
        """One DP or AP register write.  Validity is confirmed by the next read."""
        header = vswd.header_byte(ap=ap, read=False, addr=addr)
        self.transfer(header, write=True, data=value & 0xFFFF_FFFF, bad_parity=bad_parity)

    # ---- the line reset -------------------------------------------------------
    def line_reset(self, passes: int = 2) -> None:
        """Clock SWCLK with SWDIO released for 50-odd periods, then one clock with it low.

        Fifty clocks with SWDIO high is a line reset [S12]: the DP's registers go back
        to their power-on values and from then on only an IDCODE read is answered
        [S13], so the next transfer has to be one.  `passes` runs of `xfer` with a word
        of zeros clock `L_RESET_PERIODS` released periods each, and the last pass asks
        `p2` for a low clock as well, which is what brings the DP out of its reset state
        (ADIv5 [S11]).

        A line reset needs no program of its own: `xfer` with a zero word leaves SWDIO
        released for every period it clocks -- the request bits are all "released", the
        ACK loop does not touch the line, the receive loop only samples it -- so two
        passes are more than the 50 the spec asks for.  The first driver version instead
        clocked the reset from the host with one SMx_INSTR write per core cycle, which
        is ~500 host accesses for a 50-clock reset; see fw/swd/LIMITS.md.
        """
        if passes < 2:
            raise ValueError(
                f"a line reset is 50 clocks and one pass is {L_RESET_PERIODS} of them, so it "
                f"takes at least two (got {passes})"
            )
        for run in range(passes):
            last = run == passes - 1
            self.wait_parked()
            self.push(0)                              # every OSR bit 0: `out pindirs, 1`
            self.push(L_RESET_LOW_BITS if last else 0)  # releases, or releases then pulls
            self.force(set_pins_word(0))              # read: the flag selects `rx`, which
            self.force(self._xfer)                    # only samples, never drives
            self.run(4)
            self.wait_parked()


L_RESET_LOW_BITS = 0b100          # `p2`'s closing word for a line reset: two released
                                   # periods and then a pull of SWDIO low, which is what
                                   # takes the DP out of its reset state
L_RESET_PERIODS = 46              # released periods one `xfer` pass clocks with a zero word


def set_pins_word(value: int) -> int:
    """`set pins, value`: opcode 111, destination PINS (000), data in bits 4:0.

    The destination is the SET window, which the driver points at the FLAG pin,
    so this is how the host tells the program whether the transfer is a write.
    A forced instruction ignores its delay and does not advance the PC
    (docs/mvp-isa.md `### EXEC`), which is what makes it safe to use while the
    engine sits in the idle spin.
    """
    if not 0 <= value <= 31:
        raise ValueError(f"`set pins, {value}`: the data field is 5 bits")
    return (0b111 << 13) | (0b000 << 5) | value


def jmp_abs_word(address: int) -> int:
    """`jmp address` with no condition, no side-set and no delay."""
    if not 0 <= address <= 31:
        raise ValueError(f"JMP address {address} does not fit in 5 bits")
    return address & 0x1F

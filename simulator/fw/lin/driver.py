"""Host side of the MVP LIN firmware: load program.pio, set up one engine, send a frame.

Everything here is written against `docs/mvp-regmap.md` (register word
addresses, the `### Typical host sequences` startup order and the per-engine fields) and
against `docs/mvp-isa.md` (divider, side-set, autopull, open drain).  Host accesses go through
`iss.pio_iss.ChipDevice`'s write/read queue, one per tick, so every wait here is the
simulation running -- the same thing a real host does over the SPI port of
`docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                     # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}")                      # open drain: pullup=1, push_pull=False
    dev = bus.attach(ChipDevice())
    drv = LinDriver(dev, bus, pin=0, rate=19200)
    drv.load()                 # IMEM, CLKDIV, EXECCTRL, SHIFTCTRL, PINCTRL
    drv.start(0x11)            # header only: a responder on the bus answers it
    drv.start(0x22, b"\x01\x02\x03\x04")   # ... or the chip publishes the response
    bus.run(drv.frame_ticks(19200, 4) + 40_000)
    drv.stop()

The chip core clock is 50 MHz: one `verif.vip.core.Bus` tick at `ns_per_tick = 20`.

**The chip is the commander** (LIN 1.x: the master), and that is a decision the ISA forces
rather than a preference; `fw/lin/README.md` carries the argument.  A commander is a pure
bit-bang transmitter: a fixed waveform whose every field is a byte out of the TX FIFO, so the
whole frame is 26 instructions and no arithmetic.  A responder needs three things the MVP does
not have: it has to *measure* the commander's bit period off the sync field (there is no clock
recovery, only a divider the host sets once), it has to decide from a subscription table
whether to answer (a JMP condition can test `X`, `Y`, `X != Y` or one pin -- not the six bits
of a frame identifier), and it has to compute the checksum, which is an inverted modulo-256
sum of up to eight bytes plus the PID.  There is no ADD instruction in `docs/mvp-isa.md`
`## Encoding`, so that sum cannot be formed in a state machine at all.  The parity bits of the
protected identifier are the same story, which is why `make_pid()` and `checksum()` below are
host code: the chip transmits what the host computed.

Three things about this chip that an RP2040 driver does not have to think about, and that this
module exists to get right:

* **`SIDESET_COUNT = 2` with `EXECCTRL.SIDE_EN = 1` and `SIDE_PINDIR = 1`.**  The reset
  `PINCTRL` is `SIDESET_COUNT = 2, SIDE_EN = 0`, and then *every* instruction side-sets
  `SIDESET_COUNT` pins with data taken from the top bits of the side-set field -- which would
  hold the LIN wire low for ever.  With `SIDE_EN = 1` the top side-set bit is an enable, the
  program gets to say "no side-set at all" on an instruction (which is how `out pindirs, 1`
  owns the pin for a data bit, side-set having priority over the OUT channel), and
  `SIDE_PINDIR = 1` makes the side-set write pin *directions*, which is what open drain needs.
* **The host inverts every byte it pushes.**  `out pindirs, 1` writes a *direction*, and
  direction 1 drives the line to its level register's reset value 0, i.e. dominant.  So bit 0
  of a pushed word is 1 for a zero data bit: `wire_word()` is the one place that inversion
  happens, and it is undone on the wire by the receiver.
* **No IRQ exists**, so a commander with nothing left to say parks in `idle` (`jmp idle`) and
  the host reads where it is out of `SMe_ADDR` (`docs/mvp-isa.md` `## Differences from
  RP2040` item 3).
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from asm.pioasm import Program, assemble

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "lin"            # the .program name inside program.pio
SYNC_BYTE = 0x55              # LIN 2.2A 2.3.1.2
DIAGNOSTIC_IDS = (0x3C, 0x3D)  # "shall always use classic checksum", 2.3.1.5
CLUSTER_VERSIONS = ("1.3", "2.2")

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

# Instruction encodings used by the host (docs/mvp-isa.md `## Encoding`).
OP_JMP, OP_SET = 0b000, 0b111
JMP_ALWAYS = 0b000
SET_DEST_X, SET_DEST_Y = 0b001, 0b010


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def clkdiv_for_rate(rate: float, sm_cycles_per_bit: int = 1) -> int:
    """The CLKDIV word that puts one bit time at `sm_cycles_per_bit` SM cycles.

    One bit must last CORE_HZ / rate core cycles and one SM cycle lasts one CLKDIV period, so
    the divisor wanted is CORE_HZ / (rate * sm_cycles_per_bit), rounded into INT and FRAC
    (`docs/mvp-isa.md` `### Clock divider`).  The divider is exact on average, so the bit grid
    a commander puts on the wire drifts by less than one core cycle per SM cycle, which at
    every LIN rate is four orders of magnitude inside FTOL_SYNC's 1.5 %.
    """
    if rate <= 0:
        raise ValueError(f"the LIN bit rate must be positive, got {rate!r}")
    if sm_cycles_per_bit < 1:
        raise ValueError(f"a bit needs at least one SM cycle (got {sm_cycles_per_bit})")
    want = CORE_HZ / (rate * sm_cycles_per_bit)
    if want > 0xFFFF:
        raise ValueError(
            f"{rate} bit/s needs a CLKDIV INT of {want:.1f}, above the 16-bit maximum"
        )
    integer = max(1, int(want))
    frac = round((want - integer) * 256)
    if frac > 0xFF:                       # rounding pushed it past the next integer
        integer, frac = integer + 1, 0
    return clkdiv_value(integer, frac)


def ticks_per_bit(clkdiv: int, sm_cycles_per_bit: int = 1) -> float:
    """The average bit length in core cycles (ticks) a CLKDIV word produces."""
    integer = (clkdiv >> 16) & 0xFFFF or 0x10000
    return sm_cycles_per_bit * (integer + ((clkdiv >> 8) & 0xFF) / 256.0)


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address (`## Encoding` of the ISA doc)."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (OP_JMP << 13) | (JMP_ALWAYS << 5) | addr


def set_abs(dest: int, data: int) -> int:
    """`set DEST, DATA` for one of the counter registers, as the host forces it.

    The host has no way to write X or Y except a forced instruction
    (`docs/mvp-isa.md` `### Enable, restart`), and a forced instruction does not advance the
    PC, so the counters can be set at any time without disturbing the program.
    """
    if dest not in (SET_DEST_X, SET_DEST_Y):
        raise ValueError(f"SET destination {dest:03b} is not X or Y")
    if not 0 <= data <= 0x1F:
        raise ValueError(f"SET data {data!r} does not fit in 5 bits")
    return (OP_SET << 13) | (dest << 5) | data

# --------------------------------------------------------------------------------------------
# The protocol arithmetic, in host code
#
# This is a second implementation of LIN 2.2A's two equations, written here rather than
# imported from `verif/vip/lin.py` on purpose: the checker decides pass or fail with
# `make_pid()` and `lin_checksum()`, so a firmware that built its frames with the same
# functions could only agree with the judge by construction.  `fw/lin/test_lin.py` pins both
# implementations against the identifiers and checksums the LIN literature publishes, and
# against each other for all 64 identifiers.
# --------------------------------------------------------------------------------------------

def parity_bits(frame_id: int) -> tuple[int, int]:
    """P0 and P1 of the protected identifier, from LIN 2.2A eq. (1) and (2).

        P0 = ID0 ^ ID1 ^ ID2 ^ ID4
        P1 = not(ID1 ^ ID3 ^ ID4 ^ ID5)

    P0 is the even-parity bit and P1 the odd-parity one; they travel in bits 6 and 7 of the
    byte, so ID 0x00 is the PID 0x80 and ID 0x02 the PID 0x40.
    """
    if not 0 <= frame_id <= 0x3F:
        raise ValueError(f"a LIN frame identifier is 6 bits, got {frame_id:#x}")
    p0 = ((frame_id >> 0) ^ (frame_id >> 1) ^ (frame_id >> 2) ^ (frame_id >> 4)) & 1
    p1 = 1 ^ (((frame_id >> 1) ^ (frame_id >> 3) ^ (frame_id >> 4) ^ (frame_id >> 5)) & 1)
    return p0, p1


def make_pid(frame_id: int) -> int:
    """The protected identifier byte for `frame_id`: the ID with P0 and P1 in bits 7 and 6."""
    p0, p1 = parity_bits(frame_id)
    return frame_id | (p0 << 6) | (p1 << 7)


def checksum_model(frame_id: int, version: str = "2.2") -> str:
    """Which checksum model a frame identifier uses on a `version` cluster.

    0x3C and 0x3D "shall always use classic checksum" (LIN 2.2A 2.3.1.5), and a LIN 1.3
    cluster uses the classic one for every frame.
    """
    if version not in CLUSTER_VERSIONS:
        raise ValueError(f"unknown LIN version {version!r}; pick one of {CLUSTER_VERSIONS}")
    return "classic" if frame_id in DIAGNOSTIC_IDS or version == "1.3" else "enhanced"


def lin_checksum(pid: int, data, *, enhanced: bool = True) -> int:
    """The response checksum: the inverted modulo-256 sum of the data bytes (and the PID).

    `enhanced` is what the model means by "enhanced" (LIN 2.0 onwards): the protected
    identifier joins the sum.  The carry is wrapped back in, so {0xFF, 0x02} sums to 0x01.
    """
    total = sum(data) & 0xFF
    if enhanced:
        total = (total + (pid & 0xFF)) & 0xFF
    return (~total) & 0xFF


def wire_word(byte: int) -> int:
    """One byte as the host pushes it: inverted for the direction register.

    `out pindirs, 1` writes a *direction*, and direction 1 drives the line to the level
    register's reset value 0, which is the dominant (0) LIN level.  So the word's bit 0 is 1
    for a zero data bit and 0 for a one.  `OUT_SHIFTDIR = 1` sends bit 0 first, and LIN sends
    its data bits least significant first, so the inversion is bit for bit and nothing else
    has to be reversed.
    """
    if not 0 <= byte <= 0xFF:
        raise ValueError(f"a byte is 0..255 (got {byte!r})")
    return (~byte) & 0xFF


def header_words(frame_id: int) -> list[int]:
    """The two header byte fields, in the order they go out: 0x55 then the PID."""
    return [wire_word(SYNC_BYTE), wire_word(make_pid(frame_id))]


def response_words(frame_id: int, data, version: str = "2.2") -> list[int]:
    """The response byte fields: the data bytes, then the checksum the identifier calls for.

    The checksum is *not* optional and cannot be empty: a response carries 1..8 data bytes and
    then the checksum byte (LIN 2.2A 2.3.1.4/2.3.1.5), so the fields are `len(data) + 1` and
    the Y counter the program decrements is exactly that.
    """
    payload = bytes(data)
    if not 1 <= len(payload) <= 8:
        raise ValueError(f"a LIN response carries 1..8 data bytes, got {len(payload)}")
    pid = make_pid(frame_id)
    model = checksum_model(frame_id, version)
    csum = lin_checksum(pid, payload, enhanced=(model == "enhanced"))
    return [wire_word(b) for b in payload] + [wire_word(csum)]


def frame_words(frame_id: int, data=None, version: str = "2.2") -> list[int]:
    """Every word one frame puts on the wire, header first.

    `data is None` means the header only: the chip is a commander that leaves the wire to a
    responder, which is the usual LIN arrangement (the node that owns the schedule table also
    owns the data).  A `data` of 1..8 bytes makes the chip the publisher of the response as
    well, which is legal for an unconditional frame.
    """
    words = header_words(frame_id)
    if data is not None:
        words += response_words(frame_id, data, version)
    return words


def wire_bytes(data) -> bytes:
    """The bytes as they went out, for a host that wants to log the wire image."""
    return bytes(~w & 0xFF for w in data)


# --------------------------------------------------------------------------------------------
# The program: assemble it, and read back what the words say about the timing
# --------------------------------------------------------------------------------------------

def load_program(path: Path | str = PROGRAM_PATH) -> Program:
    """Assemble program.pio (asm/pioasm.py) and return its one program."""
    programs = assemble(Path(path).read_text(encoding="utf-8"))
    names = [p.name for p in programs]
    if names != [PROTO_NAME]:
        raise ValueError(f"{path}: expected one .program named {PROTO_NAME}, got {names}")
    return programs[0]


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (`docs/mvp-isa.md` `### Engine state`)."""
    return len(program.instrs)


@dataclass(frozen=True)
class Word:
    """One assembled word, split the way the driver needs to read it.

    `sideset` is the whole side-set field (the enable bit included, which is where the
    assembler puts it: `asm/pioasm.py` sets the enable itself for any non-zero operand), and
    `delay` is the low `5 - SIDESET_COUNT` bits of the delay/side-set field, so the split
    follows `docs/mvp-isa.md` `### Delay/side-set field` rather than a guess.
    """
    word: int
    op: int
    sideset: int
    delay: int
    low: int
    opt: bool

    @property
    def side_enabled(self) -> bool:
        """Whether this instruction side-sets at all (False when the `opt` enable is 0)."""
        return not self.opt or bool(self.sideset)

    @property
    def side_data(self) -> int:
        """The side-set data bit: 1 = drive the pin (dominant), 0 = release it."""
        return self.sideset & 1

    @property
    def sm_cycles(self) -> int:
        """How many SM cycles this instruction takes on its own: 1 + delay."""
        return 1 + self.delay


def decode(program: Program, word: int) -> Word:
    """Split one word into opcode, side-set, delay and the instruction's own fields."""
    sc = min(program.sideset_bits, 5)
    field = (word >> 8) & 0x1F
    return Word(word=word, op=word >> 13,
                sideset=(field >> (5 - sc)) if sc else 0,
                delay=field & ((1 << (5 - sc)) - 1),
                low=word & 0xFF, opt=program.sideset_opt)


def is_out_pindirs_1(w: Word) -> bool:
    """True for `out pindirs, 1`, the one instruction a data bit is."""
    return w.op == 0b011 and (w.low >> 5) == 0b100 and (w.low & 0x1F) == 1


def is_pull_block(w: Word) -> bool:
    """True for a bare `pull block`: PULL, IfEmpty = 0, Block = 1."""
    return w.op == 0b100 and (w.low & 0x80) and not (w.low & 0x40) and (w.low & 0x20)


def is_nop(w: Word) -> bool:
    """True for `nop` (= `mov y, y`), the break and delimiter fields."""
    return w.op == 0b101 and (w.low >> 5) == 0b010 and (w.low & 0x07) == 0b010 \
        and (w.low >> 3) & 0b11 == 0


def jmp_cond(w: Word) -> int:
    """The JMP condition field, bits 7:5."""
    return (w.low >> 5) & 0b111


def jmp_target(w: Word) -> int:
    """The JMP address field, bits 4:0 (absolute IMEM index)."""
    return w.low & 0x1F


@dataclass(frozen=True)
class ProgramTiming:
    """The frame's shape in bit times, read back out of the assembled words.

    Nothing here is written down by hand: `program_timing()` walks the words and refuses a
    program whose shape the driver's clocking does not describe, which is the one thing that
    would silently put a wrong waveform on the wire.
    """
    sm_cycles_per_bit: int
    break_bit_times: int
    delimiter_bit_times: int
    field_bit_times: int
    response_space_bit_times: int
    idle_addr: int

    @property
    def header_fields(self) -> int:
        """Byte fields in the header: the sync field and the protected identifier."""
        return 2

    def header_bit_times(self) -> int:
        """What the checker times as the header: break, delimiter and both byte fields."""
        return (self.break_bit_times + self.delimiter_bit_times
                + self.header_fields * self.field_bit_times)

    def frame_bit_times(self, n_data: Optional[int] = None) -> int:
        """Whole frame length, break field to the last byte field.

        `n_data is None` is a header-only frame: it ends at the PID's stop bit, which is where
        a responder's response space begins.
        """
        total = self.header_bit_times()
        if n_data is not None:
            total += self.response_space_bit_times + (n_data + 1) * self.field_bit_times
        return total

    def frame_ticks(self, clkdiv: int, n_data: Optional[int] = None) -> int:
        """`frame_bit_times` in core cycles, for sizing a `bus.run()`."""
        return int(self.frame_bit_times(n_data) * ticks_per_bit(clkdiv))


def _field_block(ws: list[Word], i: int, cond: int, what: str) -> tuple[int, int]:
    """One byte field in the words: start bit, eight data bits, stop bit. -> (SM cycles, next).

    The block is `pull block` (whose side-set is the start bit), eight `out pindirs, 1` whose
    side-set field is zero -- so that the OUT channel owns the pin -- and a JMP with condition
    `cond` whose side-set is the stop bit and whose target is the block's own first word.
    """
    if i >= len(ws) or not is_pull_block(ws[i]) or not ws[i].side_data:
        raise ValueError(f"{what}: expected `pull block` with a dominant side-set at word {i}")
    cycles, start = ws[i].sm_cycles, i
    i += 1
    data = []
    while i < len(ws) and is_out_pindirs_1(ws[i]):
        if ws[i].side_enabled:
            raise ValueError(
                f"{what}: the data bit at word {i} has a side-set operand, and side-set wins "
                f"over the OUT channel in the same cycle, so it would overwrite the very bit "
                f"the OSR asked for (docs/mvp-isa.md `### Pin writes`)")
        data.append(ws[i])
        cycles += ws[i].sm_cycles
        i += 1
    if len(data) != 8:
        raise ValueError(f"{what}: a byte field has 8 data bits, word {i} has {len(data)}")
    if len({d.delay for d in data}) != 1:
        raise ValueError(
            f"{what}: the data bits have different delays {sorted({d.delay for d in data})}, "
            f"so one bit is not a whole number of SM cycles and CLKDIV cannot set the rate")
    if i >= len(ws) or ws[i].op != 0b000 or jmp_cond(ws[i]) != cond \
            or jmp_target(ws[i]) != start or ws[i].side_data:
        raise ValueError(
            f"{what}: word {i} is not the field's own `jmp {'x--' if cond == 2 else 'y--'}` "
            f"stop-bit branch back to word {start}")
    return cycles + ws[i].sm_cycles, i + 1


def program_timing(program: Program) -> ProgramTiming:
    """Read the frame's shape out of the assembled words, or say why it cannot be read.

    The program is walked in the order it runs: the break field, the break delimiter, the
    header's byte field, the branch that decides whether the response is published, the idle
    loop the header-only case falls into, the response's byte field and the jump back.
    """
    ws = [decode(program, w) for w in program.instrs]
    i = 0
    brk = 0
    while i < len(ws) and is_nop(ws[i]) and ws[i].side_data:
        brk += ws[i].sm_cycles
        i += 1
    if brk < 13:
        raise ValueError(
            f"{PROTO_NAME}: the break field is {brk} bit times, below the 13 the spec asks "
            f"for (tBREAK) -- no responder would see a frame start")
    delim = 0
    while i < len(ws) and is_nop(ws[i]) and not ws[i].side_data:
        delim += ws[i].sm_cycles
        i += 1
    if delim < 1:
        raise ValueError(
            f"{PROTO_NAME}: the break delimiter is {delim} bit times, below the 1 the spec "
            f"asks for (tDELIM)")
    header, i = _field_block(ws, i, 0b010, "header field")     # the header counts with X--
    if i >= len(ws) or ws[i].op != 0b000 or jmp_cond(ws[i]) != 0b100:
        raise ValueError(f"{PROTO_NAME}: word {i} is not the `jmp y--` publish branch")
    space = ws[i].sm_cycles
    resp_addr = jmp_target(ws[i])
    i += 1
    if i >= len(ws) or jmp_cond(ws[i]) != 0b000 or jmp_target(ws[i]) != i:
        raise ValueError(f"{PROTO_NAME}: word {i} is not the idle `jmp idle` loop")
    idle = i
    i += 1
    if resp_addr != i:
        raise ValueError(
            f"{PROTO_NAME}: the publish branch goes to word {resp_addr}, but the response "
            f"field is at word {i}")
    response, i = _field_block(ws, i, 0b100, "response field")   # the response counts with Y--
    if response != header:
        raise ValueError(
            f"{PROTO_NAME}: the response field is {response} SM cycles and the header's is "
            f"{header}; one bit time means one SM cycle for both")
    if i >= len(ws) or ws[i].op != 0b000 or jmp_target(ws[i]) != idle or ws[i].side_data:
        raise ValueError(f"{PROTO_NAME}: word {i} is not the `jmp idle` that ends the response")
    if i + 1 != len(ws):
        raise ValueError(f"{PROTO_NAME}: {len(ws) - i - 1} words follow the program's end")
    bit = 1 + min(d.delay for d in ws if is_out_pindirs_1(d))
    return ProgramTiming(sm_cycles_per_bit=bit, break_bit_times=brk,
                         delimiter_bit_times=delim, field_bit_times=header,
                         response_space_bit_times=space, idle_addr=idle)


# --------------------------------------------------------------------------------------------
# Register values the program needs (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program, *, side_en: int = 1, side_pindir: int = 1,
                   wrap_bottom: int = 0) -> int:
    """EXECCTRL: SIDE_EN, SIDE_PINDIR, and the wrap window the assembler worked out (`.wrap`).

    Both bits have to be 1 for this program:

    * `SIDE_EN = 1` makes the top side-set bit an enable, which is how an instruction with no
      `side` operand writes no pin at all and lets `out pindirs, 1` own it.  With `SIDE_EN = 0`
      the field is two data bits, `side 1` and `side 2` both mean something else, and *every*
      instruction drives the line.
    * `SIDE_PINDIR = 1` makes the side-set write directions, which is the open-drain sense
      (`docs/mvp-isa.md` `### Open drain`): 1 pulls the line to its level register's reset
      value 0, 0 releases it.
    """
    if side_en and program.sideset_opt and program.sideset_bits < 2:
        raise ValueError(
            f"{PROTO_NAME}: `.side_set ... opt` with {program.sideset_bits} side-set bit "
            f"leaves no data bit once the enable takes one")
    if side_pindir and not program.sideset_pindirs:
        raise ValueError(
            f"{PROTO_NAME}: `.side_set ... pindirs` is what the side-set directions mean; "
            f"without it the side-set would drive the level and hold the wire high")
    return ((side_en << 30) | (side_pindir << 29) | (program.wrap << 12) | (wrap_bottom << 7))


def shiftctrl_value(*, pull_thresh: int = 8, out_right: int = 1, autopull: int = 1) -> int:
    """SHIFTCTRL: shift right (LSB out first), autopull every 8 bits.

    LIN sends its data bits least significant first, which is what `OUT_SHIFTDIR = 1` (the
    reset value) gives: `out pindirs, 1` takes OSR bit 0 and shifts the OSR right, so the
    host pushes every byte right-justified with its bits already inverted.

    `PULL_THRESH = 8` is one byte field: the eighth `out pindirs, 1` brings the shift count to
    the threshold and refills the OSR from the TX FIFO in the same SM cycle
    (`docs/mvp-isa.md` `### Instruction semantics`, OUT).  The `pull block` at each field's
    start bit is then a barrier while the OSR is empty and a real pop otherwise, which is what
    primes the first field after a restart and refills between the ones the host is late for.
    """
    if not 1 <= pull_thresh <= 31:
        raise ValueError(f"PULL_THRESH {pull_thresh} out of range 1..31")
    return (pull_thresh << 25) | (out_right << 19) | (autopull << 17)


def pinctrl_value(pin: int, *, sideset_count: int, set_count: int = 1,
                  out_count: int = 1) -> int:
    """PINCTRL: every window of the program points at the LIN pin, one pin wide.

    The side-set drives the pin's direction (break, delimiter, start and stop bits), OUT drives
    the same pin's direction for the data bits, and SET is unused by this program but set to
    the same one-pin window so that a later `set pindirs` cannot land anywhere else.  IN_BASE
    is unused as well -- a commander never reads the wire -- and points at the pin too.

    `sideset_count` must be the assembler's `sideset_bits`, which counts the `opt` enable bit:
    `.side_set 1 opt` is 2, one enable plus one data bit.  Leaving it at 1 (which reads like
    "one side-set bit") turns the data bit into the enable and the program stops driving the
    line; leaving it at its reset value of 2 with SIDE_EN = 0 makes every instruction side-set
    level 0 into SIDESET_BASE and hold the LIN wire low
    (`docs/mvp-isa.md` `### Delay/side-set field`).
    """
    if not 0 <= pin < NUM_PINS:
        raise ValueError(f"LIN pin {pin} is outside 0..{NUM_PINS - 1}")
    if not all(0 <= b <= 32 for b in (sideset_count, set_count, out_count)):
        raise ValueError("window counts must be 0..32")
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (pin << 15)
            | (pin << 10) | (pin << 5) | pin)


@dataclass(frozen=True)
class Config:
    """One LIN setup: which engine and pin, and at which bit rate."""
    rate: float = 19200.0
    engine: int = 0
    pin: int = 0
    version: str = "2.2"

    @property
    def nominal_ticks_per_bit(self) -> float:
        return CORE_HZ / self.rate


class LinDriver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as a LIN commander.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are
    queued on `dev` and land one per tick as the simulation runs, so `read()` and `start()`
    pump the bus while they wait.
    """

    MAX_TICKS = 2_000_000

    @property
    def TXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)

    def __init__(self, dev, bus, *, engine: int = 0, pin: int = 0, rate: float = 19200.0,
                 version: str = "2.2", program: Optional[Program] = None,
                 path: Path | str = PROGRAM_PATH) -> None:
        if engine not in (0, 1):
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        if pin not in UIO_PINS:
            raise ValueError(
                f"LIN pin {pin} is not a uio pin {list(UIO_PINS)}: open drain needs PINDIRS, "
                f"which cannot drive the input-only or output-only protocol pins "
                f"(docs/mvp-isa.md `### The 18 protocol pins`, `### Open drain`)"
            )
        self.dev, self.bus = dev, bus
        self.engine, self.pin = engine, pin
        self.program = program if program is not None else load_program(path)
        self.config = Config(rate=rate, engine=engine, pin=pin, version=version)
        self.timing = program_timing(self.program)

    # ---- addresses ----
    @property
    def imem_base(self) -> int:
        return A_INSTR_MEM[self.engine]

    def sm_addr(self, index: int) -> int:
        return A_SM_BASE + 6 * self.engine + index

    @property
    def frame_addr(self) -> int:
        """The entry point the host forces: the break field (program label `frame`)."""
        return self.program.labels["frame"]

    @property
    def idle_addr(self) -> int:
        """Where the engine parks with nothing to say, readable in `SMe_ADDR`."""
        return self.timing.idle_addr

    # ---- the register values this configuration asks for ----
    @property
    def rate(self) -> float:
        return self.config.rate

    @rate.setter
    def rate(self, value: float) -> None:
        self.config = Config(rate=value, engine=self.engine, pin=self.pin,
                             version=self.config.version)

    @property
    def version(self) -> str:
        return self.config.version

    @property
    def clkdiv(self) -> int:
        return clkdiv_for_rate(self.rate, self.timing.sm_cycles_per_bit)

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        return shiftctrl_value()

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(self.pin, sideset_count=self.program.sideset_bits)

    @property
    def words_used(self) -> int:
        return words_used(self.program)

    def ticks_per_bit(self) -> float:
        """The bit length in core cycles this configuration actually produces."""
        return ticks_per_bit(self.clkdiv, self.timing.sm_cycles_per_bit)

    def frame_ticks(self, n_data: Optional[int] = None) -> int:
        """How long one frame lasts on the wire, in core cycles.

        `n_data is None` for a header-only frame.  This is what a caller runs the bus for, plus
        whatever it wants to leave the wire idle for.
        """
        return self.timing.frame_ticks(self.clkdiv, n_data)

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

    def force(self, instruction: int) -> None:
        """Force one instruction through `SMe_INSTR` and wait for it to have run.

        A host-forced instruction executes in the core cycle after its write whatever the
        divider and `SM_ENABLE` say (`docs/mvp-isa.md` `### EXEC`), it does not advance the PC
        and its delay is ignored.  Two waits: one for the write to land, one for the cycle in
        which it runs, so the next forced instruction cannot overwrite the latch first.
        """
        self.write(self.sm_addr(SM_INSTR), instruction)
        self._settle(2)

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
    def load(self, clkdiv: Optional[int] = None) -> None:
        """Steps 1-3: stop the engine, burst-write the program, set the four registers.

        The engine is left disabled at whatever PC it had, so the caller can arm it and
        preload the TX FIFO before enabling it (a disabled engine never drains it).

        `clkdiv` overrides the divisor `clkdiv_for_rate` would derive, which is how a host
        that wants a bit period the program cannot quite reach says so -- and how the negative
        controls in `test_lin.py` put a commander on the wire that runs 5 % slow.
        """
        self.stop()
        for k, word in enumerate(self.program.instrs):
            self.write(self.imem_base + k, word)
        self.write(self.sm_addr(SM_CLKDIV), self.clkdiv if clkdiv is None else clkdiv)
        self.write(self.sm_addr(SM_EXECCTRL), self.execctrl)
        self.write(self.sm_addr(SM_SHIFTCTRL), self.shiftctrl)
        self.write(self.sm_addr(SM_PINCTRL), self.pinctrl)
        self._settle()

    def arm(self, response_fields: int = 0) -> None:
        """Steps 4-5: set both counters and force `jmp frame`, the only way to set the PC.

        `X = 1` is the header's two byte fields (sync, protected identifier) minus the one the
        counter test itself accounts for, and `Y` is the number of response byte fields this
        node will publish: 0 leaves the wire to a responder after the header, N sends N fields
        (the data bytes and the checksum) before going idle.

        All three are forced while the engine is still stopped and with the TX FIFO empty, so
        the background autopull of a forced cycle has nothing to pop: `SM_RESTART` then resets
        `osr_cnt` to 32 and the program's own `pull block` primes the first field
        (`docs/mvp-isa.md` `### Enable, restart`, `### Reference step`).
        """
        if not 0 <= response_fields <= 31:
            raise ValueError(f"{response_fields} response fields do not fit Y's 5 bits")
        self.force(set_abs(SET_DEST_X, 1))
        self.force(set_abs(SET_DEST_Y, response_fields))
        self.force(jmp_abs(self.frame_addr))

    def enable(self) -> None:
        """Step 6: SM_ENABLE with SM_RESTART and CLKDIV_RESTART in the same CTRL write.

        The engine's first SM cycle is then two core cycles after this write
        (`docs/mvp-isa.md` `### Clock divider`), and `SM_RESTART` clears `osr_cnt`, `isr_cnt`,
        `delay` and `stalled` -- so every frame starts from the same state and the same divider
        phase, whatever the last one left behind.
        """
        ctrl = self.read(A_CTRL) & ~self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, ctrl | self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def stop(self) -> None:
        """Disable the engine, keeping every register (`docs/mvp-isa.md` `### Enable, restart`)."""
        ctrl = self.read(A_CTRL)
        self.write(A_CTRL, ctrl & ~self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def restart(self) -> None:
        """SM_RESTART + CLKDIV_RESTART, keeping the enable (docs/mvp-regmap.md `**CTRL**`)."""
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
        """Push one byte field, waiting for room first so no word is dropped (TXOVER).

        The word is right-justified and inverted: `wire_word()` is the only place that happens,
        because `out pindirs, 1` writes a direction and 1 means drive, which is dominant, which
        is a zero data bit.
        """
        if not 0 <= word <= 0xFF:
            raise ValueError(f"a byte field is one word, 0..255 (got {word!r})")
        while self.tx_full:
            self._settle(1)
        self.write(A_TXF[self.engine], word)
        self._settle(1)

    def send(self, words: Sequence[int]) -> None:
        """Push several byte fields, in order, waiting for FIFO room between them."""
        for word in words:
            self.push(word)

    def preload(self, words: Sequence[int]) -> None:
        """Fill the TX FIFO while the engine is stopped.

        Only `TXFIFO_DEPTH` words fit and a disabled engine never pops, so this is how a host
        gets the frame's first fields in before the break starts: the program's first
        `pull block` would otherwise stall on the empty FIFO (and set FDEBUG.TXSTALL), and a
        stalled start bit is a stretched byte field.
        """
        words = list(words)
        if len(words) > self.TXFIFO_DEPTH:
            raise ValueError(
                f"{len(words)} words do not fit the stopped {self.TXFIFO_DEPTH}-deep TX FIFO; "
                f"call send() after enable()"
            )
        for word in words:
            self.push(word)

    # ---- one frame ----
    def start(self, frame_id: int, data=None, version: Optional[str] = None) -> list[int]:
        """Put one frame on the wire and return the words that went into the TX FIFO.

        `data is None` sends the header alone, which is the usual arrangement: the node that
        owns the schedule table answers its own frames.  1..8 `data` bytes make this node the
        publisher of the response as well, with the checksum its identifier calls for on a
        `version` cluster.
        """
        words = frame_words(frame_id, data, version or self.version)
        self.stop()
        self.arm(len(words) - 2)
        self.preload(words[: self.TXFIFO_DEPTH])
        self.enable()
        self.send(words[self.TXFIFO_DEPTH:])
        return words

    # ---- CTRL helpers ----
    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask

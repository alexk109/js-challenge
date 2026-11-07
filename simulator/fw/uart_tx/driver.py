"""Host side of the MVP UART-TX firmware: load program.pio, set up engine 0, push bytes.

Everything here is written against `docs/mvp-regmap.md` (register word
addresses, the `### Typical host sequences` startup order and the per-engine fields) and
against `docs/mvp-isa.md` (divider, side-set, autopull).  Host accesses go through
`iss.pio_iss.ChipDevice`'s write/read queue, one per tick, so every wait here is the
simulation running -- the same thing a real host does over the SPI port of
`docs/mvp-regmap.md` `## SPI transaction format`.

    bus = Bus(ns_per_tick=20)                     # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())
    drv = UartTxDriver(dev, bus, pin=0)
    drv.load()                 # IMEM, CLKDIV, EXECCTRL, SHIFTCTRL, PINCTRL
    drv.start()                # forced `jmp start`, then CTRL restart + enable
    drv.send(b"hello")         # blocks while the TX FIFO is full
    bus.run(200_000)
    drv.stop()

The chip core clock is 50 MHz: one `verif.vip.core.Bus` tick at `ns_per_tick = 20`, the
configuration the UART VIPs document in their own docstrings.

Two things about this chip that the RP2040 driver does not have to think about, and that
this module exists to get right:

* `PINCTRL.SIDESET_COUNT` resets to 2 and `EXECCTRL.SIDE_EN` to 0, and with `SIDE_EN = 0`
  *every* instruction side-sets `SIDESET_COUNT` pins with data taken from the top bits of
  the side-set field.  The program uses one `side` operand, so the driver writes
  `SIDESET_COUNT = 2` with `SIDE_EN = 1`: two bits are one enable plus one data bit, and
  only then does `side 2` mean "one pin, driven low".  It also points `SIDESET_BASE` at the
  TX pin (`docs/mvp-isa.md` `### Delay/side-set field`).
* The host port moves whole 32-bit words, so the driver right-justifies every byte it
  pushes (bits 7:0): `SHIFTCTRL.OUT_SHIFTDIR` keeps its reset value of 1, so `out pins, 1`
  sends the least significant bit first, which is the order a UART frame is in
  (`docs/mvp-isa.md` `### Instruction semantics`, OUT).
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

from asm.pioasm import Program, assemble

from fw.uart_tx.framing import DEFAULT as DEFAULT_FRAMING
from fw.uart_tx.framing import Framing
from fw.uart_tx.framing import program_for

CORE_HZ = 50_000_000          # one Bus tick at 20 ns (verif/vip/core.py)
NUM_PINS = 18                 # docs/mvp-isa.md `### The 18 protocol pins`
UIO_PINS = range(0, 8)        # the bidirectional pins: the only ones PINDIRS can drive
PROGRAM_PATH = Path(__file__).with_name("program.pio")
PROTO_NAME = "uart_tx"        # the .program name inside program.pio

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
OP_JMP = 0b000
JMP_ALWAYS = 0b000


def clkdiv_value(integer: int, frac: int = 0) -> int:
    """SMe_CLKDIV: INT in 31:16 (0 means 65536), FRAC in 15:8 in 1/256."""
    if not 1 <= integer <= 0xFFFF:
        raise ValueError(f"CLKDIV INT {integer} out of range 1..65535")
    if not 0 <= frac <= 0xFF:
        raise ValueError(f"CLKDIV FRAC {frac} out of range 0..255")
    return (integer << 16) | (frac << 8)


def clkdiv_for_baud(baud: float, sm_cycles_per_bit: int) -> int:
    """The CLKDIV word that makes one bit last `sm_cycles_per_bit` SM cycles at `baud`.

    One bit must last CORE_HZ / baud core cycles, and one SM cycle lasts one CLKDIV
    period, so the divisor wanted is CORE_HZ / (baud * sm_cycles_per_bit), rounded into
    INT and FRAC (docs/mvp-isa.md `### Clock divider`).  The divisor is exact on average,
    so the transmitter keeps the nominal grid to within one core cycle per SM cycle.
    """
    if baud <= 0:
        raise ValueError(f"baud must be positive (got {baud})")
    if sm_cycles_per_bit < 1:
        raise ValueError(f"a bit needs at least one SM cycle (got {sm_cycles_per_bit})")
    want = CORE_HZ / (baud * sm_cycles_per_bit)
    if want > 0xFFFF:
        raise ValueError(
            f"{baud} Bd needs a CLKDIV INT of {want:.1f}, above the 16-bit maximum"
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


def jmp_abs(addr: int) -> int:
    """JMP with the `always` condition and an absolute address (docs/mvp-isa.md `## Encoding`)."""
    if not 0 <= addr <= 31:
        raise ValueError(f"JMP address {addr} does not fit in 5 bits")
    return (OP_JMP << 13) | (JMP_ALWAYS << 5) | addr


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


def decode_word(program: Program, word: int) -> dict:
    """Split one word into the fields `docs/mvp-isa.md` `## Encoding` defines.

    Only what the driver needs: the opcode, the side-set bits and the delay, which are
    the fields whose split depends on PINCTRL.SIDESET_COUNT rather than on the
    instruction.  `sc` is the count including the `opt` enable bit, so the delay field is
    the low `5 - sc` bits, as the spec says.
    """
    sc = min(program.sideset_bits, 5)
    field = (word >> 8) & 0x1F
    return {
        "op": word >> 13,
        "sideset": field >> (5 - sc) if sc else 0,
        "delay": field & ((1 << (5 - sc)) - 1),
        "low": word & 0xFF,
    }


def is_out_pins_1(program: Program, word: int) -> bool:
    """True for `out pins, 1`, the one instruction this program uses per data bit."""
    fields = decode_word(program, word)
    return fields["op"] == 0b011 and (fields["low"] >> 5) == 0b000 and fields["low"] & 0x1F == 1


def _bit_cycles(program: Program, shift_bits: int) -> int:
    bits = [decode_word(program, w)["delay"] for w in program.instrs if is_out_pins_1(program, w)]
    if len(bits) != shift_bits:
        raise ValueError(
            f"{PROTO_NAME}: expected {shift_bits} `out pins, 1` data bits, found {len(bits)}; "
            f"the divider cannot be derived from this program"
        )
    if len(set(bits)) != 1:
        raise ValueError(f"{PROTO_NAME}: the data bits have different delays {bits}")
    return 1 + bits[0]


def bit_cycles(program: Program) -> int:
    """SM cycles one bit lasts, taken from the assembled words rather than assumed.

    The shipped `program.pio` sends one `out pins, 1` per data bit, each with the same delay, so
    a bit is `1 + delay` SM cycles (docs/mvp-isa.md `### Cycle accounting`: the next instruction
    runs `1 + delay` SM cycles later).  Anything else in the words would make CLKDIV mean
    something else, so this raises instead of guessing.
    """
    return _bit_cycles(program, 8)


def framed_bit_cycles(program: Program, shift_bits: int) -> int:
    """`bit_cycles` for a framing that shifts `shift_bits` bits per byte.

    Supported framings: 7 or 9 data bits, and a parity bit that is shifted out by an
    `out pins, 1` like any other bit (`fw/uart_tx/framing.py`), so a parity framing shifts one bit
    more per byte than it has data bits and a bit still lasts `1 + delay` SM cycles.
    """
    return _bit_cycles(program, shift_bits)


def words_used(program: Program) -> int:
    """Words of the 32-word IMEM the program occupies (docs/mvp-isa.md `### Engine state`)."""
    return len(program.instrs)


# --------------------------------------------------------------------------------------------
# Register values the program needs (docs/mvp-regmap.md `### Per-engine registers`)
# --------------------------------------------------------------------------------------------

def execctrl_value(program: Program, *, side_en: int = 1, wrap_bottom: int = 0) -> int:
    """EXECCTRL: SIDE_EN plus the wrap window the assembler worked out (`.wrap`).

    SIDE_EN has to be 1 for the program's `.side_set 1 opt` to mean what the assembler
    says: with SIDE_EN = 0 the top side-set bit is data, so `side 2` would drive *two* pins
    (SIDESET_BASE and SIDESET_BASE + 1) instead of one, and `side 1` would drive the TX pin
    and its neighbour (`docs/mvp-isa.md` `### Delay/side-set field`).
    """
    if side_en and program.sideset_opt and program.sideset_bits < 2:
        raise ValueError(
            f"{PROTO_NAME}: `.side_set ... opt` with {program.sideset_bits} side-set bit "
            f"leaves no data bit once the enable takes one"
        )
    return (side_en << 30) | (program.wrap << 12) | (wrap_bottom << 7)


def shiftctrl_value(*, pull_thresh: int = 8, out_right: int = 1, autopull: int = 1) -> int:
    """SHIFTCTRL: shift right (LSB out first), autopull every 8 bits.

    A UART frame puts the least significant data bit on the wire first, which is what
    `OUT_SHIFTDIR = 1` gives (the reset value): `out pins, 1` sends OSR bit 0 and shifts the
    OSR right.  So the host pushes each byte right-justified, in bits 7:0.  The RP2040
    examples leave this bit alone; writing it as 0 would send the byte reversed, which
    `verif.vip.uart.UartRx` decodes as a different byte.

    PULL_THRESH = 8 is one byte: the eighth `out pins, 1` of a byte brings the OSR count to
    the threshold and refills it from the TX FIFO in the same SM cycle
    (docs/mvp-isa.md `### Instruction semantics`, OUT).
    """
    if not 1 <= pull_thresh <= 31:
        raise ValueError(f"PULL_THRESH {pull_thresh} out of range 1..31")
    return (pull_thresh << 25) | (out_right << 19) | (autopull << 17)


def pinctrl_value(pin: int, *, sideset_count: int, set_count: int = 1,
                  out_count: int = 1) -> int:
    """PINCTRL: every window of the program points at the TX pin, one pin wide.

    OUT writes the data bits, SET writes the start/stop level and the direction, and the
    side-set writes the start bit, all on the same pin; the three channels are used at
    different times, and each one only ever covers the TX pin.  IN_BASE is unused by the
    program but is set to the TX pin for consistency.

    `sideset_count` must be the assembler's `sideset_bits`, which counts the `opt` enable
    bit: `.side_set 1 opt` is 2, one enable plus one data bit.  Leaving it at 1 (which looks
    like "one side-set bit") turns the single bit into the enable and the program never
    writes the start bit.  It also must not be left at its reset value of 2 with SIDE_EN = 0,
    where every instruction would side-set level 0 into SIDESET_BASE on every SM cycle and
    hold the TX line low (`docs/mvp-isa.md` `### Delay/side-set field`).
    """
    if not 0 <= pin < NUM_PINS:
        raise ValueError(f"TX pin {pin} is outside 0..{NUM_PINS - 1}")
    if not all(0 <= b <= 32 for b in (sideset_count, set_count, out_count)):
        raise ValueError("window counts must be 0..32")
    return ((sideset_count << 29) | (set_count << 26) | (out_count << 20) | (pin << 15)
            | (pin << 10) | (pin << 5) | pin)


@dataclass(frozen=True)
class Config:
    """One UART-TX setup: which engine and pin, and at which baud."""
    baud: float = 115200.0
    engine: int = 0
    pin: int = 0
    data_bits: int = 8          # 8N1 unless the caller asks for another framing
    stop_bits: int = 1
    parity: str = "none"

    @property
    def frame_bits(self) -> int:
        parity = 1 if self.parity != "none" else 0
        return 1 + self.data_bits + parity + self.stop_bits

    @property
    def nominal_ticks_per_bit(self) -> float:
        return CORE_HZ / self.baud


class UartTxDriver:
    """Drives one engine of an `iss.pio_iss.ChipDevice` as a UART transmitter.

    `dev` and `bus` are the ChipDevice and the Bus it is attached to: host accesses are
    queued on `dev` and land one per tick as the simulation runs, so `read()` and `send()`
    pump the bus while they wait.
    """

    MAX_TICKS = 2_000_000

    @property
    def TXFIFO_DEPTH(self) -> int:  # read at use time: the active map is fixed at target install
        return regmap.fifo_depth()

    CTRL_BITS = (CTRL_SM_ENABLE, CTRL_SM_RESTART, CTRL_CLKDIV_RESTART)

    def __init__(self, dev, bus, *, engine: int = 0, pin: int = 0, baud: float = 115200.0,
                 program: Optional[Program] = None, path: Path | str = PROGRAM_PATH,
                 framing: Optional["Framing"] = None) -> None:
        if not 0 <= engine < regmap.n_engines():
            raise ValueError(f"engine {engine} out of range (the MVP has two)")
        if pin not in UIO_PINS:
            raise ValueError(
                f"TX pin {pin} is not a uio pin {list(UIO_PINS)}: PINDIRS cannot drive "
                f"input-only or output-only protocol pins (docs/mvp-isa.md `## Pins`)"
            )
        self.dev, self.bus = dev, bus
        self.engine, self.pin = engine, pin
        #: The frame this transmitter sends (7 and 9 data bits, even and
        #: odd parity, two stop bits).  8N1, the shipped `program.pio`, is the default; any other
        #: framing is the same program shape assembled for it (`fw/uart_tx/framing.py`), and an
        #: explicit `program` always wins so a test can run a variant.
        self.framing = framing if framing is not None else DEFAULT_FRAMING
        if program is not None:
            self.program = program
        elif self.framing == DEFAULT_FRAMING:
            self.program = load_program(path)
        else:
            self.program = program_for(self.framing)
        self.config = Config(baud=baud, engine=engine, pin=pin, data_bits=self.framing.data_bits,
                             stop_bits=self.framing.stop_bits, parity=self.framing.parity)
        # `bit_cycles` is the shipped 8N1 derivation; a framing that shifts a different number of
        # bits per byte goes through `framed_bit_cycles`, which takes the count.  The 8N1 path
        # keeps calling `bit_cycles(program)` with one argument, which is what the existing
        # mutation tests in `test_uart_tx.py` stub.
        self.sm_cycles_per_bit = (bit_cycles(self.program) if self.framing.shift_bits == 8
                                  else framed_bit_cycles(self.program, self.framing.shift_bits))

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
    def baud(self) -> float:
        return self.config.baud

    @baud.setter
    def baud(self, value: float) -> None:
        self.config = Config(baud=value, engine=self.engine, pin=self.pin,
                             data_bits=self.framing.data_bits, stop_bits=self.framing.stop_bits,
                             parity=self.framing.parity)

    @property
    def clkdiv(self) -> int:
        return clkdiv_for_baud(self.baud, self.sm_cycles_per_bit)

    @property
    def execctrl(self) -> int:
        return execctrl_value(self.program)

    @property
    def shiftctrl(self) -> int:
        # PULL_THRESH is one frame's worth of shifted bits, so the OSR refills with the next byte
        # (and its parity bit, if the framing has one) in the same SM cycle the last bit of this
        # one goes out.  For 8N1 that is the shipped value of 8.
        return shiftctrl_value(pull_thresh=self.framing.shift_bits)

    @property
    def pinctrl(self) -> int:
        return pinctrl_value(self.pin, sideset_count=self.program.sideset_bits)

    @property
    def words_used(self) -> int:
        return words_used(self.program)

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
        """Step 5: force `jmp start` through SMe_INSTR, which is the only way to set the PC.

        The host-forced instruction runs in the very next core cycle whatever the divider
        and SM_ENABLE say (docs/mvp-isa.md `### EXEC`).  That forced cycle also runs the
        background autopull, so it must happen *before* the host fills the TX FIFO:
        otherwise the forced cycle pops a word into the OSR and SM_RESTART (which does not
        touch the OSR, only `osr_cnt`) leaves that stale word to be shifted out.
        """
        self.write(self.sm_addr(SM_INSTR), jmp_abs(self.start_addr))
        self._settle(2)                 # the forced JMP executes one cycle after its write

    def enable(self) -> None:
        """Step 6: SM_ENABLE with SM_RESTART and CLKDIV_RESTART in the same CTRL write.

        The engine's first SM cycle is then two core cycles after this write
        (docs/mvp-isa.md `### Clock divider`).
        """
        ctrl = self.read(A_CTRL) & ~self.ctrl_bits(*self.CTRL_BITS)
        self.write(A_CTRL, ctrl | self.ctrl_bits(*self.CTRL_BITS))
        self._settle()

    def start(self, first: Sequence[int] = ()) -> None:
        """Arm, optionally fill the TX FIFO while stopped, then enable.

        `first` is what a host that must not see a stall in FDEBUG pushes between the two:
        the program's first instruction is a blocking `pull`, so an engine started with an
        empty TX FIFO sits on FDEBUG.TXSTALL until the first byte arrives.
        """
        self.arm()
        if first:
            self.preload(first)
        self.enable()

    def stop(self) -> None:
        """Disable the engine, keeping every register (docs/mvp-isa.md `### Enable, restart`)."""
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

    def push(self, byte: int) -> None:
        """Push one TX word, waiting for room first so no word is dropped (TXOVER).

        The word is right-justified, in bits `shift_bits-1:0`: `OUT_SHIFTDIR = 1` makes the PIO
        shift the OSR right, so bit 0 of the word is the first one on the wire, as a UART frame
        wants (least significant data bit first).  The host normally goes through `push_byte`,
        which packs the parity bit in; this is the raw path, and the range it accepts is the
        framing's (`0..0xFF` for 8N1, which is what the shipped program sends).
        """
        if not 0 <= byte <= self.framing.word_mask:
            raise ValueError(
                f"a word is 0..{self.framing.word_mask} for {self.framing} "
                f"({self.framing.shift_bits} shifted bits), got {byte!r}"
            )
        while self.tx_full:
            self._settle(1)
        self.write(A_TXF[self.engine], byte)
        self._settle(1)

    def push_byte(self, value: int) -> None:
        """Push one byte, packed for this framing (parity bit included).

        This is what a host that has a byte to send wants; `push` is the raw word path.
        """
        self.push(self.framing.pack(value))

    def send(self, values: Iterable[int]) -> None:
        """Push several bytes, in order, waiting for FIFO room between them."""
        for byte in values:
            self.push(byte)

    def send_bytes(self, values: Iterable[int]) -> None:
        """Send several byte values, packing each one for this framing as it goes."""
        for value in values:
            self.push_byte(value)

    def preload(self, values: Sequence[int]) -> None:
        """Fill the TX FIFO while the engine is stopped.

        Only `TXFIFO_DEPTH` words fit and a disabled engine never pops, so this is how a
        host gets the first bytes in before starting: the program's `pull block` would
        otherwise stall on the empty FIFO (and set FDEBUG.TXSTALL) on its first SM cycle.

        The values are byte values, not words: they are packed for this framing, exactly as
        `send_bytes` would pack them one tick later.
        """
        values = list(values)
        if len(values) > self.TXFIFO_DEPTH:
            raise ValueError(
                f"{len(values)} bytes do not fit the stopped {self.TXFIFO_DEPTH}-deep TX FIFO; "
                f"call send() after start()"
            )
        for value in values:
            self.push_byte(value)

    # ---- CTRL helpers ----
    def ctrl_bits(self, *fields: int) -> int:
        """The engine-local CTRL fields named, each moved up to this engine's bit."""
        mask = 0
        for base in fields:
            mask |= base << self.engine
        return mask



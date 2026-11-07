"""Host side of the MVP CAN firmware: the register writes that run fw/can/program.pio.

Written against docs/mvp-regmap.md and the Bosch CAN Specification 2.0 Part A saved
in `research/ref/protocols/can/`, not against the ISS and not against the model in
`verif/vip/can.py`: every framing rule below is transcribed from the spec text and
cites the file and section it came from, so `fw/can/test_can.py` can check the two
implementations against each other without either one having been written from the
other.

**What the host builds and what the PIO does.** `program.pio` is the CAN bit layer:
one bit on one open-drain line per bit time, at exactly the nominal bit time, read
back in the same bit time. Everything above that -- the CRC SEQUENCE, the bit
stuffing, the field order, the INTERMISSION -- is this module, because the MVP
instruction set cannot compute a CRC (no XOR; see `fw/can/CANNOT-EXPRESS.md`) and
cannot destuff, and a CAN receiver cannot ACK without destuffing. The split is the
same one a soft-CAN stack makes on a host CPU: the framing is arithmetic, the bit
timing and the open-drain drive are the part that has to be exact in core cycles.

The judge is `verif/vip/can.py`: `CanNode` is the receiver that acknowledges, and
`CanChecker` is the passive monitor that flags every timing, stuffing, CRC, form,
ACK and intermission violation. Neither is edited and neither is softened.

    driver = CanDriver(bus, chip_device, program, bit_ticks=400)
    driver.load()
    result = driver.send(0x123, b"\\xde\\xad")     # one DATA FRAME, 125 kbit/s
    assert result.acked and not result.bit_errors

Three things in here are not obvious and are worth reading before changing them:

* `wire_bits()` builds the bits, `tx_words()` packs them, and the PIO only shifts
  them out. The bits the host pushes are *directions*: 1 = drive the line low =
  dominant, 0 = release it = recessive.
* `_sof_cycle()` decides which core cycle the SOF edge happens in. The reference model
  has no propagation delay and no oscillator error, so a node can only hard
  synchronise on a SOF edge that lands on one of *its own* bit boundaries
  (`CanNode._bit_boundary`), and the driver has to meet it there.
* `_measure_phase()` finds the engine's SM-cycle grid, because `CLKDIV_RESTART`
  fixes where in a core cycle the engine's SM cycles fall and there is no register
  that reports it.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence

from asm.pioasm import Program

#: The one `.pio` file this driver loads.
PROGRAM_PATH = Path(__file__).with_name("program.pio")

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) -----------------
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF, A_RXF = regmap.txf_addrs(), regmap.rxf_addrs()
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_IMEM = regmap.imem_bases()           # engine e, IMEM word k at A_IMEM[e] + k
A_SM = regmap.sm_base()                     # SMe_* at A_SM + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

FLEVEL_TX0, FLEVEL_RX0 = 0, 4   # shift of the engine-0 field of FLEVEL 3:0 / 7:4
FDEBUG_TXSTALL0, FDEBUG_RXSTALL0 = 24, 0

# ---- the frame format, from Bosch CAN 2.0 Part A ------------------------------------
# `01-bosch-frame-fields-coding.txt` sec. 3.1.1 (DATA FRAME) and sec. 3.1.2 (REMOTE
# FRAME), quoted in `research/ref/protocols/can/README.md`.
DOMINANT = 0                    # sec. 2: dominant is a logical '0' on a wired-AND bus
RECESSIVE = 1                   # ... and recessive a logical '1'
ID_BITS = 11                    # sec. 3.1.1: "The IDENTIFIER's length is 11 bits"
CRC_BITS = 15                   # sec. 3.1.1: CRC FIELD, 15 bits, CRC14 .. CRC0
CRC_POLY = 0x4599               # sec. 3.1.1: X^15+X^14+X^10+X^8+X^7+X^4+X^3+1
STUFF_RUN = 5                   # sec. 5: after five equal bits insert the complement
MAX_DLC = 8                     # sec. 3.1.1: "admissible numbers of data bytes: 0..8"
#: sec. 2: "An error passive station ... transmits its message ... eight 'recessive'
#: bits following INTERMISSION" is only owed by an error-passive node; an
#: error-active one owes nothing, so the tail below is INTERMISSION alone.
INTERMISSION = 3                # sec. 3.1.5: "INTERMISSION consists of three 'recessive' bits"
TAIL_BITS = INTERMISSION + 5    # three recessive bits, plus room for the host's word padding


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


PIN_LIMIT = 7                   # open drain needs a uio pin (docs/mvp-isa.md `## Pins`)
DEFAULT_CAN_PIN = 0
#: One all-recessive TX word the driver pushes behind every frame, so the engine does not
#: stall on an empty FIFO once the frame is out (see `send`).
SENTINEL = 0
#: SM cycles one bit takes in fw/can/program.pio: `out pindirs, 1 [7]`, `in pins, 1 [0]`
#: and `jmp engine`, i.e. 8 + 1 + 1. `loop_cycles()` reads the same number out of the
#: assembled words and `CanDriver` uses that one, so this is the documented value and
#: not the only source of truth.
SM_CYCLES_PER_BIT = 10
READ_LIMIT = 100_000           # core cycles one register read may take to come back

#: One 125 kbit/s bit is 8 us and one 500 kbit/s bit is 2 us, so on the ISS's 20 ns
#: core cycle that is 400 and 100 cycles.  `program.pio`'s loop is 10 SM cycles per
#: bit, so the two speeds are CLKDIV 40 and CLKDIV 10 -- the same words, only the
#: divider changes (`clkdiv_for`).
NOMINAL_TICKS = {"125k": 400, "500k": 100}


def crc15(bits: Sequence[int]) -> int:
    """The 15-bit CRC of the destuffed stream SOF..end of DATA FIELD.

    `01` sec. 3.1.1 gives the transmitter's pseudo-code verbatim, and this is it:

        CRC := 0
        for every bit NXTBIT of SOF..DATA FIELD:
            CRCNXT := NXTBIT XOR (CRC(14))
            CRC(14..1) := CRC(13..0)
            CRC(0) := CRCNXT
            if CRCNXT = 1 then CRC := CRC XOR '101001101001101' (0x4599)

    Returned MSB first, so CRC14 is bit 14 of the result.
    """
    crc = 0
    for bit in bits:
        feedback = bit ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if feedback:
            crc ^= CRC_POLY
    return crc


def stuff(bits: Sequence[int]) -> list[int]:
    """Bit-stuff `bits` the way sec. 5 CODING describes it.

    "After the occurrence of five consecutive bits of identical value, one
    complementary bit is inserted."  The inserted bit is a bit of the wire like any
    other, so it also starts the next run -- which is what keeps the longest run on
    the wire at five, and what makes a sixth equal bit the STUFF ERROR of sec. 6.1.
    """
    out: List[int] = []
    run, prev = 0, RECESSIVE
    for bit in bits:
        run = run + 1 if bit == prev else 1
        prev = bit
        out.append(bit)
        if run == STUFF_RUN:
            prev = RECESSIVE if bit == DOMINANT else DOMINANT
            out.append(prev)
            run = 1
    return out


def stuffed_region(dlc: int, rtr: bool) -> int:
    """How many logical bits are inside the stuffed region.

    sec. 5: "The frame segments START OF FRAME, ARBITRATION FIELD, CONTROL FIELD,
    DATA FIELD and CRC SEQUENCE are coded by the method of bit stuffing.  [...] The
    remaining bit fields of the DATA FRAME [...] are of fixed form and not stuffed."
    So the region is SOF (1) + IDENTIFIER (11) + RTR (1) + the two reserved bits +
    DLC (4) + 8 per data byte -- none at all in a REMOTE FRAME, whatever its DLC
    says (sec. 3.1.2) -- + the CRC SEQUENCE.
    """
    data_bits = 0 if rtr else 8 * dlc
    return 1 + ID_BITS + 1 + 2 + 4 + data_bits + CRC_BITS


def logical_bits(identifier: int, data: bytes = b"", rtr: bool = False,
                 dlc: Optional[int] = None) -> tuple[list[int], list[str]]:
    """The destuffed bit stream of one DATA or REMOTE FRAME, with a role per bit.

    Field order and widths are sec. 3.1.1's own, one line each:
    SOF (1 dominant), IDENTIFIER (ID-10..ID-0), RTR (1), the two reserved bits
    ("The reserved bits have to be sent 'dominant'"), DLC3..DLC0, the data bytes
    MSB first, CRC14..CRC0 of everything above, CRC DELIMITER (1 recessive), ACK
    SLOT (1 recessive -- a *receiver* drives it, `01` sec. 3.1.1 ACK FIELD), ACK
    DELIMITER (1 recessive), END OF FRAME (7 recessive).
    """
    if not 0 <= identifier < (1 << ID_BITS):
        raise ValueError(f"a standard CAN identifier is {ID_BITS} bits, got {identifier:#x}")
    data = bytes(data)
    if rtr and data:
        raise ValueError("a REMOTE FRAME carries no DATA FIELD (sec. 3.1.2)")
    if len(data) > MAX_DLC:
        raise ValueError(f"a classic CAN frame carries 0..{MAX_DLC} bytes, got {len(data)}")
    if dlc is None:
        dlc = len(data)
    if not 0 <= dlc <= MAX_DLC:
        raise ValueError(f"DLC must be 0..{MAX_DLC}, got {dlc}")

    bits: List[int] = [DOMINANT]
    roles: List[str] = ["sof"]
    for k in range(ID_BITS - 1, -1, -1):                    # ID-10 .. ID-0
        bits.append((identifier >> k) & 1)
        roles.append("id")
    bits.append(RECESSIVE if rtr else DOMINANT)             # RTR
    roles.append("rtr")
    bits += [DOMINANT, DOMINANT]                            # r1, r0: reserved
    roles += ["reserved", "reserved"]
    for k in range(3, -1, -1):                              # DLC3 .. DLC0
        bits.append((dlc >> k) & 1)
        roles.append("dlc")
    payload = b"" if rtr else data
    for byte in payload:                                    # data field, MSB first
        for k in range(7, -1, -1):
            bits.append((byte >> k) & 1)
            roles.append("data")
    body = list(bits)
    crc = crc15(body)                                       # sec. 3.1.1: CRC over SOF..DATA
    for k in range(CRC_BITS - 1, -1, -1):
        bits.append((crc >> k) & 1)
        roles.append("crc")
    bits.append(RECESSIVE)                                  # CRC DELIMITER
    roles.append("crc_delim")
    bits.append(RECESSIVE)                                  # ACK SLOT: the transmitter
    roles.append("ack")                                     #   releases it
    bits.append(RECESSIVE)                                  # ACK DELIMITER
    roles.append("ack_delim")
    bits += [RECESSIVE] * 7                                 # END OF FRAME
    roles += ["eof"] * 7
    return bits, roles


def wire_bits(identifier: int, data: bytes = b"", rtr: bool = False,
              dlc: Optional[int] = None, tail: int = TAIL_BITS) -> tuple[list[int], list[str]]:
    """The bits to put on the wire: the stuffed stream plus INTERMISSION.

    Everything from the CRC DELIMITER on is fixed form and not stuffed (sec. 5), so
    the cut is taken at that field. The `tail` recessive bits are INTERMISSION
    (sec. 3.1.5, three recessive bits, during which no station may start
    transmission) plus whatever the host needs to fill the last 32-bit word.
    """
    bits, roles = logical_bits(identifier, data, rtr, dlc)
    cut = roles.index("crc_delim")
    wire, wire_roles = _roles_for_stuffed(bits[:cut], roles[:cut])
    wire += bits[cut:]
    wire_roles += roles[cut:]
    wire += [RECESSIVE] * tail
    wire_roles += ["intermission"] * tail
    return wire, wire_roles


def _roles_for_stuffed(logical: Sequence[int], roles: Sequence[str]) -> tuple[list[int], list[str]]:
    """The stuffed bits and one role per *wire* bit, inserted ones marked "stuff".

    `logical_bits` labels the destuffed stream; stuffing inserts bits that carry no
    field, so the roles are walked again with the same run-length rule `stuff` used.
    """
    wire = stuff(logical)
    out: List[str] = []
    run, prev, i = 0, RECESSIVE, 0
    for bit in wire:
        if run == STUFF_RUN:
            out.append("stuff")
            run, prev = 1, bit
            continue
        out.append(roles[i])
        i += 1
        run = run + 1 if bit == prev else 1
        prev = bit
    return wire, out

def directions(wire: Sequence[int]) -> list[int]:
    """Wire bits to the *directions* `out pindirs, 1` writes.

    Bosch sec. 2 gives the bus the other way round from the direction register: a
    DOMINANT bit is a logical '0' on the wire, and it is a *drive* (direction 1) on
    a uio pin, while a RECESSIVE bit is a release (direction 0) with the pull-up
    bringing the line high (docs/mvp-isa.md `### Open drain`: the level register
    stays 0 and the program only toggles the direction). So every bit is inverted.
    """
    return [1 - bit for bit in wire]


def tx_words(wire: Sequence[int]) -> list[int]:
    """The TX words the PIO shifts out: 32 wire bits each, MSB first, 1 = dominant.

    `program.pio` has SHIFTCTRL.OUT_SHIFTDIR = 0, so `out pindirs, 1` walks the OSR
    from the top and one word is 32 consecutive wire bits. The last word is padded
    with recessive bits, which is free: the frame is over by then and the bus has to
    be recessive anyway (sec. 3.1.5 INTERMISSION). Padded bits *are* transmitted --
    a dominant padding bit would be a SOF inside INTERMISSION, so the padding is
    recessive and the bus stays idle.
    """
    wire = list(wire) + [RECESSIVE] * ((-len(wire)) % 32)
    words = []
    for i in range(0, len(wire), 32):
        word = 0
        for k, bit in enumerate(directions(wire[i:i + 32])):
            word |= bit << (31 - k)
        words.append(word)
    return words


def readback_word(word: int, acked: bool = False) -> int:
    """The RX word the PIO collects for one TX word: the bus, not the OSR.

    `in pins, 1` reads the line, and a released line reads high, so a word whose bits
    were all dominant reads all ones and vice versa: the readback is the complement
    of the TX word. The one bit where that is not true is the ACK SLOT -- there a
    receiver drives the line low while the transmitter is releasing it, which is
    exactly how the host learns that the frame was acknowledged
    (`01` sec. 3.1.1 ACK FIELD: "All stations having received the matching CRC
    SEQUENCE report this within the ACK SLOT").
    """
    return (~word & 0xFFFF_FFFF) & ~(1 if acked else 0)


def ack_bit_mask(word_index: int, bit_offset: int) -> int:
    """The mask of the ACK SLOT inside TX word `word_index`, at `bit_offset` in it.

    MSB first, so wire bit 32k of the frame is bit 31 of word k and
    `bit_offset == ack_wire_index % 32`.
    """
    if not 0 <= bit_offset < 32:
        raise ValueError(f"bit {bit_offset} is not inside word {word_index}")
    return 1 << (31 - bit_offset)


class CanError(RuntimeError):
    """The engine or the bus did something the driver cannot make sense of."""


def loop_cycles(program: Program, label: str = "engine") -> int:
    """SM cycles one turn of the program's bit loop takes.

    Read out of the assembled words rather than out of a comment: the delay field of
    every instruction from `label` to the jump back to it, plus one cycle each
    (docs/mvp-isa.md `### Cycle accounting`: an instruction from IMEM that does not
    stall retires in `1 + delay` SM cycles). This is the number the bit rate is built
    from, so a change to a delay in `program.pio` cannot leave the driver behind.
    """
    start = program.labels[label]
    total = 0
    for pc in range(start, start + len(program.instrs)):
        word = program.instrs[pc % len(program.instrs)]
        total += 1 + ((word >> 8) & 0x1F)
        if (word >> 13) == 0 and (word & 0x1F) == start:      # `jmp <label>`
            return total
    raise CanError(f"program.pio has no `jmp {label}` to close the bit loop")


def sample_offset(program: Program, label: str = "engine") -> int:
    """SM cycles into a bit at which `in pins, 1` samples the line.

    The read-back is the second instruction of the loop, so it samples one cycle plus
    the `out`'s delay into the bit time. `CanDriver` multiplies it by CLKDIV to know
    when the ISR has been filled, which is what stops `send()` from parking the engine
    before the last sample has been pushed.
    """
    word = program.instrs[program.labels[label]]
    if (word >> 13) != 3:
        raise CanError(f"program.pio does not start {label} with an `out`")
    return 1 + ((word >> 8) & 0x1F)


def clkdiv_for(bit_ticks: int, sm_cycles: int = None) -> int:
    """The CLKDIV integer part that puts `bit_ticks` core cycles in one bit time."""
    if sm_cycles is None:
        sm_cycles = SM_CYCLES_PER_BIT
    if sm_cycles <= 0 or bit_ticks % sm_cycles:
        raise CanError(
            f"a bit time of {bit_ticks} core cycles is not a whole number of "
            f"{sm_cycles} SM cycles, so no CLKDIV puts this program on that bit time"
        )
    clkdiv = bit_ticks // sm_cycles
    if not 1 <= clkdiv <= 0xFFFF:
        raise CanError(f"CLKDIV {clkdiv} for {bit_ticks} core cycles per bit is out of range")
    return clkdiv


@dataclass
class TxResult:
    """What one `send()` put on the wire, and what came back."""

    identifier: int
    data: bytes
    rtr: bool
    dlc: int
    wire_bits: int                   # bits on the wire, stuffing and INTERMISSION included
    stuff_bits: int                  # of them, the ones sec. 5 inserted
    tx_words: list[int]              # what the host pushed
    readback: list[int]              # what the PIO read back, one word per TX word
    sof_cycle: int                   # core cycle the SOF `out` executed in
    acked: bool                      # the ACK SLOT read dominant on the readback
    bit_errors: list[tuple[int, int]] = field(default_factory=list)
                                        # (word index, the bits that differ)

    @property
    def bus_bits(self) -> int:
        return len(self.readback) * 32


class CanDriver:
    """Load fw/can/program.pio into one engine and put CAN frames on the bus.

    The engine is left parked in `idle` with the line released, which is BUS IDLE
    (sec. 3.1.5). `send()` starts one frame by forcing `jmp engine` at a core cycle
    the driver picks, streams the frame's words into the TX FIFO while the engine
    shifts them out, and takes the readback out of the RX FIFO.
    """

    def __init__(self, bus, device, program: Program, bit_ticks: int,
                 engine: int = 0, can_pin: int = 0, tick_ns: float = 20.0) -> None:
        if not 0 <= can_pin <= PIN_LIMIT:
            raise ValueError(
                f"open drain needs a bidirectional uio pin 0..{PIN_LIMIT} "
                f"(docs/mvp-isa.md `### Open drain`), CAN_PIN={can_pin}"
            )
        if len(program.instrs) > 32:
            raise ValueError(
                f"program.pio needs {len(program.instrs)} words, the IMEM holds 32"
            )
        for name in ("idle", "engine"):
            if name not in program.labels:
                raise CanError(f"program.pio has no `{name}` label for the host to enter")
        self.bus = bus
        self.dev = device
        self.prog = program
        self.engine = engine
        self.can_pin = can_pin
        self.tick_ns = tick_ns
        self.sm = A_SM + 6 * engine
        self.imem = A_IMEM[engine]
        self.sm_cycles = loop_cycles(program)
        self.sample_cycles = sample_offset(program)
        self.bit_ticks = int(bit_ticks)
        self.clkdiv = clkdiv_for(self.bit_ticks, self.sm_cycles)
        self.phase: Optional[int] = None       # the engine's SM-cycle grid, mod CLKDIV
        self.sof_cycle: Optional[int] = None   # core cycle of the last frame's SOF
        self.peak_fifo = 0
        self.last: Optional[TxResult] = None
        # Core cycles of idle bus before a frame starts. Four bit times is more than
        # the three recessive bits of INTERMISSION that sec. 3.1.5 asks for, and the
        # engine is parked with the line released, so those are recessive.
        self.start_margin = 4 * self.bit_ticks

    # ---- the register words the program needs --------------------------------------
    @property
    def clkdiv_word(self) -> int:
        return (self.clkdiv & 0xFFFF) << 16

    @property
    def shiftctrl_word(self) -> int:
        """AUTOPULL and AUTOPUSH with both thresholds left at 0.

        A threshold field of 0 reads as 32 (docs/mvp-isa.md `## Conventions`,
        `thresh(f)`), which is what this program wants: 32 bits is one word in each
        direction. AUTOPULL is what makes the refill free -- the 32nd `out` pops in
        the same SM cycle -- and AUTOPUSH is what puts one readback word per 32 bits
        into the RX FIFO. Both shift directions are 0 here; they reset to 1
        (docs/mvp-regmap.md `**SMe_SHIFTCTRL**` in `### Per-engine registers`,
        bits 19 and 18; `iss/pio_iss.py:69` SHIFTCTRL_RESET = 0x000C_0000), so this
        word writes 0, which is what makes both shift registers walk MSB first --
        the order of the wire bits.
        """
        return (1 << 17) | (1 << 16)

    @property
    def execctrl_word(self) -> int:
        """Only the wrap window: the program never runs off its own end, and the wrap
        keeps a stray PC inside the idle preamble rather than in the middle of a bit."""
        return ((self.prog.wrap & 0x1F) << 12) | ((self.prog.wrap_target & 0x1F) << 7)

    @property
    def pinctrl_word(self) -> int:
        """Every window one pin wide and all of them on the CAN pin.

        The SET window is what `set pindirs, 0` uses to release the line, the OUT
        window is what `out pindirs, 1` writes the direction through, and IN_BASE is
        what `in pins, 1` reads the line back through. SIDESET_COUNT stays 0: the
        program has no side-set, and a stray side-set would win over the OUT channel
        on the same pin (docs/mvp-isa.md `### Pin writes`).
        """
        pin = self.can_pin
        return ((1 << 26) | (1 << 20) | (pin << 15) | (pin << 5) | pin)

    def ctrl_restart(self) -> int:
        """CTRL with SM_RESTART and the divider left alone.

        A restart clears ISR and `isr_cnt` (docs/mvp-isa.md `### Enable, restart`),
        which is what lines the read-back words up with the TX words: a frame that ends
        part way through a 32-bit word leaves those bits in the ISR, and the next frame
        would start shifting them out as if they were its own. The divider is *not*
        restarted, because its phase is what `send()` places the SOF edge on.
        """
        e = self.engine
        return (1 << (4 + e)) | (1 << e)

    def ctrl_enable(self, restart: bool = False) -> int:
        """CTRL with this engine enabled, and with its divider restarted if asked.

        A restart in cycle h gives the first SM cycle in h+2 (docs/mvp-isa.md
        `### Clock divider`), which is what `_sof_cycle` counts from.
        """
        e = self.engine
        return ((1 << (8 + e)) if restart else 0) | (1 << e)

    # ---- bus and register access ---------------------------------------------------
    def step(self) -> None:
        """One core cycle: the ChipDevice performs at most one queued host access."""
        self.bus.step()

    def run(self, ticks: int) -> None:
        for _ in range(ticks):
            self.step()

    def write(self, addr: int, data: int) -> None:
        """Queue a register write and step the bus, so it is performed on this cycle."""
        self.dev.write(addr, data & 0xFFFF_FFFF)
        self.step()

    def read(self, addr: int) -> int:
        """Queue a register read and step the bus until the answer is back."""
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.step()
            ticks += 1
            if ticks > READ_LIMIT:
                raise CanError(f"read of word address {addr:#04x} never came back")
        return future.result()

    def flush(self) -> None:
        """Step until every queued host access has been performed."""
        while not self.dev.idle:
            self.step()

    def fdebug(self) -> int:
        return self.read(A_FDEBUG)

    def clear_fdebug(self) -> None:
        self.write(A_FDEBUG, 0xFFFF_FFFF)

    def _level(self, tx: bool) -> int:
        shift = FLEVEL_TX0 if tx else FLEVEL_RX0
        return (self.read(A_FLEVEL) >> (8 * self.engine + shift)) & 0xF

    # ---- engine setup --------------------------------------------------------------
    def load(self) -> None:
        """Write the register file, start the engine and find its SM-cycle grid.

        The order is docs/mvp-regmap.md `### Typical host sequences`: stop the
        engine, burst the IMEM, the four configuration registers, enable. What this
        adds is the divider restart at a *chosen* core cycle, and then a measurement
        of where the engine's SM cycles actually fall, because `CLKDIV_RESTART` fixes
        the phase and no register reports it (docs/mvp-isa.md `### Clock divider`).
        """
        e = self.engine
        self.write(A_CTRL, 0)
        for k, word in enumerate(self.prog.instrs):
            self.write(self.imem + k, word)
        self.write(self.sm + SM_CLKDIV, self.clkdiv_word)
        self.write(self.sm + SM_EXECCTRL, self.execctrl_word)
        self.write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word)
        self.write(self.sm + SM_PINCTRL, self.pinctrl_word)
        self.flush()
        self.park()
        # A restart in cycle h gives en = 0 in h+1 and the first SM cycle in h+2, and
        # then one every CLKDIV cycles, so the grid is at cycles == h+2 (mod CLKDIV).
        # h+2 == CLKDIV-1 (mod CLKDIV) is what `send()` needs, so restart at
        # h == CLKDIV-3 (mod CLKDIV); the measurement below checks that the host's
        # cycle counting and the divider really agree, and retries if they do not.
        want_h = (self.clkdiv - 3) % self.clkdiv
        for _ in range(4):
            while self.bus.t % self.clkdiv != want_h:
                self.step()
            self.write(A_CTRL, self.ctrl_enable(restart=True))
            phase = self._measure_phase()
            if phase == (self.clkdiv - 1) % self.clkdiv:
                self.phase = phase
                self.run(3 * self.bit_ticks)
                return
            want_h = (want_h + phase - (self.clkdiv - 1)) % self.clkdiv
        raise CanError(
            "could not put the engine's SM cycles where a frame has to start; "
            f"measured phase {phase}, wanted {(self.clkdiv - 1) % self.clkdiv}"
        )

    def _measure_phase(self) -> int:
        """The core cycles on which this engine has an SM cycle, modulo CLKDIV.

        Measured, not derived: `program.pio` idles in a two-instruction loop
        (`set pindirs, 0` / `jmp idle`), so SM0_ADDR takes two different values on
        two consecutive SM cycles and changes once per SM cycle. Queueing
        CLKDIV + 2 reads and stepping gives the PC of every cycle in that window, and
        the cycle at which it first changes is the SM cycle that produced it: the host
        read returns the value at the *start* of the access cycle (docs/mvp-isa.md
        `### Host-to-engine timing`, "register values read by an instruction or by the
        host"), and the PC changes at the end of the cycle that executed it.
        """
        n = self.clkdiv + 2
        futures = [self.dev.read(self.sm + SM_ADDR) for _ in range(n)]
        t0 = self.bus.t
        self.run(n)
        for i in range(1, n):
            if futures[i].result() != futures[i - 1].result():
                return (t0 + i - 1) % self.clkdiv
        raise CanError("the engine did not execute a single SM cycle while measuring")

    def park(self) -> None:
        """Force `jmp idle`, so the line is released and the bus is idle."""
        self.drain([])
        self.write(self.sm + SM_INSTR, self.prog.labels["idle"])
        self.flush()
        self.run(2)

    # ---- one frame ------------------------------------------------------------------
    def _sof_cycle(self) -> int:
        """The core cycle the SOF's `out` has to execute in.

        Two things have to hold at once:

        * The engine's SM cycles are at cycles == phase (mod CLKDIV), and a host
          instruction written in cycle f is executed in f+1 whatever the divider says
          (docs/mvp-isa.md `### EXEC`). So forcing `jmp engine` in cycle c-2 makes the
          engine execute the jump in cycle c-1 and the first `out` in cycle c, as long
          as c is on the grid.
        * The edge that puts the SOF on the bus is one core cycle after the `out`
          (docs/mvp-isa.md `## Timing`, `T_OUT = 1`), and `verif/vip/core.py` resolves
          it at the end of that tick, so the *bus* sees it at tick c+1. The reference
          model has no propagation delay and no oscillator error, so both the node and
          the checker lock their bit grid on the SOF edge, and the node can only hard
          synchronise on an edge that falls on one of its own bit boundaries -- which
          are the multiples of the bit time, because its grid starts at tick 0. c+1
          therefore has to be a multiple of the bit time, i.e. c == -1 (mod bit_ticks).
        """
        if self.phase is None:
            raise CanError("load() has not run, so the engine's SM-cycle grid is unknown")
        earliest = self.bus.t + self.start_margin
        c = earliest + ((self.bit_ticks - 1 - earliest) % self.bit_ticks)
        if c % self.clkdiv != self.phase:
            raise CanError(
                f"the SOF cycle {c} is not on the engine's SM-cycle grid "
                f"({self.phase} mod {self.clkdiv}); load() put it there"
            )
        return c

    def drain(self, into: List[int]) -> None:
        """Take every word out of the RX FIFO into `into`."""
        while self._level(tx=False):
            into.append(self.read(A_RXF[self.engine]))
        self.peak_fifo = max(self.peak_fifo, self._level(tx=True))

    def send(self, identifier: int, data: bytes = b"", rtr: bool = False,
             dlc: Optional[int] = None) -> TxResult:
        """Put one DATA FRAME (or REMOTE FRAME with rtr=True) on the bus.

        Blocks until the frame and its INTERMISSION are out and the engine is parked
        again. The returned `TxResult` carries the wire bits, the words that were
        pushed, the words the PIO read back, and the two things the host can tell from
        the readback alone: whether the ACK SLOT read dominant, and whether any bit on
        the bus came back different from what was sent.
        """
        wire, roles = wire_bits(identifier, data, rtr, dlc)
        frame_words = tx_words(wire)
        ack_wire = roles.index("ack")
        ack_mask = ack_bit_mask(ack_wire // 32, ack_wire % 32)
        # One extra all-recessive word, so the engine never stalls on the empty TX FIFO
        # after the frame. A stall there would cost no bit time (the line is recessive by
        # then) but it would raise FDEBUG.TXSTALL, and FDEBUG is how this firmware says
        # "the host kept up" -- which is a statement worth being able to make.
        words = frame_words + [SENTINEL]

        self.park()
        self.clear_fdebug()
        self.write(A_CTRL, self.ctrl_restart())     # an empty ISR, so read-back aligns
        self.flush()
        c = self._sof_cycle()
        self.run(c - 3 - self.bus.t)
        # The first TX word has to be in the FIFO before the jump: AUTOPULL's
        # background refill runs on the forced cycle itself, and an OUT with an empty
        # OSR stalls instead (docs/mvp-isa.md `### Reference step`).
        self.write(A_TXF[self.engine], words[0])
        self.write(self.sm + SM_INSTR, self.prog.labels["engine"])
        self.flush()
        if self.bus.t != c - 1:
            raise CanError(f"the frame start slipped: at cycle {self.bus.t}, wanted {c - 1}")

        readback: List[int] = []
        pending = list(words[1:])
        # The last bit of the last word is at this cycle. The engine is stopped at the
        # park below one cycle later, which is why the padding is recessive: the line
        # is then left released for as long as the host takes to come back.
        last_bit = c + (len(frame_words) * 32 - 1) * self.bit_ticks
        last_sample = last_bit + self.sample_cycles * self.clkdiv + 1
        while self.bus.t <= last_sample:
            if pending and self._level(tx=True) < regmap.fifo_full_level():
                self.write(A_TXF[self.engine], pending.pop(0))
            self.drain(readback)
            self.step()
        # FDEBUG first, before the last drain: the read returns the state at the cycle
        # it is performed on, and at CLKDIV 1 the engine's next `out` -- which stalls,
        # because the frame's last word was the last one there is -- is only a couple of
        # cycles away by then. The line is recessive there, so such a stall costs no bit
        # time, but it is not what this check is about.
        stalled = self.fdebug()
        self.drain(readback)
        if pending:
            raise CanError(f"{len(pending)} TX word(s) never reached the FIFO")
        self.park()
        if stalled:
            raise CanError(
                f"FDEBUG {stalled:#010x} during the frame: the engine stalled on a FIFO, "
                "which stretches a bit time; the host has to keep both FIFOs flowing"
            )
        if len(readback) < len(frame_words):
            raise CanError(
                f"the PIO read back {len(readback)} words for {len(frame_words)} pushed"
            )
        readback = readback[:len(frame_words)]

        # The readback of a word nobody else drove is its complement (a released line
        # reads high), except in the ACK SLOT, where a receiver drove the line low. A
        # bit that is neither is a BIT ERROR of sec. 6.1.
        acked = any((word & ack_mask) == 0 for word in readback)
        bit_errors = [
            (i, (readback_word(word) ^ got) & ~ack_mask & 0xFFFF_FFFF)
            for i, (word, got) in enumerate(zip(frame_words, readback))
            if (readback_word(word) ^ got) & ~ack_mask
        ]
        result = TxResult(
            identifier=identifier, data=bytes(data), rtr=rtr,
            dlc=len(data) if dlc is None else dlc,
            wire_bits=len(wire), stuff_bits=roles.count("stuff"),
            tx_words=frame_words, readback=readback,
            sof_cycle=c, acked=acked, bit_errors=bit_errors,
        )
        self.sof_cycle, self.last = c, result
        return result

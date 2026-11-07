"""Host side of the MVP I2C firmware: the register writes that run fw/i2c/program.pio.

Written against docs/mvp-regmap.md, not against the ISS: every address and field
below is the word address and bit position of the register map, so the same driver
drives the RTL once it exists. Host accesses go through the queue of
`iss.pio_iss.ChipDevice` (one access per core cycle, in order) and the driver is what
steps the `verif.vip.core.Bus` while it waits for a read to come back.

One PIO program, four host entry points, all forced through `SMx_INSTR` because
`SMx_ADDR` is read-only and there is no IRQ to wake the engine with:

    set pindirs, 0     release both lines; the engine then idles in `init`
    set y, n           n = number of 32-bit TX words the next phase will consume
    jmp main           run a phase: START, n words, park in `idle`
    jmp stop_seq       STOP, park in `idle`

A phase is one word per byte, nine bit times per word, so the same program code
writes a byte, reads a byte back or answers with an ACK: the host says which by the
bits it pushes (see `tx_word`). A read phase leaves the bus with SCL low and SDA
released, which is where a repeated START starts from, so `write_read` below gets one
by jumping to `main` a second time.
"""
from __future__ import annotations

from fw import regmap

from typing import Callable, List, Optional, Sequence

from asm.pioasm import Program

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) -----------------
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF0, A_RXF0 = regmap.txf_addrs()[0], regmap.rxf_addrs()[0]
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_IMEM = regmap.imem_bases()              # engine e, IMEM word k at A_IMEM[e] + k
A_SM = regmap.sm_base()                        # SMe_* at A_SM + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

FLEVEL_TX0, FLEVEL_RX0 = 0, 4      # shift of the engine-0 field of FLEVEL 3:0 / 7:4

# Pin assignment of the program. fw/i2c/program.pio carries the same two numbers in
# its `.define` lines; `check_program` makes sure the two files still agree.
DEFAULT_SDA_PIN, DEFAULT_SCL_PIN = 0, 1


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


SIDE_SCL_LOW, SIDE_SCL_RELEASED = 1, 0   # the program's side-set value for SCL
TX_DATA_SHIFT = 24             # the eight data bits of a TX word live in bits 31:24
TX_NINTH_BIT = 23              # the ninth bit time of a TX word
RX_DATA_SHIFT = 1              # the eight sampled data bits of an RX word: 8:1
RX_NINTH_BIT = 0               # the ninth sampled bit of an RX word
PHASE_LIMIT = 400_000          # core cycles one phase may take before we give up


def tx_word(byte: int, ack: bool = False) -> int:
    """One byte on the bus, as the program consumes it.

    Bits 31:24 are the eight data bits, MSB first: `out pindirs, 1` walks the OSR from
    the top because SHIFTCTRL.OUT_SHIFTDIR is 0. What it walks out is a *direction*,
    not a level, and the level register stays 0 for an open-drain line
    (docs/mvp-isa.md `### Open drain`), so 1 pulls the line low and 0 releases it. The
    byte itself is a level (1 = released), so it is inverted here -- this is the
    inversion docs/mvp-isa.md `### Open drain` asks of the host.

    Bit 23 is the ninth bit time, after the eighth. On a byte the program sends it is
    the ACK clock, where the slave drives, so it has to leave SDA released (0). On a
    byte the program reads it is the master's own ACK: 1 pulls SDA low (ACK) and 0
    releases it for a NAK. That one is not inverted, because it was never a level.
    """
    if not 0 <= byte <= 0xFF:
        raise ValueError(f"byte {byte!r} does not fit in 8 bits")
    return ((~byte & 0xFF) << TX_DATA_SHIFT) | ((1 << TX_NINTH_BIT) if ack else 0)


def rx_byte(word: int) -> int:
    """The data byte of an RX word.

    The program samples nine bit times, MSB first, and SHIFTCTRL.IN_SHIFTDIR is 0, so
    every sample shifts what is already there one place up: the first bit sampled ends
    in bit 8 and the ninth in bit 0. An RX word is therefore the received byte shifted
    left one, with the ninth bit in bit 0.
    """
    return (word >> RX_DATA_SHIFT) & 0xFF


def rx_ninth(word: int) -> int:
    """The ninth sampled bit as it came off the bus: 1 = SDA was high, 0 = SDA low."""
    return (word >> RX_NINTH_BIT) & 1


def rx_ack(word: int) -> bool:
    """True when the ACK clock read SDA low, which is how an ACK looks on the wire.

    On a byte the program sent that is the slave answering; on a byte it read it is the
    master seeing its own ACK bit come back, since the program samples what it drives.
    """
    return rx_ninth(word) == 0


def read_word(ack: bool = False) -> int:
    """The TX word for a byte the program *reads*.

    All eight data bits are released, so the slave drives them, and the ninth bit is
    the master's own ACK: 1 pulls SDA low for an ACK, 0 releases it for a NAK.
    """
    return tx_word(0xFF, ack)


def jmp_word(address: int) -> int:
    """`jmp <address>` with no condition, no side-set and no delay: JMP is opcode 000
    and bits 12:0 are then all zero, so the word is the address."""
    return address & 0x1F


def set_y_word(value: int) -> int:
    """`set y, value`: opcode 111, destination Y (010), data in bits 4:0.

    Y is the word counter: the program decrements it once per byte and parks when it
    runs out, so this is how a phase is told how many words to consume.
    """
    if not 0 <= value <= 31:
        raise ValueError(f"set y, {value}: the data field is 5 bits")
    return (7 << 13) | (2 << 5) | value


def side_field(program: Program, value: int) -> int:
    """The 5-bit delay/side-set field of an instruction carrying `side value`.

    How much of the delay/side-set field is side-set depends on the program's
    `.side_set`, not on the instruction (docs/mvp-isa.md `### Delay/side-set field`),
    and with `opt` the topmost of those bits is an enable that is set for any non-zero
    value. The program also decides which way round "SCL low" and "SCL released" are:
    here the side-set writes SCL's direction, so 1 is low.
    """
    sc = program.sideset_bits
    if sc == 0:
        if value:
            raise ValueError("the program has no .side_set, so it cannot carry a side value")
        return 0
    if not 0 <= value < (1 << sc):
        raise ValueError(f"side {value} does not fit in the program's {sc} side-set bit(s)")
    bits = value | ((1 << (sc - 1)) if (program.sideset_opt and value) else 0)
    return bits << (5 - sc)


def retarget(program: Program, scl_pin: int) -> Program:
    """The same program with `wait 1 gpio` aimed at another SCL pin.

    Exactly one instruction in the program carries a pin number: the `wait 1 gpio` that
    watches SCL, whose Index field is an absolute pin index and not a window offset
    (docs/mvp-isa.md `### Field values`, WAIT with the GPIO source). Everything else is
    a PINCTRL field, which the host writes anyway. So a program assembled for one pin
    pair runs on any other pair by patching that one field, which is what the RP2040 SDK
    does when it encodes the wait at run time.
    """
    if not 0 <= scl_pin <= 31:
        raise ValueError(f"SCL pin {scl_pin} does not fit the 5-bit WAIT index")
    words, patched = [], 0
    for word in program.instrs:
        is_wait_gpio = (word >> 13) == 1 and ((word >> 5) & 3) == 0
        if is_wait_gpio and (word >> 7) & 1:          # `wait 1 gpio`
            word = (word & ~0x1F) | scl_pin
            patched += 1
        words.append(word)
    if not patched:
        raise I2cError(
            "the program has no `wait 1 gpio` to aim at SCL, so it cannot absorb clock "
            "stretching"
        )
    return Program(
        name=program.name, instrs=words, origin=program.origin,
        wrap_target=program.wrap_target, wrap=program.wrap,
        sideset_bits=program.sideset_bits, sideset_opt=program.sideset_opt,
        sideset_pindirs=program.sideset_pindirs, labels=program.labels,
    )


class I2cError(RuntimeError):
    """The engine or the bus did something the driver cannot make sense of."""


def check_program(program: Program, scl_pin: int) -> None:
    """Fail loudly if the driver and the .pio file disagree about the pins.

    The program names its pins in `.define` lines, which the assembler does not hand
    back, so the instructions that pin them down are compared instead: the bit loop has
    to wait for SCL to go high (and, after `retarget`, on the pin the host configured),
    and it has to shift SDA's direction out of the OSR one bit at a time.
    """
    waits = [
        w for w in program.instrs
        if (w >> 13) == 1 and (w & 0x1F) == scl_pin and ((w >> 5) & 3) == 0 and ((w >> 7) & 1)
    ]
    if not waits:
        raise I2cError(
            f"program.pio has no `wait 1 gpio {scl_pin}`, so it cannot wait for SCL to go "
            f"high and cannot absorb clock stretching"
        )
    if not any((w >> 13) == 3 and ((w >> 5) & 7) == 4 and (w & 0x1F) == 1 for w in program.instrs):
        raise I2cError("program.pio never puts an OSR bit on SDA with `out pindirs, 1`")
    if program.sideset_bits != 1 or program.sideset_opt or not program.sideset_pindirs:
        raise I2cError(
            f"the driver programs PINCTRL for `.side_set 1 pindirs`, program.pio says "
            f"{program.sideset_bits} bit(s), opt={program.sideset_opt}, "
            f"pindirs={program.sideset_pindirs}"
        )
    for name in ("main", "idle", "stop_seq", "init"):
        if name not in program.labels:
            raise I2cError(f"program.pio has no `{name}` label for the host to enter")


class I2cDriver:
    """Load fw/i2c/program.pio into one engine and drive I2C transactions with it."""

    def __init__(
        self,
        bus,
        device,
        program: Program,
        clkdiv: int = 14,
        engine: int = 0,
        sda_pin: int = DEFAULT_SDA_PIN,
        scl_pin: int = DEFAULT_SCL_PIN,
        bus_free_ticks: int = 70,
    ) -> None:
        if scl_pin != sda_pin + 1:
            raise ValueError(
                f"the program drives both lines through one two-pin window, so it needs "
                f"SCL = SDA + 1; got SDA={sda_pin}, SCL={scl_pin}"
            )
        if not 0 <= sda_pin <= 6:
            raise ValueError(
                f"open drain needs two uio pins (docs/mvp-isa.md `### Open drain`), "
                f"SDA={sda_pin} leaves none for SCL"
            )
        if not 1 <= clkdiv <= 0xFFFF:
            raise ValueError(f"CLKDIV integer part {clkdiv} is out of range")
        if len(program.instrs) > 32:
            raise ValueError(f"the program needs {len(program.instrs)} words, the IMEM holds 32")
        program = retarget(program, scl_pin)
        check_program(program, scl_pin)
        self.bus = bus
        self.dev = device
        self.prog = program
        self.engine = engine
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.clkdiv = clkdiv
        self.bus_free_ticks = bus_free_ticks
        self.peak_tx = self.peak_rx = 0
        self._main = program.labels["main"]
        self._stop_seq = program.labels["stop_seq"]
        init = program.labels["init"]
        # Parked means "the program is not shifting anything": either the two-word
        # idle preamble of the wrap region or the one-word `idle` spin at the end of a
        # phase. Both hold SCL low (or released) with SDA released, i.e. an idle bus
        # that a fresh START or a repeated START can both grow out of.
        self._parked = frozenset((init, init + 1, program.labels["idle"]))

    # ---- the register words the program needs --------------------------------------
    @property
    def clkdiv_word(self) -> int:
        return (self.clkdiv & 0xFFFF) << 16

    @property
    def execctrl_word(self) -> int:
        """SIDE_PINDIR, because the side-set drives a direction and open drain is a
        direction; no SIDE_EN, because the program side-sets every instruction; and the
        wrap window, so that running off the end parks in the idle preamble."""
        return (1 << 29) | ((self.prog.wrap & 0x1F) << 12) | ((self.prog.wrap_target & 0x1F) << 7)

    @property
    def shiftctrl_word(self) -> int:
        """AUTOPUSH at nine bits gives one RX word per byte with no explicit push,
        AUTOPULL is off because the program pulls explicitly (and `pull block` would
        be a no-op with autopull on and a full OSR), OUT_SHIFTDIR 0 shifts MSB first
        and IN_SHIFTDIR 0 shifts the samples in from the bottom, so a received byte
        ends up in bits 7:0 with the ninth sampled bit in bit 8."""
        return (9 << 25) | (9 << 20) | (0 << 19) | (0 << 18) | (0 << 17) | (1 << 16)

    @property
    def pinctrl_word(self) -> int:
        """Every window sits on the two I2C pins. The SET window is one pin wide
        because `set pindirs, 1` is what makes a START or a STOP: it has to reach SDA
        alone, since SCL has to stay released while SDA moves. Its other pin is covered
        by the side-set, which writes SCL's direction on every instruction anyway."""
        return (
            (1 << 29)                     # SIDESET_COUNT 1: SCL's direction
            | (1 << 26)                   # SET_COUNT 1: SDA
            | (1 << 20)                   # OUT_COUNT 1: SDA
            | (self.sda_pin << 15)        # IN_BASE: `in pins, 1` reads SDA
            | (self.scl_pin << 10)        # SIDESET_BASE
            | (self.sda_pin << 5)         # SET_BASE
            | self.sda_pin                # OUT_BASE
        )

    @property
    def sm(self) -> int:
        """Word address of this engine's `SMe_CLKDIV`."""
        return A_SM + 6 * self.engine

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
            if ticks > PHASE_LIMIT:
                raise I2cError(f"read of word address {addr:#04x} never came back")
        return future.result()

    def wait_until(self, what: Callable[[], bool], limit: int = PHASE_LIMIT) -> None:
        ticks = 0
        while not what():
            self.step()
            ticks += 1
            if ticks > limit:
                raise I2cError(f"timed out after {ticks} core cycles")

    # ---- engine setup --------------------------------------------------------------
    def load(self) -> None:
        """The host sequence of docs/mvp-regmap.md `### Typical host sequences`.

        The engine is left running in the idle preamble rather than jumped into the
        program: a phase forces `jmp main` itself, once it knows how many words it is
        about to push, so no START can go out before the data is queued.
        """
        self.reg_write(A_CTRL, 0)                                        # stop the engine
        for k, word in enumerate(self.prog.instrs):                      # IMEM burst
            self.reg_write(A_IMEM[self.engine] + k, word)
        self.reg_write(self.sm + SM_CLKDIV, self.clkdiv_word)
        self.reg_write(self.sm + SM_EXECCTRL, self.execctrl_word)
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word)
        self.reg_write(self.sm + SM_PINCTRL, self.pinctrl_word)
        # Step 4 of the same sequence: the initial pin state. At reset every pin is an
        # input, so this is only needed to undo what a previous transaction left driven.
        self.force(self.prog.instrs[self.prog.labels["init"]], side=SIDE_SCL_RELEASED)
        self.reg_write(A_CTRL, self._ctrl_start)
        # Then leave the bus alone for the bus-free time. The first forced instruction of
        # a phase pulls SCL low, which ends the high period the bus has had since reset,
        # and a timing checker that measures the first period from tick 0 (the VIP one
        # does, on purpose) wants that period to be a real one and not a reset artefact.
        self.run(self.bus_free_ticks)
        self.wait_parked()

    @property
    def _ctrl_start(self) -> int:
        """CTRL with this engine enabled and its divider restarted, so the first SM
        cycle is two core cycles after this access (docs/mvp-isa.md `### Clock divider`)."""
        e = self.engine
        return (1 << (8 + e)) | (1 << (4 + e)) | (1 << e)

    def force(self, word: int, side: int = SIDE_SCL_LOW) -> None:
        """Push one instruction into the engine's EXEC latch.

        It executes in the next core cycle whatever SM_ENABLE and the divider say, it
        does not advance the PC and its delay is ignored (docs/mvp-isa.md `### EXEC`).

        `side` defaults to holding SCL low, which is what every forced instruction of
        this driver wants: the engine is parked between phases with SCL low, and a
        forced instruction that released SCL for even one cycle would put a short SCL
        pulse on the bus, which is both a tLOW and a tHIGH the checker rightly objects
        to. `SIDE_SCL_RELEASED` is for the one instruction that wants an idle bus.
        """
        word = (word & ~(0x1F << 8)) | (side_field(self.prog, side) << 8)
        self.reg_write(self.sm + SM_INSTR, word)

    @property
    def pc(self) -> int:
        """`SMx_ADDR`, the engine's PC. There is no IRQ in the MVP, so this and the
        FIFO levels are all the host can see of the program's progress."""
        return self.reg_read(self.sm + SM_ADDR) & 0x1F

    @property
    def parked(self) -> bool:
        return self.pc in self._parked

    def wait_parked(self) -> None:
        self.wait_until(lambda: self.parked)

    def wait_left_parked(self) -> None:
        """Step until the engine has left the parked spin, i.e. a forced jump landed.

        A forced instruction runs in the *next* core cycle, so reading SM0_ADDR right
        after writing SM0_INSTR still shows the old PC: without this the caller would
        go on believing the phase it just started has finished.
        """
        self.wait_until(lambda: self.pc not in self._parked)

    def fdebug(self) -> int:
        return self.reg_read(A_FDEBUG)

    def clear_fdebug(self) -> None:
        self.reg_write(A_FDEBUG, 0xFFFF_FFFF)

    # ---- one phase of a transfer ---------------------------------------------------
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

        Both are four deep (docs/mvp-regmap.md `### Engine state`), so anything below
        four means the host kept up with the engine instead of relying on its stalls.
        """
        return max(self.peak_tx, self.peak_rx)

    def _stream(self, words: Sequence[int]) -> List[int]:
        """Push `words` one at a time and take back one RX word per byte pushed.

        The engine eats one word every nine bit times, so a word only goes in once
        the TX FIFO has room, and the RX FIFO is emptied as we go: both are four deep
        and the engine stalls (FDEBUG.TXSTALL / RXSTALL) if either fills up.
        """
        received: List[int] = []
        for word in words:
            self.wait_until(lambda: self._tx_level() < regmap.fifo_full_level())
            self.reg_write(A_TXF0, word)
            while self._rx_level():
                received.append(self.reg_read(A_RXF0))
        while len(received) < len(words):          # the last bytes are still in flight
            if self._rx_level():
                received.append(self.reg_read(A_RXF0))
            else:
                self.step()
        return received

    def phase(self, words: Sequence[int]) -> List[int]:
        """Run one phase: wait for the engine to park, load the word count, jump to
        `main` (which sends the START), stream the words, and wait for it to park
        again. Returns the RX words, one per byte, the first being the address byte.
        """
        if not 1 <= len(words) <= 31:
            raise I2cError(
                f"a phase of {len(words)} words: the program counts them in Y, and only "
                f"`set y, n` can load it, so 1..31 words per phase"
            )
        self.wait_parked()
        self.force(set_y_word(len(words)))          # Y = words this phase will consume
        self.force(jmp_word(self._main))           # START
        return self._stream(words)

    def stop(self) -> None:
        """Put a STOP on the bus, then wait out the bus-free time tBUF."""
        self.wait_parked()
        self.force(jmp_word(self._stop_seq))
        self.wait_left_parked()             # the forced jump has landed
        self.wait_parked()                  # and the STOP sequence is done
        self.run(self.bus_free_ticks)

    # ---- transactions --------------------------------------------------------------
    def write(self, address: int, data: Sequence[int] = ()) -> List[bool]:
        """START, address+W, every data byte, then park with the bus still held.

        No STOP goes out: this is the write half of a combined transfer, so the bus is
        left with SCL low and SDA released. Call `write_read` or `stop` next. Returns
        one flag per data byte, True when the slave ACKed it. The eight data bits of a
        sent byte are the PIO's own SDA coming back, so only the ninth bit says
        anything about the slave.
        """
        words = [tx_word(self._control_byte(address, 0))] + [tx_word(b) for b in data]
        received = self.phase(words)
        self._check_ack(address, received[0], "write")
        return [rx_ack(word) for word in received[1:]]

    def read(self, address: int, count: int) -> bytes:
        """START, address+R, `count` bytes with an ACK on all but the last, STOP."""
        return self.write_read(address, (), count)[1]

    def write_read(self, address: int, data: Sequence[int], count: int) -> tuple:
        """One bus transaction: write `data`, repeated START, read `count` bytes.

        The read phase re-enters `main` while the write phase left the bus in the
        state a repeated START needs, so the two halves are one transaction with no
        STOP in between. Returns (one ACK flag per written byte, the bytes read).
        """
        if count < 1:
            raise ValueError(f"read {count} bytes: I2C reads at least the address byte")
        # With nothing to write the read address goes out in the read phase itself: a
        # plain read is one transaction, START address+R data STOP. Only a real write
        # needs the bus held open for a second phase, which is what makes the repeated
        # START between them.
        acked = self.write(address, data) if data else []
        words = [tx_word(self._control_byte(address, 1))]
        words += [read_word(ack=(i < count - 1)) for i in range(count)]
        received = self.phase(words)
        self._check_ack(address, received[0], "read")
        read = bytes(rx_byte(word) for word in received[1:])
        # On a read byte the ninth sampled bit is our own ACK bit coming back off SDA,
        # so comparing it with what was pushed checks that the ACKs reached the wire:
        # an ACK reads low, and the last byte is NACKed on purpose to end the read.
        wanted = [(i < count - 1) for i in range(count)]
        echoed = [rx_ack(word) for word in received[1:]]
        self.stop()
        if echoed != wanted:
            raise I2cError(
                f"the read bytes came back with ACK bits {echoed}, the slave was sent "
                f"{wanted}"
            )
        return acked, read

    def _check_ack(self, address: int, word: int, what: str) -> None:
        """The ninth sampled bit of an address byte is the slave's answer to it.

        A write byte's data bits are the PIO's own SDA, so only the address byte and
        the ACK clock carry anything the slave said; either way the ninth bit is the
        one the slave drove.
        """
        if not rx_ack(word):
            raise I2cError(f"nothing ACKed the {what} address {address:#04x}")

    @staticmethod
    def _control_byte(address: int, read: int) -> int:
        if not 0 <= address <= 0x7F:
            raise ValueError(f"I2C address must be 7 bits, got {address:#x}")
        return (address << 1) | read



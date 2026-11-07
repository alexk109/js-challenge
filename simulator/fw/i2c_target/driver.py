"""Host side of the MVP I2C target firmware: the EEPROM and the register writes that run
fw/i2c_target/program.pio.

Written against docs/mvp-regmap.md, not against the ISS: every address and field below is
the word address and bit position of the register map, so the same driver drives the RTL
once it exists. Host accesses go through the queue of `iss.pio_iss.ChipDevice` (one access
per core cycle, in order) and this module is what steps the `verif.vip.core.Bus` while it
waits for a read to come back.

Where the two halves of the job meet
------------------------------------
A controller driver owns the bus and starts each byte itself, so it can push the word for
a byte just before that byte. A target cannot: it is entered by the bus, and the byte it
is about to clock is whatever the master sends. So the division of labour here is

  * the program owns the wire: START, bit times, the ACK drive, the STOP;
  * the host owns the protocol: the 256-byte image, the write pointer, the page
    rollover, the 7-bit address, and the *answer* to each byte.

and the two talk through the FIFOs, two 32-bit words per byte (see `data_word`,
`ack_word`). The host pushes the DATA word of a byte well before it is clocked, and the
ACK word *after* the eight data bits have gone by -- which is what lets it decide ACK or
NAK from the byte it has just read back, and therefore NACK an address that is not its
own. The cost of that decision is the low phase of the last bit time: the program is
parked in `pull block` with SDA released, which the master reads as a longer tLOW. That
is legal, it costs rate, and `fw/i2c_target/measure.py` measures how much rate.

Nothing here starts or stops a transaction. The program does that, from the bus.
"""
from __future__ import annotations

from fw import regmap

from collections import deque
from typing import Callable, List, Optional

from asm.pioasm import Program

# ---- register word addresses (docs/mvp-regmap.md `### Address map`) -----------------
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = regmap.A_CTRL, regmap.A_FSTAT, regmap.A_FDEBUG, regmap.A_FLEVEL
A_TXF0, A_RXF0 = regmap.txf_addrs()[0], regmap.rxf_addrs()[0]
A_DBG_PADOUT, A_DBG_PADOE, A_DBG_CFGINFO = regmap.A_DBG_PADOUT, regmap.A_DBG_PADOE, regmap.A_DBG_CFGINFO
A_IMEM = regmap.imem_bases()              # engine e, IMEM word k at A_IMEM[e] + k
A_SM = regmap.sm_base()                        # SMe_* at A_SM + 6e + i
SM_CLKDIV, SM_EXECCTRL, SM_SHIFTCTRL, SM_ADDR, SM_INSTR, SM_PINCTRL = range(6)

FLEVEL_TX0, FLEVEL_RX0 = 0, 4      # shift of the engine-0 field of FLEVEL 3:0 / 7:4

# Pin assignment of the program. fw/i2c_target/program.pio carries the same two numbers in
# its `.define` lines; `check_program` makes sure the two files still agree.
DEFAULT_SDA_PIN, DEFAULT_SCL_PIN = 0, 1


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)


#: Marker word the program pushes when it sees the bus go free (`mov isr, y`). Bit 31 is
#: the marker: the program samples SDA eight times per byte and autopushes at eight
#: (PUSH_THRESH 8), so a byte word is bits 7:0 of the ISR plus bit 8, which is the previous
#: ACK clock's sample -- bit 31 can never be set by a byte. Y is 32 bits wide but `set y`
#: only carries five bits of immediate, so the host loads it with a pulled word and
#: `mov y, osr` (`I2cTargetDriver.load`).
STOP_MARKER = 0x8000_001F
#: SM cycles one steady-state data bit time costs the program (`data` in program.pio),
#: excluding the stalls that wait for the master: `wait 0` (1 + LOW_HOLD) + `out pindirs` +
#: `wait 1` (1 + HIGH_HOLD) + `in pins` + `jmp !osre`, with **LOW_HOLD = HIGH_HOLD = 0** in
#: program.pio — five. Nothing computes with this number: `clkdiv_max()` sizes the divider
#: with `SM_CYCLES_FIRST_BIT`, because it is the *first* bit of a byte (the one after a
#: `pull block`) that has to fit in the master's low phase, not this one. It is kept as the
#: written-down cost of a bit time and re-exported from `fw.i2c_target`.
SM_CYCLES_PER_BIT = 5
PHASE_LIMIT = 400_000              # core cycles one host access may take before we give up


def data_word(byte: Optional[int] = None) -> int:
    """The DATA word: the eight SDA directions of the next byte, MSB first in bits 31:24.

    `byte = None` is a byte the slave *receives*: all eight drives release, and the
    master's own SDA stands. `byte` is a byte the slave *sends*: the data bits inverted,
    because what goes out on PINDIRS is a direction, not a level, and on an open-drain
    line 1 means "pull low" while a 1 in the byte means "released" (docs/mvp-isa.md
    `### Open drain`).
    """
    if byte is None:
        return 0
    if not 0 <= byte <= 0xFF:
        raise ValueError(f"byte {byte!r} does not fit in 8 bits")
    return (~byte & 0xFF) << 24


def ack_word(ack: bool) -> int:
    """The ACK word: bit 31 drives the ninth bit time, the ACK clock.

    1 pulls SDA low, which is an ACK; 0 releases it, which is a NAK. A byte the slave
    sends has to release it as well, so that the ninth `in pins, 1` samples the master's
    own ACK rather than ours.
    """
    return (1 << 31) if ack else 0


def rx_byte(word: int) -> int:
    """The data byte of an RX word: bits 7:0.

    The program samples SDA eight times per byte with SHIFTCTRL.IN_SHIFTDIR = 0 and
    autopushes at eight, so every sample shifts what is there one place up and the eight of
    them land in bits 7:0. The ninth sample, of the ACK clock, is discarded by the program
    (`mov isr, x` after it): the byte is already in the host's hands by then, and the host
    counts the bytes it sends rather than reading the master's ACK back.
    """
    return word & 0xFF


def is_stop_marker(word: int) -> bool:
    """Whether an RX word is the program's end-of-bus marker rather than a byte."""
    return bool(word & 0x8000_0000)


#: `pull block` (`0x80a0`) and `pull noblock` (`0x8080`): the drain instruction
#: docs/mvp-regmap.md `### Typical host sequences` names for emptying the TX FIFO without a
#: flush, and the one the program itself uses.
PULL_BLOCK, PULL_NOBLOCK = 0x80A0, 0x8080


def jmp_word(address: int) -> int:
    """`jmp <address>` with no condition, no side-set and no delay: JMP is opcode 000
    and bits 12:0 are then all zero, so the word is the address."""
    return address & 0x1F


def set_pindirs_word(value: int) -> int:
    """`set pindirs, <0|1>`: release or pull SDA low without touching the OSR."""
    return (7 << 13) | (4 << 5) | (value & 1)


#: `mov y, osr`: opcode 101, destination Y (010), source OSR (111). Together with a
#: `pull block` this is how the host loads a 32-bit Y, which `set y` cannot do (its data
#: field is five bits). The program pushes Y as the end-of-bus marker.
MOV_Y_OSR = (5 << 13) | (2 << 5) | 7


def set_y_word(value: int) -> int:
    """`set y, value`: opcode 111, destination Y (010), data in bits 4:0."""
    if not 0 <= value <= 31:
        raise ValueError(f"set y, {value}: the data field is 5 bits")
    return (7 << 13) | (2 << 5) | value


def poll_for(high_ticks: int, clkdiv: int) -> int:
    """How many times `poll` may look at SCL before the bus counts as free.

    `high_ticks` has to be the high period the master will *actually* run, not the mode's
    minimum: `I2cController` spreads the slack between 1/fSCL and the two half-period
    minimums, so at 50 kHz on a standard-mode bus the minimum is 200 ticks and the high
    period is 481.

    One turn of the poll loop is three SM cycles, and an SM cycle is `clkdiv + 1` core
    cycles (docs/mvp-isa.md `### Clock divider`), so `n` turns buy `3 * n * (clkdiv + 1)`
    core cycles of SCL-high. The smallest `n` that outlasts one master high period is the
    one to use: larger only risks being late for the next transaction, which the master
    starts after `tBUF`, and the next START would then look like a bit time.
    """
    per_turn = 3 * (clkdiv + 1)
    return max(1, -(-(high_ticks + 6) // per_turn))


def set_poll(program: Program, value: int) -> Program:
    """The same program with `poll`'s counter set to `value`.

    Exactly one instruction carries the number: the `set x, POLL` in front of `poll`, and
    its data field is five bits. So a program assembled for one bus runs on another by
    patching that one field, which is what the RP2040 SDK does when it encodes the wait at
    run time, and why the number is not simply assembled in.
    """
    if not 1 <= value <= 31:
        raise ValueError(f"poll {value}: the data field is 5 bits")
    at = program.labels["poll"] - 1
    word = program.instrs[at]
    if (word >> 13) != 7 or ((word >> 5) & 7) != 1:
        raise I2cTargetError(
            f"program.pio word {at} is not `set x, ...`; `poll` is not where this driver "
            f"expects it"
        )
    words = list(program.instrs)
    words[at] = (word & ~0x1F) | value
    return Program(
        name=program.name, instrs=words, origin=program.origin,
        wrap_target=program.wrap_target, wrap=program.wrap,
        sideset_bits=program.sideset_bits, sideset_opt=program.sideset_opt,
        sideset_pindirs=program.sideset_pindirs, labels=program.labels,
    )


#: Core cycles the program needs between the master's SCL falling and the first bit of a
#: byte reaching SDA: `idle`'s two waits, the jump into `bnd`, its wait, the release and
#: the pull, then the `out` itself -- six SM cycles, less the three core cycles T_IN has
#: already spent on the fall. The host's `pull block` is the only unbounded term in it.
SM_CYCLES_FIRST_BIT = 6


#: Core cycles the first bit of a byte does *not* get: the three T_IN cycles the fall has
#: already spent are counted above, and these are the margin for the `pull block` that
#: precedes it. It is the host's latency, and at 20 ns a tick the host spends a handful of
#: cycles answering, so a couple of dozen is generous -- see the measured margins in
#: fw/i2c_target/README.md.
FIRST_BIT_SLACK = 8


#: Ticks the master's START holds SDA low on top of the hold point `I2cController` puts its
#: first data bit at (`I2cController.first_hold`, which is `_hold_point(t_hd_sta)`): the two
#: ticks its `start_cond` phase spends before it begins the low half-period, plus the one
#: tick the bus takes to resolve the SDA release. This is not a number the firmware gets to
#: choose -- it is the length of the only window in which a START exists on the wire -- and
#: `start_window_ticks` below is checked against the *measured* windows in the tests.
START_SETUP = 3


def start_window_ticks(first_hold: int) -> int:
    """How many core cycles the master's START holds SDA low, from `first_hold`.

    `first_hold` is `I2cController.first_hold`, the tick (counted from the SDA fall that is
    the START) at which the master is allowed to put its first data bit on SDA. The START is
    therefore visible for `first_hold + START_SETUP` ticks.
    """
    return int(first_hold) + START_SETUP


def clkdiv_max(low_ticks: int, first_hold: int) -> int:
    """The largest CLKDIV the bus can be run at, which two separate things bound.

    **The first bit of a byte has to land inside the master's low phase.** Not the bit time:
    the steady-state bit needs five SM cycles, but the *first* bit of a byte follows a
    `pull block` and needs six, and it has to be on SDA before the master releases SCL. So
    it is the master's LOW half-period that bounds the divider, and eight core cycles are
    left as margin for the three T_IN cycles and the pull's granularity.

    **The START has to be seen at all.** `idle` recognises a START by polling
    `wait 0 gpio SDA_PIN`, and the engine only runs that instruction once per SM cycle, i.e.
    once per `CLKDIV + 1` core cycles. So the engine looks at SDA on a fixed grid, and a
    START whose SDA-low window is shorter than that grid can fall entirely between two looks
    and be missed -- after which the program is still in `idle` when the address byte is
    clocked, the master sees SDA released at the ACK clock, and the transaction comes back
    NACKed. There is no program-side fix for that: a polled `wait` cannot see a pulse
    shorter than one instruction, so the *divider* is what has to respect the window. Hence
    `CLKDIV + 1 <= first_hold + START_SETUP`, which is why `first_hold` is an argument here
    and not a constant.

    Both bounds are upper bounds on the divider and neither is implied by the other: at
    200 kHz the low phase is 142 ticks (which would allow CLKDIV 22) while the START window
    is 17 ticks (which allows 16), and the second one is the tighter -- see
    fw/i2c_target/README.md `### Rates`.
    """
    by_low_phase = (int(low_ticks) - FIRST_BIT_SLACK) // SM_CYCLES_FIRST_BIT
    by_start_window = start_window_ticks(first_hold) - 1
    return max(1, min(by_low_phase, by_start_window))


class I2cTargetError(RuntimeError):
    """The engine or the bus did something the driver cannot make sense of."""


def check_program(program: Program, sda_pin: int, scl_pin: int) -> None:
    """Fail loudly if the driver and the .pio file disagree about the pins or the shape.

    The program names its pins in `.define` lines, which the assembler does not hand
    back, so the instructions that pin them down are compared instead: it has to wait for
    SCL to go high *and* low, it has to shift SDA's direction out of the OSR, and it has
    to have no side-set at all, because a slave that drives SCL truncates the master's
    high period (see the header of fw/i2c_target/program.pio).
    """
    if program.sideset_bits:
        raise I2cTargetError(
            f"program.pio has {program.sideset_bits} side-set bit(s), but an I2C target "
            f"must not drive SCL: shortening the master's high period is a tHIGH violation"
        )
    waits = [
        w for w in program.instrs
        if (w >> 13) == 1 and (w & 0x1F) in (sda_pin, scl_pin) and ((w >> 5) & 3) == 0
    ]
    if not any((w >> 7) & 1 and (w & 0x1F) == scl_pin for w in waits):
        raise I2cTargetError(
            f"program.pio has no `wait 1 gpio {scl_pin}`, so it cannot follow the master's "
            f"SCL rising edges"
        )
    if not any((w >> 7) & 1 == 0 and (w & 0x1F) == scl_pin for w in waits):
        raise I2cTargetError(
            f"program.pio has no `wait 0 gpio {scl_pin}`, so it cannot follow the master's "
            f"SCL falling edges and would drive SDA while SCL is high"
        )
    if not any((w >> 13) == 3 and ((w >> 5) & 7) == 4 and (w & 0x1F) == 1 for w in program.instrs):
        raise I2cTargetError("program.pio never puts an OSR bit on SDA with `out pindirs, 1`")
    for name in ("idle", "ack", "bnd", "data", "poll", "stop_path"):
        if name not in program.labels:
            raise I2cTargetError(f"program.pio has no `{name}` label")

class EepromImage:
    """The 24C02 the program pretends to be: `size` bytes in `page_size`-byte pages.

    Every rule is the 24C02's, and `verif.vip.i2c.I2cEepromTarget` implements the same
    ones, so the firmware is measured against a reference that agrees with it by
    construction rather than by luck: the control byte is `1010 A2 A1 A0 R/W` with the
    word address being the whole 8-bit memory address, a page write rolls over inside its
    page, a sequential read runs the counter on without a rollover, and a STOP leaves the
    pointer where it is, which is what makes a current-address read work.
    """

    def __init__(self, address: int = 0x50, size: int = 256, page_size: int = 8) -> None:
        if not 0 <= address <= 0x7F:
            raise ValueError(f"I2C address must be 7 bits, got {address:#x}")
        self.address, self.size, self.page_size = address, size, page_size
        self.memory = bytearray(size)
        self.ptr = 0
        self.write_log: List[tuple] = []        # (byte, value), in bus order
        self.read_log: List[int] = []           # addresses read out, in bus order
        self.nacked: List[int] = []             # control bytes we refused

    def roll(self, addr: int) -> int:
        """Next address after a write: +1, but from the last address of a page straight
        back to the first address of the same page."""
        if addr % self.page_size == self.page_size - 1:
            return addr - (self.page_size - 1)
        return (addr + 1) % self.size

    def act(self, byte: int) -> bool:
        """Apply one received byte and return the ACK to drive for it.

        This is the whole 24C02 protocol state machine. `state` is what the next byte
        means: `addr` after a START or a repeated START, `word` after our own write
        address, `write` after the word address, `read` after our read address.
        """
        if self.state == "read":
            # The slave is the one sending. A byte arriving now means the master gave up
            # on the read and started again, so its next byte is an address byte.
            self.state = "addr"
        if self.state == "addr":
            if (byte >> 1) != self.address:
                self.state = "addr"          # ignore everything until a new address
                self.nacked.append(byte)
                return False                 # NAK: nothing ACKs an address that is not ours
            self.state = "read" if byte & 1 else "word"
            if self.state == "read":
                self.read_log.append(self.ptr)
            return True
        if self.state == "word":
            self.ptr = byte % self.size
            self.state = "write"
            return True
        self.memory[self.ptr] = byte         # a data byte: store it, then roll over
        self.write_log.append((self.ptr, byte))
        self.ptr = self.roll(self.ptr)
        return True

    state = "addr"


class I2cTargetDriver:
    """Load fw/i2c_target/program.pio into one engine and emulate the EEPROM behind it.

    The program runs for ever once started; it is entered by the bus, not by the host.
    What the host does is `serve()`: step the bus, empty the RX FIFO (which is where the
    bytes the master sent and the end-of-bus marker arrive) and keep the TX FIFO stocked
    with the two words per byte the program pulls. There is no "do this transaction"
    entry point on this side, and deliberately so: the transactions are the master's.
    """

    def __init__(
        self,
        bus,
        device,
        program: Program,
        clkdiv: int = 14,
        poll: int = 2,
        engine: int = 0,
        sda_pin: int = DEFAULT_SDA_PIN,
        scl_pin: int = DEFAULT_SCL_PIN,
        address: int = 0x50,
        size: int = 256,
        page_size: int = 8,
    ) -> None:
        if not 0 <= sda_pin <= 7 or not 0 <= scl_pin <= 7:
            raise ValueError(
                f"open drain needs two uio pins (docs/mvp-isa.md `### Open drain`), got "
                f"SDA={sda_pin}, SCL={scl_pin}"
            )
        if not 1 <= clkdiv <= 0xFFFF:
            raise ValueError(f"CLKDIV integer part {clkdiv} is out of range")
        if len(program.instrs) > 32:
            raise ValueError(f"the program needs {len(program.instrs)} words, the IMEM holds 32")
        check_program(program, sda_pin, scl_pin)
        self.bus, self.dev = bus, device
        self.prog = set_poll(program, poll)
        self.engine, self.sda_pin, self.scl_pin, self.clkdiv = engine, sda_pin, scl_pin, clkdiv
        self.poll = poll
        self.eeprom = EepromImage(address, size, page_size)
        self.peak_tx = self.peak_rx = 0
        self.bus_free = 0                     # end-of-bus markers seen
        self.bytes_in = 0                     # bytes the master sent us
        self.bytes_out = 0                    # bytes we sent the master
        self.stalled = 0                      # core cycles spent parked in `pull block`
        self._pending: deque = deque()        # words waiting for the TX FIFO, in order
        self._waiting = False                 # a DATA word is out, its byte has not come

    # ---- the register words the program needs --------------------------------------
    @property
    def clkdiv_word(self) -> int:
        return (self.clkdiv & 0xFFFF) << 16

    @property
    def execctrl_word(self) -> int:
        """JMP_PIN = SCL, because `jmp pin` in the program's poll asks whether the master
        is still clocking; the wrap window, so that running off the end parks in `idle`;
        no SIDE_EN and no SIDE_PINDIR, because the program has no side-set at all."""
        return ((self.scl_pin & 0x1F) << 24) | ((self.prog.wrap & 0x1F) << 12) \
            | ((self.prog.wrap_target & 0x1F) << 7)

    @property
    def shiftctrl_word(self) -> int:
        """PULL_THRESH 8 is the program's bit-time counter (`jmp !osre`: one bit is taken
        in `bnd` and seven in `data`), PUSH_THRESH 8 gives one RX word per byte at the
        eighth sample -- a whole bit time before the ACK clock, which is what leaves the
        host time to answer with the ACK word -- AUTOPULL off because the program pulls
        explicitly, AUTOPUSH on, and both shift directions 0 so that a byte is MSB first
        out of the OSR and lands in bits 7:0 of the ISR."""
        return (8 << 25) | (8 << 20) | (0 << 19) | (0 << 18) | (0 << 17) | (1 << 16)

    @property
    def pinctrl_word(self) -> int:
        """Every window sits on the SDA pin alone: SCL is only ever read, never written,
        because a target that drives SCL shortens the master's high period."""
        return (1 << 26) | (1 << 20) | (self.sda_pin << 15) | (self.sda_pin << 5) \
            | self.sda_pin

    @property
    def sm(self) -> int:
        return A_SM + 6 * self.engine

    # ---- register access -----------------------------------------------------------
    def step(self) -> None:
        """One core cycle: the ChipDevice performs at most one queued host access."""
        self.bus.step()

    def run(self, ticks: int) -> None:
        for _ in range(ticks):
            self.step()

    def reg_write(self, addr: int, data: int) -> None:
        self.dev.write(addr, data & 0xFFFF_FFFF)

    def reg_read(self, addr: int) -> int:
        future = self.dev.read(addr)
        ticks = 0
        while not future.done():
            self.step()
            ticks += 1
            if ticks > PHASE_LIMIT:
                raise I2cTargetError(f"read of word address {addr:#04x} never came back")
        return future.result()

    def wait_until(self, what: Callable[[], bool], limit: int = PHASE_LIMIT) -> None:
        ticks = 0
        while not what():
            self.step()
            ticks += 1
            if ticks > limit:
                raise I2cTargetError(f"timed out after {ticks} core cycles")

    # ---- engine setup --------------------------------------------------------------
    def load(self) -> None:
        """The host sequence of docs/mvp-regmap.md `### Typical host sequences`.

        Unlike a controller driver, this one does not jump into the program: the program's
        reset value is `idle`, which waits for a START, so the bus is left alone and the
        first transaction is entirely the master's business.
        """
        self.reg_write(A_CTRL, 0)                                        # stop the engine
        for k, word in enumerate(self.prog.instrs):                      # IMEM burst
            self.reg_write(A_IMEM[self.engine] + k, word)
        self.reg_write(self.sm + SM_CLKDIV, self.clkdiv_word)
        self.reg_write(self.sm + SM_EXECCTRL, self.execctrl_word)
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word)
        self.reg_write(self.sm + SM_PINCTRL, self.pinctrl_word)
        # Step 4 of the same sequence: the initial pin state, then Y = the marker value.
        # Both are forced through SMx_INSTR, which executes whatever the divider says
        # (docs/mvp-isa.md `### EXEC`) and does not advance the PC.
        self.force(set_pindirs_word(0))
        self.drain_tx()          # nothing of a previous run may be left in the FIFO
        # Y = the marker value, which `set y` cannot carry: push the word, pull it, and
        # move it into Y with two forced instructions. Forced cycles ignore SM_ENABLE and
        # the divider (docs/mvp-isa.md `### EXEC`), so this happens before the engine runs.
        self.reg_write(A_TXF0, STOP_MARKER)
        self.force(PULL_BLOCK)
        self.force(MOV_Y_OSR)
        self.reg_write(A_CTRL, (1 << (8 + self.engine)) | (1 << (4 + self.engine))
                       | (1 << self.engine))
        self.clear_fdebug()

    @property
    def _ctrl_start(self) -> int:
        """CTRL with this engine enabled and its divider restarted."""
        e = self.engine
        return (1 << (8 + e)) | (1 << (4 + e)) | (1 << e)

    def force(self, word: int) -> None:
        self.reg_write(self.sm + SM_INSTR, word)

    @property
    def pc(self) -> int:
        return self.reg_read(self.sm + SM_ADDR) & 0x1F

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
        return max(self.peak_tx, self.peak_rx)

    # ---- the host side of the wire: two words per byte, and the RX FIFO --------------
    def _queue(self) -> None:
        """Queue the words for the byte the program is about to clock.

        Order is the whole trick, and it is fixed by where the program pulls:

            DATA(b)   pulled at `first` / `bnd`, before the eight data bit times
            ACK(b)    pulled right after them, before the ninth (the ACK clock)

        so a word must never reach the FIFO before the one before it, and the ACK word
        of a byte we *receive* cannot be produced until that byte has come back. Three
        rules keep that from going wrong:

        * a receive byte contributes its DATA word alone: nothing else is queued until
          the byte lands, so the pull for its ACK finds the FIFO empty and waits rather
          than picking up a word meant for another byte;
        * a byte we *send* contributes DATA and ACK together, because its answer is
          known before it starts (we release the ACK clock), so the read path never
          stalls. A byte the slave sends tells the master nothing back, which is why the
          host counts them itself;
        * a marker word empties the queue, because the bytes the program was going to
          clock belong to a transaction that is over.
        """
        if self._waiting:
            return                      # a DATA word is out there, waiting for its byte
        if self.eeprom.state == "read":
            byte = self.eeprom.memory[self.eeprom.ptr]
            self.eeprom.read_log.append(self.eeprom.ptr)
            self.eeprom.ptr = (self.eeprom.ptr + 1) % self.eeprom.size
            self.bytes_out += 1
            self._pending.append(data_word(byte))
            # The ACK clock of a byte we send belongs to the master: release it, so the
            # master can ACK every byte but the last and NAK that one to end the read.
            self._pending.append(ack_word(False))
            return
        self._pending.append(data_word(None))
        self._waiting = True

    def _fill(self) -> None:
        """Move queued words into the TX FIFO. One FLEVEL read, then the writes."""
        self._queue()
        level = self._tx_level()
        while self._pending and level < regmap.fifo_full_level() - 1:
            self.reg_write(A_TXF0, self._pending.popleft())
            level += 1

    def _drain(self) -> None:
        """Act on everything the program has pushed into the RX FIFO."""
        while self._rx_level():
            word = self.reg_read(A_RXF0)
            if is_stop_marker(word):
                self.bus_free += 1
                self.eeprom.state = "addr"
                self._pending.clear()
                self._waiting = False
                self.drain_tx()
                continue
            if self.eeprom.state == "read":
                # A byte we sent: the program samples the bits we drove and autopushes
                # them like any others, so the RX FIFO carries our own data back. The ACK
                # word for it is already queued (see `_queue`), so there is nothing to
                # decide here -- and nothing to count either, or `bytes_out` would count
                # every byte twice.
                continue
            self.bytes_in += 1
            self._waiting = False
            self._pending.append(ack_word(self.eeprom.act(rx_byte(word))))  # the answer
            self._queue()                                    # and the byte after it

    def drain_tx(self) -> None:
        """Throw away whatever the program has not pulled yet in the TX FIFO.

        There is no flush in the MVP: `FJOIN_*` read 0 and the FIFOs are emptied only by
        reset or by draining (docs/mvp-isa.md `### Enable, restart`), and the register map
        names the way: forced `pull noblock` (`0x8080`), once per word. The OSR is left
        holding X, which costs nothing because the program pulls its words before it
        shifts anything out of them.
        """
        for _ in range(regmap.fifo_depth()):
            self.force(PULL_NOBLOCK)

    def serve(self, ticks: int) -> None:
        """Step the bus for `ticks` core cycles, answering the program as it goes.

        One pass costs the bus at most one FLEVEL read and one FIFO access, so the
        host's answer to a byte lands within a handful of core cycles of the byte arriving
        -- which is what `fw/i2c_target/measure.py` reports as the margin.
        """
        for _ in range(ticks):
            self._drain()
            self._fill()
            self.step()

    def serve_until(self, what: Callable[[], bool], limit: int = PHASE_LIMIT) -> int:
        """`serve` until `what()` holds. Returns the core cycles it took."""
        ticks = 0
        while not what():
            self.serve(1)
            ticks += 1
            if ticks > limit:
                raise I2cTargetError(f"timed out after {ticks} core cycles")
        return ticks

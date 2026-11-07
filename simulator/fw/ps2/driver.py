"""Host side of the MVP PS/2 firmware: the register writes that run fw/ps2/program.pio.

Written against docs/mvp-regmap.md, not against the ISS, so the same driver drives the RTL
once it exists. Host accesses go through the queue of `iss.pio_iss.ChipDevice` (one access
per core cycle, in order) and the driver is what steps the `verif.vip.core.Bus` while it
waits for a read to come back.

The chip is the **controller** (the host, the PC side): it generates the clock for the
bytes it sends and samples the clock the device generates for the bytes it receives. There
is no IRQ in the MVP and `SMx_ADDR` is read-only, so every transaction is started by
forcing a jump through `SMx_INSTR`:

    push tx_word(byte) on TXF0     the frame, LSB first, inverted for the direction register
    jmp send                       12 clocks out: 11 frame bits, then the ack clock
                                   one RX word comes back holding the device's ack bit
    set y, n                       n more frames to take after the next one
    jmp recv                       one RX word per byte, with its start/parity/stop bits

`send` waits for the acknowledgement, because "the command isn't completed until you've
received an ACK" (OSDev, PS/2 keyboard commands) and `0xFE` means send it again; that rule
is in `send_command` below. Everything the host can see of the program is `SMx_ADDR`, the
FIFO levels and `FDEBUG`, which is why the program parks in `idle` between transactions and
pushes the ack bit into the RX FIFO rather than keeping it in the ISR.

The frame and the parity bit are computed here, from the saved sources
(research/ref/protocols/ps2/README.md), and *not* imported from `verif.vip.ps2`: the
reference model is what judges this firmware, so it must not be the firmware's source.
"""
from __future__ import annotations

from fw import regmap

from dataclasses import dataclass
from math import ceil
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

# Pin assignment of the program. fw/ps2/program.pio carries the same two numbers in its
# `.define` lines; `check_program` makes sure the two files still agree.
DEFAULT_CLK_PIN, DEFAULT_DATA_PIN = 0, 1


def __getattr__(name):  # FIFO_DEPTH is read at use time: the active map is fixed at target install
    if name == "FIFO_DEPTH":
        return regmap.fifo_depth()
    raise AttributeError(name)

# ---- the protocol's numbers (research/ref/protocols/ps2/README.md, "Every number") ---
FRAME_BITS = 11                    # start, eight data, odd parity, stop
HOST_FRAME_BITS = 12               # the same, plus the device's acknowledgement clock
ACK = 0xFA                         # the device: command acknowledged
RESEND = 0xFE                      # the device: send the last byte again
CLK_MIN_HALF_NS = 30_000.0         # T3/T7 "Duration of CLK inactive 30/50 us"
CLK_MAX_HALF_NS = 50_000.0         # T4/T8 "Duration of CLK active 30/50 us"
RTS_MIN_NS = 100_000.0             # "Bring the Clock line low for at least 100 microseconds"
BUS_FREE_NS = 50_000.0             # "the keyboard waits for at least 50us" after its stop bit
DATA_SETUP_NS = 5_000.0            # T1's 5 us floor, applied to the edge that samples DATA

# The RX word of one received frame. Eleven `in pins, 1` with IN_SHIFTDIR 0 shift each new
# sample in at bit 0 and push the older ones up, so the frame lands in bits 10:0 with the
# stop bit lowest and the start bit highest: bits 9:2 are d0..d7, but *back to front*, because
# the first bit sampled is the one at the top of the field. That is the price of a protocol
# that sends LSB first into a shift register, and `rx_byte` undoes it.
RX_FRAME_MASK = 0x7FF
RX_BYTE_SHIFT = 2                   # d7 at bit 2, d0 at bit 9: bit-reversed, see rx_byte
RX_PARITY_SHIFT, RX_STOP_SHIFT, RX_START_SHIFT = 1, 0, 10
# The RX word of the acknowledgement: one `in pins, 1`, so the bit the device drove lands in
# bit 0, and it is an ACK when that bit is 0.
RX_ACK_SHIFT = 0
# The TX word of one byte to send: `out pindirs, 1` with OUT_SHIFTDIR 1 walks the OSR from
# bit 0 up, so bits 10:0 are the eleven frame bits, start bit first, LSB first.
TX_FRAME_MASK = 0x7FF
ACK_BIT = 0                        # an acknowledgement is DATA low

TX_LIMIT = 400_000                 # core cycles one transaction may take before we give up


def parity_bit(value: int) -> int:
    """The odd parity bit that follows the eight data bits: 1 when they hold an even
    number of ones.

    "The parity bit is either 1 or 0, and the 8 data bits, plus the parity bit, always have
    an odd number of 1's" (IBM PC/AT Technical Reference, Auxiliary Device; the Gdansk page
    spells it the same way: "if number of ones is even, then parity bit = 1").
    """
    return 1 - (int(value) & 0xFF).bit_count() % 2


def frame_bits(value: int) -> int:
    """The eleven bits of one frame as the OSR holds them, LSB first, start bit first.

    The same layout `verif.vip.ps2.ps2_frame_bits` uses, as one integer: bit 0 is the start
    bit (always 0), bits 1..8 are the data bits LSB first, bit 9 is the parity bit and bit 10
    is the stop bit (always 1).
    """
    word = int(value) & 0xFF
    return (word << 1) | (parity_bit(word) << 9) | (1 << 10)


def tx_word(byte: int) -> int:
    """The TX word for one byte to send, as `out pindirs, 1` writes it to DATA.

    `out pindirs, 1` walks the OSR from bit 0 up (OUT_SHIFTDIR is 1, the reset value) and
    writes one bit per clock, MSB-first within the bit: each bit is a *direction*, and on an
    open-drain line 1 pulls the line low while 0 releases it (docs/mvp-isa.md `### Open
    drain`). A frame bit of 1 therefore has to be written as a 0 and the other way round,
    which is the inversion docs/mvp-isa.md `### Open drain` asks of the host.

    Bits 11 and up are zero, so the shifts past the stop bit release DATA: that is what the
    device's acknowledgement clock needs, since the device pulls DATA low for it.
    """
    if not 0 <= byte <= 0xFF:
        raise ValueError(f"byte {byte!r} does not fit in 8 bits")
    return (~frame_bits(byte)) & TX_FRAME_MASK


def bitrev8(value: int) -> int:
    """The eight bits of `value` the other way round."""
    return int(f"{value & 0xFF:08b}"[::-1], 2)


def rx_byte(word: int) -> int:
    """The data byte of a received frame's RX word.

    Bits 9:2 of the word hold d0..d7 with d7 lowest, because IN_SHIFTDIR 0 shifts each
    sample in at bit 0 and pushes the older ones up, so the first bit sampled ends up at the
    top of the field. Reverse those eight bits and d0 is the least significant one again,
    which is what a byte is.

    The alternative is in hardware: `mov isr, isr r` reverses the whole ISR, which moves the
    frame to bits 31:21 and leaves the byte in order in bits 29:22 (d0 lowest). It costs two
    more instructions -- the MOV and a `push noblock`, because AUTOPUSH would empty the ISR
    before the reversal -- and it puts the frame where the driver has to look for it. Two
    IMEM words and an odd layout for the same eight bits is a bad trade at 32 words of IMEM,
    so the reversal happens here instead.
    """
    return bitrev8((word >> RX_BYTE_SHIFT) & 0xFF)


def rx_parity(word: int) -> int:
    return (word >> RX_PARITY_SHIFT) & 1


def rx_stop(word: int) -> int:
    return (word >> RX_STOP_SHIFT) & 1


def rx_start(word: int) -> int:
    return (word >> RX_START_SHIFT) & 1


def rx_ack(word: int) -> bool:
    """True when the acknowledgement clock found DATA low, which is what an ACK is."""
    return ((word >> RX_ACK_SHIFT) & 1) == ACK_BIT


def ns_to_ticks(ns_per_tick: float, ns: float) -> int:
    """Nanoseconds to whole ticks, rounded up. Same rounding as `verif.vip.core.Bus`."""
    return int(ceil(max(0.0, float(ns)) / ns_per_tick))


def jmp_word(address: int) -> int:
    """`jmp <address>` with no condition, no side-set and no delay: JMP is opcode 000 and
    bits 12:0 are then all zero, so the word is the address.

    This is the word the host forces to start a transaction. Its side-set field is zero,
    which is what an idle PS/2 bus wants: the clock stays released and DATA is untouched, so
    a forced jump never puts an edge on the wire (docs/mvp-isa.md `### EXEC`).
    """
    return address & 0x1F


def set_y_word(value: int) -> int:
    """`set y, value`: opcode 111, destination Y (010), data in bits 4:0.

    Y is the frame counter. The program loads 4 itself before the request-to-send, and the
    host forces this word to say how many *more* frames `recv` should take after the first.
    """
    if not 0 <= value <= 31:
        raise ValueError(f"set y, {value}: the data field is 5 bits")
    return (7 << 13) | (2 << 5) | value


class Ps2Error(RuntimeError):
    """The engine, the wire or the device did something the driver cannot make sense of."""


# ---- what the program spends per clock, read back out of the assembled words ------------
@dataclass(frozen=True)
class Ps2Timing:
    """Every duration `program.pio` spells out, in core cycles.

    Read out of the assembled instructions rather than repeated here, so a change to a
    `[DELAY]` in the .pio moves the numbers with it (`timing_of`).
    """

    words: int                 # IMEM words the program occupies
    clk_low: int               # one bit's low phase: the OUT's delay plus the loop's JMP
    clk_high: int              # the high phase: the NOP that releases the clock
    ack_low: int               # the low phase before the acknowledgement clock
    rts: int                   # request-to-send: from the clock going low to the first rise
    rx_bits: int               # `in pins, 1` per received bit, which is the PUSH_THRESH

    def half_periods(self, clkdiv: int) -> tuple:
        """(low, high) in core cycles at this divider."""
        return self.clk_low * clkdiv, self.clk_high * clkdiv

    def bit_cycles(self, clkdiv: int) -> int:
        """One PS/2 data bit time in core cycles: a low phase plus a high phase."""
        low, high = self.half_periods(clkdiv)
        return low + high

    def ack_cycles(self, clkdiv: int) -> int:
        """The rise-to-rise gap of the acknowledgement clock, in core cycles.

        One SM cycle shorter than a data bit, because the low phase before the twelfth clock
        is the loop's JMP and the sampling `in` rather than the OUT and the JMP. It is
        therefore the shortest gap `Ps2Checker` can measure on a host frame.
        """
        return self.ack_low * clkdiv + self.clk_high * clkdiv

    def line_rate_hz(self, clkdiv: int, tick_ns: float = 20.0) -> float:
        return 1e9 / (tick_ns * self.bit_cycles(clkdiv))


def timing_of(program: Program) -> Ps2Timing:
    """The clock phases, the request-to-send and the bit count of `program.pio`.

    With `.side_set 1` the delay is the low four bits of the delay/side-set field and the
    side-set value is bit 12, so a delay is `word & 0xF` and one instruction lasts
    `1 + delay` SM cycles (docs/mvp-isa.md `### Cycle accounting`). The layout it reads is
    the one in the .pio: `send` pulls, loads Y and counts the request-to-send out; `txloop`
    is OUT / NOP / JMP, so a bit's low phase is the OUT plus the JMP and its high phase is
    the NOP; the two instructions after the loop are the low phase before the
    acknowledgement clock and the acknowledgement clock itself.
    """
    if program.sideset_bits != 1 or program.sideset_opt or not program.sideset_pindirs:
        raise Ps2Error(
            f"program.pio must be `.side_set 1 pindirs` with no `opt` (the one side-set bit "
            f"is the clock's direction and every instruction writes it), not "
            f"{program.sideset_bits} bit(s), opt={program.sideset_opt}, "
            f"pindirs={program.sideset_pindirs}"
        )
    for name in ("send", "rts_wait", "txloop", "recv", "rxloop", "idle"):
        if name not in program.labels:
            raise Ps2Error(f"program.pio has no `{name}` label for the host to enter")
    words = program.instrs

    def delay(addr: int) -> int:
        # The delay/side-set field is bits 12:8; with `.side_set 1` its top bit (bit 12) is
        # the side-set value and the delay is the low four bits of it, i.e. bits 11:8.
        return (words[addr] >> 8) & 0x0F

    send = program.labels["send"]
    loop = program.labels["txloop"]
    rts_wait = program.labels["rts_wait"]
    recv = program.labels["recv"]
    rxloop = program.labels["rxloop"]
    # Y is loaded by the program itself before the request-to-send, so the number of loop
    # iterations is the data field of the `set y, n` at `send` + 1.
    rts_count = words[send + 1] & 0x1F
    # `jmp y-- rts_wait` runs rts_count + 1 times: the last one falls through.
    low = (1 + delay(loop)) + (1 + delay(loop + 2))
    high = 1 + delay(loop + 1)
    # The low phase before the acknowledgement clock: the loop's JMP pulls the clock low and
    # the `in pins, 1` releases it, so the phase is those two instructions and not the OUT
    # that makes a data bit's low phase one SM cycle longer. That is why the acknowledgement
    # clock is the shortest rise-to-rise gap on the wire and what `Ps2Checker.shortest_period`
    # reports.
    ack_low = (1 + delay(loop + 2)) + (1 + delay(loop + 3))
    rts = ((1 + delay(send))                      # pull block
           + (1 + delay(send + 1))                # set y, n
           + (rts_count + 1) * (1 + delay(rts_wait))
           + (1 + delay(send + 2))                # set x, n
           + (1 + delay(send + 3))                # jmp txloop
           + (1 + delay(loop)))                   # out pindirs, 1
    # Every `in pins, 1` of rxloop samples one bit. `jmp x-- rxloop` loops while X is
    # non-zero and falls through when it is zero, so an X of n samples n + 1 bits.
    rx_samples = (words[recv + 1] & 0x1F) + 1
    if rx_samples != FRAME_BITS:
        raise Ps2Error(
            f"the receive path samples {rx_samples} bits, a PS/2 frame is {FRAME_BITS}"
        )
    # The receive loop has to be wait(fall) / in / wait(rise) / jmp x--, or it would sample
    # one bit per SM cycle while the clock sits low: the device holds it low for a whole
    # bit time, so only the rising edge separates two bits. What follows the loop is the
    # end of the frame, whose last sample AUTOPUSH has already pushed.
    for addr, opcode, what in ((rxloop, 0b001, "`wait 0 gpio`"),
                               (rxloop + 1, 0b010, "`in pins, 1`"),
                               (rxloop + 2, 0b001, "`wait 1 gpio`")):
        if words[addr] >> 13 != opcode:
            raise Ps2Error(f"word {addr} of the receive loop is not {what}")
    if words[rxloop + 3] >> 13 != 0b000 or (words[rxloop + 3] >> 5) & 0x7 != 0b010:
        raise Ps2Error("the receive loop does not end in `jmp x--`")
    return Ps2Timing(words=len(words), clk_low=low, clk_high=high, ack_low=ack_low, rts=rts,
                     rx_bits=rx_samples)


def legal_clkdiv(timing: Ps2Timing, tick_ns: float = 20.0) -> range:
    """Every CLKDIV whose clock phases stay inside T3/T4 (and T7/T8) and inside tRTS.

    Both clock phases have to be 30..50 us, and the divider is the host's only knob, so this
    is the whole range of speeds one assembled program can run at. An integer divider is the
    only thing this driver writes (FRAC would make the phases alternate, and every limit
    here is a floor), so the answer is a contiguous integer range.
    """
    lo = max(1,
             # The high phase of every clock and the low phase before the acknowledgement
             # clock are the two short phases; both have to clear the 30 us floor.
             ceil(ns_to_ticks(tick_ns, CLK_MIN_HALF_NS) / min(timing.clk_high, timing.ack_low)),
             ceil(ns_to_ticks(tick_ns, RTS_MIN_NS) / timing.rts),
             # DATA moves in the first SM cycle of the low phase, so it has one SM cycle
             # less than that phase to be stable before the rising edge that samples it.
             ceil(ns_to_ticks(tick_ns, DATA_SETUP_NS) / max(1, timing.clk_low - 1)))
    hi = ns_to_ticks(tick_ns, CLK_MAX_HALF_NS) // timing.clk_low
    # The other direction -- DATA stable within 1 us *after* that rise -- is clear by
    # construction, because nothing writes DATA while the clock is high.
    return range(lo, hi + 1) if lo <= hi else range(0)


def retarget(program: Program, clk_pin: int) -> Program:
    """The same program with its `wait ... gpio` aimed at another clock pin.

    Exactly the instructions that watch the clock line carry a pin number, and they take an
    absolute index rather than a window offset (docs/mvp-isa.md `### Field values`, WAIT with
    the GPIO source), so a program assembled for one pin pair runs on any other pair by
    patching those fields, which is what the RP2040 SDK does when it encodes a wait at run
    time. Every other pin in the program is a PINCTRL field, which the host writes anyway.
    """
    if not 0 <= clk_pin <= 31:
        raise ValueError(f"CLK pin {clk_pin} does not fit the 5-bit WAIT index")
    words, patched = [], 0
    for word in program.instrs:
        is_wait_gpio = (word >> 13) == 1 and ((word >> 5) & 3) == 0
        if is_wait_gpio:
            word = (word & ~0x1F) | clk_pin
            patched += 1
        words.append(word)
    if not patched:
        raise Ps2Error(
            "the program has no `wait ... gpio` to aim at CLK, so it cannot follow the "
            "clock the device generates"
        )
    return Program(
        name=program.name, instrs=words, origin=program.origin,
        wrap_target=program.wrap_target, wrap=program.wrap,
        sideset_bits=program.sideset_bits, sideset_opt=program.sideset_opt,
        sideset_pindirs=program.sideset_pindirs, labels=program.labels,
    )


def check_program(program: Program, clk_pin: int) -> None:
    """Fail loudly if the driver and the .pio file disagree about the pins.

    The program names its pins in `.define` lines, which the assembler does not hand back, so
    the instructions that pin them down are compared instead: it has to watch CLK by absolute
    index (both edges), shift DATA's direction out of the OSR one bit at a time, sample DATA
    through the IN window and wait for the device's start bit.
    """
    waits = {}
    for word in program.instrs:
        if (word >> 13) == 1 and ((word >> 5) & 3) == 0 and (word & 0x1F) == clk_pin:
            waits[(word >> 7) & 1] = waits.get((word >> 7) & 1, 0) + 1
    for polarity, what in ((0, "falling"), (1, "rising")):
        if not waits.get(polarity):
            raise Ps2Error(
                f"program.pio has no `wait {polarity} gpio {clk_pin}`, so it cannot sample "
                f"DATA on the {what} edge of the clock the device generates"
            )
    if not any((word >> 13) == 3 and ((word >> 5) & 7) == 4 and (word & 0x1F) == 1
               for word in program.instrs):
        raise Ps2Error("program.pio never puts an OSR bit on DATA with `out pindirs, 1`")
    if not any((word >> 13) == 2 and ((word >> 5) & 7) == 0 and (word & 0x1F) == 1
               for word in program.instrs):
        raise Ps2Error("program.pio never samples DATA with `in pins, 1`")
    if not any((word >> 13) == 1 and ((word >> 5) & 3) == 1 for word in program.instrs):
        raise Ps2Error(
            "program.pio never waits on the IN window, so it cannot see the device's "
            "start bit"
        )


#: The divider the driver and the tests run at: 100 core cycles per SM cycle, i.e. 2 us at
#: 50 MHz, which puts both clock phases at 32 and 34 us (T3/T4 want 30..50).
DEFAULT_CLK_DIV = 100


class Ps2Driver:
    """Load fw/ps2/program.pio into one engine and drive PS/2 transactions with it."""

    def __init__(
        self,
        bus,
        device,
        program: Program,
        clkdiv: int = DEFAULT_CLK_DIV,
        engine: int = 0,
        clk_pin: int = DEFAULT_CLK_PIN,
        data_pin: int = DEFAULT_DATA_PIN,
        retries: int = 3,
        tick_ns: float = 20.0,
        bus_free_ticks: Optional[int] = None,
    ) -> None:
        if data_pin > 7 or clk_pin > 7:
            raise ValueError(
                f"both PS/2 lines are open drain, and open drain is a direction, so both "
                f"have to be uio pins 0..7 (docs/mvp-isa.md `### Open drain`); "
                f"CLK={clk_pin}, DATA={data_pin}"
            )
        if clk_pin == data_pin:
            raise ValueError(f"CLK and DATA are two wires, not one: both are pin {clk_pin}")
        if len(program.instrs) > 32:
            raise ValueError(
                f"the program needs {len(program.instrs)} words, the IMEM holds 32"
            )
        if not 0 <= engine <= 1:
            raise ValueError(f"engine {engine}: the MVP has two, 0 and 1")
        program = retarget(program, clk_pin)
        check_program(program, clk_pin)
        self.prog = program
        self.timing = timing_of(program)
        self.legal = legal_clkdiv(self.timing, tick_ns)
        if clkdiv not in self.legal:
            raise ValueError(
                f"CLKDIV {clkdiv} is outside the range this program can run at: both clock "
                f"phases have to be {CLK_MIN_HALF_NS / 1e3:.0f}..{CLK_MAX_HALF_NS / 1e3:.0f} us, "
                f"so {self.legal.start}..{self.legal.stop - 1} are legal at {tick_ns:g} ns "
                f"per tick (the request-to-send needs "
                f"{self.timing.rts} SM cycles)"
            )
        self.bus = bus
        self.dev = device
        self.engine = engine
        self.clk_pin = clk_pin
        self.data_pin = data_pin
        self.clkdiv = clkdiv
        self.retries = int(retries)
        self.tick_ns = tick_ns
        # "After sending stop bit, the keyboard waits for at least 50us, until it transmits
        # next data" (Gdansk page): the floor `tBUS-FREE` is checked against, and what the
        # host has to leave on the wire between two of its own frames.
        self.bus_free_ticks = (ns_to_ticks(tick_ns, BUS_FREE_NS) if bus_free_ticks is None
                               else int(bus_free_ticks))
        self.peak_tx = self.peak_rx = 0
        self.sent: List[int] = []
        self.resends = 0
        self._send = program.labels["send"]
        self._recv = program.labels["recv"]
        self._idle = program.labels["idle"]

    # ---- the register words the program needs --------------------------------------
    @property
    def clkdiv_word(self) -> int:
        return (self.clkdiv & 0xFFFF) << 16

    @property
    def execctrl_word(self) -> int:
        """SIDE_PINDIR, because the side-set drives a direction and open drain is a
        direction; no SIDE_EN, because the program side-sets every instruction (a `side 0`
        then means "the clock is released", which is what the idle state needs); and the
        wrap window, so that running off either end parks in `idle`."""
        return (1 << 29) | ((self.prog.wrap & 0x1F) << 12) | \
            ((self.prog.wrap_target & 0x1F) << 7)

    @property
    def shiftctrl_word(self) -> int:
        """AUTOPUSH at eleven bits empties the ISR into the RX FIFO on the last sample of a
        frame, so one RX word is one received frame and the loop needs no push. AUTOPULL is
        off because the program pulls the frame word it is about to send itself.

        OUT_SHIFTDIR 1 walks the OSR from bit 0 up, because the frame bits leave the OSR LSB
        first, and IN_SHIFTDIR 0 shifts each sample in at bit 0, which puts the frame in
        bits 10:0 with the stop bit lowest. PS/2 sends its data bits LSB first, so either
        direction leaves the byte inside its field the wrong way round; `rx_byte` reverses
        the eight bits, and the comment there says what the hardware alternative costs."""
        return (FRAME_BITS << 20) | (1 << 19) | (0 << 18) | (0 << 17) | (1 << 16)

    @property
    def pinctrl_word(self) -> int:
        """One side-set bit for CLK's direction, one OUT bit for DATA's direction, and the
        IN window on DATA. The SET window is unused: the program never writes a pin level or
        a second direction with `set`, it only ever toggles DATA's direction out of the OSR
        and lets the side-set drive CLK's."""
        return (
            (1 << 29)                     # SIDESET_COUNT 1: CLK's direction
            | (1 << 20)                   # OUT_COUNT 1: DATA
            | (self.data_pin << 15)       # IN_BASE: `in pins, 1` and `wait 0 pin 0` read DATA
            | (self.clk_pin << 10)        # SIDESET_BASE
            | self.data_pin               # OUT_BASE
        )

    @property
    def sm(self) -> int:
        """Word address of this engine's `SMe_CLKDIV`."""
        return A_SM + 6 * self.engine

    @property
    def line_rate_hz(self) -> float:
        """The PS/2 bit rate this configuration puts on the bus."""
        return self.timing.line_rate_hz(self.clkdiv, self.tick_ns)

    @property
    def clk_halves_ns(self) -> tuple:
        """(low, high) in nanoseconds: the two halves of a clock period, both of which have
        to be 30..50 us."""
        return tuple(phase * self.tick_ns for phase in self.timing.half_periods(self.clkdiv))

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
            if ticks > TX_LIMIT:
                raise Ps2Error(f"read of word address {addr:#04x} never came back")
        return future.result()

    def wait_until(self, what: Callable[[], bool], limit: int = TX_LIMIT) -> None:
        ticks = 0
        while not what():
            self.step()
            ticks += 1
            if ticks > limit:
                raise Ps2Error(f"timed out after {ticks} core cycles")

    # ---- engine setup --------------------------------------------------------------
    def load(self) -> None:
        """The host sequence of docs/mvp-regmap.md `### Typical host sequences`.

        Nothing is forced at the end: at reset every pin is an input and the level registers
        are 0, which on an open-drain pair is exactly "both lines released", so the engine is
        started in the program's own `init` and parks in `idle` with the bus idle. No forced
        instruction means no edge on the wire, and no edge before the first frame is what the
        checker's first `tBUS-FREE` measurement wants.
        """
        self.reg_write(A_CTRL, 0)                                        # stop the engine
        for k, word in enumerate(self.prog.instrs):                      # IMEM burst
            self.reg_write(A_IMEM[self.engine] + k, word)
        self.reg_write(self.sm + SM_CLKDIV, self.clkdiv_word)
        self.reg_write(self.sm + SM_EXECCTRL, self.execctrl_word)
        self.reg_write(self.sm + SM_SHIFTCTRL, self.shiftctrl_word)
        self.reg_write(self.sm + SM_PINCTRL, self.pinctrl_word)
        # Step 6 of the same sequence: enable and restart the divider together, so the first
        # SM cycle is two core cycles after this access (docs/mvp-isa.md `### Clock divider`).
        self.reg_write(A_CTRL, self._ctrl_start)
        self.wait_parked()

    @property
    def _ctrl_start(self) -> int:
        e = self.engine
        return (1 << (8 + e)) | (1 << (4 + e)) | (1 << e)

    def force(self, word: int) -> None:
        """Push one instruction into the engine's EXEC latch.

        It executes in the next core cycle whatever SM_ENABLE and the divider say, it does
        not advance the PC and its delay is ignored (docs/mvp-isa.md `### EXEC`). Every
        word this driver forces is a `jmp` or a `set y, n`, both of which have a zero
        side-set field, so a forced instruction never touches the clock: the bus stays
        exactly as the program left it.
        """
        self.reg_write(self.sm + SM_INSTR, word & 0xFFFF)

    @property
    def pc(self) -> int:
        """`SMx_ADDR`, the engine's PC. There is no IRQ in the MVP, so this and the FIFO
        levels are all the host can see of the program's progress."""
        return self.reg_read(self.sm + SM_ADDR) & 0x1F

    @property
    def parked(self) -> bool:
        return self.pc == self._idle

    def wait_parked(self) -> None:
        self.wait_until(lambda: self.parked)

    def wait_left_parked(self) -> None:
        """Step until the engine has left `idle`, i.e. a forced jump landed.

        A forced instruction runs in the *next* core cycle, so reading SM0_ADDR right after
        writing SM0_INSTR still shows the old PC: without this the caller would go on
        believing the transaction it just started has finished.
        """
        self.wait_until(lambda: not self.parked)

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

        Both are four deep (docs/mvp-regmap.md `### Engine state`), so anything below four
        means the host kept up with the engine instead of relying on its stalls.
        """
        return max(self.peak_tx, self.peak_rx)

    def _take_rx(self, limit: int = TX_LIMIT) -> int:
        """The oldest RX word, stepping the bus until one is there."""
        self.wait_until(lambda: self._rx_level() > 0, limit)
        return self.reg_read(A_RXF0)

    def wait_bus_free(self) -> None:
        """Leave the bus alone for the device's own 50 us before the next frame.

        `tBUS-FREE` is measured from the rising edge that began the previous frame's last
        clock, and it applies to the host as well as to the device: two host frames back to
        back have to be 50 us apart.
        """
        self.run(self.bus_free_ticks)

    # ---- one transaction -----------------------------------------------------------
    def send(self, byte: int) -> bool:
        """Clock one byte out to the device. True when the device acknowledged it.

        The whole frame is the program: the request-to-send, the start bit, eleven clocks of
        frame and the device's acknowledgement clock. The ack bit comes back as one RX word,
        which is the only thing the host can read of it (there is no IRQ in the MVP), so
        this is where "the command isn't completed until you've received an ACK" is decided.

        No bus-free wait at the end: the device needs its own 50 us before it starts
        answering, and `receive` has to be armed before then, so the wait belongs to the
        caller (`command` does send-then-receive, and a second `send` calls
        `wait_bus_free` first).
        """
        self.wait_parked()
        if self._tx_level() >= regmap.fifo_full_level():
            raise Ps2Error("the TX FIFO is full and nothing is draining it")
        self.reg_write(A_TXF0, tx_word(byte))
        self.force(jmp_word(self._send))
        self.wait_left_parked()             # the forced jump has landed
        self.wait_parked()                  # and the twelve clocks are out
        word = self._take_rx()              # the acknowledgement bit
        self.sent.append(byte)
        return rx_ack(word)

    def receive(self, count: int = 1) -> bytes:
        """Take `count` bytes the device sends, one frame each.

        The host arms the receive path with a forced `set y, n` and a forced `jmp recv`, and
        the engine then waits for each start bit in turn, so a run of bytes off the bus needs
        no further help from the host. Every word is checked for its start bit, its stop bit
        and its parity before it is handed back: a frame that fails is a framing error the
        host driver has to notice, not a byte to pass on.

        This may only be called while the bus is free. The receive path starts with
        `wait 0 pin 0`, so entering it in the middle of a frame would lock onto the middle
        of one; the parity and stop-bit checks are what turn that into an error rather than
        a wrong byte.
        """
        if not 1 <= count <= 32:
            raise Ps2Error(
                f"receive {count} frames: the program counts them in Y, and only "
                f"`set y, n` can load it, so 1..32 frames at a time"
            )
        self.wait_parked()
        self.force(set_y_word(count - 1))
        self.force(jmp_word(self._recv))
        out: List[int] = []
        for index in range(count):
            word = self._take_rx()
            if word & ~RX_FRAME_MASK:
                raise Ps2Error(
                    f"frame {index} from the device came back as {word:#010x}, which has bits "
                    f"above the eleven of a frame: the program's shift direction is not what "
                    f"the driver reads"
                )
            value = rx_byte(word)
            if rx_start(word) != 0 or rx_stop(word) != 1:
                raise Ps2Error(
                    f"frame {index} from the device is not framed: start bit "
                    f"{rx_start(word)}, stop bit {rx_stop(word)} (word {word:#010x})"
                )
            if rx_parity(word) != parity_bit(value):
                raise Ps2Error(
                    f"frame {index} from the device carries the wrong parity bit for "
                    f"0x{value:02X} (word {word:#010x})"
                )
            out.append(value)
        self.wait_parked()
        self.wait_bus_free()
        return bytes(out)

    def command(self, byte: int, response: int = 0, expect: int = ACK) -> bytes:
        """Send one command byte and take the device's answer: `expect` plus `response` more
        bytes.

        That is the whole command protocol of a PS/2 device (OSDev, "PS/2 Keyboard Commands"):
        every command is acknowledged with 0xFA, some answer with more bytes after it, and
        0xFE means "send that byte again", which is retried up to `retries` times. A byte the
        device never acknowledged is re-sent as well, because nothing came back at all.

        `expect` is the acknowledgement byte, and it is a parameter because one command is
        the exception: 0xEE (echo) answers with itself, not with 0xFA.
        """
        limit = self.retries
        for attempt in range(limit + 1):
            if not self.send(byte):
                self.resends += 1             # no acknowledgement: the byte was lost
                continue
            answer = self.receive(response + 1)
            if answer[0] == RESEND:
                self.resends += 1             # "the command isn't completed until an ACK"
                continue
            if answer[0] != expect:
                raise Ps2Error(
                    f"the device answered 0x{byte:02X} with 0x{answer[0]:02X}, not "
                    f"0x{expect:02X}"
                )
            return answer[1:]
        raise Ps2Error(
            f"0x{byte:02X} was not acknowledged after {limit + 1} tries "
            f"({self.resends} re-sends in total)"
        )
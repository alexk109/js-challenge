"""The UART frame the RX firmware can receive: data bits, parity, stop bits.

`fw/uart_rx/program.pio` is 8N1 -- eight `in pins, 1`, one stop-bit check -- and this module
generates the same program for 7 or 9 data bits, a parity bit and two stop bits, so that
`docs/mvp-verification-report.md` `### Known holes in the stimulus` ("UART covers 8N1 only; no
parity, no 7 or 9 data bits, no two stop bits.  Nothing runs below 9 600 Bd on the UART RX path")
is closed on the receive side too.

Three properties make this safe to do by generation:

*   **the generated 8N1 program is `program.pio`, word for word.**  The delays below are the same
    ones the shipped file spells out (`S/2-2`, `S-1`, `S*3/4-1`), so
    `fw/uart_rx/test_uart_rx_framing.py::test_generated_8n1_is_the_shipped_program` assembles both
    and compares.  The existing 8N1 tests keep testing the program that is in the tree.
*   **the timing of the samples is derived, not copied.**  Every sample lands in the middle of its
    own bit time, and the stop bits are sampled a quarter of the way into each of them, because
    that is what leaves the engine back at `hunt` before the next frame can start.  The delay a
    data-bit instruction carries is therefore `SM - 1` for every bit except the last sample before
    the stop check, which carries `SM * 3 / 4 - 1`, and with a parity bit in the frame it is the
    parity instruction that carries the short delay instead.  `sample_delays()` returns the list,
    so a test can check the arithmetic rather than trust it.
*   **the parity bit is checked by the host, and that is a real limit, not a convenience.**  The
    program shifts the parity bit into the ISR like a data bit and pushes the word; the host
    compares it with the parity of the data bits it just received.  The PIO subset cannot do it
    itself: `JMP`'s conditions are X, Y, ISR-non-zero, OSR, GPIO and IRQ only and `MOV`'s ops are
    invert and reverse (`docs/mvp-isa.md` `## Encoding` and `### Instruction semantics`, MOV), so
    there is no way to fold a register's bits and branch on the result.  The wire is still checked
    independently: `verif/vip/uart.py`'s `UartTx` is told the parity mode, and a receiver that
    accepted a wrong parity bit would show up as a `parity_error` on the judge when the tests
    compare the two.

`Framing` itself is the one from `fw/uart_tx/framing.py`: there is one definition of what a frame
is, and both directions use it.  It is not a shared *judge* -- `verif/vip/uart.py` is the judge and
is untouched -- and `test_parity_bit_agrees_with_the_judge` is what keeps two implementations of the
frame convention honest.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from fw.uart_tx.framing import STOP_BITS, Framing
from fw.uart_tx.framing import all_framings as all_tx_framings

DEFAULT = Framing()

#: The RX FIFO word pushed after a byte whose stop bit was low.  The data word leaves every bit
#: below its last data bit clear, so a word with one of them set is that marker and the two can
#: never be confused (`fw/uart_rx/program.pio`, `docs/mvp-regmap.md` `**RXF0, RXF1**`).
FRAMING_ERROR_MARKER = 1

#: The marker the host synthesises for a parity error it found, so both kinds of error are
#: reported the same way in `UartRx.markers`.  It is a different value from the program's framing
#: marker, which is what makes the two distinguishable after the fact.
PARITY_ERROR_MARKER = 2


def all_framings() -> Iterator[Framing]:
    """Every framing the receiver can be built for (the same set the transmitter can send)."""
    return all_tx_framings()


@dataclass(frozen=True)
class WordLayout:
    """Where one received frame's fields sit in the word the engine pushed.

    `SHIFTCTRL.IN_SHIFTDIR = 1` (right) gives `isr' = (ISR >> n) | (d << (32 - n))`
    (`docs/mvp-isa.md` `### Instruction semantics`, IN), so each new bit enters at bit 31 and
    everything already in the ISR moves one place down.  After a frame of `shift_bits` samples the
    **first** bit received is therefore at bit `32 - shift_bits` and the last one at bit 31, and
    because the wire sends the least significant data bit first, the byte comes out right-aligned
    in that field: data bit 0 at bit `32 - shift_bits`, up to data bit `data_bits - 1`.

    For 8N1 that is the shipped layout -- the byte at bits 31:24 with every bit below it clear --
    and for a framing with parity it is the same field one bit wider, with the parity bit, sampled
    last, at bit 31.
    """

    data_bits: int
    has_parity: bool

    @property
    def shift_bits(self) -> int:
        """Bits the program samples per frame: the data bits plus the parity bit, if any."""
        return self.data_bits + (1 if self.has_parity else 0)

    @property
    def data_shift(self) -> int:
        """Bits to shift the word right to bring the data bits down to bit 0.

        This is where data bit 0 sits, i.e. the bottom of the frame's field.
        """
        return 32 - self.shift_bits

    @property
    def data_mask(self) -> int:
        return (1 << self.data_bits) - 1

    @property
    def parity_shift(self) -> int:
        """Bit position of the parity bit in the pushed word: the last bit sampled, so bit 31."""
        return 31

    @property
    def marker_mask(self) -> int:
        """Bits a data word leaves clear, so a word with one of them set is an error marker.

        This is everything below the frame, which for 8N1 is the shipped `RX_WORD_DATA_MASK`.  A
        parity framing does not need a different marker protocol: the parity bit is *inside* the
        field and every bit below the frame is still marker space.
        """
        return (1 << self.data_shift) - 1


def layout(framing: Framing) -> WordLayout:
    return WordLayout(framing.data_bits, framing.has_parity)


def sample_delays(framing: Framing, sm_cycles_per_bit: int) -> list[int]:
    """The delay of every `in pins, 1` of a frame, in the order the program shifts them.

    An instruction runs for `1 + delay` SM cycles, so two samples `SM` SM cycles apart have delays
    of `SM - 1`, and a stop-bit check with no delay comes exactly one bit time after the sample
    before it.  There are two cases, and they are the two ends of the same arithmetic:

    *   **with parity** every sample is `SM - 1`, the parity bit included, so *every* bit of the
        frame is sampled in its own middle and the stop-bit check that follows (delay 0) is one bit
        time after the parity bit, in the middle of the first stop bit.
    *   **without parity** the last data bit carries `3 * SM / 4 - 1` instead, which pulls the
        stop-bit check forward to a *quarter* of the way into the stop bit.  That is what
        `program.pio` does for 8N1 and why: a stop bit is idle high, so any point in it will do,
        and a quarter in leaves the most room to walk back to `hunt` before the next frame starts.

    `stop_gap_bit_times` is the other half of this: how far the first stop-bit check sits after
    the last sample, which the generated program and the tests both read from here.
    """
    sm = int(sm_cycles_per_bit)
    full = sm - 1
    if framing.has_parity:
        return [full] * framing.shift_bits
    return [full] * (framing.shift_bits - 1) + [sm * 3 // 4 - 1]


def stop_gap_bit_times(framing: Framing) -> float:
    """How far the first stop-bit check sits after the last sample, in bit times.

    1.0 with parity (it follows the parity sample one bit time later, mid-bit), 0.75 without it
    (the short delay above).  A second stop check is one bit time after the first either way.
    """
    return 1.0 if framing.has_parity else 0.75


def stop_delays(framing: Framing, sm_cycles_per_bit: int) -> list[int]:
    """The bit-time spacing the stop-bit checks sit at, one entry per stop bit after the first.

    The first stop-bit check follows the last sample immediately (the short delay in
    `sample_delays` already put it a quarter of the way into that stop bit, as `program.pio` does).
    A second check has to be a whole bit time later, and a jump's delay applies whether or not the
    jump is taken (`docs/mvp-isa.md` `### Instruction semantics`, JMP), so the spacing is carried
    by the *first* check's jump rather than by an extra `nop`.
    """
    return [sm_cycles_per_bit - 1] * (framing.stop_bits - 1)


# ---------------------------------------------------------------------------------------------
# The program
# ---------------------------------------------------------------------------------------------

def program_source(framing: Framing = DEFAULT, sm_cycles_per_bit: int = 16) -> str:
    """The PIO source of the receiver for `framing` at `sm_cycles_per_bit` SM cycles per bit.

    The shape, the labels and the comments are `fw/uart_rx/program.pio`'s; what is generated is
    the number of `in pins, 1` (the data bits, plus the parity bit if the framing has one), which
    of them carries the short delay, and the number of stop-bit checks.  For 8N1 this emits the
    shipped source's instructions exactly, which is what
    `test_generated_8n1_is_the_shipped_program` checks by assembling both.
    """
    sm = int(sm_cycles_per_bit)
    delays = sample_delays(framing, sm)
    data = [(f"d{i}:", "        ", "in") for i in range(len(delays))]
    lines: list[str] = [
        ";",
        f"; fw/uart_rx: {framing.name} receiver, generated by fw/uart_rx/framing.py from the shape",
        "; of fw/uart_rx/program.pio.",
        ";",
        "; Started from the pico-examples uart_rx.pio idea, then rewritten for this chip.",
        "; What the spec forced (docs/mvp-isa.md, docs/mvp-regmap.md):",
        ";",
        ";   * No IRQ: there are no IRQ flags, so a framing error cannot raise one.  It is",
        ";     reported in the RX FIFO instead -- the byte goes in first, then a word with the",
        ";     bits below",
        ";     the frame nonzero (FRAMING_ERROR) says \"the byte just before me had a low stop",
        ";     bit\".  A data word always leaves those bits clear, so the two can never be",
        ";     confused.",
        ";   * T_IN = 3 always (no INPUT_SYNC_BYPASS) and T_OUT = 1, so a pin change reaches",
        ";     IN_VEC four core cycles after it is driven: the first instruction after a `wait`",
        ";     that completes is already four cycles late.  Every sample below is placed relative",
        ";     to that.",
        ";   * The delay field is 5 bits, so one bit time cannot be one delay.  The host",
        ";     divides the clock so that one bit time is S SM cycles, and the loop spends S SM",
        ";     cycles per bit; S and the loop are therefore a pair, and driver.py passes S here.",
        ";",
        "; Timing, in SM cycles from the cycle in which `wait 0 pin, 0` completes (cycle 0).  One",
        "; bit time is S SM cycles and the engine's cycle grid is quantised to the bit time, so",
        "; a sample taken at SM cycle m lands (m/S + d) of a bit after the falling edge that",
        "; opened the frame, where d in [0, 1/S) is the phase of that grid:",
        ";",
        ";   SM cycle 0           wait 0 pin, 0      the start bit",
        ";   SM cycle S/2         jmp pin, reject   start-bit validation, 0.5 .. 0.5+1/S into it",
        f";   SM cycle 1.5S + i    in pins, 1        bit {framing.shift_bits} samples, "
        "(i+0.5) .. (i+0.5+1/S) into it",
        f";   SM cycle {framing.shift_bits + 1}.25S       jmp pin, good_stop   stop bit 1, "
        "0.25 .. 0.25+1/S into it",
        f";   SM cycle {framing.shift_bits + 2}.25S       jmp pin, good_stop   stop bit 2, "
        "0.25 .. 0.25+1/S into it",
        ";",
        "; The start bit is validated in its middle, so a low pulse shorter than half a bit --",
        "; a glitch, or noise on an idle line -- is high again by then and is thrown away; that",
        "; is the whole of the glitch rejection, and it is why the validation is not folded into",
        "; the first `in pins`.  Every data bit is likewise sampled in the middle of its own bit",
        "; time, so the transmitter may be up to 5% off nominal before a sample can leave its",
        "; bit, at any speed.",
        "; Each stop bit is sampled a quarter of the way into it, as program.pio does for 8N1,",
        "; and a second one is exactly one bit time after the first.  A jump's delay applies",
        "; whether or not the jump is taken, so that spacing is the first check's delay and needs",
        "; no `nop`.",
        ";",
        "; What the host has to live with: the RX FIFO is four words deep, and a UART line",
        "; keeps running while this program is stalled in a blocking `push`, so bytes that",
        "; arrive after the fifth are lost.  And a frame that ends with a low stop bit leaves the",
        "; line low, so the next start bit has to be preceded by one idle bit for the receiver",
        "; to find it -- exactly as after a break.",
        ";",
    ]
    if framing.has_parity:
        lines += [
            "; The parity bit is shifted in like a data bit and lands below the data in the pushed",
            "; word; the *host* compares it with the parity of the data bits, because the PIO",
            "; subset has no way to fold a register's bits and branch on the result (the module",
            "; docstring of fw/uart_rx/framing.py has the whole argument).",
            ";",
        ]
    lines += [
        "; The RX pin is whichever pin the host puts in IN_BASE, JMP_PIN and SET_BASE; the program",
        "; only ever uses index 0 of those windows.",
        ";",
        ".program uart_rx",
        f".define S {sm}                     ; SM cycles per bit time: CLKDIV = ticks_per_bit / S",
        f".define FRAMING_ERROR {FRAMING_ERROR_MARKER}      ; FIFO word pushed after a byte whose "
        "stop bit was low",
        ".wrap_target idle",
        ".wrap reject",
        "",
        "set     pindirs, 0           ; release the RX pin: it is an input (uio pins only)",
        "",
        "idle:                         ; the line is high: idle, or a break has just ended",
        "        wait     1 pin, 0",
        "",
        "hunt:                         ; a 1 -> 0 transition: the start bit",
        "        wait     0 pin, 0",
        "",
        "; Clear the ISR (a rejected start bit may have left bits in it) while the rest of the",
        "; start bit runs out, so that the validation below is in the middle of it.",
        "        in      null, 32 [S/2-2]",
        "",
        "; Validate the start bit in its middle: a low pulse shorter than half a bit is high",
        "; again by then and is rejected, which is all the glitch rejection there is.",
        "        jmp     pin, reject [S-1]",
        "",
    ]
    spellings = {sm - 1: "S-1", sm * 3 // 4 - 1: "S*3/4-1", sm // 4 - 1: "S/4-1"}
    for i, delay in enumerate(delays):
        if delay not in spellings:
            raise ValueError(
                f"{framing} at S={sm}: a delay of {delay} is neither S-1 ({sm - 1}), "
                f"S*3/4-1 ({sm * 3 // 4 - 1}) nor S/4-1 ({sm // 4 - 1}), and there is no "
                f"`sample_delays` derivation for it")
        parity_bit = framing.has_parity and i == len(delays) - 1
        what = "the parity bit" if parity_bit else f"data bit {i}"
        short = delay != sm - 1
        lines.append(f"d{i}:     in      pins, 1 [{spellings[delay]}]"
                     f"           ; {what}"
                     f"{', with a short delay: see the header' if short else ''}")
    lines += ["", f"; The stop bit{'s' if framing.stop_bits > 1 else ''}.  `jmp pin` is taken",
              "; when the pin is high, so the jump is the good frame and the fall-through is the",
              "; framing",
              "; error.  With two stop bits the first check must not *skip* the second one: it",
              "; jumps to it instead, and the low case goes straight to the error.  A single check",
              "; that jumped to `good_stop` on the first high stop bit would leave the second",
              "; one unexamined, which is what `test_a_low_stop_bit_is_a_framing_error` is there",
              "; to catch."]
    if framing.stop_bits == 1:
        lines.append("        jmp     pin, good_stop               ; the stop bit")
    else:
        lines += [
            "        jmp     pin, second_stop [S-1]        ; stop bit 1, and the delay",
            "                                           ; runs either way",
            "        jmp     framing_error               ; ... or the stop bit was low",
            "second_stop:",
            "        jmp     pin, good_stop               ; stop bit 2, in its own bit time",
        ]
    lines += [
        "",
        "framing_error:                ; a stop bit was low",
        "        push block              ; the byte goes in whatever happened ...",
        "        set     x, FRAMING_ERROR",
        "        mov     isr, x",
        "        push block              ; ... followed by the marker that flags it",
        "        jmp     idle",
        "",
        "good_stop:                    ; the stop bits were high: a good frame",
        "        push block              ; the byte, left justified at bit 31",
        "        wait     1 pin, 0       ; (fall through) idle again ...",
        "        wait     0 pin, 0       ; ... and hunt for the next start bit",
        "        jmp     hunt",
        "",
        "reject:                       ; the low pulse was too short to be a start bit",
        "        jmp     hunt",
        "",
    ]
    return "\n".join(lines)


def program_for(framing: Framing = DEFAULT, sm_cycles_per_bit: int = 16) -> object:
    """Assemble the receiver for `framing` at `sm_cycles_per_bit` SM cycles per bit.

    The checks are the ones `driver.UartRx` cannot derive for itself: the program must fit the
    32-word IMEM, and `hunt`, `idle` and `reject` -- the three addresses the host and the wrap
    window name -- must exist.
    """
    from asm.pioasm import assemble

    prog = assemble(program_source(framing, sm_cycles_per_bit))[0]
    if len(prog.instrs) > 32:
        raise ValueError(f"{prog.name} for {framing} is {len(prog.instrs)} words, IMEM holds 32")
    for label in ("idle", "hunt", "reject", "good_stop", "framing_error"):
        if label not in prog.labels:
            raise ValueError(f"{prog.name}: the program has no `{label}` label")
    return prog

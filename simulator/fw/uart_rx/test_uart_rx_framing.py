"""fw/uart_rx: every UART frame the shipped 8N1 program cannot receive.

The receiver half of the hole `docs/mvp-verification-report.md` `### Known holes in the stimulus`
names: "UART covers 8N1 only; no parity, no 7 or 9 data bits, no two stop bits.  Nothing runs
below 9 600 Bd on the UART RX path."  `fw/uart_tx/test_uart_tx_framing.py` closes the transmitter
half; this file closes the receiver half.

What is added, and by what:

*   `fw/uart_rx/framing.py` generates the receiver for a framing from the shape of `program.pio`.
    `test_generated_8n1_is_the_shipped_program` assembles both and compares the words, so the
    generator and the firmware that ships cannot drift apart and the existing 8N1 tests keep
    testing the program in the tree.
*   `verif.vip.uart.UartTx`, **unmodified**, drives the line, configured with the data-bit count,
    the parity mode and the stop-bit count of the frame under test.  Every test asserts the decoded
    values, the framing and parity error flags, the marker words and the chip's own `FDEBUG`; no
    test decides pass or fail from the ISS internals.
*   The timing is the part that has to be *derived* rather than copied: a generated program with
    the right number of samples can still sample the wrong bit.  `test_every_sample_lands_in_the
    middle_of_its_own_bit` measures where the samples actually are -- in SM cycles, from the
    engine's own program counter -- and checks them against the nominal grid, for all 18 framings.

Negative controls, one per new claim:

*   `test_a_wrong_parity_bit_on_the_wire_is_reported` -- a transmitter with the parity bit
    inverted, which is the only way to show the host-side parity check is not vacuous.
*   `test_a_low_second_stop_bit_is_a_framing_error` -- a transmitter whose *second* stop bit is
    low, which the program can only notice if it really checks both.
*   `test_the_sample_grid_has_one_bit_of_resolution` -- the same measurement with one `in pins, 1`
    deleted, which must move the first stop-bit check 0.25 bit times out of place: *later*, because
    the sample it deletes is the one carrying the short `3 * S / 4 - 1` delay.

Rates: 300 Bd to 3 Mbd, both by simulation and by the `CLKDIV` arithmetic that reaches them.
300 Bd is 166 667 core cycles per bit time, so that rung is model-only (`SLOW`); everything else in
the ladder runs on every target.
"""
from __future__ import annotations

import copy
from typing import Optional

import pytest

from verif.vip.uart import UartTx

from fw import targets
from fw.uart_rx import framing as framing_mod
from fw.uart_rx.driver import RX_WORD_DATA_MASK, UartRx, build_program, clkdiv
from fw.uart_rx.framing import (
    DEFAULT,
    PARITY_ERROR_MARKER,
    Framing,
    all_framings,
    layout,
    program_for,
    sample_delays,
    stop_gap_bit_times,
)

RX_PIN = 0
BAUD = 115200

#: Model-only rungs: millions of core cycles (`fw/targets.py`'s `is_fast()`, the rule
#: `fw/wiegand/test_wiegand.py` uses).
SLOW = pytest.mark.skipif(not targets.is_fast(),
                          reason="millions of core cycles per byte: a model target only "
                                 "(see the module docstring)")

#: The four framings checked on every target: the shipped 8N1, a shorter frame with parity, a
#: longer one with two stops, and the ninth data bit.
PORTABLE = [Framing(8, "none", 1), Framing(7, "even", 2), Framing(8, "odd", 2),
            Framing(9, "none", 1)]


def transmitter(rx: UartRx, rate: float = 1.0) -> UartTx:
    """A `UartTx` on the receiver's net, running at `rate` times the rate `rx` was built for."""
    return rx.bus.attach(UartTx(rx.net, ticks_per_bit=rx.ticks_per_bit / rate,
                                data_bits=rx.framing.data_bits, parity=rx.framing.parity,
                                stop_bits=rx.framing.stop_bits))


class InvertedParityTx(UartTx):
    """A `UartTx` whose parity bit is wrong, the fault a noisy line looks like on the wire.

    This is the fault `verif/vip/uart.py` documents (override `frame_levels`, the function it
    drives), and it is the negative control for the receiver's host-side parity check.
    """

    def frame_levels(self, byte: int):
        levels = super().frame_levels(byte)
        # `frame_levels` is [start] + the data bits + [parity] + the stop bits, so the parity bit
        # is the one after the last data bit (`verif/vip/uart.py` builds it that way).
        levels[self.data_bits + 1] ^= 1
        return levels


class LowStopTx(UartTx):
    """A `UartTx` with one of its stop bits forced low, to drive the framing-error path.

    `which` is -1 for the last stop bit and -2 for the one before it.  With a two-stop-bit framing
    the last one is only inside the frame if the program really checks it, which is what
    `test_a_low_stop_bit_is_a_framing_error` uses it for.
    """

    def __init__(self, *args, which: int = -1, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.which = which

    def frame_levels(self, byte: int):
        levels = super().frame_levels(byte)
        levels[self.which] = 0
        return levels


def receive(rx: UartRx, tx: UartTx, n_bytes: int, slack_bits: float = 10.0) -> None:
    """Run the bus until `tx` has sent `n_bytes` frames and the host has read them all."""
    def done() -> bool:
        return not tx.busy and tx.sent >= n_bytes and len(rx.received) >= n_bytes

    limit = int((12.0 * n_bytes + slack_bits) * rx.ticks_per_bit)
    assert rx.run_until(done, limit), (
        f"the receiver did not deliver {n_bytes} byte(s) in {limit} ticks "
        f"(sent {tx.sent}, received {len(rx.received)})")


# ---------------------------------------------------------------------------------------------
# The generator: 8N1 is the shipped program, and every framing fits the IMEM
# ---------------------------------------------------------------------------------------------

def test_generated_8n1_is_the_shipped_program() -> None:
    """The generated 8N1 program and `program.pio` are the same 24 words at the same labels.

    Without this the generator could drift away from the firmware that ships: the existing 8N1
    tests in `test_uart_rx.py` run `program.pio` and these run the generator, and nothing else
    would notice.  The words are compared, not the text, so a comment cannot fail this -- and the
    generated source spells out `d1`..`d7`, which `program.pio` leaves unnamed, so the label
    dictionaries are compared only on the addresses the driver and the wrap window name.
    """
    shipped = build_program(16)
    generated = program_for(DEFAULT, 16)
    assert generated.instrs == shipped.instrs, (
        f"the generated 8N1 program is not program.pio:\n  shipped    "
        f"{[f'{w:#06x}' for w in shipped.instrs]}\n  generated  "
        f"{[f'{w:#06x}' for w in generated.instrs]}")
    for label in ("idle", "hunt", "framing_error", "good_stop", "reject"):
        assert generated.labels[label] == shipped.labels[label], label
    assert generated.wrap == shipped.wrap and generated.wrap_target == shipped.wrap_target


def test_the_word_layout_for_8n1_is_the_shipped_one() -> None:
    """8N1's marker mask is `RX_WORD_DATA_MASK` and its data starts at bit 24.

    `fw/uart_rx/driver.py` decodes 8N1 with `RX_WORD_DATA_MASK` and `(word >> 24) & 0xFF`; this
    says the framing-aware decoder computes exactly those numbers for 8N1, which is what makes
    every existing 8N1 test a test of the new path.
    """
    eight_n_one = layout(Framing())
    assert eight_n_one.marker_mask == RX_WORD_DATA_MASK == 0x00FF_FFFF
    assert eight_n_one.data_shift == 24 and eight_n_one.data_mask == 0xFF
    assert eight_n_one.shift_bits == 8
    # With parity the field is one bit wider and the parity bit is the last one sampled, so it is
    # at bit 31 and the marker space is the bits below the frame.
    with_parity = layout(Framing(8, "even", 1))
    assert with_parity.data_shift == 23 and with_parity.parity_shift == 31
    assert with_parity.marker_mask == 0x007F_FFFF
    assert layout(Framing(9, "odd", 2)).data_shift == 22
    assert layout(Framing(7)).data_shift == 25


def test_sample_delays_are_derived_not_copied() -> None:
    """The generated delays: full bit-time spacing, and a short delay on the last sample.

    `sample_delays` is what the generator writes into the source, so this checks the arithmetic it
    is built on: every sample but the last one before the stop check is `S - 1`; without parity
    that last one is `S * 3 / 4 - 1`, which puts the stop check a quarter of the way into the stop
    bit (`program.pio`'s own choice for 8N1), and with parity it is `S - 1` like all the others, so
    the stop check lands in the middle of the first stop bit.
    """
    for sm in (4, 6, 8, 16, 32):
        # No parity: every data bit but the last is a bit time from the one before, and the last
        # carries the short delay that puts the stop-bit check a quarter of the way into it.
        assert sample_delays(Framing(8, "none", 1), sm) == [sm - 1] * 7 + [sm * 3 // 4 - 1]
        assert sample_delays(Framing(9), sm) == [sm - 1] * 8 + [sm * 3 // 4 - 1]
        # With parity every sample is a bit time from the one before -- the parity bit included --
        # so the stop check that follows, at delay 0, is in the middle of the first stop bit.
        assert sample_delays(Framing(8, "even", 1), sm) == [sm - 1] * 9
        assert sample_delays(Framing(7, "odd", 2), sm) == [sm - 1] * 8
        # The bit-time spacing between two stop checks.
        assert framing_mod.stop_delays(Framing(8, "none", 2), sm) == [sm - 1]
        assert framing_mod.stop_delays(Framing(8, "none", 1), sm) == []
        assert stop_gap_bit_times(Framing(8, "none", 1)) == 0.75
        assert stop_gap_bit_times(Framing(8, "even", 1)) == 1.0


def test_every_framing_is_a_legal_imem_image() -> None:
    """All 18 framings assemble at every S the program allows, and fit 32 words.

    9 data bits with parity and two stop bits is the largest: five words of preamble, ten samples,
    two stop checks, five words of framing-error block, four of good-stop block and one of reject.
    """
    seen = list(all_framings())
    assert len(seen) == 18, seen
    for framing in seen:
        for sm in (4, 16, 32):
            prog = program_for(framing, sm)
            assert len(prog.instrs) <= 32, f"{framing} at S={sm}: {len(prog.instrs)} words"
            assert prog.wrap_target == prog.labels["idle"] and prog.wrap == prog.labels["reject"]
            assert set(("idle", "hunt", "framing_error", "good_stop", "reject")) <= set(prog.labels)


def test_the_framing_refuses_what_it_cannot_receive() -> None:
    """A data-bit count outside 5..9, an unknown parity, a stop-bit count outside 1..2.

    The framing itself is validated by `fw.uart_tx/framing.py`, which both directions share; what
    is checked here is that the receiver actually goes through it.
    """
    for args in ((4, "none", 1), (10, "none", 1), (8, "maybe", 1), (8, "none", 0), (8, "none", 3)):
        with pytest.raises(ValueError):
            UartRx(baud=BAUD, rx_pin=RX_PIN, framing=Framing(*args))
    # 8 data bits and two stop bits with S = 4 is a legal program but not a legal 115200 baud:
    # the existing argument still applies, and it applies to a generated program too.
    with pytest.raises(ValueError, match="fastest this program goes"):
        UartRx(baud=50e6 / 4 * 1.01, rx_pin=RX_PIN, framing=Framing(9, "odd", 2),
               sm_cycles_per_bit=16)


# ---------------------------------------------------------------------------------------------
# Every framing on the wire, judged by the unmodified VIP
# ---------------------------------------------------------------------------------------------

def _params(framings) -> list:
    """One `pytest.param` per framing: the portable four run on every target, the rest on models."""
    return [pytest.param(f, marks=() if f in PORTABLE else SLOW) for f in framings]


def payload(framing: Framing, count: int = 4) -> list[int]:
    """`count` values with a 0, a run of ones and the top data bit set, all `data_bits` wide.

    The four are the minimum that puts both levels in every bit position of the frame: an all-zero
    value, one with the two ends set, an alternating one, and one with only the top bit set.
    """
    m = framing.mask
    four = [0x00, 0x41 & m, 0x55 & m, (1 << (framing.data_bits - 1)) & m]
    return (four * (count // 4 + 1))[:count]


def _check(rx: UartRx, tx: UartTx, data: list[int]) -> None:
    """The verdict on one transfer: every value, no error flag, no marker, no FIFO complaint."""
    assert rx.values == data, (
        f"{rx.framing}: {[hex(v) for v in rx.values]} != {[hex(v) for v in data]}")
    assert rx.framing_errors == 0, f"{rx.framing}: a stop bit was low"
    assert rx.parity_errors == 0, f"{rx.framing}: the parity bit did not match the data bits"
    assert rx.markers == [], f"{rx.framing}: marker words {rx.markers}"
    assert rx.chip.fdebug == 0, f"FDEBUG {rx.chip.fdebug:#010x}: a FIFO stall or underrun"


@pytest.mark.parametrize("framing", _params(all_framings()), ids=lambda f: f.name)
def test_every_framing_decodes_on_the_wire(framing: Framing) -> None:
    """All 18 framings, four bytes each, at 115200 Bd on pin 0."""
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
    tx = transmitter(rx)
    data = payload(framing)
    tx.send(data)
    receive(rx, tx, len(data))
    _check(rx, tx, data)


@pytest.mark.parametrize("framing", _params(all_framings()), ids=lambda f: f.name)
@pytest.mark.parametrize("rate", [0.98, 1.02])
def test_the_sender_may_be_two_percent_off(framing: Framing, rate: float) -> None:
    """`CLKDIV` is set for the nominal rate; a sender 2% fast or slow still decodes.

    This is the check that the samples are in the right bits and not merely in the right frame: a
    sample that had drifted into a neighbouring bit shows up here first, and the tighter the frame
    the earlier it shows up.  Two bytes, deliberately: a *fast* sender's frames arrive 2% earlier
    each time, and the receiver re-anchors on every start bit, so the drift accumulates over
    back-to-back frames.  `test_back_to_back_frames_drift_against_a_fast_sender` measures how many
    frames each framing tolerates and what happens after that.
    """
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
    tx = transmitter(rx, rate=rate)
    data = payload(framing, count=2)
    tx.send(data)
    receive(rx, tx, len(data))
    _check(rx, tx, data)


def _frames_before_drift(framing: Framing, rate: float,
                         count: int = 6) -> tuple[int, int, int]:
    """(correct, flagged, received) for `count` back-to-back frames from a `rate`-fast sender.

    `correct` counts positionally, so it falls for two different reasons at once -- a frame that
    decoded to the wrong value, and a frame that never came back at all.  `received` is what tells
    the two apart, which is why it is measured and printed rather than inferred.
    """
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
    tx = transmitter(rx, rate=rate)
    data = payload(framing, count=count)
    tx.send(data)
    rx.run_until(lambda: not tx.busy and len(rx.received) >= count,
                 int((framing.frame_bits + 4) * count * rx.ticks_per_bit))
    good = sum(1 for b, want in zip(rx.received, data) if b.value == want)
    flagged = sum(1 for b in rx.received if b.framing_error or b.parity_error)
    return good, flagged, len(rx.received)


@pytest.mark.parametrize("framing", _params(all_framings()), ids=lambda f: f.name)
def test_back_to_back_frames_drift_against_a_fast_sender(framing: Framing) -> None:
    """A 2%-fast sender is fine for the first frames; the drift is then reported, not hidden.

    The receiver re-anchors on every start bit, so a sender whose frames are 2% short walks 2%
    further ahead of the receiver's grid on every frame.  The margin is `0.5 / frame_bits` --
    `program.pio`'s header works it out as `0.5 / 9.5` for 8N1 -- so the longer the frame the
    sooner the drift is spent.  Two things are asserted, and the second is the one that matters:

    *   at least two frames decode, which is the tolerance `test_the_sender_may_be_two_percent_off`
        already claims;
    *   a framing **with parity** never delivers a byte whose value is wrong without flagging it,
        because the parity bit is on the wire and the host checks it.  That is the difference a
        parity bit makes and the reason this is worth a test: a no-parity framing can only be
        checked on the first frames, which is what the first assertion is.

    The counts are printed so the table of "how many back-to-back frames at 2% fast" is in the test
    output, and `received` is printed with them because "N of 6 correct" counts
    *positions*: at 2 % fast a parity framing is measured receiving 5 bytes for 6 sent, so one
    frame in six is lost outright and the positional count falls for that reason as well as for the
    wrong-value one.  A lost frame carries no flag -- the host never receives a byte for it -- which
    is the one thing the parity bit cannot do about it.
    """
    good, flagged, received = _frames_before_drift(framing, 1.02)
    assert good >= 2, f"{framing}: only {good} of 6 frames decoded at 2% fast"
    if framing.has_parity:
        assert received == 5, (
            f"{framing}: {received} of 6 frames arrived, so the table's 'correct of six' is not "
            f"counting the same thing as last time this was measured")
        rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
        tx = transmitter(rx, rate=1.02)
        data = payload(framing, count=6)
        tx.send(data)
        rx.run_until(lambda: not tx.busy and len(rx.received) >= len(data),
                     int((framing.frame_bits + 4) * len(data) * rx.ticks_per_bit))
        # A byte is "silently wrong" if its value is not one this test sent and neither error
        # flag is set.  Comparing against the set rather than against a position, because a
        # rejected frame shifts every byte after it.
        sent = set(data)
        silent = [b for b in rx.received
                  if b.value not in sent and not (b.framing_error or b.parity_error)]
        assert not silent, (
            f"{framing}: {len(silent)} byte(s) came out wrong and unflagged: "
            f"{[(hex(b.value), b.framing_error, b.parity_error) for b in silent]} -- a parity bit "
            f"on the wire is what makes a mis-sampled bit visible")
    print(f"\n{framing.name}: {good} of 6 back-to-back bytes correct at 2% fast, "
          f"{flagged} flagged, {received} of 6 received")


# ---------------------------------------------------------------------------------------------
# Negative controls: a wrong parity bit and a low second stop bit, on the wire
# ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize("parity", ["even", "odd"])
def test_a_wrong_parity_bit_on_the_wire_is_reported(parity: str) -> None:
    """The host's parity check bites: a transmitter with the parity bit inverted is caught.

    Without this, "no parity errors" could mean "nothing checks the parity bit".  With it, the
    value still decodes -- as a real receiver reports it -- but the byte carries `parity_error` and
    a marker word names the error, and `good_values` drops it.
    """
    framing = Framing(8, parity, 1)
    data = [0x41, 0x7E, 0xC3]
    good = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
    good_tx = transmitter(good)
    good_tx.send(data)
    receive(good, good_tx, len(data))
    _check(good, good_tx, data)

    bad = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
    bad_tx = bad.bus.attach(InvertedParityTx(bad.net, ticks_per_bit=bad.ticks_per_bit,
                                             data_bits=framing.data_bits, parity=framing.parity,
                                             stop_bits=framing.stop_bits))
    bad_tx.send(data)
    receive(bad, bad_tx, len(data))
    assert bad.values == data, "only the parity bit is wrong, the data bits still decode"
    assert bad.parity_errors == 3, f"{framing}: {bad.parity_errors} parity errors, wanted 3"
    assert bad.markers == [PARITY_ERROR_MARKER] * 3, bad.markers
    assert bad.framing_errors == 0, "a parity error is not a framing error"
    assert bad.good_values == [], "a byte with a parity error is not a good byte"


@pytest.mark.parametrize("which", [-2, -1], ids=["first stop bit low", "second stop bit low"])
def test_a_low_stop_bit_is_a_framing_error(which: int) -> None:
    """Both stop bits are checked: a low first one and a low second one are both reported.

    The second case is the control for the *second* stop check.  With `8N2` the second stop bit is
    a whole bit time after the first, inside the frame only because the program samples it, so a
    program that stopped after one `jmp pin` -- the shipped 8N1 shape -- would decode these two
    bytes with no error at all.  `test_the_shipped_program_cannot_see_the_second_stop_bit` is that
    program, run on the same wire.
    """
    framing = Framing(8, "none", 2)
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
    # `idle_bits = 1.0` because a frame that ends with a low stop bit leaves the line low, and the
    # receiver finds the next start bit on a 1 -> 0 transition: without the idle bit the second
    # frame would be missed entirely, which is the rule `program.pio`'s header states and
    # `test_uart_rx.py::test_framing_error_is_reported_in_the_fifo` already covers.
    tx = rx.bus.attach(LowStopTx(rx.net, ticks_per_bit=rx.ticks_per_bit, idle_bits=1.0,
                                 data_bits=framing.data_bits, parity=framing.parity,
                                 stop_bits=framing.stop_bits, which=which))
    data = [0x41, 0x42]
    tx.send(data)
    receive(rx, tx, len(data))
    assert [b.framing_error for b in rx.received] == [True, True], (
        f"{framing}: stop bit {-which} was forced low and nothing was reported")
    assert rx.markers == [1, 1], rx.markers
    assert rx.values == data, "the data bits still decode: only a stop bit was low"
    assert rx.parity_errors == 0


def test_the_shipped_program_cannot_see_the_second_stop_bit() -> None:
    """The same wire, read by the shipped 8N1 program: the low second stop bit is not reported.

    This is the other half of the control above, and it is what makes "the second stop bit is
    checked" a claim about the *program* rather than about the framing the test configured.  The
    receiver here is `8N1` -- one stop-bit check, the shipped words, asserted equal to
    `program.pio` -- and the transmitter sends `8N2` with its second stop bit low.  The byte
    decodes with no framing error, because nothing sampled that bit.

    What the one-stop program does with the low stop bit instead is resynchronise on it: it is a
    falling edge, so it is the next start bit, and the receiver decodes one extra (garbled) frame
    from it.  That is a one-stop receiver behaving correctly on a two-stop-bit wire, and it is why
    the check above is about the *flag on the byte* rather than about the frame count.
    """
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=Framing(8, "none", 1))
    assert rx.program.instrs == build_program(16).instrs, "the 8N1 receiver is the shipped program"
    tx = rx.bus.attach(LowStopTx(rx.net, ticks_per_bit=rx.ticks_per_bit, idle_bits=1.0,
                                 data_bits=8, parity="none", stop_bits=2, which=-1))
    tx.send([0x41])
    assert rx.run_until(lambda: not tx.busy and len(rx.received) >= 1,
                        int(13 * rx.ticks_per_bit)), "the byte never arrived"
    assert rx.values[0] == 0x41, f"{[hex(v) for v in rx.values]}"
    assert rx.framing_errors == 0, (
        "a one-stop receiver has no sample in the second stop bit, so this is expected")


# ---------------------------------------------------------------------------------------------
# Where the samples actually land: the timing, measured from the engine's own program counter
# ---------------------------------------------------------------------------------------------

def _is_in_pins(word: int) -> bool:
    """True for `in pins, N`: opcode 0b010, source PINS (0b000), docs/mvp-isa.md `## Encoding`."""
    return (word >> 13) == 0b010 and ((word >> 5) & 0x7) == 0b000


def _is_jmp_pin(word: int) -> bool:
    """True for `jmp pin, L`: opcode 0b000, condition `pin` (0b110)."""
    return (word >> 13) == 0b000 and ((word >> 5) & 0x7) == 0b110


def sample_ticks(rx: UartRx) -> tuple[list[int], list[int]]:
    """The tick at which each `in pins, 1` and each stop-bit check of the first frame ran.

    Two different observables, because one of them alone is not enough:

    *   the **ISR** for the samples.  Every `in pins, 1` shifts exactly one bit into it, so a
        change in the ISR *is* a sample, with no dependence on how the model reports the program
        counter.  The payload is the all-ones value of the framing on purpose: shifting a 0 into an
        ISR that already holds that pattern at bit 31 would leave the value unchanged, and a zero
        data bit in the middle of a frame would then be invisible.  With all ones, every sample
        changes the ISR.  (`in null, 32` at the top of the frame clears an ISR that is already 0 --
        the previous `push block` did that -- so it produces no change and is not counted.)
    *   the **program-counter transitions** for the stop checks.  The counter the model exposes
        leads the instruction that is executing by the fetch pipeline, so its *value* names a
        different word than the one running; its *transitions*, though, are instruction boundaries,
        and only the boundaries are needed.  After the last sample each transition is the start of
        the next stop-bit check: the `jmp framing_error` that sits between two checks is jumped
        over, and a jumped-over instruction has no boundary.  (A stop bit that is *low* does take
        that jump, which is one extra transition -- so this measures the good path, and the
        low-stop-bit path is `test_a_low_stop_bit_is_a_framing_error`.)

    Nothing here reads the model's bookkeeping about what an instruction did: only the ISR value,
    and which word the counter names.
    """
    tx = transmitter(rx)
    tx.send([rx.framing.mask])
    samples: list[int] = []
    transitions: list[int] = []
    isr = 0
    last_pc: Optional[int] = None
    limit = int((rx.framing.frame_bits + 4) * rx.ticks_per_bit)
    for i in range(limit):
        state = rx.chip.state(0)
        if state["isr"] != isr:
            isr = state["isr"]
            if not transitions:             # only the frame's own samples, not `push`'s clear
                samples.append(i)
        elif samples and last_pc is not None and state["pc"] != last_pc:
            transitions.append(i)
            if len(transitions) == rx.framing.stop_bits:
                break
        last_pc = state["pc"]
        rx.step()
    return samples, transitions


def _check_grid(rx: UartRx, samples: list[int], stops: list[int]) -> float:
    """Assert the sample spacing and return the worst error in ticks.

    The spacing, not the absolute position: the engine's cycle grid is quantised to the bit time
    (`docs/mvp-isa.md` `## Timing`), so where the first sample falls is a phase in `[0, 1)` ticks
    and only the *differences* are the program's doing.  What the generated delays promise is that
    consecutive samples are one bit time apart, that the stop check is three quarters of a bit time
    after the last sample, and that a second stop check is a whole bit time after the first.
    """
    tpb = rx.ticks_per_bit
    assert len(samples) == rx.framing.shift_bits, (
        f"{rx.framing}: {len(samples)} samples ran, the program has {rx.framing.shift_bits}")
    assert len(stops) == rx.framing.stop_bits, (
        f"{rx.framing}: {len(stops)} stop checks ran, the framing has {rx.framing.stop_bits}")
    gap = stop_gap_bit_times(rx.framing) * tpb
    worst = 0.0
    for a, b in zip(samples, samples[1:]):
        worst = max(worst, abs((b - a) - tpb))
    for a, b in zip(stops, stops[1:]):
        worst = max(worst, abs((b - a) - tpb))
    worst = max(worst, abs((stops[0] - samples[-1]) - gap))
    assert worst < 1.0, (
        f"{rx.framing}: a sample is {worst:.3f} ticks from where it should be (one tick of the "
        f"{tpb:.4f}-tick grid is the resolution)")
    return worst


@pytest.mark.parametrize("framing", _params(all_framings()), ids=lambda f: f.name)
def test_every_sample_lands_in_its_own_bit(framing: Framing) -> None:
    """Every framing's samples are one bit time apart, and its stop checks are 3/4 and 1 bit later.

    A generated program with the right number of samples can still sample the wrong bits, and a
    decoder configured with the framing would hide it -- it is told where the bits are.  So this
    measures the timing from the engine's program counter instead and holds it to the delays
    `fw/uart_rx/framing.py` derived: `S - 1` between samples (one bit time), `S / 4 - 1` or
    `S * 3 / 4 - 1` before the first stop check without parity and `S - 1` all the way with it,
    and `S - 1` between two stop checks (one bit time).
    """
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing)
    samples, stops = sample_ticks(rx)
    worst = _check_grid(rx, samples, stops)
    print(f"\n{framing.name}: {len(samples)} samples + {len(stops)} stop checks, worst spacing "
          f"error {worst:.3f} ticks of {rx.ticks_per_bit:.3f}")


def test_the_sample_grid_has_one_bit_of_resolution() -> None:
    """The negative control for the grid measurement: one `in pins, 1` deleted.

    Deleting the last data-bit sample moves the first stop-bit check 0.25 bit times out of the grid
    -- 108.5 ticks of a 434.028-tick grid, a hundred ticks against a one-tick resolution -- which is
    what proves the measurement above is looking at where the samples are and not merely counting
    them.  The direction is *later*, not earlier, and that is worth being explicit about: the
    sample the control deletes is the one carrying the short `3 * S / 4 - 1` delay, so the check
    that followed it loses that shortened hop and now sits on the far side of a whole sample.  The
    byte decodes wrongly as well; the point of this control is the timing error, which is what the
    grid check claims to measure.
    """
    framing = Framing(8, "none", 2)
    broken_program = copy.deepcopy(program_for(framing, 16))
    data_bits = [i for i, w in enumerate(broken_program.instrs) if _is_in_pins(w)]
    assert len(data_bits) == 8, data_bits
    broken_program.instrs.pop(data_bits[-1])
    # Every label above the deleted word moves down by one, and the wrap window with it.
    for name, addr in list(broken_program.labels.items()):
        if addr > data_bits[-1]:
            broken_program.labels[name] = addr - 1
    broken_program.wrap_target = broken_program.labels["idle"]
    broken_program.wrap = broken_program.labels["reject"]

    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, framing=framing, program=broken_program)
    samples, stops = sample_ticks(rx)
    tpb = rx.ticks_per_bit
    assert len(samples) == 7 and len(stops) == 2, (
        f"the control is broken: {len(samples)} samples and {len(stops)} stop checks ran")
    # With the short-delay sample gone the stop check lands 0.25 bit times late: it has lost the
    # 0.75-bit hop the deleted sample gave it and now sits a whole sample further on.
    error = abs((stops[0] - samples[-1]) - stop_gap_bit_times(framing) * tpb)
    assert error > 20.0, (
        f"the control is broken: deleting a sample moved the stop check by only {error:.1f} ticks, "
        f"so the grid check cannot see a missing sample")
    # `_check_grid` rejects it twice over: the sample count is one short *and* the stop check is
    # a quarter of a bit time out.  Either message is a rejection; both are the grid check.
    try:
        _check_grid(rx, samples, stops)
    except AssertionError as exc:
        assert ("samples ran" in str(exc)) or ("ticks from where it should be" in str(exc)), exc
    else:
        raise AssertionError("the grid check passed a program with a sample deleted")
    print(f"\ncontrol: a program with one sample deleted is {error:.1f} ticks out "
          f"({error / tpb:.3f} bit times)")


# ---------------------------------------------------------------------------------------------
# 300 Bd to 3 Mbd
# ---------------------------------------------------------------------------------------------

#: (baud, SM cycles per bit, bytes, model-only?).  One bit time is `S` SM cycles and `CLKDIV`
#: cannot go below 1, so `S` is the fastest bit time a given baud can have (S = 16 is the program's
#: default).  300 Bd is 166 667 core cycles per bit, so that rung is model-only and carries one
#: byte; the ladder thins out at the slow end rather than the fast one.
RATES = [
    pytest.param(300.0, 4, 1, True, id="300"),
    pytest.param(9600.0, 16, 2, False, id="9600"),
    pytest.param(115200.0, 16, 4, False, id="115200"),
    pytest.param(1_000_000.0, 16, 4, False, id="1000000"),
    pytest.param(3_000_000.0, 16, 4, False, id="3000000"),
]


@pytest.mark.parametrize("baud,sm,count,slow", RATES)
def test_the_rate_ladder(baud: float, sm: int, count: int, slow: bool) -> None:
    """300 Bd to 3 Mbd, the same way: bytes in, judged by the values that come out.

    300 Bd runs with `S = 4`, the smallest the program allows, because `CLKDIV = ticks_per_bit / S`
    has to fit the 16-bit integer field: at 300 Bd and S = 16 the divisor would be 10 417 (fine),
    but at S = 4 it is 41 667 (also fine) and the bit time is then 666 667 core cycles per frame
    bit.  The arithmetic that reaches every rung is `test_clkdiv_reaches_the_ladder`, which runs on
    every target.
    """
    if slow and not targets.is_fast():
        pytest.skip(f"{baud:g} Bd is a frame of {50e6 / baud:.0f} core cycles; model targets only")
    framing = Framing(8, "even", 1)
    rx = UartRx(baud=baud, rx_pin=RX_PIN, sm_cycles_per_bit=sm, framing=framing)
    tx = transmitter(rx)
    data = payload(framing, count=count)
    tx.send(data)
    receive(rx, tx, len(data))
    _check(rx, tx, data)
    assert rx.chip.engines[0].clkdiv == clkdiv(rx.ticks_per_bit / sm), (
        f"{baud:g} Bd at S={sm}: CLKDIV is {rx.chip.engines[0].clkdiv:#010x}")
    print(f"\n{baud:g} Bd at S={sm}: {rx.ticks_per_bit:.4f} ticks per bit, CLKDIV "
          f"{rx.chip.engines[0].clkdiv:#010x}, {len(rx.received)} frames, "
          f"{len(rx.program.instrs)} of 32 IMEM words")


@pytest.mark.parametrize("baud", [300.0, 9600.0, 115200.0, 1_000_000.0, 3_000_000.0])
@pytest.mark.parametrize("sm", [4, 16])
def test_clkdiv_reaches_the_ladder(baud: float, sm: int) -> None:
    """Every rung is inside what a 16.8 `CLKDIV` can express, at both ends of the S range.

    Arithmetic only, so it runs on every target: one bit time is `S * (INT + FRAC / 256)` core
    cycles (`docs/mvp-isa.md` `### Clock divider`), so the bit length moves in steps of `S / 256`
    ticks and the integer field has to hold `CORE_HZ / (baud * S)`.
    """
    tpb = 50e6 / baud
    word = clkdiv(tpb / sm)
    integer, frac = (word >> 16) & 0xFFFF, (word >> 8) & 0xFF
    assert 1 <= integer <= 0xFFFF and 0 <= frac <= 0xFF, (
        f"{baud:g} Bd at S={sm}: CLKDIV {integer}.{frac} does not fit")
    step = sm / 256
    actual = sm * (integer + frac / 256)
    assert abs(actual - tpb) <= step / 2 + 1e-9, (
        f"{baud:g} Bd at S={sm}: CLKDIV {integer}.{frac} gives {actual:.4f} ticks per bit, "
        f"{tpb:.4f} wanted, and a 16.8 divider cannot do better than {step / 2:.4f}")

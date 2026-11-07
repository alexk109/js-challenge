"""fw/uart_tx: every UART frame the shipped 8N1 program cannot send.

`docs/mvp-verification-report.md` `### Known holes in the stimulus` names this hole: "UART covers
8N1 only; no parity, no 7 or 9 data bits, no two stop bits.  Nothing runs below 9 600 Bd on the
UART RX path."  This file closes the transmitter half of it.  `fw/uart_rx/test_uart_rx_framing.py`
closes the receiver half.

What is added, and by what:

*   `fw/uart_tx/framing.py` generates the program for a framing from the shape of the shipped
    `program.pio`; `test_generated_8n1_is_the_shipped_program` assembles both and compares the
    words, so the generator and the firmware that ships cannot drift apart and the existing 8N1
    tests keep testing the program in the tree.
*   `verif.vip.uart.UartRx`, **unmodified**, is the judge, configured with the data-bit count, the
    parity mode and the stop-bit count of the frame that was sent.  Every test asserts the decoded
    values, `parity_errors`, `framing_errors`, `aborted` and the bit-edge deviation; nothing here
    reads the ISS internals to decide pass or fail.
*   `fw/uart_tx/wire_check.py` is a checker beside the VIP, and it exists because a receiver
    cannot tell one stop bit from two on a quiet line, nor count the data bits of a frame whose
    last data bit is 1.  It measures the line itself: the low run from a start bit (the start bit
    plus the data bits) and the high run after it (the stop bits plus the engine's idle gap).
    Two tests below plant the two faults it must catch.

Every new check has a negative control, named in its docstring:

*   `test_a_wrong_parity_bit_is_caught` -- the host sends the parity bit inverted.
*   `test_data_bit_count_is_measurable_on_the_wire` -- the same measurement on a payload whose
    last data bit is 1, which must read exactly one bit time short.
*   `test_two_stop_bits_hold_the_line_one_bit_time_longer` -- a program with the second
    `set pins, 1` deleted, which must read zero bit times longer.

Rates: 300 Bd to 3 Mbd.  300 Bd is one frame of 166 667 core cycles per bit time and two million
ticks of simulation, so `SLOW` marks those tests as model-only (`fw/targets.py`'s `is_fast()`,
the same rule `fw/wiegand/test_wiegand.py` uses); the arithmetic that reaches 300 Bd at all --
`CLKDIV` INT 20 833 -- runs on every target.
"""
from __future__ import annotations

import copy

import pytest

from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.uart import UartRx, parity_bit as judge_parity_bit

from fw import targets
from fw.uart_tx import framing as framing_mod
from fw.uart_tx.driver import (
    CORE_HZ,
    UartTxDriver,
    clkdiv_for_baud,
    framed_bit_cycles,
    load_program,
    ticks_per_bit,
)
from fw.uart_tx.framing import Framing, all_framings, program_for
from fw.uart_tx.wire_check import LineRecorder, frames, high_bit_times, low_bit_times

#: One tick of bus latency between a driven value and the device that reads it
#: (`verif/vip/core.py`), which is what `UartRx`'s `latency=1` is talking about.
LATENCY = 1

#: Tests that need millions of core cycles: model targets only (`fw/targets.py` `is_fast()`).
SLOW = pytest.mark.skipif(not targets.is_fast(),
                          reason="millions of core cycles per byte: a model target only "
                                 "(see the module docstring)")

#: The four framings checked on every target, chosen to cover each axis once: the shipped 8N1, a
#: shorter frame with parity, a longer one with two stops, and the ninth data bit.
PORTABLE = [Framing(8, "none", 1), Framing(7, "even", 2), Framing(8, "odd", 2),
            Framing(9, "none", 1)]

#: `0x41` is the classic choice (0100 0001); the others are the same byte with the frame's width.
#: The top data bit is cleared, which is what makes the low run from a start bit measurable: the
#: line then stays low until the first stop bit, for exactly `1 + data_bits` bit times.
def zero_last_payload(framing: Framing, count: int = 2) -> list[int]:
    return [0x41 & ((1 << (framing.data_bits - 1)) - 1)] * count


def one_last_payload(framing: Framing, count: int = 2) -> list[int]:
    """The same payload with the top data bit set, so the line rises one bit time earlier."""
    return [zero_last_payload(framing)[0] | (1 << (framing.data_bits - 1))] * count


class Rig:
    """A bus with the chip, the judge's receiver and the wire recorder on the TX pin."""

    def __init__(self, framing: Framing = Framing(), baud: float = 115200.0, pin: int = 0,
                 program=None):
        self.framing = framing
        self.bus = Bus(ns_per_tick=20.0)
        for k in range(18):
            self.bus.add_net(f"p{k}", push_pull=(k == pin))
        self.dev = self.bus.attach(ChipDevice())
        self.drv = UartTxDriver(self.dev, self.bus, pin=pin, baud=baud, framing=framing,
                                program=program)
        self.tpb = ticks_per_bit(self.drv.clkdiv, self.drv.sm_cycles_per_bit)
        self.rx = self.bus.attach(UartRx(f"p{pin}", ticks_per_bit=self.tpb,
                                         data_bits=framing.data_bits, parity=framing.parity,
                                         stop_bits=framing.stop_bits, latency=LATENCY))
        self.rec = self.bus.attach(LineRecorder(f"p{pin}"))

    def run(self, data, extra_bits: float = 6.0) -> None:
        """Load, start, send `data` as byte values, and run long enough for the last frame."""
        self.drv.load()
        self.drv.start(first=list(data[:self.drv.TXFIFO_DEPTH]))
        self.drv.send_bytes(data[self.drv.TXFIFO_DEPTH:])
        self.bus.run(int((len(data) * (self.framing.frame_bits + 2) + extra_bits) * self.tpb))

    def measured(self) -> list:
        """The frames the judge decoded, measured on the wire by the checker beside it."""
        return frames(self.rec.transitions, [b.tick for b in self.rx.received], offset=LATENCY)


# ---------------------------------------------------------------------------------------------
# The generator: 8N1 is the shipped program, and every framing is a legal IMEM image
# ---------------------------------------------------------------------------------------------

def test_generated_8n1_is_the_shipped_program() -> None:
    """The generated 8N1 program and `program.pio` are the same 12 words.

    Without this the generator could drift away from the firmware that ships: the existing 8N1
    tests in `test_uart_tx.py` run `program.pio`, and these run the generator, and nothing else
    would notice the two diverging.  The words are compared, not the text, so a comment or
    whitespace change on either side cannot fail this.
    """
    shipped = load_program()
    generated = program_for(Framing())
    assert generated.instrs == shipped.instrs, (
        f"the generated 8N1 program is not program.pio:\n  shipped    "
        f"{[f'{w:#06x}' for w in shipped.instrs]}\n  generated  "
        f"{[f'{w:#06x}' for w in generated.instrs]}")
    assert generated.labels == shipped.labels
    assert generated.wrap == shipped.wrap and generated.sideset_bits == shipped.sideset_bits


def test_pack_is_the_identity_for_8n1() -> None:
    """`Framing().pack(v) == v` for every byte value, so the 8N1 path is bit for bit the old one.

    `preload()` and `send_bytes()` go through `pack()`, and the existing 8N1 tests push byte
    values through them.  This is the check that says those tests are unchanged by that.
    """
    eight_n_one = Framing()
    for value in range(256):
        assert eight_n_one.pack(value) == value
    assert Framing(9, "none", 1).mask == 0x1FF and Framing(7, "none", 1).mask == 0x7F


def test_parity_bit_agrees_with_the_judge() -> None:
    """The firmware's parity bit and the judge's are one function, over 1 024 cases.

    `fw/uart_tx/framing.py` writes the convention out itself rather than importing it, so that
    the host and the judge do not agree by sharing one function.  This is what turns "two
    implementations" into evidence: every value for both parities, for 7, 8 and 9 data bits.
    """
    for framing in (Framing(7, "even", 1), Framing(8, "odd", 1), Framing(9, "even", 2),
                    Framing(9, "odd", 2)):
        for value in range(0, framing.mask + 1):
            assert framing.parity_bit(value) == judge_parity_bit(value, framing.data_bits,
                                                                 framing.parity), (
                f"{framing}: parity of {value:#x}")


def test_every_framing_is_a_legal_imem_image() -> None:
    """All 18 framings assemble, fit 32 words, and shift exactly the bits they claim to.

    `shift_bits` is `SHIFTCTRL.PULL_THRESH` and the number of `out pins, 1` in the program; the
    two must agree or the OSR refills in the wrong place.
    """
    seen = list(all_framings())
    assert len(seen) == 18, seen
    for framing in seen:
        program = program_for(framing)
        assert len(program.instrs) <= 32, f"{framing}: {len(program.instrs)} words"
        assert program.labels["start"] == 0 and program.wrap == len(program.instrs) - 1
        assert program.sideset_bits == 2 and program.sideset_opt
        assert framed_bit_cycles(program, framing.shift_bits) == framing_mod.SM_CYCLES_PER_BIT
        # One `set pins, 1` per stop bit.  SET is opcode 0b111 and its PINS destination leaves
        # bits 7:5 clear (`docs/mvp-isa.md` `## Encoding`, `SET Destination`), so bits 7:0 == 1
        # is `set pins, 1`; `set pindirs, 1` has bit 7 set and cannot be mistaken for it.
        stops = [w for w in program.instrs if (w & 0xE000) >> 13 == 0b111 and w & 0xFF == 1]
        assert len(stops) == framing.stop_bits, f"{framing}: {len(stops)} stop bits in the words"
        assert framing.frame_bits == 1 + framing.shift_bits + framing.stop_bits


def test_the_framing_refuses_what_it_cannot_send() -> None:
    """A data-bit count outside 5..9, an unknown parity, a stop-bit count outside 1..2."""
    for args in ((4, "none", 1), (10, "none", 1), (8, "maybe", 1), (8, "none", 0),
                 (8, "none", 3)):
        with pytest.raises(ValueError):
            Framing(*args)
    framing = Framing(7, "even", 2)
    with pytest.raises(ValueError):
        framing.pack(0x80)                  # a 7-bit frame has no eighth bit
    with pytest.raises(ValueError):
        Framing(9).pack(0x200)               # nor has a 9-bit one a tenth


# ---------------------------------------------------------------------------------------------
# The frame on the wire, judged by the unmodified VIP
# ---------------------------------------------------------------------------------------------

def _params(framings) -> list:
    """One `pytest.param` per framing: the portable four run on every target, the rest are
    model-only because the whole 18-framing matrix is thousands of simulated core cycles."""
    return [pytest.param(f, marks=() if f in PORTABLE else SLOW) for f in framings]


def _check(rig: Rig, data: list[int]) -> None:
    """The judge's verdict on one run: every value, no parity, framing or glitch error."""
    rx = rig.rx
    assert rx.values == data, f"{rig.framing}: {[hex(v) for v in rx.values]} != " \
                             f"{[hex(v) for v in data]}"
    assert len(rx.received) == len(data), f"{rig.framing}: {len(rx.received)} frames decoded"
    assert rx.parity_errors == 0, f"{rig.framing}: the judge saw a wrong parity bit"
    assert rx.framing_errors == 0, f"{rig.framing}: the judge saw a low stop bit"
    assert rx.aborted == 0, f"{rig.framing}: the judge took a glitch for a start bit"
    assert rx.worst_dev_ticks < 1.0, (
        f"{rig.framing}: bit edges are {rx.worst_dev_ticks:.3f} ticks off the "
        f"{rig.tpb:.4f}-tick grid")
    assert rig.bus.errors == [], rig.bus.errors
    # The number of frames the wire checker measured is the number the judge decoded: a judge
    # that invented a frame (or missed one) cannot slip through the geometry tests below.
    assert len(rig.measured()) == len(rx.received)


@pytest.mark.parametrize("framing", _params(all_framings()), ids=lambda f: f.name)
def test_every_framing_decodes_on_the_wire(framing: Framing) -> None:
    """All 18 framings: 7/8/9 data bits, none/even/odd parity, one or two stop bits.

    Payload: a 0, a 1, a run of ones and the ninth bit set where the frame has one, so every bit
    position of the frame carries both levels somewhere in the run.
    """
    m = framing.mask
    data = [0x00, m, 0x41 & m, (0x15 | (1 << (framing.data_bits - 1))) & m]
    rig = Rig(framing=framing, baud=115200.0)
    rig.run(data)
    _check(rig, data)


@pytest.mark.parametrize("framing", [Framing(9, "even", 2), Framing(7, "odd", 1)],
                         ids=lambda f: f.name)
def test_a_byte_out_of_the_low_eight_bits_is_not_a_framing(framing: Framing) -> None:
    """A value with the ninth bit set only reaches the wire in a 9-bit frame.

    The negative control for the 9-bit path: the same value in an 8-bit frame does not fit, and
    `pack()` refuses it rather than silently truncating the ninth bit away.
    """
    rig = Rig(framing=framing, baud=115200.0)
    if framing.data_bits == 9:
        data = [0x155]
        rig.run(data)
        _check(rig, data)
        assert rig.rx.values == [0x155]
    else:
        with pytest.raises(ValueError, match="7 data bits"):
            framing.pack(0x155)


# ---------------------------------------------------------------------------------------------
# The frame geometry, measured on the line: how many data bits, how many stop bits
# ---------------------------------------------------------------------------------------------

#: A measurement is good to a tick, and a tick is `1 / ticks_per_bit` of a bit.  Two ticks of
#: slack, because the transmitter's own bit edges are allowed a tick each (`UartRx`'s bar, which
#: the tests above hold it to) and the geometry is a difference of two edges.
TICK_SLACK = 2.0


@pytest.mark.parametrize("framing", [Framing(7), Framing(9)], ids=lambda f: f.name)
def test_data_bit_count_is_measurable_on_the_wire(framing: Framing) -> None:
    """The line stays low for `1 + data_bits` bit times from a start bit, and one bit less when
    the last data bit is 1.

    This is the negative control that gives the measurement its resolution: a receiver cannot
    count the data bits of a frame (it is configured with the answer), so the count is measured on
    the wire, and the same measurement on a payload whose last data bit is set *must* read one bit
    time short.  A checker that could not tell the two would also pass a program with the wrong
    number of data bits, so this is the check that says it can.
    """
    low = zero_last_payload(framing)
    rig = Rig(framing=framing, baud=115200.0)
    rig.run(low)
    measured = low_bit_times(rig.measured(), rig.tpb)
    want = 1 + framing.data_bits
    assert all(abs(m - want) * rig.tpb < TICK_SLACK for m in measured), (
        f"{framing}: low runs {[round(m, 3) for m in measured]}, want {want} bit times")

    rig_one = Rig(framing=framing, baud=115200.0)
    rig_one.run(one_last_payload(framing))
    measured_one = low_bit_times(rig_one.measured(), rig_one.tpb)
    assert all(abs((m - want + 1.0) * rig_one.tpb) < TICK_SLACK for m in measured_one), (
        f"{framing}: with the last data bit set the low run must be one bit time shorter, got "
        f"{[round(m, 3) for m in measured_one]}")


def test_two_stop_bits_hold_the_line_one_bit_time_longer() -> None:
    """Two stop bits are one bit time more of high line than one, on the first frame.

    The first frame is the one to measure: it is preloaded into the TX FIFO before the engine
    starts, so the engine never waits for a FIFO word and the high run after its last data bit is
    the stop bits plus the loop's own gap -- the same gap in both runs, which is why the
    difference is exactly one bit time.

    The negative control is a program with the second `set pins, 1` deleted -- the shipped 8N1
    program, which is what is left when the second stop instruction goes.  It is run with the judge
    configured for what it *does* send (8N1), so the only thing the control can be is the geometry,
    and it has to read exactly the one-stop frame's high run.
    """
    def first_frame_high_bits(framing: Framing, program=None) -> tuple:
        rig = Rig(framing=framing, baud=115200.0, program=program)
        rig.run(zero_last_payload(framing, count=2))
        _check(rig, zero_last_payload(framing, count=2))
        measured = rig.measured()
        return measured[0].high_bit_times(rig.tpb), rig.tpb

    one, tpb = first_frame_high_bits(Framing(8, "none", 1))
    two, _ = first_frame_high_bits(Framing(8, "none", 2))
    assert abs((two - one) - 1.0) * tpb < TICK_SLACK, (
        f"two stop bits must hold the line one bit time longer than one: {one:.4f} against "
        f"{two:.4f} bit times, one bit time is {1 / tpb:.4f} ticks")

    # The planted fault: the same 8N2 program with the second stop instruction removed, which
    # leaves the shipped 8N1 program word for word.
    broken = copy.deepcopy(program_for(Framing(8, "none", 2)))
    stops = [i for i, w in enumerate(broken.instrs)
             if (w & 0xE000) >> 13 == 0b111 and w & 0xFF == 1]
    assert len(stops) == 2, stops
    broken.instrs.pop(stops[1])
    broken.wrap = len(broken.instrs) - 1
    assert broken.instrs == load_program().instrs, "the control is not the 8N1 program"
    faulted, _ = first_frame_high_bits(Framing(8, "none", 1), program=broken)
    assert abs(faulted - one) * tpb < TICK_SLACK, (
        f"the control is broken: a program with one stop bit reads {faulted:.4f} bit times, "
        f"which is the one-stop frame's {one:.4f}, so the assertion above cannot tell them apart")


@pytest.mark.parametrize("parity", ["even", "odd"])
def test_a_wrong_parity_bit_is_caught(parity: str) -> None:
    """The judge's parity check bites: the host inverts the parity bit and it says so.

    The negative control for the whole parity path.  Without it, "no parity errors" could mean
    "nothing ever checks the parity bit"; with it, a parity bit that is wrong on the wire is a
    `parity_error` from `verif.vip/uart.py` while the byte itself still decodes, which is what a
    real receiver reports.
    """
    framing = Framing(8, parity, 1)
    data = [0x41, 0x7E, 0xC3]
    good = Rig(framing=framing, baud=115200.0)
    good.run(data)
    _check(good, data)

    bad = Rig(framing=framing, baud=115200.0)
    bad.drv.load()
    bad.drv.start(first=data[:2])          # `start` takes byte values, which `preload` packs
    for value in data[2:]:
        # The raw word path, with the parity bit (bit `data_bits`) inverted.
        bad.drv.push(framing.pack(value) ^ (1 << framing.data_bits))
    bad.bus.run(int((len(data) * (framing.frame_bits + 2) + 6) * bad.tpb))
    assert bad.rx.values == data, "the data bits are still right: only the parity bit is wrong"
    assert bad.rx.parity_errors == 1, (
        f"{framing}: the judge saw {bad.rx.parity_errors} parity errors, the host inverted one "
        "bit on purpose")
    assert bad.rx.framing_errors == 0


# ---------------------------------------------------------------------------------------------
# 300 Bd to 3 Mbd
# ---------------------------------------------------------------------------------------------

#: (baud, bytes, slack in bit times, model-only?).  300 Bd is 166 666.7 core cycles per bit, so
#: even one byte is 1.9 million ticks of simulation: that is the reason the slowest rung is
#: model-only, and it is why the ladder thins out at the slow end rather than at the fast one.
RATES = [
    pytest.param(300.0, 1, 1.5, True, id="300"),
    pytest.param(9600.0, 2, 2.0, False, id="9600"),
    pytest.param(115200.0, 4, 6.0, False, id="115200"),
    pytest.param(1_000_000.0, 4, 6.0, False, id="1000000"),
    pytest.param(3_000_000.0, 8, 6.0, False, id="3000000"),
]


@pytest.mark.parametrize("baud,count,slack,slow", RATES)
def test_the_rate_ladder(baud: float, count: int, slack: float, slow: bool) -> None:
    """300 Bd to 3 Mbd, the same way: bytes out, judged by the receiver at that rate.

    The payload is `count` copies of the geometry payload, so the low run of every frame is
    `1 + data_bits` bit times: at 300 Bd that is a low run of 1.5 million ticks and the test also
    says the transmitter held the line low for exactly that long.

    `SLOW` marks the 300 Bd rung: two million ticks is minutes of simulation on a Verilator-backed
    target, so it runs on the two Python models (`fw/targets.py`'s `is_fast()`, the rule
    `fw/wiegand/test_wiegand.py` uses).  Everything below the 300 Bd rung runs on every target.
    """
    if slow:
        if not targets.is_fast():
            pytest.skip(f"{baud:g} Bd is a frame of {CORE_HZ / baud:.0f} core cycles; "
                        "model targets only (see the module docstring)")
    # 8N1, so the geometry the test measures is `1 + data_bits` with no parity bit to explain:
    # a parity bit of 1 would end the low run one bit time early and the ladder is about the rate.
    framing = Framing(8, "none", 1)
    data = zero_last_payload(framing, count=count)
    rig = Rig(framing=framing, baud=baud)
    rig.run(data, extra_bits=slack)
    _check(rig, data)
    low = low_bit_times(rig.measured(), rig.tpb)
    assert all(abs(m - 9.0) * rig.tpb < TICK_SLACK for m in low), (
        f"{baud:g} Bd: the low runs are {[round(m, 4) for m in low]} bit times, want 9 "
        f"({9 * rig.tpb:.0f} ticks each)")
    print(f"\n{baud:g} Bd: {rig.tpb:.4f} ticks per bit (nominal {CORE_HZ / baud:.4f}), "
          f"{len(rig.rx.received)} frames, worst edge "
          f"{rig.rx.worst_dev_ticks:.3f} ticks")


@pytest.mark.parametrize("baud", [300.0, 9600.0, 115200.0, 250_000.0, 1_000_000.0,
                                  2_000_000.0, 3_000_000.0, 6_250_000.0])
def test_clkdiv_reaches_the_whole_ladder(baud: float) -> None:
    """Every rate from 300 Bd to the program's ceiling is inside what `CLKDIV` can express.

    Arithmetic only, so it runs on every target: `docs/mvp-isa.md` `### Clock divider` gives a
    16.8 divisor, one bit lasts `8 * (INT + FRAC/256)` core cycles, and the bit length therefore
    moves in steps of `8 / 256` ticks.  300 Bd needs INT 20 833 and 3 Mbd needs INT 2 with
    FRAC 21, both inside the field; the ceiling is INT 1, one SM cycle per instruction.
    """
    clkdiv = clkdiv_for_baud(baud, framing_mod.SM_CYCLES_PER_BIT)
    integer, frac = (clkdiv >> 16) & 0xFFFF, (clkdiv >> 8) & 0xFF
    assert 1 <= integer <= 0xFFFF and 0 <= frac <= 0xFF
    step = framing_mod.SM_CYCLES_PER_BIT / 256
    actual = ticks_per_bit(clkdiv, framing_mod.SM_CYCLES_PER_BIT)
    assert abs(actual - CORE_HZ / baud) <= step / 2 + 1e-9, (
        f"{baud:g} Bd: CLKDIV {integer}.{frac} gives {actual:.4f} ticks per bit, "
        f"{CORE_HZ / baud:.4f} wanted, and a 16.8 divider cannot do better than {step / 2:.4f}")
    if baud == 300.0:
        assert integer == 20833 and frac == 85, f"300 Bd is CLKDIV {integer}.{frac}"
    if baud == 6_250_000.0:
        assert clkdiv == clkdiv_for_baud(CORE_HZ / 8, 8), "the ceiling is CLKDIV 1.0"

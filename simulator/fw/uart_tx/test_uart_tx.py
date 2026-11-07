"""End-to-end test of the MVP UART-TX firmware.

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with
nets p0..p17, exactly as `docs/mvp-interfaces.md` `## ISS API` describes, the host driver
loads the program and pushes bytes, and `verif.vip.uart.UartRx` -- the independent judge,
which this test does not modify -- decodes the line.  Nothing here looks at the ISS
internals to decide pass or fail: the byte values, the framing flags and the bit-edge
deviation all come from the VIP.

    make fw PROTO=uart_tx TARGET=iss      # runs
    make fw PROTO=uart_tx TARGET=rtl      # runs too: src/pio_chip.sv under cocotb, same checks

`FW_TARGET=rtl` swaps `iss.pio_iss.Chip` for a chip on the RTL and changes nothing else
(`fw/rtl_harness.py`).

Two receivers sit on the TX net, one per bit grid:

* `rx` is configured with the bit length the chip actually produces, `ticks_per_bit(clkdiv,
  8)`.  That is the grid a real receiver recovers from the line, and it isolates the
  transmitter's jitter: `docs/mvp-isa.md` `### Clock divider` fixes the algorithm, so N SM
  cycles take `N*INT + floor(N*FRAC/256)` core cycles exactly (`fw.uart_tx.report` prints
  that share per baud).  What is left is the pass criterion: every bit edge within one
  tick.  The tests below plant a fault and it reports tens of ticks.
* `rx_nominal` is configured with the requested baud's nominal grid, `CORE_HZ / baud`.  The
  same check on the requested grid additionally carries the rate error of the 16.8 divider,
  up to `8/256` of a tick per SM cycle, so it is held to `1 + 10 * |nominal - actual|`: the
  bar plus exactly what CLKDIV can be asked to express.  (UartRx measures edges in whole
  ticks, so either grid can legitimately read up to a tick above the arithmetic.)
* Whether the requested baud is reachable at all is a separate, exact check:
  `test_clkdiv_hits_the_baud_within_the_divider_resolution`.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass

import pytest

import fw.uart_tx.driver as driver_mod
from fw.uart_tx.driver import (
    CORE_HZ,
    UartTxDriver,
    bit_cycles,
    clkdiv_for_baud,
    clkdiv_value,
    load_program,
    ticks_per_bit,
    words_used,
)
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.uart import UartRx
from fw import regmap as _regmap

IMEM_SIZE = 32                       # docs/mvp-isa.md `### Engine state`
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_TXSTALL = 0x0100_0000         # bit 24 + engine (docs/mvp-regmap.md `**FDEBUG**`)
FDEBUG_BAD = 0x0003_0303             # TXOVER, RXUNDER, RXSTALL: nothing dropped or mis-timed
UIO_MASK = 0x000FF                   # the bidirectional protocol pins


@dataclass(frozen=True)
class Case:
    """One end-to-end configuration: a baud, an engine, a pin and a payload length."""
    name: str
    baud: float
    count: int = 8
    engine: int = 0
    pin: int = 0


CASES = [
    Case("115200", 115200.0, count=8),
    Case("921600", 921600.0, count=16),
    Case("6250000-max", 6_250_000.0, count=8, pin=5),      # the fastest CLKDIV = 1.0 allows
    Case("38400-engine1", 38400.0, count=6, engine=1, pin=7),
]


def payload(count: int, engine: int = 0) -> list[int]:
    """Bytes that exercise every bit pattern, then a fixed pseudo-random tail."""
    head = [0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0x7F, 0xFE]
    seed = 0x2545 + 7 * engine
    tail = []
    while len(head) + len(tail) < count:
        seed = (seed * 1103515245 + 12345) & 0xFFFF_FFFF
        tail.append((seed >> 16) & 0xFF)
    return (head + tail)[:count]


@dataclass
class Result:
    case: Case
    bus: Bus
    drv: UartTxDriver
    rx: UartRx
    rx_nominal: UartRx
    data: list[int]
    nominal_tpb: float
    actual_tpb: float


def build(case: Case) -> tuple[Bus, ChipDevice, UartTxDriver, UartRx, UartRx]:
    """A Bus at 20 ns/tick with p0..p17, the ISS on it, and two receivers on the TX pin."""
    bus = Bus(ns_per_tick=20.0)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == case.pin))
    dev = bus.attach(ChipDevice())
    drv = UartTxDriver(dev, bus, engine=case.engine, pin=case.pin, baud=case.baud)
    actual = ticks_per_bit(drv.clkdiv, drv.sm_cycles_per_bit)
    net = f"p{case.pin}"
    rx = bus.attach(UartRx(net, ticks_per_bit=actual, data_bits=8, parity="none",
                           stop_bits=1, latency=1))
    rx_nominal = bus.attach(UartRx(net, ticks_per_bit=CORE_HZ / case.baud,
                                   data_bits=8, parity="none", stop_bits=1, latency=1))
    return bus, dev, drv, rx, rx_nominal


def run_case(case: Case, data: list[int]) -> Result:
    """Load, start, send, and run long enough for the last frame to finish."""
    bus, dev, drv, rx, rx_nominal = build(case)
    drv.load()                              # IMEM + CLKDIV/EXECCTRL/SHIFTCTRL/PINCTRL
    drv.start(first=data[:drv.TXFIFO_DEPTH])
    drv.send(data[drv.TXFIFO_DEPTH:])
    tpb = drv.config.nominal_ticks_per_bit
    bus.run(int(len(data) * (drv.config.frame_bits + 2) * tpb) + 8000)
    return Result(case, bus, drv, rx, rx_nominal, list(data), tpb,
                  ticks_per_bit(drv.clkdiv, drv.sm_cycles_per_bit))


# --------------------------------------------------------------------------------------------
# The transaction, judged by the VIP
# --------------------------------------------------------------------------------------------

@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_uart_tx_sends_8n1(case: Case) -> None:
    """Every byte arrives, in order, framed 8N1, with the bit edges on the nominal grid."""
    data = payload(case.count, case.engine)
    res = run_case(case, data)
    rx = res.rx

    assert rx.values == data, f"{rx.values} != {data}"
    assert len(rx.received) == len(data)
    assert rx.framing_errors == 0, "a stop bit was low"
    assert rx.parity_errors == 0
    assert rx.aborted == 0, "the receiver sampled a start bit high: a glitch"

    # The pass criterion: every bit edge within one tick of the bit grid.
    assert rx.worst_dev_ticks < 1.0, (
        f"bit edges are up to {rx.worst_dev_ticks:.3f} ticks off the "
        f"{res.actual_tpb:.4f}-tick grid"
    )

    # ... and on the requested baud's grid, within what a 16.8 divider can express.
    rate_error = 10 * abs(res.actual_tpb - res.nominal_tpb)
    assert res.rx_nominal.worst_dev_ticks < 1.0 + rate_error, (
        f"{res.rx_nominal.worst_dev_ticks:.3f} ticks off the requested "
        f"{res.nominal_tpb:.4f}-tick grid, more than the {rate_error:.4f}-tick rate error "
        f"(achieved {res.actual_tpb:.4f} ticks per bit)"
    )

    # Frames do not overlap (>= 10 bit times) and the host keeps up (the engine waits at
    # most 3 bit times per byte for a FIFO word: it runs 82 SM cycles per byte, 10.25 bit
    # times of which are the frame itself, the rest the idle gap between stop and start).
    starts = [b.tick for b in rx.received]
    periods = [(b - a) / res.actual_tpb for a, b in zip(starts, starts[1:])]
    assert all(p >= 10.0 for p in periods), f"frame period too short: {periods}"
    assert all(p <= 13.0 for p in periods), f"the engine starved for a FIFO word: {periods}"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_uart_tx_leaves_the_chip_clean(case: Case) -> None:
    """No word dropped, no read underflow, only the TX pin driven, only this engine used."""
    data = payload(case.count, case.engine)
    res = run_case(case, data)
    drv, chip = res.drv, res.drv.dev.chip
    other = 1 - case.engine

    # TXOVER/RXUNDER/RXSTALL would mean a byte was dropped or mis-timed.  TXSTALL is the
    # engine parked on `pull block` waiting for a byte the host has not sent yet, which is
    # what a transmitter between messages does.
    fdebug = drv.fdebug()
    assert fdebug & FDEBUG_BAD == 0, f"FDEBUG {fdebug:#010x}"
    assert fdebug & ~(FDEBUG_TXSTALL << case.engine) == 0, f"unexpected FDEBUG {fdebug:#010x}"

    padoe = drv.padoe()
    assert padoe & (1 << case.pin), f"padoe {padoe:#06x}: the TX pin is not driven"
    assert padoe & UIO_MASK & ~(1 << case.pin) == 0, (
        f"padoe {padoe:#06x}: a program window leaked onto another uio pin"
    )
    assert drv.padout() & (1 << case.pin), "the TX pin is not high after the last stop bit"
    assert chip.state(case.engine)["tx_level"] == 0, "bytes are still queued"
    assert chip.state(other)["pc"] == 0, "the other engine ran"
    assert not chip.engines[other].tx, "the other engine got data"
    assert not res.bus.errors, f"contention on a push-pull net: {res.bus.errors[:2]}"
    assert drv.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"


@pytest.mark.parametrize("case", CASES, ids=[c.name for c in CASES])
def test_uart_tx_loaded_what_was_assembled(case: Case) -> None:
    """The words in the engine's IMEM are the ones `asm/pioasm.py` produced for program.pio."""
    data = payload(case.count, case.engine)
    res = run_case(case, data)
    imem = res.drv.dev.chip.engines[case.engine].imem
    assert imem[: res.drv.words_used] == res.drv.program.instrs
    assert set(imem[res.drv.words_used:]) <= {0}, "the driver wrote past the program"


# --------------------------------------------------------------------------------------------
# What the program and the divisor are, derived from the words rather than asserted by hand
# --------------------------------------------------------------------------------------------

def test_program_fits_the_imem() -> None:
    """program.pio assembles, and the driver reads its shape back out of the words."""
    program = load_program()
    assert program.name == "uart_tx"
    assert words_used(program) == 12, [f"{w:#06x}" for w in program.instrs]
    assert words_used(program) <= IMEM_SIZE
    assert program.labels["start"] == 0
    assert program.wrap == words_used(program) - 1, "WRAP_TOP must be the last instruction"
    assert program.sideset_bits == 2 and program.sideset_opt, ".side_set 1 opt is 2 bits"
    assert bit_cycles(program) == 8, "8 SM cycles per bit, i.e. one delay of 7"
    data_bits = [w for w in program.instrs if driver_mod.is_out_pins_1(program, w)]
    assert len(data_bits) == 8
    assert len(set(data_bits)) == 1, "every data bit is the same instruction"


def test_maximum_speed_on_the_iss() -> None:
    """The fastest the program can go: CLKDIV 1.0 and one SM cycle per instruction.

    A bit is 8 SM cycles, so CLKDIV = 1.0 puts a bit every 8 core cycles, and 50 MHz gives
    CORE_HZ / 8 = 6.25 MBd.  Nothing above that is reachable without changing the program,
    because the delay is counted in SM cycles.
    """
    program = load_program()
    per_bit = bit_cycles(program)
    fastest = ticks_per_bit(clkdiv_value(1), per_bit)
    assert fastest == per_bit == 8.0
    assert CORE_HZ / fastest == 6_250_000.0
    # Every SM cycle takes at least one core cycle, so this is the ceiling.
    assert clkdiv_for_baud(CORE_HZ / fastest, per_bit) == clkdiv_value(1)
    with pytest.raises(ValueError):
        clkdiv_for_baud(CORE_HZ / fastest * 1.01, per_bit)


@pytest.mark.parametrize("baud", [9600.0, 38400.0, 115200.0, 230400.0, 921600.0,
                                  2_000_000.0, 6_250_000.0])
def test_clkdiv_hits_the_baud_within_the_divider_resolution(baud: float) -> None:
    """CLKDIV for a baud is the 16.8 rounding, and CLKDIV is all the program has to go on."""
    program = load_program()
    per_bit = bit_cycles(program)
    clkdiv = clkdiv_for_baud(baud, per_bit)
    integer, frac = (clkdiv >> 16) & 0xFFFF, (clkdiv >> 8) & 0xFF
    assert 1 <= integer <= 0xFFFF and 0 <= frac <= 0xFF
    # One bit is `per_bit` SM cycles and one SM cycle is integer + frac/256 core cycles, so
    # the bit length can only move in steps of per_bit/256 ticks: half a step, rounded.
    step = per_bit / 256
    assert abs(ticks_per_bit(clkdiv, per_bit) - CORE_HZ / baud) <= step / 2 + 1e-9


def test_driver_rejects_setups_it_cannot_honour() -> None:
    """A TX pin PINDIRS cannot drive, an impossible baud and an overfull preloaded FIFO."""
    bus = Bus(ns_per_tick=20.0)
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())

    for pin in (8, 12, 13, 17, 18):        # input-only, output-only, out of range
        with pytest.raises(ValueError):
            UartTxDriver(dev, bus, pin=pin)
    with pytest.raises(ValueError):
        clkdiv_for_baud(0.0, 8)
    with pytest.raises(ValueError):
        clkdiv_for_baud(6_250_000.0 * 2, 8)

    drv = UartTxDriver(dev, bus, pin=0, baud=115200.0)
    drv.load()
    with pytest.raises(ValueError):
        drv.preload([1, 2, 3, 4, 5])
    with pytest.raises(ValueError):
        drv.push(256)


def test_bit_cycles_refuses_a_program_it_cannot_time() -> None:
    """The driver will not divide for a program whose data bits are not all alike."""
    program = load_program()
    uneven = copy.deepcopy(program)
    uneven.instrs[3] -= 1 << 8                 # one data bit with a shorter delay
    with pytest.raises(ValueError, match="different delays"):
        bit_cycles(uneven)
    short = copy.deepcopy(program)
    short.instrs.pop(5)                       # only seven data bits
    with pytest.raises(ValueError, match="data bits"):
        bit_cycles(short)


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: a broken program must fail the transaction
# --------------------------------------------------------------------------------------------

MUTATION_BAUD = 115200.0
MUTATION_DATA = (0x48, 0x6C, 0xAA)
MUTATIONS = [
    ("one data bit a cycle short", lambda p: p.instrs.__setitem__(5, p.instrs[5] - (1 << 8))),
    ("start-bit side-set dropped", lambda p: p.instrs.__setitem__(1, p.instrs[1] & ~0x1F00)),
    ("stop bit never driven high", lambda p: p.instrs.__setitem__(10, p.instrs[10] - 1)),
    ("stop bit left off", lambda p: p.instrs.__setitem__(10, 0x0000)),
    ("a data bit removed", lambda p: p.instrs.pop(5)),
    ("the byte loop never returns", lambda p: p.instrs.__setitem__(11, 0x000B)),
]


def _run_program(program, baud: float, data=MUTATION_DATA) -> UartRx:
    """Load a program object, send `data` on pin 0 of engine 0, and return the receiver."""
    bus = Bus(ns_per_tick=20.0)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == 0))
    dev = bus.attach(ChipDevice())
    drv = UartTxDriver(dev, bus, pin=0, baud=baud, program=program)
    rx = bus.attach(UartRx("p0", ticks_per_bit=ticks_per_bit(drv.clkdiv, 8), latency=1))
    drv.load()
    drv.start(first=list(data))
    bus.run(int(len(data) * 12 * CORE_HZ / baud) + 8000)
    return rx


@pytest.fixture(scope="module")
def clean_run() -> UartRx:
    """The unmutated program, run once for the mutation tests to be compared against."""
    return _run_program(load_program(), MUTATION_BAUD)


@pytest.mark.parametrize("name,mutate", MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_broken_program_fails_the_check(name: str, mutate, clean_run: UartRx) -> None:
    """Each mutation either garbles the bytes or throws the bit edges off by tens of ticks.

    `bit_cycles` is stubbed to the real answer so that the driver still computes a divisor:
    the question is whether the *transaction check* notices, not whether the driver refuses
    to start (that has its own test).
    """
    assert clean_run.worst_dev_ticks < 1.0 and not clean_run.framing_errors, \
        "the unmutated run must be clean, or this test proves nothing"

    program = copy.deepcopy(load_program())
    mutate(program)
    original = driver_mod.bit_cycles
    driver_mod.bit_cycles = lambda prog: 8
    try:
        rx = _run_program(program, MUTATION_BAUD)
    finally:
        driver_mod.bit_cycles = original

    noticed = (rx.values != clean_run.values or rx.framing_errors or rx.aborted
               or rx.worst_dev_ticks >= 1.0)
    assert noticed, (
        f"{name}: the check did not notice (values {rx.values}, framing {rx.framing_errors}, "
        f"worst {rx.worst_dev_ticks:.3f} ticks; clean run {clean_run.values} at "
        f"{clean_run.worst_dev_ticks:.3f} ticks)"
    )

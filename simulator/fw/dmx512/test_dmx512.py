"""End-to-end test of the MVP DMX512-A controller firmware (task B1-FW-dmx512).

`iss.pio_iss.ChipDevice` goes on a `verif.vip.core.Bus` at 20 ns per tick (50 MHz) with nets
p0..p17, the host driver loads the program and pushes slot values, and the pin-level models
from `verif/vip/dmx512.py` -- `DmxRx` and `DmxChecker` -- judge the wire.  Those two are the
independent judges and this test does not modify them: the frame values, the framing flags, the
measured bit rate and every violation come from the VIP, and the numbers this firmware derived
from its own instruction words are compared against them rather than trusted.

    make fw PROTO=dmx512 TARGET=iss      # runs

Wiring: `DmxRx` and `DmxChecker` are attached with `net="p{k}"`, the chip's own net for protocol
pin `k`.  That net is a push-pull net, as EIA-485 drives both states (see the `verif/vip/dmx512.py`
module docstring), so `bus.add_net(f"p{k}", push_pull=(k == case.pin))` is what makes two drivers
on the same link show up as contention instead of being quietly wired-ANDed.

What the tests below hold the firmware to, in the order they check it:

* **the transaction**: every slot arrives, in order, framed 8 data bits + 2 mark stop bits, with
  the START Code in slot 0 -- `test_controller_sends_a_legal_frame`, which asserts the checker
  reported *zero* violations, at the receiver's table and (strictly) at the transmitter's;
* **the numbers**: the break, the MAB, the bit period and the break-to-break time the checker
  measured off the edges are the ones `driver.py` computed from the words, so the derivation
  cannot rot -- `test_the_measured_frame_is_the_one_the_words_describe`;
* **the chip is left clean**: no word dropped (TXOVER), no read underflow (RXUNDER), no stall
  (TXSTALL, RXSTALL), only the DMX pin driven, the other engine untouched, no contention --
  `test_the_chip_is_left_clean`;
* **the program is what was assembled**: `test_program_fits_the_imem` and
  `test_the_engine_loaded_what_was_assembled`;
* **the driver refuses what it cannot honour**, and **the checker is not vacuous**: a starved
  FIFO, a short break, a short MAB, a stop bit driven low, a frame too short for the tables and a
  bit period off the grid each have to be caught -- `test_broken_run_fails_the_check`.

Nothing here looks inside the ISS to decide pass or fail: `Chip.state()` is used only for
after-the-fact bookkeeping (which PC the engine is parked at), never as the judgement.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, replace
from typing import Optional, Sequence

import pytest

import fw.dmx512.driver as driver_mod
from fw.dmx512.driver import (
    CORE_HZ,
    NS_PER_TICK,
    Config,
    Dmx512Driver,
    bit_cycles,
    bit_rate_limits,
    defines_of,
    frame_cycles,
    load_program,
    plan_for,
    select_bit_delay,
    slot_count,
    slots_limit,
    words_used,
)
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus
from verif.vip.dmx512 import (
    MAX_DATA_SLOTS,
    NULL_START_CODE,
    SLOT_BITS,
    DmxChecker,
    DmxRx,
    attach_checker,
    attach_rx,
    dmx_minimums,
)
from fw import regmap as _regmap

IMEM_SIZE = 32                       # docs/mvp-isa.md `### Engine state`
DBG_CFGINFO_MVP = _regmap.active().cfginfo   # 0x0020_0204 at the MVP: IMEM 32, SM_COUNT 2, FIFO_DEPTH 4
FDEBUG_TXSTALL = 0x0100_0000         # bit 24 + engine (docs/mvp-regmap.md `**FDEBUG**`)
FDEBUG_BAD = 0x0003_0303             # TXOVER, RXUNDER, RXSTALL: nothing dropped or mis-timed
UIO_MASK = 0x000FF                   # the bidirectional protocol pins
BREAK_TX = dmx_minimums(NS_PER_TICK, "transmit")["break"]        # 4600 ticks = 92 us
MAB_TX = dmx_minimums(NS_PER_TICK, "transmit")["mab"]            # 600 ticks = 12 us
B2B_TX = dmx_minimums(NS_PER_TICK, "transmit")["b2b_min"]        # 60200 ticks = 1204 us

# --------------------------------------------------------------------------------------------
# The bench and the cases
# --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Case:
    """One end-to-end configuration: a bit rate, a frame length, an engine, a pin, a payload."""

    name: str
    bit_rate: float
    slots: int = 25
    packets: int = 2
    engine: int = 0
    pin: int = 0
    strict_tx: bool = False
    profile: str = "transmit"

    @property
    def config(self) -> Config:
        return Config(bit_rate=self.bit_rate, slots=self.slots, engine=self.engine, pin=self.pin,
                      profile=self.profile)


#: The three bit rates the tables' window allows at 20 ns/tick, and two frame lengths, on two
#: engines and two pins: `bit_rate_limits()` says every whole-tick bit length from 196 to 204 is
#: inside the window, so the slow and fast ends of it are real configurations and not decoration.
CASES = [
    Case("250k-min-frame", 250_000.0, slots=25, packets=2),
    Case("250k-short-frame", 250_000.0, slots=4, packets=2),      # needs the pad loop
    Case("245k-slow-end", 245_000.0, slots=25, packets=2),
    Case("255k-fast-end", 255_000.0, slots=25, packets=2),
    Case("250k-strict-table", 250_000.0, slots=25, packets=2, strict_tx=True),
    Case("250k-engine1-pin5", 250_000.0, slots=25, packets=2, engine=1, pin=5),
    Case("250k-one-slot", 250_000.0, slots=1, packets=2),          # START Code only, most padding
    Case("250k-long-frame", 250_000.0, slots=31, packets=1),       # the program's longest frame
]
CASES_BY_NAME = {c.name: c for c in CASES}


def payload(count: int) -> list[int]:
    """Slot values that exercise every bit pattern, then a fixed pseudo-random tail.

    The first value is the START Code, so it is the NULL START Code of clause 8.5.1 whatever the
    frame length; the checker reads it back as `ObservedPacket.start_code`.
    """
    head = [NULL_START_CODE, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0x7F, 0xFE]
    seed = 0x2545
    tail = []
    while len(head) + len(tail) < count:
        seed = (seed * 1103515245 + 12345) & 0xFFFF_FFFF
        tail.append((seed >> 16) & 0xFF)
    return (head + tail)[:count]


@dataclass
class Bench:
    """A bus with the chip on it and both DMX512 models watching the DMX pin."""

    bus: Bus
    dev: ChipDevice
    drv: Dmx512Driver
    rx: DmxRx
    chk: DmxChecker
    case: Case
    values: list

    @property
    def plan(self):
        return self.drv.plan

    @property
    def rules(self) -> set:
        return {v.rule for v in self.chk.violations}


def build(case: Case, *, config: Optional[Config] = None, program=None,
          plan=None) -> Bench:
    """A Bus at 20 ns/tick with p0..p17, the ISS on it, and both models on the DMX pin.

    `DmxRx` is given the bit length the chip actually produces -- the grid a real receiver
    recovers from the line -- rather than the nominal one, so a receiver's sampling cannot be
    mistaken for a transmitter's timing.  `DmxChecker` is told nothing at all and measures the
    wire, which is the judgement this firmware has to pass.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == case.pin))
    dev = bus.attach(ChipDevice())
    net = f"p{case.pin}"
    drv = Dmx512Driver(dev, bus, config=config if config is not None else case.config)
    if plan is not None:                         # a hand-made plan: illegal on purpose
        drv.plan = plan
    elif program is not None:                    # a mutation: load these words instead
        drv.plan = replace(drv.plan, program=program)
    rx = attach_rx(bus, net=net, address=1, footprint=4, ticks_per_bit=drv.plan.tick_ticks)
    chk = attach_checker(bus, net=net, strict_tx=case.strict_tx)
    return Bench(bus, dev, drv, rx, chk, case, [])


def run_case(case: Case, *, config: Optional[Config] = None, program=None, plan=None,
             values: Optional[Sequence[int]] = None, packets: Optional[int] = None) -> Bench:
    """Load, start, send `packets` frames, and close the frames the models are holding.

    `DmxChecker` and `DmxRx` can only know a frame is over when the next BREAK arrives, so the
    last one has to be closed by hand; `finish()` is idempotent on both.
    """
    bench = build(case, config=config, program=program, plan=plan)
    bench.values = list(values) if values is not None else payload(bench.drv.slots)
    count = packets if packets is not None else case.packets
    bench.drv.send(bench.values, count)
    bench.chk.finish()
    bench.rx.finish()
    return bench


# --------------------------------------------------------------------------------------------
# The transaction, judged by the VIPs
# --------------------------------------------------------------------------------------------

@pytest.fixture(scope="module", params=CASES, ids=[c.name for c in CASES])
def bench(request) -> Bench:
    """One run per configuration, shared by the tests below.

    A frame is over only when the next BREAK arrives, so a run is not a thing that can be
    repeated per assertion for free: the whole transaction is done once here (load, start,
    `packets` frames, close) and every test below reads that run's verdicts.
    """
    case = request.param
    return run_case(case)


def test_controller_sends_a_legal_frame(bench: Bench) -> None:
    """Zero violations, and every slot arrives in order with a NULL START Code in slot 0.

    This is the bar: the checker, which measures the wire rather than believing the
    firmware, has nothing to say about the frames this firmware sent.
    """
    case, chk, rx = bench.case, bench.chk, bench.rx
    expected = bench.values[:bench.drv.slots]

    assert chk.violations == [], chk.report()
    assert len(chk.packets) == case.packets, f"{len(chk.packets)} frame(s), not {case.packets}"
    for observed in chk.packets:
        assert observed.start_code == NULL_START_CODE, (
            f"slot 0 is the START Code and must be {NULL_START_CODE:#04x} (clause 8.5.1), "
            f"got {observed.start_code:#04x}"
        )
        assert observed.slots == case.slots - 1, (
            f"{observed.slots} data slot(s), not {case.slots - 1}: one word per slot, one word "
            "for the START Code"
        )

    # The receiver is the second, independent judgement: it decodes at the centres of the bits
    # on the grid the chip produces and answers with the levels of its own address window.
    assert len(rx.packets) == case.packets
    for got in rx.packets:
        assert got.null_start, "the START Code was not a NULL START Code"
        assert got.framing_errors == 0, "a slot's stop bits were not both marks (clause 9.1)"
        assert list(got.values) == expected[1:], f"{list(got.values)} != {expected[1:]}"
    # the receiver's window is four slots wide whatever the frame length: the slots the frame
    # never carried keep the mark the receiver started with (there is no "unused slot" encoding
    # in the standard, so a receiver cannot tell them from a level of 0xff)
    window = rx.window
    sent = expected[1:1 + len(window)]
    assert window[:len(sent)] == bytes(sent), f"{window.hex()} != {bytes(sent).hex()}"
    assert window[len(sent):] == bytes(len(window) - len(sent)), (
        "and the slots a short frame did not reach are still the receiver's reset levels")
    assert rx.framing_errors == 0 and rx.mab_errors == 0
    assert not rx.short_breaks and not rx.double_breaks and not rx.glitches
    assert rx.over_512 == 0, "the frames stayed inside the 512-slot universe"
    assert rx.slot_count <= MAX_DATA_SLOTS


def test_the_measured_frame_is_the_one_the_words_describe(bench: Bench) -> None:
    """The checker's measurements equal the durations `driver.py` computed from the words.

    This is what keeps the derivation honest: if a word, a delay or the divider moved, the
    numbers below would no longer be the ones on the wire and the test would say so instead of
    the firmware quietly being wrong.
    """
    case, chk, plan = bench.case, bench.chk, bench.plan
    assert case.packets == 1 or len(chk.packets) > 1

    for observed in chk.packets:
        assert observed.break_ticks == plan.break_ticks, (
            f"break {observed.break_ticks} ticks, the words say {plan.break_ticks:.0f}"
        )
        assert observed.mab_ticks == plan.mab_ticks, (
            f"MAB {observed.mab_ticks} ticks, the words say {plan.mab_ticks:.0f}"
        )
        assert observed.bit_ticks == plan.tick_ticks, (
            f"bit period {observed.bit_ticks} ticks, the words say {plan.tick_ticks:.0f}"
        )
        assert observed.bit_rate == pytest.approx(plan.bit_rate, rel=1e-9)
        assert observed.break_ticks >= BREAK_TX, "table 6 designator 1"
        assert observed.mab_ticks >= MAB_TX, "table 6 designator 2"

    starts = [p.break_tick for p in chk.packets]
    for earlier, later in zip(starts, starts[1:]):
        assert later - earlier == plan.frame_ticks, (
            f"break-to-break {later - earlier} ticks, the words say {plan.frame_ticks:.0f} "
            "(table 6 designator 11)"
        )
    assert plan.frame_ticks >= B2B_TX
    assert plan.refresh_hz == pytest.approx(CORE_HZ / plan.frame_ticks)


def test_the_chip_is_left_clean(bench: Bench) -> None:
    """No word dropped, no read underflow, no stall, one pin driven, one engine used."""
    drv, chip, case = bench.drv, bench.drv.dev.chip, bench.case
    other = 1 - case.engine

    fdebug = drv.fdebug()
    assert fdebug & FDEBUG_BAD == 0, (
        f"FDEBUG {fdebug:#010x}: TXOVER/RXUNDER/RXSTALL mean a word was dropped or a read "
        "underflowed"
    )
    assert fdebug & ~(FDEBUG_TXSTALL << case.engine) == 0, (
        f"FDEBUG {fdebug:#010x}: even TXSTALL is not expected, because `send` fills the TX FIFO "
        "before the engine is enabled and tops it up while it runs"
    )
    assert drv.tx_level == 0, "every slot value was taken"
    assert chip.state(case.engine)["tx_level"] == 0

    padoe = drv.padoe()
    assert padoe & (1 << case.pin), f"padoe {padoe:#06x}: the DMX pin is not driven"
    assert padoe & UIO_MASK & ~(1 << case.pin) == 0, (
        f"padoe {padoe:#06x}: a program window leaked onto another uio pin"
    )
    # bits 17:13 are the output-only pins, always driven, and bits 12:8 the input-only ones,
    # never driven: so the rest of padoe is the chip's own reset value
    # (docs/mvp-regmap.md `**DBG_PADOE**`).  The DMX pin is the one the program drives.
    assert padoe & ~(1 << case.pin) & 0x00FF_FFE0 == 0x3E000, f"padoe {padoe:#06x}"
    assert drv.padout() & (1 << case.pin), "the DMX pin is not at mark after the last stop bit"
    assert chip.state(other)["pc"] == 0, "the other engine ran"
    assert not chip.engines[other].tx and not chip.engines[other].rx
    assert bench.bus.errors == [], f"contention on the link: {bench.bus.errors[:2]}"
    assert drv.cfginfo() == DBG_CFGINFO_MVP, "the chip is not an MVP PIO block"


def test_the_engine_loaded_what_was_assembled(bench: Bench) -> None:
    """The words in the engine's IMEM are the ones `asm/pioasm.py` produced for program.pio."""
    drv, case = bench.drv, bench.case
    imem = drv.dev.chip.engines[case.engine].imem
    assert list(imem[:drv.words_used]) == list(drv.program.instrs)
    assert set(imem[drv.words_used:]) <= {0}, "the driver wrote past the program"
    assert drv.dev.chip.engines[case.engine].clkdiv == drv.clkdiv
    assert drv.dev.chip.engines[case.engine].pinctrl == drv.pinctrl
    assert drv.dev.chip.engines[case.engine].shiftctrl == drv.shiftctrl
    assert drv.dev.chip.engines[case.engine].execctrl == drv.execctrl


# --------------------------------------------------------------------------------------------
# What the program and the divider are, read back out of the words
# --------------------------------------------------------------------------------------------

def test_program_fits_the_imem() -> None:
    """program.pio assembles, and the driver reads its own shape back out of the words."""
    program = load_program()
    assert program.name == "dmx512"
    assert words_used(program) == 28, [f"{w:#06x}" for w in program.instrs]
    assert words_used(program) <= IMEM_SIZE
    assert program.sideset_bits == 0 and not program.sideset_opt, (
        "no side-set, so the whole 5-bit delay field is delay: one instruction is up to 32 SM "
        "cycles long, which is what a 200-tick bit needs"
    )
    assert program.labels == {"start": 0, "packet": 3, "brk": 4, "slot": 10, "pad_r": 23,
                              "pad_i": 24, "pad_o": 26}
    assert program.wrap == words_used(program) - 1, "WRAP_TOP is the last instruction"
    assert bit_cycles(program) == 25, "25 SM cycles per bit, i.e. a delay of 24"
    assert slot_count(program) == 25, "the file's own SLOTS: a START Code and 24 data slots"
    # the frame's shape, in words: 1 pull, 2 pin setup, 1 counter, 2 break, 3 MAB, 1 counter,
    # 12 per slot pass, 2 pad counters, 2 pad passes, 1 jump back
    assert defines_of() == {"BIT_DELAY": 24, "BREAK_ITERS": 18, "SLOTS": 25,
                            "PAD_OUTER": 0, "PAD_INNER": 0}
    sided = copy.deepcopy(program)
    sided.sideset_bits = 1
    with pytest.raises(ValueError, match="side-set"):
        driver_mod.bit_cycles(sided)


def test_the_whole_frame_is_derived_from_the_words() -> None:
    """`frame_cycles` is the sum of the loops the program actually contains.

    Read it as an accounting identity rather than a magic number: the break loop, the MAB, one
    12-instruction pass per slot, the two counter reloads, the pad and the jump back to
    `packet`.  The 25 % 3 remaining SM cycles are the leading `pull block` and the three words
    that come before `packet`, none of which are inside the frame-to-frame path.
    """
    program = load_program()
    total = frame_cycles(program, 0, 0)
    accounted = (driver_mod.break_cycles(program) + driver_mod.mab_cycles(program)
                 + slot_count(program) * driver_mod.slot_cycles(program)
                 + 1 + 1 + driver_mod.pad_cycles(program, 0, 0) + 1)
    assert accounted == total, f"{accounted} != {total}"
    assert driver_mod.pad_cycles(program, 0, 0) == 35, "one pass of an unpadded pad loop"
    assert driver_mod.pad_cycles(program, 31, 6) == 7406, "(31 + 1) inner passes, 7 times"
    assert driver_mod.loop_passes(0) == 1 and driver_mod.loop_passes(24) == 25


def test_the_expressible_bit_rates_cover_the_tables_window() -> None:
    """Every bit length inside the tables' window that this clock can put on the wire.

    The question is the fastest the ISS can do.  Two answers, and they are different things:
    the *fastest DMX512-legal* bit is the shortest whole-tick bit length the tables still allow,
    and the fastest the program could express at all is one SM cycle per instruction with
    CLKDIV = 1, which is 50 times outside the window.
    """
    limits = bit_rate_limits()
    assert limits["ns_per_tick"] == NS_PER_TICK
    assert (limits["bit_rate_min"], limits["bit_rate"], limits["bit_rate_max"]) == (
        245e3, 250e3, 255e3)
    # 245 .. 255 kbit/s at 20 ns/tick is 196.08 .. 204.08 ticks, and the checker's own window is
    # half a tick wider either way (verif/vip/dmx512.py `DmxChecker._bit_in_window`).
    assert limits["min_bit_ticks"] == pytest.approx(196.08, rel=1e-3)
    assert limits["max_bit_ticks"] == pytest.approx(204.08, rel=1e-3)
    expressible = limits["expressible_bit_ticks"]
    assert expressible == list(range(196, 205)), expressible
    # so the program can say every whole-tick bit length the tables allow, at both ends:
    assert limits["fastest_bit_ticks"] == 196
    assert limits["slowest_bit_ticks"] == 204
    # In rate terms, the checker's window is these two bounds, and at 20 ns/tick one tick of bit
    # length is 2.5 kbit/s of rate -- which is all the resolution there is to judge it with.
    low_rate = 1.0e9 / (limits["checker_max_bit_ticks"] * NS_PER_TICK)
    high_rate = 1.0e9 / (limits["checker_min_bit_ticks"] * NS_PER_TICK)
    assert (low_rate, high_rate) == pytest.approx((244_401.2, 255_651.9), rel=1e-4)
    for ticks in expressible:
        delay, integer = select_bit_delay(float(ticks))
        assert (delay + 1) * integer == ticks, f"{ticks} ticks is not (1 + delay) * INT"
        assert 0 <= delay <= 31 and 1 <= integer <= 0xFFFF
        assert low_rate <= CORE_HZ / ticks <= high_rate, (
            f"{ticks} ticks is {CORE_HZ / ticks:.1f} bit/s, outside the checker's window")
    # the fastest DMX512-legal frame this chip can send, at each end of the window
    assert CORE_HZ / limits["fastest_bit_ticks"] == pytest.approx(255_102.04, rel=1e-6)
    assert CORE_HZ / limits["slowest_bit_ticks"] == pytest.approx(245_098.04, rel=1e-6)
    # and the two ceilings that are not DMX512 at all, so the numbers above are not mistaken for
    # the limit of the hardware: one instruction per bit with CLKDIV 1.0 is one core cycle per
    # bit, 50 Mbit/s, and the same program with the file's own 25 SM cycles per bit is 2 Mbit/s.
    assert CORE_HZ == 50_000_000
    assert CORE_HZ * 1 == 50_000_000 and CORE_HZ / 1 == 50_000_000
    assert CORE_HZ / bit_cycles(load_program()) == 2_000_000.0
    assert 50_000_000 / 250_000.0 == 200.0, "the ceiling is 200 times the nominal DMX512 rate"


@pytest.mark.parametrize("rate", [245e3, 247e3, 250e3, 252e3, 255e3])
def test_every_rate_in_the_window_produces_a_legal_frame(rate: float) -> None:
    """A configuration the tables allow, from the requested bit rate down to the words."""
    plan = plan_for(Config(bit_rate=rate))
    limits = plan.limits()
    assert plan.tick_ticks == round(1e9 / rate / NS_PER_TICK)
    assert limits["min_bit_ticks"] - 0.5 <= plan.tick_ticks <= limits["max_bit_ticks"] + 0.5
    assert plan.break_ticks >= limits["break"], "table 6 designator 1"
    assert plan.mab_ticks >= limits["mab"], "table 6 designator 2"
    assert limits["b2b_min"] <= plan.frame_ticks <= limits["b2b_max"], "table 6 designators 11, 13"
    assert plan.words <= IMEM_SIZE
    # the divider is an integer, so the bit period is exact: no fractional rounding to allow for
    assert plan.clkdiv & 0xFF00 == 0, "FRAC is 0: a bit is a whole number of core cycles"
    assert plan.tick_ticks == (1 + plan.bit_delay) * plan.divider_int


# --------------------------------------------------------------------------------------------
# The driver refuses what it cannot honour
# --------------------------------------------------------------------------------------------

def test_driver_rejects_setups_it_cannot_honour() -> None:
    """A pin PINDIRS cannot drive, a frame the 5-bit counter cannot hold, an illegal bit rate."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(18):
        bus.add_net(f"p{k}")
    dev = bus.attach(ChipDevice())

    for pin in (8, 12, 13, 17, 18):        # input-only, output-only, out of range
        with pytest.raises(ValueError, match="uio pin"):
            Dmx512Driver(dev, bus, pin=pin)
    for engine in (2, -1):
        with pytest.raises(ValueError, match="engine"):
            Dmx512Driver(dev, bus, engine=engine)
    for slots in (0, -1, slots_limit() + 1, 512):
        with pytest.raises(ValueError, match="slots"):
            Dmx512Driver(dev, bus, slots=slots)
    for rate in (0.0, -250e3, 244_999.0, 255_001.0, 1e6):
        with pytest.raises(ValueError, match="kbit/s"):
            Dmx512Driver(dev, bus, bit_rate=rate)
    with pytest.raises(ValueError, match="pad"):
        Dmx512Driver(dev, bus, pad=(32, 0))
    # a bit length that is not a product of two integers: the tables' window has no such value
    # at 20 ns/tick, but `select_bit_delay` is what would refuse one
    with pytest.raises(ValueError, match="whole number"):
        driver_mod.select_bit_delay(199.5)
    with pytest.raises(ValueError, match="at least one tick"):
        driver_mod.select_bit_delay(0.0)
    with pytest.raises(ValueError, match="no `.define"):
        driver_mod.set_define(".program x\n", "BIT_DELAY", 1)
    with pytest.raises(ValueError, match="no label"):
        driver_mod.word_at(load_program(), "no_such_label")

    drv = Dmx512Driver(dev, bus, pin=0, slots=4)
    drv.load()
    with pytest.raises(ValueError, match="do not fit"):
        drv.preload([1, 2, 3, 4, 5])
    with pytest.raises(ValueError, match="0..255"):
        drv.push(256)
    with pytest.raises(ValueError, match="cannot be empty"):
        drv.feed([], packets=1)
    with pytest.raises(ValueError, match="no frame to send"):
        drv.feed([0x00], packets=0)
    with pytest.raises(ValueError, match="preloaded"):
        drv.feed([0x00], packets=1, preloaded=9)


def test_plan_for_refuses_a_frame_the_tables_forbid() -> None:
    """A frame the pad loops cannot stretch to 1204 us is refused before the engine is loaded.

    `pad=(0, 0)` with a short frame is exactly the run `test_broken_run_fails_the_check` puts on
    the wire on purpose, so the driver has to accept it when asked explicitly and refuse it by
    default.  The check that refuses it is the derived one: the frame time out of the words.
    """
    short = Config(bit_rate=250e3, slots=4, pad=(0, 0))
    with pytest.raises(ValueError, match="break-to-break"):
        plan_for(short)
    padded = plan_for(Config(bit_rate=250e3, slots=4))
    assert padded.pad != (0, 0) and padded.frame_ticks >= B2B_TX
    # a pad longer than the two 5-bit counters can hold: 31 outer passes of 32 inner ones is
    # 1058 * 31 SM cycles, which at the slowest legal bit rate is 262 000 ticks of mark, so no
    # legal frame can ask for more -- the branch is there for a frame that could
    with pytest.raises(ValueError, match="more than the pad loops can hold"):
        driver_mod._pad_for(10_000_000.0, load_program(), 8)
    assert driver_mod._pad_for(0.0, load_program(), 8) == (0, 0)
    # and the automatic answer really is the smallest padding that meets the minimum: one fewer
    # outer pass would not, for every frame length and every bit rate the tables allow
    for slots in (1, 2, 3, 4, 8, 16, 24, 25, 31):
        for rate in (245e3, 250e3, 255e3):
            plan = plan_for(Config(bit_rate=rate, slots=slots))
            outer, inner = plan.pad
            per_pass = ((driver_mod.pad_cycles(plan.program, inner, 1)
                         - driver_mod.pad_cycles(plan.program, inner, 0)) * plan.divider_int)
            assert plan.frame_ticks >= B2B_TX, f"slots={slots} rate={rate}"
            assert outer == 0 or plan.frame_ticks - per_pass < B2B_TX, (
                f"slots={slots} rate={rate}: pad {plan.pad} is more than one pass short")


# --------------------------------------------------------------------------------------------
# The judge is not vacuous: a broken run must fail
# --------------------------------------------------------------------------------------------

MUTATION_CASE = CASES_BY_NAME["250k-min-frame"]


# ---- the mutations, and the rule each one has to provoke ----

def _one_cycle(word: int) -> int:
    """The same instruction with its delay cleared: one SM cycle, not a bit time."""
    return word & ~(0x1F << 8)


def _one_cycle_shorter(word: int) -> int:
    """The same instruction one SM cycle shorter, which at CLKDIV 8 is 8 ticks."""
    return word - (1 << 8)


def _drive_space(word: int) -> int:
    """`set pins, 1` -> `set pins, 0`: the same length, the other level."""
    return word & ~0x1F


def _drive_mark(word: int) -> int:
    """`set pins, 0` -> `set pins, 1`: the same length, the other level."""
    return word | 0x1F


def _patch(program, label: str, offset: int, change) -> None:
    index = program.labels[label] + offset
    program.instrs[index] = change(program.instrs[index])


@pytest.fixture(scope="module")
def clean_run() -> Bench:
    """The unmutated min-frame case, run once, so the mutation tests have something to compare."""
    bench = run_case(replace(MUTATION_CASE, packets=2))
    assert bench.chk.violations == [], f"the unmutated run must be clean: {bench.chk.report()}"
    return bench


# Each mutation patches the *assembled* words, so the driver still builds its registers and its
# timings from an unmutated program and the only thing that differs in the run is the word named
# here.  The offsets are counted from the labels the driver itself derives its timing from: the
# start bit at `slot`, the first data bit one word on, the two stop bits at SLOT_BITS - 2 and
# SLOT_BITS - 1, and the loop's jump at SLOT_BITS.
MUTATIONS = [
    ("a break of 288 ticks instead of 4752",
     lambda p: _patch(p, "brk", 0, _one_cycle), {"BREAK"}),
    ("a break of mark instead of space",
     lambda p: _patch(p, "brk", 0, _drive_mark), {"BREAK"}),
    ("no mark between the break and the START Code",
     lambda p: [_patch(p, "brk", off, _one_cycle) for off in (2, 3, 4)], {"MAB"}),
    ("the first stop bit driven low",
     lambda p: _patch(p, "slot", SLOT_BITS - 2, _drive_space), {"STOP1"}),
    ("the second stop bit driven low",
     lambda p: _patch(p, "slot", SLOT_BITS - 1, _drive_space), {"STOP2"}),
    ("one data bit 8 ticks short",
     lambda p: _patch(p, "slot", 1, _one_cycle_shorter), {"BIT-JITTER"}),
    ("the start bit a whole bit time long",
     lambda p: _patch(p, "slot", 0, lambda w: w + (1 << 8)), {"BIT-JITTER"}),
]


@pytest.mark.parametrize("name,apply,rules", MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_a_broken_program_fails_the_check(name: str, apply, rules: set,
                                         clean_run: Bench) -> None:
    """One word changed in program.pio, and the checker says which rule the wire broke.

    Without this, "zero violations" would only mean the checker is quiet.
    """
    program = copy.deepcopy(load_program())
    apply(program)
    bench = run_case(replace(MUTATION_CASE, packets=2), program=program)

    assert bench.rules & rules, (
        f"{name}: expected one of {sorted(rules)}, the checker reported {sorted(bench.rules)}"
        f"\n{bench.chk.report()}"
    )
    for violation in bench.chk.violations:
        assert violation.tick >= 0, violation


def test_a_transmitter_that_never_breaks_is_caught_by_the_receiver(clean_run: Bench) -> None:
    """Drive mark where the break's space is, and there is no packet at all.

    With the break loop driving mark, the only space left on the wire is a slot's own start bit
    and whatever zero data bits follow it: 1800 ticks for a START Code of 0x00, where table 7
    asks a receiver to recognise 4400.  So the checker does flag it -- as BREAK, a space too short
    to be one where a packet had to begin -- and no packet is ever opened.

    Clause 9.2 is the other half: a receiver that sees no reset sequence within one maximum
    break-to-break time has lost its data input, and `DmxRx.data_lost` is what says so.  Two
    models, two different questions about the same wire.
    """
    program = copy.deepcopy(load_program())
    _patch(program, "brk", 0, _drive_mark)
    bench = run_case(replace(MUTATION_CASE, packets=2), program=program)

    assert bench.rules == {"BREAK"}, bench.chk.report()
    assert bench.chk.packets == [] and bench.rx.breaks == 0, "no frame ever began"
    assert bench.rx.data_lost(bench.bus.t), "clause 9.2: the receiver has lost its data input"
    assert bench.rx.slot_count == 0
    assert clean_run.rx.breaks == clean_run.case.packets and not clean_run.rx.data_lost(
        clean_run.bus.t)


def unpadded_plan(config: Optional[Config] = None):
    """A plan for a frame with the mark-before-break padding switched off.

    `plan_for` refuses that combination on purpose -- it is a frame the tables forbid -- so the
    test that puts it on the wire has to ask for it the other way round: take a legal plan and
    replace its pad and its words, which is exactly what a firmware author would get by editing
    the two `.define`s in program.pio and looking away from the driver's checks.
    """
    config = config or Config(bit_rate=250_000.0, slots=4)
    legal = plan_for(config)
    program = driver_mod.assemble_for(
        bit_delay=legal.bit_delay, break_iters=legal.break_iters, slots=config.slots,
        pad_outer=0, pad_inner=0,
    )
    return replace(legal, program=program, pad=(0, 0))


def test_the_pad_loop_is_load_bearing(clean_run: Bench) -> None:
    """The same short frame with the padding switched off is too short for table 6.

    `pad=(0, 0)` is what the driver asks for when the frame is long enough not to need it, and
    it is the only way to say "do not pad".  A 4-slot frame without it is 16 000 ticks of
    break-to-break where the tables ask 60 200, and that is the one rule the pad loop exists to
    satisfy -- so this is the test that says the loop is not decoration.
    """
    unpadded = run_case(replace(MUTATION_CASE, packets=2), plan=unpadded_plan())
    assert unpadded.plan.pad == (0, 0)
    assert unpadded.plan.frame_ticks < B2B_TX
    assert "BREAK-TO-BREAK" in unpadded.rules, unpadded.chk.report()
    short = unpadded.chk.violations_by_rule("BREAK-TO-BREAK")[0]
    # the checker was told the receiver's table (table 7), so that is the limit it names; the
    # firmware aims at the transmitter's (table 6, 1204 us), which is 8 us higher
    assert short.required_ns == unpadded.chk.min["b2b_min"] * NS_PER_TICK
    assert short.required_ns == 1_196_000.0 and B2B_TX * NS_PER_TICK == 1_204_000.0
    assert short.measured_ns == unpadded.plan.frame_ticks * NS_PER_TICK
    assert clean_run.plan.frame_ticks >= B2B_TX and clean_run.rules == set()


def _run_until_pc(bench: Bench, address: int, times: int = 1, limit: int = 400_000) -> int:
    """Run the bus until the engine's PC has read `address` `times` times; return that tick.

    The count matters: the slot loop passes through the first `out pins, 1` once per slot, and
    the engine only *parks* there on the slot after the last FIFO word it was given (one word is
    taken by the program's `pull block`, one by each slot's refill).  So with three words pushed
    the third sighting is the stall.

    Polling `SMe_ADDR` is a host access, so a sighting is up to one access (two ticks) late --
    which is exactly what the test below needs: it pushes the missing word at that sighting, and
    the number of ticks late is what puts the edge off the grid.
    """
    bus, drv = bench.bus, bench.drv
    start, seen = bus.t, 0
    while bus.t - start < limit:
        if drv.pc() == address:
            seen += 1
            if seen == times:
                return bus.t
        bus.step()
    raise AssertionError(
        f"the engine reached {address} {seen} time(s), not {times}, within {limit} ticks")


def _run_until_parked(bench: Bench, address: int, limit: int = 400_000) -> int:
    """Run the bus until the engine's PC is `address` and is still there a whole instruction
    later; return the tick.  A stalled instruction does not advance the PC
    (`docs/mvp-isa.md` `### Stalls, delay and side-set, summarised`), so a PC that survives that
    long is parked rather than passing through."""
    bus, drv = bench.bus, bench.drv
    longer_than_an_instruction = int(3 * drv.plan.tick_ticks)
    start = bus.t
    while bus.t - start < limit:
        if drv.pc() == address:
            bus.run(longer_than_an_instruction)
            if drv.pc() == address:        # still there a whole instruction later: parked
                return bus.t
        bus.step()
    raise AssertionError(f"the engine never parked at {address} within {limit} ticks")


def test_a_late_fifo_word_puts_an_edge_off_the_grid(clean_run: Bench) -> None:
    """One FIFO word arriving a few ticks late is a bit the tables do not allow.

    Where an empty FIFO is *discovered* is worth being precise about, because it is not the
    eighth `out pins, 1` of the slot before: the background autopull runs between instructions,
    so when the eighth one has drained the OSR the very next SM cycle tries to refill it, and if
    the FIFO is empty the *first* `out pins, 1` of the next slot is the instruction that stalls.
    That is `slot + 1`, and the edge it was going to write -- data bit 0's -- then lands late.

    This test pushes the missing word at the first sighting of that PC, so the edge is a couple
    of ticks off 200: too far for the tables' 3.92 .. 4.08 us window and for the checker's two
    ticks of jitter, and reported as BIT-RATE.  That is what the host-feeding in `feed` is for:
    the FIFO is four words deep in the middle of a 250 kbit/s stream, and a refill cannot afford
    to be late.
    """
    case = replace(MUTATION_CASE, name="late", packets=1)
    bench = build(case)
    drv, chk = bench.drv, bench.chk
    refill = drv.program.labels["slot"] + 1         # where an empty FIFO is discovered: see above
    values = payload(drv.slots)
    drv.load()
    words = drv.TXFIFO_DEPTH - 1
    drv.start(first=values[:words])                         # three words, and the fourth late
    net = f"p{case.pin}"
    while bench.bus.values[net]:                           # the pin goes low: the BREAK has begun
        bench.bus.step()
    fell = bench.bus.t - 1            # `values` is the snapshot for the next tick, so the
                                      # space-to-mark... mark-to-space happened on the tick before
    plan = drv.plan
    # the slot whose refill finds the FIFO empty is `words` slots in (one word is taken by the
    # program's `pull block`, one by each slot's refill), and the instruction that stalls is its
    # first `out pins, 1`, one bit time into that slot
    park = fell + int(plan.break_ticks) + int(plan.mab_ticks) \
        + words * int(plan.slot_ticks) + int(plan.tick_ticks)
    bench.bus.run(park + 8 - bench.bus.t)                  # one SM cycle past the stall
    assert drv.pc() == refill, f"the engine should be parked on {refill}, it is on {drv.pc()}"
    assert drv.fdebug() & FDEBUG_TXSTALL, "the engine is stalled on the empty TX FIFO"
    drv.push(values[words])                                 # the word it was waiting for
    bench.bus.run(int(drv.plan.frame_ticks) + 4000)
    chk.finish()

    off_grid = [v for v in chk.violations if v.rule in ("BIT-RATE", "BIT-JITTER")]
    assert off_grid, (
        f"the late refill should have moved an edge off the grid: {chk.report()}")
    assert off_grid[0].rule == "BIT-RATE", (
        f"a couple of ticks off 200 is off the bit-rate window, not jitter: {chk.report()}")
    assert off_grid[0].measured_ns > 4_080.0, off_grid[0]
    assert chk.packets[0].slots >= 1, "and the frame it went on to send was still measured"
    assert chk.packets[0].start_code == NULL_START_CODE, "the slot values themselves were fine"
    assert clean_run.chk.violations == [], "the run that was fed on time is the clean one"


def test_a_starved_fifo_truncates_the_frame(clean_run: Bench) -> None:
    """Stop feeding the engine altogether and the frame simply stops part way through.

    Worth separating from the late word above, because a stalled `out pins, 1` writes *nothing*:
    the engine is parked on the instruction and the pin keeps the level the last completed
    instruction gave it.  So the wire has no edge to be late -- the checker, which measures
    edges, has nothing to say, and correctly says nothing.  What says so are FDEBUG.TXSTALL, the
    number of slots the frame carries, and the receiver.
    """
    case = replace(MUTATION_CASE, name="starved", packets=1)
    bench = build(case)
    drv, chk, rx = bench.drv, bench.chk, bench.rx
    refill = drv.program.labels["slot"] + 1
    drv.load()
    drv.start(first=[0x00] * drv.TXFIFO_DEPTH)
    _run_until_parked(bench, refill)
    bench.bus.run(2000)
    chk.finish()
    rx.finish()

    assert drv.fdebug() & FDEBUG_TXSTALL, (
        f"FDEBUG {drv.fdebug():#010x}: the engine is parked on an empty TX FIFO, and TXSTALL is "
        "the register that says so")
    assert chk.violations == [], (
        f"a transmitter that stops sending edges breaks no timing rule: {chk.report()}")
    assert len(chk.packets) == 1 and chk.packets[0].break_ticks == drv.plan.break_ticks, (
        "the break and the MAB it did send were legal")
    assert chk.packets[0].slots < drv.slots - 1, (
        f"{chk.packets[0].slots} data slot(s) of the {drv.slots - 1} that were asked for")
    assert rx.packets and rx.packets[0].slots == chk.packets[0].slots
    assert rx.window[:4] == bytes(4), "and the receiver's window is the mark it defaulted to"
    assert clean_run.rx.packets[0].slots == clean_run.drv.slots - 1


def test_the_slot_values_have_to_be_right_justified(clean_run: Bench) -> None:
    """`OUT_SHIFTDIR` decides which end of the 32-bit FIFO word is the first bit on the wire.

    With `OUT_SHIFTDIR = 1` (the reset value, what this firmware uses) `out pins, 1` sends bit 0
    of the OSR first, which is the order a DMX slot is in, and the host pushes each slot value
    right-justified in bits 7:0.

    `OUT_SHIFTDIR = 0` cannot produce a DMX slot at all, and that is worth spelling out because the
    register is a host choice.  The OSR then shifts left, so a slot's 32 shifts carry the word's
    *last* eight bits last -- on the wire first.  Left-justify the byte in bits 31:24, as
    `docs/mvp-isa.md` `## Differences from RP2040` item 11 says of the host port, and the byte
    arrives most significant bit first: every slot is the right bits in the wrong order.  Leave it
    right-justified, as this driver does, and the byte's bits all fall in the 24 leading shifts:
    a frame of 0x00 slots.  Neither breaks a rule of the standard -- the checker is silent on both
    -- which is exactly why the framing rules and the payload have to be checked separately.
    """
    case = replace(MUTATION_CASE, packets=1)
    values = payload(25)

    def run(out_right: int, justify) -> list:
        bench = build(case, config=replace(case.config, out_right=out_right))
        drv, rx, chk = bench.drv, bench.rx, bench.chk
        drv.send(values, 1, justify=justify)
        chk.finish()
        rx.finish()
        assert chk.violations == [], f"the shift direction breaks no timing rule: {chk.report()}"
        assert len(rx.packets) == 1 and rx.framing_errors == 0
        return list(rx.packets[0].values)

    right = run(1, lambda v: v)
    assert right == values[1:25], f"right-justified, shift right: {right}"
    reversed_in_slot = [int(f"{v:08b}"[::-1], 2) for v in values[1:25]]
    assert run(0, lambda v: v << 24) == reversed_in_slot, (
        "left-justified with the shift the other way, each byte arrives most significant bit "
        "first, which is not the order a slot is in either")
    assert run(0, lambda v: v) == [0x00] * 24, (
        "and right-justified words with the shift the other way are a frame of zeros: the byte "
        "ends up in the last eight of the 32 shifts")
    assert clean_run.rx.window != bytes(24), "so the wrong one really is different"

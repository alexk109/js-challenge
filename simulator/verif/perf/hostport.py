"""What a register access costs over the SPI host port, measured on the wire.

`docs/mvp-regmap.md` `## SPI transaction format` fixes the framing, and the framing is what the
host spends core cycles on: a write is `CMD | W0` (five bytes), a read is `CMD | TA | D` (six
bytes for the first word), the address auto-increments after a word, and the FIFO data
registers are the exception where it does not -- so one `CMD` plus `TA`s streams a whole FIFO.
The cost is reported at `SCK = core / 8`, which is 8 core cycles per SCK bit and the
slowest rate the port still guarantees reads at (`### Physical layer`: MISO changes at most
3 core cycles after an SCK rise, so reads are guaranteed from an 8-cycle period up).

So the measurement is: drive `verif.vip.spi.SpiController` -- the same host model the SPI
firmware bench uses, unmodified -- one transaction at a time, and read the cost off the wire
with `verif.perf.probe`: the CSn fall to the CSn rise is what the host paid, and the SCK rises
inside it are the bits the port really clocked.  The four host timing phases of `### Physical
layer` (CSn setup, CSn hold, the shortest SCK low and high phase) are measured the same way,
each as a difference between two cycles in the edge log, rather than inferred from the length
of the window.  Nothing here is `bytes * 8 * ticks_per_bit` written down as a formula;
`check_framing()` then *compares* the measured wire against the framing and the minimums the
regmap specifies, so a disagreement is a failure rather than a coincidence.

This number is a property of the host port, not of a protocol, so `verif/perf/measure.py`
writes the same measured value into all 14 protocols' rows and says so in the source column.
What this module does *not* measure is the cycle inside the chip that an access lands in --
`h` and the 3-cycle synchronizer are `test/host`'s and `src/host_spi.sv`'s subject, and
`test/host/test_host_spi.py` is where `h` is checked against the RTL.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from verif.perf.probe import Probe, attach
from verif.vip.core import Bus
from verif.vip.spi import CS_N, MISO, MOSI, SCK, SpiController

#: `SCK = core / 8`: 8 core cycles per SCK bit at the 50 MHz core clock
#: (docs/mvp-regmap.md `### Physical layer`), which is `half_period = 4`.
TICKS_PER_BIT = 8
NS_PER_TICK = 20.0
F_CLK_HZ = 1.0e9 / NS_PER_TICK

#: Word addresses, docs/mvp-regmap.md `### Address map`.
A_CTRL, A_FSTAT, A_FDEBUG, A_FLEVEL = 0x00, 0x01, 0x02, 0x03
A_TXF0, A_RXF0 = 0x04, 0x08

CMD_WRITE = 0x00                       # { RW, ADDR[6:0] }, RW = 0 write / 1 read
CMD_READ = 0x80
TA = 0x00                              # one turnaround byte, MOSI ignored


def _word(data: int) -> list[int]:
    """A 32-bit word as four bytes on the wire, MSB first (`### Words`)."""
    return [(data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF]


@dataclass
class AccessCost:
    """One access shape: what the host spent on the wire for it."""

    name: str
    ticks_per_bit: int
    bytes_on_the_wire: int
    cycles: int                # CSn fall to CSn rise, in core cycles
    sck_rises: int             # bits really clocked, counted off the wire
    mosi_bytes: int
    miso_bytes: int
    #: The host timing minimums of `### Physical layer`, each *measured* off the edge log
    #: rather than assumed from the window length: CSn fall to the first SCK rise, the last SCK
    #: rise to the CSn rise, and the shortest low and high phase of any SCK.  `check_framing`
    #: compares each with the two core cycles the regmap asks for.
    setup: int = 0             # CSn fall -> first SCK rise
    hold: int = 0              # last SCK rise -> CSn rise
    min_low: int = 0           # shortest SCK low phase
    min_high: int = 0          # shortest SCK high phase
    detail: dict = field(default_factory=dict)

    @property
    def ns(self) -> float:
        return self.cycles * NS_PER_TICK

    @property
    def hz(self) -> float:
        return F_CLK_HZ / self.cycles if self.cycles else 0.0

    def describe(self) -> str:
        return (f"{self.name}: CSn low for {self.cycles} core cycles ({self.ns:.0f} ns), "
                f"{self.sck_rises} SCK rises for {self.mosi_bytes} out / {self.miso_bytes} in "
                f"bytes at {self.ticks_per_bit} ticks per bit; CSn setup {self.setup}, hold "
                f"{self.hold}, shortest SCK low {self.min_low} and high {self.min_high} cycles")


def _phases(probe, fall: int, rise: int) -> dict:
    """The four host timing minimums, read off the edge log between `fall` and `rise`.

    Every one of them is a difference between two cycles the log already carries: the CSn edges,
    and each SCK edge in turn.  A phase that is absent is reported as 0, which `check_framing`
    then rejects -- a transaction with no SCK phase has no phase minimum to meet.
    """
    sck = [(e.t, e.level) for e in probe.edges if e.net == SCK and fall < e.t < rise]
    lows = [b - a for (a, la), (b, lb) in zip(sck, sck[1:])
            if la == 1 and lb == 0 and b > a]           # a high phase, then low
    highs = [b - a for (a, la), (b, lb) in zip(sck, sck[1:])
             if la == 0 and lb == 1 and b > a]           # a low phase, then high
    first_rise = next((t for t, level in sck if level == 1), rise)
    last_rise = max((t for t, level in sck if level == 1), default=fall)
    return dict(setup=max(0, first_rise - fall), hold=max(0, rise - last_rise),
                min_low=min(lows, default=0), min_high=min(highs, default=0))


def _measure(name: str, mosi: list[int], n_miso: int, ticks_per_bit: int) -> AccessCost:
    """One transaction on a bare four-wire bus, timed off the CSn window in the edge log."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for net in (SCK, MOSI, MISO, CS_N):
        bus.add_net(net, push_pull=True)
    host = bus.attach(SpiController(half_period=ticks_per_bit // 2, name="host"))
    probe = attach(bus, host, host, nets=(SCK, MOSI, MISO, CS_N), engine=0, chip_only=False)
    xfer = host.queue_transfer(mosi, n_miso)
    host.run(bus, limit=100_000)
    cs = [(e.t, e.level) for e in probe.edges if e.net == CS_N]
    fall = next(t for t, level in cs if level == 0)
    rise = next(t for t, level in cs if level == 1 and t > fall)
    sck_rises = sum(1 for e in probe.edges if e.net == SCK and e.level == 1
                    and fall <= e.t <= rise)
    phases = _phases(probe, fall, rise)
    probe.detach()
    return AccessCost(
        name=name, ticks_per_bit=ticks_per_bit, bytes_on_the_wire=len(mosi) + n_miso,
        cycles=rise - fall, sck_rises=sck_rises, mosi_bytes=len(xfer.mosi),
        miso_bytes=len(host.rx), detail={"cs_fall": fall, "cs_rise": rise,
                                         "bits_expected": 8 * (len(mosi) + n_miso)},
        **phases,
    )


def measure(ticks_per_bit: int = TICKS_PER_BIT) -> list[AccessCost]:
    """Every access shape the framing allows, at `ticks_per_bit` core cycles per SCK bit."""
    return [
        # One write: CMD | four data bytes = 40 bits.
        _measure("write", [CMD_WRITE | A_FLEVEL] + _word(0x0001_0001), 0, ticks_per_bit),
        # One read: CMD | TA | four data bytes = 48 bits.
        _measure("read", [CMD_READ | A_FLEVEL, TA] + [TA] * 4, 4, ticks_per_bit),
        # The same read from the FIFO register, where the read pops a word.
        _measure("read_rxf0_pop", [CMD_READ | A_RXF0, TA] + [TA] * 4, 4, ticks_per_bit),
        # Four words in one CMD: the FIFO registers do not auto-increment.
        _measure("txf0_burst_4", [CMD_WRITE | A_TXF0] + _word(0x1122_3344) * 4, 0, ticks_per_bit),
        # Four words in one CMD, read side: CMD | TA | D | TA | D ... = 72 bits.
        _measure("rxf0_burst_4", [CMD_READ | A_RXF0] + [TA] * 8, 16, ticks_per_bit),
        # The shortest legal transaction: a CMD with no data words has no effect on the chip,
        # but the host still spends the CMD byte on the wire.
        _measure("cmd_only", [CMD_READ | A_FDEBUG], 0, ticks_per_bit),
    ]


#: The host timing minimums of `docs/mvp-regmap.md` `### Physical layer`, in core cycles.  This
#: is the one number in this file that comes from the document rather than from the wire, and it
#: is the *judgement* the measured phases are compared against, not a measurement: a measurement
#: limit would make the check unable to fail.
MIN_PHASE = 2


def check_framing(cost: AccessCost) -> list[str]:
    """What the measured wire must show for the regmap's framing to hold; [] when it does.

    Four rules, three of them the regmap's framing and the fourth its timing minimums, all four
    checkable from the edge log and none of them the same test twice:

    1. every byte on the wire is eight SCK rises;
    2. a transaction is one CSn window, and it is longer than the bits cost -- which is rule 1
       again in the other direction, so it is only meaningful when rule 1 has passed;
    3. each of the four host timing phases (`### Physical layer`): CSn fall to the first SCK rise,
       the last SCK rise to the CSn rise, and the low and high phase of every SCK, each at least
       `MIN_PHASE` core cycles;
    4. every phase was measured at all -- a phase reported as 0 because the edge log had no such
       edge is not a phase that met its minimum.
    """
    problems = []
    if cost.sck_rises != 8 * cost.bytes_on_the_wire:
        problems.append(f"{cost.sck_rises} SCK rises for {cost.bytes_on_the_wire} bytes "
                        f"(every byte is 8 bits)")
    if cost.cycles <= cost.sck_rises * cost.ticks_per_bit:
        problems.append(f"a {cost.cycles}-cycle window cannot hold {cost.sck_rises} bits at "
                        f"{cost.ticks_per_bit} ticks each")
    for what, got in (("CSn setup, CSn fall to the first SCK rise", cost.setup),
                      ("CSn hold, the last SCK rise to the CSn rise", cost.hold),
                      ("the shortest SCK low phase", cost.min_low),
                      ("the shortest SCK high phase", cost.min_high)):
        if got < MIN_PHASE:
            problems.append(f"{what} is {got} core cycle(s); the host timing rules of "
                            f"`### Physical layer` ask for at least {MIN_PHASE}"
                            + ("" if got else " -- no such phase was measured at all, which is "
                                                   "itself the failure"))
    return problems
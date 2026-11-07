# SPDX-License-Identifier: Apache-2.0
"""The same directed stimulus on every firmware target.

`test/test_slowpath.py` measures the cycle counts on the Tiny Tapeout pins, which is what the
SDF-annotated gate-level bench needs.  This module measures the *same* counts, with the same
programs and the same expectations (`fw/slowpath/stimulus.py`), through
`iss.pio_iss.ChipDevice` on a `verif.vip.core.Bus`, so the numbers can be checked on every
`FW_TARGET` in `fw/targets.py`:

    FW_TARGET=iss    python -m pytest -q fw/slowpath
    FW_TARGET=iss_b  python -m pytest -q fw/slowpath
    FW_TARGET=rtl    python -m pytest -q fw/slowpath

That is what makes the pin-level expectations quotable rather than invented: the ISS is a cycle
model of `docs/mvp-isa.md`, so if these counts came from the derivation in `stimulus.py` alone,
the ISS would have to agree with them -- and if it ever does not, that is a disagreement between
the model and the document, to be reported and not smoothed over.

What is being measured, and why, is in `test/test_slowpath.py`'s header.  The short version: at the slow corner the
paths from the divider's `en` flop into `u_eng.ir` (the fetch) and into `u_eng.u_rxf.mem` (the
RX FIFO write) are 21.0 and 21.6 ns of cell delay, longer than the 20 ns clock, and this is the
stimulus that can see it.
"""

from __future__ import annotations

import pytest

from fw.slowpath import stimulus as sp
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus, Device

#: One access per core cycle, so a host access costs one tick here and tens of core cycles over
#: SPI (`docs/mvp-regmap.md` `### SPI transaction format`).  Same 20 ns core clock.
NS_PER_TICK = 20.0

#: `docs/mvp-isa.md` `### Constants`: 18 protocol pins, 2 engines, 4-deep FIFOs.
NUM_PINS = 18

#: The divisors, the settle time and the lost-word mask are the same ones the pin-level tests
#: use; they are named here so both files agree in one place each.
DIVISORS = (1, 2, 3)
SETTLE_CYCLES = 64
FDEBUG_LOST = 0x0003_0300


class Pins(Device):
    """The host's side of the protocol pins: drives the ones it owns, records the ones it reads.

    One `tick` per core cycle, so `samples[label][t]` is the level `label` had in core cycle `t`
    -- the same index space `test/test_slowpath.py`'s sampler uses, which is what lets both
    front-ends share `sp.release_after` and `sp.hold_cycles`.  Scheduled drives are applied
    before the sample is taken, so a pin driven for cycle `t` is the chip's input in cycle `t`.
    """

    name = "host"

    def __init__(self, bus: Bus, drive_pins, watch: dict, irq0=None, oe_of=None, levels=None):
        self.bus = bus
        self.drive_pins = {int(pin): 0 for pin in drive_pins}   # pin -> level the host holds
        if levels:
            self.drive_pins.update({int(p): int(v) for p, v in levels.items()})
        self.watch = dict(watch)                    # label -> pin to read
        #: `irq0` and `pin_oe` are not pins: the two interrupt lines and the pad direction are
        #: chip outputs, not Bus nets (`docs/backend-contract.md` `irq0: int; irq1: int`,
        #: `pin_oe: int`), so they are read from the chip.
        self.irq0 = irq0
        self.oe_of = oe_of
        self.samples = {label: [] for label in watch}
        for extra in ("irq", "oe"):
            if {"irq": irq0, "oe": oe_of}[extra] is not None:
                self.samples[extra] = []
        self.schedule = []                          # (cycle, pin, value)

    def tick(self, t: int, pins) -> dict:
        for at, pin, value in self.schedule:
            if at == t:
                self.drive_pins[pin] = value
        for label, pin in self.watch.items():
            self.samples[label].append(int(pins.get(sp.net_of(pin), 0)))
        if self.irq0 is not None:
            self.samples["irq"].append(int(self.irq0()))
        if self.oe_of is not None:
            self.samples["oe"].append(int(self.oe_of()) & 1)
        return {sp.net_of(pin): level for pin, level in self.drive_pins.items()}

    def drive_at(self, cycle: int, pin: int, value: int):
        self.schedule.append((int(cycle), int(pin), int(value)))

    def edges(self, label: str, since: int = 0, until: int | None = None) -> list:
        return [i for i in sp.rising_edges(self.samples[label])
                if i >= since and (until is None or i <= until)]

    def falls_after(self, label: str, edge: int) -> int:
        series = self.samples[label]
        for i in range(edge + 1, len(series)):
            if series[i] == 0:
                return i
        raise AssertionError(f"{label} never fell after core cycle {edge}")


class Rig:
    """A `Bus` at 20 ns/tick with `p0..p17`, the chip on it, and the host's pin device.

    `ChipDevice` is the backend contract's adapter: `fw/conftest.py` has already put this
    session's `FW_TARGET` behind `iss.pio_iss.Chip`, so the same code runs on the golden ISS, on
    ISS-B and on `src/pio_chip.sv` under cocotb.
    """

    def __init__(self, drive_pins=(), watch=None, want_irq=False, want_oe=False, levels=None):
        self.bus = Bus(ns_per_tick=NS_PER_TICK)
        for k in range(NUM_PINS):
            # Push-pull on the pins the host drives, so two drivers is an error and the host's
            # level is what the chip reads; the rest are the chip's alone.
            self.bus.add_net(sp.net_of(k), push_pull=(k in drive_pins))
        self.dev = ChipDevice()
        # Attachment order is the tick order (`verif/vip/core.py` `Bus.step`), and `Pins` has to
        # go first: it samples the resolved nets and the chip's `irq0` as they were at the end
        # of the previous tick, which is the state *during* this core cycle
        # (`docs/backend-contract.md`: "after step(), ... describe the state after the last
        # step()").  Attached second it would read them one cycle later than the nets, and the
        # latency arithmetic in `stimulus.py` is only meaningful if both are the same cycle.
        self.pins = Pins(self.bus, drive_pins, watch or {}, levels=levels,
                         irq0=(lambda: self.dev.chip.irq0) if want_irq else None,
                         oe_of=(lambda: self.dev.chip.pin_oe >> 0) if want_oe else None)
        self.bus.attach(self.pins)
        self.bus.attach(self.dev)

    # -- the clock ----------------------------------------------------------------

    @property
    def cycle(self) -> int:
        return self.bus.t - 1

    def run(self, cycles: int) -> int:
        self.bus.run(int(cycles))
        return self.cycle

    # -- one host access per core cycle ------------------------------------------

    def write(self, addr: int, *words: int) -> None:
        """One host write per word, one core cycle apart, at the same address (a burst)."""
        for i, word in enumerate(words):
            self.dev.write(addr, word & 0xFFFF_FFFF)
            while not self.dev.idle:
                self.bus.step()

    def read(self, addr: int) -> int:
        future = self.dev.read(addr)
        while not future.done():
            self.bus.step()
        return future.result()

# ---------------------------------------------------------------------------------------------
# Loading an engine, in the order of docs/mvp-regmap.md `### Typical host sequences`
# ---------------------------------------------------------------------------------------------


def load_probe(rig: Rig, engine: int, name: str, divisor: int, wrap=(0, 31)) -> None:
    """`fetch_probe`/`rxf_probe` (or a mutant) onto one engine at an integer `CLKDIV`.

    The same five steps as `test/test_slowpath.py`'s `load_probe`, and the same two forced
    instructions it needs: `jmp 0` (word 0x0000) onto the `wait`, and no `set pindirs`, because
    neither program drives a `uio` pin.
    """
    prog = sp.program(name)
    config = sp.config_for(name, divisor, wrap=wrap)
    bits = sp.ctrl_bits(engine, sp.CTRL_SM_ENABLE, sp.CTRL_SM_RESTART, sp.CTRL_CLKDIV_RESTART)
    rig.write(sp.A_CTRL, rig.read(sp.A_CTRL) & ~bits)                    # 1: stop the engine
    for i, word in enumerate(prog.instrs):        # 2: the program, IMEM words in order
        # A burst write steps the address by one per word outside the FIFO data registers
        # (docs/mvp-regmap.md `### Bursts`), and on the Bus there is no burst: one access per
        # word, at the address that word belongs to.
        rig.write(sp.imem_addr(engine, (prog.origin or 0) + i), word)
    rig.write(sp.sm_addr(engine, 0), sp.sm_clkdiv(divisor))             # 3: CLKDIV
    rig.write(sp.sm_addr(engine, 1), config["execctrl"])                #    EXECCTRL
    rig.write(sp.sm_addr(engine, 2), sp.sm_shiftctrl())                 #    SHIFTCTRL
    rig.write(sp.sm_addr(engine, 5), config["pinctrl"])                # 4: PINCTRL, forced jmp
    rig.write(sp.sm_addr(engine, 4), 0x0000)
    rig.write(sp.A_CTRL, rig.read(sp.A_CTRL) | bits)                     # 5: start it


def release_sweep(rig: Rig, engine: int, name: str, divisor: int, label: str, phases: int,
                  drain: bool = False) -> tuple:
    """One release per divider phase; returns `(latencies, words)`.

    Identical to `test/test_slowpath.py`'s `release_sweep`, down to the residue arithmetic in
    `sp.release_after` and the hold in `sp.hold_cycles`, because both call the same two
    functions.  `drain` reads one word out of the RX FIFO after each release, which is what a
    `rxf_probe` sweep needs -- `rx_valid` is a level, so without a drain there is one rising
    edge for the whole sweep and only the first release could be measured -- and it is also how
    each pushed word is read back and checked.  `fetch_probe` pushes nothing, so it must not
    drain: a host read of an empty RX FIFO returns 0 and sets `FDEBUG.RXUNDER`.
    """
    pin = sp.WAIT_PIN[name]
    latencies, words = [], []
    residue = rig.cycle % divisor
    for k in range(int(phases)):
        start = sp.release_after(rig.cycle, residue, divisor)
        # `start - 1`, not `start`: `verif.vip/core.py`'s `Bus.step` resolves a device's drives
        # into the *next* tick's snapshot, so a level driven in tick `start - 1` is the pin the
        # chip samples in its `start`-th cycle.  Without that tick the whole measurement is one
        # core cycle late (measured: every latency exactly one too big, at all three divisors).
        hold = sp.hold_cycles(divisor)
        rig.pins.drive_at(start - 1, pin, 1)
        rig.pins.drive_at(start - 1 + hold, pin, 0)
        after = start + hold + 4 * divisor + sp.OUT_PIPE + 8
        rig.run(max(1, after - rig.cycle))
        edges = rig.pins.edges(label, since=start, until=after)
        if not edges:
            raise AssertionError(
                f"{name} at CLKDIV={divisor}: release {k} in core cycle {start} produced no "
                f"rising edge on {label} within {after - start} core cycles")
        latencies.append(edges[0] - start)
        if drain:
            # Every word the release pushed, not one: `shift_probe` pushes four per release and a
            # 4-deep FIFO that is still full makes the next release's push stall, so the drain has
            # to empty it (`FLEVEL`'s RX field for this engine, bits 15:8 * engine + 4).
            got = []
            for _ in range(8):
                if not rig.read(sp.A_FLEVEL) >> sp.FLEVEL_RX(engine) & 0xF:
                    break
                got.append(rig.read(sp.RXF(engine)))
            words.append(got)
        else:
            words.append(None)
        residue += 1
    return latencies, words


def check_parked(rig: Rig, engine: int, label: str, since: int, what: str) -> None:
    """The engine must be sitting on its `wait`: `SMe_ADDR` is the PC and it does not move."""
    addr = rig.read(sp.sm_addr(engine, 3)) & 0x1F
    assert addr == 0, (f"{what}: SMe_ADDR = {addr}, not 0: the engine is not parked on the "
                       f"`wait` at address 0")
    moved = rig.pins.edges(label, since=since)
    assert not moved, f"{what}: {label} rose in core cycles {moved[:4]} while the pin was low"


# ---------------------------------------------------------------------------------------------
# The tests
# ---------------------------------------------------------------------------------------------


@pytest.mark.parametrize("divisor", DIVISORS, ids=[f"clkdiv{d}" for d in DIVISORS])
def test_fetch_latency_set(divisor: int) -> None:
    """`fetch_probe`'s answer arrives after exactly the core cycles `docs/mvp-isa.md` says.

    The whole set, not one latency: the release cycles are chosen one per divider phase (see
    `sp.release_after`), so every phase of `docs/mvp-isa.md` `### Clock divider` is accounted
    for and a path that settles one core cycle late moves the whole set.
    """
    rig = Rig(drive_pins=(sp.WAIT_PIN["fetch_probe"],),
              watch={"answer": sp.ANSWER_PIN["fetch_probe"]})
    load_probe(rig, 0, "fetch_probe", divisor)
    since = rig.cycle
    rig.run(SETTLE_CYCLES)
    check_parked(rig, 0, "answer", since, f"fetch_probe at CLKDIV={divisor}")
    latencies, _ = release_sweep(rig, 0, "fetch_probe", divisor, "answer",
                                 phases=sp.phases(divisor))
    sp.check_set(f"fetch_probe's answer latency at CLKDIV={divisor}", latencies,
                 sp.fetch_latency_set(divisor))
    first = rig.pins.edges("answer")[0]
    sp.check_exact(f"fetch_probe's answer pulse width at CLKDIV={divisor}",
                   rig.pins.falls_after("answer", first) - first, sp.fetch_pulse_width(divisor))
    assert rig.read(sp.A_FDEBUG) & FDEBUG_LOST == 0


@pytest.mark.parametrize("divisor", DIVISORS, ids=[f"clkdiv{d}" for d in DIVISORS])
def test_rxf_write_latency_and_data(divisor: int) -> None:
    """`rxf_probe`'s push reaches `rx_valid` after the ISA's core cycles, with its word intact."""
    rig = Rig(drive_pins=(sp.WAIT_PIN["rxf_probe"],), want_irq=True)
    phases = sp.phases(divisor)
    load_probe(rig, 1, "rxf_probe", divisor)
    for word in sp.TXF_WORDS[:phases]:
        rig.write(sp.TXF(1), word)
    since = rig.cycle
    rig.run(SETTLE_CYCLES)
    check_parked(rig, 1, "irq", since, f"rxf_probe at CLKDIV={divisor}")
    latencies, words = release_sweep(rig, 1, "rxf_probe", divisor, "irq", phases=phases,
                                    drain=True)
    sp.check_set(f"rxf_probe's rx_valid latency at CLKDIV={divisor}", latencies,
                 sp.rxf_irq_latency_set(divisor))
    sp.check_exact(f"rxf_probe's pushed words at CLKDIV={divisor}",
                   [w[0] for w in words], sp.TXF_WORDS[:phases])
    assert rig.read(sp.A_FDEBUG) & FDEBUG_LOST == 0


@pytest.mark.parametrize("divisor", (1, 2), ids=["clkdiv1", "clkdiv2"])
def test_anchor_trace_matches_isa(divisor: int) -> None:
    """The `docs/mvp-isa.md` `### Anchor trace` program, cycle for cycle, at this divisor.

    The document's own normative example, quoted rather than derived: `0: set pindirs,1 /
    1: set pins,1 / 2: set pins,0` with `WRAP_BOTTOM = 1`, `WRAP_TOP = 2`, so the program wraps
    and never jumps, and the table says what `uio_oe[0]` and `uio_out[0]` are in each of the five
    cycles `h+1 .. h+5`.  Where the CTRL write landed is not knowable, so the table is aligned on
    its own first transition -- the direction write, one core cycle after the SM cycle that
    executed it -- and compared from there.  At `CLKDIV = 1` that comparison *is* the table.
    """
    rig = Rig(want_oe=True, watch={"out": 0})
    load_probe(rig, 0, "anchor", divisor, wrap=(1, 2))
    since = rig.cycle
    rig.run(SETTLE_CYCLES)
    first = rig.pins.edges("oe", since=since)
    assert first, "the anchor program never drove uio_oe[0]"
    start = first[0]
    cycles = 6 * divisor + 4
    got = list(zip(rig.pins.samples["oe"][start:start + cycles],
                   rig.pins.samples["out"][start:start + cycles]))
    sp.check_exact(f"the anchor trace at CLKDIV={divisor}", got,
                   sp.anchor_expected(divisor, cycles))


@pytest.mark.parametrize("taken", (True, False), ids=["jmp-taken", "jmp-not-taken"])
def test_fetch_control_period(taken: bool) -> None:
    """One program, two loop lengths: the JMP's PIN condition follows the host's pin 8.

    `fetch_control` pulses its answer pin once per loop and loops four SM cycles when the host's
    pin is high (the branch is taken, over the two `nop`s it skips) and six when it is low.  Both
    are the ISA's `### Cycle accounting` -- a taken JMP costs one SM cycle like any other
    instruction -- so the period is exactly the loop length, which is what makes it a check on
    the fetch path's PC choice rather than on its timing.
    """
    divisor = 2
    pin = sp.WAIT_PIN["fetch_control"]
    rig = Rig(drive_pins=(pin,), levels={pin: 1 if taken else 0},
              watch={"answer": sp.ANSWER_PIN["fetch_control"]})
    load_probe(rig, 0, "fetch_control", divisor)
    rig.run(4 * divisor * divisor + 16)          # let the loop settle into its period
    before = rig.cycle
    rig.run(4 * divisor * divisor + 40)
    rises = rig.pins.edges("answer", since=before)[2:]
    assert len(rises) >= 2, (
        f"fetch_control with the pin {'high (jmp taken)' if taken else 'low (not taken)'}: only "
        f"{len(rises)} rising edges on the answer pin")
    gaps = [b - a for a, b in zip(rises, rises[1:])]
    sp.check_exact(f"fetch_control's period with the pin "
                   f"{'high (jmp taken)' if taken else 'low (not taken)'} at CLKDIV={divisor}",
                   gaps[0], sp.control_period(taken, divisor))
    assert all(g == gaps[0] for g in gaps), f"the period is not steady: {gaps}"


@pytest.mark.parametrize("name", ("fetch_probe_late", "rxf_probe_late"))
def test_planted_nop_moves_the_answer_one_sm_cycle(name: str) -> None:
    """The negative control: the mutants' answer must arrive exactly `divisor` cycles later.

    Each mutant is its program with one `nop` in front of the instruction that answers, so the
    *measured* latency set has to be the nominal one shifted by one SM cycle.  If the measurement
    were reading anything other than the program's own timing -- a constant offset, the length of
    a host access, the divider's phase -- the two would agree and this would fail.
    """
    divisor = 2
    clean, engine, label, wanted = (
        ("fetch_probe", 0, "answer", sp.fetch_latency_set(divisor)) if name.startswith("fetch")
        else ("rxf_probe", 1, "irq", sp.rxf_irq_latency_set(divisor)))
    txf = () if name.startswith("fetch") else sp.TXF_WORDS[:sp.phases(divisor)]
    got = {}
    for which in (clean, name):
        rig = Rig(drive_pins=(sp.WAIT_PIN[which],), want_irq=(engine == 1),
                  watch=({label: sp.ANSWER_PIN[which]} if engine == 0 else {}))
        for word in txf:
            rig.write(sp.TXF(engine), word)
        load_probe(rig, engine, which, divisor)
        rig.run(SETTLE_CYCLES)
        got[which] = release_sweep(rig, engine, which, divisor, label,
                                   phases=sp.phases(divisor), drain=(engine == 1))[0]
    sp.check_set(f"{name}'s latency at CLKDIV={divisor}", got[name],
                 sp.fetch_latency_set(divisor, name) if engine == 0
                 else sp.rxf_irq_latency_set(divisor, name))
    assert set(got[name]) == {v + divisor for v in wanted}, (
        f"the planted nop moved the answer by "
        f"{sorted(v - w for v, w in zip(sorted(got[name]), sorted(wanted)))}, not one SM cycle "
        f"({divisor} core cycles)")

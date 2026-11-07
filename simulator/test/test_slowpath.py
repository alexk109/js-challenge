# SPDX-License-Identifier: Apache-2.0
"""Directed chip-level tests for the paths the slow corner fails.

`test/test.py` passes the SDF-annotated gate-level netlist at the slow corner at 20 ns, where
STA says the chip misses by 9.895 ns.  Nothing in it measures a cycle count: every observation
there is a poll of a status line and every assertion is about a value, so a path that settles
one core cycle late cannot show up.  These tests measure cycle counts, over the same host port
and on the same TT pins, so `sim/sdf/run.py` runs them on the annotated netlist unchanged.

What is aimed at, and why these two paths (the census of slow paths comes from the SDF path report):

| path | STA slack at 20 ns | SDF cell delay (typ) | 20 ns | 30 ns |
|---|---|---|---|---|
| `sm_en[1] -> u_rxf.mem[40]`, the RX FIFO write | -9.895 ns | 26.211 ns | misses by 6.211 | fits |
| `sm_en[1] -> ir[12]`, the fetch | -9.514 ns | 25.721 ns | misses by 5.721 | fits |
| `div_en[0] -> u_rxf.mem[78]`, the RX FIFO write | -5.012 ns | 21.605 ns | misses by 1.605 | fits |
| `div_en[0] -> ir[4]`, the fetch | -4.307 ns | 20.996 ns | misses by 0.996 | fits |

`sm_en` is the registered `CTRL.SM_ENABLE`, so it moves once, when the host starts the engine;
`div_en` is the divider's `en` flop, so with an integer `CLKDIV = D > 1` it moves every `D`
cycles -- once per SM cycle.  Both are exercised here, at `CLKDIV` 1, 2 and 3.

Every measurement is anchored on an edge **the host owns**: a `wait 1 pin` instruction parks the
engine until a pin on `ui_in` goes high, and the host knows the exact core cycle it drove it.
Anchoring on an engine-produced pin instead would not do: `sm_en -> uo_out[6]` is -9.523 ns at
the slow corner, so that anchor would move with the thing being measured.

Each check is exact rather than a bound.  A release is issued once per divider phase -- the
release cycle is chosen so that it advances by one modulo `D` each time -- and then the *set* of
latencies is what the ISA fixes, so every phase has to be accounted for.

    test_fetch_path_ir_tracks_pc    the fetch (next PC -> 32:1 IMEM read -> ir): the answer's
                                   latency set and pulse width, the documented anchor trace's
                                   per-cycle pin values, and a taken and a not-taken JMP
    test_sm_en_to_rxf_write         engine 1's sm_en -> u_rxf write path: the latency from the
                                   release to `rx_valid`, and the word that came back
    test_slowpath_negative_control  the planted-fault mutants, through the same checkers

**Measured: all three pass at every configuration tried**, including `nom_slow_1p08V_125C` at
20 ns, which was expected to fail -- the two paths above cannot be made to fail there, as the
path report shows.  The planted-fault control is
the other half of that statement: it passes in every run, so the measurement does resolve a
one-SM-cycle shift at every corner and clock.

`test/test.py` is loaded, not copied: its `SpiHost` (the host model, the Bridge and the VIPs)
and `load_engine` (the host sequence of `docs/mvp-regmap.md` `### Typical host sequences`) are
the ones the four existing tests use, so these tests drive the chip exactly as they do.  Only
`NS_PER_TICK` is set from `SDF_TICK_NS`, which is how the SDF bench runs the same stimulus at a
30 ns clock (`sim/sdf/sdf_test.py` does the same for the same reason).
"""

import importlib.util
import os
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge, Timer

_ROOT = Path(__file__).resolve().parents[1]
for _p in (str(_ROOT), str(_ROOT / "test")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fw.slowpath import stimulus as sp                             # noqa: E402

# test/test.py is loaded by path, not by name: the repository root also holds a *directory*
# called `test`, which wins the name lookup as a namespace package (sim/sdf/sdf_test.py says so
# too).  Its cocotb tests are not re-exported here -- `make sdf-test` runs that module for them.
_spec = importlib.util.spec_from_file_location("chip_tests", _ROOT / "test" / "test.py")
chip = importlib.util.module_from_spec(_spec)
sys.modules["chip_tests"] = chip
_spec.loader.exec_module(chip)

if os.environ.get("SDF_TICK_NS"):     # the core clock is a variable (sim/sdf/README.md)
    chip.NS_PER_TICK = float(os.environ["SDF_TICK_NS"])
NS_PER_TICK = chip.NS_PER_TICK
CLK_HZ = chip.CLK_HZ

#: Where in the core cycle the sampler looks.  A pin's level register is written at the edge and
#: the pad follows it within a couple of nanoseconds at the slow corner, so a quarter of the way
#: in is settled -- and a path that spills past the edge shows up late, which is the point.
SAMPLE_PHASE = 0.25

#: The integer divisors to run at.  1.0 makes `sm_en` the only transition into the engine's
#: datapath; 2 and 3 make `div_en` transition once per SM cycle, which is the recurring path.
DIVISORS = (1, 2, 3)

#: Core cycles to let an engine reach its `wait` after the CTRL write that started it.
SETTLE_CYCLES = 64

#: `set pindirs, 0`: opcode 111, destination PINDIRS (bits 7:5 = 100), data 0.  Written to
#: `SM0_INSTR` to let go of a `uio` pin, which `set pindirs,1` would otherwise keep driving.
SET_PINDIRS_OFF_WORD = 0xE080

#: FDEBUG bits that mean a word was lost or popped out of an empty FIFO (docs/mvp-regmap.md
#: `**FDEBUG**`).  TXSTALL/RXSTALL are expected: over SPI the engine waits for the host.
FDEBUG_LOST = 0x0003_0300



# ---------------------------------------------------------------------------------------------
# Watching the pins, one core cycle at a time
# ---------------------------------------------------------------------------------------------


class Sampler:
    """One sample per core cycle: the level of each watched signal, `SAMPLE_PHASE` into it.

    Also the clock authority for these tests: a scheduled write lands in the core cycle whose
    index it names, so "drive this pin high in core cycle 214" is exact, and the samples and the
    schedule share one index space (sample `i` is core cycle `i` of the sampler).  `signals` maps
    a label to the `(port, bit)` it watches, so two bits of one port can both be watched.

    The sampler drives nothing except the host's own release pins, and only from the schedule,
    so it cannot change what the chip does.
    """

    def __init__(self, dut, signals):
        self.dut = dut
        self.signals = dict(signals)              # label -> (port, bit)
        self.samples = {label: [] for label in signals}
        self.driven = {}                          # port -> the byte the host is driving
        self.schedule = []                        # (cycle, port, bit, value)
        self._task = None

    async def _run(self):
        offset = NS_PER_TICK * SAMPLE_PHASE
        cycle = 0
        while True:
            await RisingEdge(self.dut.clk)
            await Timer(offset, unit="ns")
            for label, (port, bit) in self.signals.items():
                self.samples[label].append((int(getattr(self.dut, port).value) >> bit) & 1)
            for at, port, bit, value in self.schedule:
                if at == cycle:
                    self.drive(port, bit, value)
            cycle += 1

    def drive(self, port: str, bit: int, value: int):
        """Set one bit of one top-level port, read-modify-write so the rest is untouched.

        The read-modify-write matters for `ui_in`, where bits 2:0 are the SPI host port the
        Bridge is driving: this must not disturb CSn/SCK/MOSI.
        """
        handle = getattr(self.dut, port)
        byte = self.driven.get(port, int(handle.value))
        handle.value = (byte | (1 << bit)) if value else (byte & ~(1 << bit))
        self.driven[port] = int(handle.value)

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        if self._task:
            self._task.cancel()
            self._task = None

    # -- the clock ------------------------------------------------------------------

    def cycle(self) -> int:
        """The index of the core cycle being sampled now."""
        return len(next(iter(self.samples.values()))) - 1

    async def wait(self, cycles: int) -> int:
        """Let `cycles` core cycles pass and return the index of the cycle after them."""
        await ClockCycles(self.dut.clk, int(cycles))
        await Timer(NS_PER_TICK * SAMPLE_PHASE, unit="ns")
        return self.cycle()

    def drive_at(self, cycle: int, port: str, bit: int, value: int):
        self.schedule.append((int(cycle), port, bit, int(value)))

    # -- the record ------------------------------------------------------------------

    def edges(self, label: str, since: int = 0, until: int | None = None) -> list:
        """The cycles at or after `since` in which `label` rose."""
        found = sp.rising_edges(self.samples[label])
        return [i for i in found if i >= since and (until is None or i <= until)]

    def samples_from(self, label: str, first: int, last: int) -> list:
        """The samples of `label` from cycle `first` to cycle `last`, both inclusive."""
        return self.samples[label][first:last + 1]

    def falls_after(self, label: str, edge: int) -> int:
        """The first cycle after `edge` in which `label` fell, or raise."""
        series = self.samples[label]
        for i in range(edge + 1, len(series)):
            if series[i] == 0:
                return i
        raise AssertionError(f"{label} never fell after core cycle {edge}")


# ---------------------------------------------------------------------------------------------
# Loading an engine, in the order of docs/mvp-regmap.md `### Typical host sequences`
# ---------------------------------------------------------------------------------------------


async def load_probe(host, engine: int, name: str, divisor: int, wrap=(0, 31)) -> None:
    """`fetch_probe`/`rxf_probe` (or a mutant) onto one engine at an integer `CLKDIV`.

    The same sequence `test/test.py`'s `load_engine` runs, with the two forced instructions this
    needs: `jmp 0` to put the PC on the `wait` (word 0x0000 is `jmp 0`, `docs/mvp-isa.md`
    `### Field values`), and for `rxf_probe` nothing else -- the program reads its data word from
    the TX FIFO, which the host fills before starting the engine.
    """
    prog = sp.program(name)
    config = sp.config_for(name, divisor, wrap=wrap)
    e = sp.ctrl_bits(engine, chip.txd.CTRL_SM_ENABLE, chip.txd.CTRL_SM_RESTART,
                     chip.txd.CTRL_CLKDIV_RESTART)
    ctrl = await host.read1(chip.A_CTRL)
    await host.write(chip.A_CTRL, ctrl & ~e)                       # 1: the engine is stopped
    await host.write(chip.imem_addr(engine, prog.origin or 0), *prog.instrs)   # 2: the program
    await host.write(chip.sm_addr(engine, chip.SM_CLKDIV), sp.sm_clkdiv(divisor),
                     config["execctrl"], sp.sm_shiftctrl())         # 3: CLKDIV/EXECCTRL/SHIFTCTRL
    await host.write(chip.sm_addr(engine, chip.SM_PINCTRL), config["pinctrl"])
    await host.write(chip.sm_addr(engine, chip.SM_INSTR), 0x0000)   # 4: forced `jmp 0`
    ctrl = await host.read1(chip.A_CTRL)
    await host.write(chip.A_CTRL, ctrl | e)                        # 5: start it


async def check_parked(host, sampler: Sampler, engine: int, label: str, since: int,
                       what: str) -> None:
    """The engine must be sitting on its `wait`, or the latency below means nothing.

    Two things say so, and neither is the IRQ: `SMe_ADDR` reads the PC, which does not move while
    an instruction fetched from IMEM stalls (`docs/mvp-isa.md` `### Reference step`: a stalled
    instruction leaves the PC alone), and the answer pin must not have moved during the settle.
    `EXECCTRL.EXEC_STALLED` is *not* usable here: bit 31 is `latch_pend & latch_host`, so it
    reports a stalled host-forced instruction only, never one fetched from IMEM -- which is what
    `test/test.py`'s `test_protocol_pin_map` uses it for, on a forced `wait`.
    """
    addr = await host.read1(chip.sm_addr(engine, chip.SM_ADDR)) & 0x1F
    assert addr == 0, (
        f"{what}: SMe_ADDR = {addr}, not 0: the engine is not parked on the `wait` at address 0, "
        f"so a release would not measure what it claims to")
    moved = sampler.edges(label, since=since)
    assert not moved, (
        f"{what}: {label} rose in core cycles {moved[:4]} while the release pin was low, so the "
        f"engine was not parked")


async def release_sweep(host, sampler: Sampler, engine: int, name: str, divisor: int,
                        label: str, phases: int, drain: bool = False) -> tuple:
    """One release per divider phase, and the `(release cycle, edge cycle)` pairs they produced.

    `drain` reads one word out of the RX FIFO after each release.  A `rxf_probe` sweep needs
    that, because `rx_valid` is a level: without a drain there is one rising edge for the whole
    sweep and only the first release could be measured, and it is also how each pushed word is
    read back and checked.  `fetch_probe` pushes nothing and must not drain: a host read of an
    empty RX FIFO returns 0 and sets `FDEBUG.RXUNDER`.

    Returns `(latencies, words)`: the core cycles from each release to the rising edge of
    `label`, and the words the engine pushed, in the order it pushed them (`None`s when not
    draining).
    """
    port, bit = sp.pin_port(sp.WAIT_PIN[name], "in")
    latencies, words = [], []
    residue = sampler.cycle() % divisor
    for k in range(int(phases)):
        start = sp.release_after(sampler.cycle(), residue, divisor)
        sampler.drive_at(start, port, bit, 1)
        sampler.drive_at(start + sp.hold_cycles(divisor), port, bit, 0)
        after = start + sp.hold_cycles(divisor) + 4 * divisor + sp.OUT_PIPE + 8
        await sampler.wait(max(1, after - sampler.cycle()))
        edges = sampler.edges(label, since=start, until=after)
        if not edges:
            # The whole window, the two FIFO levels and the PC: which of those four is wrong
            # says which of the three ways this can fail it is (never answered / answered late /
            # answered before the release).
            window = "".join(str(v) for v in sampler.samples_from(label, start, after))
            flevel = await host.read1(chip.A_FLEVEL)
            pc = await host.read1(chip.sm_addr(engine, chip.SM_ADDR)) & 0x1F
            raise AssertionError(
                f"{name} at CLKDIV={divisor}: release {k} in core cycle {start} produced no "
                f"rising edge on {label} within {after - start} core cycles\n"
                f"    {label} over that window, from the release: {window}\n"
                f"    FLEVEL {flevel:#010x} (TX {flevel & 0xF}, RX {(flevel >> 4) & 0xF}), "
                f"SM{engine}_ADDR {pc}, FDEBUG {await host.read1(chip.A_FDEBUG):#010x}")
        latencies.append(edges[0] - start)
        words.append(await host.read1(chip.A_RXF[engine]) if drain else None)
        residue += 1
    return latencies, words


@cocotb.test()
async def test_fetch_path_ir_tracks_pc(dut):
    """The fetch path: next PC -> 32:1 IMEM read -> `ir`, with the ISA's cycle counts.

    Three programs, all on engine 0, all driven through the same host port as `test/test.py`:

    * `fetch_probe` waits on `ui_in[3]` and answers with a pulse on `uo_out[3]`.  The latency
      set and the pulse width are the fetch path's, at three integer divisors.
    * the `docs/mvp-isa.md` `### Anchor trace` program, verbatim, with the documented wrap
      window 1..2: its per-cycle pin values are the document's own table.
    * `fetch_control`, which reads the host's pin into `x` and branches on it, so the *same*
      program takes a taken JMP or a not-taken one and the two periods differ by one SM cycle.
    """
    host = chip.SpiHost(dut)
    await host.start()
    sampler = Sampler(dut, {"answer": sp.pin_port(sp.ANSWER_PIN["fetch_probe"], "out"),
                            "oe": ("uio_oe", 0), "out": ("uio_out", 0)})
    sampler.start()
    try:
        for divisor in DIVISORS:
            await load_probe(host, 0, "fetch_probe", divisor)
            since = sampler.cycle()
            await sampler.wait(SETTLE_CYCLES)
            await check_parked(host, sampler, 0, "answer", since,
                               f"fetch_probe at CLKDIV={divisor}")
            sweep_start = sampler.cycle()
            latencies, _ = await release_sweep(host, sampler, 0, "fetch_probe", divisor,
                                               "answer", phases=sp.phases(divisor))
            wanted = sp.fetch_latency_set(divisor)
            sp.check_set(f"fetch_probe's answer latency at CLKDIV={divisor}", latencies, wanted)
            # The answer is high for exactly one SM cycle: `set pins,1` and `set pins,0` are
            # one instruction apart, so its width is the divisor and nothing else.
            # The width of a pulse from *this* divisor's sweep: the first edge of the whole run
            # belongs to the previous divisor's, and at CLKDIV=1 it is a cycle narrower.
            first = sampler.edges("answer", since=sweep_start)[0]
            sp.check_exact(f"fetch_probe's answer pulse width at CLKDIV={divisor}",
                           sampler.falls_after("answer", first) - first,
                           sp.fetch_pulse_width(divisor))
            fdebug = await host.read1(chip.A_FDEBUG)
            assert fdebug & FDEBUG_LOST == 0, f"FDEBUG = {fdebug:#010x}"
            host.check()

        # The documented anchor trace: `CLKDIV = 1.0`, WRAP_BOTTOM = 1, WRAP_TOP = 2, and
        # CTRL written in one cycle, which the chip sees as "the first SM cycle is h+2".  Where
        # that CTRL write landed is not knowable over the host port, so the table is aligned on
        # its own first transition (`set pindirs,1` reaching `uio_oe[0]` one core cycle after the
        # SM cycle that executed it) and compared from there.
        for divisor in (1, 2):
            # `since` before the load, not after it: the CTRL write is the last thing
            # `load_probe` does, and the engine is already two SM cycles into the program by the
            # time that SPI window ends, so the first instruction has run before a `since` taken
            # afterwards would see it.
            since = sampler.cycle()
            # Release uio pin 0 first.  `set pindirs,1` drives it once and nothing clears it, so
            # on the second pass there is no rising edge to align on and the previous divisor's
            # pin pattern would be compared with this one's expectation.  A forced instruction
            # runs in the next core cycle whatever the engine is doing (`docs/mvp-isa.md`
            # `### EXEC`), which is also the only use of EXEC in this file's chip tests.
            await host.write(chip.sm_addr(0, chip.SM_INSTR), SET_PINDIRS_OFF_WORD)
            await host.wait_ticks(4)
            await load_probe(host, 0, "anchor", divisor, wrap=(1, 2))
            await sampler.wait(SETTLE_CYCLES)
            oe_edges = sampler.edges("oe", since=since)
            assert oe_edges, "the anchor program never drove uio_oe[0]"
            first = oe_edges[0]
            want = sp.anchor_expected(divisor, 6 * divisor + 4)
            got = [(o, v) for o, v in
                   zip(sampler.samples_from("oe", first, first + len(want) - 1),
                       sampler.samples_from("out", first, first + len(want) - 1))]
            if got != want:
                raise AssertionError(
                    f"the anchor trace at CLKDIV={divisor} is {got}, the ISA's is {want}\n"
                    f"    SM0_CLKDIV {await host.read1(chip.sm_addr(0, chip.SM_CLKDIV)):#010x}, "
                    f"SM0_EXECCTRL {await host.read1(chip.sm_addr(0, chip.SM_EXECCTRL)):#010x},\n"
                    f"    SM0_PINCTRL {await host.read1(chip.sm_addr(0, chip.SM_PINCTRL)):#010x}, "
                    f"SM0_ADDR {await host.read1(chip.sm_addr(0, chip.SM_ADDR)):#010x}, "
                    f"SM0_INSTR {await host.read1(chip.sm_addr(0, chip.SM_INSTR)):#010x}, "
                    f"first transition in core cycle {first}")

        # A taken and a not-taken JMP, from one program: the host's pin decides which.
        divisor = 2
        for level, taken in ((1, True), (0, False)):
            await load_probe(host, 0, "fetch_control", divisor)
            await sampler.wait(SETTLE_CYCLES)
            port, bit = sp.pin_port(sp.WAIT_PIN["fetch_control"], "in")
            # x = the host's pin, held for the whole run: this program never waits on it
            sampler.drive(*sp.pin_port(sp.WAIT_PIN["fetch_control"], "in"), level)
            await sampler.wait(12 * divisor + 16)            # let the loop settle into its period
            before = sampler.cycle()
            await sampler.wait(4 * divisor * divisor + 40)
            rises = [e for e in sampler.edges("answer", since=before)][2:]
            assert len(rises) >= 2, (
                f"fetch_control with the pin {'low' if not level else 'high'}: only {len(rises)} "
                f"rising edges on the answer pin in {4 * divisor * divisor + 40} core cycles")
            gaps = [b - a for a, b in zip(rises, rises[1:])]
            sp.check_exact(f"fetch_control's period with the pin "
                           f"{'low (jmp taken)' if taken else 'high (not taken)'} "
                           f"at CLKDIV={divisor}", gaps[0], sp.control_period(taken, divisor))
            assert all(g == gaps[0] for g in gaps), f"the period is not steady: {gaps}"
            sampler.drive(*sp.pin_port(sp.WAIT_PIN["fetch_control"], "in"), 0)
        host.check()
    finally:
        sampler.stop()
        await host.stop()


@cocotb.test()
async def test_sm_en_to_rxf_write(dut):
    """Engine 1's `sm_en` -> RX FIFO memory write path, measured at the `rx_valid` line.

    `rxf_probe` waits on `ui_in[4]`, pulls one word out of engine 1's TX FIFO and pushes it, so
    the write lands in `u_eng.u_rxf.mem`, which is where the slow corner's worst path ends
    (`_29046_/Q = u_chip.g_eng[1].u_eng.u_rxf.mem[40]`, STA -9.895 ns).  The measurement is the
    latency from the host's release to `rx_valid`, which
    `docs/mvp-regmap.md` `### tx_ready and rx_valid` fixes at two core cycles after the FIFO
    change; and the check that the word that came back is the word that went in, read out one
    release at a time.
    """
    host = chip.SpiHost(dut)
    await host.start()
    sampler = Sampler(dut, {"irq": ("uo_out", sp.BIT_RX_VALID)})
    sampler.start()
    try:
        for divisor in DIVISORS:
            phases = sp.phases(divisor)
            await load_probe(host, 1, "rxf_probe", divisor)
            for word in sp.TXF_WORDS[:phases]:
                await host.write(chip.A_TXF[1], word)
            since = sampler.cycle()
            await sampler.wait(SETTLE_CYCLES)
            await check_parked(host, sampler, 1, "irq", since, f"rxf_probe at CLKDIV={divisor}")
            latencies, words = await release_sweep(host, sampler, 1, "rxf_probe", divisor,
                                                   "irq", phases=phases, drain=True)
            sp.check_set(f"rxf_probe's rx_valid latency at CLKDIV={divisor}", latencies,
                         sp.rxf_irq_latency_set(divisor))
            sp.check_exact(f"rxf_probe's pushed words at CLKDIV={divisor}", words,
                           sp.TXF_WORDS[:phases])
            fdebug = await host.read1(chip.A_FDEBUG)
            assert fdebug & FDEBUG_LOST == 0, f"FDEBUG = {fdebug:#010x}"
            host.check()
    finally:
        sampler.stop()
        await host.stop()


@cocotb.test()
async def test_slowpath_negative_control(dut):
    """The planted-fault mutants, through the same checkers: one extra `nop`, `D` cycles later.

    Each mutant of `program.pio` is the original with one `nop` in front of the instruction that
    produces the answer, so every answer must arrive exactly one SM cycle -- `D` core cycles --
    later.  If the measurement were reading something other than the program's own timing (a
    constant offset, the SPI transaction's length, the divider's phase) the two measurements
    would agree, and this test fails.  It also asserts that the *checkers* reject the wrong set,
    so a checker that quietly accepted a one-cycle slip would fail here too.
    """
    host = chip.SpiHost(dut)
    await host.start()
    sampler = Sampler(dut, {"answer": sp.pin_port(sp.ANSWER_PIN["fetch_probe_late"], "out"),
                            "irq": ("uo_out", sp.BIT_RX_VALID)})
    sampler.start()
    try:
        divisor = 2
        # The fetch mutant, on engine 0, against the *original* program's expectation.
        await load_probe(host, 0, "fetch_probe", divisor)
        await sampler.wait(SETTLE_CYCLES)
        clean, _ = await release_sweep(host, sampler, 0, "fetch_probe", divisor, "answer",
                                       phases=sp.phases(divisor))
        await load_probe(host, 0, "fetch_probe_late", divisor)
        await sampler.wait(SETTLE_CYCLES)
        planted, _ = await release_sweep(host, sampler, 0, "fetch_probe_late", divisor, "answer",
                                         phases=sp.phases(divisor))
        sp.check_set(f"the planted nop's answer latency at CLKDIV={divisor}", planted,
                     sp.fetch_latency_set(divisor, "fetch_probe_late"))
        assert set(planted) != set(clean), (
            f"the mutant measured exactly what the clean program measured ({clean}); the "
            f"measurement is not reading the program's timing")
        for got, wanted, what in ((planted, sp.fetch_latency_set(divisor), "the mutant"),
                                  (clean, sp.fetch_latency_set(divisor, "fetch_probe_late"),
                                   "the clean program")):
            try:
                sp.check_set(f"negative control: {what} against the wrong expectation", got, wanted)
            except AssertionError as exc:
                assert "expected the set" in str(exc), exc
            else:
                raise AssertionError(
                    f"negative control: {what} was accepted against the wrong expectation "
                    f"{sorted(wanted)}")

        # The same for the RX FIFO write path, on engine 1, measured at `rx_valid`.  Both sweeps
        # pull from the TX FIFO, so the words go in first: four of them, which is all a 4-deep
        # FIFO holds and all the two sweeps between them consume.  (Without them the engine
        # parks on the `pull` with `FDEBUG.RXSTALL` set and `rx_valid` never rises -- which is
        # what the first version of this test did, and what the failure message now says.)
        for word in sp.TXF_WORDS[:4]:
            await host.write(chip.A_TXF[1], word)
        await load_probe(host, 1, "rxf_probe", divisor)
        await sampler.wait(SETTLE_CYCLES)
        clean, _ = await release_sweep(host, sampler, 1, "rxf_probe", divisor, "irq",
                                       phases=sp.phases(divisor), drain=True)
        await load_probe(host, 1, "rxf_probe_late", divisor)
        await sampler.wait(SETTLE_CYCLES)
        planted, _ = await release_sweep(host, sampler, 1, "rxf_probe_late", divisor, "irq",
                                         phases=sp.phases(divisor), drain=True)
        sp.check_set(f"the planted nop's rx_valid latency at CLKDIV={divisor}", planted,
                     sp.rxf_irq_latency_set(divisor, "rxf_probe_late"))
        assert set(planted) != set(clean), (
            f"the RX FIFO mutant measured exactly what the clean program measured ({clean})")
        host.check()
    finally:
        sampler.stop()
        await host.stop()

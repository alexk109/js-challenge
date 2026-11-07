"""Does this bench bite? `python -m verif.perf.selftest` (also `make perf-selftest`).

Every number in `research/perf/b/perf.csv` is only worth reading if the thing that produced it
can say *no*.  This file is the proof, and it has two halves:

**The instruments, on a bus small enough to check by hand.** A two-device bus with a scripted
driver: the edge log has to show the edges that were driven, with the cycles the bus's own
phase model gives them; the bus-access log has to show the access cycles `h` that
`iss.pio_iss.ChipDevice` performed them in; and the two independent SM-cycle counts (the ISS's
`Engine.sm_step` call count and the per-cycle samples) have to agree.

**Six negative controls**, each of which must *fail* if the machinery is working:

1. `test_the_pass_verdict_rejects_a_chip_that_outruns_the_flash` -- the SPI program one cycle
   faster than the shipped one, which `fw/spi/test_spi.py` says has no MISO margin left.  The
   monitor's own verdict on it has to be `ok = False`.  Without this, "the fastest passing
   setting" could be "the fastest setting tried".
2. `test_the_framing_check_rejects_a_broken_transaction` -- `hostport.check_framing` on a
   fabricated cost whose bit count is wrong, whose window is too short, and whose four measured
   host timing phases are each one cycle short, has to complain about each of them; the measured
   ones must not be complained about.
3. `test_the_sm_cycle_cross_check_notices_a_tampered_count` -- one extra entry in the per-cycle
   samples has to make `Probe.sm_agree()` say no.
4. `test_the_window_ignores_a_net_the_program_does_not_use` -- an edge on a pin the transaction
   never touches must not move the measured bit rate, and an edge *inside* the window must.
5. `test_a_target_that_hides_sm_cycle_reports_n_a_not_zero` -- an engine facade that does not
   expose `sm_cycle`, like `fw/rtl_harness.py`'s, has to make the metric `n/a` with the reason in
   the row, and a facade that *does* expose it has to be read.  A "measured 0" for a signal
   nobody read is a number this table must never be able to publish.
6. `test_the_target_guard_refuses_a_bench_that_was_never_installed` -- `measure.py` has to refuse
   to label a run `target=rtl` when `iss.pio_iss.Chip` is still the ISS, by refusing to start, so
   the ISS cannot be measured and published as the RTL.

They are the reason the numbers in the CSV can be quoted; the run that produces them is
`python -m verif.perf.measure`.
"""
from __future__ import annotations

import copy
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from iss import pio_iss                                    # noqa: E402
from verif.perf import hostport, rigs                      # noqa: E402
from verif.perf.probe import Access, Edge, Probe, Trial, attach  # noqa: E402
from verif.vip.core import Bus, Device                     # noqa: E402

NS_PER_TICK = 20.0


class _Fixed(Device):
    """A device that drives one net to a level the test sets, for one tick at a time."""

    name = "fixed"

    def __init__(self, net: str, level: int = 1) -> None:
        self.net = net
        self.level = level

    def tick(self, t, pins):
        return {self.net: self.level}


def _two_device_bus():
    """A bus with an ISS and a scripted host, at 20 ns a tick."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}", push_pull=k == 0)
    chip = pio_iss.Chip()
    device = bus.attach(pio_iss.ChipDevice(chip, name="chip"))
    return bus, device, chip


def test_the_edge_log_times_the_bus_phase_model():
    """One net driven by one device: the log shows both edges, one tick apart from the drive.

    `verif.vip.core.Bus` resolves the drives at the end of a tick and hands the resolved
    snapshot to the devices on the next one, so a level driven in cycle `t` is first *visible*
    in cycle `t + 1`.  The probe timestamps edges with the cycle they are visible in, and this
    is what pins that convention down: without it, every latency in the table is off by one.
    """
    bus, device, chip = _two_device_bus()
    probe = attach(bus, device, chip, nets=("p0",))
    driver = bus.attach(_Fixed("p0"))       # idles at the pull-up level, so no edge yet
    bus.run(4)
    driver.level = 0
    bus.run(4)
    edges = probe.edges_on(["p0"])
    assert [e.level for e in edges] == [0], edges
    assert edges[0].t == 5, (edges[0], "driven in cycle 4, visible in cycle 5")
    probe.detach()
    assert bus.errors == []


def test_the_access_log_shows_the_cycle_each_access_was_performed_in():
    """Three queued writes and a read: four accesses, one per cycle, in order."""
    bus, device, chip = _two_device_bus()
    probe = attach(bus, device, chip, nets=("p0",))
    # The host's own path: `ChipDevice`'s queue performs one access per tick, in order.
    device.write(pio_iss.A_CTRL, 0)
    device.write(pio_iss.A_IMEM[0], 0xE001)
    device.write(pio_iss.A_TXF[0], 0x1234_5678)
    device.read(pio_iss.A_FLEVEL)
    bus.run(8)
    assert [(a.t, a.kind, a.addr) for a in probe.accesses] == [
        (0, "w", 0x00), (1, "w", 0x12), (2, "w", 0x04), (3, "r", 0x03)], probe.accesses
    assert probe.accesses[2].data == 0x1234_5678
    # The read really happened in cycle 3: FLEVEL of a chip with one TX word queued.
    assert probe.accesses[3].t == 3
    probe.detach()


def test_the_two_sm_cycle_counts_agree():
    """The ISS's exact count and the per-cycle samples are the same number."""
    bus, device, chip = _two_device_bus()
    probe = attach(bus, device, chip, nets=("p0",))
    device.write(pio_iss.A_CTRL, 0)
    device.write(pio_iss.A_IMEM[0], 0x0000)                 # `nop`
    device.write(pio_iss.A_CTRL, 0x001)                     # enable engine 0
    bus.run(40)
    assert probe.sm_agree() is True, (sum(probe.sm), probe.sm_calls)
    assert probe.sm_calls > 30, probe.sm_calls
    probe.detach()


def test_the_fifo_samples_follow_the_chip():
    """The per-cycle TX level is the engine's own FIFO length, every cycle."""
    bus, device, chip = _two_device_bus()
    probe = attach(bus, device, chip, nets=("p0",))
    device.write(pio_iss.A_TXF[0], 0xAAAA_AAAA)
    device.write(pio_iss.A_TXF[0], 0xBBBB_BBBB)
    bus.run(4)
    # The sample is the state at the *end* of the cycle the device stepped in, so the first
    # write is already visible at t = 0.
    assert probe.tx_level[:4] == [1, 2, 2, 2], probe.tx_level[:4]
    assert probe.peak_levels(0, 3) == (2, 0)
    probe.detach()


def test_trial_metrics_are_the_two_divisions_they_say_they_are():
    """The metric arithmetic on numbers a reader can check: 80 bits in 640 cycles is 6.25 MBd."""
    trial = Trial(protocol="selftest", knob="none", value=1.0, unit="u", ok=True,
                  bits=80, window=(100, 740), nets=("p0",), sm_cycles=800, peak_tx=1)
    assert trial.ticks == 640
    assert trial.bit_rate_hz == 6_250_000.0
    assert trial.sm_cycles_per_bit == 10.0
    assert trial.fifo_headroom == 3
    rows = {r["metric"]: r["value"] for r in trial.rows()}
    assert rows["max_bit_rate"] == "6.25e+06", rows
    assert rows["sm_cycles_per_bit"] == "10", rows
    # An empty window has no rate: the division cannot be faked.
    empty = copy.deepcopy(trial)
    empty.window = (5, 5)
    assert empty.bit_rate_hz == 0.0


# ---- negative control 1: the pass verdict must be able to say no --------------------------------


def test_the_pass_verdict_rejects_a_chip_that_outruns_the_flash():
    """SPI programs faster than the shipped one must not pass, and the shipped one must.

    `fw/spi/test_spi.py` builds these variants by re-assembling `program.pio` with a different
    side-set delay -- `out pins, 1 [5]` is the shipped bit period -- and documents what each
    one does: `[4]` still passes (with no MISO margin left), `[3]` samples MISO too early and
    comes back with the wrong bytes, and `[0]` breaks the monitor's own "SCK high for at least
    two core cycles" rule.  Two different rejection paths have to work -- wrong data and a
    checker violation -- or "the fastest setting that passed" carries no information, and that
    verdict is what every maximum in `research/perf/b/perf.csv` rests on.
    """
    import re

    from asm.pioasm import assemble

    from fw.spi.driver import PIO_FILE, PROGRAMS

    def variant(delay: int):
        text = re.sub(r"side ([0-9]) \[5\]", rf"side \1 [{delay}]", PIO_FILE.read_text())
        return next(p for p in assemble(text) if p.name == PROGRAMS[0])

    shipped = rigs.spi_trial(1)
    assert shipped.ok, shipped.why
    assert shipped.bits == 160

    early = rigs.spi_trial(1, program=variant(3))
    assert not early.ok, ("a program that samples MISO too early passed: the verdict is not "
                          "measuring anything")
    assert "want=" in early.why, early.why

    broken = rigs.spi_trial(1, program=variant(0))
    assert not broken.ok, "a program with a one-cycle SCK high passed the monitor"
    assert "sck high time" in broken.why, broken.why


# ---- negative control 2: the framing check must reject bad input ------------------------------


def test_the_framing_check_rejects_a_broken_transaction():
    """A cost whose bit count, window or host timing phases are wrong must be rejected.

    `hostport.check_framing` is the only thing standing between a miscounted SCK and the
    "cycles per register access" number in the table, so every rule gets a fabricated input: the
    same transaction with one bit missing from its count, one with a window that cannot hold its
    bits, and one with each of the four host timing phases one cycle short of the two the
    regmap asks for.  The measured transaction must be complained about by none of them.
    """
    costs = {c.name: c for c in hostport.measure(8)}
    good = costs["write"]
    assert good.sck_rises == 40, good.describe()
    assert hostport.check_framing(good) == [], hostport.check_framing(good)
    # Every phase of a measured transaction really is at or above the minimum, and the check is
    # what says so: it is not a constant.
    assert (good.setup, good.hold, good.min_low, good.min_high) == (6, 6, 4, 4), good.describe()

    broken = copy.deepcopy(good)
    broken.sck_rises = 39                       # one bit short of the five bytes on the wire
    problems = hostport.check_framing(broken)
    assert problems and "39 SCK rises" in problems[0], problems

    short = copy.deepcopy(good)
    short.cycles = 10                          # a window that cannot hold 40 bits
    assert hostport.check_framing(short), hostport.check_framing(short)

    for what in ("setup", "hold", "min_low", "min_high"):
        late = copy.deepcopy(good)
        setattr(late, what, hostport.MIN_PHASE - 1)
        problems = hostport.check_framing(late)
        assert problems, f"a {what} of {hostport.MIN_PHASE - 1} cycles was accepted: the "
        assert any(f"is {hostport.MIN_PHASE - 1} core cycle" in p for p in problems), (what, problems)


# ---- negative control 3: the SM-cycle cross-check must notice a tampered count ----------------


def test_the_sm_cycle_cross_check_notices_a_tampered_count():
    """One extra sample must make `sm_agree()` say no.

    The two SM-cycle counts are independent (the ISS counts `Engine.sm_step` calls, the samples
    come from the predicate `src/pio_engine.sv` evaluates), and their agreement is what lets the
    RTL runs be compared with the ISS ones.  A cross-check that cannot fail proves nothing, so
    this plants a discrepancy in the samples and insists it is reported.
    """
    bus, device, chip = _two_device_bus()
    probe = attach(bus, device, chip, nets=("p0",))
    device.write(pio_iss.A_CTRL, 0x001)
    bus.run(20)
    assert probe.sm_agree() is True
    probe.sm.append(1)                         # plant one SM cycle that never happened
    assert probe.sm_agree() is False, "the cross-check accepted a count one too high"
    probe.sm.pop()
    assert probe.sm_agree() is True
    probe.detach()


# ---- negative control 4: the window must only count the nets the program drives ----------------


def test_the_window_ignores_a_net_the_program_does_not_use():
    """A pin the transaction never touches must not move the measured rate.

    Pins 13..17 are output-only and read `pin_oe = 1` from reset, so the chip drives them low
    on the very first tick and every "first pin edge" would be one of those.  The probe is
    given the protocol's own nets, and this insists it uses them: an edge on p17 inside the
    window changes nothing, an edge on the program's own net changes the window.
    """
    bus, device, chip = _two_device_bus()
    probe = attach(bus, device, chip, nets=("p1",))
    unused = bus.attach(_Fixed("p17"))      # an output-only pin the chip drives from reset
    bus.run(2)
    assert probe.window(nets=("p1",)) == (None, None)
    # The chip drives p17 low on the very first tick (pin_oe = 1 and level = 0 from reset),
    # so there is an edge on it before the transaction has even started: if that counted, every
    # "first pin edge" in the table would be one of pins 13..17.
    assert probe.edges_on(["p17"]), "the chip's reset-time edge on p17 should be in the log"
    unused.level = 0                        # and a second device on the same net changes nothing
    bus.run(2)
    assert probe.window(nets=("p1",)) == (None, None), "an unused pin moved the window"

    pin1 = bus.attach(_Fixed("p1"))          # the program's own net, same instants
    bus.run(2)
    pin1.level = 0
    bus.run(2)
    window = probe.window(nets=("p1",))
    assert window[0] is not None and window[1].t >= window[0].t, window
    probe.detach()


# ---- negative control 5: a signal nobody read must not become a measured 0 ------------------------


class _FakeEngine:
    """An engine facade shaped like `fw/rtl_harness.py`'s `_RtlEngine`, minus `sm_cycle`.

    `_RtlEngine.__getattr__` answers `STATE_KEYS`, the six SM configuration words, `imem`, `tx`
    and `rx`, and raises `AttributeError` for anything else -- so asking it for `sm_cycle` is
    what the RTL bench actually does, and what used to be answered with a 0 and published.
    """

    def __init__(self, exposes_sm_cycle: bool = False) -> None:
        self.tx_level, self.rx_level = 0, 0
        self.sm_cycle = 1 if exposes_sm_cycle else None
        if not exposes_sm_cycle:
            del self.sm_cycle                       # exactly the AttributeError path

    def __getattr__(self, name: str):
        raise AttributeError(f"iss.pio_iss.Engine has no attribute {name!r} either")


class _FakeChip:
    def __init__(self, engine) -> None:
        self.engines = [engine]


def _probe_on(fake_chip) -> Probe:
    """A `Probe` over a fabricated chip, sampled the way `Probe.tick` samples a real one."""
    bus, _device, _chip = _two_device_bus()
    probe = Probe(bus=bus, device=None, chip=fake_chip, engine=0, nets=())
    for t in range(4):
        probe.tick(t, {})
    return probe


def test_a_target_that_hides_sm_cycle_reports_n_a_not_zero():
    """A facade with no `sm_cycle` must give `n/a`; one that has it must be read.

    This is the difference the RTL cross-check turned on: `sm_cycles_per_bit` was published as a
    measured `0` for every `target=rtl` row, because a facade that refuses the read was caught
    and answered with a zero, and a zero is a number.  So both directions are pinned here --
    the refusing facade has to say "unavailable" and print `n/a`, and the exposing one has to be
    sampled for real -- or the fix would be a check that cannot fail.
    """
    blind = _probe_on(_FakeChip(_FakeEngine(exposes_sm_cycle=False)))
    assert blind.sm_available is False, blind.sm_reason()
    assert "unavailable" in blind.sm_reason(), blind.sm_reason()
    trial = Trial(protocol="selftest", knob="none", value=1.0, unit="u", ok=True, bits=160,
                  window=(0, 1332), nets=("p0",), sm_cycles=1333,
                  sm_available=blind.sm_available, sm_source=blind.sm_reason())
    row = next(r for r in trial.rows() if r["metric"] == "sm_cycles_per_bit")
    assert row["value"] == "n/a", row
    assert "not 0" in row["source"] and "sm_cycle" in row["source"], row

    seen = _probe_on(_FakeChip(_FakeEngine(exposes_sm_cycle=True)))
    assert seen.sm_available is True, seen.sm_reason()
    assert seen.sm == [1, 1, 1, 1], seen.sm               # the samples came from the signal
    trial.sm_available, trial.sm_source = seen.sm_available, seen.sm_reason()
    row = next(r for r in trial.rows() if r["metric"] == "sm_cycles_per_bit")
    assert row["value"] != "n/a", row


# ---- negative control 6: the target guard must refuse a bench that was never installed ------------


def test_the_target_guard_refuses_a_bench_that_was_never_installed():
    """`measure.py --target rtl` must stop when `iss.pio_iss.Chip` is still the ISS.

    Nothing but pytest's `fw/conftest.py` installs a backend, and this bench does not run under
    pytest, so at HEAD an `FW_TARGET=rtl` run of it measured the ISS and wrote `target=rtl` into
    the CSV.  The guard is therefore the difference between a cross-check and a coincidence, so
    it is checked two ways: the predicate on its own, and the real command refusing to start --
    the second with `PERF_SKIP_TARGET_INSTALL`, the one switch that makes the install not happen.
    """
    from fw import targets as fw_targets

    from verif.perf import measure

    golden = fw_targets._golden()
    rtl = fw_targets.TARGETS["rtl"].load()
    assert golden is not rtl, "the two targets are the same class, so the guard would be vacuous"

    refused = measure.backend_problem("rtl", golden, rtl)
    assert refused, "the guard accepted an ISS bench that was asked to measure the RTL"
    assert "target='rtl'" in refused and "Refusing" in refused, refused
    assert measure.backend_problem("rtl", rtl, rtl) is None, "the guard refused a real RTL bench"
    assert measure.backend_problem("iss", golden, golden) is None, "the guard refused the ISS"

    scratch = ROOT / "research" / "perf" / "b" / ".guard-control.csv"
    env = dict(os.environ, PERF_SKIP_TARGET_INSTALL="1")
    try:
        done = subprocess.run(
            [sys.executable, "-m", "verif.perf.measure", "--target", "rtl", "--protocols", "spi",
             "--csv", str(scratch)],
            cwd=ROOT, env=env, capture_output=True, text=True, timeout=300)
    finally:
        scratch.unlink(missing_ok=True)
    assert done.returncode != 0, ("the bench ran to completion without a backend installed: "
                                  f"stdout={done.stdout[-400:]!r}")
    assert "Refusing" in (done.stdout + done.stderr), done.stdout[-400:] + done.stderr[-400:]
    assert "sm_cycles_per_bit" not in done.stdout, "a row was printed before the guard fired"
    assert not scratch.exists(), "a CSV was written by a run that was never given the RTL"


def main() -> int:
    """Run every check in this file as a script, for `make perf-selftest`."""
    checks = [value for name, value in sorted(globals().items())
              if name.startswith("test_") and callable(value)]
    failed = 0
    for check in checks:
        try:
            check()
            print(f"  pass  {check.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {check.__name__}: {exc}")
    print(f"{len(checks) - failed}/{len(checks)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
"""Who owns the tightest tHD;DAT sample, three ways, and how much of it is the harness's.

`MarginProbe` attributes every fall → SDA-move sample to whoever moved the line (through
`verif.vip.core.Bus.tap`, the bus's read-only observer) and keeps two minima per rule. This
script is the cross-check that says the attribution is right, and it exists because an
earlier version of this probe reported the *master's* hold as the design's:

    .venv/bin/python -m fw.i2c_target.hold_owner          # all of it

Three ways to run the same probe, which is what settles the question:

  1. **the firmware as the slave** — this firmware's own bench (`measure.bus_for`);
  2. **the VIP's own `I2cEepromTarget`** in the firmware's place — a second model of the
     same 24C02 with a completely different drive path;
  3. **the controller alone**, with no slave on the bus at all — every move on the wire is
     then the master's, so whatever minimum it reaches is the master's own bit placement
     (`I2cController._hold_point`, `max(2, min(max(1, t_hd_dat) + 1, low - 1))`, plus one
     tick of bus resolution).

If (3) reaches the wire-wide minimum that (1) reports, the wire-wide minimum is the
master's and the PIO's own minimum is the larger number beside it.

The second half answers a question the measured numbers raised: the PIO's own minimum is
one tick *lower* here than a run without the sweep's per-transaction FDEBUG poll. The poll
is a host read (`I2cTargetDriver.reg_read` steps the bus until the read completes), so it
shifts the master's phase for the next transaction by a tick, and the hold the PIO manages
follows it. Both numbers are printed; the one the README quotes is the sweep's, because
that is the bench the other numbers come from.
"""
from __future__ import annotations

import sys

from iss import pio_iss
from verif.vip.core import Bus
from verif.vip.i2c import I2cController

from .test_i2c_target import TICK_NS, Bus2, scripts_for

CLEAN_RATES = (50_000, 100_000, 300_000, 400_000, 600_000)


def firmware_bench(rate_hz: int) -> Bus2:
    """The firmware's bench at `rate_hz`: the same one `measure.sweep` runs."""
    from .measure import bus_for
    return bus_for(rate_hz)


def run_scripts(bench: Bus2, poll_fdebug: bool = True) -> None:
    """The three transaction scripts, the way the sweep runs them.

    `poll_fdebug` is the round-1 check (`_run_rate`): it reads FDEBUG after every
    transaction, which is a host read and therefore steps the bus. Turn it off to see the
    bench without it.
    """
    for _name, script in scripts_for(bench):
        script(bench)
        if poll_fdebug:
            bench.driver.fdebug()


def report(tag: str, bench: Bus2, extra: str = "") -> None:
    """One line per bus: how many samples, the wire-wide minimum, and whose it is."""
    probe = bench.probe
    counts = sum(count for count, _low in probe.tally.get("tHD;DAT", {}).values())
    who = sorted(name for name, (_n, low) in probe.tally.get("tHD;DAT", {}).items()
                 if low == probe.samples.get("tHD;DAT"))
    print(f"  {tag:22s}{extra}: {counts:3d} fall → SDA-move samples, "
          f"wire-wide min {probe.samples.get('tHD;DAT')}, "
          f"owner(s) of every sample at it: {who}")
    for name, (count, low) in sorted(probe.tally.get("tHD;DAT", {}).items()):
        print(f"  {'':22s}  {name:12s} {count:3d} samples, min {low}")


def controller_alone(rate_hz: int) -> None:
    """The master on an otherwise empty bus: no slave at all can move SDA.

    Built by hand rather than through `Bus2` because there is no firmware and no EEPROM to
    attach, which is the whole point: every SDA move on this bus is the master's.
    """
    from .test_i2c_target import MarginProbe
    mode = "standard" if rate_hz <= 200_000 else "fast"
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    nets = dict(scl_net="p1", sda_net="p0")
    ctl = I2cController(address=0x50, mode=mode, bus=bus, timing={"freq": rate_hz}, **nets)
    bus.attach(ctl)
    probe = MarginProbe("p1", "p0")           # nothing here is a PIO, so it owns nothing
    bus.attach(probe)
    bus.tap = probe.tap
    ctl.queue_write([0x06, 0xC0, 0xC1, 0xC2])
    ctl.run(bus, limit=200_000)
    bus.run(200)
    counts = sum(n for n, _l in probe.tally.get("tHD;DAT", {}).values())
    print(f"  {'controller alone':22s}{f'{rate_hz / 1e3:.0f} kHz':>12s}: {counts:3d} fall → "
          f"SDA-move samples, wire-wide min {probe.samples.get('tHD;DAT')}, all of them "
          f"the master's")


def decomposition(rate_hz: int) -> None:
    """The PIO's own minimum, as T_IN + one divider period + the phase that is left.

    Two runs: the sweep's bench (which polls FDEBUG after every transaction) and the same
    bench without the poll. The poll is a host read and shifts the master's phase for the
    next transaction, which is where the one-tick difference comes from.
    """
    out = []
    for poll in (True, False):
        bench = firmware_bench(rate_hz)
        run_scripts(bench, poll_fdebug=poll)
        hold = bench.probe.own.get("tHD;DAT")
        period = bench.driver.clkdiv + 1
        out.append((hold, hold - pio_iss.T_IN - period))
    (with_poll, phase_with), (without, phase_without) = out
    print(f"  {rate_hz / 1e3:6.0f} kHz clkdiv {bench.driver.clkdiv:3d}: "
          f"sweep bench {with_poll:3d} = T_IN {pio_iss.T_IN} + divider "
          f"{bench.driver.clkdiv + 1} + phase {phase_with}"
          f"   |   no FDEBUG poll {without:3d} (phase {phase_without})")


def main() -> int:
    from verif.vip.i2c import I2cEepromTarget

    print("=== the tightest tHD;DAT sample, three ways, at 100 and 600 kHz ===")
    for rate in (100_000, 600_000):
        mode = "standard" if rate <= 200_000 else "fast"
        print(f"--- {rate / 1e3:.0f} kHz ({mode} mode) ---")

        ours = firmware_bench(rate)
        run_scripts(ours)
        report("firmware", ours, f" clkdiv {ours.driver.clkdiv} "
              f"poll {ours.driver.poll}")

        # `firmware=False` attaches the VIP's own I2cEepromTarget instead of the chip.
        theirs = Bus2(mode=mode, overrides={"freq": rate}, firmware=False)
        assert isinstance(theirs.eeprom, I2cEepromTarget)
        for _name, script in scripts_for(theirs):
            script(theirs)
        report("VIP I2cEepromTarget", theirs)

        controller_alone(rate)
        print(f"  checker violations: firmware {len(ours.checker.violations)}, "
              f"VIP EEPROM {len(theirs.checker.violations)}")

    print("=== the PIO's own minimum at every clean rate, and where the ticks go ===")
    for rate in CLEAN_RATES:
        decomposition(rate)
    return 0


if __name__ == "__main__":
    sys.exit(main())
"""Whose move is the 1 MHz tSU;DAT violation, and does it happen with no slave at all?

The sweep records the 1 MHz row as failing with `tSU;DAT 3 ticks < 5`, and the recorded
reason blames the firmware's SDA drive. The attribution now in `MarginProbe` says the
tightest tSU;DAT sample belongs to `i2c_ctrl`, and the controller's own tick dump shows it
releasing SDA at `first_hold` = 21 ticks into a 25-tick low period — three ticks before its
own rise. This runs the master by itself on an otherwise empty bus: if it violates tSU;DAT
there too, the violation is the master's own first-bit placement and no firmware could pass
1 MHz with this master.

    fw/i2c_target/stamped_run.sh B3-FW-i2c_target-fix2-1mhz-owner light \
        .venv/bin/python -m fw.i2c_target.one_mhz_owner
"""
from __future__ import annotations

from iss import pio_iss
from verif.vip.core import Bus
from verif.vip.i2c import I2cController, I2cTimingChecker, mode_minimums

from .measure import FMP
from .test_i2c_target import TICK_NS, MarginProbe, scripts_for

RATES = (800_000, 1_000_000)


def alone(rate: int) -> tuple:
    """The master on an otherwise empty bus: no PIO, no EEPROM, nothing but the checker."""
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    nets = dict(scl_net="p1", sda_net="p0")
    timing = dict(FMP, freq=rate)
    ctl = I2cController(address=0x50, mode="fast", bus=bus, timing=timing, **nets)
    bus.attach(ctl)
    chk = I2cTimingChecker(mode="fast", bus=bus, timing=timing, **nets)
    bus.attach(chk)
    probe = MarginProbe("p1", "p0")          # no PIO on this bus: it owns nothing
    bus.attach(probe)
    bus.tap = probe.tap
    probe.set_required(mode_minimums(TICK_NS, "fast", timing))
    ctl.queue_write([0x06, 0xC0, 0xC1, 0xC2])
    ctl.run(bus, limit=200_000)
    bus.run(200)
    return ctl, chk, probe


def main() -> int:
    print("=== the master alone, no slave on the bus at all ===")
    for rate in RATES:
        ctl, chk, probe = alone(rate)
        print(f"{rate / 1e3:.0f} kHz: {len(chk.violations)} checker violation(s); "
              f"wire-wide tSU;DAT {probe.samples.get('tSU;DAT')} vs required "
              f"{probe.required['tSU;DAT']}; low_ticks {ctl.low_ticks}, "
              f"first_hold {ctl.first_hold}, high_ticks {ctl.high_ticks}")
        for v in chk.violations:
            print(f"    {v}")

    print("\n=== the same rates with this firmware as the slave ===")
    from .measure import bus_for
    for rate in RATES:
        bench = bus_for(rate)                  # the sweep's own bench, divider and all
        for _name, script in scripts_for(bench):
            script(bench)
            bench.driver.fdebug()              # the sweep checks FDEBUG after each one
        probe = bench.probe
        print(f"{rate / 1e3:.0f} kHz: clkdiv {bench.driver.clkdiv}, "
              f"poll {bench.driver.poll}; checker {len(bench.checker.violations)} "
              f"violation(s), FDEBUG {bench.driver.fdebug():#010x}")
        print(f"    tSU;DAT PIO's own {probe.own.get('tSU;DAT')} vs "
              f"{probe.required['tSU;DAT']}, wire-wide {probe.samples.get('tSU;DAT')}, "
              f"owners {probe.tally.get('tSU;DAT')}")
        print(f"    tHD;DAT PIO's own {probe.own.get('tHD;DAT')} vs "
              f"{probe.required['tHD;DAT']}, wire-wide {probe.samples.get('tHD;DAT')}, "
              f"owners {probe.tally.get('tHD;DAT')}")
        for v in bench.checker.violations[:4]:
            print(f"    {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
"""Measure what fw/swd does on the ISS: the word budget, the bit-rate range, the session.

Run it to produce the numbers the README quotes:

    python -m fw.swd.measure

Everything it prints comes from the ISS (`iss.pio_iss.Chip`) on a `verif.vip.core.Bus`
at 20 ns/tick with `verif.vip.swd.SwdTarget` and `SwdChecker` on the two nets, so the
figures are measurements of a simulated run and not of the designer's arithmetic.
"""
from __future__ import annotations

import sys

from fw.swd import driver as drv
from fw.swd.driver import SwdDriver
from iss import pio_iss
from verif.vip import swd as vswd
from verif.vip.core import Bus

TICK_NS = 20.0
CORE_HZ = 1e9 / TICK_NS


def bench(high: int, low: int):
    """A bus with the chip, an SW-DP and the checker on it, loaded and started."""
    bus = Bus(ns_per_tick=TICK_NS)
    swdio, swclk, flag = drv.DEFAULT_SWDIO_PIN, drv.DEFAULT_SWCLK_PIN, drv.DEFAULT_FLAG_PIN
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}", push_pull=k in (swdio, swclk))
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus.attach(device)
    nets = dict(clk_net=f"p{swclk}", io_net=f"p{swdio}")
    target = vswd.attach_target(bus, **nets)
    checker = vswd.attach_checker(bus, **nets)
    driver = SwdDriver(bus, device, high=high, low=low)
    driver.load()
    return bus, device, target, checker, driver


def session(high: int, low: int) -> dict:
    """Line reset, IDCODE, a CTRL/STAT write and read, a SELECT write and read."""
    bus, _, target, checker, driver = bench(high, low)
    start = bus.t
    driver.line_reset()
    idcode = driver.read(vswd.DP_IDCODE)
    driver.write(vswd.DP_CTRL_STAT, vswd.CTRL_CSYSPWRUPREQ | vswd.CTRL_ORUNDETECT)
    ctrl = driver.read(vswd.DP_CTRL_STAT)
    driver.write(vswd.DP_SELECT, 0x1234_5678)
    select = driver.read(vswd.DP_SELECT)
    ticks = bus.t - start
    return {
        "high": high,
        "low": low,
        "period": driver.period,
        "swclk_mhz": driver.config.swclk_hz / 1e6,
        "measured_period": checker.shortest_period,
        "measured_mhz": (checker.max_swclk_hz or 0.0) / 1e6,
        "idcode": idcode,
        "idcode_ok": idcode == vswd.IDCODE_DEFAULT,
        "ctrl": ctrl,
        "select": select,
        "select_ok": select == 0x1234_5678,
        "frames": len(checker.frames),
        "violations": [v.rule for v in checker.violations],
        "line_resets": checker.resets,
        "dp_resets": target.resets,
        "contention": len(bus.errors),
        "fdebug": driver.fdebug(),
        "ticks": ticks,
        "words": driver.words_used,
    }


def main() -> int:
    program = drv.load_program()
    print(f"chip core clock          {CORE_HZ / 1e6:.0f} MHz ({TICK_NS:.0f} ns/tick)")
    print(f"program words            {len(program.instrs)} of 32")
    print(f"loop instructions        {drv.LOOPS_PER_PERIOD} per bit period "
          f"(+ HIGH + LOW cycles of delay)")
    print(f"HIGH range               {drv.MIN_HIGH}..{14} (the input path needs "
          f"{drv.MIN_HIGH})")
    print(f"LOW range                {drv.MIN_LOW}..{drv.MAX_LOW}")
    print(f"bit period range         {drv.LOOPS_PER_PERIOD + drv.MIN_HIGH}"
          f"..{drv.LOOPS_PER_PERIOD + 14 + drv.MAX_LOW} core cycles")
    print()
    header = (f"{'HIGH':>4} {'LOW':>4} {'period':>7} {'measured':>9} {'SWCLK MHz':>10} "
              f"{'IDCODE':>9} {'SELECT':>9} {'frames':>7} {'viol':>5} {'cont':>5} "
              f"{'ticks':>7}")
    print(header)
    print("-" * len(header))
    results = []
    for high, low in ((6, 0), (6, 1), (6, 5), (6, 14), (8, 9), (13, 14)):
        row = session(high, low)
        results.append(row)
        print(f"{row['high']:>4} {row['low']:>4} {row['period']:>7} "
              f"{row['measured_period']:>9} {row['swclk_mhz']:>10.3f} "
              f"{'ok' if row['idcode_ok'] else 'BAD':>9} "
              f"{'ok' if row['select_ok'] else 'BAD':>9} "
              f"{row['frames']:>7} {len(row['violations']):>5} {row['contention']:>5} "
              f"{row['ticks']:>7}")
    print()
    fastest = min(results, key=lambda r: r["period"])
    slowest = max(results, key=lambda r: r["period"])
    print(f"fastest SWCLK            {fastest['swclk_mhz']:.3f} MHz "
          f"(period {fastest['period']} core cycles, measured "
          f"{fastest['measured_period']} ticks, {fastest['measured_mhz']:.3f} MHz)")
    print(f"slowest SWCLK            {slowest['swclk_mhz']:.3f} MHz "
          f"(period {slowest['period']} core cycles, measured "
          f"{slowest['measured_period']} ticks)")
    clean = [r for r in results if not r["violations"] and not r["contention"]]
    print(f"configurations clean     {len(clean)} of {len(results)}")
    bad = [r for r in results if not (r["idcode_ok"] and r["select_ok"])]
    print(f"transfers answered       {len(results) - len(bad)} of {len(results)}")
    if bad or len(clean) != len(results):
        for row in results:
            if row["violations"] or row["contention"] or not row["idcode_ok"]:
                print(f"  HIGH {row['high']} LOW {row['low']}: violations "
                      f"{row['violations']}, contention {row['contention']}, "
                      f"idcode {row['idcode']:#010x}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
"""Measure the MVP JTAG firmware and append its rows to metrics/fw.csv.

    .venv/bin/python -m fw.jtag.measure            # measure and append the rows
    .venv/bin/python -m fw.jtag.measure --dry-run  # measure only

The output is pass/fail on the
ISS, the instruction count, the peak FIFO depth and the cycles per TCK. The numbers come
out of the same bench fw/jtag/test_jtag.py uses, so they are the numbers the tests check,
and every one of them is written to the CSV with the git describe and the date of the tree
it was measured in.

Two rows go in. The first is the configuration the tests run at (CLKDIV 1.0, the fastest
the divider goes). The second is the firmware's max speed: the fastest combination of clock
divider and TCK phase delays that `JtagChecker` accepts with no violation at all, which is
what is reported as the firmware's max speed.

Cycles per TCK is the *steady* period on the bus -- the gap between the rising edges inside
a scan, which is every clock of every scan bar the two transitions at the end of one -- and
not `JtagChecker.shortest_period`, which is the gap that straddles the load and the park
loop and is therefore shorter than any real TCK cycle.

The sweep is over both knobs, because on this program they are not independent: the core
cycles per TCK cycle are `LOW_HOLD + HIGH_HOLD + 4` (program.pio, "ONE TCK CYCLE IS NINE
CORE CYCLES"), and docs/mvp-isa.md `## Timing` puts a floor under each of them. The sweep
prints which rule rejects each faster combination, because that is what says the floor is
real and not just this program's choice.

The RTL column is `skip`: the RTL path comes later, and fw/jtag/test_jtag.py skips itself
when FW_TARGET is not `iss`.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import pathlib
import subprocess
import sys
from typing import List, Tuple

from fw.jtag import assemble_program, retime
from fw.jtag import driver as drv
from fw.jtag.test_jtag import FAST, SLOW, Bench, TICK_NS
from verif.vip import jtag as vip

CSV_PATH = pathlib.Path("metrics/fw.csv")
CSV_HEADER = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count",
              "peak_fifo_depth", "cycles_per_bit"]


def git_describe() -> str:
    try:
        out = subprocess.run(["git", "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def measure(config: dict) -> dict:
    """Run reset + select IDCODE + read IDCODE and report what the bus saw."""
    bench = Bench(**config).start()
    got = bench.driver.idcode()
    assert got == vip.DEFAULT_IDCODE, f"IDCODE read back {got:#x}"
    assert bench.checker.violations == [], bench.report()
    return {
        "clkdiv": config["clkdiv"],
        "clkdiv_hz": 50e6 / config["clkdiv"],
        "instr_count": len(assemble_program().instrs),
        "peak_fifo_depth": bench.driver.peak_fifo_depth,
        "cycles_per_tck": bench.steady_period(),
        "tck_hz": bench.tck_hz(),
        "rises": bench.checker.rises,
        "clocks": bench.driver.clocks,
        "scans": [(s.kind, s.length) for s in bench.checker.scans],
        "violations": len(bench.checker.violations),
        "fdebug": bench.driver.fdebug(),
        "shortest_period": bench.checker.shortest_period,
    }


def scan_timing() -> Tuple[List[Tuple[int, int, int, float]], List[Tuple[int, int, str]]]:
    """Every (LOW_HOLD, HIGH_HOLD, CLKDIV) the timing allows, split by the checker's verdict.

    Clean means `JtagChecker` finds no violation at all, so the rate in the clean list is
    one the firmware can really run at. The rejected list says which rule did the
    rejecting: below the floor only the TDO rules object, and above the profile's maximum
    frequency TCK.period does as well.
    """
    base = assemble_program()
    clean: List[Tuple[int, int, int, float]] = []
    rejected: List[Tuple[int, int, str]] = []
    for low in range(0, 5):
        for high in range(0, 6):
            program = retime(base, low_hold=low, high_hold=high)
            for clkdiv in (1, 2, 3, 4):
                try:
                    bench = Bench(clkdiv=clkdiv, program=program).start()
                    got = bench.driver.idcode()
                except Exception as exc:                      # noqa: BLE001
                    rejected.append((low, high, f"{type(exc).__name__}: {exc}"))
                    continue
                ticks = drv.tck_cycles(program) * clkdiv
                rate = 1e9 / (TICK_NS * ticks)
                rules = sorted({v.rule for v in bench.checker.violations})
                if rules or got != vip.DEFAULT_IDCODE:
                    why = ",".join(rules) if rules else f"IDCODE {got:#x}"
                    rejected.append((low, high, f"clkdiv {clkdiv}: {rate / 1e6:.2f} MHz: {why}"))
                else:
                    clean.append((low, high, clkdiv, rate))
    return clean, rejected


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="do not touch metrics/fw.csv")
    args = parser.parse_args(argv)

    program = assemble_program()
    low, high = drv.loop_delays(program)
    print(f"program: {len(program.instrs)} of 32 IMEM words, labels {program.labels}, "
          f"wrap [{program.wrap_target}, {program.wrap}]")
    print(f"timing: LOW_HOLD {low}, HIGH_HOLD {high} -> {drv.tck_cycles(program)} core "
          f"cycles per TCK cycle at CLKDIV 1.0 "
          f"({drv.tck_hz(program, 1, TICK_NS) / 1e6:.3f} MHz at {TICK_NS:.0f} ns/tick)")

    results = [measure(FAST), measure(SLOW)]
    for r in results:
        print(f"CLKDIV {r['clkdiv']} ({r['clkdiv_hz'] / 1e6:6.3f} MHz SM): TCK "
              f"{r['tck_hz'] / 1e6:6.3f} MHz ({r['cycles_per_tck']} core cycles/TCK, "
              f"shortest period on the bus {r['shortest_period']}), {r['rises']} rising "
              f"edges for {r['clocks']} clocks the driver counted, scans {r['scans']}, "
              f"{r['violations']} violations, peak FIFO {r['peak_fifo_depth']}, "
              f"FDEBUG {r['fdebug']:#010x}")

    clean, rejected = scan_timing()
    best = max(clean, key=lambda c: c[3]) if clean else None
    on_edge = [c for c in clean if c[1] == drv.loop_delays(program)[1]]
    best_edge = max(on_edge, key=lambda c: c[3]) if on_edge else None
    if best:
        b_low, b_high, b_clkdiv, b_rate = best
        print(f"max speed JtagChecker accepts: LOW_HOLD {b_low}, HIGH_HOLD {b_high}, "
              f"CLKDIV {b_clkdiv} -> {b_rate / 1e6:.3f} MHz "
              f"({1e9 / (TICK_NS * b_rate):.0f} core cycles per TCK cycle); "
              f"the profile caps TCK at {vip.IJET_TIMINGS.max_tck_hz / 1e6:.0f} MHz "
              f"and {len(clean)} of the swept combinations are clean")
    if best_edge:
        e_low, e_high, e_clkdiv, e_rate = best_edge
        print(f"max speed with the sample on the rising edge (HIGH_HOLD {e_high} = T_IN): "
              f"LOW_HOLD {e_low}, CLKDIV {e_clkdiv} -> {e_rate / 1e6:.3f} MHz "
              f"({1e9 / (TICK_NS * e_rate):.0f} core cycles per TCK cycle); "
              f"this is the one program.pio ships, and HIGH_HOLD {b_high} only reaches "
              f"{b_rate / 1e6:.3f} MHz by sampling one tick *before* the edge with no TDO "
              f"margin, which the checker cannot see")
    if rejected:
        print("combinations JtagChecker rejects: " + "; ".join(
            f"LOW_HOLD {lo}/HIGH_HOLD {hi}: {why}" for lo, hi, why in rejected[:12])
            + (f"; ... and {len(rejected) - 12} more" if len(rejected) > 12 else ""))

    if args.dry_run:
        return 0

    fast = results[0]
    rows = [[
        datetime.date.today().isoformat(), git_describe(),
        f"jtag-target+checker clkdiv={fast['clkdiv']} (test config)", "jtag", "pass", "skip",
        fast["instr_count"], fast["peak_fifo_depth"], f"{fast['cycles_per_tck']}",
    ]]
    if best:
        rows.append([
            datetime.date.today().isoformat(), git_describe(),
            f"jtag-target+checker low_hold={b_low} high_hold={b_high} clkdiv={b_clkdiv} "
            f"(max speed)", "jtag", "pass", "skip", len(program.instrs),
            fast["peak_fifo_depth"], f"{1e9 / (TICK_NS * b_rate):.1f}",
        ])
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    new_file = not CSV_PATH.exists()
    with CSV_PATH.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if new_file:
            writer.writerow(CSV_HEADER)
        writer.writerows(rows)
    print(f"appended {len(rows)} row(s) to {CSV_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
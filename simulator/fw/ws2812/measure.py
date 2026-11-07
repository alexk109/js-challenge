"""Measure the MVP WS2812 firmware and append its rows to metrics/fw.csv.

    .venv/bin/python -m fw.ws2812.measure             # measure, print, append the rows
    .venv/bin/python -m fw.ws2812.measure --dry-run   # measure and print only

The numbers come out of the same bench `fw/ws2812/test_ws2812.py` uses, so they are the
numbers the tests check, and every one of them is written to the CSV with the `git describe`
and the date of the tree it was measured in.

Four rows go in: the two programs at CLKDIV 1.0, the nominal one with the fractional divider
the tests run, and -- as the "max speed" row -- the fastest wire the ISA can drive a WS2812
line at, which is `ws2812_fast`.

`cycles_per_bit` is the shortest bit period the checker measured on the wire, in core cycles
(ticks), and not the derived one: the bit that straddles a pixel boundary is one `pull block`
longer than the rest, so the measured minimum is the derived grid minus nothing and the
measured maximum is one tick more.  The printed table carries the four high and low times as
measured, next to the windows `verif/vip/ws2812.py` enforces, because those are what says
whether a stream is legal rather than merely well-formed.

The RTL column is `skip`: the RTL path comes later, and `fw/ws2812/test_ws2812.py` skips
itself when `FW_TARGET` is not `iss`.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import pathlib
import subprocess
import sys

from fw.ws2812.driver import CORE_HZ, NS_PER_TICK, T_RESET_NS, ticks_of
from fw.ws2812.test_ws2812 import CASES, Case, build, run
from verif.vip.ws2812 import ws2812_timings

CSV_PATH = pathlib.Path("metrics/fw.csv")
CSV_HEADER = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count",
              "peak_fifo_depth", "cycles_per_bit"]
TICK_NS = NS_PER_TICK


def git_describe() -> str:
    try:
        out = subprocess.run(["git", "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def measure(case: Case) -> dict:
    """Run one case end to end and report what the wire did, in ticks and in bit/s.

    The high and low times are per bit value, measured between the edges the checker saw, and
    the last bit of each frame is left out because its low time is the reset code rather than a
    bit time (`Ws2812TimingChecker._reset_code`).
    """
    bench = run(build(case), wait=case.chain)
    limits = ws2812_timings(NS_PER_TICK)
    left = bench.chk.finalize()
    bits = [b for b in bench.chk.bits if b.low_ticks]
    zeros = [b for b in bits if b.value == 0]
    ones = [b for b in bits if b.value == 1]

    def span(events, attr: str) -> tuple[int, int]:
        values = [getattr(e, attr) for e in events]
        return (min(values), max(values)) if values else (0, 0)

    drv = bench.drv
    return {
        "case": case.name,
        "program": case.program,
        "clkdiv": f"1 + {case.frac}/256",
        "pixels": case.pixels,
        "engine": case.engine,
        "pin": case.pin,
        "pullup": case.pullup,
        "instr_count": drv.words_used,
        "peak_fifo_depth": drv.peak_tx_level,
        "bit_cycles": drv.bit_cycles,
        "shortest_period": min(bench.chk.periods),
        "longest_period": max(bench.chk.periods),
        "t0h": span(zeros, "high_ticks"),
        "t1h": span(ones, "high_ticks"),
        "t0l": span(zeros, "low_ticks"),
        "t1l": span(ones, "low_ticks"),
        "bit_rate_hz": bench.chk.bit_rate_hz,
        "bits": len(bench.chk.bits),
        "frames": len(bench.chk.frames),
        "resets": len(bench.chk.resets),
        "violations": len(bench.chk.violations) + len(left),
        "latched": [p.frames for p in bench.chain],
        "fdebug": drv.fdebug(),
        "windows": {
            "T0H": (limits["t0h_min"], limits["t0h_max"]),
            "T1H": (limits["t1h_min"], limits["t1h_max"]),
            "T0L": (0, limits["t0l_max"]),
            "T1L": (0, limits["t1l_max"]),
            "TBIT": (limits["t_bit_min"], limits["t_bit_max"]),
        },
        "reset_min_ticks": ticks_of(T_RESET_NS),
        "ticks": bench.bus.t,
    }

def report(rows: list[dict]) -> None:
    """Print the measured table: the four times against their windows, and the rate."""
    windows = rows[0]["windows"]
    head = (f"{'case':17s} {'prog':13s} {'clkdiv':9s} {'px':>3s} {'words':>5s} {'fifo':>4s} "
            f"{'period':>9s} {'bit rate':>13s} {'viol':>4s}")
    print(head)
    print("-" * len(head))
    for row in rows:
        print(f"{row['case']:17s} {row['program']:13s} {row['clkdiv']:9s} {row['pixels']:3d} "
              f"{row['instr_count']:5d} {row['peak_fifo_depth']:4d} "
              f"{row['shortest_period']:4d}..{row['longest_period']:<4d} "
              f"{row['bit_rate_hz'][0] / 1e3:6.1f}..{row['bit_rate_hz'][1] / 1e3:<6.1f} kbit/s "
              f"{row['violations']:4d}")
    print()
    print(f"windows at {NS_PER_TICK} ns per tick, from verif/vip/ws2812.py "
          f"(ws2812_timings):")
    for name, (lo, hi) in windows.items():
        print(f"  {name:5s} {lo:3d} .. {hi:3d} ticks")
    print(f"  reset {rows[0]['reset_min_ticks']} ticks "
          f"({T_RESET_NS / 1000:.0f} us, page 3 \"Above 50us\")")
    print()
    print(f"{'case':17s} {'T0H':>11s} {'T1H':>11s} {'T0L':>11s} {'T1L':>11s} "
          f"{'bits':>6s} {'frames':>6s} {'resets':>6s} {'latched':>10s} {'FDEBUG':>10s}")
    for row in rows:
        cells = " ".join(f"{lo:4d}..{hi:<4d}" for lo, hi in
                         (row["t0h"], row["t1h"], row["t0l"], row["t1l"]))
        print(f"{row['case']:17s} {cells} {row['bits']:6d} {row['frames']:6d} "
              f"{row['resets']:6d} {str(row['latched']):>10s} {row['fdebug']:#010x}")


def append_csv(rows: list[dict]) -> int:
    """Append one row per measurement to metrics/fw.csv, stamped with the tree it was measured in."""
    rows_out = []
    for row in rows:
        rows_out.append([
            datetime.date.today().isoformat(), git_describe(),
            f"fw/ws2812 ({row['program']}, clkdiv={row['clkdiv']}, {row['pixels']}px, "
            f"engine {row['engine']}, pin {row['pin']})",
            "ws2812", "pass" if row["violations"] == 0 else "fail", "skip",
            row["instr_count"], row["peak_fifo_depth"], row["shortest_period"],
        ])
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    new_file = not CSV_PATH.exists()
    with CSV_PATH.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if new_file:
            writer.writeheader(CSV_HEADER)
        writer.writerows(rows_out)
    return len(rows_out)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fw.ws2812.measure",
        description="Measure the MVP WS2812 firmware and append metrics/fw.csv rows.",
    )
    parser.add_argument("--dry-run", action="store_true", help="measure and print, write no CSV")
    args = parser.parse_args(argv)

    rows = [measure(case) for case in CASES]
    report(rows)
    worst = max(row["violations"] for row in rows)
    print()
    print(f"chip role: controller (it drives the data line; the pixels are the targets)")
    words = sorted({(row["program"], row["instr_count"]) for row in rows})
    print(f"words used of 32: " + ", ".join(f"{name}={count}" for name, count in words))
    fastest = max(1e9 / (row["shortest_period"] * TICK_NS) for row in rows)
    slowest = min(1e9 / (row["longest_period"] * TICK_NS) for row in rows)
    print(f"wire speed measured on the ISS at {CORE_HZ / 1e6:.0f} MHz: "
          f"{slowest / 1e3:.1f}..{fastest / 1e3:.1f} kbit/s "
          f"({fastest / 1e6:.3f} Mbit/s at best)")
    print(f"violations across {len(rows)} configurations: {worst}")
    if args.dry_run:
        print("dry run: metrics/fw.csv not written")
        return 0 if worst == 0 else 1
    written = append_csv(rows)
    print(f"appended {written} row(s) to {CSV_PATH}")
    return 0 if worst == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

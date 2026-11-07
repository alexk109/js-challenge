"""Measure the MVP I2C firmware and append its rows to metrics/fw.csv.

    .venv/bin/python -m fw.i2c.measure            # measure and append the rows
    .venv/bin/python -m fw.i2c.measure --dry-run  # measure only

The output is pass/fail on the
ISS, on the RTL, the instruction count, the peak FIFO depth and the cycles per bit. The
numbers come out of the same bench fw/i2c/test_i2c.py uses, so they are the numbers the
tests check, and every one of them is written to the CSV with the git describe and the
date of the tree it was measured in.

Two rows go in. The first is the configuration the tests run at (fast mode, CLKDIV 14).
The second is the firmware's max speed: the fastest CLKDIV the fast-mode checker accepts
with no violation at all, which is what fw/i2c/test_i2c.py records as MAX_CLEAN_* and
what is reported as the firmware's max speed. Since I2cTimingChecker checks fSCL, that divider is also the
fastest one inside the 400 kHz that fast mode is named for.

Cycles per bit is the *shortest* SCL period on the bus, which is the rate the spec
limits, and not the average over the nine clocks of a byte that the byte timestamps give:
the program runs the byte-to-byte bit short, so the average (146.2 cycles at CLKDIV 14)
understates the rate (357.1 kbit/s rather than 341.9).

The RTL column is `skip`: this module measures on whatever chip `FW_TARGET` names, and the RTL
path came later, so the rows below were measured on the ISS. fw/i2c/test_i2c.py no longer skips
itself on any target -- the registry (`fw/targets.py`) is the only thing that reads `FW_TARGET`.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import pathlib
import subprocess
import sys
from typing import List, Tuple

from fw.i2c import assemble_program
from fw.i2c.test_i2c import (FAST, MAX_CLEAN_BIT_RATE_HZ, MAX_CLEAN_CLK_DIV, STANDARD,
                             Bench, TICK_NS)
from verif.vip.i2c import I2C_MODES

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


def measure(config: dict, stretch: int) -> dict:
    """Run one write + repeated-START + read and report what the bus saw."""
    bench = Bench(**config).start()
    bench.eeprom.memory[0x08:0x10] = bytes(range(0x10, 0x18))
    acked, read = bench.driver.write_read(0x50, [0x0C, 0xC0], 3)
    assert read == bytes([0x15, 0x16, 0x17]), read
    assert acked == [True, True], acked
    assert bench.checker.violations == [], bench.report()
    return {
        "mode": config["mode"],
        "clkdiv": config["clkdiv"],
        "stretch_ticks": stretch,
        "clkdiv_hz": 50e6 / config["clkdiv"],
        "instr_count": len(assemble_program().instrs),
        "peak_fifo_depth": bench.driver.peak_fifo_depth,
        # The shortest SCL period the checker measured: the rate fSCL limits.
        "cycles_per_bit": bench.checker.shortest_period,
        "bit_rate_hz": bench.checker.max_scl_hz,
        "bytes": bench.checker.bytes,
        "starts": bench.checker.starts,
        "repeated_starts": bench.checker.repeated_starts,
        "stops": bench.checker.stops,
        "violations": len(bench.checker.violations),
        "fdebug": bench.driver.fdebug(),
    }


def scan_fast_mode() -> Tuple[List[Tuple[int, float]], List[Tuple[int, str]]]:
    """Every CLKDIV, split into the clean ones and the ones the checker rejected.

    Clean means the fast-mode `I2cTimingChecker` finds no violation at all, fSCL
    included, so every rate in the clean list is at or under the 400 kHz of fast mode.
    The rejected list says which rules did the rejecting, which is the interesting half:
    below the nominal rate only fSCL objects, and below that the half-period minimums
    start to object as well.
    """
    expect = bytes([0x22, 0x33, 0x44, 0x00])
    clean: List[Tuple[int, float]] = []
    rejected: List[Tuple[int, str]] = []
    for clkdiv in range(1, 33):
        bench = Bench(mode="fast", clkdiv=clkdiv, stretch_ticks=0, address=0x50)
        bench.eeprom.memory[0x10:0x14] = bytes([0x11, 0x22, 0x33, 0x44])
        bench.start()
        try:
            _, read = bench.driver.write_read(0x50, [0x10, 0x00], 4)
        except Exception as exc:
            rejected.append((clkdiv, f"{type(exc).__name__} before a readback"))
            continue
        rate = bench.checker.max_scl_hz or 0.0
        rules = sorted({v.rule for v in bench.checker.violations})
        if rules or read != expect:
            why = ",".join(rules) if rules else f"readback {bytes(read).hex()}"
            rejected.append((clkdiv, f"{rate / 1e3:.0f} kbit/s: {why}"))
        else:
            clean.append((clkdiv, rate))
    return clean, rejected


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="do not touch metrics/fw.csv")
    args = parser.parse_args(argv)

    results = [measure(FAST, FAST["stretch_ticks"]), measure(STANDARD, STANDARD["stretch_ticks"])]
    scan, rejected = scan_fast_mode()
    program = assemble_program()

    print(f"program: {len(program.instrs)} of 32 IMEM words, "
          f"labels {program.labels}, wrap [{program.wrap_target}, {program.wrap}]")
    for r in results:
        print(f"{r['mode']:9s} CLKDIV {r['clkdiv']:3d} ({r['clkdiv_hz'] / 1e6:6.3f} MHz SM), "
              f"stretch {r['stretch_ticks']:2d} ticks: {r['bit_rate_hz'] / 1e3:7.1f} kbit/s "
              f"({r['cycles_per_bit']:.1f} core cycles/bit), {r['bytes']} bytes, "
              f"{r['starts']} START ({r['repeated_starts']} repeated), {r['stops']} STOP, "
              f"{r['violations']} violations, peak FIFO {r['peak_fifo_depth']}, "
              f"FDEBUG {r['fdebug']:#010x}")
    best = min(scan, key=lambda p: p[0]) if scan else None
    if best:
        best_div, best_rate = best
        print(f"fastest CLKDIV with zero fast-mode violations: {best_div} "
              f"({best_rate / 1e3:.1f} kbit/s, {1e9 / (TICK_NS * best_rate):.0f} core "
              f"cycles/bit); clean dividers: "
              + ", ".join(f"{d}:{rate / 1e3:.0f}k" for d, rate in scan))
        recorded = (best_div, best_rate) == (MAX_CLEAN_CLK_DIV, MAX_CLEAN_BIT_RATE_HZ)
        print(f"max speed matches fw/i2c/test_i2c.py MAX_CLEAN_CLK_DIV="
              f"{MAX_CLEAN_CLK_DIV}, MAX_CLEAN_BIT_RATE_HZ={MAX_CLEAN_BIT_RATE_HZ / 1e3:.1f}"
              f" kbit/s: {recorded}"
              + ("" if recorded else "   <-- update the recorded constants"))
        if best_rate > I2C_MODES["fast"].freq:
            print(f"fSCL: {best_rate / 1e3:.1f} kbit/s is over fast mode's "
                  f"{I2C_MODES['fast'].freq / 1e3:.0f} kbit/s")
    if rejected:
        print("dividers the fast-mode checker rejects: "
              + "; ".join(f"CLKDIV {d}: {why}" for d, why in rejected))

    if args.dry_run:
        return 0

    fast = results[0]
    rows = [[
        datetime.date.today().isoformat(), git_describe(),
        f"i2c-eeprom+checker {fast['mode']} clkdiv={fast['clkdiv']} "
        f"stretch={fast['stretch_ticks']}",
        "i2c", "pass", "skip", fast["instr_count"], fast["peak_fifo_depth"],
        f"{fast['cycles_per_bit']:.1f}",
    ]]
    if best:
        # The same firmware at the fastest divider that is still inside fast mode: the
        # cycles_per_bit column is its SCL period, which is the max speed.
        rows.append([
            datetime.date.today().isoformat(), git_describe(),
            f"i2c-eeprom+checker fast clkdiv={best_div} stretch=0 (max speed)",
            "i2c", "pass", "skip", len(program.instrs), fast["peak_fifo_depth"],
            f"{1e9 / (TICK_NS * best_rate):.1f}",
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

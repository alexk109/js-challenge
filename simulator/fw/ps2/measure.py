"""Measure the MVP PS/2 firmware and append its rows to metrics/fw.csv.

    .venv/bin/python -m fw.ps2.measure            # measure and append the rows
    .venv/bin/python -m fw.ps2.measure --dry-run  # measure only

It reports pass/fail on the
ISS, the instruction count, the peak FIFO depth and the cycles per bit. The numbers come out
of the same bench fw/ps2/test_ps2.py uses, so they are the numbers the tests check, and every
one of them goes into the CSV with the git describe and the date of the tree it was measured
in.

Two rows go in. The first is the configuration the tests run at (CLKDIV 100, 15.2 kbit/s).
The second is the firmware's max speed: the fastest CLKDIV the PS/2 checker accepts with no
violation at all, which is what fw/ps2/test_ps2.py records as MAX_CLEAN_CLK_DIV and what the
notes quote. What runs out first is T4's 30 us floor on the *high* phase of the clock,
because a data bit's low phase is a whole SM cycle longer than its high phase.

Cycles per bit is the data bit time (low plus high), the rate the protocol limits. The
checker's own `shortest_period` is one SM cycle shorter than that: the acknowledgement clock,
whose low phase is the loop's JMP and the sampling `in` rather than an OUT and a JMP. Both
numbers are printed below, and the tests check both.

The RTL column is `skip`: this module measures on whatever chip `FW_TARGET` names, and the RTL
path came later, so the rows below were measured on the ISS. fw/ps2/test_ps2.py no longer skips
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

from fw.ps2 import assemble_program
from fw.ps2.driver import legal_clkdiv, ns_to_ticks, timing_of
from fw.ps2.test_ps2 import (FAST, MAX_CLEAN_ACK_PERIOD, MAX_CLEAN_BIT_CYCLES,
                             MAX_CLEAN_BIT_RATE_HZ, MAX_CLEAN_CLK_DIV, SLOW, TICK_NS,
                             Bench)
from fw.ps2.driver import ACK, BUS_FREE_NS, CLK_MAX_HALF_NS, CLK_MIN_HALF_NS, RTS_MIN_NS

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


def measure(config: dict, label: str) -> dict:
    """Run a reset plus one scan code and report what the bus saw.

    The device is on the slow rate, so the clock periods the checker measures are the host's
    own and not the device's.
    """
    bench = Bench(dev_rate="slow", **config).start()
    kbd, driver = bench.dev, bench.driver
    assert driver.command(0xFF, response=1) == bytes([0xAA]), "the reset did not answer 0xAA"
    kbd.press("a")
    assert driver.receive(1) == bytes([kbd.make_code("a")])
    assert bench.checker.violations == [], bench.report()
    timing = timing_of(assemble_program())
    return {
        "label": label,
        "clkdiv": driver.clkdiv,
        "instr_count": timing.words,
        "peak_fifo_depth": driver.peak_fifo_depth,
        "bit_cycles": timing.bit_cycles(driver.clkdiv),
        "bit_rate_hz": timing.line_rate_hz(driver.clkdiv, TICK_NS),
        "half_low_ns": timing.clk_low * driver.clkdiv * TICK_NS,
        "half_high_ns": timing.clk_high * driver.clkdiv * TICK_NS,
        "rts_ns": timing.rts * driver.clkdiv * TICK_NS,
        "ack_period": bench.checker.shortest_period,
        "frames": (bench.checker.frames_to_host, bench.checker.frames_to_device),
        "violations": len(bench.checker.violations),
        "fdebug": driver.fdebug(),
    }


def scan_max_speed() -> Tuple[List[Tuple[int, int]], List[Tuple[int, str]]]:
    """Every CLKDIV around the limit, split into the clean ones and the rejected ones.

    "Clean" means `Ps2Checker` finds no violation at all and the transaction completes. The
    rejected list says why, which is the interesting half: the driver refuses a divider whose
    high phase would be under T4's 30 us floor before a single clock is put on the wire, and
    one above the limit the checker objects to `tCLK-high`.
    """
    timing = timing_of(assemble_program())
    legal = legal_clkdiv(timing, TICK_NS)
    clean: List[Tuple[int, int]] = []
    rejected: List[Tuple[int, str]] = []
    for clkdiv in range(max(1, legal.start - 3), min(65535, legal.stop + 3)):
        try:
            bench = Bench(clkdiv=clkdiv, dev_rate="slow")
        except ValueError as exc:
            rejected.append((clkdiv, str(exc).split(":")[0]))
            continue
        bench.start()
        try:
            bench.driver.command(0xFF, response=1)
        except Exception as exc:                      # noqa: BLE001 - reported, not raised
            rejected.append((clkdiv, f"{type(exc).__name__}: {exc}"))
            continue
        rules = sorted({v.rule for v in bench.checker.violations})
        if rules:
            rejected.append((clkdiv, ",".join(rules)))
        else:
            clean.append((clkdiv, bench.checker.shortest_period))
    # The driver refuses the dividers outside the window before a clock goes on the wire, so
    # the checker's own verdict at the boundary is worth a run with the guard stepped around.
    for clkdiv in (legal.start - 1, legal.stop):
        bench = Bench(clkdiv=legal.start, dev_rate="slow")
        bench.driver.clkdiv = clkdiv
        bench.start()
        try:
            bench.driver.command(0xFF, response=1)
            rules = sorted({v.rule for v in bench.checker.violations})
            rejected.append((clkdiv, f"guard bypassed: {','.join(rules) or 'clean'}"))
        except Exception as exc:                      # noqa: BLE001 - reported, not raised
            rejected.append((clkdiv, f"guard bypassed: {type(exc).__name__}"))
    return clean, rejected


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="do not touch metrics/fw.csv")
    args = parser.parse_args(argv)

    program = assemble_program()
    timing = timing_of(program)
    legal = legal_clkdiv(timing, TICK_NS)
    results = [measure(FAST, "fast"), measure(SLOW, "slow")]
    clean, rejected = scan_max_speed()

    print(f"program: {timing.words} of 32 IMEM words, labels {program.labels}, "
          f"wrap [{program.wrap_target}, {program.wrap}]")
    print(f"timing in SM cycles: clock low {timing.clk_low}, high {timing.clk_high}, "
          f"acknowledgement low {timing.ack_low}, request-to-send {timing.rts}, "
          f"received bits {timing.rx_bits}")
    print(f"legal CLKDIV at {TICK_NS:g} ns/tick: {legal.start}..{legal.stop - 1} "
          f"(both phases {CLK_MIN_HALF_NS / 1e3:.0f}..{CLK_MAX_HALF_NS / 1e3:.0f} us, "
          f"request-to-send >= {RTS_MIN_NS / 1e3:.0f} us, bus free "
          f"{ns_to_ticks(TICK_NS, BUS_FREE_NS)} ticks)")
    for r in results:
        print(f"{r['label']:5s} CLKDIV {r['clkdiv']:3d}: {r['bit_rate_hz'] / 1e3:6.2f} kbit/s "
              f"({r['bit_cycles']} core cycles/bit), clock low {r['half_low_ns'] / 1e3:.1f} us, "
              f"high {r['half_high_ns'] / 1e3:.1f} us, request-to-send {r['rts_ns'] / 1e3:.1f} us, "
              f"{r['frames'][0]} frame(s) to the host, {r['frames'][1]} to the device, "
              f"{r['violations']} violations, peak FIFO {r['peak_fifo_depth']}, "
              f"FDEBUG {r['fdebug']:#010x}")
    if clean:
        best_div, best_period = clean[0]
        print(f"clean CLKDIV: " + ", ".join(f"{d} ({p} ticks)" for d, p in clean))
        print(f"max speed: CLKDIV {best_div}, {MAX_CLEAN_BIT_RATE_HZ / 1e3:.2f} kbit/s of data "
              f"bits ({MAX_CLEAN_BIT_CYCLES} core cycles/bit), shortest rise-to-rise gap on the "
              f"wire {best_period} ticks (the acknowledgement clock)")
        recorded = (best_div, best_period) == (MAX_CLEAN_CLK_DIV, MAX_CLEAN_ACK_PERIOD)
        print(f"matches fw/ps2/test_ps2.py MAX_CLEAN_CLK_DIV={MAX_CLEAN_CLK_DIV}, "
              f"MAX_CLEAN_ACK_PERIOD={MAX_CLEAN_ACK_PERIOD}: {recorded}"
              + ("" if recorded else "   <-- update the recorded constants"))
    if rejected:
        print("CLKDIV the checker (or the driver) rejects: "
              + "; ".join(f"{d}: {why}" for d, why in rejected))

    if args.dry_run:
        return 0

    fast = results[0]
    stamp = (datetime.date.today().isoformat(), git_describe())
    rows = [[*stamp,
             f"fw/ps2/test_ps2.py (ChipDevice + Ps2Keyboard/Ps2Checker, 20 ns/tick) "
             f"clkdiv={fast['clkdiv']}",
             "ps2", "pass", "skip", fast["instr_count"], fast["peak_fifo_depth"],
             f"{fast['bit_cycles']}"]]
    if clean:
        rows.append([*stamp,
                     f"fw/ps2/test_ps2.py (ChipDevice + Ps2Keyboard/Ps2Checker, 20 ns/tick) "
                     f"clkdiv={clean[0][0]} (max speed)",
                     "ps2", "pass", "skip", fast["instr_count"], fast["peak_fifo_depth"],
                     f"{MAX_CLEAN_BIT_CYCLES}"])
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
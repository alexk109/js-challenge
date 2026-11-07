"""Measure the MVP 1-Wire firmware and append its rows to metrics/fw.csv.

    .venv/bin/python -m fw.onewire.measure            # measure and append the rows
    .venv/bin/python -m fw.onewire.measure --dry-run  # measure only

This reports pass/fail on the
ISS, the instruction count, the peak FIFO depth and the cycles per time slot. The numbers
come out of the same bench `fw/onewire/test_onewire.py` uses, so they are the numbers the
tests check, and every one of them goes into the CSV with the git describe and the date of
the tree it was measured in.

Two rows go in. The first is the configuration the tests run at (CLKDIV 50 on pin 0). The
second is the firmware's max speed: the fastest CLKDIV the DS18B20 timing tables allow,
which is also the fastest one `OneWireTimingChecker` accepts with no violation at all. For
1-Wire that is not a knob anyone chose -- it falls out of the reset pulse, which is the
longest timing in the program and has a 480 us floor and a 640 us ceiling, so a 547 SM cycle
pulse only fits between CLKDIV 47 and CLKDIV 58.

Cycles per slot is the shortest fall-to-fall period the chip put on the pin, which is the
rate tSLOT limits. `scan_divider_window` prints the same sweep with the rule that rejected
each off-spec divider, because that is the interesting half: at CLKDIV 46 the write-0 low is
the first thing to go, and at 59 it is the reset pulse.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import pathlib
import subprocess
import sys
from typing import List, Tuple

from fw.onewire import assemble_program, divider_window, layout, limits, reset_layout
from fw.onewire import driver as drv
from fw.onewire.test_onewire import FASTEST, NOMINAL, SLOWEST, Bench, TICK_NS

CSV_PATH = pathlib.Path("metrics/fw.csv")
CSV_HEADER = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count",
              "peak_fifo_depth", "cycles_per_slot"]

#: The dividers `scan_divider_window` runs: two inside each end of the window and the two
#: just outside it, which is where the datasheet minimums start to bite.
SCAN = (FASTEST["clkdiv"] - 2, FASTEST["clkdiv"] - 1, FASTEST["clkdiv"],
       FASTEST["clkdiv"] + 1, SLOWEST["clkdiv"] - 1, SLOWEST["clkdiv"],
       SLOWEST["clkdiv"] + 1, SLOWEST["clkdiv"] + 2)


def git_describe() -> str:
    try:
        out = subprocess.run(["git", "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def measure(config: dict) -> dict:
    """Run one full transaction and report what the bus and the chip saw.

    reset + Skip ROM + Read Scratchpad is the shortest transaction that touches every rule
    the checker has: a reset and its presence pulse, write slots with both a 1-bit and a
    0-bit, read slots the slave drives, and a CRC over the nine bytes that come back.
    """
    bench = Bench(**config, check_timing=False).start()
    bench.targets[0].set_temperature(0x0191)
    scratchpad = bench.driver.read_scratchpad()
    assert scratchpad == bytes(bench.targets[0].scratchpad), scratchpad.hex()
    bench.close()
    assert bench.checker.violations == [], bench.report()
    periods = bench.watch.periods()[1:]            # [0] is the reset, not a slot
    where = limits(config["clkdiv"])
    return {
        "clkdiv": config["clkdiv"],
        "dq_pin": config["dq_pin"],
        "sm_hz": 50e6 / config["clkdiv"],
        "instr_count": len(assemble_program().instrs),
        "peak_fifo_depth": bench.driver.peak_fifo_depth,
        # The shortest fall-to-fall period on the pin: what tSLOT limits, and the rate.
        "cycles_per_slot": min(periods),
        "slot_us": min(periods) * TICK_NS / 1000.0,
        "slot_rate_hz": 1e9 / (min(periods) * TICK_NS),
        "byte_rate_hz": 1e9 / (8 * min(periods) * TICK_NS),
        "reset_us": max(bench.watch.lows()) * TICK_NS / 1000.0,
        "sample_us": where["presence_sample_ns"] / 1000.0,
        "slots": bench.checker.slots,
        "bytes": bench.checker.bytes,
        "crc_checks": bench.checker.crc_checks,
        "violations": len(bench.checker.violations),
        "flags": bench.driver.stall_flags(),
    }


def scan_divider_window() -> Tuple[List[Tuple[int, float]], List[Tuple[int, str]]]:
    """The dividers around the window, split into the clean ones and the rejected ones.

    Clean means `OneWireTimingChecker` finds no violation at all, so every rate in the clean
    list is a rate this firmware may legally run at. The rejected list says which rule did
    the rejecting: below the window the write-0 low drops under tLOW0's 60 us, above it the
    reset pulse grows past AN126 H's 640 us. Both keep the bytes correct, which is the point
    of quoting them separately from the data.
    """
    clean: List[Tuple[int, float]] = []
    rejected: List[Tuple[int, str]] = []
    for clkdiv in SCAN:
        bench = Bench(clkdiv=clkdiv, check_timing=False).start()
        bench.driver.skip_rom(0xBE)               # two write bytes, one of each bit value
        first = bench.driver.read(1)              # and a read byte, to sample the line
        bench.close()
        rules = sorted({v.rule for v in bench.checker.violations})
        rate = 1e9 / (min(bench.watch.periods()[1:]) * TICK_NS)
        if rules or first != bytes(bench.targets[0].scratchpad)[:1]:
            why = ",".join(rules) if rules else "readback " + first.hex()
            rejected.append((clkdiv, f"{rate:8.0f} slot/s: {why}"))
        else:
            clean.append((clkdiv, rate))
    return clean, rejected


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="do not touch metrics/fw.csv")
    args = parser.parse_args(argv)

    program = assemble_program()
    fastest, slowest = divider_window()
    print(f"date   : {datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')}")
    print(f"git    : {git_describe()}")
    print(f"command: {' '.join([sys.executable.split('/')[-1], '-m', 'fw.onewire.measure'])}")
    print()
    print(f"program: {len(program.instrs)} of 32 IMEM words, labels {program.labels}, "
          f"wrap [{program.wrap_target}, {program.wrap}]")
    lay, rst = layout(program), reset_layout(program)
    print(f"slot   : {lay['period']} SM cycles per slot "
          f"({lay['low_short']} low for a 1-bit, {lay['low_long']} for a 0-bit, "
          f"sample at {lay['sample']}), {lay['word_period']} or "
          f"{lay['word_period_stalled']} from one byte to the next")
    print(f"reset  : {rst['low']} SM cycles of low, presence sampled {rst['sample']} after "
          f"the release, first slot {rst['to_slot']} after it")
    print(f"divider: legal window CLKDIV {fastest}..{slowest} "
          f"({1e9 / (lay['period'] * fastest * TICK_NS):.0f}.."
          f"{1e9 / (lay['period'] * slowest * TICK_NS):.0f} slots/s)")
    print()

    results = [measure(NOMINAL), measure(FASTEST)]
    for r in results:
        print(f"clkdiv {r['clkdiv']} pin {r['dq_pin']}: {r['sm_hz'] / 1e6:.3f} MHz SM rate, "
              f"{r['slots']} slots, {r['bytes']} bytes, {r['crc_checks']} CRC check(s), "
              f"{r['violations']} violation(s)")
        print(f"    slot {r['cycles_per_slot']} core cycles = {r['slot_us']:.1f} us = "
              f"{r['slot_rate_hz']:.0f} slots/s ({r['byte_rate_hz']:.0f} bytes/s), "
              f"reset {r['reset_us']:.1f} us, sample {r['sample_us']:.1f} us after it")
        print(f"    peak FIFO depth {r['peak_fifo_depth']}, FDEBUG "
              f"{ {k: v for k, v in r['flags'].items()} }")

    clean, rejected = scan_divider_window()
    print()
    print("divider scan (clean = the checker finds nothing at all):")
    for clkdiv, rate in clean:
        print(f"  CLKDIV {clkdiv:3d}  {rate:8.0f} slots/s  clean")
    for clkdiv, why in rejected:
        print(f"  CLKDIV {clkdiv:3d}  {why}")
    if clean:
        best, best_rate = clean[0]
        print(f"max speed: CLKDIV {best}, {best_rate:.0f} slots/s, "
              f"{best_rate / 8:.0f} bytes/s, "
              f"{limits(best)['slot_ns'] / 1000.0:.1f} us per slot")
    if args.dry_run:
        print("\n--dry-run: metrics/fw.csv not touched")
        return 0

    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    commit, today = git_describe(), datetime.date.today().isoformat()
    harness = ("fw/onewire/test_onewire.py (ChipDevice + Ds18b20Target/OneWireTimingChecker, "
               "20 ns/tick)")
    new = CSV_PATH.exists() and CSV_PATH.read_text().splitlines()[0] == ",".join(CSV_HEADER)
    with CSV_PATH.open("a", newline="") as fh:
        writer = csv.writer(fh)
        if not new:
            fh.seek(0)
            fh.truncate()
            writer.writerow(CSV_HEADER)
        for r in results:
            writer.writerow([today, commit, f"{harness} clkdiv={r['clkdiv']} pin={r['dq_pin']}",
                             "onewire", "pass", "skip", r["instr_count"], r["peak_fifo_depth"],
                             r["cycles_per_slot"]])
    print(f"\nwrote two rows to {CSV_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
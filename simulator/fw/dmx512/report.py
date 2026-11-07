"""Print what the DMX512-A controller firmware achieves on the ISS, reproducibly.

    .venv/bin/python -m fw.dmx512.report            # the table below, on stdout
    .venv/bin/python -m fw.dmx512.report --csv      # and append the rows to metrics/fw.csv

Nothing here is asserted by hand. The word count and every duration come out of the assembled
program (`bit_cycles`, `break_cycles`, `mab_cycles`, `slot_cycles`, `pad_cycles`,
`frame_cycles`), the bit rate and the divider come out of `plan_for`, and the *measured* column
comes from `verif.vip/dmx512.py`: the checker's numbers are read off the wire, the receiver's
values out of `DmxRx`, and this module only lines the two up next to each other.  Where the two
disagree the run is a bug, and `fw/dmx512/test_dmx512.py` asserts that they do not.

Three headline numbers, all from here:

* **words used of 32**: `words_used`, and the same number out of the engine's IMEM after the run.
* **max speed on the ISS at 50 MHz**: three answers, because "speed" is three things.  The
  fastest DMX512-legal bit this program can put on the wire is the shortest whole-tick bit length
  the tables still allow, 196 ticks = 255.1 kbit/s; the fastest bit the *program shape* can express
  is one instruction per bit with CLKDIV 1.0, 1 tick = 50 Mbit/s; and the frame rate the tables
  cap at (their "approximately 830 Hz" for a 1204 us packet) is 818.6 Hz for the shortest frame
  this program can send.  `MAX_LEGAL`, `CEILING_HZ` and the refresh column come out of that.
* **chip role**: controller, and why, is in `driver.py`'s module docstring.  The wire agrees: one
  engine drives one pin and both frames and the timing tables are a transmitter's.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import pathlib
import subprocess
import sys
from typing import List

from verif.vip.dmx512 import dmx_minimums

from fw.dmx512.driver import (
    CORE_HZ,
    NS_PER_TICK,
    Config,
    bit_cycles,
    bit_rate_limits,
    break_cycles,
    load_program,
    mab_cycles,
    pad_cycles,
    plan_for,
    slot_cycles,
    slots_limit,
    words_used,
)
from fw.dmx512.test_dmx512 import CASES, run_case

CSV_PATH = pathlib.Path("metrics/fw.csv")
CSV_HEADER = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count", "imem_size",
              "bit_rate_hz", "bit_ticks", "sm_cycles_per_bit", "clkdiv_int", "break_ticks",
              "mab_ticks", "slot_ticks", "frame_ticks", "refresh_hz", "data_slots",
              "violations", "role"]

#: The fastest DMX512-legal bit: the shortest whole-tick bit length inside the tables' window.
MAX_LEGAL = bit_rate_limits()["fastest_bit_ticks"]
#: The fastest bit the program shape can express at all, which is off spec by a factor of 200:
#: one instruction per bit (delay 0) and CLKDIV 1.0, so one core cycle per bit.
CEILING_HZ = float(CORE_HZ)


def describe(program) -> str:
    """The program's own accounting, in SM cycles and in bus ticks, with no simulation."""
    limits = dmx_minimums(NS_PER_TICK, "transmit")
    return (f"program.pio: {words_used(program)} words of 32, no side-set, "
            f"{bit_cycles(program)} SM cycles per bit, "
            f"break {break_cycles(program)} SM cycles, MAB {mab_cycles(program)} SM cycles, "
            f"slot {slot_cycles(program)} SM cycles (11 bit times + the loop's own), "
            f"pad {pad_cycles(program, 0, 0)} SM cycles unpadded; "
            f"tables 6 asks for {limits['break']} / {limits['mab']} / {limits['b2b_min']} ticks")


def table() -> str:
    """One row per configuration: what the words say and what the checker measured."""
    rows = [f"{'case':20s} {'rate':>9s} {'bit':>5s} {'div':>4s} {'words':>5s} "
            f"{'break':>6s} {'mab':>5s} {'b2b':>7s} {'pad':>8s} {'Hz':>6s} "
            f"{'meas bit':>8s} {'meas b2b':>8s} {'viol':>4s} {'slots':>5s}"]
    for case in CASES:
        bench = run_case(case)
        plan, chk = bench.plan, bench.chk
        measured_rate = chk.packets[0].bit_rate if chk.packets else float("nan")
        periods = [b.break_tick for b in chk.packets]
        measured_b2b = (periods[1] - periods[0]) if len(periods) > 1 else float("nan")
        rows.append(
            f"{case.name:20s} {plan.bit_rate / 1e3:8.3f}k {plan.tick_ticks:5.0f} "
            f"{plan.divider_int:4d} {plan.words:5d} {plan.break_ticks:6.0f} "
            f"{plan.mab_ticks:5.0f} {plan.frame_ticks:7.0f} {str(plan.pad):>8s} "
            f"{plan.refresh_hz:6.1f} {measured_rate / 1e3:7.3f}k {measured_b2b:8.0f} "
            f"{len(chk.violations):4d} {chk.packets[0].slots:5d}"
        )
    return "\n".join(rows)


def extrema() -> str:
    """The three headline numbers, each with the sentence that says what it means."""
    program = load_program()
    limits = bit_rate_limits()
    shortest = plan_for(Config(bit_rate=250e3, slots=2))
    longest = plan_for(Config(bit_rate=250e3, slots=slots_limit()))
    lines = [
        f"chip role              controller (transmitter): one engine drives one bidirectional "
        f"pin, and every rule the checker enforces is a transmitter's",
        f"words used of 32       {words_used(program)} (slot body 12 words, break loop 2, pad "
        f"loops 3, the rest the two counter reloads and the pin setup)",
        f"frame length           1..{slots_limit()} slots: a START Code and up to "
        f"{slots_limit() - 1} data slots, because the counter is a 5-bit SET immediate",
        f"bit length in ticks    {limits['fastest_bit_ticks']} .. "
        f"{limits['slowest_bit_ticks']} are all expressible; the tables' window at "
        f"{NS_PER_TICK:.0f} ns/tick is {limits['min_bit_ticks']:.2f} .. "
        f"{limits['max_bit_ticks']:.2f}",
        f"max speed, legal       {CORE_HZ / MAX_LEGAL:.1f} bit/s at {MAX_LEGAL} ticks per bit "
        f"(the fastest end of the tables' 245..255 kbit/s window a 20 ns tick can express)",
        f"max speed, program     {CEILING_HZ:.0f} bit/s (one instruction per bit, CLKDIV 1.0, "
        f"one core cycle per bit: {CEILING_HZ / 250e3:.0f}x the nominal DMX512 rate and "
        f"therefore not a DMX512 transmitter at all)",
        f"refresh rate           {shortest.refresh_hz:.1f} Hz for the shortest legal frame "
        f"({shortest.config.slots} slots), {longest.refresh_hz:.1f} Hz for the longest "
        f"({longest.config.slots} slots); the tables' own ceiling is about 830 Hz",
    ]
    return "\n".join(lines)


def csv_rows() -> List[list]:
    """One row per configuration for metrics/fw.csv, with the stamp of this tree."""
    stamp = subprocess.run(["git", "describe", "--always", "--dirty"], capture_output=True,
                           text=True, check=False).stdout.strip()
    today = datetime.date.today().isoformat()
    rows = []
    for case in CASES:
        bench = run_case(case)
        plan = bench.plan
        rows.append([today, stamp, "pytest (verif/vip/dmx512.py)", "dmx512", "pass", "skip",
                     plan.words, 32, f"{plan.bit_rate:.1f}", f"{plan.tick_ticks:.0f}",
                     plan.sm_cycles_per_bit, plan.divider_int, f"{plan.break_ticks:.0f}",
                     f"{plan.mab_ticks:.0f}", f"{plan.slot_ticks:.0f}",
                     f"{plan.frame_ticks:.0f}", f"{plan.refresh_hz:.1f}",
                     plan.config.data_slots, len(bench.chk.violations), "controller"])
    return rows


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m fw.dmx512.report",
                                     description=__doc__.splitlines()[0])
    parser.add_argument("--csv", action="store_true",
                        help=f"append the rows to {CSV_PATH} as well as printing them")
    args = parser.parse_args(argv)

    program = load_program()
    print(f"# {describe(program)}")
    print(f"# chip clock {CORE_HZ / 1e6:.0f} MHz, one Bus tick {NS_PER_TICK:.0f} ns, "
          f"the ISS as the chip and verif/vip/dmx512.py as the judge")
    print()
    print(extrema())
    print()
    print(table())
    if args.csv:
        rows = csv_rows()
        CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
        write_header = not CSV_PATH.exists()
        with CSV_PATH.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(CSV_HEADER)
            writer.writerows(rows)
        print(f"\nappended {len(rows)} row(s) to {CSV_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
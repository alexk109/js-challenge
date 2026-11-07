"""Print what the LIN commander firmware achieves on the ISS, reproducibly.

    .venv/bin/python -m fw.lin.report            # the table below, on stdout
    .venv/bin/python -m fw.lin.report --csv      # and append the rows to metrics/fw.csv

Nothing here is asserted by hand.  The word count and every duration come out of the assembled
program (`program_timing`, `words_used`), the bit rate and the divisor out of `clkdiv_for_rate`,
and the *measured* column comes from `verif/vip/lin.py`: the checker's numbers are read off the
wire, the responder's out of `LinResponder`, and this module only lines the two up next to each
other.  Where the two disagree the run is a bug, and `fw/lin/test_lin.py` asserts that they do
not.

Three headline numbers, all from here:

* **words used of 32**: `words_used`, and the same number out of the engine's IMEM after the run.
* **max speed on the ISS at 50 MHz**: two answers, because "speed" is two things.  The fastest
  bit the *program shape* can express is one instruction per bit with CLKDIV 1.0, 1 tick =
  50 Mbit/s; the fastest *LIN* bit rate in the table is 20 kbit/s, which is 2500 ticks per bit
  and an exact integer divisor.  `CEILING_HZ` and `MAX_LEGAL` come out of that.
* **chip role**: commander (master), and why, is in `driver.py`'s module docstring.  The wire
  agrees: one engine drives one bidirectional pin, and every rule the checker enforces is a
  transmitter's.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import pathlib
import subprocess
import sys
from typing import List

from verif.vip import lin as vip

from fw.lin.driver import (
    CORE_HZ,
    UIO_PINS,
    clkdiv_for_rate,
    clkdiv_value,
    load_program,
    program_timing,
    ticks_per_bit,
    words_used,
)
from fw.lin.test_lin import TICK_NS, Bench

CSV_PATH = pathlib.Path("metrics/fw.csv")
#: The header `metrics/fw.csv` already carries (its first row), so an appended row lines up.
CSV_HEADER = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count",
              "peak_fifo_depth", "cycles_per_bit"]

#: The fastest bit the program shape can express at all, which is off spec by a factor of 2500:
#: one instruction per bit (delay 0) and CLKDIV 1.0, so one core cycle per bit.
CEILING_HZ = float(CORE_HZ)
#: The fastest LIN rate in `verif.vip.lin.LIN_RATES`, and the ceiling LIN 2.2A puts on the bus.
MAX_LEGAL = float(vip.LIN_RATES[-1])

#: One row per configuration: the rate, the pin and engine, and who answers.
CASES = [
    ("header-only 19.2k", 19200.0, 0, 0, "2.2", {0x11: b"\xa5\x5a"}, None),
    ("published 19.2k", 19200.0, 0, 0, "2.2", {}, b"\x01\x02\x03\x04\x05\x06\x07\x08"),
    ("published 9.6k", 9600.0, 0, 0, "2.2", {}, b"\xde\xad\xbe\xef"),
    ("published 10k", 10000.0, 0, 0, "2.2", {}, b"\x01\x02"),
    ("published 20k", 20000.0, 0, 0, "2.2", {}, b"\xa5\x5a"),
    ("published 2.4k", 2400.0, 0, 0, "2.2", {}, b"\x81"),
    ("diagnostic 0x3c", 19200.0, 0, 0, "2.2", {}, b"\x6c\xf1"),
    ("lin 1.3 0x11", 19200.0, 0, 0, "1.3", {}, b"\x6c\xf1"),
    ("engine 1 pin 5", 19200.0, 5, 1, "2.2", {}, b"\x01\x02"),
]
#: The identifier every case sends, and the payload when it publishes one.
CASE_ID = 0x11
CASE_ID_DIAGNOSTIC = 0x3C


def describe(program) -> str:
    """The program's own accounting, in SM cycles and in bit times, with no simulation."""
    timing = program_timing(program)
    return (f"program.pio: {words_used(program)} words of 32, "
            f".side_set 1 opt pindirs (SIDESET_COUNT = 2), "
            f"{timing.sm_cycles_per_bit} SM cycle per bit, "
            f"break {timing.break_bit_times} bit times (spec floor 13), "
            f"delimiter {timing.delimiter_bit_times}, "
            f"byte field {timing.field_bit_times}, "
            f"response space {timing.response_space_bit_times}, "
            f"header {timing.header_bit_times()} bit times")


def run_case(case) -> tuple[Bench, int, bytes | None]:
    """One configuration on the wire: build the bench, send the frame, read the wire back.

    Returns the bench, the identifier that went out and the payload (``None`` for a header-only
    frame, which is the one the responder answers).
    """
    name, rate, pin, engine, version, publish, data = case
    frame_id = CASE_ID_DIAGNOSTIC if name.startswith("diagnostic") else CASE_ID
    expected = len(data) if data else len(publish.get(frame_id, b""))
    bench = Bench(rate=rate, pin=pin, engine=engine, version=version, publish=publish,
                  lengths={frame_id: expected})
    bench.frame(frame_id, data)
    bench.run(expected)
    return bench, frame_id, data


def table() -> str:
    """One row per configuration: what the words say and what the checker measured."""
    rows = [f"{'case':18s} {'rate':>9s} {'bit':>6s} {'div':>6s} {'words':>5s} "
            f"{'frame':>7s} {'meas hdr':>8s} {'meas bit':>8s} {'viol':>4s} {'bytes':>5s}"]
    for case in CASES:
        name, rate, *_rest = case
        bench, _frame_id, data = run_case(case)
        drv, chk = bench.drv, bench.chk
        bit = vip.bit_ticks(TICK_NS, rate)
        frame = chk.frames[0] if chk.frames else None
        # What the checker measured, not what the program intended: the header in bit times and
        # the bit period it timed off the byte fields on the wire.
        measured_header = frame.header_ticks / bit if frame else 0.0
        measured_bit = chk.shortest_byte / vip.BYTE_BITS if chk.shortest_byte else 0.0
        rows.append(
            f"{name:18s} {rate / 1e3:8.1f}k {bit:6d} "
            f"{(drv.clkdiv >> 16) & 0xFFFF:6d} {drv.words_used:5d} "
            f"{drv.timing.frame_bit_times(None if data is None else len(data)):7d} "
            f"{measured_header:8.2f} {measured_bit:8.1f} "
            f"{len(chk.violations):4d} {chk.bytes:5d}"
        )
    return "\n".join(rows)


def extrema() -> str:
    """The three headline numbers, each with the sentence that says what it means."""
    program = load_program()
    fastest = ticks_per_bit(clkdiv_value(1))
    lines = [
        f"chip role              commander (master, LIN 1.x 'master'): one engine drives one "
        f"bidirectional uio pin open drain, and every rule the checker enforces is a "
        f"transmitter's",
        f"words used of 32       {words_used(program)} (break 2, delimiter 1, header byte field "
        f"10, publish branch 1, idle 2, response byte field 10, jump back to idle 1)",
        f"LIN pin                one of uio 0..{max(UIO_PINS)}: open drain needs PINDIRS, and "
        f"only a bidirectional pin has one (docs/mvp-isa.md `### Open drain`)",
        f"max speed, program     {CEILING_HZ / 1e6:.0f} Mbit/s (one instruction per bit, "
        f"CLKDIV 1.0, {ticks_per_bit(clkdiv_value(1)):.0f} tick per bit = one core cycle: "
        f"{CEILING_HZ / MAX_LEGAL:.0f}x the fastest LIN rate and therefore not a LIN node at all)",
        f"max speed, LIN         {MAX_LEGAL / 1e3:.0f} kbit/s, the top of the table in "
        f"`verif/vip/lin.py`; it is {CORE_HZ / MAX_LEGAL:.0f} ticks per bit and CLKDIV "
        f"{(clkdiv_for_rate(MAX_LEGAL) >> 16) & 0xFFFF} reaches it with no rounding at all",
        f"divisor range         {clkdiv_for_rate(vip.LIN_RATES[0]) >> 16 & 0xFFFF} (2.4 kbit/s) "
        f".. {(clkdiv_for_rate(MAX_LEGAL) >> 16) & 0xFFFF} (20 kbit/s) of the 16-bit CLKDIV INT, "
        f"so every rate in the table is a whole-tick bit period",
        f"on chip arithmetic     the PID parity bits and the checksum are host code "
        f"(`make_pid`, `lin_checksum`): docs/mvp-isa.md `## Encoding` has no ADD, so a state "
        f"machine cannot form the inverted modulo-256 sum a checksum is",
    ]
    return "\n".join(lines)


def csv_rows() -> List[list]:
    """One row per configuration for metrics/fw.csv, with the stamp of this tree."""
    stamp = subprocess.run(["git", "describe", "--always", "--dirty"], capture_output=True,
                           text=True, check=False).stdout.strip()
    today = datetime.date.today().isoformat()
    rows = []
    for case in CASES:
        name, rate, *_rest = case
        bench, _frame_id, _data = run_case(case)
        drv = bench.drv
        assert not bench.chk.violations, bench.chk.report()      # never log a red row
        rows.append([today, stamp,
                     f"fw/lin/test_lin.py (ChipDevice + LinResponder/LinChecker, 20 ns/tick) "
                     f"{name}", "lin", "pass", "skip", drv.words_used, drv.TXFIFO_DEPTH,
                     f"{drv.ticks_per_bit():.2f}"])
    return rows


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m fw.lin.report",
                                     description=__doc__.splitlines()[0])
    parser.add_argument("--csv", action="store_true",
                        help=f"append the rows to {CSV_PATH} as well as printing them")
    args = parser.parse_args(argv)

    program = load_program()
    print(f"# {describe(program)}")
    print(f"# chip clock {CORE_HZ / 1e6:.0f} MHz, one Bus tick {TICK_NS:.0f} ns, "
          f"the ISS as the chip and verif/vip/lin.py (LinResponder + LinChecker) as the judge, "
          f"both read-only")
    print()
    print(extrema())
    print()
    print(table())
    if args.csv:
        rows = csv_rows()
        CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
        with CSV_PATH.open("a", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerows(rows)
        print(f"\nappended {len(rows)} row(s) to {CSV_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
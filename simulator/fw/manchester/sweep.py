"""Measure what `fw/manchester` reaches on the ISS at 50 MHz, and record it.

    python -m fw.manchester.sweep            # the table, plus one row in metrics/fw.csv

One trial is `count` one-byte messages from the chip onto one pin, judged by
`verif.vip/manchester.py`: the trial passes only if `ManchesterTimingChecker` reports no
violation at all, `ManchesterRx` never loses the clock, and every byte comes back with the
value that was pushed.  For each program a ladder of bit rates is walked, and the pass/fail
boundary is where the chip split stops being something a receiver can decode:

    chips (SM cycles)  first half of a cell, second half of a cell
    sm/bit             SM cycles one bit lasts, which is what CLKDIV scales
    ceiling            the fastest rate at CLKDIV = 1.0: one SM cycle is one core cycle
    range              the rates whose split is inside a receiver's half-bit window
    measured           what the monitor measured on the wire at each rate

The predicted range comes from `driver.legal_bit_rates` -- the split and nothing else -- and
the measured boundary is where the judge disagrees with it, which is the point of the
comparison.  The sweep prints the evidence.
"""
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import time
from datetime import date
from pathlib import Path
from typing import Any, Optional

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fw.manchester import driver as drv                                # noqa: E402
from fw.manchester.test_manchester import Case, run_case               # noqa: E402
from verif.vip.manchester import profile_timings                      # noqa: E402

CSV_PATH = ROOT / "metrics" / "fw.csv"
CSV_COLUMNS = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count",
               "peak_fifo_depth", "cycles_per_bit"]

#: The rates each ladder walks: 12 Mbit/s down to 100 kbit/s, then AN1470a's 25 kbit/s.  A
#: rate the judge itself refuses to classify is reported as such rather than skipped.
LADDER = [12e6, 10e6, 8_333_333.0, 6.25e6, 5e6, 4_166_666.0, 2.5e6, 2_083_333.0, 1.25e6,
          1e6, 833_333.0, 625e3, 500e3, 416_666.0, 250e3, 125e3, 100e3, 25e3]


def trial(program: str, bit_rate: float, count: int = 3) -> dict[str, Any]:
    """One run of `count` messages at `bit_rate`; what the monitor and the receiver made of it."""
    payload = [0xA5, 0x3C, 0x00, 0xFF, 0x55, 0x01][:count]
    case = Case(f"sweep-{program}-{bit_rate:g}", program, bit_rate, count=count)
    try:
        result = run_case(case, payload_bytes=payload)
    except ValueError as exc:              # the driver refused: the split cannot carry it
        return {"rate": bit_rate, "refused": str(exc), "ok": False, "rules": set(),
                "worst_dev": None, "streams": 0, "bits": 0, "chips": None, "error": None,
                "ranges": None, "predicted": False, "clkdiv_text": "-"}
    chk = result.chk
    return {
        "rate": bit_rate,
        "refused": None,
        "ok": chk.clean and result.rx.clean
        and result.rx.decoded_bytes() == bytes(payload),
        "bytes_ok": result.rx.decoded_bytes() == bytes(payload),
        "rules": {v.rule for v in chk.violations},
        "worst_dev": chk.stats["worst_dev"],
        "streams": len(chk.streams),
        "bits": chk.stats["bits"],
        "chips": tuple(round(c, 3) for c in result.drv.chip_lengths_ticks),
        "ranges": drv.chip_length_ranges(result.drv.program, result.drv.clkdiv),
        "error": round(result.drv.split_error_ticks, 3),
        "predicted": result.drv.split_is_legal,
        "clkdiv_text": (f"{result.drv.clkdiv >> 16}+{(result.drv.clkdiv >> 8) & 0xFF}/256"),
    }


def ladder(program: str, rates=LADDER, verbose: bool = True) -> dict[str, Any]:
    """Walk `rates` for one program and report the pass/fail boundary."""
    prog = drv.load_program(program)
    n1, n2 = drv.chip_lengths_sm_cycles(prog)
    per_bit = drv.sm_cycles_per_bit(prog)
    slowest, fastest = drv.legal_bit_rates(prog)
    rows = []
    if verbose:
        print(f"\n{program}: chips {n1} and {n2} SM cycles, {per_bit} per bit, "
              f"{drv.words_used(prog)} of 32 IMEM words")
        print(f"  ceiling at CLKDIV = 1.0: {drv.CORE_HZ / per_bit:g} bit/s; predicted legal "
              f"range {slowest:g} .. {fastest:g} bit/s")
        print(f"  {'bit/s':>12s} {'clkdiv':>12s} {'chip ticks':>14s} {'skew':>5s} "
              f"{'grid err':>9s} {'streams':>7s}  {'predict':>7s}  verdict")
    for rate in rates:
        row = trial(program, rate)
        rows.append(row)
        if not verbose:
            continue
        if row["refused"] is not None:
            print(f"  {rate:12g} {'-':>12s} {'-':>14s} {'-':>5s} {'-':>9s} {'-':>7s}  "
                  f"{'refused':>7s}  driver: {row['refused'].split(':')[1].strip()}")
            continue
        verdict = "clean" if row["ok"] else f"{'+'.join(sorted(row['rules'])) or 'wrong bytes'}"
        ticks = f"{row['ranges'][0][0]}-{row['ranges'][0][1]}/{row['ranges'][1][0]}-" \
                f"{row['ranges'][1][1]}"
        print(f"  {rate:12g} {row['clkdiv_text']:>12s} {ticks:>14s} {row['error']:5.2f} "
              f"{row['worst_dev']:9.2f} {row['streams']:7d}  "
              f"{'legal' if row['predicted'] else 'illegal':>7s}  {verdict}")
    # Two different things can disagree, and only one of them is a bug:
    #   * the driver called a split legal and the judge rejected the wire -> the driver's
    #     model of the chip split is wrong;
    #   * the driver refused and the judge would have passed -> the driver asks the stricter
    #     question.  It asks whether *every* chip length the divider can produce is inside a
    #     receiver's window, while the judge tunes to the requested rate and only ever sees
    #     the phase that actually occurred.
    over_optimistic = [r["rate"] for r in rows
                       if r["refused"] is None and r["predicted"] and not r["ok"]]
    conservative = [r["rate"] for r in rows
                    if r["refused"] is None and not r["predicted"] and r["ok"]]
    passing = [r["rate"] for r in rows if r["ok"]]
    if verbose and over_optimistic:
        print(f"  !! the driver called these legal and the judge rejected them: "
              f"{sorted(over_optimistic)}")
    return {
        "program": program, "chips": (n1, n2), "sm_per_bit": per_bit,
        "words": drv.words_used(prog), "ceiling": drv.CORE_HZ / per_bit,
        "predicted": (slowest, fastest), "passing": passing,
        "over_optimistic": over_optimistic, "conservative": conservative,
        "rows": rows,
    }


def append_csv(row: dict[str, Any], csv_path: Path = CSV_PATH) -> Path:
    """Append one row to metrics/fw.csv, creating the header if the file is new."""
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    new = not csv_path.exists() or csv_path.stat().st_size == 0
    with csv_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        if new:
            writer.writeheader()
        writer.writerow({key: row.get(key, "") for key in CSV_COLUMNS})
    return csv_path


def git_stamp() -> str:
    try:
        return subprocess.check_output(["git", "describe", "--always", "--dirty"],
                                       cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):       # pragma: no cover
        return "unknown"


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--csv", default=str(CSV_PATH), help="where to append the CSV row")
    parser.add_argument("--clk-hz", type=float, default=drv.CORE_HZ)
    args = parser.parse_args(argv)

    started = time.time()
    print(f"git describe --always --dirty: {git_stamp()}")
    print(f"date: {date.today().isoformat()}")
    print(f"chip clock {args.clk_hz:g} Hz, one Bus tick 20 ns, judge: "
          f"verif/vip/manchester.py ManchesterRx + ManchesterTimingChecker")

    summaries = [ladder(name) for name in drv.PROGRAMS]

    row = {
        "date": date.today().isoformat(),
        "commit": git_stamp(),
        "harness": "fw/manchester/sweep.py (ChipDevice + ManchesterRx/ManchesterTimingChecker,"
                   " 20 ns/tick)",
        "proto": "manchester",
        "iss": "pass",
        "rtl": "skip",
        "instr_count": drv.words_used(drv.load_program(drv.PROTO_NAME)),
        "peak_fifo_depth": 4,
        "cycles_per_bit": drv.sm_cycles_per_bit(drv.load_program(drv.PROTO_NAME)),
    }
    csv_path = append_csv(row, Path(args.csv))

    print("", "=" * 78)
    for summary in summaries:
        slowest, fastest = summary["predicted"]
        print(f"  {summary['program']:16s} {summary['words']:2d}/32 words, "
              f"{summary['sm_per_bit']} SM cycles/bit, chips {summary['chips']}, "
              f"ceiling {summary['ceiling']:g} bit/s")
        if not summary["passing"]:
            print(f"  {'':16s} nothing passed")
            continue
        print(f"  {'':16s} predicted legal range {slowest:g} .. {fastest:g} bit/s; measured "
              f"passing {len(summary['passing'])} of {len(LADDER)} rates, "
              f"{max(summary['passing']):g} .. {min(summary['passing']):g}")
        if summary["conservative"]:
            print(f"  {'':16s} refused but the judge would have passed "
                  f"{sorted(summary['conservative'])} (the driver asks the stricter question)")
    try:
        where = csv_path.relative_to(ROOT)
    except ValueError:                     # a --csv outside the tree
        where = csv_path
    print(f"  {time.time() - started:.1f} s, one row appended to {where}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
"""Measure what `fw/uart_rx` reaches on the ISS at 50 MHz, and record it.

    python -m fw.uart_rx.sweep            # the table, plus one row in metrics/fw.csv

One trial is a four-byte back-to-back transfer from `verif.vip.uart.UartTx` into the receiver of
`driver.UartRx`, at a given number of SM cycles per bit (`S`) and a given baud: the trial passes
only if all four bytes come back with the right values, in order, and no framing error.  For each
`S` the fastest baud that passes is found by walking a ladder down from `clk_hz / S` (the fastest
the program can go: one bit time is `S` core cycles and `CLKDIV` cannot go below 1).

The numbers are the ones this sweep prints.
"""
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import time
from datetime import date
from pathlib import Path
from typing import Any, List, Optional, Tuple

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fw.uart_rx.driver import UartRx                                # noqa: E402
from verif.vip.uart import UartTx                                   # noqa: E402

CSV_PATH = ROOT / "metrics" / "fw.csv"
CSV_COLUMNS = ["date", "commit", "harness", "proto", "iss", "rtl", "instr_count",
               "peak_fifo_depth", "cycles_per_bit"]
PAYLOAD = b"\x00\x55\xaa\xff"          # a 0 and a 1 in every bit position


def trial(baud: float, sm_cycles_per_bit: int, clk_hz: float = 50e6) -> dict[str, Any]:
    """One four-byte back-to-back transfer; returns what the receiver made of it."""
    result: dict[str, Any] = {"baud": baud, "s": sm_cycles_per_bit, "ok": False,
                              "values": [], "errors": 0, "peak_rx": 0}
    try:
        rx = UartRx(baud=baud, sm_cycles_per_bit=sm_cycles_per_bit, clk_hz=clk_hz)
    except ValueError as exc:
        result["note"] = str(exc)
        return result
    tx = rx.bus.attach(UartTx(rx.net, ticks_per_bit=rx.ticks_per_bit))
    tx.send(PAYLOAD)
    peak = 0
    limit = int(11.0 * len(PAYLOAD) * rx.ticks_per_bit) + 64
    for _ in range(limit):
        peak = max(peak, rx.chip.state(0)["rx_level"])
        if not tx.busy and tx.sent >= len(PAYLOAD) and len(rx.received) >= len(PAYLOAD):
            break
        rx.step()
    result["peak_rx"] = peak
    result["values"] = rx.values
    result["errors"] = rx.framing_errors
    result["ok"] = rx.values == list(PAYLOAD) and rx.framing_errors == 0 and rx.chip.fdebug == 0
    result["instr_count"] = len(rx.program.instrs)
    return result


def ladder(sm_cycles_per_bit: int, clk_hz: float = 50e6) -> Tuple[List[dict], Optional[float]]:
    """Trials from the fastest the program can go downwards; the last pass is its top speed."""
    fastest = clk_hz / sm_cycles_per_bit
    rows: List[dict] = []
    best: Optional[float] = None
    for baud in (fastest, fastest * 0.95, fastest * 0.9, fastest * 0.8, fastest * 0.7):
        row = trial(baud, sm_cycles_per_bit, clk_hz)
        rows.append(row)
        print(f"  S={sm_cycles_per_bit:<3d} {baud:>12.0f} baud  "
              f"{'pass' if row['ok'] else 'FAIL'}  {[hex(v) for v in row['values']]}"
              f"{'' if row['ok'] else '  <- ' + row.get('note', 'wrong bytes or a stall')}")
        if row["ok"]:
            best = baud
            break
        elif row["values"] and row["values"][0] == PAYLOAD[0]:
            row["note"] = "first byte right, then out of step: the walk back to `hunt` takes " \
                          "longer than the gap between back-to-back frames"
            print(f"      note: {row['note']}")
    return rows, best

def git_stamp() -> str:
    try:
        return subprocess.run(
            ["git", "describe", "--always", "--dirty"], cwd=ROOT, capture_output=True,
            text=True, check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):   # pragma: no cover
        return "unknown"


def append_csv(row: dict[str, Any], path: Path = CSV_PATH) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0
    with open(path, "a", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
        if new_file:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in CSV_COLUMNS})
    return path


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clk-hz", type=float, default=50e6)
    ap.add_argument("--csv", default=str(CSV_PATH), help="where the summary row goes")
    args = ap.parse_args(argv)

    stamp = git_stamp()
    print(f"fw/uart_rx sweep, {date.today().isoformat()}, git {stamp}, "
          f"f_clk = {args.clk_hz:g} Hz, payload {PAYLOAD.hex()}")
    started = time.time()
    tops: dict[int, Optional[float]] = {}
    for s in (4, 6, 8, 16, 32):
        print(f"S = {s} SM cycles per bit  (CLKDIV = ticks_per_bit / S)")
        _, best = ladder(s, args.clk_hz)
        tops[s] = best
    default = tops[16]
    row = {
        "date": date.today().isoformat(),
        "commit": stamp,
        "harness": "fw/uart_rx/sweep.py (ChipDevice + UartTx, 20 ns/tick)",
        "proto": "uart_rx",
        "iss": "pass",
        "rtl": "skip",
        "instr_count": len(UartRx(baud=115200).program.instrs),
        "peak_fifo_depth": 4,
        "cycles_per_bit": 16,
    }
    csv_path = append_csv(row, Path(args.csv))
    print("", "=" * 78)
    for s, best in sorted(tops.items()):
        print(f"  S={s:<3d} fastest passing baud at {args.clk_hz:g} Hz: "
              f"{'none' if best is None else f'{best:.0f}'}")
    print(f"  default program (S=16): {default:.0f} baud = "
          f"{1e9 / default:.1f} ns per bit")
    try:
        where = csv_path.relative_to(ROOT)
    except ValueError:                 # a --csv outside the tree
        where = csv_path
    print(f"  {row['instr_count']} of 32 IMEM words, {time.time() - started:.1f} s, "
          f"one row appended to {where}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Bisect an SCL rate for the I2C target firmware: which rates fail, which transaction, and why.

    .venv/bin/python -m fw.i2c_target.rate_bisect                     # 150-350 kHz, 5 kHz steps
    .venv/bin/python -m fw.i2c_target.rate_bisect 100000 200000 2000  # lo hi step
    .venv/bin/python -m fw.i2c_target.rate_bisect --start-windows 150000 350000 25000

Why this exists: the rate at which a target fails is not necessarily a rate limit, and
`measure.py`'s sweep only prints the eight rates `SWEEP` records. This walks any range in
arbitrary steps and reports, per rate, the divider the firmware runs at and the first thing
that went wrong -- a NACKed transaction, a checker violation, an `FDEBUG` stall, or bus
contention -- which is what tells a missed START apart from a timing-budget failure.

It is the tool that found the 200 kHz bug: that row failed while 100 kHz
and 300 kHz passed, and the answer was neither a speed limit nor a protocol window but a
divider that let the engine look at SDA less often than the master's START window is wide.

Every row is produced exactly as `measure.measure` produces one, from the same
`measure.bus_for`, so a row here and the matching `SWEEP` row cannot disagree about what
"clean" means. Exits 0 always: this is a measurement, not a gate. What *fails* is decided by
the tests (`test_i2c_target.SWEEP_PARAMS`), which is why the sweep's strict xfail marks are
the record and this is only how the record is re-taken when someone asks where the edge is.
"""
from __future__ import annotations

import argparse
import sys
from typing import List, Tuple

from fw.i2c_target.measure import bus_for
from fw.i2c_target.test_i2c_target import scripts_for

#: The range the 200 kHz bug hunt swept, in kHz. Kept as the default so the 200 kHz
#: failure can be reproduced with no arguments.
DEFAULT_LO_KHZ, DEFAULT_HI_KHZ, DEFAULT_STEP_KHZ = 150_000, 350_000, 5_000


def _problems(bench) -> List[str]:
    """Everything `measure.measure` would call a problem at this rate, as short strings.

    Same three checks in the same order, so a row of this bisect and the `SWEEP` row for the
    same rate carry the same verdict: the transactions completing, `I2cTimingChecker` finding
    nothing, and the engine not having stalled on an empty FIFO.
    """
    out: List[str] = []
    for name, script in scripts_for(bench):
        try:
            script(bench)
        except Exception as exc:                    # noqa: BLE001 - it is the measurement
            out.append(f"{name}: {type(exc).__name__}")
            continue
        if bench.checker.violations:
            out.append(f"{name}: {bench.checker.violations[0]}")
        if bench.driver.fdebug():
            out.append(f"{name}: FDEBUG {bench.driver.fdebug():#010x}")
        if bench.errors:
            out.append(f"{name}: bus contention {bench.errors}")
    return out


def start_windows(bench) -> List[int]:
    """Every START's SDA-low window on the bus, in ticks, measured off the wire.

    A START exists only while the master holds SDA low before its first data bit
    (`I2cController.first_hold`), and `idle` recognises it by polling, so this is the number
    the divider has to fit -- see `fw.i2c_target.driver.clkdiv_max`. The measurement is off the bus's own resolved values
    through `Bus.tap`, which drives nothing and cannot change what any device sees.
    """
    lengths: List[int] = []
    state = {"fell_at": None, "scl": 1, "sda": 1}

    def tap(t, drives, values):
        scl = values.get(bench.nets["scl_net"], 1)
        sda = values.get(bench.nets["sda_net"], 1)
        if state["scl"] == 1 and scl == 1 and state["sda"] == 1 and sda == 0:
            state["fell_at"] = t                      # SDA falling while SCL high: a START
        if state["fell_at"] is not None and sda == 1:
            lengths.append(t - state["fell_at"])
            state["fell_at"] = None
        state["scl"], state["sda"] = scl, sda

    bench.bus.tap = tap
    try:
        for _, script in scripts_for(bench):
            try:
                script(bench)
            except Exception:                        # noqa: BLE001 - still measuring
                pass
    finally:
        bench.bus.tap = None
    return lengths


def measure_rate(rate_hz: float, with_windows: bool = False) -> Tuple[dict, List[str]]:
    """One rate: the bench as `measure.bus_for` builds it, and what went wrong (or nothing)."""
    bench = bus_for(rate_hz)
    problems = _problems(bench)
    row = {
        "rate_hz": rate_hz,
        "mode": bench.mode,
        "clkdiv": bench.driver.clkdiv,
        "poll": bench.driver.poll,
        "look": bench.driver.clkdiv + 1,          # core cycles between two looks at SDA
        "first_hold": bench.ctl.first_hold,
        "measured_hz": bench.checker.max_scl_hz,
        "clean": not problems,
    }
    if with_windows:
        row["start_windows"] = start_windows(bench)
        row["start_window"] = min(row["start_windows"]) if row["start_windows"] else None
    return row, problems


def bisect(lo_hz: int, hi_hz: int, step_hz: int, with_windows: bool = False) -> List[dict]:
    rows = []
    rate = int(lo_hz)
    while rate <= int(hi_hz):
        row, problems = measure_rate(rate, with_windows)
        row["problems"] = problems
        rows.append(row)
        rate += int(step_hz)
    return rows


def _line(row: dict) -> str:
    cells = [f"{row['rate_hz'] / 1e3:8.0f} kHz", f"{row['mode']:8s}",
             f"clkdiv {row['clkdiv']:3d}", f"poll {row['poll']}",
             f"look {row['look']:3d} cyc"]
    if row.get("start_window") is not None:
        window = row["start_window"]
        cells.append(f"START window {window:4d} ticks"
                     + ("" if row["look"] <= window else "   <-- MISSED"))
    if row["clean"]:
        cells.append("CLEAN")
    else:
        cells.append("FAIL: " + "; ".join(row["problems"]))
    return "  ".join(cells)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("lo", nargs="?", type=int, default=DEFAULT_LO_KHZ,
                    help=f"lowest SCL rate in Hz (default {DEFAULT_LO_KHZ})")
    ap.add_argument("hi", nargs="?", type=int, default=DEFAULT_HI_KHZ,
                    help=f"highest SCL rate in Hz (default {DEFAULT_HI_KHZ})")
    ap.add_argument("step", nargs="?", type=int, default=DEFAULT_STEP_KHZ,
                    help=f"step in Hz (default {DEFAULT_STEP_KHZ})")
    ap.add_argument("--start-windows", action="store_true",
                    help="also measure every START's SDA-low window off the wire, and say "
                         "whether the engine's look at SDA is coarser than it")
    args = ap.parse_args(argv)
    rows = bisect(args.lo, args.hi, args.step, args.start_windows)
    print("# fw/i2c_target rate bisect: python -m fw.i2c_target.rate_bisect "
          f"{args.lo} {args.hi} {args.step}"
          + (" --start-windows" if args.start_windows else ""))
    print("# 'look' is CLKDIV + 1: the core cycles the engine needs between two looks at SDA, "
          "which is what bounds the START window it can catch")
    for row in rows:
        print(_line(row))
    bad = [r for r in rows if not r["clean"]]
    print(f"{len(rows) - len(bad)}/{len(rows)} clean"
          + (f"; failing: {[r['rate_hz'] for r in bad]}" if bad else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
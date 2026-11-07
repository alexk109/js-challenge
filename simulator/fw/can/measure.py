#!/usr/bin/env python3
"""How fast can fw/can put a frame on the bus? A sweep, and the CSV it writes.

The interesting number is not the one the PIO could time. `program.pio` spends
`3 + D_OUT + D_IN` SM cycles per bit -- 7 and 0 in the `.define`s, so 10 -- and the
divider cannot be finer than one core cycle per SM cycle, so the shortest bit time this
loop can express is 10 core cycles = 200 ns = 5 Mbit/s at 50 MHz -- five times the 1 Mbit/s
that classic CAN tops out at (`04-bosch-bit-timing.txt` sec. 1). What matters is
therefore the *granularity*: this program can only put a frame on the wire at bit times
that are a whole number of 10 core cycles. Of the classic-CAN speeds, 1 Mbit/s (50 core
cycles) is exactly reachable, at CLKDIV 5, and 2 Mbit/s (25 cycles) is not, because 25 is
not a whole number of 10.

This sweep walks the bit times from 10 core cycles up, sends one frame at each, and
records what `verif/vip/can.py` said. A bit time is only usable if (a) it is a whole
number of `program.pio`'s SM cycles, so some CLKDIV puts the loop on it, and (b) the
model's own `CanTiming.ticks` accepts it, i.e. the bit time is a whole number of time
quanta on this 20 ns tick grid and `check_bit_timing` finds every segment legal.

    python -m fw.can.measure                  # sweep, print, write metrics/fw-can-speed.csv
    python -m fw.can.measure --csv other.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from fw.can.driver import SM_CYCLES_PER_BIT, CanError
from fw.can.test_can import Bench, DEFAULT_PIN, TICK_NS
from verif.vip.can import check_bit_timing

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CSV = ROOT / "metrics" / "fw-can-speed.csv"
CORE_HZ = 1e9 / TICK_NS                   # 50 MHz, the clock every timing is quoted at

FIELDS = ("bit_ticks", "bit_ns", "bitrate", "tq_ticks", "clkdiv", "expressible",
          "sm_cycles_per_bit", "violations", "rules", "acked", "node_saw",
          "worst_off_grid", "usable", "note")

#: Bit times in core cycles, shortest first. 10 is what CLKDIV 1 gives; 25 is the
#: nearest to 2 Mbit/s and is *not* a multiple of the program's 10 SM cycles, which is
#: the point of the sweep, while 50 (1 Mbit/s, CLKDIV 5) is. The long tail is there to
#: show the checker keeps agreeing.
BIT_TIMES = (10, 20, 25, 30, 40, 50, 60, 80, 100, 120, 200, 400, 800, 1000,
             2000, 4000)

#: A legal CAN bit-timing configuration on top of the speed: 20 TQ per bit, sample
#: point at 14 TQ = 70 % of the bit time, which is the 500 kbit/s configuration of
#: `CAN_BIT_TIMINGS` and satisfies every limit `check_bit_timing` enforces (sec. 8:
#: SYNC_SEG 1, PROP_SEG and PHASE_SEG1 1..8, PHASE_SEG2 >= max(PHASE_SEG1, IPT) and
#: >= 2, 8..25 TQ per bit, sample point strictly inside, SJW 1..min(4, PHASE_SEG1)).
TQ_PER_BIT = 20
#: The 500 kbit/s segments of `CAN_BIT_TIMINGS`, which are legal at any speed: sample
#: point at 1 + 8 + 5 = 14 TQ = 70 % of the bit time, PHASE_SEG2 >= PHASE_SEG1 and
#: >= the 2 TQ INFORMATION PROCESSING TIME, SJW 4 <= min(4, PHASE_SEG1).
SEGMENTS = {"prop_tq": 8, "phase1_tq": 5, "phase2_tq": 6, "sjw_tq": 4}


def bitrate_for(bit_ticks: int) -> float:
    """The NOMINAL BIT RATE of a bit time measured in ticks of this clock."""
    return CORE_HZ / bit_ticks


def overrides_for(bit_ticks: int) -> dict:
    """The `timing_for` overrides for this bit time.

    `tq_ns` is spelled out as well as `tq_per_bit` because `verif/vip/can.py`'s
    `timing_for` derives the time quantum from the *base* speed's bit rate and only
    then applies the overrides, so a speed it does not name needs the quantum given
    too. (That model is read-only for this task, so the workaround goes here.)
    """
    bitrate = bitrate_for(bit_ticks)
    return {
        "bitrate": bitrate,
        "tq_per_bit": TQ_PER_BIT,
        "tq_ns": 1e9 / bitrate / TQ_PER_BIT,
        **SEGMENTS,
    }


def measure(bit_ticks: int) -> dict:
    """Send one frame at this bit time and record what the checker said."""
    row = {
        "bit_ticks": bit_ticks,
        "bit_ns": bit_ticks * int(TICK_NS),
        "bitrate": f"{bitrate_for(bit_ticks):.0f}",
        "expressible": bit_ticks % SM_CYCLES_PER_BIT == 0,
        "sm_cycles_per_bit": SM_CYCLES_PER_BIT,
        "violations": 0, "rules": "", "acked": False, "node_saw": False,
        "worst_off_grid": 0, "usable": False, "note": "",
    }
    try:
        overrides = overrides_for(bit_ticks)
        bench = Bench(can_pin=DEFAULT_PIN, timing=overrides)
        check_bit_timing(bench.tk["timing"])          # never run off-spec silently
        bench.driver.load()
        bench.driver.clear_fdebug()
        result = bench.driver.send(0x123, b"\xde\xad")
    except CanError as exc:
        row["note"] = f"the program cannot time this bit time: {exc}"
        return row
    except ValueError as exc:
        row["note"] = f"no legal CAN timing on this tick grid: {exc}"
        return row
    rules = sorted({v.rule for v in bench.checker.violations})
    row.update(
        tq_ticks=bench.tk["tq_ticks"], clkdiv=bench.driver.clkdiv,
        violations=len(bench.checker.violations), rules=",".join(rules),
        acked=result.acked, node_saw=bool(bench.node_frames()),
        worst_off_grid=bench.checker.worst_off_grid,
        usable=(not bench.checker.violations and result.acked and not result.bit_errors),
        note=bench.checker.report() if bench.checker.violations else "",
    )
    return row


def sweep(bit_times=BIT_TIMES) -> list[dict]:
    """One row per bit time, shortest first."""
    return [measure(bit_ticks) for bit_ticks in bit_times]


def usable_rows(rows=None) -> list[dict]:
    rows = sweep() if rows is None else rows
    return [r for r in rows if r["usable"]]


def fastest_usable(rows=None) -> dict:
    rows = usable_rows(rows)
    if not rows:
        raise CanError("no bit time was usable: the sweep in metrics/fw-can-speed.csv says so")
    return rows[0]


#: The measured answers, recorded here so `fw/can/test_can.py` can fail when one of
#: them moves. Re-run `python -m fw.can.measure` to refresh, and say so in the commit.
#: 10 core cycles per bit is the shortest the loop can express (3 instructions and no
#: delay at CLKDIV 1) and 200 ns is 5 Mbit/s at 50 MHz -- five times the 1 Mbit/s that
#: classic CAN tops out at. The bit times in between are not all reachable: 25 cycles
#: (2 Mbit/s) is not a whole number of the program's 10 SM cycles, which is the only
#: granularity the `.define`d delays leave.
FASTEST_BIT_TICKS = 10
FASTEST_BITRATE = 5_000_000
FASTEST_CLK_DIV = 1
#: 1 Mbit/s is 50 core cycles, which *is* ten SM cycles times CLKDIV 5, so the top of
#: the classic-CAN range is exactly reachable.
CAN_CEILING_BIT_TICKS = 50
CAN_CEILING_CLK_DIV = 5


def write_csv(rows, path: Path = DEFAULT_CSV) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(FIELDS), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    args = parser.parse_args(argv)
    rows = sweep()
    print(f"{'ticks':>6} {'bit ns':>7} {'bitrate':>9} {'tq':>3} {'clkdiv':>6} {'expr':>4} "
          f"{'viol':>4} {'off':>3} {'ack':>5}  rules / note")
    for row in rows:
        print(f"{row['bit_ticks']:>6} {row['bit_ns']:>7} {row['bitrate']:>9} "
              f"{row.get('tq_ticks', 0):>3} {row.get('clkdiv', 0):>6} "
              f"{str(row['expressible']):>4} {row['violations']:>4} "
              f"{row['worst_off_grid']:>3} {str(row['acked']):>5}  "
              f"{row['rules'] or row['note'][:60]}")
    print(f"\nwritten: {write_csv(rows, args.csv)}")
    usable = usable_rows(rows)
    if usable:
        best = usable[0]
        print(f"fastest usable bit time: {best['bit_ticks']} core cycles = "
              f"{best['bit_ns']} ns = {best['bitrate']} bit/s (CLKDIV {best['clkdiv']})")
    else:
        print("no bit time was usable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
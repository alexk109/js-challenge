"""Run the lockstep fuzz over a range of seeds and record what happened.

    python -m verif.fuzz.campaign --seeds 1-20 --cycles 10000

One `lockstep.run()` per seed (the same code path `make fuzz` uses), one CSV row per seed in
`metrics/fuzz.csv`, one log line per seed, and a summary at the end that groups the mismatches by
the field they were found in, so that "every distinct mismatch" is visible at a glance. Nothing
here fixes anything: a mismatch is a finding.

Options that say what was compared and what the stimulus did:

| option | meaning |
|---|---|
| `--deep` / `--no-deep` | compare the deep fields (the default, since seeds 1-60 at 20k cycles were clean) or only the 37 shallow signals |
| `--conflict N` | `gen.gen(..., conflict=N)`: a guaranteed two-engine pin conflict at the end of every run, which is what makes the cross-engine pin rule reachable on every seed |
| `--pullup` | a `uio` pin the chip releases reads 1 |
| `--repro-dir DIR` | where a mismatching seed's repro goes (default `verif/fuzz/repros/`; the `make fuzz-plant5` / `make fuzz-deep-campaign` targets pass a directory outside the tree) |
| `--csv PATH` | where the rows go (default `metrics/fuzz.csv`) |
"""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verif.fuzz import lockstep                                     # noqa: E402

CSV_PATH = ROOT / "metrics" / "fuzz.csv"
CSV_COLUMNS = ["date", "commit", "harness", "seed", "cycles", "programs", "result",
               "first_divergence_cycle", "runtime_s", "deep", "pullup", "conflict", "fields"]


def parse_seeds(text: str) -> list[int]:
    out: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = part.split("-")
            out.extend(range(int(lo), int(hi) + 1))
        elif part:
            out.append(int(part))
    return out


def append_csv(rows: list[dict[str, Any]], path: Path = None) -> Path:
    path = path or CSV_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0
    with open(path, "a", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
        if new_file:
            writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in CSV_COLUMNS})
    return path


def summary(results: list[dict[str, Any]], stamp: str, cycles: int) -> str:
    bad = [r for r in results if r["status"] != "pass"]
    lines = ["", "=" * 78,
             f"{len(results)} seeds x {cycles} cycles, git {stamp}, "
             f"{time.strftime('%Y-%m-%d %H:%M:%S')}",
             f"{len(results) - len(bad)} seeds matched, {len(bad)} found a mismatch",
             ""]
    by_field: dict[str, list[dict[str, Any]]] = {}
    for r in bad:
        by_field.setdefault(r["mismatch"]["name"], []).append(r)
    if not bad:
        lines.append("no mismatch")
    for field, group in sorted(by_field.items()):
        lines.append(f"{field}: {len(group)} seed(s) -- "
                     f"seeds {', '.join(str(r['seed']) for r in group)}")
        for r in group:
            m = r["mismatch"]
            lines.append(f"    seed {r['seed']:>4} cycle {m['cycle']:>6} "
                         f"rtl={m['rtl_hex']} iss={m['iss_hex']}  repro {r['repro']}")
            lines.append(f"           report {r['report']}")
    total_sm = sum(r["coverage"]["sm_cycles"][0] + r["coverage"]["sm_cycles"][1]
                   for r in results)
    kinds = set()
    for r in results:
        kinds |= set(r["coverage"]["instructions_executed"])
    lines += ["", f"SM cycles executed in total: {total_sm}; instruction kinds seen: "
                  f"{', '.join(sorted(kinds))}"]
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="run the lockstep fuzz over many seeds")
    ap.add_argument("--seeds", default="1-20")
    ap.add_argument("--cycles", type=int, default=10000)
    ap.add_argument("--sim", default="verilator")
    ap.add_argument("--jobs", default="-j2")
    ap.add_argument("--no-csv", action="store_true")
    ap.add_argument("--out", default=None, help="write the full results JSON here")
    ap.add_argument("--csv", default=None, help="CSV to append to (default metrics/fuzz.csv)")
    ap.add_argument("--label", default="lockstep", help="the harness column in the CSV")
    ap.add_argument("--plant", type=int, default=0,
                    help="run against verif/fuzz/planted with this FUZZ_PLANT_RTL plant "
                         "(harness sensitivity, not a real bug)")
    deep = ap.add_mutually_exclusive_group()
    deep.add_argument("--deep", dest="deep", action="store_true", default=None,
                      help="compare the deep fields too (lockstep.DEEP_DEFAULT, which is True)")
    deep.add_argument("--no-deep", dest="deep", action="store_false",
                      help="only the 37 shallow signals of the original lockstep")
    ap.add_argument("--pullup", action="store_true",
                    help="a uio pin the chip releases reads 1 (an open-drain net with a pull-up)")
    ap.add_argument("--conflict", type=int, default=0, metavar="N",
                    help="append a guaranteed two-engine pin conflict of N cycles to every run")
    ap.add_argument("--repro-dir", default=None,
                    help="where repros of a mismatch go (default verif/fuzz/repros/, which is "
                         "committed; point a sensitivity run somewhere else to keep 20 planted "
                         "repros out of the tree -- the make targets use "
                         "FUZZ_REPRO_DIR, default /tmp/fuzz-repros)")
    ap.add_argument("--pinloop", action="store_true", help="use the pad-model stimulus")
    ap.add_argument("--fifofill", action="store_true", help="use the FIFO-storage stimulus")
    args = ap.parse_args(argv)

    stamp = lockstep.git_stamp()
    seeds = parse_seeds(args.seeds)
    label = args.label if not args.plant else f"{args.label}-plant{args.plant}"
    print(f"lockstep campaign: seeds {seeds[0]}..{seeds[-1]} ({len(seeds)}), "
          f"{args.cycles} cycles each, simulator {args.sim}, build {label}, deep {args.deep}, "
          f"pull-up {args.pullup}, conflict {args.conflict}, git {stamp}, "
          f"{time.strftime('%Y-%m-%d %H:%M:%S')}", flush=True)
    results: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    for seed in seeds:
        t0 = time.time()
        result = lockstep.run(seed=seed, cycles=args.cycles, sim=args.sim, quiet=True,
                              jobs=args.jobs, planted=bool(args.plant), plant=args.plant or 1,
                              deep=args.deep, pullup=args.pullup, conflict=args.conflict,
                              pinloop=args.pinloop, fifofill=args.fifofill,
                              repro_dir=Path(args.repro_dir) if args.repro_dir else None)
        result["wall_s"] = round(time.time() - t0, 1)
        results.append(result)
        status = result["status"]
        line = f"seed {seed:>4}: {status:<9} {result['cycles_run']:>6} cycles " \
               f"{result['runtime_s']:>6}s wall {result['wall_s']:>5}s fields {result['fields']}"
        if status == "pass":
            cov = result["coverage"]
            line += (f"  SM {cov['sm_cycles'][0]}/{cov['sm_cycles'][1]} "
                     f"stalls {cov['stall_cycles'][0]}/{cov['stall_cycles'][1]} "
                     f"kinds {len(cov['instructions_executed'])} "
                     f"host w/r {cov['bus_writes']}/{cov['bus_reads']}")
        else:
            m = result["mismatch"]
            line += f"  first divergence cycle {m['cycle']} {m['name']}: " \
                    f"rtl={m['rtl_hex']} iss={m['iss_hex']}"
            line += f"  repro {result['repro']}"
        print(line, flush=True)
        rows.append({
            "date": time.strftime("%Y-%m-%d"), "commit": stamp, "harness": label,
            "seed": seed, "cycles": args.cycles, "programs": 2, "result": status,
            "first_divergence_cycle": result["first_divergence_cycle"] or "",
            "runtime_s": result["runtime_s"],
            "deep": result.get("deep", ""), "pullup": result.get("pullup", ""),
            "conflict": args.conflict, "fields": result.get("fields", ""),
        })
    text = summary(results, stamp, args.cycles)
    print(text)
    if not args.no_csv:
        path = append_csv(rows, Path(args.csv) if args.csv else CSV_PATH)
        print(f"\nwrote {len(rows)} rows to {path}")
    if args.out:
        Path(args.out).write_text(json.dumps(results, indent=1))
        print(f"wrote {args.out}")
    return 1 if any(r["status"] != "pass" for r in results) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
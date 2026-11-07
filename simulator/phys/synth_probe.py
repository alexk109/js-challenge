#!/usr/bin/env python3
"""`make synth-probe [PROBE=name]`: synthesis-only area probes.

Runs the LibreLane-pinned Yosys (slang frontend) on each probe in
phys/probes/probes.yaml, maps to sg13cmos5l_stdcell at the typical corner
using the PDK's own latch map, and appends rows to metrics/synth.csv.
"""

import argparse
import csv
import datetime
import os
import re
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
LL = ROOT / "tools" / "ll"
OUT = ROOT / "runs" / "synth-probe"
CSV = ROOT / "metrics" / "synth.csv"
COLUMNS = ["date", "commit", "probe", "top", "params", "cells", "seq_cells",
           "area_um2", "seq_area_um2", "comb_area_um2", "top_cells"]

PDK = os.path.join(os.environ.get("PDK_ROOT", os.path.expanduser("~/ttsetup/pdk")), "ihp-sg13cmos5l")
LIB = f"{PDK}/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_typ_1p20V_25C.lib"
LATCH_MAP = f"{PDK}/libs.tech/librelane/sg13cmos5l_stdcell/latch_map.v"
SEQ = re.compile(r"sg13cmos5l_(dfrbp|dfrbpq|sdf|dlh|dll|lgcp|slgcp)")


def yosys_script(p, name):
    params = " ".join(f"-G{k}={v}" for k, v in p.get("params", {}).items())
    files = " ".join(str(ROOT / f) for f in p["files"])
    return f"""
plugin -i slang
read_liberty -lib {LIB}
read_slang --top {p['top']} {params} {files}
hierarchy -top {p['top']}
synth -flatten -top {p['top']}
techmap -map {LATCH_MAP}
dfflibmap -liberty {LIB}
abc -liberty {LIB} -D 20000
opt_clean -purge
tee -o {OUT}/{name}.stat stat -liberty {LIB}
"""


def parse_stat(path):
    text = open(path).read()
    area = float(re.search(r"Chip area for (?:top )?module.*?:\s*([\d.]+)", text).group(1))
    cells = {}
    for mt in re.finditer(r"^\s+(\d+)\s+(?:[\d.E+-]+\s+)?(sg13cmos5l_\w+)\s*$", text, re.M):
        cells[mt.group(2)] = int(mt.group(1))
    if not cells:  # older stat format: "<name> <count>"
        for mt in re.finditer(r"^\s+(sg13cmos5l_\w+)\s+(\d+)\s*$", text, re.M):
            cells[mt.group(1)] = int(mt.group(2))
    return area, cells


def cell_areas():
    lib = open(LIB).read()
    return {n: float(a) for n, a in re.findall(r'cell\s*\(\s*"?(\w+)"?\s*\)\s*\{.*?area\s*:\s*([\d.]+)', lib, re.S)}


def main():
    ap = argparse.ArgumentParser()
    # Comma-separated, so one invocation can price a family of related probes in one toolchain entry
    # instead of starting the toolchain once per probe.
    ap.add_argument("--probe", default="")
    args = ap.parse_args()

    spec = yaml.safe_load(open(ROOT / "phys" / "probes" / "probes.yaml"))["probes"]
    names = [n for n in args.probe.split(",") if n] if args.probe else list(spec)
    OUT.mkdir(parents=True, exist_ok=True)

    jobs = []
    for n in names:
        ys = OUT / f"{n}.ys"
        ys.write_text(yosys_script(spec[n], n))
        jobs.append(f"yosys -q -l {OUT}/{n}.log -s {ys} >/dev/null 2>&1 || echo FAILED {n}")
    # One toolchain entry for all probes; run them in parallel inside it.
    script = "\n".join(f"( {j} ) &" for j in jobs) + "\nwait\n"
    p = subprocess.run([str(LL), "bash", "-c", script], capture_output=True, text=True)
    sys.stdout.write(p.stdout)

    areas = cell_areas()
    commit = subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                            capture_output=True, text=True).stdout.strip() or "nogit"
    new = not CSV.exists() or CSV.stat().st_size == 0
    with open(CSV, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        if new:
            w.writeheader()
        for n in names:
            stat = OUT / f"{n}.stat"
            if not stat.exists():
                print(f"{n}: no result, see {OUT / (n + '.log')}")
                continue
            area, cells = parse_stat(stat)
            seq = {c: k for c, k in cells.items() if SEQ.match(c)}
            seq_area = sum(areas.get(c, 0) * k for c, k in seq.items())
            top = sorted(cells.items(), key=lambda kv: -kv[1] * areas.get(kv[0], 0))[:4]
            row = {
                "date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
                "commit": commit,
                "probe": n,
                "top": spec[n]["top"],
                "params": " ".join(f"{k}={v}" for k, v in spec[n].get("params", {}).items()),
                "cells": sum(cells.values()),
                "seq_cells": sum(seq.values()),
                "area_um2": round(area),
                "seq_area_um2": round(seq_area),
                "comb_area_um2": round(area - seq_area),
                "top_cells": " ".join(f"{c.replace('sg13cmos5l_', '')}:{k}" for c, k in top),
            }
            w.writerow(row)
            print(f"{n:18s} cells={row['cells']:6d} seq={row['seq_cells']:5d} "
                  f"area={row['area_um2']:8d} um2 (seq {row['seq_area_um2']}, comb {row['comb_area_um2']})  {row['top_cells']}")
    return p.returncode


if __name__ == "__main__":
    sys.exit(main())

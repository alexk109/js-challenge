"""Shrink a failing lockstep stimulus to a minimal one that still fails the same way.

Input is a repro written by `verif/fuzz/lockstep.py`
(`verif/fuzz/repros/*.json`); output is `<repro>.min.json` plus a short report. What it shrinks:

* **the cycle count**, down to the first divergence cycle (the run stops right after it);
* **the host ops**, with a delta-debugging pass (ddmin): chunks of ops are dropped, halving the
  chunk size while progress is being made, and an op that is not needed for the mismatch goes
  away for good;
* **the IMEM writes** are ops like any other, so dropping them drops instructions, which is what
  makes the surviving program readable.

A candidate "still fails" only if it still fails **in the same field**, so the minimizer cannot
quietly shrink a run onto a different (or shallower) bug.

Every candidate is a separate simulation; they are evaluated in batches through
`FUZZ_STIMULI`, so a few hundred of them cost one simulator process.

    python -m verif.fuzz.minimize verif/fuzz/repros/rtl-seed3-c000123-eng0_pc.json
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Optional

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verif.fuzz import gen, lockstep                                # noqa: E402

HERE = Path(__file__).resolve().parent


def evaluate(stims: list[gen.Stimulus], sim: str = "verilator", planted: bool = False,
             plant: int = 1, jobs: str = "-j2") -> list[dict[str, Any]]:
    """Run a list of stimuli in one simulator process; one result dict each, in order."""
    if not stims:
        return []
    payload = json.dumps([json.loads(s.to_json()) for s in stims])
    with tempfile.TemporaryDirectory(prefix="fuzzmin-") as tmp:
        listfile = Path(tmp) / "stimuli.json"
        listfile.write_text(payload)
        result = lockstep.run(seed=stims[0].seed, cycles=max(s.cycles for s in stims), sim=sim,
                              planted=planted, plant=plant, quiet=True, jobs=jobs,
                              stimuli=listfile)
    return result.get("results", [result])


def still_fails(result: dict[str, Any], field: str) -> bool:
    return (result.get("status") == "mismatch"
            and result.get("mismatch", {}).get("name") == field)


def shrink_cycles(stim: gen.Stimulus, field: str, sim: str, planted: bool, plant: int,
                  jobs: str) -> tuple[gen.Stimulus, list[dict[str, Any]]]:
    """Cut the run to just past the first divergence, if it still fails there."""
    base = evaluate([stim], sim=sim, planted=planted, plant=plant, jobs=jobs)[0]
    if base.get("status") != "mismatch":
        return stim, [base]
    div = base["first_divergence_cycle"]
    want = div + 1
    if want >= stim.cycles:
        return stim, [base]
    for length in (want, want + 2, want + 4, want + 16, stim.cycles):
        if length > stim.cycles:
            break
        shorter = stim.copy(cycles=length, ops=[o for o in stim.ops if o[0] <= length])
        result = evaluate([shorter], sim=sim, planted=planted, plant=plant, jobs=jobs)[0]
        if still_fails(result, field):
            return shorter, [result]
    return stim, [base]


def _chunks(items: list, n: int) -> list[list]:
    """Split `items` into n nearly equal contiguous chunks."""
    size = max(1, len(items) // n)
    return [items[i:i + size] for i in range(0, len(items), size)] or [[]]


def ddmin_ops(stim: gen.Stimulus, field: str, sim: str, planted: bool, plant: int,
              budget: int, jobs: str) -> tuple[gen.Stimulus, dict[str, Any]]:
    """Delta debugging over the op list: drop chunks of ops while the mismatch survives."""
    ops = list(stim.ops)
    runs = 0
    start_ops = len(ops)
    n = 2
    while len(ops) > 1 and runs < budget:
        chunks = _chunks(ops, min(n, len(ops)))
        cands = []
        for chunk in chunks:
            drop = set(id(o) for o in chunk)
            kept = [o for o in ops if id(o) not in drop]
            if len(kept) < len(ops):
                cands.append(gen.Stimulus(stim.seed, stim.cycles, kept, dict(stim.summary),
                                          build=stim.build, mode=dict(stim.mode),
                                          ext_low=stim.ext_low))
        if not cands:
            break
        if not cands:
            break
        results = evaluate(cands, sim=sim, planted=planted, plant=plant, jobs=jobs)
        runs += len(cands)
        shrink = next((c for c, r in zip(cands, results) if still_fails(r, field)), None)
        if shrink is not None:
            ops = shrink.ops
            n = max(n - 1, 2)
        elif n >= len(ops):
            break
        else:
            n = min(n * 2, len(ops))
    out = gen.Stimulus(stim.seed, stim.cycles, ops, dict(stim.summary), build=stim.build,
                       mode=dict(stim.mode), ext_low=stim.ext_low)
    return out, {"ops_before": start_ops, "ops_after": len(ops), "runs": runs,
                 "converged": runs < budget}


def minimize(path: str, out: Optional[str] = None, sim: str = "verilator", planted: bool = False,
             plant: int = 1, budget: int = 600, jobs: str = "-j2",
             quiet: bool = False) -> dict[str, Any]:
    """Minimize the repro at `path`; write the result and return a report dict."""
    stim = gen.Stimulus.load(path)
    if not planted and stim.build != "rtl":
        # the repro was made against a planted copy; replay it against the same one
        planted = True
        plant = int(stim.build.replace("planted", "") or plant)
        print(f"minimize: {path} was made against {stim.build}; replaying against that build")
    t0 = time.time()
    base = evaluate([stim], sim=sim, planted=planted, plant=plant, jobs=jobs)[0]
    if base.get("status") != "mismatch":
        raise SystemExit(f"{path}: does not mismatch with {label_of(planted, plant)} "
                         f"({base.get('status')}): nothing to minimize")
    field = base["mismatch"]["name"]
    cycle = base["first_divergence_cycle"]
    report: dict[str, Any] = {
        "input": str(path), "field": field, "divergence_cycle": cycle,
        "rtl": base["mismatch"]["rtl_hex"], "iss": base["mismatch"]["iss_hex"],
        "simulator": sim, "rtl_build": label_of(planted, plant),
        "cycles_before": stim.cycles, "ops_before": len(stim.ops),
        "git": lockstep.git_stamp(), "date": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if not quiet:
        print(f"minimize {path}: {len(stim.ops)} ops, {stim.cycles} cycles, first mismatch at "
              f"cycle {cycle} in {field} (rtl {base['mismatch']['rtl_hex']}, "
              f"iss {base['mismatch']['iss_hex']})")
    stim, _ = shrink_cycles(stim, field, sim, planted, plant, jobs)
    stim, stats = ddmin_ops(stim, field, sim, planted, plant, budget, jobs)
    report.update(stats)
    report["cycles_after"] = stim.cycles
    report["ops_after"] = len(stim.ops)
    report["runtime_s"] = round(time.time() - t0, 1)
    final = evaluate([stim], sim=sim, planted=planted, plant=plant, jobs=jobs)[0]
    report["divergence_cycle_after"] = final.get("first_divergence_cycle")
    report["still_mismatches"] = still_fails(final, field)
    stem = Path(str(path).removesuffix(".json"))
    out_path = Path(out) if out else Path(str(stem) + ".min.json")
    stim.minimized = True
    stim.save(out_path)
    report["output"] = str(out_path)
    report_path = Path(str(stem) + ".min.txt")
    report_path.write_text(text_report(stim, report) + "\n")
    report["report"] = str(report_path)
    if not quiet:
        print(text_report(stim, report))
        print(f"wrote {out_path} and {report_path}")
    return report


def label_of(planted: bool, plant: int) -> str:
    return f"planted{plant}" if planted else "rtl"


def text_report(stim: gen.Stimulus, report: dict[str, Any]) -> str:
    lines = [
        "=" * 78,
        f"minimized lockstep repro: {report['field']} at cycle "
        f"{report['divergence_cycle_after']}",
        f"  rtl={report['rtl']} iss={report['iss']} ({report['rtl_build']}, "
        f"{report['simulator']})",
        f"  ops {report['ops_before']} -> {report['ops_after']}, cycles {report['cycles_before']}"
        f" -> {report['cycles_after']}, {report['runs']} simulator runs "
        f"({'converged' if report.get('converged') else 'budget exhausted'}), "
        f"{report['runtime_s']}s",
        f"  still mismatches in {report['field']}: {report.get('still_mismatches')}",
        f"  git {report['git']}, {report['date']}",
        "",
        "surviving host ops (cycle, kind, address, data):",
    ]
    for cycle, kind, addr, data in stim.ops:
        lines.append(f"  {cycle:6d}  {kind} {addr:#04x} {data:#010x}")
    lines += ["", "replay it with:", f"  python -m verif.fuzz.lockstep --repro {report['output']}"]
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="shrink a failing lockstep stimulus")
    ap.add_argument("repro", help="a repro JSON written by lockstep.py")
    ap.add_argument("--out", default=None)
    ap.add_argument("--sim", default="verilator", choices=["verilator", "icarus"])
    ap.add_argument("--planted", action="store_true")
    ap.add_argument("--plant", type=int, default=1)
    ap.add_argument("--budget", type=int, default=600, help="simulator runs to spend")
    ap.add_argument("--jobs", default="-j2")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    report = minimize(args.repro, out=args.out, sim=args.sim, planted=args.planted,
                      plant=args.plant, budget=args.budget, jobs=args.jobs, quiet=args.quiet)
    # exit 0 as long as the minimized stimulus still mismatches, shrunk or not
    return 0 if report.get("still_mismatches") else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
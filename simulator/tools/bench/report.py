"""`make bench-report`: build the evidence-bench report in one go (about 5-10 minutes, nothing long).

Steps, each a normal CLI call against the current store (BENCH_STORE / --store): import all; analytic on chip + the cap19
and firmware workloads; iss, iss_b, iss_p on every protocol they share; iss_p bridges (engine_route on, which they need) and mixes at their default params;
synth for the MVP and sram2p; `check`; `export --html`; `snapshot latest`.  Then a one-screen summary: record counts per
level, findings by verdict, and the `off` findings that are not in smoke_allow.json.

    python -m tools.bench.report [--store PATH] [--html OUT] [--skip-run]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

from . import smoke

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_HTML = HERE / "snapshots" / "latest.html"


def bench(store: str, *args: str, timeout: int = 900) -> subprocess.CompletedProcess:
    t0 = time.time()
    p = subprocess.run([sys.executable, "-m", "tools.bench", "--store", store, *args], cwd=ROOT, capture_output=True,
                       text=True, timeout=timeout, env={**os.environ, "PYTHONPATH": str(ROOT)})
    print(f"  [{time.time() - t0:5.1f}s] bench {' '.join(args)[:100]}", flush=True)
    return p


def workload_ids(backend: str, kinds: tuple) -> list[str]:
    from .backends import load
    return [w.id for w in load(backend).workloads() if w.kind in kinds]


def steps() -> list[tuple]:
    shared = set(workload_ids("iss", ("protocol",))) & set(workload_ids("iss_b", ("protocol",)))
    protos = [w for w in workload_ids("iss_p", ("protocol",)) if w in shared]
    bridges = workload_ids("iss_p", ("bridge",))
    mixes = workload_ids("iss_p", ("mix",))
    return [("import",),
            ("run", "--backend", "analytic", "--workload", ",".join(smoke.analytic_workloads())),
            ("run", "--backend", "iss,iss_b,iss_p", "--workload", ",".join(protos), "--jobs", "4"),
            ("run", "--backend", "iss_p", "--workload", ",".join(bridges), "--set", "engine_route=true", "--jobs", "4"),
            ("run", "--backend", "iss_p", "--workload", ",".join(mixes), "--jobs", "4"),
            ("run", "--backend", "synth", "--workload", "chip"),
            ("run", "--backend", "synth", "--workload", "chip", "--set", "imem_tech=sram2p")]


def summarize(findings: list[dict], recs: list[dict], allow: list[dict]) -> str:
    lines = ["", "== bench-report summary ==", f"records: {len(recs)}"]
    by_level = Counter(r["level"] for r in recs)
    lines.append("  per level: " + ", ".join(f"{k} {v}" for k, v in sorted(by_level.items())))
    errs = sum(1 for r in recs if r["status"] == "error")
    if errs:
        lines.append(f"  error records: {errs}")
    verd = Counter(f["verdict"] for f in findings)
    lines.append("findings: " + ", ".join(f"{k} {verd.get(k, 0)}" for k in ("match", "close", "identity", "off", "one-sided")))
    unexpected = [f for f in findings if f["verdict"] == "off" and not any(smoke.covers(e, f) for e in allow)]
    lines.append(f"unexpected offs (not in smoke_allow.json): {len(unexpected)}")
    for f in unexpected[:12]:
        b = f["b"]["backend"] if f["b"] else "-"
        lines.append(f"  {f['workload']} {f['metric']}  {f['a']['backend']} vs {b}: {f['detail'][:80]}")
    if len(unexpected) > 12:
        lines.append(f"  ... and {len(unexpected) - 12} more (python -m tools.bench check)")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--store", default=os.environ.get("BENCH_STORE") or str(HERE / "results" / "records.jsonl"))
    ap.add_argument("--html", default=str(DEFAULT_HTML))
    ap.add_argument("--skip-run", action="store_true", help="only check, export and snapshot the store as it is")
    a = ap.parse_args(argv)
    t0 = time.time()
    print(f"bench-report: store {a.store}", flush=True)
    if not a.skip_run:
        for s in steps():
            p = bench(a.store, *s)
            if p.returncode != 0:
                print(f"bench-report: `bench {' '.join(s)[:80]}` exited {p.returncode}:\n{p.stdout[-800:]}\n{p.stderr[-800:]}")
                if s[0] == "import":
                    return 1
    p = bench(a.store, "check", "--json", "--all")
    findings = json.loads(p.stdout)["findings"]
    Path(a.html).parent.mkdir(parents=True, exist_ok=True)
    bench(a.store, "export", "--html", a.html)
    bench(a.store, "snapshot", "latest")
    from .store import load
    recs = [json.loads(r.to_json()) for r in load(a.store)]
    print(summarize(findings, recs, smoke.load_allow()))
    print(f"\nhtml: {a.html}\nelapsed: {time.time() - t0:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())

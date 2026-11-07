"""`make bench-smoke`: the whole bench end to end (about 5 s warm, longer cold), then the cross-level assertions.

Runs (into a throw-away store, `--store`): import all; analytic on chip, the cap19 workloads, the firmware workloads
and the host port; iss and iss_b on uart_tx, spi, i2c and spi_host_port; one rtl smoke (spi); then `check`.
Asserts that the checks matched across analytic / iss / iss_b / rtl / import and that every `off` is in
`smoke_allow.json` (known results, each with the reason). An allowlisted entry that no longer fires is reported.

    python -m tools.bench.smoke [--store PATH] [--skip-run]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ALLOW = HERE / "smoke_allow.json"
SIM_PROTOCOLS = ("uart_tx", "spi", "i2c")
#: backends (prefix) that must each take part in at least one match/close/identity-free agreement
EXPECTED_SIDES = ("analytic", "iss", "iss_b", "rtl", "import")


def bench(*args: str, store: str, timeout: int = 280) -> subprocess.CompletedProcess:
    cmd = [sys.executable, "-m", "tools.bench", "--store", store, *args]
    t0 = time.time()
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=timeout,
                       env={**os.environ, "PYTHONPATH": str(ROOT)})
    print(f"  [{time.time() - t0:5.1f}s] bench {' '.join(args)[:110]}")
    return p


def analytic_workloads() -> list[str]:
    data = json.loads((ROOT / "tools" / "archsim" / "data" / "inputs.json").read_text())
    tags = data.get("workload_tags", {})
    cap19 = [w["id"] for w in data["workloads"] if "cap19" in (w.get("tags") or [])]
    from tools.bench.backends.analytic import ANALYTIC_TO_SIM
    del tags
    return ["chip", "spi_host_port", *dict.fromkeys([*cap19, *ANALYTIC_TO_SIM])]


def have_rtl() -> bool:
    """The rtl level needs Verilator 5.x (and cocotb); without it the smoke test skips that level."""
    import re
    import shutil
    exe = shutil.which("verilator")
    if not exe:
        return False
    out = subprocess.run([exe, "--version"], capture_output=True, text=True).stdout
    m = re.search(r"Verilator (\d+)", out)
    return bool(m and int(m.group(1)) >= 5)


def run_all(store: str, rtl: bool = True) -> None:
    for args in (("import",),
                 ("run", "--backend", "analytic", "--workload", ",".join(analytic_workloads())),
                 ("run", "--backend", "iss,iss_b", "--workload", ",".join([*SIM_PROTOCOLS, "spi_host_port"]), "--jobs", "4"),
                 *((("run", "--backend", "rtl", "--workload", "spi"),) if rtl else ())):
        p = bench(*args, store=store, timeout=290)
        if p.returncode != 0:
            raise SystemExit(f"smoke: `bench {' '.join(args)}` failed ({p.returncode}):\n{p.stdout[-1500:]}\n{p.stderr[-1500:]}")


def load_allow() -> list[dict]:
    return json.loads(ALLOW.read_text()) if ALLOW.exists() else []


def covers(entry: dict, f: dict) -> bool:
    if entry["workload"] != f["workload"] or entry["metric"] != f["metric"]:
        return False
    if entry.get("config_hash") != f["config_hash"] or entry.get("workload_params") != f["workload_params"]:
        return False           # an entry names one config and one operating point; another one is a new finding
    sides = {f["a"]["backend"], f["b"]["backend"] if f["b"] else ""}
    return all(any(s.startswith(want) for s in sides) for want in entry.get("backends", []))


def assess(findings: list[dict], allow: list[dict], rtl_expected: bool = True) -> tuple[list[str], dict]:
    """(problems, stats). Pure function of the findings, so the tests can drive it."""
    problems: list[str] = []
    verdicts = Counter(f["verdict"] for f in findings)
    agreed = Counter()
    for f in findings:
        if f["verdict"] in ("match", "close"):
            for s in (f["a"], f["b"]):
                key = s["backend"].split(":")[0]
                agreed[key] += 1
    for side in (EXPECTED_SIDES if rtl_expected else tuple(x for x in EXPECTED_SIDES if x != "rtl")):
        if not agreed[side]:
            problems.append(f"no match/close finding involves {side}")
    used = set()
    for f in findings:
        if f["verdict"] != "off":
            continue
        hit = [i for i, e in enumerate(allow) if covers(e, f)]
        used.update(hit)
        if not hit:
            sides = f"{f['a']['backend']} vs {f['b']['backend']}"
            problems.append(f"unexpected off: {f['workload']} {f['metric']} {sides}: {f['detail']}")
    stats = {"verdicts": dict(verdicts), "agreed_by_side": dict(agreed),
             "allowlisted_off": sorted(used), "stale_allow": [i for i in range(len(allow)) if i not in used]}
    return problems, stats


def describe_stale(allow: list[dict], idx: list[int]) -> list[str]:
    out = []
    for i in idx:
        e = allow[i]
        why = str(e.get("reason", ""))
        out.append(f"{e.get('workload')}/{e.get('metric')} [{'+'.join(e.get('backends', []))}] "
                   f"{why[:60]}{'...' if len(why) > 60 else ''}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--store")
    ap.add_argument("--skip-run", action="store_true", help="only assess the store as it is")
    ap.add_argument("--no-rtl", action="store_true", help="skip the Verilator-based rtl level (automatic when verilator is not on PATH)")
    args = ap.parse_args(argv)
    store = args.store or str(Path(tempfile.mkdtemp(prefix="bench-smoke-")) / "records.jsonl")
    print(f"smoke: store {store}")
    t0 = time.time()
    rtl = not args.no_rtl and have_rtl()
    if not rtl:
        print("smoke: rtl level SKIPPED (no Verilator 5.x on PATH, or --no-rtl); the other levels still run")
    if not args.skip_run:
        run_all(store, rtl)
    p = bench("check", "--json", "--all", store=store)
    if p.returncode != 0:
        raise SystemExit(f"smoke: check failed:\n{p.stderr[-1500:]}")
    findings = json.loads(p.stdout)["findings"]
    allow = load_allow()
    problems, stats = assess(findings, allow, rtl)
    print(f"smoke: {len(findings)} findings in {time.time() - t0:.0f} s: {stats['verdicts']}")
    print(f"smoke: match/close by side: {stats['agreed_by_side']}")
    print(f"smoke: allowlisted off findings seen: {len(stats['allowlisted_off'])}; "
          f"stale allowlist entries: {len(stats['stale_allow'])} "
          "(expected when the smoke store has no flow/synth runs: those entries only fire on a full store)")
    for line in describe_stale(allow, stats["stale_allow"]):
        print("smoke:   stale:", line)
    for pr in problems:
        print("smoke: PROBLEM:", pr)
    if not problems:
        print("smoke: OK")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())

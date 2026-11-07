#!/usr/bin/env python
# SPDX-License-Identifier: Apache-2.0
#
# Re-apply the 7 mutations of docs/mvp-verification-report.md `### 7` to a copy of the real
# `src/host_spi.sv` and run the whole 16-test blind bench against each one.  A mutant that
# still passes every test is a hole in the bench; this script exits 1 if there is one.
#
#   test/host/mutants/run.py                       all 7, from the committed copies
#   test/host/mutants/run.py --only M4,M6          a subset
#   test/host/mutants/run.py --dir DIR --only X0   the no-op negative control (selftest.py)
#   test/host/mutants/run.py --csv PATH            one row per mutant for a CSV record
#
# Nothing here touches src/ or the bench: the mutant is compiled *as the DUT*, which is what
# test/host/Makefile's DUTFILE variable is for.  `make -C test/host host-mutate` is the same
# thing.
#
# Compute: every run is a cocotb simulation, so run this through your job queue if you have one
#   .venv/bin/python test/host/mutants/run.py
#
# What makes a row in the table trustworthy -- all three are checked, and a row that
# fails any of them is an ERROR, not a result:
#   * the mutant's sha256 is printed next to its name, so the file that was compiled is
#     identifiable after the fact;
#   * this run's own build log must contain an iverilog command line naming the mutant.
#     `make clean` runs first, because cocotb's Makefile.sim only rebuilds
#     sim_build/sim.vvp when a source is *newer* than it, and every mutation file is
#     written by generate.py in one go -- so without the clean, mutant N+1 silently runs
#     against mutant N's binary;
#   * the compiled image sim_build/sim.vvp itself must name the mutant's path and no other
#     host_spi*.sv (iverilog records source paths in the vvp).  That is what the tests
#     actually ran against, as opposed to what we asked for;
#   * the kill counts come from test/host/results.xml, which cocotb writes per test case,
#     not from the summary line, so a mis-counted summary cannot hide a survivor;
#   * the full simulator output of every run is kept next to this script
#     (test/host/results-mutants/<id>.log) for the failure messages.

from __future__ import annotations

import argparse
import csv
import hashlib
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from mutations import MUTATIONS, NEGATIVE_CONTROL, DUT_NAME, Mutation  # noqa: E402

HOST_DIR = HERE.parent              # test/host
ROOT = HOST_DIR.parents[1]          # the repo root
RAW = HOST_DIR / "results-mutants"  # per-mutant simulator logs, not committed


@dataclass
class Result:
    target: "Target"
    tests: int = 0
    passed: int = 0
    failed: int = 0
    skipped: int = 0
    failed_tests: list[str] = field(default_factory=list)
    first_failure: str = ""
    compiled: str = ""              # the iverilog line, verbatim
    bench_line: str = ""            # the bench's own "host-test: compiled ..." line
    target_sha: str = ""            # sha256 of the file we pointed make at
    vvp_duts: list[str] = field(default_factory=list)   # DUT paths inside sim.vvp
    exit_status: int = -1
    error: str = ""

    @property
    def killed(self) -> bool:
        """True only if a test actually failed.  An ERROR is not a kill: a mutant that does
        not compile has not been shown to be detectable, and must not count as one."""
        return self.failed > 0


@dataclass
class Target:
    mut: Mutation
    path: Path
    expect: str                     # "kill" or "survive"
    title: str


def targets(args) -> list[Target]:
    only = {i for i in args.only.split(",") if i}
    base = HERE if args.dir is None else args.dir
    # --extra alone means "just these": otherwise --dir <tmp> would drag in all 7 M
    # mutations as well, and they are not there.
    named = bool(only or args.extra)
    out: list[Target] = []
    for mut in MUTATIONS:
        if named and not any(mut.id == i or mut.id.startswith(i) for i in only):
            continue
        out.append(Target(mut=mut, path=base / mut.id / DUT_NAME, expect="kill",
                          title=mut.title))
    if args.negative_control:
        out.append(Target(mut=NEGATIVE_CONTROL,
                          path=base / NEGATIVE_CONTROL.id / DUT_NAME,
                          expect="survive", title=NEGATIVE_CONTROL.title))
    # An id that is not in the manifest: selftest.py uses this for a hand-built mutant that
    # iverilog must reject.  It is run like any other and has to be measured, not trusted.
    for mid in args.extra:
        out.append(Target(mut=Mutation(id=mid, title=f"(not in the manifest) {mid}"),
                          path=base / mid / DUT_NAME, expect="kill", title=mid))
    if only and not out:
        raise SystemExit(f"no mutation matches {sorted(only)}; known ids: "
                         f"{[m.id for m in MUTATIONS]}")
    return out


def dut_paths_in(vvp: Path) -> list[str]:
    """Every `*host_spi*.sv` path iverilog recorded inside the compiled `sim.vvp`.

    iverilog stores the source path of every module in the vvp image, so this says what
    the binary the tests actually ran against was built from -- not what we asked for.
    """
    blob = vvp.read_bytes() if vvp.exists() else b""
    found = set(re.findall(rb"/[A-Za-z0-9_./+-]*host_spi[A-Za-z0-9_.]*\.sv", blob))
    return sorted(p.decode() for p in found)


def run_one(target: Target, keep_raw: bool = True) -> Result:
    res = Result(target=target)
    if not target.path.exists():
        res.error = f"{target.path} does not exist; run test/host/mutants/generate.py"
        return res
    src_sha = hashlib.sha256(target.path.read_bytes()).hexdigest()
    res.target_sha = src_sha
    # The absolute path is what iverilog prints and what the vvp records; DUTFILE may be
    # relative (and for a mutant outside test/host it comes out as a pile of ../..), so
    # compare the absolute form against both.
    dutfile = os.path.relpath(target.path, HOST_DIR)
    abs_dut = str(target.path.resolve())
    env = dict(os.environ)
    # The venv's cocotb-config must win, or make picks up a second cocotb on this machine.
    env["PATH"] = f"{ROOT / '.venv' / 'bin'}{os.pathsep}{env.get('PATH', '')}"
    RAW.mkdir(exist_ok=True)
    raw = RAW / f"{target.mut.id}.log"

    # `make clean` first, and this is not politeness.  cocotb's Makefile.sim rebuilds
    # sim_build/sim.vvp only when a source is *newer* than it, so a second DUT that was
    # written before the first one was built looks up to date: the second mutant would be
    # run against the first mutant's binary and both would report the first one's failures.
    # The mutation files are all written by generate.py in one go, so this happens every
    # time.  See test/host/mutants/README.md "the stale build that made the first campaign
    # meaningless".
    cleaner = subprocess.run(["make", "clean"], cwd=HOST_DIR, env=env, text=True,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    cmd = ["make", "DUTFILE=" + dutfile, "HOST_DUT=" + dutfile]
    if keep_raw:
        raw.write_text(f"$ cd test/host && make clean\n{cleaner.stdout}"
                       f"$ cd test/host && make {' '.join(cmd[1:])}\n")
    proc = subprocess.run(cmd, cwd=HOST_DIR, env=env, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    res.exit_status = proc.returncode
    if keep_raw:
        with raw.open("a") as fh:
            fh.write(proc.stdout)
            fh.write(f"\nexit status: {proc.returncode}\n")

    for line in proc.stdout.splitlines():
        if "iverilog" in line and "-o sim_build" in line:
            res.compiled = line.strip()
        if "host-test: compiled" in line:
            res.bench_line = line.strip()

    # Evidence 1: this run's own build log has to name the mutant.
    if not res.compiled:
        res.error = (f"make did not run iverilog at all (make clean exit "
                     f"{cleaner.returncode}); the simulator would have reused a build")
        return res
    if abs_dut not in res.compiled:
        res.error = f"the iverilog line does not mention {abs_dut}: {res.compiled}"
        return res
    # Evidence 2: the image the tests ran against has to name the mutant and nothing else.
    vvp = HOST_DIR / "sim_build" / "sim.vvp"
    if not vvp.exists():
        res.error = (f"{vvp} does not exist: make did not compile anything "
                     f"(iverilog failed? see {raw})")
        return res
    res.vvp_duts = dut_paths_in(vvp)
    want_abs = abs_dut
    if res.vvp_duts != [want_abs]:
        res.error = (f"sim_build/sim.vvp was built from {res.vvp_duts}, not from "
                     f"[{want_abs}]")
        return res

    xml = HOST_DIR / "results.xml"
    if not xml.exists():
        res.error = f"{xml} does not exist; the bench did not run"
        return res
    if keep_raw:
        (RAW / f"{target.mut.id}.xml").write_text(xml.read_text())
    root = ET.parse(xml).getroot()
    for case in root.iter("testcase"):
        res.tests += 1
        kids = list(case)
        fail = [k for k in kids if k.tag in ("failure", "error")]
        skip = [k for k in kids if k.tag == "skipped"]
        if fail:
            res.failed += 1
            res.failed_tests.append(case.get("name", "?"))
            if not res.first_failure:
                res.first_failure = (fail[0].get("message") or fail[0].text or "").strip()
        elif skip:
            res.skipped += 1
        else:
            res.passed += 1
    return res


def verdict(res: Result) -> str:
    """KILLED / SURVIVED, and whether that is what this target was supposed to do."""
    if res.error:
        return f"ERROR ({res.error})"
    killed = res.killed
    want = res.target.expect == "kill"
    name = "KILLED" if killed else "SURVIVED"
    return name if killed == want else f"{name} -- UNEXPECTED (wanted {'KILL' if want else 'SURVIVE'})"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", default="",
                    help="comma-separated mutation ids, or id prefixes (M4, M4_partial)")
    ap.add_argument("--dir", type=Path, default=None,
                    help="root holding <id>/%s (default: the committed copies)" % DUT_NAME)
    ap.add_argument("--negative-control", action="store_true",
                    help="also run the no-op mutation, which must SURVIVE")
    ap.add_argument("--extra", action="append", default=[],
                    help="also run <dir>/<id>/%s, for a mutant that is not in the "
                         "manifest (selftest.py); repeatable" % DUT_NAME)
    ap.add_argument("--csv", type=Path, default=None, help="write a row per mutant here")
    args = ap.parse_args(argv)

    print(f"# source      : {ROOT / 'src' / DUT_NAME}")
    print(f"# mutant root : {args.dir or HERE}")
    print(f"# host tests  : {HOST_DIR / 'test_host_spi.py'}")
    bad = 0
    rows = []
    for t in targets(args):
        res = run_one(t)
        sha = getattr(res, "target_sha", "")
        print()
        print(f"### {t.mut.id}: {t.title}")
        print(f"    file       : {res.target.path}"
              + (f"  sha256 {sha[:16]}" if sha else ""))
        print(f"    expected   : {t.expect.upper()} (survives == a hole in the bench)"
              if t.expect == "kill" else
              f"    expected   : SURVIVE -- negative control, the copy is byte-identical"
              f" to src/{DUT_NAME}")
        print(f"    compiled   : {res.compiled or '(no iverilog line: make did not compile)'}")
        print(f"    sim.vvp    : {res.vvp_duts}")
        if res.bench_line:
            print(f"    bench line : {res.bench_line}")
        if res.error:
            print(f"    ERROR      : {res.error}")
        print(f"    make exit  : {res.exit_status}")
        print(f"    tests      : {res.tests} in results.xml, {res.passed} pass, "
              f"{res.failed} fail, {res.skipped} skip")
        if res.failed_tests:
            print(f"    killed by  : {', '.join(res.failed_tests)}")
            msg = res.first_failure
            if msg:
                lines = [l for l in msg.splitlines() if l.strip()][:6]
                for l in lines:
                    print(f"      | {l}")
        print(f"    verdict    : {verdict(res)}")
        ok = (res.killed == (t.expect == "kill")) and not res.error
        bad += not ok
        rows.append(dict(
            id=t.mut.id, title=t.title, expect=t.expect, sha256=sha,
            tests=res.tests, passed=res.passed, failed=res.failed,
            killed=int(res.killed), ok=int(ok),
            compiled=os.path.relpath(res.target.path, HOST_DIR),
            vvp_duts=";".join(res.vvp_duts),
            failing_tests=";".join(res.failed_tests), error=res.error,
            verdict=verdict(res)))

    print()
    print(f"{'mutation':36s} {'tests':>7s} {'killed':>7s}  verdict")
    for r in rows:
        print(f"{r['id']:36s} {r['passed']:3d}/{r['tests']:<3d} "
              f"{'yes' if r['killed'] else 'NO':>7s}  "
              f"{'ok' if r['ok'] else 'UNEXPECTED'} (expected {r['expect']})")
    survivors = [r["id"] for r in rows if r["expect"] == "kill" and not r["killed"]]
    errored = [r["id"] for r in rows if r["error"]]
    print()
    if bad:
        if survivors:
            print(f"RESULT: FAIL -- {len(survivors)} mutation(s) survived the bench: "
                  f"{', '.join(survivors)}")
        if errored:
            print(f"RESULT: FAIL -- {len(errored)} run(s) could not be measured at all: "
                  f"{', '.join(errored)} (an error is not a kill)")
        for r in rows:
            if r["error"]:
                print(f"  {r['id']}: {r['error']}")
    else:
        kills = sum(1 for r in rows if r["expect"] == "kill" and r["killed"])
        print(f"RESULT: PASS -- {kills}/{sum(1 for r in rows if r['expect'] == 'kill')} "
              f"mutations killed, "
              f"{sum(1 for r in rows if r['expect'] == 'survive' and not r['killed'])}"
              f"/{sum(1 for r in rows if r['expect'] == 'survive')} negative control(s) "
              f"survived")
    print(f"full per-mutant simulator output: {RAW}/<id>.log")

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"csv: {args.csv}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
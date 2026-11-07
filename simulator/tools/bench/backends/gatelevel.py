"""Backend `gl` (level "gl", long): the cocotb tests of test/test.py on the hardened gate-level netlist.

It is `make test-gl GL_CFG=<run>` (Makefile): copy runs/<GL_CFG>/proj/runs/wokwi/final/nl/*.nl.v to
test/gate_level_netlist.v, build with the PDK cell models (icarus, -DFUNCTIONAL) and run test/test.py.
GL_CFG is the run under the repo's runs/ (`bench-<config_hash>/flow` from the flow backend, or `pwr-mvp`
for the MVP research run; same lookup as the power backend), given relative to this checkout's runs/.  Side effects stay in gitignored paths (test/gate_level_netlist.v,
test/sim_build/gl, test/results.xml); runs are serialised with a lock file because those are shared.
"""
from __future__ import annotations

import fcntl
import hashlib
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from .. import knobs
from ..metrics import METRICS
from ..types import Record, RunSpec, Workload
from .physical import CHIP, _git
import os

from .power import FLOW_HINT, find_source, input_hash_for, regmap_refusal, runs_root

ROOT = Path(__file__).resolve().parents[3]
TIMEOUT_S = 3000


def parse_results(xml_text: str) -> dict:
    """Pure: cocotb/JUnit results.xml -> {"passed", "failed"} (failure, error and skipped-by-error count as failed)."""
    root = ET.fromstring(xml_text)
    passed = failed = 0
    for tc in root.iter("testcase"):
        if tc.find("failure") is not None or tc.find("error") is not None:
            failed += 1
        elif tc.find("skipped") is None:
            passed += 1
    return {"passed": passed, "failed": failed}


class GateLevel:
    name = "gl"
    level = "gl"

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        if workload.id != CHIP.id or workload.params:
            return "gl runs the 'chip' workload only (the whole test/test.py suite; no params)"
        reason = regmap_refusal(knobs.normalize(config), "test/test.py")
        if reason:
            return reason
        if find_source(knobs.normalize(config)) is None:
            return FLOW_HINT
        return None

    def knob_support(self) -> dict:
        from .physical import _support_table
        t = _support_table(True)
        for k in ("n_engines", "imem_words", "fifo_depth", "engine_route"):
            t[k] = "default_only: test/test.py speaks the MVP register map"
        return t

    def workloads(self) -> list[Workload]:
        return [CHIP]

    def is_long(self, spec: RunSpec) -> bool:
        return True

    def input_hash(self, spec: RunSpec) -> str:
        return input_hash_for(spec.config)

    def code_hash(self) -> str:
        h = hashlib.sha256()
        for g in ("test/test.py", "test/tb.v", "test/Makefile", "Makefile", "tools/bench/backends/gatelevel.py"):
            for p in sorted(ROOT.glob(g)):
                h.update(str(p.relative_to(ROOT)).encode() + b"\0" + p.read_bytes() + b"\0")
        return h.hexdigest()[:16]

    def run(self, spec: RunSpec) -> list[Record]:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        cfg = knobs.normalize(spec.config)
        w, chash = spec.workload, knobs.config_hash(cfg)
        src = find_source(cfg)
        gl_cfg = os.path.relpath(runs_root() / src.name, ROOT / "runs")      # Makefile: runs/$(GL_CFG)/proj/...
        started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        results = ROOT / "test" / "results.xml"
        lock = ROOT / "runs" / ".test-gl.lock"
        lock.parent.mkdir(exist_ok=True)
        with open(lock, "w") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            results.unlink(missing_ok=True)
            t0 = time.perf_counter()
            sram = ["SRAM_IMEM=1"] if cfg["imem_tech"] == "sram2p" else []     # test/Makefile adds the PDK macro model
            p = subprocess.run(["make", "test-gl", f"GL_CFG={gl_cfg}", *sram], cwd=ROOT, capture_output=True, text=True,
                               timeout=TIMEOUT_S)
            secs = round(time.perf_counter() - t0, 1)
            if not results.exists():
                raise RuntimeError(f"make test-gl produced no test/results.xml (rc={p.returncode}): "
                                   + (p.stdout + p.stderr)[-500:])
            counts = parse_results(results.read_text())
        prov = {"command": " ".join(["make", "test-gl", f"GL_CFG={gl_cfg}", *sram]), "commit": _git("rev-parse", "HEAD"),
                "dirty": bool(_git("status", "--porcelain", "-uno")), "started": started, "duration_s": secs,
                "run_dir": src.rundir, "make_rc": p.returncode}
        key = spec.key(self.code_hash() + ":" + self.input_hash(spec))
        note = f"netlist {src.rundir}/final/nl" + (" (research run pwr-mvp = mvp-slow, magic DRC off)" if src.kind == "research-mvp" else "")

        def rec(metric, value, status="ok"):
            return Record(workload=w.id, metric=metric, value=value, unit=METRICS[metric][0], level=self.level,
                          backend=self.name, method="measured", status=status, config=cfg, config_hash=chash,
                          workload_params=dict(w.params), note=note, provenance=dict(prov), run_key=key)
        return [rec("gl_tests_passed", counts["passed"]),
                rec("gl_tests_failed", counts["failed"], "fail" if counts["failed"] else "ok"),
                rec("gl_sim_seconds", secs)]

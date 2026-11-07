"""Deterministic fake backends for exercising the runner, CLI and cross-checks without any simulator.

`fake` (level iss) and `fake2` (level rtl) compute the same numbers from the config and the workload,
so a cross-check between them matches (tests build disagreeing records by hand instead, because a
differing workload param would change the join key). Behaviour is chosen by workload params:

    delay   seconds to sleep before answering (parallelism and timeout tests)
    mode    ok | fail (a record with status fail) | na (status n/a) | crash (hard exit 3)
            | raise (uncaught exception) | hang (sleeps 600 s) | noise (prints on stdout first)
            | childnoise (a child process prints on fd 1) | badmetric (emits a name outside the vocabulary)
    pidfile hang mode: path to write a grandchild pid to
    long    true => runner treats the spec as long (needs --allow-long)
    refuse  non-empty string => supports() refuses with that reason
    band    true => max_bit_rate carries a +-2 % band
"""
from __future__ import annotations

import hashlib
import os
import sys
import time
from pathlib import Path
from typing import Optional

from .. import knobs
from ..types import Record, RunSpec, Workload, sha

#: Knobs the fakes cannot vary: they refuse any non-default value (shows up in `bench knobs`).
DEFAULT_ONLY = ("lane", "imem_tech", "isa_xor", "isa_cond", "isa_irq")


class Fake:
    name = "fake"
    level = "iss"
    method = "measured"

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        if workload.params.get("refuse"):
            return str(workload.params["refuse"])
        d = knobs.defaults()
        for k in DEFAULT_ONLY:
            if config.get(k) != d[k]:
                return f"{self.name} runs only the default {k} ({d[k]!r}), not {config.get(k)!r}"
        return None

    def knob_support(self) -> dict:
        return {k.id: ("default_only" if k.id in DEFAULT_ONLY else "full") for k in knobs.KNOBS}

    def workloads(self) -> list[Workload]:
        return [
            Workload("count", "protocol", {"speed": 115200}, "deterministic numbers"),
            Workload("slow", "protocol", {"speed": 115200, "delay": 0.5}, "sleeps 0.5 s"),
            Workload("failing", "protocol", {"speed": 115200, "mode": "fail"}, "emits a status=fail record"),
            Workload("na", "protocol", {"speed": 115200, "mode": "na"}, "emits a status=n/a record"),
            Workload("crashing", "protocol", {"mode": "crash"}, "worker dies with exit 3"),
            Workload("raising", "protocol", {"mode": "raise"}, "backend raises"),
            Workload("hanging", "protocol", {"mode": "hang"}, "never answers (timeout test)"),
            Workload("longrun", "protocol", {"speed": 115200, "long": True}, "marked long"),
        ]

    def is_long(self, spec: RunSpec) -> bool:
        return bool(spec.workload.params.get("long"))

    def _numbers(self, spec: RunSpec) -> tuple[int, float]:
        """Cycles per bit and max bit rate, derived from config + speed only (not from the level)."""
        h = int(sha({"c": spec.config, "s": spec.workload.params.get("speed")}, 8), 16)
        cycles = 20 + h % 400
        rate = spec.config["f_core_mhz"] * 1e6 / cycles
        return cycles, rate

    def run(self, spec: RunSpec) -> list[Record]:
        p = spec.workload.params
        mode = p.get("mode", "ok")
        if p.get("delay"):
            time.sleep(float(p["delay"]))
        if mode == "noise":
            print("stray stdout from a chatty backend")
        if mode == "crash":
            print("fake backend: simulated crash", file=sys.stderr)
            os._exit(3)
        if mode == "raise":
            raise RuntimeError("fake backend: simulated exception")
        if mode == "childnoise":  # a child process writing to the inherited stdout
            os.system("echo stray-from-child")
        if mode == "hang":
            if p.get("pidfile"):  # leave a grandchild behind, to test that a timeout kills the group
                import subprocess
                Path(p["pidfile"]).write_text(str(subprocess.Popen(["sleep", "600"]).pid))
            time.sleep(600)
        cycles, rate = self._numbers(spec)
        w = spec.workload.id

        def rec(metric, value, unit, **kw):
            return Record(workload=w, metric=metric, value=value, unit=unit, level=self.level,
                          backend=self.name, method=self.method, **kw)

        if mode == "badmetric":
            return [rec("not_a_real_metric", 1, "")]
        if mode == "na":
            return [rec("max_bit_rate", None, "Hz", status="n/a", note="fake: nothing observed"),
                    rec("sm_cycles_per_bit", cycles, "cycles")]
        status = "fail" if mode == "fail" else "ok"
        return [rec("sm_cycles_per_bit", cycles, "cycles", status=status),
                rec("max_bit_rate", round(rate, 3), "Hz", status=status,
                    band=[round(rate * 0.98, 3), round(rate * 1.02, 3)] if p.get("band") else None),
                rec("verdict", "served" if rate >= 1e5 else "impossible", "")]

    def code_hash(self) -> str:
        return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:16]


class Fake2(Fake):
    name = "fake2"
    level = "rtl"

"""`iss_p`: the parametric ISS (tools/bench/sim/pio_iss_p.py) as a bench backend.

Like `iss`, it runs the perf-b protocol workloads (so FIFO-depth and IMEM sweeps work for single protocols),
and it also runs the bridge and mix workloads of tools/bench/bridges.py.  It honours the structural knobs
n_engines, imem_words, fifo_depth, engine_route and t_in: they go to the chip class through the env var
BENCH_CONFIG, set before the fw target is installed (the install is process-global, so one worker process
serves one set of structural knobs; a second set raises, as for `iss`).  Everything else must be the MVP.

Workloads offered:
    <protocol> / spi_host_port      as `iss` (params speed / ticks_per_bit)
    bridge.<a>-><b>                 bridges.BRIDGES (need engine_route=true, n_engines >= 2)
    mix.<a>+<b>[+<c>]               bridges.MIXES   (need n_engines >= number of protocols)
"""
from __future__ import annotations

import hashlib
import json
import os
import time
from datetime import datetime, timezone
from typing import Optional

from .. import knobs, workloads as wl
from ..types import Record, RunSpec, Workload
from . import sim
from .sim import ROOT, SimBackend, _files, _git

#: the knobs the chip class takes (Params.from_config)
STRUCTURAL = ("n_engines", "imem_words", "fifo_depth", "engine_route", "t_in")

_INSTALLED_P: Optional[str] = None       # the BENCH_CONFIG JSON this process was installed with

_BRIDGE_PARAMS = {"in_rate", "out_rate", "burst", "gap", "search"}


class IssP(SimBackend):
    def __init__(self) -> None:
        super().__init__("iss_p")
        self.target = "iss_p"
        self._cfg: dict = {}

    # ---- contract -------------------------------------------------------------------------------

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        from ..sim import pio_iss_p
        from .. import bridges
        cfg = knobs.normalize(config)
        for kid, default in knobs.defaults().items():
            if kid not in STRUCTURAL and cfg[kid] != default:
                return f"iss_p models the structural knobs only; knob {kid}={cfg[kid]!r} (MVP {default!r}) is not supported"
        try:
            pio_iss_p.Params.from_config(cfg)
        except ValueError as e:
            return f"iss_p: {e}"
        kind, name = bridges.split_id(workload.id)
        if kind == "bridge" or kind == "mix":
            known = bridges.BRIDGES if kind == "bridge" else bridges.MIXES
            if name not in known:
                return f"iss_p has no workload {workload.id!r}"
            bad = set(workload.params) - set(bridges.DEFAULTS[name])
            if bad:
                return f"workload {workload.id} takes params {sorted(bridges.DEFAULTS[name])}, not {sorted(bad)}"
            if kind == "bridge" and (not cfg["engine_route"] or cfg["n_engines"] < 2):
                return "bridges need engine_route=true and n_engines >= 2"
            if kind == "mix":
                need = 2 if name == "uart_tx+uart_rx" else 3
                if cfg["n_engines"] < need:
                    return f"{workload.id} needs n_engines >= {need}"
            return None
        bad = wl.engines_refusal(workload.id, cfg["n_engines"])
        if bad:
            return bad
        return super().supports(knobs.defaults() | {k: cfg[k] for k in ("f_core_mhz",)}, workload)

    def knob_support(self) -> dict:
        out = {}
        for k in knobs.KNOBS:
            if k.id in STRUCTURAL:
                out[k.id] = "full" if k.id != "t_in" else "partial: a pipeline depth in the ISS; protocol firmware assumes T_IN=3"
            else:
                out[k.id] = "default_only"
        out["n_engines"] = "partial: protocol rigs use engines 0-1; mixes need 2-3"
        out["imem_words"] = "partial: the shipped firmware is up to 29 words, so 16 fails for most protocols"
        out["engine_route"] = "full"
        return out

    def workloads(self) -> list[Workload]:
        from .. import bridges
        out = [w for w in wl.all_workloads()]
        for n in bridges.BRIDGES:
            out.append(Workload(f"bridge.{n}", "bridge", dict(bridges.DEFAULTS[n]), f"bridge {n}"))
        for n in bridges.MIXES:
            out.append(Workload(f"mix.{n}", "mix", dict(bridges.DEFAULTS[n]), f"mix {n}"))
        return out

    def is_long(self, spec: RunSpec) -> bool:
        return False

    def code_hash(self, config: Optional[dict] = None) -> str:
        h = hashlib.sha256()
        globs = self._COMMON + ["tools/bench/sim/*.py", "tools/bench/bridges.py", "tools/bench/bridges/*.pio",
                                "tools/bench/backends/sim_p.py", "asm/*.py"]
        for p in _files(globs):
            h.update(str(p.relative_to(ROOT)).encode() + b"\0" + p.read_bytes() + b"\0")
        return h.hexdigest()[:16]

    # ---- running --------------------------------------------------------------------------------

    def _install(self, sram: bool, cfg: Optional[dict] = None) -> None:
        global _INSTALLED_P
        want = json.dumps({k: self._cfg[k] for k in STRUCTURAL}, sort_keys=True)
        if _INSTALLED_P is not None:
            if _INSTALLED_P != want:
                raise RuntimeError(f"this process already installed iss_p with {_INSTALLED_P}; target install is "
                                   f"process-global, run {want} in another process")
            return
        os.environ["BENCH_CONFIG"] = want
        os.environ["FW_TARGET"] = "iss_p"
        from verif.perf import measure
        measure.install_target("iss_p")
        _INSTALLED_P = want

    def _command(self, w: Workload, sram: bool, cfg: Optional[dict] = None) -> str:
        from ..runner import repro_command
        return repro_command(RunSpec(self._cfg, w, self.name))

    def run(self, spec: RunSpec) -> list[Record]:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        self._cfg = knobs.normalize(spec.config)
        from .. import bridges
        kind, name = bridges.split_id(spec.workload.id)
        if kind not in ("bridge", "mix"):
            return self._guard(super().run(spec), self._cfg)
        w, cfg = spec.workload, self._cfg
        started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        t0 = time.perf_counter()
        self._install(False)
        fn = bridges.run_bridge if kind == "bridge" else bridges.run_mix
        res = fn(name, cfg, dict(w.params))
        prov = {"command": self._command(w, False), "commit": _git("rev-parse", "HEAD"),
                "dirty": bool(_git("status", "--porcelain")), "started": started,
                "duration_s": round(time.perf_counter() - t0, 2)}
        return self._records(res, w, cfg, prov, spec.key(self.code_hash()))

    #: protocol-rig metrics that are wrong, not just different, off the MVP structure (audit in notes/J.md)
    def _guard(self, recs: list[Record], cfg: dict) -> list[Record]:
        why = {}
        if cfg["n_engines"] > 2 or cfg["imem_words"] > 32:
            for m in ("host_port_write_cycles", "host_port_read_cycles", "write", "read", "read_rxf0_pop",
                      "txf0_burst_4", "rxf0_burst_4", "cmd_only"):
                why[m] = ("host-port cost rig (verif/perf/hostport.py) frames the 1-byte MVP command; "
                          "the wide map needs 2 bytes")
        for r in recs:
            if r.metric in why:
                r.status, r.value, r.band = "n/a", None, None
                r.note = why[r.metric] + "; " + r.note
        return recs

    def _records(self, res: dict, w: Workload, cfg: dict, prov: dict, key: str) -> list[Record]:
        from ..metrics import METRICS
        chash = knobs.config_hash(cfg)
        passed, pnote = res.pop("_pass")
        recs = []

        def rec(metric, value, note, unit=None, status="ok", method="measured"):
            if value is None:
                status = "n/a"
            recs.append(Record(workload=w.id, metric=metric, value=value, unit=unit or METRICS[metric][0],
                               level=self.level, backend=self.name, method=method, status=status, config=cfg,
                               config_hash=chash, workload_params=dict(w.params), note=note,
                               provenance=dict(prov), run_key=key))
        rec("sweep_point", "pass" if passed else "fail", (pnote or "bridge verdict: no loss, no spurious bytes")
            if w.kind == "bridge" else (pnote or "every protocol passed"), unit="pass/fail",
            status="ok" if passed else "fail")
        for metric, (value, note) in res.items():
            if metric in ("all_pass",) or metric.startswith("pass_"):
                rec(metric, bool(value), note)
            elif metric == "host_port_utilization":
                rec(metric, value, note, method="derived")
            else:
                rec(metric, value, note)
        return recs

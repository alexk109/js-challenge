"""Simulation backends at the MVP: `iss`, `iss_b`, `rtl` (one class, parameterised by fw target).

A run installs the fw target exactly as verif/perf/measure.py does (`install_target`, which also checks the
install took), runs the perf-b rig for the workload and speed, and turns the Trial rows into Records with
perf.csv's metric names.  Target installation is process-global, so one process serves one target (and one
imem_tech); asking for another raises.  The runner's worker subprocess is the normal caller.
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from .. import knobs, workloads as wl
from ..types import Record, RunSpec, Workload

ROOT = Path(__file__).resolve().parents[3]

#: The designated RTL smoke: the cheapest rig (spi, ~1.3k payload cycles) at its default speed.
RTL_SMOKE = "spi"

_INSTALLED: Optional[tuple] = None        # (target, sram, rtl params) this process was given

#: knobs the rtl backend builds (fw/Makefile via tools/bench/rtlparams.py; task H): the structural ones
RTL_KNOBS = ("imem_tech", "t_in", "n_engines", "imem_words", "fifo_depth", "engine_route")


def SRAM_MODEL_FILES() -> list[Path]:
    """The two PDK model files fw/Makefile adds to the rtl build when SRAM_IMEM=1 (PDK_ROOT as the Makefile reads it)."""
    pdk = Path(os.environ.get("PDK_ROOT") or Path.home() / "pdk")
    d = pdk / "ihp-sg13cmos5l" / "libs.ref" / "sg13cmos5l_sram" / "verilog"
    return [d / "RM_IHPSG13_2P_64x22_c2_bm_bist.v", d / "RM_IHPSG13_2P_core_behavioral_bm_bist_ideal.v"]


def _git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _files(globs: list[str]) -> list[Path]:
    out: set[Path] = set()
    for g in globs:
        for p in ROOT.glob(g):
            if p.is_file() and "__pycache__" not in p.parts and "sim_build" not in p.parts:
                out.add(p)
    return sorted(out)


class SimBackend:
    #: files each target's results depend on (beyond _COMMON)
    _COMMON = ["iss/**/*.py", "fw/**/*.py", "fw/Makefile", "verif/perf/*.py", "verif/vip/**/*.py"]
    _EXTRA = {"iss": [], "iss_b": ["verif/iss_b/**/*.py"], "rtl": ["src/*.sv"]}

    def __init__(self, target: str = "iss") -> None:
        self.target = target
        self.name = target
        self.level = "rtl" if target == "rtl" else "iss"

    # ---- contract -------------------------------------------------------------------------------

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        cfg = knobs.normalize(config)
        for kid, default in knobs.defaults().items():
            if cfg[kid] == default:
                continue
            if self.target == "rtl" and kid in RTL_KNOBS:
                continue                                    # built into the Verilator model (sram2p: SRAM_IMEM=1)
            return f"{self.name} runs the MVP only; knob {kid}={cfg[kid]!r} (MVP {default!r}) is not supported"
        if self.target == "rtl":
            from .. import rtlparams
            bad = rtlparams.invalid(cfg)
            if bad:
                return f"{self.name}: {bad}"
        if workload.id == wl.HOST_PORT:
            if set(workload.params) != {wl.HOST_PORT_PARAM}:
                return f"{wl.HOST_PORT} takes params {{{wl.HOST_PORT_PARAM}}}"
            t = workload.params[wl.HOST_PORT_PARAM]
            if isinstance(t, bool) or not isinstance(t, (int, float)) or t != int(t) or not 1 <= t <= wl.HOST_PORT_MAX_TICKS:
                return f"{wl.HOST_PORT_PARAM}={t!r} is outside the host port's range (an integer 1..{wl.HOST_PORT_MAX_TICKS})"
            return None
        if workload.id not in wl.protocol_names():
            return f"{self.name} has no workload {workload.id!r}"
        if set(workload.params) != {"speed"}:
            return f"workload {workload.id} takes params {{speed}}"
        bad = wl.speed_refusal(workload.id, workload.params["speed"])
        if bad:
            return bad
        bad = wl.engines_refusal(workload.id, cfg["n_engines"])
        if bad:
            return bad
        if self.target == "rtl" and not wl.rtl_targetable(workload.id):
            return f"{workload.id} is not RTL-targetable (verif/perf/rigs.py; payload too long for Verilator)"
        return None

    def knob_support(self) -> dict:
        return {k.id: ("full" if (self.target == "rtl" and k.id in RTL_KNOBS) else "default_only")
                for k in knobs.KNOBS}

    def workloads(self) -> list[Workload]:
        mvp = knobs.normalize({})
        return [w for w in wl.all_workloads() if self.supports(mvp, w) is None]

    def is_long(self, spec: RunSpec) -> bool:
        """Contract addition: True if the runner must gate this spec behind --allow-long."""
        if self.target != "rtl":
            return False
        w = spec.workload
        return not (w.id == RTL_SMOKE and w.params.get("speed") == wl.default_speed(RTL_SMOKE))

    def code_hash(self, config: Optional[dict] = None) -> str:
        """Hash of what a result depends on. For rtl, the PDK SRAM model files count when imem_tech=sram2p
        (or when no config is given: then the answer must cover both builds)."""
        h = hashlib.sha256()
        for p in _files(self._COMMON + self._EXTRA[self.target]):
            h.update(str(p.relative_to(ROOT)).encode() + b"\0" + p.read_bytes() + b"\0")
        sram = config is None or knobs.normalize(config)["imem_tech"] == "sram2p"
        if self.target == "rtl" and sram:
            for p in SRAM_MODEL_FILES():
                h.update(p.name.encode() + b"\0" + (p.read_bytes() if p.is_file() else b"missing") + b"\0")
        return h.hexdigest()[:16]

    # ---- running --------------------------------------------------------------------------------

    def _install(self, sram: bool, cfg: dict) -> None:
        global _INSTALLED
        from .. import rtlparams
        if bool(os.environ.get("FW_RTL_GL")) != (self.name == "gl_rig"):
            raise RuntimeError(f"{self.name}: this process is (or was) set up for the "
                               f"{'gate-level' if os.environ.get('FW_RTL_GL') else 'RTL'} harness; run it in its own process")
        rtl_params = rtlparams.sim_build(cfg)["tag"] if self.target == "rtl" else ""
        want = (self.target, sram, rtl_params)
        if _INSTALLED is not None:
            if _INSTALLED != want:
                raise RuntimeError(f"this process already installed {_INSTALLED}; target install is "
                                   f"process-global, run {want} in another process")
            return
        os.environ["FW_TARGET"] = self.target
        if self.target == "rtl":
            os.environ["SRAM_IMEM"] = "1" if sram else "0"      # read by fw/Makefile
            # the structural knobs: the harness and fw/Makefile read env BENCH_CONFIG (a separate sim_build per set)
            os.environ["BENCH_CONFIG"] = json.dumps(cfg, sort_keys=True)
            os.environ.setdefault("PYTHON", sys.executable)
        from verif.perf import measure
        measure.install_target(self.target)
        _INSTALLED = want

    def _command(self, w: Workload, sram: bool, cfg: Optional[dict] = None) -> str:
        env = f"FW_TARGET={self.target} " + ("SRAM_IMEM=1 " if sram else "")
        if self.target == "rtl" and cfg is not None and knobs.diff(cfg):
            env += "BENCH_CONFIG='" + json.dumps(cfg, sort_keys=True) + "' "
        if w.id == wl.HOST_PORT:
            args = f"--protocols spi --speeds spi={wl.default_speed('spi'):g}"
        else:
            args = f"--protocols {w.id} --speeds {w.id}={w.params['speed']:g}"
        return (f"{env}{sys.executable} -m verif.perf.measure --target {self.target} {args} "
                f"--csv /tmp/bench-repro.csv")

    def run(self, spec: RunSpec) -> list[Record]:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        cfg = knobs.normalize(spec.config)
        sram = cfg["imem_tech"] == "sram2p"
        w = spec.workload
        started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        t0 = time.perf_counter()
        self._install(sram, cfg)
        from verif.perf import measure, rigs

        rows: list[dict] = []
        ok, why = True, ""
        if w.id == wl.HOST_PORT:
            rows, _ = measure.host_port_rows(self.target, int(w.params[wl.HOST_PORT_PARAM]))
        else:
            proto = rigs.ladder_for(w.id)
            try:
                trial = proto.trial(float(w.params["speed"]))
            except Exception as e:      # a setting the rig cannot run is a failed point, not a crash
                dur = round(time.perf_counter() - t0, 2)
                prov = {"command": self._command(w, sram, cfg), "commit": _git("rev-parse", "HEAD"),
                        "dirty": bool(_git("status", "--porcelain", "-uno")), "started": started, "duration_s": dur}
                return [Record(workload=w.id, metric="sweep_point", value="fail", unit="", level=self.level,
                               backend=self.name, method="measured", status="fail", config=cfg,
                               config_hash=knobs.config_hash(cfg), workload_params=dict(w.params),
                               note=f"rig raised {type(e).__name__}: {e}", provenance=prov,
                               run_key=spec.key(self.code_hash(cfg)))]
            ok, why = trial.ok, trial.why
            rows.append(measure._sweep_row(proto, trial, self.target))
            trial.extra.setdefault("responds", proto.responds)
            rows.extend(trial.rows(target=self.target))
            for key, value in trial.extra.items():
                if isinstance(value, (int, float, str)) and str(value):
                    rows.append(dict(metric=f"measured_{key}", unit="see source",
                                     value=value if isinstance(value, str) else f"{value:g}",
                                     source=f"also measured in this run ({w.id} at {proto.knob}={trial.value:g}"
                                            f" {proto.unit})" + (f"; {trial.extra_notes[key]}"
                                                                if key in trial.extra_notes else "")))
            # host-port costs are not protocol metrics: the spi_host_port workload carries them (once)
        dur = round(time.perf_counter() - t0, 2)
        prov = {"command": self._command(w, sram, cfg), "commit": _git("rev-parse", "HEAD"),
                "dirty": bool(_git("status", "--porcelain", "-uno")), "started": started, "duration_s": dur}
        chash, key = knobs.config_hash(cfg), spec.key(self.code_hash(cfg))
        return [self._record(r, w, cfg, chash, key, prov, ok, why) for r in rows]

    def _record(self, row: dict, w: Workload, cfg: dict, chash: str, key: str, prov: dict,
                trial_ok: bool, why: str) -> Record:
        metric, raw, src = row["metric"], str(row["value"]), row["source"]
        status, value, method = "ok", None, "measured"
        if metric == "sweep_point":
            status, value = ("ok" if raw == "pass" else "fail"), raw
        elif raw == "n/a":
            status = "n/a"
        else:
            try:
                value = float(raw)
            except ValueError:
                value = raw
            if not trial_ok:
                status = "fail"
        if "FALLBACK" in src:
            method = "derived"
        note = (f"checker refused this setting: {why}; " if not trial_ok and metric != "sweep_point" else "") + src
        return Record(workload=w.id, metric=metric, value=value, unit=row["unit"], level=self.level,
                      backend=self.name, method=method, status=status, config=cfg, config_hash=chash,
                      workload_params=dict(w.params), note=note, provenance=dict(prov), run_key=key)


class Iss(SimBackend):
    def __init__(self) -> None:
        super().__init__("iss")


class IssB(SimBackend):
    def __init__(self) -> None:
        super().__init__("iss_b")


class Rtl(SimBackend):
    def __init__(self) -> None:
        super().__init__("rtl")

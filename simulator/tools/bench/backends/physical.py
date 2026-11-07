"""Physical backends: `synth` (Yosys area of the chip top, minutes) and `flow` (full LibreLane, long).

Both build the chip for a bench config through `tools.bench.rtlparams.rtl_build` (the one knob -> RTL
mapping) and run only on the "chip" workload (kind "chip"; no params).

* synth  reuses phys/synth_probe.py (`yosys_script`, `parse_stat`, `cell_areas`, `SEQ`, PDK paths): the
  same Yosys + liberty + latch map as `make synth-probe`, on the MVP sources with parameter / define
  overrides applied to a copy under runs/bench-<config_hash>/synth/.
* flow   generates the TT-style LibreLane project the way phys/flow.py does for kind=tt (derived from
  phys/cfg/mvp-slow.json, the merged MVP config) under runs/bench-<config_hash>/, runs LibreLane, and
  parses <run>/final/metrics.json with the pure function `parse_final_metrics`.

EDA tools run only through tools/ll (an optional wrapper script).  The gitignored tt/ directory is looked up in the
checkout, then in the repo (read-only).
"""
from __future__ import annotations

import importlib.util
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from .. import knobs, rtlparams
from ..types import Record, RunSpec, Workload

ROOT = Path(__file__).resolve().parents[3]
LL = ROOT / "tools" / "ll"
CHIP = Workload("chip", "chip", {}, "whole chip (area, timing)")
TOP = "tt_um_protocol_emulator"

from ..metrics import METRICS      # the closed vocabulary; this backend's names are in the "task I" section

TT_CORNER = "nom_typ_1p20V_25C"
SS_CORNER = "nom_slow_1p08V_125C"


def _git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _tt_dir() -> Path:
    """tt-support-tools checkout (gitignored): this checkout's, else the repo's."""
    for c in (ROOT / "tt", Path(_git("rev-parse", "--git-common-dir")).resolve().parent / _git("rev-parse", "--show-prefix") / "tt"):
        if (c / "tt_tool.py").exists():
            return c
    raise FileNotFoundError("tt/ (tt-support-tools) not found in the checkout or the repo")


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_SYNTH_PROBE = None


def synth_probe():
    """phys/synth_probe.py as a module (phys/ is not a package)."""
    global _SYNTH_PROBE
    if _SYNTH_PROBE is None:
        _SYNTH_PROBE = _load("synth_probe", ROOT / "phys" / "synth_probe.py")
    return _SYNTH_PROBE


def _base_cfg() -> dict:
    return json.loads(rtlparams.BASE_CFG.read_text())


def _runs_dir(cfg: dict) -> Path:
    return ROOT / "runs" / f"bench-{knobs.config_hash(cfg)}"


def _tool_stamp() -> bytes:
    """Things outside the repo that change results: the liberty (path, size, mtime) and tools/ll."""
    sp = synth_probe()
    out = b""
    for f in (Path(sp.LIB), Path(sp.LATCH_MAP)):
        st = f.stat() if f.exists() else None
        out += f"{f}|{st.st_size if st else 0}|{int(st.st_mtime) if st else 0}\0".encode()
    return out


def _hash_files(globs: list[str]) -> str:
    import hashlib
    h = hashlib.sha256(_tool_stamp())
    globs = globs + ["tools/ll"]
    for g in globs:
        for p in sorted(ROOT.glob(g)):
            if p.is_file() and "__pycache__" not in p.parts:
                h.update(str(p.relative_to(ROOT)).encode() + b"\0" + p.read_bytes() + b"\0")
    return h.hexdigest()[:16]


def _refusal(name: str, config: dict, workload: Workload, flow: bool) -> Optional[str]:
    cfg = knobs.normalize(config)
    if workload.id != CHIP.id or workload.params:
        return f"{name} runs the 'chip' workload only (no params)"
    bad = rtlparams.unsupported(cfg)
    if bad:
        return f"{name}: knob(s) {', '.join(bad)} have no RTL mapping yet (tools/bench/rtlparams.py)"
    bad_combo = rtlparams.invalid(cfg)
    if bad_combo:
        return f"{name}: {bad_combo}"
    if not flow and cfg["f_core_mhz"] != knobs.defaults()["f_core_mhz"]:
        return "synth is period-independent; f_core_mhz != MVP is only meaningful for the flow"
    if flow and cfg["die"] != knobs.defaults()["die"]:
        return "flow: only the 6x4 die is configured (phys/cfg/mvp-slow.json)"
    return None


def _support_table(flow: bool) -> dict:
    sup = rtlparams.supported_knobs()
    out = {}
    for k in knobs.KNOBS:
        if k.id == "f_core_mhz":
            out[k.id] = "full" if flow else "default_only"
        elif k.id == "die":
            out[k.id] = "default_only"
        elif k.id in sup:
            out[k.id] = "full"
        else:
            out[k.id] = "default_only"
    return out


def _rec(w: Workload, metric: str, value, backend: str, level: str, cfg: dict, chash: str, key: str,
         prov: dict, method: str = "measured", status: str = "ok", note: str = "") -> Record:
    return Record(workload=w.id, metric=metric, value=value, unit=METRICS[metric][0], level=level,
                  backend=backend, method=method, status=status, config=cfg, config_hash=chash,
                  workload_params=dict(w.params), note=note, provenance=dict(prov), run_key=key)


def _command(backend: str, cfg: dict) -> str:
    sets = " ".join(f"--set {k}={json.dumps(v)}" for k, v in sorted(knobs.diff(cfg).items()))
    return (f"PYTHONPATH=. {sys.executable} -m tools.bench run {sets} --workload chip --backend {backend}"
            + (" --allow-long" if backend == "flow" else "")).replace("  ", " ")


# ---------------------------------------------------------------------------------------------- synth



def macro_area(lib_path: str) -> float:
    """Area of the (single) macro cell in a macro liberty file."""
    m = re.search(r'^\s*cell\s*\(\s*"?\w+"?\s*\)\s*\{.*?^\s*area\s*:\s*([\d.]+)',
                  Path(lib_path).read_text(), re.S | re.M)
    if not m:
        raise ValueError(f"no cell area in {lib_path}")
    return float(m.group(1))


def synth_script(sp, name: str, files: list[Path], defines: list[str]) -> str:
    """synth_probe's own Yosys script for the chip top, with -D defines added to read_slang."""
    probe = {"top": TOP, "files": [str(f) for f in files], "params": {}}
    script = sp.yosys_script(probe, name)
    if defines:
        script = script.replace("read_slang ", "read_slang " + " ".join(f"-D {d}" for d in defines) + " ", 1)
    return script


def parse_synth(sp, stat_path: Path, areas: dict, macro_um2: float = 0.0) -> dict:
    """Pure over a Yosys `stat -liberty` file + the liberty cell areas."""
    area, cells = sp.parse_stat(stat_path)
    seq = {c: k for c, k in cells.items() if sp.SEQ.match(c)}
    seq_area = sum(areas.get(c, 0.0) * k for c, k in seq.items())
    return {"area_um2": area + macro_um2, "stdcell_area_um2": area, "seq_area_um2": seq_area, "comb_area_um2": area - seq_area,
            "macro_area_um2": macro_um2, "cell_count": sum(cells.values()), "seq_cell_count": sum(seq.values())}


class Synth:
    name = "synth"
    level = "synth"

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        return _refusal(self.name, config, workload, flow=False)

    def knob_support(self) -> dict:
        return _support_table(False)

    def workloads(self) -> list[Workload]:
        return [CHIP]

    def is_long(self, spec: RunSpec) -> bool:
        return False

    def code_hash(self) -> str:
        return _hash_files(["src/*.sv", "src/*.v", "tools/bench/rtlparams.py", "tools/bench/backends/physical.py",
                            "phys/synth_probe.py", "phys/cfg/mvp-slow.json", "phys/cfg/mvp-sram.json"])

    def run(self, spec: RunSpec) -> list[Record]:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        cfg = knobs.normalize(spec.config)
        build = rtlparams.rtl_build(cfg)
        w, chash = spec.workload, knobs.config_hash(cfg)
        started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        t0 = time.perf_counter()
        sp = synth_probe()
        out = _runs_dir(cfg) / "synth"
        shutil.rmtree(out, ignore_errors=True)
        out.mkdir(parents=True)
        sp.OUT = out
        names = _base_cfg()["sources"] + build["extra_sources"] + [f for f in build["extra_src"] if f.endswith(".v")]
        files = rtlparams.materialize(build, names, out / "src")
        ys = out / "chip.ys"
        ys.write_text(synth_script(sp, "chip", files, build["defines"]))
        p = subprocess.run([str(LL), "yosys", "-q", "-l", str(out / "chip.log"), "-s", str(ys)],
                           capture_output=True, text=True)
        stat = out / "chip.stat"
        dur = round(time.perf_counter() - t0, 2)
        prov = {"command": _command(self.name, cfg), "commit": _git("rev-parse", "HEAD"),
                "dirty": bool(_git("status", "--porcelain")), "started": started, "duration_s": dur,
                "run_dir": str(out.relative_to(ROOT))}
        key = spec.key(self.code_hash())
        if p.returncode != 0 or not stat.exists():
            tail = (p.stderr or p.stdout or "")[-400:]
            raise RuntimeError(f"yosys failed (rc={p.returncode}); see {out / 'chip.log'}: {tail}")
        macro = sum(macro_area(m) for m in self._macro_libs(build))
        vals = parse_synth(sp, stat, sp.cell_areas(), macro)
        prov["duration_s"] = round(time.perf_counter() - t0, 2)
        recs = []
        for m, v in vals.items():
            note = ""
            if m == "area_um2" and macro:
                note = f"includes {macro:.0f} um2 of SRAM macros (liberty area)"
            recs.append(_rec(w, m, round(v, 2) if isinstance(v, float) else v, self.name, self.level, cfg,
                             chash, key, prov, note=note))
        return recs

    @staticmethod
    def _macro_libs(build: dict) -> list[str]:
        pdk = synth_probe().PDK
        return [s.replace("pdk_dir::", pdk + "/") for s in build["macros"]]


# ---------------------------------------------------------------------------------------------- flow

def fmax_mhz(period_ns: float, ws) -> Optional[float]:
    """Same formula as phys/flow.py: 1000 / (period - setup WNS)."""
    if ws is None:
        return None
    return round(1000.0 / (period_ns - float(ws)), 1)


def parse_final_metrics(m: dict, period_ns: float) -> dict:
    """Pure: LibreLane `final/metrics.json` (dict) -> bench numbers; a missing key is None."""
    def g(*names):
        for n in names:
            if m.get(n) is not None:
                return m[n]
        return None
    ws_t = g(f"timing__setup__ws__corner:{TT_CORNER}", "timing__setup__ws")
    ws_s = g(f"timing__setup__ws__corner:{SS_CORNER}")
    std = g("design__instance__area__stdcell")       # placed std cells incl. repair/CTS buffers and taps, no fill
    mac = g("design__instance__area__macros")
    util = g("design__instance__utilization__stdcell", "design__instance__utilization")
    return {
        "area_um2": None if std is None else std + (mac or 0.0),
        "stdcell_area_um2": std, "macro_area_um2": mac,
        "cell_count": g("design__instance__count__stdcell", "design__instance__count"),
        "seq_cell_count": g("design__instance__count__class:sequential_cell"),
        "utilization_pct": None if util is None else round(100.0 * util, 3),
        "wns_ns_typ": ws_t, "wns_ns_slow": ws_s,
        "fmax_mhz_typ": fmax_mhz(period_ns, ws_t), "fmax_mhz_slow": fmax_mhz(period_ns, ws_s),
        "drc_errors": g("route__drc_errors"),
        "lvs_errors": g("design__lvs_error__count"),
    }


def escape_macro_brackets(conns: list) -> list:
    """PDN_MACRO_CONNECTIONS instance names are regexes, and the generate-loop brackets of
    `g_eng[0]` are a character class there (the floorplan step dies on them; the instance carries
    them escaped inside the odb).  Match the bracket pair with its optional backslash: `.{1,2}0.{1,2}`
    (the fix research/power/mkcfg.py applied for pwr-sram).  Only the first word (the instance) is touched."""
    def fix(entry: str) -> str:
        return re.sub(r"^(\S+)", lambda m: re.sub(r"\[(\d+)\]", r".{1,2}\1.{1,2}", m.group(1)), entry)
    return [fix(c) for c in conns]


def flow_config(config: dict) -> dict:
    """The phys/flow.py-style cfg (kind=tt) for a bench config: mvp-slow.json + rtl_build()."""
    cfg = knobs.normalize(config)
    build = rtlparams.rtl_build(cfg)
    base = _base_cfg()
    ll = {k: v for k, v in base["librelane"].items() if not k.startswith("//")}
    ll.update(build["librelane"])
    ll["RUN_MAGIC_DRC"] = False
    ll["ERROR_ON_MAGIC_DRC"] = False
    if ll.get("PDN_MACRO_CONNECTIONS"):
        ll["PDN_MACRO_CONNECTIONS"] = escape_macro_brackets(ll["PDN_MACRO_CONNECTIONS"])
    if build["defines"]:
        ll["VERILOG_DEFINES"] = list(build["defines"])
    out = {k: v for k, v in base.items() if not k.startswith("//")}
    out.update(sources=base["sources"] + build["extra_sources"], clock_period=build["clock_period_ns"],
               tag=f"bench-{knobs.config_hash(cfg)}", librelane=ll)
    if build["extra_src"]:
        out["extra_src"] = list(build["extra_src"])
    return out


def prepare_flow(config: dict, rundir: Path) -> tuple[Path, str, dict]:
    """Write the TT project for the config (the recipe of phys/flow.py prepare_tt, with parameter
    overrides applied to the copied sources).  Returns (workdir, config file name, flow cfg)."""
    import yaml
    cfg = knobs.normalize(config)
    build = rtlparams.rtl_build(cfg)
    fc = flow_config(cfg)
    shutil.rmtree(rundir, ignore_errors=True)       # rundir is runs/bench-<hash>/flow; synth/ is a sibling
    proj = rundir / "proj"
    (proj / "src").mkdir(parents=True)
    os.symlink(_tt_dir(), proj / "tt")
    info = yaml.safe_load((ROOT / "info.yaml").read_text())
    info["project"]["tiles"] = fc["tiles"]
    info["project"]["top_module"] = fc["top"]
    info["project"]["source_files"] = fc["sources"]
    (proj / "info.yaml").write_text(yaml.safe_dump(info, sort_keys=False))
    rtlparams.materialize(build, fc["sources"], proj / "src")
    for s in fc.get("extra_src", []):
        dst = proj / "src" / s
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "src" / s, dst)
    config_json = json.loads((ROOT / "src" / "config.json").read_text())
    config_json.pop("//", None)
    config_json["CLOCK_PERIOD"] = fc["clock_period"]
    config_json.update(fc["librelane"])
    (proj / "src" / "config.json").write_text(json.dumps(config_json, indent=2))
    bindir = str(Path(sys.executable).parent)
    env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}")      # tt_tool runs yowasp-yosys
    p = subprocess.run([sys.executable, str(proj / "tt" / "tt_tool.py"), "--project-dir", ".",
                        "--create-user-config", "--ihp"], env=env, cwd=proj, capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError("tt_tool.py --create-user-config failed: " + (p.stdout + p.stderr)[-500:])
    return proj, "src/config_merged.json", fc


def run_librelane(workdir: Path, config_name: str, to: str = "") -> int:
    """phys/flow.py's invocation; `to` stops after that step (e.g. Verilator.Lint) for a config check."""
    (workdir / "runs" / "wokwi").mkdir(parents=True, exist_ok=True)
    cmd = ('python -m librelane --pdk-root "$PDK_ROOT" --pdk ihp-sg13cmos5l --manual-pdk '
           '--run-tag wokwi --force-run-dir runs/wokwi --hide-progress-bar '
           + (f"--to {shlex.quote(to)} " if to else "") + config_name)
    with open(workdir / "flow.log", "w") as log:
        return subprocess.run([str(LL), "bash", "-c", f"cd '{workdir}' && {cmd}"],
                              stdout=log, stderr=subprocess.STDOUT).returncode


class Flow:
    name = "flow"
    level = "flow"

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        return _refusal(self.name, config, workload, flow=True)

    def knob_support(self) -> dict:
        return _support_table(True)

    def workloads(self) -> list[Workload]:
        return [CHIP]

    def is_long(self, spec: RunSpec) -> bool:
        return True

    def code_hash(self) -> str:
        return _hash_files(["src/*.sv", "src/*.v", "src/config.json", "src/pdn_cfg.tcl", "tools/bench/rtlparams.py",
                            "tools/bench/backends/physical.py", "phys/flow.py", "phys/cfg/mvp-slow.json",
                            "phys/cfg/mvp-sram.json"])

    def run(self, spec: RunSpec) -> list[Record]:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        cfg = knobs.normalize(spec.config)
        w, chash = spec.workload, knobs.config_hash(cfg)
        started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        t0 = time.perf_counter()
        rundir = _runs_dir(cfg) / "flow"
        workdir, cname, fc = prepare_flow(cfg, rundir)
        rc = run_librelane(workdir, cname)
        mpath = workdir / "runs" / "wokwi" / "final" / "metrics.json"
        prov = {"command": _command(self.name, cfg), "commit": _git("rev-parse", "HEAD"),
                "dirty": bool(_git("status", "--porcelain")), "started": started,
                "duration_s": round(time.perf_counter() - t0, 1), "run_dir": str(rundir.relative_to(ROOT))}
        if rc != 0 or not mpath.exists():
            raise RuntimeError(f"LibreLane failed (rc={rc}); see {workdir / 'flow.log'}")
        vals = parse_final_metrics(json.loads(mpath.read_text()), float(fc["clock_period"]))
        key = spec.key(self.code_hash())
        recs = []
        for m, v in vals.items():
            if v is None:
                recs.append(_rec(w, m, None, self.name, self.level, cfg, chash, key, prov, status="n/a",
                                 note=f"not in {mpath.name}"))
            else:
                recs.append(_rec(w, m, v, self.name, self.level, cfg, chash, key, prov))
        recs.append(_rec(w, "precheck_pass", None, self.name, self.level, cfg, chash, key, prov, status="n/a",
                         note="the TT precheck is a separate step, not run by this backend"))
        return recs

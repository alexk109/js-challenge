"""Backend `power` (level "power", always long): gate-level activity + OpenROAD report_power per corner.

It drives research/power (loaded, not copied): `power_bench.power_mw()` runs the gate-level cocotb
simulation for a workload if its VCD is missing (research/power/run_sim.sh -> sim_out/<tag>/<w>.vcd) and
then one `report_power` (run_power.sh + power.tcl: liberty of the corner, final odb, sdc, SPEF, read_vcd).

A power run needs a *finished flow* for the config.  Sources, in order:
  1. `<main checkout>/runs/bench-<config_hash>/flow/proj/runs/wokwi` (the `flow` backend's output);
  2. for the MVP config only: the research run `<main checkout>/runs/pwr-mvp/proj/runs/wokwi` (phys/cfg/pwr-mvp.json =
     mvp-slow with magic DRC off).
The repo is found with `git rev-parse --git-common-dir`, so every checkout sees the same artifacts.
No flow => supports() refuses ("run the flow backend for this config first (long)").

Workloads (chip-level, no params): idle, uart_tx, i2c, both_engines, host_spi_burst = the research set
(research/power/README.md).  Records are totals per corner plus a typical-corner breakdown.
"""
from __future__ import annotations

import fcntl
import hashlib
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from .. import knobs
from ..metrics import METRICS
from ..types import Record, RunSpec, Workload
from .physical import _git, _load

ROOT = Path(__file__).resolve().parents[3]
HERE = ROOT / "research" / "power"

#: bench workload id -> research/power workload name
WORKLOADS = {"idle": "idle", "uart_tx": "uart", "i2c": "i2c", "both_engines": "both", "host_spi_burst": "hostburst"}
DESCR = {"idle": "engines disabled, clock running (4000 cycles)",
         "uart_tx": "UART TX 115200 Bd on engine 0 (16000 cycles)",
         "i2c": "I2C controller fast-mode write+read on engine 0 (25669 cycles)",
         "both_engines": "UART TX looped to UART RX, both engines busy (16000 cycles)",
         "host_spi_burst": "host SPI burst: 32-word IMEM write + 4-word read (19244 cycles)"}
CORNERS = {"typ": "nom_typ_1p20V_25C", "slow": "nom_slow_1p08V_125C", "fast": "nom_fast_1p32V_m40C"}
FLOW_HINT = "run the flow backend for this config first (long)"
MACRO = "RM_IHPSG13_2P_64x22_c2_bm_bist"

_GROUPS = {"Sequential": "seq_mw", "Combinational": "comb_mw", "Clock": "clock_tree_mw", "Macro": "macro_mw"}


# ------------------------------------------------------------------------------------ report parsing

def parse_report(text: str) -> dict:
    """Pure: the summary table of OpenROAD `report_power` -> {group: {internal, switching, leakage, total}} in mW.
    Groups: Sequential, Combinational, Clock, Macro, Pad, Total.  Raises ValueError if there is no Total row."""
    out: dict = {}
    for line in text.splitlines():
        p = line.split()
        if len(p) >= 5 and p[0] in ("Sequential", "Combinational", "Clock", "Macro", "Pad", "Total"):
            try:
                v = [float(x) * 1e3 for x in p[1:5]]
            except ValueError:
                continue
            out[p[0]] = dict(zip(("internal", "switching", "leakage", "total"), v))
    if "Total" not in out:
        raise ValueError("no Total row in the report_power output")
    return out


def parse_macro_total(text: str) -> float:
    """Pure: research/power/macro_energy.py output -> its 'TOTAL macro dynamic' in mW."""
    m = re.search(r"TOTAL macro dynamic:\s*([\d.]+)\s*mW", text)
    if not m:
        raise ValueError("no TOTAL macro dynamic line")
    return float(m.group(1))


# ------------------------------------------------------------------------------------ flow sources

@dataclass(frozen=True)
class Source:
    """A finished flow: `name` is the run dir relative to runs/ (GL_CFG of `make test-gl`), `rundir` the
    ROOT-relative path (may start with ../) to proj/runs/wokwi, `tag` the sim_out tag."""
    name: str
    rundir: str
    tag: str
    kind: str                # "bench" | "research-mvp"

    def final(self) -> Path:
        return ROOT / self.rundir / "final"


def _finished(run_root: Path) -> bool:
    f = run_root / "proj" / "runs" / "wokwi" / "final"
    return any((f / "odb").glob("*.odb")) and any((f / "nl").glob("*.nl.v")) and any((f / "spef" / "nom").glob("*.spef")) \
        and any((f / "sdc").glob("*.sdc"))


def main_checkout() -> Path:
    """The repo's root (from any checkout), so all checkouts share one runs/ of flow artifacts."""
    common = _git("rev-parse", "--git-common-dir")
    return (ROOT / common).resolve().parent / _git("rev-parse", "--show-prefix") if common else ROOT


def runs_root() -> Path:
    return main_checkout() / "runs"


def _rel(p: Path) -> str:
    """ROOT-relative (may start with ../): the research shell scripts prepend $root/ to the run dir."""
    return os.path.relpath(p, ROOT)


def find_source(config: dict) -> Optional[Source]:
    """The finished flow for a config, or None.  Looked up under <main checkout>/runs:
    bench-<hash>/flow (the flow backend), or for the MVP config runs/pwr-mvp (the research run)."""
    cfg = knobs.normalize(config)
    chash = knobs.config_hash(cfg)
    rr = runs_root()
    if _finished(rr / f"bench-{chash}" / "flow"):
        return Source(f"bench-{chash}/flow", _rel(rr / f"bench-{chash}/flow/proj/runs/wokwi"), f"bench-{chash}", "bench")
    if cfg == knobs.normalize({}) and _finished(rr / "pwr-mvp"):
        return Source("pwr-mvp", _rel(rr / "pwr-mvp/proj/runs/wokwi"), "mvp", "research-mvp")
    return None


def input_hash_for(config: dict) -> str:
    """Size+mtime of the flow artifacts a run reads (metrics.json, netlist, odb, sdc, SPEF): a re-run flow changes the cache key."""
    src = find_source(config)
    if src is None:
        return "noflow"
    h = hashlib.sha256()
    f = src.final()
    for pat in ("metrics.json", "nl/*.nl.v", "odb/*.odb", "sdc/*.sdc", "spef/nom/*.spef"):
        for p in sorted(f.glob(pat)):
            st = p.stat()
            h.update(f"{p.relative_to(f)}|{st.st_size}|{st.st_mtime_ns}\0".encode())
    return h.hexdigest()[:12]


_RESEARCH = None


def research():
    """research/power/power_bench.py as a module."""
    global _RESEARCH
    if _RESEARCH is None:
        _RESEARCH = _load("power_bench", HERE / "power_bench.py")
    return _RESEARCH


# ------------------------------------------------------------------------------------ the backend

def _rec(w: Workload, metric: str, value, cfg: dict, chash: str, key: str, prov: dict, method="measured",
         status="ok", note="", name="power", level="power") -> Record:
    return Record(workload=w.id, metric=metric, value=value, unit=METRICS[metric][0], level=level, backend=name,
                  method=method, status=status, config=cfg, config_hash=chash, workload_params=dict(w.params),
                  note=note, provenance=dict(prov), run_key=key)


def make_records(w: Workload, cfg: dict, key: str, prov: dict, reports: dict, macro_dyn_mw: Optional[float],
                 sram: bool, src_note: str) -> list[Record]:
    """Pure over parsed reports: {corner short name: parse_report dict} (+ macro dynamic mW or None) -> Records."""
    chash = knobs.config_hash(cfg)
    base = f"{DESCR[w.id]}; {src_note}"
    recs = []
    for c, label in (("typ", "typical 1.20 V 25 C"), ("slow", "slow 1.08 V 125 C"), ("fast", "fast 1.32 V -40 C")):
        if c not in reports:
            recs.append(_rec(w, f"power_mw_{c}", None, cfg, chash, key, prov, status="n/a", note=f"{label} corner not run"))
            continue
        extra = ("; excludes SRAM macro dynamic power, see power_mw_typ_incl_macro" if sram else "")
        recs.append(_rec(w, f"power_mw_{c}", round(reports[c]["Total"]["total"], 4), cfg, chash, key, prov,
                         note=f"{label}; OpenROAD report_power over a gate-level VCD window; {base}{extra}"))
    typ = reports.get("typ")
    if typ is None:
        return recs
    for grp, metric in _GROUPS.items():
        if metric == "macro_mw":
            continue
        recs.append(_rec(w, metric, round(typ.get(grp, {"total": 0.0})["total"], 4), cfg, chash, key, prov,
                         note=f"typical corner, report_power {grp} group; {base}"))
    recs.append(_rec(w, "leakage_mw", round(typ["Total"]["leakage"], 6), cfg, chash, key, prov,
                     note=f"typical corner, report_power Total leakage; {base}"))
    mac = typ.get("Macro", {"total": 0.0})["total"]
    if sram:
        if macro_dyn_mw is None:
            recs.append(_rec(w, "macro_mw", None, cfg, chash, key, prov, status="n/a",
                             note="macro dynamic power could not be computed from the VCD"))
            recs.append(_rec(w, "power_mw_typ_incl_macro", None, cfg, chash, key, prov, status="n/a",
                             note="macro dynamic power could not be computed from the VCD"))
        else:
            recs.append(_rec(w, "power_mw_typ_incl_macro", round(typ["Total"]["total"] + macro_dyn_mw, 4), cfg, chash, key, prov,
                             method="estimate",
                             note=(f"typical corner: report_power total {typ['Total']['total']:.4f} mW (macro leakage only) + estimated "
                                   f"macro dynamic {macro_dyn_mw:.4f} mW (macro_energy.py, liberty energy per access); the headline "
                                   f"total for an SRAM config; {base}")))
            recs.append(_rec(w, "macro_mw", round(mac + macro_dyn_mw, 4), cfg, chash, key, prov, method="estimate",
                             note=(f"computed outside OpenROAD: macro leakage {mac:.5f} mW from report_power + dynamic "
                                   f"{macro_dyn_mw:.4f} mW = rising A_CLK/B_CLK edges per (MEN,WEN,REN) state in the VCD x liberty "
                                   f"energy per edge (research/power/macro_energy.py); OpenROAD does not annotate the macro pins")))
    else:
        recs.append(_rec(w, "macro_mw", round(mac, 4), cfg, chash, key, prov, note=f"no SRAM macro; report_power Macro group; {base}"))
    return recs


def ensure_vcd(src: Source, rw: str, sram: bool) -> Path:
    """The gate-level VCD of workload `rw` for this flow, generated from the run's own netlist when missing.
    Serialised per tag: run_sim.sh builds into one sim_build/<tag>, so parallel workloads would clobber each other.
    A failed simulation raises with the script's output (research/power/run_sim.sh swallows the exit status)."""
    vcd = HERE / "sim_out" / src.tag / f"{rw}.vcd"
    lock = ROOT / "runs" / f".power-sim-{src.tag}.lock"
    lock.parent.mkdir(exist_ok=True)
    with open(lock, "w") as lk:
        fcntl.flock(lk, fcntl.LOCK_EX)
        if vcd.exists() and Path(str(vcd) + ".window").exists():
            return vcd
        netlist = next((src.final() / "nl").glob("*.nl.v"))
        p = subprocess.run([str(HERE / "run_sim.sh"), src.tag, "yes", "1" if sram else "0", str(netlist), rw],
                           cwd=ROOT, capture_output=True, text=True)
        if p.returncode != 0 or not vcd.exists():
            raise RuntimeError(f"gate-level simulation of {rw} on {netlist} produced no VCD (rc={p.returncode}): "
                               + (p.stdout + p.stderr)[-600:])
    return vcd


def power_report(rw: str, src: Source, corner: str, sram: bool, outdir: str) -> dict:
    """One report_power through research/power/power_bench.power_mw; a failing run_power.sh raises with the tail of its log."""
    try:
        return research().power_mw(rw, rundir=src.rundir, tag=src.tag, corner=corner, sram=sram, outdir=outdir)
    except subprocess.CalledProcessError as e:
        log = ROOT / outdir / f"{rw}.{corner}.log"
        tail = log.read_text()[-600:] if log.exists() else "(no log written)"
        raise RuntimeError(f"{' '.join(map(str, e.cmd[:1]))} failed (rc={e.returncode}) for {rw} at {corner}; {log.name}: {tail}") from e


def regmap_refusal(cfg: dict, who: str) -> Optional[str]:
    """None when the testbench `who` can drive this config: test/test.py and research/power/test_power.py speak the
    MVP register map (7-bit addresses, 4-deep FIFOs), so on any other map their host writes miss and the chip sits
    idle -- a run there measures nothing it claims to (seen: gl 0/4, every power workload equal to idle)."""
    shape = {k: cfg[k] for k in ("n_engines", "imem_words", "fifo_depth", "engine_route")}
    mvp = {k: knobs.normalize({})[k] for k in shape}
    if shape == mvp:
        return None
    diff = ", ".join(f"{k}={v}" for k, v in shape.items() if v != mvp[k])
    return f"{who} speaks the MVP register map only; {diff} changes it (tools/bench/sim/REGMAP.md)"


class Power:
    name = "power"
    level = "power"

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        if workload.id not in WORKLOADS or workload.params:
            return f"power runs {', '.join(WORKLOADS)} (no params); got {workload.id!r}"
        cfg = knobs.normalize(config)
        if workload.id != "idle":       # idle makes no host access, so it is valid on any register map
            reason = regmap_refusal(cfg, f"the power testbench's {workload.id!r} activity")
            if reason:
                return reason
        if find_source(cfg) is None:
            return FLOW_HINT
        return None

    def knob_support(self) -> dict:
        from .physical import _support_table
        t = _support_table(True)
        t["t_in"] = "partial: the activity firmware is the MVP's; power is measured, not asserted correct"
        for k in ("n_engines", "imem_words", "fifo_depth", "engine_route"):
            t[k] = "partial: off the MVP register map only 'idle' runs (the activity testbench speaks the MVP map)"
        return t

    def workloads(self) -> list[Workload]:
        return [Workload(i, "chip", {}, DESCR[i]) for i in WORKLOADS]

    def is_long(self, spec: RunSpec) -> bool:
        return True

    def input_hash(self, spec: RunSpec) -> str:
        """Optional backend hook (runner mixes it into the cache key): the flow artifacts this run reads."""
        return input_hash_for(spec.config)

    def code_hash(self) -> str:
        h = hashlib.sha256()
        for g in ("research/power/*.sh", "research/power/*.tcl", "research/power/*.py", "research/power/tb_power.v",
                  "research/power/Makefile", "src/*.sv", "src/*.v", "test/test.py", "test/tb.v", "tools/bench/backends/power.py"):
            for p in sorted(ROOT.glob(g)):
                h.update(str(p.relative_to(ROOT)).encode() + b"\0" + p.read_bytes() + b"\0")
        return h.hexdigest()[:16]

    def _command(self, cfg: dict, w: Workload) -> str:
        sets = " ".join(f"--set {k}={v!r}" for k, v in sorted(knobs.diff(cfg).items()))
        return f"PYTHONPATH=. {sys.executable} -m tools.bench run {sets} --workload {w.id} --backend power --allow-long".replace("  ", " ")

    def run(self, spec: RunSpec) -> list[Record]:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        cfg = knobs.normalize(spec.config)
        w = spec.workload
        src = find_source(cfg)
        sram = cfg["imem_tech"] == "sram2p"
        rw = WORKLOADS[w.id]
        started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        t0 = time.perf_counter()
        outdir = f"runs/bench-{knobs.config_hash(cfg)}/power"
        ensure_vcd(src, rw, sram)
        reports: dict = {}
        for short, corner in CORNERS.items():
            res = power_report(rw, src, corner, sram, outdir)
            reports[short] = parse_report((ROOT / res["report"]).read_text())
        macro = None
        if sram:
            vcd = HERE / "sim_out" / src.tag / f"{rw}.vcd"
            p = subprocess.run([sys.executable, str(HERE / "macro_energy.py"), str(vcd), (Path(str(vcd) + ".window")).read_text().strip()],
                               capture_output=True, text=True, cwd=ROOT)
            macro = parse_macro_total(p.stdout) if p.returncode == 0 else None
        odb = next((src.final() / "odb").glob("*.odb"))
        st = odb.stat()
        prov = {"command": self._command(cfg, w), "commit": _git("rev-parse", "HEAD"),
                "dirty": bool(_git("status", "--porcelain", "-uno")), "started": started,
                "duration_s": round(time.perf_counter() - t0, 1), "run_dir": src.rundir,
                "reports": f"{outdir}/{rw}.<corner>.rpt",
                "input_hashes": {"odb": f"{odb.name}:{st.st_size}:{st.st_mtime_ns}"}}
        note = (f"flow source {src.rundir}" + (" (research run phys/cfg/pwr-mvp.json = mvp-slow, magic DRC off)"
                if src.kind == "research-mvp" else ""))
        return make_records(w, cfg, spec.key(self.code_hash() + ":" + self.input_hash(spec)), prov, reports, macro, sram, note)

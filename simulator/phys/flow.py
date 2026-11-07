#!/usr/bin/env python3
"""Physical flow driver behind `make flow CFG=<name>`.

Reads phys/cfg/<name>.json, runs LibreLane inside tools/ll and appends one row
to metrics/flow.csv. Two kinds of config:

  kind=tt     The TT top level, hardened the way tt-gds-action@ihp-cmos5l does
              it: tt_tool.py --create-user-config on a shadow project (so the
              tile DEF, die area and RT_MAX_LAYER come from tt-support-tools),
              then `python -m librelane ... --manual-pdk src/config_merged.json`.
  kind=block  A standalone module (probes). Same PDK, layer limits and PDN as a
              TT project, but a free-sized die at `core_util`, so a partial
              design's area and timing can be measured on its own.

Config fields: kind, top, sources (relative to src/ for tt, repo root for
block), extra_src (files relative to src/ copied into the shadow project but not
compiled, e.g. a macro blackbox netlist or pdn_cfg.tcl), tiles (tt), core_util
(block), clock_period (ns), params (recorded in the row; for block also passed to
Yosys as SYNTH_PARAMETERS), tag, librelane (raw LibreLane overrides).
"""

import argparse
import csv
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
LL = ROOT / "tools" / "ll"
FLOW_CSV = ROOT / "metrics" / "flow.csv"

COLUMNS = [
    "date", "commit", "harness", "cfg", "tag", "kind", "tiles",
    "N_SM", "IMEM_DEPTH", "DATA_W", "FIFO_DEPTH", "EN_flags",
    "clock_period_ns", "cells", "seq_cells", "stdcell_area_um2", "core_area_um2",
    "util_pct", "die_wxh", "WNS", "TNS", "hold_WNS", "fmax_tt", "fmax_ss",
    "drc_count", "routing_overflow", "cts_buffers", "antenna_viol", "lvs_errors",
    "status", "runtime_s", "flow_version",
]
PARAM_COLUMNS = ["N_SM", "IMEM_DEPTH", "DATA_W", "FIFO_DEPTH"]

TT_CORNER = "nom_typ_1p20V_25C"
SS_CORNER = "nom_slow_1p08V_125C"

# Same PDN/margins/limits as the TT template's src/config.json, for block probes.
BLOCK_DEFAULTS = {
    "VDD_PIN": "VPWR",
    "GND_PIN": "VGND",
    "RT_MAX_LAYER": "Metal4",
    "FP_SIZING": "relative",
    "FP_PDN_MULTILAYER": 0,
    "FP_PDN_VPITCH": 50.0,
    "FP_PDN_VWIDTH": 2.1,
    "GRT_ALLOW_CONGESTION": 1,
    "RUN_KLAYOUT_XOR": 0,
    "RUN_KLAYOUT_DRC": 0,
    "RUN_LINTER": 1,
    "PL_RESIZER_HOLD_SLACK_MARGIN": 0.1,
    "GRT_RESIZER_HOLD_SLACK_MARGIN": 0.05,
}


def sh(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run([str(c) for c in cmd], **kw)


def git_commit():
    p = subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                       capture_output=True, text=True)
    if p.returncode != 0:
        return "nogit"
    dirty = subprocess.run(["git", "-C", ROOT, "status", "--porcelain", "src", "phys"],
                           capture_output=True, text=True).stdout.strip()
    return p.stdout.strip() + ("-dirty" if dirty else "")


def load_json_with_comments(path):
    # TT's config.json repeats the "//" key; json keeps the last one, which is fine.
    with open(path) as f:
        return json.load(f)


def strip_comments(obj):
    """Drop "//"-prefixed keys, recursively.

    The SRAM configs document each LibreLane key next to it (src/config.json sets
    that precedent), but an unknown key is an error in LibreLane, and a "//" key
    inside MACROS[..].instances would be read as a macro instance.
    """
    if isinstance(obj, dict):
        return {k: strip_comments(v) for k, v in obj.items() if not k.startswith("//")}
    if isinstance(obj, list):
        return [strip_comments(v) for v in obj]
    return obj


def prepare_tt(cfg, rundir):
    proj = rundir / "proj"
    (proj / "src").mkdir(parents=True)
    os.symlink(ROOT / "tt", proj / "tt")

    with open(ROOT / "info.yaml") as f:
        info = yaml.safe_load(f)
    info["project"]["tiles"] = cfg["tiles"]
    info["project"]["top_module"] = cfg["top"]
    info["project"]["source_files"] = cfg["sources"]
    with open(proj / "info.yaml", "w") as f:
        yaml.safe_dump(info, f, sort_keys=False)

    # Copies, not symlinks: tt_tool's port check runs yowasp-yosys, whose WASM
    # sandbox can't follow links out of the project directory.
    for s in cfg["sources"]:
        dst = proj / "src" / s
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "src" / s, dst)

    # Files the LibreLane config points at with dir::, which resolves against the
    # config's own directory (src/config_merged.json), so they have to land in
    # proj/src/ too: PDN_CFG, a macro's port-only blackbox netlist, ...
    for s in cfg.get("extra_src", []):
        dst = proj / "src" / s
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "src" / s, dst)

    config = load_json_with_comments(ROOT / "src" / "config.json")
    config.pop("//", None)
    if "clock_period" in cfg:
        config["CLOCK_PERIOD"] = cfg["clock_period"]
    config.update(strip_comments(cfg.get("librelane", {})))
    if config.get("PDN_MACRO_CONNECTIONS"):
        # The instance word is a regex in LibreLane and the generate-loop `[0]` of g_eng[0] is a
        # character class there (OpenROAD.Floorplan dies: "No match found for regular expression").
        # Same escape as tools/bench/backends/physical.py escape_macro_brackets (duplicated: that module
        # pulls in the bench package; keep the two in step).
        config["PDN_MACRO_CONNECTIONS"] = [
            re.sub(r"^(\S+)", lambda m: re.sub(r"\[(\d+)\]", r".{1,2}\1.{1,2}", m.group(1)), e)
            for e in config["PDN_MACRO_CONNECTIONS"]]
    with open(proj / "src" / "config.json", "w") as f:
        json.dump(config, f, indent=2)

    venv_bin = ROOT / ".venv" / "bin"
    env = dict(os.environ, PATH=f"{venv_bin}:{os.environ['PATH']}")  # tt_tool runs yowasp-yosys
    p = sh([venv_bin / "python", ROOT / "tt" / "tt_tool.py",
            "--project-dir", ".", "--create-user-config", "--ihp"], env=env, cwd=proj)
    if p.returncode != 0:
        sys.exit("tt_tool.py --create-user-config failed")
    return proj, "src/config_merged.json"


def prepare_block(cfg, rundir):
    config = dict(BLOCK_DEFAULTS)
    config.update({
        "DESIGN_NAME": cfg["top"],
        "VERILOG_FILES": [str(ROOT / s) for s in cfg["sources"]],
        "CLOCK_PORT": cfg.get("clock_port", "clk"),
        "CLOCK_PERIOD": cfg.get("clock_period", 20),
        "FP_CORE_UTIL": cfg.get("core_util", 40),
        "PL_TARGET_DENSITY_PCT": cfg.get("core_util", 40) + 10,
    })
    params = {k: v for k, v in cfg.get("params", {}).items() if k != "EN_flags"}
    if params:
        config["SYNTH_PARAMETERS"] = [f"{k}={v}" for k, v in params.items()]
    config.update(strip_comments(cfg.get("librelane", {})))
    with open(rundir / "config.json", "w") as f:
        json.dump(config, f, indent=2)
    return rundir, "config.json"


def run_librelane(workdir, config_name):
    (workdir / "runs" / "wokwi").mkdir(parents=True, exist_ok=True)  # as tt_tool.harden does
    cmd = ('python -m librelane --pdk-root "$PDK_ROOT" --pdk ihp-sg13cmos5l --manual-pdk '
           '--run-tag wokwi --force-run-dir runs/wokwi --hide-progress-bar ' + config_name)
    with open(workdir / "flow.log", "w") as log:
        p = subprocess.run([str(LL), "bash", "-c", f"cd '{workdir}' && {cmd}"],
                           stdout=log, stderr=subprocess.STDOUT)
    return p.returncode


def metric(m, *names, default=""):
    for n in names:
        if n in m and m[n] is not None:
            return m[n]
    return default


def fmax_mhz(period, ws):
    if ws == "" or ws is None:
        return ""
    return round(1000.0 / (period - float(ws)), 1)


def gpl_util(run):
    for log in run.glob("*-openroad-globalplacement/openroad-globalplacement.log"):
        for line in open(log):
            mt = re.match(r"^\[INFO GPL-0019\] (?:Util|Utilization):\s*([\d.]+)", line)
            if mt:
                return float(mt.group(1))
    return ""


def make_row(cfg_name, cfg, workdir, rc, runtime):
    run = workdir / "runs" / "wokwi"
    mpath = run / "final" / "metrics.json"
    m = json.load(open(mpath)) if mpath.exists() else {}
    period = float(cfg.get("clock_period", cfg.get("librelane", {}).get("CLOCK_PERIOD", 20)))

    ws_tt = metric(m, f"timing__setup__ws__corner:{TT_CORNER}", "timing__setup__ws")
    ws_ss = metric(m, f"timing__setup__ws__corner:{SS_CORNER}")
    die = metric(m, "design__die__bbox")
    if isinstance(die, str) and die:
        x0, y0, x1, y1 = (float(v) for v in die.split())
        die = f"{x1 - x0:.0f}x{y1 - y0:.0f}"

    flow_version = ""
    resolved = run / "resolved.json"
    if resolved.exists():
        flow_version = "librelane-" + json.load(open(resolved))["meta"]["librelane_version"]

    params = cfg.get("params", {})
    row = {
        "date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "commit": git_commit(),
        "harness": os.environ.get("HARNESS", "local"),
        "cfg": cfg_name,
        "tag": cfg.get("tag", ""),
        "kind": cfg["kind"],
        "tiles": cfg.get("tiles", ""),
        "EN_flags": params.get("EN_flags", ""),
        "clock_period_ns": period,
        "cells": metric(m, "design__instance__count__stdcell", "design__instance__count"),
        "seq_cells": metric(m, "design__instance__count__class:sequential_cell"),
        "stdcell_area_um2": metric(m, "design__instance__area__stdcell", "design__instance__area"),
        "core_area_um2": metric(m, "design__core__area"),
        "util_pct": gpl_util(run),
        "die_wxh": die,
        "WNS": ws_tt,
        "TNS": metric(m, f"timing__setup__tns__corner:{TT_CORNER}", "timing__setup__tns"),
        "hold_WNS": metric(m, "timing__hold__ws"),
        "fmax_tt": fmax_mhz(period, ws_tt),
        "fmax_ss": fmax_mhz(period, ws_ss),
        "drc_count": metric(m, "route__drc_errors", "magic__drc_error__count"),
        "routing_overflow": metric(m, "global_route__overflow", "route__overflow"),
        "cts_buffers": metric(m, "design__instance__count__class:clock_buffer"),
        "antenna_viol": metric(m, "antenna__violating__nets"),
        "lvs_errors": metric(m, "design__lvs_error__count"),
        "status": "ok" if rc == 0 else f"fail({rc})",
        "runtime_s": round(runtime),
        "flow_version": flow_version,
    }
    for k in PARAM_COLUMNS:
        row[k] = params.get(k, "")
    return row


def append_row(row):
    new = not FLOW_CSV.exists() or FLOW_CSV.stat().st_size == 0
    with open(FLOW_CSV, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        if new:
            w.writeheader()
        w.writerow(row)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cfg")
    ap.add_argument("--no-row", action="store_true", help="don't append to metrics/flow.csv")
    args = ap.parse_args()

    cfg_path = ROOT / "phys" / "cfg" / f"{args.cfg}.json"
    cfg = json.load(open(cfg_path))
    rundir = ROOT / "runs" / args.cfg
    shutil.rmtree(rundir, ignore_errors=True)
    rundir.mkdir(parents=True)

    if cfg["kind"] == "tt":
        workdir, config_name = prepare_tt(cfg, rundir)
    elif cfg["kind"] == "block":
        workdir, config_name = prepare_block(cfg, rundir)
    else:
        sys.exit(f"unknown kind {cfg['kind']!r}")

    t0 = time.time()
    rc = run_librelane(workdir, config_name)
    row = make_row(args.cfg, cfg, workdir, rc, time.time() - t0)
    if not args.no_row:
        append_row(row)
    print(json.dumps(row, indent=1))
    if rc != 0:
        print(f"LibreLane failed, see {workdir / 'flow.log'}", file=sys.stderr)
    return rc


if __name__ == "__main__":
    sys.exit(main())

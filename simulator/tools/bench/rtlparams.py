"""ONE mapping from bench knobs to RTL build settings (Verilog parameters, defines, extra sources,
LibreLane config keys).  Every physical backend (synth, flow) and, later, task H's RTL variants go
through `rtl_build(config)`; nothing else in the bench decides how a knob becomes RTL.

Structural knobs: `n_engines` -> N_ENGINES, `imem_words` -> IMEM_WORDS, `fifo_depth` ->
FIFO_DEPTH, `engine_route` -> ENGINE_ROUTE, parameters of `tt_um_protocol_emulator` (which hands them to
host_spi / pio_chip; the host address width follows from them, see src/pio_chip.sv).  They are emitted
only when they differ from the MVP, so the MVP build is the untouched source.  `sim_build()` gives the
same settings in the form the Verilator / Icarus simulation builds (fw/Makefile) take.

`t_in` -> parameter T_IN of pio_chip; `imem_tech=sram2p` -> the IMEM_SRAM define, the
pio_imem_sram.sv source, and the macro / PDN keys taken from phys/cfg/mvp-sram.json (read, not copied,
so that config stays the single source of truth); `f_core_mhz` -> clock period (flow only).
A knob not listed in `supported_knobs()` is refused by the backends until task H adds it here.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

from . import knobs

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
SRAM_CFG = ROOT / "phys" / "cfg" / "mvp-sram.json"
BASE_CFG = ROOT / "phys" / "cfg" / "mvp-slow.json"

#: module -> source file (under src/) that declares it; where a parameter override is applied.
MODULE_FILE = {"pio_chip": "pio_chip.sv", "tt_um_protocol_emulator": "tt_um_protocol_emulator.v"}

#: bench knob -> (Verilog parameter, value converter) for the structural knobs, all on the top level
STRUCTURAL = {"n_engines": ("N_ENGINES", int), "imem_words": ("IMEM_WORDS", int),
              "fifo_depth": ("FIFO_DEPTH", int), "engine_route": ("ENGINE_ROUTE", int)}

_SUPPORTED = {
    "t_in": "parameter T_IN on pio_chip (default 3)",
    "imem_tech": "flop (default) or sram2p: IMEM_SRAM define + macro files/config of phys/cfg/mvp-sram.json",
    "n_engines": "parameter N_ENGINES (1-4); a wide register map (>2 engines) gives host_spi a 2-byte command",
    "imem_words": "parameter IMEM_WORDS (16/32/64/128); PAGE field above 32; sram2p needs 32",
    "fifo_depth": "parameter FIFO_DEPTH (2-16) for every TX and RX FIFO",
    "engine_route": "parameter ENGINE_ROUTE: CTRL.ROUTE_EN links engine e's RX FIFO to engine e+1's TX FIFO (n_engines >= 2)",
    "f_core_mhz": "clock period 1000/f ns; used by the flow only (synthesis is period-independent here)",
}


def supported_knobs() -> dict:
    """{knob_id: how it is applied}.  Knobs not listed are refused (value != MVP default)."""
    return dict(_SUPPORTED)


def unsupported(config: dict) -> list[str]:
    """Knobs set away from their default that rtl_build cannot honour."""
    cfg = knobs.normalize(config)
    return [f"{k}={cfg[k]!r}" for k, d in knobs.defaults().items() if cfg[k] != d and k not in _SUPPORTED]


def _structural(cfg: dict) -> dict:
    """{PARAM: value} of the structural knobs that differ from the MVP; validated."""
    out = {p: conv(cfg[k]) for k, (p, conv) in STRUCTURAL.items() if cfg[k] != knobs.defaults()[k]}
    if cfg["engine_route"] and cfg["n_engines"] < 2:
        raise ValueError("engine_route needs n_engines >= 2")
    if cfg["imem_words"] not in (16, 32, 64, 128):
        raise ValueError(f"imem_words {cfg['imem_words']} not in 16/32/64/128")
    if not 2 <= cfg["fifo_depth"] <= 16:
        raise ValueError(f"fifo_depth {cfg['fifo_depth']} not in 2..16")
    if not 1 <= cfg["n_engines"] <= 4:
        raise ValueError(f"n_engines {cfg['n_engines']} not in 1..4")
    if cfg["imem_tech"] == "sram2p" and cfg["imem_words"] != 32:
        raise ValueError("imem_tech=sram2p holds 32 words per engine (imem_words=32)")
    return out


def invalid(config: dict):
    """Why this config cannot be built (a combination the RTL refuses), or None."""
    try:
        _structural(knobs.normalize(config))
    except ValueError as exc:
        return str(exc)
    return None


def sim_build(config: dict) -> dict:
    """The RTL parameters of a simulation build of `pio_chip` (the top of fw/Makefile).

    {"params": {PARAM: value} (T_IN and the structural ones, only those off the MVP),
     "tag": "" at the MVP, else "cfg-<hash>" naming the sim_build directory,
     "gflags": ["-GN_ENGINES=3", ...] (Verilator -G), "pflags": ["-Ppio_chip.N_ENGINES=3", ...] (Icarus -P)}
    """
    cfg = knobs.normalize(config)
    params = _structural(cfg)
    if cfg["t_in"] != knobs.defaults()["t_in"]:
        params["T_IN"] = int(cfg["t_in"])
    tag = ""
    if params:
        import hashlib
        tag = "cfg-" + hashlib.sha256(json.dumps(params, sort_keys=True).encode()).hexdigest()[:8]
    return {"params": params, "tag": tag, "gflags": [f"-G{k}={v}" for k, v in sorted(params.items())],
            "pflags": [f"-Ppio_chip.{k}={v}" for k, v in sorted(params.items())]}


def _strip(obj):
    if isinstance(obj, dict):
        return {k: _strip(v) for k, v in obj.items() if not k.startswith("//")}
    if isinstance(obj, list):
        return [_strip(v) for v in obj]
    return obj


def rtl_build(config: dict) -> dict:
    """Build settings for a bench config:

    params          {module: {PARAM: value}}  Verilog parameter overrides (see apply_params); the structural knobs
                                              (off the MVP only) are under tt_um_protocol_emulator
    defines         [str]                     preprocessor defines
    extra_sources   [str]                     src/ files added to the MVP source list
    extra_src       [str]                     src/ files copied for the flow but not compiled (blackbox, pdn tcl)
    librelane       {key: value}              LibreLane config keys merged over the base flow config
    clock_period_ns float
    macros          [str]                     macro typ-corner lib files, for area accounting in synthesis
    """
    cfg = knobs.normalize(config)
    bad = unsupported(cfg)
    if bad:
        raise ValueError("rtl_build cannot honour: " + ", ".join(bad))
    b: dict = {"params": {"pio_chip": {"T_IN": int(cfg["t_in"])}}, "defines": [], "extra_sources": [],
               "extra_src": [], "librelane": {}, "macros": [],
               "clock_period_ns": round(1000.0 / cfg["f_core_mhz"], 6)}
    top = _structural(cfg)
    if top:
        b["params"]["tt_um_protocol_emulator"] = top
    if cfg["imem_tech"] == "sram2p":
        sram = json.loads(SRAM_CFG.read_text())
        base = json.loads(BASE_CFG.read_text())
        b["extra_sources"] = [s for s in sram["sources"] if s not in base["sources"]]
        b["extra_src"] = list(sram.get("extra_src", []))
        b["librelane"] = _strip(sram["librelane"])
        b["defines"] = list(b["librelane"].get("VERILOG_DEFINES", []))
        for m in b["librelane"].get("MACROS", {}).values():
            b["macros"] += [m["lib"]["*_typ_*"][0]] * len(m["instances"])   # one entry per instance
    return b


def apply_params(text: str, overrides: dict) -> str:
    """Rewrite `parameter [type] NAME = <expr>` defaults in one module's source text."""
    for name, value in overrides.items():
        pat = re.compile(r"(\bparameter\s+(?:\w+\s+)?" + re.escape(name) + r"\s*=\s*)[^,)\n]+")
        text, n = pat.subn(lambda m: m.group(1) + str(value), text, count=1)
        if n != 1:
            raise ValueError(f"parameter {name} not found")
    return text


def materialize(build: dict, names: list[str], dest: Path) -> list[Path]:
    """Copy src/<name> for each name into dest, applying build['params'] to the declaring files."""
    by_file = {MODULE_FILE[m]: p for m, p in build["params"].items()}
    out = []
    for n in names:
        text = (SRC / n).read_text()
        if n in by_file:
            text = apply_params(text, by_file[n])
        p = dest / n
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
        out.append(p)
    return out


def _main(argv: list[str]) -> int:
    """`python -m tools.bench.rtlparams tag|gflags|pflags` : fw/Makefile's view of env BENCH_CONFIG.

    BENCH_CONFIG is the JSON of a bench config (the variable the parametric ISS reads too); unset or
    empty is the MVP.  `tag` is "" at the MVP, else the sim_build suffix "cfg-<hash>"; `gflags` are the
    Verilator -G options and `pflags` the Icarus -P options, empty at the MVP.
    """
    import os
    if len(argv) != 1 or argv[0] not in ("tag", "gflags", "pflags"):
        print(_main.__doc__, file=sys.stderr)
        return 2
    raw = os.environ.get("BENCH_CONFIG", "").strip()
    b = sim_build(json.loads(raw) if raw else {})
    print(b["tag"] if argv[0] == "tag" else " ".join(b[argv[0]]))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv[1:]))

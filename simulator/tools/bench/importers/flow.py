"""metrics/flow.csv: LibreLane physical flow rows. Workload = cfg (the config file name).

Config: cfg `mvp` and `mvp-slow` are the MVP (the slow one is the same design at the slow corner);
`mvp-sram` is the MVP with imem_tech=sram2p, but only for rows whose tag is the cfg itself and whose status is
`ok` (a tagged variant such as `mvp-sram-halo10`, which failed the TT precheck, is not that design); every other
row is an external probe or variant and gets config={} and config_hash "external:<tag or cfg>".
"""
from pathlib import Path

from .. import knobs
from ._common import SRAM_CONFIG, SRAM_CONFIG_HASH, external, load_metrics_csv, mvp

REL = "metrics/flow.csv"
MVP_CFGS = ("mvp", "mvp-slow")

#: column -> (metric name, unit). The judge's joinable names; the note keeps the source column.
RENAMES = {
    "stdcell_area_um2": ("area_um2", "um2"),
    "fmax_tt": ("fmax_mhz_typ", "MHz"),
    "fmax_ss": ("fmax_mhz_slow", "MHz"),
    "WNS": ("wns_ns", "ns"),
}


def sources(root: Path) -> list[Path]:
    return [root / REL]


def config_for(params: dict):
    cfg = params.get("cfg", "")
    if cfg in MVP_CFGS:
        return mvp(params)
    tag = params.get("tag") or cfg
    if cfg == "mvp-sram":
        if tag == cfg and params.get("status") == "ok":
            return dict(SRAM_CONFIG), SRAM_CONFIG_HASH, ""
        return external(tag)
    return external(cfg)


def load(root: Path):
    return load_metrics_csv(
        root, REL, "flow", "cfg",
        command=lambda p: f"make flow CFG={p.get('cfg', '')}",
        config_for=config_for,
        renames=RENAMES,
    )

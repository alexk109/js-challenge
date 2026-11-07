"""metrics/fuzz.csv: lockstep fuzz runs. Workload = harness; seed identifies the run and stays in workload_params.

Config: harness `lockstep` is the MVP; `lockstep-sram` is the MVP with imem_tech=sram2p; any other harness
name gets config={} and config_hash "external:<harness>".
"""
from pathlib import Path

from ._common import SRAM_CONFIG, SRAM_CONFIG_HASH, external, load_metrics_csv, mvp

REL = "metrics/fuzz.csv"


def sources(root: Path) -> list[Path]:
    return [root / REL]


def config_for(params: dict):
    harness = params.get("harness", "")
    if harness == "lockstep":
        return mvp(params)
    if harness == "lockstep-sram":
        return dict(SRAM_CONFIG), SRAM_CONFIG_HASH, ""
    return external(harness, why="fuzz harness with no bench config")


def load(root: Path):
    return load_metrics_csv(root, REL, "fuzz", "harness", exclude=("seed",), param_cols=("seed",), config_for=config_for)

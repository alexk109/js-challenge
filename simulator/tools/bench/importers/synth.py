"""metrics/synth.csv: Yosys-only area probes. Workload = probe; the probe name is also the config name (cfg).

Config: always external (a probe is not a bench config): config={} and config_hash "external:<probe>".
"""
from pathlib import Path

from ._common import external, load_metrics_csv

REL = "metrics/synth.csv"

#: area_um2 -> the joinable name shared with the flow (stdcell area); the note keeps the source column.
RENAMES = {"area_um2": ("area_um2", "um2")}


def sources(root: Path) -> list[Path]:
    return [root / REL]


def config_for(params: dict):
    return external(params.get("probe", ""), why="synth probe, not a bench config")


def load(root: Path):
    return load_metrics_csv(
        root, REL, "synth", "probe",
        command=lambda p: f"make synth-probe PROBE={p.get('probe', '')}",
        config_for=config_for,
        renames=RENAMES,
    )

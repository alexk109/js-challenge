"""metrics/fw.csv: firmware sweep rows. Workload = proto. The file has a second header (cycles_per_slot)."""
from pathlib import Path

from ._common import load_metrics_csv

REL = "metrics/fw.csv"


def sources(root: Path) -> list[Path]:
    return [root / REL]


def load(root: Path):
    return load_metrics_csv(root, REL, "fw", "proto")

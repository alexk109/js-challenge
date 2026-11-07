"""metrics/mutate.csv: one row per mutant. Workload = feature (the mutant's name and location)."""
from pathlib import Path

from ._common import load_metrics_csv

REL = "metrics/mutate.csv"


def sources(root: Path) -> list[Path]:
    return [root / REL]


def load(root: Path):
    return load_metrics_csv(root, REL, "mutate", "feature")

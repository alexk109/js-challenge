"""metrics/coverage.csv: functional coverage bins. Workload = bin."""
from pathlib import Path

from ._common import load_metrics_csv

REL = "metrics/coverage.csv"


def sources(root: Path) -> list[Path]:
    return [root / REL]


def load(root: Path):
    return load_metrics_csv(root, REL, "coverage", "bin")

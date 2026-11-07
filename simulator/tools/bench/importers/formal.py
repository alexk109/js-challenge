"""metrics/formal.csv: formal property results. Header only in this tree, so it yields no records."""
from pathlib import Path

from ._common import load_metrics_csv

REL = "metrics/formal.csv"


def sources(root: Path) -> list[Path]:
    return [root / REL]


def load(root: Path):
    return load_metrics_csv(root, REL, "formal", "property")

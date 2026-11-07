"""research/perf/a/perf.csv: the no-simulation protocol model. Workload = protocol; method derived."""
from pathlib import Path

from ._common import load_long_csv

REL = "research/perf/a/perf.csv"
COMMAND = ".venv/bin/python research/perf/a/build.py"


def sources(root: Path) -> list[Path]:
    return [root / REL]


def load(root: Path):
    return load_long_csv(
        root, REL, workload_col="protocol", metric_col="metric", value_col="value",
        unit_col="units", note_col="formula", param_cols=("variant",),
        method="derived", command=COMMAND,
    )

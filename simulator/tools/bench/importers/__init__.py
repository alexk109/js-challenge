"""Importers of past evidence (level "import"). One module per source; each exposes sources(root) and load(root).

`load_all(root)` returns every importer's records in SOURCES order. Nothing here types in a number.
"""
from __future__ import annotations

import importlib
from pathlib import Path

from ..types import Record

SOURCES = (
    "flow",          # metrics/flow.csv
    "synth",         # metrics/synth.csv
    "perf_a",        # research/perf/a/perf.csv
    "perf_b",        # research/perf/b/perf.csv
    "fw",            # metrics/fw.csv
    "formal",        # metrics/formal.csv
    "fuzz",          # metrics/fuzz.csv
    "mutate",        # metrics/mutate.csv
    "coverage",      # metrics/coverage.csv
    "experiments",   # research/ideation/round2/experiments/*/RESULT.md verdict lines
)


def module(name: str):
    return importlib.import_module(f"{__name__}.{name}")


def source_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for name in SOURCES:
        files.extend(module(name).sources(root))
    return files


def load_all(root: Path) -> list[Record]:
    records: list[Record] = []
    for name in SOURCES:
        for r in module(name).load(root):
            if r.backend == "import":
                r.backend = f"import:{name}"     # one side per source in cross-checks
            records.append(r)
    return records

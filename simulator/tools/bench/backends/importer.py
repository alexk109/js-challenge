"""Backend "import": the evidence imported from files already in the repo (level "import").

It runs the importers, which read files and never run a simulator. It takes one workload ("all") and
only the MVP config, since the imported files state no other config.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

from .. import knobs
from ..importers import load_all, source_files
from ..types import Workload, sha

ROOT = Path(__file__).resolve().parents[3]


class ImporterBackend:
    name = "import"
    level = "import"

    def supports(self, config: dict, workload: Workload):
        if workload.id != "all":
            return "importers take only the workload 'all'"
        if knobs.normalize(config) != knobs.normalize({}):
            return "imported evidence is MVP-only: the source files state no other config"
        return None

    def knob_support(self) -> dict:
        return {k.id: "default_only" for k in knobs.KNOBS}

    def workloads(self):
        return [Workload("all", "import", label="all importers")]

    def run(self, spec) -> list:
        return load_all(ROOT)

    def code_hash(self) -> str:
        files = {}
        here = Path(__file__).resolve().parent
        for py in sorted([*here.glob("*.py"), *(here.parent / "importers").glob("*.py")]):
            files[py.relative_to(ROOT).as_posix()] = hashlib.sha256(py.read_bytes()).hexdigest()
        for path in source_files(ROOT):
            rel = path.relative_to(ROOT).as_posix()
            files[rel] = hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else "missing"
        return sha(files, 24)

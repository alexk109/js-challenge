"""The bench's shared types. Every module and backend imports these; nobody redefines them.

Normative text: tools/bench/CONTRACT.md. Change this file only through the orchestrator, because
every parallel task builds against it.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from typing import Any, Optional, Protocol, Sequence

SCHEMA = 1

#: Fidelity levels, cheapest first. `import` is evidence produced outside the bench and read in.
LEVELS = ("analytic", "iss", "rtl", "synth", "gl", "flow", "power", "import")
#: `synth` = Yosys synthesis only (area, cell count; minutes, not long). `flow` = full LibreLane (long).

#: How a value was obtained (archsim's vocabulary, so the two read alike).
METHODS = ("measured", "derived", "estimate", "spec", "imported")

#: Outcome of one measurement. `n/a` = the backend cannot observe it (never 0); `error` = the run broke.
STATUSES = ("ok", "fail", "n/a", "error")


def canon(obj: Any) -> str:
    """Canonical JSON: sorted keys, no whitespace. Hashes are taken over this."""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)


def sha(obj: Any, n: int = 16) -> str:
    return hashlib.sha256(canon(obj).encode()).hexdigest()[:n]


@dataclass(frozen=True)
class Knob:
    """One architecture choice. `id` uses archsim's knob vocabulary where archsim has the knob."""
    id: str
    label: str
    kind: str                      # "int" | "float" | "enum" | "bool" | "set"
    default: Any                   # the MVP as built
    domain: tuple = ()             # allowed values for enum/set; (min, max) for int/float
    unit: str = ""
    group: str = ""
    help: str = ""
    stage: int = 1                 # plan stage that first makes it runnable below the analytic level


@dataclass(frozen=True)
class Workload:
    """Something to run: a protocol bench at a speed setting, a concurrent mix, a bridge pair."""
    id: str                        # e.g. "uart_tx", "bridge.uart_rx->i2c_target"
    kind: str                      # "protocol" | "mix" | "bridge" | "chip" (chip-level: area, fmax, power)
    params: dict = field(default_factory=dict)   # e.g. {"speed": 115200}
    label: str = ""

    def key(self) -> str:
        return sha({"id": self.id, "kind": self.kind, "params": self.params})


@dataclass(frozen=True)
class RunSpec:
    """One unit of work for the runner: a config, a workload, a backend."""
    config: dict                   # normalized: every knob present (tools.bench.knobs.normalize)
    workload: Workload
    backend: str                   # backend name, e.g. "analytic", "iss", "iss_b", "rtl", "flow"

    def key(self, code_hash: str = "") -> str:
        """Cache key: same config + workload + backend + backend code => same results."""
        return sha({"c": self.config, "w": asdict(self.workload), "b": self.backend, "code": code_hash}, 24)


@dataclass
class Record:
    """One measured (or derived, or imported) number. The only thing the store holds."""
    workload: str
    metric: str                    # snake_case, stable across levels so cross-checks can join on it
    value: Any                     # number, bool or short string ("served" / "impossible")
    unit: str
    level: str                     # one of LEVELS
    backend: str
    method: str                    # one of METHODS
    status: str = "ok"             # one of STATUSES
    band: Optional[Sequence[float]] = None   # [pessimistic, optimistic] when uncertain
    config: dict = field(default_factory=dict)
    config_hash: str = ""
    workload_params: dict = field(default_factory=dict)
    note: str = ""
    provenance: dict = field(default_factory=dict)  # command, commit, dirty, input_hashes, started, duration_s, source
    run_key: str = ""
    schema: int = SCHEMA

    def to_json(self) -> str:
        return canon(asdict(self))

    @staticmethod
    def from_json(line: str) -> "Record":
        return Record(**json.loads(line))


class Backend(Protocol):
    """What every simulator / importer adapter provides. See CONTRACT.md section 3."""

    name: str
    level: str

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        """None if this backend can run it; otherwise a one-line reason (shown in the UI)."""

    def knob_support(self) -> dict:
        """{knob_id: "full" | "default_only" | "partial: <why>"} for every knob in the registry."""

    def workloads(self) -> Sequence[Workload]:
        """The workloads this backend can run at the MVP config."""

    def run(self, spec: RunSpec) -> list[Record]:
        """Run one spec in this process and return its records. Must not write the store."""

    def code_hash(self) -> str:
        """Hash of the source files whose change invalidates this backend's cached results."""

    # Optional: `def is_long(self, spec: RunSpec) -> bool`. The runner treats a missing method as False
    # and runs a True spec only with --allow-long (CONTRACT.md section 4).

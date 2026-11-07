"""research/perf/b/perf.csv: measured protocol rates from the simulators. Workload = protocol.

Params take the sim backends' shape (tools/bench/workloads.py): `{"speed": <float>}` for a protocol's speed
knob, `{"ticks_per_sck_bit": <int>}` for spi_host_port (its sim param). `target` (iss/rtl), `knob` and the raw
knob value go to the note.

Backend: `import:perf_b:<target>`, so the iss and rtl rows of one key are two sides of a cross-check.
Host-port cost rows live under spi_host_port only. perf.csv repeats them under every protocol
(`host_port_write_cycles`, `host_port_read_cycles`; the same measurement, one copy per protocol rig); those
copies are skipped here and `spi_host_port` carries the numbers (`SKIPPED_METRICS`).
"""
import re
from pathlib import Path

from ._common import load_long_csv

REL = "research/perf/b/perf.csv"
HOST_PORT = "spi_host_port"


def sources(root: Path) -> list[Path]:
    return [root / REL]


def params_for(row: dict):
    knob, value, proto = row["knob"], row["knob_value"], row["protocol"]
    prefix = f"target={row['target']} knob={knob} knob_value={value}"
    if proto == HOST_PORT:
        return {"ticks_per_sck_bit": int(float(value))}, prefix
    return {"speed": float(value)}, prefix


def is_duplicate_host_port_row(row: dict) -> bool:
    return row["protocol"] != HOST_PORT and row["knob"] == "ticks_per_sck_bit"


def load(root: Path):
    recs = _load(root)
    for r in recs:      # iss and rtl rows of the same key are different sides of a cross-check
        m = re.match(r"target=(\w+) ", r.note)
        r.backend = f"import:perf_b:{m.group(1)}" if m else "import:perf_b"
    return recs


def _load(root: Path):
    return load_long_csv(
        root, REL, workload_col="protocol", metric_col="metric", value_col="value",
        unit_col="unit", note_col="source", param_cols=(), method="imported",
        params_fn=params_for, skip_row=is_duplicate_host_port_row,
    )

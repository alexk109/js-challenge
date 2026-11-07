"""Protocol workloads derived from the perf-b rigs (verif/perf/rigs.py).

One Workload per protocol, `params == {"speed": <one ladder value>}`, defaulting to the setting perf-b accepted
(the `knob_value` of that protocol's iss `max_bit_rate` row in research/perf/b/perf.csv; the first ladder rung
when the CSV is absent).  `spi_host_port` is the host-port cost bench (params `ticks_per_sck_bit`).
"""
from __future__ import annotations

import csv
from functools import lru_cache
from pathlib import Path

from .types import Workload

ROOT = Path(__file__).resolve().parents[2]
PERF_CSV = ROOT / "research" / "perf" / "b" / "perf.csv"
HOST_PORT = "spi_host_port"
HOST_PORT_PARAM = "ticks_per_sck_bit"


def _rigs():
    from verif.perf import rigs        # lazy: importing the rigs pulls in the fw tests' dependencies
    return rigs


def protocol_names() -> tuple[str, ...]:
    from verif.perf import measure
    return tuple(measure.ORDER)


def ladder(protocol: str) -> tuple[float, ...]:
    """The speed settings perf-b walks for `protocol`, fastest first (for sweeps over `speed`)."""
    return tuple(_rigs().ladder_for(protocol).speeds)


#: Engines a perf rig uses, where more than one (audited by running every protocol at n_engines=1 on iss_p: only
#: spi fails -- fw/spi/test_spi.py's Rig drives two engines; every other rig and the host-port workload use engine 0).
ENGINES_NEEDED = {"spi": 2}


def engines_needed(workload_id: str) -> int:
    return ENGINES_NEEDED.get(workload_id, 1)


def engines_refusal(workload_id: str, n_engines: int):
    """One-line reason a backend must refuse `workload_id` at `n_engines`, or None."""
    need = engines_needed(workload_id)
    if n_engines < need:
        return f"{workload_id} needs {need} engines; n_engines={n_engines}"
    return None


def rtl_targetable(protocol: str) -> bool:
    return "rtl" in _rigs().ladder_for(protocol).targetable


@lru_cache(maxsize=1)
def accepted_speeds() -> dict[str, float]:
    """protocol -> the speed perf.csv reports its (iss) maximum at."""
    out: dict[str, float] = {}
    if PERF_CSV.exists():
        with open(PERF_CSV, newline="") as fh:
            for row in csv.DictReader(fh):
                if row["target"] == "iss" and row["metric"] == "max_bit_rate":
                    out[row["protocol"]] = float(row["knob_value"])
    return out


HOST_PORT_MAX_TICKS = 64


def default_speed(protocol: str) -> float:
    """perf-b's accepted speed for `protocol`; the first ladder rung only when perf.csv does not exist.
    A perf.csv that lacks the protocol raises: silently using another speed would make results incomparable."""
    acc = accepted_speeds()
    if protocol in acc:
        return acc[protocol]
    if PERF_CSV.exists():
        raise ValueError(f"{PERF_CSV.relative_to(ROOT)} has no iss max_bit_rate row for protocol {protocol!r}; "
                         "give an explicit speed")
    return ladder(protocol)[0]


def speed_refusal(protocol: str, speed) -> str | None:
    """None if `speed` is a number inside the protocol's ladder range, else a one-line reason."""
    lad = ladder(protocol)
    lo, hi = min(lad), max(lad)
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or speed != speed:
        return f"speed={speed!r} is not a number"
    if not lo <= speed <= hi:
        return f"speed={speed:g} is outside {protocol}'s ladder range {lo:g}..{hi:g} {_rigs().ladder_for(protocol).unit}"
    return None


def host_port_ticks() -> int:
    from verif.perf import hostport
    return hostport.TICKS_PER_BIT


def workload(protocol: str, speed: float | None = None) -> Workload:
    if protocol == HOST_PORT:
        t = host_port_ticks() if speed is None else int(speed)
        return Workload(HOST_PORT, "protocol", {HOST_PORT_PARAM: t}, "SPI host port cost per register access")
    rig = _rigs().ladder_for(protocol)
    s = default_speed(protocol) if speed is None else float(speed)
    return Workload(protocol, "protocol", {"speed": s}, f"{protocol} at {rig.knob}={s:g} {rig.unit}")


def all_workloads() -> list[Workload]:
    return [workload(p) for p in protocol_names()] + [workload(HOST_PORT)]


def sweep_workloads(protocol: str) -> list[Workload]:
    """One workload per ladder value of `protocol`."""
    return [workload(protocol, s) for s in ladder(protocol)]

"""Cross-level checks (CONTRACT.md section 6).

Records are joined on `(config_hash, workload, workload_params, metric)`. Each distinct source
(level, backend) that observed the key is compared with every other; a key seen by one source only
is `one-sided`. Verdicts: `match`, `close` (within tolerance), `off` (outside it). An `off` finding
is a result about the models, not a tuning target.

Only `status == "ok"` records with a value take part: n/a and error records mean "nobody observed
it", so they cannot agree or disagree. A value whose only other side is n/a is `one-sided (other side n/a:
<reason>)`, never `off`. When a source has several records for a key, the last wins.

Cross-level alignment:
- the analytic backend names workloads by archsim id (fw_uart_tx); `analytic.ANALYTIC_TO_SIM` (exact matches
  only, never NEAR_MATCHES) renames them to the sim workload id before joining;
- a record with no workload params (analytic is speed-free) joins the sims' record at the workload's reference
  point (its default params) when the metric is in `metrics.SPEED_INDEPENDENT`; any other metric needs equal params;
- each backend is its own side (`import:perf_b:iss` and `import:perf_b:rtl` are two sides);
- a pair where one record says "input copied from <src>" and the other side is that source's import (any side, for
  src `archsim`) is
  `identity`: the model was handed the number, so agreement proves nothing.
"""
from __future__ import annotations

import json
import re
from typing import Iterable, Optional

from .metrics import SPEED_INDEPENDENT
from .store import record_id
from .types import LEVELS, Record, canon

IDENTITY_RE = re.compile(r"input copied from ([\w-]+)")

#: Default for rates and everything not in TOLERANCES.
DEFAULT_REL = 0.02
#: Per-metric tolerance: ("exact",) | ("rel", fraction) | ("abs", amount). Overrides the default below.
TOLERANCES: dict[str, tuple] = {
    "margin": ("rel", 0.05),
}

#: Units that count discrete things: compared exactly unless TOLERANCES says otherwise.
EXACT_UNIT_WORDS = ("cycle", "tick", "count", "word")


def tolerance(metric: str, unit: str = "") -> tuple:
    if metric in TOLERANCES:
        return TOLERANCES[metric]
    if unit == "" or any(w in unit for w in EXACT_UNIT_WORDS):
        return ("exact",)
    return ("rel", DEFAULT_REL)


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _bounds(r: Record) -> tuple[float, float]:
    vals = list(r.band) if r.band else [r.value]
    return min(vals), max(vals)


def compare(a: Record, b: Record, tol: Optional[tuple] = None) -> tuple[str, str]:
    """(verdict, detail) for two records of the same key."""
    tol = tol or tolerance(a.metric, a.unit)
    if not (_num(a.value) and _num(b.value)):
        return ("match", "equal") if a.value == b.value else ("off", f"{a.value!r} != {b.value!r}")
    if a.band or b.band:
        (alo, ahi), (blo, bhi) = _bounds(a), _bounds(b)
        if ahi >= blo and bhi >= alo:
            return ("match", "equal") if a.value == b.value else ("close", "bands overlap")
        gap = max(blo - ahi, alo - bhi)
        scale = max(abs(alo), abs(ahi), abs(blo), abs(bhi))
        if tol[0] == "rel" and gap <= tol[1] * scale:
            return "close", f"bands [{alo:g}, {ahi:g}] and [{blo:g}, {bhi:g}] are {gap / scale:.3%} apart (tolerance {tol[1]:.3%})"
        if tol[0] == "abs" and gap <= tol[1]:
            return "close", f"bands [{alo:g}, {ahi:g}] and [{blo:g}, {bhi:g}] are {gap:g} apart (tolerance {tol[1]:g})"
        return "off", f"bands [{alo:g}, {ahi:g}] and [{blo:g}, {bhi:g}] do not overlap"
    if a.value == b.value:
        return "match", "equal"
    diff = abs(a.value - b.value)
    if tol[0] == "exact":
        return "off", f"{a.value:g} vs {b.value:g} (exact match required)"
    if tol[0] == "abs":
        ok, what = diff <= tol[1], f"abs diff {diff:g} vs {tol[1]:g}"
    else:
        scale = max(abs(a.value), abs(b.value))
        ok, what = diff <= tol[1] * scale, f"rel diff {diff / scale:.3%} vs {tol[1]:.3%}"
    return ("close" if ok else "off"), what


def _src(r: Record) -> dict:
    return {"level": r.level, "backend": r.backend, "value": r.value, "band": r.band, "unit": r.unit,
            "method": r.method, "id": record_id(r)}


#: analytic chip-level workload -> the flow config that is the same design (metrics/flow.csv cfg `mvp`), for these metrics
CHIP_TO_FLOW = {"chip": "mvp"}
CHIP_TO_FLOW_METRICS = ("area_um2",)


def _sim_workload(r: Record) -> str:
    """The workload id a record joins under: analytic archsim ids are renamed to the sim's (exact matches only);
    the analytic `chip` area joins the flow's `mvp` run."""
    if r.level == "analytic":
        from .backends.analytic import ANALYTIC_TO_SIM
        if r.workload in CHIP_TO_FLOW and r.metric in CHIP_TO_FLOW_METRICS:
            return CHIP_TO_FLOW[r.workload]
        return ANALYTIC_TO_SIM.get(r.workload, r.workload)
    return r.workload


def _is_reference_point(workload: str, params: dict) -> bool:
    """True if `params` is the workload's default operating point (the speed perf-b accepts, the host port's ticks)."""
    from . import workloads
    try:
        return canon(params) == canon(workloads.workload(workload).params)
    except Exception:       # unknown workload, or perf.csv lacks the protocol: nothing is a reference point
        return False


def _identity(a: Record, b: Record) -> str:
    for x, y in ((a, b), (b, a)):
        m = IDENTITY_RE.search(x.note or "")
        if m and m.group(1) == "archsim":      # an archsim input handed through: no other side can check the model with it
            return f"{x.level}/{x.backend} carries an archsim input unchanged ({x.note.split(';')[-1].strip()[:90]}); not a model prediction"
        if m and m.group(1).replace("-", "_") in y.backend:
            return f"{x.level}/{x.backend} was handed this number by {m.group(1)}; {y.backend} is that source"
    return ""


def join_keys(records: Iterable[Record]) -> list[tuple]:
    """The join key `(config_hash, workload, canonical params, metric)` of each record, in order.

    Workload ids are aligned (analytic -> sim); a speed-free record of a speed-independent metric takes the params of the
    reference-point records of the same workload when some exist, so it lands in their group."""
    records = list(records)
    keys = [(r.config_hash, _sim_workload(r), canon(r.workload_params), r.metric) for r in records]
    homes: dict[tuple, str] = {}
    for r, k in zip(records, keys):
        if r.status == "ok" and r.value is not None and k[3] in SPEED_INDEPENDENT and k[2] != "{}" \
                and _is_reference_point(k[1], r.workload_params):
            homes[(k[0], k[1], k[3])] = k[2]
    return [(k[0], k[1], homes.get((k[0], k[1], k[3]), k[2]), k[3]) if (k[3] in SPEED_INDEPENDENT and k[2] == "{}") else k
            for k in keys]


def findings(records: Iterable[Record]) -> list[dict]:
    """All comparisons over the records, as dicts:
    {config_hash, workload, workload_params, metric, verdict, detail, a, b} (b is None when one-sided).
    Verdicts: match, close, off, identity, one-sided."""
    keys = join_keys(records)
    groups: dict[tuple, dict[tuple, Record]] = {}
    nas: dict[tuple, dict[tuple, Record]] = {}
    for r, key in zip(records, keys):
        if r.status == "n/a":
            nas.setdefault(key, {})[(r.level, r.backend)] = r
        if r.status != "ok" or r.value is None:
            continue
        groups.setdefault(key, {})[(r.level, r.backend)] = r
    for key in list(nas):       # an n/a source whose key has no value anywhere is not a finding
        if key not in groups:
            del nas[key]
    order = {lv: i for i, lv in enumerate(LEVELS)}
    out = []
    for key, srcs in groups.items():
        ch, w, wp, m = key
        base = {"config_hash": ch, "workload": w, "workload_params": json.loads(wp), "metric": m}
        ranked = sorted(srcs.values(), key=lambda r: (order.get(r.level, 99), r.backend))
        if len(ranked) == 1:
            other = [n for s_, n in nas.get(key, {}).items() if s_ not in srcs]
            if other:
                n = other[0]
                detail = f"one-sided (other side n/a: {n.level}/{n.backend}: {(n.note or 'no reason given').split(';')[0][:120]})"
            else:
                detail = f"only {ranked[0].level}/{ranked[0].backend} has it"
            out.append({**base, "verdict": "one-sided", "detail": detail, "a": _src(ranked[0]), "b": None})
            continue
        for i, a in enumerate(ranked):
            for b in ranked[i + 1:]:
                ident = _identity(a, b)
                v, d = ("identity", ident) if ident else compare(a, b)
                out.append({**base, "verdict": v, "detail": d, "a": _src(a), "b": _src(b)})
    return out


def summary(found: list[dict]) -> dict:
    s = {"match": 0, "close": 0, "off": 0, "identity": 0, "one-sided": 0}
    for f in found:
        s[f["verdict"]] += 1
    return s

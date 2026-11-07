"""The ONE place the front end touches the rest of the bench (store, runner, crosscheck, backends).

Everything else in tools/bench/ui calls these functions. If the signatures in store/runner differ from
CONTRACT, fix them here. Expected:

  tools.bench.store.query(path=None, **filters) -> list[Record]
  tools.bench.runner.run_specs(specs, jobs=4, allow_long=False, use_cache=True, on_event=None) -> list[Record]
  tools.bench.runner.expand_sweep(base: dict, vary: dict[str, list], workloads, backends) -> (specs, refusals)
  tools.bench.crosscheck.findings(records) -> list[dict]   (each dict: config_hash, workload, metric, verdict, ...)
  tools.bench.backends.REGISTRY / load(name)               (load returns a backend object, or a class to call)

Fallbacks (used only when the module/function is missing, so the UI works before task A lands):
store -> read the JSONL directly; crosscheck -> a small local join; backends -> empty.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import os
from typing import Callable, Optional

from .. import knobs as _knobs
from ..types import LEVELS, Record

DEFAULT_STORE = os.path.join(os.path.dirname(os.path.dirname(__file__)), "results", "records.jsonl")


# ---------------------------------------------------------------- records
def record_id(rec: Record) -> str:
    try:
        from .. import store
        return store.record_id(rec)
    except (ImportError, AttributeError):
        return hashlib.sha256(rec.to_json().encode()).hexdigest()[:12]


def to_dict(rec: Record) -> dict:
    d = dataclasses.asdict(rec)
    d["id"] = record_id(rec)
    d["config_label"] = (("external: " + rec.config_hash[len("external:"):]) if rec.config_hash.startswith("external:")
                         else _knobs.label(rec.config) if rec.config else "MVP")
    d["config_diff"] = _knobs.diff(rec.config) if rec.config else {}
    return d


def payload(records: list[Record]) -> list[dict]:
    """Record dicts for the front end, each with `row_workload` / `row_params`: where the cross-check joins it
    (analytic ids aligned to the sims', speed-free records at the reference point), so Compare shows one row per join."""
    try:
        from .. import crosscheck
        keys = crosscheck.join_keys(records)
    except (ImportError, AttributeError):
        keys = [None] * len(records)
    out = []
    for r, k in zip(records, keys):
        d = to_dict(r)
        if k:
            d["row_workload"], d["row_params"] = k[1], json.loads(k[2])
        out.append(d)
    return out


def _read_jsonl(path: str) -> list[Record]:
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(Record.from_json(line))
    return out


def load_records(path: Optional[str] = None, **filters) -> list[Record]:
    filters = {k: v for k, v in filters.items() if v not in (None, "")}
    try:
        from .. import store
        return list(store.query(path, **filters))
    except (ImportError, AttributeError):
        pass
    recs = _read_jsonl(path or DEFAULT_STORE)
    return [r for r in recs if all(getattr(r, k, None) == v for k, v in filters.items())]


# ---------------------------------------------------------------- cross-checks
def _group_key(r: Record) -> tuple:
    return (r.config_hash, r.workload, json.dumps(r.workload_params, sort_keys=True), r.metric)


def _local_findings(records: list[Record]) -> list[dict]:
    groups: dict = {}
    for r in records:
        if r.status == "ok":
            groups.setdefault(_group_key(r), {}).setdefault(r.level, r)
    out = []
    for by_level in groups.values():
        any_r = next(iter(by_level.values()))
        vals = {lv: r.value for lv, r in by_level.items()}
        detail = ""
        if len(by_level) < 2:
            verdict, detail = "one-sided", "only level %s has it" % next(iter(by_level))
        else:
            verdict = "match"
            nums = [r for r in by_level.values()
                    if isinstance(r.value, (int, float)) and not isinstance(r.value, bool)]
            if len(nums) == len(by_level):
                if any(r.band for r in nums):
                    lo = [min(r.band) if r.band else r.value for r in nums]
                    hi = [max(r.band) if r.band else r.value for r in nums]
                    verdict = "match" if max(lo) <= min(hi) else "off"
                else:
                    v = [r.value for r in nums]
                    if len(set(v)) > 1:
                        rel = (max(v) - min(v)) / max(abs(max(v)), 1e-12)
                        verdict = "close" if rel <= 0.02 else "off"
            elif len({json.dumps(v) for v in vals.values()}) > 1:
                verdict = "off"
        out.append({"config_hash": any_r.config_hash, "workload": any_r.workload,
                    "workload_params": any_r.workload_params, "metric": any_r.metric,
                    "verdict": verdict, "values": vals, "detail": detail})
    return out


def check(records: list[Record]) -> list[dict]:
    try:
        from .. import crosscheck
        return [dict(f) for f in crosscheck.findings(records)]
    except (ImportError, AttributeError):
        return _local_findings(records)


# ---------------------------------------------------------------- backends, knobs, workloads
def _registry() -> dict:
    try:
        from .. import backends
        return dict(backends.REGISTRY)
    except (ImportError, AttributeError):
        return {}


def backend(name: str):
    from .. import backends
    b = backends.load(name)
    return b() if isinstance(b, type) else b


def backend_names() -> list[str]:
    return sorted(_registry())


def knobs_payload() -> dict:
    support: dict = {}
    for n in backend_names():
        try:
            support[n] = backend(n).knob_support()
        except Exception as e:  # a missing tool only breaks its own backend
            support[n] = {"_error": str(e)}
    return {"knobs": [dataclasses.asdict(k) for k in _knobs.KNOBS], "backends": backend_names(),
            "support": support, "levels": list(LEVELS)}


def workloads_payload(records: list[Record]) -> list[dict]:
    seen: dict = {}
    for n in backend_names():
        try:
            for w in backend(n).workloads():
                e = seen.setdefault(w.id, {"id": w.id, "kind": w.kind, "label": w.label,
                                           "params": w.params, "backends": []})
                e["backends"].append(n)
        except Exception:
            continue
    for r in records:  # also what the store has seen
        e = seen.setdefault(r.workload, {"id": r.workload, "kind": "", "label": "", "params": {}, "backends": []})
        if r.backend not in e["backends"]:
            e["backends"].append(r.backend)
    return sorted(seen.values(), key=lambda e: e["id"])


# ---------------------------------------------------------------- running
def store_file(path: Optional[str] = None) -> str:
    try:
        from .. import store
        return str(store.store_path(path))
    except (ImportError, AttributeError):
        return path or DEFAULT_STORE


def _name_error(req: dict) -> Optional[str]:
    names = backend_names()
    if names:
        bad = [b for b in req["backends"] if b not in names]
        if bad:
            return "unknown backend(s): %s; known: %s" % (", ".join(bad), ", ".join(names))
        known = {w.id for n in req["backends"] for w in backend(n).workloads()}
        badw = [w for w in req["workloads"] if w.split(":")[0] not in known]
        if badw:
            return "unknown workload(s) for the chosen backends: %s" % ", ".join(badw)
    return None


def validate(req) -> Optional[str]:
    """None if the POST /api/run body is acceptable, else a one-line reason (HTTP 400)."""
    if not isinstance(req, dict):
        return "body must be a JSON object"
    for k in ("workloads", "backends"):
        v = req.get(k)
        if not isinstance(v, list) or not v or not all(isinstance(x, str) and x for x in v):
            return "%s must be a non-empty list of names" % k
    cfgs = req.get("configs")
    if cfgs is not None and (not isinstance(cfgs, list) or not all(isinstance(c, dict) for c in cfgs)):
        return "configs must be a list of {knob: value} objects"
    sw = req.get("sweep")
    if sw is not None:
        if not isinstance(sw, dict) or not isinstance(sw.get("base", {}), dict) or not isinstance(sw.get("vary", {}), dict):
            return "sweep must be {base: {...}, vary: {knob: [values]}}"
        if not all(isinstance(v, list) and v for v in sw.get("vary", {}).values()):
            return "every vary entry must be a non-empty list"
    for k in ("allow_long", "no_cache"):
        if not isinstance(req.get(k, False), bool):
            return "%s must be true or false" % k
    try:
        for c in (cfgs or []) + ([sw.get("base", {})] if sw else []):
            _knobs.normalize(c)
        for kid, vals in ((sw or {}).get("vary") or {}).items():
            for v in vals:
                _knobs.normalize({kid: v})
        return _name_error(req)
    except _knobs.KnobError as e:
        return str(e)
    except Exception as e:
        return "%s: %s" % (type(e).__name__, e)


def start_run(req: dict, on_event: Optional[Callable] = None, store_path: Optional[str] = None) -> dict:
    """Run a POST /api/run body (blocking). Every run goes through runner.expand_sweep (a single run has vary={}).

    Emits {"event": "total", "n": N} once, then forwards the runner's events. Returns a summary dict."""
    from .. import runner
    wls, backs = list(req["workloads"]), list(req["backends"])
    specs: list = []
    refused: list = []
    if req.get("sweep"):
        sw = req["sweep"]
        sp, rf = runner.expand_sweep(sw.get("base") or {}, sw.get("vary") or {}, wls, backs)
        specs += sp
        refused += rf
    else:
        for cfg in req.get("configs") or [{}]:
            sp, rf = runner.expand_sweep(cfg, {}, wls, backs)
            specs += sp
            refused += rf
    skipped: list = []

    def fwd(ev):
        if ev.get("event") == "skipped":
            skipped.append({"backend": ev.get("backend"), "workload": ev.get("workload"), "reason": ev.get("reason", "")})
        if on_event:
            on_event(ev)

    if on_event:
        on_event({"event": "total", "n": len(specs)})
    recs = runner.run_specs(specs, jobs=int(req.get("jobs") or 4), allow_long=bool(req.get("allow_long")),
                            use_cache=not req.get("no_cache"), on_event=fwd, store_path=store_path) if specs else []
    counts: dict = {}
    for r in recs:
        counts[r.status] = counts.get(r.status, 0) + 1
    return {"specs": len(specs), "records": len(recs), "status_counts": counts,
            "refused": [r if isinstance(r, (str, dict)) else {"reason": str(r)} for r in refused], "skipped": skipped}


def count_specs(req: dict) -> int:
    """Best-effort total for the progress bar until the runner reports the real one."""
    if req.get("sweep"):
        n = 1
        for v in (req["sweep"].get("vary") or {}).values():
            n *= max(len(v), 1)
    else:
        n = max(len(req.get("configs") or [{}]), 1)
    return n * max(len(req.get("workloads") or []), 1) * max(len(req.get("backends") or []), 1)

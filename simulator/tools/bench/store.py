"""The record store and run cache (CONTRACT.md section 4).

One append-only JSONL file holds every record; the cache keeps one small JSONL per run key. Both live
next to each other, so pointing `BENCH_STORE` at a temp file moves the whole pair (tests rely on it).
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Iterable, Optional

from .types import Record, sha

ROOT = Path(__file__).resolve().parent
DEFAULT_STORE = ROOT / "results" / "records.jsonl"
SNAPSHOTS = ROOT / "snapshots"


def store_path(path: Optional[str | Path] = None) -> Path:
    """Explicit argument, else env BENCH_STORE, else tools/bench/results/records.jsonl."""
    if path:
        return Path(path)
    env = os.environ.get("BENCH_STORE")
    return Path(env) if env else DEFAULT_STORE


def cache_dir(path: Optional[str | Path] = None) -> Path:
    return store_path(path).parent / "cache"


def record_id(rec: Record) -> str:
    """Id used by `bench trace`: sha of the record's JSON, 12 hex chars."""
    return sha(json.loads(rec.to_json()), 12)


def _read(path: Path) -> list[Record]:
    if not path.exists():
        return []
    out = []
    with path.open() as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                out.append(Record.from_json(line))
            except (ValueError, TypeError) as e:  # typically a partial last line after a crash
                print(f"bench: skipping undecodable line {n} of {path}: {type(e).__name__}", file=sys.stderr)
    return out


def load(path: Optional[str | Path] = None) -> list[Record]:
    return _read(store_path(path))


def append(records: Iterable[Record], path: Optional[str | Path] = None, skip_existing: bool = False) -> int:
    """Append records; with skip_existing, records already present (same id) are not written twice."""
    p = store_path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    recs = list(records)
    if skip_existing:
        have = {record_id(r) for r in _read(p)}
        recs = [r for r in recs if record_id(r) not in have]
    if recs:
        with p.open("a") as f:
            f.write("".join(r.to_json() + "\n" for r in recs))
    return len(recs)


def query(path: Optional[str | Path] = None, *, records: Optional[list[Record]] = None, **filters) -> list[Record]:
    """Filter records. Keys: workload, metric, backend, level, method, status, config_hash, run_key
    (each a value or a list/tuple/set of accepted values), `id_prefix`, and `workload_params` (dict,
    exact). Unknown keys raise TypeError so a typo does not silently return everything."""
    allowed = {"workload", "metric", "backend", "level", "method", "status", "config_hash", "run_key"}
    id_prefix = filters.pop("id_prefix", None)
    wparams = filters.pop("workload_params", None)
    bad = set(filters) - allowed
    if bad:
        raise TypeError(f"unknown filter(s): {', '.join(sorted(bad))}")
    recs = records if records is not None else load(path)
    out = []
    for r in recs:
        ok = True
        for k, want in filters.items():
            if want is None:
                continue
            got = getattr(r, k)
            if isinstance(want, (list, tuple, set, frozenset)):
                ok = got in want
            else:
                ok = got == want
            if not ok:
                break
        if ok and wparams is not None and r.workload_params != wparams:
            ok = False
        if ok and id_prefix and not record_id(r).startswith(id_prefix):
            ok = False
        if ok:
            out.append(r)
    return out


def get_cached(run_key: str, path: Optional[str | Path] = None) -> Optional[list[Record]]:
    """Cached records, or None (a miss) when absent or damaged: a corrupt cache file means re-run."""
    f = cache_dir(path) / f"{run_key}.jsonl"
    if not f.exists():
        return None
    try:
        lines = [l for l in f.read_text().splitlines() if l.strip()]
        recs = [Record.from_json(l) for l in lines]
    except (ValueError, TypeError, OSError) as e:
        print(f"bench: ignoring corrupt cache file {f.name}: {type(e).__name__}", file=sys.stderr)
        return None
    return recs or None


def put_cached(run_key: str, records: list[Record], path: Optional[str | Path] = None) -> None:
    d = cache_dir(path)
    d.mkdir(parents=True, exist_ok=True)
    tmp = d / f"{run_key}.jsonl.tmp{os.getpid()}"
    tmp.write_text("".join(r.to_json() + "\n" for r in records))
    tmp.replace(d / f"{run_key}.jsonl")


def snapshot(name: str, records: list[Record], directory: Optional[str | Path] = None) -> Path:
    """Write records to tools/bench/snapshots/NAME.jsonl (committed). BENCH_SNAPSHOTS overrides the dir."""
    d = Path(directory or os.environ.get("BENCH_SNAPSHOTS") or SNAPSHOTS)
    d.mkdir(parents=True, exist_ok=True)
    out = d / f"{name}.jsonl"
    out.write_text("".join(r.to_json() + "\n" for r in records))
    return out


def as_dicts(records: Iterable[Record]) -> list[dict]:
    """JSON-ready rows with the record id added (what the API and --json output)."""
    return [dict(asdict(r), id=record_id(r)) for r in records]

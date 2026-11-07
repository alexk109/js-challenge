"""Runner: expand specs, apply the cache and the long-run gate, run workers in parallel (CONTRACT.md 4).

Python API used by the CLI and the front end:
    expand_sweep(base, vary, workloads, backends) -> (specs, refused)
    run_specs(specs, jobs, allow_long, use_cache, on_event=None, ...) -> records
Every run goes through a `python -m tools.bench.worker` subprocess (see worker.py for why).
"""
from __future__ import annotations

import inspect
import itertools
import os
import signal
import json
import shlex
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path
from typing import Callable, Optional, Sequence

from . import knobs, store
from .backends import load as load_backend
from .metrics import METRICS
from .types import Record, RunSpec, Workload
from .worker import spec_to_json

REPO = Path(__file__).resolve().parents[2]

#: Levels that run only with --allow-long (CONTRACT.md section 4).
LONG_LEVELS = ("flow", "gl", "power")

#: Seconds before a worker is killed, per level. Override per call with `timeouts=`.
TIMEOUTS = {"analytic": 60, "iss": 300, "rtl": 900, "synth": 900, "gl": 3600, "flow": 6 * 3600, "power": 3600, "import": 300}
#: flow: a full LibreLane run with the post-GRT repair config took ~3.5 h here (research/power, 2026-10-29).
DEFAULT_TIMEOUT = 600

TAIL = 800  # characters of stderr kept in an error record


# ---------------------------------------------------------------- parsing helpers

def parse_value(text: str):
    """'true' -> True, '8' -> 8, '1.5' -> 1.5, anything else stays a string."""
    try:
        return json.loads(text)
    except ValueError:
        return text


def parse_workload(text: str) -> tuple[str, dict]:
    """'uart_tx' or 'uart_tx:speed=115200:delay=0.5' -> (id, param overrides)."""
    wid, *pairs = text.split(":")
    params = {}
    for p in pairs:
        k, _, v = p.partition("=")
        params[k] = parse_value(v)
    return wid, params


def workload_text(w: Workload) -> str:
    return w.id + "".join(f":{k}={json.dumps(v) if not isinstance(v, str) else v}" for k, v in sorted(w.params.items()))


def _python_cmd() -> str:
    """The interpreter that ran us, relative to the repo root when it lives inside it (".venv/bin/python")."""
    exe = Path(sys.executable)
    try:
        return str(exe.relative_to(REPO))
    except ValueError:
        return str(exe)


def repro_command(spec: RunSpec) -> str:
    """A shell command that re-runs exactly this spec, bypassing the cache."""
    parts = [_python_cmd(), "-m", "tools.bench", "run"]
    for k, v in sorted(knobs.diff(spec.config).items()):
        parts += ["--set", f"{k}={json.dumps(v)}"]
    parts += ["--workload", workload_text(spec.workload), "--backend", spec.backend, "--no-cache"]
    return " ".join(shlex.quote(p) for p in parts)


# ---------------------------------------------------------------- expansion

def _offered(bname: str, listed: dict, cap: int = 10) -> str:
    ids = sorted(listed)
    if len(ids) <= cap:
        return f"workload not offered by {bname}; it offers: {', '.join(ids) or 'none'}"
    return (f"workload not offered by {bname}; it offers {', '.join(ids[:cap])} and {len(ids) - cap} more "
            f"(see `bench workloads --backend {bname}`)")


def _refusal(backend: str, workload: str, config: dict, reason: str) -> dict:
    return {"backend": backend, "workload": workload, "config": knobs.diff(config), "reason": reason}


def expand_sweep(base: dict, vary: dict, workloads: Sequence[str], backends: Sequence[str]) -> tuple[list[RunSpec], list[dict]]:
    """Cartesian product of `vary` ({knob: [values]}) over `base` knob values, x workloads x backends.

    `workloads` are CLI strings ('id' or 'id:param=v'); each is resolved against the backend's own
    workload list. Returns (specs, refused): a spec the backend cannot honour (supports() says why),
    an unknown workload, or a backend that fails to load is listed in `refused` with the reason and
    never silently replaced by a different run. A single run is a sweep with empty `vary`.
    """
    ids = sorted(vary)
    configs = []
    for combo in itertools.product(*(vary[k] for k in ids)):
        configs.append(knobs.normalize({**base, **dict(zip(ids, combo))}))
    specs: list[RunSpec] = []
    refused: list[dict] = []
    for bname in backends:
        try:
            backend = load_backend(bname)
            listed = {w.id: w for w in backend.workloads()}
        except Exception as e:  # a missing tool only breaks its own backend
            for wtext in workloads:
                for cfg in configs:
                    refused.append(_refusal(bname, wtext, cfg, f"backend failed to load: {type(e).__name__}: {e}"))
            continue
        for wtext in workloads:
            wid, over = parse_workload(wtext)
            for cfg in configs:
                if wid not in listed:
                    refused.append(_refusal(bname, wtext, cfg, _offered(bname, listed)))
                    continue
                w0 = listed[wid]
                w = Workload(w0.id, w0.kind, {**w0.params, **over}, w0.label)
                spec = RunSpec(cfg, w, bname)
                why = backend.supports(cfg, w)
                if why:
                    refused.append(_refusal(bname, wtext, cfg, why))
                else:
                    specs.append(spec)
    return specs, refused


# ---------------------------------------------------------------- git provenance

@lru_cache(maxsize=1)
def git_state() -> tuple[str, bool]:
    def git(*a):
        return subprocess.run(["git", *a], cwd=REPO, capture_output=True, text=True, timeout=30).stdout
    try:
        commit = git("rev-parse", "HEAD").strip() or "unknown"
        dirty = bool(git("status", "--porcelain", "--untracked-files=no").strip())
    except Exception:
        return "unknown", True
    return commit, dirty


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ---------------------------------------------------------------- one run

def code_hash_for(backend, spec: RunSpec) -> str:
    """The backend's code hash for this spec: backends whose code_hash takes a config may hash less or more of it."""
    try:
        takes_config = len(inspect.signature(backend.code_hash).parameters) >= 1
    except (TypeError, ValueError):
        takes_config = False
    h = backend.code_hash(spec.config) if takes_config else backend.code_hash()
    inp = getattr(backend, "input_hash", None)       # optional: hash of external inputs (e.g. flow artifacts)
    return h + ":" + inp(spec) if inp else h


def is_long(spec: RunSpec, backend) -> bool:
    """Long = backend level in LONG_LEVELS, or the backend's optional `is_long(spec)` says so."""
    if getattr(backend, "level", "") in LONG_LEVELS:
        return True
    hook = getattr(backend, "is_long", None)
    return bool(hook(spec)) if hook else False


def _error_record(spec: RunSpec, note: str, level: str, run_key: str, prov: dict) -> Record:
    return Record(workload=spec.workload.id, metric="run_error", value=None, unit="", level=level,
                  backend=spec.backend, method="measured", status="error", note=note,
                  config=spec.config, config_hash=knobs.config_hash(spec.config),
                  workload_params=spec.workload.params, run_key=run_key, provenance=prov)


def _finalize(rec: Record, spec: RunSpec, run_key: str, prov: dict) -> Record:
    """Fill what the backend leaves to the runner; flag names outside the vocabulary."""
    if not rec.config and not rec.config_hash.startswith("external:"):
        rec.config = spec.config      # an external record (imported probe, not a bench config) keeps config {}
    rec.config_hash = rec.config_hash or knobs.config_hash(rec.config)
    if not rec.workload_params:
        rec.workload_params = spec.workload.params
    rec.run_key = run_key
    rec.provenance = {**prov, **rec.provenance}
    if rec.metric not in METRICS and rec.status != "error":
        rec.note = f"metric {rec.metric!r} is not in tools/bench/metrics.py" + (f"; {rec.note}" if rec.note else "")
        rec.status = "error"
        rec.value = None
    return rec


def _worker_env() -> dict:
    """Our interpreter's bin dir first on PATH: cocotb builds the RTL simulator against whichever `python3` PATH
    finds, and /usr/bin/python3 makes it die at start ("No module named pygpi")."""
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(Path(sys.prefix) / "bin"), env.get("PATH", "")])
    return env


def _execute(spec: RunSpec, level: str, run_key: str, timeout: float) -> list[Record]:
    started = _now()
    t0 = time.time()
    commit, dirty = git_state()
    prov = {"command": repro_command(spec), "commit": commit, "dirty": dirty, "started": started}
    try:
        # own session so a timeout can kill grandchildren (vvp, verilator, ...) with the worker
        p = subprocess.Popen([sys.executable, "-m", "tools.bench.worker"], cwd=REPO, text=True,
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             start_new_session=True, env=_worker_env())
        try:
            out, err = p.communicate(spec_to_json(spec), timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except OSError:
                p.kill()
            out, err = p.communicate()
            prov["duration_s"] = round(time.time() - t0, 3)
            return [_error_record(spec, f"timed out after {timeout:g} s", level, run_key, prov)]
    except OSError as e:
        prov["duration_s"] = round(time.time() - t0, 3)
        return [_error_record(spec, f"could not start worker: {e}", level, run_key, prov)]
    prov["duration_s"] = round(time.time() - t0, 3)
    if p.returncode != 0:
        return [_error_record(spec, f"worker exit {p.returncode}: {err.strip()[-TAIL:]}", level, run_key, prov)]
    try:
        recs = [Record.from_json(line) for line in out.splitlines() if line.strip()]
    except Exception as e:
        return [_error_record(spec, f"unreadable worker output ({type(e).__name__}: {e}): {out[-TAIL:]}", level, run_key, prov)]
    if not recs:
        return [_error_record(spec, "backend returned no records", level, run_key, prov)]
    return [_finalize(r, spec, run_key, prov) for r in recs]


def run_specs(specs: Sequence[RunSpec], jobs: int = 4, allow_long: bool = False, use_cache: bool = True,
              on_event: Optional[Callable[[dict], None]] = None, timeouts: Optional[dict] = None,
              store_path=None, save: bool = True) -> list[Record]:
    """Run specs (cache first, long gate, `jobs` workers in parallel) and return their records in spec order.

    Records are appended to the store when `save` (cache hits are appended only if the store lacks
    them). Runs with any `error` record are not cached. `on_event` receives dicts
    {"event": "cache_hit"|"start"|"done"|"skipped"|"refused", "backend", "workload", "key", ...} from
    worker threads (calls are serialized; an exception in the callback is logged and ignored). Skipped
    (long gate) specs produce no records. Each run's records are appended to the store as soon as the
    run finishes, so an interrupt loses only runs still in flight.
    """
    tmo = {**TIMEOUTS, **(timeouts or {})}
    lock = threading.Lock()

    store_lock = threading.Lock()

    def emit(ev: dict) -> None:
        if on_event:
            with lock:
                try:
                    on_event(ev)
                except Exception as e:  # a broken progress callback must not lose results
                    print(f"bench: on_event raised {type(e).__name__}: {e}", file=sys.stderr)

    def save_now(recs: list[Record], skip_existing: bool = False) -> None:
        if save and recs:
            with store_lock:
                store.append(recs, store_path, skip_existing=skip_existing)

    results: dict[int, list[Record]] = {}
    todo: list[tuple[int, RunSpec, str, str]] = []
    for i, spec in enumerate(specs):
        base = {"backend": spec.backend, "workload": spec.workload.id, "params": spec.workload.params}
        try:
            backend = load_backend(spec.backend)
            key = spec.key(code_hash_for(backend, spec))
            level = backend.level
            long_ = is_long(spec, backend)
        except Exception as e:
            key = spec.key("")
            rec = _error_record(spec, f"backend failed to load: {type(e).__name__}: {e}", "import", key,
                                {"command": repro_command(spec), "commit": git_state()[0], "dirty": git_state()[1],
                                 "started": _now(), "duration_s": 0.0})
            results[i] = [rec]
            save_now([rec])
            emit({**base, "event": "done", "key": key, "n_records": 1, "error": True})
            continue
        if long_ and not allow_long:
            emit({**base, "event": "skipped", "key": key, "reason": f"long run ({level}); pass --allow-long"})
            results[i] = []
            continue
        if use_cache:
            hit = store.get_cached(key, store_path)
            if hit is not None:
                results[i] = hit
                save_now(hit, skip_existing=True)
                emit({**base, "event": "cache_hit", "key": key, "n_records": len(hit)})
                continue
        todo.append((i, spec, level, key))

    def work(item):
        i, spec, level, key = item
        emit({"event": "start", "backend": spec.backend, "workload": spec.workload.id, "params": spec.workload.params, "key": key})
        recs = _execute(spec, level, key, tmo.get(level, DEFAULT_TIMEOUT))
        failed = any(r.status == "error" for r in recs)
        if not failed:
            store.put_cached(key, recs, store_path)
        save_now(recs)
        emit({"event": "done", "backend": spec.backend, "workload": spec.workload.id, "params": spec.workload.params,
              "key": key, "n_records": len(recs), "error": failed})
        return i, recs

    if todo:
        with ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
            for i, recs in ex.map(work, todo):
                results[i] = recs
    return [r for i in range(len(specs)) for r in results.get(i, [])]

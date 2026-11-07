"""Command line: `python -m tools.bench <cmd>` (CONTRACT.md section 5). Every command takes --json."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Sequence

from . import crosscheck, knobs, runner, store
from .backends import REGISTRY, load as load_backend
from .types import Record, Workload

CONFIGS = Path(__file__).resolve().parent / "configs"


# ---------------------------------------------------------------- output helpers

def table(rows: Sequence[Sequence], header: Sequence[str]) -> str:
    cells = [[("" if c is None else str(c)) for c in r] for r in rows]
    widths = [max(len(h), *(len(r[i]) for r in cells)) if cells else len(h) for i, h in enumerate(header)]
    fmt = lambda r: "  ".join(c.ljust(w) for c, w in zip(r, widths)).rstrip()
    return "\n".join([fmt(header), fmt(["-" * w for w in widths]), *(fmt(r) for r in cells)])


def fmt_value(r: Record) -> str:
    v = r.value
    if r.status in ("n/a", "error"):
        return r.status
    s = f"{v:g}" if isinstance(v, float) else str(v)
    if r.band:
        s += f" [{r.band[0]:g}..{r.band[1]:g}]"
    return s + ("" if r.status == "ok" else f" ({r.status})")


def emit_json(obj) -> None:
    print(json.dumps(obj, indent=2, default=str))


def record_rows(recs: Sequence[Record]) -> str:
    rows = [(store.record_id(r), r.workload + _wp(r.workload_params), r.metric, fmt_value(r), r.unit, r.level,
             r.backend, r.method, knobs.label(r.config) if r.config else "") for r in recs]
    return table(rows, ("id", "workload", "metric", "value", "unit", "level", "backend", "method", "config"))


def _wp(params: dict) -> str:
    return "".join(f":{k}={v}" for k, v in sorted(params.items())) if params else ""


# ---------------------------------------------------------------- argument handling

def _split(text: Optional[str]) -> list[str]:
    return [t for t in (text or "").split(",") if t]


def _backends(text: Optional[str]) -> list[str]:
    names = _split(text)
    return names or sorted(REGISTRY)


def _base_config(args) -> dict:
    base: dict = {}
    if getattr(args, "config", None):
        p = Path(args.config)
        if not p.exists():
            p = CONFIGS / (args.config if args.config.endswith(".json") else args.config + ".json")
        base = dict(json.loads(p.read_text()).get("knobs", {}))
    for item in getattr(args, "set", None) or []:
        k, _, v = item.partition("=")
        base[k] = runner.parse_value(v)
    return base


def _vary(items: Sequence[str]) -> dict:
    out = {}
    for item in items:
        k, _, vs = item.partition("=")
        out[k] = [runner.parse_value(v) for v in vs.split(",")]
    return out


def _filters(args) -> dict:
    f = {}
    for k in ("workload", "metric", "backend", "level"):
        v = getattr(args, k, None)
        if v:
            f[k] = _split(v) if "," in v else v
    return f


# ---------------------------------------------------------------- commands

def cmd_knobs(args) -> int:
    support = {}
    for name in sorted(REGISTRY):
        try:
            support[name] = load_backend(name).knob_support()
        except Exception as e:
            support[name] = {"__error__": f"{type(e).__name__}: {e}"}
    if args.json:
        emit_json({"knobs": [{"id": k.id, "label": k.label, "kind": k.kind, "default": k.default,
                              "domain": list(k.domain), "unit": k.unit, "group": k.group, "help": k.help,
                              "stage": k.stage} for k in knobs.KNOBS], "support": support})
        return 0
    short = {"full": "full", "default_only": "default"}
    rows = [(k.id, k.kind, k.default, ("/".join(map(str, k.domain)) if k.domain else ""), k.unit,
             *(("ERR" if "__error__" in support[b] else short.get(support[b].get(k.id, "?"), support[b].get(k.id, "?")))
               for b in sorted(support))) for k in knobs.KNOBS]
    print(table(rows, ("knob", "kind", "default", "domain", "unit", *sorted(support))))
    for b in sorted(support):
        for k, v in support[b].items():
            if k == "__error__":
                print(f"\n{b}: failed to load: {v}")
            elif v.startswith("partial"):
                print(f"\n{b}.{k}: {v}")
    return 0


def cmd_workloads(args) -> int:
    out = []
    for name in _backends(args.backend):
        try:
            for w in load_backend(name).workloads():
                out.append({"backend": name, "id": w.id, "kind": w.kind, "params": w.params, "label": w.label})
        except Exception as e:
            out.append({"backend": name, "id": None, "kind": "", "params": {}, "label": f"failed to load: {e}"})
    if args.json:
        emit_json(out)
    else:
        print(table([(o["backend"], o["id"] or "-", o["kind"], json.dumps(o["params"]), o["label"]) for o in out],
                    ("backend", "workload", "kind", "params", "note")))
    return 0


def _run_and_report(args, vary: dict) -> int:
    base = _base_config(args)
    specs, refused = runner.expand_sweep(base, vary, _split(args.workload), _backends(args.backend))
    skipped: list[dict] = []

    def on_event(ev):
        if ev["event"] == "skipped":
            skipped.append(ev)
        if not args.json and ev["event"] in ("cache_hit", "done"):
            print(f"  [{ev['event']}] {ev['backend']} {ev['workload']}", file=sys.stderr)

    recs = runner.run_specs(specs, jobs=args.jobs, allow_long=args.allow_long, use_cache=not args.no_cache,
                            on_event=on_event)
    errors = [r for r in recs if r.status == "error"]
    if args.json:
        emit_json({"records": store.as_dicts(recs), "refused": refused,
                   "skipped": [{"backend": s["backend"], "workload": s["workload"], "reason": s["reason"]} for s in skipped]})
    else:
        if recs:
            print(record_rows(recs))
        print(f"\n{len(specs)} spec(s), {len(recs)} record(s), {len(errors)} error(s)")
        if refused:
            print(f"\nrefused ({len(refused)}):")
            print(table([(r["backend"], r["workload"], json.dumps(r["config"]) if r["config"] else "MVP", r["reason"])
                         for r in refused], ("backend", "workload", "config", "reason")))
        if skipped:
            print(f"\nskipped ({len(skipped)}):")
            print(table([(s["backend"], s["workload"], s["reason"]) for s in skipped], ("backend", "workload", "reason")))
    return 1 if errors else 0


def cmd_run(args) -> int:
    return _run_and_report(args, {})


def cmd_sweep(args) -> int:
    return _run_and_report(args, _vary(args.vary))


def cmd_compare(args) -> int:
    recs = store.query(metric=args.metric, **{k: v for k, v in _filters(args).items() if k != "metric"})
    recs = [r for r in recs if r.status != "error"]
    cols: list[str] = []
    cells: dict[tuple, Record] = {}
    for r in recs:  # last record wins per cell
        col = f"{r.level}/{r.backend}"
        if col not in cols:
            cols.append(col)
        cells[(r.config_hash, r.workload, json.dumps(r.workload_params, sort_keys=True), col)] = r
    cols.sort()
    rowkeys = sorted({k[:3] for k in cells}, key=lambda k: (k[1], k[2], k[0]))
    rows, jrows = [], []
    for ch, w, wp in rowkeys:
        any_r = next(r for k, r in cells.items() if k[:3] == (ch, w, wp))
        label = knobs.label(any_r.config) if any_r.config else ch
        vals = [cells.get((ch, w, wp, c)) for c in cols]
        rows.append((label, w + _wp(json.loads(wp)), *("-" if v is None else fmt_value(v) for v in vals)))
        jrows.append({"config": label, "config_hash": ch, "workload": w, "workload_params": json.loads(wp),
                      "values": {c: (None if v is None else {"value": v.value, "band": v.band, "status": v.status,
                                                              "unit": v.unit, "id": store.record_id(v)})
                                 for c, v in zip(cols, vals)}})
    if args.json:
        emit_json({"metric": args.metric, "columns": cols, "rows": jrows})
    elif not rows:
        print(f"no records for metric {args.metric!r}")
    else:
        print(table(rows, ("config", "workload", *cols)))
    return 0


def cmd_check(args) -> int:
    found = crosscheck.findings(store.load())  # filter after the join, or cross-level pairs vanish
    flt = _filters(args)

    def keep(f):
        for key, want in flt.items():
            wants = want if isinstance(want, list) else [want]
            if key in ("workload", "metric"):
                if f[key] not in wants:
                    return False
            elif not any(s and s[key] in wants for s in (f["a"], f["b"])):
                return False
        return True
    found = [f for f in found if keep(f)]
    summ = crosscheck.summary(found)
    shown = [f for f in found if args.all or f["verdict"] != "match"]
    if args.json:
        emit_json({"summary": summ, "findings": shown if not args.all else found})
    else:
        if shown:
            rows = [(f["verdict"], f["workload"] + _wp(f["workload_params"]), f["metric"],
                     f"{f['a']['level']}/{f['a']['backend']}={f['a']['value']}",
                     "-" if not f["b"] else f"{f['b']['level']}/{f['b']['backend']}={f['b']['value']}", f["detail"])
                    for f in shown]
            print(table(rows, ("verdict", "workload", "metric", "a", "b", "detail")))
            print()
        print("  ".join(f"{k}: {v}" for k, v in summ.items()))
    return 1 if (args.strict and summ["off"]) else 0


def cmd_trace(args) -> int:
    hits = store.query(id_prefix=args.prefix)
    if not hits:
        print(f"no record with id starting {args.prefix!r}", file=sys.stderr)
        return 1
    if len(hits) > 1:
        print(f"{len(hits)} records match {args.prefix!r}; give more characters:\n" + record_rows(hits), file=sys.stderr)
        return 1
    r = hits[0]
    from dataclasses import asdict
    if args.json:
        emit_json({"id": store.record_id(r), "record": asdict(r), "command": r.provenance.get("command")})
        return 0
    print(f"record   {store.record_id(r)}")
    print(f"what     {r.workload}{_wp(r.workload_params)} / {r.metric} = {fmt_value(r)} {r.unit}")
    print(f"how      level {r.level}, backend {r.backend}, method {r.method}, status {r.status}")
    if r.note:
        print(f"note     {r.note}")
    print(f"config   {knobs.label(r.config) if r.config else '-'}  (hash {r.config_hash})")
    print("provenance")
    for k, v in r.provenance.items():
        print(f"  {k:10} {v}")
    print(f"reproduce  {r.provenance.get('command', '(none recorded)')}")
    return 0


def cmd_import(args) -> int:
    if "import" not in REGISTRY:
        print("importers not installed yet (task E)", file=sys.stderr)
        return 2
    args.workload = args.name or ",".join(w.id for w in load_backend("import").workloads())
    # cached: the importer's code_hash covers every source file's content, so an unchanged source is a cache hit
    # (re-importing used to append another full copy of the past evidence on every `bench-report`)
    args.backend, args.set, args.config, args.jobs, args.allow_long = "import", [], None, 1, True
    args.no_cache = getattr(args, "no_cache", False)
    return _run_and_report(args, {})


def cmd_serve(args) -> int:
    try:
        from .ui.server import serve
    except ImportError:
        print("front end not installed yet (task F)", file=sys.stderr)
        return 2
    serve(args.port, store.store_path(args.store))
    return 0


def cmd_export(args) -> int:
    try:
        from .ui.export import export
    except ImportError:
        print("front end not installed yet (task F)", file=sys.stderr)
        return 2
    recs = store.load(args.snapshot) if args.snapshot else store.load()
    export(args.html, recs)
    print(f"wrote {args.html} ({len(recs)} records)")
    return 0


def cmd_snapshot(args) -> int:
    recs = store.query(**_filters(args))
    out = store.snapshot(args.name, recs)
    if args.json:
        emit_json({"path": str(out), "records": len(recs)})
    else:
        print(f"wrote {out} ({len(recs)} records)")
    return 0


# ---------------------------------------------------------------- parser

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="python -m tools.bench", description="Evidence bench")
    store_help = ("records file (default: env BENCH_STORE, else tools/bench/results/records.jsonl). "
                    "The run cache is the directory 'cache/' next to this file, so two store files in one directory "
                    "share one cache (use one directory per store for a fresh start). "
                  "Accepted before or after the subcommand")
    ap.add_argument("--store", help=store_help)
    # the subparsers re-declare --store with SUPPRESS so that an absent one does not overwrite the global value
    store_parent = argparse.ArgumentParser(add_help=False)
    store_parent.add_argument("--store", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    sub = ap.add_subparsers(dest="cmd")

    def add(name, fn, help_):
        p = sub.add_parser(name, help=help_, parents=[store_parent])
        p.add_argument("--json", action="store_true", help="machine-readable output")
        p.set_defaults(fn=fn)
        return p

    def runargs(p):
        p.add_argument("--config", help="config file (path, or a name under tools/bench/configs)")
        p.add_argument("--set", action="append", default=[], metavar="K=V", help="knob value (repeatable)")
        p.add_argument("--workload", required=True, help="W[,W]; W is 'id' or 'id:param=v'")
        p.add_argument("--backend", required=True, help="B[,B]")
        p.add_argument("--jobs", type=int, default=4)
        p.add_argument("--allow-long", action="store_true")
        p.add_argument("--no-cache", action="store_true")

    def filt(p, metric=True):
        p.add_argument("--workload")
        p.add_argument("--backend")
        p.add_argument("--level")
        if metric:
            p.add_argument("--metric")

    add("knobs", cmd_knobs, "knob table with the support of every backend")
    p = add("workloads", cmd_workloads, "workloads each backend can run")
    p.add_argument("--backend")
    runargs(add("run", cmd_run, "run, store, print a table"))
    p = add("sweep", cmd_sweep, "cartesian sweep over knob values")
    runargs(p)
    p.add_argument("--vary", action="append", required=True, metavar="K=V1,V2")
    p = add("compare", cmd_compare, "configs x levels for one metric")
    filt(p, metric=False)
    p.add_argument("--metric", required=True)
    p = add("check", cmd_check, "cross-level checks over the store")
    filt(p)
    p.add_argument("--all", action="store_true", help="also list matches")
    p.add_argument("--strict", action="store_true", help="exit 1 when any finding is 'off'")
    p = add("trace", cmd_trace, "a record, its provenance and the command that reproduces it "
                                "(record ids change on every real re-run; find the latest one with compare --json)")
    p.add_argument("prefix", metavar="RECORD_ID_PREFIX")
    p = add("import", cmd_import, "run the importers of past evidence")
    p.add_argument("name", nargs="?")
    p = add("serve", cmd_serve, "the local front end")
    p.add_argument("--port", type=int, default=8765)
    p = add("export", cmd_export, "static report of the store or a snapshot")
    p.add_argument("--html", required=True, metavar="OUT")
    p.add_argument("--snapshot", help="read this snapshot file instead of the store")
    p = add("snapshot", cmd_snapshot, "write a committed snapshot")
    p.add_argument("name")
    filt(p)
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if not args.cmd:
        ap.print_help(sys.stderr)
        print("\nNo subcommand given. Start with `python -m tools.bench knobs`; see tools/bench/README.md section 2.",
              file=sys.stderr)
        return 2
    if args.store:
        import os
        os.environ["BENCH_STORE"] = args.store  # also reaches worker subprocesses
    try:
        return args.fn(args)
    except knobs.KnobError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    except TypeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

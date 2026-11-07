# tools/bench contract

Normative for every bench component. `tools/bench/types.py` and `tools/bench/knobs.py` implement sections 1-2 and
are owned by the maintainers: a change there should be proposed to them rather than made in passing.

## 0. What the bench is

One environment, several simulators. A **config** (knob values, MVP by default) and a **workload** (a protocol
bench at a speed, a mix, a bridge pair, or a chip-level measurement) run on a **backend** (analytic, ISS,
RTL, gate level, flow, power, or an importer of past evidence) and produce **records**. Records go to one
store. The CLI, the front end, cross-checks and the write-up all read the store. The front end holds no
model logic.

## 1. Config and knobs

- `knobs.KNOBS` is the only list of knobs. Ids follow archsim's where archsim has the knob.
- A config is a dict knob_id -> value. `knobs.normalize(partial)` fills defaults (the MVP), validates, and
  rejects unknown ids. Everything stored or hashed is normalized. `knobs.config_hash(cfg)` identifies it.
- Config files: JSON `{"label": "...", "knobs": {...partial...}}` under `tools/bench/configs/`. The MVP is
  `configs/mvp.json` (empty knobs).

## 2. Records

`types.Record` is the only stored unit. Rules:

- `metric` names are snake_case and **the same across levels** for the same quantity, so cross-checks join on
  `(config_hash, workload, workload_params, metric)`. The metric vocabulary lives in
  `tools/bench/metrics.py` (`METRICS: {name: (unit, description)}`); a backend may only emit names listed
  there (task A creates it with the perf-b and archsim names; other tasks add theirs in their own section of
  that file, appended, never reordered).
- A value nobody observed is `status="n/a"` with the reason in `note`, never 0. A broken run is
  `status="error"`. An error record uses the placeholder metric `run_error` (crash, timeout, unreadable output, a metric
  outside the vocabulary); error records are never cached and never cross-checked.
- `method`: `measured` (a simulator or tool produced it), `derived` (a formula over measured inputs),
  `estimate` (a judgement or an archsim `judgement`-status input), `spec`, `imported`.
- `provenance` holds at least: `command` (re-runnable shell command), `commit`, `dirty` (bool), `started`
  (ISO time), `duration_s`. Importers add `source` (path, and line or row where possible).

## 3. Backends

- A backend is a class implementing `types.Backend`, registered by name in
  `tools/bench/backends/__init__.py` `REGISTRY = {name: "module.path:ClassName"}` (lazy import, so a missing
  tool only breaks its own backend). Each task adds its own line there.
- `supports()` returns None or a one-line reason. A backend never silently runs a different config than
  asked: a knob it cannot honour => it refuses via `supports()`.
- `knob_support()` covers every knob id: `"full"`, `"default_only"`, or `"partial: <why>"`.
- **Runs happen in a worker subprocess**: `python -m tools.bench.worker` reads one RunSpec as JSON on stdin and
  writes records as JSON lines on stdout. Reason: `fw/targets.py` installs a chip class globally per process,
  and the RTL harness owns child processes. Exit status 0 even when records have `fail`; non-zero only when the
  backend crashed (the runner then stores one `error` record with the stderr tail).
- Backends must not write the store, change the working tree or touch global state outside their process.

## 4. Store, cache, runner

- Store: `tools/bench/results/records.jsonl` (append-only, gitignored). Cache: `tools/bench/results/cache/<run_key>.jsonl`.
  `run_key = RunSpec.key(backend.code_hash() + input_hash)`, where `input_hash` is the optional backend method
  `input_hash(spec) -> str` (hash of external artifacts the run reads, e.g. a finished flow), empty if absent. A cache hit returns stored records without running.
- Snapshots: `bench snapshot NAME` copies the current store (or a filtered part) to
  `tools/bench/snapshots/NAME.jsonl`, which is committed. The front end and write-up can read snapshots.
- Runner: parallel worker processes (`--jobs`, default 4), per-level timeouts, and a **long-run gate**:
  levels `flow`, `gl`, `power`, and any spec for which the backend's optional `is_long(spec)` returns True, run only
  with `--allow-long` (a backend without `is_long` is never long unless its level is one of those three).

- Workloads on the CLI and API are strings `id` or `id:param=v[:param=v]`. Params override the backend's defaults
  and are part of the cross-check join key, so level-specific tuning belongs in the config, not in workload params.
  A workload a backend does not offer is a refusal.

## 5. CLI (`python -m tools.bench <cmd>`, also `make bench ARGS=...`)

| command | does |
|---|---|
| `knobs` | knob table, with the support of every registered backend per knob |
| `workloads [--backend B]` | workloads each backend can run |
| `run --config F \| --set k=v ... --workload W[,W] --backend B[,B] [--jobs N] [--allow-long] [--no-cache]` | run, store, print a table |
| `sweep --vary k=v1,v2 [--vary ...] [--set ...] --workload ... --backend ...` | cartesian sweep, skipping specs a backend refuses (listed, with reasons) |
| `compare --metric M [--workload W] [--backend ...]` | table: configs x levels for one metric |
| `check` | cross-level checks over the store (section 6); prints findings |
| `trace RECORD_ID_PREFIX` | the record, its provenance, and the command that reproduces it |
| `import [NAME]` | run the importers of past evidence (level `import`) |
| `serve [--port 8765]` | the local front end |
| `export --html OUT` | static report of the store or a snapshot (no server) |
| `snapshot NAME [filters]` | write a committed snapshot |

Every command takes `--json` for machine output. Record ids for `trace` = `sha(record json)[:12]`.

## 6. Cross-checks

`tools/bench/crosscheck.py`: join records on `(config_hash, workload, workload_params, metric)` across
levels; compare with a per-metric tolerance (`TOLERANCES` dict; default exact for cycle counts, 2 % for
rates, band overlap when either side has a band). Each comparison is a finding: `match`, `close`, `off`,
or `one-sided` (only one level has it). `off` findings are results, not tuning targets. Negative controls:
`tools/bench/tests/test_crosscheck.py` must show each rule can fail.

## 7. Tests and speed

- Tests under `tools/bench/tests/test_<area>.py`. `make bench-test` runs all non-long tests and must stay under
  ~3 minutes total. Anything slower is `@pytest.mark.long` and runs in `make bench-test-long`.
- Every task's acceptance includes: its tests pass, `make bench-test` passes, and `make verify-fast` still
  passes when the task touched anything outside `tools/bench/`.

## 8. Ownership (who may edit what)

- `tools/bench/types.py`, `knobs.py`, `CONTRACT.md`, `PLAN.md`: orchestrator only.
- `iss/pio_iss.py`, `verif/iss_b/`: read-only for everyone (they are the verified references).
- `src/*.sv`: changes only with the MVP default bit-for-bit unchanged in behaviour.
- `fw/targets.py`, `fw/*/driver.py` register addressing: owned by the firmware sources, not the bench.
- Shared registries (`backends/__init__.py` REGISTRY, `metrics.py`, `tools/bench/mk/*.mk`): each backend appends
  its own lines / its own `mk/<name>.mk` file.
- Everything else: the files of the component being changed.

"""Core tests on the fake backends: store, cache, runner, long gate, sweep, CLI."""
import json
import subprocess
import os
import sys
import time
from pathlib import Path

import pytest

from tools.bench import knobs, runner, store
from tools.bench.cli import main
from tools.bench.types import RunSpec, Workload

REPO = Path(__file__).resolve().parents[3]


@pytest.fixture(autouse=True)
def tmp_store(tmp_path, monkeypatch):
    monkeypatch.setenv("BENCH_STORE", str(tmp_path / "results" / "records.jsonl"))
    monkeypatch.setenv("BENCH_SNAPSHOTS", str(tmp_path / "snaps"))
    return tmp_path


def spec(wid="count", backend="fake", cfg=None, **params):
    base = {w.id: w for w in runner.load_backend(backend).workloads()}[wid]
    return RunSpec(knobs.normalize(cfg), Workload(wid, base.kind, {**base.params, **params}), backend)


def test_run_stores_records_with_provenance():
    recs = runner.run_specs([spec()], jobs=1)
    assert {r.metric for r in recs} == {"sm_cycles_per_bit", "max_bit_rate", "verdict"}
    assert store.load() == recs
    p = recs[0].provenance
    assert p["commit"] and isinstance(p["dirty"], bool) and p["started"] and p["duration_s"] >= 0
    assert "tools.bench run" in p["command"] and "--no-cache" in p["command"]
    assert recs[0].config_hash == knobs.config_hash(recs[0].config) and recs[0].run_key


def test_deterministic_across_configs():
    a = runner.run_specs([spec(cfg={"n_engines": 1}), spec()], jobs=2, save=False)
    assert [r.value for r in a[:3]] != [r.value for r in a[3:]]
    b = runner.run_specs([spec()], jobs=1, use_cache=False, save=False)
    assert [r.value for r in b] == [r.value for r in a[3:]]


def test_cache_hit_and_miss():
    ev = []
    runner.run_specs([spec()], on_event=ev.append)
    runner.run_specs([spec()], on_event=ev.append)
    runner.run_specs([spec()], on_event=ev.append, use_cache=False)
    runner.run_specs([spec(cfg={"fifo_depth": 8})], on_event=ev.append)
    kinds = [e["event"] for e in ev if e["event"] in ("cache_hit", "start")]
    assert kinds == ["start", "cache_hit", "start", "start"]
    n = len(store.load())
    runner.run_specs([spec()])  # another hit: the store must not grow
    assert len(store.load()) == n


def test_cache_key_follows_code_hash():
    assert spec().key("a") != spec().key("b")


def test_parallel_faster_than_serial():
    specs = [spec("slow", delay=0.8, speed=s) for s in (1, 2, 3, 4)]
    t = time.time()
    runner.run_specs(specs, jobs=4, use_cache=False, save=False)
    assert time.time() - t < 0.8 * 4 * 0.9


def test_error_capture_crash_exception_timeout_badmetric():
    crash = runner.run_specs([spec("crashing")], save=False)
    assert [(r.status, r.metric) for r in crash] == [("error", "run_error")]
    assert "exit 3" in crash[0].note and "simulated crash" in crash[0].note
    exc = runner.run_specs([spec("raising")], save=False)
    assert exc[0].status == "error" and "simulated exception" in exc[0].note
    hang = runner.run_specs([spec("hanging")], save=False, timeouts={"iss": 1})
    assert hang[0].status == "error" and "timed out" in hang[0].note
    bad = runner.run_specs([spec(mode="badmetric")], save=False)
    assert bad[0].status == "error" and "not in tools/bench/metrics.py" in bad[0].note
    ev = []  # errors are not cached
    runner.run_specs([spec("crashing")], on_event=ev.append, save=False)
    assert "cache_hit" not in [e["event"] for e in ev]


def test_stdout_noise_does_not_corrupt_records():
    recs = runner.run_specs([spec(mode="noise")], save=False)
    assert all(r.status == "ok" for r in recs) and len(recs) == 3


def test_fail_and_na_are_kept_not_errors():
    f = runner.run_specs([spec("failing")], save=False)
    assert "fail" in {r.status for r in f}
    n = runner.run_specs([spec("na")], save=False)
    na = [r for r in n if r.metric == "max_bit_rate"][0]
    assert na.status == "n/a" and na.value is None and na.note


def test_long_gate():
    ev = []
    assert runner.run_specs([spec("longrun")], on_event=ev.append) == []
    assert ev[0]["event"] == "skipped"
    assert len(runner.run_specs([spec("longrun")], allow_long=True)) == 3


def test_sweep_expansion_and_refusals():
    specs, refused = runner.expand_sweep({}, {"n_engines": [1, 2], "lane": [False, True]}, ["count"], ["fake", "fake2"])
    assert len(specs) == 4 and len(refused) == 4
    assert all("lane" in r["reason"] for r in refused)
    _, ref = runner.expand_sweep({}, {}, ["nonesuch", "count:refuse=no way"], ["fake"])
    assert "not offered" in ref[0]["reason"] and ref[1]["reason"] == "no way"
    _, ref = runner.expand_sweep({}, {}, ["count"], ["nosuchbackend"])
    assert "failed to load" in ref[0]["reason"]
    with pytest.raises(knobs.KnobError):
        runner.expand_sweep({}, {"n_engines": [99]}, ["count"], ["fake"])


def test_store_query_and_snapshot():
    runner.run_specs([spec(), spec(cfg={"n_engines": 1})])
    assert {r.metric for r in store.query(metric="verdict")} == {"verdict"}
    assert len(store.query(level=["iss", "rtl"], metric=["verdict", "max_bit_rate"])) == 4
    first = store.load()[0]
    assert store.query(id_prefix=store.record_id(first)[:6])[0] == first
    with pytest.raises(TypeError):
        store.query(metrc="x")
    out = store.snapshot("t", store.load())
    assert len(out.read_text().splitlines()) == len(store.load())


def run_cli(*argv, capsys):
    code = main(list(argv))
    return code, capsys.readouterr().out


def test_cli_end_to_end(capsys):
    code, out = run_cli("run", "--backend", "fake,fake2", "--workload", "count", "--set", "n_engines=3", "--json", capsys=capsys)
    assert code == 0 and len(json.loads(out)["records"]) == 6
    code, out = run_cli("sweep", "--vary", "n_engines=1,2", "--vary", "lane=false,true", "--workload", "count",
                        "--backend", "fake", "--json", capsys=capsys)
    d = json.loads(out)
    assert len(d["refused"]) == 2 and len(d["records"]) == 6
    code, out = run_cli("compare", "--metric", "max_bit_rate", capsys=capsys)
    assert "iss/fake" in out and "rtl/fake2" in out and "n_engines=3" in out
    code, out = run_cli("compare", "--metric", "max_bit_rate", "--json", capsys=capsys)
    assert json.loads(out)["columns"] == ["iss/fake", "rtl/fake2"]
    code, out = run_cli("check", capsys=capsys)
    assert code == 0 and "off: 0" in out
    rid = store.record_id(store.load()[0])
    code, out = run_cli("trace", rid[:8], capsys=capsys)
    assert code == 0 and "reproduce  " in out and "-m tools.bench run" in out
    code, out = run_cli("trace", "zzzz", capsys=capsys)
    assert code == 1
    code, out = run_cli("snapshot", "s1", "--metric", "verdict", capsys=capsys)
    assert "records" in out
    code, out = run_cli("knobs", capsys=capsys)
    assert "fake2" in out and "default" in out
    code, out = run_cli("workloads", "--backend", "fake", capsys=capsys)
    assert "longrun" in out


def test_cli_errors_and_missing_front_end():
    assert main(["run", "--backend", "fake", "--workload", "count", "--set", "n_engines=99"]) == 2
    assert main(["run", "--backend", "fake", "--workload", "crashing"]) == 1
    try:
        import tools.bench.ui.export  # noqa: F401
    except ImportError:
        assert main(["serve"]) == 2
        assert main(["export", "--html", "x.html"]) == 2


def test_reproduce_command_reruns_same_values():
    recs = runner.run_specs([spec(cfg={"fifo_depth": 8})], save=False)
    cmd = recs[0].provenance["command"] + " --json"
    out = subprocess.run(cmd, shell=True, cwd=REPO, capture_output=True, text=True)
    again = json.loads(out.stdout)["records"]
    assert [r["value"] for r in again] == [r.value for r in recs]


def test_corrupt_cache_is_a_miss_and_partial_store_line_is_skipped(tmp_store, capfd):
    s = spec()
    runner.run_specs([s])
    cache = list((tmp_store / "results" / "cache").glob("*.jsonl"))
    assert len(cache) == 1
    cache[0].write_text("{not json\n")
    ev = []
    recs = runner.run_specs([s], on_event=ev.append)
    assert "cache_hit" not in [e["event"] for e in ev] and len(recs) == 3
    assert "corrupt cache" in capfd.readouterr().err
    with (tmp_store / "results" / "records.jsonl").open("a") as f:
        f.write('{"workload": "x", "metr')  # a crash mid-write
    assert len(store.load()) == 6
    assert "undecodable" in capfd.readouterr().err


def test_records_stored_as_each_run_finishes_and_bad_callback_is_harmless():
    seen = []

    def on_event(ev):
        if ev["event"] == "done":
            seen.append(len(store.load()))   # already in the store when "done" is announced
            raise RuntimeError("callback bug")

    recs = runner.run_specs([spec(), spec(cfg={"n_engines": 1})], jobs=1, on_event=on_event)
    assert seen == [3, 6] and len(recs) == 6 and len(store.load()) == 6


def test_child_process_stdout_cannot_corrupt_records():
    recs = runner.run_specs([spec(mode="childnoise")], save=False)
    assert len(recs) == 3 and all(r.status == "ok" for r in recs)


def test_timeout_kills_grandchildren(tmp_store):
    pidfile = tmp_store / "pid"
    runner.run_specs([spec("hanging", pidfile=str(pidfile))], save=False, timeouts={"iss": 1.5})
    pid = int(pidfile.read_text())
    time.sleep(0.3)
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


def test_check_filters_after_join(capsys):
    runner.run_specs([spec(), spec(backend="fake2")])
    code, out = run_cli("check", "--level", "rtl", "--all", "--json", capsys=capsys)
    d = json.loads(out)
    assert d["summary"]["match"] == 3 and d["summary"]["one-sided"] == 0


def test_store_flag_before_or_after_subcommand():
    from tools.bench.cli import build_parser
    ap = build_parser()
    assert ap.parse_args(["--store", "X", "serve"]).store == "X"
    assert ap.parse_args(["serve", "--store", "X"]).store == "X"
    assert ap.parse_args(["serve"]).store is None
    assert ap.parse_args(["--store", "G", "check"]).store == "G"  # an absent local flag must not clobber it


def test_bare_invocation_prints_help_and_exits_2(capsys):
    assert main([]) == 2
    err = capsys.readouterr().err
    assert "usage:" in err and "README.md section 2" in err

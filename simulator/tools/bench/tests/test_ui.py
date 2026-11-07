"""Front end tests: every route against the fixture store, a fake run job, the static export."""
import json
import os
import re
import threading
import time
import urllib.error
import urllib.request

import pytest

from tools.bench import knobs
from tools.bench.types import Record, Workload
from tools.bench.ui import api, export, server

FIXTURE = os.path.join(os.path.dirname(os.path.dirname(__file__)), "fixtures", "sample_records.jsonl")


class FakeBackend:
    def __init__(self, name, default_only=()):
        self.name, self.default_only = name, set(default_only)

    def knob_support(self):
        return {k.id: ("default_only" if k.id in self.default_only else "full") for k in knobs.KNOBS}

    def workloads(self):
        return [Workload("uart_tx", "protocol", {"speed": 115200}, "UART TX"), Workload("spi", "protocol"),
                Workload("chip", "chip")]

    def supports(self, config, workload):
        return None


FAKES = {"analytic": FakeBackend("analytic"), "iss": FakeBackend("iss", default_only=("n_engines", "fifo_depth")),
         "rtl": FakeBackend("rtl", default_only=("fifo_depth",))}


@pytest.fixture(autouse=True)
def fake_adapter(monkeypatch):
    monkeypatch.setattr(api, "_registry", lambda: dict.fromkeys(FAKES, "x"))
    monkeypatch.setattr(api, "backend", lambda n: FAKES[n])
    calls = []

    def fake_start_run(req, on_event=None, store_path=None):
        calls.append(req)
        on_event({"event": "total", "n": 3})
        for _ in range(5):  # more events than specs: progress must never overshoot
            on_event({"event": "done"})
        return {"specs": 3, "records": 3, "status_counts": {"ok": 3}, "refused": [{"backend": "iss", "workload": "spi", "reason": "fifo_depth!=4"}], "skipped": []}

    monkeypatch.setattr(api, "start_run", fake_start_run)
    return calls


@pytest.fixture()
def base_url():
    srv = server.make_server(0, FIXTURE)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    yield "http://127.0.0.1:%d" % srv.server_address[1]
    srv.shutdown()
    srv.server_close()


def get(url, raw=False):
    with urllib.request.urlopen(url, timeout=10) as r:
        body = r.read()
        return body if raw else json.loads(body)


def post(url, obj):
    req = urllib.request.Request(url, json.dumps(obj).encode(), {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_fixture_is_wellformed():
    lines = [l for l in open(FIXTURE) if l.strip()]
    assert len(lines) >= 60
    recs = [Record.from_json(l) for l in lines]
    assert all("fixture" in r.note for r in recs)
    assert {"analytic", "iss", "rtl", "import"} <= {r.level for r in recs}
    assert {"n/a", "error", "ok"} <= {r.status for r in recs}


def test_static_pages(base_url):
    assert b"Evidence bench" in get(base_url + "/", raw=True)
    assert b"use strict" in get(base_url + "/app.js", raw=True)
    assert b"--bg" in get(base_url + "/app.css", raw=True)


def test_knobs_and_workloads(base_url):
    k = get(base_url + "/api/knobs")
    assert {x["id"] for x in k["knobs"]} == {x.id for x in knobs.KNOBS}
    assert k["support"]["iss"]["fifo_depth"] == "default_only"
    w = get(base_url + "/api/workloads")
    ids = {x["id"] for x in w}
    assert {"uart_tx", "spi", "chip", "can_rx"} <= ids  # fakes plus what the store has seen


def test_records_filters(base_url):
    allr = get(base_url + "/api/records")
    assert len(allr) >= 60 and all(r["id"] and "config_label" in r for r in allr)
    iss = get(base_url + "/api/records?level=iss")
    assert iss and all(r["level"] == "iss" for r in iss)
    one = get(base_url + "/api/records?workload=chip&metric=area_um2&level=analytic")
    assert len(one) == 3
    h = allr[0]["config_hash"]
    assert all(r["config_hash"] == h for r in get(base_url + "/api/records?config_hash=" + h))
    assert get(base_url + "/api/records?backend=nonexistent") == []


def test_check_has_all_verdicts(base_url):
    verdicts = {f["verdict"] for f in get(base_url + "/api/check")}
    assert {"match", "off", "one-sided"} <= verdicts
    off = [f for f in get(base_url + "/api/check") if f["verdict"] == "off"]
    assert any(f["workload"] == "spi" and f["metric"] == "cycles_per_byte" for f in off)


def test_trace(base_url):
    rec = get(base_url + "/api/records?level=iss")[0]
    t = get(base_url + "/api/trace/" + rec["id"])
    assert t["id"] == rec["id"] and t["provenance"]["command"]
    assert get(base_url + "/api/trace/" + rec["id"][:8])["id"] == rec["id"]
    with pytest.raises(urllib.error.HTTPError) as e:
        get(base_url + "/api/trace/zzzzzzzz")
    assert e.value.code == 404


def test_unknown_route(base_url):
    with pytest.raises(urllib.error.HTTPError) as e:
        get(base_url + "/api/nope")
    assert e.value.code == 404


def test_run_creates_job_that_finishes(base_url, fake_adapter):
    code, j = post(base_url + "/api/run", {"configs": [{"fifo_depth": 8}], "workloads": ["spi"], "backends": ["iss"]})
    assert code == 202 and j["job"]
    for _ in range(100):
        jobs = get(base_url + "/api/jobs")
        if jobs and jobs[0]["state"] != "running":
            break
        time.sleep(0.05)
    assert jobs[0]["id"] == j["job"] and jobs[0]["state"] == "done"
    assert jobs[0]["events"] == 3 and jobs[0]["total"] == 3 and jobs[0]["result"]["refused"]
    assert fake_adapter[0]["configs"] == [{"fifo_depth": 8}]


def test_run_sweep_and_errors(base_url, fake_adapter, monkeypatch):
    code, _ = post(base_url + "/api/run", {"workloads": [], "backends": ["iss"]})
    assert code == 400

    def boom(req, on_event=None, store_path=None):
        raise RuntimeError("backend exploded")

    monkeypatch.setattr(api, "start_run", boom)
    code, j = post(base_url + "/api/run", {"sweep": {"base": {}, "vary": {"fifo_depth": [2, 4]}}, "workloads": ["spi"], "backends": ["iss"]})
    assert code == 202
    for _ in range(100):
        job = get(base_url + "/api/jobs")[0]
        if job["state"] != "running":
            break
        time.sleep(0.05)
    assert job["state"] == "error" and "exploded" in job["error"]


def test_count_specs():
    assert api.count_specs({"sweep": {"vary": {"a": [1, 2, 3], "b": [1, 2]}}, "workloads": ["x", "y"], "backends": ["i"]}) == 12
    assert api.count_specs({"configs": [{}], "workloads": ["x"], "backends": ["i", "j"]}) == 2


def test_export_self_contained(tmp_path):
    recs = api.load_records(FIXTURE)
    out = export.export(str(tmp_path / "report.html"), recs)
    text = open(out, encoding="utf-8").read()
    assert text.count("<html") == 1
    assert "http://" not in text and "https://" not in text
    assert not re.search(r'(src|href)="(?!#)', text)  # no external script/link references
    m = re.search(r"window\.__BENCH__ = (.*?);</script>", text, re.S)
    data = json.loads(m.group(1).replace("<\\/", "</"))
    assert len(data["records"]) == len(recs)
    assert {f["verdict"] for f in data["findings"]} >= {"match", "off"}
    assert data["knobs"]["support"]["rtl"]["fifo_depth"] == "default_only"


def raw_post(url, body, headers):
    req = urllib.request.Request(url, body, headers)
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_post_validation_and_guards(base_url, fake_adapter):
    u = base_url + "/api/run"
    ok = {"workloads": ["spi"], "backends": ["iss"]}
    j = {"Content-Type": "application/json"}
    assert raw_post(u, json.dumps(ok).encode(), {"Content-Type": "text/plain"})[0] == 415
    assert raw_post(u, json.dumps(ok).encode(), {**j, "Origin": "http://evil.example"})[0] == 403
    assert raw_post(u, json.dumps(ok).encode(), {**j, "Host": "rebind.example:80"})[0] == 403
    try:
        assert raw_post(u, b"x" * 1_100_000, j)[0] == 413
    except urllib.error.URLError as e:     # the server may close before reading the oversized body: that is a refusal too
        assert isinstance(e.reason, (BrokenPipeError, ConnectionResetError)), e
    assert raw_post(u, b"[1]", j)[0] == 400
    code, e = post(u, {**ok, "backends": ["nope"]})
    assert code == 400 and "unknown backend" in e["error"]
    code, e = post(u, {**ok, "workloads": ["nope"]})
    assert code == 400 and "unknown workload" in e["error"]
    code, e = post(u, {**ok, "configs": [{"fifo_depth": 999}]})
    assert code == 400 and "fifo_depth" in e["error"]
    code, e = post(u, {**ok, "sweep": {"base": {}, "vary": {"bogus": [1]}}})
    assert code == 400
    assert get(base_url + "/api/records", raw=True)  # GET with a normal Host still works


def test_job_cap(base_url, monkeypatch):
    gate = threading.Event()
    monkeypatch.setattr(api, "start_run", lambda req, on_event=None, store_path=None: (gate.wait(5), {"specs": 0, "records": 0})[1])
    ok = {"workloads": ["spi"], "backends": ["iss"]}
    assert post(base_url + "/api/run", ok)[0] == 202
    assert post(base_url + "/api/run", ok)[0] == 202
    code, e = post(base_url + "/api/run", ok)
    assert code == 429
    gate.set()


def test_check_cached_by_mtime(base_url, monkeypatch):
    n = []
    real = api.check
    monkeypatch.setattr(api, "check", lambda recs: (n.append(1), real(recs))[1])
    get(base_url + "/api/check")
    get(base_url + "/api/check")
    assert len(n) == 1


def wait_job(base_url):
    for _ in range(400):
        jobs = get(base_url + "/api/jobs")
        if jobs and jobs[0]["state"] != "running":
            return jobs[0]
        time.sleep(0.05)
    raise AssertionError("job did not finish")


def test_real_job_end_to_end(tmp_path, monkeypatch):
    """A real run through task A's runner and fake backends, into a temp store the server was started on."""
    monkeypatch.undo()  # drop the adapter fakes: this test uses the real api
    store_file = tmp_path / "records.jsonl"
    srv = server.make_server(0, str(store_file))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:%d" % srv.server_address[1]
    try:
        assert get(url + "/api/records") == []
        body = {"sweep": {"base": {}, "vary": {"fifo_depth": [2, 4], "lane": [False, True]}},
                "workloads": ["count", "longrun"], "backends": ["fake", "fake2"]}
        code, j = post(url + "/api/run", body)
        assert code == 202, j
        job = wait_job(url)
        assert job["state"] == "done", job
        res = job["result"]
        assert res["records"] > 0 and res["status_counts"].get("ok")
        assert res["refused"] and "lane" in res["refused"][0]["reason"]  # fakes refuse lane=true
        assert res["skipped"] and "long" in res["skipped"][0]["reason"]
        assert job["events"] == job["total"] > 0
        recs = get(url + "/api/records")
        assert len(recs) == res["records"] and {r["level"] for r in recs} == {"iss", "rtl"}
        assert store_file.exists() and (tmp_path / "cache").exists()  # went to the server's store, not the default
        assert {f["verdict"] for f in get(url + "/api/check")} == {"match"}
        assert get(url + "/api/trace/" + recs[0]["id"])["id"] == recs[0]["id"]
    finally:
        srv.shutdown()
        srv.server_close()


def test_app_js_parses():
    quickjs = pytest.importorskip("quickjs")
    src = open(os.path.join(os.path.dirname(server.__file__), "static", "app.js"), encoding="utf-8").read()
    quickjs.Context().eval("(function(){" + src + "\n})")


def test_front_end_keeps_the_worst_finding_per_row():
    """app.js indexFindings: off > close > identity > match > one-sided, whatever the order (run under quickjs)."""
    import itertools
    import quickjs
    text = open(api.__file__.rsplit("/", 1)[0] + "/static/app.js").read()
    code = text.split("// BEGIN findIdx", 1)[1].split("// END findIdx", 1)[0]
    ctx = quickjs.Context()
    ctx.eval("function stable(o){return JSON.stringify(o);}\n" + code)

    def f(v, a, b):
        return {"config_hash": "c", "workload": "w", "workload_params": {}, "metric": "m", "verdict": v,
                "detail": "d", "a": {"backend": a}, "b": {"backend": b}}

    def index(findings):
        return json.loads(ctx.eval("JSON.stringify(indexFindings(%s))" % json.dumps(findings)))

    rows = [f("match", "iss", "rtl"), f("off", "analytic", "iss"), f("close", "iss", "iss_b"), f("identity", "a", "b")]
    for perm in itertools.permutations(rows):
        out = index(list(perm))
        assert len(out) == 1 and next(iter(out.values()))["verdict"] == "off"
        assert next(iter(out.values()))["detail"].startswith("analytic vs iss: ")
    assert next(iter(index([f("one-sided", "x", "y"), f("match", "iss", "rtl")]).values()))["verdict"] == "match"

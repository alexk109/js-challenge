"""iss / iss_b / rtl backends vs research/perf/b/perf.csv.  Each run is in its own subprocess: the fw target
install is process-global (the same reason the runner uses worker processes)."""
import csv
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
CSV = ROOT / "research" / "perf" / "b" / "perf.csv"

_RUNNER = """
import sys, json
from tools.bench import knobs, workloads as wl, backends
from tools.bench.types import RunSpec
b = backends.load(sys.argv[1])
spec = RunSpec(knobs.normalize(json.loads(sys.argv[3])), wl.workload(sys.argv[2]), b.name)
recs = b.run(spec)
print(json.dumps({"long": b.is_long(spec), "recs": [json.loads(r.to_json()) for r in recs]}))
"""


from tools.bench.runner import _worker_env  # noqa: E402


def run(backend: str, workload: str, partial: dict | None = None, timeout=300) -> list[dict]:
    p = subprocess.run([sys.executable, "-c", _RUNNER, backend, workload, json.dumps(partial or {})],
                       cwd=ROOT, env={**_worker_env(), "PYTHONPATH": str(ROOT)},   # the runner's PATH (cocotb's python)
                       capture_output=True, text=True, timeout=timeout)
    assert p.returncode == 0, p.stderr[-2000:]
    return json.loads(p.stdout.strip().splitlines()[-1])["recs"]


def csv_rows(protocol: str, target: str = "iss") -> list[dict]:
    with open(CSV, newline="") as fh:
        return [r for r in csv.DictReader(fh)
                if r["protocol"] == protocol and r["target"] == target and r["metric"] != "sweep_point"
                and not (protocol != "spi_host_port" and r["knob"] == "ticks_per_sck_bit")]   # host port: spi_host_port only


def as_pairs(recs: list[dict]):
    return {r["metric"]: (r["value"], r["unit"]) for r in recs if r["metric"] != "sweep_point"}


@pytest.mark.parametrize("proto", ["uart_tx", "spi", "i2c"])
def test_iss_matches_perf_csv(proto):
    recs = run("iss", proto)
    got = as_pairs(recs)
    want = csv_rows(proto)
    assert want
    for row in want:
        value, unit = got[row["metric"]]
        assert unit == row["unit"], row["metric"]
        if row["value"] == "n/a":
            assert value is None
            assert [r for r in recs if r["metric"] == row["metric"]][0]["status"] == "n/a"
        else:
            try:
                assert value == float(row["value"]), row["metric"]
            except ValueError:
                assert value == row["value"], row["metric"]
    assert set(got) == {r["metric"] for r in want}
    assert all(r["level"] == "iss" and r["backend"] == "iss" and r["status"] in ("ok", "n/a") for r in recs)


def test_host_port_matches_perf_csv():
    got = as_pairs(run("iss", "spi_host_port"))
    for row in csv_rows("spi_host_port"):
        assert got[row["metric"]] == (float(row["value"]), row["unit"])


@pytest.mark.parametrize("proto", ["uart_tx", "spi"])
def test_iss_b_equals_iss(proto):
    a, b = run("iss", proto), run("iss_b", proto)
    pa, pb = as_pairs(a), as_pairs(b)
    # ISS-B exposes no SM-cycle counter: that metric is n/a there (not 0), everything else is identical
    assert pb.pop("sm_cycles_per_bit")[0] is None and pa.pop("sm_cycles_per_bit")[0] is not None
    assert pa == pb
    assert {r["level"] for r in b} == {"iss"} and {r["backend"] for r in b} == {"iss_b"}


def test_refuses_non_mvp_config():
    from tools.bench import knobs, workloads as wl
    from tools.bench.backends import load

    w = wl.workload("spi")
    for name in ("iss", "iss_b"):
        why = load(name).supports(knobs.normalize({"fifo_depth": 8}), w)
        assert why and "fifo_depth" in why
        why = load(name).supports(knobs.normalize({"imem_tech": "sram2p"}), w)
        assert why and "imem_tech" in why
    rtl = load("rtl")
    assert rtl.supports(knobs.normalize({"imem_tech": "sram2p"}), w) is None
    assert rtl.supports(knobs.normalize({"n_engines": 3, "fifo_depth": 8, "imem_words": 64}), w) is None
    assert "engine_route" in rtl.supports(knobs.normalize({"n_engines": 1, "engine_route": True}), w)
    assert "host_link" in rtl.supports(knobs.normalize({"host_link": "qspi"}), w)
    assert "not RTL-targetable" in rtl.supports(knobs.normalize({}), wl.workload("uart_tx"))


def test_workloads_and_long_flag():
    from tools.bench import knobs, workloads as wl
    from tools.bench.backends import load
    from tools.bench.types import RunSpec

    assert wl.workload("i2c").params == {"speed": 13.0}
    assert wl.workload("uart_tx").params == {"speed": 6.25e6}
    assert wl.ladder("spi") == (1, 2, 4, 8, 16)
    mvp = knobs.normalize({})
    rtl = load("rtl")
    assert [w.id for w in rtl.workloads()] == ["spi", "swd", "ps2", "onewire", "spi_host_port"]
    longs = {w.id: rtl.is_long(RunSpec(mvp, w, "rtl")) for w in rtl.workloads()}
    assert longs == {"spi": False, "swd": True, "ps2": True, "onewire": True, "spi_host_port": True}
    assert not load("iss").is_long(RunSpec(mvp, wl.workload("i2c"), "iss"))
    assert load("iss").code_hash() != load("rtl").code_hash() != load("iss_b").code_hash()


def test_target_install_is_process_global():
    code = ("from tools.bench import knobs, workloads as wl; from tools.bench.backends import load\n"
            "from tools.bench.types import RunSpec\n"
            "s=lambda n: RunSpec(knobs.normalize({}), wl.workload('uart_tx'), n)\n"
            "load('iss').run(s('iss'))\n"
            "try: load('iss_b').run(s('iss_b'))\n"
            "except RuntimeError as e: print('refused', e)\n")
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env={**__import__("os").environ, "PYTHONPATH": str(ROOT)},
                       capture_output=True, text=True)
    assert "refused" in p.stdout, p.stderr[-800:]


def test_rtl_smoke_spi():
    recs = run("rtl", "spi", timeout=600)
    got = as_pairs(recs)
    assert {r["level"] for r in recs} == {"rtl"}
    iss = as_pairs(run("iss", "spi"))
    assert got["max_bit_rate"] == iss["max_bit_rate"]
    assert got["sm_cycles_per_bit"][0] is None      # the RTL harness exposes no SM counter: n/a, not 0


@pytest.mark.parametrize("backend", ["iss_p", "rtl"])
def test_spi_is_refused_at_one_engine(backend):
    """The spi rig drives two engines: supports() says so instead of the run failing on an unpack."""
    from tools.bench import knobs, workloads as wl
    from tools.bench.backends import load

    be = load(backend)
    cfg = knobs.normalize({"n_engines": 1})
    assert be.supports(cfg, wl.workload("spi")) == "spi needs 2 engines; n_engines=1"
    assert be.supports(cfg, wl.workload("swd")) is None
    assert be.supports(knobs.normalize({"n_engines": 2}), wl.workload("spi")) is None


def test_rtl_fifo_headroom_is_not_guarded_off_depth_4():
    """The rig measures headroom against the active fifo_depth, so no backend marks it n/a any more."""
    from tools.bench import knobs, workloads as wl
    from tools.bench.backends import load

    be, w = load("rtl"), wl.workload("spi")
    row = dict(metric="fifo_headroom_at_max_rate", value="3", unit="words", source="rig")
    for depth in (4, 8):
        cfg = knobs.normalize({"fifo_depth": depth})
        r = be._record(row, w, cfg, knobs.config_hash(cfg), "k", {}, True, "")
        assert r.status == "ok" and r.value == 3 and "hard-codes" not in r.note

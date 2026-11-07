"""Analytic backend (task C): MVP numbers, knob effects vs archsim's own model, refusals, record shape."""
import json
import time
from collections import Counter

import pytest

quickjs = pytest.importorskip("quickjs")

from tools.bench import knobs
from tools.bench.backends import REGISTRY, load
from tools.bench.backends import analytic as A
from tools.bench.types import RunSpec, Workload


@pytest.fixture(scope="module")
def be():
    return load("analytic")


@pytest.fixture(scope="module")
def data():
    return json.loads(A.INPUTS_JSON.read_text())


def cap19(data):
    return [w["id"] for w in data["workloads"] if "cap19" in w["tags"]]


def run(be, wl, kind="protocol", **cfg):
    return be.run(RunSpec(knobs.normalize(cfg), Workload(wl, kind), "analytic"))


def verdicts(be, ids, **cfg):
    return {w: next(r for r in run(be, w, **cfg) if r.metric == "verdict").value for w in ids}


def test_registered():
    assert REGISTRY["analytic"].endswith("AnalyticBackend")


def test_mvp_cap19(be, data):
    t = time.time()
    v = verdicts(be, cap19(data))
    assert Counter(v.values()) == {"served": 4, "impossible": 15}      # archsim's recorded MVP verdicts
    assert {w for w, x in v.items() if x == "served"} == {"uart_3m", "i2c_fmplus_ctrl", "usb_ls_device", "ws2812"}
    assert time.time() - t < 5


def test_record_shape(be):
    recs = run(be, "spi_target_10m")
    by = {r.metric: r for r in recs}
    assert set(by) == {"verdict", "binding_limit", "margin"}
    m = by["margin"]
    assert m.band[0] <= m.value <= m.band[1] and m.unit == "ns"
    assert by["verdict"].value == "impossible" and m.status == "ok" and m.value < 0
    assert all(r.level == "analytic" and r.backend == "analytic" for r in recs)
    assert all(r.config_hash == knobs.config_hash({}) and r.run_key for r in recs)
    assert m.method == "estimate"                                   # reflex rests on judgement inputs
    assert {"command", "commit", "dirty", "started", "duration_s"} <= set(m.provenance)


def test_method_derived_when_no_judgement(be):
    assert {r.method for r in run(be, "uart_3m")} == {"derived"}


def test_chip_records(be):
    by = {r.metric: r for r in run(be, "chip", kind="chip")}
    for m in ("area_um2", "core_clock_margin_mhz", "pins_fixed"):
        assert m in by
    assert by["area_um2"].value == 342472 and by["area_um2"].band[0] > by["area_um2"].band[1]   # [pess, opt]
    assert by["core_clock_margin_mhz"].value == pytest.approx(11.6)
    assert by["pins_fixed"].value == 6 and by["chip_verdict"].value == "pass"
    assert by["chip_verdict"].method == "estimate"


def _archsim_verdict(be, wl, **arch):
    be._js()
    return be._call("ArchSim.evaluateCell(Object.assign(ArchSim.defaultArch()," + json.dumps(arch) + "), "
                    + json.dumps(wl) + ", DATA).verdict")


@pytest.mark.parametrize("wl,knob,val,expect", [
    ("swd_host_10m", "host_link", "qspi", "served"),        # archsim: swd_host_10m -> qspi link
    ("eth10_tx", "host_link", "spi_fast", "served"),        # archsim: eth10_tx -> spi_fast link
])
def test_knob_flips_like_archsim_explore(be, wl, knob, val, expect):
    assert verdicts(be, [wl])[wl] == "impossible"
    got = verdicts(be, [wl], **{knob: val})[wl]
    assert got == expect == _archsim_verdict(be, wl, **{knob: val})


def test_knob_matches_archsim_on_all_cap19(be, data):
    for cfg in ({"host_link": "qspi"}, {"lane": True}, {"isa_xor": True}, {"n_engines": 3}):
        mine = verdicts(be, cap19(data), **cfg)
        ref = {w: _archsim_verdict(be, w, **cfg) for w in cap19(data)}
        assert mine == ref, cfg


def test_xor_moves_can_to_marginal(be):
    assert verdicts(be, ["can_1m"], isa_xor=True)["can_1m"] == "marginal"    # archsim's recorded MVP verdicts


@pytest.mark.parametrize("cfg,why", [
    ({"engine_route": True}, "engine_route"),
    ({"t_in": 2}, "t_in"),
    ({"imem_words": 16}, "imem_words"),
])
def test_refuses_unsupported(be, cfg, why):
    reason = be.supports(knobs.normalize(cfg), Workload("uart_3m", "protocol"))
    assert reason and why in reason


def test_refuses_unknown_workload_and_kind(be):
    cfg = knobs.normalize({})
    assert be.supports(cfg, Workload("nope", "protocol"))
    assert be.supports(cfg, Workload("x", "mix"))
    assert be.supports(cfg, Workload("uart_3m", "protocol")) is None
    assert be.supports(cfg, Workload("chip", "chip")) is None


def test_knob_support_covers_all(be):
    assert set(be.knob_support()) == set(knobs.BY_ID)


def test_workloads_and_mapping(be, data):
    ids = {w.id for w in be.workloads()}
    assert "chip" in ids and set(cap19(data)) <= ids
    assert set(A.ANALYTIC_TO_SIM) <= ids and set(A.NEAR_MATCHES) <= ids
    assert not set(A.ANALYTIC_TO_SIM) & set(A.NEAR_MATCHES)
    assert len(set(A.ANALYTIC_TO_SIM.values())) == len(A.ANALYTIC_TO_SIM)


def test_code_hash_includes_backend_source(be, monkeypatch):
    h = be.code_hash()
    real = A.Path.read_bytes
    monkeypatch.setattr(A.Path, "read_bytes", lambda self: real(self) + (b"x" if self.name == "analytic.py" else b""))
    assert be.code_hash() != h


def test_code_hash_stable(be):
    assert be.code_hash() == be.code_hash() and len(be.code_hash()) == 16

"""The pieces fit together (metric vocabulary, config handling, cross-level joins, refusals)."""
import csv
from pathlib import Path

import pytest

from .. import crosscheck, knobs, metrics, runner, workloads as wl
from ..importers import SOURCES, module
from ..types import Record, RunSpec, Workload

ROOT = Path(__file__).resolve().parents[3]


# ---------------------------------------------------------------- 1. metric vocabulary

def test_every_importer_metric_is_registered():
    for name in SOURCES:
        missing = {r.metric for r in module(name).load(ROOT) if r.metric not in metrics.METRICS}
        assert not missing, (name, sorted(missing))


def test_every_analytic_metric_is_registered():
    from ..backends.analytic import AnalyticBackend
    b = AnalyticBackend()
    mvp = knobs.normalize({})
    names = set()
    for w in b.workloads():
        names |= {r.metric for r in b.run(RunSpec(mvp, w, "analytic"))}
    assert names - set(metrics.METRICS) == set()
    assert {"max_bit_rate", "write", "read", "txf0_burst_4", "rxf0_burst_4"} <= names


def test_every_sim_metric_name_is_registered():
    with open(wl.PERF_CSV, newline="") as fh:
        names = {r["metric"] for r in csv.DictReader(fh)}
    assert names - set(metrics.METRICS) == set()


def test_speed_independent_metrics_are_registered():
    assert metrics.SPEED_INDEPENDENT <= set(metrics.METRICS)


# ---------------------------------------------------------------- 1b. external configs stay {}

def test_runner_keeps_external_config_empty():
    spec = RunSpec(knobs.normalize({}), Workload("all", "import"), "import")
    ext = Record(workload="probe", metric="area_um2", value=1.0, unit="um2", level="import", backend="import:synth",
                 method="imported", config={}, config_hash="external:probe")
    plain = Record(workload="probe", metric="area_um2", value=1.0, unit="um2", level="import", backend="import:synth",
                   method="imported")
    e = runner._finalize(ext, spec, "k", {})
    p = runner._finalize(plain, spec, "k", {})
    assert e.config == {} and e.config_hash == "external:probe"
    assert p.config == spec.config


def test_ui_labels_external_configs():
    from ..ui import api
    r = Record(workload="p", metric="area_um2", value=1.0, unit="um2", level="import", backend="import:synth",
               method="imported", config={}, config_hash="external:adpll")
    assert api.to_dict(r)["config_label"] == "external: adpll"


def test_imported_params_are_real_parameters_only():
    for name in ("flow", "synth", "fw", "mutate", "coverage"):
        assert all(r.workload_params == {} for r in module(name).load(ROOT)), name
    assert all(set(r.workload_params) <= {"seed"} for r in module("fuzz").load(ROOT))


def test_perf_b_sides_are_separate_backends():
    backends = {r.backend for r in module("perf_b").load(ROOT)}
    assert backends == {"import:perf_b:iss", "import:perf_b:rtl"}


# ---------------------------------------------------------------- 2/3. cross-level joins

def rec(workload, metric, value, level, backend, wp=None, unit="bit/s", status="ok", note="", ch="c"):
    return Record(workload=workload, metric=metric, value=value, unit=unit, level=level, backend=backend,
                  method="measured", status=status, config_hash=ch, workload_params=wp or {}, note=note)


def test_analytic_ids_align_to_sim_ids_and_join_at_the_reference_point():
    ref = wl.workload("uart_tx").params
    f = crosscheck.findings([
        rec("fw_uart_tx", "max_bit_rate", 6.25e6, "analytic", "analytic"),
        rec("uart_tx", "max_bit_rate", 6.25e6, "iss", "iss", ref),
        rec("uart_tx", "max_bit_rate", 123.0, "iss", "iss", {"speed": ref["speed"] / 2}),   # not the reference point
    ])
    joined = [x for x in f if x["a"] is not None and x["b"] is not None]
    assert [x["verdict"] for x in joined] == ["match"]
    assert joined[0]["workload"] == "uart_tx" and joined[0]["workload_params"] == ref


def test_params_are_not_ignored_for_other_metrics():
    ref = wl.workload("uart_tx").params
    f = crosscheck.findings([rec("fw_uart_tx", "margin", 5.0, "analytic", "analytic"),
                             rec("uart_tx", "margin", 5.0, "iss", "iss", ref)])
    assert [x["verdict"] for x in f] == ["one-sided", "one-sided"]


def test_near_matches_do_not_join():
    f = crosscheck.findings([rec("uart_3m", "max_bit_rate", 1.0, "analytic", "analytic"),
                             rec("uart_rx", "max_bit_rate", 1.0, "iss", "iss", wl.workload("uart_rx").params)])
    assert [x["verdict"] for x in f] == ["one-sided", "one-sided"]


def test_value_vs_na_is_one_sided_never_off():
    f = crosscheck.findings([rec("spi", "sm_cycles_per_bit", 8.33, "iss", "iss", {"speed": 8.0}, unit="cycles"),
                             rec("spi", "sm_cycles_per_bit", None, "iss", "import:perf_b:rtl", {"speed": 8.0},
                                 unit="cycles", status="n/a", note="rtl has no SM cycle counter")])
    assert len(f) == 1 and f[0]["verdict"] == "one-sided"
    assert f[0]["detail"].startswith("one-sided (other side n/a:") and "no SM cycle counter" in f[0]["detail"]


def test_each_backend_is_a_side():
    f = crosscheck.findings([rec("spi", "max_bit_rate", 1.0, "import", "import:perf_b:iss", {"speed": 8.0}),
                             rec("spi", "max_bit_rate", 1.0, "import", "import:perf_b:rtl", {"speed": 8.0})])
    assert [x["verdict"] for x in f] == ["match"]


def test_input_copy_is_identity_not_a_check():
    f = crosscheck.findings([
        rec("fw_spi_m0", "sm_cycles_per_bit", 8.0, "analytic", "analytic", note="input copied from perf-a (archsim ipu)"),
        rec("spi", "sm_cycles_per_bit", 8.0, "import", "import:perf_a", {"variant": "x"}),
        rec("spi", "sm_cycles_per_bit", 8.0, "import", "import:perf_a", {}),
    ])
    assert "identity" in {x["verdict"] for x in f}
    assert crosscheck.summary(f)["identity"] >= 1
    # against a real measurement it is an ordinary comparison
    g = crosscheck.findings([
        rec("fw_spi_m0", "sm_cycles_per_bit", 8.0, "analytic", "analytic", note="input copied from perf-a (archsim ipu)"),
        rec("spi", "sm_cycles_per_bit", 8.33, "iss", "iss", wl.workload("spi").params)])
    assert [x["verdict"] for x in g] == ["off"]


# ---------------------------------------------------------------- 6/7. refusals and hashes

def test_refusal_message_is_capped():
    msg = runner._offered("analytic", {f"w{i:02d}": None for i in range(40)})
    assert "30 more" in msg and "bench workloads --backend analytic" in msg and "w39" not in msg


def test_default_speed_raises_when_csv_lacks_the_protocol(monkeypatch):
    assert PERF_CSV_EXISTS()
    monkeypatch.setattr(wl, "accepted_speeds", lambda: {"spi": 8.0})
    assert wl.default_speed("spi") == 8.0
    with pytest.raises(ValueError, match="no iss max_bit_rate row"):
        wl.default_speed("uart_tx")


def PERF_CSV_EXISTS():
    return wl.PERF_CSV.exists()


@pytest.mark.parametrize("speed", [0, -1, 1e12, float("nan"), "fast", True])
def test_out_of_range_speed_is_a_refusal(speed):
    from ..backends import load
    why = load("iss").supports(knobs.normalize({}), Workload("uart_tx", "protocol", {"speed": speed}))
    assert why and "speed" in why


def test_out_of_range_host_port_ticks_is_a_refusal():
    from ..backends import load
    for t in (0, -3, 10 ** 6, 2.5):
        assert load("iss").supports(knobs.normalize({}), Workload(wl.HOST_PORT, "protocol", {wl.HOST_PORT_PARAM: t}))
    assert load("iss").supports(knobs.normalize({}), wl.workload(wl.HOST_PORT)) is None


def test_rtl_code_hash_covers_pdk_sram_models_only_for_sram(monkeypatch, tmp_path):
    from ..backends import load
    rtl = load("rtl")
    d = tmp_path / "ihp-sg13cmos5l" / "libs.ref" / "sg13cmos5l_sram" / "verilog"
    d.mkdir(parents=True)
    f = d / "RM_IHPSG13_2P_64x22_c2_bm_bist.v"
    f.write_text("a")
    monkeypatch.setenv("PDK_ROOT", str(tmp_path))
    sram, flop = knobs.normalize({"imem_tech": "sram2p"}), knobs.normalize({})
    h1, g1 = rtl.code_hash(sram), rtl.code_hash(flop)
    f.write_text("b")
    assert rtl.code_hash(sram) != h1          # the model file changed: sram2p results are stale
    assert rtl.code_hash(flop) == g1          # flop builds do not use it
    assert load("iss").code_hash() == load("iss").code_hash()


def test_analytic_chip_area_joins_flow_mvp_as_identity():
    f = crosscheck.findings([
        rec("chip", "area_um2", 342472.0, "analytic", "analytic", unit="um2", note="input copied from flow (K10)"),
        rec("mvp", "area_um2", 342472.0, "import", "import:flow", unit="um2")])
    assert [x["verdict"] for x in f] == ["identity"] and f[0]["workload"] == "mvp"
    g = crosscheck.findings([rec("chip", "area_um2", 400000.0, "analytic", "analytic", unit="um2"),
                             rec("mvp", "area_um2", 342472.0, "import", "import:flow", unit="um2")])
    assert [x["verdict"] for x in g] == ["off"]


def test_ceiling_bound_max_rate_is_identity_and_cycles_bound_is_a_prediction():
    from ..backends.analytic import AnalyticBackend
    b = AnalyticBackend()
    mvp = knobs.normalize({})
    notes = {}
    for w in ("fw_i2c", "fw_uart_tx", "fw_onewire", "fw_swd"):
        wk = [x for x in b.workloads() if x.id == w][0]
        notes[w] = [r.note for r in b.run(RunSpec(mvp, wk, "analytic")) if r.metric == "max_bit_rate"][0]
    assert all("input copied from archsim rate_ceiling_hz" in notes[w] for w in ("fw_i2c", "fw_onewire"))
    assert not any("input copied" in notes[w] for w in ("fw_uart_tx", "fw_swd"))
    ref = wl.workload("i2c").params
    f = crosscheck.findings([
        rec("fw_i2c", "max_bit_rate", 4e5, "analytic", "analytic", note=notes["fw_i2c"]),
        rec("i2c", "max_bit_rate", 3.2e5, "iss", "iss", ref)])
    assert [x["verdict"] for x in f] == ["identity"]

"""Power and gate-level backends: report parsing, supports() logic, record shapes (all fast, no tools)."""
import json
import subprocess
from pathlib import Path

import pytest

from tools.bench import knobs, metrics
from tools.bench.backends import gatelevel as G, load, power as P
from tools.bench.types import Record, RunSpec, Workload

FIX = Path(__file__).resolve().parents[1] / "fixtures"
CHIP = Workload("chip", "chip", {})
IDLE = Workload("idle", "chip", {})


def test_parse_report_real_idle_typ():
    r = P.parse_report((FIX / "power_idle_typ.rpt").read_text())
    assert r["Total"]["total"] == pytest.approx(8.708258)
    assert r["Total"]["leakage"] == pytest.approx(0.03717829)
    assert r["Sequential"]["total"] == pytest.approx(6.329538)
    assert r["Clock"]["total"] == pytest.approx(2.375455)
    assert r["Macro"]["total"] == 0.0
    assert sum(r[g]["total"] for g in ("Sequential", "Combinational", "Clock", "Macro", "Pad")) == pytest.approx(r["Total"]["total"], rel=1e-4)


def test_parse_report_rejects_garbage():
    with pytest.raises(ValueError):
        P.parse_report("Error: no liberty")
    assert P.parse_macro_total((FIX / "power_macro_idle.txt").read_text()) == pytest.approx(1.4523)
    with pytest.raises(ValueError):
        P.parse_macro_total("nothing")


def test_supports_needs_a_flow(monkeypatch):
    be = load("power")
    monkeypatch.setattr(P, "find_source", lambda cfg: None)
    assert be.supports({}, IDLE) == "run the flow backend for this config first (long)"
    monkeypatch.setattr(P, "find_source", lambda cfg: P.Source("pwr-mvp", "runs/pwr-mvp/proj/runs/wokwi", "mvp", "research-mvp"))
    assert be.supports({}, IDLE) is None
    for w in P.WORKLOADS:
        assert be.supports({}, Workload(w, "chip", {})) is None
    assert "no params" in be.supports({}, Workload("idle", "chip", {"speed": 1}))
    assert be.supports({}, Workload("uart_rx", "protocol", {})) is not None
    assert be.supports({}, CHIP) is not None
    assert set(be.knob_support()) == set(knobs.BY_ID)
    assert be.is_long(RunSpec(knobs.normalize({}), IDLE, "power"))
    assert [w.id for w in be.workloads()] == list(P.WORKLOADS)
    assert be.level == "power"


def _fake_final(f):
    for d, name in (("odb", "a.odb"), ("nl", "a.nl.v"), ("spef/nom", "a.spef"), ("sdc", "a.sdc")):
        (f / d).mkdir(parents=True)
        (f / d / name).write_text("x")


def test_find_source_uses_main_checkout_runs_and_limits_research_run_to_mvp(monkeypatch, tmp_path):
    monkeypatch.setattr(P, "runs_root", lambda: tmp_path / "runs")
    assert P.find_source({}) is None
    cfg = knobs.normalize({"t_in": 2})
    name = f"bench-{knobs.config_hash(cfg)}"
    _fake_final(tmp_path / "runs" / name / "flow" / "proj" / "runs" / "wokwi" / "final")
    s = P.find_source({"t_in": 2})
    assert s.kind == "bench" and s.name == f"{name}/flow" and s.tag == name
    assert P.find_source({"t_in": 3}) is None                      # a different config has no flow
    _fake_final(tmp_path / "runs" / "pwr-mvp" / "proj" / "runs" / "wokwi" / "final")
    m = P.find_source({})
    assert m.kind == "research-mvp" and m.name == "pwr-mvp" and m.tag == "mvp"
    assert P.find_source({"imem_tech": "sram2p"}) is None


def _bench_layout(tmp_path, monkeypatch, cfg_set):
    """A fake main checkout: runs/bench-<hash>/flow/proj/runs/wokwi/final (the flow backend's layout) and a fake ROOT."""
    cfg = knobs.normalize(cfg_set)
    name = f"bench-{knobs.config_hash(cfg)}"
    _fake_final(tmp_path / "runs" / name / "flow" / "proj" / "runs" / "wokwi" / "final")
    monkeypatch.setattr(P, "runs_root", lambda: tmp_path / "runs")
    monkeypatch.setattr(P, "ROOT", tmp_path)
    monkeypatch.setattr(P, "HERE", tmp_path / "research" / "power")
    (tmp_path / "research" / "power").mkdir(parents=True)
    return cfg, name


def test_ensure_vcd_generates_from_the_run_netlist_and_surfaces_failures(monkeypatch, tmp_path):
    cfg, name = _bench_layout(tmp_path, monkeypatch, {})
    src = P.find_source({})
    assert src.tag == name and src.kind == "bench"
    calls = []

    def fake_run(cmd, **kw):
        calls.append(cmd)
        out = tmp_path / "research" / "power" / "sim_out" / cmd[1]
        out.mkdir(parents=True, exist_ok=True)
        (out / "idle.vcd").write_text("$end")
        (out / "idle.vcd.window").write_text("4000\n")
        return subprocess.CompletedProcess(cmd, 0, "PASS", "")
    monkeypatch.setattr(P.subprocess, "run", fake_run)
    vcd = P.ensure_vcd(src, "idle", False)
    assert vcd.name == "idle.vcd" and vcd.exists()
    assert calls[0][1:4] == [name, "yes", "0"] and calls[0][4].endswith("a.nl.v") and calls[0][5] == "idle"
    P.ensure_vcd(src, "idle", False)
    assert len(calls) == 1                                           # present: no second simulation
    monkeypatch.setattr(P.subprocess, "run", lambda cmd, **kw: subprocess.CompletedProcess(cmd, 0, "", "boom: icarus error"))
    with pytest.raises(RuntimeError, match="produced no VCD.*boom: icarus error"):
        P.ensure_vcd(src, "uart", False)


def test_power_report_failure_carries_the_log_tail(monkeypatch, tmp_path):
    _bench_layout(tmp_path, monkeypatch, {})
    src = P.find_source({})
    (tmp_path / "runs" / "x").mkdir(parents=True)
    (tmp_path / "runs" / "x" / "idle.nom_typ_1p20V_25C.log").write_text("[ERROR ORD-0001] cannot read liberty")

    class Fake:
        @staticmethod
        def power_mw(*a, **k):
            raise subprocess.CalledProcessError(1, ["research/power/run_power.sh"])
    monkeypatch.setattr(P, "research", lambda: Fake)
    with pytest.raises(RuntimeError, match="cannot read liberty"):
        P.power_report("idle", src, "nom_typ_1p20V_25C", False, "runs/x")


def test_run_on_a_bench_flow_layout(monkeypatch, tmp_path):
    """The backend end to end on runs/bench-<hash>/flow (not pwr-mvp), with the tools faked."""
    cfg, name = _bench_layout(tmp_path, monkeypatch, {})
    rpt = (FIX / "power_idle_typ.rpt").read_text()
    seen = {}
    monkeypatch.setattr(P, "ensure_vcd", lambda src, rw, sram: seen.setdefault("vcd", (src.tag, rw)))

    def fake_report(rw, src, corner, sram, outdir):
        seen.setdefault("rundir", src.rundir)
        d = tmp_path / outdir
        d.mkdir(parents=True, exist_ok=True)
        (d / f"{rw}.{corner}.rpt").write_text(rpt)
        return {"report": f"{outdir}/{rw}.{corner}.rpt"}
    monkeypatch.setattr(P, "power_report", fake_report)
    recs = P.Power().run(RunSpec(cfg, Workload("uart_tx", "chip", {}), "power"))
    by = {r.metric: r for r in recs}
    assert seen["vcd"] == (name, "uart") and seen["rundir"] == f"runs/{name}/flow/proj/runs/wokwi"
    assert by["power_mw_typ"].value == pytest.approx(8.7083, abs=1e-4)
    assert {r.workload for r in recs} == {"uart_tx"} and all(r.status == "ok" for r in recs)
    assert f"runs/{name}/flow" in by["power_mw_typ"].note


def test_runs_root_is_the_main_checkouts():
    rr = P.runs_root()
    assert rr.name == "runs" and (rr.parent.parent / ".git").exists()


def test_input_hash_changes_when_flow_artifacts_change(monkeypatch, tmp_path):
    import os
    from tools.bench import runner
    monkeypatch.setattr(P, "runs_root", lambda: tmp_path / "runs")
    assert P.input_hash_for({}) == "noflow"
    final = tmp_path / "runs" / "pwr-mvp" / "proj" / "runs" / "wokwi" / "final"
    _fake_final(final)
    h0 = P.input_hash_for({})
    assert h0 == P.input_hash_for({}) != "noflow"
    (final / "nl" / "a.nl.v").write_text("longer netlist")
    h1 = P.input_hash_for({})
    assert h1 != h0
    os.utime(final / "spef" / "nom" / "a.spef", (1, 1))
    assert P.input_hash_for({}) != h1
    spec = RunSpec(knobs.normalize({}), IDLE, "power")
    assert runner.code_hash_for(load("power"), spec).endswith(":" + P.input_hash_for({}))
    assert runner.code_hash_for(load("gl"), RunSpec(spec.config, CHIP, "gl")).endswith(":" + P.input_hash_for({}))


def _reports(sram=False):
    typ = P.parse_report((FIX / ("power_sram_idle_typ.rpt" if sram else "power_idle_typ.rpt")).read_text())
    return {"typ": typ, "slow": {"Total": dict(typ["Total"], total=6.91)}, "fast": {"Total": dict(typ["Total"], total=10.83)}}


def test_record_shapes_flop_build():
    cfg = knobs.normalize({})
    recs = P.make_records(IDLE, cfg, "k", {"command": "c"}, _reports(), None, False, "src")
    by = {r.metric: r for r in recs}
    assert set(by) == {"power_mw_typ", "power_mw_slow", "power_mw_fast", "clock_tree_mw", "seq_mw", "comb_mw", "leakage_mw", "macro_mw"}
    assert set(by) <= set(metrics.METRICS)
    assert by["power_mw_typ"].value == pytest.approx(8.7083, abs=1e-4) and by["power_mw_slow"].value == 6.91
    assert by["clock_tree_mw"].value == pytest.approx(2.3755, abs=1e-4)
    assert by["macro_mw"].value == 0.0 and by["macro_mw"].method == "measured"
    for r in recs:
        assert (r.level, r.backend, r.status, r.workload, r.config_hash) == ("power", "power", "ok", "idle", knobs.config_hash(cfg))
        assert r.unit == "mW" and r.method in ("measured", "estimate") and r.run_key == "k"
        Record.from_json(r.to_json())


def test_macro_power_is_an_estimate_with_a_note():
    cfg = knobs.normalize({"imem_tech": "sram2p"})
    recs = {r.metric: r for r in P.make_records(IDLE, cfg, "k", {}, _reports(True), 1.4523, True, "src")}
    m = recs["macro_mw"]
    assert m.method == "estimate" and m.value == pytest.approx(1.4523 + 0.0000548, abs=1e-4)
    assert "outside OpenROAD" in m.note and "macro_energy" in m.note
    assert "excludes SRAM macro dynamic" in recs["power_mw_typ"].note and recs["power_mw_typ"].method == "measured"
    inc = recs["power_mw_typ_incl_macro"]
    assert inc.method == "estimate" and inc.value == pytest.approx(recs["power_mw_typ"].value + 1.4523, abs=1e-3)
    assert "macro" in inc.note
    none = {r.metric: r for r in P.make_records(IDLE, cfg, "k", {}, _reports(True), None, True, "src")}
    assert none["macro_mw"].status == "n/a" and none["macro_mw"].value is None
    assert none["power_mw_typ_incl_macro"].status == "n/a"


def test_input_hash_sees_sub_second_mtime_change(tmp_path, monkeypatch):
    import os
    f = tmp_path / "final"
    (f / "odb").mkdir(parents=True)
    odb = f / "odb" / "x.odb"
    odb.write_text("a")
    src = P.Source("n", "x", "t", "bench")
    monkeypatch.setattr(P, "find_source", lambda cfg: src)
    monkeypatch.setattr(P.Source, "final", lambda self: f)
    st = odb.stat()
    os.utime(odb, ns=(st.st_atime_ns, 1_700_000_000_100_000_000))
    h1 = P.input_hash_for({})
    os.utime(odb, ns=(st.st_atime_ns, 1_700_000_000_900_000_000))
    assert P.input_hash_for({}) != h1


def test_missing_corner_is_na_not_zero():
    r = _reports()
    del r["fast"]
    recs = {x.metric: x for x in P.make_records(IDLE, knobs.normalize({}), "k", {}, r, None, False, "s")}
    assert recs["power_mw_fast"].status == "n/a" and recs["power_mw_fast"].value is None


def test_gl_results_parsing_and_supports(monkeypatch):
    xml = ('<testsuites><testsuite><testcase name="a"/><testcase name="b"><failure/></testcase>'
           '<testcase name="c"/><testcase name="d"><error/></testcase></testsuite></testsuites>')
    assert G.parse_results(xml) == {"passed": 2, "failed": 2}
    be = load("gl")
    monkeypatch.setattr(G, "find_source", lambda cfg: None)
    assert be.supports({}, CHIP) == "run the flow backend for this config first (long)"
    monkeypatch.setattr(G, "find_source", lambda cfg: object())
    assert be.supports({}, CHIP) is None and be.supports({}, IDLE) is not None
    assert be.is_long(RunSpec(knobs.normalize({}), CHIP, "gl")) and be.level == "gl"
    assert set(be.knob_support()) == set(knobs.BY_ID)


@pytest.mark.long
def test_power_idle_mvp_end_to_end():
    """MVP idle on the research pwr-mvp run: 3 OpenROAD report_power runs on the existing VCD; expects ~8.71 mW typ."""
    be = load("power")
    spec = RunSpec(knobs.normalize({}), IDLE, "power")
    if be.supports(spec.config, IDLE):
        pytest.skip("no finished MVP flow on this machine")
    recs = {r.metric: r for r in be.run(spec)}
    assert recs["power_mw_typ"].value == pytest.approx(8.71, abs=0.1)
    assert recs["power_mw_slow"].value < recs["power_mw_typ"].value < recs["power_mw_fast"].value
    assert recs["leakage_mw"].value < 0.1 and recs["macro_mw"].value == 0.0


@pytest.mark.skipif(not (Path(__file__).resolve().parents[3]/"research"/"power"/"tb_power.v").exists(), reason="power testbench not present; optional power flow")
def test_tb_vcd_path_buffer_fits_long_paths():
    """tb_power.v read the +vcd= path into 128 chars; deeply nested checkout paths are longer (silent 'Unable to open')."""
    import re
    tb = (P.ROOT / "research" / "power" / "tb_power.v").read_text()
    bits = int(re.search(r"reg \[(\d+):0\] vcdname", tb).group(1)) + 1
    assert bits // 8 >= 512


def test_testbenches_refuse_other_register_maps():
    """test/test.py and the power activity testbench speak the MVP map: on the wide 3-engine map gl went 0/4 and every
    power workload read as idle. They refuse instead; idle (no host access) and imem_tech (same map) still run."""
    from tools.bench.backends.power import regmap_refusal
    from tools.bench.types import Workload
    three = knobs.normalize({"n_engines": 3, "fifo_depth": 8, "engine_route": True})
    assert "MVP register map" in regmap_refusal(three, "x") and "n_engines=3" in regmap_refusal(three, "x")
    assert regmap_refusal(knobs.normalize({"imem_tech": "sram2p"}), "x") is None
    assert regmap_refusal(knobs.normalize({}), "x") is None
    assert "MVP register map" in load("gl").supports(three, Workload("chip", "chip", {}, ""))
    assert "MVP register map" in load("power").supports(three, Workload("uart_tx", "chip", {}, ""))
    idle = load("power").supports(three, Workload("idle", "chip", {}, ""))
    assert idle is None or "MVP register map" not in idle

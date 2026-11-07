"""Physical backends: rtlparams mapping, flow config generation, metrics parser, synth smoke."""
import json
import shutil
from pathlib import Path

import pytest

from tools.bench import knobs, rtlparams
from tools.bench.backends import load, physical as P
from tools.bench.types import RunSpec, Workload

FIX = Path(__file__).resolve().parents[1] / "fixtures" / "flow_final_metrics_stub-sv.json"
CHIP = Workload("chip", "chip", {})


def test_rtl_build_mvp():
    b = rtlparams.rtl_build({})
    assert b["params"] == {"pio_chip": {"T_IN": 3}}
    assert b["defines"] == [] and b["extra_sources"] == [] and b["librelane"] == {}
    assert b["clock_period_ns"] == 20.0


def test_rtl_build_t_in_and_sram():
    b = rtlparams.rtl_build({"t_in": 2, "imem_tech": "sram2p"})
    assert b["params"]["pio_chip"]["T_IN"] == 2
    assert b["defines"] == ["IMEM_SRAM"]
    assert b["extra_sources"] == ["pio_imem_sram.sv"]
    assert "pdn_cfg.tcl" in b["extra_src"]
    assert set(b["librelane"]["MACROS"]) == {"RM_IHPSG13_2P_64x22_c2_bm_bist"}
    assert len(b["macros"]) == 2                      # one per engine instance
    assert not any(k.startswith("//") for k in b["librelane"])


def test_refusals_name_the_knobs():
    assert set(rtlparams.supported_knobs()) == {"t_in", "imem_tech", "f_core_mhz", "n_engines", "imem_words",
                                                "fifo_depth", "engine_route"}
    with pytest.raises(ValueError, match="lane=True"):
        rtlparams.rtl_build({"lane": True})
    for name in ("synth", "flow"):
        be = load(name)
        assert be.supports({}, CHIP) is None
        assert "lane=True" in be.supports({"lane": True}, CHIP)
        assert be.supports({"fifo_depth": 8, "n_engines": 3}, CHIP) is None
        assert "engine_route" in be.supports({"n_engines": 1, "engine_route": True}, CHIP)
        assert be.supports({}, Workload("uart_tx", "protocol", {"speed": 1})) is not None
        assert set(be.knob_support()) == set(knobs.BY_ID)
    assert load("synth").supports({"f_core_mhz": 40.0}, CHIP) is not None
    assert load("flow").supports({"f_core_mhz": 40.0}, CHIP) is None
    assert load("flow").is_long(RunSpec(knobs.normalize({}), CHIP, "flow"))
    assert not load("synth").is_long(RunSpec(knobs.normalize({}), CHIP, "synth"))


def test_apply_params_patches_only_the_default():
    text = (rtlparams.SRC / "pio_chip.sv").read_text()
    out = rtlparams.apply_params(text, {"T_IN": 2})
    assert "parameter int T_IN = 2," in out
    assert out.replace("T_IN = 2,", "T_IN = 3,") == text
    with pytest.raises(ValueError):
        rtlparams.apply_params(text, {"NOPE": 1})


def test_flow_config_keys():
    mvp = P.flow_config({})
    assert mvp["top"] == "tt_um_protocol_emulator" and mvp["tiles"] == "6x4"
    assert mvp["clock_period"] == 20.0
    ll = mvp["librelane"]
    assert ll["USE_SLANG"] is True and ll["RUN_POST_GRT_DESIGN_REPAIR"] is True   # mvp-slow's keys
    assert ll["RUN_MAGIC_DRC"] is False
    assert "VERILOG_DEFINES" not in ll and "extra_src" not in mvp
    assert not any(k.startswith("//") for k in mvp)
    s = P.flow_config({"imem_tech": "sram2p", "f_core_mhz": 40.0})
    assert s["clock_period"] == 25.0
    assert "pio_imem_sram.sv" in s["sources"] and s["sources"][:6] == mvp["sources"]
    assert s["librelane"]["VERILOG_DEFINES"] == ["IMEM_SRAM"] and "MACROS" in s["librelane"]
    assert s["librelane"]["RUN_MAGIC_DRC"] is False
    assert s["tag"] != mvp["tag"]


def test_parse_final_metrics_fixture():
    m = json.loads(FIX.read_text())
    v = P.parse_final_metrics(m, 20.0)
    assert v["stdcell_area_um2"] == pytest.approx(3311.28) and v["macro_area_um2"] == 0
    assert v["area_um2"] == pytest.approx(3311.28)
    assert v["cell_count"] == 228 and v["seq_cell_count"] == 23
    assert v["utilization_pct"] == pytest.approx(0.367, abs=1e-3)
    assert v["wns_ns_typ"] == pytest.approx(14.606579470487995)
    assert v["wns_ns_slow"] == pytest.approx(13.920270663499274)
    assert v["fmax_mhz_typ"] == pytest.approx(1000 / (20 - 14.6066), abs=0.1)
    assert v["fmax_mhz_slow"] == pytest.approx(1000 / (20 - 13.9203), abs=0.1)
    assert v["drc_errors"] == 0 and v["lvs_errors"] == 0


def test_parse_final_metrics_missing_is_none_not_zero():
    v = P.parse_final_metrics({}, 20.0)
    assert all(x is None for x in v.values())
    assert P.fmax_mhz(20.0, -9.895) == pytest.approx(33.5, abs=0.1)     # mvp slow-corner row of flow.csv


def test_metric_names_declared():
    produced = set(P.parse_final_metrics({}, 20.0)) | {"seq_area_um2", "comb_area_um2", "precheck_pass"}
    assert produced <= set(P.METRICS)
    try:
        from tools.bench import metrics
    except ImportError:
        return
    assert set(P.METRICS) <= set(metrics.METRICS)


def test_synth_smoke_mvp():
    """Yosys on the MVP chip top: ~20 s through tools/ll.  Cross-checked against known flow numbers."""
    if not (P.ROOT / "tools" / "ll").exists() or not Path.home().joinpath("pdk").exists():
        pytest.skip("toolchain not installed")
    recs = load("synth").run(RunSpec(knobs.normalize({}), CHIP, "synth"))
    v = {r.metric: r.value for r in recs}
    assert all(r.level == "synth" and r.method == "measured" and r.status == "ok" for r in recs)
    assert v["seq_cell_count"] == 2404                      # metrics/flow.csv mvp-slow seq_cells
    assert 250_000 < v["area_um2"] < 280_000                # post-PnR stdcell area is 342,960 (buffers, CTS)
    assert v["area_um2"] == pytest.approx(v["stdcell_area_um2"] + v["macro_area_um2"], abs=0.1)
    assert v["stdcell_area_um2"] == pytest.approx(v["seq_area_um2"] + v["comb_area_um2"], abs=0.1)
    shutil.rmtree(P._runs_dir(knobs.normalize({})) / "synth", ignore_errors=True)


@pytest.mark.skipif(not (Path(__file__).resolve().parents[3]/"tt").exists(), reason="needs the optional tt-support-tools checkout (tt/) and LibreLane")
def test_flow_config_loads_in_librelane():
    """Generated project + config load and pass LibreLane's first step (Verilator lint), a few seconds."""
    if not Path.home().joinpath("pdk").exists():
        pytest.skip("toolchain not installed")
    cfg = knobs.normalize({"imem_tech": "sram2p", "t_in": 2})
    wd, cname, _ = P.prepare_flow(cfg, P._runs_dir(cfg) / "flow")
    assert P.run_librelane(wd, cname, to="Verilator.Lint") == 0
    assert "T_IN = 2," in (wd / "src" / "pio_chip.sv").read_text()


def test_structural_knobs_map_to_top_level_params():
    b = rtlparams.rtl_build({"n_engines": 3, "fifo_depth": 8, "imem_words": 64, "engine_route": True})
    assert b["params"]["tt_um_protocol_emulator"] == {"N_ENGINES": 3, "IMEM_WORDS": 64, "FIFO_DEPTH": 8,
                                                      "ENGINE_ROUTE": 1}
    assert "tt_um_protocol_emulator" not in rtlparams.rtl_build({})["params"]      # the MVP source is untouched
    text = (rtlparams.SRC / "tt_um_protocol_emulator.v").read_text()
    out = rtlparams.apply_params(text, b["params"]["tt_um_protocol_emulator"])
    assert "parameter integer N_ENGINES = 3," in out and "parameter integer ENGINE_ROUTE = 1" in out
    with pytest.raises(ValueError, match="sram2p"):
        rtlparams.rtl_build({"imem_tech": "sram2p", "imem_words": 64})


def test_sim_build_tags_by_parameters():
    assert rtlparams.sim_build({})["tag"] == "" and rtlparams.sim_build({})["gflags"] == []
    a = rtlparams.sim_build({"fifo_depth": 8, "f_core_mhz": 40.0})       # f_core is not an RTL parameter
    b = rtlparams.sim_build({"fifo_depth": 8})
    assert a["tag"] == b["tag"] and a["tag"].startswith("cfg-")
    assert a["gflags"] == ["-GFIFO_DEPTH=8"] and a["pflags"] == ["-Ppio_chip.FIFO_DEPTH=8"]
    assert rtlparams.sim_build({"fifo_depth": 16})["tag"] != a["tag"]


def test_sram_flow_config_escapes_macro_brackets():
    """phys/cfg/mvp-sram.json dies in floorplan on unescaped g_eng[0]; the generated sram2p config must not."""
    import re
    raw = rtlparams.rtl_build({"imem_tech": "sram2p"})["librelane"]["PDN_MACRO_CONNECTIONS"]
    assert any("g_eng[0]" in c for c in raw)                       # the committed config has the bug
    conns = P.flow_config({"imem_tech": "sram2p"})["librelane"]["PDN_MACRO_CONNECTIONS"]
    assert len(conns) == len(raw) == 4
    assert not any("[" in c.split()[0] for c in conns)
    assert conns[0].startswith("u_chip.g_eng.{1,2}0.{1,2}.u_eng.g_imem_sram.u_imem.u_imem VPWR VGND VDD! VSS!")
    assert conns[3].split()[1:] == raw[3].split()[1:]               # only the instance is rewritten
    assert re.fullmatch(conns[0].split()[0], r"u_chip.g_eng\[0\].u_eng.g_imem_sram.u_imem.u_imem")
    assert re.fullmatch(conns[0].split()[0], "u_chip.g_eng[0].u_eng.g_imem_sram.u_imem.u_imem")
    assert "PDN_MACRO_CONNECTIONS" not in P.flow_config({})["librelane"]

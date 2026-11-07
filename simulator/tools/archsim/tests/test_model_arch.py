"""Task 4: model core, PLAN Appendix C.1 (link, pins, area, fmax) and the vertex / record machinery."""
import re
import pytest
from conftest import ROOT

REL = 1e-3


def D(js, arch="ArchSim.defaultArch()"):
    return js("ArchSim.derive(" + arch + ", DATA)")


def A(**kw):
    import json
    return "Object.assign(ArchSim.defaultArch(), " + json.dumps(kw) + ")"


def ap(x, rel=REL):
    return pytest.approx(x, rel=rel)


def test_t_and_link(js):
    d = D(js)
    assert d["T"] == 20
    lk = d["link"]
    assert lk["write_MBps"]["nom"] == ap(0.6993)
    assert lk["write_MBps"]["pess"] == ap(0.6431)
    assert lk["write_MBps"]["opt"] == ap(0.7246)
    assert lk["read_MBps"]["nom"] == ap(0.5714)
    assert lk["read_MBps"]["pess"] == ap(0.5333)
    assert lk["read_MBps"]["opt"] == ap(0.5882)
    assert lk["write_window_1"] == 324 and lk["write_window_B"] == 1092 and lk["read_window_1"] == 388
    assert lk["h_write_1"] == 321
    assert js("ArchSim.hWrite(DATA.links.spi, 28)") == 7233
    assert js("ArchSim.writeWindow(DATA.links.spi, 4)") == 1092
    assert lk["t_host_loop"]["nom"] == 15240
    assert lk["both_MBps"]["nom"] == ap(1 / (1 / 0.6993 + 1 / 0.5714))


def test_pins(js):
    d = D(js)
    assert d["pins"]["total_avail"] == 18 == d["records"]["pins_total_avail"]["value"]
    assert d["pins"]["tristate_avail"] == 8
    assert D(js, A(host_link="qspi", pins_reserved=2))["pins"]["total_avail"] == 24 - 12 - 2


def test_area_mvp(js, data):
    a = D(js)["area"]
    assert abs(a["total"]["nom"] - 342472) <= 1
    assert a["util_pct"]["nom"] == ap(37.95, 1e-3)
    core = data["core_area_um2"]["6x4"]
    assert a["core"] == core
    # die_area margin (limit - area): nom 289,220 / pess 156,769 / opt 408,350 (C.1)
    lim = a["limit"]
    assert lim["nom"] - a["total"]["nom"] == ap(289220)
    assert lim["pess"] - a["total"]["pess"] == ap(156769)
    assert lim["opt"] - a["total"]["opt"] == ap(408350)


def test_area_pess_opt_pnr_only(js, data):
    """C.1 prints pess/opt area 370,681 / 307,136; that is S * pnr_factor hi/lo ONLY (blocks at nominal).
    The model's vertex also moves the pinmux and host-port bands, which is what the C.1 die-margin numbers
    require (384,681 / 295,536). Both facts are asserted here; see the Task 4 report."""
    S = D(js)["area"]["synth"]["nom"]
    assert S == 264772
    assert S * data["params"]["pnr_factor"]["hi"] == ap(370681)
    assert S * data["params"]["pnr_factor"]["lo"] == ap(307136)
    a = D(js)["area"]["total"]
    assert a["pess"] == ap(384681) and a["opt"] == ap(295536)


def test_fmax(js):
    f = D(js)["fmax"]
    assert f["typ"]["nom"] == ap(61.6) and f["slow"]["nom"] == ap(38.8)
    f = D(js, A(imem_tech="sram2p", imem_words=64))["fmax"]
    assert f["slow"]["pess"] == ap(32.92) and f["slow"]["nom"] == ap(38.8) and f["slow"]["opt"] == ap(38.8)
    assert f["typ"]["pess"] == ap(53.38) and f["typ"]["nom"] == ap(61.6)
    assert D(js, A(imem_words=64))["fmax"]["slow"]["nom"] == ap(38.06, 1e-3)
    assert D(js, A(timing_lever="rtl"))["fmax"]["typ"]["nom"] == ap(87.32)
    # core_clock margin at f = 60 is fmax(typ) - 60 = 1.6 MHz
    assert D(js, A(f_core_mhz=60))["fmax"]["sel"]["nom"] - 60 == ap(1.6, 1e-2)


def test_area_presets(js, data):
    expect = {"mvp": 342472, "stack_a": 513066, "stack_b": 397607, "stack_c": 438018, "stack_d": 392083,
              "stretch_usb_sd": 330630, "wide_host": 497578}
    for pid, v in expect.items():
        d = js("ArchSim.derive(ArchSim.applyPreset('%s', DATA), DATA)" % pid)
        assert d["area"]["total"]["nom"] == ap(v), pid
    d = js("ArchSim.derive(ArchSim.applyPreset('levers_60', DATA), DATA)")
    assert d["fmax"]["typ"]["nom"] > 60  # the levers_60 core_clock margin itself is a Task 5 limit


def test_knobs_and_presets(js, data):
    knobs = js("ArchSim.KNOBS")
    assert {k["id"] for k in knobs} | {"param_overrides"} == set(js("ArchSim.defaultArch()"))
    for k in knobs:
        assert k["label"] and k["group"] and "default" in k
    for p in data["presets"]:
        arch = js("ArchSim.applyPreset('%s', DATA)" % p["id"])
        assert js("ArchSim.checkArch(ArchSim.applyPreset('%s', DATA))" % p["id"]) == []
        for key, val in p["arch"].items():
            assert arch[key] == val
    with pytest.raises(Exception):
        js("ArchSim.applyPreset('nope', DATA)")
    with pytest.raises(Exception):
        D(js, A(imem_words=48))


def test_resolve_params_env_and_overrides(js, data):
    p = js("ArchSim.resolveParams(ArchSim.defaultArch(), DATA)")
    assert p["t_pad_rt_ns"]["nom"] == 20 and p["t_pad_rt_ns"]["method"] == "literature"
    p = js("ArchSim.resolveParams(%s, DATA)" % A(env="iss"))
    for i in ["t_pad_rt_ns", "t_skew_ns", "t_setup_ext_ns", "sync_phase_cyc", "t_clk_jitter_ns"]:
        assert (p[i]["lo"], p[i]["nom"], p[i]["hi"]) == (0, 0, 0)
    assert p["t_mcu_txn_ns"]["nom"] == 1000
    p = js("ArchSim.resolveParams(%s, DATA)" % A(param_overrides={"t_mcu_txn_ns": {"nom": 500}}))
    assert (p["t_mcu_txn_ns"]["lo"], p["t_mcu_txn_ns"]["nom"], p["t_mcu_txn_ns"]["hi"]) == (200, 500, 3000)
    d = D(js, A(param_overrides={"t_mcu_txn_ns": {"lo": 1000, "nom": 1000, "hi": 1000}}))
    assert d["link"]["write_MBps"]["pess"] == d["link"]["write_MBps"]["nom"]
    # the base data is not mutated
    assert js("DATA.params.t_mcu_txn_ns.nom") == 1000


def test_vertex(js):
    v = js("ArchSim.vertex({a:{lo:1,nom:2,hi:4}, b:{lo:3,nom:3,hi:3}, c:{lo:0,nom:1,hi:2}},"
           " function(x){return x.a*x.b - x.c*x.c;}, 'high')")
    assert v["nom"] == 5 and v["pess"] == 3 * 1 - 4 and v["opt"] == 12
    assert v["pess_direction"] == {"a": "lo", "b": "none", "c": "hi"}
    v = js("ArchSim.vertex({a:{lo:1,nom:2,hi:4}}, function(x){return x.a;}, 'low')")
    assert (v["pess"], v["opt"], v["pess_direction"]) == (4, 1, {"a": "hi"})
    v = js("ArchSim.vertex({a:{lo:1,nom:2,hi:4}}, function(x){return 7;}, 'high')")
    assert v["pess"] == v["nom"] == v["opt"] == 7 and v["pess_direction"] == {"a": "none"}


def test_interval_order(js, data):
    for pid in [p["id"] for p in data["presets"]]:
        d = js("ArchSim.derive(ArchSim.applyPreset('%s', DATA), DATA)" % pid)
        a = d["area"]["total"]
        assert a["opt"] <= a["nom"] <= a["pess"]
        for c in ("typ", "slow"):
            f = d["fmax"][c]
            assert f["pess"] <= f["nom"] <= f["opt"]
        w = d["link"]["write_MBps"]
        assert w["pess"] <= w["nom"] <= w["opt"]


def test_other_links_and_buffers(js):
    d = D(js, A(host_link="nibble", deep_fifo_words=1024, deep_fifo_dir="tx"))
    assert d["buffers"] == {"fifo_bytes": 16, "buf_tx_bytes": 16 + 4096, "buf_rx_bytes": 16}
    assert d["link"]["write_MBps"]["nom"] == ap(50 * 0.5 * 0.9)
    assert d["link"]["t_host_loop"]["nom"] == ap(60 + 3 * 20)
    d = D(js, A(host_link="byte_load_only"))
    assert d["link"]["write_MBps"]["nom"] == 0 and d["link"]["t_host_loop"]["nom"] == float("inf")
    d = D(js, A(host_link="qspi"))
    assert d["link"]["write_window_1"] == 2 + 4 * (2 + 8) + 2
    d = D(js, A(capture="adpll", adpll_mult=6, ddr_io=True))
    assert d["T_tx"] == 10 and d["T_rx"] == ap(10 / 6)
    d = D(js, A(fine_edge="dtc"))
    assert d["T_tx"]["nom"] == 0.069 and d["T_rx"]["nom"] == 0.069 and d["T_tx"]["pess"] == 0.11
    assert D(js, A(die="8x4"))["area"]["core"] == 1208173
    assert D(js, A(n_engines=6))["area"]["total"]["nom"] > D(js, A(n_engines=4))["area"]["total"]["nom"]
    assert D(js, A(imem_tech="latch", imem_words=64))["area"]["total"]["nom"] > 0


def _walk(rec, ids, data, js, bad):
    for t in rec["terms"]:
        ok = js("ArchSim.refResolves(%s, DATA, %s)" % (__import__("json").dumps(t["ref"]), __import__("json").dumps(ids)))
        if not ok:
            bad.append((rec["id"], t["sym"], t["ref"]))


def test_records_resolve(js, data):
    import json
    for pid in [p["id"] for p in data["presets"]] + ["x"]:
        arch = "ArchSim.applyPreset('%s', DATA)" % pid if pid != "x" else A(fine_edge="dtc", host_link="byte", deep_fifo_words=64, deep_fifo_dir="both", capture="adpll")
        d = js("ArchSim.derive(%s, DATA)" % arch)
        ids = list(d["records"])
        bad = []
        for r in d["records"].values():
            assert r["formula"] and r["method"] == "formula" and r["label"] and r["unit"] is not None
            assert "id" in r and "value" in r and "terms" in r and "pess_direction" in r
            _walk(r, ids, data, js, bad)
        assert bad == [], (pid, bad)
    d = D(js)
    for k in ["T", "link_write_MBps", "link_read_MBps", "t_host_loop", "pins_total_avail", "fmax_slow", "fmax_typ", "area_total", "util_pct"]:
        assert k in d["records"]


def test_json_infinity(js):
    assert js("ArchSim.toJSON({a: Infinity, b: [-Infinity, 1]})") == {"a": float("inf"), "b": [float("-inf"), 1]}


def test_no_forbidden_apis():
    src = (ROOT / "model/archsim-model.js").read_text()
    for bad in ["Math.random", "Date", "document.", "window.", "require(", "import "]:
        assert bad not in src, bad

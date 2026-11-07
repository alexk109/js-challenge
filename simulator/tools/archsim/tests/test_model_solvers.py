"""Task 6: solvers (maxRate, sweep, pareto, mix, rxTolerance, legacy19) and the diagram data (timeline, edgeGrid,
streamTrace, blockDiagram). PLAN Appendix C.4 plus the model-level halves of the Task 9 and Task 10 acceptance tests."""
import importlib.util
import json
import pytest
from conftest import ROOT

REL = 1e-3
INF = float("inf")
A_MVP = "ArchSim.defaultArch()"


def ap(x, rel=REL, abs_=1e-9):
    return pytest.approx(x, rel=rel, abs=abs_)


def A(**kw):
    return "Object.assign(ArchSim.defaultArch(), " + json.dumps(kw) + ")"


def max_rate(js, wl, arch=A_MVP, vertex="nom", opts="undefined"):
    return js("ArchSim.maxRate(%s, '%s', DATA, '%s', %s)" % (arch, wl, vertex, opts))


# ------------------------------------------------------------------ C.4 max rate
def test_max_rate_iss(js):
    r = max_rate(js, "spi_target_scalable", A(env="iss"))
    assert r["rate_hz"] == ap(5.0e6)
    assert r["binding"] == "reflex"


def test_max_rate_silicon_vertices(js):
    assert max_rate(js, "spi_target_scalable")["rate_hz"] == ap(3.6496e6)
    assert max_rate(js, "spi_target_scalable", vertex="pess")["rate_hz"] == ap(3.125e6)
    assert max_rate(js, "spi_target_scalable", vertex="opt")["rate_hz"] == ap(4.4643e6)


def test_max_rate_lane(js):
    r = max_rate(js, "spi_target_scalable", A(lane=True))
    assert r["rate_hz"] == ap(17.092e6)
    assert r["binding"] == "reflex"


def test_max_rate_closed_form(js, data):
    """silicon nominal reflex: (T_IN + T_OUT + reflex_sm) T + sync T + pad + skew + setup = deadline = reflex_bits * unit"""
    P = {k: v["nom"] for k, v in data["params"].items()}
    wl = next(w for w in data["workloads"] if w["id"] == "spi_target_scalable")
    T = 20.0
    reflex = (data["iss"]["T_IN"] + data["iss"]["T_OUT"] + wl["reflex_sm"]) * T + P["sync_phase_cyc"] * T \
        + P["t_pad_rt_ns"] + P["t_skew_ns"] + P["t_setup_ext_ns"]
    assert max_rate(js, "spi_target_scalable")["rate_hz"] == ap(1e9 / (reflex / wl["reflex_bits"]))


def test_max_rate_edge_cases(js):
    r = max_rate(js, "uart_3m", A(n_engines=1))              # the engines limit fails at any rate
    assert r["rate_hz"] == 0 and r["binding"] == "engines"
    r = max_rate(js, "uart_3m", opts="{only: ['pins']}")     # a rate-independent limit alone never binds
    assert r["rate_hz"] == 1e9 and r["binding"] is None
    # fixed_clock is skipped by default: RMII at 60 MHz would otherwise fail at every rate
    r = max_rate(js, "rmii100_tx", A(f_core_mhz=60), opts="{only: ['cycles', 'fixed_clock']}")
    assert r["rate_hz"] > 0 and r["binding"] == "cycles"
    r = max_rate(js, "rmii100_tx", A(f_core_mhz=60), opts="{only: ['cycles', 'fixed_clock'], skip: []}")
    assert r["rate_hz"] == 0 and r["binding"] == "fixed_clock"


def test_max_rate_is_a_boundary(js, data):
    """just below the returned rate every limit holds, just above the binding one fails (cell evaluated with the row rebuilt)"""
    for arch, wl in [(A_MVP, "spi_target_scalable"), (A(lane=True), "spi_target_scalable"), (A_MVP, "fw_uart_tx")]:
        r = max_rate(js, wl, arch)
        w = next(x for x in data["workloads"] if x["id"] == wl)
        for f, ok in [(0.999, True), (1.001, False)]:
            cell = js("ArchSim.evaluateCell(%s, Object.assign({}, DATA.workloads.find(w => w.id === '%s'), {rate_hz: %r}), DATA, {})"
                      % (arch, wl, r["rate_hz"] * f))
            bad = [x["id"] for x in cell["limits"] if x["value"]["nom"] < 0 and x["id"] != "fixed_clock"]
            assert (bad == []) == ok, (wl, f, bad)


# ------------------------------------------------------------------ rxTolerance
def test_rx_tolerance_k18(js, data):
    t = js("ArchSim.rxTolerance(%s, 'usb_ls_host_fw_v2', DATA)" % A(env="iss"))
    assert t == ap(0.4896, rel=1e-3)
    w = next(x for x in data["workloads"] if x["id"] == "usb_ls_host_fw_v2")
    unit = 1e9 / w["rate_hz"]
    assert t == ap(100 * (unit / 2 - 20.0) / (w["packet_bits"] * unit))     # (unit/2 - T_rx) / (96 unit)
    assert js("ArchSim.rxTolerance(%s, 'spi_host_25m', DATA)" % A_MVP) is None   # no rx_eye on this row


def test_rx_tolerance_dpll_tracks(js):
    # the DPLL removes the drift term while the mismatch stays inside its tracking range, so the tolerance only grows
    base = js("ArchSim.rxTolerance(%s, 'usb_ls_host_fw_v2', DATA)" % A_MVP)
    dpll = js("ArchSim.rxTolerance(%s, 'usb_ls_host_fw_v2', DATA)" % A(capture="dpll"))
    assert dpll >= base


# ------------------------------------------------------------------ legacy19
def _protocols():
    spec = importlib.util.spec_from_file_location("protocols_ref", ROOT.parents[1] / "research/models/protocols.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_legacy19_counts(js):
    L = js("ArchSim.legacy19(DATA)")
    assert L["counts"] == [5, 5, 12, 12, 15, 17]
    assert L["lever_deltas"] == [0, 5, 5, 0, 1]
    assert L["matches_expected"] is True and L["t0_count"] == 5
    assert [t["feasible"] for t in L["tiers"]] == L["counts"]


@pytest.mark.skip(reason="needs research/models/protocols.py, which is not part of this repository")
def test_legacy19_blockers_equal_python_check(js):
    """every row, every tier: the JS port returns the same blocker strings as protocols.py check()"""
    m = _protocols()
    L = js("ArchSim.legacy19(DATA)")
    cap = dict(m.BASE)
    for (name, delta), tier in zip(m.TIERS, L["tiers"]):
        cap = {**cap, **delta}
        assert tier["name"] == name
        for row in tier["rows"]:
            req = tuple(m.P[row["name"]])
            assert row["blockers"] == m.check(req, cap), (name, row["name"])
    assert len(L["tiers"][0]["rows"]) == 19


def test_legacy19_compare_with_archsim(js):
    L = js("ArchSim.legacy19(DATA, ArchSim.defaultArch())")
    cmp_ = L["compare"]
    assert len(cmp_) == 19 and all(r["wl"] for r in cmp_)
    assert {r["name"] for r in cmp_ if r["archsim"] == "served"} == {
        "UART 3 Mbaud", "I2C 1 MHz (Fm+)", "USB LS device", "WS2812 LEDs"}
    dis = [r for r in cmp_ if r["disagree"]]
    assert dis and all(r["binding"] for r in dis)


# ------------------------------------------------------------------ sweep / pareto
def test_sweep_f_core_max_rate_shape(js):
    """40..70 step 2 -> 16 points. The chip no longer caps the rate: strictly increasing over all 16 points; the chip
    check fails exactly from 62 MHz."""
    xs = list(range(40, 71, 2))
    S = js("ArchSim.sweep(%s, 'f_core_mhz', %s, 'max_rate:spi_target_scalable', DATA)" % (A_MVP, json.dumps(xs)))
    assert len(S) == 16 and [p["x"] for p in S] == xs
    nom = [p["y"]["nom"] for p in S]
    want = [3.0395, 3.1655, 3.2895, 3.4115, 3.5315, 3.6496, 3.7659, 3.8804, 3.9932, 4.1042, 4.2135, 4.3212, 4.4272, 4.5317, 4.6347, 4.7361]
    assert nom == [ap(w * 1e6, rel=1e-3) for w in want]
    assert all(b > a for a, b in zip(nom, nom[1:]))
    assert nom[xs.index(50)] == ap(3.6496e6) and S[xs.index(50)]["y"]["pess"] == ap(3.125e6) and S[xs.index(50)]["y"]["opt"] == ap(4.4643e6)
    assert [p["chip"] for p in S] == ["fail" if x >= 62 else p["chip"] for x, p in zip(xs, S)]
    assert all((p["chip"] == "fail") == (x >= 62) for x, p in zip(xs, S))
    assert S[xs.index(62)]["chip_driver"] == "core_clock"
    for p in S:
        assert p["y"]["pess"] <= p["y"]["nom"] <= p["y"]["opt"]


def test_sweep_metrics(js):
    S = js("ArchSim.sweep(%s, 'n_engines', [1, 2, 3], 'area_um2', DATA)" % A_MVP)
    a = [p["y"]["nom"] for p in S]
    assert a[1] == ap(342472, rel=1e-5) and a == sorted(a) and len(set(a)) == 3
    S = js("ArchSim.sweep(%s, 'host_link', ['spi', 'spi_fast', 'nibble'], 'link_write_MBps', DATA)" % A_MVP)
    assert S[0]["y"]["nom"] == ap(0.6993) and S[0]["y"]["pess"] == ap(0.6431) and S[1]["y"]["nom"] > S[0]["y"]["nom"]
    S = js("ArchSim.sweep(%s, 'f_core_mhz', [50, 60], 'fmax_typ', DATA)" % A_MVP)
    assert S[0]["y"]["nom"] == ap(61.6) == S[1]["y"]["nom"]
    S = js("ArchSim.sweep(%s, 'f_core_mhz', [50], 'util_pct', DATA)" % A_MVP)
    assert S[0]["y"]["nom"] == ap(37.95, rel=1e-4)
    S = js("ArchSim.sweep(%s, 'f_core_mhz', [50, 60], 'margin:spi_target_10m:reflex', DATA)" % A_MVP)
    assert (S[0]["y"]["nom"], S[0]["y"]["pess"], S[0]["y"]["opt"]) == (ap(-87), ap(-110), ap(-62))
    assert S[1]["y"]["nom"] > S[0]["y"]["nom"]
    S = js("ArchSim.sweep(%s, 'f_core_mhz', [50], 'margin:uart_3m:no_such_limit', DATA)" % A_MVP)
    assert S[0]["y"] == {"nom": None, "pess": None, "opt": None}
    S = js("ArchSim.sweep(%s, 'f_core_mhz', [50, 60], 'chip_margin:core_clock', DATA)" % A_MVP)
    assert S[0]["y"]["nom"] == ap(11.6, rel=1e-3) and S[1]["y"]["nom"] == ap(1.6, rel=1e-2)
    for bad in ("margin:uart_3m:core_clock", "margin:uart_3m:die_area", "margin:uart_3m:board_clock"):
        with pytest.raises(Exception):
            js("ArchSim.sweep(%s, 'f_core_mhz', [50], '%s', DATA)" % (A_MVP, bad))


def test_sweep_counts(js):
    S = js("ArchSim.sweep(%s, 'f_core_mhz', [50], 'served_count:cap19', DATA)" % A_MVP)[0]["y"]
    assert S["pess"] == 4 and S["pess"] <= S["nom"] <= S["opt"]
    ev = js("ArchSim.evaluateAll(%s, DATA, 'cap19')" % A_MVP)["counts"]
    assert S["pess"] == ev["served"]
    S = js("ArchSim.sweep(%s, 'f_core_mhz', [50], 'marginal_count:cap19', DATA)" % A_MVP)[0]["y"]
    assert S["nom"] == ev["marginal"] == 0
    # an enum knob sweeps the same way; stack-A-like die pressure turns served rows into marginal ones
    S = js("ArchSim.sweep(Object.assign(ArchSim.defaultArch(), {extras: ['echo_comparator', 'pin_shadow_witness24'], lane: true}), "
           "'imem_tech', ['flop', 'sram2p'], 'served_count:cap19', DATA)")
    assert [p["x"] for p in S] == ["flop", "sram2p"]
    with pytest.raises(Exception):
        js("ArchSim.sweep(%s, 'no_such_knob', [1], 'area_um2', DATA)" % A_MVP)


def test_pareto_function(js):
    pts = [{"area": 40, "served": 4}, {"area": 36, "served": 5}, {"area": 50, "served": 5}, {"area": 36, "served": 5}, {"area": 30, "served": 2}]
    assert js("ArchSim.pareto(%s)" % json.dumps(pts)) == [1, 3, 4]            # equal points do not dominate each other
    assert js("ArchSim.pareto([{x: 1, y: 1}, {x: 2, y: 1}])") == [0]
    assert js("ArchSim.pareto([])") == []


def test_pareto_ignores_chip_fail(js):
    pts = [{"area": 40, "served": 4, "chip": "pass"}, {"area": 30, "served": 9, "chip": "fail"}, {"area": 50, "served": 5, "chip": "marginal"}]
    assert js("ArchSim.pareto(%s)" % json.dumps(pts)) == [0, 2]      # the failing point neither enters nor dominates
    assert js("ArchSim.pareto(%s)" % json.dumps([dict(p, chip="pass") for p in pts])) == [1]


def test_pareto_presets(js, data):
    """(area %, cap19 served) over the presets with the chip verdict -> [stretch_usb_sd, wide_host] (Q1 default)"""
    expr = ("DATA.presets.map(p => { const a = ArchSim.applyPreset(p.id, DATA); "
            "return {id: p.id, area: ArchSim.derive(a, DATA).area.util_pct.nom, served: ArchSim.evaluateAll(a, DATA, 'cap19').counts.served, "
            "chip: ArchSim.chipCheck(a, DATA).verdict}; })")
    pts = js(expr)
    keep = js("ArchSim.pareto(%s)" % expr)
    ids = [pts[i]["id"] for i in keep]
    assert ids == ["stretch_usb_sd", "wide_host"]
    p = pts[keep[0]]
    assert (p["area"], p["served"]) == (ap(36.64, rel=1e-3), 5)
    assert pts[keep[1]]["chip"] == "marginal" and pts[keep[1]]["served"] == 9
    assert "mvp" not in ids
    mvp = next(x for x in pts if x["id"] == "mvp")
    assert mvp["area"] == ap(37.95, rel=1e-3) and mvp["served"] == 4


# ------------------------------------------------------------------ mix
def test_mix_pass(js):
    m = js("ArchSim.mix(%s, ['ws2812', 'i2c_fmplus_ctrl'], DATA)" % A_MVP)
    rows = {r["id"]: r for r in m["rows"]}
    assert m["pass"] and m["verdict"] == "served"
    assert rows["engines"]["need"] == 2 and rows["engines"]["margin"]["nom"] == 0
    assert rows["pins"]["need"] == 3 and rows["pins_tristate"]["need"] == 2
    assert rows["bw_write"]["need"] == ap(0.1) and rows["bw_both"]["need"] == ap(0.12)
    assert rows["bw_write"]["avail"]["nom"] == ap(0.6993)
    # half-duplex time share: 0.1 / write + 0.12 / both
    assert rows["link_share"]["need"]["nom"] == ap(0.1 / 0.6993007 + 0.12 / 0.3144654)


def test_mix_fail(js):
    m = js("ArchSim.mix(%s, ['uart_3m', 'i2c_fmplus_ctrl', 'sd4_host_12m5'], DATA)" % A_MVP)
    rows = {r["id"]: r for r in m["rows"]}
    assert not m["pass"] and m["verdict"] == "impossible"
    assert rows["engines"]["verdict"] == "impossible" and rows["engines"]["need"] == 4
    assert m["exempt"] == ["sd4_host_12m5"]                                   # rate_elastic: the host may slow the clock
    assert rows["bw_both"]["need"] == ap(0.42) and rows["bw_both"]["verdict"] == "impossible"
    assert rows["pins"]["need"] == 10 and rows["pins"]["verdict"] == "served"


# ------------------------------------------------------------------ timeline (Task 9 acceptance, model half)
def test_timeline_reflex_waterfall(js):
    t = js("ArchSim.timeline(%s, 'spi_target_10m', DATA)" % A_MVP)
    segs = t["segments"]
    assert t["kind"] == "reflex" and t["deadline"] == 50
    assert len(segs) == 7
    assert [s["label"] for s in segs] == ["pad in", "sync phase", "synchronizer", "decision", "output register", "pad out", "skew + external setup"]
    assert sum(s["dur"] for s in segs) == ap(137, abs_=0.01) and segs[-1]["t1"] == ap(137, abs_=0.01)
    assert all(a["t1"] == b["t0"] for a, b in zip(segs, segs[1:])) and segs[0]["t0"] == 0
    assert t["total"] == {"nom": ap(137), "pess": ap(160), "opt": ap(112)}
    assert t["margin"]["nom"] == ap(-87)
    assert t["lane"] is None and t["active"] == "sampled"
    assert t["ticks"] and t["T"] == 20
    assert all(abs((b - a) - 20) < 1e-9 for a, b in zip(t["ticks"], t["ticks"][1:]))
    assert all(s["ref"]["kind"] for s in segs)


def test_timeline_lane_path(js):
    t = js("ArchSim.timeline(%s, 'spi_target_10m', DATA)" % A(lane=True))
    assert t["active"] == "lane" and len(t["lane"]["segments"]) == 3
    assert sum(s["dur"] for s in t["lane"]["segments"]) == ap(29.253)
    assert t["lane"]["total"]["nom"] == ap(29.253) and t["lane"]["margin"]["nom"] == ap(20.747)
    assert sum(s["dur"] for s in t["segments"]) == ap(137)                    # the sampled path stays for comparison
    assert js("ArchSim.timeline(%s, 'uart_3m', DATA)" % A(lane=True))["lane"] is None


def test_timeline_response(js):
    t = js("ArchSim.timeline(%s, 'usb_ls_device', DATA)" % A_MVP)             # 11-byte answer fits the 16-byte TX buffer
    assert t["kind"] == "response" and t["on_chip"] and len(t["segments"]) == 6
    assert t["deadline"] == ap(5000, rel=1e-3) and t["margin"]["nom"] == ap(4830)
    assert sum(s["dur"] for s in t["segments"]) == ap(170)
    t = js("ArchSim.timeline(%s, 'usb_fs_device', DATA)" % A_MVP)             # 67 bytes do not: one host decision loop is added
    assert not t["on_chip"] and t["segments"][-1]["label"] == "host decision loop"
    assert t["segments"][-1]["dur"] == ap(15240)
    assert sum(s["dur"] for s in t["segments"]) == ap(t["deadline"] - t["margin"]["nom"])
    t = js("ArchSim.timeline(%s, 'uart_3m', DATA)" % A_MVP)
    assert t["kind"] == "none" and t["segments"] == []


# ------------------------------------------------------------------ edgeGrid
def test_edge_grid_usb_fs_48(js):
    g = js("ArchSim.edgeGrid(%s, 'usb_fs_tx', DATA, 8)" % A(f_core_mhz=48))
    assert g["applicable"] and len(g["ideal"]) == len(g["placed"]) == 8
    assert all(abs(p - i) < 1e-9 for p, i in zip(g["placed"], g["ideal"]))
    assert g["integer_grid"] and g["max_error_ns"] < 1e-9
    assert g["tick_ns"] == ap(1000 / 48) and g["tol_ns"] == 3.5
    assert g["limit"]["nom"] == ap(1.5)


def test_edge_grid_usb_fs_50(js):
    g = js("ArchSim.edgeGrid(%s, 'usb_fs_tx', DATA, 8)" % A_MVP)
    assert not g["integer_grid"] and 0 < g["max_error_ns"] <= 10 + 1e-9        # at most T/2
    assert g["limit"]["nom"] == ap(-6.5 + 0.0) or g["limit"]["nom"] < 0


def test_edge_grid_eth10_tx(js):
    g = js("ArchSim.edgeGrid(%s, 'eth10_tx', DATA, 8)" % A(f_core_mhz=60))
    assert g["integer_grid"] and g["max_error_ns"] < 1e-9                       # half-bit = 3.000 cycles at 60 MHz
    g = js("ArchSim.edgeGrid(%s, 'eth10_tx', DATA, 8)" % A_MVP)
    assert not g["integer_grid"] and g["max_error_ns"] == ap(10)
    g = js("ArchSim.edgeGrid(%s, 'eth10_tx', DATA, 8)" % A(fine_edge="dtc"))
    assert g["tick_ns"] == ap(0.069) and g["max_error_ns"] <= 0.069 / 2 + 1e-9


def test_edge_grid_not_applicable(js):
    g = js("ArchSim.edgeGrid(%s, 'eth10_rx', DATA, 8)" % A_MVP)
    assert not g["applicable"] and g["ideal"] == [] and g["reason"]
    g = js("ArchSim.edgeGrid(%s, 'spi_target_10m', DATA, 8)" % A_MVP)
    assert not g["applicable"]


# ------------------------------------------------------------------ streamTrace
def test_stream_trace_underrun(js):
    s = js("ArchSim.streamTrace(%s, 'eth10_tx', DATA, 5)" % A_MVP)
    assert s["kind"] == "tx" and s["buf_B"] == 16 and s["frame_B"] == 64 and not s["ok"]
    assert s["fill_B"][0] == 16 and s["fill_B"][-1] == 0
    assert s["fill_B"] == sorted(s["fill_B"], reverse=True) and len(s["t_us"]) == 5
    assert s["frame_us"] == ap(64 / 1.25) and s["t_us"][-1] == ap(s["frame_us"])
    assert s["event"]["kind"] == "underrun"
    assert s["event"]["t_us"] == ap(16 / (1.25 - 0.6993007))
    assert s["l_max_B"] == ap(64 - 27.683, rel=1e-3) == ap(s["event"]["bytes_streamed"])      # = host_stream margin + frame
    assert s["event"]["t_us_pess"] < s["event"]["t_us"] < s["event"]["t_us_opt"]


def test_stream_trace_ok_and_rx(js):
    s = js("ArchSim.streamTrace(%s, 'uart_3m', DATA, 10)" % A_MVP)           # link faster than the line: never underruns
    assert s["ok"] and s["event"]["kind"] is None and set(s["fill_B"]) == {16}
    s = js("ArchSim.streamTrace(%s, 'usb_fs_rx_monitor', DATA, 10)" % A_MVP)  # read direction: the RX FIFO fills and overflows
    assert s["kind"] == "rx" and s["fill_B"][0] == 0 and s["event"]["kind"] == "overflow" and s["fill_B"][-1] == 16
    s = js("ArchSim.streamTrace(%s, 'eth10_tx', DATA, 5)" % A(fifo_depth=8))   # a deeper FIFO moves the underrun later
    assert s["event"]["t_us"] == ap(32 / (1.25 - 4 * 8 / ((2 + 8 * (8 + 32 * 8) + 2 + 2) * 20 + 1000) * 1000))
    s = js("ArchSim.streamTrace(%s, 'sd4_host_12m5', DATA, 5)" % A_MVP)
    assert s["elastic"] and s["ok"]
    assert js("ArchSim.streamTrace(%s, 'logic_analyzer_100ps', DATA, 5)" % A_MVP)["kind"] == "none"


# ------------------------------------------------------------------ blockDiagram
def test_block_diagram_mvp(js):
    b = js("ArchSim.blockDiagram(%s, DATA)" % A_MVP)
    assert len(b["pins"]) == 24 and [p["pin"] for p in b["pins"]] == list(range(24))
    assert sum(p["use"] == "host" for p in b["pins"]) == 6
    assert sum(p["use"] == "free" for p in b["pins"]) == 18
    assert sum(p["use"] in ("reserved", "lane_clk") for p in b["pins"]) == 0
    assert [p["bank"] for p in b["pins"]][::8] == ["ui_in", "uo_out", "uio"]
    assert sum(p["tristate"] for p in b["pins"]) == 8
    assert sum(l["area"]["nom"] for l in b["area_lines"]) == ap(342472, rel=2e-6)         # waterfall ends at area_total
    assert b["area_total"]["nom"] == ap(342472, rel=2e-6)
    assert sum(l["area"]["pess"] for l in b["area_lines"]) == ap(384681, rel=1e-4)        # the erratum vertex (C.1)
    assert sum(l["area"]["opt"] for l in b["area_lines"]) == ap(295536, rel=1e-4)
    by = {x["id"]: x for x in b["blocks"]}
    assert [e["index"] for e in b["engines"]] == [0, 1] and by["engine_0"]["enabled"]
    assert by["link"]["link"]["write_MBps"]["nom"] == ap(0.6993) and by["link"]["area"]["nom"] > 0
    for off in ("lane", "deep_fifo", "adpll", "dpll", "ddr_io", "dtc", "crc", "echo_comparator"):
        assert by[off]["enabled"] is False and by[off]["area"] is None
    assert by["pinmux"]["enabled"] and by["pinmux"]["area"]["nom"] > 0
    assert b["core_area"] == 902417 and b["util_pct"]["nom"] == ap(37.95, rel=1e-4)


def test_block_diagram_options(js):
    b = js("ArchSim.blockDiagram(%s, DATA)" % A(lane=True, pins_reserved=3, deep_fifo_words=256, capture="adpll", extras=["fc_executor"]))
    by = {x["id"]: x for x in b["blocks"]}
    assert by["lane"]["enabled"] and by["deep_fifo"]["enabled"] and by["adpll"]["enabled"] and by["fc_executor"]["enabled"]
    uses = [p["use"] for p in b["pins"]]
    assert uses.count("reserved") == 3 and uses.count("lane_clk") == 1 and uses.count("host") == 6
    assert next(p for p in b["pins"] if p["use"] == "lane_clk")["bank"] == "ui_in"
    assert sum(l["area"]["nom"] for l in b["area_lines"]) == ap(b["area_total"]["nom"], rel=1e-9)
    assert by["lane"]["limits"] == ["reflex"]
    w = js("ArchSim.blockDiagram(%s, DATA)" % A(host_link="nibble"))
    assert sum(p["use"] == "host" for p in w["pins"]) == 8

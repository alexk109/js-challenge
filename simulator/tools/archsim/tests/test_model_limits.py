"""Task 5: limits, verdicts, derivation records. PLAN Appendix C.2 (cell margins), C.3 (MVP verdicts), the
monotonicity test of 3.2 and the "every ref resolves" test of 3.1."""
import json
import pytest
from conftest import ROOT

REL = 1e-3
INF = float("inf")


def ap(x, rel=REL):
    return pytest.approx(x, rel=rel, abs=1e-9)


def A(**kw):
    return "Object.assign(ArchSim.defaultArch(), " + json.dumps(kw) + ")"


def cell(js, wl, arch="ArchSim.defaultArch()", opts="{}"):
    return js("ArchSim.evaluateCell(%s, '%s', DATA, %s)" % (arch, wl, opts))


def lim(c, lid):
    for r in c["limits"]:
        if r["id"] == lid:
            return r
    raise KeyError(lid)


def trio(c, lid):
    v = lim(c, lid)["value"]
    return v["nom"], v["pess"], v["opt"]


def check3(c, lid, nom, pess, opt, rel=REL):
    got = trio(c, lid)
    assert got == (ap(nom, rel), ap(pess, rel), ap(opt, rel)), (lid, got)


# ------------------------------------------------------------------ C.2 cell margins
def test_reflex_spi_target_10m(js):
    c = cell(js, "spi_target_10m")
    check3(c, "reflex", -87, -110, -62)
    assert lim(c, "reflex")["pess_direction"]["t_pad_rt_ns"] == "hi"
    c = cell(js, "spi_target_10m", A(lane=True))
    check3(c, "reflex", 20.747, 6.408, 36.524)
    assert c["verdict"] == "impossible" and "host_stream" in c["blocked_by"] and "reflex" not in c["blocked_by"]
    ids = {n["id"] for n in c["na"]}
    assert {"cycles", "program_words", "rx_osr"} <= ids          # the lane serves the target: core limits n/a
    assert c["flags"]["lane_serves"] is True


def test_eth10_tx(js):
    c = cell(js, "eth10_tx")
    check3(c, "tx_edge", 1.0, 0.0, 1.0)
    check3(c, "host_stream", -27.683, -31.046, -25.931)
    assert lim(c, "host_stream")["unit"] == "B"
    assert c["blocked_by"] == ["host_stream"] and c["verdict"] == "impossible"


def test_usb_fs(js):
    check3(cell(js, "usb_fs_device"), "tx_edge", -8.5, -9.5, -6.5)
    check3(cell(js, "usb_fs_tx", A(f_core_mhz=48)), "tx_edge", 1.5, 0.5, 3.5)
    # forcing the +-T/2 snap on the integer grid gives back the unconditional budget.py error
    c = cell(js, "usb_fs_tx", A(f_core_mhz=48), "{force_snap: true}")
    assert trio(c, "tx_edge")[0] == ap(3.5 - 10.4166667 - 2)


def test_usb_ls_device(js):
    c = cell(js, "usb_ls_device")
    check3(c, "response", 4830, 4810, 4850)
    assert trio(c, "rx_eye")[0] == ap(173.333)
    assert c["verdict"] == "served"


def test_eth10_rx(js):
    assert trio(cell(js, "eth10_rx"), "rx_eye")[0] == ap(-8.52)
    assert trio(cell(js, "eth10_rx", A(capture="dpll")), "rx_eye")[0] == ap(1.5)


def test_misc_cells(js):
    assert trio(cell(js, "canfd_5m"), "cycles")[0] == ap(-2.0)
    check3(cell(js, "spi_host_25m"), "pad_toggle_out", 9.697, 0.0, 20.0)
    c = cell(js, "sd4_host_25m")
    check3(c, "round_trip", 6, -4, 16)
    assert c["verdict"] == "marginal" and c["blocked_by"] == []
    assert "host_stream" in {n["id"] for n in c["na"]}            # rate_elastic: the host may slow the clock
    nom, pess, opt = trio(cell(js, "uart_3m"), "host_stream")
    assert (nom, pess, opt) == (INF, ap(551.724), INF)


# ------------------------------------------------------------------ C.3 verdicts on the MVP
def verdicts(js, tag):
    a = js("(function(){var r = ArchSim.evaluateAll(ArchSim.defaultArch(), DATA, %s); return {by: r.by_verdict, counts: r.counts};})()"
           % json.dumps(tag))
    return a["by"], a["counts"]


def test_cap19_verdicts(js):
    by, counts = verdicts(js, "cap19")
    assert set(by["served"]) == {"uart_3m", "i2c_fmplus_ctrl", "usb_ls_device", "ws2812"}
    assert by["marginal"] == [] and len(by["impossible"]) == 15
    assert counts == {"served": 4, "marginal": 0, "impossible": 15}
    blocked = {w: set(cell(js, w)["blocked_by"]) for w in by["impossible"]}
    assert blocked["can_1m"] == {"isa_xor"}                        # Q6: an autonomous ACK needs the CRC unit
    assert blocked["eth10_tx"] == {"host_stream"}
    assert blocked["dvi_250m"] == {"pad_toggle_out", "tx_edge", "cycles"}
    assert blocked["spi_target_10m"] == {"reflex", "host_stream"}
    assert blocked["usb_fs_device"] >= {"tx_edge", "response"}


def test_stretch_and_device_verdicts(js, data):
    by, _ = verdicts(js, "stretch")
    cap = {w["id"] for w in data["workloads"] if "cap19" in w["tags"]}
    only = {k: [w for w in v if w not in cap] for k, v in by.items()}
    assert set(only["served"]) == {"usb_ls_host", "canfd_2m_tx", "canfd_5m_tx", "sd4_host_12m5", "dshot600_tx"}
    assert only["marginal"] == ["sd4_host_25m"]
    assert set(only["impossible"]) == {"usb_ls_host_fw_v2", "usb_fs_tx", "usb_fs_rx_monitor", "rmii100_tx"}
    assert set(cell(js, "usb_ls_host_fw_v2")["blocked_by"]) == {"rx_eye"}
    assert set(cell(js, "usb_fs_tx")["blocked_by"]) == {"tx_edge", "host_stream"}
    assert set(cell(js, "usb_fs_rx_monitor")["blocked_by"]) == {"host_stream"}
    assert set(cell(js, "rmii100_tx")["blocked_by"]) == {"host_stream"}
    assert cell(js, "rmii100_tx", A(f_core_mhz=60))["blocked_by"].count("fixed_clock") == 1
    by, _ = verdicts(js, "device")
    dev = {w: cell(js, w) for w in ["spi_target_scalable", "spi_flash_read_decode", "i2c_target_fmplus"]}
    assert dev["spi_target_scalable"]["verdict"] == "marginal" and dev["spi_target_scalable"]["binding"] == "reflex"
    assert dev["spi_flash_read_decode"]["verdict"] == "impossible" and dev["spi_flash_read_decode"]["blocked_by"] == ["program_words"]
    assert dev["i2c_target_fmplus"]["verdict"] == "served"        # known gap, K6: the ISS max is 600 kHz
    assert set(by["served"]) | set(by["marginal"]) | set(by["impossible"]) == {
        w["id"] for w in data["workloads"] if "device" in w["tags"]}


# ------------------------------------------------------------------ shape and semantics of a cell
def test_cell_shape_and_binding(js, data):
    c = cell(js, "spi_target_10m")
    assert c["binding"] == c["headline"]["limit"] == c["ranked"][0]["id"] == "reflex"
    assert c["headline"]["nom"] == -87 and c["headline"]["unit"] == "ns"
    rels = [r["rel"] for r in c["ranked"]]
    assert rels == sorted(rels)
    assert {r["id"] for r in c["ranked"]} == {r["id"] for r in c["limits"]}
    # table order of 2.5 is kept in `limits`, and every limit of the table is either applicable or n/a with a reason
    table = [l["id"] for l in js("ArchSim.limits()")]
    assert len(table) == 18 and table[0] == "pad_toggle_out" and table[-1] == "isa_xor"
    assert [r["id"] for r in c["limits"]] == [i for i in table if i in {r["id"] for r in c["limits"]}]
    assert {r["id"] for r in c["limits"]} | {n["id"] for n in c["na"]} == set(table)
    assert all(n["reason"] for n in c["na"])
    for r in c["limits"]:
        v = r["value"]
        assert v["pess"] <= v["nom"] <= v["opt"], r["id"]
        assert r["formula"] and r["label"] and r["terms"] is not None and r["method"] == "formula"


def test_verdict_rule(js, data):
    for w in data["workloads"]:
        c = cell(js, w["id"])
        opts = [r["value"]["opt"] for r in c["limits"]]
        pess = [r["value"]["pess"] for r in c["limits"]]
        want = "impossible" if min(opts) < -1e-9 else ("served" if min(pess) >= -1e-9 else "marginal")
        assert c["verdict"] == want, w["id"]
        assert (c["blocked_by"] != []) == (want == "impossible")
        assert set(c["blocked_by"]) == {r["id"] for r in c["limits"] if r["value"]["opt"] < -1e-9}
        # the binding limit has the smallest relative nominal margin
        assert c["headline"]["rel"] == min(r["rel"] for r in c["limits"])


def chip(js, arch=None):
    return js("ArchSim.chipCheck(%s, DATA)" % (arch or "ArchSim.defaultArch()"))


def item(ck, iid):
    it = next(i for i in ck["items"] if i["id"] == iid)
    v = it["value"]
    return v["nom"], v["pess"], v["opt"]


def test_chip_check(js, data):
    ck = chip(js)
    assert ck["verdict"] == "pass" and ck["driver"] == "core_clock"
    assert [i["id"] for i in ck["items"]] == ["core_clock", "die_area", "board_clock", "pins_budget"]
    assert item(ck, "core_clock")[0] == ap(11.6, 1e-3)
    assert item(ck, "board_clock") == (16.5, 16.5, 16.5)
    d = item(ck, "die_area")
    assert d == (ap(289220, 1e-3), ap(156769, 1e-3), ap(408350, 1e-3))
    assert item(ck, "pins_budget") == (18, 18, 18)
    assert next(i for i in ck["items"] if i["id"] == "pins_budget")["scale"] == 24
    assert set(ck["records"]) == {i["id"] for i in ck["items"]}
    for i in ck["items"]:
        assert i["formula"] and i["terms"] and i["method"] == "formula"
    ck = chip(js, A(f_core_mhz=60))
    assert item(ck, "core_clock")[0] == ap(1.6, 1e-2)
    # levers_60 core_clock 38.43 [27.32, 51.13], board_clock 6.5
    ck = chip(js, "ArchSim.applyPreset('levers_60', DATA)")
    c3 = item(ck, "core_clock")
    assert (c3[0], c3[1], c3[2]) == (ap(38.43, 1e-2), ap(27.32, 1e-2), ap(51.13, 1e-2))
    assert item(ck, "board_clock")[0] == ap(6.5, 1e-2)
    # stack_a / wide_host: marginal on die area; stretch_usb_sd core_clock pess 3.38
    ck = chip(js, "ArchSim.applyPreset('stack_a', DATA)")
    assert ck["verdict"] == "marginal" and ck["driver"] == "die_area" and item(ck, "die_area")[1] == ap(-27877, 1e-3)
    ck = chip(js, "ArchSim.applyPreset('wide_host', DATA)")
    assert ck["verdict"] == "marginal" and ck["driver"] == "die_area" and item(ck, "die_area")[1] == ap(-31068, 1e-3)
    assert item(chip(js, "ArchSim.applyPreset('stretch_usb_sd', DATA)"), "core_clock")[1] == ap(3.38, 1e-2)
    # fail: f_core 62 (core_clock -0.4), 68 (board_clock -1.5 as well)
    ck = chip(js, A(f_core_mhz=62))
    assert ck["verdict"] == "fail" and ck["driver"] == "core_clock" and item(ck, "core_clock")[0] == ap(-0.4, 0.1)
    ck = chip(js, A(f_core_mhz=68))
    assert ck["verdict"] == "fail" and item(ck, "board_clock")[0] == ap(-1.5, 0.1)
    assert next(i for i in ck["items"] if i["id"] == "board_clock")["verdict"] == "fail"
    # a cell carries no chip id, and evaluateAll returns the chip check
    c = cell(js, "uart_3m")
    ids = {r["id"] for r in c["limits"]} | {n["id"] for n in c["na"]}
    assert not ids & {"core_clock", "die_area", "board_clock", "pins_budget"}
    assert not any(k in c["records"] for k in ("core_clock", "die_area", "board_clock"))
    assert [i["id"] for i in js("ArchSim.chipItems()")] == ["core_clock", "die_area", "board_clock", "isa_encoding", "pins_budget"]
    assert js("ArchSim.evaluateAll(ArchSim.defaultArch(), DATA).chip.verdict") == "pass"
    assert js("ArchSim.VERSION") == "2"


def test_chip_does_not_change_cells(js):
    a, b = cell(js, "uart_3m"), cell(js, "uart_3m", A(f_core_mhz=68))
    assert (a["verdict"], a["binding"]) == (b["verdict"], b["binding"])


def test_lite_equals_full(js, data):
    got = js("(function(){var a = ArchSim.defaultArch(), o = [];"
             " var f = ArchSim.evaluateAll(a, DATA), l = ArchSim.evaluateAll(a, DATA, null, {lite: true});"
             " Object.keys(f.cells).forEach(function(k){ var x = f.cells[k], y = l.cells[k];"
             "  o.push([k, x.verdict === y.verdict, x.binding === y.binding, x.headline.nom === y.headline.nom, y.na.length, Object.keys(y.records).length, y.limits[0].terms === undefined]); });"
             " return o;})()")
    assert got and all(r[1] and r[2] and r[3] and r[4] == 0 and r[5] == 0 and r[6] for r in got)


def test_binding_without_chip_limits(js):
    want = {"i2c_fmplus_ctrl": "program_words", "usb_ls_device": "tx_edge", "ws2812": "pad_toggle_out",
            "canfd_2m_tx": "pad_toggle_out", "sd4_host_12m5": "pins", "dshot600_tx": "tx_edge",
            "fw_uart_rx": "program_words", "fw_uart_tx": "pad_toggle_out"}
    for w, b in want.items():
        assert cell(js, w)["binding"] == b, w


def test_streaming_paging_and_links(js):
    base = cell(js, "spi_flash_read_decode")
    assert trio(base, "program_words")[0] == -8
    s = cell(js, "spi_flash_read_decode", A(streaming="out_exec"))
    assert trio(s, "program_words")[0] == INF and s["flags"]["streaming_active"] is True
    assert trio(s, "cycles")[0] == ap(trio(base, "cycles")[0] - 4)          # ipu doubles: 50 - 4 -> 50 - 8
    bw = 0.125 + 2 * 4 * 1e6 / 1e6
    assert lim(s, "host_stream")["scale"] == ap(256)
    assert trio(s, "host_stream")[0] < 0 and bw > 0
    # host_reload: no gap -> -1; with a gap -> gap - t_imem_load
    r = cell(js, "spi_flash_read_decode", A(imem_paging="host_reload"))
    assert trio(r, "program_words")[0] == -1
    r = cell(js, "eth10_rx", A(imem_paging="host_reload"))                 # gap 9600 ns, 45 words > 32
    assert trio(r, "program_words")[0] == ap(9600 - (js("ArchSim.hWrite(DATA.links.spi, 32)") * 20 + 1000))   # t_imem_load(imem_words)
    # a banked IMEM raises the capacity
    assert trio(cell(js, "spi_flash_read_decode", A(imem_words=64)), "program_words")[0] == 24
    # deep FIFO in the write direction rescues the 64-byte TX frame
    c = cell(js, "eth10_tx", A(deep_fifo_words=64))
    assert trio(c, "host_stream")[0] > 0
    # load-only link: no streaming at all
    # (L = 0: a frame that fits the buffer needs no streaming, a longer one cannot be refilled)
    c = cell(js, "eth10_tx", A(host_link="byte_load_only"))
    assert trio(c, "host_stream")[0] == ap(16 - 64) and "host_stream" in c["blocked_by"]
    # a continuous workload (no idle gap) cannot be served by a link that cannot refill: -bw_eff, not met (fix 4)
    u = cell(js, "uart_3m", A(host_link="byte_load_only"))
    assert trio(u, "host_stream")[0] == ap(-0.3) and "host_stream" in u["blocked_by"]
    assert "single frame only" in lim(c, "host_stream")["note"]
    c = cell(js, "usb_fs_device", A(host_link="byte_load_only"))
    assert trio(c, "response")[0] == -INF or c["verdict"] == "impossible"
    # env = iss zeroes the pad / skew / phase terms
    c = cell(js, "spi_target_10m", A(env="iss"))
    assert trio(c, "reflex") == (-50, -50, -50)
    # dtc places edges to half a tap
    c = cell(js, "usb_fs_device", A(fine_edge="dtc"))
    assert trio(c, "tx_edge")[0] == ap(3.5 - 0.0345 - 2)


def test_calibration_options(js):
    c = cell(js, "uart_3m", opts="{link_override_MBps: 0.5}")
    assert trio(c, "host_stream")[0] == INF
    c = cell(js, "eth10_tx", opts="{link_override_MBps: 0.5}")
    assert trio(c, "host_stream")[0] == ap(16 / (1 - 0.5 / 1.25) - 64)
    full = trio(cell(js, "usb_ls_host_fw_v2"), "rx_eye")[0]
    one = trio(cell(js, "usb_ls_host_fw_v2", opts="{mismatch_single_sided: true}"), "rx_eye")[0]
    assert one > full


# ------------------------------------------------------------------ monotonicity (3.2)
ARCHS = ["ArchSim.defaultArch()"] + ["ArchSim.applyPreset('%s', DATA)" % p
                                     for p in ["stack_a", "stretch_usb_sd", "wide_host", "levers_60", "usb_fs_48"]] + [
    A(fine_edge="dtc", capture="dpll", ddr_io=True), A(capture="adpll", adpll_mult=8, host_link="nibble"),
    A(streaming="out_exec", imem_paging="host_reload", lane=True), A(die="8x4", imem_tech="sram2p", imem_words=128, timing_lever="both"),
    # ISA levers (A4): the effect bands and d_isa_* delays are pseudo-parameters, margins stay monotone in them
    A(isa_irq=True, isa_xor=True, isa_cond=True, isa_mov_src=True, isa_word_bits=32),
    A(isa_xor=True, isa_word_bits=24, streaming="out_exec", imem_paging="host_reload", imem_tech="sram2p", die="8x4"),
    A(isa_cond=True, isa_mov_src=True, crc_unit=True, isa_xor=True, lane=True),
]


def test_monotone(js, data):
    n_checked = 0
    for arch in ARCHS:
        rows = js("(function(){var o = [], a = %s; DATA.workloads.forEach(function(w){"
                  " ArchSim.sensitivity(a, w, DATA, 5).forEach(function(s){ s.wl = w.id; o.push(s); }); }); return o;})()" % arch)
        assert rows
        for s in rows:
            m = s["margin"]
            inc = all(m[i] <= m[i + 1] for i in range(len(m) - 1))
            dec = all(m[i] >= m[i + 1] for i in range(len(m) - 1))
            assert inc or dec, (arch, s["wl"], s["limit"], s["input"], m)
            n_checked += 1
    assert n_checked > 2000


# ------------------------------------------------------------------ every ref resolves (3.1)
RESOLVE_JS = """
(function () {
  var bad = [], n = 0, archs = %s;
  archs.forEach(function (a, ai) {
    var d = ArchSim.derive(a, DATA), ids = Object.keys(d.records);
    function walk(owner, rec, extra) {
      rec.terms.forEach(function (t) {
        n++;
        if (!t.ref || !ArchSim.refResolves(t.ref, DATA, ids.concat(extra))) bad.push([ai, owner, rec.id, t.sym, t.ref]);
        if (t.value === undefined) bad.push([ai, owner, rec.id, t.sym, 'no value']);
      });
    }
    ids.forEach(function (k) { walk('arch', d.records[k], []); });
    DATA.workloads.forEach(function (w) {
      var c = ArchSim.evaluateCell(a, w, DATA, {derived: d}), extra = Object.keys(c.records);
      extra.forEach(function (k) { walk(w.id, c.records[k], extra); });
      c.limits.forEach(function (r) { walk(w.id, r, extra); });
    });
  });
  return {bad: bad, n: n};
})()
"""


def test_every_ref_resolves(js):
    out = js(RESOLVE_JS % ("[" + ", ".join(ARCHS) + "]"))        # the arch expressions are spliced in as JS
    assert out["bad"] == [], out["bad"][:10]
    assert out["n"] > 10000


def test_refs_unresolvable_detected(js):
    assert js("ArchSim.refResolves({kind:'param', id:'nope'}, DATA, [])") is False
    assert js("ArchSim.refResolves({kind:'workload_field', wl:'uart_3m', field:'nope', default:true}, DATA, [])") is False
    assert js("ArchSim.refResolves({kind:'workload_field', wl:'uart_3m', field:'ipu'}, DATA, [])") is True


# ------------------------------------------------------------------ hygiene
def test_no_workload_ids_in_js(data):
    src = (ROOT / "model/archsim-model.js").read_text()
    for w in data["workloads"]:
        assert "'" + w["id"] + "'" not in src and '"' + w["id"] + '"' not in src, w["id"]


# ------------------------------------------------------------------ fixes (verdict rounding, binding, data-borne constants)
def test_fw_rows_use_exact_rates_and_relative_tolerance(js, data):
    W = {w["id"]: w for w in data["workloads"]}
    assert W["fw_ws2812"]["rate_hz"] == 50e6 / 63                       # not the CSV's rounded 793651
    c = cell(js, "fw_ws2812")
    assert "cycles" not in c["blocked_by"] and c["verdict"] != "impossible"
    assert trio(c, "cycles")[0] > -1e-6 * lim(c, "cycles")["scale"]
    assert c["verdict"] == "served"                                      # true verdict on the MVP


def test_binding_explains_verdict(js, data):
    for pre in ("mvp", "stack_a", "wide_host"):
        for w in data["workloads"]:
            c = cell(js, w["id"], "Object.assign(ArchSim.defaultArch(), DATA.presets.filter(function(p){return p.id=='%s'})[0].arch)" % pre)
            if c["verdict"] == "impossible":
                assert c["binding"] == c["blocked_by"][0] == c["headline"]["limit"], (pre, w["id"])
                assert c["closest"] == c["ranked"][0]["id"]
            else:
                assert c["binding"] == c["closest"]


def test_constants_are_data_and_traceable(js, data):
    for lid in ("spi", "spi_fast", "qspi"):
        f = data["links"][lid]["framing"]
        assert (f["cmd_bits"], f["wr_bits"], f["rd_bits"], f["gap_cycles"], f["access_cycles"]) == (8, 32, 40, 2, 5) and f["src"]
    assert data["links"]["byte"]["framing"]["loop_pio_cycles"] == 3
    fam = {(b.get("family"), b.get("role")): b for b in data["blocks"].values() if "family" in b}
    assert (fam[("imem_flop", "anchor")]["words"], fam[("imem_flop", "anchor")]["bits"]) == (32, 16)
    assert (fam[("fifo", "anchor")]["words"], fam[("fifo", "anchor")]["bits"]) == (2, 32)
    d = js("ArchSim.derive(ArchSim.defaultArch(), DATA)")
    srcs = [t["ref"] for r in d["records"].values() for t in r["terms"]]
    assert {"kind": "link", "id": "spi", "field": "framing.cmd_bits"} in srcs
    assert {"kind": "iss", "id": "pins", "field": "ui_in"} in srcs
    assert {"kind": "block", "id": "imem_ff_per_word"} in srcs and {"kind": "block", "id": "fifo_per_depth"} in srcs
    assert all("area_cell" in b and "area_placed" in b and "area" not in b for b in d["area"]["blocks"])


def test_no_uncertainty_note(js):
    c = cell(js, "fw_ws2812")
    assert "No uncertainty inputs" in lim(c, "cycles")["note"]

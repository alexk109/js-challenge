"""Explore solvers (alternatives, knobImpact, whatWouldItTake), the extended mix, and the Timing /
Throughput helpers (rxSampling, clockBudget, hostDemand)."""
import json
import time
import pytest
from conftest import ROOT

A_MVP = "ArchSim.defaultArch()"


def A(**kw):
    return "Object.assign(ArchSim.defaultArch(), " + json.dumps(kw) + ")"


def ap(x, rel=1e-3, abs_=1e-9):
    return pytest.approx(x, rel=rel, abs=abs_)


# ------------------------------------------------------------------ alternatives
def test_alternatives_skip_rules(js):
    alts = js("ArchSim.alternatives(%s, 'full', DATA)" % A_MVP)
    knobs = {a["knob"] for a in alts}
    assert "env" not in knobs and "signoff_corner" not in knobs
    assert not knobs & {"adpll_mult", "imem_banks", "deep_fifo_dir"}                 # no-ops at the MVP
    assert {"adpll_mult"} <= {a["knob"] for a in js("ArchSim.alternatives(%s, 'full', DATA)" % A(capture="adpll"))}
    assert {"imem_banks"} <= {a["knob"] for a in js("ArchSim.alternatives(%s, 'full', DATA)" % A(imem_paging="bank_switch"))}
    assert {"deep_fifo_dir"} <= {a["knob"] for a in js("ArchSim.alternatives(%s, 'full', DATA)" % A(deep_fifo_words=64))}


def test_alternatives_neighbours(js):
    alts = js("ArchSim.alternatives(%s, 'neighbour', DATA)" % A_MVP)
    f = [a for a in alts if a["knob"] == "f_core_mhz"]
    assert [a["value"] for a in f] == [48, 52] and all(a["steps"] == 1 and a["from"] == 50 for a in f)
    assert [a["value"] for a in alts if a["knob"] == "n_engines"] == [1, 3]
    assert [a["value"] for a in alts if a["knob"] == "crc_unit"] == [True]
    assert [a["value"] for a in alts if a["knob"] == "timing_lever"] == ["tool", "rtl", "both"]
    assert [a["steps"] for a in alts if a["knob"] == "host_link"] == [1, 2, 3, 4, 5]    # option-index distance
    ex = [a for a in alts if a["knob"] == "extras"]
    assert len(ex) == len(js("ArchSim.EXTRA_IDS")) and all(a["steps"] == 1 for a in ex)
    off = js("ArchSim.alternatives(%s, 'neighbour', DATA)" % A(f_core_mhz=55))          # off-grid: the nearest below and above
    assert [a["value"] for a in off if a["knob"] == "f_core_mhz"] == [54, 56]


def test_alternatives_full_grid(js, data):
    sd = data["sweep_defaults"]["f_core_mhz"]
    grid = set(range(sd["min"], sd["max"] + 1, sd["step"])) | set(sd["extra"])
    f = [a["value"] for a in js("ArchSim.alternatives(%s, 'full', DATA)" % A_MVP) if a["knob"] == "f_core_mhz"]
    assert set(f) == grid - {50} and f == sorted(f)
    n = [a["value"] for a in js("ArchSim.alternatives(%s, 'full', DATA)" % A_MVP) if a["knob"] == "n_engines"]
    assert n == [1, 3, 4, 5, 6]


# ------------------------------------------------------------------ knobImpact
FLIPS = {                                                 # (knob, value) -> [(wl, dir)], chip verdict
    ("fine_edge", "dtc"): (["logic_analyzer_100ps up"], "pass"),
    ("n_engines", 1): (["uart_3m down"], "pass"),
    ("deep_fifo_words", 64): (["eth10_tx up"], "pass"),
    ("deep_fifo_words", 256): (["eth10_tx up"], "pass"),
    ("deep_fifo_words", 512): (["eth10_tx up"], "pass"),
    ("deep_fifo_words", 1024): (["eth10_tx up"], "marginal"),
    ("deep_fifo_words", 2048): (["eth10_tx up"], "marginal"),
    ("deep_fifo_words", 4096): (["eth10_tx up"], "fail"),
    ("crc_unit", True): (["can_1m up"], "pass"),
    ("host_link", "spi_fast"): (["eth10_tx up"], "pass"),
    ("host_link", "qspi"): (["i3c_ctrl_12m5 up", "swd_host_10m up", "eth10_tx up", "mii10_mac up"], "pass"),
    ("host_link", "nibble"): (["spi_host_25m up", "i3c_ctrl_12m5 up", "swd_host_10m up", "eth10_tx up", "mii10_mac up"], "pass"),
    ("host_link", "byte"): (["spi_host_25m up", "i3c_ctrl_12m5 up", "swd_host_10m up", "eth10_tx up"], "pass"),
    ("host_link", "byte_load_only"): (["uart_3m down", "i2c_fmplus_ctrl down", "usb_ls_device down", "ws2812 down"], "pass"),
}
CHIP_ONLY = {("n_engines", 3): "marginal", ("imem_words", 128): "marginal", ("imem_words", 256): "fail", ("imem_words", 512): "fail"}


@pytest.fixture(scope="module")
def impact(js):
    t = time.perf_counter()
    r = js("ArchSim.knobImpact(%s, DATA, {tag: 'cap19'})" % A_MVP)
    r["_ms"] = (time.perf_counter() - t) * 1000
    return r


def test_knob_impact_flip_list(impact):
    got, chip_only = {}, {}
    for k in impact["knobs"]:
        for a in k["alternatives"]:
            if k["knob"].startswith("isa_"):
                continue
            key = (k["knob"], a["value"])
            if a["flips"]:
                got[key] = ([f["wl"] + " " + f["dir"] for f in a["flips"]], a["chip"]["verdict"])
            elif a["chip"]["verdict"] != "pass":
                chip_only[key] = a["chip"]["verdict"]
    assert {k: (sorted(v[0]), v[1]) for k, v in got.items()} == {k: (sorted(v[0]), v[1]) for k, v in FLIPS.items()}
    assert chip_only == CHIP_ONLY


def test_knob_impact_shape(impact):
    assert impact["base"]["chip"]["verdict"] == "pass" and impact["base"]["counts"] == {"served": 4, "marginal": 0, "impossible": 15}
    ids = [k["knob"] for k in impact["knobs"]]
    assert "env" not in ids and "signoff_corner" not in ids
    by = {k["knob"]: k for k in impact["knobs"]}
    assert by["adpll_mult"]["no_effect"] and by["adpll_mult"]["alternatives"] == []
    a = [x for x in by["host_link"]["alternatives"] if x["value"] == "byte"][0]
    assert a["ups"] == 4 and a["downs"] == 0 and a["d_area_nom"] < 0 and a["steps"] == 4
    f = [x for x in by["f_core_mhz"]["alternatives"] if x["value"] == 52][0]
    assert f["d_core_clock_nom"] == ap(-2.0) and f["chip"]["driver"] == "core_clock"
    e = [x for x in by["n_engines"]["alternatives"] if x["value"] == 3][0]
    assert e["chip"]["driver"] == "die_area" and e["chip"]["margin"]["pess"] < 0


def test_knob_impact_order(js, impact):
    order = [k["id"] for k in js("ArchSim.KNOBS")]
    got = [k["knob"] for k in impact["knobs"]]
    assert got == [i for i in order if i not in ("env", "signoff_corner")]


def test_knob_impact_single_protocol(js):
    r = js("ArchSim.knobImpact(%s, DATA, {wl: 'eth10_tx'})" % A_MVP)
    flips = {(k["knob"], a["value"]) for k in r["knobs"] for a in k["alternatives"] if a["flips"]}
    assert ("host_link", "spi_fast") in flips and ("crc_unit", True) not in flips
    assert all(f["wl"] == "eth10_tx" for k in r["knobs"] for a in k["alternatives"] for f in a["flips"])


def test_knob_impact_time(impact):
    print("knobImpact(mvp, cap19): %.0f ms under QuickJS" % impact["_ms"])
    assert impact["_ms"] < 3000


# ------------------------------------------------------------------ whatWouldItTake
def wwit(js, wl, arch=A_MVP, data="DATA", opts="undefined"):
    return js("ArchSim.whatWouldItTake(%s, '%s', %s, %s)" % (arch, wl, data, opts))


def names(c):
    return "+".join("%s=%s" % (x["knob"], str(x["to"]).lower()) for x in c["changes"])


def test_wwit_eth10_tx(js):
    r = wwit(js, "eth10_tx")
    s = r["single"]
    assert not r["already"] and not r["out_of_reach"] and r["pairs"] == []
    assert names(s[0]) == "host_link=spi_fast" and s[0]["d_area_nom"] == 0 and s[0]["chip"] == "pass"
    assert not s[0]["estimate"] and s[0]["losses"] == [] and s[0]["steps"] == 1
    assert "deep_fifo_words=64" in [names(c) for c in s]
    chips = [c["chip"] for c in s]
    assert chips == sorted(chips, key=["pass", "marginal", "fail"].index)           # clean candidates first
    broken = [c for c in s if c["chip"] != "pass"]
    assert broken and all(c["breaks_chip"].startswith("breaks the chip check: " if c["chip"] == "fail" else "leaves the chip check marginal: ") for c in broken)
    assert all("pessimistic" in c["breaks_chip"] for c in broken)
    assert all(c["breaks_chip"] is None for c in s if c["chip"] == "pass")
    assert any(c["restore"] for c in broken)                                         # a single knob that restores the chip
    assert all(c["verdict"] == "served" for c in s)


def test_wwit_evidence_rule(js, data):
    s = wwit(js, "eth10_tx")["single"]
    est = {names(c): c["estimate"] for c in s}
    assert est["host_link=spi_fast"] is False and est["host_link=nibble"] is True       # nibble needs eta_par (judgement)
    pos = {names(c): i for i, c in enumerate(s)}
    assert pos["host_link=spi_fast"] < pos["host_link=qspi"] < pos["deep_fifo_words=64"] < pos["host_link=nibble"]
    assert s[pos["host_link=nibble"]]["d_area_nom"] < 0                                 # nibble is smaller, yet ranks after: evidence first
    assert data["params"]["eta_par"]["status"] == "judgement"
    # generic: an input is judgement-backed iff its status says so
    r = js("(function () { var D = JSON.parse(JSON.stringify(DATA)); D.params.eta_par.status = 'measured';"
           " return ArchSim.whatWouldItTake(%s, 'eth10_tx', D); })()" % A_MVP)
    assert [names(c) for c in r["single"]][0] == "host_link=nibble" and not r["single"][0]["estimate"]


def test_wwit_can_1m(js):
    r = wwit(js, "can_1m")
    assert [names(c) for c in r["single"]] == ["crc_unit=true"] and not r["single"][0]["estimate"]
    assert r["blocking"][0]["id"] == "isa_xor"


@pytest.mark.parametrize("wl", ["spi_target_10m", "jtag_target_10m"])
def test_wwit_pairs(js, wl):
    r = wwit(js, wl)
    assert r["single"] == [] and not r["out_of_reach"]
    clean = [c for c in r["pairs"] if c["chip"] == "pass"]
    assert {names(c) for c in clean} == {"host_link=nibble+lane=true", "host_link=byte+lane=true", "host_link=qspi+lane=true"}
    assert {names(c): c["steps"] for c in clean} == {"host_link=nibble+lane=true": 4, "host_link=byte+lane=true": 5, "host_link=qspi+lane=true": 3}   # lane 1 + option distance
    # evidence rule: qspi needs no new judgement input; among the estimates the placed area decides (nibble < byte < qspi)
    assert [names(c) for c in clean] == ["host_link=qspi+lane=true", "host_link=nibble+lane=true", "host_link=byte+lane=true"]
    areas = {names(c): c["d_area_nom"] for c in clean}
    assert areas["host_link=nibble+lane=true"] < areas["host_link=byte+lane=true"] < areas["host_link=qspi+lane=true"]
    assert [c["chip"] for c in r["pairs"]] == sorted((c["chip"] for c in r["pairs"]), key=["pass", "marginal", "fail"].index)
    assert all(len(c["changes"]) == 2 and c["verdict"] == "served" for c in r["pairs"])


def test_wwit_swd_host_10m(js):
    r = wwit(js, "swd_host_10m")
    assert names(r["single"][0]) == "host_link=qspi" and r["single"][0]["chip"] == "pass"
    assert {"host_link=nibble", "host_link=byte"} <= {names(c) for c in r["single"]}


@pytest.mark.parametrize("wl", ["dvi_250m", "usb_fs_device", "canfd_5m", "eth10_rx", "spi_target_25m", "i3c_target_12m5"])
def test_wwit_out_of_reach(js, wl):
    r = wwit(js, wl)
    assert r["out_of_reach"] and r["single"] == [] and r["pairs"] == [] and r["blocking"] and r["hints"]
    assert {h["limit"] for h in r["hints"]} <= {b["id"] for b in r["blocking"]}


def test_wwit_hint_text_values(js):
    r = wwit(js, "spi_target_25m")
    h = {x["limit"]: x for x in r["hints"]}
    assert h["reflex"]["knob"] == "lane" and h["reflex"]["to"] is True
    assert h["reflex"]["nom_before"] == ap(-117) and h["reflex"]["nom_after"] == ap(-9.253)
    r = wwit(js, "i3c_target_12m5")
    assert {x["limit"]: x for x in r["hints"]}["reflex"]["nom_after"] == ap(0.747)


def test_wwit_already(js):
    r = wwit(js, "uart_3m")
    assert r["already"] and r["single"] == [] and r["pairs"] == [] and not r["out_of_reach"]
    r = wwit(js, "eth10_tx", arch=A(host_link="spi_fast"))
    assert r["already"]


def test_wwit_marginal_target(js, data):
    # a marginal row is solved to served as well: stretch_usb_sd has marginal cells in the all tag
    arch = "ArchSim.applyPreset('stretch_usb_sd', DATA)"
    ev = js("ArchSim.evaluateAll(%s, DATA, 'all', {lite: true})" % arch)
    marg = ev["by_verdict"]["marginal"]
    assert marg
    r = wwit(js, marg[0], arch=arch)
    assert not r["already"] and r["verdict"] == "marginal"
    assert all(c["verdict"] == "served" for c in r["single"] + r["pairs"])


def test_wwit_time(js):
    t = time.perf_counter()
    wwit(js, "eth10_tx")
    wwit(js, "spi_target_10m")
    ms = (time.perf_counter() - t) * 1000
    print("whatWouldItTake eth10_tx + spi_target_10m: %.0f ms under QuickJS" % ms)
    assert ms < 3000


# ------------------------------------------------------------------ lite vs full
def test_lite_equals_full_on_presets(js, data):
    for p in data["presets"]:
        full = js("ArchSim.evaluateAll(ArchSim.applyPreset('%s', DATA), DATA, 'all')" % p["id"])
        lite = js("ArchSim.evaluateAll(ArchSim.applyPreset('%s', DATA), DATA, 'all', {lite: true})" % p["id"])
        assert full["cells"].keys() == lite["cells"].keys()
        for k, c in full["cells"].items():
            l = lite["cells"][k]
            assert (c["verdict"], c["binding"], c["headline"]) == (l["verdict"], l["binding"], l["headline"]), (p["id"], k)
        assert full["chip"]["verdict"] == lite["chip"]["verdict"] and full["chip"]["driver"] == lite["chip"]["driver"]


# ------------------------------------------------------------------ mix
def test_mix_members_and_rows(js):
    m = js("ArchSim.mix(%s, ['ws2812', 'i2c_fmplus_ctrl'], DATA)" % A_MVP)
    assert [x["wl"] for x in m["members"]] == ["ws2812", "i2c_fmplus_ctrl"]
    assert all(x["verdict"] in ("served", "marginal", "impossible") and x["binding"] for x in m["members"])
    rows = {r["id"]: r for r in m["rows"]}
    assert rows["imem"]["need"] == 0 and rows["imem"]["verdict"] == "served" and m["imem_no_fit"] == []
    assert rows["host_loop"]["need"]["nom"] == 0 and rows["host_loop"]["verdict"] == "served"
    assert m["chip"] == {"verdict": "pass", "driver": "core_clock"}
    ev = js("ArchSim.evaluateAll(%s, DATA, 'all', {lite: true})" % A_MVP)
    for x in m["members"]:                                   # the member verdicts are the point's own
        assert x["verdict"] == ev["cells"][x["wl"]]["verdict"] and x["binding"] == ev["cells"][x["wl"]]["binding"]


def test_mix_member_alone_impossible_is_separate(js):
    m = js("ArchSim.mix(%s, ['uart_3m', 'eth10_tx'], DATA)" % A_MVP)
    mem = {x["wl"]: x for x in m["members"]}
    assert mem["eth10_tx"]["verdict"] == "impossible" and mem["eth10_tx"]["binding"] == "host_stream"
    assert mem["uart_3m"]["verdict"] == "served"
    rows = {r["id"]: r for r in m["rows"]}
    assert rows["engines"]["verdict"] == "impossible"          # 4 engines needed on a 2-engine MVP: a shared-resource failure


def test_mix_host_loop_row(js):
    m = js("ArchSim.mix(%s, ['usb_fs_device', 'usb_ls_device'], DATA)" % A_MVP)
    assert m["host_loop_members"] == ["usb_fs_device"]         # 67-byte answer does not fit the 16-byte TX buffer
    r = {x["id"]: x for x in m["rows"]}["host_loop"]
    assert r["need"]["nom"] > 0
    t = js("ArchSim.timeline(%s, 'usb_fs_device', DATA)" % A_MVP)
    assert r["need"]["nom"] == ap(t["segments"][-1]["dur"] / t["deadline"])
    assert r["margin"]["nom"] == ap(1 - r["need"]["nom"])
    big = js("ArchSim.mix(%s, ['usb_fs_device', 'usb_ls_device'], DATA)" % A(deep_fifo_words=256))
    assert big["host_loop_members"] == [] and {x["id"]: x for x in big["rows"]}["host_loop"]["need"]["nom"] == 0


def test_mix_imem_row(js, data):
    big = [w for w in data["workloads"] if w["words"] > 32 and w["engines"] <= 2][0]["id"]
    m = js("ArchSim.mix(%s, ['%s', 'ws2812'], DATA)" % (A_MVP, big))
    r = {x["id"]: x for x in m["rows"]}["imem"]
    assert m["imem_no_fit"] == [big] and r["need"] == 1 and r["verdict"] == "impossible" and m["verdict"] == "impossible"
    assert [x for x in m["members"] if x["wl"] == big][0]["program_fits"] is False
    m = js("ArchSim.mix(%s, ['%s', 'ws2812'], DATA)" % (A(imem_words=512), big))
    assert m["imem_no_fit"] == []


def test_mix_host_reload_adds_link_time(js, data):
    gapped = [w for w in data["workloads"] if w["gap_ns"] and w["words"] > 32 and w["engines"] <= 2]
    assert gapped
    w = gapped[0]["id"]
    base = js("ArchSim.mix(%s, ['%s'], DATA)" % (A_MVP, w))
    rel = js("ArchSim.mix(%s, ['%s'], DATA)" % (A(imem_paging="host_reload"), w))
    sb = {x["id"]: x for x in base["rows"]}["link_share"]["need"]["nom"]
    sr = {x["id"]: x for x in rel["rows"]}["link_share"]["need"]["nom"]
    assert sr > sb
    d = js("ArchSim.derive(%s, DATA)" % A(imem_paging="host_reload"))
    assert sr - sb == ap(d["link"]["t_imem_load_32"]["nom"] / gapped[0]["gap_ns"])


# ------------------------------------------------------------------ timing / throughput helpers
def test_rx_sampling(js):
    r = js("ArchSim.rxSampling(%s, 'eth10_rx', DATA)" % A_MVP)
    assert r["applicable"] and r["margin"]["nom"] == ap(-8.52)
    assert r["edge_half"] == ap(25) and r["u"]["nom"] == ap(20) and r["jitter"] == ap(13.5)
    assert r["margin"]["nom"] == ap(r["edge_half"] - r["u"]["nom"] - r["drift"]["nom"] - r["jitter"])
    assert r["osr"]["need"] == 3 and r["osr"]["have"]["nom"] == ap(2.5)
    cell = js("ArchSim.evaluateCell(%s, 'eth10_rx', DATA)" % A_MVP)
    eye = [x for x in cell["limits"] if x["id"] == "rx_eye"][0]
    assert eye["value"] == {k: ap(r["margin"][k]) for k in ("nom", "pess", "opt")}          # the helper IS the limit's body
    d = js("ArchSim.rxSampling(%s, 'eth10_rx', DATA)" % A(capture="dpll"))
    assert d["margin"]["nom"] == ap(1.5) and d["u"]["nom"] == ap(10)


def test_rx_sampling_not_applicable(js):
    r = js("ArchSim.rxSampling(%s, 'spi_target_10m', DATA)" % A_MVP)
    assert r["applicable"] is False and r["reason"]


def test_clock_budget(js):
    b = js("ArchSim.clockBudget(%s, DATA)" % A_MVP)
    assert b["corner"] == "typ" and b["T"] == 20
    assert b["D"]["nom"] == ap(1000 / 61.6) and b["fmax"]["nom"] == ap(61.6)
    assert [p["id"] for p in b["parts"]] == ["base", "imem", "die"]
    assert sum(p["ns"]["nom"] for p in b["parts"]) == ap(b["D"]["nom"])
    assert b["parts"][1]["ns"]["nom"] == 0 and b["parts"][2]["ns"]["nom"] == 0           # 32 words, 6x4 die
    assert b["parts"][0]["ref"] == {"kind": "param", "id": "fmax_slow_none_mhz"}
    s = js("ArchSim.clockBudget(%s, DATA)" % A(die="8x4", imem_words=128, signoff_corner="slow"))
    assert s["corner"] == "slow" and sum(p["ns"]["nom"] for p in s["parts"]) == ap(s["D"]["nom"])
    assert s["D"]["pess"] > s["D"]["nom"] > s["D"]["opt"] and 1000 / s["D"]["pess"] == ap(s["fmax"]["pess"])
    assert s["parts"][1]["ns"]["nom"] > 0 and s["parts"][2]["ns"]["pess"] > 0 == s["parts"][2]["ns"]["nom"]
    chip = js("ArchSim.chipCheck(%s, DATA)" % A(die="8x4", imem_words=128, signoff_corner="slow"))
    cc = [i for i in chip["items"] if i["id"] == "core_clock"][0]
    assert cc["value"]["nom"] == ap(s["fmax"]["nom"] - 50)


def test_host_demand(js, data):
    h = js("ArchSim.hostDemand(%s, 'eth10_tx', DATA)" % A_MVP)
    assert h["payload_MBps"] == ap(1.25) and h["stream_MBps"] == 0 and h["bw_eff"] == ap(1.25)
    assert h["dir"] == "write" and h["buf_B"] == 16 and h["link"]["nom"] == ap(0.6993007)
    cell = js("ArchSim.evaluateCell(%s, 'eth10_tx', DATA)" % A_MVP)
    hs = [x for x in cell["limits"] if x["id"] == "host_stream"][0]
    assert hs["value"]["nom"] == ap(16 / (1 - h["link"]["nom"] / h["bw_eff"]) - h["frame_B"])
    wl = [w for w in data["workloads"] if w["words"] > 32 and w["bw_MBps"] > 0][0]       # streaming is active when the program overflows
    s = js("ArchSim.hostDemand(%s, '%s', DATA)" % (A(streaming="out_exec"), wl["id"]))
    assert s["stream_MBps"] == ap(2 * wl["ipu"] * wl["rate_hz"] / 1e6) and s["bw_eff"] == ap(wl["bw_MBps"] + s["stream_MBps"])
    assert js("ArchSim.hostDemand(%s, '%s', DATA)" % (A_MVP, wl["id"]))["stream_MBps"] == 0

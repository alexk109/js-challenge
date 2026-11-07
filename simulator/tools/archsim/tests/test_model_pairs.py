"""ArchSim.pairs and ArchSim.maxConcurrent (all-pairs concurrency over ArchSim.mix) and the reference mixes data."""
import pytest


def _pairs(js, preset, tag="cap19", extra=""):
    return js("ArchSim.pairs(ArchSim.applyPreset('%s', DATA), DATA, {tag: '%s'%s})" % (preset, tag, extra))


def _by(r):
    return {(p["a"], p["b"]): p for p in r["pairs"]}


def test_mvp_pairs_cap19(js):
    r = _pairs(js, "mvp")
    assert r["ids"] == ["uart_3m", "i2c_fmplus_ctrl", "usb_ls_device", "ws2812"]
    assert r["tally"] == {"served": 4, "marginal": 1, "impossible": 5, "total": 10}
    P = _by(r)
    # uart_3m uses both engines: it pairs with nothing, itself included
    for other in r["ids"]:
        p = P[("uart_3m", other)] if ("uart_3m", other) in P else P[(other, "uart_3m")]
        assert p["verdict"] == "impossible" and "engines" in p["failing"]
    # two one-engine protocols pair; so does a one-engine protocol with itself (two instances)
    assert P[("i2c_fmplus_ctrl", "ws2812")]["verdict"] == "served" and P[("i2c_fmplus_ctrl", "ws2812")]["failing"] == []
    assert P[("ws2812", "ws2812")]["verdict"] == "served" and P[("ws2812", "ws2812")]["binding"] is None
    # the half-duplex link share, not the engines, limits USB LS + I2C
    m = P[("i2c_fmplus_ctrl", "usb_ls_device")]
    assert m["verdict"] == "marginal" and "link_share" in m["failing"] and m["binding"] == "link_share"
    assert all(p["rows"] for p in r["pairs"] if p["verdict"] != "served")


def test_pairs_only_include_served_alone(js, data):
    r = _pairs(js, "mvp", "all")
    alone = js("ArchSim.evaluateAll(ArchSim.applyPreset('mvp', DATA), DATA, 'all').by_verdict.served")
    assert r["ids"] == alone
    assert {s["wl"] for s in r["skipped"]} == {w["id"] for w in data["workloads"]} - set(alone)
    n = len(alone)
    assert r["tally"]["total"] == len(r["pairs"]) == n * (n + 1) // 2
    assert sum(r["tally"][k] for k in ("served", "marginal", "impossible")) == r["tally"]["total"]


def test_pairs_tally_by_preset(js):
    assert _pairs(js, "stretch_usb_sd")["tally"] == {"served": 6, "marginal": 1, "impossible": 8, "total": 15}
    w = _pairs(js, "wide_host")
    assert w["tally"] == {"served": 35, "marginal": 0, "impossible": 10, "total": 45}
    # a pair with the 4-pin-wide MII row fails on pins even though each is served alone
    assert _by(w)[("mii10_mac", "mii10_mac")]["failing"] == ["pins"]


def test_pairs_agree_with_mix(js):
    r = _pairs(js, "wide_host")
    for p in r["pairs"][:12]:
        m = js("ArchSim.mix(ArchSim.applyPreset('wide_host', DATA), ['%s', '%s'], DATA)" % (p["a"], p["b"]))
        assert m["verdict"] == p["verdict"]
        assert [x["id"] for x in m["rows"] if x["verdict"] != "served"] == p["failing"]


def test_pairs_explicit_ids(js):
    r = js("ArchSim.pairs(ArchSim.applyPreset('mvp', DATA), DATA, {ids: ['ws2812', 'uart_3m', 'canfd_5m']})")
    assert r["ids"] == ["ws2812", "uart_3m"] and [s["wl"] for s in r["skipped"]] == ["canfd_5m"]
    assert r["tally"]["total"] == 3


def _mc(js, preset, ids, extra=""):
    return js("ArchSim.maxConcurrent(ArchSim.applyPreset('%s', DATA), DATA, %s%s)" % (preset, ids, extra))


def test_max_concurrent_mvp(js):
    r = _mc(js, "mvp", "['uart_3m', 'ws2812', 'i2c_fmplus_ctrl']")
    assert r["n"] == 2 and "uart_3m" not in r["members"] and r["exact"]
    # uart_3m alone is `marginal` in mix (both-direction link bandwidth 0.30 MB/s vs 0.31 MB/s at nominal): strict count 0
    assert _mc(js, "mvp", "['uart_3m']")["n"] == 0
    assert _mc(js, "mvp", "['uart_3m']", ", {marginal: true}")["n"] == 1
    assert _mc(js, "mvp", "['ws2812']")["n"] == 1
    # members that are not served alone are excluded first
    r = _mc(js, "mvp", "['can_1m', 'ws2812']")
    assert r["n"] == 1 and r["alone_excluded"] == ["can_1m"]


def test_max_concurrent_matches_brute_force(js):
    ids = "['uart_3m', 'i2c_fmplus_ctrl', 'usb_ls_device', 'ws2812', 'spi_host_25m', 'swd_host_10m']"
    for preset in ("mvp", "wide_host"):
        r = _mc(js, preset, ids)
        brute = js("(function(){var a=ArchSim.applyPreset('%s',DATA),ids=%s,best=0;"
                   "for(var m=1;m<(1<<ids.length);m++){var s=ids.filter(function(_,i){return m>>i&1});"
                   "var alone=s.every(function(id){return ArchSim.evaluateCell(a,id,DATA).verdict==='served'});"
                   "if(alone&&s.length>best&&ArchSim.mix(a,s,DATA).verdict==='served')best=s.length}return best})()" % (preset, ids))
        assert r["n"] == brute and r["exact"]


def test_max_concurrent_cap_is_reported(js):
    r = _mc(js, "wide_host", "DATA.workloads.map(function (w) { return w.id; })", ", {cap: 3}")
    assert r["exact"] is False and r["evaluations"] <= 3 and r["cap"] == 3


def test_max_concurrent_engine_bound_makes_big_lists_exact(js):
    r = _mc(js, "wide_host", "DATA.workloads.map(function (w) { return w.id; })")
    assert r["n"] == 2 and r["exact"] and r["upper_bound"] == 2


# ------------------------------------------------------------------ reference mixes (data)
def test_reference_mixes_data(data):
    ids = {w["id"] for w in data["workloads"]}
    mixes = data["mixes"]
    assert [m["id"] for m in mixes] == ["debug_probe", "usb_serial", "sensor_hub", "maker_io", "industrial", "ethernet", "bench_4"]
    for m in mixes:
        assert m["label"] and m["description"] and m["src"] and len(m["members"]) >= 2
        assert set(m["members"]) <= ids
    by = {m["id"]: m["members"] for m in mixes}
    assert by["debug_probe"] == ["swd_host_10m", "uart_3m"] and by["ethernet"] == ["eth10_tx", "eth10_rx"]


@pytest.mark.parametrize("pid,expect", [
    ("mvp", {"debug_probe": ["engines", "bw_both", "link_share"], "ethernet": ["imem", "bw_write", "bw_read", "link_share"],
             "maker_io": ["engines"], "bench_4": ["engines"]}),
    ("wide_host", {"debug_probe": ["engines"], "sensor_hub": ["engines"], "ethernet": ["imem"], "bench_4": ["engines"]}),
])
def test_reference_mix_verdicts(js, pid, expect):
    for mid, rows in expect.items():
        m = js("(function(){var mx=DATA.mixes.filter(function(x){return x.id==='%s'})[0];"
               "return ArchSim.mix(ArchSim.applyPreset('%s', DATA), mx.members, DATA)})()" % (mid, pid))
        assert m["verdict"] == "impossible"
        assert [r["id"] for r in m["rows"] if r["verdict"] != "served"] == rows

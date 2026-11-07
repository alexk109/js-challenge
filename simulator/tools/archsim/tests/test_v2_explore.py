"""The Explore tab (per-knob impact, what would it take, sweep, Pareto) and its three side-panel kinds."""
import re
import time

import pytest


def render(js, raw="{}"):
    return js(f"ArchSimUI.renderExplore(ArchSimUI.makeState({raw}, DATA))")


def focus(wl, extra=""):
    return "{focus: {wl: %r}%s}" % (wl, extra)


def section(h, name):
    m = re.search(r'<section class="ex-sec" data-section="%s".*?</section>' % name, h, re.S)
    assert m, name
    return m.group(0)


def attrs(tag_html):
    return dict(re.findall(r'([\w-]+)="([^"]*)"', tag_html))


def alts(h):
    out = {}
    for m in re.finditer(r'<(?:button|span) type="button" class="ex-alt[^"]*"([^>]*)>|<span class="ex-alt invalid"([^>]*)>', h):
        a = attrs(m.group(0))
        out[(a["data-knob"], a["data-value"])] = a
    return out


def cand_rows(h):
    return re.findall(r'<tr class="ex-cand[^"]*".*?</tr>', section(h, "wwit"), re.S)


# ------------------------------------------------------------------ per-knob impact
def test_impact_lists_the_a3_flips_with_their_chips(js):
    A = alts(section(render(js), "impact"))
    assert A[("fine_edge", "dtc")]["data-ups"] == "1" and A[("fine_edge", "dtc")]["data-chip"] == "pass"
    assert A[("n_engines", "1")]["data-downs"] == "1" and A[("n_engines", "1")]["data-ups"] == "0"
    assert A[("host_link", "nibble")]["data-ups"] == "5" and A[("host_link", "qspi")]["data-ups"] == "4"
    assert A[("host_link", "byte_load_only")]["data-downs"] == "4"
    assert A[("crc_unit", "true")]["data-ups"] == "1"
    assert A[("deep_fifo_words", "1024")]["data-chip"] == "marginal" and A[("deep_fifo_words", "4096")]["data-chip"] == "fail"
    assert A[("imem_words", "256")]["data-chip"] == "fail" and A[("imem_words", "128")]["data-chip"] == "marginal"
    assert A[("n_engines", "3")]["data-chip"] == "marginal"


def test_impact_layout_and_sort(js):
    h = section(render(js), "impact")
    rows = re.findall(r'data-knob-row="([^"]+)" data-flips="(\d)"', h)
    flags = [f for _, f in rows]
    assert flags == sorted(flags, reverse=True) and flags[0] == "1" and "0" in flags          # knobs with a flip first
    assert "env" not in [k for k, _ in rows] and "signoff_corner" not in [k for k, _ in rows]
    assert "No effect at this point" in h and "adpll_mult" not in [k for k, _ in rows]
    assert "chip ok" in h and "marginal" in h and "um2" in h and "MHz" in h                    # chip consequence and deltas
    order = re.findall(r'data-knob-row="([^"]+)"', section(render(js, "{view: {explore: {impactSort: 'order'}}}"), "impact"))
    assert order[0] == "f_core_mhz"
    assert 'data-ex="impactAll"' in h


def test_impact_chip_opens_side_panel(js):
    h = section(render(js), "impact")
    assert re.search(r'data-side="\{&quot;kind&quot;:&quot;impact&quot;,&quot;knob&quot;:&quot;crc_unit&quot;', h)
    side = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'impact', knob: 'host_link', value: 'qspi', col: 0, all: false}}, DATA))")
    assert "Impact: " in side
    assert side.count("&#9650; up") == 4 and "Chip check after the change" in side and "data-ex-apply" in side
    down = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'impact', knob: 'n_engines', value: 1, col: 0, all: false}}, DATA))")
    assert down.count("&#9660; down") == 1 and "served &#8594; impossible" in down
    gone = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'impact', knob: 'host_link', value: 'nope', col: 0}}, DATA))")
    assert "not in the current impact table" in gone


# ------------------------------------------------------------------ what would it take
def test_wwit_can_1m_shows_crc_unit(js):
    h = render(js, focus("can_1m"))
    w = section(h, "wwit")
    assert 'data-result="single"' in w
    rows = cand_rows(h)
    assert len(rows) == 1 and "crc_unit" in rows[0] and 'data-estimate="0"' in rows[0] and 'data-breaks-chip="0"' in rows[0]


def test_wwit_eth10_tx_ranking_and_chip_breakers(js):
    h = render(js, focus("eth10_tx"))
    allrows = cand_rows(h)
    assert "host_link</span> spi &#8594; <b>spi_fast</b>" in allrows[0] and "deep_fifo_words" in "".join(allrows[:3])
    # clean candidates first, then the chip-breakers with the driver, margin and restore list
    brk = [r for r in allrows if 'data-breaks-chip="1"' in r]
    assert brk and all('data-breaks-chip="0"' in r for r in allrows[:len(allrows) - len(brk)])
    assert re.search(r"(breaks|leaves) the chip check( marginal)?: die_area", brk[0]) and "pessimistic" in brk[0]
    assert "restore:" in "".join(brk)
    # estimates after evidence-backed ones among the chip-clean rows, labelled, with their judged inputs
    clean = [r for r in allrows if 'data-breaks-chip="0"' in r]
    est = [i for i, r in enumerate(clean) if 'data-estimate="1"' in r]
    assert est and est == list(range(est[0], est[0] + len(est))) and est[-1] == len(clean) - 1
    assert "estimate" in clean[est[0]] and "judged:" in clean[est[0]] and "param:eta_par" in clean[est[0]]


def test_wwit_pairs_for_spi_target_10m(js):
    h = render(js, focus("spi_target_10m"))
    w = section(h, "wwit")
    assert 'data-result="pairs"' in w and "No single knob serves it" in w
    rows = cand_rows(h)
    assert len(rows) == 3
    assert 'data-which="pairs"' in rows[0] and "qspi" in rows[0] and "lane" in rows[0] and 'data-estimate="0"' in rows[0]
    assert all('data-estimate="1"' in r and "judged:" in r for r in rows[1:])          # nibble / byte rest on a judged input


@pytest.mark.parametrize("wl", ["dvi_250m", "usb_fs_device", "canfd_5m", "eth10_rx", "spi_target_25m", "i3c_target_12m5"])
def test_wwit_out_of_reach(js, wl):
    w = section(render(js, focus(wl)), "wwit")
    assert 'data-result="out_of_reach"' in w and "Out of reach" in w and "ex-block" in w


def test_wwit_dvi_blocking_limits_and_hints(js):
    w = section(render(js, focus("dvi_250m")), "wwit")
    for short in ["pad out", "TX edge", "cycles"]:
        assert short in w
    assert "improves most with" in w and "fine_edge" in w


def test_wwit_already_served_and_side(js, data):
    served = js("Object.keys(ArchSimUI.makeState({}, DATA).all.cells).filter(function (w) { return ArchSimUI.makeState({}, DATA).all.cells[w].verdict === 'served'; })[0]")
    w = section(render(js, focus(served)), "wwit")
    assert 'data-result="already"' in w and "nothing to change" in w
    side = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'wwit', wl: 'can_1m', which: 'single', idx: 0, col: 0}}, DATA))")
    assert "crc_unit" in side and "Chip check" in side and "Collateral" in side and "data-ex-apply" in side
    est = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'wwit', wl: 'spi_target_10m', which: 'pairs', idx: 1, col: 0}}, DATA))")
    assert "estimate" in est and "eta_par" in est and "judgement" in est
    assert "not in the current list" in js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'wwit', wl: 'can_1m', which: 'single', idx: 9, col: 0}}, DATA))")


def test_solve_every_row_of_the_tag(js, data):
    off = render(js, focus("can_1m"))
    assert "Solve every row of the tag" in off and "ex-solve-row" not in off              # only on request
    h = render(js, focus("can_1m", ", view: {explore: {solveAll: true}}"))
    rows = {m[0]: m[1] for m in re.findall(r'<tr class="ex-solve-row" data-wl="([^"]+)" data-kind="([^"]+)"', h)}
    tagged = [w["id"] for w in data["workloads"] if "cap19" in w["tags"]]
    assert sorted(rows) == sorted(tagged) and len(rows) == 19
    assert rows["eth10_tx"] == "single" and rows["can_1m"] == "single" and rows["spi_target_10m"] == "pairs" and rows["dvi_250m"] == "out_of_reach"
    assert "already" in rows.values()


# ------------------------------------------------------------------ memoisation, cost, no other tab
def test_memoised_per_point_and_tag(js):
    r = js("""(function () {
      var cnt = {k: 0, w: 0}, ok = ArchSim.knobImpact, ow = ArchSim.whatWouldItTake;
      ArchSim.knobImpact = function () { cnt.k++; return ok.apply(this, arguments); };
      ArchSim.whatWouldItTake = function () { cnt.w++; return ow.apply(this, arguments); };
      try {
        var raw = {arch: {f_core_mhz: 54}, focus: {wl: 'can_1m'}};
        ArchSimUI.renderExplore(ArchSimUI.makeState(raw, DATA)); var a = [cnt.k, cnt.w];
        ArchSimUI.renderExplore(ArchSimUI.makeState(raw, DATA)); var b = [cnt.k, cnt.w];
        raw.focus.wl = 'eth10_tx'; ArchSimUI.renderExplore(ArchSimUI.makeState(raw, DATA)); var c = [cnt.k, cnt.w];
        raw.arch = {f_core_mhz: 56}; ArchSimUI.renderExplore(ArchSimUI.makeState(raw, DATA)); var d = [cnt.k, cnt.w];
        raw.view = {explore: {impactAll: true}}; ArchSimUI.renderExplore(ArchSimUI.makeState(raw, DATA)); var e = [cnt.k, cnt.w];
        return {a: a, b: b, c: c, d: d, e: e};
      } finally { ArchSim.knobImpact = ok; ArchSim.whatWouldItTake = ow; }
    })()""")
    assert r == {"a": [1, 1], "b": [1, 1], "c": [1, 2], "d": [2, 3], "e": [3, 3]}


def test_other_tabs_never_compute_explore_solvers(js):
    r = js("""(function () {
      var n = 0, ok = ArchSim.knobImpact, ow = ArchSim.whatWouldItTake;
      ArchSim.knobImpact = function () { n++; return ok.apply(this, arguments); };
      ArchSim.whatWouldItTake = function () { n++; return ow.apply(this, arguments); };
      try {
        ['capability', 'timing', 'area', 'throughput', 'trust'].forEach(function (t) { ArchSimUI.renderView(ArchSimUI.makeState({tab: t, arch: {f_core_mhz: 58}}, DATA)); });
        ArchSimUI.makeState({tab: 'explore', arch: {f_core_mhz: 58}}, DATA);          // building the state computes nothing either
        return n;
      } finally { ArchSim.knobImpact = ok; ArchSim.whatWouldItTake = ow; }
    })()""")
    assert r == 0


def test_explore_render_time_is_bounded(js):
    t = time.perf_counter()
    render(js, "{arch: {f_core_mhz: 52}, focus: {wl: 'spi_target_10m'}}")
    assert time.perf_counter() - t < 6          # QuickJS; ~0.5 s measured, a browser is faster


# ------------------------------------------------------------------ sweep
def sweep_html(js, wl="spi_target_scalable", extra=""):
    return section(render(js, focus(wl, extra)), "sweep")


def test_sweep_f_core_max_rate_16_points_and_chip_fail(js):
    s = sweep_html(js)
    assert 'data-sweep-knob="f_core_mhz"' in s and 'data-sweep-metric="max_rate:spi_target_scalable"' in s and 'data-npoints="16"' in s
    pts = [attrs(m) for m in re.findall(r'<circle class="pt[^>]*>', s)]
    xs = [float(p["data-x"]) for p in pts]
    assert xs == [float(x) for x in range(40, 71, 2)]
    y = [float(p["data-y-nom"]) for p in pts]
    assert all(b > a for a, b in zip(y, y[1:]))
    fails = [float(attrs(m)["data-chip-fail"]) for m in re.findall(r'<rect class="chipfail"[^>]*>', s)]
    assert fails == [float(x) for x in range(62, 71, 2)]
    assert [p["data-chip"] == "fail" for p in pts] == [x >= 62 for x in xs]
    assert "chip fails (core_clock)" in s


def test_sweep_metric_choices_are_scoped(js):
    s = sweep_html(js, "can_1m")
    opts = re.findall(r'<option value="([^"]+)"', s)
    assert "max_rate:can_1m" in opts and any(o.startswith("margin:can_1m:") for o in opts)
    assert "served_count:cap19" in opts and "marginal_count:cap19" in opts and "chip_margin:core_clock" in opts and "util_pct" in opts
    assert "link_write_MBps" in opts and "link_read_MBps" in opts
    assert not [o for o in opts if o.startswith(("max_rate:", "margin:")) and ":can_1m" not in o]       # never a workload list
    # a stored metric for another protocol falls back to the focused one
    s2 = section(render(js, focus("can_1m", ", view: {explore: {sweep: {metric: 'max_rate:uart_3m'}}}")), "sweep")
    assert 'data-sweep-metric="max_rate:can_1m"' in s2


def test_sweep_other_knob_and_metrics(js):
    s = section(render(js, focus("can_1m", ", view: {explore: {sweep: {knob: 'host_link', metric: 'chip_margin:die_area'}}}")), "sweep")
    assert 'data-sweep-knob="host_link"' in s and 'data-npoints="6"' in s and s.count('class="whisker"') == 6
    s = section(render(js, focus("can_1m", ", view: {explore: {sweep: {knob: 'n_engines', values: [1, 2, 3, 4], metric: 'served_count:cap19'}}}")), "sweep")
    ys = [(attrs(m)["data-x"], attrs(m)["data-y-nom"], attrs(m)["data-chip"]) for m in re.findall(r'<circle class="pt[^>]*>', s)]
    assert ys == [("1", "3", "pass"), ("2", "4", "pass"), ("3", "4", "marginal"), ("4", "4", "marginal")]
    assert 'data-npoints="4"' in s and "data-chip-fail" not in s


def test_sweep_default_grid_text(js):
    s = sweep_html(js)
    m = re.search(r'value="(40, 42[^"]*)"', s)
    assert m and m.group(1).split(", ") == [str(v) for v in range(40, 71, 2)]


def test_sweep_point_side(js):
    side = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'sweep_point', knob: 'f_core_mhz', x: 64, metric: 'max_rate:spi_target_scalable', col: 0}}, DATA))")
    assert "Sweep point" in side and "64" in side and "chip fail" in side.lower().replace("✗ ", "chip ") or "fail" in side
    assert "binding" in side and "data-ex-apply" in side


# ------------------------------------------------------------------ Pareto
def test_pareto_presets(js, data):
    p = section(render(js), "pareto")
    pts = [attrs(m) for m in re.findall(r'<(?:circle|polygon)[^>]*class="pp [^>]*>', p)]
    presets = [q for q in pts if q["data-kind"] == "preset"]
    assert len(presets) == 9 == len(data["presets"]) and 'data-front="stretch_usb_sd,wide_host"' in p
    by = {q["data-id"]: q for q in pts}
    assert by["wide_host"]["data-chip"] == "marginal" and by["wide_host"]["data-pareto"] == "front"
    assert by["mvp"]["data-pareto"] == "dominated" and by["stretch_usb_sd"]["data-pareto"] == "front"
    assert by["live"]["data-kind"] == "live" and by["live"]["data-pareto"] == "dominated"
    assert round(float(by["stretch_usb_sd"]["data-x"]), 2) == 36.64 and by["stretch_usb_sd"]["data-y"] == "5"
    assert round(float(by["mvp"]["data-x"]), 2) == 37.95 and by["mvp"]["data-y"] == "4"
    assert "[chip marginal]" in p


def test_pareto_fail_is_hollow_and_excluded(js):
    # a pinned point on a failing chip (64 MHz) is listed but never on the front
    p = section(render(js, "{pinned: [{arch: {f_core_mhz: 64}, label: 'p64'}]}"), "pareto")
    pts = {attrs(m)["data-id"]: attrs(m) for m in re.findall(r'<(?:circle|polygon)[^>]*class="pp [^>]*>', p)}
    assert pts["pin1"]["data-pareto"] == "chip-fail" and pts["pin1"]["data-chip"] == "fail" and pts["pin1"]["data-kind"] == "pinned"
    assert 'data-front="stretch_usb_sd,wide_host"' in p


# ------------------------------------------------------------------ evidence rule / no duplication
def test_no_duplicated_information_and_no_workload_list(js):
    h = render(js)
    assert "data-tab=\"capability\"" in h                                  # links to Capability instead of repeating verdict tables
    assert "Not modelled" not in h and "derivation" not in h.lower().split("<section")[0]
    assert h.count("<table") >= 2 and "ex-sweep-table" not in h

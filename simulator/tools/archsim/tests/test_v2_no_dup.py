"""PLAN-v2 Part 3 (duplication audit): every concept has one home. Renders every view (plus the chip card, the rail, the
focus bar and the side panel) for the MVP and for a pinned + scenario state and asserts that each concept's marker
appears in its home view only. Markers are computed from the model (never typed numbers), and each is first checked to
be present in its home so a renamed label cannot make the test pass vacuously."""
import re

import pytest

TABS = ["capability", "timing", "area", "throughput", "explore", "trust"]
STATES = {
    "mvp": "{}",
    "pinned_scenario": "{pinned: [{arch: {n_engines: 1}, label: 'one'}, {arch: {f_core_mhz: 60}, label: 'fast'}],"
                       " scenario: ['spi_target_10m', 'usb_fs_tx'], view: {capability: {sub: 'scenario'}}}",
}


def _text(h):
    h = re.sub(r"<(script|style)[^>]*>.*?</\1>", "", h, flags=re.S)
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", " ", h))


def _views(js, raw):
    st = "ArchSimUI.makeState(%s, DATA)" % raw
    out = {t: js("ArchSimUI.render%s(%s)" % (t.capitalize(), st)) for t in TABS}
    out["chip"] = js("ArchSimUI.renderChip(%s)" % st)
    out["rail"] = js("ArchSimUI.renderRail(%s, {open: {}})" % st)
    out["focus"] = js("ArchSimUI.renderFocus(%s)" % st)
    return out


@pytest.fixture(scope="module", params=list(STATES))
def views(request, js):
    return request.param, _views(js, STATES[request.param])


def _fmt(js, expr, digits=None):
    return js("ArchSimUI.fmtNum(%s%s)" % (expr, "" if digits is None else ", %d" % digits))


def _rec(js, raw, rid):
    return "ArchSimUI.makeState(%s, DATA).derived.records.%s.value.nom" % (raw, rid)


def _homes(vs, needle, html=False):
    return sorted(k for k, h in vs.items() if needle in (h if html else _text(h)))


def _check(vs, needle, home, html=False):
    got = _homes(vs, needle, html)
    assert got == sorted(home), "%r found in %s, expected only in %s" % (needle, got, sorted(home))


def test_link_write_mbps_only_in_throughput(js, views):
    name, vs = views
    v = _fmt(js, _rec(js, STATES[name], "link_write_MBps"), 4)
    _check(vs, v + " MB/s", ["throughput"])


def test_host_loop_and_imem_load_only_in_throughput(js, views):
    name, vs = views
    for rid in ("t_host_loop", "t_imem_load_32"):
        v = _fmt(js, _rec(js, STATES[name], rid), 2)
        got = _homes(vs, v + " us")
        assert got in ([], ["throughput"]), (rid, got)


def test_reflex_segments_only_in_timing(js, views):
    _, vs = views
    for label in ("synchronizer decision", "skew + external setup", "output register"):
        _check(vs, label, ["timing"])


def test_area_total_and_usable_only_in_area(js, views):
    name, vs = views
    for rid, field in (("area_total", "value"), ("area_limit", "value"), ("core_area", "value")):
        v = js("ArchSimUI.makeState(%s, DATA).derived.records.%s.%s" % (STATES[name], rid, field))
        v = v["nom"] if isinstance(v, dict) else v
        _check(vs, _fmt(js, repr(v), 0), ["area"])


def test_block_area_lines_only_in_area(js, views):
    _, vs = views
    _check(vs, "basis synth x pnr_factor", ["area"])
    # the Configure diagram encodes area as box size only: no area figure in the rail
    assert "um2" not in _text(vs["rail"]) and "%" not in _text(vs["rail"]).replace("%s", "")


def test_chip_rows_only_in_chip_card(js, views):
    _, vs = views
    _check(vs, "data-chip-item=", ["chip"], html=True)
    for it in ("core_clock", "die_area", "board_clock", "pins_budget"):
        _check(vs, 'data-chip-item="%s"' % it, ["chip"], html=True)


def test_chip_margins_not_repeated(js, views):
    name, vs = views
    for it in ("core_clock", "board_clock"):
        v = js("ArchSimUI.makeState(%s, DATA).chip.items.filter(function (i) { return i.id === '%s'; })[0].value.nom" % (STATES[name], it))
        _check(vs, _fmt(js, repr(v)) + " MHz", ["chip"])
    # die_area margin (limit minus use): the chip card only
    v = js("ArchSimUI.makeState(%s, DATA).chip.items.filter(function (i) { return i.id === 'die_area'; })[0].value.nom" % STATES[name])
    _check(vs, _fmt(js, repr(v), 0), ["chip"])


def test_core_clock_composition_only_in_timing(js, views):
    _, vs = views
    _check(vs, "base path (1000 / F_s / r_typ)", ["timing"])
    _check(vs, "Core clock vs fmax", ["timing"])
    # fmax is not a Capability column header any more
    assert "fmax" not in _text(vs["capability"]).lower()


def test_calibration_rows_only_in_trust(js, views):
    _, vs = views
    for needle in ("harness defect in perf-b", "perf-a-vs-b.md", "research/perf/b/perf.csv"):
        _check(vs, needle, ["trust"])
    _check(vs, "Calibration: model against measured", ["trust"])


def test_not_modelled_full_list_only_in_trust(js, views, data):
    _, vs = views
    for item in data["not_modelled"]["items"][:6]:
        _check(vs, item["text"][:40], ["trust"])


def test_how_to_read_definitions_only_in_capability(js, views):
    name, vs = views
    marker = "the smallest-margin limit among those that block the cell"
    if name == "mvp":
        _check(vs, marker, ["capability"])
    else:                                   # scenario sub-view: the matrix (and its definition) is not drawn
        assert _homes(vs, marker) in ([], ["capability"])
    assert marker not in _text(vs["explore"]) + _text(vs["trust"]) + _text(vs["timing"])


def test_verdict_counts_only_in_capability_header(js):
    """D5: the served / marginal / impossible tally is a Capability sentence; Explore may name its base once as context."""
    vs = _views(js, "{}")
    tally = re.compile(r"\d+ served, ~? ?\d+ marginal, \S* ?\d+ impossible")
    homes = sorted(k for k, h in vs.items() if tally.search(_text(h)))
    assert homes == ["capability", "explore"], homes


def test_explore_baseline_is_context_not_tally(js):
    """Explore needs its base for the flip lists; it may name the base counts once, as a context line, nowhere else."""
    t = _text(_views(js, "{}")["explore"])
    assert len(re.findall(r"\d+ served, \d+ marginal, \d+ impossible", t)) <= 1


def test_focus_pickers_only_in_focus_bar(js, views):
    _, vs = views
    _check(vs, "data-focus-pick", ["focus"], html=True)


def test_scenario_available_links_to_throughput(js):
    h = _views(js, STATES["pinned_scenario"])["capability"]
    assert 'data-tab="throughput"' in h
    # the scenario table does not restate the link figures
    link = _fmt(js, _rec(js, STATES["pinned_scenario"], "link_write_MBps"), 4)
    assert link not in _text(h)


def test_pin_budget_count_in_chip_and_allocation_in_area(js):
    vs = _views(js, "{}")
    assert "18.0 pins" in _text(vs["chip"])
    assert "18.0 pins" not in _text(vs["area"]) and "pins_budget" in vs["area"]    # area links to the check, no margin


def test_side_panel_param_calibration_labels(js):
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'calibration', id: 'K1'}}, DATA))")
    assert "all points" in h


def test_timing_strip_only_in_timing(js, views):
    """The cross-point timing strip (reflex/response margin and binding timing limit per point) is a Timing section."""
    name, vs = views
    _check(vs, "data-timing-strip", ["timing"], html=True)
    strip = re.search(r"<section[^>]*data-timing-strip.*?</section>", vs["timing"], re.S).group(0)
    assert strip.count("data-strip-point=") == (1 if name == "mvp" else 3)
    for seg in ("synchronizer decision", "skew + external setup", "output register"):   # no per-segment budget in the strip
        assert seg not in strip


def test_concurrency_views_only_in_capability_scenario(js, views):
    """reference-mix table, pair matrix + tally and the pair verdict wording live in Capability's Scenario mode only"""
    name, vs = views
    for needle in ("pairs run together", "Reference mixes on each point", "Pairwise compatibility"):
        got = _homes(vs, needle)
        assert got == (["capability"] if name == "pinned_scenario" else []), (needle, got)
    for needle in ("pairs run together", "Pairwise compatibility", "can run together"):
        assert needle not in _text(js("ArchSimUI.renderSide(ArchSimUI.makeState({}, DATA))"))

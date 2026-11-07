"""The UI core without a DOM (QuickJS): registry, shared selection, side panel, focus bar, helpers, layout."""
import json
import re

from conftest import ROOT

TABS = ["capability", "timing", "area", "throughput", "explore", "trust"]
V1_HASH = '{"a":{"f_core_mhz":60},"t":"budgets","s":{"wl":"usb_fs_tx","mix":["ws2812"]}}'


def test_tab_order_and_registry(js):
    assert js("ArchSimUI.TAB_ORDER") == TABS
    assert js("Object.keys(ArchSimUI.views).filter(function (k) { return k !== 'configure'; })") == TABS
    assert js("'configure' in ArchSimUI.views")
    for t in TABS:
        fn = "render" + t.capitalize()
        h = js(f"ArchSimUI.{fn}(ArchSimUI.makeState({{}}, DATA))")
        assert isinstance(h, str) and h
        assert js(f"ArchSimUI.renderView(ArchSimUI.makeState({{tab: {t!r}}}, DATA))") == h
    assert js("ArchSimUI.TAB_ORDER.map(function (t) { return ArchSimUI.views[t].title; })")[2] == "Area & pins"


def test_state_shape(js, data):
    st = js("(function () { var s = ArchSimUI.makeState({pinned: [{arch: {f_core_mhz: 60}, label: 'p60'}]}, DATA);"
            " return {n: s.points.length, labels: s.points.map(function (p) { return p.label; }), focus: s.focus,"
            " chip: s.points.map(function (p) { return p.chip.verdict; }), cellwl: s.cell.wl, hasAll: !!s.points[1].all.cells,"
            " tab: s.tab, tag: s.tag}; })()")
    assert st["n"] == 2 and st["labels"] == ["live", "p60"] and st["tab"] == "capability" and st["tag"] == "cap19"
    assert st["focus"] == {"wl": "spi_target_10m", "col": 0} and st["cellwl"] == "spi_target_10m" and st["hasAll"]
    assert st["chip"] == ["pass", "pass"]
    # focus on a pinned point and on a workload outside the tag (evaluated on demand)
    wl_out = next(w["id"] for w in data["workloads"] if "cap19" not in w["tags"])
    r = js(f"(function () {{ var s = ArchSimUI.makeState({{pinned: [{{arch: {{f_core_mhz: 64}}}}], focus: {{wl: {wl_out!r}, col: 1}}}}, DATA);"
           " return {col: s.focus.col, label: s.point.label, wl: s.cell.wl, chip: s.point.chip.verdict}; })()")
    assert r == {"col": 1, "label": "point 1", "wl": wl_out, "chip": "fail"}
    # bad focus values fall back
    assert js("ArchSimUI.makeState({focus: {wl: 'nope', col: 9}}, DATA).focus") == {"wl": "spi_target_10m", "col": 0}


def test_state_roundtrip_v2(js):
    raw = ("{arch: {f_core_mhz: 60, n_engines: 3}, tab: 'explore', focus: {wl: 'usb_fs_tx', col: 1}, scenario: ['ws2812'],"
           " pinned: [{arch: {f_core_mhz: 64}, label: 'hot'}], side: {kind: 'chip', item: 'core_clock', col: 1}, view: {explore: {m: 'x'}}, preset: 'mvp'}")
    enc = js(f"ArchSimUI.encodeState(ArchSimUI.makeState({raw}, DATA))")
    o = json.loads(enc)
    assert o["v"] == 2 and o["a"] == {"f_core_mhz": 60, "n_engines": 3} and o["t"] == "explore"
    assert o["f"] == {"wl": "usb_fs_tx", "col": 1} and o["x"] == ["ws2812"] and o["n"] == [{"a": {"f_core_mhz": 64}, "l": "hot"}]
    assert o["s"]["kind"] == "chip" and o["w"] == {"explore": {"m": "x"}} and "g" not in o
    back = js(f"ArchSimUI.decodeState({enc!r})")
    assert back["arch"] == o["a"] and back["tab"] == "explore" and back["focus"] == o["f"] and back["scenario"] == ["ws2812"]
    assert back["pinned"] == [{"arch": {"f_core_mhz": 64}, "label": "hot"}] and back["side"] == o["s"]
    again = js(f"ArchSimUI.encodeState(ArchSimUI.makeState(ArchSimUI.decodeState({enc!r}), DATA))")
    assert json.loads(again) == o
    d = json.loads(js("ArchSimUI.encodeState(ArchSimUI.makeState({}, DATA))"))
    assert d == {"v": 2, "a": {}}                                   # defaults are not written
    assert js("ArchSimUI.decodeState('not json')") == {}


def test_decode_v1_hash(js):
    back = js(f"ArchSimUI.decodeState({V1_HASH!r})")
    assert back["tab"] == "timing" and back["focus"]["wl"] == "usb_fs_tx"
    assert back["scenario"] == ["ws2812"] and back["arch"] == {"f_core_mhz": 60}
    s = js(f"(function () {{ var s = ArchSimUI.makeState(ArchSimUI.decodeState({V1_HASH!r}), DATA); return {{tab: s.tab, wl: s.focus.wl, sc: s.scenario, f: s.arch.f_core_mhz}}; }})()")
    assert s == {"tab": "timing", "wl": "usb_fs_tx", "sc": ["ws2812"], "f": 60}
    for old, new in [("matrix", "capability"), ("budgets", "timing"), ("architecture", "timing"), ("sweep", "explore"),
                     ("calibration", "trust"), ("inputs", "trust")]:
        assert js(f"ArchSimUI.makeState(ArchSimUI.decodeState('{{\"t\":\"{old}\"}}'), DATA).tab") == new


def test_focus_bar(js, data):
    h = js("ArchSimUI.renderFocus(ArchSimUI.makeState({}, DATA))")
    assert 'data-focus-pick="wl"' in h and 'data-focus-pick="col"' in h
    assert h.count("data-verdict=") == 1 and "<optgroup" in h and "showing pinned" not in h.lower()
    assert h.count("<option") >= len(data["workloads"])
    assert 'value="spi_target_10m" selected' in h
    h1 = js("ArchSimUI.renderFocus(ArchSimUI.makeState({pinned: [{arch: {}, label: 'A'}], focus: {col: 1}}, DATA))")
    assert "Showing pinned point <b>A</b>" in h1 and "knobs edit the live point" in h1 and 'data-action="load-point"' in h1
    assert h1.count("data-verdict=") == 1


def test_side_dispatch_and_core_kinds(js):
    assert js("ArchSimUI.renderSide(ArchSimUI.makeState({}, DATA))") == ""
    reg = ("(function () { ArchSimUI.registerSide('t_demo', function (s, ref) { return {title: 'Demo ' + ref.x, html: '<p id=\"d\">' + ref.x + ' ' + s.focus.wl + '</p>'}; });"
           " return ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 't_demo', x: 'hi'}}, DATA)); })()")
    h = js(reg)
    assert "Demo hi" in h and '<p id="d">hi spi_target_10m</p>' in h and "data-side-close" in h
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'record', id: 'T', col: 0}}, DATA))")
    assert "data-side-close" in h and "mx-rec" in h and "live point" in h
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'record', id: 'link_write_MBps', col: 0}}, DATA))")
    assert "mx-rec" in h and "0.699" in h
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'param', id: 'd_die_8x4_ns'}}, DATA))")
    assert "Extra delay on the 8x4 die" in h and "mx-src" in h and "judgement" in h
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'chip', item: 'core_clock', col: 0}}, DATA))")
    assert "mx-rec" in h and "pass" in h and "driver" in h and "chip check of the live point" in h
    assert "11.6" in h
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({pinned: [{arch: {f_core_mhz: 64}, label: 'hot'}], side: {kind: 'chip', item: 'core_clock', col: 1}}, DATA))")
    assert "pinned point hot" in h and 'data-chip-verdict="fail"' in h
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'nope'}}, DATA))")
    assert "No renderer registered" in h and "data-side-close" in h
    h = js("(function () { ArchSimUI.registerSide('t_boom', function () { throw new Error('boom'); });"
           " return ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 't_boom'}}, DATA)); })()")
    assert "render-error" in h and "boom" in h and "data-side-close" in h
    # a side ref whose point is gone is dropped
    assert js("ArchSimUI.makeState({side: {kind: 'record', id: 'T', col: 3}}, DATA).side") is None


def test_resolve_ref_isa_effect(js):
    r = js("ArchSimUI.resolveRef({sym: 'x', ref: {kind: 'isa_effect', wl: 'can_1m', lever: 'isa_xor'}}, {data: {isa: {effects: ["
           "{wl: 'can_1m', lever: 'isa_xor', method: 'judgement', status: 'estimate', rationale: 'because', src: 'fw/can/x', program: null}]}}, records: {}})")
    assert r["id"] == "isa_xor@can_1m" and r["method"] == "judgement" and r["status"] == "estimate"
    assert r["sources"] == ["fw/can/x"] and r["assumptions"] == ["because"]
    r = js("ArchSimUI.resolveRef({ref: {kind: 'isa_effect', wl: 'a', lever: 'b'}}, {data: {}, records: {}})")
    assert r["kind"] == "isa_effect" and "method" not in r


def test_helpers(js):
    assert js("ArchSimUI.fmtNum(342472)") == "342,472"
    assert js("ArchSimUI.fmtBand({nom: 50, pess: 40, opt: 60}, 'MHz')") == "50.0 MHz (40.0..60.0)"
    assert js("ArchSimUI.fmtBand({nom: 5, pess: 5, opt: 5}, 'MHz')") == "5.00 MHz"            # degenerate band collapses
    assert js("ArchSimUI.fmtBandHtml({nom: 5, pess: 5, opt: 5}, 'MHz')").count("lo num") == 0
    assert js("ArchSimUI.esc('<a&\"b>')") == "&lt;a&amp;&quot;b&gt;"
    assert 'data-verdict="served"' in js("ArchSimUI.verdictBadge('served')")
    assert 'data-chip-verdict="fail"' in js("ArchSimUI.chipBadge('fail')")
    h = js("ArchSimUI.statusBadge('estimate')")
    assert "st-estimate" in h and "estimate" in h
    js("ArchSimUI.makeState({}, DATA)")                                                       # primes the vocabulary
    assert "judgement anchored" in js("ArchSimUI.statusBadge('estimate')")
    assert "tabular" not in js("ArchSimUI.stat('k', 'v', 's')") and "data-side" not in js("ArchSimUI.stat('k', 'v', 's')")
    st = js("ArchSimUI.stat('k', 'v', 's', 'T')")
    assert "<button" in st and "&quot;kind&quot;:&quot;record&quot;" in st
    assert js("ArchSimUI.sideAttr({kind: 'a', id: \"x'y\"})").startswith(' data-side="')
    assert js("ArchSimUI.focusAttr('w', 2)") == ' data-focus-wl="w" data-focus-col="2"'
    assert js("ArchSimUI.niceTicks(0, 10, 5)") == [0, 2, 4, 6, 8, 10]
    assert js("ArchSimUI.eng(1500000)") == "1.5M"
    assert js("ArchSimUI.workloadOf(DATA, 'usb_fs_tx').id") == "usb_fs_tx"
    assert "<svg" in js("ArchSimUI.svgWrap(10, 10, '', 'x')") and "<table" in js("ArchSimUI.table(['a'], [['b']])")
    assert "mx-bar" in js("ArchSimUI.pessOptBar(-1, 1, 2, 4, 'm')") and js("typeof ArchSimUI.mark") == "function"
    assert js("ArchSimUI.tagsOf(DATA)")[0] == "cap19" and js("ArchSimUI.tagsOf(DATA)")[-1] == "all"


def test_deriv_tree_renders_chip_record(js):
    h = js("(function () { var s = ArchSimUI.makeState({}, DATA); return ArchSimUI.derivTree(s.chip.records.die_area, ArchSimUI.derivCtx(s, 0)); })()")
    assert "mx-rec" in h and "mx-terms" in h


def test_layout_skeleton():
    for text in ((ROOT / "ui/template.html").read_text(), (ROOT / "archsim.html").read_text()):
        for i in ("rail", "chip", "focus", "tabs", "view", "side"):
            assert text.count(f'id="{i}"') == 1, i
        assert 'id="detail"' not in text and 'id="summary"' not in text
    t = (ROOT / "ui/template.html").read_text()
    assert t.index('id="chip"') < t.index('id="rail"') < t.index('id="focus"') < t.index('id="view"') < t.index('id="side"')


def test_responsive_css_rules():
    css = (ROOT / "ui/archsim.css").read_text()
    for needle in ("1279px", "899px", "599px", "grid-template-columns: 340px", "400px", "body.has-side", "body.panel-open",
                   "#chip .chip-body", "--hatch", ".chip-fail"):
        assert needle in css, needle


def test_css_files_order_and_stubs():
    from tools.archsim.build_html import CSS_FILES, UI_FILES, build
    assert CSS_FILES[0] == "archsim.css" and len(CSS_FILES) == 8 and UI_FILES[0] == "archsim-ui.js" and UI_FILES[-1] == "boot.js"
    for f in CSS_FILES + ["ui/" + u for u in UI_FILES]:
        assert (ROOT / "ui" / f).exists() or (ROOT / f).exists(), f
    h = build()
    assert h.index("/* Configure styles") < h.index("/* Trust styles")
    for v in ("configure", "capability", "timing", "area", "throughput", "explore", "trust"):
        assert (ROOT / f"ui/view-{v}.js").exists() and (ROOT / f"ui/css/{v}.css").exists()
    assert not re.search(r"view-(matrix|budgets|sweep)", " ".join(UI_FILES))


def test_resolve_ref_isa_encoding_and_side_links(js):
    r = js("ArchSimUI.resolveRef({ref: {kind: 'isa_encoding', bits: 16}}, {data: {isa: {encoding: {16: {cite: [{path: 'docs/x', quote: 'q'}]}}}}, records: {}, col: 1})")
    assert r["id"] == "16-bit" and r["side"] == {"kind": "chip", "item": "isa_encoding", "col": 1} and r["sources"] == ["docs/x: q"]
    r = js("ArchSimUI.resolveRef({ref: {kind: 'isa_effect', wl: 'a', lever: 'b'}}, {data: {}, records: {}})")
    assert r["side"] == {"kind": "isa_effect", "wl": "a", "lever": "b"}


def test_side_header_label_without_point(js):
    h = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'isa_effect', wl: 'a', lever: 'b'}}, DATA))")
    assert "all points" in h

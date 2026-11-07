"""Configure view (rail, chip check card, compact block diagram, side kind 'block') under QuickJS."""
import re

from conftest import ROOT


def st(arch="{}", extra=""):
    return f"ArchSimUI.makeState({{arch: {arch}{extra}}}, DATA)"


def test_one_control_per_knob_and_groups(js):
    html = js(f"ArchSimUI.renderConfigure({st()}, {{}})")
    knobs = js("ArchSim.KNOBS.map(function (k) { return [k.id, k.group]; })")
    assert knobs
    for kid, _ in knobs:
        assert html.count(f'data-knob="{kid}"') == 1, kid
    # evaluation knobs live under their own group, everything else under its KNOBS group
    groups = re.findall(r'data-group="([^"]+)"', html)
    assert groups[-1] == "Evaluation settings"
    ev = html.split('data-group="Evaluation settings"')[1]
    assert 'data-knob="env"' in ev and 'data-knob="signoff_corner"' in ev
    before = html.split('data-group="Evaluation settings"')[0]
    assert 'data-knob="env"' not in before and 'data-knob="signoff_corner"' not in before
    for g in js("ArchSim.GROUPS"):
        if any(k[1] == g and k[0] not in ("env", "signoff_corner") for k in knobs):
            assert f'data-group="{g.replace("&", "&amp;")}"' in html, g


def test_isa_group_generic_with_estimate_label(js):
    isa = [k for k, g in js("ArchSim.KNOBS.map(function (k) { return [k.id, k.group]; })") if g == "ISA"]
    html = js(f"ArchSimUI.renderKnobPanel({st()})")
    assert "Reset to MVP" in html and "data-preset" in html
    if not isa:  # the ISA lever group missing: the group appears by itself once KNOBS has it
        assert 'data-group="ISA"' not in html
        return
    seg = html.split('data-group="ISA"')[1].split("</details>")[0]
    assert "estimate" in seg and 'data-tab="trust"' in seg
    for k in isa:
        assert f'data-knob="{k}"' in seg


def test_preset_row_and_points(js, data):
    html = js(f"ArchSimUI.renderConfigure({st()}, {{}})")
    assert html.count('data-action="load-preset"') == 1 and html.count('data-action="pin-preset"') == 1
    assert 'data-action="pin"' in html and 'data-action="reset"' in html
    for p in data["presets"]:
        assert f'value="{p["id"]}"' in html
    assert "data-autoload" not in html and "data-preset" in html
    r = js("ArchSimUI.renderConfigure(ArchSimUI.makeState({pinned: [{arch: {f_core_mhz: 64}, label: 'hot'}, {arch: {}, label: 'p2'}], "
           "focus: {col: 1}}, DATA), {})")
    assert r.count('class="pt') == 3 or r.count("data-point=") == 3
    assert 'data-action="load-point" data-value="1"' in r and 'data-action="remove-point" data-value="2"' in r
    assert 'data-focus-col="1"' in r and 'aria-current="true"' in r and "hot" in r
    # the chip verdict of the failing pinned point is shown in the list
    pts = r.split('class="cfg-points"')[1].split("cfg-diagram")[0]
    assert "fail" in pts and "pass" in pts
    # pin buttons disabled at the maximum of 5 pinned points
    five = js("ArchSimUI.renderConfigure(ArchSimUI.makeState({pinned: [{},{},{},{},{}]}, DATA), {})")
    assert five.count("disabled") >= 2


def chip(js, arch="{}", pinned=""):
    return js(f"ArchSimUI.renderChip({st(arch)})")


def test_chip_card_mvp(js, data):
    h = chip(js)
    assert 'class="chip-head" data-open-rail' in h and 'class="chip-body"' in h
    head = h.split('class="chip-body"')[0]
    assert 'data-chip-verdict="pass"' in head and "tightest: core clock" in head
    items = js("ArchSim.chipCheck(ArchSim.defaultArch(), DATA).items.map(function (i) { return i.id; })")
    assert len(items) >= 4
    for i in items:
        assert f'data-chip-item="{i}"' in h
    assert h.count('data-chip-verdict="pass"') == len(items) + 1
    assert h.count("badge driver") == 1 and h.index("badge driver") > h.index('data-chip-item="core_clock"')
    assert h.count('"kind":"chip"') == 0  # ref JSON is attribute-escaped
    assert h.count("&quot;kind&quot;:&quot;chip&quot;") == len(items)
    # margins with band for die_area, collapsed when degenerate
    assert "289,220" in h and "156,769" in h and "408,350" in h
    assert "11.6 MHz" in h and "16.5 MHz" in h
    # breakdown links
    assert h.count('data-tab="timing"') == 1 and h.count('data-tab="area"') == 2
    row = h.split('data-chip-item="board_clock"')[1].split("</div>")[0]
    assert "data-tab" not in row


def test_chip_card_stack_a_and_fail(js):
    h = js("ArchSimUI.renderChip(ArchSimUI.makeState({arch: ArchSim.applyPreset('stack_a', DATA)}, DATA))")
    head = h.split('class="chip-body"')[0]
    assert 'data-chip-verdict="marginal"' in head and "driven by die area" in head
    assert 'chip-row marginal" data-chip-item="die_area"' in h
    f = chip(js, "{f_core_mhz: 64}")
    head = f.split('class="chip-body"')[0]
    assert 'data-chip-verdict="fail"' in head and "driven by core clock" in head
    assert 'chip-row fail" data-chip-item="core_clock"' in f


def svg_of(h):
    return h[h.index("<svg"):h.index("</svg>") + 6]


def test_diagram(js):
    d = js(f"ArchSimUI.renderDiagram({st()})")
    svg = svg_of(d)
    assert len(re.findall(r'class="blk b-engine[ "]', svg)) == 2 and "not a floorplan" in svg
    assert "um2" not in svg and "um²" not in svg
    assert len(re.findall(r'class="blk b-engine[ "]', svg_of(js(f"ArchSimUI.renderDiagram({st('{n_engines: 3}')})")))) == 3
    # click opens the block side panel
    assert "&quot;kind&quot;:&quot;block&quot;" in svg and 'data-block="pinmux"' in svg
    # disabled blocks drawn off
    assert 'b-clock off' in svg or "off" in svg


def test_diagram_outlines(js):
    st_expr = st()
    r = js(f"(function () {{ var s = {st_expr}; return {{b: s.cell.binding, d: s.chip.driver, h: ArchSimUI.renderDiagram(s)}}; }})()")
    svg = svg_of(r["h"])
    assert r["d"] == "core_clock"
    # core_clock has no block; the chip driver of stack_a is die_area -> every enabled area-bearing box outlined
    h = js("ArchSimUI.renderDiagram(ArchSimUI.makeState({arch: ArchSim.applyPreset('stack_a', DATA)}, DATA))")
    assert "chip-driver" in svg_of(h)
    # binding box is outlined: engines hold e.g. cycles/program_words
    b = js("(function () { var s0 = ArchSimUI.makeState({}, DATA); var wl = Object.keys(s0.all.cells).filter(function (w) { return ['cycles', 'program_words', 'engines'].indexOf(s0.all.cells[w].binding) >= 0; })[0];"
           " var s = ArchSimUI.makeState({focus: {wl: wl}}, DATA); var c = s.cell.binding;"
           " var bd = ArchSim.blockDiagram(s.arch, DATA); var hit = bd.blocks.filter(function (x) { return x.enabled && x.limits.indexOf(c) >= 0; }).length;"
           " return {hit: hit, n: (ArchSimUI.renderDiagram(s).match(/blk[^\"]* binding/g) || []).length}; })()")
    assert b["hit"] == b["n"]
    assert b["hit"] > 0 and b["n"] > 0


def test_side_block(js):
    out = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'block', id: 'engine_0', col: 0}}, DATA))")
    assert "Engine 0" in out and "limits this block takes part in" in out and "data-tab=\"area\"" in out
    assert "chip-check items" in out and "&quot;kind&quot;:&quot;chip&quot;" in out
    missing = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'block', id: 'nope', col: 0}}, DATA))")
    assert "No block" in missing


def test_registered_and_no_hex(js):
    assert js("typeof ArchSimUI.views.configure.render") == "function" and js("typeof ArchSimUI.views.configure.renderChip") == "function"
    assert js("ArchSimUI.views.configure.render(ArchSimUI.makeState({}, DATA), {open: {}}) === ArchSimUI.renderConfigure(ArchSimUI.makeState({}, DATA), {open: {}})")
    for f in ("ui/view-configure.js", "ui/css/configure.css"):
        txt = (ROOT / f).read_text()
        assert not re.search(r"#[0-9a-fA-F]{3,8}\b(?!-)", re.sub(r"&#\d+;", "", txt)), f
        assert not re.search(r"rgba?\(", txt), f


def test_chip_numbers_only_in_card(js):
    rail = js(f"ArchSimUI.renderConfigure({st()}, {{}})")
    assert "289,220" not in rail and "11.6 MHz" not in rail

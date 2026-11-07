"""Area & pins view (QuickJS): waterfall, pin strip, area_line side kind."""
import re


def render(js, arch="{}", extra=""):
    return js(f"ArchSimUI.renderArea(ArchSimUI.makeState({{arch: {arch}{extra}}}, DATA))")


def floats(h, attr):
    return [float(x) for x in re.findall(rf'{attr}="([-0-9.]+)"', h)]


def test_area_waterfall_mvp(js):
    h = render(js)
    tot = floats(h, "data-area-total")[0]
    assert abs(tot - 342472) <= 1
    assert abs(sum(floats(h, "data-area")) - tot) <= 1
    assert "data-area-limit" in h and "data-core-area" in h
    assert floats(h, "data-area-opt")[0] <= tot <= floats(h, "data-area-pess")[0]


def test_no_die_margin_restated(js):
    h = render(js)
    assert "data-chip-verdict" not in h and "margin" not in h.split("<h2>Pins")[0].replace("die_area check (margin and verdict)", "")
    assert 'data-side="{&quot;kind&quot;:&quot;chip&quot;,&quot;item&quot;:&quot;die_area&quot;' in h


def test_pin_strip_mvp(js):
    h = render(js)
    uses = re.findall(r'data-pin-use="(\w+)"', h)
    assert len(uses) == 24 and uses.count("host") == 6
    assert h.count('data-tristate="1"') == 8


def test_pins_follow_knobs(js):
    h = render(js, "{pins_reserved: 2}")
    assert re.findall(r'data-pin-use="(\w+)"', h).count("reserved") == 2


def test_demand_follows_focus_and_scenario(js, data):
    wl = next(w for w in data["workloads"] if w["id"] == "spi_target_10m")
    h = render(js)
    assert floats(h, "data-need-pins") == [wl["pins"]]
    assert re.findall(r'data-pin-use="(\w+)"', h).count("protocol") == wl["pins"]
    h2 = render(js, "{}", ", scenario: ['spi_target_10m', 'uart_115k']")
    ids = {w["id"]: w for w in data["workloads"]}
    if "uart_115k" in ids:
        assert floats(h2, "data-need-pins") == [wl["pins"] + ids["uart_115k"]["pins"]]


def test_follows_focused_pinned_point(js):
    h = js("ArchSimUI.renderArea(ArchSimUI.makeState({pinned: [{arch: {pins_reserved: 3}}], focus: {wl: 'spi_target_10m', col: 1}}, DATA))")
    assert re.findall(r'data-pin-use="(\w+)"', h).count("reserved") == 3
    assert '&quot;col&quot;:1' in h


def test_area_line_side(js):
    st = "ArchSimUI.makeState({}, DATA)"
    ids = js(f"ArchSim.blockDiagram({st}.arch, DATA).area_lines.map(function (l) {{ return l.id; }})")
    r = js(f"ArchSimUI.renderSide(ArchSimUI.makeState({{side: {{kind: 'area_line', id: {ids[0]!r}, col: 0}}}}, DATA))")
    assert ids[0] in r and ("pnr_factor" in r or "macro_halo" in r)


def test_isa_lines_when_on(js, data):
    knobs = js("ArchSim.KNOBS.filter(function (k) { return /^isa/.test(k.id) || k.group === 'ISA'; })")
    if not knobs:
        import pytest
        pytest.skip("no ISA knobs in the model")
    vals = {}
    for k in knobs:
        if k["type"] in ("bool", "boolean"):
            vals[k["id"]] = "true"
        elif k["type"] in ("enum", "en"):
            opts = [o["id"] if isinstance(o, dict) else o for o in k["options"]]
            non_default = [o for o in opts if o != k["default"]] or opts
            vals[k["id"]] = repr(non_default[-1])
        else:
            vals[k["id"]] = repr(k["max"])
    h = render(js, "{" + ", ".join(f"{k}: {v}" for k, v in vals.items()) + "}")
    assert 'data-area-id="isa_' in h

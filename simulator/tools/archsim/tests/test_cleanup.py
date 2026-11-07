"""Task 11 Part A: single sources of truth (Not modelled, ISS documents, vocabulary, sweep grid) and theme tokens."""
import pathlib
import re

import pytest

UI = pathlib.Path(__file__).resolve().parents[1] / "ui"
VIEW_FILES = sorted(UI.glob("view-*.js"))


def _all_ui_text():
    return "\n".join(f.read_text() for f in [UI / "archsim-ui.js"] + VIEW_FILES)


def test_not_modelled_from_data(js, data):
    items = data["not_modelled"]["items"]
    assert len(items) == 16 and all(i["text"] for i in items)
    trust = js("ArchSimUI.renderTrust(ArchSimUI.makeState({}, DATA))")
    for i in items:
        assert i["text"].replace("&", "&amp;") in trust
    for f in VIEW_FILES:
        assert "Open-drain rise times" not in f.read_text()


def test_not_modelled_in_cell_side(js):
    d = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'cell', wl: 'spi_target_10m', col: 0}}, DATA))")
    assert "Silicon pad delay on IHP" in d and "IR drop" in d


def test_iss_sources_from_data(data):
    src = " ".join(data["iss"]["sources"])
    assert "docs/mvp-isa.md" in src and "docs/mvp-regmap.md" in src and "iss/pio_iss.py" in src
    assert "docs/mvp-isa.md" not in _all_ui_text()


def test_vocab_matches_plan(data):
    v = data["vocab"]
    assert len(v["methods"]) == 11 and len(v["statuses"]) == 6 and "brackets" in {c["id"] for c in v["classes"]}
    for k in ("methods", "statuses", "classes"):
        assert all(x["meaning"] for x in v[k])


def test_sweep_default_grid_is_plan_5_3(js):
    h = js("ArchSimUI.renderExplore(ArchSimUI.makeState({view: {explore: {sweep: {knob: 'f_core_mhz'}}}}, DATA))")
    m = re.search(r'value="(40, 42[^"]*)"', h)
    assert m and m.group(1).split(", ") == [str(v) for v in range(40, 71, 2)]     # 48/50/60 are already on the grid


def test_no_hardcoded_colours_in_views():
    for f in list(UI.glob("*.js")):
        txt = re.sub(r"&#\d+;|&#x[0-9a-f]+;", "", f.read_text())
        assert not re.search(r"#[0-9a-fA-F]{3,6}\b|rgb\(|'white'|'black'", txt), f.name
    for cf in [UI / "archsim.css"] + sorted((UI / "css").glob("*.css")):
        css = cf.read_text()
        # token definitions (--x: #hex) are the only place a colour may be written
        rest = "\n".join(ln for ln in css.splitlines() if not ln.lstrip().startswith("--") and "  --" not in ln)
        assert not re.search(r"#[0-9a-fA-F]{3,6}\b|rgb\(|\bwhite\b|\bblack\b", re.sub(r"white-space", "", rest)), cf.name

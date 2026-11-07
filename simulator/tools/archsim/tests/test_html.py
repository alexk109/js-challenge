"""Task 7: the built page is offline, self-contained and carries inputs.json."""
import json
import re

from tools.archsim.build_html import build

from conftest import ROOT

HTML = ROOT / "archsim.html"


def test_exists_and_size():
    assert HTML.exists()
    assert HTML.stat().st_size < 2_500_000


def test_offline():
    h = HTML.read_text()
    for m in re.finditer(r'\b(?:src|href)\s*=\s*"([^"]*)"', h):
        v = m.group(1)
        assert not v.startswith(("http", "//", "data:")), v
    assert "<link" not in h
    assert "<title>archsim</title>" in h and 'name="viewport"' in h


def test_data_roundtrip(data):
    h = HTML.read_text()
    blocks = re.findall(r'<script id="archsim-data" type="application/json">(.*?)</script>', h, re.S)
    assert len(blocks) == 1 and h.count('id="archsim-data"') == 1
    assert json.loads(blocks[0]) == data


def test_fresh():
    assert HTML.read_text() == build(), "archsim.html is stale: run make archsim"


def test_knob_panel_one_control_per_knob(js):
    html = js("ArchSimUI.renderKnobPanel(ArchSimUI.makeState({}, DATA))")
    ids = js("ArchSim.KNOBS.map(function (k) { return k.id; })")
    assert len(ids) >= 20
    for k in ids:
        assert html.count(f'data-knob="{k}"') == 1, k
    assert 'data-preset' in html and 'Reset to MVP' in html

"""Concurrency in the Capability tab's Scenario mode: reference-mix picker and per-point table, pairwise matrix with tally,
max-concurrent line, the `pair` side panel; and the single-home rule for these markers."""
import re

SCN = "view: {capability: {sub: 'scenario'}}"


def _cap(js, raw=None):
    raw = "{%s}" % SCN if raw is None else raw
    return js(f"ArchSimUI.renderCapability(ArchSimUI.makeState({raw}, DATA))")


def _text(h):
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", " ", h))


def test_reference_mix_picker_chips(js, data):
    h = _cap(js)
    for m in data["mixes"]:
        assert 'data-cap-mix="%s" aria-pressed="false"' % m["id"] in h
    assert h.count('class="mx-chip" data-cap-mix=') == len(data["mixes"])
    d = data["mixes"][0]
    sel = "{scenario: %s, %s}" % (d["members"], SCN)
    h = _cap(js, sel)
    assert 'data-cap-mix="%s" aria-pressed="true"' % d["id"] in h


def test_reference_mix_table_per_point(js, data):
    raw = "{pinned: [{arch: {n_engines: 4}, label: 'four'}], %s}" % SCN
    h = _cap(js, raw)
    rows = re.findall(r'<tr data-mix-row="([^"]+)">(.*?)</tr>', h, re.S)
    assert [r[0] for r in rows] == [m["id"] for m in data["mixes"]]
    for _, body in rows:
        assert body.count("<td") == 2                      # live point + one pinned point
    by = {i: b for i, b in rows}
    live = re.findall(r'<td class="mx-cell v-(\w+)', by["maker_io"])
    assert live[0] == "impossible"                          # 4 members, 2 engines
    assert "Engines" in _text(by["maker_io"]) and "can run together" in by["maker_io"]
    assert "2 of 4 can run together" in _text(by["maker_io"]).replace("  ", " ")


def test_reference_mix_click_wiring(js):
    h = _cap(js)
    assert re.search(r'class="mx-cellbtn" data-cap-mix="ethernet" data-focus-col="0"', h)


def test_pair_matrix_and_tally_mvp(js, data):
    h = _cap(js)
    assert "4 of 10 pairs run together" in _text(h) or "4 of 10 pairs run together" in h
    cells = re.findall(r'<button type="button" class="mx-pairbtn"[^>]*data-verdict="(\w+)"', h)
    assert len(cells) == 10 and cells.count("served") == 4 and cells.count("marginal") == 1 and cells.count("impossible") == 5
    # served-alone protocols only (cap19 on the MVP: four) and a self-pair for each
    assert h.count('<th scope="row" class="mx-pairrow"') == 4
    assert 'data-pair="ws2812|ws2812"' in h and 'data-pair="canfd_5m|canfd_5m"' not in h
    # hover text names the failing rows; click opens the pair side panel
    m = re.search(r'<button[^>]*data-pair="uart_3m\|uart_3m"[^>]*>', h)
    assert m and "Engines" in m.group(0) and "&quot;kind&quot;:&quot;pair&quot;" in m.group(0)
    assert "Left out (not served alone" in h


def test_pair_matrix_follows_focused_point_and_tag(js):
    h = _cap(js, "{preset: 'wide_host', arch: ArchSim.applyPreset('wide_host', DATA), %s}" % SCN)
    assert "35 of 45 pairs run together" in h
    h = _cap(js, "{tag: 'all', %s}" % SCN)
    assert re.search(r"\d+ of 231 pairs run together", h)


def test_pair_matrix_also_with_a_scenario(js):
    h = _cap(js, "{scenario: ['i2c_fmplus_ctrl', 'ws2812'], %s}" % SCN)
    assert "pairs run together" in h and "data-max-concurrent" in h
    assert "At most 2 of the 2 ticked protocols can run together" in _text(h)
    h = _cap(js, "{scenario: ['uart_3m', 'ws2812', 'i2c_fmplus_ctrl'], %s}" % SCN)
    assert "At most 2 of the 3 ticked protocols can run together" in _text(h)
    h = _cap(js, "{scenario: ['uart_3m'], %s}" % SCN)
    assert "At most 0 of the 1 ticked protocols can run together on this point (1 if marginal results count)" in _text(h)


def test_pair_side_panel(js):
    d = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'pair', a: 'uart_3m', b: 'ws2812', col: 0}}, DATA))")
    t = _text(d)
    assert "uart_3m" in t and "ws2812" in t and "Engines" in t and "Not checked here" in t and "arbitration" in t
    d = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'pair', a: 'ws2812', b: 'ws2812', col: 0}}, DATA))")
    assert "two instances" in _text(d) and "Every shared resource fits" in _text(d)


def test_not_modelled_concurrency_items_in_trust(js, data):
    t = _text(js("ArchSimUI.renderTrust(ArchSimUI.makeState({}, DATA))"))
    for s in ("Host MCU CPU time per stream", "Host-port arbitration latency between engines",
              "Engines needing different core clocks", "Timing interference between engines on shared pins"):
        assert s in t

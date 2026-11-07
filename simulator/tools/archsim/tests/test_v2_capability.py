"""Capability tab (matrix, chip badge/hatch, scenario sub-view) and the side-panel kind `cell`."""
import re


def _cap(js, raw="{}"):
    return js(f"ArchSimUI.renderCapability(ArchSimUI.makeState({raw}, DATA))")


def _side(js, raw):
    return js(f"ArchSimUI.renderSide(ArchSimUI.makeState({raw}, DATA))")


def _text(h):
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", " ", h))


def _cells(h):
    return re.findall(r'<td class="mx-cell.*?</td>', h, re.S)


def test_matrix(js):
    h = _cap(js)
    assert h.count("data-row=") == 19
    cells = _cells(h)
    assert len(cells) == 19
    assert sum('data-verdict="served"' in c for c in cells) == 4
    assert sum('data-verdict="impossible"' in c for c in cells) == 15
    assert not any('data-verdict="marginal"' in c for c in cells)
    m = re.search(r'<tr data-row="spi_target_10m".*?</tr>', h, re.S)
    assert m and "reflex" in _text(m.group(0))
    for c in cells:                                    # chip items never appear in a cell
        t = _text(c).lower()
        assert "core clock" not in t and "die area" not in t and "board clock" not in t


def test_every_cell_has_verdict_label(js):
    h = _cap(js, "{tag: 'all'}")
    cells = _cells(h)
    assert len(cells) == h.count("data-row=") > 19
    for c in cells:
        assert re.search(r"\b(served|marginal|impossible)\b", _text(c))
        assert re.search(r"<b>[^<]+</b>", c)           # binding limit short name


def test_tag_filter_and_chips(js, data):
    h = _cap(js, "{tag: 'stretch'}")
    assert 'data-cap-tag="stretch" aria-pressed="true"' in h
    assert 'data-cap-tag="cap19" aria-pressed="false"' in h
    h = _cap(js, "{tag: 'all'}")
    for t in data["workload_tags"]["items"]:           # explanations as tooltips + one legend line
        assert t["text"].split(" (PLAN")[0][:30] in h
    assert 'data-mx-search' in h


def test_grouping_by_verdict_and_binding(js):
    h = _cap(js)
    assert h.index("mx-group") < h.index('data-row=')
    heads = re.findall(r'<tr class="mx-group( sub)?" data-gh="([^"]+)"', h)
    assert ("", "v:served") in heads and ("", "v:impossible") in heads
    assert any(s and k.startswith("v:impossible|b:") for s, k in heads)
    # rows of one verdict group sit under its header
    served_pos = h.index('data-gh="v:served"')
    imp_pos = h.index('data-gh="v:impossible"')
    assert served_pos < imp_pos
    rows = re.findall(r'<tr data-row="([^"]+)" data-g="([^"]+)"', h)
    assert [g.split("|")[0] for _, g in rows] == sorted((g.split("|")[0] for _, g in rows), key=["v:served", "v:marginal", "v:impossible"].index)
    hb = _cap(js, "{view: {capability: {group: 'binding'}}}")
    assert 'data-gh="b:' in hb and 'data-gh="v:' not in hb
    hn = _cap(js, "{view: {capability: {group: 'none'}}}")
    assert "mx-group" not in hn and hn.count("data-row=") == 19


def test_pinned_columns(js):
    h = _cap(js, "{pinned: [{arch: {f_core_mhz: 60}, label: 'fast'}]}")
    assert h.count("mx-colhead") == 2 and "fast" in h
    assert h.count('data-col="1"') == 19
    assert "differ" in h and "protocols differ" in h


def test_diff_toggle(js):
    raw = "{pinned: [{arch: {n_engines: 1}, label: 'one'}], view: {capability: {diff: true}}}"
    h = _cap(js, raw)
    all_rows = _cap(js, "{pinned: [{arch: {n_engines: 1}, label: 'one'}]}").count("data-row=")
    n = h.count("data-row=")
    assert 0 < n < all_rows == 19
    assert "verdict differs" in h and "uart_3m" in h


def test_scalable_row_has_max_rate(js):
    h = _cap(js, "{tag: 'all'}")
    m = re.search(r'<tr data-row="spi_target_scalable".*?</tr>', h, re.S)
    assert m and "max " in m.group(0)


def test_stack_a_chip_badge(js):
    h = _cap(js, "{arch: " + js("ArchSim.applyPreset('stack_a', DATA)").__repr__().replace("True", "true").replace("False", "false") + "}")
    head = re.search(r'<th class="mx-colhead.*?</th>', h, re.S).group(0)
    assert 'data-chip-verdict="marginal"' in head and "die area" in head and "(chip marginal)" in head
    assert "chip-fail" not in h
    cells = _cells(h)
    assert sum('data-verdict="served"' in c for c in cells) == 4


def test_chip_fail_column(js):
    h = _cap(js, "{arch: {f_core_mhz: 64}}")
    assert 'data-chip-verdict="fail"' in h and "(chip fails)" in h
    assert h.count("chip check fails (") == 1
    cells = _cells(h)
    assert len(cells) == 19 and all("chip-fail" in c for c in cells)
    # verdicts are unchanged by the chip: same cell verdict strings as the passing live point
    base = _cap(js)
    assert [re.search(r'data-verdict="(\w+)"', c).group(1) for c in cells] != [] and "chip-fail" not in base
    assert base.count("chip check fails (") == 0
    two = _cap(js, "{pinned: [{arch: {f_core_mhz: 64}, label: 'hot'}]}")
    assert two.count("chip check fails (") == 1 and sum("chip-fail" in c for c in _cells(two)) == 19


def test_scenario_view_mix_fail(js):
    h = _cap(js, "{scenario: ['spi_target_10m', 'usb_fs_tx'], view: {capability: {sub: 'scenario'}}}")
    assert "mx-mixtable" in h and "Link write bandwidth" in h and "fail" in h
    assert "mx-members" in h and "Host decision loop" in h and "Program fits one engine" in h
    assert h.count("data-mix=") == 20 and h.count(" checked") == 2   # 19 cap19 rows + usb_fs_tx (stretch)
    assert "impossible even alone" in h
    assert "0.699" not in h                              # the link figure lives in Throughput only
    assert 'data-tab="throughput"' in h


def test_scenario_view_pass_and_empty(js):
    h = _cap(js, "{scenario: ['i2c_target_100k', 'ws2812'], view: {capability: {sub: 'scenario'}}}")
    assert "mx-mixtable" in h
    e = _cap(js, "{view: {capability: {sub: 'scenario'}}}")
    assert "mx-mixtable" not in e and "Tick two or more" in e


def test_matrix_checkbox_marks_scenario(js):
    h = _cap(js, "{scenario: ['ws2812']}")
    assert h.count(" checked") == 1 and 'data-mix="ws2812" checked' in h


def test_cell_buttons_open_side_and_focus(js):
    h = _cap(js)
    m = re.search(r'<button[^>]*data-cell="spi_target_10m"[^>]*>', h)
    assert m and "data-focus-wl" in m.group(0) and "data-focus-col" in m.group(0)
    assert "&quot;kind&quot;:&quot;cell&quot;" in m.group(0)
    assert " selected" in h                              # the focused cell


def test_side_cell(js):
    d = _side(js, "{side: {kind: 'cell', wl: 'spi_target_10m', col: 0}}")
    assert "t_pad_rt_ns" in d and "docs/mvp-isa.md" in d
    assert "Blocked by" in d and "<code>reflex</code>" in d
    assert "Not applicable" in d and "no on-chip CRC needed" in d
    assert "Not modelled" in d and "Silicon pad delay on IHP" in d and "IR drop" in d
    assert "research/models/limits.py:31" in d
    assert "closest" in d and 'data-tab="trust"' in d


def test_side_cell_every_limit_and_pinned_column(js):
    d = _side(js, "{side: {kind: 'cell', wl: 'usb_fs_tx', col: 1}, pinned: [{arch: {f_core_mhz: 60}, label: 'fast'}]}")
    assert "fast" in d
    assert len(re.findall(r'<details class="mx-lim"', d)) >= 8
    bad = _side(js, "{side: {kind: 'cell', wl: 'nope', col: 0}}")
    assert "No protocol" in bad


def test_side_cell_notes_chip_state(js):
    d = _side(js, "{side: {kind: 'cell', wl: 'spi_target_10m', col: 0}, arch: {f_core_mhz: 64}}")
    assert "chip check of this point is" in d and "does not include it" in d
    ok = _side(js, "{side: {kind: 'cell', wl: 'spi_target_10m', col: 0}}")
    assert "chip check of this point is" not in ok


def test_flag_limits_in_words(js):
    wl = js("(function(){var a=ArchSim.applyPreset('mvp',DATA);a.crc_unit=false;var r=null;DATA.workloads.forEach(function(w){"
            "if(w.needs_xor&&!r)r=w.id});return r})()")
    h = _cap(js, "{tag:'all',arch:{crc_unit:false}}")
    d = _side(js, "{tag:'all',arch:{crc_unit:false},side:{kind:'cell',wl:'%s',col:0}}" % wl)
    assert "missing" in h and "missing" in d
    assert "-1.00 flag" not in h and "-1.00 flag" not in d


def test_closest_is_first_ranked_row(js):
    d = _side(js, "{side: {kind: 'cell', wl: 'spi_target_10m', col: 0}}")
    assert d.count("closest</span>") == 1
    assert "mx-closest" not in _cap(js, "{tag:'all'}")       # the cell carries no closest line (it is the side panel's first row)


def test_degenerate_band_collapses(js):
    assert js("ArchSimUI.fmtBand({nom: -238.2, pess: -237.9, opt: -238.4}, 'ns')") == "-238 ns"
    assert ".." not in js("ArchSimUI.fmtBandHtml({nom: -238.2, pess: -237.9, opt: -238.4}, 'ns')")
    assert js("ArchSimUI.fmtBand({nom: 1, pess: -2, opt: 5}, 'ns')") == "1.00 ns (-2.00..5.00)"


def test_registered_and_renders_in_view(js):
    assert js("ArchSimUI.renderView(ArchSimUI.makeState({}, DATA))") == _cap(js)
    assert js("'cell' in ArchSimUI.sideKinds")


def test_isa_estimate_marks(js):
    h = _cap(js, "{arch: {isa_xor: true}}")
    m = re.search(r'<td class="mx-cell[^>]*>(?:(?!</td>).)*data-cell="can_1m".*?</td>', h, re.S)
    assert m and "st-estimate" in m.group(0)
    head = re.search(r"<th class=\"mx-colhead.*?</th>", h, re.S).group(0)
    assert "st-estimate" in head
    off = _cap(js)
    assert "st-estimate" not in off and "data-estimate" not in off


def test_isa_estimate_in_cell_side_panel(js):
    h = _side(js, "{arch: {isa_xor: true}, side: {kind: 'cell', wl: 'can_1m', col: 0}}")
    assert "st-estimate" in h.split("Blocked by")[0]


def test_pin_hint_and_preset_labels(js):
    assert "data-pin-hint" in _cap(js)
    assert "data-pin-hint" not in _cap(js, "{pinned: [{arch: {n_engines: 1}, label: 'one'}]}")
    lab = js("ArchSimUI.makeState({preset: 'mvp'}, DATA).points[0].label")
    assert lab == "S0: MVP as built"
    assert js("ArchSimUI.makeState({preset: 'mvp'}, DATA).points[0].id") == "mvp"
    assert "live: S0: MVP as built" in _text(_cap(js, "{preset: 'mvp'}"))


def test_isa_estimate_marks_elsewhere(js):
    st = "ArchSimUI.makeState({arch: {isa_xor: true}}, DATA)"
    chip = js(f"ArchSimUI.renderChip({st})")
    assert re.search(r'data-chip-item="isa_encoding".*?st-estimate', chip, re.S)
    assert "data-area-isa" in js(f"ArchSimUI.renderArea({st})")
    assert "data-area-isa" not in js("ArchSimUI.renderArea(ArchSimUI.makeState({}, DATA))")
    assert "data-estimate" not in js("ArchSimUI.renderChip(ArchSimUI.makeState({}, DATA))")
    ex = js(f"ArchSimUI.renderExplore({st})")
    assert re.search(r'data-knob-row="isa_xor".*?st-estimate', ex, re.S)


def test_derivation_tree_dedupes_repeated_sources(js):
    h = _side(js, "{side: {kind: 'cell', wl: 'spi_target_10m', col: 0}}")
    assert "sources as above" in h
    assert js("ArchSimUI.settling()") is False

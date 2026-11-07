"""The Timing tab (render-level checks via the QuickJS harness)."""
import re


def render(js, arch="{}", wl="spi_target_10m", pinned="[]", col=0):
    return js(f"ArchSimUI.renderTiming(ArchSimUI.makeState({{arch: {arch}, pinned: {pinned}, focus: {{wl: '{wl}', col: {col}}}}}, DATA))")


def floats(h, attr):
    return [float(x) for x in re.findall(rf'{attr}="([-0-9.]+)"', h)]


def test_latency_budget_mvp(js):
    h = render(js)
    segs = floats(h, "data-seg-ns")
    assert len(segs) == 7 and abs(sum(segs) - 137) <= 0.01
    assert floats(h, "data-deadline") == [50.0]
    assert 137.0 in floats(h, "data-total-nom")
    assert floats(h, "data-total-pess") == [160.0] and floats(h, "data-total-opt") == [112.0]
    assert "data-tick" in h and 'class="wave"' in h


def test_exactly_one_chart_carries_segments(js):
    h = render(js)
    assert len(floats(h, "data-seg-ns")) == 7       # the table does not repeat them as data-seg-ns
    assert h.count("data-deadline=") == 1
    assert h.count('data-total-nom=') == 1


def test_latency_lane_second_row(js):
    h = render(js, "{lane: true}")
    assert len(floats(h, "data-seg-ns")) == 7 and len(floats(h, "data-lane-seg-ns")) == 3
    assert h.count("data-total-nom=") == 2 and 'data-active="lane"' in h


def test_segments_open_side_panel(js):
    h = render(js)
    assert h.count('"kind":"segment"') >= 7 or h.count("&quot;kind&quot;:&quot;segment&quot;") >= 7
    side = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'segment', path: 'lat', i: 0, col: 0}}, DATA))")
    assert "pad in" in side and "t_pad_rt_ns" in side
    dl = js("ArchSimUI.renderSide(ArchSimUI.makeState({side: {kind: 'segment', path: 'deadline', col: 0}}, DATA))")
    assert "deadline" in dl.lower()


def test_edge_grid_usb_48(js):
    h = render(js, "{f_core_mhz: 48}", "usb_fs_tx")
    ideal, placed = floats(h, "data-edge-ideal"), floats(h, "data-edge-placed")
    assert len(ideal) == 8 and len(placed) == 8
    assert all(abs(a - b) < 1e-2 for a, b in zip(ideal, placed))
    h50 = render(js, "{f_core_mhz: 50}", "usb_fs_tx")
    assert any(abs(a - b) > 1 for a, b in zip(floats(h50, "data-edge-ideal"), floats(h50, "data-edge-placed")))


def test_rx_sampling_eth10_rx(js):
    h = render(js, "{}", "eth10_rx")
    assert floats(h, "data-rx-margin") == [-8.52]
    assert len(floats(h, "data-rx-ns")) == 3 and "data-rx-osr" in h
    h2 = render(js, "{capture: 'dpll'}", "eth10_rx")
    assert floats(h2, "data-rx-margin") == [1.5]


def test_clock_budget_mvp(js):
    h = render(js)
    D = floats(h, "data-clk-D")[0]
    assert abs(D - 1000 / 61.6) < 1e-2 and abs(D - 16.23) < 5e-3
    parts = floats(h, "data-clk-ns")
    assert abs(sum(parts) - D) < 1e-2
    assert floats(h, "data-clk-T") == [20.0]
    assert "core_clock" in h and "chip" in h          # links to the chip card instead of repeating the margin
    assert "11.6" not in h                            # the chip card's margin number is not repeated here


def test_not_applicable_sections_say_why(js):
    h = render(js, "{}", "uart_3m")
    assert h.count("tm-na") >= 2 and "no reflex or response deadline" in h
    assert "data-seg-ns" not in h


def test_figure_for_firmware_row(js):
    h = render(js, "{}", "fw_spi_m0")
    assert 'data-figure="spi"' in h and "golden ISS" in h


def test_follows_pinned_point(js):
    h = render(js, "{}", "usb_fs_tx", "[{arch: {f_core_mhz: 48}, label: 'p48'}]", 1)
    ideal, placed = floats(h, "data-edge-ideal"), floats(h, "data-edge-placed")
    assert all(abs(a - b) < 1e-2 for a, b in zip(ideal, placed)) and "p48" in h


def test_every_workload_renders(js, data):
    for w in data["workloads"]:
        assert isinstance(render(js, "{}", w["id"]), str)
        assert "Error" not in render(js, "{lane: true}", w["id"])[:50]


def test_strip_compares_points(js):
    h = js("ArchSimUI.renderTiming(ArchSimUI.makeState({focus: {wl: 'spi_target_10m'}, pinned: [{arch: {lane: true}, label: 'lane'}]}, DATA))")
    import re
    strip = re.search(r"<section[^>]*data-timing-strip.*?</section>", h, re.S).group(0)
    assert strip.count("data-strip-point=") == 2 and "data-focus-col=\"1\"" in strip
    assert "reflex" in strip and "data-verdict=" in strip

"""Throughput tab (link card, host demand, FIFO fill trace)."""
import re


def render(js, arch="{}", wl="spi_target_10m"):
    return js(f"ArchSimUI.renderThroughput(ArchSimUI.makeState({{arch: {arch}, focus: {{wl: '{wl}', col: 0}}}}, DATA))")


def test_link_card_mvp(js):
    h = render(js)
    # the link write figure appears once as a link-card stat; read, half duplex too
    assert h.count("0.6993") == 1 and "0.6431" in h and "0.7246" in h
    assert "0.5714" in h and "0.5333" in h and "0.5882" in h
    assert h.count("data-link-card") == 1
    assert '"id":"link_write_MBps"' in h.replace("&quot;", '"')


def test_stream_underrun_and_deep_fifo(js):
    h = render(js, wl="eth10_tx")
    assert 'data-stream-ok="0"' in h and 'class="envelope"' in h and "underrun" in h
    h2 = render(js, "{deep_fifo_words: 64}", "eth10_tx")
    assert 'data-stream-ok="1"' in h2


def test_streaming_out_exec_cost(js):
    on = render(js, "{streaming: 'out_exec'}", "spi_flash_read_decode")
    assert "OUT EXEC streaming is on" in on and "8.000 MB/s" in on
    off = render(js, "{}", "spi_flash_read_decode")
    assert "would add" in off and "streaming not active" in off
    hd = js("ArchSim.hostDemand({streaming: 'out_exec'}, 'spi_flash_read_decode', DATA)")
    assert hd["stream_MBps"] > 0 and abs(hd["bw_eff"] - hd["payload_MBps"] - hd["stream_MBps"]) < 1e-9


def test_every_workload_renders(js, data):
    for w in data["workloads"]:
        assert isinstance(render(js, "{}", w["id"]), str)


def test_no_bandwidth_need(js, data):
    w = next((w["id"] for w in data["workloads"] if not w.get("bw_MBps")), None)
    if w:
        assert "no sustained" in render(js, "{}", w)

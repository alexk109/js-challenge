"""gl_rig: the perf rigs on the post-layout netlist.  Fast tests cover the wrapper generator and supports(); the
gate-level run itself is @pytest.mark.long (it needs a finished flow and Icarus)."""
import json
import os
import subprocess
import sys

import pytest

from fw import gl_wrap
from tools.bench import knobs, runner, workloads as wl
from tools.bench.backends import gl_rig as GR, load
from tools.bench.backends import power as P
from tools.bench.types import RunSpec

ROOT = GR.sim.ROOT


def _netlist(aw=7, named_levels=True, engines=2, fdebug=8) -> str:
    wires = ["bus_re", "bus_we"] + [f"\\bus_addr[{i}] " for i in range(aw)] + [f"\\bus_wdata[{i}] " for i in range(32)]
    wires += [f"\\u_host.miso_sr_q[{i}] " for i in range(31)] + [f"\\u_chip.fdebug[{i}] " for i in range(fdebug)]
    for e in range(engines):
        if named_levels:
            wires += [f"\\u_chip.g_eng[{e}].u_eng.{d}x_level[{i}] " for d in "tr" for i in range(3)]
    if not named_levels:
        wires += [f"\\u_chip.flevel_w[{8 * e + n + i}] " for e in range(engines) for n in (0, 4) for i in range(3)]
    return "\n".join(f" wire {w};" for w in wires)


def test_wrap_forces_the_bus_and_reads_the_capture_register():
    v = gl_wrap.build(_netlist())
    assert "input  wire [6:0] bus_addr" in v and "AW=7" in v
    assert "force dut.bus_re = bus_re;" in v and "force dut.\\bus_addr[6]  = bus_addr[6];" in v
    assert "always @(bus_we or bus_re or bus_addr or bus_wdata) apply_bus;" in v      # icarus force is not continuous
    assert "assign bus_rdata[30] = dut.\\u_host.miso_sr_q[30] ;" in v and "bus_rdata[31] = uo_out[0]" in v
    assert "assign gl_levels[0] = dut.\\u_chip.g_eng[0].u_eng.tx_level[0] ;" in v
    assert "assign gl_levels[8] = dut.\\u_chip.g_eng[1].u_eng.tx_level[0] ;" in v
    assert "assign gl_levels[16] = 1'b0;" in v


def test_wrap_wide_map_and_flevel_fallback():
    v = gl_wrap.build(_netlist(aw=10, named_levels=False, engines=3, fdebug=12))
    assert "[9:0] bus_addr" in v and "12 FDEBUG bits" in v
    assert "assign gl_levels[20] = dut.\\u_chip.flevel_w[20] ;" in v and "assign gl_levels[24] = 1'b0;" in v


def test_wrap_names_the_missing_net():
    text = _netlist().replace(" wire bus_re;", "")
    with pytest.raises(SystemExit, match="no net 'bus_re'"):
        gl_wrap.build(text)
    with pytest.raises(SystemExit, match="FIFO level"):
        gl_wrap.build(_netlist(engines=0))


def test_supports_levels_and_hashes(monkeypatch):
    be = load("gl_rig")
    assert be.name == "gl_rig" and be.level == "gl"
    spi = wl.workload("spi")
    cfg = knobs.normalize({})
    monkeypatch.setattr(GR, "find_source", lambda c: None)
    assert be.supports(cfg, spi) == P.FLOW_HINT
    monkeypatch.setattr(GR, "find_source", lambda c: object())
    assert be.supports(cfg, spi) is None
    assert "needs 2 engines" in be.supports(knobs.normalize({"n_engines": 1}), spi)
    assert "host SPI port" in be.supports(cfg, wl.workload("spi_host_port"))
    assert be.supports(knobs.normalize({"n_engines": 3, "fifo_depth": 8, "engine_route": True}), spi) is None
    spec = RunSpec(cfg, spi, "gl_rig")
    assert be.is_long(spec)
    monkeypatch.setattr(P, "find_source", lambda c: None)
    assert runner.code_hash_for(be, spec).endswith(":noflow")          # the flow artifacts are an input
    ids = {w.id for w in be.workloads()}
    assert {"spi", "swd"} <= ids and "spi_host_port" not in ids


@pytest.mark.long
def test_gl_rig_spi_mvp_matches_rtl_cycle_exactly():
    """MVP spi at its default speed on the netlist == the RTL run, metric for metric (needs a finished MVP flow)."""
    be = load("gl_rig")
    spi = wl.workload("spi")
    if be.supports(knobs.normalize({}), spi):
        pytest.skip("no finished MVP flow on this machine")
    code = ("import json,sys\nfrom tools.bench import knobs, workloads as wl, backends\nfrom tools.bench.types import RunSpec\n"
            "b = backends.load(sys.argv[1])\nr = b.run(RunSpec(knobs.normalize({}), wl.workload('spi'), b.name))\n"
            "print(json.dumps({x.metric: [x.value, x.status] for x in r}))")
    out = {}
    for name in ("gl_rig", "rtl"):
        p = subprocess.run([sys.executable, "-c", code, name], cwd=ROOT, capture_output=True, text=True, timeout=1500,
                           env={**os.environ, "PYTHONPATH": str(ROOT)})
        assert p.returncode == 0, p.stderr[-1500:]
        out[name] = json.loads(p.stdout.strip().splitlines()[-1])
    assert out["gl_rig"]["sweep_point"][0] == "pass"
    for metric, (value, status) in out["rtl"].items():
        if isinstance(value, float):
            assert out["gl_rig"][metric] == [value, status], metric


@pytest.mark.long
def test_gl_rig_catches_a_mutated_netlist(tmp_path):
    """Negative control: swap ~90 inv_1 for buf_1 in a copy of the MVP netlist; gl_rig spi must no longer pass.
    The copy gets its own build dir (the tag includes the netlist's content hash)."""
    import re
    be = load("gl_rig")
    spi = wl.workload("spi")
    cfg = knobs.normalize({})
    if be.supports(cfg, spi):
        pytest.skip("no finished MVP flow on this machine")
    text = GR.netlist_for(cfg).read_text()
    hits = list(re.finditer(r"sg13cmos5l_inv_1 (\S+) \(\.Y\(", text))
    pick = hits[:: max(1, len(hits) // 90)]
    for m in reversed(pick):
        text = text[:m.start()] + f"sg13cmos5l_buf_1 {m.group(1)} (.X(" + text[m.end():]
    bad = tmp_path / "mutated.nl.v"
    bad.write_text(text)
    code = ("import json,sys\nfrom tools.bench import knobs, workloads as wl, backends\nfrom tools.bench.types import RunSpec\n"
            "b = backends.load('gl_rig')\nr = b.run(RunSpec(knobs.normalize({}), wl.workload('spi'), 'gl_rig'))\n"
            "print(json.dumps({x.metric: x.value for x in r}))")
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, timeout=1500,
                       env={**os.environ, "PYTHONPATH": str(ROOT), "GL_RIG_NETLIST": str(bad)})
    passed = p.returncode == 0 and json.loads(p.stdout.strip().splitlines()[-1]).get("sweep_point") == "pass"
    assert not passed, "a netlist with 90 inverters turned into buffers still passes gl_rig spi"


def test_gl_and_rtl_do_not_share_a_process(monkeypatch):
    """The fw target install is process-global: rtl in a process set up for gate level must refuse."""
    monkeypatch.setenv("FW_RTL_GL", "x.nl.v")
    with pytest.raises(RuntimeError, match="own process"):
        load("rtl")._install(False, knobs.normalize({}))
    monkeypatch.delenv("FW_RTL_GL")
    from fw import rtl_harness
    monkeypatch.setattr(rtl_harness, "GL", False)                # the RTL harness is already loaded in this process
    monkeypatch.setattr(GR, "find_source", lambda c: object())
    monkeypatch.setattr(GR, "netlist_for", lambda c: ROOT / "Makefile")
    with pytest.raises(RuntimeError, match="own process"):
        load("gl_rig")._install(False, knobs.normalize({}))

"""Bridge and mix workloads on the `iss_p` backend (tools/bench/bridges.py).

Every run goes through a worker subprocess (the real path): the fw target install is process-global and a
non-MVP `iss_p` chip installed in the pytest process would break the MVP-identity tests of test_iss_p.py.
Sizes are short: the whole file runs in well under a minute.
"""
from __future__ import annotations

import json
import subprocess
import sys
from functools import lru_cache
from pathlib import Path

import pytest

from tools.bench import bridges, knobs
from tools.bench.backends import load
from tools.bench.types import Workload

ROOT = Path(__file__).resolve().parents[3]
ROUTE = {"engine_route": True, "n_engines": 2}


def run(wid: str, params: dict | None = None, cfg: dict | None = None, timeout: int = 200) -> dict:
    """Run one workload in a worker subprocess; returns {metric: record dict}."""
    return _run(wid, json.dumps(params or {}, sort_keys=True), json.dumps({**ROUTE, **(cfg or {})}, sort_keys=True),
                timeout)


@lru_cache(maxsize=None)
def _run(wid: str, params: str, cfg: str, timeout: int) -> dict:
    spec = {"config": knobs.normalize(json.loads(cfg)),
            "workload": {"id": wid, "kind": wid.split(".")[0], "params": json.loads(params), "label": ""},
            "backend": "iss_p"}
    p = subprocess.run([sys.executable, "-m", "tools.bench.worker"], input=json.dumps(spec), cwd=ROOT,
                       env={**__import__("os").environ, "PYTHONPATH": str(ROOT)}, capture_output=True, text=True,
                       timeout=timeout)
    assert p.returncode == 0, p.stderr[-2500:]
    recs = [json.loads(line) for line in p.stdout.splitlines() if line.strip()]
    assert all(r["status"] != "error" for r in recs), recs
    return {r["metric"]: r for r in recs}


def val(recs: dict, metric: str):
    return recs[metric]["value"]


# ---- every bridge passes at an easy setting --------------------------------------------------------

@pytest.mark.parametrize("wid, params", [
    ("bridge.uart_rx->uart_tx", {"in_rate": 115200, "out_rate": 115200, "burst": 6}),
    ("bridge.uart_rx->uart_tx", {"in_rate": 38400, "out_rate": 115200, "burst": 6}),     # the reverse conversion
    ("bridge.spi_target->uart_tx", {"in_rate": 1_000_000, "out_rate": 115200, "burst": 6}),
    ("bridge.uart_rx->i2c_target", {"in_rate": 115200, "burst": 6}),
])
def test_bridge_passes_at_an_easy_setting(wid, params):
    r = run(wid, params)
    assert val(r, "sweep_point") == "pass" and r["sweep_point"]["status"] == "ok"
    assert (val(r, "bytes_in"), val(r, "bytes_out"), val(r, "bytes_lost"), val(r, "bytes_spurious")) == (6, 6, 0, 0)
    assert 0 < val(r, "peak_buffer_words") <= 2 * 4
    assert 0 < val(r, "latency_first_byte_cycles") <= val(r, "latency_p99_cycles")
    assert val(r, "host_port_utilization") == 0.0          # the host is out of the loop after configuration
    assert r["sustained_rate_no_loss"]["status"] == "n/a"  # not searched: never a fake 0
    assert r["bytes_in"]["workload_params"]["burst"] == 6 and r["bytes_in"]["level"] == "iss"


def test_sustained_rate_is_found_by_stepping_the_gap():
    r = run("bridge.uart_rx->uart_tx", {"in_rate": 115200, "out_rate": 115200, "burst": 4, "search": 1})
    assert val(r, "sustained_rate_no_loss") == pytest.approx(115200 / 10)       # back to back: source-limited
    assert "burst fits" in r["sustained_rate_no_loss"]["note"]


# ---- negative control and the FIFO-depth sweep -----------------------------------------------------
# 20 bytes at 115200 in, 57600 out: the source is twice as fast as the sink, so the backlog (about 10 words)
# is larger than a depth-4 route holds (REGMAP.md section 3: a full TX FIFO backpressures the RX FIFO, whose
# PUSH then stalls the receiver, which is the only place a byte can be lost).
SLOW_SINK = {"in_rate": 115200, "out_rate": 57600, "burst": 20}


def sweep():
    return {d: run("bridge.uart_rx->uart_tx", SLOW_SINK, {"fifo_depth": d}) for d in (4, 8, 16)}


def test_slow_sink_at_depth_4_loses_bytes_and_saturates_the_route():
    r = sweep()[4]
    assert val(r, "bytes_lost") > 0 and r["sweep_point"]["status"] == "fail"
    assert val(r, "peak_buffer_words") == 2 * 4            # both FIFOs of the route were full
    assert val(r, "bytes_in") == 20 and val(r, "bytes_out") < 20


def test_fifo_depth_sweep_changes_losses_and_peak():
    r = sweep()
    lost = {d: val(r[d], "bytes_lost") for d in r}
    peak = {d: val(r[d], "peak_buffer_words") for d in r}
    assert lost[4] > 0 and lost[8] == 0 and lost[16] == 0
    assert peak[4] <= peak[8] <= peak[16] and peak[4] < peak[8]
    assert all(peak[d] <= 2 * d for d in r)
    assert val(r[8], "sweep_point") == val(r[16], "sweep_point") == "pass"


def test_lcs_accounting_can_fail():
    """The loss/spurious accounting itself: a dropped byte, a duplicated byte and a garbage byte are all seen."""
    data = [1, 2, 3, 4, 5]
    ok = bridges._stats(data, [1, 2, 3, 4, 5], [10] * 5, [20] * 5)
    assert (ok["bytes_lost"], ok["bytes_spurious"]) == (0, 0)
    dropped = bridges._stats(data, [1, 2, 4, 5], [10] * 5, [20] * 4)
    assert (dropped["bytes_lost"], dropped["bytes_spurious"]) == (1, 0)
    garbage = bridges._stats(data, [1, 2, 99, 4, 5], [10] * 5, [20] * 5)
    assert (garbage["bytes_lost"], garbage["bytes_spurious"]) == (1, 1)
    reordered = bridges._stats(data, [1, 3, 2, 4, 5], [10] * 5, [20] * 5)
    assert reordered["bytes_lost"] == 1 and reordered["bytes_spurious"] == 1


# ---- mixes ------------------------------------------------------------------------------------------

def test_mix_uart_full_duplex():
    r = run("mix.uart_tx+uart_rx", {"n": 6}, {"n_engines": 2})
    assert val(r, "all_pass") is True and val(r, "pass_uart_tx") is True and val(r, "pass_uart_rx") is True
    assert 0 < val(r, "host_port_utilization") < 1.0 and r["host_port_utilization"]["method"] == "derived"


def test_mix_three_protocols_on_three_engines():
    r = run("mix.uart_tx+spi_rx+i2c_rd", {"n": 6}, {"n_engines": 3})
    assert val(r, "all_pass") is True
    assert all(val(r, k) is True for k in ("pass_uart_tx", "pass_spi_rx", "pass_i2c_rd"))
    assert r["sweep_point"]["status"] == "ok"


@pytest.mark.parametrize("wid, cfg", [("mix.uart_tx+spi_rx+i2c_rd", {"n_engines": 3}),
                                      ("mix.uart_tx+uart_rx", {"n_engines": 2})])
def test_mix_with_an_infeasible_poll_rate_fails(wid, cfg):
    """poll=64 asks for more host-port traffic than the SPI port carries (a FLEVEL read alone is 644 cycles)."""
    r = run(wid, {"n": 6, "poll": 64}, cfg)
    assert val(r, "all_pass") is False and r["sweep_point"]["status"] == "fail"
    assert "host port infeasible" in r["host_port_utilization"]["note"]
    assert "host port infeasible" in r["sweep_point"]["note"]


@pytest.mark.parametrize("depth, want", [(8, 1), (16, 9)])
def test_fifo_headroom_is_measured_at_the_configured_depth(depth, want):
    """Exact values from a real iss_p run (one worker process per depth: the register map is per-process)."""
    r = run("uart_tx", {"speed": 6250000.0}, {"fifo_depth": depth, "engine_route": False})
    h = r["fifo_headroom_at_max_rate"]
    assert h["status"] == "ok" and h["value"] == want and h["unit"] == f"FIFO words free of {depth}"


def test_engines_needed_table_matches_the_rigs():
    """Every perf rig runs at n_engines=1 unless workloads.ENGINES_NEEDED says otherwise (and then it fails below it)."""
    from tools.bench import workloads as wl

    for name in wl.protocol_names():
        need = wl.engines_needed(name)
        params = {"speed": wl.default_speed(name)}
        ok = run(name, params, {"n_engines": need, "engine_route": False})["sweep_point"]["status"]
        assert ok == "ok", f"{name} fails at its declared {need} engine(s)"
        if need > 1:        # the table is not stale: with the refusal switched off, the rig really fails below it
            code = ("import json, sys; from tools.bench import workloads as wl; wl.ENGINES_NEEDED.clear(); "
                    "from tools.bench.backends import sim_p; from tools.bench import knobs; "
                    "from tools.bench.types import RunSpec, Workload; "
                    f"w = Workload({name!r}, 'protocol', {params!r}, ''); "
                    f"recs = sim_p.IssP().run(RunSpec(knobs.normalize({{'n_engines': {need - 1}, 'engine_route': False}}), w, 'iss_p')); "
                    "print(next(r.status for r in recs if r.metric == 'sweep_point'))")
            out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True,
                                 env={**__import__("os").environ, "PYTHONPATH": str(ROOT)}, timeout=200)
            assert out.stdout.strip() == "fail", f"{name} runs at n_engines={need - 1}: lower ENGINES_NEEDED\n{out.stderr[-800:]}"


def test_mix_fails_when_one_protocol_is_overloaded():
    """Negative control: the host polls too rarely for a depth-4 UART RX, which then loses bytes."""
    r = run("mix.uart_tx+uart_rx", {"n": 12, "poll": 20000}, {"n_engines": 2})
    assert val(r, "pass_uart_rx") is False and val(r, "all_pass") is False
    assert r["sweep_point"]["status"] == "fail"


# ---- the backend contract ---------------------------------------------------------------------------

def test_iss_p_refuses_what_it_cannot_run():
    b = load("iss_p")
    mvp = knobs.normalize({})
    bridge = Workload("bridge.uart_rx->uart_tx", "bridge", dict(bridges.DEFAULTS["uart_rx->uart_tx"]))
    assert "engine_route" in b.supports(mvp, bridge)
    assert b.supports(knobs.normalize(ROUTE), bridge) is None
    mix3 = Workload("mix.uart_tx+spi_rx+i2c_rd", "mix", dict(bridges.DEFAULTS["uart_tx+spi_rx+i2c_rd"]))
    assert "n_engines" in b.supports(knobs.normalize(ROUTE), mix3)
    assert b.supports(knobs.normalize({"n_engines": 3}), mix3) is None
    assert "lane" in b.supports(knobs.normalize({"lane": True}), bridge)
    assert "takes params" in b.supports(knobs.normalize(ROUTE), Workload(bridge.id, "bridge", {"nope": 1}))
    assert set(b.knob_support()) == set(knobs.BY_ID)
    ids = {w.id for w in b.workloads()}
    assert {"uart_tx", "spi_host_port", "bridge.spi_target->uart_tx", "mix.uart_tx+uart_rx"} <= ids
    assert b.code_hash() == b.code_hash()


def test_metrics_are_registered_and_a_second_config_needs_another_process():
    from tools.bench.metrics import METRICS
    r = run("bridge.uart_rx->uart_tx", {"in_rate": 115200, "out_rate": 115200, "burst": 2})
    assert set(r) <= set(METRICS)
    # two structural configs in one process would silently mix chips: the backend raises instead
    code = ("import json,sys\nfrom tools.bench import knobs\nfrom tools.bench.backends import load\n"
            "from tools.bench.types import RunSpec, Workload\nb=load('iss_p')\n"
            "w=Workload('bridge.uart_rx->uart_tx','bridge',{'burst':2,'out_rate':115200})\n"
            "b.run(RunSpec(knobs.normalize({'engine_route':True,'fifo_depth':4}),w,'iss_p'))\n"
            "try:\n b.run(RunSpec(knobs.normalize({'engine_route':True,'fifo_depth':8}),w,'iss_p'))\n"
            "except RuntimeError as e:\n print('REFUSED', e)\n")
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env={**__import__("os").environ, "PYTHONPATH": str(ROOT)},
                       capture_output=True, text=True, timeout=120)
    assert "REFUSED" in p.stdout, p.stderr[-1500:]

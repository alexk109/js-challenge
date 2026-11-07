"""Tests of the parametric ISS (tools/bench/sim/pio_iss_p.py), its register map and `fw/regmap.py`."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from fw import regmap
from tools.bench.sim import lockstep_p, pio_iss_p
from tools.bench.sim.pio_iss_p import Chip, Params, chip_class

ROOT = Path(__file__).resolve().parents[3]


def _run(chip, n=1, pin=0):
    for _ in range(n):
        chip.step(pin)


def _w(chip, addr, data):
    chip.step(0, ("w", addr, data))


def _r(chip, addr):
    return chip.step(0, ("r", addr))


# ---- MVP identity ----------------------------------------------------------------------------
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_mvp_lockstep_vs_golden(seed):
    assert lockstep_p.run(seed, 1500) is None


@pytest.mark.parametrize("plant", ["fifo", "sync", "wrap"])
def test_lockstep_negative_control(plant):
    cls = type("Planted", (Chip,), {})
    lockstep_p.PLANTS[plant](cls)
    r = lockstep_p.run(1, 1500, cls)
    assert r is not None, f"planted bug {plant} was not caught"


def test_mvp_regmap_equals_literals():
    m = regmap.MVP
    assert not m.wide
    assert [m.txf(e) for e in range(2)] == [4, 5]
    assert [m.rxf(e) for e in range(2)] == [8, 9]
    assert [m.imem_base(e) for e in range(2)] == [0x12, 0x52]
    assert m.sm_base() == 0x32 and m.addr_space == 128
    assert (m.n_engines, m.imem_words, m.fifo_depth, m.engine_route) == (2, 32, 4, False)
    assert m.cfginfo == 0x0020_0204               # independent literal (docs/mvp-regmap.md DBG_CFGINFO)
    assert Chip.PARAMS == Params(2, 32, 4, False, 3)
    assert regmap.active() is regmap.MVP          # golden / iss_b / rtl: no REGMAP attribute
    assert Chip.REGMAP == regmap.MVP


def test_params_validate_and_from_config():
    assert Params.from_config({}) == Params()
    p = Params.from_config({"n_engines": 3, "fifo_depth": 8, "imem_words": 64, "f_core_mhz": 50.0,
                            "isa_xor": False})   # unmodelled knobs are ignored
    assert (p.n_engines, p.fifo_depth, p.imem_words) == (3, 8, 64)
    for bad in ({"n_engines": 5}, {"imem_words": 48}, {"fifo_depth": 1}, {"t_in": 5}):
        with pytest.raises(ValueError):
            Params.from_config(bad)


def test_env_config(monkeypatch):
    monkeypatch.setenv("BENCH_CONFIG", json.dumps({"fifo_depth": 8, "n_engines": 3, "imem_words": 64}))
    cls = pio_iss_p.chip_class_from_env()
    assert cls.PARAMS.fifo_depth == 8 and cls.REGMAP.wide and cls is pio_iss_p.chip_class_from_env()
    monkeypatch.delenv("BENCH_CONFIG")
    assert pio_iss_p.chip_class_from_env() is Chip


# ---- generalized register map ----------------------------------------------------------------
def test_wide_map_imem_and_cfginfo():
    c = chip_class(Params(n_engines=3, imem_words=64, fifo_depth=8))()
    rm = c.REGMAP
    assert rm.wide and rm.addr_space == 1024
    _w(c, rm.imem(2, 63), 0xE001)
    _w(c, rm.imem(0, 0), 0xE002)
    assert c.engines[2].imem[63] == 0xE001 and c.engines[0].imem[0] == 0xE002
    _w(c, 0x12, 0xFFFF)                           # the old window is reserved when wide
    assert all(w == 0 or w in (0xE001, 0xE002) for e in c.engines for w in e.imem)
    assert _r(c, 0x11) == 8 | 3 << 8 | 64 << 16
    with pytest.raises(ValueError):
        c.step(0, ("r", 1024))


def test_fifo_depth_and_flevel_saturation():
    c = chip_class(Params(fifo_depth=16))()
    for i in range(20):
        _w(c, 0x04, i)
    assert len(c.engines[0].tx) == 16 and c.fdebug >> 16 & 1
    assert _r(c, 0x01) >> 16 & 1                  # TXFULL
    assert _r(c, 0x03) & 0xF == 15                # nibble saturates


def test_page_field_and_pc_width():
    c = chip_class(Params(imem_words=64))()
    _w(c, c.REGMAP.sm(0, 1), (1 << 20) | (3 << 12))       # PAGE=1, wrap_top 3, wrap_bottom 0
    assert _r(c, c.REGMAP.sm(0, 1)) >> 20 & 3 == 1
    mvp = Chip()
    _w(mvp, mvp.REGMAP.sm(0, 1), 3 << 20)
    assert _r(mvp, mvp.REGMAP.sm(0, 1)) >> 20 & 3 == 0     # no page bits at 32 words
    # sequential execution carries across pages and wraps at imem_words
    c = chip_class(Params(imem_words=64))()
    _w(c, c.REGMAP.sm(0, 1), (31 << 12) | 0)              # wrap_top 31 in page 0 -> wraps to 0
    _w(c, 0x00, 1)
    _run(c, 200)
    assert c.engines[0].pc < 32


def test_absent_engines_unmapped():
    c = chip_class(Params(n_engines=1))()
    _w(c, 0x05, 7)
    assert len(c.engines) == 1 and _r(c, 0x09) == 0 and _r(c, 0x38) == 0
    _w(c, 0x00, 0xFF)
    assert _r(c, 0x00) == 1


# ---- engine route ----------------------------------------------------------------------------
def _pushing_chip():
    """engine 0 pushes x=5 forever-ish (set x,5 ; mov isr,x ; push noblock) - written raw."""
    c = chip_class(Params(n_engines=3, engine_route=True, fifo_depth=4))()
    rm = c.REGMAP
    for k, w in enumerate([0xE025, 0xA0C1, 0x8000, 0x0000]):   # set x 5; mov isr,x; push; jmp 0
        _w(c, rm.imem(0, k), w)
    _w(c, rm.sm(0, 1), 3 << 12)                                # wrap_top = 3
    return c


def test_route_moves_words_and_hides_rxf():
    c = _pushing_chip()
    _w(c, 0x00, 1 | 1 << 16)                                   # engine 0 on, link 0 on
    assert _r(c, 0x00) == 1 | 1 << 16
    _run(c, 60)
    assert len(c.engines[1].tx) == 4                            # moved until TX1 is full
    assert _r(c, 0x08) == 0 and c.fdebug >> 8 & 1 == 0          # RXF0 hidden, no RXUNDER
    # backpressure: RX0 holds the overflow, nothing is lost or invented
    _run(c, 200)
    assert len(c.engines[1].tx) == 4 and len(c.engines[0].rx) == 4
    assert set(c.engines[1].tx) == {5}


def test_route_latency_and_host_priority():
    c = chip_class(Params(engine_route=True))()
    _w(c, 0x00, 1 << 16)
    c.engines[0].rx.append(0xAB)                               # as if pushed in an earlier cycle
    c.step(0)
    assert c.engines[1].tx == [0xAB] and c.engines[0].rx == []
    c = chip_class(Params(engine_route=True, fifo_depth=2))()
    _w(c, 0x00, 1 << 16)
    c.engines[1].tx.append(1)
    c.engines[0].rx.extend([0xA, 0xB])
    c.step(0, ("w", 0x05, 2))                                  # host takes the last slot
    assert c.engines[1].tx == [1, 2] and c.engines[0].rx == [0xA, 0xB]


def test_route_off_by_default_and_without_knob():
    c = Chip()
    _w(c, 0x00, 1 << 16)
    assert _r(c, 0x00) == 0
    c = chip_class(Params(engine_route=False, n_engines=3))()
    _w(c, 0x00, 7 << 16)
    assert _r(c, 0x00) == 0


def _blocking_pusher(c, e=0):
    """set x,5 ; mov isr,x ; push block ; (wraps at 3): engine `e` pushes 5 until its RX is full."""
    rm = c.REGMAP
    for k, w in enumerate([0xE025, 0xA0C1, 0x8020]):
        _w(c, rm.imem(e, k), w)
    _w(c, rm.sm(e, 1), 2 << 12)


def test_route_blocking_source_stalls_and_resumes():
    c = chip_class(Params(engine_route=True, fifo_depth=2))()
    _blocking_pusher(c)
    _w(c, 0x00, 1 | 1 << 16)
    _run(c, 100)
    assert len(c.engines[1].tx) == 2 and len(c.engines[0].rx) == 2
    pc = c.engines[0].pc
    _run(c, 30)
    assert c.engines[0].pc == pc and c.state(0)["stalled"] == 1      # stopped on push block
    assert c.engines[1].tx == [5, 5]
    got = 0
    for _ in range(6):                                               # the next engine drains words
        got += len(c.engines[1].tx)
        c.engines[1].tx.clear()
        _run(c, 10)
    assert got >= 6 and all(w == 5 for w in c.engines[1].tx)         # source resumed, 5 each time


def test_route_chain_0_1_2():
    c = chip_class(Params(n_engines=3, engine_route=True, fifo_depth=4))()
    _w(c, 0x00, 3 << 16)
    c.engines[0].rx.extend([1, 2, 3])
    _run(c, 3)
    assert c.engines[0].rx == [] and c.engines[1].tx == [1, 2, 3]    # link 0 only; TX1 is not RX1
    c = chip_class(Params(n_engines=3, engine_route=True, fifo_depth=4))()
    _w(c, 0x00, 3 << 16)
    c.engines[1].rx.extend([7, 8])
    _run(c, 3)
    assert c.engines[2].tx == [7, 8] and c.engines[1].rx == []
    # backpressure through the chain: a full TX2 holds RX1
    c = chip_class(Params(n_engines=3, engine_route=True, fifo_depth=2))()
    _w(c, 0x00, 3 << 16)
    c.engines[1].rx.extend([1, 2, 3])
    _run(c, 10)
    assert c.engines[2].tx == [1, 2] and c.engines[1].rx == [3]


def test_route_disable_leaves_words_for_the_host():
    c = chip_class(Params(engine_route=True, fifo_depth=2))()
    c.engines[1].tx.extend([1, 2])                                   # full: the link cannot move
    _w(c, 0x00, 1 << 16)
    c.engines[0].rx.extend([0xA, 0xB])
    _run(c, 5)
    assert c.engines[0].rx == [0xA, 0xB] and _r(c, 0x08) == 0        # owned by the link
    _w(c, 0x00, 0)
    assert c.peek(0x08) == 0xA                                       # host sees the head again
    assert _r(c, 0x08) == 0xA
    assert c.engines[0].rx == [0xB]


# ---- firmware on a non-MVP config (subprocess: the target is installed once per process) ------
def test_firmware_uart_tx_wide_config():
    env = dict(os.environ, FW_TARGET="iss_p",
               BENCH_CONFIG=json.dumps({"fifo_depth": 8, "imem_words": 64, "n_engines": 3}))
    r = subprocess.run([sys.executable, "-m", "pytest", "-q", "-x", "fw/uart_tx", "-k", "115200"],
                       cwd=ROOT, env=env, capture_output=True, text=True, timeout=170)
    assert r.returncode == 0, r.stdout[-1500:] + r.stderr[-500:]


def test_firmware_engine2_wide_config():
    env = dict(os.environ, BENCH_CONFIG=json.dumps({"fifo_depth": 8, "imem_words": 64, "n_engines": 3}))
    r = subprocess.run([sys.executable, "-m", "tools.bench.sim.fw_engine2"], cwd=ROOT, env=env,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout[-1500:] + r.stderr[-1500:]
    assert "uart_tx engine 2" in r.stdout and "spi engine 2" in r.stdout

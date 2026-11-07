"""The iss_isa backend: measured lever effects, and their cross-check against the analytic estimates."""
import pytest

from tools.bench import backends, crosscheck, knobs
from tools.bench.types import RunSpec, Workload

W = {w: Workload(w, "protocol", {}, "") for w in ("can_1m", "i2c_target_fmplus", "uart_3m")}


def run(wid, partial):
    b = backends.load("iss_isa")
    cfg = knobs.normalize(partial)
    assert b.supports(cfg, W[wid]) is None
    return {r.metric: r for r in b.run(RunSpec(cfg, W[wid], "iss_isa"))}


def test_can_xor():
    off, on = run("can_1m", {}), run("can_1m", {"isa_xor": True})
    assert off["program_words"].value == 25 and off["fits_imem"].value is True and "isa_words_delta" not in off
    assert on["program_words"].value == 39 and on["isa_words_delta"].value == 14
    assert on["fits_imem"].value is False and on["margin"].value == -7
    assert on["crc_loop_sm_cycles_per_bit"].value == 7 and abs(on["crc_loop_max_bit_rate"].value - 50e6 / 7) < 1
    assert all(r.status == "ok" and r.method == "measured" and r.level == "iss" for r in on.values())


def test_a_bigger_imem_fits_it():
    on = run("can_1m", {"isa_xor": True, "imem_words": 64})
    assert on["fits_imem"].value is True and on["program_words"].value == 39


def test_cond_and_irq():
    c = run("i2c_target_fmplus", {"isa_cond": True})
    assert (c["program_words"].value, c["isa_words_delta"].value, c["margin"].value) == (28, -1, 4)
    q = run("uart_3m", {"isa_irq": True})
    assert (q["program_words"].value, q["isa_words_delta"].value) == (22, -2)
    assert "margin" not in q                      # analytic binds on `engines` there: not the same quantity


def test_refusals():
    b = backends.load("iss_isa")
    w = W["can_1m"]
    assert "opcode 110" in b.supports(knobs.normalize({"isa_cond": True, "isa_irq": True}), w)
    assert "fifo_depth" in b.supports(knobs.normalize({"fifo_depth": 8}), w)
    assert b.supports(knobs.normalize({}), Workload("uart_tx", "protocol", {}, ""))
    assert set(b.knob_support()) == {k.id for k in knobs.KNOBS}


def test_crosscheck_against_the_analytic_estimates():
    pytest.importorskip("quickjs")
    iss, ana = backends.load("iss_isa"), backends.load("analytic")
    recs = []
    for wid, lever in (("can_1m", "isa_xor"), ("i2c_target_fmplus", "isa_cond")):
        cfg = knobs.normalize({lever: True})
        for be in (iss, ana):
            recs += be.run(RunSpec(cfg, W[wid], be.name))
    found = {(f["workload"], f["metric"]): f for f in crosscheck.findings(recs) if f["metric"] == "margin"}
    assert found[("can_1m", "margin")]["verdict"] in ("match", "close")       # -7 measured inside the -11..+1 band
    assert found[("i2c_target_fmplus", "margin")]["verdict"] in ("match", "close")

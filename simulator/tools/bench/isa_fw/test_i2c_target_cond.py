"""The shipped fw/i2c_target test-suite, unchanged, against the isa_cond variant of its program (one word shorter)."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def run_suite(pio: Path, isa: str, cfg: dict, target_dir: str = "fw/i2c_target", *extra: str):
    env = {**os.environ, "PYTHONPATH": str(ROOT), "FW_TARGET": "iss_p", "BENCH_CONFIG": json.dumps(cfg),
           "ISA_FW_PKG": "fw.i2c_target", "ISA_FW_PIO": str(pio), "ISA_FW_ISA": isa}
    return subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "tools.bench.isa_fw.variant_plugin", target_dir, *extra],
                          cwd=ROOT, env=env, capture_output=True, text=True, timeout=900)


MARGIN_TEST = "test_the_tightest_margin_at_the_recorded_rates"


@pytest.mark.long
def test_variant_passes_the_unchanged_suite_with_isa_cond():
    """Everything functional passes; the one test that pins the recorded timing margins is judged separately below."""
    p = run_suite(HERE / "i2c_target_cond.pio", "cond", {"isa_cond": True}, "fw/i2c_target",
                  "--deselect", f"fw/i2c_target/test_i2c_target.py::{MARGIN_TEST}")
    assert p.returncode == 0, p.stdout[-3000:] + p.stderr[-1500:]


@pytest.mark.long
def test_the_only_difference_is_one_sm_cycle_in_the_recorded_hold_margin():
    """The variant reacts to the SCL fall one SM cycle (3 ticks at 50 kHz) sooner: tHD;DAT 94 -> 91, still >= 15."""
    p = run_suite(HERE / "i2c_target_cond.pio", "cond", {"isa_cond": True}, "fw/i2c_target",
                  "-k", "tightest")
    out = p.stdout
    assert p.returncode == 1 and "recorded ('tHD;DAT', 94, 15)" in out and "'measured': 91" in out, out[-2500:]


def test_the_plugin_really_swapped_the_program():
    # the suite's own word-count test passes for 28 words too, so check the swap on its own
    code = ("import fw.i2c_target as m\n"
            "from tools.bench.isa_fw import variant_plugin as v\n"
            "v.pytest_collection_modifyitems(None, None, [])\n"
            "print(len(m.assemble_program().instrs))")
    env = {**os.environ, "PYTHONPATH": str(ROOT), "FW_TARGET": "iss_p", "ISA_FW_PKG": "fw.i2c_target",
           "ISA_FW_PIO": str(HERE / "i2c_target_cond.pio"), "ISA_FW_ISA": "cond"}
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True, text=True)
    assert p.stdout.strip().splitlines()[-1] == "28", p.stderr[-800:]


def test_the_variant_does_not_assemble_without_the_extension():
    from asm.pioasm import AsmError, assemble
    with pytest.raises(AsmError):
        assemble((HERE / "i2c_target_cond.pio").read_text())

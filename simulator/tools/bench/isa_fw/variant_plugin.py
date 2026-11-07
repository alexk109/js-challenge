"""pytest plugin: run an existing fw/<proto> test file against a firmware VARIANT assembled with ISA extensions.

    ISA_FW_PKG=fw.i2c_target ISA_FW_PIO=tools/bench/isa_fw/i2c_target_cond.pio ISA_FW_ISA=cond
    FW_TARGET=iss_p BENCH_CONFIG='{"isa_cond": true}' pytest -p tools.bench.isa_fw.variant_plugin fw/i2c_target

Only packages that assemble through their `__init__` globals `PROGRAM_PIO` and `assemble` (fw/i2c_target) are patched;
the file under test is unchanged.
"""
import os
from pathlib import Path


def pytest_collection_modifyitems(session, config, items):
    import importlib

    from asm import pioasm
    pkg = os.environ.get("ISA_FW_PKG")
    if not pkg:
        return
    isa = tuple(x for x in os.environ.get("ISA_FW_ISA", "").split(",") if x)
    m = importlib.import_module(pkg)
    m.PROGRAM_PIO = Path(os.environ["ISA_FW_PIO"]).resolve()
    m.assemble = lambda text, _a=pioasm.assemble: _a(text, isa)

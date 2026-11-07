"""Backend `gl_rig` (level "gl", long): the perf rigs on the post-layout netlist.

`rtl` runs the protocol rigs (verif/perf/rigs.py) on the RTL through the fw cocotb harness.  This is the same
run with the finished flow's netlist in place of src/*.sv: `make -C fw SIM=icarus GL_NETLIST=<final/nl/*.nl.v>`
(PDK io / udp / stdcell models, plus the SRAM macro model for imem_tech=sram2p; -DGL_TEST -DFUNCTIONAL -DSIM,
as test/Makefile GATES=yes).  The netlist is flat and has only the Tiny Tapeout ports, so fw/gl_wrap.py puts a
`gl_top` around it that forces the netlist's own bus nets and reads host reads back from host_spi's capture
register (see its docstring): the rig's host accesses reach the chip in one cycle each, as on the RTL.  The
metric names are rtl's, so `check` compares gl_rig with rtl and iss_p directly: they are the same design and
should agree cycle for cycle.  This is what gives a config off the MVP register map a gate-level functional
check (`gl` / test/test.py speak the MVP map only).

The host SPI port is idle in this setup, so the `spi_host_port` workload is not offered (test/test.py covers
the port at gate level on the MVP).  One build directory per config: fw/sim_build/gl-<config hash>[-sram].
"""
from __future__ import annotations

import hashlib
import os
import sys
from pathlib import Path
from typing import Optional

from .. import knobs, workloads as wl
from ..types import RunSpec, Workload
from . import sim
from .power import FLOW_HINT, find_source, input_hash_for


def netlist_for(cfg: dict) -> Path:
    """The netlist to simulate: env GL_RIG_NETLIST (a mutated copy, for tests) or the config's finished flow."""
    over = os.environ.get("GL_RIG_NETLIST")
    if over:
        return Path(over)
    return sorted((find_source(cfg).final() / "nl").glob("*.nl.v"))[0]


def build_tag(cfg: dict) -> str:
    """Build-dir tag: config hash plus the netlist's content hash, so a re-run or restored flow never reuses a stale sim.vvp."""
    return knobs.config_hash(cfg) + "-" + hashlib.sha256(netlist_for(cfg).read_bytes()).hexdigest()[:8]


class GlRig(sim.SimBackend):
    def __init__(self) -> None:
        super().__init__("rtl")         # the rtl fw target: same rigs, same harness, a netlist behind it
        self.name = "gl_rig"
        self.level = "gl"

    # ---- contract -------------------------------------------------------------------------------

    def _rig_refusal(self, config: dict, workload: Workload) -> Optional[str]:
        if workload.id == wl.HOST_PORT:
            return "gl_rig drives the chip's register bus directly; the host SPI port is idle (test/test.py covers it: backend gl)"
        return super().supports(config, workload)

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        reason = self._rig_refusal(config, workload)
        if reason:
            return reason
        if find_source(knobs.normalize(config)) is None:
            return FLOW_HINT
        return None

    def workloads(self) -> list[Workload]:
        mvp = knobs.normalize({})
        return [w for w in wl.all_workloads() if self._rig_refusal(mvp, w) is None]

    def is_long(self, spec: RunSpec) -> bool:
        return True

    def input_hash(self, spec: RunSpec) -> str:
        return input_hash_for(spec.config)

    def code_hash(self, config: Optional[dict] = None) -> str:
        """The rig and harness code, the gl wrapper and Makefile, and (sram2p) the PDK macro model; src/*.sv is not
        an input here -- the netlist is, via input_hash."""
        h = hashlib.sha256()
        for p in sim._files(self._COMMON + ["tools/bench/backends/gl_rig.py"]):
            h.update(str(p.relative_to(sim.ROOT)).encode() + b"\0" + p.read_bytes() + b"\0")
        if config is None or knobs.normalize(config)["imem_tech"] == "sram2p":
            for p in sim.SRAM_MODEL_FILES():
                h.update(p.name.encode() + b"\0" + (p.read_bytes() if p.is_file() else b"missing") + b"\0")
        return h.hexdigest()[:16]

    # ---- running --------------------------------------------------------------------------------

    def _install(self, sram: bool, cfg: dict) -> None:
        src = find_source(cfg)
        if src is None:
            raise ValueError(FLOW_HINT)
        nl = netlist_for(cfg)
        h = sys.modules.get("fw.rtl_harness")
        if h is not None and not h.GL:
            raise RuntimeError("this process already loaded the RTL harness; gate level is process-global, "
                               "run gl_rig in its own process")
        os.environ["FW_RTL_SIM"] = "icarus"
        os.environ["GL_NETLIST"] = str(nl)
        os.environ["GL_TAG"] = build_tag(cfg)
        os.environ["FW_RTL_GL"] = str(nl)
        super()._install(sram, cfg)

    def _command(self, w: Workload, sram: bool, cfg: Optional[dict] = None) -> str:
        nl = find_source(cfg or {})
        env = (f"GL_NETLIST={nl.rundir}/final/nl/*.nl.v GL_TAG={build_tag(cfg)} " if nl and cfg is not None else "")
        return "FW_RTL_SIM=icarus " + env + super()._command(w, sram, cfg)



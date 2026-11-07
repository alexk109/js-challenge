"""`iss_isa`: measured instruction-set-lever effects, on the parametric ISS (tools/bench/sim/ISA_EXT.md).

Replaces archsim's hand-written `isa` effect rows (tools/archsim/data/inputs.json) with measurements. Three workloads,
named as in archsim so a cross-check joins them with the analytic backend on `(config, workload, metric)`:

    can_1m             isa_xor   crc15_xor.pio against verif/vip/can.py's CRC (words, cycles per bit, max rate)
    i2c_target_fmplus  isa_cond  fw/i2c_target/program.pio with `jmpc pin_low` (words; the unchanged fw suite is the
                                 correctness check: tools/bench/isa_fw/test_i2c_target_cond.py)
    uart_3m            isa_irq   fw/uart_rx/program.pio with `irq 4` for the framing error (words; test_uart_rx_irq.py)

Metrics: program_words, isa_words_delta, fits_imem, margin (IMEM words left; emitted only where the analytic backend's
binding limit for the same cell is also the IMEM, so the two mean the same), and for can_1m crc_loop_sm_cycles_per_bit and
crc_loop_max_bit_rate of the CRC loop. Knobs: the isa_* ones and imem_words; everything else must be the default.
"""
from __future__ import annotations

import hashlib
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from asm.pioasm import assemble

from .. import knobs
from ..types import Record, RunSpec, Workload

ROOT = Path(__file__).resolve().parents[3]
ISA_DIR = ROOT / "tools" / "bench" / "isa_fw"

#: workload -> (lever, one-line label)
WORKLOADS = {
    "can_1m": ("isa_xor", "CAN 1 Mb/s: CRC-15 step with the XOR op (crc15_xor.pio)"),
    "i2c_target_fmplus": ("isa_cond", "I2C target: SCL-fall test with jmpc pin_low (i2c_target_cond.pio)"),
    "uart_3m": ("isa_irq", "UART RX: framing error as IRQ flag 4 (uart_rx_irq.pio)"),
}
#: Words the rest of the can_1m program takes: fw/can's 5-word bit engine (measured) and the 20-word stuff detector,
#: which was counted by hand; there is no program for it in the repo.
CAN_BASE_MEASURED, CAN_BASE_COUNTED = 5, 20


def _git_rev() -> str:
    import subprocess
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


def _words(path: Path, isa=()) -> int:
    return len(assemble(path.read_text(), isa)[0].instrs)


class IssIsa:
    name = "iss_isa"
    level = "iss"
    method = "measured"

    # ---- contract --------------------------------------------------------------------------------
    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        if workload.id not in WORKLOADS or workload.kind != "protocol":
            return f"iss_isa measures {', '.join(WORKLOADS)}, not {workload.id!r}"
        cfg = knobs.normalize(config)
        if cfg["isa_cond"] and cfg["isa_irq"]:
            return "isa_cond and isa_irq both need opcode 110 at 16 bits: choose one (ISA_EXT.md)"
        for kid, default in knobs.defaults().items():
            if cfg[kid] != default and not kid.startswith("isa_") and kid != "imem_words":
                return f"iss_isa varies only the isa_* knobs and imem_words; {kid}={cfg[kid]!r} (default {default!r})"
        return None

    def knob_support(self) -> dict:
        return {k.id: ("full" if k.id.startswith("isa_") else
                       "partial: only sets the IMEM size the words are compared with" if k.id == "imem_words" else
                       "default_only") for k in knobs.KNOBS}

    def workloads(self) -> list[Workload]:
        return [Workload(w, "protocol", {}, label) for w, (_, label) in WORKLOADS.items()]

    def code_hash(self) -> str:
        h = hashlib.sha256()
        files = sorted(ISA_DIR.glob("*.py")) + sorted(ISA_DIR.glob("*.pio")) + [
            ROOT / "tools/bench/sim/pio_iss_p.py", ROOT / "asm/pioasm.py", ROOT / "fw/can/program.pio",
            ROOT / "fw/i2c_target/program.pio", ROOT / "fw/uart_rx/program.pio", ROOT / "verif/vip/can.py", Path(__file__)]
        for p in files:
            h.update(p.name.encode() + b"\0" + p.read_bytes() + b"\0")
        return h.hexdigest()[:16]

    # ---- running ---------------------------------------------------------------------------------
    def _measure(self, wid: str, lever_on: bool) -> dict:
        """-> {words, base_words, notes{metric: str}, extra: [(metric, value, unit, note)]}"""
        if wid == "can_1m":
            from tools.bench.isa_fw import crc
            base = CAN_BASE_MEASURED + CAN_BASE_COUNTED
            if not lever_on:
                return {"words": base, "base": base, "extra": [], "ok": True, "why": "",
                        "note": f"MVP: {CAN_BASE_MEASURED} fw/can (assembled) + {CAN_BASE_COUNTED} stuff detector (counted, "
                                "not shipped); the MVP has no XOR, so no CRC program exists (CANNOT-EXPRESS.md 1)"}
            ok, why = self._crc_correct(crc)
            rate_ticks = self._crc_min_bit_ticks(crc)
            extra = [("crc_loop_sm_cycles_per_bit", crc.LOOP_SM_CYCLES, "cycles", "crc15_xor.pio loop, both paths (asm + run)"),
                     ("max_bit_rate_ticks", rate_ticks, "ticks", "smallest bit time (core cycles) at which the CRC "
                                                                  "matches the VIP's at CLKDIV 1, some sampling phase")]
            return {"words": base + crc.words(), "base": base, "extra": extra, "ok": ok, "why": why,
                    "crc_words": crc.words(),
                    "note": f"{CAN_BASE_MEASURED} fw/can + {CAN_BASE_COUNTED} counted detector + {crc.words()} crc15_xor.pio"}
        if wid == "i2c_target_fmplus":
            base = _words(ROOT / "fw/i2c_target/program.pio")
            if not lever_on:
                return {"words": base, "base": base, "extra": [], "ok": True, "why": "", "note": "fw/i2c_target/program.pio assembled"}
            return {"words": _words(ISA_DIR / "i2c_target_cond.pio", ("cond",)), "base": base, "extra": [], "ok": True,
                    "why": "", "note": "i2c_target_cond.pio assembled with isa=cond; the unchanged fw suite passes "
                                       "(test_i2c_target_cond.py), one recorded hold margin moves by one SM cycle"}
        base = _words(ROOT / "fw/uart_rx/program.pio")
        if not lever_on:
            return {"words": base, "base": base, "extra": [], "ok": True, "why": "", "note": "fw/uart_rx/program.pio assembled"}
        return {"words": _words(ISA_DIR / "uart_rx_irq.pio", ("irq",)), "base": base, "extra": [], "ok": True, "why": "",
                "note": "uart_rx_irq.pio assembled with isa=irq; framing error = IRQ flag 4 (test_uart_rx_irq.py)"}

    @staticmethod
    def _crc_correct(crc) -> tuple[bool, str]:
        from verif.vip import can as vip
        for f in (vip.Frame(0x123, b"\xde\xad"), vip.Frame(0x7FF), vip.Frame(0x001, bytes(range(8)))):
            ok, why = crc.check_frame(f, 7, 1, 0, 5)
            if not ok:
                return False, f"{f}: {why}"
        return True, ""

    @staticmethod
    def _crc_min_bit_ticks(crc) -> int:
        from verif.vip import can as vip
        f = vip.Frame(0x123, b"\xde\xad")
        for bt in range(3, 16):
            if any(crc.check_frame(f, bt, 1, 0, ph)[0] for ph in range(bt)):
                return bt
        return -1

    def _analytic_binds_words(self, cfg: dict, wid: str) -> bool:
        """True if the analytic backend's binding limit for this cell is the IMEM (so its `margin` is words left)."""
        try:
            from .analytic import AnalyticBackend
            a = AnalyticBackend()
            spec = RunSpec(cfg, Workload(wid, "protocol", {}, ""), "analytic")
            if a.supports(cfg, spec.workload):
                return False
            return any(r.metric == "binding_limit" and r.value == "program_words" for r in a.run(spec))
        except Exception:
            return False

    def run(self, spec: RunSpec) -> list[Record]:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        cfg = knobs.normalize(spec.config)
        wid = spec.workload.id
        lever = WORKLOADS[wid][0]
        on = bool(cfg[lever])
        started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        t0 = time.perf_counter()
        m = self._measure(wid, on)
        chash, key = knobs.config_hash(cfg), spec.key(self.code_hash())
        prov = {"command": f"python -m tools.bench run --backend iss_isa --workload {wid} "
                           + " ".join(f"--set {k}={v}" for k, v in sorted(knobs.diff(cfg).items())),
                "commit": _git_rev(), "dirty": False, "started": started, "duration_s": round(time.perf_counter() - t0, 3)}
        recs: list[Record] = []

        def add(metric, value, unit, note="", status="ok", method="measured"):
            recs.append(Record(workload=wid, metric=metric, value=value, unit=unit, level=self.level, backend=self.name,
                               method=method, status=status, config=cfg, config_hash=chash,
                               workload_params=dict(spec.workload.params), note=note, provenance=dict(prov), run_key=key))

        st = "ok" if m["ok"] else "fail"
        why = "" if m["ok"] else f"correctness check failed: {m['why']}; "
        cap = cfg["imem_words"]
        add("program_words", m["words"], "words", why + m["note"], status=st)
        if on:
            add("isa_words_delta", m["words"] - m["base"], "words",
                f"{lever} on minus off, same workload program" + ("; the lever ADDS a CRC program the MVP cannot have"
                                                                    if wid == "can_1m" else ""), status=st)
        add("fits_imem", bool(m["words"] <= cap), "bool",
            f"{m['words']} words in one engine's IMEM of {cap}" + ("; a second engine's IMEM would take the CRC program "
                                                                    "alone" if wid == "can_1m" and on else ""), status=st)
        if self._analytic_binds_words(cfg, wid):
            add("margin", cap - m["words"], "words", "IMEM words left (analytic binds on the same limit)", status=st)
        for metric, value, unit, note in m["extra"]:
            if metric == "max_bit_rate_ticks":
                if value > 0:
                    add("crc_loop_max_bit_rate", cfg["f_core_mhz"] * 1e6 / value, "Hz", "CRC loop alone, not the chip-level max_bit_rate: " + note, status=st)
            else:
                add(metric, value, unit, note, status=st)
        return recs

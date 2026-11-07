"""Analytic backend: the archsim model (tools/archsim/model/archsim-model.js) run under QuickJS.

Level "analytic". One RunSpec = one archsim workload (protocol) or the chip-level check (workload "chip").
Bench config -> archsim arch is a pure projection (ARCH_KNOBS); bench knobs archsim does not have are
honoured only at their default (supports() refuses otherwise); archsim-only knobs stay at archsim defaults.
Nothing here re-implements the model: verdicts, margins and bands are archsim's own numbers.

method: "estimate" when the quantity rests on a judgement-status input (the rule is archsim's
`judgedInputs` in whatWouldItTake: walk the term tree of the binding limit / chip item down to
parameters, blocks and ISA effect rows; any with status "judgement" => estimate), else "derived".
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import time
from pathlib import Path
from typing import Optional

from .. import knobs as K
from ..types import Record, RunSpec, Workload

ARCHSIM = Path(__file__).resolve().parents[2] / "archsim"
MODEL_JS = ARCHSIM / "model" / "archsim-model.js"
INPUTS_JSON = ARCHSIM / "data" / "inputs.json"

#: bench knobs that map 1:1 (same id, same meaning) onto archsim arch fields.
ARCH_KNOBS = ("f_core_mhz", "n_engines", "imem_words", "imem_tech", "fifo_depth", "host_link",
              "lane", "die", "isa_xor", "isa_cond", "isa_irq")
#: bench-only knobs: archsim has no such knob, so only the default can be honoured.
BENCH_ONLY = ("t_in", "engine_route")

#: archsim workload id -> perf-b rig/protocol names (verif/perf/rigs.py, research/perf/b/perf.csv `protocol`).
#: The fw_* (mvp_fw tag) rows are the same firmware on the same protocol; speed settings differ (archsim fixes
#: one rate per workload, perf-b sweeps a ladder), so a join also needs a speed param (task D defines it).
#: Gaps (no perf-b counterpart): every cap19 / stretch / device / frames workload except the near-matches below.
ANALYTIC_TO_SIM = {
    "fw_uart_tx": "uart_tx", "fw_uart_rx": "uart_rx", "fw_i2c": "i2c", "fw_jtag": "jtag", "fw_swd": "swd",
    "fw_spi_m0": "spi", "fw_onewire": "onewire", "fw_manchester": "manchester",
    "fw_manchester_even": "manchester_even", "fw_dmx512": "dmx512", "fw_ps2": "ps2",
    "fw_wiegand": "wiegand", "fw_quadrature": "quadrature", "fw_necir": "necir",
    # perf-b's ws2812 rig runs the fast program (program=1; perf-a-vs-b.md: "b measures ws2812_fast"), so the exact
    # archsim counterpart is fw_ws2812_fast; the slow fw_ws2812 program is in NEAR_MATCHES (no sim measures it).
    "fw_ws2812_fast": "ws2812",
}
#: Near-matches (same protocol, different role or operating point); excluded from cross-checks by default.
NEAR_MATCHES = {
    "fw_ws2812": ("ws2812", "the slow ws2812 program; perf-b measures only the fast one (class NOB)"),
    "ws2812": ("ws2812", "cap19 operating point, not the perf-b firmware ladder"),
    "uart_3m": ("uart_rx", "3 Mbaud TX+RX capability row vs the uart_rx firmware rig"),
    "i2c_fmplus_ctrl": ("i2c", "Fm+ controller capability row vs the generic i2c rig ladder"),
    "spi_host_25m": ("spi", "25 MHz capability row, beyond the perf-b spi ladder"),
    "swd_host_10m": ("swd", "10 MHz capability row vs the swd firmware rig"),
    "jtag_target_10m": ("jtag", "target role at 10 MHz vs the host-side jtag rig"),
}
#: The host-port cost bench. Not an archsim workload: the analytic backend serves it from archsim's link model
#: (derive().link windows, the numbers calibration K1-K4 check), so it joins the sims' spi_host_port rows.
HOST_PORT = "spi_host_port"
SIM_WITHOUT_ANALYTIC = ()
#: Not emitted on purpose (see README "What the bench checks today"): sm_cycles_per_bit (archsim's `ipu` is core cycles
#: per wire bit at the fastest legal CLKDIV, perf-a's number, not perf-b's SM cycles per bit) and pin_event_to_response
#: (no fw_* cell has a reflex limit; reflex_sm is a placeholder). archsim's `measured.b_max_bit_rate` is perf-b's number
#: stored as an input; max_bit_rate below is the model's own K8 value and never reads it. A record that is a copied input
#: must carry the note "input copied from <source>"; crosscheck then reports the pair with that source as "identity".

_JS_HELPER = r"""
var __B = (function () {
  function refStatus(ref) {
    if (ref.kind === 'param') { var p = DATA.params[ref.id]; return p ? p.status : null; }
    if (ref.kind === 'block') { var b = DATA.blocks[ref.id]; return b ? b.status : null; }
    if (ref.kind === 'isa_effect') {
      var rows = ((DATA.isa || {}).effects || []).filter(function (e) { return e.wl === ref.wl && e.lever === ref.lever; });
      return rows.length ? rows[0].status : null;
    }
    return null;
  }
  // mirrors archsim judgedInputs/walk: judgement-status inputs reachable through derived records
  function judged(terms, recs, any, out, seen) {
    (terms || []).forEach(function (t) {
      var r = t.ref; if (!r) return;
      if (r.kind === 'derived') {
        var rr = recs[r.id];
        if (rr && !seen[r.id]) { seen[r.id] = true; judged(rr.terms, recs, any, out, seen); }
      } else if (refStatus(r) === 'judgement' && (any || r.kind === 'param' || r.kind === 'isa_effect')) {
        out[r.kind + ':' + (r.id || r.lever) + (r.wl ? ':' + r.wl : '')] = true;
      }
    });
  }
  function ivl(v) { return (v !== null && typeof v === 'object') ? { nom: v.nom, pess: v.pess, opt: v.opt } : { nom: v, pess: v, opt: v }; }
  function cell(arch, wl) {
    var d = ArchSim.derive(arch, DATA), c = ArchSim.evaluateCell(d.arch, wl, DATA, { derived: d });
    var recs = Object.assign({}, d.records, c.records), out = {};
    c.limits.forEach(function (L) {   // ISA effect rows anywhere in the cell
      judged(L.terms.filter(function (t) { return t.ref && t.ref.kind === 'isa_effect'; }), recs, true, out, {});
      if (L.id === c.binding) judged(L.terms, recs, false, out, {});
    });
    var h = c.headline, lim = c.limits.filter(function (L) { return L.id === c.binding; })[0];
    return { verdict: c.verdict, binding: c.binding, binding_label: lim.label, unit: h.unit, value: ivl({ nom: h.nom, pess: h.pess, opt: h.opt }),
             formula: lim.formula, judged: Object.keys(out).sort(), blocked_by: c.blocked_by };
  }
  function chip(arch) {
    var d = ArchSim.derive(arch, DATA), cc = ArchSim.chipCheck(d.arch, DATA, { derived: d }), items = {}, recs = d.records;
    cc.items.forEach(function (it) {
      var out = {}; judged(cc.records[it.id].terms, recs, true, out, {});
      items[it.id] = { value: ivl(it.value), unit: it.unit, verdict: it.verdict, judged: Object.keys(out).sort() };
    });
    function jr(id) { var out = {}; judged([{ ref: { kind: 'derived', id: id } }], recs, true, out, {}); return Object.keys(out).sort(); }
    var corner = d.arch.signoff_corner;
    return { verdict: cc.verdict, driver: cc.driver, items: items,
             area: ivl(d.area.total), area_judged: jr('area_total'), util_pct: ivl(d.area.util_pct), util_judged: jr('util_pct'),
             fmax: ivl(d.fmax[corner]), fmax_judged: jr('fmax_' + corner), corner: corner,
             pins: d.pins, pins_total: DATA.iss.pins.ui_in + DATA.iss.pins.uo_out + DATA.iss.pins.uio };
  }
  // the quantities the sims measure (perf.csv names): max rate as archsim's calibration K8 computes it (cycles + protocol
  // ceiling, iss environment), the perf-a SM cycles per wire bit (an input), the reflex sum T_IN + T_OUT + reflex_sm
  function sim(arch, wlId) {
    var wl = DATA.workloads.filter(function (w) { return w.id === wlId; })[0], AI = Object.assign({}, arch, { env: 'iss' });
    var out = { rate_ceiling: wl.rate_ceiling_hz, ipu: wl.ipu, ipu_src: wl.ipu_src, reflex_sm: wl.reflex_sm,
                T_IN: DATA.iss.T_IN, T_OUT: DATA.iss.T_OUT, isa_levers: !!(arch.isa_xor || arch.isa_cond || arch.isa_irq) };
    var mr = {};
    ['nom', 'pess', 'opt'].forEach(function (v) { var r = ArchSim.maxRate(AI, wl, DATA, v, { only: ['cycles', 'rate_ceiling'] }); mr[v] = r.rate_hz; out.max_rate_binding = r.binding; });
    out.max_rate = mr;
    out.judged = cell(arch, wlId).judged;
    return out;
  }
  function host(arch) {
    var d = ArchSim.derive(arch, DATA), L = d.link;
    return { write: L.write_window_1, read: L.read_window_1, txf0_burst_4: L.write_window_B, rxf0_burst_4: L.read_window_B };
  }
  return { cell: cell, chip: chip, sim: sim, host: host };
})();
"""


def _inf(x):
    if isinstance(x, str) and x in ("Infinity", "-Infinity"):
        return float(x.replace("Infinity", "inf"))
    if isinstance(x, list):
        return [_inf(v) for v in x]
    if isinstance(x, dict):
        return {k: _inf(v) for k, v in x.items()}
    return x


_GIT: dict = {}


def _git() -> tuple:
    if not _GIT:
        root = Path(__file__).resolve().parents[3]
        try:
            _GIT["c"] = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, timeout=10).stdout.strip()
            _GIT["d"] = bool(subprocess.run(["git", "status", "--porcelain", "--", "tools/archsim", "tools/bench"], cwd=root,
                                            capture_output=True, text=True, timeout=20).stdout.strip())
        except Exception:
            _GIT.update(c="unknown", d=True)
    return _GIT["c"], _GIT["d"]


class AnalyticBackend:
    name = "analytic"
    level = "analytic"

    def __init__(self):
        self._ctx = None
        self._data = None

    # ---- model access
    def _js(self):
        if self._ctx is None:
            import quickjs  # lazy: a missing quickjs only breaks this backend
            self._data = json.loads(INPUTS_JSON.read_text())
            ctx = quickjs.Context()
            ctx.eval(MODEL_JS.read_text())
            ctx.eval("var DATA = " + json.dumps(self._data) + ";")
            ctx.eval(_JS_HELPER)
            self._ctx = ctx
        return self._ctx

    def _call(self, expr: str):
        return _inf(json.loads(self._js().eval("JSON.stringify(ArchSim.toJSON(" + expr + "))")))

    def _wl_ids(self) -> list:
        self._js()
        return [w["id"] for w in self._data["workloads"]]

    # ---- Backend protocol
    def knob_support(self) -> dict:
        out = {}
        for k in K.KNOBS:
            if k.id == "imem_words":
                out[k.id] = "partial: values outside archsim's domain (imem_words 16) are refused by supports()"
            elif k.id in ARCH_KNOBS:
                out[k.id] = "full"
            else:
                out[k.id] = "default_only"
        return out

    def workloads(self):
        self._js()
        ws = [Workload(w["id"], "protocol", {}, w["name"]) for w in self._data["workloads"]]
        ws.append(Workload(HOST_PORT, "protocol", {}, "SPI host port cost per register access (archsim link model)"))
        return ws + [Workload("chip", "chip", {}, "Chip-level check (area, core clock, pins)")]

    def _arch(self, config: dict) -> dict:
        return {k: config[k] for k in ARCH_KNOBS}

    def supports(self, config: dict, workload: Workload) -> Optional[str]:
        d = K.defaults()
        for kid in BENCH_ONLY:
            if config.get(kid, d[kid]) != d[kid]:
                return f"archsim has no knob {kid}; analytic runs it only at the default ({d[kid]!r})"
        if workload.kind == "chip":
            if workload.id != "chip":
                return f"unknown chip workload {workload.id!r}"
        elif workload.kind == "protocol":
            if workload.id not in self._wl_ids() and workload.id != HOST_PORT:
                return f"{workload.id!r} is not an archsim workload"
        else:
            return f"analytic does not run {workload.kind} workloads"
        bad = self._call("ArchSim.checkArch(Object.assign(ArchSim.defaultArch()," + json.dumps(self._arch(config)) + "))")
        if bad:
            return "outside archsim's domain: " + "; ".join(bad)
        return None

    def code_hash(self) -> str:
        h = hashlib.sha256()
        for p in (MODEL_JS, INPUTS_JSON, Path(__file__)):
            h.update(p.read_bytes())
        return h.hexdigest()[:16]

    def _sim_metrics(self, spec, arch, add, num) -> None:
        """The quantity perf-b measures that archsim predicts, under perf.csv's name (speed-free: metrics.SPEED_INDEPENDENT)."""
        x = self._call(f"__B.sim({arch}, {json.dumps(spec.workload.id)})")
        rc = x["rate_ceiling"]      # the ceiling is an input: a value equal to it (whichever limit the tie names) is a copy
        ceiling = x["max_rate_binding"] == "rate_ceiling" or (rc is not None and abs(x["max_rate"]["nom"] - rc) <= 1e-3 * rc)
        num(x["max_rate"], x["judged"], "max_bit_rate", "bit/s",
            note=f"archsim K8 max rate: min(f_core / ipu, protocol ceiling), binding {x['max_rate_binding']}; "
                 + ("input copied from archsim rate_ceiling_hz (the protocol ceiling binds: its value is perf-a's, not a prediction)"
                    if ceiling else "a model output, perf-b's measured value is not an input of it"))

    # ---- run
    def _prov(self, spec: RunSpec, started: str, dur: float) -> dict:
        commit, dirty = _git()
        sets = " ".join(f"--set {k}={v}" for k, v in sorted(K.diff(spec.config).items()))
        return {"command": f"python -m tools.bench run --backend analytic --workload {spec.workload.id} {sets}".strip(),
                "commit": commit, "dirty": dirty, "started": started, "duration_s": round(dur, 4),
                "input_hashes": {"archsim-model.js": hashlib.sha256(MODEL_JS.read_bytes()).hexdigest()[:16],
                                 "inputs.json": hashlib.sha256(INPUTS_JSON.read_bytes()).hexdigest()[:16]}}

    def run(self, spec: RunSpec) -> list:
        reason = self.supports(spec.config, spec.workload)
        if reason:
            raise ValueError(reason)
        t0 = time.time()
        started = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0))
        arch = json.dumps(self._arch(spec.config))
        cfg, chash = K.normalize(spec.config), K.config_hash(spec.config)
        rkey = spec.key(self.code_hash())
        recs: list = []

        def add(metric, value, unit, judged, band=None, status="ok", note="", method=None):
            n = note
            if judged:
                n = (n + "; " if n else "") + "judgement inputs: " + ", ".join(judged)
            recs.append(Record(
                workload=spec.workload.id, metric=metric, value=value, unit=unit, level="analytic", backend="analytic",
                method=method or ("estimate" if judged else "derived"), status=status,
                band=list(band) if band is not None else None, config=cfg, config_hash=chash,
                workload_params=dict(spec.workload.params), note=n, run_key=rkey))

        def num(iv, judged, metric, unit, note=""):
            """interval {nom,pess,opt} -> record; an unbounded value is n/a (no constraint), never a fake number"""
            if any(abs(iv[f]) == float("inf") for f in ("nom", "pess", "opt")):
                add(metric, None, unit, judged, status="n/a", note=(note + "; " if note else "") + "unbounded: this limit does not constrain")
            else:
                add(metric, iv["nom"], unit, judged, band=[iv["pess"], iv["opt"]], note=note)

        if spec.workload.id == HOST_PORT:
            if cfg["host_link"] != "spi":
                add("write", None, "core cycles per transaction", [], status="n/a", note=f"host_link={cfg['host_link']}: no SPI host port")
            else:
                h = self._call(f"__B.host({arch})")
                for m, v in h.items():
                    add(m, v, "core cycles per transaction", [], method="derived",
                        note="archsim link model: CSn-low window of the SPI host port at 8 core ticks per SCK bit "
                             "(framing calibrated against perf-a/perf-b host-port runs, K1-K4)")
        elif spec.workload.kind == "protocol":
            c = self._call(f"__B.cell({arch}, {json.dumps(spec.workload.id)})")
            add("verdict", c["verdict"], "", c["judged"])
            add("binding_limit", c["binding"], "", c["judged"], note=c["binding_label"])
            num(c["value"], c["judged"], "margin", c["unit"], note=c["binding_label"] + ": " + c["formula"])
            if spec.workload.id in ANALYTIC_TO_SIM:
                self._sim_metrics(spec, arch, add, num)
        else:
            ch = self._call(f"__B.chip({arch})")
            it = ch["items"]
            cj = sorted({j for x in it.values() for j in x["judged"]})
            add("chip_verdict", ch["verdict"], "", cj, note="driver: " + ch["driver"])
            mvp_pt = K.config_hash(spec.config) == K.config_hash({})
            num(ch["area"], ch["area_judged"], "area_um2", "um2",
                note="input copied from flow (archsim K10: pnr_factor is fitted to the MVP flow area, so the MVP "
                     "agrees by construction; other configs are predictions)" if mvp_pt else "")
            num(ch["util_pct"], ch["util_judged"], "area_utilization_pct", "%")
            num(ch["fmax"], ch["fmax_judged"], "fmax_mhz", "MHz", note=f"{ch['corner']} corner")
            for iid, metric in (("die_area", "die_area_margin_um2"), ("core_clock", "core_clock_margin_mhz"),
                                ("board_clock", "board_clock_margin_mhz"), ("pins_budget", "pins_left"),
                                ("isa_encoding", "isa_opcode_room")):
                if iid in it:
                    x = it[iid]
                    num(x["value"], x["judged"], metric, x["unit"], note=f"chip check: {x['verdict']}")
            p = ch["pins"]
            used = p["link_in"] + p["link_out"] + p["reserved"]
            add("pins_fixed", used, "pins", [], method="derived",
                note=f"host link {p['link_in']} in + {p['link_out']} out + {p['reserved']} reserved, of {ch['pins_total']} project pins")
        prov = self._prov(spec, started, time.time() - t0)
        for r in recs:
            r.provenance = prov
        return recs

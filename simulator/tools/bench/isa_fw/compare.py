"""Lever x protocol: archsim's estimate next to the measurement (the table in tools/bench/notes/L.md).

    python -m tools.bench.isa_fw.compare            # markdown to stdout
"""
from __future__ import annotations

import json

from tools.bench import backends, crosscheck, knobs
from tools.bench.backends.isa import WORKLOADS
from tools.bench.types import RunSpec, Workload

from tools.bench.backends.analytic import INPUTS_JSON as INPUTS


def effect(wid: str, lever: str) -> dict:
    d = json.loads(INPUTS.read_text())
    return next(e for e in d["isa"]["effects"] if e["wl"] == wid and e["lever"] == lever)


def rows() -> tuple[list[dict], list[dict]]:
    iss, ana = backends.load("iss_isa"), backends.load("analytic")
    out, allrecs = [], []
    for wid, (lever, _) in WORKLOADS.items():
        w = Workload(wid, "protocol", {}, "")
        got = {}
        for on in (False, True):
            cfg = knobs.normalize({lever: on} if on else {})
            for be in (iss, ana):
                recs = be.run(RunSpec(cfg, w, be.name))
                allrecs += recs
                got[(be.name, on)] = {r.metric: r for r in recs}
        e = effect(wid, lever)
        m_on, a_on, a_off = got[("iss_isa", True)], got[("analytic", True)], got[("analytic", False)]
        out.append({
            "lever": lever, "workload": wid,
            "est_dwords": (e["d_words"]["nom"], e["d_words"]["lo"], e["d_words"]["hi"]),
            "est_dipu": (e["d_ipu"]["nom"], e["d_ipu"]["lo"], e["d_ipu"]["hi"]),
            "est_verdict_off": a_off["verdict"].value, "est_verdict_on": a_on["verdict"].value,
            "est_margin": a_on["margin"].value, "est_margin_band": a_on["margin"].band,
            "m_words": m_on["program_words"].value, "m_dwords": m_on["isa_words_delta"].value,
            "m_fits": m_on["fits_imem"].value,
            "m_margin": m_on["margin"].value if "margin" in m_on else None,
            "m_cycles": m_on["crc_loop_sm_cycles_per_bit"].value if "crc_loop_sm_cycles_per_bit" in m_on else None,
            "m_rate": m_on["crc_loop_max_bit_rate"].value if "crc_loop_max_bit_rate" in m_on else None,
        })
    return out, crosscheck.findings(allrecs)


def main() -> None:
    table, found = rows()
    print("| lever | workload | est. d_words (nom, lo..hi) | measured d_words | est. verdict off->on | est. IMEM margin | "
          "measured words / fits / margin | cycles per bit | max rate |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in table:
        print(f"| {r['lever']} | {r['workload']} | {r['est_dwords'][0]:+d} ({r['est_dwords'][1]:+d}..{r['est_dwords'][2]:+d}) | "
              f"{r['m_dwords']:+d} | {r['est_verdict_off']} -> {r['est_verdict_on']} | {r['est_margin']} "
              f"{r['est_margin_band']} | {r['m_words']} / {r['m_fits']} / {r['m_margin']} | "
              f"{r['m_cycles'] if r['m_cycles'] is not None else '-'} (est. d_ipu {r['est_dipu'][0]:+d}) | "
              f"{r['m_rate'] / 1e6:.2f} Mbit/s |" if r["m_rate"] else
              f"| {r['lever']} | {r['workload']} | {r['est_dwords'][0]:+d} ({r['est_dwords'][1]:+d}..{r['est_dwords'][2]:+d}) | "
              f"{r['m_dwords']:+d} | {r['est_verdict_off']} -> {r['est_verdict_on']} | {r['est_margin']} "
              f"{r['est_margin_band']} | {r['m_words']} / {r['m_fits']} / {r['m_margin']} | - | - |")
    print()
    for f in found:
        print(f"crosscheck {f['workload']:18} {f['metric']:8} {f['verdict']:9} {f['detail']}")


if __name__ == "__main__":
    main()

"""How many words would the isa_cond complements save in the SHIPPED firmware (a mechanical peephole count).

MVP has `jmp pin`, `jmp x!=y`, `jmp !osre` but not their complements; code that needs the complement writes
`jmp <cond> skip` / `jmp target` (2 words, 2 cycles when the complement holds) where `jmpc pin_low|x_eq_y|osre target`
is 1 word, 1 cycle.  This scans every assembled fw/*/program.pio for exactly that shape: a JMP with cond pin / x!=y /
!osre whose target is the word after the next one, the next being an unconditional JMP that nothing else jumps to
and no label names.  Delay and side-set bits on either word are reported (`neutral` = both zero: the rewrite
keeps the timing of the path that did not jump; the complement path still gets one cycle shorter).

    python -m tools.bench.isa_fw.scan
"""
from __future__ import annotations

import sys
from pathlib import Path

from asm.pioasm import assemble

ROOT = Path(__file__).resolve().parents[3]
COMPLEMENT = {0b110: "pin_low", 0b101: "x_eq_y", 0b111: "osre"}


def jmp_fields(w: int):
    return (w >> 13) == 0, (w >> 5) & 7, w & 0x1F, (w >> 8) & 0x1F


def scan_program(prog) -> list[dict]:
    ins = prog.instrs
    targets = {w & 0x1F for w in ins if (w >> 13) == 0}
    labelled = set(prog.labels.values())
    out = []
    for i in range(len(ins) - 1):
        is_j, cond, tgt, dl = jmp_fields(ins[i])
        is_j2, cond2, tgt2, dl2 = jmp_fields(ins[i + 1])
        if not (is_j and is_j2 and cond in COMPLEMENT and cond2 == 0 and tgt == i + 2):
            continue
        if (i + 1) in labelled or (i + 1) in targets:
            continue
        out.append({"at": i, "to": tgt2, "cond": COMPLEMENT[cond], "neutral": dl == 0 and dl2 == 0})
    return out


def scan() -> list[dict]:
    rows = []
    for pio in sorted((ROOT / "fw").glob("*/program.pio")):
        try:
            progs = assemble(pio.read_text())
        except Exception as e:                      # a few programs are templates with overrides
            rows.append({"proto": pio.parent.name, "words": None, "hits": [], "error": str(e)[:80]})
            continue
        for p in progs:
            hits = scan_program(p)
            rows.append({"proto": pio.parent.name, "program": p.name, "words": len(p.instrs), "hits": hits})
    return rows


if __name__ == "__main__":
    rows = scan()
    tot = sum(len(r["hits"]) for r in rows)
    for r in rows:
        print(f"{r['proto']:14} {str(r.get('words')):>4} words  hits {r['hits']}" + (f"  ERR {r['error']}" if r.get("error") else ""))
    print(f"total words saved by mechanical complement rewrite: {tot} in {sum(1 for r in rows if r['hits'])} programs "
          f"of {len(rows)}")
    sys.exit(0)

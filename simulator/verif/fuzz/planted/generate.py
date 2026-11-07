"""Regenerate `verif/fuzz/planted/`: copies of `src/pio_*.sv` with `-D FUZZ_PLANT_RTL` bugs.

The selftest (`make fuzz-selftest`) has to prove that the lockstep harness catches an RTL bug,
without anybody editing `src/`. So the harness builds a *copy* of
the RTL that carries a planted bug, selected by the value of `FUZZ_PLANT_RTL`:

| plant | the bug | where |
|---|---|---|
| 1 (`-DFUZZ_PLANT_RTL`) | `jmp x--` does not write X-1 | `pio_engine.sv`, JMP condition `010` |
| 2 (`-DFUZZ_PLANT_RTL2`) | `pc_update` ignores `WRAP_BOTTOM` | `pio_engine.sv`, the wrap in the bookkeeping |
| 3 (`-DFUZZ_PLANT_RTL3`) | side-set repeats while an instruction is stalled | `pio_engine.sv`, the side-set request |
| 4 (`-DFUZZ_PLANT_RTL4`) | the autopull stall needs `osr_cnt > PULL_THRESH`, not `>=` | `pio_engine.sv`, `OUT` |
| 5 (`-DFUZZ_PLANT_RTL5`) | engine 0 wins a pin conflict instead of engine 1 | `pio_chip.sv`, the resolution loop |
| 6 (`-DFUZZ_PLANT_RTL6`) | `irq0` follows only engine 0, not the OR of both | `pio_chip.sv`, the irq0 flop |
| 7 (`-DFUZZ_PLANT_RTL7`) | the FIFO push stores the word one slot along | `pio_fifo.sv`, the push |

One define per plant, because Verilator's preprocessor has no `if` on an expression (only
`ifdef`/`ifndef`/`else`/`elsif`). `make -C verif/fuzz PLANTED=1 PLANT=2` builds plant 2, `PLANT=12`
builds both.

Each plant is one small change against the spec text, so the diff of the copy against `src/` says
exactly what the plant is. The copies are committed; this script exists so that they cannot drift
away from `src/` unnoticed:

    python -m verif.fuzz.planted.generate          # rewrite the copies
    python -m verif.fuzz.planted.generate --check  # exit 1 if they are stale
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
SRC = ROOT / "src"
FILES = ("pio_chip.sv", "pio_clkdiv.sv", "pio_engine.sv", "pio_fifo.sv")

#: (file, anchor, replacement) triples, applied in order. The anchor must appear exactly once.
PLANTS: list[tuple[str, str, str]] = [
    (
        "pio_engine.sv",
        """            3'b010: begin
              take = (x != 32'd0);
              x_n  = x - 32'd1;
            end""",
        """            3'b010: begin
              take = (x != 32'd0);
`ifdef FUZZ_PLANT_RTL
              // PLANT 1: docs/mvp-isa.md `### Instruction semantics`, JMP: "X-- and Y-- always
              // write X-1 or Y-1 (mod 2^32), whether or not the jump is taken".
              x_n  = x;
`else
              x_n  = x - 32'd1;
`endif
            end""",
    ),
    (
        "pio_engine.sv",
        """        else if (WCW'(pc) == (WCW'({cfg_page, 5'd0}) | WCW'(cfg_wrap_top))) pc_n = PCW'({cfg_page, cfg_wrap_bottom});""",
        """`ifdef FUZZ_PLANT_RTL2
        // PLANT 2: docs/mvp-isa.md `pc_update`, step 3: at WRAP_TOP the next PC is
        // WRAP_BOTTOM.
        else if (WCW'(pc) == (WCW'({cfg_page, 5'd0}) | WCW'(cfg_wrap_top))) pc_n = pc + PCW'(1);
`else
        else if (WCW'(pc) == (WCW'({cfg_page, 5'd0}) | WCW'(cfg_wrap_top))) pc_n = PCW'({cfg_page, cfg_wrap_bottom});
`endif""",
    ),
]

PLANTS += [
    (
        "pio_engine.sv",
        """    if (exec_i && side_go && (src_host || !stalled)) ss_we = 1'b1;""",
        """`ifdef FUZZ_PLANT_RTL3
    // PLANT 3: docs/mvp-isa.md `### Stalls, delay and side-set, summarised`: "Side-set is written
    // once, in the first SM cycle of the instruction, even if it stalls".
    if (exec_i && side_go) ss_we = 1'b1;
`else
    if (exec_i && side_go && (src_host || !stalled)) ss_we = 1'b1;
`endif""",
    ),
    (
        "pio_engine.sv",
        """          if (cfg_autopull && osr_cnt >= pull_th) begin
            stall = 1'b1;""",
        """`ifdef FUZZ_PLANT_RTL4
          // PLANT 4: docs/mvp-isa.md `**OUT**`: autopull stalls when
          // `osr_cnt >= thresh(PULL_THRESH)`, the `>=` included.
          if (cfg_autopull && osr_cnt > pull_th) begin
`else
          if (cfg_autopull && osr_cnt >= pull_th) begin
`endif
            stall = 1'b1;""",
    ),
]

PLANTS += [
    (
        "pio_chip.sv",
        """    for (int e = 0; e < N; e++) begin
      if (os_we[e] && !os_dir[e]) lvl_n = (lvl_n & ~os_mask[e]) | (os_data[e] & os_mask[e]);""",
        """`ifdef FUZZ_PLANT_RTL5
    // PLANT 5: docs/mvp-isa.md `### Pin writes`, rule 2: "Between engines, engine 1 wins over
    // engine 0". The loop order below is what implements it.
    for (int e = N - 1; e >= 0; e--) begin
`else
    for (int e = 0; e < N; e++) begin
`endif
      if (os_we[e] && !os_dir[e]) lvl_n = (lvl_n & ~os_mask[e]) | (os_data[e] & os_mask[e]);""",
    ),
]

PLANTS += [
    (
        "pio_chip.sv",
        """    end else begin
      irq0 <= |ints0;
      irq1 <= |ints1;
    end""",
        """    end else begin
`ifdef FUZZ_PLANT_RTL6
      // PLANT 6: docs/mvp-regmap.md `### tx_ready and rx_valid`: "The OR of IRQ0_INTS drives
      // rx_valid, and the OR of IRQ1_INTS drives tx_ready".
      irq0 <= ints0[0];
`else
      irq0 <= |ints0;
`endif
      irq1 <= |ints1;
    end""",
    ),
]

PLANTS += [
    (
        "pio_fifo.sv",
        """        if (do_push) begin
          mem[wp] <= din;
          wp      <= inc(wp);
        end""",
        """        if (do_push) begin
`ifdef FUZZ_PLANT_RTL7
        // PLANT 7: docs/mvp-isa.md `### Host-to-engine timing` -- a word the
        // host pushes lands at the write pointer and comes out at the read pointer. This stores it
        // one slot along instead: the level is right, and so is everything that leaves the FIFO
        // while the queue does not wrap, because nothing reads the slot it went to. Only a
        // compare of the FIFO storage (lockstep.py --deep, `eng0.tx[i]`) sees it, and
        // `gen.fifofill()` is the run it is caught in.
        mem[inc(wp)] <= din;
`else
        mem[wp] <= din;
`endif
        wp      <= inc(wp);
        end""",
    ),
]

HEADER = """// SPDX-License-Identifier: Apache-2.0
//
// GENERATED by verif/fuzz/planted/generate.py from {src} -- do not edit.
// A copy of src/{name} with `-DFUZZ_PLANT_RTL` bugs in it (see that script). Nothing here is
// ever built into the chip: only `make fuzz-selftest` and `make fuzz-repro PLANTED=1` use it.
"""


def plant(name: str, text: str) -> str:
    out = text
    for file_name, anchor, replacement in PLANTS:
        if file_name != name:
            continue
        count = out.count(anchor)
        if count != 1:
            raise SystemExit(f"plant anchor for {name} found {count} times, expected 1:\n{anchor}")
        out = out.replace(anchor, replacement)
    return out


def generate() -> dict[str, str]:
    files = {}
    for name in FILES:
        source = (SRC / name).read_text()
        body = plant(name, source)
        # keep the original file header comment, under a header of our own
        lines = body.splitlines(keepends=True)
        while lines and not lines[0].startswith("//"):
            lines.pop(0)
        files[name] = HEADER.format(src=f"src/{name}", name=name) + "".join(lines)
    return files


def main(argv: list[str]) -> int:
    check = "--check" in argv
    files = generate()
    stale = []
    for name, text in files.items():
        path = HERE / name
        if check:
            if not path.exists() or path.read_text() != text:
                stale.append(name)
        else:
            path.write_text(text)
            print(f"wrote {path}")
    if check:
        if stale:
            print(f"planted/ is stale against src/: {', '.join(stale)}; "
                  f"run python -m verif.fuzz.planted.generate", file=sys.stderr)
            return 1
        print(f"planted/ matches src/ + {len(PLANTS)} plant(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
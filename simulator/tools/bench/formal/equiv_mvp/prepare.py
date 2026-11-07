#!/usr/bin/env python3
"""Build the inputs of the MVP-equivalence proof into ./build/ (git-ignored).

  build/orig/*.sv   the ORIGINAL (pre-task-H) src/ from revision ORIG_REV, every module renamed orig_<name>
  build/new/*.sv    the current src/ (copied)
  --mutant          one-bit change in the new design (the negative control): pio_chip.sv `pin_oe[0]` (chip level) or
                    tt_um_protocol_emulator.v `uo_out[1]` (top level), chosen by --top

ORIG_REV is the evidence-bench skeleton commit; src/ is byte-identical there and at wb/bench before task H.
"""
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
ORIG_DIR = HERE / "orig"   # frozen reference copy of the pre-knob RTL (ships with the repo)
FILES = ["pio_chip.sv", "pio_engine.sv", "pio_fifo.sv", "pio_clkdiv.sv", "host_spi.sv", "tt_um_protocol_emulator.v"]
MODULES = ["pio_chip", "pio_engine", "pio_fifo", "pio_clkdiv", "host_spi", "tt_um_protocol_emulator"]
# negative controls: a one-bit change in the new design, per proof level
MUTATIONS = {
    "pio_chip.sv": ("assign pin_oe  = {5'b11111, 5'd0, dir};",
                    "assign pin_oe  = {5'b11111, 5'd0, dir[7:1], dir[0] | rx_level[0][0]};"),       # one bit of pin_oe
    "tt_um_protocol_emulator.v": ("assign uo_out          = {pin_out[17:13], irq0, irq1, spi_miso};",
                                  "assign uo_out          = {pin_out[17:13], irq0, ~irq1, spi_miso};"),  # one bit of uo_out
}


def main(argv):
    mutant = "--mutant" in argv
    top = argv[argv.index("--top") + 1] if "--top" in argv else "top"      # which file the mutant goes into
    mfile = "pio_chip.sv" if top == "chip" else "tt_um_protocol_emulator.v"
    b = HERE / "build"
    shutil.rmtree(b, ignore_errors=True)
    (b / "orig").mkdir(parents=True)
    (b / "new").mkdir()
    for f in FILES:
        text = (ORIG_DIR / f).read_text()
        for m in MODULES:
            text = re.sub(rf"\b{m}\b", f"orig_{m}", text)
        (b / "orig" / f).write_text(text)
        new = (ROOT / "src" / f).read_text()
        if mutant and f == mfile:
            assert MUTATIONS[f][0] in new
            new = new.replace(*MUTATIONS[f])
        (b / "new" / f).write_text(new)
    print(f"prepared {b} (original = {ORIG_DIR.relative_to(ROOT)}{', MUTANT' if mutant else ''})")


if __name__ == "__main__":
    main(sys.argv[1:])

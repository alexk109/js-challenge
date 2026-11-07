#!/usr/bin/env python3
"""`make bench-equiv`: the original MVP pio_chip vs the current pio_chip at default parameters.

For the Tiny Tapeout top (host_spi + pio_chip) and for pio_chip alone:
1. equiv.ys.in  all-time sequential equivalence (Yosys equiv_make / equiv_simple / equiv_induct); must PROVE.
2. the same on a one-bit mutant of the new design; must FAIL (negative control).
3. `--bmc` (opt-in, `make bench-equiv EQUIV_BMC=1`): bmc.sby, a SymbiYosys miter of pio_chip from reset. It only reaches
   a few cycles of its depth-40 budget (README), so it is not part of the default run.

Exit status 1 unless 1 passes and 2 fails.
"""
import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LL = HERE.parents[2] / "ll"


FILES = {"chip": ["pio_clkdiv.sv", "pio_fifo.sv", "pio_engine.sv", "pio_chip.sv"],
         "top": ["pio_clkdiv.sv", "pio_fifo.sv", "pio_engine.sv", "pio_chip.sv", "host_spi.sv", "tt_um_protocol_emulator.v"]}
TOPS = {"chip": "pio_chip", "top": "tt_um_protocol_emulator"}


def equiv(level: str, tag: str) -> tuple[int, str]:
    """Prove build/orig against build/new at `level` (chip = pio_chip, top = the Tiny Tapeout top with host_spi)."""
    ys = (HERE / "equiv.ys.in").read_text().replace("@TOP@", TOPS[level])
    ys = ys.replace("@ORIG@", " ".join(f"build/orig/{f}" for f in FILES[level]))
    ys = ys.replace("@NEW@", " ".join(f"build/new/{f}" for f in FILES[level]))
    script = HERE / "build" / f"equiv_{level}_{tag}.ys"
    script.write_text(ys)
    log = HERE / "build" / f"equiv_{level}_{tag}.log"
    p = subprocess.run([str(LL), "yosys", "-q", "-l", str(log), str(script)], cwd=HERE, capture_output=True, text=True)
    text = log.read_text() if log.exists() else p.stdout + p.stderr
    m = re.search(r"Found (\d+) \$equiv cells in equiv:\s+Of those cells (\d+) are proven and (\d+) are unproven", text)
    return p.returncode, (f"{m.group(2)} of {m.group(1)} equivalence points proved, {m.group(3)} unproven" if m else "no result")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bmc", action="store_true", help="also run the SymbiYosys BMC miter (opt-in: it only reaches depth ~4)")
    ap.add_argument("--bmc-seconds", type=int, default=120)
    a = ap.parse_args()
    ok = True
    prep = [sys.executable, str(HERE / "prepare.py")]
    for level, what in (("top", "tt_um_protocol_emulator (host_spi + pio_chip)"), ("chip", "pio_chip")):
        subprocess.run(prep, check=True)
        rc, msg = equiv(level, "new")
        print(f"[{level}] original vs current {what}, all-time (equiv_induct): {'PROVED' if rc == 0 else 'FAILED'}  ({msg})")
        ok &= rc == 0
        subprocess.run(prep + ["--mutant", "--top", level], check=True)
        rc_m, msg_m = equiv(level, "mutant")
        bit = "uo_out[1]" if level == "top" else "pin_oe[0]"
        print(f"[{level}] negative control (one bit of {bit} changed): {'REJECTED' if rc_m != 0 else 'NOT REJECTED (bad)'}  ({msg_m})")
        ok &= rc_m != 0
    subprocess.run(prep, check=True)
    if a.bmc:
        work = HERE / "bmc"
        shutil.rmtree(work, ignore_errors=True)
        sby = HERE / "bmc.sby"
        p = subprocess.run(["timeout", str(a.bmc_seconds), "sby", "-f", "--yosys", f"{LL} yosys", str(sby)],
                           cwd=HERE, capture_output=True, text=True)
        log = (work / "logfile.txt").read_text() if (work / "logfile.txt").exists() else p.stdout
        depths = [int(x) for x in re.findall(r"engine_0: (\d+) [+-] :", log)]
        verdict = "no counterexample" if "failed" not in log.lower().split("summary")[-1] else "COUNTEREXAMPLE"
        done = "completed" if p.returncode == 0 else f"stopped by the {a.bmc_seconds} s limit"
        print(f"[bmc] BMC miter from reset, free inputs: {verdict} up to depth {max(depths) if depths else 0} of 40 ({done})")
        ok &= verdict == "no counterexample"
    print("bench-equiv:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

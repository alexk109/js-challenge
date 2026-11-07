"""Measure how often the cross-engine pin rule is *reached* by the plain generator.

Why this exists. docs/mvp-isa.md `### Pin writes`, rule 2 ("between engines, engine 1 wins over
engine 0") is only observable in a cycle where both engines write the same pin with different
values in the same cycle. That is not a given: it needs their clock dividers to line up, and
`gen.gen()` picks those from the seed. So the seeds where it never happens cannot see plant 5 at
all -- not because the comparison misses it, but because the golden RTL and the real RTL behave
identically there. `gen.gen(..., conflict=N)` adds a phase that forces the situation, which is
what `metrics/fuzz-deep.csv` is measured with; this script is what says how much the forcing is
needed, i.e. what the plain generator would and would not have covered.

The measurement is on the ISS alone (no simulator): the question is which requests the stimulus
causes, and both the golden RTL and the real RTL make the same ones. Same method as
`verif/fuzz/test_deep.py::test_conflict_phase_makes_the_two_engines_contest_one_pin`.

    python -m verif.fuzz.conflict_measure [--seeds 1-20] [--cycles 10000] [--conflict 0]

Exit 0 always: this is a measurement, not a test. The verdict line says how many seeds ever
contest a pin, and which seeds never do.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from iss.pio_iss import Chip                                        # noqa: E402
from verif.fuzz import gen, lockstep                                # noqa: E402


def contested_pin_cycles(stim: gen.Stimulus) -> int:
    """Cycles in which the two engines asked for the same pin register with different values."""
    chip = Chip()
    ops = {c: (k, a, d) for c, k, a, d in stim.ops}
    count = 0
    for cycle in range(stim.cycles):
        pin_in = lockstep.pad_value(gen.pin_vector(stim.seed, cycle) & ~stim.ext_low,
                                    chip.pin_out, chip.pin_oe)
        bus = ops.get(cycle)
        running = [bool(e.force or ((chip.sm_enable >> i) & 1 and e.en))
                   for i, e in enumerate(chip.engines)]
        chip.step(pin_in, None if bus is None else
                  (("w", bus[1], bus[2]) if bus[0] == "w" else ("r", bus[1])))
        if not all(running):
            continue
        reqs = [[chip.last_req[e]["out"], chip.last_req[e]["side"]] for e in range(2)]
        for i in range(2):
            for j in range(2):
                a, b = reqs[0][i], reqs[1][j]
                if a and b and a[0] == b[0]:                 # same register: level or direction
                    mask = a[1] & b[1] & 0x3FFFF
                    if mask and (a[2] ^ b[2]) & mask:
                        count += 1
    return count


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="how often do the two engines contest one pin?")
    ap.add_argument("--seeds", default="1-20")
    ap.add_argument("--cycles", type=int, default=10000)
    ap.add_argument("--conflict", type=int, default=0,
                    help="the guaranteed conflict phase gen.gen(conflict=N) adds (0: none)")
    args = ap.parse_args(argv)

    seeds: list[int] = []
    for part in args.seeds.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = part.split("-")
            seeds.extend(range(int(lo), int(hi) + 1))
        elif part:
            seeds.append(int(part))

    print(f"conflict_measure: gen.gen(seed, {args.cycles}, conflict={args.conflict}), ISS only, "
          f"git {lockstep.git_stamp()}, {time.strftime('%Y-%m-%d %H:%M:%S')}")
    counts = []
    for seed in seeds:
        t0 = time.time()
        count = contested_pin_cycles(gen.gen(seed, args.cycles, conflict=args.conflict))
        counts.append((seed, count))
        print(f"  seed {seed:>4}: {count:>6} contested pin-cycles "
              f"({'ever' if count else 'NEVER'}), {time.time() - t0:.1f}s", flush=True)
    ever = [s for s, c in counts if c]
    never = [s for s, c in counts if not c]
    print("")
    print(f"seeds that ever contest a pin: {len(ever)} of {len(counts)}")
    print(f"seeds that never do: {never if never else '(none)'}")
    print(f"contested pin-cycles per seed: min {min(c for _, c in counts)}, "
          f"median {sorted(c for _, c in counts)[len(counts) // 2]}, max {max(c for _, c in counts)}")
    print("")
    print(f"conflict_measure: {len(never)} of {len(counts)} seeds cannot see plant 5 without "
          f"conflict={args.conflict or 'N'}, so every campaign that claims a per-seed sensitivity "
          f"number has to use the forced conflict phase")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
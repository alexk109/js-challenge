"""Measure that `--pullup` is not vacuous.

Why this exists. `pad_value()` *forces* the bit of every released `uio` pin under `--pullup`, and
`Coverage.note()` then counts the bit of that same `pin_in`, so the `uio_released_high /
uio_released` ratio in the coverage is 100 % by construction -- it is 100 % for any run at all,
including one in which the level never reached the chip. A number that
cannot fail is not evidence.

What is measured here instead. The same stimulus (`gen.pinloop()`) is run twice on the same
seed, once with `--pullup` and once without, both with `--deep` and with `FUZZ_PAD_TRACE`, and the
two runs' per-cycle `in_vec` (the value the engines read, `T_IN` stages back the pad) and
`in_pipe0` are compared cycle by cycle. If `--pullup` did nothing to what the chip sees, the two
traces would be identical and the count below would be 0 -- so this number *can* fail, and the
run also prints the count of released pin-cycles it is a fraction of.

    python -m verif.fuzz.pullup_measure [--seed 1] [--cycles 300] [--sim verilator]

Exit 0 only if both runs matched and the traces differ; that is the claim "`--pullup` changes what
the chip sees, in this many cycles".
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verif.fuzz import lockstep                                     # noqa: E402

#: how many stages back `in_vec` is the pad of this cycle (src/pio_chip.sv `parameter T_IN`)
T_IN = 3


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="is --pullup non-vacuous?")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--cycles", type=int, default=300)
    ap.add_argument("--sim", default="verilator", choices=["verilator", "icarus"])
    ap.add_argument("--jobs", default="-j2")
    args = ap.parse_args(argv)

    stamp = lockstep.git_stamp()
    print(f"pullup_measure: stimulus gen.pinloop(), seed {args.seed}, {args.cycles} cycles, "
          f"{args.sim}, deep True, T_IN {T_IN}, git {stamp}, {time.strftime('%Y-%m-%d %H:%M:%S')}")

    runs = {}
    for pullup in (False, True):
        result = lockstep.run(seed=args.seed, cycles=args.cycles, sim=args.sim, pinloop=True,
                              deep=True, pullup=pullup, quiet=True, jobs=args.jobs,
                              repro_dir=Path("/tmp/fuzz-repros/pullup-measure"),
                              extra_env={"FUZZ_PAD_TRACE": "1"})
        runs[pullup] = result
        cov = result["coverage"]
        print(f"  {'with' if pullup else 'without'} --pullup: {result['status']}, "
              f"{result['cycles_run']} cycles over {result['fields']} fields, "
              f"released uio pin-cycles {cov['uio_released_pin_cycles']}, of which high "
              f"{cov['uio_released_high_pin_cycles']}, pad pattern {cov['pad_pattern']}")

    plain, pulled = runs[False], runs[True]
    if plain["status"] != "pass" or pulled["status"] != "pass":
        print("pullup_measure: FAIL, one of the two runs did not match, so there is nothing to "
              "compare")
        return 1

    in_vec_diff = [(c, a, b) for c, (a, b) in
                   enumerate(zip(plain["in_vec_trace"], pulled["in_vec_trace"])) if a != b]
    pad_diff = [(c, a, b) for c, (a, b) in
                enumerate(zip(plain["pad_trace"], pulled["pad_trace"])) if a != b]
    pad_bits = sum((a ^ b).bit_count() for _, a, b in pad_diff)
    cycles = min(len(plain["in_vec_trace"]), len(pulled["in_vec_trace"]))
    released = pulled["coverage"]["uio_released_pin_cycles"]
    print("")
    print(f"  cycles compared: {cycles}")
    print(f"  cycles where the driven pad differs between the two modes: {len(pad_diff)} "
          f"({pad_bits} pin-bits)")
    print(f"  cycles where the compared in_vec differs: {len(in_vec_diff)}")
    print(f"  released uio pin-cycles in the --pullup run: {released}")
    for cycle, a, b in in_vec_diff[:5]:
        src = cycle - T_IN
        print(f"    cycle {cycle:>4}: without --pullup in_vec {a:#07x}, with {b:#07x}"
              + (f"; the pad of cycle {src} (T_IN={T_IN}) is without "
                 f"{plain['pad_trace'][src]:#07x}, with {pulled['pad_trace'][src]:#07x}"
                 if src >= 0 else ""))
    # the consistency check the pad model could fail: a difference in what the engines read has to
    # come from a difference in what the pad carried T_IN cycles earlier, never the other way round
    unexplained = [c for c, _, _ in in_vec_diff if c - T_IN < 0 or not pad_diff
                   or plain["pad_trace"][c - T_IN] == pulled["pad_trace"][c - T_IN]]
    print("")
    print(f"  in_vec differences with no pad difference {T_IN} cycles earlier: {len(unexplained)}")
    ok = bool(in_vec_diff) and not unexplained
    # Two wordings, because the conclusion of this measurement is only available in the passing
    # case: a FAIL log that still says the open-drain path "is really exercised" states the
    # opposite of what the run found, and a FAIL here is a committed log someone reads later.
    if ok:
        print(f"pullup_measure: PASS: --pullup changes the compared in_vec in {len(in_vec_diff)} "
              f"of {cycles} cycles of the same stimulus, every one of them T_IN={T_IN} cycles "
              f"after the pad itself differed, so the open-drain path the mode exists for is "
              f"really exercised")
    else:
        print(f"pullup_measure: FAIL: --pullup changes the compared in_vec in {len(in_vec_diff)} "
              f"of {cycles} cycles of the same stimulus, {len(unexplained)} of them with no pad "
              f"difference {T_IN} cycles earlier; this run does not show the open-drain path the "
              f"mode exists for being exercised"
              + (" (no cycle's compared in_vec differs at all, so the mode is vacuous here)"
                 if not in_vec_diff else ""))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
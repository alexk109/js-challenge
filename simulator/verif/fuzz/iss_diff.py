"""Differential check: golden ISS (iss/) against the independent ISS (verif/iss_b/).

Same stimulus as the lockstep harness (verif.fuzz.gen + lockstep.pad_value), every cycle, every
compared field.
The two were written by different authors blind to each other, so a
mismatch means at least one of them misread docs/mvp-isa.md.

    .venv/bin/python -m verif.fuzz.iss_diff [first_seed] [n_seeds] [cycles]
Exits 1 on the first mismatch per seed reported; prints one line per seed.
"""
import sys

from iss.pio_iss import Chip as ChipA
from verif.fuzz import gen
from verif.fuzz.lockstep import OUTPUT_KEYS, STATE_KEYS, pad_value
from verif.iss_b.pio_iss_b import Chip as ChipB


MISSING: set = set()


def run(seed: int, cycles: int):
    stim = gen.gen(seed, cycles)
    a, b = ChipA(), ChipB()
    ops = {c: (k, addr, d) for c, k, addr, d in stim.ops}
    for c in range(stim.cycles):
        # Same pad model as the lockstep harness (lockstep.py), so both checks see one stimulus.
        pin = pad_value(gen.pin_vector(seed, c), a.pin_out, a.pin_oe)
        op = ops.get(c)
        bus = None if op is None else (("r", op[1]) if op[0] == "r" else ("w", op[1], op[2]))
        ra, rb = a.step(pin, bus), b.step(pin, bus)
        if op is not None and op[0] == "r" and ra != rb:
            return c, f"bus_rdata@{op[1]:#04x}", ra, rb
        for k in OUTPUT_KEYS:
            if getattr(a, k) != getattr(b, k):
                return c, k, getattr(a, k), getattr(b, k)
        for e in range(gen.NUM_ENGINES):
            sa, sb = a.state(e), b.state(e)
            for k in STATE_KEYS:
                if k not in sb:          # keys added to the interface after iss_b was written
                    MISSING.add(k)
                    continue
                if sa.get(k) != sb.get(k):
                    return c, f"eng{e}.{k}", sa.get(k), sb.get(k)
    return None


def main(argv):
    first = int(argv[1]) if len(argv) > 1 else 1
    n = int(argv[2]) if len(argv) > 2 else 20
    cycles = int(argv[3]) if len(argv) > 3 else 10000
    bad = 0
    for seed in range(first, first + n):
        r = run(seed, cycles)
        if r is None:
            print(f"seed {seed}: {cycles} cycles match")
        else:
            bad += 1
            c, field, va, vb = r
            print(f"seed {seed}: MISMATCH at cycle {c}, {field}: golden={va!r} iss_b={vb!r}")
    print(f"{n - bad}/{n} seeds match" + (f"; iss_b lacks state keys {sorted(MISSING)} (not compared)" if MISSING else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

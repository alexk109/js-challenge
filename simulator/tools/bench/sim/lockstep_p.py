"""Lockstep of the parametric ISS (`pio_iss_p`) against the golden ISS, at the MVP parameters.

Same stimulus as `verif/fuzz/iss_diff.py` (`verif.fuzz.gen` + the lockstep pad model). Compared every
cycle: bus read data, the four outputs, `state(e)` for both engines, `deep_state()` (FIFO contents,
IMEM, divider, pin requests) and a `peek()` of every register address. The golden ISS is never edited;
a planted divergence (`plant=`) edits the *iss_p* class only, as the negative control.

    .venv/bin/python -m tools.bench.sim.lockstep_p [first_seed] [n_seeds] [cycles] [--plant NAME]
"""
from __future__ import annotations

import sys
from typing import Callable, Optional

from iss.pio_iss import Chip as Golden
from tools.bench.sim import pio_iss_p
from verif.fuzz import gen
from verif.fuzz.lockstep import OUTPUT_KEYS, STATE_KEYS, pad_value


def _plant_fifo(chip_cls: type) -> None:
    """iss_p bug: the TX FIFO holds one word less than it should (depth - 1)."""
    orig = chip_cls.step

    def step(self, pin_in, bus=None):
        if bus and bus[0] == "w" and bus[1] in (0x04, 0x05) and len(self.engines[bus[1] - 4].tx) == self.P.fifo_depth - 1:
            bus = None
        return orig(self, pin_in, bus)
    chip_cls.step = step


def _plant_sync(chip_cls: type) -> None:
    """iss_p bug: the input synchronizer is one stage short."""
    orig = chip_cls.reset

    def reset(self):
        orig(self)
        self.sync = self.sync[:-1] or [0]
    chip_cls.reset = reset


def _plant_wrap(chip_cls: type) -> None:
    """iss_p bug: PC wraps at 16 words instead of 32. Only `chip_cls` is affected: its reset
    builds engines through a subclass, so `pio_iss_p.Engine` itself is never touched."""
    class BadEngine(pio_iss_p.Engine):
        def __init__(self, idx, P=pio_iss_p.MVP_PARAMS):
            super().__init__(idx, P)
            self.pmask = 15
    orig = chip_cls.reset

    def reset(self):
        orig(self)
        self.engines = [BadEngine(e, self.P) for e in range(self.P.n_engines)]
    chip_cls.reset = reset


PLANTS: dict[str, Callable[[type], None]] = {"fifo": _plant_fifo, "sync": _plant_sync, "wrap": _plant_wrap}


def run(seed: int, cycles: int, chip_cls: Optional[type] = None):
    """First mismatch `(cycle, field, golden, iss_p)` or None."""
    chip_cls = chip_cls or pio_iss_p.Chip
    stim = gen.gen(seed, cycles)
    a, b = Golden(), chip_cls()
    ops = {c: (k, addr, d) for c, k, addr, d in stim.ops}
    for c in range(stim.cycles):
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
                if sa.get(k) != sb.get(k):
                    return c, f"eng{e}.{k}", sa.get(k), sb.get(k)
        da, db = a.deep_state(), b.deep_state()
        if da != db:
            for k in da:
                if da[k] != db.get(k):
                    return c, f"deep.{k}", da[k], db.get(k)
        if c % 16 == 0:                     # every register address, a sample of cycles
            for addr in range(128):
                if a.peek(addr) != b.peek(addr):
                    return c, f"peek@{addr:#04x}", a.peek(addr), b.peek(addr)
    return None


def main(argv: list[str]) -> int:
    plant = None
    if "--plant" in argv:
        i = argv.index("--plant")
        plant = argv[i + 1]
        del argv[i:i + 2]
    first = int(argv[1]) if len(argv) > 1 else 1
    n = int(argv[2]) if len(argv) > 2 else 5
    cycles = int(argv[3]) if len(argv) > 3 else 5000
    cls = pio_iss_p.Chip
    if plant:
        cls = type("Planted", (pio_iss_p.Chip,), {})
        PLANTS[plant](cls)
    bad = 0
    for seed in range(first, first + n):
        r = run(seed, cycles, cls)
        if r is None:
            print(f"seed {seed}: {cycles} cycles match")
        else:
            bad += 1
            c, field, va, vb = r
            print(f"seed {seed}: MISMATCH at cycle {c}, {field}: golden={va!r} iss_p={vb!r}")
    print(f"{n - bad}/{n} seeds match" + (f" (planted bug: {plant})" if plant else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

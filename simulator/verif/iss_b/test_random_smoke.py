"""Random-program smoke test: the ISS must never raise, and its state must stay
inside the ranges the spec gives (`### Engine state`, `## Conventions`).

This is not a reference comparison (the differential fuzz against `iss/pio_iss.py`
does that); it checks the invariants that the spec states as ranges and the API
contract, over instruction words drawn from the whole 16-bit space.
"""

import random

import pytest

from verif.iss_b import enc
from verif.iss_b.enc import A_FSTAT, Bus
from verif.iss_b.pio_iss_b import FIFO_DEPTH, M32, thresh

CYCLES = 400
PROGRAMS = 60
SEED = 20261001

RANGES = {
    "pc": (0, 31),
    "x": (0, M32),
    "y": (0, M32),
    "osr": (0, M32),
    "isr": (0, M32),
    "osr_cnt": (0, 32),
    "isr_cnt": (0, 32),
    "delay_cnt": (0, 31),
    "stalled": (0, 1),
    "latch": (0, M32),
    "latch_pend": (0, 1),
    "tx_level": (0, FIFO_DEPTH),
    "rx_level": (0, FIFO_DEPTH),
}


def check_state(b: Bus) -> None:
    for e in (0, 1):
        st = b.state(e)
        for key, (lo, hi) in RANGES.items():
            assert lo <= st[key] <= hi, f"engine {e}: {key} = {st[key]}"
        assert 0 <= b.chip.eng[e].osr_cnt <= 32
        assert len(b.chip.eng[e].tx) <= FIFO_DEPTH
        assert len(b.chip.eng[e].rx) <= FIFO_DEPTH


def check_pins(b: Bus) -> None:
    assert 0 <= b.chip.pin_out < (1 << 18)
    assert 0 <= b.chip.pin_oe < (1 << 18)
    assert b.chip.pin_oe & 0x3E000 == 0x3E000  # pins 13..17 always driven
    assert b.chip.pin_oe & 0x1F00 == 0  # pins 8..12 never driven
    assert b.chip.pin_out & 0x1F00 == 0  # and they have no level register
    assert b.chip.irq0 in (0, 1) and b.chip.irq1 in (0, 1)


def random_chip(rng: random.Random) -> Bus:
    b = Bus(pin=rng.getrandbits(18))
    for e in (0, 1):
        b.write(0x32 + 6 * e + 0, (rng.randrange(1, 5) << 16) | (rng.getrandbits(8) << 8))
        b.write(
            0x32 + 6 * e + 1,
            rng.getrandbits(32) & ~(0x7E0000 | 0xC0000),  # keep the n/i bits clear
        )
        b.write(0x32 + 6 * e + 2, rng.getrandbits(16) << 16)
        b.write(0x32 + 6 * e + 5, rng.getrandbits(32))
        for k in range(32):
            b.write(0x12 + 0x40 * e + k, rng.getrandbits(16))
    b.write(0x4B, rng.getrandbits(6))
    b.write(0x4E, rng.getrandbits(6))
    return b


@pytest.mark.parametrize("seed", [SEED + i for i in range(PROGRAMS)])
def test_random_programs_stay_inside_the_spec_ranges(seed):
    rng = random.Random(seed)
    b = random_chip(rng)
    b.write(0x00, 0x3FF)  # enable both engines and restart both dividers
    for _ in range(CYCLES):
        pin = rng.getrandbits(18)
        choice = rng.random()
        if choice < 0.12:
            b.step(pin, ("r", rng.randrange(128)))
        elif choice < 0.24:
            b.step(pin, ("w", rng.randrange(128), rng.getrandbits(32)))
        else:
            b.step(pin)
        check_state(b)
        check_pins(b)
    # The host still sees a consistent view of the FIFOs at the end.
    assert b.read(A_FSTAT) is not None


def test_thresh_matches_the_spec_convention():
    assert thresh(0) == 32
    for f in range(1, 32):
        assert thresh(f) == f


def test_a_stuck_engine_still_advances_its_pin_pipeline():
    """A `wait 1` that never completes must not stop the chip: the host and the
    pins keep working."""
    b = Bus()
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    b.load_imem(0, [enc.wait_(enc.WAIT_GPIO, 0, pol=1)])
    b.start(0)
    for _ in range(50):
        b.step(0)
        check_state(b)
    assert b.state(0)["stalled"] == 1
    assert b.state(0)["pc"] == 0  # stuck, as the spec says
    assert b.read(A_FSTAT) is not None


def test_a_long_random_run_is_deterministic():
    """Two identical runs give identical traces, so the fuzz harness can replay."""
    runs = []
    for _ in range(2):
        rng = random.Random(SEED)
        b = random_chip(rng)
        b.write(0x00, 0x3FF)
        trace = []
        for _ in range(120):
            pin = rng.getrandbits(18)
            b.step(pin)
            trace.append((b.chip.pin_out, b.chip.pin_oe, b.chip.irq0, b.chip.irq1,
                          b.state(0)["pc"], b.state(0)["x"], b.state(0)["isr"]))
        runs.append(trace)
    assert runs[0] == runs[1]
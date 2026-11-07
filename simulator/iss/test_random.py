"""Random programs, configurations and host traffic: every 16-bit word is legal, so the ISS
must never raise, and its state must stay within the ranges the spec gives."""
import random

import pytest

from iss.pio_iss import Chip
from iss.testkit import CTRL, FDEBUG, RXF, TXF, imem_addr, sm

KEYS = ["pc", "ir", "x", "y", "osr", "isr", "osr_cnt", "isr_cnt", "delay_cnt", "stalled",
        "latch", "latch_pend", "latch_host", "forced", "tx_level", "rx_level"]


def _host_op(rng):
    e = rng.randrange(2)
    k = rng.random()
    if k < 0.15:
        return ("w", TXF[e], rng.getrandbits(32))
    if k < 0.30:
        return ("r", RXF[e])
    if k < 0.36:
        return ("w", sm(e, "INSTR"), rng.getrandbits(16))
    if k < 0.40:
        return ("w", imem_addr(e, rng.randrange(32)), rng.getrandbits(16))
    if k < 0.43:
        return ("w", sm(e, rng.choice(["CLKDIV", "EXECCTRL", "SHIFTCTRL", "PINCTRL"])),
                rng.getrandbits(32) & (0x0003_FF00 if rng.random() < 0.5 else 0xFFFF_FFFF))
    if k < 0.45:
        return ("w", CTRL, rng.getrandbits(10))
    if k < 0.47:
        return ("w", FDEBUG, rng.getrandbits(32))
    if k < 0.55:
        return ("r", rng.randrange(128))
    return None


@pytest.mark.parametrize("seed", range(12))
def test_random_traffic_keeps_invariants(seed):
    rng = random.Random(seed)
    c = Chip()
    for e in range(2):
        for k in range(32):
            c.step(0, ("w", imem_addr(e, k), rng.getrandbits(16)))
        c.step(0, ("w", sm(e, "CLKDIV"), (rng.choice([1, 1, 2, 3]) << 16) | (rng.getrandbits(8) << 8)))
        c.step(0, ("w", sm(e, "SHIFTCTRL"), rng.getrandbits(32)))
        c.step(0, ("w", sm(e, "PINCTRL"), rng.getrandbits(32)))
        c.step(0, ("w", sm(e, "EXECCTRL"), rng.getrandbits(32)))
    c.step(0, ("w", CTRL, 0x333))
    ext = rng.getrandbits(18)
    for _ in range(3000):
        if rng.random() < 0.05:
            ext = rng.getrandbits(18)
        r = c.step(ext, _host_op(rng))
        assert r is None or 0 <= r <= 0xFFFF_FFFF
        assert 0 <= c.pin_out < 1 << 18 and c.pin_out & 0x01F00 == 0
        assert c.pin_oe & 0x3E000 == 0x3E000 and c.pin_oe & 0x01F00 == 0
        assert c.irq0 in (0, 1) and c.irq1 in (0, 1)
        for e in range(2):
            s = c.state(e)
            assert list(s) == KEYS
            assert 0 <= s["pc"] < 32 and 0 <= s["ir"] < 1 << 16 and 0 <= s["latch"] < 1 << 16
            assert 0 <= s["osr_cnt"] <= 32 and 0 <= s["isr_cnt"] <= 32
            assert 0 <= s["delay_cnt"] < 32
            assert 0 <= s["tx_level"] <= 4 and 0 <= s["rx_level"] <= 4
            for k in ("x", "y", "osr", "isr"):
                assert 0 <= s[k] <= 0xFFFF_FFFF
            assert s["forced"] in (0, 1) and s["stalled"] in (0, 1)

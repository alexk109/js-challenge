"""A planted bug in the ISS, as a monkeypatch, for `make fuzz-selftest` check (c).

Selftest check (c): the selftest has to show that the lockstep harness notices when
the *ISS* is wrong, not only when the RTL is. It cannot touch `iss/` (that is the golden model and
read-only for the fuzz harness), so this module monkeypatches the model in the simulator process instead.

It is not a cocotb test: it has no test in it. It is loaded as a second test module
(`COCOTB_TEST_MODULES=lockstep,plant_iss`), which cocotb imports before it runs any test, so the
patch is in place by the time `lockstep.lockstep` starts.

The bug: `pc_update` (docs/mvp-isa.md) step 3 says that at `WRAP_TOP` the next PC is
`WRAP_BOTTOM`. This patch makes the ISS count up to `WRAP_TOP + 1` instead -- the same bug as
plant 2 of `verif/fuzz/planted/`, on the other side of the comparison.
"""
from __future__ import annotations

from iss.pio_iss import Engine

_original_sm_step = Engine.sm_step


def _planted_sm_step(self: Engine, in_vec: int, fd: dict) -> dict:
    """Engine.sm_step with the wrap to WRAP_BOTTOM removed."""
    pc0 = self.pc
    req = _original_sm_step(self, in_vec, fd)
    if pc0 == self.wrap_top and self.pc == self.wrap_bottom:
        self.pc = (pc0 + 1) & 0x1F
    return req


Engine.sm_step = _planted_sm_step

print("plant_iss: Engine.sm_step patched -- the ISS no longer wraps to WRAP_BOTTOM "
      "(docs/mvp-isa.md `pc_update` step 3)", flush=True)
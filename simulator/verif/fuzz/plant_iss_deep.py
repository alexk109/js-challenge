"""Six planted bugs in the *ISS*, one per field `--deep` adds, for `make fuzz-selftest`.

Every new check has to be shown to bite, and the
FIFO-contents check in particular where the 37 shallow signals cannot. The
cheapest way to do that is the way `plant_iss.py` already does it for the shallow set: break the
*model* in the simulator process, leave `src/` and `iss/` alone, and require the lockstep to notice
in the field the break belongs to.

Loaded as a second cocotb test module (`COCOTB_TEST_MODULES=lockstep,plant_iss_deep`), which cocotb
imports before it runs any test, so the patch is in place before `lockstep.lockstep` starts.
`FUZZ_DEEP_PLANT` picks which one; with nothing set, nothing is patched.

| `FUZZ_DEEP_PLANT` | the bug in the ISS | the field that must catch it |
|---|---|---|
| `fifo` | a word in the TX FIFO storage comes out wrong | `eng0.tx[1]` |
| `config` | PINCTRL.IN_BASE is stored one pin off | `eng0.pinctrl` |
| `div` | the fractional divider drops the carry out of `acc + FRAC` | `eng0.div_acc` |
| `imem` | a host IMEM write is dropped | `eng0.imem[17]` |
| `fdebug` | an FDEBUG flag is set that nothing set | `fdebug` |
| `pinreq` | an OUT request claims the wrong direction | `eng0.os_dir` |

Every one of them is a bug whose *only* symptom is the deep field: `config` touches a PINCTRL bit
the program never uses, `pinreq` a direction the program never reads back before the pin resolution
is compared, `fifo` a word nothing pops, `imem` a word nothing fetches. `verif/fuzz/gen.py
fifofill()` is the run they are checked on, because it reaches all six and never drains anything.
"""
from __future__ import annotations

import os

import iss.pio_iss as pio

PLANTS = ("fifo", "config", "div", "imem", "fdebug", "pinreq")
#: the IMEM word the `imem` plant drops: the fifofill program fetches words 0 and 1 only
IMEM_WORD = 17
#: the TX FIFO word the `fifo` plant corrupts, and the value it becomes
FIFO_WORD = 1
FIFO_VALUE = 0x0BAD_F00D


def _fifo(chip) -> None:
    """A word in engine 0's TX FIFO storage is not the word the host pushed."""
    tx = chip.engines[0].tx
    if len(tx) > FIFO_WORD:
        tx[FIFO_WORD] = FIFO_VALUE


def _config(chip) -> None:
    """PINCTRL.IN_BASE is one pin off; the fifofill program never shifts the pins, so nothing else
    in the chip can notice."""
    chip.engines[0].pinctrl ^= 1 << 15


def _div(chip) -> None:
    """The 16.8 fractional divider's accumulator loses its low bit on every reload, so `acc` never
    reaches `thresh(FRAC)` while `div_cnt` still counts the carry out of the untruncated sum."""
    eng = chip.engines[0]
    if eng.en and ((eng.clkdiv >> 8) & 0xFF):        # en == 1: this cycle reloaded
        eng.div_acc &= 0xFE


def _imem(chip) -> None:
    """A host IMEM write is dropped, in a word the program never fetches."""
    chip.engines[0].imem[IMEM_WORD] = 0xFFFF


def _fdebug(chip) -> None:
    """An FDEBUG flag is set that nothing in the run set."""
    chip.fdebug |= 1 << 16                      # TXOVER on engine 0


def _pinreq(chip) -> None:
    """An OUT request claims the wrong direction; the pin resolution that follows is wrong with
    it, and the request is the field that says so first."""
    for req in chip.last_req:
        if req["out"] is not None:
            is_dir, mask, val = req["out"]
            req["out"] = (is_dir ^ 1, mask, val)


#: which of them runs, and the sentence the simulator prints so the log says which plant it was
PATCHES = {"fifo": _fifo, "config": _config, "div": _div, "imem": _imem,
           "fdebug": _fdebug, "pinreq": _pinreq}

_original_step = pio.Chip.step


def _patched_step(self, pin_in, bus=None):
    r = _original_step(self, pin_in, bus)
    PATCHES[os.environ["FUZZ_DEEP_PLANT"]](self)
    return r


pio.Chip.step = _patched_step
print(f"plant_iss_deep: Chip.step patched -- FUZZ_DEEP_PLANT={os.environ['FUZZ_DEEP_PLANT']}",
      flush=True)

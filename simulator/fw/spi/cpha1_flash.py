"""`verif/vip/spi.py`'s flash target with its mode restriction lifted, for CPHA 1.

`SpiFlashTarget` refuses modes 1 and 2: "SpiFlashTarget supports SPI modes 0 and 3 (CPHA 0)".  That
is a true statement about a serial flash -- CPHA 0 is what flash supports -- and it is also the
only thing standing between the chip's mode 1 and mode 2 programs and the same judge the other two
modes use.  Everything else in that model is mode-generic: `_capture` and `_launch` both ask
`verif/vip/spi.py`'s own `is_capture_edge(mode, ...)` and `first_output_index(mode)`, which handle
all four modes.

So this module does the smallest thing that makes the model's CPHA-1 half reachable, and nothing
else:

*   it constructs the parent with a mode of the same **CPOL**, which the parent accepts, so every
    field the parent sets is set by the parent;
*   it then sets `mode` to the real one and calls `_reset_session()`, the one method whose result
    depends on the mode (`_out_idx = first_output_index(self.mode)`, which is 0 for CPHA 0 and -1
    for CPHA 1: a CPHA 1 target has to put a dummy bit out first).

No state is invented here and no method is overridden, so a CPHA 1 flash is the shipped model with
its CPOL chosen and its phase switched.  `fw/spi/test_spi_cpha1.py::test_the_subclass_adds_no_state`
is the guard: it builds a parent at mode 0 and this class at mode 2 -- the same CPOL, so the same
everything else -- and asserts the two objects' attributes differ in exactly `mode` and `_out_idx`,
and that `_reset_session()` is the only thing that was re-run.

The verdict this buys, and its limit: a mode 1 or mode 2 transaction is judged by a flash that only
answers an opcode it recognises and only after three address bytes have arrived, so a wrong phase
shows up as no answer at all.  What is *not* claimed: that the CPHA 0 and CPHA 1 halves of the model
agree bit for bit, because they are the same code with one index initialised differently.
"""
from __future__ import annotations

from verif.vip.spi import SpiFlashTarget, cpol

#: The CPHA 1 modes, datasheet numbering.
CPHA1_MODES = (1, 2)


class Cpha1FlashTarget(SpiFlashTarget):
    """`SpiFlashTarget` in modes 1 and 2, with the model's CPOL and phase as configured.

    `mode` is the datasheet mode, so `1` is CPOL 0 / CPHA 1 and `2` is CPOL 1 / CPHA 1.  Everything
    else is the parent's constructor, verbatim, through the parent's own `__init__`.
    """

    def __init__(self, *args, mode: int = 1, **kwargs) -> None:
        if mode not in CPHA1_MODES:
            raise ValueError(
                f"Cpha1FlashTarget is for the CPHA 1 modes {CPHA1_MODES}, got {mode!r}; use "
                f"SpiFlashTarget for modes 0 and 3")
        # The parent accepts the mode of the same CPOL (0 or 3), so it builds all of its state.
        super().__init__(*args, mode=3 if cpol(mode) else 0, **kwargs)
        self.mode = mode
        # The only mode-dependent state, recomputed with the real mode.  `_reset_session` is the
        # parent's own method: this is a phase switch, not a new command engine.
        self._reset_session()

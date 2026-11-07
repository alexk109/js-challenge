"""MVP I2S firmware: the two PIO programs, the host driver and their test.

`driver.py` loads `program.pio` (assembled with `asm/pioasm.py`) into
*both* engines of an `iss.pio_iss.ChipDevice`: engine 0 shifts the bit clock and the serial
data out, engine 1 writes the word-select level once per slot.  The chip is the I2S
**controller**; `test_i2s.py` runs the stream against `verif.vip.i2s`'s `I2sReceiver` and
`I2sChecker` on a `verif.vip.core.Bus`, and neither model is edited or softened.

Run with `make fw PROTO=i2s TARGET=iss`.
"""
from fw.i2s.driver import (
    CORE_HZ,
    EXPECTED_CELLS,
    TICK_NS,
    Config,
    I2sDriver,
    Pins,
    clkdiv_value,
    half_cycles,
    jmp_abs,
    load_program,
    pack_sample,
    pack_ws,
    program_cycles,
    set_pindirs,
    slot_cycles,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "EXPECTED_CELLS",
    "TICK_NS",
    "Config",
    "I2sDriver",
    "Pins",
    "clkdiv_value",
    "half_cycles",
    "jmp_abs",
    "load_program",
    "pack_sample",
    "pack_ws",
    "program_cycles",
    "set_pindirs",
    "slot_cycles",
    "words_used",
]
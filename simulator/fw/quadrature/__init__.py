"""MVP quadrature-encoder firmware: the PIO program, the host driver and their test.

`program.pio` (assembled with `asm/pioasm.py`) watches the three wires
of an incremental encoder's ABI interface -- A, B and the index -- and logs every change into
its RX FIFO, ten samples per word.  `driver.py` loads it into an engine of an
`iss.pio_iss.ChipDevice`, sets the four configuration registers, drains the FIFO while the
simulation runs and turns the sample log into a position with `HostDecoder`, the same count the
reference model `verif.vip.quadrature.QuadratureDecoder` keeps on its side of the wire.
`test_quadrature.py` runs that against the model on a `verif.vip.core.Bus` and requires the
model's own `QuadratureChecker` to report no violations.

The chip plays the **target** (decoder) end of the protocol: an external encoder drives the pins
and the chip only listens.  Six of the 32 IMEM words are used.

Run with `make fw PROTO=quadrature TARGET=iss`.
"""
from fw.quadrature.driver import (
    CORE_HZ,
    DEFAULT_CHANNEL,
    NUM_PINS,
    IMEM_SIZE,
    PROGRAM_PATH,
    Config,
    DecodeError,
    HostDecoder,
    IndexEvent,
    QuadratureDriver,
    clkdiv_value,
    decode_word,
    describe,
    execctrl_value,
    jmp_abs,
    load_program,
    pinctrl_value,
    sample_positions,
    samples_per_word,
    shiftctrl_value,
    ticks_per_sm_cycle,
    unpack_word,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "DEFAULT_CHANNEL",
    "NUM_PINS",
    "IMEM_SIZE",
    "PROGRAM_PATH",
    "Config",
    "DecodeError",
    "HostDecoder",
    "IndexEvent",
    "QuadratureDriver",
    "clkdiv_value",
    "decode_word",
    "describe",
    "execctrl_value",
    "jmp_abs",
    "load_program",
    "pinctrl_value",
    "sample_positions",
    "samples_per_word",
    "shiftctrl_value",
    "ticks_per_sm_cycle",
    "unpack_word",
    "words_used",
]

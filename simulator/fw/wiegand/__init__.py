"""MVP Wiegand firmware: the PIO program, the host driver and their test.

`driver.py` loads `program.pio` (assembled with `asm/pioasm.py`) into an
engine of an `iss.pio_iss.ChipDevice`, points every register window at a pair of uio pins and
pushes one frame's direction pairs into its TX FIFO; `test_wiegand.py` runs the transaction
against `verif/vip/wiegand.py`'s `WiegandReceiver` (a passive panel that decodes the credential)
and `WiegandChecker` (an analyser that measures every pulse and pause) on a `verif.vip.core.Bus`
at 20 ns per tick.

The chip plays the **reader**: it drives 26 low pulses on two open-drain lines and then leaves
them released.  The credential and both parity bits are computed on the host, from the layout in
research/ref/protocols/wiegand/, and the frame the models decode off the pins is a second,
independent implementation of that table.

Run with `make fw PROTO=wiegand TARGET=iss`.  See README.md for the numbers and for what the
32-word IMEM and one clock divider do and do not reach.
"""
from fw.wiegand.driver import (
    CORE_HZ,
    DEFAULT_LAYOUT,
    LAYOUTS,
    PROTO_NAME,
    READER_DEFAULTS_NS,
    TICK_NS,
    UIO_PINS,
    WIEGAND_WINDOW_NS,
    Config,
    Layout,
    WiegandDriver,
    bit_timing,
    bits_from_pairs,
    ceil_ticks,
    clkdiv_for_pulse,
    direction_pairs,
    divider_value,
    execctrl_value,
    frame_bits,
    frame_pairs,
    frame_words,
    jmp_abs,
    layout,
    load_program,
    parity_bit,
    phase_cycles,
    pinctrl_value,
    shiftctrl_value,
    ticks_per_sm_cycle,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "DEFAULT_LAYOUT",
    "LAYOUTS",
    "PROTO_NAME",
    "READER_DEFAULTS_NS",
    "TICK_NS",
    "UIO_PINS",
    "WIEGAND_WINDOW_NS",
    "Config",
    "Layout",
    "WiegandDriver",
    "bit_timing",
    "bits_from_pairs",
    "ceil_ticks",
    "clkdiv_for_pulse",
    "direction_pairs",
    "divider_value",
    "execctrl_value",
    "frame_bits",
    "frame_pairs",
    "frame_words",
    "jmp_abs",
    "layout",
    "load_program",
    "parity_bit",
    "phase_cycles",
    "pinctrl_value",
    "shiftctrl_value",
    "ticks_per_sm_cycle",
    "words_used",
]
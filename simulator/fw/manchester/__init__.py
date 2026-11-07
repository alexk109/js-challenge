"""MVP Manchester transmitter firmware: the PIO programs, the host driver and their test.

`driver.py` loads one of the two programs in `program.pio` (assembled
with `asm/pioasm.py`) into an engine of an `iss.pio_iss.ChipDevice`, configures it through
the register map and pushes bytes into its TX FIFO; `test_manchester.py` runs the transaction
against `verif.vip.manchester.py` -- the model's `ManchesterRx` and its passive
`ManchesterTimingChecker` -- on a `verif.vip.core.Bus` at 20 ns per tick.

The chip plays the **transmitter**.  See `fw/manchester/README.md` for what that costs, what
the two programs are for and the numbers this firmware reaches.

Run with `make fw PROTO=manchester TARGET=iss`.
"""
from fw.manchester.driver import (
    CORE_HZ,
    IDLE_LEVEL,
    INTERVAL_TOLERANCE,
    PROGRAMS,
    PROTO_NAME,
    TICK_NS,
    Config,
    ManchesterTxDriver,
    bit_loop,
    bit_rate_for_clkdiv,
    bits_per_word,
    check_convention,
    chip_lengths_sm_cycles,
    chip_lengths_ticks,
    clkdiv_for_bit_rate,
    clkdiv_value,
    gap_sm_cycles,
    half_bit_ticks,
    half_bit_tolerance_ticks,
    jmp_abs,
    legal_bit_rates,
    load_program,
    message_sm_cycles,
    pinctrl_value,
    set_pindirs,
    set_pins,
    shiftctrl_pull_thresh,
    shiftctrl_value,
    sm_cycles_per_bit,
    sm_cycles_to_ticks,
    split_error_ticks,
    split_is_legal,
    ticks_per_bit,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "IDLE_LEVEL",
    "INTERVAL_TOLERANCE",
    "PROGRAMS",
    "PROTO_NAME",
    "TICK_NS",
    "Config",
    "ManchesterTxDriver",
    "bit_loop",
    "bit_rate_for_clkdiv",
    "bits_per_word",
    "check_convention",
    "chip_lengths_sm_cycles",
    "chip_lengths_ticks",
    "clkdiv_for_bit_rate",
    "clkdiv_value",
    "gap_sm_cycles",
    "half_bit_ticks",
    "half_bit_tolerance_ticks",
    "jmp_abs",
    "legal_bit_rates",
    "load_program",
    "message_sm_cycles",
    "pinctrl_value",
    "set_pindirs",
    "set_pins",
    "shiftctrl_pull_thresh",
    "shiftctrl_value",
    "sm_cycles_per_bit",
    "sm_cycles_to_ticks",
    "split_error_ticks",
    "split_is_legal",
    "ticks_per_bit",
    "words_used",
]
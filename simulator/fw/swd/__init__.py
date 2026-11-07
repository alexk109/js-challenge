"""MVP SWD-controller firmware: the PIO program, the host driver and their test.

The chip is the **controller** (the debug probe side): it
generates SWCLK and drives SWDIO open drain, and the SW-DP on the other end is
`verif.vip.swd.SwdTarget`.  `driver.py` loads `program.pio` (assembled with
`asm/pioasm.py`) into an engine of an `iss.pio_iss.ChipDevice`, configures it
through the register map and pushes the three words of each transfer into its TX
FIFO; `test_swd.py` runs those transfers against the DP with `verif.vip.swd.
SwdChecker` watching the two nets.

Run with `make fw PROTO=swd TARGET=iss`.

`fw/swd/LIMITS.md` has the word budget of the program and what the 32-word IMEM
does not leave room for.
"""
from fw.swd.driver import (
    ACK_PERIODS,
    IN_SHIFTDIR,
    L_RESET_LOW_BITS,
    L_RESET_PERIODS,
    MIN_HIGH,
    OUT_SHIFTDIR,
    RDATA_PERIODS,
    REQUEST_PERIODS,
    PULL_THRESH,
    PUSH_THRESH,
    READ_PERIODS,
    WDATA_TX_PERIODS,
    WORDS_PER_TRANSFER,
    Config,
    SwdDriver,
    SwdError,
    check_program,
    closing_word,
    clkdiv_word,
    data_word,
    execctrl_word,
    jmp_abs_word,
    load_program,
    period_ticks,
    pinctrl_word,
    request_word,
    retarget,
    set_pins_word,
    shiftctrl_word,
    swclk_hz,
    transfer_words,
)

__all__ = [
    "ACK_PERIODS",
    "IN_SHIFTDIR",
    "L_RESET_LOW_BITS",
    "L_RESET_PERIODS",
    "MIN_HIGH",
    "OUT_SHIFTDIR",
    "RDATA_PERIODS",
    "REQUEST_PERIODS",
    "PULL_THRESH",
    "PUSH_THRESH",
    "READ_PERIODS",
    "WDATA_TX_PERIODS",
    "WORDS_PER_TRANSFER",
    "Config",
    "SwdDriver",
    "SwdError",
    "check_program",
    "closing_word",
    "clkdiv_word",
    "data_word",
    "execctrl_word",
    "jmp_abs_word",
    "load_program",
    "period_ticks",
    "pinctrl_word",
    "request_word",
    "retarget",
    "set_pins_word",
    "shiftctrl_word",
    "swclk_hz",
    "transfer_words",
]
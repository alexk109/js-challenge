"""MVP UART-TX firmware: the PIO program, the host driver and their test.

`driver.py` loads `program.pio` (assembled with `asm/pioasm.py`) into
an engine of an `iss.pio_iss.ChipDevice`, configures it through the register map and pushes
bytes into its TX FIFO; `test_uart_tx.py` runs the transaction against `verif.vip.uart.UartRx`
on a `verif.vip.core.Bus`.

Run with `make fw PROTO=uart_tx TARGET=iss`.
"""
from fw.uart_tx.driver import (
    CORE_HZ,
    Config,
    UartTxDriver,
    bit_cycles,
    clkdiv_for_baud,
    clkdiv_value,
    load_program,
    pinctrl_value,
    shiftctrl_value,
    ticks_per_bit,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "Config",
    "UartTxDriver",
    "bit_cycles",
    "clkdiv_for_baud",
    "clkdiv_value",
    "load_program",
    "pinctrl_value",
    "shiftctrl_value",
    "ticks_per_bit",
    "words_used",
]

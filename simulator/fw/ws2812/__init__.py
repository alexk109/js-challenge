"""MVP WS2812 controller firmware: the PIO program, the host driver and their test.

The chip plays the **controller** of a WS2812 chain: it drives the single
data line and the pixels are the targets, which is the side `verif/vip/ws2812.py` puts its
`Ws2812Tx` and `Ws2812Pixel` on.  `driver.py` loads `program.pio` (assembled with
`asm/pioasm.py`) into an engine of an `iss.pio_iss.ChipDevice`, configures it through the
register map and pushes one 24-bit GRB word per pixel into its TX FIFO; `test_ws2812.py` runs
the frames against `verif.vip.ws2812` on a `verif.vip.core.Bus` and lets
`Ws2812TimingChecker` judge the wire.

    make fw PROTO=ws2812 TARGET=iss

Two speeds ship, both `.program`s in `program.pio` and both driven by the same host code:
`ws2812` at the datasheet's own 1.26 us bit period (793.7 kbit/s at 50 MHz) and
`ws2812_fast` at the fastest this program shape can legally go, 0.8 us (1.25 Mbit/s), which
is inside every window the datasheet gives.  `fw/ws2812/README.md` has the measured numbers
and the limits.
"""
from fw.ws2812.driver import (
    BITS_PER_PIXEL,
    CORE_HZ,
    DEFAULT_PROGRAM,
    NS_PER_TICK,
    PROGRAMS,
    PROGRAM_PATH,
    T_RESET_NS,
    Config,
    Timing,
    Ws2812Driver,
    clkdiv_value,
    decode_word,
    grb_word,
    is_nop,
    is_out_x_1,
    is_pull_block,
    jmp_abs,
    load_program,
    max_cycles_per_instruction,
    osr_word,
    program_names,
    set_pindirs_word,
    set_pins_word,
    sm_cycles_ticks,
    ticks_of,
    timing,
    words_used,
)

#: The reset code in core cycles: 50 us at 20 ns per tick, rounded up (pages 3 and 4).
RESET_TICKS = ticks_of(T_RESET_NS)

__all__ = [
    "BITS_PER_PIXEL",
    "CORE_HZ",
    "DEFAULT_PROGRAM",
    "NS_PER_TICK",
    "PROGRAMS",
    "PROGRAM_PATH",
    "RESET_TICKS",
    "T_RESET_NS",
    "Config",
    "Timing",
    "Ws2812Driver",
    "clkdiv_value",
    "decode_word",
    "grb_word",
    "is_nop",
    "is_out_x_1",
    "is_pull_block",
    "jmp_abs",
    "load_program",
    "max_cycles_per_instruction",
    "osr_word",
    "program_names",
    "set_pindirs_word",
    "set_pins_word",
    "sm_cycles_ticks",
    "ticks_of",
    "timing",
    "words_used",
]
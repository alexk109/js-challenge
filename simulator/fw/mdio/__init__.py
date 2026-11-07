"""MVP MDIO/MDC (IEEE 802.3 clause 22) station firmware.

The chip is the *station*: it owns MDC and drives the data, and the PHYs on the bus answer a
read's turnaround and data bits.  `verif/vip/mdio.py`'s `MdioStation` model is a station, so
this firmware replaces it; `MdioPhy` and `MdioChecker` are what judge the result.

    fw/mdio/program.pio   26 words: halt, write_frame, read_frame
    fw/mdio/driver.py     MdioDriver: registers, frame words, and the frame runner
    fw/mdio/test_mdio.py  make fw PROTO=mdio TARGET=iss

`MdioDriver` is the whole host API: `start()`, then `write_reg(reg, value)` /
`read_reg(reg)` per clause 22 access, or `queue_write`/`queue_read` plus `run()` for a burst.
"""
from fw.mdio.driver import (
    ADDRESS_BITS,
    CORE_HZ,
    DATA_BITS,
    FRAME_BITS,
    IMEM_SIZE,
    MAX_MDC_HZ,
    MIN_PERIOD_NS,
    MdioBusMonitor,
    MdioDriver,
    MdioOp,
    PREAMBLE_WORD,
    PROGRAM_PATH,
    PROTO_NAME,
    Config,
    bit_cycles,
    check_program,
    clkdiv_for_mdc,
    clkdiv_value,
    control_word,
    decode_word,
    execctrl_value,
    frame_words,
    jmp_abs,
    load_program,
    pinctrl_value,
    pull_word,
    read_data,
    set_word,
    shiftctrl_value,
    ticks_per_bit,
    words_used,
)

__all__ = [
    "ADDRESS_BITS", "CORE_HZ", "DATA_BITS", "FRAME_BITS", "IMEM_SIZE", "MAX_MDC_HZ",
    "MIN_PERIOD_NS", "MdioBusMonitor", "MdioDriver", "MdioOp", "PREAMBLE_WORD", "PROGRAM_PATH",
    "PROTO_NAME", "Config", "bit_cycles", "check_program", "clkdiv_for_mdc", "clkdiv_value",
    "control_word", "decode_word", "execctrl_value", "frame_words", "jmp_abs", "load_program",
    "pinctrl_value", "pull_word", "read_data", "set_word", "shiftctrl_value", "ticks_per_bit",
    "words_used",
]

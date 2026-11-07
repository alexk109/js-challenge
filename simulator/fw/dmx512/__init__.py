"""MVP DMX512-A controller firmware: the PIO program, the host driver and their test.

`program.pio` is the whole transmitter: BREAK, MAB, START Code, data slots
and the mark before the next break, in 28 of the 32 words of one engine's IMEM.  `driver.py`
assembles it for a bit rate and a frame length, derives every duration from the words it
produced, programs an engine of an `iss.pio_iss.ChipDevice` and feeds it one FIFO word per
slot; `test_dmx512.py` puts that on a `verif.vip.core.Bus` at 20 ns per tick and lets
`verif.vip.dmx512.DmxChecker` judge the wire.

    make fw PROTO=dmx512 TARGET=iss

The chip is the **controller**: every rule ANSI E1.11 puts on the wire is a transmitter's rule
(tables 6 and 7), and the checker measures a transmitter.  What is on the wire is not this
firmware's word for it -- the checker's numbers come from the edges, and the test compares them
with the numbers the driver derived from the instruction words.
"""
from fw.dmx512.driver import (
    CORE_HZ,
    NS_PER_TICK,
    PROTO_NAME,
    SLOT_BITS,
    Config,
    Dmx512Driver,
    Plan,
    assemble_for,
    bit_cycles,
    bit_delay_for_rate,
    bit_rate_limits,
    break_cycles,
    clkdiv_value,
    defines_of,
    execctrl_value,
    frame_cycles,
    load_program,
    loop_passes,
    mab_cycles,
    pad_cycles,
    pinctrl_value,
    plan_for,
    select_bit_delay,
    shiftctrl_value,
    slot_count,
    slot_cycles,
    slots_limit,
    ticks_per_bit,
    word_at,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "NS_PER_TICK",
    "PROTO_NAME",
    "SLOT_BITS",
    "Config",
    "Dmx512Driver",
    "Plan",
    "assemble_for",
    "bit_cycles",
    "bit_delay_for_rate",
    "bit_rate_limits",
    "break_cycles",
    "clkdiv_value",
    "defines_of",
    "execctrl_value",
    "frame_cycles",
    "load_program",
    "loop_passes",
    "mab_cycles",
    "pad_cycles",
    "pinctrl_value",
    "plan_for",
    "select_bit_delay",
    "shiftctrl_value",
    "slot_count",
    "slot_cycles",
    "slots_limit",
    "ticks_per_bit",
    "word_at",
    "words_used",
]
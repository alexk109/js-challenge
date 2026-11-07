"""Directed stimulus for the chip's worst slow-corner paths.

    from fw.slowpath import stimulus as sp
    sp.program("fetch_probe")             # the assembled program
    sp.fetch_latency_set(divisor=2)       # the core cycles the ISA says that answer takes

`stimulus.py` holds the two programs of `program.pio` (and their two planted-fault mutants),
the pin map, the register words, and -- in one place -- every cycle count the tests assert,
derived from `docs/mvp-isa.md` and `docs/mvp-regmap.md`.  Two front-ends run the same
expectations: `test/test_slowpath.py` drives the Tiny Tapeout pins (which is what the
SDF-annotated gate-level bench in `sim/sdf/` runs) and `fw/slowpath/test_slowpath.py` drives
the same stimulus through `iss.pio_iss.ChipDevice`, so it runs on `FW_TARGET=iss`, `iss_b` and
`rtl`.

See `fw/slowpath/README.md` for what the paths are and what was measured.
"""
from __future__ import annotations

from .stimulus import (  # noqa: F401  (re-exported for the two front-ends)
    ANSWER_PIN,
    BIT_RX_VALID,
    IN_PIPE,
    MUTANT_EXTRA_SM_CYCLES,
    OUT_PIPE,
    PROGRAM_PIO,
    TXF_WORDS,
    WAIT_PIN,
    anchor_gaps,
    anchor_values,
    check_exact,
    check_set,
    config_for,
    ctrl_bits,
    fetch_latency_set,
    fetch_pulse_width,
    imem_addr,
    net_of,
    phases,
    pin_port,
    program,
    rising_edges,
    rxf_irq_latency_set,
    sm_addr,
    sm_clkdiv,
    sm_execctrl,
    sm_pinctrl,
    sm_shiftctrl,
)
"""MVP NEC infrared firmware: the PIO program, the host driver and their test.

`driver.py` loads `program.pio` (assembled with `asm/pioasm.py`) into an
engine of an `iss.pio_iss.ChipDevice`, sets CLKDIV and the four per-engine registers through
`docs/mvp-regmap.md`, and drains the engine's RX FIFO; `test_necir.py` drives the transaction
against `verif.vip.necir` -- its `NecirTx` is the remote, its `NecirRx` the independent decoder
and its `NecirChecker` the protocol checker -- on a `verif.vip.core.Bus` at 20 ns per tick.

    make fw PROTO=necir TARGET=iss

The chip is the **target**: a receiver.  It never drives the net, so one decoded NEC frame is one
32-bit word in the RX FIFO and that word is the judgement -- compared, bit for bit, with
`verif.vip.necir.encode_frame`.  A repeat code, which carries no data, pushes nothing.

The three numbers worth knowing before reading `program.pio`:

* **27 of 32 words.**  Three samples (the 9 ms leader mark, the leader space that tells a frame
  from a repeat code, and the data space), one bit counter, and the push.
* **S = 9 SM cycles per NEC burst** at the default, i.e. CLKDIV = 3.125 core cycles at 50 MHz.
  S is the only "speed" a receiver whose protocol rate is fixed has, and it may not be 1: the
  sample windows cross at S = 7 (`sample_window` computes them and `build_program` refuses the
  rest).
* **The model runs NEC 1000x faster than a real remote.**  `verif.vip.necir` states its elements
  in nanoseconds, so its burst is 562.5 ns where a real NEC burst is 562.5 us.  Nothing here
  depends on the scale -- pass `unit_ns=REAL_NEC_UNIT_NS` and the same 27 words decode a real
  remote, with CLKDIV = 3125.
"""
from __future__ import annotations

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    CORE_HZ,
    DEFAULT_SM_CYCLES_PER_UNIT,
    IMEM_SIZE,
    LOOP_OVERHEAD,
    MARK_REACH,
    MAX_SM_CYCLES_PER_UNIT,
    MIN_SM_CYCLES_PER_UNIT,
    NEC_UNIT_NS,
    NS_PER_TICK,
    REAL_NEC_UNIT_NS,
    RX_PINS,
    SPACE_REACH,
    NecirError,
    NecirReceiver,
    NecirWord,
    add_protocol_nets,
    attach_ir_net,
    build_program,
    check_program,
    clkdiv_fields,
    clkdiv_value,
    cycles_per_sm_cycle,
    decode_frame,
    decode_instruction,
    imem_addr,
    load_program,
    round_error,
    sample_cycles,
    sample_margins,
    sample_points,
    sample_window,
    sample_window_of,
    shiftctrl_fields,
    shiftctrl_value,
    sm_addr,
    window_of,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "DEFAULT_SM_CYCLES_PER_UNIT",
    "IMEM_SIZE",
    "LOOP_OVERHEAD",
    "MARK_REACH",
    "MAX_SM_CYCLES_PER_UNIT",
    "MIN_SM_CYCLES_PER_UNIT",
    "NEC_UNIT_NS",
    "NS_PER_TICK",
    "REAL_NEC_UNIT_NS",
    "RX_PINS",
    "SPACE_REACH",
    "NecirError",
    "NecirReceiver",
    "NecirWord",
    "add_protocol_nets",
    "attach_ir_net",
    "build_program",
    "check_program",
    "clkdiv_fields",
    "clkdiv_value",
    "cycles_per_sm_cycle",
    "decode_frame",
    "decode_instruction",
    "imem_addr",
    "load_program",
    "round_error",
    "sample_cycles",
    "sample_margins",
    "sample_points",
    "sample_window",
    "sample_window_of",
    "shiftctrl_fields",
    "shiftctrl_value",
    "sm_addr",
    "window_of",
    "words_used",
]

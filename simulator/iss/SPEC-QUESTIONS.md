# Spec questions from the golden ISS

Each item: where the spec (`docs/mvp-isa.md`, `docs/mvp-regmap.md`, `docs/mvp-interfaces.md`)
is silent or ambiguous, and the choice `iss/pio_iss.py` makes. The second ISS and the RTL
should make the same choice, or the spec should be fixed and all three follow.

1. **`iss/isa.py` has no owner.** `docs/mvp-isa.md` (`### Constants`) and `docs/mvp.md` say
   the ISS and RTL take `T_OUT`, `T_IN`, `T_IMEM` from `iss/isa.py`, shared with the
   assembler. Nothing in the repo provides it. Choice: `pio_iss.py` imports the constants from
   `iss.isa` if that module exists, else defines `T_OUT = 1`, `T_IN = 3`, `T_IMEM = 2` itself.
   `T_IN` is a parameter (pipeline depth). `T_OUT = 1` and `T_IMEM = 2` are structural, and
   the module asserts these values.

2. **A reserved MOV with destination EXEC** (Op `11` or Source `100`). The reference step
   treats "MOV EXEC" specially: no delay load, `latch_pend` kept. Choice: such a word is a
   NOP (`### Removed and reserved encodings`, item 2), not a MOV EXEC. Its delay applies, it
   writes no latch, and if it was run from the latch it clears `latch_pend`.

3. **`latch_host` stays 1 after a host instruction completes.** The text only says it is
   cleared by OUT/MOV EXEC writing the latch and by `SM_RESTART`. Choice: literal. After a
   forced instruction completes, `latch_host = 1` and `latch_pend = 0` until the next EXEC or
   restart. This is invisible in `EXEC_STALLED` (`latch_pend & latch_host`), but `state()`
   now exposes `latch_host`, so lockstep compares it. The RTL must not clear it on completion.

4. **A host `SMx_INSTR` write while an EXEC executee is pending** (`latch_pend = 1` from an
   OUT/MOV EXEC, waiting for its SM cycle because of the divider), in a cycle where the engine
   does not execute. The spec says only that the host wins the latch in the *same* cycle as
   an OUT/MOV EXEC. Choice: same outcome, from the reference step. The host write overwrites
   `latch`. If the forced instruction completes, it clears `latch_pend`, and the pending
   executee is lost. The host write itself does not change `latch_pend`.

5. **IMEM write to the PC of a stalled IMEM instruction.** `stalled` stays 1, and the next
   cycle executes the *new* word (via `IR`) as if it were a retry, so it skips its side-set.
   Choice: literal (side-set only if `not stalled`). A spec note would help firmware authors.

6. **A `CLKDIV` write in the cycle where the divider reloads** (`cnt == 1` at the end of `h`).
   "Picked up at the next reload" could mean this one. Choice: start-of-cycle state, so the
   reload at the end of `h` uses the old value and the next reload uses the new one.

7. **Read-back of out-of-range fields.** `SIDESET_COUNT` 6/7 "behave as 5", `OUT_COUNT` > 32
   "behaves as 32". Choice: `PINCTRL` stores and reads back all 32 bits as written. Only the
   behaviour is clamped.

8. **`SIDE_EN = 1` with `SIDESET_COUNT = 1`.** There is an enable bit but no data bits.
   Choice: nothing is written (zero-width window). The field has no delay bits left over
   besides the low 4.

9. **The table in `### Stalls, delay and side-set, summarised`** says
   "`osr_cnt >= PULL_THRESH`". The OUT rule says `thresh(PULL_THRESH)`. Choice: `thresh()`
   (so 0 means 32). This only needs a wording fix.

10. **Pad value of output-only pins for the input pipeline.** "Feeds the same pipeline from its
    driven value". Choice: the registered level of the pin in cycle `m` is the pad value in
    `m`, exactly as for a `uio` pin whose pad follows `uio_out`. So an engine reads back its
    own write to pin 13..17 `T_OUT + T_IN = 4` cycles after the writing SM cycle.

11. **`INT = 0, FRAC != 0`.** This is defined by the formula, but note the first period after a
    restart is 65536 cycles (`acc` starts at 0 and the first reload adds `FRAC` with no
    carry), and later periods alternate as the accumulator wraps. Tests pin this down.

12. **`ChipDevice` and missing nets.** `docs/mvp-interfaces.md` assumes nets `p0..p17`. Choice:
    a net that is absent from the bus reads as 0 and is never driven.

13. **Debug aid outside the API.** `Chip.last_in_vec` (the `IN_VEC` used in the last stepped
    cycle) and `Chip.peek(addr)` (a host read without side effects) are extras that the tests
    use. Nothing else depends on them, and the second ISS does not need them.

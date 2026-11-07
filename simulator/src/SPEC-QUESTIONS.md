# RTL spec questions

Places where `docs/mvp-isa.md` / `docs/mvp-regmap.md` / `docs/mvp-interfaces.md` left a choice to
the RTL. Each item gives the question, what `src/pio_*.sv` does now, and why. If the ISS chose
differently, the lockstep fuzz will show it; the fix belongs in the spec first.

## Q1. Is a MOV EXEC with a reserved Op or Source an "EXEC-er"?

`MOV EXEC` (dest `100`) with Op `11` or Source `100` hits two rules: the reference step's
"`I is MOV EXEC`" (no delay, `latch_pend` kept) and "reserved field values execute as a NOP,
side-set and delay apply as normal".

**RTL:** reserved wins. Such a MOV is a NOP: no latch write, `latch_pend` cleared as for any
non-EXEC from the latch, PC advances, delay loads. Same for MOV dest `011`. Only a MOV with a
valid dest `100`, valid Op and valid Source counts as `is_exec`.

## Q2. SM_RESTART in a cycle where the engine also executes

The spec: "the restart wins for those fields" (ISR, `osr_cnt`, `isr_cnt`, delay, `stalled`,
`latch_pend`, `latch_host`, force). It does not say whether the instruction's other effects in
that cycle happen.

**RTL:** they happen. FIFO pushes and pops, X/Y/OSR/PC, the latch value, pin writes and FDEBUG
flags from the engine's step in cycle `h` all take effect; only the listed fields are overridden.
Consequences worth confirming:
- A PULL, an OUT refill or a background autopull in cycle `h` pops a TX word into OSR, then
  `osr_cnt` is forced to 32, so the next autopull/PULL overwrites it: the word is lost.
- A PUSH or an autopushing IN in cycle `h` still pushes to the RX FIFO.

## Q3. `bus_rdata` outside read cycles

The interface defines `bus_rdata` for read cycles only. **RTL:** 0 whenever `bus_re = 0`
(including write cycles). Harmless if the ISS/harness compare only on reads.

## Q4. `pin_out[12:8]` (input-only pins)

The interface fixes `pin_oe[12:8] = 0` but says nothing about `pin_out[12:8]`. **RTL:** 0, the
same as `DBG_PADOUT` bits 12:8. There are no level flops for these pins.

## Q5. Host `SMe_INSTR` write in the same cycle as an engine latch update

"The host's value wins the latch" and "if the host and an engine update the same state in the
same cycle, the host wins". `latch_pend` is not written by the host.

**RTL:** in that cycle `latch` = host data, `latch_host` = 1, `force` = 1, and `latch_pend` takes
the engine's value (1 after an OUT/MOV EXEC or a stall from the latch). In `h+1` the forced
instruction runs; if it completes, `latch_pend` clears, so the engine's own EXEC'd/stalled
instruction is dropped.

## Q6. Lockstep visibility details (docs/mvp-interfaces.md)

Not a behaviour question; recorded so the harness author doesn't have to guess:
- Widths in `g_eng[e].u_eng`: `pc` 5, `ir` 16, `x`/`y`/`osr`/`isr` 32, `osr_cnt`/`isr_cnt` 6
  (0..32), `delay_cnt` 5, `stalled` 1, `latch` 16, `latch_pend` 1, `tx_level`/`rx_level` 3
  (0..4; output ports of the engine, also wired to the FIFOs).
- The spec's `force` is named `forced` (`force` is a SystemVerilog keyword). It and `latch_host`
  are not in the lockstep list but change behaviour; consider adding them to `Chip.state()`.
- Verilator needs `--public-flat-rd` (or equivalent) for the harness to read these names.

## Q7. Pin latency vs. physical timing (spec open question 2, still open)

`T_OUT = 1` puts IR decode, the OUT/SET/side-set window rotates, two-engine resolution and the
pad flop in one cycle, and the PC_next → 32:1 IMEM mux → IR path is also single-cycle. Not yet
timed (no STA run). `T_IN` is a parameter of `pio_chip`; `T_OUT` is structural, so going to
`T_OUT = 2` means adding a request register stage between `u_eng` and the resolution logic.

## Minor wording

- `### Stalls, delay and side-set, summarised` writes "`osr_cnt >= PULL_THRESH`"; the RTL uses
  `thresh(PULL_THRESH)` as in `**OUT**`.
- `### When an engine executes` says "outside SM cycles the engine changes no state of its own",
  but the next paragraph has `IR` update every core cycle. The RTL follows the IR paragraph.

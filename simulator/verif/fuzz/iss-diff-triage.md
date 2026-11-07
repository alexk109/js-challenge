# ISS differential triage: golden `iss/` vs `verif/iss_b/`

`verif/fuzz/iss_diff.py` runs both ISSs on the lockstep harness's random stimulus
(`verif/fuzz/gen.py`) and stops each seed at the first disagreement, comparing every cycle
`bus_rdata` (in a read cycle), `pin_out`, `pin_oe`, `irq0`, `irq1` and every key of
`Chip.state(e)` for both engines. Start: **0/20 seeds match**. Now: **20/20 seeds match**, and so do
a further 1,000 seeds (201..1200) and the same 20 seeds at 100,000 cycles.

Each line below is one root cause: what disagreed, which side was wrong, and the spec text
that decides it. Every one of them turned out to be a `verif/iss_b` bug; no golden-side or
spec-ambiguity finding survived the second look, which is the answer the differential was looking for.

| # | Root cause | Wrong side | Spec citation | First seen (seed, cycle, field: golden vs iss_b) | Fix |
|---|---|---|---|---|---|
| 1 | `FLEVEL` packed its four occupancies four bits apart (`FSTAT`'s step) instead of eight, so engine 1's levels landed inside engine 0's nibble | **iss_b** | `docs/mvp-regmap.md` `**FLEVEL**`: TX0 `3:0`, RX0 `7:4`, TX1 `11:8`, RX1 `15:12` | seed 2, cycle 709, `bus_rdata@0x03`: `788` vs `60` (and seed 13, cycle 133: `12802` vs `418`) | `_flevel`, `429370d` |
| 2 | A `SMe_CLKDIV` write was masked with `0x00FFFF00`, so bits `31:24` of `INT` were thrown away (an `INT` above 255 became garbage) | **iss_b** | `docs/mvp-regmap.md` `**SMe_CLKDIV**`: `31:16 INT`, `15:8 FRAC`, `7:0 — reserved` | seed 5, cycle 106, `bus_rdata@0x38`: `1072209408` (`0x3FE89E00`) vs `15244800` (`0x00E89E00`) (seeds 8, 9, 16 the same way) | `load_cfg` and `_write`, commit `84fb03a` |
| 3 | With `EXECCTRL.SIDE_EN = 1` the side-set window was `SC` pins wide, so the enable bit's pin (`SIDESET_BASE + SC - 1`) was written with 0 | **iss_b** | `docs/mvp-isa.md` `### Delay/side-set field`: "the MSB of the side-set bits is an enable, and the remaining `SC - 1` bits are side-set data" + "Side-set data bit 0 goes to pin `SIDESET_BASE`" | seed 8, cycle 299, `pin_out`: `22` vs `20`; seed 14, cycle 401, `pin_oe`: `254081` vs `254080` (seeds 3, 4, 11, 15, 17, 20 the same way) | `_sm_cycle`, commit `548b770` (+ test `c2559c1`) |
| 4 | A `MOV EXEC` whose Op or Source is a reserved field value was treated as an EXEC-er: it loaded no latch (correct) but also skipped its own delay field | **iss_b** | `docs/mvp-isa.md` `### Removed and reserved encodings` item 2: "The instruction executes as a NOP. It has no main effect, never stalls, and advances the PC as a non-jump would. **Side-set and delay apply as normal.**" | seed 19, cycle 212, `eng0.delay_cnt`: `7` vs `6` (seeds 2, 3, 4, 6, 12, 14, 17, 18, 20 the same way, after 1..3 were fixed) | `_is_exec`, commit `4a7cc2f` |
| 5 | A **non-blocking** `PULL` on an empty TX FIFO raised `FDEBUG.TXSTALL` | **iss_b** | `docs/mvp-isa.md` `### PULL`: "if the TX FIFO is empty: **Block**: stall and set `TXSTALL`. **No Block**: `OSR = X` and `osr_cnt = 0`"; `docs/mvp-regmap.md` `**FDEBUG**`: "TXSTALL \| the engine stalls on an empty TX FIFO: blocking PULL, or OUT with autopull" | seed 9, cycle 870, `bus_rdata@0x02`: `33751808` (`0x02030300`) vs `50529024` (`0x03030300`) — the extra bit is `TXSTALL[0]` | `_execute_4567`, `de9107c` |
| 6 | `Chip.state(e)` did not expose `latch_host` and `forced`, so the harness skipped two compared keys | **iss_b** (interface) | `docs/mvp-interfaces.md` `### ISS API`: keys "exactly: pc ir x y osr isr osr_cnt isr_cnt delay_cnt stalled latch latch_pend latch_host forced tx_level rx_level" | not a mismatch: `iss_diff` reported "iss_b lacks state keys ['forced', 'latch_host'] (not compared)" | `Engine.state`, commit `0b75cd2` |
| 7 | Two tests pinned the wrong behaviour and had to be corrected with the spec text: `test_flevel_reports_the_four_occupancies` and the two non-blocking-`PULL` `TXSTALL` assertions | **iss_b tests** | as rows 1 and 5 | — | `test_regmap.py`, `test_push_pull.py`, commits `429370d`, `de9107c` |

Regression tests added for every row (each one fails on the pre-fix code, verified by
re-introducing the bug): `test_flevel_nibbles_are_four_bits_wide`,
`test_side_en_leaves_only_sc_minus_one_pins_in_the_side_set_window`,
`test_a_reserved_mov_exec_is_a_nop_and_not_an_exec_er`,
`test_pull_noblock_on_an_empty_fifo_copies_x`.

`verif/iss_b/SPEC-QUESTIONS.md` was updated to match: item 11 (reserved encodings and the
delay), item 12 (count handling), and two new items, 19 (the side-set window with
`SIDE_EN = 1`) and 20 (`TXSTALL` only for the blocking `PULL`), plus a status section at
the end that says which of the ISS's documented readings the differential exercised.

## How each one was decided

The rule for this triage is that the *spec* decides, and that `iss/pio_iss.py` must not be
read. Every row above was reached from the stimulus and the observed values alone:

- The golden ISS's *documented* surface (`state(e)`, `pin_out`, `pin_oe`, `irq0`, `irq1`,
  the `bus_rdata` of a read) plus the bus-op list of `verif/fuzz/gen.py` is enough to
  reconstruct what each engine did in the divergence cycle: the instruction is fixed by the
  reference step (`force` → `latch`, else `latch_pend` → `latch`, else `delay`, else `IR`),
  and the configuration registers are known from the stimulus's own writes, so the
  side-set/OUT/SET windows can be recomputed by hand from the spec text.
- For `FDEBUG` (row 5) the flags are sticky and are only observable on a read, so a single
  run cannot see the cycle that set them. Reading `FDEBUG` in every otherwise idle cycle
  (a read has no side effect) localises the cycle exactly: seed 9 cycle 867, where engine 0
  executed `pull noblock` with an empty TX FIFO.
- `CLKDIV` (row 2) and `FLEVEL` (row 1) are read-back differences: the written value is in
  the stimulus, so the correct read-back follows from the regmap's field table alone.

## What was *not* verified

- `verif/iss_b/` is still an independent second implementation: it was corrected *towards*
  the spec, never towards the golden ISS. Where the two agreed, nothing was learned; the
  fuzz only shows that 20 (and 600 more) seeds × 10,000 cycles of this stimulus agree.
- Coverage of the 20 seeds, measured on the golden ISS with `verif.fuzz.lockstep.Coverage`:
  79,175 SM cycles, 43,575 stall cycles, 26,078 host-forced cycles, all eight opcodes
  executed, 3,656 distinct instruction forms (opcode + bits 7:5 + 4:0), 96,624 bus writes
  and 31,349 reads. What this stimulus does *not* generate: `wait 1 irq` inside the coverage
  images (it would wedge the engine), so the `wait 1 irq` stall path is only reached when
  `wedge` is rolled; `SET_COUNT`/`OUT_COUNT`/`SIDESET_COUNT` combinations outside the
  generator's own choices; and host traffic patterns `gen.HOST_MIX` does not contain.
- No RTL was involved: `src/` was not read and the lockstep RTL/ISS campaign
  (`make fuzz`) was not re-run, so "the golden ISS is right" rests on the spec plus the
  earlier lockstep fuzz result, not on this triage.
- `iss/pio_iss.py` was never read, as the rule requires; the golden side was used only
  through the `Chip` API.
- Wider campaigns were run to
  look for further root causes once the 20 seeds were clean: seeds 201..800 (600 seeds),
  seeds 801..1200 (400 seeds) and the same 20 seeds at 100,000 cycles instead of 10,000.
  All match. They are evidence that nothing else is hiding in this stimulus, not part of
  the acceptance bar.
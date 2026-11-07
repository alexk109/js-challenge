# Spec questions: where `docs/mvp-isa.md` / `docs/mvp-regmap.md` is open, and what
# this ISS does

Each item names the spec text, the reading taken, and where it shows up in
`pio_iss_b.py`. These are the places where two ISSs written from the same text can
legitimately disagree, so they are the first things to check when the differential
fuzz against `iss/pio_iss.py` reports a mismatch.

## 1. FDEBUG read-back (W1C register, RO behaviour unspecified)

**Question.** `docs/mvp-regmap.md` `## Registers` lists `FDEBUG` (`0x02`) as access
type `W1C`, and the access-type list says only "write 1 to clear" — unlike `WO` and
`SC`, where it spells out "reads return 0". A W1C register normally reads back its
flags. Which is it?

**Reading taken.** A read of `FDEBUG` returns the current flags (`_read`, case
`0x02`). The access-type table distinguishes `WO`/`SC` as read-0 and does not do so
for `W1C`, and "If a flag is set and cleared in the same cycle, it stays set" only
makes sense for a register whose value is observable.

**If ISS-A reads 0 instead:** it is a one-line change in `_read`. It affects only
software that polls `FDEBUG`, not engine behaviour.

## 2. The first `step()` is cycle 0, so a divider restart lands in cycle 2

**Question.** `## Conventions` says "Cycle 0 is the first core cycle in which `rst_n`
is sampled high. All state holds its reset value during cycle 0", and
`### Clock divider` says "A restart in cycle `h` gives `en = 0` in `h+1` and
`en = 1` in `h+2`". Nothing says whether the ISS's first `step()` *is* cycle 0 or
cycle 1.

**Reading taken.** `step()` after `reset()` runs cycle 0, so the state a caller can
first observe after one `step()` is still the reset state, and the anchor trace
(`### Anchor trace`) lines up with `h+2` as the first SM cycle of a `CTRL` write in
cycle `h`. A fuzz harness that counts cycles from 0 and one that counts from 1
would otherwise disagree by a cycle everywhere. `docs/mvp-interfaces.md` gives no
cycle numbering, so this is a harness convention, not a behaviour: both ISSs agree
if both are driven the same way.

## 3. `pin_out`/`pin_oe`/`irq0`/`irq1` report the *next* cycle's pad value

**Question.** The API doc says "Outputs reflect the state at the end of the cycle",
but `## Timing` says `T_OUT = 1`, "a pin write by an engine in cycle `n` is on the
pad in cycle `n+1`". After `step()` for cycle `n`, is `pin_out` the pad in cycle `n`
or in cycle `n+1`?

**Reading taken.** Cycle `n+1`: the attributes are the registered pad state at the end
of cycle `n`. This is the only reading under which `pin_out` and a `DBG_PADOUT` read
in the following cycle agree — the same register, read two ways
(`test_anchor_pin_reads_agree_with_pad_attributes` pins this down). The interrupt
pins are treated the same way, following "Both pins come from flops. The pin value in
cycle `n+1` is computed from the state in cycle `n`."

## 4. `IN_VEC` sees the pad of cycle `n-3`; the pipeline is three registers

**Question.** `T_IN = 3`, "an engine in cycle `n` sees the pad value from cycle
`n-3`", and the input path is "a 2-flop synchronizer and one more register".

**Reading taken.** The pad value presented in cycle `m` is in `IN_VEC` in cycle
`m + 3`, so the shift register holds `T_IN` slots and each `step()` pops the oldest
(`Chip.__init__`, `Chip.step`). The alternative reading — that the synchronizer chain
needs a fourth slot because the pad value only exists at the end of the presenting
cycle — would make it `m + 4` and would contradict the anchor trace's own sentence
("sees its own `uio[0]` level from cycle `h+4` in its `IN_VEC` from cycle `h+7`").
The anchor-trace test checks the 3-cycle delay independently.

## 5. `CTRL.SM_RESTART` wins over the engine's own update in the same cycle

**Question.** `### Enable, restart` says "If the engine also executed in cycle `h`,
the restart wins for those fields", and `### Reference step` step 3 has no notion of
a restart.

**Reading taken.** The restart is applied after the engine's updates, for exactly the
fields the ISA state table marks `SM_RESTART = 0` (`Engine.restart_state`). Note this
means a restart in the same cycle as a host `SMx_INSTR` write clears `force`, so the
forced instruction does not run — the spec lists `force` among the cleared fields, so
that follows directly.

## 6. A `CLKDIV` write is picked up at the next reload, using the cycle's own value

**Question.** `### Clock divider` says "A write to `CLKDIV` is picked up at the next
reload (`cnt == 1`), without a restart", and "Configuration writes in cycle `h` apply
from cycle `h+1`".

**Reading taken.** The reload in cycle `n` uses the `CLKDIV` value in effect *during*
cycle `n`, so a write in the reload cycle `h` takes effect at the *following* reload
(`Chip._divider` runs before `_commit` applies the config write). This follows the
general rule that decisions in a cycle use that cycle's state. The observable window
is one reload period wide and only matters if the host writes `CLKDIV` in exactly the
reload cycle.

## 7. `latch_host` is sticky across a stall, and only `OUT EXEC`/`MOV EXEC` clear it

**Question.** `### Reference step` step 3 sets
`latch_host = (src == HOST) or latch_host` when a latched instruction stalls, and
`not (src != IMEM and latch_host)` gates the delay load. `### EXEC` says `latch_host`
"is cleared when OUT EXEC or MOV EXEC writes the latch". Nothing says what clears it
when an *IMEM* instruction stalls with no host involvement.

**Reading taken.** Only the two EXEC forms clear it (`_execute`, destinations
`OUT_EXEC` and `MOV_EXEC`), because that is the only rule the spec states. So an
IMEM instruction that stalls with `latch_host` still 0 behaves normally, and a
host-written instruction that stalls keeps `latch_host = 1` until an EXEC overwrites
the latch. `state()` does not expose `latch_host`; it is visible through
`EXECCTRL.EXEC_STALLED`, which the tests use.

## 8. An IN that stalls on autopush does not shift, and the *next* attempt re-reads

**Question.** `### IN`: "**MVP:** the stall is atomic. A stalled IN shifts nothing,
and it samples its source again in the cycle where it completes."

**Reading taken.** Literal: the source (`X`, `Y`, `IN_VEC`, `OSR`, `ISR`) is read at
the start of every attempt, including the completing one, and `isr`/`isr_cnt` are
untouched by a stalled attempt. `isr_cnt` in particular keeps its pre-stall value, so
the completing attempt still reaches the autopush threshold
(`test_in_autopush_stalls_on_a_full_rx_fifo_and_is_atomic`).

## 9. `PULL` with autopull and `osr_cnt == 0` is a no-op even when the FIFO has data

**Question.** `### PULL`: "If `AUTOPULL` and `osr_cnt == 0`: no-op (the OSR is full,
so PULL is a barrier)", and `## Differences` item 15 repeats that this follows the
datasheet text literally. `PULL ifempty` below would also skip.

**Reading taken.** The barrier is checked first and returns immediately, so neither
the FIFO nor the threshold is examined
(`test_pull_with_autopull_is_a_noop_only_when_the_osr_is_full`). `osr_cnt == 0` is the
"OSR is full" case: after a PULL or a refill.

## 10. The IRQ instruction's `wait` form does not stall

**Question.** `### Removed and reserved encodings` item 1: "**IRQ (opcode `110`), all
forms** (set, clear, `wait`): no main effect and never stalls, because `irq wait`
waits for a flag that is never raised." The clause "never stalls" and the reason
given ("waits for a flag that is never raised") pull in opposite directions: the
reason would imply a permanent stall, as it does for `wait 1 irq` (item 1's next
bullet, and the stall table).

**Reading taken.** The explicit statement wins: opcode `110` in all forms is a NOP
that never stalls (`_execute_4567` has no `op == 6` case). The reason clause reads as
an explanation of why no flag ever arrives *and* therefore no IRQ state ever changes,
rather than as a stall rule. `## Differences` item 3 ("the IRQ instruction is a NOP")
supports this, and it names `wait 1 irq` separately as the stalling form.
`test_irq_wait_is_a_nop_like_every_other_irq_form` pins it; if ISS-A stalls, this is
the difference.

## 11. Reserved field values advance the PC and apply side-set and delay

**Question.** `### Removed and reserved encodings` item 2: "The instruction executes
as a NOP. It has no main effect, never stalls, and advances the PC as a non-jump
would. Side-set and delay apply as normal."

**Reading taken.** Exactly that, including the delay load: the decoder returns a
no-stall, no-effect, no-new-PC result and the shared bookkeeping loads
`delay_field(I)` and advances the PC. So a reserved encoding with a delay field
delays the next instruction
(`test_mov_reserved_field_values_are_nops`, `test_set_reserved_destinations_are_nops`).
The differential fuzz found that the code did *not* match this reading for `MOV EXEC`
with a reserved Op or Source: `_is_exec` looked at the Destination alone, so such a NOP
skipped its own delay field (and kept a latch armed forever if it came from the latch).
Fixed, with `test_a_reserved_mov_exec_is_a_nop_and_not_an_exec_er`.

## 12. `SET_COUNT` of 6 or 7 writes zeros; `OUT_COUNT` above 32 behaves as 32

**Question.** `### SET`: "**MVP:** `SET_COUNT` of 6 or 7 writes zeros to the extra
pins", and `PINCTRL` allows `SET_COUNT` 0..7 with "reset 5" and no wrap note, while
`OUT_COUNT` is 6 bits and "values above 32 behave as 32".

**Reading taken.** `SET_COUNT` is used as-is up to 7, with the data bits above 4
already zero (`window` builds the mask per pin from the data). `OUT_COUNT` is clamped
to 32 with `min(eng.out_count, 32)`. `SIDESET_COUNT` 6 and 7 become 5 via
`sideset_count()`, and the window it opens is `SC` pins with `SIDE_EN = 0` and `SC - 1`
with `SIDE_EN = 1` (item 19).

## 13. Output-only pins read back their driven level after `T_IN`

**Question.** `## Pins`: "for 13..17 the chip ignores it and feeds back its own driven
level, as the spec says", and "an output-only pin feeds the same pipeline from its
driven value, so an engine reads back an output-only pin after the same 4-cycle loop".

**Reading taken.** `Chip.step` builds the pad vector for the input pipeline as
`(pin_in & 0x1FFF) | (lvl & 0x3E000)`: the caller's 18 bits for pins 0..12, and the
current level register for 13..17. The level register at the *start* of the cycle is
used, which is the value on the pad in that cycle, so the loop is
`T_OUT + T_IN = 4` cycles. `DBG_PADOUT`/`pin_out` mask those pins to 18 bits.

## 14. Host and engine FIFO accesses in the same cycle

**Question.** `### Host-to-engine timing`: "In the same cycle, the host and the engine
each see the FIFO's start-of-cycle state: a host push to a full FIFO is dropped
(`TXOVER`) even if the engine pops in that cycle; an engine cannot pop a word the host
pushes in that cycle; a host read of an empty RX FIFO returns 0 (`RXUNDER`) even if
the engine pushes in that cycle. Otherwise, a push and a pop in the same cycle both
happen."

**Reading taken.** All four clauses are implemented in `Chip._commit`, in this order:
pop (engine), pop (host read), push (host, if it was accepted), push (engine). The
engine's pop and the host's push therefore never mix, and the host's pop takes the
word that was in the FIFO at the start of the cycle, not one the engine pushed in it
(`test_push_fifo_is_four_deep_and_keeps_its_order` exercises the ordering).

## 15. `FDEBUG`: an engine flag set in a cycle survives a host clear in that cycle

**Question.** `### Host-to-engine timing`: "If the host and an engine update the same
state in the same cycle, the host wins. The one exception is `FDEBUG`: a flag set by
an engine in a cycle stays set even if the host clears it in that cycle." And
`FDEBUG`: "If a flag is set and cleared in the same cycle, it stays set."

**Reading taken.** `fdebug` is recomputed as `(old & ~cleared) | set_this_cycle`, where
`set_this_cycle` covers both engine flags and the host's own `TXOVER`/`RXUNDER`
(`Chip._commit`).

## 16. `CTRL` reads back only `SM_ENABLE`

**Question.** `CTRL` is `RW/SC`: bits 9:8 and 5:4 are self-clearing. The access-type
list says `SC` registers "read 0", which covers the two restart bits.

**Reading taken.** A read returns `SM_ENABLE & 3` and 0 for the rest
(`_read`, case `0x00`). The alternative — storing the restart bits and reading them
back — contradicts the `SC` row of the access-type table.

## 17. `DBG_CFGINFO` is a constant `0x0020_0204`

**Question.** None really: the regmap gives the value and its fields
(`IMEM_SIZE = 32`, `SM_COUNT = 2`, `FIFO_DEPTH = 4`), and it is RO with no reset
behaviour of its own.

**Reading taken.** The constant is returned, with the three fields spelled out in the
test so a wrong bit position is caught
(`test_regmap.py::test_dbg_cfginfo_fields`).

## 18. Irregular instructions the spec does not pin down

**Question.** The spec does not describe `IN`/`OUT`/`MOV` with a *reserved*
`Destination`/`Source` value reaching the destination logic, nor `IRQ` bit 7/6, nor
PUSH/PULL bits 4:0 beyond "ignored".

**Reading taken.** All reserved source/destination values return before any effect,
so the destination logic is never reached with an unhandled value; bits 4:0 of
PUSH/PULL are ignored by the decoder (the ISS never reads them); and `IRQ` bits 7:6
are decoded and then discarded, since all IRQ forms are NOPs. `MOV` destination
`011` is reserved per `### Field values` and is a NOP.

## 19. With `SIDE_EN = 1` the side-set window is `SC - 1` pins wide

**Question.** `### Delay/side-set field` says "If `EXECCTRL.SIDE_EN = 1` and `SC >= 1`:
the MSB of the side-set bits is an enable, and the remaining `SC - 1` bits are side-set
data" and then, unconditionally, "Side-set data bit 0 goes to pin `SIDESET_BASE`, bit
`k` to pin `(SIDESET_BASE + k) mod 32`". With `SC` side-set bits but only `SC - 1` data
bits, does the window cover `SC` pins (the last one written with 0) or `SC - 1`?

**Reading taken.** `SC - 1` pins. The pin at `SIDESET_BASE + SC - 1` is not written at
all, because with `SIDE_EN = 1` no data bit maps to it (`_sm_cycle` passes `sc - 1` as the
window count; with `SIDE_EN = 0` it passes `sc`). The spec's worked encoding
`pull side 1 [7]` with `.side_set 1 opt` (`SC = 2`) has one data bit, so this is the only
reading under which `SC` counts the enable and not a pin. This was a bug found by the
differential fuzz
(`test_side_en_leaves_only_sc_minus_one_pins_in_the_side_set_window`).

## 20. `TXSTALL` is only for the *blocking* `PULL` on an empty TX FIFO

**Question.** `### PULL` puts "**stall** and set `TXSTALL`" in the Block branch and
nothing in the No Block branch, while the regmap's `**FDEBUG**` row defines TXSTALL as
"the engine stalls on an empty TX FIFO: blocking PULL, or OUT with autopull". Both say
the same thing, but the RP2040 is known to raise the flag for the non-blocking form too,
so it is worth writing down which reading this ISS implements.

**Reading taken.** The spec's: a non-blocking `PULL` on an empty TX FIFO copies X into the
OSR, sets no flag and does not stall (`_execute_4567`). Note the asymmetry with the RX
side, which the spec does spell out: a non-blocking `PUSH` on a full RX FIFO *does* set
`RXSTALL`. Also a bug found by the differential fuzz
(`test_pull_noblock_on_an_empty_fifo_copies_x`).

## Status after the differential fuzz

`verif/fuzz/iss_diff.py` ran this ISS against the golden one on 20 seeds x 10,000 core
cycles (and then 600 further seeds, plus the same 20 seeds at 100,000 cycles): every
cycle's `bus_rdata`, `pin_out`, `pin_oe`, `irq0`, `irq1` and all sixteen `Chip.state(e)`
keys now agree. The campaign executed every opcode (thousands of times each), 3,656
distinct instruction forms, 43,575 stall cycles and 26,078 host-forced cycles, so none
of the readings 1..18 above was contradicted; `verif/fuzz/iss-diff-triage.md` has the
full list of what was wrong and which spec line decided each one. Six bugs were found,
all in this ISS: `FLEVEL`'s field packing, the `SMe_CLKDIV` write mask, the side-set
window (19 above), a reserved `MOV EXEC` being treated as an EXEC-er against item 11's
reading, `TXSTALL` on a non-blocking `PULL` (20 above), and the two missing `state()`
keys `latch_host` and `forced`.
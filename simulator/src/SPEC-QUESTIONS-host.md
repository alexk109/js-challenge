# Spec questions: src/host_spi.sv

Every place `docs/mvp-regmap.md` `## SPI transaction format` left something open, what I
chose, and what the choice means for anyone writing a model or a testbench from the same
spec without seeing my code. The RTL marks each of these with `Qn`.

Read speed, INTE reset values and everything else in the spec's `## Open questions` I did
not need to decide: they are register-file or host questions, not host-port ones.

## Q1. Reset values of the input synchronizers

**Spec.** `sel = !cs_s2`, `rise = sel & sck_s2 & !sck_d`, with `CSn`, `SCK` and `MOSI` each
behind `s1`, `s2`. It gives no reset values for them.

**Choice.** `cs_s1`, `cs_s2` reset to 1, `sck_*` and `mosi_*` to 0.

**Why.** CSn high is "idle", so the port comes out of reset idle rather than looking
selected until CSn has propagated. For any legal host the choice is invisible: the two
flops cost the same two cycles of `sel` either way, and CSn is high between transactions.

## Q2. The turnaround byte is a phase of its own

**Spec.** "In each RD-phase cycle with a `rise`, `miso_sr` shifts left by one at the end of
that cycle. Elsewhere it holds." And: "During a TA byte, it shows bit 31 of the word being
read once that word is loaded."

**Choice.** Four phases: `CMD`, `WR`, `TA`, `RD`. The 8 TA bits are counted separately
from the 32 data bits, so MISO holds through the byte (showing bit 31) and shifts only in
`RD`.

**Why.** If TA were the first 8 bits of `RD`, MISO would shift 8 times during the
turnaround and the host would get `word << 8`. The spec's "elsewhere it holds" only makes
sense with the byte counted apart.

## Q3. When the burst address increments

**Spec.** "After each word access, the address increments by 1, wrapping mod 128."

**Choice.** At the end of the access cycle `h`, together with that access's other side
effects, for writes and reads alike. A read therefore increments at the first TA bit of the
word, not after its last data bit.

**Why.** `h` is the cycle the spec says side effects belong to, and the address for the
next access is then stable for the whole of the next TA byte and word.

## Q4. What the idle reset clears

**Spec.** "While `sel = 0`, the port is idle and is reset at the end of the cycle: phase
CMD, bit counter 0, MISO shift register 0." Three items are named.

**Choice.** Also cleared: the command-byte and write-word shift registers, the address
counter, the write-data register and the pending-access pulse.

**Why.** Only the three named ones are visible on the pins. The rest are dead until a
command byte arrives, so this cannot be observed from outside; it just means a transaction
always starts from a defined state.

Note the consequence for MISO: it goes 0 at the end of the first cycle in which `sel` is 0,
i.e. **three** core cycles after CSn rises at the pad (two through `cs_s2`, one for the
clear), not immediately.

## Q5. MISO between read words

**Spec.** "It is 0 while the port is idle, during CMD and during writes."

**Choice.** The shift register is cleared at the end of every RD word's 32nd shift (zeros
shifted in), so MISO reads 0 after a word until the next word is loaded at the end of its
`h`. During the TA byte the samples are therefore: the *previous* content for the first TA
bit, bit 31 of the new word from the second TA bit on.

**Why.** Deterministic, which is all the spec asks for ("undefined-but-deterministic"), and
the load lands 4 core cycles after the first TA bit's pad edge, so at the guaranteed read
speed (SCK period 8) it is there from the second TA sample.

## Q6. Shift register widths: the arriving bit is combined in combinationally

**Spec.** Silent.

**Choice.** `cmd_sr_q` is 7 bits and `wdata_sr_q` is 31 bits; the bit arriving this cycle is
`mosi_s2`, and `cmd_byte = {cmd_sr_q, mosi_s2}` / `wdata_next = {wdata_sr_q, mosi_s2}`
complete the byte/word. The registers therefore hold the value *before* the last bit, and
the last bit is taken combinationally.

**Why.** The write data for cycle `h` must be complete at the end of cycle `n`, one cycle
before `h`, so the 32nd bit cannot wait for a register round trip. (It is also what keeps
`verilator -Wall` quiet: with full-width registers their top bits would be written and
never read.)

## Q7. The burst address increments even when the access does nothing

**Spec.** "Unmapped or reserved addresses read as 0 and ignore writes. There is no error
signal." and "After each word access, the address increments by 1."

**Choice.** The increment is unconditional. A write burst at a reserved address walks the
address like any other; a read burst at a reserved address returns 0 for each word with
addresses stepping.

**Why.** The port cannot know what the register file did with the access, and "after each
word access" is unconditional. A test that wants writes to be dropped should check the
register, not the address.

## Q8. CSn going high between cycle `n` and cycle `h`

**Spec.** "the last SCK rise to CSn rise" must last at least 2 core cycles, and the access
happens in `h = n + 1` regardless.

**Choice.** The idle reset of Q4 includes the pending-access pulse, so an access is only
ever put on the bus in a cycle that is still selected.

**Why.** With the 2-cycle hold rule, `sel` is still 1 in `h` (CSn needs 3 core cycles to
reach `sel`), so this never fires for a legal host; it only pins down what an illegal one
does (the access is dropped, like a partial word).

## Q9. The `default:` arm of the phase `case`

Trivial: all four `phase_e` values are handled, and the empty `default` keeps tools that
want a full case from complaining. No behaviour.

# Framing assumptions (no `Q` number: nothing in the RTL depends on a choice)

- A read transaction is decoded strictly as `CMD | TA | word | TA | word | ...`, one read per
  turnaround byte, as the spec's frame says. A host that instead sent 32 MOSI bytes after a
  read CMD would have its first 8 bits eaten as a turnaround (the read fires 8 bits early)
  and the remaining 24 clocked as data. There is no other plausible reading.
- Bytes after the last word of a write are more words: bursts have no length limit.
- Exactly one bus access per word. No read-ahead, no write buffering, no retry: `bus_we` and
  `bus_re` are one-cycle pulses and never both high.
- `bus_addr` always holds the address the *next* access will use. `bus_wdata` holds 0 for a
  read transaction and the last written word afterwards; neither matters when `bus_we` is 0.
- Nothing advances without a `rise`, so a host may pause mid-byte or mid-word indefinitely.

# Cycle-exact numbers, as implemented (T = 20 ns core cycle)

Useful for anyone building a model or a bench from the spec:

- A pad rising edge in cycle `t` is seen by the port in cycle `t + 2`: `sck_s2` is high in
  `t + 2` and `sck_d` is still low.
- MISO changes at the end of that cycle, so the new bit is on the pin from `t + 3`: 3T after
  the edge, which is the "at most 3 core cycles plus the pad delay" the spec promises.
- A write scheduled at `n` is on the bus in `h = n + 1`; a read likewise.
- A read's word is on MISO from `n + 4` (detection `n`, access `n + 1`, load at the end of
  `h`), i.e. 4T after the first turnaround bit's pad edge.
- Fastest legal rates: writes are good at an SCK period of 4 core cycles (f_clk/4) and reads
  are good at 4 as well, because the host samples 4T after the edge and MISO moved at 3T.
  The spec only *guarantees* reads at a period of 8 (f_clk/8); 4 is the "faster reads are
  allowed if the host's setup margin covers 3T + t_pad" case. Both were simulated, at period
  4 and period 8.

# What I did not check

- Power-up while the host holds CSn low. The port follows `sel = !cs_s2` and would decode a
  transaction that starts during reset, but the spec's host timing rules assume a normal
  start, so I did not test it.
- CDC/metastability: the synchronizers are as specified (2 flops, plus `sck_d` on SCK), with
  no constraints and no MTBF analysis. `MOSI` is sampled from `mosi_s2`, not from a third
  flop, because the spec says the port samples `mosi_s2`.
- Synthesis and layout: no SDC, no Yosys, no LibreLane run for this file. `make rtl-lint` is
  an elaboration check only.
- Post-layout MISO timing (`t_pad`), which is what decides whether f_clk/4 reads really hold
  on silicon.


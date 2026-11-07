# What the 32-word IMEM does and does not leave room for

This file maps the MVP's limits for SWD (it would be a list of what cannot be expressed if SWD
could not be expressed at all).  SWD *is* expressible: `program.pio` is a working
SWD controller in 31 of the 32 words and it drives `verif.vip.swd.SwdTarget` through a
whole connection sequence with the checker reporting zero violations.  So this file is the map: what the word
budget bought, what it did not, and the three places where the MVP has no instruction and
the firmware had to be written around it.

Every number here is measured by `python -m fw.swd.measure`.

## The budget, line by line

| Words | What | Why it cannot be smaller |
|---|---|---|
| 3 | `xfer`: `pull block`, `set x, 8`, `jmp tx` | the entry point; a host-forced JMP is the only way to set the PC (`SMx_ADDR` is read-only) |
| 3 | `tx` loop: `out pindirs, 1`, `nop side 1`, `jmp x-- tx` | the transmit bit period |
| 2 | `e1`: `jmp !osre wack`, `push iffull` | the two exits of `tx` have to be told apart, and the receive loop's exit has to land on the push |
| 7 | `p2`: `pull block` + three `out`/`nop side 1` pairs | three closing bit periods: the RDATA parity, the DP's turnaround, and the pull-down that resynchronises it |
| 1 | `done`: `jmp done [15]` | the idle spin; the engine has to end up somewhere that clocks nothing |
| 2 | `wack`: `set x, 1 [LOW - 1]`, `jmp ck` | two ACK periods, plus the padding that keeps the phase boundary a whole bit period |
| 3 | `ck` loop: `nop side 1`, `nop [LOW]`, `jmp x-- ck` | the ACK bit period, which touches neither line |
| 3 | `e2` + the read path: `jmp pin wpre`, `set x, 31` | the host's read/write flag decides which data phase runs, and `jmp pin` is the only conditional that had a register free |
| 3 | `rx` loop: `nop side 1`, `in pins, 1`, `jmp x-- rx` | the receive bit period |
| 5 | `wpre`: `nop side 1`, `nop [LOW]`, `pull block`, `set x, 31`, `jmp tx` | the ACK/WDATA turnaround and the 32 WDATA bits |
| **31** | | one word spare |

## The three limits that cost the design

### 1. `set x, n` is n + 1 loop bodies, and n is 5 bits

`jmp x--` tests X *before* it decrements, and the loop body runs before the test, so a
counter of n gives n + 1 iterations (fw/swd/test_swd.py `test_retarget_only_moves_the_
delay_field` pins the related facts, and every run length in the program is written as
"one below the number of bit periods").  A 32-bit data phase is exactly `set x, 31` --
one below the 32 that `set`'s data field cannot hold.

**What it cost:** the WDATA phase cannot be 33 periods in one run, so bit 31 and the
parity bit are shifted out by the two-word `p2` block instead, and the closing word's
layout differs between a read and a write.  A 50-clock line reset cannot be one run
either, which is why `line_reset()` runs two `xfer` passes (see 3).

### 2. A bit period is three instructions plus two delays, so a 32-bit phase costs 3 words

This is the one that makes the whole thing fit, and it is the reason the timing constants
live where they do.  A loop body is `level change`, `clock high`, `jmp` -- three
instructions -- and the delays carry the rest of the period, so **one bit period costs 2
words instead of the 32 an unrolled frame would need**.  A 46-period frame written out
longhand would be about 92 words; at three words per loop body it is 31.

The price is that the period is `3 + HIGH + LOW` core cycles with **CLKDIV = 1.0**.  There
is no divider in the picture: the delays are the whole period.  That also means the
fastest SWCLK is 50 MHz / (3 + 6) = **5.56 MHz**, and HIGH cannot go below 6 because the
DP's answer is on the pad two ticks after the rising edge while this chip reads SWDIO
`T_IN` = 3 ticks after the pad (docs/mvp-isa.md `## Timing`).  The slowest is
50 MHz / (3 + 14 + 14) = **1.67 MHz**, set by the 4-bit delay field that `.side_set 1`
leaves.

### 3. No IRQ, and only two engines' worth of registers, so the host owns the decisions

There is no way for the engine to ask the host anything, and no free register to park a
request in: X is the run counter, Y the mode bit inside the loop, ISR the received word
and OSR the word being sent.  Three things fell out of that:

* **The read/write flag is a pin.**  `jmp pin` reads `EXECCTRL.JMP_PIN`, and the host
  writes FLAG with a forced `set pins, 0/1` before it jumps to `xfer`.  One word
  (`jmp pin wpre`) and one spare pin; the alternative was a second loop.
* **The request parity and the WDATA parity are computed by the host.**  A 32-bit parity
  costs about six PIO instructions per bit (`mov y, isr` / `mov x, isr` / `jmp x != y` /
  `set x, 1` / `jmp` / `set x, 0`), which is 192 words for one bit -- more than the whole
  IMEM.  The host pushes the parity in the closing word, which is what a real probe's CPU
  does, and the checker judges the wire either way.
* **The ACK is clocked but not sampled.**  `ck` runs three words and touches no line, so
  the chip never learns whether the DP answered OK.  `verif.vip.swd.SwdChecker` decodes
  the ACK from the wire, and that is what the tests assert on.  A controller that has to
  *act* on a WAIT or a FAULT -- retry, or fall back to a line reset -- has no path in
  this program.

## The line reset, and why it took two attempts

Fifty clocks with SWDIO high is a line reset [S12] and it is the first thing a probe does
[S13].  It does not fit: 50 is not `set x, n`, and a 50-word loop does not fit either.

The first version clocked it from the host, one `SMx_INSTR` write per core cycle: 50
periods × 10 core cycles = **500 forced host accesses** for one line reset, and it landed
34 clocks of the 50 before anything went wrong, which the checker reported as
`LINE-RESET 34 < 50`.  The version in `driver.py` uses `xfer` itself: a request word of
zeros leaves SWDIO released for every period `xfer` clocks (the header bits are all
"released", `ck` does not touch the line, `rx` only samples), which is 46 released
periods, and two passes with a pull-down at the end of the second are more than the 50 the
spec asks for.  Two host round trips, no extra program words.

## What is *not* in this firmware

* **No Access Port.**  The DP answers an AP access with OK and zeros (there is no AP in
  `verif.vip.swd.SwdTarget`), so an AP read comes back zero and an AP write is dropped.
  A real debug port needs MEM-AP registers, which is 32 more bits of state than the two
  engines have free.
* **No multi-drop.**  TARGET selection is a one-wire signal; the MVP's open drain can hold
  it low but nothing can read who else is on the bus, so a bus with two DP pads would need
  the arbitration the spec describes and there is nowhere to put it.
* **No sticky-flag handling in hardware.**  The chip can write ABORT (it is an ordinary
  register write, and `test_a_sticky_flag_survives_and_an_abort_write_clears_it` shows the
  DP's WDATAERR bit going up and being cleared) but it never reads a sticky bit and
  decides anything from it.
* **One frame at a time, no pipelining.**  The driver queues a whole transfer, waits for
  the engine to park, and reads one RX word.  A 33-bit phase arriving while the previous
  one is still being shifted out would need a second shift register, and the ISR is the
  only one there is.
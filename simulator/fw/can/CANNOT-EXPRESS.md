# What the MVP cannot do for CAN, and what it does instead

Every number here is measured and
re-derivable: `python -m fw.can.measure` writes `metrics/fw-can-speed.csv`,
`python -m fw.can.limits` writes `metrics/fw-can-limits.csv`, and
`make fw PROTO=can TARGET=iss` is the test that judges the rest.

The short version: **CAN's bit layer is expressible in this ISA, and the rest of CAN is
host software here.** `fw/can/program.pio` is five words and does the bit layer -- one bit per
bit time on one open-drain line, at exactly the nominal bit time, read back in the same bit
time. The frame layer (the 15-bit CRC, bit stuffing, the field order, the INTERMISSION) is
host software in `fw/can/driver.py`. Two things about the rest of the protocol are now
*counted* rather than asserted (§1 and §2): a receiver-side stuff-bit detector does fit, in
20 of the 32 IMEM words and 25 instructions per bit time, and runs at 1 Mbit/s
(measured), while the CRC-15's feedback does not
fit in 32 words however slowly the engine is clocked. What this program does not do, and does
not claim cannot be done, is the rest: the hardware CRC, the ACK, arbitration, the ERROR
FRAME and the whole of sec. 7 fault confinement. So the chip is the **transmitter**; the
receiver in every test is the model's `CanNode`, and the judge is `CanChecker`.

## What does work, and what it is worth

| | measured |
|---|---|
| IMEM words used | **5 of 32** (`python -m fw.can.limits`) |
| SM cycles per bit | 10 (`out pindirs, 1 [7]` / `in pins, 1 [0]` / `jmp engine`) |
| fastest bit time expressible | 10 core cycles = 200 ns = **5 Mbit/s** at 50 MHz, CLKDIV 1 |
| classic-CAN ceiling | 1 Mbit/s = 50 core cycles, exactly reachable at CLKDIV 5 |
| unreachable bit time | 25 core cycles (2 Mbit/s): not a multiple of the loop's 10 SM cycles |
| checker violations | zero at every bit time from 10 to 4000 core cycles (`metrics/fw-can-speed.csv`) |
| longest frame | 125 wire bits with eight data bytes and the worst stuffing of any identifier |
| TX FIFO capacity | 128 bits, so the longest frames need the host to keep feeding |

Two speeds are tested as configurations rather than as one speed with a tolerance:
`fw/can/test_can.py` runs 125 kbit/s (CLKDIV 40) and 500 kbit/s (CLKDIV 10) from the same
five words, plus a third configuration with the windows on pin 4 and engine 1, plus every
data length code 0..8, a REMOTE FRAME, four back-to-back frames, and a bit-time sweep.

## The five things the PIO cannot do, and why

### 1. The CRC-15, and therefore the acknowledgement

`01-bosch-frame-fields-coding.txt` sec. 3.1.1 defines the CRC FIELD as a 15-bit shift
register over SOF..end of DATA FIELD, with the generator
X^15 + X^14 + X^10 + X^8 + X^7 + X^4 + X^3 + 1 and the pseudo-code
`CRCNXT := NXTBIT XOR CRC(14); shift; if CRCNXT then CRC := CRC XOR 0x4599`.

That is an XOR with a feedback bit, and **the MVP instruction set has no XOR**. MOV
(docs/mvp-isa.md `### Field values`, **MOV Destination** / **Op**) offers a copy, an invert
and a bit-reverse; SET writes a 5-bit
immediate; IN and OUT shift. Nothing in the set combines two registers, and there is no
table to look a remainder up in either: IMEM is fetch-only -- "The host can write it and
cannot read it" (docs/mvp-isa.md `### Engine state`) -- and there is no other memory in the
chip. (The generator as a register polynomial is `0x4599`;
research/ref/protocols/can/README.md.)

So the spec's arithmetic is not one instruction away. **What excludes it is the budget, not
the missing instruction**, and the budget excludes the bit-injection loop as much as the
XOR. Here is the count, because the count is the argument:

* A CRC-15 step is `crc = (crc << 1) & 0x7FFF`, then, if bit 14 of the shifted value XORed
  with the next data bit is 1, `crc ^= 0x4599` -- 15 bits of feedback with seven runs of
  ones (bits 14, 10, 7, 5, 4, 3, 0).
* **No instruction in this ISA can test a single bit of a register.** X and Y cannot be
  shifted at all, and `in null, k` moves one end of the register to the other, so the
  cheapest test is on a contiguous *run* of bits at one end: `mov isr, <reg>` /
  `in null, k` / `mov x, isr` / `jmp !x, ...` is 4 instructions per run, and that sequence is
  counted and run (`python -m fw.can.limits`).
* So the conditional XOR is at least 4 instructions to test plus a branch and a load per
  run -- 6, say -- over at least five runs, i.e. **30+ instructions per data bit for the
  XOR alone**, against **32 words of IMEM** (`docs/mvp-isa.md` `### Engine state`), before
  the shift of the register, before the data bit itself has to be brought in
  (`jmp x != y` is the one inequality test there is, so the two bits have to sit in X and Y
  at once), and before the conditional inject of the CRC bit into the stream -- the same
  shape as the 7-instruction complement sequence counted in §2, because the CRC bit has to be in
  the OSR to be shifted out.

The IMEM word count is a hard limit and does not move with CLKDIV, so this is excluded at
every bit rate, not only at 5 Mbit/s. The instruction set alone does *not* rule out a
clever encoding of this -- the budget does, and the exclusion is a **budget judgement about
32 words and the counted instructions per bit, not a proof from the instruction set**. The
five-bit window test, once thought impossible, is demonstrably expressible (§2),
and so is a receiver-side stuff-bit detector; the CRC is the part that stays past counting.

The consequence is bigger than "the CRC is on the host". A CAN receiver may only
acknowledge a frame whose CRC it matched (sec. 3.1.1 ACK FIELD: "All stations having
received the matching CRC SEQUENCE report this within the ACK SLOT"). No hardware CRC means
no legal way to acknowledge, and no legal way to *receive*: the second reason is next.

### 2. Destuffing, and therefore where the ACK SLOT is

sec. 5: after five equal bits inside SOF..CRC SEQUENCE one complementary bit is inserted.
The ACK SLOT's position **on the wire** therefore depends on how many stuff bits were
inserted before it, which depends on the payload. A receiver cannot count its way to the
ACK SLOT; it has to destuff first, and destuffing needs the run length of the bits already
received -- a five-bit window tested against 0b00000 and 0b11111.

**That window can be extracted and tested, and it has been.** An earlier version of this
map said it could not ("there is no AND and no masking instruction ... so the window cannot
be extracted from the ISR at all"); a test program refuted that, and the four instructions it uses are named in
the ISA:

| step | instruction | what the ISA says |
|---|---|---|
| copy without consuming | `mov isr, osr` | **MOV** reads `OSR` as a *source* and writes a destination; the OSR keeps its bits (`docs/mvp-isa.md` `### Instruction semantics` -> **MOV**) |
| shift with zeros in | `in null, 27` | **IN Source** `011` is NULL (zeros): `isr' = (ISR << n) | 0` at `IN_SHIFTDIR = 0` (`### Field values`, `### Instruction semantics` -> **IN**) |
| put the window where a branch can see it | `mov x, isr` | **MOV Destination** `001` X |
| test it | `jmp !x, ...` | **JMP Condition** `001` `!X`: branch iff X == 0 |
| test the other value | `mov isr, osr !` before the shift | **Op** `01` invert, applied to the *copy* -- after the shift the low 27 bits are zeros, so `~window` could never be zero |

Nine instructions including the `pull`, four per test, no AND, no mask, no arithmetic --
and it runs: a test puts five
queued OSR words on `iss/pio_iss.Chip` and all five are decided correctly (0b00000 and all
zero decide `all_zero`, 0b11111 and all ones decide `all_ones`, 0b10101 decides `mixed`),
with the OSR still holding the word that was queued in every case. Re-run with `python -m fw.can.limits`.

**What is left over is the budget, and it has to be counted.** This program spends 10 SM
cycles per bit time (`program.pio`), which is the whole
instruction budget of the fastest expressible bit time: a bit time is `L` instructions plus
their delays in SM cycles, and one core cycle is `CLKDIV` SM cycles with `CLKDIV >= 1`
(`docs/mvp-isa.md` `## Timing`, `### Cycle accounting`), so a bit time of 10 core cycles --
10 SM cycles at CLKDIV 1, the 5 Mbit/s this program can reach -- admits exactly 10
instructions. Two things then narrow it, and both are register facts, not missing
instructions:

* **X and Y are the only working registers** (`docs/mvp-isa.md` `### Engine state`), and the
  only two that shift are the ISR and the OSR, which are tied to the IN and OUT windows.
  A receiver has three jobs at once -- sample the pin into the ISR, test the five-bit window
  (which needs the ISR as scratch, because X and Y cannot be shifted at all), and hand the
  destuffed stream to the host -- and there are two shift registers.
* **The only arithmetic in the set is JMP's `X--` / `Y--`.** A run-length counter can be
  loaded (`set y, 5`, **SET Destination** `010`) and decremented, and that is enough to
  count a run down to a stuff point; it cannot be incremented, and it cannot be compared
  with a constant, only with 0 (`!X`, `!Y`) or with the other register (`X != Y`).

So the exclusion is a **budget judgment, not a proof from the instruction set**, and the
window test is demonstrably expressible. The counted remainder changes what can be
claimed:
a complete receiver-side stuff-bit *detector* is **20 words of the 32 IMEM words and 25 SM
cycles per bit time on its longest path**, every path the same length so the bit time is
exact, and it runs at CLKDIV 2 -- 50 core cycles per bit, **1 Mbit/s** -- over the 130-bit
stuffed stream of the frame `fw/can/driver.py` itself builds, classifying all 111 bits after
the SOF in the stuffed region exactly as the driver stuffed them. In other words the budget
does **not** exclude a hardware stuff-bit detector at 125, 500 or 1000 kbit/s (25 divides
400, 100 and 50); what it excludes is the rest of a CAN node, and this round did not build
it.

Transmit-side stuffing fails for a different reason, and it is worth being precise about it,
because the obvious argument for it is wrong. `out pindirs, 1` does consume the OSR bit it
writes, but **looking at the next bit does not lose it**: `mov x, osr` reads the OSR as a
source and the OSR is only written when it is the *destination*
(docs/mvp-isa.md `### Instruction semantics` -> **MOV**), and a program that wants to put a
chosen bit on the line at a chosen cycle has one more route -- the SET window, whose
`SET_BASE`/`SET_COUNT` are independent fields of `SMe_PINCTRL` (docs/mvp-regmap.md
`**SMe_PINCTRL**` inside `### Per-engine registers`, `SET_COUNT` 28:26 / `SET_BASE` 9:5
against `OUT_COUNT` 25:20 / `OUT_BASE` 4:0), whose data is a 5-bit immediate
(docs/mvp-isa.md
`### Instruction semantics` -> **SET**: "PINS or PINDIRS: `Data`
zero-extended goes to the SET window"), and whose value a pin keeps until somebody writes it
again (docs/mvp-isa.md `### Pin writes` rule 3). `CanDriver.pinctrl_word` already points
both windows at the CAN pin, so the shipped firmware demonstrates the mechanism.

What makes transmit-side stuffing harder than the receiver half is this. First, **the SET
window's resolution
against an `out` in the same cycle is not specified**. `docs/mvp-isa.md` `### Pin writes`
resolves two things -- the side-set channel against the OUT/SET channel, and engine 1
against engine 0 -- and says nothing about an OUT-window write against a SET-window write to
the same pin in the same core cycle. SET *can* drive a complement in the middle of a word's
stream, and `program.pio:126` (`set pindirs, 0`) is that mechanism in the shipped firmware;
what a program cannot do is *rely* on it: with the resolution undefined, it cannot assume
the SET write overrides the `out` of the cycle the stream is running in, so it would have to
skip that `out` and spend the delay somewhere else to keep the bit time exact. Second,
**the complement bit has to be in the OSR**: `set pindirs, imm` is a 5-bit immediate and
cannot depend on data, so the only data-dependent write to an open-drain direction in this
ISA is `out pindirs, n`, which spends OSR bits. Building the complement and loading it costs
**7 instructions** (`mov isr, y !` / `in null, 31` / `mov x, isr` / `mov isr, null` /
`in x, 1` / `mov osr, isr` / `out pindirs, 1`) -- and `mov osr, isr` overwrites
the whole OSR, so the unsent remainder has to be kept in X or Y and shifted back in through
the ISR, several instructions per bit. Third, **the budget**: a bit time admits as many
instructions as it has core cycles (`docs/mvp-isa.md` `## Timing`, `### Cycle accounting`),
which is 10 at the fastest bit time this program reaches (10 SM cycles per bit at CLKDIV 1,
`program.pio`) and 50 at 1 Mbit/s, out of 32 words of IMEM;
the window test alone is 8 instructions (§1) and the complement write 7, so a
stuffer is over 10 per bit time at 5 Mbit/s and near 20 per bit time at the classic rates,
which is inside the budget there and inside 32 words. **So this map no longer claims
transmit-side hardware stuffing does not fit**: the exclusion is a budget judgement about
32 words and the counted instructions per bit time, not a proof from the instruction set,
the window test is demonstrably expressible, and the transmitter-side stuffer is simply the
part not built here -- a sender has to *write* a bit the receiver only has to
recognise, and it has to keep the unsent bits somewhere while it does it.

### 3. Receiving, at all

Everything above says the chip cannot acknowledge, so it cannot be a CAN *target*. It can
still capture the wire: `in pins, 1` reads the line and the ISR shifts it into the RX
FIFO, which is a bit-level monitor and would let a host decode a frame. That is a
deliberately narrow reading of "receive" and it is *not* what this firmware does, because
such a capture is not a CAN node: the destuffing is host software here (§2 shows a hardware
stuff-bit detector would fit, so this is a choice rather than a wall), it cannot check the
CRC and it cannot acknowledge, so on a real bus the transmitter would see an ACKNOWLEDGMENT
ERROR every time
and go to bus-off after 256 of them (sec. 7 rule 3). `fw/can/test_can.py` skips
`test_the_chip_cannot_receive_and_acknowledge` rather than pretending otherwise.

### 4. Arbitration (sec. 2)

A transmitter must compare every bit it sends with the bit it reads and, on losing,
withdraw without sending one more bit. The read-back words give the host the comparison
(`result.bit_errors`), so *detection* works -- but at the default `PUSH_THRESH` the ISR is
pushed only every 32 bits, i.e. the host learns about a lost arbitration up to 32 bit times
too late, and sec. 2 asks for a withdrawal within one bit.

This one is *nearly* expressible rather than impossible: `PUSH_THRESH = 1` would put every
sampled bit in the RX FIFO, and at 125 kbit/s a bit is 400 core cycles
(`metrics/fw-can-limits.csv`), which is enough for a host write to land inside the bit. It
would cost one RX FIFO word per bit, and the RX FIFO is four deep. It is not implemented
here because without the CRC there is no frame to arbitrate over, and because it is a
second program to justify rather than a fifth word to add.

### 5. Error handling and fault confinement (sec. 3.1.3, sec. 6, sec. 7)

This is the part that needs real state, and there is nowhere to put it:

* **TRANSMIT / RECEIVE ERROR COUNTs** are 8-bit saturating counters with thresholds at 128
  and 256, and both have to survive between frames and across engine resets
  (docs/mvp-isa.md `### Enable, restart` clears ISR and the delay counter on
  `SM_RESTART`). X and Y are 32 bits each and there is no register file beyond them.
* **bus-off recovery** needs 128 occurrences of 11 consecutive recessive bits (sec. 7
  rule 12) -- a counter to 1408 plus a run-length detector, in the same two registers.
* **ERROR FLAG** (sec. 3.1.3) must be superposed on the bus within one bit of the error
  that caused it (sec. 6.2), and **ERROR DELIMITER** is eight recessive bits after it. Six
  dominant bits are easy to emit -- they are just more bits, and the program has the
  instructions for it -- but there is nothing in the PIO that can *notice* the error. The
  read-back word arrives 32 bits later, and a host that then forces an instruction is
  already outside the one-bit window at any speed above ~10 kbit/s.
* **Nothing can report it**: there are no IRQ flags (docs/mvp-isa.md
  `## Differences from RP2040` item 3 -- the IRQ instruction is a NOP and `wait 1 irq`
  never completes), so the host's only view of the engine is `SMx_ADDR`, the FIFO levels
  and FDEBUG. An error counter that cannot be read back cannot be a fault-confinement
  counter.

`fw/can/test_can.py` skips `test_fault_confinement_and_error_frames` for this reason.

## The measured limit of the part that does work

The bit-time granularity is the loop: `3 + D_OUT + D_IN` SM cycles per bit, and CLKDIV
cannot be finer than one core cycle per SM cycle. With the `.define`s in `program.pio`
(7 and 0) that is 10 core cycles, so:

* the fastest frame this program can put on the bus is 10 core cycles = **5 Mbit/s** --
  five times the 1 Mbit/s that classic CAN tops out at (`04-bosch-bit-timing.txt` sec. 1),
  and the checker accepts it with zero violations;
* 1 Mbit/s (50 cycles) is exactly reachable, at CLKDIV 5;
* 2 Mbit/s (25 cycles) is **not**, because 25 is not a whole number of 10.

`metrics/fw-can-speed.csv` is the sweep, one row per bit time from 10 to 4000 core cycles,
with what the checker said about each. A `PUSH_THRESH`-free variant that dropped the
read-back would have a two-instruction loop and could reach 25 cycles, at the cost of the
ACK check -- which is the trade this firmware makes the other way.

## What a CAN node in this ISA would have to look like

For the thesis: a CAN node cannot be built out of this PIO without either (a) some way to
combine two registers -- an XOR, an ALU, or the 32 IMEM words made readable, because the
CRC-15's feedback is what does not fit in 32 words -- or (b) moving the protocol to the host
and keeping only the bit engine, which is what `fw/can/` does, and which is a legitimate
soft-CAN architecture, but leaves the PIO responsible for one sixth of the transfer layer
(Bosch sec. 1's own split: the transfer layer is "message framing, arbitration,
acknowledgment, error detection and signalling, and fault confinement"; the bit timing is
the part that has to be exact in core cycles, and that is the part the PIO has).

Note what is *not* on that list: **readable data memory for a destuffing state machine is
not needed.** The five-bit window lives in one of the two scratch registers and the
detector fits in 20 words (§2, and the log behind it). The second option is the interesting
one for a thesis: a CAN node that does the framing in hardware needs the CRC, and the CRC is
cheap on any RISC-V core and does not fit here; the destuffing is the part this ISA nearly
does. What the MVP *does* have is the bit-time accuracy and the open-drain drive, which are
the parts an implementation gets wrong first.

## Files

| File | What it is |
|---|---|
| `fw/can/program.pio` | the five-word bit engine, with the pin windows and the ISA citations in its header |
| `fw/can/driver.py` | the register file, the frame builder transcribed from `research/ref/protocols/can/`, the SM-cycle-grid alignment and the FIFO streaming |
| `fw/can/test_can.py` | the bench and the tests; the judge is `verif/vip/can.py` and is never edited |
| `fw/can/measure.py` | the bit-time sweep behind `metrics/fw-can-speed.csv` |
| `fw/can/limits.py` | the table behind `metrics/fw-can-limits.csv` |
| `fw/can/README.md` | how to run it and what the results were |

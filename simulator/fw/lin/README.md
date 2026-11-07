# fw/lin -- MVP LIN 2.x commander firmware

MVP firmware for LIN, judged by the reference models in `verif/vip/lin.py`: `LinResponder`
decodes the wire the way a second node does -- it finds the break field, measures the
commander's bit period off the sync field, checks the protected identifier's parity and
publishes the checksum the identifier calls for -- and `LinChecker` decodes the same wire and
checks twelve rules against the spec.  Neither model is modified here; both are the judge, and
zero violations is the bar.

    make fw PROTO=lin TARGET=iss                 # the accept command: 41 tests, ~2 min
    make fw PROTO=lin TARGET=iss_b               # the same bench on the independent second ISS
    make fw PROTO=lin TARGET=rtl                 # the same bench on src/pio_chip.sv (cocotb)
    .venv/bin/python -m fw.lin.report            # the numbers below, measured
    .venv/bin/python -m fw.lin.report --csv      # and appended to metrics/fw.csv

## What is here

| file | what it is |
|---|---|
| `program.pio` | the whole commander: break field, break delimiter, and two byte-field loops that send any number of header and response fields straight out of the TX FIFO. 26 of the 32 words of one engine's IMEM. |
| `driver.py` | assembles the program, derives the frame's shape and the divisor from the words it produced, programs an `iss.pio_iss.ChipDevice`, and computes the protected identifier and the checksum. |
| `test_lin.py` | nine configurations on a `verif.vip.core.Bus` at 20 ns/tick with `LinResponder` and `LinChecker` on the wire, plus eleven negative controls that break the firmware on purpose. |
| `report.py` | the measured numbers next to the derived ones, and the three headline numbers. |

## The chip is the commander (LIN 1.x: the master)

The protocol leaves the chip's role open, and the choice is forced rather than
preferred.  A commander is a pure bit-bang transmitter: every field is a byte field whose bits
come out of the OSR in order, so the entire protocol layer is "put the next byte in the FIFO".
A responder needs three things `docs/mvp-isa.md` does not have:

* **Bit-rate recovery.** A responder has no clock: it measures the commander's period from the
  falling edges of the sync field.  The MVP's only clock is the 16.8 divider, which the host
  sets once and which cannot be changed by a state machine mid-frame, so a chip responder could
  only ever talk to a commander that runs at a rate the host already guessed.  That is a fixed
  rate, not a sync field.
* **A subscription decision.** A responder answers only the identifiers it subscribes to.  The
  JMP conditions are `!X`, `X--`, `!Y`, `Y--`, `X != Y`, one pin and `!OSRE`
  (`docs/mvp-isa.md` `## Encoding`): there is no way to test the six bits of a frame identifier
  against a table, and no way to index anything.
* **A checksum.** The response checksum is the inverted modulo-256 sum of up to eight data bytes
  and the PID, and **there is no ADD instruction in the ISA at all**.  A sum cannot be formed in
  a state machine, so the checksum has to be host code even on a node that can transmit.

So `make_pid()` and `lin_checksum()` are in `driver.py`, and `test_lin.py` pins them against
the checker's own copies and against the identifiers and checksums the LIN literature publishes.
That is a real limit of the MVP for LIN, not of this firmware: the commander role spends 26
words and needs no arithmetic.

## The wire: open drain, and why it is emulated with PINDIRS

LIN is one wire, open drain: 0 is dominant (driven), 1 is recessive (released, the pull-up holds
it up).  The MVP has no LIN transceiver and no OE inversion (`docs/mvp-isa.md`
`## Differences from RP2040` item 10), so the program follows the convention
`### Open drain` prescribes: **the pin's level register stays at its reset value 0 and the
program drives the direction**.  `side 1` pulls the line low, `side 2` releases it, and a data
bit comes out of the OSR through `out pindirs, 1`, which writes a direction.

Two consequences, both asserted by the tests:

* **The host inverts every byte it pushes.**  Direction 1 is dominant, so bit 0 of a pushed word
  is 1 for a zero data bit.  `wire_word()` is the one place that happens; the receiver undoes it,
  and `LinChecker` reading `a55a` back off the wire is what proves it did.
* **The level register is never written.**  `test_the_chip_is_left_clean_and_only_touches_its_own_pin`
  asserts `DBG_PADOUT` bit 0 is still 0 after a whole frame, so the chip can only ever pull the
  wire low.  That is also what leaves the door open for a responder later: a node that never
  releases the wire can never listen to it.

The alternative -- push-pull levels with `set pindirs, 1` once and `out pins, 1` per bit, the
shape `fw/uart_tx` uses, which needs no inverted data and leaves `SIDESET_COUNT = 1` so a
single `nop ... [15]` could be the whole break field -- is what a LIN transceiver's TXD pin
actually does (TXD high *is* recessive to a real TJA1021-style transceiver).  It was not used
because it cannot read the wire back, and a LIN node must be able to: that is how a commander
detects that somebody else drove a dominant bit where it sent a recessive one, and it is how
the idle bus is verified.  Both wirings produce the same waveform on this bus; only one of them
leaves the pin a node can also listen on.
## The program, word by word

`program.pio` assembles to **26 words**, one SM cycle per bit:

| words | what they are | bit times |
|---|---|---|
| 0-1 | break field: two `nop side 1` with delays 7 and 5 | 8 + 6 = **14** (spec floor 13) |
| 2 | break delimiter: `nop side 2 [1]` | **2** (floor 1) |
| 3-12 | header byte field: `pull block side 1`, eight `out pindirs, 1`, `jmp x-- hdr side 2` | **10** per field, two fields = 20 |
| 13 | `jmp y-- resp side 2 [1]`: publish or fall through | **2** of response space |
| 14 | `idle: jmp idle side 2` -- and the wrap target | -- |
| 15-24 | response byte field, identical | **10** per field |
| 25 | `jmp idle side 2` | -- |

So a header is **36 bit times** and a published frame with N data bytes is
`36 + 2 + 10(N+1)`.  The spec's `T_Frame_Nominal` is `34 + 10(N+1)`; the program is two bit
times over the header (the break is one bit time over its floor) and exactly on the response.
The checker measured 36.00 bit times of header and 0 violations at every rate
(see the table under Results).

Three design points worth naming:

* **`.side_set 1 opt pindirs`.**  `SIDESET_COUNT = 2` is one side-set *data* bit plus the `opt`
  enable, and the assembler writes that enable itself for any non-zero operand
  (`asm/pioasm.py`:468), so `side 1` is "enable, data 1" and `side 2` is "enable, data 0" --
  a side-set with a zero data bit.  An instruction with no `side` operand has an all-zero
  field, the enable is 0, and **no side-set happens at all**: that is what lets `out pindirs, 1`
  own the pin for a data bit, since side-set wins over the OUT channel within one cycle
  (`### Pin writes`).  `fw/uart_tx` cannot use `opt` (its start bit rides on the side-set
  channel) and `fw/i2c` uses two pins for the same reason.
* **The break costs two instructions.**  `SIDESET_COUNT = 2` leaves the delay field three bits
  wide, so an instruction is at most 8 SM cycles: the 13-bit-time break cannot be one
  instruction.  8 + 6 = 14 gives the spec floor plus one bit time of margin for the rounding of
  the bit period, and keeps the header well inside `tHEADER_MAX`'s 1.4 x 34 = 47.6.
* **The stop bit is the loop branch.**  `jmp x-- hdr side 2` is the field's stop bit (its
  side-set releases the line) *and* the counter decrement, so a byte field costs exactly ten SM
  cycles -- ten bit times, the nominal length -- and neither counter costs a bit time.

**One SM cycle per bit** means CLKDIV alone sets the bit rate, so every rate in
`verif.vip.lin.LIN_RATES` is one integer: 2500 ticks per bit at 20 kbit/s, 2604 + 43/256 at
19.2 kbit/s, 20833 + 85/256 at 2.4 kbit/s.  The 16.8 divider is exact on average
(`docs/mvp-isa.md` `### Clock divider`), and the checker measured every byte field at exactly
ten bit times, 2604 ticks at 19.2 kbit/s.

## The host protocol, per frame

`SMx_INSTR` is the only way to set the PC (`### Enable, restart`), so the host forces three
instructions and then fills the TX FIFO:

    set x, 1        two header byte fields: sync 0x55 and the protected identifier
    set y, N        response fields this node will publish: 0 = header only
    jmp frame       the break field
    TX FIFO         ~0x55, ~PID [, ~data..., ~checksum]

`X` and `Y` are 5-bit `set` immediates, and the two loops decrement them, so nothing about the
frame length is in IMEM: one program sends a 2-byte frame and an 8-byte frame.  A host that
wants to publish more than 31 fields cannot (`arm()` says so), which is the IMEM-era version of
a limit that does not exist at this frame size.

The counters are forced *before* the TX FIFO is filled and *before* `SM_ENABLE`, because a
host-forced cycle runs the background autopull (`### Reference step`) and a word popped then
would be left in the OSR by the `SM_RESTART` that follows.

## Results

`make fw PROTO=lin TARGET=iss`: 41 tests, 0 failures.  The same bench on
`verif/iss_b/pio_iss_b.py`, the second ISS written independently from `docs/mvp-isa.md` and `iss/pio_iss.py`:
41 tests, 0 failures.  The table below is the output of `python -m fw.lin.report`.

```
case                    rate    bit    div words   frame meas hdr meas bit viol bytes
header-only 19.2k      19.2k   2604   2604    26      36    36.00   2604.0    0     5
published 19.2k        19.2k   2604   2604    26     128    36.00   2604.0    0    11
published 9.6k          9.6k   5208   5208    26      88    36.00   5208.0    0     7
published 10k          10.0k   5000   5000    26      68    36.00   5000.0    0     5
published 20k          20.0k   2500   2500    26      68    36.00   2500.0    0     5
published 2.4k          2.4k  20833  20833    26      58    36.00  20833.0    0     4
diagnostic 0x3c        19.2k   2604   2604    26      68    36.00   2604.0    0     5
lin 1.3 0x11           19.2k   2604   2604    26      68    36.00   2604.0    0     5
engine 1 pin 5         19.2k   2604   2604    26      68    36.00   2604.0    0     5
```

`meas hdr` is the header the checker timed off the wire in bit times (36.00 = the program's
own 36), `meas bit` is the period it measured from the byte fields, and `viol` is the number of
rules fired.  **Zero in every row.**

The three headline numbers:

* **words used of 32**: **26**.  Break 2, delimiter 1, header byte field 10, publish branch 1,
  idle 2, response byte field 10, jump back to idle 1.
* **max speed on the ISS at 50 MHz**: **50 Mbit/s** as a transmitter -- one instruction per bit,
  CLKDIV 1.0, one core cycle per bit -- which is 2500x the fastest LIN rate and therefore not a
  LIN node at all.  The fastest *LIN* bit rate the same program puts on the wire is
  **20 kbit/s**, the top of `LIN_RATES`, and it needs no rounding (CLKDIV INT = 2500).  The
  slowest is 2.4 kbit/s at CLKDIV INT = 20833, a third of the way to the 16-bit maximum, so the
  whole rate table is reachable with one SM cycle per bit.
* **chip role**: **commander** (master).

## What the tests hold it to

Beyond the nine configurations above, `test_lin.py` asserts:

* **The responder's own account.**  It decoded the header (`frames_seen`), measured our bit
  period off the sync field to the tick (`bit_error_ticks == 0`), passed the parity test
  (`parity_errors == []`), and published the enhanced checksum the identifier calls for.  The
  chip's frame is judged by a node that knows nothing about it.
* **The frame's own length**, compared against `program_timing`'s numbers rather than numbers
  typed into the test, and `tFRAME_MIN` explicitly not fired.
* **The checker's rules individually**, by breaking each thing a firmware can get wrong: a break
  of 11 bit times (`tBREAK`), a break that is not driven at all (`tRESPONSE` on every byte
  field), a driven break delimiter (the checker cannot see this one -- the run merges with the
  break field into a legal 16-bit-time dominant run -- so the *responder* reading the wrong
  identifier is the evidence, which is why the bench judges with the model and the checker), a
  dominant stop bit (`tSTOP`), a missing data bit (`SYNC`), `out pins` instead of `out pindirs`
  (`SYNC`), a sync field that is not 0x55 (`SYNC`), a data bit one SM cycle long (`SYNC`, and
  *not* `tBIT`, because the checker re-arms its grid at every byte field), a checksum one bit
  out (`tCS`), `OUT_SHIFTDIR` the wrong way round (`SYNC`), and a frame 5 % slow (`tBIT`).
* **The chip itself**: `FDEBUG` clean (no word dropped, no read underflow, and no TXSTALL either
  -- a commander with a host behind it must not stall), the pin released when the frame ends, its
  level register never written, no window leaking onto another pin, the other engine at PC 0 with
  an empty FIFO, the IMEM equal to the assembled words, and no contention on the wire.
* **The driver refuses setups it cannot honour**: a pin PINDIRS cannot drive (8..17), engine 2,
  a rate faster than one core cycle per bit, an overfull preloaded FIFO, a Y that does not fit in
  five bits, a frame with 0 or 9 data bytes, a frame identifier that is not six bits, and eight
  mutations of `program.pio` that would make `program_timing` derive a rate for a program whose
  shape it cannot describe.

## Not verified here

* **The RTL** (`TARGET=rtl`) is the same bench against `src/pio_chip.sv`; it was
  not run for this write-up.  The 2.4 kbit/s case asks
  `targets.is_fast()` and is skipped on a slow target, because a frame there is 1.2 million
  ticks through cocotb's pipes.
* **The TT chip** (`TARGET=chip`) does not exist yet: there is no harness for it yet,
  and `fw/targets.py` fails loudly rather than skipping.
* **Real hardware timing.**  `T_OUT = 1` and `T_IN = 3` are the ISS's constants
  (`docs/mvp-isa.md` `### Timing`), and a LIN transceiver's rise and fall times, its dominant
  state timeout and the bus's own rise/fall are not in this model at all
  (`research/ref/protocols/lin/README.md`, judgement call 13).  Nothing here says what the wire
  looks like through a real transceiver.
* **A schedule table, the LDF and diagnostics.**  The host builds each frame from a
  `(frame_id, payload)` pair; the LDF and the schedule table are the *other* node's business in
  this bench, and the transport layer is out of scope (`verif/vip/lin.py`, judgement call 13).

## What a LIN responder would need, and what the MVP has

The protocol can be expressed: the commander role above fits in 26 words, needs no arithmetic
and is bit-exact at every rate in the table, so there is no separate `CANNOT-EXPRESS.md`.  But
the *other* role is the interesting half of the answer, so here is the map, with the instruction or the register each item is missing.

A responder that meets the spec's own requirements would need, in order:

1. **Bit-rate measurement from the sync field.**  It measures the period between the falling
   edges of `0x55` and then transmits at *that* rate (`verif/vip/lin.py` `LinResponder._sync_byte`).
   The MVP has one clock per engine, the 16.8 divider, and it is set by the host for the whole
   run: a state machine cannot measure a period with it, and there is no second time base.  What
   is missing is **an instruction that measures or divides a time interval** -- `docs/mvp-isa.md`
   `## Encoding` has none, and the divider's reload is host-only (`SMe_CLKDIV`).
2. **A subscription decision.**  A responder answers only identifiers it subscribes to, and the
   LDF that says which those are is a table.  JMP's conditions are `!X`, `X--`, `!Y`, `Y--`,
   `X != Y`, one pin, `!OSRE`: enough to count and to compare two registers, not to test six bits
   of a frame identifier against a table.  What is missing is **any memory the engine can index**
   -- the ISR and OSR are shift registers, and `IN`/`OUT` address pins, not RAM.
3. **The checksum.**  An inverted modulo-256 sum of up to eight data bytes and the PID.
   **There is no ADD instruction** (`docs/mvp-isa.md` `## Encoding`, and `## Removed and reserved
   encodings` lists no arithmetic either), so this cannot be formed in a state machine at all --
   not even with 32 words and a clever loop.  This is the hard one: it is arithmetic, not control
   flow.
4. **The parity bits of the protected identifier**, the same missing ADD (P0 is a five-bit XOR).
5. **Telling the host a byte has landed.**  The natural design is `push` into the RX FIFO and an
   IRQ.  The MVP has **no IRQ flags** (`docs/mvp-isa.md` item 3 of `## Differences from RP2040`):
   `irq` is a NOP and `wait 1 irq` stalls for ever, so the host has to poll `RXF` and `FSTAT`
   through a 4-deep FIFO, and a 8-data-byte frame does not fit in it.  What is missing is
   **an interrupt line and FIFO-join**, or at least a FIFO deep enough for one response.

Three of the five are instructions or memories the brief chose not to include, and the one that
is *not* fixable by adding hardware is the checksum.  A LIN responder on this MVP would need the
host to hand it the parity bits and the checksum as well as the bit rate -- at which point it is
a shift register with a host, which is what the commander above already is.

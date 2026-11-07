# What a serial flash does that this MVP PIO target cannot

The rule: if the role cannot be expressed in the MVP — 32 words of IMEM, 18 pins, the
timing limits in `docs/mvp-isa.md` — say exactly what is missing and give the measured limit of
the closest partial program. This is that file. The partial program is
[`program.pio`](program.pio) (nine words), it passes against `verif.vip.spi.SpiController` and
`verif.vip.spi.SpiMonitor` at `half_period` 5 and slower, and its measured limits are in
[`README.md`](README.md).

The short version: **the partial program is a byte-stream engine with a host-supplied array.** It
answers a real `0x9F` and a real `0x03` on the wire, byte for byte, at 5.00 MHz. What it does not
do is *decide* anything — every byte it shifts out was chosen by the host before the transaction
started.

## 1. It cannot decode the opcode

**Missing.** Any comparison of a received value against anything. `jmp` conditions are
`!X`, `X--`, `!Y`, `Y--`, `X!=Y`, `PIN` and `!OSRE` (`docs/mvp-isa.md` `### Field values`).
`PIN` tests one absolute pin, `EXECCTRL.JMP_PIN`. There is no "compare the ISR with a constant",
no add, no subtract, no shift by a variable amount.

**How close the program gets.** One bit of the opcode, on the bit that is on the MOSI pin at that
moment, once per bit period. `jmp pin` at the right place in the loop would separate `0x9F`
(MSB 1) from `0x03` (MSB 0) — and nothing else. Two opcodes that share an MSB are
indistinguishable, and a branch of this kind costs a word in the loop and one of the three
instruction slots a bit period has at `half_period` 5 (`test_one_more_instruction_on_the_launch_path_costs_one_half_period`
measures what one such word costs: one half period of rate).

**Why the response content is not in the chip.** The only 32-bit data sources the OSR can be
loaded from are `PULL` (the host's TX FIFO), `MOV` from X, Y, ISR or NULL, and `SET` (a 5-bit
immediate). X and Y can themselves be built from 5-bit `SET` immediates, `MOV` from pins, and
shift paths — so a 24-bit constant is *expressible* in principle, but only by spending most of
the 32 IMEM words on shifts, and the JEDEC ID would then be the only thing the device could ever
answer. With 32 words total, code wins.

## 2. It cannot index an array with a received address

**Missing.** Any use of a received value as an address, a pointer, or a data selector. The
closest the ISA comes is `jmp x--` / `jmp y--`, which use a register as a *loop counter* — so an
address could at best be a skip count over something already in a FIFO, which is bounded by the
FIFO's four words.

**How close the program gets.** The 24-bit address arrives whole and the host can read it back:
`in pins, 1` with `AUTOPUSH` and `PUSH_THRESH = 8` puts one received byte in one RX FIFO word,
and the host drains it over the register bus. Four addresses are tested, including `0xFFFFFF`
(`test_the_whole_24_bit_address_reaches_the_device`). What cannot be done with it is *acting* on
it inside the device, so the flash array is the host's: `driver.py` formats the bytes a flash
would have shifted out for the address the caller asked for, and queues one 32-bit word per byte
(`jedec_stream`, `read_stream`). `test_the_answer_really_comes_from_the_host` changes what the
host queues and checks the answer changes with it, because a firmware that emitted a fixed
pattern would otherwise pass every other test here.

## 3. It has no data memory at all

The whole data budget is X, Y, OSR and ISR — 128 bits — plus 2 × 4 FIFO words (32 bits each).
That is 16 bytes of *total* state, of which the program needs the OSR for the answer and the ISR
for the received byte, and none of it is addressable: there is no indexed load or store, and the
only ALU-ish operation is the counter decrement. A device that has to serve an array larger than
its own registers cannot be built here, which is why the MVP's device-mode story is "the host
holds the array and the PIO moves the bytes".

The same limit shows up as a cost rather than a wall: the byte-boundary `pull block` refills the
whole 32-bit OSR every eight bits, so **one byte per TX FIFO word** and three of every four bytes
of FIFO bandwidth go unused. A loop that refilled only when the OSR ran dry would use them, at
the cost of the `jmp !osre` byte counter — which is the trade this program makes to keep the
ceiling at 5.00 MHz.

## 4. The host must stay a word ahead, and a miss is silent

The loop's two waits are *level* waits, so they detect edges only because the loop retires in
fewer core cycles than a bit period (five, against ten at the ceiling). If the host lets the TX
FIFO run dry, the boundary's `pull block` stalls; the phase is then lost for the rest of the
transaction, because the next `wait 1` completes on the level it already sees. `FDEBUG.TXSTALL`
records it, and every test asserts `FDEBUG == 0` — a stall that the transaction survived is a
latent failure, not a clean run. A real device does not have this property: it has its own array
and nobody has to feed it.

## 5. Mode 0 only

`docs/mvp-regmap.md` `### Physical layer` specifies mode 0 (CPOL 0, CPHA 0), and that is what
the program and the VIP runs use. The tests are mode 0; `Rig`
rejects any other mode rather than silently testing the wrong thing. For mode 3 the loop shape is
identical and only the two waits' polarities swap (`wait 1` becomes the launch, `wait 0` the
capture, because the master samples on the falling edge), so it is a two-character change — but
it is **not implemented and not measured here**, and the entry path (SCK idles high in mode 3, so
the launch wait at `idle` completes at once) would have to be checked rather than assumed.

## 6. No interrupt, so the host polls

The MVP has no IRQ (`docs/mvp-isa.md` `## Differences from RP2040`, item 3), so there is no
"byte ready" event for the host: it polls `FLEVEL` and reads `SMe_ADDR` to see where the engine
parked. `tx_ready` and `rx_valid` do exist as pins (IRQ1/IRQ0 through `docs/mvp-regmap.md`
`### tx_ready and rx_valid`) and this bench does not wire them; a host on the real chip could
poll them over the protocol pin instead of over SPI.

## 7. What is *not* missing

For the record, because these are the things a reader might expect to be on this list and they
are not:

* **The timing.** A device-mode answer inside the bit period is expressible: the measured ceiling
  is 5.00 MHz, `T_IN + 1 instruction + T_OUT`, with the decomposition measured off the chip.
* **The pin map.** The four lines can be any four distinct `uio` pins; five maps are tested. Only
  MISO has to be a `uio` pin, because its direction is host-driven.
* **Framing.** A JEDEC ID read and a READ with a 24-bit address and sequential data come back
  correctly, at any length from 1 to 255 bytes, across byte boundaries, with the received bytes
  readable by the host.
* **Parking.** The device stops driving MISO when the master releases CSn and releases the pin
  afterwards, the way a real flash sits high-Z between reads.
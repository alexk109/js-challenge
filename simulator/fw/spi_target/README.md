# `fw/spi_target` — an SPI NOR flash target (device mode) on the MVP PIO

The MVP's device-mode baseline. `program.pio` is nine words of IMEM that never drive SCK: they
watch it, answer on MISO on the falling edge, take MOSI in on the rising edge, and park when the
master releases CSn. Against `verif.vip.spi.SpiController` it serves **JEDEC ID `0x9F`** and
**READ `0x03`** (24-bit address, sequential data) in SPI mode 0, and `verif.vip.spi.SpiMonitor`
decodes the bytes off the wire.

The headline number, measured on the golden ISS at 50 MHz:

| | |
|---|---|
| **fastest SCK rate served with the right bytes and zero monitor violations** | **5.00 MHz** (10 core cycles per bit, `half_period` 5) |
| where the limit comes from | **T_IN = 3** (input synchroniser) + **1** SM cycle for the `out pins, 1` + **T_OUT = 1**, against the master's sample half a bit period later |
| IMEM used | **9 words of 32** |
| shipped test configuration | `half_period` 6 (4.17 MHz), the first rate with a core cycle of margin |

The sweep comes from `python -m fw.spi_target.measure`, and every number in it is asserted by
`test_spi_target.py`, which fails if the ceiling moves.

## How to run it

```sh
make fw PROTO=spi_target TARGET=iss        # the golden ISS (the accept command)
make fw PROTO=spi_target TARGET=iss_b      # the independent second ISS
make fw PROTO=spi_target TARGET=rtl        # src/pio_chip.sv under cocotb + Verilator
.venv/bin/python -m pytest -q fw/spi_target -rxX      # the same, with the xfail reasons
.venv/bin/python -m fw.spi_target.measure             # the sweep, ~2 minutes
```

`make fw` runs simulations, so expect it to take a while rather than
running the module directly if the machine is busy.

## The program

```
PUBLIC idle:
    wait 0 gpio CS_PIN       ; CSn low: a transaction starts
    jmp bit_loop
PUBLIC bit_loop:
    wait 0 gpio SCK_PIN      ; launch edge (SCK fell)
    out pins, 1              ; one MISO bit, the OSR's MSB
    wait 1 gpio SCK_PIN      ; capture edge (SCK rose)
    in pins, 1               ; one MOSI bit into the ISR, autopush every eight
    jmp !osre, bit_loop      ; osr_cnt is the byte counter: fall out once per byte
    jmp pin, idle            ; CSn high: the master is done, park
last:
    pull block               ; next MISO byte.  `.wrap last` returns to bit_loop.
```

**Pin map** (protocol pin indices, `docs/mvp-isa.md` `### The 18 protocol pins`): CSn = 0,
MOSI = 1, SCK = 2, MISO = 3. MOSI and SCK are swapped relative to `fw/spi` (the master firmware)
and that is forced, not cosmetic: `in pins, n` takes IN_ROT bit 0, which is pin `IN_BASE`, so
IN_BASE has to be MOSI. The two clock lines are watched with the **absolute** `gpio` wait source
rather than the relative `pin` source, which would tie SCK to exactly one pin above MOSI; with
`gpio` the four lines can be any four distinct `uio` pins, which
`test_the_four_lines_can_be_on_any_four_pins` runs on five maps. Because that source is absolute,
its index is baked into the instruction word, so moving a line is a re-assembly
(`program_with_pins`) rather than a register write.

**No side-set** (`SIDESET_COUNT = 0`): the target drives exactly one pin and does it through the
OUT window, so the whole delay field is delay. The host sets MISO's *direction* once per
transaction with a forced `set pindirs, 1` and releases it with `set pindirs, 0` afterwards, the
way a real flash sits high-Z between reads.

**The byte counter is `osr_cnt`.** `jmp !osre` is "`osr_cnt < thresh(PULL_THRESH)`"
(`docs/mvp-isa.md` `### Field values`), so with `PULL_THRESH = 8` the loop falls out once per
byte with no bit counter and no `set y, 7`. AUTOPULL is **off**, which is what makes the host's
pre-load work: with it on, `pull` is a no-op whenever `osr_cnt == 0` ("the OSR is full, so PULL
is a barrier"), and the boundary's own `pull block` would be the only refill.

**The host keeps a word ahead.** The two waits are *level* waits, so they are edge detectors only
because the loop keeps up. A `pull block` that stalls on an empty TX FIFO is not a slow
transaction but a corrupted one: the phase is lost for the rest of it, because the next
`wait 1` completes on the level it already sees. Every test asserts `FDEBUG == 0`, because a
stall or an overrun the transaction survived is a latent failure, not a clean run.
## The ceiling, and what sets it

Mode 0 is CPOL 0, so a target changes MISO on the **falling** edge and the master samples it on
the next **rising** edge (`docs/mvp-regmap.md` `### Physical layer`), which is what
`verif.vip.spi.SpiFlashTarget` does — `_launch()` on a non-capture edge, `_capture()` on a
capture edge. So the answer for a bit has to be on the net within half a bit period of the
falling edge that launched it:

```
MISO on the wire at   T + 3 (T_IN)  + 1 (the `out pins, 1`)  + 1 (T_OUT)   <=   T + half_period
=>  half_period >= 5  =>  10 core cycles per bit  =>  5.00 MHz at 50 MHz
```

All three terms are **measured off the chip**, not read off the spec, by
`test_the_latency_decomposition_adds_up_to_the_ceiling`: it watches the net's SCK edges, the
cycle each instruction retires in, and the tick MISO changes, and checks they are 3 + 1 + 1.

The sweep (`measure.py`, and `test_the_rate_sweep`, which
is parametrised and marks the failing rates as strict xfails carrying the measured reason):

| `half_period` | core cycles/bit | SCK | outcome |
|---|---|---|---|
| 2 | 4 | 12.50 MHz | fails: the loop retires 5 instructions per bit and a bit period has 4 cycles, so it loses edges — the received bytes break as well |
| 3 | 6 | 8.33 MHz | fails: MISO reaches the net 2 cycles after the master sampled it; the answer comes back shifted |
| 4 | 8 | 6.25 MHz | fails: MISO 1 cycle late — the same bits, shifted by one |
| **5** | **10** | **5.00 MHz** | **served** — the bit arrives in the very cycle the master samples it, zero margin |
| 6 | 12 | 4.17 MHz | served, one core cycle of margin (the shipped configuration) |
| 8 | 16 | 3.13 MHz | served |
| 12 | 24 | 2.08 MHz | served |

A rate counts as *served* only when all of it holds: the bytes `SpiMonitor` decoded off MISO are
the ones the device was told to send, `monitor.violations` is empty, `FDEBUG` is clear, and the
bus recorded no contention. `violations` alone is not enough — it only holds timing breaches, and
a device whose MISO arrives one cycle late breaks nothing in it, it just returns shifted bytes.

### The three terms, separated by measurement

* **Input synchroniser, 3 cycles, 60% of the budget.** T_IN = 3 (two synchroniser flops and one
  more register, `docs/mvp-isa.md` `## Pins`) is the price of seeing an edge at all, and no
  program can pay it less. This is the number thesis C1 is about: the answer cannot leave the
  engine before the edge has been *seen*, and on this chip seeing it costs 60 ns
  (`research/ideation/round2/research/C1.md` `### F3`).
* **Instruction count on the launch path, 1 cycle, 20%.** Measured by
  `test_one_more_instruction_on_the_launch_path_costs_one_half_period`: the shipped program with
  one `nop` between the launch wait and the `out pins, 1`, and nothing else changed, moves the
  ceiling from 5 to 6. One SM cycle of latency on the launch path is one half period of rate.
  Note what this is *not*: the loop is five instructions per bit (eight at a byte boundary)
  against a bit period of ten cycles at the ceiling, so the loop's total length is never the
  binding constraint at CLKDIV 1.0 — only the instructions *between seeing the edge and writing
  the pin* are.
* **Divider.** One SM cycle is `INT` core cycles, so the deadline becomes `half_period >= 3 +
  INT + 1`. At INT 1 that is the whole story and it agrees with the measurement (5 = 5). From
  INT 2 on the loop's own length binds as well — a byte boundary costs eight SM cycles and a bit
  period is `2 * half_period / INT` of them — so the two effects are not a sum and the table is
  recorded as measured (`DIVIDER_CEILINGS`, checked by
  `test_the_measured_ceiling_at_each_divider`):

  | CLKDIV | fastest served | core cycles/bit | MHz | `T_IN + INT + T_OUT` |
  |---|---|---|---|---|
  | 1.0 | `half_period` 5 | 10 | 5.00 | 5 |
  | 2.0 | `half_period` 9 | 18 | 2.78 | 6 |
  | 3.0 | `half_period` 12 | 24 | 2.08 | 7 |
  | 4.0 | `half_period` 15 | 30 | 1.67 | 8 |

  The lesson for a device-mode design: **a divider buys nothing on the target side.** It slows
  the engine, so the edge-to-answer latency grows with it; the wire rate it appears to buy is
  spent again on the answer.

## What the role cannot express

Not everything a serial flash does fits the MVP PIO, and the gaps are in the ISA rather than in
this program. [`CANNOT-EXPRESS.md`](CANNOT-EXPRESS.md) has the full list with the measured limit
of the closest partial program; the two that shape everything else:

* **The device cannot decode the opcode.** `jmp pin` tests one pin (`EXECCTRL.JMP_PIN`) and the
  only pin carrying command data is MOSI, so a target can branch on at most *one bit of the bit
  period being clocked*, and only on the bit currently on the pin. A received byte cannot be
  compared with anything. So `driver.py` is what turns "READ 16 bytes from 0x30" into the byte
  stream a flash would have shifted out, and `test_the_answer_really_comes_from_the_host` is what
  keeps that honest — a firmware that emitted a fixed pattern would pass the other tests.
* **The device cannot index an array.** The 24-bit address arrives whole (four address values
  are tested, all 24 bits, `test_the_whole_24_bit_address_reaches_the_device`) and the host can
  read it back over the register bus, but the PIO has no way to turn a received value into an
  address. The flash array is host-side; the device is a streaming engine with a four-word window
  the host refills one byte per word.

Both are the same fact seen twice: **32 words of IMEM are code, and the only wide data path the
MVP PIO has is the host's FIFO.**

## What this says about thesis C1

A target lane clocked by the bus it serves would land the answer inside the bit time
instead of 60–80 ns after the edge. This firmware is the baseline such a lane would have to beat,
and it is the *sampled* answer measured rather than argued:

* 5.00 MHz at 50 MHz, i.e. **half a bit period of 10 core cycles** is the shortest one the device
  can answer in, and 3 of those 10 cycles are the input synchroniser — the part a bus-clocked
  lane deletes.
* `docs/mvp-regmap.md` `### Physical layer` calls 6.25 MHz guaranteed for reads *on the host
  port path*, which sees the pad directly. A PIO target watching the same pins gets 5.00 MHz,
  and the difference is exactly the T_IN loop (`T_OUT + T_IN = 4` on the RP2040,
  `docs/mvp-isa.md` `## Timing`) plus the one instruction it needs to answer with.
* The failure mode above the ceiling is not a violation and not a lost byte — it is a
  **one-bit-shifted answer**, which is the quietest possible way to be too slow and the reason a
  device-mode test has to compare decoded bytes and not just timing.

## Files

| File | What it is |
|---|---|
| `program.pio` | the whole device: nine words, with the timing arithmetic and the ISA limits in its header |
| `driver.py` | the host side: program load, the arming sequence, the TX/RX streaming state machine, `jedec_stream` / `read_stream` |
| `measure.py` | the sweep that writes the log, over the same bench the tests use |
| `test_spi_target.py` | the bench and the tests, including the parametrised rate sweep with measured xfails and the latency decomposition |
| `CANNOT-EXPRESS.md` | what a real serial flash does that this cannot, and the measured limit of the partial program |

The bench's one adapter, `ChipPins`, is a `ChipDevice` subclass that names the chip's pin nets
from a list instead of by prefix: `SpiController` names its four nets by module constants
(`verif/vip/spi.py` gives it no net-name arguments), and bridging the two with a net-copying
device would cost a core cycle in each direction — one fifth of the budget measured here.
Its header says so, and `test_re_assembling_the_default_pin_map_changes_nothing` keeps the
re-assembly path identical to the shipped program.

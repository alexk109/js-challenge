# fw/ps2 — PS/2 controller firmware for the MVP chip

The chip plays the **controller** (the PC side): it generates the clock for the bytes it
sends and samples the clock the device generates for the bytes it receives. PS/2 is the one
bus in this firmware set whose clock comes from the device, so the program has to bit-bang
one pin and read another (`research/ref/protocols/ps2/README.md`, "Why PS/2").

| File | What it is |
|---|---|
| `program.pio` | the whole controller: 22 of 32 IMEM words, two loops over two pins |
| `driver.py` | the register writes (`docs/mvp-regmap.md`), the frame packing, the timing model |
| `__init__.py` | `assemble_program()` and the public names |
| `test_ps2.py` | the bench: `ChipDevice` on a `verif.vip.core.Bus` at 20 ns/tick, judged by `verif.vip.ps2` |
| `measure.py` | the numbers below, and two rows in `metrics/fw.csv` |

## How to run it

```bash
make fw PROTO=ps2 TARGET=iss        # the acceptance command: 13 tests
.venv/bin/python -m fw.ps2.measure  # the numbers, and two rows in metrics/fw.csv
```

`FW_TARGET=rtl` runs the same bench against `src/pio_chip.sv` under cocotb
(`fw/rtl_harness.py`); the driver is written against the register map, not against the ISS,
so nothing in it changes.

## The host protocol

The host pushes the bytes and forces two jumps; everything else the program does by itself.

```
push tx_word(byte) on TXF0      the frame, LSB first, inverted for the direction register
jmp send                        12 clocks: request-to-send, 11 frame bits, the ack clock
                                -> one RX word back, bit 0 = the device's acknowledgement
set y, n + jmp recv             n+1 frames from the device, one RX word each
```

```python
driver.command(0xFF, response=1)   # reset a keyboard: 0xFA then 0xAA
driver.receive(2)                  # two scan-code bytes, start/parity/stop checked
```

`send` returns whether the device acknowledged the byte, and `command` re-sends on a missing
acknowledgement or a 0xFE, up to `retries` times. Both lines stay open drain throughout, so
the driver inverts every frame bit it pushes: on a `uio` pin the level register stays 0 and
`out pindirs, 1` *pulling the line low* is what puts a 0 on the wire
(`docs/mvp-isa.md` `### Open drain`).

## Results

Measured on the golden ISS with `python -m fw.ps2.measure` and
`make fw PROTO=ps2 TARGET=iss`.

| | value |
|---|---|
| IMEM words used | **22 of 32** |
| chip role | controller (host), one engine |
| legal `CLKDIV` at 20 ns/tick | 94 … 147 |
| configuration the tests run at | `CLKDIV 100`: clock low 34.0 us, high 32.0 us, 15.15 kbit/s |
| slow configuration | `CLKDIV 140`: clock low 47.6 us, high 44.8 us, 10.82 kbit/s |
| **max speed** | **`CLKDIV 94`: 16.12 kbit/s of data bits (3102 core cycles/bit)** |
| shortest gap on the wire | 3008 ticks (60.2 us) — the acknowledgement clock |
| peak FIFO depth | 1 of 4 (the host never relies on a stall) |
| `FDEBUG` after every transaction | 0 |
| violations, all 13 tests | 0 (except the one test whose expected answer *is* a parity error) |

The request-to-send is 230 us at `CLKDIV 100` (the floor is 100 us) and the bus-free wait is
2500 ticks (50 us), both inside the timing table.

## What limits the speed

Both clock phases have to be 30..50 us (T3/T4, T7/T8) and the divider is the only knob, so
the program is written in whole SM cycles with its short phases at 16 SM cycles and its data
bits at 33. What runs out first is the pair of 16-SM-cycle phases — the high phase of every
clock and the low phase before the acknowledgement clock — which is why the fastest legal
divider is 94 and not 88 (the long phases would allow it). One divider lower, the checker
says so itself: with the driver's guard stepped around, `CLKDIV 93` produces exactly
`tCLK-high` and `tCLK-low` violations (1488 ticks = 29.76 us) and nothing else.

## Things worth knowing before porting this

- **The ISR cannot be reversed into the right byte order.** PS/2 sends its data bits LSB
  first, so the first bit sampled lands at the top of the frame in the ISR and the byte comes
  out the wrong way round inside its field, whichever shift direction is chosen. `rx_byte`
  reverses those eight bits in the driver; `mov isr, isr r` reverses the whole register,
  which moves the frame to bits 31:21 instead of flipping the byte, and costs an explicit
  `push noblock` as well (AUTOPUSH would empty the ISR before the reversal). Two IMEM words
  and a strange layout for the same eight bits is a bad trade at 32 words of IMEM, so the
  reversal happens on the host. See the comment on `rx_byte`.
- **The receive loop has to wait for both edges.** The clock stays low for a whole bit time,
  so a loop that only waits for a falling edge samples one bit per SM cycle and reads the
  same bit eleven times. `wait 0 gpio` / `in pins, 1` / `wait 1 gpio` / `jmp x--` is the shape
  the loop has to have, and `timing_of` refuses a program that does not.
- **`receive` may only be entered while the bus is free.** Its first instruction is
  `wait 0 pin 0`, so entering it in the middle of a frame locks onto the middle of one. The
  start/stop/parity checks turn that into an error rather than a wrong byte, and the caller
  discipline is in `command`: send, then receive, before the device's 50 us runs out.
- **`send` deliberately does not wait for the bus to be free.** The device starts answering
  50 us after the bus goes idle, which is after the host's acknowledgement clock, so the
  receive path has to be armed before then. The wait belongs to the caller, and `receive`
  ends with it.
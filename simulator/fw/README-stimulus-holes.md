# Closing the stimulus holes in the spec

`docs/mvp-verification-report.md` `### Known holes in the stimulus` names four firmware gaps:

> UART covers 8N1 only; no parity, no 7 or 9 data bits, no two stop bits.  Nothing runs below
> 9 600 Bd on the UART RX path.  The SPI firmware cannot do a transaction longer than 31 bytes,
> because the byte counter goes in as a 5-bit immediate through `SMe_INSTR`, and nothing splits
> the transaction.

This file is the map of what now answers that, what it is judged by, and where the numbers are.
It is the README for the three protocol directories below; each test file's own docstring is the
detailed version of its half.

| The hole | What answers it | Judged by |
|---|---|---|
| UART: no parity, no 7 or 9 data bits, no 2 stop bits (TX) | `fw/uart_tx/framing.py` generates the program; `fw/uart_tx/driver.py` takes `framing=` | `verif/vip/uart.py`'s `UartRx`, unmodified, configured with the framing |
| the same on the receive side | `fw/uart_rx/framing.py` generates the program; `fw/uart_rx/driver.py` takes `framing=` | `verif/vip/uart.py`'s `UartTx`, unmodified |
| nothing below 9 600 Bd on the RX path | the rate ladders in both `test_uart_*_framing.py` | the same VIPs, plus the `CLKDIV` arithmetic on every target |
| SPI: no transaction longer than 31 bytes | `SpiConfig(hold_cs=1)`, `SpiMaster.start_long()` / `run_long()` in `fw/spi/driver.py` | `verif/vip/spi.py`'s `SpiFlashTarget` and `SpiMonitor`, unmodified |
| SPI: "in all four SPI modes" | `spi_read_m1` / `spi_read_m2` in `fw/spi/program.pio`, `SpiConfig(cpha=1)`, and `fw/spi/cpha1_flash.py` to reach the CPHA 1 half of the flash model | the same flash model and monitor; `SpiMonitor` decodes all four modes |

## How to run it

```sh
# the acceptance command: the golden model, the three protocol directories
FW_TARGET=iss timeout 3000 .venv/bin/python -m pytest -q fw/uart_tx fw/uart_rx fw/spi

# the matrix: the second independent ISS, then the RTL under cocotb + Verilator (slow)
FW_TARGET=iss_b timeout 3000 .venv/bin/python -m pytest -q fw/uart_tx fw/uart_rx fw/spi
FW_TARGET=rtl    timeout 3000 .venv/bin/python -m pytest -q fw/uart_tx fw/uart_rx fw/spi
```

Two marks decide what runs where, both from `fw/targets.py::is_fast()` and both for the same
reason: a 300 Bd frame is 166 667 core cycles per bit time, so one byte is about two million ticks
of simulation.  `SLOW` is defined in the four test files below and marks the 300 Bd rungs and the
fourteen non-portable framings of each 18-framing matrix as model-only.  The 256-byte SPI rung is
deliberately *not* one of them (see `### The results`); every other rung runs on every target,
including the `CLKDIV` arithmetic that reaches the slow rates.

## The results

Every number below was measured by running the commands above.

| Run | Result |
|---|---|
| `FW_TARGET=iss`, the standard command | 277 passed in 94.85 s |
| `FW_TARGET=iss_b`, the same command on the second ISS | 277 passed in 112.62 s |
| `FW_TARGET=rtl`, the same command on `src/pio_chip.sv` | 190 passed, 87 skipped in 688.4 s |

The 87 skips on `rtl` are the `SLOW`-marked rungs -- the 300 Bd ones and the fourteen
non-portable framings of each 18-framing matrix -- because that target runs at about a thousand core
cycles a second here: 2 162 cycles in 2.24 s and 4 322 in 3.56 s, and 300 Bd is 166 667 ticks per
bit time (the marks were counted by firing every one of
them on a model target).  The 256-byte SPI rung is *not* one of them: its 17 290 core cycles are
14 to 18 seconds there, and it runs everywhere.  The arithmetic that *reaches* the slow
rates runs on every target.

The tests that were already in these three directories (81 of them) are unchanged and still pass;
the rest are the 196 added with the framing and long-transaction work.  The printed lines carry the measured numbers --
the rate ladder's ticks per bit and worst bit-edge deviation, the SPI transactions' chunk counts,
CS window lengths and SCK rates, and the CPHA 1 bit period -- because `pytest -s` is not needed:
`test_uart_tx_framing.py`, `test_uart_rx_framing.py`, `test_spi_long.py` and `test_spi_cpha1.py`
all print their measurements.

## Four things the work turned up

Each is a property of the chip or of the models as they stand, found while building the tests and
recorded where it is asserted, so none of them is only in this file.

**The PIO subset cannot compute a parity bit, so the receiver's parity check is host-side.**
`JMP`'s conditions are X, Y, ISR-non-zero, OSR, GPIO and IRQ, and `MOV`'s ops are invert and
reverse (`docs/mvp-isa.md` `## Encoding`, `### Instruction semantics`, MOV): there is no way to
fold a register's bits together and branch on the result.  So the receiver shifts the parity bit in
like a data bit -- it is on the wire, in the word, at bit 31 -- and the host compares it with the
parity of the data bits it just received.  The transmitter's parity bit is likewise computed by the
host and packed into the TX word, because the program shifts out a fixed number of bits.  What this
does *not* weaken: the VIP receiver on the far end of the wire is told the parity mode and raises
`parity_error` on its own, and `fw/uart_tx/test_uart_tx_framing.py` and
`fw/uart_rx/test_uart_rx_framing.py` both plant an inverted parity bit to show the check bites.
The frame convention is written twice on purpose -- once in `fw/uart_tx/framing.py` and once in the
VIP -- and `test_parity_bit_agrees_with_the_judge` compares the two over 1 024 cases, so the two
agree by evidence rather than by a shared function.

**A 2 %-fast sender is tolerated for two back-to-back frames, and then the drift is flagged rather
than hidden.**  The receiver re-anchors on every start bit, so a sender whose frames are 2 % short
walks 2 % further ahead of the receiver's grid on each frame, and the margin `0.5 / frame_bits` is
4 % for the longest framing (8O2, thirteen bit times) against 5.3 % for the shipped 8N1.  The tests
therefore claim ±2 % over *two* frames, and
`test_back_to_back_frames_drift_against_a_fast_sender` measures what happens after that: with a
parity bit on the wire the mis-sampled byte is always flagged, and a framing without parity has
nothing to flag it with.  What no flag can catch is a frame that never arrives: at 2 % fast every
parity framing is measured receiving 5 bytes for 6 sent, and a frame the host never hears about
carries nothing at all.  Both are in the log, as a count per framing.

**SPI mode 1 and 2 are not "one more program".**  `verif/vip/spi.py`'s `SpiFlashTarget` refuses
them -- correctly, since it models a serial flash -- so modes 1 and 2 need both a second program
shape *and* a way to reach the CPHA 1 half of the model.  `fw/spi/cpha1_flash.py` is that way: it
constructs the model with the mode of the same CPOL and then switches the phase, so every field the
model sets is set by the model, and `test_the_subclass_adds_no_state` asserts the two objects then
differ in exactly `mode` and `_out_idx`.  The CPHA 1 programs run a 9-core-cycle bit period
(5.556 MHz) against 8 (6.25 MHz) for the CPHA 0 ones: their capture edge has to be five core cycles
after the launch edge the target reacted to, because `T_IN = 3` is what the master's sample passes
through, and a three-cycle gap samples the value from *before* the launch.

**The phase error is not symmetric.**  A CPHA 0 program against a CPHA 1 flash reads every bit one
position out and the flash decodes opcode 7 instead of 3, so the judge rejects it by itself.  The
other pairing -- a CPHA 1 program against a CPHA 0 flash -- interoperates, because the CPHA 1
program drives MOSI a whole half bit before the target captures it and samples MISO a half bit
after the target launched it.  `test_the_wrong_phase_is_caught` asserts the asymmetry rather than
rounding it away, because "the phases disagree" is not the same statement in both directions.

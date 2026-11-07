# fw/can: CAN transmitter firmware for the MVP PIO

The chip is the **transmitter**: `program.pio` is a
five-word bit engine that puts one bit per bit time on one open-drain line at exactly the
nominal bit time and reads the line back in the same bit time, and `driver.py` builds the
frame (CRC-15, bit stuffing, field order, INTERMISSION) on the host and hands the PIO the
ready wire bits. `verif/vip/can.py`'s `CanNode` is the receiver that acknowledges and its
`CanChecker` is the judge; neither is edited.

`CANNOT-EXPRESS.md` is the map of what this ISA cannot do for CAN -- the hardware CRC, the
acknowledgement, arbitration, and all of sec. 7 fault confinement -- with the measurements
behind each claim, and with the ones that turned out not to be walls (the run-length window,
and a receiver-side stuff-bit detector on top of it: 20 words, 25 instructions per bit time,
run at 1 Mbit/s) counted rather than asserted.

## Results

| | |
|---|---|
| IMEM words used | **5 of 32** |
| checker violations | **zero** at 125 kbit/s, 500 kbit/s, and every bit time in `metrics/fw-can-speed.csv` |
| fastest bit time | 10 core cycles = 200 ns = **5 Mbit/s** at 50 MHz (CLKDIV 1) |
| classic-CAN ceiling | 1 Mbit/s is exactly reachable (50 cycles, CLKDIV 5) |
| unreachable | 2 Mbit/s (25 cycles): not a whole number of the loop's 10 SM cycles |
| host reaction budget | 400 core cycles per bit at 125 kbit/s, 100 at 500 kbit/s |
| longest frame | 125 wire bits (8 data bytes, worst stuffing), against a 128-bit TX FIFO |

## How to run it

    make fw PROTO=can TARGET=iss        # the acceptance command: 18 passed, 2 skipped
    python -m fw.can.measure            # the bit-time sweep -> metrics/fw-can-speed.csv
    python -m fw.can.limits             # the limits table -> metrics/fw-can-limits.csv

The two skipped tests are `test_the_chip_cannot_receive_and_acknowledge` and
`test_fault_confinement_and_error_frames`; both name `CANNOT-EXPRESS.md` in their skip
reason, and both are properties of the instruction set rather than of this code.

Everything runs from the repo root; the numbers above come from:

    make fw PROTO=can TARGET=iss
    .venv/bin/python -m fw.can.measure
    .venv/bin/python -m fw.can.limits

## How it is wired

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are `p0`..`p17`; the CAN line is one of those nets, so the models see the chip's
own pins with no coupling device in between and no extra latency. Every device on the net
drives 0 or releases it, which is CAN's wired-AND bus (Bosch sec. 2).

    Bus(20 ns/tick)
      ├── ChipDevice(Chip)      nets p0..p17; the CAN line is p<can_pin>
      ├── CanNode               the receiver: acknowledges, decodes, counts errors
      └── CanChecker            passive: locks its bit grid on each SOF and flags violations

## Files

| File | Lines | What it is |
|---|---|---|
| `program.pio` | 5 words | the bit engine: `set pindirs, 0` / `jmp idle` idle loop and the `out`/`in`/`jmp` bit loop |
| `driver.py` | | register words, the frame builder, the SM-cycle-grid alignment, the FIFO streaming, the ACK check |
| `test_can.py` | | the `Bench` and the tests |
| `measure.py` | | the bit-time sweep |
| `limits.py` | | the measured limits table |
| `CANNOT-EXPRESS.md` | | what the ISA cannot do, with the numbers |

## Two things worth knowing before reading `driver.py`

**The alignment.** The reference model has no propagation delay and no oscillator error,
so a node can only hard-synchronise on a SOF edge that lands on one of its own bit
boundaries, and its bit boundaries are the multiples of the bit time because its grid
starts at tick 0. `driver.py` therefore chooses the core cycle the SOF's `out` runs in
(`CanDriver._sof_cycle`), and it needs to know where the engine's SM cycles fall, because
`CLKDIV_RESTART` fixes that phase and no register reports it -- so `load()` *measures* it
(`_measure_phase`) rather than deriving it. `test_the_sof_edge_lands_on_the_bus_bit_grid`
checks the result against the tick the models actually saw.

**The host does the CRC.** Not a shortcut: the MVP has no XOR and no readable data memory,
and its 15 bits of CRC feedback do not fit in 32 IMEM words however slowly the engine is
clocked (`CANNOT-EXPRESS.md` §1 has the count), so the CRC is the host's arithmetic, exactly
as the stuffing and the field order are.
What the PIO owns is the part that has to be exact in core cycles -- the bit time, the
open-drain drive, and the read-back.
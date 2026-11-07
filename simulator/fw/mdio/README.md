# `fw/mdio/` -- MDIO/MDC (IEEE 802.3 clause 22) station firmware for the MVP chip

The chip is the **station**: it owns MDC and drives the data, and the PHYs
at the other end of the bus answer a read's turnaround and data bits.  Clause 22 has exactly
one station per bus and it is the management entity that sources MDC
(`research/ref/protocols/mdio/ti-dp83tc811-smi.txt`), and that is also the only side
`verif/vip/mdio.py` can be judged from: its `MdioStation` model *is* a station, so this
firmware replaces it, while `MdioPhy` (a 32 x 16-bit clause 22 register file with the
LAN8720A power-on values) and `MdioChecker` (a passive monitor that decodes every frame off
the pins) stay on the bus as the judge.

    make fw PROTO=mdio TARGET=iss      # the acceptance command: 42 tests
    FW_TARGET=iss_b python -m pytest -q fw/mdio    # the second ISS, behind fw/targets.py
    FW_TARGET=rtl    python -m pytest -q fw/mdio    # src/pio_chip.sv under cocotb
    python -m fw.mdio.measure          # the speed table in "Results" below

## The files

| File | What it is |
|---|---|
| `program.pio` | 23 of the 32 IMEM words: `halt` at address 0, `write_frame`, `read_frame`.  Every bit time is three instructions and 20 SM cycles. |
| `driver.py` | `MdioDriver`: the register values, the two TX words a frame is made of, and the three host phases that put one frame on the wire. |
| `test_mdio.py` | Five configurations, each a five-frame burst, judged by `verif/vip/mdio.py`; plus an independent decode of the wire, the protocol corner cases, and four planted faults. |
| `measure.py` | Sweeps MDC rates and prints what the checker said at each, for the numbers quoted below. |

## How one frame gets on the wire

A clause 22 frame is 64 bit times: 32 preamble ones, ST = 01, OP (10 read / 01 write),
PHYAD[4:0], REGAD[4:0], the two turnaround bits and DATA[15:0].  The host pushes two TX
words -- the preamble and one word holding everything from ST to DATA -- forces a `pull block`
and then forces `jmp write_frame` or `jmp read_frame` through `SMe_INSTR`, and counts 64 MDC
rising edges on a passive `MdioBusMonitor`.

Three things about the MVP shape that, and are the whole reason `driver.py` exists:

* **`SIDESET_COUNT` resets to 2 with `SIDE_EN = 0`**, where every instruction side-sets two
  pins and the side-set channel beats OUT/SET on a shared pin.  The program uses one `side`
  operand with no `opt`, so the driver writes `SIDESET_COUNT = 1` and points `SIDESET_BASE` at
  MDC: `side 1` is MDC high.
* **The host cannot write the OSR or the PC** (`SMe_ADDR` is read-only), so both come from
  forced instructions: `pull block` loads the first word, `jmp <label>` starts the frame.
* **`T_IN = 3`**: an engine in cycle `n` sees the pad value of cycle `n-3`, so a read's data is
  sampled during the low phase of the bit time it belongs to, not on the edge.  The program's
  comment at `tdata` works the arithmetic out.

Counting 64 bit times took the counting out of the program: a 5-bit `SET` immediate cannot hold
64, and `jmp !osre` runs while `osr_cnt < thresh(PULL_THRESH)`, so with `PULL_THRESH = 32`
(and `AUTOPULL` off, or the refill would reset the counter) one 32-bit word is exactly one pass
of the bit loop, and a frame is exactly two words.

## Results

All numbers below were measured on the golden ISS with `make fw PROTO=mdio TARGET=iss`.

* **`make fw PROTO=mdio TARGET=iss`: 42 passed**, `MdioChecker` reporting **zero violations**
  in every case.
* **IMEM: 23 of 32 words** (`halt` 1, `write_frame` 8, `read_frame` 14), read back out of the
  assembled words by `check_program`, not asserted by hand.
* **Fastest MDC: 2.5 MHz**, which is the clause 22 ceiling and also this firmware's fastest
  legal bit time: 20 SM cycles at `CLKDIV = 1.0` is 20 core cycles = 400 ns, exactly the
  `tclkp` floor of LAN8720A Table 5.12.  `MdioChecker.max_mdc_hz` measures 2.5000 MHz off the
  pins.  With every delay set to 0 the same program would clock 16.67 MHz, so the ceiling is
  the protocol's and not the chip's.
* **Rates measured clean**: 2.5 MHz, 2.381 MHz, 1.25 MHz, 625 kHz and 500 kHz (measured by `measure.py`).  Below 500 kHz the
  *monitor* gives up, not the firmware: `MdioChecker`'s quiet window is three times the 400 ns
  spec minimum (60 ticks) until it has measured a whole frame, and MDC's high phase alone is
  10 SM cycles per bit, so a first frame at 416.7 kHz (120 core cycles per bit) is reported as
  an abandoned frame.  `MdioPhy` learns the same way.  This is a property of the reference
  model, and it is the reason the tested rates stop at 500 kHz.
* **One frame** costs 1310 bus ticks at 2.5 MHz (64 bit times plus the host's four forced
  instructions and the 16-tick idle gap), so 38.2 k frames/s, 76.3 k register accesses per
  second, read or written.

## What a caller sees

```python
from fw.mdio import MdioDriver, MAX_MDC_HZ
from iss.pio_iss import ChipDevice
from verif.vip.core import Bus

bus = Bus(ns_per_tick=20)                       # 50 MHz chip clock
for k in range(18):
    bus.add_net(f"p{k}", push_pull=(k == 1))     # MDIO is the wired-AND line
dev = bus.attach(ChipDevice())
drv = MdioDriver(dev, bus, mdc_pin=0, mdio_pin=1, mdc_hz=MAX_MDC_HZ)
drv.start()                                      # IMEM, the four registers, pins, enable
drv.write_reg(0, 0x2100)                         # BMCR: 100 Mbit, full duplex, autoneg off
value = drv.read_reg(1).value                    # BMSR: bit 2 is the link status
```

`write_reg` / `read_reg` queue one frame and run it; `queue_write` / `queue_read` plus `run()`
put a burst of frames out back to back, which is what walking a PHY's register file looks like.
A read of an address no PHY claims comes back as `0xffff`, which is clause 22's "no PHY there"
and not an error.

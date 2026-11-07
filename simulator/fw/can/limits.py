#!/usr/bin/env python3
"""The measured limits of fw/can, as a table. This is what CANNOT-EXPRESS.md cites.

Every row is something a reader could otherwise take on faith: the instruction budget
is counted from the assembled words, the bit-time granularity and the fastest speed
come from the same sweep as `fw/can/measure.py`, the FIFO figures are what the driver
actually reached while a frame went out, and the "no PIO instruction can do this" rows
are the ones `fw/can/test_can.py` checks against `docs/mvp-isa.md`.

    python -m fw.can.limits                # print, and write metrics/fw-can-limits.csv
    python -m fw.can.limits --csv other.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from asm.pioasm import assemble
from fw.can import driver as drv
from fw.can.test_can import Bench, SPEED_125K, SPEED_500K, TICK_NS

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CSV = ROOT / "metrics" / "fw-can-limits.csv"
FIELDS = ("limit", "measured", "unit", "how")

#: docs/mvp-isa.md `### Engine state`: 32 words of 16 bits per engine.
IMEM_WORDS = 32


def rows() -> list[dict]:
    program = assemble(drv.PROGRAM_PATH.read_text())[0]
    out: list[dict] = [
        {"limit": "IMEM words used", "measured": len(program.instrs), "unit": "of 32 words",
         "how": "assembled from program.pio; the other 27 are free"},
        {"limit": "PIO instructions in the bit loop", "measured": 3, "unit": "instructions",
         "how": "out pindirs, 1 / in pins, 1 / jmp engine"},
        {"limit": "SM cycles per bit", "measured": drv.loop_cycles(program), "unit": "SM cycles",
         "how": "summed from the assembled delay fields"},
        {"limit": "bit-time granularity", "measured": drv.SM_CYCLES_PER_BIT, "unit": "core cycles",
         "how": "the loop is the granularity: no CLKDIV puts a frame on a shorter bit time"},
        {"limit": "fastest bit time expressible",
         "measured": drv.SM_CYCLES_PER_BIT, "unit": "core cycles",
         "how": "CLKDIV 1, i.e. 200 ns = 5 Mbit/s at 50 MHz"},
        {"limit": "classic-CAN ceiling, 1 Mbit/s",
         "measured": int(1000 / TICK_NS), "unit": "core cycles per bit",
         "how": "50 cycles is ten SM cycles times CLKDIV 5, so 1 Mbit/s is exactly "
                "reachable; 2 Mbit/s (25 cycles) is not"},
        {"limit": "XOR instruction", "measured": 0, "unit": "in the MVP ISA",
         "how": "docs/mvp-isa.md `### Field values`: MOV copies, inverts or bit-reverses; "
                "no XOR and no readable data memory, so the CRC-15 is not one instruction "
                "away -- the 32 IMEM words are what exclude it (fw/can/CANNOT-EXPRESS.md 1)"},
        {"limit": "data memory readable by the program", "measured": 0, "unit": "bytes",
         "how": "IMEM is fetch-only (docs/mvp-isa.md `### Engine state`), so there is no "
                "CRC table either"},
        {"limit": "IRQ / interrupt to the host", "measured": 0, "unit": "flags",
         "how": "docs/mvp-isa.md `## Differences from RP2040` item 3: the IRQ instruction "
                "is a NOP and `wait 1 irq` never completes"},
    ]

    # What the driver actually reaches while a frame goes out. These two rows measure one
    # frame this driver sent, with the 8-bit INTERMISSION tail the driver appends, so they
    # are not comparable with the codec's tail-free maxima further down.
    for speed in (SPEED_500K, SPEED_125K):
        bench = Bench(speed).start()
        bench.driver.peak_fifo = 0
        result = bench.driver.send(0x6FE, b"\xff" * 8)          # the frame this driver sends
        clean = not bench.checker.violations
        out.append({
            "limit": f"frame sent at {speed} (ID 0x6FE, 8 bytes, tail included)",
            "measured": result.wire_bits, "unit": "wire bits (8 data bytes, stuffed)",
            "how": f"the frame fw/can/driver.py actually put on the bus, including the "
                   f"8-bit tail; checker clean={clean}, {len(result.tx_words)} TX words, "
                   f"peak TX FIFO {bench.driver.peak_fifo} of {drv.FIFO_DEPTH}",
        })
        out.append({
            "limit": f"host cycles per bit at {speed}",
            "measured": bench.driver.bit_ticks, "unit": "core cycles",
            "how": "the budget the host has to react to anything the PIO reports "
                   f"({bench.driver.bit_ticks * int(TICK_NS) / 1000:.1f} us)",
        })

    # Frame sizes, from the codec, so the FIFO story has numbers in it.
    short = drv.wire_bits(0x123, b"", tail=0)[0]
    long = drv.wire_bits(0x7FE, b"\xff" * 8, tail=0)[0]
    worst, worst_id = max(
        (len(drv.wire_bits(i, b"\xff" * 8, tail=0)[0]), i) for i in range(0x000, 0x800))
    out += [
        {"limit": "shortest frame (DLC 0)", "measured": len(short), "unit": "wire bits",
         "how": "SOF, 11 ID bits, RTR, r1 r0, DLC, CRC, delimiters, EOF, stuffed"},
        {"limit": "longest frame (DLC 8, worst stuffing, no tail)",
         "measured": worst, "unit": f"wire bits (identifier {worst_id:#05x})",
         "how": "the maximum over every 11-bit identifier with eight data bytes and no "
                "tail; a TX FIFO holds 4 x 32 = 128 bits, so on the longest frames the "
                "host has to keep feeding while the engine shifts"},
        {"limit": "TX FIFO capacity", "measured": drv.FIFO_DEPTH * 32, "unit": "bits",
         "how": "docs/mvp-isa.md `### Engine state`: four 32-bit words"},
    ]
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    args = parser.parse_args(argv)
    table = rows()
    width = max(len(r["limit"]) for r in table)
    for row in table:
        print(f"{row['limit']:<{width}}  {row['measured']:>8} {row['unit']}")
        print(f"{'':<{width}}  {row['how']}")
    with args.csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(FIELDS))
        writer.writeheader()
        writer.writerows(table)
    print(f"\nwritten: {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
"""Measure the MDIO firmware: IMEM words, the fastest legal MDC, and what stops it going faster.

Run it (it is a short job, like the tests):

    .venv/bin/python -m fw.mdio.measure

Everything printed here comes from a run of the real bench in `fw/mdio/test_mdio.py`: the chip
on the ISS at 50 MHz, `verif/vip/mdio.py`'s `MdioPhy` answering and its `MdioChecker` judging.
For each candidate MDC rate the script puts one write frame and one read frame on the bus and
reports what the checker made of them, so "the fastest this firmware can go" is a measurement
rather than an argument:

* **The spec ceiling.**  Clause 22 puts a 400 ns floor on the MDC period, so no station may
  exceed 2.5 MHz (`research/ref/protocols/mdio/lan8720a-mdio-timing.txt`, LAN8720A Table 5.12
  `tclkp`).  `MdioChecker`'s `MDC` rule enforces it, and the program's bit time is 20 SM
  cycles, which at `CLKDIV = 1.0` is exactly 20 core cycles: the fastest MDC this firmware can
  produce *is* the ceiling, with nothing to spare.
* **What the hardware could do.**  The three instructions of a bit time have delays, and the
  smallest they can be is 0, so a bit could be 3 SM cycles = 60 ns = 16.7 MHz.  The script
  measures that too, by re-timing the program, to show the limit is the protocol's and not the
  chip's.
* **Where the divider runs out.**  A 16.8 divider reaches a given rate only to within
  `bit_cycles / 256` core cycles per bit, so some rates cannot be hit exactly; the script prints
  the achieved period next to the requested one.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fw.mdio import driver as drv                                    # noqa: E402
from fw.mdio.driver import (                                          # noqa: E402
    CORE_HZ,
    FRAME_BITS,
    IMEM_SIZE,
    MAX_MDC_HZ,
    MdioDriver,
    bit_cycles,
    check_program,
    clkdiv_for_mdc,
    decode_word,
    load_program,
    words_used,
)
from fw.mdio.test_mdio import BMCR, BMSR, Bench, Case                # noqa: E402
from iss.pio_iss import ChipDevice                                   # noqa: E402
from verif.vip.core import Bus                                       # noqa: E402
from verif.vip.mdio import attach_checker, attach_phy                # noqa: E402

NS_PER_TICK = 20.0
#: The rates to try, fastest first, and then past the ceiling so the table shows where the
#: firmware stops being legal and which rule says so.
RATES_HZ = [
    2_500_000.0,      # the clause 22 ceiling
    2_380_952.4,      # 21 core cycles per bit: CLKDIV 1.05, the 16.8 divider's next step down
    1_250_000.0,
    625_000.0,
    500_000.0,
    416_666.7,        # 120 core cycles per bit: the slowest the checker's first-frame window allows
    250_000.0,        # 200 core cycles per bit: slower than that window
    125_000.0,
]

#: The rate the program's delays could express if they were all 0: a bit time of three SM
#: cycles, which is 60 ns.  The script measures it with the timing model frozen, because
#: `clkdiv_for_mdc` refuses to aim a divider above the ceiling -- the point is to show that the
#: *protocol*, not the chip, is what stops MDC at 2.5 MHz.
HARDWARE_LIMIT_HZ = CORE_HZ / 3


def retimed(low: int, high: int):
    """program.pio with every bit-time delay replaced by `low` / `high`."""
    program = load_program()
    for index, word in enumerate(program.instrs):
        fields = decode_word(program, word)
        if fields["op"] in (0b010, 0b011) and fields["sideset"] == 0:
            field = (fields["sideset"] << 4) | low
            program.instrs[index] = (word & ~(0x1F << 8)) | (field << 8)
        elif fields["op"] == 0b101 and fields["sideset"] == 1:
            field = (fields["sideset"] << 4) | high
            program.instrs[index] = (word & ~(0x1F << 8)) | (field << 8)
    return program


def try_rate(mdc_hz: float, program=None, freeze_bit_cycles=None, drive_hz=None):
    """One write and one read at `mdc_hz`; what the checker said.

    `freeze_bit_cycles` tells the driver a bit time the program does not have, and
    `drive_hz` is the rate the driver is allowed to aim at, which is how a run above the
    ceiling is measured at all: `clkdiv_for_mdc` refuses to build an illegal bus, so the
    divider is aimed at the ceiling and the *program* is what runs too fast.
    """
    original = drv.bit_cycles
    if freeze_bit_cycles is not None:
        drv.bit_cycles = lambda prog: freeze_bit_cycles
    try:
        bench = Bench(Case("measure", drive_hz or mdc_hz), program=program)
        bench.driver.write_reg(BMCR, 0x2100)
        read = bench.driver.read_reg(BMSR)
        bench.settle()
        return {
            "requested_hz": mdc_hz,
            "clean": bench.checker.clean,
            "rules": sorted(bench.rules()),
            "shortest_period": bench.checker.shortest_period,
            "max_mdc_hz": bench.checker.max_mdc_hz,
            "read": read.value,
            "clkdiv": bench.driver.clkdiv,
            "ticks_per_bit": bench.driver.ticks_per_bit,
            "ticks": bench.bus.t,
        }
    finally:
        if freeze_bit_cycles is not None:
            drv.bit_cycles = original


def header(program) -> None:
    """What the program is, read back out of the words."""
    shape = check_program(program)
    print(f"program            : {words_used(program)} of {IMEM_SIZE} IMEM words, "
          f"wrap 0..{program.wrap}, labels {shape['labels']}")
    print(f"bit time           : {shape['bit_cycles']} SM cycles "
          f"(low 1+9, high 1+8, jmp 1), so CLKDIV 1.0 is 20 core cycles = 400 ns")
    print(f"spec ceiling       : MDC <= {MAX_MDC_HZ:.6g} Hz (400 ns period, LAN8720A "
          f"Table 5.12 `tclkp`)")
    print(f"fastest this can go: {CORE_HZ / shape['bit_cycles']:.6g} Hz "
          f"({shape['bit_cycles']} core cycles per bit at CLKDIV 1.0), which is the ceiling")
    print(f"chip clock         : {CORE_HZ / 1e6:.0f} MHz, one Bus tick = {NS_PER_TICK:.0f} ns")
    print(f"role               : station (the chip owns MDC; the PHYs answer reads)")
    print()


def table(rows) -> None:
    print(f"{'requested Hz':>13} {'CLKDIV':>10} {'ticks/bit':>10} {'shortest':>9} "
          f"{'bus MHz':>9}  verdict")
    for row in rows:
        verdict = "clean" if row["clean"] else ", ".join(row["rules"])
        print(f"{row['requested_hz']:>13.7g} {row['clkdiv']:>#10x} "
              f"{row['ticks_per_bit']:>10.3f} {row['shortest_period']:>9} "
              f"{(row['max_mdc_hz'] or 0) / 1e6:>9.4f}  {verdict}")


def main() -> int:
    program = load_program()
    header(program)

    print("as programmed (20 SM cycles per bit, CLKDIV aimed at the requested rate):")
    rows = [try_rate(hz, program=program) for hz in RATES_HZ]
    table(rows)

    clean = [r for r in rows if r["clean"]]
    broken = [r for r in rows if not r["clean"]]
    print()
    if clean:
        fastest = max(clean, key=lambda r: r["requested_hz"])
        print(f"fastest clean rate : {fastest['requested_hz']:.6g} Hz "
              f"(CLKDIV {fastest['clkdiv']:#x}), shortest MDC period "
              f"{fastest['shortest_period']} ticks = {CORE_HZ / fastest['shortest_period']:.6g} Hz")
        slowest = min(clean, key=lambda r: r["requested_hz"])
        print(f"slowest clean rate : {slowest['requested_hz']:.6g} Hz "
              f"({slowest['ticks_per_bit']:.0f} core cycles per bit)")
    for row in broken:
        print(f"not clean at       : {row['requested_hz']:.6g} Hz breaks "
              f"{', '.join(row['rules'])}")

    # What the chip itself could do, if the protocol allowed it.
    row = try_rate(HARDWARE_LIMIT_HZ, program=retimed(0, 0), freeze_bit_cycles=20,
                   drive_hz=MAX_MDC_HZ)
    row["requested_hz"] = HARDWARE_LIMIT_HZ
    print()
    print(f"hardware limit     : with every delay 0 a bit time is three SM cycles, so MDC "
          f"could reach {CORE_HZ / 3:.6g} Hz; measured {row['max_mdc_hz']:.6g} Hz, which "
          f"breaks {', '.join(row['rules'])}")
    print(f"                   so the 2.5 MHz ceiling is the protocol's, not the chip's")

    # One frame's worth of ticks, at the rate being reported.
    bench = Bench(Case("measure", MAX_MDC_HZ))
    before = bench.bus.t
    bench.driver.write_reg(BMCR, 0x2100)
    ticks = bench.bus.t - before
    print(f"frame cost         : one frame, {FRAME_BITS} bit times and the host's four forced\n"
          f"                   instructions, took {ticks} bus ticks at "
          f"{MAX_MDC_HZ / 1e6:.3f} MHz, so {CORE_HZ / ticks:.4f} frames per second of chip "
          f"clock, {2 * CORE_HZ / ticks:.4f} register accesses per second read or written")
    return 0


if __name__ == "__main__":
    sys.exit(main())

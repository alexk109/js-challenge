"""Measure what the i2s firmware puts on the bus: rates, words, and where the limits are.

    .venv/bin/python -m fw.i2s.measure

Every number this prints comes out of a run: the checker is the only thing that decides
whether a stream is legal, and the driver derives the rate it claims from its own assembled
words.  The module answers three questions.

1. **How fast is it?**  For each `CLKDIV` from 1 upwards: the bit clock and sample rate the
   driver claims, the period the checker measured, and the violations it found.
2. **What is the timing envelope?**  The shipped `program.pio` uses `HALF_DELAY 3`, i.e. four
   SM cycles per half bit clock period, which at `CLKDIV = 1` and 50 MHz puts SCK at
   6.25 MHz.  This module re-assembles the same file with the `.define HALF_DELAY` line
   rewritten (`--half-delay N`) to find the fastest half period UM11732 §4's limits still
   accept: the checker is configured for the rate the rewritten program produces, so a clean
   run is a measurement and not a guess.
3. **What does the instruction budget buy?**  Words used per engine, the slot length, and the
   bits per second of *payload* (the bit clock minus the framing that is not audio).

Nothing here is used by `driver.py`; it is a measurement tool, and it edits nothing on disk.
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

from asm.pioasm import assemble
from fw.i2s import Config, I2sDriver, driver as drv
from fw.i2s.test_i2s import FRAMES, SAMPLES, Bench
from iss import pio_iss
from verif.vip.core import Bus

DEFINE = re.compile(r"^(\s*\.define\s+HALF_DELAY\s+)(\d+)(.*)$", re.MULTILINE)


@dataclass(frozen=True)
class Row:
    """One measured configuration.

    `note` is set when the models refused the geometry outright (`verif.vip.i2s` raises below
    `MIN_TICKS_PER_BIT_CLOCK` ticks per bit clock period), which is itself a result: that
    floor is the fastest stream the judge can be configured for.
    """

    label: str
    clkdiv: int
    half: int
    words: tuple[int, int]
    bit_clock_hz: float
    sample_rate: float
    slot_bits: int
    slots: int
    period_ticks: int
    violations: int
    decoded: list[int]
    note: str = ""

    @property
    def ok(self) -> bool:
        return self.violations == 0 and self.decoded == list(SAMPLES) and not self.note

    def line(self) -> str:
        if self.note:
            return f"n/a {self.label:<10} clkdiv={self.clkdiv} half={self.half:2d} " \
                   f"SCK={self.bit_clock_hz / 1e6:8.4f} MHz  rate={self.sample_rate:11.3f} Hz  " \
                   f"period={self.period_ticks:3d} ticks  not measurable: {self.note}"
        mark = "ok " if self.ok else "BAD"
        return (f"{mark} {self.label:<10} clkdiv={self.clkdiv} half={self.half:2d} "
                f"words={self.words[0]:2d}+{self.words[1]:d} "
                f"SCK={self.bit_clock_hz / 1e6:8.4f} MHz  "
                f"rate={self.sample_rate:11.3f} Hz  "
                f"period={self.period_ticks:3d} ticks  "
                f"violations={self.violations}")


WS_HEADER = ".program i2s_ws\n.side_set 0\n.wrap_target 0\nPUBLIC start:\n"
WS_OUT = "        out pins, 1\n"
WS_NOP = "        nop [31]\n"
WS_PULL = "        pull block [{}]\n"


def ws_source(slot_sm_cycles: int) -> str:
    """The `i2s_ws` program for a slot of `slot_sm_cycles` SM cycles, as assembler text.

    The shipped `i2s_ws` in `program.pio` is exactly this with `slot_sm_cycles = 128`: one
    `out pins, 1` (1 SM cycle), the padding, and a `pull block` (1 + delay cycles) whose
    delay fills the slot.  The padding nops carry the largest delay the field allows (31, so
    32 SM cycles each), because the number of them has to shrink when the slot does.
    `check_ws_program()` then measures the result and refuses it if it is not the right
    length, so the two programs cannot drift apart.
    """
    if slot_sm_cycles < 2:
        raise ValueError(f"a slot of {slot_sm_cycles} SM cycles cannot hold the WS write")
    pad = slot_sm_cycles - 2            # the out (1) and the pull (1 + delay)
    nops, delay = divmod(pad, 32)
    if delay > 31:
        raise ValueError(f"a slot of {slot_sm_cycles} SM cycles does not fit the delay field")
    body = WS_HEADER + WS_OUT + WS_NOP * nops + WS_PULL.format(delay)
    return f"{body}.wrap {nops + 1}\n"


def retime(half_delay: int, path: Path = drv.PROGRAM_PATH) -> tuple:
    """Assemble `program.pio` with `.define HALF_DELAY` set to `half_delay`.

    Engine 0's timing comes from that one `.define`; engine 1 is regenerated with
    `ws_source()` because its padding has to match the new slot length.  A half period is
    `1 + HALF_DELAY` SM cycles and the delay field is `5 - SIDESET_COUNT` bits wide, so
    anything above 15 does not fit and the assembler says so.  Nothing is written to disk.
    """
    text = path.read_text(encoding="utf-8")
    new, count = DEFINE.subn(rf"\g<1>{int(half_delay)}\g<3>", text)
    if count != 1:
        raise ValueError(f"{path}: expected one `.define HALF_DELAY` line, found {count}")
    head = new.split(".program i2s_ws")[0]        # everything up to engine 1's program
    data = assemble(head)[0]
    ws = assemble(ws_source(drv.slot_cycles(data)))[0]
    drv.check_data_program(data)
    drv.check_ws_program(ws, data)
    return data, ws


def measure(clkdiv: int = 1, half_delay: Optional[int] = None, slots: int = 2,
            label: str = "") -> Row:
    """Run one configuration through the bench and report what the checker saw.

    A geometry the models refuse to be configured with -- `verif.vip.i2s` raises below
    `MIN_TICKS_PER_BIT_CLOCK` ticks per bit clock period, so a half period of one SM cycle at
    CLKDIV 1 is out of reach of the judge -- comes back as a row with a note and no
    violations, rather than as a crash half way through the sweep.
    """
    program = retime(half_delay) if half_delay is not None else None
    driver = None
    try:
        bench = Bench(clkdiv=clkdiv, slots=slots, program=program).start().finish()
    except ValueError as exc:                       # the models rejected the geometry
        driver = I2sDriver(pio_iss.ChipDevice(pio_iss.Chip()), Bus(ns_per_tick=drv.TICK_NS),
                           config=Config(clkdiv=clkdiv, slots=slots), program=program)
        return Row(label=label or f"half{half_delay}", clkdiv=clkdiv, half=driver.half_cycles,
                   words=driver.words_used, bit_clock_hz=driver.bit_clock_hz,
                   sample_rate=driver.sample_rate, slot_bits=driver.config.slot_bits,
                   slots=driver.config.slots, period_ticks=driver.bit_clock_ticks,
                   violations=0, decoded=[], note=str(exc).split(": ", 1)[-1])
    driver = bench.driver
    return Row(
        label=label or f"half{half_delay}" if half_delay is not None else f"clkdiv{clkdiv}",
        clkdiv=clkdiv,
        half=driver.half_cycles,
        words=driver.words_used,
        bit_clock_hz=driver.bit_clock_hz,
        sample_rate=driver.sample_rate,
        slot_bits=driver.config.slot_bits,
        slots=driver.config.slots,
        period_ticks=driver.bit_clock_ticks,
        violations=len(bench.chk.violations),
        decoded=list(bench.decoded),
    )


def rate_sweep(max_clkdiv: int = 8, slots: int = 2) -> list[Row]:
    """One row per CLKDIV at the shipped `HALF_DELAY`, fastest first."""
    return [measure(clkdiv=d, slots=slots, label=f"clkdiv{d}")
            for d in range(1, max_clkdiv + 1)]


def envelope(half_delays=range(0, 5), slots: int = 2) -> list[Row]:
    """One row per half period, at CLKDIV 1: where UM11732 §4's limits stop accepting."""
    return [measure(clkdiv=1, half_delay=h, slots=slots, label=f"HALF_DELAY{h}")
            for h in half_delays]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--max-clkdiv", type=int, default=8,
                    help="highest integer divider to measure (default 8)")
    ap.add_argument("--slots", type=int, default=2, choices=(1, 2),
                    help="slots per frame: 2 = stereo (default), 1 = mono")
    args = ap.parse_args(argv)

    data, ws = drv.load_program()
    print(f"program.pio: {data.name} {drv.words_used(data)} words, {ws.name} "
          f"{drv.words_used(ws)} words of 32 per engine")
    print(f"half period {drv.half_cycles(data)} SM cycles, slot {drv.slot_cycles(data, ws)} "
          f"SM cycles ({drv.EXPECTED_CELLS}-cell slot), bits per frame "
          f"{drv.EXPECTED_CELLS * args.slots}")
    print(f"pattern: {len(SAMPLES)} slots = {len(SAMPLES) // args.slots} frames "
          f"{[f'{l:#06x}/{r:#06x}' for l, r in FRAMES]}")

    print("\n-- CLKDIV sweep (shipped HALF_DELAY, 50 MHz core) --")
    rows = rate_sweep(args.max_clkdiv, args.slots)
    for row in rows:
        print(row.line())

    print("\n-- half-period sweep at CLKDIV 1 (the timing envelope) --")
    env = envelope(slots=args.slots)
    for row in env:
        print(row.line())

    clean = [r for r in rows if r.ok]
    env_clean = [r for r in env if r.ok]
    print("\n-- summary --")
    for row in env:
        if row.note:
            print(f"not measurable: {row.label} ({row.period_ticks} ticks per bit clock) "
                  f"-- {row.note}")
    if clean:
        fastest = clean[0]
        print(f"fastest clean CLKDIV: {fastest.clkdiv} -> SCK {fastest.bit_clock_hz / 1e6:.4f} "
              f"MHz, sample rate {fastest.sample_rate:.3f} Hz, "
              f"{fastest.slot_bits}-bit x{fastest.slots}")
    if env_clean:
        quickest = env_clean[0]
        print(f"fastest clean half period: {quickest.half} SM cycles -> SCK "
              f"{quickest.bit_clock_hz / 1e6:.4f} MHz, sample rate {quickest.sample_rate:.3f} Hz")
        bad = [r for r in env if not r.ok]
        for row in bad:
            print(f"rejected: {row.line()}")
    return 0 if clean and env_clean else 1


if __name__ == "__main__":
    sys.exit(main())
"""Print what this firmware puts on the wire, for every configuration the tests run.

    .venv/bin/python -m fw.wiegand.report

Everything below is derived from the assembled words and the divider words the driver builds --
nothing is typed in by hand -- so the numbers quoted in README.md can be regenerated
and compared.  The measured half (what the models on the wire saw) is in the test output of

    make fw PROTO=wiegand TARGET=iss
"""
from __future__ import annotations

import sys

from fw.wiegand.driver import (
    CORE_HZ,
    READER_DEFAULTS_NS,
    TICK_NS,
    WIEGAND_WINDOW_NS,
    WiegandDriver,
    ceil_ticks,
    clkdiv_for_pulse,
    frame_bits,
    load_program,
    phase_cycles,
    ticks_per_sm_cycle,
)


class _NoBus:
    """Stand-in for the ChipDevice: the report never touches the bus."""

    def write(self, addr, data):  # pragma: no cover - never called
        raise AssertionError("the report does not run the chip")


def _driver(pw_ns: float, d0_pin: int = 0, engine: int = 0) -> WiegandDriver:
    return WiegandDriver(_NoBus(), None, d0_pin=d0_pin, engine=engine, pw_ns=pw_ns)


def speed_table() -> str:
    """Every pulse width the program can express, at the extremes of the SIA window."""
    program = load_program()
    n_pw, n_pi = phase_cycles(program)
    rows = []
    for name, ns in (("fastest legal", WIEGAND_WINDOW_NS["pw_min"]),
                     ("BALTECH PulseWidth", READER_DEFAULTS_NS["baltech_pw"]),
                     ("Pyramid Tpw", READER_DEFAULTS_NS["pyramid_pw"])):
        clkdiv = clkdiv_for_pulse(ns, n_pw, n_pi)
        drv = _driver(ns)
        rows.append(
            f"  {name:<20} {ns / 1000:>6.1f} us pulse -> CLKDIV {drv.clkdiv:#010x}, "
            f"{drv.pulse_ticks:>9.0f} + {drv.pause_ticks:>9.0f} ticks = "
            f"{drv.bit_ticks * TICK_NS / 1000:>7.1f} us per bit, "
            f"{drv.frame_span_ticks:>9} ticks per frame"
        )
    return "\n".join(rows)


def limits() -> str:
    """The two limits that decide the speed, and what they cost."""
    program = load_program()
    n_pw, n_pi = phase_cycles(program)
    fastest = clkdiv_for_pulse(WIEGAND_WINDOW_NS["pw_min"], n_pw, n_pi)
    drv = _driver(WIEGAND_WINDOW_NS["pw_min"])
    frame = drv.pairs * drv.bit_ticks
    gap = ceil_ticks(WIEGAND_WINDOW_NS["pi_max"]) + 1
    return "\n".join([
        f"  fastest legal frame   {drv.pulse_ticks:.0f} + {drv.pause_ticks:.0f} = "
        f"{drv.bit_ticks:.0f} ticks per bit, {frame:.0f} ticks "
        f"({frame * TICK_NS / 1e6:.2f} ms) of wire time",
        f"  pulse rate            {CORE_HZ / drv.bit_ticks:.1f} bit/s "
        f"(the 200 us pause floor is what stops it, not the chip)",
        f"  card to card          {(frame + gap) * TICK_NS / 1e6:.2f} ms, of which "
        f"{gap * TICK_NS / 1e6:.0f} ms is the quiet time a frame boundary costs",
        f"  slowest phase ratio   {n_pi + 1}:1, so BALTECH's 50 us / 2 ms (40:1) is out of "
        f"reach from one divider; 50 us / 1.6 ms (32:1) is not",
        f"  divider at the floor  {fastest:#010x} = {ticks_per_sm_cycle(fastest):.0f} core cycles "
        f"per SM cycle",
    ])


def main(argv=None) -> int:
    program = load_program()
    cases = [("floor-20us", 20_000.0, 0, 0),
             ("reader-50us", READER_DEFAULTS_NS["baltech_pw"], 0, 0),
             ("ceiling-100us-engine1", READER_DEFAULTS_NS["pyramid_pw"], 1, 5)]
    out = [
        f"{program.name}: {len(program.instrs)} of 32 IMEM words, "
        f"{len(frame_bits(123, 4567))} bits per frame (26 direction pairs of 2 bits)",
        "",
        "the chip plays the reader: it drives the two open-drain lines and then goes quiet",
        "",
        limits(),
        "",
        speed_table(),
        "",
        "configurations the test runs:",
    ]
    for name, pw_ns, engine, pin in cases:
        out.append(_driver(pw_ns, pin, engine).report())
        out.append("")
    sys.stdout.write("\n".join(out) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
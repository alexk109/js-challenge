"""MVP PS/2 firmware: the PIO program and its host driver.

    from fw.ps2 import assemble_program, Ps2Driver
    program = assemble_program()
    driver = Ps2Driver(bus, chip_device, program, clkdiv=100)
    driver.load()
    driver.command(0xFF, response=1)          # reset the keyboard: 0xFA, then 0xAA
    driver.receive(3)                         # three scan-code bytes off the bus

The chip is the **controller**: `program.pio` generates the clock for a byte it sends, reads
the clock the device generates for a byte it receives, and needs nothing from the host but
the bytes themselves, a jump and a count. See the header of that file for the pin windows
and the wire sequence, `driver.py` for the register map side and for the timings read back
out of the assembled words.

Run with `make fw PROTO=ps2 TARGET=iss`.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from asm.pioasm import Program, assemble

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    ACK,
    BUS_FREE_NS,
    DEFAULT_CLK_DIV,
    DEFAULT_CLK_PIN,
    DEFAULT_DATA_PIN,
    FRAME_BITS,
    HOST_FRAME_BITS,
    RESEND,
    Ps2Driver,
    Ps2Error,
    Ps2Timing,
    check_program,
    frame_bits,
    jmp_word,
    legal_clkdiv,
    parity_bit,
    retarget,
    rx_ack,
    rx_byte,
    rx_parity,
    rx_start,
    rx_stop,
    set_y_word,
    timing_of,
    tx_word,
)

PROGRAM_PIO = Path(__file__).with_name("program.pio")

__all__ = [
    "PROGRAM_PIO",
    "assemble_program",
    "ACK",
    "RESEND",
    "BUS_FREE_NS",
    "DEFAULT_CLK_DIV",
    "DEFAULT_CLK_PIN",
    "DEFAULT_DATA_PIN",
    "FRAME_BITS",
    "HOST_FRAME_BITS",
    "Ps2Driver",
    "Ps2Error",
    "Ps2Timing",
    "check_program",
    "frame_bits",
    "jmp_word",
    "legal_clkdiv",
    "parity_bit",
    "retarget",
    "rx_ack",
    "rx_byte",
    "rx_parity",
    "rx_start",
    "rx_stop",
    "set_y_word",
    "timing_of",
    "tx_word",
]


def assemble_program(path: Optional[Path] = None) -> Program:
    """Assemble fw/ps2/program.pio and return its single `.program`.

    The IMEM is 32 words, so a file with more than one program cannot be loaded into one
    engine; that is an error here rather than a surprise at load time. `timing_of` runs on
    the result, so a program whose shape the driver does not recognise is refused before a
    transaction is attempted rather than after one has been put on the wire.
    """
    source = Path(path) if path is not None else PROGRAM_PIO
    programs = assemble(source.read_text(encoding="utf-8"))
    if len(programs) != 1:
        raise ValueError(
            f"{source} assembles to {len(programs)} programs, the driver loads one"
        )
    program = programs[0]
    if len(program.instrs) > 32:
        raise ValueError(
            f"{source} needs {len(program.instrs)} words, an engine has 32 "
            f"(docs/mvp-isa.md `### Engine state`)"
        )
    timing_of(program)          # refuses a program whose shape the driver cannot read
    return program
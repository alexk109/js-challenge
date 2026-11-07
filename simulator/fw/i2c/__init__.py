"""MVP I2C firmware: the PIO program and its host driver.

    from fw.i2c import assemble_program, I2cDriver
    program = assemble_program()
    driver = I2cDriver(bus, chip_device, program, clkdiv=14)
    driver.load()
    driver.write_read(0x50, [0x10], 4)     # one transaction: write, restART, read

`program.pio` is the whole controller: one bit loop that writes a byte, reads a byte
or answers with an ACK depending only on the bits the host pushes. See the header of
that file for the pin windows and for how it differs from the RP2040 program, and
`driver.py` for the register map side.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from asm.pioasm import Program, assemble

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    I2cDriver,
    I2cError,
    RX_DATA_SHIFT,
    RX_NINTH_BIT,
    TX_DATA_SHIFT,
    TX_NINTH_BIT,
    check_program,
    jmp_word,
    retarget,
    read_word,
    rx_ack,
    rx_byte,
    rx_ninth,
    set_y_word,
    tx_word,
)

PROGRAM_PIO = Path(__file__).with_name("program.pio")

__all__ = [
    "PROGRAM_PIO",
    "assemble_program",
    "I2cDriver",
    "I2cError",
    "check_program",
    "retarget",
    "tx_word",
    "rx_byte",
    "rx_ninth",
    "rx_ack",
    "read_word",
    "RX_DATA_SHIFT",
    "RX_NINTH_BIT",
    "jmp_word",
    "set_y_word",
    "TX_DATA_SHIFT",
    "TX_NINTH_BIT",
    "RX_NINTH_BIT",
]


def assemble_program(path: Optional[Path] = None) -> Program:
    """Assemble fw/i2c/program.pio and return its single `.program`.

    The IMEM is 32 words, so a file with more than one program cannot be loaded into
    one engine; that is an error here rather than a surprise at load time.
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
    return program

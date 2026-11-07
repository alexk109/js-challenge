"""MVP 1-Wire firmware: the PIO program and its host driver.

    from fw.onewire import assemble_program, OneWireDriver
    program = assemble_program()
    driver = OneWireDriver(bus, chip_device, program, clkdiv=50)
    driver.load()
    assert driver.read_rom() == ds.rom        # reset, Read ROM, 8 bytes back
    scratchpad = driver.read_scratchpad()    # reset, Skip ROM, Read Scratchpad, 9 bytes

`program.pio` is the whole master: a reset pulse with the presence check, and one time-slot
loop that writes a byte, reads a byte or does both depending only on the words the host
pushes. See the header of that file for the pin windows, for the SM-cycle arithmetic behind
every delay and for how it differs from an RP2040 one-wire program. `driver.py` has the
register map side, the layout checks that keep program.pio and the driver in step, and the
CLKDIV window the DS18B20 timing tables allow.

Run with `make fw PROTO=onewire TARGET=iss`.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from asm.pioasm import Program, assemble

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    BYTE_SLOTS,
    DEFAULT_CLKDIV,
    DEFAULT_DQ_PIN,
    PRESENCE_SAMPLE,
    PRESENCE_WINDOW,
    RESET_LOW,
    RESET_TO_SLOT,
    SLOT_LOW_LONG,
    SLOT_LOW_SHORT,
    SLOT_PERIOD,
    SLOT_SAMPLE,
    TICK_NS,
    OneWireDriver,
    OneWireError,
    check_program,
    cycles,
    decode_word,
    divider_window,
    jmp_word,
    layout,
    limits,
    mov_isr_null_word,
    read_word,
    reset_layout,
    rx_byte,
    tx_word,
)

PROGRAM_PIO = Path(__file__).with_name("program.pio")

__all__ = [
    "PROGRAM_PIO",
    "assemble_program",
    "OneWireDriver",
    "OneWireError",
    "check_program",
    "cycles",
    "decode_word",
    "divider_window",
    "jmp_word",
    "layout",
    "limits",
    "mov_isr_null_word",
    "read_word",
    "reset_layout",
    "rx_byte",
    "tx_word",
    "BYTE_SLOTS",
    "DEFAULT_CLKDIV",
    "DEFAULT_DQ_PIN",
    "PRESENCE_SAMPLE",
    "PRESENCE_WINDOW",
    "RESET_LOW",
    "RESET_TO_SLOT",
    "SLOT_LOW_LONG",
    "SLOT_LOW_SHORT",
    "SLOT_PERIOD",
    "SLOT_SAMPLE",
    "TICK_NS",
]


def assemble_program(path: Optional[Path] = None) -> Program:
    """Assemble fw/onewire/program.pio and return its single `.program`.

    The IMEM is 32 words, so a file with more than one program cannot be loaded into one
    engine; that is an error here rather than a surprise at load time.
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
            "(docs/mvp-isa.md `### Engine state`)"
        )
    return program
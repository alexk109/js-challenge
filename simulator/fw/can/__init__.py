"""MVP CAN firmware: the PIO bit engine and its host driver.

    from fw.can import assemble_program, CanDriver
    program = assemble_program()
    driver = CanDriver(bus, chip_device, program, bit_ticks=400)   # 125 kbit/s
    driver.load()
    result = driver.send(0x123, b"\\xde\\xad")
    assert result.acked

`program.pio` is the CAN *bit* layer and nothing more: one bit per bit time on one
open-drain line, exactly at the nominal bit time, read back in the same bit time.
The CRC SEQUENCE, the bit stuffing, the field order and the INTERMISSION are
`driver.py`. The ISA has no XOR and no data memory the program can read, so the CRC-15
is not one instruction away, and what keeps it out of the PIO is the 32 IMEM words
rather than a missing instruction; a receiver-side stuff-bit detector, by contrast, does
fit and is counted and run in `fw/can/CANNOT-EXPRESS.md` -- a CAN receiver cannot
acknowledge a frame without destuffing it, so the chip is the **transmitter** and
`verif/vip/can.py`'s `CanNode` is the receiver that acknowledges.
`fw/can/CANNOT-EXPRESS.md` is the map of what the PIO cannot do and why, with
measurements.

Run the tests with `make fw PROTO=can TARGET=iss`.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from asm.pioasm import Program, assemble

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    CRC_POLY,
    DEFAULT_CAN_PIN,
    DOMINANT,
    INTERMISSION,
    NOMINAL_TICKS,
    PROGRAM_PATH,
    RECESSIVE,
    SENTINEL,
    SM_CYCLES_PER_BIT,
    STUFF_RUN,
    CanDriver,
    CanError,
    TxResult,
    ack_bit_mask,
    clkdiv_for,
    crc15,
    directions,
    logical_bits,
    loop_cycles,
    readback_word,
    sample_offset,
    stuffed_region,
    stuff,
    tx_words,
    wire_bits,
)

__all__ = [
    "PROGRAM_PATH",
    "assemble_program",
    "program_cycles",
    "CanDriver",
    "CanError",
    "TxResult",
    "DOMINANT",
    "RECESSIVE",
    "CRC_POLY",
    "STUFF_RUN",
    "INTERMISSION",
    "NOMINAL_TICKS",
    "SM_CYCLES_PER_BIT",
    "DEFAULT_CAN_PIN",
    "SENTINEL",
    "crc15",
    "stuff",
    "stuffed_region",
    "logical_bits",
    "wire_bits",
    "directions",
    "tx_words",
    "readback_word",
    "ack_bit_mask",
    "loop_cycles",
    "sample_offset",
    "clkdiv_for",
]

PROGRAM = "can_tx"


def assemble_program(path: Optional[Path] = None) -> Program:
    """Assemble fw/can/program.pio and return its single `.program`.

    The IMEM is 32 words, so a file with more than one program cannot be loaded into
    one engine; that is an error here rather than a surprise at load time.
    """
    source = Path(path) if path is not None else PROGRAM_PATH
    programs = assemble(source.read_text(encoding="utf-8"))
    if len(programs) != 1:
        raise ValueError(f"{source} assembles to {len(programs)} programs, the driver loads one")
    program = programs[0]
    if len(program.instrs) > 32:
        raise ValueError(
            f"{source} needs {len(program.instrs)} words, an engine has 32 "
            f"(docs/mvp-isa.md `### Engine state`)"
        )
    return program


def program_cycles(path: Optional[Path] = None) -> int:
    """SM cycles one bit of the assembled program takes, read out of its own words."""
    return loop_cycles(assemble_program(path))
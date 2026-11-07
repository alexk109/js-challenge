"""MVP I2C target firmware: the PIO program, and the host that plays the EEPROM behind it.

    from fw.i2c_target import assemble_program, I2cTargetDriver
    program = assemble_program()
    driver = I2cTargetDriver(bus, chip_device, program, clkdiv=14)
    driver.load()
    ctl.queue_write_read([0x00, 0xC0], 3)     # the master decides the transaction
    driver.serve_until(lambda: ctl.idle)
    assert driver.eeprom.memory[0x00] == 0xC0

This is device mode, so it is the mirror image of `fw.i2c`: there the host starts every
byte, here the *bus* does, and `I2cTargetDriver` has no transaction entry point at all.
The program answers a 24C02-style EEPROM -- 7-bit address, 256 bytes, an 8-byte page, a
word-address pointer, sequential read, ACK and NAK -- and `verif.vip.i2c.I2cController`
plus `I2cTimingChecker` judge it. `README.md` has the numbers and the measured limits;
`measure.py` is what produced them.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from asm.pioasm import Program, assemble

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    DEFAULT_SDA_PIN,
    DEFAULT_SCL_PIN,
    PHASE_LIMIT,
    SM_CYCLES_PER_BIT,
    START_SETUP,
    STOP_MARKER,
    EepromImage,
    I2cTargetDriver,
    I2cTargetError,
    ack_word,
    check_program,
    clkdiv_max,
    poll_for,
    set_poll,
    data_word,
    is_stop_marker,
    jmp_word,
    rx_byte,
    set_pindirs_word,
    set_y_word,
    start_window_ticks,
)

PROGRAM_PIO = Path(__file__).with_name("program.pio")

__all__ = [
    "PROGRAM_PIO",
    "assemble_program",
    "I2cTargetDriver",
    "I2cTargetError",
    "EepromImage",
    "check_program",
    "clkdiv_max",
    "poll_for",
    "set_poll",
    "data_word",
    "ack_word",
    "rx_byte",
    "is_stop_marker",
    "jmp_word",
    "set_pindirs_word",
    "set_y_word",
    "DEFAULT_SDA_PIN",
    "DEFAULT_SCL_PIN",
    "SM_CYCLES_PER_BIT",
    "START_SETUP",
    "STOP_MARKER",
    "PHASE_LIMIT",
    "start_window_ticks",
]


def assemble_program(path: Optional[Path] = None) -> Program:
    """Assemble fw/i2c_target/program.pio and return its single `.program`.

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
            f"(docs/mvp-isa.md `### Engine state`)"
        )
    return program
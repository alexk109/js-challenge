"""MVP JTAG firmware: the PIO program and its host driver.

    from fw.jtag import assemble_program, JtagDriver
    program = assemble_program()
    driver = JtagDriver(bus, chip_device, program, clkdiv=1)
    driver.load()
    driver.idcode()                      # reset, select IDCODE, read its 32 bits
    driver.scan_dr(0xA5, 8)              # write a byte to whatever DR is selected

**The chip is the controller.** `program.pio` is a bit pump -- one (TMS, TDI) pair per TCK
cycle out of the TX FIFO, TDO sampled in the cycles where the host said the TAP is in a
shift state -- which is the role `verif/vip/jtag.py`'s `JtagMaster` plays and the role a
real probe plays. `driver.py` is the part above it: the 16-state walk, the IR and DR scans,
the pause that lets one scan be longer than Y's 5 bits. That is the same division of labour
the models use, so the bench in `test_jtag.py` puts the VIP's own `JtagTarget` on the far
end and lets `JtagChecker` judge the wire. See `fw/jtag/README.md` for why the chip is not
built as a TAP instead, and for the word and speed budgets.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from asm.pioasm import Program, assemble

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    DEFAULT_TCK_PIN,
    DEFAULT_TDO_PIN,
    DEFAULT_TDI_PIN,
    DEFAULT_TMS_PIN,
    DR_ENTRY_TMS,
    EXIT_TMS,
    IR_ENTRY_TMS,
    MAX_SHIFT_BITS,
    OPCODE_IDCODE,
    PROGRAM_EXIT_CLOCKS,
    JtagDriver,
    JtagError,
    check_program,
    jmp_word,
    loop_delays,
    pair,
    pack_pairs,
    retime,
    scan_bits,
    set_x_word,
    set_y_word,
    tck_cycles,
    tck_hz,
)

PROGRAM_PIO = Path(__file__).with_name("program.pio")

__all__ = [
    "PROGRAM_PIO",
    "assemble_program",
    "JtagDriver",
    "JtagError",
    "check_program",
    "loop_delays",
    "tck_cycles",
    "tck_hz",
    "pair",
    "pack_pairs",
    "retime",
    "scan_bits",
    "jmp_word",
    "set_x_word",
    "set_y_word",
    "DR_ENTRY_TMS",
    "IR_ENTRY_TMS",
    "EXIT_TMS",
    "OPCODE_IDCODE",
    "MAX_SHIFT_BITS",
    "PROGRAM_EXIT_CLOCKS",
    "FIFO_DEPTH",
    "DEFAULT_TCK_PIN",
    "DEFAULT_TMS_PIN",
    "DEFAULT_TDI_PIN",
    "DEFAULT_TDO_PIN",
]


def assemble_program(path: Optional[Path] = None) -> Program:
    """Assemble fw/jtag/program.pio and return its single `.program`.

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

def __getattr__(name):  # FIFO_DEPTH follows the active register map, so it is not frozen at import
    if name == "FIFO_DEPTH":
        from . import driver
        return driver.FIFO_DEPTH
    raise AttributeError(name)

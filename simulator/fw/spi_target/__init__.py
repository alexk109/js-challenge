"""MVP SPI *target* firmware: the PIO program and its host driver.

    from fw.spi_target import assemble_program, SpiTargetDriver, jedec_stream, read_stream
    program = assemble_program()
    driver = SpiTargetDriver(chip_device, program=program, clkdiv...)
    driver.load()
    driver.start(jedec_stream((0xC0, 0x28, 0x17)))     # queue the MISO byte stream
    driver.run(bus, ticks)                             # the master drives the wire

`program.pio` is the whole device: one five-instruction loop that watches SCK, answers on
MISO on the falling edge and takes MOSI in on the rising edge, plus the three words that
end a byte and park the engine when the master releases CSn.  See that file's header for
the pin map, the timing arithmetic behind the maximum rate, and what the ISA makes
impossible; `driver.py` is the register-map side, including why the response bytes come from
the host's TX FIFO, and `measure.py` is the rate sweep whose log the README quotes.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from asm.pioasm import Program, assemble

from .driver import (  # noqa: F401  (re-exported for the firmware's users)
    CS,
    IMEM_BASE,
    IMEM_STRIDE,
    IN_BASE,
    JMP_PIN,
    MISO,
    MOSI,
    NUM_ENGINES,
    OUT_BASE,
    SCK,
    SET_BASE,
    PINDIR,
    PROGRAM_NAME,
    PULL_BLOCK,
    SET_PINDIRS_MISO,
    SpiTargetConfig,
    SpiTargetDriver,
    jedec_stream,
    program_with_pins,
    programs,
    read_stream,
    tx_word,
)

PROGRAM_PIO = Path(__file__).with_name("program.pio")

__all__ = [
    "PROGRAM_PIO",
    "assemble_program",
    "programs",
    "SpiTargetConfig",
    "SpiTargetDriver",
    "jedec_stream",
    "program_with_pins",
    "read_stream",
    "tx_word",
    "CS",
    "MOSI",
    "SCK",
    "MISO",
    "IN_BASE",
    "OUT_BASE",
    "SET_BASE",
    "JMP_PIN",
    "PINDIR",
    "PULL_BLOCK",
    "SET_PINDIRS_MISO",
    "PROGRAM_NAME",
    "IMEM_BASE",
    "IMEM_STRIDE",
    "NUM_ENGINES",
]


def assemble_program(path: Optional[Path] = None) -> Program:
    """Assemble fw/spi_target/program.pio and return its single `.program`.

    The IMEM is 32 words, so a file with more than one program cannot be loaded into one
    engine; that is an error here rather than a surprise at load time.
    """
    source = Path(path) if path is not None else PROGRAM_PIO
    programs_ = assemble(source.read_text(encoding="utf-8"))
    if len(programs_) != 1:
        raise ValueError(f"{source} assembles to {len(programs_)} programs, the driver loads one")
    program = programs_[0]
    if len(program.instrs) > 32:
        raise ValueError(
            f"{source} needs {len(program.instrs)} words, an engine has 32 "
            f"(docs/mvp-isa.md `### Engine state`)"
        )
    return program
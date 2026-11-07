"""MVP firmware for the SPI protocol: a PIO SPI master that reads a serial flash.

* `program.pio`  the PIO program, one `.program` per SPI mode (0 and 3)
* `driver.py`    the host side: register writes, arming the engine, feeding the FIFOs
* `test_spi.py`  the test: `ChipDevice` on a `Bus`, checked against the flash VIP

`make fw PROTO=spi TARGET=iss` runs the test on the golden ISS and `make fw PROTO=spi TARGET=rtl`
runs the same test on `src/pio_chip.sv` (see `fw/rtl_harness.py`).
"""
from __future__ import annotations

from .driver import (  # noqa: F401
    A_CTRL,
    A_FDEBUG,
    A_FLEVEL,
    A_FSTAT,
    CS,
    MISO,
    MOSI,
    SCK,
    SpiConfig,
    SpiMaster,
    programs,
)

__all__ = [
    "A_CTRL",
    "A_FDEBUG",
    "A_FLEVEL",
    "A_FSTAT",
    "CS",
    "MISO",
    "MOSI",
    "SCK",
    "SpiConfig",
    "SpiMaster",
    "programs",
]

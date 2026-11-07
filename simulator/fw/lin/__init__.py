"""MVP LIN firmware: the PIO program, the host driver and their test.

`driver.py` loads `program.pio` (assembled with `asm/pioasm.py`) into an
engine of an `iss.pio_iss.ChipDevice`, configures it through the register map and pushes the
byte fields of a LIN frame into its TX FIFO; `test_lin.py` puts that on a `verif.vip.core.Bus`
and lets `verif.vip.lin` -- `LinResponder` and `LinChecker`, neither of them modified here --
read the wire back and judge it.

    make fw PROTO=lin TARGET=iss
    .venv/bin/python -m fw.lin.report          # the measured numbers, and the headline answers

**The chip is the commander** (LIN 1.x: the master).  The program is a pure bit-bang
transmitter: 26 of the 32 words of one engine, one SM cycle per bit, every field a byte out of
the TX FIFO.  The parity bits of the protected identifier and the checksum are computed by the
host (`make_pid`, `lin_checksum` in `driver.py`), because `docs/mvp-isa.md` `## Encoding` has no
ADD instruction and a state machine cannot sum eight bytes.  `README.md` has the rest of the
argument, including why the responder role is the one the MVP cannot play.

    bus = Bus(ns_per_tick=20)                      # 50 MHz
    for k in range(18):
        bus.add_net(f"p{k}")                       # open drain, pulled up
    dev = bus.attach(ChipDevice())
    rsp = attach_responder(bus, rate=19200, net="p0", publish={0x11: b"\xa5\x5a"})
    chk = attach_checker(bus, rate=19200, net="p0", lengths={0x11: 2})
    drv = LinDriver(dev, bus, pin=0, rate=19200)
    drv.load()
    drv.start(0x11)                                # header only: the responder answers it
    bus.run(drv.frame_ticks(2) + 20000)
    assert chk.clean, chk.report()
"""
from __future__ import annotations

from fw.lin.driver import (
    CORE_HZ,
    UIO_PINS,
    Config,
    LinDriver,
    ProgramTiming,
    clkdiv_for_rate,
    checksum_model,
    clkdiv_value,
    frame_words,
    header_words,
    is_nop,
    is_out_pindirs_1,
    is_pull_block,
    jmp_abs,
    lin_checksum,
    load_program,
    make_pid,
    parity_bits,
    program_timing,
    response_words,
    set_abs,
    ticks_per_bit,
    wire_bytes,
    wire_word,
    words_used,
)

__all__ = [
    "CORE_HZ",
    "Config",
    "LinDriver",
    "ProgramTiming",
    "UIO_PINS",
    "checksum_model",
    "clkdiv_for_rate",
    "clkdiv_value",
    "frame_words",
    "header_words",
    "is_nop",
    "is_out_pindirs_1",
    "is_pull_block",
    "jmp_abs",
    "lin_checksum",
    "load_program",
    "make_pid",
    "parity_bits",
    "program_timing",
    "response_words",
    "set_abs",
    "ticks_per_bit",
    "wire_bytes",
    "wire_word",
    "words_used",
]

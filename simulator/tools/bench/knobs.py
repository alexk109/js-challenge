"""The knob registry: every architecture choice the bench can vary, with the MVP as the default.

Ids and defaults follow archsim's KNOBS (tools/archsim/model/archsim-model.js) where archsim has the
knob; tools/bench/tests/test_knobs_sync.py (task C) keeps the two in step. Knobs archsim lacks are
marked `bench-only` in their help. Which levels can run each knob is NOT stored here: each backend
declares it (Backend.knob_support), and `bench knobs` builds the table from the backends.
"""
from __future__ import annotations

from typing import Any

from .types import Knob, sha

G_CLOCK, G_ENGINE, G_IO, G_CHIP, G_ISA = "clocking", "engines and memory", "host and pins", "chip", "instruction set"

KNOBS: tuple[Knob, ...] = (
    Knob("f_core_mhz", "Core clock", "float", 50.0, (10, 80), "MHz", G_CLOCK, "Supplied by the board MCU."),
    Knob("t_in", "Input synchronizer depth", "int", 3, (2, 4), "cycles", G_CLOCK,
         "bench-only. RTL parameter T_IN (src/pio_chip.sv).", stage=2),
    Knob("n_engines", "Engines", "int", 2, (1, 4), "", G_ENGINE, "Number of PIO engines."),
    Knob("imem_words", "Program memory per engine", "enum", 32, (16, 32, 64, 128), "16-bit words", G_ENGINE),
    Knob("imem_tech", "Program memory technology", "enum", "flop", ("flop", "sram2p"), "", G_ENGINE,
         "sram2p = the IHP two-port macro (src/pio_imem_sram.sv)."),
    Knob("fifo_depth", "FIFO depth (each TX and RX)", "int", 4, (2, 16), "32-bit words", G_ENGINE),
    Knob("engine_route", "Engine-to-engine route", "bool", False, (), "", G_ENGINE,
         "bench-only. Engine e's RX FIFO can feed engine e+1's TX FIFO on chip (protocol bridge)."),
    Knob("host_link", "Host link", "enum", "spi", ("spi", "spi_fast", "qspi"), "", G_IO, "archsim table B.3."),
    Knob("lane", "Bus-clocked lane", "bool", False, (), "", G_IO, "C1 / R1.", stage=2),
    Knob("die", "Tiles", "enum", "6x4", ("6x4", "8x4"), "", G_CHIP, "8x4 unconfirmed for the competition."),
    Knob("isa_xor", "XOR / CRC-step operation", "bool", False, (), "", G_ISA, "", stage=3),
    Knob("isa_cond", "More JMP conditions / compare", "bool", False, (), "", G_ISA, "", stage=3),
    Knob("isa_irq", "IRQ flags and irq / wait irq", "bool", False, (), "", G_ISA, "", stage=3),
)

BY_ID = {k.id: k for k in KNOBS}


def defaults() -> dict:
    return {k.id: k.default for k in KNOBS}


class KnobError(ValueError):
    pass


def _check(k: Knob, v: Any) -> Any:
    if k.kind == "bool":
        if not isinstance(v, bool):
            raise KnobError(f"{k.id}: expected true/false, got {v!r}")
    elif k.kind == "enum":
        if v not in k.domain:
            raise KnobError(f"{k.id}: {v!r} not in {list(k.domain)}")
    elif k.kind in ("int", "float"):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise KnobError(f"{k.id}: expected a number, got {v!r}")
        if k.kind == "int" and int(v) != v:
            raise KnobError(f"{k.id}: expected an integer, got {v!r}")
        lo, hi = k.domain
        if not lo <= v <= hi:
            raise KnobError(f"{k.id}: {v} outside [{lo}, {hi}]")
        v = int(v) if k.kind == "int" else float(v)
    return v


def normalize(partial: dict | None) -> dict:
    """Fill every knob, validate values, reject unknown ids. The MVP is `normalize({})`."""
    partial = dict(partial or {})
    unknown = sorted(set(partial) - set(BY_ID))
    if unknown:
        raise KnobError(f"unknown knob(s): {', '.join(unknown)}")
    out = defaults()
    for kid, v in partial.items():
        out[kid] = _check(BY_ID[kid], v)
    return out


def diff(config: dict) -> dict:
    """Only the knobs that differ from the MVP (for labels and compact display)."""
    d = defaults()
    return {k: v for k, v in config.items() if d.get(k) != v}


def config_hash(config: dict) -> str:
    return sha(normalize(config))


def label(config: dict) -> str:
    d = diff(normalize(config))
    return "MVP" if not d else ", ".join(f"{k}={v}" for k, v in sorted(d.items()))

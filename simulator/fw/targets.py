"""Firmware-test targets: which chip answers the bus. One registry, so tests never know.

Every firmware test in `fw/` takes its chip from `iss.pio_iss.Chip` and everything else (the host
queue `ChipDevice`, the `verif.vip.core.Bus`, the protocol VIPs and checkers) from the real code
path. A *target* is a class that implements the backend contract (`docs/backend-contract.md`) and
is swapped in for that one name before any test module imports a driver. `fw/conftest.py` does the
swap from `FW_TARGET`, once per pytest session:

    FW_TARGET=iss    the golden ISS, `iss/pio_iss.py` (default)
    FW_TARGET=iss_b  the second ISS, `verif/iss_b/pio_iss_b.py`, written blind from the same spec:
                     the firmware tests become a third independent check of the chip's semantics
    FW_TARGET=iss_p  the parametric ISS (`tools/bench/sim/pio_iss_p.py`); sizes from env BENCH_CONFIG
                     (JSON, see tools/bench/sim/REGMAP.md); the MVP when unset
    FW_TARGET=rtl    `src/pio_chip.sv` under cocotb + Verilator (`fw/rtl_harness.py`)
    FW_TARGET=chip   the TT top level over the real SPI host port (`fw/chip_harness.py`, not available yet)

An unknown target is an error, never a skip: a typo in `FW_TARGET` must not report green with
nothing tested. Adding a design (a thesis prototype, the FPGA, gates) means adding one entry here
and passing `verif/contract/` -- no test file changes.

    python -m fw.targets            # list the targets and whether each is available here
    FW_TARGET=rtl python -m pytest -q fw/spi
    python -m pytest -q verif/contract   # the contract, on every target available here
"""
from __future__ import annotations

import copy
import functools
import importlib
import os
import sys
from dataclasses import dataclass
from typing import Callable

#: The environment variable that picks the target. One name, for `make fw`, for pytest and for
#: `verif/contract/`.
ENV = "FW_TARGET"
DEFAULT = "iss"

#: `docs/mvp-regmap.md` `### Address map`: `bus_addr` is a 7-bit word address, so nothing outside
#: this window names a register and a read there returns 0 (`iss.pio_iss.Chip.peek`).
NUM_ADDRS = 128


class UnknownTarget(ValueError):
    """`FW_TARGET` names a target that does not exist. An error, never a skip."""


class TargetUnavailable(RuntimeError):
    """The target is known but cannot run here (not built yet, a tool missing)."""


@dataclass(frozen=True)
class Target:
    name: str
    what: str
    #: "fast" targets run at Python model speed; tests that need millions of cycles may restrict
    #: themselves to these (see `fw/wiegand/test_wiegand.py`).
    fast: bool
    #: Builds the `Chip` class to install, or raises `TargetUnavailable` with the reason.
    load: Callable[[], type]


#: The golden class, captured before any swap, so `install("iss")` can always restore it. It is
#: filled in by the first `install()`; `_golden()` falls back to the live name before that.
_GOLDEN: dict[str, type] = {}


def _golden() -> type:
    from iss import pio_iss

    return _GOLDEN.get("Chip") or pio_iss.Chip


@functools.lru_cache(maxsize=None)
def _iss_b() -> type:
    """`verif/iss_b/pio_iss_b.py` behind the backend contract.

    Cached, like `_rtl` and `_chip`: each of these builds a *new class object*, and two calls
    returning two classes would make `install()` idempotent in behaviour but not in identity --
    `iss.pio_iss.Chip is targets.load()` would be false the second time, and `isinstance` against
    a class captured earlier would fail. `_golden` is deliberately not cached: what it returns
    depends on whether a swap has happened yet.

    ISS-B was written blind from `docs/mvp-isa.md` to the same *documented* API as the golden ISS,
    and it is deliberately not a copy of it, so it does not happen to use the same *names* for
    everything. Two gaps and nothing else, both mechanical, both in the adapter rather than in the
    model:

    * `Chip.eng` is the golden `Chip.engines`, so `chip.engines[e]` needs one property.
    * there is no `peek()`: a free register read. It is answered by performing a real host read of
      that address on a *copy* of the chip and taking the data back. No register-read logic is
      reimplemented here, which is the point -- if ISS-B's read mux and the golden's ever disagree,
      that is a finding, not something this adapter hides. The copy is what makes it free: a host
      read of `RXFn` pops the engine's RX FIFO at the end of its cycle, and on the copy that pop
      goes nowhere. ISS-B's own convention is the one `peek()` promises anyway -- every host access
      in a cycle sees the start-of-cycle state -- so the value is the one the chip would have
      returned in this cycle.
    """
    from verif.iss_b import pio_iss_b as b

    class IssBChip(b.Chip):
        """ISS-B's behaviour, the golden model's observation names."""

        def __init__(self) -> None:
            self._pin_in = 0                       # the last `step()`'s pins, for `peek()`
            super().__init__()             # ISS-B's `reset()` re-runs this, so it covers it

        @property
        def engines(self) -> list:
            return self.eng

        def step(self, pin_in: int, bus: tuple | None = None) -> int | None:
            self._pin_in = pin_in
            return super().step(pin_in, bus)

        def peek(self, addr: int) -> int:
            if not 0 <= addr < NUM_ADDRS:          # no register there: 0, as on the golden peek
                return 0
            shadow = copy.deepcopy(self)
            return b.Chip.step(shadow, self._pin_in, ("r", addr)) or 0

    IssBChip.__name__ = IssBChip.__qualname__ = "IssBChip"
    return IssBChip


def _iss_p() -> type:
    """`tools/bench/sim/pio_iss_p.py`: the golden ISS with the sizes of env BENCH_CONFIG.

    Not cached: the class depends on the environment at install time, and `pio_iss_p.chip_class`
    caches one class per parameter set. The class carries `REGMAP`, which `fw/regmap.py` reads.
    """
    from tools.bench.sim import pio_iss_p

    return pio_iss_p.chip_class_from_env()


@functools.lru_cache(maxsize=None)
def _rtl() -> type:
    """`src/pio_chip.sv` under cocotb: the chip class `fw/rtl_harness.py` already builds."""
    try:
        from fw import rtl_harness
    except ImportError as exc:                     # cocotb / verilator missing
        raise TargetUnavailable(f"fw/rtl_harness.py does not import: {exc}") from exc
    return rtl_harness.RtlChip


@functools.lru_cache(maxsize=None)
def _chip() -> type:
    """The TT top level over the SPI host port (`fw/chip_harness.py`, not available yet).

    The harness is a separate module, so the class it exposes is looked up by a list of candidate
    names rather than pinned here: when it does not exist yet, this target reports
    `unavailable` and `FW_TARGET=chip` fails loudly rather than quietly skipping every test.
    """
    try:
        mod = importlib.import_module("fw.chip_harness")
    except ModuleNotFoundError as exc:
        if exc.name == "fw.chip_harness":
            raise TargetUnavailable(
                "fw/chip_harness.py is not built yet") from exc
        raise
    cls = (getattr(mod, "TtChip", None) or getattr(mod, "ChipOverSpi", None)
           or getattr(mod, "Chip", None))
    if cls is None:
        raise TargetUnavailable("fw/chip_harness.py defines no TtChip / ChipOverSpi / Chip class")
    return cls


TARGETS: dict[str, Target] = {t.name: t for t in (
    Target("iss", "golden ISS (iss/pio_iss.py)", True, _golden),
    Target("iss_b", "independent second ISS (verif/iss_b/pio_iss_b.py) behind the contract", True,
           _iss_b),
    Target("iss_p", "parametric ISS (tools/bench/sim/pio_iss_p.py), sizes from BENCH_CONFIG", True,
           _iss_p),
    Target("rtl", "src/pio_chip.sv under cocotb + Verilator (fw/rtl_harness.py)", False, _rtl),
    Target("chip", "TT top level over the SPI host port (fw/chip_harness.py)", False, _chip),
)}
NAMES = tuple(TARGETS)


def current() -> str:
    """The target this process runs, from `FW_TARGET` (default `iss`), validated."""
    name = os.environ.get(ENV, DEFAULT).strip().lower() or DEFAULT
    if name not in TARGETS:
        raise UnknownTarget(
            f"{ENV}={name!r} is not a target; the targets are {', '.join(NAMES)} (fw/targets.py). "
            f"Refusing to run rather than skip every test.")
    return name


def is_fast(name: str | None = None) -> bool:
    """Whether `name` (default: this process's target) runs at Python model speed."""
    return TARGETS[name or current()].fast


def install(name: str | None = None) -> type:
    """Put target `name` (default: `FW_TARGET`) behind `iss.pio_iss.Chip`. Idempotent.

    `fw/conftest.py` calls this once, before anything imports a driver, because
    `iss.pio_iss.ChipDevice()` and the two drivers that do `from iss.pio_iss import Chip` both take
    the chip from that one name. Restoring the golden model is always possible: the name is
    captured here, before the first swap.
    """
    name = name or current()
    from iss import pio_iss

    _GOLDEN.setdefault("Chip", pio_iss.Chip)
    if name == "rtl":
        from fw import rtl_harness

        rtl_harness.install()                      # it insists on FW_TARGET=rtl, and has a say
        return pio_iss.Chip                        # `install()` did the swap
    cls = TARGETS[name].load()
    pio_iss.Chip = cls
    return cls


def availability(name: str) -> tuple[bool, str]:
    """`(usable here, why)` for one target: what `verif/contract/` and `python -m fw.targets` show.

    Loading a target must have no side effects, so this is `Target.load()`, not `install()`: no
    `iss.pio_iss.Chip` is rebound and the `FW_TARGET=rtl` guard in `fw/rtl_harness.install()` is
    not involved.
    """
    try:
        TARGETS[name].load()
    except TargetUnavailable as exc:
        return False, str(exc)
    except Exception as exc:                        # a broken harness is "not available", loudly
        return False, f"{type(exc).__name__}: {exc}"
    return True, "available"


def available() -> dict[str, tuple[bool, str]]:
    """`(usable here, why)` for every target, in registry order."""
    return {name: availability(name) for name in NAMES}


def _main() -> int:
    print(f"{ENV}={os.environ.get(ENV, DEFAULT)!r}: this process would install "
          f"{current()!r}\n")
    for t in TARGETS.values():
        ok, why = availability(t.name)
        print(f"  {t.name:6} {'fast' if t.fast else 'slow':4}  {t.what}\n"
              f"          {why}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())

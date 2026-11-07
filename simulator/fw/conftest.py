"""Put the `FW_TARGET` backend behind `iss.pio_iss.Chip` once, before any driver is imported.

`fw/*/driver.py` and the 14 `fw/*/test_*.py` files take their chip from one name,
`iss.pio_iss.Chip` (`fw/rtl_harness.py` puts it there for `FW_TARGET=rtl`). This module runs at
collection time, which is before any test module is imported, and is therefore the earliest place
in the session where that name can be bound to the right class. Nothing else in `fw/` installs a
target: the per-file `_install_rtl` blocks and `skipif(FW_TARGET not in (...))` marks they replace
were the reason a typo in `FW_TARGET` used to report green with nothing tested.

    FW_TARGET=iss    pytest -q fw          # the golden ISS
    FW_TARGET=iss_b  pytest -q fw          # the second ISS: a third independent check
    FW_TARGET=rtl    pytest -q fw          # src/pio_chip.sv under cocotb
    FW_TARGET=chip   pytest -q fw          # loud failure until fw/chip_harness.py exists

A target that is *known* but cannot run here (a missing tool, a backend not built yet) is an error
too, raised as `TargetUnavailable`: `FW_TARGET=chip` today stops the session rather than skipping
14 protocol tests and calling it a pass. Unavailable targets are still listed, with the reason, by
`python -m fw.targets` and by `verif/contract/`.

See `fw/README-targets.md` and `docs/backend-contract.md`.
"""
from __future__ import annotations

from fw import targets

#: This session's target name (`FW_TARGET`, validated). A test that needs to know which backend it
#: is running on -- rather than how fast it is -- reads this; `targets.is_fast()` is the usual way.
TARGET = targets.current()

try:
    #: The class installed behind `iss.pio_iss.Chip`, for the rare test that has to name it.
    CHIP = targets.install(TARGET)
except targets.TargetUnavailable as exc:
    raise targets.TargetUnavailable(
        f"FW_TARGET={TARGET!r} is a target but it cannot run here: {exc}\n"
        f"The targets and what they need are in fw/targets.py; `python -m fw.targets` lists "
        f"which of them work on this machine.") from exc

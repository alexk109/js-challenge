# `fw/targets.py`: which chip answers the bus

Every firmware test in `fw/` takes its chip from one name, `iss.pio_iss.Chip`, and everything
else -- the host queue `iss.pio_iss.ChipDevice`, the `verif.vip.core.Bus`, the 14 protocol VIPs and
checkers -- from the real code path. `fw/targets.py` is the registry of what that name can point
at, and `fw/conftest.py` points it at the one `FW_TARGET` asks for, once per pytest session,
before any driver is imported.

    FW_TARGET=iss    python -m pytest -q fw      # the golden ISS (default)
    FW_TARGET=iss_b  python -m pytest -q fw      # the second ISS: a third independent check
    FW_TARGET=rtl    python -m pytest -q fw      # src/pio_chip.sv under cocotb + Verilator
    FW_TARGET=chip   python -m pytest -q fw      # loud failure until fw/chip_harness.py exists
    python -m fw.targets                          # the list, and what each needs here

An unknown target is an error, never a skip. `FW_TARGET=is` stops the session with a message
naming the four real targets, because the alternative -- the state this registry replaced -- was a
typo reporting green with nothing tested.

## What each target is

| `FW_TARGET` | What answers the bus | Speed |
|---|---|---|
| `iss` | `iss/pio_iss.Chip`, the golden model | `fast` |
| `iss_b` | `verif/iss_b/pio_iss_b.py`, written blind from the same spec, behind the adapter in `fw/targets.py` | `fast` |
| `rtl` | `src/pio_chip.sv` under cocotb + Verilator, driven cycle by cycle over a pair of pipes by `fw/rtl_harness.py` | `slow` |
| `chip` | the TT top level over the real SPI host port (not available yet) | `slow` |

`fast` means "Python model speed", and that is all it means: `fw/targets.py::is_fast()` is how a
test asks whether it can afford millions of cycles. `fw/wiegand/test_wiegand.py` is the example: a
whole Wiegand frame is 1.3 to 2.6 million core cycles, which is ~25 s on a model and minutes
through the RTL harness's pipes, so its `LONG` mark asks for a `fast` target rather than for a
particular identity -- which is why `FW_TARGET=iss_b` runs those tests and `FW_TARGET=rtl` skips
them.

## The adapter ISS-B needs, and why it is that small

`verif/iss_b` was written blind from `docs/mvp-isa.md` to the same *documented* API as the golden
ISS, so it has the same behaviour but not the same *names* for everything. Two gaps, both handled
in `fw/targets.py::_iss_b()` and both mechanical:

* `Chip.eng` is the golden `Chip.engines`, so `chip.engines[e]` needs one property.
* there is no `peek()`. It is answered by performing a **real host read of that address on a copy
  of the chip** and taking the data back.

The second one is the interesting decision. The alternative -- reimplementing the read mux in the
adapter -- would make `peek()` agree with the golden ISS by construction and hide any disagreement
in the two models' register files. Doing the read for real means that if ISS-B's read mux and the
golden's ever diverge, `verif/contract/` finds it. The copy is what keeps `peek()` free: a host
read of `RXFn` pops that engine's RX FIFO at the end of its cycle, and on the copy the pop goes
nowhere. ISS-B's own convention is the one `peek()` promises anyway -- every host access in a cycle
sees the start-of-cycle state -- so the value is the one the chip would have returned in this
cycle.

## What a new backend has to provide

`docs/backend-contract.md`: the `Chip` API of `docs/mvp-interfaces.md` `## ISS API`, plus the
observation surface the firmware tests read off a chip and off an engine (`engines[e].imem`,
`.tx`, `.rx`, `.pc`, the four configuration words, `peek`, `fdebug`, `sm_enable`,
`pin_out`, `pin_oe`), plus the register file and the ISA. It is measured by
`verif/contract/measure_surface.py` and checked by `verif/contract/`.

Adding one is: a module with a `Chip` class, one `Target(...)` line in `TARGETS`, then
`pytest -q verif/contract` and `FW_TARGET=<name> pytest -q fw`. No test file changes.

## The tests that were taken apart to make this

Before, each of the 14 `fw/*/test_*.py` files had its own

```python
if FW_TARGET == "rtl":
    from fw.rtl_harness import install as _install_rtl
    _install_rtl()

pytestmark = pytest.mark.skipif(FW_TARGET not in ("iss", "rtl"), ...)
```

-- 201 lines of the same decision in 14 places, and the reason `FW_TARGET=nonsense` used to report
green with every test skipped. `fw/conftest.py` does it once, at collection time, which is before
any test module is imported and therefore before `fw/necir/driver.py` and `fw/uart_rx/driver.py`
get to do `from iss.pio_iss import Chip`.

What stayed is target-specific *test* logic rather than install boilerplate: the
`FW_RTL_MAX_SIMS=12` knob in `fw/quadrature/test_quadrature.py` (that file keeps six or more chips
alive at once on the RTL, and the harness pools four simulators by default), and the nested
`FW_TARGET=rtl` re-run of `fw/spi`'s own module inside
`test_the_rtl_target_runs_the_same_module`.

## Results

Over all of `fw/`, before and after this change, on three targets, with the same 456 collected
tests in every run:

| Run | Result |
|---|---|
| `FW_TARGET=iss`, before | 455 passed, 1 skipped in 789.90s |
| `FW_TARGET=iss`, after | 455 passed, 1 skipped in 515.76s |
| `FW_TARGET=iss_b`, after | 455 passed, 1 skipped in 626.68s |
| `FW_TARGET=rtl`, before | 441 passed, 15 skipped in 7915.10s |
| `FW_TARGET=rtl`, after | 441 passed, 15 skipped in 8045.86s |

The RTL pair is the interesting one: identical results on both sides -- the same 441 passes and
the same 15 skips (13 `fw/wiegand` LONG cases that a slow target declines, the nested `fw/spi`
re-run that declines under `rtl` to avoid recursing, and `fw/ws2812`'s own chain-latency case) --
in 2:11:55 before and 2:14:05 after. Taking the per-file boilerplate out did not cost the RTL
anything; the 2.2% is machine noise, and both runs were on the same shared box at the same time.

## Files

| Path | What |
|---|---|
| `fw/targets.py` | the registry: `TARGETS`, `current()`, `is_fast()`, `install()`, `availability()`; run `python -m fw.targets` |
| `fw/conftest.py` | installs `FW_TARGET` once per pytest session, loudly if it cannot run here |
| `fw/rtl_harness.py` | the `rtl` backend |

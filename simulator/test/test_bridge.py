# SPDX-License-Identifier: Apache-2.0
#
# M-014: the pin-level Python devices of verif/vip/ drive the real RTL in cocotb
# through verif/vip/cocotb_bridge.py.
#
# One test:
#   test_pin_rules runs the Bridge against a fake DUT made of plain Python objects,
#     for the rules no real DUT can show on demand: uio_oe turning a pin from
#     output to input, pull-ups, open-drain wired-AND and push-pull contention
#     between the DUT and a device, and a pin map the bridge has to refuse.
#
# The half of M-014 that ran against the real top level -- a Device receiving from
# the UART-TX line and a Device driving the bits the stub echoed -- went away with
# the flow bring-up stub (`stub_uart_tx` was replaced with
# `host_spi` + `pio_chip`).  The bridge still runs against the real RTL, on the real
# pin map, in `test.py`: `SpiController` on `ui_in[2:0]`/`uo_out[0]`, `UartRx` on a
# uio pin and `I2cEepromTarget` + `I2cTimingChecker` on two more, all through one
# Bridge.

import sys
from pathlib import Path

# cocotb imports this module with test/ as its working directory, so put the
# repo root on sys.path before reaching for verif.vip.
_ROOT = str(Path(__file__).resolve().parents[1])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import cocotb

from verif.vip.cocotb_bridge import Bridge
from verif.vip.core import Device

# ---------------------------------------------------------------- devices


class FixedDriver(Device):
    """Drives fixed values, whatever the pins read."""

    name = "fixed_driver"

    def __init__(self, name: str, drives: dict):
        self.name = name
        self.drives = dict(drives)

    def tick(self, t: int, pins) -> dict:
        return dict(self.drives)


# ---------------------------------------------------------------- helpers


class _Sig:
    """Stand-in for a cocotb signal handle."""

    def __init__(self, value=0):
        self.value = value


class FakeDut:
    """A DUT of plain Python signals, so the pin rules need no simulator."""

    def __init__(self, **ports):
        self.clk = _Sig(0)
        for name, value in ports.items():
            setattr(self, name, _Sig(value))


# ---------------------------------------------------------------- test


@cocotb.test()
async def test_pin_rules(_dut):
    """Direction from uio_oe, pull-ups, open-drain wired-AND and contention.

    Runs against FakeDut rather than the real dut, so it needs no clock.
    """
    fake = FakeDut(ui_in=0x40, uio_in=0x00, uo_out=0b0000_1000,
                   uio_out=0b0000_0010, uio_oe=0b0000_0010)
    bridge = Bridge(fake, {"tx": ("uo_out", 3), "miso": ("uio", 1),
                           "scl": ("uio", 0), "rx": ("ui_in", 5)})
    bridge.set_net("tx", push_pull=True)
    bridge.set_net("miso", push_pull=True)   # scl stays open drain

    # The DUT drives tx and miso; scl and rx have no driver, so their pull-ups
    # reach the input pins. ui_in[6] is not in the map and stays as the test left it.
    bridge.tick()
    assert bridge.dut_drive == {"tx": 1, "miso": 1, "scl": None, "rx": None}, bridge.dut_drive
    assert (bridge.net("tx"), bridge.net("miso")) == (1, 1)
    assert fake.uio_in.value == 0b0000_0001, bin(fake.uio_in.value)
    assert fake.ui_in.value == 0b0110_0000, bin(fake.ui_in.value)

    # An open-drain device pulls scl low; releasing it brings the pull-up back.
    bridge.attach(FixedDriver("od_low", {"scl": 0}))
    bridge.tick()
    assert fake.uio_in.value == 0b0000_0000, bin(fake.uio_in.value)
    # A second device driving 1 does not fight it: wired-AND, and an open-drain
    # net records no contention however wrong the drivers are.
    bridge.attach(FixedDriver("od_high", {"scl": 1}))
    bridge.tick()
    assert bridge.net("scl") == 0, bridge.net("scl")
    assert fake.uio_in.value == 0b0000_0000, bin(fake.uio_in.value)
    assert not bridge.contentions, bridge.contentions

    # The DUT drives miso low while a device drives it high: the net reads 0 and
    # the push-pull net records a Contention naming both drivers.
    fake.uio_out.value = 0b0000_0000
    bridge.attach(FixedDriver("pp_hi", {"miso": 1}))
    bridge.tick()
    assert bridge.net("miso") == 0, bridge.net("miso")
    assert len(bridge.contentions) == 1, bridge.contentions
    err = bridge.contentions[0]
    assert (err.net, err.drivers) == ("miso", {"dut": 0, "pp_hi": 1}), err.drivers

    # Dropping uio_oe turns pin 1 into an input: the DUT releases it, so the net
    # follows its drivers and the bridge takes over the input path.
    fake.uio_oe.value = 0
    bridge.tick()
    assert bridge.dut_drive["miso"] is None, bridge.dut_drive
    assert fake.uio_in.value == 0b0000_0010, bin(fake.uio_in.value)

    # A pin map the bridge cannot make sense of is refused, not silently munged.
    # (uo_out[3] and uio_in[3] are different pins, so that map is fine.)
    for bad, why in (({"tx": ("uo_out", 9)}, "bit 9 outside"),
                     ({"tx": ("nope", 0)}, "unknown signal"),
                     ({"scl": ("uio_in", 0), "sda": ("uio", 0)}, "carries one net")):
        try:
            Bridge(fake, bad)
        except ValueError as exc:
            assert why in str(exc), f"{bad}: {exc}"
        else:
            raise AssertionError(f"pin map {bad} was accepted")

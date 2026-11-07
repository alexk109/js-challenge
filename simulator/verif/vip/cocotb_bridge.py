"""Let the pin-level devices of verif/vip/core.py drive and observe the real RTL.

One tick is one rising edge of ``dut.clk``, exactly like ``core.Bus.step()``. Per
tick the bridge

  1. samples the DUT's output pins as one more set of drivers on the nets:
     every ``uo_out`` bit, and every ``uio_out`` bit whose ``uio_oe`` bit is 1;
  2. calls ``tick()`` on every attached Device with the values resolved at the
     end of the previous tick, then resolves all drives by wired-AND;
  3. writes the resolved values onto the DUT's input pins: ``ui_in`` bits, and
     ``uio_in`` bits whose ``uio_oe`` bit is 0.

Steps 1 and 2 are ``core.Bus`` doing its usual thing with the DUT attached as one
more Device, so the two-phase rule of core.py holds for the DUT exactly as it
does for the devices (a device reacts a tick after it sees a change, like a flop
sampling its input), and a net that the DUT and a device drive with opposite
values resolves to 0 and is recorded as ``Contention``, as on any other
push-pull bus.

The pin map is a dict the test supplies: ``{net: (signal, bit)}``, where signal
is one of

    "uo_out"    dedicated output: sampled, never driven back
    "ui_in"     dedicated input: driven, never sampled
    "uio"       a bidirectional pin: sampled as ``uio_out[bit]`` while
                ``uio_oe[bit]`` is 1 and driven as ``uio_in[bit]`` while it is 0
    "uio_in"    the input half of a uio pin, for a map that already knows the
    "uio_out"   direction; the output half, likewise

Bits of ``ui_in`` / ``uio_in`` that no net maps to are left alone, so a test can
drive them itself (the host port lives on ``ui_in[2:0]``).

    bridge = Bridge(dut, {"tx": ("uo_out", 3), "sda": ("uio", 0)}, ns_per_tick=20.0)
    bridge.set_net("tx", push_pull=True)            # before start()
    bridge.attach(UartRx("tx", 434.0))
    cocotb.start_soon(bridge.start())               # kill the task to stop it
    ...
    bridge.net("tx"), bridge.bus.values, bridge.bus.errors

``ena`` is not interpreted: the DUT drives whatever its output pins show. TT
holds ``ena`` at 1 whenever the chip is selected, and the stub ignores it.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import Mapping, Optional

# The tests run with test/ as their working directory, so the repo root is not on
# sys.path; add it so `verif.vip.core` imports the same way from anywhere.
_ROOT = str(Path(__file__).resolve().parents[2])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge

from verif.vip.core import Bus, Device, Net

__all__ = ["Bridge"]

# TT's ports are 8 bits wide.
PIN_COUNT = 8

# signal -> (does the DUT drive it, does the bridge drive it, which handle carries it)
_SIGNALS = {
    "ui_in":   (False, True,  "ui_in"),
    "uo_out":  (True,  False, "uo_out"),
    "uio":     (True,  True,  "uio_in"),
    "uio_in":  (False, True,  "uio_in"),
    "uio_out": (True,  False, "uio_out"),
}


def _uint(handle) -> int:
    """The value of a signal as an int. Unknown (x/z) bits read as 0."""
    if handle is None:
        return 0
    value = getattr(handle, "value", None)
    if value is None:
        return 0
    try:
        return int(value)
    except Exception:  # BinaryValue with x/z in it has no int value
        bits = str(getattr(value, "binstr", value))
        return int(bits.translate(str.maketrans("xXzZuUwW", "00000000")), 2) or 0


class _DutDriver(Device):
    """The DUT as a Device: what its output pins drive on this tick.

    Riding on the Bus is what makes the DUT's drives resolve against the devices'
    drives by the same wired-AND rule and with the same Contention records, and
    what makes a device see a DUT output one tick late, like any other net value.
    """

    name = "dut"

    def __init__(self) -> None:
        self.drives: dict[str, Optional[int]] = {}

    def sample(self, drives: Mapping[str, Optional[int]]) -> None:
        self.drives = dict(drives)

    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        return dict(self.drives)


class Bridge:
    """Runs the devices of a verif/vip Bus against a cocotb DUT, one tick per clock."""

    def __init__(self, dut, mapping: Mapping[str, tuple], ns_per_tick: float = 20.0):
        self.dut = dut
        self.clk = getattr(dut, "clk", None)
        if self.clk is None:
            raise ValueError("the DUT has no clk to tick the bridge on")
        self.ns_per_tick = ns_per_tick
        self.mapping = _check_mapping(dut, mapping)
        self.bus = Bus(ns_per_tick=ns_per_tick)
        for net in self.mapping:
            self.bus.add_net(net)
        self.dut_driver = _DutDriver()
        self.bus.attach(self.dut_driver)
        #: What the DUT's output pins drove on the last tick (None = released).
        self.dut_drive: dict[str, Optional[int]] = {}
        # Input paths to write, keyed by the handle that carries them.
        self._in_nets: dict[str, list[tuple[str, int]]] = {"ui_in": [], "uio_in": []}
        self._needs_oe = False
        for net, (signal, bit) in self.mapping.items():
            if not _SIGNALS[signal][1]:
                continue
            handle = _SIGNALS[signal][2]
            self._in_nets[handle].append((net, bit))
            self._needs_oe = self._needs_oe or handle == "uio_in"
        self._seen_errors = 0
        self._log = logging.getLogger("cocotb.bridge")

    # -- nets and devices -------------------------------------------------

    def add_net(self, name: str, push_pull: bool = False, pullup: int = 1) -> Net:
        """Add a net that is not in the pin map (an observation net, say)."""
        return self.bus.add_net(name, push_pull, pullup)

    def set_net(self, name: str, push_pull: Optional[bool] = None,
                pullup: Optional[int] = None) -> Net:
        """Set attributes of a net, e.g. push_pull=True for a UART line."""
        net = self.bus.nets.get(name)
        if net is None:
            net = self.bus.add_net(name)
        if push_pull is not None:
            net.push_pull = push_pull
        if pullup is not None:
            net.pullup = pullup
        return net

    def attach(self, device: Device) -> Device:
        """Attach a Device, the way Bus.attach does."""
        return self.bus.attach(device)

    def net(self, name: str) -> int:
        """The value a net resolved to at the end of the last tick."""
        return self.bus.values[name]

    @property
    def contentions(self) -> list:
        """Nets where the DUT and a device (or two devices) drove opposites."""
        return self.bus.errors

    # -- the tick ---------------------------------------------------------

    def tick(self) -> dict[str, int]:
        """One whole tick: sample the DUT, resolve, drive. No simulator needed."""
        values = self.step()
        self.drive()
        return values

    def step(self) -> dict[str, int]:
        """Sample the DUT's outputs, then let every device tick and resolve."""
        self.dut_drive = self._sample()
        self.dut_driver.sample(self.dut_drive)
        values = self.bus.step()
        for err in self.bus.errors[self._seen_errors:]:
            self._log.warning("bridge: contention on net %r at tick %d: %s",
                              err.net, err.t, err.drivers)
        self._seen_errors = len(self.bus.errors)
        return values

    def drive(self) -> None:
        """Push the values resolved by the last step onto the DUT's input pins.

        Only bits a net maps to are written, so the test keeps the rest of
        ``ui_in`` for itself, and the word is only written when it changes, to
        keep the VCD free of writes that say nothing.
        """
        values = self.bus.values
        oe = _uint(getattr(self.dut, "uio_oe", None)) if self._needs_oe else 0
        for handle_name, nets in self._in_nets.items():
            if not nets:
                continue
            handle = getattr(self.dut, handle_name, None)
            now = _uint(handle)
            word = now
            for net, bit in nets:
                if handle_name == "uio_in" and ((oe >> bit) & 1):
                    continue  # the pin is an output: its input path is ignored
                word = (word & ~(1 << bit)) | ((int(values[net]) & 1) << bit)
            if handle is not None and word != now:
                handle.value = word

    async def start(self) -> None:
        """Coroutine: one tick per clock edge, until the task is killed.

        Sampling waits for ReadOnly, so what is read is what the DUT registered
        at that edge. The inputs are written half a cycle later, at the falling
        edge, so the DUT samples them at the next rising edge with a full half
        period of setup. That is the one tick of latency core.py gives every
        driver, and it keeps the bridge clear of the read/write-on-the-same-edge
        race that simulator scheduling would otherwise decide.
        """
        self._log.info("bridge: nets %s, %d device(s) besides the DUT",
                       sorted(self.mapping), len(self.bus.devices) - 1)
        self.drive()  # the pull-ups reach the input pins before the first edge
        while True:
            await RisingEdge(self.clk)
            await ReadOnly()
            self.step()
            await FallingEdge(self.clk)
            self.drive()

    # -- internals --------------------------------------------------------

    def _sample(self) -> dict[str, Optional[int]]:
        """What the DUT's output pins drive this tick, per net (None = released)."""
        uo = _uint(getattr(self.dut, "uo_out", None))
        uio_out = _uint(getattr(self.dut, "uio_out", None))
        oe = _uint(getattr(self.dut, "uio_oe", None))
        drives: dict[str, Optional[int]] = {}
        for net, (signal, bit) in self.mapping.items():
            if signal == "uo_out":
                drives[net] = (uo >> bit) & 1
            elif _SIGNALS[signal][0] and ((oe >> bit) & 1):
                drives[net] = (uio_out >> bit) & 1
            else:
                drives[net] = None  # an input pin: the DUT does not drive the net
        return drives


def _check_mapping(dut, mapping: Mapping[str, tuple]) -> dict[str, tuple]:
    """Validate the pin map and normalise it to {net: (signal, bit)}."""
    if not mapping:
        raise ValueError("the pin map is empty: name at least one net")
    out: dict[str, tuple] = {}
    owners: dict[tuple, str] = {}
    for net, pin in mapping.items():
        try:
            signal, bit = pin
        except (TypeError, ValueError):
            raise ValueError(
                f"pin map entry {net!r}: expected (signal, bit), got {pin!r}") from None
        signal = str(signal).lower()
        if signal not in _SIGNALS:
            raise ValueError(f"pin map entry {net!r}: unknown signal {signal!r}, "
                             f"expected one of {', '.join(_SIGNALS)}")
        bit = int(bit)
        if not 0 <= bit < PIN_COUNT:
            raise ValueError(f"pin map entry {net!r}: bit {bit} outside "
                             f"0..{PIN_COUNT - 1}")
        if getattr(dut, _SIGNALS[signal][2], None) is None and getattr(dut, signal, None) is None:
            raise ValueError(f"pin map entry {net!r}: the DUT has no {signal!r} port")
        pin_key = ("uio", bit) if signal.startswith("uio") else (signal, bit)
        if pin_key in owners:
            raise ValueError(f"nets {owners[pin_key]!r} and {net!r} both map to "
                             f"{signal}[{bit}]: a pin carries one net")
        owners[pin_key] = net
        out[net] = (signal, bit)
    return out

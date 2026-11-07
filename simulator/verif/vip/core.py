"""Pin-level, tick-based simulation core shared by every VIP, the ISS harness and cocotb.

One tick is one chip clock (default 20 ns at 50 MHz). Every net is wired-AND with a
pull-up: a net reads 0 if any device drives 0, else 1 if any device drives 1, else the
pull-up value (1). That covers open-drain buses (I2C) and push-pull lines (UART, SPI)
with one rule. Two devices driving opposite values on a push-pull net is contention and
is recorded as an error.

Each tick has two phases, so device order never matters:
  1. every device sees the same snapshot (the values resolved at the end of last tick)
     and returns what it drives now: {net: 0 | 1 | None}; None or absent = released;
  2. the bus resolves all drives into the next snapshot.
So a device reacts one tick after it sees a change, like a flop sampling its input.

    bus = Bus(ns_per_tick=20)
    bus.add_net("tx")
    bus.attach(UartTx(...)); bus.attach(UartRx(...))
    bus.run(10_000)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional


class Device:
    """Base class. Override tick(); keep all state inside the device."""

    name: str = "device"

    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        """Called once per tick with the resolved pin values. Return drives."""
        return {}


@dataclass
class Net:
    name: str
    push_pull: bool = False   # True: opposite drivers on the same tick is contention
    pullup: int = 1


@dataclass
class Contention:
    t: int
    net: str
    drivers: dict[str, int] = field(default_factory=dict)


class Bus:
    def __init__(self, ns_per_tick: float = 20.0, trace: bool = False, tap=None):
        self.ns_per_tick = ns_per_tick
        self.nets: dict[str, Net] = {}
        self.devices: list[Device] = []
        self.values: dict[str, int] = {}
        self.t = 0
        self.errors: list[Contention] = []
        self.trace: Optional[list[tuple[int, dict[str, int]]]] = [] if trace else None
        #: Optional observer, called at the end of every step() as
        #: `tap(t, drives, values)`: the tick just resolved, what every device drove that
        #: tick ({net: {device name: level}}) and the levels the resolution produced. It
        #: drives nothing and its return value is ignored, so attaching one cannot change
        #: what any device sees. `tools/waves/` uses it to record which pin was driven by
        #: whom per tick; `trace=True` is the older, level-only way.
        self.tap = tap

    def add_net(self, name: str, push_pull: bool = False, pullup: int = 1) -> Net:
        net = Net(name, push_pull, pullup)
        self.nets[name] = net
        self.values[name] = pullup
        return net

    def attach(self, device: Device) -> Device:
        self.devices.append(device)
        return device

    def ticks(self, ns: float) -> int:
        """Nanoseconds to whole ticks, rounded up (use for minimum timing)."""
        return -int(-ns // self.ns_per_tick)

    def step(self) -> dict[str, int]:
        snapshot = dict(self.values)
        drives: dict[str, dict[str, int]] = {n: {} for n in self.nets}
        for dev in self.devices:
            for net, v in (dev.tick(self.t, snapshot) or {}).items():
                if v is None:
                    continue
                if net not in self.nets:
                    raise KeyError(f"{dev.name} drives unknown net {net!r}")
                drives[net][getattr(dev, "name", type(dev).__name__)] = 1 if v else 0
        for name, net in self.nets.items():
            d = drives[name]
            if not d:
                self.values[name] = net.pullup
            elif 0 in d.values():
                self.values[name] = 0
                if net.push_pull and 1 in d.values():
                    self.errors.append(Contention(self.t, name, d))
            else:
                self.values[name] = 1
        if self.trace is not None:
            self.trace.append((self.t, dict(self.values)))
        if self.tap is not None:
            self.tap(self.t, drives, self.values)
        self.t += 1
        return self.values

    def run(self, n: int) -> None:
        for _ in range(n):
            self.step()

    def run_until(self, cond, limit: int) -> bool:
        """Step until cond(bus) is true. False if limit ticks pass first."""
        for _ in range(limit):
            if cond(self):
                return True
            self.step()
        return cond(self)

    def write_vcd(self, path: str) -> None:
        """Dump the trace (Bus(trace=True)) as a VCD for GTKWave."""
        if self.trace is None:
            raise RuntimeError("create the Bus with trace=True")
        ids = {n: chr(33 + i) for i, n in enumerate(self.nets)}
        with open(path, "w") as f:
            f.write(f"$timescale {int(self.ns_per_tick)}ns $end\n$scope module bus $end\n")
            for n, i in ids.items():
                f.write(f"$var wire 1 {i} {n} $end\n")
            f.write("$upscope $end\n$enddefinitions $end\n")
            last: dict[str, int] = {}
            for t, vals in self.trace:
                changed = [(n, v) for n, v in vals.items() if last.get(n) != v]
                if changed:
                    f.write(f"#{t}\n" + "".join(f"{v}{ids[n]}\n" for n, v in changed))
                    last.update(changed)

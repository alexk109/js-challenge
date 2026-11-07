"""Instrumentation for the measured MVP performance table.

Nothing here is a model of the chip and nothing here is a number taken from a datasheet: it is
the *measurement* apparatus that the 14 protocol benches of `fw/` are instrumented with, so that
every figure in `research/perf/b/perf.csv` is a reading of a simulation rather than a formula.

Three records come out of one bench, all of them per core cycle and all of them on the bus the
firmware actually drives:

* **the edge log** -- every net that changes value, timestamped with the core cycle in which the
  new level is first visible on the net.  That is the cycle after the one whose end produced it,
  because `verif.vip.core.Bus` resolves the drives at the end of a tick and hands the resolved
  snapshot to the devices on the next one.  It is also what a scope on a real board would see:
  the pad changes at the clock edge that ends the cycle.
* **the bus-access log** -- every register access the host made, timestamped with the core cycle
  in which it was *performed*: the bus access cycle `h` of `docs/mvp-regmap.md`
  `### Cycle-exact mapping to bus accesses`.
* **the engine samples** -- one sample per core cycle of "did the engine run an SM cycle", and
  of the two FIFO levels.  The SM-cycle count is taken two ways and cross-checked: on the ISS by
  counting calls of `Engine.sm_step` (an exact count), and on any target that *exposes* the RTL's
  own `sm_cycle = forced || (sm_en && div_en)` (`src/pio_engine.sv`), which is the same predicate.
  `Probe.sm_agree` says whether the two agree, so a disagreement cannot be quoted.

  A target that does not expose that signal is a third case, and the distinction matters: the
  `fw/rtl_harness.py` engine facade (`_RtlEngine.__getattr__`) answers `STATE_KEYS`, the six SM
  configuration words, `imem`, `tx` and `rx`, and raises `AttributeError` for anything else --
  `sm_cycle` is one of the "anything else".  So on `FW_TARGET=rtl` the SM-cycle metric is
  **unavailable**, not zero, and `Probe.sm_available` says which; `Trial.rows()` then writes
  `n/a` and says why.  Nothing here reconstructs the predicate from `CTRL` and `CLKDIV`: that
  would be a Python model of the gate printed in the shape of a measurement of it.

Attaching is deliberately in-place and reversible: `attach()` wraps the *instance* methods of the
`Bus` and of the `ChipDevice` the bench already built, so a bench, a driver, a program and the
judging VIP are all exactly the ones the firmware's own tests use -- this file adds no cycle and
removes none.  It never edits `src/`, `iss/`, `verif/vip/` or any `fw/` module.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional, Sequence

from verif.vip.core import Device

#: 50 MHz core clock, `docs/mvp-regmap.md` `### Physical layer` (T = 20 ns).
NS_PER_TICK = 20.0
F_CLK_HZ = 1.0e9 / NS_PER_TICK
#: docs/mvp-regmap.md `### Address map`: CTRL is the word address whose write starts a transaction.
A_CTRL = 0x00


def fifo_depth() -> int:
    """FIFO depth of the active chip (4 at the MVP), read at use time: the map is fixed at target install."""
    from fw import regmap

    return regmap.fifo_depth()


@dataclass(frozen=True)
class Edge:
    """One net changing value: the cycle it is first visible in, and the level it became."""

    t: int
    net: str
    level: int


@dataclass(frozen=True)
class Access:
    """One register bus access: the cycle `h` it was performed in, its kind, address and data."""

    t: int
    kind: str          # "w" or "r"
    addr: int
    data: int = 0


def hz(cycles: float) -> float:
    """Core cycles to Hz, at the MVP's 50 MHz (docs/mvp-regmap.md `### Physical layer`)."""
    return F_CLK_HZ / cycles


def ns(cycles: float) -> float:
    """Core cycles to nanoseconds."""
    return cycles * NS_PER_TICK

class Probe(Device):
    """The measurement apparatus, attached to a bench that is already built.

    `attach(bus, device, chip)` returns it and also appends it to `bus.devices`, so it samples
    once per core cycle *after* the chip has stepped in that cycle (it is the last device, and
    `Bus.step` walks `devices` in order): every sample is the state at the end of that cycle.

    `nets` is the protocol's own pin nets. It matters: pins 13..17 are output-only and read
    `pin_oe = 1` from reset, so the chip drives them low on the very first tick and every "first
    pin edge" would be one of those. Only the nets a bench's program actually uses are counted.
    """

    name = "perf_probe"

    def __init__(self, bus, device, chip, *, engine: int = 0, nets: Sequence[str] = (),
                 chip_only: bool = True):
        self.bus = bus
        self.device = device
        self.chip = chip
        self.chip_only = chip_only
        self.engine = engine
        self.nets = tuple(nets)
        self.edges: list[Edge] = []
        self.accesses: list[Access] = []
        self.sm: list[int] = []                 # 1 in a cycle the engine ran an SM cycle
        self.tx_level: list[int] = []
        self.rx_level: list[int] = []
        self.sm_calls = 0                       # exact count, ISS only: Engine.sm_step calls
        self._sm_prev = 0
        self._sm_source = "unmetered"
        self.sm_available = False               # see `_sample_sm_cycle`: False means "no reading"
        self._sm_tried = False                  # has the first `sm_cycle` read happened yet?
        self._sm_read: Optional[int] = None     # ... and what it said
        self._unwrapped: list[Callable] = []

    # ---- the two unwrapping steps -----------------------------------------------------

    def _install(self, obj: object, attr: str, wrapper) -> None:
        """Wrap one *instance* method, remembering how to put it back."""
        original = getattr(obj, attr)
        setattr(obj, attr, wrapper(original))
        self._unwrapped.append((obj, attr, original))

    def _meter_iss(self) -> None:
        """Count `Engine.sm_step` calls exactly, on the ISS."""
        engine = getattr(self.chip, "engines", None)
        if engine is None or not hasattr(engine[self.engine], "sm_step"):
            return
        probe = self

        def wrapper(original):
            def sm_step(*args, **kwargs):
                probe.sm_calls += 1
                return original(*args, **kwargs)

            return sm_step

        self._install(engine[self.engine], "sm_step", wrapper)
        self._sm_source = "Engine.sm_step"

    def _read_sm_cycle(self) -> Optional[int]:
        """The RTL's own `sm_cycle` signal, or None if this target does not expose it.

        The first read decides, and the answer is remembered: if the facade raises, this is a
        target that does not publish the signal, and every later read would raise the same way,
        so the reason is recorded once and the per-cycle cost is paid once.
        """
        if self._sm_tried:
            return self._sm_read
        self._sm_tried = True
        handle = getattr(self.chip, "engines", None)
        if handle is None:
            self._sm_source = f"unavailable: {type(self.chip).__name__} has no `engines`"
            return None
        engine = handle[self.engine]
        try:
            self._sm_read = 1 if int(engine.sm_cycle) else 0
        except (AttributeError, TypeError, ValueError) as exc:
            self._sm_source = (f"unavailable: {type(engine).__name__} exposes no `sm_cycle` "
                               f"({type(exc).__name__}: {exc}); this metric is ISS-only")
            return None
        self._sm_source = "sm_cycle"
        return self._sm_read

    def _sample_sm_cycle(self) -> int:
        """1 or 0 for "the engine ran an SM cycle this cycle"; 0 when it was never read.

        The 0 for "no reading" is only ever a placeholder in `self.sm`: `sm_available` is the
        flag that decides whether the metric may be published at all, so a missing signal can
        never be counted, averaged or printed as a measurement.
        """
        if self._sm_source == "Engine.sm_step":
            self.sm_available = True
            return 1 if self.sm_calls > self._sm_prev else 0
        value = self._read_sm_cycle()
        if value is None:
            return 0
        self.sm_available = True
        return value

    # ---- per-cycle sampling -----------------------------------------------------------

    def tick(self, t: int, pins) -> dict:
        if self.chip_only:
            engine = self.chip.engines[self.engine]
            self.sm.append(self._sample_sm_cycle())
            self._sm_prev = self.sm_calls
            self.tx_level.append(len(engine.tx) if hasattr(engine, "tx")
                                 else int(engine.tx_level))
            self.rx_level.append(len(engine.rx) if hasattr(engine, "rx")
                                 else int(engine.rx_level))
        return {}

    # ---- undo (so a pooled bench can be handed on) -------------------------------------

    def detach(self) -> None:
        for obj, attr, original in reversed(self._unwrapped):
            setattr(obj, attr, original)
        self._unwrapped.clear()
        if self in self.bus.devices:
            self.bus.devices.remove(self)

    # ---- the three records -------------------------------------------------------------

    def first_edge(self, t0: int, nets: Optional[Iterable[str]] = None) -> Optional[Edge]:
        """The first edge at or after `t0` on `nets` (the protocol's nets by default)."""
        wanted = self.nets if nets is None else tuple(nets)
        for edge in self.edges:
            if edge.t >= t0 and edge.net in wanted:
                return edge
        return None

    def last_edge(self, nets: Optional[Iterable[str]] = None) -> Optional[Edge]:
        """The last edge on `nets`, whatever cycle it was in."""
        wanted = self.nets if nets is None else tuple(nets)
        found = None
        for edge in self.edges:
            if edge.net in wanted:
                found = edge
        return found

    def edges_on(self, nets: Optional[Iterable[str]] = None) -> list[Edge]:
        wanted = self.nets if nets is None else tuple(nets)
        return [e for e in self.edges if e.net in wanted]

    def window(self, nets: Optional[Iterable[str]] = None) -> tuple[Optional[Edge], Optional[Edge]]:
        """(first, last) edge on `nets`: the payload window a bit rate is measured over."""
        on = self.edges_on(nets)
        return (on[0], on[-1]) if on else (None, None)

    def sm_cycles(self, t0: int, t1: int) -> int:
        """SM cycles the engine ran in `[t0, t1]`, inclusive, from the per-cycle samples."""
        return sum(self.sm[t0 : t1 + 1]) if self.sm else 0

    def sm_agree(self) -> Optional[bool]:
        """Do the per-cycle samples add up to the exact `sm_step` count? None if not metered."""
        if self._sm_source != "Engine.sm_step":
            return None
        return sum(self.sm) == self.sm_calls

    def sm_reason(self) -> str:
        """Why the SM-cycle samples are or are not a reading: for the CSV's `source` column."""
        return f"counted by counting {self._sm_source}" if self.sm_available else self._sm_source

    def peak_levels(self, t0: int, t1: int) -> tuple[int, int]:
        """(peak TX level, peak RX level) over `[t0, t1]`."""
        tx = max(self.tx_level[t0 : t1 + 1], default=0)
        rx = max(self.rx_level[t0 : t1 + 1], default=0)
        return tx, rx

    def first_ctrl_enable(self) -> Optional[Access]:
        """The `CTRL` write that switches an engine on: the one the transaction starts from.

        `CTRL` is the only register whose write both enables and restarts (docs/mvp-regmap.md
        `### Global registers`), so the write with `SM_ENABLE != 0` in its data is the host's
        "go", and the latency is measured from it.  Engine `e`'s bits are at
        `4e`, so any engine's enable counts -- this is the first one the host ever writes.
        """
        for access in self.accesses:
            if access.kind == "w" and access.addr == A_CTRL and access.data & 0x3:
                return access
        return None

    def writes_to(self, addr: int) -> list[Access]:
        return [a for a in self.accesses if a.kind == "w" and a.addr == addr]


def sm_fields(probe: Probe, t0: int, t1: int) -> dict:
    """The three `Trial` fields the SM-cycle samples feed, as one `**` splat.

    Every rig passes this straight into its `Trial(...)`, so whether the samples were a reading
    or nothing at all travels with the count and `Trial.rows()` cannot publish a 0 for a signal
    nobody could read.  Read it after the run: availability is only decided once the samples are.
    """
    return dict(sm_cycles=probe.sm_cycles(t0, t1), sm_available=probe.sm_available,
                sm_source=probe.sm_reason())


# ---------------------------------------------------------------------------------------------
# Attaching: the wrappers that turn a bench into a measured one
# ---------------------------------------------------------------------------------------------


def attach(bus, device, chip, *, engine: int = 0, nets: Sequence[str] = (),
           chip_only: bool = True) -> Probe:
    """Instrument a bench in place and return the `Probe` that reads it.

    `bus` is the `verif.vip.core.Bus` the bench built, `device` the `ChipDevice` on it (whose
    queue is where the host's accesses come from) and `chip` what that device wraps.  The two
    wrappers are the whole trick:

    * `Bus.step` is wrapped to diff the resolved net values, which is the edge log;
    * `ChipDevice.tick` is wrapped to note the queue entry it is about to perform, which is the
      bus-access log -- the device pops one access per tick and performs it in that tick, so the
      tick it is called with *is* the bus access cycle `h`.

    Nothing else is touched, so the bench that comes back is the bench the firmware tests run.
    """
    probe = Probe(bus, device, chip, engine=engine, nets=nets, chip_only=chip_only)

    def step_wrapper(original):
        def step():
            before = dict(bus.values)
            values = original()
            t = bus.t                      # first cycle the resolved values are visible in
            for net, level in values.items():
                if before[net] != level:
                    probe.edges.append(Edge(t, net, level))
            return values

        return step

    def tick_wrapper(original):
        def tick(t, pins):
            # `device` is normally the bench's `ChipDevice`, whose queue holds the host's
            # accesses as `(kind, addr, data, future)`.  A bench whose instrumented device has
            # no such queue (the host-port bench watches an SPI controller) simply has no
            # register accesses to log.
            queue = getattr(device, "queue", None)
            entry = queue[0] if queue else None
            if isinstance(entry, tuple) and len(entry) == 4:
                kind, addr, data, _future = entry
                probe.accesses.append(Access(t, kind, addr, data or 0))
            return original(t, pins)

        return tick

    probe._install(bus, "step", step_wrapper)
    probe._install(device, "tick", tick_wrapper)
    probe._meter_iss()
    bus.attach(probe)
    return probe


# ---------------------------------------------------------------------------------------------
# One trial: a protocol, one setting of its speed knob, and what was measured
# ---------------------------------------------------------------------------------------------


@dataclass
class Trial:
    """One measured run: one protocol at one value of its speed knob.

    Every field is either measured by a `Probe` or copied straight out of the protocol's own
    VIP (the checker that judged the run, and the decoder that counted the bits).  No field is
    derived from a formula in `docs/`; where a datasheet limit is being compared against, it is
    in the *verdict*, never in the measurement.
    """

    protocol: str
    knob: str                       # the speed knob this protocol exposes ("clkdiv", "baud", ...)
    value: float                    # its value at this run
    unit: str                       # the unit of `value`
    ok: bool                        # did the protocol's own checker pass with no violations?
    why: str = ""                   # the checker's own words when it did not
    bits: int = 0                   # bits on the wire inside `window`
    window: tuple[int, int] = (0, 0)   # the payload window, in core cycles
    nets: tuple[str, ...] = ()
    sm_cycles: int = 0              # SM cycles the engine ran inside `window`
    peak_tx: int = 0                # deepest the TX FIFO got inside `window` (of the active fifo_depth)
    peak_rx: int = 0                # deepest the RX FIFO got inside `window`
    enable_to_edge: Optional[int] = None    # CTRL enable -> first pin edge, in core cycles
    response: Optional[int] = None         # pin event -> response, in core cycles
    response_kind: str = ""               # "pin" (an output edge) or "fifo" (an RX FIFO push)
    response_note: str = ""                # which edge, or which FIFO, was the response
    #: 1, except for a bench that runs its protocol on a compressed time base: `fw/necir`'s
    #: bench calls one NEC protocol unit 562.5 ns where a real remote's is 562 500 ns, so the
    #: wire there is 1000x faster than reality and every rate has to be divided by this.  The
    #: core-cycle counts (SM cycles per bit, latencies, FIFO levels) are *not* affected: the
    #: chip does the same work per unit either way.
    compression: int = 1
    #: Was `sm_cycles` read at all?  False means the target does not expose the signal
    #: (`Probe.sm_available`), and then `sm_cycles_per_bit` is `n/a`, never a zero: publishing a
    #: number nobody read is the one thing this file exists to prevent.
    sm_available: bool = True
    #: Why the SM-cycle samples are or are not a reading, verbatim from the probe.
    sm_source: str = ""
    extra: dict = field(default_factory=dict)
    #: Per-key notes for `extra`, so a value that is a formula rather than a reading says so in
    #: the CSV's `source` column instead of arriving disguised as one.
    extra_notes: dict = field(default_factory=dict)

    # ---- the metrics, each a division of two measured things ---------------------------

    @property
    def ticks(self) -> int:
        """Length of the payload window, in core cycles (last edge minus first edge)."""
        return max(0, self.window[1] - self.window[0])

    @property
    def bit_rate_hz(self) -> float:
        """Bits on the wire divided by the core cycles the wire took, in real time.

        `hz()` is the 50 MHz core clock of `docs/mvp-regmap.md` `### Physical layer`; a bench
        with a compressed time base divides by its `compression` factor as well.
        """
        if self.ticks <= 0:
            return 0.0
        return hz(self.ticks / max(1, self.bits)) / self.compression

    @property
    def sm_cycles_per_bit(self) -> float:
        """Engine utilisation: SM cycles executed per bit on the wire."""
        return 0.0 if self.bits <= 0 else self.sm_cycles / self.bits

    @property
    def fifo_headroom(self) -> int:
        """Free FIFO words at the deepest point of the transaction, of the active `fifo_depth()`."""
        return fifo_depth() - max(self.peak_tx, self.peak_rx)

    def rows(self, target: str = "iss") -> list[dict]:
        """The CSV rows of this trial: one per metric, each with its unit and its source."""
        where = f"measured over core cycles {self.window[0]}..{self.window[1]} on nets " \
                f"{','.join(self.nets) or '-'}"
        out = [
            dict(metric="max_bit_rate", value=f"{self.bit_rate_hz:.6g}", unit="bit/s",
                 source=f"{self.bits} bits / {self.ticks} core cycles ({where}); "
                        f"{self.knob}={self.value:g} {self.unit}"
                        + (f"; bench time base compressed {self.compression}x"
                           if self.compression != 1 else "")),
            (dict(metric="sm_cycles_per_bit", value=f"{self.sm_cycles_per_bit:.6g}",
                  unit="SM cycles/bit",
                  source=f"{self.sm_cycles} SM cycles / {self.bits} bits ({where})"
                         + (f"; {self.sm_source}" if self.sm_source else ""))
             if self.sm_available else
             dict(metric="sm_cycles_per_bit", value="n/a", unit="SM cycles/bit",
                  source=f"not measured on this target, and not 0: {self.sm_source}")),
            dict(metric="fifo_headroom_at_max_rate", value=f"{self.fifo_headroom:d}",
                 unit=f"FIFO words free of {fifo_depth()}",
                 source=f"peak TX level {self.peak_tx}, peak RX level {self.peak_rx} "
                        f"({where}); FIFO_DEPTH={fifo_depth()}"),
        ]
        if self.enable_to_edge is None:
            out.append(dict(metric="regwrite_to_first_pin_edge", value="n/a",
                            unit="core cycles",
                            source="this firmware drives no protocol pin in this transaction"))
        else:
            out.append(dict(metric="regwrite_to_first_pin_edge",
                            value=f"{self.enable_to_edge:d}", unit="core cycles",
                            source=f"first edge on {self.nets[0] if self.nets else '-'} minus the "
                                   f"CTRL enable write's bus access cycle h"))
        if self.response is None:
            out.append(dict(metric="pin_event_to_response", value="n/a", unit="core cycles",
                            source="nothing on the wire causes this firmware to answer"
                                   if not self.extra.get("responds")
                                   else "no response edge could be attributed on this run"))
        else:
            out.append(dict(metric="pin_event_to_response", value=f"{self.response:d}",
                            unit="core cycles",
                            source=f"{self.response_kind}: {self.response_note}"))
        return [dict(protocol=self.protocol, knob=self.knob, knob_value=f"{self.value:g}",
                     target=target, **row) for row in out]

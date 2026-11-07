"""The 14 protocol benches, instrumented: one measured `Trial` per speed setting.

Every rig here is the bench its own firmware test uses -- the same `Bench`/`Rig`/`build()`
from `fw/<proto>/test_<proto>.py`, the same assembled `program.pio`, the same driver and the
same VIP checker -- with `verif.perf.probe.attach()` wrapped around the bus that bench already
built.  Nothing is re-implemented and nothing is softened: `ok` is the protocol VIP's own
verdict (`checker.violations == []`, plus the decoder agreeing with the payload), never this
file's opinion of the waveform.

The speed knob is each protocol's own -- `CLKDIV`, a bit period, a pulse width, a baud -- and
`speeds` lists the candidates fastest first, so `verif/perf/measure.py` can walk down the
ladder until the checker says no.  That boundary is what the reported maximum is: the fastest
setting whose transaction the protocol's own model accepts.

Seven protocols answer a pin event with a pin edge (spi, i2c, ps2, swd, jtag, onewire,
wiegand), four answer it with an RX FIFO push and drive no pin at all (uart_rx, quadrature,
necir, dmx512), and the other three transmit only, where nothing on the wire can make them
answer (uart_tx, manchester, ws2812).  `responds` says which, and the `pin_event_to_response`
row says "n/a" with the reason when there is none.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from verif.perf.probe import Probe, Trial, attach, sm_fields

#: FIFO depth: `verif.perf.probe.fifo_depth()` (active map, read at use time); the MVP value is 4.


@dataclass(frozen=True)
class Protocol:
    """One protocol's speed axis and the function that measures one setting on it."""

    name: str
    knob: str                                  # the knob the sweep moves
    unit: str                                  # its unit
    faster: str                                # "up" or "down": which way is faster
    speeds: tuple[float, ...]                  # candidates, fastest first
    trial: Callable[[float], Trial]
    responds: str = ""                         # "pin", "fifo" or "" (nothing to answer)
    targetable: tuple[str, ...] = ()           # targets this rig is also run on (FW_TARGET)
    note: str = ""


# ---------------------------------------------------------------------------------------------
# Shared measurement helpers
# ---------------------------------------------------------------------------------------------


def enable_latency(probe: Probe) -> Optional[int]:
    """Core cycles from the `CTRL` write that starts the transaction to the first pin edge.

    The enable write is the host's last setup access (`CTRL` with `SM_ENABLE` set), and the
    edge is the first change on any of the protocol's own nets at or after it.  Both are
    readings of the same run: `probe.accesses` for the write's bus access cycle `h`
    (docs/mvp-regmap.md `### Cycle-exact mapping to bus accesses`) and `probe.edges` for the
    edge.  None when the firmware drives no protocol pin at all.
    """
    enable = probe.first_ctrl_enable()
    if enable is None:
        return None
    edge = probe.first_edge(enable.t)
    return None if edge is None else edge.t - enable.t


def pin_response(probe: Probe, t_event: int, response_nets: tuple[str, ...]) -> Optional[int]:
    """Core cycles from the pin event at `t_event` to the chip's next edge on its own nets.

    The event is an edge on an *input* net (a target launched an answer, a device sent a
    frame); the response is the first edge the chip itself causes afterwards, which is the
    edge that acts on it.  One tick is the pad (`T_OUT` = 1) and three are the input pipeline
    (`T_IN` = 3), so a number of single digits here is the pipeline, not a stall.
    """
    for edge in probe.edges:
        if edge.t > t_event and edge.net in response_nets:
            return edge.t - t_event
    return None


def fifo_response(probe: Probe, t_event: int, engine: int = 0) -> Optional[int]:
    """Core cycles from the pin event at `t_event` to the first RX FIFO push after it.

    For a receiver that drives no pin, the answer to a wire event is the word it lands in the
    RX FIFO: the engine's `push` (or an autopush) is the cycle the host can read.  Measured on
    the per-cycle level samples, so it is the push and not the host's read.
    """
    for t in range(max(0, t_event) + 1, len(probe.rx_level)):
        if probe.rx_level[t] > probe.rx_level[t - 1]:
            return t - t_event
    return None


def rise_events(probe: Probe, net: str) -> list[int]:
    """The cycles in which `net` went high: the launches, captures and bit starts."""
    out, level = [], 0
    for edge in probe.edges:
        if edge.net != net:
            continue
        if edge.level == 1 and level == 0:
            out.append(edge.t)
        level = edge.level
    return out


def fall_events(probe: Probe, net: str) -> list[int]:
    """The cycles in which `net` went low."""
    out, level = [], 1
    for edge in probe.edges:
        if edge.net != net:
            continue
        if edge.level == 0 and level == 1:
            out.append(edge.t)
        level = edge.level
    return out


def span(probe: Probe, nets: tuple[str, ...], t0: int) -> tuple[int, int]:
    """(first, last) tick of the edges on `nets` at or after `t0`: the payload window."""
    on = [e for e in probe.edges if e.net in nets and e.t >= t0]
    return (on[0].t, on[-1].t) if on else (0, 0)


def levels(probe: Probe, t0: int, t1: int) -> tuple[int, int]:
    return probe.peak_levels(t0, t1)


def frame_starts(edges: Sequence[int], period: float, slack: float = 0.9) -> list[int]:
    """The subset of `edges` that are one whole period apart: one per payload frame.

    A protocol's frames are separated by edges of their own -- the data bits of the frame
    also make edges -- so "the first edge" is not "the first frame".  With the frame period
    measured elsewhere in the same run (from the frames the VIP decoded), the frame starts
    are the edges that are at least `slack * period` after the one before.
    """
    out: list[int] = []
    for t in edges:
        if not out or t - out[-1] >= slack * period:
            out.append(t)
    return out


def _safe(what, fallback):
    """`what()` if it answers, `fallback` if it refuses the setting (a ladder rung, not a run)."""
    try:
        return what()
    except Exception:                                     # a driver guard, not a simulation
        return fallback


def _measured(what, fallback):
    """`what()`, or the fallback *labelled as a fallback*: `(value, "why this is not a reading")`.

    A fallback is arithmetic the bench did itself, so it must not reach the CSV in the shape of a
    measurement.  This returns the pair `Trial.extra_notes` wants, and the note ends up in the
    row's `source` column, so a reader sees "fallback" next to the number rather than having to
    guess whether the driver or the bench produced it.
    """
    try:
        return what(), ""
    except Exception as exc:                              # a driver guard, not a simulation
        return fallback, (f"FALLBACK, not a measurement: the driver refused "
                          f"({type(exc).__name__}: {exc}), so this is the formula "
                          f"{fallback:.6g}")


def median(values: Sequence[float]) -> float:
    """The middle value: a period measured off the wire, not a requested one."""
    ordered = sorted(values)
    n = len(ordered)
    if not n:
        return 0.0
    return float(ordered[n // 2] if n % 2 else (ordered[n // 2 - 1] + ordered[n // 2]) / 2)

# ---------------------------------------------------------------------------------------------
# spi: full duplex, and the only protocol on this chip that drives a wire it also reads
# ---------------------------------------------------------------------------------------------


def spi_trial(clkdiv_int: float, program=None) -> Trial:
    """`fw/spi` at `CLKDIV = (clkdiv_int, 0)`: a 20-byte flash read, judged by the flash and the
    SPI monitor.

    The knob is the divider integer, and faster is *smaller* (the divider's floor is 1.0, one
    SM cycle per core cycle).  `fw/spi/test_spi.py` calls 8 core cycles a bit at CLKDIV 1.0
    the maximum and quotes 6.25 MBd; this measures it off the edge log instead of taking it.

    `program` defaults to the one `program.pio` assembles and exists for the negative control:
    `verif/perf/selftest.py` loads a variant one cycle per bit faster, which the monitor has
    to reject.
    """
    from fw.spi.test_spi import Rig, read_word

    clkdiv = int(clkdiv_int)
    rig = Rig(mode=0, clkdiv=(clkdiv, 0), program=program)
    probe = attach(rig.bus, rig.dev, rig.dev.chip, nets=("p0", "p1", "p2", "p3"))
    data = [(0x3C + i) & 0xFF for i in range(16)]
    rig.flash.poke(0x40, data)
    rx = rig.run(read_word(0x40, 16))
    xfer = rig.monitor.transfers[0] if rig.monitor.transfers else None

    ok = bool(xfer is not None and rx[4:20] == data and not rig.monitor.violations
              and not rig.bus.errors)
    why = "" if ok else (
        f"rx={bytes(rx[4:20]).hex()} want={bytes(data).hex()}; "
        f"violations={rig.monitor.violations[:2]}; errors={len(rig.bus.errors)}")

    # The payload window: the first SCK edge inside the CS window to the last one.  The SCK
    # edge before CS falls is the direction change of the forced `set pindirs`, not a bit.
    cs = probe.edges_on(["p0"])
    fall = next((e.t for e in cs if e.level == 0), 0)
    rise = next((e.t for e in cs if e.level == 1 and e.t > fall), fall)
    t0, t1 = span(probe, ("p1",), fall)
    bits = 8 * len(xfer.mosi) if xfer else 0
    tx, rx_peak = levels(probe, t0, t1)

    # The response: the flash launches its first answer bit on MISO; the chip's next edge is
    # the clock edge that samples it.
    miso = [e.t for e in probe.edges_on(["p3"]) if fall < e.t < rise]
    response = pin_response(probe, miso[0], ("p0", "p1", "p2")) if miso else None
    note = (f"flash MISO edge in cycle {miso[0]} to the chip's next CS/SCK/MOSI edge"
            if miso else "")

    return Trial(
        protocol="spi", knob="clkdiv_int", value=float(clkdiv), unit="core cycles per SM cycle",
        ok=ok, why=why, bits=bits, window=(t0, t1), nets=("p0", "p1", "p2", "p3"),
        **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=enable_latency(probe), response=response, response_kind="pin",
        response_note=note, extra={"cs_window": (fall, rise), "program": "variant"
                                   if program is not None else "spi_read_m0"},
    )


# ---------------------------------------------------------------------------------------------
# uart_tx: transmit only, judged by a UartRx model on the wire and by a second one at the
# nominal rate (which is what catches a divider that lands on the wrong baud)
# ---------------------------------------------------------------------------------------------


def uart_tx_trial(baud: float) -> Trial:
    """`fw/uart_tx` at a requested baud: 8 bytes out, read back by two receivers.

    `build()` and the transaction are `fw/uart_tx/test_uart_tx.py`'s; only the probe is added,
    between building the bus and loading the program, so the enable write is inside the
    access log.  Two receivers judge it: one at the rate the divider really produced (so a
    wrong baud shows up as framing errors or wrong bytes) and one at the nominal rate (so a
    divider that cannot reach the requested baud is caught by the timing, not by luck).
    """
    from fw.uart_tx.driver import ticks_per_bit
    from fw.uart_tx.test_uart_tx import Case, build, payload

    case = Case(f"perf-{baud:g}", baud, count=8)
    bus, dev, drv, rx, rx_nominal = build(case)
    probe = attach(bus, dev, dev.chip, nets=(f"p{case.pin}",))
    data = payload(case.count)
    drv.load()
    drv.start(first=data[: drv.TXFIFO_DEPTH])
    drv.send(data[drv.TXFIFO_DEPTH :])
    tpb = drv.config.nominal_ticks_per_bit
    bus.run(int(len(data) * (drv.config.frame_bits + 2) * tpb) + 8000)

    net = f"p{case.pin}"
    # The verdict is the one `fw/uart_tx/test_uart_tx.py` uses, with its own constants: the
    # bytes arrive in order, framed 8N1, on the divider's own bit grid and inside one tick
    # of the requested one.  TXSTALL is not a fault here (the engine waits for the host).
    from fw.uart_tx.test_uart_tx import FDEBUG_BAD, FDEBUG_TXSTALL

    achieved = ticks_per_bit(drv.clkdiv, drv.sm_cycles_per_bit)
    ok = bool(rx.values == data and len(rx.received) == len(data)
              and rx.framing_errors == 0 and rx.parity_errors == 0 and rx.aborted == 0
              and rx.worst_dev_ticks < 1.0
              and rx_nominal.worst_dev_ticks < 1.0 + 10 * abs(achieved - tpb)
              and not bus.errors
              and drv.fdebug() & ~(FDEBUG_TXSTALL << case.engine) == 0)
    why = "" if ok else (
        f"rx={rx.values} want={data}; framing {rx.framing_errors}/{rx_nominal.framing_errors} "
        f"parity {rx.parity_errors} aborted {rx.aborted}; grid deviation "
        f"{rx.worst_dev_ticks:.3f}/{rx_nominal.worst_dev_ticks:.3f} ticks; "
        f"FDEBUG={drv.fdebug():#010x}; bus errors {len(bus.errors)}")

    # The payload window cannot be "first edge to last edge": the last data bits of a frame
    # are often the same level as the stop bit, so the wire simply stops moving while the
    # frame finishes.  The end is therefore the *last frame's start* plus one measured frame
    # period -- the median gap between consecutive frame starts, which is on the wire too.
    starts = [b.tick for b in rx.received]
    periods = [b - a for a, b in zip(starts, starts[1:])]
    frame_ticks = int(median(periods)) if periods else 0
    net_edges = probe.edges_on([net])
    bits = drv.config.frame_bits * len(rx.received)
    t0 = net_edges[0].t if net_edges else 0
    t1 = (starts[-1] + frame_ticks) if starts else t0
    tx, rx_peak = levels(probe, t0, t1)
    return Trial(
        protocol="uart_tx", knob="baud", value=float(baud), unit="bit/s", ok=ok, why=why,
        bits=bits, window=(t0, t1), nets=(net,), **sm_fields(probe, t0, t1),
        peak_tx=tx, peak_rx=rx_peak, enable_to_edge=enable_latency(probe),
        response=None, response_kind="", response_note="",
        extra={"achieved_baud": 50e6 / achieved, "requested_baud": baud,
               "frame_period_ticks": frame_ticks},
    )


# ---------------------------------------------------------------------------------------------
# uart_rx: receive only, eight back-to-back frames from a UartTx model into the chip
# ---------------------------------------------------------------------------------------------


def uart_rx_trial(baud: float) -> Trial:
    """`fw/uart_rx` at a requested baud: eight back-to-back 8N1 frames, read out of the FIFO.

    The knob is the baud, and the program is rebuilt for it (`UartRx` assembles
    `sm_cycles_per_bit = 50 MHz / baud` into the `.pio`), so this measures the shipped
    firmware at that rate rather than a hand-set divider.  Nothing above `50e6 / S` baud can
    work, so the ladder's fastest rung is `S = 6`, the smallest `S` the program keeps for
    back-to-back frames (`fw/uart_rx/driver.py`, `UartRx`).
    """
    from fw.uart_rx.driver import UartRx
    from fw.uart_rx.test_uart_rx import transmitter

    # The program samples the middle of the start bit and of every data bit, so S has to be
    # even and between 4 and 32 (`build_program`); a requested baud that implies anything
    # else is rounded to the S that realises it, and the trial says which.
    value_of = baud
    s_per_bit = int(round(50e6 / baud))
    s_per_bit = min(32, max(4, s_per_bit))
    s_per_bit += s_per_bit % 2
    baud = 50e6 / s_per_bit
    rx = UartRx(baud=baud, sm_cycles_per_bit=s_per_bit)
    probe = attach(rx.bus, rx.dev, rx.chip, nets=(rx.net,))
    payload = bytes([0x00, 0x55, 0xAA, 0xFF, 0x01, 0x80, 0x7F, 0xFE])
    tx = transmitter(rx)
    tx.send(payload)
    limit = int(12.0 * len(payload) * rx.ticks_per_bit) + 256
    for _ in range(limit):
        if len(rx.received) >= len(payload):
            break
        rx.step()

    ok = (rx.values == list(payload) and rx.framing_errors == 0
          and rx.chip.fdebug == 0 and not rx.bus.errors)
    why = "" if ok else (
        f"rx={rx.values} want={list(payload)}; framing {rx.framing_errors}; "
        f"FDEBUG={rx.chip.fdebug:#010x}; bus errors {len(rx.bus.errors)}")

    # One FIFO push per frame, so the gap between pushes is one frame period, measured; the
    # frame *starts* are then the falling edges one period apart (every frame starts with a
    # start bit, and the data bits inside a frame make edges of their own).
    pushes = [t for t in range(1, len(probe.rx_level))
              if probe.rx_level[t] > probe.rx_level[t - 1]]
    period = median([b - a for a, b in zip(pushes, pushes[1:])]) if len(pushes) > 1 else 0.0
    falls = fall_events(probe, rx.net)
    starts = frame_starts(falls, period) if period else falls[:1]
    bits = 10 * len(rx.received)
    t0 = starts[0] if starts else 0
    t1 = int(starts[-1] + period) if starts and period else t0
    tx, rx_peak = levels(probe, t0, t1)
    response = fifo_response(probe, t0) if t0 else None
    return Trial(
        protocol="uart_rx", knob="baud", value=float(baud), unit="bit/s", ok=ok, why=why,
        bits=bits, window=(t0, t1), nets=(rx.net,), **sm_fields(probe, t0, t1),
        peak_tx=tx, peak_rx=rx_peak, enable_to_edge=None, response=response,
        response_kind="fifo",
        response_note=f"RXD start-bit edge in cycle {t0} to the first RX FIFO push"
                      if response else "",
        extra={"sm_cycles_per_bit": s_per_bit, "frame_period_ticks": int(period),
               "requested_baud": float(value_of)},
    )


# ---------------------------------------------------------------------------------------------
# i2c: open drain, clock stretching, and a target that answers nine bits per byte
# ---------------------------------------------------------------------------------------------


def i2c_trial(clkdiv: int) -> Trial:
    """`fw/i2c` at `CLKDIV = clkdiv`: a write, a repeated START and a four-byte read.

    The knob is the divider integer and faster is smaller.  `I2cTimingChecker` is the judge
    (it checks tBUF, tHD;DAT, tLOW, tHIGH, the data setup and hold and fSCL), and the EEPROM
    only ACKs its own address, so a divider that outruns the slave comes back as an `I2cError`
    from the driver or as a checker violation -- both are recorded, and neither is `ok`.
    """
    from fw.i2c import I2cError
    from fw.i2c.test_i2c import Bench

    expect = bytes([0x22, 0x33, 0x44, 0x00])     # 0x00 was just written at 0x10
    bench = Bench(mode="fast", clkdiv=int(clkdiv), stretch_ticks=0, address=0x50)
    bench.eeprom.memory[0x10:0x14] = bytes([0x11, 0x22, 0x33, 0x44])
    probe = attach(bench.bus, bench.device, bench.chip,
                   nets=(f"p{bench.sda_pin}", f"p{bench.scl_pin}"))
    bench.start()
    read = None
    try:
        _acked, read = bench.driver.write_read(0x50, [0x10, 0x00], 4)
    except I2cError as exc:
        return Trial(protocol="i2c", knob="clkdiv_int", value=float(clkdiv),
                     unit="core cycles per SM cycle", ok=False,
                     why=f"the driver gave up on this divider: {exc}")
    rules = sorted({v.rule for v in bench.checker.violations})
    ok = bool(read == expect and not bench.checker.violations
              and bench.driver.fdebug() == 0 and not bench.bus.errors)
    why = "" if ok else (
        f"read={bytes(read).hex()} want={expect.hex()}; checker rules={rules}; "
        f"FDEBUG={bench.driver.fdebug():#010x}; bus errors={len(bench.bus.errors)}")

    # The window is the SCL clocking: the first rising edge after the enable to the last one,
    # plus the ninth clock of the last byte, which the checker samples at the ACK.
    scl = f"p{bench.scl_pin}"
    sda = f"p{bench.sda_pin}"
    enable = probe.first_ctrl_enable()
    rises = [t for t in rise_events(probe, scl) if enable is None or t > enable.t]
    nine = 9 * (1.0 + clkdiv)             # nine SCL periods, the divider being integer here
    bits = 9 * len(bench.checker.transfers)
    t0 = rises[0] if rises else 0
    t1 = int(rises[-1] + nine) if rises else 0
    tx, rx_peak = levels(probe, t0, t1)
    # The response: the slave pulls SDA low to ACK; the chip's next SCL rise is its answer.
    slave = [t for t in fall_events(probe, sda) if enable is None or t > enable.t]
    response = pin_response(probe, slave[0], (scl,)) if slave else None
    return Trial(
        protocol="i2c", knob="clkdiv_int", value=float(clkdiv),
        unit="core cycles per SM cycle", ok=ok, why=why, bits=bits, window=(t0, t1),
        nets=(scl, sda), **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=enable_latency(probe), response=response, response_kind="pin",
        response_note=f"the EEPROM's ACK (SDA fall) in cycle {slave[0]} to the chip's next SCL "
                      f"rise" if slave else "",
        extra={"checker_rules": rules, "bytes_on_the_wire": len(bench.checker.transfers),
               "shortest_scl_period": bench.checker.shortest_period,
               "max_scl_hz": bench.checker.max_scl_hz},
    )


# ---------------------------------------------------------------------------------------------
# jtag: a controller, a TAP, and two scans (the IR scan and the 32-bit IDCODE DR scan)
# ---------------------------------------------------------------------------------------------


def jtag_trial(clkdiv: int) -> Trial:
    """`fw/jtag` at `CLKDIV = clkdiv`: the reset, the IR scan and the IDCODE DR scan.

    `JtagChecker` is the judge and `JtagTarget` only answers IDCODE when the instruction in
    force really is IDCODE, so the 32 bits that come back are the target's, not a pattern the
    two ends agree on.  The window is the two scans' own extent (`ObservedScan.t_start` and
    `t_end`, both measured off the wire) and the bits are the scans' own lengths.
    """
    from fw.jtag import driver as drv
    from fw.jtag.test_jtag import Bench
    from verif.vip import jtag as vip

    bench = Bench(clkdiv=int(clkdiv))
    probe = attach(bench.bus, bench.device, bench.chip,
                   nets=tuple(f"p{p}" for p in bench.pins))
    bench.start()
    got = bench.driver.idcode()
    rules = sorted({v.rule for v in bench.checker.violations})
    scans = list(bench.checker.scans)
    ok = bool(got == vip.DEFAULT_IDCODE and not bench.checker.violations
              and not bench.bus.errors and bench.driver.fdebug() == 0)
    why = "" if ok else (
        f"idcode={got:#010x} want {vip.DEFAULT_IDCODE:#010x}; rules={rules}; "
        f"FDEBUG={bench.driver.fdebug():#010x}; bus errors={len(bench.bus.errors)}")

    bits = sum(s.length for s in scans)
    t0 = min((s.t_start for s in scans), default=0)
    t1 = max((s.t_end for s in scans), default=t0)
    tx, rx_peak = levels(probe, t0, t1)
    # The response: the TAP puts the next TDO bit on the net after the falling edge that
    # ended the previous TCK; the chip's answer is its next TCK or TMS edge.
    tdo = f"p{bench.pins[3]}"
    events = [t for t in fall_events(probe, tdo) if t > t0]
    response = pin_response(probe, events[0], tuple(f"p{p}" for p in bench.pins[:3])) \
        if events else None
    return Trial(
        protocol="jtag", knob="clkdiv_int", value=float(clkdiv),
        unit="core cycles per SM cycle", ok=ok, why=why, bits=bits, window=(t0, t1),
        nets=tuple(f"p{p}" for p in bench.pins), **sm_fields(probe, t0, t1),
        peak_tx=tx, peak_rx=rx_peak, enable_to_edge=enable_latency(probe),
        response=response, response_kind="pin",
        response_note=f"the TAP's TDO fall in cycle {events[0]} to the chip's next TCK/TMS/TDI "
                      f"edge" if events else "",
        extra={"checker_rules": rules, "steady_tck_period": bench.steady_period(),
               "scans": [(s.kind, s.length) for s in scans]},
    )


# ---------------------------------------------------------------------------------------------
# ps2: the host is a device's master here, so the keyboard sends and the chip clocks it out
# ---------------------------------------------------------------------------------------------


def ps2_trial(clkdiv: int) -> Trial:
    """`fw/ps2` at `CLKDIV = clkdiv`: a reset, its answer, and a key press read off the bus.

    The device is put on the *slow* rate so the periods `Ps2Checker` measures are the host's
    and not the device's (`fw/ps2/test_ps2.py` does the same and says why).  A bit is one
    clock: the bits are counted as rising edges on the CLK net and the window runs from the
    first to the last of them plus one measured clock period, so nothing has to be assumed
    about which frames the sequence contains.
    """
    from fw.ps2.test_ps2 import RESET, Bench

    # Below the floor `Ps2Driver` refuses the setting, so the ladder's fast rungs are loaded
    # anyway -- a deliberate fault, so that the ladder finds a *checker* verdict and not only
    # the driver's own guard.  `extra["driver_refused"]` says which settings those were.
    refused = ""
    try:
        bench = Bench(dev_rate="slow", clkdiv=int(clkdiv))
    except ValueError as exc:
        refused = str(exc)
        bench = Bench(dev_rate="slow")
        bench.driver.clkdiv = int(clkdiv)
    clk, data = f"p{bench.clk_pin}", f"p{bench.data_pin}"
    probe = attach(bench.bus, bench.device, bench.chip, nets=(clk, data))
    bench.start()
    acked = bench.driver.command(RESET, response=1)
    scan = bench.dev.make_code("a")
    bench.dev.press("a")
    got = bench.driver.receive(1)
    rules = sorted({v.rule for v in bench.checker.violations})
    ok = bool(acked == bytes([0xAA]) and got == bytes([scan]) and not bench.checker.violations
              and not bench.bus.errors and bench.driver.fdebug() == 0)
    why = "" if ok else (
        f"reset answer={bytes(acked).hex()} want aa, scan={bytes(got).hex()} want "
        f"{bytes([scan]).hex()}; "
        f"rules={rules}; FDEBUG={bench.driver.fdebug():#010x}; bus errors={len(bench.bus.errors)}")

    rises = rise_events(probe, clk)
    period = median([b - a for a, b in zip(rises, rises[1:])])
    bits = len(rises)
    t0 = rises[0] if rises else 0
    t1 = int(rises[-1] + period) if rises else 0
    tx, rx_peak = levels(probe, t0, t1)
    # The response: the keyboard puts a frame on DATA; the chip's next clock edge clocks it out.
    events = [t for t in fall_events(probe, data) if t >= t0]
    response = pin_response(probe, events[0], (clk,)) if events else None
    return Trial(
        protocol="ps2", knob="clkdiv_int", value=float(clkdiv),
        unit="core cycles per SM cycle", ok=ok, why=why, bits=bits, window=(t0, t1),
        nets=(clk, data), **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=enable_latency(probe), response=response, response_kind="pin",
        response_note=f"the keyboard's DATA fall in cycle {events[0]} to the chip's next CLK "
                      f"edge" if events else "",
        extra={"checker_rules": rules, "clock_period_ticks": int(period),
               "frames_to_device": bench.checker.frames_to_device,
               "frames_to_host": bench.checker.frames_to_host,
               "line_rate_hz": bench.driver.line_rate_hz, "driver_refused": refused},
    )


# ---------------------------------------------------------------------------------------------
# swd: 33 transfers of a debug session, judged by a model that decodes ADIv5 off the pins
# ---------------------------------------------------------------------------------------------


def swd_trial(period: int) -> Trial:
    """`fw/swd` at a bit period of `period` core cycles: a line reset and a short session.

    `SwdDriver` retimes the program's two delay fields, so the period is the program's own
    knob (`high = MIN_HIGH`, `low = period - LOOPS_PER_PERIOD - MIN_HIGH`); its floor is
    `LOOPS_PER_PERIOD + MIN_HIGH` = 9 ticks, which is what the ladder's fastest rung is.  A
    bit is one SWCLK period, so the bits are the rising edges on the SWCLK net.
    """
    from fw.swd import driver as drv
    from fw.swd.test_swd import Bench
    from verif.vip import swd as vswd

    high = drv.MIN_HIGH
    low = int(period) - drv.LOOPS_PER_PERIOD - high
    if low < 0:
        return Trial(protocol="swd", knob="bit_period", value=float(period),
                     unit="core cycles per bit", ok=False,
                     why=f"the program's delays cannot express {period} ticks per bit "
                         f"(MIN_HIGH={drv.MIN_HIGH}, LOOPS_PER_PERIOD={drv.LOOPS_PER_PERIOD})")
    try:
        bench = Bench(high=high, low=low)
    except ValueError as exc:
        return Trial(protocol="swd", knob="bit_period", value=float(period),
                     unit="core cycles per bit", ok=False, why=f"the driver refused: {exc}")
    clk, io = f"p{bench.swclk}", f"p{bench.swdio}"
    probe = attach(bench.bus, bench.device, bench.chip, nets=(clk, io, f"p{bench.flag}"))
    bench.start()
    bench.driver.line_reset()
    idcode = bench.driver.read(vswd.DP_IDCODE)
    bench.driver.write(vswd.DP_SELECT, 0x1234_5678)
    back = bench.driver.read(vswd.DP_SELECT)
    rules = sorted({v.rule for v in bench.checker.violations})
    ok = bool(idcode == vswd.IDCODE_DEFAULT and back == 0x1234_5678
              and not bench.checker.violations
              and not bench.bus.errors and bench.driver.fdebug() == 0)
    why = "" if ok else (
        f"idcode={idcode:#010x} select={back:#010x}; rules={rules}; "
        f"FDEBUG={bench.driver.fdebug():#010x}; bus errors={len(bench.bus.errors)}")

    rises = rise_events(probe, clk)
    gaps = median([b - a for a, b in zip(rises, rises[1:])])
    bits = len(rises)
    t0 = rises[0] if rises else 0
    t1 = int(rises[-1] + gaps) if rises else 0
    tx, rx_peak = levels(probe, t0, t1)
    # The response: the DP answers on SWDIO; the chip's next SWCLK edge is what samples it.
    io_edges = [t for t in fall_events(probe, io) if t >= t0]
    response = pin_response(probe, io_edges[0], (clk,)) if io_edges else None
    swclk_hz, swclk_note = _measured(lambda: bench.driver.swclk_hz(int(period)), 50e6 / period)
    return Trial(
        protocol="swd", knob="bit_period", value=float(period), unit="core cycles per bit",
        ok=ok, why=why, bits=bits, window=(t0, t1), nets=(clk, io, f"p{bench.flag}"),
        **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=enable_latency(probe), response=response, response_kind="pin",
        response_note=f"the DP's SWDIO fall in cycle {io_edges[0]} to the chip's next SWCLK "
                      f"edge" if io_edges else "",
        extra={"checker_rules": rules, "high": high, "low": low, "frames": len(bench.checker.frames),
               "swclk_hz": swclk_hz},
        extra_notes={"swclk_hz": swclk_note} if swclk_note else {},
    )


# ---------------------------------------------------------------------------------------------
# onewire: open drain, one bit per slot, and a slave that answers inside the slot
# ---------------------------------------------------------------------------------------------


def onewire_trial(clkdiv: int) -> Trial:
    """`fw/onewire` at `CLKDIV = clkdiv`: reset, Skip ROM, Read Scratchpad, nine bytes back.

    A bit is one time slot, so the bits are the checker's own count of write and read slots
    and the window runs from the first slot's pull-low to the last one plus one measured slot
    period.  `OneWireTimingChecker` judges the slots against the DS18B20 tables at the chosen
    speed, and `PinWatch` catches a chip that ever drove the open-drain line high, which is
    the fault the wired-AND of the bus would hide.
    """
    from fw.onewire.test_onewire import SERIAL, Bench

    bench = Bench(clkdiv=int(clkdiv), dq_pin=0, serials=(SERIAL,), speed="standard")
    dq = f"p{bench.dq_pin}"
    probe = attach(bench.bus, bench.device, bench.chip, nets=(dq,))
    bench.start()
    bench.targets[0].set_temperature(0x0191)
    got = bench.driver.read_scratchpad()
    violations = bench.close()
    rules = sorted({v.rule for v in bench.checker.violations})
    want = bytes(bench.targets[0].scratchpad)
    ok = bool(got == want and not violations and bench.watch.driven_high == []
              and not bench.bus.errors)
    why = "" if ok else (
        f"scratchpad={got.hex()} want {want.hex()}; rules={rules}; "
        f"drove the line high on {len(bench.watch.driven_high)} ticks; "
        f"bus errors={len(bench.bus.errors)}")

    slots = bench.checker.write_slots + bench.checker.read_slots
    falls = fall_events(probe, dq)
    periods = [b - a for a, b in zip(falls, falls[1:])]
    period = median(periods[1:]) if len(periods) > 1 else 0.0     # [0] spans the reset pulse
    t0 = falls[1] if len(falls) > 1 else (falls[0] if falls else 0)
    t1 = int(falls[-1] + period) if falls else 0
    tx, rx_peak = levels(probe, t0, t1)
    # The response: the slave pulls DQ low inside a slot (its presence pulse, and every 0 bit
    # it sends); the chip's next edge on DQ is its answer.
    slave = [t for t in falls if t > t0]
    response = pin_response(probe, slave[0], (dq,)) if slave else None
    return Trial(
        protocol="onewire", knob="clkdiv_int", value=float(clkdiv),
        unit="core cycles per SM cycle", ok=ok, why=why, bits=slots, window=(t0, t1),
        nets=(dq,), **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=enable_latency(probe), response=response, response_kind="pin",
        response_note=f"the slave's DQ pull-low in cycle {slave[0]} to the chip's next DQ edge"
                      if slave else "",
        extra={"checker_rules": rules, "slot_period_ticks": int(period),
               "write_slots": bench.checker.write_slots, "read_slots": bench.checker.read_slots,
               "presence_pulses": bench.checker.presences},
    )


# ---------------------------------------------------------------------------------------------
# manchester: transmit only, line code with a mid-bit transition in every bit
# ---------------------------------------------------------------------------------------------


def manchester_trial(bit_rate: float) -> Trial:
    """`fw/manchester` at a requested bit rate: six one-byte messages with a sync preamble.

    The knob is the requested bit rate, which the driver turns into a 16.8 divider.  The
    shipped `program.pio` has two programs and the faster one is `manchester_even` (4 SM cycles
    per bit, 12.5 Mbit/s at CLKDIV 1.0), so anything above 10 Mbit/s is run on it; the
    `manchester` program is 5 SM cycles per bit and stops at 10 Mbit/s.  Both judges are
    configured at the *requested* rate, so a divider that lands too far off it is caught by
    `ManchesterTimingChecker` rather than by luck.  Every bit has a transition in the middle of
    it (that is the line code), so the window runs from the first edge to the last edge plus
    half a measured bit.
    """
    from fw.manchester.test_manchester import PREAMBLE_BITS, Case, build, payload

    name = "manchester_even" if bit_rate > 10e6 else "manchester"
    case = Case(f"perf-{bit_rate:g}", name, bit_rate, count=6)
    try:
        bench_bus, dev, drv, rx, chk = build(case)
    except ValueError as exc:
        return Trial(protocol="manchester", knob="bit_rate", value=float(bit_rate),
                     unit="bit/s", ok=False,
                     why=f"the driver or the judge refused this rate: {exc}")
    probe = attach(bench_bus, dev, dev.chip, nets=(f"p{case.pin}",))
    data = payload(case.count, case.engine)
    try:
        drv.load()
        drv.start(first=data[: drv.TXFIFO_DEPTH])
        drv.send(data[drv.TXFIFO_DEPTH :])
        bench_bus.run(drv.message_ticks(len(data)) + 4000)
    except ValueError as exc:
        return Trial(protocol="manchester", knob="bit_rate", value=float(bit_rate),
                     unit="bit/s", ok=False,
                     why=f"the driver or the judge refused this rate: {exc}")

    edges = probe.edges_on([f"p{case.pin}"])
    bits = (8 + PREAMBLE_BITS) * len(data)
    times = [e.t for e in edges]
    half = median([b - a for a, b in zip(times, times[1:])]) / 2.0
    t0 = times[0] if times else 0
    t1 = int(times[-1] + half) if times else 0
    tx, rx_peak = levels(probe, t0, t1)
    rules = sorted({v.rule for v in chk.violations})
    decoded = rx.decoded_bytes()
    # TXSTALL is not a fault for a transmitter between messages (`test_the_chip_is_left_clean`).
    from fw.manchester.test_manchester import FDEBUG_TXSTALL

    ok = bool(chk.clean and rx.clean and decoded == bytes(data) and not bench_bus.errors
              and drv.fdebug() & ~(FDEBUG_TXSTALL << case.engine) == 0)
    why = "" if ok else (
        f"decoded {decoded.hex()} want {bytes(data).hex()}; rules={rules}; "
        f"rx clean={rx.clean}; FDEBUG={drv.fdebug():#010x}; bus errors={len(bench_bus.errors)}")
    return Trial(
        protocol="manchester", knob="bit_rate", value=float(bit_rate), unit="bit/s", ok=ok,
        why=why, bits=bits, window=(t0, t1), nets=(f"p{case.pin}",),
        **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=enable_latency(probe), response=None, response_kind="",
        response_note="",
        extra={"half_bit_ticks": half, "clkdiv": drv.clkdiv, "program": name,
               "preamble_bits": PREAMBLE_BITS, "checker_rules": rules,
               "streams": len(chk.streams)},
    )


# ---------------------------------------------------------------------------------------------
# ws2812: transmit only, and the bit period is the program's own (two programs, no divider knob)
# ---------------------------------------------------------------------------------------------


def ws2812_trial(speed: float) -> Trial:
    """`fw/ws2812` at one of its two programs: `speed` is 1 for `ws2812_fast`, 0 for `ws2812`.

    The knob is which program is loaded: the two differ in how many SM cycles a bit takes, so
    "the requested rate" is a program choice rather than a divider setting, and this measures
    what each one puts on the wire.  One pixel, one frame, so the window is that frame's bits
    (`Ws2812TimingChecker.bits` timestamps every one of them) and the bits are their count.
    """
    from fw.ws2812.driver import grb_word
    from fw.ws2812.test_ws2812 import Case, FDEBUG_TXSTALL, PALETTE, build, run

    program = "ws2812_fast" if speed else "ws2812"
    case = Case(f"perf-{program}", program=program, pixels=1, pin=0, frames=1)
    bench = build(case)
    probe = attach(bench.bus, bench.dev, bench.dev.chip, nets=(bench.net,))
    run(bench, frames=[[PALETTE[3]]])   # the test's own run(): load, start, send, wait for the latch
    rules = sorted({v.rule for v in bench.chk.violations})
    bits = list(bench.chk.bits)
    frame = bench.chk.frames[-1] if bench.chk.frames else None
    # The judge: the frame on the wire carries the datasheet words the host pushed, and the
    # strip itself latched that colour (one pixel here, so the chain cannot reorder).
    from asm.pioasm import Program as _Program          # noqa: F401  (import kept local)

    latched = [p.color for p in bench.chain]
    ok = bool(bits and frame is not None and frame.bits == 24
              and frame.words == (grb_word(PALETTE[3]),) and latched == [PALETTE[3]]
              and not rules and not bench.bus.errors
              and bench.drv.fdebug() & ~(FDEBUG_TXSTALL << case.engine) == 0)
    why = "" if ok else (
        f"{len(bits)} bits, frame.bits={frame.bits if frame else None}, "
        f"words={[hex(w) for w in frame.words] if frame else None}; latched={latched} want "
        f"{[PALETTE[3]]}; rules={rules}; FDEBUG={bench.drv.fdebug():#010x}; "
        f"bus errors={len(bench.bus.errors)}")

    # A bit is a pulse: the checker's own bit events carry the period, so the window is the
    # first bit's high start to the last bit's low end.
    period = median([b.high_ticks + b.low_ticks for b in bits]) if bits else 0.0
    t0 = bits[0].tick if bits else 0
    t1 = int(bits[-1].tick + period) if bits else 0
    tx, rx_peak = levels(probe, t0, t1)
    return Trial(
        protocol="ws2812", knob="program", value=float(speed),
        unit="0 = ws2812, 1 = ws2812_fast", ok=ok, why=why, bits=len(bits),
        window=(t0, t1), nets=(bench.net,), **sm_fields(probe, t0, t1),
        peak_tx=tx, peak_rx=rx_peak, enable_to_edge=enable_latency(probe),
        response=None, response_kind="", response_note="",
        extra={"program": program, "bit_period_ticks": period,
               "sm_cycles_bounds": bench.drv.bit_cycles, "checker_rules": rules,
               "bit_rate_hz": 50e6 / period if period else 0.0},
    )


# ---------------------------------------------------------------------------------------------
# dmx512: transmit only, and a frame is only closed by the next BREAK
# ---------------------------------------------------------------------------------------------


def dmx512_trial(bit_rate: float) -> Trial:
    """`fw/dmx512` at a requested bit rate: two 25-slot frames on a bus with a receiver on it.

    The knob is the requested bit rate; `Dmx512Driver` picks the bit delay and the divider that
    the tables' window allows (`bit_delay_for_rate`), and `DmxChecker` judges the wire it
    actually produced.  A frame is a slot grid: 32 bits per slot, and the window is the first
    slot's edge to the last one's, measured from the checker's own packet timings.
    """
    from fw.dmx512.test_dmx512 import Case, FDEBUG_TXSTALL, build, payload
    from verif.vip.dmx512 import NULL_START_CODE

    case = Case(f"perf-{bit_rate:g}", bit_rate, slots=25, packets=2)
    try:
        bench = build(case)
    except ValueError as exc:
        return Trial(protocol="dmx512", knob="bit_rate", value=float(bit_rate), unit="bit/s",
                     ok=False, why=f"the driver refused this rate: {exc}")
    net = f"p{case.pin}"
    probe = attach(bench.bus, bench.dev, bench.dev.chip, nets=(net,))
    bench.values = payload(bench.drv.slots)
    bench.drv.send(bench.values, case.packets)
    bench.chk.finish()          # a frame is only closed by the next BREAK: see run_case()
    bench.rx.finish()
    rules = sorted(bench.rules)
    packets = list(bench.chk.packets)
    received = list(bench.rx.packets)
    expect = bench.values[: bench.drv.slots]
    # Two independent judgements, as the test uses: the checker measured the wire (start code,
    # slot count, bit period, break-to-break) and the receiver decoded the levels.
    ok = bool(packets and len(packets) == case.packets and received
              and all(p.start_code == NULL_START_CODE and p.slots == case.slots - 1
                      for p in packets)
              and all(list(g.values) == expect[1:] and g.null_start for g in received)
              and not rules and not bench.bus.errors
              and bench.drv.fdebug() & ~(FDEBUG_TXSTALL << case.engine) == 0)
    why = "" if ok else (
        f"{len(packets)} checker packet(s), {len(received)} receiver packet(s), "
        f"decoded {[list(g.values) for g in received][:1]} want {expect[1:]}; rules={rules}; "
        f"FDEBUG={bench.drv.fdebug():#010x}; bus errors={len(bench.bus.errors)}")

    period = bench.drv.plan.tick_ticks
    bits = 32 * case.slots
    # The window is one frame's slot grid, anchored on the checker's own measurements of that
    # packet: the BREAK's start and length, the MAB after it, and the bit period it fitted.
    # The second packet, so nothing of the first one's start-up is inside the window.
    if packets:
        pkt = packets[-1]
        grid = pkt.break_tick + pkt.break_ticks + int(pkt.mab_ticks or 0)
        t0 = grid
        t1 = int(grid + bits * (pkt.bit_ticks or period))
    else:
        edges = [e.t for e in probe.edges_on([net])]
        t0, t1 = (edges[0], edges[-1]) if edges else (0, 0)
    tx, rx_peak = levels(probe, t0, t1)
    return Trial(
        protocol="dmx512", knob="bit_rate", value=float(bit_rate), unit="bit/s", ok=ok,
        why=why, bits=bits, window=(t0, t1), nets=(net,),
        **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=enable_latency(probe), response=None, response_kind="",
        response_note="",
        extra={"slot_ticks": period, "packets": len(packets),
               "checker_bit_ticks": packets[0].bit_ticks if packets else None,
               "checker_bit_rate_hz": packets[0].bit_rate if packets else None,
               "checker_rules": rules, "sm_cycles_per_slot": bench.drv.plan.sm_cycles_per_bit},
    )


# ---------------------------------------------------------------------------------------------
# wiegand: a reader's pulse is a request, the card's 26 pulses are the answer
# ---------------------------------------------------------------------------------------------


def wiegand_trial(pw_ns: float) -> Trial:
    """`fw/wiegand` at a requested pulse width: the reader asks, the card answers with 26 bits.

    The knob is the pulse width in nanoseconds and faster is smaller.  A Wiegand bit is a low
    pulse (the 0 line) followed by a high pause, so a frame of 26 bits is 26 pulses and 26
    pauses: the bits are `WiegandReceiver`'s own decoded frame and the window runs from the
    first pulse's fall to the last pulse's rise plus one measured pulse-and-pause period.
    """
    from fw.wiegand.test_wiegand import CARD, Case, Bench

    case = Case(f"perf-{pw_ns:g}", float(pw_ns))
    try:
        bench = Bench(case)
    except ValueError as exc:
        return Trial(protocol="wiegand", knob="pulse_width", value=float(pw_ns),
                     unit="ns", ok=False, why=f"the driver refused this pulse width: {exc}")
    nets = (f"p{bench.driver.d0_pin}", f"p{bench.driver.d1_pin}")
    probe = attach(bench.bus, bench.device, bench.chip, nets=nets)
    refused = ""
    try:
        bench.start()
    except ValueError as exc:
        return Trial(protocol="wiegand", knob="pulse_width", value=float(pw_ns),
                     unit="ns", ok=False, why=f"the driver refused this pulse width: {exc}")

    rules = sorted(bench.rules())
    pulses = list(bench.checker.pulses)
    card = bench.card()
    ok = bool(bench.checker.clean and bench.receiver.clean and card is not None
              and (card.facility, card.number) == tuple(CARD) and card.parity_ok
              and len(pulses) == bench.driver.pairs and not rules and not bench.bus.errors
              and bench.driver.fdebug() & 0x0303_0303 == 0)
    why = "" if ok else (
        f"decoded card={card} want FC {CARD[0]} card {CARD[1]}; {len(pulses)} pulses want "
        f"{bench.driver.pairs}; rules={rules}; checker clean={bench.checker.clean}; "
        f"receiver clean={bench.receiver.clean}; FDEBUG={bench.driver.fdebug():#010x}")

    # The reader's request pulse is the fall on D0 that comes before the card's first bit.
    falls = fall_events(probe, nets[0])
    # Both lines are open drain, so the chip "drives" by releasing: its own first pin edge is
    # a rise, not a fall.  `enable_latency` would otherwise time the *reader's* pulse.
    release = next((e.t for e in probe.edges
                    if e.t >= (probe.first_ctrl_enable().t if probe.first_ctrl_enable() else 0)
                    and e.net in nets and e.level == 1), None)
    gaps = [b - a for a, b in zip([p.fall for p in pulses], [p.fall for p in pulses][1:])]
    period = median(gaps) if gaps else 0.0
    bits = len(pulses)
    t0 = pulses[0].fall if pulses else 0
    # The window ends one pulse-and-pause period after the last pulse's fall, which is where
    # the frame ends (the pause after the last bit is not a wire edge).
    t1 = int(pulses[-1].fall + period) if pulses else t0
    tx, rx_peak = levels(probe, t0, t1)
    # The response: the reader's request pulse, then the card's first data pulse.
    response = pin_response(probe, falls[0], nets) if falls else None
    return Trial(
        protocol="wiegand", knob="pulse_width", value=float(pw_ns), unit="ns", ok=ok, why=why,
        bits=bits, window=(t0, t1), nets=nets, **sm_fields(probe, t0, t1),
        peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=(None if release is None or probe.first_ctrl_enable() is None
                        else release - probe.first_ctrl_enable().t),
        response=response, response_kind="pin",
        response_note=f"the reader's D0 request pulse in cycle {falls[0]} to the card's first "
                      f"data edge" if falls else "",
        extra={"checker_rules": rules, "pulse_period_ticks": int(period),
               "pulse_width_ticks": pulses[0].width if pulses else 0,
               "frame_bits": bits, "driver_refused": refused},
    )


# ---------------------------------------------------------------------------------------------
# quadrature: the fastest input the chip can follow, which the *encoder* sets
# ---------------------------------------------------------------------------------------------


def quadrature_trial(rpm: float) -> Trial:
    """`fw/quadrature` with an encoder turned at `rpm`: 8 counts, x4, judged three ways.

    The knob is the shaft speed, because for an input protocol the rate is imposed by the
    world, not by a divider: `CLKDIV` is already at its floor of 1.0.  The verdict is the one
    `fw/quadrature/test_quadrature.py` uses -- the chip's position, the model's own decoder's
    position and the encoder's position all equal, no decoder error, no checker violation and
    no dropped word.  A bit here is one quadrature transition: four make a count, so the bits
    on the wire are four times the counts the shaft made.
    """
    from fw.quadrature.test_quadrature import NS_PER_TICK, PROFILE, Case, build
    from verif.vip.quadrature import profile_minimums

    # Above the AS5047P profile's own maximum the checker's complaints are the profile's, not
    # the chip's (`test_max_speed_on_the_iss` says so and writes the same rows), so the verdict
    # up there is whether the chip followed the shaft, not whether a real encoder could.
    profile_max_rpm = profile_minimums(NS_PER_TICK, PROFILE)["max_rpm"]
    case = Case(f"perf-{rpm:g}", float(rpm), 0.002)          # 8 counts, whatever the speed
    bench = build(case)
    nets = tuple(f"p{p}" for p in case.pins)
    probe = attach(bench.bus, bench.dev, bench.dev.chip, nets=nets)
    bench.drv.load()
    bench.drv.start()
    bench.enc.queue_turns(case.turns, case.rpm)
    ran = bench.drv.run_until(lambda: bench.enc.idle, 400_000)
    bench.drv.pump(300)
    bench.drv.flush()
    bench.ticks = bench.bus.t
    agree = (bench.drv.position == bench.dec.position == bench.enc.position)
    ok = bool(ran and agree and not bench.drv.decoder.errors
              and (not bench.chk.violations or rpm > profile_max_rpm)
              and bench.drv.fdebug() & 0x0303_0303 == 0 and bench.clean_chip)
    why = "" if ok else (
        f"ran={ran} chip={bench.drv.position} model={bench.dec.position} "
        f"encoder={bench.enc.position} counts={bench.counts()}; "
        f"decoder errors={len(bench.drv.decoder.errors)}; "
        f"violations={len(bench.chk.violations)}; FDEBUG={bench.drv.fdebug():#010x}")

    edges = [e.t for e in probe.edges_on(nets)]
    bits = 4 * bench.counts()
    t0 = edges[0] if edges else 0
    t1 = edges[-1] if edges else 0
    tx, rx_peak = levels(probe, t0, t1)
    response = fifo_response(probe, t0) if t0 else None
    return Trial(
        protocol="quadrature", knob="rpm", value=float(rpm), unit="revolutions/min",
        ok=ok, why=why, bits=bits, window=(t0, t1), nets=nets,
        **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=None, response=response, response_kind="fifo",
        response_note=f"the first A/B edge in cycle {t0} to the first RX FIFO push"
                      if response else "",
        extra={"counts": bench.counts(), "ticks_per_count": 750_000.0 / rpm,
               "chip_position": bench.drv.position, "model_position": bench.dec.position,
               "encoder_position": bench.enc.position, "clkdiv": bench.drv.clkdiv,
               "min_step_ticks": bench.drv.min_step_ticks,
               "ticks_per_sample": bench.drv.ticks_per_sample,
               "violations": len(bench.chk.violations),
               "profile_max_rpm": profile_max_rpm},
    )


# ---------------------------------------------------------------------------------------------
# necir: an IR receiver, and the wire rate is the NEC standard's, not this chip's
# ---------------------------------------------------------------------------------------------


def necir_trial(sm_cycles_per_unit: float) -> Trial:
    """`fw/necir` at a given sampling grid: one remote press, decoded into one FIFO word.

    The knob is `S`, the SM cycles per protocol unit (a half bit), which is what the program
    spends per sample; the wire rate is the NEC standard's and is *measured*, not assumed.  The
    boundary is the sampling grid rather than a rate: below `MIN_SM_CYCLES_PER_UNIT` the driver
    refuses the program, and above `MAX` the samples miss the windows `check_program` lists.
    The verdict is the checker plus the word the engine pushed, which is compared with the
    model's own `verif.vip.necir.encode_frame` for the same address and command.
    """
    from fw.necir.driver import MAX_SM_CYCLES_PER_UNIT, MIN_SM_CYCLES_PER_UNIT, build_program
    from fw.necir.test_necir import Case, build, press
    from verif.vip.necir import encode_frame

    s_per_unit = int(round(sm_cycles_per_unit))
    case = Case(f"perf-s{s_per_unit}", sm_cycles_per_unit=s_per_unit)
    try:
        build_program(s_per_unit)
    except Exception as exc:
        return Trial(protocol="necir", knob="sm_cycles_per_unit", value=float(s_per_unit),
                     unit="SM cycles per half bit", ok=False,
                     why=f"the driver refused this sampling grid ({MIN_SM_CYCLES_PER_UNIT}.."
                         f"{MAX_SM_CYCLES_PER_UNIT} are legal): {exc}")
    rig = build(case)
    probe = attach(rig.bus, rig.dev, rig.dev.chip, nets=(rig.rx.net,))
    address, command = 0x04, 0x08
    press(rig, address, command)
    rules = sorted({v.rule for v in rig.chk.violations})
    raws = list(rig.rx.raws)
    ok = bool(raws == [encode_frame(address, command)] and not rules and not rig.bus.errors
              and rig.rx.chip.fdebug == 0)
    why = "" if ok else (
        f"words={[hex(w) for w in raws]} want {[hex(encode_frame(address, command))]}; "
        f"rules={rules}; FDEBUG={rig.rx.chip.fdebug:#010x}; bus errors={len(rig.bus.errors)}")

    # The window is the IR frame: its first edge to the cycle the last word landed in the FIFO
    # (a receiver's last act on the wire is the sample that completes the frame).
    edges = [e.t for e in probe.edges_on([rig.rx.net])]
    pushes = [t for t in range(1, len(probe.rx_level))
              if probe.rx_level[t] > probe.rx_level[t - 1]]
    bits = 68                                    # leader + address + command + inverse, 8 bits each
    t0 = edges[0] if edges else 0
    t1 = pushes[-1] if pushes else t1_placeholder(t1=None, edges=edges)
    tx, rx_peak = levels(probe, t0, t1)
    response = fifo_response(probe, t0) if t0 else None
    return Trial(
        protocol="necir", knob="sm_cycles_per_unit", value=float(s_per_unit),
        unit="SM cycles per half bit", ok=ok, why=why, bits=bits, window=(t0, t1),
        nets=(rig.rx.net,), **sm_fields(probe, t0, t1), peak_tx=tx, peak_rx=rx_peak,
        enable_to_edge=None, response=response, response_kind="fifo",
        response_note=f"the first IR edge in cycle {t0} to the first RX FIFO push"
                      if response else "",
        compression=1000,        # fw/necir's bench calls a protocol unit 562.5 ns, not 562 500
        extra={"checker_rules": rules, "words": [hex(w) for w in raws],
               "unit_ns_bench": 562.5, "unit_ns_real": 562_500.0, "tolerance": case.tolerance,
               "violations": len(rig.chk.violations)},
    )


def t1_placeholder(t1=None, edges=()):
    """The fallback window end when the FIFO never filled: one unit after the last edge."""
    return (edges[-1] + 1125) if edges else 0        # 562.5 us = 28 125 ticks at 20 ns


# ---------------------------------------------------------------------------------------------
# The 14 protocols: their speed axes and the ladder of settings the sweep walks
# ---------------------------------------------------------------------------------------------

#: Every protocol in `fw/`, its speed knob and the settings to try, fastest first.  The sweep
#: stops at the first setting the protocol's own checker accepts, so the *last* entry of a
#: ladder is the slowest setting anyone bothered to measure and the boundary is the rung above
#: the answer.  `targetable` names the targets a rig is also run on; `FW_TARGET=rtl` runs the
#: four protocols listed there against `src/pio_chip.sv`.
#:
#: Those four are the ones whose payload is short enough to re-measure at the RTL harness's few
#: thousand cycles a second (spi, ps2, swd and onewire); the other ten run for hundreds of
#: thousands of core cycles, which is minutes of wall clock per rung under Verilator.  Nothing
#: about the payload decides *whether* a rig can be RTL-targetable -- that is `fw/targets.py`,
#: and `verif/perf/measure.py` installs it and then checks the install took, because the earlier
#: per-file `from fw.rtl_harness import install` blocks these rigs used to rely on are gone
#: (there is now one registry, `fw/conftest.py` its only pytest
#: caller).  `fw/dmx512` is deliberately *not* in this set: its payload is two 25-slot frames,
#: which is the same wall-clock argument as the other ten.
LADDER: tuple[Protocol, ...] = (
    Protocol("spi", "clkdiv_int", "core cycles per SM cycle", "down",
             (1, 2, 4, 8, 16), spi_trial, responds="pin", targetable=("iss", "rtl"),
             note="the divider's floor is 1.0; a faster bit needs a different program "
                  "(the negative control in verif/perf/selftest.py builds one)"),
    Protocol("i2c", "clkdiv_int", "core cycles per SM cycle", "down",
             (4, 8, 10, 11, 12, 13, 14, 20, 67), i2c_trial, responds="pin",
             note="fSCL and the phase minimums are what stop it"),
    Protocol("jtag", "clkdiv_int", "core cycles per SM cycle", "down",
             (1, 2, 4, 8, 16), jtag_trial, responds="pin",
             note="the scan itself is TCK-period bound; the divider only slows it"),
    Protocol("ps2", "clkdiv_int", "core cycles per SM cycle", "down",
             (60, 70, 80, 90, 94, 100, 120, 147), ps2_trial, responds="pin",
             targetable=("iss", "rtl"),
             note="below 94 the driver refuses, so those rungs are loaded deliberately"),
    Protocol("swd", "bit_period", "core cycles per bit", "down",
             (9, 10, 11, 14, 17, 20, 27), swd_trial, responds="pin", targetable=("iss", "rtl"),
             note="the program's two delay fields; LOW cannot go below 0 nor above 14"),
    Protocol("onewire", "clkdiv_int", "core cycles per SM cycle", "down",
             (40, 44, 46, 47, 50, 55, 58, 62), onewire_trial, responds="pin",
             targetable=("iss", "rtl"),
             note="the DS18B20 tables' slot window is what the checker holds it to"),
    Protocol("uart_tx", "baud", "bit/s", "up",
             (6_250_000.0, 2_000_000.0, 921_600.0, 115_200.0, 9_600.0), uart_tx_trial,
             note="transmit only: nothing on the wire makes it answer"),
    Protocol("uart_rx", "baud", "bit/s", "up",
             (50e6 / 4, 50e6 / 6, 50e6 / 8, 50e6 / 12, 50e6 / 16, 50e6 / 32), uart_rx_trial,
             responds="fifo",
             note="S = 50 MHz / baud must be even and 4..32; S = 4 is one SM cycle short of "
                  "back-to-back frames, which the ladder measures"),
    Protocol("manchester", "bit_rate", "bit/s", "up",
             (12_500_000.0, 12_000_000.0, 10_000_000.0, 6_250_000.0, 5_000_000.0, 1_000_000.0),
             manchester_trial,
             note="two programs: 5 SM cycles/bit to 10 Mbit/s, 4 to 12.5 Mbit/s, where the "
                  "judge's own windows touch"),
    Protocol("ws2812", "program", "0 = ws2812, 1 = ws2812_fast", "up", (1.0, 0.0),
             ws2812_trial,
             note="the bit period is in the program, not in a divider"),
    Protocol("dmx512", "bit_rate", "bit/s", "up",
             (260_000.0, 255_000.0, 250_000.0, 245_000.0, 200_000.0, 100_000.0), dmx512_trial,
             note="260 kbit/s is one rung above the tables' 245..255 window and is the rung the "
                  "boundary cell in research/perf/b/README.md quotes, so it is on the ladder "
                  "rather than in the table without a measurement behind it"),
    Protocol("wiegand", "pulse_width", "ns", "down",
             (20_000.0, 30_000.0, 50_000.0), wiegand_trial, responds="pin",
             note="a full 26-bit frame is 780 000 core cycles at 50 us; the ladder is short "
                  "because of that"),
    Protocol("quadrature", "rpm", "revolutions/min", "up",
             (896_000.0, 448_000.0, 224_000.0, 112_000.0, 56_000.0, 28_000.0), quadrature_trial,
             responds="fifo",
             note="CLKDIV is already at its floor, so the encoder sets the rate; the "
                  "AS5047P profile stops at 28 000 RPM, the chip keeps counting well past it"),
    Protocol("necir", "sm_cycles_per_unit", "SM cycles per half bit", "down",
             (6, 7, 8, 9, 10, 12), necir_trial, responds="fifo",
             note="the wire rate is the NEC standard's; the knob is the sampling grid, and "
                  "fw/necir's bench runs it on a 1:1000 compressed time base"),
)

PROTOCOLS: dict[str, Protocol] = {p.name: p for p in LADDER}


def ladder_for(name: str) -> Protocol:
    if name not in PROTOCOLS:
        raise KeyError(f"no such protocol: {name}. Known: {', '.join(sorted(PROTOCOLS))}")
    return PROTOCOLS[name]


def measure(name: str, speed: float) -> Trial:
    """One measured trial of `name` at `speed`, from its own ladder entry."""
    return ladder_for(name).trial(speed)

"""The wire's own record of a UART frame: how many bits it actually put on the line.

`verif/vip/uart.py`'s `UartRx` decodes a frame and reports a framing error when a *stop* bit is
low, which is the right check for the number of stop bits a receiver cares about -- but it cannot
tell "one stop bit" from "two" on a quiet line, because both leave the line high where the second
stop bit would be.  The firmware supports 7 and 9 data bits and two stop bits, and the
data-bit count has the same problem from the other side: a transmitter that shifted a ninth zero
bit out for a 7-bit frame would still decode as the same byte.

So this is a checker *beside* the VIP: it records every transition of one net and measures the
frame geometry in bit times -- how long the line was low (the start bit plus the data bits) and
how long it was high afterwards (the stop bits plus the engine's idle gap).  Both numbers come
from the wire and nothing else, and both are exact to a tick:

* `low_bit_times` is `1 + data_bits` for the framing that was sent, and a whole bit time away from
  that for a program with the wrong number of data bits, so `1.0` is the resolution and not a
  fudge.
* `stop_bits` is read as the *difference* between the high run of a two-stop frame and the high
  run of a one-stop frame with everything else identical, so the engine's idle gap cancels and the
  answer is "exactly one bit time more".

The two controls that keep this from being decoration live in `fw/uart_tx/test_uart_tx_framing.py`:
`test_data_bit_count_is_measurable_on_the_wire` plants a payload whose last data bit is 1, which
must read one bit time short of `Frame.low_bit_times` on the same framing, and
`test_two_stop_bits_hold_the_line_one_bit_time_longer` runs a program with the second
`set pins, 1` deleted, which reads zero bit times longer.  Neither would hold if the geometry
measurements were decoration.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional

from verif.vip.core import Device

IDLE = 1


class LineRecorder(Device):
    """Every 0 -> 1 and 1 -> 0 transition of one net, with the tick it happened on.

    Passive: it never drives, and it reads the net the way every other device on the bus reads
    it (`verif/vip/core.py`'s convention: a value driven at tick `t` is seen by devices at `t+1`).
    """

    def __init__(self, net: str, name: Optional[str] = None) -> None:
        self.net = net
        self.name = name or f"linerec[{net}]"
        self.transitions: list[tuple[int, int]] = []
        self._last = IDLE

    def tick(self, t: int, pins: Mapping[str, int]) -> dict[str, Optional[int]]:
        level = pins.get(self.net, IDLE)
        if level != self._last:
            self.transitions.append((t, level))
            self._last = level
        return {}


@dataclass(frozen=True)
class Frame:
    """One frame as the line recorded it, in ticks and in bit times."""

    start_tick: int                   # the tick the start bit began, in observation time
    low_ticks: int                    # start bit plus data bits: the line low until it goes high
    high_ticks: Optional[int] = None  # stop bits plus idle gap, up to the next start bit; None
                                       # for the last frame of a run, which the line never ends

    def low_bit_times(self, tpb: float) -> float:
        return self.low_ticks / tpb

    def high_bit_times(self, tpb: float) -> float:
        if self.high_ticks is None:
            raise ValueError(
                f"the frame at tick {self.start_tick} is the last one in the run: nothing follows "
                "it on the line, so its stop bits cannot be measured"
            )
        return self.high_ticks / tpb

    def period_bit_times(self, tpb: float) -> float:
        return (self.low_ticks + self.high_bit_times(tpb) * tpb) / tpb


def frames(transitions, starts, offset: int = 0) -> list[Frame]:
    """The geometry of the frames whose start bits are at `starts`.

    `starts` are the start-bit ticks in *drive* time, the way `verif.vip/uart.py`'s `RxByte.tick`
    reports them; the transition log is in observation time, so `offset` is the driver's latency
    (1 tick on `verif/vip/core.py`'s bus).  The caller passes the frames the judge found rather
    than letting this module find the falling edges itself, because a 1 -> 0 transition inside the
    data bits looks exactly like a start bit on a transition log; the tests assert that the number
    of frames measured here equals the number the judge decoded, so a judge that invented a frame
    cannot pass unnoticed either.

    The low run is measured to the **last** rising edge before the *next frame's* start bit, not
    the first rising edge: the data bits are whatever the byte says, so the first rise can be one
    bit into the frame and the first falling edge after the start bit is a data bit, not a frame.
    `starts` is therefore read as a sequence of frame boundaries and each frame is measured against
    the one after it; the last frame of a run has no next start bit and reports `high_ticks` None.
    The last rise is the end of the frame's final low period, which is the end of the last data bit
    whenever that bit is 0 -- the payload the geometry tests use -- and one bit time earlier when it
    is 1.  That difference is the resolution of the measurement, and it is asserted both ways.
    """
    out: list[Frame] = []
    for i, start in enumerate(starts):
        t = start + offset
        limit = starts[i + 1] + offset if i + 1 < len(starts) else 1 << 62
        rise = next((tt for tt, ll in reversed(transitions) if ll != 0 and t < tt < limit), None)
        if rise is None:
            raise ValueError(f"the line never came back high after tick {t}: not a frame")
        out.append(Frame(start_tick=t, low_ticks=rise - t,
                         high_ticks=None if i + 1 == len(starts) else limit - rise))
    return out


def low_bit_times(measured: list[Frame], tpb: float) -> list[float]:
    """The low run of each measured frame, in bit times."""
    return [f.low_bit_times(tpb) for f in measured]


def high_bit_times(measured: list[Frame], tpb: float) -> list[float]:
    """The high run after each measured frame's last data bit, in bit times."""
    return [f.high_bit_times(tpb) for f in measured if f.high_ticks is not None]

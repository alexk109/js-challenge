"""Self-tests for the pin-level PS/2 reference models: `make vip-selftest VIP=ps2`.

Everything runs at 50 MHz (20 ns per tick) at both legal line rates, `fast` (30 + 30 us,
16.67 kbit/s) and `slow` (50 + 50 us, 10 kbit/s), with a host, a device and a passive checker
watching the pins. `rig()` builds that bench and the tests then look at the keyboard's scan
codes, at the mouse's packets, at what the host decoded and at what the checker says.

The rule tests come in two flavours, and both matter for the firmware author:

* `timing=` on a model, which is the documented way to drive one part of the bench off-spec
  (`{"clk_low": 10_000}` and the like). Every timing rule has a test like that.
* a test double, for the four rules a well-behaved model cannot break by parameter alone,
  because its own code would be wrong: DATA moved in the wrong half of the clock, DATA moved
  long after the edge that samples it, a frame that stops early and a device that never
  acknowledges. The doubles are subclasses of the models, so they change one decision and
  nothing else.

The last two tests are the important ones for trusting any of this: `test_checker_is_not_vacuous`
and `test_checker_reads_the_frames_off_the_wire`.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus
from verif.vip.ps2 import (
    ACK,
    ECHO,
    FRAME_BITS,
    HOST_FRAME_BITS,
    RESEND,
    SELFTEST_OK,
    Ps2Checker,
    Ps2Device,
    Ps2Host,
    Ps2Keyboard,
    Ps2Mouse,
    attach_checker,
    attach_device,
    attach_host,
    attach_keyboard,
    attach_mouse,
    frame_ticks,
    ns_to_ticks,
    ps2_frame_bits,
    ps2_parity_bit,
    rate_minimums,
)

RATES = ["fast", "slow"]           # 16.67 kbit/s and 10 kbit/s
NS_PER_TICK = 20.0                 # 50 MHz chip clock
LIMIT = 3_000_000                  # generous: one frame is under 100_000 ticks


def rig(rate="fast", device="keyboard", host_rate=None, dev_rate=None, check_rate=None,
        host_timing=None, dev_timing=None, check_timing=None, **dev_kwargs):
    """A bus with a host, one device and a checker.

    Every timing hook is separate, which is how a test drives one part of the bench off-spec
    without touching the others: `host_timing`, `dev_timing`, `check_timing`, and the two rates
    for anything that only wants another line speed. `device=None` leaves the device off.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus, rate=host_rate or rate, timing=host_timing)
    dev = None
    if device == "keyboard":
        dev = attach_keyboard(bus, rate=dev_rate or rate, timing=dev_timing, **dev_kwargs)
    elif device == "mouse":
        dev = attach_mouse(bus, rate=dev_rate or rate, timing=dev_timing, **dev_kwargs)
    elif device == "bare":
        dev = attach_device(bus, rate=dev_rate or rate, timing=dev_timing, **dev_kwargs)
    elif device is not None:
        raise ValueError(f"unknown device {device!r}")
    chk = attach_checker(bus, rate=check_rate or rate, timing=check_timing)
    return bus, host, dev, chk


def talk(host, dev, bus, limit=LIMIT):
    """Run until the host's queue is empty and the device is quiet, then a little longer."""
    assert host.run(bus, limit=limit), "the host did not drain its queue"
    if dev is not None:
        assert dev.run(bus, limit=limit), "the device did not drain its queue"
    bus.run(2_000)
    return host, dev


def ok(chk):
    """Assert the checker is happy, printing its report when it is not."""
    assert chk.clean, chk.report()
    return chk


# ---- test doubles: one wrong decision each, so exactly one rule should fire ---------------
class LateDataHost(Ps2Host):
    """Puts each bit on DATA inside the clock *high* period, `late_ticks` after the rising
    edge, instead of during the low period before it.

    The framing is untouched -- the device samples on the rising edge and DATA is stable long
    before then -- so only the host->device rule can complain: the AT Technical Reference says
    "Data must be stable within 1 microsecond after the rising edge of the clock line".
    """

    late_ticks = 1_000        # 50 us at 20 ns/tick: late for the 1 us rule, before the next rise

    def _low(self, t):
        if self.cnt >= self.clk_low:
            self.phase, self.cnt = "high", 0
            self.clk_out = None
        else:
            self.cnt += 1

    def _high(self, t, pins):
        if self.bit == HOST_FRAME_BITS - 1 and self.cnt == 0:
            self.data_out = None              # the ack bit belongs to the device
        if 1 <= self.bit <= FRAME_BITS - 1 and self.cnt == self.late_ticks:
            self.data_out = self.next_data
        super()._high(t, pins)


class LowPhaseData(Ps2Device):
    """Moves DATA at the start of each clock *low* period instead of inside the high one.

    T1 ("time from DATA transition to falling edge of CLK: 5/25 us") and T2 ("time from rising
    edge of CLK to DATA transition: 5/T4-5 us") leave DATA exactly one place, inside the high
    period, so a move during the low period is outside the window both times. The byte still
    arrives, because the device puts the next bit on the wire in the same tick it pulls the
    clock low and the host samples on that edge, so only the rule breaks.
    """

    def _compute_timings(self):
        super()._compute_timings()
        self.data_after_rise = 10 ** 9      # never inside the high period

    def _low(self):
        if self.need_data:
            self.data_out = self.next_data
            self.need_data = False
        super()._low()


class ShortFrame(Ps2Device):
    """Stops clocking after `stop_after` bits and gives the byte up: the frame the checker saw
    begin never finishes, and nothing follows it on the wire."""

    stop_after = 8

    def _high(self):
        if self.bit >= self.stop_after - 1:
            self.phase, self.cnt = "idle", 0
            self.clk_out, self.data_out = None, None
            self.idle_left = self.bus_free
            self.tx.popleft()          # a real device would retry; this one gives up
            return
        super()._high()


class NoAck(Ps2Device):
    """Clocks the host's twelfth bit but never pulls DATA low for it."""

    def _compute_timings(self):
        super()._compute_timings()
        self.ack_drive = 10 ** 9


class StartGlitch(Ps2Device):
    """Drops the start bit again before the first clock edge.

    DATA falls on an idle-high clock, so the frame has begun, and DATA rises again while the
    clock is still high, so the first bit anything samples is a 1. Nothing else about the wire
    is wrong: the eleven clocks, the parity and the stop bit are all there.
    """

    glitch_ticks = 750

    def _compute_timings(self):
        super()._compute_timings()
        # The first high period stays a legal 30 us, so the only thing wrong with this wire is
        # the start bit: nothing else in the frame moves out of its window.
        self.start_hold = max(self.clk_low, self.glitch_ticks + 1)
        self.glitched = False

    def _step_phase(self, t, pins, phase):
        if phase == "start" and not self.glitched and self.cnt >= self.glitch_ticks:
            self.glitched = True
            self.data_out = None          # the start bit is gone before the clock moves
        super()._step_phase(t, pins, phase)


class BadFrame(Ps2Device):
    """A device with a chosen fault in the 11 bits it clocks out.

    `fault` is "parity", "stop" or "start": each flips one bit of `ps2_frame_bits()` and leaves
    everything else, so the checker sees exactly that rule and the byte still arrives.
    """

    fault = "parity"

    def frame_bits(self, value):
        bits = list(ps2_frame_bits(value))
        if self.fault == "parity":
            bits[9] ^= 1
        elif self.fault == "stop":
            bits[10] ^= 1
        elif self.fault == "start":
            bits[0] ^= 1
        else:
            raise ValueError(f"unknown fault {self.fault!r}")
        return tuple(bits)


class BadHostFrame(Ps2Host):
    """A host that puts a wrong parity bit on the wire, so the device answers 0xFE."""

    def frame_bits(self, value):
        bits = list(ps2_frame_bits(value))
        bits[9] ^= 1
        return tuple(bits)


def attach_double(bus, cls, rate="fast", **kwargs):
    """Attach a test double in place of a device, keeping the net and tick setup."""
    bus.add_net("clk") if "clk" not in bus.nets else None
    bus.add_net("data") if "data" not in bus.nets else None
    dev = cls(bus=bus, rate=rate, **kwargs)
    bus.attach(dev)
    return dev


# ---- the models talking to each other ---------------------------------------------------
@pytest.mark.parametrize("rate", RATES)
def test_reset_and_self_test(rate):
    """0xFF is 12 clocks to the device and the keyboard answers 0xFA, then its self-test byte."""
    bus, host, kbd, chk = rig(rate)
    host.send(0xFF)
    talk(host, kbd, bus)

    assert host.errors == [] and kbd.errors == []
    assert host.acks == 1 and host.nacks == 0
    assert [s.ack for s in host.sent] == [True]
    assert host.values == [ACK, SELFTEST_OK]
    assert [b.value for b in kbd.received] == [0xFF] and kbd.received[0].ack
    ok(chk)
    assert (chk.frames_to_device, chk.frames_to_host) == (1, 2)
    assert chk.frames[0].bits == ps2_frame_bits(0xFF) + (0,)      # the ack bit is DATA low
    assert chk.frames[0].ack is True and chk.acks == 1
    assert [f.bits for f in chk.frames[1:]] == [ps2_frame_bits(v) for v in (ACK, SELFTEST_OK)]
    assert all(f.direction == "to_host" for f in chk.frames[1:])


@pytest.mark.parametrize("rate", RATES)
def test_every_keyboard_command(rate):
    """The whole OSDev keyboard command table, one run: LEDs, identify, echo, scan code set."""
    bus, host, kbd, chk = rig(rate)
    host.send(0xED, 0x07)          # set LEDs: ack, then ack for the argument
    talk(host, kbd, bus)
    assert host.values == [ACK, ACK]
    assert (kbd.scroll_lock, kbd.num_lock, kbd.caps_lock) == (True, True, True)

    host.send(0xF2)               # identify: ack, 0xAB, 0x83
    talk(host, kbd, bus)
    assert host.values[-3:] == [ACK, 0xAB, 0x83]

    host.send(0xEE)               # echo: 0xEE, not an ack
    talk(host, kbd, bus)
    assert host.values[-1] == ECHO

    host.send(0xF0, 0x00)         # ask which scan code set is in use
    talk(host, kbd, bus)
    assert host.values[-2:] == [ACK, 2] and kbd.scan_code_set == 2

    host.send(0xF0, 0x01)         # select set 1, then ask again
    talk(host, kbd, bus)
    assert host.values[-2:] == [ACK, ACK] and kbd.scan_code_set == 1
    host.send(0xF0, 0x00)
    talk(host, kbd, bus)
    assert host.values[-2:] == [ACK, 1]

    host.send(0xF6)               # set defaults puts set 2 and the LEDs back
    talk(host, kbd, bus)
    assert host.values[-1] == ACK
    assert (kbd.scan_code_set, kbd.leds, kbd.enabled) == (2, 0, True)

    host.send(0xF5)               # disable: nothing is scanned until 0xF4
    talk(host, kbd, bus)
    assert kbd.enabled is False
    assert kbd.tap("a") == () and kbd.run(bus, limit=LIMIT)
    assert host.values[-1] == ACK and chk.frames_to_host == len(host.values)

    host.send(0xF4)               # enable again
    talk(host, kbd, bus)
    kbd.tap("a")
    assert kbd.run(bus, limit=LIMIT)
    assert host.values[-3:] == [ACK, 0x1C, 0xF0, 0x1C][1:]      # set 2: make, F0, make
    ok(chk)
    assert host.errors == [] and kbd.errors == []


def test_scan_code_sets_and_extended_keys():
    """Set 1 breaks with bit 7, set 2 and 3 with the 0xF0 prefix, and extended keys carry 0xE0."""
    kbd = Ps2Keyboard(scan_code_set=2)
    assert kbd.codes("a") == (0x1C,)                 # set 2 make
    assert kbd.codes("a", make=False) == (0xF0, 0x1C)
    assert kbd.codes("enter") == (0x5A,) and kbd.codes("space") == (0x29,)
    assert kbd.codes("f5") == (0x03,)                # set 2 F5 is 03, F6 is 0B
    assert kbd.codes("right") == (0xE0, 0x4D)        # extended: the 0xE0 prefix
    assert kbd.codes("right", make=False) == (0xE0, 0xF0, 0x4D)

    kbd.scan_code_set = 1
    assert kbd.codes("a") == (0x1E,)                 # set 1 make
    assert kbd.codes("a", make=False) == (0x9E,)     # set 1 break is make | 0x80
    assert kbd.codes("right") == (0x4D,)             # no prefix: the controller adds 0xE0
    kbd.scan_code_set = 3
    assert kbd.codes("a") == (0x1C,)                 # set 3 uses the set 2 make codes

    with pytest.raises(KeyError):
        kbd.codes("nosuchkey")
    with pytest.raises(ValueError):
        Ps2Keyboard(scan_code_set=4)


def test_scan_codes_reach_the_host_in_order():
    """ctrl+c typed in set 2 is 0x14 0x21 then 0xF0 0x21 0xF0 0x14, six frames, all clean."""
    bus, host, kbd, chk = rig("fast")
    kbd.press("ctrl_l")
    kbd.press("c")
    kbd.release("c")
    kbd.release("ctrl_l")
    assert kbd.run(bus, limit=LIMIT)
    assert host.values == [0x14, 0x21, 0xF0, 0x21, 0xF0, 0x14]
    ok(chk)
    assert chk.frames_to_host == 6 and chk.frames_to_device == 0
    assert bus.errors == [], "open drain: two devices cannot contend, and none did"


def test_mouse_reset_packet_and_commands():
    """The mouse command table, then a movement report the host decodes the way OSDev says."""
    bus, host, mouse, chk = rig("fast", device="mouse")
    host.send(0xFF)                   # reset: ack, self test, then the device ID 0x00
    talk(host, mouse, bus)
    assert host.values == [ACK, SELFTEST_OK, 0x00]

    host.send(0xF5)                   # disable reporting: a report is built but not sent
    talk(host, mouse, bus)
    assert host.values[-1] == ACK and mouse.enabled is False
    mouse.move(3, 3)
    assert mouse.run(bus, limit=LIMIT)
    assert host.values[-1] == ACK, "nothing may go on the wire while reporting is off"

    host.send(0xF4)                   # enable reporting
    talk(host, mouse, bus)
    host.flush_rx()
    packet = mouse.move(10, -5, buttons=1)
    assert mouse.run(bus, limit=LIMIT)
    assert host.values == [0x29, 0x0A, 0xFB], "always-one, left button, X=+10, Y=-5"
    assert packet.signed == (10, -5)  # the host's sign extension, the OSDev arithmetic
    ok(chk)

    host.send(0xF2, 0xE8, 0x02, 0xF3, 200)    # identify, resolution 4/mm, 200 samples/s
    talk(host, mouse, bus)
    assert host.values[-6:] == [ACK, 0x00, ACK, ACK, ACK, ACK]
    assert (mouse.id_byte, mouse.resolution, mouse.sample_rate) == (0x00, 4, 200)
    host.send(0xE7)                   # scaling 2:1
    talk(host, mouse, bus)
    assert mouse.scaling == 2
    host.flush_rx()
    mouse.move(1, 1)                  # 1 count at 4 counts/mm and 2:1 scaling is 8 on the wire
    assert mouse.run(bus, limit=LIMIT)
    assert host.values == [0x09, 0x08, 0x08], host.values      # the left button is still down
    host.flush_rx()
    mouse.move(1, 1, buttons=0)       # releasing it is just the next report
    assert mouse.run(bus, limit=LIMIT)
    assert host.values == [0x08, 0x08, 0x08], host.values      # head, X=+8, Y=+8
    ok(chk)

    with pytest.raises(ValueError):
        Ps2Mouse(resolution=3)
    with pytest.raises(ValueError):
        Ps2Mouse(sample_rate=50)
    with pytest.raises(ValueError):
        Ps2Mouse(scaling=3)


def test_mouse_intellimouse_wheel():
    """The 200/100/80 sample-rate magic turns the wheel on: four-byte packets, ID 0x03."""
    bus, host, mouse, chk = rig("fast", device="mouse")
    assert mouse.packet_bytes == 3 and mouse.id_byte == 0x00
    host.send(0xF3, 200, 0xF3, 100, 0xF3, 80)
    talk(host, mouse, bus)
    assert mouse.wheel is True and mouse.packet_bytes == 4 and mouse.id_byte == 0x03
    host.send(0xF2)
    talk(host, mouse, bus)
    assert host.values[-2:] == [ACK, 0x03]
    host.flush_rx()
    mouse.move(4, -4, buttons=3, z=-3)
    assert mouse.run(bus, limit=LIMIT)
    assert host.values == [0x2B, 0x04, 0xFC, 0xFD], host.values   # head, X=+4, Y=-4, Z=-3
    ok(chk)


def test_host_waits_for_the_bus_before_it_sends_again():
    """A device that answers as soon as it is spoken to gets the bus first: the host waits for
    the lines to be idle again instead of counting ticks."""
    bus, host, dev, chk = rig("fast", device="bare")
    host.send(0x11, 0x22)             # two commands queued back to back
    assert host.run(bus, limit=LIMIT)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)
    # wire order: 0x11 to the device, its 0xFA back, then 0x22 to the device
    assert [b.value for b in dev.received] == [0x11, 0x22]
    assert host.values == [ACK, ACK]
    assert [f.direction for f in chk.frames] == ["to_device", "to_host",
                                                 "to_device", "to_host"]
    assert [f.value for f in chk.frames] == [0x11, ACK, 0x22, ACK]
    assert chk.frames[0].tick < chk.frames[1].tick   # each answer waits its turn
    ok(chk)


def test_host_retries_a_byte_the_device_did_not_acknowledge():
    """No ack bit is a `nacks` entry and a re-send, and after `retries` tries an error."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus, rate="fast", retries=2)
    dev = attach_double(bus, NoAck)
    chk = attach_checker(bus, rate="fast")
    assert bus.devices == [host, dev, chk]
    host.send(0x55)
    assert host.run(bus, limit=LIMIT)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)

    assert host.nacks == 3 and host.acks == 0          # the byte plus two retries
    assert len(host.sent) == 3 and all(s.ack is False for s in host.sent)
    assert [b.value for b in dev.received] == [0x55] * 3
    assert host.errors and "never acknowledged" in host.errors[0]
    assert chk.no_acks == 3 and chk.acks == 0
    assert chk.rules_broken == {"ack-bit"}, chk.report()
    for v in chk.violations:
        assert v.rule == "ack-bit" and v.tick > 0


def test_resend_command_makes_the_device_repeat_its_last_byte():
    """0xFE is the device's "send that again": the host queues it and the byte comes back."""
    bus, host, kbd, chk = rig("fast")
    kbd.queue(0x11, 0x22)
    assert kbd.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0x11, 0x22]
    host.request_resend()
    talk(host, kbd, bus)
    assert host.values == [0x11, 0x22, 0x22], "the last byte, not the one before it"
    assert kbd.resends == 1
    ok(chk)


def test_device_answers_a_bad_host_frame_with_resend():
    """A host with a wrong parity bit gets 0xFE, and the checker says `parity`."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = BadHostFrame(bus=bus, rate="fast")
    kbd = attach_keyboard(bus, rate="fast")
    chk = attach_checker(bus, rate="fast")
    bus.attach(host)
    host.send(0xF4)
    assert host.run(bus, limit=LIMIT)
    assert kbd.run(bus, limit=LIMIT)
    bus.run(2_000)

    assert host.values == [RESEND], host.values
    assert kbd.parity_errors == 1 and kbd.framing_errors == 0
    assert kbd.received[0].parity_error is True and kbd.received[0].value == 0xF4
    assert kbd.errors and "parity" in kbd.errors[0]
    assert chk.rules_broken == {"parity"}, chk.report()
    assert chk.frames[0].parity_error is True and chk.frames[0].value == 0xF4


# ---- the checker on legal traffic --------------------------------------------------------
@pytest.mark.parametrize("rate", RATES)
def test_checker_measures_the_rate_off_the_wire(rate):
    """Legal traffic at both rates, and the checker reads the rate back off the clock."""
    bus, host, kbd, chk = rig(rate)
    host.send(0xF4, 0xED, 0x02)
    talk(host, kbd, bus)
    kbd.tap("a")
    assert kbd.run(bus, limit=LIMIT)
    bus.run(2_000)
    ok(chk)
    expect = 50_000.0 if rate == "slow" else 30_000.0          # ns per half period
    period = 2 * ns_to_ticks(NS_PER_TICK, expect)
    fastest = 1e6 / (NS_PER_TICK * period)                    # kcycle/s == kbit/s
    assert chk.shortest_period == chk.longest_period == period
    assert chk.fastest_khz == pytest.approx(fastest)
    assert chk.slowest_khz == pytest.approx(fastest)
    assert fastest == pytest.approx(10.0 if rate == "slow" else 100 / 6)
    assert chk.periods >= chk.frames_to_device * HOST_FRAME_BITS
    assert chk.frames_to_host == 6 and chk.frames_to_device == 3
    assert chk.acks == 3 and chk.no_acks == 0


def test_checker_reads_the_frames_off_the_wire():
    """Not vacuity, part one: every byte the checker decoded has the bits it should have.

    It re-derives the 11 (or 12) bits of every frame from `ps2_frame_bits` and compares them
    with what the checker saw on the pins, so a checker that decoded nothing, or decoded
    garbage, or invented frames cannot pass this.
    """
    bus, host, kbd, chk = rig("fast")
    host.send(0xF4, 0xF0, 0x02, 0xED, 0x01)
    talk(host, kbd, bus)
    for key in ("a", "enter", "right", "f5"):
        kbd.tap(key)
    assert kbd.run(bus, limit=LIMIT)
    bus.run(2_000)

    sent = [s.value for s in host.sent]
    got = [f.value for f in chk.frames if f.direction == "to_device"]
    back = [f.value for f in chk.frames if f.direction == "to_host"]
    assert got == sent == [0xF4, 0xF0, 0x02, 0xED, 0x01]
    assert back == host.values == [ACK, ACK, ACK, ACK, ACK, 0x1C, 0xF0, 0x1C,
                                   0x5A, 0xF0, 0x5A, 0xE0, 0x4D, 0xE0, 0xF0, 0x4D,
                                   0x03, 0xF0, 0x03]
    for frame in chk.frames:
        expect = ps2_frame_bits(frame.value)
        if frame.direction == "to_device":
            expect += (0,)                       # the device's acknowledgement bit
            assert frame.ack is True
        else:
            assert frame.ack is None
        assert frame.bits == expect, frame
        assert frame.bits[0] == 0 and frame.bits[10] == 1
        assert not frame.parity_error and not frame.framing_error
        assert frame.end_tick > frame.tick
    assert len(chk.frames) == len(got) + len(back)
    ok(chk)


def test_checker_is_not_vacuous():
    """Not vacuity, part two: the same pins, judged against stricter limits, must fail.

    Two things are being proved. First, that a legal PS/2 exchange is not flagged by mistake.
    Second, that the flags come from the limits the checker was given and not from a constant
    inside it: a checker told the clock must stay low for 40 us rejects a wire that runs at
    30 us per half period, and every violation it reports names that number.
    """
    bus, host, kbd, chk = rig("fast")
    host.send(0xF4)
    talk(host, kbd, bus)
    kbd.tap("a")
    assert kbd.run(bus, limit=LIMIT)
    bus.run(2_000)
    ok(chk)
    assert chk.violations == [] and chk.frames_to_host == 4 and chk.frames_to_device == 1

    tight = ns_to_ticks(NS_PER_TICK, 40_000)
    bus2, host2, kbd2, chk2 = rig("fast", check_timing={"clk_low_min": 40_000,
                                                       "clk_high_min": 40_000})
    host2.send(0xF4)
    talk(host2, kbd2, bus2)
    kbd2.tap("a")
    assert kbd2.run(bus2, limit=LIMIT)
    bus2.run(2_000)

    rules = chk2.rules_broken
    assert {"tCLK-low", "tCLK-high"} <= rules, chk2.report()
    for v in chk2.violations:
        assert v.required in (tight, chk2.min_ticks["rts_hold"]) or v.limit == "flag"
        assert v.measured < v.required and v.tick > 0
    low = chk2.violations_by_rule("tCLK-low")
    assert low and {v.required for v in low} == {tight}
    assert {v.measured for v in low} == {ns_to_ticks(NS_PER_TICK, 30_000)}
    # the framing rules are untouched by a timing limit: only the clock periods broke
    assert not (rules & {"parity", "stop-bit", "start-bit", "ack-bit", "frame-length"})
    assert chk2.frames_to_host == chk.frames_to_host


# ---- every rule, broken on purpose --------------------------------------------------------
def transitions(value: int) -> int:
    """How many times DATA moves while `value` is clocked out, start bit and stop bit included.

    T1, T2 and `data-window` constrain a *transition*, so a run of identical bits produces no
    violation at all. This is what the rule tests count against.
    """
    bits = ps2_frame_bits(value)
    return sum(1 for a, b in zip(bits, bits[1:]) if a != b)


def only(chk, rule, limit="min", at_least=2):
    """Assert `rule` is the only rule broken, and hand back its violations.

    `at_least` is 2 for a rule that must fire once per bit of a frame and 1 for one that a
    single frame gives only one chance at (the request-to-send, the bus between two frames).
    """
    assert chk.rules_broken == {rule}, chk.report()
    hits = chk.violations_by_rule(rule)
    assert len(hits) >= at_least, f"{rule} fired {len(hits)} time(s), wanted {at_least}"
    for v in hits:
        assert v.tick > 0, "every violation carries the tick it was missed on"
        assert v.limit == limit
        assert (v.measured < v.required) if limit == "min" else (v.measured > v.required)
        assert not v.ok and v.ns_per_tick == NS_PER_TICK
        assert str(v)                            # __str__ must not raise
    return hits


@pytest.mark.parametrize("dev_timing,limit", [
    ({"clk_low": 10_000}, "min"),             # T3's floor: 30 us
    ({"clk_low": 60_000}, "max"),              # T3's ceiling: 50 us
])
def test_checker_flags_the_clock_low_period_outside_t3(dev_timing, limit):
    """T3 (device->host) and T7 (host->device): the clock's inactive duration is 30..50 us.

    Both ends of the window, and the byte still arrives: only the timing is wrong, which is the
    whole point of a checker that watches the pins and not the payload. Ten of the eleven low
    periods are measured; the last one ends after the frame has closed, and a period a checker
    cannot see is not one it may complain about.
    """
    bus, host, dev, chk = rig("fast", device="bare", dev_timing=dev_timing)
    dev.queue(0x5A)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0x5A] and host.errors == []
    hits = only(chk, "tCLK-low", limit)
    need = chk.min_ticks["clk_low_min"] if limit == "min" else chk.min_ticks["clk_low_max"]
    assert {v.required for v in hits} == {need}
    assert len(hits) == chk.frames_to_host * (FRAME_BITS - 1)
    assert chk.violations_by_rule("tCLK-high") == []


@pytest.mark.parametrize("dev_timing,limit", [
    ({"clk_high": 10_000}, "min"),            # T4's floor: 30 us
    ({"clk_high": 60_000}, "max"),             # T4's ceiling: 50 us
])
def test_checker_flags_the_clock_high_period_outside_t4(dev_timing, limit):
    """T4 and T8: the clock's active duration is 30..50 us.

    Ten measurements again: the eleventh high period of a frame is the one that starts with the
    start bit, and that is `clk_low` long whatever this test sets, so it stays legal.
    """
    bus, host, dev, chk = rig("fast", device="bare", dev_timing=dev_timing)
    dev.queue(0xA5)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0xA5]
    hits = only(chk, "tCLK-high", limit)
    need = chk.min_ticks["clk_high_min"] if limit == "min" else chk.min_ticks["clk_high_max"]
    assert {v.required for v in hits} == {need}
    assert len(hits) == chk.frames_to_host * (FRAME_BITS - 1)
    assert chk.violations_by_rule("tCLK-low") == []


def test_checker_flags_a_device_that_moves_data_too_soon_after_the_rising_edge():
    """T2: DATA may not move sooner than 5 us after the rising edge (device->host).

    A device told to move it 1 us after the rise is outside T2, and -- because the two DATA
    windows are pinned to the same clock period -- outside T1 as well: with the clock high for
    only 30 us there is no position that is early for one rule and legal for the other. Both
    fire, each naming its own number, and the byte still arrives because the host samples on
    the falling edge.
    """
    bus, host, dev, chk = rig("fast", device="bare", dev_timing={"data_after_rise": 1_000})
    dev.queue(0x3C)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0x3C]
    assert chk.rules_broken == {"tSU;DAT", "tHD;DAT"}, chk.report()

    early = chk.violations_by_rule("tSU;DAT")
    assert {v.limit for v in early} == {"min"}
    assert {v.required for v in early} == {chk.min_ticks["data_after_rise_min"]}
    assert {v.measured for v in early} == {ns_to_ticks(NS_PER_TICK, 1_000)}
    assert len(early) == chk.frames_to_host * transitions(0x3C)

    late = chk.violations_by_rule("tHD;DAT")
    assert {v.limit for v in late} == {"max"}
    assert {v.required for v in late} == {chk.min_ticks["data_before_edge_max"]}
    assert {v.measured for v in late} == {ns_to_ticks(NS_PER_TICK, 29_000)}
    assert len(late) == len(early)


def test_checker_flags_data_moved_too_early_for_the_falling_edge():
    """T1's ceiling: DATA moved 30 us before the falling edge is outside the 5..25 us window.

    The only way to reach that without also breaking T2 is to slow the clock down, which is
    what the "slow" rate is for: 50 us of high period with the move 20 us after the rise leaves
    a 30 us gap.
    """
    bus, host, dev, chk = rig("slow", device="bare", dev_timing={"data_after_rise": 20_000})
    dev.queue(0x77)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0x77]
    hits = only(chk, "tHD;DAT", "max")
    assert {v.required for v in hits} == {chk.min_ticks["data_before_edge_max"]}
    assert {v.measured for v in hits} == {ns_to_ticks(NS_PER_TICK, 30_000)}
    assert len(hits) == chk.frames_to_host * transitions(0x77)


def test_checker_flags_data_moved_in_the_clock_low_period():
    """T1 and T2 leave DATA exactly one place, inside the high period.

    `LowPhaseData` moves it in the low period instead, so the transition is neither 5 to 25 us
    before the falling edge nor 5 us after the rising one, and `data-window` says so. The byte
    still arrives: the device puts the next bit down in the same tick it pulls the clock low,
    and the host samples on that edge.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus)
    dev = attach_double(bus, LowPhaseData)
    chk = attach_checker(bus)
    dev.queue(0xC3)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0xC3], "the low-period move still arrives on the sampling edge"
    assert chk.rules_broken == {"data-window"}, chk.report()
    hits = chk.violations_by_rule("data-window")
    assert len(hits) == chk.frames_to_host * transitions(0xC3)
    for v in hits:
        assert v.limit == "flag" and v.tick > 0 and "low period" in v.detail


def test_checker_flags_a_host_that_does_not_hold_the_clock_low_long_enough():
    """tRTS: "Bring the Clock line low for at least 100 microseconds" before the start bit.

    A host told to wait 50 us breaks it, and only it: the request-to-send replaces the 30..50 us
    of T3 for that one low period, so the shorter hold is not also a `tCLK-low` violation. The
    device still reads the byte.
    """
    bus, host, kbd, chk = rig("fast", host_timing={"rts_hold": 50_000})
    host.send(0xF4)
    talk(host, kbd, bus)
    assert [b.value for b in kbd.received] == [0xF4]
    hits = only(chk, "tRTS", "min", at_least=1)
    assert {v.required for v in hits} == {chk.min_ticks["rts_hold"]}
    assert {v.measured for v in hits} == {ns_to_ticks(NS_PER_TICK, 85_000)}
    assert len(hits) == chk.frames_to_device == 1


def test_checker_flags_data_set_too_late_for_the_rising_edge_that_samples_it():
    """tHD;DAT in the host->device direction: DATA has to be stable 5 us before the rise.

    The host is told to hold DATA 28 us into a 30 us low period, so the rising edge that
    samples it comes 2 us later. Nothing else breaks: the low period is still 30 us.
    """
    bus, host, kbd, chk = rig("fast", host_timing={"data_before_edge_min": 28_000})
    host.send(0xF4)
    talk(host, kbd, bus)
    assert [b.value for b in kbd.received] == [0xF4], "the device still reads the byte"
    hits = only(chk, "tHD;DAT", "min")
    assert {v.required for v in hits} == {chk.min_ticks["data_before_edge_min"]}
    assert {v.measured for v in hits} == {ns_to_ticks(NS_PER_TICK, 2_000)}
    assert len(hits) == chk.frames_to_device * transitions(0xF4)


def test_checker_flags_a_host_that_moves_data_inside_the_clock_high_period():
    """tSU;DAT in the host->device direction: "Data must be stable within 1 microsecond after
    the rising edge of the clock line."

    `LateDataHost` puts each bit down 50 us after the rising edge instead of during the low
    period before it. That is far outside the "stable within 1 microsecond after the rising
    edge" the host->device table asks for, and it is late enough that a bit whose value differs
    from its neighbour's is not yet on the wire when the device samples it, so the frame goes
    out malformed as well. Both the framing rules and `tSU;DAT` fire, and this test is about the
    timing one: it is the only rule with a *maximum* in the host->device direction, and no
    parameter of a well-behaved model can break it.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = LateDataHost(bus=bus, rate="fast")
    kbd = attach_keyboard(bus, rate="fast")
    chk = attach_checker(bus)
    bus.attach(host)
    host.send(0xF4, 0xED, 0x01)
    assert host.run(bus, limit=LIMIT)
    assert kbd.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert "tSU;DAT" in chk.rules_broken, chk.report()
    hits = chk.violations_by_rule("tSU;DAT")
    assert len(hits) >= chk.frames_to_device
    for v in hits:
        assert v.limit == "max" and v.tick > 0
        assert v.required == chk.min_ticks["data_after_rise_max"] == 50   # 1 us at 20 ns
        assert v.measured == LateDataHost.late_ticks
    assert kbd.errors, "and the device really did read the bytes wrong"
    assert kbd.received[0].framing_error or kbd.received[0].parity_error


@pytest.mark.parametrize("wait_ns,broken", [(0, True), (10_000, True), (50_000, False)])
def test_checker_flags_a_device_that_does_not_leave_the_bus_free(wait_ns, broken):
    """tBUS-FREE: "After sending stop bit, the keyboard waits for at least 50us".

    The wait is measured from the rising edge that began the clock carrying the last bit,
    because no edge follows that clock: that rise is the closest the wire gets to "this frame
    stopped being clocked". A device with no wait at all measures one clock period -- 30 us --
    plus the handful of ticks it takes to notice the bus is free, which is under 50 us, and so
    does one that waits 10 us. The 50 us the default models use does not.
    """
    bus, host, dev, chk = rig("fast", device="bare", dev_timing={"bus_free": wait_ns})
    dev.queue(0x12, 0x34)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0x12, 0x34], "the bytes are the same either way"
    if not broken:
        ok(chk)
        assert chk.frames_to_host == 2
        return
    hits = only(chk, "tBUS-FREE", "min", at_least=1)
    assert {v.required for v in hits} == {chk.min_ticks["bus_free_min"]}
    assert all(v.measured < v.required for v in hits)
    assert len(hits) == chk.frames_to_host - 1, "only the frame in between is measured"


class ExtraClock(Ps2Device):
    """Clocks one bit more than the frame has -- a twelfth falling edge to the host -- and then
    leaves the bus alone."""

    def _high(self):
        if self.cnt >= self.clk_high:
            if self.bit >= FRAME_BITS:
                self.phase, self.cnt = "idle", 0
                self.clk_out, self.data_out = None, None
                self.idle_left = self.bus_free
                self.tx.popleft()
                return
            self.bit += 1
            self.phase, self.cnt = "low", 0
            self.clk_out = 0
        else:
            super()._high()


def run_bad_frame(bus, cls, value=0xA5, **attrs):
    """Attach `cls` as the device, set `attrs` on it, queue one byte and run the frame out."""
    host = attach_host(bus)
    dev = attach_double(bus, cls)
    for key, val in attrs.items():
        assert hasattr(dev, key), f"{cls.__name__} has no attribute {key!r}"
        setattr(dev, key, val)
    chk = attach_checker(bus)
    dev.queue(value)
    assert dev.run(bus, limit=LIMIT), "the device did not finish its frame"
    bus.run(2_000)
    return host, dev, chk


@pytest.mark.parametrize("fault,rule", [("stop", "stop-bit"), ("parity", "parity")])
def test_checker_flags_a_flipped_frame_bit(fault, rule):
    """`stop-bit` and `parity`, one device each, both from the same one-bit-flip double.

    They are the frame format of the AT Technical Reference: "Stop Bit (always 1)" and an odd
    number of 1s over the eight data bits and the parity bit. The byte still reaches the host,
    flags and all -- the checker watches the wire and the payload is none of its business.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host, dev, chk = run_bad_frame(bus, BadFrame, fault=fault)

    assert host.values == [0xA5]
    frame = chk.frames[0]
    assert frame.bits != ps2_frame_bits(0xA5), "the fault really is on the wire"
    assert chk.rules_broken == {rule}, chk.report()
    hits = chk.violations_by_rule(rule)
    assert len(hits) == 1 and hits[0].limit == "flag" and hits[0].tick == frame.end_tick
    assert str(frame) in hits[0].detail
    assert (frame.bits[10] == 0) if rule == "stop-bit" else frame.parity_error
    assert frame.framing_error == (rule == "stop-bit")


def test_checker_flags_a_start_bit_that_goes_away_again():
    """`start-bit`: "Start Bit (always 0)".

    `StartGlitch` pulls DATA down on an idle-high clock and lets go of it again before the
    first clock edge, so the frame starts and the first bit sampled is a 1. A flipped start bit
    in `frame_bits()` would not show this: DATA would never fall, the frame would never start,
    and the checker would have nothing to decode. Which is why the fault is injected on the
    wire and not in the byte pattern.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host, dev, chk = run_bad_frame(bus, StartGlitch)

    assert host.values == [0xA5], "the eight data bits are still read correctly"
    frame = chk.frames[0]
    assert frame.bits[0] == 1 and frame.bits[1:] == ps2_frame_bits(0xA5)[1:]
    assert chk.rules_broken == {"start-bit"}, chk.report()
    hit = chk.violations[0]
    assert hit.limit == "flag" and hit.tick == frame.end_tick and str(frame) in hit.detail
    assert host.received[0].framing_error is True


def test_checker_flags_a_device_that_never_acknowledges():
    """`ack-bit`: the twelfth clock exists only so the device can pull DATA low for it.

    `NoAck` clocks the host's twelve bits and leaves DATA high, so the checker sees the frame
    with its last bit high. It is the only rule that breaks, and the host counts the miss.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus, retries=0)
    dev = attach_double(bus, NoAck)
    chk = attach_checker(bus)
    host.send(0x55)
    assert host.run(bus, limit=LIMIT)
    assert dev.run(bus, limit=LIMIT)
    bus.run(2_000)

    assert host.acks == 0 and host.nacks == 1
    assert [b.value for b in dev.received] == [0x55], "the device read the byte fine"
    assert dev.acks == 0, "and never drove the twelfth bit"
    assert chk.rules_broken == {"ack-bit"}, chk.report()
    assert chk.frames[0].ack is False and chk.frames[0].bits[-1] == 1
    assert len(chk.violations) == 1 and chk.violations[0].tick > 0
    assert host.errors and "never acknowledged" in host.errors[0]


def test_checker_flags_a_frame_that_stops_in_the_middle():
    """`frame-length`: a frame that begins has to finish.

    `ShortFrame` clocks eight bits and then leaves the bus idle, so the frame the checker
    started is never closed by a bit count. Nothing on the wire says "wrong" at any single edge
    -- it is the frame that never arrives that is the fault, which is why this rule needs a
    timeout as well as a bit count. It also breaks T2: letting go of DATA in the middle of a
    clock high period is a DATA transition 0 us after the rising edge, and both rules are
    really the same fault seen twice.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus)
    dev = attach_double(bus, ShortFrame)
    chk = attach_checker(bus)
    dev.queue(0x3C)
    bus.run(3 * chk.frame_timeout + 4 * frame_ticks(NS_PER_TICK))

    assert chk.rules_broken == {"frame-length", "tSU;DAT"}, chk.report()
    assert chk.frames == [], "an unfinished frame is not decoded as a frame"
    hit = chk.violations_by_rule("frame-length")[0]
    assert hit.limit == "flag" and hit.tick > chk.frame_timeout
    assert "never finished" in hit.detail and "8 of 11" in hit.detail
    t2 = chk.violations_by_rule("tSU;DAT")[0]
    assert t2.limit == "min" and t2.measured == 0 and t2.required == 250
    assert host.values == [], "a frame that stops early cannot be decoded"
    assert host.errors and "stalled after 8 of 11 bits" in host.errors[0], host.errors


def test_checker_flags_a_device_that_clocks_one_bit_too_many():
    """The other half of `frame-length`: a twelfth clock where the frame has eleven.

    `ExtraClock` keeps clocking after the stop bit. The frame itself is clean -- eleven bits, odd
    parity, stop bit high -- so the eleven clocks decode as a legal byte and the twelfth edge is
    all that is left of the fault.

    The checker cannot count that edge as a bit: the frame closed on the eleventh, and a falling
    edge with DATA high and no frame open is the shape of the host's request-to-send. The
    request-to-send turns out to have no start bit, so it is read as an inhibit and dropped, and
    what is left to say is the timing: the bus came back one clock period after the last bit,
    inside the 50 us a device has to leave it. That is `tBUS-FREE`, and its detail says a device
    that clocks more bits than its frame has lands there too.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus)
    dev = attach_double(bus, ExtraClock)
    chk = attach_checker(bus)
    dev.queue(0x3C)
    bus.run(3 * chk.frame_timeout)

    assert chk.rules_broken == {"tBUS-FREE"}, chk.report()
    assert chk.frames[0].bits == ps2_frame_bits(0x3C)
    assert not chk.frames[0].parity_error and not chk.frames[0].framing_error
    assert chk.frames[0].ack is None and chk.frames_to_device == 0
    hits = chk.violations_by_rule("tBUS-FREE")
    assert len(hits) == 1 and hits[0].limit == "min"
    assert hits[0].measured == chk.min_ticks["clk_high_min"]
    assert "clocks more bits than its frame has" in hits[0].detail


def test_the_device_does_not_transmit_while_the_host_holds_the_clock_low():
    """The rule the checker cannot see: "if the line is inactive, output from the device is not
    allowed."

    The host inhibits for T5's 50 us window with a byte queued at the device, and nothing at
    all appears on the wire: the device counts the ticks it spent waiting in `inhibited` and the
    checker sees an idle bus.
    """
    bus, host, kbd, chk = rig("fast")
    kbd.queue(0x1C)
    window = host.min_ticks["inhibit_window"]
    host.inhibit()
    bus.run(window + 4)
    assert host.inhibit_left == 0 and kbd.inhibited > 0
    assert kbd.pending == 1 and kbd.sent == []
    assert host.values == [] and bus.errors == []
    ok(chk)
    assert kbd.frames_sent == 0

    # and once the host lets go, the byte goes out on its own
    assert kbd.run(bus, limit=LIMIT)
    bus.run(2_000)
    assert host.values == [0x1C] and kbd.frames_sent == 1
    ok(chk)


# ---- the spec table itself, and the guard rails ---------------------------------------------
def test_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting. These are the tables of
    `research/ref/protocols/ps2/`, in whole ticks at 20 ns per tick.

    The IBM PC/AT Technical Reference gives both clock halves as 30..50 us (T3/T4 for
    device->host, T7/T8 for host->device), DATA 5 us after the rising edge and 5..25 us before
    the falling one (T2, T1), and DATA stable within 1 us of the rising edge in the other
    direction. The Gdansk page gives the 100 us request-to-send and the 50 us the keyboard waits
    after its stop bit.
    """
    fast = rate_minimums(NS_PER_TICK, "fast")
    assert (fast["clk_low_min"], fast["clk_low_max"]) == (1500, 2500)     # 30 / 50 us
    assert (fast["clk_high_min"], fast["clk_high_max"]) == (1500, 2500)
    assert fast["data_after_rise_min"] == 250                             # T2: 5 us
    assert (fast["data_before_edge_min"], fast["data_before_edge_max"]) == (250, 1250)   # T1
    assert fast["data_after_rise_max"] == 50                             # 1 us
    assert fast["rts_hold"] == 5000                                      # 100 us
    assert fast["bus_free_min"] == fast["bus_free"] == 2500              # 50 us
    assert fast["inhibit_window"] == 2500                                # T5's 50 us
    assert fast["min_period"] == 3000 and fast["max_period"] == 5000     # 60 / 100 us
    assert (fast["clk_low"], fast["clk_high"]) == (1500, 1500)           # 16.67 kbit/s
    assert fast["mode"] == "fast" and fast["rate"] == "fast"

    slow = rate_minimums(NS_PER_TICK, "slow")
    assert (slow["clk_low"], slow["clk_high"]) == (2500, 2500)           # 10 kbit/s
    assert slow["clk_low_max"] == 2500, "the slow rate clocks on the table's ceiling"

    assert ns_to_ticks(20.0, 30_000) == 1500 and ns_to_ticks(20.0, 30_001) == 1501
    assert ns_to_ticks(20.0, -5) == 0

    with pytest.raises(ValueError):
        rate_minimums(NS_PER_TICK, "quick")
    with pytest.raises(ValueError):
        rate_minimums(NS_PER_TICK, "fast", {"t_nonsense": 1})
    with pytest.raises(ValueError):
        rate_minimums(NS_PER_TICK, "fast", {"clk_low": -1})
    with pytest.raises(ValueError):
        Ps2Host(rate="quick")
    with pytest.raises(ValueError):
        Ps2Device(rate="quick")
    with pytest.raises(ValueError):
        Ps2Checker(rate="quick")


def test_parity_and_frame_helpers():
    """The two functions the whole protocol rests on: odd parity and the bit order."""
    assert ps2_parity_bit(0x00) == 1 and ps2_parity_bit(0xFF) == 1     # 0 and 8 ones
    assert ps2_parity_bit(0x01) == 0 and ps2_parity_bit(0x02) == 0     # 1 and 1 ones
    assert ps2_frame_bits(0xA5) == (0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1)
    assert ps2_frame_bits(0x00) == (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1)
    for value in (0x00, 0x01, 0x55, 0xAA, 0xA5, 0xFF):
        bits = ps2_frame_bits(value)
        assert len(bits) == FRAME_BITS and bits[0] == 0 and bits[10] == 1
        assert bits[1:9] == tuple((value >> i) & 1 for i in range(8))
        assert sum(bits[1:10]) % 2 == 1, "eight data bits plus parity is odd"


def test_every_model_takes_its_timing_from_the_bus_it_is_attached_to():
    """`bind()` is how a model built without a bus learns the tick rate.

    The whole bench runs at 100 MHz here, so the slow rate's 50 us half periods are 5000 ticks
    instead of the 2500 they are at 50 MHz, and the checker measures them at that size. A model
    that kept its construction-time tick rate would be twice as fast as the bus and the checker
    would say so.
    """
    bus = Bus(ns_per_tick=10.0)
    bus.add_net("clk")
    bus.add_net("data")
    kbd = Ps2Keyboard(rate="slow")            # no bus yet: 20 ns/tick, the default
    host = Ps2Host(rate="slow")
    chk = Ps2Checker(rate="slow")
    for dev in (host, kbd, chk):
        bus.attach(dev)
    assert kbd.clk_low == 2500, "still on its construction-time tick rate"
    for dev in (host, kbd, chk):
        dev.bind(bus)
    assert kbd.clk_low == 5000 and kbd.bus_free == 5000        # 50 us at 10 ns/tick
    assert (host.clk_low, chk.min_ticks["clk_low_max"]) == (5000, 5000)

    host.send(0xF4)
    talk(host, kbd, bus)
    assert host.values == [ACK] and kbd.enabled
    ok(chk)
    assert chk.ns_per_tick == 10.0 and kbd.ns_per_tick == 10.0
    assert chk.shortest_period == 10_000, "50 us + 50 us at 10 ns/tick"
    assert chk.fastest_khz == pytest.approx(10.0), "10 kcycle/s is 10 kbit/s here"
    assert bus.errors == []


def test_a_stalled_device_frame_is_an_error_not_a_hang():
    """A frame that stops mid-way must be given up on rather than waited for.

    Both receivers have a deadline (`frame_timeout`, three frames' worth of ticks). Without one a
    device that stops clocking would leave the host waiting for the rest of a byte forever, which
    in a test is a hang and not a failure.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = attach_host(bus)
    dev = attach_double(bus, ShortFrame)
    chk = attach_checker(bus)
    dev.queue(0x3C)
    bus.run(3 * host.frame_timeout)
    assert host.errors and "stalled after 8 of 11 bits" in host.errors[0], host.errors
    assert host.received == [] and host.rx_active is False
    assert "frame-length" in chk.rules_broken, chk.report()


def test_a_host_that_never_answers_ends_up_in_errors():
    """The host's other deadline: a frame that begins on the bus and never gets its twelfth bit.

    `ShortDataHost` puts the start bit down and stops there, so the host has started a frame and
    the device has nothing to answer.
    """
    class ShortDataHost(Ps2Host):
        """Puts the request-to-send and the start bit on the wire and then stops."""

        def _step_phase(self, t, pins, phase):
            if phase == "low" and self.bit >= 3 and self.cnt >= self.clk_low:
                self.phase, self.cnt = "idle", 0
                self.clk_out, self.data_out = None, None
                self.queue.popleft()
                return
            super()._step_phase(t, pins, phase)

    bus = Bus(ns_per_tick=NS_PER_TICK)
    host = ShortDataHost(bus=bus)
    kbd = attach_keyboard(bus)
    chk = attach_checker(bus)
    bus.attach(host)
    host.send(0xF4)
    bus.run(3 * kbd.frame_timeout)
    assert kbd.errors and "host frame" in kbd.errors[0], kbd.errors
    assert "stalled" in kbd.errors[0] and "of 12 bits" in kbd.errors[0]
    assert kbd.received == [] and kbd.rx_active is False
    assert host.received == [] and host.nacks == 0
    assert chk.rules_broken, chk.report()


def test_the_wire_can_be_dumped_for_a_waveform(tmp_path):
    """A PS/2 run goes through the core's VCD writer like any other, which is how a firmware
    author looks at the clock it is bit-banging."""
    bus = Bus(ns_per_tick=NS_PER_TICK, trace=True)
    host = attach_host(bus)
    kbd = attach_keyboard(bus)
    chk = attach_checker(bus)
    host.send(0xF4)
    talk(host, kbd, bus)
    path = tmp_path / "ps2.vcd"
    bus.write_vcd(str(path))
    text = path.read_text()
    assert "$timescale 20ns $end" in text
    assert "wire 1 " in text and "clk" in text and "data" in text
    assert text.count("\n") > 100, "a frame is thousands of edges"
    assert len(bus.trace) > 50_000

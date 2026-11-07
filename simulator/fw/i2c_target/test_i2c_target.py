"""End-to-end check of fw/i2c_target on the ISS: program, driver, chip and bus.

The judge is not the driver, it is `verif.vip.i2c`. `I2cController` is the master and
`I2cTimingChecker` decodes the traffic and checks every minimum of the chosen mode;
neither is edited and neither is softened, and a violation fails the test. `I2cEepromTarget`
plays the *same* 24C02 the firmware emulates, and `test_the_firmware_agrees_with_the_vip_eeprom`
makes the two memory images compare equal, so the thing being emulated is checked too.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`; the two I2C lines are wired straight to `p0` and `p1`.
Both nets are open drain and pulled up, and every device drives 0 or releases, so a PIO
that ever drove a hard 1 would show up as a bus that never rises.

`FW_TARGET=iss` runs it against `iss.pio_iss.Chip`; `FW_TARGET=iss_b` against the second,
independently written model; `FW_TARGET=rtl` against `src/pio_chip.sv` under cocotb, one
core cycle per tick (`fw/rtl_harness.py`). Nothing here knows which: the target is
swapped once per session by `fw/conftest.py`.
"""
from __future__ import annotations

import collections
from typing import Optional

import pytest

from fw.i2c_target import (I2cTargetDriver, I2cTargetError, START_SETUP, ack_word,
                           assemble_program, clkdiv_max, data_word, is_stop_marker,
                           poll_for, rx_byte, set_poll, start_window_ticks)
from fw.i2c_target import driver as drv
from iss import pio_iss
from verif.vip.core import Bus, Device
from verif.vip.i2c import (I2C_MODES, I2cController, I2cEepromTarget, I2cTimingChecker,
                           mode_minimums, ns_to_ticks)

TICK_NS = 20.0                    # 50 MHz, the clock the timing minimums are quoted at
DEFAULT_PINS = (drv.DEFAULT_SDA_PIN, drv.DEFAULT_SCL_PIN)

#: Fast mode's tHD;STA: how long a master must hold SDA low after the SCL fall of a START.
#: 4 / 0.6 / 0.26 us for standard / fast / fast-plus, so the fast column is 600 ns
#: (research/ideation/round2/research/sources/C12/ti-snls650-i2c-timing-minima.md:24).
#: `I2C_MODES["fast"].t_hd_sta` is 260 ns, which is the *fast-mode-plus* column of that same
#: table -- see `test_the_masters_start_hold_meets_the_fast_mode_minimum`.
SPEC_FAST_THD_STA_NS = 600.0
SPEC_FAST_THD_STA_TICKS = ns_to_ticks(TICK_NS, SPEC_FAST_THD_STA_NS)   # 30 at 20 ns a tick

#: The recorded sweep: the SCL rates the
#: firmware is measured at, and whether every transaction passes with zero checker
#: violations. `SWEEP_PARAMS` below turns every `clean=False` row into a parameter that
#: carries `xfail(strict=True)` *as a mark*, so the guard is pytest's and it bites: a rate
#: that starts passing is XPASS(strict), which fails the run and forces this table,
#: `XFAIL` and the README to be updated. See `measure.py` for how each row is produced.
SWEEP = [
    (50_000, True),
    (100_000, True),
    (200_000, True),
    (300_000, True),
    (400_000, True),
    (600_000, True),
    (800_000, False),
    (1_000_000, False),
]
#: Why each expected failure fails, measured with `measure.py`. Each string is the *symptom* shown, not
#: a mechanism: a `violation` one is the firmware putting a bit on the wire outside its timing
#: budget, and an `FDEBUG` one is the engine stalling on an empty FIFO. There is deliberately
#: no `nack` entry left: the 200 kHz NACK was a missed START (the divider let the engine's
#: look at SDA be coarser than the START window), fixed in `clkdiv_max` and guarded by
#: `test_the_start_window_is_longer_than_one_engine_look`.
XFAIL = {
    800_000: "FDEBUG 0x01000000 on all three transactions: the engine stalls on an empty TX "
             "FIFO, the host cannot keep the read pipeline fed that fast (the bytes still "
             "come back correct)",
    1_000_000: "tSU;DAT 3 ticks < 5, and it is the *master's* move: the bus-wide tightest "
               "tSU;DAT sample belongs to i2c_ctrl, which holds SDA low for "
               "first_hold = _hold_point(t_hd_sta) = 21 ticks into a 25-tick low period and "
               "then releases it 3 ticks before its own rise, against the 5 Fm+ asks for. "
               "The PIO's own tSU;DAT at this rate is 14 vs 5, and a bus with no slave on "
               "it at all shows the same violation (one_mhz_owner.py), so the row still fails but no "
               "firmware could pass it with this master at 1 MHz",
}
XFAIL_REASON = "see fw/i2c_target/README.md `### Rates`"

#: The rates in `SWEEP` driven off `I2C_MODES`' own fast row, i.e. the rows where the model's
#: `t_hd_sta` constant is the only one in play. Above 400 kHz `measure.FMP` overrides
#: `t_hd_sta` to 400 ns on purpose, so those rows say nothing about the constant.
FAST_MODE_RATES = [rate for rate, _ in SWEEP
                   if I2C_MODES["standard"].freq < rate <= I2C_MODES["fast"].freq]

#: `SWEEP` as pytest parameters: the id is the rate in kHz, and every row recorded as
#: failing carries a *strict* xfail mark whose reason is the measured one from `XFAIL`.
#: Strict is the whole point: with it, a rate that starts passing is XPASS -> a failure.
SWEEP_PARAMS = [
    pytest.param(
        rate_hz, clean, id=f"{rate_hz // 1000}k",
        marks=() if clean else (pytest.mark.xfail(
            strict=True, reason=f"{XFAIL_REASON} ({XFAIL[rate_hz]})"),))
    for rate_hz, clean in SWEEP
]


class MarginProbe(Device):
    """Passive bus monitor: the smallest interval each rule ever got, in core cycles, and
    *who* moved the line for it.

    `I2cTimingChecker` records only violations, so a clean run says nothing about how close
    it came; this says how close. It drives nothing and reads the resolved pin values, so
    it measures the wire rather than the models -- the same thing the checker does, minus
    the pass/fail. Rules measured, each the way the checker takes it:

    * `tLOW`, `tHIGH` -- the SCL period itself, each measured between the two edges that
      bound it: `tLOW` is fall to the next rise, `tHIGH` is rise to the next fall. That is
      where `I2cTimingChecker` takes them (`_fall` and `_rise`, `verif/vip/i2c.py`), so the
      label and the interval cannot come apart.
    * `tSU;DAT` -- the last SDA move before the SCL rise, attributed to whoever made it.
    * `tHD;DAT` -- **the SCL fall to the next SDA move**, i.e. the hold the checker arms on
      a falling edge and tests at the next data bit. This is the point that matters
      ("the margin at the tightest point: SDA valid after SCL falling"), so it is the
      orientation that matters and `tightest` judges it.
    * `SDA→fall` -- not a rule: how long SDA had *already* been stable when SCL fell, which
      is the master's bit placement rather than a minimum anything can fail. It is
      reported (it is what the divider is sized against) and never judged.

    A START's hold is lumped into `tHD;DAT` here. It cannot matter: `t_hd_sta` is larger
    than `t_hd_dat` in every mode the sweep drives (`I2C_MODES` and `measure.FMP`), and a
    probe that keeps the *minimum* only ever sees the smaller of the two.

    **Two minima per rule, because the bus-wide one is not the design's.** Every rule that
    hangs off an SDA move is recorded twice: once as the wire saw it (what
    `I2cTimingChecker` tests, and what a busy bus decides) and once as *ours* -- the
    minimum over the samples this probe attributes to the PIO (`own_name`). On this bus the
    master's own bit placement (`I2cController._hold_point`, 16 ticks in standard mode and 2
    in fast, plus one tick of bus resolution) is the wire-wide tHD;DAT minimum at every
    rate, so the bus-wide column says more about `I2cController` than about the firmware.
    Attribution is read-only: `Bus.tap` reports which device drove which net on the tick
    that was just resolved and is called after every other device has ticked, so it cannot
    change what any device sees.
    """

    name = "i2c_margin"

    def __init__(self, scl_net: str, sda_net: str, own_name: str = "chip"):
        self.scl_net, self.sda_net = scl_net, sda_net
        self.own_name = own_name
        self.reset()

    def reset(self) -> None:
        self.prev_scl = self.prev_sda = 1
        self.scl_low_since = self.scl_high_since = 0
        self.sda_since = 0                     # last change of SDA, either direction
        self.sda_owner = "?"                   # whoever made that change
        self.fell_at = None                    # the SCL fall a tHD;DAT sample is armed on
        self.samples: dict[str, int] = {}      # rule -> the wire-wide minimum
        self.own: dict[str, int] = {}          # rule -> the PIO's own minimum
        #: rule -> owner -> [samples, minimum], so a log can say whose move was tightest.
        self.tally: dict[str, dict[str, list[int]]] = {}
        self.rules = ("tLOW", "tHIGH", "tHD;DAT", "tSU;DAT", "SDA→fall")
        #: `Bus.tap`'s records for the last few ticks: {tick: {device: level}} on SDA.
        self.drives_log: collections.deque = collections.deque(maxlen=8)

    # ---- attribution: who moved SDA, read off `Bus.tap` -------------------------------
    def tap(self, t: int, drives: dict, values: dict) -> None:
        """`Bus.tap`: called at the end of every `step()`, after every device has ticked."""
        self.drives_log.append((t, dict(drives.get(self.sda_net, {}))))

    def _drives(self, t: int) -> dict:
        for tick, drives in reversed(self.drives_log):
            if tick == t:
                return drives
        return {}

    @staticmethod
    def _pulling_low(drives: dict) -> dict:
        """Open drain: only a 0 drives, so a 1 (or a missing entry) released the line."""
        return {name: level for name, level in drives.items() if level == 0}

    def owner_of(self, t: int, level: int) -> str:
        """Who made SDA read `level` on the tick that resolved at `t`.

        A pull-down is whoever drives 0 there; a release is whoever *stopped* driving 0,
        which is the difference between the previous tick's pull-downs and this one's. "?"
        means the tap record for that tick is not in the window (it cannot happen on a bus
        whose tap was attached before the first step) or nobody claims the change.
        """
        now = self._pulling_low(self._drives(t))
        if level == 0:
            names = now
        else:
            was = self._pulling_low(self._drives(t - 1))
            names = {name: 0 for name in was if name not in now}
        return ",".join(sorted(names)) or "?"

    def _see(self, rule: str, value: int, owner: str | None = None) -> None:
        """Record one sample of `rule`: the wire-wide minimum, ours, and the tally.

        `owner` is None for the rules that hang off an SCL edge rather than an SDA move
        (tLOW, tHIGH), which no PIO may move at all.
        """
        if value < 0:
            return
        self.samples[rule] = min(self.samples.get(rule, value), value)
        if owner is None:
            return
        row = self.tally.setdefault(rule, {}).setdefault(owner, [0, value])
        row[0] += 1
        row[1] = min(row[1], value)
        if owner == self.own_name:
            self.own[rule] = min(self.own.get(rule, value), value)

    def tick(self, t: int, pins) -> dict:
        scl, sda = pins[self.scl_net], pins[self.sda_net]
        prev_scl, prev_sda = self.prev_scl, self.prev_sda
        self.prev_scl, self.prev_sda = scl, sda
        # SCL first, the way `I2cTimingChecker.tick` does it: a falling edge starts the
        # hold window, so an SDA move in the same tick is a zero-length hold and has to be
        # caught as one. Checking SDA first would score the same interval as long.
        if prev_scl != scl:
            if scl == 0:
                # The rise is the other end of the high period, so the interval that
                # completes here is tHIGH (and `_see` files it under that name); the low
                # period is not over yet, so nothing of it is known until the rise.
                self._see("tHIGH", t - self.scl_high_since)
                self._see("SDA→fall", t - self.sda_since, self.sda_owner)
                self.fell_at = t               # arm the hold sample for the next SDA move
                self.scl_low_since = t
            else:
                self._see("tSU;DAT", t - self.sda_since, self.sda_owner)
                self._see("tLOW", t - self.scl_low_since)
                self.fell_at = None            # no SDA move in the low period: no sample
                self.scl_high_since = t        # the high period starts here
        if prev_sda != sda:
            # `Bus.step()` hands every device the values resolved at the end of the
            # *previous* tick (the two-phase rule), and calls `tap` at the end of this
            # one: a move seen in `tick(t)` was resolved by tick t-1, and the drives the
            # tap recorded for t-1 are the ones that caused it.
            owner = self.owner_of(t - 1, sda)
            if self.fell_at is not None:       # the checker's tHD;DAT: fall -> next move
                self._see("tHD;DAT", t - self.fell_at, owner)
                self.fell_at = None
            self.sda_since = t
            self.sda_owner = owner
        return {}

    #: How the probe's rule names map onto `mode_minimums`' keys. `SDA→fall` has no key
    #: because it is a distance and not a minimum.
    KEYS = {"tLOW": "t_low", "tHIGH": "t_high", "tHD;DAT": "t_hd_dat", "tSU;DAT": "t_su_dat"}

    def set_required(self, required: dict) -> None:
        self.required = {rule: required.get(self.KEYS[rule]) if rule in self.KEYS else None
                         for rule in self.rules}

    def margins(self) -> dict:
        """Per rule: the wire-wide minimum, the PIO's own, the mode's minimum, and the tally.

        `measured` is what `I2cTimingChecker` would fail on; `own` is the same rule over the
        samples the PIO moved the line in, which is the number the margin is about
        and the only one `tightest` judges. `tally` says who owned what, so a log can show
        whose move the wire-wide minimum belonged to instead of leaving it a mystery.
        """
        req = getattr(self, "required", {})
        return {
            rule: {"measured": self.samples.get(rule), "own": self.own.get(rule),
                   "required": req.get(rule),
                   "tally": {name: tuple(row) for name, row in self.tally.get(rule, {}).items()}}
            for rule in self.rules
        }

    #: The rules the firmware's own budget decides, and that `tightest` therefore judges:
    #: the setup before the master's clock edge and the hold after it. tLOW and tHIGH are
    #: the master's own, and `SDA→fall` is a distance with nothing to fail.
    OWN = ("tSU;DAT", "tHD;DAT")

    def tightest(self, scope: str = "own") -> dict:
        """The rule with the least room, as (measured − required) in core cycles.

        `scope="own"` (the default) judges **our own** minimum, not the wire-wide one: the
        bus-wide minimum at every rate belongs to the master's bit placement
        (`I2cController._hold_point` plus one tick of bus resolution), so judging it would
        report the master's hold as this design's. `scope="bus"` judges the wire-wide one,
        which is what `I2cTimingChecker` would fail on; it is reported beside the other's,
        never in place of it.

        A rule the PIO never moved the line for is skipped under `scope="own"` -- it says
        nothing about this design's budget -- and so is a rule with no required minimum
        (None). `measured` of None means "no edge of that kind went past at all".
        """
        field = "own" if scope == "own" else "measured"
        best = {"rule": None, "measured": None, "required": None, "margin": None}
        for rule, m in self.margins().items():
            if rule not in self.OWN:
                continue
            if m[field] is None or m["required"] is None:
                continue
            margin = m[field] - m["required"]
            if best["margin"] is None or margin < best["margin"]:
                best = {"rule": rule, "measured": m[field],
                        "required": m["required"], "margin": margin}
        return best


def attach_probe(bus: Bus, scl_net: str, sda_net: str, own_name: str = "chip") -> MarginProbe:
    """The margin probe on `bus`: as a Device, and as the bus's own observer.

    Two attachments, two jobs. `bus.attach` makes it measure the resolved pin values, the
    way `I2cTimingChecker` does; `bus.tap` makes it able to say who drove SDA on the tick a
    move was resolved on. `tap` drives nothing and is called after every device has ticked,
    so neither attachment can change what any device sees.
    """
    probe = MarginProbe(scl_net, sda_net, own_name=own_name)
    bus.attach(probe)
    bus.tap = probe.tap
    return probe

class Bus2:
    """A bus with the chip, a master, a timing checker and a margin probe on it.

    `mode` and `overrides` go to `I2cController` and `I2cTimingChecker` alike, so both see
    the same bus; `clkdiv` is the divider the firmware runs at. With `firmware=False` the
    VIP's own `I2cEepromTarget` answers instead of the chip, which is how the two 24C02s are
    compared transaction for transaction.
    """

    def __init__(self, mode="fast", overrides=None, clkdiv=14, address=0x50, ee_address=None,
                 sda_pin=DEFAULT_PINS[0], scl_pin=DEFAULT_PINS[1], firmware=True,
                 tick=TICK_NS):
        self.mode, self.tick = mode, tick
        self.overrides = dict(overrides or {})
        self.sda_pin, self.scl_pin = sda_pin, scl_pin
        self.bus = Bus(ns_per_tick=tick)
        for k in range(pio_iss.NUM_PINS):                  # p0..p17, all pulled up
            self.bus.add_net(f"p{k}")
        self.chip = pio_iss.Chip()
        self.device = pio_iss.ChipDevice(self.chip)
        self.bus.attach(self.device)
        # The models look at the bus through the chip's own pin nets, so there is no
        # coupling device in between and no extra tick of latency anywhere.
        self.nets = dict(scl_net=f"p{scl_pin}", sda_net=f"p{sda_pin}")
        self.ctl = I2cController(address=address, mode=mode, bus=self.bus,
                                 timing=self.overrides, **self.nets)
        self.bus.attach(self.ctl)          # the VIPs drive nothing but must be ticked
        self.checker = I2cTimingChecker(mode=mode, bus=self.bus, timing=self.overrides,
                                        **self.nets)
        self.bus.attach(self.checker)
        # The PIO is the chip device: with `firmware=False` there is no engine on the bus,
        # so the probe attributes to the VIP's EEPROM instead and the PIO's own minima come
        # out empty -- which is the honest answer for a bench with no PIO on it.
        self.probe = attach_probe(self.bus, own_name=self.device.name, **self.nets)
        self.probe.set_required(mode_minimums(tick, mode, self.overrides))
        self.driver = None
        if firmware:
            clk = clkdiv or clkdiv_max(self.ctl.low_ticks, self.ctl.first_hold)
            self.driver = I2cTargetDriver(
                self.bus, self.device, assemble_program(), clkdiv=clk,
                poll=poll_for(self.ctl.high_ticks, clk),
                address=address if ee_address is None else ee_address,
                sda_pin=sda_pin, scl_pin=scl_pin)
            self.driver.load()
            self.eeprom = self.driver.eeprom
        else:
            self.eeprom = I2cEepromTarget(address=address if ee_address is None else ee_address,
                                          mode=mode, bus=self.bus, **self.nets)
            self.bus.attach(self.eeprom)

    # ---- what the checker and the chip saw -----------------------------------------
    @property
    def poll(self) -> int:
        return self.driver.poll

    @property
    def errors(self) -> list:
        return self.bus.errors

    def decoded(self) -> list:
        """The bytes on the bus, as (value, ack, is_address, rw) tuples."""
        return [(t.value, t.ack, t.is_address, t.rw) for t in self.checker.transfers]

    def no_violations(self) -> None:
        assert self.checker.violations == [], self.checker.report()

    def run(self, limit: int = 400_000) -> None:
        """Run the master to idle, then let the bus settle for the trailing STOP.

        The FIFO needs a few more cycles than the master uses to raise the STOP, so the
        queue is not empty the moment `ctl.idle` goes true; 200 ticks is far more than the
        handful the host needs and far less than a bit time.
        """
        if self.driver is not None:
            self.driver.serve_until(lambda: self.ctl.idle, limit=limit)
            self.driver.serve(200)
        else:
            self.ctl.run(self.bus, limit=limit)
            self.bus.run(200)


def _write_script(bench: Bus2, address: int, ptr: int, data: bytes) -> None:
    op = bench.ctl.queue_write([ptr, *data])
    bench.run()
    assert op.ok, f"{op.kind} at tick {op.start_tick}: nack={op.nack} error={op.error}"


def _read_script(bench: Bus2, address: int, count: int) -> None:
    op = bench.ctl.queue_read(count)
    bench.run()
    assert op.ok, f"{op.kind} at tick {op.start_tick}: nack={op.nack} error={op.error}"
    assert len(op.data) == count, f"read {len(op.data)} of {count}: {bytes(op.data).hex()}"


def scripts_for(bench: Bus2):
    """The transaction scripts every bench runs, as (name, callable) pairs.

    They touch `bench.ctl` and `bench.eeprom.memory` only, so the same three run against
    the firmware and against the VIP's EEPROM.
    """
    address = bench.ctl.address
    return [
        ("page_write", lambda b: _write_script(b, address, 0x06, (0xC0, 0xC1, 0xC2))),
        ("sequential_read", lambda b: _read_script(b, address, 5)),
        ("byte_write", lambda b: _write_script(b, address, 0x20, (0x5A,))),
    ]


# ---- the headline case: write, STOP, current-address sequential read, STOP ------------
def test_write_then_current_address_sequential_read():
    bench = Bus2(mode="fast", clkdiv=8)
    eeprom, driver = bench.eeprom, bench.driver
    for i in range(16):
        eeprom.memory[i] = 0xA0 + i

    # Word address 0x06, three data bytes: 0x06, 0x07, then the 8-byte page rolls over to
    # 0x00, so the counter is left at 0x01.
    write = bench.ctl.queue_write([0x06, 0xC0, 0xC1, 0xC2])
    bench.run()
    assert write.ok, write.error or write.nack
    assert [(a, v) for a, v in eeprom.write_log] == [(0x06, 0xC0), (0x07, 0xC1),
                                                     (0x00, 0xC2)], eeprom.write_log
    assert eeprom.ptr == 0x01, eeprom.ptr

    # A STOP, then a fresh START and a current-address read from wherever the counter is.
    read_op = bench.ctl.queue_read(5)
    bench.run()
    read = bytes(read_op.data)
    assert read_op.ok and read == bytes([0xA1, 0xA2, 0xA3, 0xA4, 0xA5]), read.hex()

    bench.no_violations()
    assert (bench.checker.starts, bench.checker.repeated_starts, bench.checker.stops) == (2, 0, 2)
    assert bench.decoded()[:5] == [
        (0xA0, True, True, 0),      # address, write
        (0x06, True, False, None),  # word address
        (0xC0, True, False, None),  # three data bytes
        (0xC1, True, False, None),
        (0xC2, True, False, None),
    ]
    # Every byte is ACKed but the last of the read, which the controller NACKs on purpose
    # to end it -- and which the slave has to leave alone by releasing that ACK clock.
    assert bench.checker.acks == bench.checker.bytes - 1 == 10, (
        f"{bench.checker.acks} ACKs of {bench.checker.bytes} bytes"
    )
    assert driver.fdebug() == 0, f"FDEBUG {driver.fdebug():#010x}"
    # The program pushed one end-of-bus marker per transaction, which is how the host's
    # EEPROM state machine knows to go back to "expect an address byte".
    assert driver.bus_free == 2, driver.bus_free


def test_the_firmware_agrees_with_the_vip_eeprom():
    """The same scripts against `I2cEepromTarget`, and the two images compared.

    The checker judges the timing; this judges the *behaviour*. Without it a firmware that
    ACKed everything and returned a constant would pass every timing rule above.
    """
    ours = Bus2(mode="fast", clkdiv=8)
    theirs = Bus2(mode="fast", clkdiv=8, firmware=False)
    for i in range(16):
        ours.eeprom.memory[i] = 0xA0 + i
        theirs.eeprom.memory[i] = 0xA0 + i
    for _, script in scripts_for(ours):
        script(ours)
    for _, script in scripts_for(theirs):
        script(theirs)
    assert bytes(ours.eeprom.memory) == bytes(theirs.eeprom.memory), (
        f"image {bytes(ours.eeprom.memory[:8]).hex()} != "
        f"{bytes(theirs.eeprom.memory[:8]).hex()}"
    )
    # The checker's decode is the comparable record of what each side put on the wire.
    assert [(t.value, t.ack) for t in ours.checker.transfers] == [
        (t.value, t.ack) for t in theirs.checker.transfers]


def test_an_address_nobody_answers_is_nacked():
    """A 24C02 NACKs an address that is not its own, and so must this one.

    The ACK word is pulled from the host *after* the byte has been sampled, which is what
    makes the decision possible at all; the reference EEPROM NACKs the same address.
    """
    bench = Bus2(mode="fast", clkdiv=8, address=0x51, ee_address=0x50)
    op = bench.ctl.queue_write([0x00, 0x5A])
    bench.run()
    assert bench.driver.bytes_in == 1, "the firmware saw the address byte"
    assert bench.eeprom.state == "addr", "and went back to waiting for an address"
    assert bench.ctl.nacks, "the master saw the NACK"
    assert op.nack and not op.ok and op.acked == 0, (op.nack, op.acked)
    assert bytes(bench.eeprom.memory[:1]) == b"\x00", "nothing was written"
    assert bench.eeprom.nacked == [0xA2], bench.eeprom.nacked
    bench.no_violations()

    # The VIP's own EEPROM refuses the same address, which is the reference for the above.
    ref = Bus2(mode="fast", clkdiv=8, firmware=False, address=0x51, ee_address=0x50)
    ref_op = ref.ctl.queue_write([0x00, 0x5A])
    ref.run()
    assert ref_op.nack, "the VIP's own EEPROM NACKs an address that is not its own"


# ---- the rate sweep, parametrised, with the failing rates marked ---------------------
def _at(rate_hz: float) -> Bus2:
    """The bench the sweep uses at `rate_hz`: see `measure.bus_for`, which the recorded
    table `SWEEP` was produced with."""
    from fw.i2c_target.measure import bus_for
    return bus_for(rate_hz)


def _run_rate(bench: Bus2) -> None:
    """One transaction after another, judged the way `measure.measure()` judges a rate.

    All three of its checks are here and not only in `measure.py`: the scripts complete and
    the checker can be perfectly happy while the *engine* stalled on an empty FIFO halfway
    through (800 kHz, `FDEBUG 0x01000000`), and a sweep row that called that clean would
    disagree with the log the README quotes. It would also be a row the strict xfail mark
    could no longer catch.
    """
    for _, script in scripts_for(bench):
        script(bench)
        bench.no_violations()
        stalled = bench.driver.fdebug()
        assert not stalled, (
            f"FDEBUG {stalled:#010x} after {bench.checker.bytes} bytes: the engine stalled "
            f"on an empty FIFO"
        )
        assert not bench.errors, f"bus contention: {bench.errors}"


@pytest.mark.parametrize("rate_hz,clean", SWEEP_PARAMS)
def test_the_firmware_serves_every_transaction_at_this_rate(rate_hz, clean):
    """Every transaction, at this SCL rate, with zero checker violations.

    The rates that do not make it carry `xfail(strict=True)` *on the parameter*
    (`SWEEP_PARAMS`), so a rate that starts passing falls through this function and comes
    out XPASS(strict) -- a failure of this test -- rather than quietly improving: the table,
    `XFAIL` and the README have to be updated with the measurement. Nothing in here may
    turn a clean run into an xfail (that is what would hide it); the mark's `reason` is the
    measured one from `XFAIL`, and `measure.py` prints the rules and the numbers behind it.
    """
    bench = _at(rate_hz)
    try:
        _run_rate(bench)
    except Exception as exc:                       # noqa: BLE001 - the measurement itself
        if clean:
            raise AssertionError(f"{rate_hz / 1e3:.0f} kHz was clean when recorded: {exc}")
        raise                     # the xfail mark on this row expects exactly this
    # A clean run of a `clean=False` row reaches here and is reported as XPASS(strict).
    assert bench.checker.max_scl_hz <= float(bench.ctl.min["freq"]), (
        f"{bench.checker.max_scl_hz / 1e3:.1f} kbit/s is over the {bench.mode} maximum "
        f"this bench was configured with"
    )


#: The tightest rule at each clean rate, as (rule, the PIO's own minimum, the required
#: minimum), measured with `measure.py --dry-run`. `hold_owner.py` is the attribution
#: cross-check that says the minimum is the firmware's and not the master's.
#:
#: These are the PIO's *own* samples (`MarginProbe.own`), which is the number that the
#: "margin in core cycles at the tightest point" is about: the wire-wide minimum of an
#: SDA-move rule belongs to `i2c_ctrl`'s own bit placement (`I2cController._hold_point`,
#: 16 ticks in standard mode and 2 in fast, plus one tick of bus resolution) and is
#: reported beside it, never in place of it. Asserting the numbers, not only
#: `margin >= 0`, is what stops a silent regression of the probe from keeping the suite
#: green -- the other orientation also has a non-negative margin, which is how the
#: bus-wide number survived as the headline for two rounds.
TIGHTEST = {
    50_000: ("tHD;DAT", 94, 15),
    100_000: ("tHD;DAT", 56, 15),
    200_000: ("tHD;DAT", 21, 0),
    300_000: ("tHD;DAT", 20, 0),
    400_000: ("tHD;DAT", 19, 0),
    600_000: ("tHD;DAT", 9, 0),
}


def test_the_tightest_margin_at_the_recorded_rates():
    """How much room the design has at each rate, and which rule is tightest.

    This is the "margin in core cycles at the tightest point" -- "SDA valid after SCL
    falling", that is tHD;DAT -- measured off the wire
    rather than inferred: `MarginProbe` records the smallest interval each rule ever got,
    each the way the checker takes it, and `tightest()` judges the PIO's *own* minimum of
    each, so the tightest point it reports is the design's. The recorded numbers are in
    `TIGHTEST`, with the log they come from above it; re-record them there when a
    measurement moves.
    """
    seen = {}
    for rate_hz, clean in SWEEP:
        if not clean:
            continue
        bench = _at(rate_hz)
        _run_rate(bench)
        tight = bench.probe.tightest()
        assert tight["margin"] is not None and tight["margin"] >= 0, (
            f"{rate_hz / 1e3:.0f} kHz: {tight}"
        )
        assert tight == {"rule": TIGHTEST[rate_hz][0],
                         "measured": TIGHTEST[rate_hz][1],
                         "required": TIGHTEST[rate_hz][2],
                         "margin": TIGHTEST[rate_hz][1] - TIGHTEST[rate_hz][2]}, (
            f"{rate_hz / 1e3:.0f} kHz: measured {tight}, recorded {TIGHTEST[rate_hz]}; "
            f"re-measure with `measure.py --dry-run` and update TIGHTEST, the README's "
            f"margin table together"
        )
        seen[rate_hz] = (tight, bench.probe.tightest(scope="bus"))
    assert seen, "no rate in SWEEP passes"
    print("\nmargin per rate (core cycles, the PIO's own minimum):")
    for rate_hz, (tight, bus_tight) in sorted(seen.items()):
        print(f"  {rate_hz / 1e3:7.1f} kbit/s: tightest {tight['rule']} "
              f"{tight['measured']} vs {tight['required']} -> {tight['margin']} cycles"
              f"   (wire-wide {bus_tight['rule']} {bus_tight['measured']} vs "
              f"{bus_tight['required']} -> {bus_tight['margin']}: the master's)")
    fastest = max(seen)
    assert fastest == 600_000, (
        f"the fastest clean rate moved to {fastest / 1e3:.0f} kHz; update SWEEP, XFAIL "
        f"and the README"
    )


# ---- the probe itself, on a trace whose periods are known by arithmetic ---------------
def test_each_scl_period_is_measured_under_the_rule_that_bounds_it():
    """`tLOW` is fall-to-rise and `tHIGH` is rise-to-fall, on a hand-made clock.

    Nothing else in this file guards those two rules: `tightest()` judges `tSU;DAT` and
    `tHD;DAT`, `I2cTimingChecker` records only violations, and a *mislabelled* period is
    invisible to both — once, when `tLOW` was holding the high period and was
    printed against `t_low`, so the sweep showed four negative margins for a rule the
    firmware neither owns nor breaks. Here there is no VIP at all: the probe is ticked by
    hand, the periods are 37 and 91 ticks, and the labels have to come out the other way
    round or this fails. Deliberately not 400 kHz's own 80/45, so that a swap of the two
    names cannot hide behind a symmetric clock.
    """
    LOW, HIGH = 37, 91
    IDLE = 300                         # the bus sits idle high first, as it does on a real one
    probe = MarginProbe("scl", "sda")
    probe.set_required(mode_minimums(TICK_NS, "fast", {"freq": 400_000}))
    pins = {"scl": 1, "sda": 1}        # SDA never moves: no owner, and nothing but the
    t = 0                              # two SCL periods is left to measure
    for _ in range(IDLE):
        t += 1
        probe.tick(t, pins)
    for _ in range(3):                 # three periods, so a minimum is a minimum
        for level, ticks in ((0, LOW), (1, HIGH)):
            pins["scl"] = level
            for _ in range(ticks):
                t += 1
                probe.tick(t, pins)
    # The probe starts with `scl_high_since = 0`, so the first fall measures the idle
    # period as tHIGH; it is the oversized sample the minima below have to survive.
    assert probe.samples["tLOW"] == LOW, probe.samples
    assert probe.samples["tHIGH"] == HIGH, probe.samples
    # Neither has an owner: they hang off SCL edges, which no PIO may make.
    for rule in ("tLOW", "tHIGH"):
        row = probe.margins()[rule]
        assert row["own"] is None and row["tally"] == {}, row


# ---- what the program and the driver are made of -------------------------------------
def test_the_program_fits_the_instruction_memory():
    program = assemble_program()
    assert len(program.instrs) <= 32, (
        f"{len(program.instrs)} words, an engine has 32 (docs/mvp-isa.md `### Engine state`)"
    )
    assert program.sideset_bits == 0, (
        "a target must not drive SCL: the program has to have no side-set at all"
    )
    assert program.wrap_target == program.labels["idle"], "the wrap target is idle"
    assert program.wrap == program.labels["idle_end"], "the wrap top is the end"
    assert 0 <= program.wrap_target <= program.wrap


def test_the_driver_addresses_match_the_register_map_and_the_iss():
    """The driver is written against docs/mvp-regmap.md; this is the guard against the two
    drifting apart, in particular against the ISS renumbering something."""
    assert (drv.A_CTRL, drv.A_FSTAT, drv.A_FDEBUG, drv.A_FLEVEL) == (
        pio_iss.A_CTRL, pio_iss.A_FSTAT, pio_iss.A_FDEBUG, pio_iss.A_FLEVEL
    )
    assert (drv.A_TXF0, drv.A_RXF0) == (pio_iss.A_TXF[0], pio_iss.A_RXF[0])
    assert drv.A_IMEM == pio_iss.A_IMEM
    assert (drv.SM_CLKDIV, drv.SM_EXECCTRL, drv.SM_SHIFTCTRL, drv.SM_ADDR,
            drv.SM_INSTR, drv.SM_PINCTRL) == (
        pio_iss.SM_CLKDIV, pio_iss.SM_EXECCTRL, pio_iss.SM_SHIFTCTRL, pio_iss.SM_ADDR,
        pio_iss.SM_INSTR, pio_iss.SM_PINCTRL
    )


def test_the_word_packing_round_trips():
    for byte in (0x00, 0x01, 0x55, 0xA5, 0xFF):
        assert rx_byte(data_word(byte)) == 0, "a receive DATA word is eight releases"
        assert data_word(byte) >> 24 == (~byte & 0xFF), "a send DATA word is inverted"
        assert data_word(byte) & 0xFFFFFF == 0, "nothing outside the eight bit times"
    assert data_word() == 0 and data_word(None) == 0
    assert ack_word(True) == 1 << 31 and ack_word(False) == 0
    for value in (-1, 0x100):
        with pytest.raises(ValueError):
            data_word(value)
    assert is_stop_marker(drv.STOP_MARKER) and not is_stop_marker(0xA0)
    assert not is_stop_marker(data_word(0xFF)), "no byte word can look like the marker"


def test_the_poll_is_a_timing_assumption_and_the_host_computes_it():
    """`poll` has to outlast one master high period; one loop is three SM cycles."""
    assert poll_for(0, 8) == 1, "no high period to outlast"
    assert poll_for(43, 8) == 2, "fast mode at CLKDIV 8: one turn is 27 core cycles"
    assert poll_for(43, 16) == 1, "CLKDIV 16: one turn is 51, already enough"
    for high in (43, 233):
        for clkdiv in (4, 8, 16, 33, 67):
            n = poll_for(high, clkdiv)
            assert 3 * n * (clkdiv + 1) > high, (high, clkdiv, n)
    program = assemble_program()
    assert set_poll(program, 5).instrs != program.instrs, "the patch has to land somewhere"
    for value in (0, 32):
        with pytest.raises(ValueError):
            set_poll(program, value)
    assert len(set_poll(program, 5).instrs) == len(program.instrs)


# ---- the START window: the bug that made 200 kHz fail between two passing rates ---------
def _start_windows(bench: Bus2) -> list[int]:
    """The length in ticks of every START's SDA-low window, measured off the bus.

    A START is SDA falling while SCL reads high, and it exists on the wire only for as long
    as the master holds SDA low before putting its first data bit out
    (`I2cController.first_hold`). The window is measured here rather than computed, because
    this is the quantity the firmware's divider has to respect and the point of the test is
    that the computed number agrees with the wire.
    """
    lengths: list[int] = []
    fell_at: Optional[int] = None
    prev_scl = prev_sda = 1

    def tap(t, drives, values):
        nonlocal fell_at, prev_scl, prev_sda
        scl, sda = values.get(bench.nets["scl_net"], 1), values.get(bench.nets["sda_net"], 1)
        if prev_scl == 1 and scl == 1 and prev_sda == 1 and sda == 0:
            fell_at = t
        if fell_at is not None and sda == 1:
            lengths.append(t - fell_at)
            fell_at = None
        prev_scl, prev_sda = scl, sda

    bench.bus.tap = tap
    try:
        for _, script in scripts_for(bench):
            try:
                script(bench)
            except Exception:                        # noqa: BLE001 - a rate may still fail
                pass
    finally:
        bench.bus.tap = None
    return lengths


def start_holds(bench: Bus2) -> list[dict]:
    """Every START on the bus as the spec's tHD;STA, measured off the wire.

    A START is SDA falling while SCL reads high. What the spec calls tHD;STA is counted from
    the SCL *fall* that ends the START to the SDA *rise* that releases it, which is exactly
    what `I2cTimingChecker` arms on and compares against `min["t_hd_sta"]`
    (`verif/vip/i2c.py`, `hold_since` set at the SCL fall when `start_hold`). Two numbers
    come out of one START and they are not the same: `hold` is the tHD;STA, and `window` is
    the whole SDA-low period, which is one `start_cond` pair longer and is what the divider
    is sized against (`_start_windows`).

    Measured through `Bus.tap`, like `_start_windows`: the bus's own read-only observer,
    which reports the resolved values after every device has ticked and drives nothing.
    A START whose SCL fall never arrives (`scl_fell is None`) is reported with a hold of
    `None` rather than dropped, so a bus that never lowers SCL shows up instead of quietly
    shrinking the list.
    """
    out: list[dict] = []
    state = {"scl": 1, "sda": 1, "in_start": False, "sda_fell": None, "scl_fell": None}

    def tap(t, drives, values):
        scl = values.get(bench.nets["scl_net"], 1)
        sda = values.get(bench.nets["sda_net"], 1)
        prev_scl, prev_sda = state["scl"], state["sda"]
        if prev_sda == 1 and sda == 0 and prev_scl == 1:
            state.update(in_start=True, sda_fell=t, scl_fell=None)
        if state["in_start"] and state["scl_fell"] is None and prev_scl == 1 and scl == 0:
            state["scl_fell"] = t
        if state["in_start"] and prev_sda == 0 and sda == 1:
            fall, scl_fell = state["sda_fell"], state["scl_fell"]
            out.append({"sda_fell": fall, "scl_fell": scl_fell, "sda_rise": t,
                        "hold": None if scl_fell is None else t - scl_fell,
                        "window": t - fall})
            state.update(in_start=False, sda_fell=None, scl_fell=None)
        state["scl"], state["sda"] = scl, sda

    bench.bus.tap = tap
    try:
        for _, script in scripts_for(bench):
            try:
                script(bench)
            except Exception:                        # noqa: BLE001 - still measuring
                pass
    finally:
        bench.bus.tap = None
    return out


@pytest.mark.parametrize("rate_hz", [rate for rate, _ in SWEEP])
def test_the_start_window_is_longer_than_one_engine_look(rate_hz):
    """The engine samples SDA once per `CLKDIV + 1` core cycles; a START shorter than that
    is missed entirely and the transaction it starts comes back NACKed.

    This is the regression guard for the missed-START bug: `idle` recognises a
    START by polling `wait 0 gpio SDA_PIN`, so there is no program-side fix for a START
    narrower than one instruction: the *divider* is what has to fit the window, and
    `clkdiv_max` bounds it by `first_hold`. The 200 kHz row used to be sized by the low
    half-period alone (CLKDIV 22, a 23-core-cycle look) against a 17-tick START window, which
    is why it failed while 100 kHz and 300 kHz passed -- both of those happen to satisfy the
    second bound as well.

    The parameter is every rate in `SWEEP`, clean or not, and carries no mark: this is a
    statement about the divider and the wire, not about whether the rate serves every
    transaction, and a rate can fail at 800 kHz on an empty FIFO while its START windows are
    comfortably long enough. The rows that must serve every transaction are the sweep's own
    `SWEEP_PARAMS`, whose `xfail(strict=True)` marks are what makes the 200 kHz fix show up
    as XPASS rather than as a quietly passing row.
    """
    bench = _at(rate_hz)
    windows = _start_windows(bench)
    assert windows, "no START on the bus at all"
    tightest = min(windows)
    predicted = start_window_ticks(bench.ctl.first_hold)
    assert tightest == predicted, (
        f"the measured START window is {tightest} ticks at {rate_hz / 1e3:.0f} kHz and "
        f"first_hold + START_SETUP says {predicted}: one of the two is wrong, and the "
        f"divider is sized against the latter"
    )
    look = bench.driver.clkdiv + 1
    assert look <= tightest, (
        f"the engine looks at SDA every {look} core cycles at CLKDIV "
        f"{bench.driver.clkdiv}, and the tightest START window at {rate_hz / 1e3:.0f} kHz is "
        f"{tightest} ticks: a START can fall between two looks and be missed, which is what "
        f"made the master's transaction come back NACKed"
    )


#: Why this row is an expected failure: the model constant, not this firmware.
MODEL_THD_STA_XFAIL = (
    "verif/vip/i2c.py:72 gives the fast row t_hd_sta = 260 ns, which is the fast-mode-plus "
    "column of ti-snls650-i2c-timing-minima.md:24 (4 / 0.6 / 0.26 us for standard / fast / "
    f"fast-plus); fast mode specifies {SPEC_FAST_THD_STA_NS:.0f} ns. verif/vip is read-only "
    "here, so the master is reported, not patched. "
    "If this row ever XPASSes, the model was corrected and the report's verdict needs "
    "re-reading -- do not just delete the mark."
)


@pytest.mark.xfail(strict=True, reason=MODEL_THD_STA_XFAIL)
def test_the_masters_start_hold_meets_the_fast_mode_minimum():
    """The master itself must hold SDA long enough after the SCL fall of a START.

    **This is a finding about `verif/vip/i2c.py`, not about this firmware, and it lives here
    because `verif/vip/` is treated as read-only here.** It is the finding
    turned into a test that bites.

    Fast mode specifies tHD;STA = 600 ns (0.6 us), from the project's own saved spec source
    (`research/ideation/round2/research/sources/C12/ti-snls650-i2c-timing-minima.md:24`,
    fetched 2026-10-04, which gives 4 / 0.6 / 0.26 us for standard / fast / fast-plus).
    `verif/vip/i2c.py:72` puts **260 ns** in the fast row -- the *fast-mode-plus* column of
    that same table -- and `I2cController._hold_point` turns it into `first_hold = 14` ticks,
    so `I2cController` holds SDA low for **300 ns** where fast mode asks for 600, on every
    fast-mode rate in the sweep. Measured here off the wire, one SCL fall to the next SDA
    rise per START, at 20 ns a tick.

    Why it matters twice over. `I2cTimingChecker` measures this same hold against this same
    understated constant, so a master that violates fast mode by half raises no violation --
    the two cancel, which is why every `SWEEP` row above 400 kHz reads "0 violations". And
    the short window is what made `fw/i2c_target` fail at 200 kHz: with the spec's 600 ns
    the master's `first_hold` is 31, the START window is 34 ticks, and the pre-fix CLKDIV 22
    (a 23-cycle look) would have caught every START. The firmware's missing START-window
    budget was therefore real but **latent at 200 kHz**, not triggered.

    Strict, so the day the model's constant is corrected this row is an XPASS *failure*: the
    master has become spec-compliant and the report's (a)/(b) split has to be re-read. The
    rates are `FAST_MODE_RATES`, the `SWEEP` rows that run off `I2C_MODES`' own fast row; the
    three above 400 kHz set `t_hd_sta` to 400 ns on purpose (`measure.FMP`) and are not about
    the constant.
    """
    tightest: Optional[int] = None
    measured: list[str] = []
    for rate_hz in FAST_MODE_RATES:
        bench = _at(rate_hz)
        rows = start_holds(bench)
        assert rows, f"no START on the bus at all at {rate_hz / 1e3:.0f} kHz"
        holds = [r["hold"] for r in rows if r["hold"] is not None]
        assert len(holds) == len(rows), (
            f"{rate_hz / 1e3:.0f} kHz: a START never saw its SCL fall: {rows}"
        )
        model_ticks = bench.ctl.min["t_hd_sta"]
        measured.append(
            f"{rate_hz / 1e3:.0f} kHz: {len(rows)} STARTs, tHD;STA = "
            f"{min(holds)}-{max(holds)} ticks ({min(holds) * TICK_NS:.0f}-"
            f"{max(holds) * TICK_NS:.0f} ns), windows "
            f"{min(r['window'] for r in rows)}-{max(r['window'] for r in rows)} ticks, "
            f"I2C_MODES['fast'].t_hd_sta = {model_ticks * TICK_NS:.0f} ns "
            f"({model_ticks} ticks)")
        if tightest is None or min(holds) < tightest:
            tightest = min(holds)
    assert tightest >= SPEC_FAST_THD_STA_TICKS, (
        f"the master holds SDA low for as little as {tightest} ticks "
        f"({tightest * TICK_NS:.0f} ns) after the SCL fall of a START, and fast mode "
        f"specifies {SPEC_FAST_THD_STA_TICKS} ticks ({SPEC_FAST_THD_STA_NS:.0f} ns) of "
        f"tHD;STA -- short by {SPEC_FAST_THD_STA_TICKS - tightest} ticks. Measured: "
        + "; ".join(measured)
    )


def test_clkdiv_max_is_bounded_by_the_start_window_and_not_only_by_the_low_phase():
    """`clkdiv_max` takes the minimum of two bounds, and the second one is not decorative.

    200 kHz is the case that needs it: its low half-period is 142 ticks, which on its own
    would allow CLKDIV 22, while the START window is 17 ticks and allows 16. Standard mode's
    4 us tHD;STA makes the window 204 ticks, so the low phase binds there and 100 kHz keeps
    CLKDIV 43 -- which is why this never showed up as a rate limit.
    """
    assert start_window_ticks(14) == 17, "200 kHz fast: first_hold 14, plus START_SETUP"
    assert start_window_ticks(201) == 204, "standard mode: first_hold 201"
    assert START_SETUP == 3, "two start_cond ticks plus one tick of bus resolution"
    # low phase 142 -> 22 on its own; the START window 17 -> 16; the smaller wins.
    assert clkdiv_max(142, 14) == 16
    assert clkdiv_max(142, 201) == 22, "a wide START window leaves the low phase in charge"
    assert clkdiv_max(101, 14) == 15, "300 kHz: the low phase is already the tighter bound"
    for low in (80, 142, 267, 517):
        for first_hold in (2, 14, 21, 201):
            n = clkdiv_max(low, first_hold)
            assert n == min((low - drv.FIRST_BIT_SLACK) // drv.SM_CYCLES_FIRST_BIT,
                            start_window_ticks(first_hold) - 1), (low, first_hold, n)
            # The property that matters, whichever of the two bounds won: the engine's look
            # at SDA is never coarser than the START window it has to catch.
            assert n + 1 <= start_window_ticks(first_hold), (low, first_hold, n)
            assert n + 1 >= 1


def test_every_start_on_the_bus_is_served_by_the_program():
    """The symptom, counted: one end-of-bus marker per START the checker decoded.

    The program pushes a marker when it recognises the bus going free (`stop_path`), so a
    START it never saw leaves the program still in `idle` and no marker behind for it. At
    200 kHz before the fix the third START fell between two of the engine's looks, the
    program sat out the whole address byte, and this count came to 2 against the checker's
    3 -- which is the same failure as the `nack=True` one transaction downstream, measured
    where it happens.

    Off the wire and off the engine, so it holds on all three targets (`FW_TARGET=iss`,
    `iss_b`, `rtl`) without knowing which one is installed.
    """
    for rate_hz in (50_000, 100_000, 200_000, 300_000, 400_000, 600_000):
        bench = _at(rate_hz)
        for _, script in scripts_for(bench):
            try:
                script(bench)
            except Exception as exc:                # noqa: BLE001 - reported below
                pytest.fail(f"{rate_hz / 1e3:.0f} kHz: {type(exc).__name__}: {exc}")
        assert bench.driver.bus_free == bench.checker.starts == 3, (
            f"at {rate_hz / 1e3:.0f} kHz (CLKDIV {bench.driver.clkdiv}, a "
            f"{bench.driver.clkdiv + 1}-core-cycle look at SDA) the checker decoded "
            f"{bench.checker.starts} STARTs and the program served {bench.driver.bus_free}: "
            f"a START was missed between two looks"
        )


def test_the_pins_have_to_be_a_pair_of_uio_pins():
    """Open drain is a direction, and only uio pins have one (docs/mvp-isa.md
    `### Open drain`)."""
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus.attach(device)
    program = assemble_program()
    for sda, scl in ((0, 8), (8, 0), (0, 13), (0, 18)):
        with pytest.raises(ValueError):
            I2cTargetDriver(bus, device, program, sda_pin=sda, scl_pin=scl)
    for clkdiv in (0, 0x10000):
        with pytest.raises(ValueError):
            I2cTargetDriver(bus, device, program, clkdiv=clkdiv)


def test_a_program_that_drives_scl_is_refused():
    """`check_program` is the guard that keeps the one rule of a target intact."""
    from asm.pioasm import Program
    program = assemble_program()
    with_sideset = Program(
        name=program.name, instrs=list(program.instrs), origin=program.origin,
        wrap_target=program.wrap_target, wrap=program.wrap, sideset_bits=1,
        sideset_opt=False, sideset_pindirs=True, labels=program.labels)
    with pytest.raises(I2cTargetError, match="must not drive SCL"):
        drv.check_program(with_sideset, 0, 1)

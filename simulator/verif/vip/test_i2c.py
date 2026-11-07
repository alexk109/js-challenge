"""Self-tests for the pin-level I2C reference models: `make vip-selftest VIP=i2c`.

Everything runs at 50 MHz (20 ns per tick) in both standard (100 kHz) and fast (400 kHz)
mode, with a controller and a 24C02 talking to each other and a passive checker watching
the pins. `rig()` builds that bench; the tests then look at the EEPROM's memory, at what
the controller read back, and at what the checker says about the wire.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.i2c import (
    I2cController,
    I2cEepromTarget,
    I2cTimingChecker,
    attach_checker,
    attach_controller,
    attach_eeprom,
    mode_minimums,
)

MODES = ["standard", "fast"]          # 100 kHz and 400 kHz
NS_PER_TICK = 20.0                    # 50 MHz chip clock


class SloppyMaster(Device):
    """A deliberately badly timed master, to prove the checker's data rules really bite.

    It pulses SCL 10 ticks low and 10 ticks high and moves SDA in the same tick as the
    SCL fall, never while SCL is high: one tick of data hold and ten ticks of data
    setup, against the 15 and 13 ticks standard mode asks for at 20 ns/tick.
    """

    name = "sloppy"
    low_ticks = 10
    high_ticks = 10

    def __init__(self):
        self.bit = 0
        self.low = True
        self.left = 0
        self.scl = None
        self.sda = 1

    def tick(self, t, pins):
        if self.left <= 0:
            if self.low:                      # SCL goes high, SDA stays put
                self.low, self.left, self.scl = False, self.high_ticks, None
                self.bit += 1
            else:                             # SCL falls and SDA moves with it
                self.low, self.left, self.scl = True, self.low_ticks, 0
                self.sda = self.bit % 2
        self.left -= 1
        return {"scl": self.scl, "sda": self.sda}


def rig(mode="standard", address=0x50, stretch_ticks=0, ctl_timing=None,
        check_mode=None, eeprom_mode=None, check=True, ee_address=None,
        check_timing=None):
    """A bus with an I2cController at `address`, a 24C02 at `ee_address`, and a checker.

    The three timing hooks are how a test drives one part of the bench off-spec without
    touching the others: `ctl_timing` for the master, `check_timing` for the limits the
    monitor judges by, and the two modes for anything that only wants another class.
    """
    bus = Bus(ns_per_tick=NS_PER_TICK)
    ctl = attach_controller(bus, address=address, mode=mode, timing=ctl_timing)
    ee = attach_eeprom(bus, address=ee_address if ee_address is not None else address,
                       mode=eeprom_mode or mode, stretch_ticks=stretch_ticks)
    chk = (attach_checker(bus, mode=check_mode or mode, timing=check_timing)
           if check else None)
    return bus, ctl, ee, chk


def write_bytes(ctl, addr, data):
    """A 24C02 write is control byte, word address byte, then the data."""
    return ctl.queue_write(bytes([addr]) + bytes(data))


def settled(ctl, bus, limit=1_000_000):
    """Run the bus until the controller is idle, and assert it got there."""
    assert ctl.run(bus, limit=limit), "controller did not finish its queue"
    assert ctl.errors == [], ctl.errors
    return ctl


@pytest.mark.parametrize("mode", MODES)
def test_byte_write_then_current_address_read(mode):
    """Byte write, read it back, then a current-address read of a whole page."""
    bus, ctl, ee, chk = rig(mode)
    op = write_bytes(ctl, 0x10, b"\xa5")
    settled(ctl, bus)
    assert op.acked == 3 and not op.nack       # control, word address, data
    assert ee.memory[0x10] == 0xA5
    assert ee.counter == 0x11                   # the pointer moved on by one

    op = ctl.queue_write_read(bytes([0x10]), 1)     # random read: address, then data
    settled(ctl, bus)
    assert bytes(op.data) == b"\xa5", op.data

    op = write_bytes(ctl, 0x10, bytes(range(8)))     # a whole page, pointer rolls back
    settled(ctl, bus)
    assert op.acked == 10 and not op.nack
    assert bytes(ee.memory[0x10:0x18]) == bytes(range(8))
    assert ee.counter == 0x10

    op = ctl.queue_read(8)                     # current-address read from 0x10
    settled(ctl, bus)
    assert bytes(op.data) == bytes(range(8)), op.data
    assert chk.clean, chk.report()


@pytest.mark.parametrize("mode", MODES)
def test_page_write_wraps_inside_the_page(mode):
    """8-byte pages: writing past the end of a page rolls over to its start."""
    bus, ctl, ee, chk = rig(mode)
    op = write_bytes(ctl, 0x06, b"\xde\xad\xbe\xef")     # page 0, addresses 6, 7, 0, 1
    settled(ctl, bus)
    assert op.acked == 6 and not op.nack
    assert bytes(ee.memory[0:2]) == b"\xbe\xef"            # rolled over to the start
    assert bytes(ee.memory[6:8]) == b"\xde\xad"
    assert ee.counter == 0x02                             # last byte was 0x01
    assert ee.memory[2] == 0x00                           # nothing else was touched

    # a whole page from its first address lands exactly once
    op = write_bytes(ctl, 0x10, bytes(range(8)))
    settled(ctl, bus)
    assert bytes(ee.memory[0x10:0x18]) == bytes(range(8))
    assert ee.counter == 0x10                             # rolled back to the page start
    assert chk.clean, chk.report()


@pytest.mark.parametrize("mode", MODES)
def test_random_read_write_then_repeated_start_read(mode):
    """Write-then-repeated-START-read: one transaction, one STOP, read-back verified."""
    bus, ctl, ee, chk = rig(mode)
    op = ctl.queue_write_read(bytes([0x20]) + b"\x11\x22\x33", 3)
    settled(ctl, bus)
    assert op.acked == 6 and not op.nack        # write control+word+3, read control
    assert bytes(ee.memory[0x20:0x23]) == b"\x11\x22\x33"
    # the read part carries on from where the write left the pointer, 0x23
    assert bytes(op.data) == bytes(3), op.data
    assert ee.counter == 0x26

    # a real random read: the word address byte is sent again before the data
    op = ctl.queue_write_read(bytes([0x20]), 3)
    settled(ctl, bus)
    assert bytes(op.data) == b"\x11\x22\x33", op.data

    # the same read split into two transactions instead of one combined one
    op = ctl.queue_write_read(bytes([0xF0]) + b"\x7e", 4)
    settled(ctl, bus)
    assert ee.memory[0xF0] == 0x7E
    assert bytes(op.data) == bytes(4), op.data
    op = ctl.queue_read(2)                     # from 0xF1: 0x7E is behind us
    settled(ctl, bus)
    assert bytes(op.data) == bytes(2), op.data

    # a write then a current-address read starts where the write left the pointer,
    # which is one past the last byte written, not at it
    op = write_bytes(ctl, 0x40, b"\xaa\xbb")
    settled(ctl, bus)
    assert ee.counter == 0x42
    op = ctl.queue_read(2)
    settled(ctl, bus)
    assert bytes(op.data) == bytes(2), op.data
    op = ctl.queue_write_read(bytes([0x40]), 2)    # a random read gets back to 0x40
    settled(ctl, bus)
    assert bytes(op.data) == b"\xaa\xbb", op.data
    assert chk.clean, chk.report()


@pytest.mark.parametrize("mode", MODES)
def test_sequential_read_runs_the_pointer_on(mode):
    """A read of several bytes walks the address counter on, with no page rollover."""
    bus, ctl, ee, chk = rig(mode)
    op = write_bytes(ctl, 0x30, bytes(range(8)))          # one page, 0x30..0x37
    settled(ctl, bus)
    assert op.acked == 10 and not op.nack
    assert ee.counter == 0x30                             # the write rolled over

    op = ctl.queue_read(8)                                # current address 0x30
    settled(ctl, bus)
    assert bytes(op.data) == bytes(range(8)), op.data

    assert ee.counter == 0x38                     # a read runs the pointer on, no wrap

    op = ctl.queue_read(4)                        # on into blank memory
    settled(ctl, bus)
    assert bytes(op.data) == bytes(4), op.data
    assert ee.counter == 0x3C

    op = ctl.queue_write_read(bytes([0x30]), 16)  # one read across the page boundary
    settled(ctl, bus)
    assert bytes(op.data) == bytes(range(8)) + bytes(8), op.data
    assert chk.clean, chk.report()


@pytest.mark.parametrize("mode", MODES)
def test_nack_on_a_wrong_address(mode):
    """A 24C02 only answers its own address, and the master gives up cleanly."""
    bus, ctl, ee, chk = rig(mode, address=0x51, ee_address=0x50)   # nobody home
    op = write_bytes(ctl, 0x00, b"\x99")
    settled(ctl, bus)
    assert op.nack and op.acked == 0
    assert ee.memory == bytes(256), "nothing may be written"
    assert ctl.nacks and ctl.nacks[0][0] > 0
    assert ee.nacked_addresses and ee.nacked_addresses[0][1] == 0xA2

    op = ctl.queue_read(3)
    settled(ctl, bus)
    assert op.nack and op.data == bytearray()
    assert chk.clean, chk.report()             # a NACK is legal, just useless

    # and a bench where the two addresses agree behaves
    bus2, ctl2, ee2, chk2 = rig(mode)
    assert (ctl2.address, ee2.address) == (0x50, 0x50)
    op = write_bytes(ctl2, 0x00, b"\x99")
    settled(ctl2, bus2)
    assert not op.nack and ee2.memory[0] == 0x99
    assert chk2.clean, chk2.report()


@pytest.mark.parametrize("mode", MODES)
def test_clock_stretching_is_honoured(mode):
    """The target holds SCL low after every byte; the master waits for the bus, and
    stretching longer than a half-period makes the transaction take longer."""
    bus, ctl, ee, chk = rig(mode, stretch_ticks=300)
    op = write_bytes(ctl, 0x00, bytes(range(4)))
    settled(ctl, bus)
    assert op.acked == 6 and not op.nack
    assert bytes(ee.memory[0:4]) == bytes(range(4))
    assert ctl.errors == []
    assert chk.clean, chk.report()

    plain_bus, plain_ctl, plain_ee, plain_chk = rig(mode)   # same bus, no stretching
    plain_op = write_bytes(plain_ctl, 0x00, bytes(range(4)))
    settled(plain_ctl, plain_bus)

    stretched_ticks, plain_ticks = (op.stop_tick - op.start_tick,
                                    plain_op.stop_tick - plain_op.start_tick)
    assert stretched_ticks > plain_ticks, "stretching must make the transaction longer"
    assert plain_ee.memory == ee.memory, "stretching must not change what is on the wire"
    assert plain_chk.clean, plain_chk.report()


@pytest.mark.parametrize("mode", MODES)
def test_checker_passes_the_controller_in_both_modes(mode):
    """A mixed run of every transaction shape must leave the checker silent."""
    bus, ctl, ee, chk = rig(mode)
    write_bytes(ctl, 0x00, b"\x01\x02\x03")
    ctl.queue_read(2)
    ctl.queue_write_read(bytes([0x10]) + b"\xaa\xbb", 2)
    write_bytes(ctl, 0x07, b"\xf0\xf1")         # writing past 0x07 wraps inside the page
    ctl.queue_read(3)
    settled(ctl, bus)

    assert bytes(ee.memory[0x01:0x03]) == b"\x02\x03"
    assert ee.memory[0x07] == 0xF0
    assert ee.memory[0x00] == 0xF1             # writing past 0x07 rolled over to it
    assert chk.clean, chk.report()
    assert (chk.starts, chk.repeated_starts, chk.stops) == (6, 1, 5)
    assert bus.errors == [], "open drain: no contention is possible, and none happened"
    assert len(ctl.transfers) == 5 and all(not t[4] for t in ctl.transfers)


def test_checker_flags_a_controller_faster_than_its_mode_allows():
    """The one maximum of the spec, and the only rule no half-period floor can reach.

    A master told to run at 450 kHz still meets every fast-mode minimum -- both halves
    are longer than the floors -- so the minimums say nothing and fSCL says 446 kHz on a
    400 kHz bus.  The same bench at the mode's own rate is silent, and the cap comes from
    the mode: a checker told to allow 500 kHz lets the same wire past.
    """
    fast = mode_minimums(NS_PER_TICK, "fast")
    assert fast["freq"] == 400e3 and fast["min_period"] == 125   # 2.5 us at 20 ns/tick

    bus, ctl, ee, chk = rig("fast", ctl_timing={"freq": 450e3})
    op = write_bytes(ctl, 0x00, b"\xa5")
    settled(ctl, bus)

    assert not op.nack and ee.memory[0] == 0xA5      # it still works, just too fast
    over = chk.violations_by_rule("fSCL")
    assert over, chk.report()
    assert {v.rule for v in chk.violations} == {"fSCL"}, chk.report()
    for v in over:
        assert v.tick > 0                            # every violation carries its tick
        assert v.required == fast["min_period"]
        assert v.measured < v.required
    # 112 ticks is the fastest a 20 ns tick can express 450 kHz, and every bit is it.
    assert chk.shortest_period == 112 and chk.periods == 26
    assert {v.measured for v in over} == {112}
    assert chk.max_scl_hz > fast["freq"], chk.report()

    # ... and the same bench at the mode's own 400 kHz passes. It is a write plus a
    # repeated START and a read, so the START and STOP pulses are on the wire here: they
    # are shorter than 1/fSCL and are not clock cycles, so they are not measured. 45
    # clocks carry data and the first of them has nothing before it to pair with.
    bus2, ctl2, ee2, chk2 = rig("fast")
    op2 = ctl2.queue_write_read(bytes([0x00]), 2)
    settled(ctl2, bus2)
    assert not op2.nack and bytes(op2.data) == bytes(2), op2.data
    assert chk2.violations_by_rule("fSCL") == [], chk2.report()
    assert chk2.periods == 44 and chk2.shortest_period == fast["min_period"]
    assert chk2.max_scl_hz == fast["freq"], chk2.report()
    assert chk2.clean, chk2.report()

    # The limit is the mode's, not a constant in the checker: the same 450 kHz wire is
    # clean against a fast-mode checker told 500 kHz is the ceiling.
    bus3, ctl3, ee3, chk3 = rig("fast", ctl_timing={"freq": 450e3},
                                check_timing={"freq": 500e3})
    op3 = write_bytes(ctl3, 0x00, b"\xa5")
    settled(ctl3, bus3)
    assert not op3.nack and ee3.memory[0] == 0xA5
    assert chk3.min["min_period"] == 100               # 2.0 us at 20 ns/tick
    assert chk3.violations_by_rule("fSCL") == [], chk3.report()
    assert chk3.clean, chk3.report()


def test_checker_flags_an_over_nominal_controller_in_standard_mode_too():
    """Standard mode caps fSCL at 100 kHz, and the floors add up to more than that.

    A master cannot go below the sum of its own half-period minimums, so asking it for
    450 kHz leaves it at 235 + 200 ticks, which is 115 kbit/s -- over the 100 kHz cap and
    caught, even though no minimum was missed.
    """
    standard = mode_minimums(NS_PER_TICK, "standard")
    assert standard["freq"] == 100e3 and standard["min_period"] == 500

    bus, ctl, ee, chk = rig("standard", ctl_timing={"freq": 450e3})
    op = write_bytes(ctl, 0x00, b"\xa5")
    settled(ctl, bus)
    assert not op.nack and ee.memory[0] == 0xA5
    assert chk.shortest_period == standard["t_low"] + standard["t_high"] == 435
    assert chk.violations_by_rule("fSCL"), chk.report()
    assert {v.rule for v in chk.violations} == {"fSCL"}, chk.report()

    # A standard-mode master is exactly at the cap, which is not a violation.
    bus2, ctl2, ee2, chk2 = rig("standard")
    op2 = write_bytes(ctl2, 0x00, b"\xa5")
    settled(ctl2, bus2)
    assert not op2.nack and ee2.memory[0] == 0xA5
    assert chk2.shortest_period == 500 and chk2.max_scl_hz == 100e3
    assert chk2.clean, chk2.report()


def test_checker_flags_a_controller_with_too_short_a_low_period():
    """A controller told to use a 500 ns tLOW runs the bus under the 1.3 us fast-mode
    floor; the checker must say so, with the tick, on every half-period."""
    bus, ctl, ee, chk = rig("fast", ctl_timing={"t_low": 500})
    op = write_bytes(ctl, 0x00, b"\xa5")
    settled(ctl, bus)

    assert not op.nack and ee.memory[0] == 0xA5      # it still works, just too fast
    low = chk.violations_by_rule("tLOW")
    assert low, chk.report()
    assert len(low) > 1
    minimum = mode_minimums(NS_PER_TICK, "fast")["t_low"]
    for v in low:
        assert v.tick > 0                            # every violation carries its tick
        assert v.required == minimum
        assert v.measured < minimum
    assert {v.rule for v in chk.violations} == {"tLOW"}, chk.report()
    assert ctl.low < minimum, "the controller really is below the floor"


def test_checker_is_not_vacuous():
    """The same pin traffic judged against the other mode's limits must fail: fast-mode
    minimums are shorter than standard-mode ones, so this also proves the checker reads
    the minimums from the mode it was given -- and that fSCL is not one of them."""
    bus, ctl, ee, chk = rig("fast", check_mode="standard")
    write_bytes(ctl, 0x00, b"\xa5")
    settled(ctl, bus)
    assert chk.violations, "a 400 kHz bus must not pass a 100 kHz check"
    assert {v.rule for v in chk.violations}.issuperset({"tLOW", "tHIGH", "fSCL"})
    for v in chk.violations:
        assert v.measured < v.required and v.tick > 0


def test_checker_flags_a_sloppy_master_on_the_data_rules():
    """A master that moves SDA in the same tick as the SCL fall, and holds it for only
    10 ticks, must be caught on tHD;DAT and tSU;DAT, not just on the half-periods."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bus.add_net("scl")
    bus.add_net("sda")
    bus.attach(SloppyMaster())
    chk = attach_checker(bus, mode="standard")
    bus.run(400)

    rules = {v.rule for v in chk.violations}
    assert rules.issuperset({"tLOW", "tHIGH", "tHD;DAT", "tSU;DAT"}), chk.report()
    assert all(v.measured < v.required and v.tick > 0 for v in chk.violations)
    assert chk.starts == 0 and chk.stops == 0, "SDA only moved while SCL was low"
    assert mode_minimums(NS_PER_TICK, "standard")["t_su_dat"] > SloppyMaster.low_ticks


def test_absurd_timing_overrides_stay_legal_instead_of_deadlocking():
    """Ask the controller for a data hold time longer than a half-period. It cannot put
    SDA on the SCL edge, so it lengthens the low period: the data still moves in time and
    the checker stays silent."""
    bus, ctl, ee, chk = rig("fast", ctl_timing={"t_hd_dat": 5_000})
    assert ctl.data_hold + ctl.min["t_su_dat"] + 1 <= ctl.low_ticks
    op = write_bytes(ctl, 0x00, b"\xa5")
    settled(ctl, bus)
    assert not op.nack and ee.memory[0] == 0xA5
    assert chk.clean, chk.report()


@pytest.mark.parametrize("mode", MODES)
def test_stop_and_start_conditions_count_correctly(mode):
    """Two plain transactions and one combined write-read are three STOPs and four
    STARTs, one of them a repeated START."""
    bus, ctl, ee, chk = rig(mode)
    write_bytes(ctl, 0x00, b"\x11")               # START ... STOP
    ctl.queue_write_read(bytes([0x20]) + b"\x22\x33", 2)   # START, START, ... STOP
    write_bytes(ctl, 0x40, b"\x44")               # START ... STOP
    settled(ctl, bus)

    assert (chk.starts, chk.repeated_starts, chk.stops) == (4, 1, 3)
    assert len(chk.start_ticks) == 4 and len(chk.stop_ticks) == 3
    # START, STOP | START, repeated START, STOP | START, STOP: in that order on the wire,
    # with the repeated START inside the combined transaction and no STOP in between.
    s, p = chk.start_ticks, chk.stop_ticks
    assert s[0] < p[0] < s[1] < s[2] < p[1] < s[3] < p[2]

    # the wire: control+word+data per write, then control(read)+data for the read
    values = [t.value for t in chk.transfers]
    assert values[:5] == [0xA0, 0x00, 0x11, 0xA0, 0x20]
    addresses = [t for t in chk.transfers if t.is_address]
    assert [a.value for a in addresses] == [0xA0, 0xA0, 0xA1, 0xA0]
    assert [a.rw for a in addresses] == [0, 0, 1, 0]
    assert all(t.ack for t in chk.transfers[:3])
    # the master NACKs the last byte of the one read it does
    assert chk.nacks == 1 and chk.acks == chk.bytes - 1
    assert chk.clean, chk.report()


def test_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting: these are the NXP UM10204 timing table, in ns."""
    standard = mode_minimums(NS_PER_TICK, "standard")
    assert (standard["t_low"], standard["t_high"]) == (235, 200)
    assert (standard["t_su_sta"], standard["t_hd_sta"]) == (235, 200)
    assert (standard["t_su_dat"], standard["t_hd_dat"]) == (13, 15)
    assert (standard["t_su_sto"], standard["t_buf"]) == (235, 235)
    fast = mode_minimums(NS_PER_TICK, "fast")
    assert (fast["t_low"], fast["t_high"]) == (65, 30)
    assert (fast["t_su_sta"], fast["t_hd_sta"]) == (30, 13)
    assert (fast["t_su_dat"], fast["t_hd_dat"]) == (5, 0)
    # fSCL is the maximum of the same table, and min_period is 1/fSCL in whole ticks.
    assert (standard["freq"], standard["min_period"]) == (100e3, 500)
    assert (fast["freq"], fast["min_period"]) == (400e3, 125)
    assert mode_minimums(NS_PER_TICK, "fast", {"freq": 450e3})["min_period"] == 112
    with pytest.raises(ValueError):
        mode_minimums(NS_PER_TICK, "fast-plus-plus")
    with pytest.raises(ValueError):
        mode_minimums(NS_PER_TICK, "fast", {"t_nonsense": 1})
    with pytest.raises(ValueError):
        mode_minimums(NS_PER_TICK, "fast", {"freq": 0})


@pytest.mark.parametrize("mode", MODES)
def test_controller_rejects_a_bad_address_and_times_out_on_a_silent_bus(mode):
    """Guard rails: a 7-bit address is required, and a slave that never releases SCL
    trips the stretch timeout instead of hanging the run."""
    with pytest.raises(ValueError):
        I2cController(address=0x80, mode=mode)
    with pytest.raises(ValueError):
        I2cEepromTarget(address=0x100, mode=mode)

    bus = Bus(ns_per_tick=NS_PER_TICK)
    ctl = attach_controller(bus, mode=mode, address=0x50)
    silent = attach_eeprom(bus, mode=mode, address=0x50, stretch_ticks=10_000)
    ctl.stretch_timeout = 2_000
    op = write_bytes(ctl, 0x00, b"\xa5")
    assert ctl.run(bus, limit=100_000), "the controller must still shut the bus down"
    assert ctl.errors and "SCL" in ctl.errors[0]
    assert op.nack and silent.memory[0] == 0x00
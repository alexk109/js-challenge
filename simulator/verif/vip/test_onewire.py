"""Self-tests for the pin-level 1-Wire reference models: `make vip-selftest VIP=onewire`.

Three groups of tests:

1. the models talk to each other, at standard and at overdrive speed: reset and
   presence, Read ROM, Match ROM, Search ROM, Alarm Search, Convert T with its
   status bit, Read/Write Scratchpad, Copy Scratchpad, Recall E2, Read Power
   Supply, and a multidrop bus with two devices where only one answers;
2. the checker passes all of that legal traffic;
3. `BROKEN` below holds one piece of deliberately broken traffic per rule in
   `onewire.RULES`, and `test_checker_flags_each_spec_rule` proves the checker
   catches every one of them, while `test_checker_is_not_vacuous` proves the same
   wire passes for the wrong reason and not because the checker never complains.

**Tick rate.** The chip runs at 20 ns/tick and every timing here is derived from
`ns_per_tick`, so the tests that measure timing use 20 ns. The long protocol
transactions use 100 ns/tick (10 MHz) because 1-Wire is slow: one time slot is
60..120 us, which is 3000..6000 ticks at 20 ns, and a reset plus a nine-byte
scratchpad read is about 360000 ticks. `test_works_at_the_chip_tick_rate` runs one
full transaction at 20 ns/tick to prove the default is exercised too.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # so `verif.vip` imports

import pytest

from verif.vip.core import Bus, Device
from verif.vip.onewire import (
    ROM_COMMANDS,
    FUNCTION_COMMANDS,
    RULES,
    Ds18b20Target,
    OneWireMaster,
    OneWireTimingChecker,
    attach_checker,
    attach_ds18b20,
    attach_master,
    crc8,
    rom_from_id,
    rom_from_serial,
    speed_minimums,
)

SPEEDS = ["standard", "overdrive"]
NS_PER_TICK = 100.0      # 10 MHz: fast enough for a 750 us transaction
CHIP_NS = 20.0            # the chip's own tick rate (50 MHz)
SERIAL = b"\x11\x22\x33\x44\x55\x66"


def rig(ns_per_tick=NS_PER_TICK, speed="standard", serial=SERIAL, check=True,
        master_timing=None, target_kwargs=None, check_timing=None,
        check_speed=None, targets=1, master=True):
    """A 1-Wire bus with a master, zero or more DS18B20s, and a passive checker.

    `targets=0` is a bus with nobody on it, which is how the presence rule is
    tested. The three timing hooks are how a test breaks one part of the bench
    without touching the others: `master_timing` for what the master drives,
    `target_kwargs` for the slave model, and `check_timing` / `check_speed` for the
    limits the monitor judges by.
    """
    bus = Bus(ns_per_tick=ns_per_tick)
    mst = attach_master(bus, speed=speed, timing=master_timing) if master else None
    devices = [attach_ds18b20(bus, speed=speed, **(target_kwargs or {}))
               for _ in range(targets)]
    if serial is not None and devices:
        devices[0].rom = rom_from_serial(serial)
    chk = attach_checker(bus, speed=check_speed or speed, timing=check_timing) \
        if check else None
    return bus, mst, (devices[0] if devices else None), chk


def run(mst, bus, limit=40_000_000):
    """Run the bus until the master's queue drains, then close the trace."""
    assert mst.run(bus, limit=limit), "the master did not finish its queue"
    assert mst.errors == [], mst.errors
    return mst


def flushed(chk):
    """`finish()` is what judges a trace that ends mid-transaction, so every test
    that asserts on a checker calls it."""
    chk.finish()
    return chk


# ---- broken traffic, one case per rule ------------------------------------------
class SlotWithoutReset(Device):
    """One time slot with no reset pulse in front of it: exactly what the
    `no-reset` rule exists for, and nothing else about it is illegal."""

    name = "no_reset_master"
    low_ticks = 60
    high_ticks = 640

    def __init__(self, drive_low=True):
        self.low = True
        self.left = 0
        self.drive_low = drive_low

    def tick(self, t, pins):
        if self.left <= 0:
            self.low = not self.low
            self.left = self.high_ticks if self.low else self.low_ticks
        self.left -= 1
        return {"dq": 0 if (self.low and self.drive_low) else None}


class ReadZeroDriver(Device):
    """A slave that answers every slot by pulling the line low 25 us in.

    Nothing does that on 1-Wire: a device only drives the bus in a presence pulse
    or in a read time slot, so this is what the `slave-drive` rule catches. It
    answers every falling edge it sees after `start` (which is after the reset and
    the presence pulse, so it leaves the initialisation sequence alone) and never
    inside its own drive.
    """

    name = "bad_slave"
    at = 250          # ticks after the edge it answers, at 100 ns/tick: 25 us
    length = 400
    start = 9_000

    def __init__(self):
        self.line = 1
        self.own_from = None
        self.own_to = 0

    def tick(self, t, pins):
        line = pins["dq"]
        if line != self.line:
            if line == 0 and t >= self.start and not (
                    self.own_from is not None and self.own_from + 1 <= t <= self.own_to):
                self.own_from, self.own_to = t + self.at, t + self.at + self.length
            self.line = line
        driving = self.own_from is not None and self.own_from <= t < self.own_to
        return {"dq": 0 if driving else None}


def _bench(ns_per_tick=NS_PER_TICK, **kwargs):
    bus, mst, ds, chk = rig(ns_per_tick=ns_per_tick, **kwargs)
    return bus, mst, ds, chk


def broken_rstl_short():
    """A 200 us reset: tRSTL is 480 us at the very least.

    The device does not answer a reset that short, so what the checker sees is a
    200 us low, which is too long to be a time slot and too short to be a reset:
    it reports both rules, because on the wire those are the only two readings.
    """
    bus, mst, ds, chk = _bench(master_timing={"w_rstl": 200_000})
    mst.queue_reset()
    run(mst, bus)
    return flushed(chk)


def broken_rstl_long():
    """A 900 us reset: AN126 gives the master 640 us of maximum."""
    bus, mst, ds, chk = _bench(master_timing={"w_rstl": 900_000})
    mst.queue_reset()
    run(mst, bus)
    return flushed(chk)


def broken_no_presence():
    """A reset nobody answers."""
    bus, mst, ds, chk = _bench(targets=0)
    mst.queue_reset()
    run(mst, bus)
    return flushed(chk)


def broken_pdhigh_early():
    """A presence pulse 5 us after the reset: tPDHIGH starts at 15 us."""
    bus, mst, ds, chk = _bench(target_kwargs={"presence_delay": 5_000})
    mst.queue_reset()
    run(mst, bus)
    return flushed(chk)


def broken_pdhigh_late():
    """A presence pulse 100 us after the reset: tPDHIGH ends at 60 us."""
    bus, mst, ds, chk = _bench(target_kwargs={"presence_delay": 100_000})
    mst.queue_reset()
    run(mst, bus)
    return flushed(chk)


def broken_pdlow_short():
    """A 30 us presence pulse: tPDLOW is 60 us at the very least."""
    bus, mst, ds, chk = _bench(target_kwargs={"presence_low": 30_000})
    mst.queue_reset()
    run(mst, bus)
    return flushed(chk)


def broken_pdlow_long():
    """A 300 us presence pulse: tPDLOW ends at 240 us."""
    bus, mst, ds, chk = _bench(target_kwargs={"presence_low": 300_000})
    mst.queue_reset()
    run(mst, bus)
    return flushed(chk)


def broken_rsth():
    """A master that starts its first slot 100 us after the presence pulse instead
    of the 480 us of tRSTH."""
    bus, mst, ds, chk = _bench(master_timing={"w_post_presence": 100_000})
    mst.queue_read_scratchpad()
    run(mst, bus)
    return flushed(chk)


def broken_low1_short():
    """A write-1 low of 0.2 us: tLOW1 is at least 1 us, or the slave samples the
    line before the master has finished pulling it down."""
    bus, mst, ds, chk = _bench(master_timing={"w_low1": 200})
    mst.queue_reset()
    mst.queue_write(b"\xcc")
    run(mst, bus)
    return flushed(chk)


def broken_low1_long():
    """A 30 us write-1 low: the master must release the line within 15 us."""
    bus, mst, ds, chk = _bench(master_timing={"w_low1": 30_000})
    mst.queue_reset()
    mst.queue_write(b"\xcc")
    run(mst, bus)
    return flushed(chk)


def broken_low0_short():
    """A 40 us write-0 low: tLOW0 is 60..120 us.

    40 us and not 30: a write slot whose low is shorter than tLOW1's maximum is a
    write-1 as far as the wire is concerned -- the slave samples it as a 1 -- so a
    shorter one would not break tLOW0, it would break the data instead.
    """
    bus, mst, ds, chk = _bench(master_timing={"w_low0": 40_000})
    mst.queue_reset()
    mst.queue_write(b"\x44")
    run(mst, bus)
    return flushed(chk)


def broken_low0_long():
    """A 200 us write-0 low: tLOW0 ends at 120 us."""
    bus, mst, ds, chk = _bench(master_timing={"w_low0": 200_000})
    mst.queue_reset()
    mst.queue_write(b"\x44")
    run(mst, bus)
    return flushed(chk)


def broken_slot_short():
    """Legal 70 us traffic against a checker told tSLOT is 90..150 us.

    A master that really sent 40 us slots cannot be caught this way: at 40 us the
    next slot's falling edge lands inside the previous slot's period, and on the
    wire that is indistinguishable from a slave answering a read. What can be
    caught, and is what this case does, is a slot shorter than the limit the
    checker is holding.
    """
    bus, mst, ds, chk = _bench(check_timing={"t_slot": (90_000, 150_000)})
    mst.queue_reset()
    mst.queue_write(b"\xcc")
    mst.queue_write(b"\xbe")
    run(mst, bus)
    return flushed(chk)


def broken_slot_long():
    """130 us slots: tSLOT ends at 120 us."""
    bus, mst, ds, chk = _bench(master_timing={"w_slot": 130_000})
    mst.queue_reset()
    mst.queue_write(b"\xcc")
    run(mst, bus)
    return flushed(chk)


def broken_rec():
    """Slots 0.5 us apart at the end: tREC wants at least 1 us of recovery."""
    bus, mst, ds, chk = _bench(master_timing={"w_slot": 60_500, "w_low0": 60_000})
    mst.queue_reset()
    mst.queue_write(b"\xcc")
    run(mst, bus)
    return flushed(chk)


def broken_rdv():
    """A slave that drives its read-0 5 us into the slot: the data is not valid
    until 15 us (tRDV), so a master that samples in the datasheet's window would
    read the line as a 1."""
    bus, mst, ds, chk = _bench(target_kwargs={"tx0_start": 5_000})
    mst.queue_read_scratchpad()
    run(mst, bus)
    return flushed(chk)


def broken_tx0_short():
    """A read-0 that ends 40 us into its slot: tTX0 is 60..120 us."""
    bus, mst, ds, chk = _bench(target_kwargs={"tx0_end": 40_000})
    mst.queue_read_scratchpad()
    run(mst, bus)
    return flushed(chk)


def broken_tx0_long():
    """A read-0 that ends 200 us into its slot, overrunning the next slot."""
    bus, mst, ds, chk = _bench(target_kwargs={"tx0_end": 200_000})
    mst.queue_read_scratchpad()
    run(mst, bus)
    return flushed(chk)


def broken_crc():
    """A ROM whose CRC byte does not match the other seven."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bus.add_net("dq")
    bad = Ds18b20Target(bus=bus, rom=b"\x28\x01\x02\x03\x04\x05\x06\x00")
    mst = attach_master(bus)
    chk = attach_checker(bus)
    mst.queue_read_rom()
    run(mst, bus)
    return flushed(chk)


def broken_rom_command():
    """A function command with no ROM command in front of it: the datasheet says
    the device "will not respond if any steps in the sequence are missing"."""
    bus, mst, ds, chk = _bench()
    mst.queue_reset()
    mst.queue_write(b"\x44")          # Convert T, sent as if it were a ROM command
    run(mst, bus)
    return flushed(chk)


def broken_no_reset():
    """A time slot with no reset pulse before it."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bus.add_net("dq")
    bus.attach(SlotWithoutReset())
    chk = attach_checker(bus)
    bus.run(6_000)
    return flushed(chk)


def broken_slave_drive():
    """A slave that pulls the bus low in the middle of a write slot."""
    bus, mst, ds, chk = _bench()
    bus.attach(ReadZeroDriver())
    mst.queue_reset()
    mst.queue_write(b"\xcc")
    mst.queue_write(b"\xbe")
    run(mst, bus)
    return flushed(chk)


#: rule -> the broken traffic that breaks it. `test_every_documented_rule_is_covered`
#: checks this against `RULES`, so a new rule cannot be added without a case for it.
BROKEN = {
    "tRSTL": broken_rstl_short,
    "presence": broken_no_presence,
    "tPDHIGH": broken_pdhigh_early,
    "tPDLOW": broken_pdlow_short,
    "tRSTH": broken_rsth,
    "tLOW1": broken_low1_short,
    "tLOW0": broken_low0_short,
    "tSLOT": broken_slot_short,
    "tREC": broken_rec,
    "tRDV": broken_rdv,
    "tTX0": broken_tx0_short,
    "CRC": broken_crc,
    "rom-command": broken_rom_command,
    "no-reset": broken_no_reset,
    "slave-drive": broken_slave_drive,
}

# ---- the checker against legal traffic ------------------------------------------
@pytest.mark.parametrize("speed", SPEEDS)
def test_reset_and_presence(speed):
    """A reset pulse and the presence pulse that answers it, at both speeds."""
    bus, mst, ds, chk = rig(speed=speed)
    op = mst.queue_reset()
    run(mst, bus)
    flushed(chk)

    assert op.present is True, "the device answered the reset"
    assert ds.resets == 1 and chk.resets == 1 and chk.presences == 1
    assert chk.clean, chk.report()
    assert bus.errors == [], "open drain: no contention is possible, and none happened"


@pytest.mark.parametrize("speed", SPEEDS)
def test_read_rom(speed):
    """Read ROM [33h]: the device's own 8-byte code, CRC included."""
    bus, mst, ds, chk = rig(speed=speed)
    op = mst.queue_read_rom()
    run(mst, bus)
    flushed(chk)

    assert op.present and bytes(op.data) == ds.rom, (bytes(op.data).hex(), ds.rom.hex())
    assert ds.rom[0] == 0x28, "the family code is the first byte on the wire"
    assert crc8(ds.rom[:7]) == ds.rom[7], "the ROM CRC covers the other seven bytes"
    assert ds.rom_commands == [(ds.rom_commands[0][0], 0x33)]
    assert chk.crc_checks == 1 and chk.clean, chk.report()
    # the checker decoded exactly what went over the wire
    assert [t.value for t in chk.transfers] == [0x33] + list(ds.rom)
    assert chk.transfers[1].group == "rom" and chk.transfers[1].direction == "read"


@pytest.mark.parametrize("speed", SPEEDS)
def test_read_scratchpad_and_crc(speed):
    """Read Scratchpad [BEh]: 9 bytes, the ninth being the CRC of the other eight."""
    bus, mst, ds, chk = rig(speed=speed)
    ds.set_temperature(0x0191)                      # 25.0625 C
    op = mst.queue_read_scratchpad()
    run(mst, bus)
    flushed(chk)

    assert bytes(op.data) == bytes(ds.scratchpad), bytes(op.data).hex()
    assert op.data[:2] == b"\x91\x01", "byte 0 is the temperature LSB, byte 1 the MSB"
    assert crc8(ds.scratchpad[:8]) == ds.scratchpad[8]
    assert chk.clean and chk.crc_checks == 1, chk.report()
    assert ds.function_commands[-1][1] == 0xBE


def test_write_scratchpad_copy_and_recall():
    """Write Scratchpad [4Eh] changes TH, TL and the configuration register;
    Copy Scratchpad [48h] puts them in EEPROM and Recall E2 [B8h] fetches them."""
    bus, mst, ds, chk = rig()
    op = mst.queue_write_scratchpad(th=0x28, tl=0x0A, config=0x20)   # R1:R0 = 01
    run(mst, bus)
    assert (ds.th, ds.tl) == (0x28, 0x0A), ds.write_log
    assert ds.config == 0x3F, "0x20 written, 0x3F read back: bits 0-4 are always 1"
    assert ds.resolution == 10, "R1:R0 = 01 is 10 bits"
    assert ds.scratchpad[4] == 0x3F, (
        "bit 7 reads 0 and bits 0-4 read 1 whatever was written: 0b0011_1111")

    mst.queue_copy_scratchpad()
    run(mst, bus)
    assert bytes(ds.eeprom) == bytes([0x28, 0x0A, 0x3F]), bytes(ds.eeprom).hex()

    ds.th, ds.tl, ds.config = 0x00, 0x00, 0x7F      # clobber the scratchpad
    ds._refresh_scratchpad()
    mst.queue_recall_e2()
    run(mst, bus)
    flushed(chk)
    assert (ds.th, ds.tl, ds.config) == (0x28, 0x0A, 0x3F), "Recall E2 restored them"
    assert [c[1] for c in ds.function_commands] == [0x4E, 0x48, 0xB8]
    assert [c[1] for c in ds.rom_commands] == [0xCC, 0xCC, 0xCC], (
        "Skip ROM in front of each function command")
    assert chk.clean, chk.report()


def test_convert_t_reports_its_status():
    """Convert T [44h], then read slots in the same transaction: 0 while the
    conversion is in progress, 1 when it is done. That is how firmware catches
    itself reading the scratchpad too early."""
    bus, mst, ds, chk = rig(target_kwargs={"conv_ticks": 60_000})
    ds.set_temperature(0x0191)
    mst.queue_convert_t()
    status = mst.queue_read(1)
    run(mst, bus)
    flushed(chk)

    assert ds.converting, "600 us of conversion were asked for"
    assert bytes(status.data) == b"\x00", "all ones low: 0 means still converting"
    assert ds.scratchpad[:2] == b"\x91\x01", "the temperature is in the scratchpad"
    assert chk.clean, chk.report()

    ds.conv_ticks = 0                                # the next one finishes at once
    mst.queue_convert_t()
    status = mst.queue_read(1)
    run(mst, bus)
    flushed(chk)
    assert not ds.converting
    assert bytes(status.data) == b"\xff", "the line idles high once it is done"
    assert chk.clean, chk.report()


def test_match_rom_addresses_exactly_one_device():
    """Match ROM [55h]: only the device whose 64-bit code matches answers."""
    bus = Bus(ns_per_tick=NS_PER_TICK)
    bus.add_net("dq")
    mst = attach_master(bus)
    first = attach_ds18b20(bus, serial=b"\x01\x02\x03\x04\x05\x06")
    second = attach_ds18b20(bus, serial=b"\xaa\xbb\xcc\xdd\xee\xff")
    first.set_temperature(0x0C00)                 # so the two scratchpads differ
    chk = attach_checker(bus)

    mst.queue_match_rom(second.rom)
    mst.queue_write(b"\xbe")
    data = mst.queue_read(9)
    run(mst, bus)
    flushed(chk)

    assert bytes(data.data) == bytes(second.scratchpad)
    assert second.matched and not first.matched
    assert first.match_failures == 1, "the other device saw a ROM code that is not its own"
    assert first.rom != second.rom
    assert chk.clean, chk.report()


def test_search_rom_and_alarm_search():
    """Search ROM [F0h] answers only on the branch the master chose; Alarm Search
    [ECh] answers only while the device is in alarm."""
    for alarming, expect_rom in ((True, True), (False, False)):
        bus = Bus(ns_per_tick=NS_PER_TICK)
        bus.add_net("dq")
        mst = attach_master(bus)
        ds = attach_ds18b20(bus, alarm=alarming)
        chk = attach_checker(bus)
        branch = int.from_bytes(ds.rom, "little")
        op = mst.queue_alarm_search(bits=branch)
        run(mst, bus)
        flushed(chk)
        assert (bytes(op.data) == ds.rom) is expect_rom, bytes(op.data).hex()
        assert chk.clean, chk.report()

    # the branch bit decides: bit 0 of the ROM is 0, so a 1-branch leaves the
    # device silent and a 0-branch makes it answer with its own code.
    bus, mst, ds, chk = rig()
    op = mst.queue_search_rom(bits=(1 << 64) - 1)
    run(mst, bus)
    assert bytes(op.data) == b"\xff" * 8, bytes(op.data).hex()
    op = mst.queue_search_rom(bits=int.from_bytes(ds.rom, "little"))
    run(mst, bus)
    flushed(chk)
    assert bytes(op.data) == ds.rom, bytes(op.data).hex()
    assert chk.clean, chk.report()
    assert not chk.violations_by_rule("CRC"), (
        "a search answer is not a ROM code, so it gets no CRC check")


def test_read_power_supply_reports_parasite_power():
    """Read Power Supply [B4h]: a parasite-powered device transmits 0, an
    externally powered one transmits 1."""
    for parasite, expected in ((True, 0x00), (False, 0xFF)):
        bus, mst, ds, chk = rig(target_kwargs={"parasite_power": parasite})
        mst.queue_reset()
        mst.queue_write(b"\xcc")
        mst.queue_write(b"\xb4")
        op = mst.queue_read(1)
        run(mst, bus)
        flushed(chk)
        assert bytes(op.data) == bytes([expected]), bytes(op.data).hex()
        assert chk.clean, chk.report()


def test_absent_device_is_reported_as_absent():
    """A bus with no device: the master sees no presence, and the checker says so."""
    bus, mst, ds, chk = rig(master=True)
    bus.devices.remove(ds)                       # nothing on the bus but the master
    op = mst.queue_reset()
    run(mst, bus)
    flushed(chk)

    assert op.present is False, "nobody answered the reset"
    assert chk.presences == 0
    assert chk.violations_by_rule("presence"), chk.report()
    assert mst.errors == [], "the master gave up cleanly instead of hanging"


def test_checker_sees_a_whole_transaction():
    """The counts a test can assert on: resets, presence pulses, slots in each
    direction, bytes, CRC checks -- and no violations at all."""
    bus, mst, ds, chk = rig()
    ds.set_temperature(0x0191)
    mst.queue_read_scratchpad()
    run(mst, bus)
    flushed(chk)

    assert (chk.resets, chk.presences) == (1, 1)
    assert chk.slots == 8 + 8 + 9 * 8, chk.report()      # Skip ROM, BEh, 9 bytes
    assert chk.write_slots == 16 and chk.read_slots == 72
    assert chk.bytes == 2 + 9, [str(t) for t in chk.transfers]
    assert chk.crc_checks == 1
    assert chk.clean and bus.errors == [], chk.report()
    groups = [(t.group, t.direction) for t in chk.transfers]
    assert groups[:3] == [("rom_command", "write"), ("function", "write"),
                          ("scratchpad", "read")]
    assert all(g == ("scratchpad", "read") for g in groups[2:])


def test_works_at_the_chip_tick_rate():
    """One full transaction at 20 ns/tick, the rate the chip runs at."""
    bus, mst, ds, chk = rig(ns_per_tick=CHIP_NS)
    ds.set_temperature(0x0191)
    op = mst.queue_read_scratchpad()
    run(mst, bus)
    flushed(chk)

    assert bytes(op.data) == bytes(ds.scratchpad)
    assert chk.clean, chk.report()
    assert chk.slots == 8 + 8 + 72
    assert bus.t > 300_000, "a 1-Wire transaction really is that many ticks at 50 MHz"


# ---- the checker against broken traffic ----------------------------------------
@pytest.mark.parametrize("rule", sorted(BROKEN))
def test_checker_flags_each_spec_rule(rule):
    """One piece of broken traffic per rule in `RULES`, and every violation has to
    carry its tick."""
    chk = BROKEN[rule]()
    hits = chk.violations_by_rule(rule)
    assert hits, f"{rule} was not caught: {chk.report()}"
    for v in hits:
        assert v.tick >= 0, v
        if v.bound != "framing":
            assert v.measured is not None and v.required is not None, v
            if v.bound == "min":
                assert v.measured < v.required, v
            else:
                assert v.measured > v.required, v


def test_every_documented_rule_is_covered():
    """`RULES` is the checker's contract and `BROKEN` is the evidence for it: a new
    rule cannot be added without a case that breaks it."""
    assert set(BROKEN) == set(RULES), set(RULES) ^ set(BROKEN)
    assert len(RULES) == len(set(RULES)), "RULES has a duplicate"


@pytest.mark.parametrize("rule", ["tRSTL", "tPDHIGH", "tPDLOW", "tLOW1",
                                  "tLOW0", "tSLOT", "tTX0"])
def test_short_and_long_have_separate_bounds(rule):
    """Every timing window is enforced at both ends: a value inside it is not a
    violation, a value below the minimum and a value above the maximum are two
    different violations. (`tRSTH` and `tREC` are floors only, and `tRDV` is a
    floor with no source for an overdrive maximum, so neither has a "long" case.)"""
    cases = {
        "tRSTL": (broken_rstl_short, broken_rstl_long),
        "tPDHIGH": (broken_pdhigh_early, broken_pdhigh_late),
        "tPDLOW": (broken_pdlow_short, broken_pdlow_long),
        "tLOW1": (broken_low1_short, broken_low1_long),
        "tLOW0": (broken_low0_short, broken_low0_long),
        "tSLOT": (broken_slot_short, broken_slot_long),
        "tTX0": (broken_tx0_short, broken_tx0_long),
    }
    near, far = cases[rule]
    for builder, bound in ((near, "min"), (far, "max")):
        hits = builder().violations_by_rule(rule)
        assert hits, f"{builder.__name__} did not trip {rule}"
        assert hits[0].bound == bound, hits[0]
    # the min and max cases really are different measurements
    assert near().violations_by_rule(rule)[0].measured != \
        far().violations_by_rule(rule)[0].measured


def test_checker_is_not_vacuous():
    """Legal standard-speed traffic judged against overdrive limits must fail, and
    the same wire must pass against its own speed: the checker reads the limits it
    was given, and it is not simply never complaining."""
    bus, mst, ds, chk = rig(check_speed="overdrive")
    mst.queue_read_scratchpad()
    run(mst, bus)
    flushed(chk)

    assert chk.violations, "a 100 kbit/s bus must not pass an overdrive check"
    assert chk.rules_seen().issuperset({"tRSTL", "tLOW0"}), chk.report()
    for v in chk.violations:
        if v.bound == "framing":
            continue                    # a framing violation has no measurement
        assert v.measured is not None and v.required is not None
        assert v.measured < v.required or v.measured > v.required, v

    bus2, mst2, ds2, chk2 = rig()                    # the same bench, its own speed
    op = mst2.queue_read_scratchpad()
    run(mst2, bus2)
    flushed(chk2)
    assert chk2.clean and bytes(op.data) == bytes(ds2.scratchpad), chk2.report()


def test_checker_only_flags_what_the_traffic_broke():
    """One broken rule does not drag the rest of the bench down: the reset and the
    presence of a legal master are still clean when only the slot length is wrong."""
    chk = broken_slot_long()
    assert chk.rules_seen() == {"tSLOT"}, chk.report()
    assert chk.resets == 1 and chk.presences == 1
    assert chk.clean is False and len(chk.violations) > 1, "every slot is too long"


def test_finish_is_idempotent_and_only_needed_at_the_end():
    """`finish()` judges what the last edge could not: a trace that ends waiting
    for a presence pulse. Calling it twice must not invent a second violation."""
    bus, mst, ds, chk = rig(master=False)
    chk.tick(0, {"dq": 1})
    chk.finish()
    assert chk.violations == [], chk.report()
    # an open reset: drive the line low for the length tRSTL and stop there
    chk = attach_checker(bus)
    chk.tick(0, {"dq": 0})
    limit = speed_minimums(NS_PER_TICK)["t_rstl"][0]
    chk.tick(limit, {"dq": 0})
    chk.tick(limit + 1, {"dq": 1})
    assert chk.resets == 1 and chk.phase == "await_presence"
    assert chk.violations == [], "nothing to say yet"
    chk.finish()
    flushed(chk)
    assert chk.violations_by_rule("presence"), chk.report()


def test_violations_str_read_like_sentences():
    """The report is what a firmware author reads first, so check all three forms."""
    bus, mst, ds, chk = _bench(master_timing={"w_low0": 200_000})
    mst.queue_reset()
    mst.queue_write(b"\x44")
    run(mst, bus)
    flushed(chk)
    over = chk.violations_by_rule("tLOW0")
    assert over and "tLOW0" in str(over[0]) and ">" in str(over[0]), str(over[0])

    short = broken_low1_short().violations_by_rule("tLOW1")
    assert short and "<" in str(short[0]), str(short[0])
    framing = broken_crc().violations_by_rule("CRC")
    assert framing and "CRC" in str(framing[0]) and "computed" in str(framing[0])
    assert chk.report().startswith("standard speed at"), chk.report()


# ---- the tables the model is built from, and its guard rails ---------------------
def test_timing_table_is_the_spec():
    """Guard against the numbers quietly drifting. Every value here is one of the
    two tables in `research/ref/protocols/onewire/`, in nanoseconds.

    DS18B20 AC electrical characteristics: tSLOT 60..120, tREC 1, tLOW0 60..120,
    tLOW1 1..15, tRDV 15, tRSTH 480, tRSTL 480, tPDHIGH 15..60, tPDLOW 60..240.
    AN126 Table 2, the overdrive column: A 1/1.5/1.85, C 7/7.5/14, E 0.5/0.75/0.85,
    H 68/70/80, I 7.2/8.5/8.8, J 39.5/40.
    """
    std = speed_minimums(CHIP_NS)
    assert std["t_slot"] == (3000, 6000), "60..120 us at 20 ns/tick"
    assert std["t_rec"] == 50, "1 us"
    assert std["t_low0"] == (3000, 6000)
    assert std["t_low1"] == (50, 750), "1..15 us"
    assert std["t_read_low"] == (50, 750)
    assert std["t_rdv"] == 750, "15 us"
    assert std["t_rsth"] == 24_000, "480 us"
    assert std["t_rstl"][0] == 24_000, "480 us"
    assert std["t_pdhigh"] == (750, 3000), "15..60 us"
    assert std["t_pdlow"] == (3000, 12_000), "60..240 us"
    # the maximum of the reset low comes from AN126's H, the only source for it
    assert speed_minimums(CHIP_NS, "standard", {"t_rstl": (480_000, 960_000)}) \
        ["t_rstl"] == (24_000, 48_000)

    fast = speed_minimums(CHIP_NS, "overdrive")
    assert fast["t_rstl"] == (3400, 4000), "AN126 H: 68..80 us"
    assert fast["t_low0"] == (350, 700), "AN126 C: 7..14 us"
    assert fast["t_low1"] == (50, 93), "AN126 A: 1..1.85 us"
    assert fast["t_slot"][0] == 350, "AN126 E+F: 7.25 us, the shortest overdrive slot"
    assert fast["t_slot"][1] is None, "no overdrive maximum exists to check"
    assert fast["t_tx0"] == (None, None), "no overdrive source for TX 0"
    assert fast["t_pdhigh"] == std["t_pdhigh"], (
        "the presence pulse is not a scaled signal, so it keeps the DS18B20 numbers")

    with pytest.raises(ValueError):
        speed_minimums(CHIP_NS, "fast-mode-plus")
    with pytest.raises(ValueError):
        speed_minimums(CHIP_NS, "standard", {"t_nonsense": 1})
    with pytest.raises(ValueError):
        speed_minimums(CHIP_NS, "standard", {"speed": "overdrive"})


def test_crc_is_the_dallas_maxim_polynomial():
    """x^8 + x^5 + x^4 + 1 == 0x31, seeded with 0, no reflection, and a worked
    value: the CRC of the family code 0x28 alone is 0x9C in a ROM whose other bytes
    are zero."""
    assert crc8(b"") == 0
    assert crc8(b"\x00") == 0, "the seed is 0 and a zero byte leaves it 0"
    assert crc8(b"123456789") == 0xA1, (
        "the check value every CRC-8/MAXIM implementation has to produce")
    rom = rom_from_serial(b"\x00\x00\x00\x00\x00\x00")
    assert rom == bytes([0x28, 0, 0, 0, 0, 0, 0, crc8(bytes([0x28, 0, 0, 0, 0, 0, 0]))])
    assert crc8(rom[:7]) == rom[7]
    assert rom_from_id(1)[:7] == bytes([0x28]) + (1).to_bytes(6, "little")
    assert rom_from_serial(b"\x01\x02\x03\x04\x05\x06")[0] == 0x28
    assert rom_from_serial(b"\x01\x02\x03\x04\x05\x06", family=0x10)[0] == 0x10


def test_power_up_state_matches_the_datasheet():
    """The scratchpad a fresh DS18B20 powers up with: +85 C, TH 75 C, TL 0 C, the
    configuration register at 12 bits, the three reserved bytes, and a CRC that
    matches."""
    ds = Ds18b20Target()
    assert bytes(ds.scratchpad[:2]) == b"\x50\x05", "+85 C is 0x0550, LSB first"
    assert ds.th == 0x4B and ds.tl == 0x00, "75 C and 0 C"
    assert ds.config == 0x7F and ds.resolution == 12, "R1:R0 = 11 is 12 bits"
    assert bytes(ds.scratchpad[5:8]) == bytes([0xFF, 0xFF, 0x10]), "the reserved bytes"
    assert crc8(ds.scratchpad[:8]) == ds.scratchpad[8]
    assert ds.write_log == [], "a fresh device has been written nothing"
    ds.resolution = 9
    assert ds.config == 0x1F and ds.resolution == 9, "R1:R0 = 00 is 9 bits"
    ds.resolution = 12
    assert ds.config == 0x7F, "R1:R0 = 11 is 12 bits"
    assert bytes(ds.scratchpad[4:5]) == b"\x7f", "bits 0-4 always read 1"
    # tCONV for each resolution, as the datasheet's table has it
    for bits, ms in ((9, 93.75), (10, 187.5), (11, 375), (12, 750)):
        assert Ds18b20Target.conv_time_ticks(bits, CHIP_NS) == int(ms * 1e6 / CHIP_NS)
    assert Ds18b20Target.conv_time_ticks(12, CHIP_NS) == 37_500_000, (
        "750 ms is 37.5 million ticks at 50 MHz, which is why conv_ticks exists")
    with pytest.raises(ValueError):
        Ds18b20Target.conv_time_ticks(13, CHIP_NS)
    with pytest.raises(ValueError):
        ds.resolution = 8


def test_guard_rails():
    """A bad construction or a bad queue is an error, never a hung simulation."""
    with pytest.raises(ValueError):
        speed_minimums(CHIP_NS, "hyperdrive")
    with pytest.raises(ValueError):
        rom_from_serial(b"\x01\x02\x03")
    with pytest.raises(ValueError):
        Ds18b20Target(rom=b"\x28\x01")
    with pytest.raises(ValueError):
        Ds18b20Target().resolution = 13
    with pytest.raises(ValueError):
        Ds18b20Target(tx0_start=50_000, tx0_end=10_000)

    bus, mst, ds, chk = rig()
    with pytest.raises(ValueError):
        mst.queue_write(b"\xcc")           # no reset yet
    with pytest.raises(ValueError):
        mst.queue_read(-1)
    with pytest.raises(ValueError):
        mst.queue_search_rom(bits=1 << 64)
    mst.queue_reset()
    mst.queue_write(b"\xcc")
    mst.queue_read(1)                     # inside the transaction this is fine
    assert len(mst.ops) == 3
    assert mst.busy and not mst.idle
    run(mst, bus)
    assert mst.idle and mst.transactions[0][2] == "reset"
    assert chk.clean, chk.report()


def test_a_master_that_never_hears_a_presence_gives_up_cleanly():
    """A reset with nobody on the bus: the master waits its tPDHIGH maximum, then
    carries on, and records that nobody answered instead of hanging."""
    bus, mst, ds, chk = rig(targets=0)
    op = mst.queue_reset()
    assert mst.run(bus, limit=2_000_000), "the master must not hang on an empty bus"
    assert op.present is False
    assert mst.errors == []
    ticks = mst.presence_wait
    assert bus.t > ticks, "it really did wait before giving up"
    flushed(chk)
    assert chk.presences == 0 and chk.violations_by_rule("presence"), chk.report()


def test_the_checker_and_the_master_agree_on_every_byte():
    """The checker decodes the wire independently of the master, so their two
    accounts of the transaction have to be identical."""
    bus, mst, ds, chk = rig()
    ds.set_temperature(0x0550)
    write = mst.queue_write_scratchpad(0x28, 0x0A, 0x20)
    read = mst.queue_read_scratchpad()
    run(mst, bus)
    flushed(chk)

    written = [t.value for t in chk.transfers if t.direction == "write"]
    read_back = [t.value for t in chk.transfers if t.direction == "read"]
    assert written == [0xCC, 0x4E, 0x28, 0x0A, 0x20, 0xCC, 0xBE]
    assert read_back == list(ds.scratchpad) == list(read.data)
    assert write.ok and read.ok
    assert chk.clean and chk.bytes == 7 + 9, chk.report()
    assert bus.errors == []

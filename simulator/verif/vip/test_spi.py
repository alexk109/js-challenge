"""Self-tests for the SPI reference models in `verif/vip/spi.py`.

Run them with `make vip-selftest VIP=spi`. Everything here drives real pins on the bus
from `core.py`: no shortcuts into device state, so a test that passes means the timing
works on the wire, not that the models agree with themselves.
"""
from __future__ import annotations

import pytest

from verif.vip.core import Device
from verif.vip.spi import (
    CS_N,
    CS_ACTIVE,
    MISO,
    MOSI,
    SCK,
    CMD_ERASE,
    CMD_JEDEC,
    CMD_PROGRAM,
    CMD_RDSR,
    CMD_READ,
    CMD_WRDI,
    CMD_WREN,
    PAGE_SIZE,
    SECTOR_SIZE,
    SpiController,
    SpiFlashTarget,
    SpiMonitor,
    bits_of,
    byte_of,
    cpha,
    cpol,
    first_output_index,
    is_capture_edge,
    spi_bus,
)

#: Modes the flash datasheets support, and the half periods the tests sweep.
FLASH_MODES = (0, 3)
HALF_PERIODS = (1, 2, 3, 4, 7)
ALL_MODES = (0, 1, 2, 3)


def addr_bytes(addr: int) -> list[int]:
    return [(addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]


class PatternTarget(Device):
    """A target that puts a known bit pattern on MISO, for checking a master.

    `spi.py` ships the flash, which only speaks modes 0 and 3, so this covers the other
    two. It drives nothing while CS_N is high and puts one bit out per launch edge, the
    way a shift register does, so it works in every mode.
    """

    name = "pattern"

    def __init__(self, bits: list[int], mode: int, lead_bits: int = 0):
        self.mode = mode
        self.lead_bits = lead_bits
        self.bits = [0] * lead_bits + list(bits)
        self._out_idx = first_output_index(mode)
        self._cs: int | None = None
        self._sck: int | None = None

    def _bit(self, index: int) -> int:
        return self.bits[index] if 0 <= index < len(self.bits) else 0

    def tick(self, t: int, pins):
        cs = pins[CS_N]
        sck = pins[SCK]
        if self._cs is None:
            self._cs, self._sck = cs, sck
        elif cs != self._cs:
            if cs == CS_ACTIVE:
                self._out_idx = first_output_index(self.mode)
            self._cs, self._sck = cs, sck
        miso = None
        if cs == CS_ACTIVE:
            if self._sck is not None and sck != self._sck:
                if not is_capture_edge(self.mode, self._sck, sck):
                    self._out_idx += 1
            miso = self._bit(self._out_idx)
        self._sck = sck
        return {MISO: miso}


def build(mode: int = 0, half: int = 4, cs_setup: int = 2, cs_hold: int = 2,
          cs_gap: int = 2, monitor: SpiMonitor | None = None, flash_size: int = 4096):
    """A bus with a controller, a flash and (optionally) a monitor attached."""
    bus = spi_bus()
    ctl = SpiController(mode=mode, half_period=half, cs_setup=cs_setup, cs_hold=cs_hold,
                        cs_gap=cs_gap)
    flash = SpiFlashTarget(mode=mode, size=flash_size)
    bus.attach(ctl)
    bus.attach(flash)
    if monitor is not None:
        bus.attach(monitor)
    return bus, ctl, flash


def read_cmd(addr: int, n: int) -> list[int]:
    return [CMD_READ] + addr_bytes(addr)


def program_cmd(addr: int, data) -> list[int]:
    return [CMD_PROGRAM] + addr_bytes(addr) + list(data)


def enable_write(ctl: SpiController, addr: int, data) -> None:
    """WREN then a page program, the way a driver does it."""
    ctl.add([CMD_WREN], 0)
    ctl.add(program_cmd(addr, data), 0)


# -- helpers and edge rules ----------------------------------------------------


def test_mode_bits_match_the_datasheet():
    assert [(cpol(m), cpha(m)) for m in ALL_MODES] == [(0, 0), (0, 1), (1, 1), (1, 0)]


def test_bit_and_byte_helpers_round_trip():
    for byte in (0x00, 0x01, 0x80, 0x9F, 0xFF):
        bits = bits_of(byte)
        assert len(bits) == 8
        assert bits[0] == (byte >> 7) & 1 and bits[7] == byte & 1
        assert byte_of(bits) == byte
    assert byte_of(bits_of(0x12) + bits_of(0x34), 8) == 0x34


def test_capture_edge_is_the_leading_edge_when_cpha_is_0():
    # Mode 0: rising captures, falling launches. Mode 3: the other way round.
    assert is_capture_edge(0, 0, 1) is True and is_capture_edge(0, 1, 0) is False
    assert is_capture_edge(3, 1, 0) is True and is_capture_edge(3, 0, 1) is False
    # Mode 1: rising launches, falling captures. Mode 2: the other way round.
    assert is_capture_edge(1, 0, 1) is False and is_capture_edge(1, 1, 0) is True
    assert is_capture_edge(2, 1, 0) is False and is_capture_edge(2, 0, 1) is True
    assert is_capture_edge(0, 1, 1) is False      # no transition, so no edge


def test_first_output_index_follows_cpha():
    assert first_output_index(0) == 0 and first_output_index(3) == 0
    assert first_output_index(1) == -1 and first_output_index(2) == -1


def test_bad_parameters_are_rejected():
    with pytest.raises(ValueError):
        SpiController(mode=4)
    with pytest.raises(ValueError):
        SpiController(half_period=0)
    with pytest.raises(ValueError):
        SpiFlashTarget(mode=1)          # flash modes are 0 and 3 only
    with pytest.raises(ValueError):
        SpiFlashTarget(mode=2)
    with pytest.raises(ValueError):
        SpiMonitor(mode=-1)


def test_flash_tristates_miso_while_cs_is_high():
    bus, ctl, flash = build()
    bus.attach(PatternTarget([1] * 8, mode=0))
    ctl.add([CMD_JEDEC], 0)
    ctl.run(bus)
    assert flash.tick(0, {CS_N: 1, SCK: 0, MOSI: 0, MISO: 1}) == {MISO: None}
    assert bus.values[MISO] == 1        # the pull-up holds it high while deselected


# -- controller against the flash -----------------------------------------------


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_jedec_id(mode, half):
    bus, ctl, flash = build(mode=mode, half=half)
    flash.jedec_id = [0xEF, 0x40, 0x18]
    ctl.add([CMD_JEDEC], 3)
    ticks = ctl.run(bus)
    assert ctl.rx == [0xEF, 0x40, 0x18] == list(flash.jedec_id)
    assert ctl.transfers[0].miso == ctl.rx
    assert not bus.errors                # nothing drove MISO twice
    assert ticks > 0


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_program_then_read_back(mode, half):
    bus, ctl, flash = build(mode=mode, half=half)
    data = [0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22]
    enable_write(ctl, 0x0123, data)
    ctl.add(read_cmd(0x0123, len(data)), len(data))
    ctl.run(bus)
    assert ctl.rx == data
    assert flash.peek(0x0123, len(data)) == data
    assert flash.programs == 1
    # A read from a different address sees the same bytes shifted. `rx` is cumulative,
    # so only the bytes of this run are new.
    ctl.add(read_cmd(0x0124, 4), 4)
    ctl.run(bus)
    assert ctl.rx[-4:] == data[1:5]


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_program_uses_and_semantics(mode, half):
    """Programming only clears bits, so 1s over 0s do nothing."""
    bus, ctl, flash = build(mode=mode, half=half)
    enable_write(ctl, 0x0300, [0b1010_1010])
    enable_write(ctl, 0x0300, [0b1111_0000])
    ctl.add(read_cmd(0x0300, 1), 1)
    ctl.run(bus)
    assert ctl.rx == [0b1010_0000]
    assert flash.peek(0x0300, 1) == [0b1010_0000]


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_page_program_wraps_inside_the_page(mode, half):
    bus, ctl, flash = build(mode=mode, half=half)
    page = 0x0400
    start = page + PAGE_SIZE - 2
    enable_write(ctl, start, [0x11, 0x22, 0x33, 0x44])
    ctl.add(read_cmd(page, 4), 4)
    ctl.run(bus)
    # The last two bytes land at the start of the same page, not in the next one.
    assert ctl.rx == [0x33, 0x44, 0xFF, 0xFF]
    assert flash.peek(page + PAGE_SIZE - 2, 2) == [0x11, 0x22]
    assert flash.peek(page + PAGE_SIZE, 2) == [0xFF, 0xFF]


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_program_without_wren_does_nothing(mode, half):
    bus, ctl, flash = build(mode=mode, half=half)
    flash.poke(0x0500, [0xFF, 0xFF])
    ctl.add(program_cmd(0x0500, [0xAA]), 0)
    ctl.run(bus)
    assert flash.peek(0x0500, 2) == [0xFF, 0xFF]
    assert flash.programs == 0
    assert flash.wel == 0
    # With WEL set it lands, and WEL clears again afterwards.
    enable_write(ctl, 0x0500, [0xAA])
    ctl.run(bus)
    assert flash.peek(0x0500, 1) == [0xAA]
    assert flash.programs == 1
    assert flash.wel == 0


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_erase_needs_wren_and_clears_the_sector(mode, half):
    bus, ctl, flash = build(mode=mode, half=half, flash_size=4 * SECTOR_SIZE)
    enable_write(ctl, 0x0800, [0x5A] * 8)
    ctl.run(bus)
    assert flash.peek(0x0800, 4) == [0x5A] * 4
    # No WEL: the sector must survive.
    ctl.add([CMD_ERASE] + addr_bytes(0x0800), 0)
    ctl.run(bus)
    assert flash.peek(0x0800, 4) == [0x5A] * 4
    assert flash.erases == 0
    enable_write(ctl, 0x0800 + SECTOR_SIZE, [0x5A] * 8)   # the next sector along
    ctl.run(bus)
    ctl.add([CMD_WREN], 0)
    ctl.add([CMD_ERASE] + addr_bytes(0x0800), 0)
    ctl.add(read_cmd(0x0800, 4), 4)
    ctl.run(bus)
    assert flash.erases == 1
    assert ctl.rx == [0xFF] * 4
    # Only that 4 KB sector went: the next one is untouched.
    assert flash.peek(0x0800 + SECTOR_SIZE, 4) == [0x5A] * 4
    assert flash.programs == 2


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_read_wraps_at_the_end_of_the_array(mode, half):
    size = 4096
    bus, ctl, flash = build(mode=mode, half=half, flash_size=size)
    flash.poke(size - 2, [0x11, 0x22])
    flash.poke(0, [0x33, 0x44])
    ctl.add(read_cmd(size - 2, 4), 4)
    ctl.run(bus)
    assert ctl.rx == [0x11, 0x22, 0x33, 0x44]


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_status_register_reports_wel(mode, half):
    bus, ctl, flash = build(mode=mode, half=half)
    ctl.add([CMD_RDSR], 1)
    ctl.run(bus)
    assert ctl.rx == [0]
    ctl.add([CMD_WREN], 0)
    ctl.add([CMD_RDSR], 1)
    ctl.run(bus)
    assert ctl.rx[-1] == 1
    ctl.add([CMD_WRDI], 0)
    ctl.add([CMD_RDSR], 1)
    ctl.run(bus)
    assert ctl.rx[-1] == 0


def test_unknown_opcode_is_ignored():
    """Nothing is written and nothing is sent back: the flash drives zeros."""
    bus, ctl, flash = build()
    flash.poke(0x0900, [0xC3, 0x3C])
    ctl.add([0x11, 0x22], 2)
    ctl.run(bus)
    assert ctl.rx == [0x00, 0x00]
    assert flash.last_command == 0x11
    assert flash.peek(0x0900, 2) == [0xC3, 0x3C]


# -- the monitor against the controller -----------------------------------------


@pytest.mark.parametrize("mode", ALL_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_monitor_decodes_every_mode_like_the_controller(mode, half):
    """What went on the wire, decoded by the monitor and read by the master, must agree."""
    sent = [0x9F, 0xA5, 0x0F]
    reply = [0x12, 0x34, 0x56]
    reply_bits = [b for byte in reply for b in bits_of(byte)]
    bus = spi_bus()
    ctl = SpiController(mode=mode, half_period=half)
    mon = SpiMonitor(mode=mode, min_sck_ticks=half)
    # `lead_bits` holds the target off until the opcode bytes have gone by, the way a
    # real target answers only after it has its command and address.
    bus.attach(ctl)
    bus.attach(PatternTarget(reply_bits, mode, lead_bits=8 * len(sent)))
    bus.attach(mon)
    ctl.add(sent, len(reply))
    ctl.run(bus)
    assert len(mon.transfers) == 1
    seen = mon.transfers[0]
    # The monitor sees every byte on MOSI, the fill bytes included; the master only
    # samples MISO while it is reading.
    assert seen.mosi == sent + [0x00] * len(reply)
    assert seen.miso == [0x00] * len(sent) + reply
    assert ctl.tx == sent               # what was queued; `fill` is not counted there
    assert seen.mosi == ctl.tx + [0x00] * len(reply)
    assert ctl.rx == reply              # the master's own sample is the reply alone
    assert mon.violations == []
    assert mon.stats["bits"] == 8 * (len(sent) + len(reply))


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half", HALF_PERIODS)
def test_monitor_decodes_flash_traffic(mode, half):
    sent = [CMD_JEDEC]
    reply = [0xEF, 0x40, 0x18]
    mon = SpiMonitor(mode=mode, min_sck_ticks=half, min_cs_setup=2, min_cs_hold=2)
    bus, ctl, flash = build(mode=mode, half=half, monitor=mon)
    flash.jedec_id = reply
    ctl.add(sent, 3)
    ctl.run(bus)
    assert mon.violations == []
    seen = mon.transfers[0]
    assert seen.mosi == [CMD_JEDEC, 0, 0, 0]
    assert seen.miso == [0x00] + reply       # the opcode byte itself has no answer
    assert ctl.rx == reply
    # The decode of the transaction lines up with the pins the controller drove.
    assert seen.cs_setup is not None and seen.cs_setup >= 2
    assert seen.cs_hold is not None and seen.cs_hold >= 2
    assert seen.t_end is not None and seen.t_end > seen.t_start


@pytest.mark.parametrize("mode", FLASH_MODES)
def test_monitor_sees_every_transaction(mode):
    mon = SpiMonitor(mode=mode)
    bus, ctl, flash = build(mode=mode, half=4, monitor=mon)
    ctl.add([CMD_JEDEC], 3)
    ctl.add([CMD_WREN], 0)
    ctl.add([CMD_RDSR], 1)
    ctl.run(bus)
    assert [x.mosi for x in mon.transfers] == [
        [CMD_JEDEC, 0, 0, 0], [CMD_WREN], [CMD_RDSR, 0x00],
    ]
    # The RDSR window is two bytes wide, so the status lands in the second one.
    assert mon.transfers[2].miso == [0x00, 0x01]
    assert mon.stats["transactions"] == 3


@pytest.mark.parametrize("mode", FLASH_MODES)
def test_monitor_flags_a_short_sck_high_time(mode):
    """A limit one tick above the half period must be flagged, and nothing else."""
    half = 4
    mon = SpiMonitor(mode=mode, min_sck_ticks=half + 1)
    bus, ctl, flash = build(mode=mode, half=half, monitor=mon)
    ctl.add([CMD_JEDEC], 3)
    ctl.run(bus)
    high = "high" if cpol(mode) == 0 else "low"
    flagged = [v for v in mon.violations if v.startswith(f"sck {high} time")]
    assert flagged, mon.violations
    assert all(f"< {half + 1}" in v for v in mon.violations)
    # Both levels are half ticks, so both are under the limit.
    assert mon.stats[f"sck_{high}_min"] == half
    assert mon.stats["sck_low_min"] == half and mon.stats["sck_high_min"] == half
    # Nothing about the transfer itself is wrong.
    assert mon.transfers[0].miso[1:] == list(flash.jedec_id)
    assert ctl.rx == list(flash.jedec_id)


@pytest.mark.parametrize("mode", FLASH_MODES)
def test_monitor_is_quiet_at_exactly_the_half_period(mode):
    half = 5
    mon = SpiMonitor(mode=mode, min_sck_ticks=half, min_cs_setup=2, min_cs_hold=2)
    bus, ctl, flash = build(mode=mode, half=half, monitor=mon)
    ctl.add([CMD_JEDEC], 3)
    ctl.run(bus)
    assert mon.violations == []
    assert mon.stats["sck_high_min"] == half
    assert mon.stats["sck_low_min"] == half


def test_monitor_flags_short_cs_setup_and_hold():
    mon = SpiMonitor(mode=0, min_cs_setup=9, min_cs_hold=40)
    bus, ctl, flash = build(mode=0, half=4, cs_setup=2, cs_hold=3, monitor=mon)
    ctl.add([CMD_JEDEC], 3)
    ctl.run(bus)
    setup = [v for v in mon.violations if v.startswith("cs setup")]
    hold = [v for v in mon.violations if v.startswith("cs hold")]
    assert len(setup) == 1 and len(hold) == 1
    assert mon.transfers[0].cs_setup == 2 + 4       # cs_setup ticks, then half a period
    assert mon.transfers[0].cs_hold == 3


def test_monitor_drives_nothing():
    """It is a watcher: attaching it must not change the bus or add contention."""
    plain = spi_bus()
    ctl_a = SpiController(mode=0, half_period=4)
    plain.attach(ctl_a)
    plain.attach(SpiFlashTarget(mode=0))
    ctl_a.add([CMD_JEDEC], 3)
    ctl_a.run(plain)

    watched = spi_bus()
    ctl_b = SpiController(mode=0, half_period=4)
    mon = SpiMonitor(mode=0)
    watched.attach(ctl_b)
    watched.attach(SpiFlashTarget(mode=0))
    watched.attach(mon)
    ctl_b.add([CMD_JEDEC], 3)
    ctl_b.run(watched)

    assert ctl_a.rx == ctl_b.rx
    assert watched.errors == []
    assert mon.transfers[0].mosi[0] == CMD_JEDEC


# -- controller timing on the wire ---------------------------------------------


@pytest.mark.parametrize("mode", FLASH_MODES)
@pytest.mark.parametrize("half,cs_setup,cs_hold,cs_gap",
                         [(2, 1, 1, 1), (4, 3, 5, 2), (7, 0, 0, 4)])
def test_controller_honours_cs_setup_hold_and_gap(mode, half, cs_setup, cs_hold, cs_gap):
    bus = spi_bus(trace=True)
    ctl = SpiController(mode=mode, half_period=half, cs_setup=cs_setup, cs_hold=cs_hold,
                        cs_gap=cs_gap)
    bus.attach(ctl)
    bus.attach(SpiFlashTarget(mode=mode))
    ctl.add([CMD_JEDEC], 3)
    ctl.add([CMD_RDSR], 1)
    mon = SpiMonitor(mode=mode)
    bus.attach(mon)
    ticks = ctl.run(bus)
    trace = [v for _t, v in bus.trace]

    # Each CS_N low window on the wire is one transaction, and nothing else pulls CS_N
    # down. A window is cs_setup, two half periods per bit, then cs_hold.
    windows = []
    start = None
    for i, v in enumerate(trace):
        if v[CS_N] == CS_ACTIVE and start is None:
            start = i
        elif v[CS_N] != CS_ACTIVE and start is not None:
            windows.append((start, i))
            start = None
    assert len(windows) == 2
    for n, ((begin, end), xfer, nbits) in enumerate(zip(windows, ctl.transfers, (32, 16))):
        assert xfer.t_start == begin
        assert end - begin == cs_setup + 2 * half * nbits + cs_hold
        # The monitor sees the setup from CS_N low to the first SCK edge, which is a
        # half period into the first bit, and the hold from the last edge to CS_N high.
        seen = mon.transfers[n]
        assert seen.cs_setup == cs_setup + half
        assert seen.cs_hold == cs_hold
    assert windows[1][0] - windows[0][1] == cs_gap
    assert ticks == ctl.transfers[-1].t_end + 2 * half + 2   # the plan's trailing slack


@pytest.mark.parametrize("mode", FLASH_MODES)
def test_a_transaction_costs_setup_hold_and_two_half_periods_per_bit(mode):
    half = 3
    bus, ctl, flash = build(mode=mode, half=half, cs_setup=2, cs_hold=4)
    ctl.add([CMD_JEDEC], 5)
    ctl.run(bus)
    xfer = ctl.transfers[0]
    bits = 8 * (1 + 5)
    assert xfer.t_start == 0
    assert xfer.t_end == 2 + 2 * half * bits + 4
    # Past the three ID bytes the flash has nothing left to send, and drives zeros.
    assert ctl.rx == [0xC0, 0x28, 0x17, 0x00, 0x00]


def test_two_transactions_are_two_cs_windows():
    """Without the gap the target would read them as one long command stream."""
    bus, ctl, flash = build(mode=0, half=4, cs_setup=2, cs_hold=2, cs_gap=2)
    mon = SpiMonitor(mode=0)
    bus.attach(mon)
    ctl.add([CMD_JEDEC], 3)
    ctl.add([CMD_RDSR], 1)
    ctl.run(bus)
    assert len(mon.transfers) == 2
    assert mon.transfers[0].miso == [0x00] + list(flash.jedec_id)
    assert mon.transfers[1].miso == [0x00, 0x00]              # WEL was clear

    # With cs_gap 0 the two merge into one window and the flash sees a stream, so the
    # RDSR opcode is swallowed as a data byte of the JEDEC command.
    merged, mctl, _mflash = build(mode=0, half=4, cs_setup=2, cs_hold=2, cs_gap=0)
    mmon = SpiMonitor(mode=0)
    merged.attach(mmon)
    mctl.add([CMD_JEDEC], 3)
    mctl.add([CMD_RDSR], 1)
    mctl.run(merged)
    assert len(mmon.transfers) == 1
    assert mmon.transfers[0].mosi == [CMD_JEDEC, 0, 0, 0, CMD_RDSR, 0x00]


def test_the_flash_answers_one_tick_after_an_sck_edge():
    """Like a flop: it sees the edge, then its new bit is on the bus the tick after."""
    bus = spi_bus(trace=True)
    ctl = SpiController(mode=0, half_period=4)
    flash = SpiFlashTarget(mode=0)
    flash.jedec_id = [0xC0, 0x28, 0x17]
    bus.attach(ctl)
    bus.attach(flash)
    ctl.add([CMD_JEDEC], 3)
    ctl.run(bus)
    trace = [v for _t, v in bus.trace]
    # Trace entry t holds the values the devices see at tick t+1. The flash notices an
    # SCK edge in the entry where it appears and puts its new bit out in that same tick,
    # so the bit lands on the bus one entry later: a one tick reaction, like a flop.
    sck_changes = {i for i in range(1, len(trace)) if trace[i][SCK] != trace[i - 1][SCK]}
    miso_rises = [
        i for i in range(1, len(trace))
        if trace[i][MISO] == 1 and trace[i - 1][MISO] == 0
        and trace[i][CS_N] == CS_ACTIVE and trace[i - 1][CS_N] == CS_ACTIVE
    ]   # the rise when CS_N goes high is the release, not a launch, so it is left out
    assert miso_rises, "the flash never put a 1 on MISO"
    assert all(i - 1 in sck_changes for i in miso_rises)
    # And it lets go of MISO while CS_N is high: one tick after CS_N rises the flash has
    # seen the edge and released, so the pull-up holds the net high from then on.
    deselected = [i for i in range(1, len(trace)) if trace[i][CS_N] != CS_ACTIVE]
    assert len(deselected) > 2                       # there is an idle gap to look at
    assert all(trace[i][MISO] == 1 for i in deselected[1:])


def test_fill_byte_is_driven_while_reading():
    fill = 0x5A
    bus = spi_bus(trace=True)
    ctl = SpiController(mode=0, half_period=4, fill=fill)
    mon = SpiMonitor(mode=0)
    bus.attach(ctl)
    bus.attach(SpiFlashTarget(mode=0))
    bus.attach(mon)
    ctl.add([CMD_JEDEC], 4)
    ctl.run(bus)
    assert mon.transfers[0].mosi == [CMD_JEDEC] + [fill] * 4
    assert ctl.tx == [CMD_JEDEC]                  # `fill` is not part of what was queued

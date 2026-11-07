"""End-to-end check of fw/onewire on the ISS: program, driver, chip and bus.

The judge here is not the driver: it is `verif.vip.onewire`, which is the only thing that
looks at the bus. `Ds18b20Target` is the slave -- it answers a reset with a presence pulse,
decodes Skip ROM / Match ROM / Read ROM and the function commands, and drives read slots --
and `OneWireTimingChecker` decodes the traffic and checks every timing rule of the DS18B20
tables at standard speed. Neither is edited and neither is softened: a violation fails the
test.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`; the 1-Wire line is wired straight to `p<dq_pin>` (or
wherever a configuration puts it). That is the same wiring the RTL will have: the net is
open drain and pulled up, every device drives 0 or releases, so a PIO that ever drove a
hard 1 would show up as a bus that never rises -- and `Bench`'s open-drain watcher checks
exactly that, on the chip's own level register, every tick.

The chip is the **master** (the model is the DS18B20), so what is measured here is the
firmware's own waveform: tRSTL and tRSTH around the reset, tLOW0 for the long low of a
write-0, tLOW1 for the short low of a write-1, tSLOT for the period, tREC between slots,
tRDV around the sample point, and the framing rules (a reset nobody answered, a ROM
command that is not one of the five, a slot before any reset, a CRC).

`FW_TARGET=iss` runs it against `iss.pio_iss.Chip`; `FW_TARGET=rtl` runs the same bench
against `src/pio_chip.sv` under cocotb, one core cycle per tick (`fw/rtl_harness.py`).
"""
from __future__ import annotations

import pytest

from fw.onewire import (
    PRESENCE_WINDOW,
    SLOT_PERIOD,
    TICK_NS,
    OneWireDriver,
    OneWireError,
    assemble_program,
    divider_window,
    layout,
    limits,
    reset_layout,
    rx_byte,
    tx_word,
)
from fw.onewire import driver as drv
from iss import pio_iss
from verif.vip.core import Bus
from verif.vip.onewire import (
    crc8,
    Ds18b20Target,
    OneWireTimingChecker,
    rom_from_serial,
)

SERIAL = b"\x11\x22\x33\x44\x55\x66"

# The three configurations the tests run at, all at 20 ns/tick: the nominal divider, the
# fastest one the DS18B20 tables allow, and the slowest one on a different pin. The two
# ends of the divider window are the interesting pair -- they are the same program and the
# same checker, only the divider differs.
NOMINAL = dict(clkdiv=50, dq_pin=0)
FASTEST = dict(clkdiv=47, dq_pin=0)
SLOWEST = dict(clkdiv=58, dq_pin=4)
CONFIGS = (NOMINAL, FASTEST, SLOWEST)

# Measured on the ISS by `python -m fw.onewire.measure` (the `max speed` rows of
# `metrics/fw.csv`). `divider_window()` derives the same pair from the
# DS18B20 tables and `test_the_divider_window_is_where_the_tables_say` re-measures both ends
# on the bus, so these two cannot drift apart silently.
FASTEST_CLKDIV, SLOWEST_CLKDIV = divider_window()
FASTEST_SLOT_HZ = limits(FASTEST_CLKDIV)["slot_rate_hz"]
FASTEST_BYTE_HZ = limits(FASTEST_CLKDIV)["byte_rate_hz"]
FASTEST_SLOT_US = limits(FASTEST_CLKDIV)["slot_ns"] / 1000.0


class PinWatch:
    """A passive device that records what the chip did to the 1-Wire pin, every tick.

    Two things come out of it. First, the open-drain discipline: 1-Wire may only pull the
    line low or release it, and the level register of the pin has to stay at its reset value
    0 (docs/mvp-isa.md `### Open drain`). A firmware that wrote a 1 there -- which is what
    an RP2040 one-wire program does through `gpio_inv`, and what this MVP has no way to do --
    would still look legal on the model's net, because the wired-AND resolves to 0 while it
    drives, but it would break a bus with anything else on it. Second, the waveform itself:
    the tick of every fall and every rise of the pin, which is what the slot period, the
    write-0 low, the write-1 low, tREC and the reset pulse are measured from here, straight
    off the chip's own registers rather than off the checker's opinion of them.

    The chip device steps before this one is called (it is attached after it), so what is
    read here is the state the engine wrote in this cycle. A pin write lands on the net one
    tick later, but every measurement below is a difference of two of these ticks, so the
    latency cancels.
    """

    name = "pin_watch"

    def __init__(self, chip: pio_iss.Chip, pin: int = 0):
        self.chip = chip
        self.pin = pin
        self.falls: list = []          # ticks the pin started being pulled low
        self.rises: list = []          # ticks it was released again
        self.driven_high: list = []    # ticks the level register was 1
        self._low = False

    def tick(self, t, pins):
        low = bool((self.chip.pin_oe >> self.pin) & 1)
        if (self.chip.pin_out >> self.pin) & 1:
            self.driven_high.append(t)
        if low and not self._low:
            self.falls.append(t)
        elif not low and self._low:
            self.rises.append(t)
        self._low = low
        return {}

    # ---- the measurements, in core ticks -------------------------------------------
    def periods(self) -> list:
        """Fall to fall: one entry per time slot, plus one for the reset pulse."""
        return [b - a for a, b in zip(self.falls, self.falls[1:])]

    def lows(self) -> list:
        """How long each pull low lasted: the reset pulse and every slot's low."""
        return [r - f for f, r in zip(self.falls, self.rises)]

    def highs(self) -> list:
        """How long the line was released between two pulls: tREC, and the gap before the
        presence pulse and after it."""
        return [fall - rise for rise, fall in zip(self.rises, self.falls[1:])]


class Bench:
    """A bus with the chip, one or two DS18B20s and the timing checker on it."""

    def __init__(self, clkdiv=50, dq_pin=0, serials=(SERIAL,), speed="standard",
                 check_timing=None, watch=True):
        self.dq_pin = dq_pin
        self.net = f"p{dq_pin}"
        self.bus = Bus(ns_per_tick=TICK_NS)
        for k in range(pio_iss.NUM_PINS):                 # p0..p17, all pulled up
            self.bus.add_net(f"p{k}")
        self.chip = pio_iss.Chip()
        self.device = pio_iss.ChipDevice(self.chip)
        self.bus.attach(self.device)
        # The models look at the bus through the chip's own pin nets, so there is no
        # coupling device in between and no extra tick of latency anywhere.
        self.targets = [
            self.bus.attach(Ds18b20Target(bus=self.bus, speed=speed, net=self.net,
                                          serial=serial))
            for serial in serials
        ]
        self.checker = self.bus.attach(OneWireTimingChecker(bus=self.bus, speed=speed,
                                                           net=self.net))
        self.watch = None
        if watch:
            self.watch = self.bus.attach(PinWatch(self.chip, dq_pin))
        window = divider_window()
        self.driver = OneWireDriver(
            self.bus, self.device, assemble_program(), clkdiv=clkdiv, dq_pin=dq_pin,
            check_timing=window[0] <= clkdiv <= window[1] if check_timing is None
            else check_timing,
        )
        self.clkdiv = clkdiv

    def start(self) -> "Bench":
        self.driver.load()
        self.driver.clear_fdebug()
        return self

    # ---- what the checker and the chip saw -----------------------------------------
    def decoded(self) -> list:
        """The bytes on the bus, as (value, group, direction) tuples."""
        return [(t.value, t.group, t.direction) for t in self.checker.transfers]

    def close(self) -> list:
        """Finish the trace and return the violations. `finish()` is what judges a trace
        that ends mid-byte, so every assertion on the checker calls it."""
        return self.checker.finish()

    def clean(self) -> None:
        assert self.checker.violations == [], self.checker.report()
        assert self.bus.errors == [], f"contention on {self.bus.errors[:3]}"
        if self.watch is not None:
            assert self.watch.driven_high == [], (
                f"the chip drove dq high on {len(self.watch.driven_high)} ticks, e.g. at "
                f"{self.watch.driven_high[:5]}: an open-drain line may only be pulled low "
                "or released (docs/mvp-isa.md `### Open drain`)"
            )

    def report(self) -> str:
        return self.checker.report()

    def host_flags(self) -> dict:
        """FDEBUG with the engine parked, which is when TXSTALL is the expected one.

        The program blocks on `pull block` between every byte, so a TXSTALL here means the
        host was late, not that anything is wrong; the other three are the host's own
        mistakes (RXSTALL: the RX FIFO filled, TXOVER: the TX FIFO overflowed, RXUNDER: a
        read of an empty FIFO) and must all be clear.
        """
        self.driver.wait_boundary()
        return self.driver.stall_flags()

# ---- the headline case: Skip ROM + Read Scratchpad, judged by the wire ----------------
def test_read_scratchpad_at_the_nominal_divider():
    """reset, Skip ROM [CCh], Read Scratchpad [BEh], nine bytes back, at CLKDIV 50.

    25.0625 C is 0x0191 in the temperature register, and the ninth byte is the CRC of the
    other eight, so this exercises the whole path: the reset and its presence check, two
    write bytes (which contain both a 1-bit and a 0-bit slot), nine read bytes in which the
    slave drives the line low for every 0-bit, and the checker's CRC rule on top.
    """
    bench = Bench(**NOMINAL).start()
    bench.targets[0].set_temperature(0x0191)
    ds = bench.targets[0]

    scratchpad = bench.driver.read_scratchpad()

    assert scratchpad == bytes(ds.scratchpad), (scratchpad.hex(), bytes(ds.scratchpad).hex())
    assert scratchpad[:2] == b"\x91\x01", "byte 0 is the temperature LSB, byte 1 the MSB"
    assert crc8(scratchpad[:8]) == scratchpad[8], "the scratchpad CRC covers bytes 0..7"
    assert [v for _, v in ds.rom_commands] == [0xCC], "the slave saw Skip ROM"
    assert [v for _, v in ds.function_commands] == [0xBE], "and then Read Scratchpad"
    assert ds.resets == 1 and ds.converting is False

    # And the bus monitor, which is what actually judges the timing, is happy.
    bench.close()
    bench.clean()
    assert (bench.checker.resets, bench.checker.presences) == (1, 1)
    assert (bench.checker.write_slots, bench.checker.read_slots) == (16, 72)
    assert bench.checker.crc_checks == 1, "the 9 scratchpad bytes were CRC checked"
    assert bench.decoded()[:3] == [(0xCC, "rom_command", "write"),
                                   (0xBE, "function", "write"),
                                   (0x91, "scratchpad", "read")]

    # What the chip itself put on the pin, measured off its own registers.
    where = limits(NOMINAL["clkdiv"])
    watch = bench.watch
    lows, periods = watch.lows(), watch.periods()
    reset_low = max(lows)
    assert reset_low == pytest.approx(where["reset_low_ns"] / TICK_NS, rel=0.02), (
        f"the reset pulse was {reset_low * TICK_NS / 1000.0:.1f} us, "
        f"{where['reset_low_ns'] / 1000.0:.1f} us expected")
    assert where["reset_low_ns"] / 1000.0 == pytest.approx(547.0), "the pulse is 547 us"
    slot_lows = [n for n in lows if n != reset_low]
    assert min(slot_lows) == pytest.approx(where["low1_ns"] / TICK_NS, abs=2), (
        f"the short low is {min(slot_lows)} ticks")
    assert max(slot_lows) == pytest.approx(where["low0_ns"] / TICK_NS, abs=2), (
        f"the write-0 low is {max(slot_lows)} ticks")
    # periods()[0] is not a slot: it is the reset pulse's fall to the first slot's fall,
    # 1171 SM cycles of it, which is the pulse plus the presence sample plus the tRSTH wait.
    # The checker drops that one too (`OneWireTimingChecker._drop_reset_fall_violations`).
    slots = periods[1:]
    assert len(slots) == 87, "88 slots, so 87 fall-to-fall periods between them"
    # Two periods on the whole bus and no third: 75 SM cycles inside a byte, and 77 from one
    # byte's first slot to the next byte's. The extra two are the `pull block` between them
    # and the one SM cycle it stalls for on a write byte, where the host pushes the next word
    # only after the byte it just sent went out (`word_period` and `word_period_stalled`).
    clkdiv = NOMINAL["clkdiv"]
    assert sorted(set(slots)) == [SLOT_PERIOD * clkdiv,
                                  (SLOT_PERIOD + 2) * clkdiv], sorted(set(slots))
    assert min(slots) >= 3000 and max(slots) <= 6000, "every slot period is in tSLOT"
    assert min(watch.highs()) >= 50, "tREC: at least 1 us released between two slots"

    # The host kept both FIFOs flowing and never over- or under-flowed one. TXSTALL is the
    # program's own `pull block` between bytes, which is how the host paces it.
    flags = bench.host_flags()
    assert not flags["rx_stall"] and not flags["tx_over"] and not flags["rx_under"], flags
    assert bench.driver.peak_fifo_depth < drv.FIFO_DEPTH, "the FIFOs never filled"


@pytest.mark.parametrize("config", CONFIGS, ids=lambda c: f"clkdiv{c['clkdiv']}p{c['dq_pin']}")
def test_read_rom_at_three_configurations(config):
    """Read ROM [33h] at the nominal divider, at the fastest and at the slowest legal one.

    Three configurations of the same program: CLKDIV 50 on pin 0, CLKDIV 47 (the fastest
    the DS18B20 tables allow, 70.5 us slots) and CLKDIV 58 on pin 4 (the slowest, 87 us
    slots). Nothing in the driver knows the divider or the pin, which is the point: the
    program addresses the pin through PINCTRL and takes its timing from the divider, so the
    same words and the same transactions run at all three.
    """
    bench = Bench(**config).start()
    ds = bench.targets[0]

    rom = bench.driver.read_rom()

    assert rom == ds.rom, (rom.hex(), ds.rom.hex())
    assert rom[0] == 0x28, "the family code is the first byte on the wire"
    assert crc8(rom[:7]) == rom[7], "the ROM CRC covers the other seven bytes"
    bench.close()
    bench.clean()
    assert bench.checker.crc_checks == 1 and bench.checker.presences == 1
    assert bench.decoded() == [(0x33, "rom_command", "write")] + [
        (b, "rom", "read") for b in ds.rom]
    # The slot rate the chip actually produced, off the pin.
    where = limits(config["clkdiv"])
    assert min(bench.watch.periods()[1:]) == pytest.approx(
        SLOT_PERIOD * config["clkdiv"], abs=2), bench.report()
    assert where["slot_ns"] / 1000.0 == pytest.approx(
        SLOT_PERIOD * config["clkdiv"] * TICK_NS / 1000.0)


def test_match_rom_addresses_exactly_one_of_two_slaves():
    """Two DS18B20s on the line: Match ROM [55h] picks one, and the other stays quiet.

    This is the transaction that needs a ROM code on the wire, so it is the one that shows
    the write path carrying the right bits in the right order: the eight ROM bytes go out
    least significant bit first, and if they went out the other way round no slave would
    match and the scratchpad read would come back empty.
    """
    other = b"\xAA\xBB\xCC\xDD\xEE\xFF"
    bench = Bench(**NOMINAL, serials=(SERIAL, other)).start()
    first, second = bench.targets
    assert first.rom != second.rom and second.rom[0] == 0x28

    bench.driver.match_rom(first.rom)
    bench.driver.write((0xBE,))                  # Read Scratchpad, to that one slave
    scratchpad = bench.driver.read(9)

    assert scratchpad == bytes(first.scratchpad)
    assert first.match_failures == 0 and second.match_failures == 1, (
        "the second slave saw a ROM code that was not its own and went quiet")
    assert second.function_commands == [], "and never saw a function command"
    bench.close()
    bench.clean()


def test_write_scratchpad_copy_and_recall():
    """Write Scratchpad [4Eh] with TH, TL and the configuration byte, then Copy [48h] and
    Recall E2 [B8h]: three bytes the datasheet insists are written before the next reset,
    and the register contents that proves each of the three commands arrived."""
    bench = Bench(**NOMINAL).start()
    ds = bench.targets[0]
    driver = bench.driver
    assert ds.th == 0x4B and ds.tl == 0x00 and ds.config == 0x7F, "the power-on values"

    assert driver.write_scratchpad(0x50, 0x10, 0x3F)
    assert (ds.th, ds.tl) == (0x50, 0x10), "TH and TL took"
    assert ds.config == 0x3F, (
        "the configuration register masks its don't-care bits: bits 0-4 always read 1 and "
        "bit 7 always reads 0, so the R1:R0 that was written is what is left")
    assert ds.write_log and ds.write_log[-1][1:] == (0x50, 0x10, 0x3F), "all three bytes"
    assert ds.eeprom == bytearray((0x50, 0x10, 0x3F)), (
        "the write reached the slave's EEPROM as well, which is what Copy and Recall are "
        "about: they are how TH and TL survive a power cycle")

    # Wreck the scratchpad behind the slave's back and recall it out of the EEPROM. Recall
    # comes before Copy on purpose: this model's Copy writes the *current* scratchpad into
    # the EEPROM (scratchpad bytes 2..4, as the datasheet says), so copying first would
    # overwrite the very thing the recall is meant to fetch.
    ds.th, ds.tl, ds.config = 0x00, 0x00, 0x1F
    ds._refresh_scratchpad()
    assert ds.scratchpad[2] == 0x00, "the scratchpad is the broken one now"
    assert driver.recall_e2()
    assert (ds.th, ds.tl, ds.config) == (0x50, 0x10, 0x3F), "Recall E2 fetched them"
    assert ds.scratchpad[2] == 0x50, "and the scratchpad followed"
    assert driver.copy_scratchpad()
    assert ds.eeprom == bytearray((0x50, 0x10, 0x3F)), "Copy wrote them back"
    bench.close()
    bench.clean()
    assert [v for _, v in ds.function_commands] == [0x4E, 0xB8, 0x48]
    flags = bench.host_flags()
    assert not flags["rx_stall"] and not flags["tx_over"], flags


def test_the_divider_window_is_where_the_tables_say():
    """The CLKDIV window, measured on the bus at both ends and just outside each.

    The window is not a choice: every timing in the program is a whole number of SM cycles,
    so each DS18B20 window becomes a range of CLKDIV and the program is legal exactly where
    they overlap. At CLKDIV 47 the reset pulse is 514 us (tRSTL allows 480..640) and the
    write-0 low is 61.1 us (tLOW0 allows 60..120); at 46 the write-0 low drops to 59.8 us
    and the checker says so. At 58 the reset pulse is 635 us and at 59 it is 646 us, past
    AN126 H's 640 us maximum. Nothing else about those runs is wrong: the bytes still come
    back byte-exact.
    """
    fastest, slowest = divider_window()
    assert (fastest, slowest) == (FASTEST_CLKDIV, SLOWEST_CLKDIV)
    measured = {}
    for clkdiv, expect_rules in ((fastest - 1, {"tLOW0"}), (fastest, set()),
                                 (slowest, set()), (slowest + 1, {"tRSTL"})):
        bench = Bench(clkdiv=clkdiv).start()
        bench.driver.skip_rom(0xBE)                  # two write bytes, one of each bit value
        first = bench.driver.read(1)                 # and a read byte, to sample the line
        violations = bench.close()
        rules = {v.rule for v in violations}
        assert first == bytes(bench.targets[0].scratchpad)[:1], (
            f"CLKDIV {clkdiv} still reads back correctly")
        assert rules == expect_rules, (
            f"CLKDIV {clkdiv}: rules {sorted(rules)}, expected {sorted(expect_rules)}: "
            + bench.report())
        if not expect_rules:
            bench.clean()          # the off-spec dividers are meant to have violations
        where = limits(clkdiv)
        # The slot period the chip produced, off the pin, against what the divider says.
        assert min(bench.watch.periods()[1:]) == pytest.approx(
            SLOT_PERIOD * clkdiv, abs=2), bench.report()
        measured[clkdiv] = (where["slot_rate_hz"], where["reset_low_ns"] / 1000.0,
                            where["low0_ns"] / 1000.0)

    assert measured[fastest][1] == pytest.approx(514.1, rel=0.01), "the fastest reset pulse"
    assert measured[slowest][1] == pytest.approx(634.5, rel=0.01), "the slowest legal one"
    assert measured[fastest][0] == pytest.approx(FASTEST_SLOT_HZ)
    assert FASTEST_SLOT_HZ == pytest.approx(1e9 / (SLOT_PERIOD * fastest * TICK_NS))
    assert FASTEST_BYTE_HZ == pytest.approx(FASTEST_SLOT_HZ / 8), "eight slots per byte"
    assert FASTEST_SLOT_US == pytest.approx(70.5), "the fastest legal slot is 70.5 us"


def test_a_bus_with_no_slave_is_reported_as_absent():
    """Nothing on the line: the master still runs, and the presence check says so.

    This is what the presence sample in the reset is for. The datasheet has the master
    check for a presence pulse before it sends a ROM command, and the MVP has no IRQ and no
    status bit for a pin, so the program samples the line itself and pushes the bit into the
    RX FIFO. With no slave the bit reads high, `reset()` returns False, and the passive
    checker reports exactly the one rule that is broken -- nobody answered the reset.
    """
    bench = Bench(**NOMINAL, serials=()).start()
    present = bench.driver.reset()
    assert present is False, "nobody answered the reset"
    bench.driver.write((0xCC,))
    assert bench.driver.read(1) == b"\xff", (
        "with nobody on the line every read bit is a 1, which is what an idle bus reads")
    violations = bench.close()
    # Three rules, all of them the same fact seen from three sides: nobody answered the
    # reset (so the first slot of the transaction is also read as a presence pulse that
    # arrived far too late), and the byte the master then read back is all 1s, which is not
    # a function command.
    assert {v.rule for v in violations} == {"presence", "tPDHIGH", "rom-command"}, [
        str(v) for v in violations]
    assert bench.checker.presences == 0
    assert bench.bus.errors == [], "an idle bus cannot have contention on it"
    assert bench.watch.driven_high == [], "the master released the line, it never drove it"


# ---- what the program and the driver are made of ------------------------------------
def test_the_program_fits_the_instruction_memory():
    program = assemble_program()
    assert len(program.instrs) <= 32, (
        f"{len(program.instrs)} words, an engine has 32 (docs/mvp-isa.md `### Engine state`)")
    assert program.wrap_target == program.labels["init"], "the wrap target is the park"
    assert program.wrap == program.labels["next_byte"], "the wrap top is the last instruction"
    assert program.sideset_bits == 0, (
        "no side-set: the open-drain line is a direction written by `set pindirs`, and a "
        "side-set would take a delay bit away from every instruction")
    # The layout walk agrees with the constants the driver and the docs quote, and the
    # driver refuses a program that does not.
    lay = layout(program)
    rst = reset_layout(program)
    assert (lay["len_w_one"], lay["len_w_zero"]) == (71, 71), (
        "a write-1 slot and a write-0 slot have to be the same length in SM cycles, or "
        "tSLOT measures two periods on one bus")
    assert (lay["prefix"], lay["sample"], lay["period"], lay["word_period"]) == (3, 25, 75, 76)
    assert (lay["low_short"], lay["low_long"]) == (6, 65), "tLOW1 and tLOW0 in SM cycles"
    # program.pio as written is the CLKDIV 50 program: its presence pad is the 69 SM
    # cycles `presence_cycles(50)` asks for, and `retarget` is the identity at 50.
    assert (rst["low"], rst["sample"], rst["to_slot"]) == (
        547, drv.presence_cycles(drv.DEFAULT_CLKDIV), 623)
    assert drv.retarget(program, drv.DEFAULT_CLKDIV).instrs == program.instrs
    assert rst["pushed"], "the presence bit is pushed for the host"
    drv.check_program(program)
    # Assembling is deterministic: the driver loads whatever is in the file, so a program
    # that assembled differently twice would make every number above a coin toss.
    assert assemble_program().instrs == program.instrs


def test_the_driver_refuses_a_program_whose_layout_moved():
    """`check_program` is the guard between program.pio and everything that quotes its
    numbers. Moving one delay has to be caught here rather than as a puzzling timing
    failure on the bus later, so each of these edits has to be refused."""
    program = assemble_program()

    def edited(addr: int, delay: int):
        words = list(program.instrs)
        words[addr] = (words[addr] & ~(0x1F << 8)) | (delay << 8)
        return drv.Program(name=program.name, instrs=words, origin=program.origin,
                           wrap_target=program.wrap_target, wrap=program.wrap,
                           sideset_bits=program.sideset_bits, sideset_opt=program.sideset_opt,
                           sideset_pindirs=program.sideset_pindirs, labels=program.labels)

    def moved(shorten: int, lengthen: int):
        """A delay moved inside one branch, so the branch stays the same length and only
        the thing under test changes."""
        words = list(program.instrs)
        for addr, by in ((shorten, -1), (lengthen, 1)):
            delay = ((words[addr] >> 8) & 0x1F) + by
            words[addr] = (words[addr] & ~(0x1F << 8)) | (delay << 8)
        return drv.Program(name=program.name, instrs=words, origin=program.origin,
                           wrap_target=program.wrap_target, wrap=program.wrap,
                           sideset_bits=program.sideset_bits,
                           sideset_opt=program.sideset_opt,
                           sideset_pindirs=program.sideset_pindirs, labels=program.labels)

    one, zero = program.labels["w_one"], program.labels["w_zero"]
    with pytest.raises(drv.OneWireError, match="write-1 low"):
        drv.check_program(moved(one, one + 5), drv.DEFAULT_CLKDIV)  # short low, same length
    with pytest.raises(drv.OneWireError, match="write-0 low"):
        drv.check_program(moved(zero, zero + 3), drv.DEFAULT_CLKDIV)
    with pytest.raises(drv.OneWireError, match="sample point"):
        drv.check_program(moved(one + 2, one + 5), drv.DEFAULT_CLKDIV)  # the sample moves
    # A branch that is 1 SM cycle longer makes the two slots different lengths.
    with pytest.raises(drv.OneWireError, match="same length"):
        drv.check_program(edited(one + 1, 4), drv.DEFAULT_CLKDIV)
    # And a byte boundary that is not `pull block` leaves the host nothing to sync to.
    words = list(program.instrs)
    words[program.labels["w_byte"]] = 0x8080            # `pull noblock`
    unblocked = drv.Program(name=program.name, instrs=words, origin=program.origin,
                            wrap_target=program.wrap_target, wrap=program.wrap,
                            sideset_bits=program.sideset_bits,
                            sideset_opt=program.sideset_opt,
                            sideset_pindirs=program.sideset_pindirs, labels=program.labels)
    with pytest.raises(drv.OneWireError, match="pull block"):
        drv.check_program(unblocked, drv.DEFAULT_CLKDIV)
    # The presence pad is the divider's: an un-retargeted program is only the nominal one.
    with pytest.raises(drv.OneWireError, match="presence sample"):
        drv.check_program(program, FASTEST_CLKDIV)
    for clkdiv in (FASTEST_CLKDIV, drv.DEFAULT_CLKDIV, SLOWEST_CLKDIV):
        drv.check_program(drv.retarget(program, clkdiv), clkdiv)
    # And a divider outside the tables is refused, unless the caller says it means it.
    for outside in (FASTEST_CLKDIV - 1, SLOWEST_CLKDIV + 1):
        patched = drv.retarget(program, outside)
        with pytest.raises(drv.OneWireError, match="t(RSTL|LOW0)"):
            drv.check_program(patched, outside)
        drv.check_program(patched, outside, timing=False)
    # Retargeting only ever moves the presence pad: the three `nop`s between the reset's
    # release and its `in pins, 1`, which is what `retarget` finds for itself.
    fastest = drv.retarget(program, FASTEST_CLKDIV)
    moved = [k for k in range(len(program.instrs))
             if fastest.instrs[k] != program.instrs[k]]
    pad = range(program.labels["rst_low"] + 2, program.labels["rst_low"] + 5)
    assert moved and set(moved) <= set(pad), (moved, list(pad))
    assert all(drv.decode_word(fastest.instrs[k])[0] == "nop" for k in moved)
    # CLKDIV 47 needs 4 more SM cycles of pad than CLKDIV 50, and 58 needs 10 fewer, so the
    # pad really does move in both directions.
    assert drv.presence_cycles(FASTEST_CLKDIV) > drv.presence_cycles(drv.DEFAULT_CLKDIV)
    assert drv.presence_cycles(SLOWEST_CLKDIV) < drv.presence_cycles(drv.DEFAULT_CLKDIV)


def test_the_divider_window_arithmetic_is_the_timing_tables():
    """`divider_window()` is derived, not typed in: every datasheet window becomes a range
    of CLKDIV and the program is legal where they overlap. This pins the arithmetic, because
    the sweep above can only test the two ends."""
    fastest, slowest = divider_window()
    assert fastest == max(-(-480_000 // (drv.RESET_LOW * TICK_NS)),
                          -(-480_000 // (drv.RESET_TO_SLOT * TICK_NS)),
                          -(-60_000 // (drv.SLOT_LOW_LONG * TICK_NS)),
                          -(-60_000 // (drv.SLOT_PERIOD * TICK_NS))), "the fastest legal divider"
    assert slowest == min(640_000 // (drv.RESET_LOW * TICK_NS),
                          15_000 // (drv.SLOT_LOW_SHORT * TICK_NS),
                          120_000 // (drv.SLOT_LOW_LONG * TICK_NS),
                          120_000 // (drv.SLOT_PERIOD * TICK_NS)), "the slowest legal one"
    # The presence sample has to be where every legal presence pulse is still low: 60 us is
    # the latest a pulse may start (tPDHIGH max) and 75 us the earliest any may end
    # (tPDHIGH min + tPDLOW min). One SM cycle either side is divider phase.
    at_fastest = limits(fastest)["presence_sample_ns"]
    slack = fastest * TICK_NS
    assert PRESENCE_WINDOW[0] < at_fastest - slack and at_fastest + slack < PRESENCE_WINDOW[1], (
        f"the presence sample is {at_fastest / 1000:.1f} us at CLKDIV {fastest}, outside "
        f"{PRESENCE_WINDOW[0] / 1000:.0f}..{PRESENCE_WINDOW[1] / 1000:.0f} us")
    for clkdiv in (fastest, slowest):
        at = limits(clkdiv)["presence_sample_ns"]
        assert PRESENCE_WINDOW[0] < at - clkdiv * TICK_NS
        assert at + clkdiv * TICK_NS < PRESENCE_WINDOW[1]


def test_the_driver_addresses_match_the_register_map_and_the_iss():
    """The driver is written against docs/mvp-regmap.md; this is the guard against the two
    drifting apart, in particular against the ISS renumbering something."""
    assert (drv.A_CTRL, drv.A_FSTAT, drv.A_FDEBUG, drv.A_FLEVEL) == (
        pio_iss.A_CTRL, pio_iss.A_FSTAT, pio_iss.A_FDEBUG, pio_iss.A_FLEVEL)
    assert (drv.A_TXF0, drv.A_RXF0) == (pio_iss.A_TXF[0], pio_iss.A_RXF[0])
    assert drv.A_IMEM == pio_iss.A_IMEM
    assert (drv.A_DBG_PADOUT, drv.A_DBG_PADOE, drv.A_DBG_CFGINFO) == (
        pio_iss.A_DBG_PADOUT, pio_iss.A_DBG_PADOE, pio_iss.A_DBG_CFGINFO)
    assert (drv.SM_CLKDIV, drv.SM_EXECCTRL, drv.SM_SHIFTCTRL, drv.SM_ADDR,
            drv.SM_INSTR, drv.SM_PINCTRL) == (
        pio_iss.SM_CLKDIV, pio_iss.SM_EXECCTRL, pio_iss.SM_SHIFTCTRL, pio_iss.SM_ADDR,
        pio_iss.SM_INSTR, pio_iss.SM_PINCTRL)
    assert (drv.FLEVEL_TX0, drv.FLEVEL_RX0, drv.FIFO_DEPTH) == (0, 4, pio_iss.FIFO_DEPTH)


def test_the_word_packing_round_trips():
    for byte in (0x00, 0x01, 0x33, 0x55, 0xA5, 0xCC, 0xFF):
        word = tx_word(byte)
        assert word == ~byte & 0xFF, "the OSR holds directions, so the byte is inverted"
        assert word & ~0xFF == 0, "nothing outside the eight slots of the byte"
        # A 1-bit on the wire is a released line, which is a 0 in the word.
        assert [bit for bit in range(8) if (word >> bit) & 1] == [
            bit for bit in range(8) if not (byte >> bit) & 1]
    assert drv.read_word() == 0, "a read word releases every slot of the byte"
    with pytest.raises(ValueError):
        drv.read_word(7)
    for value in (-1, 0x100):
        with pytest.raises(ValueError):
            tx_word(value)
    # A sampled byte is right-aligned in bits 31:24 and the presence bit is bit 31,
    # because the ISR shifts right. Bit 24 is the first bit on the wire, and 1-Wire sends a
    # byte least significant bit first, so this is the byte itself and not its reverse.
    assert rx_byte(0x28 << drv.RX_BYTE_SHIFT) == 0x28 and rx_byte(0x9E << 24) == 0x9E
    assert rx_byte(0xFFFFFFFF) == 0xFF and rx_byte(0) == 0x00
    for value in (0x28, 0x01, 0x9E, 0xFF, 0x00):
        word = 0
        for bit in range(8):
            word = (word >> 1) | (((value >> bit) & 1) << 31)
        assert rx_byte(word) == value, f"0x{value:02x} survives the shift as itself"
        assert int(f"{value:08b}"[::-1], 2) != value or value in (0x00, 0xFF), (
            "and is not its own bit reversal, which is what IN_SHIFTDIR 0 would have given")
    assert drv.jmp_word(31) == 31 and drv.jmp_word(0) == 0
    # `mov isr, null`: MOV, destination ISR, no op, source NULL.
    mov = drv.mov_isr_null_word()
    assert (mov >> 13) == 5 and ((mov >> 5) & 7) == 6 and (mov & 7) == 3


def test_open_drain_needs_a_uio_pin():
    """Open drain is a direction, and only the bidirectional uio pins have one
    (docs/mvp-isa.md `### Open drain`, `### The 18 protocol pins`)."""
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus.attach(device)
    program = assemble_program()
    for pin in (-1, 8, 13, 32):
        with pytest.raises(ValueError, match="uio pin"):
            OneWireDriver(bus, device, program, dq_pin=pin)
    # Pins 0..7 all work, and the engine 1 register block is the other half of the chip.
    for pin in range(8):
        OneWireDriver(bus, device, program, dq_pin=pin, engine=pin % 2)

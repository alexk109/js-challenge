"""End-to-end check of fw/i2c on the ISS: program, driver, chip and bus.

The judge here is not the driver: it is `verif.vip.i2c`, which is the only thing that
looks at the bus. `I2cEepromTarget` (with clock stretching switched on) is a 24C02
that NACKs every address but its own and keeps a memory image, and
`I2cTimingChecker` decodes the traffic and checks every minimum of the chosen mode.
Neither is edited and neither is softened: a violation fails the test.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`; the two I2C lines are wired straight to `p0` and `p1`
(or wherever the configuration puts them). That is the same wiring the RTL will
have: the nets are open drain, pulled up, and every device drives 0 or releases, so a
PIO that ever drove a hard 1 would show up as a bus that never rises.

`FW_TARGET=iss` runs it against `iss.pio_iss.Chip`; `FW_TARGET=rtl` runs the same bench
against `src/pio_chip.sv` under cocotb, one core cycle per tick (`fw/rtl_harness.py`).
"""
from __future__ import annotations

import pytest


from fw.i2c import I2cDriver, I2cError, assemble_program
from fw.i2c import driver as drv
from iss import pio_iss
from verif.vip.core import Bus
from verif.vip.i2c import I2C_MODES, I2cEepromTarget, I2cTimingChecker, ns_to_ticks
from fw import regmap as _regmap

TICK_NS = 20.0                    # 50 MHz, the clock the timing minimums are quoted at
DEFAULT_PINS = (drv.DEFAULT_SDA_PIN, drv.DEFAULT_SCL_PIN)

# Two configurations, both far enough below their nominal rate that the minimums are
# met with room to spare: fast mode at CLKDIV 14 (9 SM cycles per bit, of which
# 3 are the SCL high phase and 7 the low one) and standard mode at CLKDIV 67.
FAST = dict(mode="fast", clkdiv=14, stretch_ticks=12)
STANDARD = dict(mode="standard", clkdiv=67, stretch_ticks=4)

# The firmware's max speed, measured by `python -m fw.i2c.measure`: the fastest
# CLKDIV whose transfer I2cTimingChecker accepts in fast mode with zero violations, and the
# SCL rate it puts on the bus. Because the checker checks fSCL (an earlier version had no maximum at
# all, so this used to read 478.7 kbit/s at CLKDIV 10, above the 400 kHz of fast mode), the
# answer is CLKDIV 13: 130 core cycles of SCL period, 384.6 kbit/s. CLKDIV 12 is one
# period too fast -- 120 cycles, 416.7 kbit/s -- and nothing else about it is wrong.
MAX_CLEAN_CLK_DIV = 13
MAX_CLEAN_SCL_TICKS = 130               # the shortest SCL period that divider puts out
MAX_CLEAN_BIT_RATE_HZ = 1e9 / (TICK_NS * MAX_CLEAN_SCL_TICKS)     # 384.6 kbit/s


class Bench:
    """A bus with the chip, an EEPROM and the timing checker on it."""

    def __init__(self, mode="fast", clkdiv=14, stretch_ticks=0, address=0x50,
                 sda_pin=DEFAULT_PINS[0], scl_pin=DEFAULT_PINS[1]):
        self.mode = mode
        self.sda_pin, self.scl_pin = sda_pin, scl_pin
        self.bus = Bus(ns_per_tick=TICK_NS)
        for k in range(pio_iss.NUM_PINS):                 # p0..p17, all pulled up
            self.bus.add_net(f"p{k}")
        self.chip = pio_iss.Chip()
        self.device = pio_iss.ChipDevice(self.chip)
        self.bus.attach(self.device)
        # The models look at the bus through the chip's own pin nets, so there is no
        # coupling device in between and no extra tick of latency anywhere.
        nets = dict(scl_net=f"p{scl_pin}", sda_net=f"p{sda_pin}")
        self.eeprom = I2cEepromTarget(address=address, mode=mode,
                                      stretch_ticks=stretch_ticks, **nets)
        self.checker = I2cTimingChecker(mode=mode, **nets)
        self.bus.attach(self.eeprom)
        self.bus.attach(self.checker)
        self.program = assemble_program()
        # The driver leaves the bus idle for this long after every STOP and after load.
        # It is the larger of tBUF and tHIGH because the checker's first SCL high period
        # is measured from tick 0 (`I2cTimingChecker`, "One caveat"), so the period the
        # bus has had since reset has to be a real one before the first forced
        # instruction pulls SCL low.
        idle_ticks = ns_to_ticks(TICK_NS, max(I2C_MODES[mode].t_buf, I2C_MODES[mode].t_high))
        self.driver = I2cDriver(
            self.bus, self.device, self.program, clkdiv=clkdiv,
            sda_pin=sda_pin, scl_pin=scl_pin, bus_free_ticks=idle_ticks,
        )

    def start(self) -> "Bench":
        self.driver.load()
        self.driver.clear_fdebug()
        return self

    # ---- what the checker saw ------------------------------------------------------
    def decoded(self) -> list:
        """The bytes on the bus, as (value, ack, is_address, rw) tuples."""
        return [(t.value, t.ack, t.is_address, t.rw) for t in self.checker.transfers]

    def bit_periods(self) -> list:
        """Average tick gap between consecutive bytes, in bit times.

        The checker timestamps a byte at its ACK clock, so the gaps between those are
        nine bit times; dividing by nine gives the average bit period of every byte on
        the bus. This is an average, not a minimum: the program runs one bit of a byte
        shorter than the others, so it understates the rate. `max_bit_rate` is the number
        a spec limits.
        """
        ticks = [t.tick for t in self.checker.transfers]
        return [(b - a) / 9.0 for a, b in zip(ticks, ticks[1:])]

    def max_bit_rate(self) -> float:
        """The fastest SCL the bus carried, in bit/s.

        That is the shortest SCL period the checker measured, which is what
        I2cTimingChecker's fSCL rule judges and what UM10204's fSCL maximum is about.
        (The byte-to-byte bit is the short one: at CLKDIV 14 the average over nine clocks
        is 146.2 core cycles but the shortest period is 140, 357.1 kbit/s.)
        """
        assert self.checker.shortest_period is not None, "the checker measured no period"
        return self.checker.max_scl_hz

    def report(self) -> str:
        return self.checker.report()


def no_violations(bench: Bench) -> None:
    assert bench.checker.violations == [], bench.report()


# ---- the headline case: write, repeated START, read, STOP, with stretching ---------
def test_write_then_repeated_start_read_in_fast_mode():
    bench = Bench(**FAST).start()
    eeprom, driver = bench.eeprom, bench.driver
    eeprom.memory[0x08:0x10] = bytes(range(0x10, 0x18))

    # One transaction: the word address 0x0C, one data byte written at 0x0C (which
    # leaves the EEPROM's counter at 0x0D), a repeated START back to address+R, three
    # bytes read from 0x0D, then a STOP.
    acked, read = driver.write_read(0x50, [0x0C, 0xC0], 3)

    assert bytes(eeprom.memory[0x0C:0x10]) == bytes([0xC0, 0x15, 0x16, 0x17])
    assert read == bytes([0x15, 0x16, 0x17])
    assert acked == [True, True], "the word address and the data byte were ACKed"

    # The slave agrees about what happened.
    assert (eeprom.starts, eeprom.stops) == (2, 1)
    assert [(a, v) for _, a, v in eeprom.write_log] == [(0x0C, 0xC0)]
    assert [a for _, a in eeprom.read_log] == [0x0D]
    assert eeprom.nacked_addresses == []

    # And so does the bus monitor, which is what actually judges the timing.
    no_violations(bench)
    assert (bench.checker.starts, bench.checker.repeated_starts, bench.checker.stops) == (2, 1, 1)
    assert bench.decoded() == [
        (0xA0, True, True, 0),      # address, write
        (0x0C, True, False, None),  # word address
        (0xC0, True, False, None),  # the byte written
        (0xA1, True, True, 1),      # repeated START, address, read
        (0x15, True, False, None),  # read back, ACKed
        (0x16, True, False, None),  # read back, ACKed
        (0x17, False, False, None),  # last byte, NACKed
    ]
    assert bench.checker.nacks == 1 and bench.checker.acks == 6
    # The clock really was stretched: without it no period would be longer than the rest.
    assert max(bench.bit_periods()) > 1.2 * min(bench.bit_periods())
    # 140 core cycles of SCL period is what CLKDIV 14 costs, 357.1 kbit/s, and the average
    # over a byte is 146.2 cycles because the byte-to-byte bit is the short one.
    assert bench.checker.shortest_period == 140, bench.report()
    assert bench.max_bit_rate() <= I2C_MODES["fast"].freq, bench.report()

    # The host kept both FIFOs flowing, so the engine never stalled on one.
    assert driver.fdebug() == 0, f"FDEBUG {driver.fdebug():#010x} after the transfer"


def test_slow_bit_rate_holds_the_mode_nominal():
    """The same program at CLKDIV 67: 74.6 kbit/s, still inside standard mode's 100."""
    bench = Bench(**STANDARD).start()
    driver = bench.driver
    driver.write_read(0x50, [0x00, 0x5A], 2)
    no_violations(bench)
    assert bench.eeprom.memory[0x00] == 0x5A
    rate = bench.max_bit_rate()
    assert bench.checker.shortest_period == 670, bench.report()
    assert rate <= I2C_MODES["standard"].freq, bench.report()
    assert 50e3 <= rate <= 100e3, f"{rate / 1e3:.1f} kbit/s is neither slow nor standard"


def test_other_pins_and_a_page_crossing_write():
    """A different pin pair and the standard-mode timings, checked against a second
    EEPROM address: a page write that rolls over inside the page, then a current-address
    read after a real STOP, which is the one path a repeated START does not cover."""
    bench = Bench(mode="standard", clkdiv=67, stretch_ticks=3, address=0x51,
                  sda_pin=4, scl_pin=5).start()
    driver, eeprom = bench.driver, bench.eeprom
    for i in range(16):
        eeprom.memory[i] = 0xA0 + i

    # Word address 0x0E in an 8-byte page: 0x0E, 0x0F, then the page rolls to 0x08, 0x09,
    # so the counter is left at 0x0A.
    driver.write(0x51, [0x0E, 0x11, 0x22, 0x33, 0x44])
    driver.stop()
    assert [(a, v) for _, a, v in eeprom.write_log] == [
        (0x0E, 0x11), (0x0F, 0x22), (0x08, 0x33), (0x09, 0x44)
    ]
    assert [eeprom.memory[a] for a in (0x08, 0x09, 0x0E, 0x0F)] == [0x33, 0x44, 0x11, 0x22]

    # A STOP, then a fresh START and a current-address read from wherever the counter is.
    read = driver.read(0x51, 3)
    assert read == bytes([0xAA, 0xAB, 0xAC])
    no_violations(bench)
    assert (bench.checker.starts, bench.checker.repeated_starts, bench.checker.stops) == (2, 0, 2)
    decoded = bench.decoded()
    assert decoded[0] == (0xA2, True, True, 0)      # address 0x51, write
    assert decoded[6] == (0xA3, True, True, 1)      # address 0x51, read
    assert [v for v, _, _, _ in decoded[7:]] == [0xAA, 0xAB, 0xAC]
    assert driver.fdebug() == 0, f"FDEBUG {driver.fdebug():#010x}"


# ---- what the program and the driver are made of ------------------------------------
def test_the_program_fits_the_instruction_memory():
    program = assemble_program()
    assert len(program.instrs) <= 32, (
        f"{len(program.instrs)} words, an engine has 32 (docs/mvp-isa.md "
        f"`### Engine state`)"
    )
    assert program.wrap_target == program.labels["main"], "the wrap target is main"
    assert program.wrap == program.labels["stop_end"], "the wrap top is the end of stop_seq"
    # The wrap window has to hold the program: running off either end lands on the idle
    # preamble or on main, never in the middle of a byte.
    assert 0 <= program.wrap_target <= program.wrap


def test_the_driver_addresses_match_the_register_map_and_the_iss():
    """The driver is written against docs/mvp-regmap.md; this is the guard against the
    two drifting apart, in particular against the ISS renumbering something."""
    assert (drv.A_CTRL, drv.A_FSTAT, drv.A_FDEBUG, drv.A_FLEVEL) == (
        pio_iss.A_CTRL, pio_iss.A_FSTAT, pio_iss.A_FDEBUG, pio_iss.A_FLEVEL
    )
    assert (drv.A_TXF0, drv.A_RXF0) == (pio_iss.A_TXF[0], pio_iss.A_RXF[0])
    if _regmap.active() is _regmap.MVP:          # the golden constants are the MVP map
        assert drv.A_IMEM == pio_iss.A_IMEM
    else:
        assert drv.A_IMEM == _regmap.imem_bases()
    assert (drv.A_DBG_PADOUT, drv.A_DBG_PADOE, drv.A_DBG_CFGINFO) == (
        pio_iss.A_DBG_PADOUT, pio_iss.A_DBG_PADOE, pio_iss.A_DBG_CFGINFO
    )
    assert (drv.SM_CLKDIV, drv.SM_EXECCTRL, drv.SM_SHIFTCTRL, drv.SM_ADDR,
            drv.SM_INSTR, drv.SM_PINCTRL) == (
        pio_iss.SM_CLKDIV, pio_iss.SM_EXECCTRL, pio_iss.SM_SHIFTCTRL, pio_iss.SM_ADDR,
        pio_iss.SM_INSTR, pio_iss.SM_PINCTRL
    )


def test_the_word_packing_round_trips():
    for byte in (0x00, 0x01, 0x55, 0xA5, 0xFF):
        word = drv.tx_word(byte)
        assert word >> drv.TX_DATA_SHIFT == (~byte & 0xFF), "the OSR holds directions"
        assert word & 0x7FFFFF == 0, "nothing outside the nine bit times"
        assert drv.read_word(ack=True) - drv.read_word() == 1 << drv.TX_NINTH_BIT
    assert drv.rx_byte(0x140) == 0xA0, "byte in bits 8:1, ninth bit in bit 0"
    assert drv.rx_ack(0x140) and not drv.rx_ack(0x141)
    for value in (-1, 0x100):
        with pytest.raises(ValueError):
            drv.tx_word(value)
    with pytest.raises(ValueError):
        drv.set_y_word(32)


def test_the_pins_have_to_be_a_pair_of_uio_pins():
    """Open drain is a direction, and only uio pins have one (docs/mvp-isa.md
    `### Open drain`), and the program drives both lines through one window."""
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    bus.attach(device)
    program = assemble_program()
    for sda, scl in ((0, 2), (3, 3), (0, 7), (0, 8)):
        with pytest.raises(ValueError):
            I2cDriver(bus, device, program, sda_pin=sda, scl_pin=scl)


def test_fastest_divider_that_still_meets_fast_mode():
    """How fast this program can go: the fastest CLKDIV whose transfer the fast-mode
    checker passes with no violations at all, which is the number reported for this firmware."""
    rates = []
    expect = bytes([0x22, 0x33, 0x44, 0x00])   # 0x00 was just written at 0x10
    for clkdiv in range(1, 25):
        bench = Bench(mode="fast", clkdiv=clkdiv, stretch_ticks=0, address=0x50)
        bench.eeprom.memory[0x10:0x14] = bytes([0x11, 0x22, 0x33, 0x44])
        bench.start()
        try:
            _, read = bench.driver.write_read(0x50, [0x10, 0x00], 4)
        except I2cError:
            continue                       # the slave could not answer that fast
        if bench.checker.violations or read != expect:
            continue
        rates.append((clkdiv, bench.max_bit_rate(), bench.checker.shortest_period))
    assert rates, "not one CLKDIV passed"
    fastest_divider, fastest_rate, fastest_ticks = rates[0]
    # What stops the program is fSCL: the half-period minimums of fast mode are 65 and 30
    # ticks, so a divider that keeps both halves legal can still run above the 400 kHz the
    # mode is named for. The recorded number is MAX_CLEAN_*, and it only holds while every
    # clean divider is at or under the mode's nominal rate.
    assert fastest_rate <= I2C_MODES["fast"].freq, (
        f"CLKDIV {fastest_divider} is clean at {fastest_rate / 1e3:.1f} kbit/s, over fast mode"
    )
    assert (fastest_divider, fastest_ticks, fastest_rate) == (
        MAX_CLEAN_CLK_DIV, MAX_CLEAN_SCL_TICKS, MAX_CLEAN_BIT_RATE_HZ
    ), (
        f"the max speed moved: {(fastest_divider, fastest_ticks, fastest_rate)}, recorded "
        f"{(MAX_CLEAN_CLK_DIV, MAX_CLEAN_SCL_TICKS, MAX_CLEAN_BIT_RATE_HZ)}; "
        f"re-run python -m fw.i2c.measure"
    )
    # The divider the tests run at has margin, and CLKDIV 12 is the one that fails only on
    # fSCL: 120 cycles of period, 416.7 kbit/s, with every other rule clean.
    assert FAST["clkdiv"] >= fastest_divider
    fast = Bench(mode="fast", clkdiv=MAX_CLEAN_CLK_DIV - 1, stretch_ticks=0, address=0x50)
    fast.eeprom.memory[0x10:0x14] = bytes([0x11, 0x22, 0x33, 0x44])
    fast.start()
    _, read = fast.driver.write_read(0x50, [0x10, 0x00], 4)
    assert read == expect, "CLKDIV 12 still reads back correctly"
    assert {v.rule for v in fast.checker.violations} == {"fSCL"}, fast.report()


def test_an_address_nobody_answers_is_reported():
    """The EEPROM NACKs every address but its own, and the driver has to notice."""
    bench = Bench(**FAST).start()
    driver = bench.driver
    with pytest.raises(I2cError, match="nothing ACKed the write address 0x52"):
        driver.write(0x52, [0x00])
    assert bench.eeprom.nacked_addresses, "the EEPROM saw the address and refused it"
    driver.wait_parked()
    no_violations(bench)

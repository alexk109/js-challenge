"""End-to-end check of fw/ps2 on the ISS: program, driver, chip and bus.

The judge here is not the driver: it is `verif.vip.ps2`, which is the only thing that looks
at the bus. `Ps2Keyboard` and `Ps2Mouse` are the devices the protocol's sources describe
(the OSDev command tables, the IBM AT frame table), and `Ps2Checker` is a passive monitor
that decodes every frame off the pins and checks every rule of the timing table -- the two
sided ones (30..50 us for both clock halves) as well as the framing rules (start, parity,
stop, the acknowledgement bit, the frame length) and `tBUS-FREE`. Neither the models nor the
checker are edited and neither is softened: a violation fails the test.

The chip sits on a `verif.vip.core.Bus` at 20 ns/tick through `iss.pio_iss.ChipDevice`,
whose nets are named `p0`..`p17`; the two PS/2 lines are wired straight to the two pins the
driver is configured for (or wherever the configuration puts them). That is the same wiring
the RTL will have: the nets are open drain, pulled up, and every device drives 0 or releases,
so a PIO that ever drove a hard 1 would show up as a bus that never rises.

The chip plays the **controller**: it generates the clock for the bytes it sends and samples
the clock the device generates for the bytes it receives (research/ref/protocols/ps2/README.md,
"Why PS/2").

`FW_TARGET=iss` runs it against `iss.pio_iss.Chip`; `FW_TARGET=rtl` runs the same bench
against `src/pio_chip.sv` under cocotb, one core cycle per tick (`fw/rtl_harness.py`).
"""
from __future__ import annotations

import pytest

from fw.ps2 import ACK, RESEND, assemble_program
from fw.ps2 import driver as drv
from fw.ps2.driver import Ps2Driver, Ps2Error
from iss import pio_iss
from verif.vip.core import Bus
from verif.vip.ps2 import (
    attach_checker,
    attach_device,
    attach_keyboard,
    attach_mouse,
    ns_to_ticks,
)

TICK_NS = 20.0                    # 50 MHz, the clock the timing limits are quoted against
DEFAULT_PINS = (drv.DEFAULT_CLK_PIN, drv.DEFAULT_DATA_PIN)

# Two configurations of one program, both far enough inside the 30..50 us of T3/T4 and
# T7/T8 to have margin: FAST is 34 us low and 32 us high (15.2 kbit/s, the top of the
# 10..16.7 kbit/s the sources give), SLOW is 47.6 and 44.8 us (10.8 kbit/s, near the
# bottom). Only CLKDIV differs; every delay is in the program.
FAST = dict(clkdiv=100)
SLOW = dict(clkdiv=140)

# The firmware's max speed, measured by `python -m fw.ps2.measure`: the fastest CLKDIV whose frames
# Ps2Checker accepts with zero violations, and the data bit rate it puts on the bus. What runs out
# first is the pair of 16-SM-cycle phases -- the high phase of every clock and the low phase
# before the acknowledgement clock -- while a data bit's low phase is a whole SM cycle
# longer and would still be legal. So the answer is the smallest CLKDIV whose short phases
# still clear the 30 us floor of T4.
MAX_CLEAN_CLK_DIV = 94
MAX_CLEAN_BIT_CYCLES = 33 * 94                 # one data bit: low + high, core cycles
MAX_CLEAN_BIT_RATE_HZ = 1e9 / (TICK_NS * MAX_CLEAN_BIT_CYCLES)
# The acknowledgement clock is one SM cycle faster than a data bit (its low phase is shorter),
# so it is the shortest rise-to-rise gap on the wire and what Ps2Checker.shortest_period is.
MAX_CLEAN_ACK_PERIOD = 32 * 94

RESET = 0xFF
SET_LEDS = 0xED
SET_SCAN_CODE_SET = 0xF0
DISABLE = 0xF5
ENABLE = 0xF4


class Bench:
    """A bus with the chip, one device and the timing checker on it."""

    def __init__(self, clkdiv=drv.DEFAULT_CLK_DIV, device="keyboard", clk_pin=DEFAULT_PINS[0],
                 data_pin=DEFAULT_PINS[1], rate="fast", dev_rate=None, **dev_kwargs):
        self.rate = rate
        # The device's own line rate is a separate knob, because `Ps2Checker` reports the
        # fastest and slowest period it measured on the wire and that is whichever of the two
        # sides clocks faster. Putting a slow device on the bus is how the host's own rate is
        # measured against the checker instead of the device's.
        self.dev_rate = dev_rate or rate
        self.clk_pin, self.data_pin = clk_pin, data_pin
        self.bus = Bus(ns_per_tick=TICK_NS)
        for k in range(pio_iss.NUM_PINS):                 # p0..p17, all pulled up
            self.bus.add_net(f"p{k}")
        self.chip = pio_iss.Chip()
        self.device = pio_iss.ChipDevice(self.chip)
        self.bus.attach(self.device)
        # The models look at the bus through the chip's own pin nets, so there is no
        # coupling device in between and no extra tick of latency anywhere.
        nets = dict(clk_net=f"p{clk_pin}", data_net=f"p{data_pin}")
        rate_dev = self.dev_rate
        if device == "keyboard":
            self.dev = attach_keyboard(self.bus, rate=rate_dev, **nets, **dev_kwargs)
        elif device == "mouse":
            self.dev = attach_mouse(self.bus, rate=rate_dev, **nets, **dev_kwargs)
        elif device == "bare":
            self.dev = attach_device(self.bus, rate=rate_dev, **nets, **dev_kwargs)
        else:
            raise ValueError(f"unknown device {device!r}")
        self.checker = attach_checker(self.bus, rate=rate, **nets)
        self.program = assemble_program()
        self.driver = Ps2Driver(
            self.bus, self.device, self.program, clkdiv=clkdiv,
            clk_pin=clk_pin, data_pin=data_pin,
            # The checker measures `tBUS-FREE` from the rising edge that opened the previous
            # frame's last clock, so the driver leaves the bus alone for exactly that long
            # between transactions. Rounding up is the model's own rule (`ns_to_ticks`).
            bus_free_ticks=ns_to_ticks(TICK_NS, drv.BUS_FREE_NS),
        )

    def start(self) -> "Bench":
        self.driver.load()
        self.driver.clear_fdebug()
        return self

    # ---- what the checker saw ------------------------------------------------------
    def frames(self) -> list:
        return [(f.value, f.direction, f.ack) for f in self.checker.frames]

    def period(self) -> int:
        """The shortest clock period the checker measured inside a frame, in core cycles."""
        assert self.checker.shortest_period is not None, "the checker measured no period"
        return self.checker.shortest_period

    def report(self) -> str:
        return self.checker.report()


def no_violations(bench: Bench) -> None:
    assert bench.checker.violations == [], bench.report()


# ---- the headline case: reset, LEDs, scan codes, at both speeds ----------------------
def test_reset_and_leds_at_both_speeds():
    """0xFF (reset) and 0xED (set LEDs) at the fast and at the slow configuration.

    Both directions on the wire: the host clocks the command out and the device clocks the
    acknowledgement and the answer back, so this covers the request-to-send, the twelve
    clocks of a host frame, the eleven clocks of a device frame and the bus-free wait
    between all of them.

    The device is put on the slow rate in both cases, so that the periods the checker
    measures are the *host's*: it reports the fastest and slowest clock on the wire, and
    against a fast device the host's own rate would be hidden behind the device's.
    """
    for config in (FAST, SLOW):
        bench = Bench(dev_rate="slow", **config).start()
        kbd, driver = bench.dev, bench.driver

        # Reset: 0xFA then the self-test byte, which is 0xAA for a keyboard.
        assert driver.command(RESET, response=1) == bytes([0xAA])
        assert kbd.enabled and kbd.scan_code_set == 2

        # LEDs: the command, then the argument byte, each acknowledged on its own.
        driver.command(SET_LEDS)
        driver.command(0x07)                      # all three LEDs on
        assert (kbd.caps_lock, kbd.num_lock, kbd.scroll_lock) == (True, True, True)

        no_violations(bench)
        # Three frames out (0xFF, 0xED and its argument) and four back: 0xFA and the 0xAA
        # self-test byte after the reset, then one 0xFA per command.
        assert (bench.checker.frames_to_device, bench.checker.frames_to_host) == (3, 4)
        assert bench.checker.acks == 3 and bench.checker.no_acks == 0
        assert [v for v, d, _ in bench.frames() if d == "to_device"] == [0xFF, 0xED, 0x07]
        assert [v for v, d, _ in bench.frames() if d == "to_host"] == [0xFA, 0xAA, 0xFA, 0xFA]
        assert driver.fdebug() == 0, f"FDEBUG {driver.fdebug():#010x}"
        # Both clock phases the divider produces are inside T3/T4 and T7/T8, and the gap the
        # checker measured between two rises is the one the program puts on the wire.
        low, high = (phase * TICK_NS for phase in driver.timing.half_periods(driver.clkdiv))
        assert 30e3 <= low <= 50e3 and 30e3 <= high <= 50e3, (low, high)
        assert driver.timing.rts * driver.clkdiv * TICK_NS >= 100e3, "the request-to-send"
        assert bench.period() == driver.timing.ack_cycles(driver.clkdiv)


def test_scan_codes_off_the_bus():
    """A key press and its break pair: what the keyboard sends, read by the chip.

    Set 2 sends a make code on the press and 0xF0 followed by the code on the release, so
    three frames in one go, taken by a single receive path entry with Y counting them.
    """
    bench = Bench(**FAST).start()
    kbd, driver = bench.dev, bench.driver
    driver.command(RESET, response=1)              # the keyboard answers and then scans
    driver.wait_bus_free()

    kbd.press("a")                                 # 0x1C
    assert driver.receive(1) == bytes([0x1C])
    kbd.release("a")                               # 0xF0 0x1C
    assert driver.receive(2) == bytes([0xF0, 0x1C])

    no_violations(bench)
    assert bench.checker.frames_to_device == 1
    # 0xFA and 0xAA after the reset, then 0x1C, 0xF0 and 0x1C.
    assert bench.checker.frames_to_host == 5
    assert driver.fdebug() == 0


def test_echo_and_the_scan_code_set_handshake():
    """0xEE (echo) and 0xF0 (get/set the scan code set), two commands with an argument each."""
    bench = Bench(**FAST).start()
    kbd, driver = bench.dev, bench.driver

    # 0xEE is the one command that answers with itself instead of 0xFA.
    assert driver.command(0xEE, expect=0xEE) == b""
    # 0xF0 takes an argument byte: 0 queries the scan code set, the keyboard answers with
    # 0xFA and the set it is using.
    driver.command(SET_SCAN_CODE_SET)
    assert driver.command(0x00, response=1) == bytes([kbd.scan_code_set])

    kbd.press("b")                                 # a set 2 make code
    assert driver.receive(1) == bytes([kbd.make_code("b")])

    no_violations(bench)
    assert [v for v, d, _ in bench.frames() if d == "to_device"] == [0xEE, 0xF0, 0x00]
    # The echo, two acknowledgements, the scan code set and the scan code for "b".
    assert [v for v, d, _ in bench.frames() if d == "to_host"] == [0xEE, 0xFA, 0xFA, 2,
                                                                   kbd.make_code("b")]


def test_scanning_can_be_switched_off_and_on():
    """0xF5 and 0xF4: nothing comes off the bus while scanning is off."""
    bench = Bench(**FAST).start()
    kbd, driver = bench.dev, bench.driver
    driver.command(DISABLE)
    assert not kbd.enabled
    kbd.press("c")                                 # a disabled keyboard queues nothing
    driver.wait_bus_free()
    assert bench.checker.frames_to_host == 1, "only the acknowledgement of 0xF5"
    assert kbd.pending == 0 and bench.checker.frames_to_device == 1
    driver.command(ENABLE)
    kbd.press("c")                                 # now it does
    assert driver.receive(1) == bytes([kbd.make_code("c")])
    no_violations(bench)
    assert bench.checker.frames_to_host == 3


def test_a_mouse_packet_arrives_intact():
    """The mouse's three-byte movement packet: one report, three frames, zero violations."""
    bench = Bench(**FAST, device="mouse").start()
    mouse, driver = bench.dev, bench.driver

    mouse.move(10, -5, buttons=1)                  # left button, 10 right and 5 up
    packet = driver.receive(3)
    # The head byte is 0x29: the always-one bit, the left button and the Y sign bit, which is
    # set because y is negative. The packet layout is verif/vip/ps2.py `Ps2MousePacket`.
    assert packet == mouse.reports[-1].wire == bytes([0x29, 0x0A, 0xFB])
    signed_x = packet[1] - ((packet[0] << 4) & 0x100)
    signed_y = packet[2] - ((packet[0] << 3) & 0x100)
    assert (signed_x, signed_y) == (10, -5)

    no_violations(bench)
    assert bench.checker.frames_to_host == 3
    assert driver.fdebug() == 0


def test_other_pins_and_the_slow_configuration():
    """Another pin pair at the slow divider: the windows are PINCTRL fields and CLKDIV."""
    bench = Bench(clkdiv=SLOW["clkdiv"], clk_pin=4, data_pin=5).start()
    kbd, driver = bench.dev, bench.driver
    assert driver.command(RESET, response=1) == bytes([0xAA])
    kbd.press("z")
    assert driver.receive(1) == bytes([kbd.make_code("z")])

    no_violations(bench)
    # The slow configuration is the lower of the two rates, and it is the one the bus shows.
    assert driver.line_rate_hz < 12e3, driver.line_rate_hz
    assert bench.checker.slowest_khz == pytest.approx(driver.line_rate_hz / 1e3, abs=0.5)
    assert bench.checker.frames_to_host == 3

def test_the_program_fits_the_instruction_memory():
    program = assemble_program()
    assert len(program.instrs) <= 32, (
        f"{len(program.instrs)} words, an engine has 32 (docs/mvp-isa.md "
        f"`### Engine state`)"
    )
    timing = drv.timing_of(program)
    assert timing.words == len(program.instrs)
    assert program.wrap_target == program.labels["idle"], "the wrap target is idle"
    assert program.wrap == program.labels["idle"], "the wrap top is idle too"
    # The wrap window has to hold the program: running off either end lands in the idle
    # preamble, never in the middle of a frame.
    assert 0 <= program.wrap_target <= program.wrap
    assert program.labels["init"] == 0 and program.labels["idle"] < program.labels["recv"], (
        "the send path falls through into idle, and idle has to come before the receive "
        "path or a finished send would arm the receiver"
    )


def test_the_driver_addresses_match_the_register_map_and_the_iss():
    """The driver is written against docs/mvp-regmap.md; this is the guard against the two
    drifting apart, in particular against the ISS renumbering something."""
    assert (drv.A_CTRL, drv.A_FSTAT, drv.A_FDEBUG, drv.A_FLEVEL) == (
        pio_iss.A_CTRL, pio_iss.A_FSTAT, pio_iss.A_FDEBUG, pio_iss.A_FLEVEL
    )
    assert (drv.A_TXF0, drv.A_RXF0) == (pio_iss.A_TXF[0], pio_iss.A_RXF[0])
    assert drv.A_IMEM == pio_iss.A_IMEM
    assert (drv.A_DBG_PADOUT, drv.A_DBG_PADOE, drv.A_DBG_CFGINFO) == (
        pio_iss.A_DBG_PADOUT, pio_iss.A_DBG_PADOE, pio_iss.A_DBG_CFGINFO
    )
    assert (drv.SM_CLKDIV, drv.SM_EXECCTRL, drv.SM_SHIFTCTRL, drv.SM_ADDR,
            drv.SM_INSTR, drv.SM_PINCTRL) == (
        pio_iss.SM_CLKDIV, pio_iss.SM_EXECCTRL, pio_iss.SM_SHIFTCTRL, pio_iss.SM_ADDR,
        pio_iss.SM_INSTR, pio_iss.SM_PINCTRL
    )


def test_the_word_packing_round_trips():
    """What the driver pushes and what it reads back, checked against the frame layout.

    The frame itself is compared with `verif.vip.ps2.ps2_frame_bits` and
    `ps2_parity_bit` -- the reference model is what the wire is judged against, so the two
    have to agree on the bits even though the driver does not import them.
    """
    from verif.vip.ps2 import ps2_frame_bits, ps2_parity_bit

    def frame_word(bits) -> int:
        """The RX word the receive path leaves in the ISR for a frame's eleven bits.

        IN_SHIFTDIR 0 shifts every sample in at bit 0 and pushes the older ones up, so the
        first bit sampled ends up highest: bit 10 is the start bit, bits 9:2 are d0..d7 with
        d7 lowest, bit 1 is the parity bit and bit 0 is the stop bit.
        """
        return sum(bit << (10 - i) for i, bit in enumerate(bits))

    for byte in (0x00, 0x01, 0x55, 0xA5, 0xFF, 0xF0, 0x1C):
        assert drv.parity_bit(byte) == ps2_parity_bit(byte)
        bits = ps2_frame_bits(byte)                        # the reference model's 11 bits
        assert drv.frame_bits(byte) == sum(b << i for i, b in enumerate(bits))
        # `tx_word` is what `out pindirs, 1` writes: the frame bits, inverted for a
        # direction register, so a driven 0 on the wire is a 1 in the word.
        word = drv.tx_word(byte)
        for i, bit in enumerate(bits):
            assert (word >> i) & 1 == 1 - bit, f"bit {i} of the frame for 0x{byte:02X}"
        assert word & ~drv.TX_FRAME_MASK == 0, "nothing outside the eleven frame bits"
        # And the way back, through the layout the receive path leaves the ISR in.
        word = frame_word(bits)
        assert drv.rx_byte(word) == byte, f"0x{byte:02X} did not survive the round trip"
        assert drv.rx_start(word) == 0 and drv.rx_stop(word) == 1
    for value in (-1, 0x100):
        with pytest.raises(ValueError):
            drv.tx_word(value)
    with pytest.raises(ValueError):
        drv.set_y_word(32)
    assert drv.rx_ack(0) and not drv.rx_ack(1 << drv.RX_ACK_SHIFT)
    # A frame with a wrong parity bit is caught by the driver, not passed on.
    good = frame_word(ps2_frame_bits(0x5A))
    assert drv.rx_parity(good) == drv.parity_bit(drv.rx_byte(good))
    bad = good ^ (1 << drv.RX_PARITY_SHIFT)
    assert drv.rx_parity(bad) != drv.parity_bit(drv.rx_byte(bad))


def test_the_pins_have_to_be_two_uio_pins():
    """Open drain is a direction, and only uio pins have one (docs/mvp-isa.md
    `### Open drain`), and the two lines have to be two pins."""
    device = pio_iss.ChipDevice(pio_iss.Chip())
    bus = Bus(ns_per_tick=TICK_NS)
    for k in range(pio_iss.NUM_PINS):
        bus.add_net(f"p{k}")
    bus.attach(device)
    program = assemble_program()
    for clk, data in ((8, 9), (0, 8), (13, 0), (3, 3)):
        with pytest.raises(ValueError):
            Ps2Driver(bus, device, program, clk_pin=clk, data_pin=data)
    # The two pins do not have to be neighbours: the side-set window and the OUT window are
    # separate PINCTRL fields, which is what fw/i2c (one window for both lines) cannot do.
    driver = Ps2Driver(bus, device, program, clk_pin=0, data_pin=7)
    assert driver.pinctrl_word == ((1 << 29) | (1 << 20) | (7 << 15) | (0 << 10) | 7)
    # A divider outside the window the program can run at is refused with the reason.
    with pytest.raises(ValueError, match="CLKDIV 50 is outside the range"):
        Ps2Driver(bus, device, program, clkdiv=50)
    with pytest.raises(ValueError, match="CLKDIV 200 is outside the range"):
        Ps2Driver(bus, device, program, clkdiv=200)


def test_fastest_divider_that_the_checker_accepts():
    """How fast this program can go: the fastest CLKDIV whose frames `Ps2Checker` passes
    with no violation at all, which is the number the README quotes.

    The device is on the slow rate so that the host is the fastest clock on the wire: the
    checker reports the periods it measured, and against a fast device the host's own rate
    would be the slower of the two and would not show up at all.
    """
    clean = []
    rejected = []
    for clkdiv in range(88, 101):
        try:
            bench = Bench(clkdiv=clkdiv, dev_rate="slow")
        except ValueError as exc:
            rejected.append((clkdiv, str(exc).split(":")[0]))
            continue
        bench.start()
        driver = bench.driver
        try:
            # One command and one byte the device sends, so both clock phases of both sides
            # are on the wire. The byte is queued after the command: a device may transmit at
            # any time, and a host frame in the middle of it would truncate it.
            driver.command(RESET, response=1)
            bench.dev.queue(0x1C)
            driver.receive(1)
        except Ps2Error:
            rejected.append((clkdiv, "the transaction did not complete"))
            continue
        rules = sorted({v.rule for v in bench.checker.violations})
        if rules or bench.checker.frames_to_host != 3:
            rejected.append((clkdiv, ",".join(rules) or "the byte did not arrive"))
            continue
        clean.append((clkdiv, bench.checker.shortest_period, bench.checker.fastest_khz))
    assert clean, f"not one CLKDIV passed: {rejected}"
    fastest_div, fastest_ticks, fastest_khz = clean[0]
    assert (fastest_div, fastest_ticks) == (MAX_CLEAN_CLK_DIV, MAX_CLEAN_ACK_PERIOD), (
        f"the max speed moved: {(fastest_div, fastest_ticks, fastest_khz)}, recorded "
        f"{(MAX_CLEAN_CLK_DIV, MAX_CLEAN_ACK_PERIOD)}; re-run python -m fw.ps2.measure"
    )
    # `Ps2Checker.shortest_period` is the acknowledgement clock, one SM cycle shorter than a
    # data bit; the data bit rate is what the divider was chosen for.
    assert fastest_ticks == MAX_CLEAN_ACK_PERIOD
    assert fastest_khz == pytest.approx(1e6 / (TICK_NS * MAX_CLEAN_ACK_PERIOD), abs=0.01)
    assert MAX_CLEAN_BIT_RATE_HZ / 1e3 == pytest.approx(
        1e6 / (TICK_NS * MAX_CLEAN_BIT_CYCLES), abs=0.01)      # kbit/s
    # Every divider the program accepts is legal, and the ones around the fastest one fail
    # only on the phase that ran out: the high phase is one SM cycle shorter than the low one.
    driver = Bench().driver
    assert clean[-1][0] <= driver.legal.stop - 1
    assert (MAX_CLEAN_CLK_DIV - 1) in [d for d, _ in rejected], rejected
    low, high = driver.timing.half_periods(MAX_CLEAN_CLK_DIV - 1)
    assert low * TICK_NS > 30e3 >= high * TICK_NS, (
        f"at CLKDIV {MAX_CLEAN_CLK_DIV - 1} the high phase is "
        f"{high * TICK_NS / 1e3:.1f} us, under T4's 30 us floor"
    )


def test_a_frame_the_device_gets_wrong_is_reported():
    """A host frame with a wrong parity bit: the device asks for a re-send, and the driver
    re-sends it. This is the 0xFE rule (OSDev: "if you send a command and the keyboard
    responds with 0xFE (resend) then you have to send the command again").

    The fault is in the *driver*: it pushes a frame with its parity bit flipped, so the chip
    puts a legal frame with a legal bit on the wire and the checker has nothing to say. Only
    the device's decode is wrong, which is what a real controller sees.
    """
    bench = Bench(**FAST).start()
    kbd, driver = bench.dev, bench.driver
    bad = drv.tx_word(RESET) ^ (1 << 9)              # the parity bit of the frame
    driver.reg_write(drv.A_TXF0, bad)
    driver.force(drv.jmp_word(driver._send))
    driver.wait_left_parked()
    driver.wait_parked()
    # The device clocks the acknowledgement bit even for a frame it could not read (the AT
    # table: "the auxiliary device ... still clocks the acknowledgement bit"), and then asks
    # for the byte again.
    assert drv.rx_ack(driver._take_rx())
    assert driver.receive(1) == bytes([RESEND])
    # The checker sees it too, and only that: a parity error is a wire-level rule, so this
    # is the one test where a violation is the expected answer.
    assert {v.rule for v in bench.checker.violations} == {"parity"}, bench.report()
    assert kbd.parity_errors == 1 and kbd.framing_errors == 0
    # And the retry the driver would make on its own now succeeds.
    assert driver.command(RESET, response=1) == bytes([0xAA])
    assert driver.resends == 0


def test_one_divider_lower_breaks_only_the_clock_high_minimum():
    """What the driver's own guard refuses, the checker would have said anyway.

    `Ps2Driver` refuses a CLKDIV whose high phase would be under T4's 30 us floor, so no
    clock ever goes out at the wrong speed. This test steps around that guard on purpose to
    show the checker's own verdict at the boundary: one divider lower, and the only rule
    that fires is the 30 us floor on the high phase -- not the request-to-send, not the low
    phase, not the framing.
    """
    bench = Bench(**FAST)
    bench.driver.clkdiv = MAX_CLEAN_CLK_DIV - 1        # the guard, stepped around on purpose
    bench.start()
    driver = bench.driver
    assert driver.command(RESET, response=1) == bytes([0xAA])
    bench.dev.queue(0x1C)
    assert driver.receive(1) == bytes([0x1C])

    # Both 16-SM-cycle phases are one divider short of legal: the high phase of every clock
    # and the low phase before the acknowledgement clock. The request-to-send, the low phase
    # of a data bit (17 SM cycles) and every framing rule are still clean.
    assert {v.rule for v in bench.checker.violations} == {"tCLK-high", "tCLK-low"}, \
        bench.report()
    low, high = driver.timing.half_periods(driver.clkdiv)      # (low, high)
    assert high * TICK_NS < 30e3 <= low * TICK_NS, (low, high)
    assert driver.timing.ack_low * driver.clkdiv * TICK_NS < 30e3

"""uart_rx with the isa_irq variant: the framing error is IRQ flag 4 instead of a marker word in the RX FIFO."""
from pathlib import Path

from asm.pioasm import assemble
from fw.uart_rx.driver import UartRx, build_program
from fw.uart_rx.test_uart_rx import BAUD, RX_PIN, BrokenStopTx, receive, transmitter
from tools.bench.sim.pio_iss_p import Params, chip_class

PIO = Path(__file__).with_name("uart_rx_irq.pio")


def variant():
    return assemble(PIO.read_text(), ("irq",))[0]


def rx_with_irq():
    return UartRx(baud=BAUD, rx_pin=RX_PIN, program=variant(), chip=chip_class(Params(isa_irq=True))())


def test_two_words_shorter():
    assert len(build_program().instrs) == 24
    assert len(variant().instrs) == 22


def test_good_frames_unchanged():
    rx = rx_with_irq()
    tx = transmitter(rx)
    tx.send(b"\x00\x55\xff\x80ABC")
    receive(rx, tx, 7)
    assert rx.values == list(b"\x00\x55\xff\x80ABC") and rx.markers == []
    assert rx.chip.irq_flags == 0


def test_framing_error_raises_flag_4_and_pushes_no_marker():
    rx = rx_with_irq()
    tx = rx.bus.attach(BrokenStopTx(rx.net, ticks_per_bit=rx.ticks_per_bit, broken=(0x41,), idle_bits=0.5))
    tx.send(b"ABC")
    receive(rx, tx, 3)
    assert rx.values == [0x41, 0x42, 0x43]
    assert rx.markers == []                         # no marker word ...
    assert rx.chip.irq_flags == 1 << 4              # ... the flag carries the news (no host register yet)


def test_the_mvp_chip_cannot_carry_it():
    # the same words on an MVP chip: IRQ is a NOP, the error is lost -- why the lever needs hardware
    rx = UartRx(baud=BAUD, rx_pin=RX_PIN, program=variant())
    tx = rx.bus.attach(BrokenStopTx(rx.net, ticks_per_bit=rx.ticks_per_bit, broken=(0x41,), idle_bits=0.5))
    tx.send(b"ABC")
    receive(rx, tx, 3)
    assert rx.markers == [] and rx.framing_errors == 0

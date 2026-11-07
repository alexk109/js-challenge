"""Firmware on engine 2 of a 3-engine `iss_p` chip: uart_tx and spi_read, end to end.

    BENCH_CONFIG='{"n_engines":3,"fifo_depth":8,"imem_words":64}' python -m tools.bench.sim.fw_engine2

Installs the `iss_p` target first (the drivers read `fw.regmap` at import), then runs the same checks
as `fw/uart_tx/test_uart_tx.py` and `fw/spi/test_spi.py::test_the_second_engine_can_do_it_too` with
the engine index 2. Prints one line per check; exit 1 on a failure.
"""
from __future__ import annotations

import os
import sys

os.environ.setdefault("FW_TARGET", "iss_p")
from fw import targets  # noqa: E402

targets.install("iss_p")

from fw import regmap  # noqa: E402
from fw.uart_tx import UartTxDriver  # noqa: E402
from fw.uart_tx.driver import CORE_HZ  # noqa: E402
from iss.pio_iss import ChipDevice  # noqa: E402
from verif.vip.core import Bus  # noqa: E402
from verif.vip.uart import UartRx  # noqa: E402


def uart_tx_engine2() -> None:
    pin, baud = 3, 115200.0
    bus = Bus(ns_per_tick=20.0)
    for k in range(18):
        bus.add_net(f"p{k}", push_pull=(k == pin))
    dev = bus.attach(ChipDevice())
    drv = UartTxDriver(dev, bus, engine=2, pin=pin, baud=baud)
    rx = bus.attach(UartRx(f"p{pin}", ticks_per_bit=CORE_HZ / baud, data_bits=8, parity="none",
                           stop_bits=1, latency=1))
    data = [0x55, 0xA3, 0x00, 0xFF, 0x7E, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]
    drv.load()
    drv.start(first=data[:drv.TXFIFO_DEPTH])
    drv.send(data[drv.TXFIFO_DEPTH:])
    bus.run(int(len(data) * 12 * CORE_HZ / baud) + 8000)
    chip = dev.chip
    assert rx.values == data, f"{rx.values} != {data}"
    assert rx.framing_errors == 0 and rx.aborted == 0
    assert chip.engines[0].pc == 0 and chip.engines[1].pc == 0, "another engine ran"
    print(f"uart_tx engine 2: {len(data)} bytes OK (TX FIFO depth {chip.P.fifo_depth}, "
          f"IMEM base {regmap.imem_addr(2, 0):#x})")


def spi_engine2() -> None:
    from fw.spi import SpiConfig, SpiMaster
    from fw.spi.test_spi import JEDEC_ID, Rig, jedec_bytes, jedec_tx

    rig = Rig(mode=0)
    master = SpiMaster(rig.dev, SpiConfig(mode=0), sm=2)
    master.load()
    while not rig.dev.idle:
        rig.bus.step()
    master.start(jedec_tx())
    assert master.run(rig.bus, max_ticks=4000)
    assert jedec_bytes(master.rx_bytes) == list(JEDEC_ID)
    assert master.read_fdebug(rig.bus) == 0
    assert rig.monitor.violations == []
    assert rig.dev.chip.engines[0].pc == 0 and rig.dev.chip.engines[1].pc == 0
    print("spi engine 2: JEDEC ID read OK")


if __name__ == "__main__":
    uart_tx_engine2()
    spi_engine2()
    sys.exit(0)

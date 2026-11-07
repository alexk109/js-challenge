"""Print what the UART-TX firmware achieves on the ISS, so the numbers are reproducible.

    .venv/bin/python -m fw.uart_tx.report

Everything here is measured, not asserted by hand: the word count and the SM cycles per bit
come out of the assembled program, the CLKDIV words out of `clkdiv_for_baud`, and the bit
edge deviations out of `verif.vip.uart.UartRx`, the independent judge.
"""
from __future__ import annotations

import copy

from verif.vip.core import Bus
from verif.vip.uart import UartRx

import fw.uart_tx.driver as driver_mod
from fw.uart_tx.driver import CORE_HZ, SM_CLKDIV, SM_EXECCTRL, SM_PINCTRL, SM_SHIFTCTRL, \
    bit_cycles, clkdiv_for_baud, clkdiv_value, load_program, ticks_per_bit, words_used
from fw.uart_tx.test_uart_tx import CASES, MUTATION_BAUD, MUTATIONS, _run_program, build, \
    payload, run_case

FRAME_BITS = 10      # 8N1: start + 8 data + stop


def divider_jitter(baud: float, per_bit: int, grid: str = "actual") -> float:
    """The bit-edge jitter the *divider alone* contributes at this baud, in ticks.

    `docs/mvp-isa.md` `### Clock divider` fixes the algorithm, so N SM cycles take exactly
    `N*INT + floor(N*FRAC/256)` core cycles and an edge k bit times into a frame can be
    predicted without simulating anything.  "actual" is the grid the chip produces, "nominal"
    is CORE_HZ / baud, which no transmitter with a fractional CLKDIV can hit exactly.  This is
    the transmitter's share: the numbers `UartRx` reports are larger by up to one further
    tick, because it measures edges in whole ticks and subtracts two of them.
    """
    clkdiv = clkdiv_for_baud(baud, per_bit)
    integer, frac = (clkdiv >> 16) & 0xFFFF, (clkdiv >> 8) & 0xFF
    step = ticks_per_bit(clkdiv, per_bit) if grid == "actual" else CORE_HZ / baud
    worst = 0.0
    for k in range(1, FRAME_BITS):
        sm_cycles = k * per_bit
        total = sm_cycles * integer + (sm_cycles * frac) // 256
        worst = max(worst, abs(total - k * step))
    return worst


def main() -> None:
    program = load_program()
    per_bit = bit_cycles(program)
    print(f"program        : {program.name}, {words_used(program)} of 32 IMEM words")
    print(f"bit time       : {per_bit} SM cycles "
          f"(delay {per_bit - 1} on each of the 8 `out pins, 1`)")
    print(f"max speed      : {CORE_HZ / ticks_per_bit(0x0001_0000, per_bit) / 1e6:.3f} MBd "
          f"at {CORE_HZ / 1e6:.0f} MHz (CLKDIV 1.0, {per_bit} core cycles per bit); a delay "
          f"of 3 would give {CORE_HZ / (4 * 1.0) / 1e6:.1f} MBd, which is the ceiling "
          f"verif.vip.uart.MIN_TICKS_PER_BIT allows")
    print()
    header = (f"{'case':>16} {'baud':>9} {'eng':>3} {'pin':>3} {'CLKDIV INT':>9} {'FRAC':>5} "
              f"{'ticks/bit':>10} {'nominal':>9} {'dev tick':>8} {'dev nom':>8} {'bytes':>5} "
              f"{'fdebug':>10}")
    print(header)
    print("-" * len(header))
    for case in CASES:
        data = payload(case.count, case.engine)
        res = run_case(case, data)
        clkdiv = res.drv.clkdiv
        ok = res.rx.values == data
        print(f"{case.name:>16} {case.baud:>9.0f} {case.engine:>3d} {case.pin:>3d} "
              f"{(clkdiv >> 16) & 0xFFFF:>9d} {(clkdiv >> 8) & 0xFF:>5d} "
              f"{res.actual_tpb:>10.4f} {res.nominal_tpb:>9.4f} "
              f"{res.rx.worst_dev_ticks:>8.4f} {res.rx_nominal.worst_dev_ticks:>8.4f} "
              f"{len(data):>5d} {res.drv.fdebug():>#010x}"
              f"{'' if ok else '  MISMATCH'}")
    print()
    print("dev tick : worst |edge - grid| against the bit grid the chip produces")
    print("dev nom  : the same against CORE_HZ/baud, which carries the CLKDIV rate error")
    print("fdebug   : 0x01000000/0x02000000 is TXSTALL, the engine waiting for the next byte")
    print()
    print("The divider's own jitter, computed from the spec rather than simulated:")
    header = f"{'baud':>9} {'CLKDIV INT':>9} {'FRAC':>5} {'actual grid':>12} {'nominal grid':>13}"
    print(header)
    print("-" * len(header))
    for baud in (9600.0, 38400.0, 115200.0, 230400.0, 921600.0, 2_000_000.0, 6_250_000.0):
        clkdiv = clkdiv_for_baud(baud, per_bit)
        print(f"{baud:>9.0f} {(clkdiv >> 16) & 0xFFFF:>9d} {(clkdiv >> 8) & 0xFF:>5d} "
              f"{divider_jitter(baud, per_bit):>12.4f} "
              f"{divider_jitter(baud, per_bit, 'nominal'):>13.4f}")
    print("The measured columns above are these plus up to one tick of whole-tick")
    print("quantization.  UartRx is configured on the actual grid, which is what a receiver")
    print("recovers from the line; whether CLKDIV reaches the requested baud is checked")
    print("separately, to half a CLKDIV step (test_clkdiv_hits_the_baud_within_the_...).")
    print()
    bus, dev, drv, rx, rx_nominal = build(CASES[0])
    drv.load()
    drv.start()
    bus.run(4000)
    st = dev.chip.state(0)
    print(f"registers: CTRL {drv.ctrl():#06x} CLKDIV {drv.read(drv.sm_addr(SM_CLKDIV)):#010x} "
          f"EXECCTRL {drv.read(drv.sm_addr(SM_EXECCTRL)):#010x} "
          f"SHIFTCTRL {drv.read(drv.sm_addr(SM_SHIFTCTRL)):#010x} "
          f"PINCTRL {drv.read(drv.sm_addr(SM_PINCTRL)):#010x} PC {st['pc']} "
          f"stalled {st['stalled']}")
    print(f"words    : {' '.join(f'{w:#06x}' for w in program.instrs)}")
    print()

    print(f"One word changed, {MUTATION_BAUD:.0f} Bd on pin 0 of engine 0: what UartRx saw")
    header = f"{'mutation':>30} {'values':>18} {'framing':>7} {'dev tick':>9}"
    print(header)
    print("-" * len(header))
    clean = _run_program(load_program(), MUTATION_BAUD)
    print(f"{'(none: the real program)':>30} "
          f"{' '.join(f'{v:02x}' for v in clean.values):>18} {clean.framing_errors:>7} "
          f"{clean.worst_dev_ticks:>9.4f}")
    for name, mutate in MUTATIONS:
        mutated = copy.deepcopy(load_program())
        mutate(mutated)
        original = driver_mod.bit_cycles
        driver_mod.bit_cycles = lambda prog: per_bit      # keep the divisor computable
        try:
            rx = _run_program(mutated, MUTATION_BAUD)
        finally:
            driver_mod.bit_cycles = original
        print(f"{name:>30} {' '.join(f'{v:02x}' for v in rx.values):>18} "
              f"{rx.framing_errors:>7} {rx.worst_dev_ticks:>9.4f}")
    print("The bar is 1.0000 tick; every mutation is caught by the values, the framing")
    print("flags or the deviation (test_broken_program_fails_the_check).")


if __name__ == "__main__":
    main()

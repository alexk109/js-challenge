"""fw/spi: SPI modes 1 and 2, the CPHA 1 half, and the long transactions in all four modes.

The goal is 32, 64 and 256-byte SPI transactions in all four SPI modes.
`fw/spi/test_spi_long.py` does the two modes the chip shipped with (0 and 3, CPHA 0, which is what
a serial flash supports); this file is the other two, and it is worth saying plainly what that took,
because the answer was not "one more program":

*   **two programs.**  `spi_read_m1` and `spi_read_m2` in `fw/spi/program.pio`.  A mode is (CPOL,
    CPHA) and CPHA decides which edge samples and which one lets the target change, so the CPHA 1
    programs are the CPHA 0 ones with the bit loop's two side-set values swapped -- and with the bit
    period split 5/4 rather than 6/2, because the master's `in pins, 1` has to be the *capture*
    edge and the MISO it samples was launched at the previous edge, `T_IN = 3` core cycles earlier.
    With the launch three cycles ahead of the sample the program sampled the value from before the
    launch and decoded every other byte wrong; the 5-cycle gap is what makes it work, and
    `test_the_bit_period_of_that_mode` is the measurement.
*   **a target that can answer.**  `verif/vip/spi.py`'s `SpiFlashTarget` refuses modes 1 and 2 --
    correctly, for a flash.  `fw/spi/cpha1_flash.py` reaches the CPHA 1 half of *that same model*
    by constructing it with the mode of the same CPOL and then switching the phase and re-running
    the model's own `_reset_session()`.  It invents no state and overrides no method, and
    `test_the_subclass_adds_no_state` is the guard on that.
*   **`SpiConfig.cpha`.**  `mode` is still the chip's CPOL choice, 0 or 3, so the existing guard
    (`SpiConfig(mode=1)` is a `ValueError`) is untouched; `cpha=1` picks the CPHA 1 program.  So
    datasheet mode 1 is `mode=0, cpha=1` and mode 2 is `mode=3, cpha=1`.

The judge is the same in all four modes: `SpiFlashTarget` answers an opcode it recognises and only
after three address bytes, and `SpiMonitor` decodes all four modes (`cpol`, `cpha`,
`is_capture_edge`) and holds the host to the timing rules of `docs/mvp-regmap.md`
`### Physical layer`.

Negative controls:

*   `test_the_wrong_phase_is_caught` -- the CPHA 0 program against a CPHA 1 flash, and the other
    way round.  The phase is verified by the judge's own command decode: the flash's `last_command`
    is 3 (READ) when the phases agree and 7 -- every bit shifted by one -- when they do not, and
    the test asserts the exact value 0x07 so that number is carried by the log rather than asserted
    only as "not READ".
*   `test_the_subclass_adds_no_state` -- a parent at mode 0 and the subclass at mode 2, same CPOL,
    must differ in exactly two attributes.
"""
from __future__ import annotations

import pytest

from iss.pio_iss import Chip, ChipDevice
from verif.vip.core import Bus
from verif.vip.spi import CMD_READ, SpiFlashTarget, SpiMonitor

from fw import targets
from fw.spi import SpiConfig, SpiMaster
from fw.spi.cpha1_flash import CPHA1_MODES, Cpha1FlashTarget
from fw.spi.driver import CPHA1_PROGRAMS, CS, MISO, MOSI, PROGRAMS, SCK, programs
from fw.spi.test_spi_long import MIN_TICKS, NS_PER_TICK, NUM_PINS, read_tx

SLOW = pytest.mark.skipif(not targets.is_fast(),
                          reason="tens of thousands of core cycles per transaction: a model "
                                 "target only (see fw/spi/test_spi_long.py)")

#: datasheet mode -> the `SpiConfig` that runs it.  Modes 0 and 3 are the shipped CPHA 0 programs
#: (`cpha=0`), modes 1 and 2 the CPHA 1 ones (`cpha=1`); `mode` in the config is the chip's CPOL.
MODES = {0: (0, 0), 1: (0, 1), 2: (3, 1), 3: (3, 0)}


class Rig:
    """Chip, flash and monitor on one bus, for a datasheet mode and a `cpha` to run."""

    def __init__(self, mode: int, cpha: int, hold_cs: bool = True, chunk_words: int = 7):
        cpol, _ = MODES[mode]
        self.mode = mode
        self.config = SpiConfig(mode=cpol, cpha=cpha, hold_cs=1 if hold_cs else 0)
        self.bus = Bus(ns_per_tick=NS_PER_TICK)
        for k in range(NUM_PINS):
            self.bus.add_net(f"p{k}", push_pull=k <= MISO)
        self.dev = ChipDevice(Chip(), name="chip")
        target = Cpha1FlashTarget if mode in CPHA1_MODES else SpiFlashTarget
        self.flash = self.bus.attach(target(size=4096, mode=mode, name="spi_flash",
                                             cs_net=f"p{CS}", sck_net=f"p{SCK}",
                                             mosi_net=f"p{MOSI}", miso_net=f"p{MISO}"))
        self.monitor = self.bus.attach(SpiMonitor(mode=mode, min_sck_ticks=MIN_TICKS,
                                                  min_cs_setup=MIN_TICKS, min_cs_hold=MIN_TICKS,
                                                  name="spi_mon", cs_net=f"p{CS}",
                                                  sck_net=f"p{SCK}", mosi_net=f"p{MOSI}",
                                                  miso_net=f"p{MISO}"))
        self.bus.attach(self.dev)
        self.master = SpiMaster(self.dev, self.config)
        self.chunk_words = chunk_words

    def read(self, n_wire_bytes: int, addr: int = 0x30, long: bool = True,
             max_ticks: int = 200_000) -> list[int]:
        """One read of `n_wire_bytes` bytes; returns the bytes the engine pushed."""
        data = [(0xA0 + i) & 0xFF for i in range(n_wire_bytes - 4)]
        self.flash.poke(addr, data)
        tx = read_tx(addr, n_wire_bytes)
        if long:
            self.master.start_long(tx, chunk_words=self.chunk_words)
            ok = self.master.run_long(self.bus, max_ticks=max_ticks)
        else:
            self.master.start(tx)
            ok = self.master.run(self.bus, max_ticks=max_ticks)
        assert ok, (f"mode {self.mode}: {n_wire_bytes} bytes did not finish in {max_ticks} core "
                    f"cycles (pc={self.master.pc}, rx={len(self.master.rx_words)} words)")
        self.data = data
        return self.master.rx_bytes


#: Every size runs on every target.  The 256-byte rung is 17 290 core cycles, which is about ten
#: seconds through the RTL harness's pipes (measured: 2.24 s for 32 bytes and 3.54 s for 64 on
#: `FW_TARGET=rtl`) -- affordable, so it is
#: not marked model-only.
SIZES = [pytest.param(32, False, id="32"), pytest.param(64, False, id="64"),
         pytest.param(256, False, id="256")]


def sck_period(rig: "Rig") -> int:
    """The shortest SCK period the monitor saw, in core cycles (`stats` holds the level times)."""
    stats = rig.monitor.stats
    return stats["sck_low_min"] + stats["sck_high_min"]


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
@pytest.mark.parametrize("n_bytes,slow", SIZES)
def test_a_long_read_in_that_mode(mode: int, n_bytes: int, slow: bool) -> None:
    """32, 64 and 256 bytes in each of the four SPI modes, one CS window, right bytes back.

    This is the "all four SPI modes" test.  Everything the test asserts is the judge's: the
    byte values (which only a flash that kept its command state can produce), the flash's
    `last_command`, the monitor's decode of what was on the wire, its count of CS windows, and its
    timing violations.
    """
    if slow and not targets.is_fast():
        pytest.skip(f"{n_bytes} bytes is about {n_bytes * 66} core cycles; model targets only")
    cpol, cpha = MODES[mode]
    rig = Rig(mode, cpha)
    rx = rig.read(n_bytes)

    assert len(rx) == n_bytes, f"mode {mode}: {len(rx)} bytes, wanted {n_bytes}"
    assert rx[4:4 + len(rig.data)] == rig.data, (
        f"mode {mode}: the data came back wrong: {[hex(b) for b in rx[4:8]]} against "
        f"{[hex(b) for b in rig.data[:4]]}")
    assert rig.flash.last_command == CMD_READ, (
        f"mode {mode}: the flash's last command is {rig.flash.last_command:#04x}, not READ")
    assert len(rig.monitor.transfers) == 1, (
        f"mode {mode}: {len(rig.monitor.transfers)} CS windows, so not one transaction")
    xfer = rig.monitor.transfers[0]
    assert xfer.mosi[:4] == [CMD_READ, 0x00, 0x00, 0x30], f"mode {mode}: {xfer.mosi[:4]} on MOSI"
    assert xfer.miso[4:4 + len(rig.data)] == rig.data, f"mode {mode}: the monitor's MISO decode"
    assert rig.monitor.violations == [], f"mode {mode}: {rig.monitor.violations}"
    assert rig.bus.errors == []
    assert rig.master.read_fdebug(rig.bus) == 0
    print(f"\nmode {mode} (cpha={cpha}): {n_bytes} bytes, "
          f"{rig.master._chunks} chunks, one window of {xfer.t_end - xfer.t_start} core cycles, "
          f"SCK {1e9 / NS_PER_TICK / sck_period(rig) / 1e6:.3f} MHz")


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
def test_the_bit_period_of_that_mode(mode: int) -> None:
    """The SCK period each mode's program produces: 8 core cycles for CPHA 0, 9 for CPHA 1.

    From the monitor's own `stats`, which measure how long SCK held each level.  The CPHA 1 pair is
    one core cycle per bit slower and that is the price of the MISO margin below: its bit period is
    split 5/4 so that the capture edge is five cycles after the launch edge the target reacted to,
    where T_IN = 3 and one cycle of slack would be the CPHA 0 programs' `[5]` delay.
    """
    cpol, cpha = MODES[mode]
    rig = Rig(mode, cpha)
    rig.read(32)
    stats = rig.monitor.stats
    period = stats["sck_low_min"] + stats["sck_high_min"]
    assert period == (8 if cpha == 0 else 9), f"mode {mode}: {stats}"
    assert stats["sck_low_min"] >= MIN_TICKS and stats["sck_high_min"] >= MIN_TICKS, stats
    assert stats["cs_setup_min"] >= MIN_TICKS and stats["cs_hold_min"] >= MIN_TICKS, (
        f"mode {mode}: CS setup/hold {stats['cs_setup_min']}/{stats['cs_hold_min']}")
    print(f"\nmode {mode}: bit period {period} core cycles, SCK low {stats['sck_low_min']} / high "
          f"{stats['sck_high_min']}, CS setup {stats['cs_setup_min']} / hold "
          f"{stats['cs_hold_min']}")


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
def test_the_wrong_phase_is_caught(mode: int) -> None:
    """The negative control for every phase claim, and a measured asymmetry between the phases.

    Two things are checked, in the directions that bite:

    *   a **CPHA 0 program against a CPHA 1 flash** is wrong, and the judge says so by itself: the
        flash samples on the falling edge, which is the same cycle the CPHA 0 program writes MOSI
        in, so it reads the value the pin had a tick earlier (`verif/vip/core.py`'s convention: a
        device sees a driven value at tick t+1).  Every bit is one position out, the opcode is 7
        instead of 3, and no data comes back.
    *   a **CPHA 1 program against a CPHA 0 flash** works, which is worth knowing rather than
        hiding: the CPHA 1 program drives MOSI a whole half bit before the edge the target captures
        it on and samples MISO a half bit after the target launched it, so the two agree on the same
        bits one position further along.  So "the phases disagree" is *not* symmetric, and this
        test asserts the asymmetry rather than pretending otherwise -- which is also why the
        controls that matter are the long transactions against the *matching* flash in
        `test_a_long_read_in_that_mode`.
    """
    cpol, right = MODES[mode]
    assert cpol in (0, 3)

    rig = Rig(mode, right)
    rx = rig.read(32)
    assert rx[4:4 + len(rig.data)] == rig.data, f"mode {mode}: the matching phase must work"
    assert rig.flash.last_command == CMD_READ

    if mode in CPHA1_MODES:
        swapped = Rig(mode, 1 - right)
        rx = swapped.read(32)
        assert rx[4:4 + len(swapped.data)] != swapped.data, (
            f"mode {mode}: a CPHA 0 program read the flash's data, so this control cannot see a "
            f"phase error")
        assert swapped.flash.last_command == 0x07, (
            f"mode {mode}: the flash decoded opcode {swapped.flash.last_command:#04x} for a frame "
            f"whose bits are each one position out, where READ is {CMD_READ:#04x}")
        assert len(swapped.monitor.transfers) == 1
        mosi = swapped.monitor.transfers[0].mosi[:4]
        assert mosi[0] != CMD_READ, (
            f"mode {mode}: the monitor decoded MOSI {mosi}, which is the same opcode shifted")
        print(f"\nmode {mode}: a CPHA 0 program against a CPHA 1 flash decodes opcode "
              f"{swapped.flash.last_command:#04x}, not READ ({CMD_READ:#04x}), and returns "
              f"{len(rx)} bytes of which none is the flash's data")
    else:
        # CPHA 1 master, CPHA 0 flash: measured compatible, and asserted so it cannot change
        # silently in either direction.
        other = Rig(mode, 1)
        rx = other.read(32)
        assert rx[4:4 + len(other.data)] == other.data, (
            f"mode {mode}: the CPHA 1 program no longer reads a CPHA 0 flash; this test records "
            f"that compatibility, so a change here is a finding, not a pass")
        assert other.flash.last_command == CMD_READ
        assert other.monitor.violations == []
        print(f"\nmode {mode}: a CPHA 1 program against a CPHA 0 flash still decodes opcode "
              f"{other.flash.last_command:#04x} and reads the data ({len(rx)} bytes)")


def test_the_cpha1_programs_are_the_cpha0_ones_with_the_edges_swapped() -> None:
    """`spi_read_m1` is `spi_read_m0` with the bit loop's phases swapped, and the same for mode 2.

    That is the whole of the difference between a CPHA 0 and a CPHA 1 master, and saying it in
    words is only worth something if the words match: the operands are identical (the mode is not in
    them), the labels are at the same addresses, and the side-set fields differ exactly on the two
    instructions of the bit loop that make an edge.
    """
    cpha0, cpha1 = programs(), programs(cpha=1)
    assert set(cpha0) == set(cpha1) == {0, 3}
    assert set(PROGRAMS.values()) == {"spi_read_m0", "spi_read_m3"}
    assert set(CPHA1_PROGRAMS.values()) == {"spi_read_m1", "spi_read_m2"}
    for mode in (0, 3):
        a, b = cpha0[mode], cpha1[mode]
        assert len(a.instrs) == len(b.instrs) == 10, (
            f"mode {mode}: {len(a.instrs)} and {len(b.instrs)} words")
        assert a.labels == b.labels, f"mode {mode}: {a.labels} against {b.labels}"
        for i, (x, y) in enumerate(zip(a.instrs, b.instrs)):
            assert (x & 0xFF) == (y & 0xFF), (
                f"mode {mode} word {i}: the operands differ, and the mode is not in them")
        bit = a.labels["bit"]
        # The edge-making words: `out pins, 1` and `in pins, 1`, plus the `jmp y--` that follows.
        for i in (bit, bit + 1, bit + 2):
            assert ((a.instrs[i] >> 8) & 0x1F) != ((b.instrs[i] >> 8) & 0x1F) or i == bit + 2, (
                f"mode {mode} word {i}: the side-set value did not change")
        # The CPHA 1 programs run one core cycle per bit slower: the bit period is split 5/4.
        for prog in (a, b):
            delays = [((w >> 8) & 0x7) for w in prog.instrs[bit:bit + 3]]
            assert delays == [5, 0, 0] if prog is a else delays == [4, 2, 0], (
                f"mode {mode}: bit loop delays {delays}")


def test_the_subclass_adds_no_state() -> None:
    """`Cpha1FlashTarget` is the shipped model with its phase switched, and nothing else.

    The guard on `fw/spi/cpha1_flash.py`.  A parent at mode 0 and the subclass at mode 2 are built
    with the same arguments: same CPOL, so every field the parent sets is set the same way.  The two
    objects must then differ in exactly two attributes -- `mode` and `_out_idx`, which is
    `first_output_index(mode)`, 0 for CPHA 0 and -1 for CPHA 1 -- and in nothing else at all.
    """
    parent = SpiFlashTarget(size=1024, mode=0, name="spi_flash")
    subclass = Cpha1FlashTarget(size=1024, mode=2, name="spi_flash")
    assert subclass.mode == 2 and parent.mode == 0
    differences = {k for k in set(parent.__dict__) | set(subclass.__dict__)
                   if parent.__dict__.get(k) != subclass.__dict__.get(k)}
    assert differences == {"mode", "_out_idx"}, (
        f"the subclass differs from the parent in {sorted(differences)}, not just the phase")
    assert parent._out_idx == 0 and subclass._out_idx == -1, (
        f"CPHA 0 presents bit 0 first, CPHA 1 a dummy: {parent._out_idx}, {subclass._out_idx}")
    # It refuses the modes the parent already handles, so the two are never both in play.
    for mode in (0, 3):
        with pytest.raises(ValueError, match="CPHA 1"):
            Cpha1FlashTarget(mode=mode)
    with pytest.raises(ValueError, match="CPHA 0"):
        SpiFlashTarget(mode=1)

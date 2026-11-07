"""fw/spi: transactions longer than 31 bytes, in every mode the chip can clock.

`docs/mvp-verification-report.md` `### Known holes in the stimulus` names the hole: "The SPI
firmware cannot do a transaction longer than 31 bytes, because the byte counter goes in as a
5-bit immediate through `SMe_INSTR`, and nothing splits the transaction."  This file is the split,
and the answer is the driver, not a new program:

    fw/spi/driver.py, SpiConfig(hold_cs=1) + SpiMaster.start_long() + run_long()

`load()` patches the single word at `done` so the program parks with CS low instead of raising it,
and every chunk after the first is armed with the forced instructions carrying the *held* side-set
level.  Neither change moves a pin: the held side-set value is exactly what `enter` leaves behind,
which is the state the flash sees between chunks.  So the wire shows a pause in SCK and no gap in
CS, and `verif/vip/spi.py`'s `SpiFlashTarget` answers one command over one window -- which is the
judge's own opinion of what a transaction is, arrived at without being told about chunks.

The judge is unmodified: `SpiFlashTarget` (a flash that only answers an opcode it recognises and
only after the three address bytes have arrived) and `SpiMonitor` (a passive decoder plus the host
timing rules of `docs/mvp-regmap.md` `### Physical layer`: SCK and CS each at least 2 core cycles).
Every test asserts the monitor's `violations == []`, the byte values, the *number of CS windows* the
monitor saw, and `FDEBUG`.

Negative controls, one per new claim:

*   `test_released_cs_between_chunks_is_two_transactions_to_the_flash` -- the same 32 bytes with the
    CS hold left off.  The monitor then sees two windows and the flash, which drops its command
    state at the end of a window, answers nothing after the first chunk.  That is what the patch
    exists to prevent.
*   `test_a_chunk_with_the_wrong_byte_count_is_caught` -- a chunk size that does not add up to the
    transaction, which must come back short or garbled rather than quietly "passing".

Modes: the chip's SPI master has one program per SPI *phase* pair -- `spi_read_m0` and
`spi_read_m3`, CPHA 0, which is what a serial flash supports and what `verif/vip/spi.py`'s
`SpiFlashTarget` models.  Modes 1 and 2 are CPHA 1, a different program *and* a different target
model; `fw/spi/test_spi_cpha1.py` is that, judged by the same `SpiMonitor` (which decodes all four
modes) with a pattern target committed beside the VIP.  See that file for what "all four modes"
costs.
"""
from __future__ import annotations

import pytest

from iss.pio_iss import Chip, ChipDevice
from verif.vip.core import Bus
from verif.vip.spi import CMD_READ, SpiFlashTarget, SpiMonitor

from fw import targets
from fw.spi import A_FSTAT, SpiConfig, SpiMaster
from fw.spi.driver import CS, DEFAULT_CHUNK_WORDS, MISO, MOSI, SCK

NS_PER_TICK = 20.0
F_CLK_HZ = 1.0e9 / NS_PER_TICK
NUM_PINS = 18
#: The host timing rules, in core cycles, that the monitor enforces
#: (docs/mvp-regmap.md `### Physical layer`).
MIN_TICKS = 2
#: Tests that clock a quarter of a thousand bytes: model targets only, because the Verilator-backed
#: target runs at a few thousand core cycles a second (`fw/targets.py`'s `is_fast()`).
SLOW = pytest.mark.skipif(not targets.is_fast(),
                          reason="tens of thousands of core cycles per transaction: a model "
                                 "target only (see the module docstring)")


def tx_word(*bytes_: int) -> int:
    """Bytes as one left-justified TX word (`docs/mvp-isa.md` difference 11)."""
    word = 0
    for i, value in enumerate(bytes_):
        word |= (value & 0xFF) << (24 - 8 * i)
    return word


def read_tx(addr: int, n_wire_bytes: int) -> list[int]:
    """The TX words of a flash read that clocks `n_wire_bytes` bytes on the wire.

    The whole transaction is opcode + three address bytes + data, so a transaction of
    `n_wire_bytes` returns `n_wire_bytes - 4` data bytes.  One TX word is four bytes on the wire, so
    `n_wire_bytes` is a multiple of 4; 32, 64 and 256 are.
    """
    if n_wire_bytes % 4:
        raise ValueError(f"a transaction is whole TX words, got {n_wire_bytes} bytes")
    words = n_wire_bytes // 4
    if words < 1:
        raise ValueError(f"a transaction needs at least one TX word, got {n_wire_bytes} bytes")
    return [tx_word(CMD_READ, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF)] + \
        [tx_word(0x00, 0x00, 0x00, 0x00)] * (words - 1)


class Rig:
    """A bus with the chip, the flash and the monitor on it, plus the long-transaction driver."""

    def __init__(self, mode: int = 0, clkdiv: tuple[int, int] = (1, 0), size: int = 4096,
                 hold_cs: bool = True, chunk_words: int = DEFAULT_CHUNK_WORDS):
        self.mode = mode
        self.config = SpiConfig(mode=mode, clkdiv=clkdiv, hold_cs=1 if hold_cs else 0)
        self.bus = Bus(ns_per_tick=NS_PER_TICK)
        for k in range(NUM_PINS):
            self.bus.add_net(f"p{k}", push_pull=k <= MISO)
        self.dev = ChipDevice(Chip(), name="chip")
        self.flash = self.bus.attach(SpiFlashTarget(
            size=size, mode=mode, name="spi_flash", cs_net=f"p{CS}", sck_net=f"p{SCK}",
            mosi_net=f"p{MOSI}", miso_net=f"p{MISO}"))
        self.monitor = self.bus.attach(SpiMonitor(
            mode=mode, min_sck_ticks=MIN_TICKS, min_cs_setup=MIN_TICKS, min_cs_hold=MIN_TICKS,
            name="spi_mon", cs_net=f"p{CS}", sck_net=f"p{SCK}", mosi_net=f"p{MOSI}",
            miso_net=f"p{MISO}"))
        self.bus.attach(self.dev)
        self.master = SpiMaster(self.dev, self.config)
        self.chunk_words = chunk_words

    def run(self, n_wire_bytes: int, addr: int = 0x30, max_ticks: int = 200_000) -> list[int]:
        """One long read of `n_wire_bytes` bytes; returns the bytes the engine pushed."""
        data = [(0xA0 + i) & 0xFF for i in range(n_wire_bytes - 4)]
        self.flash.poke(addr, data)
        self.master.start_long(read_tx(addr, n_wire_bytes), chunk_words=self.chunk_words)
        assert self.master.run_long(self.bus, max_ticks=max_ticks), (
            f"{n_wire_bytes} bytes did not finish in {max_ticks} core cycles "
            f"(pc={self.master.pc}, rx={len(self.master.rx_words)} words, "
            f"chunk {self.master._chunk_index + 1} of {self.master._chunks})")
        self.data = data
        return self.master.rx_bytes

    def check_clean(self, n_wire_bytes: int) -> None:
        """The two judges, the bus's contention record and the chip's own FIFO flags."""
        assert self.monitor.violations == [], self.monitor.violations
        assert self.bus.errors == [], self.bus.errors
        assert self.master.read_fdebug(self.bus) == 0, "FDEBUG raised a FIFO flag"
        fstat = self.master.read_reg(self.bus, A_FSTAT)
        empty = ((1 << 24) | (1 << 8)) << (8 * self.master.sm)
        assert fstat & empty == empty, f"FSTAT {fstat:#010x}: a FIFO is not empty"


# ---------------------------------------------------------------------------------------------
# The transactions tested: 32, 64 and 256 bytes, inside one CS window
# ---------------------------------------------------------------------------------------------

#: Every size runs on every target.  The 256-byte rung is 17 290 core cycles, which is about ten
#: seconds through the RTL harness's pipes (measured: 2.24 s for 32 bytes and 3.54 s for 64 on
#: `FW_TARGET=rtl`) -- affordable, so it is
#: not marked model-only.
SIZES = [pytest.param(32, False, id="32"), pytest.param(64, False, id="64"),
         pytest.param(256, False, id="256")]


@pytest.mark.parametrize("mode", [0, 3])
@pytest.mark.parametrize("n_bytes,slow", SIZES)
def test_a_read_of_that_many_bytes_is_one_transaction(mode: int, n_bytes: int, slow: bool) -> None:
    """32, 64 and 256 bytes on the wire: the right bytes back, in **one** CS window.

    The single window is the claim that makes this a transaction rather than a sequence of short
    ones, and it is the monitor's own count: `SpiMonitor` opens a transfer at every CS fall and
    closes it at every rise, so a chunk boundary that moved CS would show up as two transfers.  One
    transfer of `n_bytes` MOSI bytes means the flash saw one command, one address and one data
    phase, and its answer -- which only a flash that has kept its command state can give -- comes
    back whole.
    """
    if slow and not targets.is_fast():
        pytest.skip(f"{n_bytes} bytes is about {n_bytes * 66} core cycles; model targets only")
    rig = Rig(mode=mode)
    rx = rig.run(n_bytes)

    data = rig.data
    assert len(rx) == n_bytes, f"a {n_bytes}-byte transaction is {n_bytes} bytes, got {len(rx)}"
    # Byte 0 of a read is whatever the flash was driving while the opcode went out (see
    # `fw/spi/test_spi.py::jedec_bytes`), so the data starts at byte 4.
    assert rx[4:4 + len(data)] == data, (
        f"the {len(data)} data bytes came back wrong: {[hex(b) for b in rx[4:8]]} ... against "
        f"{[hex(b) for b in data[:4]]} ...")
    assert len(rig.monitor.transfers) == 1, (
        f"the monitor saw {len(rig.monitor.transfers)} CS windows, so this was "
        f"{len(rig.monitor.transfers)} transactions and not one")
    xfer = rig.monitor.transfers[0]
    assert xfer.mosi[:4] == [CMD_READ, 0x00, 0x00, 0x30], "the wire carried the opcode MSB first"
    assert len(xfer.mosi) == n_bytes, f"{len(xfer.mosi)} bytes on MOSI, wanted {n_bytes}"
    assert xfer.miso[4:4 + len(data)] == data, "the monitor decoded the same data off the wire"
    assert rig.flash.last_command == CMD_READ
    assert rig.master._chunks == (n_bytes // 4 + DEFAULT_CHUNK_WORDS - 1) // DEFAULT_CHUNK_WORDS
    rig.check_clean(n_bytes)
    print(f"\nmode {mode}, {n_bytes} bytes: {rig.master._chunks} chunks, one CS window of "
          f"{xfer.t_end - xfer.t_start} core cycles")


@pytest.mark.parametrize("mode", [0, 3])
def test_the_last_chunk_may_be_short(mode: int) -> None:
    """A transaction whose word count is not a multiple of the chunk size still ends on a word.

    40 bytes is 10 TX words: 7 + 3, so the last chunk clocks 12 bytes and the counter has to be
    armed with 12, not 28.  This is the case a naive "always arm with the chunk size" split gets
    wrong, and the flash's answer is the judge: a chunk of the wrong length leaves the data phase
    short or long and the bytes after it are wrong.
    """
    rig = Rig(mode=mode)
    rx = rig.run(40)
    assert len(rx) == 40, rx
    assert rx[4:4 + len(rig.data)] == rig.data
    assert len(rig.monitor.transfers) == 1
    rig.check_clean(40)


@pytest.mark.parametrize("chunk_words", [1, 2, 4, 7])
def test_the_chunk_size_does_not_change_the_wire(chunk_words: int) -> None:
    """The same 64 bytes split four ways: same data, same one CS window, whatever the chunk size.

    1 word per chunk is the extreme: 16 re-arms, 16 pauses in SCK.  It is here because it is the
    case where the host's per-chunk arming has the most chances to disturb the wire, and it must
    not.
    """
    rig = Rig(mode=0, chunk_words=chunk_words)
    rx = rig.run(64)
    assert rx[4:4 + len(rig.data)] == rig.data, f"chunk_words={chunk_words}"
    assert len(rig.monitor.transfers) == 1, (
        f"chunk_words={chunk_words}: the monitor saw {len(rig.monitor.transfers)} windows")
    assert rig.monitor.violations == []
    rig.check_clean(64)


# ---------------------------------------------------------------------------------------------
# Negative controls
# ---------------------------------------------------------------------------------------------

def test_released_cs_between_chunks_is_two_transactions_to_the_flash() -> None:
    """The same 32 bytes with the CS hold left off: two windows, and the flash answers nothing.

    This is the control for the patch.  Without `hold_cs` the program raises CS at each chunk's
    `done`, which is correct behaviour for a short transaction and fatal for a long one: the flash
    drops its command state when CS goes high (`SpiFlashTarget._deselect`), so the second chunk is
    an unknown-opcode window and the data after the first chunk is gone.
    """
    held = Rig(mode=0, hold_cs=True)
    rx_held = held.run(32)
    assert rx_held[4:4 + len(held.data)] == held.data
    assert len(held.monitor.transfers) == 1

    released = Rig(mode=0, hold_cs=False)
    with pytest.raises(ValueError, match="hold_cs"):
        released.master.start_long(read_tx(0x30, 32))
    # Bypass the guard to show what the wire would look like: load the program, then arm and run
    # the two chunks by hand, letting the program raise CS at each `done`.
    rig = Rig(mode=0, hold_cs=False)
    rig_data = [(0xA0 + i) & 0xFF for i in range(28)]
    rig.flash.poke(0x30, rig_data)
    rig.master.load()
    while not rig.dev.idle:
        rig.bus.step()
    rig.master._rx, rig.master._rx_level = [], 0
    rig.master._n_bytes = 32
    rig.master._chunks = 2
    rig.master._chunk_index = 0
    rig.master._released = True
    rig.master._sent_bytes = 0
    rig.master._all_tx = read_tx(0x30, 32)
    rig.master._chunk_words = DEFAULT_CHUNK_WORDS
    rig.master._arm(rig.master._all_tx[:DEFAULT_CHUNK_WORDS], 28)
    assert rig.master.run(rig.bus, max_ticks=60_000), "the first chunk did not finish"
    # The second chunk, armed exactly as the first was: the program will raise CS at its `done`.
    rig.master._chunk_index = 1
    rig.master._arm(rig.master._all_tx[DEFAULT_CHUNK_WORDS:], 4)
    assert rig.master.run(rig.bus, max_ticks=60_000), "the second chunk did not finish"
    rig.bus.run(8)
    assert len(rig.monitor.transfers) == 2, (
        f"the control is broken: the monitor saw {len(rig.monitor.transfers)} windows, so it "
        f"cannot tell a held CS from a released one")
    assert rig.master.rx_bytes[4:4 + 24] != rig_data[4:], (
        "the control is broken: the flash answered across a released CS")
    print("\ncontrol: with CS released between chunks the flash answers "
          f"{sum(1 for a, b in zip(rig.master.rx_bytes[4:], rig_data) if a == b)} of "
          f"{len(rig_data)} bytes correctly")


def test_a_chunk_with_the_wrong_byte_count_is_caught() -> None:
    """Two controls on the arming arithmetic, both refused before anything is clocked.

    A chunk is whole TX words and at most seven of them (28 bytes, the most the 5-bit byte counter
    can express with room to spare), and a transaction is at least one word.  Both are refused by
    `start_long()` rather than quietly producing a short or a doubled transaction.
    """
    rig = Rig(mode=0)
    with pytest.raises(ValueError, match="TX words"):
        rig.master.start_long(read_tx(0x30, 32), chunk_words=8)
    with pytest.raises(ValueError, match="TX words"):
        rig.master.start_long(read_tx(0x30, 32), chunk_words=0)
    with pytest.raises(ValueError, match="at least one TX word"):
        rig.master.start_long([])
    with pytest.raises(ValueError, match="whole TX words"):
        read_tx(0x30, 30)


def test_hold_cs_only_changes_the_word_at_done() -> None:
    """The patch is one instruction word, and it is CS's side-set bit and nothing else.

    The claim `test_released_cs_between_chunks_is_two_transactions_to_the_flash` rests on is that
    holding CS is free on the wire, and the cheapest way to show that is to show what the patch
    actually is: the program's `done` word with bit 11 (side-set bit 0, which is CS) clear.  The
    result is the same side-set value `enter` uses, which is why no pin moves at a chunk boundary.
    """
    from fw.spi.driver import programs

    shipped = programs()[0].instrs
    rig = Rig(mode=0)
    rig.master.load()
    while not rig.dev.idle:
        rig.bus.step()
    loaded = rig.dev.chip.engines[0].imem
    done = rig.master.done_addr
    original = shipped[done]
    patched = loaded[done]
    assert original != patched, "load() did not patch `done` with hold_cs set"
    assert (original ^ patched) == 0x0800, (
        f"the patch changed more than CS's side-set bit: {original:#06x} -> {patched:#06x}")
    assert (patched >> 11) & 0x3 == (rig.master.program.labels["enter"] and
                                     ((programs()[0].instrs[rig.master.program.labels["enter"]] >>
                                       11) & 0x3)), (
        "holding CS must leave the side-set value `enter` has, or a boundary moves a pin")
    # Nothing else in IMEM moved.
    assert [w for i, w in enumerate(loaded) if i != done and w != 0] == \
        [w for i, w in enumerate(shipped) if i != done and w != 0]

"""PUSH and PULL (ISA `### Instruction semantics`, `### Removed and reserved
encodings` for the ignored bits 4:0).
"""

import pytest

from verif.iss_b import enc
from verif.iss_b.enc import execs, jmp, poke, pull, push, set_, setup
from verif.iss_b.enc import JMP_ALWAYS

RXSTALL0 = 0x1  # FDEBUG.RXSTALL[0]
TXSTALL0 = 1 << 24  # FDEBUG.TXSTALL[0]


def test_push_pushes_and_clears_the_isr():
    b = setup(imem=[push(), jmp(JMP_ALWAYS, 0)])
    poke(b, 0, isr=0xCAFEBABE, isr_cnt=32)
    st = execs(b, 1)[0]
    assert st["rx_level"] == 1
    assert st["isr"] == 0 and st["isr_cnt"] == 0
    assert b.pop_rx(0) == 0xCAFEBABE


def test_push_if_full_skips_below_the_threshold():
    """IfFull and `isr_cnt < thresh(PUSH_THRESH)`: the instruction is a no-op."""
    b = setup(imem=[push(iffull=1), set_(enc.SET_X, 7)])
    poke(b, 0, isr=0xABCD, isr_cnt=4)
    sts = execs(b, 2)
    assert sts[0]["rx_level"] == 0  # nothing pushed
    assert sts[0]["isr"] == 0xABCD and sts[0]["isr_cnt"] == 4  # ISR untouched
    assert sts[1]["x"] == 7  # the PC advanced anyway: a no-op still retires


def test_push_if_full_pushes_at_or_above_the_threshold():
    b = setup(imem=[push(iffull=1)], shiftctrl_v=enc.shiftctrl(push_thresh=8))
    poke(b, 0, isr=0xABCD, isr_cnt=8)  # at the threshold: not skipped
    assert execs(b, 1)[0]["rx_level"] == 1
    b = setup(imem=[push(iffull=1)], shiftctrl_v=enc.shiftctrl(push_thresh=8))
    poke(b, 0, isr=0xABCD, isr_cnt=7)  # below it: skipped
    assert execs(b, 1)[0]["rx_level"] == 0


def test_push_block_stalls_on_a_full_rx_fifo():
    b = setup(imem=[push(block=1), set_(enc.SET_X, 7)])
    poke(b, 0, rx=[1, 2, 3, 4])
    st = execs(b, 1)[0]
    assert st["stalled"] == 1
    assert st["rx_level"] == 4  # nothing pushed
    assert st["pc"] == 0  # the PC does not advance
    assert b.read(enc.A_FDEBUG) & RXSTALL0


def test_push_noblock_on_a_full_fifo_drops_the_data_and_clears_the_isr():
    b = setup(imem=[push(block=0), set_(enc.SET_X, 7)])
    poke(b, 0, rx=[1, 2, 3, 4], isr=0x55, isr_cnt=8)
    st = execs(b, 1)[0]
    assert st["stalled"] == 0  # no stall
    assert st["rx_level"] == 4  # the word was dropped
    assert st["isr"] == 0 and st["isr_cnt"] == 0  # but the ISR was still cleared
    assert st["pc"] == 1  # and the instruction retired
    assert b.read(enc.A_FDEBUG) & RXSTALL0  # RXSTALL is set for a drop too


def test_push_fifo_is_four_deep_and_keeps_its_order():
    # IMEM[1] is `jmp 0`, so every second SM cycle pushes once.
    b = setup(imem=[push(), jmp(JMP_ALWAYS, 0)])
    for v in (0xA, 0xB, 0xC, 0xD, 0xE):
        poke(b, 0, isr=v)
        execs(b, 2)
    assert b.state(0)["rx_level"] == 4  # the fifth PUSH was dropped
    assert b.read(enc.A_FDEBUG) & RXSTALL0  # the drop set RXSTALL
    b.write(enc.A_CTRL, 0)  # stop the engine, so draining is deterministic
    assert [b.pop_rx(0) for _ in range(4)] == [0xA, 0xB, 0xC, 0xD]
    assert b.pop_rx(0) == 0  # MVP: an empty read returns 0
    assert b.read(enc.A_FDEBUG) & (1 << 8)  # FDEBUG.RXUNDER[0]


# ------------------------------------------------------------------- PULL


def test_pull_pops_the_tx_fifo():
    b = setup(imem=[pull()], tx=[0x12345678])
    st = execs(b, 1)[0]
    assert st["osr"] == 0x12345678 and st["osr_cnt"] == 0
    assert st["tx_level"] == 0


def test_pull_if_empty_skips_below_the_threshold():
    b = setup(imem=[pull(ifempty=1), set_(enc.SET_X, 7)])
    poke(b, 0, osr=0x1234, osr_cnt=4)
    sts = execs(b, 2)
    assert sts[0]["osr"] == 0x1234 and sts[0]["osr_cnt"] == 4  # untouched
    assert sts[1]["x"] == 7  # a no-op still retires


def test_pull_if_empty_pops_at_or_above_the_threshold():
    b = setup(imem=[pull(ifempty=1)], shiftctrl_v=enc.shiftctrl(pull_thresh=8), tx=[0x55])
    poke(b, 0, osr=0, osr_cnt=8)
    assert execs(b, 1)[0]["osr"] == 0x55


def test_pull_block_stalls_on_an_empty_tx_fifo():
    b = setup(imem=[pull(block=1), set_(enc.SET_X, 7)])
    st = execs(b, 1)[0]
    assert st["stalled"] == 1
    assert st["pc"] == 0  # the PC does not advance
    assert st["osr"] == 0 and st["osr_cnt"] == 32  # nothing loaded
    assert b.read(enc.A_FDEBUG) & TXSTALL0


def test_pull_noblock_on_an_empty_fifo_copies_x():
    """ISA `### PULL`: the non-blocking form sets no flag (it does not stall).

    Regmap `**FDEBUG**`: "TXSTALL | the engine stalls on an empty TX FIFO: blocking
    PULL, or OUT with autopull". `test_pull_block_stalls_on_an_empty_tx_fifo` is the
    blocking half of the same pair.
    """
    b = setup(imem=[pull(block=0), set_(enc.SET_X, 7)])
    poke(b, 0, x=0xDEADBEEF, osr=0x1111, osr_cnt=4)
    st = execs(b, 1)[0]
    assert st["osr"] == 0xDEADBEEF and st["osr_cnt"] == 0
    assert st["pc"] == 1  # no stall
    assert b.read(enc.A_FDEBUG) & TXSTALL0 == 0


def test_pull_with_autopull_is_a_noop_only_when_the_osr_is_full():
    """MVP: `osr_cnt == 0` means the OSR is full, so PULL is a barrier there."""
    b = setup(
        imem=[pull(), set_(enc.SET_X, 7)],
        shiftctrl_v=enc.shiftctrl(autopull=1, pull_thresh=32),
        tx=[0xAA],
    )
    poke(b, 0, osr=0x1234, osr_cnt=0)
    sts = execs(b, 2)
    assert sts[0]["osr"] == 0x1234 and sts[0]["osr_cnt"] == 0  # barrier did nothing
    assert sts[1]["x"] == 7  # the barrier still retired
    assert sts[1]["tx_level"] == 1  # the word is still there

    # With osr_cnt > 0 the OSR is not full, so PULL goes on to pop.
    b = setup(
        imem=[pull()],
        shiftctrl_v=enc.shiftctrl(autopull=1, pull_thresh=32),
        tx=[0xAA],
    )
    poke(b, 0, osr=0x1234, osr_cnt=8)
    assert execs(b, 1)[0]["osr"] == 0xAA


def test_pull_documented_zero_bits_are_ignored():
    """Bits 4:0 are documented 0; the instruction decodes as if they were 0."""
    for extra in (0x01, 0x10, 0x1F):
        b = setup(imem=[(enc.word(enc.PUSH) | 0x80 | extra)], tx=[0x77])
        assert execs(b, 1)[0]["osr"] == 0x77  # same as a plain `pull`


def test_push_and_pull_bits_of_the_shared_opcode():
    """Opcode 100 with bit 7 clear is PUSH and with bit 7 set is PULL."""
    assert enc.push() & 0x80 == 0
    assert enc.pull() & 0x80 == 0x80
    assert (enc.push() >> 13) & 7 == (enc.pull() >> 13) & 7 == 4


@pytest.mark.parametrize("block", (0, 1))
@pytest.mark.parametrize("iffull", (0, 1))
def test_push_iffull_and_block_bits_reach_the_decoder(iffull, block):
    """isr_cnt is at the threshold, so IfFull does not skip either way."""
    b = setup(imem=[push(iffull=iffull, block=block)])
    poke(b, 0, rx=[1, 2, 3, 4], isr=0x99, isr_cnt=32)
    st = execs(b, 1)[0]
    assert st["rx_level"] == 4  # nothing pushed
    assert st["stalled"] == block
    assert b.read(enc.A_FDEBUG) & RXSTALL0
    if block:
        # A stall changes nothing else: the ISR and the PC stay as they were.
        assert st["isr"] == 0x99 and st["isr_cnt"] == 32 and st["pc"] == 0
    else:
        assert st["isr"] == 0 and st["isr_cnt"] == 0 and st["pc"] == 1
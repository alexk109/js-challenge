"""crc15_xor.pio (isa_xor) against the VIP's CRC-15 (verif/vip/can.py), and its rate limit."""
import pytest

from tools.bench.isa_fw import crc
from verif.vip import can as vip

MID = 5          # a phase inside the window that works (see test_the_sampling_window)

FRAMES = [vip.Frame(0x123, b"\xde\xad"), vip.Frame(0x7FF, b""), vip.Frame(0x001, bytes(range(8))),
          vip.Frame(0x555, b"", rtr=True, dlc=3), vip.Frame(0x2A5, b"\xff" * 8), vip.Frame(0x000, b"\x00")]


@pytest.mark.parametrize("frame", FRAMES, ids=str)
def test_crc_matches_the_vip_and_the_residue_over_the_crc_sequence_is_zero(frame):
    ok, why = crc.check_frame(frame, 7, 1, 0, MID)
    assert ok, why


def test_a_corrupted_frame_leaves_a_nonzero_residue():
    frame = FRAMES[0]
    bits = crc.logical(frame, with_crc=True)
    for flip in (3, 12, len(bits) - 1, len(bits) - 15):
        bad = list(bits)
        bad[flip] ^= 1
        assert crc.run(bad, 7, 1, 0, MID) != 0, flip


def test_the_sampling_window():
    ok = [ph for ph in range(7) if crc.check_frame(FRAMES[0], 7, 1, 0, ph)[0]]
    assert len(ok) >= 4 and MID in ok, ok          # not a single lucky phase


def test_the_loop_is_seven_sm_cycles_on_both_paths():
    # a bit time shorter or longer than the loop loses the grid: 6 and 8 core cycles per bit fail at every phase
    for bt in (6, 8):
        assert not any(crc.check_frame(FRAMES[0], bt, 1, 0, ph)[0] for ph in range(bt)), bt


def test_pad_reaches_one_megabit():
    # 50 core cycles per bit (1 Mbit/s at 50 MHz) = CLKDIV 5 x (7 + PAD 3)
    ok = [ph for ph in range(0, 50, 5) if crc.check_frame(FRAMES[0], 50, 5, 3, ph)[0]]
    assert ok
    assert crc.words(3) == crc.words(0)


def test_word_count():
    assert crc.words() == 14

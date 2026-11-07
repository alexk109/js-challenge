"""Run fw `crc15_xor.pio` on the parametric ISS (isa_xor) against a CAN frame from verif/vip/can.py.

The VIP builds the frame and its CRC (`crc15`, Bosch's register verbatim); the firmware has to agree with it, bit by
bit, at the loop's own bit time.  The stimulus is the destuffed logical stream (the destuffer is a separate,
unshipped 20-word program), one bit per `bit_ticks` core cycles on pin 0, and a frame-over signal on pin 1.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

from asm.pioasm import Program, assemble
from tools.bench.isa_fw.rig import Rig
from verif.vip import can as vip

HERE = Path(__file__).resolve().parent
PIO = HERE / "crc15_xor.pio"
POLY_LJ = vip.CRC_POLY << 17            # generator, left-justified in 32 bits
LOOP_SM_CYCLES = 7                      # the loop's SM cycles per bit (PAD = 0)


def program(pad: int = 0) -> Program:
    text = PIO.read_text().replace(".define PAD 0", f".define PAD {pad}")
    return assemble(text, ("xor",))[0]


def logical(frame: vip.Frame, with_crc: bool) -> list[int]:
    bits, roles = vip._logical_bits(frame)
    n = roles.index("crc") if not with_crc else roles.index("crc_delim")
    return bits[:n]


def _entry_time(prog: Program, clkdiv: int) -> int:
    """Core cycle (rig time) at which the engine first reaches `bit` (the stimulus does not matter to the setup)."""
    rig = Rig(prog, ("xor",), clkdiv=clkdiv, jmp_pin=1, in_base=0, shiftctrl=0)
    rig.push(POLY_LJ)
    rig.go()
    for _ in range(200):
        rig.step()
        if rig.eng.pc == prog.labels["bit"]:
            return rig.t
    raise RuntimeError("engine never reached the loop")


def run(bits: Sequence[int], bit_ticks: int, clkdiv: int = 1, pad: int = 0, phase: int = 0) -> Optional[int]:
    """CRC register (15 bits) the firmware reports after `bits`, or None if it never reported.

    `phase` (0..bit_ticks-1) is where inside its bit window the loop's `in pins` falls; the frame-over pin rises
    at the start of the window after the last bit.  Line idle (before the frame) is recessive (1).
    """
    prog = program(pad)
    t_entry = _entry_time(prog, clkdiv)
    rig = Rig(prog, ("xor",), clkdiv=clkdiv, jmp_pin=1, in_base=0, shiftctrl=0)
    n = len(bits)
    t0 = t_entry - 1 - phase + clkdiv      # `in pins` of the first iteration reads the stimulus of t_entry + 2*clkdiv - 3

    def stim(t: int) -> int:
        if t < t0:
            return 1
        k = (t - t0) // bit_ticks
        return 3 if k >= n else bits[k] | 2 * 0

    rig.stim = stim
    rig.push(POLY_LJ)
    rig.go()
    rig.run((n + 3) * bit_ticks + 60 + t_entry)
    out = rig.drain()
    return None if not out else out[0] >> 17


def words(pad: int = 0) -> int:
    return len(program(pad).instrs)


def check_frame(frame: vip.Frame, bit_ticks: int = 7, clkdiv: int = 1, pad: int = 0, phase: int = 0) -> tuple[bool, str]:
    """True when the firmware's CRC over SOF..data equals the VIP's, and over SOF..CRC SEQUENCE is 0."""
    body = logical(frame, with_crc=False)
    full = logical(frame, with_crc=True)
    want = vip.crc15(body)
    got_tx, got_rx = run(body, bit_ticks, clkdiv, pad, phase), run(full, bit_ticks, clkdiv, pad, phase)
    ok = got_tx == want and got_rx == 0
    return ok, f"crc want {want:#06x} got {got_tx}; residue over CRC SEQUENCE got {got_rx}"

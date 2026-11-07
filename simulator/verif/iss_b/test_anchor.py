"""The spec's `### Anchor trace (normative example)`, cycle for cycle.

ISA `## Timing`, the example: engine 0, CLKDIV = 1.0, SET_BASE = 0, SET_COUNT = 1,
WRAP_BOTTOM = 1, WRAP_TOP = 2, IMEM 0/1/2 = `set pindirs,1` / `set pins,1` /
`set pins,0`, and a `CTRL = 0x101` write in cycle `h`.

The testbench model for the pads is the one the spec states for this example:
"here `uio_in` follows the pad, and an unloaded pad follows `uio_out`", so
`pin_in` bit `k` is the pad value of the cycle being stepped, which for an
unloaded `uio` pin is the level the chip is driving.
"""

from verif.iss_b import enc
from verif.iss_b.enc import Bus, jmp, mov, out, set_

IMEM = {0: 0xE081, 1: 0xE001, 2: 0xE000}  # verbatim from the spec's table

# (cycle offset from h, uio_oe[0], uio_out[0], instruction executed, PC after)
TRACE = [
    (1, 0, 0, None, 0),
    (2, 0, 0, (0, 0xE081), 1),
    (3, 1, 0, (1, 0xE001), 2),
    (4, 1, 1, (2, 0xE000), 1),  # PC 2 == WRAP_TOP, so it wraps to WRAP_BOTTOM
    (5, 1, 0, (1, 0xE001), 2),
]


def _start_engine_0() -> tuple[Bus, int]:
    """Everything the spec says to write before cycle `h`, then the CTRL write."""
    b = Bus()
    b.cfg(
        0,
        clkdiv_v=enc.clkdiv(1, 0),
        execctrl_v=enc.execctrl(wrap_bottom=1, wrap_top=2),
        pinctrl_v=enc.pinctrl(set_base=0, set_count=1),
    )
    b.load_imem(0, IMEM)
    b.idle(2)  # all IMEM words land before cycle h-1
    h = b.cycle
    b.write(enc.A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    assert b.cycle == h + 1  # the CTRL write is cycle h
    return b, h


def test_anchor_trace_cycle_for_cycle():
    b, h = _start_engine_0()

    for off, oe, out_level, executed, pc_after in TRACE:
        assert b.cycle == h + off, f"cycle {b.cycle} is not h+{off}"
        # The attributes before the step are what is on the pad in this cycle.
        assert b.pad_oe(0) == oe, f"h+{off}: uio_oe[0] is {b.pad_oe(0)}, spec says {oe}"
        assert b.pad_out(0) == out_level, f"h+{off}: uio_out[0] is {b.pad_out(0)}"

        before = b.state(0)
        if executed is not None:
            addr, instr = executed
            # The word executed in cycle n is IMEM[PC] as IMEM stood in cycle n-1.
            assert before["pc"] == addr, f"h+{off}: PC is {before['pc']}, spec says {addr}"
            assert before["ir"] == instr, f"h+{off}: IR is not {instr:#06x}"
        b.step(pin_in=b.chip.pin_out & 0xFF)  # the pad follows uio_out

        st = b.state(0)
        assert st["pc"] == pc_after, f"h+{off}: PC is {st['pc']}, spec says {pc_after}"
        assert st["ir"] == IMEM[pc_after], f"h+{off}: IR is not IMEM[{pc_after}]"
        if executed is None:
            assert (before["pc"], before["ir"]) == (0, IMEM[0])  # en was 0: no SM cycle


def test_anchor_pin_reads_agree_with_pad_attributes():
    """`DBG_PADOUT`/`DBG_PADOE` read the same registers the attributes show.

    One access per cycle, so each register gets its own pass: the attribute is
    captured in the cycle before the read, and the read is combinational.
    """
    for addr, attr in ((enc.A_PADOUT, "pin_out"), (enc.A_PADOE, "pin_oe")):
        b, _h = _start_engine_0()
        for _ in range(6):
            b.step(pin_in=b.chip.pin_out & 0xFF)
            expect = getattr(b.chip, attr)
            assert b.read(addr) == expect, f"{addr:#04x} read != {attr}"


def test_anchor_traces_three_cycle_input_pipeline():
    """ISA: "Engine 0 then sees its own `uio[0]` level from cycle `h+4` in its
    `IN_VEC` from cycle `h+7`." T_IN = 3: pad of cycle n-3 at cycle n."""
    b = Bus()
    b.cfg(0, clkdiv_v=enc.clkdiv(1, 0))
    # PC 0 drives pin 0 high, PC 1 samples all inputs, PC 2 loops back to PC 1.
    b.load_imem(0, [set_(enc.SET_PINS, 1), mov(enc.MOV_X, enc.MOVSRC_PINS), jmp(enc.JMP_ALWAYS, 1)])
    b.idle(2)
    b.write(enc.A_CTRL, enc.ctrl(sm_enable=1, clkdiv_restart=1))
    h = b.cycle - 1

    pad_history = [0] * b.cycle  # every pad was still 0 during the setup cycles
    for _ in range(12):
        c = b.cycle
        pad = b.chip.pin_out & 0xFF  # the pad of cycle c
        st = b.state(0)
        b.step(pin_in=pad)
        pad_history.append(pad)
        if st["pc"] == 1:  # `mov x, pins` executed in cycle c
            expect = pad_history[c - 3] & 1 if c >= 3 else 0
            assert b.state(0)["x"] == expect, (
                f"cycle {c}: X is {b.state(0)['x']:#x}, IN_VEC should carry the pad "
                f"of cycle {c - 3} ({expect})"
            )
    # The drive starts in cycle h+2 (cycle 1 is the first SM cycle), so the pad is
    # high from h+3 and IN_VEC from h+6; nothing may be seen earlier.
    assert h == b.cycle - 12 - 1
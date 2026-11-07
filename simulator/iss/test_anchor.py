"""docs/mvp-isa.md `### Anchor trace (normative example)`, cycle for cycle."""
from iss.testkit import CTRL, H, execctrl, imem_addr, mov, pinctrl, set_, sm


def _setup(h: H, engine1_prog=None):
    h.load(0, [0xE081, 0xE001, 0xE000])
    h.w(sm(0, "PINCTRL"), pinctrl(set_base=0, set_count=1))
    h.w(sm(0, "EXECCTRL"), execctrl(wrap_bottom=1, wrap_top=2))
    if engine1_prog is not None:
        h.load(1, engine1_prog)
        h.w(sm(1, "EXECCTRL"), execctrl(wrap_bottom=0, wrap_top=0))
    h.idle(2)  # IMEM written before cycle h-1


def test_encodings_used_by_the_anchor():
    assert [set_("pindirs", 1), set_("pins", 1), set_("pins", 0)] == [0xE081, 0xE001, 0xE000]


def test_anchor_trace():
    h = H()
    _setup(h)
    hc = h.cycle
    h.w(CTRL, 0x101)                       # cycle h
    eng = h.c.engines[0]
    # rows: cycle, en, pc executed (None = nothing), uio_oe[0], uio_out[0]  (values *in* the cycle)
    table = [
        (hc + 1, 0, None, 0, 0),
        (hc + 2, 1, 0, 0, 0),
        (hc + 3, 1, 1, 1, 0),
        (hc + 4, 1, 2, 1, 1),
        (hc + 5, 1, 1, 1, 0),
    ]
    for cyc, en, pc_exec, oe, lvl in table:
        assert h.cycle == cyc
        assert eng.en == en, cyc - hc
        assert h.c.pin_oe & 1 == oe, cyc - hc
        assert h.c.pin_out & 1 == lvl, cyc - hc
        pc_before = h.s(0)["pc"]
        h.tick()
        if pc_exec is None:
            assert h.s(0)["pc"] == pc_before == 0
        else:
            assert pc_before == pc_exec
    # after h+4 (PC 2, wraps to 1) and h+5 (PC 1) the next PC is 2
    assert h.s(0)["pc"] == 2


def test_anchor_input_loopback_seen_from_h_plus_7():
    """Engine 0 sees its own uio[0] level of cycle h+4 in IN_VEC from cycle h+7 (unloaded pad:
    uio_in follows uio_out)."""
    h = H()
    _setup(h)
    hc = h.cycle
    h.w(CTRL, 0x101)
    pad = {}
    seen = {}
    while h.cycle <= hc + 12:
        cyc = h.cycle
        pad[cyc] = h.c.pin_out & 1
        h.tick()
        seen[cyc] = h.c.last_in_vec & 1
    assert pad[hc + 3] == 0 and pad[hc + 4] == 1 and pad[hc + 5] == 0
    for cyc in range(hc + 4, hc + 13):
        assert seen[cyc] == pad[cyc - 3], cyc - hc
    assert seen[hc + 7] == 1 and seen[hc + 6] == 0


def test_anchor_loopback_observed_by_engine_1():
    """Same loop observed architecturally: engine 1 runs `mov isr, pins` every cycle."""
    h = H()
    _setup(h, engine1_prog=[mov("isr", "pins")])
    hc = h.cycle
    h.w(CTRL, 0x303)
    pad = {}
    while h.cycle <= hc + 12:
        cyc = h.cycle
        pad[cyc] = h.c.pin_out & 1
        h.tick()
        if cyc >= hc + 2:
            assert h.s(1)["isr"] & 1 == pad.get(cyc - 3, 0), cyc - hc
    # engine 0 still follows the anchor table
    assert h.s(0)["pc"] in (1, 2)

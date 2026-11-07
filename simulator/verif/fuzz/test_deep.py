"""The pure-Python half of the deep lockstep, without a simulator.

`make fuzz-deep-pytest`. The simulator-backed half is `make fuzz-selftest`; what is
checkable without a simulator is checked here, because these are the parts a reader is most likely
to doubt:

* the pad model, with and without the pull-up mode, and the negative control (`pad_value(fault=)`)
  that the pinloop stimulus has to catch;
* the two unpacking functions, against the read mux's own packing and against the spec's reset
  values, so a reader can see that they invert a documented packing rather than invent one;
* the field list: 37 shallow fields (the number docs/mvp-verification-report.md quotes), every deep
  field exactly once, and the same order the probe produces;
* the stimuli: the guaranteed pin conflict really does make the two engines ask for different things
  to the same pin in the same cycle, and `fifofill()` really does leave both TX FIFOs full.

    .venv/bin/python -m pytest -q verif/fuzz/test_deep.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from iss.pio_iss import Chip                                        # noqa: E402
from verif.fuzz import gen, lockstep                                # noqa: E402


# ---------------------------------------------------------------------------------------
# the pad model
# ---------------------------------------------------------------------------------------

def test_pad_follows_the_driven_level():
    """A `uio` pin the chip drives carries the chip's own level, whatever the world offers."""
    assert lockstep.pad_value(0x00, pin_out=0x01, pin_oe=0x01) == 0x01
    assert lockstep.pad_value(0x01, pin_out=0x01, pin_oe=0x01) == 0x01     # world says 0
    assert lockstep.pad_value(0x01, pin_out=0x00, pin_oe=0x01) == 0x00     # ... and the chip says 0


def test_pad_carries_the_world_where_the_chip_releases():
    assert lockstep.pad_value(0x05, pin_out=0xFF, pin_oe=0x00) == 0x05
    # only pins the world can drive; 13..17 are the chip's own and it feeds them back itself
    assert lockstep.pad_value(0x3FFFF, pin_out=0, pin_oe=0) == 0x1FFF


def test_pad_pullup_only_lifts_released_uio_pins():
    # every uio pin released, nothing driven: all eight read 1
    assert lockstep.pad_value(0x00, pin_out=0x00, pin_oe=0x00, pullup=True) == gen.UIO_MASK
    # an input-only pin has no direction register, so a pull-up on that net is the world's business
    assert lockstep.pad_value(0x00, pin_out=0x00, pin_oe=0x00, pullup=True) & gen.IN_ONLY_MASK == 0
    # pins 0 and 1 driven (pin 0 low, pin 1 high), pins 2..7 released and pulled up
    lifted = lockstep.pad_value(0x00, pin_out=0x02, pin_oe=0x03, pullup=True)
    assert lifted == 0xFE, hex(lifted)
    assert (lifted >> 0) & 1 == 0 and (lifted >> 1) & 1 == 1


def test_pad_fault_is_the_negative_control_and_the_rule_catches_it():
    """`FUZZ_PAD_FAULT` is what `make fuzz-selftest` uses. It cannot make the two sides disagree --
    both are handed the same `pin_in` -- so the thing that has to catch it is `pad_violations`,
    which checks the pad against the rule instead of trusting `pad_value`."""
    ext, out, oe = 0x00, 0x01, 0x01
    good = lockstep.pad_value(ext, pin_out=out, pin_oe=oe)
    bad = lockstep.pad_value(ext, pin_out=out, pin_oe=oe, fault=True)
    assert good == 0x01 and bad == 0x00
    assert lockstep.pad_violations(ext, good, out, oe, False) == []
    assert lockstep.pad_violations(ext, bad, out, oe, False) == [(0, 0, 1)]
    # the pull-up rule is part of it: a released pin has to read 1 when the net has a pull-up
    assert lockstep.pad_rule(0x00, 0x00, 0x00, True) == [1] * 8
    assert lockstep.pad_rule(0x00, 0x00, 0x00, False) == [0] * 8
    assert lockstep.pad_violations(0x00, lockstep.pad_value(0x00, 0, 0, pullup=True), 0, 0,
                                   True) == []
    assert lockstep.pad_violations(0x00, lockstep.pad_value(0x00, 0, 0, pullup=True), 0, 0,
                                   False) == [(k, 1, 0) for k in range(8)]


# ---------------------------------------------------------------------------------------
# the unpacking
# ---------------------------------------------------------------------------------------

def irq_word(b: int) -> int:
    """`src/pio_chip.sv`'s `irq_word()`, transcribed."""
    return ((b >> 2) & 0x3) << 4 | (b & 0x3)


def lanes(*b: int) -> int:
    """`src/pio_chip.sv`'s `lanes()`, transcribed."""
    out = 0
    for i, value in enumerate(reversed(b)):
        out |= (value & 0x3) << (8 * i)
    return out


def test_unpack_irq_inverts_the_read_mux():
    for compact in range(16):
        assert lockstep.unpack_irq(irq_word(compact)) == compact


def test_unpack_irq_matches_the_spec_reset_values():
    """docs/mvp-regmap.md `## Registers`: IRQ0_INTE resets to 0x003 and IRQ1_INTE to 0x010."""
    chip = Chip()
    assert chip.peek(gen.A_IRQ0_INTE) == 0x003
    assert chip.peek(gen.A_IRQ1_INTE) == 0x010
    assert lockstep.unpack_irq(chip.peek(gen.A_IRQ0_INTE)) == 0b0011
    assert lockstep.unpack_irq(chip.peek(gen.A_IRQ1_INTE)) == 0b0100


def test_unpack_fdebug_inverts_the_read_mux():
    for flags in range(256):
        word = lanes(flags >> 6 & 3, flags >> 4 & 3, flags >> 2 & 3, flags & 3)
        assert lockstep.unpack_fdebug(word) == flags


def test_unpack_fdebug_matches_the_iss_peek():
    """`dut.fdebug` is eight bits: {TXSTALL, TXOVER, RXUNDER, RXSTALL}, two engines each, from the
    top. The ISS keeps the same eight flags at bits 24, 16, 8 and 0 and builds the read word from
    them, so the inverse of the read mux must give the byte back."""
    chip = Chip()
    for flags in range(256):                       # all 256 combinations of the eight flags
        chip.fdebug = ((flags >> 6 & 3) << 24) | ((flags >> 4 & 3) << 16) \
            | ((flags >> 2 & 3) << 8) | (flags & 3)
        assert lockstep.unpack_fdebug(chip.peek(gen.A_FDEBUG)) == flags


# ---------------------------------------------------------------------------------------
# the field list
# ---------------------------------------------------------------------------------------

def test_shallow_field_count_is_the_documented_one():
    """docs/mvp-verification-report.md `### Known holes in the stimulus` quotes 37 named signals."""
    assert len(lockstep.field_names(False)) == 37


def test_deep_field_list_has_no_duplicates_and_is_longer():
    names = lockstep.field_names(True)
    assert len(names) == len(set(names))
    assert len(names) > 37
    for key in ("sm_enable", "fdebug", "inte0", "intf1", "in_vec", "in_pipe0", "res_level",
                "res_dir", "eng0.clkdiv", "eng0.div_acc", "eng1.tx[3]", "eng0.imem[31]",
                "eng1.ss_mask"):
        assert key in names


def test_deep_is_the_default_and_the_flag_can_turn_it_off(monkeypatch):
    assert lockstep.DEEP_DEFAULT is True, (
        "--deep became the default once seeds 1-60 at 20k cycles were clean")
    monkeypatch.delenv("FUZZ_DEEP", raising=False)
    assert lockstep.deep_enabled() is True
    monkeypatch.setenv("FUZZ_DEEP", "0")
    assert lockstep.deep_enabled() is False
    monkeypatch.setenv("FUZZ_DEEP", "1")
    assert lockstep.deep_enabled() is True


def test_pullup_flag_is_off_unless_asked(monkeypatch):
    monkeypatch.delenv("FUZZ_PULLUP", raising=False)
    assert lockstep.pullup_enabled() is False
    monkeypatch.setenv("FUZZ_PULLUP", "1")
    assert lockstep.pullup_enabled() is True


# ---------------------------------------------------------------------------------------
# the stimuli
# ---------------------------------------------------------------------------------------

def _requests(chip, engine):
    """The two pin write requests engine `engine` made in the last stepped cycle."""
    req = chip.last_req[engine]
    return [req["out"], req["side"]]


def test_conflict_phase_makes_the_two_engines_contest_one_pin():
    """The plant-5 claim needs a stimulus in which the cross-engine rule is *reached*."""
    stim = gen.gen(7, 600, conflict=256)
    assert stim.summary["host_ops"].get("conflict_phase") == 256
    chip = Chip()
    ops = {c: (k, a, d) for c, k, a, d in stim.ops}
    contested = 0
    for cycle in range(stim.cycles):
        pin_in = lockstep.pad_value(gen.pin_vector(stim.seed, cycle) & ~stim.ext_low,
                                    chip.pin_out, chip.pin_oe)
        bus = ops.get(cycle)
        running = [bool(e.force or ((chip.sm_enable >> i) & 1 and e.en))
                   for i, e in enumerate(chip.engines)]
        chip.step(pin_in, None if bus is None else
                  (("w", bus[1], bus[2]) if bus[0] == "w" else ("r", bus[1])))
        if not all(running):
            continue
        reqs = [_requests(chip, 0), _requests(chip, 1)]
        for i in range(2):
            for j in range(2):
                a, b = reqs[0][i], reqs[1][j]
                if a and b and a[0] == b[0]:                 # same register: level or direction
                    mask = a[1] & b[1] & 0x3FFFF
                    if mask and (a[2] ^ b[2]) & mask:
                        contested += 1
    assert contested >= 200, f"only {contested} contested pin-cycles"


def test_fifofill_leaves_both_tx_fifos_full_and_never_drains_them():
    stim = gen.fifofill(cycles=120)
    chip = Chip()
    ops = {c: (k, a, d) for c, k, a, d in stim.ops}
    for cycle in range(stim.cycles):
        bus = ops.get(cycle)
        chip.step(0, None if bus is None else
                  (("w", bus[1], bus[2]) if bus[0] == "w" else ("r", bus[1])))
    for e in range(2):
        assert len(chip.engines[e].tx) == 4, f"engine {e} TX FIFO holds {len(chip.engines[e].tx)}"
        assert len(chip.engines[e].rx) == 0


def test_fifofill_reaches_the_fields_the_deep_negative_controls_need():
    """`plant_iss_deep` is only allowed to break a field that this stimulus reaches."""
    stim = gen.fifofill(cycles=120)
    chip = Chip()
    ops = {c: (k, a, d) for c, k, a, d in stim.ops}
    seen = {"div_reload": False, "out_request": False, "imem_word_17": False, "clkdiv_frac": False}
    for cycle in range(stim.cycles):
        bus = ops.get(cycle)
        chip.step(0, None if bus is None else
                  (("w", bus[1], bus[2]) if bus[0] == "w" else ("r", bus[1])))
        eng = chip.engines[0]
        if eng.en and ((eng.clkdiv >> 8) & 0xFF):
            seen["div_reload"] = True
        if chip.last_req[0]["out"] is not None:
            seen["out_request"] = True
        if eng.imem[17]:
            seen["imem_word_17"] = True
        if (eng.clkdiv >> 8) & 0xFF == 0x41:
            seen["clkdiv_frac"] = True
    assert all(seen.values()), seen


def test_stimulus_json_round_trips_the_mode_and_the_ext_low():
    stim = gen.pinloop(cycles=100)
    stim.mode = {"deep": True, "pullup": True}
    again = gen.Stimulus.from_json(stim.to_json())
    assert again.mode == {"deep": True, "pullup": True}
    assert again.ext_low == stim.ext_low == gen.PIN_MASK
    assert again.ops == stim.ops
    # a stimulus that records no mode (every older repro) must not pin one: the defaults govern
    import json
    raw = json.loads(stim.to_json())
    raw.pop("mode")
    raw.pop("ext_low")
    old = gen.Stimulus.from_json(json.dumps(raw))
    assert old.mode == {} and old.ext_low == 0


def test_gen_without_conflict_is_unchanged():
    """`conflict=0` must produce exactly the stimulus `gen()` always produced, so that
    `metrics/fuzz.csv` and every committed repro stay reproducible."""
    assert gen.gen(3, 500).ops == gen.gen(3, 500, conflict=0).ops
    assert gen.gen(3, 500).cycles == gen.gen(3, 500, conflict=64).cycles == 500


# ---------------------------------------------------------------------------------------
# the ISS side of the deep comparison
# ---------------------------------------------------------------------------------------

def test_deep_state_is_additive_and_shaped_as_documented():
    chip = Chip()
    deep = chip.deep_state()
    assert set(deep) == {"in_vec", "res_level", "res_dir", "req", "0", "1"}
    for e in ("0", "1"):
        assert set(deep[e]) == {"tx", "rx", "imem", "div"}
        assert len(deep[e]["imem"]) == gen.IMEM_SIZE
        assert len(deep[e]["div"]) == 3
    assert len(deep["req"]) == 2
    assert chip.state(0).keys() == set(lockstep.STATE_KEYS), (
        "docs/mvp-interfaces.md `## ISS API` fixes the keys of state(); deep_state() adds nothing")


def test_deep_state_in_vec_is_the_current_cycle_s_input():
    """`last_in_vec` is the number of the cycle that was just stepped, which is one step behind
    between steps; the harness reads the state between steps, so `deep_state()` must report
    `sync[-1]`, the IN_VEC of the cycle in progress (T_IN = 3)."""
    chip = Chip()
    for cycle in range(8):
        assert chip.deep_state()["in_vec"] == (1 if cycle >= 3 else 0), cycle
        chip.step(1)


def test_deep_state_follows_a_host_fifo_write_and_an_imem_write():
    chip = Chip()
    chip.step(0, ("w", gen.A_TXF[0], 0x1234_5678))
    chip.step(0, ("w", gen.A_TXF[0], 0x9ABC_DEF0))
    chip.step(0, ("w", gen.A_IMEM[0] + 17, 0xA5A5))
    deep = chip.deep_state()
    assert deep["0"]["tx"] == [0x1234_5678, 0x9ABC_DEF0]
    assert deep["0"]["imem"][17] == 0xA5A5
    assert chip.state(0)["tx_level"] == 2


def test_deep_state_reports_this_cycle_s_pin_requests():
    """The requests are a hole in the shallow set: a side-set that rewrites the same value is
    invisible on `pin_out` and visible here."""
    chip = Chip()
    for e, word in enumerate([gen.set_(gen.SET_PINS, 1), gen.set_(gen.SET_PINS, 0)]):
        chip.step(0, ("w", gen.A_SM_BASE + 6 * e + gen.SM_PINCTRL, 1 << 26))   # SET_COUNT = 1
        chip.step(0, ("w", gen.A_SM_BASE + 6 * e + gen.SM_INSTR, word))        # force it
        before = chip.pin_out
        chip.step(0)                       # this engine executes the forced SET in this cycle
        deep = chip.deep_state()
        want = 1 if e == 0 else 0
        assert deep["req"][e]["out"] == (0, 0x1, want), deep["req"][e]
        # the requests of this cycle, and what they resolve to: the same number the RTL holds in
        # its combinational `lvl_n` during this cycle
        assert deep["res_level"] == want, hex(deep["res_level"])
        assert chip.pin_out == want, "the register takes it at the end of the cycle (T_OUT = 1)"
        if e == 0:
            assert before == 0
    # engine 1 wrote pin 0 low in a later cycle than engine 0 wrote it high, and the two never met
    # in one cycle: nothing here says anything about the cross-engine order, which is what
    # `gen.conflict_phase` is for


# ---------------------------------------------------------------------------------------
# the probe: the field list and the reads have to agree without a simulator
# ---------------------------------------------------------------------------------------

class _Signal:
    """A stand-in for a cocotb handle: `handle.value` is all the probe ever asks of one."""

    def __init__(self, value: int = 0) -> None:
        self._value = value

    @property
    def value(self) -> int:
        return self._value


class _FakeEngine:
    def __init__(self) -> None:
        for name in ("r_clkdiv", "r_execctrl", "r_shiftctrl", "r_pinctrl", "tx_head", "rx_head"):
            setattr(self, name, _Signal())
        self.u_div = type("_Div", (), {"cnt": _Signal(), "acc": _Signal(), "en": _Signal()})()
        self.u_txf = type("_Fifo", (), {"rp": _Signal(), "head": _Signal(),
                                        "mem": [_Signal() for _ in range(4)]})()
        self.u_rxf = type("_Fifo", (), {"rp": _Signal(), "head": _Signal(),
                                        "mem": [_Signal() for _ in range(4)]})()
        self.imem = [_Signal() for _ in range(gen.IMEM_SIZE)]
        for name in lockstep.ENGINE_REQ_KEYS:
            setattr(self, name, _Signal())


class _FakeDut:
    def __init__(self) -> None:
        for name in lockstep.OUTPUT_KEYS + lockstep.CHIP_DEEP_KEYS + ("bus_rdata", "lvl_n",
                                                                      "dir_n"):
            setattr(self, name, _Signal())
        self.in_pipe = [_Signal() for _ in range(3)]
        self.g_eng = [type("_G", (), {"u_eng": _FakeEngine()})() for _ in range(gen.NUM_ENGINES)]


def test_probe_reads_exactly_the_fields_the_report_names():
    """`History.add` asserts the lengths, but only inside a simulator run; this is the same check
    without one, and it also pins the order (the report prints the fields in it)."""
    dut = _FakeDut()
    engines = [dut.g_eng[e].u_eng for e in range(gen.NUM_ENGINES)]
    probe = lockstep.DeepProbe(dut, engines)
    names = lockstep.field_names(True)
    shallow = lockstep.field_names(False)

    def full(fields) -> list[str]:
        return [f"eng{e}.{n}" if e is not None else n for n, e, _, _ in fields]

    state_names = full(probe.state(Chip()))
    request_names = full(probe.requests(Chip()))
    # the deep state goes where the shallow fields stop, the requests after `bus_rdata` (which the
    # harness reads itself), and between them they are every field the report names
    read = names.index(lockstep.READ_KEY)
    assert state_names == names[len(shallow) - 1:read]
    assert request_names == names[read + 1:]
    assert len(state_names) + len(request_names) + len(shallow) == len(names)


def test_probe_uses_only_what_deep_state_offers():
    """Every key `DeepProbe` reads out of the ISS has to be there, so a rename on either side
    fails here instead of in a 20k-cycle run."""
    deep = Chip().deep_state()
    for key in ("in_vec", "res_level", "res_dir", "req"):
        assert key in deep
    for e in range(gen.NUM_ENGINES):
        for key in ("tx", "rx", "imem", "div"):
            assert key in deep[str(e)]


def test_probe_compares_only_the_write_flag_of_a_silent_channel():
    """`ss_dir` is `SIDE_PINDIR` and `ss_mask` is the mask the side-set *would* use, both of which
    are non-zero without a side-set; comparing them would be comparing the configuration, which the
    EXECCTRL field already does. So a channel that does not write is compared on its write flag
    alone, whatever the other three RTL signals hold."""
    dut = _FakeDut()
    eng = dut.g_eng[0].u_eng
    eng.ss_dir, eng.ss_mask, eng.ss_data = _Signal(1), _Signal(0x3F), _Signal(0xABCD)
    values = {f"eng{e}.{n}" if e is not None else n: (rtl, iss) for n, e, rtl, iss in
              lockstep.DeepProbe(dut, [dut.g_eng[e].u_eng for e in range(2)]).requests(Chip())}
    assert values["eng0.ss_we"] == (0, 0), "nothing wrote, so nothing may claim to"
    for key in ("eng0.ss_dir", "eng0.ss_mask", "eng0.ss_data"):
        assert values[key] == (0, 0), key
    # and the OUT channel's write flag is compared too, in both directions
    assert values["eng0.os_we"] == (0, 0)
    assert "res_level" in values and "res_dir" in values


def test_probe_reads_the_fifo_ring_from_the_read_pointer():
    """The ISS lists a FIFO oldest word first; the RTL stores it in a ring, so the probe has to
    start at `rp` or every word would look wrong."""
    dut = _FakeDut()
    eng = dut.g_eng[0].u_eng
    eng.u_txf.rp = _Signal(2)
    for k, word in enumerate([0x10, 0x20, 0x30, 0x40]):
        eng.u_txf.mem[k] = _Signal(word)
    chip = Chip()
    for k, word in enumerate([0x10, 0x20, 0x30, 0x40]):
        chip.step(0, ("w", gen.A_TXF[0], word))
    chip.engines[0].tx.pop(0)                       # one word out, so rp is 1 on both sides
    dut.g_eng[0].u_eng.u_txf.rp = _Signal(1)
    values = {f"eng{e}.{name}": (rtl, iss) for name, e, rtl, iss in
              lockstep.DeepProbe(dut, [dut.g_eng[e].u_eng for e in range(2)]).state(chip)}
    assert values["eng0.tx[0]"] == (0x20, 0x20)
    assert values["eng0.tx[2]"] == (0x40, 0x40)
    assert values["eng0.tx[3]"] == (0, 0), "a word outside the queue is not compared"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))

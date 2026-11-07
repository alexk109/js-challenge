"""The ISS API is exactly the one in `docs/mvp-interfaces.md` `### ISS API`.

The other ISS and the fuzz harness are written against that API, so a signature or
attribute drift here breaks the differential comparison in a confusing way.
"""

import ast
import inspect
from pathlib import Path

import pytest

from verif.iss_b.pio_iss_b import Chip

MODULE = Path(__file__).with_name("pio_iss_b.py")
STATE_KEYS = [
    "pc",
    "ir",
    "x",
    "y",
    "osr",
    "isr",
    "osr_cnt",
    "isr_cnt",
    "delay_cnt",
    "stalled",
    "latch",
    "latch_pend",
    "latch_host",
    "forced",
    "tx_level",
    "rx_level",
]


def test_constructor_and_reset_take_no_arguments():
    assert str(inspect.signature(Chip.__init__)) == "(self) -> None"
    assert str(inspect.signature(Chip.reset)) == "(self) -> None"
    Chip().reset()


def test_step_signature_and_return_type():
    assert str(inspect.signature(Chip.step)) == (
        "(self, pin_in: int, bus: tuple | None = None) -> int | None"
    )
    c = Chip()
    assert c.step(0) is None  # no bus: no read data
    assert c.step(0, ("w", 0x04, 0x1234)) is None  # a write returns None
    assert c.step(0, ("r", 0x08)) == 0  # a read returns the data


def test_step_accepts_a_list_for_the_bus_argument():
    c = Chip()
    assert c.step(0, ["r", 0x08]) == 0


def test_pin_in_is_taken_modulo_the_eighteen_pin_space():
    c = Chip()
    c.step(0, ("r", 0x08))
    c.step(1 << 40)  # a wider vector is masked, not an error
    assert c.pin_out < (1 << 18)


def test_the_output_attributes_exist_and_are_integers():
    c = Chip()
    for name in ("pin_out", "pin_oe", "irq0", "irq1"):
        value = getattr(c, name)
        assert isinstance(value, int), name
    assert isinstance(c.step(0), (type(None), int))


def test_state_takes_an_engine_index_and_returns_the_exact_keys():
    c = Chip()
    assert str(inspect.signature(Chip.state)) == "(self, e: int) -> dict"
    for e in (0, 1):
        st = c.state(e)
        assert list(st) == STATE_KEYS
        assert all(isinstance(v, int) for v in st.values())


def test_state_is_readable_before_any_step():
    c = Chip()
    assert list(c.state(0)) == STATE_KEYS


def test_the_module_imports_nothing_from_iss_src_or_asm():
    """The never-read rule: this ISS is independent by construction."""
    tree = ast.parse(MODULE.read_text())
    modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.add(node.module)
    forbidden = {m for m in modules if m.split(".")[0] in {"iss", "src", "asm", "verif"}}
    assert not forbidden, forbidden
    # It depends only on the standard library and its own package.
    assert {m.split(".")[0] for m in modules} <= {"verif"}


def test_there_is_no_chipdevice_class_here():
    """`Plus, in iss/pio_iss.py only: class ChipDevice`."""
    assert not hasattr(Chip, "read")
    assert not hasattr(Chip, "write")


@pytest.mark.parametrize("bad", [(1,), ("w", 0), ("x", 0, 0), ("r",), ()])
def test_a_malformed_bus_tuple_does_not_corrupt_the_chip(bad):
    c = Chip()
    before = dict(c.state(0))
    c.step(0, bad)  # ignored rather than raising
    assert c.state(0) == before
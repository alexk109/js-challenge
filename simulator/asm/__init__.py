"""The MVP PIO assembler.

``asm/pioasm.py`` holds the implementation; this module is the stable surface:

    >>> from asm import assemble
    >>> program = assemble(".program p\\n nop\\n")[0]
    >>> program.instrs
    [0xA042]

The names are looked up lazily, so ``python -m asm.pioasm file.pio`` (the CLI form
``docs/mvp-interfaces.md`` fixes) does not import ``asm.pioasm`` twice.
"""

from typing import TYPE_CHECKING

__all__ = ["AsmError", "Program", "assemble", "main"]

if TYPE_CHECKING:  # pragma: no cover - for type checkers and IDEs only
    from .pioasm import AsmError, Program, assemble, main


def __getattr__(name: str):
    if name in __all__:
        from . import pioasm

        return getattr(pioasm, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()) | set(__all__))

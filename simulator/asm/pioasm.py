"""PIO assembler for the MVP chip.

The API and CLI are fixed by ``docs/mvp-interfaces.md`` ``## Assembler API``.  Every encoding
comes from ``docs/mvp-isa.md`` ``## Encoding`` and ``### Field values`` -- nothing else, in
particular not from memory of pioasm.  Everything ``## Removed and reserved encodings``
removed is rejected: ``irq`` in all forms, ``wait ... irq`` and ``rel`` addressing.

Accepted syntax, the pioasm subset of the RP2040 datasheet sec. 3.3 that the MVP needs::

    .program NAME
    .side_set N [opt] [pindirs]
    .origin N
    .wrap [LABEL|EXPR]
    .wrap_target [LABEL|EXPR]
    .define [PUBLIC] NAME EXPR
    LABEL: [INSTRUCTION]
    PUBLIC LABEL:

    nop
    jmp [COND] TARGET               COND: !x, x--, !y, y--, x!=y, pin, !osre
    wait POL SOURCE INDEX             POL: 0|1    SOURCE: gpio|pin
    in SOURCE COUNT                   SOURCE: pins|x|y|null|isr|osr
    out DEST COUNT                    DEST: pins|x|y|null|pindirs|pc|isr|exec
    push [iffull] [block|noblock]
    pull [ifempty] [block|noblock]
    mov DEST, SRC [OP]                OP: ! (invert) or r (bit-reverse), never both
    set DEST DATA                     DEST: pins|x|y|pindirs

Comments start with ``;`` or ``//``.  Every instruction accepts a trailing ``side VALUE``
and ``[DELAY]``, in that order.  Numbers are decimal, ``0x`` hex or ``0b`` binary (``::``
groups allowed); expressions are numbers, labels, ``.define`` symbols, ``+ - * / % << >>
& | ^ ~`` and parentheses.  An expression that contains a space needs parentheses, since an
operand is otherwise a single token.  A ``.define`` without ``PUBLIC`` is private to its
program; a ``PUBLIC`` one is visible in every program of the file (and may appear before the
first ``.program``).

Choices the two spec files leave open:

* ``labels``, ``wrap`` and ``wrap_target`` hold absolute IMEM addresses, i.e. ``origin`` plus
  the instruction offset, so that a program with ``.origin`` jumps to the right place.
  Without ``.origin`` that is exactly the instruction index.
* ``.wrap``/``.wrap_target`` with no operand take the address of the next instruction.
* ``.side_set`` is one setting per program (the ``Program`` dataclass has a single value for
  it): if it appears more than once, the last one wins and applies to every instruction.
* Bit counts are 1..32; ``0`` is not accepted as a spelling of 32, though the encoding
  treats a count field of ``00000`` as 32.
* The condition of a ``jmp`` is accepted before or after the target, because the spec's own
  worked encoding writes ``jmp x-- 3 [6]``.
* ``push``/``pull`` **block by default** and ``noblock`` clears the Block bit, as pioasm does
  (RP2040 datasheet sec. 3.4.6/3.4.7, whose field layout ``## Encoding`` reproduces exactly).
  The two spec files do not state a default, but the table of worked encodings does: it prints
  0x9FA0 for ``pull side 1 [7]`` with ``.side_set 1 opt`` and names no flag word for that line,
  and bit 5 of 0x9FA0 is Block.  So the row is a blocking pull, which is what a bare ``pull``
  assembles to; ``noblock`` is the only spelling that clears it.  See ``asm/test_spec_examples.py``
  and finding F1 in ``verif/review/enc.md``.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field as dc_field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

__all__ = ["AsmError", "Program", "assemble", "main"]


class AsmError(Exception):
    """A source error: a bad line, operand or directive.

    ``line`` is the 1-based source line, or ``None`` for a whole-file error.
    """

    def __init__(self, message: str, line: Optional[int] = None) -> None:
        super().__init__(message)
        self.message = message
        self.line = line

    def __str__(self) -> str:
        if self.line is None:
            return self.message
        return f"line {self.line}: {self.message}"


@dataclass
class Program:
    """One ``.program`` of an assembled file.

    ``instrs[i]`` is the 16-bit word for IMEM address ``(origin or 0) + i``, so a host
    loads the words at ``origin`` and jumps to the label values it gets here.  ``labels``,
    ``wrap`` and ``wrap_target`` are absolute IMEM addresses (``origin`` plus offset).
    """

    name: str
    instrs: List[int] = dc_field(default_factory=list)
    origin: Optional[int] = None
    wrap_target: int = 0
    wrap: int = 0
    sideset_bits: int = 0
    sideset_opt: bool = False
    sideset_pindirs: bool = False
    labels: Dict[str, int] = dc_field(default_factory=dict)


# --------------------------------------------------------------------------------------------
# Field tables.  Every entry below is transcribed from docs/mvp-isa.md `### Field values`;
# nothing here comes from anywhere else.  Names that have no spelling (the reserved field
# values of `## Removed and reserved encodings`) are simply absent, so asking for one is an
# error.
# --------------------------------------------------------------------------------------------

JMP_ALWAYS = 0b000  # the condition encoded when no condition is written
JMP_CONDS: Dict[str, int] = {
    "!x": 0b001,
    "x--": 0b010,
    "!y": 0b011,
    "y--": 0b100,
    "x!=y": 0b101,
    "pin": 0b110,
    "!osre": 0b111,
}
WAIT_SOURCES: Dict[str, int] = {"gpio": 0b00, "pin": 0b01}  # 10 is IRQ (removed), 11 reserved
IN_SOURCES: Dict[str, int] = {
    "pins": 0b000, "x": 0b001, "y": 0b010, "null": 0b011, "isr": 0b110, "osr": 0b111,
}
OUT_DESTS: Dict[str, int] = {
    "pins": 0b000, "x": 0b001, "y": 0b010, "null": 0b011,
    "pindirs": 0b100, "pc": 0b101, "isr": 0b110, "exec": 0b111,
}
MOV_DESTS: Dict[str, int] = {
    "pins": 0b000, "x": 0b001, "y": 0b010,
    "exec": 0b100, "pc": 0b101, "isr": 0b110, "osr": 0b111,
}
MOV_SRCS: Dict[str, int] = {
    "pins": 0b000, "x": 0b001, "y": 0b010, "null": 0b011,
    "status": 0b101, "isr": 0b110, "osr": 0b111,
}
MOV_OPS: Dict[str, int] = {"": 0b00, "!": 0b01, "r": 0b10}  # 11 is reserved
SET_DESTS: Dict[str, int] = {"pins": 0b000, "x": 0b001, "y": 0b010, "pindirs": 0b100}

# Instruction-set extensions (tools/bench/sim/ISA_EXT.md).  Off by default: `assemble(text, isa=...)` enables
# them one by one ("xor", "cond", "irq"); without them these spellings are errors and output is unchanged.
ISA_NAMES = ("xor", "cond", "irq")
XOR_OP = 0b11                                    # MOV Op 11: dst = dst ^ src (mnemonic `xor DST, SRC`)
XOR_DESTS = ("x", "y", "isr", "osr")
JMPC_CONDS: Dict[str, int] = {                   # opcode 110 with isa_cond; Condition in bits 7:5
    "x_eq_y": 0b000, "x_lt_y": 0b001, "y_lt_x": 0b010, "pin_low": 0b011,
    "pin_rise": 0b100, "pin_fall": 0b101, "osre": 0b110, "isr_full": 0b111,
}

OP_JMP, OP_WAIT, OP_IN, OP_OUT = 0b000, 0b001, 0b010, 0b011
OP_SHIFT, OP_MOV, OP_SET = 0b100, 0b101, 0b111  # 0b110 is IRQ, removed in the MVP
OP_X = 0b110                                     # IRQ (isa_irq) or JMPC (isa_cond), never both


def _hint(names: Dict[str, int]) -> str:
    return ", ".join(sorted(repr(n) for n in names))


# --------------------------------------------------------------------------------------------
# Lexing: comments, tokens, and integer expressions
# --------------------------------------------------------------------------------------------


def _strip_comment(line: str) -> str:
    cut = len(line)
    for mark in ("//", ";"):
        i = line.find(mark)
        if i >= 0:
            cut = min(cut, i)
    return line[:cut]


def _tokenize(line: str) -> List[str]:
    """Split one line into tokens.

    Whitespace and ``,`` separate tokens, ``[`` and ``]`` stand alone (the delay syntax),
    and a parenthesised group stays in one token so that expressions can be evaluated.
    """
    toks: List[str] = []
    cur = ""
    depth = 0
    for ch in line:
        if ch == "(":
            depth += 1
            cur += ch
        elif ch == ")":
            depth -= 1
            cur += ch
        elif depth == 0 and (ch.isspace() or ch == ","):
            if cur:
                toks.append(cur)
                cur = ""
        elif depth == 0 and ch in "[]":
            if cur:
                toks.append(cur)
                cur = ""
            toks.append(ch)
        else:
            cur += ch
    if cur:
        toks.append(cur)
    return toks


_NUM_RE = re.compile(r"0[xX][0-9a-fA-F]+|0[bB][01]+(?:::[01]+)*|[0-9]+")
_IDENT_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_$]*")
_EXPR_TOK_RE = re.compile(r"<<|>>|[()+\-*/%&|^~]|[^\s()+\-*/%&|^~<>]+")


def _lex_expr(text: str, line: int) -> List[str]:
    toks = _EXPR_TOK_RE.findall(text)
    if "".join(toks) != "".join(text.split()):
        raise AsmError(f"cannot parse expression {text!r}", line)
    return toks


def _number(text: str, line: int) -> int:
    if text[:2].lower() == "0x":
        return int(text[2:], 16)
    if text[:2].lower() == "0b":
        return int(text[2:].replace("::", ""), 2)
    if text.isdigit():
        return int(text, 10)
    raise AsmError(f"{text!r} is not a number (decimal, 0x hex or 0b binary)", line)


class _ExprParser:
    """Recursive-descent evaluator for the pioasm integer expression language.

    Precedence, loosest first: ``|``, ``^``, ``&``, ``<<`` ``>>``, ``+`` ``-``, ``*``
    ``/`` ``%``, then unary ``- + ~``, then numbers, symbols and parentheses.
    """

    _BINOPS = _BINOPS_ALL = ("|", "^", "&", "<<", ">>", "+", "-", "*", "/", "%")

    def __init__(self, text: str, scope: "_Scope", line: int) -> None:
        self.text = text
        self.scope = scope
        self.line = line
        self.toks = _lex_expr(text, line)
        self.pos = 0

    # --- token helpers
    def _peek(self) -> Optional[str]:
        return self.toks[self.pos] if self.pos < len(self.toks) else None

    def _take(self) -> str:
        if self.pos >= len(self.toks):
            raise AsmError(f"unexpected end of expression {self.text!r}", self.line)
        tok = self.toks[self.pos]
        self.pos += 1
        return tok

    def _at(self, op: str) -> bool:
        return self._peek() == op

    # --- grammar
    def parse(self) -> int:
        if not self.toks:
            raise AsmError("empty expression", self.line)
        value = self._binary(0)
        if self.pos != len(self.toks):
            raise AsmError(f"unexpected {self._peek()!r} in expression {self.text!r}", self.line)
        return value

    def _binary(self, level: int) -> int:
        """Evaluate this precedence level and everything tighter, left to right."""
        if level >= len(_LEVELS):
            return self._unary()
        value = self._binary(level + 1)
        while self._peek() in _LEVELS[level]:
            op = self._take()
            value = _APPLY[op](self, value, self._binary(level + 1))
        return value

    def _unary(self) -> int:
        tok = self._peek()
        if tok == "-":
            self._take()
            return -self._unary()
        if tok == "+":
            self._take()
            return self._unary()
        if tok == "~":
            self._take()
            return ~self._unary()
        return self._primary()

    def _primary(self) -> int:
        tok = self._take()
        if tok == "(":
            value = self._binary(0)
            if not self._at(")"):
                raise AsmError("missing ')' in expression " + repr(self.text), self.line)
            self._take()
            return value
        if _NUM_RE.fullmatch(tok):
            return _number(tok, self.line)
        if _IDENT_RE.fullmatch(tok):
            return self.scope.symbol(tok, self.line)
        raise AsmError(f"unexpected {tok!r} in expression {self.text!r}", self.line)


# Precedence, loosest first; each level binds tighter than the one above it.
_LEVELS: Tuple[Tuple[str, ...], ...] = (
    ("|",),
    ("^",),
    ("&",),
    ("<<", ">>"),
    ("+", "-"),
    ("*", "/", "%"),
)


def _checked_shift(b: int, line: int) -> int:
    if not 0 <= b <= 63:
        raise AsmError(f"shift count {b} out of range (0..63)", line)
    return b


def _f_shl(p: "_ExprParser", a: int, b: int) -> int:
    return a << _checked_shift(b, p.line)


def _f_shr(p: "_ExprParser", a: int, b: int) -> int:
    return a >> _checked_shift(b, p.line)


def _f_div(p: "_ExprParser", a: int, b: int) -> int:
    if b == 0:
        raise AsmError("division by zero", p.line)
    return a // b


def _f_mod(p: "_ExprParser", a: int, b: int) -> int:
    if b == 0:
        raise AsmError("division by zero", p.line)
    return a % b


_APPLY: Dict[str, Callable[["_ExprParser", int, int], int]] = {
    "|": lambda p, a, b: a | b,
    "^": lambda p, a, b: a ^ b,
    "&": lambda p, a, b: a & b,
    "<<": _f_shl,
    ">>": _f_shr,
    "+": lambda p, a, b: a + b,
    "-": lambda p, a, b: a - b,
    "*": lambda p, a, b: a * b,
    "/": _f_div,
    "%": _f_mod,
}


# --------------------------------------------------------------------------------------------
# Symbols: labels and .define
# --------------------------------------------------------------------------------------------


class _Scope:
    """Resolves the names an expression may use, for one program.

    Labels win over ``.define``s, program-local defines over ``PUBLIC`` ones.  Defines are
    evaluated on first use and memoised, so a define may refer to one defined later; a cycle
    is an error.
    """

    def __init__(
        self,
        labels: Dict[str, int],
        local_defines: Dict[str, Tuple[str, int]],
        global_defines: Dict[str, Tuple[str, int]],
    ) -> None:
        self.labels = labels
        self.local_defines = local_defines
        self.global_defines = global_defines
        self._cache: Dict[str, int] = {}
        self._active: List[str] = []

    def eval(self, text: str, line: int) -> int:
        return _ExprParser(text, self, line).parse()

    def symbol(self, name: str, line: int) -> int:
        if name in self.labels:
            return self.labels[name]
        if name in self._cache:
            return self._cache[name]
        if name in self.local_defines:
            text, def_line = self.local_defines[name]
        elif name in self.global_defines:
            text, def_line = self.global_defines[name]
        else:
            raise AsmError(
                f"undefined symbol {name!r} (not a label and not defined with .define)", line
            )
        if name in self._active:
            chain = " -> ".join(self._active + [name])
            raise AsmError(f".define cycle: {chain}", line)
        self._active.append(name)
        try:
            value = self.eval(text, def_line)
        finally:
            self._active.pop()
        self._cache[name] = value
        return value


# --------------------------------------------------------------------------------------------
# Instruction encoding
# --------------------------------------------------------------------------------------------

RELAXED = "the MVP has no rel addressing, only absolute addresses"
_MODIFIER_RE = re.compile(r"[!r]+")  # e.g. '!r': the two mov ops in one token


class _Encoder:
    """Turns one program's instruction lines into 16-bit words.

    The side-set configuration is a property of the whole program (``Program`` has one
    ``sideset_bits``), so it is known before the first instruction is encoded and splits
    bits 12:8 as ``docs/mvp-isa.md`` ``### Delay/side-set field, bits 12:8`` says: the top
    ``SC`` bits are side-set, the low ``5 - SC`` bits the delay, and with ``opt`` the topmost
    of the side-set bits is the enable.
    """

    def __init__(self, scope: _Scope, sideset: Tuple[int, bool], isa: frozenset = frozenset()) -> None:
        self.scope = scope
        self.sc, self.opt = sideset
        self.isa = isa

    # --- small helpers
    def eval(self, text: str, line: int) -> int:
        return self.scope.eval(text, line)

    def bits(self, value: int, width: int, what: str, line: int) -> int:
        """Check that ``value`` fits in ``width`` unsigned bits."""
        limit = 1 << width
        if not 0 <= value < limit:
            raise AsmError(f"{what} {value} does not fit in {width} bits (0..{limit - 1})", line)
        return value

    def address(self, value: int, what: str, line: int) -> int:
        return self.bits(value, 5, what, line)

    def bit_count(self, value: int, what: str, line: int) -> int:
        """A bit count of 1..32, encoded in 5 bits with 00000 meaning 32."""
        if not 1 <= value <= 32:
            raise AsmError(f"{what} {value} out of range: a bit count is 1..32", line)
        return 0 if value == 32 else value

    def operand(self, tok: str, table: Dict[str, int], what: str, line: int) -> str:
        if tok not in table:
            raise AsmError(f"unknown {what} {tok!r} (expected one of {_hint(table)})", line)
        return tok

    def field(self, side: int, has_side: bool, delay: int, line: int) -> int:
        """Bits 12:8: the side-set bits on top, the delay below."""
        max_delay = (1 << (5 - self.sc)) - 1
        if not 0 <= delay <= max_delay:
            raise AsmError(
                f"delay {delay} does not fit: with {self.sc} side-set bit(s) the delay is "
                f"0..{max_delay}",
                line,
            )
        return (self._side(side, has_side, line) << (5 - self.sc)) | delay

    def _side(self, side: int, has_side: bool, line: int) -> int:
        if not has_side:
            return 0
        if self.sc == 0:
            raise AsmError("'side' operand but the program has no .side_set directive", line)
        limit = 1 << self.sc
        if not 0 <= side < limit:
            raise AsmError(
                f"side-set value {side} does not fit in {self.sc} side-set field bit(s) "
                f"(0..{limit - 1})",
                line,
            )
        # With opt, the topmost side-set bit is the enable: it is set for any non-zero value,
        # which is what makes `side 0` mean "no side-set" (docs/mvp-isa.md).
        return side | ((1 << (self.sc - 1)) if (self.opt and side) else 0)

    def word(self, opcode: int, low: int, side: int, has_side: bool, delay: int, line: int) -> int:
        return (opcode << 13) | (self.field(side, has_side, delay, line) << 8) | low

    def tail(self, toks: Sequence[str], i: int, line: int) -> Tuple[int, bool, int]:
        """Parse the optional ``side VALUE`` and ``[DELAY]`` tail of any instruction."""
        side, has_side, delay = 0, False, 0
        while i < len(toks):
            tok = toks[i]
            if tok == "side":
                if has_side:
                    raise AsmError("only one 'side' operand per instruction", line)
                if i + 1 >= len(toks):
                    raise AsmError("'side' needs a value, as in 'side 1'", line)
                side, has_side = self.eval(toks[i + 1], line), True
                i += 2
            elif tok == "[":
                if i + 2 >= len(toks) or toks[i + 2] != "]":
                    raise AsmError(f"a delay is written [n], not {tok!r} alone", line)
                delay = self.eval(toks[i + 1], line)
                i += 3
            elif tok == "]":
                raise AsmError(
                    "']' closes a delay: write the delay as [n] before it, not after",
                    line,
                )
            else:
                if _MODIFIER_RE.fullmatch(tok):
                    raise AsmError(
                        f"unexpected {tok!r}: a mov inverts with '!' or bit-reverses with "
                        f"'r', not both (Op 11 is reserved)",
                        line,
                    )
                raise AsmError(
                    f"unexpected {tok!r}: only 'side VALUE' and '[DELAY]' may follow an "
                    f"instruction's operands, and an expression with a space in it needs "
                    f"parentheses",
                    line,
                )
        return side, has_side, delay

    # --- the instructions
    def nop(self, toks: Sequence[str], line: int) -> int:
        side, has_side, delay = self.tail(toks, 1, line)
        return self.word(OP_MOV, (0b010 << 5) | (0b00 << 3) | 0b010, side, has_side, delay, line)

    def jmp(self, toks: Sequence[str], line: int) -> int:
        rest = list(toks[1:])
        cond, target_tok, i = JMP_ALWAYS, None, 0
        # pioasm writes `jmp target cond` and the spec's worked encoding writes
        # `jmp x-- 3 [6]`, so the condition is accepted on either side of the target.
        while i < len(rest):
            tok = rest[i]
            if tok in JMP_CONDS:
                if cond != JMP_ALWAYS:
                    raise AsmError("only one condition per jmp", line)
                cond = JMP_CONDS[tok]
                i += 1
            elif target_tok is None and tok not in ("side", "["):
                target_tok = tok
                i += 1
            else:
                break
        if target_tok is None:
            raise AsmError("jmp needs a target: 'jmp LABEL' or 'jmp 3'", line)
        target = self.address(self.eval(target_tok, line), "jmp target", line)
        side, has_side, delay = self.tail(rest, i, line)
        return self.word(OP_JMP, (cond << 5) | target, side, has_side, delay, line)

    def wait(self, toks: Sequence[str], line: int) -> int:
        if len(toks) < 4:
            raise AsmError("wait needs POLARITY, SOURCE and INDEX, as in 'wait 0 pin 0'", line)
        pol_tok, src, index_tok = toks[1], toks[2], toks[3]
        if src.lower() == "irq":
            if "irq" in self.isa:
                pol = self.eval(pol_tok, line)
                if pol not in (0, 1):
                    raise AsmError(f"wait polarity must be 0 or 1, not {pol}", line)
                idx, i = self._irq_index(toks, 3, line)
                side, has_side, delay = self.tail(toks, i, line)
                return self.word(OP_WAIT, (pol << 7) | (0b10 << 5) | idx, side, has_side, delay, line)
            raise AsmError(
                "'wait ... irq' is rejected: the MVP has no IRQ flags (they read 0), so it "
                "would wait forever or pass at once",
                line,
            )
        src = self.operand(src, WAIT_SOURCES, "wait source", line)
        pol = self.eval(pol_tok, line)
        if pol not in (0, 1):
            raise AsmError(f"wait polarity must be 0 or 1, not {pol}", line)
        index = self.address(self.eval(index_tok, line), "wait index", line)
        side, has_side, delay = self.tail(toks, 4, line)
        return self.word(
            OP_WAIT, (pol << 7) | (WAIT_SOURCES[src] << 5) | index, side, has_side, delay, line
        )

    def in_(self, toks: Sequence[str], line: int) -> int:
        if len(toks) < 3:
            raise AsmError("in needs a SOURCE and a bit COUNT, as in 'in pins, 1'", line)
        src = self.operand(toks[1], IN_SOURCES, "in source", line)
        count = self.bit_count(self.eval(toks[2], line), "in bit count", line)
        side, has_side, delay = self.tail(toks, 3, line)
        return self.word(OP_IN, (IN_SOURCES[src] << 5) | count, side, has_side, delay, line)

    def out(self, toks: Sequence[str], line: int) -> int:
        if len(toks) < 3:
            raise AsmError("out needs a DESTINATION and a bit COUNT, as in 'out pins, 1'", line)
        dest = self.operand(toks[1], OUT_DESTS, "out destination", line)
        count = self.bit_count(self.eval(toks[2], line), "out bit count", line)
        side, has_side, delay = self.tail(toks, 3, line)
        return self.word(OP_OUT, (OUT_DESTS[dest] << 5) | count, side, has_side, delay, line)

    def push(self, toks: Sequence[str], line: int) -> int:
        low, i = self._shift(toks, line, "iffull", 0)
        side, has_side, delay = self.tail(toks, i, line)
        return self.word(OP_SHIFT, low, side, has_side, delay, line)

    def pull(self, toks: Sequence[str], line: int) -> int:
        low, i = self._shift(toks, line, "ifempty", 1)
        side, has_side, delay = self.tail(toks, i, line)
        return self.word(OP_SHIFT, low, side, has_side, delay, line)

    def mov(self, toks: Sequence[str], line: int) -> int:
        if len(toks) < 3:
            raise AsmError("mov needs a DESTINATION and a SOURCE, as in 'mov x, y'", line)
        dest = self.operand(toks[1], MOV_DESTS, "mov destination", line)
        src = self.operand(toks[2], MOV_SRCS, "mov source", line)
        op = ""
        i = 3
        while i < len(toks) and toks[i] in ("!", "r"):
            if op:
                raise AsmError(
                    "a mov can invert ('!') or bit-reverse ('r'), not both: Op 11 is reserved",
                    line,
                )
            op = toks[i]
            i += 1
        side, has_side, delay = self.tail(toks, i, line)
        low = (MOV_DESTS[dest] << 5) | (MOV_OPS[op] << 3) | MOV_SRCS[src]
        return self.word(OP_MOV, low, side, has_side, delay, line)

    def set(self, toks: Sequence[str], line: int) -> int:
        if len(toks) < 3:
            raise AsmError("set needs a DESTINATION and DATA, as in 'set pins, 1'", line)
        dest = self.operand(toks[1], SET_DESTS, "set destination", line)
        data = self.bits(self.eval(toks[2], line), 5, "set data", line)
        side, has_side, delay = self.tail(toks, 3, line)
        return self.word(OP_SET, (SET_DESTS[dest] << 5) | data, side, has_side, delay, line)

    def _irq_index(self, toks: Sequence[str], i: int, line: int) -> Tuple[int, int]:
        """`N` or `N rel` -> the 5-bit index field (bit 4 = rel), and where the tail starts."""
        if i >= len(toks):
            raise AsmError("irq needs a flag number 0..7, as in 'irq 3' or 'irq 1 rel'", line)
        n = self.bits(self.eval(toks[i], line), 3, "irq flag number", line)
        rel = i + 1 < len(toks) and toks[i + 1] == "rel"
        return (0b10000 if rel else 0) | n, i + 1 + (1 if rel else 0)

    def irq(self, toks: Sequence[str], line: int) -> int:
        if "irq" not in self.isa:
            raise AsmError(
                "the MVP removed the IRQ instruction: its flags are hardwired to 0 and it is "
                "assembled as a NOP at run time, so the assembler refuses to write it",
                line,
            )
        # irq [set | nowait | wait | clear] N [rel]   (pioasm spelling; bit 6 = Clr, bit 5 = Wait)
        i, clr, wt = 1, 0, 0
        if i < len(toks) and toks[i] in ("set", "nowait", "wait", "clear"):
            clr, wt = int(toks[i] == "clear"), int(toks[i] == "wait")
            i += 1
        idx, i = self._irq_index(toks, i, line)
        side, has_side, delay = self.tail(toks, i, line)
        return self.word(OP_X, (clr << 6) | (wt << 5) | idx, side, has_side, delay, line)

    def xor(self, toks: Sequence[str], line: int) -> int:
        if "xor" not in self.isa:
            raise AsmError("xor is an extension instruction (isa_xor); the MVP has no XOR", line)
        if len(toks) < 3:
            raise AsmError("xor needs a DESTINATION and a SOURCE, as in 'xor x, isr'", line)
        dest = self.operand(toks[1], {d: MOV_DESTS[d] for d in XOR_DESTS}, "xor destination", line)
        src = self.operand(toks[2], MOV_SRCS, "xor source", line)
        side, has_side, delay = self.tail(toks, 3, line)
        low = (MOV_DESTS[dest] << 5) | (XOR_OP << 3) | MOV_SRCS[src]
        return self.word(OP_MOV, low, side, has_side, delay, line)

    def jmpc(self, toks: Sequence[str], line: int) -> int:
        if "cond" not in self.isa:
            raise AsmError("jmpc is an extension instruction (isa_cond); the MVP has only jmp", line)
        rest = list(toks[1:])
        if not rest or rest[0] not in JMPC_CONDS:
            raise AsmError(f"jmpc needs a condition (one of {_hint(JMPC_CONDS)}) and a target", line)
        cond = JMPC_CONDS[rest[0]]
        if len(rest) < 2 or rest[1] in ("side", "["):
            raise AsmError("jmpc needs a target: 'jmpc pin_rise LABEL'", line)
        target = self.address(self.eval(rest[1], line), "jmpc target", line)
        side, has_side, delay = self.tail(rest, 2, line)
        return self.word(OP_X, (cond << 5) | target, side, has_side, delay, line)

    # --- push/pull flag words: bit 7 selects PULL over PUSH, bit 6 IfFull/IfEmpty, bit 5 Block
    # pioasm semantics (RP2040 datasheet sec. 3.4.6/3.4.7): Block is the DEFAULT; `noblock` clears it.
    _FLAGS = {"iffull": "IfFull", "ifempty": "IfEmpty", "block": "Block", "noblock": "Block"}

    def _shift(self, toks: Sequence[str], line: int, if_flag: str, pull: int) -> Tuple[int, int]:
        """Read the optional flags, return bits 7:5 and where the tail starts."""
        i, seen = 1, set()
        while i < len(toks) and toks[i] in self._FLAGS:
            if toks[i] in seen:
                raise AsmError(f"duplicate {self._FLAGS[toks[i]]} flag", line)
            if {toks[i], *seen} >= {"block", "noblock"}:
                raise AsmError("conflicting Block flags: block and noblock", line)
            seen.add(toks[i])
            i += 1
        low = (
            (pull << 7)
            | (int(if_flag in seen) << 6)
            | (int("noblock" not in seen) << 5)
        )
        return low, i


# --------------------------------------------------------------------------------------------
# Programs and directives
# --------------------------------------------------------------------------------------------

INSTRUCTIONS = {
    "nop": "nop",
    "jmp": "jmp",
    "wait": "wait",
    "in": "in_",
    "out": "out",
    "push": "push",
    "pull": "pull",
    "mov": "mov",
    "set": "set",
    "irq": "irq",
    "xor": "xor",
    "jmpc": "jmpc",
}


@dataclass
class _Prog:
    """One ``.program`` while it is being collected: raw lines, labels, defines, settings."""

    name: str
    line: int
    instrs: List[Tuple[int, List[str]]] = dc_field(default_factory=list)
    labels: List[Tuple[str, int, int]] = dc_field(default_factory=list)
    defines: Dict[str, Tuple[str, int]] = dc_field(default_factory=dict)
    sideset: Optional[Tuple[int, bool, bool]] = None  # (bits incl. opt, opt, pindirs)
    origin: Optional[Tuple[str, int]] = None
    wrap: Optional[Tuple[str, int, int]] = None  # (expr or "", line, instruction index)
    wrap_target: Optional[Tuple[str, int, int]] = None


def _need_program(progs: List[_Prog], line: int) -> _Prog:
    if not progs:
        raise AsmError(
            "this line is outside any .program: a file needs a '.program NAME' directive first",
            line,
        )
    return progs[-1]


def _directive(
    prog: Optional[_Prog], toks: Sequence[str], line: int, gdefs: Dict[str, Tuple[str, int]]
) -> None:
    """Apply one directive.  ``prog`` is None only for a ``.define`` before any program."""
    name, args = toks[0], list(toks[1:])
    if name == ".define":
        _define(prog, args, line, gdefs)
    elif prog is None:
        _need_program([], line)
    elif name == ".side_set":
        if not args:
            raise AsmError(".side_set needs a bit count, as in '.side_set 1 opt'", line)
        prog.sideset = _side_set(args, line)
    elif name == ".origin":
        if not args:
            raise AsmError(".origin needs an address, as in '.origin 4'", line)
        prog.origin = (" ".join(args), line)
    elif name in (".wrap", ".wrap_target"):
        if len(args) > 1 and not _is_one_expression(args):
            raise AsmError(f"{name} takes at most one label or address", line)
        # With no operand the address is the next instruction, so keep its index as well.
        spec = (" ".join(args), line, len(prog.instrs))
        if name == ".wrap":
            prog.wrap = spec
        else:
            prog.wrap_target = spec
    else:
        raise AsmError(f"unknown directive {name!r}", line)


def _is_one_expression(args: Sequence[str]) -> bool:
    """True when the tokens are a single parenthesised group, like ``.wrap (base + 1)``."""
    return len(args) == 1 and args[0].startswith("(")


def _side_set(args: Sequence[str], line: int) -> Tuple[int, bool, bool]:
    """Parse ``.side_set N [opt] [pindirs]`` into (bits including the opt bit, opt, pindirs)."""
    count = args[0]
    if not _NUM_RE.fullmatch(count):
        raise AsmError(f".side_set needs a literal bit count, not {count!r}", line)
    bits = _number(count, line)
    opt = pindirs = False
    for flag in args[1:]:
        if flag == "opt":
            if opt:
                raise AsmError("duplicate 'opt' in .side_set", line)
            opt = True
        elif flag == "pindirs":
            if pindirs:
                raise AsmError("duplicate 'pindirs' in .side_set", line)
            pindirs = True
        else:
            raise AsmError(
                f"unknown .side_set flag {flag!r} (expected 'opt' and/or 'pindirs')", line
            )
    if opt:
        bits += 1  # the opt bit is part of the 5-bit field, so it counts towards SC
    if not 0 <= bits <= 5:
        raise AsmError(
            f".side_set {args[0]}{' opt' if opt else ''} needs {bits} bits of the 5-bit "
            f"delay/side-set field, which cannot hold more than 5",
            line,
        )
    return bits, opt, pindirs


def _define(
    prog: Optional[_Prog], args: Sequence[str], line: int, gdefs: Dict[str, Tuple[str, int]]
) -> None:
    if args and args[0] == "PUBLIC":
        args = list(args[1:])
        target = gdefs
        where = "PUBLIC"
    else:
        if prog is None:
            raise AsmError(".define before the first .program must be PUBLIC", line)
        target = prog.defines
        where = "program-local"
    if len(args) < 2:
        raise AsmError(".define needs a NAME and a value, as in '.define N 3'", line)
    name, value = args[0], " ".join(args[1:])
    if not _IDENT_RE.fullmatch(name):
        raise AsmError(f"{name!r} is not a valid symbol name for .define", line)
    if name in target:
        raise AsmError(f"{where} symbol {name!r} is defined twice", line)
    target[name] = (value, line)


def _labels_and_instruction(prog: _Prog, toks: Sequence[str], line: int) -> None:
    """Consume any run of ``[PUBLIC] name:`` labels, then the instruction (if any)."""
    toks = list(toks)
    while toks:
        if toks[0] == "PUBLIC":
            if len(toks) < 2:
                raise AsmError("PUBLIC must be followed by a label, as in 'PUBLIC start:'", line)
            toks = toks[1:]
        if not toks[0].endswith(":"):
            break
        name = toks[0][:-1]
        if not _IDENT_RE.fullmatch(name):
            raise AsmError(f"{name!r} is not a valid label name", line)
        if any(n == name for n, _, _ in prog.labels):
            first = next(l for n, l, _ in prog.labels if n == name)
            raise AsmError(f"label {name!r} is defined twice (first on line {first})", line)
        prog.labels.append((name, line, len(prog.instrs)))
        toks = toks[1:]
    if toks:
        prog.instrs.append((line, toks))


# --------------------------------------------------------------------------------------------
# The public entry point
# --------------------------------------------------------------------------------------------


def assemble(text: str, isa: Sequence[str] = ()) -> List[Program]:
    """Assemble a ``.pio`` source file and return one ``Program`` per ``.program``.

    Raises ``AsmError`` (with ``.line`` set) for anything the MVP does not accept.  ``isa`` names the
    instruction-set extensions to accept ("xor", "cond", "irq"; tools/bench/sim/ISA_EXT.md); the default
    is the MVP and its output is unchanged.  "cond" and "irq" both need opcode 110 and cannot be combined.
    """
    ext = frozenset(isa)
    if not ext <= set(ISA_NAMES):
        raise AsmError(f"unknown isa extension(s) {sorted(ext - set(ISA_NAMES))}; known: {', '.join(ISA_NAMES)}")
    if {"cond", "irq"} <= ext:
        raise AsmError("isa extensions 'cond' and 'irq' both need opcode 110 and cannot be combined at 16 bits")
    progs: List[_Prog] = []
    gdefs: Dict[str, Tuple[str, int]] = {}

    for line, raw in enumerate(text.splitlines(), start=1):
        stripped = _strip_comment(raw).strip()
        if not stripped:
            continue
        toks = _tokenize(stripped)
        if not toks:
            continue
        if toks[0] == ".program":
            if len(toks) != 2:
                raise AsmError(
                    f".program takes one name, as in '.program {toks[1] if len(toks) > 1 else 'name'}'",
                    line,
                )
            name = toks[1]
            if not _IDENT_RE.fullmatch(name):
                raise AsmError(f"{name!r} is not a valid program name", line)
            if any(p.name == name for p in progs):
                raise AsmError(f"program {name!r} is defined twice", line)
            progs.append(_Prog(name=name, line=line))
        elif toks[0].startswith("."):
            # A .define may come before the first .program (if it is PUBLIC); every other
            # directive needs a program to belong to.
            prog = progs[-1] if progs else None
            if prog is None and toks[0] != ".define":
                prog = _need_program(progs, line)
            _directive(prog, toks, line, gdefs)
        else:
            _labels_and_instruction(_need_program(progs, line), toks, line)

    return [_finish(prog, gdefs, ext) for prog in progs]


def _finish(prog: _Prog, gdefs: Dict[str, Tuple[str, int]], isa: frozenset = frozenset()) -> Program:
    # .origin first: labels, wrap and wrap_target are all offset by it.
    origin = None
    if prog.origin is not None:
        origin = _Scope({}, prog.defines, gdefs).eval(prog.origin[0], prog.origin[1])
        if not 0 <= origin <= 31:
            raise AsmError(
                f".origin {origin} is outside the 32-word instruction memory (0..31)",
                prog.origin[1],
            )

    labels: Dict[str, int] = {}
    for name, line, index in prog.labels:
        address = (origin or 0) + index
        if address > 31:
            raise AsmError(
                f"label {name!r} is at address {address}, which does not fit the 5-bit "
                f"address field of a 32-word instruction memory",
                line,
            )
        labels[name] = address

    scope = _Scope(labels, prog.defines, gdefs)
    # Defines are resolved lazily so that one may use a later one; resolve each of them once
    # now so an unused but broken .define is still an error.
    for name, (_, line) in prog.defines.items():
        scope.symbol(name, line)
    sideset = prog.sideset if prog.sideset is not None else (0, False, False)

    words: List[int] = []
    encoder = _Encoder(scope, (sideset[0], sideset[1]), isa)
    for line, toks in prog.instrs:
        op = toks[0]
        handler = getattr(encoder, INSTRUCTIONS.get(op, ""), None)
        if handler is None:
            raise AsmError(
                f"unknown instruction {op!r} (expected one of "
                f"{', '.join(sorted(INSTRUCTIONS))})",
                line,
            )
        if "rel" in toks and not (op == "irq" or (op == "wait" and "irq" in isa)):
            raise AsmError(f"'rel' is not a jump target here: {RELAXED}", line)
        words.append(handler(toks, line))

    return Program(
        name=prog.name,
        instrs=words,
        origin=origin,
        wrap_target=_wrap_point(prog.wrap_target, origin, scope),
        wrap=_wrap_point(prog.wrap, origin, scope, default=len(words) - 1),
        sideset_bits=sideset[0],
        sideset_opt=sideset[1],
        sideset_pindirs=sideset[2],
        labels=labels,
    )


def _wrap_point(
    spec: Optional[Tuple[str, int, int]], origin: Optional[int], scope: _Scope, default: int = 0
) -> int:
    """The address of a ``.wrap``/``.wrap_target``: its operand, or the next instruction."""
    if spec is None:
        return default if default > 0 else 0
    text, line, index = spec
    if not text:
        return (origin or 0) + index
    value = scope.eval(text, line)
    if not 0 <= value <= 31:
        raise AsmError(f"wrap address {value} is outside the instruction memory (0..31)", line)
    return value


# --------------------------------------------------------------------------------------------
# CLI: python -m asm.pioasm file.pio  /  python -m asm file.pio
# --------------------------------------------------------------------------------------------


def _program_lines(prog: Program, meta: bool) -> List[str]:
    """The default CLI output of one program: a ``.program`` header and its words, in hex."""
    out = [f".program {prog.name}"]
    if meta:
        if prog.origin is not None:
            out.append(f".origin {prog.origin}")
        out.append(f".side_set {prog.sideset_bits - int(prog.sideset_opt)}"
                   f"{' opt' if prog.sideset_opt else ''}"
                   f"{' pindirs' if prog.sideset_pindirs else ''}")
        out.append(f".wrap_target {prog.wrap_target}")
        out.append(f".wrap {prog.wrap}")
    out.extend(f"0x{word:04X}" for word in prog.instrs)
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m asm", description="Assemble MVP PIO programs (docs/mvp-isa.md)."
    )
    parser.add_argument("files", nargs="+", help="the .pio files to assemble")
    parser.add_argument(
        "-m", "--meta", action="store_true",
        help="also print .origin, .side_set, .wrap_target and .wrap for each program",
    )
    args = parser.parse_args(argv)

    for path in args.files:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                text = handle.read()
        except OSError as exc:
            print(f"{path}: {exc.strerror}", file=sys.stderr)
            return 2
        try:
            programs = assemble(text)
        except AsmError as exc:
            where = f"{path}:{exc.line}" if exc.line else path
            print(f"{where}: {exc.message}", file=sys.stderr)
            return 1
        for prog in programs:
            print("\n".join(_program_lines(prog, args.meta)))
    return 0


if __name__ == "__main__":
    sys.exit(main())


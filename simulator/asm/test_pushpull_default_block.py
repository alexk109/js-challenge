"""pioasm semantics: a bare push/pull blocks; `noblock` clears Block (found by B2-REVIEW-enc, F1)."""
from asm.pioasm import assemble


def word(line, sideset=""):
    return assemble(f".program t\n{sideset}\n{line}\n")[0].instrs[0]


def test_bare_pull_blocks():
    assert word("pull") == word("pull block") == 0x80A0


def test_noblock_clears_block():
    assert word("pull noblock") == 0x8080 and word("push noblock") == 0x8000


def test_bare_push_blocks():
    assert word("push") == word("push block") == 0x8020


def test_spec_table_row_pull_side_1_delay_7():
    assert word("pull side 1 [7]", ".side_set 1 opt") == 0x9FA0

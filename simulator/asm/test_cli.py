"""The CLI: ``python -m asm.pioasm file.pio``, the form ``## Assembler API`` fixes."""

import subprocess
import sys
from pathlib import Path

import pytest

from asm import main
from asm.pioasm import _program_lines, assemble

REPO = Path(__file__).resolve().parent.parent

SOURCE = """
.program demo        ; a comment
.side_set 1 opt
    nop
    pull side 1 [7]  ; a bare `pull` blocks (pioasm's default), so this is 0x9FA0
.wrap_target 0
"""


@pytest.fixture()
def demo(tmp_path: Path) -> Path:
    path = tmp_path / "demo.pio"
    path.write_text(SOURCE)
    return path


def run(*args):
    """Run the CLI as the docs spell it and return (returncode, stdout, stderr)."""
    proc = subprocess.run(
        [sys.executable, "-m", "asm.pioasm", *args],
        cwd=REPO, capture_output=True, text=True, check=False,
    )
    return proc.returncode, proc.stdout, proc.stderr


def test_cli_prints_one_word_per_line_in_hex(demo):
    code, out, err = run(str(demo))
    assert (code, err) == (0, "")
    assert out.splitlines() == [".program demo", "0xA042", "0x9FA0"]


def test_cli_as_a_module_matches_the_documented_form(demo):
    code, out, err = run(str(demo))
    assert code == 0 and err == ""
    module = subprocess.run(
        [sys.executable, "-m", "asm", str(demo)], cwd=REPO, capture_output=True, text=True
    )
    assert module.returncode == 0
    assert module.stdout == out  # `python -m asm` is the same CLI


def test_cli_meta_flag(demo):
    code, out, _ = run("-m", str(demo))
    assert code == 0
    assert out.splitlines() == [
        ".program demo", ".side_set 1 opt", ".wrap_target 0", ".wrap 1", "0xA042", "0x9FA0",
    ]


def test_cli_two_programs(tmp_path):
    path = tmp_path / "two.pio"
    path.write_text(".program a\nnop\n.program b\nnop\nnop\n")
    code, out, _ = run(str(path))
    assert code == 0
    assert out.splitlines() == [".program a", "0xA042", ".program b", "0xA042", "0xA042"]


def test_cli_reports_the_file_and_the_line(tmp_path):
    path = tmp_path / "bad.pio"
    path.write_text(".program p\n  nop\n  irq wait 0\n")
    code, out, err = run(str(path))
    assert code == 1
    assert out == ""
    assert f"{path}:3: " in err and "IRQ" in err


def test_cli_missing_file(tmp_path):
    code, _, err = run(str(tmp_path / "nope.pio"))
    assert code == 2
    assert "nope.pio" in err


def test_main_is_importable(demo, capsys):
    assert main([str(demo)]) == 0
    assert "0xA042" in capsys.readouterr().out


def test_program_lines_matches_the_assembler():
    program = assemble(SOURCE)[0]
    assert _program_lines(program, False) == [".program demo", "0xA042", "0x9FA0"]
    assert _program_lines(program, True) == [
        ".program demo", ".side_set 1 opt", ".wrap_target 0", ".wrap 1", "0xA042", "0x9FA0",
    ]

"""research/ideation/round2/experiments/*/RESULT.md: the `verdict:` line of each experiment.

Workload = experiment.<dirname>, metric = verdict, value = the word after the first `verdict:` line.
A RESULT.md with no such line yields one n/a record (source line 1) with a note.
A verdict line the file itself calls a placeholder keeps its value but gets status n/a, since the body states
the verdict and the line does not.
"""
from __future__ import annotations

import re
from pathlib import Path

from ._common import is_dirty, last_commit, make

BASE = "research/ideation/round2/experiments"
VERDICT_RE = re.compile(r"^verdict:\s*([A-Za-z]+)\b")
PLACEHOLDER_WINDOW = 8   # lines after a verdict line that may say it is a placeholder


def sources(root: Path) -> list[Path]:
    return sorted((root / BASE).glob("*/RESULT.md"))


def load(root: Path):
    out = []
    for path in sources(root):
        rel = path.relative_to(root).as_posix()
        dirname = path.parent.name
        commit = last_commit(root, rel)
        dirty = is_dirty(root, rel)
        lines = path.read_text(encoding="utf-8").splitlines()
        hit = next(((i, m.group(1)) for i, ln in enumerate(lines) if (m := VERDICT_RE.match(ln))), None)
        if hit is None:
            out.append(make(workload=f"experiment.{dirname}", metric="verdict", value=None, unit="",
                            rel=rel, line=1, params={}, command="", commit=commit, dirty=dirty,
                            status="n/a", note="no `verdict:` line in RESULT.md"))
            continue
        idx, word = hit
        window = "\n".join(lines[idx + 1: idx + 1 + PLACEHOLDER_WINDOW]).lower()
        if "placeholder" in window:
            out.append(make(workload=f"experiment.{dirname}", metric="verdict", value=word, unit="",
                            rel=rel, line=idx + 1, params={}, command="", commit=commit, dirty=dirty,
                            status="n/a",
                            note=f"placeholder verdict line (value {word} kept as written); "
                                 "the file's body states the verdict"))
            continue
        out.append(make(workload=f"experiment.{dirname}", metric="verdict", value=word, unit="",
                        rel=rel, line=idx + 1, params={}, command="", commit=commit, dirty=dirty))
    return out

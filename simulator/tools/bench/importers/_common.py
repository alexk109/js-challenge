"""Shared reading helpers for the importers. Stdlib only.

Every importer turns rows of a file already in the repo into `types.Record`s with level and backend
"import". Numbers are read from the file, never typed in. Empty cells become status "n/a" (value None),
never 0.

Config: a row gets the MVP config only when its source says it is the MVP (or a named bench config). Any
other row gets `config={}` and `config_hash="external:<name>"`, so it never joins as if it were the MVP.
"""
from __future__ import annotations

import csv
import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

from .. import knobs
from ..types import Record

INT_RE = re.compile(r"^[+-]?\d+$")
NUM_RE = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$")

#: The MVP config (the default for every row that the source says is the MVP).
CONFIG = knobs.normalize({})
CONFIG_HASH = knobs.config_hash(CONFIG)
#: The MVP with the IHP two-port SRAM program memory.
SRAM_CONFIG = knobs.normalize({"imem_tech": "sram2p"})
SRAM_CONFIG_HASH = knobs.config_hash(SRAM_CONFIG)

#: Returned by a config resolver: (config, config_hash, note to add).
Resolved = tuple


def mvp(_params: dict) -> Resolved:
    return dict(CONFIG), CONFIG_HASH, ""


def external(name: str, why: str = "not a bench config") -> Resolved:
    return {}, f"external:{name}", f"{why}: recorded as external:{name}, not as the MVP"


def parse(text: str):
    """int or float when the text is a plain decimal number, else the text unchanged."""
    if INT_RE.match(text):
        return int(text)
    if NUM_RE.match(text):
        return float(text)
    return text


def is_num(text: str) -> bool:
    return text != "" and NUM_RE.match(text) is not None


def snake(name: str) -> str:
    return re.sub(r"[^0-9a-zA-Z]+", "_", name).strip("_").lower()


_SUFFIX_UNITS = (("_um2", "um2"), ("_ns", "ns"), ("_pct", "%"), ("_mhz", "MHz"), ("_s", "s"))


def unit_from_name(column: str) -> str:
    """Unit read from a column-name suffix (e.g. `runtime_s`). Empty when the name says nothing."""
    for suffix, unit in _SUFFIX_UNITS:
        if column.endswith(suffix):
            return unit
    return ""


def git(root: Path, *args: str) -> str:
    out = subprocess.run(["git", *args], cwd=str(root), capture_output=True, text=True)
    return out.stdout.strip() if out.returncode == 0 else ""


def last_commit(root: Path, rel: str) -> str:
    """Hash of the last commit touching the file (`git log -1 --format=%H -- path`)."""
    return git(root, "log", "-1", "--format=%H", "--", rel)


def is_dirty(root: Path, rel: str) -> bool:
    return git(root, "status", "--porcelain", "--", rel) != ""


@dataclass
class Section:
    """Rows under one header line. A CSV can repeat its header mid-file (a schema change or an append)."""
    header: list[str]
    rows: list[tuple[int, list[str]]] = field(default_factory=list)   # (physical line, cells)


def read_sections(path: Path) -> list[Section]:
    """Split a CSV into sections. A row starting with the first header's first two cells is a new header."""
    with open(path, newline="") as fh:
        reader = csv.reader(fh)
        sections: list[Section] = []
        first: Optional[list[str]] = None
        cur: Optional[Section] = None
        for cells in reader:
            if not cells or all(c == "" for c in cells):
                continue
            if first is None:
                first = cells
                cur = Section(header=cells)
                sections.append(cur)
                continue
            if cells[:2] == first[:2]:
                cur = Section(header=cells)
                sections.append(cur)
                continue
            if len(cells) < len(cur.header):
                cells = cells + [""] * (len(cur.header) - len(cells))
            cur.rows.append((reader.line_num, cells))
    return sections


def make(*, workload: str, metric: str, value, unit: str, rel: str, line: int, params: dict,
         command: str, commit: str, dirty: bool, method: str = "imported",
         status: str = "ok", note: str = "",
         config: Optional[dict] = None, config_hash: Optional[str] = None) -> Record:
    """One record. `config`/`config_hash` default to the MVP; a source row that is not the MVP passes its own."""
    if value is None and status == "ok":
        status = "n/a"
    return Record(
        workload=workload,
        metric=metric,
        value=value,
        unit=unit,
        level="import",
        backend="import",
        method=method,
        status=status,
        band=None,
        config=dict(CONFIG) if config is None else dict(config),
        config_hash=CONFIG_HASH if config_hash is None else config_hash,
        workload_params=params,
        note=note,
        provenance={
            "source": f"{rel}:{line}",
            "command": command,
            "commit": commit,
            "dirty": dirty,
        },
    )


def load_metrics_csv(root: Path, rel: str, prefix: str, workload_col: str, *,
                     exclude: tuple = (),
                     command: Callable[[dict], str] = lambda p: "",
                     param_cols: tuple = (),
                     config_for: Callable[[dict], Resolved] = mvp,
                     renames: Optional[dict] = None) -> list[Record]:
    """A wide table (one row per run, one column per quantity).

    workload = the identifying column; metric = each numeric column (a column every non-empty cell of
    which parses as a number), named `<prefix>_<snake(column)>` unless `renames` maps the column to
    `(metric_name, unit)`. workload_params holds only the real parameters named in `param_cols`; every other
    non-numeric column (date, commit, harness, free text ...) goes into the note as "row: col=value; ...".
    Columns in `exclude` are identifiers, never metrics. `config_for(params)` resolves each row's config.
    """
    renames = renames or {}
    commit = last_commit(root, rel)
    dirty = is_dirty(root, rel)
    out: list[Record] = []
    for sec in read_sections(root / rel):
        hdr = sec.header
        ident = hdr.index(workload_col)
        numeric = []
        for i in range(len(hdr)):
            column = [cells[i] for _, cells in sec.rows]
            filled = [c for c in column if c != ""]
            numeric.append(bool(filled) and all(is_num(c) for c in filled))
        for line, cells in sec.rows:
            allp = {c: (parse(cells[i]) if numeric[i] else cells[i]) for i, c in enumerate(hdr) if cells[i] != ""}
            params = {c: allp[c] for c in param_cols if c in allp}
            rowtxt = "; ".join(f"{c}={cells[i]}" for i, c in enumerate(hdr)
                               if cells[i] != "" and not numeric[i] and i != ident and c not in param_cols)
            workload = cells[ident] or "(blank)"
            row_cmd = command(allp)
            config, config_hash, cnote = config_for(allp)
            for i, c in enumerate(hdr):
                if i == ident or c in exclude or not numeric[i]:
                    continue
                if c in renames:
                    metric, unit = renames[c]
                    rnote = f"source column {c} of {rel}"
                else:
                    metric, unit = f"{prefix}_{snake(c)}", unit_from_name(c)
                    rnote = ""
                note = "; ".join(n for n in (cnote, rnote, ("row: " + rowtxt) if rowtxt else "") if n)
                if cells[i] == "":
                    out.append(make(workload=workload, metric=metric, value=None, unit=unit,
                                    rel=rel, line=line, params=params, command=row_cmd,
                                    commit=commit, dirty=dirty, status="n/a",
                                    note="; ".join(n for n in (note, "empty in source") if n),
                                    config=config, config_hash=config_hash))
                else:
                    out.append(make(workload=workload, metric=metric, value=parse(cells[i]),
                                    unit=unit, rel=rel, line=line, params=params, command=row_cmd,
                                    commit=commit, dirty=dirty, note=note,
                                    config=config, config_hash=config_hash))
    return out


def load_long_csv(root: Path, rel: str, *, workload_col: str, metric_col: str, value_col: str,
                  unit_col: str, note_col: str, param_cols: tuple, method: str,
                  command: str = "",
                  params_fn: Optional[Callable[[dict], tuple]] = None,
                  skip_row: Optional[Callable[[dict], bool]] = None) -> list[Record]:
    """A long table (one row per workload x metric). One record per row; value kept as parsed (text stays text).

    `params_fn(row)` (row = {column: text}) returns `(workload_params, note_prefix)` when a source needs its
    params shaped by hand; otherwise `param_cols` are copied as parsed values.
    """
    commit = last_commit(root, rel)
    dirty = is_dirty(root, rel)
    out: list[Record] = []
    for sec in read_sections(root / rel):
        hdr = sec.header
        ix = {c: hdr.index(c) for c in hdr}
        for line, cells in sec.rows:
            row = {c: cells[ix[c]] for c in hdr}
            if skip_row and skip_row(row):
                continue
            if params_fn is None:
                params = {c: parse(row[c]) if is_num(row[c]) else row[c] for c in param_cols if row[c] != ""}
                prefix = ""
            else:
                params, prefix = params_fn(row)
            value_text = row[value_col]
            if value_text.strip().lower() == "n/a":      # the source says nobody observed it: n/a, not the string "n/a"
                value_text = ""
            note = "; ".join(n for n in (prefix.rstrip("; "), row[note_col] if note_col else "") if n)
            out.append(make(
                workload=row[workload_col] or "(blank)",
                metric=row[metric_col],
                value=parse(value_text) if value_text != "" else None,
                unit=row[unit_col],
                rel=rel, line=line, params=params, command=command, commit=commit, dirty=dirty,
                method=method, note=note,
            ))
    return out

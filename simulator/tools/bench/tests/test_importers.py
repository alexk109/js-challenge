"""Importers of past evidence: counts are checked against the source files with the csv module, not the importer."""
import csv
import re
from pathlib import Path

import pytest

from .. import knobs
from ..backends.importer import ImporterBackend
from ..importers import SOURCES, load_all, module
from ..types import Workload

ROOT = Path(__file__).resolve().parents[3]
EXPERIMENTS = ROOT / "research/ideation/round2/experiments"

# (importer module, source file, identifying column, excluded numeric columns)
WIDE = [
    ("flow", "metrics/flow.csv", "cfg", ()),
    ("synth", "metrics/synth.csv", "probe", ()),
    ("fw", "metrics/fw.csv", "proto", ()),
    ("formal", "metrics/formal.csv", "property", ()),
    ("fuzz", "metrics/fuzz.csv", "harness", ("seed",)),
    ("mutate", "metrics/mutate.csv", "feature", ()),
    ("coverage", "metrics/coverage.csv", "bin", ()),
]
LONG = [
    ("perf_a", "research/perf/a/perf.csv"),
    ("perf_b", "research/perf/b/perf.csv"),
]


def _is_float(text):
    try:
        float(text)
        return True
    except ValueError:
        return False


def _expected_wide(path, ident, exclude):
    """Rows x numeric columns, per header section. Numeric = every non-empty cell parses, and some cell is set."""
    rows = [r for r in csv.reader(open(path, newline="")) if any(c != "" for c in r)]
    first = rows[0]
    header, data, total = first, [], 0
    sections = []
    for r in rows[1:]:
        if r[:2] == first[:2]:          # a repeated header line starts a new section
            sections.append((header, data))
            header, data = r, []
        else:
            data.append(r)
    sections.append((header, data))
    for hdr, body in sections:
        for i, col in enumerate(hdr):
            if col == ident or col in exclude:
                continue
            cells = [r[i] for r in body if i < len(r) and r[i] != ""]
            if cells and all(_is_float(c) for c in cells):
                total += len(body)
    return total


def _data_rows(path):
    rows = list(csv.reader(open(path, newline="")))
    return len(rows) - 1


@pytest.mark.parametrize("name,rel,ident,exclude", WIDE, ids=[w[0] for w in WIDE])
def test_wide_csv_record_count(name, rel, ident, exclude):
    records = module(name).load(ROOT)
    assert len(records) == _expected_wide(ROOT / rel, ident, exclude)


@pytest.mark.parametrize("name,rel", LONG, ids=[w[0] for w in LONG])
def test_long_csv_one_record_per_row(name, rel):
    skipped = 0
    if name == "perf_b":    # per-protocol copies of the host-port rows are skipped; spi_host_port keeps them
        with open(ROOT / rel, newline="") as fh:
            skipped = sum(1 for r in csv.DictReader(fh) if r["protocol"] != "spi_host_port" and r["knob"] == "ticks_per_sck_bit")
        assert skipped
    assert len(module(name).load(ROOT)) == _data_rows(ROOT / rel) - skipped


def test_every_source_is_registered():
    assert set(SOURCES) == {w[0] for w in WIDE} | {w[0] for w in LONG} | {"experiments"}


@pytest.mark.skipif(not EXPERIMENTS.exists(), reason="optional experiments dir not present")
def test_experiment_verdicts_one_per_dir():
    dirs = sorted(p.parent.name for p in EXPERIMENTS.glob("*/RESULT.md"))
    assert dirs, "no RESULT.md found"
    records = module("experiments").load(ROOT)
    assert len(records) == len(dirs)
    by_workload = {r.workload: r for r in records}
    for d in dirs:
        text = (EXPERIMENTS / d / "RESULT.md").read_text(encoding="utf-8")
        if re.search(r"^verdict:", text, re.M):
            rec = by_workload[f"experiment.{d}"]
            assert rec.value in ("PASS", "KILL", "AMBER", "INCONCLUSIVE"), (d, rec.value)
            if "placeholder" in rec.note:
                assert rec.status == "n/a", d            # placeholder: value kept, not trusted
            else:
                assert rec.status == "ok", d


@pytest.mark.skipif(not EXPERIMENTS.exists(), reason="optional experiments dir not present")
def test_placeholder_verdicts_are_na_with_value_kept():
    by = {r.workload: r for r in module("experiments").load(ROOT)}
    for d in ("experiment.C2-armslack-a", "experiment.C2-armslack-b"):
        assert by[d].status == "n/a" and by[d].value is not None and "placeholder" in by[d].note


def test_every_record_has_source_line_and_commit():
    records = load_all(ROOT)
    assert records
    for r in records:
        assert r.level == "import" and r.backend.startswith("import:"), r.backend
        assert r.method in ("imported", "derived")
        src = r.provenance["source"]
        m = re.fullmatch(r"(.+):(\d+)", src)
        assert m and int(m.group(2)) >= 1, src
        assert (ROOT / m.group(1)).exists(), src
        assert r.provenance["commit"], src
        if r.status == "n/a":
            assert r.value is None or "placeholder" in r.note, src
            assert r.note, src


def test_config_is_mvp_or_named_external():
    mvp_hash = knobs.config_hash(knobs.normalize({}))
    sram_hash = knobs.config_hash(knobs.normalize({"imem_tech": "sram2p"}))
    for r in load_all(ROOT):
        if r.config_hash == mvp_hash:
            assert r.config == knobs.normalize({})
        elif r.config_hash == sram_hash:
            assert r.config == knobs.normalize({"imem_tech": "sram2p"})
        else:
            assert r.config == {} and r.config_hash.startswith("external:"), r.config_hash


def test_flow_config_mapping():
    for r in module("flow").load(ROOT):
        cfg = r.workload
        if cfg in ("mvp", "mvp-slow"):
            assert r.config_hash == knobs.config_hash(knobs.normalize({}))
        elif cfg == "mvp-sram" and "tag=mvp-sram;" in r.note + ";" and "status=ok" in r.note:
            assert r.config_hash == knobs.config_hash(knobs.normalize({"imem_tech": "sram2p"})), r.note
        elif cfg == "mvp-sram":
            assert r.config_hash.startswith("external:mvp-sram") and r.config == {}, r.note
        else:
            assert r.config_hash == f"external:{cfg}" and r.config == {}, cfg


def test_synth_always_external():
    for r in module("synth").load(ROOT):
        assert r.config == {} and r.config_hash == f"external:{r.workload}"


def test_fuzz_sram_harness_maps_to_sram_config():
    recs = module("fuzz").load(ROOT)
    sram = [r for r in recs if r.workload == "lockstep-sram"]
    assert sram, "no lockstep-sram rows"
    for r in sram:
        assert r.config == knobs.normalize({"imem_tech": "sram2p"})
        assert r.config_hash == knobs.config_hash(knobs.normalize({"imem_tech": "sram2p"}))


def test_flow_joinable_names_and_source_column_note():
    names = {r.metric for r in module("flow").load(ROOT)}
    assert {"area_um2", "fmax_mhz_typ", "fmax_mhz_slow", "wns_ns"} <= names
    assert not names & {"flow_stdcell_area_um2", "flow_fmax_tt", "flow_fmax_ss", "flow_wns"}
    for r in module("flow").load(ROOT):
        if r.metric == "fmax_mhz_typ":
            assert r.unit == "MHz" and "source column fmax_tt" in r.note
        if r.metric == "wns_ns":
            assert r.unit == "ns" and "source column WNS" in r.note
    assert "area_um2" in {r.metric for r in module("synth").load(ROOT)}


def test_perf_b_params_use_sim_shape():
    for r in module("perf_b").load(ROOT):
        keys = set(r.workload_params)
        if r.workload == "spi_host_port":
            assert keys == {"ticks_per_sck_bit"} and isinstance(r.workload_params["ticks_per_sck_bit"], int)
        else:
            assert keys == {"speed"} and isinstance(r.workload_params["speed"], float), r.workload
        assert "target=" in r.note and "knob=" in r.note


def test_perf_b_params_equal_sim_workloads():
    """The iss backend's Workload for the same protocol and speed has the same params (skipped until
    tools/bench/workloads.py from wb/bench is merged)."""
    try:
        from ..workloads import workload
    except ImportError as exc:
        pytest.skip(f"sim workloads not available: {exc}")
    checked = 0
    for r in module("perf_b").load(ROOT):
        if "speed" in r.workload_params:
            w = workload(r.workload, r.workload_params["speed"])
        elif r.workload == "spi_host_port":
            w = workload(r.workload, r.workload_params["ticks_per_sck_bit"])
        else:
            continue
        assert w.params == r.workload_params, (r.workload, r.workload_params)
        checked += 1
    assert checked > 0


def test_flow_and_synth_name_their_config():
    for r in module("flow").load(ROOT) + module("synth").load(ROOT):
        assert r.workload_params == {}, r.workload_params          # no raw CSV row as params


def test_perf_a_is_derived_and_perf_b_imported():
    assert all(r.method == "derived" for r in module("perf_a").load(ROOT))
    assert all(r.method == "imported" for r in module("perf_b").load(ROOT))


def test_backend_refuses_non_mvp_and_runs_all():
    b = ImporterBackend()
    assert b.supports({}, Workload("all", "import")) is None
    assert b.supports({}, Workload("uart_tx", "protocol")) is not None
    assert b.supports({"n_engines": 3}, Workload("all", "import")) is not None
    assert set(b.knob_support()) == {k.id for k in knobs.KNOBS}
    assert len(b.run(None)) == len(load_all(ROOT))
    assert len(b.code_hash()) == 24


def test_reimport_appends_nothing(tmp_path):
    """`bench import` is cached on the sources' content: a second import of unchanged sources adds no records
    (it used to append a full copy of the past evidence on every bench-report)."""
    import subprocess, sys
    store = tmp_path / "records.jsonl"
    cmd = [sys.executable, "-m", "tools.bench", "--store", str(store), "import"]
    for _ in range(2):
        subprocess.run(cmd, cwd=ROOT, check=True, capture_output=True)
        n = sum(1 for _ in store.open())
        if _ == 0:
            first = n
    assert first > 1000 and n == first

"""The smoke assertions, with negative controls (the end-to-end run itself is `make bench-smoke`)."""
import json

from .. import smoke


def side(backend):
    return {"level": "x", "backend": backend, "value": 1, "band": None, "unit": "", "method": "", "id": ""}


def finding(verdict, a="analytic", b="iss", w="uart_tx", m="max_bit_rate"):
    return {"workload": w, "metric": m, "config_hash": "c1", "workload_params": {"speed": 1.0}, "verdict": verdict, "detail": "d", "a": side(a), "b": side(b)}


def agreeing():
    return [finding("match", "analytic", "iss"), finding("match", "iss", "iss_b"),
            finding("close", "iss", "rtl"), finding("match", "iss", "import:perf_b:iss")]


def test_clean_run_passes():
    problems, stats = smoke.assess(agreeing(), [])
    assert problems == [] and stats["verdicts"] == {"match": 3, "close": 1}


def test_unexpected_off_is_a_problem():
    problems, _ = smoke.assess(agreeing() + [finding("off", "analytic", "iss")], [])
    assert len(problems) == 1 and "unexpected off" in problems[0]


def test_allowlisted_off_passes_and_is_matched_by_backend():
    allow = [{"workload": "uart_tx", "metric": "max_bit_rate", "config_hash": "c1", "workload_params": {"speed": 1.0},
              "backends": ["analytic"], "reason": "r"}]
    assert smoke.assess(agreeing() + [finding("off", "analytic", "iss")], allow)[0] == []
    other_cfg = {**finding("off", "analytic", "iss"), "config_hash": "c2"}
    other_pt = {**finding("off", "analytic", "iss"), "workload_params": {"speed": 2.0}}
    assert smoke.assess(agreeing() + [other_cfg], allow)[0] != [] and smoke.assess(agreeing() + [other_pt], allow)[0] != []
    assert smoke.assess(agreeing() + [finding("off", "iss", "iss_b")], allow)[0] != []


def test_missing_side_is_a_problem():
    problems, _ = smoke.assess([finding("match", "analytic", "iss")], [])
    assert any("iss_b" in p for p in problems) and any("rtl" in p for p in problems)


def test_allowlist_file_is_well_formed():
    for e in json.loads(smoke.ALLOW.read_text()):
        assert {"workload", "metric", "config_hash", "workload_params", "backends", "reason"} <= set(e) and e["reason"].strip()


def test_describe_stale_names_entries_not_indices():
    allow = [{"workload": "w", "metric": "m", "backends": ["a", "b"], "reason": "x" * 100}]
    (line,) = smoke.describe_stale(allow, [0])
    assert line.startswith("w/m [a+b] " + "x" * 60) and line.endswith("...")

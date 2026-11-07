"""Bench knobs that archsim also has: same id, bench domain a subset of archsim's, same default."""
import json

import pytest

quickjs = pytest.importorskip("quickjs")

from tools.bench import knobs
from tools.bench.backends import analytic as A
from tools.bench.types import Workload


@pytest.fixture(scope="module")
def archsim_knobs():
    ctx = quickjs.Context()
    ctx.eval(A.MODEL_JS.read_text())
    return {k["id"]: k for k in json.loads(ctx.eval("JSON.stringify(ArchSim.KNOBS)"))}


def _check(k, a):
    problems = []
    if k.default != a["default"]:
        problems.append(f"default bench {k.default!r} vs archsim {a['default']!r}")
    if k.kind in ("int", "float"):
        lo, hi = k.domain
        if not (a["min"] <= lo and hi <= a["max"]):
            problems.append(f"range bench {lo}..{hi} not inside archsim {a['min']}..{a['max']}")
    elif k.kind == "enum":
        extra = set(k.domain) - set(a["options"])
        if extra:
            problems.append(f"bench domain values {sorted(extra, key=str)} not in archsim options {a['options']}")
    return problems


SHARED = [k.id for k in knobs.KNOBS if k.id not in A.BENCH_ONLY]


def _outside(k, a):
    """Bench domain values archsim's domain does not contain."""
    if k.kind == "enum":
        return [v for v in k.domain if v not in a["options"]]
    if k.kind in ("int", "float"):
        return [v for v in k.domain if not a["min"] <= v <= a["max"]]
    return []


@pytest.mark.parametrize("kid", SHARED)
def test_knob_in_sync(kid, archsim_knobs):
    """Same id and default; every bench value archsim lacks is refused by supports(), naming the knob."""
    assert kid in archsim_knobs, f"{kid} is not bench-only but archsim lacks it"
    k, a = knobs.BY_ID[kid], archsim_knobs[kid]
    assert k.default == a["default"]
    be = A.AnalyticBackend()
    for v in _outside(k, a):
        cfg = dict(knobs.defaults(), **{kid: v})
        reason = be.supports(cfg, Workload("uart_3m", "protocol"))
        assert reason and kid in reason, (kid, v, reason)
    support = be.knob_support()[kid]
    assert (support == "full") == (not _outside(k, a)), (kid, support)


def test_imem_words_16_is_refused():
    be = A.AnalyticBackend()
    assert "imem_words" in be.supports(knobs.normalize({"imem_words": 16}), Workload("uart_3m", "protocol"))
    assert be.knob_support()["imem_words"].startswith("partial") and "refused" in be.knob_support()["imem_words"]


def test_bench_only_knobs_are_absent_from_archsim(archsim_knobs):
    for kid in A.BENCH_ONLY:
        assert kid not in archsim_knobs
        assert "bench-only" in knobs.BY_ID[kid].help


def test_arch_knobs_all_exist_in_archsim(archsim_knobs):
    assert set(A.ARCH_KNOBS) <= set(archsim_knobs)

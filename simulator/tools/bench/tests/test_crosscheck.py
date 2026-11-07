"""Cross-check rules, each with a case that makes it fail (negative controls)."""
from tools.bench import crosscheck
from tools.bench.types import Record


def rec(metric="sm_cycles_per_bit", value=100, unit="cycles", level="iss", backend="a", band=None, wp=None,
        status="ok", ch="c1", w="uart_tx"):
    return Record(workload=w, metric=metric, value=value, unit=unit, level=level, backend=backend,
                  method="measured", status=status, band=band, config_hash=ch, workload_params=wp or {})


def verdicts(*recs):
    return [f["verdict"] for f in crosscheck.findings(recs)]


def test_match():
    assert verdicts(rec(), rec(level="rtl", backend="b")) == ["match"]


def test_cycle_counts_are_exact():
    assert verdicts(rec(value=100), rec(value=101, level="rtl", backend="b")) == ["off"]
    assert verdicts(rec(value=1000), rec(value=1001, level="rtl", backend="b")) == ["off"]


def test_rates_close_within_2pct_off_beyond():
    kw = dict(metric="max_bit_rate", unit="Hz")
    assert verdicts(rec(value=1000, **kw), rec(value=1015, level="rtl", backend="b", **kw)) == ["close"]
    assert verdicts(rec(value=1000, **kw), rec(value=1100, level="rtl", backend="b", **kw)) == ["off"]


def test_per_metric_tolerance_override(monkeypatch):
    monkeypatch.setitem(crosscheck.TOLERANCES, "max_bit_rate", ("rel", 0.5))
    kw = dict(metric="max_bit_rate", unit="Hz")
    assert verdicts(rec(value=1000, **kw), rec(value=1300, level="rtl", backend="b", **kw)) == ["close"]


def test_band_overlap_and_disjoint():
    kw = dict(metric="margin", unit="MHz")
    a = rec(value=10, band=[5, 15], **kw)
    assert verdicts(a, rec(value=14, level="analytic", backend="b", **kw)) == ["close"]
    assert verdicts(a, rec(value=40, level="analytic", backend="b", **kw)) == ["off"]
    b = rec(value=50, band=[45, 55], level="analytic", backend="b", **kw)
    assert verdicts(a, b) == ["off"]


def test_strings_exact():
    kw = dict(metric="verdict", unit="")
    assert verdicts(rec(value="served", **kw), rec(value="served", level="rtl", backend="b", **kw)) == ["match"]
    assert verdicts(rec(value="served", **kw), rec(value="impossible", level="rtl", backend="b", **kw)) == ["off"]


def test_one_sided():
    assert verdicts(rec()) == ["one-sided"]
    # different config / params do not join
    assert verdicts(rec(), rec(level="rtl", backend="b", ch="c2")) == ["one-sided", "one-sided"]
    assert verdicts(rec(wp={"speed": 1}), rec(level="rtl", backend="b", wp={"speed": 2})) == ["one-sided", "one-sided"]


def test_na_and_error_do_not_participate():
    assert verdicts(rec(), rec(level="rtl", backend="b", value=None, status="n/a")) == ["one-sided"]
    assert verdicts(rec(), rec(level="rtl", backend="b", status="error")) == ["one-sided"]


def test_same_level_different_backends_and_latest_wins():
    f = crosscheck.findings([rec(value=1), rec(value=100), rec(backend="b", value=100)])
    assert [x["verdict"] for x in f] == ["match"]


def test_summary_counts():
    f = crosscheck.findings([rec(), rec(level="rtl", backend="b", value=5), rec(metric="verdict", value="x", unit="")])
    assert crosscheck.summary(f) == {"match": 0, "close": 0, "off": 1, "identity": 0, "one-sided": 1}

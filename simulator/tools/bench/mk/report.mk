# make bench-report: run the quick levels, check, export tools/bench/snapshots/latest.html, snapshot "latest" and print a
# one-screen summary (tools/bench/report.py). Under ~10 minutes; touches the current store (BENCH_STORE).
.PHONY: bench-report
bench-report:
	PYTHONPATH=. $(PY) -m tools.bench.report $(REPORT_ARGS)

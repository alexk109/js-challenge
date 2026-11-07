# make bench-smoke: the whole bench end to end (import, analytic, iss, iss_b, one rtl, check) and the cross-level assertions
# (tools/bench/smoke.py; known `off` findings and their reasons: tools/bench/smoke_allow.json). About 8 s fresh, 5 s cached.
.PHONY: bench-smoke
bench-smoke:
	PYTHONPATH=. $(PY) -m tools.bench.smoke $(SMOKE_ARGS)

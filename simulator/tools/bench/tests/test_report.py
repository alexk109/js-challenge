"""bench-report summary, and the quiet server (no traceback when a client disconnects)."""
from tools.bench import report
from tools.bench.ui import server


def test_summarize_counts_and_unexpected_offs():
    side = {"level": "iss", "backend": "iss", "value": 1}
    f = [{"verdict": "off", "workload": "w", "metric": "m", "config_hash": "h", "workload_params": {}, "a": side,
          "b": dict(side, backend="rtl"), "detail": "x"},
         {"verdict": "match", "workload": "w", "metric": "n", "config_hash": "h", "workload_params": {}, "a": side, "b": None, "detail": ""}]
    recs = [{"level": "iss", "status": "ok"}, {"level": "synth", "status": "ok"}]
    out = report.summarize(f, recs, [])
    assert "iss 1, synth 1" in out and "match 1" in out and "unexpected offs (not in smoke_allow.json): 1" in out
    allow = [{"workload": "w", "metric": "m", "config_hash": "h", "workload_params": {}, "backends": ["iss", "rtl"]}]
    assert "unexpected offs (not in smoke_allow.json): 0" in report.summarize(f, recs, allow)


def test_quiet_server_swallows_disconnects(capsys):
    srv = server.QuietServer.__new__(server.QuietServer)
    try:
        raise BrokenPipeError()
    except BrokenPipeError:
        srv.handle_error(None, ("127.0.0.1", 1))
    err = capsys.readouterr().err
    assert "disconnected" in err and "Traceback" not in err

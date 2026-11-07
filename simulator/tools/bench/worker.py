"""Worker subprocess: `python -m tools.bench.worker` (CONTRACT.md section 3).

Reads one RunSpec as JSON on stdin, runs it on its backend, writes one Record JSON per stdout line.
It is a separate process because `fw/targets.py` installs a chip class globally and the RTL harness
owns child processes. Exit 0 even when records say `fail`; non-zero only when the backend crashed.
Anything the backend prints to stdout would corrupt the stream, so stdout is redirected to stderr
at the file-descriptor level while the backend runs; records go out on a dup of the original fd 1.
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import asdict

from .types import RunSpec, Workload


def spec_from_json(text: str) -> RunSpec:
    d = json.loads(text)
    return RunSpec(config=d["config"], workload=Workload(**d["workload"]), backend=d["backend"])


def spec_to_json(spec: RunSpec) -> str:
    return json.dumps({"config": spec.config, "workload": asdict(spec.workload), "backend": spec.backend})


def main() -> int:
    from .backends import load
    spec = spec_from_json(sys.stdin.read())
    # Keep a private copy of the record pipe, then point fd 1 at stderr: child processes (vvp,
    # verilator, iverilog) inherit fd 1 and must not be able to write into the record stream.
    sys.stdout.flush()
    real_out = os.fdopen(os.dup(1), "w")
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    records = load(spec.backend).run(spec)
    for r in records:
        real_out.write(r.to_json() + "\n")
    real_out.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())

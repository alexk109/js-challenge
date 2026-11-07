"""Backend registry: name -> "module.path:ClassName", imported lazily (CONTRACT.md section 3).

Each task appends its own line. Keep one backend per line so parallel merges don't conflict.
"""
from __future__ import annotations

import importlib

REGISTRY: dict[str, str] = {
    "iss": "tools.bench.backends.sim:Iss",
    "iss_b": "tools.bench.backends.sim:IssB",
    "rtl": "tools.bench.backends.sim:Rtl",
    "iss_p": "tools.bench.backends.sim_p:IssP",
    "analytic": "tools.bench.backends.analytic:AnalyticBackend",
    "import": "tools.bench.backends.importer:ImporterBackend",
    "synth": "tools.bench.backends.physical:Synth",
    "flow": "tools.bench.backends.physical:Flow",
    "iss_isa": "tools.bench.backends.isa:IssIsa",
    "fake": "tools.bench.backends.fake:Fake",
    "fake2": "tools.bench.backends.fake:Fake2",
    "power": "tools.bench.backends.power:Power",
    "gl": "tools.bench.backends.gatelevel:GateLevel",
    "gl_rig": "tools.bench.backends.gl_rig:GlRig",
}


class UnknownBackend(KeyError):
    pass


def load(name: str):
    """Instantiate a backend by name. Import errors surface here, for that backend only."""
    try:
        target = REGISTRY[name]
    except KeyError:
        raise UnknownBackend(f"unknown backend {name!r}; known: {', '.join(sorted(REGISTRY)) or 'none'}") from None
    mod, cls = target.split(":")
    return getattr(importlib.import_module(mod), cls)()

import json, pathlib, pytest
try:
    import quickjs
except ImportError as e:
    raise RuntimeError("quickjs missing: run `make archsim-setup`") from e

ROOT = pathlib.Path(__file__).resolve().parents[1]          # tools/archsim

def _inf(x):                                                  # "Infinity" strings back to floats, recursively
    if isinstance(x, str) and x in ("Infinity", "-Infinity"):
        return float(x.replace("Infinity", "inf"))
    if isinstance(x, list):
        return [_inf(v) for v in x]
    if isinstance(x, dict):
        return {k: _inf(v) for k, v in x.items()}
    return x

@pytest.fixture(scope="session")
def data():
    return json.loads((ROOT / "data/inputs.json").read_text())

@pytest.fixture(scope="session")
def js(data):
    ctx = quickjs.Context()
    ctx.eval((ROOT / "model/archsim-model.js").read_text())
    from tools.archsim.build_html import UI_FILES             # one fixed order for page and tests
    for f in UI_FILES:                                        # boot.js is a no-op without a DOM
        ctx.eval((ROOT / "ui" / f).read_text())
    ctx.eval("var DATA = " + json.dumps(data) + ";")
    def call(expr):
        return _inf(json.loads(ctx.eval("JSON.stringify(ArchSim.toJSON(" + expr + "))")))
    return call

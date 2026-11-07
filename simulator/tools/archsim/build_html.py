"""Assemble tools/archsim/archsim.html (PLAN 4.6): inline CSS, inputs.json, model JS and UI JS into one offline page.

CLI: python -m tools.archsim.build_html
"""
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent
OUT = ROOT / "archsim.html"
# Fixed concatenation order of the UI bundle (tests/conftest.py loads the same list; boot.js must stay last).
UI_FILES = ["archsim-ui.js", "view-configure.js", "view-capability.js", "view-timing.js", "view-area.js",
            "view-throughput.js", "view-explore.js", "view-trust.js", "boot.js"]
# Fixed concatenation order of the stylesheet: core tokens/layout/shared components first, then one file per area.
CSS_FILES = ["archsim.css"] + [f"css/{n}.css" for n in
                               ("configure", "capability", "timing", "area", "throughput", "explore", "trust")]
PLACEHOLDERS = ["/*@@CSS@@*/", "<!--@@DATA@@-->", "/*@@MODEL@@*/", "/*@@UI@@*/"]


def _script_safe(js: str, name: str) -> str:
    if re.search(r"</script|<!--", js, re.I):
        raise ValueError(f"{name} contains '</script' or '<!--', which would end the inline script")
    return js


def data_script(data_text: str) -> str:
    data = json.loads(data_text)                                     # validates, and lets us minify
    body = json.dumps(data, separators=(",", ":"), ensure_ascii=False)
    body = body.replace("</", "<\\/").replace("<!--", "\\u003c!--")  # both stay valid JSON
    return '<script id="archsim-data" type="application/json">' + body + "</script>"


def build(data_text: str | None = None) -> str:
    tpl = (ROOT / "ui/template.html").read_text()
    for ph in PLACEHOLDERS:
        if tpl.count(ph) != 1:
            raise ValueError(f"template.html must contain {ph} exactly once")
    ui = "\n".join((ROOT / "ui" / f).read_text().rstrip() + "\n" for f in UI_FILES)
    parts = {
        "/*@@CSS@@*/": "\n".join((ROOT / "ui" / f).read_text().rstrip() + "\n" for f in CSS_FILES),
        "<!--@@DATA@@-->": data_script(data_text if data_text is not None else (ROOT / "data/inputs.json").read_text()),
        "/*@@MODEL@@*/": _script_safe((ROOT / "model/archsim-model.js").read_text(), "model"),
        "/*@@UI@@*/": _script_safe(ui, "ui"),
    }
    html = tpl
    for ph, text in parts.items():
        html = text.join(html.split(ph))                           # no regex, so no backslash processing
    return html


def main() -> int:
    html = build()
    OUT.write_text(html)
    print(f"{OUT.relative_to(ROOT.parent.parent)}: {len(html.encode()) / 1024:.0f} KiB")
    return 0


if __name__ == "__main__":
    sys.exit(main())

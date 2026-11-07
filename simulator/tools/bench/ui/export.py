"""Static export: one self-contained HTML file with the records embedded (no server, no external URLs)."""
from __future__ import annotations

import json
import os
import re

from . import api

STATIC = os.path.join(os.path.dirname(__file__), "static")


def _read(name: str) -> str:
    with open(os.path.join(STATIC, name), encoding="utf-8") as f:
        return f.read()


def _js_json(obj) -> str:
    # safe inside <script>: no closing tag, no HTML comment opener
    return json.dumps(obj, default=str).replace("</", "<\\/").replace("<!--", "<\\!--")


def export(out_path: str, records) -> str:
    """Write the report for `records` (list of types.Record) to out_path. Returns out_path."""
    records = list(records)
    data = {
        "records": api.payload(records),
        "knobs": api.knobs_payload(),
        "workloads": api.workloads_payload(records),
        "findings": api.check(records),
    }
    html = _read("index.html")
    html = re.sub(r"<!--INLINE_CSS-->.*?<!--/INLINE_CSS-->", lambda m: "<style>\n" + _read("app.css") + "</style>", html, flags=re.S)
    script = "<script>window.__BENCH__ = " + _js_json(data) + ";</script>\n<script>\n" + _read("app.js").replace("</", "<\\/") + "</script>"
    html = re.sub(r"<!--INLINE_JS-->.*?<!--/INLINE_JS-->", lambda m: script, html, flags=re.S)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return out_path

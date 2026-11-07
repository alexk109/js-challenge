"""Local front end server: a JSON API over the store + runner, and the static page. Stdlib only.

    python -c "from tools.bench.ui.server import serve; serve(8765, None)"

Routes: GET /api/knobs /api/workloads /api/records /api/jobs /api/check /api/trace/<id>, POST /api/run.
All calls into the rest of the bench go through `tools.bench.ui.api`.
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time
import traceback
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import api

STATIC = os.path.join(os.path.dirname(__file__), "static")
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8"}
FILTERS = ("workload", "metric", "level", "backend", "config_hash")


MAX_BODY = 1_000_000
MAX_RUNNING = 2
KEEP_JOBS = 50
LOCAL_HOSTS = ("127.0.0.1", "localhost", "::1")


def _host_ok(value) -> bool:
    """Host / Origin header must name this machine (blocks CSRF and DNS rebinding)."""
    if not value:
        return True
    try:
        host = urlparse(value if "//" in value else "//" + value).hostname
    except ValueError:
        return False
    return host in LOCAL_HOSTS


class State:
    def __init__(self, store_path):
        self.store_path = store_path
        self.jobs: dict[str, dict] = {}
        self.lock = threading.Lock()
        self._check = (None, None)

    def check(self):
        """Cross-check findings, recomputed only when the store file changes."""
        try:
            st = os.stat(api.store_file(self.store_path))
            sig = (st.st_mtime_ns, st.st_size)
        except OSError:
            sig = None
        with self.lock:
            if sig is not None and self._check[0] == sig:
                return self._check[1]
        res = api.check(self.records())
        with self.lock:
            self._check = (sig, res)
        return res

    def running(self) -> int:
        with self.lock:
            return sum(1 for j in self.jobs.values() if j["state"] == "running")

    def records(self, **filters):
        return api.load_records(self.store_path, **filters)

    def start_job(self, req: dict) -> dict:
        job = {"id": uuid.uuid4().hex[:8], "state": "running", "request": req, "events": 0,
               "total": api.count_specs(req), "started": time.time(), "finished": None, "result": None, "error": None}
        with self.lock:
            self.jobs[job["id"]] = job
            for old in [j for j in sorted(self.jobs.values(), key=lambda j: j["started"]) if j["state"] != "running"][:-KEEP_JOBS]:
                del self.jobs[old["id"]]

        def on_event(ev=None):
            ev = ev or {}
            with self.lock:
                kind = ev.get("event")
                if kind == "total":
                    job["total"] = ev.get("n", 0)
                elif kind in ("done", "cache_hit", "skipped"):
                    job["events"] = min(job["events"] + 1, job["total"] or job["events"] + 1)

        def work():
            try:
                res = api.start_run(req, on_event=on_event, store_path=self.store_path)
                with self.lock:
                    job["result"], job["state"] = res, "done"
            except Exception as e:
                with self.lock:
                    job["error"] = "%s: %s" % (type(e).__name__, e)
                    job["trace"] = traceback.format_exc()[-1500:]
                    job["state"] = "error"
            finally:
                with self.lock:
                    job["finished"] = time.time()

        threading.Thread(target=work, daemon=True).start()
        return job


def make_handler(state: State):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, default=str).encode())

        def _local(self) -> bool:
            if not (_host_ok(self.headers.get("Host")) and _host_ok(self.headers.get("Origin"))):
                self._json({"error": "forbidden: this server answers only to 127.0.0.1 / localhost"}, 403)
                return False
            return True

        def do_GET(self):
            if not self._local():
                return
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            p = u.path
            try:
                if p in ("/", "/index.html"):
                    return self._static("index.html")
                if p in ("/app.js", "/app.css"):
                    return self._static(p[1:])
                if p == "/api/knobs":
                    return self._json(api.knobs_payload())
                if p == "/api/workloads":
                    return self._json(api.workloads_payload(state.records()))
                if p == "/api/records":
                    f = {k: q[k] for k in FILTERS if k in q}
                    return self._json(api.payload(state.records(**f)))
                if p == "/api/jobs":
                    with state.lock:
                        return self._json(sorted(state.jobs.values(), key=lambda j: -j["started"]))
                if p == "/api/check":
                    return self._json(state.check())
                if p.startswith("/api/trace/"):
                    return self._trace(p[len("/api/trace/"):])
                self._json({"error": "not found"}, 404)
            except Exception as e:
                self._json({"error": "%s: %s" % (type(e).__name__, e)}, 500)

        def do_POST(self):
            if not self._local():
                return
            if urlparse(self.path).path != "/api/run":
                return self._json({"error": "not found"}, 404)
            if not (self.headers.get("Content-Type") or "").lower().startswith("application/json"):
                return self._json({"error": "Content-Type must be application/json"}, 415)
            try:
                n = int(self.headers.get("Content-Length") or 0)
                if n > MAX_BODY:
                    return self._json({"error": "request body over %d bytes" % MAX_BODY}, 413)
                req = json.loads(self.rfile.read(n) or b"{}")
                bad = api.validate(req)
                if bad:
                    return self._json({"error": bad}, 400)
                if state.running() >= MAX_RUNNING:
                    return self._json({"error": "%d jobs already running; wait for one to finish" % MAX_RUNNING}, 429)
                if not (req.get("configs") or req.get("sweep")):
                    req["configs"] = [{}]
                job = state.start_job(req)
                self._json({"job": job["id"]}, 202)
            except Exception as e:
                self._json({"error": "%s: %s" % (type(e).__name__, e)}, 400)

        def _static(self, name):
            with open(os.path.join(STATIC, name), "rb") as f:
                self._send(200, f.read(), TYPES[os.path.splitext(name)[1]])

        def _trace(self, rid):
            hits = [r for r in state.records() if api.record_id(r).startswith(rid)]
            if len(hits) != 1:
                return self._json({"error": "no record" if not hits else "ambiguous id prefix"}, 404)
            return self._json(api.to_dict(hits[0]))

    return H


class QuietServer(ThreadingHTTPServer):
    """A client that disconnects mid-response (curl | head, a closed tab) is logged as one line, not a traceback."""

    def handle_error(self, request, client_address):
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            print("client %s disconnected (%s)" % (client_address[0], type(exc).__name__), file=sys.stderr)
            return
        super().handle_error(request, client_address)


def make_server(port: int = 8765, store_path=None, host: str = "127.0.0.1") -> ThreadingHTTPServer:
    srv = QuietServer((host, port), make_handler(State(store_path)))
    srv.daemon_threads = True
    return srv


def serve(port: int = 8765, store_path=None) -> None:
    srv = make_server(port, store_path)
    print("Evidence bench at http://127.0.0.1:%d (Ctrl-C to stop)" % srv.server_address[1], flush=True)
    print("  store: %s" % (store_path or api.DEFAULT_STORE), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass

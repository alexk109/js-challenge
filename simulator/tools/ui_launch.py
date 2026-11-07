"""Open the front end: `make ui` / `./bench ui` (bench UI), `make ui-archsim` / `./bench ui --archsim` (archsim UI).

Creates .venv and installs requirements if missing, rebuilds archsim.html if stale, serves on a free port,
opens the default browser (prints the URL if none can be opened). Python 3.10+ standard library only.
"""
import argparse
import functools
import http.server
import os
import pathlib
import shutil
import subprocess
import sys
import webbrowser

ROOT = pathlib.Path(__file__).resolve().parent.parent
VENV_PY = ROOT / ".venv" / "bin" / "python"
SNAPSHOT = ROOT / "tools/bench/snapshots/committed.jsonl"
STORE = ROOT / "tools/bench/results/records.jsonl"
ARCHSIM = ROOT / "tools/archsim"


def ensure_venv() -> None:
    if VENV_PY.exists():
        return
    print("creating .venv", flush=True)
    try:
        subprocess.run([sys.executable, "-m", "venv", str(ROOT / ".venv")], check=True)
        subprocess.run([str(VENV_PY), "-m", "pip", "install", "-q", "-r", str(ROOT / "requirements.txt")], check=True)
    except (subprocess.CalledProcessError, OSError) as e:
        print("dependency install failed (%s); the front end itself needs only the standard library" % e,
              file=sys.stderr)


def ensure_store() -> None:
    if not STORE.exists() and SNAPSHOT.exists():
        STORE.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(SNAPSHOT, STORE)


def archsim_stale() -> bool:
    out = ARCHSIM / "archsim.html"
    if not out.exists():
        return True
    srcs = [p for d in ("ui", "model", "data") for p in (ARCHSIM / d).rglob("*") if p.is_file()]
    srcs.append(ARCHSIM / "build_html.py")
    return any(p.stat().st_mtime > out.stat().st_mtime for p in srcs)


def show(url: str) -> None:
    print(url, flush=True)
    try:
        if not webbrowser.open(url):
            print("no browser opened; open the URL above", flush=True)
    except Exception:
        print("no browser opened; open the URL above", flush=True)


def run(srv, url: str, no_open: bool) -> None:
    if no_open:
        print(url, flush=True)
    else:
        show(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ui")
    ap.add_argument("--archsim", action="store_true", help="open the archsim UI instead of the bench UI")
    ap.add_argument("--port", type=int, default=0, help="default: a free port")
    ap.add_argument("--no-open", action="store_true", help="print the URL only")
    args = ap.parse_args(argv)
    ensure_venv()
    if VENV_PY.exists() and pathlib.Path(sys.executable).resolve() != VENV_PY.resolve() and not os.environ.get("UI_REEXEC"):
        os.environ["UI_REEXEC"] = "1"
        os.execv(str(VENV_PY), [str(VENV_PY), str(pathlib.Path(__file__).resolve())] + (argv if argv is not None else sys.argv[1:]))
    sys.path.insert(0, str(ROOT))
    if args.archsim:
        if archsim_stale():
            from tools.archsim import build_html
            (ARCHSIM / "archsim.html").write_text(build_html.build())
        handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(ARCHSIM))
        handler.log_message = lambda *a, **k: None
        srv = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), handler)
        url = "http://127.0.0.1:%d/archsim.html" % srv.server_address[1]
    else:
        ensure_store()
        from tools.bench.ui.server import make_server
        srv = make_server(args.port, None)
        url = "http://127.0.0.1:%d/" % srv.server_address[1]
    run(srv, url, args.no_open)
    return 0


if __name__ == "__main__":
    sys.exit(main())

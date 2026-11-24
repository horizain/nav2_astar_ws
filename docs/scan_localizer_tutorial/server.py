#!/usr/bin/env python3
"""Read-only Apple-styled tutorial static server for scan_localizer.

Serves only whitelisted directories under the workspace root and exposes
two lightweight local API endpoints for WSL -> Windows VS Code integration:
1) GET  /api/env         -> WSL distro name, workspace root, code CLI presence
2) POST /api/open-vscode -> Launch Windows VS Code directly via WSL `code` CLI
"""

import argparse
import functools
import http.server
import json
import mimetypes
import os
import posixpath
import shutil
import socketserver
import subprocess
import urllib.parse
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = (WORKSPACE_ROOT / "src" / "scan_localizer").resolve()
ALLOWED_PREFIXES = (
    "/docs/",
    "/slam-mpc-academy/assets/vendor/",
    "/src/scan_localizer/",
)

EXTRA_MIMES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".mjs": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".woff": "font/woff",
    ".woff2": "font/woff2",
    ".ttf": "font/ttf",
    ".hpp": "text/plain; charset=utf-8",
    ".cpp": "text/plain; charset=utf-8",
    ".h": "text/plain; charset=utf-8",
    ".c": "text/plain; charset=utf-8",
    ".py": "text/plain; charset=utf-8",
    ".yaml": "text/plain; charset=utf-8",
    ".yml": "text/plain; charset=utf-8",
    ".xml": "text/plain; charset=utf-8",
    ".txt": "text/plain; charset=utf-8",
    ".md": "text/plain; charset=utf-8",
}


class TutorialHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def send_json(self, status: int, payload: dict):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def guess_type(self, path):
        ext = os.path.splitext(path)[1].lower()
        if ext in EXTRA_MIMES:
            return EXTRA_MIMES[ext]
        mime, _ = mimetypes.guess_type(path)
        return mime or "application/octet-stream"

    def do_GET(self):
        parsed = urllib.parse.urlsplit(self.path)
        clean_path = posixpath.normpath(urllib.parse.unquote(parsed.path))
        if clean_path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        if clean_path == "/api/env":
            self.send_json(200, {
                "wsl_distro": os.environ.get("WSL_DISTRO_NAME", "Ubuntu-24.04"),
                "workspace_root": str(WORKSPACE_ROOT),
                "has_code_cli": bool(shutil.which("code")),
            })
            return
        if clean_path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", "/docs/scan_localizer_tutorial/index.html")
            self.end_headers()
            return
        if not any(clean_path.startswith(prefix) for prefix in ALLOWED_PREFIXES):
            self.send_error(403, "Forbidden: outside whitelisted tutorial paths")
            return
        super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlsplit(self.path)
        clean_path = posixpath.normpath(urllib.parse.unquote(parsed.path))
        if clean_path != "/api/open-vscode":
            self.send_json(404, {"ok": False, "error": "Unknown API endpoint"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_json(400, {"ok": False, "error": "Invalid Content-Length"})
            return
        if length <= 0 or length > 8192:
            self.send_json(400, {"ok": False, "error": "Invalid payload size"})
            return
        try:
            data = json.loads(self.rfile.read(length).decode("utf-8"))
        except Exception:
            self.send_json(400, {"ok": False, "error": "Malformed JSON"})
            return

        rel = str(data.get("file", "")).lstrip("/")
        try:
            line = int(data.get("line", 1))
        except (TypeError, ValueError):
            line = 1
        if line < 1:
            line = 1

        target = (PKG_ROOT / rel).resolve()
        try:
            target.relative_to(PKG_ROOT)
        except ValueError:
            self.send_json(403, {"ok": False, "error": "Forbidden path outside src/scan_localizer"})
            return
        if not target.is_file():
            self.send_json(404, {"ok": False, "error": f"File not found: {rel}"})
            return

        code_bin = shutil.which("code")
        if not code_bin:
            self.send_json(503, {"ok": False, "error": "VS Code CLI 'code' not found in PATH"})
            return

        loc = f"{target}:{line}:1"
        try:
            subprocess.Popen(
                [code_bin, "--reuse-window", "--goto", loc],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as exc:
            self.send_json(500, {"ok": False, "error": str(exc)})
            return

        self.send_json(200, {"ok": True, "target": loc})

    def log_message(self, format, *args):  # noqa: A002
        return


class ThreadingServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(description="Serve scan_localizer tutorial")
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", "8090")))
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args()

    handler = functools.partial(TutorialHandler, directory=str(WORKSPACE_ROOT))
    with ThreadingServer((args.host, args.port), handler) as httpd:
        url = f"http://{args.host}:{httpd.server_address[1]}/docs/scan_localizer_tutorial/index.html"
        print(f"[scan_localizer tutorial] serving {WORKSPACE_ROOT}")
        print(f"Open: {url}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()

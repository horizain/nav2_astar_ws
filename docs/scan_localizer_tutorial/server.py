#!/usr/bin/env python3
"""Read-only Apple-styled tutorial static server for scan_localizer.

Serves only whitelisted directories under the workspace root so that:
1) Tutorial pages share exact relative URLs between file:// and http://;
2) The live code drawer can fetch real source files under src/scan_localizer/.
"""

import argparse
import functools
import http.server
import mimetypes
import os
import posixpath
import socketserver
import urllib.parse
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
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
        if clean_path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", "/docs/scan_localizer_tutorial/index.html")
            self.end_headers()
            return
        if not any(clean_path.startswith(prefix) for prefix in ALLOWED_PREFIXES):
            self.send_error(403, "Forbidden: outside whitelisted tutorial paths")
            return
        super().do_GET()

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

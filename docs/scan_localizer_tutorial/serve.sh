#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-${PORT:-8090}}"
exec python3 "${SCRIPT_DIR}/server.py" --port "${PORT}"

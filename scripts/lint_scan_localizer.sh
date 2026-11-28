#!/usr/bin/env bash
# Copyright 2026 ruijianding
# Licensed under the Apache License, Version 2.0
#
# 一键本地预检脚本：推送 PR 前提前拦截 ROS 2 Jazzy lint / 参数漂移 / 文档断链问题。
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[1/5] Python blackbox inspector unit tests"
python3 -B src/scan_localizer/test/test_inspect_blackbox.py -v

echo "[2/5] Tutorial code manifest, 66-param parity & doc links"
python3 docs/scan_localizer_tutorial/tests/sync_code_manifest.py --check-params
python3 docs/scan_localizer_tutorial/tests/check_code_refs.py
python3 docs/tools/check_tutorial_links.py

echo "[3/5] Python flake8 (ROS 2 max-line-length=99)"
if command -v flake8 >/dev/null 2>&1; then
  flake8 --max-line-length=99 \
    src/scan_localizer/tools/ \
    src/scan_localizer/test/test_inspect_blackbox.py
else
  echo "  (skip: flake8 not installed; pip install flake8)"
fi

echo "[4/5] C++ line length <= 100 & blank-line-before-brace check"
python3 - << 'EOF'
import glob, sys
patterns = [
    "src/scan_localizer/include/scan_localizer/*.hpp",
    "src/scan_localizer/src/*.cpp",
    "src/scan_localizer/src/*.hpp",
    "src/scan_localizer/test/*.cpp",
]
issues = 0
for pat in patterns:
    for p in sorted(glob.glob(pat)):
        lines = open(p, encoding="utf-8").readlines()
        for i, line in enumerate(lines, 1):
            s = line.rstrip("\n")
            if len(s) > 100:
                print(f"LONG({len(s)}): {p}:{i}")
                issues += 1
            if i > 1 and s.strip() == "}" and lines[i - 2].strip() == "":
                print(f"BLANK_BEFORE_BRACE: {p}:{i}")
                issues += 1
if issues:
    sys.exit(1)
print("  PASS: C++ line-length and brace-spacing checks clean.")
EOF

echo "[5/5] Optional ament_uncrustify / ament_cpplint (if ROS 2 is sourced)"
if command -v ament_uncrustify >/dev/null 2>&1; then
  ament_uncrustify src/scan_localizer
fi
if command -v ament_cpplint >/dev/null 2>&1; then
  ament_cpplint src/scan_localizer
fi

echo "ALL LOCAL PRE-PUSH CHECKS PASSED."

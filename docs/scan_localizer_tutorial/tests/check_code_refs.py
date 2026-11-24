#!/usr/bin/env python3
"""Verify that:
1) Every file in src/scan_localizer (all 33 files) is indexed in js/code-manifest.js;
2) Every data-file / data-lines / manifest symbol line range points to an existing file
   and valid in-bounds 1-based line numbers.
"""

import pathlib
import re
import sys

TUTORIAL_DIR = pathlib.Path(__file__).resolve().parents[1]
REPO_ROOT = TUTORIAL_DIR.parents[1]
PKG_ROOT = REPO_ROOT / "src" / "scan_localizer"


def line_count(rel_path: str) -> int:
    target = PKG_ROOT / rel_path
    if not target.is_file():
        return -1
    return len(target.read_text(encoding="utf-8", errors="ignore").splitlines())


def check_range(src_label: str, rel_path: str, range_str: str, errors: list[str]):
    total = line_count(rel_path)
    if total < 0:
        errors.append(f"{src_label}: missing file src/scan_localizer/{rel_path}")
        return
    parts = [int(x) for x in re.findall(r"\d+", range_str)]
    if not parts:
        errors.append(f"{src_label}: empty line range '{range_str}' for {rel_path}")
        return
    s = parts[0]
    e = parts[1] if len(parts) > 1 else s
    if s < 1 or e < s or e > total:
        errors.append(
            f"{src_label}: out-of-bounds line range L{s}-L{e} for {rel_path} (total={total})"
        )


def main() -> int:
    errors: list[str] = []
    refs_checked = 0

    # 1. Check all files under src/scan_localizer are present in code-manifest.js
    manifest_text = (TUTORIAL_DIR / "js" / "code-manifest.js").read_text(encoding="utf-8")
    manifest_paths = set(re.findall(r'path:\s*"([^"]+)"', manifest_text))
    actual_paths = {p.relative_to(PKG_ROOT).as_posix() for p in PKG_ROOT.rglob("*") if p.is_file()}

    missing_in_manifest = sorted(actual_paths - manifest_paths)
    extra_in_manifest = sorted(manifest_paths - actual_paths)
    if missing_in_manifest:
        errors.append(f"code-manifest.js missing files: {missing_in_manifest}")
    if extra_in_manifest:
        errors.append(f"code-manifest.js has non-existent files: {extra_in_manifest}")

    # Validate file line counts & symbol ranges in manifest
    file_blocks = re.finditer(
        r'path:\s*"([^"]+)",\s*group:\s*"[^"]+",\s*lines:\s*(\d+),.*?symbols:\s*\[(.*?)\]',
        manifest_text,
        re.S,
    )
    for m in file_blocks:
        rel, declared_lines, syms_block = m.group(1), int(m.group(2)), m.group(3)
        actual_lines = line_count(rel)
        if actual_lines != declared_lines:
            errors.append(
                f"code-manifest.js: {rel} declares lines={declared_lines}, actual={actual_lines}"
            )
        for rm in re.finditer(r'range:\s*"([^"]+)"', syms_block):
            refs_checked += 1
            check_range(f"manifest:{rel}", rel, rm.group(1), errors)

    # 2. Check all file + lines references in walkthrough-data.js
    wt_js = (TUTORIAL_DIR / "js" / "walkthrough-data.js").read_text(encoding="utf-8")
    for m in re.finditer(r'file:\s*"([^"]+)",\s*lines:\s*"([^"]+)"', wt_js):
        refs_checked += 1
        check_range("walkthrough-data.js", m.group(1), m.group(2), errors)
    for m in re.finditer(r'files:\s*\[([^\]]+)\]', wt_js):
        for fm in re.finditer(r'"([^"]+)"', m.group(1)):
            refs_checked += 1
            check_range("walkthrough-data.js:BUILD_STEPS", fm.group(1), "1-1", errors)

    # 3. Check all data-file + data-lines references across HTML pages
    for html in sorted(list(TUTORIAL_DIR.glob("*.html")) + list((TUTORIAL_DIR / "chapters").glob("*.html"))):
        text = html.read_text(encoding="utf-8")
        for m in re.finditer(r'data-file="([^"]+)"[^>]*data-lines="([^"]+)"', text):
            if "${" in m.group(1) or "${" in m.group(2):
                continue
            refs_checked += 1
            check_range(html.name, m.group(1), m.group(2), errors)
        # Also catch JS object entries like file: "...", lines: "..." in ch10.html
        for m in re.finditer(r'file:\s*"([^"]+)",\s*lines:\s*"([^"]+)"', text):
            if "${" in m.group(1) or "${" in m.group(2):
                continue
            refs_checked += 1
            check_range(html.name, m.group(1), m.group(2), errors)

    if errors:
        for e in errors:
            print("ERROR:", e)
        return 1

    print(
        f"PASS: 33/33 files covered in code-manifest.js; "
        f"{refs_checked} symbol/source line references verified cleanly."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

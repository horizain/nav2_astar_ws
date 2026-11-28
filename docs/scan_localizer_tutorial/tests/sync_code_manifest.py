#!/usr/bin/env python3
"""Step 1 helper: Automatically sync declared file line counts in
docs/scan_localizer_tutorial/js/code-manifest.js and check parameter parity
between config/localizer.yaml and LocalizerNode parameter declarations.
"""
import pathlib
import re
import sys

TUTORIAL_DIR = pathlib.Path(__file__).resolve().parents[1]
REPO_ROOT = TUTORIAL_DIR.parents[1]
PKG_ROOT = REPO_ROOT / "src" / "scan_localizer"
MANIFEST_PATH = TUTORIAL_DIR / "js" / "code-manifest.js"


def sync_manifest_line_counts() -> int:
    txt = MANIFEST_PATH.read_text(encoding="utf-8")
    updated = 0

    def repl(match: re.Match) -> str:
        nonlocal updated
        prefix, rel, mid, old_lines = (
            match.group(1),
            match.group(2),
            match.group(3),
            int(match.group(4)),
        )
        target = PKG_ROOT / rel
        if not target.is_file():
            return match.group(0)
        actual = len(target.read_text(encoding="utf-8", errors="ignore").splitlines())
        if actual != old_lines:
            print(f"sync_code_manifest: {rel}: {old_lines} -> {actual}")
            updated += 1
        return f"{prefix}{rel}{mid}{actual}"

    new_txt = re.sub(
        r'(path:\s*")([^"]+)(",\s*group:\s*"[^"]+",\s*lines:\s*)(\d+)',
        repl,
        txt,
    )
    if updated > 0:
        MANIFEST_PATH.write_text(new_txt, encoding="utf-8")
    else:
        print("sync_code_manifest: all line counts already up to date.")
    return updated


def check_yaml_node_param_parity() -> bool:
    yaml_path = PKG_ROOT / "config" / "localizer.yaml"
    params_srcs = [
        PKG_ROOT / "src" / "localizer_node.cpp",
        PKG_ROOT / "src" / "localizer_node_params.cpp",
    ]
    combined_cpp = "\n".join(
        p.read_text(encoding="utf-8") for p in params_srcs if p.is_file()
    )
    declared = set(
        re.findall(r'declare_parameter<[^>]+>\(\s*"([^"]+)"', combined_cpp)
    )
    yaml_keys = set()
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        m = re.match(r"^\s{4}([a-zA-Z0-9_]+)\s*:", line)
        if m:
            yaml_keys.add(m.group(1))

    missing_in_yaml = sorted(declared - yaml_keys)
    extra_in_yaml = sorted(yaml_keys - declared)
    if missing_in_yaml or extra_in_yaml:
        if missing_in_yaml:
            print("ERROR: declared in node but missing in localizer.yaml:", missing_in_yaml)
        if extra_in_yaml:
            print("ERROR: listed in localizer.yaml but not declared in node:", extra_in_yaml)
        return False
    print(f"PASS: localizer.yaml <-> LocalizerNode parameter parity ({len(declared)} params aligned).")
    return True


def main() -> int:
    sync_manifest_line_counts()
    if "--check-params" in sys.argv:
        if not check_yaml_node_param_parity():
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

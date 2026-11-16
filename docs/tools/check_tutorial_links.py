#!/usr/bin/env python3
"""Check local Markdown file links; remote URLs and heading anchors are not fetched."""
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit


def check(root):
    files = [root / "README.md"]
    files += list((root / "docs").rglob("*.md"))
    files += list((root / "src").rglob("README.md"))
    files += list((root / "src/mini_slam/docs").glob("*.md"))
    missing, checked = [], 0
    for file in sorted(set(files)):
        text = file.read_text(encoding="utf-8")
        fence = re.escape(chr(96) * 3)
        text = re.sub(fence + r"[\s\S]*?" + fence + r"|~~~[\s\S]*?~~~", "", text)
        quote = chr(96)
        text = re.sub(quote + "[^" + quote + "\n]*" + quote, "", text)
        for destination in re.findall(r"!?\[[^\]\n]*\]\(([^)\n]+)\)", text):
            destination = destination.strip()
            if destination.startswith("<") and destination.endswith(">"):
                destination = destination[1:-1]
            parsed = urlsplit(destination)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            checked += 1
            target = file.parent / unquote(parsed.path)
            if not target.is_file():
                missing.append((str(file.relative_to(root)), destination))
    return checked, missing


def main():
    root = Path(__file__).resolve().parents[2]
    count, missing = check(root)
    for file, target in missing:
        print(f"{file}: missing local file: {target}")
    print(f"Checked {count} local file links; missing {len(missing)}")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())

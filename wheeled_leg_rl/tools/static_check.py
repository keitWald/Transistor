"""Parse all project Python files without importing optional dependencies."""

from __future__ import annotations

import ast
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    failures = []
    files = sorted(PROJECT_ROOT.rglob("*.py"))
    for path in files:
        try:
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (SyntaxError, UnicodeDecodeError) as exc:
            failures.append((path, exc))

    if failures:
        for path, error in failures:
            print(f"FAIL {path.relative_to(PROJECT_ROOT)}: {error}")
        return 1

    print(f"Static syntax check passed: {len(files)} Python files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

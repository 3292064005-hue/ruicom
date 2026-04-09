#!/usr/bin/env python3
"""Repository hygiene validator.

Fails fast when transient cache/build artifacts are present inside one project
root. This keeps release packaging honest and makes the "clean delivery" claim
machine-verifiable instead of relying on manual zip inspection.
"""

from __future__ import annotations

import argparse
from pathlib import Path

DEFAULT_ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_DIR_NAMES = {
    '__pycache__',
    '.pytest_cache',
    '.mypy_cache',
    '.ruff_cache',
}
FORBIDDEN_FILE_SUFFIXES = {'.pyc', '.pyo'}
FORBIDDEN_FILE_NAMES = {'.DS_Store'}
SKIP_DIR_NAMES = {'.git', '.idea', '.vscode'}


def scan_repository(root: Path) -> list[str]:
    violations: list[str] = []
    for path in sorted(root.rglob('*')):
        relative = path.relative_to(root)
        if any(part in SKIP_DIR_NAMES for part in relative.parts):
            continue
        if path.is_dir() and path.name in FORBIDDEN_DIR_NAMES:
            violations.append(str(relative))
            continue
        if path.is_file():
            if path.name in FORBIDDEN_FILE_NAMES or path.suffix in FORBIDDEN_FILE_SUFFIXES:
                violations.append(str(relative))
    return violations


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Validate repository hygiene for one project tree.')
    parser.add_argument('root', nargs='?', default=str(DEFAULT_ROOT), help='project root to scan (default: repository root)')
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    violations = scan_repository(root)
    if violations:
        print('[hygiene] repository is not clean; remove transient artifacts before packaging:')
        for item in violations:
            print(f' - {item}')
        return 1
    print('[hygiene] repository tree is clean')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

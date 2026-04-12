#!/usr/bin/env python3
"""Validate third-party source intake governance."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / 'UPSTREAM_SOURCES.md'
NOTICES = ROOT / 'THIRD_PARTY_NOTICES.md'
THIRD_PARTY_DIR_NAMES = {'third_party', 'external', 'vendor'}
SKIP_DIR_NAMES = {'.git', 'build', 'devel', 'install', 'log'}


def _third_party_source_dirs(root: Path) -> list[str]:
    paths = []
    for path in root.rglob('*'):
        relative = path.relative_to(root)
        if any(part in SKIP_DIR_NAMES for part in relative.parts):
            continue
        if path.is_dir() and path.name in THIRD_PARTY_DIR_NAMES:
            paths.append(str(relative))
    return sorted(paths)


def validate(root: Path = ROOT) -> list[str]:
    errors = []
    if not UPSTREAM.exists():
        errors.append('UPSTREAM_SOURCES.md is required')
    if not NOTICES.exists():
        errors.append('THIRD_PARTY_NOTICES.md is required')
    if errors:
        return errors
    upstream = UPSTREAM.read_text(encoding='utf-8')
    notices = NOTICES.read_text(encoding='utf-8')
    for token in ('来源版本', 'commit/tag', '许可证', '局部改动说明'):
        if token not in upstream:
            errors.append('UPSTREAM_SOURCES.md must document future source intake field: {}'.format(token))
    for token in ('保留原始版权声明', 'UPSTREAM_SOURCES.md', '许可证文本'):
        if token not in notices:
            errors.append('THIRD_PARTY_NOTICES.md must document license hygiene rule: {}'.format(token))
    source_dirs = _third_party_source_dirs(root)
    if source_dirs and '未直接复制第三方源码' in upstream:
        errors.append('third-party source directories exist but UPSTREAM_SOURCES.md still says no source was copied: {}'.format(', '.join(source_dirs)))
    return errors


def main() -> int:
    errors = validate()
    if errors:
        print('[third-party] governance validation failed:')
        for error in errors:
            print(' - {}'.format(error))
        return 1
    print('[third-party] validated third-party governance')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

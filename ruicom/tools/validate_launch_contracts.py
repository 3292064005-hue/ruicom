#!/usr/bin/env python3
"""Static launch/test contract validator.

This validator is ROS-free: it parses launch/test XML, verifies that in-repo
includes resolve, that ruikang package nodes point to real installed scripts,
and that package-relative config/manifests referenced from launch args exist.
It narrows the gap between pure unit tests and full Noetic/rostest execution.
"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / 'launch'
TEST_DIR = ROOT / 'tests'
SCRIPTS_DIR = ROOT / 'scripts'
PACKAGE_NAME = 'ruikang_recon_baseline'
PACKAGE_FIND_PATTERN = re.compile(r"\$\(find\s+ruikang_recon_baseline\)/([^\s\"']+)")
PACKAGE_RELATIVE_PATH_PATTERN = re.compile(r"config/[^\s\"']+|launch/[^\s\"']+|tests/[^\s\"']+")


def _iter_xml_files() -> list[Path]:
    launch_files = sorted(LAUNCH_DIR.glob('*.launch'))
    test_files = sorted(TEST_DIR.glob('*.test'))
    return [*launch_files, *test_files]


def _extract_package_paths(text: str) -> set[str]:
    matches = set(PACKAGE_FIND_PATTERN.findall(text))
    matches.update(PACKAGE_RELATIVE_PATH_PATTERN.findall(text))
    return {item for item in matches if item and not item.startswith('msg/')}  # message names are not files


def _validate_xml_file(path: Path) -> list[str]:
    errors: list[str] = []
    text = path.read_text(encoding='utf-8')
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        return [f'{path.relative_to(ROOT)}: XML parse error: {exc}']

    for relative in sorted(_extract_package_paths(text)):
        resolved = ROOT / relative
        if relative.startswith('config/') or relative.startswith('launch/') or relative.startswith('tests/'):
            if not resolved.exists():
                errors.append(f'{path.relative_to(ROOT)}: missing package-relative path {relative}')

    for node in root.iter('node'):
        pkg = str(node.attrib.get('pkg', '')).strip()
        node_type = str(node.attrib.get('type', '')).strip()
        if pkg == PACKAGE_NAME and node_type:
            resolved = SCRIPTS_DIR / node_type
            if not resolved.exists():
                errors.append(f'{path.relative_to(ROOT)}: node type {node_type} not found under scripts/')
            elif not (resolved.stat().st_mode & 0o111):
                errors.append(f'{path.relative_to(ROOT)}: node type {node_type} is not executable')
    return errors


def main() -> int:
    errors: list[str] = []
    files = _iter_xml_files()
    if not files:
        print('[launch-contracts] no launch/test XML files found')
        return 1
    for path in files:
        errors.extend(_validate_xml_file(path))
    if errors:
        print('[launch-contracts] validation failed:')
        for item in errors:
            print(f' - {item}')
        return 1
    print(f'[launch-contracts] validated {len(files)} launch/test files')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

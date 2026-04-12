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


def _collect_declared_args(root: ET.Element) -> set[str]:
    declared: set[str] = set()

    def visit(element: ET.Element, *, in_include: bool = False) -> None:
        current_in_include = in_include or element.tag == 'include'
        if element.tag == 'arg' and not in_include:
            name = str(element.attrib.get('name', '')).strip()
            if name:
                declared.add(name)
        for child in element:
            visit(child, in_include=current_in_include)

    visit(root)
    return declared


def _resolve_package_launch_path(file_attr: str) -> Path | None:
    match = PACKAGE_FIND_PATTERN.search(file_attr)
    if not match:
        return None
    resolved = ROOT / match.group(1)
    return resolved if resolved.exists() else None


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

    declared_args_cache: dict[Path, set[str]] = {}
    for include in root.iter('include'):
        target = _resolve_package_launch_path(str(include.attrib.get('file', '')).strip())
        if target is None:
            continue
        if target not in declared_args_cache:
            try:
                include_root = ET.fromstring(target.read_text(encoding='utf-8'))
            except ET.ParseError as exc:
                errors.append(f'{path.relative_to(ROOT)}: included file {target.relative_to(ROOT)} has XML parse error: {exc}')
                continue
            declared_args_cache[target] = _collect_declared_args(include_root)
        declared_args = declared_args_cache[target]
        for arg in include.findall('arg'):
            name = str(arg.attrib.get('name', '')).strip()
            if name and name not in declared_args:
                errors.append(
                    f'{path.relative_to(ROOT)}: include {target.relative_to(ROOT)} passes undeclared arg {name}'
                )

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

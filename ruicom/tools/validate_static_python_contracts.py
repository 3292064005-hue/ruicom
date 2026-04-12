#!/usr/bin/env python3
"""Static Python contract checks that catch deploy-time NameError regressions.

The repository intentionally avoids requiring a full linter in Noetic Docker
smokes. This script keeps a small set of runtime-critical import/configuration
contracts executable in both regular CI and ROS Noetic verification.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path
from typing import Iterable, Mapping

ROOT = Path(__file__).resolve().parents[1]

SOURCE_ROOTS = ('src', 'tests', 'tools', 'scripts')

REQUIRED_IMPORTS = {
    'src/ruikang_recon_baseline/vision_node.py': {
        'detector_manifest_satisfies_scope': '.manifest_utils',
    },
    'src/ruikang_recon_baseline/mission_node.py': {
        'evaluate_navigation_runtime_probe': '.runtime_probes',
    },
    'src/ruikang_recon_baseline/platform_bridge_node.py': {
        'evaluate_platform_runtime_probe': '.runtime_probes',
    },
}

REQUIRED_CONFIG_KEYS = {
    'src/ruikang_recon_baseline/recorder_node.py': {
        ('MissionRecorderNode', '_read_config'): {'runtime_grade'},
    },
}


def _python_files(root: Path) -> Iterable[Path]:
    for source_root in SOURCE_ROOTS:
        base = root / source_root
        if not base.exists():
            continue
        yield from sorted(base.rglob('*.py'))


def _imported_symbols(tree: ast.AST) -> Mapping[str, str]:
    imported = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        module = str(node.module or '')
        if node.level:
            module = '{}{}'.format('.' * int(node.level), module)
        for alias in node.names:
            if alias.name == '*':
                continue
            imported[alias.asname or alias.name] = module
    return imported


def _find_method(tree: ast.AST, class_name: str, method_name: str) -> ast.FunctionDef | None:
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef) or node.name != class_name:
            continue
        for item in node.body:
            if isinstance(item, ast.FunctionDef) and item.name == method_name:
                return item
    return None


def _initial_config_keys(method: ast.FunctionDef) -> set[str]:
    for node in ast.walk(method):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(target, ast.Name) and target.id == 'config' for target in node.targets):
            continue
        if not isinstance(node.value, ast.Dict):
            continue
        keys = set()
        for key in node.value.keys:
            if isinstance(key, ast.Constant) and isinstance(key.value, str):
                keys.add(key.value)
        return keys
    return set()


def validate(root: Path) -> list[str]:
    errors = []
    parsed = {}
    for path in _python_files(root):
        relative = path.relative_to(root).as_posix()
        try:
            parsed[relative] = ast.parse(path.read_text(encoding='utf-8'), filename=relative)
        except SyntaxError as exc:
            errors.append('{}:{}:{} syntax error: {}'.format(relative, exc.lineno, exc.offset, exc.msg))

    for relative, symbol_map in REQUIRED_IMPORTS.items():
        tree = parsed.get(relative)
        if tree is None:
            errors.append('{} missing from static import validation set'.format(relative))
            continue
        imported = _imported_symbols(tree)
        for symbol, module in symbol_map.items():
            if imported.get(symbol) != module:
                errors.append('{} must import {} from {}, found {}'.format(relative, symbol, module, imported.get(symbol, '<missing>')))

    for relative, method_map in REQUIRED_CONFIG_KEYS.items():
        tree = parsed.get(relative)
        if tree is None:
            errors.append('{} missing from static config validation set'.format(relative))
            continue
        for (class_name, method_name), required_keys in method_map.items():
            method = _find_method(tree, class_name, method_name)
            if method is None:
                errors.append('{} missing {}.{}'.format(relative, class_name, method_name))
                continue
            keys = _initial_config_keys(method)
            missing = sorted(set(required_keys) - keys)
            if missing:
                errors.append('{} {}.{} initial config missing keys: {}'.format(relative, class_name, method_name, ', '.join(missing)))
    return errors


def main() -> int:
    errors = validate(ROOT)
    if errors:
        print('[static-python] validation failed:')
        for error in errors:
            print(' - {}'.format(error))
        return 1
    print('[static-python] validated static python contracts')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

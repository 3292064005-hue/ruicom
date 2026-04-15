#!/usr/bin/env python3
"""Validate one managed vendor bundle against one workspace root."""
from __future__ import annotations
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from ruikang_recon_baseline.vendor_bundle_manifest import load_vendor_bundle_manifest, resolve_vendor_bundle_startup_steps  # noqa: E402

MANAGED_MARKER = '.ruikang_managed_vendor_workspace'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Validate a managed vendor bundle against one workspace root')
    parser.add_argument('--workspace-root', default='', help='Vendor workspace root')
    parser.add_argument('--bundle-manifest', default=str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'))
    parser.add_argument('--require-external-workspace', action='store_true', help='Reject repository-local fixture workspaces')
    parser.add_argument('--allow-repo-workspace', action='store_true', help='Allow repository-local managed workspace roots for local smoke validation')
    return parser.parse_args()


def _ensure_setup_script(workspace_path: Path) -> None:
    candidates = [workspace_path / 'devel' / 'setup.bash', workspace_path / 'install' / 'setup.bash']
    if not any(item.exists() for item in candidates):
        raise SystemExit(f'workspace root is missing devel/setup.bash or install/setup.bash: {workspace_path}')


def _validate_launch_wrapper(absolute_path: Path) -> None:
    try:
        root = ET.parse(absolute_path).getroot()
    except Exception as exc:
        raise SystemExit(f'invalid launch xml for {absolute_path}: {exc}')
    if root.tag != 'launch':
        raise SystemExit(f'launch file must have <launch> root: {absolute_path}')
    serialized = absolute_path.read_text(encoding='utf-8')
    if '$(find ruikang_recon_baseline)/launch/managed_vendor_' not in serialized and 'ruikang/managed_vendor_bundle' not in serialized:
        raise SystemExit(f'launch wrapper must reference managed vendor integration assets: {absolute_path}')


def _validate_vendor_entrypoint(workspace_path: Path, relative_path: str, absolute_path: Path) -> None:
    if not absolute_path.exists():
        raise SystemExit(f'vendor entrypoint missing: {relative_path} -> {absolute_path}')
    rel = Path(relative_path)
    parts = rel.parts
    if len(parts) < 2:
        raise SystemExit(f'vendor entrypoint must be pkg-relative: {relative_path}')
    package_dir = workspace_path / 'src' / parts[0]
    if not package_dir.exists():
        raise SystemExit(f'vendor package directory missing: {package_dir}')
    package_xml = package_dir / 'package.xml'
    cmakelists = package_dir / 'CMakeLists.txt'
    if not package_xml.exists():
        raise SystemExit(f'vendor package missing package.xml: {package_xml}')
    if not cmakelists.exists():
        raise SystemExit(f'vendor package missing CMakeLists.txt: {cmakelists}')
    if absolute_path.suffix == '.launch':
        _validate_launch_wrapper(absolute_path)
    elif absolute_path.suffix == '.py':
        if not absolute_path.stat().st_mode & 0o111:
            raise SystemExit(f'vendor script must be executable: {absolute_path}')
        text = absolute_path.read_text(encoding='utf-8')
        first_line = text.splitlines()[0] if text else ''
        if not first_line.startswith('#!'):
            raise SystemExit(f'vendor script missing shebang: {absolute_path}')
        if 'managed vendor workspace' not in text.lower():
            raise SystemExit(f'vendor script must identify managed workspace contract: {absolute_path}')


def main() -> int:
    args = parse_args()
    manifest = load_vendor_bundle_manifest(args.bundle_manifest)
    if manifest is None:
        raise SystemExit('bundle manifest could not be loaded')
    for name, rel in manifest.managed_entrypoints.items():
        target = (ROOT / rel).resolve()
        if not target.exists():
            raise SystemExit(f'managed entrypoint missing: {name} -> {target}')
        print(f'{name}: managed -> {target}')
    workspace_root = str(args.workspace_root).strip()
    if not workspace_root:
        raise SystemExit('workspace root must be provided')
    workspace_path = Path(workspace_root).expanduser().resolve()
    if not workspace_path.exists():
        raise SystemExit(f'workspace root does not exist: {workspace_path}')
    repo_root = ROOT.resolve()
    is_repo_workspace = repo_root == workspace_path or repo_root in workspace_path.parents
    if args.require_external_workspace and is_repo_workspace and not args.allow_repo_workspace:
        raise SystemExit(f'workspace root must be external to repository when --require-external-workspace is set: {workspace_path}')
    if workspace_path.name != manifest.vendor_workspace_name:
        raise SystemExit(f'workspace root basename must be {manifest.vendor_workspace_name}: {workspace_path}')
    _ensure_setup_script(workspace_path)
    if not (workspace_path / MANAGED_MARKER).exists() and not args.allow_repo_workspace:
        raise SystemExit(f'managed workspace marker missing: {workspace_path / MANAGED_MARKER}')
    steps = resolve_vendor_bundle_startup_steps(manifest, workspace_root=str(workspace_path), repo_root=ROOT)
    for name, rel in manifest.vendor_entrypoints.items():
        abs_path = (workspace_path / 'src' / rel).resolve()
        _validate_vendor_entrypoint(workspace_path, rel, abs_path)
        print(f'{name}: vendor -> {abs_path}')
    for step in steps:
        print(f'{step.name}: {step.kind} -> {step.absolute_path}')
    print('managed vendor bundle validation passed')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())

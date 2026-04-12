"""Vendor legacy-workspace preflight helpers.

These helpers make the external MOWEN legacy workspace an explicit managed
bundle rather than an undocumented out-of-repo assumption. The preflight is
pure-Python so contract tests can validate the decision logic without ROS.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

from .domain_models import ConfigurationError
from .vendor_bundle_manifest import (
    iter_vendor_workspace_entrypoint_candidates,
    load_vendor_bundle_manifest,
    validate_vendor_bundle_lock,
)


def _normalize_mode(value: object) -> str:
    mode = str(value or '').strip().lower()
    return mode if mode in ('off', 'advisory', 'required') else 'off'


def _resolve_workspace_root(config: Mapping[str, object]) -> tuple[str, str]:
    configured = str(config.get('vendor_workspace_root', '')).strip()
    if configured:
        return configured, 'config'
    env_name = str(config.get('vendor_workspace_root_env', '')).strip()
    if env_name:
        env_value = os.environ.get(env_name, '').strip()
        if env_value:
            return env_value, 'env'
    return '', ''


def _workspace_markers(workspace_root: str) -> dict:
    root_path = Path(workspace_root).expanduser()
    return {
        'src_dir_exists': root_path.joinpath('src').exists(),
        'source_space_cmake_exists': root_path.joinpath('src', 'CMakeLists.txt').exists(),
        'devel_setup_exists': root_path.joinpath('devel', 'setup.bash').exists(),
        'install_setup_exists': root_path.joinpath('install', 'setup.bash').exists(),
    }


def enforce_vendor_bundle_preflight(report: Mapping[str, object], *, owner: str) -> None:
    """Raise when one required vendor-bundle preflight is unsatisfied.

    Args:
        report: Structured preflight report from :func:`build_vendor_bundle_preflight_report`.
        owner: Human-readable configuration owner for error messages.

    Returns:
        None.

    Raises:
        ConfigurationError: If the report is enabled in required mode and did not
            validate successfully.

    Boundary behavior:
        Advisory and off modes never raise. This keeps integration-time probes
        observable without silently weakening deploy-time hard gates.
    """
    if not bool(report.get('enabled', False)):
        return
    if not bool(report.get('required', False)):
        return
    if bool(report.get('satisfied', False)):
        return
    status = str(report.get('status', '')).strip() or 'vendor_bundle_preflight_failed'
    workspace_name = str(report.get('vendor_workspace_name', '')).strip()
    workspace_root = str(report.get('vendor_workspace_root', '')).strip()
    missing = [str(item).strip() for item in report.get('missing_entrypoints', ()) if str(item).strip()]
    missing_managed = [str(item).strip() for item in report.get('missing_managed_entrypoints', ()) if str(item).strip()]
    details = []
    if workspace_name:
        details.append(f'workspace={workspace_name}')
    if workspace_root:
        details.append(f'root={workspace_root}')
    if missing_managed:
        details.append('missing_managed_entrypoints=' + ','.join(missing_managed))
    if missing:
        details.append('missing_entrypoints=' + ','.join(missing))
    suffix = '' if not details else ' (' + '; '.join(details) + ')'
    raise ConfigurationError(f'{owner} vendor bundle preflight failed: {status}{suffix}')


def build_vendor_bundle_preflight_report(contract_summary: Mapping[str, object], config: Mapping[str, object]) -> dict:
    """Build one structured vendor-bundle preflight report.

    Args:
        contract_summary: Validated vendor runtime contract summary.
        config: Active runtime configuration mapping.

    Returns:
        One JSON-serializable report describing whether a local legacy vendor
        workspace was provided and whether required entrypoints exist.

    Boundary behavior:
        ``advisory`` mode reports unresolved/missing bundles without turning the
        report into a hard runtime gate; callers can still surface the failure in
        health details and decide separately whether to block activation.
    """
    runtime_mode = str(config.get('vendor_runtime_mode', '')).strip().lower()
    preflight_mode = _normalize_mode(config.get('vendor_bundle_preflight_mode', 'off'))
    if runtime_mode != 'isolated_legacy_workspace' or not contract_summary:
        return {
            'enabled': False,
            'mode': preflight_mode,
            'runtime_mode': runtime_mode,
            'required': False,
            'satisfied': True,
            'status': 'native_runtime',
            'vendor_workspace_name': str(contract_summary.get('vendor_workspace_name', '')).strip(),
            'vendor_workspace_root': '',
            'vendor_workspace_root_source': '',
            'workspace_exists': False,
            'entrypoints': [],
            'missing_entrypoints': [],
        }

    workspace_root, workspace_root_source = _resolve_workspace_root(config)
    workspace_name = str(contract_summary.get('vendor_workspace_name', '')).strip()
    bundle_manifest = load_vendor_bundle_manifest(str(config.get('vendor_bundle_manifest_path', '')).strip())
    bundle_lock = validate_vendor_bundle_lock(bundle_manifest, config)
    entrypoints = []
    missing = []
    vendor_entrypoint_map = dict(contract_summary.get('vendor_entrypoints', {}) or {})
    managed_entrypoints = []
    missing_managed = []
    startup_sequence = []
    missing_startup_sequence = []
    workspace_exists = False
    workspace_markers = {
        'src_dir_exists': False,
        'source_space_cmake_exists': False,
        'devel_setup_exists': False,
        'install_setup_exists': False,
    }
    repo_root = Path(__file__).resolve().parents[2]
    merged_managed_entrypoints = dict(contract_summary.get('managed_entrypoints', {}) or {})
    if bundle_manifest is not None:
        merged_managed_entrypoints.update(bundle_manifest.managed_entrypoints)
        vendor_entrypoint_map.update(bundle_manifest.vendor_entrypoints)
    for name, relative in merged_managed_entrypoints.items():
        rel = str(relative).strip()
        abs_path = str((repo_root / rel).resolve()) if rel else ''
        exists = bool(rel) and (repo_root / rel).exists()
        managed_entrypoints.append({
            'name': str(name).strip(),
            'relative_path': rel,
            'absolute_path': abs_path,
            'exists': bool(exists),
        })
        if rel and not exists:
            missing_managed.append(str(name).strip())
    if workspace_root:
        workspace_path = Path(workspace_root).expanduser()
        workspace_exists = workspace_path.exists()
        if workspace_exists:
            workspace_markers = _workspace_markers(workspace_root)
        for name, relative in vendor_entrypoint_map.items():
            rel = str(relative).strip()
            candidates = [str(item) for item in iter_vendor_workspace_entrypoint_candidates(workspace_path, rel)] if rel else []
            resolved_existing = next((item for item in candidates if Path(item).exists()), '')
            abs_path = resolved_existing or (candidates[0] if candidates else '')
            exists = workspace_exists and bool(rel) and bool(resolved_existing)
            entrypoints.append({
                'name': str(name).strip(),
                'relative_path': rel,
                'absolute_path': abs_path,
                'candidate_paths': candidates,
                'exists': bool(exists),
            })
            if rel and not exists:
                missing.append(str(name).strip())
    if bundle_manifest is not None:
        startup_sequence = list(bundle_manifest.startup_sequence)
        available_names = {item['name'] for item in managed_entrypoints if item.get('exists', False)}
        available_names.update(item['name'] for item in entrypoints if item.get('exists', False))
        missing_startup_sequence = [name for name in startup_sequence if name not in available_names]
    status = 'validated'
    satisfied = True
    if bundle_manifest is not None:
        if workspace_name and bundle_manifest.vendor_workspace_name != workspace_name:
            status = 'bundle_workspace_mismatch'
            satisfied = False
        elif runtime_mode and bundle_manifest.vendor_runtime_mode != runtime_mode:
            status = 'bundle_runtime_mode_mismatch'
            satisfied = False
        elif str(config.get('platform_adapter_type', '')).strip() and bundle_manifest.platform_adapter_type != str(config.get('platform_adapter_type', '')).strip():
            status = 'bundle_platform_adapter_mismatch'
            satisfied = False
        elif bundle_lock.get('enabled', False) and not bundle_lock.get('satisfied', False):
            status = str(bundle_lock.get('status', '')).strip() or 'bundle_lock_mismatch'
            satisfied = False
    if satisfied and not workspace_root:
        status = 'external_bundle_unresolved'
        satisfied = False
    elif satisfied and not workspace_exists:
        status = 'workspace_root_missing'
        satisfied = False
    elif satisfied and missing_managed:
        status = 'missing_managed_entrypoints'
        satisfied = False
    elif satisfied and missing_startup_sequence:
        status = 'startup_sequence_unbound'
        satisfied = False
    elif satisfied and missing:
        status = 'missing_entrypoints'
        satisfied = False
    elif satisfied and not bool(workspace_markers.get('src_dir_exists')):
        status = 'workspace_source_space_missing'
        satisfied = False
    elif satisfied and not bool(workspace_markers.get('source_space_cmake_exists')):
        status = 'workspace_cmake_marker_missing'
        satisfied = False
    return {
        'enabled': preflight_mode != 'off',
        'mode': preflight_mode,
        'runtime_mode': runtime_mode,
        'required': preflight_mode == 'required',
        'satisfied': satisfied,
        'status': status,
        'vendor_workspace_name': workspace_name,
        'vendor_workspace_root': workspace_root,
        'vendor_workspace_root_source': workspace_root_source,
        'workspace_exists': workspace_exists,
        'workspace_markers': workspace_markers,
        'bundle_manifest': bundle_manifest.to_summary() if bundle_manifest is not None else {},
        'bundle_lock': bundle_lock,
        'managed_entrypoints': managed_entrypoints,
        'missing_managed_entrypoints': missing_managed,
        'startup_sequence': startup_sequence,
        'missing_startup_sequence_entrypoints': missing_startup_sequence,
        'entrypoints': entrypoints,
        'missing_entrypoints': missing,
    }

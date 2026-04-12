"""Managed vendor-bundle manifest loaders and validators.

The repository still does not vendor proprietary MO-SERGEANT workspace sources.
This module instead turns the external runtime bundle into an explicit, locked,
versioned dependency that can be preflighted and launched in a reproducible way.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Mapping, Tuple

import yaml

from .domain_models import ConfigurationError
from .runtime_paths import expand_path, resolve_package_relative_path

_VALID_RUNTIME_MODES = ('isolated_legacy_workspace',)
_VALID_SETUP_PREFERENCES = ('devel_first', 'install_first')


@dataclass(frozen=True)
class VendorBundleManifest:
    """Normalized managed vendor runtime manifest.

    Args:
        bundle_id: Stable vendor bundle identifier.
        bundle_version: Managed bundle release version.
        platform_adapter_type: Expected platform adapter consuming this bundle.
        vendor_workspace_name: External workspace name.
        vendor_runtime_mode: Runtime strategy expected by the repository.
        required_ros_distro: Required ROS distribution for the vendor workspace.
        required_python_major: Required Python major version in the vendor workspace.
        managed_entrypoints: Repository-local integration assets.
        vendor_entrypoints: External workspace-relative entrypoints.
        startup_sequence: Ordered logical launch stages.
        source_setup_preference: Preferred setup script probing order.
        path: Absolute manifest path.
    """

    bundle_id: str
    bundle_version: str
    platform_adapter_type: str
    vendor_workspace_name: str
    vendor_runtime_mode: str
    required_ros_distro: str
    required_python_major: int
    managed_entrypoints: Dict[str, str]
    vendor_entrypoints: Dict[str, str]
    startup_sequence: Tuple[str, ...]
    source_setup_preference: str
    path: str

    def to_summary(self) -> Dict[str, object]:
        return {
            'bundle_id': self.bundle_id,
            'bundle_version': self.bundle_version,
            'platform_adapter_type': self.platform_adapter_type,
            'vendor_workspace_name': self.vendor_workspace_name,
            'vendor_runtime_mode': self.vendor_runtime_mode,
            'required_ros_distro': self.required_ros_distro,
            'required_python_major': int(self.required_python_major),
            'managed_entrypoints': dict(self.managed_entrypoints),
            'vendor_entrypoints': dict(self.vendor_entrypoints),
            'startup_sequence': list(self.startup_sequence),
            'source_setup_preference': self.source_setup_preference,
            'path': self.path,
        }


def _resolve_manifest_path(path: str) -> Path:
    resolved = resolve_package_relative_path(path)
    if resolved:
        return Path(resolved)
    expanded = Path(expand_path(path)).resolve()
    if not expanded.exists():
        raise ConfigurationError(f'vendor_bundle_manifest_path does not exist: {expanded}')
    return expanded


def _normalize_entrypoints(payload: object, *, owner: str) -> Dict[str, str]:
    if payload in ('', None):
        payload = {}
    if not isinstance(payload, Mapping):
        raise ConfigurationError(f'{owner} must be a mapping')
    normalized: Dict[str, str] = {}
    for raw_name, raw_path in dict(payload).items():
        name = str(raw_name).strip()
        rel = str(raw_path).strip()
        if not name or not rel:
            raise ConfigurationError(f'{owner} contains empty name/path entry')
        normalized[name] = rel
    return normalized


def load_vendor_bundle_manifest(path: str) -> VendorBundleManifest | None:
    """Load one managed vendor-bundle manifest.

    Args:
        path: Repository-relative or absolute manifest path. Empty values disable
            manifest loading and return ``None``.

    Returns:
        Parsed :class:`VendorBundleManifest` or ``None``.

    Raises:
        ConfigurationError: If the manifest is malformed or inconsistent.
    """
    normalized = str(path or '').strip()
    if not normalized:
        return None
    resolved = _resolve_manifest_path(normalized)
    payload = yaml.safe_load(resolved.read_text(encoding='utf-8')) or {}
    if not isinstance(payload, Mapping):
        raise ConfigurationError(f'vendor bundle manifest {resolved} must be a mapping')
    bundle_id = str(payload.get('bundle_id', '')).strip()
    bundle_version = str(payload.get('bundle_version', '')).strip()
    platform_adapter_type = str(payload.get('platform_adapter_type', '')).strip()
    vendor_workspace_name = str(payload.get('vendor_workspace_name', '')).strip()
    runtime_mode = str(payload.get('vendor_runtime_mode', '')).strip().lower()
    required_ros_distro = str(payload.get('required_ros_distro', '')).strip().lower()
    required_python_major = int(payload.get('required_python_major', 0) or 0)
    managed_entrypoints = _normalize_entrypoints(payload.get('managed_entrypoints', {}), owner='vendor bundle manifest managed_entrypoints')
    vendor_entrypoints = _normalize_entrypoints(payload.get('vendor_entrypoints', {}), owner='vendor bundle manifest vendor_entrypoints')
    startup_sequence = tuple(str(item).strip() for item in (payload.get('startup_sequence', []) or []) if str(item).strip())
    source_setup_preference = str(payload.get('source_setup_preference', 'devel_first')).strip().lower() or 'devel_first'
    missing = [
        name for name, value in [
            ('bundle_id', bundle_id),
            ('bundle_version', bundle_version),
            ('platform_adapter_type', platform_adapter_type),
            ('vendor_workspace_name', vendor_workspace_name),
            ('required_ros_distro', required_ros_distro),
        ]
        if not value
    ]
    if missing:
        raise ConfigurationError(f'vendor bundle manifest {resolved} missing required fields: {", ".join(missing)}')
    if runtime_mode not in _VALID_RUNTIME_MODES:
        raise ConfigurationError(f'vendor bundle manifest runtime mode must be one of {", ".join(_VALID_RUNTIME_MODES)}')
    if required_python_major <= 0:
        raise ConfigurationError('vendor bundle manifest required_python_major must be > 0')
    if source_setup_preference not in _VALID_SETUP_PREFERENCES:
        raise ConfigurationError(f'vendor bundle manifest source_setup_preference must be one of {", ".join(_VALID_SETUP_PREFERENCES)}')
    if not startup_sequence:
        raise ConfigurationError('vendor bundle manifest startup_sequence must not be empty')
    allowed_names = set(managed_entrypoints) | set(vendor_entrypoints)
    unknown_steps = [name for name in startup_sequence if name not in allowed_names]
    if unknown_steps:
        raise ConfigurationError(f'vendor bundle manifest startup_sequence references unknown entrypoints: {unknown_steps}')
    return VendorBundleManifest(
        bundle_id=bundle_id,
        bundle_version=bundle_version,
        platform_adapter_type=platform_adapter_type,
        vendor_workspace_name=vendor_workspace_name,
        vendor_runtime_mode=runtime_mode,
        required_ros_distro=required_ros_distro,
        required_python_major=required_python_major,
        managed_entrypoints=managed_entrypoints,
        vendor_entrypoints=vendor_entrypoints,
        startup_sequence=startup_sequence,
        source_setup_preference=source_setup_preference,
        path=str(resolved),
    )


def validate_vendor_bundle_lock(manifest: VendorBundleManifest | None, config: Mapping[str, object]) -> Dict[str, object]:
    """Validate one optional bundle lock against the active manifest.

    Args:
        manifest: Parsed managed vendor bundle manifest.
        config: Active runtime configuration.

    Returns:
        JSON-serializable lock report.

    Boundary behavior:
        Empty lock fields disable lock enforcement and still return a success
        report so health payloads can expose whether locking was in effect.
    """
    lock_id = str(config.get('vendor_bundle_lock_id', '')).strip()
    lock_version = str(config.get('vendor_bundle_lock_version', '')).strip()
    if manifest is None:
        return {
            'enabled': False,
            'satisfied': not bool(lock_id or lock_version),
            'lock_id': lock_id,
            'lock_version': lock_version,
            'bundle_id': '',
            'bundle_version': '',
            'status': 'no_manifest' if (lock_id or lock_version) else 'disabled',
        }
    if not lock_id and not lock_version:
        return {
            'enabled': False,
            'satisfied': True,
            'lock_id': '',
            'lock_version': '',
            'bundle_id': manifest.bundle_id,
            'bundle_version': manifest.bundle_version,
            'status': 'disabled',
        }
    id_ok = (not lock_id) or lock_id == manifest.bundle_id
    version_ok = (not lock_version) or lock_version == manifest.bundle_version
    return {
        'enabled': True,
        'satisfied': bool(id_ok and version_ok),
        'lock_id': lock_id,
        'lock_version': lock_version,
        'bundle_id': manifest.bundle_id,
        'bundle_version': manifest.bundle_version,
        'status': 'validated' if (id_ok and version_ok) else 'lock_mismatch',
    }


@dataclass(frozen=True)
class VendorStartupStep:
    """Resolved startup step from one managed vendor bundle.

    Args:
        name: Logical startup-step name from ``startup_sequence``.
        source: ``managed`` for repository assets or ``vendor`` for workspace
            assets.
        kind: ``launch`` for roslaunch includes or ``node`` for executable
            script entrypoints.
        relative_path: Original repository-relative or workspace-relative path.
        absolute_path: Fully resolved absolute filesystem path.
        package: ROS package owning the entrypoint when ``kind=node``.
        executable: Executable file name when ``kind=node``.

    Boundary behavior:
        Unsupported entrypoint layouts are rejected eagerly so managed runtime
        wrappers fail before any partial vendor bringup occurs.
    """

    name: str
    source: str
    kind: str
    relative_path: str
    absolute_path: str
    package: str = ''
    executable: str = ''


def _classify_entrypoint(relative_path: str, *, owner: str) -> tuple[str, str, str]:
    parts = [str(item).strip() for item in Path(relative_path).parts if str(item).strip()]
    if not parts:
        raise ConfigurationError(f'{owner} contains an empty entrypoint path')
    package = parts[0]
    suffix = Path(relative_path).suffix.lower()
    if suffix == '.launch':
        return 'launch', package, Path(relative_path).name
    if suffix == '.py' and 'scripts' in parts:
        return 'node', package, Path(relative_path).name
    raise ConfigurationError(f'{owner} entrypoint {relative_path} must resolve to pkg/launch/*.launch or pkg/scripts/*.py')


def iter_vendor_workspace_entrypoint_candidates(workspace_root: str | Path, relative_path: str) -> tuple[Path, ...]:
    """Return candidate filesystem locations for one vendor workspace entrypoint.

    The managed vendor bundle treats ``vendor_workspace_root`` as the catkin
    workspace root by default. Real MO-SERGEANT vendor workspaces store source
    packages under ``<workspace>/src``. Some older ad-hoc layouts may still
    place packages directly under the workspace root, so we probe both shapes in
    a deterministic order while preferring the canonical ``src`` layout.
    """
    workspace_path = Path(expand_path(str(workspace_root))).expanduser().resolve()
    relative = Path(relative_path)
    candidates = []
    source_space = workspace_path / 'src' / relative
    candidates.append(source_space.resolve())
    candidates.append((workspace_path / relative).resolve())
    ordered = []
    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        ordered.append(candidate)
    return tuple(ordered)


def resolve_vendor_workspace_entrypoint_path(workspace_root: str | Path, relative_path: str) -> Path:
    """Resolve one vendor workspace entrypoint against canonical and legacy layouts."""
    candidates = iter_vendor_workspace_entrypoint_candidates(workspace_root, relative_path)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise ConfigurationError(
        'vendor workspace entrypoint is missing: {} (probed: {})'.format(
            relative_path,
            ', '.join(str(item) for item in candidates),
        )
    )


def resolve_vendor_bundle_startup_steps(manifest: VendorBundleManifest, *, workspace_root: str, repo_root: str | Path | None = None) -> tuple[VendorStartupStep, ...]:
    """Resolve every startup-sequence item to one concrete filesystem target.

    Args:
        manifest: Parsed managed vendor bundle manifest.
        workspace_root: External vendor workspace root containing source-space
            packages such as ``car_bringup`` and ``nav_demo``.
        repo_root: Repository root used to resolve repository-managed assets.

    Returns:
        Tuple of resolved :class:`VendorStartupStep` objects in startup order.

    Raises:
        ConfigurationError: If any referenced path is missing or uses an
            unsupported layout.
    """
    workspace_path = Path(expand_path(workspace_root)).expanduser().resolve()
    if not workspace_path.exists():
        raise ConfigurationError(f'vendor workspace root does not exist: {workspace_path}')
    repo_path = Path(repo_root).resolve() if repo_root is not None else Path(__file__).resolve().parents[2]
    steps = []
    for name in manifest.startup_sequence:
        if name in manifest.managed_entrypoints:
            rel = manifest.managed_entrypoints[name]
            abs_path = (repo_path / rel).resolve()
            if not abs_path.exists():
                raise ConfigurationError(f'managed vendor bundle entrypoint {name} is missing: {abs_path}')
            kind, package, executable = _classify_entrypoint(rel, owner='managed_entrypoints')
            steps.append(VendorStartupStep(name=name, source='managed', kind=kind, relative_path=rel, absolute_path=str(abs_path), package=package, executable=executable))
            continue
        rel = manifest.vendor_entrypoints.get(name, '')
        if not rel:
            raise ConfigurationError(f'vendor bundle startup_sequence references unknown step {name}')
        try:
            abs_path = resolve_vendor_workspace_entrypoint_path(workspace_path, rel)
        except ConfigurationError as exc:
            raise ConfigurationError(f'vendor bundle entrypoint {name} is missing: {exc}') from exc
        kind, package, executable = _classify_entrypoint(rel, owner='vendor_entrypoints')
        steps.append(VendorStartupStep(name=name, source='vendor', kind=kind, relative_path=rel, absolute_path=str(abs_path), package=package, executable=executable))
    return tuple(steps)

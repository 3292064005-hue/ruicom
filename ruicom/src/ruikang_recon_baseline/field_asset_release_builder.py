"""Helpers for building field-asset release manifests from explicit inputs."""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import yaml

from .domain_models import ConfigurationError
from .field_assets import load_field_asset, resolve_field_asset_path
from .field_asset_release import load_field_asset_release
from .manifest_utils import detector_manifest_satisfies_scope, load_manifest
from .runtime_paths import expand_path, resolve_package_relative_path


_VALID_SCOPES = ('contract', 'reference', 'field')


def _resolve_manifest_path(path: str) -> Path:
    resolved = resolve_package_relative_path(path)
    if resolved:
        return Path(resolved)
    expanded = Path(expand_path(path)).resolve()
    if not expanded.exists():
        raise ConfigurationError('detector_manifest_path does not exist: {}'.format(expanded))
    return expanded


def build_field_asset_release_payload(*, repo_root: Path, release_id: str, release_version: str, field_asset_id: str = '', field_asset_path: str = '', verification_scope: str, detector_manifest_path: str, approved_at_utc: str, reviewer_id: str, notes: str = '') -> Dict[str, object]:
    normalized_scope = str(verification_scope).strip().lower()
    if normalized_scope not in _VALID_SCOPES:
        raise ConfigurationError('verification_scope must be one of {}'.format(', '.join(_VALID_SCOPES)))
    if not str(release_id).strip():
        raise ConfigurationError('release_id must not be empty')
    if not str(release_version).strip():
        raise ConfigurationError('release_version must not be empty')
    if not str(approved_at_utc).strip():
        raise ConfigurationError('approved_at_utc must not be empty')
    if not str(reviewer_id).strip():
        raise ConfigurationError('reviewer_id must not be empty')
    asset = load_field_asset(field_asset_id=field_asset_id, field_asset_path=field_asset_path)
    if asset is None:
        raise ConfigurationError('field asset could not be resolved')
    if not asset.satisfies_scope(normalized_scope):
        raise ConfigurationError('field asset {} does not satisfy verification scope {}'.format(asset.asset_id, normalized_scope))
    manifest_path = _resolve_manifest_path(detector_manifest_path)
    manifest = load_manifest(str(manifest_path))
    if not detector_manifest_satisfies_scope(manifest, required_scope=normalized_scope):
        raise ConfigurationError('detector manifest {} does not satisfy verification scope {}'.format(manifest_path, normalized_scope))
    asset_path_resolved = resolve_field_asset_path(field_asset_id=field_asset_id, field_asset_path=field_asset_path)
    if asset_path_resolved is None:
        raise ConfigurationError('field asset path could not be resolved')
    payload = {
        'release_id': str(release_id).strip(),
        'release_version': str(release_version).strip(),
        'verification_scope': normalized_scope,
        'field_asset_id': asset.asset_id,
        'field_asset_path': str(asset_path_resolved.resolve().relative_to(repo_root)),
        'detector_manifest_path': str(manifest_path.resolve().relative_to(repo_root)),
        'approved_at_utc': str(approved_at_utc).strip(),
        'reviewer_id': str(reviewer_id).strip(),
        'notes': str(notes).strip(),
    }
    return payload


def write_field_asset_release_manifest(*, output_path: str, payload: Dict[str, object]) -> Path:
    target = Path(output_path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding='utf-8')
    return target


def build_and_validate_field_asset_release(*, repo_root: Path, output_path: str, **kwargs) -> Path:
    payload = build_field_asset_release_payload(repo_root=repo_root, **kwargs)
    target = write_field_asset_release_manifest(output_path=output_path, payload=payload)
    release = load_field_asset_release(str(target))
    if release is None:
        raise ConfigurationError('failed to load generated release manifest: {}'.format(target))
    return target

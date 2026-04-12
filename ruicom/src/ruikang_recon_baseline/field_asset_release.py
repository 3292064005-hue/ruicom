"""Field-asset release manifest helpers.

Release manifests bind a specific field asset, detector manifest and provenance
review into one immutable deploy-facing artifact. This keeps reference releases
and future measured field releases machine-checkable.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping

import yaml

from .domain_models import ConfigurationError
from .field_assets import FieldAsset, load_field_asset
from .manifest_utils import detector_manifest_satisfies_scope, load_manifest
from .runtime_paths import expand_path, resolve_package_relative_path


@dataclass(frozen=True)
class FieldAssetRelease:
    release_id: str
    release_version: str
    verification_scope: str
    field_asset: FieldAsset
    detector_manifest_path: str
    detector_model_id: str
    approved_at_utc: str
    reviewer_id: str
    notes: str
    path: str

    def to_summary(self) -> Dict[str, object]:
        return {
            'release_id': self.release_id,
            'release_version': self.release_version,
            'verification_scope': self.verification_scope,
            'field_asset_id': self.field_asset.asset_id,
            'asset_version': str(self.field_asset.metadata.get('asset_version', '')).strip(),
            'detector_manifest_path': self.detector_manifest_path,
            'detector_model_id': self.detector_model_id,
            'approved_at_utc': self.approved_at_utc,
            'reviewer_id': self.reviewer_id,
            'path': self.path,
        }


def _resolve_release_path(path: str) -> Path:
    resolved = resolve_package_relative_path(path)
    if resolved:
        return Path(resolved)
    expanded = Path(expand_path(path)).resolve()
    if not expanded.exists():
        raise ConfigurationError(f'field_asset_release_manifest_path does not exist: {expanded}')
    return expanded


def load_field_asset_release(path: str) -> FieldAssetRelease | None:
    """Load and validate one field-asset release manifest."""
    normalized = str(path or '').strip()
    if not normalized:
        return None
    resolved = _resolve_release_path(normalized)
    payload = yaml.safe_load(resolved.read_text(encoding='utf-8')) or {}
    if not isinstance(payload, Mapping):
        raise ConfigurationError(f'field asset release manifest {resolved} must be a mapping')
    release_id = str(payload.get('release_id', '')).strip()
    release_version = str(payload.get('release_version', '')).strip()
    verification_scope = str(payload.get('verification_scope', '')).strip().lower()
    reviewer_id = str(payload.get('reviewer_id', '')).strip()
    approved_at_utc = str(payload.get('approved_at_utc', '')).strip()
    notes = str(payload.get('notes', '')).strip()
    asset_id = str(payload.get('field_asset_id', '')).strip()
    asset_path = str(payload.get('field_asset_path', '')).strip()
    detector_manifest_path = str(payload.get('detector_manifest_path', '')).strip()
    missing = [
        name for name, value in [
            ('release_id', release_id),
            ('release_version', release_version),
            ('verification_scope', verification_scope),
            ('reviewer_id', reviewer_id),
            ('approved_at_utc', approved_at_utc),
            ('detector_manifest_path', detector_manifest_path),
        ]
        if not value
    ]
    if missing:
        raise ConfigurationError(f'field asset release manifest {resolved} missing required fields: {", ".join(missing)}')
    asset = load_field_asset(field_asset_id=asset_id, field_asset_path=asset_path)
    if asset is None:
        raise ConfigurationError(f'field asset release manifest {resolved} must reference an existing asset')
    if verification_scope not in ('contract', 'reference', 'field'):
        raise ConfigurationError('field asset release verification_scope must be contract, reference or field')
    if not asset.satisfies_scope(verification_scope):
        raise ConfigurationError(
            f'field asset release {release_id} requires {verification_scope} scope but asset {asset.asset_id} provides {asset.verification_scope or "unverified"}'
        )
    asset_manifest_path = str(asset.metadata.get('detector_manifest_path', '')).strip()
    if asset_manifest_path and asset_manifest_path != detector_manifest_path:
        raise ConfigurationError(
            f'field asset release {release_id} detector manifest {detector_manifest_path} conflicts with asset manifest {asset_manifest_path}'
        )
    manifest_resolved = resolve_package_relative_path(detector_manifest_path) or str(Path(expand_path(detector_manifest_path)).resolve())
    manifest = load_manifest(manifest_resolved)
    if not detector_manifest_satisfies_scope(manifest, required_scope=verification_scope):
        raise ConfigurationError(
            f'field asset release {release_id} detector manifest {manifest.model_id or manifest_resolved} grade {manifest.deployment_grade or "ungraded"} does not satisfy {verification_scope}'
        )
    return FieldAssetRelease(
        release_id=release_id,
        release_version=release_version,
        verification_scope=verification_scope,
        field_asset=asset,
        detector_manifest_path=detector_manifest_path,
        detector_model_id=str(manifest.model_id or '').strip(),
        approved_at_utc=approved_at_utc,
        reviewer_id=reviewer_id,
        notes=notes,
        path=str(resolved),
    )

"""Field-asset loading for route and frame-region authoring.

Field assets turn map-bound route coordinates and image-space region layouts into
versioned runtime assets instead of ad-hoc per-profile hardcoding.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import yaml

from .domain_models import ConfigurationError
from .runtime_paths import expand_path, resolve_package_relative_path


_VALID_VERIFICATION_SCOPES = ('contract', 'reference', 'field')


@dataclass(frozen=True)
class FieldAsset:
    """Resolved field asset plus immutable verification metadata.

    Args:
        asset_id: Stable asset identifier.
        description: Human-readable description.
        comparison_frame: Map frame shared by mission and localization.
        route: Authoritative route definitions.
        named_regions: Authoritative named image regions.
        expected_region_names: Normalized region-name allowlist.
        metadata: Additional immutable provenance and verification details.

    Returns:
        FieldAsset dataclass instance.

    Raises:
        No explicit exception is raised during property access.

    Boundary behavior:
        Verification scope is ordered ``contract < reference < field``. Provenance
        stays separate from scope so repository-packaged field assets can support
        code-delivery closure without being mistaken for measured competition
        captures.
    """

    asset_id: str
    description: str
    comparison_frame: str
    route: List[dict]
    named_regions: List[dict]
    expected_region_names: List[str]
    metadata: Dict[str, object]

    @property
    def verified(self) -> bool:
        return bool(self.metadata.get('verified', False))

    @property
    def state(self) -> str:
        return str(self.metadata.get('asset_state', '')).strip().lower() or 'unspecified'

    @property
    def verification_scope(self) -> str:
        scope = str(self.metadata.get('verification_scope', '')).strip().lower()
        return scope if scope in _VALID_VERIFICATION_SCOPES else ''

    @property
    def provenance(self) -> str:
        provenance = str(self.metadata.get('provenance', '')).strip().lower()
        return provenance or 'unspecified'

    def satisfies_scope(self, required_scope: str) -> bool:
        """Return whether the asset satisfies the requested verification scope.

        Args:
            required_scope: Verification scope required by the runtime. Accepted
                values are ``contract``, ``reference`` and ``field``. Empty
                values disable scope filtering.

        Returns:
            ``True`` when the asset is verified and its scope is equal to or
            stronger than the requested scope.

        Raises:
            ConfigurationError: If the required scope is unknown.

        Boundary behavior:
            Unverified assets never satisfy any non-empty scope. Empty required
            scopes are treated as disabled gates and always succeed.
        """
        normalized = str(required_scope).strip().lower()
        if not normalized:
            return True
        if normalized not in _VALID_VERIFICATION_SCOPES:
            raise ConfigurationError('required_field_asset_verification_scope must be one of {}'.format(', '.join(_VALID_VERIFICATION_SCOPES)))
        if not self.verified:
            return False
        if normalized == 'contract':
            return self.verification_scope in ('contract', 'reference', 'field')
        if normalized == 'reference':
            return self.verification_scope in ('reference', 'field')
        return self.verification_scope == 'field'



def _candidate_search_roots(package_root: str | None = None) -> tuple[Path, ...]:
    if package_root:
        base = Path(expand_path(package_root)).resolve()
        candidates = [base]
        nested = base / 'config' / 'field_assets'
        if nested.exists():
            candidates.insert(0, nested)
    else:
        candidates = [Path(__file__).resolve().parents[2] / 'config' / 'field_assets']
    ordered = []
    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        ordered.append(candidate)
    return tuple(ordered)



def resolve_field_asset_path(*, field_asset_id: str = '', field_asset_path: str = '', package_root: str | None = None) -> Optional[Path]:
    normalized_path = str(field_asset_path).strip()
    if normalized_path:
        resolved = Path(expand_path(normalized_path))
        if not resolved.exists():
            raise ConfigurationError('field_asset_path does not exist: {}'.format(resolved))
        return resolved
    normalized_id = str(field_asset_id).strip()
    if not normalized_id:
        return None
    roots = _candidate_search_roots(package_root)
    for root in roots:
        resolved = root / '{}.yaml'.format(normalized_id)
        if resolved.exists():
            return resolved
    raise ConfigurationError('field_asset_id {} not found under {}'.format(normalized_id, ', '.join(str(item) for item in roots)))



def _normalize_region_names(named_regions: List[dict], expected_region_names: List[str]) -> List[str]:
    if expected_region_names:
        return [str(item).strip() for item in expected_region_names if str(item).strip()]
    return [str(item.get('name', '')).strip() for item in named_regions if str(item.get('name', '')).strip()]



def _build_region_calibration(payload: dict, resolved_path: Path) -> Dict[str, Any]:
    raw = payload.get('region_calibration', {}) or {}
    if raw in ('', None):
        raw = {}
    if not isinstance(raw, dict):
        raise ConfigurationError('field asset {} region_calibration must be a mapping'.format(resolved_path.stem))
    if not raw:
        return {}
    reference_width = int(raw.get('reference_image_width', 0) or 0)
    reference_height = int(raw.get('reference_image_height', 0) or 0)
    if reference_width <= 0 or reference_height <= 0:
        raise ConfigurationError('field asset {} region_calibration requires positive reference_image_width/reference_image_height'.format(resolved_path.stem))
    x_scale = float(raw.get('x_scale', 1.0))
    y_scale = float(raw.get('y_scale', 1.0))
    x_offset_px = float(raw.get('x_offset_px', 0.0))
    y_offset_px = float(raw.get('y_offset_px', 0.0))
    if x_scale <= 0.0 or y_scale <= 0.0:
        raise ConfigurationError('field asset {} region_calibration scale must be > 0'.format(resolved_path.stem))
    calibration_id = str(raw.get('calibration_id', '')).strip() or str(payload.get('camera_calibration_id', '')).strip()
    return {
        'calibration_id': calibration_id,
        'reference_image_width': reference_width,
        'reference_image_height': reference_height,
        'x_scale': x_scale,
        'y_scale': y_scale,
        'x_offset_px': x_offset_px,
        'y_offset_px': y_offset_px,
        'policy': str(raw.get('policy', 'scale_to_frame')).strip().lower() or 'scale_to_frame',
    }


def _infer_asset_provenance(asset_state: str, verification_scope: str) -> str:
    state = str(asset_state).strip().lower()
    scope = str(verification_scope).strip().lower()
    if state.startswith('packaged_'):
        return 'packaged'
    if 'reference' in state or scope == 'reference':
        return 'reference'
    if scope == 'field' and ('measured' in state or 'survey' in state or state == 'field_verified'):
        return 'measured'
    if scope == 'field':
        return 'field_unmeasured'
    if scope == 'contract':
        return 'contract'
    return 'unspecified'


def _build_metadata(payload: dict, resolved_path: Path) -> Dict[str, object]:
    asset_state = str(payload.get('asset_state', '')).strip().lower() or 'provisional'
    verified = bool(payload.get('verified', False))
    verification_scope = str(payload.get('verification_scope', '')).strip().lower()
    if verified:
        if verification_scope not in _VALID_VERIFICATION_SCOPES:
            raise ConfigurationError('field asset {} requires verification_scope in {}'.format(resolved_path.stem, _VALID_VERIFICATION_SCOPES))
        if asset_state in ('placeholder', 'provisional'):
            raise ConfigurationError('field asset {} cannot be verified while asset_state={}'.format(resolved_path.stem, asset_state))
    elif verification_scope:
        if verification_scope not in _VALID_VERIFICATION_SCOPES:
            raise ConfigurationError('field asset {} has invalid verification_scope {}'.format(resolved_path.stem, verification_scope))
    measured_at_utc = str(payload.get('measured_at_utc', '')).strip()
    collection_batch_id = str(payload.get('collection_batch_id', '')).strip()
    operator_id = str(payload.get('operator_id', '')).strip()
    calibration_bundle_id = str(payload.get('calibration_bundle_id', '')).strip()
    camera_calibration_id = str(payload.get('camera_calibration_id', '')).strip()
    detector_manifest_path = str(payload.get('detector_manifest_path', '')).strip()
    if verified and verification_scope == 'field':
        missing = [
            name for name, value in [
                ('measured_at_utc', measured_at_utc),
                ('collection_batch_id', collection_batch_id),
                ('operator_id', operator_id),
                ('calibration_bundle_id', calibration_bundle_id),
                ('camera_calibration_id', camera_calibration_id),
                ('detector_manifest_path', detector_manifest_path),
            ]
            if not value
        ]
        if missing:
            raise ConfigurationError('field asset {} missing field verification provenance {}'.format(resolved_path.stem, ', '.join(missing)))
    return {
        'source_path': str(resolved_path),
        'map_id': str(payload.get('map_id', '')).strip(),
        'camera_calibration_id': camera_calibration_id,
        'asset_version': str(payload.get('asset_version', '')).strip(),
        'asset_state': asset_state,
        'verified': verified,
        'verification_scope': verification_scope,
        'route_layout_id': str(payload.get('route_layout_id', '')).strip(),
        'named_region_layout_id': str(payload.get('named_region_layout_id', '')).strip(),
        'calibration_bundle_id': calibration_bundle_id,
        'region_calibration': _build_region_calibration(payload, resolved_path),
        'measured_at_utc': measured_at_utc,
        'collection_batch_id': collection_batch_id,
        'operator_id': operator_id,
        'detector_manifest_path': detector_manifest_path,
        'notes': str(payload.get('notes', '')).strip(),
        'provenance': _infer_asset_provenance(asset_state, verification_scope),
    }



def load_field_asset(*, field_asset_id: str = '', field_asset_path: str = '', package_root: str | None = None) -> Optional[FieldAsset]:
    resolved_path = resolve_field_asset_path(field_asset_id=field_asset_id, field_asset_path=field_asset_path, package_root=package_root)
    if resolved_path is None:
        return None
    payload = yaml.safe_load(resolved_path.read_text(encoding='utf-8')) or {}
    asset_id = str(payload.get('field_asset_id', '')).strip() or str(field_asset_id).strip() or resolved_path.stem
    comparison_frame = str(payload.get('comparison_frame', '')).strip() or 'map'
    route = list(payload.get('route', []) or [])
    named_regions = list(payload.get('named_regions', []) or [])
    expected_region_names = _normalize_region_names(named_regions, list(payload.get('expected_region_names', []) or []))
    if not route:
        raise ConfigurationError('field asset {} must define a non-empty route'.format(asset_id))
    if not named_regions:
        raise ConfigurationError('field asset {} must define non-empty named_regions'.format(asset_id))
    metadata = _build_metadata(payload, resolved_path)
    return FieldAsset(
        asset_id=asset_id,
        description=str(payload.get('description', '')).strip(),
        comparison_frame=comparison_frame,
        route=route,
        named_regions=named_regions,
        expected_region_names=expected_region_names,
        metadata=metadata,
    )



def _assert_compatible_inline(name: str, inline_value, asset_value, *, owner: str) -> None:
    if inline_value not in (None, '', [], {}):
        if inline_value != asset_value:
            raise ConfigurationError('{} provided both inline {} and field asset {} with different values'.format(owner, name, name))



def _apply_field_asset_manifest_alignment(config: dict, asset: FieldAsset, *, owner: str) -> None:
    """Align one deploy detector manifest with field-asset provenance.

    Args:
        config: Mutable component configuration mapping.
        asset: Resolved field asset.
        owner: Human-readable configuration owner.

    Returns:
        None. The configuration mapping is updated in place.

    Raises:
        ConfigurationError: If the component already declares a conflicting
            detector manifest path.

    Boundary behavior:
        Field assets may carry a repository-relative detector manifest path. When
        the component did not declare ``model_manifest_path`` explicitly, the
        field-asset path is injected automatically. If both are present, they
        must resolve to the same file.
    """
    manifest_hint = str(asset.metadata.get('detector_manifest_path', '')).strip()
    if not manifest_hint:
        return
    resolved_asset_manifest = resolve_package_relative_path(manifest_hint) or str(Path(expand_path(manifest_hint)).resolve())
    existing_manifest = str(config.get('model_manifest_path', '')).strip()
    if existing_manifest:
        resolved_existing = resolve_package_relative_path(existing_manifest) or str(Path(expand_path(existing_manifest)).resolve())
        if resolved_existing != resolved_asset_manifest:
            raise ConfigurationError(
                '{} model_manifest_path {} conflicts with field asset detector manifest {}'.format(
                    owner,
                    existing_manifest,
                    manifest_hint,
                )
            )
    else:
        config['model_manifest_path'] = manifest_hint
    config['field_asset_detector_manifest_path'] = manifest_hint
    config['field_asset_detector_manifest_resolved_path'] = resolved_asset_manifest



def _inject_asset_contract(config: dict, asset: FieldAsset, *, owner: str) -> None:
    config['field_asset_id'] = asset.asset_id
    config['field_asset_metadata'] = dict(asset.metadata)
    config['field_asset_verified'] = bool(asset.verified)
    config['field_asset_state'] = asset.state
    config['field_asset_verification_scope'] = asset.verification_scope
    config['field_asset_provenance'] = asset.provenance
    required_scope = str(config.get('required_field_asset_verification_scope', '')).strip().lower() or 'contract'
    required_provenance = str(config.get('required_field_asset_provenance', '')).strip().lower()
    require_verified = bool(config.get('require_verified_field_asset', False))
    if required_scope not in _VALID_VERIFICATION_SCOPES:
        raise ConfigurationError('required_field_asset_verification_scope must be one of {}'.format(', '.join(_VALID_VERIFICATION_SCOPES)))
    config['required_field_asset_verification_scope'] = required_scope
    if required_provenance:
        allowed = {'contract', 'reference', 'packaged', 'field_unmeasured', 'measured'}
        if required_provenance not in allowed:
            raise ConfigurationError('required_field_asset_provenance must be one of {}'.format(', '.join(sorted(allowed))))
        config['required_field_asset_provenance'] = required_provenance
    if require_verified and not asset.verified:
        raise ConfigurationError('{} requires verified field asset but {} is state={} verified={}'.format(owner, asset.asset_id, asset.state, asset.verified))
    if require_verified and not asset.satisfies_scope(required_scope):
        raise ConfigurationError(
            '{} requires {}-verified field asset but {} provides scope={}'.format(
                owner,
                required_scope,
                asset.asset_id,
                asset.verification_scope or 'unverified',
            )
        )
    if require_verified and required_provenance and asset.provenance != required_provenance:
        raise ConfigurationError(
            '{} requires {} field asset provenance but {} provides provenance={}'.format(
                owner,
                required_provenance,
                asset.asset_id,
                asset.provenance,
            )
        )
    config['field_asset_contract_satisfied'] = True





def _apply_field_asset_release_manifest(config: dict, *, owner: str) -> None:
    from .field_asset_release import load_field_asset_release

    release = load_field_asset_release(str(config.get('field_asset_release_manifest_path', '')).strip())
    if release is None:
        return
    config['field_asset_release_manifest'] = release.to_summary()
    release_asset = release.field_asset
    existing_asset_id = str(config.get('field_asset_id', '')).strip()
    if existing_asset_id and existing_asset_id != release_asset.asset_id:
        raise ConfigurationError(f"{owner} field_asset_id {existing_asset_id} conflicts with release manifest asset {release_asset.asset_id}")
    existing_manifest = str(config.get('model_manifest_path', '')).strip()
    if existing_manifest and existing_manifest != release.detector_manifest_path:
        raise ConfigurationError(f"{owner} model_manifest_path {existing_manifest} conflicts with release manifest detector manifest {release.detector_manifest_path}")
    config['field_asset_id'] = release_asset.asset_id
    config['model_manifest_path'] = release.detector_manifest_path

def apply_field_asset_to_mission_config(config: dict, *, package_root: str | None = None, owner: str = 'mission', domain: str | None = None) -> tuple[dict, Optional[FieldAsset]]:
    _ = domain
    _apply_field_asset_release_manifest(config, owner=owner)
    asset = load_field_asset(
        field_asset_id=str(config.get('field_asset_id', '')).strip(),
        field_asset_path=str(config.get('field_asset_path', '')).strip(),
        package_root=str(config.get('field_asset_package_root', '')).strip() or package_root,
    )
    if asset is None:
        require_verified = bool(config.get('require_verified_field_asset', False))
        config['field_asset_contract_satisfied'] = not require_verified
        if require_verified:
            raise ConfigurationError('{} requires a verified field asset but neither field_asset_id nor field_asset_path was provided'.format(owner))
        return config, None
    _assert_compatible_inline('route', config.get('route'), asset.route, owner=owner)
    _assert_compatible_inline('expected_frame_regions', config.get('expected_frame_regions'), asset.expected_region_names, owner=owner)
    config['route'] = list(asset.route)
    config['expected_frame_regions'] = list(asset.expected_region_names)
    if not str(config.get('comparison_frame', '')).strip():
        config['comparison_frame'] = asset.comparison_frame
    _apply_field_asset_manifest_alignment(config, asset, owner=owner)
    _inject_asset_contract(config, asset, owner=owner)
    return config, asset



def apply_field_asset_to_vision_config(config: dict, *, package_root: str | None = None, owner: str = 'vision', domain: str | None = None) -> tuple[dict, Optional[FieldAsset]]:
    _ = domain
    _apply_field_asset_release_manifest(config, owner=owner)
    asset = load_field_asset(
        field_asset_id=str(config.get('field_asset_id', '')).strip(),
        field_asset_path=str(config.get('field_asset_path', '')).strip(),
        package_root=str(config.get('field_asset_package_root', '')).strip() or package_root,
    )
    if asset is None:
        require_verified = bool(config.get('require_verified_field_asset', False))
        config['field_asset_contract_satisfied'] = not require_verified
        if require_verified:
            raise ConfigurationError('{} requires a verified field asset but neither field_asset_id nor field_asset_path was provided'.format(owner))
        return config, None
    _assert_compatible_inline('named_regions', config.get('named_regions'), asset.named_regions, owner=owner)
    _assert_compatible_inline('expected_region_names', config.get('expected_region_names'), asset.expected_region_names, owner=owner)
    config['named_regions'] = list(asset.named_regions)
    config['expected_region_names'] = list(asset.expected_region_names)
    region_calibration = dict(asset.metadata.get('region_calibration', {}) or {})
    if region_calibration:
        config['region_calibration'] = region_calibration
        if str(config.get('frame_region_adapter_type', '')).strip().lower() in ('none', 'named_regions', ''):
            config['frame_region_adapter_type'] = 'calibrated_named_regions'
    _apply_field_asset_manifest_alignment(config, asset, owner=owner)
    _inject_asset_contract(config, asset, owner=owner)
    return config, asset

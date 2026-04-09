"""Field-asset loading for route and frame-region authoring.

Field assets turn map-bound route coordinates and image-space region layouts into
versioned runtime assets instead of ad-hoc per-profile hardcoding.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from .domain_models import ConfigurationError
from .runtime_paths import expand_path


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
        Verification scope is ordered ``contract < reference < field`` so a
        repository-managed reference asset can satisfy reference deploy gates
        without being mistaken for a real measured field deployment asset.
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



def _default_search_root(package_root: str | None = None) -> Path:
    if package_root:
        return Path(expand_path(package_root))
    return Path(__file__).resolve().parents[2] / 'config' / 'field_assets'



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
    root = _default_search_root(package_root)
    resolved = root / '{}.yaml'.format(normalized_id)
    if not resolved.exists():
        raise ConfigurationError('field_asset_id {} not found under {}'.format(normalized_id, root))
    return resolved



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
    return {
        'source_path': str(resolved_path),
        'map_id': str(payload.get('map_id', '')).strip(),
        'camera_calibration_id': str(payload.get('camera_calibration_id', '')).strip(),
        'asset_version': str(payload.get('asset_version', '')).strip(),
        'asset_state': asset_state,
        'verified': verified,
        'verification_scope': verification_scope,
        'route_layout_id': str(payload.get('route_layout_id', '')).strip(),
        'named_region_layout_id': str(payload.get('named_region_layout_id', '')).strip(),
        'calibration_bundle_id': str(payload.get('calibration_bundle_id', '')).strip(),
        'region_calibration': _build_region_calibration(payload, resolved_path),
        'notes': str(payload.get('notes', '')).strip(),
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



def _inject_asset_contract(config: dict, asset: FieldAsset, *, owner: str) -> None:
    config['field_asset_id'] = asset.asset_id
    config['field_asset_metadata'] = dict(asset.metadata)
    config['field_asset_verified'] = bool(asset.verified)
    config['field_asset_state'] = asset.state
    config['field_asset_verification_scope'] = asset.verification_scope
    required_scope = str(config.get('required_field_asset_verification_scope', '')).strip().lower() or 'contract'
    require_verified = bool(config.get('require_verified_field_asset', False))
    if required_scope not in _VALID_VERIFICATION_SCOPES:
        raise ConfigurationError('required_field_asset_verification_scope must be one of {}'.format(', '.join(_VALID_VERIFICATION_SCOPES)))
    config['required_field_asset_verification_scope'] = required_scope
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
    config['field_asset_contract_satisfied'] = True



def apply_field_asset_to_mission_config(config: dict, *, package_root: str | None = None, owner: str = 'mission', domain: str | None = None) -> tuple[dict, Optional[FieldAsset]]:
    _ = domain
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
    _inject_asset_contract(config, asset, owner=owner)
    return config, asset



def apply_field_asset_to_vision_config(config: dict, *, package_root: str | None = None, owner: str = 'vision', domain: str | None = None) -> tuple[dict, Optional[FieldAsset]]:
    _ = domain
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
    _inject_asset_contract(config, asset, owner=owner)
    return config, asset

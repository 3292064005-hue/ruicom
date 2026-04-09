"""Schema, profile-policy and count-matrix helpers."""

from __future__ import annotations

import json
from typing import Any, Dict, Iterable, List, Mapping, Sequence

from .domain_models import CLASS_NAMES, ConfigurationError, NamedRegion, PROFILE_ROLE_POLICIES, VALID_PROFILE_ROLES, Waypoint


def class_schema_hash(class_names: Sequence[str]) -> str:
    import hashlib

    normalized = tuple(validate_dynamic_class_names(class_names, owner='class_schema_hash'))
    payload = json.dumps(list(normalized), ensure_ascii=False, separators=(',', ':'), sort_keys=False)
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()



def validate_embedded_class_schema(payload: Mapping[str, Any], *, expected_class_names: Sequence[str], owner: str) -> None:
    actual_names = validate_dynamic_class_names(payload.get('class_names', []), owner='{}.class_names'.format(owner))
    expected_names = validate_dynamic_class_names(expected_class_names, owner='{}.expected_class_names'.format(owner))
    actual_hash = str(payload.get('class_schema_hash', '')).strip()
    expected_hash = class_schema_hash(expected_names)
    if tuple(actual_names) != tuple(expected_names):
        raise ConfigurationError('{} class_names mismatch: expected {} got {}'.format(owner, expected_names, actual_names))
    if actual_hash and actual_hash != expected_hash:
        raise ConfigurationError('{} class_schema_hash mismatch: expected {} got {}'.format(owner, expected_hash, actual_hash))



def normalize_class_names(class_names: Iterable[str]) -> tuple[str, ...]:
    normalized = tuple(str(item).strip() for item in class_names if str(item).strip())
    if not normalized:
        raise ConfigurationError('class_names must not be empty')
    if len(set(normalized)) != len(normalized):
        raise ConfigurationError('class_names contains duplicates: {}'.format(normalized))
    return normalized



def ensure_legacy_class_schema(class_names: Iterable[str], *, owner: str) -> tuple[str, ...]:
    normalized = normalize_class_names(class_names)
    if tuple(normalized) != tuple(CLASS_NAMES):
        raise ConfigurationError('{} requires legacy class schema {}'.format(owner, CLASS_NAMES))
    return normalized



def validate_dynamic_class_names(class_names: Iterable[str], *, owner: str) -> tuple[str, ...]:
    normalized = normalize_class_names(class_names)
    return normalized



def validate_profile_role(profile_role: str, *, owner: str) -> str:
    normalized = str(profile_role).strip().lower()
    if normalized not in VALID_PROFILE_ROLES:
        raise ConfigurationError('{} profile_role must be one of {}'.format(owner, ', '.join(VALID_PROFILE_ROLES)))
    return normalized



def profile_role_policy(profile_role: str, *, owner: str) -> Dict[str, object]:
    normalized = validate_profile_role(profile_role, owner=owner)
    return dict(PROFILE_ROLE_POLICIES[normalized])



def validate_profile_runtime_flags(
    profile_role: str,
    *,
    owner: str,
    lifecycle_managed: bool | None = None,
    auto_start: bool | None = None,
    require_route_frame_regions: bool | None = None,
) -> str:
    normalized = validate_profile_role(profile_role, owner=owner)
    policy = profile_role_policy(normalized, owner=owner)
    if lifecycle_managed is not None and bool(policy['lifecycle_managed']) and not bool(lifecycle_managed):
        raise ConfigurationError('{} lifecycle_managed={} violates {} policy'.format(owner, lifecycle_managed, normalized))
    if auto_start is not None and not bool(policy['allow_auto_start']) and bool(auto_start):
        raise ConfigurationError('{} auto_start={} violates {} policy'.format(owner, auto_start, normalized))
    if require_route_frame_regions is not None and bool(policy['require_route_binding']) and not bool(require_route_frame_regions):
        raise ConfigurationError('{} require_route_frame_regions={} violates {} policy'.format(owner, require_route_frame_regions, normalized))
    return normalized



def validate_named_region_contract(named_regions: Sequence[dict], *, expected_region_names: Sequence[str], require_named_regions: bool, owner: str = 'named_regions') -> None:
    normalized_expected = [str(item).strip() for item in expected_region_names if str(item).strip()]
    if require_named_regions and not named_regions:
        raise ConfigurationError('{} requires non-empty named_regions'.format(owner))
    seen = []
    for idx, region_payload in enumerate(named_regions or []):
        try:
            region = NamedRegion(
                name=str(region_payload['name']).strip(),
                x0=int(region_payload['x0']),
                y0=int(region_payload['y0']),
                x1=int(region_payload['x1']),
                y1=int(region_payload['y1']),
            )
        except Exception as exc:
            raise ConfigurationError('{} invalid named_regions[{}]: {}'.format(owner, idx, exc))
        if not region.name:
            raise ConfigurationError('{} named_regions[{}].name is empty'.format(owner, idx))
        if region.x1 <= region.x0 or region.y1 <= region.y0:
            raise ConfigurationError('{} named_regions[{}] has non-positive extent'.format(owner, idx))
        seen.append(region.name)
    if len(set(seen)) != len(seen):
        raise ConfigurationError('{} named_regions contains duplicates: {}'.format(owner, seen))
    if normalized_expected and set(seen) != set(normalized_expected):
        raise ConfigurationError('{} expected named_regions {} but got {}'.format(owner, sorted(normalized_expected), sorted(seen)))



def coerce_class_count_mapping(counts: Mapping[str, Any], class_names: Sequence[str]) -> Dict[str, int]:
    normalized = validate_dynamic_class_names(class_names, owner='coerce_class_count_mapping.class_names')
    payload = {str(key).strip(): int(value) for key, value in dict(counts or {}).items() if str(key).strip()}
    return {name: int(payload.get(name, 0)) for name in normalized}



def flatten_count_matrix(region_counts: Mapping[str, Mapping[str, Any]], class_names: Sequence[str]) -> tuple[list[str], list[int]]:
    normalized_class_names = validate_dynamic_class_names(class_names, owner='flatten_count_matrix.class_names')
    region_names = [str(name).strip() for name in region_counts.keys() if str(name).strip()]
    counts_flat: List[int] = []
    for region_name in region_names:
        counts = coerce_class_count_mapping(region_counts.get(region_name, {}), normalized_class_names)
        counts_flat.extend(int(counts[name]) for name in normalized_class_names)
    return region_names, counts_flat



def unflatten_count_matrix(region_names: Sequence[str], class_names: Sequence[str], counts_flat: Sequence[Any]) -> Dict[str, Dict[str, int]]:
    normalized_class_names = validate_dynamic_class_names(class_names, owner='unflatten_count_matrix.class_names')
    normalized_region_names = [str(name).strip() for name in region_names if str(name).strip()]
    expected = len(normalized_region_names) * len(normalized_class_names)
    flattened = [int(value) for value in counts_flat]
    if len(flattened) != expected:
        raise ConfigurationError('counts_flat length {} does not match region/class matrix {}'.format(len(flattened), expected))
    restored: Dict[str, Dict[str, int]] = {}
    idx = 0
    for region_name in normalized_region_names:
        restored[region_name] = {}
        for class_name in normalized_class_names:
            restored[region_name][class_name] = int(flattened[idx])
            idx += 1
    return restored

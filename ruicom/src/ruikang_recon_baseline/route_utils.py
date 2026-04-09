"""Route and waypoint helpers."""

from __future__ import annotations

from collections import Counter
from typing import Mapping, Sequence

from .domain_models import ConfigurationError, Waypoint
from .runtime_paths import require_positive_float


def normalize_route_id(name: str, ordinal: int, total_for_name: int) -> str:
    base = str(name).strip()
    if not base:
        raise ConfigurationError('route waypoint name is empty')
    if total_for_name <= 1:
        return base
    if ordinal <= 0:
        raise ConfigurationError('route ordinal must be >= 1')
    return '{}__{}'.format(base, ordinal)



def load_waypoints(route_payload: Sequence[Mapping], dwell_default_sec: float | None = None, **kwargs) -> list[Waypoint]:
    if dwell_default_sec is None:
        dwell_default_sec = kwargs.pop('dwell_default', None)
    elif 'dwell_default' in kwargs:
        raise TypeError('use dwell_default_sec or dwell_default, not both')
    if dwell_default_sec is None:
        raise TypeError('dwell_default_sec is required')
    if kwargs:
        unexpected = ', '.join(sorted(kwargs.keys()))
        raise TypeError('unexpected keyword arguments: {}'.format(unexpected))
    if not route_payload:
        raise ConfigurationError('route must not be empty')
    name_totals = Counter(str(item.get('name', '')).strip() for item in route_payload if str(item.get('name', '')).strip())
    name_ordinals: Counter[str] = Counter()
    waypoints = []
    for idx, payload in enumerate(route_payload):
        name = str(payload.get('name', '')).strip()
        if not name:
            raise ConfigurationError('route[{}].name is empty'.format(idx))
        dwell_sec = float(payload.get('dwell_sec', dwell_default_sec))
        timeout_sec = float(payload.get('timeout_sec', dwell_sec + 5.0))
        route_id = str(payload.get('route_id', '')).strip()
        name_ordinals[name] += 1
        if not route_id:
            route_id = normalize_route_id(name, name_ordinals[name], name_totals[name])
        waypoint = Waypoint(
            name=name,
            x=float(payload['x']),
            y=float(payload['y']),
            yaw_deg=float(payload.get('yaw_deg', 0.0)),
            dwell_sec=require_positive_float('route[{}].dwell_sec'.format(idx), dwell_sec),
            timeout_sec=require_positive_float('route[{}].timeout_sec'.format(idx), timeout_sec),
            frame_region=str(payload.get('frame_region', '')).strip(),
            route_id=route_id,
            goal_frame=str(payload.get('goal_frame', 'map')).strip() or 'map',
        )
        waypoints.append(waypoint)
    if len({item.route_id for item in waypoints}) != len(waypoints):
        raise ConfigurationError('route contains duplicate route_id values')
    return waypoints



def validate_route_frame_region_contract(route: Sequence[Waypoint | Mapping], *, require_binding: bool, allowed_frame_regions: Sequence[str], owner: str = 'route') -> tuple[str, ...]:
    normalized_allowed = [str(item).strip() for item in allowed_frame_regions if str(item).strip()]
    seen_regions = []
    for idx, waypoint in enumerate(route):
        frame_region = getattr(waypoint, 'frame_region', None)
        if frame_region is None and isinstance(waypoint, Mapping):
            frame_region = str(waypoint.get('frame_region', '')).strip()
        frame_region = str(frame_region or '').strip()
        if require_binding and not frame_region:
            raise ConfigurationError('{} route[{}] is missing frame_region'.format(owner, idx))
        if frame_region:
            seen_regions.append(frame_region)
            if normalized_allowed and frame_region not in normalized_allowed:
                raise ConfigurationError('{} route[{}].frame_region {} is not in expected set {}'.format(owner, idx, frame_region, normalized_allowed))
    if require_binding and normalized_allowed and set(seen_regions) != set(normalized_allowed):
        raise ConfigurationError('{} route frame_region set {} does not match expected {}'.format(owner, sorted(set(seen_regions)), sorted(set(normalized_allowed))))
    return tuple(seen_regions)

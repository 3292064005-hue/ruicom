"""Competition task-graph DSL loader.

The DSL keeps competition mission authoring out of giant inline ROS params while
still lowering into the existing MissionPlan task specification surface.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence

import yaml

from .common import ConfigurationError, Waypoint
from .mission_plan import LEGACY_TASK_DEFAULT, SUPPORTED_TASK_TYPES
from .runtime_paths import expand_path, resolve_package_relative_path

_VALID_FILE_FORMATS = ('yaml', 'json')
_SUPPORTED_DSL_VERSIONS = (1,)


def _resolve_path(path: str) -> Path:
    resolved = resolve_package_relative_path(path)
    if resolved:
        return Path(resolved)
    expanded = Path(expand_path(path)).resolve()
    if not expanded.exists():
        raise ConfigurationError(f'mission task_dsl_path does not exist: {expanded}')
    return expanded


def _load_mapping(path: Path, file_format: str) -> Mapping[str, Any]:
    if file_format == 'json':
        payload = json.loads(path.read_text(encoding='utf-8'))
    else:
        payload = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(payload, Mapping):
        raise ConfigurationError(f'mission task DSL {path} must be a mapping')
    return payload


def _waypoint_lookup(route: Sequence[Waypoint]) -> Dict[str, Waypoint]:
    lookup: Dict[str, Waypoint] = {}
    for waypoint in route:
        for key in (waypoint.route_id, waypoint.name):
            normalized = str(key).strip()
            if normalized and normalized not in lookup:
                lookup[normalized] = waypoint
    return lookup


def _waypoint_to_mapping(waypoint: Waypoint) -> Dict[str, Any]:
    return {
        'name': waypoint.name,
        'x': waypoint.x,
        'y': waypoint.y,
        'yaw_deg': waypoint.yaw_deg,
        'goal_frame': waypoint.goal_frame,
        'dwell_sec': waypoint.dwell_sec,
        'timeout_sec': waypoint.timeout_sec,
        'frame_region': waypoint.frame_region,
        'route_id': waypoint.route_id,
    }


def load_task_graph_dsl(*, path: str, route: Sequence[Waypoint], file_format: str = 'yaml') -> List[Dict[str, Any]]:
    """Load one task-graph DSL file and lower it into task specs.

    Args:
        path: DSL file path.
        route: Already-resolved authoritative route. ``waypoint_ref`` nodes are
            bound against this route by ``route_id`` first and then by name.
        file_format: ``yaml`` or ``json``.

    Returns:
        List of task specifications consumable by :meth:`MissionPlan.from_task_specs`.

    Raises:
        ConfigurationError: If the DSL is malformed or references unknown
            waypoints.
    """
    normalized_format = str(file_format or 'yaml').strip().lower() or 'yaml'
    if normalized_format not in _VALID_FILE_FORMATS:
        raise ConfigurationError(f'mission task DSL format must be one of {", ".join(_VALID_FILE_FORMATS)}')
    resolved = _resolve_path(path)
    payload = _load_mapping(resolved, normalized_format)
    dsl_version = int(payload.get('dsl_version', 1) or 1)
    if dsl_version not in _SUPPORTED_DSL_VERSIONS:
        raise ConfigurationError(
            f'mission task DSL {resolved} uses unsupported dsl_version {dsl_version}; supported versions are {_SUPPORTED_DSL_VERSIONS}'
        )
    nodes = list(payload.get('nodes', []) or [])
    if not nodes:
        raise ConfigurationError(f'mission task DSL {resolved} must define non-empty nodes')
    lookup = _waypoint_lookup(route)
    task_specs: List[Dict[str, Any]] = []
    node_ids = set()
    for index, raw_node in enumerate(nodes):
        if not isinstance(raw_node, Mapping):
            raise ConfigurationError(f'mission task DSL nodes[{index}] must be a mapping')
        node_id = str(raw_node.get('id', '')).strip() or f'node_{index}'
        if node_id in node_ids:
            raise ConfigurationError(f'mission task DSL contains duplicate node id {node_id}')
        node_ids.add(node_id)
        task_type = str(raw_node.get('type', raw_node.get('task_type', LEGACY_TASK_DEFAULT))).strip() or LEGACY_TASK_DEFAULT
        if task_type not in SUPPORTED_TASK_TYPES:
            raise ConfigurationError(f'mission task DSL node {node_id} uses unsupported task type {task_type}')
        waypoint_ref = str(raw_node.get('waypoint_ref', '')).strip()
        inline_waypoint = raw_node.get('waypoint', {})
        if waypoint_ref:
            if inline_waypoint not in ({}, None, ''):
                raise ConfigurationError(f'mission task DSL node {node_id} must not define both waypoint_ref and inline waypoint')
            if waypoint_ref not in lookup:
                raise ConfigurationError(f'mission task DSL node {node_id} references unknown waypoint_ref {waypoint_ref}')
            waypoint_payload = _waypoint_to_mapping(lookup[waypoint_ref])
        elif isinstance(inline_waypoint, Mapping) and inline_waypoint:
            waypoint_payload = dict(inline_waypoint)
        else:
            raise ConfigurationError(f'mission task DSL node {node_id} requires waypoint_ref or waypoint')
        transitions = raw_node.get('transitions', {}) or {}
        if not isinstance(transitions, Mapping):
            raise ConfigurationError(f'mission task DSL node {node_id} transitions must be a mapping')
        outcome_edges = {
            str(key).strip().lower(): str(value).strip()
            for key, value in dict(transitions).items()
            if str(key).strip() and str(value).strip()
        }
        metadata = raw_node.get('metadata', {}) or {}
        if not isinstance(metadata, Mapping):
            raise ConfigurationError(f'mission task DSL node {node_id} metadata must be a mapping')
        task_specs.append({
            'step_id': node_id,
            'task_type': task_type,
            'objective_type': str(raw_node.get('objective_type', '')).strip(),
            'retry_limit': raw_node.get('retry_limit', None),
            'quiesce_sec': raw_node.get('quiesce_sec', None),
            'next_step_id': str(raw_node.get('next_step_id', '')).strip(),
            'failure_step_id': str(raw_node.get('failure_step_id', '')).strip(),
            'outcome_edges': outcome_edges,
            'metadata': dict(metadata),
            'waypoint': waypoint_payload,
        })
    return task_specs

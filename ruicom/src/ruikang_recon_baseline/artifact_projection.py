"""Projection helpers for dynamic/legacy compatibility lanes."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence

from .domain_models import CLASS_NAMES, ConfigurationError
from .schema_utils import class_schema_hash, coerce_class_count_mapping, validate_dynamic_class_names


def route_result_key(payload: Mapping[str, Any]) -> str:
    route_id = str(payload.get('route_id', '')).strip()
    if route_id:
        return route_id
    zone_name = str(payload.get('zone_name', '')).strip()
    if zone_name:
        return zone_name
    raise ConfigurationError('route_result_key requires route_id or zone_name')



def project_dynamic_zone_results(zone_results_dynamic: Mapping[str, Mapping[str, Any]]) -> Dict[str, dict]:
    projected: Dict[str, dict] = {}
    for key, payload in dict(zone_results_dynamic or {}).items():
        projected[str(key)] = dynamic_zone_result_to_legacy_payload(payload)
    return projected



def legacy_zone_result_to_dynamic_counts(payload: Mapping[str, Any], class_names: Sequence[str]) -> Dict[str, int]:
    normalized = validate_dynamic_class_names(class_names, owner='legacy_zone_result_to_dynamic_counts.class_names')
    if payload.get('class_names') and payload.get('class_counts'):
        payload_names = validate_dynamic_class_names(payload.get('class_names', ()), owner='legacy_zone_result_to_dynamic_counts.payload_class_names')
        counts_from_payload = {name: int(value) for name, value in zip(payload_names, payload.get('class_counts', ())) }
        return coerce_class_count_mapping(counts_from_payload, normalized)
    raw_counts = {
        'friendly': int(payload.get('friendly', 0)),
        'enemy': int(payload.get('enemy', 0)),
        'hostage': int(payload.get('hostage', 0)),
    }
    if 'counts' in payload and isinstance(payload['counts'], dict):
        raw_counts.update({str(key).strip(): int(value) for key, value in payload['counts'].items() if str(key).strip()})
    return coerce_class_count_mapping(raw_counts, normalized)



def zone_capture_payload_to_dynamic_payload(payload: Mapping[str, Any], class_names: Sequence[str]) -> Dict[str, Any]:
    normalized_class_names = validate_dynamic_class_names(class_names, owner='zone_capture_payload_to_dynamic_payload.class_names')
    counts = legacy_zone_result_to_dynamic_counts(payload, normalized_class_names)
    normalized_route_id = str(payload.get('route_id', '')).strip()
    return {
        'zone_name': str(payload.get('zone_name', '')).strip(),
        'route_id': normalized_route_id,
        'status': str(payload.get('status', '')).strip(),
        'class_names': list(normalized_class_names),
        'class_counts': [int(counts[name]) for name in normalized_class_names],
        'counts': counts,
        'capture_started_at': float(payload.get('capture_started_at', 0.0) or 0.0),
        'capture_finished_at': float(payload.get('capture_finished_at', 0.0) or 0.0),
        'frame_count': int(payload.get('frame_count', 0) or 0),
        'frame_region': str(payload.get('frame_region', '')).strip(),
        'failure_reason': str(payload.get('failure_reason', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip() or '2.0.0',
        'class_schema_hash': class_schema_hash(normalized_class_names),
        'task_type': str(payload.get('task_type', '')).strip() or 'waypoint_capture',
        'objective_type': str(payload.get('objective_type', '')).strip() or 'recon',
        'mission_outcome': str(payload.get('mission_outcome', '')).strip(),
        'task_metadata': dict(payload.get('task_metadata', {}) or {}),
        'position_estimates': [dict(item) for item in list(payload.get('position_estimates', []) or [])],
        'evidence_summary': dict(payload.get('evidence_summary', {}) or {}),
        'hazard_summary': dict(payload.get('hazard_summary', {}) or {}),
        'action_summary': dict(payload.get('action_summary', {}) or {}),
    }



def dynamic_zone_result_to_legacy_payload(payload: Mapping[str, Any]) -> Dict[str, Any]:
    dynamic_class_names = validate_dynamic_class_names(payload.get('class_names', list(CLASS_NAMES)), owner='dynamic_zone_result_to_legacy_payload.class_names')
    counts = coerce_class_count_mapping(
        payload.get('counts', dict(zip(dynamic_class_names, payload.get('class_counts', [])))),
        dynamic_class_names,
    )
    projected_counts = coerce_class_count_mapping(counts, CLASS_NAMES)
    return {
        'zone_name': str(payload.get('zone_name', '')).strip(),
        'route_id': str(payload.get('route_id', '')).strip(),
        'status': str(payload.get('status', '')).strip(),
        'friendly': int(projected_counts.get('friendly', 0)),
        'enemy': int(projected_counts.get('enemy', 0)),
        'hostage': int(projected_counts.get('hostage', 0)),
        'capture_started_at': float(payload.get('capture_started_at', 0.0) or 0.0),
        'capture_finished_at': float(payload.get('capture_finished_at', 0.0) or 0.0),
        'frame_count': int(payload.get('frame_count', 0) or 0),
        'frame_region': str(payload.get('frame_region', '')).strip(),
        'failure_reason': str(payload.get('failure_reason', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip() or '2.0.0',
        'class_names': list(dynamic_class_names),
        'class_counts': [int(counts[name]) for name in dynamic_class_names],
        'class_schema_hash': class_schema_hash(dynamic_class_names),
        'counts': counts,
        'task_type': str(payload.get('task_type', '')).strip() or 'waypoint_capture',
        'objective_type': str(payload.get('objective_type', '')).strip() or 'recon',
        'mission_outcome': str(payload.get('mission_outcome', '')).strip(),
        'task_metadata': dict(payload.get('task_metadata', {}) or {}),
        'position_estimates': [dict(item) for item in list(payload.get('position_estimates', []) or [])],
        'evidence_summary': dict(payload.get('evidence_summary', {}) or {}),
        'hazard_summary': dict(payload.get('hazard_summary', {}) or {}),
        'action_summary': dict(payload.get('action_summary', {}) or {}),
    }

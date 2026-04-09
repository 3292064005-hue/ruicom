"""Pure-Python helpers for recorder lane arbitration, schema validation and snapshot policies."""

from __future__ import annotations

import hashlib
import json
from copy import deepcopy
from typing import Any, Dict, Iterable, Mapping, Sequence

from .common import (
    CLASS_NAMES,
    ConfigurationError,
    class_schema_hash,
    coerce_class_count_mapping,
    dynamic_zone_result_to_legacy_payload,
    route_result_key,
    unflatten_count_matrix,
    validate_dynamic_class_names,
    validate_embedded_class_schema,
    zone_capture_payload_to_dynamic_payload,
)
from .system_manager_core import is_operator_control_mode


VALID_RECORDER_INPUT_MODES = ('json', 'typed', 'auto')

def build_initial_operator_audit_state() -> Dict[str, Any]:
    """Create one mutable operator-intervention audit accumulator."""
    return {
        'event_count': 0,
        'manual_command_count': 0,
        'manual_commands': [],
        'manual_takeover_count': 0,
        'manual_takeovers': [],
        'control_mode_transitions': [],
        'estop_event_count': 0,
        'estop_events': [],
        'last_control_mode': '',
        'estop_active': False,
    }


def _append_capped(items: list, payload: Mapping[str, Any], *, limit: int) -> None:
    items.append(dict(payload))
    if len(items) > max(1, int(limit)):
        del items[0:len(items) - max(1, int(limit))]


def record_runtime_evidence_operator_audit(audit: Mapping[str, Any], payload: Mapping[str, Any], *, history_limit: int = 32) -> Dict[str, Any]:
    """Update operator-intervention audit state from one runtime-evidence payload."""
    state = dict(audit or {})
    state.setdefault('event_count', 0)
    state.setdefault('manual_command_count', 0)
    state.setdefault('manual_commands', [])
    state.setdefault('manual_takeover_count', 0)
    state.setdefault('manual_takeovers', [])
    state.setdefault('control_mode_transitions', [])
    state.setdefault('estop_event_count', 0)
    state.setdefault('estop_events', [])
    state.setdefault('last_control_mode', '')
    state.setdefault('estop_active', False)
    details = dict(payload.get('details', {}) or {}) if isinstance(payload, Mapping) else {}
    event_type = str(payload.get('event_type', '')).strip().lower() if isinstance(payload, Mapping) else ''
    stamp = float(payload.get('stamp', 0.0) or 0.0) if isinstance(payload, Mapping) else 0.0
    state['event_count'] = int(state.get('event_count', 0)) + 1

    if event_type == 'manual_command':
        state['manual_command_count'] = int(state.get('manual_command_count', 0)) + 1
        _append_capped(state['manual_commands'], {
            'stamp': stamp,
            'command': str(payload.get('command', '') or details.get('command', '')).strip().lower(),
            'reason': str(payload.get('reason', '') or details.get('reason', '')).strip(),
            'accepted': bool(payload.get('accepted', details.get('accepted', False))),
            'resulting_state': str(payload.get('resulting_state', '') or details.get('resulting_state', '')).strip().upper(),
        }, limit=history_limit)

    current_control_mode = str(payload.get('current_control_mode', '') or payload.get('control_mode', '') or details.get('control_mode', '')).strip().upper()
    previous_control_mode = str(payload.get('previous_control_mode', '') or details.get('previous_control_mode', '')).strip().upper()
    if current_control_mode:
        last_control_mode = str(state.get('last_control_mode', '')).strip().upper()
        transition_source = previous_control_mode or last_control_mode
        if event_type == 'control_mode_changed' or current_control_mode != last_control_mode:
            manual_takeover = bool(payload.get('manual_takeover', details.get('manual_takeover', False))) or is_operator_control_mode(current_control_mode)
            transition = {
                'stamp': stamp,
                'previous_control_mode': transition_source,
                'current_control_mode': current_control_mode,
                'manual_takeover': manual_takeover,
            }
            _append_capped(state['control_mode_transitions'], transition, limit=history_limit)
            if manual_takeover:
                state['manual_takeover_count'] = int(state.get('manual_takeover_count', 0)) + 1
                _append_capped(state['manual_takeovers'], transition, limit=history_limit)
        state['last_control_mode'] = current_control_mode

    estop_payload_present = isinstance(payload, Mapping) and ('estop_active' in payload or event_type == 'estop_changed' or 'estop_active' in details)
    if estop_payload_present:
        active = bool(payload.get('estop_active', details.get('estop_active', state.get('estop_active', False))))
        if event_type == 'estop_changed' or active != bool(state.get('estop_active', False)):
            state['estop_event_count'] = int(state.get('estop_event_count', 0)) + 1
            _append_capped(state['estop_events'], {
                'stamp': stamp,
                'estop_active': active,
                'source': str(payload.get('source', '') or details.get('source', '')).strip(),
            }, limit=history_limit)
        state['estop_active'] = active
    return state


def summarize_operator_audit(audit: Mapping[str, Any]) -> Dict[str, Any]:
    """Project one mutable operator audit accumulator into artifact-safe data."""
    state = dict(audit or {})
    last_control_mode = str(state.get('last_control_mode', '')).strip().upper()
    estop_active = bool(state.get('estop_active', False))
    return {
        'event_count': int(state.get('event_count', 0)),
        'manual_command_count': int(state.get('manual_command_count', 0)),
        'manual_commands': [dict(item) for item in list(state.get('manual_commands', []) or [])],
        'manual_takeover_count': int(state.get('manual_takeover_count', 0)),
        'manual_takeovers': [dict(item) for item in list(state.get('manual_takeovers', []) or [])],
        'control_mode_transitions': [dict(item) for item in list(state.get('control_mode_transitions', []) or [])],
        'last_control_mode': last_control_mode,
        'operator_control_active': is_operator_control_mode(last_control_mode),
        'estop_active': estop_active,
        'estop_event_count': int(state.get('estop_event_count', 0)),
        'estop_events': [dict(item) for item in list(state.get('estop_events', []) or [])],
    }


def build_recorder_health_payload(
    *,
    stamp_sec: float,
    runtime_state: str,
    lifecycle_managed: bool,
    status: str,
    message: str,
    details: Mapping[str, Any] | None = None,
    node_name: str = 'mission_recorder_node',
) -> Dict[str, Any]:
    """Build the recorder-local health payload using business-time semantics.

    Args:
        stamp_sec: Business timestamp already resolved from the selected clock.
        runtime_state: Lifecycle/runtime state to publish.
        lifecycle_managed: Whether the recorder expects supervisor commands.
        status: Health severity label.
        message: Machine-readable reason string.
        details: Optional additional diagnostics merged into the payload.
        node_name: Published node identity.

    Returns:
        Normalized JSON-serializable health payload.

    Raises:
        ConfigurationError: If ``status`` or ``message`` is empty.

    Boundary behavior:
        ``stamp_sec`` is preserved verbatim, including ``0.0`` during early
        ``use_sim_time`` bootstrap before a clock message arrives.
    """
    normalized_status = str(status).strip()
    normalized_message = str(message).strip()
    if not normalized_status:
        raise ConfigurationError('recorder health status must not be empty')
    if not normalized_message:
        raise ConfigurationError('recorder health message must not be empty')
    merged_details = dict(details or {})
    merged_details.setdefault('runtime_state', str(runtime_state).strip())
    merged_details.setdefault('lifecycle_managed', bool(lifecycle_managed))
    return {
        'stamp': float(stamp_sec),
        'node': str(node_name).strip() or 'mission_recorder_node',
        'status': normalized_status,
        'message': normalized_message,
        'schema_version': 'recon.v2',
        'details': merged_details,
    }


def ensure_json_mapping(payload: Any, *, topic_name: str) -> dict:
    """Validate that a decoded JSON payload is a dictionary.

    Args:
        payload: Decoded JSON value.
        topic_name: Human-readable source for diagnostics.

    Returns:
        The original payload when it is a dictionary.

    Raises:
        ConfigurationError: If the payload is not a JSON object.

    Boundary behavior:
        Structured but semantically invalid payloads are rejected by the more
        specific validators below.
    """
    if not isinstance(payload, dict):
        raise ConfigurationError('{} payload must be a JSON object'.format(topic_name))
    return payload


def _ensure_mapping_or_empty(value: Any, *, topic_name: str, field_name: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ConfigurationError('{} {} must be a JSON object'.format(topic_name, field_name))
    return value


def canonical_payload_hash(payload: Mapping[str, Any]) -> str:
    """Create a stable hash for lane mismatch comparisons.

    Args:
        payload: Arbitrary JSON-like mapping.

    Returns:
        SHA-256 hex digest for the canonical JSON encoding.

    Raises:
        TypeError: If the payload contains non-JSON-serializable values.

    Boundary behavior:
        Dictionaries are normalized by key ordering. Lists preserve original order.
    """
    text = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def validate_mission_state_payload(payload: Any, *, topic_name: str) -> dict:
    """Validate recorder mission-state payloads from either lane.

    Args:
        payload: Candidate mission-state mapping from JSON or typed transport.
        topic_name: Human-readable source name for diagnostics.

    Returns:
        Normalized mission-state payload with route identity and schema metadata.

    Raises:
        ConfigurationError: If required finite-state fields or embedded class-schema
            metadata are missing or invalid.

    Boundary behavior:
        ``current_route_id`` may be empty while the mission is idle or sitting in a
        dispatch-pending transition with no active ``current_zone`` yet. Once a
        concrete zone becomes active, the route identity must remain stable across
        lanes.
    """
    payload = ensure_json_mapping(payload, topic_name=topic_name)
    details = _ensure_mapping_or_empty(payload.get('details', {}), topic_name=topic_name, field_name='details')
    class_names = validate_dynamic_class_names(payload.get('class_names', ()), owner=topic_name + '.class_names')
    schema_hash = str(payload.get('class_schema_hash', '')).strip() or class_schema_hash(class_names)
    if schema_hash != class_schema_hash(class_names):
        raise ConfigurationError('{} class_schema_hash does not match class_names'.format(topic_name))
    normalized = {
        'stamp': float(payload.get('stamp', 0.0)),
        'state': str(payload.get('state', '')).strip(),
        'event': str(payload.get('event', '')).strip(),
        'route_index': int(payload.get('route_index', -1)),
        'route_total': int(payload.get('route_total', 0)),
        'current_zone': str(payload.get('current_zone', '')).strip(),
        'current_route_id': str(payload.get('current_route_id', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip(),
        'class_names': list(class_names),
        'class_schema_hash': schema_hash,
        'details': details,
    }
    if not normalized['state']:
        raise ConfigurationError('{} missing state'.format(topic_name))
    if not normalized['event']:
        raise ConfigurationError('{} missing event'.format(topic_name))
    if normalized['route_index'] < -1:
        raise ConfigurationError('{} route_index must be >= -1'.format(topic_name))
    if normalized['route_total'] < 0:
        raise ConfigurationError('{} route_total must be >= 0'.format(topic_name))
    if normalized['route_total'] and normalized['route_index'] >= normalized['route_total'] and normalized['state'] not in ('FINISHED', 'TIMEOUT', 'FAULT'):
        raise ConfigurationError('{} route_index {} out of range for route_total {}'.format(topic_name, normalized['route_index'], normalized['route_total']))
    if normalized['route_total'] > 0 and normalized['route_index'] >= 0 and normalized['current_zone'] and not normalized['current_route_id']:
        raise ConfigurationError('{} missing current_route_id while current_zone is active'.format(topic_name))
    return normalized

def validate_zone_capture_payload(payload: Any, *, topic_name: str) -> dict:
    """Validate recorder zone-capture payloads from either lane."""
    payload = ensure_json_mapping(payload, topic_name=topic_name)
    normalized = {
        'stamp': float(payload.get('stamp', 0.0)),
        'zone_name': str(payload.get('zone_name', '')).strip(),
        'route_id': str(payload.get('route_id', '')).strip(),
        'status': str(payload.get('status', '')).strip(),
        'friendly': int(payload.get('friendly', 0)),
        'enemy': int(payload.get('enemy', 0)),
        'hostage': int(payload.get('hostage', 0)),
        'capture_started_at': float(payload.get('capture_started_at', 0.0)),
        'capture_finished_at': float(payload.get('capture_finished_at', 0.0)),
        'frame_count': int(payload.get('frame_count', 0)),
        'frame_region': str(payload.get('frame_region', '')).strip(),
        'failure_reason': str(payload.get('failure_reason', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip(),
        'class_schema_hash': str(payload.get('class_schema_hash', '')).strip(),
        'task_type': str(payload.get('task_type', '')).strip() or 'waypoint_capture',
        'objective_type': str(payload.get('objective_type', '')).strip() or 'recon',
        'mission_outcome': str(payload.get('mission_outcome', '')).strip(),
        'task_metadata': dict(payload.get('task_metadata', {}) or {}),
        'position_estimates': [dict(item) for item in list(payload.get('position_estimates', []) or [])],
        'evidence_summary': dict(payload.get('evidence_summary', {}) or {}),
        'hazard_summary': dict(payload.get('hazard_summary', {}) or {}),
        'action_summary': dict(payload.get('action_summary', {}) or {}),
    }
    if not normalized['zone_name']:
        raise ConfigurationError('{} missing zone_name'.format(topic_name))
    normalized['route_id'] = normalized['route_id'] or normalized['zone_name']
    if not normalized['status']:
        raise ConfigurationError('{} missing status'.format(topic_name))
    if normalized['frame_count'] < 0:
        raise ConfigurationError('{} frame_count must be >= 0'.format(topic_name))
    for key in ('friendly', 'enemy', 'hostage'):
        if normalized[key] < 0:
            raise ConfigurationError('{} {} must be >= 0'.format(topic_name, key))
    return normalized

def validate_legacy_zone_capture_schema(payload: Any, expected_class_names: Sequence[str], *, topic_name: str) -> dict:
    """Validate legacy zone-capture schema metadata against the recorder contract.

    Args:
        payload: Normalized legacy zone-capture mapping.
        expected_class_names: Recorder-authoritative ordered class schema.
        topic_name: Human-readable source used in diagnostics.

    Returns:
        A normalized payload carrying the recorder-authoritative ``class_schema_hash``.

    Raises:
        ConfigurationError: If the payload omits ``class_schema_hash`` or declares a
            schema that differs from ``expected_class_names``.

    Boundary behavior:
        Legacy payloads still use fixed ``friendly/enemy/hostage`` count fields, but
        they must now declare the authoritative dynamic schema hash explicitly so the
        compatibility lane cannot silently drift from ``summary_snapshot_v2``.
    """
    normalized = validate_zone_capture_payload(payload, topic_name=topic_name)
    expected = tuple(validate_dynamic_class_names(expected_class_names, owner=topic_name + '.expected_class_names'))
    received_hash = str(normalized.get('class_schema_hash', '')).strip()
    if not received_hash:
        raise ConfigurationError('{} missing class_schema_hash'.format(topic_name))
    expected_hash = class_schema_hash(expected)
    if received_hash != expected_hash:
        raise ConfigurationError('{} class_schema_hash {} does not match expected {}'.format(topic_name, received_hash, expected_hash))
    normalized['class_schema_hash'] = expected_hash
    return normalized


def validate_zone_capture_dynamic_payload(payload: Any, *, topic_name: str) -> dict:
    """Validate a forward-compatible dynamic zone-capture payload."""
    payload = ensure_json_mapping(payload, topic_name=topic_name)
    class_names = validate_dynamic_class_names(payload.get('class_names', ()), owner=topic_name + '.class_names')
    schema_hash = str(payload.get('class_schema_hash', '')).strip() or class_schema_hash(class_names)
    if schema_hash != class_schema_hash(class_names):
        raise ConfigurationError('{} class_schema_hash does not match class_names'.format(topic_name))
    raw_counts_mapping = payload.get('counts', None)
    raw_counts_flat = payload.get('class_counts', None)
    if raw_counts_mapping is None and raw_counts_flat is None:
        raise ConfigurationError('{} missing counts/class_counts'.format(topic_name))
    counts_from_mapping = None
    counts_from_flat = None
    if raw_counts_mapping is not None:
        counts_from_mapping = coerce_class_count_mapping(raw_counts_mapping, class_names)
    if raw_counts_flat is not None:
        if not isinstance(raw_counts_flat, (list, tuple)):
            raise ConfigurationError('{} class_counts must be a sequence'.format(topic_name))
        if len(raw_counts_flat) != len(class_names):
            raise ConfigurationError('{} class_counts length does not match class_names'.format(topic_name))
        counts_from_flat = {name: int(raw_counts_flat[idx]) for idx, name in enumerate(class_names)}
    if counts_from_mapping is not None and counts_from_flat is not None and counts_from_mapping != counts_from_flat:
        raise ConfigurationError('{} counts and class_counts disagree'.format(topic_name))
    counts = counts_from_mapping if counts_from_mapping is not None else counts_from_flat
    normalized = {
        'stamp': float(payload.get('stamp', 0.0)),
        'zone_name': str(payload.get('zone_name', '')).strip(),
        'route_id': str(payload.get('route_id', '')).strip(),
        'status': str(payload.get('status', '')).strip(),
        'class_names': list(class_names),
        'counts': counts,
        'class_counts': [counts[name] for name in class_names],
        'capture_started_at': float(payload.get('capture_started_at', 0.0)),
        'capture_finished_at': float(payload.get('capture_finished_at', 0.0)),
        'frame_count': int(payload.get('frame_count', 0)),
        'frame_region': str(payload.get('frame_region', '')).strip(),
        'failure_reason': str(payload.get('failure_reason', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip(),
        'class_schema_hash': schema_hash,
        'task_type': str(payload.get('task_type', '')).strip() or 'waypoint_capture',
        'objective_type': str(payload.get('objective_type', '')).strip() or 'recon',
        'mission_outcome': str(payload.get('mission_outcome', '')).strip(),
        'task_metadata': dict(payload.get('task_metadata', {}) or {}),
        'position_estimates': [dict(item) for item in list(payload.get('position_estimates', []) or [])],
        'evidence_summary': dict(payload.get('evidence_summary', {}) or {}),
        'hazard_summary': dict(payload.get('hazard_summary', {}) or {}),
        'action_summary': dict(payload.get('action_summary', {}) or {}),
    }
    if not normalized['zone_name']:
        raise ConfigurationError('{} missing zone_name'.format(topic_name))
    normalized['route_id'] = normalized['route_id'] or normalized['zone_name']
    if not normalized['status']:
        raise ConfigurationError('{} missing status'.format(topic_name))
    if normalized['frame_count'] < 0:
        raise ConfigurationError('{} frame_count must be >= 0'.format(topic_name))
    return normalized

def validate_frame_region_counts_payload(payload: Any, *, topic_name: str) -> dict:
    """Validate recorder frame-region count payloads."""
    payload = ensure_json_mapping(payload, topic_name=topic_name)
    counts = payload.get('frame_region_counts', {})
    if counts is None:
        counts = {}
    if not isinstance(counts, dict):
        raise ConfigurationError('{} frame_region_counts must be a JSON object'.format(topic_name))
    class_names = validate_dynamic_class_names(payload.get('class_names', ()), owner=topic_name + '.class_names')
    schema_hash = str(payload.get('class_schema_hash', '')).strip() or class_schema_hash(class_names)
    if schema_hash != class_schema_hash(class_names):
        raise ConfigurationError('{} class_schema_hash does not match class_names'.format(topic_name))
    normalized_counts = {}
    seen_regions = set()
    for region_name, region_counts in counts.items():
        region_key = str(region_name).strip()
        if not region_key:
            raise ConfigurationError('{} frame_region_counts contains empty region name'.format(topic_name))
        if region_key in seen_regions:
            raise ConfigurationError('{} frame_region_counts contains duplicate region {}'.format(topic_name, region_key))
        if not isinstance(region_counts, dict):
            raise ConfigurationError('{} frame_region_counts[{}] must be a JSON object'.format(topic_name, region_key))
        normalized_region = coerce_class_count_mapping(region_counts, class_names)
        extra_keys = [str(key).strip() for key in region_counts.keys() if str(key).strip() and str(key).strip() not in class_names]
        if extra_keys:
            raise ConfigurationError('{} frame_region_counts[{}] has unknown classes {}'.format(topic_name, region_key, tuple(extra_keys)))
        for class_name, value in normalized_region.items():
            if int(value) < 0:
                raise ConfigurationError('{} frame_region_counts[{}][{}] must be >= 0'.format(topic_name, region_key, class_name))
        normalized_counts[region_key] = normalized_region
        seen_regions.add(region_key)
    return {
        'stamp': float(payload.get('stamp', 0.0)),
        'frame_id': str(payload.get('frame_id', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip(),
        'mode': str(payload.get('mode', '')).strip(),
        'class_names': list(class_names),
        'class_schema_hash': schema_hash,
        'frame_region_counts': normalized_counts,
    }

def validate_frame_region_counts_typed_payload(payload: Any, *, topic_name: str) -> dict:
    """Validate typed frame-region count payloads."""
    payload = ensure_json_mapping(payload, topic_name=topic_name)
    class_names = validate_dynamic_class_names(payload.get('class_names', ()), owner=topic_name + '.class_names')
    schema_hash = str(payload.get('class_schema_hash', '')).strip() or class_schema_hash(class_names)
    if schema_hash != class_schema_hash(class_names):
        raise ConfigurationError('{} class_schema_hash does not match class_names'.format(topic_name))
    counts = unflatten_count_matrix(
        payload.get('region_names', ()),
        class_names,
        payload.get('counts_flat', ()),
    )
    return {
        'stamp': float(payload.get('stamp', 0.0)),
        'frame_id': str(payload.get('frame_id', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip(),
        'mode': str(payload.get('mode', '')).strip(),
        'class_names': list(class_names),
        'class_schema_hash': schema_hash,
        'frame_region_counts': counts,
    }


def validate_health_payload(payload: Any, *, topic_name: str) -> dict:
    """Validate recorder health payloads."""
    payload = ensure_json_mapping(payload, topic_name=topic_name)
    details = _ensure_mapping_or_empty(payload.get('details', {}), topic_name=topic_name, field_name='details')
    return {
        'stamp': float(payload.get('stamp', 0.0)),
        'node': str(payload.get('node', 'unknown')).strip() or 'unknown',
        'status': str(payload.get('status', '')).strip(),
        'message': str(payload.get('message', '')).strip(),
        'schema_version': str(payload.get('schema_version', '')).strip(),
        'details': details,
    }


def validate_summary_class_names(class_names: Iterable[Any]) -> tuple[str, ...]:
    """Validate the recorder summary class ordering.

    Args:
        class_names: Candidate class-name sequence from configuration.

    Returns:
        The normalized tuple of class names.

    Raises:
        ConfigurationError: If the configured schema is empty or contains duplicates.

    Boundary behavior:
        The recorder now treats dynamic class orderings as authoritative for v2
        artifacts while legacy outputs remain deterministic projections into
        ``friendly/enemy/hostage``.
    """
    return validate_dynamic_class_names(class_names, owner='summary_schema')


def summarize_task_result_metrics(zone_results_dynamic: Mapping[str, Mapping[str, Any]]) -> Dict[str, Dict[str, Any]]:
    """Aggregate task-oriented recorder metrics from authoritative dynamic results.

    Args:
        zone_results_dynamic: Authoritative recorder zone/task ledger.

    Returns:
        Dictionary containing task totals plus extracted hazard and facility views.

    Raises:
        No explicit exception is raised. Missing optional task fields are projected
        onto conservative defaults so legacy payloads remain summarizable.

    Boundary behavior:
        Legacy route captures without task metadata are treated as recon tasks.
    """
    totals_by_task_type: Dict[str, int] = {}
    totals_by_objective: Dict[str, int] = {}
    hazards = []
    facilities = []
    for route_id, result in dict(zone_results_dynamic or {}).items():
        task_type = str(result.get('task_type', '')).strip() or 'waypoint_capture'
        objective_type = str(result.get('objective_type', '')).strip() or 'recon'
        totals_by_task_type[task_type] = totals_by_task_type.get(task_type, 0) + 1
        totals_by_objective[objective_type] = totals_by_objective.get(objective_type, 0) + 1
        hazard_summary = dict(result.get('hazard_summary', {}) or {})
        if hazard_summary:
            hazards.append({
                'route_id': route_id,
                'zone_name': str(result.get('zone_name', '')).strip(),
                'status': str(result.get('status', '')).strip(),
                **hazard_summary,
            })
        action_summary = dict(result.get('action_summary', {}) or {})
        if action_summary:
            facilities.append({
                'route_id': route_id,
                'zone_name': str(result.get('zone_name', '')).strip(),
                'status': str(result.get('status', '')).strip(),
                **action_summary,
            })
    return {
        'totals_by_task_type': totals_by_task_type,
        'totals_by_objective': totals_by_objective,
        'hazard_observations': hazards,
        'facility_actions': facilities,
    }


def build_summary_snapshot_payload(
    *,
    generated_at_wall: float,
    schema_version: str,
    mission_state: Mapping[str, Any] | None,
    current_zone: str,
    authoritative_input: str,
    typed_seen: Mapping[str, bool],
    zone_results: Mapping[str, Mapping[str, Any]],
    latest_frame_region_counts: Mapping[str, Mapping[str, Any]],
    last_health: Mapping[str, Mapping[str, Any]],
    terminal_state_seen: bool,
    operator_interventions: Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    """Build the legacy summary snapshot payload published by the recorder."""
    mission_state_payload = deepcopy(dict(mission_state or {}))
    zone_results_payload = deepcopy(dict(zone_results))
    latest_counts_payload = deepcopy(dict(latest_frame_region_counts))
    health_payload = deepcopy(dict(last_health))
    totals = {'friendly': 0, 'enemy': 0, 'hostage': 0}
    for route_id, result in list(zone_results_payload.items()):
        result.setdefault('route_id', route_id)
        for key in totals:
            totals[key] += int(result.get(key, 0))
    return {
        'generated_at_wall': float(generated_at_wall),
        'schema_version': str(schema_version).strip(),
        'final_state': str(mission_state_payload.get('state', 'UNKNOWN')).strip() or 'UNKNOWN',
        'route_index': int(mission_state_payload.get('route_index', -1)),
        'route_total': int(mission_state_payload.get('route_total', 0)),
        'current_zone': str(current_zone).strip(),
        'current_route_id': str(mission_state_payload.get('current_route_id', '')).strip(),
        'authoritative_input': str(authoritative_input).strip(),
        'typed_seen': dict(typed_seen),
        'mission_state': mission_state_payload,
        'zone_results': zone_results_payload,
        'latest_frame_region_counts': latest_counts_payload,
        'last_health': health_payload,
        'totals': totals,
        'terminal_state_seen': bool(terminal_state_seen),
    }

def build_summary_snapshot_v2_payload(
    *,
    generated_at_wall: float,
    schema_version: str,
    mission_state: Mapping[str, Any] | None,
    current_zone: str,
    authoritative_input: str,
    typed_seen: Mapping[str, bool],
    class_names: Sequence[str],
    zone_results_dynamic: Mapping[str, Mapping[str, Any]],
    latest_frame_region_counts: Mapping[str, Mapping[str, Any]],
    last_health: Mapping[str, Mapping[str, Any]],
    terminal_state_seen: bool,
    operator_interventions: Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    """Build the forward-compatible dynamic summary snapshot payload."""
    normalized_classes = list(validate_dynamic_class_names(class_names, owner='summary_snapshot_v2.class_names'))
    mission_state_payload = deepcopy(dict(mission_state or {}))
    dynamic_results_payload = deepcopy(dict(zone_results_dynamic))
    latest_counts_payload = deepcopy(dict(latest_frame_region_counts))
    health_payload = deepcopy(dict(last_health))
    totals = {name: 0 for name in normalized_classes}
    for route_id, result in list(dynamic_results_payload.items()):
        counts = coerce_class_count_mapping(result.get('counts', {}), normalized_classes)
        result['route_id'] = result.get('route_id') or route_id
        result['class_names'] = list(normalized_classes)
        result['class_schema_hash'] = class_schema_hash(normalized_classes)
        result['counts'] = counts
        result['class_counts'] = [counts[name] for name in normalized_classes]
        dynamic_results_payload[route_id] = result
        for name in normalized_classes:
            totals[name] += counts[name]
    task_metrics = summarize_task_result_metrics(dynamic_results_payload)
    return {
        'generated_at_wall': float(generated_at_wall),
        'schema_version': str(schema_version).strip(),
        'final_state': str(mission_state_payload.get('state', 'UNKNOWN')).strip() or 'UNKNOWN',
        'route_index': int(mission_state_payload.get('route_index', -1)),
        'route_total': int(mission_state_payload.get('route_total', 0)),
        'current_zone': str(current_zone).strip(),
        'current_route_id': str(mission_state_payload.get('current_route_id', '')).strip(),
        'authoritative_input': str(authoritative_input).strip(),
        'typed_seen': dict(typed_seen),
        'mission_state': mission_state_payload,
        'class_names': list(normalized_classes),
        'class_schema_hash': class_schema_hash(normalized_classes),
        'zone_results_dynamic': dynamic_results_payload,
        'task_results_dynamic': deepcopy(dynamic_results_payload),
        'latest_frame_region_counts': latest_counts_payload,
        'last_health': health_payload,
        'totals_by_class': totals,
        'totals_by_task_type': dict(task_metrics['totals_by_task_type']),
        'totals_by_objective': dict(task_metrics['totals_by_objective']),
        'hazard_observations': list(task_metrics['hazard_observations']),
        'facility_actions': list(task_metrics['facility_actions']),
        'operator_interventions': summarize_operator_audit(operator_interventions or {}),
        'terminal_state_seen': bool(terminal_state_seen),
    }

def should_accept_lane(recorder_input_mode: str, stream_name: str, typed_seen: Dict[str, bool], lane: str) -> bool:
    """Decide whether a lane is authoritative for a given recorder stream.

    Args:
        recorder_input_mode: One of ``json``, ``typed`` or ``auto``.
        stream_name: Logical stream key such as ``mission_state`` or ``zone_capture``.
        typed_seen: Mutable state tracking whether the typed lane has already been observed.
        lane: The candidate lane being evaluated.

    Returns:
        ``True`` when the candidate lane should update recorder state.

    Raises:
        ConfigurationError: If the configured mode or lane is unsupported.

    Boundary behavior:
        ``auto`` always prefers the typed lane once it has been observed for the
        corresponding stream, while still allowing JSON bootstrap before that point.
    """
    normalized_mode = str(recorder_input_mode).strip().lower()
    if normalized_mode not in VALID_RECORDER_INPUT_MODES:
        raise ConfigurationError('recorder_authoritative_input must be one of: {}'.format(', '.join(VALID_RECORDER_INPUT_MODES)))
    normalized_lane = str(lane).strip().lower()
    if normalized_lane not in ('json', 'typed'):
        raise ConfigurationError('lane must be json or typed, got {}'.format(lane))
    if normalized_mode == normalized_lane:
        return True
    if normalized_mode == 'auto':
        if normalized_lane == 'typed':
            return True
        return not bool(typed_seen.get(stream_name, False))
    return False


def should_accept_lane_for_source(recorder_input_mode: str, lane: str, typed_seen_for_source: bool) -> bool:
    """Decide authoritative input per producer/source rather than per stream.

    Args:
        recorder_input_mode: One of ``json``, ``typed`` or ``auto``.
        lane: Candidate lane name, ``json`` or ``typed``.
        typed_seen_for_source: Whether the typed lane has already been observed for
            this concrete producer/source.

    Returns:
        ``True`` when the candidate lane should update source-local recorder state.

    Raises:
        ConfigurationError: If the configured mode or lane is unsupported.

    Boundary behavior:
        ``auto`` still allows JSON bootstrap for a source until that same source has
        produced a typed payload, which prevents unrelated typed producers from
        shadowing JSON-only producers on a shared topic.
    """
    return should_accept_lane(recorder_input_mode, 'source', {'source': bool(typed_seen_for_source)}, lane)


def projected_zone_payloads_match(authoritative_dynamic_payload: Mapping[str, Any], candidate_legacy_payload: Mapping[str, Any]) -> bool:
    """Check whether a legacy payload matches the projection of a dynamic payload.

    Args:
        authoritative_dynamic_payload: Dynamic authoritative zone payload.
        candidate_legacy_payload: Legacy JSON/typed payload candidate.

    Returns:
        ``True`` when the legacy payload exactly matches the deterministic legacy
        projection of the dynamic authoritative payload.

    Raises:
        ConfigurationError: If the authoritative dynamic payload is malformed.

    Boundary behavior:
        Dynamic-only classes are ignored for the fixed legacy fields but remain part
        of the projected payload used by recorder-side consistency checks.
    """
    projected = dynamic_zone_result_to_legacy_payload(authoritative_dynamic_payload)
    return zone_capture_payloads_match(projected, candidate_legacy_payload)


def mission_state_payloads_match(authoritative_payload: Mapping[str, Any], candidate_payload: Mapping[str, Any]) -> bool:
    """Compare mission-state payloads using both fixed fields and details hash."""
    keys = ('state', 'event', 'route_index', 'route_total', 'current_zone', 'current_route_id', 'schema_version', 'class_schema_hash')
    if not all(authoritative_payload.get(key) == candidate_payload.get(key) for key in keys):
        return False
    authoritative_classes = tuple(authoritative_payload.get('class_names', ()) or ())
    candidate_classes = tuple(candidate_payload.get('class_names', ()) or ())
    if authoritative_classes != candidate_classes:
        return False
    authoritative_stamp = round(float(authoritative_payload.get('stamp', 0.0)), 6)
    candidate_stamp = round(float(candidate_payload.get('stamp', 0.0)), 6)
    if authoritative_stamp != candidate_stamp:
        return False
    return canonical_payload_hash(authoritative_payload.get('details', {})) == canonical_payload_hash(candidate_payload.get('details', {}))

def zone_capture_payloads_match(authoritative_payload: Mapping[str, Any], candidate_payload: Mapping[str, Any]) -> bool:
    """Compare zone-capture payloads using both legacy and dynamic count representations."""
    keys = (
        'zone_name',
        'route_id',
        'status',
        'friendly',
        'enemy',
        'hostage',
        'frame_count',
        'frame_region',
        'failure_reason',
        'schema_version',
        'class_schema_hash',
    )
    if not all(authoritative_payload.get(key) == candidate_payload.get(key) for key in keys):
        return False
    stamp_keys = ('stamp', 'capture_started_at', 'capture_finished_at')
    for key in stamp_keys:
        if round(float(authoritative_payload.get(key, 0.0)), 6) != round(float(candidate_payload.get(key, 0.0)), 6):
            return False
    authoritative_dynamic = zone_capture_payload_to_dynamic_payload(
        authoritative_payload,
        authoritative_payload.get('class_names', ()) or tuple(authoritative_payload.get('counts', {}).keys()) or tuple(CLASS_NAMES),
    )
    candidate_dynamic = zone_capture_payload_to_dynamic_payload(
        candidate_payload,
        authoritative_dynamic['class_names'],
    )
    return canonical_payload_hash({
        'route_id': authoritative_dynamic.get('route_id', ''),
        'class_names': tuple(authoritative_dynamic.get('class_names', ())),
        'class_counts': tuple(int(value) for value in authoritative_dynamic.get('class_counts', ()) or ()),
        'counts': authoritative_dynamic.get('counts', {}) or {},
        'class_schema_hash': authoritative_dynamic.get('class_schema_hash', ''),
    }) == canonical_payload_hash({
        'route_id': candidate_dynamic.get('route_id', ''),
        'class_names': tuple(candidate_dynamic.get('class_names', ())),
        'class_counts': tuple(int(value) for value in candidate_dynamic.get('class_counts', ()) or ()),
        'counts': candidate_dynamic.get('counts', {}) or {},
        'class_schema_hash': candidate_dynamic.get('class_schema_hash', ''),
    })

def snapshot_flush_allowed(snapshot_dirty: bool, state_changed: bool, flush_on_state_change_only: bool, force: bool = False) -> bool:
    """Determine whether the recorder should flush its derived snapshot.

    Args:
        snapshot_dirty: Whether any subscribed stream has changed since the last flush.
        state_changed: Whether the dirty state includes mission or zone transitions.
        flush_on_state_change_only: If true, suppress health/count-only flushes.
        force: Whether the flush was explicitly requested regardless of dirty state.

    Returns:
        ``True`` when the caller should publish a snapshot.

    Raises:
        No explicit exception is raised.

    Boundary behavior:
        Forced flushes always succeed even when no dirty state is present, which is
        required for shutdown and finalization paths.
    """
    if force:
        return True
    if not snapshot_dirty:
        return False
    if flush_on_state_change_only and not state_changed:
        return False
    return True

"""Public contract package for recorder and mission outputs."""

from .compatibility import should_accept_lane, should_accept_lane_for_source, snapshot_flush_allowed
from .frame_region_counts import validate_frame_region_counts_payload, validate_frame_region_counts_typed_payload
from .health import validate_health_payload
from .mission_state import mission_state_payloads_match, validate_mission_state_payload
from .schema import class_schema_hash, ensure_legacy_class_schema, validate_dynamic_class_names, validate_summary_class_names
from .zone_result import (
    dynamic_zone_result_to_legacy_payload,
    projected_zone_payloads_match,
    validate_legacy_zone_capture_schema,
    validate_zone_capture_dynamic_payload,
    validate_zone_capture_payload,
    zone_capture_payload_to_dynamic_payload,
    zone_capture_payloads_match,
)

__all__ = [
    'class_schema_hash',
    'ensure_legacy_class_schema',
    'validate_dynamic_class_names',
    'validate_summary_class_names',
    'validate_mission_state_payload',
    'mission_state_payloads_match',
    'validate_zone_capture_payload',
    'validate_zone_capture_dynamic_payload',
    'validate_legacy_zone_capture_schema',
    'zone_capture_payloads_match',
    'projected_zone_payloads_match',
    'dynamic_zone_result_to_legacy_payload',
    'zone_capture_payload_to_dynamic_payload',
    'validate_frame_region_counts_payload',
    'validate_frame_region_counts_typed_payload',
    'validate_health_payload',
    'should_accept_lane',
    'should_accept_lane_for_source',
    'snapshot_flush_allowed',
]

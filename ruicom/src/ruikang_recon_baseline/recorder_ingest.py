"""Recorder ingest pipeline for typed/JSON mission streams."""

from __future__ import annotations

import time

import rospy
from std_msgs.msg import String

from .common import ConfigurationError, JsonCodec, class_schema_hash, route_result_key, zone_capture_payload_to_dynamic_payload
from .contracts import (
    mission_state_payloads_match,
    should_accept_lane,
    should_accept_lane_for_source,
    validate_frame_region_counts_payload,
    validate_frame_region_counts_typed_payload,
    validate_health_payload,
    validate_legacy_zone_capture_schema,
    validate_mission_state_payload,
    validate_zone_capture_dynamic_payload,
)
from .recorder_core import ensure_json_mapping


class RecorderIngestPipeline:
    """Normalize, validate, and apply recorder input lanes."""

    def __init__(self, owner):
        self.owner = owner

    def safe_load_json(self, text: str, topic_name: str):
        """Decode a JSON message into a mapping with recorder-side diagnostics."""
        try:
            return ensure_json_mapping(JsonCodec.loads(text), topic_name=topic_name)
        except Exception as exc:
            self.owner._append('invalid_json', {'topic': topic_name, 'error': str(exc), 'raw': text[:512]})
            rospy.logwarn_throttle(2.0, '%s invalid JSON: %s', topic_name, exc)
            return None

    def mark_snapshot_dirty(self, *, state_changed: bool) -> None:
        self.owner.snapshot_dirty = True
        self.owner.snapshot_state_changed = self.owner.snapshot_state_changed or state_changed

    def record_compat_shadow(self, stream: str, payload: dict, *, reason: str = '') -> None:
        event_payload = {
            'stream': str(stream).strip(),
            'reason': str(reason).strip(),
            'payload': payload,
        }
        self.owner.compat_shadow_events += 1
        self.owner._append('compat_shadow', event_payload)

    def handle_class_schema_mismatch(self, stream: str, payload: dict, message: str) -> None:
        normalized_stream = str(stream).strip()
        event_payload = {
            'stream': normalized_stream,
            'message': str(message).strip(),
            'expected_class_names': list(self.owner.config['classes']),
            'expected_class_schema_hash': class_schema_hash(self.owner.config['classes']),
            'payload': payload,
        }
        self.owner.class_schema_mismatch_count += 1
        if self.owner.config['class_schema_mismatch_policy'] == 'error':
            self.owner.consistency_blocked = True
        self.owner._append('class_schema_mismatch', event_payload)
        rospy.logwarn_throttle(2.0, 'class schema mismatch on %s: %s', normalized_stream, message)

    def validate_payload_schema(self, payload: dict, *, stream: str) -> bool:
        received_names = tuple(payload.get('class_names', ()) or ())
        if not received_names:
            self.handle_class_schema_mismatch(stream, payload, 'missing class_names')
            return False
        received_hash = str(payload.get('class_schema_hash', '')).strip()
        expected_names = tuple(self.owner.config['classes'])
        expected_hash = class_schema_hash(expected_names)
        if received_names != expected_names:
            self.handle_class_schema_mismatch(stream, payload, 'received {} expected {}'.format(received_names, expected_names))
            return False
        if received_hash and received_hash != expected_hash:
            self.handle_class_schema_mismatch(stream, payload, 'received hash {} expected {}'.format(received_hash, expected_hash))
            return False
        payload['class_schema_hash'] = expected_hash
        payload['class_names'] = list(expected_names)
        return True

    def current_zone_cb(self, msg: String):
        with self.owner._lock:
            if not self.owner.runtime.processing_allowed:
                return
            new_zone = msg.data.strip()
            if new_zone != self.owner.current_zone:
                self.owner.current_zone = new_zone
                self.mark_snapshot_dirty(state_changed=True)

    def health_cb(self, msg: String):
        payload = self.safe_load_json(msg.data, self.owner.config['health_topic'])
        if payload is None:
            return
        try:
            payload = validate_health_payload(payload, topic_name=self.owner.config['health_topic'])
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {'topic': self.owner.config['health_topic'], 'error': str(exc), 'raw': payload})
            rospy.logwarn_throttle(2.0, '%s invalid payload: %s', self.owner.config['health_topic'], exc)
            return
        with self.owner._lock:
            node_name = payload.get('node', 'unknown')
            if not should_accept_lane_for_source(
                self.owner.config['recorder_authoritative_input'],
                'json',
                self.owner.health_typed_seen_by_node.get(node_name, False),
            ):
                return
            self.owner.last_health[node_name] = payload
            self.owner._append('health', payload)
            self.mark_snapshot_dirty(state_changed=False)


    def runtime_evidence_cb(self, msg: String):
        payload = self.safe_load_json(msg.data, self.owner.config['runtime_evidence_topic'])
        if payload is None:
            return
        with self.owner._lock:
            self.owner.operator_interventions = self.owner._update_operator_interventions(payload)
            self.owner._append('runtime_evidence', payload)
            self.mark_snapshot_dirty(state_changed=False)

    def health_typed_cb(self, msg):
        details = {}
        if msg.details_json:
            parsed = self.safe_load_json(msg.details_json, self.owner.config['health_typed_topic'] + '.details_json')
            if parsed is not None:
                details = parsed
        payload = {
            'stamp': msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else 0.0,
            'node': msg.node,
            'status': msg.status,
            'message': msg.message,
            'schema_version': msg.schema_version,
            'details': details,
        }
        try:
            payload = validate_health_payload(payload, topic_name=self.owner.config['health_typed_topic'])
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {'topic': self.owner.config['health_typed_topic'], 'error': str(exc)})
            rospy.logwarn_throttle(2.0, '%s invalid typed payload: %s', self.owner.config['health_typed_topic'], exc)
            return
        with self.owner._lock:
            node_name = payload.get('node', 'unknown')
            self.owner.typed_seen['health'] = True
            self.owner.health_typed_seen_by_node[node_name] = True
            self.owner.last_health[node_name] = payload
            self.owner._append('health_typed', payload)
            self.mark_snapshot_dirty(state_changed=False)

    def frame_region_counts_cb(self, msg: String):
        payload = self.safe_load_json(msg.data, self.owner.config['frame_region_counts_topic'])
        if payload is None:
            return
        try:
            payload = validate_frame_region_counts_payload(payload, topic_name=self.owner.config['frame_region_counts_topic'])
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {'topic': self.owner.config['frame_region_counts_topic'], 'error': str(exc), 'raw': payload})
            rospy.logwarn_throttle(2.0, '%s invalid payload: %s', self.owner.config['frame_region_counts_topic'], exc)
            return
        with self.owner._lock:
            if not self.owner.ingest_allowed():
                return
            if not self.validate_payload_schema(payload, stream='frame_region_counts_json'):
                return
            if self.owner.typed_seen['frame_region_counts'] and self.owner.config['recorder_authoritative_input'] == 'auto':
                return
            self.owner.latest_frame_region_counts = payload.get('frame_region_counts', {})
            self.owner._append('frame_region_counts', payload)
            self.mark_snapshot_dirty(state_changed=False)

    def frame_region_counts_typed_cb(self, msg):
        payload = {
            'stamp': msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else 0.0,
            'frame_id': msg.frame_id,
            'schema_version': msg.schema_version,
            'mode': msg.mode,
            'region_names': list(msg.region_names),
            'class_names': list(msg.class_names),
            'class_schema_hash': getattr(msg, 'class_schema_hash', ''),
            'counts_flat': list(msg.counts_flat),
        }
        try:
            payload = validate_frame_region_counts_typed_payload(payload, topic_name=self.owner.config['frame_region_counts_typed_topic'])
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {'topic': self.owner.config['frame_region_counts_typed_topic'], 'error': str(exc)})
            rospy.logwarn_throttle(2.0, '%s invalid typed payload: %s', self.owner.config['frame_region_counts_typed_topic'], exc)
            return
        with self.owner._lock:
            if not self.owner.ingest_allowed():
                return
            if not self.validate_payload_schema(payload, stream='frame_region_counts_typed'):
                return
            self.owner.typed_seen['frame_region_counts'] = True
            self.owner.latest_frame_region_counts = payload.get('frame_region_counts', {})
            self.owner._append('frame_region_counts_typed', payload)
            self.mark_snapshot_dirty(state_changed=False)

    def lane_authoritative(self, stream_name: str, lane: str) -> bool:
        return should_accept_lane(self.owner.config['recorder_authoritative_input'], stream_name, self.owner.typed_seen, lane)

    def normalize_mission_state_typed(self, msg) -> dict:
        details = {}
        if msg.details_json:
            parsed = self.safe_load_json(msg.details_json, self.owner.config['mission_state_typed_topic'] + '.details_json')
            if parsed is not None:
                details = parsed
        details.setdefault('event_id', int(msg.event_id))
        details.setdefault('zone_name', msg.zone_name)
        if getattr(msg, 'current_route_id', ''):
            details.setdefault('current_route_id', msg.current_route_id)
        if msg.next_zone:
            details.setdefault('next_zone', msg.next_zone)
        if getattr(msg, 'next_route_id', ''):
            details.setdefault('next_route_id', msg.next_route_id)
        if msg.failure_reason:
            details.setdefault('failure_reason', msg.failure_reason)
        if msg.frame_region:
            details.setdefault('frame_region', msg.frame_region)
        if int(msg.retry_count):
            details.setdefault('retry_count', int(msg.retry_count))
        if float(msg.duration_sec):
            details.setdefault('duration_sec', float(msg.duration_sec))
        if float(msg.elapsed_sec):
            details.setdefault('elapsed_sec', float(msg.elapsed_sec))
        if float(msg.capture_deadline):
            details.setdefault('capture_deadline', float(msg.capture_deadline))
        if float(msg.state_since):
            details.setdefault('state_since', float(msg.state_since))
        return validate_mission_state_payload({
            'stamp': msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else 0.0,
            'state': msg.state,
            'event': msg.event,
            'route_index': int(msg.route_index),
            'route_total': int(msg.route_total),
            'current_zone': msg.current_zone,
            'current_route_id': getattr(msg, 'current_route_id', ''),
            'schema_version': msg.schema_version,
            'class_names': list(getattr(msg, 'class_names', ())),
            'class_schema_hash': getattr(msg, 'class_schema_hash', ''),
            'details': details,
        }, topic_name=self.owner.config['mission_state_typed_topic'])

    def _decode_typed_json_object(self, raw: str, *, topic_name: str, field_name: str) -> dict:
        text = str(raw or '').strip()
        if not text:
            return {}
        value = self.safe_load_json(text, topic_name + '.' + field_name)
        if value is None:
            raise ConfigurationError('{} invalid JSON'.format(field_name))
        return ensure_json_mapping(value, topic_name=topic_name + '.' + field_name)

    def _decode_typed_json_list(self, raw: str, *, topic_name: str, field_name: str) -> list[dict]:
        text = str(raw or '').strip()
        if not text:
            return []
        try:
            value = JsonCodec.loads(text)
        except Exception as exc:
            self.owner._append('invalid_json', {'topic': topic_name + '.' + field_name, 'error': str(exc), 'raw': text[:512]})
            rospy.logwarn_throttle(2.0, '%s invalid JSON: %s', topic_name + '.' + field_name, exc)
            raise ConfigurationError('{} invalid JSON'.format(field_name))
        if not isinstance(value, list):
            raise ConfigurationError('{} must decode to a JSON array'.format(topic_name + '.' + field_name))
        normalized = []
        for index, item in enumerate(value):
            if not isinstance(item, dict):
                raise ConfigurationError('{}[{}] must be a JSON object'.format(topic_name + '.' + field_name, index))
            normalized.append(dict(item))
        return normalized

    def normalize_zone_capture_typed(self, msg) -> dict:
        return validate_legacy_zone_capture_schema({
            'stamp': msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else 0.0,
            'zone_name': msg.zone_name,
            'route_id': getattr(msg, 'route_id', ''),
            'status': msg.status,
            'friendly': int(msg.friendly),
            'enemy': int(msg.enemy),
            'hostage': int(msg.hostage),
            'capture_started_at': float(msg.capture_started_at),
            'capture_finished_at': float(msg.capture_finished_at),
            'frame_count': int(msg.frame_count),
            'frame_region': msg.frame_region,
            'failure_reason': msg.failure_reason,
            'schema_version': msg.schema_version,
            'class_schema_hash': getattr(msg, 'class_schema_hash', ''),
        }, self.owner.config['classes'], topic_name=self.owner.config['zone_capture_typed_topic'])

    def normalize_zone_capture_dynamic_typed(self, msg) -> dict:
        topic_name = self.owner.config['zone_capture_dynamic_topic']
        return validate_zone_capture_dynamic_payload({
            'stamp': msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else 0.0,
            'zone_name': msg.zone_name,
            'route_id': getattr(msg, 'route_id', ''),
            'status': msg.status,
            'class_names': list(msg.class_names),
            'class_counts': list(msg.class_counts),
            'counts': {name: int(value) for name, value in zip(msg.class_names, msg.class_counts)},
            'capture_started_at': float(msg.capture_started_at),
            'capture_finished_at': float(msg.capture_finished_at),
            'frame_count': int(msg.frame_count),
            'frame_region': msg.frame_region,
            'failure_reason': msg.failure_reason,
            'schema_version': msg.schema_version,
            'class_schema_hash': getattr(msg, 'class_schema_hash', ''),
            'task_type': getattr(msg, 'task_type', ''),
            'objective_type': getattr(msg, 'objective_type', ''),
            'mission_outcome': getattr(msg, 'mission_outcome', ''),
            'task_metadata': self._decode_typed_json_object(getattr(msg, 'task_metadata_json', ''), topic_name=topic_name, field_name='task_metadata_json'),
            'position_estimates': self._decode_typed_json_list(getattr(msg, 'position_estimates_json', ''), topic_name=topic_name, field_name='position_estimates_json'),
            'evidence_summary': self._decode_typed_json_object(getattr(msg, 'evidence_summary_json', ''), topic_name=topic_name, field_name='evidence_summary_json'),
            'hazard_summary': self._decode_typed_json_object(getattr(msg, 'hazard_summary_json', ''), topic_name=topic_name, field_name='hazard_summary_json'),
            'action_summary': self._decode_typed_json_object(getattr(msg, 'action_summary_json', ''), topic_name=topic_name, field_name='action_summary_json'),
        }, topic_name=topic_name)

    def apply_mission_state(self, payload: dict, *, lane: str) -> None:
        previous_state = str(self.owner.last_mission_state.get('state', ''))
        if not self.validate_payload_schema(payload, stream='mission_state_' + lane):
            return
        self.owner.last_mission_state = payload
        self.owner.current_route_id = str(payload.get('current_route_id', '')).strip()
        self.mark_snapshot_dirty(state_changed=True)
        state = str(payload.get('state', ''))
        if state != previous_state or lane == 'typed':
            self.mark_snapshot_dirty(state_changed=True)
        if state in ('FINISHED', 'TIMEOUT', 'FAULT'):
            self.owner.terminal_state_seen = True
            self.owner.terminal_state_payload = payload
            self.owner.last_terminal_wall_time = time.time()
            if self.owner.last_zone_capture_wall_time <= 0.0:
                self.owner.last_zone_capture_wall_time = self.owner.last_terminal_wall_time

    def apply_zone_capture(self, payload: dict, *, lane: str) -> None:
        """Record legacy zone-capture lanes as compatibility shadow input only.

        Legacy fixed-schema payloads are no longer allowed to mutate recorder
        authoritative runtime state. They remain subscribed for backwards
        compatibility and auditing, but the authoritative summary is derived only
        from the dynamic zone-capture lane.
        """
        zone_name = str(payload.get('zone_name', '')).strip()
        if not zone_name:
            return
        if self.owner.finalized and not self.owner.config['allow_final_rewrite_after_finalize']:
            self.owner._append('late_zone_capture_after_finalize', {'lane': lane, 'payload': payload})
            return
        self.record_compat_shadow('zone_capture_legacy', payload, reason='projection_only_lane')
        self.owner.last_zone_capture_wall_time = time.time()

    def apply_zone_capture_dynamic(self, payload: dict) -> None:
        zone_name = str(payload.get('zone_name', '')).strip()
        if not zone_name:
            return
        if not self.validate_payload_schema(payload, stream='zone_capture_dynamic'):
            return
        if self.owner.finalized and not self.owner.config['allow_final_rewrite_after_finalize']:
            self.owner._append('late_zone_capture_dynamic_after_finalize', {'payload': payload})
            return
        normalized_dynamic = zone_capture_payload_to_dynamic_payload(payload, self.owner.config['classes'])
        route_key = route_result_key(normalized_dynamic)
        previous_dynamic = self.owner.zone_results_dynamic.get(route_key)
        self.owner.zone_results_dynamic[route_key] = normalized_dynamic
        self.owner.last_zone_capture_wall_time = time.time()
        if previous_dynamic != normalized_dynamic:
            self.mark_snapshot_dirty(state_changed=True)
        if self.owner.finalized and self.owner.config['allow_final_rewrite_after_finalize'] and not self.owner.consistency_blocked:
            self.owner.finalize_controller.finalize_locked(self.owner, force=True, rewrite=True)

    def mission_state_json_cb(self, msg: String):
        payload = self.safe_load_json(msg.data, self.owner.config['mission_state_topic'])
        if payload is None:
            return
        try:
            payload = validate_mission_state_payload(payload, topic_name=self.owner.config['mission_state_topic'])
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {'topic': self.owner.config['mission_state_topic'], 'error': str(exc), 'raw': payload})
            rospy.logwarn_throttle(2.0, '%s invalid payload: %s', self.owner.config['mission_state_topic'], exc)
            return
        with self.owner._lock:
            if not self.owner.ingest_allowed():
                return
            self.owner._append('mission_state_json', payload)
            if self.lane_authoritative('mission_state', 'json'):
                self.apply_mission_state(payload, lane='json')
            elif self.owner.last_mission_state and not mission_state_payloads_match(self.owner.last_mission_state, payload):
                self.record_compat_shadow('mission_state', payload, reason='mission_state_lane_mismatch')
        self.owner._maybe_finalize(force=False)

    def zone_capture_json_cb(self, msg: String):
        payload = self.safe_load_json(msg.data, self.owner.config['zone_capture_topic'])
        if payload is None:
            return
        try:
            payload = validate_legacy_zone_capture_schema(payload, self.owner.config['classes'], topic_name=self.owner.config['zone_capture_topic'])
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {'topic': self.owner.config['zone_capture_topic'], 'error': str(exc), 'raw': payload})
            rospy.logwarn_throttle(2.0, '%s invalid payload: %s', self.owner.config['zone_capture_topic'], exc)
            return
        with self.owner._lock:
            if not self.owner.ingest_allowed():
                return
            self.owner._append('zone_capture_json', payload)
            self.apply_zone_capture(payload, lane='json')
        self.owner._maybe_finalize(force=False)

    def mission_state_typed_cb(self, msg):
        try:
            payload = self.normalize_mission_state_typed(msg)
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {
                'topic': self.owner.config['mission_state_typed_topic'],
                'error': str(exc),
                'raw': {'state': msg.state, 'event': msg.event, 'route_index': int(msg.route_index), 'route_total': int(msg.route_total), 'current_zone': msg.current_zone},
            })
            rospy.logwarn_throttle(2.0, '%s invalid typed payload: %s', self.owner.config['mission_state_typed_topic'], exc)
            return
        with self.owner._lock:
            if not self.owner.ingest_allowed():
                return
            self.owner.typed_seen['mission_state'] = True
            self.owner._append('mission_state_typed', payload)
            if self.lane_authoritative('mission_state', 'typed'):
                self.apply_mission_state(payload, lane='typed')
        self.owner._maybe_finalize(force=False)

    def zone_capture_typed_cb(self, msg):
        try:
            payload = self.normalize_zone_capture_typed(msg)
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {
                'topic': self.owner.config['zone_capture_typed_topic'],
                'error': str(exc),
                'raw': {
                    'zone_name': msg.zone_name, 'status': msg.status, 'friendly': int(msg.friendly), 'enemy': int(msg.enemy),
                    'hostage': int(msg.hostage), 'frame_count': int(msg.frame_count), 'frame_region': msg.frame_region,
                    'failure_reason': msg.failure_reason,
                },
            })
            rospy.logwarn_throttle(2.0, '%s invalid typed payload: %s', self.owner.config['zone_capture_typed_topic'], exc)
            return
        with self.owner._lock:
            if not self.owner.ingest_allowed():
                return
            self.owner.typed_seen['zone_capture'] = True
            self.owner._append('zone_capture_typed', payload)
            self.apply_zone_capture(payload, lane='typed')
        self.owner._maybe_finalize(force=False)

    def zone_capture_dynamic_typed_cb(self, msg):
        try:
            payload = self.normalize_zone_capture_dynamic_typed(msg)
        except (ConfigurationError, TypeError, ValueError) as exc:
            self.owner._append('invalid_payload', {'topic': self.owner.config['zone_capture_dynamic_topic'], 'error': str(exc)})
            rospy.logwarn_throttle(2.0, '%s invalid dynamic typed payload: %s', self.owner.config['zone_capture_dynamic_topic'], exc)
            return
        with self.owner._lock:
            if not self.owner.ingest_allowed():
                return
            self.owner.typed_seen['zone_capture_dynamic'] = True
            self.owner._append('zone_capture_dynamic_typed', payload)
            self.apply_zone_capture_dynamic(payload)
        self.owner._maybe_finalize(force=False)

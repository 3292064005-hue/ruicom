"""ROS recorder node wired through ingest and finalize helper stages."""

from __future__ import annotations

import os
import threading
import time
from typing import Dict

import rospy
from std_msgs.msg import String

from .artifact_builders import RecorderArtifactBuilder
from .common import CLASS_NAMES, ConfigurationError, JsonCodec, SCHEMA_VERSION, expand_path, project_dynamic_zone_results, require_positive_float, resolve_output_root, validate_dynamic_class_names, validate_profile_runtime_flags
from .contracts import snapshot_flush_allowed, validate_summary_class_names
from .io_core import AsyncJsonlWriter, atomic_write_json
from .lifecycle_protocol import decode_lifecycle_control, encode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import FrameRegionCounts, HealthState, MissionState, ZoneCapture, ZoneCaptureDynamic
from .time_core import NodeClock
from .recorder_core import (
    VALID_RECORDER_INPUT_MODES,
    build_initial_operator_audit_state,
    build_recorder_health_payload,
)
from .recorder_finalize import RecorderFinalizeController
from .recorder_ingest import RecorderIngestPipeline
from .submission_adapters import load_and_validate_submission_contract


class MissionRecorderNode:
    """Record mission data streams and write final artifacts atomically."""

    def __init__(self):
        rospy.init_node('mission_recorder_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self.output_root = resolve_output_root(
            self.config['output_root'],
            rospy.get_namespace(),
            self.config['output_root_use_namespace'],
        )
        os.makedirs(self.output_root, exist_ok=True)
        self.writer = AsyncJsonlWriter(
            os.path.join(self.output_root, 'mission_log.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.final_summary_json = os.path.join(self.output_root, 'final_summary.json')
        self.final_summary_csv = os.path.join(self.output_root, 'final_summary.csv')
        self.final_summary_v2_json = os.path.join(self.output_root, 'final_summary_v2.json')
        self.final_summary_v2_csv = os.path.join(self.output_root, 'final_summary_v2.csv')
        self.current_snapshot_json = os.path.join(self.output_root, 'summary_snapshot.json')
        self.runtime_metrics_json = os.path.join(self.output_root, 'runtime_metrics.json')
        self.current_snapshot_v2_json = os.path.join(self.output_root, 'summary_snapshot_v2.json')
        self.official_report_json = os.path.join(self.output_root, 'official_report.json')
        self.official_report_receipt_json = os.path.join(self.output_root, 'official_report_submission_receipt.json')
        self.authoritative_replay_manifest_json = os.path.join(self.output_root, 'authoritative_replay_manifest.json')
        self.submission_contract = load_and_validate_submission_contract(self.config['official_report_contract_path']) if self.config['official_report_mode'] == 'judge_contract' else None
        self.last_submission_receipt: Dict[str, object] = {}

        self._lock = threading.RLock()
        self._initialize_runtime_storage()

        self.artifact_builder = RecorderArtifactBuilder()
        self.finalize_controller = RecorderFinalizeController(self.artifact_builder)
        self.ingest = RecorderIngestPipeline(self)

        rospy.Subscriber(self.config['mission_state_topic'], String, self._mission_state_json_cb, queue_size=50)
        rospy.Subscriber(self.config['zone_capture_topic'], String, self._zone_capture_json_cb, queue_size=50)
        rospy.Subscriber(self.config['frame_region_counts_topic'], String, self._frame_region_counts_cb, queue_size=50)
        rospy.Subscriber(self.config['frame_region_counts_typed_topic'], FrameRegionCounts, self._frame_region_counts_typed_cb, queue_size=50)
        rospy.Subscriber(self.config['health_topic'], String, self._health_cb, queue_size=50)
        rospy.Subscriber(self.config['health_typed_topic'], HealthState, self._health_typed_cb, queue_size=50)
        rospy.Subscriber(self.config['runtime_evidence_topic'], String, self._runtime_evidence_cb, queue_size=50)
        rospy.Subscriber(self.config['current_zone_topic'], String, self._current_zone_cb, queue_size=50)
        rospy.Subscriber(self.config['mission_state_typed_topic'], MissionState, self._mission_state_typed_cb, queue_size=10)
        rospy.Subscriber(self.config['zone_capture_typed_topic'], ZoneCapture, self._zone_capture_typed_cb, queue_size=10)
        rospy.Subscriber(self.config['zone_capture_dynamic_topic'], ZoneCaptureDynamic, self._zone_capture_dynamic_typed_cb, queue_size=10)
        if self.config['lifecycle_managed']:
            self.control_command_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_command_callback, queue_size=20)
        else:
            self.control_command_sub = None
        self.summary_pub = rospy.Publisher(self.config['summary_snapshot_topic'], String, queue_size=10, latch=True)
        self.summary_dynamic_pub = rospy.Publisher(self.config['summary_snapshot_dynamic_topic'], String, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.snapshot_timer = rospy.Timer(rospy.Duration(1.0 / self.config['snapshot_flush_hz']), self._snapshot_timer_cb)
        self._publish_health('ok', 'node_ready', {'authoritative_input': self.config['recorder_authoritative_input'], **self.runtime.snapshot()})
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            'mission_recorder_node started. output_root=%s authoritative_input=%s snapshot_flush_hz=%.3f finalize_policy=%s lifecycle_state=%s',
            self.output_root,
            self.config['recorder_authoritative_input'],
            self.config['snapshot_flush_hz'],
            self.config['terminal_finalize_policy'],
            self.runtime.state,
        )

    def _initialize_runtime_storage(self) -> None:
        """Reset recorder in-memory state while preserving writer/output handles."""
        self.current_zone = ''
        self.current_route_id = ''
        self.latest_frame_region_counts: Dict[str, dict] = {}
        self.zone_results_dynamic: Dict[str, dict] = {}
        self.last_mission_state: Dict[str, object] = {}
        self.last_health = {}
        self.health_typed_seen_by_node: Dict[str, bool] = {}
        self.compat_shadow_events = 0
        self.class_schema_mismatch_count = 0
        self.finalized = False
        self.consistency_blocked = False
        self.typed_seen = {
            'mission_state': False,
            'zone_capture': False,
            'zone_capture_dynamic': False,
            'frame_region_counts': False,
            'health': False,
        }
        self.snapshot_dirty = False
        self.snapshot_state_changed = False
        self.terminal_state_seen = False
        self.terminal_state_payload: Dict[str, object] = {}
        self.last_terminal_wall_time = 0.0
        self.last_zone_capture_wall_time = 0.0
        self.operator_interventions = build_initial_operator_audit_state()
        self.last_submission_receipt = {}

    def _read_config(self):
        """Read and validate recorder parameters."""
        config = {
            'mission_state_topic': rospy.get_param('~mission_state_topic', 'recon/mission_state'),
            'mission_state_typed_topic': rospy.get_param('~mission_state_typed_topic', 'recon/mission_state_typed'),
            'zone_capture_topic': rospy.get_param('~zone_capture_topic', 'recon/zone_capture_result'),
            'zone_capture_typed_topic': rospy.get_param('~zone_capture_typed_topic', 'recon/zone_capture_result_typed'),
            'zone_capture_dynamic_topic': rospy.get_param('~zone_capture_dynamic_topic', 'recon/zone_capture_result_dynamic'),
            'frame_region_counts_topic': rospy.get_param('~frame_region_counts_topic', 'recon/zone_counts'),
            'frame_region_counts_typed_topic': rospy.get_param('~frame_region_counts_typed_topic', 'recon/zone_counts_typed'),
            'current_zone_topic': rospy.get_param('~current_zone_topic', 'recon/current_zone'),
            'health_topic': rospy.get_param('~health_topic', 'recon/health'),
            'health_typed_topic': rospy.get_param('~health_typed_topic', 'recon/health_typed'),
            'runtime_evidence_topic': rospy.get_param('~runtime_evidence_topic', 'recon/runtime/evidence'),
            'summary_snapshot_topic': rospy.get_param('~summary_snapshot_topic', 'recon/summary_snapshot'),
            'summary_snapshot_dynamic_topic': rospy.get_param('~summary_snapshot_dynamic_topic', 'recon/summary_snapshot_v2'),
            'output_root': rospy.get_param('~output_root', '~/.ros/ruikang_recon'),
            'output_root_use_namespace': bool(rospy.get_param('~output_root_use_namespace', True)),
            'writer_queue_size': int(rospy.get_param('~writer_queue_size', 512)),
            'writer_rotate_max_bytes': int(rospy.get_param('~writer_rotate_max_bytes', 5 * 1024 * 1024)),
            'writer_rotate_keep': int(rospy.get_param('~writer_rotate_keep', 3)),
            'snapshot_flush_hz': float(rospy.get_param('~snapshot_flush_hz', 1.0)),
            'snapshot_flush_on_state_change_only': bool(rospy.get_param('~snapshot_flush_on_state_change_only', False)),
            'recorder_authoritative_input': rospy.get_param('~recorder_authoritative_input', 'auto'),
            'terminal_finalize_policy': rospy.get_param('~terminal_finalize_policy', 'quiesced'),
            'terminal_quiesce_sec': float(rospy.get_param('~terminal_quiesce_sec', 0.5)),
            'allow_final_rewrite_after_finalize': bool(rospy.get_param('~allow_final_rewrite_after_finalize', False)),
            'class_schema_mismatch_policy': str(rospy.get_param('~class_schema_mismatch_policy', 'error')).strip().lower(),
            'classes': rospy.get_param('~classes', list(CLASS_NAMES)),
            'profile_role': str(rospy.get_param('~profile_role', 'integration')).strip().lower(),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower(),
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'official_report_mode': str(rospy.get_param('~official_report_mode', 'artifact')).strip().lower(),
            'official_report_schema': str(rospy.get_param('~official_report_schema', 'ruikang.recon.official_report')).strip() or 'ruikang.recon.official_report',
            'official_report_version': str(rospy.get_param('~official_report_version', '1.0.0')).strip() or '1.0.0',
            'official_report_sink_path': str(rospy.get_param('~official_report_sink_path', '')).strip(),
            'official_report_contract_path': str(rospy.get_param('~official_report_contract_path', '')).strip(),
            'official_report_submission_id': str(rospy.get_param('~official_report_submission_id', '')).strip(),
            'acceptance_stage': str(rospy.get_param('~acceptance_stage', 'contract_smoke')).strip() or 'contract_smoke',
        }
        config['snapshot_flush_hz'] = require_positive_float('snapshot_flush_hz', config['snapshot_flush_hz'])
        config['terminal_quiesce_sec'] = require_positive_float('terminal_quiesce_sec', config['terminal_quiesce_sec'])
        normalized_mode = str(config['recorder_authoritative_input']).strip().lower()
        if normalized_mode not in VALID_RECORDER_INPUT_MODES:
            raise ConfigurationError('recorder_authoritative_input must be one of: {}'.format(', '.join(VALID_RECORDER_INPUT_MODES)))
        config['recorder_authoritative_input'] = normalized_mode
        finalize_policy = str(config['terminal_finalize_policy']).strip().lower()
        if finalize_policy not in ('immediate', 'quiesced'):
            raise ConfigurationError('terminal_finalize_policy must be one of: immediate, quiesced')
        config['terminal_finalize_policy'] = finalize_policy
        if config['class_schema_mismatch_policy'] not in ('warn', 'error'):
            raise ConfigurationError('class_schema_mismatch_policy must be one of: warn, error')
        if config['time_source_mode'] not in ('ros', 'wall'):
            raise ConfigurationError('time_source_mode must be ros or wall')
        if config['runtime_grade'] not in ('integration', 'contract', 'reference', 'field'):
            raise ConfigurationError('runtime_grade must be one of: integration, contract, reference, field')
        if config['official_report_mode'] not in ('disabled', 'artifact', 'judge_contract'):
            raise ConfigurationError('official_report_mode must be disabled, artifact or judge_contract')
        config['official_report_contract_path'] = expand_path(config['official_report_contract_path']) if str(config['official_report_contract_path']).strip() else ''
        if config['official_report_mode'] == 'judge_contract' and not config['official_report_contract_path']:
            raise ConfigurationError('official_report_contract_path must be set when official_report_mode=judge_contract')
        config['profile_role'] = validate_profile_runtime_flags(
            config['profile_role'],
            owner='mission_recorder_node',
            lifecycle_managed=bool(config['lifecycle_managed']),
        )
        validate_summary_class_names(config['classes'])
        validate_dynamic_class_names(config['classes'], owner='mission_recorder_node.dynamic_schema')
        return config

    @property
    def zone_results(self) -> Dict[str, dict]:
        """Return the legacy projection of recorder authoritative zone results."""
        return project_dynamic_zone_results(self.zone_results_dynamic)

    def ingest_allowed(self) -> bool:
        """Return whether recorder should currently mutate authoritative runtime state."""
        return self.runtime.processing_allowed and not self.consistency_blocked

    def _publish_health(self, status: str, message: str, details: dict | None = None) -> None:
        """Publish recorder-local health on both JSON and typed lanes.

        Args:
            status: Health severity label.
            message: Machine-readable reason string.
            details: Optional diagnostic payload merged into the published message.

        Returns:
            None.

        Raises:
            No explicit exception is raised. Publisher transport errors propagate
            through rospy as usual.

        Boundary behavior:
            The JSON payload uses the recorder business clock so system-manager
            freshness checks remain consistent under ``use_sim_time``; wall-clock
            timestamps remain reserved for file/log artifacts.
        """
        details = dict(details or {})
        details.setdefault('writer_ready', self.writer.last_error in (None, ''))
        details.setdefault('runtime_state', self.runtime.state)
        details.setdefault('lifecycle_managed', bool(self.config['lifecycle_managed']))
        payload = build_recorder_health_payload(
            stamp_sec=self.clock.now_business_sec(),
            runtime_state=self.runtime.state,
            lifecycle_managed=bool(self.config['lifecycle_managed']),
            status=status,
            message=message,
            details=details,
        )
        payload['schema_version'] = SCHEMA_VERSION
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = self.clock.now_ros_time()
        typed.header.frame_id = 'map'
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(payload.get('details', {}))
        self.health_typed_pub.publish(typed)

    def _append(self, event_type: str, payload: dict):
        """Append one recorder event to the JSONL writer."""
        self.writer.write({'wall_stamp': time.time(), 'type': event_type, 'payload': payload})

    def _update_operator_interventions(self, payload: dict) -> dict:
        """Fold one runtime-evidence payload into the operator intervention audit."""
        from .recorder_core import record_runtime_evidence_operator_audit
        self.operator_interventions = record_runtime_evidence_operator_audit(self.operator_interventions, payload)
        return dict(self.operator_interventions)

    def _inject_recorder_diagnostics(self, payload: dict) -> dict:
        """Attach recorder-local diagnostics to a summary or final artifact payload."""
        payload['recorder_diagnostics'] = {
            'compat_shadow_events': int(self.compat_shadow_events),
            'class_schema_mismatch_count': int(self.class_schema_mismatch_count),
            'consistency_blocked': bool(self.consistency_blocked),
            'health_typed_seen_by_node': {key: bool(value) for key, value in sorted(self.health_typed_seen_by_node.items())},
            'class_schema_mismatch_policy': self.config['class_schema_mismatch_policy'],
            'runtime_state': self.runtime.state,
            'lifecycle_managed': bool(self.config['lifecycle_managed']),
            'zone_capture_compatibility_mode': 'projection_only',
            'writer_ready': self.writer.last_error in (None, ''),
            'official_report_mode': self.config['official_report_mode'],
            'official_report_contract_path': self.config['official_report_contract_path'],
            'official_report_adapter_type': str((self.submission_contract or {}).get('adapter_type', '')).strip(),
            'last_submission_receipt': dict(self.last_submission_receipt or {}),
            'acceptance_stage': self.config['acceptance_stage'],
            'runtime_grade': self.config['runtime_grade'],
        }
        return payload

    def _snapshot_timer_cb(self, _event) -> None:
        self._flush_snapshot(force=False)
        self._maybe_finalize(force=False)

    def _flush_snapshot(self, force: bool) -> None:
        """Persist snapshot artifacts when flush policy allows."""
        with self._lock:
            if not snapshot_flush_allowed(
                snapshot_dirty=self.snapshot_dirty,
                state_changed=self.snapshot_state_changed,
                flush_on_state_change_only=self.config['snapshot_flush_on_state_change_only'],
                force=force,
            ):
                return
            try:
                snapshot = self._inject_recorder_diagnostics(self.artifact_builder.build_summary(self))
                snapshot_v2 = self._inject_recorder_diagnostics(self.artifact_builder.build_summary_v2(self))
                writer_error = self.writer.last_error
                if writer_error:
                    snapshot.setdefault('writer_status', {})['last_error'] = writer_error
                    snapshot_v2.setdefault('writer_status', {})['last_error'] = writer_error
                if self.writer.dropped_messages:
                    snapshot.setdefault('writer_status', {})['dropped_messages'] = self.writer.dropped_messages
                    snapshot_v2.setdefault('writer_status', {})['dropped_messages'] = self.writer.dropped_messages
                atomic_write_json(self.current_snapshot_json, snapshot)
                atomic_write_json(self.current_snapshot_v2_json, snapshot_v2)
                self.summary_pub.publish(String(data=JsonCodec.dumps(snapshot)))
                self.summary_dynamic_pub.publish(String(data=JsonCodec.dumps(snapshot_v2)))
                self.snapshot_dirty = False
                self.snapshot_state_changed = False
            except Exception as exc:
                self._append('snapshot_write_failed', {'error': str(exc)})
                rospy.logwarn_throttle(2.0, 'summary snapshot write failed: %s', exc)

    def _maybe_finalize(self, force: bool) -> None:
        with self._lock:
            if self.runtime.state == 'SHUTDOWN' and not force:
                return
            self.finalize_controller.finalize_locked(self, force=force)

    def _control_command_callback(self, msg: String) -> None:
        """Apply external lifecycle commands to recorder-local ingest/finalize state."""
        envelope = decode_lifecycle_control(msg.data)
        command = envelope.command
        if not command or not envelope.matches(rospy.get_name().split('/')[-1]):
            return
        with self._lock:
            result = self.runtime.apply(command)
            details = {'command': command, 'target': envelope.target, 'issued_by': envelope.issued_by, 'metadata': dict(envelope.metadata or {})}
            if not result.accepted:
                self._publish_health('warn', result.message, details)
                return
            if command == 'reset':
                self._initialize_runtime_storage()
            elif command == 'shutdown':
                self._maybe_finalize(force=True)
            self.snapshot_dirty = True
            self.snapshot_state_changed = True
            self._publish_health('ok' if result.state == 'ACTIVE' else 'warn', result.message, details)

    def _current_zone_cb(self, msg: String):
        self.ingest.current_zone_cb(msg)

    def _health_cb(self, msg: String):
        self.ingest.health_cb(msg)

    def _health_typed_cb(self, msg: HealthState):
        self.ingest.health_typed_cb(msg)

    def _frame_region_counts_cb(self, msg: String):
        self.ingest.frame_region_counts_cb(msg)

    def _frame_region_counts_typed_cb(self, msg: FrameRegionCounts):
        self.ingest.frame_region_counts_typed_cb(msg)


    def _runtime_evidence_cb(self, msg: String):
        self.ingest.runtime_evidence_cb(msg)

    def _mission_state_json_cb(self, msg: String):
        self.ingest.mission_state_json_cb(msg)

    def _zone_capture_json_cb(self, msg: String):
        self.ingest.zone_capture_json_cb(msg)

    def _mission_state_typed_cb(self, msg: MissionState):
        self.ingest.mission_state_typed_cb(msg)

    def _zone_capture_typed_cb(self, msg: ZoneCapture):
        self.ingest.zone_capture_typed_cb(msg)

    def _zone_capture_dynamic_typed_cb(self, msg: ZoneCaptureDynamic):
        self.ingest.zone_capture_dynamic_typed_cb(msg)

    def _on_shutdown(self):
        try:
            self._maybe_finalize(force=True)
        finally:
            try:
                self.snapshot_timer.shutdown()
            except Exception as exc:
                rospy.logwarn('mission_recorder snapshot timer shutdown failed: %s', exc)
            self.writer.close()
            if self.writer.last_error:
                rospy.logwarn('mission_recorder writer closed with last_error=%s', self.writer.last_error)

    def spin(self):
        rospy.spin()

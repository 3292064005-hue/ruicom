"""ROS mission node wiring for the executor-based mission runtime."""

from __future__ import annotations

import os
from typing import Dict, List, Optional

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray

from .common import (
    SCHEMA_VERSION,
    ConfigurationError,
    Detection,
    DetectionFrame,
    JsonCodec,
    MissionStateSnapshot,
    PoseSnapshot,
    class_schema_hash,
    dynamic_zone_result_to_legacy_payload,
    load_waypoints,
    project_dynamic_zone_results,
    quaternion_to_yaw_rad,
    resolve_output_root,
    validate_route_frame_region_contract,
)
from .io_core import AsyncJsonlWriter
from .mission_config import read_mission_config
from .mission_context import MissionContext
from .mission_publishers import build_mission_publishers
from .mission_schema_guard import MissionSchemaGuard
from .mission_core import AggregationPolicy, ArrivalEvaluator, CaptureWindowAggregator
from .mission_executor import MissionExecutor, MissionRuntimeHooks
from .mission_plan import MissionPlan
from .msg import DetectionArray, HealthState, MissionState, ZoneCapture, ZoneCaptureDynamic
from .navigation_adapters import build_navigation_registry
from .pose_sources import TfLookupPoseSource
from .recovery_policies import RetryThenFailPolicy
from .lifecycle_protocol import decode_lifecycle_control, encode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .time_core import NodeClock


class MissionManagerNode:
    """Mission execution node with explicit executor, context, and policy seams."""

    _CONTEXT_FIELDS = {
        'state', 'state_since', 'route_index', 'retry_count', 'current_zone', 'current_route_id',
        'current_step_id', 'current_task_type', 'current_task_objective',
        'zone_results_dynamic', 'latest_detection_frame', 'mission_started_at',
        'dispatch_started_at', 'current_capture_deadline', 'last_health_publish_mono',
        'last_tf_warning_mono', 'dispatch_quiesce_until', 'event_counter',
        'schema_blocked_reason', 'preflight_deadline', 'last_preflight_warning_mono',
    }

    def __getattr__(self, name):
        if name in self._CONTEXT_FIELDS:
            return getattr(self.context, name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        if name in self._CONTEXT_FIELDS and 'context' in self.__dict__:
            setattr(self.context, name, value)
            return
        super().__setattr__(name, value)

    def __init__(self):
        rospy.init_node('mission_manager_node', anonymous=False)
        self.config = read_mission_config(rospy)
        self.clock = NodeClock(self.config['time_source_mode'])
        self.schema_guard = MissionSchemaGuard(
            class_names=self.config['classes'],
            mismatch_policy=self.config['class_schema_mismatch_policy'],
            on_mismatch=self._handle_schema_mismatch,
        )
        self.class_names = self.schema_guard.validate_local_schema()
        self.class_schema_hash = class_schema_hash(self.class_names)
        self.output_root = resolve_output_root(
            self.config['output_root'],
            rospy.get_namespace(),
            self.config['output_root_use_namespace'],
        )
        os.makedirs(self.output_root, exist_ok=True)
        if self.config['tasks']:
            self.plan = MissionPlan.from_task_specs(self.config['tasks'], dwell_default_sec=self.config['dwell_default_sec'])
            self.route = list(self.plan.waypoints)
        else:
            self.route = load_waypoints(self.config['route'], self.config['dwell_default_sec'])
            self.plan = MissionPlan.from_waypoints(self.route)
        validate_route_frame_region_contract(
            self.route,
            require_binding=self.config['require_route_frame_regions'],
            allowed_frame_regions=self.config['expected_frame_regions'],
            owner='mission_manager_node.route',
        )
        self.context = MissionContext()
        self.runtime = ManagedRuntimeState(lifecycle_managed=bool(self.config['lifecycle_managed']))
        self.arrival_evaluator = ArrivalEvaluator(
            comparison_frame=self.config['comparison_frame'],
            reach_tolerance_m=self.config['goal_reach_tolerance_m'],
            pose_timeout_sec=self.config['pose_timeout_sec'],
        )
        self.capture_aggregator = CaptureWindowAggregator(
            AggregationPolicy(
                class_names=self.class_names,
                reduction=self.config['capture_reduction'],
                min_valid_frames=self.config['capture_min_valid_frames'],
            )
        )
        self.event_writer = AsyncJsonlWriter(
            os.path.join(self.output_root, 'mission_manager_events.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.execution_trace_writer = AsyncJsonlWriter(
            os.path.join(self.output_root, 'execution_trace.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.recovery_trace_writer = AsyncJsonlWriter(
            os.path.join(self.output_root, 'recovery_trace.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.publishers = build_mission_publishers(self.config)
        self.detections_sub = rospy.Subscriber(self.config['detections_topic'], DetectionArray, self._detections_callback, queue_size=20)
        self.control_command_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_command_callback, queue_size=20)
        self._pose_subscribers: List[rospy.Subscriber] = []
        self.tf_pose_source: Optional[TfLookupPoseSource] = None
        self._configure_pose_source()
        self.nav_adapter, self.nav_capability = self._build_navigation_adapter()
        self._last_navigation_runtime_live_sec = 0.0
        self._last_navigation_feedback_live_sec = 0.0
        self._last_navigation_result_live_sec = 0.0
        self._last_navigation_goal_dispatch_sec = 0.0
        self._navigation_runtime_status_sub = None
        self._navigation_runtime_feedback_sub = None
        self._navigation_runtime_result_sub = None
        self._configure_navigation_runtime_liveness_probe()
        self.navigation_runtime_probe = self._navigation_runtime_probe()
        self.recovery_policy = RetryThenFailPolicy(
            retry_limit=self.config['retry_limit'],
            navigation_failure_quiesce_sec=self.config['navigation_failure_quiesce_sec'],
        )
        self.executor = MissionExecutor(
            plan=self.plan,
            context=self.context,
            clock=self.clock,
            nav_adapter=self.nav_adapter,
            capture_aggregator=self.capture_aggregator,
            recovery_policy=self.recovery_policy,
            hooks=MissionRuntimeHooks(
                emit_state=self._emit_state,
                publish_zone_capture=self._publish_zone_capture,
                publish_current_zone=lambda zone: self.publishers.current_zone.publish(String(data=zone)),
                publish_health=self._publish_health,
                refresh_pose_source=self._refresh_pose_source,
                preflight_ready=self._preflight_ready,
                record_navigation_dispatch=self._record_navigation_dispatch,
            ),
            mission_timeout_sec=self.config['mission_timeout_sec'],
            preflight_timeout_sec=self.config['preflight_timeout_sec'],
            class_names=self.class_names,
            class_schema_hash=self.class_schema_hash,
        )
        self.paused_from_state = ''
        runtime_contract = dict(self.config.get('navigation_runtime_contract', {}))
        runtime_contract.setdefault('adapter_type', self.nav_capability.name)
        runtime_binding_summary = getattr(self.nav_adapter, 'runtime_binding_summary', None)
        if callable(runtime_binding_summary):
            runtime_contract['runtime_binding'] = runtime_binding_summary()
        self._publish_health('ok', 'node_ready', {'profile_role': self.config['profile_role'], 'lifecycle_managed': self.config['lifecycle_managed'], 'route_bound': bool(self.route), 'navigation_contract_satisfied': bool(self.config.get('navigation_contract_satisfied', False)), 'field_asset_ready': bool(self.config.get('field_asset_contract_satisfied', False)), 'field_asset_verified': bool(self.config.get('field_asset_verified', False)), 'field_asset_state': str(self.config.get('field_asset_state', '')).strip(), 'field_asset_verification_scope': str(self.config.get('field_asset_verification_scope', '')).strip(), 'task_graph_enabled': bool(self.config.get('tasks')), 'navigation_capability': runtime_contract, 'navigation_runtime_probe': dict(self.navigation_runtime_probe), 'navigation_runtime_probe_satisfied': bool(self.navigation_runtime_probe.get('satisfied', False)), 'vendor_runtime_contract_satisfied': bool(self.config.get('vendor_runtime_contract_satisfied', True)), 'vendor_runtime_binding_report': dict(self.config.get('vendor_runtime_binding_report', {})), 'vendor_bundle_preflight': dict(self.config.get('vendor_bundle_preflight', {})), 'vendor_bundle_preflight_satisfied': bool(self.config.get('vendor_bundle_preflight_satisfied', True)), 'pose_source_ready': self.arrival_evaluator.pose_fresh(self.clock.now_business_sec())})
        if self.config['auto_start']:
            self.executor.start()
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            'mission_manager_node started. route_len=%d adapter=%s pose_source=%s time_source=%s',
            len(self.route),
            self.nav_capability.name,
            self.config['pose_source_type'],
            self.config['time_source_mode'],
        )

    @property
    def zone_results(self) -> Dict[str, dict]:
        """Return the legacy projection of the authoritative dynamic zone state.

        Args:
            None.

        Returns:
            Legacy-compatible zone-result mapping keyed by route id.

        Raises:
            ConfigurationError: If any dynamic payload cannot be projected.

        Boundary behavior:
            The returned mapping is computed on demand and is never stored as a
            second authoritative runtime copy.
        """
        return project_dynamic_zone_results(self.zone_results_dynamic)

    def _reset_runtime(self) -> None:
        """Reset runtime state to an idle, externally controllable baseline.

        Args:
            None.

        Returns:
            None.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            The reset is idempotent and clears any in-flight capture window and
            current-zone publication before returning the node to ``IDLE``.
        """
        self.nav_adapter.cancel(self.clock.now_business_sec())
        self.capture_aggregator.reset()
        self.context = MissionContext()
        self.executor.context = self.context
        self.publishers.current_zone.publish(String(data=''))
        self.paused_from_state = ''

    def _start_or_resume(self, *, source: str) -> None:
        """Activate mission execution from an external lifecycle command.

        Args:
            source: Human-readable source string used in emitted diagnostics.

        Returns:
            None.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            Terminal or idle states are restarted from the first waypoint, while a
            paused mission resumes by re-entering ``DISPATCH_PENDING`` for the
            current route slot.
        """
        if self.state == 'PAUSED':
            resume_state = self.paused_from_state or 'DISPATCH_PENDING'
            self.preflight_deadline = self.clock.now_business_sec() + self.config['preflight_timeout_sec']
            self._emit_state('mission_resumed', state='DISPATCH_PENDING', details={'resume_from': resume_state, 'source': source})
            self.paused_from_state = ''
            return
        self.capture_aggregator.reset()
        self.executor.start()
        self._publish_health('ok', 'mission_activated', {'source': source})

    def _pause_runtime(self, *, source: str) -> None:
        """Pause mission execution and cancel any outstanding navigation goal.

        Args:
            source: Human-readable command source for diagnostics.

        Returns:
            None.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            Pausing from ``IDLE`` or a terminal state is treated as a no-op health
            warning rather than a hard error.
        """
        if self.state in ('IDLE', 'FINISHED', 'TIMEOUT', 'FAULT', 'SHUTDOWN'):
            self._publish_health('warn', 'pause_ignored', {'source': source, 'state': self.state})
            return
        self.paused_from_state = self.state
        self.nav_adapter.cancel(self.clock.now_business_sec())
        self._emit_state('mission_paused', state='PAUSED', details={'paused_from': self.paused_from_state, 'source': source})

    def _control_command_callback(self, msg: String) -> None:
        """Handle lifecycle commands published by the system manager.

        Args:
            msg: String command message. Supported values are ``activate``,
                ``pause``, ``reset`` and ``shutdown``.

        Returns:
            None.

        Raises:
            No explicit exception is raised. Invalid commands are reported through
            the health channel and ignored.

        Boundary behavior:
            Repeated ``activate`` requests while already active are treated as a
            restart only from terminal states; otherwise they simply emit a health
            warning and preserve the current mission slot.
        """
        envelope = decode_lifecycle_control(msg.data)
        command = envelope.command
        if not command or not envelope.matches(rospy.get_name().split('/')[-1]):
            return
        details = {'command': command, 'target': envelope.target, 'issued_by': envelope.issued_by, 'metadata': dict(envelope.metadata or {})}
        try:
            runtime_result = self.runtime.apply(command)
            if not runtime_result.accepted:
                self._publish_health('warn', runtime_result.message, details)
                return
            if command == 'configure':
                self._publish_health('warn', runtime_result.message, details)
                return
            if command in ('activate', 'start', 'resume'):
                if self.state in ('DISPATCH_PENDING', 'DISPATCHED', 'ACTIVE', 'DWELL'):
                    self._publish_health('warn', 'activate_ignored', {'state': self.state, **details})
                    return
                self._start_or_resume(source=command)
                return
            if command == 'pause':
                self._pause_runtime(source=command)
                return
            if command == 'reset':
                self._reset_runtime()
                self._emit_state('mission_reset', state='IDLE', details={'source': command, **details})
                self._publish_health('ok', 'mission_reset', {'source': command, **details})
                return
            if command == 'shutdown':
                self.nav_adapter.cancel(self.clock.now_business_sec())
                self.capture_aggregator.reset()
                self._emit_state('mission_shutdown', state='SHUTDOWN', details={'source': command, **details})
                self._publish_health('warn', 'mission_shutdown', {'source': command, **details})
                return
            self._publish_health('warn', 'unknown_control_command', details)
        except Exception as exc:
            self._publish_health('error', 'control_command_failed', {'error': str(exc), **details})
            self._emit_state('fault', state='FAULT', details={'error': str(exc), **details})

    def _handle_schema_mismatch(self, stream: str, message: str, *, details: Optional[dict] = None) -> None:
        self.schema_blocked_reason = '{}: {}'.format(stream, message)
        payload = {'stream': stream, 'message': message, 'policy': self.config['class_schema_mismatch_policy']}
        if details:
            payload['details'] = details
        payload.setdefault('details', {})
        payload['details'].setdefault('runtime_state', self.runtime.state)
        payload['details'].setdefault('current_step_id', str(self.current_step_id).strip())
        payload['details'].setdefault('current_task_type', str(self.current_task_type).strip())
        payload['details'].setdefault('current_task_objective', str(self.current_task_objective).strip())
        payload['details'].setdefault('lifecycle_managed', bool(self.config['lifecycle_managed']))
        payload['details'].setdefault('route_bound', bool(self.route))
        payload['details'].setdefault('pose_source_ready', self.arrival_evaluator.pose_fresh(self.clock.now_business_sec()))
        payload['details'].setdefault('navigation_contract_satisfied', bool(self.config.get('navigation_contract_satisfied', False)))
        self.navigation_runtime_probe = self._navigation_runtime_probe()
        payload['details'].setdefault('navigation_runtime_probe', dict(self.navigation_runtime_probe))
        payload['details'].setdefault('navigation_runtime_probe_satisfied', bool(self.navigation_runtime_probe.get('satisfied', False)))
        payload['details'].setdefault('field_asset_ready', bool(self.config.get('field_asset_contract_satisfied', False)))
        payload['details'].setdefault('vendor_runtime_contract_satisfied', bool(self.config.get('vendor_runtime_contract_satisfied', True)))
        payload['details'].setdefault('vendor_bundle_preflight', dict(self.config.get('vendor_bundle_preflight', {})))
        payload['details'].setdefault('vendor_bundle_preflight_satisfied', bool(self.config.get('vendor_bundle_preflight_satisfied', True)))
        payload['details'].setdefault('vendor_runtime_binding_report', dict(self.config.get('vendor_runtime_binding_report', {})))
        payload['details'].setdefault('field_asset_verified', bool(self.config.get('field_asset_verified', False)))
        payload['details'].setdefault('field_asset_state', str(self.config.get('field_asset_state', '')).strip())
        self.event_writer.write({'type': 'schema_mismatch', 'payload': payload, 'wall_stamp': self.clock.now_wall_sec()})
        self._publish_health('warn', 'class_schema_mismatch', payload)
        if self.config['class_schema_mismatch_policy'] == 'error':
            self._emit_state('schema_mismatch', state='FAULT', details=payload)

    def _preflight_blockers(self, waypoint, now_sec: float) -> list[str]:
        blockers = []
        if self.schema_blocked_reason:
            blockers.append('schema_blocked')
        if self.config['preflight_require_pose'] and not self.arrival_evaluator.pose_fresh(now_sec):
            blockers.append('pose_unavailable')
        if self.config['preflight_require_detections']:
            if self.latest_detection_frame is None:
                blockers.append('detections_unavailable')
            else:
                age = now_sec - float(self.latest_detection_frame.stamp)
                if age > self.config['detections_timeout_sec']:
                    blockers.append('detections_stale')
        if waypoint.goal_frame != self.config['comparison_frame'] and self.config['navigation_adapter_type'] == 'simple_topic':
            blockers.append('comparison_frame_mismatch')
        return blockers

    def _preflight_ready(self, waypoint, now_sec: float) -> bool:
        blockers = self._preflight_blockers(waypoint, now_sec)
        if not blockers:
            return True
        if now_sec >= self.preflight_deadline:
            details = {'blockers': blockers, 'zone_name': waypoint.name, 'current_route_id': waypoint.route_id}
            if self.config['preflight_failure_policy'] == 'warn':
                self._publish_health('warn', 'preflight_failed_continue', details)
                return True
            self._emit_state('preflight_failed', state='FAULT', details=details)
            return False
        if (self.clock.now_monotonic_sec() - self.last_preflight_warning_mono) >= 1.0:
            self.last_preflight_warning_mono = self.clock.now_monotonic_sec()
            self._publish_health('warn', 'preflight_waiting', {'blockers': blockers, 'zone_name': waypoint.name})
        return False

    def _configure_pose_source(self) -> None:
        pose_source_type = str(self.config['pose_source_type']).strip()
        if pose_source_type == 'odometry':
            self._pose_subscribers.append(rospy.Subscriber(self.config['odom_topic'], Odometry, self._odom_callback, queue_size=20))
            return
        if pose_source_type == 'amcl_pose':
            self._pose_subscribers.append(rospy.Subscriber(self.config['amcl_pose_topic'], PoseWithCovarianceStamped, self._amcl_callback, queue_size=20))
            return
        self.tf_pose_source = TfLookupPoseSource(
            target_frame=self.config['tf_target_frame'],
            source_frame=self.config['tf_source_frame'],
            lookup_timeout_sec=self.config['tf_lookup_timeout_sec'],
            clock=self.clock,
        )

    def _navigation_registry(self):
        """Build the navigation capability registry for the configured node.

        Args:
            None.

        Returns:
            Registry that can materialize one navigation adapter and expose its
            declared capabilities for diagnostics and contract checks.

        Raises:
            ConfigurationError: If a requested backend is unsupported.

        Boundary behavior:
            Registry construction is deterministic; capability metadata is shared
            with startup-time contract validation so build-time and run-time
            semantics cannot silently drift apart.
        """
        return build_navigation_registry(
            self.config,
            clock=self.clock,
            arrival_evaluator=self.arrival_evaluator,
            builders_enabled=True,
        )

    def _navigation_capability_summary(self) -> dict:
        """Return the resolved navigation capability contract for diagnostics.

        Args:
            None.

        Returns:
            JSON-serializable capability summary.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            Empty capability state before adapter construction resolves to an empty
            mapping so early health publication can stay side-effect free.
        """
        if not getattr(self, 'nav_capability', None):
            return {}
        summary = {
            'name': self.nav_capability.name,
            'status_source': self.nav_capability.status_source,
            'supports_cancel': self.nav_capability.supports_cancel,
            'requires_external_status': self.nav_capability.requires_external_status,
            'planner_interface': self.nav_capability.planner_interface,
            'controller_interface': self.nav_capability.controller_interface,
            'recovery_interface': self.nav_capability.recovery_interface,
            'localization_interface': self.nav_capability.localization_interface,
            'goal_transport': self.nav_capability.goal_transport,
            'status_transport': self.nav_capability.status_transport,
            'cancel_transport': self.nav_capability.cancel_transport,
        }
        runtime_binding_summary = getattr(self.nav_adapter, 'runtime_binding_summary', None)
        if callable(runtime_binding_summary):
            summary['runtime_binding'] = runtime_binding_summary()
        return summary
    def _navigation_runtime_probe(self) -> dict:
        """Probe runtime navigation resources visible on the current ROS graph.

        Returns:
            Dictionary containing per-resource visibility and aggregate readiness
            flags suitable for system-manager gates.

        Raises:
            No explicit exception is raised. Probe failures are downgraded into a
            structured health payload so startup diagnostics remain observable.

        Boundary behavior:
            Actionlib backends are considered semantically ready only when their
            status channel is not merely visible on the graph but also emits live
            status traffic within ``navigation_runtime_live_timeout_sec``.
        """
        try:
            published_topics = rospy.get_published_topics()
        except Exception as exc:
            return {
                'satisfied': False,
                'probe_error': str(exc),
            }
        action_server_ready = bool(
            str(self.config.get('navigation_goal_transport', '')).strip().lower() != 'actionlib'
            or getattr(self.nav_capability, 'name', '') == 'move_base_action'
        )
        dispatch_active = bool(self.state in ('DISPATCHED', 'ACTIVE', 'DWELL') and float(self.dispatch_started_at or 0.0) > 0.0)
        return evaluate_navigation_runtime_probe(
            self.config,
            published_topics=published_topics,
            action_server_ready=action_server_ready,
            now_sec=self.clock.now_business_sec(),
            runtime_live_stamp_sec=float(self._last_navigation_runtime_live_sec),
            runtime_live_timeout_sec=float(
                self.config.get(
                    'navigation_runtime_live_timeout_sec',
                    self.config.get('navigation_status_timeout_sec', 3.0),
                )
            ),
            feedback_live_stamp_sec=float(self._last_navigation_feedback_live_sec),
            result_live_stamp_sec=float(self._last_navigation_result_live_sec),
            dispatch_goal_stamp_sec=float(self._last_navigation_goal_dispatch_sec),
            require_action_roundtrip=dispatch_active,
        )

    def _navigation_runtime_live_cb(self, _msg) -> None:
        """Record one runtime-liveness heartbeat for navigation backend status.

        Args:
            _msg: ROS status message from the configured action/topic transport.

        Returns:
            None.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            The callback stores only a monotonic business timestamp because the
            runtime probe cares about freshness, not payload semantics.
        """
        self._last_navigation_runtime_live_sec = self.clock.now_business_sec()

    def _navigation_runtime_feedback_cb(self, _msg) -> None:
        """Record one action-feedback heartbeat for navigation roundtrip probes."""
        self._last_navigation_feedback_live_sec = self.clock.now_business_sec()

    def _navigation_runtime_result_cb(self, _msg) -> None:
        """Record one action-result heartbeat for navigation roundtrip probes."""
        self._last_navigation_result_live_sec = self.clock.now_business_sec()

    def _record_navigation_dispatch(self, _waypoint, dispatch_sec: float) -> None:
        """Record the most recent goal-dispatch timestamp used by live probes."""
        self._last_navigation_goal_dispatch_sec = float(dispatch_sec or 0.0)

    def _configure_navigation_runtime_liveness_probe(self) -> None:
        """Subscribe to one runtime status stream used for readiness liveness.

        Args:
            None.

        Returns:
            None.

        Raises:
            No explicit exception is raised. Unsupported transports simply leave
            runtime-live probing disabled.

        Boundary behavior:
            Only actionlib transports are subscribed here because they expose a
            standard status heartbeat topic. Internal pose-evaluator backends do
            not publish such a channel and therefore remain graph-only.
        """
        status_transport = str(self.config.get('navigation_status_transport', '')).strip().lower()
        if status_transport != 'actionlib':
            return
        action_name = str(self.config.get('move_base_action_name', '')).strip().strip('/')
        if not action_name:
            return
        status_topic = '{}/status'.format(action_name)
        feedback_topic = '{}/feedback'.format(action_name)
        result_topic = '{}/result'.format(action_name)
        self._navigation_runtime_status_sub = rospy.Subscriber(
            status_topic,
            GoalStatusArray,
            self._navigation_runtime_live_cb,
            queue_size=20,
        )
        self._navigation_runtime_feedback_sub = rospy.Subscriber(
            feedback_topic,
            rospy.AnyMsg,
            self._navigation_runtime_feedback_cb,
            queue_size=20,
        )
        self._navigation_runtime_result_sub = rospy.Subscriber(
            result_topic,
            rospy.AnyMsg,
            self._navigation_runtime_result_cb,
            queue_size=20,
        )
    def _build_navigation_adapter(self):
        """Build the configured navigation adapter and its capability manifest.

        Args:
            None.

        Returns:
            Tuple of ``(adapter, capability)`` for downstream runtime and health
            reporting.

        Raises:
            ConfigurationError: If the adapter type is unsupported or backend
            initialization fails.

        Boundary behavior:
            Capability metadata is returned even for simple backends so the system
            manager can reason about cancellation and status provenance uniformly.
        """
        registry = self._navigation_registry()
        return registry.build(self.config['navigation_adapter_type'])

    def _publish_health(self, status: str, message: str, details: dict | None = None):
        now_mono = self.clock.now_monotonic_sec()
        if status == 'ok' and (now_mono - self.last_health_publish_mono) < 0.5:
            return
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'mission_manager_node',
            'status': status,
            'message': message,
            'schema_version': SCHEMA_VERSION,
            'current_zone': self.current_zone,
            'route_index': self.route_index,
        }
        if details:
            payload['details'] = details
        payload.setdefault('details', {})
        payload['details'].setdefault('runtime_state', self.runtime.state)
        payload['details'].setdefault('current_step_id', str(self.current_step_id).strip())
        payload['details'].setdefault('current_task_type', str(self.current_task_type).strip())
        payload['details'].setdefault('current_task_objective', str(self.current_task_objective).strip())
        payload['details'].setdefault('lifecycle_managed', bool(self.config['lifecycle_managed']))
        payload['details'].setdefault('route_bound', bool(self.route))
        payload['details'].setdefault('pose_source_ready', self.arrival_evaluator.pose_fresh(self.clock.now_business_sec()))
        payload['details'].setdefault('navigation_contract_satisfied', bool(self.config.get('navigation_contract_satisfied', False)))
        self.navigation_runtime_probe = self._navigation_runtime_probe()
        payload['details'].setdefault('navigation_runtime_probe', dict(self.navigation_runtime_probe))
        payload['details'].setdefault('navigation_runtime_probe_satisfied', bool(self.navigation_runtime_probe.get('satisfied', False)))
        payload['details'].setdefault('field_asset_ready', bool(self.config.get('field_asset_contract_satisfied', False)))
        payload['details'].setdefault('vendor_runtime_contract_satisfied', bool(self.config.get('vendor_runtime_contract_satisfied', True)))
        payload['details'].setdefault('vendor_bundle_preflight', dict(self.config.get('vendor_bundle_preflight', {})))
        payload['details'].setdefault('vendor_bundle_preflight_satisfied', bool(self.config.get('vendor_bundle_preflight_satisfied', True)))
        payload['details'].setdefault('vendor_runtime_binding_report', dict(self.config.get('vendor_runtime_binding_report', {})))
        payload['details'].setdefault('field_asset_verified', bool(self.config.get('field_asset_verified', False)))
        payload['details'].setdefault('field_asset_state', str(self.config.get('field_asset_state', '')).strip())
        writer_error = self.event_writer.last_error
        if writer_error:
            payload.setdefault('details', {})['writer_error'] = writer_error
        dropped_messages = self.event_writer.dropped_messages
        if dropped_messages:
            payload.setdefault('details', {})['writer_dropped_messages'] = dropped_messages
        self.publishers.health_json.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = self.clock.to_ros_time(payload['stamp'])
        typed.header.frame_id = self.config['health_frame_id']
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(payload.get('details', {}))
        self.publishers.health_typed.publish(typed)
        self.last_health_publish_mono = now_mono

    def _mission_state_fixed_fields(self, event_details: dict) -> dict:
        """Extract stable typed fields from the flexible mission-state details payload."""
        return {
            'event_id': int(event_details.get('event_id', 0)),
            'zone_name': str(event_details.get('zone_name', self.current_zone)).strip(),
            'current_route_id': str(event_details.get('current_route_id', self.current_route_id)).strip(),
            'next_zone': str(event_details.get('next_zone', '')).strip(),
            'next_route_id': str(event_details.get('next_route_id', '')).strip(),
            'failure_reason': str(event_details.get('reason', event_details.get('failure_reason', ''))).strip(),
            'frame_region': str(event_details.get('frame_region', '')).strip(),
            'retry_count': int(event_details.get('retry_count', 0)),
            'duration_sec': float(event_details.get('duration_sec', 0.0) or 0.0),
            'elapsed_sec': float(event_details.get('elapsed_sec', 0.0) or 0.0),
            'capture_deadline': float(event_details.get('capture_deadline', 0.0) or 0.0),
            'state_since': float(self.state_since or 0.0),
            'class_names': list(self.class_names),
            'class_schema_hash': self.class_schema_hash,
        }

    def _emit_state(self, event: str, state: Optional[str] = None, details: Optional[dict] = None):
        """Publish a mission-state event without embedding historical zone summaries."""
        if state is not None:
            self.state = state
            self.state_since = self.clock.now_business_sec()
        self.event_counter += 1
        event_details = {
            'event_id': self.event_counter,
            'zone_name': self.current_zone,
            'current_step_id': str(self.current_step_id).strip(),
            'current_task_type': str(self.current_task_type).strip(),
            'current_task_objective': str(self.current_task_objective).strip(),
            **(details or {}),
        }
        if self.config['embed_zone_results_in_state']:
            event_details['zone_results'] = self.zone_results
        snapshot = MissionStateSnapshot(
            stamp=self.clock.now_business_sec(),
            state=self.state,
            event=event,
            route_index=self.route_index,
            route_total=len(self.route),
            current_zone=self.current_zone,
            current_route_id=self.current_route_id,
            class_names=list(self.class_names),
            class_schema_hash=self.class_schema_hash,
            details=event_details,
        )
        self.publishers.mission_state_json.publish(String(data=JsonCodec.dumps(snapshot.to_dict())))
        fixed = self._mission_state_fixed_fields(event_details)
        msg = MissionState()
        msg.header.stamp = self.clock.to_ros_time(snapshot.stamp)
        msg.header.frame_id = self.config['comparison_frame']
        msg.state = snapshot.state
        msg.event = snapshot.event
        msg.route_index = snapshot.route_index
        msg.route_total = snapshot.route_total
        msg.current_zone = snapshot.current_zone
        msg.schema_version = snapshot.schema_version
        msg.event_id = fixed['event_id']
        msg.zone_name = fixed['zone_name']
        msg.current_route_id = fixed['current_route_id']
        msg.next_zone = fixed['next_zone']
        msg.next_route_id = fixed['next_route_id']
        msg.failure_reason = fixed['failure_reason']
        msg.frame_region = fixed['frame_region']
        msg.retry_count = fixed['retry_count']
        msg.duration_sec = fixed['duration_sec']
        msg.elapsed_sec = fixed['elapsed_sec']
        msg.capture_deadline = fixed['capture_deadline']
        msg.state_since = fixed['state_since']
        msg.class_names = list(fixed['class_names'])
        msg.class_schema_hash = fixed['class_schema_hash']
        msg.details_json = JsonCodec.dumps(snapshot.details)
        self.publishers.mission_state_typed.publish(msg)
        self.event_writer.write({'type': 'mission_state', 'payload': snapshot.to_dict(), 'wall_stamp': self.clock.now_wall_sec()})
        self.execution_trace_writer.write({
            'wall_stamp': self.clock.now_wall_sec(),
            'stamp': snapshot.stamp,
            'event': event,
            'state': snapshot.state,
            'route_index': snapshot.route_index,
            'route_total': snapshot.route_total,
            'current_zone': snapshot.current_zone,
            'current_route_id': snapshot.current_route_id,
            'details': snapshot.details,
        })
        if event in ('navigation_retry', 'schema_mismatch', 'fault', 'mission_timeout', 'preflight_failed') or snapshot.state in ('FAULT', 'TIMEOUT'):
            self.recovery_trace_writer.write({
                'wall_stamp': self.clock.now_wall_sec(),
                'stamp': snapshot.stamp,
                'event': event,
                'state': snapshot.state,
                'current_zone': snapshot.current_zone,
                'current_route_id': snapshot.current_route_id,
                'details': snapshot.details,
            })

    def _publish_zone_capture(self, result):
        """Publish one zone-capture result on both dynamic and legacy lanes."""
        dynamic_payload = result.to_dict()
        legacy_payload = dynamic_zone_result_to_legacy_payload(dynamic_payload)
        self.publishers.zone_capture_json.publish(String(data=JsonCodec.dumps(legacy_payload)))
        msg = ZoneCapture()
        msg.header.stamp = self.clock.to_ros_time(result.capture_finished_at or self.clock.now_business_sec())
        msg.header.frame_id = self.config['comparison_frame']
        msg.zone_name = result.zone_name
        msg.route_id = str(dynamic_payload.get('route_id', '')).strip()
        msg.status = result.status
        msg.friendly = int(legacy_payload['friendly'])
        msg.enemy = int(legacy_payload['enemy'])
        msg.hostage = int(legacy_payload['hostage'])
        msg.capture_started_at = result.capture_started_at
        msg.capture_finished_at = result.capture_finished_at
        msg.frame_count = result.frame_count
        msg.frame_region = result.frame_region
        msg.failure_reason = result.failure_reason
        msg.schema_version = result.schema_version
        msg.class_schema_hash = str(dynamic_payload.get('class_schema_hash', '')).strip() or self.class_schema_hash
        self.publishers.zone_capture_typed.publish(msg)
        dynamic = ZoneCaptureDynamic()
        dynamic.header.stamp = msg.header.stamp
        dynamic.header.frame_id = msg.header.frame_id
        dynamic.zone_name = result.zone_name
        dynamic.route_id = str(dynamic_payload.get('route_id', '')).strip()
        dynamic.status = result.status
        dynamic.class_names = list(dynamic_payload['class_names'])
        dynamic.class_counts = [int(value) for value in dynamic_payload['class_counts']]
        dynamic.capture_started_at = result.capture_started_at
        dynamic.capture_finished_at = result.capture_finished_at
        dynamic.frame_count = result.frame_count
        dynamic.frame_region = result.frame_region
        dynamic.failure_reason = result.failure_reason
        dynamic.schema_version = result.schema_version
        dynamic.class_schema_hash = str(dynamic_payload.get('class_schema_hash', '')).strip() or self.class_schema_hash
        dynamic.task_type = str(dynamic_payload.get('task_type', '')).strip() or 'waypoint_capture'
        dynamic.objective_type = str(dynamic_payload.get('objective_type', '')).strip() or 'recon'
        dynamic.mission_outcome = str(dynamic_payload.get('mission_outcome', '')).strip()
        dynamic.task_metadata_json = JsonCodec.dumps(dynamic_payload.get('task_metadata', {}) or {})
        dynamic.position_estimates_json = JsonCodec.dumps(list(dynamic_payload.get('position_estimates', []) or []))
        dynamic.evidence_summary_json = JsonCodec.dumps(dynamic_payload.get('evidence_summary', {}) or {})
        dynamic.hazard_summary_json = JsonCodec.dumps(dynamic_payload.get('hazard_summary', {}) or {})
        dynamic.action_summary_json = JsonCodec.dumps(dynamic_payload.get('action_summary', {}) or {})
        self.publishers.zone_capture_dynamic.publish(dynamic)
        self.event_writer.write({
            'type': 'zone_capture',
            'payload': {'dynamic': dynamic_payload, 'legacy_projection': legacy_payload},
            'wall_stamp': self.clock.now_wall_sec(),
        })

    def _detections_callback(self, msg: DetectionArray):
        if msg.class_names:
            payload = {'class_names': list(msg.class_names), 'class_schema_hash': msg.class_schema_hash}
            if not self.schema_guard.validate_upstream_schema(payload, stream='detections'):
                return
        detections = []
        for item in msg.detections:
            frame_region = str(item.frame_region).strip()
            observed_position_type = str(getattr(item, 'observed_position_type', '') or '').strip()
            observed_position_label = str(getattr(item, 'observed_position_label', '') or '').strip()
            observed_position_x_m = float(getattr(item, 'observed_position_x_m', 0.0) or 0.0)
            observed_position_y_m = float(getattr(item, 'observed_position_y_m', 0.0) or 0.0)
            evidence_source = str(getattr(item, 'evidence_source', '') or '').strip() or str(msg.detector_type or '').strip()
            if not observed_position_type:
                observed_position_type = 'frame_region' if frame_region else 'bbox_center'
            if not observed_position_label and observed_position_type == 'frame_region':
                observed_position_label = frame_region
            detections.append(Detection(
                class_name=item.class_name,
                score=float(item.score),
                x1=int(item.x1), y1=int(item.y1), x2=int(item.x2), y2=int(item.y2),
                frame_region=frame_region,
                observed_position_type=observed_position_type,
                observed_position_label=observed_position_label,
                observed_position_x_m=observed_position_x_m,
                observed_position_y_m=observed_position_y_m,
                evidence_source=evidence_source,
            ))
        frame = DetectionFrame(
            stamp=msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else self.clock.now_business_sec(),
            frame_id=msg.header.frame_id,
            detector_type=msg.detector_type,
            schema_version=msg.schema_version,
            detections=detections,
            source_image_width=int(msg.source_image_width),
            source_image_height=int(msg.source_image_height),
            class_names=list(msg.class_names) or list(self.class_names),
            class_schema_hash=msg.class_schema_hash or self.class_schema_hash,
        )
        self.latest_detection_frame = frame
        if self.state == 'DWELL':
            self.capture_aggregator.feed(frame, self.current_capture_deadline)

    def _odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else self.clock.now_business_sec()
        orientation = msg.pose.pose.orientation
        pose = PoseSnapshot(
            stamp=stamp,
            frame_id=msg.header.frame_id,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw_rad=quaternion_to_yaw_rad(orientation.x, orientation.y, orientation.z, orientation.w),
            source='odometry',
        )
        self.arrival_evaluator.update_pose(pose)

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else self.clock.now_business_sec()
        orientation = msg.pose.pose.orientation
        pose = PoseSnapshot(
            stamp=stamp,
            frame_id=msg.header.frame_id,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw_rad=quaternion_to_yaw_rad(orientation.x, orientation.y, orientation.z, orientation.w),
            source='amcl_pose',
        )
        self.arrival_evaluator.update_pose(pose)

    def _refresh_pose_source(self) -> None:
        if self.tf_pose_source is None:
            return
        try:
            self.arrival_evaluator.update_pose(self.tf_pose_source.lookup())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException, tf2_ros.TimeoutException) as exc:
            now_mono = self.clock.now_monotonic_sec()
            if (now_mono - self.last_tf_warning_mono) >= 1.0:
                self.last_tf_warning_mono = now_mono
                self._publish_health('warn', 'tf_pose_lookup_failed', {
                    'target_frame': self.config['tf_target_frame'],
                    'source_frame': self.config['tf_source_frame'],
                    'error': str(exc),
                })

    def step(self):
        """Advance one mission tick through the executor."""
        self.executor.step()

    def _on_shutdown(self):
        try:
            self.nav_adapter.cancel(self.clock.now_business_sec())
            self.nav_adapter.close()
        except Exception:
            pass
        for subscriber in (self._navigation_runtime_status_sub, self._navigation_runtime_feedback_sub, self._navigation_runtime_result_sub):
            try:
                if subscriber is not None:
                    subscriber.unregister()
            except Exception:
                pass
        self.event_writer.close()
        self.execution_trace_writer.close()
        self.recovery_trace_writer.close()

    def spin(self):
        rate = rospy.Rate(self.config['publish_rate_hz'])
        while not rospy.is_shutdown():
            try:
                self.step()
            except Exception as exc:
                rospy.logerr_throttle(2.0, 'mission_manager step failed: %s', exc)
                self._publish_health('error', 'mission_step_failed', {'error': str(exc)})
                self._emit_state('fault', state='FAULT', details={'error': str(exc)})
            rate.sleep()

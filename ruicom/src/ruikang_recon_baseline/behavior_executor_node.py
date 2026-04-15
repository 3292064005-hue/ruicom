#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS node that bridges semantic behavior commands to downstream execution transports.

The node is lifecycle-aware and owns the following responsibilities:

1. Consume normalized mission behavior commands from ``recon/behavior_command``.
2. Publish downstream execution requests on configured transport topics.
3. Consume downstream execution receipts from configured result topics.
4. Publish normalized feedback receipts back to ``recon/behavior_feedback``.

It intentionally does *not* auto-succeed based on elapsed time. Terminal success
must come from a matching downstream receipt or from explicit timeout/cancel
paths.
"""

from __future__ import annotations

from typing import Dict, Optional

import rospy
from std_msgs.msg import String

from .behavior_actions import BehaviorActionCommand, BehaviorActionFeedback
from .behavior_executor_core import (
    BehaviorExecutorCore,
    DownstreamCancelEnvelope,
    DownstreamDispatchEnvelope,
    build_behavior_executor_specs,
    extract_command_id_from_payload_text,
)
from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, require_positive_float
from .lifecycle_protocol import decode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import HealthState
from .time_core import NodeClock


class BehaviorExecutorNode:
    """Lifecycle-aware external-execution bridge for mission behavior commands."""

    def __init__(self):
        rospy.init_node('behavior_executor_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self.core = BehaviorExecutorCore(action_specs=build_behavior_executor_specs(self.config))

        self.feedback_pub = rospy.Publisher(self.config['behavior_feedback_topic'], String, queue_size=20)
        self.state_pub = rospy.Publisher(self.config['behavior_executor_state_topic'], String, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.evidence_pub = rospy.Publisher(self.config['runtime_evidence_topic'], String, queue_size=10)
        self.command_sub = rospy.Subscriber(self.config['behavior_command_topic'], String, self._command_cb, queue_size=20)
        self.control_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_command_cb, queue_size=20) if self.config['lifecycle_managed'] else None

        self.dispatch_pubs: Dict[str, rospy.Publisher] = {}
        self.cancel_pubs: Dict[str, rospy.Publisher] = {}
        self.result_subs: Dict[str, rospy.Subscriber] = {}
        self._wire_transport_catalog()

        self._last_feedback_signature = ()
        self._last_state_signature = ()
        self._last_health_signature = ()
        self._last_health_emit_sec = 0.0
        self._event_seq = 0
        self._publish_health('ok', 'node_ready', self._health_details())

    def _read_config(self) -> dict:
        """Read, normalize and validate ROS parameters.

        Raises:
            ConfigurationError: If one required topic or timing parameter is
                missing or invalid.
        """
        config = {
            'behavior_command_topic': str(rospy.get_param('~behavior_command_topic', 'recon/behavior_command')).strip() or 'recon/behavior_command',
            'behavior_feedback_topic': str(rospy.get_param('~behavior_feedback_topic', 'recon/behavior_feedback')).strip() or 'recon/behavior_feedback',
            'behavior_executor_state_topic': str(rospy.get_param('~behavior_executor_state_topic', 'recon/behavior_executor/state')).strip() or 'recon/behavior_executor/state',
            'runtime_evidence_topic': str(rospy.get_param('~runtime_evidence_topic', 'recon/runtime/evidence')).strip() or 'recon/runtime/evidence',
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 10.0)),
            'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz', 1.0)),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower() or 'ros',
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'runtime_grade': str(rospy.get_param('~runtime_grade', 'integration')).strip().lower() or 'integration',
            'default_pending_to_active_sec': float(rospy.get_param('~default_pending_to_active_sec', 0.2)),
            'default_dispatch_topic': str(rospy.get_param('~default_dispatch_topic', 'recon/platform/behavior_execution/request')).strip() or 'recon/platform/behavior_execution/request',
            'default_result_topic': str(rospy.get_param('~default_result_topic', 'recon/platform/behavior_execution/result')).strip() or 'recon/platform/behavior_execution/result',
            'default_cancel_topic': str(rospy.get_param('~default_cancel_topic', 'recon/platform/behavior_execution/cancel')).strip() or 'recon/platform/behavior_execution/cancel',
            'require_downstream_dispatch_subscriber': bool(rospy.get_param('~require_downstream_dispatch_subscriber', False)),
            'require_downstream_result_publisher': bool(rospy.get_param('~require_downstream_result_publisher', False)),
            'action_specs': rospy.get_param('~action_specs', {}),
        }
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['health_heartbeat_hz'] = require_positive_float('health_heartbeat_hz', config['health_heartbeat_hz'])
        if config['default_pending_to_active_sec'] < 0.0:
            raise ConfigurationError('default_pending_to_active_sec must be >= 0')
        if config['time_source_mode'] not in ('ros', 'wall'):
            raise ConfigurationError('behavior_executor_node time_source_mode must be ros or wall')
        if config['runtime_grade'] not in ('integration', 'contract', 'reference', 'field'):
            raise ConfigurationError('behavior_executor_node runtime_grade must be one of integration, contract, reference, field')
        return config

    def _wire_transport_catalog(self) -> None:
        for spec in self.core._action_specs.values():
            if spec.dispatch_topic not in self.dispatch_pubs:
                self.dispatch_pubs[spec.dispatch_topic] = rospy.Publisher(spec.dispatch_topic, String, queue_size=20)
            if spec.cancel_topic not in self.cancel_pubs:
                self.cancel_pubs[spec.cancel_topic] = rospy.Publisher(spec.cancel_topic, String, queue_size=20)
            if spec.result_topic not in self.result_subs:
                self.result_subs[spec.result_topic] = rospy.Subscriber(
                    spec.result_topic,
                    String,
                    self._result_cb,
                    callback_args=spec.result_topic,
                    queue_size=20,
                )

    def _emit_runtime_event(self, event_type: str, details: Optional[dict] = None) -> None:
        self._event_seq += 1
        payload = {
            'stamp': self.clock.now_business_sec(),
            'schema_version': SCHEMA_VERSION,
            'event_type': str(event_type).strip().lower(),
            'event_seq': int(self._event_seq),
            'details': dict(details or {}),
        }
        payload['details'].setdefault('runtime_grade', self.config['runtime_grade'])
        self.evidence_pub.publish(String(data=JsonCodec.dumps(payload)))

    def _feedback_signature(self, feedback: BehaviorActionFeedback) -> tuple:
        return (
            feedback.command_id,
            feedback.status,
            round(float(feedback.stamp), 6),
            JsonCodec.dumps(dict(feedback.details or {})),
            str(feedback.source or ''),
        )

    def _publish_feedback(self, feedback: BehaviorActionFeedback) -> None:
        self.feedback_pub.publish(String(data=JsonCodec.dumps(feedback.to_dict())))
        self._last_feedback_signature = self._feedback_signature(feedback)

    @staticmethod
    def _connections(endpoint) -> int:
        getter = getattr(endpoint, 'get_num_connections', None)
        if getter is None:
            return 0
        try:
            return int(getter())
        except Exception:
            return 0

    def _transport_catalog_summary(self) -> dict:
        dispatch_topics = sorted(self.dispatch_pubs.keys())
        result_topics = sorted(self.result_subs.keys())
        cancel_topics = sorted(self.cancel_pubs.keys())
        dispatch_connections = {topic: self._connections(pub) for topic, pub in self.dispatch_pubs.items()}
        result_connections = {topic: self._connections(sub) for topic, sub in self.result_subs.items()}
        return {
            'dispatch_topics': dispatch_topics,
            'result_topics': result_topics,
            'cancel_topics': cancel_topics,
            'dispatch_connections': dispatch_connections,
            'result_connections': result_connections,
            'downstream_binding_declared': bool(dispatch_topics) and bool(result_topics) and bool(cancel_topics),
            'downstream_dispatch_ready': all(count > 0 for count in dispatch_connections.values()) if self.config['require_downstream_dispatch_subscriber'] else True,
            'downstream_result_ready': all(count > 0 for count in result_connections.values()) if self.config['require_downstream_result_publisher'] else True,
        }

    def _health_details(self) -> Dict[str, object]:
        catalog = self._transport_catalog_summary()
        details = {
            'runtime_grade': self.config['runtime_grade'],
            'command_topic_declared': bool(self.config['behavior_command_topic']),
            'feedback_topic_declared': bool(self.config['behavior_feedback_topic']),
            'action_catalog_ready': bool(self.core.supported_actions()),
            'supported_actions': list(self.core.supported_actions()),
            **catalog,
            **self.runtime.snapshot(),
        }
        details.update(self.core.runtime_summary(now_sec=self.clock.now_business_sec()))
        return details

    def _publish_health(self, status: str, message: str, details: Optional[dict] = None) -> None:
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'behavior_executor_node',
            'status': str(status).strip(),
            'message': str(message).strip(),
            'schema_version': SCHEMA_VERSION,
            'details': dict(details or {}),
        }
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = self.clock.now_ros_time()
        typed.header.frame_id = self.config['health_frame_id']
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(payload.get('details', {}))
        self.health_typed_pub.publish(typed)
        self._last_health_emit_sec = float(payload['stamp'])
        self._last_health_signature = (payload['status'], payload['message'], JsonCodec.dumps(payload['details']))

    def _publish_state(self) -> None:
        summary = self.core.runtime_summary(now_sec=self.clock.now_business_sec())
        payload = {
            'stamp': self.clock.now_business_sec(),
            'schema_version': SCHEMA_VERSION,
            'runtime_grade': self.config['runtime_grade'],
            'runtime': self.runtime.snapshot(),
            'executor': summary,
            'transport': self._transport_catalog_summary(),
        }
        signature = (
            payload['runtime']['runtime_state'],
            JsonCodec.dumps(payload['executor']),
            JsonCodec.dumps(payload['transport']),
        )
        if signature == self._last_state_signature:
            return
        self.state_pub.publish(String(data=JsonCodec.dumps(payload)))
        self._last_state_signature = signature

    def _coerce_command(self, payload: Dict[str, object]) -> BehaviorActionCommand:
        command = BehaviorActionCommand(
            command_id=str(payload.get('command_id', '')).strip(),
            action_type=str(payload.get('action_type', '')).strip(),
            route_id=str(payload.get('route_id', '')).strip(),
            zone_name=str(payload.get('zone_name', '')).strip(),
            step_id=str(payload.get('step_id', '')).strip(),
            task_type=str(payload.get('task_type', '')).strip(),
            objective_type=str(payload.get('objective_type', '')).strip(),
            metadata=dict(payload.get('metadata', {}) or {}),
            issued_at=float(payload.get('issued_at', 0.0) or 0.0),
            timeout_sec=float(payload.get('timeout_sec', 0.0) or 0.0),
        )
        if not command.command_id:
            raise ConfigurationError('behavior command requires command_id')
        if not command.action_type:
            raise ConfigurationError('behavior command requires action_type')
        if not command.task_type:
            raise ConfigurationError('behavior command requires task_type')
        if float(command.timeout_sec) <= 0.0:
            raise ConfigurationError('behavior command requires timeout_sec > 0')
        return command

    def _reject_command(self, *, command_id: str, reason: str) -> None:
        feedback = BehaviorActionFeedback(
            command_id=str(command_id).strip() or 'unknown',
            status='FAILED',
            stamp=self.clock.now_business_sec(),
            details={'reason': str(reason).strip()},
            source='behavior_executor',
        )
        self._publish_feedback(feedback)

    def _publish_dispatch(self, envelope: DownstreamDispatchEnvelope) -> None:
        publisher = self.dispatch_pubs[envelope.dispatch_topic]
        publisher.publish(String(data=JsonCodec.dumps(envelope.payload)))
        self._emit_runtime_event('downstream_dispatch_published', {
            'command_id': envelope.command_id,
            'action_type': envelope.action_type,
            'dispatch_topic': envelope.dispatch_topic,
            'result_topic': envelope.result_topic,
            'cancel_topic': envelope.cancel_topic,
        })

    def _publish_cancel(self, envelope: DownstreamCancelEnvelope) -> None:
        publisher = self.cancel_pubs[envelope.cancel_topic]
        publisher.publish(String(data=JsonCodec.dumps(envelope.payload)))
        self._emit_runtime_event('downstream_cancel_published', {
            'command_id': envelope.command_id,
            'action_type': envelope.action_type,
            'cancel_topic': envelope.cancel_topic,
        })

    def _command_cb(self, msg: String) -> None:
        command_id = extract_command_id_from_payload_text(msg.data)
        if self.runtime.lifecycle_managed and not self.runtime.processing_allowed:
            self._reject_command(command_id=command_id, reason='runtime_not_active')
            return
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._publish_health('warn', 'malformed_behavior_command', {**self._health_details(), 'error': str(exc)})
            return
        if not isinstance(payload, dict):
            self._publish_health('warn', 'malformed_behavior_command', {**self._health_details(), 'error': 'payload_not_object'})
            return
        command_id = str(payload.get('command_id', '')).strip()
        try:
            command = self._coerce_command(payload)
            dispatch = self.core.accept(command, now_sec=self.clock.now_business_sec())
            self._publish_dispatch(dispatch)
            self._emit_runtime_event('behavior_command_accepted', {'command': command.to_dict(), 'dispatch_topic': dispatch.dispatch_topic, 'result_topic': dispatch.result_topic})
            self._publish_health('ok', 'behavior_command_accepted', self._health_details())
        except Exception as exc:
            self._reject_command(command_id=command_id, reason=str(exc))
            self._emit_runtime_event('behavior_command_rejected', {'command_id': command_id, 'reason': str(exc)})
            self._publish_health('warn', 'behavior_command_rejected', {**self._health_details(), 'error': str(exc), 'command_id': command_id})

    def _result_cb(self, msg: String, topic_name: str) -> None:
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._publish_health('warn', 'malformed_downstream_result', {**self._health_details(), 'error': str(exc), 'result_topic': topic_name})
            return
        if not isinstance(payload, dict):
            self._publish_health('warn', 'malformed_downstream_result', {**self._health_details(), 'error': 'payload_not_object', 'result_topic': topic_name})
            return
        try:
            feedback = self.core.observe_result(
                command_id=str(payload.get('command_id', '')).strip(),
                status=str(payload.get('status', '')).strip(),
                now_sec=float(payload.get('stamp', 0.0) or 0.0) or self.clock.now_business_sec(),
                details=dict(payload.get('details', {}) or {}),
                source=str(payload.get('source', '')).strip() or topic_name,
            )
        except Exception as exc:
            self._publish_health('warn', 'downstream_result_rejected', {**self._health_details(), 'error': str(exc), 'result_topic': topic_name})
            return
        if feedback is None:
            return
        self._emit_runtime_event('downstream_result_observed', {'result_topic': topic_name, 'feedback': feedback.to_dict()})

    def _control_command_cb(self, msg: String) -> None:
        try:
            control = decode_lifecycle_control(msg.data)
        except Exception:
            return
        if control.target and control.target != 'behavior_executor_node':
            return
        result = self.runtime.apply(control.command)
        details = self._health_details()
        details.update({'control_command': control.command, 'control_message': result.message})
        if result.changed and result.state in ('IDLE', 'PAUSED', 'SHUTDOWN'):
            cancel_wire = self.core.build_cancel_envelope(now_sec=self.clock.now_business_sec(), reason='lifecycle_' + control.command)
            if cancel_wire is not None:
                self._publish_cancel(cancel_wire)
            feedback = self.core.cancel(now_sec=self.clock.now_business_sec(), reason='lifecycle_' + control.command)
            if feedback is not None:
                self._publish_feedback(feedback)
                self._emit_runtime_event('behavior_command_cancelled', {'feedback': feedback.to_dict()})
        self._publish_health('ok' if result.accepted else 'warn', result.message, details)
        self._publish_state()

    def spin(self) -> None:
        """Run the executor loop until ROS shutdown."""
        rate = rospy.Rate(self.config['publish_rate_hz'])
        heartbeat_interval = 1.0 / self.config['health_heartbeat_hz']
        while not rospy.is_shutdown():
            now_sec = self.clock.now_business_sec()
            active = self.core.active
            if active is not None:
                feedback = self.core.poll(now_sec)
                signature = self._feedback_signature(feedback)
                if signature != self._last_feedback_signature:
                    self._publish_feedback(feedback)
                    if feedback.status in ('SUCCEEDED', 'FAILED', 'TIMEOUT'):
                        self._emit_runtime_event('behavior_command_completed', {'feedback': feedback.to_dict()})
                        self._publish_health('ok' if feedback.status == 'SUCCEEDED' else 'warn', 'behavior_command_' + feedback.status.lower(), self._health_details())
                        self.core.clear_terminal()
            self._publish_state()
            details = self._health_details()
            health_signature = ('ok', 'behavior_executor_ready', JsonCodec.dumps(details))
            if health_signature != self._last_health_signature or (now_sec - self._last_health_emit_sec) >= heartbeat_interval:
                self._publish_health('ok', 'behavior_executor_ready', details)
            rate.sleep()

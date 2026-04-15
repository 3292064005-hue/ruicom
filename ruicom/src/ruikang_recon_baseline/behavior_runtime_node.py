#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Repository-managed downstream behavior execution runtime.

This node consumes normalized execution requests from ``behavior_executor_node``
and performs concrete repository-visible work:

* emits velocity commands onto the safety ingress bus;
* emits structured actuator commands for integration adapters;
* observes platform bridge state and typed detections;
* publishes explicit terminal execution receipts.

It is intentionally not a timer-only simulator. Success requires observable
platform or detection evidence according to per-action policy.
"""

from __future__ import annotations

from collections import Counter
from typing import Dict, Optional

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .behavior_runtime_core import BehaviorRuntimeCore, build_behavior_runtime_specs
from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, require_positive_float
from .lifecycle_protocol import decode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import DetectionArray, HealthState
from .time_core import NodeClock


class BehaviorRuntimeNode:
    def __init__(self):
        rospy.init_node('behavior_runtime_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self.core = BehaviorRuntimeCore(action_specs=build_behavior_runtime_specs(self.config))
        self.request_sub = rospy.Subscriber(self.config['request_topic'], String, self._request_cb, queue_size=20)
        self.cancel_sub = rospy.Subscriber(self.config['cancel_topic'], String, self._cancel_cb, queue_size=20)
        self.platform_state_sub = rospy.Subscriber(self.config['platform_bridge_state_topic'], String, self._platform_state_cb, queue_size=20) if self.config['platform_bridge_state_topic'] else None
        self.detections_sub = rospy.Subscriber(self.config['detections_topic'], DetectionArray, self._detections_cb, queue_size=20) if self.config['detections_topic'] else None
        self.actuator_state_sub = rospy.Subscriber(self.config['actuator_state_topic'], String, self._actuator_state_cb, queue_size=20) if self.config['actuator_state_topic'] else None
        self.result_pub = rospy.Publisher(self.config['result_topic'], String, queue_size=20)
        self.command_pub = rospy.Publisher(self.config['command_output_topic'], Twist, queue_size=20)
        self.actuator_pub = rospy.Publisher(self.config['actuator_command_topic'], String, queue_size=20)
        self.state_pub = rospy.Publisher(self.config['behavior_runtime_state_topic'], String, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.evidence_pub = rospy.Publisher(self.config['runtime_evidence_topic'], String, queue_size=10)
        self.control_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_cb, queue_size=20) if self.config['lifecycle_managed'] else None
        self._last_platform_summary: Dict[str, object] = {}
        self._last_platform_stamp = 0.0
        self._last_detection_stamp = 0.0
        self._last_detection_counts: Dict[str, int] = {}
        self._last_state_signature = ()
        self._last_health_signature = ()
        self._last_health_emit_sec = 0.0
        self._event_seq = 0
        self._publish_health('ok', 'node_ready', self._health_details())

    def _read_config(self) -> dict:
        config = {
            'request_topic': str(rospy.get_param('~request_topic', 'recon/platform/behavior_execution/request')).strip() or 'recon/platform/behavior_execution/request',
            'result_topic': str(rospy.get_param('~result_topic', 'recon/platform/behavior_execution/result')).strip() or 'recon/platform/behavior_execution/result',
            'cancel_topic': str(rospy.get_param('~cancel_topic', 'recon/platform/behavior_execution/cancel')).strip() or 'recon/platform/behavior_execution/cancel',
            'command_output_topic': str(rospy.get_param('~command_output_topic', 'cmd_vel_raw')).strip() or 'cmd_vel_raw',
            'actuator_command_topic': str(rospy.get_param('~actuator_command_topic', 'recon/platform/behavior_execution/actuator_command')).strip() or 'recon/platform/behavior_execution/actuator_command',
            'platform_bridge_state_topic': str(rospy.get_param('~platform_bridge_state_topic', 'recon/platform/bridge_state')).strip() or 'recon/platform/bridge_state',
            'detections_topic': str(rospy.get_param('~detections_topic', 'recon/detections')).strip() or 'recon/detections',
            'behavior_runtime_state_topic': str(rospy.get_param('~behavior_runtime_state_topic', 'recon/behavior_runtime/state')).strip() or 'recon/behavior_runtime/state',
            'actuator_state_topic': str(rospy.get_param('~actuator_state_topic', 'recon/platform/behavior_execution/actuator_state')).strip() or 'recon/platform/behavior_execution/actuator_state',
            'runtime_evidence_topic': str(rospy.get_param('~runtime_evidence_topic', 'recon/runtime/evidence')).strip() or 'recon/runtime/evidence',
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 15.0)),
            'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz', 1.0)),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower() or 'ros',
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'runtime_grade': str(rospy.get_param('~runtime_grade', 'integration')).strip().lower() or 'integration',
            'default_command_hold_sec': float(rospy.get_param('~default_command_hold_sec', 0.5)),
            'default_settle_sec': float(rospy.get_param('~default_settle_sec', 0.2)),
            'detection_age_sec': float(rospy.get_param('~detection_age_sec', 1.5)),
            'platform_state_age_sec': float(rospy.get_param('~platform_state_age_sec', 1.5)),
            'action_specs': rospy.get_param('~action_specs', {}),
        }
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['health_heartbeat_hz'] = require_positive_float('health_heartbeat_hz', config['health_heartbeat_hz'])
        config['detection_age_sec'] = require_positive_float('detection_age_sec', config['detection_age_sec'])
        config['platform_state_age_sec'] = require_positive_float('platform_state_age_sec', config['platform_state_age_sec'])
        if config['time_source_mode'] not in ('ros', 'wall'):
            raise ConfigurationError('behavior_runtime_node time_source_mode must be ros or wall')
        if config['runtime_grade'] not in ('integration', 'contract', 'reference', 'field'):
            raise ConfigurationError('behavior_runtime_node runtime_grade must be integration, contract, reference or field')
        return config

    def _connections(self, endpoint) -> int:
        getter = getattr(endpoint, 'get_num_connections', None)
        if getter is None:
            return 0
        try:
            return int(getter())
        except Exception:
            return 0

    def _emit_event(self, event_type: str, details: Optional[dict] = None) -> None:
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

    def _platform_state_cb(self, msg: String) -> None:
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        if isinstance(payload, dict):
            self._last_platform_summary = payload
            self._last_platform_stamp = self.clock.now_business_sec()

    def _detections_cb(self, msg: DetectionArray) -> None:
        counts = Counter()
        for item in getattr(msg, 'detections', []) or []:
            counts[str(item.class_name).strip()] += 1
        self._last_detection_counts = {str(name): int(value) for name, value in counts.items()}
        self._last_detection_stamp = self.clock.now_business_sec()

    def _detection_fresh(self) -> bool:
        return self._last_detection_stamp > 0.0 and (self.clock.now_business_sec() - self._last_detection_stamp) <= float(self.config['detection_age_sec'])

    def _platform_fresh(self) -> bool:
        return self._last_platform_stamp > 0.0 and (self.clock.now_business_sec() - self._last_platform_stamp) <= float(self.config['platform_state_age_sec'])

    def _actuator_state_fresh(self) -> bool:
        return self._last_actuator_state_stamp > 0.0 and (self.clock.now_business_sec() - self._last_actuator_state_stamp) <= float(self.config['platform_state_age_sec'])

    def _health_details(self) -> Dict[str, object]:
        return {
            'runtime_grade': self.config['runtime_grade'],
            'request_topic_declared': bool(self.config['request_topic']),
            'result_topic_declared': bool(self.config['result_topic']),
            'cancel_topic_declared': bool(self.config['cancel_topic']),
            'command_output_topic_declared': bool(self.config['command_output_topic']),
            'execution_catalog_ready': bool(self.core.supported_actions()),
            'supported_actions': list(self.core.supported_actions()),
            'request_subscriber_bound': self._connections(self.request_sub) >= 0,
            'result_publisher_bound': self._connections(self.result_pub) >= 0,
            'platform_state_fresh': self._platform_fresh() if self.config['platform_bridge_state_topic'] else True,
            'detection_input_fresh': self._detection_fresh() if self.config['detections_topic'] else True,
            'actuator_state_fresh': self._actuator_state_fresh() if self.config['actuator_state_topic'] else True,
            'platform_state_connections': self._connections(self.platform_state_sub) if self.platform_state_sub is not None else 0,
            'detections_connections': self._connections(self.detections_sub) if self.detections_sub is not None else 0,
            'actuator_state_connections': self._connections(self.actuator_state_sub) if self.actuator_state_sub is not None else 0,
            'runtime_state': self.runtime.state,
            'processing_allowed': bool(self.runtime.processing_allowed),
            'has_active_command': bool(self.core.active is not None),
        }

    def _publish_health(self, status: str, message: str, details: Optional[dict] = None) -> None:
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'behavior_runtime_node',
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
        typed.details_json = JsonCodec.dumps(payload['details'])
        self.health_typed_pub.publish(typed)
        self._last_health_signature = (payload['status'], payload['message'], JsonCodec.dumps(payload['details']))
        self._last_health_emit_sec = float(payload['stamp'])

    def _publish_state(self) -> None:
        payload = {
            'stamp': self.clock.now_business_sec(),
            'schema_version': SCHEMA_VERSION,
            'runtime_grade': self.config['runtime_grade'],
            'runtime': self.runtime.snapshot(),
            'health': self._health_details(),
            'active_command': {
                'command_id': self.core.active.command_id,
                'action_type': self.core.active.action_type,
                'task_type': self.core.active.task_type,
            } if self.core.active is not None else {},
        }
        signature = (payload['runtime']['runtime_state'], JsonCodec.dumps(payload['active_command']), JsonCodec.dumps(payload['health']))
        if signature == self._last_state_signature:
            return
        self.state_pub.publish(String(data=JsonCodec.dumps(payload)))
        self._last_state_signature = signature

    def _publish_result(self, payload: Dict[str, object]) -> None:
        self.result_pub.publish(String(data=JsonCodec.dumps(payload)))

    def _publish_command(self, payload: Dict[str, object]) -> None:
        velocity = dict(payload.get('velocity', {}) or {})
        msg = Twist()
        msg.linear.x = float(velocity.get('linear_x', 0.0) or 0.0)
        msg.linear.y = float(velocity.get('linear_y', 0.0) or 0.0)
        msg.angular.z = float(velocity.get('angular_z', 0.0) or 0.0)
        self.command_pub.publish(msg)

    def _publish_actuator(self, payload: Dict[str, object]) -> None:
        self.actuator_pub.publish(String(data=JsonCodec.dumps(payload)))

    def _extract_command_id(self, raw_text: str) -> str:
        try:
            payload = JsonCodec.loads(raw_text)
        except Exception:
            return ''
        if not isinstance(payload, dict):
            return ''
        command = dict(payload.get('command', {}) or {})
        if command:
            return str(command.get('command_id', '')).strip()
        return str(payload.get('command_id', '')).strip()

    def _request_cb(self, msg: String) -> None:
        command_id = self._extract_command_id(msg.data)
        if self.runtime.lifecycle_managed and not self.runtime.processing_allowed:
            self._publish_result({'command_id': command_id, 'status': 'FAILED', 'stamp': self.clock.now_business_sec(), 'details': {'reason': 'runtime_not_active'}, 'source': 'behavior_runtime'})
            return
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._publish_health('warn', 'malformed_behavior_runtime_request', {**self._health_details(), 'error': str(exc)})
            return
        if not isinstance(payload, dict):
            self._publish_health('warn', 'malformed_behavior_runtime_request', {**self._health_details(), 'error': 'payload_not_object'})
            return
        try:
            dispatch = self.core.accept_request(payload, now_sec=self.clock.now_business_sec())
        except Exception as exc:
            command = dict(payload.get('command', {}) or {})
            self._publish_result({'command_id': str(command.get('command_id', '')).strip(), 'status': 'FAILED', 'stamp': self.clock.now_business_sec(), 'details': {'reason': str(exc)}, 'source': 'behavior_runtime'})
            self._publish_health('warn', 'behavior_runtime_request_rejected', {**self._health_details(), 'error': str(exc)})
            return
        self._publish_command(dispatch.command_payload)
        self._publish_actuator(dispatch.actuator_payload)
        self._emit_event('behavior_runtime_request_accepted', {'command_id': dispatch.command_id, 'action_type': dispatch.action_type})
        self._publish_health('ok', 'behavior_runtime_request_accepted', self._health_details())

    def _actuator_state_cb(self, msg: String) -> None:
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        if isinstance(payload, dict):
            self._last_actuator_state = payload
            self._last_actuator_state_stamp = self.clock.now_business_sec()

    def _cancel_cb(self, msg: String) -> None:
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        feedback = self.core.cancel(now_sec=self.clock.now_business_sec(), reason=str(payload.get('reason', 'cancelled')).strip() or 'cancelled')
        if feedback is not None:
            self._publish_result(feedback)
            self._emit_event('behavior_runtime_cancelled', {'feedback': feedback})

    def _control_cb(self, msg: String) -> None:
        try:
            envelope = decode_lifecycle_control(msg.data)
        except Exception:
            return
        if envelope.target and envelope.target != 'behavior_runtime_node':
            return
        result = self.runtime.apply(envelope.command)
        if result.changed and result.state in ('IDLE', 'PAUSED', 'SHUTDOWN'):
            feedback = self.core.cancel(now_sec=self.clock.now_business_sec(), reason='lifecycle_' + envelope.command)
            if feedback is not None:
                self._publish_result(feedback)
        self._publish_health('ok' if result.accepted else 'warn', result.message, self._health_details())
        self._publish_state()

    def spin(self) -> None:
        rate = rospy.Rate(self.config['publish_rate_hz'])
        heartbeat_interval = 1.0 / self.config['health_heartbeat_hz']
        while not rospy.is_shutdown():
            now_sec = self.clock.now_business_sec()
            command_payload = self.core.command_tick_payload(now_sec=now_sec)
            if command_payload is not None:
                self._publish_command(command_payload)
            result = self.core.evaluate(
                now_sec=now_sec,
                platform_summary=dict(self._last_platform_summary.get('details', self._last_platform_summary) if isinstance(self._last_platform_summary, dict) else {}),
                detection_counts=dict(self._last_detection_counts),
                detection_fresh=self._detection_fresh(),
                actuator_state=dict(self._last_actuator_state),
                actuator_state_fresh=self._actuator_state_fresh(),
            )
            if result is not None:
                self._publish_result(result)
                self._emit_event('behavior_runtime_completed', {'feedback': result})
                self.core.pop_terminal()
            else:
                pending = self.core.pop_terminal()
                if pending is not None:
                    self._publish_result(pending)
            self._publish_state()
            details = self._health_details()
            signature = ('ok', 'behavior_runtime_ready', JsonCodec.dumps(details))
            if signature != self._last_health_signature or (now_sec - self._last_health_emit_sec) >= heartbeat_interval:
                self._publish_health('ok', 'behavior_runtime_ready', details)
            rate.sleep()


def main() -> int:
    node = BehaviorRuntimeNode()
    node.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

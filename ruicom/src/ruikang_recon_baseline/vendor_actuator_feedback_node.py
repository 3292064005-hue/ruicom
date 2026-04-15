#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Dict, Optional
import rospy
from std_msgs.msg import String
from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, require_positive_float
from .lifecycle_protocol import decode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import HealthState
from .time_core import NodeClock
from .vendor_actuator_feedback_core import VendorActuatorFeedbackCore


def extract_command_id_from_payload_text(text: str) -> str:
    try:
        payload = JsonCodec.loads(text)
    except Exception:
        return ''
    return str((payload or {}).get('command_id', '')).strip()


class VendorActuatorFeedbackNode:
    def __init__(self):
        rospy.init_node('vendor_actuator_feedback_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self.core = VendorActuatorFeedbackCore()
        self._last_health_signature = ()
        self._last_health_emit_sec = 0.0
        self._last_result_sec = 0.0
        self.feedback_pub = rospy.Publisher(self.config['actuator_feedback_topic'], String, queue_size=20, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.command_sub = rospy.Subscriber(self.config['actuator_command_topic'], String, self._command_cb, queue_size=20)
        self.result_sub = rospy.Subscriber(self.config['actuator_result_topic'], String, self._result_cb, queue_size=20)
        self.cancel_sub = rospy.Subscriber(self.config['cancel_topic'], String, self._cancel_cb, queue_size=20)
        self.control_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_cb, queue_size=20) if self.config['lifecycle_managed'] else None
        self._publish_health('ok', 'node_ready', self._health_details())

    def _read_config(self) -> dict:
        config = {
            'actuator_command_topic': str(rospy.get_param('~actuator_command_topic', 'recon/platform/vendor/actuator_command')).strip() or 'recon/platform/vendor/actuator_command',
            'actuator_feedback_topic': str(rospy.get_param('~actuator_feedback_topic', 'recon/platform/vendor/actuator_feedback')).strip() or 'recon/platform/vendor/actuator_feedback',
            'actuator_result_topic': str(rospy.get_param('~actuator_result_topic', 'recon/platform/vendor/actuator_result_raw')).strip() or 'recon/platform/vendor/actuator_result_raw',
            'cancel_topic': str(rospy.get_param('~cancel_topic', 'recon/platform/behavior_execution/cancel')).strip() or 'recon/platform/behavior_execution/cancel',
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 15.0) or 15.0),
            'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz', 1.0) or 1.0),
            'result_timeout_sec': float(rospy.get_param('~result_timeout_sec', 1.0) or 1.0),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower() or 'ros',
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'runtime_grade': str(rospy.get_param('~runtime_grade', 'integration')).strip().lower() or 'integration',
        }
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['health_heartbeat_hz'] = require_positive_float('health_heartbeat_hz', config['health_heartbeat_hz'])
        config['result_timeout_sec'] = require_positive_float('result_timeout_sec', config['result_timeout_sec'])
        if config['time_source_mode'] not in ('ros', 'wall'):
            raise ConfigurationError('vendor_actuator_feedback_node time_source_mode must be ros or wall')
        return config

    @staticmethod
    def _connections(endpoint) -> int:
        getter = getattr(endpoint, 'get_num_connections', None)
        if getter is None:
            return 0
        try:
            return int(getter())
        except Exception:
            return 0

    def _health_details(self) -> Dict[str, object]:
        return {
            'runtime_grade': self.config['runtime_grade'],
            'actuator_command_topic_declared': bool(self.config['actuator_command_topic']),
            'actuator_feedback_topic_declared': bool(self.config['actuator_feedback_topic']),
            'actuator_result_topic_declared': bool(self.config['actuator_result_topic']),
            'command_input_bound': self._connections(self.command_sub) > 0,
            'actuator_result_bound': self._connections(self.result_sub) > 0,
            'actuator_feedback_output_bound': self._connections(self.feedback_pub) > 0,
            **self.runtime.snapshot(),
            'has_active_command': bool(self.core.active is not None),
        }

    def _publish_health(self, status: str, message: str, details: Optional[dict] = None) -> None:
        payload = {'stamp': self.clock.now_business_sec(), 'node': 'vendor_actuator_feedback_node', 'status': str(status).strip(), 'message': str(message).strip(), 'schema_version': SCHEMA_VERSION, 'details': dict(details or {})}
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState(); typed.header.stamp = self.clock.now_ros_time(); typed.header.frame_id = self.config['health_frame_id']; typed.node = payload['node']; typed.status = payload['status']; typed.message = payload['message']; typed.schema_version = payload['schema_version']; typed.details_json = JsonCodec.dumps(payload['details']); self.health_typed_pub.publish(typed)

    def _publish_terminal(self, payload: dict) -> None:
        self.feedback_pub.publish(String(data=JsonCodec.dumps(payload)))

    def _command_cb(self, msg: String) -> None:
        command_id = extract_command_id_from_payload_text(msg.data)
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._publish_health('warn', 'malformed_vendor_actuator_command', {**self._health_details(), 'error': str(exc)})
            return
        if self.runtime.lifecycle_managed and not self.runtime.is_active:
            self._publish_terminal({'command_id': command_id, 'action_type': str(payload.get('action_type', '')).strip(), 'task_type': str(payload.get('task_type', '')).strip(), 'status': 'FAILED', 'stamp': self.clock.now_business_sec(), 'details': {'reason': 'runtime_not_active'}, 'source': 'vendor_actuator_feedback'})
            return
        try:
            self.core.accept(payload, now_sec=self.clock.now_business_sec())
        except Exception as exc:
            self._publish_terminal({'command_id': command_id, 'action_type': str(payload.get('action_type', '')).strip(), 'task_type': str(payload.get('task_type', '')).strip(), 'status': 'FAILED', 'stamp': self.clock.now_business_sec(), 'details': {'reason': str(exc)}, 'source': 'vendor_actuator_feedback'})
            self._publish_health('warn', 'vendor_actuator_command_rejected', {**self._health_details(), 'error': str(exc), 'command_id': command_id})
            return
        self._publish_health('ok', 'vendor_actuator_feedback_ready', self._health_details())

    def _result_cb(self, msg: String) -> None:
        self._last_result_sec = self.clock.now_business_sec()
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._publish_health('warn', 'malformed_actuator_result_raw', {**self._health_details(), 'error': str(exc)})
            return
        self.core.observe_result(payload, now_sec=self.clock.now_business_sec())

    def _cancel_cb(self, msg: String) -> None:
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        command_id = str(payload.get('command_id', '')).strip()
        active = self.core.active
        if active is None or active.command_id != command_id:
            return
        terminal = self.core.cancel(now_sec=self.clock.now_business_sec(), reason=str(payload.get('reason', 'cancelled')).strip() or 'cancelled')
        if terminal is not None:
            self._publish_terminal(terminal)

    def _control_cb(self, msg: String) -> None:
        control = decode_lifecycle_control(msg.data)
        if control.target and control.target != 'vendor_actuator_feedback_node':
            return
        changed = self.runtime.apply(control.command)
        if changed:
            self._publish_health('ok', 'runtime_state_changed', self._health_details())
            if not self.runtime.is_active:
                terminal = self.core.cancel(now_sec=self.clock.now_business_sec(), reason='runtime_not_active')
                if terminal is not None:
                    self._publish_terminal(terminal)

    def spin(self):
        rate = rospy.Rate(self.config['publish_rate_hz'])
        while not rospy.is_shutdown():
            now = self.clock.now_business_sec()
            terminal = self.core.evaluate(now_sec=now, result_timeout_sec=self.config['result_timeout_sec'])
            if terminal is not None:
                self._publish_terminal(terminal)
            details = self._health_details(); sig = ('ok', 'vendor_actuator_feedback_ready', JsonCodec.dumps(details))
            if sig != self._last_health_signature or (now - self._last_health_emit_sec) >= (1.0 / self.config['health_heartbeat_hz']):
                self._publish_health('ok', 'vendor_actuator_feedback_ready', details)
                self._last_health_signature = sig; self._last_health_emit_sec = now
            rate.sleep()

if __name__ == '__main__':
    try:
        VendorActuatorFeedbackNode().spin()
    except rospy.ROSInterruptException:
        pass

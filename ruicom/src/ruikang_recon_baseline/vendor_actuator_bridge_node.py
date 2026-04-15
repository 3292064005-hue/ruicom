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
from .vendor_actuator_bridge_core import VendorActuatorBridgeCore


def extract_command_id_from_payload_text(text: str) -> str:
    try:
        payload = JsonCodec.loads(text)
    except Exception:
        return ''
    return str((payload or {}).get('command_id', '')).strip()


class VendorActuatorBridgeNode:
    def __init__(self):
        rospy.init_node('vendor_actuator_bridge_node', anonymous=False)
        self.config = self._read_config(); self.clock = NodeClock(self.config['time_source_mode']); self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed']); self.core = VendorActuatorBridgeCore(); self._last_health_signature = (); self._last_health_emit_sec = 0.0; self._last_raw_result_sec = 0.0
        self.state_pub = rospy.Publisher(self.config['bridge_state_topic'], String, queue_size=20, latch=True)
        self.output_pub = rospy.Publisher(self.config['raw_command_output_topic'], String, queue_size=20)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.command_sub = rospy.Subscriber(self.config['input_command_topic'], String, self._command_cb, queue_size=20)
        self.result_sub = rospy.Subscriber(self.config['raw_result_topic'], String, self._raw_result_cb, queue_size=20)
        self.cancel_sub = rospy.Subscriber(self.config['cancel_topic'], String, self._cancel_cb, queue_size=20)
        self.control_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_cb, queue_size=20) if self.config['lifecycle_managed'] else None
        self._publish_health('ok', 'node_ready', self._health_details())

    def _read_config(self) -> dict:
        c = {
            'input_command_topic': str(rospy.get_param('~input_command_topic', 'recon/platform/vendor/actuator_command')).strip() or 'recon/platform/vendor/actuator_command',
            'raw_command_output_topic': str(rospy.get_param('~raw_command_output_topic', 'recon/platform/vendor/actuator_command_raw')).strip() or 'recon/platform/vendor/actuator_command_raw',
            'raw_result_topic': str(rospy.get_param('~raw_result_topic', 'recon/platform/vendor/actuator_result_raw')).strip() or 'recon/platform/vendor/actuator_result_raw',
            'bridge_state_topic': str(rospy.get_param('~bridge_state_topic', 'recon/platform/vendor/actuator_bridge_state')).strip() or 'recon/platform/vendor/actuator_bridge_state',
            'cancel_topic': str(rospy.get_param('~cancel_topic', 'recon/platform/behavior_execution/cancel')).strip() or 'recon/platform/behavior_execution/cancel',
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 15.0) or 15.0),
            'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz', 1.0) or 1.0),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower() or 'ros',
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'runtime_grade': str(rospy.get_param('~runtime_grade', 'integration')).strip().lower() or 'integration',
        }
        c['publish_rate_hz'] = require_positive_float('publish_rate_hz', c['publish_rate_hz']); c['health_heartbeat_hz'] = require_positive_float('health_heartbeat_hz', c['health_heartbeat_hz'])
        if c['time_source_mode'] not in ('ros', 'wall'): raise ConfigurationError('vendor_actuator_bridge_node time_source_mode must be ros or wall')
        return c

    @staticmethod
    def _connections(endpoint) -> int:
        getter = getattr(endpoint, 'get_num_connections', None)
        if getter is None: return 0
        try: return int(getter())
        except Exception: return 0

    def _health_details(self) -> Dict[str, object]:
        return {
            'runtime_grade': self.config['runtime_grade'],
            'input_command_topic_declared': bool(self.config['input_command_topic']),
            'raw_command_output_topic_declared': bool(self.config['raw_command_output_topic']),
            'raw_result_topic_declared': bool(self.config['raw_result_topic']),
            'command_input_bound': self._connections(self.command_sub) > 0,
            'raw_command_output_bound': self._connections(self.output_pub) > 0,
            'raw_result_bound': self._connections(self.result_sub) > 0,
            'has_active_command': bool(self.core.active is not None),
            **self.runtime.snapshot(),
        }

    def _publish_health(self, status: str, message: str, details: Optional[dict] = None) -> None:
        payload = {'stamp': self.clock.now_business_sec(), 'node': 'vendor_actuator_bridge_node', 'status': str(status).strip(), 'message': str(message).strip(), 'schema_version': SCHEMA_VERSION, 'details': dict(details or {})}
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState(); typed.header.stamp = self.clock.now_ros_time(); typed.header.frame_id = self.config['health_frame_id']; typed.node = payload['node']; typed.status = payload['status']; typed.message = payload['message']; typed.schema_version = payload['schema_version']; typed.details_json = JsonCodec.dumps(payload['details']); self.health_typed_pub.publish(typed)

    def _publish_state(self, payload: dict) -> None:
        self.state_pub.publish(String(data=JsonCodec.dumps(payload)))

    def _command_cb(self, msg: String) -> None:
        command_id = extract_command_id_from_payload_text(msg.data)
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._publish_health('warn', 'malformed_vendor_actuator_command', {**self._health_details(), 'error': str(exc)})
            return
        if self.runtime.lifecycle_managed and not self.runtime.is_active:
            self._publish_state({'command_id': command_id, 'state': 'FAILED', 'stamp': self.clock.now_business_sec(), 'details': {'reason': 'runtime_not_active'}, 'source': 'vendor_actuator_bridge'})
            return
        try:
            state = self.core.accept(payload, now_sec=self.clock.now_business_sec())
        except Exception as exc:
            self._publish_state({'command_id': command_id, 'state': 'FAILED', 'stamp': self.clock.now_business_sec(), 'details': {'reason': str(exc)}, 'source': 'vendor_actuator_bridge'})
            self._publish_health('warn', 'vendor_actuator_bridge_rejected', {**self._health_details(), 'error': str(exc), 'command_id': command_id})
            return
        self.output_pub.publish(String(data=JsonCodec.dumps(payload)))
        self._publish_state(state.to_dict())
        self._publish_health('ok', 'vendor_actuator_bridge_ready', self._health_details())

    def _raw_result_cb(self, msg: String) -> None:
        self._last_raw_result_sec = self.clock.now_business_sec()
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._publish_health('warn', 'malformed_vendor_actuator_result_raw', {**self._health_details(), 'error': str(exc)})
            return
        state = self.core.observe_raw_result(payload, now_sec=self.clock.now_business_sec())
        if state is not None:
            self._publish_state(state.to_dict())

    def _cancel_cb(self, msg: String) -> None:
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        command_id = str(payload.get('command_id', '')).strip()
        active = self.core.active
        if active is None or command_id != active.command_id:
            return
        state = self.core.cancel(now_sec=self.clock.now_business_sec(), reason=str(payload.get('reason', 'cancelled')).strip() or 'cancelled')
        if state is not None:
            self._publish_state(state.to_dict())

    def _control_cb(self, msg: String) -> None:
        control = decode_lifecycle_control(msg.data)
        if control.target and control.target != 'vendor_actuator_bridge_node':
            return
        changed = self.runtime.apply(control.command)
        if changed:
            self._publish_health('ok', 'runtime_state_changed', self._health_details())
            if not self.runtime.is_active:
                state = self.core.cancel(now_sec=self.clock.now_business_sec(), reason='runtime_not_active')
                if state is not None:
                    self._publish_state(state.to_dict())

    def spin(self):
        rate = rospy.Rate(self.config['publish_rate_hz'])
        while not rospy.is_shutdown():
            now = self.clock.now_business_sec()
            timeout_state = self.core.maybe_timeout(now_sec=now)
            if timeout_state is not None:
                self._publish_state(timeout_state.to_dict())
            heartbeat = self.core.heartbeat_state(now_sec=now)
            if heartbeat is not None:
                sig = JsonCodec.dumps(heartbeat.to_dict())
                if sig != self._last_health_signature:
                    self._publish_state(heartbeat.to_dict())
                    self._last_health_signature = sig
            details = self._health_details(); sig = ('ok', 'vendor_actuator_bridge_ready', JsonCodec.dumps(details))
            if sig != self._last_health_signature or (now - self._last_health_emit_sec) >= (1.0 / self.config['health_heartbeat_hz']):
                self._publish_health('ok', 'vendor_actuator_bridge_ready', details)
                self._last_health_signature = sig; self._last_health_emit_sec = now
            rate.sleep()

if __name__ == '__main__':
    try:
        VendorActuatorBridgeNode().spin()
    except rospy.ROSInterruptException:
        pass

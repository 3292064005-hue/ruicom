#!/usr/bin/env python3
from __future__ import annotations

from typing import Dict, Optional

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, require_positive_float
from .lifecycle_protocol import decode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .mowen_serial_protocol import (
    ChassisTelemetry,
    ChassisVelocityCommand,
    PROTOCOL_MODES,
    decode_managed_telemetry,
    iter_managed_frames,
    pack_legacy_init_frame,
    pack_legacy_velocity_frame,
    pack_managed_velocity_frame,
)
from .msg import HealthState
from .time_core import NodeClock

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover - imported lazily in real runtime only
    serial = None


class MowenSerialBridgeNode:
    """Upper-computer serial bridge between ROS ``cmd_vel`` and the chassis MCU.

    Function:
        Translate ROS velocity commands into the MO-SERGEANT serial protocol,
        keep the serial link alive, and convert chassis telemetry back into the
        normalized vendor feedback topics used by the platform contract.

    Inputs:
        ``~vendor_command_topic`` : ``geometry_msgs/Twist`` command ingress.
        ``~control_command_topic`` : optional lifecycle control channel.

    Outputs:
        ``~feedback_topic`` : ``std_msgs/Bool`` authoritative execution feedback.
        ``~telemetry_topic`` : JSON telemetry/bridge-state payload.
        ``~health_topic`` / ``~health_typed_topic`` : runtime diagnostics.

    Exceptions:
        ``ConfigurationError`` is raised on invalid parameters.
        Runtime serial I/O failures are surfaced through health and the node
        automatically retries on the next loop.

    Boundary behavior:
        - ``legacy_newt`` remains wire-compatible with the vendor ``newt.py``
          packet format.
        - ``managed_crc16`` supports structured telemetry and is the preferred
          real-field mode once the MCU firmware is upgraded.
        - when command traffic goes stale, the node sends one explicit zero
          command so the chassis does not continue moving on old velocity.
    """

    def __init__(self):
        rospy.init_node('mowen_serial_bridge_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self._last_health_emit_sec = 0.0
        self._last_health_signature = ''
        self._last_command_sec = 0.0
        self._last_command_msg: Optional[Twist] = None
        self._last_telemetry_sec = 0.0
        self._last_serial_write_sec = 0.0
        self._last_feedback_value = False
        self._zero_command_sent = False
        self._read_buffer = bytearray()
        self._sequence = 0
        self._serial = None
        self.feedback_pub = rospy.Publisher(self.config['feedback_topic'], Bool, queue_size=10)
        self.telemetry_pub = rospy.Publisher(self.config['telemetry_topic'], String, queue_size=10)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.command_sub = rospy.Subscriber(self.config['vendor_command_topic'], Twist, self._command_cb, queue_size=20)
        self.control_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_cb, queue_size=20) if self.config['lifecycle_managed'] else None
        self._publish_health('ok', 'mowen_serial_bridge_ready', self._health_details())

    def _read_config(self) -> dict:
        """Read and validate bridge configuration."""
        config = {
            'vendor_command_topic': str(rospy.get_param('~vendor_command_topic', 'recon/platform/vendor/cmd_vel')).strip() or 'recon/platform/vendor/cmd_vel',
            'feedback_topic': str(rospy.get_param('~feedback_topic', 'recon/platform/vendor/base_feedback_raw')).strip() or 'recon/platform/vendor/base_feedback_raw',
            'telemetry_topic': str(rospy.get_param('~telemetry_topic', 'recon/platform/vendor/chassis_telemetry')).strip() or 'recon/platform/vendor/chassis_telemetry',
            'serial_port': str(rospy.get_param('~serial_port', '/dev/carserial')).strip() or '/dev/carserial',
            'baudrate': int(rospy.get_param('~baudrate', 115200) or 115200),
            'protocol_mode': str(rospy.get_param('~protocol_mode', 'legacy_newt')).strip() or 'legacy_newt',
            'write_rate_hz': float(rospy.get_param('~write_rate_hz', 20.0) or 20.0),
            'feedback_timeout_sec': float(rospy.get_param('~feedback_timeout_sec', 0.6) or 0.6),
            'telemetry_timeout_sec': float(rospy.get_param('~telemetry_timeout_sec', 1.2) or 1.2),
            'zero_command_timeout_sec': float(rospy.get_param('~zero_command_timeout_sec', 0.5) or 0.5),
            'legacy_feedback_policy': str(rospy.get_param('~legacy_feedback_policy', 'telemetry_required')).strip().lower() or 'telemetry_required',
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz', 1.0) or 1.0),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower() or 'ros',
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
        }
        if config['protocol_mode'] not in PROTOCOL_MODES:
            raise ConfigurationError(f'mowen_serial_bridge protocol_mode must be one of {PROTOCOL_MODES}')
        if config['legacy_feedback_policy'] not in ('telemetry_required', 'command_write'):
            raise ConfigurationError('mowen_serial_bridge legacy_feedback_policy must be telemetry_required or command_write')
        config['write_rate_hz'] = require_positive_float('mowen_serial_bridge.write_rate_hz', config['write_rate_hz'])
        config['feedback_timeout_sec'] = require_positive_float('mowen_serial_bridge.feedback_timeout_sec', config['feedback_timeout_sec'])
        config['telemetry_timeout_sec'] = require_positive_float('mowen_serial_bridge.telemetry_timeout_sec', config['telemetry_timeout_sec'])
        config['zero_command_timeout_sec'] = require_positive_float('mowen_serial_bridge.zero_command_timeout_sec', config['zero_command_timeout_sec'])
        config['health_heartbeat_hz'] = require_positive_float('mowen_serial_bridge.health_heartbeat_hz', config['health_heartbeat_hz'])
        if config['time_source_mode'] not in ('ros', 'wall'):
            raise ConfigurationError('mowen_serial_bridge time_source_mode must be ros or wall')
        return config

    def _ensure_serial(self):
        if self._serial is not None:
            return self._serial
        if serial is None:
            raise RuntimeError('pyserial is not available')
        self._serial = serial.Serial(self.config['serial_port'], self.config['baudrate'], timeout=0.02)
        if self.config['protocol_mode'] == 'legacy_newt':
            self._serial.write(pack_legacy_init_frame())
        self._last_serial_write_sec = self.clock.now_business_sec()
        return self._serial

    def _close_serial(self):
        ser = self._serial
        self._serial = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _command_cb(self, msg: Twist) -> None:
        self._last_command_msg = msg
        self._last_command_sec = self.clock.now_business_sec()
        self._sequence = (self._sequence + 1) & 0xFFFF
        self._zero_command_sent = False

    def _control_cb(self, msg: String) -> None:
        control = decode_lifecycle_control(msg.data)
        if control.target and control.target != 'mowen_serial_bridge_node':
            return
        changed = self.runtime.apply(control.command)
        if changed:
            self._publish_health('ok', 'runtime_state_changed', self._health_details())

    def _connections(self, endpoint) -> int:
        getter = getattr(endpoint, 'get_num_connections', None)
        if getter is None:
            return 0
        try:
            return int(getter())
        except Exception:
            return 0

    def _fresh(self, stamp_sec: float, timeout_sec: float) -> bool:
        now = self.clock.now_business_sec()
        return stamp_sec > 0.0 and (now - stamp_sec) <= timeout_sec

    def _effective_feedback(self) -> tuple[bool, str]:
        telemetry_fresh = self._fresh(self._last_telemetry_sec, self.config['telemetry_timeout_sec'])
        write_fresh = self._fresh(self._last_serial_write_sec, self.config['feedback_timeout_sec'])
        if self.config['protocol_mode'] == 'managed_crc16':
            return telemetry_fresh, 'managed_crc16.telemetry'
        if self.config['legacy_feedback_policy'] == 'command_write':
            return write_fresh, 'legacy_newt.command_write'
        return telemetry_fresh, 'legacy_newt.telemetry_required'

    def _health_details(self) -> Dict[str, object]:
        telemetry_fresh = self._fresh(self._last_telemetry_sec, self.config['telemetry_timeout_sec'])
        command_fresh = self._fresh(self._last_command_sec, self.config['feedback_timeout_sec'])
        write_fresh = self._fresh(self._last_serial_write_sec, self.config['feedback_timeout_sec'])
        feedback_value, feedback_source = self._effective_feedback()
        return {
            'protocol_mode': self.config['protocol_mode'],
            'legacy_feedback_policy': self.config['legacy_feedback_policy'],
            'serial_port': self.config['serial_port'],
            'vendor_command_topic_declared': bool(self.config['vendor_command_topic']),
            'feedback_topic_declared': bool(self.config['feedback_topic']),
            'telemetry_topic_declared': bool(self.config['telemetry_topic']),
            'command_input_bound': self._connections(self.command_sub) > 0,
            'feedback_output_bound': self._connections(self.feedback_pub) > 0,
            'telemetry_output_bound': self._connections(self.telemetry_pub) > 0,
            'serial_open': self._serial is not None,
            'telemetry_fresh': telemetry_fresh,
            'command_fresh': command_fresh,
            'write_fresh': write_fresh,
            'execution_feedback_source': feedback_source,
            'execution_feedback_fresh': bool(feedback_value),
            **self.runtime.snapshot(),
        }

    def _publish_health(self, status: str, message: str, details: Optional[dict] = None) -> None:
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'mowen_serial_bridge_node',
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

    def _publish_telemetry_payload(self, telemetry: Optional[ChassisTelemetry] = None) -> None:
        output_feedback, feedback_source = self._effective_feedback()
        payload = {
            'schema_version': SCHEMA_VERSION,
            'source': 'mowen_serial_bridge',
            'protocol_mode': self.config['protocol_mode'],
            'legacy_feedback_policy': self.config['legacy_feedback_policy'],
            'feedback_source': feedback_source,
            'command_fresh': self._fresh(self._last_command_sec, self.config['feedback_timeout_sec']),
            'telemetry_fresh': self._fresh(self._last_telemetry_sec, self.config['telemetry_timeout_sec']),
            'write_fresh': self._fresh(self._last_serial_write_sec, self.config['feedback_timeout_sec']),
            'execution_feedback': bool(output_feedback),
            'sequence': None,
            'linear_x_mps': None,
            'linear_y_mps': None,
            'angular_z_rps': None,
            'battery_voltage_v': None,
            'estop': None,
            'heartbeat_ok': None,
        }
        if telemetry is not None:
            payload.update({
                'sequence': telemetry.sequence,
                'linear_x_mps': telemetry.linear_x_mps,
                'linear_y_mps': telemetry.linear_y_mps,
                'angular_z_rps': telemetry.angular_z_rps,
                'battery_voltage_v': telemetry.battery_voltage_v,
                'estop': telemetry.estop,
                'heartbeat_ok': telemetry.heartbeat_ok,
            })
        self.telemetry_pub.publish(String(data=JsonCodec.dumps(payload)))

    def _emit_feedback(self, telemetry: Optional[ChassisTelemetry] = None) -> None:
        if telemetry is not None:
            self._last_telemetry_sec = self.clock.now_business_sec()
        feedback, _ = self._effective_feedback()
        self._last_feedback_value = bool(feedback)
        self.feedback_pub.publish(Bool(data=self._last_feedback_value))
        self._publish_telemetry_payload(telemetry)

    def _build_command(self, twist: Twist) -> ChassisVelocityCommand:
        return ChassisVelocityCommand(
            linear_x_mps=float(twist.linear.x),
            linear_y_mps=float(twist.linear.y),
            angular_z_rps=float(twist.angular.z),
            sequence=self._sequence,
        )

    def _write_frame(self, command: ChassisVelocityCommand) -> None:
        ser = self._ensure_serial()
        frame = pack_legacy_velocity_frame(command) if self.config['protocol_mode'] == 'legacy_newt' else pack_managed_velocity_frame(command)
        ser.write(frame)
        self._last_serial_write_sec = self.clock.now_business_sec()

    def _write_command(self):
        """Drive the serial bus from the most recent command and enforce stop-on-stale.

        Returns:
            None.

        Raises:
            RuntimeError: Propagated from serial creation/write failures.

        Boundary behavior:
            When no recent command exists, exactly one zero-velocity frame is sent
            after ``zero_command_timeout_sec`` so the chassis stops cleanly.
        """
        if self.runtime.lifecycle_managed and not self.runtime.is_active:
            return
        now = self.clock.now_business_sec()
        command_age = now - self._last_command_sec if self._last_command_sec > 0.0 else None
        if self._last_command_msg is not None and command_age is not None and command_age <= self.config['zero_command_timeout_sec']:
            self._write_frame(self._build_command(self._last_command_msg))
            self._zero_command_sent = False
            return
        if self._last_command_sec > 0.0 and command_age is not None and command_age > self.config['zero_command_timeout_sec'] and not self._zero_command_sent:
            self._write_frame(ChassisVelocityCommand(sequence=self._sequence))
            self._zero_command_sent = True

    def _read_telemetry(self):
        ser = self._ensure_serial()
        waiting = getattr(ser, 'in_waiting', 0)
        if waiting:
            self._read_buffer.extend(ser.read(waiting))
        if not self._read_buffer:
            return
        remaining = bytearray(self._read_buffer)
        for frame in iter_managed_frames(bytes(self._read_buffer)):
            try:
                telemetry = decode_managed_telemetry(frame)
            except Exception:
                continue
            self._emit_feedback(telemetry)
            frame_bytes = bytes(frame)
            index = remaining.find(frame_bytes)
            if index >= 0:
                del remaining[:index + len(frame_bytes)]
        self._read_buffer = remaining

    def spin(self):
        rate = rospy.Rate(self.config['write_rate_hz'])
        while not rospy.is_shutdown():
            try:
                self._write_command()
                self._read_telemetry()
                self._emit_feedback(None)
                details = self._health_details()
                signature = JsonCodec.dumps(details)
                now = self.clock.now_business_sec()
                if signature != self._last_health_signature or (now - self._last_health_emit_sec) >= (1.0 / self.config['health_heartbeat_hz']):
                    self._publish_health('ok', 'mowen_serial_bridge_ready', details)
                    self._last_health_signature = signature
                    self._last_health_emit_sec = now
            except Exception as exc:
                self._publish_health('warn', 'mowen_serial_bridge_runtime_error', {**self._health_details(), 'error': str(exc)})
                self._close_serial()
            rate.sleep()


if __name__ == '__main__':
    try:
        MowenSerialBridgeNode().spin()
    except rospy.ROSInterruptException:
        pass

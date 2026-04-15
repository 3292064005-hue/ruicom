#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""MO-SERGEANT managed vendor workspace contract chassis bridge.

This historical ``rosrun car_bringup newt.py`` entrypoint now implements the
actual serial bridge expected by the vendor workflow instead of a placeholder.
It keeps ROS Melodic/Python 2 compatibility, publishes authoritative feedback
and telemetry topics, and actively reconnects when the chassis serial port is
replugged or temporarily unavailable.
"""
from __future__ import print_function

import json
import os
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

try:
    import serial
except Exception as exc:  # pragma: no cover - environment dependent
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

LEGACY_HEADER = b'\xAA\xBB\x0A\x12\x02'
LEGACY_INIT = b'\x11\x00\x00\x00\x00\x00\x00\x00\x00'
MANAGED_SOF = b'\x55\xAA'


def _as_bytes(byte_list):
    if sys.version_info[0] >= 3:
        return bytes(byte_list)
    return ''.join(chr(int(item) & 0xFF) for item in byte_list)


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, float(value)))


def _i16_le(value):
    value = int(value)
    if value < 0:
        value = (1 << 16) + value
    return _as_bytes([value & 0xFF, (value >> 8) & 0xFF])


def _crc16_ibm(data):
    crc = 0xFFFF
    for item in bytearray(data):
        crc ^= int(item)
        for _ in range(8):
            if crc & 0x0001:
                crc = ((crc >> 1) ^ 0xA001) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc & 0xFFFF


def _parse_i16_le(lo, hi):
    value = int(lo) | (int(hi) << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def pack_legacy_velocity(cmd_msg):
    vx = int(round(_clamp(cmd_msg.linear.x, -1.5, 1.5) * 1000.0))
    vy = int(round(_clamp(cmd_msg.linear.y, -1.5, 1.5) * 1000.0))
    wz = int(round(_clamp(cmd_msg.angular.z, -4.0, 4.0) * 1000.0))
    return LEGACY_HEADER + _i16_le(vx) + _i16_le(vy) + _i16_le(wz) + b'\x00'


def parse_managed_telemetry(frame_bytes):
    if len(frame_bytes) != 19:
        return None
    raw = bytearray(frame_bytes)
    if raw[0] != 0x55 or raw[1] != 0xAA:
        return None
    if raw[2] != 0x81 or raw[3] != 11:
        return None
    if raw[-2] != 0x0A or raw[-1] != 0x0D:
        return None
    crc_expected = _crc16_ibm(raw[2:15])
    crc_received = int(raw[15]) | (int(raw[16]) << 8)
    if crc_expected != crc_received:
        return None
    payload = raw[4:15]
    sequence = int(payload[0]) | (int(payload[1]) << 8)
    flags = int(payload[10])
    return {
        'sequence': sequence,
        'linear_x_mps': _parse_i16_le(payload[2], payload[3]) / 1000.0,
        'linear_y_mps': _parse_i16_le(payload[4], payload[5]) / 1000.0,
        'angular_z_rps': _parse_i16_le(payload[6], payload[7]) / 1000.0,
        'battery_voltage_v': (int(payload[8]) | (int(payload[9]) << 8)) / 1000.0,
        'estop': bool(flags & 0x01),
        'heartbeat_ok': bool(flags & 0x02),
    }


class NewtBridge(object):
    """Bridge ``/cmd_vel`` into the MO-SERGEANT chassis serial interface.

    Parameters:
        ``~serial_port``: serial device path, default ``/dev/carserial``.
        ``~baudrate``: serial baudrate, default ``115200``.
        ``~cmd_vel_topic``: ROS ingress topic, default ``/cmd_vel``.
        ``~feedback_topic``: execution feedback topic.
        ``~telemetry_topic``: JSON diagnostics topic.
        ``~legacy_feedback_policy``: ``telemetry_required`` or ``command_write``.

    Boundary behavior:
        - one zero-velocity frame is emitted when commands go stale;
        - serial disconnects are converted into reconnect attempts;
        - managed telemetry, when present, upgrades feedback from write-path
          liveness to telemetry-backed execution freshness.
    """

    def __init__(self):
        rospy.init_node('newt_bridge', anonymous=False)
        self.serial_port = rospy.get_param('~serial_port', os.environ.get('MOWEN_CHASSIS_SERIAL_PORT', '/dev/carserial'))
        self.baudrate = int(rospy.get_param('~baudrate', int(os.environ.get('MOWEN_CHASSIS_BAUDRATE', '115200'))))
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.feedback_topic = rospy.get_param('~feedback_topic', 'recon/platform/vendor/base_feedback_raw')
        self.telemetry_topic = rospy.get_param('~telemetry_topic', 'recon/platform/vendor/chassis_telemetry')
        self.publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 20.0))
        self.command_timeout_sec = float(rospy.get_param('~command_timeout_sec', 0.5))
        self.feedback_timeout_sec = float(rospy.get_param('~feedback_timeout_sec', 0.6))
        self.telemetry_timeout_sec = float(rospy.get_param('~telemetry_timeout_sec', 1.2))
        self.reconnect_interval_sec = float(rospy.get_param('~reconnect_interval_sec', 1.0))
        self.legacy_feedback_policy = str(rospy.get_param('~legacy_feedback_policy', 'telemetry_required')).strip().lower() or 'telemetry_required'
        if self.legacy_feedback_policy not in ('telemetry_required', 'command_write'):
            raise ValueError('legacy_feedback_policy must be telemetry_required or command_write')
        if serial is None:
            raise RuntimeError('pyserial import failed: %s' % SERIAL_IMPORT_ERROR)
        self.feedback_pub = rospy.Publisher(self.feedback_topic, Bool, queue_size=10)
        self.telemetry_pub = rospy.Publisher(self.telemetry_topic, String, queue_size=10)
        self.serial_handle = None
        self.last_connect_attempt_sec = 0.0
        self.last_command_msg = Twist()
        self.last_command_stamp = 0.0
        self.last_write_stamp = 0.0
        self.last_telemetry_stamp = 0.0
        self.read_buffer = bytearray()
        self.zero_command_sent = False
        rospy.Subscriber(self.cmd_vel_topic, Twist, self._command_callback, queue_size=20)

    def _command_callback(self, msg):
        self.last_command_msg = msg
        self.last_command_stamp = rospy.get_time()
        self.zero_command_sent = False

    def _open_serial(self):
        now = rospy.get_time()
        if self.serial_handle is not None:
            return self.serial_handle
        if self.last_connect_attempt_sec > 0.0 and (now - self.last_connect_attempt_sec) < self.reconnect_interval_sec:
            return None
        self.last_connect_attempt_sec = now
        try:
            handle = serial.Serial(self.serial_port, self.baudrate, timeout=0.02)
            handle.write(LEGACY_INIT)
            self.serial_handle = handle
            return self.serial_handle
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'newt bridge failed to open %s: %s' % (self.serial_port, exc))
            self.serial_handle = None
            return None

    def _close_serial(self):
        handle = self.serial_handle
        self.serial_handle = None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def _write_command(self, msg):
        handle = self._open_serial()
        if handle is None:
            return False
        try:
            handle.write(pack_legacy_velocity(msg))
            self.last_write_stamp = rospy.get_time()
            return True
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'newt bridge serial write failed: %s' % exc)
            self._close_serial()
            return False

    def _write_zero_once(self):
        if self.zero_command_sent:
            return
        if self._write_command(Twist()):
            self.zero_command_sent = True

    def _command_fresh(self):
        return self.last_command_stamp > 0.0 and (rospy.get_time() - self.last_command_stamp) <= self.command_timeout_sec

    def _telemetry_fresh(self):
        return self.last_telemetry_stamp > 0.0 and (rospy.get_time() - self.last_telemetry_stamp) <= self.telemetry_timeout_sec

    def _write_fresh(self):
        return self.last_write_stamp > 0.0 and (rospy.get_time() - self.last_write_stamp) <= self.feedback_timeout_sec

    def _effective_feedback(self):
        if self._telemetry_fresh():
            return True, 'managed_telemetry'
        if self.legacy_feedback_policy == 'command_write':
            return self._write_fresh(), 'legacy_command_write'
        return False, 'telemetry_required'

    def _drain_telemetry(self):
        handle = self._open_serial()
        if handle is None:
            return None
        try:
            waiting = getattr(handle, 'in_waiting', 0)
            if waiting:
                self.read_buffer.extend(handle.read(waiting))
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'newt bridge serial read failed: %s' % exc)
            self._close_serial()
            return None
        payload = None
        while True:
            start = self.read_buffer.find(bytearray(MANAGED_SOF))
            if start < 0:
                if len(self.read_buffer) > 256:
                    self.read_buffer = self.read_buffer[-64:]
                break
            if len(self.read_buffer) < start + 19:
                if start > 0:
                    self.read_buffer = self.read_buffer[start:]
                break
            frame = bytes(self.read_buffer[start:start + 19])
            del self.read_buffer[:start + 19]
            decoded = parse_managed_telemetry(frame)
            if decoded is None:
                continue
            payload = decoded
            self.last_telemetry_stamp = rospy.get_time()
        return payload

    def _publish_state(self, telemetry_payload):
        feedback, feedback_source = self._effective_feedback()
        payload = {
            'source': 'vendor_workspace.newt_bridge',
            'contract': 'managed vendor workspace',
            'serial_port': self.serial_port,
            'cmd_vel_topic': self.cmd_vel_topic,
            'feedback_policy': self.legacy_feedback_policy,
            'command_fresh': self._command_fresh(),
            'write_fresh': self._write_fresh(),
            'telemetry_fresh': self._telemetry_fresh(),
            'execution_feedback': bool(feedback),
            'feedback_source': feedback_source,
            'serial_open': self.serial_handle is not None,
        }
        if telemetry_payload is not None:
            payload.update(telemetry_payload)
        self.feedback_pub.publish(Bool(data=bool(feedback)))
        self.telemetry_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            telemetry_payload = None
            if self._command_fresh():
                self._write_command(self.last_command_msg)
                self.zero_command_sent = False
            elif self.last_command_stamp > 0.0:
                self._write_zero_once()
            telemetry_payload = self._drain_telemetry()
            self._publish_state(telemetry_payload)
            rate.sleep()


def main():
    bridge = NewtBridge()
    bridge.spin()


if __name__ == '__main__':
    main()

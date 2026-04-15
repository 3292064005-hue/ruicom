#!/usr/bin/env python3
"""Bridge chassis telemetry into authoritative odom/TF for MO-SERGEANT.

Function:
    Convert the vendor chassis telemetry stream into ``nav_msgs/Odometry`` and
    the ``odom -> base_frame`` transform required by AMCL and move_base.

Inputs:
    - ``~telemetry_topic`` (std_msgs/String JSON payload emitted by the chassis
      bridge or vendor ``newt.py``)
    - ``~cmd_vel_topic`` (geometry_msgs/Twist) as an explicit degraded fallback
      only when ``~allow_command_fallback`` is enabled
    - ``~imu_topic`` (optional sensor_msgs/Imu) for yaw override

Outputs:
    - ``~odom_topic`` (nav_msgs/Odometry)
    - TF ``odom_frame -> base_frame``
    - ``~health_topic`` / ``~health_typed_topic`` diagnostics

Exceptions:
    No explicit exception escapes the main loop. Malformed telemetry is counted
    and reported through health rather than terminating navigation bringup.

Boundary behavior:
    - Telemetry velocities are preferred when fresh.
    - Command fallback is opt-in and always marked as degraded in diagnostics.
    - When neither source is fresh, the node publishes zero twist while keeping
      the last integrated pose so downstream navigation does not diverge.
"""
from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from ruikang_recon_baseline.msg import HealthState
from ruikang_recon_baseline.common import JsonCodec, SCHEMA_VERSION


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    half = 0.5 * float(yaw)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def _quaternion_to_yaw(msg: Quaternion) -> float:
    siny_cosp = 2.0 * (float(msg.w) * float(msg.z) + float(msg.x) * float(msg.y))
    cosy_cosp = 1.0 - 2.0 * (float(msg.y) * float(msg.y) + float(msg.z) * float(msg.z))
    return math.atan2(siny_cosp, cosy_cosp)


class ChassisOdometryNode:
    def __init__(self) -> None:
        rospy.init_node('chassis_odometry_node', anonymous=False)
        self.config = self._read_config()
        self.odom_pub = rospy.Publisher(self.config['odom_topic'], Odometry, queue_size=20)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10, latch=True)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10, latch=True)
        self.tf_broadcaster = TransformBroadcaster()
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._yaw = 0.0
        self._last_step_sec = 0.0
        self._last_telemetry_sec = 0.0
        self._last_command_sec = 0.0
        self._last_imu_sec = 0.0
        self._last_health_emit_sec = 0.0
        self._telemetry_velocity = (0.0, 0.0, 0.0)
        self._command_velocity = (0.0, 0.0, 0.0)
        self._imu_yaw: Optional[float] = None
        self._malformed_telemetry_count = 0
        self._last_source = 'idle'
        rospy.Subscriber(self.config['telemetry_topic'], String, self._telemetry_cb, queue_size=50)
        rospy.Subscriber(self.config['cmd_vel_topic'], Twist, self._command_cb, queue_size=50)
        if self.config['imu_topic']:
            rospy.Subscriber(self.config['imu_topic'], Imu, self._imu_cb, queue_size=50)

    def _read_config(self) -> Dict[str, object]:
        config: Dict[str, object] = {
            'telemetry_topic': str(rospy.get_param('~telemetry_topic', 'recon/platform/vendor/chassis_telemetry')).strip() or 'recon/platform/vendor/chassis_telemetry',
            'cmd_vel_topic': str(rospy.get_param('~cmd_vel_topic', '/cmd_vel')).strip() or '/cmd_vel',
            'odom_topic': str(rospy.get_param('~odom_topic', 'odom')).strip() or 'odom',
            'base_frame': str(rospy.get_param('~base_frame', 'base_footprint')).strip() or 'base_footprint',
            'odom_frame': str(rospy.get_param('~odom_frame', 'odom')).strip() or 'odom',
            'imu_topic': str(rospy.get_param('~imu_topic', '')).strip(),
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 30.0) or 30.0),
            'telemetry_timeout_sec': float(rospy.get_param('~telemetry_timeout_sec', 0.7) or 0.7),
            'command_timeout_sec': float(rospy.get_param('~command_timeout_sec', 0.5) or 0.5),
            'imu_timeout_sec': float(rospy.get_param('~imu_timeout_sec', 1.0) or 1.0),
            'allow_command_fallback': bool(rospy.get_param('~allow_command_fallback', False)),
            'use_imu_yaw': bool(rospy.get_param('~use_imu_yaw', True)),
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
        }
        if config['publish_rate_hz'] <= 0.0:
            raise ValueError('publish_rate_hz must be > 0')
        return config

    def _telemetry_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or '{}')
        except Exception:
            self._malformed_telemetry_count += 1
            return
        try:
            vx = float(payload.get('linear_x_mps', 0.0) or 0.0)
            vy = float(payload.get('linear_y_mps', 0.0) or 0.0)
            wz = float(payload.get('angular_z_rps', 0.0) or 0.0)
        except Exception:
            self._malformed_telemetry_count += 1
            return
        self._telemetry_velocity = (vx, vy, wz)
        self._last_telemetry_sec = rospy.get_time()

    def _command_cb(self, msg: Twist) -> None:
        self._command_velocity = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        self._last_command_sec = rospy.get_time()

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_yaw = _quaternion_to_yaw(msg.orientation)
        self._last_imu_sec = rospy.get_time()

    def _fresh(self, stamp: float, timeout_sec: float) -> bool:
        now = rospy.get_time()
        return stamp > 0.0 and (now - stamp) <= timeout_sec

    def _select_velocity(self) -> Tuple[Tuple[float, float, float], str]:
        if self._fresh(self._last_telemetry_sec, float(self.config['telemetry_timeout_sec'])):
            return self._telemetry_velocity, 'telemetry'
        if bool(self.config['allow_command_fallback']) and self._fresh(self._last_command_sec, float(self.config['command_timeout_sec'])):
            return self._command_velocity, 'command_fallback'
        return (0.0, 0.0, 0.0), 'idle'

    def _publish_health(self, source: str, twist: Tuple[float, float, float]) -> None:
        now = rospy.get_time()
        if (now - self._last_health_emit_sec) < 1.0:
            return
        details = {
            'telemetry_topic': self.config['telemetry_topic'],
            'cmd_vel_topic': self.config['cmd_vel_topic'],
            'odom_topic': self.config['odom_topic'],
            'base_frame': self.config['base_frame'],
            'odom_frame': self.config['odom_frame'],
            'telemetry_fresh': self._fresh(self._last_telemetry_sec, float(self.config['telemetry_timeout_sec'])),
            'command_fresh': self._fresh(self._last_command_sec, float(self.config['command_timeout_sec'])),
            'imu_fresh': self._fresh(self._last_imu_sec, float(self.config['imu_timeout_sec'])),
            'source': source,
            'twist': {'vx': twist[0], 'vy': twist[1], 'wz': twist[2]},
            'malformed_telemetry_count': int(self._malformed_telemetry_count),
            'allow_command_fallback': bool(self.config['allow_command_fallback']),
        }
        payload = {
            'stamp': now,
            'node': 'chassis_odometry_node',
            'status': 'ok' if source != 'idle' else 'warn',
            'message': 'chassis_odometry_active' if source != 'idle' else 'chassis_odometry_idle',
            'schema_version': SCHEMA_VERSION,
            'details': details,
        }
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = rospy.Time.now()
        typed.header.frame_id = str(self.config['health_frame_id'])
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(details)
        self.health_typed_pub.publish(typed)
        self._last_health_emit_sec = now

    def _publish_odom(self, twist: Tuple[float, float, float], stamp: rospy.Time) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = str(self.config['odom_frame'])
        odom.child_frame_id = str(self.config['base_frame'])
        odom.pose.pose.position.x = self._pose_x
        odom.pose.pose.position.y = self._pose_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = _yaw_to_quaternion(self._yaw)
        odom.twist.twist.linear.x = twist[0]
        odom.twist.twist.linear.y = twist[1]
        odom.twist.twist.angular.z = twist[2]
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.04
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.08
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = str(self.config['odom_frame'])
        transform.child_frame_id = str(self.config['base_frame'])
        transform.transform.translation.x = self._pose_x
        transform.transform.translation.y = self._pose_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def spin(self) -> None:
        rate = rospy.Rate(float(self.config['publish_rate_hz']))
        while not rospy.is_shutdown():
            now_sec = rospy.get_time()
            if self._last_step_sec <= 0.0:
                self._last_step_sec = now_sec
            dt = max(0.0, min(0.2, now_sec - self._last_step_sec))
            twist, source = self._select_velocity()
            self._last_source = source
            vx, vy, wz = twist
            if dt > 0.0:
                if bool(self.config['use_imu_yaw']) and self._imu_yaw is not None and self._fresh(self._last_imu_sec, float(self.config['imu_timeout_sec'])):
                    self._yaw = float(self._imu_yaw)
                else:
                    self._yaw += wz * dt
                cos_yaw = math.cos(self._yaw)
                sin_yaw = math.sin(self._yaw)
                self._pose_x += (vx * cos_yaw - vy * sin_yaw) * dt
                self._pose_y += (vx * sin_yaw + vy * cos_yaw) * dt
            self._last_step_sec = now_sec
            stamp = rospy.Time.now()
            self._publish_odom(twist, stamp)
            self._publish_health(source, twist)
            rate.sleep()


if __name__ == '__main__':
    try:
        ChassisOdometryNode().spin()
    except rospy.ROSInterruptException:
        pass

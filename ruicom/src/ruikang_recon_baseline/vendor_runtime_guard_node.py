#!/usr/bin/env python3
"""Launch-time guard for separating contract/reference/field vendor runtime paths."""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import rospy
from std_msgs.msg import String

from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION
from .msg import HealthState


class VendorRuntimeGuardNode:
    """Validate the selected vendor runtime grade before deploy starts.

    Function:
        Prevent field-grade runs from silently falling back to repository-managed
        fixtures or half-declared vendor launch paths.

    Inputs:
        Launch parameters only.

    Outputs:
        ``recon/health`` and ``recon/health_typed`` diagnostics.

    Boundary behavior:
        - ``contract`` and ``reference`` grades may use repository-managed
          fixtures for smoke and controlled reference validation.
        - ``field`` grade must either declare concrete vendor launch files for
          each enabled hardware seam or explicitly disable that seam because the
          external graph is already running.
    """

    def __init__(self) -> None:
        rospy.init_node('vendor_runtime_guard_node')
        self.config = self._read_config()
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10, latch=True)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10, latch=True)
        self._validate()
        self._publish_health('ok', 'vendor_runtime_guard_ready', self._details())

    def _read_config(self) -> Dict[str, object]:
        grade = str(rospy.get_param('~deploy_grade', 'field')).strip().lower() or 'field'
        if grade not in ('contract', 'reference', 'field'):
            raise ConfigurationError('deploy_grade must be contract, reference, or field')
        config: Dict[str, object] = {
            'deploy_grade': grade,
            'vendor_bringup_launch': str(rospy.get_param('~vendor_bringup_launch', '')).strip(),
            'vendor_navigation_launch': str(rospy.get_param('~vendor_navigation_launch', '')).strip(),
            'vendor_camera_launch': str(rospy.get_param('~vendor_camera_launch', '')).strip(),
            'vendor_lidar_launch': str(rospy.get_param('~vendor_lidar_launch', '')).strip(),
            'vendor_imu_launch': str(rospy.get_param('~vendor_imu_launch', '')).strip(),
            'vendor_camera_driver_launch': str(rospy.get_param('~vendor_camera_driver_launch', '')).strip(),
            'vendor_lidar_driver_launch': str(rospy.get_param('~vendor_lidar_driver_launch', '')).strip(),
            'vendor_imu_driver_launch': str(rospy.get_param('~vendor_imu_driver_launch', '')).strip(),
            'enable_vendor_bringup': bool(rospy.get_param('~enable_vendor_bringup', True)),
            'enable_vendor_navigation': bool(rospy.get_param('~enable_vendor_navigation', True)),
            'enable_vendor_camera': bool(rospy.get_param('~enable_vendor_camera', True)),
            'enable_vendor_lidar': bool(rospy.get_param('~enable_vendor_lidar', True)),
            'enable_vendor_imu': bool(rospy.get_param('~enable_vendor_imu', True)),
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
        }
        return config

    def _is_repo_fixture(self, launch_path: str) -> bool:
        normalized = launch_path.replace('\\', '/').strip()
        if not normalized:
            return False
        repo_markers = (
            'managed_vendor_',
            '/vendor_workspace/newznzc_ws/',
            'find ruikang_recon_baseline',
        )
        return any(marker in normalized for marker in repo_markers)

    def _validate_enabled_launch(self, enabled_key: str, launch_key: str) -> None:
        enabled = bool(self.config[enabled_key])
        launch_path = str(self.config[launch_key])
        if not enabled:
            return
        if self.config['deploy_grade'] != 'field':
            return
        if not launch_path:
            raise ConfigurationError(
                '{} must be set when {}=true under field deploy grade; disable the seam only when an external vendor graph already provides it'.format(
                    launch_key, enabled_key
                )
            )
        if self._is_repo_fixture(launch_path):
            raise ConfigurationError(
                '{} points to a repository-managed fixture and is forbidden under field deploy grade: {}'.format(
                    launch_key, launch_path
                )
            )

    def _validate_wrapper_driver(self, enabled_key: str, launch_key: str, driver_key: str, marker: str) -> None:
        if self.config['deploy_grade'] != 'field' or not bool(self.config[enabled_key]):
            return
        launch_path = str(self.config[launch_key])
        if marker not in launch_path.replace('\\', '/'):
            return
        if not str(self.config[driver_key]).strip():
            raise ConfigurationError(
                '{} requires {} under field deploy grade so the wrapper cannot become a no-op'.format(
                    launch_key, driver_key
                )
            )

    def _validate(self) -> None:
        self._validate_enabled_launch('enable_vendor_bringup', 'vendor_bringup_launch')
        self._validate_enabled_launch('enable_vendor_navigation', 'vendor_navigation_launch')
        self._validate_enabled_launch('enable_vendor_camera', 'vendor_camera_launch')
        self._validate_enabled_launch('enable_vendor_lidar', 'vendor_lidar_launch')
        self._validate_enabled_launch('enable_vendor_imu', 'vendor_imu_launch')
        self._validate_wrapper_driver('enable_vendor_camera', 'vendor_camera_launch', 'vendor_camera_driver_launch', 'OrbbecSDK_ROS/dabai_dcw2.launch')
        self._validate_wrapper_driver('enable_vendor_lidar', 'vendor_lidar_launch', 'vendor_lidar_driver_launch', 'lslidar_driver/launch/lslidar_serial.launch')
        self._validate_wrapper_driver('enable_vendor_imu', 'vendor_imu_launch', 'vendor_imu_driver_launch', 'wit_ros_imu/launch/wit_imu.launch')

    def _details(self) -> Dict[str, object]:
        return {
            'deploy_grade': self.config['deploy_grade'],
            'vendor_bringup_launch': self.config['vendor_bringup_launch'],
            'vendor_navigation_launch': self.config['vendor_navigation_launch'],
            'vendor_camera_launch': self.config['vendor_camera_launch'],
            'vendor_lidar_launch': self.config['vendor_lidar_launch'],
            'vendor_imu_launch': self.config['vendor_imu_launch'],
            'vendor_camera_driver_launch': self.config['vendor_camera_driver_launch'],
            'vendor_lidar_driver_launch': self.config['vendor_lidar_driver_launch'],
            'vendor_imu_driver_launch': self.config['vendor_imu_driver_launch'],
            'enable_vendor_bringup': self.config['enable_vendor_bringup'],
            'enable_vendor_navigation': self.config['enable_vendor_navigation'],
            'enable_vendor_camera': self.config['enable_vendor_camera'],
            'enable_vendor_lidar': self.config['enable_vendor_lidar'],
            'enable_vendor_imu': self.config['enable_vendor_imu'],
        }

    def _publish_health(self, status: str, message: str, details: Dict[str, object]) -> None:
        payload = {
            'stamp': rospy.get_time(),
            'node': 'vendor_runtime_guard_node',
            'status': status,
            'message': message,
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
        typed.details_json = JsonCodec.dumps(payload['details'])
        self.health_typed_pub.publish(typed)

    def spin(self) -> None:
        rospy.spin()


if __name__ == '__main__':
    try:
        VendorRuntimeGuardNode().spin()
    except rospy.ROSInterruptException:
        pass

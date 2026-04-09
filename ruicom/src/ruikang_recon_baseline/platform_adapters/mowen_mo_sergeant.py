"""MO-SERGEANT platform contract for the 墨问/睿抗 integration line."""

from __future__ import annotations

from .base import PlatformAdapterCapability


MOWEN_MO_SERGEANT_PLATFORM = PlatformAdapterCapability(
    name='mowen_mo_sergeant',
    description='MO-SERGEANT mecanum platform bridged through the vendor ROS navigation stack and a downstream base-feedback watchdog.',
    shared_defaults={
        'vendor_runtime_mode': 'isolated_legacy_workspace',
        'vendor_workspace_ros_distro': 'melodic',
        'vendor_workspace_python_major': 2,
        'motion_model': 'mecanum_holonomic',
        'cmd_vel_semantics': 'planar_xy_yaw',
        'allow_odom_feedback_fallback': False,
    },
    mission_defaults={
        'navigation_adapter_type': 'move_base_action',
        'move_base_action_name': 'move_base',
        'pose_source_type': 'amcl_pose',
        'odom_topic': 'odom',
        'amcl_pose_topic': 'amcl_pose',
        'tf_target_frame': 'map',
        'tf_source_frame': 'base_link',
    },
    vision_defaults={
        'camera_topic': '/camera/rgb/image_raw',
    },
    safety_defaults={
        'input_topic': 'cmd_vel_raw',
        'output_topic': 'cmd_vel',
        'output_feedback_topic': 'recon/platform/base_feedback',
        'require_output_feedback': True,
        'output_feedback_timeout_sec': 0.6,
    },
    mission_required_topics=('amcl_pose', 'odom', 'recon/navigation_status'),
    mission_required_actions=('move_base',),
    vision_required_topics=('camera/rgb/image_raw',),
    safety_required_topics=('cmd_vel_raw', 'recon/platform/base_feedback', 'recon/estop', 'recon/control_mode'),
    bridge_required_topics=('cmd_vel', 'cmd_vel_raw', 'recon/platform/base_feedback'),
)

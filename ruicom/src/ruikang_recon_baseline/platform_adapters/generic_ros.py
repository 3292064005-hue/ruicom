"""Generic ROS-navigation platform contract."""

from __future__ import annotations

from .base import PlatformAdapterCapability


GENERIC_ROS_PLATFORM = PlatformAdapterCapability(
    name='generic_ros_nav',
    description='Generic ROS1 navigation stack with move_base, odom and pose topics.',
    shared_defaults={
        'vendor_runtime_mode': 'native_noetic',
        'vendor_workspace_ros_distro': 'noetic',
        'vendor_workspace_python_major': 3,
        'motion_model': 'differential',
        'cmd_vel_semantics': 'planar_x_yaw',
        'allow_odom_feedback_fallback': True,
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
        'camera_topic': 'camera/color/image_raw',
    },
    safety_defaults={
        'input_topic': 'cmd_vel_raw',
        'output_topic': 'cmd_vel',
        'output_feedback_topic': '',
        'require_output_feedback': False,
        'output_feedback_timeout_sec': 1.0,
    },
    mission_required_topics=('amcl_pose', 'odom', 'recon/navigation_status'),
    mission_required_actions=('move_base',),
    vision_required_topics=('camera/color/image_raw',),
    safety_required_topics=('cmd_vel_raw', 'recon/estop', 'recon/control_mode'),
    bridge_required_topics=('cmd_vel', 'cmd_vel_raw'),
)

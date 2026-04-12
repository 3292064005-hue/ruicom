"""Mission-node configuration loading and normalization."""

from __future__ import annotations

from typing import Dict

from .common import (
    CLASS_NAMES,
    ConfigurationError,
    detector_manifest_satisfies_scope,
    load_manifest,
    load_waypoints,
    require_positive_float,
    validate_profile_runtime_flags,
)
from .deploy_contracts import validate_deploy_stage_contract
from .field_assets import apply_field_asset_to_mission_config
from .mission_dsl import load_task_graph_dsl
from .navigation_contracts import (
    apply_navigation_contract_defaults,
    validate_navigation_contract_bindings,
    validate_navigation_runtime_strategy,
)
from .platform_adapters import apply_platform_mission_defaults, validate_platform_contract_bindings, validate_platform_runtime_strategy
from .runtime_paths import resolve_package_relative_path
from .vendor_runtime_contracts import (
    build_vendor_runtime_binding_report,
    load_vendor_runtime_contract,
    validate_vendor_runtime_contract,
)
from .vendor_bundle_preflight import build_vendor_bundle_preflight_report, enforce_vendor_bundle_preflight


VALID_POSE_SOURCE_TYPES = ('odometry', 'amcl_pose', 'tf_lookup')
VALID_NAVIGATION_ADAPTER_TYPES = ('move_base_action', 'simple_topic', 'goal_topic_status')



def read_mission_config(rospy_module) -> Dict:
    """Read, normalize and validate mission-node ROS parameters.

    Field assets remain optional for integration profiles. When configured they
    become the authoritative source for route and frame-region bindings. The
    navigation stack is validated as an explicit planner/controller/recovery/
    localization contract rather than a descriptive afterthought.
    """
    config = {
        'current_zone_topic': rospy_module.get_param('~current_zone_topic', 'recon/current_zone'),
        'mission_state_topic': rospy_module.get_param('~mission_state_topic', 'recon/mission_state'),
        'mission_state_typed_topic': rospy_module.get_param('~mission_state_typed_topic', 'recon/mission_state_typed'),
        'zone_capture_topic': rospy_module.get_param('~zone_capture_topic', 'recon/zone_capture_result'),
        'zone_capture_typed_topic': rospy_module.get_param('~zone_capture_typed_topic', 'recon/zone_capture_result_typed'),
        'zone_capture_dynamic_topic': rospy_module.get_param('~zone_capture_dynamic_topic', 'recon/zone_capture_result_dynamic'),
        'detections_topic': rospy_module.get_param('~detections_topic', 'recon/detections'),
        'health_topic': rospy_module.get_param('~health_topic', 'recon/health'),
        'health_typed_topic': rospy_module.get_param('~health_typed_topic', 'recon/health_typed'),
        'health_frame_id': str(rospy_module.get_param('~health_frame_id', rospy_module.get_param('~comparison_frame', 'map'))).strip() or str(rospy_module.get_param('~comparison_frame', 'map')).strip() or 'map',
        'output_root': rospy_module.get_param('~output_root', '~/.ros/ruikang_recon'),
        'output_root_use_namespace': bool(rospy_module.get_param('~output_root_use_namespace', True)),
        'lifecycle_managed': bool(rospy_module.get_param('~lifecycle_managed', False)),
        'control_command_topic': str(rospy_module.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
        'auto_start': bool(rospy_module.get_param('~auto_start', True)),
        'publish_rate_hz': float(rospy_module.get_param('~publish_rate_hz', 5.0)),
        'goal_reach_tolerance_m': float(rospy_module.get_param('~goal_reach_tolerance_m', 0.35)),
        'pose_timeout_sec': float(rospy_module.get_param('~pose_timeout_sec', 1.5)),
        'detections_timeout_sec': float(rospy_module.get_param('~detections_timeout_sec', rospy_module.get_param('~pose_timeout_sec', 1.5))),
        'comparison_frame': rospy_module.get_param('~comparison_frame', 'map'),
        'dwell_default_sec': float(rospy_module.get_param('~dwell_default_sec', 4.0)),
        'mission_timeout_sec': float(rospy_module.get_param('~mission_timeout_sec', 240.0)),
        'retry_limit': int(rospy_module.get_param('~retry_limit', 1)),
        'capture_reduction': rospy_module.get_param('~capture_reduction', 'median'),
        'capture_min_valid_frames': int(rospy_module.get_param('~capture_min_valid_frames', 2)),
        'platform_adapter_type': str(rospy_module.get_param('~platform_adapter_type', 'generic_ros_nav')).strip() or 'generic_ros_nav',
        'vendor_runtime_mode': str(rospy_module.get_param('~vendor_runtime_mode', '')).strip(),
        'vendor_workspace_ros_distro': str(rospy_module.get_param('~vendor_workspace_ros_distro', '')).strip(),
        'vendor_workspace_python_major': rospy_module.get_param('~vendor_workspace_python_major', 0),
        'vendor_runtime_contract_path': str(rospy_module.get_param('~vendor_runtime_contract_path', '')).strip(),
        'vendor_bundle_preflight_mode': str(rospy_module.get_param('~vendor_bundle_preflight_mode', 'off')).strip().lower() or 'off',
        'vendor_workspace_root': str(rospy_module.get_param('~vendor_workspace_root', '')).strip(),
        'vendor_workspace_root_env': str(rospy_module.get_param('~vendor_workspace_root_env', 'MOWEN_VENDOR_WORKSPACE_ROOT')).strip() or 'MOWEN_VENDOR_WORKSPACE_ROOT',
        'motion_model': str(rospy_module.get_param('~motion_model', '')).strip(),
        'cmd_vel_semantics': str(rospy_module.get_param('~cmd_vel_semantics', '')).strip(),
        'allow_odom_feedback_fallback': bool(rospy_module.get_param('~allow_odom_feedback_fallback', False)),
        'navigation_adapter_type': rospy_module.get_param('~navigation_adapter_type', ''),
        'move_base_action_name': rospy_module.get_param('~move_base_action_name', ''),
        'wait_for_action_server_sec': float(rospy_module.get_param('~wait_for_action_server_sec', 3.0)),
        'simple_goal_topic': rospy_module.get_param('~simple_goal_topic', 'move_base_simple/goal'),
        'simple_goal_topic_type': str(rospy_module.get_param('~simple_goal_topic_type', 'geometry_msgs/PoseStamped')).strip(),
        'navigation_status_topic': rospy_module.get_param('~navigation_status_topic', 'recon/navigation_status'),
        'navigation_status_topic_type': str(rospy_module.get_param('~navigation_status_topic_type', 'actionlib_msgs/GoalStatusArray')).strip(),
        'navigation_cancel_topic': rospy_module.get_param('~navigation_cancel_topic', ''),
        'navigation_cancel_topic_type': str(rospy_module.get_param('~navigation_cancel_topic_type', 'actionlib_msgs/GoalID')).strip(),
        'navigation_status_timeout_sec': float(rospy_module.get_param('~navigation_status_timeout_sec', 3.0)),
        'navigation_runtime_live_timeout_sec': float(rospy_module.get_param('~navigation_runtime_live_timeout_sec', rospy_module.get_param('~navigation_status_timeout_sec', 3.0))),
        'navigation_failure_quiesce_sec': float(rospy_module.get_param('~navigation_failure_quiesce_sec', 0.5)),
        'pose_source_type': rospy_module.get_param('~pose_source_type', ''),
        'odom_topic': rospy_module.get_param('~odom_topic', ''),
        'odom_topic_type': str(rospy_module.get_param('~odom_topic_type', 'nav_msgs/Odometry')).strip(),
        'amcl_pose_topic': rospy_module.get_param('~amcl_pose_topic', ''),
        'amcl_pose_topic_type': str(rospy_module.get_param('~amcl_pose_topic_type', 'geometry_msgs/PoseWithCovarianceStamped')).strip(),
        'tf_target_frame': rospy_module.get_param('~tf_target_frame', rospy_module.get_param('~comparison_frame', 'map')),
        'tf_source_frame': rospy_module.get_param('~tf_source_frame', 'base_link'),
        'tf_lookup_timeout_sec': float(rospy_module.get_param('~tf_lookup_timeout_sec', 0.2)),
        'simulate_arrival_without_pose': bool(rospy_module.get_param('~simulate_arrival_without_pose', False)),
        'synthetic_arrival_delay_sec': float(rospy_module.get_param('~synthetic_arrival_delay_sec', 1.0)),
        'navigation_planner_backend': str(rospy_module.get_param('~navigation_planner_backend', '')).strip(),
        'navigation_controller_backend': str(rospy_module.get_param('~navigation_controller_backend', '')).strip(),
        'navigation_recovery_backend': str(rospy_module.get_param('~navigation_recovery_backend', '')).strip(),
        'navigation_localization_backend': str(rospy_module.get_param('~navigation_localization_backend', '')).strip(),
        'navigation_goal_transport': str(rospy_module.get_param('~navigation_goal_transport', '')).strip(),
        'navigation_status_transport': str(rospy_module.get_param('~navigation_status_transport', '')).strip(),
        'navigation_cancel_transport': str(rospy_module.get_param('~navigation_cancel_transport', '')).strip(),
        'navigation_backend_profile': str(rospy_module.get_param('~navigation_backend_profile', '')).strip(),
        'time_source_mode': rospy_module.get_param('~time_source_mode', 'ros'),
        'classes': rospy_module.get_param('~classes', list(CLASS_NAMES)),
        'class_schema_mismatch_policy': str(rospy_module.get_param('~class_schema_mismatch_policy', 'error')).strip().lower(),
        'preflight_require_pose': bool(rospy_module.get_param('~preflight_require_pose', True)),
        'preflight_require_detections': bool(rospy_module.get_param('~preflight_require_detections', True)),
        'preflight_timeout_sec': float(rospy_module.get_param('~preflight_timeout_sec', 5.0)),
        'preflight_failure_policy': str(rospy_module.get_param('~preflight_failure_policy', 'error')).strip().lower(),
        'embed_zone_results_in_state': bool(rospy_module.get_param('~embed_zone_results_in_state', False)),
        'profile_role': str(rospy_module.get_param('~profile_role', 'deploy')).strip().lower(),
        'runtime_grade': str(rospy_module.get_param('~runtime_grade', 'integration')).strip().lower() or 'integration',
        'require_route_frame_regions': bool(rospy_module.get_param('~require_route_frame_regions', False)),
        'expected_frame_regions': rospy_module.get_param('~expected_frame_regions', []),
        'route': rospy_module.get_param('~route', []),
        'tasks': rospy_module.get_param('~tasks', []),
        'task_dsl_path': str(rospy_module.get_param('~task_dsl_path', '')).strip(),
        'task_dsl_format': str(rospy_module.get_param('~task_dsl_format', 'yaml')).strip().lower() or 'yaml',
        'field_asset_release_manifest_path': str(rospy_module.get_param('~field_asset_release_manifest_path', '')).strip(),
        'field_asset_id': str(rospy_module.get_param('~field_asset_id', '')).strip(),
        'field_asset_path': str(rospy_module.get_param('~field_asset_path', '')).strip(),
        'field_asset_package_root': str(rospy_module.get_param('~field_asset_package_root', '')).strip(),
        'model_manifest_path': str(rospy_module.get_param('~model_manifest_path', '')).strip(),
        'require_verified_field_asset': bool(rospy_module.get_param('~require_verified_field_asset', False)),
        'required_field_asset_verification_scope': str(rospy_module.get_param('~required_field_asset_verification_scope', 'contract')).strip().lower() or 'contract',
        'required_field_asset_provenance': str(rospy_module.get_param('~required_field_asset_provenance', '')).strip().lower(),
        'writer_queue_size': int(rospy_module.get_param('~writer_queue_size', 512)),
        'writer_rotate_max_bytes': int(rospy_module.get_param('~writer_rotate_max_bytes', 5 * 1024 * 1024)),
        'writer_rotate_keep': int(rospy_module.get_param('~writer_rotate_keep', 3)),
        'behavior_action_backend_type': str(rospy_module.get_param('~behavior_action_backend_type', 'disabled')).strip().lower() or 'disabled',
        'behavior_command_topic': str(rospy_module.get_param('~behavior_command_topic', 'recon/behavior_command')).strip() or 'recon/behavior_command',
        'behavior_feedback_topic': str(rospy_module.get_param('~behavior_feedback_topic', 'recon/behavior_feedback')).strip() or 'recon/behavior_feedback',
        'behavior_action_timeout_sec': float(rospy_module.get_param('~behavior_action_timeout_sec', 3.0)),
        'require_behavior_feedback': bool(rospy_module.get_param('~require_behavior_feedback', True)),
        'behavior_action_memory_outcome_status': str(rospy_module.get_param('~behavior_action_memory_outcome_status', 'SUCCEEDED')).strip().upper() or 'SUCCEEDED',
        'behavior_action_memory_delay_sec': float(rospy_module.get_param('~behavior_action_memory_delay_sec', 0.0)),
        'behavior_action_memory_feedback_details': rospy_module.get_param('~behavior_action_memory_feedback_details', {}),
    }

    capability = apply_platform_mission_defaults(config)
    config['platform_contract'] = capability.summary()
    config['platform_contract_bindings'] = validate_platform_contract_bindings(
        capability,
        config,
        owner='mission_manager_node',
        domain='mission',
    )
    config['platform_runtime_contract'] = validate_platform_runtime_strategy(
        capability,
        config,
        owner='mission_manager_node',
    )
    vendor_runtime_contract = load_vendor_runtime_contract(config.get('vendor_runtime_contract_path', ''))
    if vendor_runtime_contract:
        config['vendor_runtime_contract_path'] = str(vendor_runtime_contract.get('path', '')).strip()
    config['vendor_runtime_contract'] = validate_vendor_runtime_contract(
        vendor_runtime_contract,
        config,
        owner='mission_manager_node',
        domain='mission',
    )
    config['vendor_runtime_contract_satisfied'] = bool(config['vendor_runtime_contract'].get('satisfied', False))
    config['vendor_runtime_binding_report'] = build_vendor_runtime_binding_report(
        config['vendor_runtime_contract'],
        config,
    )
    config['vendor_bundle_preflight'] = build_vendor_bundle_preflight_report(
        config['vendor_runtime_contract'],
        config,
    )
    config['vendor_bundle_preflight_satisfied'] = bool(config['vendor_bundle_preflight'].get('satisfied', True))
    enforce_vendor_bundle_preflight(config['vendor_bundle_preflight'], owner='mission_manager_node')
    for param_name, binding in dict(config['vendor_runtime_contract'].get('required_bindings', {})).items():
        expected_type = str(binding.get('expected_type', '')).strip()
        if not expected_type:
            continue
        if param_name == 'odom_topic' and not str(config.get('odom_topic_type', '')).strip():
            config['odom_topic_type'] = expected_type
        elif param_name == 'amcl_pose_topic' and not str(config.get('amcl_pose_topic_type', '')).strip():
            config['amcl_pose_topic_type'] = expected_type
        elif param_name == 'move_base_action_name':
            config.setdefault('action_status_topic_type', 'actionlib_msgs/GoalStatusArray')

    if str(config['pose_source_type']).strip() not in VALID_POSE_SOURCE_TYPES:
        raise ConfigurationError('Unsupported pose_source_type: {}'.format(config['pose_source_type']))
    if str(config['navigation_adapter_type']).strip().lower() not in VALID_NAVIGATION_ADAPTER_TYPES:
        raise ConfigurationError('Unsupported navigation_adapter_type: {}'.format(config['navigation_adapter_type']))

    config, field_asset = apply_field_asset_to_mission_config(
        config,
        package_root=config.get('field_asset_package_root') or None,
        owner='mission_manager_node',
        domain='mission',
    )
    if field_asset is not None:
        config['comparison_frame'] = str(config.get('comparison_frame', '')).strip() or field_asset.comparison_frame
        config.setdefault('health_frame_id', config['comparison_frame'])
    if config.get('task_dsl_path'):
        if config.get('tasks'):
            raise ConfigurationError('mission_manager_node must not define both tasks and task_dsl_path')
        route = load_waypoints(config.get('route', []), config.get('dwell_default_sec', 4.0))
        config['tasks'] = load_task_graph_dsl(
            path=str(config.get('task_dsl_path', '')).strip(),
            route=route,
            file_format=str(config.get('task_dsl_format', 'yaml')).strip().lower() or 'yaml',
        )
        config['task_graph_contract'] = {
            'dsl_path': str(config.get('task_dsl_path', '')).strip(),
            'dsl_format': str(config.get('task_dsl_format', 'yaml')).strip().lower() or 'yaml',
            'task_count': len(config['tasks']),
        }
    else:
        config['task_graph_contract'] = {
            'dsl_path': '',
            'dsl_format': str(config.get('task_dsl_format', 'yaml')).strip().lower() or 'yaml',
            'task_count': len(config.get('tasks', []) or []),
        }

    navigation_capability = apply_navigation_contract_defaults(config)
    config['navigation_contract_bindings'] = validate_navigation_contract_bindings(config, owner='mission_manager_node')
    config['navigation_runtime_contract'] = validate_navigation_runtime_strategy(config, owner='mission_manager_node')
    config['navigation_contract_satisfied'] = True
    config['navigation_capability'] = navigation_capability

    if str(config['class_schema_mismatch_policy']) not in ('warn', 'error'):
        raise ConfigurationError('class_schema_mismatch_policy must be warn or error')
    if str(config['preflight_failure_policy']) not in ('warn', 'error'):
        raise ConfigurationError('preflight_failure_policy must be warn or error')
    if str(config['runtime_grade']) not in ('integration', 'contract', 'reference', 'field'):
        raise ConfigurationError('runtime_grade must be one of: integration, contract, reference, field')
    if str(config['behavior_action_backend_type']) not in ('disabled', 'memory', 'topic'):
        raise ConfigurationError('behavior_action_backend_type must be one of: disabled, memory, topic')
    if bool(config['require_behavior_feedback']) and str(config['behavior_action_backend_type']) == 'topic' and not str(config['behavior_feedback_topic']).strip():
        raise ConfigurationError('behavior_feedback_topic must not be empty when require_behavior_feedback=true')
    if str(config['capture_reduction']) not in ('median', 'max'):
        raise ConfigurationError('capture_reduction must be one of: median, max')
    if str(config['time_source_mode']).strip().lower() not in ('ros', 'wall'):
        raise ConfigurationError('time_source_mode must be ros or wall')
    if not str(config['comparison_frame']).strip():
        raise ConfigurationError('comparison_frame must not be empty')

    config['profile_role'] = validate_profile_runtime_flags(
        config['profile_role'],
        owner='mission_manager_node',
        lifecycle_managed=config['lifecycle_managed'],
        auto_start=config['auto_start'],
        require_route_frame_regions=config['require_route_frame_regions'],
    )

    manifest_path = resolve_package_relative_path(config.get('model_manifest_path', ''))
    if manifest_path:
        config['model_manifest_path'] = manifest_path
        manifest = load_manifest(manifest_path)
        required_manifest_scope = ''
        if config['profile_role'] == 'deploy':
            required_manifest_scope = str(config.get('required_field_asset_verification_scope', '')).strip().lower() or 'contract'
            if not detector_manifest_satisfies_scope(manifest, required_scope=required_manifest_scope):
                raise ConfigurationError(
                    'mission detector manifest {} grade {} does not satisfy required scope {}'.format(
                        manifest.model_id or config['model_manifest_path'],
                        manifest.deployment_grade or 'ungraded',
                        required_manifest_scope,
                    )
                )
        config['deploy_manifest_contract'] = {
            'path': manifest_path,
            'model_id': str(manifest.model_id).strip(),
            'deployment_grade': str(manifest.deployment_grade).strip(),
            'required_scope': required_manifest_scope,
            'scope_satisfied': bool(required_manifest_scope == '' or detector_manifest_satisfies_scope(manifest, required_scope=required_manifest_scope)),
        }
    else:
        config['deploy_manifest_contract'] = {
            'path': '',
            'model_id': '',
            'deployment_grade': '',
            'required_scope': '',
            'scope_satisfied': True,
        }
    config['deploy_stage_contract'] = validate_deploy_stage_contract(config, owner='mission_manager_node')

    if not isinstance(config['expected_frame_regions'], list):
        raise ConfigurationError('expected_frame_regions must be a list')
    config['expected_frame_regions'] = [str(item).strip() for item in config['expected_frame_regions'] if str(item).strip()]
    if not isinstance(config['tasks'], list):
        raise ConfigurationError('tasks must be a list')
    if config['tasks'] and config['route'] and not str(config.get('task_dsl_path', '')).strip():
        raise ConfigurationError('mission config must define either route or tasks, not both when task_dsl_path is unset')
    task_types = {str(item.get('task_type', '')).strip() for item in list(config['tasks'] or []) if isinstance(item, dict)}
    config['behavior_action_contract'] = {
        'backend_type': str(config['behavior_action_backend_type']),
        'command_topic': str(config['behavior_command_topic']),
        'feedback_topic': str(config['behavior_feedback_topic']),
        'timeout_sec': float(config['behavior_action_timeout_sec']),
        'require_feedback': bool(config['require_behavior_feedback']),
        'task_types_requiring_backend': sorted(task_types.intersection({'hazard_avoid', 'facility_attack'})),
    }
    if config['behavior_action_contract']['task_types_requiring_backend'] and str(config['runtime_grade']) in ('reference', 'field') and str(config['behavior_action_backend_type']) == 'disabled':
        raise ConfigurationError('runtime_grade {} with task-level execute/confirm steps requires behavior_action_backend_type'.format(config['runtime_grade']))

    for key in [
        'publish_rate_hz', 'goal_reach_tolerance_m', 'pose_timeout_sec', 'detections_timeout_sec',
        'dwell_default_sec', 'mission_timeout_sec', 'wait_for_action_server_sec',
        'navigation_status_timeout_sec', 'navigation_failure_quiesce_sec', 'tf_lookup_timeout_sec',
        'synthetic_arrival_delay_sec', 'preflight_timeout_sec', 'behavior_action_timeout_sec',
    ]:
        config[key] = require_positive_float(key, config[key])
    if int(config['retry_limit']) < 0:
        raise ConfigurationError('retry_limit must be >= 0')
    if int(config['capture_min_valid_frames']) <= 0:
        raise ConfigurationError('capture_min_valid_frames must be > 0')

    return config

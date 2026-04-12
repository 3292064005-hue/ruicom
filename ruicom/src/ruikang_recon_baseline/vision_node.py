#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS wrapper for frame-level vision detections and optional frame-region summaries."""

from __future__ import annotations

import os
from collections import deque

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import (
    CLASS_NAMES,
    SCHEMA_VERSION,
    ConfigurationError,
    DetectionFrame,
    JsonCodec,
    class_schema_hash,
    flatten_count_matrix,
    normalize_class_names,
    load_manifest,
    require_positive_float,
    resolve_output_root,
    resolve_package_relative_path,
    validate_dynamic_class_names,
    validate_named_region_contract,
    validate_named_regions,
    validate_profile_runtime_flags,
)
from .field_assets import apply_field_asset_to_vision_config
from .deploy_contracts import validate_deploy_stage_contract
from .platform_adapters import apply_platform_vision_defaults, validate_platform_contract_bindings, validate_platform_runtime_strategy
from .vendor_runtime_contracts import (
    build_vendor_runtime_binding_report,
    load_vendor_runtime_contract,
    validate_vendor_runtime_contract,
)
from .vendor_bundle_preflight import build_vendor_bundle_preflight_report, enforce_vendor_bundle_preflight
from .io_core import AsyncJsonlWriter
from .lifecycle_protocol import decode_lifecycle_control, encode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .manifest_utils import detector_manifest_satisfies_scope
from .model_requirements import enforce_onnx_model_requirement
from .time_core import NodeClock
from .vision_core import FrameRegionAdapter, build_detector, draw_overlay
from .msg import Detection as DetectionMsg
from .msg import DetectionArray, FrameRegionCounts, HealthState


class VisionCounterNode:
    """Publish frame detections, debug overlays and optional frame-region counts.

    The node intentionally does not convert frame-region counts into physical
    mission-zone results. Physical-zone aggregation is deferred to the mission
    manager so that task-space semantics remain independent from camera layout.
    """

    def __init__(self):
        rospy.init_node('vision_counter_node', anonymous=False)
        self.bridge = CvBridge()
        self.class_names = validate_dynamic_class_names(rospy.get_param('~classes', list(CLASS_NAMES)), owner='vision_counter_node')
        self.class_schema_hash = class_schema_hash(self.class_names)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self.output_root = resolve_output_root(
            self.config['output_root'],
            rospy.get_namespace(),
            self.config['output_root_use_namespace'],
        )
        os.makedirs(self.output_root, exist_ok=True)
        self.event_writer = AsyncJsonlWriter(
            path=os.path.join(self.output_root, 'vision_events.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.detector_type = ''
        self.detector_capability = {}
        self.detector = self._build_detector_with_policy()
        self.region_adapter = FrameRegionAdapter(
            self.config['frame_region_adapter_type'],
            self.config['named_regions'],
            region_calibration=self.config.get('region_calibration', {}),
        )
        self.region_history = {
            str(item['name']): deque(maxlen=max(1, int(self.config['stable_window'])))
            for item in self.config['named_regions']
        }
        self.last_health_publish_mono = 0.0
        self.last_artifact_save_mono = 0.0
        self.last_event_log_mono = 0.0
        self.last_frame_id = ''
        self.last_frame_stamp = 0.0
        self.frames_processed = 0

        self.detections_pub = rospy.Publisher(self.config['detections_topic'], DetectionArray, queue_size=10)
        self.detections_json_pub = rospy.Publisher(self.config['detections_json_topic'], String, queue_size=10)
        self.frame_region_counts_pub = rospy.Publisher(self.config['frame_region_counts_topic'], String, queue_size=10)
        self.frame_region_counts_typed_pub = rospy.Publisher(self.config['frame_region_counts_typed_topic'], FrameRegionCounts, queue_size=10)
        self.overlay_pub = rospy.Publisher(self.config['overlay_topic'], Image, queue_size=2)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        if self.config['lifecycle_managed']:
            self.control_command_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_command_callback, queue_size=20)
        else:
            self.control_command_sub = None
        self.image_sub = rospy.Subscriber(
            self.config['camera_topic'],
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self._publish_health('ok', 'node_ready', {'profile_role': self.config['profile_role'], 'detector_type': self.detector_type, 'topic_classes': self._topic_classification(), 'field_asset_ready': bool(self.config.get('field_asset_contract_satisfied', False)), 'vendor_runtime_binding_report': dict(self.config.get('vendor_runtime_binding_report', {})), 'vendor_bundle_preflight': dict(self.config.get('vendor_bundle_preflight', {})), 'vendor_bundle_preflight_satisfied': bool(self.config.get('vendor_bundle_preflight_satisfied', True)), **self.runtime.snapshot()})
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            'vision_counter_node started. detector=%s camera_topic=%s time_source=%s output_root=%s',
            self.detector_type,
            self.config['camera_topic'],
            self.config['time_source_mode'],
            self.output_root,
        )

    def _read_config(self):
        """Read and validate node-private ROS parameters.

        Args:
            None. The method pulls private ROS parameters from the node namespace.

        Returns:
            A validated configuration dictionary.

        Raises:
            ConfigurationError: If topic names, profile flags, detector runtime
                contracts, field-asset gates or detector-schema requirements are
                invalid.

        Boundary behavior:
            Deploy profiles may keep a color-blob backend only when they carry an
            explicit detector manifest contract. Relative manifest paths are
            resolved against the package root so profile YAML stays portable.
        """
        named_regions = rospy.get_param('~named_regions', [])
        config = {
            'camera_topic': rospy.get_param('~camera_topic', 'camera/color/image_raw'),
            'overlay_topic': rospy.get_param('~overlay_topic', 'recon/overlay/image_raw'),
            'detections_topic': rospy.get_param('~detections_topic', 'recon/detections'),
            'detections_json_topic': rospy.get_param('~detections_json_topic', 'recon/detections_json'),
            'frame_region_counts_topic': rospy.get_param('~frame_region_counts_topic', 'recon/zone_counts'),
            'frame_region_counts_typed_topic': rospy.get_param('~frame_region_counts_typed_topic', 'recon/zone_counts_typed'),
            'health_topic': rospy.get_param('~health_topic', 'recon/health'),
            'health_typed_topic': rospy.get_param('~health_typed_topic', 'recon/health_typed'),
            'health_frame_id': str(rospy.get_param('~health_frame_id', '')).strip(),
            'output_root': rospy.get_param('~output_root', '~/.ros/ruikang_recon'),
            'output_root_use_namespace': bool(rospy.get_param('~output_root_use_namespace', True)),
            'detector_type': rospy.get_param('~detector_type', 'color_blob'),
            'onnx_model_path': rospy.get_param('~onnx_model_path', ''),
            'onnx_model_path_env': str(rospy.get_param('~onnx_model_path_env', '')).strip(),
            'model_manifest_path': rospy.get_param('~model_manifest_path', ''),
            'parser_type': rospy.get_param('~parser_type', 'yolo_v5_like'),
            'input_size': int(rospy.get_param('~input_size', 640)),
            'confidence_threshold': float(rospy.get_param('~confidence_threshold', 0.35)),
            'score_threshold': float(rospy.get_param('~score_threshold', 0.25)),
            'nms_threshold': float(rospy.get_param('~nms_threshold', 0.45)),
            'min_area_px': int(rospy.get_param('~min_area_px', 180)),
            'max_area_px': int(rospy.get_param('~max_area_px', 80000)),
            'save_interval_sec': float(rospy.get_param('~save_interval_sec', 2.0)),
            'event_log_interval_sec': float(rospy.get_param('~event_log_interval_sec', 0.0)),
            'stable_window': int(rospy.get_param('~stable_window', 5)),
            'publish_debug_image': bool(rospy.get_param('~publish_debug_image', True)),
            'publish_json_debug_topics': bool(rospy.get_param('~publish_json_debug_topics', True)),
            'save_artifacts': bool(rospy.get_param('~save_artifacts', True)),
            'save_event_log': bool(rospy.get_param('~save_event_log', True)),
            'strict_backend': bool(rospy.get_param('~strict_backend', True)),
            'detector_schema_policy': str(rospy.get_param('~detector_schema_policy', 'subset_allowed')).strip().lower(),
            'frame_region_adapter_type': rospy.get_param('~frame_region_adapter_type', 'none'),
            'expected_region_names': rospy.get_param('~expected_region_names', []),
            'profile_role': str(rospy.get_param('~profile_role', 'deploy')).strip().lower(),
            'named_regions': named_regions,
            'region_calibration': rospy.get_param('~region_calibration', {}),
            'color_blob': rospy.get_param('~color_blob', {}),
            'writer_queue_size': int(rospy.get_param('~writer_queue_size', 512)),
            'writer_rotate_max_bytes': int(rospy.get_param('~writer_rotate_max_bytes', 5 * 1024 * 1024)),
            'writer_rotate_keep': int(rospy.get_param('~writer_rotate_keep', 3)),
            'time_source_mode': rospy.get_param('~time_source_mode', 'ros'),
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'platform_adapter_type': str(rospy.get_param('~platform_adapter_type', 'generic_ros_nav')).strip() or 'generic_ros_nav',
            'vendor_runtime_mode': str(rospy.get_param('~vendor_runtime_mode', '')).strip(),
            'vendor_workspace_ros_distro': str(rospy.get_param('~vendor_workspace_ros_distro', '')).strip(),
            'vendor_workspace_python_major': rospy.get_param('~vendor_workspace_python_major', 0),
            'vendor_runtime_contract_path': str(rospy.get_param('~vendor_runtime_contract_path', '')).strip(),
            'vendor_bundle_preflight_mode': str(rospy.get_param('~vendor_bundle_preflight_mode', 'off')).strip().lower() or 'off',
            'vendor_workspace_root': str(rospy.get_param('~vendor_workspace_root', '')).strip(),
            'vendor_workspace_root_env': str(rospy.get_param('~vendor_workspace_root_env', 'MOWEN_VENDOR_WORKSPACE_ROOT')).strip() or 'MOWEN_VENDOR_WORKSPACE_ROOT',
            'motion_model': str(rospy.get_param('~motion_model', '')).strip(),
            'cmd_vel_semantics': str(rospy.get_param('~cmd_vel_semantics', '')).strip(),
            'allow_odom_feedback_fallback': bool(rospy.get_param('~allow_odom_feedback_fallback', False)),
            'require_verified_field_asset': bool(rospy.get_param('~require_verified_field_asset', False)),
            'required_field_asset_verification_scope': str(rospy.get_param('~required_field_asset_verification_scope', 'contract')).strip().lower() or 'contract',
            'required_field_asset_provenance': str(rospy.get_param('~required_field_asset_provenance', '')).strip().lower(),
            'allowed_deploy_detector_types': rospy.get_param('~allowed_deploy_detector_types', ['onnx', 'color_blob']),
            'require_detector_manifest_in_deploy': bool(rospy.get_param('~require_detector_manifest_in_deploy', False)),
            'camera_ready_timeout_sec': float(rospy.get_param('~camera_ready_timeout_sec', 1.5)),
            'camera_topic_type': str(rospy.get_param('~camera_topic_type', 'sensor_msgs/Image')).strip(),
            'field_asset_release_manifest_path': str(rospy.get_param('~field_asset_release_manifest_path', '')).strip(),
        }
        config['input_size'] = int(require_positive_float('input_size', config['input_size']))
        config['camera_ready_timeout_sec'] = require_positive_float('camera_ready_timeout_sec', config['camera_ready_timeout_sec'])
        config['allowed_deploy_detector_types'] = [str(item).strip().lower() for item in config['allowed_deploy_detector_types'] if str(item).strip()]
        if not str(config.get('onnx_model_path', '')).strip() and str(config.get('onnx_model_path_env', '')).strip():
            env_value = os.environ.get(str(config.get('onnx_model_path_env', '')).strip(), '').strip()
            if env_value:
                config['onnx_model_path'] = env_value
        enforce_onnx_model_requirement(config, owner='vision_counter_node')
        config['confidence_threshold'] = require_positive_float('confidence_threshold', config['confidence_threshold'], allow_zero=True)
        config['score_threshold'] = require_positive_float('score_threshold', config['score_threshold'], allow_zero=True)
        config['nms_threshold'] = require_positive_float('nms_threshold', config['nms_threshold'], allow_zero=True)
        config['save_interval_sec'] = require_positive_float('save_interval_sec', config['save_interval_sec'], allow_zero=True)
        config['event_log_interval_sec'] = require_positive_float('event_log_interval_sec', config['event_log_interval_sec'], allow_zero=True)
        if int(config['stable_window']) <= 0:
            raise ConfigurationError('stable_window must be > 0')
        if int(config['min_area_px']) <= 0 or int(config['max_area_px']) < int(config['min_area_px']):
            raise ConfigurationError('min_area_px/max_area_px are invalid')
        if str(config['time_source_mode']).strip().lower() not in ('ros', 'wall'):
            raise ConfigurationError('time_source_mode must be ros or wall')
        if str(config['frame_region_adapter_type']).strip().lower() not in ('none', 'named_regions', 'calibrated_named_regions'):
            raise ConfigurationError('frame_region_adapter_type must be none, named_regions or calibrated_named_regions')
        if str(config['detector_schema_policy']) not in ('subset_allowed', 'full'):
            raise ConfigurationError('detector_schema_policy must be subset_allowed or full')
        config['profile_role'] = validate_profile_runtime_flags(
            config['profile_role'],
            owner='vision_counter_node',
            lifecycle_managed=bool(config['lifecycle_managed']),
        )
        config['deploy_stage_contract'] = validate_deploy_stage_contract(config, owner='vision_counter_node')
        capability = apply_platform_vision_defaults(config)
        config['platform_contract'] = capability.summary()
        config['platform_contract_bindings'] = validate_platform_contract_bindings(
            capability,
            config,
            owner='vision_counter_node',
            domain='vision',
        )
        config['platform_runtime_contract'] = validate_platform_runtime_strategy(
            capability,
            config,
            owner='vision_counter_node',
        )
        vendor_runtime_contract = load_vendor_runtime_contract(config.get('vendor_runtime_contract_path', ''))
        if vendor_runtime_contract:
            config['vendor_runtime_contract_path'] = str(vendor_runtime_contract.get('path', '')).strip()
        config['vendor_runtime_contract'] = validate_vendor_runtime_contract(
            vendor_runtime_contract,
            config,
            owner='vision_counter_node',
            domain='vision',
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
        enforce_vendor_bundle_preflight(config['vendor_bundle_preflight'], owner='vision_counter_node')
        for param_name, binding in dict(config['vendor_runtime_contract'].get('required_bindings', {})).items():
            expected_type = str(binding.get('expected_type', '')).strip()
            if param_name == 'camera_topic' and expected_type and not str(config.get('camera_topic_type', '')).strip():
                config['camera_topic_type'] = expected_type
        if not str(config['camera_topic']).strip():
            raise ConfigurationError('camera_topic must not be empty')
        if not isinstance(config['expected_region_names'], list):
            raise ConfigurationError('expected_region_names must be a list')
        config['expected_region_names'] = [str(item).strip() for item in config['expected_region_names'] if str(item).strip()]
        config['field_asset_id'] = str(rospy.get_param('~field_asset_id', '')).strip()
        config['field_asset_path'] = str(rospy.get_param('~field_asset_path', '')).strip()
        config['field_asset_package_root'] = str(rospy.get_param('~field_asset_package_root', '')).strip()
        config, field_asset = apply_field_asset_to_vision_config(
            config,
            package_root=config.get('field_asset_package_root') or None,
            owner='vision_counter_node',
        )
        if field_asset is not None:
            config['field_asset_metadata'] = dict(field_asset.metadata)
            config['field_asset_verified'] = bool(field_asset.verified)
            config['field_asset_state'] = field_asset.state
            config['field_asset_verification_scope'] = field_asset.verification_scope
        manifest_path = resolve_package_relative_path(config.get('model_manifest_path', ''))
        if manifest_path:
            config['model_manifest_path'] = manifest_path
            manifest = load_manifest(manifest_path)
            manifest_class_names = tuple(validate_dynamic_class_names(manifest.class_names, owner='vision_detector_manifest'))
            if config['detector_schema_policy'] == 'full' and manifest_class_names != tuple(self.class_names):
                raise ConfigurationError('detector manifest class_names {} must exactly match configured classes {}'.format(manifest_class_names, self.class_names))
            missing = [name for name in manifest_class_names if name not in self.class_names]
            if missing:
                raise ConfigurationError('detector manifest class_names {} are not a subset of configured classes {}'.format(missing, self.class_names))
            manifest_detector_type = str(manifest.detector_type or '').strip().lower()
            configured_detector_type = str(config.get('detector_type', '')).strip().lower()
            if manifest_detector_type and configured_detector_type and manifest_detector_type != configured_detector_type:
                raise ConfigurationError(
                    'detector manifest detector_type {} must match configured detector_type {}'.format(
                        manifest_detector_type,
                        configured_detector_type,
                    )
                )
            required_manifest_scope = ''
            if config['profile_role'] == 'deploy' and config['require_detector_manifest_in_deploy']:
                required_manifest_scope = str(config.get('required_field_asset_verification_scope', '')).strip().lower() or 'contract'
                if not detector_manifest_satisfies_scope(manifest, required_scope=required_manifest_scope):
                    raise ConfigurationError(
                        'deploy detector manifest {} grade {} does not satisfy required scope {}'.format(
                            manifest.model_id or config['model_manifest_path'],
                            manifest.deployment_grade or 'ungraded',
                            required_manifest_scope,
                        )
                    )
            config['detector_manifest_class_names'] = list(manifest_class_names)
            config['detector_manifest_model_id'] = str(manifest.model_id).strip()
            config['detector_manifest_detector_type'] = manifest_detector_type
            config['detector_manifest_deployment_grade'] = str(manifest.deployment_grade).strip()
            config['detector_manifest_required_scope'] = required_manifest_scope
            config['detector_manifest_grade_satisfied'] = bool(required_manifest_scope == '' or detector_manifest_satisfies_scope(manifest, required_scope=required_manifest_scope))
        elif config['profile_role'] == 'deploy' and config['require_detector_manifest_in_deploy']:
            raise ConfigurationError('deploy vision profile requires non-empty model_manifest_path')
        else:
            config['detector_manifest_grade_satisfied'] = True
        if config['frame_region_adapter_type'] != 'none':
            validate_named_region_contract(
                config['named_regions'],
                expected_region_names=config['expected_region_names'],
                require_named_regions=True,
                owner='vision_counter_node.named_regions',
            )
        elif config['named_regions'] or config['expected_region_names']:
            validate_named_region_contract(
                config['named_regions'],
                expected_region_names=config['expected_region_names'],
                require_named_regions=False,
                owner='vision_counter_node.named_regions',
            )
        validate_dynamic_class_names(self.class_names, owner='vision_counter_node.dynamic_schema')
        if config['profile_role'] == 'deploy' and config['allowed_deploy_detector_types'] and str(config['detector_type']).strip().lower() not in set(config['allowed_deploy_detector_types']):
            raise ConfigurationError('deploy profile detector_type {} must be one of {}'.format(config['detector_type'], config['allowed_deploy_detector_types']))
        return config

    def _topic_classification(self) -> dict:
        """Describe the semantic tier of the node's published topics.

        Args:
            None.

        Returns:
            Mapping that classifies authoritative, compatibility and debug outputs.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            Topics disabled by configuration are still listed so deploy profiles can
            expose their intended shape to operators and supervisors.
        """
        return {
            'authoritative': [self.config['detections_topic'], self.config['frame_region_counts_typed_topic']],
            'compatibility': [self.config['frame_region_counts_topic']],
            'debug': [self.config['detections_json_topic'], self.config['overlay_topic']],
        }

    def _build_detector_with_policy(self):
        """Construct the configured detector while enforcing schema compatibility.

        Args:
            None. Configuration and requested class ordering come from node state.

        Returns:
            A detector instance whose emitted class names are compatible with the
            configured authoritative class ordering.

        Raises:
            ConfigurationError: If the detector would emit classes outside the
                configured schema or if fallback policy cannot preserve compatibility.

        Boundary behavior:
            The color-blob backend is allowed to implement only the legacy
            ``friendly/enemy/hostage`` subset as long as that subset is contained in
            the configured dynamic class ordering.
        """
        try:
            self.detector_type, detector, capability = build_detector(self.config, self.class_names)
            self.detector_capability = {
                'name': capability.name,
                'schema_mode': capability.schema_mode,
                'required_config_keys': list(capability.required_config_keys),
                'description': capability.description,
            }
            detector_class_names = tuple(validate_dynamic_class_names(
                getattr(detector, 'class_names', self.class_names),
                owner='vision_detector_backend',
            ))
            missing = [name for name in detector_class_names if name not in self.class_names]
            if missing:
                raise ConfigurationError(
                    'Detector class schema {} is not a subset of configured classes {}'.format(
                        detector_class_names,
                        self.class_names,
                    )
                )
            if self.detector_type == 'onnx' and detector_class_names != tuple(self.class_names):
                raise ConfigurationError(
                    'ONNX detector class schema {} must exactly match configured classes {}'.format(
                        detector_class_names,
                        self.class_names,
                    )
                )
            if self.config['detector_schema_policy'] == 'full' and detector_class_names != tuple(self.class_names):
                raise ConfigurationError(
                    'Detector backend {} exposes subset schema {} but profile requires full schema {}'.format(
                        self.detector_type,
                        detector_class_names,
                        self.class_names,
                    )
                )
            return detector
        except Exception as exc:
            if self.config['detector_type'] == 'onnx' and not self.config['strict_backend']:
                rospy.logwarn('ONNX backend unavailable (%s), falling back to color_blob.', exc)
                fallback_config = dict(self.config)
                fallback_config['detector_type'] = 'color_blob'
                self.detector_type, detector, capability = build_detector(fallback_config, self.class_names)
                self.detector_capability = {
                    'name': capability.name,
                    'schema_mode': capability.schema_mode,
                    'required_config_keys': list(capability.required_config_keys),
                    'description': capability.description,
                }
                detector_class_names = tuple(validate_dynamic_class_names(
                    getattr(detector, 'class_names', self.class_names),
                    owner='vision_detector_backend_fallback',
                ))
                missing = [name for name in detector_class_names if name not in self.class_names]
                if missing:
                    raise ConfigurationError(
                        'Fallback detector class schema {} is not a subset of configured classes {}'.format(
                            detector_class_names,
                            self.class_names,
                        )
                    )
                if self.config['detector_schema_policy'] == 'full' and detector_class_names != tuple(self.class_names):
                    raise ConfigurationError(
                        'Fallback detector backend {} exposes subset schema {} but profile requires full schema {}'.format(
                            self.detector_type,
                            detector_class_names,
                            self.class_names,
                        )
                    )
                return detector
            raise

    def _publish_health(self, status: str, message: str, extra: dict | None = None) -> None:
        """Publish synchronized JSON and typed health status messages.

        Args:
            status: Health level such as ``ok`` or ``warn``.
            message: Human-readable health reason.
            extra: Optional structured diagnostics dictionary.

        Returns:
            None. Messages are published on ROS topics.

        Raises:
            No explicit exception is raised. Publisher transport issues are handled by ROS.

        Boundary behavior:
            ``ok`` health events are rate-limited so per-frame processing does not spam
            downstream observers, while warnings and errors bypass the throttle.
        """
        now_mono = self.clock.now_monotonic_sec()
        if status == 'ok' and (now_mono - self.last_health_publish_mono) < 0.5:
            return
        merged_details = dict(extra or {})
        merged_details.setdefault('runtime_state', self.runtime.state)
        merged_details.setdefault('lifecycle_managed', bool(self.config['lifecycle_managed']))
        merged_details.setdefault('camera_topic_bound', bool(str(self.config['camera_topic']).strip()))
        merged_details.setdefault('camera_frame_fresh', (self.clock.now_business_sec() - float(self.last_frame_stamp)) <= float(self.config['camera_ready_timeout_sec']) if self.last_frame_stamp > 0.0 else False)
        merged_details.setdefault('detector_contract_satisfied', True)
        merged_details.setdefault('detector_manifest_grade_satisfied', bool(self.config.get('detector_manifest_grade_satisfied', True)))
        merged_details.setdefault('detector_manifest_model_id', str(self.config.get('detector_manifest_model_id', '')).strip())
        merged_details.setdefault('detector_manifest_deployment_grade', str(self.config.get('detector_manifest_deployment_grade', '')).strip())
        merged_details.setdefault('onnx_model_requirement', dict(self.config.get('onnx_model_requirement', {})))
        merged_details.setdefault('field_asset_ready', bool(self.config.get('field_asset_contract_satisfied', False)))
        merged_details.setdefault('vendor_runtime_contract_satisfied', bool(self.config.get('vendor_runtime_contract_satisfied', True)))
        merged_details.setdefault('vendor_bundle_preflight', dict(self.config.get('vendor_bundle_preflight', {})))
        merged_details.setdefault('vendor_bundle_preflight_satisfied', bool(self.config.get('vendor_bundle_preflight_satisfied', True)))
        merged_details.setdefault('vendor_runtime_binding_report', dict(self.config.get('vendor_runtime_binding_report', {})))
        merged_details.setdefault('field_asset_verified', bool(self.config.get('field_asset_verified', False)))
        merged_details.setdefault('field_asset_state', str(self.config.get('field_asset_state', '')).strip())
        merged_details.setdefault('frames_processed', int(self.frames_processed))
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'vision_counter_node',
            'status': status,
            'message': message,
            'schema_version': SCHEMA_VERSION,
        }
        if merged_details:
            payload['details'] = merged_details
        writer_error = self.event_writer.last_error
        if writer_error:
            payload.setdefault('details', {})['writer_error'] = writer_error
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = self.clock.to_ros_time(payload['stamp'])
        typed.header.frame_id = str((extra or {}).get('frame_id', '')).strip() or self.last_frame_id or self.config['health_frame_id']
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(payload.get('details', {}))
        self.health_typed_pub.publish(typed)
        self.last_health_publish_mono = now_mono

    def _publish_typed_detections(self, frame: DetectionFrame) -> None:
        msg = DetectionArray()
        msg.header.stamp = self.clock.to_ros_time(frame.stamp)
        msg.header.frame_id = frame.frame_id
        msg.source_frame = frame.frame_id
        msg.detector_type = frame.detector_type
        msg.schema_version = frame.schema_version
        msg.source_image_width = int(frame.source_image_width)
        msg.source_image_height = int(frame.source_image_height)
        msg.class_names = list(frame.class_names or self.class_names)
        msg.class_schema_hash = str(frame.class_schema_hash or self.class_schema_hash)
        for item in frame.detections:
            det_msg = DetectionMsg()
            det_msg.class_name = item.class_name
            det_msg.score = float(item.score)
            det_msg.x1 = int(item.x1)
            det_msg.y1 = int(item.y1)
            det_msg.x2 = int(item.x2)
            det_msg.y2 = int(item.y2)
            det_msg.frame_region = item.frame_region
            det_msg.observed_position_type = str(item.observed_position_type)
            det_msg.observed_position_label = str(item.observed_position_label)
            det_msg.observed_position_x_m = float(item.observed_position_x_m)
            det_msg.observed_position_y_m = float(item.observed_position_y_m)
            det_msg.evidence_source = str(item.evidence_source)
            msg.detections.append(det_msg)
        self.detections_pub.publish(msg)

    def _region_counts_payload(self, frame: DetectionFrame) -> dict:
        counts = self.region_adapter.stable_region_counts(
            frame=frame,
            history=self.region_history,
            stable_window=int(self.config['stable_window']),
            class_names=self.class_names,
        )
        return {
            'stamp': frame.stamp,
            'frame_id': frame.frame_id,
            'schema_version': SCHEMA_VERSION,
            'frame_region_counts': counts,
            'mode': 'frame_regions',
            'class_names': list(self.class_names),
            'class_schema_hash': self.class_schema_hash,
        }

    def _publish_frame_region_counts(self, region_payload: dict) -> None:
        self.frame_region_counts_pub.publish(String(data=JsonCodec.dumps(region_payload)))
        region_names, counts_flat = flatten_count_matrix(region_payload['frame_region_counts'], self.class_names)
        msg = FrameRegionCounts()
        msg.header.stamp = self.clock.to_ros_time(region_payload['stamp'])
        msg.header.frame_id = region_payload['frame_id']
        msg.frame_id = region_payload['frame_id']
        msg.mode = region_payload['mode']
        msg.schema_version = region_payload['schema_version']
        msg.region_names = list(region_names)
        msg.class_names = list(self.class_names)
        msg.class_schema_hash = self.class_schema_hash
        msg.counts_flat = [int(item) for item in counts_flat]
        self.frame_region_counts_typed_pub.publish(msg)

    def _should_emit_event_log(self) -> bool:
        if not self.config['save_event_log']:
            return False
        interval = float(self.config['event_log_interval_sec'])
        if interval <= 0.0:
            return True
        now_mono = self.clock.now_monotonic_sec()
        if (now_mono - self.last_event_log_mono) < interval:
            return False
        self.last_event_log_mono = now_mono
        return True

    def _write_event_log(self, frame: DetectionFrame, region_payload: dict) -> None:
        """Persist lightweight structured events independently from image artifacts.

        Args:
            frame: Current detection frame.
            region_payload: Stable frame-region count payload for the same frame.

        Returns:
            None. Events are queued to the asynchronous JSONL writer.

        Raises:
            No explicit exception is raised. Queue overflow is handled by ``AsyncJsonlWriter``.

        Boundary behavior:
            Event logging remains active even when heavy image artifacts are disabled,
            preserving a complete structured audit trail for offline analysis.
        """
        if not self._should_emit_event_log():
            return
        wall_stamp = self.clock.now_wall_sec()
        self.event_writer.write({'type': 'detections', 'payload': frame.to_dict(), 'wall_stamp': wall_stamp})
        self.event_writer.write({'type': 'frame_region_counts', 'payload': region_payload, 'wall_stamp': wall_stamp})

    def _save_image_artifact(self, overlay_image, frame: DetectionFrame) -> None:
        """Persist an annotated overlay image according to the configured save interval.

        Args:
            overlay_image: OpenCV BGR image with annotations.
            frame: Detection frame associated with the image.

        Returns:
            None. The method writes an image file when allowed by policy.

        Raises:
            No explicit exception is raised. Failures are surfaced through health events.

        Boundary behavior:
            Saving is skipped when artifacts are disabled or when the configured interval
            has not elapsed since the previous successful save.
        """
        if not self.config['save_artifacts']:
            return
        now_mono = self.clock.now_monotonic_sec()
        if (now_mono - self.last_artifact_save_mono) < self.config['save_interval_sec']:
            return
        timestamp = '{:.3f}'.format(frame.stamp)
        saved = cv2.imwrite(os.path.join(self.output_root, 'annotated_{}.jpg'.format(timestamp)), overlay_image)
        if not saved:
            self._publish_health('warn', 'artifact_save_failed', {'timestamp': timestamp})
            return
        self.last_artifact_save_mono = now_mono

    def _reset_runtime(self) -> None:
        """Reset frame-local history and counters after an external lifecycle reset."""
        self.region_history = {
            str(item['name']): deque(maxlen=max(1, int(self.config['stable_window'])))
            for item in self.config['named_regions']
        }
        self.last_frame_id = ''
        self.last_artifact_save_mono = 0.0
        self.last_event_log_mono = 0.0

    def _control_command_callback(self, msg: String) -> None:
        """Handle system-manager lifecycle commands for vision processing."""
        envelope = decode_lifecycle_control(msg.data)
        command = envelope.command
        if not command or not envelope.matches(rospy.get_name().split('/')[-1]):
            return
        result = self.runtime.apply(command)
        details = {'command': command, 'target': envelope.target, 'issued_by': envelope.issued_by, 'metadata': dict(envelope.metadata or {})}
        if not result.accepted:
            self._publish_health('warn', result.message, details)
            return
        if command == 'reset':
            self._reset_runtime()
        self._publish_health('ok' if result.state == 'ACTIVE' else 'warn', result.message, details)

    def _image_callback(self, msg: Image) -> None:
        if not self.runtime.processing_allowed:
            return
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logerr_throttle(2.0, 'cv_bridge conversion failed: %s', exc)
            self._publish_health('error', 'cv_bridge_failed', {'error': str(exc)})
            return
        try:
            detections = self.detector.detect(frame_bgr)
        except Exception as exc:
            rospy.logerr_throttle(2.0, 'detector failed: %s', exc)
            self._publish_health('error', 'detector_failed', {'error': str(exc)})
            return
        frame_height, frame_width = int(frame_bgr.shape[0]), int(frame_bgr.shape[1])
        active_regions = self.region_adapter.region_payloads(frame_width, frame_height)
        detections = self.region_adapter.assign(detections, frame_width=frame_width, frame_height=frame_height)
        stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else self.clock.now_business_sec()
        self.last_frame_id = msg.header.frame_id
        self.last_frame_stamp = self.clock.now_business_sec()
        self.frames_processed += 1
        frame = DetectionFrame(
            stamp=float(stamp),
            frame_id=msg.header.frame_id,
            detector_type=self.detector_type,
            schema_version=SCHEMA_VERSION,
            detections=detections,
            source_image_width=int(frame_bgr.shape[1]),
            source_image_height=int(frame_bgr.shape[0]),
            class_names=list(self.class_names),
            class_schema_hash=self.class_schema_hash,
        )
        region_payload = self._region_counts_payload(frame)
        need_overlay = self.config['publish_debug_image'] or self.config['save_artifacts']
        overlay = draw_overlay(frame_bgr, detections, active_regions, region_payload['frame_region_counts'], self.class_names) if need_overlay else None
        self._publish_typed_detections(frame)
        if self.config['publish_json_debug_topics']:
            self.detections_json_pub.publish(String(data=JsonCodec.dumps(frame.to_dict())))
        self._publish_frame_region_counts(region_payload)
        self._write_event_log(frame, region_payload)
        if self.config['publish_debug_image'] and overlay is not None:
            try:
                self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
            except Exception as exc:
                rospy.logwarn_throttle(2.0, 'overlay publish failed: %s', exc)
        if overlay is not None:
            self._save_image_artifact(overlay, frame)
        self._publish_health('ok', 'frame_processed', {'detections': len(frame.detections), 'frame_id': frame.frame_id})

    def _on_shutdown(self) -> None:
        self.event_writer.close()

    def spin(self) -> None:
        rospy.spin()


if __name__ == '__main__':
    try:
        node = VisionCounterNode()
        node.spin()
    except (rospy.ROSInterruptException, ConfigurationError):
        raise

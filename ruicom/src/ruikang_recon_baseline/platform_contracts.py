"""Platform-contract binding and runtime validation helpers."""

from __future__ import annotations

from typing import TYPE_CHECKING, Dict, Iterable, Mapping, Sequence

from .domain_models import ConfigurationError

if TYPE_CHECKING:
    from .platform_adapters.base import PlatformAdapterCapability

TOPIC_BINDING_KEYS: Sequence[str] = (
    'camera_topic',
    'detections_topic',
    'odom_topic',
    'amcl_pose_topic',
    'simple_goal_topic',
    'navigation_status_topic',
    'navigation_cancel_topic',
    'output_feedback_topic',
    'estop_topic',
    'control_mode_topic',
    'upstream_command_topic',
    'command_input_topic',
)
ACTION_BINDING_KEYS: Sequence[str] = (
    'move_base_action_name',
    'navigation_action_name',
)
VALID_VENDOR_RUNTIME_MODES: Sequence[str] = ('native_noetic', 'isolated_legacy_workspace')
VALID_MOTION_MODELS: Sequence[str] = ('differential', 'mecanum_holonomic')
VALID_CMD_VEL_SEMANTICS: Sequence[str] = ('planar_x_yaw', 'planar_xy_yaw')
PLATFORM_RUNTIME_INTERFACE_REQUIRED_TOPICS: Sequence[str] = (
    'upstream_command_topic',
    'command_input_topic',
    'safety_output_topic',
    'output_feedback_topic',
    'control_mode_topic',
    'estop_topic',
    'runtime_evidence_topic',
)



def normalize_binding_name(value: object) -> str:
    """Normalize a topic/action resource name for contract comparisons.

    Args:
        value: Arbitrary resource identifier possibly containing leading or
            trailing whitespace and slashes.

    Returns:
        str: Slash-normalized resource path without leading/trailing slashes.

    Boundary behavior:
        Empty or non-string-like values normalize to an empty string so callers
        can treat missing bindings uniformly.
    """
    return str(value or '').strip().strip('/')



def binding_names_equal(left: object, right: object) -> bool:
    """Return whether two bindings resolve to the exact same resource path.

    Args:
        left: First logical or concrete binding.
        right: Second logical or concrete binding.

    Returns:
        bool: ``True`` when both values normalize to the same non-empty path.

    Boundary behavior:
        Namespace-aware suffix matching is intentionally *not* used here because
        command-loop protection must reject only exact self-aliases, not
        legitimate namespaced variants such as ``vendor/cmd_vel`` versus
        ``cmd_vel``.
    """
    normalized_left = normalize_binding_name(left)
    normalized_right = normalize_binding_name(right)
    return bool(normalized_left) and normalized_left == normalized_right



def validate_command_topic_flow_contract(
    config: Mapping[str, object],
    *,
    owner: str,
    upstream_command_topic_key: str = 'upstream_command_topic',
    command_input_topic_key: str = 'command_input_topic',
    safety_output_topic_key: str = 'safety_output_topic',
) -> Dict[str, str]:
    """Validate that bridge/safety command topics cannot self-loop.

    Args:
        config: Normalized launch/profile configuration mapping.
        owner: Human-readable validation owner for error messages.
        upstream_command_topic_key: Config key holding the vendor/upstream
            command topic observed by the platform bridge.
        command_input_topic_key: Config key holding the safety ingress topic.
        safety_output_topic_key: Config key holding the post-safety output topic.

    Returns:
        Dict[str, str]: Normalized flow summary for diagnostics.

    Raises:
        ConfigurationError: When any two command-flow topics collapse onto the
            same exact ROS resource path and would therefore create a self-loop
            or bypass the safety boundary.

    Boundary behavior:
        Only exact normalized equality is rejected. A namespaced upstream topic
        such as ``vendor/cmd_vel`` remains valid even though it logically ends in
        ``cmd_vel`` because it does not alias the safety output topic.
    """
    upstream_command_topic = normalize_binding_name(config.get(upstream_command_topic_key, ''))
    command_input_topic = normalize_binding_name(config.get(command_input_topic_key, ''))
    safety_output_topic = normalize_binding_name(config.get(safety_output_topic_key, ''))
    collisions: list[str] = []
    if binding_names_equal(upstream_command_topic, command_input_topic):
        collisions.append(
            '{} {} and {} both resolve to {}'.format(
                owner,
                upstream_command_topic_key,
                command_input_topic_key,
                upstream_command_topic,
            )
        )
    if binding_names_equal(upstream_command_topic, safety_output_topic):
        collisions.append(
            '{} {} and {} both resolve to {}'.format(
                owner,
                upstream_command_topic_key,
                safety_output_topic_key,
                upstream_command_topic,
            )
        )
    if binding_names_equal(command_input_topic, safety_output_topic):
        collisions.append(
            '{} {} and {} both resolve to {}'.format(
                owner,
                command_input_topic_key,
                safety_output_topic_key,
                command_input_topic,
            )
        )
    if collisions:
        raise ConfigurationError('; '.join(collisions))
    return {
        'satisfied': True,
        'upstream_command_topic': upstream_command_topic,
        'command_input_topic': command_input_topic,
        'safety_output_topic': safety_output_topic,
    }


def validate_platform_runtime_interface_contract(config: Mapping[str, object], *, owner: str) -> Dict[str, object]:
    """Validate the minimum runtime interface shared by bridge, safety and ops."""
    missing_topics = [
        key for key in PLATFORM_RUNTIME_INTERFACE_REQUIRED_TOPICS
        if not normalize_binding_name(config.get(key, ''))
    ]
    if missing_topics:
        raise ConfigurationError('{} missing platform runtime interface topics: {}'.format(owner, missing_topics))
    command_flow = validate_command_topic_flow_contract(config, owner=owner)
    return {
        'satisfied': True,
        'required_topic_keys': list(PLATFORM_RUNTIME_INTERFACE_REQUIRED_TOPICS),
        'bound_topics': {
            key: normalize_binding_name(config.get(key, ''))
            for key in PLATFORM_RUNTIME_INTERFACE_REQUIRED_TOPICS
        },
        'command_flow': command_flow,
    }



def _normalize_names(values: Iterable[object]) -> list[str]:
    normalized: list[str] = []
    for value in values:
        text = str(value or '').strip().strip('/')
        if text:
            normalized.append(text)
    return normalized


def _binding_satisfies(bound_name: str, required_name: str) -> bool:
    """Return whether one bound resource satisfies a logical contract name.

    The platform contract expresses logical resources such as ``odom`` or
    ``camera/rgb/image_raw``. Runtime launchers may legitimately place these
    resources under vendor namespaces, for example ``vendor/odom`` or
    ``vendor/camera/rgb/image_raw``. Exact string equality is therefore too
    strict for deploy/runtime validation and would reject correctly namespaced
    integrations.

    Matching rules:
    - exact match succeeds;
    - a namespaced binding also succeeds when its normalized path ends with the
      required logical resource boundary.

    Boundary behavior:
    ``foo/odom_extra`` does not satisfy ``odom`` because the suffix must match a
    full path boundary.
    """
    normalized_bound = normalize_binding_name(bound_name)
    normalized_required = normalize_binding_name(required_name)
    if not normalized_bound or not normalized_required:
        return False
    if normalized_bound == normalized_required:
        return True
    return normalized_bound.endswith('/' + normalized_required)



def collect_bound_topics(config: Mapping[str, object]) -> list[str]:
    return _normalize_names(config.get(key, '') for key in TOPIC_BINDING_KEYS)



def collect_bound_actions(config: Mapping[str, object]) -> list[str]:
    return _normalize_names(config.get(key, '') for key in ACTION_BINDING_KEYS)



def _required_for_domain(capability: PlatformAdapterCapability, domain: str) -> tuple[list[str], list[str]]:
    normalized = str(domain).strip().lower()
    if normalized == 'mission':
        return _normalize_names(capability.mission_required_topics), _normalize_names(capability.mission_required_actions)
    if normalized == 'vision':
        return _normalize_names(capability.vision_required_topics), _normalize_names(capability.vision_required_actions)
    if normalized == 'safety':
        return _normalize_names(capability.safety_required_topics), _normalize_names(capability.safety_required_actions)
    if normalized == 'bridge':
        return _normalize_names(capability.bridge_required_topics), _normalize_names(capability.bridge_required_actions)
    return _normalize_names(capability.required_topics), _normalize_names(capability.required_actions)



def validate_platform_contract_bindings(
    capability: PlatformAdapterCapability,
    config: Mapping[str, object],
    *,
    owner: str,
    domain: str = 'all',
) -> Dict[str, object]:
    bound_topics = collect_bound_topics(config)
    bound_actions = collect_bound_actions(config)
    required_topics, required_actions = _required_for_domain(capability, domain)
    missing_topics = [
        name for name in required_topics
        if not any(_binding_satisfies(bound, name) for bound in bound_topics)
    ]
    missing_actions = [
        name for name in required_actions
        if not any(_binding_satisfies(bound, name) for bound in bound_actions)
    ]
    if missing_topics or missing_actions:
        problems = []
        if missing_topics:
            problems.append('topics={}'.format(missing_topics))
        if missing_actions:
            problems.append('actions={}'.format(missing_actions))
        raise ConfigurationError(
            '{} does not satisfy {} platform contract {}: {}'.format(owner, domain, capability.name, ', '.join(problems))
        )
    return {
        'adapter_type': capability.name,
        'domain': domain,
        'required_topics': required_topics,
        'required_actions': required_actions,
        'bound_topics': bound_topics,
        'bound_actions': bound_actions,
    }



def validate_platform_runtime_strategy(
    capability: PlatformAdapterCapability,
    config: Mapping[str, object],
    *,
    owner: str,
) -> Dict[str, object]:
    """Validate shared runtime and motion contracts for one platform profile.

    Args:
        capability: Resolved platform capability.
        config: Normalized component configuration.
        owner: Human-readable validation owner.

    Returns:
        JSON-serializable runtime strategy summary.

    Raises:
        ConfigurationError: If runtime mode, ROS distro, Python major, motion
            model or command semantics violate the resolved platform contract.

    Boundary behavior:
        Generic platforms may remain entirely native Noetic, while vendor-backed
        MO-SERGEANT deployments are forced into an isolated legacy workspace
        strategy rather than pretending the old Melodic/Python2 stack is natively
        ABI-compatible with this repository.
    """
    vendor_runtime_mode = str(config.get('vendor_runtime_mode', '')).strip().lower()
    vendor_workspace_ros_distro = str(config.get('vendor_workspace_ros_distro', '')).strip().lower()
    try:
        vendor_workspace_python_major = int(config.get('vendor_workspace_python_major', 0))
    except (TypeError, ValueError) as exc:
        raise ConfigurationError('{} vendor_workspace_python_major is invalid: {}'.format(owner, exc))
    motion_model = str(config.get('motion_model', '')).strip().lower()
    cmd_vel_semantics = str(config.get('cmd_vel_semantics', '')).strip().lower()
    allow_odom_feedback_fallback = bool(config.get('allow_odom_feedback_fallback', False))

    if vendor_runtime_mode not in VALID_VENDOR_RUNTIME_MODES:
        raise ConfigurationError('{} vendor_runtime_mode must be one of {}'.format(owner, ', '.join(VALID_VENDOR_RUNTIME_MODES)))
    if motion_model not in VALID_MOTION_MODELS:
        raise ConfigurationError('{} motion_model must be one of {}'.format(owner, ', '.join(VALID_MOTION_MODELS)))
    if cmd_vel_semantics not in VALID_CMD_VEL_SEMANTICS:
        raise ConfigurationError('{} cmd_vel_semantics must be one of {}'.format(owner, ', '.join(VALID_CMD_VEL_SEMANTICS)))

    expected_runtime_mode = str(capability.shared_defaults.get('vendor_runtime_mode', '')).strip().lower()
    expected_ros_distro = str(capability.shared_defaults.get('vendor_workspace_ros_distro', '')).strip().lower()
    expected_python_major = int(capability.shared_defaults.get('vendor_workspace_python_major', vendor_workspace_python_major or 0) or 0)
    expected_motion_model = str(capability.shared_defaults.get('motion_model', '')).strip().lower()
    expected_cmd_vel_semantics = str(capability.shared_defaults.get('cmd_vel_semantics', '')).strip().lower()
    expected_odom_fallback = bool(capability.shared_defaults.get('allow_odom_feedback_fallback', allow_odom_feedback_fallback))

    if expected_runtime_mode and vendor_runtime_mode != expected_runtime_mode:
        raise ConfigurationError('{} vendor_runtime_mode {} must match platform contract {}'.format(owner, vendor_runtime_mode, expected_runtime_mode))
    if expected_motion_model and motion_model != expected_motion_model:
        raise ConfigurationError('{} motion_model {} must match platform contract {}'.format(owner, motion_model, expected_motion_model))
    if expected_cmd_vel_semantics and cmd_vel_semantics != expected_cmd_vel_semantics:
        raise ConfigurationError('{} cmd_vel_semantics {} must match platform contract {}'.format(owner, cmd_vel_semantics, expected_cmd_vel_semantics))
    if allow_odom_feedback_fallback != expected_odom_fallback:
        raise ConfigurationError('{} allow_odom_feedback_fallback {} must match platform contract {}'.format(owner, allow_odom_feedback_fallback, expected_odom_fallback))

    if motion_model == 'differential' and cmd_vel_semantics != 'planar_x_yaw':
        raise ConfigurationError('{} differential motion model requires cmd_vel_semantics=planar_x_yaw'.format(owner))
    if motion_model == 'mecanum_holonomic' and cmd_vel_semantics != 'planar_xy_yaw':
        raise ConfigurationError('{} mecanum_holonomic motion model requires cmd_vel_semantics=planar_xy_yaw'.format(owner))

    if vendor_runtime_mode == 'native_noetic':
        if vendor_workspace_ros_distro != 'noetic':
            raise ConfigurationError('{} native_noetic runtime requires vendor_workspace_ros_distro=noetic'.format(owner))
        if vendor_workspace_python_major != 3:
            raise ConfigurationError('{} native_noetic runtime requires vendor_workspace_python_major=3'.format(owner))
    else:
        if not vendor_workspace_ros_distro:
            raise ConfigurationError('{} isolated_legacy_workspace runtime requires non-empty vendor_workspace_ros_distro'.format(owner))
        if vendor_workspace_python_major <= 0:
            raise ConfigurationError('{} isolated_legacy_workspace runtime requires positive vendor_workspace_python_major'.format(owner))
        if expected_ros_distro and vendor_workspace_ros_distro != expected_ros_distro:
            raise ConfigurationError('{} vendor_workspace_ros_distro {} must match platform contract {}'.format(owner, vendor_workspace_ros_distro, expected_ros_distro))
        if expected_python_major and vendor_workspace_python_major != expected_python_major:
            raise ConfigurationError('{} vendor_workspace_python_major {} must match platform contract {}'.format(owner, vendor_workspace_python_major, expected_python_major))

    return {
        'adapter_type': capability.name,
        'vendor_runtime_mode': vendor_runtime_mode,
        'vendor_workspace_ros_distro': vendor_workspace_ros_distro,
        'vendor_workspace_python_major': vendor_workspace_python_major,
        'motion_model': motion_model,
        'cmd_vel_semantics': cmd_vel_semantics,
        'allow_odom_feedback_fallback': allow_odom_feedback_fallback,
    }

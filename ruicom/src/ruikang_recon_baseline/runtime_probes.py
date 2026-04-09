"""Pure helpers for runtime ROS graph probes.

These helpers keep launch/runtime resource checks testable without requiring a
live ROS master inside unit tests. Node wrappers can obtain published-topic
snapshots from ``rospy.get_published_topics()`` and feed them into these pure
functions to produce deterministic readiness diagnostics.
"""

from __future__ import annotations

from typing import Iterable, Mapping

from .domain_models import ConfigurationError

ACTIONLIB_STATUS_TOPIC_TYPE = 'actionlib_msgs/GoalStatusArray'
ACTIONLIB_FEEDBACK_TOPIC_TYPE = 'move_base_msgs/MoveBaseActionFeedback'
ACTIONLIB_RESULT_TOPIC_TYPE = 'move_base_msgs/MoveBaseActionResult'
TOPIC_GOAL_TYPE = 'geometry_msgs/PoseStamped'
TOPIC_CANCEL_TYPE = 'actionlib_msgs/GoalID'
ODOM_TOPIC_TYPE = 'nav_msgs/Odometry'
AMCL_POSE_TOPIC_TYPE = 'geometry_msgs/PoseWithCovarianceStamped'
FEEDBACK_TOPIC_TYPE = 'std_msgs/Bool'
COMMAND_TOPIC_TYPE = 'geometry_msgs/Twist'


def normalize_resource_name(value: object) -> str:
    return str(value or '').strip().strip('/')


def _matches_visible(bound_name: str, visible_name: str) -> bool:
    """Return whether one runtime probe binding exactly matches one visible topic.

    Runtime probes operate on already-resolved launch outputs. Unlike contract
    binding validation, they must not treat arbitrary suffix matches as valid
    because that can incorrectly mark another robot or another namespace as
    ready. Namespaced integrations remain valid as long as the configured probe
    binding itself carries the namespace (for example ``vendor_a/odom``).
    """
    bound = normalize_resource_name(bound_name)
    visible = normalize_resource_name(visible_name)
    if not bound or not visible:
        return False
    return bound == visible


def visible_topic_entries(published_topics: Iterable[object]) -> tuple[tuple[str, str], ...]:
    normalized: list[tuple[str, str]] = []
    for item in published_topics or ():
        if isinstance(item, (list, tuple)) and item:
            name = normalize_resource_name(item[0])
            topic_type = str(item[1]).strip() if len(item) > 1 else ''
        else:
            name = normalize_resource_name(item)
            topic_type = ''
        if name:
            normalized.append((name, topic_type))
    return tuple(normalized)


def visible_topic_names(published_topics: Iterable[object]) -> tuple[str, ...]:
    return tuple(name for name, _ in visible_topic_entries(published_topics))


def topic_visible(topic_name: object, published_topics: Iterable[object]) -> bool:
    bound = normalize_resource_name(topic_name)
    if not bound:
        return False
    return any(_matches_visible(bound, visible) for visible in visible_topic_names(published_topics))


def topic_type_matches(topic_name: object, published_topics: Iterable[object], expected_type: object) -> bool:
    bound = normalize_resource_name(topic_name)
    expected = str(expected_type or '').strip()
    if not bound or not expected:
        return True
    for visible_name, visible_type in visible_topic_entries(published_topics):
        if _matches_visible(bound, visible_name):
            return str(visible_type or '').strip() == expected
    return False


def action_runtime_topics(action_name: object) -> tuple[str, ...]:
    base = normalize_resource_name(action_name)
    if not base:
        return ()
    return tuple(f'{base}/{suffix}' for suffix in ('status', 'feedback', 'result'))


def evaluate_navigation_runtime_probe(
    config: Mapping[str, object],
    *,
    published_topics: Iterable[object],
    action_server_ready: bool,
    now_sec: float,
    runtime_live_stamp_sec: float = 0.0,
    runtime_live_timeout_sec: float = 0.0,
    feedback_live_stamp_sec: float = 0.0,
    result_live_stamp_sec: float = 0.0,
    dispatch_goal_stamp_sec: float = 0.0,
    require_action_roundtrip: bool = False,
) -> dict:
    """Evaluate whether navigation runtime resources are visible and usable."""
    goal_transport = str(config.get('navigation_goal_transport', '')).strip().lower()
    status_transport = str(config.get('navigation_status_transport', '')).strip().lower()
    cancel_transport = str(config.get('navigation_cancel_transport', '')).strip().lower()
    localization_backend = str(config.get('navigation_localization_backend', '')).strip().lower()
    if goal_transport not in ('actionlib', 'topic'):
        raise ConfigurationError('unsupported navigation_goal_transport {}'.format(goal_transport))
    if status_transport not in ('actionlib', 'topic', 'internal_pose_evaluator'):
        raise ConfigurationError('unsupported navigation_status_transport {}'.format(status_transport))
    if cancel_transport not in ('actionlib', 'topic', 'none'):
        raise ConfigurationError('unsupported navigation_cancel_transport {}'.format(cancel_transport))
    if localization_backend not in ('odometry_topic', 'amcl_pose_topic', 'tf_lookup_pose'):
        raise ConfigurationError('unsupported navigation_localization_backend {}'.format(localization_backend))

    action_name = str(config.get('move_base_action_name', '')).strip()
    action_status_topic = '{}/status'.format(normalize_resource_name(action_name)) if normalize_resource_name(action_name) else ''
    goal_topic = str(config.get('simple_goal_topic', '')).strip()
    status_topic = str(config.get('navigation_status_topic', '')).strip()
    cancel_topic = str(config.get('navigation_cancel_topic', '')).strip()
    odom_topic = str(config.get('odom_topic', '')).strip()
    amcl_pose_topic = str(config.get('amcl_pose_topic', '')).strip()

    runtime_live_timeout_sec = float(runtime_live_timeout_sec or 0.0)
    runtime_live = bool(
        runtime_live_timeout_sec > 0.0
        and runtime_live_stamp_sec > 0.0
        and (float(now_sec) - float(runtime_live_stamp_sec)) <= runtime_live_timeout_sec
    )

    feedback_topic = '{}/feedback'.format(normalize_resource_name(action_name)) if normalize_resource_name(action_name) else ''
    result_topic = '{}/result'.format(normalize_resource_name(action_name)) if normalize_resource_name(action_name) else ''
    feedback_live_stamp_sec = float(feedback_live_stamp_sec or 0.0)
    result_live_stamp_sec = float(result_live_stamp_sec or 0.0)
    dispatch_goal_stamp_sec = float(dispatch_goal_stamp_sec or 0.0)
    feedback_live = bool(
        runtime_live_timeout_sec > 0.0
        and feedback_live_stamp_sec > 0.0
        and (float(now_sec) - feedback_live_stamp_sec) <= runtime_live_timeout_sec
    )
    result_live = bool(
        runtime_live_timeout_sec > 0.0
        and result_live_stamp_sec > 0.0
        and (float(now_sec) - result_live_stamp_sec) <= runtime_live_timeout_sec
    )
    action_roundtrip_observed = bool(
        dispatch_goal_stamp_sec > 0.0
        and (feedback_live_stamp_sec >= dispatch_goal_stamp_sec or result_live_stamp_sec >= dispatch_goal_stamp_sec)
    )
    action_roundtrip_grace_active = bool(
        dispatch_goal_stamp_sec > 0.0
        and not action_roundtrip_observed
        and runtime_live_timeout_sec > 0.0
        and (float(now_sec) - dispatch_goal_stamp_sec) <= runtime_live_timeout_sec
    )
    action_roundtrip_live = (not bool(require_action_roundtrip)) or action_roundtrip_observed or action_roundtrip_grace_active

    probe = {
        'goal_transport': goal_transport,
        'status_transport': status_transport,
        'cancel_transport': cancel_transport,
        'localization_backend': localization_backend,
        'action_server_ready': bool(action_server_ready),
        'runtime_live_timeout_sec': runtime_live_timeout_sec,
        'runtime_live_stamp_sec': float(runtime_live_stamp_sec or 0.0),
        'runtime_live': runtime_live,
        'feedback_live_stamp_sec': feedback_live_stamp_sec,
        'feedback_runtime_live': feedback_live,
        'result_live_stamp_sec': result_live_stamp_sec,
        'result_runtime_live': result_live,
        'dispatch_goal_stamp_sec': dispatch_goal_stamp_sec,
        'require_action_roundtrip': bool(require_action_roundtrip),
        'action_roundtrip_observed': action_roundtrip_observed,
        'action_roundtrip_grace_active': action_roundtrip_grace_active,
        'action_roundtrip_live': action_roundtrip_live,
        'action_runtime_topics_visible': all(topic_visible(topic, published_topics) for topic in action_runtime_topics(action_name)) if action_name else False,
        'action_status_topic_type_ok': topic_type_matches(action_status_topic, published_topics, config.get('action_status_topic_type', ACTIONLIB_STATUS_TOPIC_TYPE)) if action_status_topic else False,
        'action_feedback_topic_type_ok': topic_type_matches(feedback_topic, published_topics, config.get('action_feedback_topic_type', ACTIONLIB_FEEDBACK_TOPIC_TYPE)) if feedback_topic else False,
        'action_result_topic_type_ok': topic_type_matches(result_topic, published_topics, config.get('action_result_topic_type', ACTIONLIB_RESULT_TOPIC_TYPE)) if result_topic else False,
        'goal_topic_visible': topic_visible(goal_topic, published_topics) if goal_topic else False,
        'goal_topic_type_ok': topic_type_matches(goal_topic, published_topics, config.get('simple_goal_topic_type', TOPIC_GOAL_TYPE)) if goal_topic else False,
        'status_topic_visible': topic_visible(status_topic, published_topics) if status_topic else False,
        'status_topic_type_ok': topic_type_matches(status_topic, published_topics, config.get('navigation_status_topic_type', ACTIONLIB_STATUS_TOPIC_TYPE)) if status_topic else False,
        'cancel_topic_visible': topic_visible(cancel_topic, published_topics) if cancel_topic else False,
        'cancel_topic_type_ok': topic_type_matches(cancel_topic, published_topics, config.get('navigation_cancel_topic_type', TOPIC_CANCEL_TYPE)) if cancel_topic else False,
        'odom_topic_visible': topic_visible(odom_topic, published_topics) if odom_topic else False,
        'odom_topic_type_ok': topic_type_matches(odom_topic, published_topics, config.get('odom_topic_type', ODOM_TOPIC_TYPE)) if odom_topic else False,
        'amcl_pose_topic_visible': topic_visible(amcl_pose_topic, published_topics) if amcl_pose_topic else False,
        'amcl_pose_topic_type_ok': topic_type_matches(amcl_pose_topic, published_topics, config.get('amcl_pose_topic_type', AMCL_POSE_TOPIC_TYPE)) if amcl_pose_topic else False,
    }
    graph_checks = []
    action_graph_ready = bool(probe['action_runtime_topics_visible']) and bool(probe['action_status_topic_type_ok'])
    action_runtime_ready = bool(probe['action_server_ready']) and action_graph_ready and bool(probe['action_feedback_topic_type_ok']) and bool(probe['action_result_topic_type_ok'])
    action_runtime_live = action_runtime_ready and bool(probe['runtime_live']) and bool(probe['action_roundtrip_live'])
    graph_checks.append(action_graph_ready if goal_transport == 'actionlib' else bool(probe['goal_topic_visible']) and bool(probe['goal_topic_type_ok']))
    if status_transport == 'actionlib':
        graph_checks.append(action_graph_ready)
    elif status_transport == 'topic':
        graph_checks.append(bool(probe['status_topic_visible']) and bool(probe['status_topic_type_ok']))
    if cancel_transport == 'actionlib':
        graph_checks.append(action_graph_ready)
    elif cancel_transport == 'topic':
        graph_checks.append(bool(probe['cancel_topic_visible']) and bool(probe['cancel_topic_type_ok']))
    if localization_backend == 'odometry_topic':
        graph_checks.append(bool(probe['odom_topic_visible']) and bool(probe['odom_topic_type_ok']))
    elif localization_backend == 'amcl_pose_topic':
        graph_checks.append(bool(probe['amcl_pose_topic_visible']) and bool(probe['amcl_pose_topic_type_ok']))
    graph_satisfied = all(graph_checks) if graph_checks else True
    semantic_checks = []
    semantic_checks.append(action_runtime_live if goal_transport == 'actionlib' else bool(probe['goal_topic_visible']) and bool(probe['goal_topic_type_ok']))
    if status_transport == 'actionlib':
        semantic_checks.append(action_runtime_live)
    elif status_transport == 'topic':
        semantic_checks.append(bool(probe['status_topic_visible']) and bool(probe['status_topic_type_ok']))
    if cancel_transport == 'actionlib':
        semantic_checks.append(action_runtime_live)
    elif cancel_transport == 'topic':
        semantic_checks.append(bool(probe['cancel_topic_visible']) and bool(probe['cancel_topic_type_ok']))
    if localization_backend == 'odometry_topic':
        semantic_checks.append(bool(probe['odom_topic_visible']) and bool(probe['odom_topic_type_ok']))
    elif localization_backend == 'amcl_pose_topic':
        semantic_checks.append(bool(probe['amcl_pose_topic_visible']) and bool(probe['amcl_pose_topic_type_ok']))
    semantic_satisfied = all(semantic_checks) if semantic_checks else True
    probe['graph_satisfied'] = graph_satisfied
    probe['semantic_satisfied'] = semantic_satisfied
    probe['mission_ready'] = semantic_satisfied
    probe['satisfied'] = semantic_satisfied
    return probe


def evaluate_platform_runtime_probe(config: Mapping[str, object], *, published_topics: Iterable[object]) -> dict:
    """Evaluate vendor-platform observability for the bridge runtime.

    Boundary behavior:
        The probe intentionally does *not* require an upstream ``cmd_vel``
        publisher because many stacks only publish commands while a navigation
        goal is active. Instead, startup readiness focuses on whether feedback
        and status surfaces are actually observable with the expected ROS types.
    """
    upstream_feedback_topic = str(config.get('upstream_feedback_topic', '')).strip()
    upstream_odom_topic = str(config.get('upstream_odom_topic', '')).strip()
    upstream_navigation_status_topic = str(config.get('upstream_navigation_status_topic', '')).strip()
    upstream_command_topic = str(config.get('upstream_command_topic', '')).strip()
    probe = {
        'explicit_feedback_topic_visible': topic_visible(upstream_feedback_topic, published_topics) if upstream_feedback_topic else False,
        'explicit_feedback_topic_type_ok': topic_type_matches(upstream_feedback_topic, published_topics, config.get('upstream_feedback_topic_type', FEEDBACK_TOPIC_TYPE)) if upstream_feedback_topic else False,
        'odom_topic_visible': topic_visible(upstream_odom_topic, published_topics) if upstream_odom_topic else False,
        'odom_topic_type_ok': topic_type_matches(upstream_odom_topic, published_topics, config.get('upstream_odom_topic_type', ODOM_TOPIC_TYPE)) if upstream_odom_topic else False,
        'navigation_status_topic_visible': topic_visible(upstream_navigation_status_topic, published_topics) if upstream_navigation_status_topic else False,
        'navigation_status_topic_type_ok': topic_type_matches(upstream_navigation_status_topic, published_topics, config.get('upstream_navigation_status_topic_type', ACTIONLIB_STATUS_TOPIC_TYPE)) if upstream_navigation_status_topic else False,
        'command_topic_visible': topic_visible(upstream_command_topic, published_topics) if upstream_command_topic else False,
        'command_topic_type_ok': topic_type_matches(upstream_command_topic, published_topics, config.get('upstream_command_topic_type', COMMAND_TOPIC_TYPE)) if upstream_command_topic else False,
    }
    feedback_observable = (bool(probe['explicit_feedback_topic_visible']) and bool(probe['explicit_feedback_topic_type_ok'])) or (bool(probe['odom_topic_visible']) and bool(probe['odom_topic_type_ok']))
    status_observable = True if not upstream_navigation_status_topic else (bool(probe['navigation_status_topic_visible']) and bool(probe['navigation_status_topic_type_ok']))
    command_declared = bool(upstream_command_topic)
    graph_satisfied = feedback_observable and status_observable and command_declared
    probe['feedback_observable'] = feedback_observable
    probe['status_observable'] = status_observable
    probe['command_declared'] = command_declared
    probe['graph_satisfied'] = graph_satisfied
    probe['semantic_satisfied'] = feedback_observable and status_observable
    probe['mission_ready'] = probe['semantic_satisfied']
    probe['satisfied'] = probe['semantic_satisfied']
    return probe

"""Explicit navigation-contract validation helpers.

These helpers make the planner/controller/recovery/localization split a runtime
contract rather than a health-only description. They do not instantiate the
navigation backend; instead they validate that mission configuration, bound
resources and declared capability metadata are mutually consistent.
"""

from __future__ import annotations

from typing import Dict, Mapping

from .domain_models import ConfigurationError
from .navigation_adapters.registry import describe_navigation_capability

VALID_LOCALIZATION_BACKENDS = ('odometry_topic', 'amcl_pose_topic', 'tf_lookup_pose')
VALID_GOAL_TRANSPORTS = ('actionlib', 'topic')
VALID_STATUS_TRANSPORTS = ('actionlib', 'topic', 'internal_pose_evaluator')
VALID_CANCEL_TRANSPORTS = ('actionlib', 'topic', 'none')
VALID_BACKEND_PROFILES = ('move_base_managed', 'topic_pose_evaluator', 'topic_status_bridge')


def _normalize(value: object) -> str:
    return str(value or '').strip().lower()


def _expected_localization_backend(pose_source_type: object) -> str:
    normalized = _normalize(pose_source_type)
    mapping = {
        'odometry': 'odometry_topic',
        'amcl_pose': 'amcl_pose_topic',
        'tf_lookup': 'tf_lookup_pose',
    }
    if normalized not in mapping:
        raise ConfigurationError('Unsupported pose_source_type for navigation contract: {}'.format(pose_source_type))
    return mapping[normalized]



def apply_navigation_contract_defaults(config: dict) -> dict:
    """Populate explicit navigation contract fields from adapter capability.

    Empty fields are treated as unspecified and are backfilled from the adapter
    capability plus the configured pose-source type. Explicit values are kept so
    mismatches remain detectable by subsequent validation.
    """
    capability = describe_navigation_capability(config)
    if config.get('navigation_planner_backend') in (None, ''):
        config['navigation_planner_backend'] = capability['planner_interface']
    if config.get('navigation_controller_backend') in (None, ''):
        config['navigation_controller_backend'] = capability['controller_interface']
    if config.get('navigation_recovery_backend') in (None, ''):
        config['navigation_recovery_backend'] = capability['recovery_interface']
    if config.get('navigation_localization_backend') in (None, ''):
        config['navigation_localization_backend'] = _expected_localization_backend(config.get('pose_source_type', ''))
    if config.get('navigation_goal_transport') in (None, ''):
        config['navigation_goal_transport'] = capability['goal_transport']
    if config.get('navigation_status_transport') in (None, ''):
        config['navigation_status_transport'] = capability['status_transport']
    if config.get('navigation_cancel_transport') in (None, ''):
        config['navigation_cancel_transport'] = capability['cancel_transport']
    if config.get('navigation_backend_profile') in (None, ''):
        config['navigation_backend_profile'] = capability['backend_profile']
    config['navigation_capability_contract'] = capability
    return capability



def validate_navigation_runtime_strategy(config: Mapping[str, object], *, owner: str) -> Dict[str, object]:
    """Validate logical consistency of the navigation contract.

    This is intentionally stricter than a descriptive health summary: planner,
    controller, recovery, localization and transport declarations must match the
    resolved adapter capability, otherwise runtime startup fails.
    """
    capability = describe_navigation_capability(config)
    planner_backend = _normalize(config.get('navigation_planner_backend', ''))
    controller_backend = _normalize(config.get('navigation_controller_backend', ''))
    recovery_backend = _normalize(config.get('navigation_recovery_backend', ''))
    localization_backend = _normalize(config.get('navigation_localization_backend', ''))
    goal_transport = _normalize(config.get('navigation_goal_transport', ''))
    status_transport = _normalize(config.get('navigation_status_transport', ''))
    cancel_transport = _normalize(config.get('navigation_cancel_transport', ''))
    backend_profile = _normalize(config.get('navigation_backend_profile', ''))
    expected_localization = _expected_localization_backend(config.get('pose_source_type', ''))

    for label, value, valid in [
        ('navigation_localization_backend', localization_backend, VALID_LOCALIZATION_BACKENDS),
        ('navigation_goal_transport', goal_transport, VALID_GOAL_TRANSPORTS),
        ('navigation_status_transport', status_transport, VALID_STATUS_TRANSPORTS),
        ('navigation_cancel_transport', cancel_transport, VALID_CANCEL_TRANSPORTS),
        ('navigation_backend_profile', backend_profile, VALID_BACKEND_PROFILES),
    ]:
        if value not in valid:
            raise ConfigurationError('{} {} must be one of {}'.format(owner, label, ', '.join(valid)))

    expectations = {
        'navigation_planner_backend': _normalize(capability['planner_interface']),
        'navigation_controller_backend': _normalize(capability['controller_interface']),
        'navigation_recovery_backend': _normalize(capability['recovery_interface']),
        'navigation_localization_backend': expected_localization,
        'navigation_goal_transport': _normalize(capability['goal_transport']),
        'navigation_status_transport': _normalize(capability['status_transport']),
        'navigation_cancel_transport': _normalize(capability['cancel_transport']),
        'navigation_backend_profile': _normalize(capability['backend_profile']),
    }
    actuals = {
        'navigation_planner_backend': planner_backend,
        'navigation_controller_backend': controller_backend,
        'navigation_recovery_backend': recovery_backend,
        'navigation_localization_backend': localization_backend,
        'navigation_goal_transport': goal_transport,
        'navigation_status_transport': status_transport,
        'navigation_cancel_transport': cancel_transport,
        'navigation_backend_profile': backend_profile,
    }
    for key, expected in expectations.items():
        actual = actuals[key]
        if actual != expected:
            raise ConfigurationError('{} {}={} must match resolved navigation contract {}'.format(owner, key, actual, expected))

    return {
        'adapter_type': capability['name'],
        'planner_backend': planner_backend,
        'controller_backend': controller_backend,
        'recovery_backend': recovery_backend,
        'localization_backend': localization_backend,
        'goal_transport': goal_transport,
        'status_transport': status_transport,
        'cancel_transport': cancel_transport,
        'backend_profile': backend_profile,
        'status_source': capability['status_source'],
        'supports_cancel': bool(capability['supports_cancel']),
        'requires_external_status': bool(capability['requires_external_status']),
    }



def validate_navigation_contract_bindings(config: Mapping[str, object], *, owner: str) -> Dict[str, object]:
    """Validate concrete topics/actions required by the resolved navigation contract."""
    capability = describe_navigation_capability(config)
    goal_transport = _normalize(config.get('navigation_goal_transport', capability['goal_transport']))
    status_transport = _normalize(config.get('navigation_status_transport', capability['status_transport']))
    cancel_transport = _normalize(config.get('navigation_cancel_transport', capability['cancel_transport']))
    localization_backend = _normalize(config.get('navigation_localization_backend', ''))

    bound_topics = []
    bound_actions = []
    missing_topics = []
    missing_actions = []

    if goal_transport == 'actionlib':
        action_name = str(config.get('move_base_action_name', '')).strip().strip('/')
        if not action_name:
            missing_actions.append('move_base_action_name')
        else:
            bound_actions.append(action_name)
    elif goal_transport == 'topic':
        goal_topic = str(config.get('simple_goal_topic', '')).strip().strip('/')
        if not goal_topic:
            missing_topics.append('simple_goal_topic')
        else:
            bound_topics.append(goal_topic)

    if status_transport == 'topic':
        status_topic = str(config.get('navigation_status_topic', '')).strip().strip('/')
        if not status_topic:
            missing_topics.append('navigation_status_topic')
        else:
            bound_topics.append(status_topic)
    elif status_transport == 'actionlib' and goal_transport == 'actionlib':
        action_name = str(config.get('move_base_action_name', '')).strip().strip('/')
        if action_name:
            bound_actions.append(action_name)

    if cancel_transport == 'topic':
        cancel_topic = str(config.get('navigation_cancel_topic', '')).strip().strip('/')
        if not cancel_topic:
            missing_topics.append('navigation_cancel_topic')
        else:
            bound_topics.append(cancel_topic)
    elif cancel_transport == 'actionlib' and goal_transport == 'actionlib':
        action_name = str(config.get('move_base_action_name', '')).strip().strip('/')
        if action_name:
            bound_actions.append(action_name)

    localization_topic_key = {
        'odometry_topic': 'odom_topic',
        'amcl_pose_topic': 'amcl_pose_topic',
    }
    if localization_backend in localization_topic_key:
        key = localization_topic_key[localization_backend]
        topic = str(config.get(key, '')).strip().strip('/')
        if not topic:
            missing_topics.append(key)
        else:
            bound_topics.append(topic)

    if missing_topics or missing_actions:
        problems = []
        if missing_topics:
            problems.append('topics={}'.format(missing_topics))
        if missing_actions:
            problems.append('actions={}'.format(missing_actions))
        raise ConfigurationError('{} does not satisfy navigation contract {}: {}'.format(owner, capability['name'], ', '.join(problems)))

    return {
        'adapter_type': capability['name'],
        'bound_topics': bound_topics,
        'bound_actions': bound_actions,
        'goal_transport': goal_transport,
        'status_transport': status_transport,
        'cancel_transport': cancel_transport,
        'localization_backend': localization_backend,
    }

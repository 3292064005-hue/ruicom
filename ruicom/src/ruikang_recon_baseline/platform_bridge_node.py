#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS bridge that normalizes vendor platform topics into baseline task-layer contracts."""

from __future__ import annotations

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, require_positive_float
from .lifecycle_protocol import decode_lifecycle_control, encode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import HealthState
from .platform_adapters import (
    PlatformAdapterRegistry,
    apply_platform_safety_defaults,
    validate_platform_contract_bindings,
    validate_platform_runtime_strategy,
)
from .platform_contracts import validate_command_topic_flow_contract
from .platform_bridge_core import PlatformBridgeSnapshot, evaluate_platform_bridge
from .vendor_bundle_preflight import build_vendor_bundle_preflight_report, enforce_vendor_bundle_preflight
from .vendor_runtime_contracts import (
    build_vendor_runtime_binding_report,
    load_vendor_runtime_contract,
    validate_vendor_runtime_contract,
)


class PlatformBridgeNode:
    """Bridge vendor signals into normalized contract topics.

    Responsibilities:
    1. normalize downstream execution feedback and mode/estop/status signals;
    2. relay the upstream navigation/controller velocity output into the single
       safety input bus (``cmd_vel_raw`` by default) so deploy profiles cannot
       accidentally bypass the safety mux;
    3. surface lifecycle-aware health so the system manager can supervise the
       bridge as a first-class managed component.
    """

    def __init__(self):
        rospy.init_node('platform_bridge_node')
        self.config = self._read_config()
        self._capability = PlatformAdapterRegistry().resolve(self.config['platform_adapter_type'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self._last_explicit_feedback_sec = 0.0
        self._last_odom_sec = 0.0
        self._explicit_feedback_value = False
        self._last_mode = str(self.config['default_control_mode']).strip() or 'UNKNOWN'
        self._last_estop = bool(self.config['default_estop'])
        self._last_nav_status_json = self.config['default_navigation_status_json']
        self._first_command_seen_sec = 0.0
        self._last_command_sec = 0.0
        self._last_forwarded_command_sec = 0.0
        self._command_observed_count = 0
        self._command_forward_count = 0
        self._command_drop_count = 0
        self._last_health_publish_sec = 0.0

        self.feedback_pub = rospy.Publisher(self.config['output_feedback_topic'], Bool, queue_size=10)
        self.summary_pub = rospy.Publisher(self.config['platform_bridge_state_topic'], String, queue_size=10)
        self.mode_pub = rospy.Publisher(self.config['control_mode_topic'], String, queue_size=10, latch=True)
        self.estop_pub = rospy.Publisher(self.config['estop_topic'], Bool, queue_size=10, latch=True)
        self.command_pub = rospy.Publisher(self.config['command_input_topic'], Twist, queue_size=10)
        self.nav_status_pub = rospy.Publisher(self.config['navigation_status_topic'], String, queue_size=10, latch=True) if self.config['navigation_status_topic'] else None
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)

        self.feedback_sub = rospy.Subscriber(self.config['upstream_feedback_topic'], Bool, self._feedback_cb, queue_size=10) if self.config['upstream_feedback_topic'] else None
        self.odom_sub = rospy.Subscriber(self.config['upstream_odom_topic'], Odometry, self._odom_cb, queue_size=10) if self.config['upstream_odom_topic'] else None
        self.mode_sub = rospy.Subscriber(self.config['upstream_control_mode_topic'], String, self._mode_cb, queue_size=10) if self.config['upstream_control_mode_topic'] else None
        self.estop_sub = rospy.Subscriber(self.config['upstream_estop_topic'], Bool, self._estop_cb, queue_size=10) if self.config['upstream_estop_topic'] else None
        self.nav_status_sub = rospy.Subscriber(self.config['upstream_navigation_status_topic'], GoalStatusArray, self._nav_status_cb, queue_size=10) if self.config['upstream_navigation_status_topic'] else None
        self.command_sub = rospy.Subscriber(self.config['upstream_command_topic'], Twist, self._command_cb, queue_size=10) if self.config['upstream_command_topic'] else None
        self.control_command_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_command_cb, queue_size=20) if self.config['lifecycle_managed'] else None

        self.mode_pub.publish(String(data=self._last_mode))
        self.estop_pub.publish(Bool(data=self._last_estop))
        if self.nav_status_pub is not None and self._last_nav_status_json:
            self.nav_status_pub.publish(String(data=self._last_nav_status_json))
        self._publish_health('ok', 'node_ready', self._summary_payload(self._current_decision()))
        rospy.loginfo(
            'platform_bridge_node started. adapter=%s upstream_command=%s -> %s feedback_output=%s lifecycle_managed=%s',
            self.config['platform_adapter_type'],
            self.config['upstream_command_topic'],
            self.config['command_input_topic'],
            self.config['output_feedback_topic'],
            self.config['lifecycle_managed'],
        )

    def _read_config(self) -> dict:
        """Read and validate bridge configuration.

        Returns:
            dict: Normalized bridge configuration.

        Raises:
            ConfigurationError: When required bindings are missing or numeric
                constraints are invalid.

        Boundary behavior:
            Empty upstream feedback topics are allowed when odom heartbeat is
            configured, because some vendor stacks only expose odometry rather
            than an explicit execution heartbeat.
        """
        config = {
            'platform_adapter_type': str(rospy.get_param('~platform_adapter_type', 'generic_ros_nav')).strip() or 'generic_ros_nav',
            'profile_role': str(rospy.get_param('~profile_role', 'deploy')).strip().lower() or 'deploy',
            'output_feedback_topic': str(rospy.get_param('~output_feedback_topic', '')).strip(),
            'output_feedback_timeout_sec': float(rospy.get_param('~output_feedback_timeout_sec', 0.6)),
            'require_output_feedback': bool(rospy.get_param('~require_output_feedback', False)),
            'upstream_feedback_topic': str(rospy.get_param('~upstream_feedback_topic', '')).strip(),
            'upstream_odom_topic': str(rospy.get_param('~upstream_odom_topic', '')).strip(),
            'upstream_control_mode_topic': str(rospy.get_param('~upstream_control_mode_topic', '')).strip(),
            'upstream_estop_topic': str(rospy.get_param('~upstream_estop_topic', '')).strip(),
            'upstream_navigation_status_topic': str(rospy.get_param('~upstream_navigation_status_topic', '')).strip(),
            'upstream_command_topic': str(rospy.get_param('~upstream_command_topic', 'cmd_vel')).strip(),
            'command_input_topic': str(rospy.get_param('~command_input_topic', 'cmd_vel_raw')).strip() or 'cmd_vel_raw',
            'safety_output_topic': str(rospy.get_param('~safety_output_topic', '')).strip(),
            'control_mode_topic': str(rospy.get_param('~control_mode_topic', 'recon/control_mode')).strip() or 'recon/control_mode',
            'estop_topic': str(rospy.get_param('~estop_topic', 'recon/estop')).strip() or 'recon/estop',
            'navigation_status_topic': str(rospy.get_param('~navigation_status_topic', 'recon/navigation_status')).strip() or 'recon/navigation_status',
            'default_control_mode': str(rospy.get_param('~default_control_mode', 'COMMANDER')).strip() or 'COMMANDER',
            'default_estop': bool(rospy.get_param('~default_estop', False)),
            'default_navigation_status_json': str(rospy.get_param('~default_navigation_status_json', '')).strip(),
            'platform_bridge_state_topic': str(rospy.get_param('~platform_bridge_state_topic', 'recon/platform/bridge_state')).strip() or 'recon/platform/bridge_state',
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 20.0)),
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'vendor_runtime_mode': str(rospy.get_param('~vendor_runtime_mode', '')).strip(),
            'vendor_workspace_ros_distro': str(rospy.get_param('~vendor_workspace_ros_distro', '')).strip(),
            'vendor_workspace_python_major': int(rospy.get_param('~vendor_workspace_python_major', 0)),
            'vendor_runtime_contract_path': str(rospy.get_param('~vendor_runtime_contract_path', '')).strip(),
            'vendor_bundle_preflight_mode': str(rospy.get_param('~vendor_bundle_preflight_mode', 'off')).strip().lower() or 'off',
            'vendor_workspace_root': str(rospy.get_param('~vendor_workspace_root', '')).strip(),
            'vendor_workspace_root_env': str(rospy.get_param('~vendor_workspace_root_env', 'MOWEN_VENDOR_WORKSPACE_ROOT')).strip() or 'MOWEN_VENDOR_WORKSPACE_ROOT',
            'motion_model': str(rospy.get_param('~motion_model', '')).strip(),
            'cmd_vel_semantics': str(rospy.get_param('~cmd_vel_semantics', '')).strip(),
            'allow_odom_feedback_fallback': bool(rospy.get_param('~allow_odom_feedback_fallback', True)),
            'upstream_feedback_topic_type': str(rospy.get_param('~upstream_feedback_topic_type', 'std_msgs/Bool')).strip(),
            'upstream_odom_topic_type': str(rospy.get_param('~upstream_odom_topic_type', 'nav_msgs/Odometry')).strip(),
            'upstream_navigation_status_topic_type': str(rospy.get_param('~upstream_navigation_status_topic_type', 'actionlib_msgs/GoalStatusArray')).strip(),
            'upstream_command_topic_type': str(rospy.get_param('~upstream_command_topic_type', 'geometry_msgs/Twist')).strip(),
        }
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['output_feedback_timeout_sec'] = require_positive_float('output_feedback_timeout_sec', config['output_feedback_timeout_sec'])
        capability = apply_platform_safety_defaults(config)
        if not str(config.get('safety_output_topic', '')).strip():
            config['safety_output_topic'] = str(capability.safety_defaults.get('output_topic', 'cmd_vel')).strip() or 'cmd_vel'
        config['command_flow_contract'] = validate_command_topic_flow_contract(config, owner='platform_bridge_node')
        config['require_command_traffic_for_readiness'] = bool(config['profile_role'] == 'deploy')
        config['platform_contract'] = capability.summary()
        config['platform_runtime_contract'] = validate_platform_runtime_strategy(
            capability,
            config,
            owner='platform_bridge_node',
        )
        vendor_runtime_contract = load_vendor_runtime_contract(config.get('vendor_runtime_contract_path', ''))
        if vendor_runtime_contract:
            config['vendor_runtime_contract_path'] = str(vendor_runtime_contract.get('path', '')).strip()
        config['vendor_runtime_contract'] = validate_vendor_runtime_contract(
            vendor_runtime_contract,
            config,
            owner='platform_bridge_node',
            domain='bridge',
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
        enforce_vendor_bundle_preflight(config['vendor_bundle_preflight'], owner='platform_bridge_node')
        for param_name, binding in dict(config['vendor_runtime_contract'].get('required_bindings', {})).items():
            expected_type = str(binding.get('expected_type', '')).strip()
            if param_name == 'upstream_feedback_topic' and expected_type and not str(config.get('upstream_feedback_topic_type', '')).strip():
                config['upstream_feedback_topic_type'] = expected_type
            elif param_name == 'upstream_odom_topic' and expected_type and not str(config.get('upstream_odom_topic_type', '')).strip():
                config['upstream_odom_topic_type'] = expected_type
            elif param_name == 'upstream_navigation_status_topic' and expected_type and not str(config.get('upstream_navigation_status_topic_type', '')).strip():
                config['upstream_navigation_status_topic_type'] = expected_type
            elif param_name == 'upstream_command_topic' and expected_type and not str(config.get('upstream_command_topic_type', '')).strip():
                config['upstream_command_topic_type'] = expected_type
        if not config['upstream_odom_topic']:
            config['upstream_odom_topic'] = str(capability.mission_defaults.get('odom_topic', '')).strip()
        if not config['output_feedback_topic']:
            raise ConfigurationError('platform_bridge_node requires a non-empty output_feedback_topic')
        if not config['upstream_feedback_topic'] and not config['upstream_odom_topic']:
            raise ConfigurationError('platform_bridge_node requires upstream_feedback_topic or upstream_odom_topic')
        if not config['upstream_command_topic']:
            raise ConfigurationError('platform_bridge_node requires upstream_command_topic')
        config['platform_contract_bindings'] = validate_platform_contract_bindings(capability, config, owner='platform_bridge_node', domain='bridge')
        return config

    def _feedback_cb(self, msg: Bool) -> None:
        self._explicit_feedback_value = bool(msg.data)
        self._last_explicit_feedback_sec = rospy.get_time()

    def _odom_cb(self, _msg: Odometry) -> None:
        self._last_odom_sec = rospy.get_time()

    def _mode_cb(self, msg: String) -> None:
        self._last_mode = str(msg.data).strip() or self._last_mode
        self.mode_pub.publish(String(data=self._last_mode))

    def _estop_cb(self, msg: Bool) -> None:
        self._last_estop = bool(msg.data)
        self.estop_pub.publish(Bool(data=self._last_estop))

    def _nav_status_cb(self, msg: GoalStatusArray) -> None:
        payload = {
            'header': {
                'seq': int(msg.header.seq),
                'stamp': float(msg.header.stamp.to_sec()),
                'frame_id': str(msg.header.frame_id),
            },
            'status_list': [
                {
                    'goal_id': {
                        'stamp': float(item.goal_id.stamp.to_sec()),
                        'id': str(item.goal_id.id),
                    },
                    'status': int(item.status),
                    'text': str(item.text),
                }
                for item in msg.status_list
            ],
        }
        self._last_nav_status_json = JsonCodec.dumps(payload)
        if self.nav_status_pub is not None:
            self.nav_status_pub.publish(String(data=self._last_nav_status_json))

    def _control_command_cb(self, msg: String) -> None:
        """Apply lifecycle commands coming from the system manager.

        Args:
            msg: ROS string message containing activate/pause/reset/shutdown.

        Returns:
            None. Runtime state is updated in place and reflected through health.

        Raises:
            No explicit exception is raised. Invalid commands are reported via
            health diagnostics instead of crashing the bridge.

        Boundary behavior:
            Repeated activate/pause commands are accepted as harmless no-ops so
            late or duplicate supervisor broadcasts do not destabilize runtime.
        """
        envelope = decode_lifecycle_control(msg.data)
        command = envelope.command
        if not command or not envelope.matches(rospy.get_name().split('/')[-1]):
            return
        result = self.runtime.apply(command)
        level = 'ok' if result.accepted else 'warn'
        details = {
            'command': command,
            'target': envelope.target,
            'issued_by': envelope.issued_by,
            'metadata': dict(envelope.metadata or {}),
            'accepted': bool(result.accepted),
            'runtime_state': result.state,
            'changed': bool(result.changed),
        }
        self._publish_health(level, result.message, details)

    def _command_cb(self, msg: Twist) -> None:
        """Observe and optionally forward one upstream velocity command.

        Args:
            msg: Incoming ROS ``Twist`` from the upstream navigation/controller
                stack.

        Returns:
            None. Internal observability counters and timestamps are updated in
            place; the message is forwarded to ``command_input_topic`` only when
            lifecycle processing is allowed.

        Raises:
            No explicit exception is raised. Runtime gating converts inactive
            lifecycle states into counted drops rather than transport failures.

        Boundary behavior:
            Traffic observability is recorded even while lifecycle processing is
            paused or idle so the supervisor can prove the upstream command path
            exists before activating the mission stack.
        """
        now_sec = rospy.get_time()
        self._last_command_sec = now_sec
        if self._first_command_seen_sec <= 0.0:
            self._first_command_seen_sec = now_sec
        self._command_observed_count += 1
        if not self.runtime.processing_allowed:
            self._command_drop_count += 1
            return
        self._command_forward_count += 1
        self._last_forwarded_command_sec = now_sec
        self.command_pub.publish(msg)

    def _current_decision(self):
        return evaluate_platform_bridge(
            PlatformBridgeSnapshot(
                now_sec=rospy.get_time(),
                feedback_timeout_sec=self.config['output_feedback_timeout_sec'],
                explicit_feedback_enabled=bool(self.config['upstream_feedback_topic']),
                explicit_feedback_value=self._explicit_feedback_value,
                explicit_feedback_stamp_sec=self._last_explicit_feedback_sec,
                odom_feedback_enabled=bool(self.config['upstream_odom_topic']),
                odom_stamp_sec=self._last_odom_sec,
                allow_odom_as_feedback=bool(self.config['allow_odom_feedback_fallback']),
                command_bridge_enabled=bool(self.config['upstream_command_topic']),
                upstream_command_stamp_sec=self._last_command_sec,
            )
        )

    def _platform_runtime_probe(self) -> dict:
        """Probe vendor-platform observability on the current ROS graph.

        Returns:
            Dictionary with per-resource visibility and aggregate readiness.

        Raises:
            No explicit exception is raised. ROS graph lookup failures are
            reported as structured probe errors.
        """
        try:
            published_topics = rospy.get_published_topics()
        except Exception as exc:
            return {
                'satisfied': False,
                'probe_error': str(exc),
            }
        return evaluate_platform_runtime_probe(self.config, published_topics=published_topics)

    def _summary_payload(self, decision) -> dict:
        runtime_probe = self._platform_runtime_probe()
        summary = {
            'adapter_type': self.config['platform_adapter_type'],
            'source': decision.source,
            'output_feedback': bool(decision.output_feedback),
            'execution_feedback_fresh': bool(decision.output_feedback),
            'explicit_feedback_topic': self.config['upstream_feedback_topic'],
            'upstream_odom_topic': self.config['upstream_odom_topic'],
            'upstream_command_topic': self.config['upstream_command_topic'],
            'command_input_topic': self.config['command_input_topic'],
            'upstream_estop_topic': self.config['upstream_estop_topic'],
            'upstream_navigation_status_topic': self.config['upstream_navigation_status_topic'],
            'output_feedback_topic': self.config['output_feedback_topic'],
            'explicit_feedback_fresh': bool(decision.explicit_feedback_fresh),
            'odom_feedback_fresh': bool(decision.odom_feedback_fresh),
            'odom_observability_fresh': bool(decision.odom_feedback_fresh),
            'allow_odom_feedback_fallback': bool(self.config['allow_odom_feedback_fallback']),
            'command_bridge_fresh': bool(decision.command_bridge_fresh),
            'command_observed_count': int(self._command_observed_count),
            'command_forward_count': int(self._command_forward_count),
            'command_drop_count': int(self._command_drop_count),
            'first_command_seen_at': float(self._first_command_seen_sec),
            'last_command_seen_at': float(self._last_command_sec),
            'last_forwarded_command_at': float(self._last_forwarded_command_sec),
            'command_traffic_seen': bool(self._command_observed_count > 0),
            'command_forwarding_active': bool(self._command_forward_count > 0),
            'require_command_traffic_for_readiness': bool(self.config['require_command_traffic_for_readiness']),
            'command_path_bound': bool(self.config['upstream_command_topic']) and bool(self.config['command_input_topic']),
            'safety_output_topic': self.config['safety_output_topic'],
            'command_flow_contract': dict(self.config['command_flow_contract']),
            'command_flow_contract_satisfied': bool(self.config['command_flow_contract'].get('satisfied', True)),
            'control_mode': self._last_mode,
            'estop_active': self._last_estop,
            'navigation_status_topic': self.config['navigation_status_topic'],
            'feedback_contract_satisfied': bool(decision.output_feedback) or not self.config['require_output_feedback'],
            'vendor_runtime_mode': self.config['vendor_runtime_mode'],
            'vendor_workspace_ros_distro': self.config['vendor_workspace_ros_distro'],
            'vendor_workspace_python_major': int(self.config['vendor_workspace_python_major']),
            'motion_model': self.config['motion_model'],
            'cmd_vel_semantics': self.config['cmd_vel_semantics'],
            'platform_runtime_contract': dict(self.config['platform_runtime_contract']),
            'platform_contract_bindings': dict(self.config['platform_contract_bindings']),
            'lifecycle_managed': bool(self.config['lifecycle_managed']),
            'runtime_state': self.runtime.state,
            'processing_allowed': bool(self.runtime.processing_allowed),
            'platform_contract_satisfied': True,
            'vendor_runtime_contract_satisfied': bool(self.config.get('vendor_runtime_contract_satisfied', True)),
            'vendor_bundle_preflight': dict(self.config.get('vendor_bundle_preflight', {})),
            'vendor_bundle_preflight_satisfied': bool(self.config.get('vendor_bundle_preflight_satisfied', True)),
            'vendor_runtime_binding_report': dict(self.config.get('vendor_runtime_binding_report', {})),
            'platform_runtime_probe': dict(runtime_probe),
            'platform_runtime_probe_satisfied': bool(runtime_probe.get('satisfied', False)),
        }
        return summary

    def _publish_health(self, status: str, message: str, details: dict | None = None) -> None:
        payload = {
            'stamp': rospy.get_time(),
            'node': 'platform_bridge_node',
            'status': str(status).strip(),
            'message': str(message).strip(),
            'schema_version': SCHEMA_VERSION,
            'details': dict(details or {}),
        }
        payload['details'].setdefault('lifecycle_managed', bool(self.config['lifecycle_managed']))
        payload['details'].setdefault('runtime_state', self.runtime.state)
        payload['details'].setdefault('processing_allowed', bool(self.runtime.processing_allowed))
        runtime_probe = self._platform_runtime_probe()
        payload['details'].setdefault('platform_runtime_probe', dict(runtime_probe))
        payload['details'].setdefault('platform_runtime_probe_satisfied', bool(runtime_probe.get('satisfied', False)))
        payload['details'].setdefault('vendor_bundle_preflight', dict(self.config.get('vendor_bundle_preflight', {})))
        payload['details'].setdefault('vendor_bundle_preflight_satisfied', bool(self.config.get('vendor_bundle_preflight_satisfied', True)))
        payload['details'].setdefault('command_flow_contract_satisfied', bool(self.config['command_flow_contract'].get('satisfied', True)))
        payload['details'].setdefault('vendor_runtime_binding_report', dict(self.config.get('vendor_runtime_binding_report', {})))
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = rospy.Time.now()
        typed.header.frame_id = self.config['health_frame_id']
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(payload.get('details', {}))
        self.health_typed_pub.publish(typed)
        self._last_health_publish_sec = payload['stamp']

    def spin(self) -> None:
        rate = rospy.Rate(self.config['publish_rate_hz'])
        last_health_signature = None
        while not rospy.is_shutdown():
            decision = self._current_decision()
            summary = self._summary_payload(decision)
            self.feedback_pub.publish(Bool(data=decision.output_feedback))
            self.mode_pub.publish(String(data=self._last_mode))
            self.estop_pub.publish(Bool(data=self._last_estop))
            if self.nav_status_pub is not None and self._last_nav_status_json:
                self.nav_status_pub.publish(String(data=self._last_nav_status_json))
            self.summary_pub.publish(String(data=JsonCodec.dumps(summary)))

            health_status = 'ok'
            health_message = 'bridge_ready'
            if self.config['require_output_feedback'] and not summary['feedback_contract_satisfied']:
                health_status = 'warn'
                health_message = 'feedback_contract_unready'
            elif not summary['command_path_bound']:
                health_status = 'error'
                health_message = 'command_path_unbound'
            elif not summary['command_flow_contract_satisfied']:
                health_status = 'error'
                health_message = 'command_flow_contract_unready'
            elif summary['require_command_traffic_for_readiness'] and not summary['command_traffic_seen']:
                health_status = 'warn'
                health_message = 'command_traffic_unseen'
            health_signature = (
                health_status,
                health_message,
                summary['runtime_state'],
                summary['feedback_contract_satisfied'],
                summary['command_path_bound'],
                summary['command_flow_contract_satisfied'],
                summary['command_traffic_seen'],
                summary['command_forwarding_active'],
            )
            now_sec = rospy.get_time()
            if health_signature != last_health_signature or (now_sec - self._last_health_publish_sec) >= 1.0:
                self._publish_health(health_status, health_message, summary)
                last_health_signature = health_signature
            rate.sleep()


if __name__ == '__main__':
    PlatformBridgeNode().spin()

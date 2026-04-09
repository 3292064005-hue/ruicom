#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Safety controller node with arbitration, freshness checks and fail-safe stop."""

from __future__ import annotations

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, VelocityCommand, require_positive_float, validate_profile_runtime_flags
from .lifecycle_protocol import decode_lifecycle_control, encode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import HealthState
from .platform_adapters import (
    apply_platform_safety_defaults,
    validate_platform_contract_bindings,
    validate_platform_runtime_strategy,
)
from .safety_core import SafetyController, classify_safety_health, evaluate_output_feedback_policy
from .time_core import NodeClock


class CmdSafetyMuxNode:
    """Arbitrate velocity sources and publish a fail-safe output command."""

    def __init__(self):
        rospy.init_node('cmd_safety_mux_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        self.controller = self._build_controller()
        self.first_input_command_sec = 0.0
        self.last_input_command_sec = 0.0
        self.input_command_observed_count = 0
        self.input_command_drop_count = 0
        self.last_output_command_sec = 0.0
        self.output_command_emitted_count = 0
        self.output_pub = rospy.Publisher(self.config['output_topic'], Twist, queue_size=10)
        self.state_pub = rospy.Publisher(self.config['safety_state_topic'], String, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        rospy.Subscriber(self.config['estop_topic'], Bool, self._estop_cb, queue_size=20)
        rospy.Subscriber(self.config['control_mode_topic'], String, self._mode_cb, queue_size=20)
        if self.config['lifecycle_managed']:
            rospy.Subscriber(self.config['control_command_topic'], String, self._control_command_cb, queue_size=20)
        self._build_source_subscribers()
        self.last_output_feedback_sec = 0.0
        self.output_feedback_state = False
        if self.config['output_feedback_topic']:
            rospy.Subscriber(self.config['output_feedback_topic'], Bool, self._output_feedback_cb, queue_size=20)
        self.last_health_signature = None
        self.last_health_emit_mono = 0.0
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            'cmd_safety_mux_node started. output_topic=%s time_source=%s health_emit_mode=%s lifecycle_state=%s',
            self.config['output_topic'],
            self.config['time_source_mode'],
            self.config['health_emit_mode'],
            self.runtime.state,
        )

    def _build_controller(self) -> SafetyController:
        return SafetyController(
            max_linear_x=self.config['max_linear_x'],
            max_linear_y=self.config['max_linear_y'],
            max_angular_z=self.config['max_angular_z'],
            command_timeout_sec=self.config['command_timeout_sec'],
            estop_timeout_sec=self.config['estop_timeout_sec'],
            require_fresh_estop=self.config['require_fresh_estop'],
            default_mode=self.config['default_mode'],
        )

    def _read_config(self):
        sources = rospy.get_param('~input_sources', [])
        single_topic = rospy.get_param('~input_topic', 'cmd_vel_raw')
        if not sources:
            sources = [{'name': 'autonomy', 'topic': single_topic, 'priority': 100}]
        config = {
            'input_sources': sources,
            'platform_adapter_type': str(rospy.get_param('~platform_adapter_type', 'generic_ros_nav')).strip() or 'generic_ros_nav',
            'output_topic': rospy.get_param('~output_topic', 'cmd_vel'),
            'estop_topic': rospy.get_param('~estop_topic', 'recon/estop'),
            'control_mode_topic': rospy.get_param('~control_mode_topic', 'recon/control_mode'),
            'safety_state_topic': rospy.get_param('~safety_state_topic', 'recon/safety_state'),
            'health_topic': rospy.get_param('~health_topic', 'recon/health'),
            'health_typed_topic': rospy.get_param('~health_typed_topic', 'recon/health_typed'),
            'max_linear_x': float(rospy.get_param('~max_linear_x', 0.45)),
            'max_linear_y': float(rospy.get_param('~max_linear_y', 0.45)),
            'max_angular_z': float(rospy.get_param('~max_angular_z', 1.20)),
            'command_timeout_sec': float(rospy.get_param('~command_timeout_sec', 0.5)),
            'estop_timeout_sec': float(rospy.get_param('~estop_timeout_sec', 2.0)),
            'require_fresh_estop': bool(rospy.get_param('~require_fresh_estop', False)),
            'default_mode': rospy.get_param('~default_mode', 'AUTO'),
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 20.0)),
            'time_source_mode': rospy.get_param('~time_source_mode', 'ros'),
            'health_emit_mode': rospy.get_param('~health_emit_mode', 'edge'),
            'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz', 1.0)),
            'warn_on_idle_command_stale': bool(rospy.get_param('~warn_on_idle_command_stale', False)),
            'output_feedback_topic': str(rospy.get_param('~output_feedback_topic', '')).strip(),
            'output_feedback_timeout_sec': float(rospy.get_param('~output_feedback_timeout_sec', 1.0)),
            'require_output_feedback': bool(rospy.get_param('~require_output_feedback', False)),
            'profile_role': str(rospy.get_param('~profile_role', 'integration')).strip().lower(),
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'vendor_runtime_mode': str(rospy.get_param('~vendor_runtime_mode', '')).strip(),
            'vendor_workspace_ros_distro': str(rospy.get_param('~vendor_workspace_ros_distro', '')).strip(),
            'vendor_workspace_python_major': int(rospy.get_param('~vendor_workspace_python_major', 0)),
            'motion_model': str(rospy.get_param('~motion_model', '')).strip(),
            'cmd_vel_semantics': str(rospy.get_param('~cmd_vel_semantics', '')).strip(),
            'allow_odom_feedback_fallback': bool(rospy.get_param('~allow_odom_feedback_fallback', True)),
        }
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['health_heartbeat_hz'] = require_positive_float('health_heartbeat_hz', config['health_heartbeat_hz'])
        config['output_feedback_timeout_sec'] = require_positive_float('output_feedback_timeout_sec', config['output_feedback_timeout_sec'])
        capability = apply_platform_safety_defaults(config)
        config['platform_contract'] = capability.summary()
        config['platform_runtime_contract'] = validate_platform_runtime_strategy(
            capability,
            config,
            owner='cmd_safety_mux_node',
        )
        config['platform_contract_bindings'] = validate_platform_contract_bindings(
            capability,
            config,
            owner='cmd_safety_mux_node',
            domain='safety',
        )
        evaluate_output_feedback_policy(
            require_output_feedback=config['require_output_feedback'],
            output_feedback_topic=config['output_feedback_topic'],
            output_feedback_fresh=True,
        )
        if not config['input_sources']:
            raise ConfigurationError('input_sources must not be empty')
        for idx, source in enumerate(config['input_sources']):
            topic = str(source.get('topic', '')).strip()
            if not topic:
                raise ConfigurationError('input_sources[{}].topic is empty'.format(idx))
            source.setdefault('name', topic)
            try:
                source['priority'] = int(source.get('priority', 0))
            except (TypeError, ValueError) as exc:
                raise ConfigurationError('input_sources[{}].priority is invalid: {}'.format(idx, exc))
        if str(config['time_source_mode']).strip().lower() not in ('ros', 'wall'):
            raise ConfigurationError('time_source_mode must be ros or wall')
        if str(config['health_emit_mode']).strip().lower() not in ('edge', 'periodic'):
            raise ConfigurationError('health_emit_mode must be edge or periodic')
        config['profile_role'] = validate_profile_runtime_flags(
            config['profile_role'],
            owner='cmd_safety_mux_node',
            lifecycle_managed=bool(config['lifecycle_managed']),
        )
        return config

    def _build_source_subscribers(self):
        self.source_subscribers = []
        for source in self.config['input_sources']:
            topic = str(source.get('topic', '')).strip()
            name = str(source.get('name', topic)).strip() or topic
            priority = int(source.get('priority', 0))
            self.source_subscribers.append(rospy.Subscriber(topic, Twist, self._twist_cb, callback_args=(name, priority), queue_size=20))

    def _twist_cb(self, msg: Twist, callback_args):
        """Observe one safety-ingress velocity command and feed arbitration.

        Args:
            msg: Incoming ``Twist`` on one configured safety ingress topic.
            callback_args: Tuple of ``(source_name, priority)`` resolved at
                subscriber construction time.

        Returns:
            None. The controller state and traffic observability counters are
            mutated in place.

        Raises:
            No explicit exception is raised. Lifecycle-inactive periods are
            recorded as dropped traffic instead of failing subscriber callbacks.

        Boundary behavior:
            The node records input traffic before lifecycle gating so the system
            manager can prove the bridge-to-safety path is live prior to
            activation.
        """
        now_sec = self.clock.now_business_sec()
        self.last_input_command_sec = now_sec
        if self.first_input_command_sec <= 0.0:
            self.first_input_command_sec = now_sec
        self.input_command_observed_count += 1
        if not self.runtime.processing_allowed:
            self.input_command_drop_count += 1
            return
        name, priority = callback_args
        command = VelocityCommand(linear_x=msg.linear.x, linear_y=msg.linear.y, angular_z=msg.angular.z)
        self.controller.update_command(name, priority, now_sec, command)

    def _estop_cb(self, msg: Bool):
        if not self.runtime.processing_allowed:
            return
        self.controller.update_estop(bool(msg.data), self.clock.now_business_sec())

    def _mode_cb(self, msg: String):
        if not self.runtime.processing_allowed:
            return
        self.controller.update_mode(msg.data)

    def _output_feedback_cb(self, msg: Bool):
        self.output_feedback_state = bool(msg.data)
        self.last_output_feedback_sec = self.clock.now_business_sec()

    def _reset_runtime(self) -> None:
        self.controller = self._build_controller()
        self.last_output_feedback_sec = 0.0
        self.output_feedback_state = False

    def _control_command_cb(self, msg: String) -> None:
        envelope = decode_lifecycle_control(msg.data)
        command = envelope.command
        if not command or not envelope.matches(rospy.get_name().split('/')[-1]):
            return
        result = self.runtime.apply(command)
        details = {'reason': result.message, 'command': command, 'target': envelope.target, 'issued_by': envelope.issued_by, 'metadata': dict(envelope.metadata or {}), **self.runtime.snapshot()}
        if not result.accepted:
            self._maybe_publish_health(details, 'warn', result.message)
            return
        if command == 'reset':
            self._reset_runtime()
        if command in ('pause', 'reset', 'shutdown'):
            self.output_pub.publish(Twist())
        self._maybe_publish_health(details, 'ok' if result.state == 'ACTIVE' else 'warn', result.message)

    def _output_feedback_fresh(self, now_sec: float) -> bool:
        if not self.config['output_feedback_topic']:
            return not self.config['require_output_feedback']
        return (now_sec - self.last_output_feedback_sec) <= self.config['output_feedback_timeout_sec'] and self.output_feedback_state

    def _apply_output_feedback_gate(self, status, payload: dict, output_feedback_fresh: bool) -> tuple[Twist, str, str]:
        """Apply downstream controller feedback policy to the outgoing command."""
        output_allowed, feedback_reason = evaluate_output_feedback_policy(
            require_output_feedback=self.config['require_output_feedback'],
            output_feedback_topic=self.config['output_feedback_topic'],
            output_feedback_fresh=output_feedback_fresh,
        )
        if not output_allowed:
            payload['reason'] = feedback_reason
            payload['command_fresh'] = False
            payload['output_inhibited_by_feedback'] = True
            payload['output'] = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
            return Twist(), 'warn', feedback_reason
        payload['output_inhibited_by_feedback'] = False
        cmd = Twist()
        cmd.linear.x = status.output.linear_x
        cmd.linear.y = status.output.linear_y
        cmd.angular.z = status.output.angular_z
        level, message = classify_safety_health(status, warn_on_idle_command_stale=self.config['warn_on_idle_command_stale'])
        return cmd, level, message

    def _maybe_publish_health(self, status_payload: dict, level: str, message: str):
        now_mono = self.clock.now_monotonic_sec()
        heartbeat_interval = 1.0 / self.config['health_heartbeat_hz']
        merged = dict(status_payload)
        merged.update(self.runtime.snapshot())
        merged.setdefault('command_path_bound', bool(self.config['input_sources']) and bool(str(self.config['output_topic']).strip()))
        merged.setdefault('command_traffic_seen', bool(self.input_command_observed_count > 0))
        merged.setdefault('input_command_observed_count', int(self.input_command_observed_count))
        merged.setdefault('input_command_drop_count', int(self.input_command_drop_count))
        merged.setdefault('first_input_command_at', float(self.first_input_command_sec))
        merged.setdefault('last_input_command_at', float(self.last_input_command_sec))
        merged.setdefault('output_command_emitted', bool(self.output_command_emitted_count > 0))
        merged.setdefault('output_command_emitted_count', int(self.output_command_emitted_count))
        merged.setdefault('last_output_command_at', float(self.last_output_command_sec))
        merged.setdefault('output_feedback_required_satisfied', (not bool(self.config['require_output_feedback'])) or bool(merged.get('output_feedback_fresh')))
        merged.setdefault('vendor_runtime_mode', self.config['vendor_runtime_mode'])
        merged.setdefault('vendor_workspace_ros_distro', self.config['vendor_workspace_ros_distro'])
        merged.setdefault('vendor_workspace_python_major', int(self.config['vendor_workspace_python_major']))
        merged.setdefault('motion_model', self.config['motion_model'])
        merged.setdefault('cmd_vel_semantics', self.config['cmd_vel_semantics'])
        merged.setdefault('platform_runtime_contract', dict(self.config['platform_runtime_contract']))
        merged.setdefault('platform_contract_bindings', dict(self.config['platform_contract_bindings']))
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'cmd_safety_mux_node',
            'status': level,
            'message': message,
            'details': merged,
        }
        signature = (
            level,
            message,
            merged.get('reason'),
            merged.get('mode'),
            merged.get('selected_source'),
            merged.get('estop_active'),
            merged.get('runtime_state'),
            merged.get('command_traffic_seen'),
            merged.get('output_command_emitted'),
        )
        should_emit = self.config['health_emit_mode'] == 'periodic'
        if not should_emit:
            should_emit = signature != self.last_health_signature or (now_mono - self.last_health_emit_mono) >= heartbeat_interval
        if not should_emit:
            return
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed_msg = HealthState()
        typed_msg.header.stamp = self.clock.now_ros_time()
        typed_msg.node = 'cmd_safety_mux_node'
        typed_msg.status = level
        typed_msg.message = message
        typed_msg.schema_version = SCHEMA_VERSION
        typed_msg.details_json = JsonCodec.dumps(merged)
        self.health_typed_pub.publish(typed_msg)
        self.last_health_signature = signature
        self.last_health_emit_mono = now_mono

    def spin(self):
        rate = rospy.Rate(self.config['publish_rate_hz'])
        while not rospy.is_shutdown():
            now_sec = self.clock.now_business_sec()
            if not self.runtime.processing_allowed:
                payload = {'reason': 'lifecycle_inactive', 'output': {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}}
                self.output_pub.publish(Twist())
                state_payload = dict(payload)
                state_payload.update(self.runtime.snapshot())
                self.state_pub.publish(String(data=JsonCodec.dumps(state_payload)))
                self._maybe_publish_health(payload, 'warn', 'lifecycle_inactive')
                rate.sleep()
                continue
            status = self.controller.evaluate(now_sec)
            payload = status.to_dict()
            payload['output_feedback_required'] = bool(self.config['require_output_feedback'])
            payload['output_feedback_topic'] = self.config['output_feedback_topic']
            payload['output_feedback_fresh'] = self._output_feedback_fresh(now_sec)
            cmd, level, message = self._apply_output_feedback_gate(status, payload, payload['output_feedback_fresh'])
            self.output_pub.publish(cmd)
            self.last_output_command_sec = now_sec
            self.output_command_emitted_count += 1
            state_payload = dict(payload)
            state_payload.update(self.runtime.snapshot())
            self.state_pub.publish(String(data=JsonCodec.dumps(state_payload)))
            self._maybe_publish_health(payload, level, message)
            rate.sleep()

    def _on_shutdown(self):
        self.output_pub.publish(Twist())


if __name__ == '__main__':
    try:
        node = CmdSafetyMuxNode()
        node.spin()
    except (rospy.ROSInterruptException, ConfigurationError):
        pass

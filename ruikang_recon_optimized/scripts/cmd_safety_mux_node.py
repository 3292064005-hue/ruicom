#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Safety controller node with arbitration, freshness checks and fail-safe stop."""

from __future__ import annotations

import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from ruikang_recon_baseline.common import ConfigurationError, JsonCodec, VelocityCommand, require_positive_float
from ruikang_recon_baseline.safety_core import SafetyController


class CmdSafetyMuxNode:
    """Arbitrate velocity sources and publish a fail-safe output command."""

    def __init__(self):
        rospy.init_node('cmd_safety_mux_node', anonymous=False)
        self.config = self._read_config()
        self.controller = SafetyController(
            max_linear_x=self.config['max_linear_x'],
            max_linear_y=self.config['max_linear_y'],
            max_angular_z=self.config['max_angular_z'],
            command_timeout_sec=self.config['command_timeout_sec'],
            estop_timeout_sec=self.config['estop_timeout_sec'],
            require_fresh_estop=self.config['require_fresh_estop'],
            default_mode=self.config['default_mode'],
        )
        self.output_pub = rospy.Publisher(self.config['output_topic'], Twist, queue_size=10)
        self.state_pub = rospy.Publisher(self.config['safety_state_topic'], String, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        rospy.Subscriber(self.config['estop_topic'], Bool, self._estop_cb, queue_size=20)
        rospy.Subscriber(self.config['control_mode_topic'], String, self._mode_cb, queue_size=20)
        self._build_source_subscribers()
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo('cmd_safety_mux_node started. output_topic=%s', self.config['output_topic'])

    def _read_config(self):
        sources = rospy.get_param('~input_sources', [])
        single_topic = rospy.get_param('~input_topic', '/cmd_vel_raw')
        if not sources:
            sources = [{'name': 'autonomy', 'topic': single_topic, 'priority': 100}]
        config = {
            'input_sources': sources,
            'output_topic': rospy.get_param('~output_topic', '/cmd_vel'),
            'estop_topic': rospy.get_param('~estop_topic', '/recon/estop'),
            'control_mode_topic': rospy.get_param('~control_mode_topic', '/recon/control_mode'),
            'safety_state_topic': rospy.get_param('~safety_state_topic', '/recon/safety_state'),
            'health_topic': rospy.get_param('~health_topic', '/recon/health'),
            'max_linear_x': float(rospy.get_param('~max_linear_x', 0.45)),
            'max_linear_y': float(rospy.get_param('~max_linear_y', 0.45)),
            'max_angular_z': float(rospy.get_param('~max_angular_z', 1.20)),
            'command_timeout_sec': float(rospy.get_param('~command_timeout_sec', 0.5)),
            'estop_timeout_sec': float(rospy.get_param('~estop_timeout_sec', 2.0)),
            'require_fresh_estop': bool(rospy.get_param('~require_fresh_estop', False)),
            'default_mode': rospy.get_param('~default_mode', 'AUTO'),
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 20.0)),
        }
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        if not config['input_sources']:
            raise ConfigurationError('input_sources must not be empty')
        for idx, source in enumerate(config['input_sources']):
            topic = str(source.get('topic', '')).strip()
            if not topic:
                raise ConfigurationError('input_sources[{}].topic is empty'.format(idx))
            source.setdefault('name', topic)
            source.setdefault('priority', 0)
        return config

    def _build_source_subscribers(self):
        self.source_subscribers = []
        for source in self.config['input_sources']:
            topic = str(source.get('topic', '')).strip()
            name = str(source.get('name', topic)).strip() or topic
            priority = int(source.get('priority', 0))
            self.source_subscribers.append(rospy.Subscriber(topic, Twist, self._twist_cb, callback_args=(name, priority), queue_size=20))

    def _twist_cb(self, msg: Twist, callback_args):
        name, priority = callback_args
        command = VelocityCommand(linear_x=msg.linear.x, linear_y=msg.linear.y, angular_z=msg.angular.z)
        self.controller.update_command(name, priority, time.time(), command)

    def _estop_cb(self, msg: Bool):
        self.controller.update_estop(bool(msg.data), time.time())

    def _mode_cb(self, msg: String):
        self.controller.update_mode(msg.data)

    def _publish_health(self, status: str, reason: str, payload: dict):
        self.health_pub.publish(String(data=JsonCodec.dumps({
            'stamp': time.time(),
            'node': 'cmd_safety_mux_node',
            'status': status,
            'message': reason,
            'details': payload,
        })))

    def spin(self):
        rate = rospy.Rate(self.config['publish_rate_hz'])
        while not rospy.is_shutdown():
            status = self.controller.evaluate(time.time())
            cmd = Twist()
            cmd.linear.x = status.output.linear_x
            cmd.linear.y = status.output.linear_y
            cmd.angular.z = status.output.angular_z
            self.output_pub.publish(cmd)
            payload = status.to_dict()
            self.state_pub.publish(String(data=JsonCodec.dumps(payload)))
            level = 'ok' if payload['reason'] == 'ok' else 'warn'
            self._publish_health(level, payload['reason'], payload)
            rate.sleep()

    def _on_shutdown(self):
        self.output_pub.publish(Twist())


if __name__ == '__main__':
    try:
        node = CmdSafetyMuxNode()
        node.spin()
    except (rospy.ROSInterruptException, ConfigurationError):
        pass

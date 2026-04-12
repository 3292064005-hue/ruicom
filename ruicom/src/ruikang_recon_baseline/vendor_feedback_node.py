#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS node that normalizes explicit vendor execution feedback.

The adapter converts one explicit vendor bool heartbeat into the canonical
``recon/platform/vendor/base_feedback_raw`` stream expected by the platform
bridge. The node can optionally require recent upstream command traffic so a
latched vendor ``true`` does not keep the deploy line falsely healthy after the
command path has gone idle.
"""

from __future__ import annotations

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION, require_positive_float
from .msg import HealthState
from .vendor_feedback_core import VendorFeedbackSnapshot, evaluate_vendor_feedback


class VendorFeedbackNode:
    """Normalize one explicit vendor feedback source into a managed heartbeat."""

    def __init__(self):
        rospy.init_node('vendor_feedback_node')
        self.config = self._read_config()
        self._last_source_value = False
        self._last_source_stamp_sec = 0.0
        self._last_command_stamp_sec = 0.0
        self._last_health_publish_sec = 0.0

        self.feedback_pub = rospy.Publisher(self.config['output_feedback_topic'], Bool, queue_size=10)
        self.summary_pub = rospy.Publisher(self.config['summary_topic'], String, queue_size=10)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)

        self.source_sub = rospy.Subscriber(self.config['explicit_feedback_topic'], Bool, self._source_cb, queue_size=10)
        self.command_sub = None
        if self.config['require_recent_command']:
            self.command_sub = rospy.Subscriber(self.config['command_topic'], Twist, self._command_cb, queue_size=10)

        self._publish_decision_health(self._current_decision())
        rospy.loginfo(
            'vendor_feedback_node started. source=%s output=%s require_recent_command=%s command_topic=%s',
            self.config['explicit_feedback_topic'],
            self.config['output_feedback_topic'],
            self.config['require_recent_command'],
            self.config['command_topic'],
        )

    def _read_config(self) -> dict:
        """Read and validate adapter configuration.

        Returns:
            dict: Normalized configuration mapping.

        Raises:
            ConfigurationError: When required topics or numeric parameters are
                missing or invalid.

        Boundary behavior:
            The explicit feedback source is always mandatory. Command gating is
            optional and only activates when ``require_recent_command`` is true.
        """
        config = {
            'explicit_feedback_topic': str(rospy.get_param('~explicit_feedback_topic', '')).strip(),
            'explicit_feedback_timeout_sec': float(rospy.get_param('~explicit_feedback_timeout_sec', 0.6)),
            'require_recent_command': bool(rospy.get_param('~require_recent_command', True)),
            'command_topic': str(rospy.get_param('~command_topic', 'recon/platform/vendor/cmd_vel')).strip(),
            'command_timeout_sec': float(rospy.get_param('~command_timeout_sec', 1.0)),
            'output_feedback_topic': str(rospy.get_param('~output_feedback_topic', 'recon/platform/vendor/base_feedback_raw')).strip() or 'recon/platform/vendor/base_feedback_raw',
            'summary_topic': str(rospy.get_param('~summary_topic', 'recon/platform/vendor_feedback_state')).strip() or 'recon/platform/vendor_feedback_state',
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 20.0)),
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
        }
        config['publish_rate_hz'] = require_positive_float('publish_rate_hz', config['publish_rate_hz'])
        config['explicit_feedback_timeout_sec'] = require_positive_float(
            'explicit_feedback_timeout_sec',
            config['explicit_feedback_timeout_sec'],
        )
        if not config['explicit_feedback_topic']:
            raise ConfigurationError('vendor_feedback_node requires non-empty explicit_feedback_topic')
        if config['explicit_feedback_topic'] == config['output_feedback_topic']:
            raise ConfigurationError('vendor_feedback_node explicit_feedback_topic must differ from output_feedback_topic to avoid feedback self-loop')
        if config['require_recent_command'] and not str(config['command_topic']).strip():
            raise ConfigurationError('vendor_feedback_node requires non-empty command_topic when require_recent_command=true')
        if config['require_recent_command']:
            config['command_timeout_sec'] = require_positive_float('command_timeout_sec', config['command_timeout_sec'])
        return config

    def _source_cb(self, msg: Bool) -> None:
        self._last_source_value = bool(msg.data)
        self._last_source_stamp_sec = rospy.get_time()

    def _command_cb(self, _msg: Twist) -> None:
        self._last_command_stamp_sec = rospy.get_time()

    def _current_decision(self):
        return evaluate_vendor_feedback(
            VendorFeedbackSnapshot(
                now_sec=rospy.get_time(),
                source_enabled=True,
                source_value=self._last_source_value,
                source_stamp_sec=self._last_source_stamp_sec,
                source_timeout_sec=self.config['explicit_feedback_timeout_sec'],
                require_recent_command=self.config['require_recent_command'],
                command_stamp_sec=self._last_command_stamp_sec,
                command_timeout_sec=self.config['command_timeout_sec'],
            )
        )

    def _summary_payload(self, decision) -> dict:
        return {
            'schema_version': SCHEMA_VERSION,
            'explicit_feedback_topic': self.config['explicit_feedback_topic'],
            'explicit_feedback_timeout_sec': float(self.config['explicit_feedback_timeout_sec']),
            'require_recent_command': bool(self.config['require_recent_command']),
            'command_topic': self.config['command_topic'],
            'command_timeout_sec': float(self.config['command_timeout_sec']),
            'output_feedback_topic': self.config['output_feedback_topic'],
            'summary_topic': self.config['summary_topic'],
            'last_source_value': bool(self._last_source_value),
            'last_source_stamp_sec': float(self._last_source_stamp_sec),
            'last_command_stamp_sec': float(self._last_command_stamp_sec),
            'source_fresh': bool(decision.source_fresh),
            'command_recent': bool(decision.command_recent),
            'output_feedback': bool(decision.output_feedback),
            'reason': str(decision.reason),
        }

    def _publish_decision_health(self, decision) -> None:
        summary = self._summary_payload(decision)
        if not summary['source_fresh']:
            self._publish_health('warn', 'explicit_feedback_stale', summary)
            return
        if not summary['output_feedback'] and summary['reason'] == 'source_false':
            self._publish_health('warn', 'explicit_feedback_false', summary)
            return
        if not summary['output_feedback'] and summary['reason'] == 'command_stale':
            self._publish_health('warn', 'vendor_command_stale', summary)
            return
        self._publish_health('ok', 'vendor_feedback_ready', summary)

    def _publish_health(self, status: str, message: str, details: dict | None = None) -> None:
        payload = {
            'stamp': rospy.get_time(),
            'node': 'vendor_feedback_node',
            'status': str(status).strip(),
            'message': str(message).strip(),
            'schema_version': SCHEMA_VERSION,
            'details': dict(details or {}),
        }
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
        self._last_health_publish_sec = float(payload['stamp'])

    def spin(self) -> None:
        rate = rospy.Rate(self.config['publish_rate_hz'])
        last_health_signature = None
        while not rospy.is_shutdown():
            decision = self._current_decision()
            summary = self._summary_payload(decision)
            self.feedback_pub.publish(Bool(data=decision.output_feedback))
            self.summary_pub.publish(String(data=JsonCodec.dumps(summary)))

            health_signature = (
                summary['reason'],
                summary['source_fresh'],
                summary['command_recent'],
                summary['output_feedback'],
            )
            now_sec = rospy.get_time()
            if health_signature != last_health_signature or (now_sec - self._last_health_publish_sec) >= 1.0:
                self._publish_decision_health(decision)
                last_health_signature = health_signature
            rate.sleep()

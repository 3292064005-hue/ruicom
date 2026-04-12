#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Managed-sidecar contract gate for explicit vendor execution feedback."""

from __future__ import annotations

import rospy
from std_msgs.msg import String

from .common import ConfigurationError, JsonCodec, SCHEMA_VERSION
from .msg import HealthState
from .vendor_sidecar_contract_core import VendorSidecarFeedbackContract, validate_vendor_sidecar_feedback_contract


class VendorSidecarContractNode:
    """Validate sidecar feedback source declarations before deploy activation."""

    def __init__(self):
        rospy.init_node('vendor_sidecar_contract_node')
        self.config = self._read_config()
        self.decision = validate_vendor_sidecar_feedback_contract(
            VendorSidecarFeedbackContract(
                require_explicit_feedback_source=self.config['require_explicit_feedback_source'],
                vendor_execution_feedback_topic=self.config['vendor_execution_feedback_topic'],
                upstream_feedback_topic=self.config['upstream_feedback_topic'],
                native_feedback_source_declared=self.config['native_feedback_source_declared'],
            )
        )
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self._last_publish_sec = 0.0
        self._publish_health('ok', 'vendor_sidecar_contract_ready')
        rospy.loginfo(
            'vendor_sidecar_contract_node ready. source_mode=%s upstream=%s vendor_source=%s',
            self.decision.source_mode,
            self.decision.upstream_feedback_topic,
            self.decision.vendor_execution_feedback_topic,
        )

    def _read_config(self) -> dict:
        return {
            'require_explicit_feedback_source': bool(rospy.get_param('~require_explicit_feedback_source', True)),
            'vendor_execution_feedback_topic': str(rospy.get_param('~vendor_execution_feedback_topic', '')).strip(),
            'upstream_feedback_topic': str(rospy.get_param('~upstream_feedback_topic', 'recon/platform/vendor/base_feedback_raw')).strip() or 'recon/platform/vendor/base_feedback_raw',
            'native_feedback_source_declared': bool(rospy.get_param('~native_feedback_source_declared', False)),
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'publish_rate_hz': float(rospy.get_param('~publish_rate_hz', 1.0) or 1.0),
        }

    def _details(self) -> dict:
        return {
            'schema_version': SCHEMA_VERSION,
            'source_mode': self.decision.source_mode,
            'require_explicit_feedback_source': bool(self.config['require_explicit_feedback_source']),
            'vendor_execution_feedback_topic': self.decision.vendor_execution_feedback_topic,
            'upstream_feedback_topic': self.decision.upstream_feedback_topic,
            'native_feedback_source_declared': bool(self.config['native_feedback_source_declared']),
        }

    def _publish_health(self, status: str, message: str) -> None:
        payload = {
            'stamp': rospy.get_time(),
            'node': 'vendor_sidecar_contract_node',
            'status': str(status).strip(),
            'message': str(message).strip(),
            'schema_version': SCHEMA_VERSION,
            'details': self._details(),
        }
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        typed = HealthState()
        typed.header.stamp = rospy.Time.now()
        typed.header.frame_id = self.config['health_frame_id']
        typed.node = payload['node']
        typed.status = payload['status']
        typed.message = payload['message']
        typed.schema_version = payload['schema_version']
        typed.details_json = JsonCodec.dumps(payload['details'])
        self.health_typed_pub.publish(typed)
        self._last_publish_sec = float(payload['stamp'])

    def spin(self) -> None:
        rate = rospy.Rate(max(0.2, float(self.config['publish_rate_hz'])))
        while not rospy.is_shutdown():
            if (rospy.get_time() - self._last_publish_sec) >= 1.0:
                self._publish_health('ok', 'vendor_sidecar_contract_ready')
            rate.sleep()


def main() -> None:
    try:
        VendorSidecarContractNode().spin()
    except ConfigurationError as exc:
        rospy.logfatal(str(exc))
        raise


if __name__ == '__main__':
    main()

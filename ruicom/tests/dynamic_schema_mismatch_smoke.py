#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rostest negative integration check for dynamic class-schema mismatches."""

from __future__ import annotations

import json
import threading
import time
import unittest

import rospy

from ruikang_recon_baseline.common import resolve_output_root
from ruikang_recon_baseline.msg import HealthState, MissionState, ZoneCaptureDynamic


def namespaced_topic(namespace: str, topic: str) -> str:
    """Resolve a relative topic against the configured ROS namespace."""
    normalized_ns = '/' + str(namespace).strip('/ ') if str(namespace).strip('/ ') else ''
    normalized_topic = '/' + str(topic).strip('/ ')
    return normalized_ns + normalized_topic


class DynamicSchemaMismatchSmokeTest(unittest.TestCase):
    """Validate that mission schema gates fault on mismatched embedded schemas."""

    def setUp(self):
        self._done = threading.Event()
        self.namespace = str(rospy.get_param('~namespace', '')).strip()
        self.output_root = resolve_output_root(
            rospy.get_param('~output_root'),
            self.namespace,
            bool(rospy.get_param('~output_root_use_namespace', True)),
        )
        self.expected_stream = str(rospy.get_param('~expected_stream', 'detections')).strip() or 'detections'
        self.expected_class_names = list(rospy.get_param('~expected_class_names', ['friendly', 'enemy', 'hostage', 'neutral']))
        self.expected_received_class_names = list(rospy.get_param('~expected_received_class_names', ['friendly', 'enemy', 'hostage']))
        self._terminal_state = None
        self._terminal_event = None
        self._health_payload = None
        self._dynamic_zone_count = 0
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~mission_state_typed_topic', 'recon/mission_state_typed')), MissionState, self._mission_state_cb, queue_size=10)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~zone_capture_dynamic_topic', 'recon/zone_capture_result_dynamic')), ZoneCaptureDynamic, self._zone_capture_cb, queue_size=10)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~health_typed_topic', 'recon/health_typed')), HealthState, self._health_cb, queue_size=20)

    def _maybe_done(self):
        if self._terminal_state == 'FAULT' and self._terminal_event == 'class_schema_mismatch' and self._health_payload is not None:
            self._done.set()

    def _mission_state_cb(self, msg: MissionState):
        self._terminal_state = msg.state
        self._terminal_event = msg.event
        self._maybe_done()

    def _zone_capture_cb(self, _msg: ZoneCaptureDynamic):
        self._dynamic_zone_count += 1

    def _health_cb(self, msg: HealthState):
        if msg.message != 'class_schema_mismatch':
            return
        payload = json.loads(msg.details_json or '{}')
        if payload.get('stream') != self.expected_stream:
            return
        self._health_payload = payload
        self._maybe_done()

    def test_dynamic_schema_mismatch_faults_before_zone_capture(self):
        self.assertTrue(self._done.wait(timeout=20.0), 'Timed out waiting for mission schema mismatch fault')
        time.sleep(1.0)
        self.assertEqual(self._terminal_state, 'FAULT')
        self.assertEqual(self._terminal_event, 'class_schema_mismatch')
        self.assertIsNotNone(self._health_payload)
        self.assertEqual(self._health_payload.get('stream'), self.expected_stream)
        self.assertEqual(self._health_payload.get('expected_class_names'), self.expected_class_names)
        self.assertEqual(self._health_payload.get('received_class_names'), self.expected_received_class_names)
        self.assertEqual(self._dynamic_zone_count, 0)


if __name__ == '__main__':
    import rostest
    rospy.init_node('dynamic_schema_mismatch_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'dynamic_schema_mismatch_smoke', DynamicSchemaMismatchSmokeTest)

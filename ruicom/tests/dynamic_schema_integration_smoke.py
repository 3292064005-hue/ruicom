#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rostest integration check for dynamic class-schema mission/recorder flow."""

from __future__ import annotations

import json
import os
import threading
import time
import unittest

import rospy
from std_msgs.msg import String

from ruikang_recon_baseline.common import resolve_output_root
from ruikang_recon_baseline.msg import MissionState, ZoneCaptureDynamic


REQUIRED_FINAL_ARTIFACTS = [
    'final_summary.json',
    'final_summary.csv',
    'final_summary_v2.json',
    'final_summary_v2.csv',
]


def namespaced_topic(namespace: str, topic: str) -> str:
    """Resolve a relative topic against the configured ROS namespace."""
    normalized_ns = '/' + str(namespace).strip('/ ') if str(namespace).strip('/ ') else ''
    normalized_topic = '/' + str(topic).strip('/ ')
    return normalized_ns + normalized_topic


class DynamicSchemaIntegrationSmokeTest(unittest.TestCase):
    """Validate the dynamic authoritative lane with an extended class schema."""

    def setUp(self):
        self._done = threading.Event()
        self.namespace = str(rospy.get_param('~namespace', '')).strip()
        self.output_root = resolve_output_root(
            rospy.get_param('~output_root'),
            self.namespace,
            bool(rospy.get_param('~output_root_use_namespace', True)),
        )
        self.expected_route_total = int(rospy.get_param('~expected_route_total', 2))
        self.expected_class_names = list(rospy.get_param('~expected_class_names', ['friendly', 'enemy', 'hostage', 'neutral']))
        raw_expected_zone_counts = rospy.get_param('~expected_zone_counts', {})
        self.expected_zone_counts = {
            str(zone_name): {str(class_name): int(value) for class_name, value in counts.items()}
            for zone_name, counts in raw_expected_zone_counts.items()
        }
        self._terminal_state = None
        self._dynamic_zones = {}
        self._latest_summary_v2 = None
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~mission_state_typed_topic', 'recon/mission_state_typed')), MissionState, self._mission_state_cb, queue_size=10)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~zone_capture_dynamic_topic', 'recon/zone_capture_result_dynamic')), ZoneCaptureDynamic, self._zone_capture_cb, queue_size=20)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~summary_snapshot_dynamic_topic', 'recon/summary_snapshot_v2')), String, self._summary_v2_cb, queue_size=10)

    def _maybe_done(self):
        if self._terminal_state == 'FINISHED' and len(self._dynamic_zones) >= self.expected_route_total and self._latest_summary_v2 is not None:
            self._done.set()

    def _mission_state_cb(self, msg: MissionState):
        self._terminal_state = msg.state
        self._maybe_done()

    def _zone_capture_cb(self, msg: ZoneCaptureDynamic):
        self._dynamic_zones[msg.route_id or msg.zone_name] = {
            'status': msg.status,
            'class_names': list(msg.class_names),
            'class_counts': [int(value) for value in msg.class_counts],
            'counts': {name: int(value) for name, value in zip(msg.class_names, msg.class_counts)},
            'frame_count': int(msg.frame_count),
        }
        self._maybe_done()

    def _summary_v2_cb(self, msg: String):
        payload = json.loads(msg.data)
        if payload.get('route_total') == self.expected_route_total:
            self._latest_summary_v2 = payload
            self._maybe_done()

    def _load_json(self, filename: str):
        with open(os.path.join(self.output_root, filename), 'r', encoding='utf-8') as handle:
            return json.load(handle)

    def test_dynamic_schema_integration_preserves_extended_classes_and_legacy_projection(self):
        self.assertTrue(self._done.wait(timeout=25.0), 'Timed out waiting for dynamic schema integration route completion')
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if all(os.path.exists(os.path.join(self.output_root, name)) for name in REQUIRED_FINAL_ARTIFACTS):
                break
            time.sleep(0.2)
        missing = [name for name in REQUIRED_FINAL_ARTIFACTS if not os.path.exists(os.path.join(self.output_root, name))]
        self.assertFalse(missing, 'Missing final artifacts: {}'.format(missing))

        legacy_summary = self._load_json('final_summary.json')
        dynamic_summary = self._load_json('final_summary_v2.json')
        self.assertEqual(self._terminal_state, 'FINISHED')
        self.assertEqual(dynamic_summary['route_total'], self.expected_route_total)
        self.assertEqual(dynamic_summary['class_names'], self.expected_class_names)
        self.assertFalse(dynamic_summary['recorder_diagnostics']['consistency_blocked'])
        self.assertEqual(len(dynamic_summary['zone_results_dynamic']), self.expected_route_total)
        self.assertEqual(len(legacy_summary['zone_results']), self.expected_route_total)

        expected_totals = {name: 0 for name in self.expected_class_names}
        for route_id, expected_counts in self.expected_zone_counts.items():
            self.assertIn(route_id, dynamic_summary['zone_results_dynamic'])
            dynamic_zone = dynamic_summary['zone_results_dynamic'][route_id]
            self.assertEqual(dynamic_zone['class_names'], self.expected_class_names)
            self.assertEqual(dynamic_zone['class_counts'], [expected_counts[name] for name in self.expected_class_names])
            self.assertEqual(dynamic_zone['counts'], expected_counts)
            self.assertEqual(dynamic_zone['status'], 'ok')
            self.assertIn(route_id, legacy_summary['zone_results'])
            legacy_zone = legacy_summary['zone_results'][route_id]
            self.assertEqual(legacy_zone['friendly'], expected_counts['friendly'])
            self.assertEqual(legacy_zone['enemy'], expected_counts['enemy'])
            self.assertEqual(legacy_zone['hostage'], expected_counts['hostage'])
            self.assertNotIn('neutral', legacy_zone)
            for class_name in self.expected_class_names:
                expected_totals[class_name] += expected_counts[class_name]
        self.assertEqual(dynamic_summary['totals_by_class'], expected_totals)
        self.assertEqual(dynamic_summary['totals_by_class']['neutral'], 6)


if __name__ == '__main__':
    import rostest
    rospy.init_node('dynamic_schema_integration_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'dynamic_schema_integration_smoke', DynamicSchemaIntegrationSmokeTest)

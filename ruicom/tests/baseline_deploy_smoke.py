#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Supervisor-aware deploy smoke test for the baseline mission chain."""

from __future__ import annotations

import json
import os
import threading
import time
import unittest

import rospy
from std_msgs.msg import String

from ruikang_recon_baseline.common import JsonCodec, resolve_output_root
from ruikang_recon_baseline.msg import DetectionArray, HealthState, MissionState, ZoneCaptureDynamic


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


class BaselineDeploySmokeTest(unittest.TestCase):
    """Validate that deploy mode reaches supervisor-controlled ACTIVE closure."""

    def setUp(self):
        self._done = threading.Event()
        self.namespace = str(rospy.get_param('~namespace', '')).strip()
        self.output_root = resolve_output_root(
            rospy.get_param('~output_root'),
            self.namespace,
            bool(rospy.get_param('~output_root_use_namespace', True)),
        )
        self.expected_route_total = int(rospy.get_param('~expected_route_total', 4))
        self.expected_class_names = list(rospy.get_param('~expected_class_names', ['friendly', 'enemy', 'hostage']))
        self.expected_detector_type = str(rospy.get_param('~expected_detector_type', 'color_blob')).strip() or 'color_blob'
        self.expected_managed_nodes = list(rospy.get_param('~expected_managed_nodes', [
            'vision_counter_node',
            'mission_manager_node',
            'mission_recorder_node',
            'cmd_safety_mux_node',
            'platform_bridge_node',
        ]))
        raw_expected_zone_counts = rospy.get_param('~expected_zone_counts', {})
        self.expected_zone_counts = {
            str(zone_name): {str(class_name): int(value) for class_name, value in counts.items()}
            for zone_name, counts in raw_expected_zone_counts.items()
        }
        self._terminal_state = None
        self._dynamic_zones = {}
        self._latest_summary_v2 = None
        self._vision_frames_seen = 0
        self._vision_detection_total = 0
        self._last_detector_type = ''
        self._manager_state = None
        self._manager_activate_seen = False
        self._node_runtime_states = {}
        self._node_lifecycle_managed = {}
        self._node_command_traffic_seen = {}
        self._manager_state_topic = rospy.get_param('~manager_state_topic', 'recon/system_manager/state')
        self._health_typed_topic = rospy.get_param('~health_typed_topic', 'recon/health_typed')
        rospy.Subscriber(namespaced_topic(self.namespace, self._manager_state_topic), String, self._manager_state_cb, queue_size=20)
        rospy.Subscriber(namespaced_topic(self.namespace, self._health_typed_topic), HealthState, self._health_typed_cb, queue_size=50)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~mission_state_typed_topic', 'recon/mission_state_typed')), MissionState, self._mission_state_cb, queue_size=10)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~zone_capture_dynamic_topic', 'recon/zone_capture_result_dynamic')), ZoneCaptureDynamic, self._zone_capture_cb, queue_size=20)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~summary_snapshot_dynamic_topic', 'recon/summary_snapshot_v2')), String, self._summary_v2_cb, queue_size=10)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~detections_topic', 'recon/detections')), DetectionArray, self._detections_cb, queue_size=10)

    def _managed_nodes_active(self) -> bool:
        return all(self._node_runtime_states.get(node) == 'ACTIVE' for node in self.expected_managed_nodes)

    def _managed_nodes_marked_managed(self) -> bool:
        return all(bool(self._node_lifecycle_managed.get(node)) for node in self.expected_managed_nodes)

    def _maybe_done(self):
        if (
            self._terminal_state == 'FINISHED'
            and len(self._dynamic_zones) >= self.expected_route_total
            and self._latest_summary_v2 is not None
            and self._vision_frames_seen > 0
            and self._vision_detection_total > 0
            and self._last_detector_type == self.expected_detector_type
            and self._manager_state == 'ACTIVE'
            and self._manager_activate_seen
            and self._managed_nodes_active()
            and self._managed_nodes_marked_managed()
            and all(self._node_command_traffic_seen.get(node) for node in ('cmd_safety_mux_node', 'platform_bridge_node'))
        ):
            self._done.set()

    def _manager_state_cb(self, msg: String):
        payload = json.loads(msg.data)
        self._manager_state = str(payload.get('state', '')).strip().upper()
        self._manager_activate_seen = self._manager_activate_seen or str(payload.get('command', '')).strip().lower() == 'activate'
        self._maybe_done()

    def _health_typed_cb(self, msg: HealthState):
        try:
            details = JsonCodec.loads(msg.details_json) if msg.details_json else {}
        except Exception:
            details = {}
        node = str(msg.node).strip()
        if node in self.expected_managed_nodes:
            self._node_runtime_states[node] = str(details.get('runtime_state', '')).strip().upper()
            self._node_lifecycle_managed[node] = bool(details.get('lifecycle_managed'))
            if node in ('cmd_safety_mux_node', 'platform_bridge_node'):
                self._node_command_traffic_seen[node] = bool(details.get('command_traffic_seen'))
            self._maybe_done()

    def _mission_state_cb(self, msg: MissionState):
        self._terminal_state = msg.state
        self._maybe_done()

    def _zone_capture_cb(self, msg: ZoneCaptureDynamic):
        self._dynamic_zones[msg.route_id or msg.zone_name] = {
            'zone_name': msg.zone_name,
            'status': msg.status,
            'class_names': list(msg.class_names),
            'class_counts': [int(value) for value in msg.class_counts],
            'counts': {name: int(value) for name, value in zip(msg.class_names, msg.class_counts)},
            'frame_count': int(msg.frame_count),
            'failure_reason': msg.failure_reason,
        }
        self._maybe_done()

    def _summary_v2_cb(self, msg: String):
        payload = json.loads(msg.data)
        if payload.get('route_total') == self.expected_route_total:
            self._latest_summary_v2 = payload
            self._maybe_done()

    def _detections_cb(self, msg: DetectionArray):
        if msg.detector_type == self.expected_detector_type and msg.detections:
            self._vision_frames_seen += 1
            self._vision_detection_total += len(msg.detections)
            self._last_detector_type = msg.detector_type
            self._maybe_done()

    def _load_json(self, filename: str):
        with open(os.path.join(self.output_root, filename), 'r', encoding='utf-8') as handle:
            return json.load(handle)

    def test_baseline_deploy_proves_supervisor_managed_closure(self):
        self.assertTrue(self._done.wait(timeout=35.0), 'Timed out waiting for deploy supervisor closure')
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
        self.assertEqual(self._manager_state, 'ACTIVE')
        self.assertTrue(self._manager_activate_seen)
        self.assertTrue(self._managed_nodes_marked_managed())
        self.assertTrue(self._managed_nodes_active())
        self.assertEqual(dynamic_summary['route_total'], self.expected_route_total)
        self.assertEqual(dynamic_summary['class_names'], self.expected_class_names)
        self.assertEqual(len(dynamic_summary['zone_results_dynamic']), self.expected_route_total)
        self.assertEqual(legacy_summary['route_total'], self.expected_route_total)
        self.assertEqual(len(legacy_summary['zone_results']), self.expected_route_total)
        self.assertFalse(dynamic_summary['recorder_diagnostics']['consistency_blocked'])
        self.assertTrue(dynamic_summary['recorder_diagnostics']['lifecycle_managed'])
        self.assertEqual(dynamic_summary['recorder_diagnostics']['runtime_state'], 'ACTIVE')
        self.assertGreater(self._vision_frames_seen, 0)
        self.assertGreater(self._vision_detection_total, 0)
        self.assertEqual(self._last_detector_type, self.expected_detector_type)

        expected_totals = {name: 0 for name in self.expected_class_names}
        for route_id, expected_counts in self.expected_zone_counts.items():
            self.assertIn(route_id, dynamic_summary['zone_results_dynamic'])
            dynamic_zone = dynamic_summary['zone_results_dynamic'][route_id]
            self.assertEqual(dynamic_zone['class_names'], self.expected_class_names)
            self.assertEqual(dynamic_zone['counts'], expected_counts)
            self.assertEqual(dynamic_zone['class_counts'], [expected_counts[name] for name in self.expected_class_names])
            self.assertEqual(dynamic_zone['status'], 'ok')
            self.assertGreaterEqual(int(dynamic_zone['frame_count']), 1)
            self.assertIn(route_id, legacy_summary['zone_results'])
            legacy_zone = legacy_summary['zone_results'][route_id]
            for legacy_name in ('friendly', 'enemy', 'hostage'):
                self.assertEqual(int(legacy_zone[legacy_name]), int(expected_counts.get(legacy_name, 0)))
                expected_totals[legacy_name] += int(expected_counts.get(legacy_name, 0))
        self.assertEqual(dynamic_summary['totals_by_class'], expected_totals)


if __name__ == '__main__':
    import rostest
    rospy.init_node('baseline_deploy_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'baseline_deploy_smoke', BaselineDeploySmokeTest)

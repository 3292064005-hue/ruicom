#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rostest proving deploy launch wiring generates mission -> behavior closure."""
from __future__ import annotations
import unittest
import rospy
from std_msgs.msg import String
from ruikang_recon_baseline.msg import DetectionArray, HealthState, MissionState
from ruikang_recon_baseline.common import JsonCodec
from ruikang_recon_baseline.lifecycle_protocol import encode_lifecycle_control


def namespaced_topic(namespace: str, topic: str) -> str:
    normalized_ns = '/' + str(namespace).strip('/ ') if str(namespace).strip('/ ') else ''
    normalized_topic = '/' + str(topic).strip('/ ')
    return normalized_ns + normalized_topic


class BehaviorRuntimeSmokeTest(unittest.TestCase):
    def setUp(self):
        self.namespace = str(rospy.get_param('~namespace', '')).strip()
        self.feedback_topic = namespaced_topic(self.namespace, rospy.get_param('~behavior_feedback_topic', 'recon/behavior_feedback'))
        self.command_topic = namespaced_topic(self.namespace, rospy.get_param('~behavior_command_topic', 'recon/behavior_command'))
        self.detections_topic = namespaced_topic(self.namespace, rospy.get_param('~detections_topic', 'recon/detections'))
        self.mission_state_topic = namespaced_topic(self.namespace, rospy.get_param('~mission_state_typed_topic', 'recon/mission_state_typed'))
        self.health_topic = namespaced_topic(self.namespace, rospy.get_param('~health_typed_topic', 'recon/health_typed'))
        self.control_topic = namespaced_topic(self.namespace, rospy.get_param('~control_command_topic', 'recon/system_manager/command'))
        self.expected_managed_nodes = list(rospy.get_param('~expected_managed_nodes', ['vendor_actuator_bridge_node', 'vendor_actuator_device_node', 'vendor_actuator_feedback_node', 'behavior_actuator_node', 'behavior_runtime_node', 'behavior_executor_node', 'mission_manager_node', 'vision_counter_node']))
        self.runtime_states = {}
        self.feedback = None
        self.command_seen = None
        self.detection_frames = 0
        self.mission_state = ''
        self.control_pub = rospy.Publisher(self.control_topic, String, queue_size=5)
        rospy.Subscriber(self.feedback_topic, String, self._feedback_cb, queue_size=10)
        rospy.Subscriber(self.command_topic, String, self._command_cb, queue_size=10)
        rospy.Subscriber(self.detections_topic, DetectionArray, self._detections_cb, queue_size=10)
        rospy.Subscriber(self.mission_state_topic, MissionState, self._mission_state_cb, queue_size=10)
        rospy.Subscriber(self.health_topic, HealthState, self._health_cb, queue_size=50)

    def _health_cb(self, msg: HealthState):
        try:
            details = JsonCodec.loads(msg.details_json) if msg.details_json else {}
        except Exception:
            details = {}
        node = str(msg.node).strip()
        if node in self.expected_managed_nodes:
            self.runtime_states[node] = str(details.get('runtime_state', '')).strip().upper()

    def _command_cb(self, msg: String):
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        if str(payload.get('action_type', '')).strip() == 'facility_attack':
            self.command_seen = payload

    def _feedback_cb(self, msg: String):
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        if str(payload.get('command_id', '')).strip() and str(payload.get('status', '')).strip().upper() == 'SUCCEEDED':
            self.feedback = payload

    def _detections_cb(self, msg: DetectionArray):
        if msg.detections:
            self.detection_frames += 1

    def _mission_state_cb(self, msg: MissionState):
        self.mission_state = str(msg.state).strip().upper()

    def _all_runtime_nodes_active(self) -> bool:
        return all(self.runtime_states.get(node) == 'ACTIVE' for node in self.expected_managed_nodes)

    def _publish_start(self) -> None:
        cmd = encode_lifecycle_control('start', target='mission_manager_node', issued_by='behavior_runtime_smoke')
        for _ in range(5):
            self.control_pub.publish(String(data=cmd))
            rospy.sleep(0.1)

    def test_behavior_runtime_chain_succeeds_through_strict_deploy_mainline(self):
        deadline = rospy.Time.now() + rospy.Duration(40.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline and not self._all_runtime_nodes_active():
            rospy.sleep(0.1)
        self.assertTrue(self._all_runtime_nodes_active(), f'managed nodes did not become ACTIVE: {self.runtime_states}')
        self._publish_start()
        deadline = rospy.Time.now() + rospy.Duration(40.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.command_seen and self.feedback and self.detection_frames > 0 and self.mission_state == 'FINISHED':
                break
            rospy.sleep(0.1)
        self.assertIsNotNone(self.command_seen, 'mission never emitted facility_attack command')
        self.assertIsNotNone(self.feedback, 'behavior chain never emitted success feedback')
        self.assertGreater(self.detection_frames, 0, 'vision never published detections')
        self.assertEqual(self.feedback['command_id'], self.command_seen['command_id'])
        self.assertEqual(self.feedback['status'], 'SUCCEEDED')
        self.assertEqual(self.command_seen['action_type'], 'facility_attack')
        self.assertEqual(self.mission_state, 'FINISHED')
        for node in self.expected_managed_nodes:
            self.assertEqual(self.runtime_states.get(node), 'ACTIVE')

if __name__ == '__main__':
    import rostest
    rospy.init_node('behavior_runtime_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'behavior_runtime_smoke', BehaviorRuntimeSmokeTest)

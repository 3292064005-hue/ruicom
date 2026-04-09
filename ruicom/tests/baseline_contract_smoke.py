#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rostest contract check for the baseline mission/recorder path without move_base."""

import json
import os
import threading
import time
import unittest

import rospy
from std_msgs.msg import String

from ruikang_recon_baseline.msg import MissionState, ZoneCaptureDynamic


TERMINAL_STATES = ('FINISHED', 'TIMEOUT', 'FAULT')
EXPECTED_CLASS_NAMES = ['friendly', 'enemy', 'hostage']
EXPECTED_ZONE_NAME = 'zone_a'
EXPECTED_DYNAMIC_STATUS = 'insufficient_observation'
EXPECTED_FAILURE_REASON = 'not_enough_frames'
EXPECTED_ZERO_COUNTS = [0, 0, 0]
REQUIRED_FINAL_ARTIFACTS = [
    'final_summary.json',
    'final_summary.csv',
    'final_summary_v2.json',
    'final_summary_v2.csv',
]


def _normalize_namespace(value):
    raw = str(value or '').strip().strip('/')
    return raw


def _resolve_topic(namespace, relative_topic):
    rel = str(relative_topic or '').strip().strip('/')
    if not rel:
        raise ValueError('relative_topic must not be empty')
    ns = _normalize_namespace(namespace)
    return '/{}/{}'.format(ns, rel) if ns else '/{}'.format(rel)


class BaselineContractSmokeTest(unittest.TestCase):
    def setUp(self):
        self._done = threading.Event()
        self._namespace = rospy.get_param('~namespace', '')
        self._topics = {
            'mission_state_typed': _resolve_topic(self._namespace, rospy.get_param('~mission_state_typed_topic', 'recon/mission_state_typed')),
            'zone_capture_dynamic': _resolve_topic(self._namespace, rospy.get_param('~zone_capture_dynamic_topic', 'recon/zone_capture_result_dynamic')),
            'summary_snapshot': _resolve_topic(self._namespace, rospy.get_param('~summary_snapshot_topic', 'recon/summary_snapshot')),
            'summary_snapshot_v2': _resolve_topic(self._namespace, rospy.get_param('~summary_snapshot_dynamic_topic', 'recon/summary_snapshot_v2')),
        }
        self._seen = {
            'mission_state_typed': False,
            'zone_capture_dynamic': False,
            'summary_snapshot': False,
            'summary_snapshot_v2': False,
        }
        self._latest = {
            'mission_state_typed': None,
            'zone_capture_dynamic': None,
            'summary_snapshot': None,
            'summary_snapshot_v2': None,
        }
        rospy.Subscriber(self._topics['mission_state_typed'], MissionState, self._mission_state_cb, queue_size=1)
        rospy.Subscriber(self._topics['zone_capture_dynamic'], ZoneCaptureDynamic, self._zone_capture_cb, queue_size=1)
        rospy.Subscriber(self._topics['summary_snapshot'], String, self._summary_cb, queue_size=1)
        rospy.Subscriber(self._topics['summary_snapshot_v2'], String, self._summary_v2_cb, queue_size=1)

    def _check_done(self):
        if all(self._seen.values()):
            self._done.set()

    def _mission_state_cb(self, msg):
        if msg.state in TERMINAL_STATES:
            payload = {
                'state': msg.state,
                'event': msg.event,
                'route_index': int(msg.route_index),
                'route_total': int(msg.route_total),
                'current_zone': msg.current_zone,
                'failure_reason': msg.failure_reason,
            }
            self._latest['mission_state_typed'] = payload
            self._seen['mission_state_typed'] = True
            self._check_done()

    def _zone_capture_cb(self, msg):
        payload = {
            'zone_name': msg.zone_name,
            'status': msg.status,
            'class_names': list(msg.class_names),
            'class_counts': [int(value) for value in msg.class_counts],
            'frame_count': int(msg.frame_count),
            'failure_reason': msg.failure_reason,
        }
        if self._is_expected_dynamic_payload(payload):
            self._latest['zone_capture_dynamic'] = payload
            self._seen['zone_capture_dynamic'] = True
            self._check_done()

    def _summary_cb(self, msg):
        payload = json.loads(msg.data)
        if self._is_expected_legacy_summary(payload):
            self._latest['summary_snapshot'] = payload
            self._seen['summary_snapshot'] = True
            self._check_done()

    def _summary_v2_cb(self, msg):
        payload = json.loads(msg.data)
        if self._is_expected_dynamic_summary(payload):
            self._latest['summary_snapshot_v2'] = payload
            self._seen['summary_snapshot_v2'] = True
            self._check_done()

    def _is_expected_dynamic_payload(self, payload):
        return (
            payload.get('zone_name') == EXPECTED_ZONE_NAME
            and payload.get('status') == EXPECTED_DYNAMIC_STATUS
            and payload.get('class_names') == EXPECTED_CLASS_NAMES
            and payload.get('class_counts') == EXPECTED_ZERO_COUNTS
            and payload.get('frame_count') == 0
            and payload.get('failure_reason') == EXPECTED_FAILURE_REASON
        )

    def _is_expected_legacy_summary(self, payload):
        mission_state = payload.get('mission_state', {})
        zone = payload.get('zone_results', {}).get(EXPECTED_ZONE_NAME, {})
        return (
            (mission_state.get('state') or payload.get('final_state')) in TERMINAL_STATES
            and payload.get('route_total') == 1
            and payload.get('totals') == {'friendly': 0, 'enemy': 0, 'hostage': 0}
            and zone.get('status') == EXPECTED_DYNAMIC_STATUS
            and zone.get('friendly') == 0
            and zone.get('enemy') == 0
            and zone.get('hostage') == 0
            and zone.get('frame_count') == 0
            and zone.get('failure_reason') == EXPECTED_FAILURE_REASON
        )

    def _is_expected_dynamic_summary(self, payload):
        mission_state = payload.get('mission_state', {})
        zone = payload.get('zone_results_dynamic', {}).get(EXPECTED_ZONE_NAME, {})
        return (
            (mission_state.get('state') or payload.get('final_state')) in TERMINAL_STATES
            and payload.get('route_total') == 1
            and payload.get('class_names') == EXPECTED_CLASS_NAMES
            and payload.get('totals_by_class') == {'friendly': 0, 'enemy': 0, 'hostage': 0}
            and zone.get('status') == EXPECTED_DYNAMIC_STATUS
            and zone.get('class_names') == EXPECTED_CLASS_NAMES
            and zone.get('class_counts') == EXPECTED_ZERO_COUNTS
            and zone.get('counts') == {'friendly': 0, 'enemy': 0, 'hostage': 0}
            and zone.get('frame_count') == 0
            and zone.get('failure_reason') == EXPECTED_FAILURE_REASON
        )

    def _load_json(self, output_root, filename):
        with open(os.path.join(output_root, filename), 'r', encoding='utf-8') as handle:
            return json.load(handle)

    def test_baseline_contract_generates_terminal_artifacts(self):
        self.assertTrue(self._done.wait(timeout=15.0), 'Timed out waiting for strong baseline contract invariants on topics: {}'.format(self._topics))
        output_root = rospy.get_param('~output_root')
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if all(os.path.exists(os.path.join(output_root, name)) for name in REQUIRED_FINAL_ARTIFACTS):
                break
            time.sleep(0.2)
        missing = [name for name in REQUIRED_FINAL_ARTIFACTS if not os.path.exists(os.path.join(output_root, name))]
        self.assertFalse(missing, 'Missing final artifacts: {}'.format(missing))

        legacy_summary = self._load_json(output_root, 'final_summary.json')
        dynamic_summary = self._load_json(output_root, 'final_summary_v2.json')
        self.assertTrue(self._is_expected_legacy_summary(legacy_summary), legacy_summary)
        self.assertTrue(self._is_expected_dynamic_summary(dynamic_summary), dynamic_summary)
        self.assertEqual(
            dynamic_summary['zone_results_dynamic'][EXPECTED_ZONE_NAME]['class_counts'],
            self._latest['zone_capture_dynamic']['class_counts'],
        )
        self.assertEqual(
            dynamic_summary['mission_state']['state'],
            self._latest['mission_state_typed']['state'],
        )


if __name__ == '__main__':
    import rostest
    rospy.init_node('baseline_contract_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'baseline_contract_smoke', BaselineContractSmokeTest)

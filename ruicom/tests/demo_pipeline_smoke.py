#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rostest smoke check for the demo synthetic profile."""

import threading
import unittest

import rospy
from std_msgs.msg import String

from ruikang_recon_baseline.msg import DetectionArray, MissionState


class DemoPipelineSmokeTest(unittest.TestCase):
    def setUp(self):
        self._event = threading.Event()
        self._seen = {
            'detections': False,
            'mission_state_typed': False,
            'zone_counts': False,
        }
        rospy.Subscriber('/recon/detections', DetectionArray, self._detections_cb, queue_size=1)
        rospy.Subscriber('/recon/mission_state_typed', MissionState, self._mission_state_cb, queue_size=1)
        rospy.Subscriber('/recon/zone_counts', String, self._zone_counts_cb, queue_size=1)

    def _check_done(self):
        if all(self._seen.values()):
            self._event.set()

    def _detections_cb(self, _msg):
        self._seen['detections'] = True
        self._check_done()

    def _mission_state_cb(self, _msg):
        self._seen['mission_state_typed'] = True
        self._check_done()

    def _zone_counts_cb(self, _msg):
        self._seen['zone_counts'] = True
        self._check_done()

    def test_demo_pipeline_topics_publish(self):
        self.assertTrue(self._event.wait(timeout=10.0), 'Timed out waiting for demo pipeline topics')


if __name__ == '__main__':
    import rostest
    rospy.init_node('demo_profile_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'demo_profile_smoke', DemoPipelineSmokeTest)

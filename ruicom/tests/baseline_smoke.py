#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rostest smoke check for the minimal baseline wrapper."""

import threading
import unittest

import rospy
from std_msgs.msg import String

from ruikang_recon_baseline.common import JsonCodec
from ruikang_recon_baseline.msg import HealthState


class BaselineWrapperSmokeTest(unittest.TestCase):
    def setUp(self):
        self._safety_event = threading.Event()
        self._recorder_event = threading.Event()
        self._recorder_runtime_state = ''
        self._recorder_lifecycle_managed = None
        rospy.Subscriber('/recon/safety_state', String, self._state_cb, queue_size=1)
        rospy.Subscriber('/recon/health_typed', HealthState, self._health_typed_cb, queue_size=10)

    def _state_cb(self, _msg):
        self._safety_event.set()

    def _health_typed_cb(self, msg: HealthState):
        if str(msg.node).strip() != 'mission_recorder_node':
            return
        try:
            details = JsonCodec.loads(msg.details_json) if msg.details_json else {}
        except Exception:
            details = {}
        self._recorder_runtime_state = str(details.get('runtime_state', '')).strip().upper()
        self._recorder_lifecycle_managed = bool(details.get('lifecycle_managed'))
        if self._recorder_runtime_state:
            self._recorder_event.set()

    def test_baseline_wrapper_publishes_safety_state(self):
        self.assertTrue(self._safety_event.wait(timeout=10.0), 'Timed out waiting for /recon/safety_state')

    def test_baseline_wrapper_keeps_recorder_unmanaged_and_active(self):
        self.assertTrue(self._recorder_event.wait(timeout=10.0), 'Timed out waiting for mission_recorder_node health on /recon/health_typed')
        self.assertFalse(self._recorder_lifecycle_managed, 'baseline.launch must keep recorder unmanaged for compatibility mode')
        self.assertEqual(self._recorder_runtime_state, 'ACTIVE')


if __name__ == '__main__':
    import rostest
    rospy.init_node('baseline_profile_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'baseline_profile_smoke', BaselineWrapperSmokeTest)

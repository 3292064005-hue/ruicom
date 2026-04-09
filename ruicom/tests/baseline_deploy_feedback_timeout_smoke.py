import json
import threading
import time
import unittest

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from ruikang_recon_baseline.msg import HealthState


def namespaced_topic(namespace: str, topic: str) -> str:
    normalized_ns = '/' + str(namespace).strip('/ ') if str(namespace).strip('/ ') else ''
    normalized_topic = '/' + str(topic).strip('/ ')
    return normalized_ns + normalized_topic


class BaselineDeployFeedbackTimeoutSmokeTest(unittest.TestCase):
    def setUp(self):
        self.namespace = str(rospy.get_param('~namespace', '')).strip()
        self._done = threading.Event()
        self._warn_seen = False
        self._state_seen = False
        self._output_zero_seen = False
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~health_typed_topic', 'recon/health_typed')), HealthState, self._health_cb, queue_size=20)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~safety_state_topic', 'recon/safety_state')), String, self._state_cb, queue_size=20)
        rospy.Subscriber(namespaced_topic(self.namespace, rospy.get_param('~output_topic', 'cmd_vel')), Twist, self._output_cb, queue_size=20)

    def _health_cb(self, msg: HealthState):
        if msg.status == 'warn' and 'output_feedback_stale' in msg.message:
            self._warn_seen = True
            if self._state_seen and self._output_zero_seen:
                self._done.set()

    def _state_cb(self, msg: String):
        payload = json.loads(msg.data)
        if payload.get('reason') == 'output_feedback_stale' and payload.get('output_inhibited_by_feedback'):
            self._state_seen = True
            if self._warn_seen and self._output_zero_seen:
                self._done.set()

    def _output_cb(self, msg: Twist):
        if abs(msg.linear.x) < 1e-9 and abs(msg.linear.y) < 1e-9 and abs(msg.angular.z) < 1e-9:
            self._output_zero_seen = True
            if self._warn_seen and self._state_seen:
                self._done.set()

    def test_missing_feedback_forces_fail_safe_stop(self):
        self.assertTrue(self._done.wait(timeout=15.0), 'Timed out waiting for feedback-timeout fail-safe')


if __name__ == '__main__':
    import rostest
    rospy.init_node('baseline_deploy_feedback_timeout_smoke_test', anonymous=True)
    rostest.rosrun('ruikang_recon_baseline', 'baseline_deploy_feedback_timeout_smoke', BaselineDeployFeedbackTimeoutSmokeTest)

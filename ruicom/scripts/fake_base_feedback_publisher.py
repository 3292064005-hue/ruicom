#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish a steady downstream base-feedback heartbeat for deploy/smoke tests."""

from __future__ import annotations

import rospy
from std_msgs.msg import Bool


class FakeBaseFeedbackPublisher:
    def __init__(self):
        rospy.init_node('fake_base_feedback_publisher', anonymous=False)
        self.topic = str(rospy.get_param('~topic', 'recon/platform/base_feedback')).strip() or 'recon/platform/base_feedback'
        self.value = bool(rospy.get_param('~value', True))
        self.publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 20.0))
        self.publisher = rospy.Publisher(self.topic, Bool, queue_size=10)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.publisher.publish(Bool(data=self.value))
            rate.sleep()


if __name__ == '__main__':
    FakeBaseFeedbackPublisher().spin()

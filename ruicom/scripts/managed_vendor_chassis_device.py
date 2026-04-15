#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ManagedVendorChassisDevice:
    """Managed chassis adapter used by strict deploy validation.

    It converts repo-level cmd_vel into vendor command echoes and bounded base
    feedback so deploy smoke exercises the same topics as the managed vendor
    runtime instead of relying on ad-hoc fake publishers.
    """
    def __init__(self):
        rospy.init_node('managed_vendor_chassis_device', anonymous=False)
        self.input_topic = str(rospy.get_param('~input_topic', 'cmd_vel')).strip() or 'cmd_vel'
        self.vendor_command_topic = str(rospy.get_param('~vendor_command_topic', 'recon/platform/vendor/cmd_vel')).strip() or 'recon/platform/vendor/cmd_vel'
        self.feedback_topic = str(rospy.get_param('~feedback_topic', 'recon/platform/vendor/base_feedback_raw')).strip() or 'recon/platform/vendor/base_feedback_raw'
        self.feedback_duration_sec = float(rospy.get_param('~feedback_duration_sec', 0.4) or 0.4)
        self.command_pub = rospy.Publisher(self.vendor_command_topic, Twist, queue_size=10)
        self.feedback_pub = rospy.Publisher(self.feedback_topic, Bool, queue_size=10)
        self.last_command_sec = 0.0
        rospy.Subscriber(self.input_topic, Twist, self._cb, queue_size=20)
    def _cb(self, msg: Twist):
        self.last_command_sec = rospy.get_time()
        self.command_pub.publish(msg)
    def spin(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            fresh = (rospy.get_time() - self.last_command_sec) <= self.feedback_duration_sec if self.last_command_sec > 0.0 else False
            self.feedback_pub.publish(Bool(data=fresh))
            rate.sleep()

if __name__ == '__main__':
    ManagedVendorChassisDevice().spin()

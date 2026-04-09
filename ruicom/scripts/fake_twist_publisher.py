#!/usr/bin/env python3
"""Publish a constant Twist command for safety-node smoke tests."""

from __future__ import annotations

import rospy
from geometry_msgs.msg import Twist


def main() -> None:
    rospy.init_node('fake_twist_publisher', anonymous=False)
    topic = str(rospy.get_param('~topic', 'cmd_vel_raw')).strip() or 'cmd_vel_raw'
    rate_hz = float(rospy.get_param('~publish_rate_hz', 10.0))
    msg = Twist()
    msg.linear.x = float(rospy.get_param('~linear_x', 0.2))
    msg.linear.y = float(rospy.get_param('~linear_y', 0.0))
    msg.angular.z = float(rospy.get_param('~angular_z', 0.0))
    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

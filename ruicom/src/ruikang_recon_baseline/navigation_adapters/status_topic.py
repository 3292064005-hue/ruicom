"""Navigation adapter backed by a goal topic plus external status topic."""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String

from ..common import Waypoint, require_positive_float, yaw_deg_to_quaternion_tuple
from ..time_core import NodeClock
from .base import NavigationAdapterBase

from ..navigation_status_payloads import CANONICAL_NAV_STATUSES, normalize_navigation_status_payload



class GoalTopicStatusAdapter(NavigationAdapterBase):
    """Publish goals and consume navigation status from an external topic."""

    @property
    def supports_cancel(self) -> bool:
        return self.cancel_publisher is not None

    @property
    def cancel_has_ack(self) -> bool:
        return self.cancel_publisher is not None

    @property
    def status_source(self) -> str:
        return 'external_status'

    def __init__(self, goal_topic: str, status_topic: str, status_timeout_sec: float, clock: NodeClock, cancel_topic: str = ''):
        self.publisher = rospy.Publisher(goal_topic, PoseStamped, queue_size=5)
        cancel_topic = str(cancel_topic).strip()
        self.cancel_publisher = rospy.Publisher(cancel_topic, String, queue_size=5) if cancel_topic else None
        self.status_timeout_sec = require_positive_float('navigation_status_timeout_sec', status_timeout_sec)
        self.clock = clock
        self.current_waypoint = None
        self.goal_sent_at = 0.0
        self.last_status = 'IDLE'
        self.last_status_stamp = 0.0
        self.subscriber = rospy.Subscriber(status_topic, String, self._status_callback, queue_size=20)

    def _status_callback(self, msg: String) -> None:
        normalized = normalize_navigation_status_payload(msg.data)
        if not normalized:
            rospy.logwarn_throttle(2.0, 'Ignoring unsupported navigation status payload: %s', str(msg.data).strip())
            return
        self.last_status = normalized
        self.last_status_stamp = self.clock.now_business_sec()

    def dispatch(self, waypoint: Waypoint) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.clock.now_ros_time()
        msg.header.frame_id = waypoint.goal_frame
        msg.pose.position.x = waypoint.x
        msg.pose.position.y = waypoint.y
        z, w = yaw_deg_to_quaternion_tuple(waypoint.yaw_deg)[2:]
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        self.publisher.publish(msg)
        self.current_waypoint = waypoint
        self.goal_sent_at = self.clock.now_business_sec()
        self.last_status = 'PENDING'
        self.last_status_stamp = self.goal_sent_at

    def poll(self, now_sec: float) -> str:
        if self.current_waypoint is None:
            return 'IDLE'
        if self.last_status_stamp <= 0.0:
            return 'PENDING'
        if (now_sec - self.last_status_stamp) > self.status_timeout_sec:
            return 'ABORTED'
        return self.last_status

    def cancel(self, now_sec=None) -> bool:
        now_sec = now_sec if now_sec is not None else self.clock.now_business_sec()
        self.current_waypoint = None
        if self.cancel_publisher is not None:
            self.cancel_publisher.publish(String(data='cancel'))
            self.last_status = 'PREEMPTED'
            self.last_status_stamp = now_sec
            return True
        self.last_status = 'PREEMPTED'
        self.last_status_stamp = now_sec
        return False

    def close(self) -> None:
        try:
            self.subscriber.unregister()
        except Exception:
            pass

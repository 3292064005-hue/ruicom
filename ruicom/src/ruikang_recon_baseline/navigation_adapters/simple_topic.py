"""Simple navigation adapter that relies on local pose arrival checks."""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

from ..common import Waypoint, require_positive_float, yaw_deg_to_quaternion_tuple
from ..mission_core import ArrivalEvaluator
from ..time_core import NodeClock
from .base import NavigationAdapterBase


class SimpleGoalTopicAdapter(NavigationAdapterBase):
    """Publish goals on a topic and derive success from pose arrival.

    Boundary behavior:
        When ``simulate_arrival_without_pose`` is enabled, stale pose data does not
        block completion forever; completion is synthesized after
        ``synthetic_arrival_delay_sec``.
    """

    @property
    def supports_cancel(self) -> bool:
        return False

    @property
    def status_source(self) -> str:
        return 'pose_evaluator'

    def __init__(
        self,
        goal_topic: str,
        arrival_evaluator: ArrivalEvaluator,
        clock: NodeClock,
        simulate_arrival_without_pose: bool,
        synthetic_arrival_delay_sec: float,
    ):
        self.publisher = rospy.Publisher(goal_topic, PoseStamped, queue_size=5)
        self.evaluator = arrival_evaluator
        self.clock = clock
        self.current_waypoint = None
        self.goal_sent_at = 0.0
        self.simulate_arrival_without_pose = bool(simulate_arrival_without_pose)
        self.synthetic_arrival_delay_sec = require_positive_float('synthetic_arrival_delay_sec', synthetic_arrival_delay_sec)

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

    def poll(self, now_sec: float) -> str:
        if self.current_waypoint is None:
            return 'IDLE'
        if self.evaluator.has_arrived(self.current_waypoint, now_sec):
            return 'SUCCEEDED'
        if self.simulate_arrival_without_pose and not self.evaluator.pose_fresh(now_sec) and (now_sec - self.goal_sent_at) >= self.synthetic_arrival_delay_sec:
            return 'SUCCEEDED'
        return 'ACTIVE'

    def cancel(self, now_sec=None) -> bool:
        _ = now_sec
        self.current_waypoint = None
        return False

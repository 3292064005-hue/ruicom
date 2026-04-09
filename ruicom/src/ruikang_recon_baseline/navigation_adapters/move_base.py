"""Navigation adapter backed by move_base actionlib."""

from __future__ import annotations

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from ..common import Waypoint, yaw_deg_to_quaternion_tuple
from ..time_core import NodeClock
from .base import NavigationAdapterBase


class MoveBaseActionAdapter(NavigationAdapterBase):
    """Dispatch waypoints through ``move_base`` actionlib.

    Args:
        action_name: Action server name.
        wait_for_server_sec: Maximum wait when connecting.
        clock: Node clock used for timestamps.

    Raises:
        RuntimeError: If the move_base action server is unavailable.
    """

    @property
    def cancel_has_ack(self) -> bool:
        return True

    @property
    def status_source(self) -> str:
        return 'actionlib'

    def __init__(self, action_name: str, wait_for_server_sec: float, clock: NodeClock):
        self.client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(wait_for_server_sec)):
            raise RuntimeError('move_base action server not available: {}'.format(action_name))
        self.clock = clock

    def dispatch(self, waypoint: Waypoint) -> None:
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = self.clock.now_ros_time()
        goal.target_pose.header.frame_id = waypoint.goal_frame
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y
        _, _, z, w = yaw_deg_to_quaternion_tuple(waypoint.yaw_deg)
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        self.client.send_goal(goal)

    def poll(self, now_sec: float) -> str:
        _ = now_sec
        state = self.client.get_state()
        if state in (actionlib.GoalStatus.PENDING,):
            return 'PENDING'
        if state in (actionlib.GoalStatus.ACTIVE,):
            return 'ACTIVE'
        if state in (actionlib.GoalStatus.SUCCEEDED,):
            return 'SUCCEEDED'
        if state in (actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED):
            return 'PREEMPTED'
        if state in (actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.LOST):
            return 'ABORTED'
        return 'IDLE'

    def cancel(self, now_sec=None) -> bool:
        _ = now_sec
        self.client.cancel_goal()
        return True

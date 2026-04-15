#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deterministic fake move_base action server for integration validation."""

from __future__ import annotations

import math
from typing import Optional

import actionlib
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult


class ManagedMoveBaseActionServer:
    """Serve ``move_base`` goals and publish bounded AMCL pose feedback.

    The fixture supports an optional bootstrap pose stream so mission preflight can
    observe a fresh ``amcl_pose`` before the first dispatch. This keeps the fake
    navigation environment aligned with the mission contract instead of relying on
    a goal-first side effect to make pose available.
    """

    def __init__(self):
        rospy.init_node('managed_move_base_action_server', anonymous=False)
        self.action_name = str(rospy.get_param('~action_name', 'move_base')).strip() or 'move_base'
        self.pose_topic = str(rospy.get_param('~pose_topic', 'amcl_pose')).strip() or 'amcl_pose'
        self.pose_frame = str(rospy.get_param('~pose_frame', 'map')).strip() or 'map'
        self.goal_completion_delay_sec = float(rospy.get_param('~goal_completion_delay_sec', 0.25))
        self.pose_publish_delay_sec = float(rospy.get_param('~pose_publish_delay_sec', 0.05))
        self.publish_initial_pose = bool(rospy.get_param('~publish_initial_pose', False))
        self.initial_pose_x = float(rospy.get_param('~initial_pose_x', 0.0))
        self.initial_pose_y = float(rospy.get_param('~initial_pose_y', 0.0))
        self.initial_pose_yaw_deg = float(rospy.get_param('~initial_pose_yaw_deg', 0.0))
        self.initial_pose_publish_hz = float(rospy.get_param('~initial_pose_publish_hz', 5.0))
        self.stop_initial_pose_after_first_goal = bool(rospy.get_param('~stop_initial_pose_after_first_goal', True))
        if self.goal_completion_delay_sec <= 0.0:
            raise ValueError('goal_completion_delay_sec must be > 0')
        if self.pose_publish_delay_sec < 0.0:
            raise ValueError('pose_publish_delay_sec must be >= 0')
        if self.publish_initial_pose and self.initial_pose_publish_hz <= 0.0:
            raise ValueError('initial_pose_publish_hz must be > 0 when publish_initial_pose=true')
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseWithCovarianceStamped, queue_size=10)
        self.server = actionlib.SimpleActionServer(self.action_name, MoveBaseAction, execute_cb=self._execute, auto_start=False)
        self._goal_seen = False
        self._bootstrap_timer: Optional[rospy.Timer] = None
        self.server.start()
        if self.publish_initial_pose:
            self._publish_explicit_pose(self.initial_pose_x, self.initial_pose_y, math.radians(self.initial_pose_yaw_deg))
            self._bootstrap_timer = rospy.Timer(rospy.Duration(1.0 / self.initial_pose_publish_hz), self._bootstrap_pose_timer_cb)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            'managed_move_base_action_server started. action_name=%s pose_topic=%s bootstrap_pose=%s',
            self.action_name,
            self.pose_topic,
            self.publish_initial_pose,
        )

    def _build_pose_message(self, x: float, y: float, yaw_rad: float) -> PoseWithCovarianceStamped:
        """Build a deterministic ``amcl_pose`` sample for the requested planar pose.

        Args:
            x: Pose x position in meters.
            y: Pose y position in meters.
            yaw_rad: Heading in radians.

        Returns:
            Fully populated ``PoseWithCovarianceStamped`` message.

        Raises:
            No explicit exception is raised. All validation happens in ``__init__``.

        Boundary behavior:
            Covariance remains fixed and intentionally small so the fixture stays
            deterministic across repeated integration runs.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.pose_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        covariance = [0.0] * 36
        covariance[0] = 0.02
        covariance[7] = 0.02
        covariance[35] = math.radians(5.0) ** 2
        msg.pose.covariance = covariance
        return msg

    def _publish_explicit_pose(self, x: float, y: float, yaw_rad: float) -> None:
        """Publish one synthetic ``amcl_pose`` sample.

        Args:
            x: Pose x position in meters.
            y: Pose y position in meters.
            yaw_rad: Heading in radians.

        Returns:
            None. The message is published on ``self.pose_topic``.

        Raises:
            No explicit exception is raised. ROS transport handles delivery.

        Boundary behavior:
            The message uses the current ROS time so freshness checks see the pose as
            live input during mission preflight.
        """
        self.pose_pub.publish(self._build_pose_message(x, y, yaw_rad))

    def _bootstrap_pose_timer_cb(self, _event) -> None:
        """Keep publishing the configured initial pose until the first goal arrives."""
        if self.stop_initial_pose_after_first_goal and self._goal_seen:
            return
        self._publish_explicit_pose(self.initial_pose_x, self.initial_pose_y, math.radians(self.initial_pose_yaw_deg))

    def _publish_pose_for_goal(self, goal) -> None:
        self.pose_pub.publish(self._build_pose_message(
            float(goal.target_pose.pose.position.x),
            float(goal.target_pose.pose.position.y),
            2.0 * math.atan2(float(goal.target_pose.pose.orientation.z), float(goal.target_pose.pose.orientation.w)),
        ))

    def _execute(self, goal) -> None:
        """Acknowledge one goal after a bounded delay and publish the matching pose.

        Args:
            goal: ``MoveBaseGoal`` received from the client.

        Returns:
            None. Success or preemption is reported through the action server.

        Raises:
            No explicit exception is raised. The server reports transport-level state
            transitions through ``SimpleActionServer``.

        Boundary behavior:
            A matching pose sample is always published before success is reported,
            which keeps arrival evaluation deterministic for the mission node.
        """
        self._goal_seen = True
        started = rospy.Time.now().to_sec()
        rate = rospy.Rate(50.0)
        pose_published = False
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.server.set_preempted(text='preempted by managed server')
                return
            elapsed = rospy.Time.now().to_sec() - started
            if not pose_published and elapsed >= self.pose_publish_delay_sec:
                self._publish_pose_for_goal(goal)
                pose_published = True
            if elapsed >= self.goal_completion_delay_sec:
                if not pose_published:
                    self._publish_pose_for_goal(goal)
                self.server.set_succeeded(MoveBaseResult(), text='managed goal succeeded')
                return
            rate.sleep()

    def _on_shutdown(self) -> None:
        if self._bootstrap_timer is not None:
            try:
                self._bootstrap_timer.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    try:
        ManagedMoveBaseActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

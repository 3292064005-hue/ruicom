"""Pose source adapters used by mission execution."""

from __future__ import annotations

import tf2_ros

from .common import PoseSnapshot, require_positive_float, quaternion_to_yaw_rad
from .time_core import NodeClock


class TfLookupPoseSource:
    """Resolve robot pose from the TF tree into the mission comparison frame."""

    def __init__(self, target_frame: str, source_frame: str, lookup_timeout_sec: float, clock: NodeClock):
        self.target_frame = str(target_frame).strip()
        self.source_frame = str(source_frame).strip()
        self.lookup_timeout_sec = require_positive_float('tf_lookup_timeout_sec', lookup_timeout_sec)
        self.clock = clock
        self.buffer = tf2_ros.Buffer(cache_time=None)
        self.listener = tf2_ros.TransformListener(self.buffer)

    def lookup(self) -> PoseSnapshot:
        transform = self.buffer.lookup_transform(
            self.target_frame,
            self.source_frame,
            self.clock.now_ros_time(),
            timeout=self.clock.to_ros_time(self.lookup_timeout_sec),
        )
        rotation = transform.transform.rotation
        return PoseSnapshot(
            stamp=transform.header.stamp.to_sec() if transform.header.stamp and transform.header.stamp.to_sec() > 0 else self.clock.now_business_sec(),
            frame_id=transform.header.frame_id,
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
            yaw_rad=quaternion_to_yaw_rad(rotation.x, rotation.y, rotation.z, rotation.w),
            source='tf_lookup',
        )

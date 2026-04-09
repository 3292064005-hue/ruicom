"""Time helpers that separate ROS business time, wall time and monotonic time."""

from __future__ import annotations

import time
from typing import Literal

import rospy

from .common import ConfigurationError

TimeSourceMode = Literal['ros', 'wall']


class NodeClock:
    """Provide explicit access to business, wall and monotonic clocks.

    Args:
        time_source_mode: ``ros`` to use ROS time for business semantics, or
            ``wall`` to preserve legacy wall-clock driven semantics.

    Returns:
        A clock object exposing ``now_business_sec()``, ``now_wall_sec()`` and
        ``now_monotonic_sec()``.

    Raises:
        ConfigurationError: If ``time_source_mode`` is empty or unsupported.

    Boundary behavior:
        ``now_business_sec()`` may legitimately return ``0.0`` when ``/use_sim_time``
        is enabled but a clock message has not arrived yet. Callers must treat that as
        an environment state, not as an implicit failure.
    """

    def __init__(self, time_source_mode: str = 'ros'):
        normalized = str(time_source_mode).strip().lower() or 'ros'
        if normalized not in ('ros', 'wall'):
            raise ConfigurationError('time_source_mode must be one of: ros, wall')
        self.time_source_mode: TimeSourceMode = normalized  # type: ignore[assignment]

    def now_business_sec(self) -> float:
        """Return the business timestamp used by mission logic and freshness checks."""
        if self.time_source_mode == 'ros':
            return rospy.Time.now().to_sec()
        return time.time()

    def now_ros_time(self) -> rospy.Time:
        """Return a ROS ``Time`` instance aligned with the configured business clock."""
        return rospy.Time.from_sec(self.now_business_sec())

    def to_ros_time(self, stamp_sec: float) -> rospy.Time:
        """Convert a floating-point timestamp into ``rospy.Time`` safely."""
        return rospy.Time.from_sec(max(0.0, float(stamp_sec)))

    @staticmethod
    def now_wall_sec() -> float:
        """Return wall clock time for host-local artifacts and filenames."""
        return time.time()

    @staticmethod
    def now_monotonic_sec() -> float:
        """Return a monotonic timestamp for throttling and interval measurement."""
        return time.monotonic()

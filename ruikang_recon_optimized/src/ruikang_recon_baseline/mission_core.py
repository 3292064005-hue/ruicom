"""Mission-domain helpers independent of ROS message transport."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

from .common import ConfigurationError, CountSeries, DetectionFrame, PoseSnapshot, Waypoint, ZoneCaptureResult, counts_template


@dataclass
class AggregationPolicy:
    class_names: Iterable[str]
    reduction: str = 'median'
    min_valid_frames: int = 1

    def validate(self) -> None:
        if self.reduction not in ('median', 'max'):
            raise ConfigurationError('Unsupported capture reduction: {}'.format(self.reduction))
        if int(self.min_valid_frames) <= 0:
            raise ConfigurationError('capture_min_valid_frames must be > 0')


class CaptureWindowAggregator:
    """Aggregate detections that arrive inside one capture window."""

    def __init__(self, policy: AggregationPolicy):
        self.policy = policy
        self.policy.validate()
        self.series = CountSeries(policy.class_names)
        self.capture_started_at = 0.0
        self.capture_finished_at = 0.0
        self.zone_name = ''
        self.frame_region = ''

    def start(self, zone_name: str, start_time: float, frame_region: str = '') -> None:
        self.series = CountSeries(self.policy.class_names)
        self.capture_started_at = float(start_time)
        self.capture_finished_at = float(start_time)
        self.zone_name = zone_name
        self.frame_region = frame_region

    def feed(self, frame: DetectionFrame, capture_end_hint: float) -> bool:
        if frame.stamp < self.capture_started_at:
            return False
        if frame.stamp > capture_end_hint:
            return False
        counts = counts_template(self.policy.class_names)
        for detection in frame.detections:
            if self.frame_region and detection.frame_region != self.frame_region:
                continue
            if detection.class_name in counts:
                counts[detection.class_name] += 1
        self.series.add_sample(counts)
        self.capture_finished_at = max(self.capture_finished_at, min(frame.stamp, capture_end_hint))
        return True

    def finalize(self, status: str, failure_reason: str = '') -> ZoneCaptureResult:
        if self.policy.reduction == 'max':
            counts = self.series.max_counts()
        else:
            counts = self.series.median_counts()
        if self.series.frame_count < max(1, int(self.policy.min_valid_frames)) and status == 'ok':
            status = 'insufficient_observation'
            failure_reason = failure_reason or 'not_enough_frames'
        return ZoneCaptureResult(
            zone_name=self.zone_name,
            status=status,
            friendly=int(counts.get('friendly', 0)),
            enemy=int(counts.get('enemy', 0)),
            hostage=int(counts.get('hostage', 0)),
            capture_started_at=self.capture_started_at,
            capture_finished_at=self.capture_finished_at,
            frame_count=self.series.frame_count,
            frame_region=self.frame_region,
            failure_reason=failure_reason,
        )


class ArrivalEvaluator:
    """Frame-consistent waypoint arrival evaluator for simple-topic mode."""

    def __init__(self, comparison_frame: str, reach_tolerance_m: float, pose_timeout_sec: float):
        if not str(comparison_frame).strip():
            raise ConfigurationError('comparison_frame must not be empty')
        if float(reach_tolerance_m) <= 0.0:
            raise ConfigurationError('goal_reach_tolerance_m must be > 0')
        if float(pose_timeout_sec) <= 0.0:
            raise ConfigurationError('pose_timeout_sec must be > 0')
        self.comparison_frame = str(comparison_frame)
        self.reach_tolerance_m = float(reach_tolerance_m)
        self.pose_timeout_sec = float(pose_timeout_sec)
        self.last_pose: Optional[PoseSnapshot] = None

    def update_pose(self, pose: PoseSnapshot) -> None:
        self.last_pose = pose

    def pose_fresh(self, now_sec: float) -> bool:
        return self.last_pose is not None and (now_sec - self.last_pose.stamp) <= self.pose_timeout_sec

    def distance_to_waypoint(self, waypoint: Waypoint, now_sec: float):
        if not self.pose_fresh(now_sec):
            return None
        assert self.last_pose is not None
        if self.last_pose.frame_id != self.comparison_frame or waypoint.goal_frame != self.comparison_frame:
            return None
        dx = self.last_pose.x - waypoint.x
        dy = self.last_pose.y - waypoint.y
        return (dx ** 2 + dy ** 2) ** 0.5

    def has_arrived(self, waypoint: Waypoint, now_sec: float) -> bool:
        distance = self.distance_to_waypoint(waypoint, now_sec)
        return distance is not None and distance <= self.reach_tolerance_m

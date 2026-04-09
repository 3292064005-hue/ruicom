"""Mission-domain helpers independent of ROS message transport."""

from __future__ import annotations

from collections import Counter, defaultdict
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
    """Aggregate detections that arrive inside one capture window.

    The aggregator keeps both class-count samples and lightweight evidence needed
    for competition-facing reports: observed position labels, evidence sources and
    per-class observation density inside the capture window.
    """

    def __init__(self, policy: AggregationPolicy):
        self.policy = policy
        self.policy.validate()
        self.series = CountSeries(policy.class_names)
        self.capture_started_at = 0.0
        self.capture_finished_at = 0.0
        self.zone_name = ''
        self.route_id = ''
        self.frame_region = ''
        self.position_samples = []
        self.evidence_sources = Counter()
        self.detected_labels = Counter()
        self.detected_class_labels = defaultdict(Counter)

    def start(self, zone_name: str, start_time: float, frame_region: str = '', route_id: str = '') -> None:
        self.series = CountSeries(self.policy.class_names)
        self.capture_started_at = float(start_time)
        self.capture_finished_at = float(start_time)
        self.zone_name = zone_name
        self.route_id = route_id
        self.frame_region = frame_region
        self.position_samples = []
        self.evidence_sources = Counter()
        self.detected_labels = Counter()
        self.detected_class_labels = defaultdict(Counter)

    def _record_detection_evidence(self, frame: DetectionFrame, detection) -> None:
        position_type = str(getattr(detection, 'observed_position_type', '') or '').strip() or ('frame_region' if detection.frame_region else 'bbox_center')
        position_label = str(getattr(detection, 'observed_position_label', '') or '').strip() or str(detection.frame_region or self.frame_region or self.zone_name).strip()
        source = str(getattr(detection, 'evidence_source', '') or '').strip() or str(frame.detector_type or 'detector').strip() or 'detector'
        sample = {
            'class_name': str(detection.class_name).strip(),
            'position_type': position_type,
            'position_label': position_label,
            'source': source,
            'stamp': float(frame.stamp),
            'bbox_center_px': {'x': int(detection.center[0]), 'y': int(detection.center[1])},
        }
        observed_position_x_m = float(getattr(detection, 'observed_position_x_m', 0.0) or 0.0)
        observed_position_y_m = float(getattr(detection, 'observed_position_y_m', 0.0) or 0.0)
        if observed_position_x_m or observed_position_y_m:
            sample['position_xy_m'] = {'x': observed_position_x_m, 'y': observed_position_y_m}
        self.position_samples.append(sample)
        self.evidence_sources[source] += 1
        if position_label:
            self.detected_labels[position_label] += 1
            self.detected_class_labels[sample['class_name']][position_label] += 1

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
                self._record_detection_evidence(frame, detection)
        self.series.add_sample(counts)
        self.capture_finished_at = max(self.capture_finished_at, min(frame.stamp, capture_end_hint))
        return True

    def _summarize_positions(self) -> list[dict]:
        grouped: dict[tuple[str, str, str], dict] = {}
        for sample in self.position_samples:
            key = (
                str(sample.get('class_name', '')).strip(),
                str(sample.get('position_type', '')).strip(),
                str(sample.get('position_label', '')).strip(),
            )
            entry = grouped.setdefault(key, {
                'class_name': key[0],
                'position_type': key[1],
                'position_label': key[2],
                'observations': 0,
                'sources': Counter(),
                'latest_stamp': 0.0,
                'mean_bbox_center_px': {'x': 0.0, 'y': 0.0},
                'mean_position_xy_m': None,
            })
            entry['observations'] += 1
            entry['sources'][str(sample.get('source', '')).strip() or 'detector'] += 1
            entry['latest_stamp'] = max(entry['latest_stamp'], float(sample.get('stamp', 0.0) or 0.0))
            bbox = sample.get('bbox_center_px', {}) or {}
            entry['mean_bbox_center_px']['x'] += float(bbox.get('x', 0.0) or 0.0)
            entry['mean_bbox_center_px']['y'] += float(bbox.get('y', 0.0) or 0.0)
            xy = sample.get('position_xy_m', None)
            if xy is not None:
                if entry['mean_position_xy_m'] is None:
                    entry['mean_position_xy_m'] = {'x': 0.0, 'y': 0.0}
                entry['mean_position_xy_m']['x'] += float(xy.get('x', 0.0) or 0.0)
                entry['mean_position_xy_m']['y'] += float(xy.get('y', 0.0) or 0.0)
        results = []
        for entry in grouped.values():
            count = max(1, int(entry['observations']))
            entry['sources'] = dict(entry['sources'])
            entry['mean_bbox_center_px'] = {
                'x': round(entry['mean_bbox_center_px']['x'] / count, 3),
                'y': round(entry['mean_bbox_center_px']['y'] / count, 3),
            }
            if entry['mean_position_xy_m'] is not None:
                entry['mean_position_xy_m'] = {
                    'x': round(entry['mean_position_xy_m']['x'] / count, 4),
                    'y': round(entry['mean_position_xy_m']['y'] / count, 4),
                }
            results.append(entry)
        results.sort(key=lambda item: (-int(item['observations']), item['class_name'], item['position_label']))
        return results

    def _evidence_summary(self) -> dict:
        return {
            'accepted_frame_count': int(self.series.frame_count),
            'evidence_sources': dict(self.evidence_sources),
            'position_observation_count': len(self.position_samples),
            'position_labels': dict(self.detected_labels),
            'class_position_labels': {
                class_name: dict(counter)
                for class_name, counter in sorted(self.detected_class_labels.items())
            },
        }

    def reset(self) -> None:
        """Clear any in-flight capture window state.

        Args:
            None.

        Returns:
            None.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            The method is idempotent and may be called even when no capture window
            is currently active.
        """
        self.series = CountSeries(self.policy.class_names)
        self.capture_started_at = 0.0
        self.capture_finished_at = 0.0
        self.zone_name = ''
        self.route_id = ''
        self.frame_region = ''
        self.position_samples = []
        self.evidence_sources = Counter()
        self.detected_labels = Counter()
        self.detected_class_labels = defaultdict(Counter)

    def finalize(self, status: str, failure_reason: str = '') -> ZoneCaptureResult:
        """Finalize the capture window into one deterministic zone result.

        Args:
            status: Requested result status, usually ``ok`` or a navigation/capture
                failure code.
            failure_reason: Optional machine-readable failure reason.

        Returns:
            A ``ZoneCaptureResult`` carrying both the forward-compatible dynamic
            class-count mapping and richer competition-facing evidence metadata.

        Raises:
            No explicit exception is raised. Aggregation policy validation already
            happened at construction time.

        Boundary behavior:
            When too few valid frames were observed, an ``ok`` status is rewritten to
            ``insufficient_observation`` while preserving the measured counts.
        """
        if self.policy.reduction == 'max':
            counts = self.series.max_counts()
        else:
            counts = self.series.median_counts()
        if self.series.frame_count < max(1, int(self.policy.min_valid_frames)) and status == 'ok':
            status = 'insufficient_observation'
            failure_reason = failure_reason or 'not_enough_frames'
        class_names = list(self.policy.class_names)
        return ZoneCaptureResult(
            zone_name=self.zone_name,
            status=status,
            route_id=self.route_id,
            capture_started_at=self.capture_started_at,
            capture_finished_at=self.capture_finished_at,
            frame_count=self.series.frame_count,
            frame_region=self.frame_region,
            failure_reason=failure_reason,
            class_names=class_names,
            class_counts=[int(counts.get(name, 0)) for name in class_names],
            task_type='recon_zone',
            objective_type='recon',
            mission_outcome='recon_completed' if status == 'ok' else status,
            position_estimates=self._summarize_positions(),
            evidence_summary=self._evidence_summary(),
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

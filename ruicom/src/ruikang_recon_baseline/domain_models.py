"""Core domain models shared across runtime components.

This module intentionally keeps pure data definitions and lightweight helpers so
higher-level runtime modules can depend on a stable, testable model layer.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, field
from statistics import median
from typing import Any, Dict, Iterable, List, Sequence

SCHEMA_VERSION = '2.4.0'
CLASS_NAMES = ('friendly', 'enemy', 'hostage')
VALID_CONTROL_MODES = ('AUTO', 'MANUAL', 'ESTOP')
VALID_PROFILE_ROLES = ('deploy', 'integration', 'contract', 'demo', 'legacy')
PROFILE_ROLE_POLICIES = {
    'deploy': {
        'lifecycle_managed': True,
        'allow_auto_start': False,
        'require_route_binding': True,
    },
    'integration': {
        'lifecycle_managed': False,
        'allow_auto_start': True,
        'require_route_binding': False,
    },
    'contract': {
        'lifecycle_managed': False,
        'allow_auto_start': True,
        'require_route_binding': False,
    },
    'demo': {
        'lifecycle_managed': False,
        'allow_auto_start': True,
        'require_route_binding': False,
    },
    'legacy': {
        'lifecycle_managed': False,
        'allow_auto_start': True,
        'require_route_binding': False,
    },
}


class ConfigurationError(RuntimeError):
    """Raised when required configuration is missing or invalid."""


@dataclass(frozen=True)
class Detection:
    class_name: str
    score: float
    x1: int
    y1: int
    x2: int
    y2: int
    frame_region: str = ''
    observed_position_type: str = ''
    observed_position_label: str = ''
    observed_position_x_m: float = 0.0
    observed_position_y_m: float = 0.0
    evidence_source: str = ''

    @property
    def bbox(self) -> List[int]:
        return [int(self.x1), int(self.y1), int(self.x2), int(self.y2)]

    @property
    def center(self) -> tuple[int, int]:
        return (int((self.x1 + self.x2) / 2), int((self.y1 + self.y2) / 2))

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        payload['bbox'] = self.bbox
        return payload


@dataclass
class DetectionFrame:
    stamp: float
    frame_id: str
    detector_type: str
    schema_version: str
    detections: List[Detection] = field(default_factory=list)
    source_image_width: int = 0
    source_image_height: int = 0
    class_names: List[str] = field(default_factory=list)
    class_schema_hash: str = ''

    def to_dict(self) -> Dict[str, Any]:
        from .schema_utils import class_schema_hash, validate_dynamic_class_names

        normalized_class_names = list(validate_dynamic_class_names(
            self.class_names or tuple(item.class_name for item in self.detections) or tuple(CLASS_NAMES),
            owner='DetectionFrame.class_names',
        ))
        return {
            'stamp': float(self.stamp),
            'frame_id': self.frame_id,
            'detector_type': self.detector_type,
            'schema_version': self.schema_version,
            'source_image_width': int(self.source_image_width),
            'source_image_height': int(self.source_image_height),
            'class_names': normalized_class_names,
            'class_schema_hash': self.class_schema_hash or class_schema_hash(normalized_class_names),
            'detections': [item.to_dict() for item in self.detections],
        }


@dataclass(frozen=True)
class NamedRegion:
    name: str
    x0: int
    y0: int
    x1: int
    y1: int

    def contains(self, x: int, y: int) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw_deg: float = 0.0
    dwell_sec: float = 0.0
    timeout_sec: float = 0.0
    frame_region: str = ''
    route_id: str = ''
    goal_frame: str = 'map'

    def to_dict(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'x': float(self.x),
            'y': float(self.y),
            'yaw_deg': float(self.yaw_deg),
            'dwell_sec': float(self.dwell_sec),
            'timeout_sec': float(self.timeout_sec),
            'frame_region': self.frame_region,
            'route_id': self.route_id,
            'goal_frame': self.goal_frame,
        }


@dataclass(frozen=True)
class PoseSnapshot:
    stamp: float
    frame_id: str
    x: float
    y: float
    yaw_rad: float
    source: str = ''


@dataclass
class ZoneCaptureResult:
    zone_name: str
    status: str
    class_names: List[str]
    class_counts: List[int]
    capture_started_at: float
    capture_finished_at: float
    frame_count: int
    frame_region: str = ''
    failure_reason: str = ''
    schema_version: str = SCHEMA_VERSION
    route_id: str = ''
    class_schema_hash: str = ''
    task_type: str = 'waypoint_capture'
    objective_type: str = 'recon'
    mission_outcome: str = ''
    task_metadata: Dict[str, Any] = field(default_factory=dict)
    position_estimates: List[Dict[str, Any]] = field(default_factory=list)
    evidence_summary: Dict[str, Any] = field(default_factory=dict)
    hazard_summary: Dict[str, Any] = field(default_factory=dict)
    action_summary: Dict[str, Any] = field(default_factory=dict)

    @property
    def counts(self) -> Dict[str, int]:
        return {name: int(value) for name, value in zip(self.class_names, self.class_counts)}

    @property
    def friendly(self) -> int:
        return int(self.counts.get('friendly', 0))

    @property
    def enemy(self) -> int:
        return int(self.counts.get('enemy', 0))

    @property
    def hostage(self) -> int:
        return int(self.counts.get('hostage', 0))

    def to_dict(self) -> Dict[str, Any]:
        from .schema_utils import class_schema_hash, coerce_class_count_mapping, validate_dynamic_class_names

        normalized_class_names = list(validate_dynamic_class_names(self.class_names, owner='ZoneCaptureResult.class_names'))
        counts = coerce_class_count_mapping(self.counts, normalized_class_names)
        normalized_payload = {
            'zone_name': self.zone_name,
            'route_id': self.route_id,
            'status': self.status,
            'class_names': normalized_class_names,
            'class_counts': [int(counts[name]) for name in normalized_class_names],
            'counts': counts,
            'capture_started_at': float(self.capture_started_at),
            'capture_finished_at': float(self.capture_finished_at),
            'frame_count': int(self.frame_count),
            'frame_region': self.frame_region,
            'failure_reason': self.failure_reason,
            'schema_version': self.schema_version,
            'class_schema_hash': self.class_schema_hash or class_schema_hash(normalized_class_names),
            'task_type': str(self.task_type).strip() or 'waypoint_capture',
            'objective_type': str(self.objective_type).strip() or 'recon',
            'mission_outcome': str(self.mission_outcome).strip(),
            'task_metadata': dict(self.task_metadata or {}),
            'position_estimates': [dict(item) for item in list(self.position_estimates or [])],
            'evidence_summary': dict(self.evidence_summary or {}),
            'hazard_summary': dict(self.hazard_summary or {}),
            'action_summary': dict(self.action_summary or {}),
        }
        return normalized_payload


@dataclass
class MissionStateSnapshot:
    stamp: float
    state: str
    event: str
    route_index: int
    route_total: int
    current_zone: str
    current_route_id: str = ''
    schema_version: str = SCHEMA_VERSION
    class_names: List[str] = field(default_factory=list)
    class_schema_hash: str = ''
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        from .schema_utils import class_schema_hash, validate_dynamic_class_names

        normalized_class_names = list(validate_dynamic_class_names(self.class_names or tuple(CLASS_NAMES), owner='MissionStateSnapshot.class_names'))
        return {
            'stamp': float(self.stamp),
            'state': self.state,
            'event': self.event,
            'route_index': int(self.route_index),
            'route_total': int(self.route_total),
            'current_zone': self.current_zone,
            'current_route_id': self.current_route_id,
            'schema_version': self.schema_version,
            'class_names': normalized_class_names,
            'class_schema_hash': self.class_schema_hash or class_schema_hash(normalized_class_names),
            'details': self.details,
        }


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class SafetyStatus:
    stamp: float
    mode: str
    selected_source: str
    estop_active: bool
    estop_fresh: bool
    command_fresh: bool
    reason: str
    output: VelocityCommand
    observed_source_count: int
    fresh_source_count: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            'stamp': float(self.stamp),
            'mode': str(self.mode),
            'selected_source': str(self.selected_source),
            'estop_active': bool(self.estop_active),
            'estop_fresh': bool(self.estop_fresh),
            'command_fresh': bool(self.command_fresh),
            'reason': str(self.reason),
            'output': {
                'linear_x': float(self.output.linear_x),
                'linear_y': float(self.output.linear_y),
                'angular_z': float(self.output.angular_z),
            },
            'observed_source_count': int(self.observed_source_count),
            'fresh_source_count': int(self.fresh_source_count),
        }


@dataclass(frozen=True)
class ModelManifest:
    """Detector manifest resolved from disk.

    Args:
        input_size: Square input size expected by the detector backend.
        class_names: Ordered class schema emitted by the model.
        parser_type: Backend-specific parser identifier.
        confidence_threshold: Post-parser confidence threshold.
        score_threshold: Parser score threshold used before confidence fusion.
        nms_threshold: Non-maximum suppression threshold.
        detector_type: Optional detector backend name declared by the manifest.
        deployment_grade: Validation grade for deploy gating. Accepted runtime
            values are normalized in ``manifest_utils``.
        model_id: Stable manifest/model identifier for audit trails.

    Boundary behavior:
        ``deployment_grade`` may be empty for non-deploy example manifests, but
        deploy profiles can require it explicitly and reject weak manifests.
    """

    input_size: int
    class_names: Sequence[str]
    parser_type: str
    confidence_threshold: float
    score_threshold: float
    nms_threshold: float
    detector_type: str = ''
    deployment_grade: str = ''
    model_id: str = ''


class JsonCodec:
    @staticmethod
    def dumps(payload: Any) -> str:
        import json
        return json.dumps(payload, ensure_ascii=False, sort_keys=True)

    @staticmethod
    def loads(payload: str) -> Any:
        import json
        return json.loads(payload)


class CountSeries:
    """Time-windowed count series for capture-window aggregation."""

    def __init__(self, class_names: Sequence[str]):
        self.class_names = tuple(class_names)
        self.samples: List[Dict[str, int]] = []

    @property
    def frame_count(self) -> int:
        return len(self.samples)

    def add(self, counts: Dict[str, int]) -> None:
        self.samples.append({name: int(counts.get(name, 0)) for name in self.class_names})

    def add_sample(self, counts: Dict[str, int]) -> None:
        self.add(counts)

    def reduce(self, method: str) -> Dict[str, int]:
        if not self.samples:
            return {name: 0 for name in self.class_names}
        if method == 'max':
            return {
                name: max(sample[name] for sample in self.samples)
                for name in self.class_names
            }
        if method != 'median':
            raise ConfigurationError('Unsupported reduction: {}'.format(method))
        return {
            name: int(median(sample[name] for sample in self.samples))
            for name in self.class_names
        }

    def median_counts(self) -> Dict[str, int]:
        return self.reduce('median')

    def max_counts(self) -> Dict[str, int]:
        return self.reduce('max')

    def aggregate_votes(self) -> Dict[str, int]:
        if not self.samples:
            return {name: 0 for name in self.class_names}
        winner_series = {name: [] for name in self.class_names}
        for sample in self.samples:
            if not sample:
                continue
            peak = max(sample.values())
            winners = {name for name, value in sample.items() if value == peak and value > 0}
            for name in self.class_names:
                winner_series[name].append(1 if name in winners else 0)
        return {
            name: int(sum(series))
            for name, series in winner_series.items()
        }


def counts_template(class_names: Iterable[str]) -> Dict[str, int]:
    return {name: 0 for name in class_names}

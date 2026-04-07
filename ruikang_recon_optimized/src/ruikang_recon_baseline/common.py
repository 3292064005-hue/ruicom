"""Common data models and utilities for the reconnaissance baseline package."""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from statistics import median
from typing import Any, Dict, Iterable, List, Optional, Sequence

SCHEMA_VERSION = '2.0.0'
CLASS_NAMES = ('friendly', 'enemy', 'hostage')
VALID_CONTROL_MODES = ('AUTO', 'MANUAL', 'ESTOP')


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

    def to_dict(self) -> Dict[str, Any]:
        return {
            'stamp': float(self.stamp),
            'frame_id': self.frame_id,
            'detector_type': self.detector_type,
            'schema_version': self.schema_version,
            'source_image_width': int(self.source_image_width),
            'source_image_height': int(self.source_image_height),
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
    yaw_deg: float
    dwell_sec: float
    timeout_sec: float
    frame_region: str = ''
    goal_frame: str = 'map'

    @classmethod
    def from_dict(cls, payload: Dict[str, Any], dwell_default: float) -> 'Waypoint':
        waypoint = cls(
            name=str(payload.get('name', '')).strip(),
            x=float(payload.get('x', 0.0)),
            y=float(payload.get('y', 0.0)),
            yaw_deg=float(payload.get('yaw_deg', 0.0)),
            dwell_sec=float(payload.get('dwell_sec', dwell_default)),
            timeout_sec=float(payload.get('timeout_sec', 30.0)),
            frame_region=str(payload.get('frame_region', '')).strip(),
            goal_frame=str(payload.get('goal_frame', 'map')).strip() or 'map',
        )
        if not waypoint.name:
            raise ConfigurationError('route waypoint name is empty')
        if waypoint.dwell_sec <= 0.0:
            raise ConfigurationError('route waypoint {} has invalid dwell_sec {}'.format(waypoint.name, waypoint.dwell_sec))
        if waypoint.timeout_sec <= 0.0:
            raise ConfigurationError('route waypoint {} has invalid timeout_sec {}'.format(waypoint.name, waypoint.timeout_sec))
        return waypoint


@dataclass
class PoseSnapshot:
    stamp: float
    frame_id: str
    x: float
    y: float
    yaw_rad: float
    source: str


@dataclass
class ZoneCaptureResult:
    zone_name: str
    status: str
    friendly: int
    enemy: int
    hostage: int
    capture_started_at: float
    capture_finished_at: float
    frame_count: int
    frame_region: str = ''
    failure_reason: str = ''
    schema_version: str = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionStateSnapshot:
    stamp: float
    state: str
    event: str
    route_index: int
    route_total: int
    current_zone: str
    details: Dict[str, Any] = field(default_factory=dict)
    schema_version: str = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            'stamp': float(self.stamp),
            'state': self.state,
            'event': self.event,
            'route_index': int(self.route_index),
            'route_total': int(self.route_total),
            'current_zone': self.current_zone,
            'schema_version': self.schema_version,
            'details': self.details,
        }


@dataclass
class VelocityCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass
class SafetyStatus:
    stamp: float
    mode: str
    selected_source: str
    estop_active: bool
    estop_fresh: bool
    command_fresh: bool
    reason: str
    output: VelocityCommand

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        payload['output'] = asdict(self.output)
        return payload


@dataclass
class ModelManifest:
    input_size: int
    class_names: List[str]
    parser_type: str
    confidence_threshold: float
    score_threshold: float
    nms_threshold: float


class JsonCodec:
    @staticmethod
    def dumps(payload: Dict[str, Any]) -> str:
        return json.dumps(payload, ensure_ascii=False, sort_keys=False)

    @staticmethod
    def loads(text: str) -> Dict[str, Any]:
        return json.loads(text)


class CountSeries:
    def __init__(self, class_names: Iterable[str]):
        self.class_names = tuple(class_names)
        self.samples: List[Dict[str, int]] = []

    def add_sample(self, counts: Dict[str, int]) -> None:
        normalized = {name: int(counts.get(name, 0)) for name in self.class_names}
        self.samples.append(normalized)

    def median_counts(self) -> Dict[str, int]:
        if not self.samples:
            return {name: 0 for name in self.class_names}
        result = {}
        for name in self.class_names:
            values = [int(sample.get(name, 0)) for sample in self.samples]
            result[name] = int(median(values))
        return result

    def max_counts(self) -> Dict[str, int]:
        if not self.samples:
            return {name: 0 for name in self.class_names}
        result = {}
        for name in self.class_names:
            values = [int(sample.get(name, 0)) for sample in self.samples]
            result[name] = max(values)
        return result

    @property
    def frame_count(self) -> int:
        return len(self.samples)


def counts_template(class_names: Iterable[str]) -> Dict[str, int]:
    return {name: 0 for name in class_names}


def expand_path(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def yaw_deg_to_quaternion_tuple(yaw_deg: float) -> tuple[float, float, float, float]:
    yaw = math.radians(float(yaw_deg))
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))




def quaternion_to_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    """Convert a quaternion into a planar yaw angle in radians.

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component.

    Returns:
        The yaw component in radians, normalized by ``atan2`` semantics.

    Raises:
        No explicit exception is raised. Non-finite inputs propagate through ``math``.

    Boundary behavior:
        This helper extracts only the planar yaw term. Roll/pitch information is ignored.
    """
    siny_cosp = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cosy_cosp = 1.0 - 2.0 * (float(y) * float(y) + float(z) * float(z))
    return math.atan2(siny_cosp, cosy_cosp)

def require_positive_float(name: str, value: Any, *, allow_zero: bool = False) -> float:
    numeric = float(value)
    if allow_zero:
        valid = numeric >= 0.0
    else:
        valid = numeric > 0.0
    if not valid:
        comparator = '>=' if allow_zero else '>'
        raise ConfigurationError('{} must be {} 0, got {}'.format(name, comparator, numeric))
    return numeric


def normalize_class_names(class_names: Sequence[Any]) -> tuple[str, ...]:
    result = tuple(str(item).strip() for item in class_names if str(item).strip())
    if not result:
        raise ConfigurationError('classes must not be empty')
    if len(set(result)) != len(result):
        raise ConfigurationError('classes contains duplicates: {}'.format(result))
    return result


def validate_named_regions(regions: Sequence[Dict[str, Any]]) -> List[NamedRegion]:
    normalized: List[NamedRegion] = []
    seen_names = set()
    for idx, region in enumerate(regions):
        name = str(region.get('name', '')).strip()
        if not name:
            raise ConfigurationError('named_regions[{}] missing name'.format(idx))
        if name in seen_names:
            raise ConfigurationError('named_regions contains duplicate name {}'.format(name))
        x0 = int(region.get('x0', 0))
        y0 = int(region.get('y0', 0))
        x1 = int(region.get('x1', 0))
        y1 = int(region.get('y1', 0))
        if x1 <= x0 or y1 <= y0:
            raise ConfigurationError('named_regions[{}] has invalid bounds ({}, {}, {}, {})'.format(idx, x0, y0, x1, y1))
        normalized.append(NamedRegion(name=name, x0=x0, y0=y0, x1=x1, y1=y1))
        seen_names.add(name)
    return normalized


def load_manifest(path: str) -> ModelManifest:
    manifest_path = expand_path(path)
    if not os.path.isfile(manifest_path):
        raise FileNotFoundError('Model manifest does not exist: {}'.format(manifest_path))
    with open(manifest_path, 'r', encoding='utf-8') as handle:
        content = handle.read()
    payload = json.loads(content)
    required = ['input_size', 'class_names', 'parser_type', 'confidence_threshold', 'score_threshold', 'nms_threshold']
    missing = [key for key in required if key not in payload]
    if missing:
        raise ConfigurationError('Model manifest missing keys: {}'.format(', '.join(missing)))
    manifest = ModelManifest(
        input_size=int(payload['input_size']),
        class_names=[str(item) for item in payload['class_names']],
        parser_type=str(payload['parser_type']),
        confidence_threshold=float(payload['confidence_threshold']),
        score_threshold=float(payload['score_threshold']),
        nms_threshold=float(payload['nms_threshold']),
    )
    if manifest.input_size <= 0:
        raise ConfigurationError('Model manifest input_size must be > 0')
    normalize_class_names(manifest.class_names)
    require_positive_float('confidence_threshold', manifest.confidence_threshold, allow_zero=True)
    require_positive_float('score_threshold', manifest.score_threshold, allow_zero=True)
    require_positive_float('nms_threshold', manifest.nms_threshold, allow_zero=True)
    return manifest

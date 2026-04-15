from __future__ import annotations

"""Competition target semantics fusion for soldier/mine/friend-or-foe recognition.

The vision detector is allowed to emit either legacy classes
(`friendly`/`enemy`/`hostage`) or competition-native raw classes
(`soldier`/`mine`/`friendly_marker`/`enemy_marker`). This core converts those
raw detections into mission-facing semantic classes that are stable enough for
competition execution:

- ``friendly_soldier``
- ``enemy_soldier``
- ``unknown_soldier``
- ``confirmed_mine``

The implementation intentionally keeps the policy deterministic and testable:
track-lite temporal windows, spatial association between soldier boxes and
marker boxes, and simple geometric gates for ground-hugging mine candidates.
"""

from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple

from .common import ConfigurationError, Detection, DetectionFrame, validate_dynamic_class_names

SEMANTIC_CLASS_NAMES = (
    'friendly_soldier',
    'enemy_soldier',
    'unknown_soldier',
    'confirmed_mine',
)
LEGACY_ALIAS_MAP = {
    'friendly': 'friendly_soldier',
    'enemy': 'enemy_soldier',
    'hostage': 'confirmed_mine',
}
RAW_CLASS_NAMES = (
    'soldier',
    'mine',
    'friendly_marker',
    'enemy_marker',
    'friendly',
    'enemy',
    'hostage',
)


@dataclass(frozen=True)
class CompetitionFusionConfig:
    """Pure-Python configuration for competition semantics fusion.

    Attributes:
        semantic_class_names: Ordered semantic classes published downstream.
        stability_window: Number of most recent observations per coarse track.
        min_confirmed_frames: Minimum positive observations required to emit a
            semantic detection.
        marker_overlap_iou: Minimum IoU between soldier and marker boxes before
            the marker is considered attached to that soldier.
        marker_center_margin_px: Additional permissive association margin so a
            chest badge or armband can still be associated even if IoU is small.
        mine_ground_band_ratio: Minimum y-center ratio required for mine
            confirmation; mines are expected in the lower image band.
        mine_max_height_ratio: Maximum bbox height ratio for a mine candidate.
        mine_min_width_px: Small-target rejection lower bound.
        soldier_min_height_px: Minimum soldier bbox height.
    """

    semantic_class_names: Tuple[str, ...] = SEMANTIC_CLASS_NAMES
    stability_window: int = 5
    min_confirmed_frames: int = 3
    marker_overlap_iou: float = 0.08
    marker_center_margin_px: int = 24
    mine_ground_band_ratio: float = 0.58
    mine_max_height_ratio: float = 0.24
    mine_min_width_px: int = 8
    soldier_min_height_px: int = 26

    def validate(self) -> 'CompetitionFusionConfig':
        semantic = tuple(validate_dynamic_class_names(self.semantic_class_names, owner='CompetitionFusionConfig.semantic_class_names'))
        if self.stability_window <= 0:
            raise ConfigurationError('competition_perception stability_window must be > 0')
        if self.min_confirmed_frames <= 0 or self.min_confirmed_frames > self.stability_window:
            raise ConfigurationError('competition_perception min_confirmed_frames must be in [1, stability_window]')
        if self.marker_overlap_iou < 0.0 or self.marker_overlap_iou > 1.0:
            raise ConfigurationError('competition_perception marker_overlap_iou must be in [0, 1]')
        if self.mine_ground_band_ratio < 0.0 or self.mine_ground_band_ratio > 1.0:
            raise ConfigurationError('competition_perception mine_ground_band_ratio must be in [0, 1]')
        if self.mine_max_height_ratio <= 0.0 or self.mine_max_height_ratio > 1.0:
            raise ConfigurationError('competition_perception mine_max_height_ratio must be in (0, 1]')
        if self.marker_center_margin_px < 0:
            raise ConfigurationError('competition_perception marker_center_margin_px must be >= 0')
        if self.mine_min_width_px <= 0:
            raise ConfigurationError('competition_perception mine_min_width_px must be > 0')
        if self.soldier_min_height_px <= 0:
            raise ConfigurationError('competition_perception soldier_min_height_px must be > 0')
        return CompetitionFusionConfig(
            semantic_class_names=semantic,
            stability_window=int(self.stability_window),
            min_confirmed_frames=int(self.min_confirmed_frames),
            marker_overlap_iou=float(self.marker_overlap_iou),
            marker_center_margin_px=int(self.marker_center_margin_px),
            mine_ground_band_ratio=float(self.mine_ground_band_ratio),
            mine_max_height_ratio=float(self.mine_max_height_ratio),
            mine_min_width_px=int(self.mine_min_width_px),
            soldier_min_height_px=int(self.soldier_min_height_px),
        )


class CompetitionPerceptionCore:
    """Fuse raw detector outputs into competition semantic detections.

    Inputs:
        - :class:`DetectionFrame` containing raw detector outputs.
        - configuration thresholds bundled in :class:`CompetitionFusionConfig`.

    Outputs:
        - New :class:`DetectionFrame` whose detections belong to
          ``semantic_class_names``.

    Exceptions:
        - :class:`ConfigurationError` when construction parameters are invalid.

    Boundary behavior:
        - Legacy raw classes are accepted and converted directly.
        - Soldier detections without a stable friend-or-foe marker become
          ``unknown_soldier`` once temporally stable.
        - Mine detections must live near the ground band and remain compact.
    """

    def __init__(self, config: CompetitionFusionConfig):
        self.config = config.validate()
        self._history: MutableMapping[Tuple[str, str], Deque[bool]] = defaultdict(lambda: deque(maxlen=self.config.stability_window))

    @staticmethod
    def _iou(a: Detection, b: Detection) -> float:
        ax1, ay1, ax2, ay2 = a.x1, a.y1, a.x2, a.y2
        bx1, by1, bx2, by2 = b.x1, b.y1, b.x2, b.y2
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        if iw <= 0 or ih <= 0:
            return 0.0
        inter = float(iw * ih)
        area_a = float(max(0, ax2 - ax1) * max(0, ay2 - ay1))
        area_b = float(max(0, bx2 - bx1) * max(0, by2 - by1))
        union = area_a + area_b - inter
        return inter / union if union > 1e-6 else 0.0

    @staticmethod
    def _center_distance(a: Detection, b: Detection) -> Tuple[int, int]:
        ax, ay = a.center
        bx, by = b.center
        return abs(ax - bx), abs(ay - by)

    @staticmethod
    def _coarse_track_key(det: Detection, width: int, height: int) -> str:
        w = max(1, int(width))
        h = max(1, int(height))
        cx, cy = det.center
        gx = int((float(cx) / float(w)) * 8.0)
        gy = int((float(cy) / float(h)) * 6.0)
        return f'{gx}:{gy}:{det.frame_region or "*"}'

    def _record_and_confirm(self, semantic_class: str, coarse_track: str, observed: bool) -> bool:
        key = (semantic_class, coarse_track)
        history = self._history[key]
        history.append(bool(observed))
        positives = sum(1 for item in history if item)
        return len(history) >= self.config.min_confirmed_frames and positives >= self.config.min_confirmed_frames

    def _classify_soldier(self, soldier: Detection, markers: Sequence[Detection], width: int, height: int) -> str:
        best = ''
        best_score = 0.0
        for marker in markers:
            iou = self._iou(soldier, marker)
            dx, dy = self._center_distance(soldier, marker)
            within_margin = dx <= self.config.marker_center_margin_px and dy <= max(self.config.marker_center_margin_px, (soldier.y2 - soldier.y1) // 3)
            score = iou + (0.05 if within_margin else 0.0)
            if score < self.config.marker_overlap_iou and not within_margin:
                continue
            if score > best_score:
                best_score = score
                if marker.class_name == 'friendly_marker':
                    best = 'friendly_soldier'
                elif marker.class_name == 'enemy_marker':
                    best = 'enemy_soldier'
        if best:
            return best
        coarse = self._coarse_track_key(soldier, width, height)
        return 'unknown_soldier' if self._record_and_confirm('unknown_soldier', coarse, True) else ''

    def _is_ground_mine(self, det: Detection, width: int, height: int) -> bool:
        bbox_h = max(1, det.y2 - det.y1)
        bbox_w = max(1, det.x2 - det.x1)
        cy_ratio = float(det.center[1]) / float(max(1, height))
        h_ratio = float(bbox_h) / float(max(1, height))
        return cy_ratio >= self.config.mine_ground_band_ratio and h_ratio <= self.config.mine_max_height_ratio and bbox_w >= self.config.mine_min_width_px

    def _normalize_raw(self, detections: Iterable[Detection]) -> Tuple[List[Detection], List[Detection], List[Detection]]:
        soldiers: List[Detection] = []
        markers: List[Detection] = []
        mines: List[Detection] = []
        for det in detections:
            cls = str(det.class_name).strip()
            if cls == 'soldier':
                soldiers.append(det)
            elif cls == 'mine':
                mines.append(det)
            elif cls in ('friendly_marker', 'enemy_marker'):
                markers.append(det)
            elif cls in LEGACY_ALIAS_MAP:
                semantic = LEGACY_ALIAS_MAP[cls]
                if semantic == 'confirmed_mine':
                    mines.append(Detection(class_name='mine', score=det.score, x1=det.x1, y1=det.y1, x2=det.x2, y2=det.y2, frame_region=det.frame_region, observed_position_type=det.observed_position_type, observed_position_label=det.observed_position_label, observed_position_x_m=det.observed_position_x_m, observed_position_y_m=det.observed_position_y_m, evidence_source=det.evidence_source or 'legacy_alias'))
                else:
                    soldiers.append(Detection(class_name='soldier', score=det.score, x1=det.x1, y1=det.y1, x2=det.x2, y2=det.y2, frame_region=det.frame_region, observed_position_type=det.observed_position_type, observed_position_label=det.observed_position_label, observed_position_x_m=det.observed_position_x_m, observed_position_y_m=det.observed_position_y_m, evidence_source=det.evidence_source or 'legacy_alias'))
                    marker_class = 'friendly_marker' if semantic == 'friendly_soldier' else 'enemy_marker'
                    markers.append(Detection(class_name=marker_class, score=det.score, x1=det.x1, y1=det.y1, x2=det.x2, y2=det.y2, frame_region=det.frame_region, observed_position_type=det.observed_position_type, observed_position_label=det.observed_position_label, observed_position_x_m=det.observed_position_x_m, observed_position_y_m=det.observed_position_y_m, evidence_source=det.evidence_source or 'legacy_alias'))
        return soldiers, markers, mines

    def process(self, frame: DetectionFrame) -> DetectionFrame:
        width = int(frame.source_image_width or 640)
        height = int(frame.source_image_height or 480)
        soldiers, markers, mines = self._normalize_raw(frame.detections)
        semantic_detections: List[Detection] = []
        for soldier in soldiers:
            if (soldier.y2 - soldier.y1) < self.config.soldier_min_height_px:
                continue
            semantic_class = self._classify_soldier(soldier, markers, width, height)
            if not semantic_class:
                continue
            coarse = self._coarse_track_key(soldier, width, height)
            if semantic_class != 'unknown_soldier' and not self._record_and_confirm(semantic_class, coarse, True):
                continue
            semantic_detections.append(Detection(
                class_name=semantic_class,
                score=float(soldier.score),
                x1=int(soldier.x1),
                y1=int(soldier.y1),
                x2=int(soldier.x2),
                y2=int(soldier.y2),
                frame_region=str(soldier.frame_region),
                observed_position_type=str(soldier.observed_position_type),
                observed_position_label=str(soldier.observed_position_label),
                observed_position_x_m=float(soldier.observed_position_x_m),
                observed_position_y_m=float(soldier.observed_position_y_m),
                evidence_source='competition_perception',
            ))
        for mine in mines:
            if not self._is_ground_mine(mine, width, height):
                continue
            coarse = self._coarse_track_key(mine, width, height)
            if not self._record_and_confirm('confirmed_mine', coarse, True):
                continue
            semantic_detections.append(Detection(
                class_name='confirmed_mine',
                score=float(mine.score),
                x1=int(mine.x1),
                y1=int(mine.y1),
                x2=int(mine.x2),
                y2=int(mine.y2),
                frame_region=str(mine.frame_region),
                observed_position_type=str(mine.observed_position_type),
                observed_position_label=str(mine.observed_position_label),
                observed_position_x_m=float(mine.observed_position_x_m),
                observed_position_y_m=float(mine.observed_position_y_m),
                evidence_source='competition_perception',
            ))
        return DetectionFrame(
            stamp=float(frame.stamp),
            frame_id=str(frame.frame_id),
            detector_type='competition_perception',
            schema_version=str(frame.schema_version),
            detections=semantic_detections,
            source_image_width=width,
            source_image_height=height,
            class_names=list(self.config.semantic_class_names),
        )

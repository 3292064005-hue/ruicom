"""Pure-Python vision core used by the ROS wrappers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Sequence

import cv2
import numpy as np

from .common import CLASS_NAMES, ConfigurationError, Detection, DetectionFrame, ModelManifest, NamedRegion, counts_template, load_manifest, validate_dynamic_class_names, validate_named_regions


@dataclass(frozen=True)
class DetectorCapability:
    """Static capability description for one detector backend."""

    name: str
    schema_mode: str
    required_config_keys: Sequence[str]
    description: str
    builder: Callable[[Dict, Sequence[str]], object]


class DetectorRegistry:
    """Thin capability registry around available detector backends."""

    def __init__(self, capabilities: Dict[str, DetectorCapability]):
        self.capabilities = {str(name).strip().lower(): capability for name, capability in capabilities.items()}

    def build(self, detector_type: str, config: Dict, class_names: Sequence[str]):
        normalized = str(detector_type).strip().lower()
        if normalized not in self.capabilities:
            raise ConfigurationError('Unsupported detector_type: {}'.format(detector_type))
        capability = self.capabilities[normalized]
        for key in capability.required_config_keys:
            if not str(config.get(key, '')).strip():
                raise ConfigurationError('detector {} requires non-empty config key {}'.format(normalized, key))
        return capability.name, capability.builder(config, class_names), capability




class ColorBlobDetector:
    """Configurable HSV-based blob detector for synthetic and lightweight demos."""

    schema_mode = 'subset'

    def __init__(self, config: Dict, class_names: Sequence[str]):
        requested_class_names = tuple(validate_dynamic_class_names(class_names, owner='ColorBlobDetector.requested_class_names'))
        self.config = config
        self.color_blob = config.get('color_blob', {})
        self.class_hsv_ranges = self._load_class_hsv_ranges(self.color_blob, requested_class_names)
        self.class_names = tuple(name for name in requested_class_names if name in self.class_hsv_ranges)
        if not self.class_names:
            raise ConfigurationError('color_blob detector does not expose any configured classes from {}'.format(requested_class_names))
        self.min_area = int(config.get('min_area_px', 180))
        self.max_area = int(config.get('max_area_px', 80000))
        if self.min_area <= 0 or self.max_area <= 0 or self.max_area < self.min_area:
            raise ConfigurationError('min_area_px/max_area_px are invalid')
        self.morph_kernel = int(self.color_blob.get('morph_kernel', 5))
        if self.morph_kernel <= 0 or self.morph_kernel % 2 == 0:
            raise ConfigurationError('color_blob.morph_kernel must be a positive odd integer')
        self.min_aspect_ratio = float(self.color_blob.get('min_aspect_ratio', 0.20))
        self.max_aspect_ratio = float(self.color_blob.get('max_aspect_ratio', 1.80))
        self.min_fill_ratio = float(self.color_blob.get('min_fill_ratio', 0.18))
        if self.min_aspect_ratio <= 0 or self.max_aspect_ratio <= 0 or self.max_aspect_ratio < self.min_aspect_ratio:
            raise ConfigurationError('color_blob aspect-ratio bounds are invalid')
        if self.min_fill_ratio < 0.0 or self.min_fill_ratio > 1.0:
            raise ConfigurationError('color_blob.min_fill_ratio must be in [0, 1]')

    @staticmethod
    def _normalize_range_entries(raw_entry) -> List[tuple[list[int], list[int]]]:
        if raw_entry is None or raw_entry == {} or raw_entry == []:
            raise ConfigurationError('color_blob class_map entry must define at least one HSV range')
        if isinstance(raw_entry, dict) and 'ranges' in raw_entry:
            raw_ranges = raw_entry.get('ranges', [])
        elif isinstance(raw_entry, dict) and 'lower' in raw_entry and 'upper' in raw_entry:
            raw_ranges = [raw_entry]
        elif isinstance(raw_entry, list):
            raw_ranges = raw_entry
        else:
            raise ConfigurationError('color_blob class_map entry must define lower/upper or ranges')
        if not raw_ranges:
            raise ConfigurationError('color_blob class_map entry must define at least one HSV range')
        normalized = []
        for entry in raw_ranges:
            if not isinstance(entry, dict):
                raise ConfigurationError('color_blob range entry must be a mapping')
            lower = entry.get('lower')
            upper = entry.get('upper')
            if not isinstance(lower, (list, tuple)) or not isinstance(upper, (list, tuple)) or len(lower) != 3 or len(upper) != 3:
                raise ConfigurationError('color_blob lower/upper ranges must be length-3 sequences')
            normalized.append(([int(value) for value in lower], [int(value) for value in upper]))
        return normalized

    @classmethod
    def _load_class_hsv_ranges(cls, color_blob: Dict, requested_class_names: Sequence[str]) -> Dict[str, List[tuple[list[int], list[int]]]]:
        class_map = color_blob.get('class_map', {}) if isinstance(color_blob, dict) else {}
        normalized: Dict[str, List[tuple[list[int], list[int]]]] = {}
        if isinstance(class_map, dict) and class_map:
            for class_name, raw_entry in class_map.items():
                normalized_name = str(class_name).strip()
                if not normalized_name:
                    raise ConfigurationError('color_blob class_map contains an empty class name')
                normalized[normalized_name] = cls._normalize_range_entries(raw_entry)
        if normalized:
            return normalized
        legacy_defaults = {
            'friendly': [],
            'enemy': [],
            'hostage': [],
        }
        if 'red_1' in color_blob and 'red_2' in color_blob:
            legacy_defaults['friendly'].extend(cls._normalize_range_entries([color_blob['red_1'], color_blob['red_2']]))
        if 'blue' in color_blob:
            legacy_defaults['enemy'].extend(cls._normalize_range_entries(color_blob['blue']))
        if 'black' in color_blob:
            legacy_defaults['hostage'].extend(cls._normalize_range_entries(color_blob['black']))
        return {name: ranges for name, ranges in legacy_defaults.items() if ranges and name in requested_class_names}

    @staticmethod
    def _build_mask(hsv: np.ndarray, lower, upper) -> np.ndarray:
        return cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))

    def _post_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _extract(self, mask: np.ndarray, class_name: str) -> List[Detection]:
        detections: List[Detection] = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area or area > self.max_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue
            aspect_ratio = float(w) / float(h)
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue
            rect_area = float(w * h)
            fill_ratio = area / rect_area if rect_area > 1e-6 else 0.0
            if fill_ratio < self.min_fill_ratio:
                continue
            score = min(0.99, max(0.10, fill_ratio))
            detections.append(Detection(class_name=class_name, score=score, x1=x, y1=y, x2=x + w, y2=y + h))
        return detections

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        detections: List[Detection] = []
        for class_name in self.class_names:
            ranges = self.class_hsv_ranges.get(class_name, [])
            if not ranges:
                continue
            mask = None
            for lower, upper in ranges:
                partial = self._build_mask(hsv, lower, upper)
                mask = partial if mask is None else cv2.bitwise_or(mask, partial)
            if mask is None:
                continue
            mask = self._post_mask(mask)
            detections.extend(self._extract(mask, class_name))
        return detections


class OnnxDetector:
    """Generic ONNX detector with manifest-driven parser selection."""

    schema_mode = 'full'

    def __init__(self, config: Dict, class_names: Sequence[str]):
        model_path = config.get('onnx_model_path', '')
        if not model_path:
            raise FileNotFoundError('onnx_model_path is empty')
        manifest_path = str(config.get('model_manifest_path', '')).strip()
        if manifest_path:
            self.manifest = load_manifest(manifest_path)
        else:
            self.manifest = ModelManifest(
                input_size=int(config.get('input_size', 640)),
                class_names=[str(item) for item in class_names],
                parser_type=str(config.get('parser_type', 'yolo_v5_like')),
                confidence_threshold=float(config.get('confidence_threshold', 0.35)),
                score_threshold=float(config.get('score_threshold', 0.25)),
                nms_threshold=float(config.get('nms_threshold', 0.45)),
            )
            if self.manifest.input_size <= 0:
                raise ConfigurationError('input_size must be > 0')
        self.class_names = list(self.manifest.class_names)
        self.net = cv2.dnn.readNetFromONNX(model_path)

    def _preprocess(self, image_bgr: np.ndarray):
        h, w = image_bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(
            image_bgr,
            scalefactor=1.0 / 255.0,
            size=(self.manifest.input_size, self.manifest.input_size),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
        )
        return blob, float(w) / float(self.manifest.input_size), float(h) / float(self.manifest.input_size)

    def _parse_yolo_like(self, output: np.ndarray, scale_x: float, scale_y: float) -> List[Detection]:
        detections: List[Detection] = []
        arr = np.squeeze(np.array(output))
        if arr.ndim == 1 or arr.size == 0:
            return detections
        if arr.ndim != 2:
            return detections
        if arr.shape[1] >= 6 and arr.shape[1] <= 10:
            for row in arr:
                x1, y1, x2, y2, score, cls_id = row[:6]
                if float(score) < self.manifest.confidence_threshold:
                    continue
                cls_id = int(cls_id)
                if cls_id < 0 or cls_id >= len(self.class_names):
                    continue
                detections.append(Detection(
                    class_name=self.class_names[cls_id],
                    score=float(score),
                    x1=int(x1 * scale_x),
                    y1=int(y1 * scale_y),
                    x2=int(x2 * scale_x),
                    y2=int(y2 * scale_y),
                ))
            return detections
        if arr.shape[1] < 6:
            return detections
        boxes = []
        scores = []
        class_ids = []
        for row in arr:
            obj = float(row[4])
            cls_scores = row[5:]
            if cls_scores.size == 0:
                continue
            cls_id = int(np.argmax(cls_scores))
            cls_score = float(cls_scores[cls_id])
            score = obj * cls_score
            if obj < self.manifest.score_threshold or score < self.manifest.confidence_threshold:
                continue
            cx, cy, w, h = row[:4]
            x1 = int((cx - w / 2.0) * scale_x)
            y1 = int((cy - h / 2.0) * scale_y)
            boxes.append([x1, y1, int(w * scale_x), int(h * scale_y)])
            scores.append(score)
            class_ids.append(cls_id)
        if not boxes:
            return detections
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.manifest.confidence_threshold, self.manifest.nms_threshold)
        if len(indices) == 0:
            return detections
        for idx in np.array(indices).reshape(-1):
            cls_id = class_ids[int(idx)]
            if cls_id < 0 or cls_id >= len(self.class_names):
                continue
            x, y, w, h = boxes[int(idx)]
            detections.append(Detection(
                class_name=self.class_names[cls_id],
                score=float(scores[int(idx)]),
                x1=int(x),
                y1=int(y),
                x2=int(x + w),
                y2=int(y + h),
            ))
        return detections

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        blob, scale_x, scale_y = self._preprocess(image_bgr)
        self.net.setInput(blob)
        output = self.net.forward()
        if self.manifest.parser_type not in ('yolo_v5_like', 'yolo_v8_like'):
            raise ConfigurationError('Unsupported parser_type: {}'.format(self.manifest.parser_type))
        return self._parse_yolo_like(output, scale_x, scale_y)


class FrameRegionAdapter:
    """Assign frame-local region names to detections.

    The adapter can operate in two modes:

    - ``named_regions``: consume the configured pixel rectangles as-is.
    - ``calibrated_named_regions``: treat configured regions as authored in one
      reference image plane and project them into the current frame dimensions
      using the supplied region-calibration metadata.
    """

    def __init__(self, adapter_type: str, regions: Sequence[Dict], *, region_calibration: Dict[str, Any] | None = None):
        self.adapter_type = str(adapter_type).strip().lower()
        self._base_region_payloads = [dict(item) for item in (regions or [])]
        self._base_regions: List[NamedRegion] = validate_named_regions(regions) if regions else []
        self.region_calibration = dict(region_calibration or {})
        self.regions: List[NamedRegion] = list(self._base_regions)
        self._resolved_frame_shape: tuple[int, int] = (0, 0)

    def _scale_ratio(self, target_width: int, target_height: int) -> tuple[float, float]:
        reference_width = int(self.region_calibration.get('reference_image_width', 0) or 0)
        reference_height = int(self.region_calibration.get('reference_image_height', 0) or 0)
        if reference_width <= 0 or reference_height <= 0:
            return 1.0, 1.0
        x_scale = float(self.region_calibration.get('x_scale', 1.0) or 1.0)
        y_scale = float(self.region_calibration.get('y_scale', 1.0) or 1.0)
        return (float(target_width) / float(reference_width)) * x_scale, (float(target_height) / float(reference_height)) * y_scale

    def resolve_regions(self, frame_width: int, frame_height: int) -> List[NamedRegion]:
        if self.adapter_type in ('none', '', 'disabled'):
            self.regions = []
            self._resolved_frame_shape = (int(frame_width), int(frame_height))
            return []
        if self.adapter_type != 'calibrated_named_regions' or not self.region_calibration:
            self.regions = list(self._base_regions)
            self._resolved_frame_shape = (int(frame_width), int(frame_height))
            return list(self.regions)
        width = max(1, int(frame_width))
        height = max(1, int(frame_height))
        if self._resolved_frame_shape == (width, height) and self.regions:
            return list(self.regions)
        ratio_x, ratio_y = self._scale_ratio(width, height)
        x_offset_px = float(self.region_calibration.get('x_offset_px', 0.0) or 0.0)
        y_offset_px = float(self.region_calibration.get('y_offset_px', 0.0) or 0.0)
        projected_payloads = []
        for payload in self._base_region_payloads:
            projected_payloads.append({
                'name': payload['name'],
                'x0': int(round(float(payload['x0']) * ratio_x + x_offset_px)),
                'y0': int(round(float(payload['y0']) * ratio_y + y_offset_px)),
                'x1': int(round(float(payload['x1']) * ratio_x + x_offset_px)),
                'y1': int(round(float(payload['y1']) * ratio_y + y_offset_px)),
            })
        self.regions = validate_named_regions(projected_payloads)
        self._resolved_frame_shape = (width, height)
        return list(self.regions)

    def region_payloads(self, frame_width: int, frame_height: int) -> List[Dict[str, int]]:
        return [
            {'name': region.name, 'x0': region.x0, 'y0': region.y0, 'x1': region.x1, 'y1': region.y1}
            for region in self.resolve_regions(frame_width, frame_height)
        ]

    def assign(self, detections: Sequence[Detection], *, frame_width: int = 0, frame_height: int = 0) -> List[Detection]:
        if self.adapter_type in ('none', '', 'disabled'):
            return list(detections)
        if frame_width > 0 and frame_height > 0:
            self.resolve_regions(frame_width, frame_height)
        updated: List[Detection] = []
        for detection in detections:
            cx, cy = detection.center
            region_name = ''
            for region in self.regions:
                if region.contains(cx, cy):
                    region_name = region.name
                    break
            updated.append(Detection(
                class_name=detection.class_name,
                score=detection.score,
                x1=detection.x1,
                y1=detection.y1,
                x2=detection.x2,
                y2=detection.y2,
                frame_region=region_name,
                observed_position_type=detection.observed_position_type,
                observed_position_label=detection.observed_position_label,
                observed_position_x_m=detection.observed_position_x_m,
                observed_position_y_m=detection.observed_position_y_m,
                evidence_source=detection.evidence_source,
            ))
        return updated

    def stable_region_counts(self, frame: DetectionFrame, history: Dict[str, Deque[Dict[str, int]]], stable_window: int, class_names: Sequence[str]) -> Dict[str, Dict[str, int]]:
        if self.adapter_type in ('none', '', 'disabled') or not self.regions:
            return {}
        counts = {region.name: counts_template(class_names) for region in self.regions}
        for detection in frame.detections:
            if detection.frame_region and detection.frame_region in counts and detection.class_name in counts[detection.frame_region]:
                counts[detection.frame_region][detection.class_name] += 1
        result: Dict[str, Dict[str, int]] = {}
        for region in self.regions:
            bucket = history.setdefault(region.name, deque(maxlen=max(1, stable_window)))
            bucket.append(counts[region.name])
            result[region.name] = counts_template(class_names)
            if not bucket:
                continue
            for class_name in class_names:
                values = [entry.get(class_name, 0) for entry in bucket]
                result[region.name][class_name] = int(np.median(values))
        return result


def build_detector(config: Dict, class_names: Sequence[str]):
    """Build the configured detector together with its capability manifest.

    Args:
        config: Detector configuration mapping.
        class_names: Requested authoritative class ordering.

    Returns:
        Tuple of ``(detector_type, detector, capability)``.

    Raises:
        ConfigurationError: If the backend type is unsupported or required config
            fields are missing.

    Boundary behavior:
        The color-blob backend has no mandatory file-path configuration, while ONNX
        requires a concrete model path.
    """
    registry = DetectorRegistry({
        'color_blob': DetectorCapability(
            name='color_blob',
            schema_mode='subset',
            required_config_keys=(),
            description='HSV color-blob detector for synthetic/demo usage',
            builder=lambda cfg, names: ColorBlobDetector(cfg, names),
        ),
        'onnx': DetectorCapability(
            name='onnx',
            schema_mode='full',
            required_config_keys=('onnx_model_path',),
            description='ONNX runtime detector with manifest-driven parsing',
            builder=lambda cfg, names: OnnxDetector(cfg, names),
        ),
    })
    return registry.build(str(config.get('detector_type', 'color_blob')), config, class_names)


def draw_overlay(
    image_bgr: np.ndarray,
    detections: Sequence[Detection],
    regions: Sequence[Dict],
    region_counts: Dict[str, Dict[str, int]],
    class_names: Sequence[str] | None = None,
) -> np.ndarray:
    """Render an annotated debug overlay for detections and region counts.

    Args:
        image_bgr: Source BGR image.
        detections: Detection list for the current frame.
        regions: Optional named-region definitions.
        region_counts: Stable per-region count mapping.
        class_names: Preferred ordered class list for region labels.

    Returns:
        Annotated BGR canvas.

    Raises:
        No explicit exception is raised. OpenCV errors propagate naturally.

    Boundary behavior:
        When ``class_names`` is omitted, the overlay derives a deterministic class
        ordering from ``region_counts`` and falls back to the legacy schema only when
        no dynamic information is available.
    """
    canvas = image_bgr.copy()
    color_map = {
        'friendly': (0, 0, 255),
        'enemy': (255, 0, 0),
        'hostage': (20, 20, 20),
    }
    overlay_classes = [str(item).strip() for item in (class_names or ()) if str(item).strip()]
    if not overlay_classes:
        discovered = []
        for counts in region_counts.values():
            for key in counts.keys():
                key = str(key).strip()
                if key and key not in discovered:
                    discovered.append(key)
        overlay_classes = discovered or list(CLASS_NAMES)
    for region in regions:
        x0, y0, x1, y1 = int(region.get('x0', 0)), int(region.get('y0', 0)), int(region.get('x1', 0)), int(region.get('y1', 0))
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 255, 0), 2)
        label_name = str(region.get('name', 'region'))
        counts = region_counts.get(label_name, counts_template(overlay_classes))
        counts = counts_template(overlay_classes) | {name: int(counts.get(name, 0)) for name in overlay_classes}
        compact_parts = []
        for class_name in overlay_classes:
            alias = class_name[:1].upper() if len(class_name) == 1 else class_name[:2].upper()
            compact_parts.append('{}:{}'.format(alias, counts.get(class_name, 0)))
        label = '{} {}'.format(label_name, ' '.join(compact_parts)).strip()
        cv2.putText(canvas, label, (x0 + 4, max(18, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    for detection in detections:
        color = color_map.get(detection.class_name, (0, 255, 255))
        cv2.rectangle(canvas, (detection.x1, detection.y1), (detection.x2, detection.y2), color, 2)
        label = '{}:{:.2f}'.format(detection.class_name, detection.score)
        if detection.frame_region:
            label += ' @{}'.format(detection.frame_region)
        cv2.putText(canvas, label, (detection.x1, max(16, detection.y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return canvas

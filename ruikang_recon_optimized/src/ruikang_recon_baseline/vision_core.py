"""Pure-Python vision core used by the ROS wrappers."""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, List, Sequence

import cv2
import numpy as np

from .common import CLASS_NAMES, ConfigurationError, Detection, DetectionFrame, ModelManifest, NamedRegion, counts_template, load_manifest, validate_named_regions


class ColorBlobDetector:
    """Simple HSV blob detector for the synthetic and quick-start pipelines."""

    def __init__(self, config: Dict, class_names: Sequence[str]):
        self.config = config
        self.class_names = list(class_names)
        self.min_area = int(config.get('min_area_px', 180))
        self.max_area = int(config.get('max_area_px', 80000))
        if self.min_area <= 0 or self.max_area <= 0 or self.max_area < self.min_area:
            raise ConfigurationError('min_area_px/max_area_px are invalid')
        color_blob = config.get('color_blob', {})
        for key in ('red_1', 'red_2', 'blue', 'black'):
            if key not in color_blob:
                raise ConfigurationError('color_blob configuration missing {}'.format(key))
        self.color_blob = color_blob
        self.morph_kernel = int(color_blob.get('morph_kernel', 5))
        if self.morph_kernel <= 0 or self.morph_kernel % 2 == 0:
            raise ConfigurationError('color_blob.morph_kernel must be a positive odd integer')
        self.min_aspect_ratio = float(color_blob.get('min_aspect_ratio', 0.20))
        self.max_aspect_ratio = float(color_blob.get('max_aspect_ratio', 1.80))
        self.min_fill_ratio = float(color_blob.get('min_fill_ratio', 0.18))
        if self.min_aspect_ratio <= 0 or self.max_aspect_ratio <= 0 or self.max_aspect_ratio < self.min_aspect_ratio:
            raise ConfigurationError('color_blob aspect-ratio bounds are invalid')
        if self.min_fill_ratio < 0.0 or self.min_fill_ratio > 1.0:
            raise ConfigurationError('color_blob.min_fill_ratio must be in [0, 1]')

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
        cfg = self.color_blob
        red = cv2.bitwise_or(
            self._build_mask(hsv, cfg['red_1']['lower'], cfg['red_1']['upper']),
            self._build_mask(hsv, cfg['red_2']['lower'], cfg['red_2']['upper']),
        )
        blue = self._build_mask(hsv, cfg['blue']['lower'], cfg['blue']['upper'])
        black = self._build_mask(hsv, cfg['black']['lower'], cfg['black']['upper'])
        red = self._post_mask(red)
        blue = self._post_mask(blue)
        black = self._post_mask(black)
        detections: List[Detection] = []
        detections.extend(self._extract(red, 'friendly'))
        detections.extend(self._extract(blue, 'enemy'))
        detections.extend(self._extract(black, 'hostage'))
        return detections


class OnnxDetector:
    """Generic ONNX detector with manifest-driven parser selection."""

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
    """Assign frame-local region names to detections."""

    def __init__(self, adapter_type: str, regions: Sequence[Dict]):
        self.adapter_type = str(adapter_type).strip().lower()
        self.regions: List[NamedRegion] = validate_named_regions(regions) if regions else []

    def assign(self, detections: Sequence[Detection]) -> List[Detection]:
        if self.adapter_type in ('none', '', 'disabled'):
            return list(detections)
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
    detector_type = str(config.get('detector_type', 'color_blob')).strip().lower()
    if detector_type == 'onnx':
        return detector_type, OnnxDetector(config, class_names)
    if detector_type != 'color_blob':
        raise ConfigurationError('Unsupported detector_type: {}'.format(detector_type))
    return detector_type, ColorBlobDetector(config, class_names)


def draw_overlay(image_bgr: np.ndarray, detections: Sequence[Detection], regions: Sequence[Dict], region_counts: Dict[str, Dict[str, int]]) -> np.ndarray:
    canvas = image_bgr.copy()
    color_map = {
        'friendly': (0, 0, 255),
        'enemy': (255, 0, 0),
        'hostage': (20, 20, 20),
    }
    for region in regions:
        x0, y0, x1, y1 = int(region.get('x0', 0)), int(region.get('y0', 0)), int(region.get('x1', 0)), int(region.get('y1', 0))
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 255, 0), 2)
        label_name = str(region.get('name', 'region'))
        counts = region_counts.get(label_name, counts_template(CLASS_NAMES))
        label = '{} F:{} E:{} H:{}'.format(label_name, counts.get('friendly', 0), counts.get('enemy', 0), counts.get('hostage', 0))
        cv2.putText(canvas, label, (x0 + 4, max(18, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    for detection in detections:
        color = color_map.get(detection.class_name, (0, 255, 255))
        cv2.rectangle(canvas, (detection.x1, detection.y1), (detection.x2, detection.y2), color, 2)
        label = '{}:{:.2f}'.format(detection.class_name, detection.score)
        if detection.frame_region:
            label += ' @{}'.format(detection.frame_region)
        cv2.putText(canvas, label, (detection.x1, max(16, detection.y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return canvas

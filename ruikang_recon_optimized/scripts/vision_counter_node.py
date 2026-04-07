#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS wrapper for frame-level vision detections and optional frame-region summaries."""

from __future__ import annotations

import os
import time
from collections import deque

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ruikang_recon_baseline.common import (
    CLASS_NAMES,
    SCHEMA_VERSION,
    ConfigurationError,
    DetectionFrame,
    JsonCodec,
    expand_path,
    normalize_class_names,
    require_positive_float,
    validate_named_regions,
)
from ruikang_recon_baseline.io_core import AsyncJsonlWriter
from ruikang_recon_baseline.vision_core import FrameRegionAdapter, build_detector, draw_overlay
from ruikang_recon_baseline.msg import Detection as DetectionMsg
from ruikang_recon_baseline.msg import DetectionArray


class VisionCounterNode:
    """Publish frame detections, debug overlays and optional frame-region counts.

    The node intentionally does not convert frame-region counts into physical
    mission-zone results. Physical-zone aggregation is deferred to the mission
    manager so that task-space semantics remain independent from camera layout.
    """

    def __init__(self):
        rospy.init_node('vision_counter_node', anonymous=False)
        self.bridge = CvBridge()
        self.class_names = normalize_class_names(rospy.get_param('~classes', list(CLASS_NAMES)))
        self.config = self._read_config()
        self.output_root = expand_path(self.config['output_root'])
        os.makedirs(self.output_root, exist_ok=True)
        self.event_writer = AsyncJsonlWriter(
            path=os.path.join(self.output_root, 'vision_events.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.detector_type = ''
        self.detector = self._build_detector_with_policy()
        self.region_adapter = FrameRegionAdapter(self.config['frame_region_adapter_type'], self.config['named_regions'])
        self.region_history = {
            str(item['name']): deque(maxlen=max(1, int(self.config['stable_window'])))
            for item in self.config['named_regions']
        }
        self.last_health_publish = 0.0
        self.last_artifact_save = 0.0

        self.detections_pub = rospy.Publisher(self.config['detections_topic'], DetectionArray, queue_size=10)
        self.detections_json_pub = rospy.Publisher(self.config['detections_json_topic'], String, queue_size=10)
        self.frame_region_counts_pub = rospy.Publisher(self.config['frame_region_counts_topic'], String, queue_size=10)
        self.overlay_pub = rospy.Publisher(self.config['overlay_topic'], Image, queue_size=2)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.image_sub = rospy.Subscriber(
            self.config['camera_topic'],
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo('vision_counter_node started. detector=%s camera_topic=%s', self.detector_type, self.config['camera_topic'])

    def _read_config(self):
        """Read and validate node-private ROS parameters."""
        named_regions = rospy.get_param('~named_regions', [])
        config = {
            'camera_topic': rospy.get_param('~camera_topic', '/camera/color/image_raw'),
            'overlay_topic': rospy.get_param('~overlay_topic', '/recon/overlay/image_raw'),
            'detections_topic': rospy.get_param('~detections_topic', '/recon/detections'),
            'detections_json_topic': rospy.get_param('~detections_json_topic', '/recon/detections_json'),
            'frame_region_counts_topic': rospy.get_param('~frame_region_counts_topic', '/recon/zone_counts'),
            'health_topic': rospy.get_param('~health_topic', '/recon/health'),
            'output_root': rospy.get_param('~output_root', '~/.ros/ruikang_recon'),
            'detector_type': rospy.get_param('~detector_type', 'color_blob'),
            'onnx_model_path': rospy.get_param('~onnx_model_path', ''),
            'model_manifest_path': rospy.get_param('~model_manifest_path', ''),
            'parser_type': rospy.get_param('~parser_type', 'yolo_v5_like'),
            'input_size': int(rospy.get_param('~input_size', 640)),
            'confidence_threshold': float(rospy.get_param('~confidence_threshold', 0.35)),
            'score_threshold': float(rospy.get_param('~score_threshold', 0.25)),
            'nms_threshold': float(rospy.get_param('~nms_threshold', 0.45)),
            'min_area_px': int(rospy.get_param('~min_area_px', 180)),
            'max_area_px': int(rospy.get_param('~max_area_px', 80000)),
            'save_interval_sec': float(rospy.get_param('~save_interval_sec', 2.0)),
            'stable_window': int(rospy.get_param('~stable_window', 5)),
            'publish_debug_image': bool(rospy.get_param('~publish_debug_image', True)),
            'save_artifacts': bool(rospy.get_param('~save_artifacts', True)),
            'strict_backend': bool(rospy.get_param('~strict_backend', True)),
            'frame_region_adapter_type': rospy.get_param('~frame_region_adapter_type', 'none'),
            'named_regions': named_regions,
            'color_blob': rospy.get_param('~color_blob', {}),
            'writer_queue_size': int(rospy.get_param('~writer_queue_size', 512)),
            'writer_rotate_max_bytes': int(rospy.get_param('~writer_rotate_max_bytes', 5 * 1024 * 1024)),
            'writer_rotate_keep': int(rospy.get_param('~writer_rotate_keep', 3)),
        }
        config['input_size'] = int(require_positive_float('input_size', config['input_size']))
        config['save_interval_sec'] = require_positive_float('save_interval_sec', config['save_interval_sec'], allow_zero=True)
        if int(config['stable_window']) <= 0:
            raise ConfigurationError('stable_window must be > 0')
        if int(config['min_area_px']) <= 0 or int(config['max_area_px']) < int(config['min_area_px']):
            raise ConfigurationError('min_area_px/max_area_px are invalid')
        if config['frame_region_adapter_type'] != 'none':
            validate_named_regions(config['named_regions'])
        elif config['named_regions']:
            validate_named_regions(config['named_regions'])
        return config

    def _build_detector_with_policy(self):
        try:
            self.detector_type, detector = build_detector(self.config, self.class_names)
            return detector
        except Exception as exc:
            if self.config['detector_type'] == 'onnx' and not self.config['strict_backend']:
                rospy.logwarn('ONNX backend unavailable (%s), falling back to color_blob.', exc)
                fallback_config = dict(self.config)
                fallback_config['detector_type'] = 'color_blob'
                self.detector_type, detector = build_detector(fallback_config, self.class_names)
                return detector
            raise

    def _publish_health(self, status: str, message: str, extra: dict | None = None) -> None:
        now_sec = time.time()
        if status == 'ok' and (now_sec - self.last_health_publish) < 0.5:
            return
        payload = {
            'stamp': now_sec,
            'node': 'vision_counter_node',
            'status': status,
            'message': message,
            'schema_version': SCHEMA_VERSION,
        }
        if extra:
            payload['details'] = extra
        writer_error = self.event_writer.last_error
        if writer_error:
            payload.setdefault('details', {})['writer_error'] = writer_error
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        self.last_health_publish = now_sec

    def _publish_typed_detections(self, frame: DetectionFrame) -> None:
        msg = DetectionArray()
        msg.header.stamp = rospy.Time.from_sec(frame.stamp)
        msg.header.frame_id = frame.frame_id
        msg.source_frame = frame.frame_id
        msg.detector_type = frame.detector_type
        msg.schema_version = frame.schema_version
        msg.source_image_width = int(frame.source_image_width)
        msg.source_image_height = int(frame.source_image_height)
        for item in frame.detections:
            det_msg = DetectionMsg()
            det_msg.class_name = item.class_name
            det_msg.score = float(item.score)
            det_msg.x1 = int(item.x1)
            det_msg.y1 = int(item.y1)
            det_msg.x2 = int(item.x2)
            det_msg.y2 = int(item.y2)
            det_msg.frame_region = item.frame_region
            msg.detections.append(det_msg)
        self.detections_pub.publish(msg)

    def _region_counts_payload(self, frame: DetectionFrame) -> dict:
        counts = self.region_adapter.stable_region_counts(
            frame=frame,
            history=self.region_history,
            stable_window=int(self.config['stable_window']),
            class_names=self.class_names,
        )
        return {
            'stamp': frame.stamp,
            'frame_id': frame.frame_id,
            'schema_version': SCHEMA_VERSION,
            'frame_region_counts': counts,
            'mode': 'frame_regions',
        }

    def _save_artifacts(self, overlay_image, frame: DetectionFrame, region_payload: dict) -> None:
        if not self.config['save_artifacts']:
            return
        now_sec = time.time()
        if (now_sec - self.last_artifact_save) < self.config['save_interval_sec']:
            return
        timestamp = '{:.3f}'.format(now_sec)
        saved = cv2.imwrite(os.path.join(self.output_root, 'annotated_{}.jpg'.format(timestamp)), overlay_image)
        if not saved:
            self._publish_health('warn', 'artifact_save_failed', {'timestamp': timestamp})
        self.event_writer.write({'type': 'detections', 'payload': frame.to_dict()})
        self.event_writer.write({'type': 'frame_region_counts', 'payload': region_payload})
        self.last_artifact_save = now_sec

    def _image_callback(self, msg: Image) -> None:
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logerr_throttle(2.0, 'cv_bridge conversion failed: %s', exc)
            self._publish_health('error', 'cv_bridge_failed', {'error': str(exc)})
            return
        try:
            detections = self.detector.detect(frame_bgr)
        except Exception as exc:
            rospy.logerr_throttle(2.0, 'detector failed: %s', exc)
            self._publish_health('error', 'detector_failed', {'error': str(exc)})
            return
        detections = self.region_adapter.assign(detections)
        stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0 else rospy.Time.now().to_sec()
        frame = DetectionFrame(
            stamp=float(stamp),
            frame_id=msg.header.frame_id,
            detector_type=self.detector_type,
            schema_version=SCHEMA_VERSION,
            detections=detections,
            source_image_width=int(frame_bgr.shape[1]),
            source_image_height=int(frame_bgr.shape[0]),
        )
        region_payload = self._region_counts_payload(frame)
        overlay = draw_overlay(frame_bgr, detections, self.config['named_regions'], region_payload['frame_region_counts'])
        self._publish_typed_detections(frame)
        self.detections_json_pub.publish(String(data=JsonCodec.dumps(frame.to_dict())))
        self.frame_region_counts_pub.publish(String(data=JsonCodec.dumps(region_payload)))
        if self.config['publish_debug_image']:
            try:
                self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
            except Exception as exc:
                rospy.logwarn_throttle(2.0, 'overlay publish failed: %s', exc)
        self._save_artifacts(overlay, frame, region_payload)
        self._publish_health('ok', 'frame_processed', {'detections': len(frame.detections)})

    def _on_shutdown(self) -> None:
        self.event_writer.close()

    def spin(self) -> None:
        rospy.spin()


if __name__ == '__main__':
    try:
        node = VisionCounterNode()
        node.spin()
    except (rospy.ROSInterruptException, ConfigurationError):
        raise

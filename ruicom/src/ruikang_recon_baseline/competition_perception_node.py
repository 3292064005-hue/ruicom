#!/usr/bin/env python3
from __future__ import annotations

from typing import Dict, Optional

import rospy
from std_msgs.msg import String

from .common import ConfigurationError, Detection, DetectionFrame, JsonCodec, SCHEMA_VERSION, class_schema_hash, require_positive_float
from .competition_perception_core import CompetitionFusionConfig, CompetitionPerceptionCore, SEMANTIC_CLASS_NAMES
from .lifecycle_protocol import decode_lifecycle_control
from .lifecycle_runtime import ManagedRuntimeState
from .msg import DetectionArray, HealthState, Detection as DetectionMsg
from .time_core import NodeClock


class CompetitionPerceptionNode:
    """ROS wrapper that converts raw detector outputs into competition semantics.

    Function:
        Subscribe to the raw detector topic and publish mission-facing semantic
        detections for soldier/mine/friend-or-foe logic.

    Inputs:
        - ``~input_detections_topic`` : DetectionArray from the raw detector.
        - lifecycle commands when ``~lifecycle_managed`` is enabled.

    Outputs:
        - ``~output_detections_topic`` : DetectionArray with semantic classes.
        - ``~health_topic`` / ``~health_typed_topic``.

    Exceptions:
        - ConfigurationError when thresholds or topic names are invalid.

    Boundary behavior:
        - When lifecycle-managed and inactive, inputs are ignored but health is
          still published so system_manager can reason about readiness.
    """

    def __init__(self):
        rospy.init_node('competition_perception_node', anonymous=False)
        self.config = self._read_config()
        self.clock = NodeClock(self.config['time_source_mode'])
        self.runtime = ManagedRuntimeState(lifecycle_managed=self.config['lifecycle_managed'])
        fusion_config = CompetitionFusionConfig(
            semantic_class_names=tuple(self.config['semantic_class_names']),
            stability_window=self.config['stability_window'],
            min_confirmed_frames=self.config['min_confirmed_frames'],
            marker_overlap_iou=self.config['marker_overlap_iou'],
            marker_center_margin_px=self.config['marker_center_margin_px'],
            mine_ground_band_ratio=self.config['mine_ground_band_ratio'],
            mine_max_height_ratio=self.config['mine_max_height_ratio'],
            mine_min_width_px=self.config['mine_min_width_px'],
            soldier_min_height_px=self.config['soldier_min_height_px'],
        )
        self.core = CompetitionPerceptionCore(fusion_config)
        self.class_names = list(fusion_config.semantic_class_names)
        self.class_schema_hash = class_schema_hash(self.class_names)
        self._last_health_signature = ''
        self._last_health_emit_sec = 0.0
        self._last_input_sec = 0.0
        self.output_pub = rospy.Publisher(self.config['output_detections_topic'], DetectionArray, queue_size=20)
        self.health_pub = rospy.Publisher(self.config['health_topic'], String, queue_size=10)
        self.health_typed_pub = rospy.Publisher(self.config['health_typed_topic'], HealthState, queue_size=10)
        self.input_sub = rospy.Subscriber(self.config['input_detections_topic'], DetectionArray, self._detections_cb, queue_size=20)
        self.control_sub = rospy.Subscriber(self.config['control_command_topic'], String, self._control_cb, queue_size=20) if self.config['lifecycle_managed'] else None
        self._publish_health('ok', 'competition_perception_ready', self._health_details())

    def _read_config(self) -> dict:
        config = {
            'input_detections_topic': str(rospy.get_param('~input_detections_topic', 'recon/detections')).strip() or 'recon/detections',
            'output_detections_topic': str(rospy.get_param('~output_detections_topic', 'recon/competition_detections')).strip() or 'recon/competition_detections',
            'health_topic': str(rospy.get_param('~health_topic', 'recon/health')).strip() or 'recon/health',
            'health_typed_topic': str(rospy.get_param('~health_typed_topic', 'recon/health_typed')).strip() or 'recon/health_typed',
            'health_frame_id': str(rospy.get_param('~health_frame_id', 'map')).strip() or 'map',
            'semantic_class_names': rospy.get_param('~semantic_class_names', list(SEMANTIC_CLASS_NAMES)),
            'stability_window': int(rospy.get_param('~stability_window', 5) or 5),
            'min_confirmed_frames': int(rospy.get_param('~min_confirmed_frames', 3) or 3),
            'marker_overlap_iou': float(rospy.get_param('~marker_overlap_iou', 0.08) or 0.08),
            'marker_center_margin_px': int(rospy.get_param('~marker_center_margin_px', 24) or 24),
            'mine_ground_band_ratio': float(rospy.get_param('~mine_ground_band_ratio', 0.58) or 0.58),
            'mine_max_height_ratio': float(rospy.get_param('~mine_max_height_ratio', 0.24) or 0.24),
            'mine_min_width_px': int(rospy.get_param('~mine_min_width_px', 8) or 8),
            'soldier_min_height_px': int(rospy.get_param('~soldier_min_height_px', 26) or 26),
            'health_heartbeat_hz': float(rospy.get_param('~health_heartbeat_hz', 1.0) or 1.0),
            'input_freshness_sec': float(rospy.get_param('~input_freshness_sec', 1.5) or 1.5),
            'time_source_mode': str(rospy.get_param('~time_source_mode', 'ros')).strip().lower() or 'ros',
            'lifecycle_managed': bool(rospy.get_param('~lifecycle_managed', False)),
            'control_command_topic': str(rospy.get_param('~control_command_topic', 'recon/system_manager/command')).strip() or 'recon/system_manager/command',
            'runtime_grade': str(rospy.get_param('~runtime_grade', 'competition')).strip().lower() or 'competition',
        }
        config['health_heartbeat_hz'] = require_positive_float('competition_perception.health_heartbeat_hz', config['health_heartbeat_hz'])
        config['input_freshness_sec'] = require_positive_float('competition_perception.input_freshness_sec', config['input_freshness_sec'])
        if config['time_source_mode'] not in ('ros', 'wall'):
            raise ConfigurationError('competition_perception_node time_source_mode must be ros or wall')
        return config

    @staticmethod
    def _connections(endpoint) -> int:
        getter = getattr(endpoint, 'get_num_connections', None)
        if getter is None:
            return 0
        try:
            return int(getter())
        except Exception:
            return 0

    def _health_details(self) -> Dict[str, object]:
        now = self.clock.now_business_sec()
        return {
            'runtime_grade': self.config['runtime_grade'],
            'input_detections_topic_declared': bool(self.config['input_detections_topic']),
            'output_detections_topic_declared': bool(self.config['output_detections_topic']),
            'input_topic_bound': self._connections(self.input_sub) > 0,
            'output_topic_bound': self._connections(self.output_pub) > 0,
            'input_fresh': self._last_input_sec > 0.0 and (now - self._last_input_sec) <= self.config['input_freshness_sec'],
            'semantic_catalog_ready': True,
            **self.runtime.snapshot(),
        }

    def _publish_health(self, status: str, message: str, details: Optional[dict] = None) -> None:
        payload = {
            'stamp': self.clock.now_business_sec(),
            'node': 'competition_perception_node',
            'status': str(status).strip(),
            'message': str(message).strip(),
            'schema_version': SCHEMA_VERSION,
            'details': dict(details or {}),
        }
        self.health_pub.publish(String(data=JsonCodec.dumps(payload)))
        msg = HealthState()
        msg.header.stamp = self.clock.now_ros_time()
        msg.header.frame_id = self.config['health_frame_id']
        msg.node = payload['node']
        msg.status = payload['status']
        msg.message = payload['message']
        msg.schema_version = payload['schema_version']
        msg.details_json = JsonCodec.dumps(payload['details'])
        self.health_typed_pub.publish(msg)

    @staticmethod
    def _frame_from_msg(msg: DetectionArray) -> DetectionFrame:
        detections = [
            Detection(
                class_name=item.class_name,
                score=float(item.score),
                x1=int(item.x1),
                y1=int(item.y1),
                x2=int(item.x2),
                y2=int(item.y2),
                frame_region=str(item.frame_region),
                observed_position_type=str(item.observed_position_type),
                observed_position_label=str(item.observed_position_label),
                observed_position_x_m=float(item.observed_position_x_m),
                observed_position_y_m=float(item.observed_position_y_m),
                evidence_source=str(item.evidence_source),
            )
            for item in msg.detections
        ]
        return DetectionFrame(
            stamp=float(msg.header.stamp.to_sec()),
            frame_id=str(msg.source_frame),
            detector_type=str(msg.detector_type),
            schema_version=str(msg.schema_version),
            detections=detections,
            source_image_width=int(msg.source_image_width),
            source_image_height=int(msg.source_image_height),
            class_names=list(msg.class_names),
            class_schema_hash=str(msg.class_schema_hash),
        )

    def _publish_detection_frame(self, frame: DetectionFrame) -> None:
        msg = DetectionArray()
        msg.header.stamp = self.clock.now_ros_time()
        msg.source_frame = str(frame.frame_id)
        msg.detector_type = str(frame.detector_type)
        msg.schema_version = str(frame.schema_version)
        msg.source_image_width = int(frame.source_image_width)
        msg.source_image_height = int(frame.source_image_height)
        msg.class_names = list(frame.class_names or self.class_names)
        msg.class_schema_hash = class_schema_hash(msg.class_names)
        for det in frame.detections:
            item = DetectionMsg()
            item.class_name = det.class_name
            item.score = float(det.score)
            item.x1 = int(det.x1)
            item.y1 = int(det.y1)
            item.x2 = int(det.x2)
            item.y2 = int(det.y2)
            item.frame_region = det.frame_region
            item.observed_position_type = det.observed_position_type
            item.observed_position_label = det.observed_position_label
            item.observed_position_x_m = float(det.observed_position_x_m)
            item.observed_position_y_m = float(det.observed_position_y_m)
            item.evidence_source = det.evidence_source
            msg.detections.append(item)
        self.output_pub.publish(msg)

    def _detections_cb(self, msg: DetectionArray) -> None:
        self._last_input_sec = self.clock.now_business_sec()
        if self.runtime.lifecycle_managed and not self.runtime.is_active:
            return
        frame = self._frame_from_msg(msg)
        semantic = self.core.process(frame)
        self._publish_detection_frame(semantic)

    def _control_cb(self, msg: String) -> None:
        control = decode_lifecycle_control(msg.data)
        if control.target and control.target != 'competition_perception_node':
            return
        changed = self.runtime.apply(control.command)
        if changed:
            self._publish_health('ok', 'runtime_state_changed', self._health_details())

    def spin(self) -> None:
        rate = rospy.Rate(max(1.0, self.config['health_heartbeat_hz'] * 10.0))
        while not rospy.is_shutdown():
            details = self._health_details()
            signature = JsonCodec.dumps(details)
            now = self.clock.now_business_sec()
            if signature != self._last_health_signature or (now - self._last_health_emit_sec) >= (1.0 / self.config['health_heartbeat_hz']):
                self._publish_health('ok', 'competition_perception_ready', details)
                self._last_health_signature = signature
                self._last_health_emit_sec = now
            rate.sleep()


if __name__ == '__main__':
    try:
        CompetitionPerceptionNode().spin()
    except rospy.ROSInterruptException:
        pass

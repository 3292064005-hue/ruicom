#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deterministic typed detection publisher for integration validation."""

from __future__ import annotations

import rospy

from ruikang_recon_baseline.common import class_schema_hash, coerce_class_count_mapping, validate_dynamic_class_names
from ruikang_recon_baseline.msg import Detection, DetectionArray


class FakeDetectionArrayPublisher:
    """Publish a stable typed detection frame at a configurable rate.

    The fixture emits the same embedded schema metadata that the real mission
    contract requires: ordered ``class_names`` plus ``class_schema_hash``.
    Integration tests may also override or omit the embedded metadata in order
    to exercise schema-mismatch paths explicitly without introducing a separate
    one-off publisher implementation.
    """

    def __init__(self):
        rospy.init_node('fake_detection_array_publisher', anonymous=False)
        self.topic = str(rospy.get_param('~detections_topic', 'recon/detections')).strip() or 'recon/detections'
        self.frame_id = str(rospy.get_param('~frame_id', 'synthetic_camera')).strip() or 'synthetic_camera'
        self.detector_type = str(rospy.get_param('~detector_type', 'integration_fixture')).strip() or 'integration_fixture'
        self.schema_version = str(rospy.get_param('~schema_version', '2.2.0')).strip() or '2.2.0'
        self.publish_hz = float(rospy.get_param('~publish_hz', 10.0))
        self.source_image_width = int(rospy.get_param('~source_image_width', 640))
        self.source_image_height = int(rospy.get_param('~source_image_height', 480))
        self.frame_region = str(rospy.get_param('~frame_region', '')).strip()
        raw_counts = rospy.get_param('~count_by_class', {'friendly': 2, 'enemy': 1, 'hostage': 1})
        raw_class_names = rospy.get_param('~class_names', list(raw_counts.keys()) if isinstance(raw_counts, dict) else ['friendly', 'enemy', 'hostage'])
        self.omit_embedded_class_names = bool(rospy.get_param('~omit_embedded_class_names', False))
        self.omit_embedded_class_schema_hash = bool(rospy.get_param('~omit_embedded_class_schema_hash', False))
        raw_embedded_class_names = rospy.get_param('~embedded_class_names', raw_class_names)
        raw_embedded_class_schema_hash = rospy.get_param('~embedded_class_schema_hash', '')
        self.observed_position_type = str(rospy.get_param('~observed_position_type', 'frame_region')).strip() or 'frame_region'
        self.observed_position_label = str(rospy.get_param('~observed_position_label', self.frame_region)).strip()
        self.observed_position_x_m = float(rospy.get_param('~observed_position_x_m', 0.0))
        self.observed_position_y_m = float(rospy.get_param('~observed_position_y_m', 0.0))
        self.evidence_source = str(rospy.get_param('~evidence_source', self.detector_type)).strip() or self.detector_type
        if self.publish_hz <= 0.0:
            raise ValueError('publish_hz must be > 0')
        if not isinstance(raw_counts, dict):
            raise ValueError('count_by_class must be a mapping')
        self.class_names = list(validate_dynamic_class_names(raw_class_names, owner='fake_detection_array_publisher.class_names'))
        self.counts = coerce_class_count_mapping(raw_counts, self.class_names)
        self.embedded_class_names = [] if self.omit_embedded_class_names else list(
            validate_dynamic_class_names(raw_embedded_class_names, owner='fake_detection_array_publisher.embedded_class_names')
        )
        derived_embedded_hash = class_schema_hash(self.embedded_class_names) if self.embedded_class_names else ''
        self.embedded_class_schema_hash = '' if self.omit_embedded_class_schema_hash else (str(raw_embedded_class_schema_hash).strip() or derived_embedded_hash)
        self.publisher = rospy.Publisher(self.topic, DetectionArray, queue_size=10)
        rospy.loginfo(
            'fake_detection_array_publisher started. topic=%s class_names=%s embedded_class_names=%s omit_names=%s omit_hash=%s',
            self.topic,
            self.class_names,
            self.embedded_class_names,
            self.omit_embedded_class_names,
            self.omit_embedded_class_schema_hash,
        )

    def _build_message(self) -> DetectionArray:
        """Build one deterministic detection frame including embedded schema metadata.

        Args:
            None. All values come from validated node configuration.

        Returns:
            A ``DetectionArray`` compatible with mission schema validation.

        Raises:
            No explicit exception is raised. Configuration validation happens during
            node construction.

        Boundary behavior:
            Missing classes in ``count_by_class`` are projected to zero using the
            authoritative detection-class ordering declared by ``class_names``. The
            embedded transport schema may intentionally differ when a mismatch test
            needs to exercise the mission gate.
        """
        msg = DetectionArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.source_frame = self.frame_id
        msg.detector_type = self.detector_type
        msg.schema_version = self.schema_version
        msg.class_names = list(self.embedded_class_names)
        msg.class_schema_hash = str(self.embedded_class_schema_hash)
        msg.source_image_width = max(0, self.source_image_width)
        msg.source_image_height = max(0, self.source_image_height)
        x_cursor = 20
        row_offsets = {name: 20 + index * 80 for index, name in enumerate(self.class_names)}
        for class_name in self.class_names:
            normalized_count = max(0, int(self.counts.get(class_name, 0)))
            for idx in range(normalized_count):
                det = Detection()
                det.class_name = str(class_name)
                det.score = 0.95
                det.x1 = x_cursor + idx * 30
                det.y1 = row_offsets.get(str(class_name), 20)
                det.x2 = det.x1 + 18
                det.y2 = det.y1 + 40
                det.frame_region = self.frame_region
                det.observed_position_type = self.observed_position_type
                det.observed_position_label = self.observed_position_label or self.frame_region
                det.observed_position_x_m = float(self.observed_position_x_m)
                det.observed_position_y_m = float(self.observed_position_y_m)
                det.evidence_source = self.evidence_source
                msg.detections.append(det)
            x_cursor += 40
        return msg

    def spin(self) -> None:
        """Continuously publish the configured deterministic detection frame."""
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            self.publisher.publish(self._build_message())
            rate.sleep()


if __name__ == '__main__':
    try:
        node = FakeDetectionArrayPublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass

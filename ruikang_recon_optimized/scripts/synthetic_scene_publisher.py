#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Synthetic top-view image source for end-to-end demo validation."""

from __future__ import annotations

import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class SyntheticScenePublisher:
    """Publish a deterministic multi-region synthetic scene."""

    def __init__(self):
        rospy.init_node('synthetic_scene_publisher', anonymous=False)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/color/image_raw')
        self.width = int(rospy.get_param('~width', 640))
        self.height = int(rospy.get_param('~height', 480))
        self.fps = float(rospy.get_param('~fps', 5.0))
        self.named_regions = rospy.get_param('~named_regions', [])
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.camera_topic, Image, queue_size=2)
        self.frame_index = 0
        rospy.loginfo('synthetic_scene_publisher started. camera_topic=%s', self.camera_topic)

    @staticmethod
    def _class_color(class_name: str):
        if class_name == 'friendly':
            return (0, 0, 255)
        if class_name == 'enemy':
            return (255, 0, 0)
        return (20, 20, 20)

    @staticmethod
    def _draw_person(canvas, cx: int, cy: int, color):
        head_r = 8
        body_w = 16
        body_h = 30
        leg_h = 14
        cv2.circle(canvas, (cx, cy - body_h // 2 - head_r), head_r, color, thickness=-1)
        cv2.rectangle(canvas, (cx - body_w // 2, cy - body_h // 2), (cx + body_w // 2, cy + body_h // 2), color, thickness=-1)
        cv2.line(canvas, (cx - 4, cy + body_h // 2), (cx - 8, cy + body_h // 2 + leg_h), color, thickness=4)
        cv2.line(canvas, (cx + 4, cy + body_h // 2), (cx + 8, cy + body_h // 2 + leg_h), color, thickness=4)

    def _layout_positions(self, region: dict, count: int):
        x0, y0, x1, y1 = int(region['x0']), int(region['y0']), int(region['x1']), int(region['y1'])
        inner_w = max(30, x1 - x0 - 40)
        inner_h = max(30, y1 - y0 - 40)
        cols = max(1, int(math.ceil(math.sqrt(max(1, count)))))
        rows = max(1, int(math.ceil(float(max(1, count)) / cols)))
        positions = []
        idx = 0
        for row in range(rows):
            for col in range(cols):
                if idx >= count:
                    break
                px = x0 + 25 + int((col + 0.5) * inner_w / cols)
                py = y0 + 35 + int((row + 0.5) * inner_h / rows)
                wobble_x = int(5 * math.sin(0.15 * self.frame_index + idx))
                wobble_y = int(4 * math.cos(0.18 * self.frame_index + idx))
                positions.append((px + wobble_x, py + wobble_y))
                idx += 1
        return positions

    def _render(self):
        image = np.full((self.height, self.width, 3), 245, dtype=np.uint8)
        for region in self.named_regions:
            x0, y0, x1, y1 = int(region['x0']), int(region['y0']), int(region['x1']), int(region['y1'])
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 180, 0), 2)
            title = str(region.get('name', 'region'))
            cv2.putText(image, title, (x0 + 8, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 120, 0), 2, cv2.LINE_AA)
            for class_name in ('friendly', 'enemy', 'hostage'):
                count = int(region.get(class_name, 0))
                for cx, cy in self._layout_positions(region, count):
                    self._draw_person(image, cx, cy, self._class_color(class_name))
        cv2.putText(image, 'Synthetic Recon Scene', (15, self.height - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 50, 50), 2, cv2.LINE_AA)
        return image

    def spin(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            frame = self._render()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'synthetic_camera'
            self.publisher.publish(msg)
            self.frame_index += 1
            rate.sleep()


if __name__ == '__main__':
    try:
        node = SyntheticScenePublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass

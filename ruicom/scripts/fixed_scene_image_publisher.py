#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FixedSceneImagePublisher:
    def __init__(self):
        rospy.init_node('fixed_scene_image_publisher', anonymous=False)
        self.camera_topic = str(rospy.get_param('~camera_topic', '/camera/rgb/image_raw')).strip() or '/camera/rgb/image_raw'
        self.publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 8.0) or 8.0)
        self.width = int(rospy.get_param('~width', 640) or 640)
        self.height = int(rospy.get_param('~height', 480) or 480)
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.camera_topic, Image, queue_size=2)
        self.frame = self._build_frame(self.width, self.height)
    @staticmethod
    def _build_frame(width: int, height: int):
        image = np.full((height, width, 3), 255, dtype=np.uint8)
        cv2.putText(image, 'Fixed Recon Scene', (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (32,32,32), 2, cv2.LINE_AA)
        # draw one enemy-like green silhouette similar to synthetic_scene_publisher
        cx, cy = int(width * 0.55), int(height * 0.52)
        body_w, body_h, head_r = 40, 90, 18
        color = (0, 190, 0)
        cv2.circle(image, (cx, cy - body_h // 2 - head_r), head_r, color, thickness=-1)
        cv2.rectangle(image, (cx - body_w // 2, cy - body_h // 2), (cx + body_w // 2, cy + body_h // 2), color, thickness=-1)
        cv2.line(image, (cx - 6, cy + body_h // 2), (cx - 14, cy + body_h // 2 + 26), color, thickness=5)
        cv2.line(image, (cx + 6, cy + body_h // 2), (cx + 14, cy + body_h // 2 + 26), color, thickness=5)
        return image
    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            self.publisher.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8'))
            rate.sleep()

if __name__ == '__main__':
    FixedSceneImagePublisher().spin()

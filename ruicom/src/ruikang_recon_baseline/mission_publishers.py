"""Mission-node publisher bundle construction."""

from __future__ import annotations

from dataclasses import dataclass

import rospy
from std_msgs.msg import String

from .msg import HealthState, MissionState, ZoneCapture, ZoneCaptureDynamic


@dataclass(frozen=True)
class MissionPublisherBundle:
    current_zone: rospy.Publisher
    mission_state_json: rospy.Publisher
    mission_state_typed: rospy.Publisher
    zone_capture_json: rospy.Publisher
    zone_capture_typed: rospy.Publisher
    zone_capture_dynamic: rospy.Publisher
    health_json: rospy.Publisher
    health_typed: rospy.Publisher



def build_mission_publishers(config: dict) -> MissionPublisherBundle:
    return MissionPublisherBundle(
        current_zone=rospy.Publisher(config['current_zone_topic'], String, queue_size=10, latch=True),
        mission_state_json=rospy.Publisher(config['mission_state_topic'], String, queue_size=20, latch=True),
        mission_state_typed=rospy.Publisher(config['mission_state_typed_topic'], MissionState, queue_size=20, latch=True),
        zone_capture_json=rospy.Publisher(config['zone_capture_topic'], String, queue_size=20, latch=True),
        zone_capture_typed=rospy.Publisher(config['zone_capture_typed_topic'], ZoneCapture, queue_size=20, latch=True),
        zone_capture_dynamic=rospy.Publisher(config['zone_capture_dynamic_topic'], ZoneCaptureDynamic, queue_size=20, latch=True),
        health_json=rospy.Publisher(config['health_topic'], String, queue_size=20),
        health_typed=rospy.Publisher(config['health_typed_topic'], HealthState, queue_size=20),
    )

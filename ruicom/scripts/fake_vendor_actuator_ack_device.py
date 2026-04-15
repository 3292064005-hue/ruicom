#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import rospy
from std_msgs.msg import String
from ruikang_recon_baseline.common import JsonCodec

class FakeVendorActuatorAckDevice:
    def __init__(self):
        rospy.init_node('fake_vendor_actuator_ack_device', anonymous=False)
        self.command_topic = str(rospy.get_param('~command_topic', 'recon/platform/vendor/actuator_command_raw')).strip() or 'recon/platform/vendor/actuator_command_raw'
        self.ack_topic = str(rospy.get_param('~ack_topic', 'recon/platform/vendor/actuator_device_ack')).strip() or 'recon/platform/vendor/actuator_device_ack'
        self.completion_delay_sec = float(rospy.get_param('~completion_delay_sec', 0.15) or 0.15)
        self.status = str(rospy.get_param('~status', 'SUCCEEDED')).strip().upper() or 'SUCCEEDED'
        self.pub = rospy.Publisher(self.ack_topic, String, queue_size=20)
        self.sub = rospy.Subscriber(self.command_topic, String, self._cb, queue_size=20)
    def _emit(self, payload):
        msg = {
            'command_id': str(payload.get('command_id', '')).strip(),
            'status': self.status,
            'stamp': rospy.get_time(),
            'details': {
                'device': 'fake_vendor_actuator_ack_device',
                'action_type': str(payload.get('action_type', '')).strip(),
            },
        }
        self.pub.publish(String(data=JsonCodec.dumps(msg)))
    def _cb(self, msg: String):
        try:
            payload = JsonCodec.loads(msg.data)
        except Exception:
            return
        rospy.Timer(rospy.Duration(self.completion_delay_sec), lambda _evt, p=payload: self._emit(p), oneshot=True)
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    FakeVendorActuatorAckDevice().spin()

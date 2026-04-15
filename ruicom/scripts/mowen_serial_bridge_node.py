#!/usr/bin/env python3
from ruikang_recon_baseline.mowen_serial_bridge_node import MowenSerialBridgeNode
import rospy

if __name__ == '__main__':
    try:
        MowenSerialBridgeNode().spin()
    except rospy.ROSInterruptException:
        pass

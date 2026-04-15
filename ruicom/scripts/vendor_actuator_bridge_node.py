#!/usr/bin/env python3
from ruikang_recon_baseline.vendor_actuator_bridge_node import VendorActuatorBridgeNode

if __name__ == '__main__':
    try:
        VendorActuatorBridgeNode().spin()
    except Exception as exc:
        import rospy
        raise

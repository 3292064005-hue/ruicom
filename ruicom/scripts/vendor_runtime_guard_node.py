#!/usr/bin/env python3
from ruikang_recon_baseline.vendor_runtime_guard_node import VendorRuntimeGuardNode
import rospy

if __name__ == '__main__':
    try:
        VendorRuntimeGuardNode().spin()
    except rospy.ROSInterruptException:
        pass

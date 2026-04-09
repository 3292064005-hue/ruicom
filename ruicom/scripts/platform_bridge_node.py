#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the vendor platform bridge node."""

from ruikang_recon_baseline.platform_bridge_node import PlatformBridgeNode


if __name__ == '__main__':
    PlatformBridgeNode().spin()

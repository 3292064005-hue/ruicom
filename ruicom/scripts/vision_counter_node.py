#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the field-asset-aware vision node."""

from ruikang_recon_baseline.vision_node import VisionCounterNode


if __name__ == '__main__':
    VisionCounterNode().spin()

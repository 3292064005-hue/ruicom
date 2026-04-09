#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the lifecycle-aware system manager node."""

from ruikang_recon_baseline.system_manager_node import SystemManagerNode


if __name__ == '__main__':
    SystemManagerNode().spin()

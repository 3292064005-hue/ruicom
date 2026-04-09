#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the executor-based mission manager node."""

from ruikang_recon_baseline.mission_node import MissionManagerNode


if __name__ == '__main__':
    MissionManagerNode().spin()

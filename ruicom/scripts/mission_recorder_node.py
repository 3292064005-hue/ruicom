#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the staged mission recorder node."""

from ruikang_recon_baseline.recorder_node import MissionRecorderNode


if __name__ == '__main__':
    MissionRecorderNode().spin()

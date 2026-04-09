#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the lifecycle-aware safety mux node."""

from ruikang_recon_baseline.safety_node import CmdSafetyMuxNode


if __name__ == '__main__':
    CmdSafetyMuxNode().spin()

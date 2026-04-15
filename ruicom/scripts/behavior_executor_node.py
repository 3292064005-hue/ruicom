#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the deploy behavior executor node."""

from ruikang_recon_baseline.behavior_executor_node import BehaviorExecutorNode


if __name__ == '__main__':
    BehaviorExecutorNode().spin()

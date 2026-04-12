#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entrypoint for the explicit vendor feedback adapter."""

from ruikang_recon_baseline.vendor_feedback_node import VendorFeedbackNode


if __name__ == '__main__':
    VendorFeedbackNode().spin()

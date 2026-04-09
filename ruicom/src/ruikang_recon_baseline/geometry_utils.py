"""Geometry and numeric helper functions."""

from __future__ import annotations

import math


def clamp(value: float, limit: float) -> float:
    return max(-float(limit), min(float(limit), float(value)))



def yaw_deg_to_quaternion_tuple(yaw_deg: float):
    half = math.radians(float(yaw_deg)) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))



def quaternion_to_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cosy_cosp = 1.0 - 2.0 * (float(y) * float(y) + float(z) * float(z))
    return math.atan2(siny_cosp, cosy_cosp)

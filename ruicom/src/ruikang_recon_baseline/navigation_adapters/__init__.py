"""Navigation adapter package.

This module intentionally exposes only ROS-free symbols at import time so unit
and contract tests can resolve navigation capability metadata without requiring
full ROS actionlib/message dependencies.
"""

from .base import NavigationAdapterBase
from .registry import (
    NavigationAdapterCapability,
    NavigationAdapterRegistry,
    build_navigation_registry,
    describe_navigation_capability,
)

__all__ = [
    'NavigationAdapterBase',
    'NavigationAdapterCapability',
    'NavigationAdapterRegistry',
    'build_navigation_registry',
    'describe_navigation_capability',
]

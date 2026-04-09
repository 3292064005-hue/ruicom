"""Declarative platform-adapter capability registry."""

from .base import PlatformAdapterCapability
from .registry import (
    PlatformAdapterRegistry,
    apply_platform_mission_defaults,
    apply_platform_safety_defaults,
    apply_platform_vision_defaults,
)
from ..platform_contracts import validate_platform_contract_bindings, validate_platform_runtime_strategy

__all__ = [
    'PlatformAdapterCapability',
    'PlatformAdapterRegistry',
    'apply_platform_mission_defaults',
    'apply_platform_safety_defaults',
    'apply_platform_vision_defaults',
    'validate_platform_contract_bindings',
    'validate_platform_runtime_strategy',
]

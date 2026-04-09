"""Registry for declarative platform adapters."""

from __future__ import annotations

from typing import Dict

from ..domain_models import ConfigurationError
from .base import PlatformAdapterCapability
from .generic_ros import GENERIC_ROS_PLATFORM
from .mowen_mo_sergeant import MOWEN_MO_SERGEANT_PLATFORM


class PlatformAdapterRegistry:
    def __init__(self, capabilities: Dict[str, PlatformAdapterCapability] | None = None):
        capabilities = capabilities or {
            GENERIC_ROS_PLATFORM.name: GENERIC_ROS_PLATFORM,
            MOWEN_MO_SERGEANT_PLATFORM.name: MOWEN_MO_SERGEANT_PLATFORM,
        }
        self.capabilities = {str(name).strip().lower(): value for name, value in capabilities.items()}

    def resolve(self, adapter_type: str) -> PlatformAdapterCapability:
        normalized = str(adapter_type or '').strip().lower() or GENERIC_ROS_PLATFORM.name
        if normalized not in self.capabilities:
            raise ConfigurationError('Unsupported platform_adapter_type: {}'.format(adapter_type))
        return self.capabilities[normalized]



def _apply_defaults(config: dict, defaults: dict) -> None:
    for key, value in defaults.items():
        current = config.get(key)
        if current in (None, ''):
            config[key] = value



def _apply_capability_defaults(config: dict, capability: PlatformAdapterCapability, *, domain_defaults: dict) -> None:
    _apply_defaults(config, dict(capability.shared_defaults))
    _apply_defaults(config, dict(domain_defaults))
    config['platform_adapter_type'] = capability.name
    config['platform_required_topics'] = list(capability.required_topics)
    config['platform_required_actions'] = list(capability.required_actions)



def apply_platform_mission_defaults(config: dict) -> PlatformAdapterCapability:
    capability = PlatformAdapterRegistry().resolve(config.get('platform_adapter_type', GENERIC_ROS_PLATFORM.name))
    _apply_capability_defaults(config, capability, domain_defaults=dict(capability.mission_defaults))
    return capability



def apply_platform_vision_defaults(config: dict) -> PlatformAdapterCapability:
    capability = PlatformAdapterRegistry().resolve(config.get('platform_adapter_type', GENERIC_ROS_PLATFORM.name))
    _apply_capability_defaults(config, capability, domain_defaults=dict(capability.vision_defaults))
    return capability



def apply_platform_safety_defaults(config: dict) -> PlatformAdapterCapability:
    """Apply platform-specific safety defaults.

    Side effects:
    - populates safety-facing topic defaults such as ``input_topic`` and
      ``output_feedback_topic`` when omitted by the caller.
    - mirrors the normalized safety ingress into ``command_input_topic`` so the
      platform contract validator and platform bridge share a single source of
      truth.

    Args:
        config: Mutable configuration dictionary for the safety component.

    Returns:
        PlatformAdapterCapability: The resolved platform capability descriptor.

    Boundary behavior:
        Empty values are treated as unspecified and therefore backfilled from
        platform defaults. Existing explicit values are preserved.
    """
    capability = PlatformAdapterRegistry().resolve(config.get('platform_adapter_type', GENERIC_ROS_PLATFORM.name))
    _apply_capability_defaults(config, capability, domain_defaults=dict(capability.safety_defaults))
    command_input_topic = str(config.get('command_input_topic', '') or '').strip()
    input_topic = str(config.get('input_topic', '') or '').strip()
    if not command_input_topic and input_topic:
        config['command_input_topic'] = input_topic
    if config.get('profile_role') == 'deploy' and capability.safety_defaults.get('require_output_feedback'):
        config['require_output_feedback'] = True
    return capability

"""Capability registry and shared factory helpers for navigation adapters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Dict, Mapping, Optional

from ..common import ConfigurationError
from ..navigation_runtime import CompositeNavigationAdapter, NavigationRuntimeBinding, NoOpCanceller

if TYPE_CHECKING:
    from ..mission_core import ArrivalEvaluator
    from ..time_core import NodeClock


@dataclass(frozen=True)
class NavigationAdapterCapability:
    """Static capability description for one navigation backend."""

    name: str
    status_source: str
    supports_cancel: bool
    requires_external_status: bool
    description: str
    binding_builder: Callable[[], NavigationRuntimeBinding]
    planner_interface: str = 'goal_dispatch'
    controller_interface: str = 'goal_dispatch'
    recovery_interface: str = 'executor_policy'
    localization_interface: str = 'external_pose_source'
    backend_profile: str = 'generic_goal_dispatch'
    goal_transport: str = 'unknown'
    status_transport: str = 'unknown'
    cancel_transport: str = 'none'


class NavigationAdapterRegistry:
    """Thin registry wrapper around navigation role factories."""

    def __init__(self, capabilities: Mapping[str, NavigationAdapterCapability]):
        self.capabilities = {str(name).strip().lower(): capability for name, capability in capabilities.items()}

    def build(self, adapter_type: str):
        """Build a composed adapter and return the instance plus its static capability."""
        normalized = str(adapter_type).strip().lower()
        if normalized not in self.capabilities:
            raise ConfigurationError('Unsupported navigation_adapter_type: {}'.format(adapter_type))
        capability = self.capabilities[normalized]
        binding = capability.binding_builder()
        return CompositeNavigationAdapter(binding), capability

    def describe(self, adapter_type: str) -> Dict[str, object]:
        """Return a JSON-serializable capability mapping for diagnostics."""
        normalized = str(adapter_type).strip().lower()
        if normalized not in self.capabilities:
            raise ConfigurationError('Unsupported navigation_adapter_type: {}'.format(adapter_type))
        capability = self.capabilities[normalized]
        return {
            'name': capability.name,
            'status_source': capability.status_source,
            'supports_cancel': capability.supports_cancel,
            'requires_external_status': capability.requires_external_status,
            'description': capability.description,
            'planner_interface': capability.planner_interface,
            'controller_interface': capability.controller_interface,
            'recovery_interface': capability.recovery_interface,
            'localization_interface': capability.localization_interface,
            'backend_profile': capability.backend_profile,
            'goal_transport': capability.goal_transport,
            'status_transport': capability.status_transport,
            'cancel_transport': capability.cancel_transport,
        }


def _builders_disabled(name: str):
    def _raise_disabled():
        raise ConfigurationError('Navigation adapter builder disabled for contract-only capability resolution: {}'.format(name))
    return _raise_disabled


def _build_move_base_binding(config: Mapping[str, object], clock: 'NodeClock') -> NavigationRuntimeBinding:
    from .move_base import MoveBaseActionAdapter

    backend = MoveBaseActionAdapter(
        action_name=config['move_base_action_name'],
        wait_for_server_sec=config['wait_for_action_server_sec'],
        clock=clock,
    )
    return NavigationRuntimeBinding(dispatcher=backend, status_monitor=backend, canceller=backend)


def _build_simple_topic_binding(
    config: Mapping[str, object],
    arrival_evaluator: 'ArrivalEvaluator',
    clock: 'NodeClock',
) -> NavigationRuntimeBinding:
    from .simple_topic import SimpleGoalTopicAdapter

    backend = SimpleGoalTopicAdapter(
        goal_topic=config['simple_goal_topic'],
        arrival_evaluator=arrival_evaluator,
        clock=clock,
        simulate_arrival_without_pose=config['simulate_arrival_without_pose'],
        synthetic_arrival_delay_sec=config['synthetic_arrival_delay_sec'],
    )
    return NavigationRuntimeBinding(dispatcher=backend, status_monitor=backend, canceller=NoOpCanceller(on_cancel=backend.cancel))


def _build_goal_topic_status_binding(config: Mapping[str, object], clock: 'NodeClock') -> NavigationRuntimeBinding:
    from .status_topic import GoalTopicStatusAdapter

    backend = GoalTopicStatusAdapter(
        goal_topic=config['simple_goal_topic'],
        status_topic=config['navigation_status_topic'],
        status_timeout_sec=config['navigation_status_timeout_sec'],
        clock=clock,
        cancel_topic=config['navigation_cancel_topic'],
    )
    canceller = backend if backend.supports_cancel else NoOpCanceller(on_cancel=backend.cancel)
    return NavigationRuntimeBinding(dispatcher=backend, status_monitor=backend, canceller=canceller)


def _build_capabilities(
    config: Mapping[str, object],
    *,
    clock: Optional['NodeClock'] = None,
    arrival_evaluator: Optional['ArrivalEvaluator'] = None,
    builders_enabled: bool = True,
) -> Dict[str, NavigationAdapterCapability]:
    def builder_or_disabled(name: str, factory: Callable[[], NavigationRuntimeBinding]) -> Callable[[], NavigationRuntimeBinding]:
        return factory if builders_enabled else _builders_disabled(name)

    if builders_enabled and clock is None:
        raise ConfigurationError('clock is required to build navigation adapters')
    if builders_enabled and str(config.get('navigation_adapter_type', '')).strip().lower() == 'simple_topic' and arrival_evaluator is None:
        raise ConfigurationError('arrival_evaluator is required to build simple_topic navigation adapter')

    return {
        'move_base_action': NavigationAdapterCapability(
            name='move_base_action',
            status_source='actionlib',
            supports_cancel=True,
            requires_external_status=False,
            description='move_base actionlib adapter',
            planner_interface='move_base_global_planner',
            controller_interface='move_base_local_controller',
            recovery_interface='executor_policy',
            localization_interface='pose_source_external',
            backend_profile='move_base_managed',
            goal_transport='actionlib',
            status_transport='actionlib',
            cancel_transport='actionlib',
            binding_builder=builder_or_disabled('move_base_action', lambda: _build_move_base_binding(config, clock)),
        ),
        'simple_topic': NavigationAdapterCapability(
            name='simple_topic',
            status_source='pose_evaluator',
            supports_cancel=False,
            requires_external_status=False,
            description='goal topic adapter with local pose arrival evaluation',
            planner_interface='external_goal_dispatch',
            controller_interface='external_controller_stack',
            recovery_interface='executor_policy',
            localization_interface='pose_evaluator',
            backend_profile='topic_pose_evaluator',
            goal_transport='topic',
            status_transport='internal_pose_evaluator',
            cancel_transport='none',
            binding_builder=builder_or_disabled('simple_topic', lambda: _build_simple_topic_binding(config, arrival_evaluator, clock)),
        ),
        'goal_topic_status': NavigationAdapterCapability(
            name='goal_topic_status',
            status_source='external_status',
            supports_cancel=bool(str(config.get('navigation_cancel_topic', '')).strip()),
            requires_external_status=True,
            description='goal topic adapter with external status feedback',
            planner_interface='external_goal_dispatch',
            controller_interface='external_controller_stack',
            recovery_interface='external_status_plus_executor_policy',
            localization_interface='pose_source_external',
            backend_profile='topic_status_bridge',
            goal_transport='topic',
            status_transport='topic',
            cancel_transport='topic' if bool(str(config.get('navigation_cancel_topic', '')).strip()) else 'none',
            binding_builder=builder_or_disabled('goal_topic_status', lambda: _build_goal_topic_status_binding(config, clock)),
        ),
    }



def build_navigation_registry(
    config: Mapping[str, object],
    *,
    clock: Optional['NodeClock'] = None,
    arrival_evaluator: Optional['ArrivalEvaluator'] = None,
    builders_enabled: bool = True,
) -> NavigationAdapterRegistry:
    return NavigationAdapterRegistry(
        _build_capabilities(
            config,
            clock=clock,
            arrival_evaluator=arrival_evaluator,
            builders_enabled=builders_enabled,
        )
    )



def describe_navigation_capability(config: Mapping[str, object]) -> Dict[str, object]:
    adapter_type = str(config.get('navigation_adapter_type', '')).strip()
    if not adapter_type:
        raise ConfigurationError('navigation_adapter_type must not be empty for navigation capability resolution')
    registry = build_navigation_registry(config, builders_enabled=False)
    return registry.describe(adapter_type)

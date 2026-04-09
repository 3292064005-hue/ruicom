"""Platform-adapter capability declarations."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence



def _merge_unique(*groups: Sequence[str]) -> tuple[str, ...]:
    seen: list[str] = []
    for group in groups:
        for item in group:
            text = str(item).strip()
            if text and text not in seen:
                seen.append(text)
    return tuple(seen)


@dataclass(frozen=True)
class PlatformAdapterCapability:
    """Static capability declaration for one platform family.

    Args:
        name: Stable adapter identifier.
        description: Human-readable platform description.
        shared_defaults: Cross-component defaults such as runtime strategy,
            motion model, command semantics and bridge policy.
        mission_defaults: Mission-node specific defaults.
        vision_defaults: Vision-node specific defaults.
        safety_defaults: Safety-node specific defaults.
        *_required_topics/actions: Domain-specific binding requirements.

    Boundary behavior:
        ``shared_defaults`` are applied before domain-specific defaults so profile
        files may override either layer explicitly.
    """

    name: str
    description: str
    shared_defaults: Mapping[str, object] = field(default_factory=dict)
    mission_defaults: Mapping[str, object] = field(default_factory=dict)
    vision_defaults: Mapping[str, object] = field(default_factory=dict)
    safety_defaults: Mapping[str, object] = field(default_factory=dict)
    mission_required_topics: Sequence[str] = field(default_factory=tuple)
    mission_required_actions: Sequence[str] = field(default_factory=tuple)
    vision_required_topics: Sequence[str] = field(default_factory=tuple)
    vision_required_actions: Sequence[str] = field(default_factory=tuple)
    safety_required_topics: Sequence[str] = field(default_factory=tuple)
    safety_required_actions: Sequence[str] = field(default_factory=tuple)
    bridge_required_topics: Sequence[str] = field(default_factory=tuple)
    bridge_required_actions: Sequence[str] = field(default_factory=tuple)

    @property
    def required_topics(self) -> tuple[str, ...]:
        return _merge_unique(self.mission_required_topics, self.vision_required_topics, self.safety_required_topics, self.bridge_required_topics)

    @property
    def required_actions(self) -> tuple[str, ...]:
        return _merge_unique(self.mission_required_actions, self.vision_required_actions, self.safety_required_actions, self.bridge_required_actions)

    def summary(self) -> dict:
        return {
            'name': self.name,
            'description': self.description,
            'shared_defaults': dict(self.shared_defaults),
            'mission_defaults': dict(self.mission_defaults),
            'vision_defaults': dict(self.vision_defaults),
            'safety_defaults': dict(self.safety_defaults),
            'mission_required_topics': list(self.mission_required_topics),
            'mission_required_actions': list(self.mission_required_actions),
            'vision_required_topics': list(self.vision_required_topics),
            'vision_required_actions': list(self.vision_required_actions),
            'safety_required_topics': list(self.safety_required_topics),
            'safety_required_actions': list(self.safety_required_actions),
            'bridge_required_topics': list(self.bridge_required_topics),
            'bridge_required_actions': list(self.bridge_required_actions),
            'required_topics': list(self.required_topics),
            'required_actions': list(self.required_actions),
        }

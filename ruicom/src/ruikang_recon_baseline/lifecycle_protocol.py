"""Lifecycle control envelope helpers.

This module keeps the supervisor command protocol deterministic while remaining
backward compatible with the historical plain-string command topic.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from .common import JsonCodec

BROADCAST_TARGETS = {'', '*', 'all', 'broadcast'}


@dataclass(frozen=True)
class LifecycleControlEnvelope:
    """Normalized lifecycle command envelope.

    Attributes:
        command: Canonical command verb such as ``configure`` or ``activate``.
        target: Optional node target. Empty means broadcast.
        issued_by: Human-readable source identifier.
        metadata: Free-form structured annotations.
    """

    command: str
    target: str = ''
    issued_by: str = ''
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @property
    def is_broadcast(self) -> bool:
        return str(self.target).strip().lower() in BROADCAST_TARGETS

    def matches(self, node_name: str) -> bool:
        normalized_node = str(node_name).strip()
        if not normalized_node:
            return self.is_broadcast
        return self.is_broadcast or str(self.target).strip() == normalized_node

    def as_dict(self) -> dict:
        return {
            'command': str(self.command).strip().lower(),
            'target': str(self.target).strip(),
            'issued_by': str(self.issued_by).strip(),
            'metadata': dict(self.metadata or {}),
        }



def decode_lifecycle_control(raw: object) -> LifecycleControlEnvelope:
    """Decode one lifecycle control message.

    Boundary behavior:
        Historical plain-string commands remain valid and decode to broadcast
        envelopes. JSON payloads may target one specific node by name.
    """

    text = str(raw or '').strip()
    if not text:
        return LifecycleControlEnvelope(command='')
    if text.startswith('{'):
        try:
            payload = JsonCodec.loads(text)
        except Exception:
            payload = None
        if isinstance(payload, Mapping):
            return LifecycleControlEnvelope(
                command=str(payload.get('command', '')).strip().lower(),
                target=str(payload.get('target', '')).strip(),
                issued_by=str(payload.get('issued_by', '')).strip(),
                metadata=dict(payload.get('metadata', {}) or {}),
            )
    return LifecycleControlEnvelope(command=text.strip().lower())



def encode_lifecycle_control(command: str, *, target: str = '', issued_by: str = '', metadata: Mapping[str, Any] | None = None) -> str:
    """Encode one lifecycle control command for the shared ROS String topic."""

    envelope = LifecycleControlEnvelope(
        command=str(command or '').strip().lower(),
        target=str(target).strip(),
        issued_by=str(issued_by).strip(),
        metadata=dict(metadata or {}),
    )
    if envelope.is_broadcast and not envelope.issued_by and not envelope.metadata:
        return envelope.command
    return JsonCodec.dumps(envelope.as_dict())

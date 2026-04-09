"""Mission-node class-schema validation helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional, Sequence

from .common import ConfigurationError, class_schema_hash, validate_dynamic_class_names, validate_embedded_class_schema


@dataclass
class MissionSchemaGuard:
    """Validate mission-local and upstream class-schema consistency.

    Args:
        class_names: Authoritative mission class ordering.
        mismatch_policy: ``warn`` or ``error``.
        on_mismatch: Callback invoked with ``stream``, ``message`` and optional
            details whenever validation fails.
    """

    class_names: Sequence[str]
    mismatch_policy: str
    on_mismatch: Callable[[str, str, Optional[dict]], None]

    def validate_local_schema(self) -> tuple[str, ...]:
        return validate_dynamic_class_names(self.class_names, owner='mission_manager_node')

    @property
    def expected_hash(self) -> str:
        return class_schema_hash(self.class_names)

    def validate_upstream_schema(self, payload: Dict, *, stream: str) -> bool:
        try:
            validate_embedded_class_schema(payload, expected_class_names=self.class_names, owner=stream)
            return True
        except ConfigurationError as exc:
            details = {
                'expected_class_names': list(self.class_names),
                'expected_class_schema_hash': self.expected_hash,
                'payload_class_names': list(payload.get('class_names', [])),
                'payload_class_schema_hash': str(payload.get('class_schema_hash', '')).strip(),
            }
            self.on_mismatch(stream, str(exc), details)
            return False

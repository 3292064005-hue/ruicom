from __future__ import annotations

"""Repository-managed downstream behavior runtime.

This module closes the gap between semantic behavior execution requests and
observable in-repository runtime effects. It consumes normalized transport
requests from :mod:`behavior_executor_node`, drives concrete repository control
channels (the safety ingress velocity bus plus a structured actuator command
signal), and emits explicit terminal receipts only after observable success
criteria are met.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, MutableMapping, Optional, Sequence, Tuple

from .common import ConfigurationError

_ALLOWED_SUCCESS_POLICIES = {'platform_clear', 'detection_confirmed'}
_ALLOWED_ACTUATOR_STATES = {'ACTIVE', 'COMPLETED', 'FAILED'}


@dataclass(frozen=True)
class RuntimeActionSpec:
    action_type: str
    success_policy: str
    command_hold_sec: float
    settle_sec: float
    require_platform_feedback: bool
    require_ultrasonic_clear: bool
    require_actuator_state: bool = False
    required_actuator_states: Tuple[str, ...] = ()
    command_velocity: Dict[str, float] = field(default_factory=dict)
    actuator_command: Dict[str, Any] = field(default_factory=dict)
    confirmation_classes: Tuple[str, ...] = ()
    required_detection_count: int = 0
    max_detection_age_sec: float = 1.5

    def validate(self) -> None:
        if not str(self.action_type).strip():
            raise ConfigurationError('runtime action_type must not be empty')
        if str(self.success_policy).strip().lower() not in _ALLOWED_SUCCESS_POLICIES:
            raise ConfigurationError('runtime success_policy must be one of {}'.format(', '.join(sorted(_ALLOWED_SUCCESS_POLICIES))))
        if float(self.command_hold_sec) < 0.0:
            raise ConfigurationError('runtime command_hold_sec must be >= 0 for {}'.format(self.action_type))
        if float(self.settle_sec) < 0.0:
            raise ConfigurationError('runtime settle_sec must be >= 0 for {}'.format(self.action_type))
        if int(self.required_detection_count) < 0:
            raise ConfigurationError('runtime required_detection_count must be >= 0 for {}'.format(self.action_type))
        if float(self.max_detection_age_sec) <= 0.0:
            raise ConfigurationError('runtime max_detection_age_sec must be > 0 for {}'.format(self.action_type))
        invalid_states = [item for item in self.required_actuator_states if item not in _ALLOWED_ACTUATOR_STATES]
        if invalid_states:
            raise ConfigurationError('runtime required_actuator_states must be within {} for {}'.format(sorted(_ALLOWED_ACTUATOR_STATES), self.action_type))


@dataclass(frozen=True)
class RuntimeDispatch:
    command_id: str
    action_type: str
    command_payload: Dict[str, Any]
    actuator_payload: Dict[str, Any]


@dataclass(frozen=True)
class RuntimeSnapshot:
    command_id: str
    action_type: str
    task_type: str
    metadata: Dict[str, Any]
    timeout_sec: float
    started_at: float
    hold_until: float
    spec: RuntimeActionSpec
    cancelled: bool = False
    success_since: float = 0.0


class BehaviorRuntimeCore:
    def __init__(self, *, action_specs: Mapping[str, RuntimeActionSpec]):
        specs: Dict[str, RuntimeActionSpec] = {}
        for action_type, raw_spec in dict(action_specs or {}).items():
            spec = raw_spec
            spec.validate()
            specs[str(action_type).strip().lower() or str(spec.action_type).strip().lower()] = spec
        if not specs:
            raise ConfigurationError('behavior runtime requires at least one action spec')
        self._action_specs = specs
        self._active: Optional[RuntimeSnapshot] = None
        self._pending_terminal: Optional[Dict[str, Any]] = None

    @property
    def active(self) -> Optional[RuntimeSnapshot]:
        return self._active

    def supported_actions(self) -> Tuple[str, ...]:
        return tuple(sorted(self._action_specs.keys()))

    def _resolve_spec(self, action_type: str) -> RuntimeActionSpec:
        normalized = str(action_type).strip().lower()
        if normalized not in self._action_specs:
            raise ConfigurationError('unsupported runtime action_type: {}'.format(action_type))
        return self._action_specs[normalized]

    def _command_velocity(self, spec: RuntimeActionSpec, metadata: Mapping[str, Any]) -> Dict[str, float]:
        velocity = dict(spec.command_velocity or {})
        for key in ('linear_x', 'linear_y', 'angular_z'):
            if key in metadata:
                try:
                    velocity[key] = float(metadata.get(key) or 0.0)
                except Exception:
                    pass
        return {
            'linear_x': float(velocity.get('linear_x', 0.0) or 0.0),
            'linear_y': float(velocity.get('linear_y', 0.0) or 0.0),
            'angular_z': float(velocity.get('angular_z', 0.0) or 0.0),
        }

    def accept_request(self, payload: Mapping[str, Any], *, now_sec: float) -> RuntimeDispatch:
        if self._active is not None or self._pending_terminal is not None:
            raise RuntimeError('behavior runtime is busy')
        command = dict(payload.get('command', {}) or {})
        metadata = dict(command.get('metadata', {}) or {})
        command_id = str(command.get('command_id', '')).strip()
        action_type = str(command.get('action_type', '')).strip()
        task_type = str(command.get('task_type', '')).strip()
        timeout_sec = float(command.get('timeout_sec', 0.0) or 0.0)
        if not command_id:
            raise ConfigurationError('runtime request requires command.command_id')
        if not action_type:
            raise ConfigurationError('runtime request requires command.action_type')
        if timeout_sec <= 0.0:
            raise ConfigurationError('runtime request requires command.timeout_sec > 0')
        spec = self._resolve_spec(action_type)
        snapshot = RuntimeSnapshot(
            command_id=command_id,
            action_type=action_type,
            task_type=task_type,
            metadata=metadata,
            timeout_sec=timeout_sec,
            started_at=float(now_sec),
            hold_until=float(now_sec) + float(spec.command_hold_sec),
            spec=spec,
            cancelled=False,
            success_since=0.0,
        )
        self._active = snapshot
        return RuntimeDispatch(
            command_id=command_id,
            action_type=action_type,
            command_payload={
                'command_id': command_id,
                'action_type': action_type,
                'task_type': task_type,
                'stamp': float(now_sec),
                'velocity': self._command_velocity(spec, metadata),
                'metadata': metadata,
            },
            actuator_payload={
                'command_id': command_id,
                'action_type': action_type,
                'task_type': task_type,
                'stamp': float(now_sec),
                'timeout_sec': float(command.get('timeout_sec', timeout_sec) or timeout_sec),
                'hold_sec': float(spec.command_hold_sec),
                'actuator_command': dict(spec.actuator_command or {}),
                'metadata': metadata,
            },
        )

    def command_tick_payload(self, *, now_sec: float) -> Optional[Dict[str, Any]]:
        active = self._active
        if active is None or active.cancelled:
            return None
        if float(now_sec) > float(active.hold_until):
            return None
        return {
            'command_id': active.command_id,
            'action_type': active.action_type,
            'task_type': active.task_type,
            'stamp': float(now_sec),
            'velocity': self._command_velocity(active.spec, active.metadata),
            'metadata': dict(active.metadata or {}),
        }

    def cancel(self, *, now_sec: float, reason: str) -> Optional[Dict[str, Any]]:
        active = self._active
        if active is None:
            return None
        self._pending_terminal = {
            'command_id': active.command_id,
            'status': 'FAILED',
            'stamp': float(now_sec),
            'details': {
                'reason': str(reason).strip() or 'cancelled',
                'action_type': active.action_type,
                'task_type': active.task_type,
            },
            'source': 'behavior_runtime',
        }
        self._active = None
        return dict(self._pending_terminal)

    def evaluate(self, *, now_sec: float, platform_summary: Mapping[str, Any], detection_counts: Mapping[str, int], detection_fresh: bool, actuator_state: Mapping[str, Any] | None = None, actuator_state_fresh: bool = False) -> Optional[Dict[str, Any]]:
        active = self._active
        if active is None:
            return None
        spec = active.spec
        if float(now_sec) > float(active.started_at) + float(active.timeout_sec):
            self._pending_terminal = {
                'command_id': active.command_id,
                'status': 'FAILED',
                'stamp': float(now_sec),
                'details': {
                    'reason': 'runtime_timeout',
                    'action_type': active.action_type,
                    'task_type': active.task_type,
                },
                'source': 'behavior_runtime',
            }
            self._active = None
            return dict(self._pending_terminal)
        platform_ok = True
        if spec.require_platform_feedback:
            platform_ok = all([
                bool(platform_summary.get('feedback_contract_satisfied') or platform_summary.get('output_feedback') or platform_summary.get('output_feedback_required_satisfied')),
                bool(platform_summary.get('execution_feedback_fresh', False) or platform_summary.get('explicit_feedback_fresh', False) or platform_summary.get('odom_feedback_fresh', False)),
                bool(platform_summary.get('vendor_runtime_contract_satisfied', True)),
                bool(platform_summary.get('vendor_bundle_preflight_satisfied', True)),
            ])
        ultrasonic_ok = True
        if spec.require_ultrasonic_clear:
            ultrasonic_ok = not bool(platform_summary.get('ultrasonic_hazard_active', False))
            if bool(platform_summary.get('upstream_ultrasonic_topic', '')):
                ultrasonic_ok = ultrasonic_ok and bool(platform_summary.get('ultrasonic_fresh', False))
        actuator_ok = True
        if spec.require_actuator_state:
            state_name = str((actuator_state or {}).get('state', '')).strip().upper()
            state_command_id = str((actuator_state or {}).get('command_id', '')).strip()
            allowed_states = tuple(spec.required_actuator_states or ('ACTIVE', 'COMPLETED'))
            actuator_ok = bool(actuator_state_fresh and state_command_id == active.command_id and state_name in allowed_states)
        detection_ok = True
        if str(spec.success_policy).lower() == 'detection_confirmed':
            required_classes = list(spec.confirmation_classes or ())
            metadata_classes = active.metadata.get('confirmation_classes', [])
            if metadata_classes:
                required_classes = [str(item).strip() for item in metadata_classes if str(item).strip()]
            required_count = int(active.metadata.get('required_detection_count', spec.required_detection_count) or spec.required_detection_count)
            if not detection_fresh:
                detection_ok = False
            else:
                if required_classes:
                    observed = sum(int(detection_counts.get(name, 0) or 0) for name in required_classes)
                else:
                    observed = sum(int(value or 0) for value in detection_counts.values())
                detection_ok = observed >= required_count
        success_ready = platform_ok and ultrasonic_ok and actuator_ok and detection_ok and float(now_sec) >= float(active.hold_until)
        success_since = float(active.success_since)
        if success_ready:
            if success_since <= 0.0:
                self._active = RuntimeSnapshot(**{**active.__dict__, 'success_since': float(now_sec)})
                return None
            if (float(now_sec) - success_since) >= float(spec.settle_sec):
                self._pending_terminal = {
                    'command_id': active.command_id,
                    'status': 'SUCCEEDED',
                    'stamp': float(now_sec),
                    'details': {
                        'action_type': active.action_type,
                        'task_type': active.task_type,
                        'platform_ok': bool(platform_ok),
                        'ultrasonic_clear': bool(ultrasonic_ok),
                        'actuator_confirmed': bool(actuator_ok),
                        'detection_confirmed': bool(detection_ok),
                        'platform_summary': dict(platform_summary or {}),
                        'actuator_state': dict(actuator_state or {}),
                        'detection_counts': {str(k): int(v or 0) for k, v in dict(detection_counts or {}).items()},
                    },
                    'source': 'behavior_runtime',
                }
                self._active = None
                return dict(self._pending_terminal)
            return None
        if success_since > 0.0:
            self._active = RuntimeSnapshot(**{**active.__dict__, 'success_since': 0.0})
        return None

    def pop_terminal(self) -> Optional[Dict[str, Any]]:
        payload = self._pending_terminal
        self._pending_terminal = None
        return dict(payload) if payload is not None else None


def _coerce_str_tuple(values: Sequence[object] | None) -> Tuple[str, ...]:
    return tuple(str(item).strip() for item in (values or ()) if str(item).strip())


def build_behavior_runtime_specs(config: Mapping[str, object]) -> Dict[str, RuntimeActionSpec]:
    raw_catalog = dict(config.get('action_specs', {}) or {})
    if not raw_catalog:
        raise ConfigurationError('behavior runtime action_specs must not be empty')
    specs: Dict[str, RuntimeActionSpec] = {}
    default_hold = float(config.get('default_command_hold_sec', 0.5) or 0.5)
    default_settle = float(config.get('default_settle_sec', 0.2) or 0.2)
    for action_type, raw in raw_catalog.items():
        payload = dict(raw or {}) if isinstance(raw, MutableMapping) else {}
        canonical_action = str(payload.get('action_type', action_type)).strip().lower() or str(action_type).strip().lower()
        spec = RuntimeActionSpec(
            action_type=canonical_action,
            success_policy=str(payload.get('success_policy', 'platform_clear')).strip().lower() or 'platform_clear',
            command_hold_sec=float(payload.get('command_hold_sec', default_hold) or 0.0),
            settle_sec=float(payload.get('settle_sec', default_settle) or 0.0),
            require_platform_feedback=bool(payload.get('require_platform_feedback', True)),
            require_ultrasonic_clear=bool(payload.get('require_ultrasonic_clear', canonical_action == 'hazard_avoid')),
            require_actuator_state=bool(payload.get('require_actuator_state', bool(payload.get('actuator_command')) and canonical_action == 'facility_attack')),
            required_actuator_states=_coerce_str_tuple(payload.get('required_actuator_states', ('ACTIVE', 'COMPLETED'))),
            command_velocity=dict(payload.get('command_velocity', {}) or {}),
            actuator_command=dict(payload.get('actuator_command', {}) or {}),
            confirmation_classes=_coerce_str_tuple(payload.get('confirmation_classes', ())),
            required_detection_count=int(payload.get('required_detection_count', 0) or 0),
            max_detection_age_sec=float(payload.get('max_detection_age_sec', 1.5) or 1.5),
        )
        spec.validate()
        specs[canonical_action] = spec
    return specs

"""Pure behavior-execution engine for deploy-grade action transports.

The mission executor decides *when* a semantic task action such as
``hazard_avoid`` or ``facility_attack`` must run. This module owns the next
layer down: it validates the command, binds it to one configured downstream
execution transport, tracks receipts coming back from that transport, and
produces normalized feedback packets for the mission topic backend.

Unlike the previous repository-managed receipt simulator, this core never marks
an action successful merely because time passed. Success/failure must come from a
matching downstream receipt or from an explicit timeout/cancellation path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, MutableMapping, Optional, Sequence, Tuple

from .behavior_actions import BehaviorActionCommand, BehaviorActionFeedback, CANONICAL_BEHAVIOR_STATUSES
from .common import ConfigurationError, JsonCodec


_TERMINAL_STATUSES = {'SUCCEEDED', 'FAILED', 'TIMEOUT'}
_ALLOWED_BUSY_POLICIES = {'reject'}
_ALLOWED_TIMEOUT_POLICIES = {'timeout'}
_ALLOWED_LIVE_STATUSES = {'PENDING', 'ACTIVE'}


@dataclass(frozen=True)
class BehaviorExecutionSpec:
    """One configured downstream execution profile for a behavior action type.

    Attributes:
        action_type: Canonical action name such as ``hazard_avoid``.
        supported_task_types: Task types that may legally use this profile.
        pending_to_active_sec: Optional local transition window used to graduate
            a dispatched command from ``PENDING`` to ``ACTIVE`` while waiting for
            explicit downstream progress receipts.
        dispatch_topic: Topic carrying downstream execution requests.
        result_topic: Topic carrying downstream execution receipts.
        cancel_topic: Topic carrying downstream cancellation requests.
        dispatch_details: Structured execution hints forwarded inside the
            outbound request envelope.
        success_details: Structured details merged into terminal success
            feedback after a downstream ``SUCCEEDED`` receipt.
        busy_policy: Policy used when a new command arrives while another one is
            still active.
        timeout_policy: Policy used when the command deadline expires.

    Boundary behavior:
        Empty ``supported_task_types`` means the profile accepts any task whose
        canonical action type matches. A downstream result topic is mandatory so
        deploy-grade profiles cannot silently auto-succeed.
    """

    action_type: str
    supported_task_types: Tuple[str, ...]
    pending_to_active_sec: float
    dispatch_topic: str
    result_topic: str
    cancel_topic: str
    dispatch_details: Dict[str, Any] = field(default_factory=dict)
    success_details: Dict[str, Any] = field(default_factory=dict)
    busy_policy: str = 'reject'
    timeout_policy: str = 'timeout'

    def validate(self) -> None:
        """Validate one execution profile in place.

        Raises:
            ConfigurationError: If one field is empty, negative, or unsupported.
        """
        if not str(self.action_type).strip():
            raise ConfigurationError('behavior execution spec action_type must not be empty')
        if float(self.pending_to_active_sec) < 0.0:
            raise ConfigurationError('behavior execution spec pending_to_active_sec must be >= 0 for {}'.format(self.action_type))
        if not str(self.dispatch_topic).strip():
            raise ConfigurationError('behavior execution spec dispatch_topic must not be empty for {}'.format(self.action_type))
        if not str(self.result_topic).strip():
            raise ConfigurationError('behavior execution spec result_topic must not be empty for {}'.format(self.action_type))
        if not str(self.cancel_topic).strip():
            raise ConfigurationError('behavior execution spec cancel_topic must not be empty for {}'.format(self.action_type))
        if str(self.busy_policy).strip().lower() not in _ALLOWED_BUSY_POLICIES:
            raise ConfigurationError(
                'behavior execution spec busy_policy must be one of {} for {}'.format(
                    ', '.join(sorted(_ALLOWED_BUSY_POLICIES)),
                    self.action_type,
                )
            )
        if str(self.timeout_policy).strip().lower() not in _ALLOWED_TIMEOUT_POLICIES:
            raise ConfigurationError(
                'behavior execution spec timeout_policy must be one of {} for {}'.format(
                    ', '.join(sorted(_ALLOWED_TIMEOUT_POLICIES)),
                    self.action_type,
                )
            )


@dataclass(frozen=True)
class BehaviorExecutionSnapshot:
    """Runtime snapshot for one active behavior command."""

    command: BehaviorActionCommand
    spec: BehaviorExecutionSpec
    accepted_at: float
    terminal_feedback: Optional[BehaviorActionFeedback] = None
    live_feedback: Optional[BehaviorActionFeedback] = None


@dataclass(frozen=True)
class DownstreamDispatchEnvelope:
    """One normalized downstream execution request to publish on ROS topics."""

    command_id: str
    action_type: str
    dispatch_topic: str
    result_topic: str
    cancel_topic: str
    payload: Dict[str, Any]


@dataclass(frozen=True)
class DownstreamCancelEnvelope:
    """One normalized downstream execution cancel request."""

    command_id: str
    action_type: str
    cancel_topic: str
    payload: Dict[str, Any]


def extract_command_id_from_payload_text(payload_text: str) -> str:
    """Best-effort extraction of ``command_id`` from one wire payload.

    Args:
        payload_text: Raw JSON text received from a ROS topic.

    Returns:
        str: Extracted command id, or an empty string when unavailable.

    Boundary behavior:
        The helper is deliberately forgiving; malformed JSON simply yields an
        empty identifier instead of raising, so reject paths can still respond
        safely before a command is fully validated.
    """
    try:
        payload = JsonCodec.loads(payload_text)
    except Exception:
        return ''
    if not isinstance(payload, dict):
        return ''
    return str(payload.get('command_id', '')).strip()


class BehaviorExecutorCore:
    """Stateful downstream-receipt-driven executor for topic-backed actions.

    A command progresses through the following states:

    ``PENDING`` -> ``ACTIVE`` -> terminal receipt

    Terminal success/failure requires a matching downstream receipt. The core
    only synthesizes terminal feedback for timeout/cancellation paths.
    """

    def __init__(self, *, action_specs: Mapping[str, BehaviorExecutionSpec]):
        specs: Dict[str, BehaviorExecutionSpec] = {}
        for action_type, raw_spec in dict(action_specs or {}).items():
            spec = raw_spec
            spec.validate()
            key = str(action_type).strip().lower() or str(spec.action_type).strip().lower()
            specs[key] = spec
        if not specs:
            raise ConfigurationError('behavior executor requires at least one action spec')
        self._action_specs = specs
        self._active: Optional[BehaviorExecutionSnapshot] = None

    @property
    def active(self) -> Optional[BehaviorExecutionSnapshot]:
        return self._active

    def supported_actions(self) -> Tuple[str, ...]:
        return tuple(sorted(self._action_specs.keys()))

    def _resolve_spec(self, command: BehaviorActionCommand) -> BehaviorExecutionSpec:
        action_type = str(command.action_type).strip().lower()
        if action_type not in self._action_specs:
            raise ConfigurationError('unsupported behavior action_type: {}'.format(command.action_type))
        spec = self._action_specs[action_type]
        allowed_task_types = {str(item).strip() for item in spec.supported_task_types if str(item).strip()}
        if allowed_task_types and str(command.task_type).strip() not in allowed_task_types:
            raise ConfigurationError(
                'behavior action {} does not accept task_type {}'.format(command.action_type, command.task_type)
            )
        return spec

    def accept(self, command: BehaviorActionCommand, *, now_sec: float) -> DownstreamDispatchEnvelope:
        """Accept one new command and build its downstream dispatch envelope.

        Raises:
            RuntimeError: If another command is still active.
            ConfigurationError: If the command does not match any configured
                execution profile.
        """
        if self._active is not None:
            current_status = self.poll(now_sec).status
            if current_status in _TERMINAL_STATUSES:
                raise RuntimeError('behavior executor is draining terminal feedback for command {}'.format(self._active.command.command_id))
            raise RuntimeError('behavior executor is busy with command {}'.format(self._active.command.command_id))
        spec = self._resolve_spec(command)
        self._active = BehaviorExecutionSnapshot(
            command=command,
            spec=spec,
            accepted_at=float(now_sec),
            terminal_feedback=None,
            live_feedback=BehaviorActionFeedback(
                command_id=command.command_id,
                status='PENDING',
                stamp=float(now_sec),
                details={
                    'action_type': command.action_type,
                    'dispatch_topic': spec.dispatch_topic,
                    'result_topic': spec.result_topic,
                    'cancel_topic': spec.cancel_topic,
                    'phase': 'dispatch_pending',
                },
                source='behavior_executor',
            ),
        )
        return self.build_dispatch_envelope(command.command_id, now_sec=now_sec)

    def build_dispatch_envelope(self, command_id: str, *, now_sec: float) -> DownstreamDispatchEnvelope:
        """Build the transport envelope for the currently active command.

        Args:
            command_id: Identifier of the active command.
            now_sec: Business timestamp for the dispatch envelope.

        Raises:
            RuntimeError: If there is no matching active command.
        """
        active = self._active
        if active is None or active.command.command_id != str(command_id).strip():
            raise RuntimeError('behavior executor has no active command {}'.format(command_id))
        command = active.command
        spec = active.spec
        payload = {
            'schema_version': 1,
            'stamp': float(now_sec),
            'command': command.to_dict(),
            'execution': {
                'dispatch_topic': spec.dispatch_topic,
                'result_topic': spec.result_topic,
                'cancel_topic': spec.cancel_topic,
                'dispatch_details': dict(spec.dispatch_details or {}),
                'runtime_expectations': {
                    'pending_to_active_sec': float(spec.pending_to_active_sec),
                    'timeout_sec': float(command.timeout_sec),
                },
            },
        }
        return DownstreamDispatchEnvelope(
            command_id=command.command_id,
            action_type=command.action_type,
            dispatch_topic=spec.dispatch_topic,
            result_topic=spec.result_topic,
            cancel_topic=spec.cancel_topic,
            payload=payload,
        )

    def clear_terminal(self) -> None:
        if self._active is not None and self._active.terminal_feedback is not None:
            self._active = None

    def cancel(self, *, now_sec: float, reason: str) -> Optional[BehaviorActionFeedback]:
        if self._active is None:
            return None
        feedback = BehaviorActionFeedback(
            command_id=self._active.command.command_id,
            status='FAILED',
            stamp=float(now_sec),
            details={
                'reason': str(reason).strip() or 'cancelled',
                'action_type': self._active.command.action_type,
                'cancel_topic': self._active.spec.cancel_topic,
            },
            source='behavior_executor',
        )
        self._active = BehaviorExecutionSnapshot(
            command=self._active.command,
            spec=self._active.spec,
            accepted_at=self._active.accepted_at,
            terminal_feedback=feedback,
            live_feedback=feedback,
        )
        return feedback

    def build_cancel_envelope(self, *, now_sec: float, reason: str) -> Optional[DownstreamCancelEnvelope]:
        """Build the downstream cancel envelope for the active command.

        Returns ``None`` when no command is active.
        """
        if self._active is None:
            return None
        command = self._active.command
        spec = self._active.spec
        payload = {
            'schema_version': 1,
            'stamp': float(now_sec),
            'command_id': command.command_id,
            'action_type': command.action_type,
            'reason': str(reason).strip() or 'cancelled',
            'route_id': command.route_id,
            'step_id': command.step_id,
            'task_type': command.task_type,
        }
        return DownstreamCancelEnvelope(
            command_id=command.command_id,
            action_type=command.action_type,
            cancel_topic=spec.cancel_topic,
            payload=payload,
        )

    def observe_result(
        self,
        *,
        command_id: str,
        status: str,
        now_sec: float,
        details: Optional[Mapping[str, Any]] = None,
        source: str = '',
    ) -> Optional[BehaviorActionFeedback]:
        """Observe one downstream execution receipt.

        Args:
            command_id: Receipt identifier that must match the active command.
            status: Canonical downstream status.
            now_sec: Receipt timestamp.
            details: Structured downstream details.
            source: Human-readable receipt source.

        Returns:
            Optional[BehaviorActionFeedback]: Updated feedback for the active
            command, or ``None`` when the receipt does not apply.
        """
        active = self._active
        if active is None or str(command_id).strip() != active.command.command_id:
            return None
        normalized_status = str(status).strip().upper() or 'FAILED'
        if normalized_status not in CANONICAL_BEHAVIOR_STATUSES:
            raise ConfigurationError('unsupported downstream behavior status: {}'.format(status))
        if normalized_status == 'IDLE':
            return None
        payload_details = dict(details or {})
        payload_details.setdefault('action_type', active.command.action_type)
        payload_details.setdefault('task_type', active.command.task_type)
        payload_details.setdefault('route_id', active.command.route_id)
        feedback = BehaviorActionFeedback(
            command_id=active.command.command_id,
            status=normalized_status,
            stamp=float(now_sec),
            details=payload_details,
            source=str(source).strip() or 'behavior_executor_runtime',
        )
        if normalized_status == 'SUCCEEDED':
            merged = dict(active.spec.success_details or {})
            merged.update(payload_details)
            feedback = BehaviorActionFeedback(
                command_id=feedback.command_id,
                status=feedback.status,
                stamp=feedback.stamp,
                details=merged,
                source=feedback.source,
            )
        terminal_feedback = feedback if normalized_status in _TERMINAL_STATUSES else None
        self._active = BehaviorExecutionSnapshot(
            command=active.command,
            spec=active.spec,
            accepted_at=active.accepted_at,
            terminal_feedback=terminal_feedback,
            live_feedback=feedback,
        )
        return feedback

    def poll(self, now_sec: float) -> BehaviorActionFeedback:
        if self._active is None:
            raise RuntimeError('behavior executor has no active command')
        if self._active.terminal_feedback is not None:
            return self._active.terminal_feedback
        command = self._active.command
        spec = self._active.spec
        now_sec = float(now_sec)
        accepted_at = float(self._active.accepted_at)
        elapsed = max(0.0, now_sec - accepted_at)
        timeout_sec = float(command.timeout_sec) if float(command.timeout_sec) > 0.0 else 0.0
        if timeout_sec <= 0.0:
            raise ConfigurationError('behavior command {} requires timeout_sec > 0'.format(command.command_id))
        if elapsed > timeout_sec:
            feedback = BehaviorActionFeedback(
                command_id=command.command_id,
                status='TIMEOUT',
                stamp=now_sec,
                details={
                    'timeout_sec': timeout_sec,
                    'action_type': command.action_type,
                    'result_topic': spec.result_topic,
                    'reason': 'downstream_result_timeout',
                },
                source='behavior_executor',
            )
            self._active = BehaviorExecutionSnapshot(
                command=command,
                spec=spec,
                accepted_at=accepted_at,
                terminal_feedback=feedback,
                live_feedback=feedback,
            )
            return feedback
        if self._active.live_feedback is not None:
            live = self._active.live_feedback
            if live.status == 'ACTIVE':
                return live
            if live.status == 'PENDING' and elapsed < float(spec.pending_to_active_sec):
                return live
        status = 'PENDING' if elapsed < float(spec.pending_to_active_sec) else 'ACTIVE'
        phase = 'awaiting_downstream_dispatch' if status == 'PENDING' else 'awaiting_downstream_result'
        return BehaviorActionFeedback(
            command_id=command.command_id,
            status=status,
            stamp=now_sec,
            details={
                'action_type': command.action_type,
                'task_type': command.task_type,
                'dispatch_topic': spec.dispatch_topic,
                'result_topic': spec.result_topic,
                'cancel_topic': spec.cancel_topic,
                'phase': phase,
            },
            source='behavior_executor',
        )

    def runtime_summary(self, *, now_sec: float) -> Dict[str, Any]:
        active = self._active
        if active is None:
            return {
                'supported_actions': list(self.supported_actions()),
                'has_active_command': False,
                'current_status': 'IDLE',
                'configured_dispatch_topics': {name: spec.dispatch_topic for name, spec in self._action_specs.items()},
                'configured_result_topics': {name: spec.result_topic for name, spec in self._action_specs.items()},
            }
        feedback = self.poll(now_sec)
        return {
            'supported_actions': list(self.supported_actions()),
            'has_active_command': True,
            'current_status': feedback.status,
            'command_id': active.command.command_id,
            'action_type': active.command.action_type,
            'task_type': active.command.task_type,
            'route_id': active.command.route_id,
            'accepted_at': float(active.accepted_at),
            'dispatch_topic': active.spec.dispatch_topic,
            'result_topic': active.spec.result_topic,
            'cancel_topic': active.spec.cancel_topic,
            'feedback': feedback.to_dict(),
        }


def _coerce_str_tuple(values: Sequence[object] | None) -> Tuple[str, ...]:
    return tuple(str(item).strip() for item in (values or ()) if str(item).strip())


def build_behavior_executor_specs(config: Mapping[str, object]) -> Dict[str, BehaviorExecutionSpec]:
    """Build validated action specs from one ROS/config mapping.

    Required defaults:
        ``default_dispatch_topic``
        ``default_result_topic``
        ``default_cancel_topic``
    """
    raw_catalog = dict(config.get('action_specs', {}) or {})
    if not raw_catalog:
        raise ConfigurationError('behavior executor action_specs must not be empty')
    default_pending = float(config.get('default_pending_to_active_sec', 0.2) or 0.2)
    if default_pending < 0.0:
        raise ConfigurationError('default_pending_to_active_sec must be >= 0')
    default_dispatch_topic = str(config.get('default_dispatch_topic', '')).strip()
    default_result_topic = str(config.get('default_result_topic', '')).strip()
    default_cancel_topic = str(config.get('default_cancel_topic', '')).strip()
    if not default_dispatch_topic:
        raise ConfigurationError('default_dispatch_topic must not be empty')
    if not default_result_topic:
        raise ConfigurationError('default_result_topic must not be empty')
    if not default_cancel_topic:
        raise ConfigurationError('default_cancel_topic must not be empty')
    specs: Dict[str, BehaviorExecutionSpec] = {}
    for action_type, raw in raw_catalog.items():
        payload = dict(raw or {}) if isinstance(raw, MutableMapping) else {}
        canonical_action = str(payload.get('action_type', action_type)).strip().lower() or str(action_type).strip().lower()
        spec = BehaviorExecutionSpec(
            action_type=canonical_action,
            supported_task_types=_coerce_str_tuple(payload.get('supported_task_types', (canonical_action,))),
            pending_to_active_sec=float(payload.get('pending_to_active_sec', default_pending) or 0.0),
            dispatch_topic=str(payload.get('dispatch_topic', default_dispatch_topic)).strip() or default_dispatch_topic,
            result_topic=str(payload.get('result_topic', default_result_topic)).strip() or default_result_topic,
            cancel_topic=str(payload.get('cancel_topic', default_cancel_topic)).strip() or default_cancel_topic,
            dispatch_details=dict(payload.get('dispatch_details', payload.get('success_details', {})) or {}),
            success_details=dict(payload.get('success_details', {}) or {}),
            busy_policy=str(payload.get('busy_policy', 'reject')).strip().lower() or 'reject',
            timeout_policy=str(payload.get('timeout_policy', 'timeout')).strip().lower() or 'timeout',
        )
        spec.validate()
        specs[canonical_action] = spec
    return specs

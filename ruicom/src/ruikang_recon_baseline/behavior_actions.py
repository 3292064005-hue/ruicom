"""Behavior-action backends for task-level execute/confirm mission steps.

This module upgrades semantic task types such as ``hazard_avoid`` and
``facility_attack`` from pure observation labels into explicit behavior
executions. The backend is transport-agnostic at the interface level so deploy
profiles may wire real bridges while tests can stay pure-Python.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional

from .common import ConfigurationError, expand_path


CANONICAL_BEHAVIOR_STATUSES = ('IDLE', 'PENDING', 'ACTIVE', 'SUCCEEDED', 'FAILED', 'TIMEOUT')


@dataclass(frozen=True)
class BehaviorActionCommand:
    """One behavior command dispatched to a downstream execution backend.

    Attributes:
        command_id: Stable per-dispatch identifier.
        action_type: Canonical action kind such as ``hazard_avoid``.
        route_id: Mission route identifier owning the action.
        zone_name: Human-readable zone label.
        step_id: Declarative mission step id.
        task_type: Original task type declared by the mission plan.
        objective_type: Domain objective carried by the step.
        metadata: Opaque step metadata forwarded to the backend.
        issued_at: Business-time dispatch timestamp.
        timeout_sec: Backend completion timeout.
    """

    command_id: str
    action_type: str
    route_id: str
    zone_name: str
    step_id: str
    task_type: str
    objective_type: str
    metadata: Dict[str, Any] = field(default_factory=dict)
    issued_at: float = 0.0
    timeout_sec: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            'command_id': self.command_id,
            'action_type': self.action_type,
            'route_id': self.route_id,
            'zone_name': self.zone_name,
            'step_id': self.step_id,
            'task_type': self.task_type,
            'objective_type': self.objective_type,
            'metadata': dict(self.metadata or {}),
            'issued_at': float(self.issued_at),
            'timeout_sec': float(self.timeout_sec),
        }


@dataclass(frozen=True)
class BehaviorActionFeedback:
    """Normalized backend feedback for one behavior action.

    Attributes:
        command_id: Identifier matching the dispatched command.
        status: Canonical backend status.
        stamp: Business-time feedback timestamp.
        details: Backend-defined structured details.
        source: Human-readable producer/source id.
    """

    command_id: str
    status: str
    stamp: float
    details: Dict[str, Any] = field(default_factory=dict)
    source: str = ''

    def to_dict(self) -> Dict[str, Any]:
        return {
            'command_id': self.command_id,
            'status': self.status,
            'stamp': float(self.stamp),
            'details': dict(self.details or {}),
            'source': self.source,
        }


class BehaviorActionBackendBase:
    """Abstract execute/confirm backend used by mission behaviors.

    Implementations dispatch one command and later expose terminal state through
    :meth:`poll`. The interface is intentionally compact so transport details do
    not leak into the mission executor.
    """

    @property
    def enabled(self) -> bool:
        return True

    @property
    def requires_feedback(self) -> bool:
        return True

    def dispatch(self, command: BehaviorActionCommand) -> None:
        """Dispatch one behavior command.

        Args:
            command: Normalized command payload.

        Raises:
            RuntimeError: If the backend cannot accept a new command.
        """
        raise NotImplementedError

    def poll(self, now_sec: float) -> BehaviorActionFeedback:
        """Return the latest normalized action feedback.

        Args:
            now_sec: Current business timestamp.

        Returns:
            BehaviorActionFeedback reflecting the active command state.

        Raises:
            RuntimeError: If the backend is enabled but has no active command.
        """
        raise NotImplementedError

    def cancel(self, now_sec: float) -> bool:
        """Cancel the active behavior command.

        Args:
            now_sec: Current business timestamp.

        Returns:
            ``True`` when an explicit cancel signal was emitted.
        """
        raise NotImplementedError

    def runtime_binding_summary(self) -> Dict[str, Any]:
        """Return backend binding diagnostics for health/artifact reporting."""
        return {'enabled': bool(self.enabled)}


class DisabledBehaviorActionBackend(BehaviorActionBackendBase):
    """No-op backend used when execute/confirm is disabled for a profile.

    Boundary behavior:
        Dispatch raises immediately because callers should not silently think a
        real behavior action executed when the profile disabled the backend.
    """

    @property
    def enabled(self) -> bool:
        return False

    @property
    def requires_feedback(self) -> bool:
        return False

    def dispatch(self, command: BehaviorActionCommand) -> None:
        raise RuntimeError('behavior action backend is disabled for command {}'.format(command.command_id))

    def poll(self, now_sec: float) -> BehaviorActionFeedback:
        _ = now_sec
        raise RuntimeError('behavior action backend is disabled')

    def cancel(self, now_sec: float) -> bool:
        _ = now_sec
        return False


class MemoryBehaviorActionBackend(BehaviorActionBackendBase):
    """Pure-Python backend for unit tests and deterministic local validation.

    Args:
        outcome_status: Terminal status returned after ``completion_delay_sec``.
        completion_delay_sec: Synthetic completion delay.
        feedback_details: Optional details attached to the terminal feedback.

    Boundary behavior:
        The backend transitions ``PENDING -> ACTIVE -> <terminal>`` based purely
        on business time and therefore does not depend on ROS transport.
    """

    def __init__(self, *, outcome_status: str = 'SUCCEEDED', completion_delay_sec: float = 0.0, feedback_details: Mapping[str, Any] | None = None):
        normalized = str(outcome_status).strip().upper() or 'SUCCEEDED'
        if normalized not in CANONICAL_BEHAVIOR_STATUSES:
            raise ConfigurationError('memory behavior backend outcome_status must be one of {}'.format(', '.join(CANONICAL_BEHAVIOR_STATUSES)))
        self.outcome_status = normalized
        self.completion_delay_sec = max(0.0, float(completion_delay_sec))
        self.feedback_details = dict(feedback_details or {})
        self.active_command: Optional[BehaviorActionCommand] = None
        self.cancelled = False

    def dispatch(self, command: BehaviorActionCommand) -> None:
        self.active_command = command
        self.cancelled = False

    def poll(self, now_sec: float) -> BehaviorActionFeedback:
        if self.active_command is None:
            raise RuntimeError('memory behavior backend has no active command')
        if self.cancelled:
            return BehaviorActionFeedback(self.active_command.command_id, 'FAILED', float(now_sec), {'reason': 'cancelled'}, source='memory')
        elapsed = float(now_sec) - float(self.active_command.issued_at)
        if elapsed < 0.0:
            elapsed = 0.0
        if elapsed == 0.0:
            return BehaviorActionFeedback(self.active_command.command_id, 'PENDING', float(now_sec), {}, source='memory')
        if elapsed < self.completion_delay_sec:
            return BehaviorActionFeedback(self.active_command.command_id, 'ACTIVE', float(now_sec), {}, source='memory')
        return BehaviorActionFeedback(self.active_command.command_id, self.outcome_status, float(now_sec), dict(self.feedback_details), source='memory')

    def cancel(self, now_sec: float) -> bool:
        _ = now_sec
        self.cancelled = True
        return True

    def runtime_binding_summary(self) -> Dict[str, Any]:
        return {
            'enabled': True,
            'backend_type': 'memory',
            'completion_delay_sec': float(self.completion_delay_sec),
            'outcome_status': self.outcome_status,
        }


class TopicBehaviorActionBackend(BehaviorActionBackendBase):
    """ROS topic backend that dispatches JSON commands and consumes JSON feedback.

    Args:
        command_topic: Topic carrying serialized :class:`BehaviorActionCommand`.
        feedback_topic: Topic carrying serialized :class:`BehaviorActionFeedback`.
        timeout_sec: Active command timeout.
        clock: Mission/business clock used for fallback timestamps.
        require_feedback: Whether missing feedback should time out instead of
            being treated as fire-and-forget success.

    Boundary behavior:
        When ``require_feedback`` is false, the backend treats dispatch as
        immediately successful once the command is published. This keeps contract
        smoke profiles deterministic while real deploy profiles can require
        explicit backend receipts.
    """

    def __init__(self, *, command_topic: str, feedback_topic: str, timeout_sec: float, clock, require_feedback: bool):
        import rospy
        from std_msgs.msg import String

        self._rospy = rospy
        self._String = String
        self.command_topic = str(command_topic).strip()
        self.feedback_topic = str(feedback_topic).strip()
        self.timeout_sec = float(timeout_sec)
        if self.timeout_sec <= 0.0:
            raise ConfigurationError('behavior_action_timeout_sec must be > 0')
        if not self.command_topic:
            raise ConfigurationError('behavior_command_topic must not be empty when behavior_action_backend_type=topic')
        if require_feedback and not self.feedback_topic:
            raise ConfigurationError('behavior_feedback_topic must not be empty when require_behavior_feedback=true')
        self.clock = clock
        self._require_feedback = bool(require_feedback)
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=10)
        self.feedback_sub = rospy.Subscriber(self.feedback_topic, String, self._feedback_cb, queue_size=20) if self.feedback_topic else None
        self.active_command: Optional[BehaviorActionCommand] = None
        self.last_feedback: Optional[BehaviorActionFeedback] = None

    @property
    def requires_feedback(self) -> bool:
        return self._require_feedback

    def _feedback_cb(self, msg) -> None:
        import time
        from .common import JsonCodec

        try:
            payload = JsonCodec.loads(msg.data)
        except Exception as exc:
            self._rospy.logwarn_throttle(2.0, 'Ignoring malformed behavior feedback JSON: %s', exc)
            return
        if not isinstance(payload, dict):
            self._rospy.logwarn_throttle(2.0, 'Ignoring non-object behavior feedback payload: %s', type(payload).__name__)
            return
        if self.active_command is None:
            return
        command_id = str(payload.get('command_id', '')).strip()
        if command_id != self.active_command.command_id:
            return
        status = str(payload.get('status', '')).strip().upper() or 'FAILED'
        if status not in CANONICAL_BEHAVIOR_STATUSES:
            self._rospy.logwarn_throttle(2.0, 'Ignoring unsupported behavior feedback status: %s', status)
            return
        stamp = float(payload.get('stamp', 0.0) or 0.0) or float(getattr(self.clock, 'now_business_sec', time.time)())
        self.last_feedback = BehaviorActionFeedback(
            command_id=command_id,
            status=status,
            stamp=stamp,
            details=dict(payload.get('details', {}) or {}),
            source=str(payload.get('source', '')).strip(),
        )

    def dispatch(self, command: BehaviorActionCommand) -> None:
        from .common import JsonCodec

        self.active_command = command
        self.last_feedback = None
        self.command_pub.publish(self._String(data=JsonCodec.dumps(command.to_dict())))

    def poll(self, now_sec: float) -> BehaviorActionFeedback:
        if self.active_command is None:
            raise RuntimeError('behavior action backend has no active command')
        if not self._require_feedback:
            return BehaviorActionFeedback(
                command_id=self.active_command.command_id,
                status='SUCCEEDED',
                stamp=float(now_sec),
                details={'fire_and_forget': True},
                source='topic_backend',
            )
        if self.last_feedback is not None:
            return self.last_feedback
        elapsed = float(now_sec) - float(self.active_command.issued_at)
        if elapsed > self.timeout_sec:
            return BehaviorActionFeedback(
                command_id=self.active_command.command_id,
                status='TIMEOUT',
                stamp=float(now_sec),
                details={'timeout_sec': self.timeout_sec},
                source='topic_backend',
            )
        status = 'PENDING' if elapsed <= 0.0 else 'ACTIVE'
        return BehaviorActionFeedback(
            command_id=self.active_command.command_id,
            status=status,
            stamp=float(now_sec),
            details={},
            source='topic_backend',
        )

    def cancel(self, now_sec: float) -> bool:
        _ = now_sec
        if self.active_command is None:
            return False
        # Topic-based behavior backends treat cancellation as best-effort by
        # publishing a terminal failed receipt for local observability.
        self.last_feedback = BehaviorActionFeedback(
            command_id=self.active_command.command_id,
            status='FAILED',
            stamp=float(now_sec),
            details={'reason': 'cancelled'},
            source='topic_backend',
        )
        return True

    def runtime_binding_summary(self) -> Dict[str, Any]:
        return {
            'enabled': True,
            'backend_type': 'topic',
            'command_topic': self.command_topic,
            'feedback_topic': self.feedback_topic,
            'timeout_sec': float(self.timeout_sec),
            'require_feedback': bool(self._require_feedback),
        }


def build_behavior_action_backend(config: Mapping[str, Any], *, clock) -> BehaviorActionBackendBase:
    """Build the configured behavior backend.

    Args:
        config: Mission/runtime configuration mapping.
        clock: Business-time clock used by active backends.

    Returns:
        Concrete :class:`BehaviorActionBackendBase` implementation.

    Raises:
        ConfigurationError: If the backend type is unsupported.
    """
    backend_type = str(config.get('behavior_action_backend_type', 'disabled')).strip().lower() or 'disabled'
    if backend_type == 'disabled':
        return DisabledBehaviorActionBackend()
    if backend_type == 'memory':
        return MemoryBehaviorActionBackend(
            outcome_status=str(config.get('behavior_action_memory_outcome_status', 'SUCCEEDED')).strip().upper() or 'SUCCEEDED',
            completion_delay_sec=float(config.get('behavior_action_memory_delay_sec', 0.0) or 0.0),
            feedback_details=dict(config.get('behavior_action_memory_feedback_details', {}) or {}),
        )
    if backend_type == 'topic':
        return TopicBehaviorActionBackend(
            command_topic=str(config.get('behavior_command_topic', '')).strip(),
            feedback_topic=str(config.get('behavior_feedback_topic', '')).strip(),
            timeout_sec=float(config.get('behavior_action_timeout_sec', 3.0) or 3.0),
            clock=clock,
            require_feedback=bool(config.get('require_behavior_feedback', True)),
        )
    raise ConfigurationError('behavior_action_backend_type must be one of disabled, memory, topic')

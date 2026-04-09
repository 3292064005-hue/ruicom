"""Pure-Python safety controller for command arbitration and fail-safe logic."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .common import ConfigurationError, SafetyStatus, VALID_CONTROL_MODES, VelocityCommand, clamp


@dataclass
class SourceCommand:
    source_name: str
    priority: int
    stamp: float
    command: VelocityCommand


class SafetyController:
    """Pure-Python arbitration core for multi-source velocity control.

    Args:
        max_linear_x: Maximum permitted x velocity magnitude.
        max_linear_y: Maximum permitted y velocity magnitude.
        max_angular_z: Maximum permitted yaw-rate magnitude.
        command_timeout_sec: Freshness deadline for source commands.
        estop_timeout_sec: Freshness deadline for the estop signal.
        require_fresh_estop: Whether a stale estop signal should stop motion.
        default_mode: Initial control mode.

    Returns:
        A stateful controller that can be fed commands and queried through ``evaluate``.

    Raises:
        ConfigurationError: If limits, timeouts or the default mode are invalid.

    Boundary behavior:
        Stale sources remain stored for observability, but only fresh sources are
        considered during arbitration.
    """

    def __init__(
        self,
        max_linear_x: float,
        max_linear_y: float,
        max_angular_z: float,
        command_timeout_sec: float,
        estop_timeout_sec: float,
        require_fresh_estop: bool,
        default_mode: str = 'AUTO',
    ):
        self.max_linear_x = float(max_linear_x)
        self.max_linear_y = float(max_linear_y)
        self.max_angular_z = float(max_angular_z)
        self.command_timeout_sec = float(command_timeout_sec)
        self.estop_timeout_sec = float(estop_timeout_sec)
        self.require_fresh_estop = bool(require_fresh_estop)
        self.mode = str(default_mode).strip().upper() or 'AUTO'
        if self.mode not in VALID_CONTROL_MODES:
            raise ConfigurationError('default_mode must be one of {}'.format(', '.join(VALID_CONTROL_MODES)))
        if self.command_timeout_sec <= 0.0:
            raise ConfigurationError('command_timeout_sec must be > 0')
        if self.estop_timeout_sec <= 0.0:
            raise ConfigurationError('estop_timeout_sec must be > 0')
        self.estop_active = False
        self.estop_stamp = 0.0
        self.sources: Dict[str, SourceCommand] = {}

    def update_mode(self, mode: str) -> None:
        normalized = str(mode).strip().upper()
        if normalized in VALID_CONTROL_MODES:
            self.mode = normalized

    def update_estop(self, active: bool, stamp: float) -> None:
        self.estop_active = bool(active)
        self.estop_stamp = float(stamp)

    def update_command(self, source_name: str, priority: int, stamp: float, command: VelocityCommand) -> None:
        self.sources[source_name] = SourceCommand(source_name=source_name, priority=int(priority), stamp=float(stamp), command=command)

    def _fresh_estop(self, now_sec: float) -> bool:
        return (now_sec - self.estop_stamp) <= self.estop_timeout_sec

    def _fresh_candidates(self, now_sec: float):
        return [item for item in self.sources.values() if (now_sec - item.stamp) <= self.command_timeout_sec]

    def _best_command(self, now_sec: float) -> Tuple[Optional[SourceCommand], int]:
        candidates = self._fresh_candidates(now_sec)
        if not candidates:
            return None, 0
        candidates.sort(key=lambda item: (-item.priority, -item.stamp))
        return candidates[0], len(candidates)

    def evaluate(self, now_sec: float) -> SafetyStatus:
        fresh_estop = self._fresh_estop(now_sec)
        best, fresh_count = self._best_command(now_sec)
        command_fresh = best is not None
        output = VelocityCommand()
        reason = 'ok'
        selected_source = best.source_name if best else ''
        if self.mode == 'ESTOP':
            reason = 'manual_estop_mode'
        elif self.estop_active:
            reason = 'estop_active'
        elif self.require_fresh_estop and not fresh_estop:
            reason = 'estop_signal_stale'
        elif self.mode not in VALID_CONTROL_MODES:
            reason = 'unknown_mode'
        elif not command_fresh:
            reason = 'command_stale'
        else:
            assert best is not None
            output = VelocityCommand(
                linear_x=clamp(best.command.linear_x, self.max_linear_x),
                linear_y=clamp(best.command.linear_y, self.max_linear_y),
                angular_z=clamp(best.command.angular_z, self.max_angular_z),
            )
        return SafetyStatus(
            stamp=float(now_sec),
            mode=self.mode,
            selected_source=selected_source,
            estop_active=self.estop_active,
            estop_fresh=fresh_estop,
            command_fresh=command_fresh,
            reason=reason,
            output=output,
            observed_source_count=len(self.sources),
            fresh_source_count=fresh_count,
        )




def evaluate_output_feedback_policy(require_output_feedback: bool, output_feedback_topic: str, output_feedback_fresh: bool) -> tuple[bool, str]:
    """Evaluate whether downstream controller feedback allows motion output.

    Args:
        require_output_feedback: Whether downstream feedback is mandatory before
            permitting non-zero motion output.
        output_feedback_topic: Bool feedback topic emitted by the downstream
            controller/driver watchdog.
        output_feedback_fresh: Whether the latest feedback sample is both timely
            and logically healthy.

    Returns:
        A ``(output_allowed, reason)`` tuple. ``reason`` is ``'ok'`` when the
        policy admits motion output and ``'output_feedback_stale'`` when motion
        must be inhibited.

    Raises:
        ConfigurationError: If feedback is required but no feedback topic is
            configured.

    Boundary behavior:
        Optional feedback topics remain observability-only. Required feedback
        converts stale/missing downstream heartbeats into a fail-safe stop.
    """
    normalized_topic = str(output_feedback_topic).strip()
    if require_output_feedback and not normalized_topic:
        raise ConfigurationError('require_output_feedback=true requires output_feedback_topic')
    if require_output_feedback and not output_feedback_fresh:
        return False, 'output_feedback_stale'
    return True, 'ok'


def classify_safety_health(status: SafetyStatus, warn_on_idle_command_stale: bool) -> tuple[str, str]:
    """Map controller state to an externally published health level and message.

    Args:
        status: Evaluated controller status.
        warn_on_idle_command_stale: Whether an operator-idle system without any
            observed commands should still emit a warning.

    Returns:
        A ``(level, message)`` tuple suitable for health publishing.

    Raises:
        No explicit exception is raised.

    Boundary behavior:
        ``command_stale`` is downgraded to ``info`` only when no source command has
        ever been observed and ``warn_on_idle_command_stale`` is disabled.
    """
    if status.reason == 'ok':
        return 'ok', 'ok'
    if status.reason == 'command_stale' and not warn_on_idle_command_stale and status.observed_source_count == 0:
        return 'info', 'command_idle'
    if status.reason in ('manual_estop_mode', 'estop_active', 'estop_signal_stale', 'command_stale', 'unknown_mode'):
        return 'warn', status.reason
    return 'warn', status.reason

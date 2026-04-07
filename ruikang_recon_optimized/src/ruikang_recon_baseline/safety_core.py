"""Pure-Python safety controller for command arbitration and fail-safe logic."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

from .common import ConfigurationError, SafetyStatus, VALID_CONTROL_MODES, VelocityCommand, clamp


@dataclass
class SourceCommand:
    source_name: str
    priority: int
    stamp: float
    command: VelocityCommand


class SafetyController:
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

    def _best_command(self, now_sec: float) -> Optional[SourceCommand]:
        candidates = [item for item in self.sources.values() if (now_sec - item.stamp) <= self.command_timeout_sec]
        if not candidates:
            return None
        candidates.sort(key=lambda item: (-item.priority, -item.stamp))
        return candidates[0]

    def evaluate(self, now_sec: float) -> SafetyStatus:
        fresh_estop = self._fresh_estop(now_sec)
        best = self._best_command(now_sec)
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
        )

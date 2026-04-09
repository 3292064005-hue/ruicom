"""Small reusable lifecycle helpers for managed runtime nodes."""

from __future__ import annotations

from dataclasses import dataclass


_VALID_COMMANDS = {
    'configure': 'configure',
    'activate': 'activate',
    'start': 'activate',
    'resume': 'activate',
    'pause': 'pause',
    'reset': 'reset',
    'shutdown': 'shutdown',
}


@dataclass(frozen=True)
class LifecycleCommandResult:
    """Result of applying one external lifecycle command.

    Attributes:
        accepted: Whether the command changed runtime state or was accepted as a
            harmless no-op.
        state: Normalized runtime lifecycle state after the command.
        message: Machine-readable diagnostic reason.
        changed: Whether the runtime state changed.
    """

    accepted: bool
    state: str
    message: str
    changed: bool


class ManagedRuntimeState:
    """Minimal lifecycle state holder shared by managed runtime nodes.

    The helper intentionally models only node-local execution gating rather than a
    full ROS lifecycle implementation. Nodes start in ``ACTIVE`` when unmanaged and
    in ``IDLE`` when external lifecycle control is enabled. Managed nodes may be
    explicitly ``configure``-ed into ``READY`` before activation so the supervisor
    can stage bring-up in a deterministic order.
    """

    def __init__(self, *, lifecycle_managed: bool):
        self.lifecycle_managed = bool(lifecycle_managed)
        self.state = 'IDLE' if self.lifecycle_managed else 'ACTIVE'

    @property
    def processing_allowed(self) -> bool:
        return self.state == 'ACTIVE'

    @property
    def configured(self) -> bool:
        return self.state in ('READY', 'ACTIVE', 'PAUSED', 'SHUTDOWN')

    def snapshot(self) -> dict:
        return {
            'lifecycle_managed': bool(self.lifecycle_managed),
            'runtime_state': self.state,
            'processing_allowed': self.processing_allowed,
            'configured': self.configured,
        }

    def apply(self, command: str) -> LifecycleCommandResult:
        normalized = _VALID_COMMANDS.get(str(command).strip().lower(), '')
        if not normalized:
            return LifecycleCommandResult(False, self.state, 'unknown_control_command', False)
        if not self.lifecycle_managed:
            return LifecycleCommandResult(False, self.state, 'lifecycle_unmanaged', False)
        if normalized == 'configure':
            if self.state in ('READY', 'ACTIVE', 'PAUSED'):
                return LifecycleCommandResult(True, self.state, 'configure_ignored', False)
            if self.state == 'SHUTDOWN':
                return LifecycleCommandResult(True, self.state, 'configure_ignored', False)
            self.state = 'READY'
            return LifecycleCommandResult(True, self.state, 'runtime_configured', True)
        if normalized == 'activate':
            if self.state == 'ACTIVE':
                return LifecycleCommandResult(True, self.state, 'activate_ignored', False)
            if self.state == 'SHUTDOWN':
                return LifecycleCommandResult(True, self.state, 'activate_ignored', False)
            self.state = 'ACTIVE'
            return LifecycleCommandResult(True, self.state, 'runtime_activated', True)
        if normalized == 'pause':
            if self.state == 'PAUSED':
                return LifecycleCommandResult(True, self.state, 'pause_ignored', False)
            if self.state == 'SHUTDOWN':
                return LifecycleCommandResult(True, self.state, 'pause_ignored', False)
            if self.state == 'IDLE':
                return LifecycleCommandResult(True, self.state, 'pause_ignored', False)
            self.state = 'PAUSED'
            return LifecycleCommandResult(True, self.state, 'runtime_paused', True)
        if normalized == 'reset':
            if self.state == 'IDLE':
                return LifecycleCommandResult(True, self.state, 'reset_ignored', False)
            self.state = 'IDLE'
            return LifecycleCommandResult(True, self.state, 'runtime_reset', True)
        if normalized == 'shutdown':
            if self.state == 'SHUTDOWN':
                return LifecycleCommandResult(True, self.state, 'shutdown_ignored', False)
            self.state = 'SHUTDOWN'
            return LifecycleCommandResult(True, self.state, 'runtime_shutdown', True)
        return LifecycleCommandResult(False, self.state, 'unknown_control_command', False)

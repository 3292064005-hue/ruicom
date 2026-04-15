from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional
from .common import ConfigurationError

@dataclass(frozen=True)
class ActuatorCommand:
    command_id: str
    action_type: str
    task_type: str
    stamp: float
    timeout_sec: float
    actuator_command: Dict[str, Any]
    metadata: Dict[str, Any]

@dataclass(frozen=True)
class ActuatorState:
    command_id: str
    action_type: str
    task_type: str
    state: str
    stamp: float
    details: Dict[str, Any]
    def to_dict(self) -> Dict[str, Any]:
        return {'command_id': self.command_id, 'action_type': self.action_type, 'task_type': self.task_type, 'state': self.state, 'stamp': float(self.stamp), 'details': dict(self.details or {}), 'source': 'behavior_actuator'}

class BehaviorActuatorCore:
    def __init__(self):
        self._active: Optional[ActuatorCommand] = None
    @property
    def active(self) -> Optional[ActuatorCommand]:
        return self._active
    def accept(self, payload: Mapping[str, Any], *, now_sec: float) -> ActuatorState:
        command_id = str(payload.get('command_id','')).strip(); action_type = str(payload.get('action_type','')).strip(); task_type = str(payload.get('task_type', action_type)).strip(); timeout_sec = float(payload.get('timeout_sec',0.0) or 0.0)
        if not command_id: raise ConfigurationError('behavior actuator command requires command_id')
        if not action_type: raise ConfigurationError('behavior actuator command requires action_type')
        if timeout_sec <= 0.0: raise ConfigurationError('behavior actuator command requires timeout_sec > 0')
        if self._active is not None: raise RuntimeError('behavior actuator is busy')
        self._active = ActuatorCommand(command_id=command_id, action_type=action_type, task_type=task_type, stamp=float(now_sec), timeout_sec=float(timeout_sec), actuator_command=dict(payload.get('actuator_command',{}) or {}), metadata=dict(payload.get('metadata',{}) or {}))
        return ActuatorState(command_id=command_id, action_type=action_type, task_type=task_type, state='ACTIVE', stamp=float(now_sec), details={'accepted': True, 'actuator_command': dict(payload.get('actuator_command',{}) or {}), 'metadata': dict(payload.get('metadata',{}) or {})})
    def cancel(self, *, now_sec: float, reason: str) -> Optional[ActuatorState]:
        active=self._active
        if active is None: return None
        self._active=None
        return ActuatorState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='FAILED', stamp=float(now_sec), details={'reason': str(reason).strip() or 'cancelled'})
    def heartbeat_state(self, *, now_sec: float) -> Optional[ActuatorState]:
        active=self._active
        if active is None: return None
        return ActuatorState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='ACTIVE', stamp=float(now_sec), details={'actuator_command': dict(active.actuator_command), 'metadata': dict(active.metadata)})
    def maybe_timeout(self, *, now_sec: float) -> Optional[ActuatorState]:
        active=self._active
        if active is None or float(now_sec) <= float(active.stamp)+float(active.timeout_sec): return None
        self._active=None
        return ActuatorState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='FAILED', stamp=float(now_sec), details={'reason': 'downstream_feedback_timeout', 'timeout_sec': float(active.timeout_sec)})
    def observe_vendor_feedback(self, payload: Mapping[str, Any], *, now_sec: float) -> Optional[ActuatorState]:
        active=self._active
        if active is None or str(payload.get('command_id','')).strip() != active.command_id: return None
        status = str(payload.get('status','')).strip().upper(); details=dict(payload.get('details',{}) or {})
        if status in ('SUCCEEDED','COMPLETED'):
            self._active=None
            return ActuatorState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='COMPLETED', stamp=float(now_sec), details={**details, 'downstream_status': status})
        if status in ('FAILED','TIMEOUT','CANCELLED'):
            self._active=None
            return ActuatorState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='FAILED', stamp=float(now_sec), details={**details, 'downstream_status': status, 'reason': str(details.get('reason', status.lower())).strip() or status.lower()})
        return None

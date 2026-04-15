from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional
from .common import ConfigurationError

@dataclass(frozen=True)
class VendorActuatorBridgeCommand:
    command_id: str
    action_type: str
    task_type: str
    stamp: float
    timeout_sec: float
    actuator_command: Dict[str, Any]
    metadata: Dict[str, Any]

@dataclass(frozen=True)
class VendorActuatorBridgeState:
    command_id: str
    action_type: str
    task_type: str
    state: str
    stamp: float
    details: Dict[str, Any]
    def to_dict(self) -> Dict[str, Any]:
        return {
            'command_id': self.command_id,
            'action_type': self.action_type,
            'task_type': self.task_type,
            'state': self.state,
            'stamp': float(self.stamp),
            'details': dict(self.details or {}),
            'source': 'vendor_actuator_bridge',
        }

class VendorActuatorBridgeCore:
    def __init__(self):
        self._active: Optional[VendorActuatorBridgeCommand] = None

    @property
    def active(self) -> Optional[VendorActuatorBridgeCommand]:
        return self._active

    def accept(self, payload: Mapping[str, Any], *, now_sec: float) -> VendorActuatorBridgeState:
        command_id = str(payload.get('command_id', '')).strip()
        action_type = str(payload.get('action_type', '')).strip()
        task_type = str(payload.get('task_type', action_type)).strip()
        timeout_sec = float(payload.get('timeout_sec', 0.0) or 0.0)
        if not command_id:
            raise ConfigurationError('vendor actuator bridge command requires command_id')
        if not action_type:
            raise ConfigurationError('vendor actuator bridge command requires action_type')
        if timeout_sec <= 0.0:
            raise ConfigurationError('vendor actuator bridge command requires timeout_sec > 0')
        if self._active is not None:
            raise RuntimeError('vendor actuator bridge is busy')
        self._active = VendorActuatorBridgeCommand(
            command_id=command_id,
            action_type=action_type,
            task_type=task_type,
            stamp=float(now_sec),
            timeout_sec=float(timeout_sec),
            actuator_command=dict(payload.get('actuator_command', {}) or {}),
            metadata=dict(payload.get('metadata', {}) or {}),
        )
        return VendorActuatorBridgeState(command_id=command_id, action_type=action_type, task_type=task_type, state='FORWARDED', stamp=float(now_sec), details={'accepted': True})

    def heartbeat_state(self, *, now_sec: float) -> Optional[VendorActuatorBridgeState]:
        active = self._active
        if active is None:
            return None
        return VendorActuatorBridgeState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='FORWARDED', stamp=float(now_sec), details={'actuator_command': dict(active.actuator_command), 'metadata': dict(active.metadata)})

    def maybe_timeout(self, *, now_sec: float) -> Optional[VendorActuatorBridgeState]:
        active = self._active
        if active is None or float(now_sec) <= float(active.stamp) + float(active.timeout_sec):
            return None
        self._active = None
        return VendorActuatorBridgeState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='FAILED', stamp=float(now_sec), details={'reason': 'raw_command_output_timeout', 'timeout_sec': float(active.timeout_sec)})

    def observe_raw_result(self, payload: Mapping[str, Any], *, now_sec: float) -> Optional[VendorActuatorBridgeState]:
        active = self._active
        if active is None or str(payload.get('command_id', '')).strip() != active.command_id:
            return None
        status = str(payload.get('status', '')).strip().upper()
        if status in ('SUCCEEDED', 'COMPLETED', 'FAILED', 'TIMEOUT', 'CANCELLED'):
            self._active = None
            state = 'COMPLETED' if status in ('SUCCEEDED', 'COMPLETED') else 'FAILED'
            return VendorActuatorBridgeState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state=state, stamp=float(now_sec), details={'downstream_status': status, **dict(payload.get('details', {}) or {})})
        return None

    def cancel(self, *, now_sec: float, reason: str) -> Optional[VendorActuatorBridgeState]:
        active = self._active
        if active is None:
            return None
        self._active = None
        return VendorActuatorBridgeState(command_id=active.command_id, action_type=active.action_type, task_type=active.task_type, state='FAILED', stamp=float(now_sec), details={'reason': str(reason).strip() or 'cancelled'})

from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional
from .common import ConfigurationError

@dataclass(frozen=True)
class VendorActuatorCommand:
    command_id: str
    action_type: str
    task_type: str
    stamp: float
    timeout_sec: float
    actuator_command: Dict[str, Any]
    metadata: Dict[str, Any]

class VendorActuatorFeedbackCore:
    def __init__(self):
        self._active: Optional[VendorActuatorCommand] = None
        self._last_result: Optional[Dict[str, Any]] = None
        self._last_result_stamp = 0.0

    @property
    def active(self) -> Optional[VendorActuatorCommand]:
        return self._active

    def accept(self, payload: Mapping[str, Any], *, now_sec: float) -> Dict[str, Any]:
        command_id = str(payload.get('command_id', '')).strip()
        action_type = str(payload.get('action_type', '')).strip()
        task_type = str(payload.get('task_type', action_type)).strip()
        timeout_sec = float(payload.get('timeout_sec', 0.0) or 0.0)
        if not command_id:
            raise ConfigurationError('vendor actuator command requires command_id')
        if not action_type:
            raise ConfigurationError('vendor actuator command requires action_type')
        if timeout_sec <= 0.0:
            raise ConfigurationError('vendor actuator command requires timeout_sec > 0')
        if self._active is not None:
            raise RuntimeError('vendor actuator feedback runtime is busy')
        self._active = VendorActuatorCommand(command_id=command_id, action_type=action_type, task_type=task_type, stamp=float(now_sec), timeout_sec=float(timeout_sec), actuator_command=dict(payload.get('actuator_command', {}) or {}), metadata=dict(payload.get('metadata', {}) or {}))
        return {'command_id': command_id, 'action_type': action_type, 'task_type': task_type, 'status': 'ACTIVE', 'stamp': float(now_sec), 'details': {'accepted': True}, 'source': 'vendor_actuator_feedback'}

    def observe_result(self, payload: Mapping[str, Any], *, now_sec: float) -> None:
        self._last_result = {'command_id': str(payload.get('command_id', '')).strip(), 'status': str(payload.get('status', '')).strip().upper(), 'details': dict(payload.get('details', {}) or {}), 'stamp': float(payload.get('stamp', now_sec) or now_sec)}
        self._last_result_stamp = float(now_sec)

    def evaluate(self, *, now_sec: float, result_timeout_sec: float) -> Optional[Dict[str, Any]]:
        active = self._active
        if active is None:
            return None
        result = self._last_result or {}
        result_command_id = str(result.get('command_id', '')).strip()
        result_status = str(result.get('status', '')).strip().upper()
        result_stamp = float(result.get('stamp', 0.0) or 0.0)
        if result_command_id == active.command_id and result_stamp >= float(active.stamp) and (float(now_sec) - self._last_result_stamp) <= float(result_timeout_sec):
            if result_status in ('SUCCEEDED', 'COMPLETED'):
                self._active = None
                return {'command_id': active.command_id, 'action_type': active.action_type, 'task_type': active.task_type, 'status': 'SUCCEEDED', 'stamp': float(now_sec), 'details': {'reason': 'actuator_result_confirmed', 'result_stamp': float(result_stamp), **dict(result.get('details', {}) or {})}, 'source': 'vendor_actuator_feedback'}
            if result_status in ('FAILED', 'TIMEOUT', 'CANCELLED'):
                self._active = None
                details = dict(result.get('details', {}) or {})
                return {'command_id': active.command_id, 'action_type': active.action_type, 'task_type': active.task_type, 'status': 'FAILED', 'stamp': float(now_sec), 'details': {'reason': str(details.get('reason', result_status.lower())).strip() or result_status.lower(), 'result_stamp': float(result_stamp), **details}, 'source': 'vendor_actuator_feedback'}
        if float(now_sec) > float(active.stamp) + float(active.timeout_sec):
            self._active = None
            return {'command_id': active.command_id, 'action_type': active.action_type, 'task_type': active.task_type, 'status': 'FAILED', 'stamp': float(now_sec), 'details': {'reason': 'vendor_actuator_timeout', 'timeout_sec': float(active.timeout_sec)}, 'source': 'vendor_actuator_feedback'}
        return None

    def cancel(self, *, now_sec: float, reason: str) -> Optional[Dict[str, Any]]:
        active = self._active
        if active is None:
            return None
        self._active = None
        return {'command_id': active.command_id, 'action_type': active.action_type, 'task_type': active.task_type, 'status': 'FAILED', 'stamp': float(now_sec), 'details': {'reason': str(reason).strip() or 'cancelled'}, 'source': 'vendor_actuator_feedback'}

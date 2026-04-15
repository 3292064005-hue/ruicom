from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional
from .common import ConfigurationError

_ALLOWED_DEVICE_MODES = {'external_ack', 'loopback'}
_ALLOWED_RESULT_STATUS = {'SUCCEEDED', 'COMPLETED', 'FAILED', 'TIMEOUT', 'CANCELLED'}

@dataclass(frozen=True)
class VendorActuatorRawCommand:
    command_id: str
    action_type: str
    task_type: str
    stamp: float
    timeout_sec: float
    payload: Dict[str, Any]


class VendorActuatorDeviceCore:
    def __init__(self, *, device_mode: str = 'external_ack', loopback_completion_delay_sec: float = 0.2):
        normalized = str(device_mode).strip().lower() or 'external_ack'
        if normalized not in _ALLOWED_DEVICE_MODES:
            raise ConfigurationError(f'vendor actuator device_mode must be one of {sorted(_ALLOWED_DEVICE_MODES)}')
        delay = float(loopback_completion_delay_sec)
        if delay <= 0.0:
            raise ConfigurationError('vendor actuator loopback_completion_delay_sec must be > 0')
        self.device_mode = normalized
        self.loopback_completion_delay_sec = delay
        self._active: Optional[VendorActuatorRawCommand] = None
        self._last_ack: Optional[Dict[str, Any]] = None
        self._last_ack_observed_at = 0.0

    @property
    def active(self) -> Optional[VendorActuatorRawCommand]:
        return self._active

    def accept(self, payload: Mapping[str, Any], *, now_sec: float) -> Dict[str, Any]:
        command_id = str(payload.get('command_id', '')).strip()
        action_type = str(payload.get('action_type', '')).strip()
        task_type = str(payload.get('task_type', action_type)).strip()
        timeout_sec = float(payload.get('timeout_sec', 0.0) or 0.0)
        if not command_id:
            raise ConfigurationError('vendor actuator raw command requires command_id')
        if not action_type:
            raise ConfigurationError('vendor actuator raw command requires action_type')
        if timeout_sec <= 0.0:
            raise ConfigurationError('vendor actuator raw command requires timeout_sec > 0')
        if self._active is not None:
            raise RuntimeError('vendor actuator device is busy')
        self._active = VendorActuatorRawCommand(
            command_id=command_id,
            action_type=action_type,
            task_type=task_type,
            stamp=float(now_sec),
            timeout_sec=timeout_sec,
            payload=dict(payload),
        )
        return {
            'command_id': command_id,
            'state': 'ACTIVE',
            'stamp': float(now_sec),
            'details': {'device_mode': self.device_mode},
            'source': 'vendor_actuator_device',
        }

    def observe_ack(self, payload: Mapping[str, Any], *, now_sec: float) -> None:
        status = str(payload.get('status', '')).strip().upper()
        if status and status not in _ALLOWED_RESULT_STATUS:
            raise ConfigurationError(f'vendor actuator ack status must be one of {sorted(_ALLOWED_RESULT_STATUS)}')
        self._last_ack = {
            'command_id': str(payload.get('command_id', '')).strip(),
            'status': status,
            'stamp': float(payload.get('stamp', now_sec) or now_sec),
            'details': dict(payload.get('details', {}) or {}),
        }
        self._last_ack_observed_at = float(now_sec)

    def evaluate(self, *, now_sec: float, ack_freshness_sec: float) -> Optional[Dict[str, Any]]:
        active = self._active
        if active is None:
            return None
        if float(now_sec) > float(active.stamp) + float(active.timeout_sec):
            self._active = None
            return {
                'command_id': active.command_id,
                'status': 'FAILED',
                'stamp': float(now_sec),
                'details': {'reason': 'actuator_device_timeout', 'timeout_sec': float(active.timeout_sec)},
                'source': 'vendor_actuator_device',
            }
        if self.device_mode == 'loopback':
            if float(now_sec) >= float(active.stamp) + float(self.loopback_completion_delay_sec):
                self._active = None
                return {
                    'command_id': active.command_id,
                    'status': 'SUCCEEDED',
                    'stamp': float(now_sec),
                    'details': {'reason': 'loopback_completion', 'device_mode': self.device_mode},
                    'source': 'vendor_actuator_device',
                }
            return None
        ack = self._last_ack or {}
        if (
            str(ack.get('command_id', '')).strip() == active.command_id
            and float(ack.get('stamp', 0.0) or 0.0) >= float(active.stamp)
            and (float(now_sec) - float(self._last_ack_observed_at)) <= float(ack_freshness_sec)
        ):
            status = str(ack.get('status', '')).strip().upper() or 'FAILED'
            self._active = None
            if status in ('SUCCEEDED', 'COMPLETED'):
                return {
                    'command_id': active.command_id,
                    'status': 'SUCCEEDED',
                    'stamp': float(now_sec),
                    'details': {'reason': 'device_ack_confirmed', **dict(ack.get('details', {}) or {})},
                    'source': 'vendor_actuator_device',
                }
            return {
                'command_id': active.command_id,
                'status': 'FAILED',
                'stamp': float(now_sec),
                'details': {'reason': str(dict(ack.get('details', {}) or {}).get('reason', status.lower())).strip() or status.lower(), **dict(ack.get('details', {}) or {})},
                'source': 'vendor_actuator_device',
            }
        return None

    def cancel(self, *, now_sec: float, reason: str) -> Optional[Dict[str, Any]]:
        active = self._active
        if active is None:
            return None
        self._active = None
        return {
            'command_id': active.command_id,
            'status': 'FAILED',
            'stamp': float(now_sec),
            'details': {'reason': str(reason).strip() or 'cancelled'},
            'source': 'vendor_actuator_device',
        }

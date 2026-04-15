from __future__ import annotations

"""Shared upper-computer/baseboard serial protocol helpers for MO-SERGEANT.

This module intentionally supports two wire formats:

- ``legacy_newt`` keeps compatibility with the vendor ROS example where
  ``newt.py`` subscribes to ``/cmd_vel`` and sends a velocity packet over
  ``/dev/carserial``.
- ``managed_crc16`` adds structured telemetry/ack frames suitable for the
  repository-managed runtime and the AT32F403AVCT7 baseboard.
"""

from dataclasses import dataclass
import struct
from typing import Iterable, Optional

LEGACY_HEADER = b'\xAA\xBB\x0A\x12\x02'
LEGACY_INIT = b'\x11\x00\x00\x00\x00\x00\x00\x00\x00'
MANAGED_SOF = 0xAA55
MANAGED_EOF = 0x0D0A
TYPE_VELOCITY_COMMAND = 0x01
TYPE_ACTUATOR_COMMAND = 0x02
TYPE_CHASSIS_TELEMETRY = 0x81
TYPE_ACTUATOR_ACK = 0x82
PROTOCOL_MODES = ('legacy_newt', 'managed_crc16')


@dataclass(frozen=True)
class ChassisVelocityCommand:
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float
    sequence: int = 0


@dataclass(frozen=True)
class ChassisTelemetry:
    sequence: int
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float
    battery_voltage_v: float
    estop: bool
    heartbeat_ok: bool


@dataclass(frozen=True)
class ActuatorAck:
    sequence: int
    command_id: str
    success: bool
    detail_code: int


class ProtocolError(ValueError):
    pass


def _crc16_ibm(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _int16_le(value: int) -> bytes:
    return struct.pack('<h', int(value))


def _u16_le(value: int) -> bytes:
    return struct.pack('<H', int(value) & 0xFFFF)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))


def _quantize_velocity_mm(value_mps: float) -> int:
    return int(round(_clamp(value_mps, -1.5, 1.5) * 1000.0))


def _quantize_yaw_mrad(value_rps: float) -> int:
    return int(round(_clamp(value_rps, -4.0, 4.0) * 1000.0))


def pack_legacy_init_frame() -> bytes:
    return LEGACY_INIT


def pack_legacy_velocity_frame(command: ChassisVelocityCommand) -> bytes:
    """Encode the vendor-compatible velocity packet.

    Input values are clamped, converted to mm/s or mrad/s, and packed in the
    little-endian byte order expected by the vendor chassis bridge example.
    """

    vx = _quantize_velocity_mm(command.linear_x_mps)
    vy = _quantize_velocity_mm(command.linear_y_mps)
    wz = _quantize_yaw_mrad(command.angular_z_rps)
    return LEGACY_HEADER + _int16_le(vx) + _int16_le(vy) + _int16_le(wz) + b'\x00'


def pack_managed_velocity_frame(command: ChassisVelocityCommand) -> bytes:
    payload = _u16_le(command.sequence) + _int16_le(_quantize_velocity_mm(command.linear_x_mps)) + _int16_le(_quantize_velocity_mm(command.linear_y_mps)) + _int16_le(_quantize_yaw_mrad(command.angular_z_rps))
    body = _u16_le(MANAGED_SOF) + bytes([TYPE_VELOCITY_COMMAND, len(payload)]) + payload
    crc = _u16_le(_crc16_ibm(body[2:]))
    return body + crc + _u16_le(MANAGED_EOF)


def pack_managed_actuator_frame(sequence: int, command_id: str, command_name: str, intensity: int = 1) -> bytes:
    encoded_id = str(command_id).encode('utf-8')[:32]
    encoded_name = str(command_name).encode('utf-8')[:16]
    payload = _u16_le(sequence) + bytes([len(encoded_id)]) + encoded_id + bytes([len(encoded_name)]) + encoded_name + bytes([max(0, min(255, int(intensity)))])
    body = _u16_le(MANAGED_SOF) + bytes([TYPE_ACTUATOR_COMMAND, len(payload)]) + payload
    crc = _u16_le(_crc16_ibm(body[2:]))
    return body + crc + _u16_le(MANAGED_EOF)


def parse_managed_frame(frame: bytes) -> tuple[int, bytes]:
    if len(frame) < 8:
        raise ProtocolError('managed frame too short')
    sof, frame_type, payload_len = struct.unpack('<HBB', frame[:4])
    if sof != MANAGED_SOF:
        raise ProtocolError('invalid managed SOF')
    expected_len = 4 + payload_len + 2 + 2
    if len(frame) != expected_len:
        raise ProtocolError(f'managed frame length mismatch: expected {expected_len} got {len(frame)}')
    payload = frame[4:4 + payload_len]
    crc_rx = struct.unpack('<H', frame[4 + payload_len:6 + payload_len])[0]
    eof = struct.unpack('<H', frame[6 + payload_len:8 + payload_len])[0]
    if eof != MANAGED_EOF:
        raise ProtocolError('invalid managed EOF')
    crc_expected = _crc16_ibm(frame[2:4 + payload_len])
    if crc_rx != crc_expected:
        raise ProtocolError(f'crc mismatch: expected {crc_expected:#06x} got {crc_rx:#06x}')
    return frame_type, payload


def decode_managed_telemetry(frame: bytes) -> ChassisTelemetry:
    frame_type, payload = parse_managed_frame(frame)
    if frame_type != TYPE_CHASSIS_TELEMETRY:
        raise ProtocolError(f'expected telemetry frame got type {frame_type:#x}')
    if len(payload) != 11:
        raise ProtocolError('telemetry payload must be 11 bytes')
    seq, vx, vy, wz, battery_mv, flags = struct.unpack('<HhhhHB', payload)
    return ChassisTelemetry(
        sequence=int(seq),
        linear_x_mps=float(vx) / 1000.0,
        linear_y_mps=float(vy) / 1000.0,
        angular_z_rps=float(wz) / 1000.0,
        battery_voltage_v=float(battery_mv) / 1000.0,
        estop=bool(flags & 0x01),
        heartbeat_ok=bool(flags & 0x02),
    )


def decode_managed_actuator_ack(frame: bytes) -> ActuatorAck:
    frame_type, payload = parse_managed_frame(frame)
    if frame_type != TYPE_ACTUATOR_ACK:
        raise ProtocolError(f'expected actuator ack frame got type {frame_type:#x}')
    if len(payload) < 5:
        raise ProtocolError('actuator ack payload too short')
    seq = struct.unpack('<H', payload[:2])[0]
    id_len = payload[2]
    if len(payload) < 3 + id_len + 2:
        raise ProtocolError('actuator ack payload malformed')
    command_id = payload[3:3 + id_len].decode('utf-8', errors='replace')
    success = bool(payload[3 + id_len])
    detail_code = int(payload[4 + id_len])
    return ActuatorAck(sequence=int(seq), command_id=command_id, success=success, detail_code=detail_code)


def iter_managed_frames(buffer: bytes) -> Iterable[bytes]:
    data = bytes(buffer)
    idx = 0
    while idx + 8 <= len(data):
        if data[idx:idx + 2] != _u16_le(MANAGED_SOF):
            idx += 1
            continue
        payload_len = data[idx + 3]
        frame_len = 4 + payload_len + 2 + 2
        if idx + frame_len > len(data):
            break
        yield data[idx:idx + frame_len]
        idx += frame_len

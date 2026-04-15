import unittest

from ruikang_recon_baseline.mowen_serial_protocol import (
    ActuatorAck,
    ChassisVelocityCommand,
    TYPE_ACTUATOR_ACK,
    TYPE_CHASSIS_TELEMETRY,
    _crc16_ibm,
    decode_managed_actuator_ack,
    decode_managed_telemetry,
    pack_legacy_velocity_frame,
    pack_managed_velocity_frame,
)


class MowenSerialProtocolTest(unittest.TestCase):
    def test_pack_legacy_velocity_frame_matches_vendor_layout(self):
        frame = pack_legacy_velocity_frame(ChassisVelocityCommand(0.1, -0.2, 0.3))
        self.assertEqual(frame[:5], b'\xAA\xBB\x0A\x12\x02')
        self.assertEqual(len(frame), 12)

    def test_pack_and_decode_managed_telemetry(self):
        payload = b'\x01\x00' + b'\x64\x00' + b'\x9c\xff' + b'\x2c\x01' + b'\x88\x13' + b'\x03'
        body = b'\x55\xAA' + bytes([TYPE_CHASSIS_TELEMETRY, len(payload)]) + payload
        crc = _crc16_ibm(body[2:])
        frame = body + crc.to_bytes(2, 'little') + b'\x0A\x0D'
        telemetry = decode_managed_telemetry(frame)
        self.assertAlmostEqual(telemetry.linear_x_mps, 0.1)
        self.assertAlmostEqual(telemetry.linear_y_mps, -0.1)
        self.assertAlmostEqual(telemetry.angular_z_rps, 0.3)
        self.assertAlmostEqual(telemetry.battery_voltage_v, 5.0)
        self.assertTrue(telemetry.estop)
        self.assertTrue(telemetry.heartbeat_ok)

    def test_decode_managed_actuator_ack(self):
        command_id = b'attack-01'
        payload = b'\x02\x00' + bytes([len(command_id)]) + command_id + b'\x01\x07'
        body = b'\x55\xAA' + bytes([TYPE_ACTUATOR_ACK, len(payload)]) + payload
        crc = _crc16_ibm(body[2:])
        frame = body + crc.to_bytes(2, 'little') + b'\x0A\x0D'
        ack = decode_managed_actuator_ack(frame)
        self.assertEqual(ack, ActuatorAck(sequence=2, command_id='attack-01', success=True, detail_code=7))


if __name__ == '__main__':
    unittest.main()

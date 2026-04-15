import unittest
from ruikang_recon_baseline.vendor_actuator_device_core import VendorActuatorDeviceCore


class VendorActuatorDeviceCoreTest(unittest.TestCase):
    def test_loopback_mode_emits_success_after_delay(self):
        core = VendorActuatorDeviceCore(device_mode='loopback', loopback_completion_delay_sec=0.1)
        core.accept({'command_id': 'cmd-1', 'action_type': 'facility_attack', 'task_type': 'facility_attack', 'timeout_sec': 1.0}, now_sec=0.0)
        self.assertIsNone(core.evaluate(now_sec=0.05, ack_freshness_sec=0.5))
        result = core.evaluate(now_sec=0.11, ack_freshness_sec=0.5)
        self.assertEqual(result['status'], 'SUCCEEDED')
        self.assertEqual(result['details']['reason'], 'loopback_completion')

    def test_external_ack_mode_requires_matching_ack(self):
        core = VendorActuatorDeviceCore(device_mode='external_ack', loopback_completion_delay_sec=0.1)
        core.accept({'command_id': 'cmd-2', 'action_type': 'facility_attack', 'task_type': 'facility_attack', 'timeout_sec': 1.0}, now_sec=1.0)
        self.assertIsNone(core.evaluate(now_sec=1.1, ack_freshness_sec=0.5))
        core.observe_ack({'command_id': 'cmd-2', 'status': 'SUCCEEDED', 'stamp': 1.2, 'details': {'device': 'unit'}}, now_sec=1.21)
        result = core.evaluate(now_sec=1.22, ack_freshness_sec=0.5)
        self.assertEqual(result['status'], 'SUCCEEDED')
        self.assertEqual(result['details']['reason'], 'device_ack_confirmed')

if __name__ == '__main__':
    unittest.main()

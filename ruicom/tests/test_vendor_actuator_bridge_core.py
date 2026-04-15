import unittest

from ruikang_recon_baseline.vendor_actuator_bridge_core import VendorActuatorBridgeCore


class VendorActuatorBridgeCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = VendorActuatorBridgeCore()

    def _payload(self, command_id='cmd-1', timeout_sec=0.2):
        return {
            'command_id': command_id,
            'action_type': 'facility_attack',
            'task_type': 'facility_attack',
            'timeout_sec': timeout_sec,
            'actuator_command': {'command_name': 'facility_attack'},
        }

    def test_accept_and_complete_on_raw_result(self):
        state = self.core.accept(self._payload(timeout_sec=1.0), now_sec=10.0)
        self.assertEqual(state.state, 'FORWARDED')
        self.assertIsNone(self.core.observe_raw_result({'command_id': 'other', 'status': 'SUCCEEDED'}, now_sec=10.1))
        done = self.core.observe_raw_result({'command_id': 'cmd-1', 'status': 'SUCCEEDED', 'details': {'driver': 'stub'}}, now_sec=10.2)
        self.assertIsNotNone(done)
        self.assertEqual(done.state, 'COMPLETED')

    def test_timeout(self):
        self.core.accept(self._payload(timeout_sec=0.1), now_sec=10.0)
        done = self.core.maybe_timeout(now_sec=10.2)
        self.assertEqual(done.state, 'FAILED')

if __name__ == '__main__':
    unittest.main()

import unittest

from ruikang_recon_baseline.vendor_actuator_feedback_core import VendorActuatorFeedbackCore


class VendorActuatorFeedbackCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = VendorActuatorFeedbackCore()

    def _payload(self, command_id='cmd-1', timeout_sec=0.2):
        return {
            'command_id': command_id,
            'action_type': 'facility_attack',
            'task_type': 'facility_attack',
            'timeout_sec': timeout_sec,
            'actuator_command': {'command_name': 'facility_attack'},
        }

    def test_requires_result_after_accept(self):
        self.core.observe_result({'command_id': 'cmd-1', 'status': 'SUCCEEDED', 'stamp': 9.0, 'details': {}}, now_sec=9.0)
        self.core.accept(self._payload(timeout_sec=1.0), now_sec=10.0)
        self.assertIsNone(self.core.evaluate(now_sec=10.1, result_timeout_sec=0.5))
        self.core.observe_result({'command_id': 'cmd-1', 'status': 'SUCCEEDED', 'stamp': 10.2, 'details': {'driver': 'stub'}}, now_sec=10.2)
        terminal = self.core.evaluate(now_sec=10.21, result_timeout_sec=0.5)
        self.assertIsNotNone(terminal)
        self.assertEqual(terminal['status'], 'SUCCEEDED')
        self.assertEqual(terminal['details']['reason'], 'actuator_result_confirmed')

    def test_propagates_failure_from_result(self):
        self.core.accept(self._payload(timeout_sec=1.0), now_sec=10.0)
        self.core.observe_result({'command_id': 'cmd-1', 'status': 'FAILED', 'stamp': 10.1, 'details': {'reason': 'device_rejected'}}, now_sec=10.1)
        terminal = self.core.evaluate(now_sec=10.11, result_timeout_sec=0.5)
        self.assertEqual(terminal['status'], 'FAILED')
        self.assertEqual(terminal['details']['reason'], 'device_rejected')

    def test_times_out_without_result(self):
        self.core.accept(self._payload(timeout_sec=0.1), now_sec=10.0)
        terminal = self.core.evaluate(now_sec=10.2, result_timeout_sec=0.5)
        self.assertIsNotNone(terminal)
        self.assertEqual(terminal['status'], 'FAILED')


if __name__ == '__main__':
    unittest.main()

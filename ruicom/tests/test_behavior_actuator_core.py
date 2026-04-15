import unittest

from ruikang_recon_baseline.behavior_actuator_core import BehaviorActuatorCore


class BehaviorActuatorCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = BehaviorActuatorCore()

    def _payload(self, command_id='cmd-1', action_type='facility_attack', timeout_sec=0.2):
        return {
            'command_id': command_id,
            'action_type': action_type,
            'task_type': action_type,
            'stamp': 10.0,
            'timeout_sec': timeout_sec,
            'actuator_command': {'command_name': action_type},
            'metadata': {'zone': 'zone_c'},
        }

    def test_accept_creates_active_state(self):
        state = self.core.accept(self._payload(), now_sec=10.0)
        self.assertEqual(state.state, 'ACTIVE')
        self.assertEqual(state.command_id, 'cmd-1')

    def test_complete_on_vendor_feedback(self):
        self.core.accept(self._payload(timeout_sec=1.0), now_sec=10.0)
        self.assertIsNone(self.core.observe_vendor_feedback({'command_id': 'other', 'status': 'SUCCEEDED', 'details': {}}, now_sec=10.05))
        done = self.core.observe_vendor_feedback({'command_id': 'cmd-1', 'status': 'SUCCEEDED', 'details': {'ack': True}}, now_sec=10.2)
        self.assertIsNotNone(done)
        self.assertEqual(done.state, 'COMPLETED')
        self.assertEqual(done.details['downstream_status'], 'SUCCEEDED')

    def test_timeout_returns_failed(self):
        self.core.accept(self._payload(timeout_sec=0.1), now_sec=10.0)
        self.assertIsNone(self.core.maybe_timeout(now_sec=10.05))
        done = self.core.maybe_timeout(now_sec=10.2)
        self.assertIsNotNone(done)
        self.assertEqual(done.state, 'FAILED')
        self.assertEqual(done.details['reason'], 'downstream_feedback_timeout')

    def test_cancel_returns_failed(self):
        self.core.accept(self._payload(), now_sec=10.0)
        state = self.core.cancel(now_sec=10.1, reason='operator_stop')
        self.assertIsNotNone(state)
        self.assertEqual(state.state, 'FAILED')
        self.assertEqual(state.details['reason'], 'operator_stop')


if __name__ == '__main__':
    unittest.main()

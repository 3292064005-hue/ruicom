import unittest

from ruikang_recon_baseline.behavior_runtime_core import BehaviorRuntimeCore, build_behavior_runtime_specs


class BehaviorRuntimeCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = BehaviorRuntimeCore(action_specs=build_behavior_runtime_specs({
            'default_command_hold_sec': 0.4,
            'default_settle_sec': 0.1,
            'action_specs': {
                'hazard_avoid': {
                    'success_policy': 'platform_clear',
                    'require_platform_feedback': True,
                    'require_ultrasonic_clear': True,
                    'command_velocity': {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.3},
                },
                'facility_attack': {
                    'success_policy': 'detection_confirmed',
                    'require_platform_feedback': True,
                    'confirmation_classes': ['enemy'],
                    'required_detection_count': 1,
                    'require_actuator_state': True,
                },
            },
        }))

    def _request(self, action_type='hazard_avoid', command_id='cmd-1'):
        return {
            'command': {
                'command_id': command_id,
                'action_type': action_type,
                'task_type': action_type,
                'timeout_sec': 2.0,
                'metadata': {'confirmation_classes': ['enemy'], 'required_detection_count': 1},
            }
        }

    def test_hazard_action_requires_platform_and_clearance(self):
        dispatch = self.core.accept_request(self._request('hazard_avoid'), now_sec=10.0)
        self.assertEqual(dispatch.command_payload['velocity']['angular_z'], 0.3)
        self.assertIsNone(self.core.evaluate(now_sec=10.3, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': True, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True, 'ultrasonic_hazard_active': True}, detection_counts={}, detection_fresh=False))
        self.assertIsNone(self.core.evaluate(now_sec=10.5, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': True, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True, 'ultrasonic_hazard_active': False}, detection_counts={}, detection_fresh=False))
        result = self.core.evaluate(now_sec=10.7, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': True, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True, 'ultrasonic_hazard_active': False}, detection_counts={}, detection_fresh=False)
        self.assertEqual(result['status'], 'SUCCEEDED')

    def test_facility_action_requires_detection_confirmation(self):
        self.core.accept_request(self._request('facility_attack'), now_sec=10.0)
        self.assertIsNone(self.core.evaluate(now_sec=10.4, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': True, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True}, detection_counts={'enemy': 0}, detection_fresh=True, actuator_state={'command_id': 'cmd-1', 'state': 'ACTIVE'}, actuator_state_fresh=True))
        self.assertIsNone(self.core.evaluate(now_sec=10.5, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': True, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True}, detection_counts={'enemy': 1}, detection_fresh=True, actuator_state={'command_id': 'cmd-1', 'state': 'ACTIVE'}, actuator_state_fresh=True))
        result = self.core.evaluate(now_sec=10.7, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': True, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True}, detection_counts={'enemy': 1}, detection_fresh=True, actuator_state={'command_id': 'cmd-1', 'state': 'ACTIVE'}, actuator_state_fresh=True)
        self.assertEqual(result['status'], 'SUCCEEDED')

    def test_cancel_emits_failed_terminal(self):
        self.core.accept_request(self._request('hazard_avoid'), now_sec=10.0)
        payload = self.core.cancel(now_sec=10.1, reason='operator_stop')
        self.assertEqual(payload['status'], 'FAILED')
        self.assertEqual(payload['details']['reason'], 'operator_stop')


if __name__ == '__main__':
    unittest.main()


class BehaviorRuntimeFeedbackSemanticsTest(unittest.TestCase):
    def test_platform_feedback_requires_fresh_contract(self):
        core = BehaviorRuntimeCore(action_specs=build_behavior_runtime_specs({
            'action_specs': {
                'hazard_avoid': {
                    'success_policy': 'platform_clear',
                    'require_platform_feedback': True,
                    'require_ultrasonic_clear': True,
                }
            }
        }))
        core.accept_request({'command': {'command_id': 'cmd-1', 'action_type': 'hazard_avoid', 'task_type': 'hazard_avoid', 'timeout_sec': 2.0, 'metadata': {}}}, now_sec=10.0)
        self.assertIsNone(core.evaluate(now_sec=10.6, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': False, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True, 'ultrasonic_hazard_active': False}, detection_counts={}, detection_fresh=False))

    def test_facility_action_requires_actuator_state(self):
        core = BehaviorRuntimeCore(action_specs=build_behavior_runtime_specs({
            'action_specs': {
                'facility_attack': {
                    'success_policy': 'detection_confirmed',
                    'require_platform_feedback': True,
                    'require_actuator_state': True,
                    'confirmation_classes': ['enemy'],
                    'required_detection_count': 1,
                }
            }
        }))
        core.accept_request({'command': {'command_id': 'cmd-1', 'action_type': 'facility_attack', 'task_type': 'facility_attack', 'timeout_sec': 2.0, 'metadata': {'confirmation_classes': ['enemy'], 'required_detection_count': 1}}}, now_sec=10.0)
        self.assertIsNone(core.evaluate(now_sec=10.6, platform_summary={'feedback_contract_satisfied': True, 'execution_feedback_fresh': True, 'vendor_runtime_contract_satisfied': True, 'vendor_bundle_preflight_satisfied': True}, detection_counts={'enemy': 1}, detection_fresh=True, actuator_state={'command_id': 'cmd-1', 'state': 'FAILED'}, actuator_state_fresh=True))

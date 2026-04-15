import unittest

from ruikang_recon_baseline.behavior_actions import BehaviorActionCommand
from ruikang_recon_baseline.behavior_executor_core import (
    BehaviorExecutorCore,
    build_behavior_executor_specs,
    extract_command_id_from_payload_text,
)
from ruikang_recon_baseline.system_manager_core import resolve_required_nodes


class BehaviorExecutorCoreTest(unittest.TestCase):
    def setUp(self):
        self.specs = build_behavior_executor_specs({
            'default_pending_to_active_sec': 0.1,
            'default_dispatch_topic': 'recon/platform/behavior_execution/request',
            'default_result_topic': 'recon/platform/behavior_execution/result',
            'default_cancel_topic': 'recon/platform/behavior_execution/cancel',
            'action_specs': {
                'hazard_avoid': {
                    'supported_task_types': ['hazard_avoid'],
                    'dispatch_details': {'profile': 'hazard_detour'},
                    'success_details': {'profile': 'hazard_detour'},
                },
                'facility_attack': {
                    'supported_task_types': ['facility_attack'],
                    'dispatch_details': {'profile': 'facility_action'},
                    'success_details': {'profile': 'facility_action'},
                },
            },
        })
        self.core = BehaviorExecutorCore(action_specs=self.specs)

    def _command(self, *, command_id='cmd-1', action_type='hazard_avoid', task_type='hazard_avoid', issued_at=10.0, timeout_sec=2.0):
        return BehaviorActionCommand(
            command_id=command_id,
            action_type=action_type,
            route_id='route-a',
            zone_name='zone_a',
            step_id='step_a',
            task_type=task_type,
            objective_type=task_type,
            issued_at=issued_at,
            timeout_sec=timeout_sec,
        )

    def test_accept_builds_dispatch_and_requires_explicit_result_for_success(self):
        dispatch = self.core.accept(self._command(), now_sec=10.0)
        self.assertEqual(dispatch.dispatch_topic, 'recon/platform/behavior_execution/request')
        self.assertEqual(dispatch.result_topic, 'recon/platform/behavior_execution/result')
        self.assertEqual(self.core.poll(10.0).status, 'PENDING')
        self.assertEqual(self.core.poll(10.2).status, 'ACTIVE')
        # No downstream result means no synthetic success.
        self.assertEqual(self.core.poll(11.0).status, 'ACTIVE')

    def test_downstream_success_is_terminal(self):
        self.core.accept(self._command(), now_sec=10.0)
        feedback = self.core.observe_result(
            command_id='cmd-1',
            status='SUCCEEDED',
            now_sec=10.4,
            details={'confirmed_by': 'external_executor'},
            source='recon/platform/behavior_execution/result',
        )
        self.assertIsNotNone(feedback)
        self.assertEqual(feedback.status, 'SUCCEEDED')
        self.assertEqual(feedback.details['profile'], 'hazard_detour')
        self.assertEqual(feedback.details['confirmed_by'], 'external_executor')
        self.core.clear_terminal()
        self.assertIsNone(self.core.active)

    def test_timeout_requires_no_synthetic_success(self):
        self.core.accept(self._command(timeout_sec=0.2), now_sec=10.0)
        feedback = self.core.poll(10.3)
        self.assertEqual(feedback.status, 'TIMEOUT')
        self.assertEqual(feedback.details['reason'], 'downstream_result_timeout')

    def test_busy_executor_rejects_overlapping_command(self):
        self.core.accept(self._command(command_id='cmd-1'), now_sec=10.0)
        with self.assertRaises(RuntimeError):
            self.core.accept(self._command(command_id='cmd-2'), now_sec=10.1)

    def test_task_type_mismatch_is_rejected(self):
        with self.assertRaises(Exception):
            self.core.accept(self._command(task_type='facility_attack'), now_sec=10.0)

    def test_cancel_returns_failed_feedback_and_cancel_wire(self):
        self.core.accept(self._command(), now_sec=10.0)
        cancel_wire = self.core.build_cancel_envelope(now_sec=10.1, reason='lifecycle_pause')
        self.assertIsNotNone(cancel_wire)
        self.assertEqual(cancel_wire.cancel_topic, 'recon/platform/behavior_execution/cancel')
        feedback = self.core.cancel(now_sec=10.2, reason='lifecycle_pause')
        self.assertIsNotNone(feedback)
        self.assertEqual(feedback.status, 'FAILED')
        self.assertEqual(feedback.details['reason'], 'lifecycle_pause')

    def test_extract_command_id_from_wire_is_best_effort(self):
        self.assertEqual(extract_command_id_from_payload_text('{"command_id": "abc"}'), 'abc')
        self.assertEqual(extract_command_id_from_payload_text('not-json'), '')

    def test_system_manager_role_filter_supports_behavior_executor(self):
        resolved = resolve_required_nodes(
            ['vision_counter_node', 'behavior_executor_node', 'mission_manager_node'],
            {'vision': True, 'mission': True, 'recorder': False, 'safety': False, 'bridge': False, 'behavior': True},
        )
        self.assertEqual(resolved, ('vision_counter_node', 'behavior_executor_node', 'mission_manager_node'))
        resolved = resolve_required_nodes(
            ['vision_counter_node', 'behavior_executor_node', 'mission_manager_node'],
            {'vision': True, 'mission': True, 'recorder': False, 'safety': False, 'bridge': False, 'behavior': False},
        )
        self.assertEqual(resolved, ('vision_counter_node', 'mission_manager_node'))



    def test_terminal_feedback_must_be_drained_before_accepting_new_command(self):
        self.core.accept(self._command(), now_sec=10.0)
        self.core.observe_result(command_id='cmd-1', status='SUCCEEDED', now_sec=10.2, details={'source': 'runtime'}, source='runtime')
        with self.assertRaises(RuntimeError):
            self.core.accept(self._command(command_id='cmd-2'), now_sec=10.3)
        self.core.clear_terminal()
        dispatch = self.core.accept(self._command(command_id='cmd-2'), now_sec=10.4)
        self.assertEqual(dispatch.command_id, 'cmd-2')

if __name__ == '__main__':
    unittest.main()

import unittest

from ruikang_recon_baseline.common import ConfigurationError
from ruikang_recon_baseline.navigation_contracts import (
    apply_navigation_contract_defaults,
    validate_navigation_contract_bindings,
    validate_navigation_runtime_strategy,
)


class NavigationContractTest(unittest.TestCase):
    def test_move_base_contract_accepts_actionlib_amcl_stack(self):
        config = {
            'navigation_adapter_type': 'move_base_action',
            'move_base_action_name': 'move_base',
            'simple_goal_topic': 'move_base_simple/goal',
            'navigation_status_topic': 'recon/navigation_status',
            'navigation_cancel_topic': '',
            'navigation_status_timeout_sec': 3.0,
            'wait_for_action_server_sec': 3.0,
            'simulate_arrival_without_pose': False,
            'synthetic_arrival_delay_sec': 1.0,
            'pose_source_type': 'amcl_pose',
            'amcl_pose_topic': 'amcl_pose',
            'odom_topic': 'odom',
        }
        apply_navigation_contract_defaults(config)
        runtime = validate_navigation_runtime_strategy(config, owner='unit.nav')
        bindings = validate_navigation_contract_bindings(config, owner='unit.nav')
        self.assertEqual(runtime['planner_backend'], 'move_base_global_planner')
        self.assertEqual(runtime['localization_backend'], 'amcl_pose_topic')
        self.assertIn('move_base', bindings['bound_actions'])

    def test_simple_topic_contract_rejects_move_base_backends(self):
        config = {
            'navigation_adapter_type': 'simple_topic',
            'simple_goal_topic': 'move_base_simple/goal',
            'move_base_action_name': '',
            'navigation_status_topic': 'recon/navigation_status',
            'navigation_cancel_topic': '',
            'navigation_status_timeout_sec': 3.0,
            'wait_for_action_server_sec': 3.0,
            'simulate_arrival_without_pose': True,
            'synthetic_arrival_delay_sec': 0.5,
            'pose_source_type': 'odometry',
            'amcl_pose_topic': 'amcl_pose',
            'odom_topic': 'odom',
            'navigation_planner_backend': 'move_base_global_planner',
            'navigation_controller_backend': 'move_base_local_controller',
            'navigation_recovery_backend': 'executor_policy',
            'navigation_localization_backend': 'odometry_topic',
            'navigation_goal_transport': 'topic',
            'navigation_status_transport': 'internal_pose_evaluator',
            'navigation_cancel_transport': 'none',
        }
        with self.assertRaises(ConfigurationError):
            validate_navigation_runtime_strategy(config, owner='unit.simple')

    def test_goal_topic_status_requires_status_topic(self):
        config = {
            'navigation_adapter_type': 'goal_topic_status',
            'simple_goal_topic': 'nav_goal',
            'move_base_action_name': '',
            'navigation_status_topic': '',
            'navigation_cancel_topic': 'nav_cancel',
            'navigation_status_timeout_sec': 3.0,
            'wait_for_action_server_sec': 3.0,
            'simulate_arrival_without_pose': False,
            'synthetic_arrival_delay_sec': 1.0,
            'pose_source_type': 'tf_lookup',
            'amcl_pose_topic': '',
            'odom_topic': '',
            'navigation_planner_backend': 'external_goal_dispatch',
            'navigation_controller_backend': 'external_controller_stack',
            'navigation_recovery_backend': 'external_status_plus_executor_policy',
            'navigation_localization_backend': 'tf_lookup_pose',
            'navigation_goal_transport': 'topic',
            'navigation_status_transport': 'topic',
            'navigation_cancel_transport': 'topic',
        }
        with self.assertRaises(ConfigurationError):
            validate_navigation_contract_bindings(config, owner='unit.status')


if __name__ == '__main__':
    unittest.main()

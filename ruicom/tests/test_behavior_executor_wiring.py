import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


class BehaviorExecutorWiringTest(unittest.TestCase):
    def test_common_behavior_executor_config_declares_external_execution_contract(self):
        payload = yaml.safe_load((ROOT / 'config/common/behavior_executor.yaml').read_text())
        self.assertEqual(payload['behavior_command_topic'], 'recon/behavior_command')
        self.assertEqual(payload['behavior_feedback_topic'], 'recon/behavior_feedback')
        self.assertEqual(payload['default_dispatch_topic'], 'recon/platform/behavior_execution/request')
        self.assertEqual(payload['default_result_topic'], 'recon/platform/behavior_execution/result')
        self.assertEqual(payload['default_cancel_topic'], 'recon/platform/behavior_execution/cancel')
        self.assertIn('hazard_avoid', payload['action_specs'])
        self.assertIn('facility_attack', payload['action_specs'])

    def test_deploy_core_launch_wires_behavior_executor_node(self):
        text = (ROOT / 'launch/core.launch').read_text()
        self.assertIn('type="behavior_executor_node.py"', text)
        self.assertIn('common_behavior_executor_config', text)
        self.assertIn('enable_behavior_executor_component', text)

    def test_reference_and_field_deploy_enable_behavior_executor(self):
        for rel_path in ['launch/reference_field_deploy.launch', 'launch/real_field_deploy.launch', 'launch/field_deploy.launch']:
            with self.subTest(path=rel_path):
                text = (ROOT / rel_path).read_text()
                self.assertIn('enable_behavior_executor', text)
                self.assertIn('profile_behavior_executor_config', text)

    def test_system_manager_orders_executor_before_mission_and_uses_binding_readiness(self):
        payload = yaml.safe_load((ROOT / 'config/common/system_manager.yaml').read_text())
        self.assertIn('vendor_actuator_bridge_node', payload['required_nodes'])
        self.assertIn('vendor_actuator_device_node', payload['required_nodes'])
        self.assertIn('vendor_actuator_feedback_node', payload['required_nodes'])
        self.assertIn('behavior_actuator_node', payload['required_nodes'])
        self.assertIn('behavior_runtime_node', payload['required_nodes'])
        self.assertIn('behavior_executor_node', payload['required_nodes'])
        self.assertLess(payload['required_nodes'].index('vendor_actuator_bridge_node'), payload['required_nodes'].index('vendor_actuator_device_node'))
        self.assertLess(payload['required_nodes'].index('vendor_actuator_device_node'), payload['required_nodes'].index('vendor_actuator_feedback_node'))
        self.assertLess(payload['required_nodes'].index('vendor_actuator_feedback_node'), payload['required_nodes'].index('behavior_actuator_node'))
        self.assertLess(payload['required_nodes'].index('behavior_actuator_node'), payload['required_nodes'].index('behavior_runtime_node'))
        self.assertLess(payload['required_nodes'].index('behavior_runtime_node'), payload['required_nodes'].index('behavior_executor_node'))
        self.assertLess(payload['required_nodes'].index('behavior_executor_node'), payload['required_nodes'].index('mission_manager_node'))
        self.assertEqual(
            payload['readiness_requirements']['behavior_executor_node'],
            [
                'command_topic_declared',
                'feedback_topic_declared',
                'action_catalog_ready',
                'downstream_binding_declared',
                'downstream_dispatch_ready',
                'downstream_result_ready',
            ],
        )

        self.assertEqual(
            payload['readiness_requirements']['vendor_actuator_bridge_node'],
            [
                'input_command_topic_declared',
                'raw_command_output_topic_declared',
                'raw_result_topic_declared',
                'command_input_bound',
                'raw_command_output_bound',
            ],
        )

        self.assertEqual(
            payload['readiness_requirements']['vendor_actuator_device_node'],
            [
                'raw_command_topic_declared',
                'raw_result_topic_declared',
                'external_ack_topic_declared',
                'command_input_bound',
                'raw_result_output_bound',
                'external_ack_bound',
            ],
        )

        self.assertEqual(
            payload['readiness_requirements']['behavior_actuator_node'],
            [
                'actuator_command_topic_declared',
                'actuator_state_topic_declared',
                'actuator_output_topic_declared',
                'vendor_feedback_topic_declared',
                'actuator_output_bound',
                'downstream_feedback_bound',
            ],
        )


if __name__ == '__main__':
    unittest.main()



class BehaviorRuntimeWiringTest(unittest.TestCase):
    def test_behavior_runtime_config_exposes_platform_execution_contract(self):
        payload = yaml.safe_load((ROOT / 'config/common/behavior_runtime.yaml').read_text(encoding='utf-8'))
        self.assertEqual(payload['request_topic'], 'recon/platform/behavior_execution/request')
        self.assertEqual(payload['result_topic'], 'recon/platform/behavior_execution/result')
        self.assertEqual(payload['cancel_topic'], 'recon/platform/behavior_execution/cancel')
        self.assertEqual(payload['command_output_topic'], 'cmd_vel_raw')
        self.assertIn('hazard_avoid', payload['action_specs'])
        self.assertIn('facility_attack', payload['action_specs'])

    def test_reference_and_field_deploy_enable_behavior_runtime(self):
        for rel_path in ['launch/reference_field_deploy.launch', 'launch/real_field_deploy.launch', 'launch/field_deploy.launch']:
            content = (ROOT / rel_path).read_text(encoding='utf-8')
            with self.subTest(path=rel_path):
                self.assertIn('enable_behavior_runtime', content)
                self.assertIn('profile_behavior_runtime_config', content)


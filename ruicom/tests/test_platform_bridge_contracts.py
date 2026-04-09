import unittest

from ruikang_recon_baseline.domain_models import ConfigurationError
from ruikang_recon_baseline.platform_bridge_core import PlatformBridgeSnapshot, evaluate_platform_bridge
from ruikang_recon_baseline.platform_contracts import validate_command_topic_flow_contract


class PlatformBridgeContractTest(unittest.TestCase):
    def test_command_bridge_fresh_tracks_upstream_velocity_activity(self):
        decision = evaluate_platform_bridge(
            PlatformBridgeSnapshot(
                now_sec=5.0,
                feedback_timeout_sec=1.0,
                explicit_feedback_enabled=False,
                explicit_feedback_value=False,
                explicit_feedback_stamp_sec=0.0,
                odom_feedback_enabled=True,
                odom_stamp_sec=4.8,
                allow_odom_as_feedback=True,
                command_bridge_enabled=True,
                upstream_command_stamp_sec=4.9,
            )
        )
        self.assertTrue(decision.command_bridge_fresh)
        self.assertTrue(decision.output_feedback)
        self.assertEqual(decision.source, 'odom_feedback_fallback')

    def test_command_bridge_stale_is_reported_independently(self):
        decision = evaluate_platform_bridge(
            PlatformBridgeSnapshot(
                now_sec=5.0,
                feedback_timeout_sec=1.0,
                explicit_feedback_enabled=True,
                explicit_feedback_value=True,
                explicit_feedback_stamp_sec=4.7,
                odom_feedback_enabled=False,
                odom_stamp_sec=0.0,
                allow_odom_as_feedback=False,
                command_bridge_enabled=True,
                upstream_command_stamp_sec=3.0,
            )
        )
        self.assertFalse(decision.command_bridge_fresh)
        self.assertTrue(decision.output_feedback)

    def test_odom_only_is_observability_when_fallback_disabled(self):
        decision = evaluate_platform_bridge(
            PlatformBridgeSnapshot(
                now_sec=5.0,
                feedback_timeout_sec=1.0,
                explicit_feedback_enabled=False,
                explicit_feedback_value=False,
                explicit_feedback_stamp_sec=0.0,
                odom_feedback_enabled=True,
                odom_stamp_sec=4.8,
                allow_odom_as_feedback=False,
                command_bridge_enabled=False,
                upstream_command_stamp_sec=0.0,
            )
        )
        self.assertTrue(decision.odom_feedback_fresh)
        self.assertFalse(decision.output_feedback)
        self.assertEqual(decision.source, 'odom_observability_only')




class CommandTopicFlowContractTest(unittest.TestCase):
    def test_rejects_upstream_topic_aliasing_safety_output_topic(self):
        with self.assertRaises(ConfigurationError):
            validate_command_topic_flow_contract(
                {
                    'upstream_command_topic': 'cmd_vel',
                    'command_input_topic': 'cmd_vel_raw',
                    'safety_output_topic': 'cmd_vel',
                },
                owner='platform_bridge_node',
            )

    def test_accepts_namespaced_vendor_command_topic(self):
        summary = validate_command_topic_flow_contract(
            {
                'upstream_command_topic': 'vendor/cmd_vel',
                'command_input_topic': 'cmd_vel_raw',
                'safety_output_topic': 'cmd_vel',
            },
            owner='platform_bridge_node',
        )
        self.assertEqual(summary['upstream_command_topic'], 'vendor/cmd_vel')
        self.assertEqual(summary['command_input_topic'], 'cmd_vel_raw')
        self.assertEqual(summary['safety_output_topic'], 'cmd_vel')

if __name__ == '__main__':
    unittest.main()

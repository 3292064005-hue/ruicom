import unittest

from ruikang_recon_baseline.system_manager_core import SystemManagerCore, resolve_required_nodes


class SystemManagerCoreTest(unittest.TestCase):
    def test_auto_activate_after_all_required_nodes_ready(self):
        core = SystemManagerCore(required_nodes=['vision_counter_node', 'mission_manager_node'], ready_timeout_sec=5.0, health_freshness_sec=2.0, auto_activate=True, readiness_requirements={'vision_counter_node': ['camera_frame_fresh'], 'mission_manager_node': ['route_bound']})
        core.bootstrap(1.0)
        core.observe_health(node='vision_counter_node', status='ok', message='ready', stamp=1.1, details={'camera_frame_fresh': True})
        core.observe_health(node='mission_manager_node', status='ok', message='ready', stamp=1.2, details={'route_bound': True})
        decision = core.tick(1.3)
        self.assertEqual(decision.state, 'ACTIVE')
        self.assertEqual(decision.command, 'activate')
        self.assertTrue(decision.details['readiness']['graph_ready'])
        self.assertTrue(decision.details['readiness']['semantic_ready'])
        self.assertTrue(decision.details['readiness']['graph_ready'])
        self.assertTrue(decision.details['readiness']['semantic_ready'])
        self.assertTrue(decision.details['readiness']['mission_ready'])


    def test_auto_activate_does_not_require_pre_activation_command_traffic(self):
        core = SystemManagerCore(
            required_nodes=['cmd_safety_mux_node', 'platform_bridge_node'],
            ready_timeout_sec=5.0,
            health_freshness_sec=2.0,
            auto_activate=True,
            readiness_requirements={
                'cmd_safety_mux_node': ['command_path_bound', 'output_feedback_required_satisfied'],
                'platform_bridge_node': ['command_path_bound', 'command_flow_contract_satisfied', 'feedback_contract_satisfied', 'platform_runtime_probe_satisfied', 'vendor_runtime_contract_satisfied'],
            },
        )
        core.bootstrap(1.0)
        core.observe_health(
            node='cmd_safety_mux_node',
            status='ok',
            message='ready',
            stamp=1.1,
            details={
                'command_path_bound': True,
                'output_feedback_required_satisfied': True,
                'command_traffic_seen': False,
            },
        )
        core.observe_health(
            node='platform_bridge_node',
            status='ok',
            message='ready',
            stamp=1.2,
            details={
                'command_path_bound': True,
                'command_flow_contract_satisfied': True,
                'feedback_contract_satisfied': True,
                'platform_runtime_probe_satisfied': True,
                'vendor_runtime_contract_satisfied': True,
                'command_traffic_seen': False,
            },
        )
        decision = core.tick(1.3)
        self.assertEqual(decision.state, 'ACTIVE')
        self.assertEqual(decision.command, 'activate')

    def test_warning_node_causes_degraded(self):
        core = SystemManagerCore(required_nodes=['vision_counter_node'], ready_timeout_sec=5.0, health_freshness_sec=2.0, auto_activate=False)
        core.bootstrap(1.0)
        core.observe_health(node='vision_counter_node', status='warn', message='camera_low_fps', stamp=1.1)
        decision = core.tick(1.2)
        self.assertEqual(decision.state, 'DEGRADED')
        self.assertEqual(decision.reason, 'node_warning')



    def test_bootstrap_accepts_zero_sim_time(self):
        core = SystemManagerCore(required_nodes=['vision_counter_node'], ready_timeout_sec=5.0, health_freshness_sec=2.0, auto_activate=False)
        core.bootstrap(0.0)
        decision = core.tick(0.0)
        self.assertEqual(decision.state, 'BOOTSTRAP')
        self.assertEqual(decision.reason, 'waiting_for_ready')

    def test_required_nodes_filter_respects_disabled_components(self):
        resolved = resolve_required_nodes(
            ['vision_counter_node', 'mission_manager_node', 'mission_recorder_node', 'cmd_safety_mux_node'],
            {'vision': True, 'mission': True, 'recorder': True, 'safety': False},
        )
        self.assertEqual(
            resolved,
            ('vision_counter_node', 'mission_manager_node', 'mission_recorder_node'),
        )

    def test_required_nodes_filter_rejects_empty_result(self):
        with self.assertRaises(ValueError):
            resolve_required_nodes(
                ['cmd_safety_mux_node'],
                {'vision': False, 'mission': False, 'recorder': False, 'safety': False},
            )

    def test_startup_timeout_faults_when_required_node_missing(self):
        core = SystemManagerCore(required_nodes=['vision_counter_node'], ready_timeout_sec=1.0, health_freshness_sec=2.0, auto_activate=False)
        core.bootstrap(1.0)
        decision = core.tick(2.5)
        self.assertEqual(decision.state, 'FAULT')
        self.assertEqual(decision.reason, 'startup_timeout')


    def test_resource_not_ready_blocks_bootstrap_until_detail_turns_true(self):
        core = SystemManagerCore(required_nodes=['vision_counter_node'], ready_timeout_sec=5.0, health_freshness_sec=2.0, auto_activate=False, readiness_requirements={'vision_counter_node': ['camera_frame_fresh']})
        core.bootstrap(1.0)
        core.observe_health(node='vision_counter_node', status='ok', message='ready', stamp=1.1, details={'camera_frame_fresh': False})
        decision = core.tick(1.2)
        self.assertEqual(decision.state, 'BOOTSTRAP')
        self.assertEqual(decision.reason, 'waiting_for_ready')
        core.observe_health(node='vision_counter_node', status='ok', message='ready', stamp=1.3, details={'camera_frame_fresh': True})
        decision = core.tick(1.4)
        self.assertEqual(decision.state, 'READY')

if __name__ == '__main__':
    unittest.main()

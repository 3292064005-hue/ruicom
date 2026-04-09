import unittest

from ruikang_recon_baseline.common import Detection, Waypoint
from ruikang_recon_baseline.lifecycle_protocol import decode_lifecycle_control, encode_lifecycle_control
from ruikang_recon_baseline.lifecycle_runtime import ManagedRuntimeState
from ruikang_recon_baseline.mission_plan import MissionPlan, MissionStep
from ruikang_recon_baseline.system_manager_core import SystemManagerCore
from ruikang_recon_baseline.vision_core import FrameRegionAdapter


class LifecycleProtocolExtensionTest(unittest.TestCase):
    def test_encode_decode_targeted_envelope_roundtrip(self):
        wire = encode_lifecycle_control('configure', target='vision_counter_node', issued_by='system_manager_node', metadata={'reason': 'configure_node'})
        envelope = decode_lifecycle_control(wire)
        self.assertEqual(envelope.command, 'configure')
        self.assertEqual(envelope.target, 'vision_counter_node')
        self.assertTrue(envelope.matches('vision_counter_node'))
        self.assertFalse(envelope.matches('mission_manager_node'))

    def test_managed_runtime_supports_configure_ready_phase(self):
        runtime = ManagedRuntimeState(lifecycle_managed=True)
        self.assertEqual(runtime.state, 'IDLE')
        result = runtime.apply('configure')
        self.assertTrue(result.accepted)
        self.assertEqual(runtime.state, 'READY')
        self.assertTrue(runtime.configured)
        self.assertFalse(runtime.processing_allowed)
        result = runtime.apply('activate')
        self.assertEqual(runtime.state, 'ACTIVE')
        self.assertTrue(runtime.processing_allowed)


class SystemManagerStageOrderingTest(unittest.TestCase):
    def test_core_configures_then_activates_managed_nodes_in_order(self):
        core = SystemManagerCore(
            required_nodes=['vision_counter_node', 'mission_manager_node'],
            ready_timeout_sec=5.0,
            health_freshness_sec=2.0,
            auto_activate=True,
            readiness_requirements={
                'vision_counter_node': ['camera_frame_fresh'],
                'mission_manager_node': ['route_bound'],
            },
        )
        core.bootstrap(1.0)
        core.observe_health(node='vision_counter_node', status='ok', message='ready', stamp=1.1, details={'camera_frame_fresh': True, 'lifecycle_managed': True, 'runtime_state': 'IDLE'})
        core.observe_health(node='mission_manager_node', status='ok', message='ready', stamp=1.2, details={'route_bound': True, 'lifecycle_managed': True, 'runtime_state': 'IDLE'})
        decision = core.tick(1.3)
        self.assertEqual(decision.command, 'configure')
        self.assertEqual(decision.details['target_node'], 'vision_counter_node')

        core.observe_health(node='vision_counter_node', status='ok', message='configured', stamp=1.4, details={'camera_frame_fresh': True, 'lifecycle_managed': True, 'runtime_state': 'READY'})
        decision = core.tick(1.5)
        self.assertEqual(decision.command, 'configure')
        self.assertEqual(decision.details['target_node'], 'mission_manager_node')

        core.observe_health(node='mission_manager_node', status='ok', message='configured', stamp=1.6, details={'route_bound': True, 'lifecycle_managed': True, 'runtime_state': 'READY'})
        decision = core.tick(1.7)
        self.assertEqual(decision.command, 'activate')
        self.assertEqual(decision.details['target_node'], 'vision_counter_node')




class MissionRuntimeStateCompatibilityTest(unittest.TestCase):
    def test_managed_mission_runtime_reaches_ready_before_activation(self):
        runtime = ManagedRuntimeState(lifecycle_managed=True)
        self.assertEqual(runtime.state, 'IDLE')
        configured = runtime.apply('configure')
        self.assertTrue(configured.accepted)
        self.assertEqual(runtime.state, 'READY')
        activated = runtime.apply('activate')
        self.assertTrue(activated.accepted)
        self.assertEqual(runtime.state, 'ACTIVE')

class MissionGraphOutcomeEdgeTest(unittest.TestCase):
    def test_outcome_edges_override_failure_fallback(self):
        steps = [
            MissionStep(
                waypoint=Waypoint(name='zone_a', x=0.0, y=0.0, route_id='zone_a'),
                index=0,
                total=2,
                step_id='step_a',
                next_step_id='step_b',
                outcome_edges={'navigation_timeout': 'step_b'},
            ),
            MissionStep(
                waypoint=Waypoint(name='zone_b', x=1.0, y=0.0, route_id='zone_b'),
                index=1,
                total=2,
                step_id='step_b',
            ),
        ]
        plan = MissionPlan.from_steps(steps)
        self.assertEqual(plan.next_index_for_outcome(0, 'navigation_timeout'), 1)
        self.assertEqual(plan.next_index_for_outcome(0, 'success'), 1)


class CalibratedRegionAdapterTest(unittest.TestCase):
    def test_adapter_projects_reference_regions_to_runtime_frame(self):
        adapter = FrameRegionAdapter(
            'calibrated_named_regions',
            [{'name': 'zone_a', 'x0': 100, 'y0': 50, 'x1': 200, 'y1': 150}],
            region_calibration={
                'reference_image_width': 400,
                'reference_image_height': 200,
                'x_scale': 1.0,
                'y_scale': 1.0,
                'x_offset_px': 0.0,
                'y_offset_px': 0.0,
            },
        )
        regions = adapter.region_payloads(800, 400)
        self.assertEqual(regions[0]['x0'], 200)
        self.assertEqual(regions[0]['y0'], 100)
        detections = adapter.assign([Detection(class_name='friendly', score=0.9, x1=220, y1=110, x2=260, y2=160)], frame_width=800, frame_height=400)
        self.assertEqual(detections[0].frame_region, 'zone_a')


if __name__ == '__main__':
    unittest.main()

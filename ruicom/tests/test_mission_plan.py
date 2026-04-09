import unittest

from ruikang_recon_baseline.common import Waypoint
from ruikang_recon_baseline.mission_plan import MissionPlan, MissionStep


class MissionPlanExtensibilityTest(unittest.TestCase):
    def setUp(self):
        self.waypoints = [
            Waypoint(name='zone_a', x=1.0, y=0.0, yaw_deg=0.0, dwell_sec=4.0, timeout_sec=30.0, route_id='zone_a__0'),
            Waypoint(name='zone_b', x=2.0, y=0.0, yaw_deg=0.0, dwell_sec=4.0, timeout_sec=30.0, route_id='zone_b__1'),
        ]

    def test_from_waypoints_builds_default_waypoint_capture_steps(self):
        plan = MissionPlan.from_waypoints(self.waypoints)
        self.assertEqual(plan.task_types(), ('waypoint_capture', 'waypoint_capture'))
        self.assertEqual(plan.step_at(0).route_id, 'zone_a__0')
        self.assertEqual(plan.as_sequence()[1].name, 'zone_b')
        self.assertEqual(plan.next_index_for_outcome(0, 'success'), 1)

    def test_from_steps_reindexes_custom_step_metadata(self):
        plan = MissionPlan.from_steps([
            MissionStep(waypoint=self.waypoints[0], index=99, total=99, task_type='waypoint_capture', metadata={'phase': 'scan'}, step_id='scan_a'),
            MissionStep(waypoint=self.waypoints[1], index=98, total=99, task_type='waypoint_capture', metadata={'phase': 'egress'}, step_id='scan_b'),
        ])
        self.assertEqual(plan.step_at(0).index, 0)
        self.assertEqual(plan.step_at(1).total, 2)
        self.assertEqual(plan.step_at(1).metadata['phase'], 'egress')
        self.assertEqual(plan.step_by_id('scan_a').route_id, 'zone_a__0')


    def test_from_task_specs_supports_competition_task_types_and_objectives(self):
        plan = MissionPlan.from_task_specs([
            {
                'step_id': 'recon_a',
                'task_type': 'recon_zone',
                'waypoint': {'name': 'zone_a', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 20.0, 'route_id': 'zone_a__0'},
            },
            {
                'step_id': 'hazard_b',
                'task_type': 'hazard_avoid',
                'metadata': {'hazard_id': 'cone_b'},
                'waypoint': {'name': 'zone_b', 'x': 1.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 20.0, 'route_id': 'zone_b__1'},
            },
            {
                'step_id': 'facility_c',
                'task_type': 'facility_attack',
                'metadata': {'confirmation_classes': ['enemy'], 'required_detection_count': 2},
                'waypoint': {'name': 'zone_c', 'x': 2.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 20.0, 'route_id': 'zone_c__2'},
            },
        ], dwell_default_sec=4.0)
        self.assertEqual(plan.execution_types(), ('recon_zone', 'hazard_avoid', 'facility_attack'))
        self.assertEqual(plan.step_at(1).metadata['hazard_type'], 'anti_tank_cone')
        self.assertEqual(plan.step_at(2).normalized_objective_type, 'facility_attack')

    def test_from_task_specs_supports_graph_edges_and_waypoint_only(self):
        plan = MissionPlan.from_task_specs([
            {
                'step_id': 'scan_a',
                'task_type': 'waypoint_capture',
                'retry_limit': 3,
                'next_step_id': 'transit_b',
                'waypoint': {'name': 'zone_a', 'x': 1.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 20.0, 'route_id': 'zone_a__0'},
            },
            {
                'step_id': 'transit_b',
                'task_type': 'waypoint_only',
                'failure_step_id': 'scan_a',
                'quiesce_sec': 1.5,
                'waypoint': {'name': 'zone_b', 'x': 2.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 20.0, 'route_id': 'zone_b__1'},
            },
        ], dwell_default_sec=4.0)
        self.assertEqual(plan.task_types(), ('waypoint_capture', 'waypoint_only'))
        self.assertEqual(plan.step_at(0).retry_limit, 3)
        self.assertAlmostEqual(plan.step_at(1).quiesce_sec, 1.5)
        self.assertEqual(plan.next_index_for_outcome(0, 'success'), 1)
        self.assertEqual(plan.next_index_for_outcome(1, 'failure'), 0)


if __name__ == '__main__':
    unittest.main()

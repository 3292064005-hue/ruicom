import unittest

from ruikang_recon_baseline.common import Waypoint, ZoneCaptureResult
from ruikang_recon_baseline.mission_context import MissionContext
from ruikang_recon_baseline.mission_core import AggregationPolicy, CaptureWindowAggregator
from ruikang_recon_baseline.mission_executor import MissionExecutor, MissionRuntimeHooks
from ruikang_recon_baseline.mission_plan import MissionPlan, MissionStep
from ruikang_recon_baseline.navigation_adapters.base import NavigationAdapterBase
from ruikang_recon_baseline.recovery_policies import RetryThenFailPolicy


class _DummyClock:
    def __init__(self, now_sec: float = 10.0):
        self._now_sec = float(now_sec)

    def now_business_sec(self) -> float:
        return self._now_sec


class _DummyNavAdapter(NavigationAdapterBase):
    def dispatch(self, waypoint):
        self.last_waypoint = waypoint

    def poll(self, now_sec: float) -> str:
        _ = now_sec
        return 'SUCCEEDED'

    def cancel(self, now_sec=None) -> bool:
        _ = now_sec
        return True


class MissionBehaviorTest(unittest.TestCase):
    def setUp(self):
        waypoint = Waypoint(name='zone_a', x=1.0, y=0.0, yaw_deg=0.0, dwell_sec=3.0, timeout_sec=20.0, frame_region='zone_a', route_id='zone_a__0')
        step = MissionStep(waypoint=waypoint, index=0, total=1, task_type='hazard_avoid', metadata={'confirmation_classes': ['enemy'], 'allowed_detection_count': 0, 'required_observation_frames': 2})
        self.plan = MissionPlan.from_steps([step])
        self.context = MissionContext(dispatch_started_at=5.0)
        self.executor = MissionExecutor(
            plan=self.plan,
            context=self.context,
            clock=_DummyClock(),
            nav_adapter=_DummyNavAdapter(),
            capture_aggregator=CaptureWindowAggregator(AggregationPolicy(class_names=('friendly', 'enemy', 'hostage'))),
            recovery_policy=RetryThenFailPolicy(retry_limit=1, navigation_failure_quiesce_sec=0.5),
            hooks=MissionRuntimeHooks(
                emit_state=lambda *args, **kwargs: None,
                publish_zone_capture=lambda *args, **kwargs: None,
                publish_current_zone=lambda *args, **kwargs: None,
                publish_health=lambda *args, **kwargs: None,
                refresh_pose_source=lambda *args, **kwargs: None,
                preflight_ready=lambda *args, **kwargs: True,
            ),
            mission_timeout_sec=30.0,
            preflight_timeout_sec=2.0,
            class_names=('friendly', 'enemy', 'hostage'),
            class_schema_hash='schema',
        )

    def test_transit_navigation_success_returns_immediate_terminal_result(self):
        waypoint = Waypoint(name='zone_b', x=2.0, y=0.0, yaw_deg=0.0, dwell_sec=0.0, timeout_sec=20.0, frame_region='zone_b', route_id='zone_b__1')
        step = MissionPlan.from_steps([
            MissionStep(waypoint=waypoint, index=0, total=1, task_type='transit', step_id='transit_b')
        ]).step_at(0)
        resolution = self.executor.behaviors.resolve('transit').on_navigation_succeeded(self.executor, step, 12.0)
        self.assertFalse(resolution.begin_dwell)
        self.assertIsNotNone(resolution.result)
        self.assertEqual(resolution.outcome, 'success')
        self.assertEqual(resolution.result.mission_outcome, 'transit_reached')

    def test_hazard_avoid_requires_clear_confirmation(self):
        step = self.plan.step_at(0)
        base_result = ZoneCaptureResult(
            zone_name='zone_a',
            status='ok',
            class_names=['friendly', 'enemy', 'hostage'],
            class_counts=[0, 0, 0],
            capture_started_at=5.0,
            capture_finished_at=8.0,
            frame_count=2,
            frame_region='zone_a',
            route_id='zone_a__0',
            evidence_summary={'accepted_frame_count': 2},
        )
        resolution = self.executor.behaviors.resolve('hazard_avoid').finalize_capture(self.executor, step, base_result)
        self.assertEqual(resolution.outcome, 'hazard_cleared')
        self.assertEqual(resolution.result.status, 'ok')
        self.assertEqual(resolution.result.mission_outcome, 'hazard_avoided')
        self.assertTrue(resolution.result.hazard_summary['confirmed'])

    def test_hazard_avoid_fails_when_hazard_still_detected(self):
        step = self.plan.step_at(0)
        base_result = ZoneCaptureResult(
            zone_name='zone_a',
            status='ok',
            class_names=['friendly', 'enemy', 'hostage'],
            class_counts=[0, 1, 0],
            capture_started_at=5.0,
            capture_finished_at=8.0,
            frame_count=2,
            frame_region='zone_a',
            route_id='zone_a__0',
            evidence_summary={'accepted_frame_count': 2},
        )
        resolution = self.executor.behaviors.resolve('hazard_avoid').finalize_capture(self.executor, step, base_result)
        self.assertEqual(resolution.outcome, 'failure')
        self.assertEqual(resolution.result.status, 'hazard_unconfirmed')
        self.assertEqual(resolution.result.mission_outcome, 'hazard_avoidance_failed')
        self.assertFalse(resolution.result.hazard_summary['confirmed'])

    def test_facility_attack_report_only_succeeds_without_detection_confirmation(self):
        waypoint = Waypoint(name='zone_c', x=3.0, y=0.0, yaw_deg=0.0, dwell_sec=3.0, timeout_sec=20.0, frame_region='zone_c', route_id='zone_c__2')
        step = MissionPlan.from_steps([
            MissionStep(
                waypoint=waypoint,
                index=0,
                total=1,
                task_type='facility_attack',
                metadata={'attack_mode': 'report_only', 'confirmation_classes': ['enemy'], 'required_detection_count': 2},
                step_id='facility_c_report',
            )
        ]).step_at(0)
        base_result = ZoneCaptureResult(
            zone_name='zone_c',
            status='ok',
            class_names=['friendly', 'enemy', 'hostage'],
            class_counts=[0, 0, 0],
            capture_started_at=5.0,
            capture_finished_at=8.0,
            frame_count=3,
            frame_region='zone_c',
            route_id='zone_c__2',
            evidence_summary={'accepted_frame_count': 3},
        )
        resolution = self.executor.behaviors.resolve('facility_attack').finalize_capture(self.executor, step, base_result)
        self.assertEqual(resolution.outcome, 'action_reported')
        self.assertEqual(resolution.result.status, 'ok')
        self.assertEqual(resolution.result.mission_outcome, 'facility_action_reported')
        self.assertFalse(resolution.result.action_summary['confirmed'])

    def test_facility_attack_confirms_detection_threshold(self):
        waypoint = Waypoint(name='zone_c', x=3.0, y=0.0, yaw_deg=0.0, dwell_sec=3.0, timeout_sec=20.0, frame_region='zone_c', route_id='zone_c__2')
        step = MissionPlan.from_steps([
            MissionStep(
                waypoint=waypoint,
                index=0,
                total=1,
                task_type='facility_attack',
                metadata={'attack_mode': 'detection_confirmed', 'confirmation_classes': ['enemy'], 'required_detection_count': 2},
                step_id='facility_c',
            )
        ]).step_at(0)
        base_result = ZoneCaptureResult(
            zone_name='zone_c',
            status='ok',
            class_names=['friendly', 'enemy', 'hostage'],
            class_counts=[0, 2, 0],
            capture_started_at=5.0,
            capture_finished_at=8.0,
            frame_count=3,
            frame_region='zone_c',
            route_id='zone_c__2',
            evidence_summary={'accepted_frame_count': 3},
        )
        resolution = self.executor.behaviors.resolve('facility_attack').finalize_capture(self.executor, step, base_result)
        self.assertEqual(resolution.outcome, 'action_confirmed')
        self.assertEqual(resolution.result.mission_outcome, 'facility_action_confirmed')
        self.assertTrue(resolution.result.action_summary['confirmed'])
    def test_facility_attack_detection_confirmed_fails_without_required_detection(self):
        waypoint = Waypoint(name='zone_c', x=3.0, y=0.0, yaw_deg=0.0, dwell_sec=3.0, timeout_sec=20.0, frame_region='zone_c', route_id='zone_c__2')
        step = MissionPlan.from_steps([
            MissionStep(
                waypoint=waypoint,
                index=0,
                total=1,
                task_type='facility_attack',
                metadata={'attack_mode': 'detection_confirmed', 'confirmation_classes': ['enemy'], 'required_detection_count': 2},
                step_id='facility_c_confirm',
            )
        ]).step_at(0)
        base_result = ZoneCaptureResult(
            zone_name='zone_c',
            status='ok',
            class_names=['friendly', 'enemy', 'hostage'],
            class_counts=[0, 1, 0],
            capture_started_at=5.0,
            capture_finished_at=8.0,
            frame_count=3,
            frame_region='zone_c',
            route_id='zone_c__2',
            evidence_summary={'accepted_frame_count': 3},
        )
        resolution = self.executor.behaviors.resolve('facility_attack').finalize_capture(self.executor, step, base_result)
        self.assertEqual(resolution.outcome, 'failure')
        self.assertEqual(resolution.result.status, 'action_unconfirmed')
        self.assertEqual(resolution.result.mission_outcome, 'facility_action_failed')


if __name__ == '__main__':
    unittest.main()

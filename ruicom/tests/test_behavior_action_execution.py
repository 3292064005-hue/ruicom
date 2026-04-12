import unittest

from ruikang_recon_baseline.behavior_actions import (
    BehaviorActionCommand,
    DisabledBehaviorActionBackend,
    MemoryBehaviorActionBackend,
)
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


class BehaviorActionBackendTest(unittest.TestCase):
    def test_memory_backend_transitions_to_success(self):
        backend = MemoryBehaviorActionBackend(outcome_status='SUCCEEDED', completion_delay_sec=1.0)
        command = BehaviorActionCommand(
            command_id='cmd-1',
            action_type='hazard_avoid',
            route_id='zone_a__0',
            zone_name='zone_a',
            step_id='step_a',
            task_type='hazard_avoid',
            objective_type='hazard_avoid',
            issued_at=10.0,
            timeout_sec=2.0,
        )
        backend.dispatch(command)
        self.assertEqual(backend.poll(10.0).status, 'PENDING')
        self.assertEqual(backend.poll(10.5).status, 'ACTIVE')
        self.assertEqual(backend.poll(11.0).status, 'SUCCEEDED')

    def test_disabled_backend_rejects_dispatch(self):
        backend = DisabledBehaviorActionBackend()
        command = BehaviorActionCommand(
            command_id='cmd-1',
            action_type='facility_attack',
            route_id='zone_c__2',
            zone_name='zone_c',
            step_id='step_c',
            task_type='facility_attack',
            objective_type='facility_attack',
            issued_at=10.0,
            timeout_sec=2.0,
        )
        with self.assertRaises(RuntimeError):
            backend.dispatch(command)


class CommandConfirmedFacilityExecutionTest(unittest.TestCase):
    def setUp(self):
        waypoint = Waypoint(name='zone_c', x=3.0, y=0.0, yaw_deg=0.0, dwell_sec=3.0, timeout_sec=20.0, frame_region='zone_c', route_id='zone_c__2')
        step = MissionStep(
            waypoint=waypoint,
            index=0,
            total=1,
            task_type='facility_attack',
            metadata={'attack_mode': 'command_confirmed', 'confirmation_classes': ['enemy'], 'required_detection_count': 0},
            step_id='facility_c_cmd',
        )
        self.plan = MissionPlan.from_steps([step])
        self.context = MissionContext(dispatch_started_at=5.0)
        self.clock = _DummyClock(10.0)
        self.states = []
        self.executor = MissionExecutor(
            plan=self.plan,
            context=self.context,
            clock=self.clock,
            nav_adapter=_DummyNavAdapter(),
            capture_aggregator=CaptureWindowAggregator(AggregationPolicy(class_names=('friendly', 'enemy', 'hostage'))),
            recovery_policy=RetryThenFailPolicy(retry_limit=1, navigation_failure_quiesce_sec=0.5),
            hooks=MissionRuntimeHooks(
                emit_state=lambda event, state, details: self.states.append((event, state, details or {})),
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
            behavior_backend=MemoryBehaviorActionBackend(outcome_status='SUCCEEDED', completion_delay_sec=0.0),
            behavior_action_timeout_sec=2.0,
        )

    def test_command_confirmed_path_dispatches_real_behavior_action(self):
        step = self.plan.step_at(0)
        resolution = self.executor.behaviors.resolve('facility_attack').on_navigation_succeeded(self.executor, step, 10.0)
        self.assertIsNotNone(resolution.action_request)
        self.assertFalse(resolution.begin_dwell)
        self.executor._apply_navigation_success_resolution(step, resolution)
        self.assertEqual(self.context.state, 'TASK_ACTION')
        self.assertEqual(self.context.active_behavior_command['action_type'], 'facility_attack')

    def test_command_confirmed_finalize_uses_backend_feedback(self):
        step = self.plan.step_at(0)
        self.context.active_behavior_feedback = {'status': 'SUCCEEDED', 'details': {'source': 'memory'}}
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
        self.assertEqual(resolution.outcome, 'action_confirmed')
        self.assertTrue(resolution.result.action_summary['backend_feedback'])
        self.assertEqual(resolution.result.mission_outcome, 'facility_action_confirmed')


if __name__ == '__main__':
    unittest.main()

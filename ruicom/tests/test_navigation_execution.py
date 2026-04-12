import unittest

from ruikang_recon_baseline.common import Waypoint
from ruikang_recon_baseline.mission_plan import MissionPlan, MissionStep
from ruikang_recon_baseline.navigation_adapters.base import NavigationAdapterBase
from ruikang_recon_baseline.navigation_execution import NavigationRuntimeExecutive


class _DummyClock:
    def __init__(self, now_sec: float = 10.0):
        self._now_sec = float(now_sec)

    def now_business_sec(self) -> float:
        return self._now_sec


class _SequencedAdapter(NavigationAdapterBase):
    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.dispatched = []
        self.cancelled = False

    @property
    def cancel_has_ack(self) -> bool:
        return True

    def dispatch(self, waypoint):
        self.dispatched.append(waypoint.route_id)

    def poll(self, now_sec: float) -> str:
        _ = now_sec
        if not self.statuses:
            return 'IDLE'
        return self.statuses.pop(0)

    def cancel(self, now_sec=None) -> bool:
        _ = now_sec
        self.cancelled = True
        return True


class NavigationExecutionTest(unittest.TestCase):
    def test_dispatch_and_poll_track_runtime_metadata(self):
        step = MissionPlan.from_steps([
            MissionStep(
                waypoint=Waypoint(name='zone_a', x=1.0, y=0.0, yaw_deg=0.0, dwell_sec=0.0, timeout_sec=10.0, frame_region='zone_a', route_id='zone_a__0'),
                index=0,
                total=1,
                task_type='transit',
                step_id='step_a',
            )
        ]).step_at(0)
        executive = NavigationRuntimeExecutive(adapter=_SequencedAdapter(['PENDING', 'ACTIVE', 'SUCCEEDED']), clock=_DummyClock(10.0))
        record = executive.dispatch(step, now_sec=10.0)
        self.assertEqual(record.route_id, 'zone_a__0')
        self.assertEqual(executive.poll(10.0).status, 'DISPATCHED')
        active = executive.poll(10.5)
        self.assertEqual(active.status, 'ACTIVE')
        self.assertGreaterEqual(active.elapsed_sec, 0.5)
        done = executive.poll(11.0)
        self.assertEqual(done.status, 'SUCCEEDED')

    def test_cancel_updates_runtime_contract(self):
        step = MissionPlan.from_steps([
            MissionStep(
                waypoint=Waypoint(name='zone_b', x=2.0, y=0.0, yaw_deg=0.0, dwell_sec=0.0, timeout_sec=10.0, frame_region='zone_b', route_id='zone_b__1'),
                index=0,
                total=1,
                task_type='transit',
                step_id='step_b',
            )
        ]).step_at(0)
        adapter = _SequencedAdapter(['ACTIVE'])
        executive = NavigationRuntimeExecutive(adapter=adapter, clock=_DummyClock(10.0))
        executive.dispatch(step, now_sec=10.0)
        self.assertTrue(executive.cancel(10.2))
        summary = executive.runtime_contract()
        self.assertTrue(summary['cancel_has_ack'])
        self.assertEqual(summary['cancel_count'], 1)
        self.assertTrue(adapter.cancelled)


if __name__ == '__main__':
    unittest.main()

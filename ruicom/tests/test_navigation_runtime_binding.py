import unittest

from ruikang_recon_baseline.navigation_status_payloads import normalize_navigation_status_payload
from ruikang_recon_baseline.navigation_runtime import (
    CompositeNavigationAdapter,
    NavigationCanceller,
    NavigationGoalDispatcher,
    NavigationRuntimeBinding,
    NavigationStatusMonitor,
    NoOpCanceller,
)


class _FakeDispatcher(NavigationGoalDispatcher):
    def __init__(self):
        self.dispatched = []

    def dispatch(self, waypoint):
        self.dispatched.append(waypoint)


class _FakeStatusMonitor(NavigationStatusMonitor):
    status_source = 'external_status'

    def __init__(self):
        self.next_status = 'ACTIVE'

    def poll(self, now_sec: float) -> str:
        _ = now_sec
        return self.next_status


class _FakeCanceller(NavigationCanceller):
    supports_cancel = True
    cancel_has_ack = True

    def __init__(self):
        self.cancelled = []

    def cancel(self, now_sec=None) -> bool:
        self.cancelled.append(now_sec)
        return True


class NavigationRuntimeBindingTest(unittest.TestCase):
    def test_composite_adapter_delegates_to_explicit_runtime_roles(self):
        dispatcher = _FakeDispatcher()
        monitor = _FakeStatusMonitor()
        canceller = _FakeCanceller()
        adapter = CompositeNavigationAdapter(
            NavigationRuntimeBinding(dispatcher=dispatcher, status_monitor=monitor, canceller=canceller)
        )

        waypoint = object()
        adapter.dispatch(waypoint)
        self.assertEqual(dispatcher.dispatched, [waypoint])
        self.assertEqual(adapter.poll(10.0), 'ACTIVE')
        self.assertTrue(adapter.cancel(10.5))
        self.assertEqual(canceller.cancelled, [10.5])
        self.assertTrue(adapter.supports_cancel)
        self.assertTrue(adapter.cancel_has_ack)
        self.assertEqual(adapter.status_source, 'external_status')

    def test_runtime_binding_summary_exposes_role_split(self):
        adapter = CompositeNavigationAdapter(
            NavigationRuntimeBinding(
                dispatcher=_FakeDispatcher(),
                status_monitor=_FakeStatusMonitor(),
                canceller=_FakeCanceller(),
            )
        )
        summary = adapter.runtime_binding_summary()
        self.assertEqual(summary['dispatcher_role'], '_FakeDispatcher')
        self.assertEqual(summary['status_monitor_role'], '_FakeStatusMonitor')
        self.assertEqual(summary['canceller_role'], '_FakeCanceller')
        self.assertEqual(summary['status_source'], 'external_status')
        self.assertTrue(summary['supports_cancel'])
        self.assertTrue(summary['cancel_has_ack'])

    def test_noop_canceller_reports_no_explicit_cancel_ack(self):
        called = []
        canceller = NoOpCanceller(on_cancel=lambda now_sec: called.append(now_sec))
        self.assertFalse(canceller.supports_cancel)
        self.assertFalse(canceller.cancel_has_ack)
        self.assertFalse(canceller.cancel(2.0))
        self.assertEqual(called, [2.0])


if __name__ == '__main__':
    unittest.main()


class StatusTopicNormalizationTest(unittest.TestCase):
    def test_plain_status_payload_normalizes(self):
        self.assertEqual(normalize_navigation_status_payload('navigation_active'), 'ACTIVE')

    def test_json_status_payload_from_platform_bridge_normalizes(self):
        payload = {
            'status_list': [
                {'status': 1, 'text': 'moving'},
                {'status': 3, 'text': 'done'},
            ]
        }
        self.assertEqual(normalize_navigation_status_payload(str(payload).replace("'", '"')), 'SUCCEEDED')

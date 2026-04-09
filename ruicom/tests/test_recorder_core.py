import unittest

from ruikang_recon_baseline.common import ConfigurationError
from ruikang_recon_baseline.recorder_core import build_recorder_health_payload


class RecorderCoreHealthPayloadTest(unittest.TestCase):
    def test_health_payload_preserves_zero_business_time_for_sim_bootstrap(self):
        payload = build_recorder_health_payload(
            stamp_sec=0.0,
            runtime_state='IDLE',
            lifecycle_managed=True,
            status='ok',
            message='node_ready',
            details={'source': 'unit'},
        )
        self.assertEqual(payload['stamp'], 0.0)
        self.assertEqual(payload['node'], 'mission_recorder_node')
        self.assertEqual(payload['status'], 'ok')
        self.assertEqual(payload['message'], 'node_ready')
        self.assertEqual(payload['details']['runtime_state'], 'IDLE')
        self.assertTrue(payload['details']['lifecycle_managed'])
        self.assertEqual(payload['details']['source'], 'unit')

    def test_health_payload_rejects_empty_status_or_message(self):
        with self.assertRaises(ConfigurationError):
            build_recorder_health_payload(
                stamp_sec=1.0,
                runtime_state='ACTIVE',
                lifecycle_managed=False,
                status='',
                message='node_ready',
            )
        with self.assertRaises(ConfigurationError):
            build_recorder_health_payload(
                stamp_sec=1.0,
                runtime_state='ACTIVE',
                lifecycle_managed=False,
                status='ok',
                message='',
            )


if __name__ == '__main__':
    unittest.main()

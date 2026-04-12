import unittest

from ruikang_recon_baseline.vendor_feedback_core import VendorFeedbackSnapshot, evaluate_vendor_feedback


class VendorFeedbackCoreTest(unittest.TestCase):
    def test_fresh_true_source_with_recent_command_sets_feedback_true(self):
        decision = evaluate_vendor_feedback(
            VendorFeedbackSnapshot(
                now_sec=10.0,
                source_enabled=True,
                source_value=True,
                source_stamp_sec=9.7,
                source_timeout_sec=0.6,
                require_recent_command=True,
                command_stamp_sec=9.8,
                command_timeout_sec=0.5,
            )
        )
        self.assertTrue(decision.output_feedback)
        self.assertTrue(decision.source_fresh)
        self.assertTrue(decision.command_recent)
        self.assertEqual(decision.reason, 'explicit_feedback_confirmed')

    def test_true_source_is_gated_low_when_command_is_stale(self):
        decision = evaluate_vendor_feedback(
            VendorFeedbackSnapshot(
                now_sec=10.0,
                source_enabled=True,
                source_value=True,
                source_stamp_sec=9.8,
                source_timeout_sec=0.6,
                require_recent_command=True,
                command_stamp_sec=8.0,
                command_timeout_sec=0.5,
            )
        )
        self.assertFalse(decision.output_feedback)
        self.assertTrue(decision.source_fresh)
        self.assertFalse(decision.command_recent)
        self.assertEqual(decision.reason, 'command_stale')

    def test_source_false_overrides_recent_command(self):
        decision = evaluate_vendor_feedback(
            VendorFeedbackSnapshot(
                now_sec=10.0,
                source_enabled=True,
                source_value=False,
                source_stamp_sec=9.8,
                source_timeout_sec=0.6,
                require_recent_command=True,
                command_stamp_sec=9.9,
                command_timeout_sec=0.5,
            )
        )
        self.assertFalse(decision.output_feedback)
        self.assertTrue(decision.source_fresh)
        self.assertTrue(decision.command_recent)
        self.assertEqual(decision.reason, 'source_false')

    def test_source_disabled_forces_feedback_false(self):
        decision = evaluate_vendor_feedback(
            VendorFeedbackSnapshot(
                now_sec=10.0,
                source_enabled=False,
                source_value=True,
                source_stamp_sec=9.8,
                source_timeout_sec=0.6,
                require_recent_command=False,
                command_stamp_sec=0.0,
                command_timeout_sec=1.0,
            )
        )
        self.assertFalse(decision.output_feedback)
        self.assertFalse(decision.source_fresh)
        self.assertEqual(decision.reason, 'source_disabled')


if __name__ == '__main__':
    unittest.main()

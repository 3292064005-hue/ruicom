import unittest

from ruikang_recon_baseline.common import ConfigurationError
from ruikang_recon_baseline.vendor_sidecar_contract_core import (
    VendorSidecarFeedbackContract,
    validate_vendor_sidecar_feedback_contract,
)


class VendorSidecarFeedbackContractCoreTest(unittest.TestCase):
    def test_adapter_mode_requires_distinct_vendor_source_topic(self):
        with self.assertRaises(ConfigurationError):
            validate_vendor_sidecar_feedback_contract(
                VendorSidecarFeedbackContract(
                    require_explicit_feedback_source=True,
                    vendor_execution_feedback_topic='recon/platform/vendor/base_feedback_raw',
                    upstream_feedback_topic='recon/platform/vendor/base_feedback_raw',
                    native_feedback_source_declared=False,
                )
            )

    def test_contract_rejects_undeclared_feedback_source_when_required(self):
        with self.assertRaises(ConfigurationError):
            validate_vendor_sidecar_feedback_contract(
                VendorSidecarFeedbackContract(
                    require_explicit_feedback_source=True,
                    vendor_execution_feedback_topic='',
                    upstream_feedback_topic='recon/platform/vendor/base_feedback_raw',
                    native_feedback_source_declared=False,
                )
            )

    def test_contract_rejects_ambiguous_adapter_and_native_modes(self):
        with self.assertRaises(ConfigurationError):
            validate_vendor_sidecar_feedback_contract(
                VendorSidecarFeedbackContract(
                    require_explicit_feedback_source=True,
                    vendor_execution_feedback_topic='/vendor/chassis/feedback',
                    upstream_feedback_topic='recon/platform/vendor/base_feedback_raw',
                    native_feedback_source_declared=True,
                )
            )

    def test_native_mode_is_accepted_when_explicitly_declared(self):
        decision = validate_vendor_sidecar_feedback_contract(
            VendorSidecarFeedbackContract(
                require_explicit_feedback_source=True,
                vendor_execution_feedback_topic='',
                upstream_feedback_topic='recon/platform/vendor/base_feedback_raw',
                native_feedback_source_declared=True,
            )
        )
        self.assertEqual(decision.source_mode, 'native')

    def test_adapter_mode_is_accepted_when_vendor_topic_is_declared(self):
        decision = validate_vendor_sidecar_feedback_contract(
            VendorSidecarFeedbackContract(
                require_explicit_feedback_source=True,
                vendor_execution_feedback_topic='/vendor/chassis/feedback',
                upstream_feedback_topic='recon/platform/vendor/base_feedback_raw',
                native_feedback_source_declared=False,
            )
        )
        self.assertEqual(decision.source_mode, 'adapter')
        self.assertEqual(decision.vendor_execution_feedback_topic, '/vendor/chassis/feedback')


if __name__ == '__main__':
    unittest.main()

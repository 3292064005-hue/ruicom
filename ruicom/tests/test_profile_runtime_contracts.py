import unittest

from ruikang_recon_baseline.common import ConfigurationError, validate_profile_runtime_flags


class ProfileRuntimeContractTest(unittest.TestCase):
    def test_deploy_component_validation_allows_non_mission_flags_to_be_omitted(self):
        normalized = validate_profile_runtime_flags(
            'deploy',
            owner='unit.component',
            lifecycle_managed=True,
        )
        self.assertEqual(normalized, 'deploy')

    def test_deploy_component_validation_rejects_unmanaged_runtime(self):
        with self.assertRaises(ConfigurationError):
            validate_profile_runtime_flags(
                'deploy',
                owner='unit.component',
                lifecycle_managed=False,
            )

    def test_deploy_mission_validation_still_enforces_route_binding_and_autostart(self):
        with self.assertRaises(ConfigurationError):
            validate_profile_runtime_flags(
                'deploy',
                owner='unit.mission',
                lifecycle_managed=True,
                auto_start=True,
                require_route_frame_regions=True,
            )
        with self.assertRaises(ConfigurationError):
            validate_profile_runtime_flags(
                'deploy',
                owner='unit.mission',
                lifecycle_managed=True,
                auto_start=False,
                require_route_frame_regions=False,
            )


if __name__ == '__main__':
    unittest.main()

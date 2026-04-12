import json
import tempfile
import unittest
import yaml
from pathlib import Path

from ruikang_recon_baseline.common import ConfigurationError
from ruikang_recon_baseline.deploy_contracts import validate_deploy_stage_contract
from ruikang_recon_baseline.field_assets import load_field_asset
from ruikang_recon_baseline.mission_config import read_mission_config
from ruikang_recon_baseline.submission_adapters import load_and_validate_submission_contract
from ruikang_recon_baseline.vendor_bundle_preflight import build_vendor_bundle_preflight_report


class DeployStageContractTest(unittest.TestCase):
    def test_field_stage_requires_manifest(self):
        with self.assertRaises(ConfigurationError):
            validate_deploy_stage_contract(
                {
                    'profile_role': 'deploy',
                    'required_field_asset_verification_scope': 'field',
                    'require_verified_field_asset': True,
                    'model_manifest_path': '',
                },
                owner='unit.field',
            )


    def test_reference_stage_requires_manifest(self):
        with self.assertRaises(ConfigurationError):
            validate_deploy_stage_contract(
                {
                    'profile_role': 'deploy',
                    'required_field_asset_verification_scope': 'reference',
                    'require_verified_field_asset': True,
                    'model_manifest_path': '',
                },
                owner='unit.reference',
            )

    def test_reference_stage_marks_manifest_required(self):
        report = validate_deploy_stage_contract(
            {
                'profile_role': 'deploy',
                'required_field_asset_verification_scope': 'reference',
                'require_verified_field_asset': True,
                'model_manifest_path': 'config/manifests/mowen_reference_field_detector_manifest.json',
            },
            owner='unit.reference',
        )
        self.assertTrue(report['model_manifest_required'])


class FieldAssetProvenanceTest(unittest.TestCase):
    def test_field_verified_asset_requires_provenance(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / 'field.yaml'
            path.write_text(
                '\n'.join(
                    [
                        'field_asset_id: test',
                        'comparison_frame: map',
                        'route:',
                        '  - zone_name: zone_a',
                        '    goal: {x: 1.0, y: 2.0, yaw_deg: 0}',
                        'named_regions:',
                        '  - {name: zone_a, x0: 0, y0: 0, x1: 10, y1: 10}',
                        'asset_state: measured',
                        'verified: true',
                        'verification_scope: field',
                    ]
                ),
                encoding='utf-8',
            )
            with self.assertRaises(ConfigurationError):
                load_field_asset(field_asset_path=str(path))


class VendorBundlePreflightTest(unittest.TestCase):
    def test_managed_entrypoints_are_reported(self):
        report = build_vendor_bundle_preflight_report(
            {
                'vendor_workspace_name': 'newznzc_ws',
                'vendor_entrypoints': {},
                'managed_entrypoints': {
                    'repo_sidecar_launch': 'launch/mowen_vendor_sidecar.launch',
                    'repo_sidecar_wrapper': 'tools/run_mowen_vendor_sidecar.sh',
                },
            },
            {
                'vendor_runtime_mode': 'isolated_legacy_workspace',
                'vendor_bundle_preflight_mode': 'advisory',
                'vendor_workspace_root': '',
                'vendor_workspace_root_env': '',
            },
        )
        self.assertTrue(report['managed_entrypoints'])
        self.assertFalse(report['satisfied'])
        self.assertEqual(report['status'], 'external_bundle_unresolved')


class SubmissionContractTest(unittest.TestCase):
    def test_contract_id_is_required(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / 'contract.json'
            path.write_text(json.dumps({'adapter_type': 'file_drop', 'endpoint_path': str(path)}), encoding='utf-8')
            with self.assertRaises(ConfigurationError):
                load_and_validate_submission_contract(str(path))


class _FakeRospy:
    def __init__(self, params):
        self._params = dict(params)

    def get_param(self, name, default=None):
        key = str(name)
        if key in self._params:
            return self._params[key]
        normalized = key[1:] if key.startswith('~') else key
        return self._params.get(normalized, default)


class MissionConfigDeployContractTest(unittest.TestCase):
    def _load_profile(self, rel_path: str):
        payload = json.loads(json.dumps(yaml.safe_load((Path(__file__).resolve().parents[1] / rel_path).read_text(encoding='utf-8')) or {}))
        payload.setdefault('platform_adapter_type', 'generic_ros_nav')
        payload.setdefault('vendor_runtime_mode', 'native_noetic')
        payload.setdefault('vendor_workspace_ros_distro', 'noetic')
        payload.setdefault('vendor_workspace_python_major', 3)
        payload.setdefault('motion_model', 'differential')
        payload.setdefault('cmd_vel_semantics', 'planar_x_yaw')
        payload.setdefault('allow_odom_feedback_fallback', True)
        payload.setdefault('vendor_bundle_preflight_mode', 'off')
        return payload

    def test_reference_profile_records_manifest_contract(self):
        params = self._load_profile('config/profiles/reference_deploy/mission.yaml')
        config = read_mission_config(_FakeRospy(params))
        self.assertEqual(config['deploy_stage_contract']['deploy_stage'], 'reference')
        self.assertTrue(config['deploy_stage_contract']['model_manifest_required'])
        self.assertTrue(str(config['model_manifest_path']).endswith('config/manifests/mowen_reference_field_detector_manifest.json'))
        self.assertEqual(config['deploy_manifest_contract']['required_scope'], 'reference')
        self.assertTrue(config['deploy_manifest_contract']['scope_satisfied'])

    def test_field_profile_records_field_manifest_contract_once_resolved(self):
        params = self._load_profile('config/profiles/field_deploy/mission.yaml')
        config = read_mission_config(_FakeRospy(params))
        self.assertEqual(config['deploy_stage_contract']['deploy_stage'], 'field')
        self.assertTrue(config['deploy_stage_contract']['model_manifest_required'])
        self.assertTrue(str(config['model_manifest_path']).endswith('config/manifests/mowen_packaged_field_detector_manifest.json'))
        self.assertEqual(config['deploy_manifest_contract']['required_scope'], 'field')
        self.assertTrue(config['deploy_manifest_contract']['scope_satisfied'])

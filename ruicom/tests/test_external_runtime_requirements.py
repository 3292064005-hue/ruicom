import os
import tempfile
import unittest
from pathlib import Path

import yaml

from ruikang_recon_baseline.field_assets import apply_field_asset_to_vision_config
from ruikang_recon_baseline.model_requirements import (
    enforce_onnx_model_requirement,
    resolve_onnx_model_requirement,
)


class ExternalRuntimeRequirementTest(unittest.TestCase):
    def test_field_asset_release_keeps_env_bound_onnx_requirement_explicit(self):
        root = Path(__file__).resolve().parents[1]
        payload = yaml.safe_load((root / 'config/profiles/reference_deploy/vision.yaml').read_text(encoding='utf-8')) or {}
        resolved, _ = apply_field_asset_to_vision_config(dict(payload), owner='reference_deploy.vision')
        self.assertEqual(resolved.get('detector_type'), 'onnx')
        self.assertEqual(resolved.get('onnx_model_path', ''), '')
        self.assertEqual(resolved.get('onnx_model_path_env', ''), 'RUIKANG_REFERENCE_ONNX_MODEL')

    def test_env_bound_requirement_can_be_materialized(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            model = Path(tmpdir) / 'model.onnx'
            model.write_bytes(b'not-validated-in-static-ci')
            old = os.environ.get('RUIKANG_REFERENCE_ONNX_MODEL', '')
            os.environ['RUIKANG_REFERENCE_ONNX_MODEL'] = str(model)
            try:
                root = Path(__file__).resolve().parents[1]
                payload = yaml.safe_load((root / 'config/profiles/reference_deploy/vision.yaml').read_text(encoding='utf-8')) or {}
                payload['onnx_model_path'] = os.environ['RUIKANG_REFERENCE_ONNX_MODEL']
                resolved, _ = apply_field_asset_to_vision_config(dict(payload), owner='reference_deploy.vision')
                self.assertEqual(resolved.get('onnx_model_path'), str(model))
            finally:
                if old:
                    os.environ['RUIKANG_REFERENCE_ONNX_MODEL'] = old
                else:
                    os.environ.pop('RUIKANG_REFERENCE_ONNX_MODEL', None)

    def test_missing_env_bound_model_reports_actionable_status(self):
        report = resolve_onnx_model_requirement(
            {'detector_type': 'onnx', 'onnx_model_path_env': 'RUIKANG_REFERENCE_ONNX_MODEL'},
            environ={},
        )
        self.assertFalse(report['satisfied'])
        self.assertEqual(report['source'], 'env')
        self.assertEqual(report['binding'], 'RUIKANG_REFERENCE_ONNX_MODEL')
        self.assertEqual(report['status'], 'missing_model_path')

    def test_enforced_model_requirement_resolves_and_records_path(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            model = Path(tmpdir) / 'model.onnx'
            model.write_bytes(b'not-validated-in-static-ci')
            config = {
                'detector_type': 'onnx',
                'onnx_model_path': str(model),
                'onnx_model_path_env': 'RUIKANG_REFERENCE_ONNX_MODEL',
            }
            report = enforce_onnx_model_requirement(config, owner='unit.vision')
            self.assertTrue(report['satisfied'])
            self.assertEqual(config['onnx_model_path'], str(model))
            self.assertEqual(config['onnx_model_requirement']['status'], 'validated')


if __name__ == '__main__':
    unittest.main()

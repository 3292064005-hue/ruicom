import tempfile
import unittest
from pathlib import Path

from ruikang_recon_baseline.field_asset_release import load_field_asset_release
from ruikang_recon_baseline.field_asset_release_builder import build_and_validate_field_asset_release


ROOT = Path(__file__).resolve().parents[1]


class FieldAssetReleaseBuilderTest(unittest.TestCase):
    def test_build_round_trip(self):
        with tempfile.TemporaryDirectory(prefix='field_asset_release_builder_') as tmpdir:
            target = build_and_validate_field_asset_release(
                repo_root=ROOT,
                output_path=str(Path(tmpdir) / 'generated.yaml'),
                release_id='generated_release',
                release_version='1.0.0',
                field_asset_id='mowen_raicom_packaged_field_verified',
                verification_scope='field',
                detector_manifest_path='config/manifests/mowen_packaged_field_detector_manifest.json',
                approved_at_utc='2026-04-13T00:00:00Z',
                reviewer_id='unit_test',
                notes='round trip',
            )
            release = load_field_asset_release(str(target))
            self.assertIsNotNone(release)
            self.assertEqual(release.release_id, 'generated_release')
            self.assertEqual(release.verification_scope, 'field')


if __name__ == '__main__':
    unittest.main()

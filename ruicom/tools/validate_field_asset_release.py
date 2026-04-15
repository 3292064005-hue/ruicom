#!/usr/bin/env python3
"""Validate repository-managed field-asset release manifests."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.field_asset_release import load_field_asset_release  # noqa: E402
from ruikang_recon_baseline.field_asset_release_builder import build_and_validate_field_asset_release  # noqa: E402


def main() -> int:
    release_dir = ROOT / 'config' / 'field_assets' / 'releases'
    manifests = sorted(release_dir.glob('*.yaml'))
    if not manifests:
        raise SystemExit('no field-asset release manifests found')
    for path in manifests:
        release = load_field_asset_release(str(path))
        assert release is not None
        print(f'{path.relative_to(ROOT)}: release_id={release.release_id} scope={release.verification_scope} asset={release.field_asset.asset_id} detector={release.detector_model_id}')
    import tempfile
    with tempfile.TemporaryDirectory(prefix='field_asset_release_build_') as tmpdir:
        generated = build_and_validate_field_asset_release(
            repo_root=ROOT,
            output_path=str(Path(tmpdir) / 'generated_release.yaml'),
            release_id='generated_packaged_field_release',
            release_version='1.0.0',
            field_asset_id='mowen_raicom_packaged_field_verified',
            verification_scope='field',
            detector_manifest_path='config/manifests/mowen_packaged_field_detector_manifest.json',
            approved_at_utc='2026-04-13T00:00:00Z',
            reviewer_id='validator_generated_release',
            notes='validator round-trip build check',
        )
        print(f'{generated}: generated release build round-trip validated')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

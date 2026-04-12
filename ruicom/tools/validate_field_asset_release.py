#!/usr/bin/env python3
"""Validate repository-managed field-asset release manifests."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.field_asset_release import load_field_asset_release  # noqa: E402


def main() -> int:
    release_dir = ROOT / 'config' / 'field_assets' / 'releases'
    manifests = sorted(release_dir.glob('*.yaml'))
    if not manifests:
        raise SystemExit('no field-asset release manifests found')
    for path in manifests:
        release = load_field_asset_release(str(path))
        assert release is not None
        print(f'{path.relative_to(ROOT)}: release_id={release.release_id} scope={release.verification_scope} asset={release.field_asset.asset_id} detector={release.detector_model_id}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

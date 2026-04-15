#!/usr/bin/env python3
"""Build one field-asset release manifest from explicit inputs."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.field_asset_release_builder import build_and_validate_field_asset_release  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Create a validated field asset release manifest from explicit production inputs')
    parser.add_argument('--release-id', required=True)
    parser.add_argument('--release-version', required=True)
    parser.add_argument('--field-asset-id', default='')
    parser.add_argument('--field-asset-path', default='')
    parser.add_argument('--verification-scope', choices=['contract', 'reference', 'field'], required=True)
    parser.add_argument('--detector-manifest-path', required=True)
    parser.add_argument('--approved-at-utc', required=True)
    parser.add_argument('--reviewer-id', required=True)
    parser.add_argument('--output-path', required=True)
    parser.add_argument('--notes', default='')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    target = build_and_validate_field_asset_release(
        repo_root=ROOT,
        output_path=args.output_path,
        release_id=args.release_id,
        release_version=args.release_version,
        field_asset_id=args.field_asset_id,
        field_asset_path=args.field_asset_path,
        verification_scope=args.verification_scope,
        detector_manifest_path=args.detector_manifest_path,
        approved_at_utc=args.approved_at_utc,
        reviewer_id=args.reviewer_id,
        notes=args.notes,
    )
    print(target)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Render one field asset's vision regions into a reviewable PNG.

The tool exists to make field-asset calibration and named-region authoring
visible during code review and deploy preparation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from ruikang_recon_baseline.field_assets import load_field_asset
from ruikang_recon_baseline.vision_core import FrameRegionAdapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Render calibrated field-asset regions to a PNG overlay.')
    parser.add_argument('--field-asset-id', default='', help='Field asset identifier under config/field_assets')
    parser.add_argument('--field-asset-path', default='', help='Explicit field asset YAML path')
    parser.add_argument('--width', type=int, default=640, help='Target render width in pixels')
    parser.add_argument('--height', type=int, default=480, help='Target render height in pixels')
    parser.add_argument('--output', required=True, help='Output PNG path')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    asset = load_field_asset(field_asset_id=args.field_asset_id, field_asset_path=args.field_asset_path)
    if asset is None:
        raise SystemExit('field asset is required')
    adapter_type = 'calibrated_named_regions' if asset.metadata.get('region_calibration') else 'named_regions'
    adapter = FrameRegionAdapter(adapter_type, asset.named_regions, region_calibration=asset.metadata.get('region_calibration', {}))
    region_payloads = adapter.region_payloads(args.width, args.height)
    canvas = np.zeros((args.height, args.width, 3), dtype=np.uint8)
    canvas[:] = (24, 24, 24)
    for idx, region in enumerate(region_payloads):
        x0, y0, x1, y1 = int(region['x0']), int(region['y0']), int(region['x1']), int(region['y1'])
        color = ((37 * (idx + 3)) % 255, (83 * (idx + 5)) % 255, (131 * (idx + 7)) % 255)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), color, 2)
        cv2.putText(canvas, str(region['name']), (x0 + 6, max(18, y0 + 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    cv2.putText(canvas, asset.asset_id, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (220, 220, 220), 2, cv2.LINE_AA)
    if asset.metadata.get('region_calibration'):
        cv2.putText(canvas, 'calibrated_named_regions', (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), canvas):
        raise SystemExit('failed to write {}'.format(output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

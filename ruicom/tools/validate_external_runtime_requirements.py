#!/usr/bin/env python3
"""Report deploy-time external requirements that cannot be proven in static CI.

This tool makes environment-bound artifacts explicit instead of silently treating
profile validation as proof that deploy profiles are runnable. It focuses on
items such as externally supplied ONNX model paths.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.field_assets import apply_field_asset_to_vision_config  # noqa: E402
from ruikang_recon_baseline.model_requirements import resolve_onnx_model_requirement  # noqa: E402


def load_yaml(path: Path):
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def collect_onnx_requirement(owner: str, payload: dict) -> dict | None:
    requirement = resolve_onnx_model_requirement(payload)
    if not requirement.get('required'):
        return None
    return {
        **requirement,
        'owner': owner,
    }


def main() -> int:
    failures = []
    for profile in ('reference_deploy', 'field_deploy'):
        payload = load_yaml(ROOT / 'config' / 'profiles' / profile / 'vision.yaml')
        resolved, _ = apply_field_asset_to_vision_config(dict(payload), owner=f'{profile}.vision')
        requirement = collect_onnx_requirement(f'{profile}.vision', resolved)
        if requirement is None:
            continue
        satisfied = requirement['satisfied']
        print('[external-runtime] {owner}: source={source} binding={binding} path={path} satisfied={ok}'.format(
            owner=requirement['owner'],
            source=requirement['source'],
            binding=requirement['binding'] or '<empty>',
            path=requirement['resolved_path'] or '<unset>',
            ok=satisfied,
        ))
        if not satisfied:
            failures.append(requirement)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Inspect one authoritative recorder summary artifact."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.authoritative_replay import build_authoritative_replay_manifest, load_authoritative_summary  # noqa: E402


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Inspect final_summary_v2.json and print authoritative replay metadata')
    parser.add_argument('summary_path')
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    summary = load_authoritative_summary(args.summary_path)
    manifest = build_authoritative_replay_manifest(summary, summary_path=str(Path(args.summary_path).resolve()), output_root=str(Path(args.summary_path).resolve().parent))
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations
import argparse
import shutil
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'vendor_workspace' / 'newznzc_ws'


def parse_args():
    p = argparse.ArgumentParser(description='Provision one external managed vendor workspace bundle')
    p.add_argument('--output-root', default='', help='Optional parent directory for the external workspace bundle')
    p.add_argument('--workspace-name', default='newznzc_ws')
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if not FIXTURE.exists():
        raise SystemExit(f'fixture workspace missing: {FIXTURE}')
    parent = Path(args.output_root).expanduser().resolve() if str(args.output_root).strip() else Path(tempfile.mkdtemp(prefix='ruikang_vendor_ws_')).resolve()
    parent.mkdir(parents=True, exist_ok=True)
    dest = parent / str(args.workspace_name).strip()
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(FIXTURE, dest)
    (dest / '.ruikang_managed_vendor_workspace').write_text('managed-bundle\n', encoding='utf-8')
    print(dest)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

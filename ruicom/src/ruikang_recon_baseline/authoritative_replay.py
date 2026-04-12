"""Authoritative replay helpers for recorder v2 artifacts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Mapping

from .domain_models import ConfigurationError


def _path_record(path: str) -> Dict[str, object]:
    normalized = str(path or '').strip()
    if not normalized:
        return {'path': '', 'exists': False}
    resolved = Path(normalized).expanduser().resolve()
    return {'path': str(resolved), 'exists': resolved.exists()}


def build_authoritative_replay_manifest(
    summary_v2: Mapping[str, object],
    *,
    summary_path: str,
    output_root: str,
    summary_legacy_path: str = '',
    runtime_metrics_path: str = '',
    official_report_path: str = '',
    official_report_receipt_path: str = '',
    summary_snapshot_v2_path: str = '',
    summary_snapshot_legacy_path: str = '',
    final_summary_v2_csv_path: str = '',
    final_summary_csv_path: str = '',
    mission_log_path: str = '',
) -> Dict[str, object]:
    """Build one replay manifest that indexes authoritative recorder artifacts.

    Args:
        summary_v2: Final authoritative summary payload.
        summary_path: Path to ``final_summary_v2.json``.
        output_root: Artifact output directory.
        summary_legacy_path: Path to legacy ``final_summary.json`` if produced.
        runtime_metrics_path: Path to ``runtime_metrics.json``.
        official_report_path: Path to ``official_report.json``.
        official_report_receipt_path: Path to submission receipt JSON.
        summary_snapshot_v2_path: Path to current v2 snapshot JSON.
        summary_snapshot_legacy_path: Path to current legacy snapshot JSON.
        final_summary_v2_csv_path: Path to authoritative v2 CSV export.
        final_summary_csv_path: Path to legacy CSV export.
        mission_log_path: Path to JSONL mission log.

    Returns:
        JSON-serializable replay manifest indexing the authoritative summary plus
        adjacent recorder artifacts.
    """
    zone_results_dynamic = dict(summary_v2.get('zone_results_dynamic', {}) or {})
    class_names = [str(item).strip() for item in (summary_v2.get('class_names', []) or []) if str(item).strip()]
    replay_artifacts = {
        'final_summary_v2': _path_record(summary_path),
        'final_summary_legacy': _path_record(summary_legacy_path),
        'runtime_metrics': _path_record(runtime_metrics_path),
        'official_report': _path_record(official_report_path),
        'official_report_receipt': _path_record(official_report_receipt_path),
        'summary_snapshot_v2': _path_record(summary_snapshot_v2_path),
        'summary_snapshot_legacy': _path_record(summary_snapshot_legacy_path),
        'final_summary_v2_csv': _path_record(final_summary_v2_csv_path),
        'final_summary_csv': _path_record(final_summary_csv_path),
        'mission_log': _path_record(mission_log_path),
    }
    return {
        'schema_version': str(summary_v2.get('schema_version', '')).strip(),
        'summary_v2_path': str(Path(summary_path).expanduser().resolve()),
        'output_root': str(Path(output_root).expanduser().resolve()),
        'class_names': class_names,
        'route_ids': sorted(zone_results_dynamic.keys()),
        'replay_source': 'final_summary_v2',
        'result_count': len(zone_results_dynamic),
        'replay_artifacts': replay_artifacts,
        'evidence_summary': dict(summary_v2.get('evidence_summary', {}) or {}),
        'mission_state': dict(summary_v2.get('mission_state', {}) or {}),
    }


def load_authoritative_summary(path: str) -> Dict[str, object]:
    resolved = Path(path).expanduser().resolve()
    payload = json.loads(resolved.read_text(encoding='utf-8'))
    if not isinstance(payload, Mapping):
        raise ConfigurationError(f'authoritative summary {resolved} must be a mapping')
    if 'zone_results_dynamic' not in payload:
        raise ConfigurationError(f'authoritative summary {resolved} missing zone_results_dynamic')
    return dict(payload)

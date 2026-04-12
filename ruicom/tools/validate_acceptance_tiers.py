#!/usr/bin/env python3
"""Validate deploy acceptance tiers across contract, reference and field profiles.

This tool enforces the staged deployment ladder introduced by the runtime-grade
and behavior-action refactor. It is intentionally narrow: it only inspects the
profile YAML files that define managed deploy semantics and exits non-zero when
one tier drifts from its declared contract.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, Iterable, Tuple

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = REPO_ROOT / 'config' / 'profiles'


class ValidationError(RuntimeError):
    """Raised when one acceptance tier violates the declared runtime contract."""


EXPECTED = {
    'baseline': {
        'runtime_grade': 'contract',
        'behavior_action_backend_type': 'disabled',
        'require_behavior_feedback': False,
    },
    'reference_deploy': {
        'runtime_grade': 'reference',
        'behavior_action_backend_type': 'topic',
        'require_behavior_feedback': True,
    },
    'field_deploy': {
        'runtime_grade': 'field',
        'behavior_action_backend_type': 'topic',
        'require_behavior_feedback': True,
    },
}

RUNTIME_GRADE_FILES = ('mission.yaml', 'platform.yaml', 'recorder.yaml', 'system_manager.yaml')


def _load_yaml(path: Path) -> Dict[str, object]:
    """Load one YAML document from disk.

    Args:
        path: Absolute or repository-relative path to the YAML file.

    Returns:
        Parsed mapping. Empty files are normalized to an empty dict.

    Raises:
        ValidationError: The file is missing or does not decode to a mapping.
    """
    if not path.exists():
        raise ValidationError(f'missing profile file: {path.relative_to(REPO_ROOT)}')
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        raise ValidationError(f'profile file must decode to a mapping: {path.relative_to(REPO_ROOT)}')
    return data


def _profile_file_map(profile_name: str) -> Dict[str, Dict[str, object]]:
    """Load the YAML contract files for one deploy profile."""
    profile_dir = CONFIG_ROOT / profile_name
    return {name: _load_yaml(profile_dir / name) for name in RUNTIME_GRADE_FILES if (profile_dir / name).exists()}


def validate_profile(profile_name: str) -> Dict[str, object]:
    """Validate one acceptance tier profile.

    Args:
        profile_name: Name of the deploy profile under ``config/profiles``.

    Returns:
        Compact summary dictionary that can be rendered in logs or artifacts.

    Raises:
        ValidationError: Any runtime-grade, backend, or feedback rule drifts.
    """
    expected = EXPECTED[profile_name]
    file_map = _profile_file_map(profile_name)
    runtime_grades = {
        file_name: str(payload.get('runtime_grade', '')).strip().lower()
        for file_name, payload in file_map.items()
        if file_name in RUNTIME_GRADE_FILES
    }
    missing = [name for name in RUNTIME_GRADE_FILES if name not in file_map]
    if missing:
        raise ValidationError(f'{profile_name}: missing required profile files: {", ".join(missing)}')
    drift = {name: grade for name, grade in runtime_grades.items() if grade != expected['runtime_grade']}
    if drift:
        rendered = ', '.join(f'{name}={grade!r}' for name, grade in sorted(drift.items()))
        raise ValidationError(f'{profile_name}: runtime_grade drift detected ({rendered}), expected {expected["runtime_grade"]!r}')

    mission = file_map['mission.yaml']
    backend = str(mission.get('behavior_action_backend_type', '')).strip().lower()
    require_feedback = bool(mission.get('require_behavior_feedback', False))
    if backend != expected['behavior_action_backend_type']:
        raise ValidationError(
            f'{profile_name}: behavior_action_backend_type={backend!r}, expected {expected["behavior_action_backend_type"]!r}'
        )
    if require_feedback != bool(expected['require_behavior_feedback']):
        raise ValidationError(
            f'{profile_name}: require_behavior_feedback={require_feedback!r}, expected {expected["require_behavior_feedback"]!r}'
        )
    return {
        'profile': profile_name,
        'runtime_grade': expected['runtime_grade'],
        'behavior_action_backend_type': backend,
        'require_behavior_feedback': require_feedback,
        'files_checked': list(RUNTIME_GRADE_FILES),
    }


def validate_all(profile_names: Iterable[str] = EXPECTED.keys()) -> Tuple[Dict[str, object], ...]:
    """Validate all known acceptance tiers and return deterministic summaries."""
    return tuple(validate_profile(str(name)) for name in profile_names)


def main() -> int:
    try:
        summaries = validate_all()
    except ValidationError as exc:
        print(f'[acceptance-tiers] {exc}', file=sys.stderr)
        return 1
    for summary in summaries:
        print(
            '[acceptance-tiers] '
            f'{summary["profile"]}: '
            f'grade={summary["runtime_grade"]} '
            f'backend={summary["behavior_action_backend_type"]} '
            f'require_feedback={summary["require_behavior_feedback"]}'
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

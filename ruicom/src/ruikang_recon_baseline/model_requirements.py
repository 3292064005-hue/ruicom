"""Pure helpers for external detector model binding validation."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

from .domain_models import ConfigurationError
from .runtime_paths import resolve_package_relative_path


def resolve_onnx_model_requirement(config: Mapping[str, object], environ: Mapping[str, str] | None = None) -> dict:
    """Resolve the configured ONNX model binding into a deterministic report."""
    detector_type = str(config.get('detector_type', '')).strip().lower()
    if detector_type != 'onnx':
        return {
            'required': False,
            'satisfied': True,
            'source': '',
            'binding': '',
            'raw_path': '',
            'resolved_path': '',
            'status': 'not_required',
        }
    environ = os.environ if environ is None else environ
    direct_path = str(config.get('onnx_model_path', '')).strip()
    env_name = str(config.get('onnx_model_path_env', '')).strip()
    if direct_path:
        source = 'path'
        binding = direct_path
        raw_path = direct_path
    else:
        source = 'env'
        binding = env_name
        raw_path = str(environ.get(env_name, '')).strip() if env_name else ''
    resolved_path = resolve_package_relative_path(raw_path) if raw_path else ''
    exists = bool(resolved_path) and Path(resolved_path).exists()
    if not raw_path:
        status = 'missing_model_path'
    elif not exists:
        status = 'model_path_not_found'
    else:
        status = 'validated'
    return {
        'required': True,
        'satisfied': bool(exists),
        'source': source,
        'binding': binding,
        'raw_path': raw_path,
        'resolved_path': resolved_path,
        'status': status,
    }


def enforce_onnx_model_requirement(config: dict, *, owner: str, environ: Mapping[str, str] | None = None) -> dict:
    """Attach and enforce the ONNX model binding report for a detector config."""
    report = resolve_onnx_model_requirement(config, environ=environ)
    config['onnx_model_requirement'] = dict(report)
    if report['required'] and report['satisfied']:
        config['onnx_model_path'] = str(report['resolved_path'])
        return report
    if not report['required']:
        return report
    if report['status'] == 'missing_model_path':
        binding_hint = ' or set {}'.format(report['binding']) if report['binding'] else ''
        raise ConfigurationError('{} requires an ONNX model path; set onnx_model_path{}'.format(owner, binding_hint))
    raise ConfigurationError(
        '{} ONNX model path does not exist: {} resolved to {}'.format(
            owner,
            report['binding'] or report['raw_path'] or '<empty>',
            report['resolved_path'] or '<unset>',
        )
    )

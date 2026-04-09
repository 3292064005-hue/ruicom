"""External vendor-runtime contract loaders and validators.

These helpers codify the boundary between this repository and an external
vendor workspace. They do not pretend the vendor runtime has been absorbed into
this repository; instead they make the dependency explicit, versioned, and
machine-validated.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping

import yaml

from .domain_models import ConfigurationError
from .runtime_paths import resolve_package_relative_path

VALID_BINDING_RESOURCE_TYPES = ('topic', 'action')
VALID_BINDING_DOMAINS = ('mission', 'vision', 'bridge')
_OPTIONAL_BINDING_METADATA_FIELDS = (
    'vendor_resource',
    'normalized_resource',
    'bridge_output',
    'vendor_entrypoint',
    'notes',
)
_OPTIONAL_CONTRACT_TOP_LEVEL_FIELDS = ('vendor_workspace_name', 'vendor_entrypoints')


def _normalize_binding_spec(domain: str, param_name: object, payload: object) -> dict:
    """Normalize one domain binding spec from the YAML contract.

    Args:
        domain: Contract domain name.
        param_name: Runtime configuration parameter bound by the contract.
        payload: Raw YAML payload for the binding.

    Returns:
        Normalized binding specification dictionary.

    Raises:
        ConfigurationError: If the domain, parameter name, resource type, or
            optional evidence metadata are invalid.

    Boundary behavior:
        Optional evidence metadata is preserved verbatim as strings so launch
        and health diagnostics can surface how one runtime binding is expected
        to be sourced from the external vendor workspace.
    """
    normalized_domain = str(domain or '').strip().lower()
    if normalized_domain not in VALID_BINDING_DOMAINS:
        raise ConfigurationError('vendor runtime contract domain must be one of {}'.format(', '.join(VALID_BINDING_DOMAINS)))
    normalized_param = str(param_name or '').strip()
    if not normalized_param:
        raise ConfigurationError('vendor runtime contract {} binding parameter name is empty'.format(normalized_domain))
    if not isinstance(payload, Mapping):
        raise ConfigurationError('vendor runtime contract {} binding {} must be a mapping'.format(normalized_domain, normalized_param))
    resource = str(payload.get('resource', 'topic')).strip().lower() or 'topic'
    if resource not in VALID_BINDING_RESOURCE_TYPES:
        raise ConfigurationError(
            'vendor runtime contract {} binding {} resource must be one of {}'.format(
                normalized_domain,
                normalized_param,
                ', '.join(VALID_BINDING_RESOURCE_TYPES),
            )
        )
    expected_type = str(payload.get('expected_type', '')).strip()
    normalized = {
        'resource': resource,
        'expected_type': expected_type,
    }
    for key in _OPTIONAL_BINDING_METADATA_FIELDS:
        raw_value = payload.get(key, '')
        if raw_value in (None, '', [], {}):
            continue
        normalized[key] = str(raw_value).strip()
    return normalized


def _normalize_vendor_entrypoints(raw_value: object, *, resolved: str) -> dict:
    """Normalize top-level vendor entrypoint metadata.

    Args:
        raw_value: Raw YAML value under ``vendor_entrypoints``.
        resolved: Resolved contract path for error messages.

    Returns:
        Mapping of entrypoint name to human-readable resource string.

    Raises:
        ConfigurationError: If the payload is not a mapping.

    Boundary behavior:
        Empty entrypoint names or values are discarded so optional documentation
        hints do not become hard contract blockers.
    """
    if raw_value in (None, '', [], {}):
        return {}
    if not isinstance(raw_value, Mapping):
        raise ConfigurationError('vendor runtime contract {} vendor_entrypoints must be a mapping'.format(resolved))
    return {
        str(name).strip(): str(value).strip()
        for name, value in raw_value.items()
        if str(name).strip() and str(value).strip()
    }


def load_vendor_runtime_contract(path: str, *, package_root: str | None = None) -> dict:
    """Load one vendor-runtime contract YAML file.

    Boundary behavior:
        Empty paths intentionally resolve to an empty contract so native Noetic
        stacks do not need to carry an external-vendor contract.
    """
    resolved = resolve_package_relative_path(path, package_root=package_root)
    if not resolved:
        return {}
    payload = yaml.safe_load(Path(resolved).read_text(encoding='utf-8')) or {}
    if not isinstance(payload, Mapping):
        raise ConfigurationError('vendor runtime contract {} must contain a mapping payload'.format(resolved))

    contract = {
        'contract_id': str(payload.get('contract_id', '')).strip(),
        'contract_version': int(payload.get('contract_version', 0) or 0),
        'platform_adapter_type': str(payload.get('platform_adapter_type', '')).strip(),
        'vendor_runtime_mode': str(payload.get('vendor_runtime_mode', '')).strip().lower(),
        'vendor_workspace_ros_distro': str(payload.get('vendor_workspace_ros_distro', '')).strip().lower(),
        'vendor_workspace_python_major': int(payload.get('vendor_workspace_python_major', 0) or 0),
        'vendor_workspace_name': str(payload.get('vendor_workspace_name', '')).strip(),
        'vendor_entrypoints': _normalize_vendor_entrypoints(payload.get('vendor_entrypoints', {}), resolved=resolved),
        'path': resolved,
    }
    bindings: Dict[str, Dict[str, dict]] = {}
    for domain in VALID_BINDING_DOMAINS:
        raw_bindings = payload.get('{}_required_bindings'.format(domain), {}) or {}
        if not isinstance(raw_bindings, Mapping):
            raise ConfigurationError('vendor runtime contract {} {}_required_bindings must be a mapping'.format(resolved, domain))
        bindings[domain] = {
            str(param_name).strip(): _normalize_binding_spec(domain, param_name, value)
            for param_name, value in raw_bindings.items()
            if str(param_name).strip()
        }
    contract['bindings'] = bindings
    if not contract['contract_id']:
        raise ConfigurationError('vendor runtime contract {} requires non-empty contract_id'.format(resolved))
    if contract['contract_version'] <= 0:
        raise ConfigurationError('vendor runtime contract {} requires positive contract_version'.format(resolved))
    if not contract['platform_adapter_type']:
        raise ConfigurationError('vendor runtime contract {} requires non-empty platform_adapter_type'.format(resolved))
    return contract


def build_vendor_runtime_binding_report(contract_summary: Mapping[str, object], config: Mapping[str, object]) -> dict:
    """Build one operator-facing binding report from a validated contract.

    Args:
        contract_summary: Output of :func:`validate_vendor_runtime_contract`.
        config: Active runtime configuration mapping.

    Returns:
        Structured report containing the active bindings and vendor evidence.

    Raises:
        No explicit exception is raised.

    Boundary behavior:
        Empty contracts return a satisfied empty report so native Noetic stacks
        can omit the external-vendor boundary entirely.
    """
    required_bindings = dict(contract_summary.get('required_bindings', {}) or {})
    evidence_bindings = []
    for param_name, binding in required_bindings.items():
        evidence_bindings.append(
            {
                'parameter': str(param_name).strip(),
                'configured_resource': str(config.get(param_name, '')).strip(),
                'resource_kind': str(binding.get('resource', '')).strip(),
                'expected_type': str(binding.get('expected_type', '')).strip(),
                'vendor_resource': str(binding.get('vendor_resource', '')).strip(),
                'normalized_resource': str(binding.get('normalized_resource', '')).strip(),
                'bridge_output': str(binding.get('bridge_output', '')).strip(),
                'vendor_entrypoint': str(binding.get('vendor_entrypoint', '')).strip(),
                'notes': str(binding.get('notes', '')).strip(),
            }
        )
    return {
        'satisfied': bool(contract_summary.get('satisfied', True)),
        'contract_id': str(contract_summary.get('contract_id', '')).strip(),
        'contract_version': int(contract_summary.get('contract_version', 0) or 0),
        'domain': str(contract_summary.get('domain', '')).strip(),
        'path': str(contract_summary.get('path', '')).strip(),
        'vendor_workspace_name': str(contract_summary.get('vendor_workspace_name', '')).strip(),
        'vendor_entrypoints': dict(contract_summary.get('vendor_entrypoints', {}) or {}),
        'bindings': evidence_bindings,
    }


def validate_vendor_runtime_contract(contract: Mapping[str, object], config: Mapping[str, object], *, owner: str, domain: str) -> dict:
    """Validate that one component satisfies the declared external-vendor contract."""
    normalized_domain = str(domain or '').strip().lower()
    if normalized_domain not in VALID_BINDING_DOMAINS:
        raise ConfigurationError('vendor runtime contract validation domain must be one of {}'.format(', '.join(VALID_BINDING_DOMAINS)))

    vendor_runtime_mode = str(config.get('vendor_runtime_mode', '')).strip().lower()
    contract_path = str(config.get('vendor_runtime_contract_path', '')).strip()
    if vendor_runtime_mode == 'isolated_legacy_workspace' and not contract:
        raise ConfigurationError('{} isolated_legacy_workspace runtime requires non-empty vendor_runtime_contract_path'.format(owner))
    if not contract:
        return {
            'satisfied': True,
            'contract_id': '',
            'contract_version': 0,
            'domain': normalized_domain,
            'path': '',
            'vendor_workspace_name': '',
            'vendor_entrypoints': {},
            'required_bindings': {},
        }

    adapter_type = str(config.get('platform_adapter_type', '')).strip()
    contract_adapter_type = str(contract.get('platform_adapter_type', '')).strip()
    if contract_adapter_type and adapter_type and contract_adapter_type != adapter_type:
        raise ConfigurationError('{} vendor_runtime_contract adapter {} must match platform_adapter_type {}'.format(owner, contract_adapter_type, adapter_type))

    expected_runtime_mode = str(contract.get('vendor_runtime_mode', '')).strip().lower()
    expected_ros_distro = str(contract.get('vendor_workspace_ros_distro', '')).strip().lower()
    expected_python_major = int(contract.get('vendor_workspace_python_major', 0) or 0)
    vendor_workspace_ros_distro = str(config.get('vendor_workspace_ros_distro', '')).strip().lower()
    vendor_workspace_python_major = int(config.get('vendor_workspace_python_major', 0) or 0)
    if expected_runtime_mode and vendor_runtime_mode != expected_runtime_mode:
        raise ConfigurationError('{} vendor_runtime_mode {} must match vendor runtime contract {}'.format(owner, vendor_runtime_mode, expected_runtime_mode))
    if expected_ros_distro and vendor_workspace_ros_distro != expected_ros_distro:
        raise ConfigurationError('{} vendor_workspace_ros_distro {} must match vendor runtime contract {}'.format(owner, vendor_workspace_ros_distro, expected_ros_distro))
    if expected_python_major and vendor_workspace_python_major != expected_python_major:
        raise ConfigurationError('{} vendor_workspace_python_major {} must match vendor runtime contract {}'.format(owner, vendor_workspace_python_major, expected_python_major))

    required_bindings = dict((contract.get('bindings') or {}).get(normalized_domain, {}) or {})
    missing = {}
    for param_name, binding in required_bindings.items():
        if not str(config.get(param_name, '')).strip():
            missing[param_name] = dict(binding)
    if missing:
        raise ConfigurationError('{} vendor runtime contract {} missing {} bindings: {}'.format(owner, contract.get('contract_id', ''), normalized_domain, sorted(missing.keys())))

    return {
        'satisfied': True,
        'contract_id': str(contract.get('contract_id', '')).strip(),
        'contract_version': int(contract.get('contract_version', 0) or 0),
        'domain': normalized_domain,
        'path': contract_path or str(contract.get('path', '')).strip(),
        'vendor_workspace_name': str(contract.get('vendor_workspace_name', '')).strip(),
        'vendor_entrypoints': dict(contract.get('vendor_entrypoints', {}) or {}),
        'required_bindings': required_bindings,
    }

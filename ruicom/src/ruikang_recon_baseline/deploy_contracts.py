"""Deploy-stage contract helpers.

These helpers keep contract, reference-field and real-field entry points aligned
with explicit runtime policy instead of relying on launch-file convention only.
"""

from __future__ import annotations

from typing import Mapping

from .domain_models import ConfigurationError

_VALID_DEPLOY_STAGES = ('contract', 'reference', 'field')


def normalize_deploy_stage(value: object, *, owner: str) -> str:
    """Normalize one deploy-stage string.

    Args:
        value: Raw stage value.
        owner: Human-readable configuration owner used in error messages.

    Returns:
        Normalized stage string.

    Raises:
        ConfigurationError: If the stage is unknown.

    Boundary behavior:
        Empty values default to ``contract`` so compatibility aliases remain
        conservative unless they explicitly request a stronger stage.
    """
    normalized = str(value or '').strip().lower() or 'contract'
    if normalized not in _VALID_DEPLOY_STAGES:
        raise ConfigurationError('{} deploy_stage must be one of {}'.format(owner, ', '.join(_VALID_DEPLOY_STAGES)))
    return normalized


def validate_deploy_stage_contract(config: Mapping[str, object], *, owner: str) -> dict:
    """Validate deploy-stage specific runtime gates.

    Args:
        config: Active runtime configuration mapping.
        owner: Human-readable owner for diagnostics.

    Returns:
        One JSON-serializable deploy-stage summary.

    Raises:
        ConfigurationError: If one deploy-stage hard gate is violated.

    Boundary behavior:
        The helper is intentionally conservative: ``deploy`` profiles default to
        ``contract`` stage unless they request ``reference`` or ``field`` via
        ``required_field_asset_verification_scope``. Once ``reference`` or
        ``field`` is requested, detector-manifest presence becomes a hard gate
        instead of a reporting hint so later profile additions cannot silently
        bypass deploy-stage policy.
    """
    profile_role = str(config.get('profile_role', '')).strip().lower()
    required_scope = normalize_deploy_stage(config.get('required_field_asset_verification_scope', ''), owner=owner)
    report = {
        'profile_role': profile_role,
        'deploy_stage': required_scope,
        'synthetic_input_allowed': required_scope != 'field',
        'requires_verified_field_asset': bool(config.get('require_verified_field_asset', False)),
        'model_manifest_required': bool(profile_role == 'deploy' and required_scope in ('reference', 'field')),
    }
    if profile_role == 'deploy' and required_scope == 'field' and not bool(config.get('require_verified_field_asset', False)):
        raise ConfigurationError('{} field deploy requires require_verified_field_asset=true'.format(owner))
    manifest_required = bool(profile_role == 'deploy' and required_scope in ('reference', 'field'))
    if manifest_required and not str(config.get('model_manifest_path', '')).strip():
        raise ConfigurationError('{} {} deploy requires non-empty model_manifest_path'.format(owner, required_scope))
    return report

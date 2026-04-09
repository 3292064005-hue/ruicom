"""Model-manifest and named-region loader helpers."""

from __future__ import annotations

import json
from pathlib import Path

from .domain_models import ConfigurationError, ModelManifest, NamedRegion

VALID_MANIFEST_DEPLOYMENT_GRADES = ('contract_verified', 'reference_verified', 'field_verified')


def validate_named_regions(named_regions):
    normalized = []
    for idx, payload in enumerate(named_regions or []):
        try:
            region = NamedRegion(
                name=str(payload['name']).strip(),
                x0=int(payload['x0']),
                y0=int(payload['y0']),
                x1=int(payload['x1']),
                y1=int(payload['y1']),
            )
        except Exception as exc:
            raise ConfigurationError('named_regions[{}] is invalid: {}'.format(idx, exc))
        if not region.name:
            raise ConfigurationError('named_regions[{}].name is empty'.format(idx))
        if region.x1 <= region.x0 or region.y1 <= region.y0:
            raise ConfigurationError('named_regions[{}] has non-positive extent'.format(idx))
        normalized.append(region)
    names = [item.name for item in normalized]
    if len(set(names)) != len(names):
        raise ConfigurationError('named_regions contains duplicate names: {}'.format(names))
    return normalized


def normalize_manifest_deployment_grade(raw_grade: object) -> str:
    grade = str(raw_grade or '').strip().lower()
    if grade and grade not in VALID_MANIFEST_DEPLOYMENT_GRADES:
        raise ConfigurationError(
            'model manifest deployment_grade must be one of {}'.format(', '.join(VALID_MANIFEST_DEPLOYMENT_GRADES))
        )
    return grade


def detector_manifest_satisfies_scope(manifest: ModelManifest, *, required_scope: str) -> bool:
    """Return whether one detector manifest satisfies the requested deploy grade.

    Args:
        manifest: Parsed model manifest.
        required_scope: Requested verification scope. ``contract`` accepts
            ``contract_verified``, ``reference_verified`` and ``field_verified``.
            ``reference`` accepts ``reference_verified`` and ``field_verified``.
            ``field`` accepts only ``field_verified``. Empty values disable grade filtering.

    Returns:
        ``True`` when the manifest grade is strong enough for the requested
        deploy scope.

    Raises:
        ConfigurationError: If the requested scope is unknown.
    """
    scope = str(required_scope or '').strip().lower()
    if not scope:
        return True
    if scope not in ('contract', 'reference', 'field'):
        raise ConfigurationError('required detector manifest scope must be contract, reference or field')
    grade = normalize_manifest_deployment_grade(manifest.deployment_grade)
    if not grade:
        return False
    if scope == 'contract':
        return grade in ('contract_verified', 'reference_verified', 'field_verified')
    if scope == 'reference':
        return grade in ('reference_verified', 'field_verified')
    return grade == 'field_verified'


def load_manifest(path: str) -> ModelManifest:
    payload = json.loads(Path(path).read_text(encoding='utf-8'))
    if 'class_names' not in payload or not payload['class_names']:
        raise ConfigurationError('model manifest requires non-empty class_names')
    detector_type = str(payload.get('detector_type', '')).strip().lower()
    return ModelManifest(
        input_size=int(payload['input_size']),
        class_names=[str(item) for item in payload['class_names']],
        parser_type=str(payload['parser_type']).strip(),
        confidence_threshold=float(payload['confidence_threshold']),
        score_threshold=float(payload['score_threshold']),
        nms_threshold=float(payload['nms_threshold']),
        detector_type=detector_type,
        deployment_grade=normalize_manifest_deployment_grade(payload.get('deployment_grade', '')),
        model_id=str(payload.get('model_id', '')).strip(),
    )

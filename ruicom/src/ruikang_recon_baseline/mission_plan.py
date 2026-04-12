"""Mission plan models independent of ROS transport concerns."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple

from .common import ConfigurationError, Waypoint, load_waypoints

LEGACY_TASK_DEFAULT = 'waypoint_capture'
TASK_TYPE_ALIASES = {
    'waypoint_capture': 'recon_zone',
    'waypoint_only': 'transit',
}
SUPPORTED_TASK_TYPES = (
    'waypoint_capture',
    'waypoint_only',
    'transit',
    'recon_zone',
    'hazard_avoid',
    'facility_attack',
    'finish',
)
DEFAULT_OBJECTIVE_BY_EXECUTION_TYPE = {
    'recon_zone': 'recon',
    'transit': 'transit',
    'hazard_avoid': 'hazard_avoid',
    'facility_attack': 'facility_attack',
    'finish': 'finish',
}


def canonical_execution_type(task_type: str) -> str:
    """Return the canonical execution type consumed by the runtime executor.

    Args:
        task_type: Declared mission-step task type.

    Returns:
        Canonical execution type used by the executor.

    Raises:
        ValueError: If ``task_type`` is unsupported.

    Boundary behavior:
        Legacy declarative types remain accepted and are projected onto the richer
        execution types so existing route-only missions stay runnable.
    """
    normalized = str(task_type or '').strip() or LEGACY_TASK_DEFAULT
    if normalized not in SUPPORTED_TASK_TYPES:
        raise ValueError('unsupported mission task_type {}'.format(normalized))
    return TASK_TYPE_ALIASES.get(normalized, normalized)



def default_objective_type(task_type: str, explicit_objective: str = '') -> str:
    """Resolve the semantic objective bound to one task step."""
    normalized = str(explicit_objective or '').strip()
    if normalized:
        return normalized
    return DEFAULT_OBJECTIVE_BY_EXECUTION_TYPE[canonical_execution_type(task_type)]



def _normalize_metadata(metadata: Mapping[str, Any] | None, *, owner: str) -> Mapping[str, Any]:
    if metadata in (None, ''):
        return {}
    if not isinstance(metadata, Mapping):
        raise ConfigurationError('{} metadata must be a mapping'.format(owner))
    return dict(metadata)



def _validate_task_metadata(task_type: str, metadata: Mapping[str, Any], *, owner: str) -> Mapping[str, Any]:
    execution_type = canonical_execution_type(task_type)
    normalized = dict(_normalize_metadata(metadata, owner=owner))
    if execution_type == 'hazard_avoid':
        normalized.setdefault('hazard_id', '')
        normalized.setdefault('hazard_type', 'anti_tank_cone')
        normalized.setdefault('avoidance_strategy', 'navigation_detour')
        normalized.setdefault('hazard_region', '')
        confirmation_classes = normalized.get('confirmation_classes', [])
        if confirmation_classes in (None, ''):
            confirmation_classes = []
        if not isinstance(confirmation_classes, (list, tuple)):
            raise ConfigurationError('{} confirmation_classes must be a sequence'.format(owner))
        normalized['confirmation_classes'] = [str(item).strip() for item in confirmation_classes if str(item).strip()]
        try:
            normalized['allowed_detection_count'] = max(0, int(normalized.get('allowed_detection_count', 0) or 0))
        except (TypeError, ValueError) as exc:
            raise ConfigurationError('{} allowed_detection_count is invalid: {}'.format(owner, exc))
        try:
            normalized['required_observation_frames'] = max(1, int(normalized.get('required_observation_frames', 1) or 1))
        except (TypeError, ValueError) as exc:
            raise ConfigurationError('{} required_observation_frames is invalid: {}'.format(owner, exc))
    elif execution_type == 'facility_attack':
        normalized.setdefault('attack_mode', 'report_only')
        if normalized['attack_mode'] not in ('report_only', 'detection_confirmed', 'command_confirmed'):
            raise ConfigurationError('{} attack_mode must be report_only, detection_confirmed or command_confirmed'.format(owner))
        confirmation_classes = normalized.get('confirmation_classes', [])
        if confirmation_classes in (None, ''):
            confirmation_classes = []
        if not isinstance(confirmation_classes, (list, tuple)):
            raise ConfigurationError('{} confirmation_classes must be a sequence'.format(owner))
        normalized['confirmation_classes'] = [str(item).strip() for item in confirmation_classes if str(item).strip()]
        required_detection_count = normalized.get('required_detection_count', 1)
        try:
            normalized['required_detection_count'] = max(0, int(required_detection_count))
        except (TypeError, ValueError) as exc:
            raise ConfigurationError('{} required_detection_count is invalid: {}'.format(owner, exc))
        normalized.setdefault('target_facility_id', '')
        normalized.setdefault('target_facility_type', 'enemy_secret_facility')
    elif execution_type == 'finish':
        normalized.setdefault('finish_reason', 'route_complete')
    elif execution_type == 'recon_zone':
        normalized.setdefault('required_position_type', 'frame_region')
    return normalized


@dataclass(frozen=True)
class MissionStep:
    """One executable mission step bound to a waypoint-oriented task.

    Args:
        waypoint: Physical target associated with the step.
        index: Zero-based position within the containing mission plan.
        total: Total number of steps in the containing mission plan.
        task_type: Declarative step kind. Legacy values ``waypoint_capture`` and
            ``waypoint_only`` stay valid while richer task kinds are executed via
            ``execution_type``.
        objective_type: Domain objective bound to the step, such as ``recon`` or
            ``facility_attack``.
        metadata: Optional immutable-style task annotations.
        step_id: Stable task identifier used for explicit graph edges.
        next_step_id: Optional success edge. Empty values fall back to the next
            sequential step.
        failure_step_id: Optional failure edge used after the retry policy gives
            up on the current step.
        outcome_edges: Optional explicit mapping from executor outcomes to target
            ``step_id`` values. This refines ``success`` / ``failure`` into a
            richer task graph without breaking legacy edges.
        retry_limit: Optional per-step retry override.
        quiesce_sec: Optional per-step redispatch backoff override.

    Boundary behavior:
        The default ``task_type`` preserves today's linear waypoint/capture mission
        semantics so legacy routes continue to execute unchanged while task-graph
        profiles may declare richer competition-specific behavior types.
    """

    waypoint: Waypoint
    index: int
    total: int
    task_type: str = LEGACY_TASK_DEFAULT
    objective_type: str = ''
    metadata: Mapping[str, Any] = field(default_factory=dict)
    step_id: str = ''
    next_step_id: str = ''
    failure_step_id: str = ''
    outcome_edges: Mapping[str, str] = field(default_factory=dict)
    retry_limit: Optional[int] = None
    quiesce_sec: Optional[float] = None

    @property
    def zone_name(self) -> str:
        return self.waypoint.name

    @property
    def route_id(self) -> str:
        return self.waypoint.route_id

    @property
    def execution_type(self) -> str:
        return canonical_execution_type(self.task_type)

    @property
    def normalized_objective_type(self) -> str:
        return default_objective_type(self.task_type, self.objective_type)


@dataclass(frozen=True)
class MissionPlan:
    """Immutable mission plan derived from configured executable steps."""

    steps: Tuple[MissionStep, ...]
    step_id_to_index: Mapping[str, int] = field(default_factory=dict)

    @classmethod
    def from_waypoints(cls, waypoints: Iterable[Waypoint]) -> 'MissionPlan':
        """Build the default linear mission plan from waypoint definitions."""
        ordered_waypoints = tuple(waypoints)
        return cls.from_steps(
            MissionStep(
                waypoint=waypoint,
                index=index,
                total=len(ordered_waypoints),
                step_id=waypoint.route_id or 'step_{}'.format(index),
                task_type=LEGACY_TASK_DEFAULT,
            )
            for index, waypoint in enumerate(ordered_waypoints)
        )

    @classmethod
    def from_task_specs(cls, task_specs: Iterable[Mapping[str, Any]], *, dwell_default_sec: float) -> 'MissionPlan':
        """Build a declarative task graph from task specifications.

        Args:
            task_specs: Sequence of task payloads. Each task may define waypoint
                fields directly or under a nested ``waypoint`` mapping.
            dwell_default_sec: Default dwell used when a task omits it.

        Returns:
            MissionPlan containing normalized MissionStep objects.

        Raises:
            ConfigurationError: If task ids are duplicated, references are invalid
                or waypoint payloads are malformed.

        Boundary behavior:
            Missing success edges fall back to the next sequential step so existing
            linear missions may be expressed as task specs without additional graph
            authoring.
        """
        ordered_payloads = list(task_specs or [])
        if not ordered_payloads:
            return cls.from_steps(())
        steps: list[MissionStep] = []
        total = len(ordered_payloads)
        for index, payload in enumerate(ordered_payloads):
            if not isinstance(payload, Mapping):
                raise ConfigurationError('tasks[{}] must be a mapping'.format(index))
            waypoint_payload = payload.get('waypoint', payload)
            if not isinstance(waypoint_payload, Mapping):
                raise ConfigurationError('tasks[{}].waypoint must be a mapping'.format(index))
            waypoint = load_waypoints([waypoint_payload], dwell_default_sec=dwell_default_sec)[0]
            raw_step_id = str(payload.get('step_id', '')).strip() or waypoint.route_id or 'step_{}'.format(index)
            raw_task_type = str(payload.get('task_type', '')).strip() or LEGACY_TASK_DEFAULT
            try:
                canonical_execution_type(raw_task_type)
            except ValueError as exc:
                raise ConfigurationError('tasks[{}] {}'.format(index, exc))
            raw_objective_type = str(payload.get('objective_type', '')).strip()
            raw_next_step_id = str(payload.get('next_step_id', '')).strip()
            raw_failure_step_id = str(payload.get('failure_step_id', '')).strip()
            raw_outcome_edges = payload.get('outcome_edges', {})
            if raw_outcome_edges in (None, ''):
                raw_outcome_edges = {}
            if not isinstance(raw_outcome_edges, Mapping):
                raise ConfigurationError('tasks[{}].outcome_edges must be a mapping'.format(index))
            outcome_edges = {
                str(outcome).strip().lower(): str(target).strip()
                for outcome, target in dict(raw_outcome_edges).items()
                if str(outcome).strip() and str(target).strip()
            }
            raw_retry_limit = payload.get('retry_limit', None)
            if raw_retry_limit in ('', None):
                retry_limit = None
            else:
                try:
                    retry_limit = int(raw_retry_limit)
                except (TypeError, ValueError) as exc:
                    raise ConfigurationError('tasks[{}].retry_limit is invalid: {}'.format(index, exc))
                if retry_limit < 0:
                    raise ConfigurationError('tasks[{}].retry_limit must be >= 0'.format(index))
            raw_quiesce_sec = payload.get('quiesce_sec', None)
            if raw_quiesce_sec in ('', None):
                quiesce_sec = None
            else:
                try:
                    quiesce_sec = float(raw_quiesce_sec)
                except (TypeError, ValueError) as exc:
                    raise ConfigurationError('tasks[{}].quiesce_sec is invalid: {}'.format(index, exc))
                if quiesce_sec < 0.0:
                    raise ConfigurationError('tasks[{}].quiesce_sec must be >= 0'.format(index))
            metadata = _validate_task_metadata(raw_task_type, payload.get('metadata', {}), owner='tasks[{}]'.format(index))
            steps.append(MissionStep(
                waypoint=waypoint,
                index=index,
                total=total,
                task_type=raw_task_type,
                objective_type=default_objective_type(raw_task_type, raw_objective_type),
                metadata=dict(metadata),
                step_id=raw_step_id,
                next_step_id=raw_next_step_id,
                failure_step_id=raw_failure_step_id,
                outcome_edges=outcome_edges,
                retry_limit=retry_limit,
                quiesce_sec=quiesce_sec,
            ))
        return cls.from_steps(steps)

    @classmethod
    def from_steps(cls, steps: Iterable[MissionStep]) -> 'MissionPlan':
        """Build an immutable plan from already-declared mission steps."""
        ordered_steps = tuple(steps)
        normalized_steps = []
        step_ids: list[str] = []
        total = len(ordered_steps)
        for index, step in enumerate(ordered_steps):
            if step.waypoint is None:
                raise ValueError('mission step {} is missing waypoint'.format(index))
            step_id = str(step.step_id or step.route_id or 'step_{}'.format(index)).strip()
            if not step_id:
                raise ValueError('mission step {} is missing step_id'.format(index))
            if step_id in step_ids:
                raise ValueError('mission plan contains duplicate step_id {}'.format(step_id))
            task_type = str(step.task_type).strip() or LEGACY_TASK_DEFAULT
            if task_type not in SUPPORTED_TASK_TYPES:
                raise ValueError('mission step {} task_type {} is unsupported'.format(step_id, task_type))
            retry_limit = None if step.retry_limit is None else int(step.retry_limit)
            if retry_limit is not None and retry_limit < 0:
                raise ValueError('mission step {} retry_limit must be >= 0'.format(step_id))
            quiesce_sec = None if step.quiesce_sec is None else float(step.quiesce_sec)
            if quiesce_sec is not None and quiesce_sec < 0.0:
                raise ValueError('mission step {} quiesce_sec must be >= 0'.format(step_id))
            objective_type = default_objective_type(task_type, step.objective_type)
            metadata = _validate_task_metadata(task_type, step.metadata, owner='mission step {}'.format(step_id))
            step_ids.append(step_id)
            normalized_steps.append(MissionStep(
                waypoint=step.waypoint,
                index=index,
                total=total,
                task_type=task_type,
                objective_type=objective_type,
                metadata=dict(metadata),
                step_id=step_id,
                next_step_id=str(step.next_step_id).strip(),
                failure_step_id=str(step.failure_step_id).strip(),
                outcome_edges={str(key).strip().lower(): str(value).strip() for key, value in dict(step.outcome_edges or {}).items() if str(key).strip() and str(value).strip()},
                retry_limit=retry_limit,
                quiesce_sec=quiesce_sec,
            ))
        step_id_to_index = {step.step_id: step.index for step in normalized_steps}
        validated_steps = []
        for index, step in enumerate(normalized_steps):
            next_step_id = step.next_step_id
            failure_step_id = step.failure_step_id
            if next_step_id and next_step_id not in step_id_to_index:
                raise ValueError('mission step {} references unknown next_step_id {}'.format(step.step_id, next_step_id))
            if failure_step_id and failure_step_id not in step_id_to_index:
                raise ValueError('mission step {} references unknown failure_step_id {}'.format(step.step_id, failure_step_id))
            normalized_outcome_edges = {}
            for outcome, target_step_id in dict(step.outcome_edges or {}).items():
                normalized_outcome = str(outcome).strip().lower()
                normalized_target = str(target_step_id).strip()
                if not normalized_outcome or not normalized_target:
                    continue
                if normalized_target not in step_id_to_index:
                    raise ValueError('mission step {} references unknown outcome target {} for {}'.format(step.step_id, normalized_target, normalized_outcome))
                normalized_outcome_edges[normalized_outcome] = normalized_target
            if not next_step_id and index + 1 < total:
                next_step_id = normalized_steps[index + 1].step_id
            if next_step_id and 'success' not in normalized_outcome_edges:
                normalized_outcome_edges['success'] = next_step_id
            if failure_step_id and 'failure' not in normalized_outcome_edges:
                normalized_outcome_edges['failure'] = failure_step_id
            validated_steps.append(MissionStep(
                waypoint=step.waypoint,
                index=index,
                total=total,
                task_type=step.task_type,
                objective_type=step.objective_type,
                metadata=step.metadata,
                step_id=step.step_id,
                next_step_id=next_step_id,
                failure_step_id=failure_step_id,
                outcome_edges=normalized_outcome_edges,
                retry_limit=step.retry_limit,
                quiesce_sec=step.quiesce_sec,
            ))
        return cls(tuple(validated_steps), dict(step_id_to_index))

    @property
    def waypoints(self) -> Tuple[Waypoint, ...]:
        """Return the waypoint projection of the immutable step list."""
        return tuple(step.waypoint for step in self.steps)

    def __len__(self) -> int:
        return len(self.steps)

    def step_at(self, index: int) -> Optional[MissionStep]:
        if index < 0 or index >= len(self.steps):
            return None
        return self.steps[index]

    def step_by_id(self, step_id: str) -> Optional[MissionStep]:
        index = self.step_id_to_index.get(str(step_id).strip())
        if index is None:
            return None
        return self.steps[index]

    def is_exhausted(self, index: int) -> bool:
        return index >= len(self.steps)

    def as_sequence(self) -> Sequence[Waypoint]:
        return self.waypoints

    def task_types(self) -> Tuple[str, ...]:
        """Return the declarative task types carried by the plan."""
        return tuple(step.task_type for step in self.steps)

    def execution_types(self) -> Tuple[str, ...]:
        """Return canonical execution types consumed by the runtime executor."""
        return tuple(step.execution_type for step in self.steps)

    def next_index_for_outcome(self, current_index: int, outcome: str) -> Optional[int]:
        """Resolve the next step index for one execution outcome.

        Boundary behavior:
            ``outcome_edges`` takes precedence. Missing specialized outcomes fall
            back to ``failure`` then ``success`` so legacy linear plans remain
            valid while richer task graphs may branch on executor-specific causes.
        """
        step = self.step_at(current_index)
        if step is None:
            raise ValueError('current_index {} is outside mission plan'.format(current_index))
        normalized_outcome = str(outcome).strip().lower()
        if not normalized_outcome:
            raise ValueError('outcome must not be empty')
        target_step_id = str(dict(step.outcome_edges or {}).get(normalized_outcome, '')).strip()
        if not target_step_id and normalized_outcome != 'failure':
            target_step_id = str(dict(step.outcome_edges or {}).get('failure', '')).strip() if normalized_outcome.endswith('failure') or normalized_outcome.endswith('timeout') else ''
        if not target_step_id:
            target_step_id = str(dict(step.outcome_edges or {}).get('success', '')).strip() or step.next_step_id
        if not target_step_id and normalized_outcome == 'failure' and step.failure_step_id:
            target_step_id = step.failure_step_id
        if not target_step_id:
            return None
        return self.step_id_to_index[target_step_id]

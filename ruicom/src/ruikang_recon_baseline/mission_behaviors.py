"""Task-behavior helpers for mission execution.

The mission executor owns scheduler state, retries, and publication hooks. This
module owns task-specific completion semantics so richer task types do not keep
expanding imperative branches inside the executor itself.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Dict

from .common import ConfigurationError, ZoneCaptureResult
from .mission_plan import MissionStep

if TYPE_CHECKING:
    from .mission_executor import MissionExecutor


@dataclass(frozen=True)
class BehaviorActionRequest:
    """Task-level request to execute one real behavior backend action.

    Attributes:
        action_type: Canonical downstream command kind.
        payload: Structured command payload forwarded to the backend.
        timeout_sec: Per-command timeout.
        dwell_after_success: Whether mission execution should enter a dwell
            confirmation window after backend success.
    """

    action_type: str
    payload: Dict[str, Any] = field(default_factory=dict)
    timeout_sec: float = 0.0
    dwell_after_success: bool = True


@dataclass(frozen=True)
class NavigationSuccessOutcome:
    """Task-specific reaction to a navigation success event.

    Attributes:
        begin_dwell: Whether the executor should open a capture window.
        result: Optional immediate terminal result when no dwell is required.
        outcome: Optional mission-plan outcome edge used when ``result`` is
            present.
        action_request: Optional backend action to execute before any dwell
            confirmation window opens.
    """

    begin_dwell: bool
    result: ZoneCaptureResult | None = None
    outcome: str = ''
    action_request: BehaviorActionRequest | None = None


@dataclass(frozen=True)
class CaptureFinalizeOutcome:
    """Task-specific finalization of a completed dwell/capture window."""

    result: ZoneCaptureResult
    outcome: str


class MissionStepBehavior:
    """Base behavior contract implemented per canonical execution type."""

    execution_type = ''

    def on_navigation_succeeded(self, executor: 'MissionExecutor', step: MissionStep, now_sec: float) -> NavigationSuccessOutcome:
        raise NotImplementedError

    def finalize_capture(self, executor: 'MissionExecutor', step: MissionStep, base_result: ZoneCaptureResult) -> CaptureFinalizeOutcome:
        return CaptureFinalizeOutcome(result=executor.decorate_result(step, base_result), outcome='success')

    def _requested_action(self, executor: 'MissionExecutor', step: MissionStep, *, action_type: str, payload: Dict[str, Any]) -> BehaviorActionRequest | None:
        backend = getattr(executor, 'behavior_backend', None)
        if backend is None or not getattr(backend, 'enabled', False):
            return None
        return BehaviorActionRequest(
            action_type=action_type,
            payload=dict(payload or {}),
            timeout_sec=float(step.metadata.get('behavior_timeout_sec', executor.behavior_action_timeout_sec) or executor.behavior_action_timeout_sec),
            dwell_after_success=bool(step.metadata.get('behavior_dwell_after_success', True)),
        )


class ReconZoneBehavior(MissionStepBehavior):
    execution_type = 'recon_zone'

    def on_navigation_succeeded(self, executor: 'MissionExecutor', step: MissionStep, now_sec: float) -> NavigationSuccessOutcome:
        _ = executor, step, now_sec
        return NavigationSuccessOutcome(begin_dwell=True)


class TransitBehavior(MissionStepBehavior):
    execution_type = 'transit'

    def on_navigation_succeeded(self, executor: 'MissionExecutor', step: MissionStep, now_sec: float) -> NavigationSuccessOutcome:
        return NavigationSuccessOutcome(
            begin_dwell=False,
            result=executor.build_zero_result(step, status='ok', finished_at=now_sec),
            outcome='success',
        )


class FinishBehavior(MissionStepBehavior):
    execution_type = 'finish'

    def on_navigation_succeeded(self, executor: 'MissionExecutor', step: MissionStep, now_sec: float) -> NavigationSuccessOutcome:
        return NavigationSuccessOutcome(
            begin_dwell=False,
            result=executor.build_zero_result(step, status='ok', finished_at=now_sec),
            outcome='success',
        )


class HazardAvoidBehavior(MissionStepBehavior):
    execution_type = 'hazard_avoid'

    def on_navigation_succeeded(self, executor: 'MissionExecutor', step: MissionStep, now_sec: float) -> NavigationSuccessOutcome:
        action_request = self._requested_action(
            executor,
            step,
            action_type='hazard_avoid',
            payload={
                'hazard_id': str(step.metadata.get('hazard_id', '')).strip() or step.route_id,
                'hazard_type': str(step.metadata.get('hazard_type', 'anti_tank_cone')).strip() or 'anti_tank_cone',
                'avoidance_strategy': str(step.metadata.get('avoidance_strategy', 'navigation_detour')).strip() or 'navigation_detour',
                'hazard_region': str(step.metadata.get('hazard_region', step.waypoint.frame_region)).strip(),
            },
        )
        return NavigationSuccessOutcome(begin_dwell=action_request is None, action_request=action_request)

    def finalize_capture(self, executor: 'MissionExecutor', step: MissionStep, base_result: ZoneCaptureResult) -> CaptureFinalizeOutcome:
        result = executor.decorate_result(step, base_result)
        confirmation_classes = list(step.metadata.get('confirmation_classes', []) or [])
        allowed_detection_count = int(step.metadata.get('allowed_detection_count', 0) or 0)
        required_observation_frames = int(step.metadata.get('required_observation_frames', 1) or 1)
        confirmed_detection_count = sum(int(result.counts.get(name, 0)) for name in confirmation_classes) if confirmation_classes else sum(int(value) for value in result.counts.values())
        observation_frame_count = max(
            int(result.frame_count or 0),
            int((result.evidence_summary or {}).get('accepted_frame_count', 0) or 0),
        )
        backend_feedback = dict(executor.context.active_behavior_feedback or {})
        backend_confirmed = not backend_feedback or str(backend_feedback.get('status', '')).strip().upper() == 'SUCCEEDED'
        confirmed = (
            result.status == 'ok'
            and observation_frame_count >= required_observation_frames
            and confirmed_detection_count <= allowed_detection_count
            and backend_confirmed
        )
        result.hazard_summary = {
            'hazard_id': str(step.metadata.get('hazard_id', '')).strip() or result.route_id,
            'hazard_type': str(step.metadata.get('hazard_type', 'anti_tank_cone')).strip() or 'anti_tank_cone',
            'avoidance_strategy': str(step.metadata.get('avoidance_strategy', 'navigation_detour')).strip() or 'navigation_detour',
            'hazard_region': str(step.metadata.get('hazard_region', result.frame_region)).strip(),
            'confirmation_classes': confirmation_classes,
            'allowed_detection_count': allowed_detection_count,
            'observed_detection_count': confirmed_detection_count,
            'required_observation_frames': required_observation_frames,
            'observed_frame_count': observation_frame_count,
            'backend_action': dict(executor.context.active_behavior_command or {}),
            'backend_feedback': backend_feedback,
            'confirmed': confirmed,
        }
        if confirmed:
            result.status = 'ok'
            result.failure_reason = ''
            result.mission_outcome = 'hazard_avoided'
            return CaptureFinalizeOutcome(result=result, outcome='hazard_cleared')
        if result.status == 'ok':
            result.status = 'hazard_unconfirmed'
            result.failure_reason = result.failure_reason or ('hazard_backend_unconfirmed' if not backend_confirmed else 'hazard_clearance_unconfirmed')
        result.mission_outcome = 'hazard_avoidance_failed'
        return CaptureFinalizeOutcome(result=result, outcome='failure')


class FacilityAttackBehavior(MissionStepBehavior):
    execution_type = 'facility_attack'

    def on_navigation_succeeded(self, executor: 'MissionExecutor', step: MissionStep, now_sec: float) -> NavigationSuccessOutcome:
        attack_mode = str(step.metadata.get('attack_mode', 'report_only')).strip() or 'report_only'
        if attack_mode != 'command_confirmed':
            _ = executor, now_sec
            return NavigationSuccessOutcome(begin_dwell=True)
        action_request = self._requested_action(
            executor,
            step,
            action_type='facility_attack',
            payload={
                'target_facility_id': str(step.metadata.get('target_facility_id', '')).strip() or step.route_id,
                'target_facility_type': str(step.metadata.get('target_facility_type', 'enemy_secret_facility')).strip() or 'enemy_secret_facility',
                'attack_mode': attack_mode,
            },
        )
        if action_request is None:
            raise ConfigurationError('facility_attack command_confirmed requires an enabled behavior action backend')
        return NavigationSuccessOutcome(begin_dwell=False, action_request=action_request)

    def finalize_capture(self, executor: 'MissionExecutor', step: MissionStep, base_result: ZoneCaptureResult) -> CaptureFinalizeOutcome:
        result = executor.decorate_result(step, base_result)
        confirmation_classes = list(step.metadata.get('confirmation_classes', []) or [])
        required_detection_count = int(step.metadata.get('required_detection_count', 1) or 0)
        attack_mode = str(step.metadata.get('attack_mode', 'report_only')).strip() or 'report_only'
        confirmed_detection_count = sum(int(result.counts.get(name, 0)) for name in confirmation_classes) if confirmation_classes else sum(int(value) for value in result.counts.values())
        backend_feedback = dict(executor.context.active_behavior_feedback or {})
        backend_confirmed = not backend_feedback or str(backend_feedback.get('status', '')).strip().upper() == 'SUCCEEDED'
        detections_confirmed = confirmed_detection_count >= required_detection_count
        if attack_mode == 'command_confirmed':
            confirmed = backend_confirmed and detections_confirmed
        else:
            confirmed = detections_confirmed and result.status == 'ok'
        result.action_summary = {
            'attempted': True,
            'confirmed': confirmed,
            'attack_mode': attack_mode,
            'target_facility_id': str(step.metadata.get('target_facility_id', '')).strip() or result.route_id,
            'target_facility_type': str(step.metadata.get('target_facility_type', 'enemy_secret_facility')).strip() or 'enemy_secret_facility',
            'confirmation_classes': confirmation_classes,
            'required_detection_count': required_detection_count,
            'confirmed_detection_count': confirmed_detection_count,
            'backend_action': dict(executor.context.active_behavior_command or {}),
            'backend_feedback': backend_feedback,
        }
        if attack_mode == 'report_only':
            if result.status == 'ok':
                result.failure_reason = ''
                result.mission_outcome = 'facility_action_reported'
                return CaptureFinalizeOutcome(result=result, outcome='action_reported')
            result.mission_outcome = 'facility_action_failed'
            return CaptureFinalizeOutcome(result=result, outcome='failure')
        if confirmed:
            result.status = 'ok'
            result.failure_reason = ''
            result.mission_outcome = 'facility_action_confirmed'
            return CaptureFinalizeOutcome(result=result, outcome='action_confirmed')
        if result.status == 'ok':
            result.status = 'action_unconfirmed'
            result.failure_reason = result.failure_reason or ('facility_backend_unconfirmed' if not backend_confirmed else 'facility_confirmation_missing')
        result.mission_outcome = 'facility_action_failed'
        return CaptureFinalizeOutcome(result=result, outcome='failure')


class MissionBehaviorRegistry:
    """Registry of task behaviors keyed by canonical execution type."""

    def __init__(self):
        self._behaviors: Dict[str, MissionStepBehavior] = {
            behavior.execution_type: behavior
            for behavior in (
                ReconZoneBehavior(),
                TransitBehavior(),
                HazardAvoidBehavior(),
                FacilityAttackBehavior(),
                FinishBehavior(),
            )
        }

    def register(self, behavior: MissionStepBehavior, *, replace: bool = False) -> None:
        """Register one behavior implementation against its execution type."""
        execution_type = str(getattr(behavior, 'execution_type', '')).strip()
        if not execution_type:
            raise ConfigurationError('mission behavior execution_type must not be empty')
        if execution_type in self._behaviors and not replace:
            raise ConfigurationError('mission behavior {} is already registered'.format(execution_type))
        self._behaviors[execution_type] = behavior

    def supported_execution_types(self) -> tuple[str, ...]:
        """Return supported behavior execution types for DSL contract checks."""
        return tuple(sorted(self._behaviors))

    def resolve(self, execution_type: str) -> MissionStepBehavior:
        normalized = str(execution_type or '').strip()
        behavior = self._behaviors.get(normalized)
        if behavior is None:
            raise ConfigurationError('unsupported mission execution_type {}'.format(normalized))
        return behavior

"""Mission execution orchestration independent of ROS transport plumbing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from .common import ZoneCaptureResult
from .mission_context import MissionContext
from .mission_core import CaptureWindowAggregator
from .mission_plan import MissionPlan, MissionStep, canonical_execution_type, default_objective_type
from .navigation_adapters import NavigationAdapterBase
from .recovery_policies import RetryThenFailPolicy


@dataclass(frozen=True)
class MissionRuntimeHooks:
    """Callbacks that bridge executor decisions back into the ROS node."""

    emit_state: Callable[[str, Optional[str], Optional[dict]], None]
    publish_zone_capture: Callable[[ZoneCaptureResult], None]
    publish_current_zone: Callable[[str], None]
    publish_health: Callable[[str, str, dict], None]
    refresh_pose_source: Callable[[], None]
    preflight_ready: Callable[[object, float], bool]
    record_navigation_dispatch: Callable[[object, float], None] | None = None


class MissionExecutor:
    """Drive mission state transitions using a plan, context, and policy objects.

    The executor owns the mission progression logic while the node owns ROS-specific
    transport, parameter loading, and artifact publication.
    """

    def __init__(
        self,
        *,
        plan: MissionPlan,
        context: MissionContext,
        clock,
        nav_adapter: NavigationAdapterBase,
        capture_aggregator: CaptureWindowAggregator,
        recovery_policy: RetryThenFailPolicy,
        hooks: MissionRuntimeHooks,
        mission_timeout_sec: float,
        preflight_timeout_sec: float,
        class_names,
        class_schema_hash: str,
    ):
        self.plan = plan
        self.context = context
        self.clock = clock
        self.nav_adapter = nav_adapter
        self.capture_aggregator = capture_aggregator
        self.recovery_policy = recovery_policy
        self.hooks = hooks
        self.mission_timeout_sec = float(mission_timeout_sec)
        self.preflight_timeout_sec = float(preflight_timeout_sec)
        self.class_names = tuple(class_names)
        self.class_schema_hash = class_schema_hash

    def current_step(self) -> Optional[MissionStep]:
        return self.plan.step_at(self.context.route_index)

    def current_waypoint(self):
        step = self.current_step()
        return None if step is None else step.waypoint

    def _bind_active_step(self, step: Optional[MissionStep]) -> None:
        if step is None:
            self.context.current_step_id = ''
            self.context.current_task_type = ''
            self.context.current_task_objective = ''
            return
        self.context.current_step_id = str(step.step_id)
        self.context.current_task_type = str(step.task_type)
        self.context.current_task_objective = str(step.normalized_objective_type)

    def _task_details(self, step: MissionStep) -> dict:
        return {
            'step_id': step.step_id,
            'task_type': step.task_type,
            'execution_type': step.execution_type,
            'objective_type': step.normalized_objective_type,
            'task_metadata': dict(step.metadata or {}),
        }

    def _zero_result(self, step: MissionStep, *, status: str, finished_at: float, failure_reason: str = '') -> ZoneCaptureResult:
        waypoint = step.waypoint
        execution_type = step.execution_type
        objective_type = step.normalized_objective_type
        evidence_summary = {
            'accepted_frame_count': 0,
            'evidence_sources': {},
            'position_observation_count': 0,
            'position_labels': {},
            'class_position_labels': {},
        }
        hazard_summary = {}
        action_summary = {}
        mission_outcome = status
        if execution_type == 'transit':
            mission_outcome = 'transit_reached' if status == 'ok' else status
        elif execution_type == 'finish':
            mission_outcome = 'finish_reached' if status == 'ok' else status
        elif execution_type == 'hazard_avoid':
            hazard_summary = {
                'hazard_id': str(step.metadata.get('hazard_id', '')).strip() or waypoint.route_id,
                'hazard_type': str(step.metadata.get('hazard_type', 'anti_tank_cone')).strip() or 'anti_tank_cone',
                'avoidance_strategy': str(step.metadata.get('avoidance_strategy', 'navigation_detour')).strip() or 'navigation_detour',
                'hazard_region': str(step.metadata.get('hazard_region', waypoint.frame_region)).strip(),
                'confirmed': status == 'ok',
            }
            mission_outcome = 'hazard_avoided' if status == 'ok' else status
        elif execution_type == 'facility_attack':
            action_summary = {
                'attempted': True,
                'confirmed': False,
                'attack_mode': str(step.metadata.get('attack_mode', 'report_only')).strip() or 'report_only',
                'target_facility_id': str(step.metadata.get('target_facility_id', '')).strip() or waypoint.route_id,
                'target_facility_type': str(step.metadata.get('target_facility_type', 'enemy_secret_facility')).strip() or 'enemy_secret_facility',
                'confirmation_classes': list(step.metadata.get('confirmation_classes', []) or []),
                'required_detection_count': int(step.metadata.get('required_detection_count', 1) or 1),
                'confirmed_detection_count': 0,
            }
            mission_outcome = 'facility_action_failed' if status != 'ok' else 'facility_action_attempted'
        return ZoneCaptureResult(
            zone_name=waypoint.name,
            status=status,
            class_names=list(self.class_names),
            class_counts=[0 for _ in self.class_names],
            capture_started_at=self.context.dispatch_started_at,
            capture_finished_at=finished_at,
            frame_count=0,
            frame_region=waypoint.frame_region,
            failure_reason=failure_reason,
            route_id=waypoint.route_id,
            class_schema_hash=self.class_schema_hash,
            task_type=step.task_type,
            objective_type=objective_type,
            mission_outcome=mission_outcome,
            task_metadata=dict(step.metadata or {}),
            evidence_summary=evidence_summary,
            hazard_summary=hazard_summary,
            action_summary=action_summary,
        )

    def _decorate_result(self, step: MissionStep, result: ZoneCaptureResult) -> ZoneCaptureResult:
        result.task_type = step.task_type
        result.objective_type = step.normalized_objective_type
        result.task_metadata = dict(step.metadata or {})
        if step.execution_type == 'recon_zone' and not result.mission_outcome:
            result.mission_outcome = 'recon_completed' if result.status == 'ok' else result.status
        return result

    def _finalize_facility_attack(self, step: MissionStep, result: ZoneCaptureResult) -> tuple[ZoneCaptureResult, str]:
        result = self._decorate_result(step, result)
        confirmation_classes = list(step.metadata.get('confirmation_classes', []) or [])
        required_detection_count = int(step.metadata.get('required_detection_count', 1) or 1)
        if confirmation_classes:
            confirmed_detection_count = sum(int(result.counts.get(name, 0)) for name in confirmation_classes)
        else:
            confirmed_detection_count = sum(int(value) for value in result.counts.values())
        confirmed = confirmed_detection_count >= required_detection_count and result.status == 'ok'
        result.action_summary = {
            'attempted': True,
            'confirmed': confirmed,
            'attack_mode': str(step.metadata.get('attack_mode', 'report_only')).strip() or 'report_only',
            'target_facility_id': str(step.metadata.get('target_facility_id', '')).strip() or result.route_id,
            'target_facility_type': str(step.metadata.get('target_facility_type', 'enemy_secret_facility')).strip() or 'enemy_secret_facility',
            'confirmation_classes': confirmation_classes,
            'required_detection_count': required_detection_count,
            'confirmed_detection_count': confirmed_detection_count,
        }
        if confirmed:
            result.status = 'ok'
            result.failure_reason = ''
            result.mission_outcome = 'facility_action_confirmed'
            return result, 'action_confirmed'
        if result.status == 'ok':
            result.status = 'action_unconfirmed'
            result.failure_reason = result.failure_reason or 'facility_confirmation_missing'
        result.mission_outcome = 'facility_action_failed'
        return result, 'failure'

    def _advance_index(self, current_index: int, outcome: str) -> Optional[int]:
        return self.plan.next_index_for_outcome(current_index, outcome)

    def advance_after_result(self, result: ZoneCaptureResult, *, outcome: str) -> None:
        dynamic_payload = result.to_dict()
        route_key = result.route_id or result.zone_name
        self.context.zone_results_dynamic[route_key] = dynamic_payload
        self.hooks.publish_zone_capture(result)
        next_index = self._advance_index(self.context.route_index, outcome)
        self.context.route_index = len(self.plan) if next_index is None else int(next_index)
        self.context.retry_count = 0
        self.context.dispatch_quiesce_until = 0.0
        self.context.current_zone = ''
        self.context.current_route_id = ''
        self._bind_active_step(None)
        self.hooks.publish_current_zone('')
        if next_index is None or self.plan.is_exhausted(self.context.route_index):
            duration = self.clock.now_business_sec() - self.context.mission_started_at
            self.hooks.emit_state('mission_finished', 'FINISHED', {'duration_sec': duration, 'result_outcome': result.mission_outcome, 'result_status': result.status})
            return
        next_step = self.current_step()
        assert next_step is not None
        self._bind_active_step(next_step)
        self.context.preflight_deadline = self.clock.now_business_sec() + self.preflight_timeout_sec
        self.hooks.emit_state('goto_next', 'DISPATCH_PENDING', {
            'next_zone': next_step.zone_name,
            'next_route_id': next_step.route_id,
            'next_step_id': next_step.step_id,
            'next_task_type': next_step.task_type,
            'next_execution_type': next_step.execution_type,
            'next_objective_type': next_step.normalized_objective_type,
        })

    def start(self) -> None:
        now_sec = self.clock.now_business_sec()
        self.context.route_index = 0
        self.context.retry_count = 0
        self.context.zone_results_dynamic = {}
        self.context.mission_started_at = now_sec
        self.context.current_zone = ''
        self.context.current_route_id = ''
        self.context.schema_blocked_reason = ''
        self.context.dispatch_quiesce_until = 0.0
        self.context.preflight_deadline = now_sec + self.preflight_timeout_sec
        self._bind_active_step(self.current_step())
        self.hooks.emit_state('mission_started', 'DISPATCH_PENDING', None)

    def dispatch_waypoint(self, step: MissionStep) -> None:
        waypoint = step.waypoint
        self.context.current_zone = waypoint.name
        self.context.current_route_id = waypoint.route_id
        self._bind_active_step(step)
        self.hooks.publish_current_zone(waypoint.name)
        self.context.dispatch_started_at = self.clock.now_business_sec()
        if callable(self.hooks.record_navigation_dispatch):
            self.hooks.record_navigation_dispatch(waypoint, self.context.dispatch_started_at)
        self.nav_adapter.dispatch(waypoint)
        details = {
            'goal': waypoint.__dict__.copy(),
            'retry_count': self.context.retry_count,
            'current_route_id': waypoint.route_id,
            **self._task_details(step),
        }
        self.hooks.emit_state('goal_dispatched', 'DISPATCHED', details)

    def begin_dwell(self, step: MissionStep) -> None:
        waypoint = step.waypoint
        now_sec = self.clock.now_business_sec()
        self.capture_aggregator.start(waypoint.name, now_sec, frame_region=waypoint.frame_region, route_id=waypoint.route_id)
        self.context.current_capture_deadline = now_sec + waypoint.dwell_sec
        self.hooks.emit_state('capture_window_started', 'DWELL', {
            'capture_deadline': self.context.current_capture_deadline,
            'frame_region': waypoint.frame_region,
            'current_route_id': waypoint.route_id,
            **self._task_details(step),
        })

    def handle_navigation_failure(self, step: MissionStep, reason: str) -> None:
        now_sec = self.clock.now_business_sec()
        decision = self.recovery_policy.on_navigation_failure(
            context=self.context,
            adapter=self.nav_adapter,
            reason=reason,
            now_sec=now_sec,
            step=step,
        )
        if decision.action == 'retry':
            self.context.retry_count = int(decision.details['retry_count'])
            self.context.dispatch_quiesce_until = float(decision.quiesce_until)
            self.context.preflight_deadline = now_sec + self.preflight_timeout_sec
            details = dict(decision.details)
            details['current_route_id'] = step.route_id
            details.update(self._task_details(step))
            self.hooks.emit_state('navigation_retry', 'DISPATCH_PENDING', details)
            return
        result = self._zero_result(step, status='navigation_failure', finished_at=now_sec, failure_reason=reason)
        outcome = 'navigation_timeout' if 'timeout' in str(reason).lower() else 'failure'
        self.advance_after_result(result, outcome=outcome)

    def poll_navigation(self, step: MissionStep, now_sec: float) -> None:
        waypoint = step.waypoint
        self.hooks.refresh_pose_source()
        status = self.nav_adapter.poll(now_sec)
        dispatch_elapsed = now_sec - self.context.dispatch_started_at
        if dispatch_elapsed > waypoint.timeout_sec:
            cancel_sent = self.nav_adapter.cancel(now_sec)
            timeout_reason = 'navigation_timeout' if cancel_sent else 'navigation_timeout_no_cancel_ack'
            self.handle_navigation_failure(step, timeout_reason)
            return
        if status == 'SUCCEEDED':
            if step.execution_type in ('transit', 'hazard_avoid', 'finish'):
                result = self._zero_result(step, status='ok', finished_at=now_sec)
                self.advance_after_result(result, outcome='success')
                return
            self.begin_dwell(step)
            return
        if status in ('ABORTED', 'PREEMPTED'):
            self.handle_navigation_failure(step, status.lower())
            return
        if status in ('DISPATCHED', 'PENDING', 'ACTIVE'):
            new_state = 'ACTIVE' if status == 'ACTIVE' else 'DISPATCHED'
            if self.context.state != new_state:
                details = {'elapsed_sec': dispatch_elapsed}
                details.update(self._task_details(step))
                self.hooks.emit_state('navigation_{}'.format(status.lower()), new_state, details)
            return
        self.hooks.publish_health('warn', 'navigation_unknown_status', {'status': status, **self._task_details(step)})

    def poll_dwell(self, step: MissionStep, now_sec: float) -> None:
        waypoint = step.waypoint
        if self.context.latest_detection_frame is None:
            self.hooks.publish_health('warn', 'waiting_for_detections', {'zone': waypoint.name, **self._task_details(step)})
        if now_sec < self.context.current_capture_deadline:
            return
        base_result = self.capture_aggregator.finalize(status='ok')
        if step.execution_type == 'facility_attack':
            result, outcome = self._finalize_facility_attack(step, base_result)
            self.advance_after_result(result, outcome=outcome)
            return
        result = self._decorate_result(step, base_result)
        self.advance_after_result(result, outcome='success')

    def step(self) -> None:
        """Advance the mission state machine by one scheduler tick.

        Args:
            None.

        Returns:
            None.

        Raises:
            No explicit exception is raised. All operator-visible faults are pushed
            through health/state publication instead.

        Boundary behavior:
            When a quiesce interval is active after a navigation retry, dispatch is
            deferred until the interval expires instead of spinning a hot loop.
        """
        now_sec = self.clock.now_business_sec()
        if self.context.mission_started_at and (now_sec - self.context.mission_started_at) > self.mission_timeout_sec:
            step = self.current_step()
            if step is None:
                self.hooks.emit_state('mission_timeout', 'TIMEOUT', {'duration_sec': now_sec - self.context.mission_started_at})
                return
            result = self._zero_result(step, status='mission_timeout', finished_at=now_sec, failure_reason='mission_timeout')
            self.advance_after_result(result, outcome='failure')
            return
        step = self.current_step()
        if step is None:
            return
        self._bind_active_step(step)
        if self.context.state == 'DISPATCH_PENDING':
            if now_sec < self.context.dispatch_quiesce_until:
                return
            if not self.hooks.preflight_ready(step.waypoint, now_sec):
                return
            self.dispatch_waypoint(step)
            return
        if self.context.state in ('DISPATCHED', 'ACTIVE'):
            self.poll_navigation(step, now_sec)
            return
        if self.context.state == 'DWELL':
            self.poll_dwell(step, now_sec)
            return

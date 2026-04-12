"""Navigation execution façade used by the mission executor.

The mission executor should not need to know how dispatch timestamps, status
sources, or cancel acknowledgements are tracked. This façade centralizes that
runtime bookkeeping while still delegating actual transport work to a concrete
navigation adapter.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from .mission_plan import MissionStep
from .navigation_adapters.base import NavigationAdapterBase


@dataclass(frozen=True)
class NavigationDispatchRecord:
    """One navigation dispatch event."""

    route_id: str
    zone_name: str
    step_id: str
    task_type: str
    objective_type: str
    dispatched_at: float
    adapter_status_source: str


@dataclass(frozen=True)
class NavigationExecutionStatus:
    """Canonical status snapshot produced by :class:`NavigationRuntimeExecutive`."""

    status: str
    route_id: str
    zone_name: str
    step_id: str
    task_type: str
    objective_type: str
    dispatched_at: float
    elapsed_sec: float
    adapter_status_source: str
    cancel_supported: bool
    cancel_has_ack: bool
    details: Dict[str, Any] = field(default_factory=dict)


class NavigationRuntimeExecutive:
    """Runtime façade around one navigation adapter.

    Args:
        adapter: Concrete navigation adapter implementation.
        clock: Mission/business clock.

    Boundary behavior:
        The executive is stateless until the first dispatch. Polling before any
        dispatch returns ``IDLE`` with empty route metadata.
    """

    def __init__(self, *, adapter: NavigationAdapterBase, clock):
        self.adapter = adapter
        self.clock = clock
        self._active_record: Optional[NavigationDispatchRecord] = None
        self._last_status = 'IDLE'
        self._last_status_stamp = 0.0
        self._dispatch_count = 0
        self._cancel_count = 0

    def dispatch(self, step: MissionStep, now_sec: Optional[float] = None) -> NavigationDispatchRecord:
        issued_at = float(now_sec if now_sec is not None else self.clock.now_business_sec())
        self.adapter.dispatch(step.waypoint)
        self._dispatch_count += 1
        self._active_record = NavigationDispatchRecord(
            route_id=step.route_id,
            zone_name=step.zone_name,
            step_id=step.step_id,
            task_type=step.task_type,
            objective_type=step.normalized_objective_type,
            dispatched_at=issued_at,
            adapter_status_source=self.adapter.status_source,
        )
        self._last_status = 'DISPATCHED'
        self._last_status_stamp = issued_at
        return self._active_record

    def poll(self, now_sec: Optional[float] = None) -> NavigationExecutionStatus:
        current_sec = float(now_sec if now_sec is not None else self.clock.now_business_sec())
        record = self._active_record
        if record is None:
            return NavigationExecutionStatus(
                status='IDLE',
                route_id='',
                zone_name='',
                step_id='',
                task_type='',
                objective_type='',
                dispatched_at=0.0,
                elapsed_sec=0.0,
                adapter_status_source=self.adapter.status_source,
                cancel_supported=bool(self.adapter.supports_cancel),
                cancel_has_ack=bool(self.adapter.cancel_has_ack),
                details={'dispatch_count': int(self._dispatch_count), 'cancel_count': int(self._cancel_count)},
            )
        status = str(self.adapter.poll(current_sec)).strip().upper() or 'IDLE'
        if status == 'PENDING':
            status = 'DISPATCHED'
        elapsed = max(0.0, current_sec - float(record.dispatched_at))
        self._last_status = status
        self._last_status_stamp = current_sec
        return NavigationExecutionStatus(
            status=status,
            route_id=record.route_id,
            zone_name=record.zone_name,
            step_id=record.step_id,
            task_type=record.task_type,
            objective_type=record.objective_type,
            dispatched_at=record.dispatched_at,
            elapsed_sec=elapsed,
            adapter_status_source=record.adapter_status_source,
            cancel_supported=bool(self.adapter.supports_cancel),
            cancel_has_ack=bool(self.adapter.cancel_has_ack),
            details={
                'dispatch_count': int(self._dispatch_count),
                'cancel_count': int(self._cancel_count),
                'last_status': self._last_status,
                'last_status_stamp': float(self._last_status_stamp),
            },
        )

    def cancel(self, now_sec: Optional[float] = None) -> bool:
        current_sec = float(now_sec if now_sec is not None else self.clock.now_business_sec())
        cancelled = bool(self.adapter.cancel(current_sec))
        if cancelled:
            self._cancel_count += 1
            self._last_status = 'PREEMPTED'
            self._last_status_stamp = current_sec
        return cancelled

    def runtime_contract(self) -> Dict[str, Any]:
        return {
            'status_source': self.adapter.status_source,
            'supports_cancel': bool(self.adapter.supports_cancel),
            'cancel_has_ack': bool(self.adapter.cancel_has_ack),
            'dispatch_count': int(self._dispatch_count),
            'cancel_count': int(self._cancel_count),
            'last_status': self._last_status,
            'last_status_stamp': float(self._last_status_stamp),
        }

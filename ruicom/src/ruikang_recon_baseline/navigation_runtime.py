"""Runtime navigation-role composition helpers.

This module turns navigation execution from a single opaque adapter instance into
an explicit runtime binding of dispatcher, status monitor and canceller roles.
The executor still consumes a single ``NavigationAdapterBase`` surface for
backward compatibility, but that surface is now composed from explicit runtime
roles rather than implicitly assuming one monolithic backend object.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

from .common import Waypoint


class NavigationGoalDispatcher:
    """Dispatch one mission waypoint into a navigation backend."""

    role_name = 'goal_dispatcher'

    def dispatch(self, waypoint: Waypoint) -> None:
        raise NotImplementedError

    def close(self) -> None:
        return None


class NavigationStatusMonitor:
    """Observe backend progress and map it onto canonical mission statuses."""

    role_name = 'status_monitor'
    status_source = 'internal'

    def poll(self, now_sec: float) -> str:
        raise NotImplementedError

    def close(self) -> None:
        return None


class NavigationCanceller:
    """Cancel an active goal, optionally with explicit backend acknowledgement."""

    role_name = 'goal_canceller'
    supports_cancel = False
    cancel_has_ack = False

    def cancel(self, now_sec: Optional[float] = None) -> bool:
        _ = now_sec
        return False

    def close(self) -> None:
        return None


@dataclass(frozen=True)
class NavigationRuntimeBinding:
    """Explicit runtime composition of navigation roles.

    The fields intentionally mirror the split introduced in the architectural
    plan: planner/controller dispatch happens through the dispatcher surface,
    backend progression comes from the status monitor, and cancellation is owned
    by the canceller surface.
    """

    dispatcher: NavigationGoalDispatcher
    status_monitor: NavigationStatusMonitor
    canceller: NavigationCanceller

    def close(self) -> None:
        """Close distinct role instances without double-closing shared objects."""
        seen = set()
        for component in (self.dispatcher, self.status_monitor, self.canceller):
            marker = id(component)
            if marker in seen:
                continue
            seen.add(marker)
            close = getattr(component, 'close', None)
            if callable(close):
                close()

    def summary(self) -> dict:
        """Return a JSON-serializable description of the runtime role binding."""
        return {
            'dispatcher_role': type(self.dispatcher).__name__,
            'status_monitor_role': type(self.status_monitor).__name__,
            'canceller_role': type(self.canceller).__name__,
            'status_source': getattr(self.status_monitor, 'status_source', 'internal'),
            'supports_cancel': bool(getattr(self.canceller, 'supports_cancel', False)),
            'cancel_has_ack': bool(getattr(self.canceller, 'cancel_has_ack', False)),
        }


class CompositeNavigationAdapter:
    """Compatibility adapter composed from explicit runtime roles."""

    def __init__(self, binding: NavigationRuntimeBinding):
        self.binding = binding

    @property
    def supports_cancel(self) -> bool:
        return bool(getattr(self.binding.canceller, 'supports_cancel', False))

    @property
    def cancel_has_ack(self) -> bool:
        return bool(getattr(self.binding.canceller, 'cancel_has_ack', False))

    @property
    def status_source(self) -> str:
        return str(getattr(self.binding.status_monitor, 'status_source', 'internal'))

    def dispatch(self, waypoint: Waypoint) -> None:
        self.binding.dispatcher.dispatch(waypoint)

    def poll(self, now_sec: float) -> str:
        return self.binding.status_monitor.poll(now_sec)

    def cancel(self, now_sec: Optional[float] = None) -> bool:
        return bool(self.binding.canceller.cancel(now_sec))

    def close(self) -> None:
        self.binding.close()

    def runtime_binding_summary(self) -> dict:
        return self.binding.summary()


class NoOpCanceller(NavigationCanceller):
    """Canceller surface for backends without explicit cancel support."""

    role_name = 'goal_canceller'
    supports_cancel = False
    cancel_has_ack = False

    def __init__(self, on_cancel=None):
        self._on_cancel = on_cancel

    def cancel(self, now_sec: Optional[float] = None) -> bool:
        if callable(self._on_cancel):
            self._on_cancel(now_sec)
        return False

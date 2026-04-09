"""Stable navigation adapter interfaces used by mission execution."""

from __future__ import annotations

from typing import Optional

from ..common import Waypoint


class NavigationAdapterBase:
    """Base class for ROS navigation adapters.

    Implementations are responsible for dispatching waypoints into the concrete
    navigation backend and translating backend status into the canonical mission
    status vocabulary.
    """

    @property
    def supports_cancel(self) -> bool:
        """Whether the adapter supports active cancellation."""
        return True

    @property
    def cancel_has_ack(self) -> bool:
        """Whether cancellation is externally acknowledged by the backend."""
        return False

    @property
    def status_source(self) -> str:
        """Return the provenance of navigation status values."""
        return 'internal'

    def dispatch(self, waypoint: Waypoint) -> None:
        """Dispatch one waypoint to the backend."""
        raise NotImplementedError

    def poll(self, now_sec: float) -> str:
        """Return a canonical navigation status for the active waypoint."""
        raise NotImplementedError

    def cancel(self, now_sec: Optional[float] = None) -> bool:
        """Cancel the active goal and return whether an explicit cancel was sent."""
        raise NotImplementedError

    def close(self) -> None:
        """Release backend resources. Default implementations are no-ops."""
        return None

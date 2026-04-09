"""Mission recovery policies for dispatch and navigation failures."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

from .mission_context import MissionContext
from .mission_plan import MissionStep
from .navigation_adapters import NavigationAdapterBase


@dataclass(frozen=True)
class RecoveryDecision:
    """Result produced by a recovery policy."""

    action: str
    details: Dict[str, object] = field(default_factory=dict)
    quiesce_until: float = 0.0


class RetryThenFailPolicy:
    """Retry a waypoint a bounded number of times before failing it.

    Args:
        retry_limit: Maximum number of retries before terminal failure.
        navigation_failure_quiesce_sec: Delay before redispatch when the adapter
            cannot acknowledge cancel.
    """

    def __init__(self, retry_limit: int, navigation_failure_quiesce_sec: float):
        self.retry_limit = int(retry_limit)
        self.navigation_failure_quiesce_sec = float(navigation_failure_quiesce_sec)

    def _effective_retry_limit(self, step: Optional[MissionStep]) -> int:
        if step is None or step.retry_limit is None:
            return self.retry_limit
        return int(step.retry_limit)

    def _effective_quiesce_sec(self, step: Optional[MissionStep]) -> float:
        if step is None or step.quiesce_sec is None:
            return self.navigation_failure_quiesce_sec
        return float(step.quiesce_sec)

    def on_navigation_failure(
        self,
        *,
        context: MissionContext,
        adapter: NavigationAdapterBase,
        reason: str,
        now_sec: float,
        step: Optional[MissionStep] = None,
    ) -> RecoveryDecision:
        """Decide whether to retry or terminally fail the active waypoint.

        Args:
            context: Mission runtime context.
            adapter: Active navigation adapter.
            reason: Canonical navigation failure reason.
            now_sec: Current business clock time.
            step: Optional active mission step carrying retry/backoff overrides.

        Returns:
            RecoveryDecision describing retry or fail.

        Raises:
            No explicit exception is raised.

        Boundary behavior:
            Per-step retry and quiesce overrides supersede global defaults without
            mutating the baseline profile configuration.
        """
        retry_limit = self._effective_retry_limit(step)
        quiesce_sec = self._effective_quiesce_sec(step)
        if context.retry_count < retry_limit:
            next_retry_count = context.retry_count + 1
            quiesce_until = 0.0
            if not adapter.supports_cancel or not adapter.cancel_has_ack:
                quiesce_until = now_sec + quiesce_sec
            return RecoveryDecision(
                action='retry',
                quiesce_until=quiesce_until,
                details={
                    'reason': reason,
                    'retry_count': next_retry_count,
                    'retry_limit': retry_limit,
                    'adapter_supports_cancel': adapter.supports_cancel,
                    'adapter_cancel_has_ack': adapter.cancel_has_ack,
                    'adapter_status_source': adapter.status_source,
                    'dispatch_quiesce_until': quiesce_until,
                    'step_id': step.step_id if step is not None else '',
                },
            )
        return RecoveryDecision(action='fail', details={'reason': reason, 'retry_limit': retry_limit, 'step_id': step.step_id if step is not None else ''})

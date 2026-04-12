"""Pure helpers for deriving explicit vendor execution feedback.

This module exists to keep the pre-bridge execution-feedback contract testable
without a live ROS graph. The vendor feedback adapter is intentionally strict:
only an explicit, fresh vendor signal may assert execution readiness, and the
signal can optionally be gated on recent upstream command traffic so a stale
latched ``true`` does not masquerade as an actively executing chassis.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VendorFeedbackSnapshot:
    """Current vendor feedback inputs used to derive one normalized heartbeat.

    Attributes:
        now_sec: Current monotonic or ROS time in seconds.
        source_enabled: Whether an explicit vendor feedback topic is configured.
        source_value: Last observed explicit vendor feedback value.
        source_stamp_sec: Timestamp of the last explicit vendor feedback sample.
        source_timeout_sec: Freshness timeout for the explicit vendor feedback.
        require_recent_command: Whether command recency is required in addition
            to the explicit vendor feedback signal.
        command_stamp_sec: Timestamp of the last upstream command sample.
        command_timeout_sec: Freshness timeout for command recency.
    """

    now_sec: float
    source_enabled: bool
    source_value: bool
    source_stamp_sec: float
    source_timeout_sec: float
    require_recent_command: bool
    command_stamp_sec: float
    command_timeout_sec: float


@dataclass(frozen=True)
class VendorFeedbackDecision:
    """Derived explicit vendor feedback state.

    Attributes:
        output_feedback: Normalized bool published toward the platform bridge.
        source_fresh: Whether the explicit feedback source is fresh enough.
        command_recent: Whether recent command traffic satisfies the configured
            freshness gate.
        reason: Machine-readable decision reason for diagnostics.
    """

    output_feedback: bool
    source_fresh: bool
    command_recent: bool
    reason: str



def _is_fresh(now_sec: float, stamp_sec: float, timeout_sec: float) -> bool:
    if timeout_sec <= 0.0 or stamp_sec <= 0.0:
        return False
    return (float(now_sec) - float(stamp_sec)) <= float(timeout_sec)



def evaluate_vendor_feedback(snapshot: VendorFeedbackSnapshot) -> VendorFeedbackDecision:
    """Derive one normalized explicit vendor feedback decision.

    Args:
        snapshot: Current input values and freshness thresholds.

    Returns:
        VendorFeedbackDecision describing the normalized heartbeat state.

    Raises:
        ValueError: If one configured timeout is non-positive.

    Boundary behavior:
        - Missing or stale explicit feedback always forces ``False``.
        - A fresh explicit ``False`` also forces ``False``.
        - When ``require_recent_command`` is enabled, a fresh explicit ``True``
          is still held low until recent upstream command traffic is observed.
    """
    if float(snapshot.source_timeout_sec) <= 0.0:
        raise ValueError('source_timeout_sec must be > 0')
    if bool(snapshot.require_recent_command) and float(snapshot.command_timeout_sec) <= 0.0:
        raise ValueError('command_timeout_sec must be > 0 when require_recent_command is enabled')
    if not bool(snapshot.source_enabled):
        return VendorFeedbackDecision(
            output_feedback=False,
            source_fresh=False,
            command_recent=not bool(snapshot.require_recent_command),
            reason='source_disabled',
        )
    source_fresh = _is_fresh(snapshot.now_sec, snapshot.source_stamp_sec, snapshot.source_timeout_sec)
    command_recent = (not bool(snapshot.require_recent_command)) or _is_fresh(
        snapshot.now_sec,
        snapshot.command_stamp_sec,
        snapshot.command_timeout_sec,
    )
    if not source_fresh:
        return VendorFeedbackDecision(
            output_feedback=False,
            source_fresh=False,
            command_recent=command_recent,
            reason='source_stale',
        )
    if not bool(snapshot.source_value):
        return VendorFeedbackDecision(
            output_feedback=False,
            source_fresh=True,
            command_recent=command_recent,
            reason='source_false',
        )
    if not command_recent:
        return VendorFeedbackDecision(
            output_feedback=False,
            source_fresh=True,
            command_recent=False,
            reason='command_stale',
        )
    return VendorFeedbackDecision(
        output_feedback=True,
        source_fresh=True,
        command_recent=True,
        reason='explicit_feedback_confirmed',
    )

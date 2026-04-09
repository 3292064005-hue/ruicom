"""Pure runtime logic for bridging vendor platform signals into normalized recon topics."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PlatformBridgeSnapshot:
    """Current bridge inputs used to derive normalized platform feedback and command-path state."""

    now_sec: float
    feedback_timeout_sec: float
    explicit_feedback_enabled: bool
    explicit_feedback_value: bool
    explicit_feedback_stamp_sec: float
    odom_feedback_enabled: bool
    odom_stamp_sec: float
    command_bridge_enabled: bool
    upstream_command_stamp_sec: float
    allow_odom_as_feedback: bool = False


@dataclass(frozen=True)
class PlatformBridgeDecision:
    """Derived normalized bridge output state."""

    output_feedback: bool
    source: str
    explicit_feedback_fresh: bool
    odom_feedback_fresh: bool
    command_bridge_fresh: bool
    execution_feedback_fresh: bool



def _is_fresh(now_sec: float, stamp_sec: float, timeout_sec: float) -> bool:
    if timeout_sec <= 0.0 or stamp_sec <= 0.0:
        return False
    return (now_sec - stamp_sec) <= timeout_sec



def evaluate_platform_bridge(snapshot: PlatformBridgeSnapshot) -> PlatformBridgeDecision:
    """Derive normalized base-feedback state from vendor inputs.

    Boundary behavior:
        - When explicit feedback is configured and fresh, its bool value wins.
        - Otherwise, odom freshness is treated as observability only unless the
          active platform contract explicitly allows odom fallback.
        - Command-path freshness is tracked independently so deploy contracts can
          distinguish “feedback alive” from “nav output is really bridged”.
    """

    if float(snapshot.feedback_timeout_sec) <= 0.0:
        raise ValueError('feedback_timeout_sec must be > 0')
    explicit_fresh = snapshot.explicit_feedback_enabled and _is_fresh(
        snapshot.now_sec,
        snapshot.explicit_feedback_stamp_sec,
        snapshot.feedback_timeout_sec,
    )
    odom_fresh = snapshot.odom_feedback_enabled and _is_fresh(
        snapshot.now_sec,
        snapshot.odom_stamp_sec,
        snapshot.feedback_timeout_sec,
    )
    command_bridge_fresh = snapshot.command_bridge_enabled and _is_fresh(
        snapshot.now_sec,
        snapshot.upstream_command_stamp_sec,
        snapshot.feedback_timeout_sec,
    )
    if explicit_fresh:
        return PlatformBridgeDecision(
            output_feedback=bool(snapshot.explicit_feedback_value),
            source='explicit_feedback',
            explicit_feedback_fresh=True,
            odom_feedback_fresh=odom_fresh,
            command_bridge_fresh=command_bridge_fresh,
            execution_feedback_fresh=bool(snapshot.explicit_feedback_value),
        )
    if odom_fresh and bool(snapshot.allow_odom_as_feedback):
        return PlatformBridgeDecision(
            output_feedback=True,
            source='odom_feedback_fallback',
            explicit_feedback_fresh=False,
            odom_feedback_fresh=True,
            command_bridge_fresh=command_bridge_fresh,
            execution_feedback_fresh=True,
        )
    if odom_fresh:
        return PlatformBridgeDecision(
            output_feedback=False,
            source='odom_observability_only',
            explicit_feedback_fresh=False,
            odom_feedback_fresh=True,
            command_bridge_fresh=command_bridge_fresh,
            execution_feedback_fresh=False,
        )
    return PlatformBridgeDecision(
        output_feedback=False,
        source='stale',
        explicit_feedback_fresh=explicit_fresh,
        odom_feedback_fresh=odom_fresh,
        command_bridge_fresh=command_bridge_fresh,
        execution_feedback_fresh=False,
    )

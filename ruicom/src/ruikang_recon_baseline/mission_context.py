"""Mission execution state container."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

from .common import DetectionFrame


@dataclass
class MissionContext:
    """Mutable mission runtime state shared across node and executor.

    The context acts as a lightweight blackboard so ROS callbacks, execution logic,
    and artifact emission can exchange state without growing ad-hoc attributes on the
    node object itself.
    """

    state: str = 'IDLE'
    state_since: float = 0.0
    route_index: int = -1
    retry_count: int = 0
    current_zone: str = ''
    current_route_id: str = ''
    current_step_id: str = ''
    current_task_type: str = ''
    current_task_objective: str = ''
    zone_results_dynamic: Dict[str, dict] = field(default_factory=dict)
    latest_detection_frame: Optional[DetectionFrame] = None
    mission_started_at: float = 0.0
    dispatch_started_at: float = 0.0
    current_capture_deadline: float = 0.0
    last_health_publish_mono: float = 0.0
    last_tf_warning_mono: float = 0.0
    dispatch_quiesce_until: float = 0.0
    event_counter: int = 0
    schema_blocked_reason: str = ''
    preflight_deadline: float = 0.0
    last_preflight_warning_mono: float = 0.0
    active_behavior_command: Dict[str, object] = field(default_factory=dict)
    active_behavior_feedback: Dict[str, object] = field(default_factory=dict)
    behavior_action_deadline: float = 0.0
    behavior_dwell_after_success: bool = False

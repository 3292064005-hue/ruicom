"""Runtime ROS graph expectation helpers."""

from __future__ import annotations

from typing import Dict, Iterable, List, Mapping


def _normalize_items(items: Iterable[object]) -> List[str]:
    return [str(item).strip() for item in items if str(item).strip()]


def build_runtime_graph_expectations(*, enable_vision: bool, enable_mission: bool, enable_recorder: bool, enable_safety: bool, enable_platform_bridge: bool, mission_config: Mapping[str, object] | None = None, vision_config: Mapping[str, object] | None = None, platform_config: Mapping[str, object] | None = None) -> Dict[str, object]:
    mission_config = dict(mission_config or {})
    vision_config = dict(vision_config or {})
    platform_config = dict(platform_config or {})
    expected_nodes: List[str] = []
    expected_topics: List[str] = []
    expected_actions: List[str] = []
    if enable_vision:
        expected_nodes.append('vision_counter_node')
        expected_topics.extend(_normalize_items([
            vision_config.get('camera_topic', 'camera/color/image_raw'),
            vision_config.get('detections_topic', 'recon/detections'),
            vision_config.get('frame_region_counts_typed_topic', 'recon/zone_counts_typed'),
        ]))
    if enable_mission:
        expected_nodes.append('mission_manager_node')
        expected_topics.extend(_normalize_items([
            mission_config.get('detections_topic', 'recon/detections'),
            mission_config.get('mission_state_topic', 'recon/mission_state'),
        ]))
        action_name = str(mission_config.get('move_base_action_name', '')).strip()
        if action_name:
            expected_actions.append(action_name)
    if enable_recorder:
        expected_nodes.append('mission_recorder_node')
        expected_topics.extend(_normalize_items(['recon/summary_snapshot_v2']))
    if enable_safety:
        expected_nodes.append('cmd_safety_mux_node')
        expected_topics.extend(_normalize_items([
            platform_config.get('command_input_topic', 'cmd_vel_raw'),
            platform_config.get('safety_output_topic', 'cmd_vel'),
        ]))
    if enable_platform_bridge:
        expected_nodes.append('platform_bridge_node')
        expected_topics.extend(_normalize_items([
            platform_config.get('upstream_feedback_topic', 'recon/platform/vendor/base_feedback_raw'),
            platform_config.get('navigation_status_topic', 'recon/navigation_status'),
        ]))
    return {
        'expected_nodes': sorted(set(expected_nodes)),
        'expected_topics': sorted(set(expected_topics)),
        'expected_actions': sorted(set(expected_actions)),
    }

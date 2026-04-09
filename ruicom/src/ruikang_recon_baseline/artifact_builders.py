"""Artifact builders for recorder snapshots and final summaries."""

from __future__ import annotations

import csv
import time

from .common import SCHEMA_VERSION
from .recorder_core import build_summary_snapshot_payload, build_summary_snapshot_v2_payload


class RecorderArtifactBuilder:
    """Build recorder snapshot and final artifact payloads from node state."""

    def build_summary(self, owner):
        """Build the legacy summary snapshot/final-summary payload."""
        return build_summary_snapshot_payload(
            generated_at_wall=time.time(),
            schema_version=SCHEMA_VERSION,
            mission_state=owner.last_mission_state,
            current_zone=owner.current_zone,
            authoritative_input=owner.config['recorder_authoritative_input'],
            typed_seen=owner.typed_seen,
            zone_results=owner.zone_results,
            latest_frame_region_counts=owner.latest_frame_region_counts,
            last_health=owner.last_health,
            terminal_state_seen=owner.terminal_state_seen,
            operator_interventions=owner.operator_interventions,
        )

    def build_summary_v2(self, owner):
        """Build the dynamic summary snapshot/final-summary payload."""
        return build_summary_snapshot_v2_payload(
            generated_at_wall=time.time(),
            schema_version=SCHEMA_VERSION,
            mission_state=owner.last_mission_state,
            current_zone=owner.current_zone,
            authoritative_input=owner.config['recorder_authoritative_input'],
            typed_seen=owner.typed_seen,
            class_names=owner.config['classes'],
            zone_results_dynamic=owner.zone_results_dynamic,
            latest_frame_region_counts=owner.latest_frame_region_counts,
            last_health=owner.last_health,
            terminal_state_seen=owner.terminal_state_seen,
            operator_interventions=owner.operator_interventions,
        )

    def write_csv_legacy(self, owner, zone_results: dict):
        """Write the legacy CSV projection of zone results."""
        with open(owner.final_summary_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['route_id', 'zone_name', 'status', 'friendly', 'enemy', 'hostage', 'frame_count', 'frame_region', 'capture_started_at', 'capture_finished_at', 'failure_reason'])
            for route_id, result in zone_results.items():
                writer.writerow([
                    route_id,
                    result.get('zone_name', ''),
                    result.get('status', ''),
                    result.get('friendly', 0),
                    result.get('enemy', 0),
                    result.get('hostage', 0),
                    result.get('frame_count', 0),
                    result.get('frame_region', ''),
                    result.get('capture_started_at', 0),
                    result.get('capture_finished_at', 0),
                    result.get('failure_reason', ''),
                ])

    def write_csv_v2(self, owner, zone_results_dynamic: dict):
        """Write the dynamic CSV projection aligned to the configured class order."""
        class_names = list(owner.config['classes'])
        header = ['route_id', 'zone_name', 'status', *class_names, 'frame_count', 'frame_region', 'capture_started_at', 'capture_finished_at', 'failure_reason']
        with open(owner.final_summary_v2_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for route_id, result in zone_results_dynamic.items():
                counts = result.get('counts', {})
                writer.writerow([
                    route_id,
                    result.get('zone_name', ''),
                    result.get('status', ''),
                    *[counts.get(name, 0) for name in class_names],
                    result.get('frame_count', 0),
                    result.get('frame_region', ''),
                    result.get('capture_started_at', 0),
                    result.get('capture_finished_at', 0),
                    result.get('failure_reason', ''),
                ])


    def build_official_report(self, owner):
        report = self.build_summary_v2(owner)
        sink_path = str(owner.config.get('official_report_sink_path', '')).strip()
        submission_mode = str(owner.config.get('official_report_mode', 'artifact')).strip().lower() or 'artifact'
        contract_path = str(owner.config.get('official_report_contract_path', '')).strip()
        adapter_type = str((owner.submission_contract or {}).get('adapter_type', '')).strip()
        return {
            "report_schema": owner.config["official_report_schema"],
            "report_version": owner.config["official_report_version"],
            "generated_at_wall": time.time(),
            "mission_state": report.get("mission_state", {}),
            "route_total": report.get("route_total", 0),
            "class_names": list(report.get("class_names", [])),
            "zone_results_dynamic": dict(report.get("zone_results_dynamic", {})),
            "task_results_dynamic": dict(report.get("task_results_dynamic", {})),
            "totals_by_class": dict(report.get("totals_by_class", {})),
            "totals_by_task_type": dict(report.get("totals_by_task_type", {})),
            "totals_by_objective": dict(report.get("totals_by_objective", {})),
            "hazard_observations": list(report.get("hazard_observations", [])),
            "facility_actions": list(report.get("facility_actions", [])),
            "operator_interventions": dict(report.get("operator_interventions", {})),
            "recorder_authoritative_input": owner.config["recorder_authoritative_input"],
            "acceptance_stage": str(owner.config.get('acceptance_stage', '')).strip() or 'contract_smoke',
            "submission_contract": {
                "mode": submission_mode,
                "sink_path": sink_path,
                "contract_path": contract_path,
                "adapter_type": adapter_type,
                "ready": bool(submission_mode == 'artifact' or contract_path),
                "submission_id": str(owner.config.get('official_report_submission_id', '')).strip(),
            },
        }

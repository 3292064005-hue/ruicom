"""Recorder finalization controller."""

from __future__ import annotations

import time

import rospy

from .artifact_builders import RecorderArtifactBuilder
from .common import ConfigurationError
from .io_core import atomic_write_json
from .submission_adapters import submit_official_report


class RecorderFinalizeController:
    """Handle snapshot flush and terminal artifact finalization."""

    def __init__(self, builder: RecorderArtifactBuilder):
        self.builder = builder

    def should_finalize_locked(self, owner, force: bool) -> bool:
        """Determine whether finalization should proceed under the current state."""
        if owner.finalized and not force:
            return False
        if force:
            return True
        if not owner.terminal_state_seen:
            return False
        if owner.config['terminal_finalize_policy'] == 'immediate':
            return True
        route_total = int(owner.last_mission_state.get('route_total', 0) or 0)
        if route_total > 0 and len(owner.zone_results_dynamic) >= route_total:
            return True
        now_wall = time.time()
        quiet_for_terminal = now_wall - max(owner.last_terminal_wall_time, 0.0)
        quiet_for_zone_capture = now_wall - max(owner.last_zone_capture_wall_time, owner.last_terminal_wall_time, 0.0)
        return quiet_for_terminal >= owner.config['terminal_quiesce_sec'] and quiet_for_zone_capture >= owner.config['terminal_quiesce_sec']

    def finalize_locked(self, owner, *, force: bool, rewrite: bool = False):
        """Finalize recorder artifacts atomically.

        Args:
            owner: Recorder node state container.
            force: Whether to force finalization regardless of terminal quiet time.
            rewrite: Whether an already-finalized artifact may be rewritten.
        """
        if not self.should_finalize_locked(owner, force=force) and not rewrite:
            return
        if owner.finalized and not rewrite and not force:
            return
        try:
            summary = owner._inject_recorder_diagnostics(self.builder.build_summary(owner))
            summary_v2 = owner._inject_recorder_diagnostics(self.builder.build_summary_v2(owner))
            if owner.consistency_blocked:
                raise ConfigurationError('recorder consistency gate blocked finalization due to lane mismatches')
            writer_error = owner.writer.last_error
            if writer_error:
                summary.setdefault('writer_status', {})['last_error'] = writer_error
                summary_v2.setdefault('writer_status', {})['last_error'] = writer_error
            if owner.writer.dropped_messages:
                summary.setdefault('writer_status', {})['dropped_messages'] = owner.writer.dropped_messages
                summary_v2.setdefault('writer_status', {})['dropped_messages'] = owner.writer.dropped_messages
            atomic_write_json(owner.final_summary_json, summary)
            atomic_write_json(owner.final_summary_v2_json, summary_v2)
            self.builder.write_csv_legacy(owner, summary['zone_results'])
            self.builder.write_csv_v2(owner, summary_v2['zone_results_dynamic'])
            if owner.config['official_report_mode'] != 'disabled':
                official_report = owner._inject_recorder_diagnostics(self.builder.build_official_report(owner))
                atomic_write_json(owner.official_report_json, official_report)
                if owner.config['official_report_mode'] == 'artifact':
                    if owner.config['official_report_sink_path']:
                        atomic_write_json(owner.config['official_report_sink_path'], official_report)
                    owner.last_submission_receipt = {}
                elif owner.config['official_report_mode'] == 'judge_contract':
                    receipt = submit_official_report(
                        owner.submission_contract or {},
                        official_report,
                        report_path=owner.official_report_json,
                        submission_id=str(owner.config.get('official_report_submission_id', '')).strip(),
                    )
                    owner.last_submission_receipt = dict(receipt)
                    atomic_write_json(owner.official_report_receipt_json, receipt)
                    official_report = owner._inject_recorder_diagnostics(dict(official_report))
                    atomic_write_json(owner.official_report_json, official_report)
                    if owner.config['official_report_sink_path']:
                        atomic_write_json(owner.config['official_report_sink_path'], official_report)
            owner.finalized = True
            owner._flush_snapshot(force=True)
            rospy.loginfo('mission_recorder finalized: %s (rewrite=%s)', owner.final_summary_json, rewrite)
        except Exception as exc:
            owner._append('finalize_failed', {'error': str(exc)})
            rospy.logerr_throttle(2.0, 'mission_recorder finalize failed: %s', exc)

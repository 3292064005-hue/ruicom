#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record mission and health streams into durable artifacts."""

from __future__ import annotations

import csv
import os
import time

import rospy
from std_msgs.msg import String

from ruikang_recon_baseline.common import JsonCodec, SCHEMA_VERSION, expand_path
from ruikang_recon_baseline.io_core import AsyncJsonlWriter, atomic_write_json
from ruikang_recon_baseline.msg import MissionState, ZoneCapture


class MissionRecorderNode:
    """Record mission data streams and write final artifacts atomically."""

    def __init__(self):
        rospy.init_node('mission_recorder_node', anonymous=False)
        self.config = self._read_config()
        self.output_root = expand_path(self.config['output_root'])
        os.makedirs(self.output_root, exist_ok=True)
        self.writer = AsyncJsonlWriter(
            os.path.join(self.output_root, 'mission_log.jsonl'),
            max_queue_size=int(self.config['writer_queue_size']),
            rotate_max_bytes=int(self.config['writer_rotate_max_bytes']),
            rotate_keep=int(self.config['writer_rotate_keep']),
        )
        self.final_summary_json = os.path.join(self.output_root, 'final_summary.json')
        self.final_summary_csv = os.path.join(self.output_root, 'final_summary.csv')
        self.current_snapshot_json = os.path.join(self.output_root, 'summary_snapshot.json')

        self.current_zone = ''
        self.latest_frame_region_counts = {}
        self.zone_results = {}
        self.last_mission_state = {}
        self.last_health = {}
        self.finalized = False

        rospy.Subscriber(self.config['mission_state_topic'], String, self._mission_state_json_cb, queue_size=50)
        rospy.Subscriber(self.config['zone_capture_topic'], String, self._zone_capture_json_cb, queue_size=50)
        rospy.Subscriber(self.config['frame_region_counts_topic'], String, self._frame_region_counts_cb, queue_size=50)
        rospy.Subscriber(self.config['health_topic'], String, self._health_cb, queue_size=50)
        rospy.Subscriber(self.config['current_zone_topic'], String, self._current_zone_cb, queue_size=50)
        rospy.Subscriber(self.config['mission_state_typed_topic'], MissionState, self._mission_state_typed_cb, queue_size=10)
        rospy.Subscriber(self.config['zone_capture_typed_topic'], ZoneCapture, self._zone_capture_typed_cb, queue_size=10)
        self.summary_pub = rospy.Publisher(self.config['summary_snapshot_topic'], String, queue_size=10, latch=True)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo('mission_recorder_node started. output_root=%s', self.output_root)

    def _read_config(self):
        return {
            'mission_state_topic': rospy.get_param('~mission_state_topic', '/recon/mission_state'),
            'mission_state_typed_topic': rospy.get_param('~mission_state_typed_topic', '/recon/mission_state_typed'),
            'zone_capture_topic': rospy.get_param('~zone_capture_topic', '/recon/zone_capture_result'),
            'zone_capture_typed_topic': rospy.get_param('~zone_capture_typed_topic', '/recon/zone_capture_result_typed'),
            'frame_region_counts_topic': rospy.get_param('~frame_region_counts_topic', '/recon/zone_counts'),
            'current_zone_topic': rospy.get_param('~current_zone_topic', '/recon/current_zone'),
            'health_topic': rospy.get_param('~health_topic', '/recon/health'),
            'summary_snapshot_topic': rospy.get_param('~summary_snapshot_topic', '/recon/summary_snapshot'),
            'output_root': rospy.get_param('~output_root', '~/.ros/ruikang_recon'),
            'writer_queue_size': int(rospy.get_param('~writer_queue_size', 512)),
            'writer_rotate_max_bytes': int(rospy.get_param('~writer_rotate_max_bytes', 5 * 1024 * 1024)),
            'writer_rotate_keep': int(rospy.get_param('~writer_rotate_keep', 3)),
        }

    def _append(self, event_type: str, payload: dict):
        self.writer.write({'stamp': time.time(), 'type': event_type, 'payload': payload})

    def _safe_load_json(self, text: str, topic_name: str):
        try:
            return JsonCodec.loads(text)
        except Exception as exc:
            self._append('invalid_json', {'topic': topic_name, 'error': str(exc), 'raw': text[:512]})
            rospy.logwarn_throttle(2.0, '%s invalid JSON: %s', topic_name, exc)
            return None

    def _safe_publish_snapshot(self):
        try:
            snapshot = self._build_summary()
            writer_error = self.writer.last_error
            if writer_error:
                snapshot.setdefault('writer_status', {})['last_error'] = writer_error
            if self.writer.dropped_messages:
                snapshot.setdefault('writer_status', {})['dropped_messages'] = self.writer.dropped_messages
            atomic_write_json(self.current_snapshot_json, snapshot)
            self.summary_pub.publish(String(data=JsonCodec.dumps(snapshot)))
        except Exception as exc:
            self._append('snapshot_write_failed', {'error': str(exc)})
            rospy.logwarn_throttle(2.0, 'summary snapshot write failed: %s', exc)

    def _current_zone_cb(self, msg: String):
        self.current_zone = msg.data.strip()

    def _health_cb(self, msg: String):
        payload = self._safe_load_json(msg.data, self.config['health_topic'])
        if payload is None:
            return
        self.last_health[payload.get('node', 'unknown')] = payload
        self._append('health', payload)
        self._safe_publish_snapshot()

    def _frame_region_counts_cb(self, msg: String):
        payload = self._safe_load_json(msg.data, self.config['frame_region_counts_topic'])
        if payload is None:
            return
        self.latest_frame_region_counts = payload.get('frame_region_counts', {})
        self._append('frame_region_counts', payload)
        self._safe_publish_snapshot()

    def _mission_state_json_cb(self, msg: String):
        payload = self._safe_load_json(msg.data, self.config['mission_state_topic'])
        if payload is None:
            return
        self.last_mission_state = payload
        self._append('mission_state_json', payload)
        state = str(payload.get('state', ''))
        if state in ('FINISHED', 'TIMEOUT', 'FAULT'):
            self._finalize()
        self._safe_publish_snapshot()

    def _zone_capture_json_cb(self, msg: String):
        payload = self._safe_load_json(msg.data, self.config['zone_capture_topic'])
        if payload is None:
            return
        zone_name = str(payload.get('zone_name', '')).strip()
        if zone_name:
            self.zone_results[zone_name] = payload
        self._append('zone_capture_json', payload)
        self._safe_publish_snapshot()

    def _mission_state_typed_cb(self, msg: MissionState):
        self._append('mission_state_typed', {
            'stamp': msg.header.stamp.to_sec() if msg.header.stamp else time.time(),
            'state': msg.state,
            'event': msg.event,
            'route_index': msg.route_index,
            'route_total': msg.route_total,
            'current_zone': msg.current_zone,
            'schema_version': msg.schema_version,
            'details_json': msg.details_json,
        })

    def _zone_capture_typed_cb(self, msg: ZoneCapture):
        self._append('zone_capture_typed', {
            'stamp': msg.header.stamp.to_sec() if msg.header.stamp else time.time(),
            'zone_name': msg.zone_name,
            'status': msg.status,
            'friendly': msg.friendly,
            'enemy': msg.enemy,
            'hostage': msg.hostage,
            'frame_count': msg.frame_count,
            'frame_region': msg.frame_region,
            'failure_reason': msg.failure_reason,
            'schema_version': msg.schema_version,
        })

    def _build_summary(self):
        totals = {'friendly': 0, 'enemy': 0, 'hostage': 0}
        for zone_name, result in self.zone_results.items():
            _ = zone_name
            for key in totals:
                totals[key] += int(result.get(key, 0))
        return {
            'generated_at': time.time(),
            'schema_version': SCHEMA_VERSION,
            'final_state': self.last_mission_state.get('state', 'UNKNOWN'),
            'route_index': self.last_mission_state.get('route_index', -1),
            'route_total': self.last_mission_state.get('route_total', 0),
            'current_zone': self.current_zone,
            'zone_results': self.zone_results,
            'latest_frame_region_counts': self.latest_frame_region_counts,
            'last_health': self.last_health,
            'totals': totals,
        }

    def _write_csv(self, zone_results: dict):
        with open(self.final_summary_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['zone', 'status', 'friendly', 'enemy', 'hostage', 'frame_count', 'frame_region', 'capture_started_at', 'capture_finished_at', 'failure_reason'])
            for zone_name, result in zone_results.items():
                writer.writerow([
                    zone_name,
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

    def _finalize(self):
        if self.finalized:
            return
        summary = self._build_summary()
        writer_error = self.writer.last_error
        if writer_error:
            summary.setdefault('writer_status', {})['last_error'] = writer_error
        if self.writer.dropped_messages:
            summary.setdefault('writer_status', {})['dropped_messages'] = self.writer.dropped_messages
        atomic_write_json(self.final_summary_json, summary)
        self._write_csv(summary['zone_results'])
        self.finalized = True
        rospy.loginfo('mission_recorder finalized: %s', self.final_summary_json)

    def _on_shutdown(self):
        try:
            self._finalize()
        finally:
            self.writer.close()

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MissionRecorderNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

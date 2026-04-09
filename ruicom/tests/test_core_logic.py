import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from ruikang_recon_baseline.common import ConfigurationError, Detection, DetectionFrame, PoseSnapshot, Waypoint, class_schema_hash, load_waypoints, quaternion_to_yaw_rad, validate_route_frame_region_contract
from ruikang_recon_baseline.io_core import AsyncJsonlWriter
from ruikang_recon_baseline.mission_core import AggregationPolicy, ArrivalEvaluator, CaptureWindowAggregator
from ruikang_recon_baseline.safety_core import SafetyController, evaluate_output_feedback_policy
from ruikang_recon_baseline.vision_core import ColorBlobDetector, FrameRegionAdapter


class CaptureWindowAggregatorTest(unittest.TestCase):
    def test_median_aggregation_with_frame_region_filter(self):
        aggregator = CaptureWindowAggregator(AggregationPolicy(class_names=('friendly', 'enemy', 'hostage'), reduction='median', min_valid_frames=2))
        aggregator.start('zone_a', 10.0, frame_region='zone_a')
        for stamp, friendly_count in [(10.1, 1), (10.2, 2), (10.3, 2)]:
            detections = [Detection('friendly', 0.9, 0, 0, 1, 1, frame_region='zone_a') for _ in range(friendly_count)]
            detections.append(Detection('enemy', 0.8, 0, 0, 1, 1, frame_region='zone_b'))
            frame = DetectionFrame(stamp=stamp, frame_id='camera', detector_type='unit', schema_version='2.0', detections=detections)
            aggregator.feed(frame, capture_end_hint=11.0)
        result = aggregator.finalize(status='ok')
        self.assertEqual(result.friendly, 2)
        self.assertEqual(result.enemy, 0)
        self.assertEqual(result.frame_count, 3)

    def test_frames_after_capture_deadline_are_rejected(self):
        aggregator = CaptureWindowAggregator(AggregationPolicy(class_names=('friendly', 'enemy', 'hostage'), reduction='median', min_valid_frames=1))
        aggregator.start('zone_a', 10.0, frame_region='zone_a')
        accepted = aggregator.feed(
            DetectionFrame(stamp=10.2, frame_id='camera', detector_type='unit', schema_version='2.0', detections=[Detection('friendly', 0.9, 0, 0, 1, 1, frame_region='zone_a')]),
            capture_end_hint=10.5,
        )
        rejected = aggregator.feed(
            DetectionFrame(stamp=10.7, frame_id='camera', detector_type='unit', schema_version='2.0', detections=[Detection('friendly', 0.9, 0, 0, 1, 1, frame_region='zone_a')]),
            capture_end_hint=10.5,
        )
        result = aggregator.finalize(status='ok')
        self.assertTrue(accepted)
        self.assertFalse(rejected)
        self.assertEqual(result.frame_count, 1)
        self.assertEqual(result.friendly, 1)



    def test_finalize_emits_position_evidence_summary(self):
        aggregator = CaptureWindowAggregator(AggregationPolicy(class_names=('friendly', 'enemy', 'hostage'), reduction='median', min_valid_frames=1))
        aggregator.start('zone_a', 10.0, frame_region='zone_a')
        frame = DetectionFrame(
            stamp=10.2,
            frame_id='camera',
            detector_type='unit',
            schema_version='2.0',
            detections=[
                Detection('friendly', 0.9, 10, 10, 20, 20, frame_region='zone_a', observed_position_type='frame_region', observed_position_label='zone_a', evidence_source='unit_test'),
                Detection('friendly', 0.9, 12, 12, 22, 22, frame_region='zone_a', observed_position_type='frame_region', observed_position_label='zone_a', evidence_source='unit_test'),
            ],
        )
        aggregator.feed(frame, capture_end_hint=11.0)
        result = aggregator.finalize(status='ok')
        self.assertEqual(result.position_estimates[0]['position_label'], 'zone_a')
        self.assertEqual(result.evidence_summary['position_observation_count'], 2)
        self.assertEqual(result.evidence_summary['evidence_sources']['unit_test'], 2)

class ArrivalEvaluatorTest(unittest.TestCase):
    def test_frame_mismatch_returns_none(self):
        evaluator = ArrivalEvaluator(comparison_frame='map', reach_tolerance_m=0.5, pose_timeout_sec=1.0)
        evaluator.update_pose(PoseSnapshot(stamp=1.0, frame_id='odom', x=0.0, y=0.0, yaw_rad=0.0, source='odom'))
        waypoint = Waypoint(name='zone_a', x=0.0, y=0.0, yaw_deg=0.0, dwell_sec=3.0, timeout_sec=10.0, goal_frame='map')
        self.assertIsNone(evaluator.distance_to_waypoint(waypoint, now_sec=1.1))


class RouteIdentityTest(unittest.TestCase):
    def test_load_waypoints_assigns_stable_unique_route_ids(self):
        waypoints = load_waypoints([
            {'name': 'zone_a', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
            {'name': 'zone_b', 'route_id': 'custom_b', 'x': 2.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
            {'name': 'zone_a', 'x': 1.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
        ], dwell_default=1.0)
        self.assertEqual([item.route_id for item in waypoints], ['zone_a__1', 'custom_b', 'zone_a__2'])

    def test_load_waypoints_rejects_duplicate_explicit_route_ids(self):
        with self.assertRaises(ConfigurationError):
            load_waypoints([
                {'name': 'zone_a', 'route_id': 'dup', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
                {'name': 'zone_b', 'route_id': 'dup', 'x': 1.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
            ], dwell_default=1.0)




class RouteFrameRegionContractTest(unittest.TestCase):
    def test_route_frame_region_contract_accepts_known_bindings(self):
        waypoints = load_waypoints([
            {'name': 'zone_a', 'frame_region': 'zone_a', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
            {'name': 'zone_b', 'frame_region': 'zone_b', 'x': 1.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
        ], dwell_default=1.0)
        bindings = validate_route_frame_region_contract(waypoints, require_binding=True, allowed_frame_regions=['zone_a', 'zone_b'])
        self.assertEqual(bindings, ('zone_a', 'zone_b'))

    def test_route_frame_region_contract_rejects_unknown_region(self):
        waypoints = load_waypoints([
            {'name': 'zone_a', 'frame_region': 'zone_x', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'timeout_sec': 5.0},
        ], dwell_default=1.0)
        with self.assertRaises(ConfigurationError):
            validate_route_frame_region_contract(waypoints, require_binding=True, allowed_frame_regions=['zone_a'])

class SafetyOutputFeedbackPolicyTest(unittest.TestCase):
    def test_required_feedback_without_topic_raises(self):
        with self.assertRaises(ConfigurationError):
            evaluate_output_feedback_policy(True, '', True)

    def test_stale_required_feedback_blocks_output(self):
        self.assertEqual(evaluate_output_feedback_policy(True, 'driver/heartbeat', False), (False, 'output_feedback_stale'))

    def test_optional_feedback_keeps_output_allowed(self):
        self.assertEqual(evaluate_output_feedback_policy(False, '', False), (True, 'ok'))


class ColorBlobDetectorConfigTest(unittest.TestCase):
    def test_class_map_requires_at_least_one_range(self):
        config = {
            'min_area_px': 10,
            'max_area_px': 1000,
            'color_blob': {
                'morph_kernel': 3,
                'class_map': {
                    'friendly': {'ranges': []},
                },
            },
        }
        with self.assertRaises(ConfigurationError):
            ColorBlobDetector(config, ['friendly'])

    def test_detector_accepts_dynamic_class_map(self):
        config = {
            'min_area_px': 10,
            'max_area_px': 1000,
            'color_blob': {
                'morph_kernel': 3,
                'class_map': {
                    'neutral': {'ranges': [{'lower': [0, 0, 0], 'upper': [10, 10, 10]}]},
                },
            },
        }
        detector = ColorBlobDetector(config, ['neutral'])
        self.assertEqual(detector.class_names, ('neutral',))



class SafetyControllerTest(unittest.TestCase):
    def test_high_priority_source_wins_and_is_clamped(self):
        controller = SafetyController(0.5, 0.5, 1.0, command_timeout_sec=1.0, estop_timeout_sec=1.0, require_fresh_estop=False)
        controller.update_estop(False, 0.0)
        controller.update_command('low', 10, 0.1, command=type('cmd', (), {'linear_x': 0.1, 'linear_y': 0.0, 'angular_z': 0.0})())
        controller.update_command('high', 20, 0.2, command=type('cmd', (), {'linear_x': 2.0, 'linear_y': 0.0, 'angular_z': 2.0})())
        status = controller.evaluate(0.3)
        self.assertEqual(status.selected_source, 'high')
        self.assertEqual(status.output.linear_x, 0.5)
        self.assertEqual(status.output.angular_z, 1.0)

    def test_invalid_default_mode_raises(self):
        with self.assertRaises(ConfigurationError):
            SafetyController(0.5, 0.5, 1.0, command_timeout_sec=1.0, estop_timeout_sec=1.0, require_fresh_estop=False, default_mode='BROKEN')


class FrameRegionAdapterTest(unittest.TestCase):
    def test_named_region_assignment(self):
        adapter = FrameRegionAdapter('named_regions', [{'name': 'zone_a', 'x0': 0, 'y0': 0, 'x1': 100, 'y1': 100}])
        detections = [Detection('friendly', 0.9, 10, 10, 20, 20)]
        assigned = adapter.assign(detections)
        self.assertEqual(assigned[0].frame_region, 'zone_a')

    def test_region_assignment_preserves_position_evidence(self):
        adapter = FrameRegionAdapter('named_regions', [{'name': 'zone_a', 'x0': 0, 'y0': 0, 'x1': 100, 'y1': 100}])
        detections = [Detection(
            'friendly',
            0.9,
            10,
            10,
            20,
            20,
            observed_position_type='map_xy',
            observed_position_label='alpha',
            observed_position_x_m=1.25,
            observed_position_y_m=2.5,
            evidence_source='pose_fusion',
        )]
        assigned = adapter.assign(detections)
        self.assertEqual(assigned[0].frame_region, 'zone_a')
        self.assertEqual(assigned[0].observed_position_type, 'map_xy')
        self.assertEqual(assigned[0].observed_position_label, 'alpha')
        self.assertAlmostEqual(assigned[0].observed_position_x_m, 1.25)
        self.assertAlmostEqual(assigned[0].observed_position_y_m, 2.5)
        self.assertEqual(assigned[0].evidence_source, 'pose_fusion')

    def test_invalid_named_region_raises(self):
        with self.assertRaises(ConfigurationError):
            FrameRegionAdapter('named_regions', [{'name': 'zone_a', 'x0': 10, 'y0': 10, 'x1': 5, 'y1': 100}])




class QuaternionHelperTest(unittest.TestCase):
    def test_quaternion_to_yaw_rad_extracts_planar_rotation(self):
        yaw = quaternion_to_yaw_rad(0.0, 0.0, 0.70710678118, 0.70710678118)
        self.assertAlmostEqual(yaw, 1.57079632679, places=5)

class AsyncJsonlWriterTest(unittest.TestCase):
    def test_queue_overflow_does_not_raise(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / 'events.jsonl'
            writer = AsyncJsonlWriter(str(path), max_queue_size=1, rotate_max_bytes=0, rotate_keep=1)
            for idx in range(5):
                writer.write({'idx': idx})
            writer.close()
            self.assertTrue(path.exists())

    def test_write_failure_sets_last_error(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / 'events.jsonl'
            with mock.patch('builtins.open', side_effect=OSError('disk full')):
                writer = AsyncJsonlWriter(str(path), max_queue_size=2, rotate_max_bytes=0, rotate_keep=1)
                writer.write({'idx': 1})
                time.sleep(0.3)
                writer.close()
                self.assertIn('disk full', writer.last_error or '')


if __name__ == '__main__':
    unittest.main()

import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from ruikang_recon_baseline.common import ConfigurationError, Detection, DetectionFrame, PoseSnapshot, Waypoint, quaternion_to_yaw_rad
from ruikang_recon_baseline.io_core import AsyncJsonlWriter
from ruikang_recon_baseline.mission_core import AggregationPolicy, ArrivalEvaluator, CaptureWindowAggregator
from ruikang_recon_baseline.safety_core import SafetyController
from ruikang_recon_baseline.vision_core import FrameRegionAdapter


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


class ArrivalEvaluatorTest(unittest.TestCase):
    def test_frame_mismatch_returns_none(self):
        evaluator = ArrivalEvaluator(comparison_frame='map', reach_tolerance_m=0.5, pose_timeout_sec=1.0)
        evaluator.update_pose(PoseSnapshot(stamp=1.0, frame_id='odom', x=0.0, y=0.0, yaw_rad=0.0, source='odom'))
        waypoint = Waypoint(name='zone_a', x=0.0, y=0.0, yaw_deg=0.0, dwell_sec=3.0, timeout_sec=10.0, goal_frame='map')
        self.assertIsNone(evaluator.distance_to_waypoint(waypoint, now_sec=1.1))


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

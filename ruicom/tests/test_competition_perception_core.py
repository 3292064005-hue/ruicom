import unittest

from ruikang_recon_baseline.common import Detection, DetectionFrame
from ruikang_recon_baseline.competition_perception_core import CompetitionFusionConfig, CompetitionPerceptionCore


class CompetitionPerceptionCoreTest(unittest.TestCase):
    def test_legacy_enemy_promotes_to_enemy_soldier(self):
        core = CompetitionPerceptionCore(CompetitionFusionConfig(stability_window=3, min_confirmed_frames=2))
        frame = DetectionFrame(
            stamp=1.0,
            frame_id='camera',
            detector_type='unit',
            schema_version='2.4.0',
            source_image_width=640,
            source_image_height=480,
            class_names=['friendly', 'enemy', 'hostage'],
            detections=[Detection('enemy', 0.9, 200, 120, 260, 300)],
        )
        first = core.process(frame)
        self.assertEqual(first.detections, [])
        second = core.process(frame)
        self.assertEqual(len(second.detections), 1)
        self.assertEqual(second.detections[0].class_name, 'enemy_soldier')

    def test_soldier_marker_association(self):
        core = CompetitionPerceptionCore(CompetitionFusionConfig(stability_window=3, min_confirmed_frames=2))
        frame = DetectionFrame(
            stamp=1.0,
            frame_id='camera',
            detector_type='unit',
            schema_version='2.4.0',
            source_image_width=640,
            source_image_height=480,
            class_names=['soldier', 'enemy_marker'],
            detections=[
                Detection('soldier', 0.9, 200, 100, 300, 320),
                Detection('enemy_marker', 0.8, 225, 160, 265, 230),
            ],
        )
        core.process(frame)
        result = core.process(frame)
        self.assertEqual([item.class_name for item in result.detections], ['enemy_soldier'])

    def test_mine_requires_ground_band(self):
        core = CompetitionPerceptionCore(CompetitionFusionConfig(stability_window=3, min_confirmed_frames=2))
        floating = DetectionFrame(
            stamp=1.0,
            frame_id='camera',
            detector_type='unit',
            schema_version='2.4.0',
            source_image_width=640,
            source_image_height=480,
            class_names=['mine'],
            detections=[Detection('mine', 0.9, 100, 100, 130, 130)],
        )
        ground = DetectionFrame(
            stamp=1.1,
            frame_id='camera',
            detector_type='unit',
            schema_version='2.4.0',
            source_image_width=640,
            source_image_height=480,
            class_names=['mine'],
            detections=[Detection('mine', 0.9, 100, 360, 140, 400)],
        )
        self.assertEqual(core.process(floating).detections, [])
        core.process(ground)
        result = core.process(ground)
        self.assertEqual([item.class_name for item in result.detections], ['confirmed_mine'])


if __name__ == '__main__':
    unittest.main()

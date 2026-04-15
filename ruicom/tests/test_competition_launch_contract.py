import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class CompetitionLaunchContractTest(unittest.TestCase):
    def test_reference_competition_launch_enables_competition_perception(self):
        text = (ROOT / 'launch/reference_competition_deploy.launch').read_text(encoding='utf-8')
        self.assertIn('enable_competition_perception" value="true', text)
        self.assertIn('config/profiles/reference_competition/mission.yaml', text)
        self.assertIn('config/profiles/reference_competition/behavior_runtime.yaml', text)
        self.assertIn('config/profiles/reference_competition/system_manager.yaml', text)

    def test_field_competition_launch_enables_competition_perception(self):
        text = (ROOT / 'launch/field_competition_deploy.launch').read_text(encoding='utf-8')
        self.assertIn('enable_competition_perception" value="true', text)
        self.assertIn('config/profiles/field_competition/mission.yaml', text)
        self.assertIn('config/profiles/field_competition/behavior_runtime.yaml', text)
        self.assertIn('config/profiles/field_competition/system_manager.yaml', text)


if __name__ == '__main__':
    unittest.main()

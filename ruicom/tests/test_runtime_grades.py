import unittest
from pathlib import Path

import yaml


class RuntimeGradeProfileTest(unittest.TestCase):
    def _load_yaml(self, rel_path: str):
        root = Path(__file__).resolve().parents[1]
        return yaml.safe_load((root / rel_path).read_text(encoding='utf-8')) or {}

    def test_deploy_profiles_carry_explicit_runtime_grades(self):
        baseline = self._load_yaml('config/profiles/baseline/mission.yaml')
        reference = self._load_yaml('config/profiles/reference_deploy/mission.yaml')
        field = self._load_yaml('config/profiles/field_deploy/mission.yaml')
        self.assertEqual(baseline.get('runtime_grade'), 'contract')
        self.assertEqual(reference.get('runtime_grade'), 'reference')
        self.assertEqual(field.get('runtime_grade'), 'field')

    def test_behavior_backend_is_enabled_for_reference_and_field_profiles(self):
        reference = self._load_yaml('config/profiles/reference_deploy/mission.yaml')
        field = self._load_yaml('config/profiles/field_deploy/mission.yaml')
        self.assertEqual(reference.get('behavior_action_backend_type'), 'topic')
        self.assertEqual(field.get('behavior_action_backend_type'), 'topic')
        self.assertTrue(reference.get('require_behavior_feedback'))
        self.assertTrue(field.get('require_behavior_feedback'))


if __name__ == '__main__':
    unittest.main()

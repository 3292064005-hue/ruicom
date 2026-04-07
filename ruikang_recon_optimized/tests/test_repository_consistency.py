import xml.etree.ElementTree as ET
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class RepositoryConsistencyTest(unittest.TestCase):
    def test_all_yaml_files_parse(self):
        yaml_files = sorted((ROOT / 'config').rglob('*.yaml'))
        self.assertTrue(yaml_files)
        for path in yaml_files:
            with self.subTest(path=path.relative_to(ROOT)):
                with path.open('r', encoding='utf-8') as handle:
                    payload = yaml.safe_load(handle)
                self.assertIsNotNone(payload)

    def test_launch_files_reference_existing_yaml(self):
        launch_dir = ROOT / 'launch'
        for launch_file in sorted(launch_dir.glob('*.launch')):
            content = launch_file.read_text(encoding='utf-8')
            for token in ['config/common/vision.yaml', 'config/common/mission.yaml', 'config/common/recorder.yaml', 'config/common/safety.yaml']:
                if token in content:
                    self.assertTrue((ROOT / token).exists(), token)

    def test_xml_files_parse(self):
        for path in [ROOT / 'package.xml', *sorted((ROOT / 'launch').glob('*.launch'))]:
            with self.subTest(path=path.relative_to(ROOT)):
                ET.parse(path)

    def test_tf_and_status_config_documented_in_common_mission(self):
        content = (ROOT / 'config/common/mission.yaml').read_text(encoding='utf-8')
        for token in ['navigation_status_topic', 'navigation_status_timeout_sec', 'tf_target_frame', 'tf_source_frame', 'tf_lookup_timeout_sec']:
            with self.subTest(token=token):
                self.assertIn(token, content)


if __name__ == '__main__':
    unittest.main()

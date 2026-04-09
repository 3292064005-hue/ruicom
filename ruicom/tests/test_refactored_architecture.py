import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class RefactoredArchitectureTest(unittest.TestCase):
    def test_new_executor_and_recorder_modules_exist(self):
        expected = [
            'src/ruikang_recon_baseline/mission_node.py',
            'src/ruikang_recon_baseline/mission_executor.py',
            'src/ruikang_recon_baseline/mission_context.py',
            'src/ruikang_recon_baseline/mission_plan.py',
            'src/ruikang_recon_baseline/recovery_policies.py',
            'src/ruikang_recon_baseline/pose_sources.py',
            'src/ruikang_recon_baseline/navigation_adapters/base.py',
            'src/ruikang_recon_baseline/navigation_adapters/move_base.py',
            'src/ruikang_recon_baseline/navigation_adapters/simple_topic.py',
            'src/ruikang_recon_baseline/navigation_adapters/status_topic.py',
            'src/ruikang_recon_baseline/recorder_node.py',
            'src/ruikang_recon_baseline/recorder_ingest.py',
            'src/ruikang_recon_baseline/recorder_finalize.py',
            'src/ruikang_recon_baseline/artifact_builders.py',
            'src/ruikang_recon_baseline/contracts/__init__.py',
            'docs/upstream_alignment.md',
            'UPSTREAM_SOURCES.md',
            'THIRD_PARTY_NOTICES.md',
        ]
        for rel_path in expected:
            with self.subTest(path=rel_path):
                self.assertTrue((ROOT / rel_path).exists(), rel_path)

    def test_wrapper_scripts_are_thin_entrypoints(self):
        mission_wrapper = (ROOT / 'scripts/mission_manager_node.py').read_text(encoding='utf-8')
        recorder_wrapper = (ROOT / 'scripts/mission_recorder_node.py').read_text(encoding='utf-8')
        self.assertIn('from ruikang_recon_baseline.mission_node import MissionManagerNode', mission_wrapper)
        self.assertIn('from ruikang_recon_baseline.recorder_node import MissionRecorderNode', recorder_wrapper)
        self.assertLessEqual(len(mission_wrapper.splitlines()), 12)
        self.assertLessEqual(len(recorder_wrapper.splitlines()), 12)

    def test_executor_wires_plan_context_and_policy(self):
        tree = ast.parse((ROOT / 'src/ruikang_recon_baseline/mission_node.py').read_text(encoding='utf-8'))
        init_func = None
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == 'MissionManagerNode':
                for item in node.body:
                    if isinstance(item, ast.FunctionDef) and item.name == '__init__':
                        init_func = item
                        break
        self.assertIsNotNone(init_func)
        source = (ROOT / 'src/ruikang_recon_baseline/mission_node.py').read_text(encoding='utf-8')
        for token in ['MissionPlan.from_waypoints', 'MissionContext()', 'RetryThenFailPolicy', 'MissionExecutor(']:
            with self.subTest(token=token):
                self.assertIn(token, source)

    def test_readme_and_summary_document_refactor(self):
        readme = (ROOT / 'README.md').read_text(encoding='utf-8')
        summary = (ROOT / 'IMPLEMENTATION_SUMMARY.md').read_text(encoding='utf-8')
        for token in ['mission_executor.py', 'recorder_ingest.py', 'contracts/', 'THIRD_PARTY_NOTICES.md']:
            with self.subTest(token=token):
                self.assertIn(token, readme)
                self.assertIn(token, summary)

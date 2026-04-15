import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'vendor_workspace' / 'newznzc_ws'
VALIDATOR = ROOT / 'tools' / 'validate_managed_vendor_bundle.py'
PROVISION = ROOT / 'tools' / 'provision_vendor_workspace_fixture.py'

class ManagedVendorBundleGateTest(unittest.TestCase):
    def test_repo_workspace_is_rejected_when_external_required(self):
        result = subprocess.run(
            ['python3', str(VALIDATOR), '--workspace-root', str(FIXTURE), '--require-external-workspace'],
            cwd=str(ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('must be external to repository', result.stdout)

    def test_provisioned_external_workspace_passes_strict_validation(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            result = subprocess.run(
                ['python3', str(PROVISION), '--output-root', tmpdir],
                cwd=str(ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=True,
            )
            external_root = Path(result.stdout.strip())
            self.assertTrue(external_root.exists())
            self.assertTrue((external_root / '.ruikang_managed_vendor_workspace').exists())
            check = subprocess.run(
                ['python3', str(VALIDATOR), '--workspace-root', str(external_root), '--require-external-workspace'],
                cwd=str(ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.assertEqual(check.returncode, 0, check.stdout)
            self.assertIn('managed vendor bundle validation passed', check.stdout)

    def test_ci_workflow_provisions_or_uses_external_workspace(self):
        text = (ROOT / '.github/workflows/ci.yml').read_text(encoding='utf-8')
        self.assertIn('RUIKANG_VENDOR_WORKSPACE_ROOT', text)
        self.assertIn('provision_vendor_workspace_fixture.py', text)
        self.assertIn('--require-external-workspace', text)
        self.assertNotIn('--allow-repo-workspace', text)

if __name__ == '__main__':
    unittest.main()

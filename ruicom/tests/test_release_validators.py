import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ReleaseValidatorScriptsTest(unittest.TestCase):
    def _run(self, relative: str, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(ROOT / relative), *args],
            cwd=str(ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )

    def test_repository_hygiene_validator_passes_on_clean_temp_copy(self):
        with tempfile.TemporaryDirectory() as tmp:
            dst = Path(tmp) / 'repo'

            def _ignore(directory: str, names: list[str]):
                ignored = []
                for name in names:
                    if name in {'__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache', '.git'}:
                        ignored.append(name)
                    elif name.endswith(('.pyc', '.pyo')):
                        ignored.append(name)
                return ignored

            shutil.copytree(ROOT, dst, ignore=_ignore)
            result = self._run('tools/validate_repository_hygiene.py', str(dst))
            self.assertEqual(result.returncode, 0, msg=result.stdout)
            self.assertIn('repository tree is clean', result.stdout)

    def test_launch_contract_validator_passes_on_repo(self):
        result = self._run('tools/validate_launch_contracts.py')
        self.assertEqual(result.returncode, 0, msg=result.stdout)
        self.assertIn('validated', result.stdout)


if __name__ == '__main__':
    unittest.main()

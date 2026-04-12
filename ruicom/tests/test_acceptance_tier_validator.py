import subprocess
import sys
from pathlib import Path


def test_acceptance_tier_validator_succeeds():
    repo_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, 'tools/validate_acceptance_tiers.py'],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    output = result.stdout
    assert 'baseline: grade=contract' in output
    assert 'reference_deploy: grade=reference' in output
    assert 'field_deploy: grade=field' in output

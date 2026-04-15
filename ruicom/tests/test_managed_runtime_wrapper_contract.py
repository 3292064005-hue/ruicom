from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.vendor_bundle_manifest import load_vendor_bundle_manifest, resolve_vendor_bundle_startup_steps
from tools.run_managed_vendor_runtime import _build_launch_lines


def test_runtime_wrapper_propagates_vendor_launch_contracts() -> None:
    manifest = load_vendor_bundle_manifest(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'))
    steps = resolve_vendor_bundle_startup_steps(manifest, workspace_root=str(ROOT / 'vendor_workspace/newznzc_ws'), repo_root=str(ROOT))
    launch_text = '\n'.join(_build_launch_lines(steps))
    required_snippets = [
        'name="vendor_bringup_launch"',
        'name="vendor_navigation_launch"',
        'name="vendor_camera_launch"',
        'name="vendor_lidar_launch"',
        'name="vendor_imu_launch"',
        'name="enable_vendor_bringup" default="false"',
        'name="enable_vendor_navigation" default="false"',
        'name="enable_vendor_camera" default="false"',
        'name="enable_vendor_lidar" default="false"',
        'name="enable_vendor_imu" default="false"',
        'name="allow_command_fallback" default="false"',
    ]
    for snippet in required_snippets:
        assert snippet in launch_text

import json
import tempfile
import unittest
from pathlib import Path

from ruikang_recon_baseline.authoritative_replay import build_authoritative_replay_manifest, load_authoritative_summary
from ruikang_recon_baseline.field_asset_release import load_field_asset_release
from ruikang_recon_baseline.mission_dsl import load_task_graph_dsl
from ruikang_recon_baseline.runtime_graph import build_runtime_graph_expectations
from ruikang_recon_baseline.vendor_bundle_manifest import load_vendor_bundle_manifest, resolve_vendor_bundle_startup_steps, validate_vendor_bundle_lock
from ruikang_recon_baseline.common import load_waypoints

ROOT = Path(__file__).resolve().parents[1]


class ManagedVendorBundleManifestTest(unittest.TestCase):
    def test_load_managed_vendor_bundle_manifest(self):
        manifest = load_vendor_bundle_manifest(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'))
        assert manifest is not None
        self.assertEqual(manifest.bundle_id, 'mowen_mo_sergeant_managed_bundle')
        self.assertIn('navigation_launch', manifest.vendor_entrypoints)
        self.assertIn('repo_sidecar_launch', manifest.managed_entrypoints)
        self.assertIn('navigation_launch', manifest.startup_sequence)

    def test_validate_vendor_bundle_lock(self):
        manifest = load_vendor_bundle_manifest(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'))
        report = validate_vendor_bundle_lock(manifest, {
            'vendor_bundle_lock_id': 'mowen_mo_sergeant_managed_bundle',
            'vendor_bundle_lock_version': '1.0.0',
        })
        self.assertTrue(report['satisfied'])
        mismatch = validate_vendor_bundle_lock(manifest, {
            'vendor_bundle_lock_id': 'other',
            'vendor_bundle_lock_version': '1.0.0',
        })
        self.assertFalse(mismatch['satisfied'])


    def test_resolve_vendor_bundle_startup_steps(self):
        manifest = load_vendor_bundle_manifest(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'))
        assert manifest is not None
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace = Path(tmpdir) / 'newznzc_ws'
            (workspace / 'src').mkdir(parents=True, exist_ok=True)
            (workspace / 'devel').mkdir(parents=True, exist_ok=True)
            (workspace / 'devel' / 'setup.bash').write_text('stub', encoding='utf-8')
            (workspace / 'src' / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.0)', encoding='utf-8')
            for rel in [
                'car_bringup/scripts/newt.py',
                'nav_demo/launch/nav777.launch',
                'OrbbecSDK_ROS/dabai_dcw2.launch',
                'lslidar_driver/launch/lslidar_serial.launch',
                'wit_ros_imu/launch/wit_imu.launch',
            ]:
                target = workspace / 'src' / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('stub', encoding='utf-8')
            steps = resolve_vendor_bundle_startup_steps(manifest, workspace_root=str(workspace), repo_root=ROOT)
            self.assertEqual([step.name for step in steps][-1], 'repo_sidecar_launch')
            self.assertEqual(steps[0].kind, 'node')
            self.assertEqual(steps[1].kind, 'launch')
            self.assertEqual(steps[0].package, 'car_bringup')


    def test_preflight_accepts_catkin_source_space_layout(self):
        from ruikang_recon_baseline.vendor_bundle_preflight import build_vendor_bundle_preflight_report

        manifest = load_vendor_bundle_manifest(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'))
        assert manifest is not None
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace = Path(tmpdir) / 'newznzc_ws'
            (workspace / 'src').mkdir(parents=True, exist_ok=True)
            (workspace / 'devel').mkdir(parents=True, exist_ok=True)
            (workspace / 'devel' / 'setup.bash').write_text('stub', encoding='utf-8')
            (workspace / 'src' / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.0)', encoding='utf-8')
            for rel in manifest.vendor_entrypoints.values():
                target = workspace / 'src' / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('stub', encoding='utf-8')
            contract_summary = {
                'vendor_workspace_name': manifest.vendor_workspace_name,
                'vendor_entrypoints': dict(manifest.vendor_entrypoints),
                'managed_entrypoints': dict(manifest.managed_entrypoints),
            }
            report = build_vendor_bundle_preflight_report(contract_summary, {
                'vendor_runtime_mode': manifest.vendor_runtime_mode,
                'vendor_bundle_preflight_mode': 'required',
                'vendor_workspace_root': str(workspace),
                'vendor_bundle_manifest_path': str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'),
                'vendor_bundle_lock_id': manifest.bundle_id,
                'vendor_bundle_lock_version': manifest.bundle_version,
                'platform_adapter_type': manifest.platform_adapter_type,
            })
            self.assertTrue(report['satisfied'])
            self.assertEqual(report['status'], 'validated')


class FieldAssetReleaseManifestTest(unittest.TestCase):
    def test_reference_release_loads(self):
        release = load_field_asset_release(str(ROOT / 'config/field_assets/releases/mowen_raicom_reference_release.yaml'))
        assert release is not None
        self.assertEqual(release.verification_scope, 'reference')
        self.assertEqual(release.field_asset.asset_id, 'mowen_raicom_reference_field_verified')
        self.assertEqual(release.detector_model_id, 'mowen_reference_field_onnx_v1')


class MissionDslTest(unittest.TestCase):
    def test_competition_task_dsl_lowers_to_task_specs(self):
        route_payload = json.loads(json.dumps(__import__('yaml').safe_load((ROOT / 'config/field_assets/mowen_raicom_reference_field_verified.yaml').read_text(encoding='utf-8'))['route']))
        route = load_waypoints(route_payload, dwell_default_sec=4.0)
        tasks = load_task_graph_dsl(
            path=str(ROOT / 'config/task_graphs/mowen_reference_competition_tasks.yaml'),
            route=route,
            file_format='yaml',
        )
        self.assertEqual(tasks[0]['step_id'], 'recon_zone_a')
        self.assertEqual(tasks[2]['task_type'], 'hazard_avoid')
        self.assertEqual(tasks[3]['metadata']['attack_mode'], 'command_confirmed')


class RuntimeGraphExpectationTest(unittest.TestCase):
    def test_build_runtime_graph_expectations(self):
        payload = build_runtime_graph_expectations(
            enable_vision=True,
            enable_mission=True,
            enable_recorder=True,
            enable_safety=True,
            enable_platform_bridge=True,
            mission_config={'move_base_action_name': 'move_base'},
            vision_config={'camera_topic': 'camera/rgb/image_raw'},
            platform_config={'command_input_topic': 'cmd_vel_raw', 'safety_output_topic': 'cmd_vel'},
        )
        self.assertIn('vision_counter_node', payload['expected_nodes'])
        self.assertIn('move_base', payload['expected_actions'])
        self.assertIn('cmd_vel_raw', payload['expected_topics'])


class AuthoritativeReplayManifestTest(unittest.TestCase):
    def test_build_and_load_authoritative_summary(self):
        summary = {
            'schema_version': '2.2.0',
            'class_names': ['friendly', 'enemy', 'hostage'],
            'zone_results_dynamic': {'zone_a': {'status': 'ok'}},
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            summary_path = Path(tmpdir) / 'final_summary_v2.json'
            summary_path.write_text(json.dumps(summary), encoding='utf-8')
            loaded = load_authoritative_summary(str(summary_path))
            metrics_path = Path(tmpdir) / 'runtime_metrics.json'
            metrics_path.write_text('{}', encoding='utf-8')
            manifest = build_authoritative_replay_manifest(
                loaded,
                summary_path=str(summary_path),
                output_root=tmpdir,
                runtime_metrics_path=str(metrics_path),
            )
            self.assertEqual(manifest['result_count'], 1)
            self.assertEqual(manifest['route_ids'], ['zone_a'])
            self.assertTrue(manifest['replay_artifacts']['runtime_metrics']['exists'])


class LogicalFacadeImportsTest(unittest.TestCase):
    def test_facade_packages_import(self):
        from ruikang_recon_baseline.mission import load_task_graph_dsl as mission_loader
        from ruikang_recon_baseline.ops import build_runtime_graph_expectations as graph_builder
        from ruikang_recon_baseline.platform import load_vendor_bundle_manifest as bundle_loader
        self.assertTrue(callable(mission_loader))
        self.assertTrue(callable(graph_builder))
        self.assertTrue(callable(bundle_loader))


if __name__ == '__main__':
    unittest.main()

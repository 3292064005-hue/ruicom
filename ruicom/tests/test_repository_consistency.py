import ast
import json
import xml.etree.ElementTree as ET
import unittest
from pathlib import Path

import yaml
from ruikang_recon_baseline.field_assets import apply_field_asset_to_mission_config, apply_field_asset_to_vision_config

ROOT = Path(__file__).resolve().parents[1]


def _load_include_arg_values(path: Path) -> dict:
    tree = ET.parse(path)
    include = tree.getroot().find('include')
    if include is None:
        raise AssertionError(f'{path.relative_to(ROOT)} must contain one include block')
    payload = {}
    for arg in include.findall('arg'):
        name = str(arg.attrib.get('name', '')).strip()
        if name:
            payload[name] = str(arg.attrib.get('value', '')).strip()
    return payload


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
            for token in [
                'config/common/vision.yaml',
                'config/common/mission.yaml',
                'config/common/recorder.yaml',
                'config/common/safety.yaml',
                'launch/core.launch',
            ]:
                if token in content:
                    self.assertTrue((ROOT / token).exists(), token)

    def test_launch_profile_files_exist(self):
        expected = [
            'config/profiles/baseline/vision.yaml',
            'config/profiles/baseline/mission.yaml',
            'config/profiles/baseline/recorder.yaml',
            'config/profiles/baseline/safety.yaml',
            'config/profiles/baseline_legacy/vision.yaml',
            'config/profiles/baseline_legacy/mission.yaml',
            'config/profiles/baseline_legacy/recorder.yaml',
            'config/profiles/baseline_legacy/safety.yaml',
            'config/profiles/baseline_contract/vision.yaml',
            'config/profiles/baseline_contract/mission.yaml',
            'config/profiles/baseline_contract/recorder.yaml',
            'config/profiles/baseline_contract/safety.yaml',
            'config/profiles/demo/synthetic.yaml',
            'config/profiles/demo/vision.yaml',
            'config/profiles/demo/mission.yaml',
            'config/profiles/demo/recorder.yaml',
            'config/profiles/demo/safety.yaml',
            'config/profiles/baseline_integration/synthetic.yaml',
            'config/profiles/baseline_integration/vision.yaml',
            'config/profiles/baseline_integration/mission.yaml',
            'config/profiles/baseline_integration/recorder.yaml',
            'config/profiles/baseline_integration/safety.yaml',
            'config/profiles/dynamic_integration/mission.yaml',
            'config/profiles/dynamic_integration/recorder.yaml',
        ]
        for rel_path in expected:
            with self.subTest(path=rel_path):
                self.assertTrue((ROOT / rel_path).exists(), rel_path)


    def test_baseline_deploy_launch_enables_system_manager(self):
        content = (ROOT / 'launch/deploy.launch').read_text(encoding='utf-8')
        self.assertIn('enable_system_manager', content)
        self.assertIn('profile_system_manager_config', content)

    def test_core_launch_passes_enabled_components_to_system_manager(self):
        content = (ROOT / 'launch/core.launch').read_text(encoding='utf-8')
        self.assertIn('enable_vision_component', content)
        self.assertIn('enable_mission_component', content)
        self.assertIn('enable_recorder_component', content)
        self.assertIn('enable_safety_component', content)
        self.assertIn('mission_recorder_node', content)
        self.assertGreaterEqual(content.count('name="lifecycle_managed" value="true"'), 4)
        self.assertGreaterEqual(content.count('control_command_topic'), 4)
        self.assertIn('enable_safety" default="true"', content)


    def test_core_launch_exposes_platform_bridge_toggle(self):
        content = (ROOT / 'launch/core.launch').read_text(encoding='utf-8')
        self.assertIn('enable_platform_bridge', content)
        self.assertIn('platform_bridge_node.py', content)

    def test_deploy_and_integration_launch_enable_platform_bridge_by_default(self):
        for rel_path in ['launch/deploy.launch', 'launch/integration.launch', 'launch/legacy.launch', 'launch/contract_deploy.launch']:
            content = (ROOT / rel_path).read_text(encoding='utf-8')
            with self.subTest(path=rel_path):
                self.assertIn('<arg name="enable_platform_bridge" default="true" />', content)


    def test_baseline_deploy_smoke_uses_dedicated_supervisor_fixture(self):
        content = (ROOT / 'tests/baseline_deploy_smoke.test').read_text(encoding='utf-8')
        self.assertIn('baseline_deploy_smoke.py', content)
        self.assertIn('enable_synthetic', content)
        self.assertIn('profile_synthetic_config', content)
        self.assertIn('enable_vision" value="true"', content)
        self.assertNotIn('type="baseline_integration_smoke.py"', content)

    def test_recorder_and_safety_configs_expose_profile_runtime_controls(self):
        recorder_common = yaml.safe_load((ROOT / 'config/common/recorder.yaml').read_text(encoding='utf-8'))
        safety_common = yaml.safe_load((ROOT / 'config/common/safety.yaml').read_text(encoding='utf-8'))
        baseline_recorder = yaml.safe_load((ROOT / 'config/profiles/baseline/recorder.yaml').read_text(encoding='utf-8'))
        baseline_safety = yaml.safe_load((ROOT / 'config/profiles/baseline/safety.yaml').read_text(encoding='utf-8'))
        self.assertEqual(recorder_common.get('profile_role'), 'integration')
        self.assertEqual(recorder_common.get('time_source_mode'), 'ros')
        self.assertFalse(recorder_common.get('lifecycle_managed', True))
        self.assertEqual(safety_common.get('profile_role'), 'integration')
        self.assertFalse(safety_common.get('lifecycle_managed', True))
        self.assertEqual(baseline_recorder.get('profile_role'), 'deploy')
        self.assertTrue(baseline_recorder.get('lifecycle_managed'))
        self.assertEqual(baseline_safety.get('profile_role'), 'deploy')
        self.assertTrue(baseline_safety.get('lifecycle_managed'))

    def test_vendor_sidecar_launch_exposes_feedback_source_contract_gate(self):
        content = (ROOT / 'launch/mowen_vendor_sidecar.launch').read_text(encoding='utf-8')
        self.assertIn('vendor_sidecar_contract_node.py', content)
        self.assertIn('native_feedback_source_declared', content)
        self.assertIn('require_explicit_feedback_source', content)
        self.assertIn('required="true"', content)

    def test_ci_runs_profile_contract_validator(self):
        content = (ROOT / '.github/workflows/ci.yml').read_text(encoding='utf-8')
        self.assertIn('Validate profile contracts', content)
        self.assertIn('python tools/validate_profile_contracts.py', content)

    def test_platform_profiles_define_bridge_metadata(self):
        for rel_path in [
            'config/common/platform.yaml',
            'config/profiles/baseline/platform.yaml',
            'config/profiles/baseline_integration/platform.yaml',
            'config/profiles/baseline_legacy/platform.yaml',
        ]:
            payload = yaml.safe_load((ROOT / rel_path).read_text(encoding='utf-8'))
            with self.subTest(path=rel_path):
                self.assertIn('enable_platform_bridge', payload)
                self.assertIn('platform_bridge_state_topic', payload)
                self.assertIn('control_mode_topic', payload)
                self.assertIn('estop_topic', payload)
                self.assertIn('navigation_status_topic', payload)
                self.assertIn('upstream_command_topic', payload)
                self.assertIn('command_input_topic', payload)
                self.assertIn('vendor_runtime_mode', payload)
                self.assertIn('vendor_workspace_ros_distro', payload)
                self.assertIn('vendor_workspace_python_major', payload)
                self.assertIn('motion_model', payload)
                self.assertIn('cmd_vel_semantics', payload)
                self.assertIn('allow_odom_feedback_fallback', payload)



    def test_baseline_deploy_aliases_contract_deploy(self):
        payload = _load_include_arg_values(ROOT / 'launch' / 'baseline_deploy.launch')
        self.assertEqual((ROOT / 'launch' / 'baseline_deploy.launch').read_text(encoding='utf-8').count('contract_deploy.launch'), 1)
        self.assertEqual(payload.get('required_field_asset_verification_scope'), 'contract')

    def test_semantic_deploy_launch_is_a_parameter_transparent_alias(self):
        payload = _load_include_arg_values(ROOT / 'launch' / 'semantic_deploy.launch')
        expected = {
            'namespace': '$(arg namespace)',
            'enable_synthetic': '$(arg enable_synthetic)',
            'enable_vision': '$(arg enable_vision)',
            'enable_mission': '$(arg enable_mission)',
            'enable_recorder': '$(arg enable_recorder)',
            'enable_safety': '$(arg enable_safety)',
            'enable_platform_bridge': '$(arg enable_platform_bridge)',
            'output_root': '$(arg output_root)',
            'profile_synthetic_config': '$(arg profile_synthetic_config)',
            'profile_platform_config': '$(arg profile_platform_config)',
            'profile_vision_config': '$(arg profile_vision_config)',
            'profile_mission_config': '$(arg profile_mission_config)',
            'profile_recorder_config': '$(arg profile_recorder_config)',
            'profile_safety_config': '$(arg profile_safety_config)',
            'profile_system_manager_config': '$(arg profile_system_manager_config)',
            'camera_topic': '$(arg camera_topic)',
            'detections_topic': '$(arg detections_topic)',
            'odom_topic': '$(arg odom_topic)',
            'amcl_pose_topic': '$(arg amcl_pose_topic)',
            'move_base_action_name': '$(arg move_base_action_name)',
            'field_asset_id': '$(arg field_asset_id)',
            'field_asset_path': '$(arg field_asset_path)',
            'field_asset_package_root': '$(arg field_asset_package_root)',
            'require_verified_field_asset': '$(arg require_verified_field_asset)',
            'required_field_asset_verification_scope': '$(arg required_field_asset_verification_scope)',
            'use_sim_time': '$(arg use_sim_time)',
            'model_manifest_path': '$(arg model_manifest_path)',
            'onnx_model_path': '$(arg onnx_model_path)',
            'vendor_runtime_contract_path': '$(arg vendor_runtime_contract_path)',
        }
        self.assertEqual(payload, expected)

    def test_mowen_vendor_runtime_launch_forwards_bundle_metadata_args(self):
        payload = _load_include_arg_values(ROOT / 'launch' / 'mowen_vendor_runtime.launch')
        self.assertEqual(payload.get('vendor_bundle_manifest_path'), '$(arg vendor_bundle_manifest_path)')
        self.assertEqual(payload.get('vendor_bundle_lock_id'), '$(arg vendor_bundle_lock_id)')
        self.assertEqual(payload.get('vendor_bundle_lock_version'), '$(arg vendor_bundle_lock_version)')

    def test_mowen_vendor_runtime_launch_forwards_feedback_source_contract_args(self):
        payload = _load_include_arg_values(ROOT / 'launch' / 'mowen_vendor_runtime.launch')
        self.assertEqual(payload.get('vendor_execution_feedback_topic'), '$(arg vendor_execution_feedback_topic)')
        self.assertEqual(payload.get('native_feedback_source_declared'), '$(arg native_feedback_source_declared)')
        self.assertEqual(payload.get('require_explicit_feedback_source'), '$(arg require_explicit_feedback_source)')

    def test_field_deploy_launch_is_compatibility_alias_of_real_field_deploy(self):
        payload = _load_include_arg_values(ROOT / 'launch' / 'field_deploy.launch')
        content = (ROOT / 'launch' / 'field_deploy.launch').read_text(encoding='utf-8')
        self.assertIn('launch/real_field_deploy.launch', content)
        for key in [
            'namespace', 'enable_vision', 'enable_mission', 'enable_recorder', 'enable_safety',
            'enable_platform_bridge', 'output_root', 'camera_topic', 'detections_topic',
            'field_asset_id', 'field_asset_path', 'field_asset_package_root', 'use_sim_time',
        ]:
            with self.subTest(key=key):
                self.assertEqual(payload.get(key), f'$(arg {key})')

    def test_real_field_deploy_launch_forces_verified_field_asset_gate(self):
        payload = _load_include_arg_values(ROOT / 'launch' / 'real_field_deploy.launch')
        self.assertEqual(payload.get('enable_synthetic'), 'false')
        self.assertEqual(payload.get('require_verified_field_asset'), 'true')
        self.assertEqual(payload.get('required_field_asset_verification_scope'), 'field')

    def test_xml_files_parse(self):
        xml_files = [
            ROOT / 'package.xml',
            *sorted((ROOT / 'launch').glob('*.launch')),
            ROOT / 'tests/baseline_profile_smoke.test',
            ROOT / 'tests/baseline_contract_smoke.test',
            ROOT / 'tests/baseline_integration_smoke.test',
            ROOT / 'tests/baseline_integration_namespace_smoke.test',
            ROOT / 'tests/dynamic_schema_integration_smoke.test',
            ROOT / 'tests/dynamic_schema_integration_namespace_smoke.test',
            ROOT / 'tests/dynamic_schema_mismatch_smoke.test',
            ROOT / 'tests/demo_profile_smoke.test',
            ROOT / 'tests/baseline_deploy_smoke.test',
            ROOT / 'tests/baseline_deploy_feedback_timeout_smoke.test',
        ]
        for path in xml_files:
            with self.subTest(path=path.relative_to(ROOT)):
                ET.parse(path)

    def test_mission_config_documents_navigation_contract_controls(self):
        content = (ROOT / 'config/common/mission.yaml').read_text(encoding='utf-8')
        for token in [
            'navigation_status_topic',
            'navigation_status_timeout_sec',
            'navigation_cancel_topic',
            'navigation_failure_quiesce_sec',
            'tf_target_frame',
            'tf_source_frame',
            'tf_lookup_timeout_sec',
            'navigation_planner_backend',
            'navigation_controller_backend',
            'navigation_recovery_backend',
            'navigation_localization_backend',
            'navigation_goal_transport',
            'navigation_status_transport',
            'navigation_cancel_transport',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_common_mission_default_disables_embedded_zone_results(self):
        payload = yaml.safe_load((ROOT / 'config/common/mission.yaml').read_text(encoding='utf-8'))
        self.assertFalse(payload.get('embed_zone_results_in_state', True))

    def test_common_configs_use_relative_topics_by_default(self):
        for rel_path in [
            'config/common/mission.yaml',
            'config/common/recorder.yaml',
            'config/common/safety.yaml',
            'config/common/synthetic.yaml',
            'config/common/vision.yaml',
        ]:
            payload = yaml.safe_load((ROOT / rel_path).read_text(encoding='utf-8'))
            with self.subTest(path=rel_path):
                for key, value in payload.items():
                    if key.endswith('_topic') or key.endswith('_name') and 'move_base' in key:
                        if isinstance(value, str) and value:
                            self.assertFalse(value.startswith('/'), '{}:{} should be relative by default'.format(rel_path, key))

    def test_launch_files_expose_use_sim_time(self):
        for rel_path in ['launch/core.launch', 'launch/baseline.launch', 'launch/baseline_legacy.launch', 'launch/baseline_contract.launch', 'launch/demo_synthetic.launch', 'launch/mowen_integration.launch']:
            content = (ROOT / rel_path).read_text(encoding='utf-8')
            with self.subTest(path=rel_path):
                self.assertIn('use_sim_time', content)

    def test_runtime_controls_and_profiles_are_documented(self):
        tokens = [
            'time_source_mode',
            'embed_zone_results_in_state',
            'snapshot_flush_hz',
            'recorder_authoritative_input',
            'health_emit_mode',
            'use_sim_time',
            'output_root_use_namespace',
            'detections_timeout_sec',
            'navigation_cancel_topic',
            'navigation_failure_quiesce_sec',
            'save_event_log',
            'event_log_interval_sec',
            'terminal_finalize_policy',
            'terminal_quiesce_sec',
            'health_typed_topic',
            'frame_region_counts_typed_topic',
            'zone_capture_dynamic_topic',
            'zone_capture_compatibility_mode',
            'control_command_topic',
            'baseline_legacy.launch',
            'config/profiles/baseline_legacy/',
            'tools/validate_profile_contracts.py',
            'baseline_contract.launch',
            'baseline_integration_smoke.test',
            'baseline_integration_namespace_smoke.test',
            'dynamic_schema_integration_smoke.test',
            'dynamic_schema_integration_namespace_smoke.test',
            'dynamic_schema_mismatch_smoke.test',
            'Dockerfile.noetic',
            'tools/run_noetic_verification.sh',
            'class_schema_hash',
            'route_id',
            'official_report_mode',
            'official_report_schema',
            'official_report_contract_path',
            'operator_interventions',
            'require_verified_field_asset',
            'required_field_asset_verification_scope',
            'vendor_runtime_mode',
            'motion_model',
            'cmd_vel_semantics',
            'allow_odom_feedback_fallback',
            'camera_ready_timeout_sec',
            'upstream_command_topic',
            'command_input_topic',
        ]
        content = (ROOT / 'README.md').read_text(encoding='utf-8')
        for token in tokens:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_mission_manager_uses_configured_dynamic_class_schema(self):
        tree = ast.parse((ROOT / 'src/ruikang_recon_baseline/mission_node.py').read_text(encoding='utf-8'))
        init_func = None
        for node in tree.body:
            if isinstance(node, ast.ClassDef) and node.name == 'MissionManagerNode':
                for item in node.body:
                    if isinstance(item, ast.FunctionDef) and item.name == '__init__':
                        init_func = item
                        break
        self.assertIsNotNone(init_func, 'MissionManagerNode.__init__ not found')
        assigns_class_names = False
        uses_class_names_in_policy = False
        for sub in ast.walk(init_func):
            if isinstance(sub, ast.Assign):
                for target in sub.targets:
                    if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == 'self' and target.attr == 'class_names':
                        assigns_class_names = True
            if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name) and sub.func.id == 'AggregationPolicy':
                for kw in sub.keywords:
                    if kw.arg == 'class_names' and isinstance(kw.value, ast.Attribute) and isinstance(kw.value.value, ast.Name) and kw.value.value.id == 'self' and kw.value.attr == 'class_names':
                        uses_class_names_in_policy = True
        self.assertTrue(assigns_class_names)
        self.assertTrue(uses_class_names_in_policy)

    def test_recorder_and_docs_expose_projection_only_compat_lane_and_per_source_health_lane(self):
        recorder_content = (ROOT / 'src/ruikang_recon_baseline/recorder_node.py').read_text(encoding='utf-8') + (ROOT / 'src/ruikang_recon_baseline/recorder_ingest.py').read_text(encoding='utf-8')
        for token in ['health_typed_seen_by_node', 'consistency_blocked', 'zone_capture_compatibility_mode', 'compat_shadow', 'recorder_diagnostics']:
            with self.subTest(token=token):
                self.assertIn(token, recorder_content)
        readme_content = (ROOT / 'README.md').read_text(encoding='utf-8')
        for token in [
            'typed/JSON 选择从“整条 health 流”收紧为“按 node 粒度”',
            'dynamic authoritative lane',
            'legacy `ZoneCapture.msg` / `final_summary.json` 退化为兼容投影',
            'zone_capture_compatibility_mode',
            'control_command_topic',
            '外部接口 / 观测接口',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, readme_content)

    def test_cmake_requires_rostest_and_installs_fake_integration_fixtures(self):
        content = (ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
        self.assertIn('find_package(rostest REQUIRED)', content)
        self.assertNotIn('find_package(rostest QUIET)', content)
        for token in [
            'scripts/fake_move_base_action_server.py',
            'scripts/fake_detection_array_publisher.py',
            'add_rostest(tests/baseline_integration_smoke.test)',
            'add_rostest(tests/dynamic_schema_integration_smoke.test)',
            'add_rostest(tests/dynamic_schema_integration_namespace_smoke.test)',
            'add_rostest(tests/dynamic_schema_mismatch_smoke.test)',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_baseline_contract_smoke_test_exposes_namespace_and_contract_topics(self):
        content = (ROOT / 'tests/baseline_contract_smoke.test').read_text(encoding='utf-8')
        for token in ['<arg name="namespace"', 'mission_state_typed_topic', 'zone_capture_dynamic_topic', 'summary_snapshot_topic', 'summary_snapshot_dynamic_topic']:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_baseline_integration_smoke_assets_exist(self):
        for rel_path in [
            'tests/baseline_integration_smoke.py',
            'tests/baseline_integration_smoke.test',
            'tests/dynamic_schema_integration_smoke.py',
            'tests/dynamic_schema_integration_smoke.test',
            'tests/dynamic_schema_integration_namespace_smoke.test',
            'tests/dynamic_schema_mismatch_smoke.py',
            'tests/dynamic_schema_mismatch_smoke.test',
            'scripts/fake_move_base_action_server.py',
            'scripts/fake_detection_array_publisher.py',
        ]:
            with self.subTest(path=rel_path):
                self.assertTrue((ROOT / rel_path).exists(), rel_path)

    def test_baseline_integration_fixtures_are_executable(self):
        for rel_path in [
            'tests/baseline_integration_smoke.py',
            'tests/dynamic_schema_integration_smoke.py',
            'tests/dynamic_schema_mismatch_smoke.py',
            'scripts/fake_move_base_action_server.py',
            'scripts/fake_detection_array_publisher.py',
            'tools/run_noetic_verification.sh',
        ]:
            with self.subTest(path=rel_path):
                self.assertTrue((ROOT / rel_path).stat().st_mode & 0o111, rel_path)

    def test_baseline_integration_smoke_checks_full_chain_and_dynamic_artifacts(self):
        content = (ROOT / 'tests/baseline_integration_smoke.py').read_text(encoding='utf-8')
        for token in [
            'DetectionArray',
            'zone_results_dynamic',
            'recorder_diagnostics',
            'consistency_blocked',
            'final_summary_v2.json',
            'expected_detector_type',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)
        launch_content = (ROOT / 'tests/baseline_integration_smoke.test').read_text(encoding='utf-8')
        for token in [
            'enable_synthetic" value="true',
            'enable_vision" value="true',
            'baseline_integration/synthetic.yaml',
            'expected_detector_type',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, launch_content)

    def test_dynamic_schema_integration_smoke_checks_extended_schema_runtime(self):
        content = (ROOT / 'tests/dynamic_schema_integration_smoke.py').read_text(encoding='utf-8')
        for token in ['neutral', 'totals_by_class', 'zone_results_dynamic', 'final_summary_v2.json', 'resolve_output_root']:
            with self.subTest(token=token):
                self.assertIn(token, content)
        launch_content = (ROOT / 'tests/dynamic_schema_integration_smoke.test').read_text(encoding='utf-8')
        for token in ['dynamic_integration/mission.yaml', 'dynamic_integration/recorder.yaml', 'neutral: 3', 'output_root_use_namespace']:
            with self.subTest(token=token):
                self.assertIn(token, launch_content)

    def test_dynamic_schema_mismatch_smoke_checks_fault_path(self):
        content = (ROOT / 'tests/dynamic_schema_mismatch_smoke.py').read_text(encoding='utf-8')
        for token in ['class_schema_mismatch', 'FAULT', 'received_class_names', 'expected_class_names']:
            with self.subTest(token=token):
                self.assertIn(token, content)
        launch_content = (ROOT / 'tests/dynamic_schema_mismatch_smoke.test').read_text(encoding='utf-8')
        for token in ['embedded_class_names', 'dynamic_schema_mismatch_fixture', 'health_typed_topic']:
            with self.subTest(token=token):
                self.assertIn(token, launch_content)

    def test_dynamic_schema_namespace_wrapper_exists(self):
        content = (ROOT / 'tests/dynamic_schema_integration_namespace_smoke.test').read_text(encoding='utf-8')
        for token in ['dynamic_schema_integration_smoke.test', 'namespace" value="robot1"']:
            with self.subTest(token=token):
                self.assertIn(token, content)



    def test_message_contracts_expose_route_identity_and_schema_hash_fields(self):
        expectations = {
            'msg/Detection.msg': ['string observed_position_type', 'string observed_position_label', 'float32 observed_position_x_m', 'float32 observed_position_y_m', 'string evidence_source'],
            'msg/DetectionArray.msg': ['string[] class_names', 'string class_schema_hash'],
            'msg/FrameRegionCounts.msg': ['string class_schema_hash'],
            'msg/MissionState.msg': ['string current_route_id', 'string next_route_id', 'string[] class_names', 'string class_schema_hash'],
            'msg/ZoneCapture.msg': ['string route_id', 'string class_schema_hash'],
            'msg/ZoneCaptureDynamic.msg': ['string route_id', 'string class_schema_hash'],
        }
        for rel_path, tokens in expectations.items():
            content = (ROOT / rel_path).read_text(encoding='utf-8')
            for token in tokens:
                with self.subTest(path=rel_path, token=token):
                    self.assertIn(token, content)

    def test_readme_documents_route_identity_schema_gate_and_preflight_controls(self):
        content = (ROOT / 'README.md').read_text(encoding='utf-8')
        for token in [
            'route_id',
            'current_route_id',
            'class_schema_mismatch_policy',
            'preflight_require_pose',
            'preflight_require_detections',
            'preflight_timeout_sec',
            'preflight_failure_policy',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_baseline_contract_profile_disables_detection_preflight_without_vision(self):
        payload = yaml.safe_load((ROOT / 'config/profiles/baseline_contract/mission.yaml').read_text(encoding='utf-8'))
        self.assertFalse(payload.get('preflight_require_detections', True))

    def test_common_mission_config_exposes_detections_timeout(self):
        payload = yaml.safe_load((ROOT / 'config/common/mission.yaml').read_text(encoding='utf-8'))
        self.assertIn('detections_timeout_sec', payload)
        self.assertGreater(float(payload['detections_timeout_sec']), 0.0)


    def test_cmake_installs_rostest_python_executables(self):
        content = (ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
        for token in [
            'tests/demo_pipeline_smoke.py',
            'tests/baseline_smoke.py',
            'tests/baseline_contract_smoke.py',
            'tests/baseline_deploy_smoke.py',
            'tests/baseline_deploy_feedback_timeout_smoke.py',
            'tests/baseline_integration_smoke.py',
            'tests/dynamic_schema_integration_smoke.py',
            'tests/dynamic_schema_mismatch_smoke.py',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)


    def test_profile_contract_validator_script_exists_and_is_executable(self):
        path = ROOT / 'tools/validate_profile_contracts.py'
        self.assertTrue(path.exists())
        self.assertTrue(path.stat().st_mode & 0o111)

    def test_release_validator_scripts_exist_and_are_executable(self):
        for rel_path in ['tools/validate_repository_hygiene.py', 'tools/validate_launch_contracts.py']:
            path = ROOT / rel_path
            with self.subTest(path=rel_path):
                self.assertTrue(path.exists())
                self.assertTrue(path.stat().st_mode & 0o111)


    def test_noetic_verification_workflow_exists_and_uses_repo_script(self):
        content = (ROOT / '.github/workflows/noetic-verification.yml').read_text(encoding='utf-8')
        self.assertIn('Dockerfile.noetic', content)
        self.assertIn('tools/run_noetic_verification.sh', content)
        self.assertIn('docker run --rm ruikang-noetic-verify', content)

    def test_noetic_verification_script_runs_static_validators_and_namespace_smokes(self):
        content = (ROOT / 'tools/run_noetic_verification.sh').read_text(encoding='utf-8')
        for token in [
            'tools/validate_repository_hygiene.py',
            'tools/validate_launch_contracts.py',
            'tests/baseline_integration_namespace_smoke.test',
            'tests/dynamic_schema_integration_namespace_smoke.test',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_vendor_runtime_launch_exists_and_exposes_external_launch_args(self):
        content = (ROOT / 'launch/mowen_vendor_runtime.launch').read_text(encoding='utf-8')
        for token in [
            'vendor_namespace',
            'vendor_ns_prefix',
            'vendor_bringup_launch',
            'vendor_navigation_launch',
            'vendor_camera_launch',
            'vendor_lidar_launch',
            'vendor_imu_launch',
            'upstream_navigation_status_topic',
            'field_asset_path',
            'mowen_vendor_sidecar.launch',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_vendor_runtime_launch_groups_vendor_includes_under_vendor_namespace(self):
        content = (ROOT / 'launch/mowen_vendor_runtime.launch').read_text(encoding='utf-8')
        self.assertIn('launch/mowen_vendor_sidecar.launch', content)
        self.assertIn('<arg name="vendor_namespace" value="$(arg vendor_namespace)" />', content)

    def test_vendor_runtime_bridge_defaults_follow_vendor_namespace_prefix(self):
        content = (ROOT / 'launch/mowen_vendor_runtime.launch').read_text(encoding='utf-8')
        self.assertIn('launch/mowen_vendor_sidecar.launch', content)
        sidecar = (ROOT / 'launch/mowen_vendor_sidecar.launch').read_text(encoding='utf-8')
        for token in [
            "upstream_feedback_topic\" default=\"$(eval arg('vendor_ns_prefix') + '/recon/platform/vendor/base_feedback_raw')",
            "upstream_odom_topic\" default=\"$(eval arg('vendor_ns_prefix') + '/odom')",
            "upstream_navigation_status_topic\" default=\"$(eval arg('vendor_ns_prefix') + '/move_base/status')",
            "upstream_command_topic\" default=\"$(eval arg('vendor_ns_prefix') + '/cmd_vel' if arg('vendor_ns_prefix') else 'recon/platform/vendor/cmd_vel')",
        ]:
            with self.subTest(token=token):
                self.assertIn(token, sidecar)

    def test_system_manager_requires_bridge_feedback_contract_for_readiness(self):
        payload = yaml.safe_load((ROOT / 'config/common/system_manager.yaml').read_text(encoding='utf-8'))
        self.assertEqual(
            payload.get('readiness_requirements', {}).get('platform_bridge_node'),
            ['command_path_bound', 'command_flow_contract_satisfied', 'feedback_contract_satisfied', 'platform_runtime_probe_satisfied', 'vendor_runtime_contract_satisfied', 'vendor_bundle_preflight_satisfied'],
        )


    def test_field_deploy_defaults_use_field_specific_platform_and_manager_profiles(self):
        content = (ROOT / 'launch/field_deploy.launch').read_text(encoding='utf-8')
        self.assertIn('config/profiles/field_deploy/platform.yaml', content)
        self.assertIn('config/profiles/field_deploy/system_manager.yaml', content)

    def test_platform_bridge_and_safety_topics_are_aligned(self):
        platform_payload = yaml.safe_load((ROOT / 'config/profiles/baseline/platform.yaml').read_text(encoding='utf-8'))
        safety_payload = yaml.safe_load((ROOT / 'config/common/safety.yaml').read_text(encoding='utf-8'))
        self.assertEqual(platform_payload.get('control_mode_topic'), safety_payload.get('control_mode_topic'))
        self.assertEqual(platform_payload.get('estop_topic'), safety_payload.get('estop_topic'))
        self.assertEqual(platform_payload.get('safety_output_topic'), safety_payload.get('output_topic'))

    def test_baseline_profile_now_matches_authoritative_integration_semantics(self):
        baseline_vision, baseline_asset = apply_field_asset_to_vision_config(
            yaml.safe_load((ROOT / 'config/profiles/baseline/vision.yaml').read_text(encoding='utf-8')),
            owner='baseline.vision',
        )
        baseline_mission, _ = apply_field_asset_to_mission_config(
            yaml.safe_load((ROOT / 'config/profiles/baseline/mission.yaml').read_text(encoding='utf-8')),
            owner='baseline.mission',
        )
        integration_vision, _ = apply_field_asset_to_vision_config(
            yaml.safe_load((ROOT / 'config/profiles/baseline_integration/vision.yaml').read_text(encoding='utf-8')),
            owner='baseline_integration.vision',
        )
        integration_mission, _ = apply_field_asset_to_mission_config(
            yaml.safe_load((ROOT / 'config/profiles/baseline_integration/mission.yaml').read_text(encoding='utf-8')),
            owner='baseline_integration.mission',
        )
        self.assertIsNotNone(baseline_asset)
        self.assertEqual(baseline_vision.get('named_regions'), integration_vision.get('named_regions'))
        self.assertEqual(baseline_mission.get('route'), integration_mission.get('route'))
        self.assertTrue(baseline_mission.get('require_route_frame_regions', False))
        self.assertEqual(set(baseline_vision.get('expected_region_names', [])), {'zone_a', 'zone_b', 'zone_c', 'zone_d'})

    def test_baseline_legacy_profile_preserves_unbound_route_behavior(self):
        legacy_vision = yaml.safe_load((ROOT / 'config/profiles/baseline_legacy/vision.yaml').read_text(encoding='utf-8'))
        legacy_mission = yaml.safe_load((ROOT / 'config/profiles/baseline_legacy/mission.yaml').read_text(encoding='utf-8'))
        self.assertEqual(legacy_vision.get('frame_region_adapter_type'), 'none')
        self.assertEqual(legacy_vision.get('field_asset_id'), 'mowen_raicom_default')
        self.assertEqual(legacy_mission.get('field_asset_id'), 'mowen_raicom_default')
        self.assertFalse(legacy_mission.get('require_route_frame_regions', True))

    def test_field_asset_backed_profiles_do_not_duplicate_inline_route_or_regions(self):
        expectations = {
            'config/profiles/baseline/vision.yaml': ['named_regions', 'expected_region_names'],
            'config/profiles/baseline/mission.yaml': ['route', 'expected_frame_regions'],
            'config/profiles/baseline_integration/vision.yaml': ['named_regions', 'expected_region_names'],
            'config/profiles/baseline_integration/mission.yaml': ['route', 'expected_frame_regions'],
            'config/profiles/baseline_contract/vision.yaml': ['named_regions', 'expected_region_names'],
            'config/profiles/baseline_contract/mission.yaml': ['route', 'expected_frame_regions'],
            'config/profiles/demo/vision.yaml': ['named_regions', 'expected_region_names'],
            'config/profiles/demo/mission.yaml': ['route', 'expected_frame_regions'],
        }
        for rel_path, forbidden in expectations.items():
            payload = yaml.safe_load((ROOT / rel_path).read_text(encoding='utf-8'))
            with self.subTest(path=rel_path):
                self.assertTrue(payload.get('field_asset_id'))
                for key in forbidden:
                    self.assertNotIn(key, payload)

    def test_baseline_integration_profiles_bind_frame_regions_semantically(self):
        vision_payload = yaml.safe_load((ROOT / 'config/profiles/baseline_integration/vision.yaml').read_text(encoding='utf-8'))
        mission_payload = yaml.safe_load((ROOT / 'config/profiles/baseline_integration/mission.yaml').read_text(encoding='utf-8'))
        field_asset = yaml.safe_load((ROOT / 'config/field_assets/mowen_raicom_default.yaml').read_text(encoding='utf-8'))
        self.assertEqual(vision_payload.get('frame_region_adapter_type'), 'named_regions')
        self.assertEqual(vision_payload.get('field_asset_id'), 'mowen_raicom_contract_verified')
        self.assertEqual(mission_payload.get('field_asset_id'), 'mowen_raicom_contract_verified')
        region_names = {item['name'] for item in field_asset.get('named_regions', [])}
        route_region_names = {item['frame_region'] for item in field_asset.get('route', [])}
        self.assertTrue(region_names)
        self.assertEqual(region_names, route_region_names)
        self.assertNotIn('', route_region_names)

    def test_integration_launches_expose_namespace_and_expected_output_root(self):
        for rel_path in [
            'tests/baseline_integration_smoke.test',
            'tests/dynamic_schema_integration_smoke.test',
            'tests/dynamic_schema_mismatch_smoke.test',
        ]:
            content = (ROOT / rel_path).read_text(encoding='utf-8')
            with self.subTest(path=rel_path):
                self.assertIn('<arg name="namespace"', content)
                self.assertIn('output_root_use_namespace', content)

    def test_baseline_launch_defaults_safety_off_for_mainline_runs(self):
        content = (ROOT / 'launch/baseline.launch').read_text(encoding='utf-8')
        self.assertIn('<arg name="enable_safety" default="false" />', content)

    def test_mission_manager_preflight_uses_dedicated_detection_timeout(self):
        tree = ast.parse((ROOT / 'src/ruikang_recon_baseline/mission_node.py').read_text(encoding='utf-8'))
        target = None
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == '_preflight_blockers':
                target = node
                break
        self.assertIsNotNone(target)
        uses_detection_timeout = False
        for sub in ast.walk(target):
            if isinstance(sub, ast.Constant) and sub.value == 'detections_timeout_sec':
                uses_detection_timeout = True
                break
        self.assertTrue(uses_detection_timeout)


    def test_readme_directory_tree_mentions_legacy_launch_and_validator(self):
        content = (ROOT / 'README.md').read_text(encoding='utf-8')
        for token in [
            'baseline_legacy/',
            'baseline_legacy.launch',
            'tools/',
            'validate_profile_contracts.py',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_implementation_summary_is_synchronized_with_feedback_guard_and_legacy_profile(self):
        content = (ROOT / 'IMPLEMENTATION_SUMMARY.md').read_text(encoding='utf-8')
        for token in [
            'baseline_legacy.launch',
            'detector_schema_policy',
            'validate_profile_contracts.py',
            'output_feedback_topic',
            '未真实环境验证',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)

    def test_baseline_compatibility_launch_uses_nonmanaged_integration_profiles(self):
        payload = _load_include_arg_values(ROOT / 'launch/integration.launch')
        self.assertEqual(payload.get('profile_vision_config'), '$(find ruikang_recon_baseline)/config/profiles/mowen_integration/vision.yaml')
        self.assertEqual(payload.get('profile_mission_config'), '$(find ruikang_recon_baseline)/config/profiles/mowen_integration/mission.yaml')
        self.assertEqual(payload.get('profile_recorder_config'), '$(find ruikang_recon_baseline)/config/profiles/mowen_integration/recorder.yaml')
        self.assertEqual(payload.get('profile_safety_config'), '$(find ruikang_recon_baseline)/config/profiles/mowen_integration/safety.yaml')
        self.assertEqual(payload.get('profile_platform_config'), '$(find ruikang_recon_baseline)/config/profiles/mowen_integration/platform.yaml')
        recorder_payload = yaml.safe_load((ROOT / 'config/profiles/baseline_integration/recorder.yaml').read_text(encoding='utf-8'))
        safety_payload = yaml.safe_load((ROOT / 'config/profiles/baseline_integration/safety.yaml').read_text(encoding='utf-8'))
        self.assertEqual(recorder_payload.get('profile_role'), 'integration')
        self.assertFalse(recorder_payload.get('lifecycle_managed', True))
        self.assertEqual(safety_payload.get('profile_role'), 'integration')
        self.assertFalse(safety_payload.get('lifecycle_managed', True))

    def test_baseline_integration_smoke_uses_nonmanaged_integration_recorder_and_safety_profiles(self):
        payload = _load_include_arg_values(ROOT / 'tests/baseline_integration_smoke.test')
        self.assertEqual(payload.get('profile_recorder_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_integration/recorder.yaml')
        self.assertEqual(payload.get('profile_safety_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_integration/safety.yaml')


    def test_mowen_integration_profile_alias_matches_legacy_baseline_integration(self):
        for relative in [
            'platform.yaml',
            'vision.yaml',
            'mission.yaml',
            'recorder.yaml',
            'safety.yaml',
            'synthetic.yaml',
        ]:
            canonical = (ROOT / 'config/profiles/mowen_integration' / relative).read_text(encoding='utf-8')
            legacy = (ROOT / 'config/profiles/baseline_integration' / relative).read_text(encoding='utf-8')
            with self.subTest(relative=relative):
                self.assertEqual(canonical, legacy)


    def test_baseline_legacy_launch_uses_legacy_profiles(self):
        payload = _load_include_arg_values(ROOT / 'launch/legacy.launch')
        self.assertEqual(payload.get('profile_vision_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_legacy/vision.yaml')
        self.assertEqual(payload.get('profile_mission_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_legacy/mission.yaml')
        self.assertEqual(payload.get('profile_recorder_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_legacy/recorder.yaml')
        self.assertEqual(payload.get('profile_safety_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_legacy/safety.yaml')

    def test_baseline_contract_launch_uses_contract_profiles(self):
        payload = _load_include_arg_values(ROOT / 'launch/contract.launch')
        self.assertEqual(payload.get('profile_vision_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_contract/vision.yaml')
        self.assertEqual(payload.get('profile_mission_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_contract/mission.yaml')
        self.assertEqual(payload.get('profile_recorder_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_contract/recorder.yaml')
        self.assertEqual(payload.get('profile_safety_config'), '$(find ruikang_recon_baseline)/config/profiles/baseline_contract/safety.yaml')

    def test_demo_synthetic_launch_uses_demo_profiles(self):
        payload = _load_include_arg_values(ROOT / 'launch/demo_synthetic.launch')
        self.assertEqual(payload.get('profile_synthetic_config'), '$(find ruikang_recon_baseline)/config/profiles/demo/synthetic.yaml')
        self.assertEqual(payload.get('profile_vision_config'), '$(find ruikang_recon_baseline)/config/profiles/demo/vision.yaml')
        self.assertEqual(payload.get('profile_mission_config'), '$(find ruikang_recon_baseline)/config/profiles/demo/mission.yaml')
        self.assertEqual(payload.get('profile_recorder_config'), '$(find ruikang_recon_baseline)/config/profiles/demo/recorder.yaml')
        self.assertEqual(payload.get('profile_safety_config'), '$(find ruikang_recon_baseline)/config/profiles/demo/safety.yaml')


    def test_baseline_profile_smoke_keeps_recorder_and_safety_enabled(self):
        payload = _load_include_arg_values(ROOT / 'tests/baseline_profile_smoke.test')
        include_file = (ROOT / 'tests/baseline_profile_smoke.test').read_text(encoding='utf-8')
        self.assertIn('launch/baseline.launch', include_file)
        self.assertEqual(payload.get('enable_recorder'), 'true')
        self.assertEqual(payload.get('enable_safety'), 'true')


    def test_mowen_vendor_runtime_launch_aligns_namespaced_task_layer_inputs(self):
        content = (ROOT / 'launch/mowen_vendor_runtime.launch').read_text(encoding='utf-8')
        self.assertIn('launch/mowen_vendor_sidecar.launch', content)
        for token in [
            '<arg name="odom_topic" value="$(arg odom_topic)" />',
            '<arg name="amcl_pose_topic" value="$(arg amcl_pose_topic)" />',
            '<arg name="move_base_action_name" value="$(arg move_base_action_name)" />',
            '<arg name="upstream_feedback_topic" value="$(arg upstream_feedback_topic)" />',
            '<arg name="upstream_navigation_status_topic" value="$(arg upstream_navigation_status_topic)" />',
            '<arg name="upstream_command_topic" value="$(arg upstream_command_topic)" />',
        ]:
            with self.subTest(forward=token):
                self.assertIn(token, content)
        sidecar = (ROOT / 'launch/mowen_vendor_sidecar.launch').read_text(encoding='utf-8')
        for token in [
            "<arg name=\"camera_topic\" default=\"$(eval arg('vendor_ns_prefix') + '/camera/rgb/image_raw' if arg('vendor_ns_prefix') else '/camera/rgb/image_raw')\" />",
            "<arg name=\"odom_topic\" default=\"$(eval arg('vendor_ns_prefix') + '/odom' if arg('vendor_ns_prefix') else 'odom')\" />",
            "<arg name=\"amcl_pose_topic\" default=\"$(eval arg('vendor_ns_prefix') + '/amcl_pose' if arg('vendor_ns_prefix') else 'amcl_pose')\" />",
            "<arg name=\"move_base_action_name\" default=\"$(eval arg('vendor_ns_prefix') + '/move_base' if arg('vendor_ns_prefix') else 'move_base')\" />",
            '<arg name="enable_platform_bridge" value="true" />',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, sidecar)




    def test_mowen_vendor_runtime_keeps_bridge_under_managed_reference_deploy(self):
        content = (ROOT / 'launch/mowen_vendor_runtime.launch').read_text(encoding='utf-8')
        self.assertIn('launch/mowen_vendor_sidecar.launch', content)
        sidecar = (ROOT / 'launch/mowen_vendor_sidecar.launch').read_text(encoding='utf-8')
        self.assertIn('<arg name="enable_platform_bridge" value="true" />', sidecar)
        self.assertIn('launch/reference_field_deploy.launch', sidecar)
        self.assertNotIn('<node pkg="ruikang_recon_baseline" type="platform_bridge_node.py" name="platform_bridge_node"', content)

    def test_reference_field_deploy_launch_uses_reference_profiles(self):
        payload = _load_include_arg_values(ROOT / 'launch/reference_field_deploy.launch')
        self.assertEqual(payload.get('required_field_asset_verification_scope'), 'reference')
        content = (ROOT / 'launch/reference_field_deploy.launch').read_text(encoding='utf-8')
        self.assertIn('config/profiles/reference_deploy/vision.yaml', content)
        self.assertIn('config/profiles/reference_deploy/mission.yaml', content)

    def test_field_deploy_launch_uses_field_specific_profiles_and_explicit_overrides(self):
        payload = _load_include_arg_values(ROOT / 'launch/field_deploy.launch')
        self.assertEqual(payload.get('profile_vision_config'), '$(arg profile_vision_config)')
        self.assertEqual(payload.get('profile_mission_config'), '$(arg profile_mission_config)')
        self.assertNotIn('required_field_asset_verification_scope', payload)
        self.assertEqual(payload.get('model_manifest_path'), '$(arg model_manifest_path)')
        content = (ROOT / 'launch/field_deploy.launch').read_text(encoding='utf-8')
        self.assertIn('config/profiles/field_deploy/vision.yaml', content)
        self.assertIn('config/profiles/field_deploy/mission.yaml', content)

    def test_reference_deploy_profiles_embed_reference_assets_and_manifest(self):
        mission_payload = yaml.safe_load((ROOT / 'config/profiles/reference_deploy/mission.yaml').read_text(encoding='utf-8'))
        vision_payload = yaml.safe_load((ROOT / 'config/profiles/reference_deploy/vision.yaml').read_text(encoding='utf-8'))
        self.assertEqual(mission_payload.get('field_asset_id', ''), 'mowen_raicom_reference_field_verified')
        self.assertEqual(vision_payload.get('field_asset_id', ''), 'mowen_raicom_reference_field_verified')
        self.assertEqual(vision_payload.get('model_manifest_path', ''), 'config/manifests/mowen_reference_field_detector_manifest.json')
        self.assertEqual(mission_payload.get('model_manifest_path', ''), 'config/manifests/mowen_reference_field_detector_manifest.json')
        self.assertEqual(mission_payload.get('required_field_asset_verification_scope'), 'reference')
        self.assertEqual(vision_payload.get('required_field_asset_verification_scope'), 'reference')

    def test_deploy_profiles_define_navigation_backend_profile(self):
        for rel in ['config/profiles/baseline/mission.yaml', 'config/profiles/reference_deploy/mission.yaml', 'config/profiles/field_deploy/mission.yaml']:
            payload = yaml.safe_load((ROOT / rel).read_text(encoding='utf-8'))
            with self.subTest(path=rel):
                self.assertTrue(str(payload.get('navigation_backend_profile', '')).strip())

    def test_field_deploy_profiles_resolve_packaged_field_release(self):
        mission_payload = yaml.safe_load((ROOT / 'config/profiles/field_deploy/mission.yaml').read_text(encoding='utf-8'))
        vision_payload = yaml.safe_load((ROOT / 'config/profiles/field_deploy/vision.yaml').read_text(encoding='utf-8'))
        self.assertEqual(mission_payload.get('field_asset_id', ''), '')
        self.assertEqual(vision_payload.get('field_asset_id', ''), '')
        self.assertEqual(mission_payload.get('field_asset_release_manifest_path', ''), 'config/field_assets/releases/mowen_raicom_packaged_field_release.yaml')
        self.assertEqual(vision_payload.get('field_asset_release_manifest_path', ''), 'config/field_assets/releases/mowen_raicom_packaged_field_release.yaml')
        self.assertEqual(vision_payload.get('model_manifest_path', ''), 'config/manifests/mowen_packaged_field_detector_manifest.json')
        self.assertEqual(mission_payload.get('required_field_asset_verification_scope'), 'field')
        self.assertEqual(vision_payload.get('required_field_asset_verification_scope'), 'field')

    def test_vendor_profiles_bind_explicit_vendor_runtime_contract(self):
        for rel in [
            'config/profiles/baseline/platform.yaml',
            'config/profiles/baseline_integration/platform.yaml',
            'config/profiles/baseline_legacy/platform.yaml',
        ]:
            payload = yaml.safe_load((ROOT / rel).read_text(encoding='utf-8'))
            with self.subTest(path=rel):
                self.assertEqual(payload.get('vendor_runtime_contract_path'), 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml')

    def test_deploy_launch_exposes_profile_override_and_vendor_contract_args(self):
        content = (ROOT / 'launch/deploy.launch').read_text(encoding='utf-8')
        for token in [
            '<arg name="profile_platform_config"',
            '<arg name="profile_vision_config"',
            '<arg name="profile_mission_config"',
            '<arg name="model_manifest_path"',
            '<arg name="vendor_runtime_contract_path"',
        ]:
            with self.subTest(token=token):
                self.assertIn(token, content)



    def test_vendor_platform_profiles_gate_bundle_preflight_by_entry_role(self):
        expected_modes = {
            'config/profiles/baseline/platform.yaml': 'advisory',
            'config/profiles/baseline_legacy/platform.yaml': 'required',
            'config/profiles/mowen_integration/platform.yaml': 'required',
            'config/profiles/baseline_integration/platform.yaml': 'required',
            'config/profiles/reference_deploy/platform.yaml': 'required',
            'config/profiles/field_deploy/platform.yaml': 'required',
        }
        for rel, expected_mode in expected_modes.items():
            payload = yaml.safe_load((ROOT / rel).read_text(encoding='utf-8'))
            with self.subTest(path=rel):
                self.assertEqual(payload.get('vendor_bundle_preflight_mode'), expected_mode)

    def test_common_system_manager_readiness_consumes_vendor_bundle_preflight(self):
        payload = yaml.safe_load((ROOT / 'config/common/system_manager.yaml').read_text(encoding='utf-8'))
        for node_name in ('vision_counter_node', 'mission_manager_node', 'platform_bridge_node'):
            with self.subTest(node=node_name):
                self.assertIn('vendor_bundle_preflight_satisfied', payload['readiness_requirements'][node_name])

    def test_baseline_system_manager_profile_removes_vendor_bundle_gate_for_repository_smoke(self):
        payload = yaml.safe_load((ROOT / 'config/profiles/baseline/system_manager.yaml').read_text(encoding='utf-8'))
        readiness = payload.get('readiness_requirements', {})
        for node_name in ('vision_counter_node', 'mission_manager_node', 'platform_bridge_node'):
            with self.subTest(node=node_name):
                self.assertNotIn('vendor_bundle_preflight_satisfied', readiness[node_name])

    def test_example_judge_contract_exists(self):
        payload = json.loads((ROOT / 'config/judge_contracts/file_drop_example.json').read_text(encoding='utf-8'))
        self.assertEqual(payload.get('adapter_type'), 'file_drop')
        self.assertIn('endpoint_path', payload)




    def test_vendor_feedback_adapter_is_namespaced_with_deploy_runtime(self):
        launch_text = (ROOT / 'launch' / 'mowen_vendor_sidecar.launch').read_text(encoding='utf-8')
        self.assertIn('name="vendor_feedback_node" ns="$(arg namespace)"', launch_text)
if __name__ == '__main__':
    unittest.main()

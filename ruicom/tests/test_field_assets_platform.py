import unittest
import tempfile
from pathlib import Path

import yaml

from ruikang_recon_baseline.domain_models import ConfigurationError
from ruikang_recon_baseline.field_assets import apply_field_asset_to_mission_config, apply_field_asset_to_vision_config, load_field_asset
from ruikang_recon_baseline.platform_adapters import (
    PlatformAdapterRegistry,
    apply_platform_safety_defaults,
    apply_platform_vision_defaults,
)
from ruikang_recon_baseline.platform_bridge_core import PlatformBridgeSnapshot, evaluate_platform_bridge
from ruikang_recon_baseline.platform_contracts import (
    validate_platform_contract_bindings,
    validate_platform_runtime_interface_contract,
    validate_platform_runtime_strategy,
)
from ruikang_recon_baseline.common import detector_manifest_satisfies_scope, load_manifest
from ruikang_recon_baseline.runtime_probes import evaluate_navigation_runtime_probe, evaluate_platform_runtime_probe
from ruikang_recon_baseline.vendor_runtime_contracts import (
    build_vendor_runtime_binding_report,
    load_vendor_runtime_contract,
    validate_vendor_runtime_contract,
)
from ruikang_recon_baseline.vendor_bundle_preflight import build_vendor_bundle_preflight_report, enforce_vendor_bundle_preflight

ROOT = Path(__file__).resolve().parents[1]


class FieldAssetContractTest(unittest.TestCase):
    def test_mowen_field_asset_loads_route_and_regions(self):
        asset = load_field_asset(field_asset_id='mowen_raicom_default')
        self.assertEqual(asset.asset_id, 'mowen_raicom_default')
        self.assertEqual(asset.state, 'provisional')
        self.assertFalse(asset.verified)
        self.assertEqual(len(asset.route), 4)
        self.assertEqual(len(asset.named_regions), 4)
        self.assertEqual(set(asset.expected_region_names), {'zone_a', 'zone_b', 'zone_c', 'zone_d'})

    def test_contract_verified_asset_is_available_for_deploy(self):
        asset = load_field_asset(field_asset_id='mowen_raicom_contract_verified')
        self.assertTrue(asset.verified)
        self.assertEqual(asset.state, 'contract_verified')
        self.assertEqual(asset.verification_scope, 'contract')
        self.assertTrue(asset.satisfies_scope('contract'))
        self.assertFalse(asset.satisfies_scope('field'))

    def test_reference_field_asset_is_available_for_reference_deploy(self):
        asset = load_field_asset(field_asset_id='mowen_raicom_reference_field_verified')
        self.assertTrue(asset.verified)
        self.assertEqual(asset.verification_scope, 'reference')
        self.assertTrue(asset.satisfies_scope('contract'))
        self.assertTrue(asset.satisfies_scope('reference'))
        self.assertFalse(asset.satisfies_scope('field'))
        self.assertEqual(asset.metadata['detector_manifest_path'], 'config/manifests/mowen_reference_field_detector_manifest.json')

    def test_packaged_field_asset_is_available_for_field_deploy(self):
        asset = load_field_asset(field_asset_id='mowen_raicom_packaged_field_verified')
        self.assertTrue(asset.verified)
        self.assertEqual(asset.verification_scope, 'field')
        self.assertTrue(asset.satisfies_scope('field'))
        self.assertEqual(asset.metadata['detector_manifest_path'], 'config/manifests/mowen_packaged_field_detector_manifest.json')

    def test_mission_field_asset_resolves_route_bindings(self):
        payload = yaml.safe_load((ROOT / 'config/profiles/baseline/mission.yaml').read_text(encoding='utf-8'))
        resolved, asset = apply_field_asset_to_mission_config(payload, owner='unit.mission')
        self.assertIsNotNone(asset)
        self.assertEqual(resolved['field_asset_id'], 'mowen_raicom_contract_verified')
        self.assertTrue(resolved['field_asset_contract_satisfied'])
        self.assertEqual(len(resolved['route']), 4)
        self.assertEqual({item['frame_region'] for item in resolved['route']}, {'zone_a', 'zone_b', 'zone_c', 'zone_d'})

    def test_vision_field_asset_resolves_named_regions(self):
        payload = yaml.safe_load((ROOT / 'config/profiles/baseline/vision.yaml').read_text(encoding='utf-8'))
        resolved, asset = apply_field_asset_to_vision_config(payload, owner='unit.vision')
        self.assertIsNotNone(asset)
        self.assertEqual(resolved['field_asset_id'], 'mowen_raicom_contract_verified')
        self.assertTrue(resolved['field_asset_contract_satisfied'])
        self.assertEqual({item['name'] for item in resolved['named_regions']}, {'zone_a', 'zone_b', 'zone_c', 'zone_d'})

    def test_field_scope_gate_rejects_contract_only_asset(self):
        payload = {'field_asset_id': 'mowen_raicom_contract_verified', 'require_verified_field_asset': True, 'required_field_asset_verification_scope': 'field'}
        with self.assertRaises(ConfigurationError):
            apply_field_asset_to_mission_config(payload, owner='unit.scope')

    def test_field_asset_backed_profiles_do_not_duplicate_inline_truth(self):
        for rel_path, forbidden_keys in [
            ('config/profiles/baseline/mission.yaml', ['route', 'expected_frame_regions']),
            ('config/profiles/baseline/vision.yaml', ['named_regions', 'expected_region_names']),
            ('config/profiles/baseline_integration/mission.yaml', ['route', 'expected_frame_regions']),
            ('config/profiles/baseline_integration/vision.yaml', ['named_regions', 'expected_region_names']),
        ]:
            payload = yaml.safe_load((ROOT / rel_path).read_text(encoding='utf-8'))
            with self.subTest(path=rel_path):
                self.assertTrue(payload.get('field_asset_id'))
                for key in forbidden_keys:
                    self.assertNotIn(key, payload)

    def test_missing_required_field_asset_raises_for_mission(self):
        payload = {'require_verified_field_asset': True, 'required_field_asset_verification_scope': 'contract'}
        with self.assertRaises(ConfigurationError):
            apply_field_asset_to_mission_config(payload, package_root=str(ROOT / 'config' / 'field_assets'))

    def test_missing_required_field_asset_raises_for_vision(self):
        payload = {'require_verified_field_asset': True, 'required_field_asset_verification_scope': 'contract'}
        with self.assertRaises(ConfigurationError):
            apply_field_asset_to_vision_config(payload, package_root=str(ROOT / 'config' / 'field_assets'))

    def test_asset_manifest_alignment_injects_model_manifest_when_missing(self):
        payload = {'field_asset_id': 'mowen_raicom_reference_field_verified', 'model_manifest_path': ''}
        resolved, asset = apply_field_asset_to_vision_config(payload, owner='unit.vision.asset_manifest')
        self.assertIsNotNone(asset)
        self.assertEqual(resolved['model_manifest_path'], 'config/manifests/mowen_reference_field_detector_manifest.json')
        self.assertEqual(resolved['field_asset_detector_manifest_path'], 'config/manifests/mowen_reference_field_detector_manifest.json')

    def test_asset_manifest_alignment_rejects_conflicting_model_manifest(self):
        payload = {
            'field_asset_id': 'mowen_raicom_reference_field_verified',
            'model_manifest_path': 'config/manifests/baseline_deploy_detector_manifest.json',
        }
        with self.assertRaises(ConfigurationError):
            apply_field_asset_to_vision_config(payload, owner='unit.vision.asset_manifest_conflict')


class PlatformAdapterContractTest(unittest.TestCase):
    def test_registry_exposes_mowen_adapter(self):
        capability = PlatformAdapterRegistry().resolve('mowen_mo_sergeant')
        self.assertIn('move_base', capability.required_actions)
        self.assertIn('recon/platform/base_feedback', capability.required_topics)
        self.assertIn('cmd_vel_raw', capability.required_topics)
        self.assertIn('camera/rgb/image_raw', capability.required_topics)
        self.assertEqual(capability.shared_defaults['motion_model'], 'mecanum_holonomic')
        self.assertEqual(capability.shared_defaults['cmd_vel_semantics'], 'planar_xy_yaw')

    def test_deploy_safety_defaults_require_feedback_for_mowen_adapter(self):
        config = {
            'platform_adapter_type': 'mowen_mo_sergeant',
            'profile_role': 'deploy',
            'output_feedback_topic': '',
            'require_output_feedback': False,
            'output_feedback_timeout_sec': 1.0,
        }
        capability = apply_platform_safety_defaults(config)
        self.assertEqual(capability.name, 'mowen_mo_sergeant')
        self.assertEqual(config['output_feedback_topic'], 'recon/platform/base_feedback')
        self.assertEqual(config['input_topic'], 'cmd_vel_raw')
        self.assertTrue(config['require_output_feedback'])
        self.assertFalse(config['allow_odom_feedback_fallback'])

    def test_vision_defaults_apply_platform_camera_topic(self):
        config = {
            'platform_adapter_type': 'mowen_mo_sergeant',
            'camera_topic': '',
        }
        capability = apply_platform_vision_defaults(config)
        self.assertEqual(capability.name, 'mowen_mo_sergeant')
        self.assertEqual(config['camera_topic'], '/camera/rgb/image_raw')


class PlatformBindingValidationTest(unittest.TestCase):
    def test_mission_contract_validation_rejects_missing_move_base_binding(self):
        capability = PlatformAdapterRegistry().resolve('generic_ros_nav')
        with self.assertRaises(ConfigurationError):
            validate_platform_contract_bindings(
                capability,
                {'amcl_pose_topic': 'amcl_pose', 'odom_topic': 'odom'},
                owner='unit.mission',
                domain='mission',
            )

    def test_vision_contract_validation_rejects_missing_camera_binding(self):
        capability = PlatformAdapterRegistry().resolve('mowen_mo_sergeant')
        with self.assertRaises(ConfigurationError):
            validate_platform_contract_bindings(capability, {'camera_topic': ''}, owner='unit.vision', domain='vision')

    def test_safety_contract_validation_rejects_missing_feedback_binding(self):
        capability = PlatformAdapterRegistry().resolve('mowen_mo_sergeant')
        with self.assertRaises(ConfigurationError):
            validate_platform_contract_bindings(capability, {'output_feedback_topic': ''}, owner='unit.safety', domain='safety')

    def test_mowen_safety_contract_accepts_required_bindings(self):
        capability = PlatformAdapterRegistry().resolve('mowen_mo_sergeant')
        summary = validate_platform_contract_bindings(
            capability,
            {
                'amcl_pose_topic': 'amcl_pose',
                'odom_topic': 'odom',
                'move_base_action_name': 'move_base',
                'output_feedback_topic': 'recon/platform/base_feedback',
                'command_input_topic': 'cmd_vel_raw',
                'estop_topic': 'recon/estop',
                'control_mode_topic': 'recon/control_mode',
            },
            owner='unit.safety',
            domain='safety',
        )
        self.assertIn('recon/platform/base_feedback', summary['bound_topics'])
        self.assertIn('move_base', summary['bound_actions'])

    def test_runtime_interface_contract_requires_evidence_and_control_topics(self):
        summary = validate_platform_runtime_interface_contract(
            {
                'upstream_command_topic': 'recon/platform/vendor/cmd_vel',
                'command_input_topic': 'cmd_vel_raw',
                'safety_output_topic': 'cmd_vel',
                'output_feedback_topic': 'recon/platform/base_feedback',
                'control_mode_topic': 'recon/control_mode',
                'estop_topic': 'recon/estop',
                'runtime_evidence_topic': 'recon/runtime/evidence',
            },
            owner='unit.platform_runtime_interface',
        )
        self.assertTrue(summary['satisfied'])
        self.assertEqual(summary['bound_topics']['command_input_topic'], 'cmd_vel_raw')

    def test_runtime_interface_contract_rejects_missing_evidence_topic(self):
        with self.assertRaises(ConfigurationError):
            validate_platform_runtime_interface_contract(
                {
                    'upstream_command_topic': 'recon/platform/vendor/cmd_vel',
                    'command_input_topic': 'cmd_vel_raw',
                    'safety_output_topic': 'cmd_vel',
                    'output_feedback_topic': 'recon/platform/base_feedback',
                    'control_mode_topic': 'recon/control_mode',
                    'estop_topic': 'recon/estop',
                    'runtime_evidence_topic': '',
                },
                owner='unit.platform_runtime_interface',
            )

    def test_runtime_strategy_rejects_invalid_motion_semantics_pair(self):
        capability = PlatformAdapterRegistry().resolve('mowen_mo_sergeant')
        with self.assertRaises(ConfigurationError):
            validate_platform_runtime_strategy(
                capability,
                {
                    'vendor_runtime_mode': 'isolated_legacy_workspace',
                    'vendor_workspace_ros_distro': 'melodic',
                    'vendor_workspace_python_major': 2,
                    'motion_model': 'mecanum_holonomic',
                    'cmd_vel_semantics': 'planar_x_yaw',
                    'allow_odom_feedback_fallback': False,
                },
                owner='unit.runtime',
            )

    def test_namespaced_mowen_bindings_satisfy_logical_contract(self):
        capability = PlatformAdapterRegistry().resolve('mowen_mo_sergeant')
        mission_summary = validate_platform_contract_bindings(
            capability,
            {
                'amcl_pose_topic': '/vendor/amcl_pose',
                'odom_topic': '/vendor/odom',
                'navigation_status_topic': '/vendor/recon/navigation_status',
                'move_base_action_name': '/vendor/move_base',
            },
            owner='unit.mission',
            domain='mission',
        )
        vision_summary = validate_platform_contract_bindings(
            capability,
            {'camera_topic': '/vendor/camera/rgb/image_raw'},
            owner='unit.vision',
            domain='vision',
        )
        self.assertIn('vendor/amcl_pose', mission_summary['bound_topics'])
        self.assertIn('vendor/move_base', mission_summary['bound_actions'])
        self.assertIn('vendor/camera/rgb/image_raw', vision_summary['bound_topics'])


class PlatformBridgeCoreTest(unittest.TestCase):
    def test_explicit_feedback_has_priority_when_fresh(self):
        decision = evaluate_platform_bridge(
            PlatformBridgeSnapshot(
                now_sec=10.0,
                feedback_timeout_sec=0.6,
                explicit_feedback_enabled=True,
                explicit_feedback_value=False,
                explicit_feedback_stamp_sec=9.8,
                odom_feedback_enabled=True,
                odom_stamp_sec=9.95,
                allow_odom_as_feedback=True,
                command_bridge_enabled=False,
                upstream_command_stamp_sec=0.0,
            )
        )
        self.assertFalse(decision.output_feedback)
        self.assertEqual(decision.source, 'explicit_feedback')

    def test_odom_heartbeat_falls_back_when_explicit_feedback_missing(self):
        decision = evaluate_platform_bridge(
            PlatformBridgeSnapshot(
                now_sec=10.0,
                feedback_timeout_sec=0.6,
                explicit_feedback_enabled=False,
                explicit_feedback_value=False,
                explicit_feedback_stamp_sec=0.0,
                odom_feedback_enabled=True,
                odom_stamp_sec=9.7,
                allow_odom_as_feedback=True,
                command_bridge_enabled=False,
                upstream_command_stamp_sec=0.0,
            )
        )
        self.assertTrue(decision.output_feedback)
        self.assertEqual(decision.source, 'odom_feedback_fallback')

    def test_stale_inputs_force_feedback_false(self):
        decision = evaluate_platform_bridge(
            PlatformBridgeSnapshot(
                now_sec=10.0,
                feedback_timeout_sec=0.6,
                explicit_feedback_enabled=True,
                explicit_feedback_value=True,
                explicit_feedback_stamp_sec=9.0,
                odom_feedback_enabled=True,
                odom_stamp_sec=9.0,
                allow_odom_as_feedback=False,
                command_bridge_enabled=False,
                upstream_command_stamp_sec=0.0,
            )
        )
        self.assertFalse(decision.output_feedback)
        self.assertEqual(decision.source, 'stale')


class VendorRuntimeContractEvidenceTest(unittest.TestCase):
    def test_vendor_runtime_contract_carries_entrypoints_and_binding_evidence(self):
        contract = load_vendor_runtime_contract(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml'))
        summary = validate_vendor_runtime_contract(
            contract,
            {
                'platform_adapter_type': 'mowen_mo_sergeant',
                'vendor_runtime_mode': 'isolated_legacy_workspace',
                'vendor_workspace_ros_distro': 'melodic',
                'vendor_workspace_python_major': 2,
                'move_base_action_name': 'move_base',
                'odom_topic': 'odom',
                'amcl_pose_topic': 'amcl_pose',
            },
            owner='unit.vendor',
            domain='mission',
        )
        report = build_vendor_runtime_binding_report(summary, {'move_base_action_name': 'move_base', 'odom_topic': 'odom', 'amcl_pose_topic': 'amcl_pose'})
        self.assertEqual(report['vendor_workspace_name'], 'newznzc_ws')
        self.assertIn('navigation_launch', report['vendor_entrypoints'])
        self.assertEqual(report['bindings'][0]['parameter'], 'move_base_action_name')
        self.assertTrue(report['bindings'][0]['vendor_entrypoint'])




class VendorBundlePreflightTest(unittest.TestCase):
    def test_required_preflight_surfaces_unresolved_external_bundle(self):
        contract = load_vendor_runtime_contract(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml'))
        summary = validate_vendor_runtime_contract(
            contract,
            {
                'platform_adapter_type': 'mowen_mo_sergeant',
                'vendor_runtime_mode': 'isolated_legacy_workspace',
                'vendor_workspace_ros_distro': 'melodic',
                'vendor_workspace_python_major': 2,
                'move_base_action_name': 'move_base',
                'odom_topic': 'odom',
                'amcl_pose_topic': 'amcl_pose',
                'vendor_bundle_preflight_mode': 'required',
            },
            owner='unit.vendor.preflight',
            domain='mission',
        )
        report = build_vendor_bundle_preflight_report(summary, {'vendor_runtime_mode': 'isolated_legacy_workspace', 'vendor_bundle_preflight_mode': 'required'})
        self.assertEqual(report['status'], 'external_bundle_unresolved')
        self.assertFalse(report['satisfied'])
        self.assertTrue(report['required'])

class RuntimeProbeSemanticStageTest(unittest.TestCase):
    def test_navigation_probe_surfaces_graph_and_semantic_stages(self):
        probe = evaluate_navigation_runtime_probe(
            {
                'navigation_goal_transport': 'actionlib',
                'navigation_status_transport': 'actionlib',
                'navigation_cancel_transport': 'actionlib',
                'navigation_localization_backend': 'amcl_pose_topic',
                'move_base_action_name': 'move_base',
                'amcl_pose_topic': 'amcl_pose',
            },
            published_topics=[
                ('move_base/status', 'actionlib_msgs/GoalStatusArray'),
                ('move_base/feedback', 'move_base_msgs/MoveBaseActionFeedback'),
                ('move_base/result', 'move_base_msgs/MoveBaseActionResult'),
                ('amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped'),
            ],
            action_server_ready=True,
            now_sec=10.0,
            runtime_live_stamp_sec=9.5,
            runtime_live_timeout_sec=3.0,
        )
        self.assertTrue(probe['graph_satisfied'])
        self.assertTrue(probe['semantic_satisfied'])
        self.assertTrue(probe['mission_ready'])

    def test_platform_probe_tracks_declared_command_contract_without_prestart_traffic(self):
        probe = evaluate_platform_runtime_probe(
            {
                'upstream_feedback_topic': 'recon/platform/vendor/base_feedback_raw',
                'upstream_feedback_topic_type': 'std_msgs/Bool',
                'upstream_odom_topic': 'odom',
                'upstream_odom_topic_type': 'nav_msgs/Odometry',
                'upstream_navigation_status_topic': 'move_base/status',
                'upstream_navigation_status_topic_type': 'actionlib_msgs/GoalStatusArray',
                'upstream_command_topic': 'recon/platform/vendor/cmd_vel',
                'upstream_command_topic_type': 'geometry_msgs/Twist',
            },
            published_topics=[
                ('recon/platform/vendor/base_feedback_raw', 'std_msgs/Bool'),
                ('odom', 'nav_msgs/Odometry'),
                ('move_base/status', 'actionlib_msgs/GoalStatusArray'),
            ],
        )
        self.assertTrue(probe['graph_satisfied'])
        self.assertTrue(probe['semantic_satisfied'])
        self.assertTrue(probe['command_declared'])

    def test_field_asset_id_resolves_from_repo_root_package_override(self):
        from pathlib import Path
        repo_root = Path(__file__).resolve().parents[1]
        asset = load_field_asset(field_asset_id='mowen_raicom_packaged_field_verified', package_root=str(repo_root))
        self.assertIsNotNone(asset)
        assert asset is not None
        self.assertEqual(asset.asset_id, 'mowen_raicom_packaged_field_verified')


if __name__ == '__main__':
    unittest.main()


class ManifestAndRuntimeProbeTest(unittest.TestCase):
    def test_contract_detector_manifest_matches_contract_scope(self):
        manifest = load_manifest(str(ROOT / 'config/manifests/baseline_deploy_detector_manifest.json'))
        self.assertTrue(detector_manifest_satisfies_scope(manifest, required_scope='contract'))
        self.assertFalse(detector_manifest_satisfies_scope(manifest, required_scope='field'))

    def test_reference_field_detector_manifest_matches_reference_scope(self):
        manifest = load_manifest(str(ROOT / 'config/manifests/mowen_reference_field_detector_manifest.json'))
        self.assertTrue(detector_manifest_satisfies_scope(manifest, required_scope='contract'))
        self.assertTrue(detector_manifest_satisfies_scope(manifest, required_scope='reference'))
        self.assertFalse(detector_manifest_satisfies_scope(manifest, required_scope='field'))

    def test_navigation_runtime_probe_requires_visible_localization_topic(self):
        probe = evaluate_navigation_runtime_probe(
            {
                'navigation_goal_transport': 'actionlib',
                'navigation_status_transport': 'actionlib',
                'navigation_cancel_transport': 'actionlib',
                'navigation_localization_backend': 'amcl_pose_topic',
                'move_base_action_name': 'move_base',
                'amcl_pose_topic': 'amcl_pose',
            },
            published_topics=[('move_base/status', 'actionlib_msgs/GoalStatusArray')],
            action_server_ready=True,
            now_sec=10.0,
            runtime_live_stamp_sec=9.5,
            runtime_live_timeout_sec=3.0,
        )
        self.assertFalse(probe['satisfied'])
        self.assertFalse(probe['amcl_pose_topic_visible'])

    def test_navigation_runtime_probe_requires_action_runtime_topics_for_actionlib(self):
        probe = evaluate_navigation_runtime_probe(
            {
                'navigation_goal_transport': 'actionlib',
                'navigation_status_transport': 'actionlib',
                'navigation_cancel_transport': 'actionlib',
                'navigation_localization_backend': 'odometry_topic',
                'move_base_action_name': 'move_base',
                'odom_topic': 'odom',
            },
            published_topics=[('odom', 'nav_msgs/Odometry')],
            action_server_ready=True,
            now_sec=10.0,
            runtime_live_stamp_sec=9.5,
            runtime_live_timeout_sec=3.0,
        )
        self.assertFalse(probe['action_runtime_topics_visible'])
        self.assertFalse(probe['satisfied'])

    def test_platform_runtime_probe_accepts_feedback_and_status_visibility(self):
        probe = evaluate_platform_runtime_probe(
            {
                'upstream_feedback_topic': 'recon/platform/vendor/base_feedback_raw',
                'upstream_odom_topic': 'odom',
                'upstream_navigation_status_topic': 'move_base/status',
            },
            published_topics=[
                ('recon/platform/vendor/base_feedback_raw', 'std_msgs/Bool'),
                ('move_base/status', 'actionlib_msgs/GoalStatusArray'),
            ],
        )
        self.assertTrue(probe['satisfied'])
        self.assertTrue(probe['feedback_observable'])
        self.assertTrue(probe['status_observable'])

    def test_runtime_probe_requires_exact_resolved_names_not_suffix_matches(self):
        nav_probe = evaluate_navigation_runtime_probe(
            {
                'navigation_goal_transport': 'actionlib',
                'navigation_status_transport': 'actionlib',
                'navigation_cancel_transport': 'actionlib',
                'navigation_localization_backend': 'amcl_pose_topic',
                'move_base_action_name': 'move_base',
                'amcl_pose_topic': 'amcl_pose',
            },
            published_topics=[
                ('robot_b/move_base/status', 'actionlib_msgs/GoalStatusArray'),
                ('robot_b/move_base/feedback', 'move_base_msgs/MoveBaseActionFeedback'),
                ('robot_b/move_base/result', 'move_base_msgs/MoveBaseActionResult'),
                ('robot_b/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped'),
            ],
            action_server_ready=True,
            now_sec=10.0,
            runtime_live_stamp_sec=9.5,
            runtime_live_timeout_sec=3.0,
        )
        self.assertFalse(nav_probe['graph_satisfied'])
        self.assertFalse(nav_probe['semantic_satisfied'])
        platform_probe = evaluate_platform_runtime_probe(
            {
                'upstream_feedback_topic': 'recon/platform/vendor/base_feedback_raw',
                'upstream_odom_topic': 'odom',
                'upstream_navigation_status_topic': 'move_base/status',
                'upstream_command_topic': 'recon/platform/vendor/cmd_vel',
            },
            published_topics=[
                ('robot_b/recon/platform/vendor/base_feedback_raw', 'std_msgs/Bool'),
                ('robot_b/odom', 'nav_msgs/Odometry'),
                ('robot_b/move_base/status', 'actionlib_msgs/GoalStatusArray'),
            ],
        )
        self.assertFalse(platform_probe['graph_satisfied'])
        self.assertFalse(platform_probe['semantic_satisfied'])


class VendorRuntimeContractTest(unittest.TestCase):
    def test_mowen_vendor_runtime_contract_loads(self):
        contract = load_vendor_runtime_contract(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml'))
        self.assertTrue(str(contract['contract_id']).startswith('mowen_mo_sergeant_vendor_runtime_v'))
        self.assertEqual(contract['vendor_runtime_mode'], 'isolated_legacy_workspace')
        self.assertIn('move_base_action_name', contract['bindings']['mission'])
        self.assertIn('camera_topic', contract['bindings']['vision'])
        self.assertIn('upstream_feedback_topic', contract['bindings']['bridge'])

    def test_vendor_runtime_contract_requires_declared_bridge_bindings(self):
        contract = load_vendor_runtime_contract(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml'))
        with self.assertRaises(ConfigurationError):
            validate_vendor_runtime_contract(
                contract,
                {
                    'platform_adapter_type': 'mowen_mo_sergeant',
                    'vendor_runtime_mode': 'isolated_legacy_workspace',
                    'vendor_workspace_ros_distro': 'melodic',
                    'vendor_workspace_python_major': 2,
                    'upstream_feedback_topic': '',
                    'upstream_odom_topic': 'odom',
                    'upstream_navigation_status_topic': 'move_base/status',
                    'upstream_command_topic': 'cmd_vel',
                    'vendor_runtime_contract_path': str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml'),
                },
                owner='unit.bridge',
                domain='bridge',
            )

    def test_navigation_runtime_probe_rejects_wrong_localization_topic_type(self):
        probe = evaluate_navigation_runtime_probe(
            {
                'navigation_goal_transport': 'actionlib',
                'navigation_status_transport': 'actionlib',
                'navigation_cancel_transport': 'actionlib',
                'navigation_localization_backend': 'amcl_pose_topic',
                'move_base_action_name': 'move_base',
                'amcl_pose_topic': 'amcl_pose',
            },
            published_topics=[
                ('move_base/status', 'actionlib_msgs/GoalStatusArray'),
                ('move_base/feedback', 'move_base_msgs/MoveBaseActionFeedback'),
                ('move_base/result', 'move_base_msgs/MoveBaseActionResult'),
                ('amcl_pose', 'nav_msgs/Odometry'),
            ],
            action_server_ready=True,
            now_sec=10.0,
            runtime_live_stamp_sec=9.5,
            runtime_live_timeout_sec=3.0,
        )
        self.assertTrue(probe['amcl_pose_topic_visible'])
        self.assertFalse(probe['amcl_pose_topic_type_ok'])
        self.assertFalse(probe['satisfied'])

    def test_platform_runtime_probe_rejects_wrong_feedback_topic_type(self):
        probe = evaluate_platform_runtime_probe(
            {
                'upstream_feedback_topic': 'recon/platform/vendor/base_feedback_raw',
                'upstream_odom_topic': 'odom',
                'upstream_navigation_status_topic': 'move_base/status',
            },
            published_topics=[
                ('recon/platform/vendor/base_feedback_raw', 'std_msgs/String'),
                ('move_base/status', 'actionlib_msgs/GoalStatusArray'),
            ],
        )
        self.assertTrue(probe['explicit_feedback_topic_visible'])
        self.assertFalse(probe['explicit_feedback_topic_type_ok'])
        self.assertFalse(probe['satisfied'])


class VendorBundlePreflightEnforcementTest(unittest.TestCase):
    def test_required_preflight_raises_when_bundle_is_unresolved(self):
        with self.assertRaises(ConfigurationError):
            enforce_vendor_bundle_preflight({
                'enabled': True,
                'required': True,
                'satisfied': False,
                'status': 'external_bundle_unresolved',
                'vendor_workspace_name': 'newznzc_ws',
                'vendor_workspace_root': '',
                'missing_entrypoints': [],
            }, owner='unit.vendor.required')

    def test_required_preflight_requires_workspace_markers(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            root_path = Path(tmpdir)
            workspace = root_path / 'newznzc_ws'
            workspace.mkdir()
            (workspace / 'src').mkdir()
            contract = load_vendor_runtime_contract(str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml'))
            summary = validate_vendor_runtime_contract(
                contract,
                {
                    'platform_adapter_type': 'mowen_mo_sergeant',
                    'vendor_runtime_mode': 'isolated_legacy_workspace',
                    'vendor_workspace_ros_distro': 'melodic',
                    'vendor_workspace_python_major': 2,
                    'move_base_action_name': 'move_base',
                    'odom_topic': 'odom',
                    'amcl_pose_topic': 'amcl_pose',
                    'vendor_bundle_preflight_mode': 'required',
                    'vendor_workspace_root': str(workspace),
                },
                owner='unit.vendor.markers',
                domain='mission',
            )
            report = build_vendor_bundle_preflight_report(summary, {
                'vendor_runtime_mode': 'isolated_legacy_workspace',
                'vendor_bundle_preflight_mode': 'required',
                'vendor_workspace_root': str(workspace),
            })
            self.assertEqual(report['status'], 'missing_entrypoints')
            self.assertFalse(report['satisfied'])


class NavigationRuntimeRoundtripProbeTest(unittest.TestCase):
    def test_action_runtime_probe_requires_feedback_or_result_after_dispatch(self):
        probe = evaluate_navigation_runtime_probe(
            {
                'navigation_goal_transport': 'actionlib',
                'navigation_status_transport': 'actionlib',
                'navigation_cancel_transport': 'actionlib',
                'navigation_localization_backend': 'amcl_pose_topic',
                'move_base_action_name': 'move_base',
                'amcl_pose_topic': 'amcl_pose',
            },
            published_topics=[
                ('move_base/status', 'actionlib_msgs/GoalStatusArray'),
                ('move_base/feedback', 'move_base_msgs/MoveBaseActionFeedback'),
                ('move_base/result', 'move_base_msgs/MoveBaseActionResult'),
                ('amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped'),
            ],
            action_server_ready=True,
            now_sec=10.0,
            runtime_live_stamp_sec=9.9,
            runtime_live_timeout_sec=3.0,
            feedback_live_stamp_sec=0.0,
            result_live_stamp_sec=0.0,
            dispatch_goal_stamp_sec=4.0,
            require_action_roundtrip=True,
        )
        self.assertFalse(probe['action_roundtrip_live'])
        self.assertFalse(probe['satisfied'])

    def test_action_runtime_probe_accepts_feedback_after_dispatch(self):
        probe = evaluate_navigation_runtime_probe(
            {
                'navigation_goal_transport': 'actionlib',
                'navigation_status_transport': 'actionlib',
                'navigation_cancel_transport': 'actionlib',
                'navigation_localization_backend': 'amcl_pose_topic',
                'move_base_action_name': 'move_base',
                'amcl_pose_topic': 'amcl_pose',
            },
            published_topics=[
                ('move_base/status', 'actionlib_msgs/GoalStatusArray'),
                ('move_base/feedback', 'move_base_msgs/MoveBaseActionFeedback'),
                ('move_base/result', 'move_base_msgs/MoveBaseActionResult'),
                ('amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped'),
            ],
            action_server_ready=True,
            now_sec=10.0,
            runtime_live_stamp_sec=9.9,
            runtime_live_timeout_sec=3.0,
            feedback_live_stamp_sec=9.7,
            result_live_stamp_sec=0.0,
            dispatch_goal_stamp_sec=9.5,
            require_action_roundtrip=True,
        )
        self.assertTrue(probe['action_roundtrip_observed'])
        self.assertTrue(probe['action_roundtrip_live'])
        self.assertTrue(probe['satisfied'])

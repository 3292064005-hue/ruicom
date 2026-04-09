#!/usr/bin/env python3
"""Static validator for profile files and launch-composed runtime contracts.

The validator checks both profile-local semantics and the concrete launch-file to
profile wiring used by deploy, compatibility, rollback, contract and demo
entry points.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.common import (  # noqa: E402
    load_waypoints,
    validate_named_region_contract,
    validate_profile_runtime_flags,
    validate_route_frame_region_contract,
)
from ruikang_recon_baseline.field_assets import apply_field_asset_to_mission_config, apply_field_asset_to_vision_config  # noqa: E402
from ruikang_recon_baseline.navigation_contracts import (  # noqa: E402
    apply_navigation_contract_defaults,
    validate_navigation_contract_bindings,
    validate_navigation_runtime_strategy,
)
from ruikang_recon_baseline.platform_adapters import (  # noqa: E402
    PlatformAdapterRegistry,
    apply_platform_mission_defaults,
    apply_platform_safety_defaults,
    validate_platform_runtime_strategy,
)
from ruikang_recon_baseline.platform_contracts import validate_platform_contract_bindings  # noqa: E402


def load_yaml(path: Path):
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}




def load_launch_arg_values(path: Path) -> dict:
    """Return argument values declared inside the first include block of a launch file."""
    tree = ET.parse(path)
    include = tree.getroot().find('include')
    if include is None:
        raise RuntimeError(f'{path.relative_to(ROOT)} must contain one include block')
    payload = {}
    for arg in include.findall('arg'):
        name = str(arg.attrib.get('name', '')).strip()
        if name:
            payload[name] = str(arg.attrib.get('value', '')).strip()
    return payload


def assert_launch_profile_mapping(path: Path, expected: dict, *, owner: str) -> None:
    payload = load_launch_arg_values(path)
    for key, expected_value in expected.items():
        actual = payload.get(key, '')
        if actual != expected_value:
            raise RuntimeError(f'{owner} expected {key}={expected_value!r} but found {actual!r}')


def validate_launch_compositions() -> None:
    find_prefix = '$(find ruikang_recon_baseline)/'
    assert_launch_profile_mapping(
        ROOT / 'launch' / 'deploy.launch',
        {
            'profile_platform_config': '$(arg profile_platform_config)',
            'profile_vision_config': '$(arg profile_vision_config)',
            'profile_mission_config': '$(arg profile_mission_config)',
            'profile_recorder_config': '$(arg profile_recorder_config)',
            'profile_safety_config': '$(arg profile_safety_config)',
            'profile_system_manager_config': '$(arg profile_system_manager_config)',
            'model_manifest_path': '$(arg model_manifest_path)',
            'vendor_runtime_contract_path': '$(arg vendor_runtime_contract_path)',
        },
        owner='deploy.launch',
    )
    assert_launch_profile_mapping(
        ROOT / 'launch' / 'integration.launch',
        {
            'profile_platform_config': find_prefix + 'config/profiles/mowen_integration/platform.yaml',
            'profile_vision_config': find_prefix + 'config/profiles/mowen_integration/vision.yaml',
            'profile_mission_config': find_prefix + 'config/profiles/mowen_integration/mission.yaml',
            'profile_recorder_config': find_prefix + 'config/profiles/mowen_integration/recorder.yaml',
            'profile_safety_config': find_prefix + 'config/profiles/mowen_integration/safety.yaml',
        },
        owner='integration.launch',
    )
    assert_launch_profile_mapping(
        ROOT / 'launch' / 'contract.launch',
        {
            'profile_platform_config': find_prefix + 'config/profiles/baseline_contract/platform.yaml',
            'profile_vision_config': find_prefix + 'config/profiles/baseline_contract/vision.yaml',
            'profile_mission_config': find_prefix + 'config/profiles/baseline_contract/mission.yaml',
            'profile_recorder_config': find_prefix + 'config/profiles/baseline_contract/recorder.yaml',
            'profile_safety_config': find_prefix + 'config/profiles/baseline_contract/safety.yaml',
        },
        owner='contract.launch',
    )
    assert_launch_profile_mapping(
        ROOT / 'launch' / 'legacy.launch',
        {
            'profile_platform_config': find_prefix + 'config/profiles/baseline_legacy/platform.yaml',
            'profile_vision_config': find_prefix + 'config/profiles/baseline_legacy/vision.yaml',
            'profile_mission_config': find_prefix + 'config/profiles/baseline_legacy/mission.yaml',
            'profile_recorder_config': find_prefix + 'config/profiles/baseline_legacy/recorder.yaml',
            'profile_safety_config': find_prefix + 'config/profiles/baseline_legacy/safety.yaml',
        },
        owner='legacy.launch',
    )
    assert_launch_profile_mapping(
        ROOT / 'launch' / 'demo_synthetic.launch',
        {
            'profile_platform_config': find_prefix + 'config/profiles/demo/platform.yaml',
            'profile_synthetic_config': find_prefix + 'config/profiles/demo/synthetic.yaml',
            'profile_vision_config': find_prefix + 'config/profiles/demo/vision.yaml',
            'profile_mission_config': find_prefix + 'config/profiles/demo/mission.yaml',
            'profile_recorder_config': find_prefix + 'config/profiles/demo/recorder.yaml',
            'profile_safety_config': find_prefix + 'config/profiles/demo/safety.yaml',
        },
        owner='demo_synthetic.launch',
    )
    assert_launch_profile_mapping(
        ROOT / 'launch' / 'field_deploy.launch',
        {
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
            'field_asset_id': '$(arg field_asset_id)',
            'field_asset_path': '$(arg field_asset_path)',
            'field_asset_package_root': '$(arg field_asset_package_root)',
            'require_verified_field_asset': 'true',
            'required_field_asset_verification_scope': 'field',
            'use_sim_time': '$(arg use_sim_time)',
            'model_manifest_path': '$(arg model_manifest_path)',
            'vendor_runtime_contract_path': '$(arg vendor_runtime_contract_path)',
        },
        owner='field_deploy.launch',
    )
    assert_launch_profile_mapping(
        ROOT / 'launch' / 'semantic_deploy.launch',
        {
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
            'field_asset_id': '$(arg field_asset_id)',
            'field_asset_path': '$(arg field_asset_path)',
            'field_asset_package_root': '$(arg field_asset_package_root)',
            'require_verified_field_asset': '$(arg require_verified_field_asset)',
            'required_field_asset_verification_scope': '$(arg required_field_asset_verification_scope)',
            'use_sim_time': '$(arg use_sim_time)',
            'model_manifest_path': '$(arg model_manifest_path)',
            'vendor_runtime_contract_path': '$(arg vendor_runtime_contract_path)',
        },
        owner='semantic_deploy.launch',
    )
    # legacy wrappers remain as migration aliases.
    for wrapper, target in [
        ('baseline.launch', 'integration.launch'),
        ('baseline_deploy.launch', 'deploy.launch'),
        ('baseline_contract.launch', 'contract.launch'),
        ('baseline_legacy.launch', 'legacy.launch'),
        ('mowen_integration.launch', 'integration.launch'),
    ]:
        content = (ROOT / 'launch' / wrapper).read_text(encoding='utf-8')
        if target not in content:
            raise RuntimeError(f'{wrapper} must include {target} as a compatibility alias')
    assert_launch_profile_mapping(
        ROOT / 'tests' / 'baseline_contract_smoke.test',
        {
            'namespace': '$(arg namespace)',
            'output_root': '$(arg output_root)',
        },
        owner='tests/baseline_contract_smoke.test',
    )
    assert_launch_profile_mapping(
        ROOT / 'tests' / 'demo_profile_smoke.test',
        {},
        owner='tests/demo_profile_smoke.test',
    )


def merged_profile_payload(profile_name: str, component: str) -> dict:
    common_payload = load_yaml(ROOT / 'config' / 'common' / '{}.yaml'.format(component))
    profile_dir = ROOT / 'config' / 'profiles' / profile_name
    profile_payload = load_yaml(profile_dir / '{}.yaml'.format(component)) if (profile_dir / '{}.yaml'.format(component)).exists() else {}
    payload = dict(common_payload)
    payload.update(profile_payload)
    return payload


def assert_field_asset_is_authoritative(payload: dict, *, asset_backed_keys: list[str], owner: str) -> None:
    if not str(payload.get('field_asset_id', '')).strip() and not str(payload.get('field_asset_path', '')).strip():
        return
    for key in asset_backed_keys:
        value = payload.get(key)
        if value not in (None, '', [], {}):
            raise RuntimeError(f"{owner} must not duplicate inline {key} when field_asset_id/field_asset_path is set")


def validate_profile(profile_name: str, *, require_vision: bool = True, require_mission: bool = True) -> None:
    profile_dir = ROOT / 'config' / 'profiles' / profile_name
    vision_specific = load_yaml(profile_dir / 'vision.yaml') if (profile_dir / 'vision.yaml').exists() else {}
    mission_specific = load_yaml(profile_dir / 'mission.yaml') if (profile_dir / 'mission.yaml').exists() else {}
    if require_vision and not vision_specific:
        raise RuntimeError('profile {} is missing vision.yaml'.format(profile_name))
    if require_mission and not mission_specific:
        raise RuntimeError('profile {} is missing mission.yaml'.format(profile_name))
    platform_specific = load_yaml(profile_dir / 'platform.yaml') if (profile_dir / 'platform.yaml').exists() else {}
    if not platform_specific:
        raise RuntimeError('profile {} is missing platform.yaml'.format(profile_name))
    platform_payload = merged_profile_payload(profile_name, 'platform')
    capability_for_runtime = PlatformAdapterRegistry().resolve(platform_payload.get('platform_adapter_type', 'generic_ros_nav'))
    validate_platform_runtime_strategy(capability_for_runtime, platform_payload, owner='{}.platform'.format(profile_name))
    vision_payload = merged_profile_payload(profile_name, 'vision')
    mission_payload = merged_profile_payload(profile_name, 'mission')
    recorder_payload = merged_profile_payload(profile_name, 'recorder')
    safety_payload = merged_profile_payload(profile_name, 'safety')
    if vision_specific:
        assert_field_asset_is_authoritative(vision_specific, asset_backed_keys=['named_regions', 'expected_region_names'], owner='{}.vision'.format(profile_name))
    if mission_specific:
        assert_field_asset_is_authoritative(mission_specific, asset_backed_keys=['route', 'expected_frame_regions'], owner='{}.mission'.format(profile_name))
    if vision_payload:
        vision_payload.update(platform_payload)
        capability = PlatformAdapterRegistry().resolve(vision_payload.get('platform_adapter_type', 'generic_ros_nav'))
        vision_payload, _ = apply_field_asset_to_vision_config(vision_payload, owner='{}.vision'.format(profile_name))
        validate_profile_runtime_flags(
            vision_payload.get('profile_role', 'integration'),
            owner='{}.vision'.format(profile_name),
            lifecycle_managed=bool(vision_payload.get('lifecycle_managed', False)),
        )
        validate_named_region_contract(
            vision_payload.get('named_regions', []),
            expected_region_names=vision_payload.get('expected_region_names', []),
            require_named_regions=str(vision_payload.get('frame_region_adapter_type', 'none')).strip().lower() == 'named_regions',
            owner='{}.vision'.format(profile_name),
        )
    if mission_payload:
        mission_payload.update(platform_payload)
        capability = apply_platform_mission_defaults(mission_payload)
        validate_platform_contract_bindings(capability, mission_payload, owner='{}.mission'.format(profile_name), domain='mission')
        mission_payload, _ = apply_field_asset_to_mission_config(mission_payload, owner='{}.mission'.format(profile_name))
        apply_navigation_contract_defaults(mission_payload)
        validate_navigation_contract_bindings(mission_payload, owner='{}.mission'.format(profile_name))
        validate_navigation_runtime_strategy(mission_payload, owner='{}.mission'.format(profile_name))
        validate_profile_runtime_flags(
            mission_payload.get('profile_role', 'integration'),
            owner='{}.mission'.format(profile_name),
            lifecycle_managed=bool(mission_payload.get('lifecycle_managed', False)),
            auto_start=bool(mission_payload.get('auto_start', True)),
            require_route_frame_regions=bool(mission_payload.get('require_route_frame_regions', False)),
        )
        route = load_waypoints(mission_payload.get('route', []), float(mission_payload.get('dwell_default_sec', 4.0)))
        validate_route_frame_region_contract(
            route,
            require_binding=bool(mission_payload.get('require_route_frame_regions', False)),
            allowed_frame_regions=mission_payload.get('expected_frame_regions', []),
            owner='{}.mission'.format(profile_name),
        )
    validate_profile_runtime_flags(
        recorder_payload.get('profile_role', 'integration'),
        owner='{}.recorder'.format(profile_name),
        lifecycle_managed=bool(recorder_payload.get('lifecycle_managed', False)),
    )
    safety_payload.update(platform_payload)
    capability = apply_platform_safety_defaults(safety_payload)
    validate_platform_contract_bindings(capability, safety_payload, owner='{}.safety'.format(profile_name), domain='safety')
    validate_profile_runtime_flags(
        safety_payload.get('profile_role', 'integration'),
        owner='{}.safety'.format(profile_name),
        lifecycle_managed=bool(safety_payload.get('lifecycle_managed', False)),
    )
    if vision_payload and mission_payload and vision_payload.get('expected_region_names') and mission_payload.get('expected_frame_regions'):
        left = set(vision_payload.get('expected_region_names', []))
        right = set(mission_payload.get('expected_frame_regions', []))
        if left != right:
            raise RuntimeError('profile {} expected region mismatch: vision={} mission={}'.format(profile_name, sorted(left), sorted(right)))


def validate_system_manager_profile(profile_name: str) -> None:
    profile_dir = ROOT / 'config' / 'profiles' / profile_name
    common_payload = load_yaml(ROOT / 'config' / 'common' / 'system_manager.yaml')
    profile_payload = load_yaml(profile_dir / 'system_manager.yaml') if (profile_dir / 'system_manager.yaml').exists() else {}
    if not profile_payload:
        raise RuntimeError('profile {} is missing system_manager.yaml'.format(profile_name))
    payload = dict(common_payload)
    payload.update(profile_payload)
    required_nodes = [str(item).strip() for item in payload.get('required_nodes', []) if str(item).strip()]
    if not required_nodes:
        raise RuntimeError('profile {} system_manager.required_nodes must not be empty'.format(profile_name))
    if 'mission_manager_node' not in required_nodes:
        raise RuntimeError('profile {} system_manager.required_nodes must include mission_manager_node'.format(profile_name))
    if 'vision_counter_node' not in required_nodes:
        raise RuntimeError('profile {} system_manager.required_nodes must include vision_counter_node'.format(profile_name))
    if float(payload.get('ready_timeout_sec', 0.0) or 0.0) <= 0.0:
        raise RuntimeError('profile {} system_manager.ready_timeout_sec must be > 0'.format(profile_name))
    if float(payload.get('health_freshness_sec', 0.0) or 0.0) <= 0.0:
        raise RuntimeError('profile {} system_manager.health_freshness_sec must be > 0'.format(profile_name))


def main() -> int:
    profiles = {
        'baseline': dict(require_vision=True, require_mission=True),
        'baseline_integration': dict(require_vision=True, require_mission=True),
        'mowen_integration': dict(require_vision=True, require_mission=True),
        'baseline_contract': dict(require_vision=True, require_mission=True),
        'baseline_legacy': dict(require_vision=True, require_mission=True),
        'demo': dict(require_vision=True, require_mission=True),
        'dynamic_integration': dict(require_vision=False, require_mission=True),
        'reference_deploy': dict(require_vision=True, require_mission=True),
    }
    for profile_name, kwargs in profiles.items():
        validate_profile(profile_name, **kwargs)
    baseline_vision, _ = apply_field_asset_to_vision_config(load_yaml(ROOT / 'config/profiles/baseline/vision.yaml'), owner='baseline.vision')
    baseline_mission, _ = apply_field_asset_to_mission_config(load_yaml(ROOT / 'config/profiles/baseline/mission.yaml'), owner='baseline.mission')
    integration_vision_file = ROOT / 'config/profiles/mowen_integration/vision.yaml'
    integration_mission_file = ROOT / 'config/profiles/mowen_integration/mission.yaml'
    integration_vision, _ = apply_field_asset_to_vision_config(load_yaml(integration_vision_file), owner='mowen_integration.vision')
    integration_mission, _ = apply_field_asset_to_mission_config(load_yaml(integration_mission_file), owner='mowen_integration.mission')
    if baseline_vision.get('field_asset_id', '') != integration_vision.get('field_asset_id', ''):
        raise RuntimeError('baseline vision field_asset_id drifted from mowen_integration authoritative asset')
    if baseline_mission.get('field_asset_id', '') != integration_mission.get('field_asset_id', ''):
        raise RuntimeError('baseline mission field_asset_id drifted from mowen_integration authoritative asset')
    for relative in ['platform.yaml', 'vision.yaml', 'mission.yaml', 'recorder.yaml', 'safety.yaml', 'synthetic.yaml']:
        legacy_alias = load_yaml(ROOT / 'config/profiles/baseline_integration' / relative)
        canonical_payload = load_yaml(ROOT / 'config/profiles/mowen_integration' / relative)
        if legacy_alias != canonical_payload:
            raise RuntimeError(f'baseline_integration alias drifted from mowen_integration canonical profile for {relative}')
    reference_deploy_vision = load_yaml(ROOT / 'config/profiles/reference_deploy/vision.yaml')
    reference_deploy_mission = load_yaml(ROOT / 'config/profiles/reference_deploy/mission.yaml')
    if reference_deploy_vision.get('field_asset_id', '') != 'mowen_raicom_reference_field_verified':
        raise RuntimeError('reference_deploy vision must use mowen_raicom_reference_field_verified asset')
    if reference_deploy_mission.get('field_asset_id', '') != 'mowen_raicom_reference_field_verified':
        raise RuntimeError('reference_deploy mission must use mowen_raicom_reference_field_verified asset')
    if reference_deploy_vision.get('model_manifest_path', '') != 'config/manifests/mowen_reference_field_detector_manifest.json':
        raise RuntimeError('reference_deploy vision must use config/manifests/mowen_reference_field_detector_manifest.json')
    if reference_deploy_mission.get('required_field_asset_verification_scope', '') != 'reference':
        raise RuntimeError('reference_deploy mission must require reference asset scope')
    if reference_deploy_vision.get('required_field_asset_verification_scope', '') != 'reference':
        raise RuntimeError('reference_deploy vision must require reference asset scope')

    field_deploy_vision = load_yaml(ROOT / 'config/profiles/field_deploy/vision.yaml')
    field_deploy_mission = load_yaml(ROOT / 'config/profiles/field_deploy/mission.yaml')
    if field_deploy_vision.get('field_asset_id', ''):
        raise RuntimeError('field_deploy vision must not embed a repository reference asset')
    if field_deploy_mission.get('field_asset_id', ''):
        raise RuntimeError('field_deploy mission must not embed a repository reference asset')
    if field_deploy_vision.get('model_manifest_path', ''):
        raise RuntimeError('field_deploy vision must not embed a reference detector manifest')
    if field_deploy_vision.get('required_field_asset_verification_scope', '') != 'field':
        raise RuntimeError('field_deploy vision must require field scope')
    if field_deploy_mission.get('required_field_asset_verification_scope', '') != 'field':
        raise RuntimeError('field_deploy mission must require field scope')
    for rel in [
        'config/profiles/baseline/platform.yaml',
        'config/profiles/baseline_integration/platform.yaml',
        'config/profiles/baseline_legacy/platform.yaml',
    ]:
        payload = load_yaml(ROOT / rel)
        contract_rel = str(payload.get('vendor_runtime_contract_path', '')).strip()
        if not contract_rel:
            raise RuntimeError(f'{rel} must declare vendor_runtime_contract_path')
        if not (ROOT / contract_rel).exists():
            raise RuntimeError(f'{rel} vendor_runtime_contract_path does not exist: {contract_rel}')
    baseline_platform = load_yaml(ROOT / 'config/profiles/baseline/platform.yaml')
    if baseline_platform.get('vendor_bundle_preflight_mode', '') != 'advisory':
        raise RuntimeError('baseline platform must keep vendor_bundle_preflight_mode=advisory so repository deploy smoke remains self-contained')
    baseline_system_manager = load_yaml(ROOT / 'config/profiles/baseline/system_manager.yaml')
    baseline_readiness = dict(baseline_system_manager.get('readiness_requirements', {}) or {})
    for node_name in ('vision_counter_node', 'mission_manager_node', 'platform_bridge_node'):
        values = [str(item).strip() for item in baseline_readiness.get(node_name, []) if str(item).strip()]
        if 'vendor_bundle_preflight_satisfied' in values:
            raise RuntimeError(f'baseline system_manager must not gate {node_name} on vendor_bundle_preflight_satisfied')

    if baseline_vision.get('field_asset_id', '') != 'mowen_raicom_contract_verified':
        raise RuntimeError('baseline deploy vision must use mowen_raicom_contract_verified asset')
    manifest_path = baseline_vision.get('model_manifest_path', '')
    if not manifest_path:
        raise RuntimeError('baseline deploy vision must define model_manifest_path')
    manifest_file = ROOT / manifest_path
    if not manifest_file.exists():
        raise RuntimeError('baseline deploy detector manifest does not exist: {}'.format(manifest_file))
    validate_system_manager_profile('baseline')
    validate_system_manager_profile('field_deploy')
    validate_system_manager_profile('reference_deploy')
    deploy_launch = (ROOT / 'launch' / 'deploy.launch').read_text(encoding='utf-8')
    if 'enable_system_manager' not in deploy_launch:
        raise RuntimeError('deploy.launch must enable the system manager')
    if 'enable_safety' not in deploy_launch:
        raise RuntimeError('deploy.launch must expose enable_safety controls')
    validate_launch_compositions()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

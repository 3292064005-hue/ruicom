#!/usr/bin/env python3
"""Managed vendor runtime launcher.

This wrapper resolves the managed vendor bundle manifest into one concrete
roslaunch invocation. It validates the external workspace, renders the startup
sequence into a temporary launch file, then sources the vendor workspace setup
script before executing that launch file.
"""
from __future__ import annotations

import argparse
import atexit
import os
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path
from xml.sax.saxutils import quoteattr

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ruikang_recon_baseline.vendor_bundle_manifest import (  # noqa: E402
    load_vendor_bundle_manifest,
    resolve_vendor_bundle_startup_steps,
)

_SIDE_CAR_ARG_SPECS = [
    ('namespace', ''),
    ('vendor_namespace', ''),
    ('deploy_grade', 'field'),
    ('output_root', ''),
    ('detections_topic', ''),
    ('camera_topic', '/camera/rgb/image_raw'),
    ('use_sim_time', 'false'),
    ('field_asset_id', ''),
    ('field_asset_path', ''),
    ('field_asset_package_root', ''),
    ('model_manifest_path', ''),
    ('onnx_model_path', ''),
    ('vendor_runtime_contract_path', str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml')),
    ('vendor_execution_feedback_topic', ''),
    ('native_feedback_source_declared', 'false'),
    ('require_explicit_feedback_source', 'true'),
    ('vendor_feedback_command_topic', 'recon/platform/vendor/cmd_vel'),
    ('vendor_feedback_timeout_sec', '0.6'),
    ('vendor_feedback_command_timeout_sec', '1.0'),
    ('require_recent_vendor_command_for_feedback', 'true'),
    ('upstream_feedback_topic', 'recon/platform/vendor/base_feedback_raw'),
    ('odom_topic', 'odom'),
    ('amcl_pose_topic', 'amcl_pose'),
    ('move_base_action_name', 'move_base'),
    ('upstream_odom_topic', 'odom'),
    ('upstream_control_mode_topic', ''),
    ('upstream_estop_topic', ''),
    ('upstream_navigation_status_topic', 'move_base/status'),
    ('upstream_command_topic', 'recon/platform/vendor/cmd_vel'),
    ('vendor_bundle_manifest_path', ''),
    ('vendor_bundle_lock_id', ''),
    ('vendor_bundle_lock_version', ''),
    ('vendor_bringup_launch', ''),
    ('vendor_navigation_launch', ''),
    ('vendor_camera_launch', ''),
    ('vendor_lidar_launch', ''),
    ('vendor_imu_launch', ''),
    ('enable_vendor_bringup', 'true'),
    ('enable_vendor_navigation', 'true'),
    ('enable_vendor_camera', 'true'),
    ('enable_vendor_lidar', 'true'),
    ('enable_vendor_imu', 'true'),
    ('vendor_camera_driver_launch', os.environ.get('MOWEN_CAMERA_DRIVER_LAUNCH', '')),
    ('vendor_lidar_driver_launch', os.environ.get('MOWEN_LIDAR_DRIVER_LAUNCH', '')),
    ('vendor_imu_driver_launch', os.environ.get('MOWEN_IMU_DRIVER_LAUNCH', '')),
    ('map_file', os.environ.get('MOWEN_NAV_MAP_FILE', '')),
    ('scan_topic', 'scan'),
    ('base_frame', 'base_footprint'),
    ('odom_frame', 'odom'),
    ('global_frame', 'map'),
    ('telemetry_topic', 'recon/platform/vendor/chassis_telemetry'),
    ('imu_topic', 'wit/imu'),
    ('allow_command_fallback', 'false'),
    ('serial_port', os.environ.get('MOWEN_CHASSIS_SERIAL_PORT', '/dev/carserial')),
    ('baudrate', os.environ.get('MOWEN_CHASSIS_BAUDRATE', '115200')),
    ('legacy_feedback_policy', 'telemetry_required'),
]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Launch the repository-managed vendor runtime bundle')
    parser.add_argument('--workspace-root', default=os.environ.get('MOWEN_VENDOR_WORKSPACE_ROOT', ''), help='External vendor workspace root')
    parser.add_argument('--bundle-manifest', default=str(ROOT / 'config/vendor_runtime/mowen_mo_sergeant_managed_bundle.yaml'), help='Vendor bundle manifest path')
    parser.add_argument('roslaunch_args', nargs=argparse.REMAINDER, help='Extra roslaunch arguments appended after --')
    return parser.parse_args()


def _external_include(step, sidecar_defaults):
    lines = [f'    <include file={quoteattr(step.absolute_path)}>']
    if step.name == 'chassis_bridge':
        mapping = {
            'serial_port': 'serial_port',
            'baudrate': 'baudrate',
            'cmd_vel_topic': 'upstream_command_topic',
            'feedback_topic': 'upstream_feedback_topic',
            'telemetry_topic': 'telemetry_topic',
            'legacy_feedback_policy': 'legacy_feedback_policy',
        }
        for target, source in mapping.items():
            lines.append(f'      <arg name={quoteattr(target)} value={quoteattr(f"$(arg {source})")} />')
    elif step.name == 'navigation_launch':
        mapping = {
            'namespace': 'namespace',
            'map_file': 'map_file',
            'scan_topic': 'scan_topic',
            'odom_topic': 'upstream_odom_topic',
            'cmd_vel_topic': 'upstream_command_topic',
            'base_frame': 'base_frame',
            'odom_frame': 'odom_frame',
            'global_frame': 'global_frame',
            'telemetry_topic': 'telemetry_topic',
            'imu_topic': 'imu_topic',
            'allow_command_fallback': 'allow_command_fallback',
            'use_rviz': None,
        }
        for target, source in mapping.items():
            if source is None:
                lines.append(f'      <arg name={quoteattr(target)} value="false" />')
            else:
                lines.append(f'      <arg name={quoteattr(target)} value={quoteattr(f"$(arg {source})")} />')
    elif step.name == 'camera_launch':
        lines.extend([
            f'      <arg name="driver_launch" value="$(arg vendor_camera_driver_launch)" />',
            f'      <arg name="camera_topic" value="$(arg camera_topic)" />',
            f'      <arg name="require_driver_launch" value="$(eval arg(\'deploy_grade\') == \'field\')" />',
        ])
    elif step.name == 'lidar_launch':
        lines.extend([
            f'      <arg name="driver_launch" value="$(arg vendor_lidar_driver_launch)" />',
            f'      <arg name="scan_topic" value="$(arg scan_topic)" />',
            f'      <arg name="require_driver_launch" value="$(eval arg(\'deploy_grade\') == \'field\')" />',
        ])
    elif step.name == 'imu_launch':
        lines.extend([
            f'      <arg name="driver_launch" value="$(arg vendor_imu_driver_launch)" />',
            f'      <arg name="imu_topic" value="$(arg imu_topic)" />',
            f'      <arg name="require_driver_launch" value="$(eval arg(\'deploy_grade\') == \'field\')" />',
        ])
    lines.append('    </include>')
    return lines


def _build_launch_lines(steps) -> list[str]:
    lines = ['<launch>']
    sidecar_defaults = dict(_SIDE_CAR_ARG_SPECS)
    external_step_to_flag = {
        'chassis_bridge': ('enable_vendor_bringup', 'vendor_bringup_launch'),
        'navigation_launch': ('enable_vendor_navigation', 'vendor_navigation_launch'),
        'camera_launch': ('enable_vendor_camera', 'vendor_camera_launch'),
        'lidar_launch': ('enable_vendor_lidar', 'vendor_lidar_launch'),
        'imu_launch': ('enable_vendor_imu', 'vendor_imu_launch'),
    }
    for step in steps:
        override = external_step_to_flag.get(step.name)
        if override:
            flag_name, launch_name = override
            sidecar_defaults[flag_name] = 'false'
            if step.kind == 'launch':
                sidecar_defaults[launch_name] = step.absolute_path
    for name, _default in _SIDE_CAR_ARG_SPECS:
        lines.append(f'  <arg name={quoteattr(name)} default={quoteattr(str(sidecar_defaults[name]))} />')
    lines.append('')
    lines.append('  <group ns="$(arg vendor_namespace)">')
    sidecar_seen = False
    for step in steps:
        if step.name == 'repo_sidecar_launch':
            lines.append('  </group>')
            lines.append('')
            lines.append(f'  <include file={quoteattr(step.absolute_path)}>')
            for name, _ in _SIDE_CAR_ARG_SPECS:
                lines.append(f'    <arg name={quoteattr(name)} value={quoteattr(f"$(arg {name})")} />')
            lines.append('  </include>')
            sidecar_seen = True
            continue
        if step.source == 'managed':
            raise SystemExit(f'unsupported managed startup step in sequence: {step.name}')
        if step.kind == 'launch':
            lines.extend(_external_include(step, sidecar_defaults))
        elif step.kind == 'node':
            node_name = f'managed_{step.name}'.replace('-', '_')
            lines.append(
                f'    <node pkg={quoteattr(step.package)} type={quoteattr(step.executable)} '
                f'name={quoteattr(node_name)} output="screen" required="true">'
            )
            if step.name == 'chassis_bridge':
                mapping = {
                    'serial_port': 'serial_port',
                    'baudrate': 'baudrate',
                    'cmd_vel_topic': 'upstream_command_topic',
                    'feedback_topic': 'upstream_feedback_topic',
                    'telemetry_topic': 'telemetry_topic',
                    'legacy_feedback_policy': 'legacy_feedback_policy',
                }
                for target, source in mapping.items():
                    lines.append(f'      <param name={quoteattr(target)} value={quoteattr(f"$(arg {source})")} />')
            lines.append('    </node>')
        else:
            raise SystemExit(f'unsupported startup step kind {step.kind!r} for {step.name}')
    if not sidecar_seen:
        lines.append('  </group>')
        raise SystemExit('managed vendor bundle startup sequence must include repo_sidecar_launch')
    lines.append('</launch>')
    return lines


def _write_temp_launch(steps) -> Path:
    lines = _build_launch_lines(steps)
    handle = tempfile.NamedTemporaryFile('w', encoding='utf-8', prefix='ruikang_managed_vendor_', suffix='.launch', delete=False)
    handle.write('\n'.join(lines) + '\n')
    handle.flush()
    handle.close()
    path = Path(handle.name).resolve()
    atexit.register(lambda: path.exists() and path.unlink())
    return path


def main() -> int:
    args = _parse_args()
    manifest = load_vendor_bundle_manifest(args.bundle_manifest)
    if manifest is None:
        raise SystemExit('bundle manifest is required')
    workspace_root = str(args.workspace_root or '').strip()
    if not workspace_root:
        raise SystemExit('workspace root must be provided via --workspace-root or MOWEN_VENDOR_WORKSPACE_ROOT')
    workspace_path = Path(workspace_root).expanduser().resolve()
    if not workspace_path.exists():
        raise SystemExit(f'vendor workspace root does not exist: {workspace_path}')
    if manifest.source_setup_preference == 'install_first':
        setup_candidates = [workspace_path / 'install' / 'setup.bash', workspace_path / 'devel' / 'setup.bash']
    else:
        setup_candidates = [workspace_path / 'devel' / 'setup.bash', workspace_path / 'install' / 'setup.bash']
    setup_script = next((path for path in setup_candidates if path.exists()), None)
    if setup_script is None:
        raise SystemExit(f'workspace is missing setup.bash under devel/ or install/: {workspace_path}')
    steps = resolve_vendor_bundle_startup_steps(manifest, workspace_root=str(workspace_path), repo_root=ROOT)
    launch_file = _write_temp_launch(steps)
    extra = list(args.roslaunch_args)
    if extra[:1] == ['--']:
        extra = extra[1:]
    roslaunch_cmd = [
        'roslaunch',
        str(launch_file),
        f'vendor_bundle_manifest_path:={manifest.path}',
        f'vendor_bundle_lock_id:={manifest.bundle_id}',
        f'vendor_bundle_lock_version:={manifest.bundle_version}',
        *extra,
    ]
    shell_cmd = 'source {setup} && exec {launch}'.format(
        setup=shlex.quote(str(setup_script)),
        launch=' '.join(shlex.quote(item) for item in roslaunch_cmd),
    )
    return subprocess.call(['bash', '-lc', shell_cmd])


if __name__ == '__main__':
    raise SystemExit(main())

# MO-SERGEANT runtime tracks and ownership

## Runtime track split

`mowen_vendor_runtime.launch` and `mowen_vendor_sidecar.launch` now accept `deploy_grade`:

- `contract`: repository contract smoke and schema validation.
- `reference`: repository-managed reference-field validation.
- `field`: real-field deployment. This grade is guarded by `vendor_runtime_guard_node` and rejects repository-managed fixture launches.

For `field` grade, every enabled vendor seam must provide a concrete external launch file:

- `vendor_bringup_launch`
- `vendor_navigation_launch`
- `vendor_camera_launch`
- `vendor_lidar_launch`
- `vendor_imu_launch`

When the external vendor graph is already running, disable the corresponding seam explicitly instead of leaving the launch path empty.

## Chassis entrypoints

Two bottom-entry paths now coexist:

1. `scripts/mowen_serial_bridge_node.py`
   - repository authoritative ROS-side serial bridge;
   - supports `legacy_newt` and `managed_crc16`;
   - publishes canonical `recon/platform/vendor/base_feedback_raw` and `recon/platform/vendor/chassis_telemetry`.
2. `vendor_workspace/newznzc_ws/src/car_bringup/scripts/newt.py`
   - vendor-compatible `rosrun car_bringup newt.py` entry;
   - keeps the historical workflow for Melodic/Python 2 workspaces;
   - emits one explicit zero-velocity frame when commands go stale.

`legacy_feedback_policy` is now explicit on both bridges:

- `telemetry_required` (default): execution feedback is only true when managed telemetry is observed.
- `command_write`: transitional policy for legacy chassis firmware that cannot yet return telemetry.

## Navigation entrypoints

- `launch/field_real_navigation.launch` is the repository canonical real-navigation wrapper.
- `vendor_workspace/newznzc_ws/src/nav_demo/launch/nav777.launch` is now a real `map_server + amcl + move_base` launch instead of a fake action server wrapper.
- `MOWEN_NAV_MAP_FILE` can be used to provide the map yaml path without editing launch files.

## Sensor and mode ownership

The MO-SERGEANT profiles now declare these ownership topics explicitly:

- `upstream_ultrasonic_topic: ultrasonic/ranges`
- `upstream_imu_topic: wit/imu`
- `upstream_voice_command_topic: mowen/voice/command`
- `upstream_ps2_command_topic: mowen/ps2/command`
- `upstream_ps2_twist_topic: mowen/ps2/cmd_vel`

`COMMANDER` remains the default authoritative control mode for deploy profiles. `handle` and `voice` are treated as upstream operator overrides and must be wired through the bridge contract before use in field runs.
On the firmware side, the AT32 target now includes USART2 voice-command ingestion, UART4 ultrasonic frame ingestion, and a GPIO bit-banged PS2 receiver path. Those inputs participate in local mode arbitration (`COMMANDER` / `HANDLE` / `VOICE`) and the ultrasonic stop gate before wheel PID output is emitted.

## Firmware boundary

The repository firmware directory now contains both the stronger host-testable runtime core (`runtime.hpp`) and an AT32-targeted bare-metal build tree under `firmware/mowen_chassis_controller/at32/`, including startup, linker script, measured-telemetry runtime loop, board-level UART/PWM/encoder/ADC/watchdog porting, and the reproducible `tools/build_at32_firmware.sh` build path. Real hardware flashing and motor/encoder tuning still require on-board verification.

# MO-SERGEANT 外部 vendor 工作空间接入说明

## 2026-04-14 real-field updates

- `mowen_vendor_runtime.launch` / `mowen_vendor_sidecar.launch` now expose `deploy_grade={contract,reference,field}`.
- `field` grade is guarded by `vendor_runtime_guard_node.py` and rejects repository-managed fixture launches.
- `launch/field_real_navigation.launch` and `vendor_workspace/newznzc_ws/src/nav_demo/launch/nav777.launch` now point to a real `map_server + chassis_odometry_node + amcl + move_base` chain.
- `vendor_workspace/newznzc_ws/src/car_bringup/scripts/newt.py` is now a working serial bridge instead of a fixture stub.
- camera/lidar/imu wrapper launches no longer route into repository synthetic publishers; they now act as integration wrappers around externally provided driver launches.


本仓库**不内置**墨问原厂专有工作空间，但现在提供了明确的外部接入入口：

- `launch/mowen_vendor_runtime.launch`
- `launch/mowen_vendor_sidecar.launch`
- `tools/run_mowen_vendor_sidecar.sh`
- `scripts/platform_bridge_node.py`
- `src/ruikang_recon_baseline/platform_bridge_node.py`

## 目标

把外部 vendor 工作空间中的运行链（底盘连接、导航、相机、雷达、IMU）接到本仓库的统一任务层 contract，而不是把专有代码直接复制进本仓库。

## 对接对象

根据当前资料，实物链路通常包含以下入口：

- 底盘连接：`car_bringup/scripts/newt.py`
- 导航：`nav_demo/launch/nav777.launch` 或同类导航 launch
- 深度相机：`OrbbecSDK_ROS/dabai_dcw2.launch`
- 激光雷达：`lslidar_driver/launch/lslidar_serial.launch`
- IMU：`wit_ros_imu/launch/wit_imu.launch`

## 统一后的 task-layer contract

本仓库任务层默认消费这些统一话题：

- `recon/platform/base_feedback`
- `recon/control_mode`
- `recon/estop`
- `recon/navigation_status`
- `odom`
- `amcl_pose`
- `/camera/rgb/image_raw`

其中：

- `recon/platform/base_feedback` 由 `platform_bridge_node` 归一化输出
- `recon/control_mode` 由 `platform_bridge_node` 归一化输出
- `recon/estop` 由 `platform_bridge_node` 归一化输出
- `recon/navigation_status` 可由 `platform_bridge_node` 直接镜像 vendor `move_base/status`

## 推荐启动方式

在 `mowen_vendor_runtime.launch` 中，本仓库内部现在由 `deploy_grade` 显式决定进入 `contract_deploy.launch`、`reference_field_deploy.launch` 或 `real_field_deploy.launch`。 该 deploy 主链现在还会同步起受管 `behavior_runtime_node / behavior_executor_node`，用于消费 `recon/behavior_command`，向外部动作执行运行时发布 `recon/platform/behavior_execution/request` / `cancel`，并把下游 `recon/platform/behavior_execution/result` 规整回 `recon/behavior_feedback`。比赛 DSL 中的 `hazard_avoid` / `facility_attack` 因此不再依赖仓内定时自完成回执。仓库内置的 `mowen_raicom_reference_field_verified` 与 `config/manifests/mowen_reference_field_detector_manifest.json` 只服务于参考闭环；真实 `field_deploy.launch` 仍必须通过 `field_asset_id/field_asset_path` 与 `model_manifest_path` 显式覆盖为实测资产。外部 vendor 的 upstream 话题覆盖会直接透传给受管 `platform_bridge_node`，因此 bridge 仍然处在 system manager 监督下，而不是额外单起脱离 required_nodes 的实例。受控 vendor wrapper `tools/run_managed_vendor_runtime.py` 现在会读取 `mowen_mo_sergeant_managed_bundle.yaml` 的 `startup_sequence`，把 workspace 里的 `newt.py` / `nav777.launch` / 相机 / 雷达 / IMU 入口渲染成临时 roslaunch，并把 `vendor_*_launch` / `enable_vendor_*` / `allow_command_fallback` 等关键 seam 参数完整透传，再在同一启动图中接入仓内 `mowen_vendor_sidecar.launch`。
同时，仓库内的 `deploy.launch` / `baseline_deploy.launch` 仍保留为 contract-smoke 级入口，不强制要求外部 vendor bundle 已解析；对应的 `config/profiles/baseline/platform.yaml` 只做 advisory 级 preflight，避免仓库自带的 Noetic smoke/rostest 在未提供 `newznzc_ws` 时被默认硬门禁堵死。

### 方式一：直接提供外部 launch 路径

```bash
roslaunch ruikang_recon_baseline mowen_vendor_runtime.launch \
  field_asset_path:=/abs/path/to/verified_field_asset.yaml \
  vendor_namespace:=vendor_a \
  vendor_bringup_launch:=/abs/path/to/car_bringup_bridge.launch \
  vendor_navigation_launch:=/abs/path/to/nav777.launch \
  vendor_camera_launch:=/abs/path/to/dabai_dcw2.launch \
  vendor_lidar_launch:=/abs/path/to/lslidar_serial.launch \
  vendor_imu_launch:=/abs/path/to/wit_imu.launch
```

### 方式二：只接已经在外部启动好的 ROS 图

如果 vendor 工作空间已经单独启动，本仓库只需要桥接已存在的话题；若这些节点本身位于某个统一 namespace 下，也可以直接传 `vendor_namespace` 让默认 upstream 话题自动对齐：

```bash
roslaunch ruikang_recon_baseline mowen_vendor_runtime.launch \
  field_asset_path:=/abs/path/to/verified_field_asset.yaml \
  vendor_namespace:=vendor_a \
  enable_vendor_bringup:=false \
  enable_vendor_navigation:=false \
  enable_vendor_camera:=false \
  enable_vendor_lidar:=false \
  enable_vendor_imu:=false
```

`vendor_namespace` 的作用是把外部 include 的 vendor launch 统一包进同一作用域，并让 bridge 默认用 `vendor_ns_prefix` 自动生成 upstream 话题前缀。现在该前缀也会同步投射到任务层默认消费的 `camera_topic`、`odom_topic`、`amcl_pose_topic` 与 `move_base_action_name`，避免 vendor 节点进入命名空间后 mission/vision 仍错误订阅全局话题。命令链是例外：当 `vendor_namespace` 为空时，`upstream_command_topic` 默认保持在安全 relay `recon/platform/vendor/cmd_vel`，不会自动回落到全局 `/cmd_vel`。

边界要写清楚：如果原厂 launch 内部已经把某些 topic 写成绝对路径，那么 group namespace 不会改写这些绝对 topic，此时仍需手工覆盖 `camera_topic`、`odom_topic`、`amcl_pose_topic`、`move_base_action_name`，以及 `upstream_feedback_topic`、`upstream_odom_topic`、`upstream_navigation_status_topic`、`upstream_command_topic`。



对 field grade 还要额外注意：相机 / 雷达 / IMU 的 vendor wrapper 现在支持 `driver_launch` 与 `require_driver_launch`。在 `field` 级运行时，如果外部驱动 launch 没有显式给出，wrapper 会 fail-fast，而不是静默空跑。
另外，历史 direct vendor 入口（`nav_demo/launch/nav777.launch`、`nav_demo/launch/nav_c.launch`、`car_bringup/launch/gmapping.launch`）现在也会把雷达/IMU wrapper 以 `require_driver_launch:=true` 的方式接入，因此 direct path 与 sidecar field path 在驱动门禁强度上保持一致，不再允许仅靠 wrapper 声明就静默空跑。

## vendor 绑定证据链

`config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml` 现在除了 required binding 之外，还记录：

- `vendor_workspace_name`
- `vendor_entrypoints`
- 每个 binding 的 `vendor_resource`
- 可选 `bridge_output`
- 对接说明 `notes`

mission / vision / platform bridge 会把这些字段整理成 `vendor_runtime_binding_report` 发布到 health details。这样接入线不再只说明“要有这些 topic/action”，还会说明这些绑定在 vendor 工作空间里应当从哪个入口、哪个原始资源、经过哪条桥接链出来。运行期 graph probe 现在对解析后的绑定名做精确匹配，不再接受其他命名空间的 suffix 同名话题冒充本线资源。

## 关于底盘执行反馈

如果 vendor 栈能直接提供显式底盘执行反馈，应把该 bool 话题接到：

- `vendor_execution_feedback_topic`（推荐，经 `vendor_feedback_node` 归一化后输出到 `upstream_feedback_topic`）
- 或直接接到 `upstream_feedback_topic`（当 vendor 已经原生发布 `recon/platform/vendor/base_feedback_raw` 时），同时必须显式设置 `native_feedback_source_declared:=true`

`mowen_vendor_sidecar.launch` 现在把这件事做成硬门禁：

- 默认 `require_explicit_feedback_source:=true`
- 你必须二选一：
  - 提供 `vendor_execution_feedback_topic`，走仓内 adapter 模式；
  - 设置 `native_feedback_source_declared:=true`，明确声明 vendor 图已经原生发布 canonical `upstream_feedback_topic`。
- 两者同时设置会被拒绝，因为这会让 sidecar 无法判断到底谁是权威显式反馈源。

另外，adapter 模式下 `vendor_execution_feedback_topic` 不能与 `upstream_feedback_topic` 相同，否则会形成订阅/发布同一资源的自循环。

`vendor_feedback_node` 现在是仓内显式反馈适配器，会把 vendor bool 心跳规范化到 `recon/platform/vendor/base_feedback_raw`，并可选要求近期 `cmd_vel` 交通同时存在，避免 latched `true` 长时间把 deploy 线误判为健康。

如果没有显式 bool 反馈，是否允许退回到 `upstream_odom_topic` 心跳新鲜度由 `allow_odom_feedback_fallback` 控制。

- `generic_ros_nav` / integration 允许把 odom 仅作为 feedback fallback
- `mowen_mo_sergeant` / deploy 默认关闭该退回路径，只把 odom 当作 observability，不再把“里程计有心跳”误当成“底盘执行链健康”

因此，MO-SERGEANT 实车 deploy 线应优先接入显式 `upstream_feedback_topic`，而不是依赖 odom 代替执行反馈。


### 显式反馈适配器示例

```bash
roslaunch ruikang_recon_baseline mowen_vendor_runtime.launch   field_asset_path:=/abs/path/to/reference_or_field_asset.yaml   vendor_namespace:=vendor_a   vendor_execution_feedback_topic:=/vendor_a/chassis/execution_feedback   vendor_feedback_command_topic:=/vendor_a/cmd_vel
```

这条链会把 vendor 的 `/vendor_a/chassis/execution_feedback` 归一化到 `recon/platform/vendor/base_feedback_raw`，再由 `platform_bridge_node` 统一投射到 `recon/platform/base_feedback`。

### vendor 已原生发布 canonical feedback 的示例

```bash
roslaunch ruikang_recon_baseline mowen_vendor_runtime.launch   field_asset_path:=/abs/path/to/reference_or_field_asset.yaml   vendor_namespace:=vendor_a   upstream_feedback_topic:=/vendor_a/recon/platform/vendor/base_feedback_raw   native_feedback_source_declared:=true
```

这表示显式反馈不是通过仓内 adapter 注入，而是 vendor 图已经原生发布 canonical bool 反馈。若没有 `native_feedback_source_declared:=true`，受管 sidecar 会拒绝启动。

## 关于控制模式与急停

如有 vendor 控制模式或急停话题，可分别映射到：

- `upstream_control_mode_topic`
- `upstream_estop_topic`

`platform_bridge_node` 会统一发布到：

- `recon/control_mode`
- `recon/estop`

这样 safety 节点与 platform bridge 使用的是同一条主链，不会再出现 bridge 与 safety 订阅话题不一致的问题。

## 边界说明

本文件解决的是**接入边界与运行约束**，不是把 vendor 专有代码复制进仓库。

仍然需要在真实 Noetic / catkin / rostest / 实车环境中完成联调验证。

另外，`platform_bridge_node` 现在位于与 deploy 主链相同的 namespace 下，并会通过 `recon/health` / `recon/health_typed` 向 `system_manager_node` 报告自身生命周期与桥接状态，不再是游离于 supervisor 之外的外围组件。


## Deploy contract additions

- platform bridge now relays `upstream_command_topic -> command_input_topic` so `cmd_vel_raw` becomes the single safety ingress.
- system manager READY now depends on resource readiness details rather than only node liveness.
- recorder can emit `official_report.json` when `official_report_mode=artifact`.
- field assets now carry `asset_version`, `asset_state`, `verified`, and calibration/layout ids.


## 运行时策略约束

- `vendor_runtime_mode=isolated_legacy_workspace`
- `vendor_workspace_ros_distro=melodic`
- `vendor_workspace_python_major=2`
- `motion_model=mecanum_holonomic`
- `cmd_vel_semantics=planar_xy_yaw`
- `allow_odom_feedback_fallback=false`
- `vendor_runtime_contract_path=config/vendor_runtime/mowen_mo_sergeant_vendor_contract.yaml`

这组约束现在会在 `mission` / `vision` / `platform_bridge` 启动期统一校验；如果 profile 声明与 adapter contract 或外部 vendor runtime contract 不一致，节点会直接拒绝启动，而不是把不一致留到实车联调阶段。


## 5. Navigation contract（新增硬约束）

接入 MO-SERGEANT vendor 工作空间时，除了 `move_base_action_name`、`odom_topic`、`amcl_pose_topic` 与 `camera_topic`，还需要保证 mission profile 中声明的 navigation contract 与真实接入链一致：

- `navigation_planner_backend=move_base_global_planner`
- `navigation_controller_backend=move_base_local_controller`
- `navigation_recovery_backend=executor_policy`
- `navigation_localization_backend=amcl_pose_topic`
- `navigation_goal_transport=actionlib`
- `navigation_status_transport=actionlib`
- `navigation_cancel_transport=actionlib`

这些字段现在不是文档说明，而是 mission 启动期硬校验。若 vendor 运行线改成 goal-topic/status-topic 组合，必须同步修改 profile，不能只改 launch topic 而不改 contract。


## 运行期探针与门禁

`mission_manager_node` 现在会在启动后对导航 action/topic/localization 资源做一次运行期探针，`platform_bridge_node` 也会对 vendor 反馈与导航状态可见性做探针；两者结果都会进入统一 health details，并被 `system_manager_node` 当作 readiness requirement。这样 deploy 不再只验证声明绑定，还会验证关键运行图资源是否真的可见。

另外，deploy/deploy 现在对 detector manifest 的 `deployment_grade` 有强约束：contract deploy 接受 `contract_verified` / `reference_verified` / `field_verified`，reference deploy 接受 `reference_verified` / `field_verified`，field deploy 只接受 `field_verified`。


## 2026-04-09 command-path hardening

- `platform_bridge_node` now validates `upstream_command_topic`, `command_input_topic` and `safety_output_topic` cannot alias the same ROS resource, preventing bridge/safety self-loops in deploy profiles.
- bridge/safety health now expose `command_traffic_seen` for post-activation observability, but deploy readiness no longer blocks on pre-activation command traffic because many vendor planners only emit `cmd_vel` after a goal is active.
- baseline and integration-style platform profiles now use `recon/platform/vendor/cmd_vel` as the default upstream bridge ingress, while vendor runtime launch may still override this explicitly when integrating a namespaced vendor graph.


- Navigation runtime live readiness now requires action status freshness plus post-dispatch action feedback/result roundtrip evidence when a goal is active.


## 2026-04-09 managed sidecar bundle

- `mowen_vendor_sidecar.launch` is now the canonical in-repo managed sidecar entry.
- `mowen_vendor_runtime.launch` is implemented as a pure compatibility alias that forwards every public argument into `mowen_vendor_sidecar.launch`.
- vendor preflight now validates both repo-managed entrypoints and external workspace entrypoints.

## 受控 bundle / release / ONNX / task DSL

本轮实现后，MO-SERGEANT 接入线除了原有 `vendor_runtime_contract` 外，还新增了四类仓内可审计约束：

- `vendor_bundle_manifest_path`：声明受控 vendor bundle 清单，收口工作空间标识、锁版本、推荐启动序列与所需入口。
- `vendor_bundle_lock_id` / `vendor_bundle_lock_version`：让 profile 明确声明自己依赖的 bundle 锁，而不是仅依赖“某个外部工作空间名字”。
- `field_asset_release_manifest_path`：把 map / route / calibration / detector scope 绑定成单一 release，mission/vision 会在启动期同步应用。
- `task_dsl_path`：deploy 线可把赛场任务图与 authoritative route 一起发布，mission 会先应用 field asset release，再把 task DSL 降低成运行时 tasks。`reference_deploy` 与 `field_deploy` 现在分别使用独立的 DSL 文件，避免 field 线继续耦合 reference 命名。

另外，reference / field deploy 的视觉主路径已切到 ONNX：

- `config/profiles/reference_deploy/vision.yaml` 默认读取 `RUIKANG_REFERENCE_ONNX_MODEL`
- `config/profiles/field_deploy/vision.yaml` 默认读取 `RUIKANG_FIELD_ONNX_MODEL`，并通过 `config/field_assets/releases/mowen_raicom_packaged_field_release.yaml` 绑定 packaged field asset 与 field detector manifest；真实比赛前请替换该 release。

如果显式提供 `onnx_model_path`，则以显式参数为准。

## authoritative replay

`mission_recorder_node` finalize 后现在会额外写出 `authoritative_replay_manifest.json`，把 dynamic summary、runtime evidence、schema 信息与关键 artifact 绑定为单一 replay 入口，便于后续回放、审计与赛后复核。


## Behavior runtime and field release production-line updates

- Added `behavior_runtime_node.py` as the repository-managed downstream execution runtime. It consumes `recon/platform/behavior_execution/request`, publishes concrete commands on `cmd_vel_raw` and `recon/platform/behavior_execution/actuator_command`, observes `recon/platform/bridge_state` and `recon/detections`, and emits explicit terminal receipts on `recon/platform/behavior_execution/result`.
- Added `tools/validate_managed_vendor_bundle.py` so managed vendor bundles can be validated against one concrete external workspace before deploy handoff. CI/Noetic verification now validate one external managed vendor workspace bundle. When `RUIKANG_VENDOR_WORKSPACE_ROOT` is not provided, the workflows provision an external managed workspace bundle from the repository templates first; the repository-local fixture itself is never accepted as an external workspace unless an explicit local allow flag is used.
- Added `tools/build_field_asset_release.py` so measured/reference/contract field asset release manifests can be materialized from explicit inputs instead of hand-editing YAML.


- Added the full in-repository actuator confirmation chain: `behavior_actuator_node.py` -> `vendor_actuator_bridge_node.py` -> `vendor_actuator_device_node.py` -> `vendor_actuator_feedback_node.py`. `facility_attack` success now depends on actuator-specific raw result confirmation rather than generic `base_feedback_raw`. This closes the repository-level produced/consumed gap and removes the prior timer-based self-confirmation path. It is still a repository-controlled integration chain: real hardware execution remains subject to external vendor workspace, ROS runtime, and device-side confirmation in the target environment.


## Competition semantics and managed chassis bridge

For the competition workflow, repository-local integration now covers two extra seams that were previously only implicit:

- **Competition semantic perception** through `competition_perception_node`, which upgrades detector outputs into mission-facing classes that the competition task graphs can consume directly.
- **Managed upper-computer serial bridge** through `mowen_serial_protocol.py` and `mowen_serial_bridge_node.py`, plus a portable firmware-side reference implementation in `firmware/mowen_chassis_controller/`.

The vendor documentation still shows the original ROS-side pattern of running `newt.py` to connect the chassis and `nav777.launch` for navigation bringup. Those upstream steps remain the reference external runtime contract; the repository additions here formalize and test the managed seam around them, but they do not replace hardware-side acceptance.

## 底盘固件目标构建

现在仓库内同时保留 host reference runtime 和 AT32 target runtime：

```bash
bash tools/build_at32_firmware.sh
```

该脚本会在 `firmware/mowen_chassis_controller/at32/build/` 生成 `mowen_chassis_at32.elf` 与 `mowen_chassis_at32.bin`，用于真实板级联调前的目标构建收口。

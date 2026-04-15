# ruikang_recon_baseline

面向“智能侦察”类比赛任务的 ROS1（Noetic）工程基线。当前版本的主目标不是伪造外部专有系统，而是把**视觉计数、任务区裁决、任务记录、兼容输出、验证基建**收敛成一条可扩展、可审计、可替换的基线。

当前版本聚焦以下闭环：

- **物理任务区与图像 frame-region 解耦**：视觉只负责帧级检测与可选 frame-region 统计；最终任务区结论由 mission 驻留窗口统一裁决。
- **dynamic authoritative lane 收口**：`mission_manager_node.py` 与 `mission_recorder_node.py` 现在以动态 `class_names` 顺序作为 authoritative zone/result contract；legacy `ZoneCapture.msg` / `final_summary.json` 退化为兼容投影。
- **typed 优先、JSON 兼容**：关键链路持续支持 typed + JSON 双通道，但 health 的 typed/JSON 选择从“整条 health 流”收紧为“按 node 粒度”，避免不同 producer 互相遮蔽。
- **兼容 lane 边界化**：recorder 不再让 legacy `ZoneCapture.msg` 参与 authoritative 运行态；legacy lane 仅作为 `projection_only` compatibility shadow 保留，用于兼容观察与审计，不再阻断 finalize。
- **class schema 跨节点强校验**：vision / mission / recorder 现在都显式携带 `class_names` 与 `class_schema_hash`；`class_schema_mismatch_policy` 控制 schema 漂移时是告警继续还是直接阻断。
- **route 身份与展示名分离**：zone 结果与工件现在收口到稳定 `route_id`，`zone_name` 退回展示字段，避免重复航点名覆盖结果；若 route 中同名航点重复出现，默认按该名字自己的出现序号生成 `name__1`、`name__2`。
- **dispatch 前置检查显式化**：mission 新增 `preflight_require_pose`、`preflight_require_detections`、`preflight_timeout_sec`、`preflight_failure_policy` 与独立 `detections_timeout_sec`，避免在输入盲区直接发车。
- **artifact 与观测链路解耦**：`vision_events.jsonl` 与图片工件解耦；图片继续按 `save_interval_sec` 节流，事件日志由 `save_event_log` / `event_log_interval_sec` 独立控制。
- **baseline 语义集成验证补齐**：`baseline_integration_smoke.test` 现在使用 integration 专用 mission/vision profile，显式绑定 `frame_region` 与 route，并通过 bootstrap `amcl_pose` 夹具证明 baseline 主链的语义闭环，而不再只是 transport smoke。
- **platform adapter 显式化**：新增 `config/common/platform.yaml` 与 `src/ruikang_recon_baseline/platform_adapters/`，把 generic ROS 栈与 MO-SERGEANT 接入线的外部 contract 独立出来，不再把平台假设散落在 mission/safety 节点内部。
- **field asset 资产化**：新增 `config/field_assets/` 与 `field_asset_id` 机制，把 route / named_regions / map 绑定从 profile 常量提升为可版本化赛场资产，并要求 field-asset-backed profile 不再重复内联真值。
- **field deploy 平台默认值收口**：`launch/field_deploy.launch` 与 `launch/semantic_deploy.launch` 现在默认进入 `config/profiles/field_deploy/platform.yaml` 和 `config/profiles/field_deploy/system_manager.yaml`，不再误走 generic baseline 平台 profile。
- **reference deploy 参考场地包分域化**：仓库新增 `mowen_raicom_reference_field_verified` 场地资产与 `mowen_reference_field_detector_manifest.json`，仓库将参考资产收口到 `config/profiles/reference_deploy/{vision,mission}.yaml`；`mowen_vendor_runtime.launch` 现在作为纯 alias 转发到 `mowen_vendor_sidecar.launch`，并通过 `deploy_grade` 在 `contract_deploy.launch` / `reference_field_deploy.launch` / `real_field_deploy.launch` 之间做显式切换；仓库同时补充了 `mowen_raicom_packaged_field_verified` + `mowen_raicom_packaged_field_release` 作为 field_deploy 的代码级自包含 packaged 闭环输入，但其 notes 已明确声明：真实比赛前必须切换到 real_field_deploy 所要求的实测(measured)场地资产与 detector manifest。
- **vendor bundle 硬门禁按入口分级**：`mowen_integration` / `baseline_legacy` / `reference_field_deploy` / `field_deploy` 继续要求 `vendor_bundle_preflight_mode=required`；`deploy.launch` / `baseline_deploy.launch` 仍保持仓库自洽的 contract smoke 入口，因此 `config/profiles/baseline/platform.yaml` 仅使用 `advisory`，并在 `config/profiles/baseline/system_manager.yaml` 中去除 `vendor_bundle_preflight_satisfied` 的强制 ready gate。
- **managed vendor bundle manifest 收口**：platform profile 现在可显式声明 `vendor_bundle_manifest_path`、`vendor_bundle_lock_id`、`vendor_bundle_lock_version`；preflight 会同时校验受控 bundle 清单、锁版本与启动序列，避免 profile 只声明“依赖外部工作空间”但没有仓内版本证据。
- **field asset release manifest 收口**：mission / vision 除了 `field_asset_id/field_asset_path` 外，还可声明 `field_asset_release_manifest_path`；release manifest 会把 map / route / named_regions / calibration / detector scope 绑定成单一发布单元，避免 profile 自行拼装 reference/field 资产。
- **field-asset 缺失路径收硬**：当 `require_verified_field_asset=true` 时，缺省不填 `field_asset_id/field_asset_path` 也会直接报错，不能再靠“完全不声明资产”绕过 deploy/field gate。
- **navigation contract 与运行时角色显式化**：mission 配置现在不仅声明 `navigation_adapter_type`，还显式声明并校验 `navigation_planner_backend`、`navigation_controller_backend`、`navigation_recovery_backend`、`navigation_localization_backend` 以及 goal/status/cancel transport；运行态进一步拆成 dispatcher / status monitor / canceller 三个显式角色，planner/controller/recovery/localization 不再只是健康摘要描述，而是启动期硬 contract。
- **deploy safety 硬闭环收口**：MO-SERGEANT deploy 线现在通过 platform adapter 默认要求 `recon/platform/base_feedback`，并新增 `platform_bridge_node` 将 vendor 原始反馈/里程计心跳桥接到统一 contract，同时把 vendor 控制模式/急停/导航状态统一到 `recon/control_mode`、`recon/estop`、`recon/navigation_status`。`tests/baseline_deploy_smoke.test` 也改为先喂 `recon/platform/vendor/base_feedback_raw` 再由桥接节点归一化，证明 deploy 路径不再只是直接往最终 feedback topic 塞测试夹具。
- **ordered lifecycle 编排**：`system_manager_node` 现在支持按 required_nodes 顺序下发 `configure -> activate`，并通过 JSON envelope（target / issued_by / metadata）对 managed 节点做定向控制；旧 plain-string 广播命令仍兼容。
- **runtime evidence 总线**：`system_manager_node` 新增 `recon/runtime/evidence` 聚合 topic，把 manager state、mission state、各受管节点健康快照和本轮调度细节统一投影为单条 JSON 证据流；recorder 会将该证据流落盘到 mission log。
- **calibration-aware regions**：field asset 新增 `region_calibration`，vision 现在支持 `calibrated_named_regions`，可把参考分辨率下 author 的 named regions 按标定比例投影到当前图像尺寸，而不是永远硬吃静态像素框。
- **vendor 绑定证据链显式化**：`vendor_runtime_contract` 现在不仅声明 required bindings，还可携带 `vendor_workspace_name`、`vendor_entrypoints`、`vendor_resource`、`bridge_output` 等证据字段；mission / vision / platform bridge 的 health details 会同步发布 `vendor_runtime_binding_report`。
- **runtime probe 精确匹配化**：运行期 topic probe 不再使用双向 suffix 宽匹配，而改为对已解析后的绑定名做精确匹配，避免把其他机器人或其他命名空间的同名话题误判成 ready。
- **运行入口语义拆明**：新增 `launch/deploy.launch`、`launch/integration.launch`、`launch/contract.launch`、`launch/legacy.launch`；历史 `baseline*.launch` 保留为迁移别名，避免旧入口直接失效。
- **dynamic schema 集成验证补齐**：`dynamic_schema_integration_smoke.test` 现在要求 fake detections 显式携带 `class_names` 与 `class_schema_hash`，专门验证扩类 schema 在 mission / recorder 主链中的运行时行为。
- **dynamic schema 负向异常路径补齐**：新增 `dynamic_schema_mismatch_smoke.test`，用故意错误的 embedded schema 验证 mission 会按 `class_schema_mismatch_policy=error` 进入 `FAULT`，而不是悄悄吞掉异常输入。
- **namespace 真正落到 I/O 边界**：默认 topic/action 名保持相对路径，`output_root_use_namespace` 可把 ROS namespace 派生到工件目录。
- **环境冻结材料补齐**：新增 `Dockerfile.noetic` 与 `docker/entrypoint.noetic.sh`，并把当前推荐支持环境收口到 Noetic/Focal 冻结线。
- **runtime grade / 行为后端 / 硬件策略入主链**：mission / platform_bridge / recorder / system_manager 统一接收 `runtime_grade`，artifact 与健康态会带出 contract/reference/field 等级；`hazard_avoid` 与 `facility_attack(command_confirmed)` 不再停留在纯语义 dwell，而是通过 `behavior_action_backend` 走显式动作执行传输契约；platform bridge 新增 ultrasonic / IMU / voice / PS2 接入与 `runtime_evidence_topic`，将硬件策略必须项落入安全链与证据链。
- **deploy 行为执行传输桥已入监督主链**：仓库新增 `behavior_runtime_node / behavior_executor_node.py` 与 `config/{common,profiles/*}/behavior_executor.yaml`，由 `deploy -> core` 直接起受管执行节点，消费 `recon/behavior_command`、向外部执行运行时发布 `recon/platform/behavior_execution/request` / `cancel`，并把下游 `recon/platform/behavior_execution/result` 规整回 `recon/behavior_feedback`；`reference_field_deploy.launch` / `real_field_deploy.launch` / `field_deploy.launch` 默认启用该节点，system manager 也会把 `behavior_executor_node` 纳入 ready gate。

> 本仓库仍然**不内置官方专有 SDK、裁判系统协议、真实地图参数和真实识别模型**。它提供的是一套可直接接真实 ROS1 导航栈、相机和识别模型的规范基线。

## 1. 当前目录结构

```text
ruikang_recon_baseline/
├── .dockerignore
├── .github/workflows/ci.yml
├── CMakeLists.txt
├── Dockerfile.noetic
├── IMPLEMENTATION_SUMMARY.md
├── README.md
├── docker/
│   └── entrypoint.noetic.sh
├── msg/
│   ├── Detection.msg
│   ├── DetectionArray.msg
│   ├── FrameRegionCounts.msg
│   ├── HealthState.msg
│   ├── MissionState.msg
│   ├── ZoneCapture.msg
│   └── ZoneCaptureDynamic.msg
├── config/
│   ├── common/
│   ├── field_assets/
│   ├── profiles/
│   │   ├── baseline/
│   │   ├── baseline_contract/
│   │   ├── baseline_integration/   # 兼容别名
│   │   ├── mowen_integration/      # 规范入口
│   │   ├── baseline_legacy/
│   │   ├── dynamic_integration/
│   │   └── demo/
│   └── models/
├── launch/
│   ├── baseline.launch
│   ├── baseline_contract.launch
│   ├── baseline_deploy.launch
│   ├── baseline_legacy.launch
│   ├── contract.launch
│   ├── core.launch
│   ├── demo.launch
│   ├── demo_synthetic.launch
│   ├── deploy.launch
│   ├── field_deploy.launch
│   ├── integration.launch
│   ├── mowen_integration.launch
│   ├── legacy.launch
│   └── semantic_deploy.launch
├── scripts/
│   ├── cmd_safety_mux_node.py
│   ├── fake_base_feedback_publisher.py
│   ├── platform_bridge_node.py
│   ├── fake_detection_array_publisher.py
│   ├── fake_move_base_action_server.py
│   ├── mission_manager_node.py
│   ├── mission_recorder_node.py
│   ├── synthetic_scene_publisher.py
│   └── vision_counter_node.py
├── src/ruikang_recon_baseline/
│   ├── artifact_projection.py
│   ├── common.py
│   ├── domain_models.py
│   ├── field_assets.py
│   ├── geometry_utils.py
│   ├── io_core.py
│   ├── manifest_utils.py
│   ├── mission_config.py
│   ├── mission_core.py
│   ├── mission_publishers.py
│   ├── mission_schema_guard.py
│   ├── platform_adapters/
│   ├── recorder_core.py
│   ├── route_utils.py
│   ├── runtime_paths.py
│   ├── safety_core.py
│   ├── safety_node.py
│   ├── schema_utils.py
│   ├── time_core.py
│   ├── vision_core.py
│   └── vision_node.py
├── tools/
│   └── validate_profile_contracts.py
└── tests/
    ├── baseline_contract_smoke.py
    ├── baseline_contract_smoke.test
    ├── baseline_integration_smoke.py
    ├── baseline_integration_smoke.test
    ├── baseline_integration_namespace_smoke.test
    ├── baseline_profile_smoke.test
    ├── dynamic_schema_integration_namespace_smoke.test
    ├── dynamic_schema_integration_smoke.py
    ├── dynamic_schema_integration_smoke.test
    ├── dynamic_schema_mismatch_smoke.py
    ├── dynamic_schema_mismatch_smoke.test
    ├── baseline_smoke.py
    ├── demo_pipeline_smoke.py
    ├── demo_profile_smoke.test
    ├── test_architecture_controls.py
    ├── test_core_logic.py
    ├── test_repository_consistency.py
    └── test_time_core.py
```

## 1.1 当前平台/场地边界

- **平台层**：通过 `platform_adapter_type` 明确声明当前外部平台 contract。当前内置 `generic_ros_nav` 与 `mowen_mo_sergeant` 两条声明线，并通过 `platform_bridge_node` 把 vendor 原始反馈、控制模式、急停与导航状态桥接到统一任务层 contract。
- **平台 contract 绑定校验**：mission / safety 现在会在启动期校验声明的 `required_topics` / `required_actions` 是否真的绑定到节点配置，避免 adapter 只写声明不进主链。
- **场地资产层**：通过 `field_asset_id` 把 route / named_regions / map 绑定到 `config/field_assets/*.yaml`。
- **任务层**：vision / mission / recorder / safety 继续只消费 contract，而不内嵌原厂底盘驱动或专有工作空间代码。

## 2. 节点职责

### 2.1 `vision_counter_node.py`
输入：相机图像。输出：

- `DetectionArray` typed：`recon/detections`
- JSON 兼容：`recon/detections_json`
- frame-region 统计 JSON：`recon/zone_counts`
- frame-region 统计 typed：`recon/zone_counts_typed`
- 调试图像：`recon/overlay/image_raw`
- `recon/runtime/evidence`
- 健康度 JSON：`recon/health`
- 健康度 typed：`recon/health_typed`

说明：

- 节点启动时用 `~classes` 校验**动态 class schema**；不再把内部 authoritative schema 钉死为 `friendly/enemy/hostage`。
- `color_blob` 检测器允许作为配置 schema 的**子集**运行；缺失类按 0 处理。当前默认 `color_blob + synthetic` 只证明 legacy 三类/其子集，不等价于端到端动态感知扩类能力。
- `onnx` 检测器要求 manifest `class_names` 与运行配置中的 `classes` **严格一致**，防止模型/任务侧静默错配。
- `save_artifacts` 仅控制图片工件；`save_event_log` 独立控制 `vision_events.jsonl`。
- `event_log_interval_sec` 为 0 时表示每帧记录结构化事件日志。

### 2.2 `mission_manager_node.py`
输入：

- 帧级检测 `recon/detections`
- 位姿源（`amcl_pose`、`odom` 或 TF 查找）
- 导航反馈（`move_base` action、simple topic 或 `goal_topic_status`）

输出：

- 任务状态 JSON：`recon/mission_state`
- typed 任务状态：`recon/mission_state_typed`
- 区域结果 JSON：`recon/zone_capture_result`
- fixed-schema typed 区域结果：`recon/zone_capture_result_typed`
- dynamic typed 区域结果：`recon/zone_capture_result_dynamic`
- 健康度 JSON / typed：`recon/health` / `recon/health_typed`

说明：

- 物理区结果只在 mission 驻留窗口内聚合，只有这里才会把检测结果绑定到真实任务区。
- 当前 authoritative 聚合 contract 是动态 `class_names` 顺序；`ZoneCapture.msg` 与 legacy JSON 是从 dynamic authoritative 结果投影出来的兼容 lane。
- 检测输入必须显式携带 `class_names` 与 `class_schema_hash`；一旦与 mission 配置不一致，会按 `class_schema_mismatch_policy` 触发 health / state 告警或直接转入 `FAULT`。
- `MissionState.msg` 收紧了常用固定字段（如 `event_id`、`zone_name`、`failure_reason`、`duration_sec` 等），同时补充 `current_route_id` / `next_route_id` 以稳定表达 route 身份，并保留 `details_json` 兼容尾巴。
- `navigation_cancel_topic` 与 `navigation_failure_quiesce_sec` 用于收口 adapter 的取消与重派发行为。
- dispatch 前置检查由 `preflight_require_pose`、`preflight_require_detections`、`preflight_timeout_sec`、`preflight_failure_policy` 与 `detections_timeout_sec` 控制；`baseline_contract` profile 显式关闭 detection preflight，因为它默认不启 vision。

### 2.3 `mission_recorder_node.py`
记录任务状态、区结果、frame-region 统计、健康度，并异步写入 `mission_log.jsonl`。

生成：

- `summary_snapshot.json`
- `summary_snapshot_v2.json`
- `final_summary.json`
- `final_summary.csv`
- `final_summary_v2.json`
- `final_summary_v2.csv`

说明：

- `recorder_authoritative_input: auto|typed|json`
- `terminal_finalize_policy: immediate|quiesced`
- `terminal_quiesce_sec`
- `allow_final_rewrite_after_finalize`
- `zone_capture_compatibility_mode: projection_only`
- `control_command_topic`
- `class_schema_mismatch_policy: warn|error`

当前行为：

- 默认策略为 `quiesced`，先看到终态，再等待静默窗口或 route 收敛后写最终工件。
- `zone_results_dynamic` 是 recorder 内部 authoritative zone state。
- legacy `ZoneCapture.msg` / `final_summary.json` 退化为兼容投影；legacy zone-capture lane 只记录 `compat_shadow` 事件，不再回灌 recorder authoritative state，也不再以 compatibility mismatch 阻断 finalize。
- `summary_snapshot` 继续保留 legacy 摘要字段并镜像 `mission_state`，用于兼容既有 smoke/消费者。
- `summary_snapshot_v2` 与 `final_summary_v2.*` 则完整保留 `class_names`、`class_schema_hash`、`current_route_id`、`zone_results_dynamic` 与 recorder diagnostics；nested zone result 的 `class_schema_hash` 会按 authoritative `class_names` 强制重算，避免 legacy lane 的脏 hash 污染最终工件。
- `zone_results*` 与 CSV 现在都以 `route_id` 作为结果键/首列，`zone_name` 仅保留展示语义。
- health 的 typed/JSON 选择从“整条 health 流”收紧为“按 node 粒度”；JSON-only producer 不会再被其他 typed producer 整体遮蔽。

### 2.4 `cmd_safety_mux_node.py`
支持多源速度输入、优先级仲裁、速度限幅、命令超时、急停 freshness 检查。

输出：

- `cmd_vel`
- `recon/safety_state`
- `recon/health`
- `recon/health_typed`

说明：

- 当前它仍是**安全壳/外围控制链**，不是 mission route dispatch 本身的一部分。
- 因此 safety 的可运行性不应被误解为 mission-control 主链已被完全证明。

### 2.5 `synthetic_scene_publisher.py`
为 `demo_synthetic.launch` 生成稳定可复现的俯视合成画面，仅用于 demo profile。

### 2.6 测试夹具

- `fake_move_base_action_server.py`：最小可控 `move_base` action 伪服务，支持 bootstrap `amcl_pose` 以满足首跳 preflight，再按固定延迟回发目标 pose 并返回成功。
- `fake_detection_array_publisher.py`：最小可控 `DetectionArray` 伪发布器，默认显式携带 `class_names` 与 `class_schema_hash`，并支持按参数故意发送错误/缺失的 embedded schema，用于 dynamic schema 正向与负向集成验证。

## 3. 构建依赖

推荐环境：**Ubuntu 20.04 + ROS Noetic**。

### 3.1 使用 rosdep 安装依赖

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.2 手工依赖（兜底）

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-actionlib \
  ros-noetic-actionlib-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-std-msgs \
  ros-noetic-move-base-msgs \
  ros-noetic-tf2-ros \
  ros-noetic-rostest \
  python3-opencv \
  python3-numpy \
  python3-yaml
```

### 3.3 环境冻结（推荐）

仓库新增：

- `Dockerfile.noetic`
- `docker/entrypoint.noetic.sh`

用于冻结 ROS Noetic + Ubuntu 20.04/Focal 构建环境，降低主机环境漂移带来的假问题。

## 4. 编译

```bash
cd ~/catkin_ws/src
cp -r /path/to/ruikang_recon_baseline .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 5. 启动方式

### 5.1 demo profile（纯合成闭环）

```bash
roslaunch ruikang_recon_baseline demo_synthetic.launch
```

### 5.2 baseline deploy profile（受 system manager 管理的语义 deploy / smoke 入口）

```bash
roslaunch ruikang_recon_baseline baseline_deploy.launch
```

`baseline_deploy.launch` 现作为 `contract_deploy.launch` 的兼容别名，会显式打开 `enable_system_manager:=true`，并默认启用 safety；mission 不再依赖节点自启动，而是由 `system_manager_node` 在 vision/mission/recorder/safety/platform_bridge 健康就绪后统一发布 `activate`。该入口额外暴露了 `enable_synthetic` 与 `profile_synthetic_config`，因此被明确界定为**contract deploy / 受管 smoke**，而不是实车 field deploy。

### 5.2.1 field deploy（实车入口，强制 verified field asset）

```bash
roslaunch ruikang_recon_baseline field_deploy.launch \
  field_asset_path:=/abs/path/to/verified_field_asset.yaml
```

`field_deploy.launch` 现在不再复用 baseline contract 级 field asset / detector manifest 默认值，而是切换到 `config/profiles/field_deploy/{vision,mission}.yaml`。该入口仍沿用 deploy 主链，并强制 `require_verified_field_asset:=true` 与 `required_field_asset_verification_scope:=field`。仓库默认通过 packaged field release 注入 field-scoped detector manifest 与 authoritative asset；若切到真实比赛链，则需显式覆盖为实测 field release / asset / detector manifest，否则 vision / mission 两侧会在启动期执行 deploy-stage contract 校验并拒绝进入运行态。

### 5.3 baseline compatibility profile（兼容旧入口，不作为 deploy 主入口）

```bash
roslaunch ruikang_recon_baseline baseline.launch
```

`baseline.launch` 保留为兼容入口，内部统一加载 integration 风格的 vision/mission/recorder/safety profile，并保持旧的非托管启动方式；它不承担新的 deploy 语义。当前规范的墨问集成入口为 `mowen_integration.launch` 与 `config/profiles/mowen_integration/*.yaml`，`baseline_integration` 仅保留为兼容别名并由 validator 强制要求与 canonical profile 不得漂移。需要受管 smoke 时请使用 `contract_deploy.launch`（`baseline_deploy.launch` 仍保留兼容别名），需要与 `deploy.launch` 参数能力等价的语义 deploy 入口时请使用 `semantic_deploy.launch`，需要真实 field deploy 时请使用 `field_deploy.launch`。


### 5.4 baseline contract（无 `move_base` 的主链合同验证）

```bash
roslaunch ruikang_recon_baseline baseline_contract.launch
```

### 5.5 `core.launch`

`core.launch` 关键参数：

- `namespace`
- `enable_synthetic`
- `enable_vision`
- `enable_mission`
- `enable_recorder`
- `enable_safety`
- `output_root`
- `camera_topic`
- `detections_topic`
- `use_sim_time`

示例：

```bash
roslaunch ruikang_recon_baseline baseline_deploy.launch namespace:=robot1 output_root:=/tmp/recon_logs
```

### 5.6 namespace 说明

当前 common 配置中的 topic / action 默认值已经改为**相对路径**。因此：

- 当 `namespace:=robot1` 时，`recon/detections` 会实际展开到 `/robot1/recon/detections`
- `move_base_action_name: "move_base"` 会展开到 `/robot1/move_base`
- 若 `output_root_use_namespace: true`，工件目录会派生到 `/tmp/recon_logs/robot1/`

如果必须接绝对路径的外部系统，请显式 remap 或在 profile 中写绝对路径，而不是依赖 common 默认值。

## 6. 配置分层

- `config/common/*.yaml`：节点通用参数
- `config/profiles/baseline/*.yaml`：受 system manager 管理的 deploy 主链参数（用于 `baseline_deploy.launch` / `deploy.launch` 语义闭环）
- `config/profiles/baseline_contract/*.yaml`：无真实导航栈时的主链合同测试参数
- `config/profiles/mowen_integration/*.yaml`：规范的墨问集成 profile
- `config/profiles/baseline_integration/*.yaml`：兼容别名 profile（validator 强制要求与 canonical 保持一致）
- `config/profiles/baseline_legacy/*.yaml`：旧未绑定 route/frame-region 语义的回滚参数
- `config/profiles/demo/*.yaml`：合成演示链路默认参数
- `config/common/system_manager.yaml` / `config/profiles/baseline/system_manager.yaml`：栈级生命周期与 supervisor 参数
- `config/models/onnx_manifest_example.json`：ONNX 模型 manifest 示例

### 6.1 时间语义

- `use_sim_time: true`：通过 launch 将 `/use_sim_time` 注入运行环境，配合 `time_source_mode: "ros"` 使用
- `publish_json_debug_topics: false`：在 deploy profile 下关闭 `recon/detections_json` 这类 debug/兼容 topic，避免旁路 topic 被误认为 authoritative 主链
- `time_source_mode: "ros"`：状态机、freshness、mission timeout、capture window 统一使用 ROS time
- `time_source_mode: "wall"`：兼容 legacy wall time 行为
- 本地文件时间戳、JSONL writer 元数据和工件文件名继续使用 wall time
- 节流与 interval 判断使用 monotonic time

### 6.2 mission / recorder / vision 关键参数

#### mission

- `navigation_cancel_topic`
- `navigation_failure_quiesce_sec`
- `zone_capture_dynamic_topic`
- `health_typed_topic`
- `health_frame_id`
- `output_root_use_namespace`
- `class_schema_mismatch_policy`
- `preflight_require_pose`
- `preflight_require_detections`
- `detections_timeout_sec`
- `preflight_timeout_sec`
- `preflight_failure_policy`

#### recorder

- `frame_region_counts_typed_topic`
- `health_typed_topic`
- `zone_capture_dynamic_topic`
- `summary_snapshot_dynamic_topic`
- `terminal_finalize_policy`
- `terminal_quiesce_sec`
- `allow_final_rewrite_after_finalize`
- `zone_capture_compatibility_mode: projection_only`
- `control_command_topic`
- `lifecycle_managed`
- `profile_role`
- `time_source_mode`
- `class_schema_mismatch_policy`
- `output_root_use_namespace`

#### vision

- `frame_region_counts_typed_topic`
- `health_typed_topic`
- `health_frame_id`
- `save_event_log`
- `event_log_interval_sec`
- `output_root_use_namespace`

#### recorder / safety 仍有关键参数

- `recorder_authoritative_input: auto|typed|json`
- `snapshot_flush_hz`
- `snapshot_flush_on_state_change_only`
- `health_emit_mode: edge|periodic`
- `health_heartbeat_hz`
- `warn_on_idle_command_stale`

## 7. ONNX 模型切换

在 `config/common/vision.yaml` 或 profile 覆盖中配置：

```yaml
detector_type: "onnx"
onnx_model_path: "/abs/path/to/model.onnx"
model_manifest_path: "$(find ruikang_recon_baseline)/config/models/onnx_manifest_example.json"
strict_backend: true
```

当前版本要求：

- manifest `class_names`
- `vision.yaml` 的 `classes`
- `mission.yaml` / `recorder.yaml` 的 `classes`

必须完全一致，否则直接启动失败，防止静默错配。

reference / field deploy profile 现在默认把 ONNX 模型路径收口到环境变量，而不是把绝对路径硬编码进 profile：

- `config/profiles/reference_deploy/vision.yaml` 读取 `RUIKANG_REFERENCE_ONNX_MODEL`
- `config/profiles/field_deploy/vision.yaml` 读取 `RUIKANG_FIELD_ONNX_MODEL`

也可以继续显式提供 `onnx_model_path`；若两者同时提供，则显式配置优先。

## 7.1 任务 DSL / 赛场 release

reference / field deploy 现在支持通过 `task_dsl_path` 加载任务图 DSL；仓库内置示例位于：

- `config/task_graphs/mowen_reference_competition_tasks.yaml`

当 profile 同时声明 `field_asset_release_manifest_path` 与 `task_dsl_path` 时，mission 会先应用 release manifest，再基于 authoritative route 降低成运行时 `tasks`。这意味着 deploy 线可以同时保留 route 真值和 task graph，而不再要求手工维护两套互相漂移的任务定义。

## 7.2 authoritative replay 工件

authoritative replay manifest 现在除 `final_summary_v2.json` 外，还会索引 `final_summary.json`、`runtime_metrics.json`、`official_report.json`、submission receipt、snapshot、CSV 与 `mission_log.jsonl`，用于赛后复核与离线回放入口收口。

recorder finalize 现在会额外产出 `authoritative_replay_manifest.json`，用于把 authoritative summary、dynamic schema、runtime evidence 与关键工件索引为单一 replay 入口。

## 7.3 运行契约矩阵

### 主链接口

- `recon/detections`：视觉 typed 检测主链输入。
- `recon/mission_state_typed`：任务状态主 typed lane。
- `recon/zone_capture_result_dynamic`：dynamic authoritative lane。
- `recon/summary_snapshot_v2`：dynamic 摘要主题，保留 `class_names`、`zone_results_dynamic` 与 recorder diagnostics。

### 兼容接口

- `recon/mission_state`：legacy JSON 兼容 lane。
- `recon/zone_capture_result`：legacy JSON 兼容 lane。
- `recon/zone_capture_result_typed`：legacy fixed-schema typed 兼容 lane。
- 即使是 legacy `recon/zone_capture_result` / `recon/zone_capture_result_typed`，现在也必须携带与当前 recorder schema 一致的 `class_schema_hash`；缺失或错误时会按 `class_schema_mismatch_policy` 拒收/阻断。
- `recon/summary_snapshot`：legacy 摘要主题，同时镜像 `mission_state` 嵌套对象。
- `final_summary.json` / `final_summary.csv`：legacy fixed-schema 兼容工件。

### 外部接口 / 观测接口

- `recon/detections_json`
- `recon/zone_counts`
- `recon/zone_counts_typed`
- `recon/overlay/image_raw`
- `recon/runtime/evidence`
- `recon/safety_state`
- `recon/health`
- `recon/health_typed`
- `cmd_vel`

其中一部分是**外部接口 / 观测接口**，不应被误判为 mission 主链已消费完成的内部闭环。

## 8. 输出文件

默认目录：

```text
~/.ros/ruikang_recon/
```

若 `output_root_use_namespace: true` 且 namespace 非空，则落到：

```text
~/.ros/ruikang_recon/<namespace>/
```

主要文件：

- `vision_events.jsonl`
- `mission_manager_events.jsonl`
- `mission_log.jsonl`
- `summary_snapshot.json`
- `summary_snapshot_v2.json`
- `final_summary.json`
- `final_summary.csv`
- `final_summary_v2.json`
- `final_summary_v2.csv`
- `annotated_*.jpg`

## 8.0 外部 vendor 工作空间接入

仓库新增 `launch/mowen_vendor_sidecar.launch` 作为仓内 canonical managed sidecar 入口，`launch/mowen_vendor_runtime.launch` 现在仅作为**纯兼容 alias**转发所有公开参数到 sidecar，而不是再维护一份重复实现。sidecar 现在通过 `deploy_grade={contract,reference,field}` 显式选择 deploy 主链；`field_deploy.launch` 默认附带 packaged field release 以完成仓内代码闭环；真实比赛前仍要求替换为实测 verified field asset 与 field-scoped detector manifest。同时新增 `vendor_namespace` / `vendor_ns_prefix` 契约：外部 vendor launch 会统一放进 `vendor_namespace` 作用域，bridge 的默认 upstream 话题会自动跟随该前缀生成（例如 `/vendor_a/odom`、`/vendor_a/cmd_vel`、`/vendor_a/move_base/status`、`/vendor_a/recon/platform/vendor/base_feedback_raw`）。默认 `vendor_namespace` 为空时，camera/odom/amcl/move_base 等消费面仍可等价对齐全局图；但上游 command ingress 不再默认回落到全局 `/cmd_vel`，以避免与 safety 输出形成自回环。此时应显式提供安全的 vendor command relay topic，或手工覆盖 `upstream_command_topic`。该入口把 upstream 话题覆盖直接透传进 `reference_field_deploy -> deploy -> core` 的受管 `platform_bridge_node`，不再通过额外单起一个脱离 system manager required_nodes 的 bridge 实例来接线。针对 `isolated_legacy_workspace` 运行策略，profile 现在还必须声明 `vendor_runtime_contract_path`，并由 mission / vision / platform_bridge 在启动期共同校验外部 vendor runtime contract。

典型用法：

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

如果 vendor 工作空间已经单独启动，也可以把所有 `enable_vendor_*` 关闭，只保留 `platform_bridge_node` 来桥接运行中的 ROS 图。更详细说明见 `docs/mowen_vendor_integration.md`。

注意：`vendor_namespace` 只负责把 **相对 topic 名** 的 vendor launch 统一包进同一作用域；如果原厂 launch 内部写死了绝对 topic，仍需显式覆盖 `upstream_*` 参数，不能把这一层收口夸大成“任意 vendor launch 都天然支持多实例”。

## 8.1 Noetic 一键验证脚本

此外，仓库新增 `.github/workflows/noetic-verification.yml`，用于在 GitHub Actions 中通过 `Dockerfile.noetic` 自动执行 `tools/run_noetic_verification.sh`。该工作流的作用是把 catkin/rostest 验证路径固化为 CI，而不是只停留在文档说明。


仓库自带 `tools/run_noetic_verification.sh`，用于在真实 Noetic 环境中自动完成：

- 先执行 `tools/validate_repository_hygiene.py`，拒绝带 `__pycache__/.pyc/.pytest_cache` 的脏树进入发布验证
- 先执行 `tools/validate_launch_contracts.py`，静态检查 launch/test XML、package 内 node 脚本与 package-relative 配置引用
- 执行 `tools/validate_profile_contracts.py`
- 建立临时 catkin 工作区
- 链接当前 package
- 执行 `catkin_make`
- 执行 `catkin_make tests`
- 执行 `catkin_make run_tests_ruikang_recon_baseline`
- 依次执行显式 rostest smoke，其中包含 baseline/dynamic-schema 的 namespace 变体
- 额外执行 `python3 -m unittest discover -s tests -p 'test_*.py'`

示例：

```bash
bash tools/run_noetic_verification.sh
bash tools/run_noetic_verification.sh /tmp/ruikang_verify
```

> 该脚本依赖本机已安装 `/opt/ros/noetic`、`catkin_make`、`rostest` 与 `roscore`。

## 9. 测试

### 9.1 纯 Python 单元测试

```bash
cd /path/to/ruikang_recon_baseline
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py'
```

### 9.2 demo profile rostest smoke

- `tests/demo_profile_smoke.test`
- `tests/demo_pipeline_smoke.py`

### 9.3 baseline wrapper smoke

- `tests/baseline_profile_smoke.test`
- `tests/baseline_smoke.py`

这个测试主要确认 wrapper 与 safety 壳可启动，不证明真实 baseline mission/vision 主链。

### 9.4 baseline contract smoke

- `tests/baseline_contract_smoke.test`
- `tests/baseline_contract_smoke.py`

这个测试不会依赖真实 `move_base`，但会覆盖 mission → recorder 的终态封存与最终工件生成。

### 9.5 baseline semantic integration smoke

- `tests/baseline_integration_smoke.test`
- `tests/baseline_integration_namespace_smoke.test`
- `tests/baseline_integration_smoke.py`
- `scripts/fake_move_base_action_server.py`
- `config/profiles/baseline_integration/synthetic.yaml`
- `config/profiles/baseline_integration/vision.yaml`
- `config/profiles/baseline_integration/mission.yaml`

这个测试现在使用 fake `move_base` + bootstrap `amcl_pose` + `synthetic_scene_publisher.py` + `vision_counter_node.py`，用于验证 **synthetic_scene_publisher → vision_counter → mission_manager → mission_recorder** 的 baseline 语义闭环，而不是只验证 transport 可达：

- synthetic 图像发布
- `vision_counter_node` 产生 `recon/detections`
- integration profile 显式启用 `named_regions` frame-region adapter
- mission route 中每个 waypoint 显式绑定对应 `frame_region`
- fake `move_base` 在首跳 dispatch 前提供 bootstrap `amcl_pose`，满足 `preflight_require_pose`
- mission dwell 结束后输出 dynamic zone capture
- recorder finalize 并写出 `final_summary_v2.json`
- 测试断言每个 zone 的 `class_counts` 与 synthetic region 配置一致，终态必须为 `FINISHED`

这个夹具仍然不是“真实机器人 / 真实导航 / 真实视觉模型”验收，但它已经从 transport smoke 升级为 semantic integration，并额外提供一个 namespaced 变体用于验证 namespace 到 topic / output root 的派生链。

### 9.6 dynamic schema integration smoke

- `tests/dynamic_schema_integration_smoke.test`
- `tests/dynamic_schema_integration_namespace_smoke.test`
- `tests/dynamic_schema_integration_smoke.py`
- `scripts/fake_move_base_action_server.py`
- `scripts/fake_detection_array_publisher.py`
- `config/profiles/dynamic_integration/mission.yaml`
- `config/profiles/dynamic_integration/recorder.yaml`

这个测试专门验证扩类运行时：mission / recorder 以 `class_names=[friendly, enemy, hostage, neutral]` 运作，fake detections 也必须显式携带相同的 `class_names` / `class_schema_hash`，检查：

- dynamic authoritative lane 保留扩展类
- `final_summary_v2.json` 中的 `class_names` / `totals_by_class` 正确
- legacy `final_summary.json` 仍只保留 `friendly/enemy/hostage` 兼容投影
- consistency gate 未被误触发
- 终态必须为 `FINISHED`，不再接受 `FAULT` 作为“测试通过”
- 另提供 namespaced 变体，验证 dynamic schema 链路下 namespace → topic / output root 的派生一致性

### 9.7 dynamic schema mismatch smoke

- `tests/dynamic_schema_mismatch_smoke.test`
- `tests/dynamic_schema_mismatch_smoke.py`
- `scripts/fake_detection_array_publisher.py`

这个负向集成测试使用与正向用例相同的 fake detection fixture，但故意把 embedded `class_names` 改成与 mission 配置不一致的顺序/集合，专门验证：

- mission 会发布 `class_schema_mismatch` health 诊断
- mission 终态会进入 `FAULT`
- mismatch 发生在 zone capture 之前，不会误产出 dynamic zone 结果
- 异常路径仍沿用正式 `DetectionArray` contract，而不是额外的 mock 分支

## 10. 兼容性说明

- `recon/zone_counts` 仍存在，但语义明确为 **frame-region counts**
- `recon/mission_state`、`recon/zone_capture_result` 的 JSON 话题仍保留
- recorder 默认 `auto`：typed lane 一旦可用即优先；health 的 typed/JSON 选择从“整条 health 流”收紧为“按 node 粒度”
- legacy `ZoneCapture.msg` / `final_summary.json` 退化为兼容投影；新的 authoritative zone/result contract 位于 dynamic lane 与 v2 artifact
- `MissionState.msg` 保留 `details_json` 兼容尾巴，但常用字段已收紧为固定 typed 字段
- `embed_zone_results_in_state` 默认关闭，避免 mission event 膨胀

## 11. 证据等级与已知边界

当前测试与运行材料建议按以下证据等级理解：

- **contract smoke**：验证节点 contract、消息结构、终态封存与工件生成，不证明真实导航/视觉语义
- **transport smoke**：验证 topic / action / artifact 流转，不必然证明任务区语义正确
- **semantic integration**：验证 route / frame-region / authoritative recorder 的语义闭环
- **hardware acceptance**：真实机器人、真实导航、真实地图、真实模型联调；当前仓库未提供

已知边界：

- 本仓库不内置真实导航栈、真实相机驱动、真实 ONNX 模型和真实赛道参数
- baseline route 中的坐标仍应由真实场地标定结果覆盖，不应直接视为比赛最终参数
- safety 当前仍是外围控制安全壳，不等价于 planner→controller→actuator 主控制闭环已经证明
- 默认 `color_blob + synthetic` 只证明 legacy 三类/其子集，不等价于任意动态扩类感知模型都已端到端打通
- 当前推荐支持环境收口为 Noetic/Focal 冻结线；跨发行版或跨 ROS 代际运行不在本轮保证范围内
- 本轮已补 baseline semantic integration 与 dynamic schema integration 夹具，但当前压缩包内仍未包含真实地图、真实 `move_base`、真实 AMCL/TF 和真实模型权重
- 仓库已保留 `docs/` 目录用于补充材料，但 README 仍是主要设计、运行与迁移说明入口


### 5.7 baseline legacy profile（保留旧未绑定 route 语义，便于回滚）

```bash
roslaunch ruikang_recon_baseline baseline_legacy.launch
```

这个入口保留旧的“route 不显式绑定 `frame_region`、vision 关闭 region adapter”的行为，仅用于回滚、兼容与对比验证，不再作为默认 deploy 主入口。

### 6.4 profile contract guard

- `profile_role: deploy|integration|contract|demo|legacy`
- `require_route_frame_regions` / `expected_frame_regions`
- `expected_region_names`
- `detector_schema_policy: subset_allowed|full`
- `tools/validate_profile_contracts.py`

`baseline_deploy.launch`、`field_deploy.launch`、`semantic_deploy.launch`、`baseline.launch`、`baseline_legacy.launch`、`baseline_contract.launch`、`demo_synthetic.launch` 与 `tests/baseline_integration_smoke.test` 现在都会进入 launch-composed contract 校验；其中 `semantic_deploy.launch` 被约束为 `deploy.launch` 的参数透明别名，`field_deploy.launch` 额外在 launch 层强制 `require_verified_field_asset=true`。route/region 合同继续按 profile 声明执行，deploy 角色还会额外校验 `lifecycle_managed=true` / `auto_start=false`。`baseline.launch` 保留兼容入口，`baseline_legacy.launch` 保留旧语义并作为回滚入口。

### 6.5 system manager / lifecycle gate

- `enable_system_manager`
- `lifecycle_managed`
- `control_command_topic`
- `config/common/system_manager.yaml`
- `config/profiles/baseline/system_manager.yaml`

`baseline_deploy.launch` 会把 vision / mission / recorder / safety / platform_bridge 全部切换到 `lifecycle_managed=true`；mission 同时强制 `auto_start=false`。`system_manager_node` 统一负责 `BOOTSTRAP -> READY -> ACTIVE -> PAUSED -> DEGRADED -> FAULT -> SHUTDOWN` 的外部控制语义，并通过统一 `control_command_topic` 按顺序下发 `configure -> activate -> pause -> reset -> shutdown`。deploy profile 现在还会通过 `profile_role`/`lifecycle_managed` 的组合校验 vision、mission、recorder、safety 的运行态合同，且 `platform_bridge_node` 已提升为一等受监管节点。system_manager readiness 继续要求 command/feedback/runtime 绑定成立，但不再把 pre-activation `command_traffic_seen` 当成 READY 前置条件。

### 6.6 safety 执行链反馈

- `output_feedback_topic`
- `output_feedback_timeout_sec`
- `require_output_feedback`

当外部 controller/driver 能回传执行链心跳时，`cmd_safety_mux_node.py` 会把 downstream feedback freshness 纳入 `recon/safety_state` 与 health 诊断。若 `require_output_feedback=true`，则缺失/过期反馈会在启动期校验 topic 配置，并在运行期把输出强制收敛为零速 fail-safe stop，避免“只发出了 `cmd_vel` 就误判执行链已活着”。

还提供了 `tests/baseline_deploy_feedback_timeout_smoke.test` 作为缺失反馈的负向集成夹具，用于证明 deploy safety 在无下游反馈时会进入 fail-safe stop。

## 2026-04 架构收口（executor / contracts / staged recorder）

本轮重构不再把 `scripts/mission_manager_node.py` 与 `scripts/mission_recorder_node.py` 当作主实现文件；它们现在只保留 ROS 入口职责，核心逻辑下沉到 `src/ruikang_recon_baseline/`：

- `mission_node.py`：ROS mission 节点装配层
- `mission_executor.py`：任务推进 / dispatch / dwell / timeout / finish 主链
- `mission_context.py`：运行时 blackboard / context
- `mission_plan.py`：基于 route 的任务计划模型；当前默认输出 `waypoint_capture` declarative step，给后续任务图/BT 演进预留扩展缝合层
- `recovery_policies.py`：navigation failure recovery policy
- `navigation_adapters/`：`move_base_action` / `simple_topic` / `goal_topic_status` 正式扩展边界
- `system_manager_core.py` / `system_manager_node.py`：栈级 lifecycle / supervisor 与 mission 控制命令发布
- `recorder_node.py`：ROS recorder 节点装配层
- `recorder_ingest.py`：typed / JSON lane ingest 与 compatibility shadow 边界层
- `recorder_finalize.py`：terminal finalize / rewrite 控制
- `artifact_builders.py`：snapshot / final_summary / CSV 构建
- `contracts/`：mission_state / zone_result / frame_region_counts / health / compatibility 稳定接口层

这次调整的目标不是引入第二套控制平面，而是把原有单体节点拆成更可审计、可测试、可迁移的内部边界。`launch/core.launch` 的运行入口和 topic contract 继续保持兼容。

### 新增治理文件

- `docs/upstream_alignment.md`：GitHub 顶级项目对齐原则与当前仓库吸收边界
- `UPSTREAM_SOURCES.md`：上游参考项目与用途登记
- `THIRD_PARTY_NOTICES.md`：第三方代码/许可证声明

### 当前事实边界

- 已完成：执行框架边界、contracts 包、staged recorder、wrapper 脚本化、纯 Python 单测回归
- 未完成：真实 Noetic/catkin/rostest 验证、真实 move_base/AMCL/TF/相机/模型联调
- 结论口径仍然是：**已静态确认 / 已沙箱验证 / 未真实环境验证**


- `tests/baseline_deploy_smoke.test`：针对 `baseline_deploy.launch` 的 supervisor/deploy 路径 smoke，用 fake navigation + synthetic scene + vision 真处理链验证 `system_manager -> vision/mission/recorder/safety/platform_bridge` 的受控 ACTIVE 闭环。
- `tests/baseline_profile_smoke.test`：针对 `baseline.launch` 的 compatibility 路径 smoke，直接检查 `mission_recorder_node` 在 `recon/health_typed` 上发布 `lifecycle_managed=false` 且 `runtime_state=ACTIVE`，避免 compatibility 入口再次漂移成 managed/IDLE。


## Deploy contract additions

- platform bridge now relays `upstream_command_topic -> command_input_topic` so `cmd_vel_raw` becomes the single safety ingress.
- system manager READY now depends on resource readiness details rather than only node liveness.
- recorder can emit `official_report.json` when `official_report_mode=artifact`.
- field assets now carry `asset_version`, `asset_state`, `verified`, and calibration/layout ids.
- deploy/runtime related keys now documented in config and README: `official_report_mode`, `official_report_schema`, `require_verified_field_asset`, `camera_ready_timeout_sec`, `upstream_command_topic`, `command_input_topic`.


## Deploy/runtime contract hardening

- `required_field_asset_verification_scope`：deploy 与 field_deploy 现在区分 `contract` / `field` 两级验证门禁；`field_deploy.launch` 强制 `required_field_asset_verification_scope=field`。
- `vendor_runtime_mode`：平台运行时策略显式化，当前内置 `native_noetic` 与 `isolated_legacy_workspace`。
- `vendor_runtime_contract_path`：当 profile 使用 `isolated_legacy_workspace` 时，必须指向一份外部 vendor runtime contract YAML；mission / vision / platform_bridge 会在启动期按 domain 校验必需绑定与运行时版本约束。
- `motion_model`：平台运动学真相源显式化，当前基线使用 `differential`，MO-SERGEANT 使用 `mecanum_holonomic`。
- `cmd_vel_semantics`：速度语义与运动学绑定，分别约束 `planar_x_yaw` 与 `planar_xy_yaw`。
- `allow_odom_feedback_fallback`：只允许 integration/特定平台把 odom 当执行反馈兜底；deploy 型 MO-SERGEANT 默认关闭，只把 odom 作为 observability。

### Detector / asset / runtime defaults

- contract deploy profile 默认使用 `mowen_raicom_contract_verified` 合同级场地资产，并要求 `require_verified_field_asset=true`；`field_deploy.launch` 改用专用 field profile，不再继承 contract 级 field asset / detector manifest 默认值，并额外要求 field-scoped asset 与 field-scoped detector manifest grade。
- baseline deploy vision 现在要求 `model_manifest_path` 非空，并通过 manifest 绑定 detector schema。
- `tools/validate_profile_contracts.py` 现已同时校验平台 runtime strategy、detector manifest、field asset verification scope 与 launch 入口透明传参。


## 2026-04-09 command-path hardening

- `platform_bridge_node` now validates `upstream_command_topic`, `command_input_topic` and `safety_output_topic` cannot alias the same ROS resource, preventing bridge/safety self-loops in deploy profiles.
- bridge/safety health now expose `command_traffic_seen` for post-activation observability, but deploy readiness no longer blocks on pre-activation command traffic because many vendor planners only emit `cmd_vel` after a goal is active.
- baseline and integration-style platform profiles now use `recon/platform/vendor/cmd_vel` as the default upstream bridge ingress, while vendor runtime launch may still override this explicitly when integrating a namespaced vendor graph.


- Navigation runtime live readiness now requires action status freshness plus post-dispatch action feedback/result roundtrip evidence when a goal is active.


### Field asset review tool

- `tools/render_field_asset_regions.py`：按 field asset 与 `region_calibration` 渲染 review PNG，用于赛场资产人工复核。


## Competition task runtime extensions

The mission runtime now supports competition-facing declarative task types in addition to the legacy `waypoint_capture` / `waypoint_only` pair:

- `recon_zone`
- `transit`
- `hazard_avoid`
- `facility_attack`
- `finish`

Legacy task types remain accepted and are projected onto the richer execution model so old route-only profiles stay runnable.

### Competition evidence and reports

Dynamic task artifacts now preserve task semantics and evidence fields in `zone_results_dynamic` and the alias `task_results_dynamic`, including:

- `task_type`
- `objective_type`
- `mission_outcome`
- `position_estimates`
- `evidence_summary`
- `hazard_summary`
- `action_summary`

`summary_snapshot_v2` / `final_summary_v2.json` now also expose:

- `totals_by_task_type`
- `totals_by_objective`
- `hazard_observations`
- `facility_actions`

The recorder additionally supports `acceptance_stage` and a richer official-report contract. `official_report_mode` now accepts `disabled`, `artifact`, or `judge_contract`. `judge_contract` now requires `official_report_contract_path` and uses a contract-driven submission adapter (`file_drop`, `http_post`, or `command_exec`) that emits a submission receipt. `official_report_sink_path` is optional shadow-copy output in this mode.

### Acceptance staging

Recorder profiles are now split by acceptance intent instead of reusing one deploy recorder profile everywhere:

- `config/profiles/baseline/recorder.yaml` → `acceptance_stage: deploy_managed`
- `config/profiles/reference_deploy/recorder.yaml` → `acceptance_stage: reference_field_dryrun`
- `config/profiles/field_deploy/recorder.yaml` → `acceptance_stage: competition_field_acceptance`

This keeps contract smoke, reference dry-run, and field acceptance artifacts distinguishable in final deliverables.


### Typed detection transport extensions

`Detection.msg` now carries richer competition-facing evidence fields end-to-end:

- `observed_position_type`
- `observed_position_label`
- `observed_position_x_m`
- `observed_position_y_m`
- `evidence_source`

This prevents the typed detection lane from collapsing richer position evidence back to just `bbox + frame_region`.


### Operator intervention audit

`system_manager_node` now emits explicit runtime-evidence events for:

- `manual_command`
- `control_mode_changed`
- `estop_changed`

`mission_recorder_node` folds these into `operator_interventions` inside `summary_snapshot_v2`, `final_summary_v2`, and `official_report.json` so deploy profiles can audit human takeover, control-mode transitions, and estop activations.


## Deployment entrypoints

- `contract_deploy.launch`: contract-gated deploy smoke.
- `reference_field_deploy.launch`: reference-field gated managed deploy.
- `real_field_deploy.launch`: canonical real-field deploy entry with synthetic disabled and field-grade asset/model gates.
- `field_deploy.launch`: compatibility alias to `real_field_deploy.launch`.


## Behavior runtime and field release production-line updates

- Added `behavior_runtime_node.py` as the repository-managed downstream execution runtime. It consumes `recon/platform/behavior_execution/request`, publishes concrete commands on `cmd_vel_raw` and `recon/platform/behavior_execution/actuator_command`, observes `recon/platform/bridge_state` and `recon/detections`, and emits explicit terminal receipts on `recon/platform/behavior_execution/result`.
- Added `tools/validate_managed_vendor_bundle.py` so managed vendor bundles can be validated against one concrete external workspace before deploy handoff. CI/Noetic verification now validate one external managed vendor workspace bundle. When `RUIKANG_VENDOR_WORKSPACE_ROOT` is not provided, the workflows provision an external managed workspace bundle from the repository templates first; the repository-local fixture itself is never accepted as an external workspace unless an explicit local allow flag is used.
- Added `tools/build_field_asset_release.py` so measured/reference/contract field asset release manifests can be materialized from explicit inputs instead of hand-editing YAML.


- Added the full in-repository actuator confirmation chain: `behavior_actuator_node.py` -> `vendor_actuator_bridge_node.py` -> `vendor_actuator_device_node.py` -> `vendor_actuator_feedback_node.py`. `facility_attack` success now depends on actuator-specific raw result confirmation rather than generic `base_feedback_raw`. This closes the repository-level produced/consumed gap and removes the prior timer-based self-confirmation path. It is still a repository-controlled integration chain: real hardware execution remains subject to external vendor workspace, ROS runtime, and device-side confirmation in the target environment.


## Competition perception + chassis control additions

This repository now includes a competition-oriented semantic perception layer and a managed chassis control path for the RAICOM reconnaissance workflow.

- Added `competition_perception_node` to convert raw detector outputs into mission-facing semantic classes such as `enemy_soldier`, `friendly_soldier`, `unknown_soldier`, and `confirmed_mine`.
- Added deploy entrypoints `reference_competition_deploy.launch` and `field_competition_deploy.launch` so competition semantics can be enabled without breaking the baseline/reference/field profiles.
- Added `mowen_serial_protocol.py` and `mowen_serial_bridge_node.py` to formalize the upper-computer serial protocol against the documented `newt.py` vendor bridge layout and a managed CRC16 extension path.
- Added `firmware/mowen_chassis_controller/` as a portable PlatformIO-oriented firmware core covering frame parsing, mecanum wheel solving, PID axis control, and telemetry encoding for the M4 chassis controller.

These additions improve code-level closure for competition semantics and managed chassis communication. The repository now also includes an AT32-targeted firmware implementation with startup, linker script, board-level UART/PWM/encoder/ADC/watchdog porting, and a reproducible target build script; however, on-board flashing and full electromechanical validation still require a real device environment.
The AT32 target implementation now also carries firmware-side ownership for voice, ultrasonic, and PS2 inputs, and the historical direct vendor navigation/mapping launches require explicit external lidar/IMU driver launches instead of silently no-op wrapping them.


## 8.2 AT32 firmware target build

仓库现在除了 `native_ref` runtime core 之外，还提供 `firmware/mowen_chassis_controller/at32/` 目标固件树以及 `tools/build_at32_firmware.sh`。

```bash
bash tools/build_at32_firmware.sh
```

该脚本会产出 `mowen_chassis_at32.elf` / `mowen_chassis_at32.bin`，用于 AT32F403AVCT7 底盘控制器的目标构建。

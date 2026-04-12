# 实施摘要（复核后收口版）

本轮代码落地后的真实收口目标不是“把外部 vendor 旧栈伪装成已经完全迁到 Noetic/Python3”，而是把仓库内部**能被本仓控制的运行约束、场地资产门禁、任务编排主链、平台反馈语义、导航 contract、验证与文档**全部做成不可绕过的硬约束。

## 一、已落实项

补充保留项：既有 `mission_executor.py` 主执行编排、`recorder_ingest.py` / finalize 分层、`contracts/` 契约资产与 `THIRD_PARTY_NOTICES.md` 第三方来源说明均保持可达且未被本轮重构打散。本摘要是实现收口说明，不等价于真实环境验收报告。


### P0
- **P0-01 运行时策略硬约束**：`vendor_runtime_mode`、`vendor_workspace_ros_distro`、`vendor_workspace_python_major`、`motion_model`、`cmd_vel_semantics`、`allow_odom_feedback_fallback` 已进入 platform contract，并在 mission/safety/bridge 启动期校验。
- **P0-01b 外部 vendor runtime contract**：`isolated_legacy_workspace` profile 现在必须声明 `vendor_runtime_contract_path`，由 mission / vision / platform_bridge 分域校验外部工作空间绑定与版本约束。contract 还会显式携带 `vendor_workspace_name`、`vendor_entrypoints`、`vendor_resource`、`bridge_output` 等证据字段，并在 health 中汇总为 `vendor_runtime_binding_report`。
- **P0-01c vendor bundle 按入口分级**：真实 vendor/legacy/reference/field 入口继续使用 `vendor_bundle_preflight_mode=required`；但仓库自带的 `deploy.launch` / `baseline_deploy.launch` 保持 contract smoke 定位，因此 baseline 平台 profile 下调为 `advisory`，并通过 `config/profiles/baseline/system_manager.yaml` 去掉 `vendor_bundle_preflight_satisfied` 的 ready 强约束，避免仓库自身的 Noetic smoke/rostest 被默认外部依赖阻断。
- **P0-03 runtime probe 收口**：`runtime_probes.py` 改为对解析后的绑定名做精确匹配，graph/semantic/mission 三阶段 readiness 不再接受其他命名空间或其他机器人图中的 suffix 同名话题。
- **P0-02 deploy / field asset 门禁**：`require_verified_field_asset=true` 时，缺省不填 `field_asset_id/field_asset_path` 也会直接失败；`field_deploy.launch` 强制 `required_field_asset_verification_scope=field`，并默认进入 `config/profiles/field_deploy/platform.yaml` / `system_manager.yaml`，不再误走 generic baseline 平台 profile。仓库新增 `mowen_raicom_reference_field_verified` 与 `mowen_raicom_packaged_field_verified` 两条受控资产线：前者限定给 `reference_field_deploy.launch`，后者通过 `mowen_raicom_packaged_field_release` 提供 field_deploy 的仓内代码闭环输入；真实比赛前仍必须替换为实测 field 资产。
- **P0-03 detector / manifest / schema 闭环**：baseline deploy profile 已要求 detector manifest；profile validator 会检查 manifest 文件存在且 deploy profile 不再靠空模型 contract 冒充闭环。
- **P0-03b ONNX deploy 主路径**：`reference_deploy` / `field_deploy` 视觉 profile 已切到 `detector_type=onnx`，并通过环境变量收口模型路径；同时新增 packaged field detector manifest 以完成 field 级 deploy contract 装配。`color_blob` 仅保留为 contract/debug fallback。
- **P0-02b field asset release 收口**：field asset 不再只是一张裸 YAML；mission / vision 新增 `field_asset_release_manifest_path`，release 会把 route、named_regions、map、calibration 与 detector scope 作为单一发布单元校验。
- **P1-02b task DSL 落地**：reference / field deploy 新增 `task_dsl_path`，运行时会基于 authoritative route 降低出 `tasks`，并允许 deploy 线同时保留 route 真值与 task graph。
- **P0-02 / P1-02 / P1-03 / P2-02 贯通**：新增 `behavior_actions.py` 与 `navigation_execution.py`，将 `hazard_avoid`、`facility_attack(command_confirmed)` 升级为显式动作后端；mission / recorder / system_manager / platform_bridge 统一传播 `runtime_grade`；platform bridge 接入 ultrasonic / IMU / voice / PS2 并产出 runtime evidence；仓库新增 `tools/validate_acceptance_tiers.py` 固化 contract / reference / field 三级验收阶梯。

### P1
- **P1-01 feedback 语义收硬**：`platform_bridge_core.py` 不再把 odom heartbeat 默认等价为执行反馈；MO-SERGEANT deploy 默认要求显式 `recon/platform/base_feedback`，odom 仅可在允许时作为 fallback 或 observability。
- **P1-02 task graph 基础模型**：mission plan / executor 已支持 `step_id`、`next_step_id`、`failure_step_id`、`retry_limit`、`quiesce_sec`，不再只有线性 waypoint list。
- **P1-03 navigation contract 显式化**：mission 配置现在显式声明并校验：
  - `navigation_planner_backend`
  - `navigation_controller_backend`
  - `navigation_recovery_backend`
  - `navigation_localization_backend`
  - `navigation_goal_transport`
  - `navigation_status_transport`
  - `navigation_cancel_transport`

  这些字段不再只是 health 摘要描述，而是启动期硬 contract；`navigation_adapter_type`、`pose_source_type`、topic/action 绑定与 transport 现在必须相互一致。运行时也不再只是假定一个单体 adapter，而是显式组合成 dispatcher / status monitor / canceller 三个角色。
- **P1-04 平台运动学真相源冻结**：generic ROS 与 MO-SERGEANT 两条平台线的 motion model / cmd_vel semantics 已在 platform adapter 中固定，不再允许 profile 任意漂移。

### P2
- **P2-01 校验与测试**：新增 navigation/runtime probe 语义阶段校验、vendor binding evidence 校验，以及 field-deploy 默认 profile 一致性校验；profile validator 现在同时校验 platform contract 与 navigation contract。
- **P2-02 文档同步**：README、`docs/mowen_vendor_integration.md`、本摘要已按当前代码状态同步；`mowen_integration.launch` 与 `config/profiles/mowen_integration/*.yaml` 现在是规范墨问集成入口，`baseline_integration` 仅保留为兼容别名。

## 二、仍然不能夸大的边界

以下事项**没有在本仓内被伪装成已完成**：
- 墨问 vendor 专有工作空间整体迁移到 Noetic/Python3
- 真实底盘 / 雷达 / 相机 / IMU / move_base / map / amcl 联调
- `catkin_make` / `rostest` / 实车 bringup / 实场地图 / 真实 detector 模型验证

仓库当前给出的是真正可审计的**代码与配置基线**，不是“整车实战已完全交付”的虚假闭环。

## 三、验证边界

### 已静态确认
- mission / platform / safety / field asset / navigation contract 主链已收口
- 文档、launch、profile、validator 已同步

### 已沙箱验证
- `python3 -m compileall -q src scripts tools`
- `PYTHONPATH=src python3 -m pytest -q tests/test_*.py`
- `python3 tools/validate_profile_contracts.py`
- `python3 tools/validate_field_asset_release.py`
- `python3 tools/validate_launch_contracts.py`
- `python3 tools/validate_acceptance_tiers.py`

### 未真实环境验证
- `catkin_make`
- `rostest`
- 真实 Noetic ROS graph
- 真实墨问 vendor 工作空间
- 真实硬件与现场资产


## 四、兼容入口与既有约束

- 兼容入口继续保留：`baseline.launch`、`baseline_deploy.launch`、`baseline_contract.launch`、`baseline_legacy.launch`。其中 `baseline_legacy.launch` 只作为旧语义回滚入口，不承担新的 deploy 门禁语义。
- detector 侧既有 `detector_schema_policy` / manifest/schema 约束没有被回退，只是现在由更硬的 contract 和 validator 接管。
- safety 侧 `output_feedback_topic` 仍是 deploy 线路的重要 contract；MO-SERGEANT deploy 默认要求显式 feedback，而不是 odom 冒充执行反馈。
- profile 与 launch 一致性继续通过 `validate_profile_contracts.py` 做静态校验。


## 2026-04-09 command-path hardening

- `platform_bridge_node` now validates `upstream_command_topic`, `command_input_topic` and `safety_output_topic` cannot alias the same ROS resource, preventing bridge/safety self-loops in deploy profiles.
- bridge/safety health now expose `command_traffic_seen` for post-activation observability, but deploy readiness no longer blocks on pre-activation command traffic because many vendor planners only emit `cmd_vel` after a goal is active.
- baseline and integration-style platform profiles now use `recon/platform/vendor/cmd_vel` as the default upstream bridge ingress, while vendor runtime launch may still override this explicitly when integrating a namespaced vendor graph.


## 2026-04-09 r3
- Added hard deploy-grade separation between contract/reference/real-field assets and manifests.
- Introduced `reference_field_deploy.launch`; `mowen_vendor_runtime.launch` now uses the managed reference deploy line instead of pretending the in-repo reference asset is a real field asset.
- Productized external legacy vendor dependency boundaries via `vendor_bundle_preflight.py` and surfaced bundle reports in mission/vision/platform bridge health details.
- Strengthened navigation readiness with runtime-live action status freshness checks.


- Navigation runtime live readiness now requires action status freshness plus post-dispatch action feedback/result roundtrip evidence when a goal is active.


## 2026-04-09 hardening

- Added `navigation_backend_profile` as an explicit runtime contract.
- Added managed vendor sidecar entrypoints and repo-visible preflight checks.
- Added canonical `real_field_deploy.launch` and preserved `field_deploy.launch` as an alias.
- Added `runtime_metrics.json` artifact and stronger submission-contract identity checks.

# ruikang_recon_baseline

面向“智能侦察”类比赛任务的 ROS1（Noetic）工程基线，当前版本已按架构整改完成以下主线修正：

- **任务区语义与图像 ROI 解耦**：视觉只输出帧级检测与可选 frame-region 统计，物理任务区由 mission route 驱动。
- **导航反馈显式化**：baseline 主线使用 `move_base` action；同时新增 `goal_topic_status` 适配器，用于非 `move_base` 但有显式状态流的导航栈。
- **工程化闭环**：新增自定义消息、配置分层、异步落盘、健康度输出、速度仲裁与纯 Python 测试。

> 该工程仍然**不伪造官方专有 SDK、裁判系统协议和未公开地图参数**。它提供的是一套可直接接真实 ROS1 导航栈、相机和识别模型的规范基线。

## 1. 当前目录结构

```text
ruikang_recon_baseline/
├── .github/workflows/ci.yml
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── msg/
│   ├── Detection.msg
│   ├── DetectionArray.msg
│   ├── MissionState.msg
│   └── ZoneCapture.msg
├── config/
│   ├── common/
│   ├── profiles/
│   │   ├── baseline/
│   │   └── demo/
│   └── models/
├── launch/
│   ├── baseline.launch
│   └── demo_synthetic.launch
├── scripts/
│   ├── vision_counter_node.py
│   ├── mission_manager_node.py
│   ├── mission_recorder_node.py
│   ├── cmd_safety_mux_node.py
│   └── synthetic_scene_publisher.py
├── src/ruikang_recon_baseline/
│   ├── common.py
│   ├── vision_core.py
│   ├── mission_core.py
│   ├── safety_core.py
│   └── io_core.py
└── tests/
    ├── test_core_logic.py
    ├── test_repository_consistency.py
    ├── demo_profile_smoke.test
    └── demo_pipeline_smoke.py
```

## 2. 节点职责

### 2.1 `vision_counter_node.py`
- 输入：相机图像。
- 输出：
  - `DetectionArray` typed 话题 `/recon/detections`
  - JSON 兼容话题 `/recon/detections_json`
  - frame-region 统计 `/recon/zone_counts`（保留原话题名，语义改为 **frame_region_counts**）
  - 调试图像 `/recon/overlay/image_raw`
  - 健康度 `/recon/health`
- 说明：视觉节点**不再直接给出物理区最终结果**。

### 2.2 `mission_manager_node.py`
- 输入：
  - 帧级检测 `/recon/detections`
  - 位姿源（`/amcl_pose`、`/odom` 或 TF 查找）
  - 导航反馈（`move_base` action、simple topic 或 `goal_topic_status`）
- 输出：
  - 任务状态 JSON `/recon/mission_state`
  - typed 任务状态 `/recon/mission_state_typed`
  - 区域结果 JSON `/recon/zone_capture_result`
  - typed 区域结果 `/recon/zone_capture_result_typed`
- 说明：物理区结果在 **驻留窗口** 内聚合，只有这里才会把检测结果绑定到实际任务区。

### 2.3 `mission_recorder_node.py`
- 记录任务状态、区结果、frame-region 统计、健康度。
- 异步写入 `mission_log.jsonl`，并生成：
  - `summary_snapshot.json`
  - `final_summary.json`
  - `final_summary.csv`

### 2.4 `cmd_safety_mux_node.py`
- 支持多源速度输入、优先级仲裁、速度限幅、命令超时、急停 freshness 检查。
- 输出：
  - `/cmd_vel`
  - `/recon/safety_state`
  - `/recon/health`

### 2.5 `synthetic_scene_publisher.py`
- 为 `demo_synthetic.launch` 生成稳定可复现的俯视合成画面。
- 仅服务 demo profile，不代表真实比赛机位。

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
sudo apt install -y   ros-noetic-cv-bridge   ros-noetic-image-transport   ros-noetic-actionlib   ros-noetic-actionlib-msgs   ros-noetic-geometry-msgs   ros-noetic-nav-msgs   ros-noetic-sensor-msgs   ros-noetic-std-msgs   ros-noetic-move-base-msgs   ros-noetic-tf2-ros   python3-opencv   python3-numpy
```

## 4. 编译

```bash
cd ~/catkin_ws/src
cp -r /path/to/ruikang_recon_baseline .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 由于当前对话运行环境不带 ROS Noetic，我已完成的是：
> - 全部 Python 文件语法校验
> - 纯 Python 单元测试
> - launch / config / message / README 闭环检查
- 纯 Python 单元测试与静态仓库一致性测试
> 但**没有在本对话容器中实际运行 catkin_make 或 roslaunch**。

## 5. 启动方式

### 5.1 demo profile（纯合成闭环）

```bash
roslaunch ruikang_recon_baseline demo_synthetic.launch
```

这个 profile 的特性：
- 视觉启用 `named_regions` frame adapter
- mission 使用 `simple_topic` 导航适配器
- 允许 `simulate_arrival_without_pose`

### 5.2 baseline profile（接真实导航栈）

```bash
roslaunch ruikang_recon_baseline baseline.launch
```

这个 profile 的特性：
- 视觉不做 ROI -> 物理区绑定
- mission 使用 `move_base` action
- 位姿源默认 `/amcl_pose`，也支持 `tf_lookup`
- 安全链默认 `require_fresh_estop: true`

## 6. 配置分层

- `config/common/*.yaml`：节点通用参数。
- `config/profiles/baseline/*.yaml`：真实链路默认参数。
- `config/profiles/demo/*.yaml`：演示链路默认参数。
- `config/models/onnx_manifest_example.json`：ONNX 模型 manifest 示例。

### 6.1 物理任务区与 frame-region 的关系

- `mission.route[*].name`：**物理任务区名称**。
- `mission.route[*].frame_region`：驻留窗口中可选的**图像局部过滤器**。
- `vision.named_regions`：只描述图像里的局部区域，不再代表物理导航区。

真实赛道通常设置为：

```yaml
frame_region: ""
```

也就是在当前相机视野内直接统计，而不是预先把整场硬切成四块像素框。

## 7. ONNX 模型切换

在 `config/common/vision.yaml` 或 profile 覆盖中配置：

```yaml
detector_type: "onnx"
onnx_model_path: "/abs/path/to/model.onnx"
model_manifest_path: "$(find ruikang_recon_baseline)/config/models/onnx_manifest_example.json"
strict_backend: true
```

- `strict_backend: true`：模型不可用时直接启动失败。
- `strict_backend: false`：仅 demo 允许回退到 `color_blob`。

## 8. 输出文件

默认目录：

```text
~/.ros/ruikang_recon/
```

主要文件：
- `vision_events.jsonl`
- `mission_manager_events.jsonl`
- `mission_log.jsonl`
- `summary_snapshot.json`
- `final_summary.json`
- `final_summary.csv`
- `annotated_*.jpg`

## 9. 测试

### 9.1 纯 Python 单元测试

```bash
cd /path/to/ruikang_recon_baseline
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py'
```

### 9.2 demo profile rostest smoke

仓库包含：
- `tests/demo_profile_smoke.test`
- `tests/demo_pipeline_smoke.py`

用于在具备 ROS Noetic / rostest 的环境里对 `demo_synthetic.launch` 做话题级冒烟验证。

### 9.3 CI

仓库包含：
- `.github/workflows/ci.yml`
  - Python 单元测试
  - `industrial_ci` 的 ROS Noetic 构建任务

## 10. 兼容性处理

本次改造保留了以下旧接口，避免下游一次性全断：

- `/recon/zone_counts` 仍存在，但语义已明确为 **frame-region counts**，不再被 mission manager 当作物理区最终答案。
- `/recon/mission_state` 仍保留 JSON 版本。
- 新增 typed 话题与 JSON 兼容双轨输出，便于旧工具链继续消费。

## 11. 位姿源与导航适配扩展

### 11.1 TF 位姿源

当位姿源配置为 `tf_lookup` 时，mission manager 会周期性查询：

```yaml
pose_source_type: "tf_lookup"
tf_target_frame: "map"
tf_source_frame: "base_link"
tf_lookup_timeout_sec: 0.2
```

这样 simple-topic 模式也可以在统一 frame 下做真实到达判定，而不是被迫依赖 `/odom` 或 `/amcl_pose` 的 frame 恰好相同。

### 11.2 非 move_base 导航状态适配

若你的导航栈不提供 `move_base` action，但能提供显式状态 topic，可配置：

```yaml
navigation_adapter_type: "goal_topic_status"
simple_goal_topic: "/your/navigation_goal"
navigation_status_topic: "/your/navigation_status"
navigation_status_timeout_sec: 3.0
```

其中状态 topic 需发布以下规范化字符串之一：

- `PENDING`
- `ACTIVE`
- `SUCCEEDED`
- `PREEMPTED`
- `ABORTED`
- `IDLE`

## 12. 已知边界

1. 当前工程没有实现官方未公开的裁判系统协议。
2. 当前工程已支持 TF 位姿查找，但**本对话环境未实跑 ROS TF 树**。
3. 若你的真实导航栈既不是 `move_base` action，也没有显式状态 topic，仍需继续补对应 adapter。

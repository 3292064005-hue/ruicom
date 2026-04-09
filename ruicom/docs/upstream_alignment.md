# GitHub 顶级项目对齐说明

本仓库本轮对齐的重点是**结构与执行模型**，不是整仓复制外部项目实现。

## 对齐目标

- BehaviorTree.CPP：任务表达与执行分层思路
- Nav2：navigator / recovery / adapter seam 思路
- SMACC2：事件驱动状态机与可扩展行为组织思路
- Aerostack2：planner / executor / behavior 分层思路
- TurtleBot3 Autorace：竞赛型仓库的分包与运行入口组织方式

## 本轮实际吸收范围

- 将 mission 主链拆为：`mission_node.py` + `mission_executor.py` + `mission_context.py` + `mission_plan.py` + `recovery_policies.py`
- 将导航后端拆为：`navigation_adapters/`
- 将 recorder 拆为：`recorder_node.py` + `recorder_ingest.py` + `recorder_finalize.py` + `artifact_builders.py`
- 将 contract 稳定接口显式化为：`contracts/`

## 未做的事

- 未复制 Nav2 / BehaviorTree.CPP / SMACC2 / Aerostack2 的整仓代码
- 未把 ROS2/C++ 框架硬塞进当前 ROS1/Python 主链
- 未把静态验证夸大为真实环境可交付

## 迁移原则

1. 先吸收执行模型与边界，再评估局部工具/测试脚手架复用。
2. 任何未来引入的第三方源码，都必须同步登记到 `UPSTREAM_SOURCES.md` 与 `THIRD_PARTY_NOTICES.md`。
3. 若未来真的引入上游源码，必须记录来源仓库、版本/commit、许可证、修改范围、保留声明。

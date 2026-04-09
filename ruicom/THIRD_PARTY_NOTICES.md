# Third-Party Notices

## Current round status

- 本轮未直接复制第三方源码文件进入仓库。
- 当前新增的 `mission_executor.py`、`mission_context.py`、`mission_plan.py`、`recovery_policies.py`、`navigation_adapters/`、`recorder_ingest.py`、`recorder_finalize.py`、`artifact_builders.py`、`contracts/` 均为本仓库内的重构实现。

## License hygiene rule

若未来引入第三方源码，必须同步满足：

1. 保留原始版权声明与许可证文本。
2. 在 `UPSTREAM_SOURCES.md` 登记来源仓库、版本/commit、用途、修改范围。
3. 在 README 或专门文档中说明迁移边界与兼容影响。

# UI 无关刀路工作流开发规划

> 文档状态：执行中
>
> 当前主阶段：W3——统一批次结果与发布前置条件
>
> 当前任务键：`W3/batch-contract`
>
> 代码审计基线：`612ab76`
>
> 基线日期：2026-08-24
>
> 最近验证：2026-11-08，RelWithDebInfo 完整构建成功，CTest 54/54 通过（含中立材料执行、取消和并行预算回归）
>
> 维护角色：`oap_toolpath_workflow` 目标维护者

本文维护识别/区域输入到完整刀路批次的 UI 无关编排。桌面对话、进度显示和工作区提交由 [App Workflows](../../app/workflows/README.md) 负责；Job 与 IPW 分别由 [manufacturing/job](../../manufacturing/job/DEVELOPMENT_PLAN.md) 和 [manufacturing/ipw](../../manufacturing/ipw/DEVELOPMENT_PLAN.md) 负责。

## 当前基线

`compatibility/LegacyProfileSimulationAdapter` 明确承接旧 BRep 和显示消费者需要的
Profile→Pocket 载体转换。它归属同时组合两个领域的 workflow 层；Profile 内核仅
提供中立执行视图，原生 IPW 不经过此兼容桥。该桥仍是待迁移边界，不能作为新内核
接口继续扩散。

2026-11-08 调度收敛：Pocket/Profile/Hole 的同步服务入口与原生 worker 传播公共
`ToolpathParallelScope`，显式零预算覆盖安全准备和通用验证；独立任务使用稳定索引预算
包装。该 scope 不延长输入或回调生命周期，不代替既有取消、异常和发布门禁。
QThread/future 继续复用已有快照、等待与过期结果处理。主要重计算根请求在启动前
通过 `HeavyJobScheduler` 共用一个执行槽和最多 32 个 FIFO 待执行项；自动选刀 Runner、
识别与仿真也接入同一队列，排队取消不会执行几何。异步服务返回的句柄析构会取消并等待，
显式零预算、已有取消/诊断及固定顺序合并契约保持不变。队列只保证启动顺序，不保证抢占公平。

2026-11-08 服务结构试点：三领域共用私有 `detail/ToolpathServiceSupport.h`
参数准备和算法诊断构造；Profile 的生成与评估实化复用区域租约、回调串行化和
安全结果处理。Hole 的输入映射/覆盖审计、Pocket 的逐区结果、Profile 的有序合并
仍由各服务负责。归一化必须在转移诊断前读取有效性，修复 Profile 无效参数仍调用
算法的问题。此轮没有引入公共工序基类，也没有完成 W3 的 Formal/Preview 证据模型。

`workflow_contract` 通过注入算法验证诊断顺序、异常映射、部分成功、取消、孔输入
映射和显式预算传播；`profile_invalid_parameters` 独立验证错误参数阻止算法调用。
重构前旧库通过前者、未通过后者；重构后两者均通过。完整 CTest 53/53 通过，
构建及回归日志保存在本地 `build/structure-refactor-build.log` 与
`build/structure-refactor-ctest.log`。

| 能力 | 当前实现 | 状态 |
| --- | --- | --- |
| 服务 | Hole、Pocket、Profile 专用服务和完整批次结果 | 已完成 |
| 总入口 | `ToolpathWorkflow` 组合区域构建、算法选择和领域服务 | 已完成 |
| 算法注入 | 只读 Registry + Selection，不直接构造具体算法 | 已完成 |
| 程序适配 | 专用 batch 到中立程序 adapter 聚合 | 已完成 |
| 异步 | `ToolpathAsyncExecutor`、全局并行预算、进度和协作取消 | 已完成 |
| 孔批次 | 独立 `HoleBatchExecutor` 和仿真适配 | 已完成 |

## 不可破坏的契约

1. Workflow 不访问 Widget、MainWindow、项目文件或展示对象。
2. 输入指针/引用的生命周期由同步调用或异步请求值明确冻结，后台不读可变 UI 状态。
3. 算法 unavailable、取消、验证失败和程序适配失败使用不同稳定结果。
4. 批量操作定义是否允许部分成功；正式发布默认 fail-fast，不由 UI 猜测。
5. 材料依赖通过显式前序输入表达。
6. 进度事件单调、取消有界响应；任务结束后不继续调用呈现回调。
7. Workflow 只返回候选/已验证值，不自行安装 Job、IPW、预览或 NC 文件。

## 路线图

| 阶段 | 优先级 | 状态 | 交付 |
| --- | --- | --- | --- |
| W0 专用服务 | P0 | 完成 | 三领域服务、批次和程序适配 |
| W1 注册表注入 | P0 | 完成 | 算法选择与 unavailable 诊断 |
| W2 异步执行 | P0 | 完成 | 进度、取消和并行预算 |
| W3 批次契约收敛 | P0 | **下一阶段** | 公共状态/证据头、Formal/Preview 前置条件和一致诊断 |
| W4 依赖感知执行 | P1 | 待实施 | Job operation DTO、父 IPW 身份和可重建请求 |
| W5 调度与性能 | P1 | 待实施 | 公平并行预算、任务指标和大批次背压 |

## W3 实施重点

- 为三类批次统一最小公共状态：cancelled、validation、diagnostics、operations、输入/验证 fingerprint 和发布级别；
- 保留各领域专属证据，不用一个布尔 `ok` 淹没失败阶段；
- 明确 Preview 可返回带诊断候选，Formal 必须通过完整证据、程序和材料前置条件；
- 统一异步请求的值冻结规则，禁止后台持有临时 UI 容器指针；
- 让 App UseCase 只负责事务和呈现，不重复实现领域发布判断；
- 增加三领域取消、unavailable、空输入、验证失败和回调生命周期对称测试。

## 验收与维护

```powershell
ctest --test-dir build -C Release --output-on-failure --parallel 4 `
  -R "^(hole_.*|pocket_.*|profile_.*|app_workflow_controllers|task_lifecycle|workflow_internal_boundaries|module_boundaries)$"
```

新增服务或异步入口必须同步 source/public manifest、直接依赖、取消测试和 [App Workflows](../../app/workflows/README.md) 的所有权表。

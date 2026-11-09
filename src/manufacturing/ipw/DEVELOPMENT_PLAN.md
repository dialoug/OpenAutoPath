# 统一 IPW 与材料状态开发规划

> 文档状态：执行中
>
> 规划范围：全局工序链、Multi-Dexel 材料真值、不可变检查点、仿真与审计
>
> 当前主阶段：I6——类型与换轴收敛
>
> 当前任务键：`I6/common-operation-carrier`
>
> 代码审计基线：`612ab76`
>
> 基线日期：2026-08-24
>
> 最近验证：2026-11-08，RelWithDebInfo 完整构建成功，CTest 60/60 通过（含正式精加工、桌面项目 IPW、取消和并行预算回归）
>
> 维护角色：`oap_manufacturing` IPW/Simulation 维护者

本文维护 `oap_manufacturing::ipw` 的总体路线。Job 身份与持久化见 [../job/DEVELOPMENT_PLAN.md](../job/DEVELOPMENT_PLAN.md)，Multi-Dexel 内核位于 [../../toolpath/pocket/simulation/MultiDexelStock.h](../../toolpath/pocket/simulation/MultiDexelStock.h)。

本文是 Multi-Dexel 材料真值、节点/检查点身份、backend/engine ID、缓存复用和跨表示误差门禁的唯一权威规划。特征文档只描述消费要求，不单独定义引擎版本。

## 1. 目标与材料真值

一个 Job 只能有一条按全局 `sequence` 排列的材料状态链。每个节点从唯一父检查点开始，事务式应用一道工序，成功后发布内容寻址的不可变检查点；失败或取消时父节点保持 Ready。

```text
初始毛坯 B-Rep
  -> 根 Multi-Dexel 检查点
  -> Hole / Pocket / Profile / SurfaceFinishing 节点
  -> 每节点 stateKey + contentFingerprint
  -> 可复用连续前缀
  -> 实时显示网格、逐移动回放、余量/过切比较
  -> 按需 B-Rep 适配或精确审计
```

Multi-Dexel 区间是跨工序材料真值。B-Rep 和三角网格是初始化、换轴适配、旧内核兼容、显示或按需审计产物，不得建立可独立向后推进的第二条真值链。

## 2. 当前实现基线

2026-11-08：原生材料更新新增 `FixedAxisToolpathView`，借用工序 ID、刀轴和权威
moves；Hole 与 Profile 不再为原生 IPW 构造 Pocket 工序。借用仅持续到同步 apply
返回，检查点和结果不保留视图。Profile 的旧载体转换限制在 workflow 的显式兼容
桥中，由 BRep 回退和显示消费者调用。刀具快照绑定、节点顺序、backend/engine ID、
材料指纹和取消发布策略保持不变；此次结构迁移不引入引擎语义版本变更。

| 能力 | 当前实现 | 状态 |
| --- | --- | --- |
| 会话计划 | `IpwSession` 节点状态机、父键/stateKey、连续前缀复用和向后失效 | 已完成 |
| 检查点 | `IpwCheckpointStore` 不可变发布、同键幂等、错误内容拒绝和 retain | 已完成 |
| 材料内核 | 分块多区间 Dexel、内容指纹、tile 事务、直接显示网格和定量比较 | 已完成 |
| 孔 | 原生旋转刀具轮廓更新，同轴/异轴均不经长期 B-Rep 真值 | 已完成 |
| 槽腔 | 语义图元固定轴更新、逐移动前缀和必要的换轴 B-Rep 过渡 | 已完成 |
| Profile | 原生 Dexel 分支直接使用中立执行视图；BRep/显示仍走显式兼容桥 | 原生路径已解耦 |
| SurfaceFinishing | OCL 刀位以中立视图接入真实平/球/圆鼻刀轮廓；同轴和异轴均保留父 Dexel 网格 | 已接入 |
| 执行器 | Job 映射、根检查点、逐节点执行、取消、进度和统计 | 已完成 |
| 桌面 | 正式项目四类工序按全局顺序统一执行，检查点时间线与精加工逐移动重放 | 已接入 |

当前 `IpwOperationKind` 覆盖 Hole/Pocket/Profile/SurfaceFinishing/Other。
SurfaceFinishing 通过 `FixedAxisToolpathView` 与显式 `DexelToolProfileDefinition`
接入原生材料内核，不构造 Pocket/Profile 工序。旧瞬时显示请求仍使用明确的仅显示
适配载体，并传递真实圆角刀具轮廓；跨 Setup 的部分槽腔/Profile 仍会显式物化 B-Rep。

## 3. 不可破坏的契约

1. 节点按全局 `sequence` 唯一有序，`parentStateKey` 只指向根或直接前序节点。
2. 状态只允许 Dirty → Computing → Ready/Failed/Cancelled；不能跳过 Computing 发布。
3. 节点仅在全部前序 Ready 时可运行；失败节点之后不得继续伪造材料。
4. `stateKey` 覆盖父键、工序输入指纹和引擎版本；内容指纹覆盖全部 Dexel 区间。
5. 检查点发布前在副本上完成全部移动、取消检查、定量门禁和内容指纹计算。
6. 同一 `stateKey` 发布不同内容必须失败；缓存只复用完全相同连续前缀。
7. Multi-Dexel 是材料真值；派生 B-Rep/显示网格不参与其内容指纹。
8. Rapid/Retract 不去料，但保留时间线位置；Cut/Plunge 等去料语义必须显式。
9. 刀具轮廓、刀轴、弦差和参数来自该工序的冻结快照，不使用当前 UI 默认值。
10. 显示、比较或按需审计失败不能反向破坏已发布材料检查点。

## 4. 阶段路线图

| 阶段 | 优先级 | 状态 | 主要交付 | 验收门禁 |
| --- | --- | --- | --- | --- |
| I0 会话与检查点 | P0 | 完成 | 节点状态机、stateKey、不可变 Store 和连续前缀 | 错误父键/内容拒绝，失效范围正确 |
| I1 Multi-Dexel 真值 | P0 | 完成 | 多区间材料、tile 更新、显示网格和内容摘要 | 体积/网格与材料区间闭合 |
| I2 Hole 原生更新 | P0 | 完成 | 旋转刀具轮廓、原生孔节点和逐工序明细 | 不经 B-Rep，取消无部分状态 |
| I3 Pocket/Profile 混合链 | P0 | 完成 | 固定轴语义扫掠、换轴过渡和全局顺序 | 混合工序检查点与回放一致 |
| I4 定量比较与桌面时间线 | P0 | 完成 | 余量/过切、实时网格、前缀回放和选择 | 显示/比较不改变材料真值 |
| I6 类型与换轴收敛 | P1 | **下一阶段** | Profile 一等后端、通用工序载体、减少 B-Rep 方向过渡 | 新旧结果等价且 backend ID 升级 |
| I7 持久检查点与扩展性能 | P2 | 增强 | 可选磁盘缓存、配额、跨进程校验和大 Job 性能 | 缓存损坏可拒绝且不影响重建 |

## 5. OpenCAMLib 正式精加工接入后的引擎契约

- 引擎 ID 为 `oap-multi-dexel-ipw-v8-surface-finishing`。
- 选项指纹版本为 `oap-ipw-engine-options-v12-surface-finishing`。
- 精加工 backend 为 `tiled-multi-dexel-finishing-v1-rounded-oriented`。
- 精加工执行前与 Ready 前缀复用前均校验当前生成结果，参数/刀具/装夹/目标变更不能借缓存绕过失效门禁。
- 正式项目仿真按所有启用工序的全局 sequence 执行，精加工消耗真实前序粗加工检查点。
- 保留通用旋转刀具轮廓、换轴扫掠以及材料表面显示，它们属于共享仿真能力。
- 旧版已移除的曲面工序项目仍在存储边界被拒绝；新精加工使用独立的 schema 定义。
- `surface_finishing_ipw` 回归覆盖真实 OCL 球刀/圆鼻刀、粗加工前缀、异轴执行、分包发布、参数失效及取消。

## 6. I6/I7 后续重点

- 以通用中立执行流替代 Pocket 作为 Profile/其他工序的历史载体，同时保留领域诊断；
- 同轴路径走原生 tile 更新；不同轴优先 oriented sweep，只有无法证明等价时保留显式 B-Rep fallback；
- 可选磁盘检查点必须版本化格式、校验和、原子写入、配额和 LRU，不写入 `.oapjob`；
- 跨进程缓存命中后仍校验 stateKey、backend ID、内容摘要和父链；
- 基准记录每节点 tile 复制、区间数、体积、执行时间、峰值内存、缓存命中和取消延迟。

## 7. 构建与验收

```powershell
cmake --build build --config Release --parallel 16 -- /verbosity:minimal

ctest --test-dir build -C Release --output-on-failure --parallel 4 `
  -R "^(multi_dexel_ipw|dexel_.*|hole_ipw_reference|stock_simulation_workspace|ipw_display_modes|machining_job|profile_.*|pocket_.*|module_boundaries)$"
```

引擎 ID、检查点结构或扫掠内核变化必须运行 Release 全量测试，并记录旧/新基准差异。不得用大型本地未跟踪 STEP 作为 CI 唯一证据。

## 8. 维护规则与当前下一步

下一阶段定义不依赖 Pocket 领域类型的中立工序执行载体，再迁移 Profile，并验证新旧内容指纹等价。任何材料语义变化都必须升级 backend/engine ID。

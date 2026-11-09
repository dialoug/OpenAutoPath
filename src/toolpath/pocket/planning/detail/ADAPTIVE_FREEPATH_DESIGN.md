# Adaptive FreePath 重构设计与维护契约

## 1. 目标

`ClearingPattern::FreePath` 现在表示残料与包角反馈驱动的二维 Adaptive
Clearing，而不是“圆形阿基米德螺旋裁剪成若干开放段”。实现以同级目录外的
`D:/claude/pyocc-exp/cam_exp` Python 验证原型为行为来源，并将其接入当前项目
已有的 Clipper、OCCT 分层、安全验证、进退刀、仿真和 NC 流程。

本文与以下文件共同构成维护边界：

- `FreePathPattern.cpp`：策略选择、wavefront 主路径、包角反馈、曲率候选和
  Beam Search；
- `AdaptiveFreePathRaster.*`：RemainingMaterial、刀具扫掠、包角代理和残料
  连通分量；
- `AdaptiveFreePathLinking.*`：逐行移植 Python 的不等半径 CSC
  （圆弧—直线—圆弧）连接候选与半径降级搜索；
- `PocketMoveAssembler.cpp`：把规划器给出的首选 CSC Link 提升到世界坐标，
  连续几何复验后提交；失败时执行公共退刀/重进恢复；
- `PocketClearing.cpp`：逐深度层准备真实刀心区域，聚合 Adaptive 诊断。

不要再次把 FreePath 实现成只依赖包围盒中心的固定螺旋，也不要把显示抽稀、
安全域或 NC 特例写进 FreePath 文件。

## 2. Python 行为到 C++ 模块的映射

| pyocc-exp | 当前 C++ | 说明 |
|---|---|---|
| `RasterRemainingMaterial` | `AdaptiveFreePathRaster` | `bytearray` 对应紧凑字节数组；上限均为 260000 个 cell |
| `safe-center predicate` | Clipper `PathsD` + `segmentSafe` | Clipper 连续区域是权威安全域，栅格只用于反馈 |
| rectangle wavefront spiral | `rectanglePrimaryPass` | 无岛轴对齐矩形生成单条连续圆角矩形螺旋 |
| constrained core spiral | `radialPrimaryPass` | 星形连续可达区生成边界约束中心螺旋，再处理径向残料组件 |
| distance-field wavefront | `genericWavefrontPrimaryPass` | 使用 Clipper 精确负偏置代替栅格 marching squares，保持由内向外及拓扑分支语义 |
| swept-area update | `removeSegment/removePolyline` | 刀具胶囊扫掠；cell 半对角线作为保守覆盖量 |
| disk/angular engagement | `engagementRatio/angularEngagement` | 默认 72 个圆周 bin，仍是代理值而非精确 entry/exit |
| curvature candidates | `expandCandidates` | 保持曲率、参考转向、理想螺旋、左右偏转和直线候选 |
| predicted-material Beam Search | `chooseCandidate` | 默认深度 3、宽度 4、未来层折扣 0.72 |
| residual component loop | `residualCleanup` | 直到残料阈值、pass 上限或没有可取得进展的组件 |
| CSC Cutting/Link 分离 | `AdaptiveFreePathLinking` + `preferredLinkFromPrevious` | 首选不等半径 CSC；只把已清材料段标为 Link，并由装配器安全复验 |

Clipper 精确偏置替换 marching squares 是有意的生产化变化：它保留 Python 的
wavefront 层级和拓扑策略，同时避免在已经拥有工业多边形布尔库的 C++ 项目中
重新引入栅格等值线误差。

## 3. 规划流水线

```text
当前深度真实截面
  -> 刀具半径/余量/保护材料/上一刀余料补偿
  -> authoritative cutter-center PathsD
  -> reconstruct material envelope for feedback only
  -> AdaptiveFreePathRaster
  -> strategy selection
       rectangle -> coherent rounded-rectangle spiral
       star-shaped -> boundary-constrained core + radial components
       general/islands -> topology-aware inward-offset wavefronts
  -> full outer and island boundary finishing passes
  -> RemainingMaterial connected components
  -> component seed scoring
  -> free-endpoint curvature candidates
  -> per-candidate swept-material and angular-engagement prediction
  -> bounded Beam Search
  -> commit first motion and update RemainingMaterial
  -> repeat to tolerance/pass limit
  -> preferred cleared-stock unequal-radius CSC Links
  -> PocketMoveAssembler continuous safety verification
  -> direct Cut Link or retract/rapid/re-entry recovery
```

主路径不是全部局部自适应：与 Python 原型一致，矩形/wavefront/中心螺旋先提供
全局稳定骨架；包角反馈重点用于径向分支和主路径后的残料 pass。

## 4. 几何与状态不变量

1. `reachable` 是唯一权威刀心安全区域。栅格 cell、安全采样和残料预测不得扩大
   该区域。
2. `materialArea` 只用于 RemainingMaterial 反馈。它由刀心域按刀具半径外扩重建，
   不能反过来用于批准刀心位置。
3. 每次提交 Cutting run 后必须立即从栅格扣除刀具扫掠；候选搜索只能使用当前
   状态或候选私有的 `ignored` cell 集合。
4. `preferredLinkFromPrevious` 必须从上一 run 的真实结束点开始，在当前 run 的
   起点结束，并且在生成时不覆盖未切材料。
5. 首选 Link 不是安全授权。`PocketMoveAssembler` 必须用当前深度的连续安全域
   再次检查每段；失败后走公共退刀恢复。
6. 岛边界与外边界都必须有独立 finishing pass。不能恢复旧实现“只追加正面积
   外轮廓”的行为。
7. FreePath 严格保持 Python 的中心向外执行顺序，不消费公共
   `expansionDirection`。旧项目中保存的由外向内值必须在参数归一化和持久化
   边界迁移为 `InsideToOutside`，不得通过反转 runs 破坏残料状态或首选 CSC
   Link。`cutMode` 只改变公共铣削旋向，不改变中心向外的 run 顺序。
8. 取消后不得发布部分层刀路。局部搜索每 256 个 segment 检查一次控制令牌。

## 5. 参数契约

参数位于 `AdaptiveClearingParams`，默认值与 Python API 对齐：

| 参数 | 默认 | 含义 |
|---|---:|---|
| `targetEngagement` | 0.10 | 刀具圆盘材料占比目标 |
| `targetEngagementAngleDeg` | 70° | 局部最大连续包角目标 |
| `maximumEngagementAngleDeg` | 150° | 有合规候选时过滤过载候选 |
| `engagementAngleSamples` | 72 | 刀具圆周采样数 |
| `minimumTurnRadiusFactor` | 0.60 | 最小转弯半径/刀具直径 |
| `remainingTolerance` | 0.015 | 停止补刀的残料比例 |
| `materialCellSize` | 0 | 自动取 `min(step/2, diameter/10)` |
| `maximumAdaptivePasses` | 60 | 局部 pass 上限 |
| `lookaheadDepth` | 3 | Beam Search 深度；1 为单步模式 |
| `beamWidth` | 4 | 每层保留分支数 |

这些参数需要同时维护在：规范化、项目设置、加工任务 JSON、参数对话框和回归
测试中。新增字段必须提供旧项目文件缺失字段时的默认值。

## 6. 诊断

`PocketClearingResult` 聚合每个深度层的：

- `adaptivePassCount`；
- `adaptiveRemainingComponentCount`；
- `adaptiveOverloadSegmentCount`；
- `adaptiveWavefrontLayerCount`；
- `adaptiveCentralSpiralCount`；
- `adaptiveRadialComponentCount`；
- `adaptiveRadialPassCount`；
- `adaptiveRemainingMaterialFraction`（各层最大值）；
- `adaptiveMaximumEngagementAngleDeg`（各层最大值）。

残料比例来自反馈栅格，是规划质量指标，不是最终工件连续几何验收。最终材料
结果仍应使用 Multi-Dexel/IPW 或解析 BRep 仿真复核。

## 7. 性能边界

- 栅格自动放大 cell，最多 260000 单元；
- 单个局部 pass 最多 20000 个运动决策；
- Adaptive pass 默认最多 60；
- lookahead 最大 4，beam width 最大 16；
- 每层拥有独立 tracker，继续由 `PocketClearing` 的层并行机制调度；
- 不允许用播放/显示抽稀掩盖规划器点数失控。显示层可以有通用预算，但不能成为
  FreePath 正确运行的前提。

## 8. 测试要求

至少覆盖：

1. 矩形只产生一条连续主路径和边界收边，残料低于阈值；
2. 凹区产生多个 wavefront 层且所有段位于刀心安全域；
3. 单岛/双岛路径不会跨岛，且包含岛 finishing pass；
4. 降低最大包角后 `adaptiveMaximumEngagementAngleDeg` 受控；
5. `lookaheadDepth=1` 和默认 Beam Search 都能收敛；
6. 首选 Link 不安全时装配器退刀恢复；
7. 多深度层、保护材料、上一刀余料和协作取消；
8. 星形连续区域选择中心螺旋与径向分支，而边界径向距离不连续的十字区域
   回退到拓扑 wavefront；
9. 公开清腔结果中能观察到径向分支间的不等半径 CSC，且私有 linking 头不泄漏到
   模块边界之外；
10. 参数 JSON/QSettings 往返及旧数据默认值。
11. FreePath 的旧由外向内参数被规范为中心向外，普通 Pocket 与 Profile 公共
    刀心入口均保持 Python run 顺序；规划器已经生成的首选 CSC Link 不得因旧
    方向参数被清空。

## 9. 已删除的旧实现负担

重构后不再需要：

- 固定圆形螺旋的曲率弦高“自适应采样”代码；
- `clipOpenSpiral` 和四象限 `applyNxCornerOpenRunOrder`；
- 为旧螺旋数百万 G1 点专门引入的 FreePath 显示补丁；
- 根 `docs/16_nx_freepath_csc_porting_guide.md` 中基于旧 `ffv_exp` CSC 分支的
  迁移方案；
- `.gitignore` 中为旧文档和 `ffv_exp` 临时加入的规则。

通用口袋显示预算若被其他模式需要，应以独立性能需求和测试保留，不能再以
FreePath 为理由耦合。

## 10. 后续演进

下一步优先级：

1. 用连续边界与刀具圆精确求交替换栅格包角代理；
2. 将 G1 恒曲率候选升级为 biarc/clothoid 近似 G2；
3. 把相邻 wavefront 的贪心顺序升级为显式拓扑图调度；
4. 用连续几何覆盖分析校准栅格 `remainingTolerance`；
5. 在现有 JSONL 加工摘要之外按需接入 UI 诊断，但不要让诊断模型污染 NC
   运动模型。

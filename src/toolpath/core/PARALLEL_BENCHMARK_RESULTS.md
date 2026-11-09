# 并行调度简化验收记录

日期：2026-11-08（Asia/Hong_Kong）。当前调度契约见 [Core 开发规划](DEVELOPMENT_PLAN.md)。本文测试数量和性能数值仅对应该次测量。

最终默认策略已恢复并通过 RelWithDebInfo 完整应用构建及 CTest 46/46 回归（63.42 秒）。

## 输入与范围

用户授权自行生成 STEP 测试件；不使用客户模型。测试程序按固定尺寸生成带底壁的实际
加工件，并导出每个区域的切削体。每次基准重新读取区域 STEP，再向真实
`PocketAutomaticToolpath::generate` 提交固定区域元数据。这测量调度与刀路规划，不包含
特征识别性能。STEP 读入、生成文件和摘要计算不计入 `milliseconds`。

| 样本 | 几何 | 区域/层数 | 候选 |
| --- | --- | --- | --- |
| small | 50 × 40 mm 矩形腔，深 4 mm | 1 / 2 | 3 把平底刀 |
| multi | 24 个独立矩形腔，深 8 mm | 24 / 每腔 4 | 3 把平底刀 |
| deep | 矩形深腔，深 40 mm | 1 / 20 | 3 把平底刀 |
| islands | 100 × 80 mm 腔，12 个 8 × 8 mm 岛，深 8 mm | 1 / 4 | 3 把平底刀 |
| many_tools | L 形腔，深 4 mm | 1 / 2 | 10 把平底刀、9 种刀径 |
| stress | 192 个独立矩形腔，深 20 mm | 192 / 每腔 10 | 3 把平底刀 |

切深 2 mm，Offset 清料，50% 步距，固定进给和安全高度。每次进程内缓存从空开始。
生成文件位于本机忽略目录 `artifacts/parallel-benchmark/step/`；主加工件是上述六个
同名 `.step`，其余 `*-region-N.step` 是区域切削体。固定源代码参数即生成配方。

## 方法

- RelWithDebInfo 优化构建；本机 8 个逻辑处理器、约 15.76 GiB 物理内存。
- 每个样本/配置独立进程运行，先 1 次预热/文件准备，再测 3 次，取中位数并保留原始记录。
- `OAP_CAM_MAX_THREADS` 分别为 1、4、8；它限制额外计算槽，不等于进程实际线程总数。
- PowerShell 外部观察进程：每约 10 ms 采样线程数与 OS 工作集峰值。线程峰值是采样值，
  可能漏掉短命线程；内存是整个进程（含 STEP 读入和结果摘要）的峰值，非精确规划分配量。
- SHA-256 覆盖量化到 1e-6 的 moves、线性/圆弧原语及 fallback、刀具快照、Setup、工序与来源顺序。
  另记录缓存命中/未命中、候选评估/剪枝、规划并发峰值、OCCT 并行及失败回退次数。
- 计时有系统调度波动；小于 100 ms 的微小差值不作为删除优化的证据。比较同一配置的
  输出摘要必须一致；不能用旧版绕过预算的“1 线程”成绩作为严格串行性能保证。
- 删除策略的门槛：输出一致、测得峰值内存无明显恶化，且有稳定的耗时/线程收益；
  不能依据这批合成样本推断任意客户模型或内存压力下的普遍性能。

## 对照构建

| 标签 | 差异 |
| --- | --- |
| baseline / baseline-stress | P1 修复后、P2/P3 前的保存二进制；仍含无预算验证入口 |
| dynamic | P2/P3 完整预算与 OCCT 准入；保留动态内存反馈、预取和受预算限制的嵌套计算 |
| fixed | 同 dynamic，但禁用动态反馈和每区域内存采样线程，保留固定初始内存准入 |
| no-prefetch | 同 dynamic，但全阶段预取窗口从 4 降为 1 |
| regions-only | 同 dynamic，但区域内不借额外槽，验证预算和 OCCT 门禁仍然生效 |

对照构建脚本临时改变私有实现常量，逐个保存二进制，最后恢复原始源码并重建。
这些实验不是新增的产品设置。原始 JSON、每次 stdout/stderr 日志保留在
`artifacts/parallel-benchmark/`，实验二进制与政策 JSON 保留在 `build/parallel-experiments/`。

## 复现命令

```powershell
cmake --build build --config RelWithDebInfo --target oap_test_pocket_automatic_toolpath --parallel 6
./tools/Build-PocketParallelExperiments.ps1
./tools/Measure-PocketParallelism.ps1 -Tag dynamic -Executable "$PWD/build/parallel-experiments/dynamic.exe"
./tools/Measure-PocketParallelism.ps1 -Tag fixed -Executable "$PWD/build/parallel-experiments/fixed.exe" -ThreadLimits 4,8
./tools/Measure-PocketParallelism.ps1 -Tag no-prefetch -Executable "$PWD/build/parallel-experiments/no-prefetch.exe" -ThreadLimits 8
./tools/Measure-PocketParallelism.ps1 -Tag regions-only -Executable "$PWD/build/parallel-experiments/regions-only.exe" -ThreadLimits 8
```

使用与 CTest 相同的 Qt/OCCT DLL 路径。脚本自动从 CTest JSON 发现该路径，单次进程超时
3 分钟即终止并报错，失败日志不丢弃。历史 baseline 二进制不由当前源代码重新生成。

## 结果与决策

完成 264 次进程运行，其中 198 次测量、66 次预热/准备。六个样本在全部配置、重复运行
及对照构建中，各自的输出摘要完全一致。六个主 STEP 再读入并通过串行
`BRepCheck_Analyzer` 检查：实体/面数分别为 small 1/11、multi 24/264、deep 1/11、
islands 1/71、many_tools 1/13、stress 192/2112。文件总计 226 个、约 10.4 MiB。

下表为配置上限 8 时的规划耗时中位数（毫秒）。这是整组顺序测量，不是隔离机器上的
性能认证；当时的可用物理内存会影响初始准入，原始记录保留了实际规划峰值。

| 样本 | P1 基线 | dynamic | fixed | no-prefetch | regions-only |
| --- | ---: | ---: | ---: | ---: | ---: |
| small | 20.4 | 22.3 | 41.6 | 40.1 | 20.1 |
| multi | 227.8 | 517.1 | 849.0 | 638.9 | 593.0 |
| deep | 81.8 | 187.0 | 307.8 | 177.9 | 231.5 |
| islands | 191.6 | 360.5 | 450.4 | 412.6 | 493.3 |
| many_tools | 107.0 | 279.2 | 233.6 | 213.3 | 265.5 |
| stress | 9622.1 | 11723.0 | 14438.7 | 11572.0 | 11650.8 |

初始表出现明显的时段差异，因此另做了三组相邻的 baseline/dynamic 复测，仍各预热 1 次、
测量 3 次。基线同一二进制也明显变慢，不能把初始表的全部差异归因于代码变化：

| 样本 | 相邻复测基线 ms | 相邻复测 dynamic ms | 基线/新实现进程线程采样峰值 | 基线/新实现进程工作集峰值 MiB |
| --- | ---: | ---: | --- | --- |
| multi | 731.0 | 678.5 | 26 / 20 | 37.39 / 37.06 |
| islands | 619.3 | 534.1 | 12 / 12 | 44.03 / 43.86 |
| many_tools | 234.3 | 225.3 | 12 / 12 | 26.52 / 26.56 |

以配置上限 1 运行时，multi/deep/islands/stress 的新实现采样线程峰值均为 5，原基线
对应为 12/10/12/12；计算规划峰值均为 1。剩余线程包含 CRT/库线程及一个内存采样线程，
不能宣称进程只有一个线程。配置上限 8 的 stress 峰值工作集，dynamic 189.35 MiB，
fixed 188.31 MiB，no-prefetch 188.63 MiB，regions-only 188.06 MiB；没有出现内存增长数量级变化。

最终选择 **dynamic**：

1. 保留动态内存反馈与采样。fixed 减少线程，但本轮固定准入通常只允许 2–3 个槽；
   dynamic 在多腔计算中能逐步使用 8 个槽。测量未提供删除动态准入的充分收益证据。
2. 保留 4 候选预取。禁用预取对 many_tools 有利，但 multi/islands 退化；stress 只有小幅波动。
   不用单一用例的收益替换通用默认策略。
3. 保留受预算限制的嵌套计算。regions-only 降低单腔线程峰值，但 deep/islands 明显变慢；
   多腔压力样本耗时接近 dynamic，不能据此禁用单腔可用的层/候选并行。
4. 已删除重复的专用额度控制/租约与第二层回调锁，合并公共并行入口；没有保留用户可见
   的实验开关。现有小型私有编译常量仅供对照脚本临时替换，默认全部恢复。

本次结论是预算、入口和职责收敛完成，结果保持一致；**不宣称普遍加速或严格全进程线程上限**。
内存压力、其他硬件和真实客户模型的延迟需要各自测量，不能从这些合成样本外推。

原始记录：`artifacts/parallel-benchmark/{baseline,baseline-stress,dynamic,fixed,no-prefetch,regions-only}.json`
及 `*-check-*.json`；汇总为同目录 `summary.json`。使用
`tools/Summarize-PocketParallelBenchmarks.ps1` 自动拒绝摘要不一致或测量次数不足的记录。

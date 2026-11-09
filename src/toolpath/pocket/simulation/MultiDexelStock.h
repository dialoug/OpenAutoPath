// 固定网格方向的多区间 Dexel 在制品状态。
//
// 每个 UV 网格列保存任意数量、互不重叠的材料 W 区间，因此能够保留通孔、
// 盲孔、悬空材料和多层实体。刀具操作轴可以不同于网格 W 轴；状态按 tile
// 共享，一次工序只复制被刀具扫掠触及的 tile，可直接作为不可变 IPW 检查点。
//
// 属于 oap_toolpath_pocket 的毛坯仿真层（CMake target: oap_toolpath_pocket）。
// 实现按构建、移动更新、查询和显示拆为独立编译单元；detail 头仅供模块内部使用。
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/simulation/FixedAxisToolpathView.h"
#include "toolpath/pocket/simulation/DexelDisplayMesh.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"
#include "toolpath/core/simulation/DexelToolProfile.h"

namespace oap::toolpath {

class MultiDexelStock;
class StockTargetCache;
struct PocketToolpathOperation;

// multi-dexel 构建统计：单元/区间数量与耗时。
struct MultiDexelBuildMetrics {
    double milliseconds = 0.0;
    int cellCount = 0;
    int activeCellCount = 0;
    int intervalCount = 0;
    int tileCount = 0;
};

// multi-dexel 应用结果：应用某工序去料后的状态。
struct MultiDexelApplyResult {
    std::shared_ptr<const MultiDexelStock> stock;
    StockRemovalRecord record;
    int cuttingMoveCount = 0;
    int touchedTileCount = 0;
    int changedCellCount = 0;
    int planarLayerCount = 0;
    int nonPlanarMoveCount = 0;
    int toolProfileBandCount = 0;
    double maximumToolProfileAxialError = 0.0;
    double maximumToolProfileRadialError = 0.0;
    double updateMilliseconds = 0.0;
};

using MultiDexelDisplayMeshMetrics =
    DexelDisplayMeshMetrics;

// 线程模型：实例经 std::shared_ptr<const> 持有且完全不可变，
// 可安全跨线程共享；每道工序 apply* 返回全新的共享状态（COW），
// 因而能作为不可变 IPW 检查点长期保存。apply* 内部并行栅格化时
// 不修改任何既有 tile。
class MultiDexelStock {
public:
    // 仅为实现文件中的栅格化/更新辅助函数公开不透明类型名称；
    // 调用方无法构造或访问其内容。
    class Impl;

    static std::shared_ptr<const MultiDexelStock> fromShape(
        const TopoDS_Shape& shape,
        const gp_Vec& outwardAxis,
        double referenceCutterDiameter,
        const StockSimulationOptions& options,
        MultiDexelBuildMetrics* metrics = nullptr,
        std::string* error = nullptr,
        std::function<bool()> cancellationRequested = {});

    // 将派生出的 BRep 过渡体重新光栅化到现有毛坯的精确网格上。同轴混合 IPW
    // 链必须保持单一不可变网格,使每个工序检查点都能与根状态直接可比。
    static std::shared_ptr<const MultiDexelStock> fromShapeOnGrid(
        const TopoDS_Shape& shape,
        const MultiDexelStock& referenceGrid,
        const StockSimulationOptions& options,
        MultiDexelBuildMetrics* metrics = nullptr,
        std::string* error = nullptr,
        std::function<bool()> cancellationRequested = {});

    // 将另一形状光栅化到此毛坯的精确 UV 网格上。供原生多 interval 比较使用，
    // 使目标审计无需实例化检查点 BRep。局部审计区域允许显式裁剪到网格范围；
    // 目标/检查点等完整材料形状仍要求投影完全位于网格内。
    bool rasterizeShapeOnGrid(
        const TopoDS_Shape& shape,
        DexelDisplayGrid& output,
        MultiDexelBuildMetrics* metrics = nullptr,
        std::string* error = nullptr,
        std::function<bool()> cancellationRequested = {},
        bool runParallel = true,
        double geometricTolerance = 1.0e-5,
        bool allowProjectionClipping = false,
        const StockTargetCache* targetCache = nullptr) const;

    bool valid() const noexcept;
    const std::string& error() const noexcept;
    const gp_Pnt& origin() const noexcept;
    const gp_Vec& uAxis() const noexcept;
    const gp_Vec& vAxis() const noexcept;
    const gp_Vec& wAxis() const noexcept;
    double uMin() const noexcept;
    double uMax() const noexcept;
    double vMin() const noexcept;
    double vMax() const noexcept;
    double resolution() const noexcept;
    double geometricTolerance() const noexcept;
    int nx() const noexcept;
    int ny() const noexcept;
    int tileSize() const noexcept;
    int tileCount() const noexcept;
    int activeCellCount() const noexcept;
    int intervalCount() const noexcept;
    double volume() const noexcept;

    bool axisAligned(
        const gp_Vec& outwardAxis,
        double angularTolerance = 1.0e-6) const noexcept;
    std::vector<DexelInterval> cellIntervals(
        int i,
        int j) const;
    std::span<const DexelInterval> cellIntervalsView(
        int i,
        int j) const noexcept;
    int sharedTileCountWith(
        const MultiDexelStock& other) const noexcept;

    // 摘要覆盖网格定义及每一个材料区间，不依赖派生 BRep 的拓扑编号。
    // 首次请求时线程安全地计算；临时动画帧无需请求，正式检查点仍使用原摘要算法。
    const std::string& contentFingerprint() const;

    // Neutral fixed-axis entry points consume borrowed moves synchronously.
    MultiDexelApplyResult applyFixedAxis(
        const FixedAxisToolpathView& operation,
        const ToolpathParams& params,
        int sequenceIndex,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    MultiDexelApplyResult applyAxialToolpath(
        const FixedAxisToolpathView& operation,
        const ToolpathParams& params,
        const DexelToolProfileDefinition& toolProfile,
        int sequenceIndex,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    MultiDexelApplyResult applyOrientedToolpath(
        const FixedAxisToolpathView& operation,
        const ToolpathParams& params,
        const DexelToolProfileDefinition& toolProfile,
        int sequenceIndex,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    MultiDexelApplyResult applyToolpathPrefix(
        const FixedAxisToolpathView& operation,
        const ToolpathParams& params,
        const DexelToolProfileDefinition& toolProfile,
        int sequenceIndex,
        std::size_t completedMoveCount,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    // Compatibility adapters for existing Pocket callers.
    MultiDexelApplyResult applyPocket(
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        int sequenceIndex,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    // 孔等其它固定轴程序复用同一 tile 事务内核，并显式提供旋转刀具轮廓。
    // operation 只承载实际移动和刀轴；Rapid/Retract 仍由公共内核跳过。
    MultiDexelApplyResult applyAxialToolpath(
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        const DexelToolProfileDefinition& toolProfile,
        int sequenceIndex,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    // 在不改变父网格方向的前提下，将固定刀轴旋转轮廓的连续线性扫掠
    // 投影为每条 Dexel 射线上的区间。用于不同 Setup/刀轴的孔加工。
    MultiDexelApplyResult applyOrientedToolpath(
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        const DexelToolProfileDefinition& toolProfile,
        int sequenceIndex,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    // 从父材料状态重放当前工序的移动前缀，生成不进入检查点仓库的临时帧。
    // completedMoveCount 按 clearing.moves 的执行顺序计数，Rapid/Retract 会
    // 保留时间线位置但不去除材料。完整前缀与 applyPocket() 使用同一内核，
    // 结果内容应与工序检查点一致。
    MultiDexelApplyResult applyPocketPrefix(
        const FixedAxisToolpathView& operation,
        const ToolpathParams& params,
        int sequenceIndex,
        std::size_t completedMoveCount,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    MultiDexelApplyResult applyPocketPrefix(
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        int sequenceIndex,
        std::size_t completedMoveCount,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    // 使用显式球刀/圆鼻刀轮廓重放通用固定轴执行流前缀；刀轴与父网格
    // 不一致时自动走 oriented sweep。用于 IPW 逐移动临时帧。
    MultiDexelApplyResult applyToolpathPrefix(
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        const DexelToolProfileDefinition& toolProfile,
        int sequenceIndex,
        std::size_t completedMoveCount,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;

    // 仅为显示、旧 BRep 内核适配或按需审计重建棱柱 Compound；
    // 返回形状不参与本对象的内容指纹和后续 Dexel 减料。
    bool buildSnapshot(
        TopoDS_Shape& output,
        int* solidCount = nullptr,
        std::string* error = nullptr,
        std::function<bool()> cancellationRequested = {}) const;

    // 直接从材料区间提取外表面三角网格，不创建 TopoDS 拓扑。水平面会按
    // 相同 W 高度合并矩形；相邻列之间只输出区间差所形成的暴露侧壁。
    bool buildDisplayMesh(
        Handle(Poly_Triangulation)& output,
        MultiDexelDisplayMeshMetrics* metrics = nullptr,
        std::string* error = nullptr,
        std::function<bool()> cancellationRequested = {},
        const DexelDisplayMeshOptions& options = {}) const;

private:
    explicit MultiDexelStock(std::shared_ptr<const Impl> impl);

    MultiDexelApplyResult applyMoveLimit(
        const FixedAxisToolpathView& operation,
        const ToolpathParams& params,
        int sequenceIndex,
        std::size_t completedMoveCount,
        bool allowEmptyCuttingPrefix,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested,
        const DexelToolProfileDefinition* toolProfile,
        bool orientedSweep) const;

    std::shared_ptr<const Impl> m_impl;
};

} // namespace oap::toolpath

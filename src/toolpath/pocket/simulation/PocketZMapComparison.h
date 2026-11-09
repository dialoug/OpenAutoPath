// 固定刀轴腔加工的 Z-map 原生余量检查。
//
// 目标实体投影到与减料仿真相同的网格后，逐列计算初始缺料、剩余余量
// 和过切。定量阶段只保存目标栅格；用户查看时才从差异区间直接生成
// 三角网格，不创建差异 BRep Compound。
//
// 属于 oap_toolpath_pocket 的毛坯仿真层。定量比较与差异网格的实现位于
// 模块私有的 detail/PocketZMap*.cpp 与 detail/MultiDexelComparison.cpp，
// 本头文件只暴露结构化数据与两个比较器接口。
#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Poly_Triangulation.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/pocket/simulation/DexelDisplayMesh.h"
#include "toolpath/core/simulation/StockComparison.h"

namespace oap::toolpath {

class MultiDexelStock;

// Z-map 单元差：单格的高度差记录。
struct PocketZMapCellDelta {
    int cellIndex = -1;
    double beforeTop = 0.0;
    double afterTop = 0.0;
};

// Z-map 工序差：单工序的高度差汇总。
struct PocketZMapOperationDelta {
    std::string operationId;
    int sequenceIndex = -1;
    std::vector<PocketZMapCellDelta> cells;
};

// 一次 Z-map 仿真的完整快照：网格定义 + 每列初始/最终顶面 + 逐工序
// 差分 + 特征余量域。由 pocket 仿真流程一次性构建，之后所有比较器
// 只读共享这份数据（valid() 校验数组长度一致）。
struct PocketZMapSimulationData {
    std::shared_ptr<const MultiDexelStock> gridSource;
    gp_Pnt origin;
    gp_Vec uAxis;
    gp_Vec vAxis;
    gp_Vec wAxis;
    double uMin = 0.0;
    double uMax = 0.0;
    double vMin = 0.0;
    double vMax = 0.0;
    double resolution = 0.0;
    int nx = 0;
    int ny = 0;
    double initialVolume = 0.0;
    double finalVolume = 0.0;
    std::vector<double> bottoms;
    std::vector<double> initialTops;
    std::vector<double> finalTops;
    std::vector<PocketZMapOperationDelta> operationDeltas;
    std::vector<StockFeatureAllowanceScope> featureAllowanceScopes;

    bool valid() const noexcept;
    std::size_t cellCount() const noexcept;
};

// multi-dexel 工序状态：某工序去料后的区间场状态。
struct MultiDexelOperationState {
    std::string operationId;
    int sequenceIndex = -1;
    std::shared_ptr<const MultiDexelStock> before;
    std::shared_ptr<const MultiDexelStock> after;
};

// multi-dexel 比较数据：各工序状态与高度差比较。
struct MultiDexelComparisonData {
    std::shared_ptr<const MultiDexelStock> initial;
    std::shared_ptr<const MultiDexelStock> final;
    std::vector<MultiDexelOperationState> operations;
    std::vector<StockFeatureAllowanceScope> featureAllowanceScopes;

    bool valid() const noexcept;
};

// 差异视图枚举：决定 PocketZMapDisplaySource 输出哪种三角网格
// （初始缺料 / 剩余余量 / 总过切 / 单工序过切 / 特征余量域）。
enum class PocketZMapDifferenceView {
    InitialMissing,
    RemainingAllowance,
    Overcut,
    OperationOvercut,
    FeatureAllowance,
};

// 按需生成差异显示网格的不可变源；持有网格化目标与 Z-map 数据，
// 由 compare/attributeOvercut 在量化阶段之后创建并共享。
class PocketZMapDisplaySource {
public:
    class Impl;

    // 按 view 输出对应差异面的三角网格。operationId 仅对 OperationOvercut
    // 视图有意义；options 只影响显示产物，不修改差异真值。
    bool buildDisplayMesh(
        PocketZMapDifferenceView view,
        Handle(Poly_Triangulation)& output,
        DexelDisplayMeshMetrics* metrics = nullptr,
        std::string operationId = {},
        std::string* error = nullptr,
        std::function<bool()> cancellationRequested = {},
        const DexelDisplayMeshOptions& options = {}) const;

    const std::string& contentFingerprint() const noexcept;

private:
    explicit PocketZMapDisplaySource(
        std::shared_ptr<const Impl> impl);

    std::shared_ptr<const Impl> m_impl;

    friend class PocketZMapComparison;
    friend class MultiDexelComparison;
};

// 基于 Z-map 快照的定量余量比较器：输出缺料/余量/过切汇总，
// 并把过切按工序归属归因（attributeOvercut）。
class PocketZMapComparison {
public:
    StockComparisonResult compare(
        std::shared_ptr<const PocketZMapSimulationData> data,
        const TopoDS_Shape& initialStock,
        const TopoDS_Shape& targetShape,
        const StockComparisonOptions& options = {},
        const ToolpathGenerationControl* control = nullptr) const;

    StockOvercutAttributionResult attributeOvercut(
        std::shared_ptr<const PocketZMapSimulationData> data,
        const TopoDS_Shape& targetShape,
        const StockComparisonOptions& options = {},
        const ToolpathGenerationControl* control = nullptr,
        std::shared_ptr<const PocketZMapDisplaySource>
            displaySource = {}) const;
};

// 基于多区间 Dexel 检查点链（MultiDexelComparisonData）的比较器，
// 供已有 Multi-Dexel 仿真链路的审计使用，无需实例化中间 BRep。
class MultiDexelComparison {
public:
    StockComparisonResult compare(
        std::shared_ptr<const MultiDexelComparisonData> data,
        const TopoDS_Shape& initialStock,
        const TopoDS_Shape& targetShape,
        const StockComparisonOptions& options = {},
        const ToolpathGenerationControl* control = nullptr) const;

    StockOvercutAttributionResult attributeOvercut(
        std::shared_ptr<const MultiDexelComparisonData> data,
        const TopoDS_Shape& targetShape,
        const StockComparisonOptions& options = {},
        const ToolpathGenerationControl* control = nullptr,
        std::shared_ptr<const PocketZMapDisplaySource>
            displaySource = {}) const;
};

} // namespace oap::toolpath

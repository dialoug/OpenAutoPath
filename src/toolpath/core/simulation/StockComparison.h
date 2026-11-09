// 初始毛坯、仿真后毛坯和目标成品之间的精确体积差异检查。
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "analysis/HoleGeometry.h"
#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/simulation/InProcessStock.h"

namespace oap::toolpath {

class PocketZMapDisplaySource;
class StockTargetCache;

// 毛坯比较选项：几何容差、并行开关与预校验标记。
struct StockComparisonOptions {
    double geometricTolerance = 1.0e-5;
    bool runParallel = true;
    // 仿真器已经逐步校验初始和最终毛坯时可跳过重复的全拓扑分析；
    // 目标实体仍始终独立校验。
    bool initialAndFinalShapesPrevalidated = false;
    std::shared_ptr<StockTargetCache> targetCache;
};

// 单个识别结构中的目标去除材料及当前剩余量。特征结果彼此独立，重叠结构
// 可能覆盖同一材料，因此不能把 remainingAllowanceVolume 直接相加成总体余量。
struct StockFeatureAllowance {
    std::string featureId;
    std::string operationId;
    std::string featureKind;
    int sourceIndex = -1;
    double targetRemovalVolume = 0.0;
    double remainingAllowanceVolume = 0.0;
    double completionRatio = 0.0;
    bool approximateBoundary = false;
    // BRep 审计可直接保留局部余量；Multi-Dexel 路径由显示源按 featureId 派生。
    TopoDS_Shape remainingAllowance;
};

// 特征余量范围：特征/工序标识、区域与近似边界标记。
struct StockFeatureAllowanceScope {
    std::string featureId;
    std::string operationId;
    std::string featureKind;
    int sourceIndex = -1;
    TopoDS_Shape region;
    bool approximateBoundary = false;
};

// 构造孔特征余量的局部审计域。原始重载供运行期 Operation 使用，允许把
// 钻尖实际到达深度纳入范围；几何重载会为开放端和锥底增加保守轴向覆盖。
TopoDS_Shape buildHoleFeatureAllowanceRegion(
    const gp_Pnt& entry,
    const gp_Pnt& bottom,
    double radius,
    double entryExtension = 0.0,
    double bottomExtension = 0.0);

TopoDS_Shape buildHoleFeatureAllowanceRegion(
    const RecognizedHoleGeometry& hole,
    double boundaryMargin = 0.0);

// 毛坯比较结果：初始缺料、剩余余量、过切与特征余量诊断。
struct StockComparisonResult {
    // 目标在初始毛坯外的部分：表示毛坯缺料、尺寸不足或坐标未对齐。
    TopoDS_Shape initialMissing;
    // 最终毛坯仍位于目标成品外的部分：表示后续仍需加工的余量。
    TopoDS_Shape remainingAllowance;
    // 初始时存在、但仿真后从目标成品内消失的部分：表示真正的过切。
    TopoDS_Shape overcut;
    std::vector<ToolpathDiagnostic> diagnostics;
    std::vector<StockFeatureAllowance> featureAllowances;

    double initialVolume = 0.0;
    double finalVolume = 0.0;
    double targetVolume = 0.0;
    double initialMissingVolume = 0.0;
    double remainingAllowanceVolume = 0.0;
    double overcutVolume = 0.0;
    double targetContainmentRatio = 0.0;
    double inputValidationMilliseconds = 0.0;
    double initialMissingMilliseconds = 0.0;
    double remainingAllowanceMilliseconds = 0.0;
    double initialTargetIntersectionMilliseconds = 0.0;
    double overcutMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
    double heightFieldTargetMilliseconds = 0.0;
    double heightFieldDifferenceMilliseconds = 0.0;
    double heightFieldResolution = 0.0;
    // 高度场只按单元中心采样。目标边界相邻单元中的过切是离散上界，
    // 不能与完全位于目标内部单元中的确定过切混为一谈。
    double heightFieldCertainOvercutVolume = 0.0;
    double heightFieldBoundaryUncertaintyVolume = 0.0;
    int heightFieldCellCount = 0;
    int heightFieldDifferenceSolidCount = 0;

    std::array<double, 3> initialBoundsCenter{0.0, 0.0, 0.0};
    std::array<double, 3> targetBoundsCenter{0.0, 0.0, 0.0};
    std::array<double, 3> boundsCenterOffset{0.0, 0.0, 0.0};
    bool hasBounds = false;
    bool targetBoundsInsideInitialBounds = false;
    bool initialContainsTarget = false;
    bool heightFieldComparison = false;
    // 高度场路径只保存目标栅格和差异规则。差异 BRep 不在定量阶段生成；
    // 桌面端需要显示时由该来源按需构造直接三角网格。
    std::shared_ptr<const PocketZMapDisplaySource>
        heightFieldDisplaySource;
    bool cancelled = false;
    bool complete = false;
};

// 用精确 BRep 差集结果按特征区域派生局部余量。该函数不改变总体比较结果；
// 单个特征失败时跳过该项，避免局部审计破坏已经完成的总体审计。
void analyzeFeatureAllowances(
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape,
    const std::vector<StockFeatureAllowanceScope>& scopes,
    StockComparisonResult& result);

// 工序过切归因：工序、体积与边界不确定性体积。
struct StockOperationOvercut {
    std::string operationId;
    int sequenceIndex = -1;
    // 对高度场归因，volume 是单元中心采样得到的过切上界。
    // certainVolume 排除目标边界相邻单元，剩余部分记录为边界不确定量。
    double volume = 0.0;
    double certainVolume = 0.0;
    double boundaryUncertaintyVolume = 0.0;
    TopoDS_Shape shape;
};

// 过切归因结果：各工序过切明细与高度场显示源。
struct StockOvercutAttributionResult {
    std::vector<StockOperationOvercut> operations;
    std::vector<ToolpathDiagnostic> diagnostics;
    double attributedVolume = 0.0;
    double totalMilliseconds = 0.0;
    std::shared_ptr<const PocketZMapDisplaySource>
        heightFieldDisplaySource;
    bool cancelled = false;
    bool complete = false;
};

// 毛坯比较服务：比较初始/最终毛坯与目标形状，产出余量与过切。
class StockComparison {
public:
    static constexpr std::size_t ProgressStepCount = 4;

    StockComparisonResult compare(
        const TopoDS_Shape& initialStock,
        const TopoDS_Shape& finalStock,
        const TopoDS_Shape& targetShape,
        const StockComparisonOptions& options = {},
        const ToolpathGenerationControl* control = nullptr) const;

    // 根据顺序毛坯快照递推仍存在的目标材料。每道工序直接从上一步目标
    // 材料中减去当前毛坯，避免独立 Common 产生非单调体积波动。
    StockOvercutAttributionResult attributeOvercut(
        const TopoDS_Shape& initialStock,
        const TopoDS_Shape& targetShape,
        const std::vector<StockRemovalRecord>& steps,
        const StockComparisonOptions& options = {},
        const ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::toolpath

// 平底刀恒深语义原语的解析分层 BRep 快路径。资格或构造失败由调用方
// 整工序回退到既有 Clipper 分层，禁止混合发布半解析结果。
//
// 位于 oap_toolpath_pocket 的 fixedaxis/ 模块私有实现目录（内部细节）：
// 只在 AnalyticPlanarSweep / AnalyticPlanarBoolean 把每个原语输出为
// 保留 Line/Circle 曲线的解析面时生效，作为 FixedAxisLayeredBRepBuilder
// 的整工序替代路径。
#pragma once

#include <functional>
#include <string>

#include <TopoDS_Shape.hxx>

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::toolpath {

// 解析固定轴分层 BRep 构建结果：去除体积与统计。
struct AnalyticFixedAxisLayeredBRepBuildResult {
    TopoDS_Shape removalVolume;
    std::string reason;
    int cuttingMoveCount = 0;
    int primitiveCount = 0;
    int arcCount = 0;
    int linearRampCount = 0;
    int linearRampGroupCount = 0;
    int helicalRampMoveCount = 0;
    int helicalRampGroupCount = 0;
    int containedRampCount = 0;
    int containedRampGroupCount = 0;
    int rampSweepSliceCount = 0;
    int planarLayerCount = 0;
    int removalSolidCount = 0;
    int footprintOuterCount = 0;
    int footprintHoleCount = 0;
    int booleanGroupCount = 0;
    int closedOffsetFillRecordCount = 0;
    int toolProfileBandCount = 0;
    double footprintArea = 0.0;
    double maximumFootprintArea = 0.0;
    double maximumRampAxialError = 0.0;
    double maximumBoundaryFitError = 0.0;
    double maximumToolProfileAxialError = 0.0;
    double maximumToolProfileRadialError = 0.0;
    double pathCollectionMilliseconds = 0.0;
    double rampProcessingMilliseconds = 0.0;
    double planarBooleanMilliseconds = 0.0;
    double solidBuildMilliseconds = 0.0;
    double buildMilliseconds = 0.0;
    bool attempted = false;
    bool eligible = false;
    bool succeeded = false;
    bool cancelled = false;
    bool noMaterial = false;
    bool closedOffsetFillUsed = false;
};

// 解析固定轴分层 BRep 构建器：用解析扫掠直接构造去除体（快路径）。
class AnalyticFixedAxisLayeredBRepBuilder {
public:
    AnalyticFixedAxisLayeredBRepBuildResult build(
        const TopoDS_Shape& stock,
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;
};

} // namespace oap::toolpath

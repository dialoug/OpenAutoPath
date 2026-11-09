// 为固定轴铣削构建低拓扑结构的去除 BRep。
//
// 接受平底、牛鼻和球头刀具在恒定深度切削、轴向下刀、直线斜坡及采样螺旋
// 斜坡上的运动。刀具中心路径按二维 Clipper 笔画求并,分解为不重叠的
// 轴向片体,再提升为闭合 OCCT 棱柱。
//
// 位于 oap_toolpath_pocket 的 fixedaxis/ 模块私有实现目录（内部细节）。
// 构建顺序：先尝试 AnalyticFixedAxisLayeredBRepBuilder 解析快路径，
// 仅在资格不满足或构造失败时整工序回退到本文件的 Clipper 分层路径。
#pragma once

#include <functional>
#include <string>

#include <TopoDS_Shape.hxx>

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::toolpath {

// 固定轴分层 BRep 构建结果：去除体积与分层统计。
struct FixedAxisLayeredBRepBuildResult {
    TopoDS_Shape removalVolume;
    std::string error;
    int cuttingMoveCount = 0;
    int planarLayerCount = 0;
    int removalSolidCount = 0;
    int skippedInvalidRemovalSolidCount = 0;
    int footprintOuterCount = 0;
    int footprintHoleCount = 0;
    int linearRampMoveCount = 0;
    int helicalRampMoveCount = 0;
    int rampSweepGroupCount = 0;
    int containedRampSweepGroupCount = 0;
    int containedRampMoveCount = 0;
    int skippedRampContainmentGroupCount = 0;
    int rampSweepSliceCount = 0;
    int toolProfileBandCount = 0;
    double footprintArea = 0.0;
    double maximumFootprintArea = 0.0;
    double maximumCurveFitError = 0.0;
    double maximumChordError = 0.0;
    double maximumRampAxialError = 0.0;
    double maximumToolProfileAxialError = 0.0;
    double maximumToolProfileRadialError = 0.0;
    double pathCollectionMilliseconds = 0.0;
    double sweepInflationMilliseconds = 0.0;
    double rampProcessingMilliseconds = 0.0;
    double slabUnionMilliseconds = 0.0;
    double solidBuildMilliseconds = 0.0;
    double buildMilliseconds = 0.0;
    int analyticPlanarPrimitiveCount = 0;
    int analyticPlanarArcCount = 0;
    int analyticPlanarBooleanGroupCount = 0;
    int analyticPlanarFallbackOperationCount = 0;
    double analyticPlanarMilliseconds = 0.0;
    bool analyticPlanarAttempted = false;
    bool analyticPlanarUsed = false;
    std::string analyticPlanarFallbackReason;
    bool cancelled = false;
    bool unsupported = false;
    bool noMaterial = false;

    bool ok() const noexcept
    {
        return !cancelled && !unsupported &&
               error.empty() && !removalVolume.IsNull();
    }
};

// 固定轴分层 BRep 构建器：Clipper 分层路径构建去除棱柱。
class FixedAxisLayeredBRepBuilder {
public:
    FixedAxisLayeredBRepBuildResult build(
        const TopoDS_Shape& stock,
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        const StockSimulationOptions& options,
        std::function<bool()> cancellationRequested = {}) const;
};

} // namespace oap::toolpath

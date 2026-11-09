// 固定轴 BRep 的轻量输入预检。
//
// 预检只解析整次审计共享的刀具几何和移动类别，不构造任何 OCCT 减料体。
// 工序级错误仍由构造器生成对应快照，确保 UI 能准确显示失败工序。
//
// 位于 oap_toolpath_pocket 的 fixedaxis/ 模块私有实现目录：该目录是
// 固定轴 BRep 仿真内部细节，与其头文件配套的 detail/*.cpp 与
// Analytic* 解析快路径均不向模块外发布。
#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>

#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::toolpath {

// 输出汇总：对整批工序统计移动类别，推导有效刀具参数（含按毛坯跨度
// 推断刃长的回退），供后续构造器复用同一套参数。
struct FixedAxisBRepPreflightResult {
    ToolpathParams effectiveParams;
    std::vector<ToolpathDiagnostic> diagnostics;
    int constantDepthCutMoveCount = 0;
    int axialPlungeMoveCount = 0;
    int linearRampMoveCount = 0;
    int helicalRampMoveCount = 0;
    int helicalRampOperationCount = 0;
    bool inferredCuttingEdgeLength = false;
    double effectiveCuttingEdgeLength = 0.0;
};

// 对 stock 与整批 operations 做轻量预检。只读取几何、不做 OCCT 减料；
// 统计恒深切削 / 轴向进刀 / 直线斜坡 / 螺旋斜坡数量并推导有效刃长。
// 输入：stock 毛坯、operations 工序序列、params 批次参数。
class FixedAxisBRepPreflight {
public:
    FixedAxisBRepPreflightResult inspect(
        const TopoDS_Shape& stock,
        const std::vector<PocketToolpathOperation>& operations,
        const ToolpathParams& params,
        const StockSimulationOptions& options = {}) const;
};

} // namespace oap::toolpath

// 刀心清料图案生成的公共门面（oap_toolpath_pocket）。
// 输入已做刀具半径补偿的二维刀心可达区（Clipper2 PathsD），按清料图案、
// 走刀模式与行距生成 ClearingLoop。这是 Pocket 图案生成器对外接口，
// Profile 等模块无需依赖 detail 头。
#pragma once

#include <vector>

#include "clipper2/clipper.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/planning/PocketClearing.h"

namespace oap::toolpath {

// 对已经完成刀具半径补偿的二维刀心可达区生成清料图案。
// 这是 Pocket 图案生成器的公共门面，Profile 等模块无需依赖 detail 头。
struct CutterCenterClearingPatternResult {
    std::vector<ClearingLoop> loops;
    int passCount = 0;
    bool spiralFallback = false;
};

CutterCenterClearingPatternResult generateCutterCenterClearingPattern(
    const Clipper2Lib::PathsD& centerArea,
    ClearingPattern pattern,
    CutMode cutMode,
    double stepOver,
    double toolDiameter,
    const AdaptiveClearingParams& adaptive,
    double zigzagAngle,
    bool perimeterPass,
    int precision,
    double arcTolerance,
    const ToolpathGenerationControl* control = nullptr);

} // namespace oap::toolpath

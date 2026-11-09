// 内部清料图案生成器。输入为归一化后的 Clipper 路径；输出保持在二维平面内，
// 深度和世界坐标由 PocketClearing 补充。
#pragma once

#include <algorithm>
#include <cstddef>

#include "PocketClipperUtils.h"
#include "toolpath/pocket/planning/PocketClearing.h"

namespace oap::toolpath::detail {

struct PocketPatternParallelAudit {
    std::size_t batchCount = 0u;
    std::size_t taskCount = 0u;
    std::size_t peakParallelism = 1u;

    void append(const PocketPatternParallelAudit& other)
    {
        batchCount += other.batchCount;
        taskCount += other.taskCount;
        peakParallelism = std::max(
            peakParallelism, other.peakParallelism);
    }
};

struct AdaptiveFreePathDiagnostics {
    int adaptivePasses = 0;
    int remainingComponents = 0;
    int overloadSegments = 0;
    int wavefrontLayers = 0;
    int centralSpiralRuns = 0;
    int radialComponents = 0;
    int radialPasses = 0;
    double remainingMaterialFraction = 0.0;
    double maximumEngagementAngleDeg = 0.0;
};

// 等距同心环图案：从首刀偏置 firstDelta 起以 step 逐层内缩（或按
// expansionDirection 反向输出），输出每圈的闭合环，含岛环（isHole）。
std::vector<ClearingLoop> generateOffsetLoops(
    const PathsD& region2d, double firstDelta, double step, int maxRings,
    bool reverse, PocketClearingDirection expansionDirection,
    int precision, double arcTolerance, int& ringCountOut,
    const ToolpathGenerationControl* control = nullptr,
    PocketPatternParallelAudit* parallelAudit = nullptr);

// 螺旋图案：把连续偏移环插值成一条少抬刀的连续折线；仅适用于
// 单连通区域（至多一个岛），不适用时返回空由调用方回退到 Offset。
std::vector<ClearingLoop> generateSpiralLoops(
    const PathsD& region2d, double firstDelta, double step, int maxRings,
    bool reverse, PocketClearingDirection expansionDirection,
    int precision, double arcTolerance, int& ringCountOut);

// pyocc-exp 的残料反馈型 Adaptive Clearing。输入可以是原始加工区域，也
// 可以是已经补偿的刀心区域；生成器内部维护栅格残料、wavefront 主路径、
// 包角反馈局部 pass 与首选 G1 连接，并严格保持 Python 的中心向外顺序。
// 公共流程仍负责深度、铣削旋向和最终安全恢复。
std::vector<ClearingLoop> generateFreePathLoops(
    const PathsD& region2d, double firstDelta, double step,
    double toolDiameter,
    const AdaptiveClearingParams& adaptive,
    bool reverse,
    int precision, double arcTolerance,
    int& ringCountOut,
    const ToolpathGenerationControl* control = nullptr,
    AdaptiveFreePathDiagnostics* diagnostics = nullptr);

// 之字（ZigZag）图案：按 angleDeg 旋转坐标后等距扫描线裁剪区域，
// 奇数行反向形成往复路径；perimeterPass 时先输出一圈贴边轮廓。
std::vector<ClearingLoop> generateZigZagLoops(
    const PathsD& region2d, double radius, double extraOffset, double step,
    double angleDeg, bool perimeterPass, bool reverse, int precision,
    double arcTolerance,
    const ToolpathGenerationControl* control = nullptr,
    PocketPatternParallelAudit* parallelAudit = nullptr);

} // namespace oap::toolpath::detail

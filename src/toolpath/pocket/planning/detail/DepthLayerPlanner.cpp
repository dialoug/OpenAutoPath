// 根据总深度、步下量和分层截面生成单调且覆盖底面的加工深度序列。
#include "toolpath/pocket/planning/detail/DepthLayerPlanner.h"

#include <algorithm>
#include <cmath>

#include "PocketGeometryUtils.h"

namespace oap::toolpath::detail {

namespace {
constexpr double kDepthTolerance = 1.0e-9;
}

std::vector<DepthLayer> buildDepthLayers(const PocketRegion& region,
                                         const DepthParams& depth,
                                         PocketClearingResult& result)
{
    std::vector<DepthLayer> layers;

    // 深度坐标约定：0 = 腔底，向开口为正。useFeatureDepth 时从识别深度
    // 顶部（region.depth）开始逐层切到 0（底面）；否则用显式配置的
    // startDepth/finalDepth 范围。
    const double recognizedDepth = std::max(0.0, region.depth);
    double start = depth.useFeatureDepth ? recognizedDepth : depth.startDepth;
    double final = depth.useFeatureDepth ? 0.0 : depth.finalDepth;

    if (depth.useFeatureDepth && recognizedDepth <= kDepthTolerance)
        appendWarning(result.warning,
                      "recognized pocket depth is zero; generating a single floor layer");

    if (!std::isfinite(start) || !std::isfinite(final)) {
        appendWarning(result.warning,
                      "invalid depth range; generating a single floor layer");
        start = 0.0;
        final = 0.0;
    }

    double stepDown = depth.stepDown;
    if (!std::isfinite(stepDown) || stepDown <= kDepthTolerance) {
        stepDown = std::abs(start - final);
        if (stepDown <= kDepthTolerance)
            stepDown = 1.0;
        appendWarning(result.warning,
                      "invalid step-down; using full depth in one layer");
    }

    result.startDepthOffset = start;
    result.finalDepthOffset = final;
    result.stepDown = stepDown;

    const double travel = std::abs(start - final);
    if (travel <= kDepthTolerance) {
        layers.push_back({0, final});
        return layers;
    }

    const double direction = (final < start) ? -1.0 : 1.0;

    // 精修留量：粗加工只切到 final 上方的 finishTrim 处，最后一刀单独
    // 贴底完成精修，避免粗切重复刮到底面。
    double finishTrim = std::max(0.0, depth.finishDepth);
    if (!std::isfinite(finishTrim))
        finishTrim = 0.0;
    if (finishTrim > kDepthTolerance && finishTrim + kDepthTolerance >= travel) {
        appendWarning(result.warning,
                      "finish depth >= total depth; ignoring finish allowance");
        finishTrim = 0.0;
    }
    result.finishDepth = finishTrim;

    const double roughFinal = final - direction * finishTrim;
    const double travelRough = std::abs(start - roughFinal);
    const int maxLayers =
        std::max(1, static_cast<int>(std::ceil(travelRough / stepDown)) + 2);
    double current = start;

    for (int layer = 0; layer < maxLayers; ++layer) {
        const double remaining = std::abs(roughFinal - current);
        if (remaining <= kDepthTolerance)
            break;

        const double delta = std::min(stepDown, remaining);
        double next = current + direction * delta;
        if (std::abs(roughFinal - next) <= kDepthTolerance)
            next = roughFinal;

        layers.push_back({static_cast<int>(layers.size()), next});
        current = next;
    }

    if (layers.empty())
        layers.push_back({static_cast<int>(layers.size()), roughFinal});
    if (std::abs(layers.back().offset - final) > kDepthTolerance)
        layers.push_back({static_cast<int>(layers.size()), final});

    return layers;
}

} // namespace oap::toolpath::detail

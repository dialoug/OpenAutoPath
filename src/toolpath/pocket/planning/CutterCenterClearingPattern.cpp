// generateCutterCenterClearingPattern 实现（oap_toolpath_pocket）。
// 参见 CutterCenterClearingPattern.h；具体图案构造委托 detail/PocketPatterns.h。
#include "toolpath/pocket/planning/CutterCenterClearingPattern.h"

#include <algorithm>
#include <cmath>

#include "toolpath/pocket/planning/detail/PocketPatterns.h"

namespace oap::toolpath {

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
    const ToolpathGenerationControl* control)
{
    CutterCenterClearingPatternResult result;
    if (centerArea.empty() || !std::isfinite(stepOver) || stepOver <= 0.0)
        return result;

    const bool reverse = cutMode == CutMode::Conventional;
    if (pattern == ClearingPattern::ZigZag) {
        result.loops = detail::generateZigZagLoops(
            centerArea, 0.0, 0.0, stepOver, zigzagAngle,
            perimeterPass, reverse, precision, arcTolerance, control);
        for (const ClearingLoop& loop : result.loops)
            result.passCount = std::max(result.passCount, loop.ring + 1);
        return result;
    }

    if (pattern == ClearingPattern::FreePath) {
        result.loops = detail::generateFreePathLoops(
            centerArea, 0.0, stepOver, toolDiameter, adaptive, reverse,
            precision, arcTolerance, result.passCount, control);
        return result;
    }

    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(centerArea);
    const double span = std::max(
        std::abs(bounds.right - bounds.left),
        std::abs(bounds.bottom - bounds.top));
    const int maximumRings = std::clamp(
        static_cast<int>(std::ceil(span / stepOver)) + 4,
        1, 10000);
    if (pattern == ClearingPattern::Spiral) {
        result.loops = detail::generateSpiralLoops(
            centerArea, 0.0, stepOver, maximumRings,
            reverse, PocketClearingDirection::OutsideToInside,
            precision, arcTolerance, result.passCount);
        if (!result.loops.empty())
            return result;
        result.spiralFallback = true;
    }

    result.loops = detail::generateOffsetLoops(
        centerArea, 0.0, stepOver, maximumRings,
        reverse, PocketClearingDirection::OutsideToInside,
        precision, arcTolerance, result.passCount, control);
    return result;
}

} // namespace oap::toolpath

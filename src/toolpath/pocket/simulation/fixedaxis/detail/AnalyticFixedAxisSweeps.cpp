// Conservative planar coverage and analytic ramp sweep collection.
#include "toolpath/pocket/simulation/fixedaxis/AnalyticFixedAxisLayeredBRepBuilder.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <gp_Ax3.hxx>
#include "toolpath/pocket/simulation/fixedaxis/AnalyticPlanarSweep.h"

#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisTypes.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisGeometry.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisSweeps.h"

namespace oap::toolpath::analytic_fixed_axis_detail {

bool linearRampCoveredByPlanarCuts(
    const AnalyticLinearRampGroup& group,
    const std::vector<AnalyticPlanarCut>& cuts,
    const FixedAxisFrame& frame,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double tolerance)
{
    if (cuts.empty())
        return false;
    for (const ToolpathPrimitive& primitive : group.segments) {
        const ToolpathMove& segment = primitive.linear;
        const double fromW = frame.axial(segment.from);
        const double toW = frame.axial(segment.to);
        const double requiredBottom = std::max(
            std::min(fromW, toW), stockMinimum);
        const double requiredTop = std::min(
            std::max(fromW, toW) + cuttingLength,
            stockMaximum);
        if (requiredTop - requiredBottom <= tolerance)
            continue;

        const auto from = frame.project(segment.from);
        const auto to = frame.project(segment.to);
        std::vector<std::pair<double, double>> intervals;
        for (const AnalyticPlanarCut& cut : cuts) {
            if (cut.top <= requiredBottom + tolerance ||
                cut.bottom >= requiredTop - tolerance ||

                !projectedSegmentContains(
                    cut, from, to, tolerance)) {
                continue;
            }
            intervals.emplace_back(
                std::max(cut.bottom, requiredBottom),
                std::min(cut.top, requiredTop));
        }
        if (!intervalCovered(
                std::move(intervals),
                requiredBottom, requiredTop, tolerance)) {
            return false;
        }
    }
    return true;
}

bool helicalRampCoveredByPlanarCuts(
    const AnalyticHelicalRampGroup& group,
    const std::vector<AnalyticPlanarCoverage>& cuts,
    const FixedAxisFrame& frame,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double tolerance)
{
    if (cuts.empty() || group.arcs.empty())
        return false;
    for (const ToolpathPrimitive& primitive : group.arcs) {
        const double fromW = frame.axial(primitive.arc.from);
        const double toW = frame.axial(primitive.arc.to);
        const double requiredBottom = std::max(
            std::min(fromW, toW), stockMinimum);
        const double requiredTop = std::min(
            std::max(fromW, toW) + cuttingLength,
            stockMaximum);
        if (requiredTop - requiredBottom <= tolerance)
            continue;

        ToolpathPrimitive projectedArc;
        if (!makePlanarHelicalSlice(
                primitive, frame, 0.0, 1.0,
                tolerance, projectedArc)) {
            return false;
        }
        const std::string geometryKey =
            projectedCoverageKey(projectedArc, frame);
        std::vector<std::pair<double, double>> intervals;
        for (const AnalyticPlanarCoverage& cut : cuts) {
            if (cut.geometryKey != geometryKey ||
                cut.top <= requiredBottom + tolerance ||
                cut.bottom >= requiredTop - tolerance) {
                continue;
            }
            intervals.emplace_back(
                std::max(cut.bottom, requiredBottom),
                std::min(cut.top, requiredTop));
        }
        if (!intervalCovered(
                std::move(intervals),
                requiredBottom, requiredTop, tolerance)) {
            return false;
        }
    }
    return true;
}

bool appendLinearRampRecords(
    const AnalyticLinearRampGroup& group,
    const FixedAxisFrame& frame,
    const gp_Ax3& referencePlane,
    const AnalyticPlanarSweep& planarSweep,
    double cutterRadius,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double axialStep,
    double tolerance,
    int maximumSlices,
    const std::function<bool()>& cancellationRequested,
    std::vector<AnalyticSweepRecord>& records,
    AnalyticFixedAxisLayeredBRepBuildResult& result)
{
    const double low = std::max(
        group.minimumTip, stockMinimum - cuttingLength);
    const double high = std::min(
        group.maximumTip, stockMaximum);
    if (high - low <= tolerance)
        return true;

    const double firstBinValue = std::floor(
        (low - stockMinimum) / axialStep);
    const double lastBinValue = std::ceil(
        (high - stockMinimum) / axialStep) - 1.0;
    if (!std::isfinite(firstBinValue) ||
        !std::isfinite(lastBinValue) ||
        firstBinValue < static_cast<double>(
            std::numeric_limits<long long>::lowest()) ||
        firstBinValue > static_cast<double>(
            std::numeric_limits<long long>::max()) ||
        lastBinValue < static_cast<double>(
            std::numeric_limits<long long>::lowest()) ||
        lastBinValue > static_cast<double>(
            std::numeric_limits<long long>::max())) {
        result.reason =
            "analytic linear ramp has an invalid axial range";
        return false;
    }

    const long long firstBin =
        static_cast<long long>(firstBinValue);
    const long long lastBin =
        static_cast<long long>(lastBinValue);
    for (long long bin = firstBin;
         bin <= lastBin; ++bin) {
        if (cancelled(cancellationRequested)) {
            result.cancelled = true;
            return false;
        }
        const double binBottom = stockMinimum +
            static_cast<double>(bin) * axialStep;
        const double binTop = binBottom + axialStep;
        const double layerLow = std::max(low, binBottom);
        const double layerHigh = std::min(high, binTop);
        if (layerHigh - layerLow <= tolerance)
            continue;

        // 在一个刀尖轴向带内，只有 [pieceHigh,
        // pieceLow + cuttingLength] 被整段中心线共同覆盖。
        // 因此该棱柱严格内接于真实平底刀斜坡扫掠；缺失楔形的
        // 最大轴向厚度不超过当前带高。
        const double bottom = std::max(
            layerHigh, stockMinimum);
        const double top = std::min(
            layerLow + cuttingLength, stockMaximum);
        if (top - bottom <= tolerance)
            continue;

        for (const ToolpathPrimitive& primitive :
             group.segments) {
            const ToolpathMove& ramp = primitive.linear;
            const double fromW = frame.axial(ramp.from);
            const double toW = frame.axial(ramp.to);
            const double sourceLow = std::min(fromW, toW);
            const double sourceHigh = std::max(fromW, toW);
            const double pieceLow = std::max(
                layerLow, sourceLow);
            const double pieceHigh = std::min(
                layerHigh, sourceHigh);
            if (pieceHigh - pieceLow <= tolerance)
                continue;
            if (result.rampSweepSliceCount >= maximumSlices) {
                result.reason =
                    "analytic linear ramp exceeds the axial slice budget";
                return false;
            }

            ToolpathPrimitive slice = primitive;
            slice.linear.from = interpolateAtAxial(
                ramp.from, ramp.to,
                fromW, toW, pieceLow);
            slice.linear.to = interpolateAtAxial(
                ramp.from, ramp.to,
                fromW, toW, pieceHigh);
            AnalyticPlanarSweepResult sweep =
                planarSweep.build(
                    referencePlane, slice,
                    cutterRadius, tolerance);
            if (!sweep.ok()) {
                result.reason = sweep.error.empty()
                    ? "analytic linear-ramp planar sweep construction failed"
                    : sweep.error;
                return false;
            }
            records.push_back(
                {bottom, top, std::move(sweep.face),
                 primitiveGeometryKey(slice, frame)});
            ++result.rampSweepSliceCount;
        }
        result.maximumRampAxialError = std::max(
            result.maximumRampAxialError,
            layerHigh - layerLow);

        if (bin == std::numeric_limits<long long>::max())
            break;
    }
    return true;
}

bool appendHelicalRampRecords(
    const AnalyticHelicalRampGroup& group,
    const FixedAxisFrame& frame,
    const gp_Ax3& referencePlane,
    const AnalyticPlanarSweep& planarSweep,
    double cutterRadius,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double axialStep,
    double tolerance,
    int maximumSlices,
    const std::function<bool()>& cancellationRequested,
    std::vector<AnalyticSweepRecord>& records,
    AnalyticFixedAxisLayeredBRepBuildResult& result)
{
    const double low = std::max(
        group.minimumTip, stockMinimum - cuttingLength);
    const double high = std::min(
        group.maximumTip, stockMaximum);
    if (high - low <= tolerance)
        return true;

    const double firstBinValue = std::floor(
        (low - stockMinimum) / axialStep);
    const double lastBinValue = std::ceil(
        (high - stockMinimum) / axialStep) - 1.0;
    if (!std::isfinite(firstBinValue) ||
        !std::isfinite(lastBinValue) ||
        firstBinValue < static_cast<double>(
            std::numeric_limits<long long>::lowest()) ||
        firstBinValue > static_cast<double>(
            std::numeric_limits<long long>::max()) ||
        lastBinValue < static_cast<double>(
            std::numeric_limits<long long>::lowest()) ||
        lastBinValue > static_cast<double>(
            std::numeric_limits<long long>::max())) {
        result.reason =
            "analytic helical ramp has an invalid axial range";
        return false;
    }

    const long long firstBin =
        static_cast<long long>(firstBinValue);
    const long long lastBin =
        static_cast<long long>(lastBinValue);
    for (long long bin = firstBin;
         bin <= lastBin; ++bin) {
        if (cancelled(cancellationRequested)) {
            result.cancelled = true;
            return false;
        }
        const double binBottom = stockMinimum +
            static_cast<double>(bin) * axialStep;
        const double binTop = binBottom + axialStep;
        const double layerLow = std::max(low, binBottom);
        const double layerHigh = std::min(high, binTop);
        if (layerHigh - layerLow <= tolerance)
            continue;

        // 与线性斜坡相同，只发布当前刀尖带内所有位置共同覆盖的轴向
        // 区间。平面轮廓则直接取语义螺旋的精确圆弧子段，不使用弦线。
        const double bottom = std::max(
            layerHigh, stockMinimum);
        const double top = std::min(
            layerLow + cuttingLength, stockMaximum);
        if (top - bottom <= tolerance)
            continue;

        for (const ToolpathPrimitive& primitive : group.arcs) {
            const ToolpathCircularArc& helix = primitive.arc;
            const double fromW = frame.axial(helix.from);
            const double toW = frame.axial(helix.to);
            const double sourceLow = std::min(fromW, toW);
            const double sourceHigh = std::max(fromW, toW);
            const double pieceLow = std::max(
                layerLow, sourceLow);
            const double pieceHigh = std::min(
                layerHigh, sourceHigh);
            if (pieceHigh - pieceLow <= tolerance)
                continue;
            if (result.rampSweepSliceCount >= maximumSlices) {
                result.reason =
                    "analytic helical ramp exceeds the axial slice budget";
                return false;
            }

            const double denominator = toW - fromW;
            if (std::abs(denominator) <= tolerance) {
                result.reason =
                    "analytic helical ramp has no axial travel";
                return false;
            }
            const double firstFraction = std::clamp(
                (pieceLow - fromW) / denominator,
                0.0, 1.0);
            const double secondFraction = std::clamp(
                (pieceHigh - fromW) / denominator,
                0.0, 1.0);
            const double startFraction = std::min(
                firstFraction, secondFraction);
            const double endFraction = std::max(
                firstFraction, secondFraction);
            ToolpathPrimitive slice;
            if (!makePlanarHelicalSlice(
                    primitive, frame,
                    startFraction, endFraction,
                    tolerance, slice)) {
                result.reason =
                    "analytic helical-ramp subdivision failed";
                return false;
            }
            AnalyticPlanarSweepResult sweep =
                planarSweep.build(
                    referencePlane, slice,
                    cutterRadius, tolerance);
            if (!sweep.ok()) {
                result.reason = sweep.error.empty()
                    ? "analytic helical-ramp planar sweep construction failed"
                    : sweep.error;
                return false;
            }
            records.push_back(
                {bottom, top, std::move(sweep.face),
                 primitiveGeometryKey(slice, frame)});
            ++result.rampSweepSliceCount;
        }
        result.maximumRampAxialError = std::max(
            result.maximumRampAxialError,
            layerHigh - layerLow);

        if (bin == std::numeric_limits<long long>::max())
            break;
    }
    return true;
}

} // namespace oap::toolpath::analytic_fixed_axis_detail

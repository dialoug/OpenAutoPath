// Z-map comparison and overcut attribution; display and rasterization are separate stages.
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Precision.hxx>
#include <TopoDS_Shape.hxx>

#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/pocket/simulation/detail/PocketZMapDisplayState.h"
#include "toolpath/pocket/simulation/detail/PocketZMapMultiDexelSupport.h"

namespace oap::toolpath {
using namespace zmap_detail;

bool PocketZMapSimulationData::valid() const noexcept
{
    const std::size_t count = cellCount();
    return nx > 0 && ny > 0 &&
           std::isfinite(resolution) &&
           resolution > 0.0 &&
           uAxis.Magnitude() > Precision::Confusion() &&
           vAxis.Magnitude() > Precision::Confusion() &&
           wAxis.Magnitude() > Precision::Confusion() &&
           bottoms.size() == count &&
           initialTops.size() == count &&
           finalTops.size() == count;
}

std::size_t PocketZMapSimulationData::cellCount() const noexcept
{
    if (nx <= 0 || ny <= 0)
        return 0u;
    return static_cast<std::size_t>(nx) *
           static_cast<std::size_t>(ny);
}

StockComparisonResult PocketZMapComparison::compare(
    std::shared_ptr<const PocketZMapSimulationData> dataSource,
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control) const
{
    StockComparisonResult result;
    ScopedElapsedMilliseconds totalTimer(
        result.totalMilliseconds);
    result.heightFieldComparison = true;
    if (!dataSource) {
        reportComparisonFailure(
            result, control,
            "Z-map comparison requires a retained grid");
        return result;
    }
    const PocketZMapSimulationData& data =
        *dataSource;
    result.heightFieldResolution = data.resolution;
    result.heightFieldCellCount =
        static_cast<int>(data.cellCount());
    result.initialVolume = data.initialVolume;
    result.finalVolume = data.finalVolume;
    const double tolerance = std::isfinite(
                                 options.geometricTolerance)
        ? std::max(
              options.geometricTolerance,
              Precision::Confusion())
        : std::max(1.0e-5, Precision::Confusion());
    auto progress = [&](std::size_t step) {
        if (control)
            control->progress(
                step, StockComparison::ProgressStepCount);
    };
    progress(0);
    if (!data.valid() || initialStock.IsNull() ||
        targetShape.IsNull()) {
        reportComparisonFailure(
            result, control,
            "Z-map comparison requires valid grid, initial stock, and target");
        return result;
    }
    if (cancelled(control)) {
        reportComparisonFailure(
            result, control,
            "Z-map comparison was cancelled before target rasterization");
        return result;
    }

    std::array<double, 6> initialBounds{};
    std::array<double, 6> targetBounds{};
    result.hasBounds =
        shapeBounds(initialStock, initialBounds);
    TargetField target;
    std::string error;
    const SteadyClock::time_point targetStart =
        SteadyClock::now();
    if (!rasterizeTarget(
            data, targetShape, options, control,
            target, targetBounds, error)) {
        result.heightFieldTargetMilliseconds =
            elapsedMilliseconds(targetStart);
        reportComparisonFailure(
            result, control, std::move(error));
        return result;
    }
    result.heightFieldTargetMilliseconds =
        elapsedMilliseconds(targetStart);
    result.targetVolume = target.volume;
    if (result.hasBounds) {
        for (int axis = 0; axis < 3; ++axis) {
            result.initialBoundsCenter[axis] =
                0.5 *
                (initialBounds[axis] +
                 initialBounds[axis + 3]);
            result.targetBoundsCenter[axis] =
                0.5 *
                (targetBounds[axis] +
                 targetBounds[axis + 3]);
            result.boundsCenterOffset[axis] =
                result.targetBoundsCenter[axis] -
                result.initialBoundsCenter[axis];
        }
        result.targetBoundsInsideInitialBounds =
            targetBounds[0] >= initialBounds[0] - tolerance &&
            targetBounds[1] >= initialBounds[1] - tolerance &&
            targetBounds[2] >= initialBounds[2] - tolerance &&
            targetBounds[3] <= initialBounds[3] + tolerance &&
            targetBounds[4] <= initialBounds[4] + tolerance &&
            targetBounds[5] <= initialBounds[5] + tolerance;
    }

    const std::size_t count = data.cellCount();
    std::vector<Interval> primary(count);
    std::vector<Interval> secondary(count);
    const SteadyClock::time_point differenceStart =
        SteadyClock::now();
    for (std::size_t cell = 0; cell < count; ++cell) {
        const double targetBottom = target.bottoms[cell];
        const double targetTop = target.tops[cell];
        if (!finiteInterval(
                targetBottom, targetTop, tolerance)) {
            continue;
        }
        const double initialBottom = data.bottoms[cell];
        const double initialTop = data.initialTops[cell];
        if (!finiteInterval(
                initialBottom, initialTop, tolerance)) {
            setInterval(
                primary[cell], targetBottom, targetTop,
                tolerance);
            continue;
        }
        setInterval(
            primary[cell],
            targetBottom,
            std::min(targetTop, initialBottom),
            tolerance);
        setInterval(
            secondary[cell],
            std::max(targetBottom, initialTop),
            targetTop,
            tolerance);
    }
    result.initialMissingVolume =
        intervalVolume(data, primary) +
        intervalVolume(data, secondary);
    const double volumeTolerance = std::max(
        tolerance * tolerance * tolerance,
        1.0e-9 * std::max({
            1.0, result.initialVolume, result.targetVolume}));
    progress(1);

    std::fill(
        primary.begin(), primary.end(), Interval{});
    std::fill(
        secondary.begin(), secondary.end(), Interval{});
    for (std::size_t cell = 0; cell < count; ++cell) {
        const double stockBottom = data.bottoms[cell];
        const double finalTop = data.finalTops[cell];
        if (!finiteInterval(
                stockBottom, finalTop, tolerance)) {
            continue;
        }
        const double targetBottom = target.bottoms[cell];
        const double targetTop = target.tops[cell];
        if (!finiteInterval(
                targetBottom, targetTop, tolerance)) {
            setInterval(
                primary[cell], stockBottom, finalTop,
                tolerance);
            continue;
        }
        setInterval(
            primary[cell],
            stockBottom,
            std::min(finalTop, targetBottom),
            tolerance);
        setInterval(
            secondary[cell],
            std::max(stockBottom, targetTop),
            finalTop,
            tolerance);
    }
    result.remainingAllowanceVolume =
        intervalVolume(data, primary) +
        intervalVolume(data, secondary);
    progress(2);

    result.targetContainmentRatio = std::clamp(
        1.0 -
            result.initialMissingVolume /
                result.targetVolume,
        0.0, 1.0);
    result.initialContainsTarget =
        result.initialMissingVolume <= volumeTolerance;
    progress(3);

    std::fill(
        primary.begin(), primary.end(), Interval{});
    for (std::size_t cell = 0; cell < count; ++cell) {
        const double initialBottom = data.bottoms[cell];
        const double initialTop = data.initialTops[cell];
        const double targetBottom = target.bottoms[cell];
        const double targetTop = target.tops[cell];
        if (!finiteInterval(
                initialBottom, initialTop, tolerance) ||
            !finiteInterval(
                targetBottom, targetTop, tolerance)) {
            continue;
        }
        const double presentBottom =
            std::max(initialBottom, targetBottom);
        const double presentTop =
            std::min(initialTop, targetTop);
        const double finalTop = data.finalTops[cell];
        setInterval(
            primary[cell],
            std::max(presentBottom, finalTop),
            presentTop,
            tolerance);
    }
    result.overcutVolume =
        intervalVolume(data, primary);
    std::fill(
        secondary.begin(), secondary.end(), Interval{});
    for (std::size_t cell = 0; cell < count; ++cell) {
        if (!finiteInterval(
                primary[cell].lower,
                primary[cell].upper,
                tolerance)) {
            continue;
        }
        if (!targetBoundaryCell(
                data, target, cell, tolerance)) {
            secondary[cell] = primary[cell];
        }
    }
    result.heightFieldCertainOvercutVolume =
        intervalVolume(data, secondary);
    result.heightFieldBoundaryUncertaintyVolume =
        std::max(
            0.0,
            result.overcutVolume -
                result.heightFieldCertainOvercutVolume);
    buildZMapFeatureAllowances(
        data, target, tolerance, control,
        result.featureAllowances);
    result.heightFieldDifferenceMilliseconds =
        elapsedMilliseconds(differenceStart);
    progress(4);

    if (!result.initialContainsTarget) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::StockTargetOutsideInitial,
            ToolpathDiagnosticSeverity::Error,
            "target material is missing from the initial Z-map stock");
    }
    if (result.remainingAllowanceVolume >
        volumeTolerance) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::StockRemainingAllowance,
            ToolpathDiagnosticSeverity::Info,
            "Z-map material remains outside the target after current operations");
    }
    if (result.heightFieldCertainOvercutVolume >
        volumeTolerance) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::StockOvercutDetected,
            ToolpathDiagnosticSeverity::Error,
            "current operations removed Z-map material in target-interior cells");
    } else if (
        result.heightFieldBoundaryUncertaintyVolume >
        volumeTolerance) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::StockOvercutDetected,
            ToolpathDiagnosticSeverity::Warning,
            "potential Z-map overcut is limited to target-boundary cells; refine the grid or run BRep audit");
    }
    auto displayImpl =
        std::make_shared<PocketZMapDisplaySource::Impl>();
    displayImpl->data = std::move(dataSource);
    displayImpl->target = std::move(target);
    displayImpl->tolerance = tolerance;
    displayImpl->fingerprint =
        displaySourceFingerprint(
            *displayImpl->data,
            displayImpl->target);
    result.heightFieldDisplaySource =
        std::shared_ptr<const PocketZMapDisplaySource>(
            new PocketZMapDisplaySource(
                std::move(displayImpl)));
    result.complete = true;
    return result;
}

StockOvercutAttributionResult
PocketZMapComparison::attributeOvercut(
    std::shared_ptr<const PocketZMapSimulationData> dataSource,
    const TopoDS_Shape& targetShape,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control,
    std::shared_ptr<const PocketZMapDisplaySource>
        displaySource) const
{
    StockOvercutAttributionResult result;
    ScopedElapsedMilliseconds totalTimer(
        result.totalMilliseconds);
    if (!dataSource) {
        addAttributionDiagnostic(
            result,
            ToolpathDiagnosticCode::StockComparisonFailed,
            ToolpathDiagnosticSeverity::Error,
            "Z-map overcut attribution requires a retained grid");
        return result;
    }
    const PocketZMapSimulationData& data =
        *dataSource;
    if (control) {
        control->progress(
            0, data.operationDeltas.size());
    }
    if (!data.valid() || targetShape.IsNull()) {
        addAttributionDiagnostic(
            result,
            ToolpathDiagnosticCode::StockComparisonFailed,
            ToolpathDiagnosticSeverity::Error,
            "Z-map overcut attribution requires valid grid and target");
        return result;
    }
    const double tolerance = std::isfinite(
                                 options.geometricTolerance)
        ? std::max(
              options.geometricTolerance,
              Precision::Confusion())
        : std::max(1.0e-5, Precision::Confusion());
    if (!displaySource ||
        !displaySource->m_impl ||
        displaySource->m_impl->data.get() !=
            dataSource.get()) {
        TargetField target;
        std::array<double, 6> targetBounds{};
        std::string error;
        if (!rasterizeTarget(
                data, targetShape, options, control,
                target, targetBounds, error)) {
            if (cancelled(control)) {
                result.cancelled = true;
                addAttributionDiagnostic(
                    result,
                    ToolpathDiagnosticCode::
                        GenerationCancelled,
                    ToolpathDiagnosticSeverity::Warning,
                    std::move(error));
            } else {
                addAttributionDiagnostic(
                    result,
                    ToolpathDiagnosticCode::
                        StockComparisonFailed,
                    ToolpathDiagnosticSeverity::Error,
                    std::move(error));
            }
            return result;
        }
        auto displayImpl =
            std::make_shared<
                PocketZMapDisplaySource::Impl>();
        displayImpl->data = dataSource;
        displayImpl->target = std::move(target);
        displayImpl->tolerance = tolerance;
        displayImpl->fingerprint =
            displaySourceFingerprint(
                *displayImpl->data,
                displayImpl->target);
        displaySource =
            std::shared_ptr<const
                PocketZMapDisplaySource>(
                new PocketZMapDisplaySource(
                    std::move(displayImpl)));
    }
    const TargetField& target =
        displaySource->m_impl->target;
    result.heightFieldDisplaySource =
        displaySource;
    const double volumeTolerance = std::max(
        tolerance * tolerance * tolerance,
        1.0e-9 * std::max({
            1.0, data.initialVolume, target.volume}));
    std::vector<Interval> intervals(data.cellCount());
    std::vector<Interval> certainIntervals(
        data.cellCount());
    for (std::size_t operationIndex = 0;
         operationIndex < data.operationDeltas.size();
         ++operationIndex) {
        if (cancelled(control)) {
            result.cancelled = true;
            addAttributionDiagnostic(
                result,
                ToolpathDiagnosticCode::GenerationCancelled,
                ToolpathDiagnosticSeverity::Warning,
                "Z-map overcut attribution was cancelled");
            return result;
        }
        std::fill(
            intervals.begin(), intervals.end(), Interval{});
        std::fill(
            certainIntervals.begin(),
            certainIntervals.end(),
            Interval{});
        const PocketZMapOperationDelta& operation =
            data.operationDeltas[operationIndex];
        for (const PocketZMapCellDelta& change :
             operation.cells) {
            if (change.cellIndex < 0 ||
                static_cast<std::size_t>(
                    change.cellIndex) >=
                    data.cellCount()) {
                continue;
            }
            const std::size_t cell =
                static_cast<std::size_t>(
                    change.cellIndex);
            const double initialBottom =
                data.bottoms[cell];
            const double initialTop =
                data.initialTops[cell];
            const double targetBottom =
                target.bottoms[cell];
            const double targetTop =
                target.tops[cell];
            if (!finiteInterval(
                    initialBottom, initialTop, tolerance) ||
                !finiteInterval(
                    targetBottom, targetTop, tolerance)) {
                continue;
            }
            setInterval(
                intervals[cell],
                std::max({
                    change.afterTop,
                    initialBottom,
                    targetBottom}),
                std::min({
                    change.beforeTop,
                    initialTop,
                    targetTop}),
                tolerance);
        }
        const double volume =
            intervalVolume(data, intervals);
        if (volume > volumeTolerance) {
            for (std::size_t cell = 0;
                 cell < intervals.size(); ++cell) {
                if (finiteInterval(
                        intervals[cell].lower,
                        intervals[cell].upper,
                        tolerance) &&
                    !targetBoundaryCell(
                        data, target, cell, tolerance)) {
                    certainIntervals[cell] =
                        intervals[cell];
                }
            }
            StockOperationOvercut attribution;
            attribution.operationId =
                operation.operationId;
            attribution.sequenceIndex =
                operation.sequenceIndex;
            attribution.volume = volume;
            attribution.certainVolume =
                intervalVolume(
                    data, certainIntervals);
            attribution.boundaryUncertaintyVolume =
                std::max(
                    0.0,
                    attribution.volume -
                        attribution.certainVolume);
            result.attributedVolume += volume;
            const double certainVolume =
                attribution.certainVolume;
            result.operations.push_back(
                std::move(attribution));
            addAttributionDiagnostic(
                result,
                ToolpathDiagnosticCode::StockOvercutDetected,
                certainVolume > volumeTolerance
                    ? ToolpathDiagnosticSeverity::Error
                    : ToolpathDiagnosticSeverity::Warning,
                operation.operationId +
                    (certainVolume > volumeTolerance
                         ? ": operation removed target-interior Z-map material"
                         : ": potential overcut is limited to target-boundary Z-map cells"),
                operation.operationId);
        }
        if (control) {
            control->progress(
                operationIndex + 1,
                data.operationDeltas.size());
        }
    }
    result.complete = true;
    return result;
}
} // namespace oap::toolpath

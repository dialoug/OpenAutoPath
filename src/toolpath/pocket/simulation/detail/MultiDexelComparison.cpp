// Independently compiled private MultiDexelComparison implementation.
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include <Precision.hxx>
#include <TopoDS_Shape.hxx>

#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/pocket/simulation/detail/PocketZMapDisplayState.h"
#include "toolpath/pocket/simulation/detail/PocketZMapMultiDexelSupport.h"

namespace oap::toolpath {
using namespace oap::toolpath::zmap_detail;

bool MultiDexelComparisonData::valid() const noexcept
{
    if (!initial || !final ||
        !initial->valid() || !final->valid()) {
        return false;
    }
    const double tolerance = std::max(
        1.0e-8,
        1.0e-9 * initial->resolution());
    if (!sameMultiDexelGrid(
            *initial, *final, tolerance)) {
        return false;
    }
    return std::all_of(
        operations.begin(), operations.end(),
        [&](const MultiDexelOperationState& operation) {
            return operation.before &&
                   operation.after &&
                   sameMultiDexelGrid(
                       *initial,
                       *operation.before,
                       tolerance) &&
                   sameMultiDexelGrid(
                       *initial,
                       *operation.after,
                       tolerance);
        });
}

StockComparisonResult MultiDexelComparison::compare(
    std::shared_ptr<const MultiDexelComparisonData>
        dataSource,
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control) const
{
    StockComparisonResult result;
    ScopedElapsedMilliseconds totalTimer(
        result.totalMilliseconds);
    result.heightFieldComparison = true;
    if (!dataSource || !dataSource->valid() ||
        initialStock.IsNull() ||
        targetShape.IsNull()) {
        reportComparisonFailure(
            result, control,
            "multi-dexel comparison requires valid checkpoints, initial stock, and target");
        return result;
    }
    const double tolerance = std::isfinite(
                                 options.geometricTolerance)
        ? std::max(
              options.geometricTolerance,
              Precision::Confusion())
        : std::max(1.0e-5, Precision::Confusion());
    result.initialVolume =
        dataSource->initial->volume();
    result.finalVolume =
        dataSource->final->volume();
    result.heightFieldResolution =
        dataSource->initial->resolution();
    result.heightFieldCellCount =
        dataSource->initial->nx() *
        dataSource->initial->ny();
    auto progress = [&](std::size_t step) {
        if (control) {
            control->progress(
                step,
                StockComparison::ProgressStepCount);
        }
    };
    progress(0);
    if (cancelled(control)) {
        reportComparisonFailure(
            result, control,
            "multi-dexel comparison was cancelled before target rasterization");
        return result;
    }

    std::array<double, 6> initialBounds{};
    std::array<double, 6> targetBounds{};
    const auto cache = options.targetCache && options.targetCache->matchesShape(targetShape)
        ? options.targetCache : nullptr;
    result.hasBounds =
        shapeBounds(initialStock, initialBounds) &&
        (cache ? cache->bounds(targetBounds) : shapeBounds(targetShape, targetBounds));
    DexelDisplayGrid target;
    MultiDexelBuildMetrics targetMetrics;
    std::string error;
    const SteadyClock::time_point targetStart =
        SteadyClock::now();
    const auto cacheKey = stockTargetGridKey(*dataSource->initial, tolerance);
    const auto cached = cache ? cache->multiDexel(cacheKey) : nullptr;
    if (cached) target = *cached;
    if (!cached && !dataSource->initial->rasterizeShapeOnGrid(
            targetShape,
            target,
            &targetMetrics,
            &error,
            [control] {
                return cancelled(control);
            },
            options.runParallel,
            tolerance, false, cache.get())) {
        result.heightFieldTargetMilliseconds =
            elapsedMilliseconds(targetStart);
        reportComparisonFailure(
            result, control, std::move(error));
        return result;
    }
    if (cancelled(control)) {
        reportComparisonFailure(result, control, "multi-dexel target rasterization was cancelled");
        return result;
    }
    if (cache && !cached) cache->store(cacheKey, target);
    result.heightFieldTargetMilliseconds =
        elapsedMilliseconds(targetStart);
    result.targetVolume =
        displayGridVolume(target);
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
            targetBounds[0] >=
                initialBounds[0] - tolerance &&
            targetBounds[1] >=
                initialBounds[1] - tolerance &&
            targetBounds[2] >=
                initialBounds[2] - tolerance &&
            targetBounds[3] <=
                initialBounds[3] + tolerance &&
            targetBounds[4] <=
                initialBounds[4] + tolerance &&
            targetBounds[5] <=
                initialBounds[5] + tolerance;
    }

    const SteadyClock::time_point differenceStart =
        SteadyClock::now();
    MultiDexelDifferenceSet differences;
    if (!buildMultiDexelDifferences(
            *dataSource, target, tolerance,
            control, differences, error)) {
        result.heightFieldDifferenceMilliseconds =
            elapsedMilliseconds(differenceStart);
        reportComparisonFailure(
            result, control,
            error.empty()
                ? "multi-dexel difference calculation failed"
                : std::move(error));
        return result;
    }
    result.heightFieldDifferenceMilliseconds =
        elapsedMilliseconds(differenceStart);
    result.initialMissingVolume =
        differences.initialMissingVolume;
    result.remainingAllowanceVolume =
        differences.remainingAllowanceVolume;
    result.overcutVolume =
        differences.overcutVolume;
    result.heightFieldCertainOvercutVolume =
        differences.certainOvercutVolume;
    result.heightFieldBoundaryUncertaintyVolume =
        std::max(
            0.0,
            result.overcutVolume -
                result.heightFieldCertainOvercutVolume);
    buildMultiDexelFeatureAllowances(
        *dataSource, target, tolerance, control,
        result.featureAllowances);
    result.heightFieldDifferenceSolidCount =
        static_cast<int>(
            differences.initialMissing.intervals.size() +
            differences.remainingAllowance.intervals.size() +
            differences.overcut.intervals.size());
    progress(4);

    const double volumeTolerance = std::max(
        tolerance * tolerance * tolerance,
        1.0e-9 * std::max({
            1.0,
            result.initialVolume,
            result.targetVolume}));
    result.targetContainmentRatio =
        result.targetVolume > volumeTolerance
        ? std::clamp(
              1.0 -
                  result.initialMissingVolume /
                      result.targetVolume,
              0.0, 1.0)
        : 0.0;
    result.initialContainsTarget =
        result.initialMissingVolume <=
        volumeTolerance;
    if (!result.initialContainsTarget) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::
                StockTargetOutsideInitial,
            ToolpathDiagnosticSeverity::Error,
            "target material is missing from the initial multi-dexel stock");
    }
    if (result.remainingAllowanceVolume >
        volumeTolerance) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::
                StockRemainingAllowance,
            ToolpathDiagnosticSeverity::Info,
            "multi-dexel material remains outside the target after current operations");
    }
    if (result.heightFieldCertainOvercutVolume >
        volumeTolerance) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::
                StockOvercutDetected,
            ToolpathDiagnosticSeverity::Error,
            "current operations removed multi-dexel material in target-interior cells");
    } else if (
        result.heightFieldBoundaryUncertaintyVolume >
        volumeTolerance) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::
                StockOvercutDetected,
            ToolpathDiagnosticSeverity::Warning,
            "potential multi-dexel overcut is limited to target-boundary cells; refine the grid or run an explicit BRep audit");
    }

    auto displayImpl =
        std::make_shared<
            PocketZMapDisplaySource::Impl>();
    displayImpl->multiDexelData =
        std::move(dataSource);
    displayImpl->multiDexelTarget =
        std::move(target);
    displayImpl->multiDexelInitialMissing =
        std::move(differences.initialMissing);
    displayImpl->multiDexelRemainingAllowance =
        std::move(differences.remainingAllowance);
    displayImpl->multiDexelOvercut =
        std::move(differences.overcut);
    displayImpl->tolerance = tolerance;
    displayImpl->fingerprint =
        multiDexelDisplaySourceFingerprint(
            *displayImpl->multiDexelData,
            displayImpl->multiDexelTarget);
    result.heightFieldDisplaySource =
        std::shared_ptr<const PocketZMapDisplaySource>(
            new PocketZMapDisplaySource(
                std::move(displayImpl)));
    result.complete = true;
    return result;
}

StockOvercutAttributionResult
MultiDexelComparison::attributeOvercut(
    std::shared_ptr<const MultiDexelComparisonData>
        dataSource,
    const TopoDS_Shape& targetShape,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control,
    std::shared_ptr<const PocketZMapDisplaySource>
        displaySource) const
{
    StockOvercutAttributionResult result;
    ScopedElapsedMilliseconds totalTimer(
        result.totalMilliseconds);
    if (!dataSource || !dataSource->valid() ||
        targetShape.IsNull()) {
        addAttributionDiagnostic(
            result,
            ToolpathDiagnosticCode::
                StockComparisonFailed,
            ToolpathDiagnosticSeverity::Error,
            "multi-dexel overcut attribution requires valid checkpoints and target");
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
        displaySource->m_impl
                ->multiDexelData.get() !=
            dataSource.get()) {
        DexelDisplayGrid target;
        MultiDexelBuildMetrics targetMetrics;
        std::string error;
        if (!dataSource->initial
                 ->rasterizeShapeOnGrid(
                     targetShape,
                     target,
                     &targetMetrics,
                     &error,
                     [control] {
                         return cancelled(control);
                     },
                     options.runParallel,
                     tolerance)) {
            if (cancelled(control)) {
                result.cancelled = true;
                addAttributionDiagnostic(
                    result,
                    ToolpathDiagnosticCode::
                        GenerationCancelled,
                    ToolpathDiagnosticSeverity::
                        Warning,
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
        MultiDexelDifferenceSet differences;
        if (!buildMultiDexelDifferences(
                *dataSource, target, tolerance,
                control, differences, error)) {
            if (cancelled(control)) {
                result.cancelled = true;
            }
            addAttributionDiagnostic(
                result,
                result.cancelled
                    ? ToolpathDiagnosticCode::
                          GenerationCancelled
                    : ToolpathDiagnosticCode::
                          StockComparisonFailed,
                result.cancelled
                    ? ToolpathDiagnosticSeverity::
                          Warning
                    : ToolpathDiagnosticSeverity::
                          Error,
                std::move(error));
            return result;
        }
        auto displayImpl =
            std::make_shared<
                PocketZMapDisplaySource::Impl>();
        displayImpl->multiDexelData =
            dataSource;
        displayImpl->multiDexelTarget =
            std::move(target);
        displayImpl->multiDexelInitialMissing =
            std::move(differences.initialMissing);
        displayImpl->multiDexelRemainingAllowance =
            std::move(
                differences.remainingAllowance);
        displayImpl->multiDexelOvercut =
            std::move(differences.overcut);
        displayImpl->tolerance = tolerance;
        displayImpl->fingerprint =
            multiDexelDisplaySourceFingerprint(
                *displayImpl->multiDexelData,
                displayImpl->multiDexelTarget);
        displaySource =
            std::shared_ptr<const
                PocketZMapDisplaySource>(
                new PocketZMapDisplaySource(
                    std::move(displayImpl)));
    }
    result.heightFieldDisplaySource =
        displaySource;
    const DexelDisplayGrid& target =
        displaySource->m_impl->multiDexelTarget;
    const double targetVolume =
        displayGridVolume(target);
    const double volumeTolerance = std::max(
        tolerance * tolerance * tolerance,
        1.0e-9 * std::max({
            1.0,
            dataSource->initial->volume(),
            targetVolume}));
    if (control) {
        control->progress(
            0, dataSource->operations.size());
    }
    for (std::size_t operationIndex = 0;
         operationIndex <
             dataSource->operations.size();
         ++operationIndex) {
        if (cancelled(control)) {
            result.cancelled = true;
            addAttributionDiagnostic(
                result,
                ToolpathDiagnosticCode::
                    GenerationCancelled,
                ToolpathDiagnosticSeverity::Warning,
                "multi-dexel overcut attribution was cancelled");
            return result;
        }
        const MultiDexelOperationState& operation =
            dataSource->operations[operationIndex];
        DexelDisplayGrid operationGrid;
        double volume = 0.0;
        double certainVolume = 0.0;
        std::string error;
        if (!buildMultiDexelOperationOvercut(
                *dataSource, target, operation,
                tolerance,
                [control] {
                    return cancelled(control);
                },
                operationGrid,
                volume,
                certainVolume,
                error)) {
            if (cancelled(control)) {
                result.cancelled = true;
            }
            addAttributionDiagnostic(
                result,
                result.cancelled
                    ? ToolpathDiagnosticCode::
                          GenerationCancelled
                    : ToolpathDiagnosticCode::
                          StockComparisonFailed,
                result.cancelled
                    ? ToolpathDiagnosticSeverity::
                          Warning
                    : ToolpathDiagnosticSeverity::
                          Error,
                std::move(error),
                operation.operationId);
            return result;
        }
        if (volume > volumeTolerance) {
            StockOperationOvercut attribution;
            attribution.operationId =
                operation.operationId;
            attribution.sequenceIndex =
                operation.sequenceIndex;
            attribution.volume = volume;
            attribution.certainVolume =
                certainVolume;
            attribution.boundaryUncertaintyVolume =
                std::max(
                    0.0,
                    volume - certainVolume);
            result.attributedVolume += volume;
            result.operations.push_back(
                std::move(attribution));
            addAttributionDiagnostic(
                result,
                ToolpathDiagnosticCode::
                    StockOvercutDetected,
                certainVolume > volumeTolerance
                    ? ToolpathDiagnosticSeverity::Error
                    : ToolpathDiagnosticSeverity::Warning,
                operation.operationId +
                    (certainVolume > volumeTolerance
                         ? ": operation removed target-interior multi-dexel material"
                         : ": potential overcut is limited to target-boundary multi-dexel cells"),
                operation.operationId);
        }
        if (control) {
            control->progress(
                operationIndex + 1u,
                dataSource->operations.size());
        }
    }
    result.complete = true;
    return result;
}

} // namespace oap::toolpath

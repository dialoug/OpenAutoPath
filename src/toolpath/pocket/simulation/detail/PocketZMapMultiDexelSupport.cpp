// Independently compiled private PocketZMapMultiDexelSupport implementation.
#include "toolpath/pocket/simulation/detail/PocketZMapMultiDexelSupport.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace oap::toolpath::zmap_detail {

bool buildMultiDexelDifferences(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    double tolerance,
    const ToolpathGenerationControl* control,
    MultiDexelDifferenceSet& output,
    std::string& error)
{
    output = {};
    output.initialMissing =
        gridLike(*data.initial, tolerance);
    output.remainingAllowance =
        gridLike(*data.initial, tolerance);
    output.overcut =
        gridLike(*data.initial, tolerance);
    std::vector<DexelInterval> missing;
    std::vector<DexelInterval> remaining;
    std::vector<DexelInterval> initialTarget;
    std::vector<DexelInterval> overcut;
    missing.reserve(4u);
    remaining.reserve(4u);
    initialTarget.reserve(4u);
    overcut.reserve(4u);
    for (int j = 0; j < data.initial->ny(); ++j) {
        if (cancelled(control)) {
            error =
                "multi-dexel difference calculation was cancelled";
            return false;
        }
        for (int i = 0; i < data.initial->nx(); ++i) {
            const auto initial =
                data.initial->cellIntervalsView(i, j);
            const auto final =
                data.final->cellIntervalsView(i, j);
            const auto targetIntervals =
                target.intervalsAt(i, j);
            subtractIntervals(
                targetIntervals, initial,
                tolerance, missing);
            subtractIntervals(
                final, targetIntervals,
                tolerance, remaining);
            intersectIntervals(
                initial, targetIntervals,
                tolerance, initialTarget);
            subtractIntervals(
                initialTarget, final,
                tolerance, overcut);
            const double area =
                multiDexelCellArea(
                    *data.initial, i, j);
            if (!appendCellIntervals(
                    output.initialMissing, missing,
                    area,
                    output.initialMissingVolume,
                    error) ||
                !appendCellIntervals(
                    output.remainingAllowance,
                    remaining, area,
                    output.remainingAllowanceVolume,
                    error) ||
                !appendCellIntervals(
                    output.overcut, overcut,
                    area, output.overcutVolume,
                    error)) {
                return false;
            }
            if (!overcut.empty() &&
                !multiDexelTargetBoundaryCell(
                    target, i, j, tolerance)) {
                for (const DexelInterval& interval :
                     overcut) {
                    output.certainOvercutVolume +=
                        (interval.upper -
                         interval.lower) *
                        area;
                }
            }
        }
    }
    return output.initialMissing.valid() &&
           output.remainingAllowance.valid() &&
           output.overcut.valid();
}

void buildMultiDexelFeatureAllowances(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    double tolerance,
    const ToolpathGenerationControl* control,
    std::vector<StockFeatureAllowance>& output)
{
    output.clear();
    output.reserve(data.featureAllowanceScopes.size());
    for (const StockFeatureAllowanceScope& scope :
         data.featureAllowanceScopes) {
        if (cancelled(control) || scope.region.IsNull() ||
            scope.featureId.empty()) {
            continue;
        }
        DexelDisplayGrid region;
        std::string error;
        if (!data.initial->rasterizeShapeOnGrid(
                scope.region, region, nullptr, &error,
                [control] { return cancelled(control); },
                true, tolerance, true) ||
            !region.valid()) {
            continue;
        }

        double targetRemovalVolume = 0.0;
        double remainingVolume = 0.0;
        std::vector<DexelInterval> outsideTarget;
        std::vector<DexelInterval> scopedTarget;
        std::vector<DexelInterval> remaining;
        for (int j = 0; j < data.initial->ny(); ++j) {
            if (cancelled(control))
                break;
            for (int i = 0; i < data.initial->nx(); ++i) {
                subtractIntervals(
                    data.initial->cellIntervalsView(i, j),
                    target.intervalsAt(i, j), tolerance,
                    outsideTarget);
                intersectIntervals(
                    outsideTarget, region.intervalsAt(i, j),
                    tolerance, scopedTarget);
                intersectIntervals(
                    data.final->cellIntervalsView(i, j),
                    scopedTarget, tolerance, remaining);
                const double area = multiDexelCellArea(
                    *data.initial, i, j);
                for (const DexelInterval& interval : scopedTarget) {
                    targetRemovalVolume +=
                        (interval.upper - interval.lower) * area;
                }
                for (const DexelInterval& interval : remaining) {
                    remainingVolume +=
                        (interval.upper - interval.lower) * area;
                }
            }
        }
        if (cancelled(control) ||
            targetRemovalVolume <= tolerance * tolerance * tolerance) {
            continue;
        }
        StockFeatureAllowance allowance;
        allowance.featureId = scope.featureId;
        allowance.operationId = scope.operationId;
        allowance.featureKind = scope.featureKind;
        allowance.sourceIndex = scope.sourceIndex;
        allowance.targetRemovalVolume = targetRemovalVolume;
        allowance.remainingAllowanceVolume = std::clamp(
            remainingVolume, 0.0, targetRemovalVolume);
        allowance.completionRatio = std::clamp(
            1.0 - allowance.remainingAllowanceVolume /
                      targetRemovalVolume,
            0.0, 1.0);
        allowance.approximateBoundary =
            scope.approximateBoundary;
        output.push_back(std::move(allowance));
    }
}

bool buildMultiDexelFeatureAllowanceGrid(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    const std::string& featureId,
    double tolerance,
    std::function<bool()> cancellationRequested,
    DexelDisplayGrid& output,
    std::string& error)
{
    const auto found = std::find_if(
        data.featureAllowanceScopes.begin(),
        data.featureAllowanceScopes.end(),
        [&](const StockFeatureAllowanceScope& scope) {
            return scope.featureId == featureId;
        });
    if (found == data.featureAllowanceScopes.end() ||
        found->region.IsNull()) {
        error = "multi-dexel feature allowance scope was not found: " +
            featureId;
        return false;
    }
    DexelDisplayGrid region;
    if (!data.initial->rasterizeShapeOnGrid(
            found->region, region, nullptr, &error,
            cancellationRequested, true, tolerance, true) ||
        !region.valid()) {
        return false;
    }
    output = gridLike(*data.initial, tolerance);
    double ignoredVolume = 0.0;
    std::vector<DexelInterval> outsideTarget;
    std::vector<DexelInterval> scopedTarget;
    std::vector<DexelInterval> remaining;
    for (int j = 0; j < data.initial->ny(); ++j) {
        if (cancellationRequested && cancellationRequested()) {
            error = "multi-dexel feature allowance calculation was cancelled";
            return false;
        }
        for (int i = 0; i < data.initial->nx(); ++i) {
            subtractIntervals(
                data.initial->cellIntervalsView(i, j),
                target.intervalsAt(i, j), tolerance, outsideTarget);
            intersectIntervals(
                outsideTarget, region.intervalsAt(i, j),
                tolerance, scopedTarget);
            intersectIntervals(
                data.final->cellIntervalsView(i, j),
                scopedTarget, tolerance, remaining);
            if (!appendCellIntervals(
                    output, remaining,
                    multiDexelCellArea(*data.initial, i, j),
                    ignoredVolume, error)) {
                return false;
            }
        }
    }
    return output.valid();
}

void buildZMapFeatureAllowances(
    const PocketZMapSimulationData& data,
    const TargetField& target,
    double tolerance,
    const ToolpathGenerationControl* control,
    std::vector<StockFeatureAllowance>& output)
{
    output.clear();
    if (!data.gridSource || !data.gridSource->valid())
        return;
    output.reserve(data.featureAllowanceScopes.size());
    for (const StockFeatureAllowanceScope& scope :
         data.featureAllowanceScopes) {
        if (cancelled(control) || scope.region.IsNull() ||
            scope.featureId.empty()) {
            continue;
        }
        DexelDisplayGrid region;
        std::string error;
        if (!data.gridSource->rasterizeShapeOnGrid(
                scope.region, region, nullptr, &error,
                [control] { return cancelled(control); },
                true, tolerance, true) || !region.valid()) {
            continue;
        }

        double targetRemovalVolume = 0.0;
        double remainingVolume = 0.0;
        std::vector<DexelInterval> initial;
        std::vector<DexelInterval> targetIntervals;
        std::vector<DexelInterval> final;
        std::vector<DexelInterval> outsideTarget;
        std::vector<DexelInterval> scopedTarget;
        std::vector<DexelInterval> remaining;
        initial.reserve(1u);
        targetIntervals.reserve(1u);
        final.reserve(1u);
        for (int j = 0; j < data.ny; ++j) {
            if (cancelled(control))
                break;
            for (int i = 0; i < data.nx; ++i) {
                const std::size_t cell = offset(data, i, j);
                initial.clear();
                targetIntervals.clear();
                final.clear();
                if (finiteInterval(
                        data.bottoms[cell], data.initialTops[cell],
                        tolerance)) {
                    initial.push_back(
                        {data.bottoms[cell], data.initialTops[cell]});
                }
                if (finiteInterval(
                        target.bottoms[cell], target.tops[cell],
                        tolerance)) {
                    targetIntervals.push_back(
                        {target.bottoms[cell], target.tops[cell]});
                }
                if (finiteInterval(
                        data.bottoms[cell], data.finalTops[cell],
                        tolerance)) {
                    final.push_back(
                        {data.bottoms[cell], data.finalTops[cell]});
                }
                subtractIntervals(
                    initial, targetIntervals, tolerance, outsideTarget);
                intersectIntervals(
                    outsideTarget, region.intervalsAt(i, j),
                    tolerance, scopedTarget);
                intersectIntervals(
                    final, scopedTarget, tolerance, remaining);
                const double area = cellArea(data, i, j);
                for (const DexelInterval& interval : scopedTarget) {
                    targetRemovalVolume +=
                        (interval.upper - interval.lower) * area;
                }
                for (const DexelInterval& interval : remaining) {
                    remainingVolume +=
                        (interval.upper - interval.lower) * area;
                }
            }
        }
        if (cancelled(control) ||
            targetRemovalVolume <= tolerance * tolerance * tolerance) {
            continue;
        }
        StockFeatureAllowance allowance;
        allowance.featureId = scope.featureId;
        allowance.operationId = scope.operationId;
        allowance.featureKind = scope.featureKind;
        allowance.sourceIndex = scope.sourceIndex;
        allowance.targetRemovalVolume = targetRemovalVolume;
        allowance.remainingAllowanceVolume = std::clamp(
            remainingVolume, 0.0, targetRemovalVolume);
        allowance.completionRatio = std::clamp(
            1.0 - allowance.remainingAllowanceVolume /
                      targetRemovalVolume,
            0.0, 1.0);
        allowance.approximateBoundary =
            scope.approximateBoundary;
        output.push_back(std::move(allowance));
    }
}

bool buildMultiDexelOperationOvercut(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target,
    const MultiDexelOperationState& operation,
    double tolerance,
    std::function<bool()> cancellationRequested,
    DexelDisplayGrid& output,
    double& volume,
    double& certainVolume,
    std::string& error)
{
    output = gridLike(*data.initial, tolerance);
    volume = 0.0;
    certainVolume = 0.0;
    std::vector<DexelInterval> removed;
    std::vector<DexelInterval> overcut;
    removed.reserve(4u);
    overcut.reserve(4u);
    for (int j = 0; j < data.initial->ny(); ++j) {
        if (cancellationRequested &&
            cancellationRequested()) {
            error =
                "multi-dexel operation overcut calculation was cancelled";
            return false;
        }
        for (int i = 0; i < data.initial->nx(); ++i) {
            subtractIntervals(
                operation.before->cellIntervalsView(i, j),
                operation.after->cellIntervalsView(i, j),
                tolerance, removed);
            intersectIntervals(
                removed, target.intervalsAt(i, j),
                tolerance, overcut);
            const double area =
                multiDexelCellArea(
                    *data.initial, i, j);
            if (!appendCellIntervals(
                    output, overcut, area,
                    volume, error)) {
                return false;
            }
            if (!overcut.empty() &&
                !multiDexelTargetBoundaryCell(
                    target, i, j, tolerance)) {
                for (const DexelInterval& interval :
                     overcut) {
                    certainVolume +=
                        (interval.upper -
                         interval.lower) *
                        area;
                }
            }
        }
    }
    return output.valid();
}

double displayGridVolume(
    const DexelDisplayGrid& grid)
{
    double volume = 0.0;
    for (int j = 0; j < grid.ny; ++j) {
        for (int i = 0; i < grid.nx; ++i) {
            const double u0 =
                grid.uMin +
                static_cast<double>(i) *
                    grid.resolution;
            const double u1 =
                std::min(
                    grid.uMax,
                    u0 + grid.resolution);
            const double v0 =
                grid.vMin +
                static_cast<double>(j) *
                    grid.resolution;
            const double v1 =
                std::min(
                    grid.vMax,
                    v0 + grid.resolution);
            const double area =
                (u1 - u0) * (v1 - v0);
            for (const DexelInterval& interval :
                 grid.intervalsAt(i, j)) {
                volume +=
                    (interval.upper -
                     interval.lower) *
                    area;
            }
        }
    }
    return volume;
}

std::string multiDexelDisplaySourceFingerprint(
    const MultiDexelComparisonData& data,
    const DexelDisplayGrid& target)
{
    ContentFingerprint hash;
    hash.add(std::uint64_t{0x6d756c7469646973ull});
    hash.add(data.initial->contentFingerprint());
    hash.add(data.final->contentFingerprint());
    hash.add(static_cast<std::uint64_t>(
        target.offsets.size()));
    for (const std::uint32_t value : target.offsets)
        hash.add(static_cast<std::uint64_t>(value));
    for (const DexelInterval& interval :
         target.intervals) {
        hash.add(interval.lower);
        hash.add(interval.upper);
    }
    hash.add(static_cast<std::uint64_t>(
        data.operations.size()));
    for (const MultiDexelOperationState& operation :
         data.operations) {
        hash.add(operation.operationId);
        hash.add(static_cast<std::uint64_t>(
            operation.sequenceIndex));
        hash.add(
            operation.before->contentFingerprint());
        hash.add(
            operation.after->contentFingerprint());
    }
    return hash.finish();
}

void reportComparisonFailure(
    StockComparisonResult& result,
    const ToolpathGenerationControl* control,
    std::string message)
{
    if (cancelled(control)) {
        result.cancelled = true;
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::GenerationCancelled,
            ToolpathDiagnosticSeverity::Warning,
            std::move(message));
    } else {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::StockComparisonFailed,
            ToolpathDiagnosticSeverity::Error,
            std::move(message));
    }
}

} // namespace oap::toolpath::zmap_detail

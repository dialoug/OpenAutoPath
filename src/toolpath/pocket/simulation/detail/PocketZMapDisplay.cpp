// Independently compiled private PocketZMapDisplay implementation.
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Poly_Triangulation.hxx>

#include "toolpath/pocket/simulation/DexelDisplayMesh.h"
#include "toolpath/pocket/simulation/detail/PocketZMapDisplayState.h"
#include "toolpath/pocket/simulation/detail/PocketZMapMultiDexelSupport.h"

namespace oap::toolpath {
using namespace oap::toolpath::zmap_detail;

PocketZMapDisplaySource::PocketZMapDisplaySource(
    std::shared_ptr<const Impl> impl)
    : m_impl(std::move(impl))
{
}

const std::string&
PocketZMapDisplaySource::contentFingerprint() const noexcept
{
    static const std::string empty;
    return m_impl ? m_impl->fingerprint : empty;
}

bool PocketZMapDisplaySource::buildDisplayMesh(
    PocketZMapDifferenceView view,
    Handle(Poly_Triangulation)& output,
    DexelDisplayMeshMetrics* metrics,
    std::string operationId,
    std::string* error,
    std::function<bool()> cancellationRequested,
    const DexelDisplayMeshOptions& options) const
{
    output.Nullify();
    if (metrics)
        *metrics = {};
    auto fail = [&](std::string message) {
        if (error)
            *error = std::move(message);
        return false;
    };
    if (m_impl && m_impl->multiDexelData) {
        const DexelDisplayGrid* grid = nullptr;
        DexelDisplayGrid operationGrid;
        switch (view) {
            case PocketZMapDifferenceView::InitialMissing:
                grid =
                    &m_impl->multiDexelInitialMissing;
                break;
            case PocketZMapDifferenceView::
                RemainingAllowance:
                grid =
                    &m_impl->multiDexelRemainingAllowance;
                break;
            case PocketZMapDifferenceView::Overcut:
                grid = &m_impl->multiDexelOvercut;
                break;
            case PocketZMapDifferenceView::FeatureAllowance: {
                std::string buildError;
                if (!buildMultiDexelFeatureAllowanceGrid(
                        *m_impl->multiDexelData,
                        m_impl->multiDexelTarget,
                        operationId,
                        m_impl->tolerance,
                        cancellationRequested,
                        operationGrid,
                        buildError)) {
                    return fail(std::move(buildError));
                }
                grid = &operationGrid;
                break;
            }
            case PocketZMapDifferenceView::
                OperationOvercut: {
                const auto found = std::find_if(
                    m_impl->multiDexelData
                        ->operations.begin(),
                    m_impl->multiDexelData
                        ->operations.end(),
                    [&](const MultiDexelOperationState&
                            candidate) {
                        return candidate.operationId ==
                               operationId;
                    });
                if (found ==
                    m_impl->multiDexelData
                        ->operations.end()) {
                    return fail(
                        "multi-dexel difference operation was not found: " +
                        operationId);
                }
                double volume = 0.0;
                double certainVolume = 0.0;
                std::string buildError;
                if (!buildMultiDexelOperationOvercut(
                        *m_impl->multiDexelData,
                        m_impl->multiDexelTarget,
                        *found,
                        m_impl->tolerance,
                        cancellationRequested,
                        operationGrid,
                        volume,
                        certainVolume,
                        buildError)) {
                    return fail(std::move(buildError));
                }
                grid = &operationGrid;
                break;
            }
        }
        if (!grid || !grid->valid()) {
            return fail(
                "multi-dexel difference display source is invalid");
        }
        if (grid->intervals.empty()) {
            return fail(
                "multi-dexel difference contains no display intervals");
        }
        return buildDexelDisplayMesh(
            *grid, output, metrics, error,
            std::move(cancellationRequested),
            options);
    }
    if (!m_impl || !m_impl->data ||
        !m_impl->data->valid()) {
        return fail(
            "Z-map difference display source is invalid");
    }
    const PocketZMapSimulationData& data =
        *m_impl->data;
    if (m_impl->target.bottoms.size() !=
            data.cellCount() ||
        m_impl->target.tops.size() !=
            data.cellCount()) {
        return fail(
            "Z-map difference target grid is invalid");
    }

    DexelDisplayGrid featureRegion;
    if (view == PocketZMapDifferenceView::FeatureAllowance) {
        const auto found = std::find_if(
            data.featureAllowanceScopes.begin(),
            data.featureAllowanceScopes.end(),
            [&](const StockFeatureAllowanceScope& scope) {
                return scope.featureId == operationId;
            });
        if (found == data.featureAllowanceScopes.end() ||
            found->region.IsNull()) {
            return fail(
                "Z-map feature allowance scope was not found: " +
                operationId);
        }
        std::string rasterError;
        if (!data.gridSource ||
            !data.gridSource->rasterizeShapeOnGrid(
                found->region, featureRegion, nullptr, &rasterError,
                cancellationRequested, true, m_impl->tolerance, true) ||
            !featureRegion.valid()) {
            return fail(std::move(rasterError));
        }
    }

    const PocketZMapOperationDelta* operation = nullptr;
    std::vector<PocketZMapCellDelta> changes;
    if (view ==
        PocketZMapDifferenceView::OperationOvercut) {
        const auto found = std::find_if(
            data.operationDeltas.begin(),
            data.operationDeltas.end(),
            [&](const PocketZMapOperationDelta& candidate) {
                return candidate.operationId ==
                       operationId;
            });
        if (found == data.operationDeltas.end()) {
            return fail(
                "Z-map difference operation was not found: " +
                operationId);
        }
        operation = &*found;
        changes = operation->cells;
        std::sort(
            changes.begin(), changes.end(),
            [](const PocketZMapCellDelta& left,
               const PocketZMapCellDelta& right) {
                return left.cellIndex <
                       right.cellIndex;
            });
    }

    DexelDisplayGrid grid;
    grid.origin = data.origin;
    grid.uAxis = data.uAxis;
    grid.vAxis = data.vAxis;
    grid.wAxis = data.wAxis;
    grid.uMin = data.uMin;
    grid.uMax = data.uMax;
    grid.vMin = data.vMin;
    grid.vMax = data.vMax;
    grid.resolution = data.resolution;
    grid.tolerance = m_impl->tolerance;
    grid.nx = data.nx;
    grid.ny = data.ny;
    grid.offsets.reserve(data.cellCount() + 1u);
    grid.offsets.push_back(0u);
    grid.intervals.reserve(
        operation
        ? changes.size()
        : std::min<std::size_t>(
              data.cellCount(), 500'000u));

    auto append = [&](double lower, double upper) {
        if (!finiteInterval(
                lower, upper, m_impl->tolerance)) {
            return;
        }
        if (!grid.intervals.empty() &&
            grid.offsets.back() <
                grid.intervals.size() &&
            lower <=
                grid.intervals.back().upper +
                    m_impl->tolerance) {
            grid.intervals.back().upper =
                std::max(
                    grid.intervals.back().upper,
                    upper);
            return;
        }
        grid.intervals.push_back({lower, upper});
    };

    std::size_t changeIndex = 0u;
    std::vector<DexelInterval> initialIntervals;
    std::vector<DexelInterval> targetIntervals;
    std::vector<DexelInterval> finalIntervals;
    std::vector<DexelInterval> outsideTarget;
    std::vector<DexelInterval> scopedTarget;
    std::vector<DexelInterval> remaining;
    for (std::size_t cell = 0;
         cell < data.cellCount(); ++cell) {
        if (cancellationRequested &&
            cell % static_cast<std::size_t>(
                       std::max(1, data.nx)) == 0u &&
            cancellationRequested()) {
            return fail(
                "Z-map difference display mesh was cancelled");
        }
        const double stockBottom =
            data.bottoms[cell];
        const double initialTop =
            data.initialTops[cell];
        const double finalTop =
            data.finalTops[cell];
        const double targetBottom =
            m_impl->target.bottoms[cell];
        const double targetTop =
            m_impl->target.tops[cell];
        switch (view) {
            case PocketZMapDifferenceView::InitialMissing:
                if (finiteInterval(
                        targetBottom,
                        targetTop,
                        m_impl->tolerance)) {
                    if (!finiteInterval(
                            stockBottom,
                            initialTop,
                            m_impl->tolerance)) {
                        append(targetBottom, targetTop);
                    } else {
                        append(
                            targetBottom,
                            std::min(
                                targetTop,
                                stockBottom));
                        append(
                            std::max(
                                targetBottom,
                                initialTop),
                            targetTop);
                    }
                }
                break;
            case PocketZMapDifferenceView::
                RemainingAllowance:
                if (finiteInterval(
                        stockBottom,
                        finalTop,
                        m_impl->tolerance)) {
                    if (!finiteInterval(
                            targetBottom,
                            targetTop,
                            m_impl->tolerance)) {
                        append(stockBottom, finalTop);
                    } else {
                        append(
                            stockBottom,
                            std::min(
                                finalTop,
                                targetBottom));
                        append(
                            std::max(
                                stockBottom,
                                targetTop),
                            finalTop);
                    }
                }
                break;
            case PocketZMapDifferenceView::Overcut:
                if (finiteInterval(
                        stockBottom,
                        initialTop,
                        m_impl->tolerance) &&
                    finiteInterval(
                        targetBottom,
                        targetTop,
                        m_impl->tolerance)) {
                    append(
                        std::max({
                            stockBottom,
                            targetBottom,
                            finalTop}),
                        std::min(
                            initialTop,
                            targetTop));
                }
                break;
            case PocketZMapDifferenceView::FeatureAllowance: {
                initialIntervals.clear();
                targetIntervals.clear();
                finalIntervals.clear();
                if (finiteInterval(
                        stockBottom, initialTop, m_impl->tolerance)) {
                    initialIntervals.push_back({stockBottom, initialTop});
                }
                if (finiteInterval(
                        targetBottom, targetTop, m_impl->tolerance)) {
                    targetIntervals.push_back({targetBottom, targetTop});
                }
                if (finiteInterval(
                        stockBottom, finalTop, m_impl->tolerance)) {
                    finalIntervals.push_back({stockBottom, finalTop});
                }
                subtractIntervals(
                    initialIntervals, targetIntervals,
                    m_impl->tolerance, outsideTarget);
                const int i = static_cast<int>(
                    cell % static_cast<std::size_t>(data.nx));
                const int j = static_cast<int>(
                    cell / static_cast<std::size_t>(data.nx));
                intersectIntervals(
                    outsideTarget, featureRegion.intervalsAt(i, j),
                    m_impl->tolerance, scopedTarget);
                intersectIntervals(
                    finalIntervals, scopedTarget,
                    m_impl->tolerance, remaining);
                for (const DexelInterval& interval : remaining)
                    append(interval.lower, interval.upper);
                break;
            }
            case PocketZMapDifferenceView::
                OperationOvercut:
                while (changeIndex < changes.size() &&
                       changes[changeIndex].cellIndex <
                           static_cast<int>(cell)) {
                    ++changeIndex;
                }
                while (changeIndex < changes.size() &&
                       changes[changeIndex].cellIndex ==
                           static_cast<int>(cell)) {
                    const PocketZMapCellDelta& change =
                        changes[changeIndex++];
                    if (finiteInterval(
                            stockBottom,
                            initialTop,
                            m_impl->tolerance) &&
                        finiteInterval(
                            targetBottom,
                            targetTop,
                            m_impl->tolerance)) {
                        append(
                            std::max({
                                change.afterTop,
                                stockBottom,
                                targetBottom}),
                            std::min({
                                change.beforeTop,
                                initialTop,
                                targetTop}));
                    }
                }
                break;
        }
        if (grid.intervals.size() >
            std::numeric_limits<std::uint32_t>::max()) {
            return fail(
                "Z-map difference interval count exceeds display capacity");
        }
        grid.offsets.push_back(
            static_cast<std::uint32_t>(
                grid.intervals.size()));
    }
    if (grid.intervals.empty()) {
        return fail(
            "Z-map difference contains no display intervals");
    }
    return buildDexelDisplayMesh(
        grid, output, metrics, error,
        std::move(cancellationRequested),
        options);
}

} // namespace oap::toolpath

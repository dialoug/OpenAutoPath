// 当前 Multi-Dexel/IPW 材料真值 E2E：不可变减料、取消回滚和比较预算。
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/pocket/simulation/StockTargetCache.h"

#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Compound.hxx>

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <array>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

oap::toolpath::StockSimulationOptions options(bool parallel = false)
{
    oap::toolpath::StockSimulationOptions result;
    result.zMapResolution = 1.0;
    result.zMapMaxCellCount = 100000;
    result.zMapTileSize = 8;
    result.runParallel = parallel;
    return result;
}

bool holeAllowanceRegionCoversConicalBottomAndMargins()
{
    oap::RecognizedHoleGeometry hole;
    hole.entry = gp_Pnt(0.0, 0.0, 0.0);
    hole.bottom = gp_Pnt(0.0, 0.0, 5.0);
    hole.radius = 2.0;
    hole.bottomProfile = oap::HoleBottomProfile::Conical;
    const TopoDS_Shape region =
        oap::toolpath::buildHoleFeatureAllowanceRegion(hole, 0.25);
    if (!expect(!region.IsNull(),
                "conical hole allowance region should be valid")) {
        return false;
    }
    Bnd_Box bounds;
    BRepBndLib::AddOptimal(
        region, bounds, Standard_False, Standard_False);
    Standard_Real xMin = 0.0;
    Standard_Real yMin = 0.0;
    Standard_Real zMin = 0.0;
    Standard_Real xMax = 0.0;
    Standard_Real yMax = 0.0;
    Standard_Real zMax = 0.0;
    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    return expect(
        zMin <= -0.249 && zMax >= 6.999,
        "conical hole allowance region should cover entry margin and tip depth");
}

std::shared_ptr<const oap::toolpath::MultiDexelStock> rasterize(
    const TopoDS_Shape& shape,
    std::string& error)
{
    return oap::toolpath::MultiDexelStock::fromShape(
        shape,
        gp_Vec(0.0, 0.0, 1.0),
        2.0,
        options(),
        nullptr,
        &error);
}

oap::toolpath::PocketToolpathOperation pocketCut(
    double x0,
    double x1,
    double y,
    double z)
{
    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "multi-dexel-pocket";
    operation.outwardToolAxis = gp_Vec(0.0, 0.0, 1.0);
    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(x0, y, z);
    cut.to = gp_Pnt(x1, y, z);
    operation.clearing.moves.push_back(cut);
    return operation;
}

oap::toolpath::ToolpathParams cutterParams(double diameter = 2.0)
{
    oap::toolpath::ToolpathParams params;
    params.tool.diameter = diameter;
    params.tool.cuttingEdgeLength = 2.0;
    return params;
}

bool immutableRemovalIsMonotonicAndDeterministic()
{
    std::string error;
    const auto source = rasterize(
        BRepPrimAPI_MakeBox(64.0, 32.0, 10.0).Shape(), error);
    if (!expect(source && source->valid(),
                "the initial Multi-Dexel stock should rasterize")) {
        if (!error.empty())
            std::cerr << error << '\n';
        return false;
    }

    const auto operation = pocketCut(2.0, 62.0, 16.0, 4.0);
    const auto params = cutterParams(4.0);
    const auto serial = source->applyPocket(
        operation, params, 0, options(false));
    const auto parallel = source->applyPocket(
        operation, params, 0, options(true));
    if (!expect(serial.record.status ==
                        oap::toolpath::StockRemovalStatus::Applied &&
                    parallel.record.status ==
                        oap::toolpath::StockRemovalStatus::Applied &&
                    serial.stock && parallel.stock,
                "serial and parallel removal should publish complete states") ||
        !expect(serial.stock->volume() < source->volume() &&
                    serial.record.volumeAfter <=
                        serial.record.volumeBefore,
                "material volume must be monotonically non-increasing") ||
        !expect(serial.stock->contentFingerprint() ==
                        parallel.stock->contentFingerprint() &&
                    serial.changedCellCount == parallel.changedCellCount &&
                    serial.touchedTileCount == parallel.touchedTileCount,
                "parallel tile scheduling must preserve deterministic truth") ||
        !expect(source->contentFingerprint() !=
                        serial.stock->contentFingerprint() &&
                    serial.stock->sharedTileCountWith(*source) > 0,
                "a child should change content while sharing untouched tiles")) {
        return false;
    }

    const std::string parentFingerprint = source->contentFingerprint();
    const double parentVolume = source->volume();
    std::atomic_int cancellationPolls{0};
    const auto cancelled = source->applyPocket(
        operation,
        params,
        0,
        options(false),
        [&cancellationPolls] {
            return cancellationPolls.fetch_add(
                       1, std::memory_order_relaxed) >= 2;
        });
    return expect(cancelled.record.status ==
                      oap::toolpath::StockRemovalStatus::Cancelled &&
                      !cancelled.stock,
                  "a cancelled transaction must not publish a child") &&
           expect(source->volume() == parentVolume &&
                      source->contentFingerprint() == parentFingerprint &&
                      cancelled.record.volumeAfter == parentVolume,
                  "cancellation must preserve the immutable parent");
}

bool tileUpdatesRespectBudgetsAndRollbackFailures()
{
    using namespace oap::toolpath;
    std::string error;
    const auto source = rasterize(
        BRepPrimAPI_MakeBox(64.0, 32.0, 10.0).Shape(), error);
    if (!expect(source && source->valid(), "budget fixture should rasterize"))
        return false;
    const auto operation = pocketCut(2.0, 62.0, 16.0, 4.0);
    const auto params = cutterParams(4.0);
    const std::string original = source->contentFingerprint();
    const double originalVolume = source->volume();
    const auto serial = source->applyPocket(operation, params, 0, options(false));
    if (!expect(serial.stock && serial.touchedTileCount > 2,
                "budget fixture should update several independent tiles"))
        return false;
    const std::string expected = serial.stock->contentFingerprint();
    const auto caller = std::this_thread::get_id();

    // Exercise explicit zero, partial and forced-serial budgets through the
    // real public stock API, including scope propagation into tile callbacks.
    for (const auto [capacity, parallel] :
         {std::pair<std::size_t, bool>{0u, true}, {1u, true}, {1u, false}}) {
        ToolpathParallelBudget budget(capacity);
        std::atomic_size_t acquired{0u}, released{0u}, requests{0u};
        ToolpathGenerationControl control;
        control.tryAcquireParallelWorkers = [&](std::size_t requested) {
            requests.fetch_add(1u);
            const auto count = budget.tryAcquire(requested);
            acquired.fetch_add(count);
            return count;
        };
        control.releaseParallelWorkers = [&](std::size_t count) {
            released.fetch_add(count);
            budget.release(count);
        };
        std::mutex mutex;
        std::set<std::thread::id> callbackThreads;
        bool wrongScope = false;
        ToolpathParallelScope scope(&control);
        const auto result = source->applyPocket(
            operation, params, 0, options(parallel), [&] {
                std::lock_guard<std::mutex> lock(mutex);
                callbackThreads.insert(std::this_thread::get_id());
                wrongScope = wrongScope || currentToolpathParallelControl() != &control;
                return false;
            });
        const bool canBorrow = parallel && toolpathHardwareParallelism() > 1u;
        if (!expect(result.stock && result.stock->contentFingerprint() == expected &&
                        result.changedCellCount == serial.changedCellCount,
                    "zero/partial/serial budgets must preserve complete material truth") ||
            !expect(!wrongScope && acquired == released && budget.available() == capacity &&
                        requests.load() == (canBorrow ? 1u : 0u) &&
                        acquired.load() == (canBorrow ? capacity : 0u),
                    "tile tasks must inherit and return exactly their granted budget") ||
            !expect(callbackThreads.contains(caller) &&
                        (acquired.load() != 0u || callbackThreads.size() == 1u),
                    "zero-worker and forced-serial tile callbacks must stay on their caller"))
            return false;
    }

    // Another CAM job may hold the default pool. A standalone stock update
    // must then complete on its caller, not open a second hardware-sized pool.
    auto& global = globalToolpathParallelBudget();
    const auto availableBefore = global.available();
    {
        ToolpathParallelLease held(nullptr, global.capacity(), global.capacity());
        std::atomic_bool offCaller{false};
        const auto result = source->applyPocket(
            operation, params, 0, options(true), [&] {
                if (std::this_thread::get_id() != caller)
                    offCaller.store(true);
                return false;
            });
        if (!expect(result.stock && result.stock->contentFingerprint() == expected &&
                        !offCaller && global.available() == 0u,
                    "stock updates must fall back to the caller while another job holds the pool"))
            return false;
    }
    if (!expect(global.available() == availableBefore,
                "global contention must leave all borrowed slots available"))
        return false;

    for (const bool throwFromCallback : {false, true}) {
        ToolpathParallelBudget budget(1u);
        std::atomic_size_t acquired{0u}, released{0u};
        ToolpathGenerationControl control;
        control.tryAcquireParallelWorkers = [&](std::size_t requested) {
            const auto count = budget.tryAcquire(requested);
            acquired.fetch_add(count);
            return count;
        };
        control.releaseParallelWorkers = [&](std::size_t count) {
            released.fetch_add(count);
            budget.release(count);
        };
        ToolpathParallelScope scope(&control);
        std::atomic_int polls{0};
        MultiDexelApplyResult result;
        bool caught = false;
        try {
            result = source->applyPocket(
                operation, params, 0, options(true), [&] {
                    // The initial preflight check succeeds. Fail only after
                    // tile execution has started, exercising worker cleanup.
                    if (polls.fetch_add(1) < 2)
                        return false;
                    if (throwFromCallback)
                        throw std::runtime_error("injected tile callback failure");
                    return true;
                });
        } catch (const std::runtime_error& failure) {
            caught = std::string(failure.what()) == "injected tile callback failure";
        }
        if (!expect(caught == throwFromCallback && !result.stock &&
                        (caught || result.record.status == StockRemovalStatus::Cancelled),
                    "a cancelled or failed tile transaction must not publish a child") ||
            !expect(source->contentFingerprint() == original && source->volume() == originalVolume &&
                        acquired == released && budget.available() == 1u,
                    "tile failure must join workers, return its lease and preserve the parent"))
            return false;
    }
    // Failure must not poison a later transaction using the same stock/pool.
    const auto recovered = source->applyPocket(operation, params, 0, options(true));
    return expect(recovered.stock && recovered.stock->contentFingerprint() == expected,
                  "a successful update must remain possible after cancelled/failed updates");
}

TopoDS_Shape separatedLayerTarget()
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    builder.Add(
        compound,
        BRepPrimAPI_MakeBox(
            gp_Pnt(0.0, 0.0, 0.0), 10.0, 10.0, 3.0).Shape());
    builder.Add(
        compound,
        BRepPrimAPI_MakeBox(
            gp_Pnt(0.0, 0.0, 7.0), 10.0, 10.0, 3.0).Shape());
    return compound;
}

std::shared_ptr<oap::toolpath::MultiDexelComparisonData> comparisonData(
    std::shared_ptr<const oap::toolpath::MultiDexelStock> initial,
    std::shared_ptr<const oap::toolpath::MultiDexelStock> final)
{
    auto result =
        std::make_shared<oap::toolpath::MultiDexelComparisonData>();
    result->initial = initial;
    result->final = final;
    oap::toolpath::MultiDexelOperationState operation;
    operation.operationId = "comparison-operation";
    operation.sequenceIndex = 0;
    operation.before = std::move(initial);
    operation.after = std::move(final);
    result->operations.push_back(std::move(operation));
    return result;
}

std::shared_ptr<oap::toolpath::PocketZMapSimulationData> zMapData(
    std::shared_ptr<const oap::toolpath::MultiDexelStock> stock,
    const oap::toolpath::StockFeatureAllowanceScope& scope)
{
    auto result =
        std::make_shared<oap::toolpath::PocketZMapSimulationData>();
    result->gridSource = stock;
    result->origin = stock->origin();
    result->uAxis = stock->uAxis();
    result->vAxis = stock->vAxis();
    result->wAxis = stock->wAxis();
    result->uMin = stock->uMin();
    result->uMax = stock->uMax();
    result->vMin = stock->vMin();
    result->vMax = stock->vMax();
    result->resolution = stock->resolution();
    result->nx = stock->nx();
    result->ny = stock->ny();
    result->initialVolume = stock->volume();
    result->finalVolume = stock->volume();
    const std::size_t count = static_cast<std::size_t>(result->nx) *
        static_cast<std::size_t>(result->ny);
    result->bottoms.resize(count);
    result->initialTops.resize(count);
    result->finalTops.resize(count);
    for (int j = 0; j < result->ny; ++j) {
        for (int i = 0; i < result->nx; ++i) {
            const auto intervals = stock->cellIntervalsView(i, j);
            const std::size_t cell = static_cast<std::size_t>(j) *
                static_cast<std::size_t>(result->nx) +
                static_cast<std::size_t>(i);
            result->bottoms[cell] = intervals.front().lower;
            result->initialTops[cell] = intervals.front().upper;
            result->finalTops[cell] = intervals.front().upper;
        }
    }
    result->featureAllowanceScopes.push_back(scope);
    return result;
}

bool comparisonClassifiesAllowanceAndOvercut()
{
    const TopoDS_Shape initialShape =
        BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    const TopoDS_Shape targetShape = separatedLayerTarget();
    std::string error;
    const auto initial = rasterize(initialShape, error);
    const auto target = rasterize(targetShape, error);
    if (!expect(initial && target && initial->valid() && target->valid() &&
                    target->cellIntervals(5, 5).size() == 2,
                "comparison fixtures should preserve separated layers")) {
        if (!error.empty())
            std::cerr << error << '\n';
        return false;
    }

    const oap::toolpath::MultiDexelComparison comparison;
    const auto exact = comparison.compare(
        comparisonData(initial, target), initialShape, targetShape);
    constexpr double tolerance = 1.0e-6;
    if (!expect(exact.complete && exact.heightFieldComparison &&
                    exact.initialMissingVolume <= tolerance &&
                    exact.remainingAllowanceVolume <= tolerance &&
                    exact.overcutVolume <= tolerance,
                "an exact Multi-Dexel target should stay within all budgets")) {
        return false;
    }

    auto allowanceData = comparisonData(initial, initial);
    oap::toolpath::StockFeatureAllowanceScope featureScope;
    featureScope.featureId = "left-half-feature";
    featureScope.operationId = "comparison-operation";
    featureScope.featureKind = "pocket";
    featureScope.region =
        BRepPrimAPI_MakeBox(5.0, 10.0, 10.0).Shape();
    featureScope.approximateBoundary = true;
    allowanceData->featureAllowanceScopes.push_back(featureScope);
    const auto allowance = comparison.compare(
        allowanceData, initialShape, targetShape);
    if (!expect(allowance.complete &&
                    std::abs(allowance.remainingAllowanceVolume - 400.0) <=
                        tolerance &&
                    allowance.overcutVolume <= tolerance &&
                    allowance.featureAllowances.size() == 1u &&
                    std::abs(allowance.featureAllowances.front()
                                 .targetRemovalVolume - 200.0) <= tolerance &&
                    std::abs(allowance.featureAllowances.front()
                                 .remainingAllowanceVolume - 200.0) <= tolerance &&
                    allowance.featureAllowances.front().completionRatio <=
                        tolerance,
                "retained middle material should be classified globally and per feature")) {
        return false;
    }
    Handle(Poly_Triangulation) multiFeatureMesh;
    oap::toolpath::DexelDisplayMeshMetrics multiFeatureMetrics;
    error.clear();
    if (!expect(
            allowance.heightFieldDisplaySource &&
                allowance.heightFieldDisplaySource->buildDisplayMesh(
                    oap::toolpath::PocketZMapDifferenceView::FeatureAllowance,
                    multiFeatureMesh, &multiFeatureMetrics,
                    featureScope.featureId, &error) &&
                !multiFeatureMesh.IsNull() &&
                std::abs(multiFeatureMetrics.intervalVolume - 200.0) <=
                    tolerance,
            "Multi-Dexel feature allowance should build an isolated display mesh")) {
        if (!error.empty())
            std::cerr << error << '\n';
        return false;
    }

    oap::toolpath::StockComparisonResult brepAllowance =
        oap::toolpath::StockComparison().compare(
            initialShape, initialShape, targetShape);
    oap::toolpath::analyzeFeatureAllowances(
        initialShape, targetShape, {featureScope}, brepAllowance);
    if (!expect(
            brepAllowance.complete &&
                brepAllowance.featureAllowances.size() == 1u &&
                std::abs(brepAllowance.featureAllowances.front()
                             .remainingAllowanceVolume - 200.0) <= tolerance &&
                !brepAllowance.featureAllowances.front()
                     .remainingAllowance.IsNull(),
            "BRep feature allowance should use the same local material semantics")) {
        return false;
    }
    const TopoDS_Shape singleIntervalTarget =
        BRepPrimAPI_MakeBox(10.0, 10.0, 6.0).Shape();
    const auto zMapAllowance =
        oap::toolpath::PocketZMapComparison().compare(
            zMapData(initial, featureScope),
            initialShape, singleIntervalTarget);
    if (!expect(
            zMapAllowance.complete &&
                std::abs(zMapAllowance.remainingAllowanceVolume - 400.0) <=
                    tolerance &&
                zMapAllowance.featureAllowances.size() == 1u &&
                std::abs(zMapAllowance.featureAllowances.front()
                             .remainingAllowanceVolume - 200.0) <= tolerance,
            "single-interval Z-map should publish the same per-feature allowance")) {
        return false;
    }
    Handle(Poly_Triangulation) zMapFeatureMesh;
    oap::toolpath::DexelDisplayMeshMetrics zMapFeatureMetrics;
    error.clear();
    if (!expect(
            zMapAllowance.heightFieldDisplaySource &&
                zMapAllowance.heightFieldDisplaySource->buildDisplayMesh(
                    oap::toolpath::PocketZMapDifferenceView::FeatureAllowance,
                    zMapFeatureMesh, &zMapFeatureMetrics,
                    featureScope.featureId, &error) &&
                !zMapFeatureMesh.IsNull() &&
                std::abs(zMapFeatureMetrics.intervalVolume - 200.0) <=
                    tolerance,
            "Z-map feature allowance should build an isolated display mesh")) {
        if (!error.empty())
            std::cerr << error << '\n';
        return false;
    }

    const auto overcut = comparison.compare(
        comparisonData(initial, target), initialShape, initialShape);
    if (!expect(overcut.complete &&
                    overcut.remainingAllowanceVolume <= tolerance &&
                    std::abs(overcut.overcutVolume - 400.0) <= tolerance,
                "missing target material should be classified as overcut")) {
        return false;
    }

    oap::toolpath::ToolpathGenerationControl control;
    control.isCancellationRequested = [] { return true; };
    const auto cancelled = comparison.compare(
        comparisonData(initial, target),
        initialShape,
        targetShape,
        {},
        &control);
    return expect(cancelled.cancelled && !cancelled.complete,
                  "comparison should honor cooperative cancellation");
}

bool targetCachePreservesAuditAndInvalidatesInputs()
{
    using namespace oap::toolpath;
    const auto initialShape = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    const auto targetShape = BRepPrimAPI_MakeBox(10.0, 10.0, 6.0).Shape();
    auto cache = std::make_shared<StockTargetCache>(targetShape, "model-v1");
    std::array<double, 6> bounds{};
    if (!expect(cache->bounds(bounds), "cache computes exact bounds") ||
        !expect(!cache->matches(targetShape, "model-v2"), "model revision invalidates target cache")) return false;
    gp_Trsf translation;
    translation.SetTranslation(gp_Vec(1, 0, 0));
    if (!expect(!cache->matchesShape(targetShape.Moved(TopLoc_Location(translation))),
                "target location invalidates cache") ||
        !expect(!cache->matchesShape(targetShape.Reversed()), "target orientation invalidates cache")) return false;
    BRep_Builder builder;
    TopoDS_Compound first, second;
    builder.MakeCompound(first); builder.MakeCompound(second);
    builder.Add(first, targetShape); builder.Add(second, targetShape);
    StockTargetCache wrapped(first, "same");
    if (!expect(wrapped.matches(second, "same"), "rebuilt wrappers reuse indexed geometry")) return false;

    const std::array<gp_Vec, 6> axes{gp_Vec(0,0,1), gp_Vec(0,0,-1), gp_Vec(1,0,0),
        gp_Vec(-1,0,0), gp_Vec(0,1,0), gp_Vec(0,-1,0)};
    StockComparisonOptions cachedOptions;
    cachedOptions.targetCache = cache;
    for (const auto& axis : axes) {
        std::string error;
        auto initial = MultiDexelStock::fromShape(initialShape, axis, 2, options(), nullptr, &error);
        if (!expect(initial && initial->valid(), "six-axis cache test grid")) return false;
        const auto key = stockTargetGridKey(*initial, cachedOptions.geometricTolerance);
        if (!expect(!cache->multiDexel(key), "changed axis must miss raster cache")) return false;
        auto data = comparisonData(initial, initial);
        const auto baseline = MultiDexelComparison().compare(data, initialShape, targetShape);
        const auto cold = MultiDexelComparison().compare(data, initialShape, targetShape, cachedOptions);
        const auto entry = cache->multiDexel(key);
        const auto warm = MultiDexelComparison().compare(data, initialShape, targetShape, cachedOptions);
        if (!expect(entry && entry == cache->multiDexel(key), "warm audit reuses target raster") ||
            !expect(baseline.complete && cold.complete && warm.complete &&
                std::abs(baseline.targetVolume - 600.0) < 0.001 &&
                baseline.targetVolume == warm.targetVolume && baseline.overcutVolume == warm.overcutVolume &&
                baseline.remainingAllowanceVolume == warm.remainingAllowanceVolume &&
                cold.initialMissingVolume == warm.initialMissingVolume,
                "six-axis cached native audit preserves quantities")) return false;
        auto changedKey = key; changedKey[16] *= 0.5;
        if (!expect(!cache->multiDexel(changedKey), "resolution invalidates raster")) return false;
        changedKey = key; changedKey[19] *= 2;
        if (!expect(!cache->multiDexel(changedKey), "tolerance invalidates raster")) return false;
        changedKey = key; changedKey[0] += 1;
        if (!expect(!cache->multiDexel(changedKey), "grid origin invalidates raster")) return false;

        const auto heightData = zMapData(initial, {});
        const auto zBaseline = PocketZMapComparison().compare(heightData, initialShape, targetShape);
        const auto zCold = PocketZMapComparison().compare(heightData, initialShape, targetShape, cachedOptions);
        const auto heightKey = stockTargetGridKey(*heightData, cachedOptions.geometricTolerance);
        const auto heightEntry = cache->heightField(heightKey);
        const auto zWarm = PocketZMapComparison().compare(heightData, initialShape, targetShape, cachedOptions);
        if (!expect(heightEntry && heightEntry == cache->heightField(heightKey) && !cache->multiDexel(key),
                "height raster reuses cache and releases previous native raster") ||
            !expect(zBaseline.complete && zCold.complete && zWarm.complete &&
                zBaseline.targetVolume == zWarm.targetVolume &&
                zBaseline.remainingAllowanceVolume == zWarm.remainingAllowanceVolume &&
                zBaseline.overcutVolume == zWarm.overcutVolume, "cached height audit preserves quantities")) return false;
        ToolpathGenerationControl control;
        control.isCancellationRequested = [] { return true; };
        const auto cancelled = PocketZMapComparison().compare(heightData, initialShape, targetShape, cachedOptions, &control);
        if (!expect(cancelled.cancelled && !cancelled.complete, "cache hit respects cancellation")) return false;
        StockTargetCache noBudget(targetShape, "model-v1", {}, 0);
        noBudget.store(heightKey, *heightEntry);
        if (!expect(!noBudget.heightField(heightKey), "oversized target raster is not retained")) return false;
        const auto wrongTarget = PocketZMapComparison().compare(heightData, initialShape, initialShape, cachedOptions);
        if (!expect(wrongTarget.complete && std::abs(wrongTarget.remainingAllowanceVolume) < 1.e-6,
                "foreign target cannot use cached bounds or raster")) return false;
    }
    return true;
}

bool incrementalStatisticsAndLazyFingerprintRemainDeterministic()
{
    using namespace oap::toolpath;
    std::string error;
    auto source = rasterize(BRepPrimAPI_MakeBox(32, 32, 10).Shape(), error);
    if (!source) return false;
    ToolpathParams params;
    params.tool.diameter = 2;
    params.tool.cuttingEdgeLength = 20;
    auto operation = pocketCut(1, 30, 2, 5);
    const auto second = pocketCut(1, 30, 5, 4);
    operation.clearing.moves.push_back(second.clearing.moves.front());
    const auto first = source->applyPocketPrefix(operation, params, 0, 1, options(true));
    if (!first.stock) return false;
    const auto incremental = first.stock->applyPocket(second, params, 1, options(true));
    const auto full = source->applyPocket(operation, params, 0, options(false));
    if (!incremental.stock || !full.stock) return false;
    // Concurrent first readers must publish one complete, original-format digest.
    std::array<std::string, 8> hashes;
    std::vector<std::thread> readers;
    for (std::size_t i = 0; i < hashes.size(); ++i)
        readers.emplace_back([&, i] { hashes[i] = incremental.stock->contentFingerprint(); });
    for (auto& reader : readers) reader.join();
    for (const auto& hash : hashes)
        if (!expect(hash == full.stock->contentFingerprint(), "concurrent lazy digest matches full replay")) return false;
    double volume = 0;
    int active = 0, intervals = 0;
    const auto& stock = *incremental.stock;
    for (int j = 0; j < stock.ny(); ++j) for (int i = 0; i < stock.nx(); ++i) {
        const auto cell = stock.cellIntervalsView(i, j);
        active += !cell.empty(); intervals += static_cast<int>(cell.size());
        const double area = std::min(stock.resolution(), stock.uMax() - stock.uMin() - i * stock.resolution()) *
            std::min(stock.resolution(), stock.vMax() - stock.vMin() - j * stock.resolution());
        for (const auto& interval : cell) volume += area * (interval.upper - interval.lower);
    }
    return expect(std::abs(stock.volume() - volume) < 1.e-7 && active == stock.activeCellCount() &&
        intervals == stock.intervalCount() && stock.volume() == full.stock->volume() &&
        source->volume() > stock.volume() && source->contentFingerprint() != hashes[0],
        "incremental metadata matches full interval scan without mutating parent");
}

} // namespace

int main()
{
    if (!targetCachePreservesAuditAndInvalidatesInputs() ||
        !incrementalStatisticsAndLazyFingerprintRemainDeterministic() ||
        !holeAllowanceRegionCoversConicalBottomAndMargins() ||
        !immutableRemovalIsMonotonicAndDeterministic() ||
        !tileUpdatesRespectBudgetsAndRollbackFailures() ||
        !comparisonClassifiesAllowanceAndOvercut()) {
        return 1;
    }
    std::cout << "Multi-Dexel IPW smoke tests passed\n";
    return 0;
}

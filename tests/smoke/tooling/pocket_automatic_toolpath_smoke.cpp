#include "tooling/adapter/pocket/PocketAutomaticToolpath.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/ToolpathOcctParallel.h"
#include "toolpath/core/validation/ToolpathMoveValidator.h"
#include "toolpath/core/validation/ToolpathPrimitiveValidator.h"
#include "toolpath/pocket/planning/PocketLayerSlicer.h"
#include "toolpath/pocket/simulation/PocketCamSimulationScene.h"

#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace {

constexpr double kTolerance = 1.0e-6;

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

oap::toolpath::PocketRegion lShapedPocket()
{
    oap::toolpath::PocketRegion region;
    region.kind = oap::FeatureKind::Pocket;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0),
        region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.depth = 2.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(0.0, 0.0),
        gp_Pnt2d(50.0, 0.0),
        gp_Pnt2d(50.0, 14.0),
        gp_Pnt2d(14.0, 14.0),
        gp_Pnt2d(14.0, 50.0),
        gp_Pnt2d(0.0, 50.0)};
    region.boundary.signedArea = 1204.0;
    return region;
}

oap::toolpath::PocketRegion narrowPocket()
{
    oap::toolpath::PocketRegion region;
    region.kind = oap::FeatureKind::Slot;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(60.0, 0.0, 0.0),
        region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.depth = 2.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(0.0, 0.0),
        gp_Pnt2d(30.0, 0.0),
        gp_Pnt2d(30.0, 7.0),
        gp_Pnt2d(0.0, 7.0)};
    region.boundary.signedArea = 210.0;
    return region;
}

oap::toolpath::PocketRegion layeredRectangularPocket()
{
    oap::toolpath::PocketRegion region;
    region.kind = oap::FeatureKind::Pocket;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0),
        region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.floorPlane = gp_Pln(gp_Pnt(0.0, 0.0, 0.0), region.openDir);
    region.depth = 4.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(0.0, 0.0),
        gp_Pnt2d(30.0, 0.0),
        gp_Pnt2d(30.0, 20.0),
        gp_Pnt2d(0.0, 20.0)};
    region.boundary.signedArea = 600.0;
    BRep_Builder builder;
    builder.MakeCompound(region.sectionShape);
    builder.Add(
        region.sectionShape,
        BRepPrimAPI_MakeBox(30.0, 20.0, region.depth).Shape());
    return region;
}

oap::toolpath::ToolpathParams roughingParams()
{
    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 4.0;
    params.tool.horizFeed = 600.0;
    params.tool.vertFeed = 180.0;
    params.tool.horizRapid = 1800.0;
    params.tool.vertRapid = 900.0;
    params.depth.safeHeight = 2.0;
    params.depth.clearanceHeight = 4.0;
    params.depth.useFeatureDepth = true;
    params.depth.stepDown = 2.0;
    params.clearing.pattern = oap::toolpath::ClearingPattern::Offset;
    params.clearing.entryMode =
        oap::toolpath::PocketEntryMode::VerticalPlunge;
    params.clearing.stepOverPercent = 50.0;
    return params;
}

bool cachesRegionSectionsWithSingleFlight()
{
    constexpr std::size_t requestCount = 8u;
    const oap::toolpath::PocketRegion region = lShapedPocket();
    oap::toolpath::PocketMaterialSectionCache cache;
    std::vector<oap::toolpath::PocketLayerSlice> slices(requestCount);
    std::vector<std::string> errors(requestCount);
    std::vector<unsigned char> succeeded(requestCount, 0u);
    std::atomic<std::size_t> ready{0u};
    std::atomic_bool start{false};
    std::vector<std::thread> workers;
    workers.reserve(requestCount);
    for (std::size_t index = 0; index < requestCount; ++index) {
        workers.emplace_back([&, index]() {
            ready.fetch_add(1u, std::memory_order_release);
            while (!start.load(std::memory_order_acquire))
                std::this_thread::yield();
            succeeded[index] = cache.sliceRegion(
                region, 0.0, slices[index], errors[index])
                ? 1u
                : 0u;
        });
    }
    while (ready.load(std::memory_order_acquire) < requestCount)
        std::this_thread::yield();
    start.store(true, std::memory_order_release);
    for (std::thread& worker : workers)
        worker.join();

    const bool allSucceeded = std::all_of(
        succeeded.begin(), succeeded.end(),
        [](unsigned char value) { return value != 0u; });
    const bool contoursAgree = std::all_of(
        slices.begin(), slices.end(),
        [&region](const oap::toolpath::PocketLayerSlice& slice) {
            return slice.contours.size() == 1u &&
                   slice.contours.front().points.size() ==
                       region.boundary.points.size();
        });
    return expect(allSucceeded && contoursAgree,
                  "concurrent region-section requests should agree") &&
        expect(cache.missCount() == 1u && cache.entryCount() == 1u,
               "single-flight should compute one region-section entry") &&
        expect(cache.hitCount() == requestCount - 1u,
               "single-flight waiters should be recorded as cache hits");
}

bool parallelizesIndependentSectionWiresDeterministically()
{
    oap::toolpath::PocketRegion region;
    region.kind = oap::FeatureKind::Pocket;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0), region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.floorPlane = gp_Pln(
        gp_Pnt(0.0, 0.0, 0.0), region.openDir);
    region.depth = 4.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(60.0, 0.0),
        gp_Pnt2d(60.0, 4.0), gp_Pnt2d(0.0, 4.0)};
    region.boundary.signedArea = 240.0;
    BRep_Builder builder;
    builder.MakeCompound(region.sectionShape);
    constexpr std::size_t boxCount = 12u;
    for (std::size_t index = 0u; index < boxCount; ++index) {
        builder.Add(
            region.sectionShape,
            BRepPrimAPI_MakeBox(
                gp_Pnt(5.0 * static_cast<double>(index), 0.0, 0.0),
                4.0, 4.0, region.depth).Shape());
    }

    oap::toolpath::ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers =
        [](std::size_t) { return std::size_t{0u}; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    oap::toolpath::PocketLayerSlice serial;
    std::string serialError;
    const bool serialOk = oap::toolpath::PocketLayerSlicer().slice(
        region, 2.0, serial, serialError, nullptr,
        &serialControl);

    std::atomic_size_t acquired{0u};
    std::atomic_size_t released{0u};
    oap::toolpath::ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers =
        [&acquired](std::size_t requested) {
            acquired.fetch_add(requested, std::memory_order_relaxed);
            return requested;
        };
    parallelControl.releaseParallelWorkers =
        [&released](std::size_t count) {
            released.fetch_add(count, std::memory_order_relaxed);
        };
    oap::toolpath::PocketLayerSlice parallel;
    std::string parallelError;
    const bool parallelOk = oap::toolpath::PocketLayerSlicer().slice(
        region, 2.0, parallel, parallelError, nullptr,
        &parallelControl);

    if (!expect(serialOk && parallelOk,
                "serial and parallel multi-wire sections should succeed") ||
        !expect(serial.contours.size() == boxCount &&
                    parallel.contours.size() == boxCount,
                "every independent box should produce one contour") ||
        !expect(serial.wireParallelBatchCount == 0u &&
                    serial.occtInternalParallelCount == 0u &&
                    (parallel.wireParallelBatchCount > 0u) ==
                        (oap::toolpath::toolpathHardwareParallelism() > 1u) &&
                    parallel.wirePeakParallelism ==
                        std::min(
                            boxCount,
                            oap::toolpath::toolpathHardwareParallelism()),
                "wire conversion should use the full shared budget") ||
        !expect(parallel.occtInternalParallelCount ==
                        (oap::toolpath::toolpathOcctParallelism() > 1u &&
                         oap::toolpath::toolpathOcctParallelism() <=
                             oap::toolpath::toolpathHardwareParallelism() ? 1u : 0u) &&
                    parallel.occtSerialFallbackCount == 0u,
                "one large section should use guarded OCCT parallelism") ||
        !expect(acquired.load(std::memory_order_relaxed) ==
                    released.load(std::memory_order_relaxed),
                "section work should return every borrowed token")) {
        if (!serialError.empty())
            std::cerr << serialError << '\n';
        if (!parallelError.empty())
            std::cerr << parallelError << '\n';
        return false;
    }
    for (std::size_t index = 0u;
         index < serial.contours.size(); ++index) {
        const auto& left = serial.contours[index];
        const auto& right = parallel.contours[index];
        if (!expect(
                left.points.size() == right.points.size() &&
                    std::abs(left.signedArea - right.signedArea) <=
                        kTolerance &&
                    left.points.front().Distance(
                        right.points.front()) <= kTolerance,
                "parallel section output should retain stable contour order")) {
            return false;
        }
    }
    return true;
}

void addTool(oap::tooling::ToolLibrary& library,
             int toolNumber,
             const QString& id,
             double diameter,
             oap::tooling::ToolType type = oap::tooling::ToolType::EndMill,
             double cornerRadius = 0.0,
             double cuttingEdgeLength = 20.0,
             double overallLength = 60.0)
{
    oap::tooling::ToolBit tool;
    tool.id = id;
    tool.name = QStringLiteral("T%1 D%2").arg(toolNumber).arg(diameter);
    tool.type = type;
    tool.geometry.diameter = diameter;
    tool.geometry.cornerRadius = cornerRadius;
    tool.geometry.cuttingEdgeLength = cuttingEdgeLength;
    tool.geometry.overallLength = overallLength;
    QString error;
    if (!library.upsertTool(tool, &error)) {
        std::cerr << "FAIL: could not add test tool: "
                  << error.toStdString() << '\n';
        return;
    }

    oap::tooling::LibraryToolSlot slot;
    slot.toolNumber = toolNumber;
    slot.toolId = id;
    slot.cuttingDefaults.spindleSpeed = 8000.0 + toolNumber;
    slot.cuttingDefaults.horizontalFeed = 500.0 + toolNumber;
    slot.cuttingDefaults.verticalFeed = 150.0 + toolNumber;
    if (!library.assignToolToSlot(slot, &error)) {
        std::cerr << "FAIL: could not assign test tool: "
                  << error.toStdString() << '\n';
    }
}

oap::tooling::ToolLibrary testLibrary()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("pocket-test"),
        QStringLiteral("Pocket Test"));
    addTool(library, 1, QStringLiteral("flat-12"), 12.0);
    addTool(library, 2, QStringLiteral("flat-6"), 6.0);
    addTool(library, 3, QStringLiteral("flat-3"), 3.0);
    addTool(
        library, 4, QStringLiteral("drill-2"), 2.0,
        oap::tooling::ToolType::Drill, 0.0);
    return library;
}

bool sameMove(const oap::toolpath::ToolpathMove& a,
              const oap::toolpath::ToolpathMove& b)
{
    return a.type == b.type && a.intent == b.intent &&
        a.loopSequence == b.loopSequence && a.depthIndex == b.depthIndex &&
        a.ring == b.ring && std::abs(a.feed - b.feed) <= kTolerance &&
        a.from.Distance(b.from) <= kTolerance &&
        a.to.Distance(b.to) <= kTolerance;
}

bool samePocketProgram(const oap::tooling::PocketAutomaticToolpathResult& a,
                       const oap::tooling::PocketAutomaticToolpathResult& b)
{
    if (!a.ok() || !b.ok() || a.stages.size() != b.stages.size() ||
        a.batch.operations.size() != b.batch.operations.size() ||
        std::abs(a.selectedCuttingDistance - b.selectedCuttingDistance) > kTolerance)
        return false;
    for (std::size_t i = 0; i < a.stages.size(); ++i) {
        const auto& x = a.stages[i];
        const auto& y = b.stages[i];
        if (x.tool.toolNumber != y.tool.toolNumber || x.operationCount != y.operationCount ||
            std::abs(x.previousToolDiameter - y.previousToolDiameter) > kTolerance ||
            std::abs(x.movementDistance - y.movementDistance) > kTolerance ||
            std::abs(x.cuttingDistance - y.cuttingDistance) > kTolerance)
            return false;
    }
    for (std::size_t i = 0; i < a.batch.operations.size(); ++i) {
        const auto& x = a.batch.operations[i];
        const auto& y = b.batch.operations[i];
        if (x.operationId != y.operationId || x.algorithmId != y.algorithmId ||
            x.tool.toolNumber != y.tool.toolNumber ||
            std::abs(x.tool.diameter - y.tool.diameter) > kTolerance ||
            x.setupIndex != y.setupIndex || x.setupFace != y.setupFace ||
            x.sourceRegionIndex != y.sourceRegionIndex ||
            x.sourceFaceIds != y.sourceFaceIds || x.floorFaceId != y.floorFaceId ||
            (x.outwardToolAxis - y.outwardToolAxis).Magnitude() > kTolerance ||
            std::abs(x.restMachiningPreviousToolDiameter -
                     y.restMachiningPreviousToolDiameter) > kTolerance ||
            !std::equal(x.clearing.moves.begin(), x.clearing.moves.end(),
                        y.clearing.moves.begin(), y.clearing.moves.end(), sameMove) ||
            x.clearing.primitives.size() != y.clearing.primitives.size())
            return false;
        for (std::size_t j = 0; j < x.clearing.primitives.size(); ++j) {
            const auto& p = x.clearing.primitives[j];
            const auto& q = y.clearing.primitives[j];
            if (p.type != q.type || p.context.intent != q.context.intent ||
                p.context.depthIndex != q.context.depthIndex ||
                p.context.loopSequence != q.context.loopSequence ||
                p.context.ring != q.context.ring ||
                !std::equal(p.fallbackMoves.begin(), p.fallbackMoves.end(),
                            q.fallbackMoves.begin(), q.fallbackMoves.end(), sameMove))
                return false;
            if (p.type == oap::toolpath::ToolpathPrimitiveType::Linear) {
                if (!sameMove(p.linear, q.linear))
                    return false;
            } else if (p.type == oap::toolpath::ToolpathPrimitiveType::CircularArc) {
                if (p.arc.moveType != q.arc.moveType || p.arc.direction != q.arc.direction ||
                    p.arc.from.Distance(q.arc.from) > kTolerance ||
                    p.arc.to.Distance(q.arc.to) > kTolerance ||
                    p.arc.center.Distance(q.arc.center) > kTolerance ||
                    (p.arc.axis - q.arc.axis).Magnitude() > kTolerance ||
                    std::abs(p.arc.feed - q.arc.feed) > kTolerance ||
                    std::abs(p.arc.sweepRadians - q.arc.sweepRadians) > kTolerance)
                    return false;
            } else {
                return false; // These pocket fixtures must not emit drill cycles.
            }
        }
    }
    return true;
}

bool respectsCallerBudgetAndReturnsLeases()
{
    const auto library = testLibrary();
    const std::vector<oap::toolpath::PocketRegion> regions{
        lShapedPocket(), narrowPocket(), layeredRectangularPocket()};
    const auto params = roughingParams();
    const auto generate = [&](const oap::toolpath::ToolpathGenerationControl& control) {
        return oap::tooling::PocketAutomaticToolpath().generate(
            library, regions, params, "BUDGET_", &control);
    };
    oap::toolpath::ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers = [](std::size_t) { return 0u; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    const auto caller = std::this_thread::get_id();
    std::atomic_bool usedOtherThread{false};
    serialControl.reportProgress = [&](std::size_t, std::size_t) {
        if (std::this_thread::get_id() != caller)
            usedOtherThread.store(true);
    };
    const auto serial = generate(serialControl);
    if (!expect(serial.ok() && serial.planningPeakParallelism == 1 &&
                    serial.planningFinalParallelism == 1 &&
                    serial.parallelRegionBatchCount == 0 &&
                    serial.parallelTransitionBatchCount == 0 && !usedOtherThread.load(),
                "explicit zero budget must keep actual pocket planning serial"))
        return false;

    oap::toolpath::ToolpathParallelBudget budget(1u);
    std::atomic_size_t acquired{0u}, released{0u};
    oap::toolpath::ToolpathGenerationControl bounded;
    bounded.tryAcquireParallelWorkers = [&](std::size_t requested) {
        const auto granted = budget.tryAcquire(requested);
        acquired.fetch_add(granted);
        return granted;
    };
    bounded.releaseParallelWorkers = [&](std::size_t count) {
        released.fetch_add(count);
        budget.release(count);
    };
    const auto parallel = generate(bounded);
    if (!expect(samePocketProgram(serial, parallel) &&
                    parallel.planningPeakParallelism <= 2 &&
                    parallel.planningFinalParallelism <= 2 &&
                    acquired.load() == released.load() && budget.available() == 1u &&
                    (oap::toolpath::toolpathHardwareParallelism() == 1u || acquired.load() > 0u),
                "partial grants must preserve the ordered program and return every lease"))
        return false;

    std::atomic_bool cancel{false};
    bounded.isCancellationRequested = [&] { return cancel.load(); };
    bounded.reportProgress = [&](std::size_t, std::size_t) { cancel.store(true); };
    const auto cancelled = generate(bounded);
    if (!expect(cancelled.cancelled && !cancelled.ok() && cancelled.batch.operations.empty() &&
                    acquired.load() == released.load() && budget.available() == 1u,
                "cancellation during planning must return the parent lease"))
        return false;
    bounded.isCancellationRequested = {};
    bounded.reportProgress = [](std::size_t, std::size_t) {
        throw std::runtime_error("budget-test failure");
    };
    const auto failed = generate(bounded);
    return expect(!failed.ok() && failed.error.find("budget-test failure") != std::string::npos &&
                      acquired.load() == released.load() && budget.available() == 1u,
                  "worker exceptions must become failures and return the parent lease");
}

bool simultaneousPocketJobsShareGlobalBudget()
{
    auto& budget = oap::toolpath::globalToolpathParallelBudget();
    const auto capacity = budget.capacity();
    if (capacity == 0u) {
        const auto result = oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {lShapedPocket(), narrowPocket()}, roughingParams());
        return expect(result.ok() && result.planningPeakParallelism == 1,
                      "a caller without control must respect the zero global budget");
    }
    std::mutex mutex;
    std::condition_variable ready;
    bool holding = false, resume = false;
    std::size_t granted = 0u;
    oap::toolpath::ToolpathGenerationControl firstControl;
    oap::toolpath::attachGlobalToolpathParallelBudget(firstControl);
    const auto acquire = firstControl.tryAcquireParallelWorkers;
    firstControl.tryAcquireParallelWorkers = [&](std::size_t requested) {
        const auto count = acquire(requested);
        std::unique_lock<std::mutex> lock(mutex);
        granted = count;
        holding = true;
        ready.notify_all();
        ready.wait(lock, [&] { return resume; });
        return count;
    };
    oap::tooling::PocketAutomaticToolpathResult first;
    std::thread worker([&] {
        first = oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {lShapedPocket(), narrowPocket()}, roughingParams(),
            "SHARED_", &firstControl);
    });
    bool acquiredInTime;
    {
        std::unique_lock<std::mutex> lock(mutex);
        acquiredInTime = ready.wait_for(lock, std::chrono::seconds(5), [&] { return holding; });
    }
    // Real second planner, no supplied control: must finish while the first
    // holds the entire shared pool, rather than waiting for additional slots.
    const auto second = oap::tooling::PocketAutomaticToolpath().generate(
        testLibrary(), {lShapedPocket(), narrowPocket()}, roughingParams(), "SHARED_");
    const auto availableWhileHeld = budget.available();
    {
        std::lock_guard<std::mutex> lock(mutex);
        resume = true;
    }
    ready.notify_all();
    worker.join();
    return expect(acquiredInTime && granted == capacity && availableWhileHeld == 0u &&
                      second.planningPeakParallelism == 1 &&
                      samePocketProgram(first, second) && budget.available() == capacity,
                  "concurrent pocket jobs must share global slots with nonblocking serial fallback");
}

bool inheritedBudgetCoversNestedValidation()
{
    using namespace oap::toolpath;
    const auto* original = currentToolpathParallelControl();
    ToolpathParallelBudget budget(2u);
    std::atomic_size_t acquired{0u}, released{0u}, active{0u}, peak{0u};
    std::atomic_bool wrongContext{false};
    ToolpathGenerationControl control;
    control.tryAcquireParallelWorkers = [&](std::size_t count) {
        const auto granted = budget.tryAcquire(count);
        acquired.fetch_add(granted);
        return granted;
    };
    control.releaseParallelWorkers = [&](std::size_t count) {
        released.fetch_add(count);
        budget.release(count);
    };
    {
        ToolpathParallelScope scope(&control);
        toolpathParallelForBudgeted(12u, [&](std::size_t) {
            toolpathParallelForBudgeted(8u, [&](std::size_t) {
                if (currentToolpathParallelControl() != &control) wrongContext.store(true);
                const auto now = active.fetch_add(1u) + 1u;
                auto previous = peak.load();
                while (previous < now && !peak.compare_exchange_weak(previous, now)) {}
                std::this_thread::yield();
                active.fetch_sub(1u);
            });
        });
        try {
            toolpathParallelForBudgeted(8u, [](std::size_t index) {
                if (index == 0) throw std::runtime_error("nested validation test");
            });
            return expect(false, "parallel callback exception must propagate");
        } catch (const std::runtime_error&) { }
        if (!expect(currentToolpathParallelControl() == &control && budget.available() == 2u,
                    "exception must restore the inherited context and every lease")) return false;
    }
    if (!expect(!wrongContext.load() && peak.load() <= 3u &&
                    currentToolpathParallelControl() == original && acquired.load() == released.load(),
                "nested work must inherit one shared budget and restore worker scopes")) return false;

    std::vector<ToolpathMove> moves(9000);
    std::vector<ToolpathPrimitive> primitives;
    for (std::size_t i = 0; i < moves.size(); ++i) {
        moves[i].type = ToolpathMoveType::Cut;
        moves[i].from = gp_Pnt(double(i), 0, 0);
        moves[i].to = gp_Pnt(double(i + 1u), 0, 0);
        moves[i].feed = i % 2048u == 0u ? 0.0 : 100.0;
        primitives.push_back(makeLinearPrimitive(moves[i]));
    }
    // An invalid primitive across a chunk boundary must retain its original index.
    primitives[4096].linear.to = primitives[4096].linear.from;
    const auto validate = [&] {
        ToolpathValidationReport report;
        validateToolpathMoves(moves, "DENSE_", report);
        validateToolpathPrimitives(primitives, "DENSE_", report);
        validateToolpathPrimitiveExpansion(primitives, moves, "DENSE_", report, true);
        return report;
    };
    ToolpathGenerationControl zero;
    zero.tryAcquireParallelWorkers = [](std::size_t) { return 0u; };
    zero.releaseParallelWorkers = [](std::size_t) {};
    ToolpathValidationReport serial, parallel;
    {
        ToolpathParallelScope scope(&zero);
        serial = validate();
        const auto before = acquired.load();
        toolpathParallelForBudgeted(4u, [&](std::size_t) {
            if (currentToolpathParallelControl() != &control) wrongContext.store(true);
        }, &control);
        if (!expect(currentToolpathParallelControl() == &zero &&
                        (toolpathHardwareParallelism() == 1u || acquired.load() > before),
                    "explicit budget must override and then restore the outer zero budget")) return false;
    }
    const auto beforeValidation = acquired.load();
    {
        ToolpathParallelScope scope(&control);
        parallel = validate();
    }
    return expect(!wrongContext.load() && budget.available() == 2u &&
        (toolpathHardwareParallelism() == 1u || acquired.load() > beforeValidation) &&
        serial.errorCount == parallel.errorCount && serial.warningCount == parallel.warningCount &&
        std::equal(serial.issues.begin(), serial.issues.end(), parallel.issues.begin(), parallel.issues.end(),
            [](const auto& a, const auto& b) {
                return a.code == b.code && a.severity == b.severity && a.message == b.message &&
                    a.operationId == b.operationId && a.moveIndex == b.moveIndex;
            }), "dense validation must use the inherited budget and preserve diagnostic order");
}

bool occtInternalParallelismRequiresExclusiveFullBudget()
{
    using namespace oap::toolpath;
    const auto backend = toolpathOcctParallelism();
    const bool supported = backend > 1u && backend <= toolpathHardwareParallelism();
    std::size_t acquired = 0u, released = 0u;
    ToolpathGenerationControl control;
    control.tryAcquireParallelWorkers = [&](std::size_t count) { acquired += count; return count; };
    control.releaseParallelWorkers = [&](std::size_t count) { released += count; };
    const auto before = toolpathOcctParallelStatistics();
    try {
        ToolpathOcctParallelLease lease(true, &control);
        if (!expect(lease.enabled() == supported,
                    "OCCT admission must respect the configured backend size")) return false;
        if (lease.enabled()) {
            ToolpathOcctParallelLease nested(true, &control);
            if (!expect(!nested.enabled(), "nested OCCT admission must fall back without blocking")) return false;
        }
        throw std::runtime_error("occt-lease-test");
    } catch (const std::runtime_error&) { }
    if (!expect(acquired == released, "OCCT exceptions must release the entire budget")) return false;
    if (supported) {
        bool acquireReentryDenied = false, releaseReentryDenied = false;
        control.tryAcquireParallelWorkers = [&](std::size_t count) {
            ToolpathOcctParallelLease reentry(true);
            acquireReentryDenied = !reentry.enabled();
            acquired += count;
            return count;
        };
        control.releaseParallelWorkers = [&](std::size_t count) {
            ToolpathOcctParallelLease reentry(true);
            releaseReentryDenied = !reentry.enabled();
            released += count;
        };
        {
            ToolpathOcctParallelLease lease(true, &control);
            if (!expect(lease.enabled(), "OCCT full grant must remain usable after exception")) return false;
        }
        if (!expect(acquireReentryDenied && releaseReentryDenied && acquired == released,
                    "budget callbacks must not recursively lock the OCCT admission gate")) return false;
        control.tryAcquireParallelWorkers = [](std::size_t) -> std::size_t {
            throw std::runtime_error("occt-acquire-test");
        };
        try {
            ToolpathOcctParallelLease lease(true, &control);
            return expect(false, "OCCT acquire callback exception must propagate");
        } catch (const std::runtime_error&) { }
        control.tryAcquireParallelWorkers = [&](std::size_t count) { acquired += count; return count; };
        {
            ToolpathOcctParallelLease restored(true, &control);
            if (!expect(restored.enabled(), "failed construction must restore OCCT gate and thread scope")) return false;
        }
    }
    control.releaseParallelWorkers = [&](std::size_t count) { released += count; };
    control.tryAcquireParallelWorkers = [&](std::size_t count) {
        const auto partial = count > 0u ? count - 1u : 0u;
        acquired += partial;
        return partial;
    };
    {
        ToolpathOcctParallelLease partial(true, &control);
        if (!expect(!partial.enabled(), "partial OCCT grants must not enable an unbounded backend")) return false;
    }
    control.tryAcquireParallelWorkers = [](std::size_t) { return 0u; };
    {
        ToolpathParallelScope scope(&control);
        ToolpathOcctParallelLease zero(true);
        if (!expect(!zero.enabled(), "OCCT admission must inherit explicit zero budget")) return false;
    }
    const auto after = toolpathOcctParallelStatistics();
    return expect(acquired == released && after.requested > before.requested &&
        (supported ? after.budgetDenied > before.budgetDenied && after.busy > before.busy
                   : after.backendDenied > before.backendDenied),
        "OCCT admission diagnostics must distinguish backend, budget and gate denial");
}

// Reproducible generated STEP corpus, deliberately independent of customer CAD.
// The part STEP contains actual stock with pockets; per-region STEP files retain
// the machining volumes so the benchmark isolates scheduling from recognition.
int runParallelBenchmark(const std::string& sample, const QString& directory)
{
    const bool islands = sample == "islands";
    const bool manyTools = sample == "many_tools";
    const bool deep = sample == "deep";
    const bool multi = sample == "multi";
    const bool stress = sample == "stress";
    if (!islands && !manyTools && !deep && !multi && !stress && sample != "small")
        return 2;
    QDir().mkpath(directory);
    const int count = stress ? 192 : (multi ? 24 : 1);
    const double depth = deep ? 40.0 : (stress ? 20.0 : (multi || islands ? 8.0 : 4.0));
    const double width = islands ? 100.0 : 50.0;
    const double height = islands ? 80.0 : 40.0;
    std::vector<oap::toolpath::PocketRegion> regions;
    BRep_Builder builder;
    TopoDS_Compound parts;
    builder.MakeCompound(parts);
    const QString partPath = directory + "/" + QString::fromStdString(sample) + ".step";
    const bool createPart = !QFileInfo::exists(partPath);
    const auto saveStep = [](const TopoDS_Shape& shape, const QString& path) {
        STEPControl_Writer writer;
        return writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone &&
            writer.Write(path.toUtf8().constData()) == IFSelect_RetDone;
    };
    for (int i = 0; i < count; ++i) {
        auto region = manyTools ? lShapedPocket() : layeredRectangularPocket();
        region.depth = depth;
        const double x = (i % 6) * 70.0;
        const double y = (i / 6) * 60.0;
        region.frame = gp_Ax3(gp_Pnt(x, y, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
        region.floorPlane = gp_Pln(gp_Pnt(x, y, 0), gp_Dir(0, 0, 1));
        if (!manyTools) {
            region.boundary.points = {gp_Pnt2d(0, 0), gp_Pnt2d(width, 0),
                                      gp_Pnt2d(width, height), gp_Pnt2d(0, height)};
            region.boundary.signedArea = width * height;
        }
        const QString regionPath = directory + "/" + QString::fromStdString(sample) +
            "-region-" + QString::number(i) + ".step";
        if (!QFileInfo::exists(regionPath) || createPart) {
            BRepBuilderAPI_MakePolygon polygon;
            for (const auto& p : region.boundary.points)
                polygon.Add(gp_Pnt(x + p.X(), y + p.Y(), 0));
            polygon.Close();
            TopoDS_Shape cavity = BRepPrimAPI_MakePrism(
                BRepBuilderAPI_MakeFace(polygon.Wire()).Face(), gp_Vec(0, 0, depth)).Shape();
            if (islands) {
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 4; ++column) {
                        const double ix = 12.0 + 22.0 * column;
                        const double iy = 12.0 + 22.0 * row;
                        BRepAlgoAPI_Cut cut(cavity, BRepPrimAPI_MakeBox(
                            gp_Pnt(ix, iy, -1), 8, 8, depth + 2).Shape());
                        if (!cut.IsDone()) return 3;
                        cavity = cut.Shape();
                    }
                }
            }
            if (!saveStep(cavity, regionPath)) return 3;
            if (createPart) {
                BRepAlgoAPI_Cut cut(BRepPrimAPI_MakeBox(
                    gp_Pnt(x - 3, y - 3, -3), width + 6, manyTools ? 56 : height + 6,
                    depth + 3).Shape(), cavity);
                if (!cut.IsDone()) return 3;
                builder.Add(parts, cut.Shape());
            }
        }
        STEPControl_Reader reader;
        if (reader.ReadFile(regionPath.toUtf8().constData()) != IFSelect_RetDone ||
            reader.TransferRoots() == 0) return 3;
        builder.MakeCompound(region.sectionShape);
        builder.Add(region.sectionShape, reader.OneShape());
        regions.push_back(std::move(region));
    }
    if (createPart && !saveStep(parts, partPath)) return 3;
    auto library = testLibrary();
    if (manyTools) {
        for (int i = 0; i < 7; ++i)
            addTool(library, 10 + i, QStringLiteral("bench-%1").arg(i), 5.0 + i);
    }
    auto params = roughingParams();
    const auto start = std::chrono::steady_clock::now();
    const auto result = oap::tooling::PocketAutomaticToolpath().generate(
        library, regions, params, "BENCH_");
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    if (!result.ok()) {
        std::cerr << result.error << '\n';
        return 4;
    }
    // Quantized canonical geometry and semantic stream, excluding timings and
    // speculative candidate audits (which legitimately vary with scheduling).
    std::ostringstream canonical;
    canonical << std::fixed << std::setprecision(6);
    std::size_t moves = 0, primitives = 0;
    const auto point = [&](const auto& p) { canonical << p.X() << ',' << p.Y() << ',' << p.Z() << ';'; };
    const auto move = [&](const auto& m) {
        canonical << int(m.type) << ',' << int(m.intent) << ',' << m.loopSequence << ','
                  << m.depthIndex << ',' << m.ring << ',' << m.feed << ';';
        point(m.from); point(m.to);
    };
    for (const auto& operation : result.batch.operations) {
        canonical << operation.operationId << ';' << operation.algorithmId << ';'
                  << oap::toolpath::toolpathToolContextKey(operation.tool) << ';'
                  << operation.setupIndex << ',' << int(operation.setupFace) << ','
                  << operation.sourceRegionIndex << ',' << operation.restMachiningPreviousToolDiameter << ';';
        point(operation.outwardToolAxis);
        for (const auto& m : operation.clearing.moves) move(m);
        moves += operation.clearing.moves.size();
        primitives += operation.clearing.primitives.size();
        for (const auto& p : operation.clearing.primitives) {
            canonical << int(p.type) << ',' << int(p.context.intent) << ',' << p.context.loopSequence << ','
                      << p.context.depthIndex << ',' << p.context.ring << ';';
            if (p.type == oap::toolpath::ToolpathPrimitiveType::Linear) move(p.linear);
            else {
                canonical << int(p.arc.moveType) << ',' << int(p.arc.direction) << ',' << p.arc.feed << ','
                          << p.arc.sweepRadians << ';';
                point(p.arc.from); point(p.arc.to); point(p.arc.center); point(p.arc.axis);
            }
            for (const auto& m : p.fallbackMoves) move(m);
        }
    }
    std::size_t occt = 0, fallback = 0;
    for (const auto& audit : result.stageAudits) {
        occt += audit.occtInternalParallelCount;
        fallback += audit.occtSerialFallbackCount;
    }
    QJsonObject output{{"sample", QString::fromStdString(sample)}, {"milliseconds", milliseconds},
        {"hash", QString::fromLatin1(QCryptographicHash::hash(
            QByteArray::fromStdString(canonical.str()), QCryptographicHash::Sha256).toHex())},
        {"moves", double(moves)}, {"primitives", double(primitives)},
        {"planning_peak", result.planningPeakParallelism}, {"region_peak", result.planningPeakRegionParallelism},
        {"cache_hits", double(result.batch.materialSectionCacheHits)},
        {"cache_misses", double(result.batch.materialSectionCacheMisses)},
        {"evaluated", result.evaluatedTransitionCount}, {"pruned", result.prunedTransitionCount},
        {"occt_parallel", double(occt)}, {"occt_fallback", double(fallback)}};
    std::cout << "BENCHMARK " << QJsonDocument(output).toJson(QJsonDocument::Compact).constData() << '\n';
    return 0;
}

bool reusesRegionSectionsAcrossToolCandidates()
{
    oap::toolpath::ToolpathParams params = roughingParams();
    params.depth.stepDown = 2.0;
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {layeredRectangularPocket()}, params,
            "POCKET_AUTO_SECTION_CACHE_");
    if (!expect(result.ok(),
                "layered pocket selection should succeed with cached sections")) {
        std::cerr << result.error << '\n';
        return false;
    }
    return expect(result.batch.materialSectionCacheMisses > 0u,
                  "layered selection should populate the section cache") &&
        expect(result.batch.materialSectionCacheHits > 0u,
               "tool candidates should reuse cached region sections") &&
        expect(result.batch.materialSectionCacheEntries ==
                   result.batch.materialSectionCacheMisses,
               "single-flight should create one entry per cache miss");
}

bool validatesDeferredCandidateOnDemand()
{
    const oap::toolpath::PocketRegion region = lShapedPocket();
    oap::toolpath::PocketToolpathService service;
    oap::toolpath::PocketToolpathBatch candidate =
        service.generateCandidate(
            {region}, roughingParams(),
            "POCKET_DEFERRED_VALIDATION_");
    if (!expect(
            !candidate.operations.empty() && candidate.validation.ok() &&
                !candidate.safetyValidationPerformed &&
                candidate.safetyValidationMilliseconds == 0.0,
            "candidate generation should defer geometric safety validation")) {
        return false;
    }

    for (auto& move : candidate.operations.front().clearing.moves) {
        if (move.type != oap::toolpath::ToolpathMoveType::Cut)
            continue;
        move.from.Translate(gp_Vec(1000.0, 1000.0, 0.0));
        move.to.Translate(gp_Vec(1000.0, 1000.0, 0.0));
    }
    const bool safetyPassed =
        service.validateCandidateSafety({region}, candidate);
    return expect(
               !safetyPassed && candidate.safetyValidationPerformed,
               "deferred validation should reject an unsafe selected candidate") &&
        expect(
               candidate.validation.errorCount > 0 &&
                   candidate.safetyValidation.errorCount > 0,
               "unsafe deferred candidates should retain a safety error report");
}

bool selectsAValidDescendingFlatEndMillChain()
{
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {lShapedPocket()}, roughingParams(),
            "POCKET_AUTO_TEST_");
    if (!expect(result.ok(), "automatic pocket selection should succeed")) {
        std::cerr << result.error << '\n';
        for (const auto& issue : result.batch.validation.issues)
            std::cerr << issue.code << ": " << issue.message << '\n';
        return false;
    }
    if (!expect(result.candidateToolCount == 3,
                "only assigned flat end mills should be candidates") ||
        !expect(result.evaluatedTransitionCount +
                        result.prunedTransitionCount ==
                    3,
                "every descending transition should be evaluated or safely pruned") ||
        !expect(result.selectedCuttingDistance <=
                    result.baselineSmallToolCuttingDistance + kTolerance,
                "selected chain must not cut farther than the smallest-tool baseline") ||
        !expect(result.stages.size() >= 2,
                "the fixture should select a larger cutter plus a rest-cutting stage")) {
        std::cerr << "selected stages=" << result.stages.size()
                  << ", cutting distance=" << result.selectedCuttingDistance
                  << ", baseline="
                  << result.baselineSmallToolCuttingDistance
                  << '\n';
        return false;
    }

    double previousDiameter = 0.0;
    for (std::size_t index = 0; index < result.stages.size(); ++index) {
        const auto& stage = result.stages[index];
        if (!expect(stage.tool.valid,
                    "every selected stage should retain its tool snapshot") ||
            !expect(stage.tool.toolType == "end_mill",
                    "every selected stage should use a flat end mill") ||
            !expect(stage.operationCount > 0,
                    "reported stages should contain generated operations")) {
            return false;
        }
        if (index > 0 &&
            !expect(stage.tool.diameter < previousDiameter - kTolerance,
                    "selected tool diameters should strictly descend")) {
            return false;
        }
        previousDiameter = stage.tool.diameter;
    }

    bool foundRestOperation = false;
    bool optimizedRestTravel = false;
    double selectedCuttingDistance = 0.0;
    double selectedAllMoveDistance = 0.0;
    for (const auto& operation : result.batch.operations) {
        if (!expect(operation.tool.valid,
                    "each operation should retain the selected library tool") ||
            !expect(operation.sourceRegionIndex == 0,
                    "each operation should retain its source region") ||
            !expect(std::abs(
                        operation.tool.diameter -
                        2.0 * operation.clearing.toolRadius) <= kTolerance,
                    "operation tool and clearing geometry should agree")) {
            return false;
        }
        if (operation.restMachiningPreviousToolDiameter > 0.0 &&
            !expect(operation.restMachiningPreviousToolDiameter >
                        operation.tool.diameter,
                    "rest machining should reference a larger previous cutter")) {
            return false;
        }
        foundRestOperation = foundRestOperation ||
            operation.restMachiningPreviousToolDiameter > 0.0;
        optimizedRestTravel = optimizedRestTravel ||
            operation.clearing.travelReorderedLoopCount > 0u ||
            operation.clearing.travelAdjustedStartCount > 0u;
        for (const auto& move : operation.clearing.moves) {
            const double distance = move.from.Distance(move.to);
            selectedAllMoveDistance += distance;
            if (move.type == oap::toolpath::ToolpathMoveType::Cut)
                selectedCuttingDistance += distance;
        }
    }
    if (!expect(foundRestOperation,
                "the selected chain should contain actual rest machining")) {
        return false;
    }
    if (!expect(
            optimizedRestTravel,
            "fragmented rest machining should optimize loop travel")) {
        return false;
    }
    if (!expect(
            std::abs(
                result.selectedCuttingDistance -
                selectedCuttingDistance) <= kTolerance,
            "automatic selection cost must equal Cut move distance only") ||
        !expect(
            selectedAllMoveDistance >
                selectedCuttingDistance + kTolerance,
            "the fixture must contain excluded rapid, plunge, or retract moves")) {
        return false;
    }

    oap::toolpath::PocketCamSimulationOptions sceneOptions;
    sceneOptions.pathSimplificationTolerance = 0.05;
    const oap::toolpath::PocketCamSimulationSceneResult sceneResult =
        oap::toolpath::PocketCamSimulationSceneBuilder().build(
            BRepPrimAPI_MakeBox(80.0, 80.0, 10.0).Shape(),
            result.batch.operations,
            result.batch.effectiveParams,
            sceneOptions);
    if (!expect(sceneResult.ok(),
                "multi-tool OpenGL scene should be prepared") ||
        !expect(sceneResult.scene.tools.size() >= 2u,
                "multi-tool OpenGL scene should retain distinct tool meshes") ||
        !expect(sceneResult.scene.analyticLinearMotionCount > 0u,
                "flat-end planar G1 moves should enter the analytic cache") ||
        !expect(sceneResult.scene.analyticArcMotionCount > 0u,
                "semantic G2/G3 fallback spans should retain analytic arcs")) {
        std::cerr << sceneResult.error << '\n';
        return false;
    }

    oap::toolpath::PocketToolpathOperation denseOperation =
        result.batch.operations.front();
    denseOperation.clearing.moves.clear();
    denseOperation.clearing.primitives.clear();
    denseOperation.clearing.moves.reserve(5000u);
    const double denseZ = result.batch.operations.front()
                              .clearing.moves.front().from.Z();
    for (std::size_t index = 0u; index < 5000u; ++index) {
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = gp_Pnt(
            0.01 * static_cast<double>(index), 5.0, denseZ);
        move.to = gp_Pnt(
            0.01 * static_cast<double>(index + 1u), 5.0, denseZ);
        move.depthIndex = 0;
        move.loopSequence = 0;
        move.ring = 0;
        denseOperation.clearing.moves.push_back(std::move(move));
    }
    oap::toolpath::PocketCamSimulationOptions denseSceneOptions;
    denseSceneOptions.maxDisplayMotionCount = 1000u;
    denseSceneOptions.allowAutomaticPathSimplification = true;
    const oap::toolpath::PocketCamSimulationSceneResult denseSceneResult =
        oap::toolpath::PocketCamSimulationSceneBuilder().build(
            BRepPrimAPI_MakeBox(80.0, 80.0, 10.0).Shape(),
            {denseOperation}, result.batch.effectiveParams,
            denseSceneOptions);
    if (!expect(
            denseSceneResult.ok() &&
                denseSceneResult.scene.sourceMotionCount == 5000u &&
                denseSceneResult.scene.pathSimplified &&
                denseSceneResult.scene.motions.size() <= 1000u,
            "dense OpenGL paths should simplify before exceeding the display budget")) {
        std::cerr << denseSceneResult.error << '\n';
        return false;
    }

    std::vector<unsigned char> usedVisualTools(
        sceneResult.scene.tools.size(), 0u);
    std::vector<std::size_t> previousSourceMoveEnds(
        result.batch.operations.size(), 0u);
    std::size_t countedAnalyticLines = 0u;
    std::size_t countedAnalyticArcs = 0u;
    std::size_t trustedAnalyticArcs = 0u;
    for (const auto& motion : sceneResult.scene.motions) {
        if (!expect(motion.operationIndex >= 0 &&
                        static_cast<std::size_t>(motion.operationIndex) <
                            result.batch.operations.size(),
                    "OpenGL motion should retain its operation index") ||
            !expect(motion.toolIndex >= 0 &&
                        static_cast<std::size_t>(motion.toolIndex) <
                            sceneResult.scene.tools.size(),
                    "OpenGL motion should retain a valid visual tool index")) {
            return false;
        }
        const auto& operation = result.batch.operations[
            static_cast<std::size_t>(motion.operationIndex)];
        const std::size_t operationIndex =
            static_cast<std::size_t>(motion.operationIndex);
        const auto& visualTool = sceneResult.scene.tools[
            static_cast<std::size_t>(motion.toolIndex)];
        if (!expect(
                std::abs(
                    visualTool.diameter -
                    operation.tool.diameter) <= kTolerance,
                "OpenGL motion tool diameter should match its operation")) {
            return false;
        }
        if (!expect(
                motion.sourceMoveEndCount >
                        previousSourceMoveEnds[operationIndex] &&
                    motion.sourceMoveEndCount <=
                        operation.clearing.moves.size(),
                "OpenGL motion should retain a monotonic source move prefix")) {
            return false;
        }
        previousSourceMoveEnds[operationIndex] =
            motion.sourceMoveEndCount;
        if (motion.analyticSweep.kind ==
            oap::toolpath::PocketCamAnalyticSweepKind::Linear) {
            ++countedAnalyticLines;
        } else if (motion.analyticSweep.kind ==
                   oap::toolpath::PocketCamAnalyticSweepKind::CircularArc) {
            ++countedAnalyticArcs;
            if (!expect(
                    motion.analyticSweep.sweepRadians > 0.0,
                    "analytic arc subspan should retain a positive sweep")) {
                return false;
            }
            if (motion.analyticPlaybackTrusted)
                ++trustedAnalyticArcs;
        }
        usedVisualTools[static_cast<std::size_t>(motion.toolIndex)] = 1u;
    }
    if (!expect(
            countedAnalyticLines ==
                sceneResult.scene.analyticLinearMotionCount,
            "reported analytic G1 count should match mapped motions") ||
        !expect(
            countedAnalyticArcs ==
                sceneResult.scene.analyticArcMotionCount,
            "reported analytic G2/G3 count should match mapped motions") ||
        !expect(
            trustedAnalyticArcs > 0u,
            "validated semantic arcs should retain analytic playback")) {
        return false;
    }
    if (!expect(
            std::count(
                usedVisualTools.begin(), usedVisualTools.end(),
                static_cast<unsigned char>(1u)) >= 2,
            "OpenGL playback should exercise at least two selected tools")) {
        return false;
    }

    // Clipper 或外部刀路可能只提供圆弧的折线 fallback。显示场应从未简化的
    // Cut moves 恢复定半径圆弧，同时保持权威 primitives/NC 数据不变。
    oap::toolpath::PocketToolpathOperation facetedArcOperation =
        result.batch.operations.front();
    facetedArcOperation.operationId = "faceted-arc-display-recovery";
    facetedArcOperation.tool.cornerRadius = 0.0;
    facetedArcOperation.clearing.moves.clear();
    facetedArcOperation.clearing.primitives.clear();
    gp_Vec arcAxis = facetedArcOperation.outwardToolAxis;
    arcAxis.Normalize();
    gp_Vec arcU = std::abs(arcAxis.X()) < 0.8
        ? gp_Vec(1.0, 0.0, 0.0)
        : gp_Vec(0.0, 1.0, 0.0);
    arcU -= arcAxis.Multiplied(arcU.Dot(arcAxis));
    arcU.Normalize();
    gp_Vec arcV = arcAxis.Crossed(arcU);
    arcV.Normalize();
    const gp_Pnt arcCenter(40.0, 40.0, 5.0);
    constexpr std::size_t arcSegmentCount = 18u;
    constexpr double arcRadius = 20.0;
    constexpr double halfPi = 1.57079632679489661923;
    const auto arcPoint = [&](std::size_t index) {
        const double angle = halfPi *
            static_cast<double>(index) /
            static_cast<double>(arcSegmentCount);
        gp_Pnt point = arcCenter;
        point.Translate(
            arcU.Multiplied(arcRadius * std::cos(angle)) +
            arcV.Multiplied(arcRadius * std::sin(angle)));
        return point;
    };
    for (std::size_t index = 0u;
         index < arcSegmentCount; ++index) {
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = arcPoint(index);
        move.to = arcPoint(index + 1u);
        move.loopSequence = 7;
        move.depthIndex = 0;
        move.ring = 0;
        move.feed = 600.0;
        facetedArcOperation.clearing.moves.push_back(move);
        facetedArcOperation.clearing.primitives.push_back(
            oap::toolpath::makeLinearPrimitive(move));
    }
    const auto facetedArcScene =
        oap::toolpath::PocketCamSimulationSceneBuilder().build(
            BRepPrimAPI_MakeBox(80.0, 80.0, 10.0).Shape(),
            {facetedArcOperation}, result.batch.effectiveParams);
    if (!expect(
            facetedArcScene.ok(),
            "faceted arc display recovery scene should build") ||
        !expect(
            facetedArcScene.scene.recoveredAnalyticArcMotionCount ==
                arcSegmentCount,
            "every faceted circular Cut move should recover an analytic arc") ||
        !expect(
            facetedArcScene.scene.analyticFallbackMotionCount == 0u,
            "a recovered faceted circle should not retain planar fallback motions") ||
        !expect(
            std::none_of(
                facetedArcScene.scene.motions.begin(),
                facetedArcScene.scene.motions.end(),
                [](const auto& motion) {
                    return motion.analyticPlaybackTrusted;
                }),
            "display-only recovered arcs must keep authoritative linear playback")) {
        std::cerr << facetedArcScene.error << '\n';
        return false;
    }

    // A primitive can carry stale or externally supplied direction metadata
    // while its controller-independent fallback moves remain authoritative.
    // Never reinterpret a short fallback chord as the complementary ~360
    // degree arc: that would create an OpenGL-only overcut.
    oap::toolpath::PocketToolpathOperation mismatchedArcOperation =
        facetedArcOperation;
    mismatchedArcOperation.operationId =
        "mismatched-arc-direction-display-fallback";
    mismatchedArcOperation.clearing.moves.assign(
        facetedArcOperation.clearing.moves.begin(),
        facetedArcOperation.clearing.moves.begin() + 2);
    oap::toolpath::ToolpathCircularArc mismatchedArc;
    mismatchedArc.moveType = oap::toolpath::ToolpathMoveType::Cut;
    mismatchedArc.direction =
        oap::toolpath::ToolpathArcDirection::Clockwise;
    mismatchedArc.from = arcPoint(0u);
    mismatchedArc.to = arcPoint(2u);
    mismatchedArc.center = arcCenter;
    mismatchedArc.axis = arcAxis;
    mismatchedArc.sweepRadians =
        halfPi * 2.0 / static_cast<double>(arcSegmentCount);
    mismatchedArc.feed = 600.0;
    mismatchedArcOperation.clearing.primitives = {
        oap::toolpath::makeArcPrimitive(
            mismatchedArc,
            mismatchedArcOperation.clearing.moves,
            oap::toolpath::ToolpathPrimitiveContext{
                7, 0, 0,
                oap::toolpath::ToolpathMoveIntent::General})};
    const auto mismatchedArcScene =
        oap::toolpath::PocketCamSimulationSceneBuilder().build(
            BRepPrimAPI_MakeBox(80.0, 80.0, 10.0).Shape(),
            {mismatchedArcOperation}, result.batch.effectiveParams);
    if (!expect(
            mismatchedArcScene.ok(),
            "mismatched arc direction scene should build") ||
        !expect(
            mismatchedArcScene.scene.analyticArcMotionCount == 0u,
            "direction-mismatched fallback chords must remain linear") ||
        !expect(
            std::none_of(
                mismatchedArcScene.scene.motions.begin(),
                mismatchedArcScene.scene.motions.end(),
                [](const auto& motion) {
                    return motion.analyticPlaybackTrusted;
                }),
            "direction-mismatched arcs must not become trusted playback")) {
        std::cerr << mismatchedArcScene.error << '\n';
        return false;
    }

    const oap::toolpath::ToolpathProgram program = result.batch.toProgram();
    if (!expect(program.operations.size() == result.batch.operations.size(),
                "program conversion should retain every selected operation")) {
        return false;
    }
    for (std::size_t index = 0; index < program.operations.size(); ++index) {
        if (!expect(
                program.operations[index].tool.toolNumber ==
                    result.batch.operations[index].tool.toolNumber,
                "program conversion should retain each operation tool")) {
            return false;
        }
    }
    return true;
}

bool validatesOnlySelectedCandidateStages()
{
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {lShapedPocket()}, roughingParams(),
            "POCKET_AUTO_DEFERRED_SAFETY_");
    if (!expect(result.ok(),
                "deferred safety validation should retain a valid plan")) {
        std::cerr << result.error << '\n';
        return false;
    }
    const int auditedSafetyStages = static_cast<int>(std::count_if(
        result.stageAudits.begin(), result.stageAudits.end(),
        [](const oap::tooling::PocketAutomaticToolpathStageAudit& audit) {
            return audit.safetyValidated;
        }));
    double fullGenerationMilliseconds = 0.0;
    double transitionGenerationMilliseconds = 0.0;
    double clearingMilliseconds = 0.0;
    double sectionMilliseconds = 0.0;
    double patternMilliseconds = 0.0;
    double moveAssemblyMilliseconds = 0.0;
    double structuralValidationMilliseconds = 0.0;
    double safetyValidationMilliseconds = 0.0;
    double maximumGenerationMilliseconds = 0.0;
    for (const auto& audit : result.stageAudits) {
        if (audit.previousToolDiameter >
            audit.tool.diameter + kTolerance) {
            transitionGenerationMilliseconds +=
                audit.generationMilliseconds;
        } else {
            fullGenerationMilliseconds += audit.generationMilliseconds;
        }
        clearingMilliseconds += audit.clearingMilliseconds;
        sectionMilliseconds += audit.sectionMilliseconds;
        patternMilliseconds += audit.patternMilliseconds;
        moveAssemblyMilliseconds += audit.moveAssemblyMilliseconds;
        structuralValidationMilliseconds +=
            audit.structuralValidationMilliseconds;
        safetyValidationMilliseconds +=
            audit.safetyValidationMilliseconds;
        maximumGenerationMilliseconds = std::max(
            maximumGenerationMilliseconds,
            audit.generationMilliseconds);
    }
    const auto nearTiming = [](double left, double right) {
        return std::abs(left - right) <= 1.0e-6;
    };
    return expect(
               result.generatedStageCount ==
                   static_cast<int>(result.stageAudits.size()) +
                       result.selectedStageMaterializationCount,
               "candidate evaluations and selected materializations should be counted") &&
        expect(
               result.safetyValidatedStageCount == auditedSafetyStages,
               "safety validation statistics should match stage audits") &&
        expect(
               result.safetyValidatedStageCount ==
                   static_cast<int>(result.stages.size()),
               "only selected non-empty stages should be safety validated") &&
        expect(
               result.safetyValidatedStageCount < result.generatedStageCount,
               "unselected candidate stages should skip safety validation") &&
        expect(
               result.safetyRejectedStageCount == 0 &&
                   result.planningReplanCount == 0 &&
                   result.reusedStageEvaluationCount == 0 &&
                   result.selectedStageMaterializationCount > 0,
               "a safe plan should materialize only its selected chain without replanning") &&
        expect(result.batch.safetyValidationPerformed,
               "the final automatic batch should record completed safety validation") &&
        expect(
               std::all_of(
                   result.stageAudits.begin(), result.stageAudits.end(),
                   [](const auto& audit) {
                       return audit.generationMilliseconds >= 0.0 &&
                           audit.clearingMilliseconds >= 0.0 &&
                           audit.sectionMilliseconds >= 0.0 &&
                           audit.patternMilliseconds >= 0.0 &&
                           audit.moveAssemblyMilliseconds >= 0.0 &&
                           audit.structuralValidationMilliseconds >= 0.0 &&
                           audit.safetyValidationMilliseconds >= 0.0;
                   }) &&
                   maximumGenerationMilliseconds > 0.0,
               "every candidate should retain non-negative phase timing") &&
        expect(
               nearTiming(
                   result.fullStageGenerationMilliseconds,
                   fullGenerationMilliseconds) &&
                   nearTiming(
                       result.transitionStageGenerationMilliseconds,
                       transitionGenerationMilliseconds) &&
                   nearTiming(
                       result.candidateClearingMilliseconds,
                       clearingMilliseconds) &&
                   nearTiming(
                       result.candidateSectionMilliseconds,
                       sectionMilliseconds) &&
                   nearTiming(
                       result.candidatePatternMilliseconds,
                       patternMilliseconds) &&
                   nearTiming(
                       result.candidateMoveAssemblyMilliseconds,
                       moveAssemblyMilliseconds) &&
                   nearTiming(
                       result.candidateStructuralValidationMilliseconds,
                       structuralValidationMilliseconds) &&
                   nearTiming(
                       result.planningSafetyValidationMilliseconds,
                       safetyValidationMilliseconds) &&
                   nearTiming(
                       result.maximumStageGenerationMilliseconds,
                       maximumGenerationMilliseconds),
               "automatic planning timing totals should match stage audits");
}

bool plansEachRegionWithoutCrossRegionToolRejection()
{
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {lShapedPocket(), narrowPocket()},
            roughingParams(), "POCKET_AUTO_MULTI_");
    if (!expect(result.ok(),
                "multi-region automatic selection should succeed")) {
        std::cerr << result.error << '\n';
        return false;
    }
    const bool wideUsesLarge = std::any_of(
        result.batch.operations.begin(), result.batch.operations.end(),
        [](const oap::toolpath::PocketToolpathOperation& operation) {
            return operation.sourceRegionIndex == 0 &&
                   std::abs(operation.tool.diameter - 12.0) <= kTolerance;
        });
    const bool narrowAvoidsLarge = std::none_of(
        result.batch.operations.begin(), result.batch.operations.end(),
        [](const oap::toolpath::PocketToolpathOperation& operation) {
            return operation.sourceRegionIndex == 1 &&
                   (std::abs(operation.tool.diameter - 12.0) <= kTolerance ||
                    std::abs(
                        operation.restMachiningPreviousToolDiameter - 12.0) <=
                        kTolerance);
        });
    const bool narrowUsesSmallerTool = std::any_of(
        result.batch.operations.begin(), result.batch.operations.end(),
        [](const oap::toolpath::PocketToolpathOperation& operation) {
            return operation.sourceRegionIndex == 1 &&
                   operation.tool.diameter <= 6.0 + kTolerance;
        });
    const auto acceptedLargeForWideRegion = std::find_if(
        result.stageAudits.begin(), result.stageAudits.end(),
        [](const oap::tooling::PocketAutomaticToolpathStageAudit& audit) {
            return std::abs(audit.tool.diameter - 12.0) <= kTolerance &&
                   audit.previousToolDiameter <= kTolerance &&
                   audit.regionIndices.size() == 1u &&
                   audit.regionIndices.front() == 0;
        });
    const auto rejectedLargeForNarrowRegion = std::find_if(
        result.stageAudits.begin(), result.stageAudits.end(),
        [](const oap::tooling::PocketAutomaticToolpathStageAudit& audit) {
            return std::abs(audit.tool.diameter - 12.0) <= kTolerance &&
                   audit.previousToolDiameter <= kTolerance &&
                   audit.regionIndices.size() == 1u &&
                   audit.regionIndices.front() == 1;
        });
    return expect(
               wideUsesLarge,
               "a large cutter should remain selected for the wide region") &&
        expect(
               narrowAvoidsLarge && narrowUsesSmallerTool,
               "the narrow region should use only its successful smaller-tool chain") &&
        expect(
               acceptedLargeForWideRegion != result.stageAudits.end() &&
                   acceptedLargeForWideRegion->usable,
               "the wide region should retain its successful large-tool audit") &&
        expect(
               rejectedLargeForNarrowRegion != result.stageAudits.end() &&
                   !rejectedLargeForNarrowRegion->usable &&
               rejectedLargeForNarrowRegion->failedRegionCount == 1 &&
                   !rejectedLargeForNarrowRegion->failureDetails.empty(),
               "the narrow region should retain its independent large-tool failure audit") &&
        expect(result.planningParallelism <= 1 ||
                   result.parallelRegionBatchCount > 0,
               "independent regions should use bounded parallel planning") &&
        expect(result.planningPeakParallelism >= 1 &&
                   result.planningPeakParallelism <=
                       result.planningCpuParallelism &&
                   result.planningPeakRegionParallelism >= 1 &&
                   result.planningPeakRegionParallelism <=
                       result.planningPeakParallelism &&
                   result.planningFinalParallelism >= 1 &&
                   result.planningFinalParallelism <=
                       result.planningCpuParallelism,
               "the dynamic region queue should stay within its CPU budget");
}

bool reportsTheSmallestToolPlanningFailure()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("oversize-pocket-test"),
        QStringLiteral("Oversize Pocket Test"));
    addTool(library, 1, QStringLiteral("flat-12-only"), 12.0);
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            library, {narrowPocket()}, roughingParams(),
            "POCKET_AUTO_OVERSIZE_");
    return expect(!result.ok() && result.stageAudits.size() == 1,
                  "an oversized smallest tool must fail with one stage audit") &&
        expect(result.error.find("T1") != std::string::npos &&
                   result.error.find("D12.000") != std::string::npos &&
                   result.error.find("region 1") != std::string::npos,
               "the final planning failure must identify the tool and failed region") &&
        expect(result.error.find("NoClearableArea") != std::string::npos ||
                   result.error.find("ToolDoesNotFit") != std::string::npos,
               "the final planning failure must retain the clearing failure code");
}

bool reportsWhyEveryAssignedFlatEndMillWasRejected()
{
    oap::toolpath::PocketRegion deep = lShapedPocket();
    deep.depth = 80.0;
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {deep}, roughingParams(),
            "POCKET_AUTO_DEEP_");
    return expect(!result.ok() && result.candidateToolCount == 0,
                  "an unreachable deep pocket should reject every candidate") &&
        expect(result.error.find("最大单层切深 2.000 mm") !=
                       std::string::npos &&
                   result.error.find("最深刀具可达距离 80.000 mm") !=
                       std::string::npos,
               "the rejection should distinguish layer engagement from reach") &&
        expect(result.error.find("铣刀 3 把") != std::string::npos &&
                   result.error.find("刃长不足 0 把") != std::string::npos &&
                   result.error.find("总长不足 3 把") != std::string::npos,
               "the rejection should report each filtering reason");
}

bool acceptsShorterFlutesForLayeredRoughing()
{
    oap::toolpath::PocketRegion deep = lShapedPocket();
    deep.depth = 25.0;
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            testLibrary(), {deep}, roughingParams(),
            "POCKET_AUTO_LAYERED_DEEP_");
    if (!expect(result.ok(),
                "layered roughing should not require flute length to equal total depth")) {
        std::cerr << result.error << '\n';
        return false;
    }
    return expect(result.candidateToolCount == 3,
                  "all long-reach flat end mills should remain candidates");
}

bool plansDeepAndNarrowRegionsWithDifferentCompatibleTools()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("mixed-reach-pocket-test"),
        QStringLiteral("Mixed Reach Pocket Test"));
    addTool(
        library, 1, QStringLiteral("long-flat-10"), 10.0,
        oap::tooling::ToolType::EndMill, 0.0, 20.0, 70.0);
    addTool(
        library, 2, QStringLiteral("short-flat-4"), 4.0,
        oap::tooling::ToolType::EndMill, 0.0, 20.0, 50.0);
    addTool(
        library, 3, QStringLiteral("short-flat-2"), 2.0,
        oap::tooling::ToolType::EndMill, 0.0, 6.0, 50.0);

    oap::toolpath::PocketRegion deep = lShapedPocket();
    deep.depth = 55.0;
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            library, {deep, narrowPocket()}, roughingParams(),
            "POCKET_AUTO_MIXED_REACH_");
    if (!expect(
            result.ok(),
            "deep and narrow regions should use separate compatible tool groups")) {
        std::cerr << result.error << '\n';
        return false;
    }

    bool deepUsesLongTool = false;
    bool narrowUsesSmallTool = false;
    for (const auto& operation : result.batch.operations) {
        deepUsesLongTool = deepUsesLongTool ||
            (operation.sourceRegionIndex == 0 &&
             std::abs(operation.tool.diameter - 10.0) <= kTolerance);
        narrowUsesSmallTool = narrowUsesSmallTool ||
            (operation.sourceRegionIndex == 1 &&
             operation.tool.diameter <= 4.0 + kTolerance);
    }
    const bool auditsNarrowGroup = std::any_of(
        result.stageAudits.begin(), result.stageAudits.end(),
        [](const oap::tooling::PocketAutomaticToolpathStageAudit& audit) {
            return audit.regionIndices.size() == 1u &&
                   audit.regionIndices.front() == 1;
        });
    return expect(
               result.candidateToolCount == 3,
               "a tool usable by any region should remain a candidate") &&
        expect(
               deepUsesLongTool,
               "the deep region should retain the long-reach cutter") &&
        expect(
               narrowUsesSmallTool,
               "the shallow narrow region should use its shorter small cutter") &&
        expect(
               auditsNarrowGroup,
               "stage audits should identify the original grouped regions") &&
        expect(
               result.selectedCuttingDistance <=
                   result.baselineSmallToolCuttingDistance + kTolerance,
               "grouped planning should preserve each group's shortest-path bound");
}

bool evaluatesIndependentTransitionsInParallel()
{
    oap::tooling::ToolLibrary library = testLibrary();
    addTool(library, 5, QStringLiteral("flat-10"), 10.0);
    addTool(library, 6, QStringLiteral("flat-8"), 8.0);
    const oap::tooling::PocketAutomaticToolpathResult result =
        oap::tooling::PocketAutomaticToolpath().generate(
            library, {lShapedPocket()}, roughingParams(),
            "POCKET_AUTO_PARALLEL_");
    if (!expect(result.ok(),
                "parallel transition evaluation should retain a valid result")) {
        std::cerr << result.error << '\n';
        return false;
    }
    const int maximumHardwareParallelism = static_cast<int>(
        oap::toolpath::toolpathHardwareParallelism());
    return expect(
               result.planningCpuParallelism == maximumHardwareParallelism,
               "automatic planning should report its hardware budget") &&
        expect(
               result.planningParallelism >= 1 &&
                   result.planningParallelism <=
                       result.planningCpuParallelism &&
                   result.planningParallelism <=
                       result.planningMemoryParallelism,
               "automatic planning should obey CPU and memory budgets") &&
        expect(
               result.evaluatedTransitionCount +
                       result.prunedTransitionCount ==
                   10,
               "five cutters should account for every descending transition") &&
        expect(result.planningParallelism <= 1 ||
                   result.parallelTransitionBatchCount > 0,
               "independent remaining transitions should use a parallel batch") &&
        expect(result.planningParallelism <= 1 ||
                   result.planningBorrowedTokenCount > 0,
               "parallel candidate work should borrow from the shared token pool") &&
        expect(result.planningParallelism <= 1 ||
                   (result.parallelFullStageBatchCount > 0 &&
                    result.prefetchedFullStageCount > 0),
               "independent full stages should use bounded prefetch") &&
        expect(result.selectedCuttingDistance <=
                   result.baselineSmallToolCuttingDistance + kTolerance,
               "parallel evaluation must preserve the shortest-path bound");
}


bool acceptsRoundedMillingTools()
{
    for (const auto type : {oap::tooling::ToolType::BallEndMill, oap::tooling::ToolType::BullNose}) {
        oap::tooling::ToolLibrary library;
        const double radius = type == oap::tooling::ToolType::BallEndMill ? 2.0 : 0.5;
        // Ball libraries may leave the derived radius at zero.
        addTool(library, 1, QStringLiteral("rounded"), 4.0, type,
            type == oap::tooling::ToolType::BallEndMill ? 0.0 : radius);
        auto result = oap::tooling::PocketAutomaticToolpath().generate(
            library, {lShapedPocket()}, roughingParams(), "ROUNDED_");
        if (!expect(result.ok() && result.candidateToolCount == 1,
                "rounded milling tool should generate: " + result.error)) return false;
        for (const auto& operation : result.batch.operations)
            if (!expect(operation.tool.cornerRadius == radius &&
                        operation.tool.toolType == oap::tooling::toolTypeId(type).toStdString(),
                    "rounded cutter geometry was flattened")) return false;
        if (!expect(std::any_of(result.batch.diagnostics.begin(), result.batch.diagnostics.end(),
                [](const auto& diagnostic) { return diagnostic.code ==
                    oap::toolpath::ToolpathDiagnosticCode::RoundedCutterResidual; }),
                "rounded cutter must report possible residual material")) return false;
        // A same-diameter flat tool has fuller floor coverage even with a larger T number.
        addTool(library, 2, QStringLiteral("flat"), 4.0);
        result = oap::tooling::PocketAutomaticToolpath().generate(
            library, {lShapedPocket()}, roughingParams(), "SAME_DIAMETER_");
        if (!expect(result.ok() && result.batch.operations.front().tool.toolNumber == 2,
                "same-diameter preference should preserve fuller floor coverage")) return false;
        oap::tooling::ToolLibrary roundedChain;
        addTool(roundedChain, 1, QStringLiteral("large-rounded"), 12.0, type,
            type == oap::tooling::ToolType::BallEndMill ? 6.0 : 1.0);
        addTool(roundedChain, 2, QStringLiteral("small-rounded"), 4.0, type, radius);
        const auto chain = oap::tooling::PocketAutomaticToolpath().generate(
            roundedChain, {lShapedPocket()}, roughingParams(), "ROUNDED_CHAIN_");
        if (!expect(chain.ok(), "rounded chain failed: " + chain.error)) return false;
        for (const auto& audit : chain.stageAudits)
            if (!expect(audit.previousToolDiameter == 0.0,
                    "rounded predecessors must not claim flat-cylinder rest coverage")) return false;
    }
    oap::tooling::ToolLibrary mixedReach;
    addTool(mixedReach, 1, QStringLiteral("short-flat"), 4.0,
        oap::tooling::ToolType::EndMill, 0.0, 3.0, 3.0);
    addTool(mixedReach, 2, QStringLiteral("long-ball"), 4.0,
        oap::tooling::ToolType::BallEndMill, 2.0, 20.0, 60.0);
    auto deep = narrowPocket();
    deep.depth = 6.0;
    const auto mixed = oap::tooling::PocketAutomaticToolpath().generate(
        mixedReach, {lShapedPocket(), deep}, roughingParams(), "ROUNDED_REACH_");
    if (!expect(mixed.ok(), "same-diameter rounded reach candidate was discarded: " + mixed.error)) return false;
    for (const auto& operation : mixed.batch.operations)
        if (!expect(operation.tool.toolNumber == (operation.sourceRegionIndex == 0 ? 1 : 2),
                "same-diameter preference must be applied after region reach filtering")) return false;
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc == 3 && std::string_view(argv[1]) == "--verify-benchmark-step") {
        for (const auto* sample : {"small", "multi", "deep", "islands", "many_tools", "stress"}) {
            const QString path = QString::fromUtf8(argv[2]) + "/" + sample + ".step";
            STEPControl_Reader reader;
            if (reader.ReadFile(path.toUtf8().constData()) != IFSelect_RetDone ||
                reader.TransferRoots() == 0 ||
                !BRepCheck_Analyzer(reader.OneShape(), Standard_True, Standard_False).IsValid()) return 3;
            int solids = 0, faces = 0;
            for (TopExp_Explorer it(reader.OneShape(), TopAbs_SOLID); it.More(); it.Next()) ++solids;
            for (TopExp_Explorer it(reader.OneShape(), TopAbs_FACE); it.More(); it.Next()) ++faces;
            std::cout << "STEP_VALID " << sample << " solids=" << solids << " faces=" << faces << '\n';
        }
        return 0;
    }
    if (argc == 4 && std::string_view(argv[1]) == "--parallel-benchmark")
        return runParallelBenchmark(argv[2], QString::fromUtf8(argv[3]));
    if (argc == 2 && std::string_view(argv[1]) == "--parallel-budget") {
        return inheritedBudgetCoversNestedValidation() &&
                occtInternalParallelismRequiresExclusiveFullBudget() &&
                parallelizesIndependentSectionWiresDeterministically() &&
                respectsCallerBudgetAndReturnsLeases() &&
                simultaneousPocketJobsShareGlobalBudget() ? 0 : 1;
    }
    if (!acceptsRoundedMillingTools() ||
        !cachesRegionSectionsWithSingleFlight() ||
        !parallelizesIndependentSectionWiresDeterministically() ||
        !reusesRegionSectionsAcrossToolCandidates() ||
        !validatesDeferredCandidateOnDemand() ||
        !selectsAValidDescendingFlatEndMillChain() ||
        !validatesOnlySelectedCandidateStages() ||
        !plansEachRegionWithoutCrossRegionToolRejection() ||
        !reportsTheSmallestToolPlanningFailure() ||
        !acceptsShorterFlutesForLayeredRoughing() ||
        !plansDeepAndNarrowRegionsWithDifferentCompatibleTools() ||
        !evaluatesIndependentTransitionsInParallel() ||
        !reportsWhyEveryAssignedFlatEndMillWasRejected()) {
        return 1;
    }
    std::cout << "PASS: automatic pocket tool selection checks\n";
    return 0;
}

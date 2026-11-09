#include "tooling/adapter/profile/ProfileAutomaticToolpath.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/pocket/simulation/PocketCamSimulationScene.h"
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"

#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

constexpr double kTolerance = 1.0e-6;

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool sharesProcessWideParallelBudget()
{
    auto& budget = oap::toolpath::globalToolpathParallelBudget();
    if (budget.capacity() == 0u)
        return true;
    oap::toolpath::ToolpathGenerationControl first;
    oap::toolpath::ToolpathGenerationControl second;
    oap::toolpath::attachGlobalToolpathParallelBudget(first);
    oap::toolpath::attachGlobalToolpathParallelBudget(second);
    const std::size_t firstGrant =
        first.tryAcquireParallelWorkers(budget.capacity());
    const std::size_t secondGrant =
        second.tryAcquireParallelWorkers(budget.capacity());
    if (secondGrant > 0u)
        second.releaseParallelWorkers(secondGrant);
    first.releaseParallelWorkers(firstGrant);
    return expect(
        firstGrant == budget.capacity() && secondGrant == 0u &&
            budget.available() == budget.capacity(),
        "all CAM controls should lease additional workers from one process-wide budget");
}

oap::toolpath::ProfileRegion lShapedExternalProfile()
{
    oap::toolpath::ProfileRegion region;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0), region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.depth = 2.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(20.0, 20.0), gp_Pnt2d(80.0, 20.0),
        gp_Pnt2d(80.0, 40.0), gp_Pnt2d(40.0, 40.0),
        gp_Pnt2d(40.0, 80.0), gp_Pnt2d(20.0, 80.0)};
    region.boundary.signedArea = 2000.0;
    region.stockBoundary.closed = true;
    region.stockBoundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(100.0, 0.0),
        gp_Pnt2d(100.0, 100.0), gp_Pnt2d(0.0, 100.0)};
    region.stockBoundary.signedArea = 10000.0;
    region.source.rootFaceId = "profile-root";
    region.source.rootWireId = "profile-outer-wire";
    return region;
}

void addTool(
    oap::tooling::ToolLibrary& library,
    int toolNumber,
    const QString& id,
    double diameter,
    oap::tooling::ToolType type = oap::tooling::ToolType::EndMill,
    double cornerRadius = 0.0,
    double cuttingLength = 20.0,
    double overallLength = 60.0)
{
    oap::tooling::ToolBit tool;
    tool.id = id;
    tool.name = QStringLiteral("T%1 D%2")
        .arg(toolNumber).arg(diameter);
    tool.type = type;
    tool.geometry.diameter = diameter;
    tool.geometry.cornerRadius = cornerRadius;
    tool.geometry.cuttingEdgeLength = cuttingLength;
    tool.geometry.overallLength = overallLength;
    QString error;
    if (!library.upsertTool(tool, &error)) {
        std::cerr << "FAIL: could not add profile test tool: "
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
        std::cerr << "FAIL: could not assign profile test tool: "
                  << error.toStdString() << '\n';
    }
}

oap::toolpath::ToolpathParams externalClearingParams()
{
    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 4.0;
    params.tool.horizFeed = 500.0;
    params.tool.vertFeed = 150.0;
    params.tool.horizRapid = 1800.0;
    params.tool.vertRapid = 900.0;
    params.depth.useFeatureDepth = true;
    params.depth.stepDown = 2.0;
    params.depth.safeHeight = 3.0;
    params.depth.clearanceHeight = 5.0;
    params.profile.machiningMode =
        oap::toolpath::ProfileMachiningMode::ExternalClearing;
    params.profile.roughingAllowance = 0.2;
    params.profile.sideAllowance = 0.0;
    params.profile.finishPass = true;
    params.clearing.pattern = oap::toolpath::ClearingPattern::Offset;
    params.clearing.stepOverPercent = 50.0;
    return params;
}

bool sameAutomaticProgram(
    const oap::tooling::ProfileAutomaticToolpathResult& left,
    const oap::tooling::ProfileAutomaticToolpathResult& right,
    bool compareAudits = true)
{
    if (left.stages.size() != right.stages.size() ||
        (compareAudits &&
         left.stageAudits.size() != right.stageAudits.size()) ||
        left.batch.operations.size() != right.batch.operations.size()) {
        return false;
    }
    for (std::size_t index = 0u; index < left.stages.size(); ++index) {
        const auto& a = left.stages[index];
        const auto& b = right.stages[index];
        if (a.tool.toolNumber != b.tool.toolNumber ||
            std::abs(
                a.previousToolDiameter - b.previousToolDiameter) >
                kTolerance ||
            a.finishPass != b.finishPass ||
            a.operationCount != b.operationCount ||
            std::abs(a.cuttingDistance - b.cuttingDistance) >
                kTolerance ||
            std::abs(
                a.mandatoryCuttingDistanceLowerBound -
                b.mandatoryCuttingDistanceLowerBound) > kTolerance) {
            return false;
        }
    }
    if (compareAudits) {
        for (std::size_t index = 0u;
             index < left.stageAudits.size(); ++index) {
            const auto& a = left.stageAudits[index];
            const auto& b = right.stageAudits[index];
            if (a.stage.tool.toolNumber != b.stage.tool.toolNumber ||
                std::abs(
                    a.stage.previousToolDiameter -
                    b.stage.previousToolDiameter) > kTolerance ||
                a.evaluationOnly != b.evaluationOnly ||
                a.usable != b.usable ||
                a.failedRegionCount != b.failedRegionCount ||
                a.validationErrorCount != b.validationErrorCount ||
                a.failureDetails != b.failureDetails) {
                return false;
            }
        }
    }
    for (std::size_t operationIndex = 0u;
         operationIndex < left.batch.operations.size();
         ++operationIndex) {
        const auto& a = left.batch.operations[operationIndex];
        const auto& b = right.batch.operations[operationIndex];
        if (a.tool.toolNumber != b.tool.toolNumber ||
            a.sourceRegionIndex != b.sourceRegionIndex ||
            a.contour.moves.size() != b.contour.moves.size() ||
            a.contour.loops.size() != b.contour.loops.size()) {
            return false;
        }
        for (std::size_t moveIndex = 0u;
             moveIndex < a.contour.moves.size(); ++moveIndex) {
            const auto& moveA = a.contour.moves[moveIndex];
            const auto& moveB = b.contour.moves[moveIndex];
            if (moveA.type != moveB.type ||
                moveA.loopSequence != moveB.loopSequence ||
                moveA.depthIndex != moveB.depthIndex ||
                moveA.ring != moveB.ring ||
                moveA.from.Distance(moveB.from) > kTolerance ||
                moveA.to.Distance(moveB.to) > kTolerance) {
                return false;
            }
        }
    }
    return true;
}

bool selectsLargeToolThenSmallRestTool()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("profile-auto-test"),
        QStringLiteral("Profile Automatic Test"));
    addTool(library, 1, QStringLiteral("flat-20"), 20.0);
    addTool(library, 2, QStringLiteral("flat-4"), 4.0);
    // 钻头不能进入外轮廓铣削候选集。
    addTool(
        library, 3, QStringLiteral("drill-2"), 2.0,
        oap::tooling::ToolType::Drill, 0.0);

    const auto result = oap::tooling::ProfileAutomaticToolpath().generate(
        library, {lShapedExternalProfile()},
        externalClearingParams(), "PROFILE_AUTO_TEST_");
    if (!expect(result.ok(),
                "automatic profile tool selection should succeed")) {
        std::cerr << result.error << '\n';
        for (const auto& audit : result.stageAudits) {
            std::cerr << "audit T" << audit.stage.tool.toolNumber
                      << " D" << audit.stage.tool.diameter
                      << " previous=" << audit.stage.previousToolDiameter
                      << " cut=" << audit.stage.cuttingDistance
                      << " operations=" << audit.stage.operationCount
                      << " usable=" << audit.usable << '\n';
        }
        return false;
    }
    if (!expect(result.candidateToolCount == 2,
                "only assigned reachable milling cutters should be candidates") ||
        !expect(result.stages.size() >= 2,
                "the concave exterior should select a multi-tool chain") ||
        !expect(result.stages.front().tool.diameter >
                    result.stages.back().tool.diameter,
                "selected profile stages should descend by diameter") ||
        !expect(!result.stages.front().finishPass &&
                    result.stages.back().finishPass,
                "only the final small-tool stage should finish the wall") ||
        !expect(result.evaluatedStageCount >
                    result.materializedStageCount &&
                    result.materializedStageCount ==
                        static_cast<int>(result.stages.size()) &&
                    result.promotedStageCount ==
                        result.materializedStageCount &&
                    result.cachedEvaluationStageCount >
                        result.promotedStageCount &&
                    result.releasedEvaluationStageCount ==
                        result.cachedEvaluationStageCount &&
                    result.peakRetainedEvaluationLoopCount > 0u &&
                    result.rejectedTransitionCount == 0 &&
                    result.parallelEvaluationBatchCount == 0 &&
                    result.evaluationPeakParallelism == 1u,
                "automatic profile planning should materialize only the selected validated chain")) {
        return false;
    }

    bool foundFullLargeTool = false;
    bool foundSmallToolRest = false;
    for (const auto& operation : result.batch.operations) {
        if (!expect(operation.tool.valid,
                    "automatic profile operations should retain library tool identity") ||
            !expect(std::abs(
                        operation.tool.diameter -
                        2.0 * operation.contour.toolRadius) <= kTolerance,
                    "profile operation tool and generated geometry should agree")) {
            return false;
        }
        if (operation.restMachiningPreviousToolDiameter <= 0.0) {
            foundFullLargeTool = foundFullLargeTool ||
                operation.tool.diameter > 4.0 + kTolerance;
        } else {
            foundSmallToolRest = foundSmallToolRest ||
                operation.contour.restMachining &&
                operation.restMachiningPreviousToolDiameter >
                    operation.tool.diameter + kTolerance;
        }
    }
    if (!expect(foundFullLargeTool && foundSmallToolRest,
                "the final batch should contain both full clearing and true rest machining") ||
        !expect(result.batch.algorithmId == "profile.auto_tool_library" &&
                    result.validation.ok(),
                "the selected multi-tool profile program should retain its algorithm and validation evidence")) {
        return false;
    }

    const std::vector<oap::toolpath::PocketToolpathOperation>
        simulationOperations =
            oap::toolpath::legacyProfileSimulationOperations(
                result.batch.operations);
    const auto scene =
        oap::toolpath::PocketCamSimulationSceneBuilder().build(
            BRepPrimAPI_MakeBox(100.0, 100.0, 5.0).Shape(),
            simulationOperations, result.batch.effectiveParams);
    if (!expect(scene.ok(),
                "multi-tool profile OpenGL scene should be prepared") ||
        !expect(scene.scene.tools.size() >= 2u,
                "profile OpenGL playback should retain both selected cutters")) {
        std::cerr << scene.error << '\n';
        return false;
    }
    std::vector<unsigned char> usedTools(
        scene.scene.tools.size(), 0u);
    for (const auto& motion : scene.scene.motions) {
        if (!expect(motion.toolIndex >= 0 &&
                        static_cast<std::size_t>(motion.toolIndex) <
                            usedTools.size(),
                    "every profile OpenGL motion should reference a valid tool")) {
            return false;
        }
        usedTools[static_cast<std::size_t>(motion.toolIndex)] = 1u;
    }
    return expect(
        std::count(
            usedTools.begin(), usedTools.end(),
            static_cast<unsigned char>(1u)) >= 2,
        "profile OpenGL playback should exercise both full and rest cutters");
}

bool reportsConcreteSmallestToolFailure()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("profile-auto-failure-test"),
        QStringLiteral("Profile Automatic Failure Test"));
    addTool(library, 7, QStringLiteral("flat-4-failure"), 4.0);

    oap::toolpath::ProfileRegion region = lShapedExternalProfile();
    // 毛坯与成品边界完全重合，没有可供外轮廓粗加工去除的材料。
    // 用它稳定触发最小候选刀的 full stage 失败。
    region.stockBoundary = region.boundary;
    auto params = externalClearingParams();
    params.profile.finishPass = false;
    const auto result = oap::tooling::ProfileAutomaticToolpath().generate(
        library, {region}, params,
        "PROFILE_AUTO_FAILURE_");

    const bool detailed = !result.ok() &&
            result.error.find("smallest usable cutter T7 D4") !=
                std::string::npos &&
            result.error.find("region 1:") != std::string::npos &&
            result.error.find("profile exterior offset produced") !=
                std::string::npos &&
            result.candidateGeometryCacheEntryCount == 0u;
    if (!detailed)
        std::cerr << "automatic profile failure: " << result.error << '\n';
    return expect(
        detailed,
        "automatic profile failure should identify the smallest cutter and its first failing region");
}

bool reusesMaterialSectionsAcrossAutomaticStages()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("profile-auto-cache-test"),
        QStringLiteral("Profile Automatic Cache Test"));
    addTool(library, 11, QStringLiteral("flat-10-cache"), 10.0);
    addTool(library, 12, QStringLiteral("flat-8-cache"), 8.0);
    addTool(library, 13, QStringLiteral("flat-6-cache"), 6.0);
    addTool(library, 14, QStringLiteral("flat-4-cache"), 4.0);

    oap::toolpath::ProfileRegion region;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0), region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.depth = 2.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(20.0, 20.0), gp_Pnt2d(80.0, 20.0),
        gp_Pnt2d(80.0, 80.0), gp_Pnt2d(20.0, 80.0)};
    region.boundary.signedArea = 3600.0;
    region.stockBoundary.closed = true;
    region.stockBoundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(100.0, 0.0),
        gp_Pnt2d(100.0, 100.0), gp_Pnt2d(0.0, 100.0)};
    region.stockBoundary.signedArea = 10000.0;
    region.source.rootFaceId = "profile-cache-root";
    region.source.rootWireId = "profile-cache-wire";
    region.protectedMaterialShape = BRepPrimAPI_MakeBox(
        gp_Pnt(20.0, 20.0, 0.0), 60.0, 60.0, 2.0).Shape();
    BRep_Builder builder;
    builder.MakeCompound(region.sectionShape);
    builder.Add(region.sectionShape, region.protectedMaterialShape);
    region.useMaterialSectionBoundaryAtEveryDepth = true;

    auto params = externalClearingParams();
    params.depth.stepDown = 1.0;
    const auto result = oap::tooling::ProfileAutomaticToolpath().generate(
        library, {region}, params, "PROFILE_AUTO_CACHE_");
    oap::toolpath::ToolpathGenerationControl exhaustiveControl;
    exhaustiveControl.profileExhaustiveAutomaticEvaluation = true;
    exhaustiveControl.tryAcquireParallelWorkers =
        [](std::size_t) { return 0u; };
    exhaustiveControl.releaseParallelWorkers = [](std::size_t) {};
    const auto exhaustive =
        oap::tooling::ProfileAutomaticToolpath().generate(
            library, {region}, params,
            "PROFILE_AUTO_CACHE_EXHAUSTIVE_",
            &exhaustiveControl);
    oap::toolpath::ToolpathGenerationControl boundedControl;
    boundedControl.profileEvaluationCacheByteLimit = 1u;
    boundedControl.tryAcquireParallelWorkers =
        [](std::size_t) { return 0u; };
    boundedControl.releaseParallelWorkers = [](std::size_t) {};
    const auto bounded =
        oap::tooling::ProfileAutomaticToolpath().generate(
            library, {region}, params,
            "PROFILE_AUTO_CACHE_BOUNDED_", &boundedControl);
    if (!expect(result.ok(),
                "cached automatic profile generation should succeed")) {
        std::cerr << result.error << '\n';
        return false;
    }
    const bool timedStages = std::all_of(
        result.stageAudits.begin(), result.stageAudits.end(),
        [](const oap::tooling::ProfileAutomaticToolpathStageAudit& audit) {
            return audit.stage.generationMilliseconds > 0.0;
        });
    std::cout << "profile automatic cache: hits="
              << result.materialSectionCacheHitCount
              << ", misses=" << result.materialSectionCacheMissCount
              << ", geometry_hits="
              << result.candidateGeometryCacheHitCount
              << ", geometry_misses="
              << result.candidateGeometryCacheMissCount
              << ", geometry_entries="
              << result.candidateGeometryCacheEntryCount
              << ", geometry_bytes="
              << result.candidateGeometryCacheEstimatedBytes
              << ", evaluated=" << result.evaluatedStageCount
              << ", pruned="
              << result.costBoundPrunedTransitionCount
              << ", materialized=" << result.materializedStageCount
              << ", promoted=" << result.promotedStageCount
              << ", cached=" << result.cachedEvaluationStageCount
              << ", released=" << result.releasedEvaluationStageCount
              << ", peak_loops="
              << result.peakRetainedEvaluationLoopCount
              << ", peak_bytes="
              << result.peakRetainedEvaluationBytes
              << ", total_ms=" << result.totalMilliseconds << '\n';
    return expect(
        result.candidateToolCount == 4 && result.ok() && exhaustive.ok() &&
            bounded.ok() &&
            result.evaluatedStageCount <
                exhaustive.evaluatedStageCount &&
            exhaustive.evaluatedStageCount == 10 &&
            exhaustive.costBoundPrunedTransitionCount == 0 &&
            result.evaluatedStageCount +
                    result.costBoundPrunedTransitionCount ==
                exhaustive.evaluatedStageCount &&
            result.costBoundPrunedTransitionCount > 0 &&
            sameAutomaticProgram(result, exhaustive, false) &&
            sameAutomaticProgram(result, bounded, false) &&
            result.evictedEvaluationStageCount == 0 &&
            result.reevaluatedEvictedStageCount == 0 &&
            result.transferredEvaluationStageCount ==
                result.materializedStageCount &&
            result.peakRetainedEvaluationBytes > 0u &&
            bounded.evaluationCacheByteLimit == 1u &&
            bounded.evictedEvaluationStageCount > 0 &&
            bounded.reevaluatedEvictedStageCount > 0 &&
            bounded.releasedEvaluationStageCount ==
                bounded.cachedEvaluationStageCount &&
            result.materializedStageCount < result.evaluatedStageCount &&
            result.promotedStageCount == result.materializedStageCount &&
            result.cachedEvaluationStageCount ==
                result.releasedEvaluationStageCount &&
            result.peakRetainedEvaluationLoopCount > 0u &&
            result.materialSectionCacheMissCount > 0u &&
            result.candidateGeometryCacheMissCount > 0u &&
            result.candidateGeometryCacheHitCount >=
                result.candidateGeometryCacheMissCount &&
            result.candidateGeometryCacheEntryCount ==
                result.candidateGeometryCacheMissCount &&
            result.candidateGeometryCacheEntryCount <
                exhaustive.candidateGeometryCacheEntryCount &&
            result.candidateGeometryCacheEstimatedBytes > 0u &&
            result.totalMilliseconds > 0.0 && timedStages,
        "cost-bounded automatic planning should skip dominated transitions while preserving the exhaustive program");
}

bool parallelEvaluationPreservesSerialProgram()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("profile-auto-parallel-test"),
        QStringLiteral("Profile Automatic Parallel Test"));
    addTool(library, 31, QStringLiteral("flat-20-parallel"), 20.0);
    addTool(library, 32, QStringLiteral("flat-10-parallel"), 10.0);
    addTool(library, 33, QStringLiteral("flat-6-parallel"), 6.0);
    addTool(library, 34, QStringLiteral("flat-4-parallel"), 4.0);

    oap::toolpath::ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers =
        [](std::size_t) { return 0u; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    const auto serial =
        oap::tooling::ProfileAutomaticToolpath().generate(
            library, {lShapedExternalProfile()},
            externalClearingParams(), "PROFILE_AUTO_SERIAL_",
            &serialControl);

    std::atomic_size_t acquired{0u};
    std::atomic_size_t released{0u};
    oap::toolpath::ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers =
        [&](std::size_t requested) {
            const std::size_t granted = std::min<std::size_t>(
                requested, 3u);
            acquired.fetch_add(granted, std::memory_order_relaxed);
            return granted;
        };
    parallelControl.releaseParallelWorkers =
        [&](std::size_t count) {
            released.fetch_add(count, std::memory_order_relaxed);
        };
    const auto parallel =
        oap::tooling::ProfileAutomaticToolpath().generate(
            library, {lShapedExternalProfile()},
            externalClearingParams(), "PROFILE_AUTO_PARALLEL_",
            &parallelControl);

    if (!expect(serial.ok() && parallel.ok(),
                "serial and parallel profile planning should succeed")) {
        std::cerr << "serial: " << serial.error
                  << "\nparallel: " << parallel.error << '\n';
        return false;
    }
    std::cout << "profile automatic parallel: serial_eval_ms="
              << serial.evaluationMilliseconds
              << ", parallel_eval_ms="
              << parallel.evaluationMilliseconds
              << ", batches="
              << parallel.parallelEvaluationBatchCount
              << ", stages="
              << parallel.parallelEvaluatedStageCount
              << ", peak="
              << parallel.evaluationPeakParallelism << '\n';
    return expect(
        serial.parallelEvaluationBatchCount == 0 &&
            serial.evaluationPeakParallelism == 1u &&
            parallel.parallelEvaluationBatchCount > 0 &&
            parallel.parallelEvaluatedStageCount >= 2 &&
            parallel.evaluationPeakParallelism >= 2u &&
            parallel.evaluatedStageCount == serial.evaluatedStageCount &&
            acquired.load(std::memory_order_relaxed) > 0u &&
            acquired.load(std::memory_order_relaxed) ==
                released.load(std::memory_order_relaxed) &&
            sameAutomaticProgram(serial, parallel),
        "bounded parallel evaluation should preserve the serial tool chain, audits and moves");
}

bool prunesLargeToolLibraryWithStableLatency()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("profile-auto-large-library-test"),
        QStringLiteral("Profile Automatic Large Library Test"));
    const std::vector<double> diameters = {
        24.0, 20.0, 18.0, 16.0, 14.0, 12.0,
        10.0, 8.0, 7.0, 6.0, 5.0, 4.0};
    for (std::size_t index = 0u; index < diameters.size(); ++index) {
        addTool(
            library, 60 + static_cast<int>(index),
            QStringLiteral("flat-large-library-%1").arg(index),
            diameters[index]);
    }
    const auto region = lShapedExternalProfile();
    const auto params = externalClearingParams();
    auto pruned = oap::tooling::ProfileAutomaticToolpath().generate(
        library, {region}, params, "PROFILE_AUTO_LARGE_");
    std::vector<double> samples{pruned.totalMilliseconds};
    for (int repeat = 0; repeat < 4; ++repeat) {
        const auto repeated =
            oap::tooling::ProfileAutomaticToolpath().generate(
                library, {region}, params,
                "PROFILE_AUTO_LARGE_");
        if (!expect(
                repeated.ok() &&
                    sameAutomaticProgram(pruned, repeated),
                "repeated large-library planning should preserve audits and every move")) {
            return false;
        }
        samples.push_back(repeated.totalMilliseconds);
    }
    oap::toolpath::ToolpathGenerationControl exhaustiveControl;
    exhaustiveControl.profileExhaustiveAutomaticEvaluation = true;
    exhaustiveControl.tryAcquireParallelWorkers =
        [](std::size_t) { return 0u; };
    exhaustiveControl.releaseParallelWorkers = [](std::size_t) {};
    const auto exhaustive =
        oap::tooling::ProfileAutomaticToolpath().generate(
            library, {region}, params,
            "PROFILE_AUTO_LARGE_EXHAUSTIVE_",
            &exhaustiveControl);
    std::sort(samples.begin(), samples.end());
    const double p50 = samples[samples.size() / 2u];
    const double p95 = samples.back();
    const int exhaustiveStageCount =
        static_cast<int>(
            diameters.size() * (diameters.size() + 1u) / 2u);
    std::cout << "profile automatic 12-tool: evaluated="
              << pruned.evaluatedStageCount
              << ", pruned="
              << pruned.costBoundPrunedTransitionCount
              << ", exhaustive=" << exhaustive.evaluatedStageCount
              << ", p50_ms=" << p50 << ", p95_ms=" << p95
              << ", peak_bytes="
              << pruned.peakRetainedEvaluationBytes
              << ", geometry_bytes="
              << pruned.candidateGeometryCacheEstimatedBytes << '\n';
    return expect(
        pruned.ok() && exhaustive.ok() &&
            pruned.candidateToolCount ==
                static_cast<int>(diameters.size()) &&
            exhaustive.evaluatedStageCount == exhaustiveStageCount &&
            pruned.evaluatedStageCount <
                exhaustive.evaluatedStageCount &&
            pruned.evaluatedStageCount +
                    pruned.costBoundPrunedTransitionCount ==
                exhaustive.evaluatedStageCount &&
            sameAutomaticProgram(pruned, exhaustive, false) &&
            pruned.releasedEvaluationStageCount ==
                pruned.cachedEvaluationStageCount &&
            pruned.peakRetainedEvaluationBytes > 0u &&
            pruned.candidateGeometryCacheEstimatedBytes > 0u &&
            p50 > 0.0 && p95 <= std::max(10.0, 5.0 * p50),
        "12-tool planning should prune safely and satisfy its P50/P95 stability gate");
}

bool cancelsInsideParallelEvaluationBatch()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("profile-auto-parallel-cancel-test"),
        QStringLiteral("Profile Automatic Parallel Cancel Test"));
    addTool(library, 41, QStringLiteral("flat-20-cancel"), 20.0);
    addTool(library, 42, QStringLiteral("flat-10-cancel"), 10.0);
    addTool(library, 43, QStringLiteral("flat-6-cancel"), 6.0);
    addTool(library, 44, QStringLiteral("flat-4-cancel"), 4.0);

    std::atomic_size_t progressCount{0u};
    std::atomic_bool cancel{false};
    oap::toolpath::ToolpathGenerationControl control;
    control.isCancellationRequested = [&]() {
        return cancel.load(std::memory_order_relaxed);
    };
    control.reportProgress = [&](std::size_t, std::size_t) {
        if (progressCount.fetch_add(
                1u, std::memory_order_relaxed) + 1u >= 4u) {
            cancel.store(true, std::memory_order_relaxed);
        }
    };
    control.tryAcquireParallelWorkers =
        [](std::size_t requested) {
            return std::min<std::size_t>(requested, 2u);
        };
    control.releaseParallelWorkers = [](std::size_t) {};

    const auto result =
        oap::tooling::ProfileAutomaticToolpath().generate(
            library, {lShapedExternalProfile()},
            externalClearingParams(), "PROFILE_AUTO_CANCEL_",
            &control);
    return expect(
        result.cancelled && !result.ok() &&
            result.batch.operations.empty() &&
            result.parallelEvaluationBatchCount > 0 &&
            result.evaluationPeakParallelism >= 2u &&
            result.totalMilliseconds > 0.0 &&
            result.totalMilliseconds < 5000.0,
        "cancellation from a parallel profile evaluation batch should discard the partial program");
}

bool replansAfterSelectedSafetyFailure()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("profile-auto-replan-test"),
        QStringLiteral("Profile Automatic Replan Test"));
    addTool(library, 21, QStringLiteral("flat-20-collision"), 20.0);
    addTool(library, 22, QStringLiteral("flat-4-safe"), 4.0);
    oap::tooling::ToolBit* large =
        library.toolById(QStringLiteral("flat-20-collision"));
    if (!expect(large != nullptr,
                "replan fixture should expose its large cutter")) {
        return false;
    }
    large->geometry.cuttingEdgeLength = 2.0;
    large->geometry.overallLength = 2.0;
    large->geometry.holderDiameter = 80.0;
    large->geometry.holderLength = 10.0;

    const auto result = oap::tooling::ProfileAutomaticToolpath().generate(
        library, {lShapedExternalProfile()},
        externalClearingParams(), "PROFILE_AUTO_REPLAN_");
    if (!expect(result.ok(),
                "automatic profile planning should recover from a selected-stage collision")) {
        std::cerr << result.error << '\n';
        return false;
    }
    const bool recordedRejectedMaterialization = std::any_of(
        result.stageAudits.begin(), result.stageAudits.end(),
        [](const oap::tooling::ProfileAutomaticToolpathStageAudit& audit) {
            return !audit.evaluationOnly && !audit.usable &&
                audit.validationErrorCount > 0;
        });
    return expect(
        result.rejectedTransitionCount >= 1 &&
            result.materializedStageCount >= 2 &&
            result.promotedStageCount ==
                result.materializedStageCount &&
            result.cachedEvaluationStageCount ==
                result.releasedEvaluationStageCount &&
            result.stages.size() == 1u &&
            std::abs(result.stages.front().tool.diameter - 4.0) <=
                kTolerance &&
            recordedRejectedMaterialization,
        "failed selected profile transitions should be rejected before the safe chain is published");
}

bool separatesDirectionAndDepthRequirements()
{
    oap::tooling::ToolLibrary library;
    addTool(library, 1, QStringLiteral("short-small"), 5.0,
        oap::tooling::ToolType::EndMill, 0.0, 2.0, 3.0);
    addTool(library, 2, QStringLiteral("long-large"), 10.0);
    auto top = lShapedExternalProfile();
    top.source.rootFaceId = "shallow-top";
    auto side = top;
    side.source.rootFaceId = "deep-side";
    side.depth = 10.0;
    side.openDir = gp_Dir(1, 0, 0);
    side.frame = gp_Ax3(gp_Pnt(0, 0, 0), side.openDir, gp_Dir(0, 1, 0));
    const auto params = externalClearingParams();
    const auto result = oap::tooling::ProfileAutomaticToolpath().generate(library, {top, side}, params);
    if (!expect(result.ok(), result.error)) return false;
    bool shallowSmall = false, deepLarge = false;
    std::vector<std::string> ids;
    for (const auto& operation : result.batch.operations) {
        if (!expect(operation.sourceRegionIndex == 0 || operation.sourceRegionIndex == 1,
                    "grouped profile lost its source region index")) return false;
        if (operation.sourceRegionIndex == 0)
            shallowSmall |= operation.tool.toolId == "short-small";
        else {
            if (!expect(operation.tool.toolId == "long-large", "deep side selected a short tool")) return false;
            deepLarge = true;
        }
        if (!expect(std::find(ids.begin(), ids.end(), operation.operationId) == ids.end(),
                    "grouped profile operation ids collided")) return false;
        ids.push_back(operation.operationId);
    }
    if (!expect(shallowSmall && deepLarge, "deep side filtered the shallow region's small cutter")) return false;
    side.depth = 120.0;
    const auto rejected = oap::tooling::ProfileAutomaticToolpath().generate(library, {top, side}, params);
    if (!expect(!rejected.ok() && rejected.batch.operations.empty() &&
                rejected.error.find("deep-side") != std::string::npos &&
                rejected.error.find("120") != std::string::npos &&
                rejected.error.find("shallow-top") == std::string::npos,
                "unreachable scope must report the failing region without publishing a partial program")) return false;
    oap::toolpath::ToolpathGenerationControl cancel;
    cancel.isCancellationRequested = [] { return true; };
    const auto cancelled = oap::tooling::ProfileAutomaticToolpath().generate(library, {top, side}, params, "CANCEL_", &cancel);
    return expect(cancelled.cancelled && cancelled.batch.operations.empty(), "grouped cancellation published operations");
}

bool retriesLargerValidatedFinishingTool()
{
    oap::tooling::ToolLibrary library;
    addTool(library, 51, QStringLiteral("safe-large"), 20.0);
    auto referenceLibrary = library;
    addTool(library, 52, QStringLiteral("unsafe-small"), 4.0);
    auto* small = library.toolById(QStringLiteral("unsafe-small"));
    small->geometry.cuttingEdgeLength = 2.0;
    small->geometry.overallLength = 2.0;
    small->geometry.holderDiameter = 80.0;
    small->geometry.holderLength = 10.0;
    const auto params = externalClearingParams();
    const auto region = lShapedExternalProfile();
    const auto result = oap::tooling::ProfileAutomaticToolpath().generate(library, {region}, params);
    const auto reference = oap::tooling::ProfileAutomaticToolpath().generate(referenceLibrary, {region}, params);
    if (!expect(result.ok() && reference.ok(), "larger final tool fallback failed")) {
        std::cerr << result.error << '\n';
        return false;
    }
    const bool warning = std::any_of(result.validation.issues.begin(), result.validation.issues.end(),
        [](const auto& issue) { return issue.code == "profile.final_tool_fallback"; });
    return expect(warning && result.stages.back().finishPass &&
        result.batch.effectiveParams.tool.diameter == 20.0 &&
        sameAutomaticProgram(result, reference, false),
        "fallback must replan the full finish pass and match the independently validated larger-tool path");
}


bool acceptsRoundedMillingTools()
{
    for (const auto type : {oap::tooling::ToolType::BallEndMill, oap::tooling::ToolType::BullNose}) {
        oap::tooling::ToolLibrary library;
        const double radius = type == oap::tooling::ToolType::BallEndMill ? 2.0 : 0.5;
        // Ball libraries may leave the derived radius at zero.
        addTool(library, 1, QStringLiteral("rounded"), 4.0, type,
            type == oap::tooling::ToolType::BallEndMill ? 0.0 : radius);
        auto result = oap::tooling::ProfileAutomaticToolpath().generate(
            library, {lShapedExternalProfile()}, externalClearingParams(), "ROUNDED_");
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
        result = oap::tooling::ProfileAutomaticToolpath().generate(
            library, {lShapedExternalProfile()}, externalClearingParams(), "SAME_DIAMETER_");
        if (!expect(result.ok() && result.batch.operations.front().tool.toolNumber == 2,
                "same-diameter preference should preserve fuller floor coverage")) return false;
        oap::tooling::ToolLibrary roundedChain;
        addTool(roundedChain, 1, QStringLiteral("large-rounded"), 12.0, type,
            type == oap::tooling::ToolType::BallEndMill ? 6.0 : 1.0);
        addTool(roundedChain, 2, QStringLiteral("small-rounded"), 4.0, type, radius);
        const auto chain = oap::tooling::ProfileAutomaticToolpath().generate(
            roundedChain, {lShapedExternalProfile()}, externalClearingParams(), "ROUNDED_CHAIN_");
        if (!expect(chain.ok(), "rounded chain failed: " + chain.error)) return false;
        for (const auto& audit : chain.stageAudits)
            if (!expect(audit.stage.previousToolDiameter == 0.0,
                    "rounded predecessors must not claim flat-cylinder rest coverage")) return false;
    }
    return true;
}

} // namespace

int main()
{
    if (!acceptsRoundedMillingTools()) return 1;
    {
        oap::tooling::ToolLibrary library("unknown-reach", "Unknown reach");
        addTool(library, 1, QStringLiteral("unknown-length"), 5.0,
                oap::tooling::ToolType::EndMill, 0.0, 20.0, 0.0);
        if (!expect(!oap::tooling::ProfileAutomaticToolpath::hasCandidateTool(
                library, lShapedExternalProfile(), externalClearingParams()),
                "unknown tool reach must not authorize dropping setup directions")) return 1;
    }
    if (!sharesProcessWideParallelBudget() ||
        !separatesDirectionAndDepthRequirements() ||
        !retriesLargerValidatedFinishingTool() ||
        !selectsLargeToolThenSmallRestTool() ||
        !reportsConcreteSmallestToolFailure() ||
        !reusesMaterialSectionsAcrossAutomaticStages() ||
        !parallelEvaluationPreservesSerialProgram() ||
        !prunesLargeToolLibraryWithStableLatency() ||
        !cancelsInsideParallelEvaluationBatch() ||
        !replansAfterSelectedSafetyFailure())
        return 1;
    std::cout << "PASS: automatic profile tool selection checks\n";
    return 0;
}

#include "analysis/StepIndexer.h"
#include "analysis/AnalysisPipeline.h"
#include "analysis/ModelIndex.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"
#include "toolpath/profile/planning/ProfileContour.h"
#include "toolpath/profile/program/ProfileProgramAdapter.h"
#include "toolpath/profile/validation/ProfileSafetyValidator.h"
#include "toolpath/workflow/ToolpathWorkflow.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <filesystem>

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp.hxx>

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool sameContourProgram(
    const oap::toolpath::ProfileContourResult& left,
    const oap::toolpath::ProfileContourResult& right)
{
    if (left.ok != right.ok ||
        left.loops.size() != right.loops.size() ||
        left.moves.size() != right.moves.size() ||
        left.primitives.size() != right.primitives.size()) {
        return false;
    }
    for (std::size_t index = 0u; index < left.loops.size(); ++index) {
        const auto& a = left.loops[index];
        const auto& b = right.loops[index];
        if (a.closed != b.closed || a.passKind != b.passKind ||
            a.depthIndex != b.depthIndex ||
            a.radialPassIndex != b.radialPassIndex ||
            a.sequence != b.sequence ||
            a.points.size() != b.points.size()) {
            return false;
        }
        for (std::size_t point = 0u; point < a.points.size(); ++point) {
            if (a.points[point].Distance(b.points[point]) > 1.0e-7)
                return false;
        }
    }
    for (std::size_t index = 0u; index < left.moves.size(); ++index) {
        const auto& a = left.moves[index];
        const auto& b = right.moves[index];
        if (a.type != b.type ||
            a.loopSequence != b.loopSequence ||
            a.depthIndex != b.depthIndex || a.ring != b.ring ||
            a.from.Distance(b.from) > 1.0e-7 ||
            a.to.Distance(b.to) > 1.0e-7) {
            return false;
        }
    }
    return true;
}

oap::toolpath::ProfileRegion rectangularProfile()
{
    oap::toolpath::ProfileRegion region;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0),
        region.openDir, gp_Dir(1.0, 0.0, 0.0));
    region.depth = 6.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(40.0, 0.0),
        gp_Pnt2d(40.0, 30.0), gp_Pnt2d(0.0, 30.0)};
    region.boundary.signedArea = 1200.0;
    region.source.rootFaceId = "F_top";
    region.source.rootWireId = "W_outer";
    return region;
}

oap::toolpath::ToolpathParams profileParams()
{
    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 4.0;
    params.tool.horizFeed = 600.0;
    params.tool.vertFeed = 180.0;
    params.depth.useFeatureDepth = true;
    params.depth.stepDown = 2.0;
    params.depth.safeHeight = 3.0;
    params.depth.clearanceHeight = 5.0;
    params.profile.radialPassCount = 3;
    params.profile.radialStepOver = 1.0;
    params.profile.sideAllowance = 0.2;
    return params;
}

bool followsFreeCadProfileSemantics()
{
    const oap::toolpath::ProfileRegion region = rectangularProfile();
    oap::toolpath::ToolpathParams params = profileParams();
    const oap::toolpath::ProfileContourResult result =
        oap::toolpath::ProfileContour().generate(region, params);
    if (!expect(result.ok, "profile generation should succeed") ||
        !expect(result.depthLayerCount == 3,
                "six millimetres at two millimetres step-down should create three layers") ||
        !expect(result.radialPassCount == 3 && result.loops.size() == 9,
                "each depth should contain three exterior radial passes") ||
        !expect(std::abs(result.finalOffset - 2.2) < 1.0e-7,
                "final offset should equal tool radius plus side allowance") ||
        !expect(result.loops[0].offset > result.loops[1].offset &&
                    result.loops[1].offset > result.loops[2].offset,
                "radial roughing should run from farthest to final contour") ||
        !expect(result.loops[0].area > 1200.0,
                "tool-center profile must remain outside protected material") ||
        !expect(result.loops.front().depthOffset == 4.0 &&
                    result.loops.back().depthOffset == 0.0,
                "depth passes should descend from opening toward final wall depth")) {
        return false;
    }

    oap::toolpath::ProfileToolpathOperation operation;
    operation.operationId = "profile-1";
    operation.rootFaceId = region.source.rootFaceId;
    operation.rootWireId = region.source.rootWireId;
    operation.contour = result;
    const oap::toolpath::ToolpathOperation neutral =
        oap::toolpath::toToolpathOperation(operation);
    return expect(
        neutral.type == oap::toolpath::ToolpathOperationType::ProfileContour &&
            neutral.metadata.at("profile_source") == "root_outer_wire" &&
            neutral.moves.size() == result.moves.size(),
        "profile adapter should preserve distinct operation semantics and moves");
}

bool reversesCutDirectionOnly()
{
    const oap::toolpath::ProfileRegion region = rectangularProfile();
    oap::toolpath::ToolpathParams climbParams = profileParams();
    climbParams.depth.stepDown = region.depth;
    climbParams.profile.radialPassCount = 1;
    oap::toolpath::ToolpathParams conventionalParams = climbParams;
    conventionalParams.profile.cutMode = oap::toolpath::CutMode::Conventional;
    const auto climb = oap::toolpath::ProfileContour().generate(
        region, climbParams);
    const auto conventional = oap::toolpath::ProfileContour().generate(
        region, conventionalParams);
    if (!expect(climb.ok && conventional.ok,
                "both profile cut directions should generate"))
        return false;
    const auto area = [](const std::vector<gp_Pnt2d>& points) {
        double sum = 0.0;
        for (std::size_t index = 0; index < points.size(); ++index) {
            const gp_Pnt2d& a = points[index];
            const gp_Pnt2d& b = points[(index + 1) % points.size()];
            sum += a.X() * b.Y() - b.X() * a.Y();
        }
        return 0.5 * sum;
    };
    return expect(area(climb.loops.front().points) > 0.0 &&
                      area(conventional.loops.front().points) < 0.0 &&
                      std::abs(climb.loops.front().area -
                               conventional.loops.front().area) < 1.0e-7,
                  "climb/conventional should reverse traversal without changing geometry");
}

bool linksSafeSameDepthProfilePasses()
{
    const oap::toolpath::ProfileRegion region = rectangularProfile();
    const oap::toolpath::ToolpathParams params = profileParams();
    const oap::toolpath::ProfileContourResult result =
        oap::toolpath::ProfileContour().generate(region, params);
    oap::toolpath::ToolpathGenerationControl evaluationControl;
    evaluationControl.profileEvaluationOnly = true;
    const oap::toolpath::ProfileContourResult evaluation =
        oap::toolpath::ProfileContour().generate(
            region, params, &evaluationControl);
    const oap::toolpath::ProfileToolpathBatch evaluationBatch =
        oap::toolpath::ToolpathWorkflow().generateProfiles(
            {region}, params, "PROFILE_EVALUATION_",
            &evaluationControl);
    const oap::toolpath::ProfileContourResult promoted =
        oap::toolpath::ProfileContour().materializeEvaluation(
            region, params, evaluation);
    const oap::toolpath::ProfileToolpathBatch promotedBatch =
        oap::toolpath::ProfileToolpathService().materializeEvaluated(
            {region}, evaluationBatch);
    double fullCuttingDistance = 0.0;
    for (const oap::toolpath::ToolpathMove& move : result.moves) {
        if (move.type == oap::toolpath::ToolpathMoveType::Cut)
            fullCuttingDistance += move.from.Distance(move.to);
    }
    if (!expect(result.ok && result.loops.size() == 9u,
                "profile linking fixture should generate three passes on three layers") ||
        !expect(result.directCutLinkCount == 6,
                "every pair of same-depth radial profile passes should link at cutting depth") ||
        !expect(result.retractEntryCount == result.depthLayerCount,
                "linked profile passes should require only one entry per depth layer") ||
        !expect(result.totalMilliseconds > 0.0 &&
                    result.offsetMilliseconds > 0.0 &&
                    result.moveAssemblyMilliseconds > 0.0,
                "profile generation should expose phase timing metrics") ||
        !expect(evaluation.ok && evaluation.evaluationOnly &&
                    evaluation.moves.empty() &&
                    evaluation.primitives.empty() &&
                    evaluation.loops.size() == result.loops.size() &&
                    std::abs(evaluation.estimatedCuttingDistance -
                             fullCuttingDistance) <= 1.0e-6,
                "profile evaluation should match full cutting distance without materializing moves") ||
        !expect(!evaluationBatch.safetyValidationPerformed &&
                    evaluationBatch.operations.size() == 1u &&
                    evaluationBatch.operations.front()
                        .contour.evaluationOnly &&
                    evaluationBatch.operations.front()
                        .contour.moves.empty(),
                "profile service evaluation should skip move safety validation") ||
        !expect(promoted.ok && !promoted.evaluationOnly &&
                    promoted.materializationMilliseconds > 0.0 &&
                    promoted.directCutLinkCount ==
                        result.directCutLinkCount &&
                    promoted.retractEntryCount ==
                        result.retractEntryCount &&
                    sameContourProgram(result, promoted),
                "promoted evaluation contours should exactly match full move generation") ||
        !expect(promotedBatch.safetyValidationPerformed &&
                    promotedBatch.safetyValidation.ok() &&
                    promotedBatch.operations.size() == 1u &&
                    promotedBatch.metrics.materializationMilliseconds >
                        0.0 &&
                    sameContourProgram(
                        result,
                        promotedBatch.operations.front().contour),
                "profile service promotion should validate the same executable program")) {
        std::cerr << "profile links=" << result.directCutLinkCount
                  << ", entries=" << result.retractEntryCount
                  << ", warning=" << result.warning << '\n';
        return false;
    }

    for (std::size_t index = 1u; index < result.moves.size(); ++index) {
        if (!expect(
                result.moves[index - 1u].to.Distance(
                    result.moves[index].from) <= 1.0e-7,
                "profile move stream should remain continuous after direct-link optimization")) {
            return false;
        }
    }
    for (std::size_t index = 1u; index < result.loops.size(); ++index) {
        if (result.loops[index - 1u].depthIndex ==
            result.loops[index].depthIndex) {
            if (!expect(
                    result.loops[index - 1u].linkSafetyBoundaries ==
                        result.loops[index].linkSafetyBoundaries,
                    "same-depth profile loops should share immutable safety boundaries")) {
                return false;
            }
        }
    }

    oap::toolpath::ProfileToolpathOperation operation;
    operation.operationId = "profile-linking";
    operation.outwardToolAxis = gp_Vec(region.openDir);
    operation.contour = result;
    const auto safety = oap::toolpath::ProfileSafetyValidator().validate(
        region, operation, params);
    return expect(
        safety.report.ok(),
        "same-depth profile cut links should remain outside the finished-part envelope");
}

bool parallelizesIndependentProfileRegions()
{
    std::vector<oap::toolpath::ProfileRegion> regions;
    for (int regionIndex = 0; regionIndex < 8; ++regionIndex) {
        auto region = rectangularProfile();
        const double shift = 60.0 * regionIndex;
        for (gp_Pnt2d& point : region.boundary.points)
            point.SetX(point.X() + shift);
        region.source.rootFaceId =
            "F_parallel_" + std::to_string(regionIndex);
        region.source.rootWireId =
            "W_parallel_" + std::to_string(regionIndex);
        regions.push_back(std::move(region));
    }
    const auto params = profileParams();
    oap::toolpath::ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers =
        [](std::size_t) { return 0u; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    const auto serial = oap::toolpath::ProfileToolpathService().generate(
        regions, params, "PROFILE_REGION_", &serialControl);

    std::atomic_size_t acquired{0u};
    std::atomic_size_t released{0u};
    oap::toolpath::ToolpathParallelBudget budget(
        oap::toolpath::toolpathHardwareParallelism() - 1u);
    oap::toolpath::ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers =
        [&](std::size_t requested) {
            const std::size_t granted =
                budget.tryAcquire(requested);
            acquired.fetch_add(granted, std::memory_order_relaxed);
            return granted;
        };
    parallelControl.releaseParallelWorkers = [&](std::size_t count) {
        released.fetch_add(count, std::memory_order_relaxed);
        budget.release(count);
    };
    const auto parallel = oap::toolpath::ProfileToolpathService().generate(
        regions, params, "PROFILE_REGION_", &parallelControl);

    oap::toolpath::ToolpathGenerationControl evaluationControl =
        serialControl;
    evaluationControl.profileEvaluationOnly = true;
    const auto evaluation =
        oap::toolpath::ProfileToolpathService().generate(
            regions, params, "PROFILE_REGION_", &evaluationControl);
    const auto promoted =
        oap::toolpath::ProfileToolpathService().materializeEvaluated(
            regions, evaluation, &parallelControl);
    if (!expect(serial.operations.size() == regions.size() &&
                    serial.failures.empty() &&
                    parallel.operations.size() == serial.operations.size() &&
                    parallel.failures.empty() &&
                    promoted.operations.size() == serial.operations.size() &&
                    promoted.failures.empty(),
                "serial, parallel and promoted multi-region profile batches should succeed")) {
        return false;
    }
    for (std::size_t index = 0u; index < serial.operations.size(); ++index) {
        if (!expect(
                sameContourProgram(
                    serial.operations[index].contour,
                    parallel.operations[index].contour) &&
                    sameContourProgram(
                        serial.operations[index].contour,
                        promoted.operations[index].contour),
                "parallel region generation and promotion should preserve every move")) {
            return false;
        }
    }
    return expect(
        serial.metrics.parallelRegionBatchCount == 0 &&
            parallel.metrics.parallelRegionBatchCount > 0 &&
            parallel.metrics.regionPeakParallelism ==
                oap::toolpath::toolpathParallelismFor(regions.size()) &&
            promoted.metrics.parallelRegionBatchCount > 0 &&
            promoted.metrics.regionPeakParallelism >= 2u &&
            acquired.load(std::memory_order_relaxed) > 0u &&
            acquired.load(std::memory_order_relaxed) ==
                released.load(std::memory_order_relaxed),
        "independent profile regions should share a bounded worker lease and merge deterministically");
}

bool parallelizesIndependentDepthLayerPatterns()
{
    const auto region = rectangularProfile();
    auto params = profileParams();
    params.depth.stepDown = 1.0;
    oap::toolpath::ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers =
        [](std::size_t) { return 0u; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    const auto serial = oap::toolpath::ProfileContour().generate(
        region, params, &serialControl);

    std::atomic_size_t acquired{0u};
    std::atomic_size_t released{0u};
    oap::toolpath::ToolpathParallelBudget budget(
        oap::toolpath::toolpathHardwareParallelism() - 1u);
    oap::toolpath::ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers =
        [&](std::size_t requested) {
            const std::size_t granted =
                budget.tryAcquire(requested);
            acquired.fetch_add(granted, std::memory_order_relaxed);
            return granted;
        };
    parallelControl.releaseParallelWorkers = [&](std::size_t count) {
        released.fetch_add(count, std::memory_order_relaxed);
        budget.release(count);
    };
    const auto parallel = oap::toolpath::ProfileContour().generate(
        region, params, &parallelControl);
    return expect(
        serial.ok && parallel.ok &&
            sameContourProgram(serial, parallel) &&
            serial.parallelLayerBatchCount == 0 &&
            serial.layerPeakParallelism == 1u &&
            parallel.parallelLayerBatchCount == 1 &&
            parallel.parallelLayerCount == parallel.depthLayerCount &&
            parallel.layerPeakParallelism ==
                oap::toolpath::toolpathParallelismFor(parallel.depthLayerCount) &&
            acquired.load(std::memory_order_relaxed) > 0u &&
            acquired.load(std::memory_order_relaxed) ==
                released.load(std::memory_order_relaxed),
        "depth prefix envelopes should stay ordered while independent layer patterns run in parallel");
}

bool clearsStockAcrossOpenProfileNotch()
{
    oap::toolpath::ProfileRegion region = rectangularProfile();
    region.boundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(40.0, 0.0),
        gp_Pnt2d(40.0, 30.0), gp_Pnt2d(25.0, 30.0),
        gp_Pnt2d(25.0, 22.0), gp_Pnt2d(15.0, 22.0),
        gp_Pnt2d(15.0, 30.0), gp_Pnt2d(0.0, 30.0)};
    region.boundary.signedArea = 1120.0;
    region.stockBoundary.closed = true;
    region.stockBoundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(40.0, 0.0),
        gp_Pnt2d(40.0, 30.0), gp_Pnt2d(0.0, 30.0)};
    region.stockBoundary.signedArea = 1200.0;

    oap::toolpath::ToolpathParams params = profileParams();
    params.depth.stepDown = region.depth;
    params.profile.machiningMode =
        oap::toolpath::ProfileMachiningMode::ExternalClearing;
    params.profile.roughingAllowance = 0.2;
    params.profile.finishPass = true;
    params.clearing.pattern = oap::toolpath::ClearingPattern::ZigZag;
    params.clearing.stepOverPercent = 40.0;
    const auto result = oap::toolpath::ProfileContour().generate(
        region, params);

    int clearingLoops = 0;
    bool cutsInsideOpenNotch = false;
    bool hasOpenClearingPath = false;
    bool hasFinalContour = false;
    for (const oap::toolpath::ProfileLoop& loop : result.loops) {
        if (loop.passKind == oap::toolpath::ProfilePassKind::Finish) {
            hasFinalContour = true;
            continue;
        }
        ++clearingLoops;
        hasOpenClearingPath = hasOpenClearingPath || !loop.closed;
        for (const gp_Pnt2d& point : loop.points) {
            if (point.X() > 17.0 && point.X() < 23.0 &&
                point.Y() > 24.0 && point.Y() < 30.0) {
                cutsInsideOpenNotch = true;
            }
        }
    }
    return expect(
        result.ok && result.usedExternalClearing &&
            clearingLoops >= 2 && hasOpenClearingPath &&
            cutsInsideOpenNotch && hasFinalContour &&
            result.retractEntryCount >= 2 &&
            result.retractEntryCount + result.directCutLinkCount ==
                static_cast<int>(result.loops.size()),
        "stock-aware profile clearing must sweep the notch and retract when a same-depth shortcut is unsafe");
}

bool completesDisconnectedClearingAreaBeforeTravel()
{
    oap::toolpath::ProfileRegion region = rectangularProfile();
    region.depth = 2.0;
    // 横贯毛坯的成品条带把外部余料分成上下两个不连通区域。偏置图案原始
    // 输出为上下区域逐圈交替，本回归要求同一深度先完成一侧的全部圈次。
    region.boundary.points = {
        gp_Pnt2d(0.0, 45.0), gp_Pnt2d(100.0, 45.0),
        gp_Pnt2d(100.0, 55.0), gp_Pnt2d(0.0, 55.0)};
    region.boundary.signedArea = 1000.0;
    region.stockBoundary.closed = true;
    region.stockBoundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(100.0, 0.0),
        gp_Pnt2d(100.0, 100.0), gp_Pnt2d(0.0, 100.0)};
    region.stockBoundary.signedArea = 10000.0;

    oap::toolpath::ToolpathParams params = profileParams();
    params.depth.stepDown = region.depth;
    params.profile.machiningMode =
        oap::toolpath::ProfileMachiningMode::ExternalClearing;
    params.profile.roughingAllowance = 0.2;
    params.profile.finishPass = false;
    params.clearing.pattern = oap::toolpath::ClearingPattern::Offset;
    params.clearing.stepOverPercent = 50.0;
    const auto result = oap::toolpath::ProfileContour().generate(
        region, params);

    int previousArea = -1;
    int areaTransitions = 0;
    int lowerLoopCount = 0;
    int upperLoopCount = 0;
    int lowerPreviousRing = -1;
    int upperPreviousRing = -1;
    bool ringsRemainOrdered = true;
    for (const oap::toolpath::ProfileLoop& loop : result.loops) {
        if (loop.points.empty() ||
            loop.passKind != oap::toolpath::ProfilePassKind::Clearing) {
            continue;
        }
        double meanY = 0.0;
        for (const gp_Pnt2d& point : loop.points)
            meanY += point.Y();
        meanY /= static_cast<double>(loop.points.size());
        const int area = meanY < 50.0 ? 0 : 1;
        if (previousArea >= 0 && area != previousArea)
            ++areaTransitions;
        previousArea = area;
        int& loopCount = area == 0 ? lowerLoopCount : upperLoopCount;
        int& previousRing =
            area == 0 ? lowerPreviousRing : upperPreviousRing;
        ringsRemainOrdered = ringsRemainOrdered &&
            loop.radialPassIndex >= previousRing;
        previousRing = loop.radialPassIndex;
        ++loopCount;
    }
    return expect(
        result.ok && result.depthLayerCount == 1 &&
            lowerLoopCount >= 2 && upperLoopCount >= 2 &&
            areaTransitions == 1 && ringsRemainOrdered,
        "same-depth disconnected profile clearing areas must complete one area before travelling to the next");
}

bool completesSplitClearingBranchBeforeSibling()
{
    oap::toolpath::ProfileRegion region = rectangularProfile();
    region.depth = 2.0;
    // The protected bar nearly reaches the left stock edge. At the first
    // offset the upper and lower clearing lobes share a narrow corridor;
    // later rings split into two branches. Each branch must then finish
    // before machining switches to its sibling.
    region.boundary.points = {
        gp_Pnt2d(8.0, 45.0), gp_Pnt2d(100.0, 45.0),
        gp_Pnt2d(100.0, 55.0), gp_Pnt2d(8.0, 55.0)};
    region.boundary.signedArea = 920.0;
    region.stockBoundary.closed = true;
    region.stockBoundary.points = {
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(100.0, 0.0),
        gp_Pnt2d(100.0, 100.0), gp_Pnt2d(0.0, 100.0)};
    region.stockBoundary.signedArea = 10000.0;

    oap::toolpath::ToolpathParams params = profileParams();
    params.depth.stepDown = region.depth;
    params.profile.machiningMode =
        oap::toolpath::ProfileMachiningMode::ExternalClearing;
    params.profile.roughingAllowance = 0.2;
    params.profile.finishPass = false;
    params.clearing.pattern = oap::toolpath::ClearingPattern::Offset;
    params.clearing.stepOverPercent = 50.0;
    const auto result = oap::toolpath::ProfileContour().generate(
        region, params);

    int previousBranch = -1;
    int branchTransitions = 0;
    int lowerLoops = 0;
    int upperLoops = 0;
    for (const oap::toolpath::ProfileLoop& loop : result.loops) {
        if (loop.points.empty() ||
            loop.passKind != oap::toolpath::ProfilePassKind::Clearing) {
            continue;
        }
        double meanY = 0.0;
        for (const gp_Pnt2d& point : loop.points)
            meanY += point.Y();
        meanY /= static_cast<double>(loop.points.size());
        if (meanY >= 40.0 && meanY <= 60.0)
            continue;
        const int branch = meanY < 40.0 ? 0 : 1;
        if (previousBranch >= 0 && branch != previousBranch)
            ++branchTransitions;
        previousBranch = branch;
        branch == 0 ? ++lowerLoops : ++upperLoops;
    }
    return expect(
        result.ok && result.depthLayerCount == 1 &&
            lowerLoops >= 2 && upperLoops >= 2 &&
            branchTransitions == 1,
        "same-depth offset branches joined by a narrow entrance must complete one branch before its sibling");
}

bool limitsSmallCutterToPreviousToolRestMaterial()
{
    oap::toolpath::ProfileRegion region = rectangularProfile();
    region.depth = 2.0;
    // L 形保护轮廓放在更大的矩形毛坯中：大刀可高效清除外围，但无法
    // 完整进入内凹角；小刀余料阶段应只覆盖这部分增量可达材料。
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

    oap::toolpath::ToolpathParams params = profileParams();
    params.tool.diameter = 4.0;
    params.depth.stepDown = region.depth;
    params.profile.machiningMode =
        oap::toolpath::ProfileMachiningMode::ExternalClearing;
    params.profile.roughingAllowance = 0.2;
    params.profile.finishPass = false;
    params.clearing.pattern = oap::toolpath::ClearingPattern::Offset;
    params.clearing.stepOverPercent = 50.0;
    const auto full = oap::toolpath::ProfileContour().generate(
        region, params);
    oap::toolpath::ToolpathGenerationControl cachedControl;
    cachedControl.profileCandidateGeometryCache =
        std::make_shared<
            oap::toolpath::ProfileCandidateGeometryCache>();
    const auto cachedFull = oap::toolpath::ProfileContour().generate(
        region, params, &cachedControl);
    params.clearing.restMachiningPreviousToolDiameter = 20.0;
    const auto rest = oap::toolpath::ProfileContour().generate(
        region, params);
    const auto cachedRest = oap::toolpath::ProfileContour().generate(
        region, params, &cachedControl);
    const std::size_t entriesBeforeAllowanceChange =
        cachedControl.profileCandidateGeometryCache->entryCount();
    auto changedAllowance = params;
    changedAllowance.profile.roughingAllowance = 0.5;
    const auto changedAllowanceResult =
        oap::toolpath::ProfileContour().generate(
            region, changedAllowance, &cachedControl);

    const auto cuttingDistance = [](const auto& result) {
        double distance = 0.0;
        for (const auto& move : result.moves) {
            if (move.type == oap::toolpath::ToolpathMoveType::Cut)
                distance += move.from.Distance(move.to);
        }
        return distance;
    };
    return expect(
        full.ok && rest.ok && cachedFull.ok && cachedRest.ok &&
            changedAllowanceResult.ok && rest.restMachining &&
            !rest.noRestMaterial &&
            std::abs(rest.restMachiningPreviousToolDiameter - 20.0) <
                1.0e-9 &&
            !rest.loops.empty() &&
            full.travelReorderedLoopCount == 0u &&
            full.travelAdjustedStartCount == 0u &&
            rest.travelAdjustedStartCount > 0u &&
            sameContourProgram(full, cachedFull) &&
            sameContourProgram(rest, cachedRest) &&
            cachedRest.candidateGeometryCacheHitCount > 0u &&
            cachedControl.profileCandidateGeometryCache->entryCount() >
                entriesBeforeAllowanceChange &&
            cuttingDistance(rest) + 1.0e-6 < cuttingDistance(full),
        "small-cutter rest machining must preserve uncached geometry, reuse cutter areas and invalidate changed allowances");
}

bool validatesFinishedPartAndToolReach()
{
    const oap::toolpath::ProfileRegion region = rectangularProfile();
    oap::toolpath::ToolpathParams params = profileParams();
    params.tool.cuttingEdgeLength = 12.0;
    params.tool.overallLength = 30.0;
    params.tool.shankDiameter = 4.0;

    oap::toolpath::ProfileToolpathOperation operation;
    operation.operationId = "profile-safety";
    operation.outwardToolAxis = gp_Vec(region.openDir);
    operation.contour =
        oap::toolpath::ProfileContour().generate(region, params);
    const auto safe = oap::toolpath::ProfileSafetyValidator().validate(
        region, operation, params);
    if (!expect(operation.contour.ok && safe.report.ok(),
                "generated exterior profile should pass finished-part safety validation")) {
        return false;
    }

    auto intrusion = operation;
    const auto cut = std::find_if(
        intrusion.contour.moves.begin(), intrusion.contour.moves.end(),
        [](const oap::toolpath::ToolpathMove& move) {
            return move.type == oap::toolpath::ToolpathMoveType::Cut;
        });
    if (!expect(cut != intrusion.contour.moves.end(),
                "profile fixture should contain a cutting move")) {
        return false;
    }
    cut->from = gp_Pnt(20.0, 15.0, 0.0);
    cut->to = gp_Pnt(21.0, 15.0, 0.0);
    const auto unsafe = oap::toolpath::ProfileSafetyValidator().validate(
        region, intrusion, params);
    if (!expect(!unsafe.report.ok() && !unsafe.diagnostics.empty() &&
                    unsafe.diagnostics.front().code ==
                        oap::toolpath::ToolpathDiagnosticCode::
                            ProfileFinishedPartIntrusion,
                "a cutting move inside the finished part must be rejected")) {
        return false;
    }

    auto rapidCollision = operation;
    const auto rapid = std::find_if(
        rapidCollision.contour.moves.begin(),
        rapidCollision.contour.moves.end(),
        [](const oap::toolpath::ToolpathMove& move) {
            return move.type == oap::toolpath::ToolpathMoveType::Rapid;
        });
    if (!expect(rapid != rapidCollision.contour.moves.end(),
                "profile fixture should contain a rapid move")) {
        return false;
    }
    rapid->from = gp_Pnt(20.0, 15.0, 0.0);
    rapid->to = gp_Pnt(21.0, 15.0, 0.0);
    const auto rapidUnsafe =
        oap::toolpath::ProfileSafetyValidator().validate(
            region, rapidCollision, params);
    if (!expect(!rapidUnsafe.report.ok() &&
                    rapidUnsafe.diagnostics.front().code ==
                        oap::toolpath::ToolpathDiagnosticCode::
                            NonCuttingMoveCollision,
                "a rapid move through finished material must be rejected")) {
        return false;
    }

    oap::toolpath::ToolpathParams assemblyParams = params;
    assemblyParams.tool.cuttingEdgeLength = region.depth;
    assemblyParams.tool.overallLength = region.depth;
    assemblyParams.tool.holderDiameter = 20.0;
    assemblyParams.tool.holderLength = 5.0;
    const auto assemblyUnsafe =
        oap::toolpath::ProfileSafetyValidator().validate(
            region, operation, assemblyParams);
    if (!expect(!assemblyUnsafe.report.ok() &&
                    std::any_of(
                        assemblyUnsafe.diagnostics.begin(),
                        assemblyUnsafe.diagnostics.end(),
                        [](const oap::toolpath::ToolpathDiagnostic& item) {
                            return item.code ==
                                oap::toolpath::ToolpathDiagnosticCode::
                                    CuttingMoveCollision;
                        }),
                "a holder envelope that reaches the finished-part top edge must be rejected")) {
        return false;
    }

    oap::toolpath::ProfileRegion deepRegion = region;
    deepRegion.depth = 70.0;
    oap::toolpath::ToolpathParams layeredParams = profileParams();
    layeredParams.tool.cuttingEdgeLength = 5.0;
    layeredParams.tool.overallLength = 100.0;
    const auto layered = oap::toolpath::ProfileToolpathService().generate(
        {deepRegion}, layeredParams);
    if (!expect(
            layered.operations.size() == 1 && layered.failures.empty(),
            "a deep profile must accept a cutting edge that covers each axial layer when tool reach is sufficient")) {
        return false;
    }

    oap::toolpath::ToolpathParams shortReachParams = profileParams();
    shortReachParams.tool.cuttingEdgeLength = 5.0;
    shortReachParams.tool.overallLength = 50.0;
    const auto shortReach = oap::toolpath::ProfileToolpathService().generate(
        {deepRegion}, shortReachParams);
    if (!expect(
            shortReach.operations.empty() &&
                std::any_of(
                    shortReach.diagnostics.begin(),
                    shortReach.diagnostics.end(),
                    [](const oap::toolpath::ToolpathDiagnostic& item) {
                        return item.code ==
                            oap::toolpath::ToolpathDiagnosticCode::
                                ProfileToolReachInsufficient;
                    }),
            "a deep profile must reject a known tool reach shorter than total depth")) {
        return false;
    }

    params.tool.cuttingEdgeLength = 1.0;
    const auto rejected = oap::toolpath::ProfileToolpathService().generate(
        {region, region}, params);
    return expect(
        rejected.operations.empty() && rejected.failures.size() == 2 &&
            rejected.safetyValidationPerformed &&
            !rejected.safetyValidation.ok() &&
            rejected.diagnostics.size() >= 2 &&
            rejected.diagnostics[0].operationId == "PROFILE1" &&
            rejected.diagnostics[0].sourceIndex == 0 &&
            rejected.diagnostics[1].operationId == "PROFILE2" &&
            rejected.diagnostics[1].sourceIndex == 1 &&
            rejected.failures[0].contour.error.find(
                "maximum axial engagement") != std::string::npos,
        "profile service must reject each insufficient layer with a stable region identity");
}

bool buildsRealOuterWireRegion()
{
    Handle(TDocStd_Document) document;
    Handle(XCAFApp_Application) application =
        XCAFApp_Application::GetApplication();
    application->NewDocument("MDTV-XCAF", document);
    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    shapeTool->AddShape(
        BRepPrimAPI_MakeBox(40.0, 30.0, 6.0).Shape(), Standard_False);

    oap::Model model;
    oap::StepIndexer indexer;
    if (!expect(indexer.build(document, model),
                "box fixture should build a stable STEP index")) {
        return false;
    }

    oap::toolpath::RootOuterWireProfileSource source;
    for (const oap::Face& face : model.faces) {
        const TopoDS_Face* geometry = indexer.faceForId(face.face_id);
        if (!geometry)
            continue;
        BRepAdaptor_Surface surface(*geometry);
        if (surface.GetType() != GeomAbs_Plane)
            continue;
        gp_Dir outward = surface.Plane().Axis().Direction();
        if (geometry->Orientation() == TopAbs_REVERSED)
            outward.Reverse();
        if (outward.Dot(gp::DZ()) > 0.999) {
            source.rootFaceId = face.face_id;
            source.unitId = face.unit_id;
            break;
        }
    }
    for (const oap::Wire& wire : model.wires) {
        if (wire.face_id == source.rootFaceId &&
            wire.wire_role == "outer" && wire.is_closed) {
            source.rootWireId = wire.wire_id;
            for (const std::string& edgeId : wire.edge_ids) {
                const auto edge = std::find_if(
                    model.edges.begin(), model.edges.end(),
                    [&](const oap::Edge& value) {
                        return value.edge_id == edgeId;
                    });
                if (edge == model.edges.end())
                    continue;
                for (const std::string& faceId : edge->face_ids) {
                    if (faceId != source.rootFaceId)
                        source.sourceFaceIds.push_back(faceId);
                }
            }
            break;
        }
    }
    std::sort(source.sourceFaceIds.begin(), source.sourceFaceIds.end());
    source.sourceFaceIds.erase(
        std::unique(source.sourceFaceIds.begin(), source.sourceFaceIds.end()),
        source.sourceFaceIds.end());

    std::string error;
    const std::optional<oap::toolpath::ProfileRegion> region =
        oap::toolpath::ProfileRegionBuilder().buildOne(
            source, indexer, {}, &error);
    if (!expect(region.has_value(),
                "a real box outer wire and its four walls should form a profile region")) {
        std::cerr << "region error: " << error << '\n';
        return false;
    }
    const oap::toolpath::ProfileContourResult result =
        oap::toolpath::ProfileContour().generate(*region, profileParams());
    return expect(
        std::abs(region->depth - 6.0) < 1.0e-6 &&
            region->boundary.points.size() >= 4 &&
            region->boundary.signedArea > 0.0 && result.ok,
        "real OCCT outer-wire geometry should reach executable profile moves");
}

bool buildsSteppedSamplePositiveZProfile()
{
    const std::filesystem::path sample =
        std::filesystem::path(__FILE__).parent_path().parent_path()
            .parent_path().parent_path() /
        "local-data" / "samples" / "step" / "123_stp.stp";
    if (!std::filesystem::exists(sample))
        return true;

    Handle(TDocStd_Document) document;
    oap::StepIndexer indexer;
    oap::Model model;
    const auto analysis = oap::AnalysisPipeline().execute(
        sample.string(), document, indexer, model);
    if (!expect(analysis.ok(), "123_stp sample should load for profile regression"))
        return false;
    oap::ModelIndex modelIndex;
    modelIndex.build(model);
    oap::ThreeAxisWireTraceRequest request;
    request.model = &model;
    request.modelIndex = &modelIndex;
    request.stepIndexer = &indexer;
    const auto recognized = oap::ThreeAxisWireTraceUseCase().execute(request);
    const auto positiveZSource = std::find_if(
        recognized.profileSources.begin(), recognized.profileSources.end(),
        [](const oap::ThreeAxisProfileSource& value) {
            return value.rootFaceId == "F2";
        });
    const auto negativeYSource = std::find_if(
        recognized.profileSources.begin(), recognized.profileSources.end(),
        [](const oap::ThreeAxisProfileSource& value) {
            return value.rootFaceId == "F1";
        });
    if (!expect(positiveZSource != recognized.profileSources.end(),
                "123_stp F2 should be recognized as the +Z profile root")) {
        return false;
    }
    if (!expect(negativeYSource != recognized.profileSources.end(),
                "123_stp F1 should be recognized as the -Y profile root")) {
        return false;
    }
    const auto mapSource = [](const oap::ThreeAxisProfileSource& source) {
        oap::toolpath::RootOuterWireProfileSource mapped;
        mapped.unitId = source.unitId;
        mapped.axisLabel = source.axisLabel;
        mapped.normalDirection = source.normalDirection;
        mapped.rootFaceId = source.rootFaceId;
        mapped.rootWireId = source.rootWireId;
        mapped.sourceFaceIds = source.sourceFaceIds;
        mapped.machiningFaceIds = source.machiningFaceIds;
        mapped.terminalFaceIds = source.terminalFaceIds;
        mapped.mainOuterFaceIds = source.mainOuterFaceIds;
        return mapped;
    };
    std::string error;
    const auto region = oap::toolpath::ProfileRegionBuilder().buildOne(
        mapSource(*positiveZSource), indexer, {}, &error);
    std::string negativeYError;
    const auto negativeYRegion = oap::toolpath::ProfileRegionBuilder().buildOne(
        mapSource(*negativeYSource), indexer, {}, &negativeYError);
    oap::toolpath::ToolpathParams params = profileParams();
    params.tool.diameter = 2.0;
    params.depth.stepDown = 2.0;
    params.profile.radialPassCount = 1;
    const auto contour = region
        ? oap::toolpath::ProfileContour().generate(*region, params)
        : oap::toolpath::ProfileContourResult{};
    auto restParams = params;
    restParams.clearing.restMachiningPreviousToolDiameter = 10.0;
    const auto restContour = region
        ? oap::toolpath::ProfileContour().generate(*region, restParams)
        : oap::toolpath::ProfileContourResult{};
    std::vector<oap::toolpath::ProfileRegion> sampleRegions;
    for (const auto& source : recognized.profileSources) {
        std::string sourceError;
        if (auto built = oap::toolpath::ProfileRegionBuilder().buildOne(
                mapSource(source), indexer, {}, &sourceError)) {
            sampleRegions.push_back(std::move(*built));
        } else {
            std::cerr << "123 profile build " << source.rootFaceId
                      << " failed: " << sourceError << '\n';
        }
    }
    const auto allProfiles = oap::toolpath::ProfileToolpathService().generate(
        sampleRegions, params);

    std::vector<oap::toolpath::RootInnerWirePocketSource> pocketSources;
    pocketSources.reserve(recognized.pocketSources.size());
    for (const oap::ThreeAxisPocketSource& source : recognized.pocketSources) {
        oap::toolpath::RootInnerWirePocketSource mapped;
        mapped.kind = source.kind;
        mapped.sourceFaceIds = source.sourceFaceIds;
        mapped.rootFaceId = source.rootFaceId;
        mapped.rootWireId = source.rootWireId;
        mapped.allowOpenSectionContours = source.allowOpenSectionContours;
        mapped.openBoundaryFaceCount = source.openBoundaryFaceCount;
        pocketSources.push_back(std::move(mapped));
    }
    const auto pocketRegions =
        oap::toolpath::PocketRegionBuilder().buildRootInnerWireRegions(
            pocketSources, indexer);
    oap::toolpath::ToolpathParams freePathParams;
    freePathParams.tool.diameter = 5.0;
    freePathParams.tool.cuttingEdgeLength = 5.0;
    freePathParams.tool.horizFeed = 600.0;
    freePathParams.tool.vertFeed = 200.0;
    freePathParams.depth.safeHeight = 10.0;
    freePathParams.depth.clearanceHeight = 20.0;
    freePathParams.depth.useFeatureDepth = true;
    freePathParams.depth.stepDown = 2.0;
    freePathParams.clearing.pattern =
        oap::toolpath::ClearingPattern::FreePath;
    freePathParams.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::InsideToOutside;
    freePathParams.clearing.entryMode =
        oap::toolpath::PocketEntryMode::LinearRamp;
    freePathParams.clearing.rampAngleDeg = 3.0;
    freePathParams.clearing.stepOverPercent = 50.0;
    freePathParams.clearing.precision = 3;
    freePathParams.clearing.arcTolerance = 0.0;
    const auto freePathBatch =
        oap::toolpath::PocketToolpathService().generate(
            pocketRegions, freePathParams, "123_FREEPATH");
    if (!document.IsNull())
        XCAFApp_Application::GetApplication()->Close(document);
    if (!expect(region.has_value(),
                "123_stp +Z profile should accept stepped/rounded outer features")) {
        std::cerr << "region error: " << error << '\n';
        return false;
    }
    if (!expect(negativeYRegion.has_value() && negativeYRegion->depth < 200.0,
                "123_stp -Y profile depth must follow its outer feature instead of the full 500 mm part span")) {
        std::cerr << "-Y region error: " << negativeYError;
        if (negativeYRegion)
            std::cerr << " depth=" << negativeYRegion->depth;
        std::cerr << '\n';
        return false;
    }
    if (!expect(contour.ok && !contour.loops.empty(),
                "123_stp +Z stepped profile should pass contour generation")) {
        std::cerr << "contour error: " << contour.error << '\n';
        return false;
    }
    if (!expect(restContour.ok && !restContour.loops.empty(),
                "123_stp +Z D10-to-D2 rest profile should pass contour generation")) {
        std::cerr << "rest contour error: " << restContour.error << '\n';
        return false;
    }
    const auto completesSeparatedAreasPerDepth = [](const auto& generated) {
        int depth = -1;
        int previousArea = -1;
        int transitions = 0;
        int areaMask = 0;
        bool observedBothAreas = false;
        for (const auto& loop : generated.loops) {
            if (loop.passKind != oap::toolpath::ProfilePassKind::Clearing ||
                loop.points.empty()) {
                continue;
            }
            if (loop.depthIndex != depth) {
                if (depth >= 0 && areaMask == 3) {
                    observedBothAreas = true;
                    if (transitions > 1)
                        return false;
                }
                depth = loop.depthIndex;
                previousArea = -1;
                transitions = 0;
                areaMask = 0;
            }
            double meanY = 0.0;
            for (const auto& point : loop.points)
                meanY += point.Y();
            meanY /= static_cast<double>(loop.points.size());
            if (meanY >= 150.0 && meanY <= 250.0)
                continue;
            const int area = meanY < 150.0 ? 0 : 1;
            areaMask |= 1 << area;
            if (previousArea >= 0 && area != previousArea)
                ++transitions;
            previousArea = area;
        }
        if (depth >= 0 && areaMask == 3) {
            observedBothAreas = true;
            if (transitions > 1)
                return false;
        }
        return observedBothAreas;
    };
    if (!expect(
            completesSeparatedAreasPerDepth(contour) &&
                completesSeparatedAreasPerDepth(restContour),
            "123_stp +Z full and D10-to-D2 rest clearing must finish one separated area per depth before travelling")) {
        return false;
    }
    if (!expect(
            sampleRegions.size() == 6 &&
                allProfiles.operations.size() == sampleRegions.size() &&
                allProfiles.failures.empty(),
            "123_stp should retain all six setup-direction profile operations with a 2 mm cutter")) {
        for (const auto& failure : allProfiles.failures) {
            const auto& failedRegion = sampleRegions.at(
                static_cast<std::size_t>(failure.regionIndex));
            std::cerr << "profile generation "
                      << failedRegion.source.rootFaceId << " axis="
                      << failedRegion.source.axisLabel << " normal="
                      << failedRegion.source.normalDirection << " failed: "
                      << failure.contour.error << '\n';
        }
        return false;
    }
    if (!expect(
            pocketRegions.size() == recognized.pocketSources.size() &&
                freePathBatch.operations.size() == pocketRegions.size() &&
                freePathBatch.failures.empty() &&
                freePathBatch.safetyValidation.errorCount == 0,
            "123_stp D5 FreePath must clear every pocket without crossing protected material")) {
        for (const auto& issue : freePathBatch.safetyValidation.issues)
            std::cerr << issue.code << ": " << issue.message << '\n';
        return false;
    }
    return expect(
        region->useMaterialSectionBoundaryAtEveryDepth &&
            region->depth < 50.0 &&
            std::all_of(
                contour.loops.begin(), contour.loops.end(),
                [](const oap::toolpath::ProfileLoop& loop) {
                    return loop.materialBoundaries &&
                        !loop.materialBoundaries->empty();
                }),
        "123_stp +Z profile must use per-layer material sections over the local outer-feature depth");
}

bool separatesOppositeDieProfileStructures()
{
    const std::filesystem::path sample =
        std::filesystem::path(__FILE__).parent_path().parent_path()
            .parent_path().parent_path() /
        "local-data" / "samples" / "step" /
        "DIE-001_M250219-P3.stp";
    if (!std::filesystem::exists(sample))
        return true;

    Handle(TDocStd_Document) document;
    oap::StepIndexer indexer;
    oap::Model model;
    const auto analysis = oap::AnalysisPipeline().execute(
        sample.string(), document, indexer, model);
    if (!expect(analysis.ok(), "DIE sample should load for profile regression"))
        return false;
    oap::ModelIndex modelIndex;
    modelIndex.build(model);
    oap::ThreeAxisWireTraceRequest request;
    request.model = &model;
    request.modelIndex = &modelIndex;
    request.stepIndexer = &indexer;
    const auto recognized = oap::ThreeAxisWireTraceUseCase().execute(request);
    const auto findRoot = [&](const char* id) {
        return std::find_if(
            recognized.profileSources.begin(), recognized.profileSources.end(),
            [&](const oap::ThreeAxisProfileSource& source) {
                return source.rootFaceId == id;
            });
    };
    const auto f5 = findRoot("F5");
    const auto f17 = findRoot("F17");
    if (!expect(f5 != recognized.profileSources.end() &&
                    f17 != recognized.profileSources.end(),
                "DIE F5 and F17 should remain separate Z profile roots")) {
        return false;
    }
    const auto mapSource = [](const oap::ThreeAxisProfileSource& source) {
        oap::toolpath::RootOuterWireProfileSource mapped;
        mapped.unitId = source.unitId;
        mapped.axisLabel = source.axisLabel;
        mapped.normalDirection = source.normalDirection;
        mapped.rootFaceId = source.rootFaceId;
        mapped.rootWireId = source.rootWireId;
        mapped.sourceFaceIds = source.sourceFaceIds;
        mapped.machiningFaceIds = source.machiningFaceIds;
        mapped.terminalFaceIds = source.terminalFaceIds;
        mapped.mainOuterFaceIds = source.mainOuterFaceIds;
        return mapped;
    };
    std::string f5Error;
    std::string f17Error;
    const auto f5Region = oap::toolpath::ProfileRegionBuilder().buildOne(
        mapSource(*f5), indexer, {}, &f5Error);
    const auto f17Region = oap::toolpath::ProfileRegionBuilder().buildOne(
        mapSource(*f17), indexer, {}, &f17Error);
    oap::toolpath::ToolpathParams params = profileParams();
    params.tool.diameter = 6.0;
    params.depth.stepDown = 2.0;
    params.profile.radialPassCount = 1;
    const auto f5Contour = f5Region
        ? oap::toolpath::ProfileContour().generate(*f5Region, params)
        : oap::toolpath::ProfileContourResult{};
    const auto f17Contour = f17Region
        ? oap::toolpath::ProfileContour().generate(*f17Region, params)
        : oap::toolpath::ProfileContourResult{};
    const auto f17Batch = f17Region
        ? oap::toolpath::ProfileToolpathService().generate({*f17Region}, params)
        : oap::toolpath::ProfileToolpathBatch{};
    std::vector<oap::toolpath::ProfileRegion> allRegions;
    for (const auto& source : recognized.profileSources) {
        std::string buildError;
        if (auto value = oap::toolpath::ProfileRegionBuilder().buildOne(
                mapSource(source), indexer, {}, &buildError)) {
            allRegions.push_back(std::move(*value));
        } else {
            std::cerr << "DIE skip " << source.rootFaceId << ": "
                      << buildError << '\n';
        }
    }
    const auto allBatch = oap::toolpath::ProfileToolpathService().generate(
        allRegions, params);
    auto diameterTenParams = params;
    diameterTenParams.tool.diameter = 10.0;
    diameterTenParams.tool.cuttingEdgeLength = 5.0;
    const auto diameterTenBatch =
        oap::toolpath::ProfileToolpathService().generate(
            allRegions, diameterTenParams);
    // 复现桌面端自动选刀的最小刀阶段。F4（X-）含离散曲线，历史上
    // 全量清料偏置与安全验证边界相差约 0.02~0.04 mm，导致所有刀径
    // 都无法建立自动规划的起始状态。
    auto diameterTwoParams = params;
    diameterTwoParams.tool.diameter = 2.0;
    diameterTwoParams.tool.cuttingEdgeLength = 6.0;
    diameterTwoParams.depth.safeHeight = 10.0;
    diameterTwoParams.depth.clearanceHeight = 20.0;
    diameterTwoParams.profile.roughingAllowance = 0.2;
    diameterTwoParams.profile.finishPass = false;
    diameterTwoParams.clearing.pattern =
        oap::toolpath::ClearingPattern::Offset;
    diameterTwoParams.clearing.stepOverPercent = 50.0;
    const auto diameterTwoBatch =
        oap::toolpath::ProfileToolpathService().generate(
            allRegions, diameterTwoParams);
    if (!diameterTwoBatch.failures.empty()) {
        for (const auto& failure : diameterTwoBatch.failures) {
            const auto& failedRegion = allRegions.at(
                static_cast<std::size_t>(failure.regionIndex));
            std::cerr << "DIE D2 regression "
                      << failedRegion.source.rootFaceId << " failed: "
                      << failure.contour.error << '\n';
            const auto diagnostic = std::find_if(
                diameterTwoBatch.diagnostics.begin(),
                diameterTwoBatch.diagnostics.end(),
                [&failure](const auto& item) {
                    return item.sourceIndex == failure.regionIndex &&
                        item.severity ==
                            oap::toolpath::ToolpathDiagnosticSeverity::Error;
                });
            if (diagnostic != diameterTwoBatch.diagnostics.end() &&
                diagnostic->moveIndex >= 0 &&
                diagnostic->moveIndex < static_cast<int>(
                    failure.contour.moves.size())) {
                const auto& move = failure.contour.moves.at(
                    static_cast<std::size_t>(diagnostic->moveIndex));
                std::cerr << "  depth=" << diagnostic->depthIndex
                          << ", loop=" << move.loopSequence
                          << ", ring=" << move.ring
                          << ", from=(" << move.from.X() << ','
                          << move.from.Y() << ',' << move.from.Z()
                          << "), to=(" << move.to.X() << ','
                          << move.to.Y() << ',' << move.to.Z() << ")\n";
            }
        }
    }
    const bool f44ExcludedFromF17 = std::find(
        f17->sourceFaceIds.begin(), f17->sourceFaceIds.end(), "F44") ==
        f17->sourceFaceIds.end();
    const bool f5OwnsConnectedF44Step =
        std::find(
            f5->sourceFaceIds.begin(), f5->sourceFaceIds.end(), "F42") !=
            f5->sourceFaceIds.end() &&
        std::find(
            f5->machiningFaceIds.begin(), f5->machiningFaceIds.end(), "F44") !=
            f5->machiningFaceIds.end();
    const oap::MoldCoordinateFrame moldFrame =
        oap::MoldCoordinateFrame::fromModel(model);
    const bool setupDirectionsMatchExpectedRoots =
        f5Region && f17Region &&
        oap::classifyMoldSetupFace(
            moldFrame, gp_Vec(f17Region->openDir)) ==
            oap::MoldSetupFace::PositiveZ &&
        oap::classifyMoldSetupFace(
            moldFrame, gp_Vec(f5Region->openDir)) ==
            oap::MoldSetupFace::NegativeZ;
    const bool f17HasContinuousDisplaySegments = std::all_of(
        f17Contour.loops.begin(), f17Contour.loops.end(),
        [&](const oap::toolpath::ProfileLoop& loop) {
            if (loop.passKind != oap::toolpath::ProfilePassKind::Finish)
                return true;
            if (loop.world.size() < 3)
                return false;
            for (std::size_t index = 0; index < loop.world.size(); ++index) {
                if (loop.world[index].Distance(
                        loop.world[(index + 1) % loop.world.size()]) >
                    params.tool.diameter + 1.0e-6) {
                    return false;
                }
            }
            return true;
        });
    const auto usesCompleteMonotonicEnvelope = [&params](
        const auto& contour, const auto& region) {
        const int expectedLayerCount = static_cast<int>(
            std::ceil(region.depth / params.depth.stepDown));
        if (contour.depthLayerCount != expectedLayerCount ||
            contour.loops.empty() ||
            std::abs(contour.loops.back().depthOffset) > 1.0e-7) {
            return false;
        }
        double previousArea = 0.0;
        for (const oap::toolpath::ProfileLoop& loop : contour.loops) {
            if (loop.passKind != oap::toolpath::ProfilePassKind::Finish)
                continue;
            if (loop.area + 1.0e-2 < previousArea)
                return false;
            previousArea = loop.area;
        }
        return true;
    };
    const auto containsARealSectionContraction = [](const auto& contour) {
        const auto materialArea = [](const auto& points) {
            double twiceArea = 0.0;
            for (std::size_t index = 0; index < points.size(); ++index) {
                const auto& from = points[index];
                const auto& to = points[(index + 1) % points.size()];
                twiceArea += from.X() * to.Y() - to.X() * from.Y();
            }
            return std::abs(twiceArea) * 0.5;
        };
        double previousArea = -1.0;
        for (const oap::toolpath::ProfileLoop& loop : contour.loops) {
            if (!loop.materialBoundaries)
                return false;
            double currentArea = 0.0;
            for (const auto& boundary : *loop.materialBoundaries)
                currentArea += materialArea(boundary);
            if (previousArea >= 0.0 && currentArea + 1.0e-3 < previousArea)
                return true;
            previousArea = currentArea;
        }
        return false;
    };
    const auto generatedRoot = [](const auto& batch, const char* root) {
        return std::find_if(
                   batch.operations.begin(), batch.operations.end(),
                   [root](const oap::toolpath::ProfileToolpathOperation& op) {
                       return op.rootFaceId == root;
                   }) != batch.operations.end();
    };
    const bool ok = expect(
        f5Region && f17Region && f5Contour.ok && f17Contour.ok &&
            f17Batch.operations.size() == 1 && f17Batch.failures.empty() &&
            allBatch.operations.size() >= 4 &&
            generatedRoot(allBatch, "F5") &&
            generatedRoot(allBatch, "F17") &&
            generatedRoot(allBatch, "F19") &&
            generatedRoot(allBatch, "F24") &&
            generatedRoot(diameterTenBatch, "F5") &&
            generatedRoot(diameterTenBatch, "F17") &&
            generatedRoot(diameterTenBatch, "F19") &&
            generatedRoot(diameterTenBatch, "F24") &&
            diameterTwoBatch.operations.size() == allRegions.size() &&
            diameterTwoBatch.failures.empty() &&
            setupDirectionsMatchExpectedRoots &&
            f5->normalDirection == 1 && f17->normalDirection == -1 &&
            f44ExcludedFromF17 &&
            f5OwnsConnectedF44Step &&
            f5Region->depth > 5.4 && f5Region->depth < 6.0 &&
            f17HasContinuousDisplaySegments &&
            usesCompleteMonotonicEnvelope(f17Contour, *f17Region) &&
            usesCompleteMonotonicEnvelope(f5Contour, *f5Region) &&
            containsARealSectionContraction(f17Contour) &&
            std::all_of(
                f17Contour.loops.begin(), f17Contour.loops.end(),
                [](const oap::toolpath::ProfileLoop& loop) {
                    return loop.materialBoundaries &&
                        !loop.materialBoundaries->empty();
                }),
        "DIE opposite Z profiles must retain the cumulative maximum envelope through contracting sections");
    if (!document.IsNull())
        XCAFApp_Application::GetApplication()->Close(document);
    return ok;
}

// Both bosses belong to one solid, but their horizontal section has two outer
// components. Every profile root must protect both components, including links.
bool preservesDisconnectedBosses()
{
    const TopoDS_Shape base = BRepPrimAPI_MakeBox(80.0, 40.0, 10.0).Shape();
    const TopoDS_Shape left = BRepPrimAPI_MakeBox(
        gp_Pnt(5.0, 5.0, 10.0), 20.0, 30.0, 8.0).Shape();
    const TopoDS_Shape right = BRepPrimAPI_MakeBox(
        gp_Pnt(45.0, 5.0, 10.0), 25.0, 30.0, 8.0).Shape();
    const TopoDS_Shape solid = BRepAlgoAPI_Fuse(
        BRepAlgoAPI_Fuse(base, left).Shape(), right).Shape();
    oap::toolpath::ProfileRegion region = rectangularProfile();
    region.frame = gp_Ax3(gp_Pnt(0, 0, 10), gp::DZ(), gp::DX());
    region.depth = 8.0;
    region.protectedMaterialShape = solid;
    BRep_Builder builder;
    builder.MakeCompound(region.sectionShape);
    builder.Add(region.sectionShape, solid);
    region.useMaterialSectionBoundaryAtEveryDepth = true;
    region.stockBoundary.points = {{0, 0}, {80, 0}, {80, 40}, {0, 40}};
    region.stockBoundary.closed = true;
    region.stockBoundary.signedArea = 3200;
    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 4;
    params.tool.cuttingEdgeLength = 20;
    params.tool.overallLength = 50;
    params.depth.stepDown = 2;
    BRepClass3d_SolidClassifier classifier(solid);
    for (int root = 0; root < 2; ++root) {
        const double x0 = root == 0 ? 5 : 45;
        const double x1 = root == 0 ? 25 : 70;
        region.boundary.points = {{x0, 5}, {x1, 5}, {x1, 35}, {x0, 35}};
        region.boundary.signedArea = (x1 - x0) * 30;
        auto contour = oap::toolpath::ProfileContour().generate(region, params);
        if (!expect(contour.ok && !contour.moves.empty(),
                    "disconnected bosses must produce executable clearing"))
            return false;
        for (const auto& move : contour.moves) {
            if (move.type != oap::toolpath::ToolpathMoveType::Cut &&
                move.type != oap::toolpath::ToolpathMoveType::Plunge)
                continue;
            const int samples = std::max(1, static_cast<int>(
                std::ceil(move.from.Distance(move.to))));
            for (int index = 0; index <= samples; ++index) {
                const double t = static_cast<double>(index) / samples;
                const gp_Pnt center(
                    move.from.X() + t * (move.to.X() - move.from.X()),
                    move.from.Y() + t * (move.to.Y() - move.from.Y()),
                    move.from.Z() + t * (move.to.Z() - move.from.Z()) + 0.1);
                for (int radial = 0; radial < 17; ++radial) {
                    const double angle = radial * 2.0 * 3.141592653589793 / 16;
                    const double radius = radial == 16 ? 0 : params.tool.radius() - 0.02;
                    const gp_Pnt point(center.X() + radius * std::cos(angle),
                                       center.Y() + radius * std::sin(angle), center.Z());
                    classifier.Perform(point, 1.0e-7);
                    if (!expect(classifier.State() != TopAbs_IN,
                                "cutting and linking sweeps must preserve both solid bosses"))
                        return false;
                }
            }
        }
        oap::toolpath::ProfileToolpathOperation operation;
        operation.operationId = "two-bosses";
        operation.outwardToolAxis = gp_Vec(region.openDir);
        operation.contour = contour;
        if (!expect(oap::toolpath::ProfileSafetyValidator().validate(
                        region, operation, params).report.ok(),
                    "safe multi-component profile must pass validation"))
            return false;
        // Inject a crossing through the sibling boss. The checker must inspect
        // the full protected section even though this is another root's move.
        auto crossing = *std::find_if(contour.moves.begin(), contour.moves.end(),
            [](const auto& move) { return move.type == oap::toolpath::ToolpathMoveType::Cut; });
        const double siblingX = root == 0 ? 55 : 15;
        crossing.from = gp_Pnt(siblingX, 0, 16);
        crossing.to = gp_Pnt(siblingX, 40, 16);
        operation.contour.moves = {crossing};
        if (!expect(!oap::toolpath::ProfileSafetyValidator().validate(
                        region, operation, params).report.ok(),
                    "validator must reject crossing a non-root boss"))
            return false;
    }
    return true;
}

// Optional real-world fixture, supplied locally without committing customer CAD.
bool validatesExternalProfileSample(
    const char* sample = std::getenv("OAP_PROFILE_REGRESSION_STEP"))
{
    if (!sample || !*sample)
        return true;
    Handle(TDocStd_Document) document;
    oap::StepIndexer indexer;
    oap::Model model;
    if (!expect(oap::AnalysisPipeline().execute(
                    sample, document, indexer, model).ok(),
                "external profile regression sample must load"))
        return false;
    oap::ModelIndex modelIndex;
    modelIndex.build(model);
    oap::ThreeAxisWireTraceRequest request;
    request.model = &model;
    request.modelIndex = &modelIndex;
    request.stepIndexer = &indexer;
    const auto recognized = oap::ThreeAxisWireTraceUseCase().execute(request);
    int checked = 0;
    for (const auto& source : recognized.profileSources) {
        if (source.axisLabel != "Z" || source.normalDirection != 1)
            continue;
        oap::toolpath::RootOuterWireProfileSource mapped;
        mapped.unitId = source.unitId;
        mapped.axisLabel = source.axisLabel;
        mapped.normalDirection = source.normalDirection;
        mapped.rootFaceId = source.rootFaceId;
        mapped.rootWireId = source.rootWireId;
        mapped.sourceFaceIds = source.sourceFaceIds;
        mapped.machiningFaceIds = source.machiningFaceIds;
        mapped.terminalFaceIds = source.terminalFaceIds;
        mapped.mainOuterFaceIds = source.mainOuterFaceIds;
        auto region = oap::toolpath::ProfileRegionBuilder().buildOne(mapped, indexer);
        if (!expect(region.has_value(), "recognized top profile must build"))
            return false;
        oap::toolpath::ToolpathParams params;
        params.tool.diameter = 6;
        params.tool.cuttingEdgeLength = 100;
        params.tool.overallLength = 150;
        params.depth.stepDown = 2;
        params.clearing.stepOverPercent = 50;
        const auto batch = oap::toolpath::ProfileToolpathService().generate(
            {*region}, params);
        if (!expect(batch.operations.size() == 1 && batch.failures.empty() &&
                        batch.safetyValidation.ok(),
                    "real top profile must generate and pass safety checks"))
            return false;
        BRepClass3d_SolidClassifier classifier(region->protectedMaterialShape);
        int checkedPoints = 0;
        for (const auto& move : batch.operations.front().contour.moves) {
            if (move.type != oap::toolpath::ToolpathMoveType::Cut)
                continue;
            const int samples = std::max(1, static_cast<int>(
                std::ceil(move.from.Distance(move.to))));
            for (int index = 0; index <= samples; ++index) {
                const double t = static_cast<double>(index) / samples;
                gp_Pnt point(move.from.X() + t * (move.to.X() - move.from.X()),
                             move.from.Y() + t * (move.to.Y() - move.from.Y()),
                             move.from.Z() + t * (move.to.Z() - move.from.Z()));
                point.Translate(gp_Vec(region->openDir) * 0.1);
                classifier.Perform(point, 1.0e-7);
                if (!expect(classifier.State() != TopAbs_IN,
                            "real profile cutting paths must stay outside the finished solid"))
                    return false;
                ++checkedPoints;
            }
        }
        std::cout << "real profile root=" << source.rootFaceId
                  << " checked cutting points=" << checkedPoints
                  << " intrusions=0 generation_ms=" << batch.metrics.totalMilliseconds
                  << " layer_workers=" << batch.metrics.layerPeakParallelism << '\n';
        ++checked;
    }
    if (!document.IsNull())
        XCAFApp_Application::GetApplication()->Close(document);
    return expect(checked > 0, "sample must exercise positive Z profiles");
}

} // namespace

int main(int argc, char** argv)
{
    if (argc == 3 && std::string_view(argv[1]) == "--profile-sample")
        return validatesExternalProfileSample(argv[2]) ? 0 : 1;
    if (!preservesDisconnectedBosses() || !followsFreeCadProfileSemantics() || !reversesCutDirectionOnly() ||
        !linksSafeSameDepthProfilePasses() ||
        !parallelizesIndependentProfileRegions() ||
        !parallelizesIndependentDepthLayerPatterns() ||
        !clearsStockAcrossOpenProfileNotch() ||
        !completesDisconnectedClearingAreaBeforeTravel() ||
        !completesSplitClearingBranchBeforeSibling() ||
        !limitsSmallCutterToPreviousToolRestMaterial() ||
        !validatesFinishedPartAndToolReach() ||
        !buildsRealOuterWireRegion() ||
        !separatesOppositeDieProfileStructures() ||
        !buildsSteppedSamplePositiveZProfile() ||
        !validatesExternalProfileSample()) {
        return 1;
    }
    std::cout << "profile contour smoke tests passed\n";
    return 0;
}

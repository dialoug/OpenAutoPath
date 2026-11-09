#include "toolpath/core/ToolpathParamNormalizer.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/pocket/planning/CutterCenterClearingPattern.h"
#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/pocket/validation/PocketSafetyValidator.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"

#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <map>
#include <string_view>

namespace {

constexpr double kTolerance = 1.0e-7;

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

oap::toolpath::PocketRegion rectangularPocket()
{
    oap::toolpath::PocketRegion region;
    region.kind = oap::FeatureKind::Pocket;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0),
        region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.depth = 4.0;
    region.boundary.closed = true;
    region.boundary.points = {
        gp_Pnt2d(0.0, 0.0),
        gp_Pnt2d(40.0, 0.0),
        gp_Pnt2d(40.0, 30.0),
        gp_Pnt2d(0.0, 30.0)};
    region.boundary.signedArea = 1200.0;
    return region;
}

double polygonArea(const std::vector<gp_Pnt2d>& points)
{
    double twiceArea = 0.0;
    for (std::size_t index = 0u; index < points.size(); ++index) {
        const gp_Pnt2d& first = points[index];
        const gp_Pnt2d& second = points[(index + 1u) % points.size()];
        twiceArea += first.X() * second.Y() - second.X() * first.Y();
    }
    return 0.5 * twiceArea;
}

oap::toolpath::PocketRegion pocketRegion(
    std::vector<gp_Pnt2d> boundary,
    std::vector<std::vector<gp_Pnt2d>> islands = {})
{
    oap::toolpath::PocketRegion region;
    region.kind = oap::FeatureKind::Pocket;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(
        gp_Pnt(0.0, 0.0, 0.0), region.openDir,
        gp_Dir(1.0, 0.0, 0.0));
    region.depth = 1.0;
    region.boundary.closed = true;
    region.boundary.points = std::move(boundary);
    region.boundary.signedArea = polygonArea(region.boundary.points);
    for (std::vector<gp_Pnt2d>& points : islands) {
        oap::toolpath::Loop2d island;
        island.closed = true;
        island.points = std::move(points);
        island.signedArea = polygonArea(island.points);
        region.islands.push_back(std::move(island));
    }
    return region;
}

bool pointInPolygon(const gp_Pnt2d& point,
                    const std::vector<gp_Pnt2d>& polygon)
{
    bool inside = false;
    for (std::size_t current = 0u, previous = polygon.size() - 1u;
         current < polygon.size(); previous = current++) {
        const gp_Pnt2d& a = polygon[current];
        const gp_Pnt2d& b = polygon[previous];
        if ((a.Y() > point.Y()) != (b.Y() > point.Y()) &&
            point.X() < (b.X() - a.X()) * (point.Y() - a.Y()) /
                    (b.Y() - a.Y()) + a.X()) {
            inside = !inside;
        }
    }
    return inside;
}

oap::toolpath::ToolpathParams offsetRampParams()
{
    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 4.0;
    params.tool.horizFeed = 600.0;
    params.tool.vertFeed = 200.0;
    params.depth.safeHeight = 2.0;
    params.depth.clearanceHeight = 4.0;
    params.depth.useFeatureDepth = true;
    params.depth.stepDown = 2.0;
    params.clearing.pattern = oap::toolpath::ClearingPattern::Offset;
    params.clearing.entryMode =
        oap::toolpath::PocketEntryMode::RampOnShape;
    params.clearing.rampAngleDeg = 10.0;
    params.clearing.stepOverPercent = 50.0;
    return params;
}

bool hasCutLink(
    const oap::toolpath::PocketClearingResult& result,
    const oap::toolpath::ClearingLoop& previous,
    const oap::toolpath::ClearingLoop& next)
{
    const gp_Pnt& previousEnd =
        previous.closed ? previous.world.front() : previous.world.back();
    const gp_Pnt& nextStart = next.world.front();
    return std::any_of(
        result.moves.begin(), result.moves.end(),
        [&](const oap::toolpath::ToolpathMove& move) {
            return move.type == oap::toolpath::ToolpathMoveType::Cut &&
                   move.loopSequence == next.sequence &&
                   move.from.Distance(previousEnd) <= kTolerance &&
                   move.to.Distance(nextStart) <= kTolerance;
        });
}

bool samePlanarPoints(const std::vector<gp_Pnt2d>& left,
                      const std::vector<gp_Pnt2d>& right)
{
    if (left.size() != right.size())
        return false;
    for (std::size_t index = 0u; index < left.size(); ++index) {
        if (left[index].Distance(right[index]) > kTolerance)
            return false;
    }
    return true;
}

bool sameFreePathRuns(
    const std::vector<oap::toolpath::ClearingLoop>& left,
    const std::vector<oap::toolpath::ClearingLoop>& right)
{
    if (left.size() != right.size())
        return false;
    for (std::size_t index = 0u; index < left.size(); ++index) {
        if (left[index].closed != right[index].closed ||
            left[index].isHole != right[index].isHole ||
            left[index].ring != right[index].ring ||
            !samePlanarPoints(left[index].points, right[index].points) ||
            !samePlanarPoints(
                left[index].preferredLinkFromPrevious,
                right[index].preferredLinkFromPrevious)) {
            return false;
        }
    }
    return true;
}

bool parallelOffsetRingsPreserveDeterministicOutput()
{
    const auto expectedParallelism = std::min<std::size_t>(
        4u, oap::toolpath::toolpathHardwareParallelism());
    const bool hasParallelWorkers = expectedParallelism > 1u;
    const oap::toolpath::PocketRegion region = rectangularPocket();
    oap::toolpath::ToolpathParams params = offsetRampParams();
    params.depth.stepDown = region.depth;

    oap::toolpath::ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers =
        [](std::size_t) { return std::size_t{0u}; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    const oap::toolpath::PocketClearingResult serial =
        oap::toolpath::PocketClearing().generate(
            region, params, &serialControl);

    std::atomic_size_t acquired{0u};
    std::atomic_size_t released{0u};
    oap::toolpath::ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers =
        [&acquired](std::size_t requested) {
            const std::size_t granted = std::min<std::size_t>(3u, requested);
            acquired.fetch_add(granted, std::memory_order_relaxed);
            return granted;
        };
    parallelControl.releaseParallelWorkers =
        [&released](std::size_t count) {
            released.fetch_add(count, std::memory_order_relaxed);
        };
    const oap::toolpath::PocketClearingResult parallel =
        oap::toolpath::PocketClearing().generate(
            region, params, &parallelControl);

    if (!expect(serial.ok && parallel.ok,
                "serial and parallel offset generation should succeed") ||
        !expect(!serial.usedParallelPatternProcessing &&
                    parallel.usedParallelPatternProcessing == hasParallelWorkers,
                "only the leased offset run should report ring parallelism") ||
        !expect(parallel.patternPeakParallelism == expectedParallelism &&
                    (parallel.patternParallelBatchCount > 0u) == hasParallelWorkers &&
                    (parallel.patternParallelTaskCount > 0u) == hasParallelWorkers,
                "offset generation should honor hardware and granted worker limits") ||
        !expect(serial.travelChoiceParallelBatchCount == 0u &&
                    (parallel.travelChoiceParallelBatchCount > 0u) == hasParallelWorkers &&
                    parallel.travelChoicePeakParallelism == expectedParallelism,
                "adjacent loop links should use the shared worker budget") ||
        !expect(acquired.load(std::memory_order_relaxed) ==
                    released.load(std::memory_order_relaxed),
                "offset ring workers should return every borrowed token") ||
        !expect(serial.ringCount == parallel.ringCount &&
                    serial.loops.size() == parallel.loops.size() &&
                    serial.moves.size() == parallel.moves.size() &&
                    serial.primitives.size() == parallel.primitives.size(),
                "parallel offset generation should preserve result sizes")) {
        return false;
    }

    for (std::size_t loopIndex = 0u;
         loopIndex < serial.loops.size(); ++loopIndex) {
        const oap::toolpath::ClearingLoop& left = serial.loops[loopIndex];
        const oap::toolpath::ClearingLoop& right = parallel.loops[loopIndex];
        if (!expect(left.ring == right.ring &&
                        left.closed == right.closed &&
                        left.isHole == right.isHole &&
                        left.points.size() == right.points.size(),
                    "parallel offset generation should preserve loop metadata")) {
            return false;
        }
        for (std::size_t pointIndex = 0u;
             pointIndex < left.points.size(); ++pointIndex) {
            if (!expect(
                    left.points[pointIndex].Distance(
                        right.points[pointIndex]) <= kTolerance,
                    "parallel offset generation should preserve point order")) {
                return false;
            }
        }
    }
    for (std::size_t moveIndex = 0u;
         moveIndex < serial.moves.size(); ++moveIndex) {
        const auto& left = serial.moves[moveIndex];
        const auto& right = parallel.moves[moveIndex];
        if (!expect(
                left.type == right.type &&
                    left.intent == right.intent &&
                    left.loopSequence == right.loopSequence &&
                    left.depthIndex == right.depthIndex &&
                    left.ring == right.ring &&
                    left.from.Distance(right.from) <= kTolerance &&
                    left.to.Distance(right.to) <= kTolerance,
                "parallel move preparation should preserve move order")) {
            return false;
        }
    }
    return true;
}

bool parallelZigZagScanLinesPreserveDeterministicOutput()
{
    const auto expectedParallelism = std::min<std::size_t>(
        4u, oap::toolpath::toolpathHardwareParallelism());
    const oap::toolpath::PocketRegion region = rectangularPocket();
    oap::toolpath::ToolpathParams params = offsetRampParams();
    params.depth.stepDown = region.depth;
    params.clearing.pattern = oap::toolpath::ClearingPattern::ZigZag;
    params.clearing.perimeterPass = true;

    oap::toolpath::ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers =
        [](std::size_t) { return std::size_t{0u}; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    const oap::toolpath::PocketClearingResult serial =
        oap::toolpath::PocketClearing().generate(
            region, params, &serialControl);

    oap::toolpath::ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers =
        [](std::size_t requested) {
            return std::min<std::size_t>(3u, requested);
        };
    parallelControl.releaseParallelWorkers = [](std::size_t) {};
    const oap::toolpath::PocketClearingResult parallel =
        oap::toolpath::PocketClearing().generate(
            region, params, &parallelControl);

    if (!expect(serial.ok && parallel.ok &&
                    parallel.usedParallelPatternProcessing == (expectedParallelism > 1u) &&
                    parallel.patternPeakParallelism == expectedParallelism,
                "zigzag scan lines should honor hardware and granted worker limits") ||
        !expect(serial.loops.size() == parallel.loops.size() &&
                    serial.moves.size() == parallel.moves.size() &&
                    serial.primitives.size() == parallel.primitives.size(),
                "parallel zigzag generation should preserve result sizes")) {
        return false;
    }
    for (std::size_t loopIndex = 0u;
         loopIndex < serial.loops.size(); ++loopIndex) {
        const auto& left = serial.loops[loopIndex];
        const auto& right = parallel.loops[loopIndex];
        if (!expect(left.closed == right.closed &&
                        left.points.size() == right.points.size(),
                    "parallel zigzag generation should preserve loop metadata")) {
            return false;
        }
        for (std::size_t pointIndex = 0u;
             pointIndex < left.points.size(); ++pointIndex) {
            if (!expect(
                    left.points[pointIndex].Distance(
                        right.points[pointIndex]) <= kTolerance,
                    "parallel zigzag generation should preserve point order")) {
                return false;
            }
        }
    }
    return true;
}

bool linksEveryOffsetRingWithinEachDepthLayer()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    const oap::toolpath::ToolpathParams params = offsetRampParams();
    const oap::toolpath::PocketClearingResult result =
        oap::toolpath::PocketClearing().generate(region, params);
    if (!expect(result.ok, "the rectangular pocket should generate") ||
        !expect(result.depthLayerCount == 2,
                "the fixture should contain two depth layers") ||
        !expect(
            result.travelReorderedLoopCount == 0u &&
                result.travelAdjustedStartCount == 0u,
            "ordinary full clearing should retain its pattern order")) {
        if (!result.error.empty())
            std::cerr << result.error << '\n';
        return false;
    }

    std::map<int, int> loopCountByDepth;
    for (const oap::toolpath::ClearingLoop& loop : result.loops)
        ++loopCountByDepth[loop.depthIndex];
    if (!expect(
            std::all_of(
                loopCountByDepth.begin(), loopCountByDepth.end(),
                [](const auto& item) { return item.second > 1; }),
            "each depth layer should contain multiple offset rings")) {
        return false;
    }

    const int entryCount =
        result.verticalEntryCount +
        result.rampOnShapeEntryCount +
        result.linearRampEntryCount +
        result.zigzagRampEntryCount +
        result.helicalRampEntryCount;
    if (!expect(entryCount == result.depthLayerCount,
                "each layer should contain exactly one entry") ||
        !expect(result.rampOnShapeEntryCount == result.depthLayerCount,
                "each layer entry should remain a ramp on shape")) {
        return false;
    }

    int expectedLinks = 0;
    int actualLinks = 0;
    for (std::size_t index = 1; index < result.loops.size(); ++index) {
        const oap::toolpath::ClearingLoop& previous =
            result.loops[index - 1];
        const oap::toolpath::ClearingLoop& next = result.loops[index];
        if (previous.depthIndex != next.depthIndex)
            continue;
        ++expectedLinks;
        if (hasCutLink(result, previous, next))
            ++actualLinks;
    }
    if (!expect(expectedLinks > 0 && actualLinks == expectedLinks,
                "same-depth offset rings should have direct cut links")) {
        return false;
    }

    int retractGroups = 0;
    bool previousWasRetract = false;
    for (const oap::toolpath::ToolpathMove& move : result.moves) {
        const bool retract =
            move.type == oap::toolpath::ToolpathMoveType::Retract;
        if (retract && !previousWasRetract)
            ++retractGroups;
        previousWasRetract = retract;
    }
    if (!expect(retractGroups == result.depthLayerCount,
                "each layer should retract only once")) {
        return false;
    }

    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "pocket-layer-linking";
    operation.clearing = result;
    const oap::toolpath::PocketSafetyValidationResult validation =
        oap::toolpath::PocketSafetyValidator().validate(
            region, operation, params);
    return expect(validation.report.errorCount == 0,
                  "direct ring links should remain inside the safe area");
}

bool generatesNxStyleStraightAndZigzagEntries()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();

    oap::toolpath::ToolpathParams straightParams = offsetRampParams();
    straightParams.clearing.entryMode =
        oap::toolpath::PocketEntryMode::LinearRamp;
    straightParams.clearing.rampFeed = 123.0;
    const oap::toolpath::PocketClearingResult straight =
        oap::toolpath::PocketClearing().generate(region, straightParams);
    if (!expect(straight.ok && straight.linearRampEntryCount ==
                    straight.depthLayerCount &&
                    straight.entryFallbackCount == 0,
                "NX-style straight ramp should fit the rectangular pocket")) {
        return false;
    }
    const bool usesConfiguredRampFeed = std::any_of(
        straight.moves.begin(), straight.moves.end(),
        [](const oap::toolpath::ToolpathMove& move) {
            const double horizontal = std::hypot(
                move.to.X() - move.from.X(),
                move.to.Y() - move.from.Y());
            return move.intent ==
                       oap::toolpath::ToolpathMoveIntent::
                           PocketLinearRampEntry &&
                   horizontal > kTolerance &&
                   std::abs(move.feed - 123.0) <= kTolerance;
        });
    if (!expect(usesConfiguredRampFeed,
                "inclined straight ramp moves should use ramp feed")) {
        return false;
    }

    oap::toolpath::ToolpathParams zigzagParams = offsetRampParams();
    zigzagParams.clearing.entryMode =
        oap::toolpath::PocketEntryMode::ZigZagRamp;
    zigzagParams.clearing.zigzagRampWidth = 8.0;
    zigzagParams.clearing.rampStartReference =
        oap::toolpath::PocketRampStartReference::OpeningPlane;
    const oap::toolpath::PocketClearingResult zigzag =
        oap::toolpath::PocketClearing().generate(region, zigzagParams);
    return expect(
        zigzag.ok && zigzag.zigzagRampEntryCount ==
                          zigzag.depthLayerCount &&
            zigzag.entryFallbackCount == 0,
        "NX-style zigzag ramp should fit from the opening plane");
}

bool generatesFreePathThroughTheCommonPocketPipeline()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    oap::toolpath::ToolpathParams params = offsetRampParams();
    params.depth.stepDown = region.depth;
    params.clearing.pattern = oap::toolpath::ClearingPattern::FreePath;
    params.clearing.entryMode =
        oap::toolpath::PocketEntryMode::VerticalPlunge;

    // ClearingParams 的公共默认值是由外向内。FreePath 必须忽略这个旧值，
    // 严格保留 pyocc-exp 的中心向外 run 和 CSC Link 顺序。
    const oap::toolpath::PocketClearingResult legacyDirectionRequest =
        oap::toolpath::PocketClearing().generate(region, params);
    const oap::toolpath::ToolpathParamNormalization normalization =
        oap::toolpath::ToolpathParamNormalizer().normalize(params);
    params.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::InsideToOutside;
    const oap::toolpath::PocketClearingResult result =
        oap::toolpath::PocketClearing().generate(region, params);
    if (!expect(result.ok && result.loops.size() >= 2,
                "Adaptive FreePath should generate a primary run and finishing pass") ||
        !expect(!result.loops.front().closed && result.loops.back().closed,
                "Adaptive FreePath should expose a coherent primary run and boundary pass") ||
        !expect(!result.moves.empty() && !result.primitives.empty(),
                "the common pocket assembler should build FreePath moves")) {
        if (!result.error.empty())
            std::cerr << result.error << '\n';
        return false;
    }
    if (!expect(
            result.moves.size() < 20000u,
            "Adaptive FreePath should keep the generated move stream bounded")) {
        return false;
    }
    if (!expect(
            legacyDirectionRequest.ok &&
                sameFreePathRuns(legacyDirectionRequest.loops, result.loops),
            "FreePath should ignore legacy radial direction without changing Python run or CSC link order") ||
        !expect(
            normalization.effective.clearing.expansionDirection ==
                oap::toolpath::PocketClearingDirection::InsideToOutside,
            "FreePath parameters should normalize to the Python center-out direction")) {
        return false;
    }

    Clipper2Lib::PathD centerBoundary;
    centerBoundary.reserve(region.boundary.points.size());
    for (const gp_Pnt2d& point : region.boundary.points)
        centerBoundary.emplace_back(point.X(), point.Y());
    const oap::toolpath::CutterCenterClearingPatternResult profilePattern =
        oap::toolpath::generateCutterCenterClearingPattern(
            Clipper2Lib::PathsD{std::move(centerBoundary)},
            oap::toolpath::ClearingPattern::FreePath,
            oap::toolpath::CutMode::Climb,
            2.0, params.tool.diameter, params.clearing.adaptive,
            0.0, false, params.clearing.precision,
            params.clearing.arcTolerance);
    if (!expect(
            profilePattern.loops.size() >= 2u &&
                !profilePattern.loops.front().closed &&
                profilePattern.loops.back().closed,
            "the Profile cutter-center facade should preserve the center-out FreePath run order")) {
        return false;
    }

    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "freepath-common-pipeline";
    operation.clearing = result;
    const auto validation =
        oap::toolpath::PocketSafetyValidator().validate(
            region, operation, params);
    return expect(
        validation.report.errorCount == 0,
        "FreePath should pass the existing pocket safety validator");
}

bool clipsFreePathChordThroughNarrowIsland()
{
    const std::vector<gp_Pnt2d> island{
        {48.0, 48.0}, {52.0, 48.0},
        {52.0, 52.0}, {48.0, 52.0}};
    const oap::toolpath::PocketRegion region = pocketRegion(
        {{0.0, 0.0}, {100.0, 0.0},
         {100.0, 100.0}, {0.0, 100.0}},
        {island});
    oap::toolpath::ToolpathParams params = offsetRampParams();
    params.depth.stepDown = region.depth;
    params.clearing.pattern = oap::toolpath::ClearingPattern::FreePath;
    params.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::InsideToOutside;
    params.clearing.adaptive.materialCellSize = 0.5;
    const oap::toolpath::PocketClearingResult clipped =
        oap::toolpath::PocketClearing().generate(region, params);
    if (!expect(clipped.ok && !clipped.loops.empty(),
                "Adaptive FreePath island fixture should generate")) {
        return false;
    }

    bool hasIslandFinishingPass = false;
    for (const oap::toolpath::ClearingLoop& loop : clipped.loops) {
        hasIslandFinishingPass = hasIslandFinishingPass || loop.isHole;
        const std::size_t segmentCount = loop.closed
            ? loop.points.size()
            : loop.points.size() - 1u;
        for (std::size_t index = 0; index < segmentCount; ++index) {
            const gp_Pnt2d segmentMidpoint{
                0.5 * (loop.points[index].X() +
                       loop.points[(index + 1u) % loop.points.size()].X()),
                0.5 * (loop.points[index].Y() +
                       loop.points[(index + 1u) % loop.points.size()].Y())};
            if (!expect(
                    !pointInPolygon(segmentMidpoint, island),
                    "Adaptive FreePath must not cross an island")) {
                return false;
            }
        }
    }
    return expect(
        clipped.loops.size() >= 2u && hasIslandFinishingPass &&
            clipped.adaptiveRemainingMaterialFraction < 0.05,
        "Adaptive FreePath should finish the island and bound residual material");
}

bool usesRadialBranchesAndRejectsDiscontinuousRadialTopology()
{
    constexpr double pi = 3.14159265358979323846;
    std::vector<gp_Pnt2d> fourLobe;
    for (int index = 0; index < 96; ++index) {
        const double angle = 2.0 * pi * static_cast<double>(index) / 96.0;
        const double radius = 30.0 - 6.0 * std::cos(4.0 * angle);
        fourLobe.emplace_back(
            radius * std::cos(angle), radius * std::sin(angle));
    }
    oap::toolpath::ToolpathParams params = offsetRampParams();
    params.tool.diameter = 8.0;
    params.depth.stepDown = 1.0;
    params.clearing.pattern = oap::toolpath::ClearingPattern::FreePath;
    params.clearing.stepOverPercent = 20.0;
    params.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::InsideToOutside;
    params.clearing.adaptive.materialCellSize = 0.8;
    const oap::toolpath::PocketRegion radialRegion =
        pocketRegion(std::move(fourLobe));
    const oap::toolpath::PocketClearingResult radial =
        oap::toolpath::PocketClearing().generate(radialRegion, params);
    if (!expect(
            radial.ok && radial.adaptiveCentralSpiralCount == 1 &&
                radial.adaptiveRadialComponentCount >= 4 &&
                radial.adaptiveRadialPassCount >= 4 &&
                radial.adaptiveRemainingMaterialFraction < 0.03,
            "a four-lobe pocket should use the pyocc radial branch planner")) {
        return false;
    }
    bool hasSampledCscLink = false;
    for (std::size_t index = 1u; index < radial.loops.size(); ++index) {
        const auto& link = radial.loops[index].preferredLinkFromPrevious;
        if (link.size() > 2u) {
            hasSampledCscLink = true;
            break;
        }
    }

    params.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::OutsideToInside;
    const oap::toolpath::PocketClearingResult legacyDirectionRadial =
        oap::toolpath::PocketClearing().generate(radialRegion, params);

    const std::vector<gp_Pnt2d> cross{
        {-15.0, -30.0}, {15.0, -30.0}, {15.0, -12.0},
        {40.0, -12.0}, {40.0, 12.0}, {15.0, 12.0},
        {15.0, 30.0}, {-15.0, 30.0}, {-15.0, 12.0},
        {-40.0, 12.0}, {-40.0, -12.0}, {-15.0, -12.0}};
    const oap::toolpath::PocketRegion crossRegion = pocketRegion(cross);
    const oap::toolpath::PocketClearingResult wavefront =
        oap::toolpath::PocketClearing().generate(crossRegion, params);
    return expect(
        hasSampledCscLink,
        "radial branches should expose a sampled CSC link") &&
        expect(
            legacyDirectionRadial.ok && sameFreePathRuns(
                legacyDirectionRadial.loops, radial.loops),
            "legacy radial direction should not change runs or CSC links") &&
        expect(
            wavefront.ok &&
            wavefront.adaptiveCentralSpiralCount == 0 &&
            wavefront.adaptiveRadialComponentCount == 0 &&
            wavefront.adaptiveWavefrontLayerCount > 0 &&
            wavefront.adaptiveRemainingMaterialFraction < 0.03,
            "a cross should stay on topology wavefronts");
}

bool respectsAdaptiveEngagementAndSingleStepMode()
{
    const oap::toolpath::PocketRegion region = pocketRegion({
        {-40.0, -30.0}, {40.0, -30.0},
        {40.0, 30.0}, {-40.0, 30.0}});
    oap::toolpath::ToolpathParams params = offsetRampParams();
    params.tool.diameter = 8.0;
    params.depth.stepDown = region.depth;
    params.clearing.pattern = oap::toolpath::ClearingPattern::FreePath;
    params.clearing.stepOverPercent = 40.0;
    params.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::InsideToOutside;
    auto& adaptive = params.clearing.adaptive;
    adaptive.targetEngagementAngleDeg = 60.0;
    adaptive.maximumEngagementAngleDeg = 120.0;
    adaptive.lookaheadDepth = 1;
    adaptive.beamWidth = 1;
    adaptive.materialCellSize = 0.8;
    const oap::toolpath::PocketClearingResult result =
        oap::toolpath::PocketClearing().generate(region, params);
    return expect(
        result.ok && result.adaptiveOverloadSegmentCount == 0 &&
            result.adaptiveMaximumEngagementAngleDeg <= 120.0 + 1.0e-6 &&
            result.adaptiveRemainingMaterialFraction < 0.03,
        "single-step adaptive planning should respect the maximum engagement angle");
}

bool rejectsAnExplicitBoundaryViolation()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    const oap::toolpath::ToolpathParams params = offsetRampParams();
    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "pocket-boundary-violation";
    operation.clearing =
        oap::toolpath::PocketClearing().generate(region, params);
    const auto cut = std::find_if(
        operation.clearing.moves.begin(),
        operation.clearing.moves.end(),
        [](const oap::toolpath::ToolpathMove& move) {
            return move.type == oap::toolpath::ToolpathMoveType::Cut;
        });
    if (!expect(cut != operation.clearing.moves.end(),
                "the safety fixture should contain a cut move")) {
        return false;
    }
    cut->to.SetX(-10.0);
    const oap::toolpath::PocketSafetyValidationResult validation =
        oap::toolpath::PocketSafetyValidator().validate(
            region, operation, params);
    return expect(validation.report.errorCount > 0,
                  "the prepared safety area must reject an outside cut");
}

bool validatesPreparedProtectedMaterialEnvelope()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    const oap::toolpath::ToolpathParams params = offsetRampParams();
    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "pocket-protected-material";
    operation.clearing.usedPerLayerSections = true;
    operation.clearing.usedProtectedMaterialSections = true;

    oap::toolpath::ProtectedMaterialLayer layer;
    layer.depthIndex = 0;
    layer.depthOffset = 0.0;
    oap::toolpath::Loop2d material;
    material.closed = true;
    material.points = {
        gp_Pnt2d(20.0, 10.0),
        gp_Pnt2d(25.0, 10.0),
        gp_Pnt2d(25.0, 20.0),
        gp_Pnt2d(20.0, 20.0)};
    material.signedArea = 50.0;
    layer.contours.push_back(material);
    operation.clearing.protectedMaterialLayers.push_back(layer);

    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(5.0, 5.0, 0.0);
    cut.to = gp_Pnt(10.0, 5.0, 0.0);
    cut.depthIndex = 0;
    operation.clearing.moves.push_back(cut);
    const auto safe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    if (!expect(safe.report.errorCount == 0,
                "a cut away from protected material should remain valid")) {
        return false;
    }

    operation.clearing.moves.front().from = gp_Pnt(15.0, 15.0, 0.0);
    operation.clearing.moves.front().to = gp_Pnt(30.0, 15.0, 0.0);
    const auto unsafe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    return expect(
        unsafe.report.errorCount > 0,
        "the prepared envelope must reject a cut through protected material");
}

bool validatesNestedProtectedMaterialWithExactClearance()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    const oap::toolpath::ToolpathParams params = offsetRampParams();
    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "nested-protected-material";
    operation.clearing.usedPerLayerSections = true;
    operation.clearing.usedProtectedMaterialSections = true;

    oap::toolpath::ProtectedMaterialLayer layer;
    layer.depthIndex = 0;
    oap::toolpath::Loop2d materialOuter;
    materialOuter.closed = true;
    materialOuter.points = {
        gp_Pnt2d(8.0, 5.0), gp_Pnt2d(32.0, 5.0),
        gp_Pnt2d(32.0, 25.0), gp_Pnt2d(8.0, 25.0)};
    materialOuter.signedArea = 480.0;
    oap::toolpath::Loop2d materialHole;
    materialHole.closed = true;
    materialHole.points = {
        gp_Pnt2d(12.0, 10.0), gp_Pnt2d(12.0, 20.0),
        gp_Pnt2d(28.0, 20.0), gp_Pnt2d(28.0, 10.0)};
    materialHole.signedArea = -160.0;
    layer.contours = {materialOuter, materialHole};
    operation.clearing.protectedMaterialLayers.push_back(layer);

    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(18.0, 15.0, 0.0);
    cut.to = gp_Pnt(22.0, 15.0, 0.0);
    cut.depthIndex = 0;
    operation.clearing.moves.push_back(cut);
    const auto safe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    if (!expect(
            safe.report.errorCount == 0,
            "a cutter centre with exact clearance inside a nested material hole should be safe")) {
        return false;
    }

    operation.clearing.moves.front().from = gp_Pnt(13.0, 15.0, 0.0);
    operation.clearing.moves.front().to = gp_Pnt(13.5, 15.0, 0.0);
    const auto unsafe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    return expect(
        unsafe.report.errorCount > 0,
        "exact nested material clearance must still reject a real incursion");
}

bool validatesProtectedMaterialRestMachiningEndToEnd()
{
    oap::toolpath::PocketRegion region = rectangularPocket();
    region.useSectionBoundaryAtEveryDepth = true;
    BRep_Builder builder;
    builder.MakeCompound(region.sectionShape);
    builder.Add(
        region.sectionShape,
        BRepPrimAPI_MakeBox(40.0, 30.0, 5.0).Shape());
    region.protectedMaterialShape = BRepPrimAPI_MakeBox(
        gp_Pnt(17.0, 11.0, 0.0), 6.0, 8.0, 5.0).Shape();

    oap::toolpath::ToolpathParams params = offsetRampParams();
    params.clearing.restMachiningPreviousToolDiameter = 12.0;
    const oap::toolpath::PocketClearingResult clearing =
        oap::toolpath::PocketClearing().generate(region, params);
    if (!expect(clearing.ok && clearing.restMachining &&
                    !clearing.moves.empty(),
                "protected rest machining should generate a usable path")) {
        if (!clearing.error.empty())
            std::cerr << clearing.error << '\n';
        return false;
    }

    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "protected-rest-machining";
    operation.clearing = clearing;
    const auto validation =
        oap::toolpath::PocketSafetyValidator().validate(
            region, operation, params);
    return expect(
        validation.report.errorCount == 0,
        "generated rest paths and entries should satisfy final safety validation");
}

bool validatesLargeOperationWithDeterministicFirstViolation()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    const oap::toolpath::ToolpathParams params = offsetRampParams();
    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "large-pocket-safety-validation";

    constexpr std::size_t moveCount = 5000;
    operation.clearing.moves.reserve(moveCount);
    for (std::size_t index = 0; index < moveCount; ++index) {
        const double x = 5.0 +
            0.1 * static_cast<double>(index % 200u);
        const double y = 5.0 +
            0.1 * static_cast<double>((index / 200u) % 100u);
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = gp_Pnt(x, y, 0.0);
        move.to = gp_Pnt(x + 0.04, y, 0.0);
        move.depthIndex = 0;
        operation.clearing.moves.push_back(move);
    }

    const auto safe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    if (!expect(
            safe.report.errorCount == 0,
            "a large parallel safety batch should accept safe cuts")) {
        return false;
    }

    operation.clearing.moves.back().to.SetX(-10.0);
    const auto unsafe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    return expect(
               unsafe.report.errorCount == 1,
               "a large parallel safety batch should report one violation") &&
           expect(
               !unsafe.report.issues.empty() &&
                   unsafe.report.issues.front().moveIndex ==
                       static_cast<int>(moveCount - 1u),
               "parallel safety diagnostics should retain the first move index");
}

bool validatesLargeArcBatchInParallel()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    const oap::toolpath::ToolpathParams params = offsetRampParams();
    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "large-arc-safety-validation";

    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(8.0, 10.0, 0.0);
    cut.to = gp_Pnt(12.0, 10.0, 0.0);
    cut.depthIndex = 0;
    operation.clearing.moves.push_back(cut);

    constexpr std::size_t arcCount = 2000;
    operation.clearing.primitives.reserve(arcCount);
    for (std::size_t index = 0; index < arcCount; ++index) {
        oap::toolpath::ToolpathCircularArc arc;
        arc.from = gp_Pnt(12.0, 10.0, 0.0);
        arc.to = gp_Pnt(10.0, 12.0, 0.0);
        arc.center = gp_Pnt(10.0, 10.0, 0.0);
        arc.axis = gp_Vec(0.0, 0.0, 1.0);
        arc.sweepRadians = 1.5707963267948966;
        arc.feed = 500.0;
        operation.clearing.primitives.push_back(
            oap::toolpath::makeArcPrimitive(arc, {}));
    }

    const auto safe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    if (!expect(
            safe.report.errorCount == 0,
            "a large parallel arc batch should remain safe")) {
        return false;
    }

    oap::toolpath::ToolpathPrimitive& last =
        operation.clearing.primitives.back();
    last.arc.from = gp_Pnt(3.0, 1.0, 0.0);
    last.arc.to = gp_Pnt(1.0, 3.0, 0.0);
    last.arc.center = gp_Pnt(1.0, 1.0, 0.0);
    const auto unsafe = oap::toolpath::PocketSafetyValidator().validate(
        region, operation, params);
    return expect(
        unsafe.report.errorCount == 1,
        "a large parallel arc batch should report an unsafe arc");
}

bool rejectsIncompleteSafetyValidation()
{
    using namespace oap::toolpath;
    const auto region = pocketRegion({
        gp_Pnt2d(0.0, 0.0), gp_Pnt2d(200000.0, 0.0),
        gp_Pnt2d(200000.0, 30.0), gp_Pnt2d(0.0, 30.0)});
    const auto params = offsetRampParams(); // radius 2: sample spacing is 0.5 mm.
    PocketToolpathOperation operation;
    operation.operationId = "safety-budget-boundary";
    ToolpathMove cut;
    cut.type = ToolpathMoveType::Cut;
    cut.depthIndex = 0;
    cut.from = gp_Pnt(10.0, 10.0, 0.0);
    cut.to = gp_Pnt(100009.5, 10.0, 0.0); // 199999 intervals + first point.
    operation.clearing.moves.push_back(cut);
    const auto exact = PocketSafetyValidator().validate(region, operation, params);
    if (!expect(exact.report.ok(), "exactly filling the safety budget should pass"))
        return false;

    // The entire segment is geometrically safe, but requires one sample too many.
    operation.clearing.moves.front().to.SetX(100010.0);
    const auto limited = PocketSafetyValidator().validate(region, operation, params);
    if (!expect(!limited.report.ok() && limited.report.errorCount == 1 &&
                    !limited.cancelled && !limited.diagnostics.empty() &&
                    limited.diagnostics.front().code ==
                        ToolpathDiagnosticCode::PocketSafetySamplingLimited &&
                    limited.diagnostics.front().severity == ToolpathDiagnosticSeverity::Error,
                "incomplete validation must fail without claiming a boundary collision"))
        return false;

    // A later move must not disappear behind an already exhausted budget.
    operation.clearing.moves.front() = cut;
    cut.from = cut.to;
    cut.to.SetX(200001.0);
    operation.clearing.moves.push_back(cut);
    const auto tail = PocketSafetyValidator().validate(region, operation, params);
    return expect(!tail.report.ok(), "an unchecked tail must prevent validation success");
}

bool cancelsSafetyValidationCooperatively()
{
    const oap::toolpath::PocketRegion region = rectangularPocket();
    const oap::toolpath::ToolpathParams params = offsetRampParams();
    oap::toolpath::PocketToolpathOperation operation;
    operation.operationId = "cancel-pocket-safety-validation";

    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(8.0, 10.0, 0.0);
    cut.to = gp_Pnt(12.0, 10.0, 0.0);
    cut.depthIndex = 0;
    operation.clearing.moves.push_back(cut);

    std::size_t cancellationChecks = 0;
    const auto validation =
        oap::toolpath::PocketSafetyValidator().validate(
            region, operation, params,
            [&cancellationChecks] {
                return ++cancellationChecks >= 2u;
            });
    return expect(
        validation.cancelled,
        "pocket safety validation should observe cancellation");
}

} // namespace

int main()
{
    if (!parallelOffsetRingsPreserveDeterministicOutput() ||
        !parallelZigZagScanLinesPreserveDeterministicOutput() ||
        !linksEveryOffsetRingWithinEachDepthLayer() ||
        !generatesNxStyleStraightAndZigzagEntries() ||
        !generatesFreePathThroughTheCommonPocketPipeline() ||
        !clipsFreePathChordThroughNarrowIsland() ||
        !usesRadialBranchesAndRejectsDiscontinuousRadialTopology() ||
        !respectsAdaptiveEngagementAndSingleStepMode() ||
        !rejectsAnExplicitBoundaryViolation() ||
        !validatesPreparedProtectedMaterialEnvelope() ||
        !validatesNestedProtectedMaterialWithExactClearance() ||
        !validatesProtectedMaterialRestMachiningEndToEnd() ||
        !validatesLargeOperationWithDeterministicFirstViolation() ||
        !validatesLargeArcBatchInParallel() ||
        !cancelsSafetyValidationCooperatively() ||
        !rejectsIncompleteSafetyValidation()) {
        return 1;
    }
    std::cout << "PASS: pocket layer linking checks\n";
    return 0;
}

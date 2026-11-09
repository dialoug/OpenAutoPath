// 孔自动规划与 Job 重建共享批次内核的纯内存回归。
#include "toolpath/hole/HoleBatchExecution.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleToolpathValidator.h"
#include "toolpath/workflow/HoleBatchExecutor.h"
#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <numbers>
#include <string>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

bool nearValue(double first, double second)
{
    return std::abs(first - second) <= 1.0e-9;
}

bool bindsToolLibraryParameters()
{
    oap::toolpath::ToolpathParams base;
    base.tool.diameter = 2.0;
    base.tool.horizFeed = 20.0;
    base.tool.vertFeed = 10.0;
    base.tool.spindleSpeed = 1000.0;
    base.tool.tipCompensation = 0.25;
    base.hole.peckDepth = 2.0;
    base.hole.drillTipAngleDeg = 118.0;
    base.hole.drillCycleMode = oap::toolpath::HoleDrillCycleMode::Auto;

    oap::toolpath::ToolpathToolContext drill;
    drill.valid = true;
    drill.toolType = "drill";
    drill.diameter = 6.0;
    drill.horizontalFeed = 120.0;
    drill.verticalFeed = 60.0;
    drill.spindleSpeed = 3200.0;
    drill.tipAngle = 140.0;
    drill.peckDepth = 1.5;
    drill.drillCycle =
        oap::toolpath::ToolpathDrillCyclePreference::ChipBreak;

    const oap::toolpath::ToolpathParams bound =
        oap::toolpath::bindHoleBatchParameters(base, drill);
    if (!expect(nearValue(bound.tool.diameter, 6.0) &&
                    nearValue(bound.tool.horizFeed, 120.0) &&
                    nearValue(bound.tool.vertFeed, 60.0) &&
                    nearValue(bound.tool.spindleSpeed, 3200.0) &&
                    nearValue(bound.hole.peckDepth, 1.5) &&
                    nearValue(bound.hole.drillTipAngleDeg, 140.0) &&
                    bound.hole.drillCycleMode ==
                        oap::toolpath::HoleDrillCycleMode::ChipBreak,
                "automatic drill-library context should bind once")) {
        return false;
    }

    oap::toolpath::ToolpathToolContext endMill = drill;
    endMill.toolType = "end_mill";
    const oap::toolpath::ToolpathParams milling =
        oap::toolpath::bindHoleBatchParameters(base, endMill);
    return expect(milling.hole.drillCycleMode ==
                          oap::toolpath::HoleDrillCycleMode::Direct &&
                      nearValue(milling.hole.peckDepth, 0.0) &&
                      nearValue(milling.hole.drillTipAngleDeg, 0.0) &&
                      nearValue(milling.tool.tipCompensation, 0.0),
                  "a valid milling tool should clear drill-only parameters");
}

bool aggregatesSummaryWithoutOperationsOrTransactionState()
{
    oap::toolpath::HoleMachiningResult target;
    target.smallCount = 1;
    target.setupCount = 4;
    target.cancelled = false;
    target.warnings.push_back("existing");

    oap::toolpath::HoleMachiningResult source;
    source.smallCount = 2;
    source.mediumCount = 3;
    source.solidSweepCheckCount = 5;
    source.solidSweepConstructedPartCount = 7;
    source.rapidRepairRecoveredLinkCount = 11;
    source.optimizedRouteLength = 13.5;
    source.setupCount = 9;
    source.cancelled = true;
    source.warnings.push_back("source");
    source.operations.emplace_back();

    oap::toolpath::appendHoleMachiningSummary(target, source);
    return expect(target.smallCount == 3 && target.mediumCount == 3 &&
                      target.solidSweepCheckCount == 5 &&
                      target.solidSweepConstructedPartCount == 7 &&
                      target.rapidRepairRecoveredLinkCount == 11 &&
                      nearValue(target.optimizedRouteLength, 13.5) &&
                      target.warnings.size() == 2 &&
                      target.operations.empty() &&
                      target.setupCount == 4 && !target.cancelled,
                  "summary aggregation must leave mapped operations and transaction state to the caller");
}

oap::toolpath::HoleMachiningInput executorHole(double x)
{
    oap::toolpath::HoleMachiningInput result;
    result.kind = oap::FeatureKind::BlindHole;
    result.entry = gp_Pnt(x, 0.0, 10.0);
    result.bottom = gp_Pnt(x, 0.0, 4.0);
    result.radius = 2.0;
    result.bottomProfile = oap::HoleBottomProfile::Planar;
    result.entryResolution =
        oap::HoleEntryResolution::MoldEnvelopeProjected;
    result.setupFace = oap::MoldSetupFace::PositiveZ;
    result.preserveEntrySide = true;
    result.mappingState =
        oap::toolpath::HoleMachiningMappingState::Ready;
    return result;
}

oap::toolpath::ToolpathToolContext executorDrill()
{
    oap::toolpath::ToolpathToolContext result;
    result.valid = true;
    result.toolNumber = 5;
    result.toolType = "drill";
    result.diameter = 4.0;
    result.cuttingEdgeLength = 20.0;
    result.overallLength = 30.0;
    result.verticalFeed = 100.0;
    result.horizontalFeed = 200.0;
    result.spindleSpeed = 3000.0;
    result.tipAngle = 118.0;
    return result;
}

oap::toolpath::HoleBatchExecutorRequest executorRequest(bool parallel)
{
    oap::toolpath::HoleBatchExecutorRequest request;
    request.parallel = parallel;
    request.baseParams.tool.diameter = 4.0;
    request.baseParams.tool.cuttingEdgeLength = 20.0;
    request.baseParams.tool.overallLength = 30.0;
    request.baseParams.tool.vertFeed = 100.0;
    request.baseParams.tool.horizFeed = 200.0;
    request.baseParams.tool.spindleSpeed = 3000.0;
    request.baseParams.depth.safeHeight = 2.0;
    request.baseParams.depth.clearanceHeight = 4.0;
    request.baseParams.hole.useCenterSpot = false;
    request.baseParams.hole.drillCycleMode =
        oap::toolpath::HoleDrillCycleMode::Direct;
    for (int index = 0; index < 2; ++index) {
        oap::toolpath::HoleBatchExecutorItem item;
        item.logicalId = index == 0 ? "batch-B" : "batch-A";
        item.tool = executorDrill();
        item.inputs.push_back(executorHole(index * 10.0));
        item.strategies.push_back(
            oap::toolpath::HoleMachiningStrategy::SmallDrill);
        request.batches.push_back(std::move(item));
    }
    return request;
}

// Characterize the geometric difference between sampled moves and native arcs.
// This is not an acceptance tolerance for production machining.
bool helixSamplingPreservesArcButChangesChordError()
{
    using namespace oap::toolpath;
    auto hole = executorHole(0.0);
    hole.kind = oap::FeatureKind::ThroughHole;
    hole.entry = gp_Pnt(0.0, 0.0, 0.0);
    hole.bottom = gp_Pnt(0.0, 0.0, -12.0);
    hole.radius = 5.0;
    auto params = executorRequest(false).baseParams;
    params.hole.allowance = 0.2;
    params.hole.helixPitch = 2.0;
    params.hole.minSampleAngleDeg = 3.0;
    params.hole.maxSampleAngleDeg = 20.0;
    params.hole.maxSamplePoints = 10000;
    const std::vector<HoleMachiningStrategy> strategies{
        HoleMachiningStrategy::MediumHelical};
    constexpr double radius = 2.8;
    std::size_t previousCutCount = 0;
    for (const double angle : {10.0, 20.0}) {
        params.hole.sampleAngleDeg = angle;
        const auto generated = HoleMachining().generate(
            {hole}, params, nullptr, &strategies, nullptr);
        if (!expect(!generated.cancelled && generated.operations.size() == 1,
                    "helix fixture must generate exactly one operation"))
            return false;
        const auto& operation = generated.operations.front();
        std::size_t cutCount = 0;
        double maximumError = 0.0;
        for (const auto& move : operation.moves) {
            if (move.type != ToolpathMoveType::Cut)
                continue;
            ++cutCount;
            // For this Z-axis helix, the linear midpoint has the exact
            // interpolated depth, so its radial deficit measures chord error.
            const double x = 0.5 * (move.from.X() + move.to.X());
            const double y = 0.5 * (move.from.Y() + move.to.Y());
            maximumError = std::max(maximumError, radius - std::hypot(x, y));
            if (!expect(nearValue(std::hypot(move.from.X(), move.from.Y()), radius) &&
                            nearValue(std::hypot(move.to.X(), move.to.Y()), radius),
                        "sampled cutting endpoints must lie on the intended cylinder"))
                return false;
        }
        const double expectedError = radius *
            (1.0 - std::cos(angle * std::numbers::pi / 360.0));
        if (!expect(cutCount > 0 && nearValue(maximumError, expectedError),
                    "measured midpoint error must match analytic chord error"))
            return false;
        double arcSweep = 0.0;
        for (const auto& primitive : operation.primitives) {
            if (primitive.type != ToolpathPrimitiveType::CircularArc)
                continue;
            const auto& arc = primitive.arc;
            if (!expect(nearValue(std::hypot(arc.from.X() - arc.center.X(),
                                       arc.from.Y() - arc.center.Y()), radius),
                        "native arcs must retain the intended center-path radius"))
                return false;
            arcSweep += arc.sweepRadians;
        }
        if (!expect(nearValue(arcSweep, 14.0 * std::numbers::pi),
                    "native arcs must retain six descending turns plus one finish turn"))
            return false;
        if (previousCutCount &&
            !expect(previousCutCount == 2 * cutCount,
                    "doubling the angle must halve cuts for this integral-turn fixture"))
            return false;
        previousCutCount = cutCount;
        std::cout << "helix angle=" << angle << " deg, cuts=" << cutCount
                  << ", midpoint error=" << maximumError << " mm\n";
    }

    // Seven turns = 2520 degrees. A budget of 141 points requires an
    // 18-degree step, although the requested step is 10 degrees.
    params.hole.sampleAngleDeg = 10.0;
    params.hole.maxSamplePoints = 141;
    const auto adjusted = HoleMachining().generate(
        {hole}, params, nullptr, &strategies, nullptr);
    if (!expect(adjusted.operations.size() == 1,
                "sampling must adapt when the budget fits the allowed angle range"))
        return false;
    std::size_t adjustedCuts = 0;
    double adjustedError = 0.0;
    for (const auto& move : adjusted.operations.front().moves) {
        if (move.type != ToolpathMoveType::Cut)
            continue;
        ++adjustedCuts;
        adjustedError = std::max(adjustedError, radius - std::hypot(
            0.5 * (move.from.X() + move.to.X()),
            0.5 * (move.from.Y() + move.to.Y())));
    }
    if (!expect(adjustedCuts == 140 && nearValue(adjustedError,
                    radius * (1.0 - std::cos(9.0 * std::numbers::pi / 180.0))),
                "budget adaptation must produce the expected 18-degree geometry"))
        return false;
    std::cout << "helix budget=141, cuts=" << adjustedCuts
              << ", midpoint error=" << adjustedError << " mm\n";

    params.hole.maxSampleAngleDeg = 15.0;
    const auto rejected = HoleMachining().generate(
        {hole}, params, nullptr, &strategies, nullptr);
    const bool reportsBudgetError = std::any_of(
        rejected.diagnostics.begin(), rejected.diagnostics.end(),
        [](const ToolpathDiagnostic& diagnostic) {
            return diagnostic.code == ToolpathDiagnosticCode::SamplingBudgetExceeded &&
                   diagnostic.severity == ToolpathDiagnosticSeverity::Error;
        });
    if (!expect(rejected.operations.empty() && rejected.skippedCount == 1 &&
                    reportsBudgetError,
                "incompatible budget and angle cap must reject the hole with a diagnostic"))
        return false;

    params.hole.maxSampleAngleDeg = 20.0;
    params.hole.sampleAngleDeg = 20.0;
    params.hole.maxSamplePoints = 10000;
    params.hole.helixChordTolerance = 0.02;
    const auto precise = HoleMachining().generate(
        {hole}, params, nullptr, &strategies, nullptr);
    if (!expect(precise.operations.size() == 1,
                "feasible chord tolerance must generate a helix"))
        return false;
    double preciseError = 0.0;
    for (const auto& move : precise.operations.front().moves) {
        if (move.type == ToolpathMoveType::Cut) {
            preciseError = std::max(preciseError, radius - std::hypot(
                0.5 * (move.from.X() + move.to.X()),
                0.5 * (move.from.Y() + move.to.Y())));
        }
    }
    if (!expect(preciseError <= 0.02 + 1.0e-12 && preciseError > 0.019,
                "enabled chord tolerance must constrain actual generated geometry"))
        return false;
    std::cout << "helix chord limit=0.02 mm, midpoint error="
              << preciseError << " mm\n";
    params.hole.maxSamplePoints = 141;
    const auto precisionConflict = HoleMachining().generate(
        {hole}, params, nullptr, &strategies, nullptr);
    if (!expect(precisionConflict.operations.empty() &&
                    precisionConflict.skippedCount == 1,
                "budget must not silently relax the chord tolerance"))
        return false;
    params.hole.maxSamplePoints = 10000;
    params.hole.minSampleAngleDeg = 15.0;
    const auto minimumConflict = HoleMachining().generate(
        {hole}, params, nullptr, &strategies, nullptr);
    if (!expect(minimumConflict.operations.empty(),
                "minimum angle conflicting with chord tolerance must be rejected"))
        return false;
    return true;
}

bool reentriesRequirePreviouslyCutCenterline()
{
    using namespace oap::toolpath;
    ToolpathMove cut;
    cut.type = ToolpathMoveType::Cut;
    cut.from = gp_Pnt(0, 0, 2);
    cut.to = gp_Pnt(0, 0, -3);
    ToolpathMove reentry;
    reentry.type = ToolpathMoveType::Rapid;
    reentry.intent = ToolpathMoveIntent::DrillCycleReentry;
    reentry.depthIndex = 1;
    reentry.from = cut.from;
    reentry.to = gp_Pnt(0, 0, -2);
    const auto valid = [](const std::vector<ToolpathMove>& moves) {
        ToolpathValidationReport report;
        validateHoleDrillReentries(moves, gp_Vec(0, 0, -1), "test", report);
        return report.ok();
    };
    if (!expect(valid({cut, reentry}), "reentry into a previous cut must be allowed") ||
        !expect(!valid({reentry}), "reentry without prior cutting must be rejected"))
        return false;
    auto shifted = reentry;
    shifted.from.SetX(5);
    shifted.to.SetX(5);
    auto deeper = reentry;
    deeper.to.SetZ(-4);
    auto reversed = reentry;
    std::swap(reversed.from, reversed.to);
    if (!expect(!valid({cut, shifted}), "parallel displaced reentry must be rejected") ||
        !expect(!valid({cut, deeper}), "reentry beyond prior cutting must be rejected") ||
        !expect(!valid({cut, reversed}), "outward movement cannot claim inward reentry"))
        return false;
    auto disconnected = cut;
    disconnected.from.SetZ(-5);
    disconnected.to.SetZ(-8);
    auto crossesGap = reentry;
    crossesGap.to.SetZ(-7);
    if (!expect(!valid({cut, disconnected, crossesGap}),
                "separate cuts must not imply that an uncut gap was cleared"))
        return false;
    auto request = executorRequest(false);
    request.baseParams.hole.drillCycleMode = HoleDrillCycleMode::Peck;
    request.baseParams.hole.peckDepth = 2;
    const std::vector<HoleMachiningStrategy> strategies{HoleMachiningStrategy::SmallDrill};
    const auto generated = HoleMachining().generate(
        {executorHole(0)}, request.baseParams, nullptr, &strategies, nullptr);
    return expect(generated.operations.size() == 1 &&
                      valid(generated.operations.front().moves),
                  "normal multi-peck drilling must retain valid reentries");
}

bool executorIsDeterministicAndTransactional()
{
    const oap::toolpath::HoleBatchExecutor executor;
    const oap::toolpath::HoleBatchExecutorResult serial =
        executor.execute(executorRequest(false));
    const oap::toolpath::HoleBatchExecutorResult parallel =
        executor.execute(executorRequest(true));
    if (!expect(serial.ok() && parallel.ok() &&
                    serial.batches.size() == 2 &&
                    parallel.batches.size() == 2 &&
                    serial.batches[0].logicalId == "batch-B" &&
                    serial.batches[1].logicalId == "batch-A" &&
                    parallel.batches[0].logicalId == "batch-B" &&
                    parallel.batches[1].logicalId == "batch-A" &&
                    serial.machining.smallCount ==
                        parallel.machining.smallCount &&
                    serial.machining.optimizedRouteLength ==
                        parallel.machining.optimizedRouteLength,
                "serial and parallel execution must retain request order and summary")) {
        return false;
    }

    oap::toolpath::ToolpathGenerationControl cancelledControl;
    cancelledControl.isCancellationRequested = [] { return true; };
    const oap::toolpath::HoleBatchExecutorResult cancelled =
        executor.execute(executorRequest(true), &cancelledControl);
    return expect(cancelled.cancelled && !cancelled.ok() &&
                      cancelled.batches.empty() &&
                      cancelled.machining.operations.empty() &&
                      cancelled.machining.cancelled,
                  "cancelled execution must not publish partial batches or aggregate");
}

bool cancelsDuringVisitOptimization()
{
    oap::MoldCoordinateFrame frame;
    frame.length = 1000.0;
    frame.width = 1000.0;
    frame.thickness = 100.0;
    frame.isValid = true;
    oap::toolpath::HoleRapidPlanningContext rapidContext;
    rapidContext.moldFrame = &frame;

    oap::toolpath::HoleBatchExecutorRequest request = executorRequest(false);
    request.batches.resize(1);
    request.batches.front().inputs.clear();
    request.batches.front().strategies.clear();
    for (int index = 0; index < 300; ++index) {
        request.batches.front().inputs.push_back(
            executorHole(static_cast<double>(index)));
        request.batches.front().strategies.push_back(
            oap::toolpath::HoleMachiningStrategy::SmallDrill);
    }
    request.rapidContext = &rapidContext;

    std::atomic_size_t cancellationChecks{0};
    oap::toolpath::ToolpathGenerationControl control;
    control.isCancellationRequested = [&cancellationChecks] {
        return cancellationChecks.fetch_add(
                   1, std::memory_order_relaxed) >= 350;
    };
    const oap::toolpath::HoleBatchExecutorResult cancelled =
        oap::toolpath::HoleBatchExecutor().execute(request, &control);
    return expect(cancelled.cancelled && cancelled.batches.empty() &&
                      cancelled.machining.operations.empty() &&
                      cancellationChecks.load(std::memory_order_relaxed) > 350,
                  "cancellation during visit optimization must discard the transaction promptly");
}

bool boundsLargeVisitOptimization()
{
    oap::MoldCoordinateFrame frame;
    frame.length = 1000.0;
    frame.width = 1000.0;
    frame.thickness = 100.0;
    frame.isValid = true;
    oap::toolpath::HoleRapidPlanningContext rapidContext;
    rapidContext.moldFrame = &frame;

    oap::toolpath::HoleBatchExecutorRequest request = executorRequest(false);
    request.batches.resize(1);
    request.batches.front().inputs.clear();
    request.batches.front().strategies.clear();
    for (int index = 0; index < 501; ++index) {
        request.batches.front().inputs.push_back(
            executorHole(static_cast<double>(index)));
        request.batches.front().strategies.push_back(
            oap::toolpath::HoleMachiningStrategy::SmallDrill);
    }
    request.rapidContext = &rapidContext;

    const oap::toolpath::HoleBatchExecutorResult result =
        oap::toolpath::HoleBatchExecutor().execute(request);
    const bool hasBoundedRoutingWarning = std::any_of(
        result.machining.warnings.begin(), result.machining.warnings.end(),
        [](const std::string& warning) {
            return warning.find("bounded nearest-neighbor") !=
                std::string::npos;
        });
    return expect(result.ok() && result.batches.size() == 1 &&
                      hasBoundedRoutingWarning,
                  "large visit sets must use the bounded routing fallback");
}

bool sharesCallerWorkerBudget()
{
    auto request = executorRequest(true);
    for (int index = 2; index < 8; ++index) {
        auto batch = request.batches.front();
        batch.logicalId = "batch-" + std::to_string(index);
        request.batches.push_back(std::move(batch));
    }
    oap::toolpath::ToolpathParallelBudget budget(2);
    std::size_t borrowed = 0;
    std::size_t returned = 0;
    oap::toolpath::ToolpathGenerationControl control;
    control.tryAcquireParallelWorkers = [&](std::size_t requested) {
        const auto granted = budget.tryAcquire(requested);
        borrowed += granted;
        return granted;
    };
    control.releaseParallelWorkers = [&](std::size_t count) {
        returned += count;
        budget.release(count);
    };
    const auto parallel = oap::toolpath::HoleBatchExecutor().execute(request, &control);
    request.parallel = false;
    const auto serial = oap::toolpath::HoleBatchExecutor().execute(request);
    if (!expect(parallel.ok() && serial.ok() && parallel.batches.size() == 8 &&
                    serial.batches.size() == 8 && borrowed == returned &&
                    borrowed == std::min<std::size_t>(
                        2, oap::toolpath::toolpathHardwareParallelism() - 1) &&
                    budget.available() == budget.capacity(),
                "hole batches must respect and return caller worker leases"))
        return false;
    for (std::size_t index = 0; index < serial.batches.size(); ++index) {
        if (!expect(parallel.batches[index].logicalId == serial.batches[index].logicalId &&
                        parallel.batches[index].generated.machining.optimizedRouteLength ==
                            serial.batches[index].generated.machining.optimizedRouteLength,
                    "bounded hole workers must retain deterministic batch order"))
            return false;
    }
    return true;
}

} // namespace

int main()
{
    if (!bindsToolLibraryParameters() ||
        !helixSamplingPreservesArcButChangesChordError() ||
        !reentriesRequirePreviouslyCutCenterline() ||
        !aggregatesSummaryWithoutOperationsOrTransactionState() ||
        !executorIsDeterministicAndTransactional() ||
        !sharesCallerWorkerBudget() ||
        !cancelsDuringVisitOptimization() ||
        !boundsLargeVisitOptimization()) {
        return 1;
    }
    std::cout << "hole batch execution smoke tests passed\n";
    return 0;
}

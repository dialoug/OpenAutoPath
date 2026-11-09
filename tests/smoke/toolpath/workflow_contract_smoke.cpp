// Public service contracts: intentionally independent of private orchestration helpers.
#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"
#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace {
using namespace oap::toolpath;
constexpr char kAlgorithm[] = "test.workflow.contract";
int failures = 0;

void check(bool condition, std::string_view message)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool hasDiagnostic(const std::vector<ToolpathDiagnostic>& diagnostics,
                   ToolpathDiagnosticCode code)
{
    return std::any_of(diagnostics.begin(), diagnostics.end(),
        [code](const auto& diagnostic) { return diagnostic.code == code; });
}

class HoleAlgorithm final : public IHoleToolpathAlgorithm {
public:
    std::function<HoleMachiningResult(const std::vector<HoleMachiningInput>&,
        const std::vector<HoleMachiningStrategy>*, const ToolpathGenerationControl*)> run;
    std::string_view id() const noexcept override { return kAlgorithm; }
    HoleMachiningResult generate(const std::vector<HoleMachiningInput>& holes,
        const ToolpathParams&, const HoleRapidPlanningContext*,
        const std::vector<HoleMachiningStrategy>* overrides,
        const ToolpathGenerationControl* control) const override
    { return run(holes, overrides, control); }
};

class PocketAlgorithm final : public IPocketToolpathAlgorithm {
public:
    std::function<PocketClearingResult(const ToolpathGenerationControl*)> run;
    std::string_view id() const noexcept override { return kAlgorithm; }
    PocketClearingResult generate(const PocketRegion&, const ToolpathParams&) const override
    { return run(nullptr); }
    PocketClearingResult generate(const PocketRegion&, const ToolpathParams&,
        const ToolpathGenerationControl* control) const override
    { return run(control); }
};

class ProfileAlgorithm final : public IProfileToolpathAlgorithm {
public:
    std::function<ProfileContourResult(const ToolpathGenerationControl*)> run;
    std::string_view id() const noexcept override { return kAlgorithm; }
    ProfileContourResult generate(const ProfileRegion&, const ToolpathParams&,
        const ToolpathGenerationControl* control) const override
    { return run(control); }
};

struct Fixture {
    std::shared_ptr<HoleAlgorithm> hole = std::make_shared<HoleAlgorithm>();
    std::shared_ptr<PocketAlgorithm> pocket = std::make_shared<PocketAlgorithm>();
    std::shared_ptr<ProfileAlgorithm> profile = std::make_shared<ProfileAlgorithm>();
    std::shared_ptr<ToolpathAlgorithmRegistry> registry =
        std::make_shared<ToolpathAlgorithmRegistry>();
    ToolpathParams params;
    ToolpathGenerationControl control;
    Fixture()
    {
        params.tool.diameter = 6.0;
        // Zero additional workers gives deterministic region order without bypassing admission.
        control.tryAcquireParallelWorkers = [](std::size_t) { return 0u; };
        control.releaseParallelWorkers = [](std::size_t) {};
        control.profileEvaluationOnly = true;
        hole->run = [](const auto&, const auto*, const auto*) { return HoleMachiningResult{}; };
        pocket->run = [](const auto*) { return PocketClearingResult{}; };
        profile->run = [](const auto*) { return ProfileContourResult{}; };
        registry->registerHoleAlgorithm(hole);
        registry->registerPocketAlgorithm(pocket);
        registry->registerProfileAlgorithm(profile);
    }
    HoleToolpathBatch holes(std::vector<HoleMachiningInput> inputs = {readyHole()})
    { return HoleToolpathService(registry, kAlgorithm).generate(inputs, params, nullptr, nullptr, &control); }
    PocketToolpathBatch pockets(std::size_t count = 1)
    { return PocketToolpathService(registry, kAlgorithm).generateCandidate(
        std::vector<PocketRegion>(count), params, "P", &control); }
    ProfileToolpathBatch profiles(std::size_t count = 1)
    { return ProfileToolpathService(registry, kAlgorithm).generate(
        std::vector<ProfileRegion>(count), params, "R", &control); }
    static HoleMachiningInput readyHole()
    {
        HoleMachiningInput hole;
        hole.mappingState = HoleMachiningMappingState::Ready;
        return hole;
    }
};

PocketClearingResult pocketSuccess()
{
    PocketClearingResult result;
    result.ok = true;
    ToolpathMove move;
    move.type = ToolpathMoveType::Cut;
    move.to = gp_Pnt(1.0, 0.0, 0.0);
    move.feed = 100.0;
    result.moves.push_back(move);
    return result;
}

ProfileContourResult profileSuccess()
{
    ProfileContourResult result;
    result.ok = true;
    result.evaluationOnly = true;
    return result;
}

void unavailableAndDiagnosticOrder()
{
    Fixture fixture;
    fixture.registry = std::make_shared<ToolpathAlgorithmRegistry>();
    fixture.params.tool.horizFeed = -1.0;
    const auto hole = fixture.holes({Fixture::readyHole(), HoleMachiningInput{}});
    const auto pocket = fixture.pockets(2);
    const auto profile = fixture.profiles(2);
    for (const auto* diagnostics : {&hole.diagnostics, &pocket.diagnostics, &profile.diagnostics}) {
        check(diagnostics->size() >= 2 && (*diagnostics)[0].code == ToolpathDiagnosticCode::FeedRateAdjusted &&
            (*diagnostics)[1].code == ToolpathDiagnosticCode::AlgorithmUnavailable,
            "normalization warning precedes unavailable algorithm diagnostic");
    }
    check(hole.diagnostics.size() == 3 && hole.diagnostics.back().code == ToolpathDiagnosticCode::HoleOpeningUnresolved &&
        hole.diagnostics.back().sourceIndex == 1, "hole mapping diagnostic follows service diagnostics");
    check(hole.machining.skippedCount == 2, "hole unavailable counts ready and unresolved inputs");
    check(pocket.failures.size() == 2 && pocket.regionOutcomes.size() == 2 &&
        pocket.failures[0].clearing.failureCode == PocketClearingFailureCode::AlgorithmUnavailable,
        "pocket unavailable creates a failure and mapping for every region");
    check(profile.failures.empty() && profile.operations.empty(),
        "profile unavailable is batch diagnostic without per-region failures");
    check(!pocket.safetyValidationPerformed && !profile.safetyValidationPerformed,
        "candidate and evaluation-only services do not claim safety validation");
}

void invalidParametersAndCancellationPrecedence()
{
    Fixture fixture;
    fixture.params.tool.diameter = 0.0;
    fixture.control.isCancellationRequested = [] { return true; };
    const auto hole = fixture.holes();
    const auto pocket = fixture.pockets(2);
    check(hole.cancelled && hole.machining.skippedCount == 0 && hole.diagnostics.size() >= 2 &&
        hole.diagnostics[0].code == ToolpathDiagnosticCode::InvalidToolDiameter &&
        hole.diagnostics[1].code == ToolpathDiagnosticCode::GenerationCancelled,
        "hole cancellation wins after recording invalid parameters");
    check(!pocket.cancelled && pocket.failures.size() == 2 &&
        pocket.failures[0].clearing.failureCode == PocketClearingFailureCode::InvalidRegion &&
        !hasDiagnostic(pocket.diagnostics, ToolpathDiagnosticCode::GenerationCancelled),
        "pocket invalid parameters return before cancellation");
}

void exceptionMappingAndPartialSuccess()
{
    for (const bool unknown : {false, true}) {
        Fixture fixture;
        fixture.hole->run = [unknown](const auto&, const auto*, const auto*) -> HoleMachiningResult {
            if (unknown) throw 7;
            throw std::runtime_error("contract error");
        };
        int pocketCalls = 0;
        fixture.pocket->run = [&](const auto*) -> PocketClearingResult {
            if (++pocketCalls == 1) {
                if (unknown) throw 7;
                throw std::runtime_error("contract error");
            }
            return pocketSuccess();
        };
        int profileCalls = 0;
        fixture.profile->run = [&](const auto*) -> ProfileContourResult {
            if (++profileCalls == 1) {
                if (unknown) throw 7;
                throw std::runtime_error("contract error");
            }
            return profileSuccess();
        };
        const auto hole = fixture.holes();
        const auto pocket = fixture.pockets(2);
        const auto profile = fixture.profiles(2);
        check(hole.machining.skippedCount == 1 && hole.machining.operations.empty() &&
            hasDiagnostic(hole.diagnostics, ToolpathDiagnosticCode::AlgorithmExecutionFailed),
            "hole exception is a batch error and skips inputs");
        check(pocket.failures.size() == 1 && pocket.operations.size() == 1 && pocketCalls == 2,
            "pocket exception preserves later successful regions");
        if (!pocket.failures.empty())
            check(pocket.failures[0].clearing.failureCode == PocketClearingFailureCode::AlgorithmExecutionFailed &&
                pocket.failures[0].clearing.error == (unknown ? "unknown exception" : "contract error"),
                "pocket exception failure code and message");
        check(hasDiagnostic(pocket.diagnostics, ToolpathDiagnosticCode::AlgorithmExecutionFailed) &&
            !pocket.diagnostics.empty() && pocket.diagnostics.back().sourceIndex == 0,
            "pocket exception diagnostic identifies failed source region");
        if (!pocket.operations.empty())
            check(pocket.operations[0].operationId == "P1" && pocket.operations[0].sourceRegionIndex == 1 &&
                pocket.operationForRegion(0) == nullptr && pocket.operationForRegion(1) != nullptr,
                "pocket success numbering is dense and mapping retains original region");
        check(profile.failures.size() == 1 && profile.operations.size() == 1 && profileCalls == 2,
            "profile exception preserves later successful regions");
        if (!profile.failures.empty())
            check(profile.failures[0].contour.failureCode == ProfileContourFailureCode::OffsetFailed &&
                profile.failures[0].contour.error == (unknown ? "unknown profile algorithm exception" : "contract error"),
                "profile exception retains domain-specific failure mapping");
        check(!hasDiagnostic(profile.diagnostics, ToolpathDiagnosticCode::AlgorithmExecutionFailed),
            "profile exception is represented in failures without a duplicate batch diagnostic");
        if (!profile.operations.empty())
            check(profile.operations[0].operationId == "R2" && profile.operations[0].sourceRegionIndex == 1,
                "profile operation numbering follows source region index");
    }
}

void cancellationDiscardsPartialResults()
{
    for (const bool callbackCancellation : {false, true}) {
        Fixture fixture;
        bool cancelled = false;
        fixture.control.isCancellationRequested = [&] { return cancelled; };
        fixture.hole->run = [&](const auto&, const auto*, const auto*) {
            HoleMachiningResult result;
            result.operations.emplace_back();
            result.cancelled = !callbackCancellation;
            cancelled = callbackCancellation;
            return result;
        };
        int pocketCalls = 0;
        fixture.pocket->run = [&](const auto*) {
            auto result = pocketSuccess();
            if (++pocketCalls == 2) {
                result.cancelled = !callbackCancellation;
                cancelled = callbackCancellation;
            }
            return result;
        };
        int profileCalls = 0;
        fixture.profile->run = [&](const auto*) {
            auto result = profileSuccess();
            if (++profileCalls == 2) {
                result.cancelled = !callbackCancellation;
                cancelled = callbackCancellation;
            }
            return result;
        };
        const auto hole = fixture.holes();
        cancelled = false;
        const auto pocket = fixture.pockets(3);
        cancelled = false;
        const auto profile = fixture.profiles(3);
        check(hole.cancelled && hole.machining.operations.empty() &&
            hasDiagnostic(hole.diagnostics, ToolpathDiagnosticCode::GenerationCancelled),
            "hole algorithm/callback cancellation discards generated operations");
        check(pocket.cancelled && pocket.operations.empty() && pocket.failures.empty() &&
            pocket.regionOutcomes.empty() && pocketCalls == 2 &&
            hasDiagnostic(pocket.diagnostics, ToolpathDiagnosticCode::GenerationCancelled),
            "pocket cancellation discards the whole batch and stops subsequent regions");
        check(profile.cancelled && profile.operations.empty() && profile.failures.empty() &&
            !hasDiagnostic(profile.diagnostics, ToolpathDiagnosticCode::GenerationCancelled),
            "profile cancellation clears results without introducing a batch diagnostic");
    }
}

void holeInputMapping()
{
    Fixture fixture;
    auto unresolvedSetup = HoleMachiningInput{};
    unresolvedSetup.mappingState = HoleMachiningMappingState::UnresolvedSetup;
    fixture.hole->run = [](const auto& holes, const auto* strategies, const auto*) {
        check(holes.size() == 2 && strategies && strategies->size() == 2 &&
            (*strategies)[0] == HoleMachiningStrategy::MediumHelical &&
            (*strategies)[1] == HoleMachiningStrategy::Auto,
            "hole input filtering aligns overrides and defaults missing entries");
        HoleMachiningResult result;
        result.setupCount = 1;
        HoleMachiningOperation operation;
        operation.sourceHoleIndex = 1;
        operation.coveredSourceHoleIndices = {0, 1, 99};
        result.operations.push_back(operation);
        return result;
    };
    const std::vector<HoleMachiningStrategy> overrides = {
        HoleMachiningStrategy::SmallDrill, HoleMachiningStrategy::MediumHelical};
    const auto result = HoleToolpathService(fixture.registry, kAlgorithm).generate(
        {HoleMachiningInput{}, Fixture::readyHole(), unresolvedSetup, Fixture::readyHole()},
        fixture.params, nullptr, &overrides, &fixture.control);
    check(result.machining.operations.size() == 1 && result.machining.skippedCount == 2,
        "hole mapping retains ready operation and counts both unresolved inputs");
    if (!result.machining.operations.empty()) {
        const auto& operation = result.machining.operations[0];
        check(operation.sourceHoleIndex == 3 && operation.coveredSourceHoleIndices == std::vector<int>({1, 3}) &&
            operation.algorithmId == kAlgorithm, "hole source indices remap and invalid coverage indices are removed");
    }
}

void explicitBudgetPropagation()
{
    Fixture fixture;
    int acquisitions = 0;
    fixture.control.tryAcquireParallelWorkers = [&](std::size_t) { ++acquisitions; return 0u; };
    const auto inspect = [&](const ToolpathGenerationControl* control) {
        check(hasToolpathParallelBudget(control), "algorithm receives explicit complete budget");
        const int before = acquisitions;
        ToolpathParallelLease explicitLease(control, 1, 1);
        ToolpathParallelLease inheritedLease(nullptr, 1, 1);
        check(explicitLease.count() == 0 && inheritedLease.count() == 0 && acquisitions == before + 2,
            "explicit zero-worker budget is authoritative in algorithm and nested synchronous lease");
    };
    fixture.hole->run = [&](const auto&, const auto*, const auto* control) {
        inspect(control); return HoleMachiningResult{};
    };
    fixture.pocket->run = [&](const auto* control) {
        inspect(control); return PocketClearingResult{};
    };
    fixture.profile->run = [&](const auto* control) {
        inspect(control); return ProfileContourResult{};
    };
    const auto* previous = currentToolpathParallelControl();
    fixture.holes();
    fixture.pockets();
    fixture.profiles();
    check(currentToolpathParallelControl() == previous, "service scope restores caller parallel control");
}

void profileInvalidParametersRegression()
{
    Fixture fixture;
    fixture.params.tool.diameter = 0.0;
    int calls = 0;
    fixture.profile->run = [&](const auto*) { ++calls; return profileSuccess(); };
    const auto batch = fixture.profiles(2);
    check(calls == 0 && batch.operations.empty() && batch.failures.empty() &&
        hasDiagnostic(batch.diagnostics, ToolpathDiagnosticCode::InvalidToolDiameter),
        "profile invalid parameters must stop before algorithm invocation");
}

// Stable geometry fingerprint for comparing the already-built implementation to
// a structural refactor. Quantization ignores sub-tolerance floating point noise;
// execution times, cache counters and parallel scheduling metrics are excluded.
std::uint64_t contourFingerprint(const ProfileContourResult& result)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    const auto number = [&](double value) { out << std::llround(value * 1.0e7) << ','; };
    const auto point = [&](const gp_Pnt& value) { number(value.X()); number(value.Y()); number(value.Z()); };
    const auto vector = [&](const gp_Vec& value) { number(value.X()); number(value.Y()); number(value.Z()); };
    const auto move = [&](const ToolpathMove& value) {
        out << static_cast<int>(value.type) << ',' << static_cast<int>(value.intent) << ','
            << value.loopSequence << ',' << value.depthIndex << ',' << value.ring << ',';
        point(value.from); point(value.to); number(value.feed);
    };
    const auto boundaries = [&](const SharedProfileBoundaries& value) {
        out << static_cast<bool>(value) << ',';
        if (!value) return;
        out << value->size() << ',';
        for (const auto& boundary : *value) {
            out << boundary.size() << ',';
            for (const auto& p : boundary) { number(p.X()); number(p.Y()); }
        }
    };
    out << result.ok << ',' << result.cancelled << ',' << static_cast<int>(result.failureCode) << ','
        << result.warning.size() << ':' << result.warning << ',' << result.error.size() << ':' << result.error << ','
        << result.depthLayerCount << ',' << result.radialPassCount << ',' << result.clearingPassCount << ','
        << result.finishingPassCount << ',' << result.usedExternalClearing << ',' << result.restMachining << ','
        << result.noRestMaterial << ',' << result.evaluationOnly << ',' << result.directCutLinkCount << ','
        << result.retractEntryCount << ',' << result.travelReorderedLoopCount << ','
        << result.travelAdjustedStartCount << ',';
    for (const double value : {result.restMachiningPreviousToolDiameter, result.toolRadius,
        result.finalOffset, result.radialStepOver, result.safeHeightOffset,
        result.clearanceHeightOffset, result.estimatedCuttingDistance}) number(value);
    out << result.loops.size() << ',';
    for (const auto& loop : result.loops) {
        out << loop.closed << ',' << static_cast<int>(loop.passKind) << ',' << loop.depthIndex << ','
            << loop.radialPassIndex << ',' << loop.sequence << ',';
        number(loop.depthOffset); number(loop.offset); number(loop.area);
        out << loop.points.size() << ',';
        for (const auto& p : loop.points) { number(p.X()); number(p.Y()); }
        out << loop.world.size() << ',';
        for (const auto& p : loop.world) point(p);
        boundaries(loop.materialBoundaries); boundaries(loop.linkSafetyBoundaries);
    }
    out << result.moves.size() << ',';
    for (const auto& value : result.moves) move(value);
    out << result.primitives.size() << ',';
    for (const auto& primitive : result.primitives) {
        out << static_cast<int>(primitive.type) << ',' << primitive.context.loopSequence << ','
            << primitive.context.depthIndex << ',' << primitive.context.ring << ','
            << static_cast<int>(primitive.context.intent) << ',';
        switch (primitive.type) {
        case ToolpathPrimitiveType::Linear:
            move(primitive.linear);
            break;
        case ToolpathPrimitiveType::CircularArc: {
            const auto& arc = primitive.arc;
            out << static_cast<int>(arc.moveType) << ',' << static_cast<int>(arc.direction) << ',';
            point(arc.from); point(arc.to); point(arc.center); vector(arc.axis);
            number(arc.sweepRadians); number(arc.feed);
            break;
        }
        case ToolpathPrimitiveType::DrillCycle: {
            const auto& cycle = primitive.drillCycle;
            out << static_cast<int>(cycle.type) << ',' << static_cast<int>(cycle.retractMode) << ','
                << static_cast<int>(cycle.source) << ',';
            point(cycle.clearance); point(cycle.retract); point(cycle.entry); point(cycle.bottom); vector(cycle.axis);
            for (const double value : {cycle.nominalDepth, cycle.depthDiameterRatio, cycle.peckDepth,
                cycle.chipBreakRetract, cycle.reentryClearance, cycle.dwellSeconds, cycle.feed}) number(value);
            break;
        }
        }
        out << primitive.fallbackMoves.size() << ',';
        for (const auto& value : primitive.fallbackMoves) move(value);
    }
    std::uint64_t hash = UINT64_C(14695981039346656037);
    for (const unsigned char byte : out.str()) {
        hash ^= byte;
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

void profileSnapshots()
{
    Fixture fixture;
    fixture.control.profileEvaluationOnly = false;
    ProfileRegion region;
    region.openDir = gp_Dir(0.0, 0.0, 1.0);
    region.frame = gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), region.openDir, gp_Dir(1.0, 0.0, 0.0));
    region.depth = 6.0;
    region.boundary.closed = true;
    region.boundary.points = {{0, 0}, {40, 0}, {40, 30}, {0, 30}};
    region.boundary.signedArea = 1200.0;
    auto params = fixture.params;
    params.tool.diameter = 4.0;
    params.tool.horizFeed = 600.0;
    params.tool.vertFeed = 180.0;
    params.depth.useFeatureDepth = true;
    params.depth.stepDown = 2.0;
    params.depth.safeHeight = 3.0;
    params.depth.clearanceHeight = 5.0;
    params.profile.machiningMode = ProfileMachiningMode::ContourOnly;
    params.profile.radialPassCount = 3;
    params.profile.radialStepOver = 1.0;
    params.profile.sideAllowance = 0.2;
    const auto emit = [](const char* name, const ProfileContourResult& result) {
        check(result.ok && !result.loops.empty(), name);
        std::cout << name << " ok=" << result.ok << " layers=" << result.depthLayerCount
            << " loops=" << result.loops.size() << " moves=" << result.moves.size()
            << " primitives=" << result.primitives.size() << " fingerprint="
            << std::hex << contourFingerprint(result) << std::dec << '\n';
    };
    emit("rectangle-contour", ProfileContour().generate(region, params, &fixture.control));
    params.profile.cutMode = CutMode::Conventional;
    params.profile.leadInLength = 1.0;
    params.profile.leadOutLength = 1.0;
    emit("rectangle-conventional-leads", ProfileContour().generate(region, params, &fixture.control));
    params.profile.cutMode = CutMode::Climb;
    params.profile.leadInLength = 0.0;
    params.profile.leadOutLength = 0.0;
    region.stockBoundary = region.boundary;
    region.boundary.points = {{0, 0}, {40, 0}, {40, 30}, {25, 30},
        {25, 22}, {15, 22}, {15, 30}, {0, 30}};
    region.boundary.signedArea = 1120.0;
    params.profile.machiningMode = ProfileMachiningMode::ExternalClearing;
    params.profile.roughingAllowance = 0.2;
    params.profile.finishPass = true;
    params.clearing.pattern = ClearingPattern::ZigZag;
    params.clearing.stepOverPercent = 40.0;
    emit("notch-external-clearing", ProfileContour().generate(region, params, &fixture.control));
    fixture.control.profileEvaluationOnly = true;
    auto evaluation = ProfileContour().generate(region, params, &fixture.control);
    emit("notch-evaluation", evaluation);
    fixture.control.profileEvaluationOnly = false;
    emit("notch-materialized", ProfileContour().materializeEvaluation(
        region, params, std::move(evaluation), &fixture.control));
    params.clearing.restMachiningPreviousToolDiameter = 8.0;
    emit("notch-rest-machining", ProfileContour().generate(region, params, &fixture.control));
}
} // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::string_view(argv[1]) == "--profile-invalid-parameters") {
        profileInvalidParametersRegression();
    } else if (argc > 1 && std::string_view(argv[1]) == "--profile-snapshot") {
        profileSnapshots();
    } else {
        unavailableAndDiagnosticOrder();
        invalidParametersAndCancellationPrecedence();
        exceptionMappingAndPartialSuccess();
        cancellationDiscardsPartialResults();
        holeInputMapping();
        explicitBudgetPropagation();
    }
    if (failures == 0)
        std::cout << "workflow public contracts passed\n";
    return failures == 0 ? 0 : 1;
}

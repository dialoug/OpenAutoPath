// 纯孔几何 -> 显式加工决策 -> 当前加工输入的映射契约。
#include "tooling/adapter/hole/HoleMachiningInputMapper.h"
#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"
#include "tooling/adapter/AssignedToolContext.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"

#include <algorithm>
#include <iostream>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

oap::toolpath::HoleMachiningInput readyInput()
{
    oap::toolpath::HoleMachiningInput hole;
    hole.kind = oap::FeatureKind::CountersinkHole;
    hole.manuallyConfirmed = true;
    hole.entry = gp_Pnt(1.0, 2.0, 12.0);
    hole.bottom = gp_Pnt(1.0, 2.0, 3.0);
    hole.radius = 2.5;
    hole.bottomProfile = oap::HoleBottomProfile::Planar;
    hole.entryResolution = oap::HoleEntryResolution::TopologyOpening;
    hole.setupFace = oap::MoldSetupFace::PositiveZ;
    hole.setupFaceEntryDistance = 0.1;
    hole.setupFaceOtherEndpointDistance = 8.9;
    hole.setupFaceDecision = oap::MoldSetupFaceDecision::TopologyTieBreak;
    hole.preserveEntrySide = true;
    hole.sourceFaceIds = {"F4", "F7"};
    hole.stageWallFaceIds = {"F4"};
    hole.stageRole = oap::HoleStageRole::CountersinkLargeDiameter;
    hole.parentRelation = oap::HoleParentRelation::CoaxialStage;
    hole.parentCounterboreFaceId = "F2";
    hole.parentCounterboreFloorFaceId = "F3";
    hole.parentCounterboreRadius = 4.0;
    hole.mappingState = oap::toolpath::HoleMachiningMappingState::Ready;
    return hole;
}

oap::toolpath::HoleMachiningInput remap(
    const oap::toolpath::HoleMachiningInput& source)
{
    return oap::tooling::HoleMachiningInputMapper().map(
        static_cast<const oap::RecognizedHoleGeometry&>(source),
        static_cast<const oap::toolpath::HoleMachiningDecision&>(source));
}

bool mapsResolvedInputWithoutLosingFields()
{
    const oap::toolpath::HoleMachiningInput source = readyInput();
    const oap::toolpath::HoleMachiningInput input = remap(source);
    if (!expect(input.ready() &&
                    input.mappingState ==
                        oap::toolpath::HoleMachiningMappingState::Ready,
                "resolved decisions should produce a ready machining input") ||
        !expect(input.depth() == 9.0 &&
                    input.sourceFaceIds == source.sourceFaceIds &&
                    input.stageWallFaceIds == source.stageWallFaceIds,
                "recognized geometry should retain only geometric/topology facts")) {
        return false;
    }

    return expect(input.kind == source.kind &&
                      input.manuallyConfirmed == source.manuallyConfirmed &&
                      input.entry.Distance(source.entry) < 1.0e-12 &&
                      input.bottom.Distance(source.bottom) < 1.0e-12 &&
                      input.radius == source.radius &&
                      input.entryResolution == source.entryResolution &&
                      input.setupFace == source.setupFace &&
                      input.preserveEntrySide == source.preserveEntrySide &&
                      input.parentCounterboreFaceId ==
                          source.parentCounterboreFaceId,
                  "explicit mapping should preserve current behavior");
}

bool unresolvedStateNeverDefaultsSilently()
{
    oap::toolpath::HoleMachiningInput source = readyInput();

    source.entryResolution = oap::HoleEntryResolution::Unresolved;
    oap::toolpath::HoleMachiningInput unresolvedEntry =
        remap(source);
    if (!expect(!unresolvedEntry.ready() &&
                    unresolvedEntry.mappingState ==
                        oap::toolpath::HoleMachiningMappingState::UnresolvedEntry,
                "unresolved entry must block machining")) {
        return false;
    }

    source = readyInput();
    source.setupFace = oap::MoldSetupFace::Unresolved;
    const oap::toolpath::HoleMachiningInput unresolvedSetup =
        remap(source);
    return expect(!unresolvedSetup.ready() &&
                      unresolvedSetup.mappingState ==
                          oap::toolpath::HoleMachiningMappingState::UnresolvedSetup &&
                      unresolvedSetup.setupFace ==
                          oap::MoldSetupFace::Unresolved,
                   "unresolved setup must stay explicit instead of defaulting");
}

oap::tooling::ToolLibrary drillLibrary()
{
    oap::tooling::ToolLibrary library(
        QStringLiteral("dto-test"), QStringLiteral("DTO test"));
    oap::tooling::ToolBit drill;
    drill.id = QStringLiteral("drill-5");
    drill.name = QStringLiteral("D5 drill");
    drill.type = oap::tooling::ToolType::Drill;
    drill.geometry.diameter = 5.0;
    drill.geometry.cuttingEdgeLength = 20.0;
    drill.geometry.overallLength = 30.0;
    drill.geometry.tipAngle = 118.0;
    drill.geometry.tipCompensation = 0.4;
    drill.geometry.shankDiameter = 6.0;
    drill.geometry.holderDiameter = 28.0;
    drill.geometry.holderLength = 40.0;
    drill.geometry.spindleEnvelopeDiameter = 60.0;
    drill.geometry.spindleEnvelopeLength = 80.0;
    library.upsertTool(std::move(drill));

    oap::tooling::LibraryToolSlot slot;
    slot.toolNumber = 5;
    slot.toolId = QStringLiteral("drill-5");
    slot.cuttingDefaults.verticalFeed = 100.0;
    slot.cuttingDefaults.horizontalFeed = 200.0;
    slot.cuttingDefaults.spindleSpeed = 3000.0;
    slot.cuttingDefaults.horizontalRapid = 1200.0;
    slot.cuttingDefaults.verticalRapid = 800.0;
    slot.cuttingDefaults.peckDepth = 2.0;
    slot.cuttingDefaults.drillCycle = oap::tooling::DrillCycle::G83Peck;
    slot.coolantDefault = oap::tooling::CoolantMode::Flood;
    library.assignToolToSlot(std::move(slot));
    return library;
}

bool assignedToolCreatesIndependentCamSnapshot()
{
    oap::tooling::ToolLibrary library = drillLibrary();
    QVector<oap::tooling::AssignedTool> assigned = library.assignedTools();
    if (!expect(assigned.size() == 1,
                "the test library should expose one occupied T slot")) {
        return false;
    }

    const oap::toolpath::ToolpathToolContext snapshot =
        oap::tooling::makeToolpathToolContext(assigned.front());
    assigned.front().tool.geometry.diameter = 99.0;
    assigned.front().slot.cuttingDefaults.spindleSpeed = 1.0;

    return expect(snapshot.valid && snapshot.toolNumber == 5 &&
                      snapshot.toolId == "drill-5" &&
                      snapshot.diameter == 5.0 &&
                      snapshot.spindleSpeed == 3000.0 &&
                      snapshot.horizontalFeed == 200.0 &&
                      snapshot.verticalFeed == 100.0 &&
                      snapshot.horizontalRapid == 1200.0 &&
                      snapshot.verticalRapid == 800.0 &&
                      snapshot.tipAngle == 118.0 &&
                      snapshot.tipCompensation == 0.4 &&
                      snapshot.peckDepth == 2.0 &&
                      snapshot.drillCycle ==
                          oap::toolpath::ToolpathDrillCyclePreference::Peck &&
                      snapshot.coolant ==
                          oap::toolpath::ToolpathCoolantMode::Flood,
                  "library defaults must become a complete independent CAM snapshot");
}

bool automaticPlanningUsesExplicitInputs()
{
    oap::toolpath::HoleMachiningInput planningHole = readyInput();
    planningHole.kind = oap::FeatureKind::BlindHole;
    planningHole.bottomProfile = oap::HoleBottomProfile::Conical;
    planningHole.stageRole = oap::HoleStageRole::Standard;
    planningHole.parentRelation = oap::HoleParentRelation::None;
    planningHole.parentCounterboreFaceId.clear();
    planningHole.parentCounterboreFloorFaceId.clear();
    planningHole.parentCounterboreRadius = 0.0;
    const oap::toolpath::HoleMachiningInput ready = remap(planningHole);
    oap::toolpath::HoleMachiningInput unresolvedEntry = ready;
    unresolvedEntry.entryResolution =
        oap::HoleEntryResolution::Unresolved;
    unresolvedEntry.mappingState =
        oap::toolpath::HoleMachiningMappingState::UnresolvedEntry;
    oap::toolpath::HoleMachiningInput unresolvedSetup = ready;
    unresolvedSetup.setupFace = oap::MoldSetupFace::Unresolved;
    unresolvedSetup.mappingState =
        oap::toolpath::HoleMachiningMappingState::UnresolvedSetup;

    oap::toolpath::ToolpathParams params;
    params.hole.useCenterSpot = false;
    params.hole.smallHoleDiameterTolerance = 0.01;
    const std::vector<oap::toolpath::HoleMachiningInput> inputs{
        ready, unresolvedEntry, unresolvedSetup};
    const oap::tooling::HoleToolSelectionPlan plan =
        oap::tooling::HoleToolSelectionPlanner().plan(
            drillLibrary(), inputs, params);

    std::vector<std::size_t> sourceIndices;
    for (const oap::tooling::HoleToolBatch& batch : plan.batches) {
        for (const oap::tooling::HoleToolTask& task : batch.tasks)
            sourceIndices.push_back(task.sourceHoleIndex);
    }
    const auto hasDiagnostic =
        [&plan](oap::toolpath::ToolpathDiagnosticCode code, int sourceIndex) {
            return std::any_of(
                plan.diagnostics.begin(), plan.diagnostics.end(),
                [code, sourceIndex](
                    const oap::toolpath::ToolpathDiagnostic& diagnostic) {
                    return diagnostic.code == code &&
                           diagnostic.sourceIndex == sourceIndex;
                });
        };
    return expect(
        sourceIndices == std::vector<std::size_t>{0},
        "only Ready inputs should enter automatic planning") &&
        expect(
            hasDiagnostic(
                oap::toolpath::ToolpathDiagnosticCode::HoleOpeningUnresolved,
                1),
            "unresolved entry should retain its source index and blocking diagnostic") &&
        expect(
            hasDiagnostic(
                oap::toolpath::ToolpathDiagnosticCode::HoleOutsideSixFace,
                2),
            "unresolved setup should retain its source index and six-face warning");
}

bool toolpathServicePreservesInputSourceIndices()
{
    oap::toolpath::HoleMachiningInput source = readyInput();
    source.kind = oap::FeatureKind::BlindHole;
    source.bottomProfile = oap::HoleBottomProfile::Conical;
    source.stageRole = oap::HoleStageRole::Standard;
    source.parentRelation = oap::HoleParentRelation::None;
    const oap::toolpath::HoleMachiningInput ready = remap(source);
    oap::toolpath::HoleMachiningInput unresolvedSetup = ready;
    unresolvedSetup.setupFace = oap::MoldSetupFace::Unresolved;
    unresolvedSetup.mappingState =
        oap::toolpath::HoleMachiningMappingState::UnresolvedSetup;

    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 5.0;
    params.tool.cuttingEdgeLength = 20.0;
    params.tool.overallLength = 30.0;
    params.tool.vertFeed = 100.0;
    params.tool.horizFeed = 200.0;
    params.tool.spindleSpeed = 3000.0;
    params.depth.safeHeight = 2.0;
    params.depth.clearanceHeight = 4.0;
    params.hole.useCenterSpot = false;
    params.hole.drillTipAngleDeg = 118.0;
    params.hole.drillCycleMode =
        oap::toolpath::HoleDrillCycleMode::Direct;
    const std::vector<oap::toolpath::HoleMachiningInput> inputs{
        unresolvedSetup, ready};
    const std::vector<oap::toolpath::HoleMachiningStrategy> strategies{
        oap::toolpath::HoleMachiningStrategy::Auto,
        oap::toolpath::HoleMachiningStrategy::SmallDrill};
    const oap::toolpath::HoleToolpathBatch batch =
        oap::toolpath::HoleToolpathService().generate(
            inputs, params, nullptr, &strategies);
    const bool mappedWarning = std::any_of(
        batch.diagnostics.begin(), batch.diagnostics.end(),
        [](const oap::toolpath::ToolpathDiagnostic& diagnostic) {
            return diagnostic.code ==
                       oap::toolpath::ToolpathDiagnosticCode::
                           HoleOutsideSixFace &&
                   diagnostic.sourceIndex == 0;
        });
    return expect(
        batch.machining.operations.size() == 1 &&
            batch.machining.operations[0].sourceHoleIndex == 1,
        "filtered inputs must retain original source indices in generated operations") &&
        expect(
            mappedWarning && batch.coverage.complete() &&
                batch.coverage.outsideSixFaceSourceHoleIndices ==
                    std::vector<int>{0},
            "input-aware generation should audit unresolved setups without treating them as machined");
}

} // namespace

int main()
{
    if (!mapsResolvedInputWithoutLosingFields() ||
        !unresolvedStateNeverDefaultsSilently() ||
        !assignedToolCreatesIndependentCamSnapshot() ||
        !automaticPlanningUsesExplicitInputs() ||
        !toolpathServicePreservesInputSourceIndices()) {
        return 1;
    }
    std::cout << "hole DTO mapping smoke tests passed\n";
    return 0;
}

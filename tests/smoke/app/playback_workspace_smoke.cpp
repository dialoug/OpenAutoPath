#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/playback/PlaybackCameraFollower.h"
#include "app/playback/PlaybackPanelViewModel.h"

#include "app/presentation/OverlayController.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "app/workflows/playback/PlaybackProgramAssembler.h"
#include "analysis/StepIndexer.h"

#include <AIS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <QCoreApplication>

#include <iostream>
#include <memory>
#include <string_view>
#include <unordered_set>

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

class RecordingSink final :
    public oap::app::presentation::PresentationSink {
public:
    void display(const Handle(AIS_InteractiveObject)& object) override
    {
        displayed.insert(object.get());
    }
    void erase(const Handle(AIS_InteractiveObject)& object) override
    {
        displayed.erase(object.get());
    }
    void remove(const Handle(AIS_InteractiveObject)& object) override
    {
        displayed.erase(object.get());
    }
    void redisplay(const Handle(AIS_InteractiveObject)&) override {}
    void deactivate(const Handle(AIS_InteractiveObject)&) override {}
    void setDisplayPriority(
        const Handle(AIS_InteractiveObject)&, int) override {}
    bool isDisplayed(
        const Handle(AIS_InteractiveObject)& object) const override
    {
        return displayed.contains(object.get());
    }

    std::unordered_set<const AIS_InteractiveObject*> displayed;
};

Handle(AIS_InteractiveObject) box(double x)
{
    Handle(AIS_Shape) shape = new AIS_Shape(
        BRepPrimAPI_MakeBox(
            gp_Pnt(x, 0.0, 0.0), 1.0, 1.0, 1.0).Shape());
    return shape;
}

oap::toolpath::ToolpathProgram twoToolProgram()
{
    oap::toolpath::ToolpathProgram program;
    oap::toolpath::ToolpathOperation first;
    first.type = oap::toolpath::ToolpathOperationType::HoleMachining;
    first.featureKind = oap::FeatureKind::BlindHole;
    first.operationId = "operation-a";
    first.tool.valid = true;
    first.tool.toolNumber = 1;
    for (int index = 0; index < 2; ++index) {
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = gp_Pnt(index, 0.0, 0.0);
        move.to = gp_Pnt(index + 1.0, 0.0, 0.0);
        first.moves.push_back(move);
    }
    oap::toolpath::ToolpathOperation second = first;
    second.operationId = "operation-b";
    second.tool.toolNumber = 2;
    second.moves.resize(1);
    second.moves.front().from = gp_Pnt(2.0, 0.0, 0.0);
    second.moves.front().to = gp_Pnt(3.0, 0.0, 0.0);
    program.operations.push_back(std::move(first));
    program.operations.push_back(std::move(second));
    return program;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    const auto sink = std::make_shared<RecordingSink>();
    oap::app::presentation::OverlayController overlays(sink);
    oap::app::PlaybackWorkspaceCoordinator playback(
        overlays, Handle(AIS_InteractiveContext){}, Handle(V3d_View){});

    const oap::toolpath::ToolpathProgram program = twoToolProgram();
    playback.replaceProgram(program, oap::app::PlaybackProgramBuildOptions{});
    if (!expect(
            playback.moveCount() == 3 &&
                playback.toolBatchStarts() == std::vector<int>({0, 2}),
            "workspace must atomically install moves and tool batches")) {
        return 1;
    }
    playback.setOperationVisible("operation-a", false);
    playback.setSoloOperation("operation-b");
    if (!expect(
            !playback.operationVisible("operation-a") &&
                playback.operationVisible("operation-b") &&
                playback.soloOperationId() == "operation-b",
            "workspace must retain operation hide and solo display state")) {
        return 1;
    }
    playback.replaceProgram(program, oap::app::PlaybackProgramBuildOptions{});
    if (!expect(
            playback.operationVisible("operation-a") &&
                playback.soloOperationId().empty(),
            "installing a new program must reset operation display filters")) {
        return 1;
    }

    oap::app::PlaybackPanelViewModelInput panelInput;
    panelInput.moves = &playback.moves();
    panelInput.toolBatchStarts = &playback.toolBatchStarts();
    panelInput.index = 2;
    panelInput.fraction = 1.0;
    const oap::app::PlaybackPanelViewModel batchBoundary =
        oap::app::PlaybackPanelViewModelBuilder::build(panelInput);
    if (!expect(
            batchBoundary.informationMoveIndex == 2 &&
                batchBoundary.previousToolEnabled &&
                !batchBoundary.nextToolEnabled &&
                batchBoundary.toolText.contains(QStringLiteral("T2")),
            "an exact batch boundary must announce the upcoming tool")) {
        return 1;
    }

    panelInput.index = playback.moveCount();
    const oap::app::PlaybackPanelViewModel completed =
        oap::app::PlaybackPanelViewModelBuilder::build(panelInput);
    if (!expect(
            completed.informationMoveIndex == 2 &&
                completed.infoText.startsWith(QStringLiteral("第 3/3 步")),
            "the completed position must retain the final move context")) {
        return 1;
    }

    std::vector<oap::app::PlaybackMove> manualMoves(1);
    manualMoves.front().kind = oap::FeatureKind::BlindHole;
    manualMoves.front().move.type = oap::toolpath::ToolpathMoveType::Cut;
    manualMoves.front().toolBatchIndex = -1;
    panelInput.moves = &manualMoves;
    panelInput.toolBatchStarts = nullptr;
    panelInput.index = 0;
    panelInput.fraction = 0.5;
    const oap::app::PlaybackPanelViewModel manual =
        oap::app::PlaybackPanelViewModelBuilder::build(panelInput);
    if (!expect(
            manual.toolText == QStringLiteral("未绑定刀具") &&
                !manual.previousToolEnabled && !manual.nextToolEnabled,
            "an unbound move must not invent a fallback tool")) {
        return 1;
    }

    panelInput.moves = nullptr;
    panelInput.index = -1;
    const oap::app::PlaybackPanelViewModel empty =
        oap::app::PlaybackPanelViewModelBuilder::build(panelInput);
    if (!expect(
            empty.navigationMoveIndex == -1 &&
                empty.phaseText == QStringLiteral("等待开始"),
            "an empty or negative boundary position must be safe")) {
        return 1;
    }

    const oap::app::PlaybackCameraPose centered =
        oap::app::PlaybackCameraFollower::recenter(
            {gp_Pnt(1.0, 2.0, 3.0), gp_Pnt(0.0, 0.0, 0.0)},
            gp_Pnt(4.0, 5.0, 6.0));
    if (!expect(
            centered.eye.Distance(gp_Pnt(5.0, 7.0, 9.0)) < 1.0e-9 &&
                centered.at.Distance(gp_Pnt(4.0, 5.0, 6.0)) < 1.0e-9 &&
                !oap::app::PlaybackCameraFollower::follow(
                    Handle(V3d_View){}, gp_Pnt{}, true),
            "camera following must preserve the pose offset and protect null views")) {
        return 1;
    }

    oap::app::HolePreviewEntry holePreview;
    holePreview.sourceIndex = 0;
    holePreview.operation.setupIndex = 0;
    holePreview.operation.entry = gp_Pnt(0.0, 0.0, 0.0);
    holePreview.operation.coveredSourceHoleIndices = {0};
    oap::toolpath::ToolpathMove previewMove;
    previewMove.type = oap::toolpath::ToolpathMoveType::Cut;
    previewMove.from = gp_Pnt(0.0, 0.0, 0.0);
    previewMove.to = gp_Pnt(1.0, 0.0, 0.0);
    holePreview.operation.moves.push_back(previewMove);
    const std::vector<oap::app::HolePreviewEntry> holePreviews{
        holePreview};

    const oap::app::ToolpathPreviewBuildStats previewStats =
        playback.replaceHolePreview(holePreviews, gp_Trsf{});
    if (!expect(
            previewStats.objectCount > 0 &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::Toolpath) > 0,
            "workspace must own and publish the full static preview")) {
        return 1;
    }

    playback.setCurrentToolOnly(true);
    const oap::app::PlaybackToolFilterChange presentedBatch =
        playback.updateCurrentToolFilterPresentation(
            0, true, true, gp_Trsf{});
    if (!expect(
            presentedBatch.kind ==
                    oap::app::PlaybackToolFilterChangeKind::RebuildBatch &&
                !overlays.visible(
                    oap::app::presentation::OverlayGroup::Toolpath) &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::CurrentToolpath) > 0,
            "current-tool filter must replace the full preview atomically")) {
        return 1;
    }

    playback.clearCurrentToolpathPreview();
    playback.applyStaticOverlayVisibility(0, 1.0);
    const auto missingCurrentFallback =
        playback.visibilityForPosition(0, 1.0);
    if (!expect(
            missingCurrentFallback.staticOverlays.fullToolpath &&
                !missingCurrentFallback.staticOverlays.currentToolpath &&
                overlays.visible(
                    oap::app::presentation::OverlayGroup::Toolpath),
            "an empty current-tool preview must fall back to the full path")) {
        return 1;
    }

    playback.setCurrentToolOnly(false);
    playback.updateCurrentToolFilterPresentation(
        0, true, true, gp_Trsf{});
    if (!expect(
            overlays.visible(
                oap::app::presentation::OverlayGroup::Toolpath) &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::CurrentToolpath) == 0,
            "disabling current-tool filter must restore the full preview")) {
        return 1;
    }

    oap::app::RecognizedHoleSource source;
    source.sourceFaceIds = {"missing-face"};
    const std::vector<oap::app::RecognizedHoleSource> sources{source};
    const oap::StepIndexer emptyIndexer;
    const oap::app::HoleStageHighlightStats highlightStats =
        playback.replaceHoleStageHighlight(
            holePreviews, sources, {0}, emptyIndexer,
            [](const std::string&) { return false; }, gp_Trsf{}, true);
    if (!expect(
            highlightStats.moveCount == 1 &&
                highlightStats.operationCount == 1 &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::HoleStageHighlight) > 0,
            "workspace must own hole-stage path presentation without a wall")) {
        return 1;
    }

    const auto start = playback.visibilityForPosition(0, 1.0);
    const auto playing = playback.visibilityForPosition(1, 1.0);
    if (!expect(
            start.staticOverlays.fullToolpath && !start.remainingPath &&
                !playing.staticOverlays.fullToolpath &&
                playing.remainingPath && playing.hasVisibleToolpath(),
            "nonzero playback must replace static paths with a remaining path")) {
        return 1;
    }

    playback.setCurrentToolOnly(true);
    const oap::app::PlaybackToolFilterChange batchChange =
        playback.updateCurrentToolFilter(2);
    if (!expect(
            batchChange.kind ==
                    oap::app::PlaybackToolFilterChangeKind::RebuildBatch &&
                batchChange.batchIndex == 1,
            "current-tool filtering must publish one stable batch change")) {
        return 1;
    }

    overlays.replace(
        oap::app::presentation::OverlayGroup::PlaybackRemainingPath,
        {box(0.0)});
    overlays.replace(
        oap::app::presentation::OverlayGroup::PlaybackTrail,
        {box(2.0)});
    overlays.replace(
        oap::app::presentation::OverlayGroup::PlaybackCursor,
        {box(4.0)});
    playback.publishPosition(1, 1.0, gp_Trsf{}, true);
    if (!expect(
            overlays.size(oap::app::presentation::
                              OverlayGroup::PlaybackRemainingPath) == 1 &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::PlaybackTrail) == 0 &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::PlaybackCursor) == 0,
            "rewind must preserve remaining path while clearing cursor and trail")) {
        return 1;
    }

    playback.clearPresentation();
    if (!expect(
            overlays.size(oap::app::presentation::
                              OverlayGroup::PlaybackRemainingPath) == 0,
            "full presentation cleanup must remove the remaining path")) {
        return 1;
    }

    playback.clearStaticToolpathPresentation();
    if (!expect(
            overlays.size(oap::app::presentation::OverlayGroup::Toolpath) == 0 &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::CurrentToolpath) == 0 &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::HoleStageHighlight) == 0,
            "static presentation cleanup must remove every derived group")) {
        return 1;
    }

    // Query the actual remaining-path state through the public workspace. Hidden
    // moves retain their original positions so seek/fraction/tool ranges do
    // not shift when a display-only filter changes during playback.
    playback.replaceProgram(program, oap::app::PlaybackProgramBuildOptions{});
    playback.setCurrentToolOnly(false);
    playback.setOperationVisible("operation-a", false);
    playback.publishPosition(1, 0.25, gp_Trsf{}, false);
    const auto remaining = overlays.first(
        oap::app::presentation::OverlayGroup::PlaybackRemainingPath);
    if (!expect(!remaining.IsNull() && !playback.remainingSegmentVisible(0) &&
                    !playback.remainingSegmentVisible(1) && playback.remainingSegmentVisible(2),
                "lazy remaining-path creation must apply hidden operations without shifting move indices"))
        return 1;
    playback.setOperationVisible("operation-a", true);
    if (!expect(playback.remainingSegmentVisible(1) && playback.remainingSegmentVisible(2),
                "showing an operation must update the cached remaining path immediately"))
        return 1;
    playback.setSoloOperation("operation-b");
    if (!expect(!playback.remainingSegmentVisible(1) && playback.remainingSegmentVisible(2),
                "solo must also filter the cached dynamic path"))
        return 1;
    playback.setSoloOperation({});
    playback.setCurrentToolOnly(true);
    playback.publishPosition(1, 0.5, gp_Trsf{}, false);
    if (!expect(playback.remainingSegmentVisible(1) && !playback.remainingSegmentVisible(2),
                "current-tool range must remain in original move coordinates"))
        return 1;
    playback.setOperationVisible("operation-a", false);
    if (!expect(!playback.remainingSegmentVisible(1) && !playback.remainingSegmentVisible(2),
                "hiding the current tool operation must leave no remaining segments in its range"))
        return 1;
    playback.publishPosition(2, 0.5, gp_Trsf{}, false);
    if (!expect(!playback.remainingSegmentVisible(1) && playback.remainingSegmentVisible(2) &&
                    overlays.first(oap::app::presentation::OverlayGroup::PlaybackRemainingPath).get() == remaining.get(),
                "seeking across a hidden operation must preserve indices and reuse the filtered cache"))
        return 1;

    std::cout << "PASS: playback workspace coordination checks\n";
    return 0;
}

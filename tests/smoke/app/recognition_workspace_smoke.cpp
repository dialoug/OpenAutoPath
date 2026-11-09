#include "app/recognition/RecognitionWorkspaceCoordinator.h"

#include "analysis/MoldCoordinateFrame.h"
#include "analysis/Model.h"
#include "analysis/StepIndexer.h"
#include "app/presentation/SceneController.h"
#include "app/presentation/OverlayController.h"

#include <AIS_InteractiveObject.hxx>
#include <Bnd_Box.hxx>

#include <iostream>
#include <memory>
#include <string_view>
#include <thread>
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

oap::Model minimalModel()
{
    oap::Model model;
    oap::Unit unit;
    unit.unit_id = "U1";
    unit.bbox = {0.0, 0.0, 0.0, 10.0, 10.0, 2.0};
    model.units.push_back(unit);
    oap::Face face;
    face.face_id = "F1";
    face.unit_id = "U1";
    face.surface_type = "plane";
    model.faces.push_back(face);
    return model;
}

} // namespace

int main()
{
    const auto sink = std::make_shared<RecordingSink>();
    oap::app::presentation::OverlayController overlays(sink);
    oap::app::RecognitionWorkspaceCoordinator workspace(
        overlays,
        [](const oap::app::ThreeAxisRecognitionRequest&,
           const oap::app::ThreeAxisRecognitionTaskControl&) {
            oap::app::ThreeAxisRecognitionTaskResult result;
            result.recognition.holeCount = 7;
            return result;
        });

    const std::uint64_t requestId = workspace.start({});
    for (int attempt = 0;
         attempt < 10000 && !workspace.ready(requestId); ++attempt) {
        std::this_thread::yield();
    }
    const auto task = workspace.take(requestId);
    if (!expect(
            task && task->recognition.holeCount == 7,
            "workspace must own and publish recognition task results")) {
        return 1;
    }

    oap::SceneController scene;
    const oap::Model model = minimalModel();
    const oap::MoldCoordinateFrame frame =
        oap::MoldCoordinateFrame::fromBounds(model.units.front().bbox);
    oap::RecognitionResult recognition;
    recognition.holeCount = 1;
    recognition.faceKind["F1"] = oap::FeatureKind::BlindHole;
    oap::ThreeAxisWireTraceUseCaseResult wireTrace;
    wireTrace.throughStructureFaceIds.insert("F2");
    bool beforeCommitCalled = false;
    const oap::app::RecognitionInstallResult installed =
        workspace.installThreeAxisResult(
            recognition, wireTrace, model, frame, scene, 12,
            [&beforeCommitCalled] { beforeCommitCalled = true; });
    if (!expect(
            installed.installed() && beforeCommitCalled &&
                installed.hasCombinedPresentation &&
                workspace.hasResult() &&
                workspace.result().holeCount == 1 &&
                workspace.hasHoleToolpathInputs() &&
                installed.message.contains(QStringLiteral("耗时 12 ms")),
            "successful install must atomically replace all recognition state")) {
        return 1;
    }

    oap::HoleGeometryRecognitionResult holeOnlyRecognition;
    holeOnlyRecognition.aagBuildMs = 3;
    const auto& holeOnlyInputs =
        workspace.installHoleGeometryRecognition(
            std::move(holeOnlyRecognition), model, frame);
    if (!expect(
            workspace.hasHoleToolpathInputs() &&
                holeOnlyInputs.empty() &&
                workspace.holeGeometryProfile().aagBuildMs == 3,
            "hole-only recognition must publish its profile and mapped inputs through one workspace command")) {
        return 1;
    }

    const oap::StepIndexer emptyIndexer;
    const auto& pocketRegions =
        workspace.ensurePocketRegionCache(emptyIndexer);
    if (!expect(
            workspace.hasPocketRegionCache() &&
                pocketRegions.regions.empty(),
            "pocket-region derivation must publish its cache through the workspace")) {
        return 1;
    }

    oap::RecognitionResult replacement;
    replacement.holeCount = 99;
    oap::ThreeAxisWireTraceUseCaseResult cancelled;
    cancelled.cancelled = true;
    const oap::app::RecognitionInstallResult rejected =
        workspace.installThreeAxisResult(
            replacement, cancelled, model, frame, scene, 0);
    if (!expect(
            rejected.status ==
                    oap::app::RecognitionInstallStatus::Cancelled &&
                workspace.result().holeCount == 1,
            "cancelled install must preserve the last valid session")) {
        return 1;
    }

    const oap::app::ManualRecognitionEditResult manualEdit =
        workspace.applyManualFeatureEdit(
            oap::FeatureKind::BlindHole, {"F1"}, model);
    if (!expect(
            manualEdit.updated() &&
                manualEdit.affectedInstanceCount == 1 &&
                workspace.result().instances.size() == 1 &&
                workspace.result().instances.front().manual &&
                !workspace.hasHoleToolpathInputs(),
            "manual edit must atomically replace recognition instances and invalidate derived inputs")) {
        return 1;
    }
    const oap::app::ManualRecognitionEditResult cleared =
        workspace.clearFeatureSelection({"F1"});
    if (!expect(
            cleared.updated() && cleared.affectedFaceCount == 1 &&
                workspace.result().instances.empty(),
            "manual clear must remove complete matching instances")) {
        return 1;
    }

    oap::toolpath::HoleMachiningInput restoredHole;
    restoredHole.kind = oap::FeatureKind::BlindHole;
    restoredHole.sourceFaceIds.push_back("F1");
    workspace.synchronizeLoadedProjectHoles({restoredHole});
    if (!expect(
            workspace.hasHoleToolpathInputs() &&
                workspace.holeToolpathInputs().size() == 1 &&
                workspace.result().instances.size() == 1,
            "loaded-project holes must synchronize through the recognition facade")) {
        return 1;
    }

    const oap::app::BaseFaceRecognitionResult baseFaces =
        workspace.recognizeBaseFaces(model, emptyIndexer);
    if (!expect(
            baseFaces.installed && baseFaces.baseFaceCount == 0,
            "base-face result must be installed atomically even when no mapped shapes exist")) {
        return 1;
    }

    Bnd_Box bounds;
    bounds.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    if (!expect(
            workspace.appendInspectionBoundingBox(
                bounds,
                Quantity_Color(
                    1.0, 1.0, 0.0, Quantity_TOC_sRGB),
                1.0, gp_Trsf{}) &&
                overlays.size(oap::app::presentation::
                                  OverlayGroup::TemporaryHighlight) > 0,
            "workspace must own transient inspection overlays")) {
        return 1;
    }
    workspace.clearInspection();
    if (!expect(
            overlays.size(oap::app::presentation::
                              OverlayGroup::TemporaryHighlight) == 0,
            "inspection cleanup must remove its semantic overlay group")) {
        return 1;
    }

    workspace.resetForModel();
    if (!expect(
            !workspace.hasResult() &&
                workspace.throughStructureFaces().empty(),
            "model reset must clear recognition-derived state")) {
        return 1;
    }

    std::cout << "PASS: recognition workspace coordination checks\n";
    return 0;
}

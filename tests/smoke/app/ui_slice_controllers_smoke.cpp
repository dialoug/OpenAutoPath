#include "app/presentation/OverlayController.h"
#include "app/panels/ResultPanel.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/ui/cam/CamUiController.h"
#include "app/ui/document/DocumentUiController.h"
#include "app/ui/panels/CamWorkflowPanel.h"
#include "app/ui/panels/FeatureAllowancePanel.h"
#include "app/ui/panels/FileListPanel.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/ui/recognition/RecognitionSelectionUiController.h"
#include "app/ui/simulation/SimulationUiController.h"

#include <AIS_InteractiveObject.hxx>
#include <QApplication>
#include <QDockWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QTreeWidget>
#include <QLabel>

#include <iostream>
#include <memory>
#include <string_view>

namespace {
bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

class NullSink final : public oap::app::presentation::PresentationSink {
public:
    void display(const Handle(AIS_InteractiveObject)&) override {}
    void erase(const Handle(AIS_InteractiveObject)&) override {}
    void remove(const Handle(AIS_InteractiveObject)&) override {}
    void redisplay(const Handle(AIS_InteractiveObject)&) override {}
    void deactivate(const Handle(AIS_InteractiveObject)&) override {}
    void setDisplayPriority(const Handle(AIS_InteractiveObject)&, int) override {}
    bool isDisplayed(const Handle(AIS_InteractiveObject)&) const override
    {
        return false;
    }
};
}

int main(int argc, char** argv)
{
    QApplication application(argc, argv);
    QMainWindow window;

    int selectedFaces = 0;
    oap::app::RecognitionSelectionUiCallbacks recognitionCallbacks;
    recognitionCallbacks.faceSelected = [&](const QString&) { ++selectedFaces; };
    oap::app::RecognitionSelectionUiController recognition(
        window, std::move(recognitionCallbacks));
    recognition.resultPanel().faceSelected(QStringLiteral("face-1"));
    if (!expect(selectedFaces == 1, "recognition signal must cross its callback port") ||
        !expect(recognition.analysisDock().objectName() == QStringLiteral("analysisDock"),
                "recognition slice must own its stable dock identity")) {
        return 1;
    }

    if (!expect(recognition.analysisDock().windowTitle() == QStringLiteral("模型") &&
                    window.findChild<QDockWidget*>(QStringLiteral("neighborsDock")) == nullptr &&
                    window.findChild<QDockWidget*>(QStringLiteral("matingDock")) == nullptr,
                "model browsing must remain available without distance-analysis docks")) {
        return 1;
    }

    QString activatedPath;
    oap::app::DocumentUiController document(
        window, [&](const QString& path) { activatedPath = path; });
    document.fileList().fileActivated(QStringLiteral("part.step"));
    if (!expect(activatedPath == QStringLiteral("part.step"),
                "document slice must route file activation")) {
        return 1;
    }

    int openRequests = 0;
    int profileRequests = 0;
    int holeRequests = 0;
    int pocketRequests = 0;
    int finishingRequests = 0;
    int projectSimulationRequests = 0;
    int engravingSimulationRequests = 0;
    int operationPlaybackRequests = 0;
    int operationSoloRequests = 0;
    int previewRequests = 0;
    QString previewFormalOperation;
    QString regeneratedFinishing;
    QString editedFinishing;
    int holeParameterRequests = 0;
    QString selectedFormalOperation;
    oap::app::CamUiCallbacks camCallbacks;
    camCallbacks.previewOperation = [&](const QString& id) {
        ++previewRequests;
        previewFormalOperation = id;
    };
    camCallbacks.openPart = [&] { ++openRequests; };
    camCallbacks.generateProfile = [&] { ++profileRequests; };
    camCallbacks.generateHole = [&] { ++holeRequests; };
    camCallbacks.generatePocket = [&] { ++pocketRequests; };
    camCallbacks.generateFinishing = [&] { ++finishingRequests; };
    camCallbacks.regeneratePocket = [&] { ++pocketRequests; };
    camCallbacks.regenerateProfile = [&] { ++profileRequests; };
    camCallbacks.regenerateFinishing = [&](const QString& id) { ++finishingRequests; regeneratedFinishing = id; };
    camCallbacks.operationActivated = [&](int) { ++operationPlaybackRequests; };
    camCallbacks.operationSoloRequested = [&](const QString&) { ++operationSoloRequests; };
    camCallbacks.simulateProject = [&] { ++projectSimulationRequests; };
    camCallbacks.simulateEngraving = [&] { ++engravingSimulationRequests; };
    camCallbacks.editFinishingOperation = [&](const QString& id) { editedFinishing = id; };
    camCallbacks.editHoleParameters = [&] { ++holeParameterRequests; };
    camCallbacks.formalOperationSelected =
        [&](const QString& id) { selectedFormalOperation = id; };
    oap::app::CamUiController cam(window, document.dock(), std::move(camCallbacks));
    cam.panel().openPartRequested();
    cam.panel().generateProfileRequested();
    cam.panel().generateFinishingRequested();
    cam.panel().simulateProjectRequested();
    cam.operationTree().editFinishingOperationRequested(QStringLiteral("finish-1"));
    cam.operationTree().editHoleParametersRequested();
    cam.operationTree().formalOperationSelected(QStringLiteral("profile-formal-1"));
    if (!expect(openRequests == 1 && profileRequests == 1 && finishingRequests == 1 &&
                    projectSimulationRequests == 1 && editedFinishing == QStringLiteral("finish-1") && holeParameterRequests == 1 &&
                    selectedFormalOperation == QStringLiteral("profile-formal-1"),
                "CAM slice must preserve command arguments") ||
        !expect(cam.panel().findChild<QWidget*>(
                    QStringLiteral("generateEngravingAction")),
                "integrated engraving generation action must be present") ||
        !expect(cam.dock().objectName() == QStringLiteral("camWorkflowDock"),
                "CAM slice must own its stable dock identity")) {
        return 1;
    }

    if (!expect(!cam.operationTree().findChild<QWidget*>(QStringLiteral("generateLocalHoleToolpathButton")),
                "manual local planning action must be absent"))
        return 1;
    if (!expect(!cam.operationTree().findChild<QWidget*>("generateHoleToolpathButton") &&
                cam.panel().findChildren<QPushButton*>("generateHoleToolpathAction").size() == 1,
                "CAM must expose one global hole generation button without the old tree duplicate")) return 1;
    oap::app::CamWorkflowSummary singleToolSummary;
    singleToolSummary.hasModel = true;
    singleToolSummary.recognitionReady = true;
    singleToolSummary.directToolReady = true;
    singleToolSummary.holeGenerationReady = true;
    singleToolSummary.pocketGenerationReady = true;
    singleToolSummary.profileGenerationReady = true;
    cam.panel().setSummary(singleToolSummary);
    auto* holeAction = cam.panel().findChild<QPushButton*>("generateHoleToolpathAction");
    auto* pocketAction = cam.panel().findChild<QPushButton*>("generatePocketToolpathAction");
    auto* profileAction = cam.panel().findChild<QPushButton*>("generateProfileToolpathAction");
    if (!expect(holeAction && pocketAction && profileAction && holeAction->isEnabled() &&
                pocketAction->isEnabled() && profileAction->isEnabled(),
                "direct tools enable each recognized family without any global assigned tool")) return 1;
    holeAction->click(); pocketAction->click(); profileAction->click();
    if (!expect(holeRequests == 1 && pocketRequests == 1 && profileRequests == 2,
                "direct tool generation must use the existing three family commands")) return 1;
    singleToolSummary.pocketGenerationReady = false;
    cam.panel().setSummary(singleToolSummary);
    if (!expect(!pocketAction->isEnabled() && holeAction->isEnabled() && profileAction->isEnabled(),
                "tool readiness must remain independent across operation families")) return 1;
    singleToolSummary.busy = true;
    singleToolSummary.pocketGenerationReady = true;
    cam.panel().setSummary(singleToolSummary);
    if (!expect(!holeAction->isEnabled() && !pocketAction->isEnabled() && !profileAction->isEnabled(),
                "busy workspace must disable direct-tool generation")) return 1;
    oap::app::HoleToolBatchView treeBatch;
    oap::app::CamWorkflowSummary finishingSummary;
    finishingSummary.hasModel = true;
    finishingSummary.hasProjectContent = true;
    finishingSummary.activeProgramAvailable = true;
    finishingSummary.activeProgramName = QStringLiteral("雕刻加工");
    finishingSummary.setupFramesValid = true;
    finishingSummary.finishingGenerationReady = true;
    finishingSummary.finishingOperationCount = 1;
    cam.panel().setSummary(finishingSummary);
    auto* finishingAction = cam.panel().findChild<QPushButton*>(QStringLiteral("generateEngravingAction"));
    auto* projectIpwAction = cam.panel().findChild<QWidget*>(QStringLiteral("simulateProjectIpwAction"));
    auto* engravingIpwAction = cam.panel().findChild<QPushButton*>(QStringLiteral("simulateEngravingIpwAction"));
    if (!expect(engravingIpwAction && engravingIpwAction->isEnabled(), "engraving has its own enabled IPW action")) return 1;
    engravingIpwAction->click();
    if (!expect(engravingSimulationRequests == 1 && projectSimulationRequests == 1,
                "engraving IPW must dispatch its own command, not global project simulation")) return 1;
    if (!expect(finishingAction && finishingAction->isEnabled() && projectIpwAction && projectIpwAction->isEnabled(),
                "engraving without feature recognition or a global library enables generation and project IPW"))
        return 1;
    finishingAction->click();
    if (!expect(finishingRequests == 2 && finishingAction->text().contains(QStringLiteral("雕刻")),
                "engraving uses the normal generation command directly"))
        return 1;
    finishingSummary.busy = true;
    cam.panel().setSummary(finishingSummary);
    if (!expect(!finishingAction->isEnabled() && !projectIpwAction->isEnabled() && !engravingIpwAction->isEnabled(),
                "busy workspace disables finishing and project IPW actions"))
        return 1;

    auto* saveProject = cam.panel().findChild<QPushButton*>("saveMachiningProjectAction");
    auto* exportProgram = cam.panel().findChild<QPushButton*>("exportCurrentProgramAction");
    auto* outputStatus = cam.panel().findChild<QLabel*>("camWorkflowOutputStatus");
    if (!expect(saveProject && exportProgram && outputStatus, "project/output actions must expose stable identities")) return 1;
    oap::app::CamWorkflowSummary outputSummary;
    outputSummary.hasModel = true;
    cam.panel().setSummary(outputSummary);
    if (!expect(!engravingIpwAction->isEnabled(), "no engraving program must disable engraving IPW")) return 1;
    if (!expect(!saveProject->isEnabled() && !exportProgram->isEnabled() && !projectIpwAction->isEnabled(),
                "a loaded model alone must not imply a savable or exportable program")) return 1;
    outputSummary.hasProjectContent = true;
    cam.panel().setSummary(outputSummary);
    if (!expect(saveProject->isEnabled() && !exportProgram->isEnabled() && !projectIpwAction->isEnabled(),
                "disabled formal definitions may be saved without advertising runtime playback or IPW")) return 1;
    outputSummary.pendingRegeneration = true;
    outputSummary.profileOperationCount = 1;
    outputSummary.activeProgramName = QStringLiteral("外轮廓");
    cam.panel().setSummary(outputSummary);
    if (!expect(!saveProject->isEnabled() && !exportProgram->isEnabled() && !projectIpwAction->isEnabled() &&
                outputStatus->text().contains(QStringLiteral("外轮廓")) && outputStatus->text().contains(QStringLiteral("重新生成")),
                "pending project regeneration must block save/IPW and explain the unavailable active preview")) return 1;
    outputSummary.activeProgramAvailable = true;
    outputSummary.activeProgramName = QStringLiteral("雕刻加工");
    outputSummary.finishingOperationCount = 1;
    cam.panel().setSummary(outputSummary);
    if (!expect(!saveProject->isEnabled() && exportProgram->isEnabled() && !projectIpwAction->isEnabled() &&
                outputStatus->text().contains(QStringLiteral("当前预览：雕刻加工")),
                "an unaffected current family may export while other pending families block full project save/IPW")) return 1;
    outputSummary.busy = true;
    cam.panel().setSummary(outputSummary);
    if (!expect(!exportProgram->isEnabled(), "busy state must override a valid active export program")) return 1;

    treeBatch.setupLabel = QStringLiteral("+Z 装夹");
    treeBatch.toolLabel = QStringLiteral("T7");
    treeBatch.family = oap::app::ToolpathBatchFamily::Profile;
    oap::app::ToolpathOperationView treeOperation;
    treeOperation.operationId = QStringLiteral("profile-formal-1-r0");
    treeOperation.formalOperationId = QStringLiteral("profile-formal-1");
    treeOperation.label = QStringLiteral("外轮廓");
    treeOperation.family = oap::app::ToolpathBatchFamily::Profile;
    treeBatch.operations.push_back(treeOperation);
    cam.operationTree().setProgram({treeBatch});
    cam.operationTree().selectFormalOperation(
        QStringLiteral("profile-formal-1"));
    treeBatch.operations.front().operationId =
        QStringLiteral("profile-formal-1-r4");
    cam.operationTree().setProgram({treeBatch});
    if (!expect(
            cam.operationTree().selectedFormalOperationId() ==
                QStringLiteral("profile-formal-1"),
            "CAM tree selection must survive regenerated runtime suffixes")) {
        return 1;
    }

    treeBatch.family = oap::app::ToolpathBatchFamily::SurfaceFinishing;
    treeBatch.operations.front().family = oap::app::ToolpathBatchFamily::SurfaceFinishing;
    treeBatch.operations.front().operationId = QStringLiteral("finish-1");
    treeBatch.operations.front().formalOperationId = QStringLiteral("finish-1");
    cam.operationTree().setProgram({treeBatch});
    cam.operationTree().selectFormalOperation(QStringLiteral("finish-1"));
    if (!expect(cam.operationTree().selectedFormalOperationId() == QStringLiteral("finish-1"),
                "engraving must be selectable as a formal tree operation"))
        return 1;

    auto* regenerate = cam.operationTree().findChild<QPushButton*>("regenerateToolpathOperationButton");
    auto* editOperation = cam.operationTree().findChild<QPushButton*>("editToolpathOperationButton");
    auto* previewOperation = cam.operationTree().findChild<QPushButton*>("previewToolpathFamilyButton");
    auto* operationTree = cam.operationTree().findChild<QTreeWidget*>("holeToolTree");
    if (!expect(regenerate && editOperation && previewOperation && operationTree, "selected-operation controls must remain available")) return 1;
    oap::app::CamWorkflowSummary available;
    available.hasModel = true;
    available.hasProjectContent = true;
    available.pendingRegeneration = true;
    available.holeGenerationReady = true;
    available.pocketGenerationReady = true;
    available.profileGenerationReady = true;
    available.finishingGenerationReady = true;
    const auto selectFamily = [&](oap::app::ToolpathBatchFamily family) {
        treeBatch.family = family;
        treeBatch.operations.front().family = family;
        treeBatch.operations.front().operationId = QStringLiteral("finish-1-runtime");
        treeBatch.operations.front().hasPlayback = false;
        treeBatch.operations.front().startMoveIndex = 0;
        treeBatch.operations.front().endMoveIndex = 10;
        cam.operationTree().setProgram({treeBatch});
        cam.operationTree().selectFormalOperation("finish-1");
    };
    cam.panel().setSummary(available);
    selectFamily(oap::app::ToolpathBatchFamily::Hole);
    if (!expect(regenerate->isEnabled() && regenerate->text().contains(QStringLiteral("全部孔")),
                "hole inspector must explicitly rebuild the full family")) return 1;
    regenerate->click();
    if (!expect(holeRequests == 2, "hole inspector must reuse the one global hole-generation callback")) return 1;
    available.holeGenerationReady = false;
    cam.panel().setSummary(available);
    if (!expect(!regenerate->isEnabled() && editOperation->isEnabled(),
                "selected hole readiness must refresh without reselection while parameter editing remains possible")) return 1;
    regenerate->click();
    if (!expect(holeRequests == 2, "unavailable selected hole must not generate")) return 1;
    for (const auto family : {oap::app::ToolpathBatchFamily::Pocket, oap::app::ToolpathBatchFamily::Profile,
                             oap::app::ToolpathBatchFamily::SurfaceFinishing}) {
        selectFamily(family);
        if (!expect(regenerate->isEnabled() && regenerate->text().contains(QStringLiteral("全部")),
                    "inspector rebuild scope must be the full selected operation family")) return 1;
        regenerate->click();
        available.pocketGenerationReady = family != oap::app::ToolpathBatchFamily::Pocket;
        available.profileGenerationReady = family != oap::app::ToolpathBatchFamily::Profile;
        available.finishingGenerationReady = family != oap::app::ToolpathBatchFamily::SurfaceFinishing;
        cam.panel().setSummary(available);
        if (!expect(!regenerate->isEnabled(), "each inspector must honor its own family readiness")) return 1;
        available.pocketGenerationReady = available.profileGenerationReady = available.finishingGenerationReady = true;
        cam.panel().setSummary(available);
    }
    if (!expect(pocketRequests == 2 && profileRequests == 3 && finishingRequests == 3 && regeneratedFinishing == "finish-1",
                "inspector rebuilds must route exactly once to the selected family")) return 1;
    auto* selectedItem = operationTree->currentItem();
    if (!expect(selectedItem && selectedItem->text(1) == QStringLiteral("未载入预览"),
                "formal definitions without runtime paths must not advertise playback")) return 1;
    operationTree->itemClicked(selectedItem, 1);
    operationTree->itemDoubleClicked(selectedItem, 0);
    if (!expect(operationPlaybackRequests == 0 && operationSoloRequests == 0,
                "nonplayable formal operations must not seek or hide all other paths via solo")) return 1;
    previewOperation->click();
    if (!expect(previewRequests == 1 && previewFormalOperation == "finish-1" &&
                    holeRequests == 2 && pocketRequests == 2 && profileRequests == 3 && finishingRequests == 3,
                "inactive family preview must route its formal ID without implicitly regenerating any family")) return 1;
    treeBatch.operations.front().hasPlayback = true;
    available.pendingRegeneration = false;
    available.activeProgramAvailable = true;
    available.activeProgramName = QStringLiteral("雕刻加工");
    available.finishingOperationCount = 1;
    cam.panel().setSummary(available);
    cam.operationTree().setProgram({treeBatch});
    cam.operationTree().selectFormalOperation("finish-1");
    selectedItem = operationTree->currentItem();
    operationTree->itemClicked(selectedItem, 1);
    operationTree->itemDoubleClicked(selectedItem, 0);
    if (!expect(operationPlaybackRequests == 1 && operationSoloRequests == 1,
                "playable selected operations must retain seek and solo behavior")) return 1;
    available.busy = true;
    cam.panel().setSummary(available);
    if (!expect(!regenerate->isEnabled() && !editOperation->isEnabled() && !previewOperation->isEnabled(),
                "busy workspace must disable selected-operation actions even when readiness remains true")) return 1;
    previewOperation->click();
    if (!expect(previewRequests == 1, "busy workspace must not switch preview family")) return 1;
    available.busy = false;
    cam.panel().setSummary(available);
    operationTree->setCurrentItem(operationTree->topLevelItem(0));
    if (!expect(!regenerate->isEnabled() && !editOperation->isEnabled() && !previewOperation->isEnabled(),
                "selecting a group must clear operation-specific actions")) return 1;

    oap::toolpath::HoleCoverageReport coverage;
    coverage.audited = true;
    oap::app::HoleCoverageOperationBinding binding;
    binding.formalOperationId = QStringLiteral("hole-formal-1");
    binding.sequence = 0;
    binding.enabled = false;
    binding.hasPlayback = false;
    binding.sourceHoleIndices = {0};
    cam.coverageAudit().setReport(coverage, {binding});
    cam.coverageAudit().selectFormalOperation(
        QStringLiteral("hole-formal-1"));
    if (!expect(
            cam.coverageAudit().selectedFormalOperationId() ==
                QStringLiteral("hole-formal-1"),
            "coverage audit must expose selectable formal operations without playback")) {
        return 1;
    }

    auto sink = std::make_shared<NullSink>();
    oap::app::presentation::OverlayController overlays(sink);
    oap::app::StockSimulationWorkspaceCoordinator workspace(
        overlays, Handle(AIS_InteractiveContext){}, Handle(V3d_View){});
    oap::app::SimulationUiController simulation(
        cam.featureAllowance(), workspace, [] { return gp_Trsf{}; }, &window);
    cam.featureAllowance().allowanceSelected(-1);

    // Exercise widget teardown with a selected formal operation. QWidget
    // destroys its children after the derived panel's QString/QHash members;
    // the child tree must not call the inspector during that interval.
    for (int iteration = 0; iteration < 16; ++iteration) {
        auto lifetimePanel = std::make_unique<HoleToolTreePanel>();
        auto lifetimeBatch = treeBatch;
        const QString formalId = QStringLiteral("lifetime-formal-operation-%1-with-owned-string-storage").arg(iteration);
        lifetimeBatch.operations.front().formalOperationId = formalId;
        lifetimeBatch.operations.front().operationId = formalId + QStringLiteral("-runtime");
        lifetimePanel->setProgram({lifetimeBatch});
        lifetimePanel->selectFormalOperation(formalId);
        lifetimePanel->setSoloOperation(lifetimeBatch.operations.front().operationId);
        if (!expect(lifetimePanel->selectedFormalOperationId() == formalId,
                    "teardown regression must leave a selected, heap-backed formal operation")) return 1;
        lifetimePanel.reset();
    }

    std::cout << "PASS: modular UI slice controller checks\n";
    return 0;
}

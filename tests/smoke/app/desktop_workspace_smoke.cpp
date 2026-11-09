#include "app/orchestration/DesktopWorkspaceCoordinator.h"

#include "app/document/ModelDocumentWorkspaceCoordinator.h"
#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/presentation/OverlayController.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"

#include <AIS_InteractiveObject.hxx>
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

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    const auto sink = std::make_shared<RecordingSink>();
    oap::app::presentation::OverlayController overlays(sink);
    oap::app::RecognitionWorkspaceCoordinator recognition(overlays);
    oap::app::PlaybackWorkspaceCoordinator playback(
        overlays, Handle(AIS_InteractiveContext){}, Handle(V3d_View){});
    oap::app::StockSimulationSession initialStockSession;
    initialStockSession.hasHoleSimulation() = true;
    initialStockSession.hasPocketSimulation() = true;
    oap::app::StockSimulationWorkspaceCoordinator stock(
        overlays, Handle(AIS_InteractiveContext){}, Handle(V3d_View){},
        std::move(initialStockSession));
    oap::app::ModelDocumentWorkspaceCoordinator document(".");
    oap::app::MachiningWorkspaceCoordinator machining;
    oap::app::DesktopWorkspaceCoordinator desktop(
        machining, stock, playback, recognition, document);

    oap::app::MachiningProjectLoadResult loaded;
    const oap::app::MachiningProjectInstallResult installed =
        desktop.installProject(std::move(loaded));
    if (!expect(
            !stock.hasHoleSimulation() && !stock.hasPocketSimulation() &&
                installed.holeOperationCount == 0 &&
                machining.activePreview() ==
                    oap::app::ActiveToolpathPreview::None,
            "empty project install must clear stale simulations and leave no active preview")) {
        return 1;
    }

    oap::toolpath::PocketToolpathOperation pocketOperation;
    pocketOperation.operationId = "POCKET_1";
    oap::toolpath::PocketToolpathBatch pocketBatch;
    pocketBatch.operations.push_back(pocketOperation);
    oap::manufacturing::MachiningJob pocketJob;
    pocketJob.setPocketResult(std::move(pocketBatch), "pocket-runtime");
    pocketJob.definition().pocketOperations.push_back({});
    oap::app::PocketPreviewEntry pocketEntry;
    pocketEntry.key = "pocket-1";
    pocketEntry.operation = pocketOperation;
    oap::toolpath::ToolpathParams pocketParams;
    desktop.publishPocketProgram(
        std::move(pocketJob), {pocketEntry}, pocketParams, pocketParams);
    if (!expect(
            machining.status().hasPocketProgram &&
                machining.hasPocketPreview() &&
                machining.activePreview() ==
                    oap::app::ActiveToolpathPreview::Pocket,
            "pocket publication must update the machining workspace atomically")) {
        return 1;
    }

    oap::toolpath::ProfileToolpathOperation profileOperation;
    profileOperation.operationId = "PROFILE_1";
    oap::toolpath::ProfileToolpathBatch profileBatch;
    profileBatch.operations.push_back(profileOperation);
    oap::manufacturing::MachiningJob profileJob;
    profileJob.setProfileResult(std::move(profileBatch), "profile-runtime");
    profileJob.definition().profileOperations.push_back({});
    oap::toolpath::ToolpathParams profileParams;
    desktop.publishProfileProgram(
        std::move(profileJob), profileParams, profileParams);
    if (!expect(
            machining.status().hasProfileProgram &&
                machining.hasProfilePreview() &&
                machining.activePreview() ==
                    oap::app::ActiveToolpathPreview::Profile,
            "profile publication must replace the active machining program")) {
        return 1;
    }

    desktop.resetForModel();
    if (!expect(
            machining.activePreview() ==
                    oap::app::ActiveToolpathPreview::None &&
                !machining.hasPocketPreview() &&
                !machining.hasProfilePreview() &&
                !stock.hasHoleSimulation() && !stock.hasPocketSimulation(),
            "model reset must clear all coordinated workspace state")) {
        return 1;
    }

    desktop.cancelAndWait();
    std::cout << "PASS: desktop workspace orchestration checks\n";
    return 0;
}

#include "app/simulation/StockSimulationWorkspaceCoordinator.h"

#include "app/presentation/OverlayController.h"

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

TopoDS_Shape boxShape(double x)
{
    return BRepPrimAPI_MakeBox(
        gp_Pnt(x, 0.0, 0.0), 1.0, 1.0, 1.0).Shape();
}

Handle(AIS_InteractiveObject) boxPresentation(double x)
{
    return new AIS_Shape(boxShape(x));
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    const auto sink = std::make_shared<RecordingSink>();
    oap::app::presentation::OverlayController overlays(sink);
    oap::app::StockSimulationSession initialSession;
    initialSession.hasHoleSimulation() = true;
    initialSession.holeSourceFingerprint() = QStringLiteral("hole-a");
    initialSession.holeSimulation().initialStock = boxShape(0.0);
    initialSession.holeSimulation().complete = true;
    const TopoDS_Shape firstSetupStock = boxShape(2.0);
    const TopoDS_Shape secondSetupStock = boxShape(3.0);
    const TopoDS_Shape finalHoleStock = boxShape(4.0);
    oap::toolpath::StockRemovalRecord step;
    step.stockAfter = firstSetupStock;
    initialSession.holeSimulation().steps.push_back(std::move(step));
    step = oap::toolpath::StockRemovalRecord{};
    step.stockAfter = secondSetupStock;
    initialSession.holeSimulation().steps.push_back(std::move(step));
    step = oap::toolpath::StockRemovalRecord{};
    step.stockAfter = finalHoleStock;
    initialSession.holeSimulation().steps.push_back(std::move(step));
    oap::app::StockSimulationWorkspaceCoordinator workspace(
        overlays, Handle(AIS_InteractiveContext){}, Handle(V3d_View){},
        std::move(initialSession));

    workspace.setDisplayQuality(42);
    workspace.setTriangulationEnabled(false);
    workspace.setDisplayMode(
        oap::app::IpwOcctDisplayMode::SurfaceWithEdges);
    if (!expect(
            workspace.displayQuality() == 10 &&
                !workspace.triangulationEnabled() &&
                !workspace.occtDisplayEnabled(),
            "workspace must own and validate IPW display configuration")) {
        return 1;
    }

    std::vector<oap::toolpath::HoleMachiningOperation> holeOperations(3u);
    holeOperations[0].setupIndex = 0;
    holeOperations[1].setupIndex = 1;
    holeOperations[2].setupIndex = 1;
    oap::app::HoleOpenGlSimulationRequest openGlRequest;
    QString openGlError;
    if (!expect(
            workspace.buildHoleOpenGlRequest(
                holeOperations, oap::toolpath::ToolpathParams{}, 1,
                boxShape(10.0), openGlRequest, openGlError) &&
                openGlRequest.operations.size() == 2u &&
                openGlRequest.initialStock.IsSame(firstSetupStock) &&
                openGlRequest.finalStock.IsSame(finalHoleStock) &&
                openGlRequest.displayQuality == 10,
            "hole OpenGL input must slice one contiguous setup between authoritative IPW snapshots")) {
        return 1;
    }

    overlays.replace(
        oap::app::presentation::OverlayGroup::PocketIpw,
        {boxPresentation(4.0)});

    workspace.displayHoleSnapshot(1, gp_Trsf{});
    if (!expect(
            overlays.size(
                oap::app::presentation::OverlayGroup::HoleStock) == 1 &&
                overlays.size(
                    oap::app::presentation::OverlayGroup::PocketIpw) == 0,
            "publishing a hole snapshot must atomically remove pocket IPW")) {
        return 1;
    }

    workspace.clearHole(false);
    if (!expect(
            workspace.hasHoleSimulation() &&
                workspace.holeSourceFingerprint() ==
                    QStringLiteral("hole-a") &&
                overlays.size(
                    oap::app::presentation::OverlayGroup::HoleStock) == 0,
            "display-only hole cleanup must preserve simulation results")) {
        return 1;
    }
    workspace.clearHole(true);
    if (!expect(
            !workspace.hasHoleSimulation() &&
                workspace.holeSourceFingerprint().isEmpty(),
            "full hole cleanup must remove result identity")) {
        return 1;
    }

    oap::app::StockSimulationSession pocketSession;
    pocketSession.hasPocketSimulation() = true;
    pocketSession.pocketUsesUnifiedIpw() = true;
    pocketSession.pocketSourceFingerprint() = QStringLiteral("pocket-a");
    oap::app::StockSimulationWorkspaceCoordinator pocketWorkspace(
        overlays, Handle(AIS_InteractiveContext){}, Handle(V3d_View){},
        std::move(pocketSession));
    oap::manufacturing::MachiningJob timelineJob;
    oap::manufacturing::ipw::IpwOperationDefinition timelineOperation;
    timelineOperation.id = "profile-formal-timeline";
    timelineOperation.setupId = "setup-z";
    timelineOperation.toolControllerId = "tool-ball";
    timelineOperation.inputFingerprint = "profile-input";
    timelineOperation.sequence = 4;
    timelineOperation.kind =
        oap::manufacturing::ipw::IpwOperationKind::Profile;
    timelineJob.ipwSession().configure(
        "stock-timeline", "engine-timeline", {timelineOperation});
    const auto unavailableTimeline =
        pocketWorkspace.timelineOperations(timelineJob);
    if (!expect(
            unavailableTimeline.size() == 1 &&
                unavailableTimeline.front().formalOperationId ==
                    QStringLiteral("profile-formal-timeline") &&
                unavailableTimeline.front().label.contains(
                    QStringLiteral("#5")) &&
                unavailableTimeline.front().status.contains(
                    QStringLiteral("待重建")) &&
                !unavailableTimeline.front().replayAvailable,
            "IPW timeline must retain stable IDs and explicit no-replay state for dirty nodes")) {
        return 1;
    }
    overlays.replace(
        oap::app::presentation::OverlayGroup::PocketIpw,
        {boxPresentation(6.0)});
    pocketWorkspace.clearPocket(false);
    if (!expect(
            pocketWorkspace.hasPocketSimulation() &&
                pocketWorkspace.pocketUsesUnifiedIpw() &&
                pocketWorkspace.pocketSourceFingerprint() ==
                    QStringLiteral("pocket-a") &&
                overlays.size(
                    oap::app::presentation::OverlayGroup::PocketIpw) == 0,
            "display-only pocket cleanup must preserve immutable checkpoints")) {
        return 1;
    }

    pocketWorkspace.clearPocket(true);
    if (!expect(
            !pocketWorkspace.hasPocketSimulation() &&
                !pocketWorkspace.pocketUsesUnifiedIpw() &&
                pocketWorkspace.pocketSourceFingerprint().isEmpty(),
            "full pocket cleanup must remove result identity")) {
        return 1;
    }

    oap::app::StockSimulationSession resetSession;
    resetSession.hasPocketSimulation() = true;
    resetSession.pocketSourceFingerprint() =
        QStringLiteral("pocket-config");
    resetSession.holeMarginX() = 3.0;
    oap::app::StockSimulationWorkspaceCoordinator resetWorkspace(
        overlays, Handle(AIS_InteractiveContext){}, Handle(V3d_View){},
        std::move(resetSession));
    resetWorkspace.resetForModel();
    if (!expect(
            !resetWorkspace.hasPocketSimulation() &&
                resetWorkspace.pocketSourceFingerprint() ==
                    QStringLiteral("pocket-config") &&
                resetWorkspace.holeSourceSettings().marginX == 3.0,
            "model reset must clear derived results but retain user stock settings")) {
        return 1;
    }

    std::cout << "PASS: stock simulation workspace lifecycle checks\n";
    return 0;
}

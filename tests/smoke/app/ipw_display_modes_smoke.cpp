// OCCT 主视口 Multi-Dexel 派生显示形式的后台产物回归。
#include "app/workflows/simulation/IpwDisplayMeshController.h"
#include "app/workflows/simulation/IpwDisplayMeshWorker.h"

#include <BRepPrimAPI_MakeBox.hxx>

#include <QCoreApplication>

#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

std::shared_ptr<const oap::toolpath::MultiDexelStock> makeStock()
{
    oap::toolpath::StockSimulationOptions options;
    options.zMapResolution = 1.0;
    options.zMapMaxCellCount = 10'000;
    options.zMapTileSize = 4;
    options.runParallel = false;
    std::string error;
    auto stock = oap::toolpath::MultiDexelStock::fromShape(
        BRepPrimAPI_MakeBox(4.0, 3.0, 2.0).Shape(),
        gp_Vec(0.0, 0.0, 1.0),
        2.0,
        options,
        nullptr,
        &error);
    if (!stock)
        std::cerr << error << '\n';
    return stock;
}

oap::app::IpwDisplayMeshResult build(
    oap::app::IpwDisplayMeshWorker& worker,
    const std::shared_ptr<const oap::toolpath::MultiDexelStock>& stock,
    oap::app::IpwOcctDisplayMode mode)
{
    std::optional<oap::app::IpwDisplayMeshResult> captured;
    const QMetaObject::Connection connection = QObject::connect(
        &worker,
        &oap::app::IpwDisplayMeshWorker::resultReady,
        &worker,
        [&captured](const oap::app::IpwDisplayMeshResult& result) {
            captured = result;
        },
        Qt::DirectConnection);
    oap::app::IpwDisplayMeshRequest request;
    request.stockSource = stock;
    request.displayMode = mode;
    worker.build(std::move(request));
    QObject::disconnect(connection);
    return captured.value_or(oap::app::IpwDisplayMeshResult{});
}

bool controllerPublishesOnlyNewest(
    const std::shared_ptr<const oap::toolpath::MultiDexelStock>& stock)
{
    oap::app::IpwDisplayMeshController controller;
    std::vector<oap::app::IpwDisplayMeshResult> results;
    QObject::connect(
        &controller,
        &oap::app::IpwDisplayMeshController::resultReady,
        [&results](const oap::app::IpwDisplayMeshResult& result) {
            results.push_back(result);
        });

    oap::app::IpwDisplayMeshRequest stale;
    stale.stockSource = stock;
    stale.displayMode =
        oap::app::IpwOcctDisplayMode::SurfaceWithEdges;
    oap::app::IpwDisplayMeshRequest current;
    current.stockSource = stock;
    current.displayMode = oap::app::IpwOcctDisplayMode::Off;
    if (!controller.submit(std::move(stale)) ||
        !controller.submit(std::move(current))) {
        return expect(false,
                      "the IPW controller should accept display requests");
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(3);
    while (results.empty() &&
           std::chrono::steady_clock::now() < deadline) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    QCoreApplication::processEvents();
    controller.cancelAndWait();
    controller.cancelAndWait();
    return expect(results.size() == 1 && results.front().ok() &&
                      results.front().displayMode ==
                          oap::app::IpwOcctDisplayMode::Off &&
                      !controller.running(),
                  "only the newest IPW request may cross the controller boundary");
}

bool partialAndIncrementalReplay()
{
    using namespace oap::app;
    using namespace oap::toolpath;
    StockSimulationOptions options;
    options.zMapResolution = 0.5;
    options.runParallel = false;
    const auto initial = MultiDexelStock::fromShape(
        BRepPrimAPI_MakeBox(24, 18, 8).Shape(), gp_Vec(0, 0, 1), 4, options);
    ToolpathParams params;
    params.tool.diameter = 4;
    PocketToolpathOperation operation;
    operation.operationId = "partial-replay";
    operation.outwardToolAxis = gp_Vec(0, 0, 1);
    ToolpathMove cut;
    cut.type = ToolpathMoveType::Cut;
    cut.from = {3, 5, 3}; cut.to = {12, 5, 3};
    operation.clearing.moves.push_back(cut);
    cut.from = cut.to; cut.to = {21, 5, 3};
    operation.clearing.moves.push_back(cut);
    auto source = std::make_shared<IpwTransientFrameSource>();
    source->parentStock = initial;
    source->operations = std::make_shared<const std::vector<PocketToolpathOperation>>(std::vector{operation});
    source->params = params; source->options = options;
    IpwDisplayMeshWorker worker;
    IpwDisplayMeshResult result;
    QObject::connect(&worker, &IpwDisplayMeshWorker::resultReady, &worker,
        [&](const IpwDisplayMeshResult& value) { result = value; }, Qt::DirectConnection);
    const auto replay = [&] {
        IpwDisplayMeshRequest request;
        request.transientSource = source;
        request.displayMode = IpwOcctDisplayMode::Off;
        worker.build(request);
        return result.ok() && result.materialStock && result.completedPrefixStock;
    };
    source->completedMoveCount = 1;
    if (!expect(replay(), "completed prefix must build")) return false;
    const auto prefix = result.completedPrefixStock;
    source->parentStock = prefix;
    source->firstMoveCount = 1;
    source->nextMoveFraction = 0.5;
    if (!expect(replay(), "legacy flat-tool sources must support partial incremental replay")) return false;
    auto referenceOperation = operation;
    referenceOperation.clearing.moves[1].to = {16.5, 5, 3};
    const auto reference = initial->applyPocketPrefix(referenceOperation, params, 0, 2, options).stock;
    if (!expect(reference && result.materialStock->volume() == reference->volume() &&
                result.completedPrefixStock->volume() == prefix->volume() && result.cuttingMoveCount == 1,
                "partial replay must retain the uncut prefix and process only the new move")) return false;
    source->nextMoveFraction = 0;
    source->completedMoveCount = 2;
    if (!expect(replay(), "incremental endpoint must build")) return false;
    const auto complete = initial->applyPocketPrefix(operation, params, 0, 2, options).stock;
    if (!expect(complete && result.materialStock->volume() == complete->volume() && result.cuttingMoveCount == 1,
                "incremental endpoint must equal checkpoint replay without recutting the prefix")) return false;
    source->parentStock = initial;
    source->firstMoveCount = source->completedMoveCount = 0;
    source->nextMoveFraction = 0.25;
    if (!expect(replay(), "partial rewind must build from the original checkpoint")) return false;
    referenceOperation.clearing.moves.resize(1);
    referenceOperation.clearing.moves[0].to = {5.25, 5, 3};
    const auto rewind = initial->applyPocketPrefix(referenceOperation, params, 0, 1, options).stock;
    if (!expect(rewind && result.materialStock->volume() == rewind->volume(), "rewind must not retain future cuts")) return false;
    // Unknown blade length depends on the full operation's axial span. A partial
    // horizontal cut must not lose the length inferred from the preceding plunge.
    ToolpathMove plunge;
    plunge.type = ToolpathMoveType::Plunge;
    plunge.from = {3, 5, 10}; plunge.to = {3, 5, 3};
    operation.clearing.moves.insert(operation.clearing.moves.begin(), plunge);
    source->operations = std::make_shared<const std::vector<PocketToolpathOperation>>(std::vector{operation});
    source->completedMoveCount = 1;
    source->nextMoveFraction = 0.5;
    if (!expect(replay(), "partial cut following a plunge must build")) return false;
    operation.clearing.moves[1].to = {7.5, 5, 3};
    const auto plungeReference = initial->applyPocketPrefix(operation, params, 0, 2, options).stock;
    return expect(plungeReference && result.materialStock->volume() == plungeReference->volume(),
        "slicing playback must preserve the cutting length inferred from the full operation");
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    const auto stock = makeStock();
    if (!expect(stock && stock->valid(),
                "the display fixture should rasterize")) {
        return 1;
    }

    oap::app::IpwDisplayMeshWorker worker;
    using Mode = oap::app::IpwOcctDisplayMode;
    for (const Mode mode : {
             Mode::SurfaceWithEdges,
             Mode::SurfaceShaded,
             Mode::SurfaceWireframe}) {
        const auto result = build(worker, stock, mode);
        if (!expect(result.ok() && !result.mesh.IsNull() &&
                        result.materialStock == stock,
                    "every surface style should preserve its material stock")) {
            return 1;
        }
    }

    const auto columns = build(worker, stock, Mode::DexelColumns);
    const auto endpoints = build(worker, stock, Mode::IntervalEndpoints);
    const auto prisms = build(worker, stock, Mode::IntervalPrisms);
    const auto section = build(worker, stock, Mode::MidSection);
    const auto brep = build(worker, stock, Mode::BRepSnapshot);
    const auto off = build(worker, stock, Mode::Off);

    return !(
        expect(columns.ok() && columns.primitiveGeometry &&
                   columns.primitiveGeometry->kind ==
                       oap::app::IpwDisplayPrimitiveKind::Segments &&
                   columns.primitiveGeometry->vertices.size() ==
                       static_cast<std::size_t>(stock->intervalCount()) * 2u,
               "Dexel columns should contain one segment per interval") &&
        expect(endpoints.ok() && endpoints.primitiveGeometry &&
                   endpoints.primitiveGeometry->kind ==
                       oap::app::IpwDisplayPrimitiveKind::Points &&
                   endpoints.primitiveGeometry->vertices.size() ==
                       static_cast<std::size_t>(stock->intervalCount()) * 2u,
               "endpoint mode should contain two points per interval") &&
        expect(prisms.ok() && prisms.primitiveGeometry &&
                   prisms.primitiveGeometry->kind ==
                       oap::app::IpwDisplayPrimitiveKind::Prisms &&
                   prisms.primitiveGeometry->prisms.size() ==
                       static_cast<std::size_t>(stock->intervalCount()),
               "voxel mode should contain one prism per interval") &&
        expect(section.ok() && section.primitiveGeometry &&
                   section.primitiveGeometry->kind ==
                       oap::app::IpwDisplayPrimitiveKind::Quads &&
                   !section.primitiveGeometry->quads.empty(),
               "mid-section mode should contain occupied quads") &&
        expect(brep.ok() && !brep.shape.IsNull() && brep.mesh.IsNull(),
               "BRep mode should publish topology without a mesh") &&
        expect(off.ok() && off.mesh.IsNull() && !off.primitiveGeometry &&
                   off.shape.IsNull() && off.materialStock == stock,
               "off mode should publish material without display geometry") &&
        controllerPublishesOnlyNewest(stock) && partialAndIncrementalReplay());
}

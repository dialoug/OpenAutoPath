#include "app/workflows/shared/UndoHistory.h"
#include "app/workflows/hole/HoleToolpathGenerationUseCase.h"
#include "app/workflows/hole/HoleToolpathCommandPolicy.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/workflows/hole/HoleToolpathWorkflowController.h"
#include "app/workflows/hole/HoleToolpathRunner.h"
#include "app/workflows/pocket/PocketToolpathGenerationUseCase.h"
#include "app/workflows/pocket/PocketToolpathWorkflowController.h"
#include "app/workflows/profile/ProfileToolpathGenerationUseCase.h"
#include "app/workflows/profile/ProfileSetupContextAssigner.h"
#include "app/workflows/profile/ProfileToolpathWorkflowController.h"
#include "app/workflows/simulation/StockSimulationCoordinator.h"
#include "app/workflows/simulation/StockSimulationUseCases.h"
#include "app/workflows/simulation/StockSimulationWorkflowController.h"
#include "app/workflows/shared/ToolpathExportController.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "app/workflows/project/ToolLibraryController.h"
#include "tooling/adapter/hole/HoleStageIdentity.h"
#include "tooling/library/ToolLibraryStorage.h"
#include "toolpath/algorithms/ToolpathAlgorithm.h"
#include "toolpath/core/HeavyJobScheduler.h"
#include "toolpath/pocket/simulation/PocketCamSimulationScene.h"
#include "toolpath/workflow/hole/HoleSimulationAdapter.h"
#include "analysis/ModelIndex.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/StepIndexer.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <cmath>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string_view>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QSettings>
#include <QStandardPaths>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

bool runPocketSimulationChecks();
bool runPocketPresentationChecks();

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

class ScopedHeavyJobBlocker {
public:
    ScopedHeavyJobBlocker()
    {
        const auto state = m_state;
        m_future = oap::toolpath::submitHeavyJob(
            std::make_shared<std::atomic_bool>(false),
            [state] {
                std::unique_lock lock(state->mutex);
                state->entered = true;
                state->changed.notify_all();
                state->changed.wait_for(lock, std::chrono::seconds(20),
                                        [&] { return state->released; });
                state->finished = true;
            }, [](oap::toolpath::HeavyJobRejection) {});
    }
    ~ScopedHeavyJobBlocker() { release(); }
    bool entered()
    {
        std::unique_lock lock(m_state->mutex);
        return m_state->changed.wait_for(lock, std::chrono::seconds(3),
                                        [&] { return m_state->entered; });
    }
    bool stillBlocked()
    {
        std::lock_guard lock(m_state->mutex);
        return m_state->entered && !m_state->finished;
    }
    void release()
    {
        {
            std::lock_guard lock(m_state->mutex);
            m_state->released = true;
        }
        m_state->changed.notify_all();
        if (m_future.valid()) m_future.wait();
    }
private:
    struct State {
        std::mutex mutex;
        std::condition_variable changed;
        bool entered = false, released = false, finished = false;
    };
    std::shared_ptr<State> m_state = std::make_shared<State>();
    std::future<void> m_future;
};

bool automaticHoleRunnerReportsQueueFull()
{
    ScopedHeavyJobBlocker blocker;
    if (!expect(blocker.entered(), "runner admission blocker must start"))
        return false;
    struct PendingJobs {
        std::vector<std::shared_ptr<std::atomic_bool>> cancellation;
        std::vector<std::future<int>> futures;
        ~PendingJobs()
        {
            for (const auto& token : cancellation)
                oap::toolpath::cancelHeavyJob(token);
            for (auto& future : futures)
                if (future.valid()) future.wait();
        }
    } pending;
    bool full = false;
    for (int index = 0; index < 128; ++index) {
        auto cancellation = std::make_shared<std::atomic_bool>(false);
        auto future = oap::toolpath::submitHeavyJob(
            cancellation, [] { return 1; },
            [](oap::toolpath::HeavyJobRejection reason) {
                return reason == oap::toolpath::HeavyJobRejection::QueueFull ? -1 : -2;
            });
        pending.cancellation.push_back(cancellation);
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            full = future.get() == -1;
            break;
        }
        pending.futures.push_back(std::move(future));
    }
    if (!expect(full && blocker.stillBlocked(), "fixture must fill the bounded queue"))
        return false;
    const auto result = oap::app::generateAutomaticHolesWithProgress(
        {}, {}, {}, {}, {});
    const std::string expected = oap::toolpath::heavyJobRejectionMessage(
        oap::toolpath::HeavyJobRejection::QueueFull);
    return expect(!result.batch.cancelled && result.batch.machining.operations.empty() &&
                      std::any_of(result.batch.diagnostics.begin(), result.batch.diagnostics.end(),
                          [&](const auto& diagnostic) {
                              return diagnostic.code == oap::toolpath::ToolpathDiagnosticCode::AlgorithmExecutionFailed &&
                                  diagnostic.severity == oap::toolpath::ToolpathDiagnosticSeverity::Error &&
                                  diagnostic.message == expected;
                          }) && blocker.stillBlocked(),
                  "automatic-hole Runner must return a structured queue-full failure without starting geometry");
}

bool holeOpenGlAdapterPreservesDrillGeometry()
{
    oap::toolpath::HoleMachiningOperation hole;
    hole.operationId = "HOLE-OPENGL-1";
    hole.algorithmId = "small-drill";
    hole.sourceHoleIndex = 7;
    hole.kind = oap::FeatureKind::BlindHole;
    hole.setupFace = oap::MoldSetupFace::PositiveZ;
    hole.setupIndex = 0;
    hole.entry = gp_Pnt(0.0, 0.0, 10.0);
    hole.bottom = gp_Pnt(0.0, 0.0, 0.0);
    hole.radius = 2.5;
    hole.tool.valid = true;
    hole.tool.toolNumber = 3;
    hole.tool.toolId = "DRILL-5";
    hole.tool.toolName = "5 mm drill";
    hole.tool.diameter = 5.0;
    hole.tool.cuttingEdgeLength = 20.0;
    hole.tool.overallLength = 30.0;
    hole.tool.tipAngle = 118.0;

    oap::toolpath::ToolpathMove plunge;
    plunge.type = oap::toolpath::ToolpathMoveType::Plunge;
    plunge.from = hole.entry;
    plunge.to = hole.bottom;
    hole.moves.push_back(plunge);

    const oap::toolpath::PocketToolpathOperation adapted =
        oap::toolpath::holeSimulationOperation(hole);
    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 5.0;
    params.tool.cuttingEdgeLength = 20.0;
    params.tool.overallLength = 30.0;
    const TopoDS_Shape stock = BRepPrimAPI_MakeBox(
        gp_Pnt(-10.0, -10.0, -5.0),
        gp_Pnt(10.0, 10.0, 15.0));
    const oap::toolpath::PocketCamSimulationSceneResult scene =
        oap::toolpath::PocketCamSimulationSceneBuilder{}.build(
            stock, {adapted}, params);

    const bool valid =
        adapted.operationId == hole.operationId &&
        adapted.sourceRegionIndex == hole.sourceHoleIndex &&
        adapted.clearing.moves.size() == 1u &&
        adapted.outwardToolAxis.Dot(gp_Vec(0.0, 0.0, 1.0)) >
            1.0 - 1.0e-9 &&
        scene.ok() && scene.scene.tools.size() == 1u &&
        !scene.scene.tools.front().flatEnd &&
        scene.scene.tools.front().profile.size() >= 2u &&
        scene.scene.tools.front().profile.back().first == 0.0 &&
        scene.scene.tools.front().profile.back().second == 0.0;
    return expect(
        valid,
        "hole OpenGL adaptation must retain the setup axis, moves, and conical drill tip");
}

bool profileRegionsFollowSixFaceSetupOrder()
{
    oap::MoldCoordinateFrame frame;
    frame.isValid = true;

    const auto region = [](const gp_Dir& openDir, const char* id) {
        oap::toolpath::ProfileRegion value;
        value.openDir = openDir;
        value.source.rootFaceId = id;
        return value;
    };
    const std::vector<oap::toolpath::ProfileRegion> input{
        region(gp_Dir(0.0, 0.0, -1.0), "-Z"),
        region(gp_Dir(0.0, 1.0, 0.0), "+Y-a"),
        region(gp_Dir(0.0, 0.0, 1.0), "+Z"),
        region(gp_Dir(-1.0, 0.0, 0.0), "-X"),
        region(gp_Dir(0.0, 1.0, 0.0), "+Y-b"),
        region(gp_Dir(1.0, 0.0, 0.0), "+X"),
        region(gp_Dir(0.0, -1.0, 0.0), "-Y")};

    oap::toolpath::ProfileToolpathBatch batch;
    batch.operations.resize(input.size());
    for (std::size_t index = 0; index < batch.operations.size(); ++index)
        batch.operations[index].sourceRegionIndex = static_cast<int>(index);

    const std::vector<oap::toolpath::ProfileRegion> ordered =
        oap::app::orderProfileRegionsBySetup(input, frame);
    oap::app::assignProfileSetupContexts(batch, input, frame);
    return expect(
        ordered.size() == 7 &&
            ordered[0].source.rootFaceId == "+Z" &&
            ordered[1].source.rootFaceId == "+X" &&
            ordered[2].source.rootFaceId == "+Y-a" &&
            ordered[3].source.rootFaceId == "+Y-b" &&
            ordered[4].source.rootFaceId == "-X" &&
            ordered[5].source.rootFaceId == "-Y" &&
            ordered[6].source.rootFaceId == "-Z" &&
            batch.operations[0].setupFace ==
                oap::MoldSetupFace::NegativeZ &&
            batch.operations[0].setupIndex ==
                static_cast<int>(oap::MoldSetupFace::NegativeZ),
        "profile regions must use the canonical stable six-face setup order");
}

const oap::tooling::ToolBit* toolByCurrentOrLegacyId(
    const oap::tooling::ToolLibrary& library,
    const QString& id)
{
    if (const oap::tooling::ToolBit* exact = library.toolById(id))
        return exact;
    for (const oap::tooling::ToolBit& tool : library.tools()) {
        if (tool.attributes.value(QStringLiteral("legacy_tool_id")) == id)
            return &tool;
    }
    return nullptr;
}

// Deliberately synthetic fixtures: the same tool appears in two user libraries.
QVector<oap::tooling::ToolLibrary> testToolLibraries()
{
    oap::tooling::ToolBit tool;
    tool.id = QStringLiteral("test-cutter");
    tool.name = QStringLiteral("synthetic cutter");
    tool.type = oap::tooling::ToolType::EndMill;
    tool.geometry.diameter = 4.0;
    tool.geometry.cuttingEdgeLength = 8.0;
    tool.geometry.overallLength = 0.0;
    QVector<oap::tooling::ToolLibrary> libraries;
    for (int index = 0; index < 2; ++index) {
        oap::tooling::ToolLibrary library(
            QStringLiteral("test-library-%1").arg(index),
            QStringLiteral("Synthetic library %1").arg(index));
        library.upsertTool(tool);
        libraries.push_back(std::move(library));
    }
    return libraries;
}

bool catalogMigrationPreservesAnExplicitZeroLength()
{
    QTemporaryDir temporaryDirectory;
    if (!expect(temporaryDirectory.isValid(), "catalog fixture needs a directory"))
        return false;
    const auto libraries = testToolLibraries();
    QJsonObject legacyLibrary = QJsonDocument::fromJson(
        oap::tooling::ToolLibraryStorage::serializeLibrary(libraries.front())).object();
    legacyLibrary.insert(QStringLiteral("schema"), QStringLiteral("faceface-tool-library"));
    oap::tooling::ToolLibrary imported;
    QString importError;
    if (!expect(oap::tooling::ToolLibraryStorage::deserializeLibrary(
                    QJsonDocument(legacyLibrary).toJson(), imported, &importError) &&
                    QJsonDocument::fromJson(oap::tooling::ToolLibraryStorage::serializeLibrary(imported))
                        .object().value(QStringLiteral("schema")).toString() ==
                        QStringLiteral("oap-tool-library"),
                "legacy library import must write the new OAP schema"))
        return false;
    const QString catalogPath = temporaryDirectory.filePath(QStringLiteral("catalog.json"));
    QString error;
    if (!expect(oap::tooling::ToolLibraryStorage::saveCatalog(catalogPath, libraries, &error),
                "synthetic catalog must save"))
        return false;

    oap::tooling::ToolAssetCatalog loaded;
    bool requiresRewrite = true;
    if (!expect(oap::tooling::ToolLibraryStorage::loadCatalog(
                    catalogPath, loaded, &error, &requiresRewrite) && !requiresRewrite,
                "current catalog must reload without migration"))
        return false;
    const auto* current = toolByCurrentOrLegacyId(loaded.libraries().front(),
                                                 QStringLiteral("test-cutter"));
    if (!expect(current && current->geometry.overallLength == 0.0,
                "explicit zero length must survive reload"))
        return false;

    // Real v1 layout embeds ToolBits in each library, without global assets.
    QJsonArray legacyLibraries;
    for (const auto& library : libraries) {
        legacyLibraries.append(QJsonDocument::fromJson(
            oap::tooling::ToolLibraryStorage::serializeLibrary(library))
                .object().value(QStringLiteral("library")));
    }
    const QJsonObject legacyRoot{
        {QStringLiteral("schema"), QStringLiteral("faceface-tool-library-catalog")},
        {QStringLiteral("version"), 1},
        {QStringLiteral("libraries"), legacyLibraries}};
    QFile legacyCatalog(catalogPath);
    if (!expect(legacyCatalog.open(QIODevice::WriteOnly | QIODevice::Truncate),
                "legacy fixture must open"))
        return false;
    legacyCatalog.write(QJsonDocument(legacyRoot).toJson());
    legacyCatalog.close();
    if (!expect(oap::tooling::ToolLibraryStorage::loadCatalog(
                    catalogPath, loaded, &error, &requiresRewrite) && requiresRewrite,
                "v1 catalog must migrate its storage structure"))
        return false;
    const auto* migrated = toolByCurrentOrLegacyId(loaded.libraries().front(),
                                                  QStringLiteral("test-cutter"));
    return expect(migrated && migrated->geometry.overallLength == 0.0 &&
                      migrated->geometry.cuttingEdgeLength == 8.0 &&
                      loaded.libraries().size() == libraries.size(),
                  "legacy migration must preserve user geometry without adding defaults");
}

bool toolLibraryStartupImportsSimulationLibrary()
{
    QTemporaryDir temporaryDirectory;
    if (!expect(temporaryDirectory.isValid(), "startup fixture needs a directory"))
        return false;
    // Isolate both Qt settings and the standard catalog path from real user data.
    QStandardPaths::setTestModeEnabled(true);
    QCoreApplication::setApplicationName(QFileInfo(temporaryDirectory.path()).fileName());
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, temporaryDirectory.path());
    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, temporaryDirectory.path());
    const QString path = oap::tooling::ToolLibraryStorage::defaultCatalogPath();
    if (!expect(!QFileInfo::exists(path), "isolated catalog must not exist"))
        return false;
    struct Cleanup {
        QString path;
        ~Cleanup() {
            QFile::remove(path);
            const QDir directory = QFileInfo(path).dir();
            QDir().rmdir(directory.absolutePath());
            QDir().rmdir(QFileInfo(directory.absolutePath()).dir().absolutePath());
        }
    } cleanup{path};
    oap::app::ToolLibraryController controller;
    const QString simulationId = QStringLiteral("simulation-general-mm-v1");
    if (!expect(controller.load().isEmpty() && controller.libraries().size() == 1 &&
                    controller.catalog().assets().size() == 281 &&
                    controller.activeLibraryId() == simulationId && QFileInfo::exists(path),
                "first startup must import and select the embedded simulation library"))
        return false;
    QString error;
    auto editedCatalog = controller.catalog();
    const QString editedName = QStringLiteral("User edited simulation library");
    editedCatalog.libraryById(simulationId)->setName(editedName);
    auto editedTool = editedCatalog.assets().first();
    editedTool.geometry.overallLength += 7.0;
    if (!expect(editedCatalog.upsertAsset(editedTool, &error) &&
                    oap::tooling::ToolLibraryStorage::saveCatalog(path, editedCatalog, &error) &&
                    controller.load().isEmpty() && controller.libraries().size() == 1 &&
                    controller.activeLibrary() &&
                    controller.activeLibrary()->name() == editedName &&
                    controller.catalog().assetById(editedTool.id) &&
                    controller.catalog().assetById(editedTool.id)->geometry.overallLength ==
                        editedTool.geometry.overallLength,
                "repeated startup must preserve user edits without duplicating the simulation library"))
        return false;
    QSettings settings(QStringLiteral("OAP"), QStringLiteral("OpenAutoPath"));
    settings.setValue(QStringLiteral("tooling/active_library_id"), QStringLiteral("test-library-1"));
    settings.sync();
    if (!expect(oap::tooling::ToolLibraryStorage::saveCatalog(path, testToolLibraries(), &error) &&
                    controller.load().isEmpty() && controller.libraries().size() == 3 &&
                    controller.activeLibraryId() == QStringLiteral("test-library-1"),
                "startup must add simulation while preserving user libraries and active selection"))
        return false;
    if (!expect(oap::tooling::ToolLibraryStorage::saveCatalog(
                    path, oap::tooling::ToolAssetCatalog{}, &error) &&
                    controller.load().isEmpty() && controller.libraries().size() == 1 &&
                    controller.activeLibraryId() == simulationId,
                "an empty catalog must receive the simulation library on startup"))
        return false;
    QFile damaged(path);
    if (!damaged.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return false;
    damaged.write("invalid catalog");
    damaged.close();
    const auto warnings = controller.load();
    if (!damaged.open(QIODevice::ReadOnly))
        return false;
    return expect(!warnings.isEmpty() && controller.libraries().isEmpty() &&
                      damaged.readAll() == QByteArray("invalid catalog"),
                  "damaged user catalog must be reported and preserved without fallback data");
}

bool globalToolAssetsPersistIndependently()
{
    oap::tooling::ToolAssetCatalog catalog;
    QString error;
    if (!expect(oap::tooling::ToolAssetCatalog::fromLibraries(
                    testToolLibraries(), catalog, &error),
                "synthetic libraries must migrate into a global asset catalog")) {
        std::cerr << error.toStdString() << '\n';
        return false;
    }

    qsizetype materializedToolCount = 0;
    for (const oap::tooling::ToolLibrary& library : catalog.libraries())
        materializedToolCount += library.tools().size();
    if (!expect(catalog.assets().size() < materializedToolCount,
                "identical legacy ToolBits must share one global asset")) {
        return false;
    }

    oap::tooling::ToolBit standalone;
    standalone.id = QStringLiteral("standalone-regression-tool");
    standalone.name = QStringLiteral("standalone regression drill");
    standalone.type = oap::tooling::ToolType::Drill;
    standalone.geometry.diameter = 5.0;
    if (!expect(catalog.upsertAsset(standalone, &error),
                "a standalone global ToolBit must be accepted") ||
        !expect(catalog.referencingLibraryIds(standalone.id).isEmpty(),
                "a standalone ToolBit must not require a library reference")) {
        return false;
    }

    QTemporaryDir temporaryDirectory;
    const QString catalogPath =
        temporaryDirectory.filePath(QStringLiteral("catalog.json"));
    if (!expect(temporaryDirectory.isValid() &&
                    oap::tooling::ToolLibraryStorage::saveCatalog(
                        catalogPath, catalog, &error),
                "the global asset catalog must save")) {
        std::cerr << error.toStdString() << '\n';
        return false;
    }

    oap::tooling::ToolAssetCatalog loaded;
    bool requiresCatalogRewrite = true;
    if (!expect(oap::tooling::ToolLibraryStorage::loadCatalog(
                    catalogPath, loaded, &error,
                    &requiresCatalogRewrite),
                "the global asset catalog must reload") ||
        !expect(!requiresCatalogRewrite,
                "a v3 global asset catalog must not request migration") ||
        !expect(loaded.assetById(standalone.id) &&
                    loaded.referencingLibraryIds(standalone.id).isEmpty(),
                "a standalone ToolBit must survive a catalog round trip")) {
        std::cerr << error.toStdString() << '\n';
        return false;
    }

    const QString libraryId = loaded.libraries().front().id();
    if (!expect(loaded.attachAsset(libraryId, standalone.id, &error),
                "a global ToolBit must attach to a library by reference")) {
        return false;
    }
    standalone.name = QStringLiteral("updated global regression drill");
    if (!expect(loaded.upsertAsset(standalone, &error),
                "editing a global ToolBit must succeed")) {
        return false;
    }
    const oap::tooling::ToolLibrary* referenced = loaded.libraryById(libraryId);
    return expect(referenced && referenced->toolById(standalone.id) &&
                      referenced->toolById(standalone.id)->name == standalone.name,
                  "global ToolBit edits must propagate to library snapshots");
}

bool profileIpwBuildsPocketEquivalentPlaybackInput()
{
    namespace manufacturing = oap::manufacturing;
    manufacturing::MachiningJob job;
    job.definition().part.sourceFingerprint = "profile-ipw-part";
    manufacturing::SetupDefinition setup;
    setup.id = "setup-z";
    setup.setupIndex = 0;
    job.definition().setups.push_back(setup);
    manufacturing::ProfileOperationDefinition definition;
    definition.id = "profile-ipw";
    definition.setupId = setup.id;
    definition.toolControllerId = "mill-2";
    definition.sequence = 0;
    definition.inputFingerprint = "profile-ipw-v1";
    definition.state = manufacturing::JobOperationState::Generated;
    job.definition().profileOperations.push_back(definition);

    oap::toolpath::ToolpathParams params;
    params.tool.diameter = 2.0;
    params.tool.cuttingEdgeLength = 8.0;
    oap::toolpath::ProfileToolpathOperation operation;
    operation.operationId = definition.id + "-001";
    operation.setupIndex = 0;
    operation.outwardToolAxis = gp_Vec(0.0, 0.0, 1.0);
    operation.contour.ok = true;
    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(2.0, -1.6, 4.0);
    cut.to = gp_Pnt(18.0, -1.6, 4.0);
    operation.contour.moves.push_back(cut);
    operation.contour.primitives.push_back(
        oap::toolpath::makeLinearPrimitive(cut));
    operation.contour.toolRadius = 1.0;
    operation.contour.finalOffset = 1.6;
    oap::toolpath::ProfileToolpathBatch batch;
    batch.effectiveParams = params;
    batch.operations.push_back(operation);
    job.setProfileResult(std::move(batch), "profile-ipw-runtime");

    oap::app::StockSimulationSession session;
    oap::app::StockSimulationCoordinator coordinator;
    const TopoDS_Shape stock = BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, -0.5, 0.0), 20.0, 20.5, 10.0).Shape();
    const std::vector<oap::toolpath::ProfileToolpathOperation> previews{
        operation};
    oap::app::ProfileStockSimulationUseCaseRequest request;
    request.settings.source.sourceMode = 1;
    request.settings.selectedSetup = 0;
    request.settings.displayQuality = 1;
    request.settings.triangulationEnabled = false;
    request.session = &session;
    request.coordinator = &coordinator;
    request.job = &job;
    request.operations = &previews;
    request.effectiveParams = &params;
    request.generatedInitialStock = stock;
    request.targetShape =
        BRepPrimAPI_MakeBox(20.0, 20.0, 10.0).Shape();
    request.persistSettings = false;

    // Exercise the real private IPW Runner through its public use case. Its
    // work borrows job, so an exceptional presenter must remove pending work
    // before unwinding, not leave a promise future with dangling captures.
    {
        ScopedHeavyJobBlocker blocker;
        if (!expect(blocker.entered(), "IPW admission blocker must start"))
            return false;
        bool presented = false, queued = false, threw = false;
        std::size_t submittedRevision = 0;
        std::string submittedStateKey;
        std::function<void()> cancelPending;
        request.interaction.showProgress = [&](const oap::app::WorkflowProgressRequest& progress) {
            presented = true;
            queued = oap::toolpath::heavyJobScheduler().pendingCount() == 1u;
            cancelPending = progress.cancel;
            submittedRevision = job.ipwSession().revision();
            submittedStateKey = job.ipwSession().initialStateKey();
            throw std::runtime_error("injected IPW presenter failure");
        };
        try {
            oap::app::ProfileStockSimulationUseCase::execute(request);
        } catch (const std::runtime_error& error) {
            threw = std::string_view(error.what()) == "injected IPW presenter failure";
        }
        const bool removedBeforeReturn =
            oap::toolpath::heavyJobScheduler().pendingCount() == 0u;
        const bool stillBlocked = blocker.stillBlocked();
        // Keep the fixture safe even if the production guard regresses.
        if (cancelPending) cancelPending();
        blocker.release();
        auto fence = oap::toolpath::submitHeavyJob(
            std::make_shared<std::atomic_bool>(false), [] {},
            [](oap::toolpath::HeavyJobRejection) {});
        fence.get();
        request.interaction.showProgress = {};
        if (!expect(presented && queued && threw && removedBeforeReturn && stillBlocked &&
                        job.ipwSession().revision() == submittedRevision &&
                        job.ipwSession().initialStateKey() == submittedStateKey &&
                        !session.hasPocketSimulation(),
                    "exceptional IPW presenter must cancel queued borrowed-job work before return and leave no later checkpoint writes"))
            return false;
    }
    const auto result =
        oap::app::ProfileStockSimulationUseCase::execute(request);
    Handle(Poly_Triangulation) allowanceMesh;
    std::string allowanceMeshError;
    const bool allowanceMeshOk =
        session.pocketComparison().heightFieldDisplaySource &&
        session.pocketComparison().heightFieldDisplaySource
            ->buildDisplayMesh(
                oap::toolpath::PocketZMapDifferenceView::FeatureAllowance,
                allowanceMesh, nullptr, definition.id,
                &allowanceMeshError);
    const bool valid =
        result.ok() && session.hasPocketSimulation() &&
        session.pocketUsesUnifiedIpw() &&
        session.pocketComparison().complete &&
        session.pocketComparison().featureAllowances.size() == 1 &&
        session.pocketComparison().featureAllowances.front().featureKind ==
            "profile" &&
        session.pocketComparison().featureAllowances.front()
                .remainingAllowanceVolume > 0.0 &&
        allowanceMeshOk && !allowanceMesh.IsNull() &&
        result.camInput.operations.size() == 1 &&
        result.camInput.operations.front().clearing.moves.size() == 1 &&
        result.camInput.initialIpwStock &&
        result.camInput.finalIpwStock &&
        result.camInput.operationInitialIpwSources.size() == 1 &&
        result.camInput.operationInitialIpwSources.front() &&
        result.camInput.operationInitialIpwSources.front()->operations &&
        result.camInput.operationInitialIpwSources.front()
                ->operations->front().operationId == operation.operationId &&
        session.pocketTransientOperations().size() == 1 &&
        session.pocketTransientOperations().front();
    if (!valid) {
        std::cerr << "profile IPW diagnostics: ok=" << result.ok()
                  << ", error=" << result.error.toStdString()
                  << ", warning=" << result.warning.toStdString()
                  << ", comparison="
                  << session.pocketComparison().complete
                  << ", allowances="
                  << session.pocketComparison().featureAllowances.size()
                  << ", globalRemaining="
                  << session.pocketComparison().remainingAllowanceVolume
                  << ", targetVolume="
                  << session.pocketComparison().targetVolume
                  << ", camOps=" << result.camInput.operations.size()
                  << ", transient="
                  << session.pocketTransientOperations().size() << '\n';
        if (session.pocketSimulation().pocketZMapData) {
            std::cerr << "  zmap scopes="
                      << session.pocketSimulation().pocketZMapData
                             ->featureAllowanceScopes.size()
                      << ", resolution="
                      << session.pocketSimulation().pocketZMapData
                             ->resolution << '\n';
        }
        std::cerr << "  allowance mesh=" << allowanceMeshOk
                  << ", error=" << allowanceMeshError << '\n';
        if (!session.pocketComparison().featureAllowances.empty()) {
            const auto& allowance =
                session.pocketComparison().featureAllowances.front();
            std::cerr << "  target=" << allowance.targetRemovalVolume
                      << ", remaining="
                      << allowance.remainingAllowanceVolume
                      << ", completion=" << allowance.completionRatio
                      << '\n';
        }
    }
    return expect(
        valid,
        "profile IPW must publish the same checkpoint and move-replay inputs as pocket IPW");
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    if (!automaticHoleRunnerReportsQueueFull())
        return 1;
    if (!runPocketSimulationChecks())
        return 1;
    using oap::app::HoleToolpathCommandKind;
    if (!holeOpenGlAdapterPreservesDrillGeometry())
        return 1;
    if (!profileRegionsFollowSixFaceSetupOrder())
        return 1;
    if (!expect(
            oap::app::resolveHoleToolpathCommand(false, false) ==
                    HoleToolpathCommandKind::GenerateFromRecognition &&
                oap::app::resolveHoleToolpathCommand(true, false) ==
                    HoleToolpathCommandKind::GenerateFromRecognition &&
                oap::app::resolveHoleToolpathCommand(true, true) ==
                    HoleToolpathCommandKind::RebuildInstalledProgram,
            "the hole command must route first generation and stale-source jobs through recognition") ||
        !expect(
            !oap::app::holeToolpathGenerationAvailable(
                true, false, false) &&
                !oap::app::holeToolpathGenerationAvailable(
                    false, true, false) &&
                oap::app::holeToolpathGenerationAvailable(
                    true, true, false) &&
                !oap::app::holeToolpathGenerationAvailable(
                    true, true, true),
            "hole generation requires assigned library tools") ||
        !expect(
            !oap::app::pocketToolpathGenerationAvailable(
                true, false, false) &&
                !oap::app::pocketToolpathGenerationAvailable(
                    false, true, false) &&
                oap::app::pocketToolpathGenerationAvailable(
                    true, true, false),
            "pocket generation requires assigned library tools")) {
        return 1;
    }

    if (!toolLibraryStartupImportsSimulationLibrary())
        return 1;
    if (!catalogMigrationPreservesAnExplicitZeroLength())
        return 1;
    if (!globalToolAssetsPersistIndependently())
        return 1;
    if (!profileIpwBuildsPocketEquivalentPlaybackInput())
        return 1;

    bool progressPresented = false;
    bool cancellationForwarded = false;
    bool taskReady = false;
    oap::app::WorkflowProgressRequest progressRequest;
    progressRequest.title = QStringLiteral("test progress");
    progressRequest.initialTotal = 2;
    progressRequest.snapshot = [] {
        return oap::app::WorkflowProgressSnapshot{1, 2, {}};
    };
    progressRequest.ready = [&taskReady] { return taskReady; };
    progressRequest.cancel = [&cancellationForwarded] {
        cancellationForwarded = true;
    };
    oap::app::waitForWorkflowTask(
        [&](const oap::app::WorkflowProgressRequest& presented) {
            progressPresented =
                presented.title == QStringLiteral("test progress") &&
                presented.snapshot().completed == 1;
            presented.cancel();
            taskReady = true;
        },
        progressRequest);
    if (!expect(
            progressPresented && cancellationForwarded && taskReady,
            "workflow interaction must forward progress and cancellation without Widgets")) {
        return 1;
    }

    oap::app::StockSimulationCoordinator stock;
    const auto cancel = stock.beginSimulation();
    if (!expect(stock.simulationRunning() && !cancel->load(),
                "simulation begin must publish one live cancel token")) {
        return 1;
    }
    stock.requestCancel();
    if (!expect(cancel->load(), "simulation cancel must reach the runner"))
        return 1;
    stock.finishSimulation();
    const auto secondCancel = stock.beginSimulation();
    if (!expect(stock.busy() && secondCancel,
                "simulation work must participate in busy state"))
        return 1;
    stock.finishSimulation();

    oap::app::StockSourceSettings invalidSource;
    invalidSource.sourceMode = 2;
    invalidSource.stepPath =
        QStringLiteral("Z:/definitely/missing/stock.step");
    QString sourceError;
    if (!expect(
            !oap::app::StockSimulationWorkflowController::validateSource(
                invalidSource, sourceError) && !sourceError.isEmpty(),
            "an imported stock source must reference an existing file")) {
        return 1;
    }

    oap::app::StockSimulationSession reportSession;
    reportSession.hasHoleSimulation() = true;
    reportSession.holeSimulation().initialVolume = 100.0;
    reportSession.holeSimulation().finalVolume = 80.0;
    reportSession.holeSimulation().removedVolume = 20.0;
    reportSession.holeSimulation().complete = true;
    const QString holeSummary =
        oap::app::StockSimulationWorkflowController::holeSummary(
            reportSession, {}, {});
    if (!expect(holeSummary.contains(QStringLiteral("去除 20.000")),
                "hole simulation reporting must be UI-independent")) {
        return 1;
    }

    oap::app::IpwDisplayMeshResult staleDisplay;
    staleDisplay.displayMode =
        oap::app::IpwOcctDisplayMode::SurfaceWithEdges;
    const auto discarded = stock.evaluateDisplayResult(
        staleDisplay, oap::app::IpwOcctDisplayMode::Off, true);
    if (!expect(
            discarded.disposition ==
                oap::app::IpwDisplayPublicationDisposition::Discard,
            "an IPW result must not revive display mode Off")) {
        return 1;
    }

    oap::app::IpwDisplayMeshResult readyDisplay;
    readyDisplay.displayMode =
        oap::app::IpwOcctDisplayMode::SurfaceWithEdges;
    readyDisplay.key.contentFingerprint = "stock-ready";
    readyDisplay.key.kind =
        oap::app::IpwDisplayArtifactKind::Overcut;
    readyDisplay.mesh = new Poly_Triangulation(
        3, 1, Standard_False, Standard_False);
    readyDisplay.mesh->SetNode(1, gp_Pnt(0.0, 0.0, 0.0));
    readyDisplay.mesh->SetNode(2, gp_Pnt(1.0, 0.0, 0.0));
    readyDisplay.mesh->SetNode(3, gp_Pnt(0.0, 1.0, 0.0));
    readyDisplay.mesh->SetTriangle(1, Poly_Triangle(1, 2, 3));
    const auto publication = stock.evaluateDisplayResult(
        readyDisplay,
        oap::app::IpwOcctDisplayMode::SurfaceWithEdges,
        true);
    if (!expect(
            publication.disposition ==
                    oap::app::IpwDisplayPublicationDisposition::Ready &&
                publication.cached &&
                std::abs(publication.style.red - 0.92) < 1.0e-12,
            "IPW publication policy must own caching and semantic style")) {
        return 1;
    }

    oap::manufacturing::MachiningJob pocketJob;
    pocketJob.definition().part.sourceFingerprint = "part-fingerprint";
    oap::manufacturing::SetupDefinition setup;
    setup.id = "setup-0";
    setup.setupIndex = 0;
    pocketJob.definition().setups.push_back(setup);
    oap::manufacturing::HoleOperationDefinition holeDefinition;
    holeDefinition.id = "hole-0";
    holeDefinition.setupId = setup.id;
    pocketJob.definition().holeOperations.push_back(holeDefinition);
    oap::manufacturing::PocketOperationDefinition pocketDefinition;
    pocketDefinition.id = "pocket-0";
    pocketDefinition.setupId = setup.id;
    pocketJob.definition().pocketOperations.push_back(pocketDefinition);
    oap::app::PocketPreviewEntry pocketPreview;
    pocketPreview.operation.operationId = pocketDefinition.id;
    pocketPreview.operation.setupIndex = 0;
    const std::vector<oap::app::PocketPreviewEntry> pocketPreviews{
        pocketPreview};
    oap::toolpath::ToolpathParams effectiveParams;
    effectiveParams.tool.diameter = 10.0;
    oap::app::StockSourceSettings generatedSource;
    generatedSource.sourceMode = 1;
    generatedSource.marginX = 1.0;
    generatedSource.marginY = 2.0;
    generatedSource.marginZ = 3.0;
    const TopoDS_Shape box =
        BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    oap::toolpath::PocketToolpathBatch simulationBatch;
    simulationBatch.operations = {pocketPreview.operation};
    simulationBatch.effectiveParams = effectiveParams;
    pocketJob.setPocketResult(simulationBatch,
        oap::manufacturing::MachiningJobStorage::pocketExecutionFingerprint(
            pocketJob.definition()).toStdString());
    oap::app::PocketSimulationPreparation preparation =
        oap::app::StockSimulationWorkflowController::preparePocket(
            pocketJob, 0, generatedSource, {}, 5,
            box, box);
    if (!expect(
            preparation.ok &&
                preparation.includedOperationIds ==
                    std::vector<std::string>{"hole-0", "pocket-0"},
            "pocket IPW selection must preserve the shared hole-to-pocket order")) {
        return 1;
    }
    oap::app::IpwSimulationRunResult installResult;
    installResult.simulation.initialStock = box;
    installResult.simulation.removedVolume = 4.0;
    oap::app::StockSimulationWorkflowController::installPocketResult(
        reportSession, pocketJob, preparation, installResult);
    if (!expect(
            reportSession.hasPocketSimulation() &&
                reportSession.pocketUsesUnifiedIpw() &&
                reportSession.pocketSourceFingerprint() ==
                    QString::fromStdString(preparation.stockFingerprint),
            "pocket simulation installation must update the session atomically")) {
        return 1;
    }

    oap::manufacturing::ProfileOperationDefinition profileDefinition;
    profileDefinition.id = "profile-0";
    profileDefinition.setupId = setup.id;
    profileDefinition.sequence = 2;
    pocketJob.definition().profileOperations.push_back(profileDefinition);
    oap::toolpath::ProfileToolpathOperation profilePreview;
    profilePreview.operationId = profileDefinition.id;
    profilePreview.setupIndex = 0;
    oap::toolpath::ToolpathMove profileCut;
    profileCut.type = oap::toolpath::ToolpathMoveType::Cut;
    profileCut.from = gp_Pnt(0.0, 0.0, 5.0);
    profileCut.to = gp_Pnt(20.0, 0.0, 5.0);
    profilePreview.contour.moves.push_back(profileCut);
    const std::vector<oap::toolpath::ProfileToolpathOperation>
        profilePreviews{profilePreview};
    const oap::app::ProfileSimulationPreparation profilePreparation =
        oap::app::StockSimulationWorkflowController::prepareProfile(
            pocketJob, profilePreviews, 0, generatedSource, {}, 5,
            effectiveParams, box, box);
    if (!expect(
            profilePreparation.ok &&
                profilePreparation.operations.size() == 1 &&
                profilePreparation.includedOperationIds ==
                    std::vector<std::string>{
                        "hole-0", "pocket-0", "profile-0"},
            "profile IPW selection must include prior same-setup material checkpoints")) {
        return 1;
    }
    oap::app::IpwSimulationRunResult profileInstallResult;
    profileInstallResult.simulation.initialStock = box;
    profileInstallResult.simulation.removedVolume = 2.0;
    oap::app::StockSimulationWorkflowController::installProfileResult(
        reportSession, pocketJob,
        profilePreparation, profileInstallResult);
    if (!expect(
            reportSession.hasPocketSimulation() &&
                reportSession.pocketUsesUnifiedIpw() &&
                reportSession.pocketSourceFingerprint() ==
                    QString::fromStdString(
                        profilePreparation.stockFingerprint),
            "profile simulation installation must publish into the unified IPW workspace")) {
        return 1;
    }

    oap::app::MachiningWorkspaceSession workspace;
    const auto exportResult =
        oap::app::ToolpathExportController::prepare(workspace);
    if (!expect(!exportResult.ok && !exportResult.error.isEmpty(),
                "an empty workspace must not create an export plan")) {
        return 1;
    }
    if (!expect(!oap::app::HoleToolpathWorkflowController::commit({}).ok,
                "an incomplete hole commit must fail without mutation")) {
        return 1;
    }
    if (!expect(!oap::app::PocketToolpathWorkflowController::prepareCandidateJob({}).ok,
                "an incomplete pocket commit must fail without mutation")) {
        return 1;
    }
    if (!expect(!oap::app::ProfileToolpathWorkflowController::commit({}).ok,
                "an incomplete profile commit must fail without mutation")) {
        return 1;
    }
    if (!expect(
            oap::app::HoleToolpathGenerationUseCase::execute({}).failure ==
                oap::app::HoleToolpathGenerationFailure::InvalidRequest,
            "an incomplete hole generation use case must fail explicitly")) {
        return 1;
    }
    if (!expect(
            oap::app::PocketToolpathGenerationUseCase::execute({}).failure ==
                oap::app::PocketToolpathGenerationFailure::InvalidRequest,
            "an incomplete pocket generation use case must fail explicitly")) {
        return 1;
    }
    if (!expect(
            oap::app::ProfileToolpathGenerationUseCase::execute({}).failure ==
                oap::app::ProfileToolpathGenerationFailure::InvalidRequest,
            "an incomplete profile generation use case must fail explicitly")) {
        return 1;
    }
    const oap::tooling::ToolLibrary automaticLibrary;
    const std::vector<oap::toolpath::PocketRegion> noRegions;
    const std::vector<oap::toolpath::SkippedRegion> noSkippedRegions;
    oap::MoldCoordinateFrame emptyMoldFrame;
    oap::toolpath::MoldSetupWorkFrames emptySetupFrames;
    oap::app::PocketToolpathGenerationRequest emptyPocketRequest;
    emptyPocketRequest.regions = &noRegions;
    emptyPocketRequest.buildSkipped = &noSkippedRegions;
    emptyPocketRequest.params = &effectiveParams;
    emptyPocketRequest.moldFrame = &emptyMoldFrame;
    emptyPocketRequest.currentJob = &pocketJob;
    emptyPocketRequest.setupFrames = &emptySetupFrames;
    if (!expect(
            oap::app::PocketToolpathGenerationUseCase::execute(emptyPocketRequest).failure ==
                oap::app::PocketToolpathGenerationFailure::InvalidRequest,
            "pocket generation must reject missing library instead of using a manual tool")) return 1;
    emptyPocketRequest.automaticLibrary = &automaticLibrary;
    if (!expect(
            oap::app::PocketToolpathGenerationUseCase::execute(
                emptyPocketRequest).failure ==
                oap::app::PocketToolpathGenerationFailure::EmptyProgram,
            "a pocket request without recognized candidates must remain empty")) {
        return 1;
    }
    const std::vector<oap::toolpath::ProfileRegion> noProfileRegions;
    oap::app::ProfileToolpathGenerationRequest emptyProfileRequest;
    emptyProfileRequest.recognizedRegions = &noProfileRegions;
    emptyProfileRequest.params = &effectiveParams;
    emptyProfileRequest.moldFrame = &emptyMoldFrame;
    emptyProfileRequest.currentJob = &pocketJob;
    emptyProfileRequest.setupFrames = &emptySetupFrames;
    if (!expect(
            oap::app::ProfileToolpathGenerationUseCase::execute(emptyProfileRequest).failure ==
                oap::app::ProfileToolpathGenerationFailure::InvalidRequest,
            "profile generation must reject missing library instead of using a manual tool")) return 1;
    emptyProfileRequest.automaticLibrary = &automaticLibrary;
    if (!expect(
            oap::app::ProfileToolpathGenerationUseCase::execute(
                emptyProfileRequest).failure ==
                oap::app::ProfileToolpathGenerationFailure::EmptyProgram,
            "a profile request without recognized regions must remain empty")) {
        return 1;
    }
    emptyPocketRequest.candidateCount = 1;
    if (!expect(
            oap::app::PocketToolpathGenerationUseCase::execute(
                emptyPocketRequest).failure ==
                oap::app::PocketToolpathGenerationFailure::EmptyProgram,
            "a pocket request without buildable regions must remain empty")) {
        return 1;
    }

    if (!runPocketPresentationChecks())
        return 1;

    if (!expect(
            !oap::app::HoleStockSimulationUseCase::execute({}).ok,
            "an incomplete hole stock use case must fail explicitly")) {
        return 1;
    }
    if (!expect(
            oap::app::PocketStockSimulationUseCase::execute({}).failure ==
                oap::app::PocketStockSimulationFailure::InvalidRequest,
            "an incomplete pocket stock use case must fail explicitly")) {
        return 1;
    }

    oap::manufacturing::MachiningJob holeSimulationJob;
    oap::toolpath::HoleToolpathBatch holeSimulationBatch;
    holeSimulationBatch.effectiveParams = effectiveParams;
    holeSimulationBatch.machining.operations.push_back(
        oap::toolpath::HoleMachiningOperation{});
    holeSimulationJob.setHoleResult(
        std::move(holeSimulationBatch), "hole-simulation-test");
    oap::app::StockSimulationSession holeSimulationSession;
    oap::app::StockSimulationCoordinator holeSimulationCoordinator;
    oap::app::HoleStockSimulationUseCaseRequest holeSimulationRequest;
    holeSimulationRequest.session = &holeSimulationSession;
    holeSimulationRequest.coordinator = &holeSimulationCoordinator;
    holeSimulationRequest.job = &holeSimulationJob;
    holeSimulationRequest.buildInitialStock =
        [box](const oap::app::StockSourceSettings&,
              TopoDS_Shape& shape, int& count, QString&) {
            shape = box;
            count = 1;
            return true;
        };
    holeSimulationRequest.buildTarget =
        [box](TopoDS_Shape& shape, int& count, QString&) {
            shape = box;
            count = 1;
            return true;
        };
    holeSimulationRequest.runner =
        [](oap::app::HoleStockSimulationRequest,
           std::shared_ptr<std::atomic_bool>) {
            oap::app::HoleStockSimulationRunResult result;
            result.cancelled = true;
            return result;
        };
    const oap::app::HoleStockRunOutcome cancelledHoleSimulation =
        oap::app::HoleStockSimulationUseCase::execute(
            holeSimulationRequest);
    if (!expect(
            cancelledHoleSimulation.ok &&
                cancelledHoleSimulation.status.contains(
                    QStringLiteral("已取消")) &&
                !holeSimulationCoordinator.busy(),
            "hole stock cancellation must install no partial snapshot and finish lifecycle")) {
        return 1;
    }

    const oap::app::PocketStockSimulationDialogState setupState =
        oap::app::PocketStockSimulationUseCase::buildDialogState(
            reportSession, simulationBatch.operations, 12, true,
            64u * 1024u * 1024u, QStringLiteral("D:/tmp"));
    if (!expect(
            setupState.setups.size() == 1 &&
                setupState.selectedSetup == 0 &&
                setupState.setups.front().label.contains(
                    QStringLiteral("1 道腔工序")) &&
                setupState.displayCacheMiB == 64,
            "pocket stock dialog state must group setup choices deterministically")) {
        return 1;
    }

    oap::app::StockSimulationSession cancelledPocketSession;
    oap::app::StockSimulationCoordinator cancelledPocketCoordinator;
    oap::app::PocketStockSimulationUseCaseRequest cancelledPocketRequest;
    cancelledPocketRequest.settings = setupState;
    cancelledPocketRequest.settings.source.sourceMode = 1;
    cancelledPocketRequest.session = &cancelledPocketSession;
    cancelledPocketRequest.coordinator = &cancelledPocketCoordinator;
    cancelledPocketRequest.job = &pocketJob;
    cancelledPocketRequest.generatedInitialStock = box;
    cancelledPocketRequest.targetShape = box;
    cancelledPocketRequest.persistSettings = false;
    cancelledPocketRequest.runner =
        [](oap::manufacturing::MachiningJob&,
           oap::app::IpwSimulationRequest,
           std::shared_ptr<std::atomic_bool>) {
            oap::app::IpwSimulationRunResult result;
            result.cancelled = true;
            return result;
        };
    const oap::app::PocketStockSimulationUseCaseResult
        cancelledPocketSimulation =
            oap::app::PocketStockSimulationUseCase::execute(
                cancelledPocketRequest);
    if (!expect(
            cancelledPocketSimulation.failure ==
                    oap::app::PocketStockSimulationFailure::Cancelled &&
                !cancelledPocketSession.hasPocketSimulation() &&
                !cancelledPocketCoordinator.busy(),
            "pocket stock cancellation must reject partial installation and finish lifecycle")) {
        return 1;
    }
    oap::app::StockSimulationSession emptyPocketSession;
    oap::app::StockSimulationCoordinator emptyPocketCoordinator;
    oap::app::PocketStockSimulationUseCaseRequest emptyPocketSimulationRequest =
        cancelledPocketRequest;
    emptyPocketSimulationRequest.session = &emptyPocketSession;
    emptyPocketSimulationRequest.coordinator = &emptyPocketCoordinator;
    emptyPocketSimulationRequest.runner =
        [](oap::manufacturing::MachiningJob&,
           oap::app::IpwSimulationRequest,
           std::shared_ptr<std::atomic_bool>) {
            return oap::app::IpwSimulationRunResult{};
        };
    const auto emptyPocketSimulation =
        oap::app::PocketStockSimulationUseCase::execute(
            emptyPocketSimulationRequest);
    if (!expect(
            emptyPocketSimulation.failure ==
                    oap::app::PocketStockSimulationFailure::EmptyResult &&
                !emptyPocketSession.hasPocketSimulation() &&
                !emptyPocketCoordinator.busy(),
            "an empty IPW result must not publish pocket simulation state")) {
        return 1;
    }
    oap::app::PocketStockSimulationUseCaseRequest invalidImportedPocket =
        cancelledPocketRequest;
    invalidImportedPocket.settings.source.sourceMode = 2;
    invalidImportedPocket.settings.source.stepPath =
        QStringLiteral("Z:/definitely/missing/imported-stock.step");
    const auto invalidImportedResult =
        oap::app::PocketStockSimulationUseCase::execute(
            invalidImportedPocket);
    if (!expect(
            invalidImportedResult.failure ==
                oap::app::PocketStockSimulationFailure::InvalidSource,
            "an invalid imported stock must fail before the IPW runner")) {
        return 1;
    }

    // This fixture belongs to camera framing, independent of preview/export tests.
    oap::toolpath::PocketToolpathOperation acceptanceSmall;
    acceptanceSmall.kind = oap::FeatureKind::Slot;
    acceptanceSmall.sourceFaceIds = {"F2", "F1"};
    for (int index = 0; index < 3; ++index) {
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = gp_Pnt(static_cast<double>(index), 0.0, 0.0);
        move.to = gp_Pnt(static_cast<double>(index + 1), 0.0, 0.0);
        acceptanceSmall.clearing.moves.push_back(move);
    }
    acceptanceSmall.operationId = "small";
    acceptanceSmall.outwardToolAxis = gp_Vec(0.0, 0.0, 1.0);
    oap::toolpath::PocketToolpathOperation acceptanceLarge =
        acceptanceSmall;
    acceptanceLarge.operationId = "large";
    acceptanceLarge.clearing.moves.clear();
    oap::toolpath::ToolpathMove acceptanceMove;
    acceptanceMove.type = oap::toolpath::ToolpathMoveType::Cut;
    acceptanceMove.from = gp_Pnt(0.0, 0.0, 0.0);
    acceptanceMove.to = gp_Pnt(10.0, 5.0, 0.0);
    acceptanceLarge.clearing.moves.push_back(acceptanceMove);
    const auto acceptanceView =
        oap::app::PocketStockSimulationUseCase::
            selectVisualAcceptanceView(
                {acceptanceSmall, acceptanceLarge}, 6.0);
    if (!expect(
            acceptanceView && acceptanceView->valid &&
                acceptanceView->operationId == QStringLiteral("large") &&
                acceptanceView->halfHeight >= 6.0,
            "visual acceptance must select the largest stable cutting footprint")) {
        return 1;
    }

    oap::app::MachiningWorkspaceSession holeWorkbench;
    oap::toolpath::HoleMachiningInput workbenchHole;
    workbenchHole.kind = oap::FeatureKind::CountersinkHole;
    workbenchHole.entry = gp_Pnt(0.0, 0.0, 10.0);
    workbenchHole.bottom = gp_Pnt(0.0, 0.0, 0.0);
    workbenchHole.radius = 8.0;
    workbenchHole.stageWallFaceIds = {"F-WALL"};
    workbenchHole.sourceFaceIds = {"F-WALL", "F-FLOOR"};
    workbenchHole.setupFace = oap::MoldSetupFace::PositiveZ;
    workbenchHole.mappingState =
        oap::toolpath::HoleMachiningMappingState::Ready;
    oap::Model generationModel;
    oap::StepIndexer generationIndexer;
    const std::vector<oap::toolpath::HoleMachiningInput> generationInputs{workbenchHole};
    oap::app::HoleToolpathGenerationRequest missingLibraryRequest;
    missingLibraryRequest.recognized = &generationInputs;
    missingLibraryRequest.model = &generationModel;
    missingLibraryRequest.indexer = &generationIndexer;
    missingLibraryRequest.moldFrame = &emptyMoldFrame;
    missingLibraryRequest.params = &effectiveParams;
    missingLibraryRequest.currentJob = &holeWorkbench.job();
    missingLibraryRequest.setupFrames = &emptySetupFrames;
    if (!expect(
            oap::app::HoleToolpathGenerationUseCase::execute(missingLibraryRequest).failure ==
                oap::app::HoleToolpathGenerationFailure::InvalidRequest,
            "hole generation must reject missing library instead of using a manual tool")) return 1;
    const std::string workbenchKey =
        oap::tooling::holeStageKey(workbenchHole);
    holeWorkbench.recognizedHoleSources().push_back(
        {workbenchHole, workbenchHole.sourceFaceIds, workbenchKey});
    holeWorkbench.activePreview() =
        oap::app::ActiveToolpathPreview::Hole;
    oap::toolpath::HoleToolpathBatch installedBatch;
    oap::toolpath::HoleMachiningOperation installedOperation;
    installedOperation.sourceHoleIndex = 0;
    installedOperation.setupIndex = 0;
    installedOperation.setupFace = oap::MoldSetupFace::PositiveZ;
    installedOperation.phase =
        oap::toolpath::HoleMachiningPhase::FlatEndMill;
    installedBatch.machining.operations.push_back(installedOperation);
    holeWorkbench.job().setHoleResult(installedBatch, "active-hole");

    if (!expect(
            oap::app::HoleToolpathWorkbenchCoordinator::selectedStageIndex(
                holeWorkbench, {"F-WALL"}) == 0,
            "hole workbench selection must resolve one complete stage")) {
        return 1;
    }
    oap::app::PlaybackSession workbenchPlayback;
    oap::app::PlaybackMove workbenchMove;
    workbenchMove.kind = oap::FeatureKind::CountersinkHole;
    workbenchMove.sourceIndex = 0;
    workbenchMove.setupIndex = 0;
    workbenchMove.setupFace = oap::MoldSetupFace::PositiveZ;
    workbenchMove.phase =
        oap::toolpath::HoleMachiningPhase::FlatEndMill;
    workbenchMove.toolBatchIndex = 0;
    workbenchMove.operationId = "hole-op-a";
    workbenchPlayback.moves().push_back(workbenchMove);
    workbenchPlayback.moves().push_back(workbenchMove);
    oap::app::PlaybackMove secondWorkbenchOperation = workbenchMove;
    secondWorkbenchOperation.operationId = "hole-op-b";
    workbenchPlayback.moves().push_back(secondWorkbenchOperation);
    workbenchPlayback.toolBatchStarts().push_back(0);
    const oap::app::HoleToolTreeModel disabledTree =
        oap::app::HoleToolpathWorkbenchCoordinator::buildTreeModel(
            holeWorkbench, workbenchPlayback.moves(),
            workbenchPlayback.toolBatchStarts());
    if (!expect(
            disabledTree.batches.size() == 1 &&
                disabledTree.batches.front().operations.size() == 2 &&
                disabledTree.batches.front().operations.front().operationId ==
                    QStringLiteral("hole-op-a") &&
                disabledTree.batches.front().operations.front().family ==
                    oap::app::ToolpathBatchFamily::Hole &&
                disabledTree.batches.front().operations.front().sourceIndex == 0 &&
                disabledTree.batches.front().operations.front().stageKey ==
                    QString::fromStdString(workbenchKey) &&
                disabledTree.batches.front().operations.front().detailText.contains(
                    QStringLiteral("自动孔加工")) &&
                disabledTree.batches.front().operations.front().startMoveIndex == 0 &&
                disabledTree.batches.front().operations.front().endMoveIndex == 2 &&
                disabledTree.batches.front().operations.back().operationId ==
                    QStringLiteral("hole-op-b") &&
                disabledTree.batches.front().operations.back().startMoveIndex == 2 &&
                disabledTree.batches.front().operations.back().endMoveIndex == 3,
            "the tree model must expose operation ranges for automatic hole toolpaths")) {
        return 1;
    }

    oap::app::MachiningWorkspaceSession bindingWorkspace = holeWorkbench;
    oap::manufacturing::HoleOperationDefinition boundOperation;
    boundOperation.id = "hole-formal-a";
    boundOperation.sequence = 3;
    boundOperation.enabled = false;
    boundOperation.state =
        oap::manufacturing::JobOperationState::Stale;
    oap::manufacturing::HoleTargetReference boundTarget;
    boundTarget.sourceHoleIndex = 0;
    boundTarget.coveredSourceHoleIndices = {0, 2};
    boundOperation.targets.push_back(std::move(boundTarget));
    bindingWorkspace.job().definition().holeOperations.push_back(
        std::move(boundOperation));
    oap::app::PlaybackMove boundMove = workbenchMove;
    boundMove.operationId = "hole-formal-a-r2";
    const auto coverageBindings =
        oap::app::HoleToolpathWorkbenchCoordinator::
            buildCoverageOperationBindings(
                bindingWorkspace, {boundMove});
    if (!expect(
            coverageBindings.size() == 1 &&
                coverageBindings.front().formalOperationId ==
                    QStringLiteral("hole-formal-a") &&
                coverageBindings.front().sourceHoleIndices ==
                    QVector<int>({0, 2}) &&
                coverageBindings.front().hasPlayback &&
                !coverageBindings.front().enabled &&
                coverageBindings.front().state ==
                    oap::manufacturing::JobOperationState::Stale,
            "coverage bindings must keep stable formal IDs and explicit stale/playback state")) {
        return 1;
    }

    oap::app::PlaybackSession mixedToolPlayback;
    mixedToolPlayback.moves().push_back(workbenchMove);
    oap::app::PlaybackMove automaticPocketMove;
    automaticPocketMove.kind = oap::FeatureKind::Pocket;
    automaticPocketMove.setupIndex = 0;
    automaticPocketMove.setupFace = oap::MoldSetupFace::PositiveZ;
    automaticPocketMove.tool.valid = true;
    automaticPocketMove.tool.toolNumber = 12;
    automaticPocketMove.tool.toolName = "D10 flat end mill";
    automaticPocketMove.tool.toolType = "end_mill";
    automaticPocketMove.tool.diameter = 10.0;
    mixedToolPlayback.moves().push_back(automaticPocketMove);
    oap::app::PlaybackMove unboundPocketMove;
    unboundPocketMove.kind = oap::FeatureKind::Slot;
    unboundPocketMove.setupIndex = 1;
    unboundPocketMove.setupFace = oap::MoldSetupFace::PositiveX;
    mixedToolPlayback.moves().push_back(unboundPocketMove);
    mixedToolPlayback.toolBatchStarts() = {0, 1, 2};
    holeWorkbench.pocketPreviewEffectiveParams().tool.diameter = 8.0;
    const oap::app::HoleToolTreeModel mixedToolTree =
        oap::app::HoleToolpathWorkbenchCoordinator::buildTreeModel(
            holeWorkbench, mixedToolPlayback.moves(),
            mixedToolPlayback.toolBatchStarts());
    if (!expect(
            mixedToolTree.batches.size() == 3 &&
                mixedToolTree.batches[1].family ==
                    oap::app::ToolpathBatchFamily::Pocket &&
                mixedToolTree.batches[1].toolLabel.contains(
                    QStringLiteral("T12")) &&
                mixedToolTree.batches[2].family ==
                    oap::app::ToolpathBatchFamily::Pocket &&
                mixedToolTree.batches[2].toolLabel == QStringLiteral("未绑定刀具"),
            "the tool tree must show library tools and mark unbound tools without inventing a diameter")) {
        return 1;
    }

    oap::app::PlaybackSession profilePlayback;
    const auto appendProfileMove = [&profilePlayback](
        const char* operationId, oap::MoldSetupFace setupFace,
        int batchIndex) {
        oap::app::PlaybackMove move;
        move.kind = oap::FeatureKind::Pocket;
        move.operationType =
            oap::toolpath::ToolpathOperationType::ProfileContour;
        move.operationId = operationId;
        move.setupFace = setupFace;
        move.setupIndex = static_cast<int>(setupFace);
        move.toolBatchIndex = batchIndex;
        profilePlayback.toolBatchStarts().push_back(
            static_cast<int>(profilePlayback.moves().size()));
        profilePlayback.moves().push_back(std::move(move));
    };
    appendProfileMove(
        "PROFILE1", oap::MoldSetupFace::PositiveZ, 0);
    appendProfileMove(
        "PROFILE2", oap::MoldSetupFace::PositiveX, 1);
    appendProfileMove(
        "PROFILE3", oap::MoldSetupFace::NegativeY, 2);
    appendProfileMove(
        "PROFILE4", oap::MoldSetupFace::NegativeZ, 3);
    const oap::app::HoleToolTreeModel profileTree =
        oap::app::HoleToolpathWorkbenchCoordinator::buildTreeModel(
            holeWorkbench, profilePlayback.moves(),
            profilePlayback.toolBatchStarts());
    if (!expect(
            profileTree.batches.size() == 4 &&
                profileTree.batches[0].operations.size() == 1 &&
                profileTree.batches[0].operations.front().operationId ==
                    QStringLiteral("PROFILE1") &&
                profileTree.batches[0].family ==
                    oap::app::ToolpathBatchFamily::Profile &&
                profileTree.batches[0].setupOrder ==
                    static_cast<int>(oap::MoldSetupFace::PositiveZ),
            "the CAM tree must retain all profile setups, including +Z")) {
        return 1;
    }

    oap::app::PlaybackMove inspectorBaseMove;
    inspectorBaseMove.operationType = oap::toolpath::ToolpathOperationType::ProfileContour;
    inspectorBaseMove.setupFace = oap::MoldSetupFace::PositiveZ;
    inspectorBaseMove.setupIndex = 0;
    inspectorBaseMove.tool.valid = true;
    inspectorBaseMove.tool.toolNumber = 61;
    inspectorBaseMove.tool.toolType = "end_mill";
    inspectorBaseMove.tool.diameter = 6.0;

    oap::app::MachiningWorkspaceSession inspectorWorkspace;
    oap::manufacturing::SetupDefinition inspectorSetup;
    inspectorSetup.id = "setup-inspector";
    inspectorSetup.setupIndex = 0;
    inspectorSetup.face = oap::MoldSetupFace::PositiveZ;
    inspectorSetup.valid = true;
    inspectorSetup.confirmation.confirmed = true;
    inspectorWorkspace.job().definition().setups.push_back(inspectorSetup);
    oap::manufacturing::JobToolController inspectorTool;
    inspectorTool.id = "tool-inspector";
    inspectorTool.tool = inspectorBaseMove.tool;
    inspectorWorkspace.job().definition().toolControllers.push_back(
        inspectorTool);
    oap::manufacturing::ProfileOperationDefinition inspectorDefinition;
    inspectorDefinition.id = "profile-inspector";
    inspectorDefinition.setupId = inspectorSetup.id;
    inspectorDefinition.toolControllerId = inspectorTool.id;
    inspectorDefinition.algorithmId =
        oap::toolpath::kBuiltinProfileAlgorithmId;
    inspectorDefinition.sequence = 4;
    inspectorDefinition.state =
        oap::manufacturing::JobOperationState::Stale;
    inspectorDefinition.inputFingerprint = "profile-inspector-input";
    inspectorDefinition.target.rootFaceId = "root-face";
    inspectorDefinition.target.rootWireId = "root-wire";
    inspectorDefinition.target.machiningFaceIds = {"side-a", "side-b", "side-c"};
    inspectorWorkspace.job().definition().profileOperations.push_back(
        inspectorDefinition);
    oap::manufacturing::ipw::IpwOperationDefinition predecessorNode;
    predecessorNode.id = "rough-formal";
    predecessorNode.setupId = inspectorSetup.id;
    predecessorNode.toolControllerId = inspectorTool.id;
    predecessorNode.inputFingerprint = "rough-input";
    predecessorNode.sequence = 3;
    predecessorNode.kind =
        oap::manufacturing::ipw::IpwOperationKind::Pocket;
    oap::manufacturing::ipw::IpwOperationDefinition profileNode;
    profileNode.id = inspectorDefinition.id;
    profileNode.setupId = inspectorDefinition.setupId;
    profileNode.toolControllerId = inspectorDefinition.toolControllerId;
    profileNode.inputFingerprint = inspectorDefinition.inputFingerprint;
    profileNode.sequence = inspectorDefinition.sequence;
    profileNode.kind =
        oap::manufacturing::ipw::IpwOperationKind::Profile;
    inspectorWorkspace.job().ipwSession().configure(
        "stock-inspector", "engine-inspector",
        {predecessorNode, profileNode});
    oap::app::PlaybackMove inspectorMove = inspectorBaseMove;
    inspectorMove.operationId = "profile-inspector-r0";
    const oap::app::HoleToolTreeModel inspectorTree =
        oap::app::HoleToolpathWorkbenchCoordinator::buildTreeModel(
            inspectorWorkspace, {inspectorMove}, {0});
    const oap::app::ToolpathOperationView* inspectorView = nullptr;
    for (const auto& batch : inspectorTree.batches) {
        for (const auto& operation : batch.operations) {
            if (operation.formalOperationId ==
                QStringLiteral("profile-inspector")) {
                inspectorView = &operation;
            }
        }
    }
    const auto hasInspectorIssue = [inspectorView](const QString& code) {
        return inspectorView && std::any_of(
            inspectorView->issues.begin(), inspectorView->issues.end(),
            [&code](const oap::app::ToolpathOperationIssueView& issue) {
                return issue.code == code;
            });
    };
    if (!expect(
            inspectorView && inspectorView->targetCount == 1 &&
                inspectorView->targetFaceCount == 3 &&
                inspectorView->protectedFaceCount == 0 &&
                inspectorView->boundaryFaceCount == 0 &&
                inspectorView->boundaryMode.contains(
                    QStringLiteral("根面真实外环")) &&
                inspectorView->predecessorOperationId ==
                    QStringLiteral("rough-formal") &&
                inspectorView->ipwStatus == QStringLiteral("待重建") &&
                !inspectorView->parentIpwStateKey.isEmpty() &&
                !inspectorView->ipwStateKey.isEmpty() &&
                hasInspectorIssue(
                    QStringLiteral("formal.upstream_stale")) &&
                hasInspectorIssue(QStringLiteral("ipw.node_dirty")) &&
                hasInspectorIssue(
                    QStringLiteral("ipw.parent_checkpoint_missing")) &&
                inspectorView->detailText.contains(
                    QStringLiteral("正式输入与依赖")) &&
                inspectorView->detailText.contains(
                    QStringLiteral("[formal.upstream_stale]")),
            "formal operation inspector must project targets, boundary mode, parent IPW and structured stale reasons")) {
        return 1;
    }

    oap::app::CommandHistory history;
    history.pushVisibility({"F1"});
    history.pushMachiningJob(
        oap::manufacturing::MachiningJob{},
        QStringLiteral("job edit"));
    const auto last = history.takeLast();
    const auto previous = history.takeLast();
    if (!expect(
            last && last->kind ==
                        oap::app::UndoRecord::Kind::MachiningJob &&
                last->machiningJob.has_value() && previous &&
                previous->kind == oap::app::UndoRecord::Kind::Visibility,
            "command history must preserve Job and visibility time order")) {
        return 1;
    }

    std::cout << "PASS: app workflow controller checks\n";
    return 0;
}

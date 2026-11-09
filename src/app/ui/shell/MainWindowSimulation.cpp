// MainWindow 毛坯与 IPW 仿真命令编排。
#include "MainWindow.h"
#include "app/ui/panels/CamWorkflowPanel.h"

#include "app/presentation/IpwVisualAcceptanceCapture.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/workflows/hole/HoleStockGeometryBuilder.h"
#include "app/workflows/hole/HoleStockSimulationRunner.h"
#include "app/ui/dialogs/PocketCamSimulatorDialog.h"
#include "app/dialogs/QtWorkflowInteraction.h"
#include "app/dialogs/StockSimulationDialogs.h"
#include "app/ui/panels/FeatureAllowancePanel.h"
#include "app/workflows/simulation/StockSimulationUseCases.h"
#include "app/workflows/simulation/StockSimulationWorkflowController.h"

#include <utility>
#include <vector>
#include <map>

#include <QAction>
#include <QDebug>
#include <QDockWidget>
#include <QFileInfo>
#include <QInputDialog>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QTimer>

#include <TopoDS_Solid.hxx>

namespace {
constexpr int kIpwVisualAcceptanceImageSize = 1024;
} // namespace

bool MainWindow::ensureHoleProgram()
{
    if (m_machiningWorkspace->activateInstalledHoleProgram()) {
        return true;
    }
    if (m_actHolePaths) {
        QSignalBlocker block(m_actHolePaths);
        m_actHolePaths->setChecked(true);
    }
    showHolePaths();
    return m_machiningWorkspace->hasHoleProgram() &&
           !m_machiningWorkspace->holeToolpathNeedsRegeneration();
}

bool MainWindow::buildHoleInitialStock(double marginX, double marginY,
                                       double marginZ, bool perUnit,
                                       TopoDS_Shape& stock,
                                       int& stockBlockCount,
                                       QString& error) const
{
    std::vector<oap::app::HoleStockUnitGeometry> units;
    units.reserve(model().units.size());
    for (const oap::Unit& unit : model().units) {
        const TopoDS_Shape* sourceShape =
            stepIndexer().shapeForUnit(unit.unit_id);
        units.push_back({
            unit.unit_id,
            unit.bbox,
            sourceShape ? *sourceShape : TopoDS_Shape{}});
    }
    oap::app::HoleStockGeometryBuildResult built =
        oap::app::HoleStockGeometryBuilder::buildInitialStock(
            units, marginX, marginY, marginZ, perUnit);
    for (const std::string& warning : built.warnings) {
        qWarning().noquote()
            << "[OpenAutoPath]"
            << QString::fromStdString(warning);
    }
    stock = built.shape;
    stockBlockCount = built.solidCount;
    error = QString::fromStdString(built.error);
    return built.ok();
}

bool MainWindow::buildHoleTargetShape(TopoDS_Shape& target,
                                      int& targetSolidCount,
                                      QString& error) const
{
    std::vector<TopoDS_Solid> selectedSolids;
    std::vector<TopoDS_Solid> fallbackSolids;
    const auto appendSolidForFace = [&](const std::string& faceId) {
        const TopoDS_Solid* solid =
            stepIndexer().solidForFaceId(faceId);
        if (solid && !solid->IsNull())
            selectedSolids.push_back(*solid);
    };
    for (const HolePreviewEntry& entry : m_machiningWorkspace->holePreviewEntries()) {
        const RecognizedHoleSource* source =
            m_machiningWorkspace->recognizedHoleSource(entry.sourceIndex);
        if (!source)
            continue;
        for (const std::string& faceId :
             source->sourceFaceIds)
            appendSolidForFace(faceId);
    }

    for (const oap::Unit& unit : model().units) {
        const TopoDS_Solid* solid =
            stepIndexer().solidForUnit(unit.unit_id);
        if (solid && !solid->IsNull())
            fallbackSolids.push_back(*solid);
    }
    oap::app::HoleStockGeometryBuildResult built =
        oap::app::HoleStockGeometryBuilder::buildTargetShape(
            selectedSolids, fallbackSolids);
    target = built.shape;
    targetSolidCount = built.solidCount;
    error = QString::fromStdString(built.error);
    return built.ok();
}

void MainWindow::refreshPocketIpwDisplayMode()
{
    m_machiningWorkspace->refreshPocketSimulationDisplay(
        *m_stockSimulationWorkspace, sceneController().modelTransform());
}

bool MainWindow::showPocketIpwTimeline(bool allowOpenGl)
{
    const std::vector<oap::app::IpwTimelineOperation> operations =
        m_machiningWorkspace->pocketSimulationTimeline(
            *m_stockSimulationWorkspace);
    if (operations.empty())
        return false;

    const bool continueToOpenGl =
        oap::app::showIpwTimelineDialog(
            this, operations,
            [this](int operationIndex, int completedMoveCount) {
                m_machiningWorkspace->displayPocketSimulationTransient(
                    *m_stockSimulationWorkspace,
                    operationIndex,
                    completedMoveCount,
                    sceneController().modelTransform());
            },
            [this](const QString& formalOperationId) {
                synchronizeFormalOperationSelection(formalOperationId);
            }, allowOpenGl);
    m_machiningWorkspace->finishPocketSimulationTimeline(
        *m_stockSimulationWorkspace, sceneController().modelTransform());
    return continueToOpenGl;
}

void MainWindow::simulateHoleStock()
{
    if (m_uiBusy)
        return;
    if (!ensureHoleProgram() ||
        !m_machiningWorkspace->hasHolePreviewParams()) {
        QMessageBox::information(
            this, QStringLiteral("孔毛胚仿真"),
            QStringLiteral("未能生成可仿真的有效孔加工程序。"));
        return;
    }

    const oap::app::StockSourceSettings initial =
        m_stockSimulationWorkspace->holeSourceSettings();

    oap::app::HoleStockDialogCallbacks callbacks;
    callbacks.model = [this] {
        return m_stockSimulationWorkspace->holeDialogModel(
            m_machiningWorkspace->holePreviewEntries());
    };
    callbacks.describe =
        [this](const oap::app::HoleStockDisplaySelection& selection) {
            return m_stockSimulationWorkspace->holeSummary(
                m_machiningWorkspace->holePreviewEntries(), selection);
        };
    callbacks.present =
        [this](const oap::app::HoleStockDisplaySelection& selection) {
            if (selection.viewMode == 0) {
                m_stockSimulationWorkspace->displayHoleSnapshot(
                    selection.stepIndex, sceneController().modelTransform());
            } else if (selection.viewMode == 4) {
                m_stockSimulationWorkspace->displayHoleOperationOvercut(
                    selection.overcutAttributionIndex,
                    sceneController().modelTransform());
            } else {
                m_stockSimulationWorkspace->displayHoleComparison(
                    selection.viewMode, sceneController().modelTransform());
            }
        };
    callbacks.clear = [this] {
        m_stockSimulationWorkspace->clearHole();
    };
    callbacks.openGl = [this] { showHoleOpenGlSimulator(); };
    callbacks.publishStatus = [this](const QString& status) {
        statusBar()->showMessage(status);
    };
    callbacks.run =
        [this](const oap::app::StockSourceSettings& settings) {
            oap::app::HoleStockSimulationUseCaseRequest request;
            request.interaction =
                makeWorkflowInteraction();
            request.settings = settings;
            request.buildInitialStock =
                [this](const oap::app::StockSourceSettings& source,
                       TopoDS_Shape& stock,
                       int& blockCount,
                       QString& error) {
                    return buildHoleInitialStock(
                        source.marginX, source.marginY, source.marginZ,
                        source.sourceMode == 0,
                        stock, blockCount, error);
                };
            request.buildTarget =
                [this](TopoDS_Shape& target,
                       int& solidCount,
                       QString& error) {
                    return buildHoleTargetShape(
                        target, solidCount, error);
                };
            request.beforeRun =
                [this] { m_stockSimulationWorkspace->clearHole(); };
            request.closeRequested =
                [this] { return m_closeWhenIdle; };

            oap::app::HoleStockRunOutcome outcome =
                m_machiningWorkspace->executeHoleStockSimulation(
                    *m_stockSimulationWorkspace, request);
            refreshFeatureAllowanceView();
            if (outcome.closeRequested)
                QTimer::singleShot(0, this, &QWidget::close);
            return outcome;
        };

    oap::app::HoleStockSimulationDialog dialog(
        this, std::move(initial),
        QFileInfo(currentPath()).absolutePath(),
        std::move(callbacks));
    dialog.exec();
}

void MainWindow::showHoleOpenGlSimulator()
{
    const QString title = QStringLiteral("孔 OpenGL 仿真");
    const oap::toolpath::HoleToolpathBatch* holeResult =
        m_machiningWorkspace->holeProgram();
    if (!holeResult || holeResult->machining.operations.empty()) {
        QMessageBox::information(
            this, title, QStringLiteral("当前没有可播放的孔加工程序。"));
        return;
    }

    std::map<int, oap::MoldSetupFace> setupFaces;
    for (const oap::toolpath::HoleMachiningOperation& operation :
         holeResult->machining.operations) {
        setupFaces.try_emplace(operation.setupIndex, operation.setupFace);
    }
    if (setupFaces.empty()) {
        QMessageBox::information(
            this, title, QStringLiteral("孔程序没有有效的装夹面。"));
        return;
    }

    std::vector<int> setupIndices;
    QStringList setupLabels;
    setupIndices.reserve(setupFaces.size());
    for (const auto& [setupIndex, setupFace] : setupFaces) {
        setupIndices.push_back(setupIndex);
        setupLabels.push_back(
            QStringLiteral("Setup %1 · %2")
                .arg(setupIndex + 1)
                .arg(QString::fromLatin1(
                    oap::moldSetupFaceName(setupFace))));
    }
    int selectedChoice = 0;
    if (setupLabels.size() > 1) {
        bool accepted = false;
        const QString selected = QInputDialog::getItem(
            this, title, QStringLiteral("选择要模拟的固定轴装夹面"),
            setupLabels, 0, false, &accepted);
        if (!accepted)
            return;
        selectedChoice = setupLabels.indexOf(selected);
        if (selectedChoice < 0)
            return;
    }

    TopoDS_Shape targetShape;
    int targetSolidCount = 0;
    QString error;
    if (!buildHoleTargetShape(targetShape, targetSolidCount, error)) {
        QMessageBox::warning(this, title, error);
        return;
    }

    oap::app::HoleOpenGlSimulationRequest request;
    if (!m_stockSimulationWorkspace->buildHoleOpenGlRequest(
            holeResult->machining.operations,
            holeResult->effectiveParams,
            setupIndices[static_cast<std::size_t>(selectedChoice)],
            targetShape, request, error)) {
        QMessageBox::information(this, title, error);
        return;
    }

    oap::app::HoleOpenGlSimulationResult prepared =
        oap::app::prepareHoleOpenGlWithProgress(
            makeWorkflowInteraction().showProgress,
            std::move(request));
    if (prepared.cancelled) {
        statusBar()->showMessage(QStringLiteral("孔 OpenGL 仿真已取消"));
        return;
    }
    if (!prepared.ok()) {
        error = prepared.error.empty()
            ? QStringLiteral("无法构建孔 OpenGL 仿真输入。")
            : QString::fromStdString(prepared.error);
        QMessageBox::warning(this, title, error);
        return;
    }
    if (!prepared.warning.empty()) {
        statusBar()->showMessage(
            QString::fromStdString(prepared.warning));
    }

    prepared.input.interaction.rotationMode =
        m_view->rotationMode() == OcctView::RotationMode::Model
        ? oap::app::PocketCamRotationMode::Model
        : oap::app::PocketCamRotationMode::Camera;
    prepared.input.interaction.modelRotationVerticalInverted =
        m_view->modelRotationVerticalInverted();
    prepared.input.interaction.rotateButton =
        m_view->mouseButton(OcctView::MouseAction::Rotate);
    prepared.input.interaction.panButton =
        m_view->mouseButton(OcctView::MouseAction::Pan);

    statusBar()->showMessage(
        QStringLiteral(
            "正在启动孔 OpenGL 仿真：%1 道工序，网格 %2 mm，%3 个目标实体")
            .arg(prepared.input.operations.size())
            .arg(prepared.gridResolution, 0, 'f', 3)
            .arg(targetSolidCount));
    if (!oap::app::showPocketCamSimulatorDialog(
            this, std::move(prepared.input), &error)) {
        QMessageBox::warning(this, title, error);
        statusBar()->showMessage(QStringLiteral("孔 OpenGL 仿真启动失败"));
        return;
    }
    statusBar()->showMessage(QStringLiteral("孔 OpenGL 仿真已关闭"));
}

void MainWindow::simulatePocketStock()
{
    // 腔毛坯仿真主入口:先保证存在与当前参数一致的腔清腔预览,弹出设置
    // 对话框(毛坯来源/余量/显示质量)并同步到加工项目,再执行统一 IPW
    // 逐工序减料;结束后可选进入 FreeCAD CAM OpenGL 交互预览。
    if (m_uiBusy)
        return;
    if (model().faces.empty() ||
        !m_recognitionWorkspace->hasResult()) {
        QMessageBox::information(
            this, QStringLiteral("腔毛胚仿真"),
            QStringLiteral(
                "请先打开 STEP 文件并执行内外环结构识别。"));
        return;
    }
    if (!m_machiningWorkspace->hasPocketPreviewParams() ||
        !m_machiningWorkspace->currentPocketResult() ||
        !oap::app::MachiningConfigurationSession::parametersEqual(
            m_machiningWorkspace->pocketPreviewParams(),
            m_machiningWorkspace->pocketParams())) {
        showPocketClearing();
    }
    if (!m_machiningWorkspace->hasPocketPreviewParams() ||
        !m_machiningWorkspace->currentPocketResult()) {
        QMessageBox::information(
            this, QStringLiteral("腔毛胚仿真"),
            QStringLiteral(
                "请先执行“显示腔清腔”，生成并验证当前腔刀路。"));
        return;
    }

    const oap::app::PocketStockSimulationDialogState initial =
        m_stockSimulationWorkspace->pocketDialogState(
            m_machiningWorkspace->currentPocketResult()->operations,
            QFileInfo(currentPath()).absolutePath());
    const auto edited =
        oap::app::editPocketStockSimulationSettings(this, initial);
    if (!edited)
        return;
    const oap::app::PocketStockSimulationDialogState settings = *edited;

    TopoDS_Shape targetShape;
    int targetSolidCount = 0;
    QString error;
    if (!buildHoleTargetShape(
            targetShape, targetSolidCount, error)) {
        QMessageBox::warning(
            this, QStringLiteral("腔毛胚仿真"), error);
        return;
    }

    TopoDS_Shape generatedInitialStock;
    int stockBlockCount = 0;
    if (settings.source.sourceMode != 2 &&
        !buildHoleInitialStock(
            settings.source.marginX,
            settings.source.marginY,
            settings.source.marginZ,
            settings.source.sourceMode == 0,
            generatedInitialStock, stockBlockCount, error)) {
        QMessageBox::warning(
            this, QStringLiteral("腔毛胚仿真"), error);
        return;
    }

    QString jobCaptureError;
    if (!synchronizePocketJobFromResult(jobCaptureError)) {
        QMessageBox::warning(
            this, QStringLiteral("统一 IPW 腔加工仿真"),
            QStringLiteral(
                "当前腔刀路无法捕获到加工项目：\n%1")
                .arg(jobCaptureError));
        return;
    }

    oap::app::PocketStockSimulationUseCaseRequest request;
    request.interaction = makeWorkflowInteraction();
    request.settings = settings;
    request.generatedInitialStock = generatedInitialStock;
    request.targetShape = targetShape;
    request.exactTargetBounds = stepIndexer().exactBoundsForShape(targetShape, model());
    request.beforeRun = [this] {
        m_stockSimulationWorkspace->clearPocket();
    };
    request.closeRequested = [this] { return m_closeWhenIdle; };

    oap::app::PocketStockSimulationUseCaseResult simulated =
        m_machiningWorkspace->executePocketStockSimulation(
            *m_stockSimulationWorkspace, request);
    refreshFeatureAllowanceView();
    m_stockSimulationWorkspace->setDisplayQuality(
        settings.displayQuality);
    m_stockSimulationWorkspace->setTriangulationEnabled(
        settings.triangulationEnabled);

    using Failure = oap::app::PocketStockSimulationFailure;
    if (simulated.failure == Failure::CloseRequested) {
        QTimer::singleShot(0, this, &QWidget::close);
        return;
    }
    if (simulated.failure == Failure::Cancelled) {
        statusBar()->showMessage(simulated.error);
        return;
    }
    if (!simulated.ok()) {
        QMessageBox::warning(
            this, QStringLiteral("统一 IPW 腔加工仿真"),
            simulated.error);
        statusBar()->showMessage(simulated.error);
        return;
    }
    if (!simulated.warning.isEmpty()) {
        QMessageBox::warning(
            this,
            QStringLiteral("统一 IPW 派生审计未完成"),
            simulated.warning);
    }

    const QString ipwSummary = simulated.summary;
    statusBar()->showMessage(ipwSummary);
    const bool ipwOcctDisplayEnabled =
        m_stockSimulationWorkspace->occtDisplayEnabled();
    if (ipwOcctDisplayEnabled) {
        if (!showPocketIpwTimeline()) {
            statusBar()->showMessage(
                QStringLiteral(
                    "已保留统一 IPW 最终检查点；%1")
                    .arg(ipwSummary));
            return;
        }
    } else {
        m_stockSimulationWorkspace->clearPocket(false);
        statusBar()->showMessage(
            m_stockSimulationWorkspace->displayMode() ==
                    oap::app::IpwOcctDisplayMode::Off
            ? QStringLiteral(
                  "Multi-Dexel 主视口显示已关闭；"
                  "仅保留材料检查点和审计结果。%1")
                  .arg(ipwSummary)
            : QStringLiteral(
                  "当前表面形式需要三角化；"
                  "仅保留材料检查点和审计结果。%1")
                  .arg(ipwSummary));
    }

    oap::app::PocketCamSimulatorInput input =
        std::move(simulated.camInput);
    input.interaction.rotationMode =
        m_view->rotationMode() == OcctView::RotationMode::Model
        ? oap::app::PocketCamRotationMode::Model
        : oap::app::PocketCamRotationMode::Camera;
    input.interaction.modelRotationVerticalInverted =
        m_view->modelRotationVerticalInverted();
    input.interaction.rotateButton =
        m_view->mouseButton(OcctView::MouseAction::Rotate);
    input.interaction.panButton =
        m_view->mouseButton(OcctView::MouseAction::Pan);

    if (!input.visualAcceptanceDirectory.isEmpty()) {
        oap::app::presentation::IpwVisualAcceptanceCaptureRequest capture;
        capture.directory = input.visualAcceptanceDirectory;
        capture.view = m_view->view();
        capture.context = m_view->context();
        capture.ipwPresentation =
            m_stockSimulationWorkspace->pocketPresentation();
        capture.camera.valid = input.visualAcceptanceView.valid;
        capture.camera.center = input.visualAcceptanceView.center;
        capture.camera.eyeDirection =
            input.visualAcceptanceView.eyeDirection;
        capture.camera.up = input.visualAcceptanceView.up;
        capture.camera.halfHeight =
            input.visualAcceptanceView.halfHeight;
        capture.imageSize = kIpwVisualAcceptanceImageSize;
        const auto captured =
            oap::app::presentation::IpwVisualAcceptanceCapture::
                capture(capture);
        input.visualAcceptanceDirectory = captured.directory;
        if (!captured.warnings.isEmpty())
            statusBar()->showMessage(captured.warnings.back());
    }
    statusBar()->showMessage(
        QStringLiteral(
            "正在启动 FreeCAD CAM OpenGL 仿真：%1 道工序，"
            "%2 个目标实体")
            .arg(input.operations.size())
            .arg(targetSolidCount));
    if (!oap::app::showPocketCamSimulatorDialog(
            this, std::move(input), &error)) {
        QMessageBox::warning(
            this, QStringLiteral("腔毛胚仿真"), error);
        statusBar()->showMessage(
            QStringLiteral("FreeCAD CAM 仿真启动失败"));
        return;
    }
    statusBar()->showMessage(
        QStringLiteral(
            "FreeCAD CAM 交互预览已关闭；%1")
            .arg(ipwSummary));
}

void MainWindow::simulateProfileStock()
{
    const QString title = QStringLiteral("外轮廓 IPW 仿真");
    if (m_uiBusy)
        return;
    if (model().faces.empty() ||
        !m_recognitionWorkspace->hasResult()) {
        QMessageBox::information(
            this, title,
            QStringLiteral("请先打开 STEP 文件并执行 CAM 特征识别。"));
        return;
    }
    if (!m_machiningWorkspace->profilePreviewMatchesCurrentParameters() ||
        !m_machiningWorkspace->hasProfilePreview()) {
        showProfileContour();
    }
    if (!m_machiningWorkspace->hasProfilePreviewParams() ||
        !m_machiningWorkspace->hasProfilePreview()) {
        QMessageBox::information(
            this, title,
            QStringLiteral(
                "请先执行“生成 / 更新外轮廓刀路”，生成并通过安全校验。"));
        return;
    }

    const auto initial =
        m_stockSimulationWorkspace->profileDialogState(
            m_machiningWorkspace->profilePreviewOperations(),
            QFileInfo(currentPath()).absolutePath());
    const auto edited = oap::app::editPocketStockSimulationSettings(
        this, initial, QStringLiteral("外轮廓"), false);
    if (!edited)
        return;
    const oap::app::PocketStockSimulationDialogState settings = *edited;

    TopoDS_Shape targetShape;
    int targetSolidCount = 0;
    QString error;
    if (!buildHoleTargetShape(
            targetShape, targetSolidCount, error)) {
        QMessageBox::warning(this, title, error);
        return;
    }

    TopoDS_Shape generatedInitialStock;
    int stockBlockCount = 0;
    if (settings.source.sourceMode != 2 &&
        !buildHoleInitialStock(
            settings.source.marginX,
            settings.source.marginY,
            settings.source.marginZ,
            settings.source.sourceMode == 0,
            generatedInitialStock, stockBlockCount, error)) {
        QMessageBox::warning(this, title, error);
        return;
    }

    oap::app::MachiningProjectProfileCaptureRequest capture;
    capture.definition = machiningProjectDefinitionContext();
    capture.rootSources =
        &m_recognitionWorkspace->profileSources();
    capture.setupFrames = &moldSetupWorkFrames();
    QString captureError;
    if (!m_machiningWorkspace->captureProfileProjectPreview(
            capture, &captureError)) {
        QMessageBox::warning(
            this, title,
            QStringLiteral(
                "当前外轮廓刀路无法捕获到加工项目：\n%1")
                .arg(captureError));
        return;
    }

    oap::app::ProfileStockSimulationUseCaseRequest request;
    request.interaction = makeWorkflowInteraction();
    request.settings = settings;
    request.generatedInitialStock = generatedInitialStock;
    request.targetShape = targetShape;
    request.exactTargetBounds = stepIndexer().exactBoundsForShape(targetShape, model());
    request.beforeRun = [this] {
        m_stockSimulationWorkspace->clearPocket();
    };
    request.closeRequested = [this] { return m_closeWhenIdle; };

    oap::app::ProfileStockSimulationUseCaseResult simulated =
        m_machiningWorkspace->executeProfileStockSimulation(
            *m_stockSimulationWorkspace, request);
    refreshFeatureAllowanceView();
    m_stockSimulationWorkspace->setDisplayQuality(
        settings.displayQuality);
    m_stockSimulationWorkspace->setTriangulationEnabled(
        settings.triangulationEnabled);

    using Failure = oap::app::PocketStockSimulationFailure;
    if (simulated.failure == Failure::CloseRequested) {
        QTimer::singleShot(0, this, &QWidget::close);
        return;
    }
    if (simulated.failure == Failure::Cancelled) {
        statusBar()->showMessage(simulated.error);
        return;
    }
    if (!simulated.ok()) {
        QMessageBox::warning(this, title, simulated.error);
        statusBar()->showMessage(simulated.error);
        return;
    }
    if (!simulated.warning.isEmpty()) {
        QMessageBox::warning(
            this, QStringLiteral("外轮廓 IPW 派生审计未完成"),
            simulated.warning);
    }

    const QString summary = simulated.summary;
    statusBar()->showMessage(summary);
    if (m_stockSimulationWorkspace->occtDisplayEnabled()) {
        // Profile 逐移动重放通过适配后的真实 moves/primitives 驱动；
        // 时间线关闭后自动回到最终材料检查点。
        if (!showPocketIpwTimeline()) {
            statusBar()->showMessage(
                QStringLiteral(
                    "已保留外轮廓 IPW 最终检查点；%1")
                    .arg(summary));
            return;
        }
    } else {
        m_stockSimulationWorkspace->clearPocket(false);
    }

    oap::app::PocketCamSimulatorInput input =
        std::move(simulated.camInput);
    input.interaction.rotationMode =
        m_view->rotationMode() == OcctView::RotationMode::Model
        ? oap::app::PocketCamRotationMode::Model
        : oap::app::PocketCamRotationMode::Camera;
    input.interaction.modelRotationVerticalInverted =
        m_view->modelRotationVerticalInverted();
    input.interaction.rotateButton =
        m_view->mouseButton(OcctView::MouseAction::Rotate);
    input.interaction.panButton =
        m_view->mouseButton(OcctView::MouseAction::Pan);

    if (!input.visualAcceptanceDirectory.isEmpty()) {
        oap::app::presentation::IpwVisualAcceptanceCaptureRequest
            visualCapture;
        visualCapture.directory = input.visualAcceptanceDirectory;
        visualCapture.view = m_view->view();
        visualCapture.context = m_view->context();
        visualCapture.ipwPresentation =
            m_stockSimulationWorkspace->pocketPresentation();
        visualCapture.camera.valid = input.visualAcceptanceView.valid;
        visualCapture.camera.center = input.visualAcceptanceView.center;
        visualCapture.camera.eyeDirection =
            input.visualAcceptanceView.eyeDirection;
        visualCapture.camera.up = input.visualAcceptanceView.up;
        visualCapture.camera.halfHeight =
            input.visualAcceptanceView.halfHeight;
        visualCapture.imageSize = kIpwVisualAcceptanceImageSize;
        const auto captured =
            oap::app::presentation::IpwVisualAcceptanceCapture::capture(
                visualCapture);
        input.visualAcceptanceDirectory = captured.directory;
        if (!captured.warnings.isEmpty())
            statusBar()->showMessage(captured.warnings.back());
    }
    statusBar()->showMessage(
        QStringLiteral(
            "正在启动外轮廓 IPW 交互仿真：%1 道工序，%2 个目标实体")
            .arg(input.operations.size())
            .arg(targetSolidCount));
    if (!oap::app::showPocketCamSimulatorDialog(
            this, std::move(input), &error)) {
        QMessageBox::warning(this, title, error);
        statusBar()->showMessage(
            QStringLiteral("外轮廓 IPW 交互仿真启动失败"));
        return;
    }
    statusBar()->showMessage(
        QStringLiteral("外轮廓 IPW 交互预览已关闭；%1")
            .arg(summary));
}

void MainWindow::showFeatureAllowanceView()
{
    if (m_uiBusy)
        return;
    refreshFeatureAllowanceView();
    if (m_camWorkflowDock && m_camWorkflowPanel) {
        m_camWorkflowDock->show();
        m_camWorkflowDock->raise();
        m_camWorkflowPanel->focusStage(
            oap::app::CamWorkflowStage::Verification);
    }
    if (m_featureAllowancePanel)
        m_featureAllowancePanel->activateCurrent();
}

void MainWindow::refreshFeatureAllowanceView()
{
    if (!m_featureAllowancePanel || !m_stockSimulationWorkspace)
        return;
    m_featureAllowancePanel->setItems(
        m_stockSimulationWorkspace->featureAllowanceItems());
    refreshCamWorkflowPanel();
}

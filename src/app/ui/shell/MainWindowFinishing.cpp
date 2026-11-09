// 雕刻加工沿用统一参数、生成、工序树、主视图播放与项目 IPW 流程。
#include "app/ui/shell/MainWindow.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/dialogs/QtWorkflowInteraction.h"
#include "app/dialogs/StockSimulationDialogs.h"
#include "app/ui/dialogs/PocketCamSimulatorDialog.h"
#include "app/workflows/engraving/EngravingToolpathGenerationUseCase.h"
#include "app/workflows/simulation/StockSimulationUseCases.h"

#include <QAction>
#include <QFileInfo>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QTimer>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <algorithm>

void MainWindow::generateEngravingToolpaths()
{
    if (m_uiBusy)
        return;
    const QString title = QStringLiteral("雕刻加工");
    if (model().units.empty()) {
        QMessageBox::information(this, title, QStringLiteral("请先加载雕刻件 STEP 模型。无需先识别孔或槽腔。"));
        return;
    }
    oap::app::EngravingToolpathGenerationRequest request;
    request.targets.reserve(model().units.size());
    request.unitIds.reserve(model().units.size());
    for (const auto& unit : model().units) {
        const auto* shape = stepIndexer().shapeForUnit(unit.unit_id);
        if (!shape || shape->IsNull()) {
            QMessageBox::warning(this, title, QStringLiteral("部分实体缺少完整几何，请重新加载模型后生成雕刻刀路。"));
            return;
        }
        request.targets.push_back(*shape);
        request.unitIds.push_back(unit.unit_id);
    }
    request.definition = machiningProjectDefinitionContext();
    request.setupFrames = &moldSetupWorkFrames();
    request.interaction = makeWorkflowInteraction();
    const auto generated = m_machiningWorkspace->generateEngraving(std::move(request));
    if (m_closeWhenIdle) {
        QTimer::singleShot(0, this, &QWidget::close);
        return;
    }
    if (!generated.ok) {
        statusBar()->showMessage(generated.error);
        if (!generated.cancelled)
            QMessageBox::warning(this, title, generated.error);
        return;
    }
    m_stockSimulationWorkspace->clearHole();
    m_stockSimulationWorkspace->clearPocket();
    if (m_actHolePaths) {
        const QSignalBlocker blocker(m_actHolePaths);
        m_actHolePaths->setChecked(false);
    }
    m_playbackWorkspace->replaceProgramPreview(generated.program, sceneController().modelTransform());
    rebuildPlaybackMovesFromProgram(generated.program, false);
    refreshCamWorkflowPanel();
    if (m_view && !m_view->view().IsNull())
        m_view->view()->Redraw();
    statusBar()->showMessage(generated.summary);
}

void MainWindow::simulateProjectStock()
{
    simulateOrderedStock(false);
}

void MainWindow::simulateEngravingStock()
{
    simulateOrderedStock(true);
}

void MainWindow::simulateOrderedStock(bool engravingOnly)
{
    const QString title = engravingOnly ? QStringLiteral("雕刻 IPW") : QStringLiteral("加工项目 IPW");
    if (m_uiBusy)
        return;
    if (!m_machiningWorkspace->hasProjectContent() || model().units.empty()) {
        QMessageBox::information(this, title, QStringLiteral("请先生成并应用正式加工工序。"));
        return;
    }
    auto initial = m_stockSimulationWorkspace->profileDialogState(
        {}, QFileInfo(currentPath()).absolutePath());
    initial.setups = {{-1, QStringLiteral("全部启用工序（按项目全局顺序）")}};
    if (engravingOnly) {
        initial.scopeDescription = QStringLiteral("按全局顺序计算至所选方向的最后一道雕刻工序，保留前序加工的剩余毛坯；可手动打开仅播放所选雕刻刀路的动画。");
        const auto program = m_machiningWorkspace->finishingPreviewProgram();
        if (program.empty()) {
            QMessageBox::information(this, title, QStringLiteral("请先生成有效的雕刻刀路。"));
            return;
        }
        initial.setups = {{-1, QStringLiteral("全部雕刻方向（含前序材料准备）")}};
        for (const auto& operation : program.operations) {
            if (std::none_of(initial.setups.begin(), initial.setups.end(),
                    [&](const auto& choice) { return choice.setupIndex == operation.setupIndex; }))
                initial.setups.push_back({operation.setupIndex,
                    QStringLiteral("雕刻 %1（含前序材料准备）").arg(QString::fromLatin1(oap::moldSetupFaceName(operation.setupFace)))});
        }
    }
    initial.selectedSetup = -1;
    const auto edited = oap::app::editPocketStockSimulationSettings(this, initial,
        engravingOnly ? QStringLiteral("雕刻") : QStringLiteral("完整加工项目"), false);
    if (!edited)
        return;
    const auto settings = *edited;
    BRep_Builder builder;
    TopoDS_Compound target;
    builder.MakeCompound(target);
    for (const auto& unit : model().units) {
        const auto* shape = stepIndexer().shapeForUnit(unit.unit_id);
        if (!shape || shape->IsNull()) {
            QMessageBox::warning(this, title, QStringLiteral("部分完整模型几何缺失，不能执行项目 IPW。"));
            return;
        }
        builder.Add(target, *shape);
    }
    TopoDS_Shape initialStock;
    int stockCount = 0;
    QString error;
    if (settings.source.sourceMode != 2 &&
        !buildHoleInitialStock(settings.source.marginX, settings.source.marginY,
            settings.source.marginZ, settings.source.sourceMode == 0, initialStock, stockCount, error)) {
        QMessageBox::warning(this, title, error);
        return;
    }
    oap::app::ProjectStockSimulationUseCaseRequest request;
    request.engravingOnly = engravingOnly;
    request.interaction = makeWorkflowInteraction();
    request.settings = settings;
    request.generatedInitialStock = initialStock;
    request.targetShape = target;
    request.exactTargetBounds = stepIndexer().exactBoundsForShape(target, model());
    request.beforeRun = [this] { m_stockSimulationWorkspace->clearPocket(); };
    request.closeRequested = [this] { return m_closeWhenIdle; };
    auto simulated = m_machiningWorkspace->executeProjectStockSimulation(*m_stockSimulationWorkspace, request);
    refreshFeatureAllowanceView();
    m_stockSimulationWorkspace->setDisplayQuality(settings.displayQuality);
    m_stockSimulationWorkspace->setTriangulationEnabled(settings.triangulationEnabled);
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
        return;
    }
    if (!simulated.warning.isEmpty())
        QMessageBox::warning(this, title, simulated.warning);
    statusBar()->showMessage(simulated.summary);
    refreshCamWorkflowPanel();
    // Use the same checkpoint/timeline review as Pocket and Profile. Only an
    // explicit animation request may leave this review for the OpenGL dialog.
    // Disabled OCCT presentation must not silently opt the user into playback.
    const bool allowAnimation = simulated.playbackError.isEmpty();
    if (!allowAnimation)
        statusBar()->showMessage(simulated.summary + QStringLiteral(" 动画不可用：") + simulated.playbackError);
    if (!showPocketIpwTimeline(allowAnimation)) {
        statusBar()->showMessage(QStringLiteral("已保留 IPW 最终检查点；%1").arg(simulated.summary));
        return;
    }
    auto input = std::move(simulated.camInput);
    input.interaction.rotationMode =
        m_view->rotationMode() == OcctView::RotationMode::Model
        ? oap::app::PocketCamRotationMode::Model : oap::app::PocketCamRotationMode::Camera;
    input.interaction.modelRotationVerticalInverted = m_view->modelRotationVerticalInverted();
    input.interaction.rotateButton = m_view->mouseButton(OcctView::MouseAction::Rotate);
    input.interaction.panButton = m_view->mouseButton(OcctView::MouseAction::Pan);
    if (!oap::app::showPocketCamSimulatorDialog(this, std::move(input), &error))
        QMessageBox::warning(this, title, error);
    statusBar()->showMessage(simulated.summary);
}

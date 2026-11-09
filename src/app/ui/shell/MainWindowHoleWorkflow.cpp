// MainWindow 自动孔加工、孔树与覆盖审计命令。
#include "MainWindow.h"

#include "app/orchestration/DesktopWorkspaceCoordinator.h"
#include "app/ui/panels/CamWorkflowPanel.h"

#include "app/panels/ResultPanel.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/workflows/hole/HoleToolpathRunner.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/dialogs/QtWorkflowInteraction.h"

#include <algorithm>
#include <utility>

#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>

#include <QDialog>
#include <QDockWidget>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>

namespace {
const char* featureKindZh(oap::FeatureKind kind)
{
    switch (kind) {
        case oap::FeatureKind::ThroughHole:
            return "通孔";
        case oap::FeatureKind::BlindHole:
            return "盲孔";
        case oap::FeatureKind::CountersinkHole:
            return "沉孔";
        case oap::FeatureKind::Slot:
            return "槽";
        case oap::FeatureKind::Pocket:
            return "腔";
    }
    return "特征";
}

} // namespace

void MainWindow::clearHoleStageHighlights()
{
    if (m_machiningWorkspace->highlightedHoleStageSources().empty())
        return;
    const qsizetype clearedCount = static_cast<qsizetype>(
        m_machiningWorkspace->highlightedHoleStageSources().size());
    clearHoleStageHighlightDisplay(true);
    if (m_view && !m_view->context().IsNull()) {
        m_view->context()->ClearSelected(Standard_False);
        m_view->view()->Redraw();
    }
    statusBar()->showMessage(
        QStringLiteral("已清除全部 %1 个孔段高亮。").arg(clearedCount));
}

void MainWindow::generateOrUpdateHolePaths()
{
    m_machiningWorkspace->markHoleRegenerationRequired();
    if (m_actHolePaths) {
        const QSignalBlocker blocker(m_actHolePaths);
        m_actHolePaths->setChecked(true);
    }
    showHolePaths();
}

void MainWindow::rebuildHoleToolTree()
{
    refreshHoleCoverageAudit();
    if (!m_holeToolTreePanel)
        return;
    oap::app::HoleToolTreeModel model =
        m_machiningWorkspace->holeToolTreeModel(
            m_playbackWorkspace->moves(),
            m_playbackWorkspace->toolBatchStarts());
    m_holeToolTreePanel->setProgram(
        model.batches);
    refreshCamWorkflowPanel();
}
void MainWindow::refreshHoleCoverageAudit()
{
    if (!m_holeCoverageAuditPanel)
        return;

    // 审计失败可能没有生成任何 Operation，此时 holeResult() 按现有语义返回
    // nullptr；覆盖视图仍必须显示 runtime 中的失败报告。
    const oap::toolpath::HoleCoverageReport& coverage =
        m_machiningWorkspace->holeCoverageReport();
    if (!coverage.audited) {
        m_holeCoverageAuditPanel->clear();
        return;
    }

    m_holeCoverageAuditPanel->setReport(
        coverage,
        m_machiningWorkspace->holeCoverageOperationBindings(
            m_playbackWorkspace->moves()));
    const bool hasWarnings =
        !coverage.outsideSixFaceSourceHoleIndices.empty() ||
        !coverage.warningIssues.empty();
    if ((!coverage.complete() || hasWarnings) &&
        m_camWorkflowDock && m_camWorkflowPanel) {
        m_camWorkflowDock->show();
        m_camWorkflowDock->raise();
        m_camWorkflowPanel->focusStage(
            oap::app::CamWorkflowStage::Verification);
    }
}

void MainWindow::synchronizeFormalOperationSelection(
    const QString& formalOperationId)
{
    if (formalOperationId.isEmpty())
        return;
    if (m_holeToolTreePanel &&
        m_holeToolTreePanel->selectedFormalOperationId() !=
            formalOperationId) {
        m_holeToolTreePanel->selectFormalOperation(formalOperationId);
    }
    if (m_holeCoverageAuditPanel &&
        m_holeCoverageAuditPanel->selectedFormalOperationId() !=
            formalOperationId) {
        m_holeCoverageAuditPanel->selectFormalOperation(
            formalOperationId);
    }
}

void MainWindow::locateHoleCoverageSource(int sourceHoleIndex)
{
    const RecognizedHoleSource* source =
        m_machiningWorkspace->recognizedHoleSource(sourceHoleIndex);
    if (!source) {
        statusBar()->showMessage(
            QStringLiteral("孔段 %1 没有可用的识别源映射")
                .arg(sourceHoleIndex + 1));
        return;
    }

    const std::vector<std::string>& locationFaceIds =
        source->hole.stageWallFaceIds.empty()
        ? source->sourceFaceIds : source->hole.stageWallFaceIds;
    if (!locationFaceIds.empty()) {
        const QString firstFace = QString::fromStdString(
            locationFaceIds.front());
        // 同时把分析/特征树滚动到首个源面，便于继续查看几何属性；这些 API
        // 会屏蔽自身信号，不会抢走覆盖审计树的当前行。
        if (m_resultPanel)
            m_resultPanel->selectFaceById(firstFace);
        if (m_featurePanel && !m_featurePanel->isEmpty())
            m_featurePanel->selectFaceById(firstFace);
    }

    const QString setup =
        source->hole.setupFace == oap::MoldSetupFace::Unresolved
        ? QStringLiteral("未解析装夹")
        : QStringLiteral("%1 装夹面").arg(
              QString::fromLatin1(
                  oap::moldSetupFaceName(source->hole.setupFace)));
    const double diameter = 2.0 * source->hole.radius;
    const double depth = source->hole.entry.Distance(source->hole.bottom);
    const QString diameterText = std::isfinite(diameter) && diameter > 0.0
        ? QString::number(diameter, 'f', 3)
        : QStringLiteral("无效");
    const QString depthText = std::isfinite(depth)
        ? QString::number(depth, 'f', 3)
        : QStringLiteral("无效");
    QString locationStatus =
        QStringLiteral("已定位孔段 %1：%2 · Ø%3 mm · 深度 %4 mm · %5")
            .arg(sourceHoleIndex + 1)
            .arg(QString::fromUtf8(featureKindZh(source->hole.kind)))
            .arg(diameterText)
            .arg(depthText)
            .arg(setup);
    if (locationFaceIds.empty())
        locationStatus += QStringLiteral("（无源面映射，仅按孔轴定位）");

    highlightFaces(locationFaceIds, {});

    // 以孔段源面和孔轴共同建立局部包围盒。即使源面映射缺失，也能缩放到
    // entry-bottom 轴段；保留最小余量，避免极短孔造成过度放大。
    if (m_view && !m_view->view().IsNull()) {
        Bnd_Box bounds;
        const auto addFinitePoint = [&bounds](const gp_Pnt& point) {
            if (std::isfinite(point.X()) && std::isfinite(point.Y()) &&
                std::isfinite(point.Z())) {
                bounds.Update(point.X(), point.Y(), point.Z());
            }
        };
        addFinitePoint(source->hole.entry);
        addFinitePoint(source->hole.bottom);
        for (const std::string& faceId : locationFaceIds) {
            const TopoDS_Face* face =
                stepIndexer().faceForId(faceId);
            if (face && !face->IsNull())
                BRepBndLib::Add(*face, bounds);
        }
        if (!bounds.IsVoid()) {
            const double radiusMargin =
                std::isfinite(source->hole.radius) && source->hole.radius > 0.0
                ? source->hole.radius * 0.5 : 1.0;
            bounds.Enlarge(std::max(1.0, radiusMargin));
            try {
                m_view->view()->FitAll(bounds, 0.20, Standard_True);
            } catch (const Standard_Failure& error) {
                qWarning().noquote()
                    << "[OpenAutoPath] failed to fit audit hole:"
                    << error.GetMessageString();
            }
        }
    }

    // highlightFaces 会刷新通用选择统计；定位提示应当作为本次操作的最终状态。
    statusBar()->showMessage(locationStatus);
}

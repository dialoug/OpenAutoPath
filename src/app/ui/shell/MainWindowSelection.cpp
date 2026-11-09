// MainWindow 选择、可见性与检查适配器。
#include "MainWindow.h"
#include "app/ui/shell/MainWindowUiBuilder.h"
#include "app/ui/view/OcctView.h"
#include "app/panels/ResultPanel.h"
#include "app/panels/FeaturePanel.h"
#include "app/panels/GroupPanel.h"
#include "app/ui/panels/FileListPanel.h"
#include "app/ui/dialogs/ShortcutSettingsDialog.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/workflows/hole/HoleJobSourceSynchronizer.h"
#include "app/presentation/OverlayController.h"

#include "recognition/FeatureRecognizer.h"
#include "analysis/AabbBoundaryFace.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleMoldEnvelopeEntryResolver.h"
#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepPrimAPI_MakeBox.hxx>

#include <set>
#include <tuple>
#include <iterator>
#include <sstream>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <array>
#include <optional>
#include <unordered_set>

#include <QFileDialog>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSettings>
#include <QDir>
#include <QDateTime>
#include <QMenuBar>
#include <QMessageBox>
#include <QToolBar>
#include <QLabel>
#include <QSlider>
#include <QDockWidget>
#include <QListWidget>
#include <QStatusBar>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QElapsedTimer>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QTabWidget>
#include <QScreen>
#include <QDebug>
#include <QMenu>
#include <QAction>
#include <QActionGroup>
#include <QSignalBlocker>
#include <QApplication>
#include <QProgressDialog>
#include <QStringList>
#include <QMetaObject>
#include <QTimer>
#include <QHash>

#include <STEPCAFControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <QCloseEvent>
#include <XCAFPrs_AISObject.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Line.hxx>
#include <AIS_Point.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Graphic3d_DisplayPriority.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Quantity_Color.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_PointAspect.hxx>
#include <Standard_Failure.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <gp.hxx>
#include <gp_Vec.hxx>
#include <TDF_LabelSequence.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <OSD.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <Standard_Failure.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Poly_Triangle.hxx>
#include <V3d.hxx>

void MainWindow::onEntityPicked(const TopoDS_Shape &shape, int shapeType,
                                bool additive, bool wholeFeature)
{
    const oap::app::ScenePickResult picked = m_sceneInteraction.mapPick(
        shape, shapeType,
        m_view->selectionMode() == OcctView::SelMode::Unit,
        additive, wholeFeature,
        m_resultPanel ? m_resultPanel->selectedFaceIds()
                      : std::vector<std::string>{},
        stepIndexer(), modelIndex(),
        [this](const std::string& faceId) {
            return featureFacesContaining(faceId);
        });
    if (!picked.accepted())
        return;

    const QString entityId = QString::fromStdString(picked.entityId);
    switch (picked.action) {
        case oap::app::ScenePickAction::SelectUnit: {
            const QString unitId = QString::fromStdString(picked.unitId);
            m_resultPanel->selectUnitById(unitId);
            if (m_groupPanel)
                m_groupPanel->selectUnitById(unitId);
            break;
        }
        case oap::app::ScenePickAction::SelectFaces: {
            m_resultPanel->selectFacesByIds(picked.faceIds);
            if (m_featurePanel)
                m_featurePanel->selectFacesByIds(picked.faceIds);
            highlightFaces(
                picked.faceIds,
                picked.matchedFeature
                    ? QStringLiteral("整特征累加选择：当前 %1 个面")
                          .arg(static_cast<int>(picked.faceIds.size()))
                    : QStringLiteral("独立面累加选择：当前 %1 个面")
                          .arg(static_cast<int>(picked.faceIds.size())));
            if (m_groupPanel) {
                if (const oap::Face* face =
                        modelIndex().face(picked.entityId)) {
                    m_groupPanel->selectUnitById(
                        QString::fromStdString(face->unit_id));
                }
            }
            break;
        }
        case oap::app::ScenePickAction::ToggleFace: {
            m_resultPanel->toggleFaceById(entityId);
            const std::vector<std::string> selected =
                m_resultPanel->selectedFaceIds();
            if (m_featurePanel)
                m_featurePanel->selectFacesByIds(selected);
            highlightFaces(selected, {});
            if (m_groupPanel) {
                if (const oap::Face* face =
                        modelIndex().face(picked.entityId)) {
                    m_groupPanel->selectUnitById(
                        QString::fromStdString(face->unit_id));
                }
            }
            break;
        }
        case oap::app::ScenePickAction::SelectFace:
            m_resultPanel->selectFaceById(entityId);
            if (m_featurePanel)
                m_featurePanel->selectFaceById(entityId);
            if (m_groupPanel) {
                if (const oap::Face* face =
                        modelIndex().face(picked.entityId)) {
                    m_groupPanel->selectUnitById(
                        QString::fromStdString(face->unit_id));
                }
            }
            if (picked.requestHoleStageHighlight)
                highlightHoleStageToolpathForFace(picked.entityId);
            break;
        case oap::app::ScenePickAction::ShowEdge:
            if (!picked.faceIds.empty()) {
                m_resultPanel->selectFaceById(
                    QString::fromStdString(picked.faceIds.front()));
            }
            m_resultPanel->showEdgeProperties(entityId);
            statusBar()->showMessage(
                QStringLiteral("已拾取边 %1 (曲线 %2,面 %3)")
                    .arg(entityId)
                    .arg(QString::fromStdString(picked.entitySubtype))
                    .arg(picked.relatedFaceCount));
            break;
        case oap::app::ScenePickAction::ShowWire:
            if (!picked.faceIds.empty()) {
                m_resultPanel->selectFaceById(
                    QString::fromStdString(picked.faceIds.front()));
            }
            m_resultPanel->showWireProperties(entityId);
            statusBar()->showMessage(
                QStringLiteral("已拾取线环 %1 (%2,在面 %3 上)")
                    .arg(entityId)
                    .arg(QString::fromStdString(picked.entitySubtype))
                    .arg(picked.faceIds.empty()
                             ? QStringLiteral("—")
                             : QString::fromStdString(
                                   picked.faceIds.front())));
            break;
        case oap::app::ScenePickAction::None:
            return;
    }
    setVisibilityActionsEnabled();
    updateStatusBar();
}

std::vector<std::string> MainWindow::featureFacesContaining(
    const std::string& faceId) const
{
    std::vector<std::string> result;
    std::unordered_set<std::string> seen;
    const auto appendContainingFeature =
        [&](const std::vector<std::string>& candidate) {
            if (std::find(
                    candidate.begin(), candidate.end(),
                    faceId) == candidate.end())
                return;
            for (const std::string& id : candidate) {
                if (seen.insert(id).second)
                    result.push_back(id);
            }
        };

    // 孔与人工修正实例。
    if (m_recognitionWorkspace->hasResult()) {
        for (const oap::RecognitionResult::Instance& instance :
             m_recognitionWorkspace->result().instances) {
            appendContainingFeature(instance.faceIds);
        }
    }

    // 盲槽保留路径分支；相互边连通的盲腔已合为一个排他特征。
    for (const WireTraceFeatureSubset& subset :
         m_recognitionWorkspace->wireTraceFeatureSubsets()) {
        appendContainingFeature(subset.faceIds);
    }

    // 当前通结构识别结果以单面为一个命中项。
    if (m_recognitionWorkspace->throughStructureFaces().contains(faceId) &&
        seen.insert(faceId).second) {
        result.push_back(faceId);
    }

    return result;
}

void MainWindow::onSelectionCleared()
{
    clearTransientOverlay();
    if (m_resultPanel) m_resultPanel->clearSelection();
    if (m_featurePanel) m_featurePanel->clearSelection();
    if (m_groupPanel) m_groupPanel->clearSelection();
    m_sceneInteraction.resetSelection();
    if (!m_view->context().IsNull())
        m_view->context()->ClearSelected(Standard_False);
    if (!m_view->view().IsNull()) m_view->view()->Redraw();
    setVisibilityActionsEnabled();
    updateStatusBar();
}

void MainWindow::onEntitiesBoxPicked(const std::vector<TopoDS_Shape>& shapes,
                                     int shapeType)
{
    const oap::app::SceneBoxSelectionResult selection =
        m_sceneInteraction.mapBoxPick(
            shapes, shapeType,
            m_view->selectionMode() == OcctView::SelMode::Unit,
            stepIndexer(), modelIndex());
    if (selection.selectUnits) {
        m_resultPanel->selectUnitsByIds(selection.unitIds);
        if (m_groupPanel)
            m_groupPanel->selectUnitsByIds(selection.unitIds);
        statusBar()->showMessage(
            QStringLiteral("框选单元：%1 个")
                .arg(selection.unitIds.size()));
    } else {
        m_resultPanel->selectFacesByIds(selection.faceIds);
        if (m_groupPanel)
            m_groupPanel->selectUnitsByIds(selection.unitIds);
        const TopAbs_ShapeEnum type =
            static_cast<TopAbs_ShapeEnum>(selection.shapeType);
        const QString noun = type == TopAbs_EDGE
            ? QStringLiteral("边")
            : type == TopAbs_WIRE ? QStringLiteral("线环")
                                  : QStringLiteral("面");
        statusBar()->showMessage(
            QStringLiteral("框选%1：%2 个，关联 %3 个面")
                .arg(noun)
                .arg(selection.entityCount)
                .arg(selection.faceIds.size()));
    }
    setVisibilityActionsEnabled();
}

void MainWindow::setSelectionModeFace() { m_view->setSelectionMode(OcctView::SelMode::Face); updateStatusBar(); }
void MainWindow::setSelectionModeEdge() { m_view->setSelectionMode(OcctView::SelMode::Edge); updateStatusBar(); }
void MainWindow::setSelectionModeWire() { m_view->setSelectionMode(OcctView::SelMode::Wire); updateStatusBar(); }
void MainWindow::setSelectionModeUnit() { m_view->setSelectionMode(OcctView::SelMode::Unit); updateStatusBar(); }

void MainWindow::onTreeFaceSelected(const QString &faceId)
{
    m_sceneInteraction.setSelectionSource(
        oap::app::SelectionSource::Analysis);
    highlightFaces({faceId.toStdString()}, QStringLiteral("面 %1").arg(faceId));
}

void MainWindow::onFeatureSelectionChanged()
{
    // 识别面板成为选择来源(后续 Hide/Only 作用于其选择),并在 3D 高亮当前面集合。
    m_sceneInteraction.setSelectionSource(
        oap::app::SelectionSource::Features);
    const auto ids = m_featurePanel ? m_featurePanel->selectedFaceIds()
                                    : std::vector<std::string>{};
    highlightFaces(ids,
        QStringLiteral("特征选择: %1 个面").arg((int)ids.size()));
}

void MainWindow::onGroupSelectionChanged()
{
    // 分组面板成为选择来源(后续 Hide/Only 作用于其选择),并在 3D 高亮当前面集合。
    m_sceneInteraction.setSelectionSource(
        oap::app::SelectionSource::Groups);
    const auto ids = m_groupPanel ? m_groupPanel->selectedFaceIds()
                                  : std::vector<std::string>{};
    highlightFaces(ids,
        QStringLiteral("分组选择: %1 个面").arg((int)ids.size()));
}

void MainWindow::highlightFaces(const std::vector<std::string> &faceIds,
                                const QString &status)
{
    // 在 3D 中高亮一组面:选中其按面 AIS 对象。隐藏的面跳过(需求 §12.4)。
    if (m_view->context().IsNull())
        return;
    clearTransientOverlay();
    m_sceneInteraction.selectVisibleFaces(
        m_view->context(), faceIds);
    m_view->view()->Redraw();
    if (!status.isEmpty())
        statusBar()->showMessage(status);
    setVisibilityActionsEnabled();
    updateStatusBar();
}

std::vector<std::string> MainWindow::currentSelectedFaceIds() const
{
    return m_sceneInteraction.currentSelection(
        currentSelectionSnapshot());
}

oap::app::SelectionSnapshot MainWindow::currentSelectionSnapshot() const
{
    oap::app::SelectionSnapshot snapshot;
    if (m_resultPanel)
        snapshot.analysis = m_resultPanel->selectedFaceIds();
    if (m_featurePanel)
        snapshot.features = m_featurePanel->selectedFaceIds();
    if (m_groupPanel)
        snapshot.groups = m_groupPanel->selectedFaceIds();
    return snapshot;
}

void MainWindow::refreshAfterVisibility()
{
    m_view->view()->Redraw();
    setVisibilityActionsEnabled();
    updateStatusBar();
}

void MainWindow::hideSelected()
{
    if (m_sceneInteraction.applyVisibility(
            oap::app::SceneVisibilityCommand::HideSelected,
            currentSelectionSnapshot(), m_undoHistory,
            [this] { clearOverlay(); })) {
        refreshAfterVisibility();
    }
}

void MainWindow::onlySelected()
{
    if (m_sceneInteraction.applyVisibility(
            oap::app::SceneVisibilityCommand::OnlySelected,
            currentSelectionSnapshot(), m_undoHistory,
            [this] { clearOverlay(); })) {
        refreshAfterVisibility();
    }
}

void MainWindow::invertShow()
{
    if (m_sceneInteraction.applyVisibility(
            oap::app::SceneVisibilityCommand::Invert,
            currentSelectionSnapshot(), m_undoHistory,
            [this] { clearTransientOverlay(); })) {
        refreshAfterVisibility();
    }
}

void MainWindow::showSelected()
{
    if (m_sceneInteraction.applyVisibility(
            oap::app::SceneVisibilityCommand::ShowSelected,
            currentSelectionSnapshot(), m_undoHistory,
            [this] { clearOverlay(); })) {
        refreshAfterVisibility();
    }
}

void MainWindow::showAll()
{
    if (m_sceneInteraction.applyVisibility(
            oap::app::SceneVisibilityCommand::ShowAll,
            currentSelectionSnapshot(), m_undoHistory)) {
        refreshAfterVisibility();
    }
}

void MainWindow::undoVisibility()
{
    oap::app::SceneUndoResult undo = m_sceneInteraction.takeUndo(
        m_undoHistory, [this] { clearOverlay(); });
    if (undo.restoredVisibility) {
        refreshAfterVisibility();
        return;
    }
}

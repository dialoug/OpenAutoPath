// MainWindow 的视图导航、显示模式和模型变换同步。
#include "MainWindow.h"

#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/presentation/OverlayController.h"

#include <algorithm>

#include <QAction>
#include <QSignalBlocker>
#include <QStatusBar>

#include <Bnd_Box.hxx>
#include <V3d.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace {

constexpr double kFitAllMargin = 0.1;

}

void MainWindow::snapToNearestStandardView()
{
    if (m_view->view().IsNull() || m_view->view()->Camera().IsNull())
        return;

    // 把当前观察方向(eye→at)取反得到视线矢量,与六个标准视图投影轴做点积,
    // 取点积最大(夹角最小)者作为最近方位。固定相机旋转模型时先把视线
    // 反变换回模型坐标系再比较,避免模型变换干扰方位判定。
    gp_Dir modelViewDirection = -m_view->view()->Camera()->Direction();
    if (m_view->rotationMode() == OcctView::RotationMode::Model) {
        const gp_Trsf inverseModel = m_view->modelTransform().Inverted();
        modelViewDirection.Transform(inverseModel);
    }
    struct Candidate {
        V3d_TypeOfOrientation orientation;
        const char* label;
    };
    const Candidate candidates[] = {
        {V3d_TypeOfOrientation_Zup_Front, "前视图"},
        {V3d_TypeOfOrientation_Zup_Back, "后视图"},
        {V3d_TypeOfOrientation_Zup_Left, "左视图"},
        {V3d_TypeOfOrientation_Zup_Right, "右视图"},
        {V3d_TypeOfOrientation_Zup_Top, "顶视图"},
        {V3d_TypeOfOrientation_Zup_Bottom, "底视图"},
    };
    const Candidate* nearest = &candidates[0];
    double bestAlignment =
        modelViewDirection.Dot(V3d::GetProjAxis(nearest->orientation));
    for (const Candidate& candidate : candidates) {
        const double alignment =
            modelViewDirection.Dot(V3d::GetProjAxis(candidate.orientation));
        if (alignment > bestAlignment) {
            bestAlignment = alignment;
            nearest = &candidate;
        }
    }
    m_view->setStandardOrientation(nearest->orientation);
    const bool fitted = fitModelInView();
    m_view->view()->Redraw();
    statusBar()->showMessage(
        fitted
            ? QStringLiteral("已吸附到最近方位：%1，并适合窗口")
                  .arg(QString::fromUtf8(nearest->label))
            : QStringLiteral("已吸附到最近方位：%1；没有可适配的可见模型")
                  .arg(QString::fromUtf8(nearest->label)));
}

void MainWindow::focusCurrentSelection()
{
    const std::vector<std::string> faceIds = currentSelectedFaceIds();
    if (faceIds.empty()) {
        statusBar()->showMessage(QStringLiteral("请先选择要定位的面或单元。"));
        return;
    }
    if (fitViewToFaces(faceIds))
        statusBar()->showMessage(
            QStringLiteral("已将当前选择居中定位：%1 个面")
                .arg(static_cast<int>(faceIds.size())));
    else
        statusBar()->showMessage(
            QStringLiteral("当前选择没有可用的几何范围，无法定位。"));
}

bool MainWindow::fitModelInView()
{
    if (m_view->view().IsNull())
        return false;
    oap::Bbox bbox;
    for (const oap::Face& face : model().faces)
        if (!sceneController().isHidden(face.face_id))
            bbox.merge(face.bbox);
    if (!bbox.valid())
        return false;

    // 把可见面的模型包围盒 8 个角点经模型变换(固定相机旋转模型时非恒等)
    // 映射到视图空间,再 FitAll——确保“适合窗口”始终以屏幕可视范围为准,
    // 而不是被模型变换旋出视野的原始坐标。
    Bnd_Box viewBox;
    const gp_Trsf& transform = sceneController().modelTransform();
    for (int ix = 0; ix < 2; ++ix)
        for (int iy = 0; iy < 2; ++iy)
            for (int iz = 0; iz < 2; ++iz) {
                gp_Pnt corner(ix == 0 ? bbox.xmin : bbox.xmax,
                              iy == 0 ? bbox.ymin : bbox.ymax,
                              iz == 0 ? bbox.zmin : bbox.zmax);
                corner.Transform(transform);
                viewBox.Update(corner.X(), corner.Y(), corner.Z());
            }
    const auto size = bbox.size();
    const double maxDimension = std::max({size[0], size[1], size[2]});
    viewBox.Enlarge(std::max(maxDimension * 0.03, 1.0e-3));
    m_view->view()->FitAll(viewBox, kFitAllMargin, Standard_True);
    return true;
}

bool MainWindow::fitViewToFaces(const std::vector<std::string>& faceIds)
{
    if (faceIds.empty() || m_view->view().IsNull())
        return false;
    oap::Bbox bbox;
    for (const std::string& faceId : faceIds) {
        const oap::Face* face = modelIndex().face(faceId);
        if (face)
            bbox.merge(face->bbox);
    }
    if (!bbox.valid())
        return false;

    Bnd_Box viewBox;
    const gp_Trsf& transform = sceneController().modelTransform();
    for (int ix = 0; ix < 2; ++ix)
        for (int iy = 0; iy < 2; ++iy)
            for (int iz = 0; iz < 2; ++iz) {
                gp_Pnt corner(ix == 0 ? bbox.xmin : bbox.xmax,
                              iy == 0 ? bbox.ymin : bbox.ymax,
                              iz == 0 ? bbox.zmin : bbox.zmax);
                corner.Transform(transform);
                viewBox.Update(corner.X(), corner.Y(), corner.Z());
            }
    const auto size = bbox.size();
    const double maxDimension = std::max({size[0], size[1], size[2]});
    viewBox.Enlarge(std::max(maxDimension * 0.03, 1.0e-3));
    m_view->view()->FitAll(viewBox, kFitAllMargin, Standard_True);
    m_view->view()->Redraw();
    return true;
}

void MainWindow::applyModelTransformToOverlays()
{
    const gp_Trsf& transform = sceneController().modelTransform();
    const auto applySequence =
        [&transform](NCollection_Sequence<Handle(AIS_InteractiveObject)>& sequence) {
            for (NCollection_Sequence<Handle(AIS_InteractiveObject)>::Iterator
                     it(sequence);
                 it.More(); it.Next()) {
                if (!it.Value().IsNull())
                    it.Value()->SetLocalTransformation(transform);
            }
        };
    if (m_overlayController) {
        m_overlayController->setLocalTransformation(
            oap::app::presentation::OverlayGroup::HoleStock,
            transform);
        m_overlayController->setLocalTransformation(
            oap::app::presentation::OverlayGroup::PocketIpw,
            transform);
        m_overlayController->setLocalTransformation(
            oap::app::presentation::OverlayGroup::HoleStageHighlight,
            transform);
        m_overlayController->setLocalTransformation(
            oap::app::presentation::OverlayGroup::TemporaryHighlight,
            transform);
        m_overlayController->setLocalTransformation(
            oap::app::presentation::OverlayGroup::Toolpath,
            transform);
        m_overlayController->setLocalTransformation(
            oap::app::presentation::OverlayGroup::CurrentToolpath,
            transform);
    }

    if (m_playbackWorkspace)
        m_playbackWorkspace->applyModelTransform(transform);
}

void MainWindow::toggleShaded(bool shaded)
{
    sceneController().setShaded(shaded);
    if (m_actShaded)
        m_actShaded->setText(
            shaded ? QStringLiteral("着色") : QStringLiteral("线框"));
}

void MainWindow::toggleEdges(bool enabled)
{
    sceneController().setEdges(enabled);
}

void MainWindow::toggleConvexityEdges(bool enabled)
{
    sceneController().setConvexityEdges(enabled);
    if (!m_view->view().IsNull())
        m_view->view()->Redraw();
    if (enabled) {
        statusBar()->showMessage(
            QStringLiteral("凹凸边：凹边 %1（红），光顺边 %2（蓝），凸边 %3（绿）")
                .arg(static_cast<qulonglong>(
                    sceneController().visibleConcaveEdgeCount()))
                .arg(static_cast<qulonglong>(
                    sceneController().visibleSmoothEdgeCount()))
                .arg(static_cast<qulonglong>(
                    sceneController().visibleConvexEdgeCount())));
    } else {
        statusBar()->showMessage(QStringLiteral("已关闭凹凸边显示"));
    }
}

void MainWindow::toggleFaded(bool enabled)
{
    sceneController().setFaded(enabled);
    if (m_actFeatureColors &&
        m_actFeatureColors->isChecked() &&
        m_recognitionWorkspace->hasResult()) {
        applyRecognitionColors();
    }
    if (!m_view->view().IsNull())
        m_view->view()->Redraw();
    statusBar()->showMessage(
        enabled ? QStringLiteral("已启用褪色底色")
                : QStringLiteral("已恢复 STEP 原始颜色"));
}

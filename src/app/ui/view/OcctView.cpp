// OpenAutoPath — OcctView：在 QWidget 的原生窗口上承载 OpenCASCADE 的 V3d_View。
#include "app/ui/view/OcctView.h"
#include <QCoreApplication>
#include "app/presentation/ToolpathSegmentsObject.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QContextMenuEvent>
#include <QPainter>
#include <QPen>
#include <QRect>
#include <QStringList>
#include <QToolTip>

#include <cmath>

#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <AIS_ViewCube.hxx>
#include <AIS_AnimationCamera.hxx>
#include <AIS_Shape.hxx>
#include <Graphic3d_Camera.hxx>
#include <gp_Ax3.hxx>
#include <gp_Ax1.hxx>
#include <gp_Mat.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <WNT_Window.hxx>
#include <Quantity_Color.hxx>
#include <TopAbs.hxx>
#include <TopoDS_Shape.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <AIS_SelectionScheme.hxx>
#include <Graphic3d_Vec2.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <Prs3d_DatumParts.hxx>
#include <Prs3d_TypeOfHighlight.hxx>
#include <Bnd_Box.hxx>

namespace {
// 3D 视图最小尺寸（像素）。
constexpr int kMinViewWidth = 400;
constexpr int kMinViewHeight = 300;
// 背景灰度的合法区间。
constexpr double kGrayMin = 0.0;
constexpr double kGrayMax = 1.0;
// 浮点取整四舍五入偏置。
constexpr double kRoundBias = 0.5;
// 区分单击与拖动的曼哈顿距离阈值（像素）。
constexpr int kClickDragThresholdPx = 4;
// 滚轮缩放因子（>0 放大，否则缩小）。
constexpr double kZoomInFactor = 1.1;
constexpr double kZoomOutFactor = 0.9;
constexpr double kModelRotationRadiansPerPixel = 0.008;
constexpr int kRubberBandBorderPx = 2;
const QColor kRubberBandColor(64, 160, 255);

QString toolpathSegmentDescription(
    const oap::app::presentation::ToolpathPreviewSegment& segment)
{
    using oap::appdetail::HolePreviewSegmentKind;
    QString kind;
    switch (segment.kind) {
        case HolePreviewSegmentKind::Cut:
            kind = QCoreApplication::translate("OpenAutoPath", "切削");
            break;
        case HolePreviewSegmentKind::Rapid:
            kind = QCoreApplication::translate("OpenAutoPath", "快移");
            break;
        case HolePreviewSegmentKind::Vertical:
            kind = QCoreApplication::translate("OpenAutoPath", "进/退刀");
            break;
    }

    const auto& identity = segment.identity;
    QStringList details;
    const QString command = identity.commandName.empty()
        ? QString()
        : QStringLiteral(" · %1")
              .arg(QString::fromStdString(identity.commandName));
    details << (identity.moveIndex >= 0
        ? QCoreApplication::translate("OpenAutoPath", "刀路段 #%1 · %2%3")
              .arg(identity.moveIndex + 1)
              .arg(kind, command)
        : QCoreApplication::translate("OpenAutoPath", "刀路段 · %1%2").arg(kind, command));
    if (!identity.operationId.empty()) {
        details << QCoreApplication::translate("OpenAutoPath", "工序：%1")
            .arg(QString::fromStdString(identity.operationId));
    }
    if (identity.primitiveIndex >= 0) {
        QString primitive = QCoreApplication::translate("OpenAutoPath", "图元 #%1")
            .arg(identity.primitiveIndex + 1);
        if (identity.primitiveMoveIndex >= 0 &&
            identity.primitiveMoveCount > 0) {
            primitive += QCoreApplication::translate("OpenAutoPath", " · 展开段 %1/%2")
                .arg(identity.primitiveMoveIndex + 1)
                .arg(identity.primitiveMoveCount);
        }
        details << primitive;
    }
    if (!identity.commandSummary.empty()) {
        details << QCoreApplication::translate("OpenAutoPath", "语义：%1")
            .arg(QString::fromStdString(identity.commandSummary));
    }
    QStringList source;
    if (!identity.featureName.empty())
        source << QString::fromStdString(identity.featureName);
    if (identity.sourceIndex >= 0)
        source << QCoreApplication::translate("OpenAutoPath", "源 #%1").arg(identity.sourceIndex + 1);
    if (identity.setupIndex >= 0)
        source << QCoreApplication::translate("OpenAutoPath", "装夹 #%1").arg(identity.setupIndex + 1);
    if (!source.empty())
        details << QCoreApplication::translate("OpenAutoPath", "来源：%1").arg(source.join(QStringLiteral(" · ")));

    QStringList tool;
    if (identity.toolNumber >= 0)
        tool << QStringLiteral("T%1").arg(identity.toolNumber);
    if (!identity.toolName.empty())
        tool << QString::fromStdString(identity.toolName);
    if (!identity.toolType.empty())
        tool << QString::fromStdString(identity.toolType);
    if (!tool.empty())
        details << QCoreApplication::translate("OpenAutoPath", "刀具：%1").arg(tool.join(QStringLiteral(" · ")));

    QStringList process;
    if (identity.operationMoveIndex >= 0) {
        process << QCoreApplication::translate("OpenAutoPath", "工序内 #%1")
            .arg(identity.operationMoveIndex + 1);
    }
    if (identity.depthIndex >= 0)
        process << QCoreApplication::translate("OpenAutoPath", "深度层 %1").arg(identity.depthIndex + 1);
    if (identity.loopSequence >= 0)
        process << QCoreApplication::translate("OpenAutoPath", "环序 %1").arg(identity.loopSequence);
    if (identity.ring >= 0)
        process << QCoreApplication::translate("OpenAutoPath", "偏置环 %1").arg(identity.ring);
    if (identity.feed > 0.0)
        process << QStringLiteral("F%1 mm/min").arg(identity.feed, 0, 'f', 1);
    if (!process.empty())
        details << QCoreApplication::translate("OpenAutoPath", "参数：%1").arg(process.join(QStringLiteral(" · ")));

    const double dx = segment.to.X() - segment.from.X();
    const double dy = segment.to.Y() - segment.from.Y();
    const double dz = segment.to.Z() - segment.from.Z();
    details << QCoreApplication::translate("OpenAutoPath", "方向：ΔX %1  ΔY %2  ΔZ %3  长度 %4 mm")
        .arg(dx, 0, 'f', 3)
        .arg(dy, 0, 'f', 3)
        .arg(dz, 0, 'f', 3)
        .arg(segment.from.Distance(segment.to), 0, 'f', 3);
    return details.join(QLatin1Char('\n'));
}

class SelectionFrameWindow final : public QWidget
{
public:
    explicit SelectionFrameWindow(QWidget* owner)
        : QWidget(owner, Qt::Tool | Qt::FramelessWindowHint |
                         Qt::NoDropShadowWindowHint | Qt::WindowTransparentForInput)
    {
        setAttribute(Qt::WA_ShowWithoutActivating);
        setAttribute(Qt::WA_NoSystemBackground);
        setAttribute(Qt::WA_TranslucentBackground);
        setFocusPolicy(Qt::NoFocus);
        hide();
    }

    void showFrame(QWidget* anchor, const QRect& rectInAnchor)
    {
        const QRect rect = rectInAnchor.normalized();
        if (!anchor || rect.isEmpty()) { hide(); return; }
        QRect windowRect(anchor->mapToGlobal(rect.topLeft()), rect.size());
        windowRect.adjust(-kRubberBandBorderPx, -kRubberBandBorderPx,
                          kRubberBandBorderPx, kRubberBandBorderPx);
        setGeometry(windowRect);
        m_frameRect = QRect(QPoint(kRubberBandBorderPx, kRubberBandBorderPx), rect.size());
        show();
        raise();
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setPen(QPen(kRubberBandColor, kRubberBandBorderPx));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(m_frameRect.adjusted(1, 1, -1, -1));
    }

private:
    QRect m_frameRect;
};

gp_XYZ orientationVector(V3d_TypeOfOrientation orientation)
{
    switch (orientation) {
        case V3d_Xpos: return {1, 0, 0}; case V3d_Ypos: return {0, 1, 0};
        case V3d_Zpos: return {0, 0, 1}; case V3d_Xneg: return {-1, 0, 0};
        case V3d_Yneg: return {0, -1, 0}; case V3d_Zneg: return {0, 0, -1};
        case V3d_XposYpos: return {1, 1, 0}; case V3d_XposZpos: return {1, 0, 1};
        case V3d_YposZpos: return {0, 1, 1}; case V3d_XnegYneg: return {-1, -1, 0};
        case V3d_XnegYpos: return {-1, 1, 0}; case V3d_XnegZneg: return {-1, 0, -1};
        case V3d_XnegZpos: return {-1, 0, 1}; case V3d_YnegZneg: return {0, -1, -1};
        case V3d_YnegZpos: return {0, -1, 1}; case V3d_XposYneg: return {1, -1, 0};
        case V3d_XposZneg: return {1, 0, -1}; case V3d_YposZneg: return {0, 1, -1};
        case V3d_XposYposZpos: return {1, 1, 1};
        case V3d_XposYnegZpos: return {1, -1, 1};
        case V3d_XposYposZneg: return {1, 1, -1};
        case V3d_XnegYposZpos: return {-1, 1, 1};
        case V3d_XposYnegZneg: return {1, -1, -1};
        case V3d_XnegYposZneg: return {-1, 1, -1};
        case V3d_XnegYnegZpos: return {-1, -1, 1};
        case V3d_XnegYnegZneg: return {-1, -1, -1};
    }
    return {0, 0, 1};
}

V3d_TypeOfOrientation orientationFromVector(const gp_XYZ& vector)
{
    const int x = vector.X() > 0.5 ? 1 : (vector.X() < -0.5 ? -1 : 0);
    const int y = vector.Y() > 0.5 ? 1 : (vector.Y() < -0.5 ? -1 : 0);
    const int z = vector.Z() > 0.5 ? 1 : (vector.Z() < -0.5 ? -1 : 0);
    if (x == 1 && y == 0 && z == 0) return V3d_Xpos;
    if (x == 0 && y == 1 && z == 0) return V3d_Ypos;
    if (x == 0 && y == 0 && z == 1) return V3d_Zpos;
    if (x == -1 && y == 0 && z == 0) return V3d_Xneg;
    if (x == 0 && y == -1 && z == 0) return V3d_Yneg;
    if (x == 0 && y == 0 && z == -1) return V3d_Zneg;
    if (x == 1 && y == 1 && z == 0) return V3d_XposYpos;
    if (x == 1 && y == 0 && z == 1) return V3d_XposZpos;
    if (x == 0 && y == 1 && z == 1) return V3d_YposZpos;
    if (x == -1 && y == -1 && z == 0) return V3d_XnegYneg;
    if (x == -1 && y == 1 && z == 0) return V3d_XnegYpos;
    if (x == -1 && y == 0 && z == -1) return V3d_XnegZneg;
    if (x == -1 && y == 0 && z == 1) return V3d_XnegZpos;
    if (x == 0 && y == -1 && z == -1) return V3d_YnegZneg;
    if (x == 0 && y == -1 && z == 1) return V3d_YnegZpos;
    if (x == 1 && y == -1 && z == 0) return V3d_XposYneg;
    if (x == 1 && y == 0 && z == -1) return V3d_XposZneg;
    if (x == 0 && y == 1 && z == -1) return V3d_YposZneg;
    if (x == 1 && y == 1 && z == 1) return V3d_XposYposZpos;
    if (x == 1 && y == -1 && z == 1) return V3d_XposYnegZpos;
    if (x == 1 && y == 1 && z == -1) return V3d_XposYposZneg;
    if (x == -1 && y == 1 && z == 1) return V3d_XnegYposZpos;
    if (x == 1 && y == -1 && z == -1) return V3d_XposYnegZneg;
    if (x == -1 && y == 1 && z == -1) return V3d_XnegYposZneg;
    if (x == -1 && y == -1 && z == 1) return V3d_XnegYnegZpos;
    return V3d_XnegYnegZneg;
}

class MoldViewCube : public AIS_ViewCube
{
public:
    void setMoldFrame(const oap::MoldCoordinateFrame* frame)
    {
        m_hasFrame = frame && frame->isValid;
        if (!m_hasFrame) {
            m_x = gp_Dir(1, 0, 0);
            m_y = gp_Dir(0, 1, 0);
            m_z = gp_Dir(0, 0, 1);
            m_frameTransform = gp_Trsf();
            return;
        }
        m_x = frame->xAxis;
        m_y = frame->yAxis;
        m_z = frame->zAxis;
        const gp_Ax3 localSystem(gp_Pnt(0, 0, 0), m_z, m_x);
        const gp_Ax3 worldSystem;
        m_frameTransform.SetTransformation(localSystem, worldSystem);
    }

    const gp_Trsf& frameTransform() const
    {
        return m_frameTransform;
    }

    V3d_TypeOfOrientation worldOrientation(
        V3d_TypeOfOrientation orientation) const
    {
        if (!m_hasFrame)
            return orientation;
        const gp_XYZ local = orientationVector(orientation);
        const gp_XYZ world = m_x.XYZ() * local.X() +
                             m_y.XYZ() * local.Y() +
                             m_z.XYZ() * local.Z();
        return orientationFromVector(world);
    }

    void StartAnimation(const Handle(AIS_ViewCubeOwner)& owner) override
    {
        if (!m_hasFrame || owner.IsNull()) {
            AIS_ViewCube::StartAnimation(owner);
            return;
        }
        const gp_XYZ local = orientationVector(owner->MainOrientation());
        const gp_XYZ world = m_x.XYZ() * local.X() +
                             m_y.XYZ() * local.Y() +
                             m_z.XYZ() * local.Z();
        Handle(AIS_ViewCubeOwner) mapped =
            new AIS_ViewCubeOwner(this, orientationFromVector(world));
        AIS_ViewCube::StartAnimation(mapped);

        if (!ViewAnimation().IsNull() && !ViewAnimation()->CameraEnd().IsNull()) {
            gp_Vec eye(world);
            if (eye.SquareMagnitude() > 0.0)
                eye.Normalize();
            gp_Vec up(m_z);
            up -= eye.Multiplied(up.Dot(eye));
            if (up.SquareMagnitude() < 1.0e-12) {
                up = gp_Vec(m_y);
                up -= eye.Multiplied(up.Dot(eye));
            }
            if (up.SquareMagnitude() > 1.0e-12) {
                up.Normalize();
                ViewAnimation()->CameraEnd()->SetUp(gp_Dir(up));
            }
        }
    }

private:
    bool m_hasFrame = false;
    gp_Trsf m_frameTransform;
    gp_Dir m_x{1, 0, 0};
    gp_Dir m_y{0, 1, 0};
    gp_Dir m_z{0, 0, 1};
};
} // namespace

OcctView::OcctView(QWidget* parent) : QWidget(parent)
{
    // OCCT 在该控件的原生表面上绘制，因此要阻止 Qt 在其上覆盖绘制。
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(kMinViewWidth, kMinViewHeight);
}

void OcctView::initViewer()
{
    if (!m_view.IsNull())
        return;

    Handle(Aspect_DisplayConnection) display = new Aspect_DisplayConnection();
    Handle(OpenGl_GraphicDriver) driver = new OpenGl_GraphicDriver(display);

    m_viewer = new V3d_Viewer(driver);
    m_viewer->SetDefaultLights();
    m_viewer->SetLightOn();

    m_context = new AIS_InteractiveContext(m_viewer);

    const Quantity_Color selectedFaceColor(0.45, 1.0, 0.55, Quantity_TOC_RGB);
    const Quantity_Color dynamicFaceColor(0.45, 0.78, 1.0, Quantity_TOC_RGB);
    for (Prs3d_TypeOfHighlight type : {Prs3d_TypeOfHighlight_Selected,
                                       Prs3d_TypeOfHighlight_LocalSelected}) {
        const Handle(Prs3d_Drawer)& style = m_context->HighlightStyle(type);
        if (!style.IsNull()) {
            style->SetColor(selectedFaceColor);
            style->SetTransparency(0.0f);
            style->SetDisplayMode(AIS_Shaded);
        }
    }
    const Handle(Prs3d_Drawer)& dynamicStyle =
        m_context->HighlightStyle(Prs3d_TypeOfHighlight_Dynamic);
    if (!dynamicStyle.IsNull()) {
        dynamicStyle->SetColor(dynamicFaceColor);
        dynamicStyle->SetTransparency(0.0f);
        dynamicStyle->SetDisplayMode(AIS_WireFrame);
    }

    m_view = m_viewer->CreateView();
    Aspect_Drawable hwnd = (Aspect_Drawable)winId();
    Handle(WNT_Window) window = new WNT_Window((Aspect_Handle)hwnd);
    m_view->SetWindow(window);
    if (!window->IsMapped())
        window->Map();
    m_view->MustBeResized();
    setBackgroundGray(m_grayLevel);

    showViewCube();
    emit viewerInitialized();
}

void OcctView::showViewCube()
{
    if (m_context.IsNull())
        return;
    if (m_viewCube.IsNull()) {
        m_viewCube = new MoldViewCube();
        m_viewCube->SetAxesRadius(3.0);
        m_viewCube->SetAxesConeRadius(7.0);

        Handle(Prs3d_DatumAspect) axes = new Prs3d_DatumAspect();
        axes->ShadingAspect(Prs3d_DatumParts_XAxis)->SetColor(Quantity_NOC_RED);
        axes->ShadingAspect(Prs3d_DatumParts_YAxis)->SetColor(Quantity_NOC_GREEN);
        axes->ShadingAspect(Prs3d_DatumParts_ZAxis)->SetColor(Quantity_NOC_BLUE);
        axes->TextAspect(Prs3d_DatumParts_XAxis)->SetColor(Quantity_NOC_RED);
        axes->TextAspect(Prs3d_DatumParts_YAxis)->SetColor(Quantity_NOC_GREEN);
        axes->TextAspect(Prs3d_DatumParts_ZAxis)->SetColor(Quantity_NOC_BLUE);
        m_viewCube->Attributes()->SetDatumAspect(axes);
    }
    if (!m_context->IsDisplayed(m_viewCube))
        m_context->Display(m_viewCube, Standard_False);

    if (!m_hasViewCubeVisualCenter) {
        Bnd_Box bounds;
        m_viewCube->BoundingBox(bounds);
        if (!bounds.IsVoid() && !bounds.IsWhole()) {
            Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
            Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
            bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
            m_viewCubeVisualCenter.SetCoord((xmin + xmax) * 0.5,
                                            (ymin + ymax) * 0.5,
                                            (zmin + zmax) * 0.5);
            m_hasViewCubeVisualCenter = true;
        }
    }
    m_context->Activate(m_viewCube, 0);
    syncViewCubeTransform();
}

void OcctView::setMoldCoordinateFrame(const oap::MoldCoordinateFrame& frame)
{
    initViewer();
    if (m_context.IsNull() || m_viewCube.IsNull())
        return;
    // RemoveAll() 会移除 HUD；每次载入模型时显式恢复视图立方体。
    showViewCube();
    MoldViewCube* cube = dynamic_cast<MoldViewCube*>(m_viewCube.get());
    if (cube)
        cube->setMoldFrame(frame.isValid ? &frame : nullptr);
    syncViewCubeTransform();
    m_context->Redisplay(m_viewCube, Standard_False);
    setSelectionMode(m_selMode);
    m_view->Redraw();
}

void OcctView::clearMoldCoordinateFrame()
{
    setMoldCoordinateFrame(oap::MoldCoordinateFrame{});
}

void OcctView::setRotationMode(RotationMode mode)
{
    if (m_rotationMode == mode)
        return;

    if (m_rotationMode == RotationMode::Model &&
        mode == RotationMode::Camera &&
        !m_view.IsNull() && !m_view->Camera().IsNull()) {
        const gp_Trsf inverseModel = m_modelTransform.Inverted();
        Handle(Graphic3d_Camera) camera = new Graphic3d_Camera(m_view->Camera());
        gp_Pnt eye = camera->Eye();
        gp_Pnt center = camera->Center();
        gp_Dir up = camera->OrthogonalizedUp();
        eye.Transform(inverseModel);
        center.Transform(inverseModel);
        up.Transform(inverseModel);
        camera->SetEyeAndCenter(eye, center);
        camera->SetUp(up);
        m_view->SetCamera(camera);

        m_rotationMode = mode;
        gp_Trsf identity;
        updateModelTransform(identity);
        m_view->Invalidate();
        m_view->Redraw();
        return;
    }

    m_rotationMode = mode;
    syncViewCubeTransform();
    if (!m_view.IsNull())
        m_view->Redraw();
}

void OcctView::setModelCenter(double x, double y, double z)
{
    m_modelCenter.SetCoord(x, y, z);
    m_hasModelCenter = true;
}

void OcctView::clearModelCenter()
{
    m_hasModelCenter = false;
}

void OcctView::resetModelTransform()
{
    gp_Trsf identity;
    updateModelTransform(identity);
}

void OcctView::updateModelTransform(const gp_Trsf& transform)
{
    m_modelTransform = transform;
    syncViewCubeTransform();
    emit modelTransformChanged();
}

void OcctView::syncViewCubeTransform()
{
    if (m_viewCube.IsNull())
        return;

    gp_Quaternion orientation;
    if (MoldViewCube* cube = dynamic_cast<MoldViewCube*>(m_viewCube.get()))
        orientation = cube->frameTransform().GetRotation();
    if (m_rotationMode == RotationMode::Model)
        orientation = m_modelTransform.GetRotation() * orientation;

    gp_Trsf transform;
    transform.SetRotation(orientation);
    if (m_hasViewCubeVisualCenter) {
        const gp_Pnt rotatedCenter = m_viewCubeVisualCenter.Transformed(transform);
        transform.SetTranslationPart(
            gp_Vec(rotatedCenter, m_viewCubeVisualCenter));
    }
    m_viewCube->SetLocalTransformation(transform);
}

void OcctView::rotateModelByPixels(int dx, int dy)
{
    // 固定相机模式的模型旋转:绕相机 up 轴做 yaw、绕相机 side 轴做 pitch,
    // 二者预乘到模型变换上,使模型绕 m_modelCenter 旋转而相机保持不动。
    if ((dx == 0 && dy == 0) || m_view.IsNull())
        return;
    const Handle(Graphic3d_Camera)& camera = m_view->Camera();
    if (camera.IsNull())
        return;

    const gp_Pnt sourceCenter =
        m_hasModelCenter ? m_modelCenter : gp_Pnt(0.0, 0.0, 0.0);
    const gp_Pnt pivot = sourceCenter.Transformed(m_modelTransform);
    gp_Trsf yaw;
    yaw.SetRotation(gp_Ax1(pivot, camera->OrthogonalizedUp()),
                    dx * kModelRotationRadiansPerPixel);
    gp_Trsf pitch;
    const int verticalPixels = m_modelRotationVerticalInverted ? -dy : dy;
    pitch.SetRotation(gp_Ax1(pivot, camera->SideRight()),
                      verticalPixels * kModelRotationRadiansPerPixel);

    gp_Trsf delta = yaw;
    delta.PreMultiply(pitch);
    gp_Trsf next = m_modelTransform;
    next.PreMultiply(delta);
    updateModelTransform(next);
}

void OcctView::setStandardOrientation(V3d_TypeOfOrientation orientation)
{
    if (m_view.IsNull())
        return;
    if (m_rotationMode == RotationMode::Camera) {
        m_view->SetProj(orientation);
        return;
    }

    const Handle(Graphic3d_Camera)& currentCamera = m_view->Camera();
    if (currentCamera.IsNull())
        return;
    Handle(Graphic3d_Camera) fixedCamera = new Graphic3d_Camera(currentCamera);
    m_view->SetProj(orientation);
    Handle(Graphic3d_Camera) presetCamera = new Graphic3d_Camera(m_view->Camera());
    m_view->SetCamera(fixedCamera);

    // 固定相机下设定标准方位:先用临时相机求出预设方向的取景,再计算
    // “当前相机→预设相机”的相对旋转矩阵并作为模型变换应用,视觉上模型
    // 转到预设方位、相机保持不动(中心仍为 m_modelCenter)。
    const gp_Mat presetFrame(presetCamera->SideRight().XYZ(),
                             presetCamera->OrthogonalizedUp().XYZ(),
                             presetCamera->Direction().XYZ());
    const gp_Mat fixedFrame(fixedCamera->SideRight().XYZ(),
                            fixedCamera->OrthogonalizedUp().XYZ(),
                            fixedCamera->Direction().XYZ());
    const gp_Quaternion rotation(fixedFrame * presetFrame.Transposed());
    const gp_Pnt sourceCenter =
        m_hasModelCenter ? m_modelCenter : gp_Pnt(0.0, 0.0, 0.0);
    const gp_Pnt visualCenter = sourceCenter.Transformed(m_modelTransform);
    const gp_Vec rotatedCenter = rotation * gp_Vec(sourceCenter.XYZ());
    const gp_Pnt rotatedCenterPoint(rotatedCenter.XYZ());
    gp_Trsf transform;
    transform.SetTransformation(rotation,
                                gp_Vec(rotatedCenterPoint, visualCenter));
    updateModelTransform(transform);
    m_view->Invalidate();
}

void OcctView::setFaceSelectionEnabled(bool on)
{
    m_faceSelection = on;
    if (m_context.IsNull()) return;
    m_context->Deactivate();
    if (on)
        setSelectionMode(m_selMode); // 按当前模式激活拾取
    else if (!m_viewCube.IsNull())
        m_context->Activate(m_viewCube, 0);
}

void OcctView::setSelectionMode(SelMode mode)
{
    m_selMode = mode;
    if (m_context.IsNull() || !m_faceSelection) return;

    // 选择模式 -> 激活的 OCC 拓扑类型。Unit 模式仍按面拾取,所属 unit 由 MainWindow 解析。
    TopAbs_ShapeEnum topo = TopAbs_FACE;
    switch (mode) {
        case SelMode::Edge: topo = TopAbs_EDGE; break;
        case SelMode::Wire: topo = TopAbs_WIRE; break;
        case SelMode::Face:
        case SelMode::Unit: topo = TopAbs_FACE; break;
    }

    m_context->Deactivate();
    AIS_ListOfInteractive objs;
    m_context->DisplayedObjects(objs);
    for (AIS_ListOfInteractive::Iterator it(objs); it.More(); it.Next()) {
        if (it.Value() == m_viewCube) {
            m_context->Activate(m_viewCube, 0);
            continue;
        }
        if (!Handle(oap::app::presentation::ToolpathSegmentsObject)::DownCast(
                 it.Value()).IsNull()) {
            m_context->Activate(it.Value(), 0);
            continue;
        }
        if (!Handle(AIS_Shape)::DownCast(it.Value()).IsNull())
            m_context->Activate(it.Value(), AIS_Shape::SelectionMode(topo));
    }
}

void OcctView::setBackgroundGray(double level)
{
    // 钳制到 [0,1]，避免异常的滑块取值产生无效颜色。
    if (level < kGrayMin) level = kGrayMin;
    if (level > kGrayMax) level = kGrayMax;
    m_grayLevel = level;

    if (m_view.IsNull())
        return; // 已记录在 m_grayLevel 中；待 initViewer() 运行后应用

    Quantity_Color color(level, level, level, Quantity_TOC_sRGB);
    if (!m_viewer.IsNull())
        m_viewer->SetDefaultBackgroundColor(color);
    m_view->SetBackgroundColor(color);
    m_view->Redraw();
}

void OcctView::showEvent(QShowEvent*)
{
    initViewer();
}

void OcctView::resizeEvent(QResizeEvent*)
{
    if (!m_view.IsNull()) {
        m_view->MustBeResized();
    }
}

void OcctView::paintEvent(QPaintEvent*)
{
    Handle(oap::app::presentation::ToolpathSegmentsObject) source =
        Handle(oap::app::presentation::ToolpathSegmentsObject)::DownCast(
            m_hoveredToolpathObject);
    if (!source.IsNull() && !m_context.IsNull() &&
        (!m_context->IsDisplayed(source) ||
         !source->isSegmentVisible(m_hoveredToolpathSegment))) {
        clearToolpathHover();
    }
    if (!m_view.IsNull())
        m_view->Redraw();
}

void OcctView::leaveEvent(QEvent* event)
{
    clearToolpathHover();
    if (!m_view.IsNull())
        m_view->Redraw();
    QWidget::leaveEvent(event);
}

void OcctView::keyPressEvent(QKeyEvent* e)
{
    if (e && e->key() == Qt::Key_Escape) {
        emit escapePressed();
        e->accept();
        return;
    }
    QWidget::keyPressEvent(e);
}

QPoint OcctView::toPixels(const QPoint& logical) const
{
    // Qt 鼠标坐标是逻辑像素;OCCT 视图绑定的是 HWND 物理像素表面。
    // 二者相差 devicePixelRatio(Windows 缩放 125% -> 1.25),不换算则拾取点偏移。
    const double r = devicePixelRatioF();
    return QPoint(int(logical.x() * r + kRoundBias), int(logical.y() * r + kRoundBias));
}

TopAbs_ShapeEnum OcctView::activeSelectionShapeType() const
{
    switch (m_selMode) {
        case SelMode::Edge: return TopAbs_EDGE;
        case SelMode::Wire: return TopAbs_WIRE;
        case SelMode::Face:
        case SelMode::Unit: return TopAbs_FACE;
    }
    return TopAbs_FACE;
}

std::vector<TopoDS_Shape> OcctView::selectedShapesOfType(TopAbs_ShapeEnum type) const
{
    std::vector<TopoDS_Shape> shapes;
    if (m_context.IsNull()) return shapes;
    for (m_context->InitSelected(); m_context->MoreSelected(); m_context->NextSelected()) {
        Handle(StdSelect_BRepOwner) owner =
            Handle(StdSelect_BRepOwner)::DownCast(m_context->SelectedOwner());
        if (!owner.IsNull() && owner->HasShape() && owner->Shape().ShapeType() == type)
            shapes.push_back(owner->Shape());
    }
    return shapes;
}

void OcctView::setMouseButton(MouseAction action, Qt::MouseButton button)
{
    switch (action) {
        case MouseAction::Select: m_selectButton = button; break;
        case MouseAction::BoxSelect: m_boxSelectButton = button; break;
        case MouseAction::Rotate: m_rotateButton = button; break;
        case MouseAction::Pan: m_panButton = button; break;
        case MouseAction::ContextMenu: m_contextButton = button; break;
    }
}

Qt::MouseButton OcctView::mouseButton(MouseAction action) const
{
    switch (action) {
        case MouseAction::Select: return m_selectButton;
        case MouseAction::BoxSelect: return m_boxSelectButton;
        case MouseAction::Rotate: return m_rotateButton;
        case MouseAction::Pan: return m_panButton;
        case MouseAction::ContextMenu: return m_contextButton;
    }
    return Qt::NoButton;
}

void OcctView::mousePressEvent(QMouseEvent* e)
{
    m_lastPos = e->pos();
    m_pressPos = e->pos();
    m_boxSelecting = false;
    if (e->button() == m_rotateButton && !m_view.IsNull()) {
        if (!m_context.IsNull())
            m_context->ClearDetected(Standard_False);
        clearToolpathHover();
        m_view->Redraw();
        if (m_rotationMode == RotationMode::Camera) {
            const QPoint p = toPixels(e->pos());
            m_view->StartRotation(p.x(), p.y());
        }
    }
}

void OcctView::contextMenuEvent(QContextMenuEvent*)
{
    // 吞掉 Qt 的 contextMenu 事件;右键菜单由 mouseReleaseEvent 中的“单击 vs 拖动”
    // 判定触发,从而让右键拖动仍能用于平移。
}

void OcctView::mouseReleaseEvent(QMouseEvent* e)
{
    if (m_view.IsNull() || m_context.IsNull())
        return;

    // 右键未拖动 -> 弹出 3D 右键菜单(拖动则在 mouseMoveEvent 中已用于平移)。
    if (e->button() == m_contextButton) {
        if ((e->pos() - m_pressPos).manhattanLength() <= kClickDragThresholdPx)
        {
            emit contextMenuRequested(e->globalPosition().toPoint());
            return;
        }
    }

    if (e->button() != m_selectButton && e->button() != m_boxSelectButton)
        return;
    if (m_boxSelecting) {
        if (m_selectionFrame) m_selectionFrame->hide();
        m_boxSelecting = false;
        const QRect rect(m_pressPos, e->pos());
        const QRect normalized = rect.normalized();
        if (normalized.width() > kClickDragThresholdPx &&
            normalized.height() > kClickDragThresholdPx) {
            const QPoint topLeft = toPixels(normalized.topLeft());
            const QPoint bottomRight = toPixels(normalized.bottomRight());
            const bool additive = e->modifiers().testFlag(Qt::ControlModifier);
            m_context->SelectRectangle(
                Graphic3d_Vec2i(topLeft.x(), topLeft.y()),
                Graphic3d_Vec2i(bottomRight.x(), bottomRight.y()), m_view,
                additive ? AIS_SelectionScheme_XOR : AIS_SelectionScheme_Replace);
            const TopAbs_ShapeEnum type = activeSelectionShapeType();
            emit entitiesBoxPicked(selectedShapesOfType(type), int(type));
            m_view->Redraw();
        }
        return;
    }
    if (e->button() != m_selectButton) return;
    if ((e->pos() - m_pressPos).manhattanLength() > kClickDragThresholdPx)
        return; // 这是拖动，而非单击

    const QPoint p = toPixels(e->pos());
    // 按住 Ctrl = 多选:对命中面做异或切换(已选则取消、未选则追加),其余选择不动;
    // 否则替换式单选(清空旧选)。两种方案均用 OCCT 原生选择驱动 3D 高亮。
    // 按住 Shift = 整特征累加选择，由 MainWindow 展开统一结构实例。
    const bool additive = (e->modifiers() & Qt::ControlModifier) != 0;
    const bool wholeFeature = (e->modifiers() & Qt::ShiftModifier) != 0;
    m_context->MoveTo(p.x(), p.y(), m_view, Standard_True);
    Handle(AIS_ViewCubeOwner) cubeOwner =
        Handle(AIS_ViewCubeOwner)::DownCast(m_context->DetectedOwner());
    if (!cubeOwner.IsNull() && !m_viewCube.IsNull()) {
        if (m_rotationMode == RotationMode::Model) {
            V3d_TypeOfOrientation orientation = cubeOwner->MainOrientation();
            if (MoldViewCube* cube =
                    dynamic_cast<MoldViewCube*>(m_viewCube.get()))
                orientation = cube->worldOrientation(orientation);
            setStandardOrientation(orientation);
        } else {
            m_viewCube->HandleClick(cubeOwner);
        }
        m_view->Redraw();
        return;
    }
    Handle(oap::app::presentation::ToolpathSegmentOwner) toolpathOwner =
        Handle(oap::app::presentation::ToolpathSegmentOwner)::DownCast(
            m_context->DetectedOwner());
    if (!toolpathOwner.IsNull()) {
        Handle(oap::app::presentation::ToolpathSegmentsObject) object =
            Handle(oap::app::presentation::ToolpathSegmentsObject)::DownCast(
                toolpathOwner->Selectable());
        const auto* segment = object.IsNull()
            ? nullptr : object->segment(toolpathOwner->segmentIndex());
        m_context->SelectDetected(additive ? AIS_SelectionScheme_XOR
                                           : AIS_SelectionScheme_Replace);
        if (segment) {
            emit toolpathSegmentPicked(
                toolpathSegmentDescription(*segment),
                QString::fromStdString(segment->identity.operationId),
                segment->identity.moveIndex);
        }
        m_view->Redraw();
        return;
    }
    clearToolpathHover();
    m_context->SelectDetected(additive ? AIS_SelectionScheme_XOR
                                       : AIS_SelectionScheme_Replace);
    // 取刚命中的 owner(而非遍历整个选择集):多选时只上报本次切换的那个面,
    // 由 MainWindow 同步到树镜像。命中为空(点中背景)时不做任何事。
    Handle(StdSelect_BRepOwner) owner =
        Handle(StdSelect_BRepOwner)::DownCast(m_context->DetectedOwner());
    if (!owner.IsNull() && owner->HasShape()) {
        const TopoDS_Shape& s = owner->Shape();
        const TopAbs_ShapeEnum t = s.ShapeType();
        // 拾取命中 FACE / EDGE / WIRE 时上报;MainWindow 据此解析 id。
        if (t == TopAbs_FACE || t == TopAbs_EDGE || t == TopAbs_WIRE)
            emit entityPicked(s, (int)t, additive, wholeFeature);
    } else if (!additive) {
        emit selectionCleared();
    }
    m_view->Redraw();
}

void OcctView::mouseMoveEvent(QMouseEvent* e)
{
    if (m_view.IsNull())
        return;
    const QPoint p = toPixels(e->pos());
    if (e->buttons() & m_rotateButton) {
        if (m_rotationMode == RotationMode::Model) {
            const QPoint last = toPixels(m_lastPos);
            rotateModelByPixels(p.x() - last.x(), p.y() - last.y());
        } else {
            m_view->Rotation(p.x(), p.y());
        }
    }
    else if (e->buttons() & m_panButton) {
        const QPoint last = toPixels(m_lastPos);
        m_view->Pan(p.x() - last.x(), last.y() - p.y());
    }
    else if ((e->buttons() & m_boxSelectButton) && m_faceSelection && !m_context.IsNull()) {
        if ((e->pos() - m_pressPos).manhattanLength() > kClickDragThresholdPx) {
            if (!m_selectionFrame) m_selectionFrame = new SelectionFrameWindow(this);
            m_boxSelecting = true;
            static_cast<SelectionFrameWindow*>(m_selectionFrame)->showFrame(
                this, QRect(m_pressPos, e->pos()).normalized());
        }
    }
    else if (m_faceSelection && !m_context.IsNull()) {
        // 无按键移动:按当前选择模式(Face/Edge/Wire/Unit→面)做悬停动态高亮。
        // MoveTo 命中即高亮对应子形状,鼠标移开(未命中)时 OCCT 自动清除上一次高亮。
        // Detection and the lightweight hover presentation are published in
        // one redraw, avoiding two viewer updates for every mouse move.
        m_context->MoveTo(p.x(), p.y(), m_view, Standard_False);
        updateToolpathHover(e->globalPosition().toPoint());
        m_view->Redraw();
    }
    m_lastPos = e->pos();
}

void OcctView::clearToolpathHover()
{
    Handle(oap::app::presentation::ToolpathHoverObject) hover =
        Handle(oap::app::presentation::ToolpathHoverObject)::DownCast(
            m_toolpathHoverPresentation);
    if (!hover.IsNull()) {
        hover->clearSegment();
        if (!m_context.IsNull() && m_context->IsDisplayed(hover))
            m_context->Erase(hover, Standard_False);
    }
    m_hoveredToolpathObject.Nullify();
    m_hoveredToolpathSegment = -1;
    QToolTip::hideText();
}

bool OcctView::updateToolpathHover(const QPoint& globalPos)
{
    if (m_context.IsNull()) {
        clearToolpathHover();
        return false;
    }
    Handle(oap::app::presentation::ToolpathSegmentOwner) owner =
        Handle(oap::app::presentation::ToolpathSegmentOwner)::DownCast(
            m_context->DetectedOwner());
    Handle(oap::app::presentation::ToolpathSegmentsObject) object;
    if (!owner.IsNull()) {
        object = Handle(oap::app::presentation::ToolpathSegmentsObject)::DownCast(
            owner->Selectable());
    }
    const int segmentIndex = owner.IsNull() ? -1 : owner->segmentIndex();
    if (object.IsNull()) {
        if (!m_hoveredToolpathObject.IsNull())
            clearToolpathHover();
        return false;
    }
    if (m_hoveredToolpathObject == object &&
        m_hoveredToolpathSegment == segmentIndex) {
        return true;
    }

    const auto* segment = object->segment(segmentIndex);
    if (!segment || !object->isSegmentVisible(segmentIndex)) {
        clearToolpathHover();
        return false;
    }

    Handle(oap::app::presentation::ToolpathHoverObject) hover =
        Handle(oap::app::presentation::ToolpathHoverObject)::DownCast(
            m_toolpathHoverPresentation);
    if (hover.IsNull()) {
        hover = new oap::app::presentation::ToolpathHoverObject();
        m_toolpathHoverPresentation = hover;
    }
    m_hoveredToolpathObject = object;
    m_hoveredToolpathSegment = segmentIndex;
    hover->SetLocalTransformation(object->LocalTransformation());
    hover->setSegment(*segment, object->style());
    if (m_context->IsDisplayed(hover)) {
        m_context->Redisplay(hover, Standard_False);
    } else {
        m_context->Display(hover, 0, -1, Standard_False);
    }
    QToolTip::showText(globalPos, toolpathSegmentDescription(*segment), this);
    return true;
}

void OcctView::wheelEvent(QWheelEvent* e)
{
    if (m_view.IsNull())
        return;
    const int d = e->angleDelta().y();
    m_view->SetZoom(d > 0 ? kZoomInFactor : kZoomOutFactor);
}

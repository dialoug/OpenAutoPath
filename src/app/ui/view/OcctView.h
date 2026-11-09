// 一个 QWidget，直接通过其原生窗口句柄承载 OpenCASCADE 的 V3d_View。
#pragma once

#include <QWidget>
#include <QPoint>
#include <QRect>

#include <V3d_Viewer.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_TypeOfOrientation.hxx>

#include <vector>

#include "analysis/MoldCoordinateFrame.h"

class AIS_ViewCube;
class QKeyEvent;

// OCCT 视图控件：OpenCASCADE 3D 视图的 Qt 封装。
class OcctView : public QWidget
{
    Q_OBJECT
public:
    explicit OcctView(QWidget* parent = nullptr);

    // 返回空的 paint engine，告诉 Qt 该控件由外部渲染器绘制。
    QPaintEngine* paintEngine() const override { return nullptr; }

    const Handle(AIS_InteractiveContext)& context() const { return m_context; }
    const Handle(V3d_View)& view() const { return m_view; }

    // 构建绑定到该控件 HWND 的 OCCT driver/viewer/view。在控件拥有有效的
    // 原生句柄后调用一次（顶层窗口显示之后）。
    void initViewer();
    void showViewCube();

    // 将持久视图立方体绑定到模具工作坐标系；STEP 几何本身不变换。
    void setMoldCoordinateFrame(const oap::MoldCoordinateFrame& frame);
    void clearMoldCoordinateFrame();

    enum class RotationMode { Camera, Model };
    void setRotationMode(RotationMode mode);
    RotationMode rotationMode() const { return m_rotationMode; }
    void setModelRotationVerticalInverted(bool inverted)
    {
        m_modelRotationVerticalInverted = inverted;
    }
    bool modelRotationVerticalInverted() const
    {
        return m_modelRotationVerticalInverted;
    }
    void setModelCenter(double x, double y, double z);
    void clearModelCenter();
    void resetModelTransform();
    const gp_Trsf& modelTransform() const { return m_modelTransform; }
    void setStandardOrientation(V3d_TypeOfOrientation orientation);

    // 将查看器背景设为中性灰。level 取值 [0,1]：0 = 黑，1 = 白。
    void setBackgroundGray(double level);
    double backgroundGray() const { return m_grayLevel; }

    // 启用面级选择，使左键拾取返回所属的 TopoDS_Face。
    void setFaceSelectionEnabled(bool on);

    // 3D 选择模式(需求 §11.3)。Unit 模式拾取面后由 MainWindow 解析所属 unit。
    enum class SelMode { Face, Edge, Wire, Unit };
    void setSelectionMode(SelMode mode); // 切换当前拾取优先命中的对象类型
    SelMode selectionMode() const { return m_selMode; }

    // 单击和拖动动作允许共用按键：默认左键单击拾取、左键拖动框选，
    // 右键单击菜单、右键拖动平移。
    enum class MouseAction { Select, BoxSelect, Rotate, Pan, ContextMenu };
    void setMouseButton(MouseAction action, Qt::MouseButton button);
    Qt::MouseButton mouseButton(MouseAction action) const;

signals:
    // 当原生 OCCT 上下文与视图均已创建完成后发出。
    void viewerInitialized();

    // 固定相机旋转模型时通知场景中的全部显示对象同步变换。
    void modelTransformChanged();

    // 拾取命中某个子形状时发出。shapeType 为命中的拓扑类型(FACE/EDGE/WIRE)。
    // additive=true 表示拾取时按住 Ctrl(多选/切换):该面在选择集合中被异或切换,
    // 而非清空后单选。MainWindow 通过 StepIndexer 将其解析为对应 id。
    void entityPicked(const TopoDS_Shape& shape, int shapeType, bool additive,
                      bool wholeFeature);
    void selectionCleared();
    // shapes 是 OCCT 应用 Replace/XOR 选择规则后的完整选择集，接收者无需再次
    // 解释 Ctrl 等输入状态。
    void entitiesBoxPicked(const std::vector<TopoDS_Shape>& shapes,
                           int shapeType);

    // 右键单击(非拖动)时发出,携带全局坐标,供 MainWindow 弹出 3D 右键菜单。
    void contextMenuRequested(const QPoint& globalPos);

    // 三维视图获得焦点时按下 Esc，请求上层清空累计的孔段高亮。
    void escapePressed();

    // 普通刀路线段点击：除状态栏描述外同时上报工序和移动索引，供 CAM
    // 工序树反向定位；不改变模型面的选择集合。
    void toolpathSegmentPicked(
        const QString& description,
        const QString& operationId,
        int moveIndex);

protected:
    void showEvent(QShowEvent*) override;
    void resizeEvent(QResizeEvent*) override;
    void paintEvent(QPaintEvent*) override;
    void leaveEvent(QEvent*) override;
    void keyPressEvent(QKeyEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void contextMenuEvent(QContextMenuEvent* e) override;

private:
    // 逻辑像素(Qt 坐标)-> 物理像素(OCCT/HWND 坐标)。Windows 缩放(125%/150%)下
    // devicePixelRatio != 1,若不换算,拾取点与渲染像素会随距离逐渐偏移。
    QPoint toPixels(const QPoint& logical) const;
    TopAbs_ShapeEnum activeSelectionShapeType() const;
    std::vector<TopoDS_Shape> selectedShapesOfType(TopAbs_ShapeEnum type) const;
    void rotateModelByPixels(int dx, int dy);
    void updateModelTransform(const gp_Trsf& transform);
    void syncViewCubeTransform();
    void clearToolpathHover();
    bool updateToolpathHover(const QPoint& globalPos);

    Handle(V3d_Viewer)             m_viewer;
    Handle(V3d_View)               m_view;
    Handle(AIS_InteractiveContext) m_context;
    Handle(AIS_ViewCube)           m_viewCube;
    gp_Pnt                         m_viewCubeVisualCenter;
    gp_Pnt                         m_modelCenter;
    gp_Trsf                        m_modelTransform;
    QPoint                         m_lastPos;
    QPoint                         m_pressPos;       // 用于区分单击与拖动
    QWidget*                       m_selectionFrame = nullptr;
    bool                           m_boxSelecting = false;
    bool                           m_faceSelection = false;
    bool                           m_hasViewCubeVisualCenter = false;
    bool                           m_hasModelCenter = false;
    bool                           m_modelRotationVerticalInverted = false;
    RotationMode                   m_rotationMode = RotationMode::Camera;
    SelMode                        m_selMode = SelMode::Face; // 当前 3D 选择模式
    Qt::MouseButton                m_selectButton = Qt::LeftButton;
    Qt::MouseButton                m_boxSelectButton = Qt::LeftButton;
    Qt::MouseButton                m_rotateButton = Qt::MiddleButton;
    Qt::MouseButton                m_panButton = Qt::RightButton;
    Qt::MouseButton                m_contextButton = Qt::RightButton;
    Handle(AIS_InteractiveObject) m_hoveredToolpathObject;
    Handle(AIS_InteractiveObject) m_toolpathHoverPresentation;
    int                            m_hoveredToolpathSegment = -1;
    double                         m_grayLevel = 0.30; // 与初始的 GRAY30 背景一致
};

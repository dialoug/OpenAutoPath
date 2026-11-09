// 腔仿真准备与模态 OpenGL UI 共享的值类型。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QString>
#include <Qt>
#include <Poly_Triangulation.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/simulation/PocketCamSimulationScene.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"

namespace oap::toolpath {
class MultiDexelStock;
}

namespace oap::app {

struct IpwTransientFrameSource;

// 模态腔仿真窗口的旋转模式：Camera 旋转相机，Model 旋转模型。
enum class PocketCamRotationMode {
    Camera,
    Model,
};

// 鼠标交互选项：旋转/平移按钮与垂直轴方向是否反转。
struct PocketCamInteractionOptions {
    PocketCamRotationMode rotationMode = PocketCamRotationMode::Camera;
    bool modelRotationVerticalInverted = false;
    Qt::MouseButton rotateButton = Qt::LeftButton;
    Qt::MouseButton panButton = Qt::RightButton;
};

// 视觉验收视图参数：相机朝向、视域与曲面边界网格化偏差统计。
struct PocketCamVisualAcceptanceView {
    bool valid = false;
    gp_Pnt center;
    gp_Dir eyeDirection{0.0, 0.0, 1.0};
    gp_Dir up{0.0, 1.0, 0.0};
    double halfHeight = 1.0;
    double linearDeflection = 0.0;
    double angularDeflection = 0.0;
    bool boundaryDeviationMeasured = false;
    std::size_t boundaryEdgeCount = 0u;
    std::size_t boundarySegmentCount = 0u;
    std::size_t boundarySampleCount = 0u;
    std::size_t curvedBoundaryEdgeCount = 0u;
    std::size_t curvedBoundarySampleCount = 0u;
    double maximumCurveToPolylineDeviation = 0.0;
    double rmsCurveToPolylineDeviation = 0.0;
    double maximumPolylineToCurveDeviation = 0.0;
    double maximumCurvedBoundaryDeviation = 0.0;
    double rmsCurvedBoundaryDeviation = 0.0;
    QString operationId;
};

// 腔仿真输入：毛坯/目标/工序/参数与选项，以及预先计算好的
// 首帧与各工序起点的 IPW 源，供 OpenGL 显示与回退/跳转复用。
struct PocketCamSimulatorInput {
    QString title;
    TopoDS_Shape stock;
    TopoDS_Shape target;
    std::vector<toolpath::PocketToolpathOperation> operations;
    // Project playback and transient frames share the full path without copies.
    std::shared_ptr<const std::vector<toolpath::PocketToolpathOperation>> authoritativeOperations;
    toolpath::ToolpathParams params;
    toolpath::PocketCamSimulationOptions options;
    // 首个槽腔工序之前的统一 IPW 材料真值。OpenGL 首帧
    // 直接上传该状态，避免为显示重新栅格化原始形体。
    std::shared_ptr<const toolpath::MultiDexelStock> initialIpwStock;
    // 各 operation 开始前的不可变 IPW 派生源。回退/跳转时在后台
    // 恢复目标工序基线，不需要从第一段刀路重放。
    std::vector<std::shared_ptr<const IpwTransientFrameSource>>
        operationInitialIpwSources;
    std::shared_ptr<const toolpath::MultiDexelStock> finalIpwStock;
    Handle(Poly_Triangulation) finalIpwDisplayMesh;
    std::string finalIpwDisplayMeshFingerprint;
    QString visualAcceptanceDirectory;
    PocketCamVisualAcceptanceView visualAcceptanceView;
    PocketCamInteractionOptions interaction;
};

} // namespace oap::app

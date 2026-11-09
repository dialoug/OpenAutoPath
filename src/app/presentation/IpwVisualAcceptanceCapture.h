// 用于可选的 IPW/OpenGL 视觉验收证据的 OCCT 视口捕获。
// 模块:oap_app_presentation。提供两阶段截图:整视图快照 + 仅 IPW 的 ROI
// 特写,供自动化验收脚本留存“最终视图”与“槽腔特写”证据。
#pragma once

#include <QString>
#include <QStringList>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Triangulation.hxx>
#include <V3d_View.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace oap::app::presentation {

// ROI 特写相机参数:正对视向、中心与视口半高;valid=false 表示不导出特写。
struct IpwAcceptanceCamera {
    bool valid = false;
    gp_Pnt center;
    gp_Dir eyeDirection{0.0, 0.0, 1.0};
    gp_Dir up{0.0, 1.0, 0.0};
    double halfHeight = 1.0;
};

// 捕获请求:指定输出目录、视图/上下文、待特写的 IPW 呈现与相机。
struct IpwVisualAcceptanceCaptureRequest {
    QString directory;
    Handle(V3d_View) view;
    Handle(AIS_InteractiveContext) context;
    Handle(AIS_Triangulation) ipwPresentation;
    IpwAcceptanceCamera camera;
    int imageSize = 1024;
};

// 捕获结果:各阶段是否成功 + 警告列表(不抛异常,失败以 warning 上报)。
struct IpwVisualAcceptanceCaptureResult {
    QString directory;
    bool finalViewportSaved = false;
    bool roiRequested = false;
    bool roiSaved = false;
    QStringList warnings;
};

// 静态工具类:capture() 在调用线程同步执行截图,过程中会临时隐藏其它对象、
// 切换正交相机,结束后恢复原相机与对象显示状态。
class IpwVisualAcceptanceCapture {
public:
    static IpwVisualAcceptanceCaptureResult capture(
        const IpwVisualAcceptanceCaptureRequest& request);
};

} // namespace oap::app::presentation

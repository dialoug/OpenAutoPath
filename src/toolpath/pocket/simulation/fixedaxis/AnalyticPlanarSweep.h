// 固定轴解析二维刀具扫掠原型。输出 Face 保留 Line/Circle 曲线，供分层
// BRep 后端在接入生产路径前独立验证面积、拓扑和数值稳定性。
//
// 位于 oap_toolpath_pocket 的 fixedaxis/ 模块私有实现目录（内部细节），
// 由 AnalyticFixedAxisLayeredBRepBuilder 的解析快路径调用：为点/直线/
// 圆弧/整圆原语直接构造解析扫掠面，替代逐点折线的近似偏置。
#pragma once

#include <string>

#include <TopoDS_Face.hxx>
#include <gp_Ax3.hxx>

#include "toolpath/core/ToolpathPrimitives.h"

namespace oap::toolpath {

enum class AnalyticPlanarSweepKind {
    Point,
    Linear,
    CircularArc,
    FullCircle,
};

// 解析平面扫掠结果：扫掠面、种类、路径长与面积。
struct AnalyticPlanarSweepResult {
    TopoDS_Face face;
    std::string error;
    AnalyticPlanarSweepKind kind =
        AnalyticPlanarSweepKind::Point;
    double pathLength = 0.0;
    double area = 0.0;
    double expectedArea = 0.0;
    double maximumInputDeviation = 0.0;
    int lineEdgeCount = 0;
    int circleEdgeCount = 0;
    int unsupportedEdgeCount = 0;
    bool pointFallback = false;

    bool ok() const noexcept
    {
        return error.empty() && !face.IsNull();
    }
};

// 解析平面扫掠：为点/线/圆弧原语直接构造解析扫掠面。
class AnalyticPlanarSweep {
public:
    // frame 的 Location 定义输出平面，X/YDirection 定义局部二维坐标，
    // Direction 定义圆弧正向。带轴向变化的圆弧由后续分层阶段处理。
    AnalyticPlanarSweepResult build(
        const gp_Ax3& frame,
        const ToolpathPrimitive& primitive,
        double cutterRadius,
        double tolerance = 1.0e-7,
        double inputConsistencyTolerance = 0.0) const;
};

} // namespace oap::toolpath

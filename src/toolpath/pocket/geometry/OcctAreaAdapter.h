// OcctAreaAdapter —— OCCT 几何与二维刀路之间的边界转换器。
// 所有二维坐标均使用传入的 gp_Ax3 局部坐标系表示。
#pragma once

#include <vector>

#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <TopoDS_Wire.hxx>

namespace oap::toolpath {

// 二维环：平面点序列（Clipper 路径别名）。
struct Loop2d {
    TopoDS_Wire wire;             // 保留原始拓扑，供诊断使用。
    std::vector<gp_Pnt2d> points; // 隐式闭合，不重复保存末端点。
    bool closed = false;
    // 该环由开放截面折线加“末点 -> 首点”的虚拟出口边封闭。
    bool virtualOpenBoundary = false;
    double signedArea = 0.0;
};

// 无状态纯几何工具类：只有静态方法，不持有任何模型或区域数据。
// 调用方(PocketRegionBuilder、各切片/图案模块)负责传入一致的 frame，
// 所有输出二维环都隐式闭合、按 frame 局部坐标表达。
class OcctAreaAdapter {
public:
    // 除浮点误差外，以下两个函数互为逆变换。
    static gp_Pnt2d projectPoint(const gp_Pnt& point, const gp_Ax3& frame);
    static gp_Pnt pointFrom2d(const gp_Pnt2d& point, const gp_Ax3& frame);

    // 将 OCCT 线框转换为 frame 坐标系下的有序二维多边形。
    // 返回环为隐式闭合形式，会移除重复的末端点。
    static Loop2d wireToLoop2d(const TopoDS_Wire& wire,
                               const gp_Ax3& frame,
                               double deflection = 0.01,
                               double tolerance = 1.0e-7);

    // 鞋带公式计算二维多边形有符号面积；正值 = 逆时针(在 frame 平面内)。
    static double signedArea(const std::vector<gp_Pnt2d>& points);
    // 反转环的点序并把 signedArea 取负，用于统一外环/内环的绕向约定。
    static void reverseLoop(Loop2d& loop);
};

} // namespace oap::toolpath

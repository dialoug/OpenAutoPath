// 模具六面分类和多装夹规划共享的局部坐标框架。
// 属于 oap_analysis 的"模具坐标系"部分:由模具包围盒派生右手工作坐标系,
// 供 MoldSurfaceClassifier、孔加工方向判定等下游几何计算使用。
#pragma once

#include <array>

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include "Model.h"

namespace oap {

// 由完整模具包围盒派生出的右手工作坐标系。几何仍保留在 STEP 世界坐标中，
// 该结构只负责稳定、可逆的坐标映射和孔加工访问顺序。
struct MoldCoordinateFrame {
    gp_Pnt origin;
    gp_Dir xAxis{1.0, 0.0, 0.0};
    gp_Dir yAxis{0.0, 1.0, 0.0};
    gp_Dir zAxis{0.0, 0.0, 1.0};
    double length = 0.0;
    double width = 0.0;
    double thickness = 0.0;
    bool isValid = false;

    // 世界坐标 -> 局部坐标:以 origin 为原点的正交投影,返回局部系下的点。
    gp_Pnt worldToLocal(const gp_Pnt& world) const;
    // 局部坐标 -> 世界坐标:worldToLocal 的逆变换。
    gp_Pnt localToWorld(const gp_Pnt& local) const;

    // 局部 XY 平面的左下、左上、右下、右上四角。
    std::array<gp_Pnt, 4> planarCorners() const;

    // 以全部 unit 包围盒的并集构建坐标系;模型为空或包围盒无效时 isValid=false。
    static MoldCoordinateFrame fromModel(const Model& model);
    // 由单个包围盒构建:最长轴 -> 局部 x(length),次长轴 -> 局部 y(width),
    // 最短轴 -> 局部 z(thickness);包围盒退化(任一跨度 <= 0)时 isValid=false。
    static MoldCoordinateFrame fromBounds(const Bbox& bounds);
};

} // namespace oap

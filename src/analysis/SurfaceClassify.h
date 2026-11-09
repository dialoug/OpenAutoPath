// 面表面类型统一分类(SurfaceClassify) —— oap_analysis 模块的几何分类部分。
// 为 OCCT 的"解析面 vs 数值平面"差异提供一致的"有效平面"语义,供 STEP 索引
// (StepIndexer)、边界判定(detail/AabbBoundaryFace)与模具表面分类共用。
#pragma once

// OCCT 的 BRepAdaptor_Surface::GetType() 只报告底层曲面类型，不会把几何上
// 等价于平面的拉伸面、旋转面、Bezier/BSpline 面或偏移面自动归类为平面。
// 对这些非解析面做一次紧容差平面回归，让所有需要平面参数的识别流程
// 使用同一套“有效平面”语义。

#include <GeomAbs_SurfaceType.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pln.hxx>

namespace oap {

// 分类结果:type 为表面类型(非解析面经平面回归命中时提升为 GeomAbs_Plane,
// 并由 planeFromRegression 标记该提升);hasPlane / plane 给出有效平面参数。
struct SurfaceKind {
    GeomAbs_SurfaceType type = GeomAbs_OtherSurface;
    bool planeFromRegression = false;
    bool hasPlane = false;
    gp_Pln plane;
};

// 完整分类:返回表面类型与(可能经回归得到的)平面参数。
SurfaceKind classifySurface(const TopoDS_Face& face);
// 仅取有效表面类型(等价于 classifySurface(face).type)。
GeomAbs_SurfaceType effectiveSurfaceType(const TopoDS_Face& face);
// 若该面可视为平面则输出 plane 并返回 true;否则返回 false 且不改动 plane。
bool effectivePlane(const TopoDS_Face& face, gp_Pln& plane);

} // namespace oap

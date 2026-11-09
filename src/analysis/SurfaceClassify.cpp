// SurfaceClassify 实现 —— 解析面直接采用 OCCT 报告的类型;对几何上等价于平面
// 的拉伸 / 旋转 / Bezier / BSpline / 偏移面,用紧容差平面回归提升为 GeomAbs_Plane。
#include "SurfaceClassify.h"

#include <BRepAdaptor_Surface.hxx>
#include <ShapeAnalysis_CanonicalRecognition.hxx>
#include <Standard_Failure.hxx>

namespace oap {
namespace {

// 只有数值上可视作平面的非解析面才会回归。
constexpr double kPlanarRegressionTolerance = 1.0e-3;

} // namespace

// 分类主入口:解析面(plane/cylinder/...)直接取 OCCT 类型;非解析面走
// ShapeAnalysis_CanonicalRecognition 的平面回归,命中即按平面处理。
// 畸形面抛出的 Standard_Failure 被吞掉,返回"未知表面"而非让调用方崩溃。
SurfaceKind classifySurface(const TopoDS_Face& face)
{
    SurfaceKind result;
    try {
        BRepAdaptor_Surface surface(face);
        result.type = surface.GetType();

        switch (result.type) {
        case GeomAbs_Plane:
            result.hasPlane = true;
            result.plane = surface.Plane();
            break;
        case GeomAbs_SurfaceOfExtrusion:
        case GeomAbs_SurfaceOfRevolution:
        case GeomAbs_BezierSurface:
        case GeomAbs_BSplineSurface:
        case GeomAbs_OffsetSurface: {
            ShapeAnalysis_CanonicalRecognition canonical(face);
            gp_Pln plane;
            if (canonical.IsPlane(kPlanarRegressionTolerance, plane)) {
                result.type = GeomAbs_Plane;
                result.planeFromRegression = true;
                result.hasPlane = true;
                result.plane = plane;
            }
            break;
        }
        default:
            break;
        }
    } catch (const Standard_Failure&) {
        result.type = GeomAbs_OtherSurface;
        result.planeFromRegression = false;
        result.hasPlane = false;
    }
    return result;
}

// 便捷包装:只取有效表面类型(供不需要平面参数的场景使用)。
GeomAbs_SurfaceType effectiveSurfaceType(const TopoDS_Face& face)
{
    return classifySurface(face).type;
}

// 便捷包装:需要平面参数时使用,返回的平面与 classifySurface 的结论一致。
bool effectivePlane(const TopoDS_Face& face, gp_Pln& plane)
{
    const SurfaceKind kind = classifySurface(face);
    if (kind.hasPlane)
        plane = kind.plane;
    return kind.hasPlane;
}

} // namespace oap

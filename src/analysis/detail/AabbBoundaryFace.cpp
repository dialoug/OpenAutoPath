// detail/AabbBoundaryFace 实现 —— 基于"轴对齐平面极值盒"的公共边界面判定。
// 内部实现细节:为分析模块(如 MoldSurfaceClassifier)提供面是否位于实体 AABB
// 边界上的几何判定,不直接暴露给 UI 层。
#include "analysis/AabbBoundaryFace.h"

#include "analysis/SurfaceClassify.h"

#include <BRepBndLib.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <cmath>

namespace oap {
namespace {
constexpr double kAngularTolerance = 0.01;
constexpr double kDistanceTolerance = 1.0e-1;

bool isAxisAlignedPlane(const gp_Pln& plane)
{
    const gp_Dir normal = plane.Axis().Direction();
    return std::abs(std::abs(normal.X()) - 1.0) < kAngularTolerance ||
           std::abs(std::abs(normal.Y()) - 1.0) < kAngularTolerance ||
           std::abs(std::abs(normal.Z()) - 1.0) < kAngularTolerance;
}

// 消除 Bnd_Box 内置的默认 gap,避免公差污染后续的距离比较。
void tightenBox(Bnd_Box& box)
{
    if (!box.IsVoid())
        box.SetGap(0.0);
}
}

// 只累加"有效平面且近似轴对齐"的面的极值,生成轴对齐平面极值盒。
// 思路:非平面凸起(如曲面加强筋)会拉偏主外表面的参考边界,故先过滤;
// 没有符合条件的面时返回空盒,由调用方回退到整体 AABB。
Bnd_Box buildAxisAlignedPlanarBox(const TopoDS_Shape& shape)
{
    Bnd_Box planarBox;
    if (shape.IsNull())
        return planarBox;

    for (TopExp_Explorer explorer(shape, TopAbs_FACE);
         explorer.More(); explorer.Next()) {
        try {
            const TopoDS_Face face =
                TopoDS::Face(explorer.Current());
            gp_Pln plane;
            if (!effectivePlane(face, plane) || !isAxisAlignedPlane(plane))
                continue;

            Bnd_Box faceBox;
            BRepBndLib::AddOptimal(
                face, faceBox,
                Standard_False, Standard_False);
            addAxisAlignedPlanarFaceBox(plane, faceBox, planarBox);
        } catch (const Standard_Failure&) {
        }
    }
    tightenBox(planarBox);
    return planarBox;
}

void addAxisAlignedPlanarFaceBox(const gp_Pln& plane,
                                const Bnd_Box& faceBox,
                                Bnd_Box& referenceBox)
{
    if (faceBox.IsVoid() || !isAxisAlignedPlane(plane)) return;
    referenceBox.Add(faceBox);
    tightenBox(referenceBox);
}

// 判定 face 是否落在 solidBox 的某个 AABB 边界面上:要求该面平面近似轴对齐、
// 平面位置贴近盒的某个极值(min/max),且在另外两个轴方向与盒范围相交。
// 三个候选轴中任一个满足即认为在边界上;也接受调用方预计算的面盒以省一次求盒。
bool isFaceOnAabbBoundary(const TopoDS_Face& face,
                          const Bnd_Box& solidBox,
                          const Bnd_Box* suppliedFaceBox)
{
    if (solidBox.IsVoid()) return false;
    gp_Pln plane;
    if (!effectivePlane(face, plane)) return false;

    Bnd_Box localFaceBox;
    if (!suppliedFaceBox) {
        BRepBndLib::AddOptimal(face, localFaceBox, Standard_False, Standard_False);
    } else {
        localFaceBox = *suppliedFaceBox;
    }
    return isFaceOnAabbBoundary(plane, solidBox, localFaceBox);
}

bool isFaceOnAabbBoundary(const gp_Pln& plane,
                         const Bnd_Box& solidBox,
                         const Bnd_Box& faceBox)
{
    if (solidBox.IsVoid() || faceBox.IsVoid()) return false;
    Bnd_Box localFaceBox = faceBox;
    tightenBox(localFaceBox);

    Standard_Real sx0, sy0, sz0, sx1, sy1, sz1;
    Standard_Real fx0, fy0, fz0, fx1, fy1, fz1;
    solidBox.Get(sx0, sy0, sz0, sx1, sy1, sz1);
    localFaceBox.Get(fx0, fy0, fz0, fx1, fy1, fz1);

    const gp_Pnt origin = plane.Location();
    const gp_Dir normal = plane.Axis().Direction();
    struct Axis {
        double normal;
        double plane;
        double boxMin;
        double boxMax;
        double faceOtherMin1;
        double faceOtherMax1;
        double boxOtherMin1;
        double boxOtherMax1;
        double faceOtherMin2;
        double faceOtherMax2;
        double boxOtherMin2;
        double boxOtherMax2;
    };
    const Axis axes[] = {
        {normal.X(), origin.X(), sx0, sx1, fy0, fy1, sy0, sy1, fz0, fz1, sz0, sz1},
        {normal.Y(), origin.Y(), sy0, sy1, fx0, fx1, sx0, sx1, fz0, fz1, sz0, sz1},
        {normal.Z(), origin.Z(), sz0, sz1, fx0, fx1, sx0, sx1, fy0, fy1, sy0, sy1},
    };
    for (const Axis& axis : axes) {
        if (std::abs(std::abs(axis.normal) - 1.0) >= kAngularTolerance) continue;
        if (std::abs(axis.plane - axis.boxMin) >= kDistanceTolerance &&
            std::abs(axis.plane - axis.boxMax) >= kDistanceTolerance)
            continue;
        if (axis.faceOtherMax1 < axis.boxOtherMin1 - kDistanceTolerance ||
            axis.faceOtherMin1 > axis.boxOtherMax1 + kDistanceTolerance ||
            axis.faceOtherMax2 < axis.boxOtherMin2 - kDistanceTolerance ||
            axis.faceOtherMin2 > axis.boxOtherMax2 + kDistanceTolerance)
            return false;
        return true;
    }
    return false;
}

// 便捷判定:面盒不与 solidBox 相离,且面位于 solidBox 的 AABB 边界上。
bool isOuterFace(const TopoDS_Face& face, const Bnd_Box& solidBox)
{
    Bnd_Box faceBox;
    BRepBndLib::AddOptimal(face, faceBox, Standard_False, Standard_False);
    tightenBox(faceBox);
    return !solidBox.IsOut(faceBox) &&
           isFaceOnAabbBoundary(face, solidBox, &faceBox);
}

} // namespace oap

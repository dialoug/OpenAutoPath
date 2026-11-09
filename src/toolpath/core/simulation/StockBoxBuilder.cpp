// 毛坯盒构造实现（oap_toolpath_core/simulation）。
// 优先用分析阶段记录的包围盒；记录失效时退回从原始 OCCT 形状重新求界，
// 避免旧缓存或局部求界失败阻断毛坯仿真。
#include "toolpath/core/simulation/StockBoxBuilder.h"

#include <cmath>
#include <string>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>

namespace oap::toolpath {
namespace {

bool finiteBounds(const Bbox& bounds) noexcept
{
    return std::isfinite(bounds.xmin) && std::isfinite(bounds.ymin) &&
           std::isfinite(bounds.zmin) && std::isfinite(bounds.xmax) &&
           std::isfinite(bounds.ymax) && std::isfinite(bounds.zmax);
}

bool boundsFromBox(const Bnd_Box& box, Bbox& bounds) noexcept
{
    if (box.IsVoid() || box.IsWhole())
        return false;
    try {
        box.Get(bounds.xmin, bounds.ymin, bounds.zmin,
                bounds.xmax, bounds.ymax, bounds.zmax);
        return bounds.valid() && finiteBounds(bounds);
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
}

// 从形状计算包围盒：优先 AddOptimal，失败时退回更宽松的 Add 稳健求界。
bool boundsFromShape(const TopoDS_Shape& shape, Bbox& bounds) noexcept
{
    if (shape.IsNull())
        return false;

    // AddOptimal 与分析阶段保持一致。若异常 STEP 的精确求界失败，再退回更宽松的
    // Add；毛坯允许略微偏大，但不能因为一个坏曲面完全无法开始仿真。
    try {
        Bnd_Box optimal;
        BRepBndLib::AddOptimal(shape, optimal);
        if (boundsFromBox(optimal, bounds))
            return true;
    } catch (const Standard_Failure&) {
        // 继续尝试稳健求界。
    } catch (...) {
        // 继续尝试稳健求界。
    }

    try {
        Bnd_Box robust;
        BRepBndLib::Add(shape, robust);
        return boundsFromBox(robust, bounds);
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
}

// 按包围盒各轴单边余量构造轴对齐长方体。先做输入/边界有效性检查，
// 再调用 OCCT MakeBox；注意 OCCT 7.9 构造函数不生成拓扑，必须显式
// Build 后才能检查 IsDone。
bool makeBox(const Bbox& bounds,
             double marginX,
             double marginY,
             double marginZ,
             TopoDS_Shape& shape,
             std::string& failure) noexcept
{
    shape.Nullify();
    if (!bounds.valid() || !finiteBounds(bounds)) {
        failure = "bounds are invalid or contain non-finite coordinates";
        return false;
    }

    const double x = bounds.xmin - marginX;
    const double y = bounds.ymin - marginY;
    const double z = bounds.zmin - marginZ;
    const double dx = bounds.xmax - bounds.xmin + 2.0 * marginX;
    const double dy = bounds.ymax - bounds.ymin + 2.0 * marginY;
    const double dz = bounds.zmax - bounds.zmin + 2.0 * marginZ;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) {
        failure = "expanded box contains non-finite coordinates or dimensions";
        return false;
    }
    if (dx <= Precision::Confusion() || dy <= Precision::Confusion() ||
        dz <= Precision::Confusion()) {
        failure = "expanded box has a zero or sub-tolerance dimension";
        return false;
    }

    try {
        BRepPrimAPI_MakeBox maker(gp_Pnt(x, y, z), dx, dy, dz);
        // OCCT 7.9 的 MakeBox 构造函数只初始化算法，Build/Shape 才真正生成拓扑。
        // 在 Build 之前检查 IsDone 会把所有正常盒子都误判为失败。
        maker.Build();
        if (!maker.IsDone()) {
            failure = "OCCT did not complete box construction";
            return false;
        }
        shape = maker.Shape();
        if (shape.IsNull()) {
            failure = "OCCT returned an empty box";
            return false;
        }
        return true;
    } catch (const Standard_Failure&) {
        failure = "OCCT raised an exception during box construction";
    } catch (...) {
        failure = "unknown exception during box construction";
    }
    return false;
}

} // namespace

StockBoxBuildResult buildStockBox(
    const Bbox& recordedBounds,
    const TopoDS_Shape* sourceShape,
    double marginX,
    double marginY,
    double marginZ)
{
    StockBoxBuildResult result;
    if (!std::isfinite(marginX) || !std::isfinite(marginY) ||
        !std::isfinite(marginZ) || marginX < 0.0 || marginY < 0.0 ||
        marginZ < 0.0) {
        result.message = "stock margins must be finite and non-negative";
        return result;
    }

    result.effectiveBounds = recordedBounds;
    std::string recordedFailure;
    if (makeBox(recordedBounds, marginX, marginY, marginZ,
                result.shape, recordedFailure)) {
        result.message = "stock box constructed from recorded bounds";
        return result;
    }

    if (!sourceShape || sourceShape->IsNull()) {
        result.message = recordedFailure + "; source shape is unavailable";
        return result;
    }

    Bbox shapeBounds;
    if (!boundsFromShape(*sourceShape, shapeBounds)) {
        result.message = recordedFailure +
            "; could not recompute finite bounds from source shape";
        return result;
    }

    result.effectiveBounds = shapeBounds;
    result.usedShapeBounds = true;
    std::string recomputedFailure;
    if (!makeBox(shapeBounds, marginX, marginY, marginZ,
                 result.shape, recomputedFailure)) {
        result.message = recordedFailure +
            "; recomputed shape bounds also failed: " + recomputedFailure;
        return result;
    }

    result.message = "stock box constructed from recomputed source-shape bounds";
    return result;
}

} // namespace oap::toolpath

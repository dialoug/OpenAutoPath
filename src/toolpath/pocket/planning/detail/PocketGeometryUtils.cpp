// 槽腔局部坐标、环采样和点线几何计算工具，不承担图案或移动策略决策。
#include "toolpath/pocket/planning/detail/PocketGeometryUtils.h"

#include <cmath>

#include <gp_Vec.hxx>

#include "toolpath/pocket/geometry/OcctAreaAdapter.h"

namespace oap::toolpath::detail {

namespace {
constexpr double kDepthTolerance = 1.0e-9;
}

void appendWarning(std::string& warning, const std::string& message)
{
    if (!warning.empty())
        warning += "; ";
    warning += message;
}

gp_Pnt pointFrom2dAtDepth(const gp_Pnt2d& point,
                          const gp_Ax3& frame,
                          double depthOffset)
{
    // 在 frame 平面上重建 3D 点后，沿 frame +Z（开口方向）抬高 depthOffset；
    // 深度为 0 时原样返回，避免无谓的向量运算。
    gp_Pnt out = OcctAreaAdapter::pointFrom2d(point, frame);
    if (std::abs(depthOffset) > kDepthTolerance)
        out.Translate(gp_Vec(frame.Direction()) * depthOffset);
    return out;
}

} // namespace oap::toolpath::detail

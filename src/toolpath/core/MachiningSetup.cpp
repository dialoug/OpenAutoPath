// 模具六面到标准三轴工件坐标的确定性映射。
#include "MachiningSetup.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

#include "analysis/MoldCoordinateFrame.h"

namespace oap::toolpath {
namespace {

gp_Pnt localPoint(const MoldCoordinateFrame& frame,
                  double x, double y, double z)
{
    return frame.localToWorld(gp_Pnt(x, y, z));
}

} // namespace

const char* workCoordinateCode(WorkCoordinateSystem system) noexcept
{
    switch (system) {
        case WorkCoordinateSystem::None: return "";
        case WorkCoordinateSystem::G54: return "G54";
        case WorkCoordinateSystem::G55: return "G55";
        case WorkCoordinateSystem::G56: return "G56";
        case WorkCoordinateSystem::G57: return "G57";
        case WorkCoordinateSystem::G58: return "G58";
        case WorkCoordinateSystem::G59: return "G59";
    }
    return "";
}

std::optional<MachiningSetup> makeMoldFaceMachiningSetup(
    const MoldCoordinateFrame& moldFrame,
    MoldSetupFace face,
    WorkCoordinateSystem workOffset)
{
    if (!moldFrame.isValid || face == MoldSetupFace::Unresolved)
        return std::nullopt;

    const double length = moldFrame.length;
    const double width = moldFrame.width;
    const double thickness = moldFrame.thickness;
    std::array<gp_Pnt, 4> corners;
    gp_Vec outward;
    gp_Vec xDirection;

    // 六个面共用同一套推导：先在模具局部坐标列出该面的四个角点、朝外
    // 法向与面内 X 方向（保证每次面内坐标均为右手系），再在下方求面内
    // X/Y 方向上的二维左下角作为原点，使该面上的局部坐标恒非负。
    switch (face) {
        case MoldSetupFace::PositiveZ:
            corners = {localPoint(moldFrame, 0, 0, thickness),
                       localPoint(moldFrame, length, 0, thickness),
                       localPoint(moldFrame, 0, width, thickness),
                       localPoint(moldFrame, length, width, thickness)};
            outward = gp_Vec(moldFrame.zAxis);
            xDirection = gp_Vec(moldFrame.xAxis);
            break;
        case MoldSetupFace::NegativeZ:
            corners = {localPoint(moldFrame, 0, 0, 0),
                       localPoint(moldFrame, length, 0, 0),
                       localPoint(moldFrame, 0, width, 0),
                       localPoint(moldFrame, length, width, 0)};
            outward = gp_Vec(moldFrame.zAxis).Reversed();
            xDirection = gp_Vec(moldFrame.xAxis);
            break;
        case MoldSetupFace::PositiveX:
            corners = {localPoint(moldFrame, length, 0, 0),
                       localPoint(moldFrame, length, width, 0),
                       localPoint(moldFrame, length, 0, thickness),
                       localPoint(moldFrame, length, width, thickness)};
            outward = gp_Vec(moldFrame.xAxis);
            xDirection = gp_Vec(moldFrame.yAxis);
            break;
        case MoldSetupFace::NegativeX:
            corners = {localPoint(moldFrame, 0, 0, 0),
                       localPoint(moldFrame, 0, width, 0),
                       localPoint(moldFrame, 0, 0, thickness),
                       localPoint(moldFrame, 0, width, thickness)};
            outward = gp_Vec(moldFrame.xAxis).Reversed();
            xDirection = gp_Vec(moldFrame.yAxis);
            break;
        case MoldSetupFace::PositiveY:
            corners = {localPoint(moldFrame, 0, width, 0),
                       localPoint(moldFrame, length, width, 0),
                       localPoint(moldFrame, 0, width, thickness),
                       localPoint(moldFrame, length, width, thickness)};
            outward = gp_Vec(moldFrame.yAxis);
            xDirection = gp_Vec(moldFrame.xAxis);
            break;
        case MoldSetupFace::NegativeY:
            corners = {localPoint(moldFrame, 0, 0, 0),
                       localPoint(moldFrame, length, 0, 0),
                       localPoint(moldFrame, 0, 0, thickness),
                       localPoint(moldFrame, length, 0, thickness)};
            outward = gp_Vec(moldFrame.yAxis).Reversed();
            xDirection = gp_Vec(moldFrame.xAxis);
            break;
        case MoldSetupFace::Unresolved:
            return std::nullopt;
    }

    if (outward.SquareMagnitude() <= 0.0 ||
        xDirection.SquareMagnitude() <= 0.0)
        return std::nullopt;
    outward.Normalize();
    xDirection.Normalize();
    gp_Vec yDirection = outward.Crossed(xDirection);
    if (yDirection.SquareMagnitude() <= 0.0)
        return std::nullopt;
    yDirection.Normalize();

    // 选取从装夹外侧观察时的二维左下角，使所有面上的 X/Y 坐标均非负。
    const gp_Pnt reference = corners.front();
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    for (const gp_Pnt& corner : corners) {
        const gp_Vec delta(reference, corner);
        minX = std::min(minX, delta.Dot(xDirection));
        minY = std::min(minY, delta.Dot(yDirection));
    }
    const gp_Pnt origin = reference.Translated(
        xDirection.Multiplied(minX) + yDirection.Multiplied(minY));

    MachiningSetup setup;
    setup.workFrame = gp_Ax3(
        origin, gp_Dir(outward), gp_Dir(xDirection));
    setup.workOffset = workOffset;
    setup.useMoldFaceWorkFrame = true;
    return setup;
}

} // namespace oap::toolpath

// 刀具编辑器三维预览几何实现：用轴向剖面旋转生成刀刃，并补充装配包络。
#include "app/view/ToolPreviewGeometryBuilder.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Precision.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace oap::app::view {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kArcSegments = 18;

struct RadiusAtHeight {
    double radius = 0.0;
    double height = 0.0;
};

TopoDS_Shape cylinder(double diameter, double length, double baseHeight)
{
    if (!(diameter > Precision::Confusion()) ||
        !(length > Precision::Confusion())) {
        return {};
    }
    return BRepPrimAPI_MakeCylinder(
               gp_Ax2(gp_Pnt(0.0, 0.0, baseHeight), gp_Dir(0.0, 0.0, 1.0)),
               diameter * 0.5, length)
        .Shape();
}

TopoDS_Shape revolvedProfile(const std::vector<RadiusAtHeight>& outside)
{
    if (outside.size() < 2)
        return {};

    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(gp_Pnt(0.0, 0.0, outside.front().height));
    for (const RadiusAtHeight& point : outside)
        polygon.Add(gp_Pnt(std::max(0.0, point.radius), 0.0, point.height));
    polygon.Add(gp_Pnt(0.0, 0.0, outside.back().height));
    polygon.Close();
    if (!polygon.IsDone())
        return {};

    BRepBuilderAPI_MakeFace face(TopoDS::Wire(polygon.Shape()));
    if (!face.IsDone())
        return {};
    BRepPrimAPI_MakeRevol revolved(
        face.Face(), gp_Ax1(gp_Pnt(0.0, 0.0, 0.0),
                           gp_Dir(0.0, 0.0, 1.0)),
        2.0 * kPi, true);
    return revolved.IsDone() ? revolved.Shape() : TopoDS_Shape{};
}

TopoDS_Shape cutterShape(tooling::ToolType type, double diameter,
                         double cuttingLength, double cornerRadius,
                         double tipAngle)
{
    const double radius = diameter * 0.5;
    if (!(radius > Precision::Confusion()) ||
        !(cuttingLength > Precision::Confusion())) {
        return {};
    }

    std::vector<RadiusAtHeight> profile;
    switch (type) {
        case tooling::ToolType::BallEndMill: {
            const double ballHeight = std::min(radius, cuttingLength);
            for (int index = 0; index <= kArcSegments; ++index) {
                const double height = ballHeight * index / kArcSegments;
                const double sphereHeight = std::min(height, radius);
                const double radial = std::sqrt(std::max(
                    0.0, radius * radius -
                             (sphereHeight - radius) *
                                 (sphereHeight - radius)));
                profile.push_back({radial, height});
            }
            if (cuttingLength > ballHeight)
                profile.push_back({radius, cuttingLength});
            break;
        }
        case tooling::ToolType::BullNose: {
            const double corner = std::clamp(cornerRadius, 0.0, radius);
            if (corner <= Precision::Confusion()) {
                profile = {{radius, 0.0}, {radius, cuttingLength}};
                break;
            }
            const double arcHeight = std::min(corner, cuttingLength);
            for (int index = 0; index <= kArcSegments; ++index) {
                const double height = arcHeight * index / kArcSegments;
                const double radial = radius - corner + std::sqrt(std::max(
                    0.0, corner * corner -
                             (height - corner) * (height - corner)));
                profile.push_back({radial, height});
            }
            if (cuttingLength > arcHeight)
                profile.push_back({radius, cuttingLength});
            break;
        }
        case tooling::ToolType::CenterDrill:
        case tooling::ToolType::Drill:
        case tooling::ToolType::IndexableDrill:
        case tooling::ToolType::ChamferMill:
        case tooling::ToolType::Tap: {
            const double angle = tipAngle > 0.0 && tipAngle < 180.0
                ? tipAngle : 118.0;
            const double idealTipHeight = radius /
                std::tan(0.5 * angle * kPi / 180.0);
            const double tipHeight = std::min(cuttingLength, idealTipHeight);
            profile = {{0.0, 0.0}, {radius, tipHeight}};
            if (cuttingLength > tipHeight)
                profile.push_back({radius, cuttingLength});
            break;
        }
        default:
            profile = {{radius, 0.0}, {radius, cuttingLength}};
            break;
    }
    return revolvedProfile(profile);
}

} // namespace

ToolPreviewGeometry buildToolPreviewGeometry(
    tooling::ToolType type,
    const tooling::ToolGeometry& sourceGeometry)
{
    const tooling::ToolGeometry geometry =
        tooling::normalizedToolGeometry(type, sourceGeometry);
    const double diameter = geometry.diameter > 0.0
        ? geometry.diameter : 6.0;
    const double overallLength = geometry.overallLength > 0.0
        ? geometry.overallLength
        : std::max(30.0, geometry.cuttingEdgeLength * 1.5);
    const double cuttingLength = geometry.cuttingEdgeLength > 0.0
        ? std::min(geometry.cuttingEdgeLength, overallLength)
        : overallLength * 0.4;
    const double shankDiameter = geometry.shankDiameter > 0.0
        ? geometry.shankDiameter : diameter;

    ToolPreviewGeometry result;
    double toolTop = overallLength;
    if (type == tooling::ToolType::Probe) {
        const double ballRadius = diameter * 0.5;
        result.cuttingPart = BRepPrimAPI_MakeSphere(
            gp_Pnt(0.0, 0.0, ballRadius), ballRadius).Shape();
        const double stemStart = ballRadius;
        const double requestedStemLength = geometry.cuttingEdgeLength > 0.0
            ? geometry.cuttingEdgeLength
            : std::max(0.0, overallLength - stemStart);
        toolTop = std::max(overallLength,
                           stemStart + requestedStemLength);
        const double stemLength = std::max(0.0, toolTop - stemStart);
        result.shank = cylinder(shankDiameter, stemLength, stemStart);
    } else {
        result.cuttingPart = cutterShape(
            type, diameter, cuttingLength, geometry.cornerRadius,
            geometry.tipAngle);
        result.shank = cylinder(
            shankDiameter, std::max(0.0, overallLength - cuttingLength),
            cuttingLength);
    }

    const double holderDiameter = geometry.holderDiameter > 0.0
        ? geometry.holderDiameter : shankDiameter * 2.2;
    const double holderLength = geometry.holderLength > 0.0
        ? geometry.holderLength : overallLength * 0.35;
    result.holder = cylinder(holderDiameter, holderLength, toolTop);

    const double spindleDiameter = geometry.spindleEnvelopeDiameter > 0.0
        ? geometry.spindleEnvelopeDiameter : holderDiameter * 1.7;
    const double spindleLength = geometry.spindleEnvelopeLength > 0.0
        ? geometry.spindleEnvelopeLength : holderLength * 0.75;
    result.spindleEnvelope = cylinder(
        spindleDiameter, spindleLength, toolTop + holderLength);
    return result;
}

} // namespace oap::app::view

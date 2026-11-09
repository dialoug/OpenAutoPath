#include "app/view/ToolPreviewGeometryBuilder.h"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>

#include <cmath>
#include <iostream>
#include <string_view>

namespace {

bool expect(bool condition, std::string_view message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

double volume(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        return 0.0;
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

bool assemblySegmentsAreBuilt()
{
    oap::tooling::ToolGeometry geometry;
    geometry.diameter = 10.0;
    geometry.cuttingEdgeLength = 20.0;
    geometry.overallLength = 50.0;
    geometry.shankDiameter = 10.0;
    geometry.holderDiameter = 24.0;
    geometry.holderLength = 30.0;
    geometry.spindleEnvelopeDiameter = 42.0;
    geometry.spindleEnvelopeLength = 20.0;

    const auto preview = oap::app::view::buildToolPreviewGeometry(
        oap::tooling::ToolType::EndMill, geometry);
    if (!expect(!preview.isEmpty(), "end mill preview must not be empty") ||
        !expect(volume(preview.cuttingPart) > 0.0,
                "cutting part must be a solid") ||
        !expect(volume(preview.shank) > 0.0, "shank must be a solid") ||
        !expect(volume(preview.holder) > 0.0, "holder must be a solid") ||
        !expect(volume(preview.spindleEnvelope) > 0.0,
                "spindle envelope must be a solid")) {
        return false;
    }

    Bnd_Box cuttingBounds;
    BRepBndLib::Add(preview.cuttingPart, cuttingBounds);
    double xmin = 0.0;
    double ymin = 0.0;
    double zmin = 0.0;
    double xmax = 0.0;
    double ymax = 0.0;
    double zmax = 0.0;
    cuttingBounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return expect(std::abs((xmax - xmin) - 10.0) < 1.0e-5,
                  "end mill preview must preserve diameter") &&
           expect(std::abs((zmax - zmin) - 20.0) < 1.0e-5,
                  "end mill preview must preserve cutting length");
}

bool specializedProfilesAreSolid()
{
    oap::tooling::ToolGeometry geometry;
    geometry.diameter = 12.0;
    geometry.cuttingEdgeLength = 24.0;
    geometry.overallLength = 60.0;
    geometry.shankDiameter = 12.0;
    geometry.tipAngle = 118.0;

    const auto ball = oap::app::view::buildToolPreviewGeometry(
        oap::tooling::ToolType::BallEndMill, geometry);
    geometry.cornerRadius = 1.5;
    const auto bull = oap::app::view::buildToolPreviewGeometry(
        oap::tooling::ToolType::BullNose, geometry);
    const auto drill = oap::app::view::buildToolPreviewGeometry(
        oap::tooling::ToolType::Drill, geometry);
    const auto probe = oap::app::view::buildToolPreviewGeometry(
        oap::tooling::ToolType::Probe, geometry);

    return expect(volume(ball.cuttingPart) > 0.0,
                  "ball end profile must produce a solid") &&
           expect(volume(bull.cuttingPart) > 0.0,
                  "bull nose profile must produce a solid") &&
           expect(volume(drill.cuttingPart) > 0.0,
                  "drill profile must produce a solid") &&
           expect(volume(probe.cuttingPart) > 0.0 &&
                      volume(probe.shank) > 0.0,
                  "probe ball and stem must produce solids");
}

} // namespace

int main()
{
    const bool passed = assemblySegmentsAreBuilt() &&
                        specializedProfilesAreSolid();
    if (passed)
        std::cout << "tool preview geometry smoke passed\n";
    return passed ? 0 : 1;
}

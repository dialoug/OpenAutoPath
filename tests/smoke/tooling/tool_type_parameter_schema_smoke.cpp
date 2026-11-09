#include "tooling/library/ToolLibrary.h"

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

bool schemaSeparatesToolTypeParameters()
{
    const auto drill = oap::tooling::toolTypeParameterSchema(
        oap::tooling::ToolType::Drill);
    const auto ball = oap::tooling::toolTypeParameterSchema(
        oap::tooling::ToolType::BallEndMill);
    const auto tap = oap::tooling::toolTypeParameterSchema(
        oap::tooling::ToolType::Tap);
    const auto probe = oap::tooling::toolTypeParameterSchema(
        oap::tooling::ToolType::Probe);

    return expect(drill.tipAngleVisible &&
                      drill.tipCompensationVisible &&
                      drill.drillingDefaultsVisible,
                  "drills must expose drilling-specific parameters") &&
           expect(ball.cornerRadiusVisible &&
                      ball.cornerRadiusDerivedFromDiameter &&
                      !ball.tipAngleVisible,
                  "ball end mills must derive their ball radius") &&
           expect(tap.threadSpecificationVisible && tap.tipAngleVisible &&
                      !tap.drillingDefaultsVisible,
                  "taps must expose thread shape without pecking") &&
           expect(!probe.diameterRequired && !probe.fluteCountVisible,
                  "probes must not be treated as fluted cutting tools");
}

bool normalizationRemovesInapplicableGeometry()
{
    oap::tooling::ToolGeometry geometry;
    geometry.diameter = 12.0;
    geometry.cornerRadius = 1.0;
    geometry.tipAngle = 118.0;
    geometry.tipCompensation = 2.0;

    const auto ball = oap::tooling::normalizedToolGeometry(
        oap::tooling::ToolType::BallEndMill, geometry);
    const auto endMill = oap::tooling::normalizedToolGeometry(
        oap::tooling::ToolType::EndMill, geometry);
    return expect(std::abs(ball.cornerRadius - 6.0) < 1.0e-9,
                  "ball radius must equal half of the diameter") &&
           expect(ball.tipAngle == 0.0 && ball.tipCompensation == 0.0,
                  "ball end mills must not retain drill-tip values") &&
           expect(endMill.cornerRadius == 0.0 &&
                      endMill.tipAngle == 0.0 &&
                      endMill.tipCompensation == 0.0,
                  "flat end mills must clear unrelated shape values");
}

bool validationUsesTheSelectedToolType()
{
    oap::tooling::ToolGeometry drill;
    const QStringList drillErrors =
        oap::tooling::toolGeometryValidationErrors(
            oap::tooling::ToolType::Drill, drill);
    const QStringList probeErrors =
        oap::tooling::toolGeometryValidationErrors(
            oap::tooling::ToolType::Probe, drill);

    oap::tooling::ToolGeometry invalidBall;
    invalidBall.diameter = 10.0;
    invalidBall.cornerRadius = 4.0;
    const QStringList ballErrors =
        oap::tooling::toolGeometryValidationErrors(
            oap::tooling::ToolType::BallEndMill, invalidBall);
    return expect(!drillErrors.isEmpty(),
                  "a drill must require a positive diameter") &&
           expect(probeErrors.isEmpty(),
                  "an empty probe geometry remains backward compatible") &&
           expect(!ballErrors.isEmpty(),
                  "an explicitly inconsistent ball radius must be rejected");
}

} // namespace

int main()
{
    const bool passed = schemaSeparatesToolTypeParameters() &&
                        normalizationRemovesInapplicableGeometry() &&
                        validationUsesTheSelectedToolType();
    if (passed)
        std::cout << "tool type parameter schema smoke passed\n";
    return passed ? 0 : 1;
}

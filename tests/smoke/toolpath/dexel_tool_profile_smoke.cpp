#include "toolpath/core/simulation/DexelToolProfile.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace {

using oap::toolpath::DexelToolProfile;
using oap::toolpath::DexelToolProfileBuildOptions;
using oap::toolpath::DexelToolProfileDefinition;
using oap::toolpath::DexelToolProfileFamily;
using oap::toolpath::DexelToolProfileKind;
using oap::toolpath::buildDexelToolProfile;

bool expect(bool condition, const std::string& message)
{
    if (!condition)
        std::cerr << "FAILED: " << message << '\n';
    return condition;
}

DexelToolProfileBuildOptions options()
{
    DexelToolProfileBuildOptions result;
    result.axialTolerance = 0.25;
    result.radialTolerance = 0.25;
    result.geometricTolerance = 1.0e-6;
    result.maximumBandCount = 128;
    return result;
}

bool buildsFlatProfile()
{
    DexelToolProfile profile;
    std::string error;
    return expect(
               buildDexelToolProfile(
                   {DexelToolProfileFamily::RoundedEnd,
                    10.0, 20.0, 0.0, 0.0},
                   options(), profile, &error),
               "flat profile should build: " + error) &&
           expect(
               profile.kind ==
                       DexelToolProfileKind::FlatEnd &&
                   profile.bands.size() == 1 &&
                   profile.bands.front().lower == 0.0 &&
                   profile.bands.front().upper == 20.0 &&
                   profile.bands.front().radius == 5.0 &&
                    profile.maximumAxialError == 0.0 &&
                    profile.maximumRadialError == 0.0,
               "flat profile should remain one exact cylindrical band");
}

bool buildsBullAndBallProfiles()
{
    DexelToolProfile bull;
    DexelToolProfile ball;
    std::string error;
    if (!expect(
            buildDexelToolProfile(
                {DexelToolProfileFamily::RoundedEnd,
                 10.0, 8.0, 1.0, 0.0},
                options(), bull, &error),
            "bull-nose profile should build: " + error) ||
        !expect(
            buildDexelToolProfile(
                {DexelToolProfileFamily::RoundedEnd,
                 10.0, 8.0, 5.0, 0.0},
                options(), ball, &error),
            "ball-end profile should build: " + error)) {
        return false;
    }

    const auto monotonicRadii =
        [](const DexelToolProfile& profile) {
            for (std::size_t index = 1;
                 index < profile.bands.size();
                 ++index) {
                if (profile.bands[index].radius + 1.0e-9 <
                    profile.bands[index - 1].radius) {
                    return false;
                }
            }
            return true;
        };
    return expect(
               bull.kind ==
                       DexelToolProfileKind::BullNose &&
                   bull.bands.front().lower == 0.0 &&
                   std::abs(
                       bull.bands.front().radius -
                       4.0) < 1.0e-9 &&
                   bull.bands.back().radius == 5.0,
               "bull-nose profile should start at its flat radius and end at full radius") &&
           expect(
               ball.kind ==
                       DexelToolProfileKind::BallEnd &&
                   ball.bands.front().lower > 0.0 &&
                   ball.bands.back().radius == 5.0,
               "ball-end profile should conservatively omit only its zero-area apex band") &&
           expect(
               monotonicRadii(bull) &&
                   monotonicRadii(ball) &&
                   bull.maximumAxialError <=
                       options().axialTolerance +
                           1.0e-9 &&
                    ball.maximumAxialError <=
                        options().axialTolerance +
                            1.0e-9 &&
                    bull.maximumRadialError <=
                        options().radialTolerance +
                            1.0e-9 &&
                    ball.maximumRadialError <=
                        options().radialTolerance +
                            1.0e-9,
                "rounded profile radii and axial/radial errors should be bounded");
}

bool adaptsToRadialTolerance()
{
    auto loose = options();
    loose.axialTolerance = 1.0;
    loose.radialTolerance = 10.0;
    auto tight = loose;
    tight.radialTolerance = 0.1;

    DexelToolProfile looseBall;
    DexelToolProfile tightBall;
    std::string error;
    if (!expect(
            buildDexelToolProfile(
                {DexelToolProfileFamily::RoundedEnd,
                 10.0, 8.0, 5.0, 0.0},
                loose, looseBall, &error),
            "loose radial ball profile should build: " + error) ||
        !expect(
            buildDexelToolProfile(
                {DexelToolProfileFamily::RoundedEnd,
                 10.0, 8.0, 5.0, 0.0},
                tight, tightBall, &error),
            "tight radial ball profile should build: " + error)) {
        return false;
    }

    const auto radiusAtHeight = [](double height) {
        const double vertical = 5.0 - height;
        return std::sqrt(std::max(
            0.0, 25.0 - vertical * vertical));
    };
    const bool everyBandBounded = std::all_of(
        tightBall.bands.begin(), tightBall.bands.end(),
        [&](const auto& band) {
            if (band.lower >= 5.0)
                return true;
            const double upper = std::min(5.0, band.upper);
            return radiusAtHeight(upper) - band.radius <=
                tight.radialTolerance + 1.0e-9;
        });
    return expect(
               tightBall.bands.size() > looseBall.bands.size(),
               "tight radial tolerance should adaptively add curved profile bands") &&
           expect(
               tightBall.maximumAxialError <=
                       tight.axialTolerance + 1.0e-9 &&
                   tightBall.maximumRadialError <=
                       tight.radialTolerance + 1.0e-9 &&
                   everyBandBounded,
               "every adaptive ball band should satisfy both error bounds");
}

bool buildsDrillPointProfile()
{
    DexelToolProfile profile;
    std::string error;
    if (!expect(
            buildDexelToolProfile(
                {DexelToolProfileFamily::DrillPoint,
                 10.0, 20.0, 0.0, 118.0},
                options(), profile, &error),
            "drill-point profile should build: " + error)) {
        return false;
    }
    const auto& cylinder = profile.bands.back();
    return expect(
               profile.kind ==
                       DexelToolProfileKind::DrillPoint &&
                   profile.bands.size() > 2 &&
                   cylinder.radius == 5.0 &&
                   cylinder.upper == 20.0,
               "drill profile should end in a full-radius cylindrical band") &&
           expect(
               profile.maximumAxialError <=
                        options().axialTolerance + 1.0e-9 &&
                    profile.maximumRadialError <=
                        options().radialTolerance + 1.0e-9,
                "drill-point undercut should respect axial and radial tolerances");
}

bool rejectsInvalidProfiles()
{
    DexelToolProfile profile;
    std::string error;
    auto limited = options();
    limited.maximumBandCount = 2;
    auto invalidRadial = options();
    invalidRadial.radialTolerance =
        invalidRadial.geometricTolerance;
    return expect(
               !buildDexelToolProfile(
                   {DexelToolProfileFamily::RoundedEnd,
                    10.0, 8.0, 6.0, 0.0},
                   options(), profile, &error),
               "corner radius above the tool radius should be rejected") &&
            expect(
                !buildDexelToolProfile(
                   {DexelToolProfileFamily::DrillPoint,
                    10.0, 20.0, 0.0, 180.0},
                   options(), profile, &error),
               "degenerate drill angle should be rejected") &&
           expect(
               !buildDexelToolProfile(
                   {DexelToolProfileFamily::DrillPoint,
                    10.0, 2.0, 0.0, 118.0},
                   options(), profile, &error),
               "a drill tip longer than the cutting edge should be rejected") &&
           expect(
               !buildDexelToolProfile(
                   {DexelToolProfileFamily::RoundedEnd,
                    10.0, 8.0, 5.0, 0.0},
                    limited, profile, &error),
                "profile band budget overflow should be rejected") &&
            expect(
                !buildDexelToolProfile(
                    {DexelToolProfileFamily::RoundedEnd,
                     10.0, 8.0, 5.0, 0.0},
                    invalidRadial, profile, &error),
                "radial tolerance below geometric resolution should be rejected");
}

} // namespace

int main()
{
    if (!buildsFlatProfile())
        return 1;
    if (!buildsBullAndBallProfiles())
        return 1;
    if (!adaptsToRadialTolerance())
        return 1;
    if (!buildsDrillPointProfile())
        return 1;
    if (!rejectsInvalidProfiles())
        return 1;

    std::cout << "Dexel tool profile smoke checks passed\n";
    return 0;
}

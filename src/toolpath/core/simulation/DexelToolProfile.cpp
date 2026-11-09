// 轴对称刀具→固定刀轴 Dexel 保守轮廓的构建实现（oap_toolpath_core/simulation）。
// 曲面刀尖被离散为轴向内接圆柱带，每个带取轴向区间内最小半径，
// 确保材料去除不会越过解析刀具轮廓。
#include "toolpath/core/simulation/DexelToolProfile.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace oap::toolpath {
namespace {

// 刀尖局部圆角在高度 h 处的解析半径：平底恒为刀半径，圆角区按
// 圆弧截面求水平半径。输入高度先夹到 [0, cornerRadius]。
double roundedRadiusAtHeight(
    double toolRadius,
    double cornerRadius,
    double height)
{
    if (cornerRadius <= 0.0)
        return toolRadius;
    height = std::clamp(
        height, 0.0, cornerRadius);
    const double vertical =
        cornerRadius - height;
    return toolRadius - cornerRadius +
           std::sqrt(std::max(
               0.0,
               cornerRadius * cornerRadius -
                   vertical * vertical));
}

// 从轴向高度 0 开始逐带推进：先按轴向容差试探带顶，若带内解析半径
// 增量超过径向容差，则用二分把带顶回收到满足约束的位置，使每个带
// 同时满足轴向与径向误差上限；半径过小的带被跳过（切向零余量）。
bool appendCurvedBands(
    double axialHeight,
    double axialTolerance,
    double radialTolerance,
    int maximumBandCount,
    double geometricTolerance,
    const auto& radiusAtHeight,
    DexelToolProfile& output,
    std::string& error)
{
    if (axialHeight <= geometricTolerance)
        return true;
    const int existingBandCount =
        static_cast<int>(output.bands.size());
    int generatedBandCount = 0;
    double lower = 0.0;
    while (axialHeight - lower > geometricTolerance) {
        if (existingBandCount + generatedBandCount >=
            maximumBandCount) {
            error =
                "Dexel tool profile exceeds the adaptive band budget";
            return false;
        }

        const double lowerRadius = radiusAtHeight(lower);
        double upper = std::min(
            axialHeight, lower + axialTolerance);
        double upperRadius = radiusAtHeight(upper);
        if (!std::isfinite(lowerRadius) ||
            !std::isfinite(upperRadius) ||
            upperRadius + geometricTolerance < lowerRadius) {
            error =
                "Dexel tool profile radius is not finite and monotonic";
            return false;
        }

        if (upperRadius - lowerRadius >
            radialTolerance * (1.0 + 1.0e-12)) {
            double accepted = lower;
            double rejected = upper;
            for (int iteration = 0; iteration < 64; ++iteration) {
                const double middle = 0.5 * (accepted + rejected);
                const double middleRadius = radiusAtHeight(middle);
                if (!std::isfinite(middleRadius)) {
                    error =
                        "Dexel tool profile radius is not finite";
                    return false;
                }
                if (middleRadius - lowerRadius <= radialTolerance)
                    accepted = middle;
                else
                    rejected = middle;
            }
            upper = accepted;
            upperRadius = radiusAtHeight(upper);
        }
        if (upper - lower <= geometricTolerance) {
            error =
                "Dexel tool profile tolerances are below geometric resolution";
            return false;
        }

        ++generatedBandCount;
        output.maximumAxialError =
            std::max(
                output.maximumAxialError,
                upper - lower);
        output.maximumRadialError =
            std::max(
                output.maximumRadialError,
                std::max(0.0, upperRadius - lowerRadius));
        if (lowerRadius > geometricTolerance) {
            output.bands.push_back(
                {lower, upper, lowerRadius});
        }
        lower = upper;
    }
    return true;
}

} // namespace

// 校验刀具轮廓：数值有限、带区间单调递增且在切削长度内、半径合理。
bool DexelToolProfile::valid(double tolerance) const noexcept
{
    if (!std::isfinite(tolerance) ||
        tolerance < 0.0 ||
        !std::isfinite(maximumRadius) ||
        maximumRadius <= tolerance ||
        !std::isfinite(cuttingLength) ||
        cuttingLength <= tolerance ||
        !std::isfinite(maximumAxialError) ||
        maximumAxialError < 0.0 ||
        !std::isfinite(maximumRadialError) ||
        maximumRadialError < 0.0 ||
        bands.empty()) {
        return false;
    }
    double previousLower = -tolerance;
    for (const DexelToolProfileBand& band : bands) {
        if (!std::isfinite(band.lower) ||
            !std::isfinite(band.upper) ||
            !std::isfinite(band.radius) ||
            band.lower < -tolerance ||
            band.lower + tolerance < previousLower ||
            band.upper <= band.lower + tolerance ||
            band.upper > cuttingLength + tolerance ||
            band.radius <= tolerance ||
            band.radius > maximumRadius + tolerance) {
            return false;
        }
        previousLower = band.lower;
    }
    return true;
}

bool buildDexelToolProfile(
    const DexelToolProfileDefinition& definition,
    const DexelToolProfileBuildOptions& options,
    DexelToolProfile& output,
    std::string* error)
{
    output = {};
    auto fail = [&](std::string message) {
        if (error)
            *error = std::move(message);
        return false;
    };

    if (!std::isfinite(options.geometricTolerance) ||
        options.geometricTolerance <= 0.0) {
        return fail(
            "Dexel tool profile requires a positive geometric tolerance");
    }
    const double tolerance =
        options.geometricTolerance;
    if (!std::isfinite(options.axialTolerance) ||
        options.axialTolerance <= tolerance) {
        return fail(
            "Dexel tool profile requires an axial tolerance above the geometric tolerance");
    }
    if (!std::isfinite(options.radialTolerance) ||
        options.radialTolerance <= tolerance) {
        return fail(
            "Dexel tool profile requires a radial tolerance above the geometric tolerance");
    }
    if (options.maximumBandCount <= 0) {
        return fail(
            "Dexel tool profile requires a positive axial band budget");
    }
    if (!std::isfinite(definition.diameter) ||
        definition.diameter <= 2.0 * tolerance) {
        return fail(
            "Dexel tool profile requires a positive finite diameter");
    }
    if (!std::isfinite(definition.cuttingLength) ||
        definition.cuttingLength <= tolerance) {
        return fail(
            "Dexel tool profile requires a positive finite cutting length");
    }

    const double toolRadius =
        0.5 * definition.diameter;
    output.maximumRadius = toolRadius;
    output.cuttingLength =
        definition.cuttingLength;
    std::string buildError;

    if (definition.family ==
        DexelToolProfileFamily::RoundedEnd) {
        if (!std::isfinite(definition.cornerRadius) ||
            definition.cornerRadius < -tolerance ||
            definition.cornerRadius >
                toolRadius + tolerance) {
            return fail(
                "Dexel rounded tool requires a finite corner radius within the tool radius");
        }
        const double cornerRadius = std::clamp(
            definition.cornerRadius,
            0.0, toolRadius);
        if (cornerRadius <= tolerance) {
            output.kind =
                DexelToolProfileKind::FlatEnd;
            output.bands.push_back(
                {0.0, definition.cuttingLength,
                 toolRadius});
        } else {
            output.kind =
                cornerRadius >= toolRadius - tolerance
                ? DexelToolProfileKind::BallEnd
                : DexelToolProfileKind::BullNose;
            const double cornerHeight =
                std::min(
                    cornerRadius,
                    definition.cuttingLength);
            if (!appendCurvedBands(
                    cornerHeight,
                    options.axialTolerance,
                    options.radialTolerance,
                    options.maximumBandCount,
                    tolerance,
                    [&](double height) {
                        return roundedRadiusAtHeight(
                            toolRadius,
                            cornerRadius,
                            height);
                    },
                    output, buildError)) {
                return fail(std::move(buildError));
            }
            if (definition.cuttingLength >
                cornerRadius + tolerance) {
                if (static_cast<int>(
                        output.bands.size()) >=
                    options.maximumBandCount) {
                    return fail(
                        "Dexel tool profile exceeds the axial band budget");
                }
                output.bands.push_back(
                    {cornerRadius,
                     definition.cuttingLength,
                     toolRadius});
            }
        }
    } else {
        if (!std::isfinite(definition.tipAngleDeg) ||
            definition.tipAngleDeg <= 0.0 ||
            definition.tipAngleDeg >= 180.0) {
            return fail(
                "Dexel drill profile requires a tip angle between 0 and 180 degrees");
        }
        const double tangent =
            std::tan(
                0.5 * definition.tipAngleDeg *
                std::numbers::pi / 180.0);
        if (!std::isfinite(tangent) ||
            tangent <= tolerance) {
            return fail(
                "Dexel drill profile has a degenerate tip angle");
        }
        const double tipLength =
            toolRadius / tangent;
        if (!std::isfinite(tipLength) ||
            tipLength >=
                definition.cuttingLength - tolerance) {
            return fail(
                "Dexel drill profile tip does not fit within the cutting length");
        }
        output.kind =
            DexelToolProfileKind::DrillPoint;
        if (!appendCurvedBands(
                tipLength,
                options.axialTolerance,
                options.radialTolerance,
                options.maximumBandCount,
                tolerance,
                [&](double height) {
                    return height * tangent;
                },
                output, buildError)) {
            return fail(std::move(buildError));
        }
        if (static_cast<int>(output.bands.size()) >=
            options.maximumBandCount) {
            return fail(
                "Dexel tool profile exceeds the axial band budget");
        }
        output.bands.push_back(
            {tipLength,
             definition.cuttingLength,
             toolRadius});
    }

    if (!output.valid(tolerance)) {
        return fail(
            "Dexel tool profile produced no usable inscribed bands");
    }
    if (error)
        error->clear();
    return true;
}

} // namespace oap::toolpath

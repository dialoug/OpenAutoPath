// 轴对称刀具在固定刀轴 Dexel 内核中的保守离散轮廓。
//
// 刀尖位于轴向高度 0，正轴方向指向主轴。曲面刀尖被离散为若干内接圆柱带，
// 每个带使用该轴向区间内的最小半径，因此材料去除不会越过解析刀具轮廓。
#pragma once

#include <string>
#include <vector>

namespace oap::toolpath {

enum class DexelToolProfileFamily {
    RoundedEnd,
    DrillPoint,
};

enum class DexelToolProfileKind {
    FlatEnd,
    BullNose,
    BallEnd,
    DrillPoint,
};

// dexel 刀具轮廓定义：族、直径、切削长、角半径与顶尖角。
struct DexelToolProfileDefinition {
    DexelToolProfileFamily family =
        DexelToolProfileFamily::RoundedEnd;
    double diameter = 0.0;
    double cuttingLength = 0.0;
    // RoundedEnd 使用；0 为平底，等于刀具半径为球头，中间值为圆角刀。
    double cornerRadius = 0.0;
    // DrillPoint 使用；钻尖包含角，单位为度。
    double tipAngleDeg = 0.0;
};

// 刀具轮廓构建选项：轴向/径向/几何容差与最大分段带数。
struct DexelToolProfileBuildOptions {
    double axialTolerance = 0.05;
    // 每个内接带从下边界半径到上边界解析半径的最大增量。
    // 与 axialTolerance 同时生效，任一约束先到即结束当前带。
    double radialTolerance = 0.05;
    double geometricTolerance = 1.0e-5;
    int maximumBandCount = 512;
};

// 刀具轮廓分段带：轴向区间与该段半径。
struct DexelToolProfileBand {
    double lower = 0.0;
    double upper = 0.0;
    double radius = 0.0;

    bool operator==(const DexelToolProfileBand&) const =
        default;
};

// 刀具轮廓：种类、最大半径、切削长与轴向/径向误差。
struct DexelToolProfile {
    DexelToolProfileKind kind =
        DexelToolProfileKind::FlatEnd;
    double maximumRadius = 0.0;
    double cuttingLength = 0.0;
    // 曲面带的最大轴向厚度，也是相对解析轮廓的保守轴向欠切上界。
    double maximumAxialError = 0.0;
    // 曲面带使用下边界最小半径，记录带顶解析半径与该内接半径的最大差。
    double maximumRadialError = 0.0;
    std::vector<DexelToolProfileBand> bands;

    bool valid(double tolerance = 1.0e-9) const noexcept;
};

bool buildDexelToolProfile(
    const DexelToolProfileDefinition& definition,
    const DexelToolProfileBuildOptions& options,
    DexelToolProfile& output,
    std::string* error = nullptr);

} // namespace oap::toolpath

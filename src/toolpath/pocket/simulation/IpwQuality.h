#pragma once
#include "toolpath/pocket/simulation/DexelDisplayMesh.h"
#include <algorithm>
#include <cmath>

namespace oap::toolpath {
// 显示质量档位：影响三角网格与图元的生成预算。
enum class IpwDisplayQuality {
    Interactive,
    Balanced,
    Detailed,
};

struct IpwDisplayQualityProfile {
    IpwDisplayQuality quality =
        IpwDisplayQuality::Detailed;
    toolpath::DexelDisplayMeshOptions meshOptions;
};

// 同一 1～10 级质量同时约束 Multi-Dexel 材料真值的 XY 采样密度。
// requestedResolution 是期望值；MultiDexelStock 仍可为满足 maximumCellCount
// 而采用更粗的实际分辨率，调用方必须显示最终实际值而不是只显示请求值。
struct IpwSimulationQualityProfile {
    IpwDisplayQuality quality =
        IpwDisplayQuality::Detailed;
    double cellsPerCutterDiameter = 80.0;
    double requestedResolution = 0.0;
    double requestedPlanarErrorUpperBound = 0.0;
    int maximumCellCount = 8'000'000;
};

// 1..10 级显示质量 → 网格轴向量化（按材料分辨率比例）与
// 平面单元/三角形预算；材料真值与比较结果不受这些选项影响。
inline IpwDisplayQualityProfile ipwDisplayQualityProfile(
    int level,
    double gridResolution)
{
    IpwDisplayQualityProfile profile;
    const int clamped = std::clamp(level, 1, 10);
    const double resolution =
        std::isfinite(gridResolution) &&
                gridResolution > 0.0
        ? gridResolution
        : 1.0;
    if (clamped <= 3) {
        profile.quality =
            IpwDisplayQuality::Interactive;
        profile.meshOptions.axialQuantization =
            0.5 * resolution;
        profile.meshOptions.maximumPlanarCellCount =
            125'000u;
        profile.meshOptions.maximumTriangleCount =
            1'000'000u;
    } else if (clamped <= 7) {
        profile.quality =
            IpwDisplayQuality::Balanced;
        profile.meshOptions.axialQuantization =
            0.25 * resolution;
        profile.meshOptions.maximumPlanarCellCount =
            250'000u;
        profile.meshOptions.maximumTriangleCount =
            2'000'000u;
    } else {
        profile.quality =
            IpwDisplayQuality::Detailed;
        // 高分辨率材料网格上的自由曲面可能让每个 XY 单元都产生独立
        // 水平片。保留 1/8 单元的轴向显示量化，使详细档仍能在 OCCT
        // 三角形预算内发布；材料真值、体积和比较结果不受影响。
        profile.meshOptions.axialQuantization =
            0.125 * resolution;
        profile.meshOptions.maximumPlanarCellCount =
            500'000u;
        profile.meshOptions.maximumTriangleCount =
            4'000'000u;
    }
    return profile;
}

// 1..10 级仿真质量 → 每刀径单元数与最大单元数，
// 由此推导请求的 XY 采样分辨率与平面误差上界。
inline IpwSimulationQualityProfile ipwSimulationQualityProfile(
    int level,
    double referenceCutterDiameter)
{
    IpwSimulationQualityProfile profile;
    const int clamped = std::clamp(level, 1, 10);
    const double cutterDiameter =
        std::isfinite(referenceCutterDiameter) &&
                referenceCutterDiameter > 0.0
        ? referenceCutterDiameter
        : 1.0;
    if (clamped <= 3) {
        profile.quality =
            IpwDisplayQuality::Interactive;
        profile.cellsPerCutterDiameter = 20.0;
        profile.maximumCellCount = 1'000'000;
    } else if (clamped <= 7) {
        profile.quality =
            IpwDisplayQuality::Balanced;
        profile.cellsPerCutterDiameter = 40.0;
        profile.maximumCellCount = 4'000'000;
    } else {
        profile.quality =
            IpwDisplayQuality::Detailed;
        profile.cellsPerCutterDiameter = 80.0;
        profile.maximumCellCount = 8'000'000;
    }
    profile.requestedResolution =
        std::max(
            0.01,
            cutterDiameter /
                profile.cellsPerCutterDiameter);
    profile.requestedPlanarErrorUpperBound =
        profile.requestedResolution /
        std::sqrt(2.0);
    return profile;
}


} // namespace oap::toolpath

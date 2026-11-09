// 固定方向区间场的直接显示网格。
//
// 输入是按 UV 单元压平的有序 W 区间。网格提取只生成材料外表面：
// 水平面合并共面矩形，侧壁由相邻单元区间差得到，不创建 TopoDS 拓扑。
//
// 属于 oap_toolpath_pocket 的毛坯仿真层：DexelInterval / DexelDisplayGrid
// 是本模块区间的公共数据约定，既被 MultiDexelStock 的内部实现使用，
// 也被 Z-map 差异显示（detail/PocketZMapDisplay.cpp）复用。
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include <Poly_Triangulation.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath {

// dexel 区间：单格在 W 方向的材料上下界。
struct DexelInterval {
    double lower = 0.0;
    double upper = 0.0;

    bool operator==(const DexelInterval&) const = default;
};

// dexel 显示网格：按 UV 单元压平的有序 W 区间。
struct DexelDisplayGrid {
    gp_Pnt origin;
    gp_Vec uAxis;
    gp_Vec vAxis;
    gp_Vec wAxis;
    double uMin = 0.0;
    double uMax = 0.0;
    double vMin = 0.0;
    double vMax = 0.0;
    double resolution = 0.0;
    double tolerance = 1.0e-5;
    int nx = 0;
    int ny = 0;
    // offsets.size() == nx * ny + 1；每个单元引用 intervals 中一段
    // 按 lower 递增、互不重叠的材料区间。
    std::vector<std::uint32_t> offsets;
    std::vector<DexelInterval> intervals;

    // 不变式：offsets 恰好为 cellCount()+1 个、首尾分别对齐 0 与 intervals
    // 总数，且每个单元内区间 lower 严格递增、互不重叠。valid() 一次性校验
    // 全部约束；越界访问由 intervalsAt() 以空 span 兜底。
    bool valid() const noexcept;
    std::size_t cellCount() const noexcept;
    // 返回 (i,j) 单元的区间视图；参数越界或网格不合法时返回空 span。
    std::span<const DexelInterval> intervalsAt(
        int i,
        int j) const noexcept;
};

// dexel 显示网格统计：单元/区间/面片计数与耗时。
struct DexelDisplayMeshMetrics {
    double milliseconds = 0.0;
    double sourceIntervalVolume = 0.0;
    double intervalVolume = 0.0;
    double maximumAxialError = 0.0;
    // 1 表示直接使用源 XY 网格;大于 1 的值描述仅用于显示的平面聚合步距。
    int planarDecimationStride = 1;
    double planarErrorUpperBound = 0.0;
    int vertexCount = 0;
    int triangleCount = 0;
    int horizontalQuadCount = 0;
    int sideQuadCount = 0;
};

// dexel 显示网格构建选项：聚合步长与容差。
struct DexelDisplayMeshOptions {
    // 仅影响派生显示网格，不修改材料真值。0 表示保留原始区间高度；
    // 正值会把区间端点量化到该轴向步长，以减少自由曲面显示三角形。
    double axialQuantization = 0.0;
    // 仅用于显示的稠密平面 interval 单元上限。0 表示保持源网格不变。
    // 聚合保留 interval 体积,且从不改动权威的 Multi-Dexel 毛坯/检查点。
    std::size_t maximumPlanarCellCount = 0u;
    std::size_t maximumTriangleCount = 2'000'000u;
};

// 从区间场提取外表面三角网格。output 仅接收合法材料网格（失败时置空），
// error/metrics 为可选输出，cancellationRequested 返回 true 时中止并报
// 「cancelled」错误。options 只影响显示产物，从不修改 grid 的权威区间。
bool buildDexelDisplayMesh(
    const DexelDisplayGrid& grid,
    Handle(Poly_Triangulation)& output,
    DexelDisplayMeshMetrics* metrics = nullptr,
    std::string* error = nullptr,
    std::function<bool()> cancellationRequested = {},
    const DexelDisplayMeshOptions& options = {});

// 对已经生成的统一 IPW 网格计算确定性内容指纹，供 OCCT/OpenGL
// 确认使用同一份 Multi-Dexel 显示产物。
std::string displayMeshContentFingerprint(
    const Handle(Poly_Triangulation)& mesh);

} // namespace oap::toolpath

// buildDexelDisplayMesh 实现（oap_toolpath_pocket）。
// 把按 UV 单元压平的有序 W 区间提取为材料外表面显示网格：水平面合并共面
// 矩形、侧壁取相邻单元区间差，不创建 TopoDS 拓扑。参见 DexelDisplayMesh.h。
#include "toolpath/pocket/simulation/DexelDisplayMesh.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <Poly_Triangle.hxx>
#include <Standard_Failure.hxx>
#include <gp_Dir.hxx>

namespace oap::toolpath {
namespace {

using SteadyClock = std::chrono::steady_clock;

class Fnv1a64 {
public:
    void add(const void* data, std::size_t size) noexcept
    {
        const auto* bytes =
            static_cast<const unsigned char*>(data);
        for (std::size_t index = 0; index < size; ++index) {
            m_value ^= bytes[index];
            m_value *= 1099511628211ull;
        }
    }

    template <typename Value>
    void addValue(const Value& value) noexcept
    {
        add(&value, sizeof(value));
    }

    std::uint64_t value() const noexcept
    {
        return m_value;
    }

private:
    std::uint64_t m_value = 14695981039346656037ull;
};

double elapsedMilliseconds(
    const SteadyClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start)
        .count();
}

double cellU0(const DexelDisplayGrid& grid, int i)
{
    return grid.uMin +
           static_cast<double>(i) * grid.resolution;
}

double cellU1(const DexelDisplayGrid& grid, int i)
{
    return std::min(
        grid.uMax, cellU0(grid, i) + grid.resolution);
}

double cellV0(const DexelDisplayGrid& grid, int j)
{
    return grid.vMin +
           static_cast<double>(j) * grid.resolution;
}

double cellV1(const DexelDisplayGrid& grid, int j)
{
    return std::min(
        grid.vMax, cellV0(grid, j) + grid.resolution);
}

gp_Pnt localToWorld(
    const DexelDisplayGrid& grid,
    double u,
    double v,
    double w)
{
    gp_Pnt result = grid.origin;
    result.Translate(grid.uAxis.Multiplied(u));
    result.Translate(grid.vAxis.Multiplied(v));
    result.Translate(grid.wAxis.Multiplied(w));
    return result;
}

// 计算 material 中未被 neighbor 覆盖的暴露区间段，用于相邻单元侧壁生成。
// 两个输入均为有序不重叠区间；本函数保持输出同序且合并相邻段，
// 与 DexelIntervalOps 的减法语义一致但以「段差」形式返回。
std::vector<DexelInterval> difference(
    std::span<const DexelInterval> material,
    std::span<const DexelInterval> neighbor,
    double tolerance)
{
    std::vector<DexelInterval> result;
    for (const DexelInterval& source : material) {
        double cursor = source.lower;
        for (const DexelInterval& cover : neighbor) {
            if (cover.upper <= cursor + tolerance)
                continue;
            if (cover.lower >= source.upper - tolerance)
                break;
            if (cover.lower > cursor + tolerance) {
                result.push_back({
                    cursor,
                    std::min(source.upper, cover.lower)});
            }
            cursor = std::max(cursor, cover.upper);
            if (cursor >= source.upper - tolerance)
                break;
        }
        if (cursor < source.upper - tolerance) {
            result.push_back({cursor, source.upper});
        }
    }
    return result;
}

double intervalVolume(const DexelDisplayGrid& grid)
{
    double volume = 0.0;
    for (int j = 0; j < grid.ny; ++j) {
        for (int i = 0; i < grid.nx; ++i) {
            const double area =
                (cellU1(grid, i) -
                 cellU0(grid, i)) *
                (cellV1(grid, j) -
                 cellV0(grid, j));
            for (const DexelInterval& interval :
                 grid.intervalsAt(i, j)) {
                volume +=
                    (interval.upper - interval.lower) *
                    area;
            }
        }
    }
    return volume;
}

// 按显示预算推导平面聚合步距：先按面积比开方估计初始步距，
// 再向上调整直至聚合后的单元数不超过上限。
int planarAggregationStride(
    const DexelDisplayGrid& grid,
    std::size_t maximumCellCount)
{
    if (maximumCellCount == 0u ||
        grid.cellCount() <= maximumCellCount ||
        grid.intervals.size() <= maximumCellCount) {
        return 1;
    }
    const double ratio =
        static_cast<double>(grid.cellCount()) /
        static_cast<double>(maximumCellCount);
    int stride = std::max(
        2, static_cast<int>(std::ceil(std::sqrt(ratio))));
    auto aggregatedCellCount = [&](int candidate) {
        const std::size_t nx =
            (static_cast<std::size_t>(grid.nx) +
             static_cast<std::size_t>(candidate) - 1u) /
            static_cast<std::size_t>(candidate);
        const std::size_t ny =
            (static_cast<std::size_t>(grid.ny) +
             static_cast<std::size_t>(candidate) - 1u) /
            static_cast<std::size_t>(candidate);
        return nx * ny;
    };
    while (aggregatedCellCount(stride) > maximumCellCount)
        ++stride;
    return stride;
}

bool aggregatePlanarDisplayGrid(
    const DexelDisplayGrid& source,
    int stride,
    DexelDisplayGrid& output,
    const std::function<bool()>& cancellationRequested,
    std::string& error)
{
    if (stride <= 1) {
        output = source;
        return true;
    }
    output = {};
    output.origin = source.origin;
    output.uAxis = source.uAxis;
    output.vAxis = source.vAxis;
    output.wAxis = source.wAxis;
    output.uMin = source.uMin;
    output.uMax = source.uMax;
    output.vMin = source.vMin;
    output.vMax = source.vMax;
    output.resolution =
        source.resolution * static_cast<double>(stride);
    output.tolerance = source.tolerance;
    output.nx = (source.nx + stride - 1) / stride;
    output.ny = (source.ny + stride - 1) / stride;
    output.offsets.reserve(output.cellCount() + 1u);
    output.intervals.reserve(std::min(
        source.intervals.size(), output.cellCount()));
    output.offsets.push_back(0u);

    struct WeightedInterval {
        double volume = 0.0;
        double firstMoment = 0.0;
    };
    std::vector<WeightedInterval> accumulated;
    std::vector<DexelInterval> representatives;
    // 每个粗单元先按层下标对齐累加体积与一阶矩，再换算成「厚度不变、
    // 质心正确的代表区间」；这样平面聚合只损失 XY 分辨率，不损失区间体积。
    for (int coarseJ = 0; coarseJ < output.ny; ++coarseJ) {
        if (cancellationRequested && cancellationRequested()) {
            error =
                "dexel display planar aggregation was cancelled";
            return false;
        }
        const int sourceJ0 = coarseJ * stride;
        const int sourceJ1 =
            std::min(source.ny, sourceJ0 + stride);
        for (int coarseI = 0; coarseI < output.nx; ++coarseI) {
            const int sourceI0 = coarseI * stride;
            const int sourceI1 =
                std::min(source.nx, sourceI0 + stride);
            accumulated.clear();
            double blockArea = 0.0;
            for (int j = sourceJ0; j < sourceJ1; ++j) {
                const double v0 = cellV0(source, j);
                const double v1 = cellV1(source, j);
                for (int i = sourceI0; i < sourceI1; ++i) {
                    const double area =
                        (cellU1(source, i) -
                         cellU0(source, i)) *
                        (v1 - v0);
                    blockArea += area;
                    const auto intervals =
                        source.intervalsAt(i, j);
                    if (accumulated.size() < intervals.size())
                        accumulated.resize(intervals.size());
                    for (std::size_t index = 0;
                         index < intervals.size(); ++index) {
                        const DexelInterval& interval =
                            intervals[index];
                        const double volume =
                            (interval.upper - interval.lower) *
                            area;
                        accumulated[index].volume += volume;
                        accumulated[index].firstMoment +=
                            0.5 *
                            (interval.lower + interval.upper) *
                            volume;
                    }
                }
            }

            representatives.clear();
            if (blockArea > 0.0) {
                for (const WeightedInterval& weighted :
                     accumulated) {
                    if (weighted.volume <=
                        source.tolerance * blockArea) {
                        continue;
                    }
                    const double thickness =
                        weighted.volume / blockArea;
                    const double midpoint =
                        weighted.firstMoment / weighted.volume;
                    representatives.push_back({
                        midpoint - 0.5 * thickness,
                        midpoint + 0.5 * thickness});
                }
            }
            std::sort(
                representatives.begin(), representatives.end(),
                [](const DexelInterval& left,
                   const DexelInterval& right) {
                    return left.lower < right.lower;
                });

            // 若取平均后的 interval 层互相重叠,则围绕体积质心合并它们的厚度。
            // 这样既保留每个粗网格单元的体积,又恢复有序、不重叠的不变式。
            for (const DexelInterval& interval : representatives) {
                DexelInterval merged = interval;
                while (!output.intervals.empty() &&
                       output.offsets.back() <
                           output.intervals.size() &&
                       merged.lower <=
                           output.intervals.back().upper +
                               source.tolerance) {
                    const DexelInterval previous =
                        output.intervals.back();
                    output.intervals.pop_back();
                    const double previousThickness =
                        previous.upper - previous.lower;
                    const double mergedThickness =
                        merged.upper - merged.lower;
                    const double totalThickness =
                        previousThickness + mergedThickness;
                    const double midpoint =
                        (0.5 * (previous.lower + previous.upper) *
                             previousThickness +
                         0.5 * (merged.lower + merged.upper) *
                             mergedThickness) /
                        totalThickness;
                    merged = {
                        midpoint - 0.5 * totalThickness,
                        midpoint + 0.5 * totalThickness};
                }
                output.intervals.push_back(merged);
            }
            if (output.intervals.size() >
                std::numeric_limits<std::uint32_t>::max()) {
                error =
                    "aggregated dexel interval count exceeds display capacity";
                return false;
            }
            output.offsets.push_back(
                static_cast<std::uint32_t>(
                    output.intervals.size()));
        }
    }
    if (!output.valid()) {
        error = "planar dexel display aggregation is invalid";
        return false;
    }
    return true;
}

} // namespace

std::size_t DexelDisplayGrid::cellCount() const noexcept
{
    if (nx <= 0 || ny <= 0)
        return 0u;
    return static_cast<std::size_t>(nx) *
           static_cast<std::size_t>(ny);
}

bool DexelDisplayGrid::valid() const noexcept
{
    const std::size_t count = cellCount();
    if (count == 0u ||
        offsets.size() != count + 1u ||
        offsets.front() != 0u ||
        offsets.back() != intervals.size() ||
        !std::isfinite(resolution) ||
        resolution <= 0.0 ||
        !std::isfinite(uMin) ||
        !std::isfinite(uMax) ||
        !std::isfinite(vMin) ||
        !std::isfinite(vMax) ||
        uMax <= uMin ||
        vMax <= vMin ||
        uAxis.Magnitude() <= 0.0 ||
        vAxis.Magnitude() <= 0.0 ||
        wAxis.Magnitude() <= 0.0) {
        return false;
    }
    for (std::size_t cell = 0; cell < count; ++cell) {
        if (offsets[cell] > offsets[cell + 1u] ||
            offsets[cell + 1u] > intervals.size()) {
            return false;
        }
        double previousUpper =
            -std::numeric_limits<double>::infinity();
        for (std::uint32_t index = offsets[cell];
             index < offsets[cell + 1u]; ++index) {
            const DexelInterval& interval =
                intervals[index];
            if (!std::isfinite(interval.lower) ||
                !std::isfinite(interval.upper) ||
                interval.upper <= interval.lower ||
                interval.lower < previousUpper) {
                return false;
            }
            previousUpper = interval.upper;
        }
    }
    return true;
}

std::span<const DexelInterval>
DexelDisplayGrid::intervalsAt(
    int i,
    int j) const noexcept
{
    if (i < 0 || j < 0 || i >= nx || j >= ny ||
        offsets.size() != cellCount() + 1u) {
        return {};
    }
    const std::size_t cell =
        static_cast<std::size_t>(j) *
            static_cast<std::size_t>(nx) +
        static_cast<std::size_t>(i);
    const std::uint32_t begin = offsets[cell];
    const std::uint32_t end = offsets[cell + 1u];
    if (begin > end || end > intervals.size())
        return {};
    if (begin == end)
        return {};
    return {
        intervals.data() + begin,
        static_cast<std::size_t>(end - begin)};
}

bool buildDexelDisplayMesh(
    const DexelDisplayGrid& sourceGrid,
    Handle(Poly_Triangulation)& output,
    DexelDisplayMeshMetrics* metrics,
    std::string* error,
    std::function<bool()> cancellationRequested,
    const DexelDisplayMeshOptions& options)
{
    const SteadyClock::time_point start =
        SteadyClock::now();
    output.Nullify();
    if (metrics)
        *metrics = {};
    auto finishMetrics = [&] {
        if (metrics) {
            metrics->milliseconds =
                elapsedMilliseconds(start);
        }
    };
    auto fail = [&](std::string message) {
        finishMetrics();
        if (error)
            *error = std::move(message);
        return false;
    };
    if (!sourceGrid.valid())
        return fail("cannot mesh an invalid dexel interval grid");
    if (sourceGrid.intervals.empty())
        return fail("dexel interval grid contains no material");

    const int planarStride = planarAggregationStride(
        sourceGrid, options.maximumPlanarCellCount);
    // 主流程：先按预算做平面聚合（可选），再做轴向量化（可选），
    // 最后只对「聚合+量化」后的显示网格提取水平面与侧壁四边形。
    // 源区间场始终保持权威，显示网格绝不回写毛坯。
    DexelDisplayGrid planarGrid;
    std::string planarError;
    if (planarStride > 1 &&
        !aggregatePlanarDisplayGrid(
            sourceGrid, planarStride, planarGrid,
            cancellationRequested, planarError)) {
        return fail(std::move(planarError));
    }
    const DexelDisplayGrid& planarSource =
        planarStride > 1 ? planarGrid : sourceGrid;

    DexelDisplayGrid quantizedGrid;
    const DexelDisplayGrid* displayGrid =
        &planarSource;
    const double quantization =
        std::isfinite(options.axialQuantization)
        ? std::max(0.0, options.axialQuantization)
        : 0.0;
    double maximumAxialError = 0.0;
    if (quantization >
        std::max(
            planarSource.tolerance,
            planarSource.resolution * 1.0e-9)) {
        quantizedGrid = planarSource;
        quantizedGrid.offsets.clear();
        quantizedGrid.intervals.clear();
        quantizedGrid.offsets.reserve(
            planarSource.cellCount() + 1u);
        quantizedGrid.intervals.reserve(
            planarSource.intervals.size());
        quantizedGrid.offsets.push_back(0u);
        for (int j = 0; j < planarSource.ny; ++j) {
            if (cancellationRequested &&
                cancellationRequested()) {
                return fail(
                    "dexel display mesh quantization was cancelled");
            }
            for (int i = 0; i < planarSource.nx; ++i) {
                for (const DexelInterval& source :
                     planarSource.intervalsAt(i, j)) {
                    DexelInterval interval{
                        std::round(
                            source.lower / quantization) *
                            quantization,
                        std::round(
                            source.upper / quantization) *
                            quantization};
                    if (interval.upper - interval.lower <=
                            planarSource.tolerance) {
                        const double midpoint =
                            0.5 *
                            (source.lower + source.upper);
                        interval.lower =
                            midpoint -
                            0.5 * quantization;
                        interval.upper =
                            midpoint +
                            0.5 * quantization;
                    }
                    maximumAxialError = std::max({
                        maximumAxialError,
                        std::abs(
                            interval.lower -
                            source.lower),
                        std::abs(
                            interval.upper -
                            source.upper)});
                    if (!quantizedGrid.intervals.empty() &&
                        quantizedGrid.offsets.back() <
                            quantizedGrid.intervals.size() &&
                        interval.lower <=
                            quantizedGrid.intervals.back()
                                    .upper +
                                planarSource.tolerance) {
                        quantizedGrid.intervals.back().upper =
                            std::max(
                                quantizedGrid.intervals.back()
                                    .upper,
                                interval.upper);
                    } else {
                        quantizedGrid.intervals.push_back(
                            interval);
                    }
                }
                if (quantizedGrid.intervals.size() >
                    std::numeric_limits<
                        std::uint32_t>::max()) {
                    return fail(
                        "quantized dexel interval count exceeds display capacity");
                }
                quantizedGrid.offsets.push_back(
                    static_cast<std::uint32_t>(
                        quantizedGrid.intervals.size()));
            }
        }
        displayGrid = &quantizedGrid;
    }
    const DexelDisplayGrid& grid = *displayGrid;

    struct MeshVertex {
        gp_Pnt point;
        gp_Dir normal;
    };
    const std::size_t maximumTriangleCount =
        std::clamp<std::size_t>(
            options.maximumTriangleCount,
            2u, 20'000'000u);
    std::vector<MeshVertex> vertices;
    vertices.reserve(std::min<std::size_t>(
        600'000u,
        std::max<std::size_t>(
            1u, grid.intervals.size())));
    int horizontalQuads = 0;
    int sideQuads = 0;
    bool budgetExceeded = false;
    auto addQuad = [&](
                       const gp_Pnt& p0,
                       const gp_Pnt& p1,
                       const gp_Pnt& p2,
                       const gp_Pnt& p3,
                       const gp_Dir& normal,
                       bool horizontal) {
        if (vertices.size() / 3u + 2u >
            maximumTriangleCount) {
            budgetExceeded = true;
            return;
        }
        vertices.push_back({p0, normal});
        vertices.push_back({p1, normal});
        vertices.push_back({p2, normal});
        vertices.push_back({p0, normal});
        vertices.push_back({p2, normal});
        vertices.push_back({p3, normal});
        if (horizontal)
            ++horizontalQuads;
        else
            ++sideQuads;
    };
    auto world = [&](double u, double v, double w) {
        return localToWorld(grid, u, v, w);
    };

    try {
        const double mergeTolerance = std::max(
            10.0 * grid.tolerance,
            grid.resolution * 1.0e-7);
        auto emitHorizontal = [&](bool upperSurface) {
            // 水平面提取：对每个未消费的区间端点，向 U/V 两个方向贪心扩展
            // 成最大共面矩形（同层同端点高度），一次输出一个合并四边形；
            // used[] 标记已消费的「单元×区间层」，保证每层只生成一次。
            std::vector<unsigned char> used(
                grid.intervals.size(), 0u);
            const gp_Dir normal(
                upperSurface
                    ? grid.wAxis
                    : grid.wAxis.Reversed());
            for (int j = 0; j < grid.ny; ++j) {
                if (cancellationRequested &&
                    cancellationRequested()) {
                    return false;
                }
                for (int i = 0; i < grid.nx; ++i) {
                    const std::size_t firstCell =
                        static_cast<std::size_t>(j) *
                            static_cast<std::size_t>(
                                grid.nx) +
                        static_cast<std::size_t>(i);
                    const auto firstIntervals =
                        grid.intervalsAt(i, j);
                    for (std::size_t intervalIndex = 0;
                         intervalIndex <
                             firstIntervals.size();
                         ++intervalIndex) {
                        const std::size_t firstFlat =
                            grid.offsets[firstCell] +
                            intervalIndex;
                        if (used[firstFlat] != 0u)
                            continue;
                        const double w =
                            upperSurface
                            ? firstIntervals[intervalIndex]
                                  .upper
                            : firstIntervals[intervalIndex]
                                  .lower;
                        int width = 1;
                        while (i + width < grid.nx) {
                            const std::size_t candidate =
                                static_cast<std::size_t>(j) *
                                    static_cast<std::size_t>(
                                        grid.nx) +
                                static_cast<std::size_t>(
                                    i + width);
                            const auto intervals =
                                grid.intervalsAt(
                                    i + width, j);
                            if (intervalIndex >=
                                    intervals.size() ||
                                used[
                                    grid.offsets[candidate] +
                                    intervalIndex] != 0u) {
                                break;
                            }
                            const double candidateW =
                                upperSurface
                                ? intervals[intervalIndex]
                                      .upper
                                : intervals[intervalIndex]
                                      .lower;
                            if (std::abs(candidateW - w) >
                                mergeTolerance) {
                                break;
                            }
                            ++width;
                        }
                        int height = 1;
                        for (; j + height < grid.ny;
                             ++height) {
                            bool matches = true;
                            for (int x = 0; x < width; ++x) {
                                const int candidateI = i + x;
                                const int candidateJ =
                                    j + height;
                                const std::size_t candidate =
                                    static_cast<std::size_t>(
                                        candidateJ) *
                                        static_cast<std::size_t>(
                                            grid.nx) +
                                    static_cast<std::size_t>(
                                        candidateI);
                                const auto intervals =
                                    grid.intervalsAt(
                                        candidateI,
                                        candidateJ);
                                if (intervalIndex >=
                                        intervals.size() ||
                                    used[
                                        grid.offsets[
                                            candidate] +
                                        intervalIndex] !=
                                        0u) {
                                    matches = false;
                                    break;
                                }
                                const double candidateW =
                                    upperSurface
                                    ? intervals[
                                          intervalIndex]
                                          .upper
                                    : intervals[
                                          intervalIndex]
                                          .lower;
                                if (std::abs(
                                        candidateW - w) >
                                    mergeTolerance) {
                                    matches = false;
                                    break;
                                }
                            }
                            if (!matches)
                                break;
                        }
                        for (int y = 0; y < height; ++y) {
                            for (int x = 0; x < width; ++x) {
                                const std::size_t cell =
                                    static_cast<std::size_t>(
                                        j + y) *
                                        static_cast<std::size_t>(
                                            grid.nx) +
                                    static_cast<std::size_t>(
                                        i + x);
                                used[
                                    grid.offsets[cell] +
                                    intervalIndex] = 1u;
                            }
                        }
                        const double u0 = cellU0(grid, i);
                        const double u1 =
                            cellU1(grid, i + width - 1);
                        const double v0 = cellV0(grid, j);
                        const double v1 =
                            cellV1(grid, j + height - 1);
                        if (upperSurface) {
                            addQuad(
                                world(u0, v0, w),
                                world(u1, v0, w),
                                world(u1, v1, w),
                                world(u0, v1, w),
                                normal, true);
                        } else {
                            addQuad(
                                world(u0, v0, w),
                                world(u0, v1, w),
                                world(u1, v1, w),
                                world(u1, v0, w),
                                normal, true);
                        }
                        if (budgetExceeded)
                            return false;
                    }
                }
            }
            return true;
        };
        if (!emitHorizontal(true) ||
            !emitHorizontal(false)) {
            if (budgetExceeded) {
                return fail(
                    "dexel display mesh exceeded its triangle budget");
            }
            return fail(
                "dexel display mesh was cancelled");
        }

        const gp_Dir positiveU(grid.uAxis);
        const gp_Dir negativeU(grid.uAxis.Reversed());
        const gp_Dir positiveV(grid.vAxis);
        const gp_Dir negativeV(grid.vAxis.Reversed());
        // 侧壁生成：对每条 U 向格线，用 difference() 求出左右单元暴露段，
        // 正向暴露输出 +U 法线侧壁、反向暴露输出 -U 法线侧壁；V 向同理。
        for (int boundary = 0;
             boundary <= grid.nx; ++boundary) {
            for (int j = 0; j < grid.ny; ++j) {
                if (cancellationRequested &&
                    cancellationRequested()) {
                    return fail(
                        "dexel display mesh was cancelled");
                }
                const auto left =
                    grid.intervalsAt(boundary - 1, j);
                const auto right =
                    grid.intervalsAt(boundary, j);
                const double u =
                    boundary == grid.nx
                    ? grid.uMax
                    : cellU0(grid, boundary);
                const double v0 = cellV0(grid, j);
                const double v1 = cellV1(grid, j);
                for (const DexelInterval& exposed :
                     difference(
                         left, right, mergeTolerance)) {
                    addQuad(
                        world(u, v0, exposed.lower),
                        world(u, v1, exposed.lower),
                        world(u, v1, exposed.upper),
                        world(u, v0, exposed.upper),
                        positiveU, false);
                }
                for (const DexelInterval& exposed :
                     difference(
                         right, left, mergeTolerance)) {
                    addQuad(
                        world(u, v0, exposed.lower),
                        world(u, v0, exposed.upper),
                        world(u, v1, exposed.upper),
                        world(u, v1, exposed.lower),
                        negativeU, false);
                }
                if (budgetExceeded) {
                    return fail(
                        "dexel display mesh exceeded its triangle budget");
                }
            }
        }
        for (int boundary = 0;
             boundary <= grid.ny; ++boundary) {
            for (int i = 0; i < grid.nx; ++i) {
                if (cancellationRequested &&
                    cancellationRequested()) {
                    return fail(
                        "dexel display mesh was cancelled");
                }
                const auto lower =
                    grid.intervalsAt(i, boundary - 1);
                const auto upper =
                    grid.intervalsAt(i, boundary);
                const double v =
                    boundary == grid.ny
                    ? grid.vMax
                    : cellV0(grid, boundary);
                const double u0 = cellU0(grid, i);
                const double u1 = cellU1(grid, i);
                for (const DexelInterval& exposed :
                     difference(
                         lower, upper, mergeTolerance)) {
                    addQuad(
                        world(u0, v, exposed.lower),
                        world(u0, v, exposed.upper),
                        world(u1, v, exposed.upper),
                        world(u1, v, exposed.lower),
                        positiveV, false);
                }
                for (const DexelInterval& exposed :
                     difference(
                         upper, lower, mergeTolerance)) {
                    addQuad(
                        world(u0, v, exposed.lower),
                        world(u1, v, exposed.lower),
                        world(u1, v, exposed.upper),
                        world(u0, v, exposed.upper),
                        negativeV, false);
                }
                if (budgetExceeded) {
                    return fail(
                        "dexel display mesh exceeded its triangle budget");
                }
            }
        }
        if (vertices.empty()) {
            return fail(
                "dexel display mesh contains no triangles");
        }
        if (vertices.size() >
            static_cast<std::size_t>(
                std::numeric_limits<Standard_Integer>::
                    max())) {
            return fail(
                "dexel display mesh exceeds OCCT index capacity");
        }

        const Standard_Integer nodeCount =
            static_cast<Standard_Integer>(
                vertices.size());
        const Standard_Integer triangleCount =
            nodeCount / 3;
        Handle(Poly_Triangulation) triangulation =
            new Poly_Triangulation(
                nodeCount, triangleCount,
                Standard_False, Standard_True);
        for (Standard_Integer index = 1;
             index <= nodeCount; ++index) {
            const MeshVertex& vertex =
                vertices[static_cast<std::size_t>(
                    index - 1)];
            triangulation->SetNode(
                index, vertex.point);
            triangulation->SetNormal(
                index, vertex.normal);
        }
        for (Standard_Integer triangle = 1;
             triangle <= triangleCount; ++triangle) {
            const Standard_Integer first =
                (triangle - 1) * 3 + 1;
            triangulation->SetTriangle(
                triangle,
                Poly_Triangle(
                    first, first + 1, first + 2));
        }
        triangulation->Deflection(grid.resolution);
        output = std::move(triangulation);
        if (metrics) {
            metrics->sourceIntervalVolume =
                intervalVolume(sourceGrid);
            metrics->vertexCount = nodeCount;
            metrics->triangleCount =
                triangleCount;
            metrics->horizontalQuadCount =
                horizontalQuads;
            metrics->sideQuadCount =
                sideQuads;
            metrics->intervalVolume =
                intervalVolume(grid);
            metrics->maximumAxialError =
                maximumAxialError;
            metrics->planarDecimationStride =
                planarStride;
            metrics->planarErrorUpperBound =
                planarSource.resolution / std::sqrt(2.0);
        }
        finishMetrics();
        if (error)
            error->clear();
        return true;
    } catch (const Standard_Failure& failure) {
        const char* message =
            failure.GetMessageString();
        return fail(
            std::string(
                "OCCT failed while building dexel display mesh: ") +
            (message ? message : "unknown OCCT failure"));
    } catch (const std::exception& failure) {
        return fail(
            std::string(
                "failed to build dexel display mesh: ") +
            failure.what());
    } catch (...) {
        return fail(
            "unknown failure while building dexel display mesh");
    }
}

std::string displayMeshContentFingerprint(
    const Handle(Poly_Triangulation)& mesh)
{
    if (mesh.IsNull() || mesh->NbNodes() <= 0 ||
        mesh->NbTriangles() <= 0) {
        return {};
    }
    Fnv1a64 hash;
    const Standard_Integer nodeCount = mesh->NbNodes();
    const Standard_Integer triangleCount =
        mesh->NbTriangles();
    const Standard_Boolean hasNormals = mesh->HasNormals();
    hash.addValue(nodeCount);
    hash.addValue(triangleCount);
    hash.addValue(hasNormals);
    for (Standard_Integer index = 1;
         index <= nodeCount; ++index) {
        const gp_Pnt point = mesh->Node(index);
        const double coordinates[] = {
            point.X(), point.Y(), point.Z()};
        hash.add(coordinates, sizeof(coordinates));
        if (hasNormals) {
            const gp_Dir normal = mesh->Normal(index);
            const double components[] = {
                normal.X(), normal.Y(), normal.Z()};
            hash.add(components, sizeof(components));
        }
    }
    for (Standard_Integer index = 1;
         index <= triangleCount; ++index) {
        Standard_Integer first = 0;
        Standard_Integer second = 0;
        Standard_Integer third = 0;
        mesh->Triangle(index).Get(
            first, second, third);
        const Standard_Integer nodes[] = {
            first, second, third};
        hash.add(nodes, sizeof(nodes));
    }
    std::ostringstream output;
    output << "mesh-fnv1a64-"
           << std::hex << std::setw(16)
           << std::setfill('0') << hash.value();
    return output.str();
}

} // namespace oap::toolpath

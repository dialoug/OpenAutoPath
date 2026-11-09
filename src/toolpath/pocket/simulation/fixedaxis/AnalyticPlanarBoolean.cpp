// AnalyticPlanarBoolean 实现（oap_toolpath_pocket 的 fixedaxis 私有目录）。
// 同平面解析 Face 的空间分组并集：非相交区域直接保留，相交组只提交一次
// 多参数 OCCT Fuse，避免随原语数量增长的逐条布尔链。参见 AnalyticPlanarBoolean.h。
#include "toolpath/pocket/simulation/fixedaxis/AnalyticPlanarBoolean.h"
#include "toolpath/core/ToolpathOcctParallel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pln.hxx>

namespace oap::toolpath {
namespace {

using Clock = std::chrono::steady_clock;
constexpr std::int64_t kMaximumCellsPerFace = 1000000;

struct Bounds3d {
    double minimumX = 0.0;
    double minimumY = 0.0;
    double minimumZ = 0.0;
    double maximumX = 0.0;
    double maximumY = 0.0;
    double maximumZ = 0.0;
};

struct CellKey {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const CellKey&) const = default;
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const noexcept
    {
        const std::uint64_t x =
            static_cast<std::uint64_t>(key.x);
        const std::uint64_t y =
            static_cast<std::uint64_t>(key.y);
        const std::uint64_t z =
            static_cast<std::uint64_t>(key.z);
        const std::uint64_t xy =
            (x * 0x9e3779b185ebca87ULL) ^
            (y + 0x9e3779b97f4a7c15ULL +
             (x << 6U) + (x >> 2U));
        const std::uint64_t mixed =
            xy ^ (z + 0x517cc1b727220a95ULL +
                  (xy << 6U) + (xy >> 2U));
        return static_cast<std::size_t>(mixed);
    }
};

class CancellationProgressIndicator final
    : public Message_ProgressIndicator {
public:
    explicit CancellationProgressIndicator(
        std::function<bool()> cancellationRequested)
        : m_cancellationRequested(
              std::move(cancellationRequested))
    {
    }

protected:
    Standard_Boolean UserBreak() override
    {
        try {
            return m_cancellationRequested &&
                   m_cancellationRequested();
        } catch (...) {
            return Standard_True;
        }
    }

    void Show(const Message_ProgressScope&,
              const Standard_Boolean) override
    {
    }

private:
    std::function<bool()> m_cancellationRequested;
};

bool cancelled(
    const std::function<bool()>& cancellationRequested)
{
    if (!cancellationRequested)
        return false;
    try {
        return cancellationRequested();
    } catch (...) {
        return true;
    }
}

class DisjointSet {
public:
    explicit DisjointSet(std::size_t count)
        : m_parent(count), m_rank(count, 0u)
    {
        std::iota(m_parent.begin(), m_parent.end(), 0u);
    }

    std::size_t root(std::size_t value)
    {
        if (m_parent[value] != value)
            m_parent[value] = root(m_parent[value]);
        return m_parent[value];
    }

    void unite(std::size_t first, std::size_t second)
    {
        first = root(first);
        second = root(second);
        if (first == second)
            return;
        if (m_rank[first] < m_rank[second])
            std::swap(first, second);
        m_parent[second] = first;
        if (m_rank[first] == m_rank[second])
            ++m_rank[first];
    }

private:
    std::vector<std::size_t> m_parent;
    std::vector<unsigned char> m_rank;
};

bool finiteBounds(const Bounds3d& bounds)
{
    return std::isfinite(bounds.minimumX) &&
           std::isfinite(bounds.minimumY) &&
           std::isfinite(bounds.minimumZ) &&
           std::isfinite(bounds.maximumX) &&
           std::isfinite(bounds.maximumY) &&
           std::isfinite(bounds.maximumZ) &&
           bounds.minimumX <= bounds.maximumX &&
           bounds.minimumY <= bounds.maximumY &&
           bounds.minimumZ <= bounds.maximumZ;
}

bool faceBounds(const TopoDS_Face& face, Bounds3d& output)
{
    if (face.IsNull())
        return false;
    Bnd_Box box;
    box.SetGap(0.0);
    BRepBndLib::Add(face, box);
    if (box.IsVoid() || box.IsWhole())
        return false;
    box.Get(
        output.minimumX, output.minimumY, output.minimumZ,
        output.maximumX, output.maximumY, output.maximumZ);
    return finiteBounds(output);
}

bool overlaps(
    const Bounds3d& first,
    const Bounds3d& second,
    double tolerance)
{
    return first.maximumX + tolerance >= second.minimumX &&
           second.maximumX + tolerance >= first.minimumX &&
           first.maximumY + tolerance >= second.minimumY &&
           second.maximumY + tolerance >= first.minimumY &&
           first.maximumZ + tolerance >= second.minimumZ &&
           second.maximumZ + tolerance >= first.minimumZ;
}

double faceArea(const TopoDS_Face& face)
{
    GProp_GProps properties;
    BRepGProp::SurfaceProperties(face, properties);
    return std::abs(properties.Mass());
}

bool facePlane(const TopoDS_Face& face, gp_Pln& plane)
{
    const BRepAdaptor_Surface surface(face, Standard_True);
    if (surface.GetType() != GeomAbs_Plane)
        return false;
    plane = surface.Plane();
    return true;
}

bool samePlane(
    const gp_Pln& first,
    const gp_Pln& second,
    double tolerance)
{
    return std::abs(
               first.Axis().Direction().Dot(
                   second.Axis().Direction())) >=
               1.0 - 1.0e-8 &&
           first.Distance(second.Location()) <= tolerance;
}

bool validAnalyticFace(
    const TopoDS_Face& face,
    int& lineEdges,
    int& circleEdges,
    int& unsupportedEdges,
    double& area)
{
    if (face.IsNull() ||
        !BRepCheck_Analyzer(
             face, Standard_True,
             Standard_False)
             .IsValid()) {
        return false;
    }
    area = faceArea(face);
    if (!std::isfinite(area) || area <= 0.0)
        return false;
    for (TopExp_Explorer explorer(face, TopAbs_EDGE);
         explorer.More(); explorer.Next()) {
        const BRepAdaptor_Curve curve(
            TopoDS::Edge(explorer.Current()));
        if (curve.GetType() == GeomAbs_Line) {
            ++lineEdges;
        } else if (curve.GetType() == GeomAbs_Circle) {
            ++circleEdges;
        } else {
            ++unsupportedEdges;
        }
    }
    return unsupportedEdges == 0;
}

// 按 AABB 哈希格子把互相重叠（含容差）的 Face 聚成连通组：
// 组内任意成员经格子邻居传递合并，非相交组之间不产生关联，
// 从而把「N 条逐条布尔」降为「每组一次多参数 Fuse」。
std::vector<std::vector<std::size_t>> spatialGroups(
    const std::vector<Bounds3d>& bounds,
    double tolerance)
{
    std::vector<double> extents;
    extents.reserve(bounds.size());
    double originX = std::numeric_limits<double>::infinity();
    double originY = std::numeric_limits<double>::infinity();
    double originZ = std::numeric_limits<double>::infinity();
    for (const Bounds3d& item : bounds) {
        extents.push_back(std::max({
            item.maximumX - item.minimumX,
            item.maximumY - item.minimumY,
            item.maximumZ - item.minimumZ}));
        originX = std::min(originX, item.minimumX);
        originY = std::min(originY, item.minimumY);
        originZ = std::min(originZ, item.minimumZ);
    }
    std::nth_element(
        extents.begin(),
        extents.begin() +
            static_cast<std::ptrdiff_t>(extents.size() / 2),
        extents.end());
    const double cellSize = std::max(
        {16.0 * tolerance,
         extents[extents.size() / 2],
         Precision::Confusion()});
    const auto cell = [&](double value, double origin) {
        return static_cast<std::int64_t>(
            std::floor((value - origin) / cellSize));
    };

    DisjointSet sets(bounds.size());
    std::unordered_map<CellKey,
                       std::vector<std::size_t>,
                       CellKeyHash> cells;
    std::vector<std::size_t> globalIndices;
    for (std::size_t index = 0; index < bounds.size(); ++index) {
        const std::int64_t minimumX = cell(
            bounds[index].minimumX - tolerance, originX);
        const std::int64_t maximumX = cell(
            bounds[index].maximumX + tolerance, originX);
        const std::int64_t minimumY = cell(
            bounds[index].minimumY - tolerance, originY);
        const std::int64_t maximumY = cell(
            bounds[index].maximumY + tolerance, originY);
        const std::int64_t minimumZ = cell(
            bounds[index].minimumZ - tolerance, originZ);
        const std::int64_t maximumZ = cell(
            bounds[index].maximumZ + tolerance, originZ);
        const std::int64_t width = maximumX - minimumX + 1;
        const std::int64_t height = maximumY - minimumY + 1;
        const std::int64_t depth = maximumZ - minimumZ + 1;
        if (width <= 0 || height <= 0 ||
            depth <= 0 ||
            width > kMaximumCellsPerFace ||
            height > kMaximumCellsPerFace ||
            depth > kMaximumCellsPerFace ||
            width > kMaximumCellsPerFace / height ||
            width * height > kMaximumCellsPerFace / depth) {
            // 极端跨区 Face 直接与全部已知 AABB 比较，避免无界 tile 展开。
            for (std::size_t previous = 0;
                 previous < index; ++previous) {
                if (overlaps(
                        bounds[index], bounds[previous],
                        tolerance)) {
                    sets.unite(index, previous);
                }
            }
            globalIndices.push_back(index);
            continue;
        }
        for (const std::size_t previous : globalIndices) {
            if (overlaps(
                    bounds[index], bounds[previous],
                    tolerance)) {
                sets.unite(index, previous);
            }
        }
        for (std::int64_t x = minimumX; x <= maximumX; ++x) {
            for (std::int64_t y = minimumY; y <= maximumY; ++y) {
                for (std::int64_t z = minimumZ; z <= maximumZ; ++z) {
                    std::vector<std::size_t>& occupants =
                        cells[{x, y, z}];
                    for (const std::size_t previous : occupants) {
                        if (overlaps(
                                bounds[index], bounds[previous],
                                tolerance)) {
                            sets.unite(index, previous);
                        }
                    }
                    occupants.push_back(index);
                }
            }
        }
    }

    std::unordered_map<std::size_t,
                       std::vector<std::size_t>> grouped;
    for (std::size_t index = 0; index < bounds.size(); ++index)
        grouped[sets.root(index)].push_back(index);
    std::vector<std::vector<std::size_t>> result;
    result.reserve(grouped.size());
    for (auto& [root, members] : grouped) {
        static_cast<void>(root);
        std::sort(members.begin(), members.end());
        result.push_back(std::move(members));
    }
    std::sort(
        result.begin(), result.end(),
        [](const auto& first, const auto& second) {
            return first.front() < second.front();
        });
    return result;
}

// 对单个空间组执行一次多参数 BRepAlgoAPI_Fuse：首个面作参数、其余面作
// 工具，构造后简化结果并拆回 Face 列表；支持通过进度指示器取消。
bool fuseGroup(
    const std::vector<TopoDS_Face>& inputs,
    const std::vector<std::size_t>& group,
    double tolerance,
    bool runParallel,
    const std::function<bool()>& cancellationRequested,
    bool& wasCancelled,
    std::vector<TopoDS_Face>& output)
{
    if (cancelled(cancellationRequested)) {
        wasCancelled = true;
        return false;
    }
    if (group.size() == 1u) {
        output.push_back(inputs[group.front()]);
        return true;
    }
    TopTools_ListOfShape arguments;
    TopTools_ListOfShape tools;
    arguments.Append(inputs[group.front()]);
    for (std::size_t index = 1;
         index < group.size(); ++index) {
        tools.Append(inputs[group[index]]);
    }
    try {
        ToolpathOcctParallelLease lease(runParallel);
        BRepAlgoAPI_Fuse fuse;
        fuse.SetArguments(arguments);
        fuse.SetTools(tools);
        fuse.SetFuzzyValue(tolerance);
        fuse.SetRunParallel(lease.enabled());
        if (cancellationRequested) {
            const Handle(Message_ProgressIndicator) progress =
                new CancellationProgressIndicator(
                    cancellationRequested);
            fuse.Build(progress->Start());
        } else {
            fuse.Build();
        }
        if (cancelled(cancellationRequested)) {
            wasCancelled = true;
            return false;
        }
        if (!fuse.IsDone() || fuse.Shape().IsNull())
            return false;
        fuse.SimplifyResult(
            Standard_True, Standard_True);
        if (cancelled(cancellationRequested)) {
            wasCancelled = true;
            return false;
        }
        for (TopExp_Explorer explorer(
                 fuse.Shape(), TopAbs_FACE);
             explorer.More(); explorer.Next()) {
            output.push_back(
                TopoDS::Face(explorer.Current()));
        }
        return !output.empty();
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
}

} // namespace

// 把一组共平面、边类型仅为 Line/Circle 的 Face 求并。
// 主流程：校验同平面与解析边 → 空间分组 → 逐组 Fuse →
// 输出 Compound 与各统计量。任何一步失败都返回 error 而非半成品。
AnalyticPlanarBooleanResult AnalyticPlanarBoolean::unite(
    const std::vector<TopoDS_Face>& inputs,
    double sourceTolerance,
    bool runParallel,
    std::function<bool()> cancellationRequested) const
{
    AnalyticPlanarBooleanResult result;
    const Clock::time_point start = Clock::now();
    const auto finish = [&] {
        result.buildMilliseconds =
            std::chrono::duration<double, std::milli>(
                Clock::now() - start)
                .count();
        return result;
    };
    result.inputFaceCount = static_cast<int>(inputs.size());
    if (cancelled(cancellationRequested)) {
        result.cancelled = true;
        return finish();
    }
    if (!std::isfinite(sourceTolerance) ||
        sourceTolerance <= 0.0) {
        result.error =
            "analytic planar boolean requires a positive tolerance";
        return finish();
    }
    if (inputs.empty()) {
        result.error =
            "analytic planar boolean requires at least one face";
        return finish();
    }
    const double tolerance = std::max(
        sourceTolerance, Precision::Confusion());

    std::vector<Bounds3d> bounds(inputs.size());
    std::optional<gp_Pln> referencePlane;
    try {
        for (std::size_t index = 0; index < inputs.size(); ++index) {
            if (cancelled(cancellationRequested)) {
                result.cancelled = true;
                return finish();
            }
            int lines = 0;
            int circles = 0;
            int unsupported = 0;
            double area = 0.0;
            gp_Pln plane;
            if (!facePlane(inputs[index], plane) ||
                (referencePlane &&
                 !samePlane(
                     *referencePlane, plane,
                     tolerance)) ||
                !faceBounds(inputs[index], bounds[index]) ||
                !validAnalyticFace(
                    inputs[index], lines, circles,
                    unsupported, area)) {
                result.error =
                    "analytic planar boolean received an invalid or non-analytic face";
                return finish();
            }
            if (!referencePlane)
                referencePlane = plane;
            result.inputArea += area;
        }
    } catch (const Standard_Failure&) {
        result.error =
            "failed to inspect analytic planar boolean inputs";
        return finish();
    } catch (...) {
        result.error =
            "failed to inspect analytic planar boolean inputs";
        return finish();
    }

    const std::vector<std::vector<std::size_t>> groups =
        spatialGroups(bounds, tolerance);
    result.spatialGroupCount = static_cast<int>(groups.size());
    for (const std::vector<std::size_t>& group : groups) {
        if (cancelled(cancellationRequested)) {
            result.faces.clear();
            result.cancelled = true;
            return finish();
        }
        result.largestGroupSize = std::max(
            result.largestGroupSize,
            static_cast<int>(group.size()));
        if (group.size() > 1u)
            ++result.fusedGroupCount;
        const std::size_t outputBegin = result.faces.size();
        if (!fuseGroup(
                inputs, group, tolerance,
                runParallel, cancellationRequested,
                result.cancelled, result.faces) ||
            result.faces.size() == outputBegin) {
            result.faces.clear();
            if (result.cancelled)
                return finish();
            result.error =
                "analytic planar boolean failed for an intersecting spatial group";
            return finish();
        }
    }

    BRep_Builder compoundBuilder;
    TopoDS_Compound compound;
    compoundBuilder.MakeCompound(compound);
    try {
        for (const TopoDS_Face& face : result.faces) {
            if (cancelled(cancellationRequested)) {
                result.faces.clear();
                result.cancelled = true;
                return finish();
            }
            double area = 0.0;
            if (!validAnalyticFace(
                    face,
                    result.lineEdgeCount,
                    result.circleEdgeCount,
                    result.unsupportedEdgeCount,
                    area)) {
                result.faces.clear();
                result.error =
                    "analytic planar boolean produced invalid topology or lost curve semantics";
                return finish();
            }
            result.unionArea += area;
            compoundBuilder.Add(compound, face);
        }
    } catch (const Standard_Failure&) {
        result.faces.clear();
        result.error =
            "failed to validate analytic planar boolean output";
        return finish();
    } catch (...) {
        result.faces.clear();
        result.error =
            "failed to validate analytic planar boolean output";
        return finish();
    }
    result.outputFaceCount =
        static_cast<int>(result.faces.size());
    result.overlapArea = std::max(
        0.0, result.inputArea - result.unionArea);
    result.area = compound;
    return finish();
}

} // namespace oap::toolpath
